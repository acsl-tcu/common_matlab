function [c,ceq,GC,GCeq]=TRAJECTORY_C7_NLP_ANALYTIC(z,baseU,baseL,leaf_maps,shapes,inflation,duration_s,limits,prepared)
%TRAJECTORY_C7_NLP_ANALYTIC Full nonlinear NLP constraints + exact derivatives.
% c<=0, ceq==0, GC=dc/dz and GCeq=dceq/dz in FMINCON COLUMN layout.
% All derivatives are analytic at smooth points. At shape corners, zero
% derivative norms and tied support vertices, use a valid subgradient.
% This does NOT alter any geometric certificate or authorize HLC commit.
% Inputs are prepared by TRAJECTORY_SIMULTANEOUS_C7_SEARCH, not raw sensors.
assert(isnumeric(z)&&isreal(z)&&isvector(z)&&all(isfinite(z(:))), ...
    'TRAJECTORY_C7_NLP_ANALYTIC:Z','Finite optimization vector required.');
assert(isequal(size(baseU),[3 24])&&isequal(size(baseL),[3 24])&& ...
    iscell(leaf_maps)&&~isempty(leaf_maps)&&numel(inflation)==3&& ...
    isfinite(duration_s)&&duration_s>0, ...
    'TRAJECTORY_C7_NLP_ANALYTIC:Input','Prepared controls, leaves and time required.');
ns=numel(shapes);nl=numel(leaf_maps);nh=3*ns*nl;
% The online solver reuses fixed de Casteljau / Bernstein derivative maps
% for the ENTIRE solve. The 8-argument API remains valid for audits.
if nargin<9 || isempty(prepared)
    prepared=TRAJECTORY_C7_NLP_STATIC_PREPARE(leaf_maps,duration_s);
else
    assert(isstruct(prepared)&&isfield(prepared,'duration_s')&& ...
        isequal(prepared.duration_s,duration_s)&& ...
        isfield(prepared,'leaf')&&numel(prepared.leaf)==nl, ...
        'TRAJECTORY_C7_NLP_ANALYTIC:Cache', ...
        'Cached matrices must match this solve and its duration.');
end
nvar=48+3*nh;
z=double(z(:));
assert(numel(z)==nvar, ...
    'TRAJECTORY_C7_NLP_ANALYTIC:Z','Optimization vector size mismatch.');
U=double(baseU);L=double(baseL);
U(:,9:16)=reshape(z(1:24),3,8);
L(:,9:16)=reshape(z(25:48),3,8);
N=reshape(z(49:end),3,[]);
haslimits=~isempty(limits);
ngeo=96*ns*nl;
nkin=246*nl*double(haslimits);
c=zeros(ngeo+nkin,1);
ceq=zeros(nh,1);
need_jac=nargout>=3;
if need_jac
    GC=zeros(nvar,numel(c));GCeq=zeros(nvar,nh);
else
    GC=[];GCeq=[];
end
idx=0;
for k=1:nl
    M=prepared.leaf(k).map;
    uk=U*M;lk=L*M;
    F=prepared.leaf(k).free_map;
    for j=1:ns
        for b=1:3
            h=3*((k-1)*ns+j-1)+b;
            n=N(:,h);mag=norm(n);
            if need_jac
                [hs,p]=TRAJECTORY_OBSTACLE_SUPPORT.value_and_point(shapes(j),n);
            else
                hs=TRAJECTORY_OBSTACLE_SUPPORT.value(shapes(j),n);
            end
            switch b
                case 1
                    P=uk;block=1;
                case 2
                    P=lk;block=2;
                otherwise
                    P=[uk lk];block=3;
            end
            m=size(P,2);
            rng=idx+(1:m);
            c(rng)=hs+inflation(b)*mag-n.'*P;
            if need_jac
                % Support gradient at differentiable n is its maximizer;
                % at ties, a maximizer is a valid convex subgradient.
                dn=p;
                if mag>0
                    dn=dn+inflation(b)*n/mag;
                end
                nhrows=48+3*(h-1)+(1:3);
                GC(nhrows,rng)=dn-P;
                if b==1 || b==3
                    for a=1:8
                        q=3*(a-1)+(1:3);
                        GC(q,idx+(1:24))=-n*F(a,:);
                    end
                end
                if b==2 || b==3
                    offset=double(b==3)*24;
                    for a=1:8
                        q=24+3*(a-1)+(1:3);
                        GC(q,idx+offset+(1:24))=-n*F(a,:);
                    end
                end
                GCeq(nhrows,h)=2*n;
            end
            ceq(h)=n.'*n-1;
            idx=idx+m;
        end
    end
end
assert(idx==ngeo, ...
    'TRAJECTORY_C7_NLP_ANALYTIC:GeometryRows','Geometry constraint size mismatch.');
if haslimits
    assert(isstruct(limits)&&all(isfield(limits,{'uav','payload'})), ...
        'TRAJECTORY_C7_NLP_ANALYTIC:Limits','Live reference limits missing.');
    dt=duration_s/nl;
    for k=1:nl
        M=prepared.leaf(k).map;
        for b=1:2
            if b==1
                D=U*M;maxima=double(limits.uav.derivative_norm_max(:));base=0;
            else
                D=L*M;maxima=double(limits.payload.derivative_norm_max(:));base=24;
            end
            for order=1:6
                scale=(24-order)/dt;
                D=scale*diff(D,1,2);
                B=prepared.leaf(k).derivative_maps{order};
                val=sqrt(sum(D.^2,1));
                m=size(D,2);rng=idx+(1:m);
                c(rng)=val(:)-maxima(order);
                if need_jac
                    for i=1:m
                        if val(i)>0
                            direction=D(:,i)/val(i);
                        else
                            % Zero is a valid subgradient of ||D|| at 0.
                            direction=zeros(3,1);
                        end
                        for a=1:8
                            q=base+3*(a-1)+(1:3);
                            GC(q,idx+i)=direction*B(a+8,i);
                        end
                    end
                end
                idx=idx+m;
            end
        end
    end
end
assert(idx==numel(c), ...
    'TRAJECTORY_C7_NLP_ANALYTIC:Rows','Kinematic constraint size mismatch.');
end

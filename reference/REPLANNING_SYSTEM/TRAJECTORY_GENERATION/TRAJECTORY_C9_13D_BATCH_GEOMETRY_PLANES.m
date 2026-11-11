function B=TRAJECTORY_C9_13D_BATCH_GEOMETRY_PLANES(F,shapes,phases,rad,L,delta,ground_active,do_outer,shape_outer)
% FUSED GEOMETRY-PLANE ASSEMBLY, one call per candidate QP.
% Identical real-arithmetic search inequalities to the ordinary
% center-derived 3-body local planes; no extra restrictions, directions,
% stages, collision exemptions or candidate tests. Final FULL-TIME proof
% still required, independent of this QP.  Does NOT apply to witness-guided
% or coherent-plane modes, whose original code stays available.
%
% Geometry rows are ordered [body UAV/cable/payload, shape, phase], with
% ground as the final shape in every phase, matching legacy MATLAB loops.
% All physical parameters and shapes arrive from this Flight invocation.
N=numel(phases);S=numel(shapes);K=S+double(ground_active);
B=struct('status','UNKNOWN_NOT_COMPUTED','A',zeros(13,0),'b',zeros(0,1), ...
    'normal',zeros(3,0),'threshold',zeros(0,1),'lambda',zeros(0,1), ...
    'phase',zeros(0,1),'rhs',zeros(0,1),'obstacle_count',0, ...
    'ground_count',0,'outer_groups_tested',0,'outer_groups_skipped',0, ...
    'elapsed_ms',NaN,'all_geometry_rows',0);
clk=tic;
assert(isequal(size(F.payload_m),[3 N])&& ...
    isequal(size(F.cable_direction),[3 N])&& ...
    isequal(size(F.payload_J_fixed_phase),[3 13 N])&& ...
    isequal(size(F.cable_q_J_fixed_phase),[3 13 N])&& ...
    numel(rad)==3&&L>0&&all(isfinite([rad(:);L;delta;phases(:)])), ...
    'C9_BATCH_PLANES:Input','Valid simultaneous 13D tangent required.');
G=zeros(13,3,K,N);Bvals=zeros(3,K,N);normals=zeros(3,3,K,N);
thresholds=zeros(3,K,N);lambdas=zeros(3,K,N);
ph=repmat(reshape(double(phases),1,1,1,N),[3,K,1,1]);
keep=true(3,K,N);
p=double(F.payload_m);q=double(F.cable_direction);
JP=double(F.payload_J_fixed_phase);JQ=double(F.cable_q_J_fixed_phase);
for j=1:S
    O=shapes(j);os=double(O.center(:));
    rel=p-os;lam_c=max(0,min(1,sum(rel.*q,1)/L));
    if do_outer
        B.outer_groups_tested=B.outer_groups_tested+N;
        d=sqrt(sum(rel.*rel,1));
        threshold=L+max(rad)+delta+shape_outer(j);
        mag=max([ones(1,N);abs(p);abs(os)*ones(1,N); ...
            repmat([L;max(rad);delta;shape_outer(j)],1,N)],[],1);
        far=isfinite(d)&(d>threshold+64*eps(mag));
        B.outer_groups_skipped=B.outer_groups_skipped+nnz(far);
        keep(:,j,:)=repmat(reshape(~far,1,1,N),3,1,1);
    end
    for body=1:3
        if body==1
            lam=ones(1,N);
        elseif body==2
            lam=lam_c;
        else
            lam=zeros(1,N);
        end
        c=p-L*(q.*lam);
        nv=c-os;len=sqrt(sum(nv.*nv,1));
        % Legacy treats an undefined plane direction as UNKNOWN.  Do not
        % hide zero normals behind the outer broadphase or a numeric floor.
                active=reshape(keep(body,j,:),1,N);
        if any(len(active)<=0|~isfinite(len(active)))
            B.status='UNKNOWN_13D_PLANE_DIRECTION_UNDEFINED';return
        end
        % Far groups have no plane; assign a unit basis solely as a
        % finite internal placeholder, never emit their rows.
        nv(:,~active)=repmat([1;0;0],1,nnz(~active));
        len(~active)=1;
        n=nv./len;
        h=TRAJECTORY_C9_SUPPORT_BATCH(O,n);
        jac=JP-L*(JQ.*reshape(lam,1,1,N));
        grad=reshape(sum(jac.*reshape(n,[3,1,N]),1),13,N);
        rhs=h+rad(body)+delta-sum(n.*c,1);
        G(:,body,j,:)=reshape(grad,13,1,1,N);
        Bvals(body,j,:)=reshape(rhs,1,1,N);
        normals(:,body,j,:)=reshape(n,3,1,1,N);
        thresholds(body,j,:)=reshape(h+rad(body)+delta,1,1,N);
        lambdas(body,j,:)=reshape(lam,1,1,N);
    end
end
if ground_active
    j=K;lam_c=double(q(3,:)>0);
    for body=1:3
        if body==1,lam=ones(1,N);
        elseif body==2,lam=lam_c;
        else,lam=zeros(1,N);end
        z=p(3,:)-L*lam.*q(3,:);
        grad=reshape(JP(3,:,:)-L*reshape(lam,1,1,N).*JQ(3,:,:),13,N);
        G(:,body,j,:)=reshape(grad,13,1,1,N);
        Bvals(body,j,:)=reshape(rad(body)+delta-z,1,1,N);
        normals(:,body,j,:)=repmat(reshape([0;0;1],3,1,1,1),[1,1,1,N]);
        thresholds(body,j,:)=rad(body)+delta;
        lambdas(body,j,:)=reshape(lam,1,1,N);
    end
end
all_keep=reshape(keep,1,[]);all_grad=reshape(G,13,[]);
all_rhs=reshape(Bvals,1,[]);all_n=reshape(normals,3,[]);
all_h=reshape(thresholds,1,[]);all_lam=reshape(lambdas,1,[]);
all_ph=reshape(ph,1,[]);
B.all_geometry_rows=nnz(all_keep);
if any(~isfinite(reshape(all_grad(:,all_keep),[],1))) || ...
        any(~isfinite(all_rhs(all_keep)))
    B.status='UNKNOWN_13D_BATCH_NONFINITE_PLANE';return
end
nz=any(all_grad~=0,1);
if any(all_keep&~nz&(all_rhs>0))
    B.status='UNKNOWN_13D_BATCH_UNMOVABLE_SEPARATION';return
end
emitted=all_keep&nz;
B.A=all_grad(:,emitted);
B.b=all_rhs(emitted).';
% The legacy plane-merit evaluator includes all defined directions, even
% zero-gradient redundant ones; preserve this fact and ordering.
B.normal=all_n(:,all_keep);
B.threshold=all_h(all_keep).';
B.lambda=all_lam(all_keep).';
B.phase=all_ph(all_keep).';
B.rhs=all_rhs(all_keep).';
kinds=reshape(repmat(reshape(1:K,1,K,1),[3,1,N]),1,[]);
B.obstacle_count=nnz(emitted&(kinds<=S));
B.ground_count=nnz(emitted&(kinds==K)&logical(ground_active));
B.status='LOCAL_BATCH_GEOMETRY_PLANES_READY__NOT_CERTIFIED';
B.elapsed_ms=1000*toc(clk);
end

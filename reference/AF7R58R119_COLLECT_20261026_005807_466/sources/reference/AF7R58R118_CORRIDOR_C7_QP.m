function R=AF7R58R118_CORRIDOR_C7_QP(times,waypoints,startJets,endJets,cells,live,solver)
% Solve a single multi-segment position-C7 Bezier corridor-constrained QP.
% All knot derivatives 0..7 are shared decision variables, including the
% intermediate WAYPOINT POSITIONS. The chosen waypoints are only a branch
% seed for constructing the upstream safety corridor, NOT equality knots.
% Nominal p^(8) squared energy is minimized subject to EVERY segment's
% EVERY payload Bezier control point lying in the strict safe halfspaces.
% UAV and whole cable STILL require independent continuous R80 proof.
% Entire return connector is included in this ONE optimization.
% Numeric floating-point check, NOT an outward-rounded IEEE certificate.
assert(isrow(times)&&numel(times)>=2&&all(isfinite(times))&& ...
    all(diff(times)>0),'R118:OrderedTimeKnots');
K=numel(times);S=K-1;nv=8*max(0,K-2);n=3*nv;
assert(isequal(size(waypoints),[3 K])&&all(isfinite(waypoints(:)))&& ...
    isequal(size(startJets),[3 8])&&isequal(size(endJets),[3 8])&& ...
    all(isfinite(startJets(:)))&&all(isfinite(endJets(:))), ...
    'R118:InvalidBoundaryJets');
assert(isequal(waypoints(:,1),startJets(:,1)) && ...
    isequal(waypoints(:,end),endJets(:,1)),'R118:BoundaryPositionMismatch');
assert(iscell(cells)&&numel(cells)==S,'R118:CorridorCellCount');
assert(isstruct(live)&&isfield(live,'rL_m')&&isfinite(live.rL_m)&& ...
    live.rL_m>=0,'R118:ExternalPayloadRadiusRequired');
assert(isstruct(solver)&&isfield(solver,'max_iterations')&& ...
    isfinite(solver.max_iterations)&&solver.max_iterations>=1&& ...
    fix(solver.max_iterations)==solver.max_iterations,'R118:SolverWorkRequired');
% Avoid dimensional conditioning by solving for dimensionless-time jets:
% y_{kd}=Tscale^d*p^{(d)}(t_k). No arbitrary physical bound is introduced.
tScale=median(diff(times));
B=zeros(16*S,nv);base=zeros(3,16*S);scaleVec=zeros(1,nv);
for k=2:K-1
    for d=0:7
        scaleVec((k-2)*8+d+1)=tScale^d;
    end
end
Jbase=zeros(3,8,K);
Jbase(:,1,:)=reshape(waypoints,[3 1 K]);
Jbase(:,:,1)=startJets;Jbase(:,:,K)=endJets;
for s=1:S
    T=times(s+1)-times(s);ix=(s-1)*16+(1:16);
    base(:,ix)=local_hermite(Jbase(:,:,s),Jbase(:,:,s+1),T);
    for k=max(2,s):min(K-1,s+1)
        for d=0:7
            idx=(k-2)*8+d+1;E=zeros(1,8);E(d+1)=1/scaleVec(idx);
            if k==s,P=local_hermite(E,zeros(1,8),T);
            else,P=local_hermite(zeros(1,8),E,T);end
            B(ix,idx)=P(:);
        end
    end
end
% Positive Bernstein Gram for degree-7 polynomial p^(8).
G=zeros(8);
for i=0:7
    for j=0:7
        G(i+1,j+1)=nchoosek(7,i)*nchoosek(7,j)/ ...
            (15*nchoosek(14,i+j));
    end
end
WG=chol(G,'upper');F=zeros(8*S,nv);bb=zeros(8*S,3);
for s=1:S
    T=times(s+1)-times(s);ix=(s-1)*16+(1:16);jj=(s-1)*8+(1:8);
    bb(jj,:)=sqrt(T)*WG*deriv8(base(:,ix),T).';
    for k=1:nv
        F(jj,k)=sqrt(T)*WG*deriv8(B(ix,k).',T).';
    end
end
% QR-whiten the eighth-derivative map instead of forming F'*F.
% This prevents squared-condition-number loss inherent in Hilbert-like
% eighth-derivative Gram matrices for degree-15 Bernstein polynomials.
if nv>0
    [Qorth,Rtri]=qr(F,0);
    sing=svd(F,0);
    if numel(sing)<nv || sing(end)<= ...
            4096*eps(sing(1))*size(F,1)
        R=failed('INCONCLUSIVE_MIN8_RANK',times);return;
    end
    invR=Rtri\eye(nv);
    H=eye(n);
    f=reshape(Qorth.'*bb,[],1);
else
    invR=zeros(0);
    H=zeros(0);f=zeros(0,1);
end
A=zeros(0,n);rhs=zeros(0,1);
for s=1:S
    C=cells{s};
    assert(isstruct(C)&&isfield(C,'A')&&isfield(C,'b'), ...
        'R118:MalformedPrebuiltCorridor');
    planes=double(C.A);bound=double(C.b(:));
    assert(size(planes,2)==3&&size(planes,1)==numel(bound)&& ...
        all(isfinite(planes(:)))&&all(isfinite(bound)), ...
        'R118:InvalidCorridorHalfspaces');
    ix=(s-1)*16+(1:16);
    for j=1:size(planes,1)
        a=planes(j,:);na=norm(a);
        assert(isfinite(na)&&na>0,'R118:NonzeroNormalRequired');
        a=a/na;b=bound(j)/na-live.rL_m;
        for c=1:16
            row=zeros(1,n);
            for z=1:3
                row((z-1)*nv+(1:nv))=a(z)*B(ix(c),:);
            end
            guard=512*eps(max([1;abs(a(:));abs(b);abs(base(:,ix(c)))]));
            bRow=b-a*base(:,ix(c))-guard;
            % A fixed endpoint control point has no solver DOFs.
            % Accept it only when its corridor inequality is strict.
            if ~any(row)
                if bRow<0
                    R=failed('INCONCLUSIVE_FIXED_ENDPOINT_OUTSIDE_CORRIDOR',times);
                    return;
                end
            else
                A(end+1,:)=row; %#ok<AGROW>
                rhs(end+1,1)=bRow; %#ok<AGROW>
            end
        end
    end
end
if n==0
    y=zeros(0,1);
    info=struct('primal_feasible',true,'iterations',0, ...
         'status','FIXED_ENDPOINTS','max_violation',0,'kkt_residual',0);
else
    % Solve in whitened coordinates z=diag(Rtri,Rtri,Rtri)*y.
    % This is the SAME minimum-p^(8) objective; no extra tuning weight.
    invFull=kron(eye(3),invR);
    [z,info]=hildreth_qp(H,f,A*invFull,rhs, ...
         round(solver.max_iterations));
    if ~info.primal_feasible
        R=failed('INCONCLUSIVE_CONSTRAINED_QP_PRIMAL',times);
        R.solver=info;return;
    end
    y=invFull*z;
end
J=Jbase;
for k=2:K-1
    for d=0:7
        idx=(k-2)*8+d+1;
        for z=1:3
            J(z,d+1,k)=J(z,d+1,k)+y((z-1)*nv+idx)/scaleVec(idx);
        end
    end
end
seg=repmat(struct('t0',0,'t1',0,'duration_s',0, ...
 'payload_cp',zeros(3,16),'trajectory_frame','HLC_PAYLOAD_WORLD'),1,S);
for s=1:S
    T=times(s+1)-times(s);
    seg(s)=struct('t0',times(s),'t1',times(s+1),'duration_s',T, ...
        'payload_cp',local_hermite(J(:,:,s),J(:,:,s+1),T), ...
        'trajectory_frame','HLC_PAYLOAD_WORLD');
end
% Independent primal, endpoint, and C7-knot checks (not just QP exit flag).
minimumMargin=Inf;
for s=1:S
    C=cells{s};P=seg(s).payload_cp;
    for j=1:size(C.A,1)
        nrm=norm(C.A(j,:));
        vv=(C.b(j)-C.A(j,:)*P)/nrm-live.rL_m;
        minimumMargin=min(minimumMargin,min(vv));
    end
end
maxKnotMismatch=0;
maxNormalizedKnotError=0;
cpGlobalScale=max([1;abs(J(:))]);
for s=1:S-1
    D1=seg(s).payload_cp;D2=seg(s+1).payload_cp;
    for d=0:7
        mismatch=max(abs(D1(:,end)-D2(:,1)));
        maxKnotMismatch=max(maxKnotMismatch,mismatch);
        % Repeated finite differences amplify roundoff by roughly
        % (degree/duration)^d. The mathematical C7 identity is the ONE
        % shared J(:,:,k); this numerical check detects gross defects.
        Tmin=min(seg(s).duration_s,seg(s+1).duration_s);
        amp=(15/Tmin)^d;
        tol=4096*eps(cpGlobalScale)*max(1,amp);
        maxNormalizedKnotError=max(maxNormalizedKnotError,mismatch/tol);
        if d<7
            D1=(size(D1,2)-1)/seg(s).duration_s*diff(D1,1,2);
            D2=(size(D2,2)-1)/seg(s+1).duration_s*diff(D2,1,2);
        end
    end
end
cpAll=[seg.payload_cp];if ~(minimumMargin>512*eps(max([1;abs(waypoints(:))])) && ...
       maxNormalizedKnotError<=1)
    R=failed('INCONCLUSIVE_C7_KNOT_OR_CORRIDOR',times);
    R.min_payload_corridor_margin_m=minimumMargin;
    R.max_c7_knot_residual=maxKnotMismatch;
    R.max_c7_normalized_error=maxNormalizedKnotError;return;
end
R=struct('status','CONSTRAINED_C7_PAYLOAD_CORRIDOR_QP__REFERENCE_ONLY', ...
    'reference_candidate_found',true,'segments',seg, ...
    'times_s',times,'num_segments',S,'knot_jets',J, ...
    'waypoints',reshape(J(:,1,:),3,K), ...
    'min_payload_corridor_margin_m',minimumMargin, ...
    'max_c7_knot_residual',maxKnotMismatch, ...
    'max_c7_normalized_error',maxNormalizedKnotError, ...
    'continuity_order',7,'solver',info, ...
    'entire_connector_jointly_solved',true, ...
    'uav_cable_continuous_certified',false, ...
    'yaw_c6_certified',false,'execution_certified',false, ...
    'production_commit_authorized',false);
end
function [y,info]=hildreth_qp(H,f,A,b,maxIter)
n=numel(f);m=numel(b);y=zeros(n,1);iter=0;
if rcond(H)<1e-13 || any(~isfinite([H(:);f(:)]))
    info=struct('primal_feasible',false,'iterations',0, ...
        'status','ILL_CONDITIONED_HESSIAN','max_violation',Inf, ...
        'kkt_residual',Inf);return;
end
x0=-(H\f);
if m==0
    y=x0;info=struct('primal_feasible',true,'iterations',0, ...
        'status','UNCONSTRAINED_MIN8','max_violation',0,'kkt_residual',0);return;
end
W=H\A.';G=A*W;lambda=zeros(m,1);
if any(diag(G)<=0)||any(~isfinite(G(:)))
    info=struct('primal_feasible',false,'iterations',0, ...
        'status','DEGENERATE_CORRIDOR_ROWS','max_violation',Inf, ...
        'kkt_residual',Inf);return;
end
% The residual is updated exactly for each dual coordinate.
res=A*x0-b;tol=256*eps(max([1;abs(b);abs(A*x0)]));
for iter=1:maxIter
    for i=1:m
        new=max(0,lambda(i)+res(i)/G(i,i));
        delta=new-lambda(i);
        if delta~=0
            lambda(i)=new;
            res=res-delta*G(:,i);
        end
    end
    if max(res)<=tol && max(abs(lambda.*res))<=max(1,norm(lambda,Inf))*tol
        break;
    end
end
y=x0-W*lambda;
violation=max(A*y-b);station=norm(H*y+f+A.'*lambda,Inf);
primal=isfinite(violation)&&violation<=tol && ...
    isfinite(station)&&station<=1e-8*max([1;abs(f(:));abs(H*y)]);
info=struct('primal_feasible',primal,'iterations',iter, ...
    'status','NUMERIC_QP_KKT_CHECKED','max_violation',violation, ...
    'kkt_residual',station, 'active_count',nnz(lambda>0));
end
function P=local_hermite(A,B,T)
P=zeros(size(A,1),16);
for j=0:7
    u=zeros(size(A,1),1);v=u;
    for k=0:j
        falling=prod(15:-1:(16-k));
        z=nchoosek(j,k)*T^k/falling;
        u=u+z*A(:,k+1);
        v=v+(-1)^k*z*B(:,k+1);
    end
    P(:,j+1)=u;P(:,16-j)=v;
end
end
function P=deriv8(P,T)
for j=1:8
    P=(size(P,2)-1)/T*diff(P,1,2);
end
end
function R=failed(status,times)
R=struct('status',status,'reference_candidate_found',false, ...
    'times_s',times,'production_commit_authorized',false, ...
    'segments',struct([]));
end

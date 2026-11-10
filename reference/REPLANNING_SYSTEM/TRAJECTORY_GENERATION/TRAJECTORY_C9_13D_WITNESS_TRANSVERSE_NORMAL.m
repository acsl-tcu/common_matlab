function W=TRAJECTORY_C9_13D_WITNESS_TRANSVERSE_NORMAL(piece,phase,L,shape,body_center,body_lambda,g,radius,guard,iteration)
% Derive a LOCAL SEARCH plane normal from a failed real-time certificate.
% The construction uses actual C9 kinematics, observed shape support and
% current gravity. This is NOT a collision certificate or a fixed bypass.
% Symmetric sign alternatives are enumerated by the outer search; the
% independent full-time certificate alone can accept a C9 trajectory.
% body_lambda=0 payload, 1 UAV, closest fraction for a cable witness.
W=struct('active',false,'normal',zeros(3,1),'deficit_m',NaN, ...
    'candidate_count',0,'support_calls',0,'status','NO_TRANSVERSE_DIRECTION');
T=double(piece.duration_s);P=double(piece.control_points_m);
if ~(isequal(size(P),[3 24])&&isfinite(T)&&T>0&& ...
        all(isfinite(P(:)))&&all(isfinite(body_center(:)))&& ...
        numel(body_center)==3&&numel(g)==3&&all(isfinite(g(:)))&& ...
        isfinite(body_lambda)&&body_lambda>=0&&body_lambda<=1)
    W.status='INVALID_GEOMETRIC_INPUT';return
end
phase=double(phase);
if ~(isscalar(phase)&&isfinite(phase)&&phase>=0&&phase<=1)
    W.status='INVALID_WITNESS_PHASE';return
end
L=double(L);
if ~(isscalar(L)&&isfinite(L)&&L>0)
    W.status='INVALID_LIVE_CABLE_LENGTH';return
end
b22=bernstein_basis(22,phase);
b21=bernstein_basis(21,phase);
b20=bernstein_basis(20,phase);
vL=(23/T)*diff(P,1,2)*b22;
aL=(23*22/T^2)*diff(P,2,2)*b21;
jL=(23*22*21/T^3)*diff(P,3,2)*b20;
w=double(g(:))-aL;s=norm(w);
if ~(isfinite(s)&&s>0),W.status='DEGENERATE_TAUT_DIRECTION';return;end
q=w/s;
qdot=-(eye(3)-q*q.')*jL/s;
vbody=vL-body_lambda*L*qdot;
sv=norm(vbody);
if ~(isfinite(sv)&&sv>0),W.status='ZERO_BODY_VELOCITY';return;end
t=vbody/sv;
d=double(body_center(:))-double(shape.center(:));
g=double(g(:));
dt=d-t*(t.'*d);
V=[dt,cross(t,dt),cross(t,g)];
% The numerical threshold only distinguishes representable zero vectors;
% it is not a flight speed floor or an empirical safety margin.
scale=max([1;abs(d(:));abs(g(:));abs(vbody(:))]);
threshold=64*eps(scale);
best=Inf;nBest=[];candidateNumber=0;
for k=1:size(V,2)
    magnitude=norm(V(:,k));
    if ~(isfinite(magnitude)&&magnitude>threshold),continue;end
    base=V(:,k)/magnitude;
    for sigma=[1 -1]
        n=sigma*base;
        h=TRAJECTORY_OBSTACLE_SUPPORT.value(shape,n);
        W.support_calls=W.support_calls+1;
        deficit=h+double(radius)+double(guard)-n.'*body_center(:);
        if ~(isfinite(deficit)&&deficit>0),continue;end
        candidateNumber=candidateNumber+1;
        if isempty(nBest)
            scoreTolerance=0;
        else
            scoreTolerance=64*eps(max([1;abs(deficit);abs(best)]));
        end
        if isempty(nBest)||deficit<best-scoreTolerance
            nBest=n;best=deficit;
        elseif abs(deficit-best)<=scoreTolerance && mod(iteration,2)==1
            % Exact/roundoff-level geometrical ties have TWO valid sides.
            % Alternate them across existing same-QP outer iterations.
            nBest=n;
        end
    end
end
W.candidate_count=candidateNumber;
if isempty(nBest)
    W.status='NO_VIOLATED_DATA_DERIVED_TRANSVERSE_PLANE';return
end
W.active=true;W.normal=nBest;W.deficit_m=best;
W.status='SAME_13D_QP_TOPOLOGICAL_GUIDE__FULL_TIME_PROOF_REQUIRED';
end

function b=bernstein_basis(n,u)
b=zeros(n+1,1);
if u==0,b(1)=1;return;end
if u==1,b(end)=1;return;end
if u<=0.5
    b(1)=(1-u)^n;
    for k=1:n,b(k+1)=b(k)*(n-k+1)*u/(k*(1-u));end
else
    b(end)=u^n;
    for k=n:-1:1,b(k)=b(k+1)*k*(1-u)/((n-k+1)*u);end
end
end

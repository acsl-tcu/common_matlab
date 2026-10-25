function Q=AF7R58R80_CONTINUOUS_SEPARATE_BODY_CERT(C,obs,g,L,rQ,rC,rL,max_nodes)
% R80: continuous-time, separately inflated UAV / full straight cable / payload.
% NO bounding ball enclosing the whole suspended-load system.
% For a payload-flat reference pL(t), the taut-cable reference from the
% existing native model is pQ(t)=pL(t)+L*(pL''(t)+g*ez)/norm(pL''(t)+g*ez).
% Each leaf proves norm(specific force)>0 with a Bernstein support plane,
% bounds motion through Bernstein first/third derivatives, then proves
% three independent ellipsoid separating-plane inequalities across ALL t
% and ALL lambda in [0,1] of the taut cable segment. Fail closed.
%
% CERTIFICATE SCOPE: reference-level known static ellipsoids ONLY. Not a
% physical/closed-loop reachability, actuator, cable-tension, or unknown-space
% certificate. Use the caller's live gravity, cable length, three radii and
% work budget. No global enclosing sphere and no sampled acceptance.
assert(isstruct(C)&&isfield(C,'payload_cp')&& ...
    isequal(size(C.payload_cp),[3,16])&& ...
    isnumeric(C.payload_cp)&&all(isfinite(C.payload_cp(:)))&& ...
    isfield(C,'t0')&&isfield(C,'t1')&& ...
    isfinite(C.t0)&&isfinite(C.t1)&&C.t1>C.t0&& ...
    isfield(C,'trajectory_frame')&& ...
    strcmp(char(C.trajectory_frame),'HLC_PAYLOAD_WORLD'), ...
    '[R80] finite 3x16 HLC-payload-frame Bezier connector required.');
A=[g,L,rQ,rC,rL];
assert(isnumeric(A)&&numel(A)==5&&all(isfinite(A))&& ...
    g>0&&L>0&&all(A(3:5)>=0), ...
    '[R80] live g,L,rQ,rC,rL required; no guessed fallback.');
assert(isnumeric(max_nodes)&&isscalar(max_nodes)&&isfinite(max_nodes)&& ...
    max_nodes==fix(max_nodes)&&max_nodes>=1, ...
    '[R80] finite positive caller-authorized numerical work budget required.');
Q=struct('certified',false,'continuous_time',false, ...
    'status','INCONCLUSIVE_NOT_RUN', 'geometry','SEPARATE_UAV_CABLE_PAYLOAD', ...
    'uses_one_whole_body_ball',false, 'known_obstacle_count',numel(obs), ...
    'nodes',0,'leaves',0,'min_margin_m',NaN, ...
    'specific_force_lower_m_s2',NaN, ...
    'reference_only',true,'input_constraints_certified',false, ...
    'tracking_and_observer_certified',false, ...
    'unknown_space_certified',false,'production_committed',false);
if isempty(obs)
    Q.status='INCONCLUSIVE_NO_KNOWN_OBSTACLE_COVERAGE';return;
end
assert(isstruct(obs),'[R80] obstacles require a struct array.');
for k=1:numel(obs)
    o=obs(k);
    assert(isfield(o,'center')&&isfield(o,'R')&&isfield(o,'axes')&& ...
        isequal(size(o.center),[3 1])&&isequal(size(o.axes),[3 1])&& ...
        isequal(size(o.R),[3 3])&& ...
        all(isfinite(o.center(:)))&&all(isfinite(o.axes(:)))&& ...
        all(isfinite(o.R(:)))&&all(o.axes>0)&& ...
        norm(o.R'*o.R-eye(3),'fro')<1e-9&&abs(det(o.R)-1)<1e-9, ...
        '[R80] invalid static oriented ellipsoid.');
end
stack=struct('cp',double(C.payload_cp),'a',double(C.t0),'b',double(C.t1));
worst=Inf;sigmin=Inf;nodes=0;leaves=0;reason='';
while ~isempty(stack)
    if nodes>=max_nodes
        reason='INCONCLUSIVE_RESOURCE_BUDGET';break;
    end
    S=stack(end);stack(end)=[];nodes=nodes+1;
    [ok,margin,sigma]=r80_leaf(S.cp,S.b-S.a,obs,g,L,rQ,rC,rL);
    if ok
        leaves=leaves+1;worst=min(worst,margin);sigmin=min(sigmin,sigma);
        continue;
    end
    mid=S.a+0.5*(S.b-S.a);
    if ~(mid>S.a&&mid<S.b)
        reason='INCONCLUSIVE_FLOAT_INTERVAL_RESOLUTION';break;
    end
    [P1,P2]=r80_split(S.cp);
    stack(end+1)=struct('cp',P2,'a',mid,'b',S.b); %#ok<AGROW>
    stack(end+1)=struct('cp',P1,'a',S.a,'b',mid); %#ok<AGROW>
end
Q.nodes=nodes;Q.leaves=leaves;
if isempty(stack)&&isempty(reason)
    Q.certified=true;Q.continuous_time=true;
    Q.status='CERTIFIED_SEPARATE_BODY_REFERENCE_KNOWN_ELLIPSOIDS';
    Q.min_margin_m=worst;Q.specific_force_lower_m_s2=sigmin;
else
    if isempty(reason),reason='INCONCLUSIVE_SEPARATION';end
    Q.status=reason;
end
end

function [ok,worst,sigmaLB]=r80_leaf(P,T,obs,g,L,rQ,rC,rL)
ok=false;worst=-Inf;sigmaLB=NaN;
if ~(isfinite(T)&&T>0),return;end
D1=(15/T)*diff(P,1,2);
D2=(14/T)*diff(D1,1,2);
D3=(13/T)*diff(D2,1,2);
V=max(vecnorm(D1,2,1));J=max(vecnorm(D3,2,1));
F=D2+[zeros(2,size(D2,2));g*ones(1,size(D2,2))];
% This projection lower-bounds ||pL''+g ez|| continuously by convex-hull.
% Unlike g-max||p''||, it succeeds for arbitrary horizontal acceleration.
nf=mean(F,2);nfn=norm(nf);
if ~(isfinite(nfn)&&nfn>0),return;end
nf=nf/nfn;sigmaLB=min(nf'*F);
if ~(isfinite(sigmaLB)&&sigmaLB>0),return;end
VQ=V+L*J/sigmaLB;VC=max(V,VQ);h=T/2;
if ~(isfinite(VQ)&&isfinite(VC)),return;end
pL=r80_decasteljau(P,0.5);
a=r80_decasteljau(D2,0.5);
sf=a+[0;0;g];ns=norm(sf);
if ~(isfinite(ns)&&ns>=sigmaLB),return;end
pQ=pL+(L/ns)*sf;
% For each ellipsoid, three INDEPENDENT normals may certify the UAV,
% payload and entire straight cable. The cable normal is shared between
% all lambda because min(n'pL,n'pQ) bounds its whole convex segment.
worst=Inf;
for k=1:numel(obs)
    o=obs(k);c=double(o.center);R=double(o.R);ax=double(o.axes);
    [mL,passL]=r80_best_margin(pL,pQ,c,R,ax,rL,V*h,'L');
    [mQ,passQ]=r80_best_margin(pL,pQ,c,R,ax,rQ,VQ*h,'Q');
    [mC,passC]=r80_best_margin(pL,pQ,c,R,ax,rC,VC*h,'C');
    worst=min([worst,mL,mQ,mC]);
    if ~(passL&&passQ&&passC),ok=false;return;end
end
ok=true;
end

function [best,pass]=r80_best_margin(pL,pQ,c,R,ax,radius,travel,kind)
% Ellipsoid support is exact for arbitrary SO(3) orientation.
D=pQ-pL;denom=D'*D;
if denom>0
    tau=max(0,min(1,((c-pL)'*D)/denom));
else
    tau=0;
end
pNear=pL+tau*D;
centers=[pNear-c,pL-c,pQ-c,(pL+pQ)/2-c];
N=centers;
for i=1:size(centers,2)
    z=centers(:,i);
    q=R'*z;
    if norm(q)>0
        grad=R*(q./(ax.^2));
        N(:,end+1)=grad; %#ok<AGROW>
    end
end
N=[N,R,-R];
best=-Inf;pass=false;
for j=1:size(N,2)
    n=N(:,j);nn=norm(n);
    if ~(isfinite(nn)&&nn>0),continue;end
    n=n/nn;
    ell=sqrt(sum((ax.*(R'*n)).^2));
    switch kind
        case 'L', low=n'*pL;
        case 'Q', low=n'*pQ;
        otherwise, low=min(n'*pL,n'*pQ);
    end
    margin=low - n'*c - ell - radius - travel;
    % Strict floating-point slack: no near-zero point is declared safe.
    roundoff=128*eps(max([1,abs(low),abs(n'*c),ell,radius,travel]));
    margin=margin-roundoff;
    if margin>best,best=margin;end
    if margin>0,pass=true;return;end
end
end

function Q=r80_decasteljau(P,u)
while size(P,2)>1
    P=(1-u)*P(:,1:end-1)+u*P(:,2:end);
end
Q=P(:,1);
end

function [A,B]=r80_split(P)
N=size(P,2);A=zeros(size(P));B=zeros(size(P));
A(:,1)=P(:,1);B(:,end)=P(:,end);
for k=2:N
    P=0.5*(P(:,1:end-1)+P(:,2:end));
    A(:,k)=P(:,1);B(:,N-k+1)=P(:,end);
end
end

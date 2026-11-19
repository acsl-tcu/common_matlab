function data = CBF_REAL_ZOH_DIAG_EVAL(x, u, parts, P, G, order)
% CBF_REAL_ZOH_DIAG_EVAL  Exact time-Taylor coefficients (to roundoff).
% Source algorithm used by generate_CBF_REAL_HOCBF_ZOH.m.
% NOT a validated safety filter. No numerical finite differentiation.
% This source is copied into GENERATED under a distinctive name after
% generator checks; method-specific ZOH assumption lives ONLY here.
%
% x (19x1) [q(4);w(3);pL(3);vL(3);pT(3);wL(3)], u(4x1)=[T;tau].
% All 4 physical inputs are held constant for the Taylor expansion.
% This is independent of the nominal output order in the HLC hierarchy.
% The generator checks lower-order input effects on each geometric barrier.
if nargin<6, order=6; end
validateattributes(x,{'double'},{'vector','numel',19,'finite','real'});
validateattributes(u,{'double'},{'vector','numel',4,'finite','real'});
validateattributes(order,{'double'},{'scalar','integer','>=',1,'<=',6});
validateattributes(P.m,{'double'},{'scalar','positive','finite'});
validateattributes(P.mL,{'double'},{'scalar','nonnegative','finite'});
validateattributes(P.cableL,{'double'},{'scalar','positive','finite'});
validateattributes(P.J,{'double'},{'vector','numel',3,'positive','finite'});
x=x(:);u=u(:);
if abs(norm(x(1:4))-1)>1e-5
    error('CBFRealZOH:Quaternion','Input quaternion must be unit length');
end
if abs(norm(x(14:16))-1)>1e-5
    error('CBFRealZOH:CableDirection','Cable unit direction pT must have norm 1');
end
% No detections is a normal sensor result and must be handled safely.
if isempty(parts), parts=struct([]); end
if isstruct(parts) && isscalar(parts) && isfield(parts,'parts')
    parts=parts.parts;
end
if ~isstruct(parts)
    error('CBFRealZOH:Obstacle','parts must be a struct array (may be empty)');
end
N=order;
X=zeros(19,N+1);
X(:,1)=x;
for k=0:N-1
    X(:,k+2)=physical_rhs_coefficient(X(:,1:k+1),u,P,k)/(k+1);
end
pL=X(8:10,:); pT=X(14:16,:);
pD=pL-P.cableL*pT;
pC=pL-(P.cableL/2)*pT;
a=G.cableAxisShort; b=G.cableAxisLong;
Qc=zeros(3,3,N+1);
for k=0:N
    Qc(:,:,k+1)=(b*b-a*a)*outer_coefficient(pT,pT,k);
end
Qc(:,:,1)=Qc(:,:,1)+a*a*eye(3);

nObs=numel(parts);
ng=double(logical(G.includeGround));
nb=3*nObs+3*ng;
D=zeros(N+1,nb);
names=cell(1,nb);
components=cell(1,nb);
kinds=cell(1,nb);
idx=0;
for j=1:nObs
    [ctr,Qo,label]=parse_obstacle(parts(j),j);
    pose={pD,pC,pL};
    labels={'drone','cable','load'};
    for k=1:3
        idx=idx+1;
        Qs=zeros(3,3,N+1);
        switch k
            case 1, Qs(:,:,1)=G.droneSafetyRadius^2*eye(3);
            case 2, Qs=Qc;
            case 3, Qs(:,:,1)=G.loadSafetyRadius^2*eye(3);
        end
        U= (1+G.obstacleOuterEta)*Qs;
        U(:,:,1)=U(:,:,1)+(1+1/G.obstacleOuterEta)*Qo;
        delta=pose{k};delta(:,1)=delta(:,1)-ctr;
        D(:,idx)=quad_separation_jet(delta,U,N);
        names{idx}=sprintf('%s/%s',label,labels{k});
        components{idx}=labels{k};
        kinds{idx}='obstacle';
    end
end
if G.includeGround
    idx=idx+1;
    D(:,idx)=deriv_from_coefficient(pD(3,:)-[G.groundZ+G.droneSafetyRadius zeros(1,N)]);
    names{idx}='ground/drone';components{idx}='drone';kinds{idx}='ground';
    idx=idx+1;
    zSeries=Qc(3,3,:);zSeries=reshape(zSeries,1,[]);
    radiusSeries=positive_sqrt_series(zSeries,N);
    D(:,idx)=deriv_from_coefficient(pC(3,:)-radiusSeries-[G.groundZ zeros(1,N)]);
    names{idx}='ground/cable';components{idx}='cable';kinds{idx}='ground';
    idx=idx+1;
    D(:,idx)=deriv_from_coefficient(pL(3,:)-[G.groundZ+G.loadSafetyRadius zeros(1,N)]);
    names{idx}='ground/load';components{idx}='load';kinds{idx}='ground';
end

data=struct('derivatives',D,'names',{names},'components',{components}, ...
    'kinds',{kinds},'stateTaylor',X,'order',N, ...
    'inputZOH',u,'model','generateModel_SupendedLoad physical x19', ...
    'warning','ZOH jets only: intersample and global HOCBF safety unverified');
end

function f=physical_rhs_coefficient(X,u,P,k)
q=X(1:4,:);w=X(5:7,:);vL=X(11:13,:);
pT=X(14:16,:);wL=X(17:19,:);
m=P.m;L=P.cableL;mL=P.mL;J=P.J(:);
T=u(1);
f=zeros(19,1);
% qdot = 1/2 q tensor omega
f(1)=-.5*(conv_at(q(2,:),w(1,:),k)+conv_at(q(3,:),w(2,:),k)+conv_at(q(4,:),w(3,:),k));
f(2)= .5*(conv_at(q(1,:),w(1,:),k)+conv_at(q(3,:),w(3,:),k)-conv_at(q(4,:),w(2,:),k));
f(3)= .5*(conv_at(q(1,:),w(2,:),k)+conv_at(q(4,:),w(1,:),k)-conv_at(q(2,:),w(3,:),k));
f(4)= .5*(conv_at(q(1,:),w(3,:),k)+conv_at(q(2,:),w(2,:),k)-conv_at(q(3,:),w(1,:),k));
Jw=J.*w;
f(5:7)=-cross_at(w,Jw,k)./J;
if k==0, f(5:7)=f(5:7)+u(2:4)./J; end
f(8:10)=vL(:,k+1);
b3=zeros(3,k+1);
for j=0:k,b3(:,j+1)=body_z_axis_coeff(q,j);end
vT=zeros(3,k+1);
for j=0:k,vT(:,j+1)=cross_at(wL,pT,j);end
% aL=-g e3 + (T*dot(pT,b3)-mL*dot(vT,vT))*pT/(m+mL)
s=zeros(1,k+1);
for j=0:k
    s(j+1)=T*dot_at(pT,b3,j)-m*L*dot_at(vT,vT,j);
end
for j=0:k
    f(11:13)=f(11:13)+s(j+1)*pT(:,k-j+1)/(m+mL);
end
if k==0
    f(13)=f(13)-P.gravity;
    if isfield(P,'dstx'),f(11)=f(11)+P.dstx;end
    if isfield(P,'dsty'),f(12)=f(12)+P.dsty;end
end
f(14:16)=vT(:,k+1);
f(17:19)=-(T/(m*L))*cross_at(pT,b3,k);
end

function b=body_z_axis_coeff(q,k)
b=zeros(3,1);
b(1)=2*(conv_at(q(2,:),q(4,:),k)+conv_at(q(1,:),q(3,:),k));
b(2)=2*(conv_at(q(3,:),q(4,:),k)-conv_at(q(1,:),q(2,:),k));
b(3)=conv_at(q(1,:),q(1,:),k)-conv_at(q(2,:),q(2,:),k)...
    -conv_at(q(3,:),q(3,:),k)+conv_at(q(4,:),q(4,:),k);
end

function v=conv_at(a,b,k)
v=0;
for j=0:k
    v=v+a(j+1)*b(k-j+1);
end
end

function v=dot_at(a,b,k)
v=0;
for j=0:k
    v=v+a(:,j+1).'*b(:,k-j+1);
end
end

function v=cross_at(a,b,k)
v=zeros(3,1);
for j=0:k
    v=v+cross(a(:,j+1),b(:,k-j+1));
end
end

function v=outer_coefficient(a,b,k)
v=zeros(3);
for j=0:k
    v=v+a(:,j+1)*b(:,k-j+1).';
end
end

function [center,Q,label]=parse_obstacle(part,index)
if ~isfield(part,'center')
    error('CBFRealZOH:ObstacleCenter','parts(%d) has no center',index);
end
center=part.center(:);
if ~isequal(size(center),[3,1]) || any(~isfinite(center))
    error('CBFRealZOH:ObstacleCenter','Obstacle center must be finite 3x1');
end
if isfield(part,'kind')
    kind=char(part.kind);
else
    kind='ellipsoid';
end
switch kind
    case 'ellipsoid'
        if isfield(part,'params')
            axes=part.params.axes(:);
        elseif isfield(part,'axes')
            axes=part.axes(:);
        else
            error('CBFRealZOH:ObstacleAxes','Ellipsoid axes missing');
        end
        R=part.R;
    case 'sphere'
        if isfield(part,'params'),r=part.params.radius;
        else,r=part.radius;end
        axes=repmat(r,3,1);R=eye(3);
    otherwise
        error('CBFRealZOH:ObstacleKind', ...
            'Expected single ellipsoid or sphere, got %s',kind);
end
if ~isequal(size(R),[3,3]) || any(~isfinite(R(:))) || ...
        norm(R.'*R-eye(3),'fro')>1e-5 || ...
        numel(axes)~=3 || any(~isfinite(axes)) || any(axes<=0)
    error('CBFRealZOH:BadObstacle','Invalid obstacle orientation or axes');
end
Q=R*diag(axes.^2)*R.';
if isfield(part,'obstacleName')
    label=char(part.obstacleName);
elseif isfield(part,'name')
    label=char(part.name);
else
    label=sprintf('obstacle%d',index);
end
end

function d=quad_separation_jet(delta,U,N)
% E_system - E_obstacle is contained in E_outer with
% Qouter=(1+eta)Qsystem + (1+1/eta)Qobstacle.
% h=delta'*Qouter^-1*delta-1 >= 0 => ellipsoids are separated.
% This outer approximation is conservative but shape-aware and smooth.
J=zeros(3,3,N+1);
Q0=U(:,:,1);
if rcond(Q0)<1e-12
    error('CBFRealZOH:OuterEllipsoid','Outer ellipsoid matrix is singular');
end
J(:,:,1)=Q0\eye(3);
for k=1:N
    S=zeros(3);
    for i=1:k
        S=S+U(:,:,i+1)*J(:,:,k-i+1);
    end
    J(:,:,k+1)=-(Q0\S);
end
y=zeros(3,N+1);
for k=0:N
    for i=0:k
        y(:,k+1)=y(:,k+1)+J(:,:,i+1)*delta(:,k-i+1);
    end
end
h=zeros(1,N+1);
for k=0:N
    h(k+1)=dot_at(delta,y,k);
end
h(1)=h(1)-1;
d=deriv_from_coefficient(h);
end

function s=positive_sqrt_series(z,N)
if z(1)<=0
    error('CBFRealZOH:CableGeometry','Cable ellipsoid cross-section is invalid');
end
s=zeros(1,N+1);s(1)=sqrt(z(1));
for k=1:N
    v=0;
    for j=1:k-1,v=v+s(j+1)*s(k-j+1);end
    s(k+1)=(z(k+1)-v)/(2*s(1));
end
end

function d=deriv_from_coefficient(z)
N=numel(z)-1;
d=zeros(N+1,1);
for j=0:N,d(j+1)=factorial(j)*z(j+1);end
end
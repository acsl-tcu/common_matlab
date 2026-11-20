function data = CBF_REAL_XY_JET(x,u,P)
% Source for the dedicated 3-D attitude-dependent cable-ellipsoid HOCBF generator. No dynamic extension.
% Fourth-order time-Taylor coefficients of original 19-state physical plant.
% u = [HLC thrust; tauRoll; tauPitch; HLC tauYaw] held for derivatives
% at this instant; only tauRoll and tauPitch are optimization variables.
% This evaluator does NOT change T or tauYaw; no ground barrier is used.
validateattributes(x,{'double'},{'vector','numel',19,'real','finite'});
validateattributes(u,{'double'},{'vector','numel',4,'real','finite'});
validateattributes(P.m,{'numeric'},{'scalar','positive','finite'});
validateattributes(P.mL,{'numeric'},{'scalar','nonnegative','finite'});
validateattributes(P.cableL,{'numeric'},{'scalar','positive','finite'});
validateattributes(P.J,{'numeric'},{'vector','numel',3,'positive','finite'});
x=x(:);u=u(:);
if abs(norm(x(1:4))-1)>1e-5 || abs(norm(x(14:16))-1)>1e-5
    error('CBFXY:UnitState','Quaternion and cable direction must be unit.');
end
N=4; X=zeros(19,N+1);X(:,1)=x;
for k=0:N-1
    X(:,k+2)=physical_rhs_coefficient(X(:,1:k+1),u,P,k)/(k+1);
end
data=struct('stateTaylor',X,'order',N,'fixedThrust',u(1), ...
    'fixedYawTorque',u(4),'physicalInputUsed',u);
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


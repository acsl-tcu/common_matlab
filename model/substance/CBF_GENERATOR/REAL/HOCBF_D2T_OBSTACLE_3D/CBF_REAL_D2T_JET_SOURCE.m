function jet = CBF_REAL_D2T_JET_SOURCE(x,T,dT,ddT,tau,P)
% Twenty-one-state thrust-dynamic-extension time Taylor jet through order 4.
% State y=[x(19);T;dT], controls v=[ddT;tauX;tauY], tauZ fixed.
% Assumes ddT and all torques CONSTANT through the local continuous interval:
% T(s)=T+dT*s+0.5*ddT*s^2. This is NOT the original thrust ZOH plant.
% 19-state physical equations match CBF_COMMON_MODEL (same quaternion order).
validateattributes(x,{'double'},{'vector','numel',19,'finite','real'});
validateattributes(T,{'double'},{'scalar','finite','real'});
validateattributes(dT,{'double'},{'scalar','finite','real'});
validateattributes(ddT,{'double'},{'scalar','finite','real'});
validateattributes(tau,{'double'},{'vector','numel',3,'finite','real'});
validateattributes(P.m,{'numeric'},{'scalar','positive','finite'});
validateattributes(P.mL,{'numeric'},{'scalar','nonnegative','finite'});
validateattributes(P.cableL,{'numeric'},{'scalar','positive','finite'});
validateattributes(P.gravity,{'numeric'},{'scalar','finite'});
validateattributes(P.J,{'numeric'},{'vector','numel',3,'positive','finite'});
x=x(:);tau=tau(:);
if max(abs([norm(x(1:4))-1;norm(x(14:16))-1]))>1e-5
    error('CBFD2T:UnitState','q and cable direction must be unit.');
end
N=4;X=zeros(19,N+1);X(:,1)=x;
Tc=[T,dT,0.5*ddT,0,0]; % ordinary Taylor coefficients, not derivatives
for k=0:N-1
    X(:,k+2)=rhs_taylor(X(:,1:k+1),Tc,tau,P,k)/(k+1);
end
Y=zeros(21,5);Y(1:19,:)=X;Y(20,:)=Tc;Y(21,1:2)=[dT,ddT];
jet=struct('stateTaylor',X,'extendedTaylor',Y, ...
    'thrustTaylor',Tc,'order',N,'extendedStateDimension',21, ...
    'control',[ddT;tau(1:2)],'fixedYawTorque',tau(3), ...
    'thrustIsZOH',false);
end

function f=rhs_taylor(X,Tc,tau,P,k)
q=X(1:4,:);w=X(5:7,:);vL=X(11:13,:);
pT=X(14:16,:);wL=X(17:19,:);
m=P.m;M=P.m+P.mL;L=P.cableL;J=P.J(:);
f=zeros(19,1);
f(1)=-0.5*(conv_at(q(2,:),w(1,:),k)+conv_at(q(3,:),w(2,:),k)+conv_at(q(4,:),w(3,:),k));
f(2)= 0.5*(conv_at(q(1,:),w(1,:),k)+conv_at(q(3,:),w(3,:),k)-conv_at(q(4,:),w(2,:),k));
f(3)= 0.5*(conv_at(q(1,:),w(2,:),k)+conv_at(q(4,:),w(1,:),k)-conv_at(q(2,:),w(3,:),k));
f(4)= 0.5*(conv_at(q(1,:),w(3,:),k)+conv_at(q(2,:),w(2,:),k)-conv_at(q(3,:),w(1,:),k));
Jw=J.*w;
f(5:7)=-cross_at(w,Jw,k)./J;
if k==0,f(5:7)=f(5:7)+tau./J;end
f(8:10)=vL(:,k+1);
b3=zeros(3,k+1);vT=zeros(3,k+1);
for j=0:k
    b3(:,j+1)=body_axis(q,j);
    vT(:,j+1)=cross_at(wL,pT,j);
end
s=zeros(1,k+1);
for j=0:k
    for l=0:j,s(j+1)=s(j+1)+Tc(l+1)*dot_at(pT,b3,j-l);end
    s(j+1)=s(j+1)-m*L*dot_at(vT,vT,j);
end
for j=0:k
    f(11:13)=f(11:13)+s(j+1)*pT(:,k-j+1)/M;
end
if k==0
    f(13)=f(13)-P.gravity;
    if isfield(P,'dstx'),f(11)=f(11)+P.dstx;end
    if isfield(P,'dsty'),f(12)=f(12)+P.dsty;end
end
f(14:16)=vT(:,k+1);
for j=0:k
    f(17:19)=f(17:19)-Tc(j+1)*cross_at(pT,b3,k-j)/(m*L);
end
end

function b=body_axis(q,k)
b=[2*(conv_at(q(2,:),q(4,:),k)+conv_at(q(1,:),q(3,:),k)); ...
   2*(conv_at(q(3,:),q(4,:),k)-conv_at(q(1,:),q(2,:),k)); ...
   conv_at(q(1,:),q(1,:),k)-conv_at(q(2,:),q(2,:),k)- ...
   conv_at(q(3,:),q(3,:),k)+conv_at(q(4,:),q(4,:),k)];
end
function v=conv_at(a,b,k)
v=0;for j=0:k,v=v+a(j+1)*b(k-j+1);end
end
function v=dot_at(a,b,k)
v=0;for j=0:k,v=v+a(:,j+1).'*b(:,k-j+1);end
end
function v=cross_at(a,b,k)
v=zeros(3,1);for j=0:k,v=v+cross(a(:,j+1),b(:,k-j+1));end
end

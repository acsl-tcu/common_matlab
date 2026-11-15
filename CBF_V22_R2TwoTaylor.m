function Z = CBF_V22_R2TwoTaylor(x,u,s,K)
% Full 3D suspended-load dynamics, R2 with ONLY TWO thrust integrators.
% x=[x19;T;Tdot] (21x1), u=[Tddot;tau_x;tau_y;tau_z] (4x1).
% Z(:,k+1)=x^(k)/k! from exact Taylor-jet recurrences.
% TEST ONLY. No replacement of actual plant, generator, or HLC.
x=x(:);u=u(:);
assert(numel(x)==21 && numel(u)==4 && isscalar(K) && K==floor(K) && K>=1 && K<=8, ...
    'V22 requires 21 states, four controls and 1<=K<=8.');
P=cbf_parameter_vector(s);
Z=zeros(21,K+1);Z(:,1)=x;
for k=0:K-1
    D=rhsJet(Z(:,1:k+1),u,P);
    Z(:,k+2)=D(:,k+1)/(k+1);
end
assert(all(isfinite(Z(:))), 'V22 nonfinite Taylor coefficients.');
end

function D=rhsJet(X,u,P)
K=size(X,2)-1;
m=P(1);mL=P(2);L=P(3);g=P(4);J=P(5:7);dist=[P(14);P(15);0];
q=X(1:4,:);w=X(5:7,:);vL=X(11:13,:);
qc=X(14:16,:);wc=X(17:19,:);
C=@(v)cbf_jet('const',v,K);
M=@(a,b)cbf_jet('mul',a,b);
Cr=@(a,b)cbf_jet('cross',a,b);
Dot=@(a,b)cbf_jet('dot',a,b);
qv=q(2:4,:);
dq=[-0.5*Dot(qv,w);0.5*(M(q(1,:),w)+Cr(qv,w))];
b=[2*(M(q(2,:),q(4,:))+M(q(1,:),q(3,:))); ...
   2*(M(q(3,:),q(4,:))-M(q(1,:),q(2,:))); ...
   M(q(1,:),q(1,:))-M(q(2,:),q(2,:))-M(q(3,:),q(3,:))+M(q(4,:),q(4,:))];
T=X(20,:);force=M(T,b);
Dqc=Cr(wc,qc);
Dwc=Cr(-qc,force)/(m*L);
centrifugal=Dot(Dqc,Dqc);
loadScalar=Dot(qc,force)-m*L*centrifugal;
Dvl=C(-g*[0;0;1]+dist)+M(qc,loadScalar)/(m+mL);
Dw=(C(u(2:4))-Cr(w,J.*w))./J;
% The two added states are T and Tdot; the new input is Tddot.
D=[dq;Dw;vL;Dvl;Dqc;Dwc;X(21,:);C(u(1))];
end

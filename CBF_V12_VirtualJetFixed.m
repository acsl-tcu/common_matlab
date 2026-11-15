function [pL,qCable,pDrone] = CBF_V12_VirtualJetFixed(x,nu,P,K)
%CBF_V12_VIRTUALJETFIXED Independent TEST-ONLY corrected flat-output Taylor map.
% The cable direction q(t) depends on the *second* derivative of pL(t).
% Therefore q^(K) requires pL^(K+2). In particular, K=4 needs pL^(6)
% (nu(1:3)), not merely pL^(4). This does NOT replace CBF core.
% x=[pL(0..5),yaw,yawdot] (20x1); nu=[pL(6),yawddot] (4x1).
% Taylor columns contain time derivative divided by factorial(order).
if nargin<4,K=4;end
x=x(:);nu=nu(:);P=P(:);
if numel(x)~=20 || numel(nu)~=4 || K<0 || K>4 || K~=floor(K)
    error('CBF:V12FlatOrder','Require x20, nu4, and Taylor order K=0..4.');
end
fullPL=zeros(3,K+3);
for j=0:min(K+2,5)
    fullPL(:,j+1)=x(3*j+1:3*j+3)/factorial(j);
end
if K+2>=6
    fullPL(:,7)=nu(1:3)/factorial(6);
end
pL=fullPL(:,1:K+1);
a=zeros(3,K+1);
for j=0:K
    a(:,j+1)=(j+2)*(j+1)*fullPL(:,j+3);
end
a(:,1)=a(:,1)+P(4)*[0;0;1]-[P(14);P(15);0];
if norm(a(:,1))<1e-8
    error('CBF:V12FlatSingular','Cable direction cannot be reconstructed: ||pLdd+g-dist|| small.');
end
% Reuse the SAME Taylor jet primitives as the installed CBF implementation.
norm2=cbf_jet('dot',a,a);
invNorm=cbf_jet('inv',cbf_jet('sqrt',norm2));
qCable=-cbf_jet('mul',a,invNorm);
pDrone=pL-P(3)*qCable;
end

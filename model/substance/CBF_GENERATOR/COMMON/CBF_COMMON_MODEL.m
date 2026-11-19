function [dx, detail] = CBF_COMMON_MODEL(x,u,P)
% CBF_COMMON_MODEL  Model consistent with generateModel_SupendedLoad.m.
% Generic instantaneous real-input model. No ZOH, no dT=0 assumption here.
%
% State x (19x1): [q(4); omegaBody(3); pLoad(3); vLoad(3);
%                 pT(3); omegaCable(3)].
% Real input u:   [thrust; torqueX; torqueY; torqueZ].
%   pDrone = pLoad - cableL*pT  (pT points drone -> load).
% Parameters P: m, mL, cableL, gravity, J=[jx;jy;jz].
% This intentionally excludes estimator-side disturbance terms; the REAL
% generator must disclose this assumption before integrating a CBF.
% Optional P.dstx and P.dsty match FLxyDst when available.
%
q=x(1:4); w=x(5:7); pL=x(8:10); vL=x(11:13); %#ok<NASGU>
pT=x(14:16); wL=x(17:19);
T=u(1); tau=u(2:4);
m=P.m; mL=P.mL; L=P.cableL; J=P.J(:);

% Hamilton quaternion, scalar-first, body->world, same convention as the
% RodriguesQuaternion(q) and L'*omega/2 in supplied model generator.
qdot=0.5*[-q(2)*w(1)-q(3)*w(2)-q(4)*w(3);
             q(1)*w(1)+q(3)*w(3)-q(4)*w(2);
             q(1)*w(2)+q(4)*w(1)-q(2)*w(3);
             q(1)*w(3)+q(2)*w(2)-q(3)*w(1)];
b3=[2*(q(2)*q(4)+q(1)*q(3));
    2*(q(3)*q(4)-q(1)*q(2));
    q(1)^2-q(2)^2-q(3)^2+q(4)^2];
pTdot=cross(wL,pT);
wLdot=-cross(pT,T*b3)/(m*L);
aL=[0;0;-P.gravity] + ...
    (dot(pT,T*b3)-m*L*dot(pTdot,pTdot))*pT/(m+mL);
% Optional controller-side horizontal disturbance terms (FLxyDst model).
if isfield(P,'dstx'),aL(1)=aL(1)+P.dstx;end
if isfield(P,'dsty'),aL(2)=aL(2)+P.dsty;end
wdot=(tau-cross(w,J.*w))./J;
dx=[qdot; wdot; vL; aL; pTdot; wLdot];
if nargout>1
    detail=struct('pDrone',x(8:10)-L*pT, 'pLoad',x(8:10), ...
        'pT',pT,'b3',b3,'pTdot',pTdot,'aLoad',aL);
end
end

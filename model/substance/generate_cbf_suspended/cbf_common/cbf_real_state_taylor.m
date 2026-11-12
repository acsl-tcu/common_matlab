function Z = cbf_real_state_taylor(x,nu,P,K)
%CBF_REAL_STATE_TAYLOR  実入力拡張モデルの時間Taylor係数を厳密な級数演算で得る。
% X=[quaternion(4); omegaBody(3);pLoad(3);vLoad(3);qCable(3);
%    omegaCable(3);T;dotT;ddotT;T3;tau(3);dotTau(3)] (29状態)。
% 決定入力nu=[T^(4);tauX^(2);tauY^(2);tauZ^(2)] (4入力)。
% 時間刻み・数値微分は使わない。0..K階の係数を返す。
if numel(x)~=29 || numel(nu)~=4,error('CBF:StateSize','実入力の状態29、決定入力4が必要です。');end
x=x(:);nu=nu(:);
Z=zeros(29,K+1);Z(:,1)=x;
for k=0:K-1
    rhs=localRHS(Z(:,1:k+1),nu,P);
    Z(:,k+2)=rhs(:,k+1)/(k+1);
end
if any(~isfinite(Z(:))),error('CBF:InvalidTaylor','実入力モデルのTaylor係数が非有限です。');end
end

function D=localRHS(X,nu,P)
K=size(X,2)-1;
m=P(1);mL=P(2);L=P(3);g=P(4);J=P(5:7);dist=[P(14);P(15);0];
q=X(1:4,:); w=X(5:7,:); vL=X(11:13,:);
qc=X(14:16,:);wc=X(17:19,:);T=X(20,:);tau=X(24:26,:);
C=@(v)cbf_jet('const',v,K);
M=@(a,b)cbf_jet('mul',a,b);
Cr=@(a,b)cbf_jet('cross',a,b);
Dot=@(a,b)cbf_jet('dot',a,b);
% quaternion derivative (q0 scalar and vector qv)
qv=q(2:4,:);
dq=[-0.5*Dot(qv,w);0.5*(M(q(1,:),w)+Cr(qv,w))];
% 機体のz軸方向 b=R(q)*e3。quaternionの単位ノルムを前提。
b=[2*(M(q(2,:),q(4,:))+M(q(1,:),q(3,:))); ...
   2*(M(q(3,:),q(4,:))-M(q(1,:),q(2,:))); ...
   M(q(1,:),q(1,:))-M(q(2,:),q(2,:))-M(q(3,:),q(3,:))+M(q(4,:),q(4,:))];
force=M(T,b);
Dqc=Cr(wc,qc);
Dwc=Cr(-qc,force)/(m*L);
centrifugal=Dot(Dqc,Dqc);
loadScalar=Dot(qc,force)-m*L*centrifugal;
Dvl=C(-g*[0;0;1]+dist)+M(qc,loadScalar)/(m+mL);
Dw=(tau-Cr(w,J.*w))./J;
D=[dq;Dw;vL;Dvl;Dqc;Dwc; ...
   X(21:23,:);C(nu(1));X(27:29,:);C(nu(2:4))];
end

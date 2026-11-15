function Z=CBF_V11_RealTaylor(mode,x,u,s,K)
%CBF_V11_REALTAYLOR  テスト専用：元の物理モデルで実入力候補を評価。
% R1: 状態19, u=[T;tau_x;tau_y;tau_z]（物理入力）。
% R2: 状態23, u=[T^(4);tau_x;tau_y;tau_z]（推力だけ4段拡張）。
% いずれも元の吊り荷力学・クォータニオン力学をそのまま使う。
% 既存生成器・既存CBF本体の変更は行わない。
mode=upper(char(mode));x=x(:);u=u(:);
if ~any(strcmp(mode,{'R1','R2'})) || numel(u)~=4 || ...
   (strcmp(mode,'R1') && numel(x)~=19) || ...
   (strcmp(mode,'R2') && numel(x)~=23)
    error('CBF:V11Input','R1=19状態、R2=23状態、入力4次元が必要です。');
end
if ~isscalar(K)||K<1||K~=floor(K)||K>8
    error('CBF:V11Order','Taylor次数Kは1～8の整数にしてください。');
end
P=cbf_parameter_vector(s);
Z=zeros(numel(x),K+1);Z(:,1)=x;
for k=0:K-1
    D=localRHS(Z(:,1:k+1),u,P,mode);
    Z(:,k+2)=D(:,k+1)/(k+1);
end
if any(~isfinite(Z(:)))
    error('CBF:V11Nonfinite','Taylor係数が非有限です。');
end
end

function D=localRHS(X,u,P,mode)
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
if strcmp(mode,'R1')
    T=C(u(1));
else
    T=X(20,:);
end
force=M(T,b);
Dqc=Cr(wc,qc);
Dwc=Cr(-qc,force)/(m*L);
centrifugal=Dot(Dqc,Dqc);
loadScalar=Dot(qc,force)-m*L*centrifugal;
Dvl=C(-g*[0;0;1]+dist)+M(qc,loadScalar)/(m+mL);
Dw=(C(u(2:4))-Cr(w,J.*w))./J;
D=[dq;Dw;vL;Dvl;Dqc;Dwc];
if strcmp(mode,'R2')
    D=[D;X(21:23,:);C(u(1))];
end
end

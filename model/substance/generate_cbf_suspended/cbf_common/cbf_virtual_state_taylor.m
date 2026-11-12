function [pL,qCable,pDrone] = cbf_virtual_state_taylor(x,nu,P,K)
%CBF_VIRTUAL_STATE_TAYLOR 理想平坦出力のTaylor係数から張り紐姿勢を復元。
% x=[pL; pL'; pL''; pL''' ;pL'''';pL''''' ; yaw;yaw']。
% nu=[pLx^(6);pLy^(6);pLz^(6);yaw'']。
% 紐方向q=-(pL''+g*e3-distXY)/norm(...)。張り・正張力が前提。
if numel(x)~=20 || numel(nu)~=4,error('CBF:StateSize','仮想入力状態20、決定入力4が必要です。');end
x=x(:);nu=nu(:);
pL=zeros(3,K+1);
for j=0:min(K,5)
    pL(:,j+1)=x(3*j+1:3*j+3)/factorial(j);
end
if K>=6,pL(:,7)=nu(1:3)/factorial(6);end
% pL'' のk階Taylor係数。k>4は今回の4次ケーブル条件で未使用。
a=zeros(3,K+1);
for j=0:min(K,4)
    if j+3<=size(pL,2)
        a(:,j+1)=(j+2)*(j+1)*pL(:,j+3);
    end
end
a(:,1)=a(:,1)+P(4)*[0;0;1]-[P(14);P(15);0];
if norm(a(:,1))<=eps(max(1,norm(a(:,1))))
    error('CBF:FlatMapSingularity','荷物加速度から紐方向を復元できない状態です。');
end
n2=cbf_jet('dot',a,a);
invN=cbf_jet('inv',cbf_jet('sqrt',n2));
qCable=-cbf_jet('mul',a,invN);
pDrone=pL-P(3)*qCable;
end

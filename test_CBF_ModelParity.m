function test_CBF_ModelParity()
%TEST_CBF_MODELPARITY 同じ「張った紐」の物理状態を2入力表現で照合。
% 実入力モデルで算出したddpLに対して、仮想側のpL''を一致させる。
test_CBF_SetupPaths(); s=test_CBF_Fixture();
m=s.physical.massDrone;mL=s.physical.massLoad;L=s.physical.cableLength;
g=s.physical.gravity;dist=[s.physical.disturbanceXY(:);0];
xR=zeros(29,1);
a=0.14; b=-0.09; q0=[cos(a/2);sin(a/2);0;0];
qy=[cos(b/2);0;sin(b/2);0];
% クォータニオン積（回転の順番はここでのみ使用）
xR(1:4)=[q0(1)*qy(1)-q0(2:4).'*qy(2:4); ...
    q0(1)*qy(2:4)+qy(1)*q0(2:4)+cross(q0(2:4),qy(2:4))];
xR(8:10)=[0.24;-0.07;1.5];xR(11:13)=[0.03;-0.02;0.01];
qc=[0.13;-0.15;-1];qc=qc/norm(qc);xR(14:16)=qc;
wc=[0.02;0.01;0.0];wc=wc-qc*(qc.'*wc);xR(17:19)=wc;
xR(20)=(m+mL)*g;
F=test_CBF_IndependentPlantRHS(xR,zeros(4,1),s);
pLddot=F(11:13);
assert(norm(pLddot+g*[0;0;1]-dist)>1e-4,'張力がゼロの状態は本テスト対象外です。');
aVec=pLddot+g*[0;0;1]-dist;
assert(norm(-aVec/norm(aVec)-qc)<1e-10, ...
    '実入力系のpL加速度と紐方向が一致しません（張力条件を再確認）。');
xV=zeros(20,1);xV(1:3)=xR(8:10);xV(4:6)=xR(11:13);
xV(7:9)=pLddot;
obsS=struct('center',[0.9;0.2;1.6],'radius',0.21);
c=0.21; bb=-0.43; aa=0.19;
Rz=[cos(c) -sin(c) 0;sin(c) cos(c) 0;0 0 1];
Ry=[cos(bb) 0 sin(bb);0 1 0;-sin(bb) 0 cos(bb)];
Rx=[1 0 0;0 cos(aa) -sin(aa);0 sin(aa) cos(aa)];
obsE=struct('center',obsS.center,'axes',[0.15;0.31;0.22],'R',Rz*Ry*Rx);
shapes={'C','B','D'};
for j=1:3
    for k=1:2
        if k==1,obs=obsS;kind='sphere';else,obs=obsE;kind='ellipsoid';end
        hr=cbf_evaluate_case('real',shapes{j},kind,xR,s,obs);
        hv=cbf_evaluate_case('virtual',shapes{j},kind,xV,s,obs);
        assert(numel(hr)==numel(hv));
        assert(norm(hr-hv,Inf)<1e-9*(1+norm(hr,Inf)), ...
            '実入力・仮想入力で同じ形状のhが一致しません: %s %s',shapes{j},kind);
    end
end
fprintf(' [PASS] C/B/D×球/回転楕円体: 同一物理状態のhが一致\n');
end

function test_CBF_HOCBF()
%TEST_CBF_HOCBF  実入力6次／仮想入力4・6次CBFの回帰テスト。
% common_matlab直下で test_CBF_All() から呼び出されます。
% 幾何0階一致、4入力のアフィン係数、初期補助安全条件、
% 独立したホバリング解析式、任意姿勢の回転楕円体を検査。
test_CBF_SetupPaths();
s=test_CBF_Fixture();
obsS=struct('center',[2;0;1.2],'radius',0.25);
% x/y/zの回転を合成した任意姿勢楕円体（非水平姿勢も含む）
a=0.23;b=-0.41;c=0.37;
Rx=[1 0 0;0 cos(a) -sin(a);0 sin(a) cos(a)];
Ry=[cos(b) 0 sin(b);0 1 0;-sin(b) 0 cos(b)];
Rz=[cos(c) -sin(c) 0;sin(c) cos(c) 0;0 0 1];
obsE=struct('center',[2;0;1.2],'axes',[0.25;0.38;0.50],'R',Rz*Ry*Rx);

xR=zeros(29,1);xR(1)=1;xR(8:10)=[0;0;1];xR(14:16)=[0;0;-1];
xR(20)=(s.physical.massDrone+s.physical.massLoad)*s.physical.gravity;
xV=zeros(20,1);xV(1:3)=xR(8:10);
types={'sphere','ellipsoid'};shapes={'C','B','D'};
for im=1:2
    if im==1,mode='real';x=xR;else,mode='virtual';x=xV;end
    for is=1:3
        for it=1:2
            shape=shapes{is};kind=types{it};
            if it==1,obs=obsS;else,obs=obsE;end
            fn=str2func(sprintf('CBF_%s_%s_%s',mode,shape,kind));
            [h,info,qp]=fn(x,s,obs);
            assert(size(qp.A,2)==4 && numel(qp.b)==size(qp.A,1));
            assert(isequal(size(qp.A,1),numel(qp.labels)));
            assert(all(isfinite(qp.A(:))) && all(isfinite(qp.b)));
            assert(all(qp.earlyInputResidual<1e-8),'低次入力が混入しています。');
            assert(all(h>0),'テストの安全距離が破れています。');
            if im==1
                assert(all(qp.order(1:numel(h))==6));
                assert(size(qp.A,1)==numel(h)+8);
                assert(qp.inputLimitIncluded);
            else
                assert(qp.order(1)==4 && qp.order(2)==6);
                assert(size(qp.A,1)==numel(h));
                assert(~qp.inputLimitIncluded);
                assert(norm(qp.A(:,4))<1e-8, '球・紐モデルにyaw入力が直接現れています。');
            end
            fprintf('[PASS] %s / %s / %s : %d件の高次CBF行\n',mode,shape,kind,numel(h));
        end
    end
end

% 独立解析：ホバリングで吊り荷の6階hにおける推力4階入力係数。
[~,~,qr]=CBF_real_B_sphere(xR,s,obsS);
pL=xR(8:10);d=pL-obsS.center;r=s.shape.loadRadius+obsS.radius+s.cbf.safetyMargin;
grad=2*d-r*d/sqrt(d.'*d+s.cbf.smoothEpsilon^2);
expected=grad(3)/(s.physical.massDrone+s.physical.massLoad);
assert(abs(qr.A(2,1)-expected)<1e-7*(1+abs(expected)), ...
    '吊り荷の6階hに対する推力係数が独立解析と一致しません。');
% 独立解析：同じ平衡点で仮想入力の機体4階hの水平作用係数。
[~,~,qv]=CBF_virtual_B_sphere(xV,s,obsS);
pD=pL+[0;0;s.physical.cableLength];d=pD-obsS.center;
r=s.shape.droneRadius+obsS.radius+s.cbf.safetyMargin;
grad=2*d-r*d/sqrt(d.'*d+s.cbf.smoothEpsilon^2);
expected=grad(1)*s.physical.cableLength/s.physical.gravity;
assert(abs(qv.A(1,1)-expected)<1e-7*(1+abs(expected)), ...
    '機体の4階hに対する仮想入力係数が独立解析と一致しません。');
% 独立解析：ホバリングでtau_x''が機体横方向の6階微分に現れる。
obsTorque=struct('center',[1;2;1.2],'radius',0.25);
[~,~,qtau]=CBF_real_B_sphere(xR,s,obsTorque);
pD=pL+[0;0;s.physical.cableLength];d=pD-obsTorque.center;
r=s.shape.droneRadius+obsTorque.radius+s.cbf.safetyMargin;
grad=2*d-r*d/sqrt(d.'*d+s.cbf.smoothEpsilon^2);
T0=xR(20);
expected=-grad(2)*T0/(s.physical.massDrone*s.physical.inertia(1));
assert(abs(qtau.A(1,2)-expected)<1e-6*(1+abs(expected)), ...
    '機体の6階hに対するtau_x二階入力係数が解析式と一致しません。');

% ケーブル軸方向では4階の入力係数がゼロになることを報告する。
obsAxis=struct('center',[0;0;3.0],'radius',0.25);
[~,~,qAxis]=CBF_virtual_B_sphere(xV,s,obsAxis);
assert(norm(qAxis.A(1,:))<1e-8 && qAxis.degenerate(1), ...
    'ケーブル軸方向で入力作用が退化することを検知できません。');

% 機体が傾き、紐に振れがある一般状態でもアフィン入力係数を検査。
xr=xR;theta=0.17;xr(1:4)=[cos(theta/2);sin(theta/2);0;0];
xr(5:7)=[0.04;-0.03;0.02];
qc=[0.11;-0.09;-1];xr(14:16)=qc/norm(qc);
xr(17:19)=[0.03;0.02;0.001];
xr(21:23)=[0.05;-0.02;0.01];
xr(24:26)=[0.02;-0.04;0.01];
xr(27:29)=[0.01;0.02;-0.02];
[~,~,qGeneral]=CBF_real_B_ellipsoid(xr,s,obsE);
assert(all(isfinite(qGeneral.A(:))) && all(isfinite(qGeneral.b)));
assert(all(qGeneral.earlyInputResidual<1e-7));
xv=xV;xv(4:6)=[0.2;-0.1;0.05];
xv(7:9)=[0.05;0.04;0.01];xv(10:12)=[0.02;0.01;-0.01];
xv(13:15)=[0.01;-0.02;0.01];xv(16:18)=[0.03;0.02;0.02];
[~,~,qGeneralV]=CBF_virtual_D_ellipsoid(xv,s,obsE);
assert(all(isfinite(qGeneralV.A(:))) && all(isfinite(qGeneralV.b)));
assert(all(qGeneralV.earlyInputResidual<1e-7));
fprintf('[PASS] ホバリング解析式、相対次数の低次作用、姿勢回転楕円体\n');
end

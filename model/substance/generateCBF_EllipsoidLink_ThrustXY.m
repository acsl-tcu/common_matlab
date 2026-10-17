%% generateCBF_EllipsoidLink_ThrustXY.m
% 19状態の既存モデルを推力2階積分器で21状態に拡張し、4階HOCBFを生成。
% 新規生成ファイル: CBF_Constraints_HOCBF_ELLIPSOID_THRUST_XY.m
% 元のXY版制約・名目制御器・モデルファイルは変更しない。
% 実行: generateCBF_EllipsoidLink_ThrustXY
% 必須: 19状態版 FLxyDst.m, GLxyDst.m, Symbolic Math Toolbox
%
% xAug=[x19; u1; du1], nu=d^2u1/dt^2.
% 実際の推力を逐次ゼロ次ホールドするシミュレーションでは、
% 理想的な連続時間二重積分器と一致しない（実行後に評価が必要）。

outName='CBF_Constraints_HOCBF_ELLIPSOID_THRUST_XY';
fprintf('\n=== Ellipsoid HOCBF with thrust dynamic extension ===\n');
assert(exist('FLxyDst','file')==2 && exist('GLxyDst','file')==2, ...
    '19状態版 FLxyDst.m と GLxyDst.m が必要です。');

%% 1. 既存19状態モデル: 元のgeneratorと順序・物理パラメータを合わせる
syms q0 q1 q2 q3 o1 o2 o3 real
syms pl1 pl2 pl3 dpl1 dpl2 dpl3 pT1 pT2 pT3 ol1 ol2 ol3 real
syms m Lx Ly lx ly jx jy jz gravity km1 km2 km3 km4 k1 k2 k3 k4 rotor_r mL cableL dstx dsty real
syms u1 du1 u2 u3 u4 real
syms obj real
q=[q0;q1;q2;q3]; ob=[o1;o2;o3];
pl=[pl1;pl2;pl3]; pT=[pT1;pT2;pT3];
x=[q;ob;pl;[dpl1;dpl2;dpl3];pT;[ol1;ol2;ol3]];
P22=[m,Lx,Ly,lx,ly,jx,jy,jz,gravity,km1,km2,km3,km4, ...
     k1,k2,k3,k4,rotor_r,mL,cableL,dstx,dsty];
P9=[m,jx,jy,jz,gravity,mL,cableL,dstx,dsty];
f=FLxyDst(x,P22);
g=GLxyDst(x,P22);
assert(isequal(size(f),[19,1]) && isequal(size(g),[19,4]), ...
    'FLxyDst/GLxyDstは19状態・4入力の関数が必要です。');
fprintf('[PASS] 19-state dynamics and 4 inputs\n');

%% 2. 動的拡張 (u1 と du1 を状態、nu=ddu1 を新たなQP入力にする)
xAug=[x;u1;du1];
fAug=[f+g(:,1)*u1;du1;sym(0)];
gNu=[sym(zeros(19,1));sym(0);sym(1)];
gT=[g(:,2:4);sym(zeros(2,3))];
gQP=[gNu,gT(:,1:2)];  % QP入力 [nu;u2;u3]
assert(isequal(size(fAug),[21,1]) && isequal(size(gQP),[21,3]), ...
    '動的拡張の状態・入力次元が不一致。');
fprintf('[PASS] 21-state augmented dynamics, controls [nu,u2,u3]; u4 fixed\n');

%% 3. 従来と同じ幾何安全関数 (Sherman-Morrison)
syms xo yo zo w11 w22 w33 w12 w13 w23 kappa real
syms gamma1 gamma2 gamma3 gamma4 real
W=[w11,w12,w13;w12,w22,w23;w13,w23,w33];
d=pl-(cableL/2)*pT-[xo;yo;zo];
h0=d.'*W*d - kappa*(pT.'*W*d)^2/(1+kappa*(pT.'*W*pT))-1;

%% 4. 4階HOCBF: phi1..phi3までは入力項を含めない
fprintf('[STEP] 4th-order Lie derivatives of the augmented system...\n');
psi0=h0;
psi1=jacobian(psi0,xAug)*fAug+gamma1*psi0;
psi2=jacobian(psi1,xAug)*fAug+gamma2*psi1;
psi3=jacobian(psi2,xAug)*fAug+gamma3*psi2;
% psi0~psi2 が機体角速度ob・du1に依存しないことを記号的に検証。
for k=1:3
    psi={psi0,psi1,psi2};
    assert(isequal(jacobian(psi{k},[ob;du1]),sym(zeros(1,4))), ...
        '%d階未満でQP操作入力が出現しています。',k);
end
fprintf('[PASS] Lie_g psi0..psi2 = 0 (symbolic)\n');

Lg=jacobian(psi3,xAug)*[gNu,gT]; % 1x4: nu,u2,u3,u4
Lf=jacobian(psi3,xAug)*fAug;
A_sym=-Lg(1:3);
b_sym=Lf+Lg(4)*u4+gamma4*psi3;

%% 5. 制約を別名で生成。元ファイルへの上書きはしない。
XD_unused=sym('XD_unused',[28,1]);
sys_unused=sym('sys_unused');
obs_params=[xo;yo;zo;w11;w22;w33;w12;w13;w23;kappa];
gamma_params=[gamma1;gamma2;gamma3;gamma4];
fprintf('[STEP] Generating %s.m ...\n',outName);
matlabFunction(A_sym,b_sym,psi0,psi1,psi2,psi3, ...
    'File',[outName,'.m'], ...
    'Vars',{obj,xAug,XD_unused,u4,obs_params,gamma_params,sys_unused,P9}, ...
    'Outputs',{'A_qp','b_qp','h1','h2','h3','h4'}, ...
    'Optimize',true);
rehash;
assert(exist(outName,'file')==2,'CBF関数の生成に失敗。');
fprintf('[PASS] Generated %s.m\n',outName);

%% 6. 自動検証: 相対次数、4階係数、幾何h、QP右辺を数値微分で照合
fprintf('\n=== AUTO VERIFY (augmented model) ===\n');
rx=20*pi/180;ry=-15*pi/180;rz=25*pi/180;
Rx=[1,0,0;0,cos(rx),-sin(rx);0,sin(rx),cos(rx)];
Ry=[cos(ry),0,sin(ry);0,1,0;-sin(ry),0,cos(ry)];
Rz=[cos(rz),-sin(rz),0;sin(rz),cos(rz),0;0,0,1];
center=[0.5;1;0.5];radii=[0.75;1;0.65];
Pn=[0.75,0.02,0.025,0.035,9.81,0.14,2.0,0,0];
gam=[2;8;16;32]; u4n=0.04;
[shape,C,kapp]=localShape(Rz*Ry*Rx,radii,Pn(7),0.04,0.10,1);
obsn=[center;shape];
p22=zeros(1,22);p22([1,6,7,8,9,19,20,21,22])=Pn;
rng(20261017,'twister');
N=8; maxLow=0;maxA=0;maxB=0;maxH=0;maxNuLink=0;nonzeroNu=0;nonzeroTorque=0;
for i=1:N
    qn=[1;0.1*randn(3,1)];qn=qn/norm(qn);
    pn=[0.2*randn(2,1);-1];pn=pn/norm(pn);
    ol=0.2*randn(3,1);ol=ol-pn*(pn.'*ol);
    xn=[qn;0.2*randn(3,1);0.7*randn(3,1);0.3*randn(3,1);pn;ol];
    xe=[xn;8+rand();0.3*randn()];
    ev=@(xx)feval(outName,[],xx,zeros(28,1),u4n,obsn,gam,0.04,Pn);
    [An,bn,h1n,h2n,h3n,h4n]=ev(xe);
    assert(numel(An)==3 && all(isfinite([An(:);bn;h1n;h2n;h3n;h4n])), ...
        '関数の出力に異常があります (sample=%d)',i);
    p=xe(8:10)-Pn(7)/2*xe(14:16)-center;
    Sout=C+kapp*(pn*pn.');
    hGeom=p.'*(Sout\p)-1;
    maxH=max(maxH,abs(h1n-hGeom));
    assert(abs(h1n-hGeom)<1e-7*max(1,abs(hGeom)), ...
        '幾何安全関数が一致しません (sample=%d)',i);

    Gn=GLxyDst(xn,p22);Fn=FLxyDst(xn,p22);
    cols=[[zeros(20,1);1], [Gn(:,2:4);zeros(2,3)]]; % nu,u2,u3,u4
    coeff=zeros(4,4);
    for j=1:4
        direction=cols(:,j);
        e=1e-5/max(1,norm(direction,inf));
        [~,~,a,b,c,d4]=ev(xe+e*direction);
        [~,~,aa,bb,cc,dd]=ev(xe-e*direction);
        coeff(:,j)=([a;b;c;d4]-[aa;bb;cc;dd])/(2*e);
    end
    maxLow=max(maxLow,max(abs(coeff(1:3,:)),[],'all'));
    maxA=max(maxA,max(abs(An(:).'+coeff(4,1:3))));
    if abs(coeff(4,1))>1e-7,nonzeroNu=nonzeroNu+1;end
    if norm(coeff(4,2:3))>1e-7,nonzeroTorque=nonzeroTorque+1;end
    % d(psi3)/d(du1) = d(psi2)/d(u1) の関係を確認
    e=1e-5;
    xp=xe;xm=xe;xp(20)=xp(20)+e;xm(20)=xm(20)-e;
    [~,~,~,~,psi2p]=ev(xp);[~,~,~,~,psi2m]=ev(xm);
    maxNuLink=max(maxNuLink,abs(coeff(4,1)-(psi2p-psi2m)/(2*e)));
    drift=[Fn+Gn(:,1)*xe(20)+Gn(:,4)*u4n;xe(21);0];
    e=1e-6/max(1,norm(drift,inf));
    [~,~,~,~,~,psi3p]=ev(xe+e*drift);
    [~,~,~,~,~,psi3m]=ev(xe-e*drift);
    bFD=(psi3p-psi3m)/(2*e)+gam(4)*h4n;
    maxB=max(maxB,abs(bn-bFD));
    assert(max(abs(coeff(1:3,:)),[],'all')<1e-5, ...
        '1~3階にQP入力係数が存在します。');
    assert(max(abs(An(:).'+coeff(4,1:3)))<3e-3*max(1,norm(An,inf)), ...
        'QPのA=-Lgが一致しません。');
    assert(abs(bn-bFD)<4e-3*max(1,abs(bn)), ...
        'QPのbがLie微分と一致しません。');
    if i==1
        fprintf('[TRACE] A=[%.6g %.6g %.6g], b=%.6g\n',An,bn);
        fprintf('[TRACE] coeff[nu,u2,u3,u4]=[%.6g %.6g %.6g %.6g]\n',coeff(4,:));
    end
end
fprintf('[CHECK] 1~3階の最大入力係数     %.3e\n',maxLow);
fprintf('[CHECK] 4階のnu係数の非零     %d/%d\n',nonzeroNu,N);
fprintf('[CHECK] 4階のXY係数の非零     %d/%d\n',nonzeroTorque,N);
fprintf('[CHECK] max |A+Lg|             %.3e\n',maxA);
fprintf('[CHECK] max |b-bFD|            %.3e\n',maxB);
fprintf('[CHECK] max |h-hGeom|         %.3e\n',maxH);
fprintf('[CHECK] max |Lg_nu - dpsi2/du1| %.3e\n',maxNuLink);
assert(nonzeroNu>=ceil(0.75*N) && nonzeroTorque>=ceil(0.75*N), ...
    '4階で新規入力またはXYトルクが十分に現れません。');
assert(maxNuLink<3e-3,'推力動的拡張の微分関係に誤差があります。');
fprintf('[PASS] 21状態の制約生成と数値検証に成功しました。\n');
fprintf('[NOTE] 連続時間二重積分器と、ZOH推力プラントの差は未検証です。\n');

function [shape,C,kappa]=localShape(R,r,L,ropeRadius,margin,beta)
rho=ropeRadius+margin;a=L/2+rho;b2=rho*a;
kappa=(1+beta)*(a*a-b2);
C=(1+1/beta)*R*diag(r(:).^2)*R.'+(1+beta)*b2*eye(3);
W=C\eye(3);W=(W+W.')/2;
shape=[W(1,1);W(2,2);W(3,3);W(1,2);W(1,3);W(2,3);kappa];
end

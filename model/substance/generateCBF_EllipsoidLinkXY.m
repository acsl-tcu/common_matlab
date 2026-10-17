%% generateCBF_EllipsoidLinkXY.m
% 既存 generateController_SuspendedLoad_HOCBF_LINK_XY.m の HOCBF 部分のみ実行。
% ノミナル制御器は生成しない。生成後に相対次数とQP係数を自動検証する。
% 必要: Symbolic Math Toolbox, 19状態版 FLxyDst.m / GLxyDst.m
% 実行: generateCBF_EllipsoidLinkXY
%
% 制約関数の引数は旧版と同じ9個:
% [A,b,h1,h2,h3,h4] = CBF_Constraints_HOCBF_ELLIPSOID_LINK_XY( ...
%     obj,x,xd,u1_val,u4_val,obs_params,gamma_params,sys_params,P);
% obs_params = [障害物中心(3); Wの独立成分(6); kappa] (10x1)
% W,kappa は静止楕円体と紐寸法から各障害物ごとに事前計算する。
% sys_params と xd は既存呼び出しとの互換性のため残す（式には使わない）。
%
% 理論上の前提: Lie微分中のu1は時間一定。時変u1の厳密な安全保証ではない。

scriptDir = fileparts(mfilename('fullpath'));
if ~isempty(scriptDir), cd(scriptDir); end
outName = 'CBF_Constraints_HOCBF_ELLIPSOID_LINK_XY';
fprintf('\n=== Ellipsoid link HOCBF: generate + verify ===\n');
assert(exist('FLxyDst','file')==2 && exist('GLxyDst','file')==2, ...
    '19状態版のFLxyDst.m, GLxyDst.m が必要です。');

%% 1. 19状態モデル（元のgenerateControllerと同じ変数・順序）
syms q0 q1 q2 q3 o1 o2 o3 real
syms pl1 pl2 pl3 dpl1 dpl2 dpl3 pT1 pT2 pT3 ol1 ol2 ol3 real
syms m Lx Ly lx ly jx jy jz gravity km1 km2 km3 km4 k1 k2 k3 k4 rotor_r mL cableL dstx dsty real
syms u1 u2 u3 u4 U1_val U4_val real
syms obj real
q = [q0;q1;q2;q3]; ob=[o1;o2;o3]; pl=[pl1;pl2;pl3]; pT=[pT1;pT2;pT3];
x = [q;ob;pl;[dpl1;dpl2;dpl3];pT;[ol1;ol2;ol3]];
param22 = [m,Lx,Ly,lx,ly,jx,jy,jz,gravity,km1,km2,km3,km4, ...
           k1,k2,k3,k4,rotor_r,mL,cableL,dstx,dsty];
physicalParam = [m,jx,jy,jz,gravity,mL,cableL,dstx,dsty];
f = FLxyDst(x,param22);
g = GLxyDst(x,param22);
assert(isequal(size(f),[19,1]) && isequal(size(g),[19,4]), ...
    'FLxyDst/GLxyDst が19状態モデルではありません（モデルの上書きに注意）。');
fprintf('[PASS] 19-state F(19x1), G(19x4)\n');

%% 2. 元のHOCBFと同じ固定u1分解
FG_xy = f + g*[u1;u2;u3;u4];
f_xy  = subs(FG_xy,[u2,u3,u4],[0,0,0]);
g_xy  = g(:,2:4);  % 入力アフィンなので MyCoeff(FG_xy,[u2;u3;u4]) と同じ
assert(isequal(simplify(FG_xy-f_xy-g_xy*[u2;u3;u4]),sym(zeros(19,1))), ...
    '入力分解 FG_xy=f_xy+g_xy*u234 に不整合があります。');

%% 3. 変更箇所: 球体hを任意姿勢楕円体×紐姿勢楕円体のhへ置換
% d = xL-(L/2)pT-cObs
% S_out = C + kappa*pT*pT', W=C^(-1)
% h = d' S_out^(-1) d - 1 （Sherman-Morrisonで3x3記号逆行列を回避）
syms xo yo zo w11 w22 w33 w12 w13 w23 kappa real
syms gamma1 gamma2 gamma3 gamma4 real
W = [w11,w12,w13;w12,w22,w23;w13,w23,w33];
d = pl - (cableL/2)*pT - [xo;yo;zo];
h0 = d.'*W*d - kappa*(pT.'*W*d)^2/(1+kappa*(pT.'*W*pT)) - 1;

%% 4. 既存の4階HOCBF再帰・QPの符号は維持
fprintf('[STEP] HOCBF Lie derivatives...\n');
psi0 = h0;
psi1 = jacobian(psi0,x)*f_xy + gamma1*psi0;
psi2 = jacobian(psi1,x)*f_xy + gamma2*psi1;
psi3 = jacobian(psi2,x)*f_xy + gamma3*psi2;

% g(:,2:4)は機体角速度の微分にだけ作用。
% したがってpsi0～psi2が角速度obに依存しないことが1～3階の条件。
lowPsi = {psi0,psi1,psi2};
for k = 1:3
    assert(isequal(jacobian(lowPsi{k},ob),sym(zeros(1,3))), ...
        '%d階でu2,u3,u4が現れます。',k);
end
fprintf('[PASS] 1～3階のトルク係数 = 0 (symbolic)\n');

Lf_psi3 = jacobian(psi3,x)*f_xy;
Lg_psi3 = jacobian(psi3,x)*g_xy;  % [u2,u3,u4] の係数
A_sym = -Lg_psi3(1:2);
b_sym = Lf_psi3 + Lg_psi3(3)*u4 + gamma4*psi3;

%% 5. 制約関数を生成（ノミナル側の関数は生成しない）
XD_unused = sym('XD_unused',[28,1]);
sys_unused = sym('sys_unused');
obs_params = [xo;yo;zo;w11;w22;w33;w12;w13;w23;kappa];
gamma_params = [gamma1;gamma2;gamma3;gamma4];
A_out = subs(A_sym,[u1,u4],[U1_val,U4_val]);
b_out = subs(b_sym,[u1,u4],[U1_val,U4_val]);
h_out = subs([psi0;psi1;psi2;psi3],u1,U1_val);
fprintf('[STEP] Generating %s.m ...\n',outName);
matlabFunction(A_out,b_out,h_out(1),h_out(2),h_out(3),h_out(4), ...
    'File',[outName '.m'], ...
    'Vars',{obj,x,XD_unused,U1_val,U4_val,obs_params,gamma_params,sys_unused,physicalParam}, ...
    'Outputs',{'A_qp','b_qp','h1','h2','h3','h4'}, ...
    'Optimize',true);
rehash;
assert(exist(outName,'file')==2,'制約関数の生成に失敗しました。');
fprintf('[PASS] Generated %s.m\n',outName);

%% 6. 自動検証: 相対次数とQP係数、楕円体幾何
fprintf('\n=== AUTO VERIFY ===\n');
% 実際の障害物姿勢は任意: 検証にはロール/ピッチ/ヨー複合回転を使用
rx=20*pi/180; ry=-15*pi/180; rz=25*pi/180;
Rx=[1,0,0;0,cos(rx),-sin(rx);0,sin(rx),cos(rx)];
Ry=[cos(ry),0,sin(ry);0,1,0;-sin(ry),0,cos(ry)];
Rz=[cos(rz),-sin(rz),0;sin(rz),cos(rz),0;0,0,1];
center=[0.5;1.0;0.5]; radii=[0.75;1.0;0.65];
Pn=[0.75,0.02,0.025,0.035,9.81,0.14,2.0,0,0];
gam=[2;8;16;32]; u1n=8; u4n=0.04;
[shape,Sout]=localEllipsoidShape(Rz*Ry*Rx,radii,Pn(7),0.04,0.15,1.0);
obsn=[center;shape];
P22=zeros(1,22); P22([1,6,7,8,9,19,20,21,22])=Pn;
rng(20261017,'twister');
N=8; maxLow=0; maxA=0; maxB=0; maxH=0; maxYaw=0; active4=0; activeU1=0;
for i=1:N
    qn=[1;0.1*randn(3,1)]; qn=qn/norm(qn);
    pn=[0.2*randn(2,1);-1]; pn=pn/norm(pn);
    ol=0.2*randn(3,1); ol=ol-pn*(pn.'*ol);
    xn=[qn;0.2*randn(3,1);0.7*randn(3,1);0.3*randn(3,1);pn;ol];
    evalC=@(xx) feval(outName,[],xx,zeros(28,1),u1n,u4n,obsn,gam,0.04,Pn);
    [An,bn,h1n,h2n,h3n,h4n]=evalC(xn);
    assert(numel(An)==2 && isscalar(bn) && ...
        all(isfinite([An(:);bn;h1n;h2n;h3n;h4n])), ...
        '出力サイズまたは有限値の異常: sample=%d',i);

    dp=xn(8:10)-Pn(7)/2*xn(14:16)-center;
    hGeom=dp.'*(Sout(xn(14:16))\dp)-1;
    maxH=max(maxH,abs(h1n-hGeom));
    assert(abs(h1n-hGeom)<1e-7*max(1,abs(hGeom)), ...
        '安全関数の幾何式が不一致: sample=%d',i);

    Gn=GLxyDst(xn,P22); Fn=FLxyDst(xn,P22);
    coeff=zeros(4,3);
    for j=1:3
        dir=Gn(:,j+1); dt=1e-5/max(1,norm(dir,inf));
        [~,~,ap,bp,cp,ep]=evalC(xn+dt*dir);
        [~,~,am,bm,cm,em]=evalC(xn-dt*dir);
        coeff(:,j)=([ap;bp;cp;ep]-[am;bm;cm;em])/(2*dt);
    end
    maxLow=max(maxLow,max(abs(coeff(1:3,:)),[],'all'));
    maxA=max(maxA,max(abs(An(:).'+coeff(4,1:2))));
    if norm(coeff(4,1:2))>1e-7, active4=active4+1; end
    maxYaw=max(maxYaw,abs(coeff(4,3)));
    % 推力 u1 が安全関数の2階に現れるかも確認（今回は固定入力）
    dirU1=Gn(:,1); dtU1=1e-5/max(1,norm(dirU1,inf));
    [~,~,~,h2p]=evalC(xn+dtU1*dirU1);
    [~,~,~,h2m]=evalC(xn-dtU1*dirU1);
    if abs((h2p-h2m)/(2*dtU1))>1e-7, activeU1=activeU1+1; end

    drift=Fn+Gn(:,1)*u1n+Gn(:,4)*u4n;
    dt=1e-6/max(1,norm(drift,inf));
    [~,~,~,~,~,p]=evalC(xn+dt*drift);
    [~,~,~,~,~,m4]=evalC(xn-dt*drift);
    bFD=(p-m4)/(2*dt)+gam(4)*h4n;
    maxB=max(maxB,abs(bn-bFD));
    assert(max(abs(coeff(1:3,:)),[],'all')<1e-5, ...
        '1～3階にトルクの数値係数あり: sample=%d',i);
    assert(max(abs(An(:).'+coeff(4,1:2)))<2e-3*max(1,norm(An,inf)), ...
        'QPのA=-Lgが不一致: sample=%d',i);
    assert(abs(bn-bFD)<3e-3*max(1,abs(bn)), ...
        'QPのbが4階Lie微分と不一致: sample=%d',i);
    if i==1
        fprintf('[TRACE] A=[%.6g, %.6g], b=%.6g, h=[%.6g, %.6g, %.6g, %.6g]\n', ...
            An(1),An(2),bn,h1n,h2n,h3n,h4n);
        fprintf('[TRACE] 4階係数 [u2,u3,u4]=[%.6g %.6g %.6g]\n',coeff(4,:));
    end
end
fprintf('[CHECK] 1～3階: max coefficient = %.3e\n',maxLow);
fprintf('[CHECK] 2階: u1係数非ゼロ = %d/%d (今回は固定)\n',activeU1,N);
fprintf('[CHECK] 4階: XY係数非ゼロ = %d/%d\n',active4,N);
fprintf('[CHECK] 4階: max |u4係数| = %.3e\n',maxYaw);
fprintf('[CHECK] max |A+Lg| = %.3e\n',maxA);
fprintf('[CHECK] max |b-bFD| = %.3e\n',maxB);
fprintf('[CHECK] max |h-hGeom| = %.3e\n',maxH);
assert(activeU1>=ceil(0.75*N),'2階のu1入力係数が十分に現れません。');
assert(active4>=ceil(0.75*N),'4階のXY入力係数が十分に現れません。');
fprintf('[PASS] 自動検証成功: 生成された制約を数値微分で照合しました。\n');
fprintf('[NOTE] 推力u1の時間微分は含まれません。連続時間の安全保証は未確認です。\n');

function [shape,Sout] = localEllipsoidShape(R,r,L,ropeRadius,margin,beta)
% 紐カプセルを含む楕円体と静止障害物楕円体のMinkowski和を外接近似。
assert(all(size(R)==[3,3]) && norm(R.'*R-eye(3),'fro')<1e-8 ...
    && det(R)>0 && numel(r)==3 && all(r>0) && L>0 ...
    && ropeRadius>0 && margin>=0 && beta>0, '楕円体パラメータが不正です。');
rho=ropeRadius+margin;
a=L/2+rho; b2=rho*a;
kappa=(1+beta)*(a*a-b2);
Sobs=R*diag(r(:).^2)*R.';
C=(1+1/beta)*Sobs+(1+beta)*b2*eye(3);
W=C\eye(3); W=(W+W.')/2;
shape=[W(1,1);W(2,2);W(3,3);W(1,2);W(1,3);W(2,3);kappa];
Sout=@(pT) C+kappa*(pT*pT.');
end

%% generateCBF_ELLIPSOID_LINK_XY_VERIFY.m
% 目的：既存 generateController_SuspendedLoad_HOCBF_LINK_XY.m の
%       「実推力 u1 固定 -> 4階 HOCBF -> A*[u2;u3] <= b」を変えず、
%       紐中点球体の安全関数のみ任意姿勢楕円体×紐方向楕円体に置換する。
%
% 1) CBF_Constraints_HOCBF_ELLIPSOID_LINK_XY.m を一つだけ生成
% 2) このスクリプト内で相対次数と A,b の数値検査まで実行
%    （別途 verify.m は生成しない）
%
% 必要なもの：Symbolic Math Toolbox、同じフォルダまたはパス上の
%             「19状態版」FLxyDst.m と GLxyDst.m
% 利用方法：MATLABのコマンドウィンドウで
%             generateCBF_ELLIPSOID_LINK_XY_VERIFY
%
% 【大事な注意】
% ・元の第1層 / 第2層コントローラの生成・上書きはしない。
% ・u1 は Lie 微分の間は定数とする（サンプルホールド仮定）。
%   実際の u1(x,t) の時間微分を含めた連続時間安全保証ではない。
% ・静止した任意姿勢楕円体に対する「紐の衝突回避」のみを対象とする。
%   機体本体、吊り荷本体の安全性は別途必要。
%
% -------------------------------------------------------------------------
% 既存コントローラの呼び出しと引数の順序を揃えた9引数：
%   [A,b,h1,h2,h3,h4] = CBF_Constraints_HOCBF_ELLIPSOID_LINK_XY( ...
%     obj, x, xd, u1_val, u4_val, obs_params, gamma_params, sys_params, P);
%   obs_params = [xo;yo;zo; w11;w22;w33;w12;w13;w23;kappa]  (10x1)
%   xd(28x1) と sys_params は旧API互換の未使用引数。
%   形状パラメータの作り方は末尾 localMakeShape を参照。
% -------------------------------------------------------------------------

scriptDir = fileparts(mfilename('fullpath'));
if ~isempty(scriptDir), cd(scriptDir); end
clc;
clearvars -except scriptDir;

outputName = 'CBF_Constraints_HOCBF_ELLIPSOID_LINK_XY';
numTests = 12;   % 簡易検証のランダム状態数（増やしてよい）

fprintf('\n========== ELLIPSOID HOCBF GENERATOR ==========\n');
fprintf('[INFO] output: %s.m\n',outputName);
fprintf('[INFO] u1 fixed during Lie differentiation; optimize u2,u3 only.\n');
assert(exist('FLxyDst','file')==2 && exist('GLxyDst','file')==2, ...
    'FLxyDst.m と GLxyDst.m (19状態版) をパスに配置してください');

%% (A) 既存コードと同じ 19 状態の定義・運動方程式
syms q0 q1 q2 q3 o1 o2 o3 real
syms pl1 pl2 pl3 dpl1 dpl2 dpl3 pT1 pT2 pT3 ol1 ol2 ol3 real
syms m jx jy jz gravity mL cableL dstx dsty real
syms Lx Ly lx ly km1 km2 km3 km4 k1 k2 k3 k4 rotor_r real
syms u1 u2 u3 u4 real
syms U1_val U4_val real
syms obj real

q = [q0;q1;q2;q3];
ob = [o1;o2;o3];
pl = [pl1;pl2;pl3];
pT = [pT1;pT2;pT3];
x = [q;ob;pl;[dpl1;dpl2;dpl3];pT;[ol1;ol2;ol3]];

% 与えられた generateController と同じ22項パラメータで FL/GL を呼ぶ。
param22 = [m,Lx,Ly,lx,ly,jx,jy,jz,gravity,km1,km2,km3,km4, ...
           k1,k2,k3,k4,rotor_r,mL,cableL,dstx,dsty];
P = [m,jx,jy,jz,gravity,mL,cableL,dstx,dsty];
try
    f = FLxyDst(x,param22);
    g = GLxyDst(x,param22);
catch ME
    error('19状態版 FLxyDst/GLxyDst を呼び出せません。別モデルによる上書きを確認してください。詳細: %s',ME.message);
end
assert(isequal(size(f),[19 1]) && isequal(size(g),[19 4]), ...
    ['FL/GL が19x1/19x4ではありません。generateModel_SupendedLoad.m の' ...
     '後半で別のモデルに上書きされていないか確認してください。']);
assert(isequal(size(g(5:7,2:4)),[3 3]),'トルク入力行列のサイズが異なります');
fprintf('[PASS] Model dimensions: F=19x1, G=19x4\n');

%% (B) 既存制約と同じ「確定した実入力 u1 を保持する」分解
FG_xy = f + g*[u1;u2;u3;u4];
f_xy = subs(FG_xy,[u2,u3,u4],[0,0,0]);
g_xy = g(:,2:4);  % [u2 u3 u4] の入力列。MyCoeff と同じ結果。

affineGap = simplify(FG_xy-(f_xy+g_xy*[u2;u3;u4]));
assert(all(isAlways(affineGap==0),'all'), '入力アフィン分解に誤りがあります');
fprintf('[PASS] Input-affine separation: FG = f_xy + G23*[u2;u3;u4]\n');

%% (C) ★ここが球体版からの局所変更：安全関数 h0
% 紐中心 c_s = x_L - (L/2)*pT, 姿勢方向 pT（単位長さ）
% 障害物楕円体中心 c_o, 形状 S_o=R*diag(radii.^2)*R'.
% 紐のカプセルを外接する楕円体 S_s = b^2 I + (a^2-b^2) pT pT'.
% 楕円体のMinkowski和の外接近似：
%   S_out=(1+1/beta)*S_o + (1+beta)*S_s = C+kappa*pT*pT'.
%   W=inv(C), kappa=(1+beta)*(a^2-b^2).
% Sherman--Morrisonにより S_out^{-1} を3x3の記号逆行列なしで記述。
% betaは正の定数。pTを単位長さとする状態多様体上で正しい外接形状。
syms xo yo zo w11 w22 w33 w12 w13 w23 kappa real
syms gamma1 gamma2 gamma3 gamma4 real
W = [w11,w12,w13;w12,w22,w23;w13,w23,w33];
cObs = [xo;yo;zo];
cLink = pl-(cableL/2)*pT;
d = cLink-cObs;
h0 = d.'*W*d - kappa*(pT.'*W*d)^2/(1+kappa*(pT.'*W*pT)) - 1;

%% (D) ★ここは元のHOCBF式のまま（psi1,psi2,psi3, A, b）
% 4階相対次数：h0,h0dot,h0ddot,h0(3),h0(4)の順に入力を調べる。
% gamma1..4 > 0 とする HOCBF 再帰。
fprintf('[INFO] Deriving HOCBF derivatives; Symbolic may take time...\n');
clockStart = tic;
psi0 = h0;
psi1 = jacobian(psi0,x)*f_xy + gamma1*psi0;
fprintf('[STEP] psi1 derived (%.1f s)\n',toc(clockStart));
psi2 = jacobian(psi1,x)*f_xy + gamma2*psi1;
fprintf('[STEP] psi2 derived (%.1f s)\n',toc(clockStart));
psi3 = jacobian(psi2,x)*f_xy + gamma3*psi2;
fprintf('[STEP] psi3 derived (%.1f s)\n',toc(clockStart));

% 理論上 ψ0, ψ1, ψ2 には機体角速度 ob が含まれない。
% 入力 u2,u3,u4 は obdot にのみ作用するので、1,2,3階で入力が出ない。
low1 = jacobian(psi0,ob);
low2 = jacobian(psi1,ob);
low3 = jacobian(psi2,ob);
assert(isequal(low1,sym(zeros(1,3))), '第1階でトルクが出現');
assert(isequal(low2,sym(zeros(1,3))), '第2階でトルクが出現');
assert(isequal(low3,sym(zeros(1,3))), '第3階でトルクが出現');
fprintf('[PASS] Structural Lie test: u2,u3,u4 coefficients are zero at orders 1..3\n');

Lf_psi3 = jacobian(psi3,x)*f_xy;
Lg_psi3 = jacobian(psi3,x)*g_xy; % [coefficient u2, u3, u4]
A_sym = -Lg_psi3(1:2);
b_sym = Lf_psi3 + Lg_psi3(3)*u4 + gamma4*psi3;
fprintf('[STEP] 4th-order A,b derived (%.1f s)\n',toc(clockStart));

% 旧関数と同じ9引数にして、後でコントローラを最小限だけ差し替えられるようにする。
XD_dummy = sym('XD_unused',[28 1]);  % 参照軌道は幾何安全関数では使用しない。
sys_dummy = sym('sys_unused');        % 旧APIのrl。形状は obs_params に事前計算。
obs_params = [xo;yo;zo;w11;w22;w33;w12;w13;w23;kappa];
gam_params = [gamma1;gamma2;gamma3;gamma4];

A_out = subs(A_sym,[u1,u4],[U1_val,U4_val]);
b_out = subs(b_sym,[u1,u4],[U1_val,U4_val]);
h_out = subs([psi0;psi1;psi2;psi3],u1,U1_val);

%% (E) 制約ファイルをひとつだけ生成
fprintf('[INFO] Exporting %s.m ...\n',outputName);
matlabFunction(A_out,b_out,h_out(1),h_out(2),h_out(3),h_out(4), ...
    'File',[outputName '.m'], ...
    'Vars',{obj,x,XD_dummy,U1_val,U4_val,obs_params,gam_params,sys_dummy,P}, ...
    'Outputs',{'A_qp','b_qp','h1','h2','h3','h4'}, ...
    'Optimize',true);
rehash;
assert(exist(outputName,'file')==2,'制約ファイルの生成が確認できません');
fprintf('[PASS] Generated: %s.m (%.1f s)\n',outputName,toc(clockStart));

%% (F) ★検証を同じgenerateファイル内で実行（別のverify.mは不要）
% verifyの中では生成関数を実際に呼び、差分方向微分を用いて独立に検査。
% 数学的に保証された一律の安全性ではなく、生成の整合性の数値サンプル検証。
fprintf('\n========== AUTO VERIFY (inside generator) ==========\n');

% テスト用の任意3軸回転、異方性半径を設定（障害物は静止）。
rz = 25*pi/180; ry = -15*pi/180; rx = 20*pi/180;
Rz=[cos(rz),-sin(rz),0;sin(rz),cos(rz),0;0,0,1];
Ry=[cos(ry),0,sin(ry);0,1,0;-sin(ry),0,cos(ry)];
Rx=[1,0,0;0,cos(rx),-sin(rx);0,sin(rx),cos(rx)];
Robs=Rz*Ry*Rx;
center=[0.5;7.5;4.0]; radii=[0.75;1.0;0.65];
Pnum=[0.75,0.02,0.025,0.035,9.81,0.14,2.0,0.0,0.0];
gam=[2;8;16;32]; thrust=8.0; yawTorque=0.04;
cableRadius=0.04; margin=0.15; betaShape=1.0;
[shape, Souter] = localMakeShape(Robs,radii,Pnum(7),cableRadius,margin,betaShape);
params=[center;shape];  % center3 + W6 + kappa1 => 10
fprintf('[INFO] Test obstacle Rz*Ry*Rx=[25,-15,20]deg, radii=[.75,1,.65]m\n');

rng(20261017,'twister');
worstLow=0; worstA=0; worstB=0; worstH=0;
nonzeroXY=0; maxYaw=0; nonzeroThrustSecond=0;
for sample=1:numTests
    qn=randn(4,1); qn=qn/norm(qn);
    pn=randn(3,1); pn=pn/norm(pn);
    omn=0.25*randn(3,1); omn=omn-(omn.'*pn)*pn;
    xn=[qn;0.2*randn(3,1); 2*randn(3,1);0.5*randn(3,1);pn;omn];
    xdnum=zeros(28,1); sysnum=cableRadius;
    [An,bn,h1n,h2n,h3n,h4n] = feval(outputName,[],xn,xdnum, ...
        thrust,yawTorque,params,gam,sysnum,Pnum);
    assert(numel(An)==2 && isscalar(bn), 'QP出力のサイズが違います');
    assert(all(isfinite([An(:);bn;h1n;h2n;h3n;h4n])), ...
        '非有限値が出ました（sample=%d）',sample);

    % A) 元の外接形状行列から直接計算したh0と生成出力が一致するか
    cmid = xn(8:10)-(Pnum(7)/2)*xn(14:16);
    dif = cmid-center;
    hDirect=dif.'*(Souter(xn(14:16))\dif)-1;
    hgap=abs(h1n-hDirect);
    worstH=max(worstH,hgap);
    assert(hgap < 2e-8*max(1,abs(hDirect)), ...
        'h1の幾何学的な計算と不一致（sample=%d）',sample);

    % B) FLxyDst/GLxyDst に基づく有限差分で入力が初めて現れる階数を検査
    Gnum=GLxyDst(xn,localP22(Pnum));
    Fnum=FLxyDst(xn,localP22(Pnum));
    lowSample=0; numericLG4=zeros(1,3);
    for j=2:4
        vdir=Gnum(:,j);
        step=1e-5/max(1,norm(vdir,inf));
        xp=xn+step*vdir; xm=xn-step*vdir;
        [~,~,p1,p2,p3,p4]=feval(outputName,[],xp,xdnum, ...
            thrust,yawTorque,params,gam,sysnum,Pnum);
        [~,~,m1,m2,m3,m4]=feval(outputName,[],xm,xdnum, ...
            thrust,yawTorque,params,gam,sysnum,Pnum);
        numericalLG=([p1;p2;p3;p4]-[m1;m2;m3;m4])/(2*step);
        lowSample=max(lowSample,max(abs(numericalLG(1:3))));
        numericLG4(j-1)=numericalLG(4);
    end
    worstLow=max(worstLow,lowSample);
    if norm(numericLG4(1:2))>1e-7, nonzeroXY=nonzeroXY+1; end
    maxYaw=max(maxYaw,abs(numericLG4(3)));
    assert(lowSample<1e-5, ...
        '1〜3階で水平トルクが現れました（sample=%d; %.3e）',sample,lowSample);

    % C) QP の A は 4階トルク係数の負号と一致しているか
    errA=max(abs(reshape(An,1,2)+numericLG4(1:2)));
    worstA=max(worstA,errA);
    assert(errA<2e-3*max(1,norm(An,inf)), ...
        'QP係数Aの符号/値が4階入力係数と不一致（sample=%d）',sample);

    % D) QPの b は L_{f_xy+g4*u4}(psi3)+gamma4*psi3 と一致するか
    drift=Fnum+Gnum(:,1)*thrust+Gnum(:,4)*yawTorque;
    dtTest=1e-6/max(1,norm(drift,inf));
    xPlus=xn+dtTest*drift; xMinus=xn-dtTest*drift;
    [~,~,~,~,~,psiPlus]=feval(outputName,[],xPlus,xdnum, ...
        thrust,yawTorque,params,gam,sysnum,Pnum);
    [~,~,~,~,~,psiMinus]=feval(outputName,[],xMinus,xdnum, ...
        thrust,yawTorque,params,gam,sysnum,Pnum);
    bFD=(psiPlus-psiMinus)/(2*dtTest)+gam(4)*h4n;
    errB=abs(bn-bFD);
    worstB=max(worstB,errB);
    assert(errB<3e-3*max(1,abs(bn)), ...
        'QP定数項bが4階Lie微分と不一致（sample=%d）',sample);

    % E) 参考：u1 は一般に2階から効き始める（今回は最適化せず固定）
    vdir=Gnum(:,1); step=1e-5/max(1,norm(vdir,inf));
    [~,~,~,p2]=feval(outputName,[],xn+step*vdir,xdnum, ...
        thrust,yawTorque,params,gam,sysnum,Pnum);
    [~,~,~,m2]=feval(outputName,[],xn-step*vdir,xdnum, ...
        thrust,yawTorque,params,gam,sysnum,Pnum);
    if abs((p2-m2)/(2*step))>1e-7, nonzeroThrustSecond=nonzeroThrustSecond+1; end

    if sample==1
        fprintf('[SAMPLE] h1..h4 = [%.6g %.6g %.6g %.6g]\n',h1n,h2n,h3n,h4n);
        fprintf('[SAMPLE] A=[%.6g %.6g], b=%.6g\n',An(1),An(2),bn);
        fprintf('[SAMPLE] 4th-order Lg=[u2 %.6g, u3 %.6g, u4 %.6g]\n',numericLG4);
    end
end
fprintf('[TRACE] torque relative degree: orders 1,2,3 -> zero (u2,u3,u4); order 4 -> see below\n');
fprintf('[CHECK] order1..3 max |Lg psi0..2|: %.3e\n',worstLow);
fprintf('[CHECK] order4 xy nonzero states: %d/%d\n',nonzeroXY,numTests);
fprintf('[CHECK] order4 max |u4 coefficient|: %.3e\n',maxYaw);
fprintf('[CHECK] u1 appears in second derivative: %d/%d states\n', ...
    nonzeroThrustSecond,numTests);
fprintf('[CHECK] max |A + Lg_4(xy)|: %.3e\n',worstA);
fprintf('[CHECK] max |b - [L_f psi3 + gamma4 psi3]|: %.3e\n',worstB);
fprintf('[CHECK] max |h1 - geometric h|: %.3e\n',worstH);
assert(nonzeroXY>=ceil(0.75*numTests), ...
    '4階で水平トルク係数が非ゼロとなる状態が少なすぎます');
assert(nonzeroThrustSecond>=ceil(0.75*numTests), ...
    'u1の2階係数が非ゼロになるケースを十分に確認できません');
fprintf('\n[PASS] Generated MATLAB function passes built-in numerical audits.\n');
fprintf('[WARNING] 固定u1の仮定のみを検証。時変u1でのCBF安全性保証ではありません。\n');
fprintf('========== END ==========\n');

%% Local functions: generator一体型にして別のverify.mを必要としない。
function [shape,Sfun] = localMakeShape(R,r,L,cableRadius,margin,beta)
    % W, kappa を予め計算して生成関数内の3x3逆行列を回避。
    % 半長 L/2、膨張半径 rho のカプセルを保守的楕円体で外接する。
    assert(isequal(size(R),[3,3]) && norm(R.'*R-eye(3),'fro')<1e-8 ...
        && det(R)>0,'障害物回転行列はSO(3)にしてください');
    assert(all(r>0) && all(isfinite(r)) && L>0 && beta>0 ...
        && cableRadius>0 && margin>=0,'楕円体・紐パラメータが不正です');
    rho = cableRadius+margin;
    a = L/2+rho;
    b2 = rho*a;
    kappa = (1+beta)*(a*a-b2);
    Sobs=R*diag(r.^2)*R.';
    C=(1+1/beta)*Sobs+(1+beta)*b2*eye(3);
    W=C\eye(3);W=(W+W.')/2;
    shape=[W(1,1);W(2,2);W(3,3);W(1,2);W(1,3);W(2,3);kappa];
    Sfun=@(pT) C+kappa*(pT*pT.');
end

function P22 = localP22(P9)
    % 元の FL/GL 関数は質量・慣性等を位置 1,6,7,8,9,19..22 から受け取る。
    P22=zeros(1,22);
    P22([1,6,7,8,9,19,20,21,22])=P9;
end

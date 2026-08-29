% このスクリプトが置いてあるフォルダの絶対パスを取得して移動
mfile_path = fileparts(mfilename('fullpath'));
cd(mfile_path);

%% Define the nonlinear physical model of a quadrotor
syms p1 p2 p3 dp1 dp2 dp3 ddp1 ddp2 ddp3 q0 q1 q2 q3 o1 o2 o3 real
syms u u1 u2 u3 u4 T1 T2 T3 T4 real
syms m Lx Ly lx ly jx jy jz gravity km1 km2 km3 km4 k1 k2 k3 k4 real
%% Controller design
clc
syms xd1(t) xd2(t) xd3(t) xd4(t) v1(t)
syms t real
xd = [xd1(t),xd2(t),xd3(t),xd4(t)]; % reference を後で決める時はこっち
%% 
p	= [  p1;  p2;  p3];             % Position　：xb : 進行方向，zb ：ホバリング時に上向き
dp	= [ dp1; dp2; dp3];             % Velocity
ddp	= [ddp1;ddp2;ddp3];             % Accelaletion
q	= [  q0;  q1;  q2;  q3];        % Quaternion
ob	= [  o1;  o2;  o3];             % Angular velocity
x = [q;p;dp;ob];
physicalParam = [m, Lx, Ly, lx, ly, jx, jy, jz, gravity, km1, km2, km3, km4, k1, k2, k3, k4];
f = F(x,physicalParam);
g = G(x,physicalParam);
%g= [g1 g2 g3 g4];
%% 1st layer
clc
% % Define virtual output: h1
h1 = p3 - xd(3);
q	= [  q0;  q1;  q2;  q3];        % Quaternion
% % **************************************** % %
% LieD(h1,g,x) % For check 
dh1 = LieD(h1,f,x)+diff(h1,t);
alpha1 = LieD(dh1,f,x)+diff(dh1,t);
beta1 = simplify(LieD(dh1,g,x));
H = [1/beta1(1),-beta1(2)/beta1(1),-beta1(3)/beta1(1),-beta1(4)/beta1(1); zeros(3,1),eye(3)];
syms e1 e2 real
He = [1/(beta1(1)+e1),-beta1(2)/(beta1(1)+e1),-beta1(3)/(beta1(1)+e1),-beta1(4)/(beta1(1)+e1); zeros(3,1),eye(3)];
% H = pinv(beta1);
% nH = null(H');


% syms v1(t) dv1(t) ddv1(t) dddv1(t) ddddv1(t)
% dv1(t) = diff(v1(t),t);
% ddv1(t) = diff(v1(t),t,2);
% dddv1(t) = diff(v1(t),t,3);
% ddddv1(t) = diff(v1(t),t,4);
% Qz = diag([1.0,1.0]);               % Weight of state
% Rz = 0.1;                           % Weight of input
% Nz = 0;
% Fz = lqr([0,1;0,0], [0;1], Qz, Rz, Nz);
 %v1 = -Fz * [h1;dh1]; % v1を事前に決めておく時はこっち
%% 2nd layer
syms v2 v3 v4
FG = simplify(f+g*H*[-alpha1+v1(t);u2;u3;u4]);	% v1を後で設計する時はこっち
g1 = simplify(MyCoeff(FG,[u2;u3;u4]));
f1 = subs(FG,[u2,u3,u4],[0,0,0]);
%simplify(FG-(f1+g1*[v2;v3;v4]))   % For check

% FG = simplify(f+g*(H*(-alpha1+v1(t))+nH*[v2;v3;v4]));	% v1を後で設計する時はこっち
% g1 = simplify(MyCoeff(FG,[v2;v3;v4]));
% f1 = subs(FG,[v2,v3,v4],[0,0,0]);

%%
% % Define virtual output: h2, h3, h4
h2 = p1 - xd(1);
h3 = p2 - xd(2);
[~,~,yaw] = Quat2Eul(q);
h4 = yaw - xd(4);
%h4 = 2*(q0*q3 + q1*q2) - xd(4); % これでは姿勢は変わらない
%% **************************************** % %
clc
dh2 = LieD(h2,f1,x)+diff(h2,t);
dh3 = LieD(h3,f1,x)+diff(h3,t);
dh4 = LieD(h4,f1,x)+diff(h4,t);
ddh2 = LieD(dh2,f1,x)+diff(dh2,t);
ddh3 = LieD(dh3,f1,x)+diff(dh3,t);
dddh2 = LieD(ddh2,f1,x)+diff(ddh2,t);
dddh3 = LieD(ddh3,f1,x)+diff(ddh3,t);
% % For check
% 	[LieD(h2,g1,x),LieD(h3,g1,x)]
% 	simplify([LieD(dh2,g1,x),LieD(dh3,g1,x)])
% 	simplify([LieD(ddh2,g1,x),LieD(ddh3,g1,x)])
% 	[LieD(h2,g1,x),LieD(h3,g1,x),LieD(h4,g1,x)]
% 	simplify([LieD(dh2,g1,x),LieD(dh3,g1,x)])
% 	simplify([LieD(ddh2,g1,x),LieD(ddh3,g1,x)])
%% 
% % Derive 2nd layer controller
alpha2 = [LieD(dddh2,f1,x)+diff(dddh2,t); LieD(dddh3,f1,x)+diff(dddh3,t); LieD(dh4,f1,x)+diff(dh4,t)];
beta2 = [LieD(dddh2,g1,x); LieD(dddh3,g1,x); LieD(dh4,g1,x)];
% Qxy = diag([1.0, 1.0, 0.1, 0.01]);	% Weight of state
% Rxy = 1.0;                          % Weight of input
% Nxy = 0;
% Fxy = lqr([0,1,0,0;0,0,1,0;0,0,0,1;0,0,0,0], [0;0;0;1], Qxy, Rxy, Nxy);
% Qr = diag([1.0, 1.0]);              % Weight of state
% Rr = 1;                             % Weight of input
% Nr = 0;
% Fr = lqr([0,1;0,0], [0;1], Qr, Rr, Nr);
% if flag.predefinedReference
%     v2 = [-F4*[h2;dh2;ddh2;dddh2]; -Fxy*[h3;dh3;ddh3;dddh3]; -Fr*[h4;dh4]];
%     U2 = inv(beta2)*(-alpha2+v2);   % v2 を事前に設計しておく時はこっち
% else
    syms v2(t) v3(t) v4(t)
    U2 = inv(beta2)*(-alpha2+[v2(t);v3(t);v4(t)]);  % v2を後で設計する時はこっち
    %U2 = beta2/(-alpha2+[v2(t);v3(t);v4(t)]);  % v2を後で設計する時はこっち
    % U2e = (adjoint(beta2)/(det(beta2)+e2))*(-alpha2+[v2(t);v3(t);v4(t)]);  % v2を後で設計する時はこっち
%end
%% Initialize xd as an unspecified function of t
% % If regenerate Uf, Us or Xd functions, evaluate this section.
    xd = [xd1(t),xd2(t),xd3(t),xd4(t)];
    dxd = diff(xd,t);
    ddxd = diff(xd,t,2);
    dddxd = diff(xd,t,3);
    ddddxd = diff(xd,t,4);
%% Set variables for output functions
    syms Xd1 Xd2 Xd3 Xd4 dXd1 dXd2 dXd3 dXd4 ddXd1 ddXd2 ddXd3 ddXd4 dddXd1 dddXd2 dddXd3 dddXd4 ddddXd1 ddddXd2 ddddXd3 ddddXd4 real
    syms V1 V2 V3 V4 dV1 ddV1 dddV1 real
    XD = {Xd1 Xd2 Xd3 Xd4 dXd1 dXd2 dXd3 dXd4 ddXd1 ddXd2 ddXd3 ddXd4 dddXd1 dddXd2 dddXd3 dddXd4 ddddXd1 ddddXd2 ddddXd3 ddddXd4};
    XDf = fliplr(XD);
    V1v = {V1 dV1 ddV1 dddV1};
    V1vf = fliplr(V1v);
    xdRef = fliplr([xd dxd ddxd dddxd ddddxd]);
    vInput1 = fliplr([v1(t) diff(v1(t),t) diff(v1(t),t,2) diff(v1(t),t,3)]);

%% ========================================================================= %%
%% FastBridge 完全準拠: 衝突円錐 ECBF (動的拡大状態 z in R^14, 相対次数3)
%% ========================================================================= %%
fprintf('FastBridge Collision Cone ECBF 制約の導出を開始します...\n');

%% 1. 拡大状態量とパラメータの定義
syms aT daT d2aT real             % 正規化推力 aT=T/m, 1階微分, 2階微分 (決定変数)
syms taux tauy tauz real          % トルク入力 (決定変数)
syms mu1 mu2 mu3 real             % 障害物中心座標
syms A11 A12 A13 A22 A23 A33 real % 楕円共分散逆行列
syms c_scale real                 % 信頼区間スケール
syms lambda_cbf positive real     % ECBF 極配置パラメータ

e3 = [0; 0; 1];
J  = diag([jx, jy, jz]);

%% 2. 衝突円錐バリア関数 h(r, v) の定義 (論文 Eq. 4, 8, 10)
mu_vec = [mu1; mu2; mu3];
A_mat  = [A11, A12, A13; A12, A22, A23; A13, A23, A33];

r_vec     = mu_vec - p;
gamma_val = r_vec.' * A_mat * r_vec - c_scale^2;
beta_val  = dp.' * A_mat * dp;
delta_val = r_vec.' * A_mat * dp;

h_cc = simplify(beta_val * gamma_val - delta_val^2);

%% 3. 拡大状態系 z in R^14 における ドリフト f_ext(z) と 入力行列 G_ext(z)
% 拡大状態ベクトル: z = [q(4); p(3); dp(3); ob(3); aT; daT]
z_ext = [q; p; dp; ob; aT; daT];

% クォータニオン回転行列 R
R_mat = [q0^2+q1^2-q2^2-q3^2,   2*(q1*q2-q0*q3),       2*(q1*q3+q0*q2);
         2*(q1*q2+q0*q3),       q0^2-q1^2+q2^2-q3^2,   2*(q2*q3-q0*q1);
         2*(q1*q3-q0*q2),       2*(q2*q3+q0*q1),       q0^2-q1^2-q2^2+q3^2];
b3 = R_mat * e3;

% クォータニオン運動学
Omega_w = [  0, -o1, -o2, -o3;
            o1,   0,  o3, -o2;
            o2, -o3,   0,  o1;
            o3,  o2, -o1,   0];
q_dot = 0.5 * Omega_w * q;

% ダイナミクス展開
p_dot  = dp;
dp_dot = -gravity * e3 + aT * b3;

gyro_drift  = -J \ cross(ob, J * ob);
G_w_tau     = J \ eye(3);
aT_dot      = daT;
daT_drift   = sym(0);

f_ext = [q_dot;
         p_dot;
         dp_dot;
         gyro_drift;
         aT_dot;
         daT_drift];

% 入力 eta = [d2aT; taux; tauy; tauz] に対する入力行列
g_d2aT = [zeros(4,1); zeros(3,1); zeros(3,1); zeros(3,1); 0; 1];
g_tx   = [zeros(4,1); zeros(3,1); zeros(3,1); G_w_tau(:,1); 0; 0];
g_ty   = [zeros(4,1); zeros(3,1); zeros(3,1); G_w_tau(:,2); 0; 0];
g_tz   = [zeros(4,1); zeros(3,1); zeros(3,1); G_w_tau(:,3); 0; 0];
G_ext  = [g_d2aT, g_tx, g_ty, g_tz];

%% 4. Lie微分と相対次数3のECBF制約構築 (論文 Eq. 15)
Lfh  = simplify(jacobian(h_cc, z_ext) * f_ext);
Lf2h = simplify(jacobian(Lfh,  z_ext) * f_ext);
Lf3h = simplify(jacobian(Lf2h, z_ext) * f_ext);

LgLf2h = simplify(jacobian(Lf2h, z_ext) * G_ext);

% 不等式制約: A_cbf * eta <= b_cbf
A_qp = simplify(-LgLf2h);
b_qp = simplify(Lf3h + 3*lambda_cbf*Lf2h + 3*(lambda_cbf^2)*Lfh + (lambda_cbf^3)*h_cc);

%% 5. MATLAB 関数の生成・エクスポート
fprintf('MATLAB 関数を生成中...\n');
splat_params = [mu1; mu2; mu3; A11; A12; A13; A22; A23; A33; c_scale];

% 1. バリア関数値の計算
matlabFunction(h_cc, 'file', 'FastBridge_CBF_h.m', ...
    'vars', {z_ext, splat_params}, 'outputs', {'h_val'});

% 2. Lie微分の診断関数
matlabFunction(Lfh, Lf2h, Lf3h, 'file', 'FastBridge_CBF_Lie.m', ...
    'vars', {z_ext, splat_params, physicalParam}, 'outputs', {'Lfh', 'Lf2h', 'Lf3h'});

% 3. QP不等式制約行列 (A_cbf * eta <= b_cbf)
matlabFunction(A_qp, b_qp, 'file', 'FastBridge_CBF_QP.m', ...
    'vars', {z_ext, splat_params, physicalParam, lambda_cbf}, 'outputs', {'A_cbf', 'b_cbf'});

fprintf('====================================================\n');
fprintf('  [OK] FastBridge 完全準拠 ECBF 関数群 生成完了\n');
fprintf('====================================================\n\n');

%% ========================================================================= %%
%% 生成後 動作確認テスト & QPシミュレーション
%% ========================================================================= %%
fprintf('========================================\n');
fprintf('  生成済み FastBridge ECBF 関数の動作テスト\n');
fprintf('========================================\n');

% 1. 物理パラメータ設定 (m=0.461kg, 慣性モーメント)
m_val   = 0.461;
g_val   = 9.81;
jx_val  = 2.5e-3; jy_val = 2.5e-3; jz_val = 4.0e-3;
phys_test = [m_val, 0.1, 0.1, 0.05, 0.05, jx_val, jy_val, jz_val, g_val, ...
             1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0];

% 2. クアッドローター拡大状態 z_test (14次元): 前方(+x)へ 3.0 m/s で直進
q_test   = [1; 0; 0; 0];
p_test   = [0.0; 0.0; 1.0];
dp_test  = [3.0; 0.0; 0.0];
ob_test  = [0.0; 0.0; 0.0];
aT_test  = g_val;                 % ホバリング推力 (T/m = g)
daT_test = 0.0;
z_test   = [q_test; p_test; dp_test; ob_test; aT_test; daT_test];

% 3. 3DGS 障害物 (3.0m 前方に配置)
mu_test = [3.0; 0.0; 1.0];
Sigma_inv = diag([1/(0.5^2), 1/(0.3^2), 1/(0.8^2)]);
A_test = [Sigma_inv(1,1); Sigma_inv(1,2); Sigma_inv(1,3); ...
          Sigma_inv(2,2); Sigma_inv(2,3); Sigma_inv(3,3)];
c_scale_test = 1.0;
splat_test = [mu_test; A_test; c_scale_test];

% 4. 制御ゲイン
lambda_test = 3.0;

% 5. 関数の評価
h_val = FastBridge_CBF_h(z_test, splat_test);
[Lfh_val, Lf2h_val, Lf3h_val] = FastBridge_CBF_Lie(z_test, splat_test, phys_test);
[A_cbf_val, b_cbf_val] = FastBridge_CBF_QP(z_test, splat_test, phys_test, lambda_test);

fprintf('  バリア関数値 h_cc        : % .4f\n', double(h_val));
fprintf('  Lf h, Lf^2 h, Lf^3 h      : [% .2e, % .2e, % .2e]\n', double(Lfh_val), double(Lf2h_val), double(Lf3h_val));
fprintf('  A_cbf 行列サイズ         : [%d x %d] (期待値: [1 x 4])\n', size(A_cbf_val, 1), size(A_cbf_val, 2));
fprintf('  A_cbf (d2aT, tx, ty, tz)  : [% .3e, % .3e, % .3e, % .3e]\n', double(A_cbf_val));
fprintf('  b_cbf の値               : % .3e\n', double(b_cbf_val));

% 6. QP 検証 (quadprog): ノミナル入力 eta_nom に対するフィルタリング
eta_nom = [0.0; 0.0; 0.0; 0.0];
H_qp = eye(4);
f_qp = -eta_nom;

lb = [-50.0; -0.5; -0.5; -0.1];
ub = [ 50.0;  0.5;  0.5;  0.1];

opts = optimoptions('quadprog', 'Display', 'off');
[eta_opt, ~, exitflag] = quadprog(H_qp, f_qp, double(A_cbf_val), double(b_cbf_val), [], [], lb, ub, [], opts);

if exitflag == 1
    fprintf('\n  [OK] QP Safety Filter 最適化成功\n');
    fprintf('  最適修正入力 eta*:\n');
    fprintf('    d2aT  (推力加加速度) : % .4f m/s^4\n', eta_opt(1));
    fprintf('    tau_x (ロールトルク) : % .4f Nm\n', eta_opt(2));
    fprintf('    tau_y (ピッチトルク) : % .4f Nm\n', eta_opt(3));
    fprintf('    tau_z (ヨートルク)   : % .4f Nm\n', eta_opt(4));
    fprintf('========================================\n\n');
else
    warning('QPが正常に終了しませんでした (exitflag = %d)', exitflag);
end
% %% Make functions of z
% % % If either model, virtual output or parameters is changed, then evaluate this section.
%     disp("Start: make functions of virtual states.");
%     matlabFunction(subs([h1;dh1], [xdRef], [XDf]),'file','Z1.m','vars',{x cell2sym(XD) physicalParam},'outputs',{'cZ1'});
%     matlabFunction(subs([h2;dh2;ddh2;dddh2], [xdRef vInput1], [XDf V1vf]),'file','Z2.m','vars',{x cell2sym(XD) cell2sym(V1v) physicalParam},'outputs',{'cZ2'});
%     matlabFunction(subs([h3;dh3;ddh3;dddh3], [xdRef vInput1], [XDf V1vf]),'file','Z3.m','vars',{x cell2sym(XD) cell2sym(V1v) physicalParam},'outputs',{'cZ3'});
%     matlabFunction(subs([h4;dh4], [xdRef vInput1], [XDf V1vf]),'file','Z4.m','vars',{x cell2sym(XD) cell2sym(V1v) physicalParam},'outputs',{'cZ4'});
% 
% %% Make functions of virtual inputs
% clc
%     disp("Start: make functions of virtual inputs.");
%     clear dt
%     syms f11 f12 f21 f22 f23 f24 f31 f32 f33 f34 f41 f42 dt k real
%     F1 = [f11 f12];
%     F2 = [f21 f22 f23 f24];
%     F3 = [f31 f32 f33 f34];
%     F4 = [f41 f42];
%     A1=[0,1;0,0]-[0;1]*F1; % closed loop : continuous
%     matlabFunction(subs([-F1*[h1;dh1],-F1*A1*[h1;dh1],-F1*A1*A1*[h1;dh1],-F1*A1*A1*A1*[h1;dh1]], [xdRef], [XDf]),'file','Vf.m','vars',{x cell2sym(XD) physicalParam F1},'outputs',{'V1'});
%     matlabFunction(subs([-F2*[h2;dh2;ddh2;dddh2],-F3*[h3;dh3;ddh3;dddh3],-F4*[h4;dh4]], [xdRef vInput1], [XDf V1vf]),'file','Vs.m','vars',{x cell2sym(XD) cell2sym(V1v) physicalParam F2 F3 F4},'outputs',{'cV2'});
%     %%
% 
%     A1 = expm([0,1;0,0]*dt)-int(expm([0,1;0,0]*(dt-k))*[0;1],k,[0,dt])*F1; % closed loop discrete
%     matlabFunction(subs([-F1*[h1;dh1],-F1*A1*[h1;dh1],-F1*A1*A1*[h1;dh1],-F1*A1*A1*A1*[h1;dh1]], [xdRef], [XDf]),'file','Vfd.m','vars',{dt x cell2sym(XD) physicalParam F1},'outputs',{'V1'});
%     A2 = expm(diag([1,1,1],1)*dt)-int(expm(diag([1,1,1],1)*(dt-k))*[0;0;0;1],k,[0,dt])*F2; % closed loop discrete
%     A3 = expm(diag([1,1,1],1)*dt)-int(expm(diag([1,1,1],1)*(dt-k))*[0;0;0;1],k,[0,dt])*F3; % closed loop discrete
%     A4 = expm([0 1;0 0]*dt)-int(expm([0 1;0 0]*(dt-k))*[0;1],k,[0,dt])*F4; % closed loop discrete
%     matlabFunction(subs([-F2*[h2;dh2;ddh2;dddh2],-F3*[h3;dh3;ddh3;dddh3],-F4*[h4;dh4]], [xdRef vInput1], [XDf V1vf]),'file','Vsd.m','vars',{dt x cell2sym(XD) cell2sym(V1v) physicalParam F2 F3 F4},'outputs',{'cV2'});
% 
%     % % For check
% %     Vf(0,x0,Xd(0))
% %     Vs(0,x0,Xd(0),Vf(0,x0,Xd(0)))
% %% Make functions of actual inputs taking t, x, xd, v1 and v2 as arguments
% % % If either model, virtual output or parameters is changed, then evaluate this section. It'll take few minutes.
% % % Usage: u = Uf(...) + Us(...)
%     matlabFunction(subs(H(:,1)*(-alpha1+v1(t)), [xdRef vInput1], [XDf V1vf]),'file','Uf.m','vars',{x cell2sym(XD) cell2sym(V1v) physicalParam},'outputs',{'U1'});
%     matlabFunction(subs(H(:,2:4)*U2, [xdRef vInput1 v2(t) v3(t) v4(t)], [XDf V1vf [V2 V3 V4]]),'file','Us.m','vars',{x cell2sym(XD) cell2sym(V1v) [V2;V3;V4] physicalParam},'outputs',{'U2'});
%     %matlabFunction(subs(He(:,1)*(-alpha1+v1(t)), [xdRef vInput1], [XDf V1vf]),'file','Ufe.m','vars',{t x cell2sym(XD) cell2sym(V1v) [physicalParam,e1,e2]},'outputs',{'cU1'});
%     %matlabFunction(subs(He(:,2:4)*U2e, [xdRef vInput1 v2(t) v3(t) v4(t)], [XDf V1vf [V2 V3 V4]]),'file','Use.m','vars',{t x cell2sym(XD) cell2sym(V1v) [V2;V3;V4] [physicalParam,e1,e2]},'outputs',{'cU2'});
% % % For check
% %     Uf(0,x0,Xd(0),Vf(0,x0,Xd(0)))
% %     Us(0,x0,Xd(0),Vf(0,x0,Xd(0)),Vs(0,x0,Xd(0),Vf(0,x0,Xd(0))))
% % %%
% % matlabFunction(subs(alpha1, [xdRef], [XDf]),'file','alpha1.m','vars',{cell2sym(XD) physicalParam},'outputs',{'al1'});
% % matlabFunction(subs(alpha2, [xdRef vInput1], [XDf V1vf]),'file','alpha2.m','vars',{t x cell2sym(XD) cell2sym(V1v) physicalParam},'outputs',{'al2'});
% % %%
% % matlabFunction(subs(beta2, [xdRef vInput1], [XDf V1vf]),'file','beta2.m','vars',{t x cell2sym(XD) cell2sym(V1v) physicalParam},'outputs',{'be2'});
% % %%
% % matlabFunction(subs(He, [xdRef vInput1], [XDf V1vf]),'file','He.m','vars',{t x cell2sym(XD) cell2sym(V1v) e1 physicalParam},'outputs',{'mat'});
% % %%
% % matlabFunction(beta1,'file','beta1.m','vars',{t x physicalParam},'outputs',{'beta1'});

%% Local functions
function pd = pdiff(flist, vars)
% % flistをvarsで微分したもののシンボリック配列を返す？
    flist = flist(:);
    vars = vars(:);
%    pd = arrayfun(@(x) diff(flist(1), x), vars)';
    pdiff_tmp = @(flist, vars) arrayfun(@(func) arrayfun(@(x) diff(func, x), vars)',flist,'UniformOutput',false);
    pd = cell2sym(pdiff_tmp(flist,vars));
end
function dh = LieD(h,f,x)
    x = x(:);
    dh = pdiff(h,x)*f;
end
function mat = MyCoeff(M,vars)
    vars = vars(:);
    mat = coder.nullcopy(sym(zeros(length(M),length(vars))));
    for i = 1:length(M)
        for j = 1:length(vars)
            mat(i,j) = subs(M(i)-subs(M(i),vars(j),0),vars(j),1);
        end
    end
end
function [eul,th,psi] = Quat2Eul(q,varargin)
% % phi...roll, th...pitch, psi...yaw
    switch nargin
        case 1
            q0 = q(1);
            q1 = q(2);
            q2 = q(3);
            q3 = q(4);
        case 4
            q0 = q;
            q1 = varargin{1};
            q2 = varargin{2};
            q3 = varargin{3};
    end
	phi = atan2((2*(q0*q1 + q2*q3)),(q0^2 - q1^2 - q2^2 + q3^2));
	th  = asin(2*(q0*q2 - q1*q3));
	psi = atan2((2*(q0*q3 + q1*q2)),(q0^2 + q1^2 - q2^2 - q3^2));
	switch  nargout
        case 1
		eul = [phi, th, psi];
        case 3
		eul = phi;
	end
end
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

% %% =========================================================================
% %% FastBridge 論文完全準拠: 単機クアッドロータ 衝突円錐 ECBF (相対次数3)
% %% Target: A_cbf * [T; taux; tauy; tauz] <= b_cbf
% %% =========================================================================
% disp("==============================================================");
% disp(" Start: FastBridge 衝突円錐 ECBF (相対次数3) 厳密導出");
% disp(" Target: A_cbf * u <= b_cbf");
% disp("==============================================================");
% 
% % 1. シンボリック変数の定義
% syms q0 q1 q2 q3 real               % クォータニオン
% syms px py pz real                   % 位置
% syms vx vy vz real                   % 速度
% syms wx wy wz real                   % 角速度
% syms m jx jy jz gravity real         % 物理定数
% 
% syms T_curr taux tauy tauz real      % 最適化決定変数 u
% syms T_prev1 T_prev2 dt real         % カスケード推力履歴と刻み幅
% syms mu1 mu2 mu3 real                % 障害物中心
% syms A11 A12 A13 A22 A23 A33 real    % 楕円逆共分散行列 (球なら A = eye(3)/ro^2)
% syms c_scale lambda_cbf real         % スケールとHurwitzゲイン
% 
% q = [q0; q1; q2; q3];
% p = [px; py; pz];
% v = [vx; vy; vz];
% w = [wx; wy; wz];
% J = diag([jx; jy; jz]);
% invJ = diag([1/jx, 1/jy, 1/jz]);
% e3 = [0; 0; 1];
% 
% u_vec = [T_curr; taux; tauy; tauz];
% tau_m = [taux; tauy; tauz];
% 
% % 2. 幾何・運動学モデル (クォータニオン回転行列 R)
% R = [q0^2+q1^2-q2^2-q3^2,   2*(q1*q2-q0*q3),       2*(q1*q3+q0*q2);
%      2*(q1*q2+q0*q3),       q0^2-q1^2+q2^2-q3^2,   2*(q2*q3-q0*q1);
%      2*(q1*q3-q0*q2),       2*(q2*q3+q0*q1),       q0^2-q1^2-q2^2+q3^2];
% b3 = R * e3;
% 
% hat_e3 = [0, -1, 0; 1, 0, 0; 0, 0, 0];
% hat_w  = [0, -w(3), w(2); w(3), 0, -w(1); -w(2), w(1), 0];
% 
% % -------------------------------------------------------------------------
% % 3. 論文準拠の相対ベクトルとバリア関数 (r = mu - p)
% % -------------------------------------------------------------------------
% mu = [mu1; mu2; mu3];
% A  = [A11, A12, A13;
%       A12, A22, A23;
%       A13, A23, A33];
% 
% % 🌟 論文準拠: 自機から障害物へのベクトル
% r = mu - p; 
% 
% tau_look = 0.5; % 予測時間 [s]
% % 接近中 (r'*A*v > 0) に急速に小さくなる安全マージン関数
% h_cc = simplify(r.' * A * r - c_scale^2 - 2 * tau_look * (r.' * A * v));
% 
% % -------------------------------------------------------------------------
% % 4. 基礎時間微分 (ベース推力 aT_base = T_prev1 / m)
% % -------------------------------------------------------------------------
% aT_base = T_prev1 / m;
% acc_base = -gravity * e3 + aT_base * b3;
% dw_drift = -invJ * cross(w, J * w);
% 
% db3 = simplify(-R * hat_e3 * w);
% ddb3_drift = simplify(-R * hat_e3 * dw_drift + R * (hat_w * hat_w) * e3);
% 
% dh_dt  = simplify(jacobian(h_cc, [p; v]) * [v; acc_base]);
% ddh_dt = simplify(jacobian(dh_dt, [p; v; w]) * [v; acc_base; dw_drift]);
% 
% % -------------------------------------------------------------------------
% % 5. スナップ感度と有限差分モデル
% % -------------------------------------------------------------------------
% % dr/dt = -v, dv/dt = a, da/dt = j, dj/dt = snap より、rの微分は負
% w_snap = simplify(- 2 * tau_look * (A * r));
% 
% G_T_FD = simplify((1 / (m * dt^2)) * b3 + (2 / (m * dt)) * db3);
% G_tau  = simplify(-(T_prev1 / m) * R * hat_e3 * invJ);
% 
% F_snap_drift = simplify( ...
%     (T_prev1 / m) * ddb3_drift ...
%   - (2 * T_prev1 / (m * dt)) * db3 ...
%   + ((-2 * T_prev1 + T_prev2) / (m * dt^2)) * b3 ...
% );
% 
% Phi_drift = simplify(jacobian(ddh_dt, [p; v; w]) * [v; acc_base; dw_drift]);
% 
% Lg_T   = simplify(w_snap.' * G_T_FD);
% Lg_tau = simplify(w_snap.' * G_tau);
% drift_h3 = simplify(w_snap.' * F_snap_drift + Phi_drift);
% 
% % -------------------------------------------------------------------------
% % 6. 相対次数3 ECBF 制約の構築 (標準形: -Lg*u <= drift)
% % -------------------------------------------------------------------------
% ecbf_drift = simplify( ...
%     drift_h3 + ...
%     3 * lambda_cbf * ddh_dt + ...
%     3 * (lambda_cbf^2) * dh_dt + ...
%     (lambda_cbf^3) * h_cc ...
% );
% 
% % 🌟 標準形式 (psi_3 >= 0 <===> A_cbf * u <= b_cbf)
% A_cbf_sym = simplify([-Lg_T, -Lg_tau]);
% b_cbf_sym = simplify(ecbf_drift);
% 
% % 7. MATLAB 関数としてエクスポート
% z_base = [q; p; v; w];
% splat_params = [mu1; mu2; mu3; A11; A12; A13; A22; A23; A33; c_scale];
% phys_params  = [m; jx; jy; jz; gravity];
% 
% matlabFunction( ...
%     A_cbf_sym, ...
%     b_cbf_sym, ...
%     h_cc, ...
%     dh_dt, ...
%     'file', 'FastBridge_SingleDrone_ECBF.m', ...
%     'vars', {z_base, splat_params, phys_params, lambda_cbf, T_prev1, T_prev2, dt}, ...
%     'outputs', {'A_cbf', 'b_cbf', 'h0', 'dh'} ...
% );
% 
% disp("==============================================================");
% disp(" Done: 論文完全準拠 FastBridge_SingleDrone_ECBF.m 生成完了！");
% disp("==============================================================");
% %% =========================================================================
% %% [Verification Test] 論文完全準拠 単機 ECBF 検証テスト
% %% =========================================================================
% fprintf('\n==============================================================\n');
% fprintf(' [Verification Test] 単機 ECBF 生成関数の厳密検証テストを開始します\n');
% fprintf('==============================================================\n');
% 
% % 物理パラメータ
% m_t = 0.461; g_t = 9.81;
% jx_t = 2.5e-3; jy_t = 2.5e-3; jz_t = 4.0e-3;
% phys_t = [m_t; jx_t; jy_t; jz_t; g_t];
% 
% % 状態: 前方(+X)へ 3.0 m/s で直進飛行中
% q_t = [1; 0; 0; 0];
% p_t = [0.0; 0.0; 1.0];
% v_t = [3.0; 0.0; 0.0];
% w_t = [0.0; 0.0; 0.0];
% z_t = [q_t; p_t; v_t; w_t];
% 
% T_hov = m_t * g_t;
% dt_t = 0.02;
% lambda_t = 4.0;
% u_nom_t = [T_hov; 0.0; 0.0; 0.0];
% 
% % --- 1. シンボリック感度解析 ---
% fprintf('--- 1. シンボリック感度 J_u の構造解析 (各入力への結合確認) ---\n');
% sens_u1 = ~isequal(diff(A_cbf_sym(1), T_curr), sym(0)) || ~isequal(A_cbf_sym(1), sym(0));
% sens_u2 = ~isequal(A_cbf_sym(2), sym(0));
% sens_u3 = ~isequal(A_cbf_sym(3), sym(0));
% sens_u4 = ~isequal(A_cbf_sym(4), sym(0));
% 
% fprintf('  d(psi)/du1 (推力 T 感度)         : %s\n', mat2str(sens_u1));
% fprintf('  d(psi)/du2 (Rollトルク taux 感度) : %s\n', mat2str(sens_u2));
% fprintf('  d(psi)/du3 (Pitchトルク tauy 感度): %s\n', mat2str(sens_u3));
% fprintf('  d(psi)/du4 (Yawトルク tauz 感度)   : %s\n', mat2str(sens_u4));
% 
% % --- 2. 数値テスト ---
% fprintf('\n--- 2. 数値テスト環境による代数符号＆感度の評価 ---\n');
% 
% % 障害物: 半径 0.5m の球 (A = eye(3)/0.5^2)
% ro_t = 0.5;
% A_sph = diag([1/ro_t^2, 1/ro_t^2, 1/ro_t^2]);
% A_flat = [A_sph(1,1); A_sph(1,2); A_sph(1,3); A_sph(2,2); A_sph(2,3); A_sph(3,3)];
% 
% % Case A: 安全 (障害物が後方 X = -3.0m にあり、前進して離脱中)
% mu_safe = [-3.0; 0.0; 1.0];
% splat_safe = [mu_safe; A_flat; 1.0];
% [A_safe, b_safe, h0_safe, ~] = FastBridge_SingleDrone_ECBF(...
%     z_t, splat_safe, phys_t, lambda_t, T_hov, T_hov, dt_t);
% A_safe = double(A_safe); b_safe = double(b_safe);
% viol_safe = A_safe * u_nom_t - b_safe;
% 
% fprintf('  [Case A: 安全 (障害物が後方にあり衝突コース外)]\n');
% fprintf('    h_cc (衝突円錐マージン): %+7.4e (正であるべき)\n', double(h0_safe));
% fprintf('    A_cbf (感度 1x4)       : [%+.2e, %+.2e, %+.2e, %+.2e]\n', A_safe);
% fprintf('    b_cbf (ドリフト)       : %+.4e\n', b_safe);
% fprintf('    A*u_nom - b            : %+.4e (負であるべき: 安全合格)\n\n', viol_safe);
% 
% % Case B: 危険 (前方 X = 1.5m に障害物があり、3.0 m/s で正面衝突コース突入中)
% mu_dang = [1.5; 0.0; 1.0];
% splat_dang = [mu_dang; A_flat; 1.0];
% [A_dang, b_dang, h0_dang, ~] = FastBridge_SingleDrone_ECBF(...
%     z_t, splat_dang, phys_t, lambda_t, T_hov, T_hov, dt_t);
% A_dang = double(A_dang); b_dang = double(b_dang);
% viol_dang = A_dang * u_nom_t - b_dang;
% 
% fprintf('  [Case B: 衝突コース突入状態 (前方1.5mに突進中)]\n');
% fprintf('    h_cc (衝突円錐マージン): %+7.4e (負であるべき: 衝突円錐内)\n', double(h0_dang));
% fprintf('    A_cbf (感度 1x4)       : [%+.2e, %+.2e, %+.2e, %+.2e]\n', A_dang);
% fprintf('    b_cbf (ドリフト)       : %+.4e\n', b_dang);
% fprintf('    A*u_nom - b            : %+.4e (正であるべき: 制約違反検知)\n\n', viol_dang);
% 
% % --- 3. 判定 ---
% if viol_safe <= 0 && viol_dang > 0 && sens_u1 && (sens_u2 || sens_u3)
%     fprintf('  [PASS] 論文完全準拠の感度結合と符号整合性を確認しました！\n');
% else
%     fprintf('  [FAIL] 符号または感度を確認してください。\n');
% end
% fprintf('==============================================================\n');
%% =========================================================================
%% FastBridge 準拠: 単機クアッドロータ 衝突円錐 ECBF (相対次数3・独立ゲイン版)
%% Target: A_cbf * [T; taux; tauy; tauz] <= b_cbf
%% =========================================================================
clear;
clc;
disp("==============================================================");
disp(" Start: FastBridge 衝突円錐 ECBF (相対次数3・独立ゲイン) 厳密導出");
disp(" Target: A_cbf * u <= b_cbf");
disp("==============================================================");

% 1. シンボリック変数の定義
syms q0 q1 q2 q3 real               % クォータニオン
syms px py pz real                   % 位置
syms vx vy vz real                   % 速度
syms wx wy wz real                   % 角速度
syms m jx jy jz gravity real         % 物理定数

syms T_curr taux tauy tauz real      % 最適化決定変数 u
syms T_prev1 T_prev2 dt real         % カスケード推力履歴と刻み幅
syms mu1 mu2 mu3 real                % 障害物中心
syms A11 A12 A13 A22 A23 A33 real    % 楕円逆共分散行列 (球なら A = eye(3)/ro^2)
syms c_scale real                    % 信頼区間スケール

% 🌟 独立ゲインパラメータ [k0:位置, k1:速度, k2:加速度]
syms k0 k1 k2 real
cbf_gains = [k0; k1; k2];

q = [q0; q1; q2; q3];
p = [px; py; pz];
v = [vx; vy; vz];
w = [wx; wy; wz];
J = diag([jx; jy; jz]);
invJ = diag([1/jx, 1/jy, 1/jz]);
e3 = [0; 0; 1];

u_vec = [T_curr; taux; tauy; tauz];
tau_m = [taux; tauy; tauz];

% 2. 幾何・運動学モデル (クォータニオン回転行列 R)
R = [q0^2+q1^2-q2^2-q3^2,   2*(q1*q2-q0*q3),       2*(q1*q3+q0*q2);
     2*(q1*q2+q0*q3),       q0^2-q1^2+q2^2-q3^2,   2*(q2*q3-q0*q1);
     2*(q1*q3-q0*q2),       2*(q2*q3+q0*q1),       q0^2-q1^2-q2^2+q3^2];
b3 = R * e3;

hat_e3 = [0, -1, 0; 1, 0, 0; 0, 0, 0];
hat_w  = [0, -w(3), w(2); w(3), 0, -w(1); -w(2), w(1), 0];

% 3. 論文準拠の相対ベクトルとバリア関数 (r = mu - p)
mu = [mu1; mu2; mu3];
A  = [A11, A12, A13;
      A12, A22, A23;
      A13, A23, A33];

% 自機から障害物へのベクトル
r = mu - p; 

% [FIX] FastBridge 論文 Eq(10) に準拠した真の Collision Cone CBF
% h(r,v) = (v^T A v)(r^T A r - c^2) - (r^T A v)^2 >= 0
beta_term  = v.' * A * v;
gamma_term = r.' * A * r - c_scale^2;
delta_term = r.' * A * v;
h_cc = simplify(beta_term * gamma_term - delta_term^2);

% 4. 基礎時間微分 (ベース推力 aT_base = T_prev1 / m)
aT_base = T_prev1 / m;
acc_base = -gravity * e3 + aT_base * b3;
dw_drift = -invJ * cross(w, J * w);

db3 = simplify(-R * hat_e3 * w);
ddb3_drift = simplify(-R * hat_e3 * dw_drift + R * (hat_w * hat_w) * e3);

dh_dt  = simplify(jacobian(h_cc, [p; v]) * [v; acc_base]);
ddh_dt = simplify(jacobian(dh_dt, [p; v; w]) * [v; acc_base; dw_drift]);

% 5. スナップ感度と有限差分モデル
% w_snap は nabla_v(h_cc)
w_snap = simplify(2 * gamma_term * (A * v) - 2 * delta_term * (A * r));

G_T_FD = simplify((1 / (m * dt^2)) * b3 + (2 / (m * dt)) * db3);
G_tau  = simplify(-(T_prev1 / m) * R * hat_e3 * invJ);

F_snap_drift = simplify( ...
    (T_prev1 / m) * ddb3_drift ...
  - (2 * T_prev1 / (m * dt)) * db3 ...
  + ((-2 * T_prev1 + T_prev2) / (m * dt^2)) * b3 ...
);

Phi_drift = simplify(jacobian(ddh_dt, [p; v; w]) * [v; acc_base; dw_drift]);

Lg_T   = simplify(w_snap.' * G_T_FD);
Lg_tau = simplify(w_snap.' * G_tau);
drift_h3 = simplify(w_snap.' * F_snap_drift + Phi_drift);

% 6. 独立ゲイン版 相対次数3 ECBF 制約の構築
% psi_3 = h^(3) + k2*ddh + k1*dh + k0*h >= 0
ecbf_drift = simplify( ...
    drift_h3 + ...
    k2 * ddh_dt + ...
    k1 * dh_dt + ...
    k0 * h_cc ...
);

% 標準形式: A_cbf * u <= b_cbf  <===>  - (Lg_T*T + Lg_tau*tau) <= ecbf_drift
A_cbf_sym = simplify([-Lg_T, -Lg_tau]);
b_cbf_sym = simplify(ecbf_drift);

% 7. MATLAB 関数のエクスポート
z_base = [q; p; v; w];
splat_params = [mu1; mu2; mu3; A11; A12; A13; A22; A23; A33; c_scale];
phys_params  = [m; jx; jy; jz; gravity];

matlabFunction( ...
    A_cbf_sym, ...
    b_cbf_sym, ...
    h_cc, ...
    dh_dt, ...
    'file', 'FastBridge_SingleDrone_ECBF.m', ...
    'vars', {z_base, splat_params, phys_params, cbf_gains, T_prev1, T_prev2, dt}, ...
    'outputs', {'A_cbf', 'b_cbf', 'h0', 'dh'} ...
);

disp("==============================================================");
disp(" Done: FastBridge_SingleDrone_ECBF.m 生成完了！");
disp("==============================================================");

%% =========================================================================
%% [Verification Test] 独立ゲイン版 単機 ECBF 検証テスト
%% =========================================================================
fprintf('\n==============================================================\n');
fprintf(' [Verification Test] 単機 ECBF 生成関数の厳密検証テストを開始します\n');
fprintf('==============================================================\n');

% 物理パラメータ (単機クアッドロータ)
m_t = 0.461; g_t = 9.81;
jx_t = 2.5e-3; jy_t = 2.5e-3; jz_t = 4.0e-3;
phys_t = [m_t; jx_t; jy_t; jz_t; g_t];

% 状態: 前方(+X)へ 3.0 m/s で直進突進中
q_t = [1; 0; 0; 0];
p_t = [0.0; 0.0; 1.0];
v_t = [3.0; 0.0; 0.0];
w_t = [0.0; 0.0; 0.0];
z_t = [q_t; p_t; v_t; w_t];

T_hov = m_t * g_t;
dt_t = 0.02;

% 🌟 独立ゲイン設計:
%   k0: 位置反発 (高階ドリフトを凌駕し衝突を確実に検知)
%   k1: 速度制動
%   k2: 加速度抑制 (過剰な正の自乗ドリフト爆縮を防止)
k0_t = 80.0;
k1_t = 10.0;
k2_t = 2.0;
gains_t = [k0_t; k1_t; k2_t];

% ノミナル入力 (ホバリング直進指示: トルクゼロ)
u_nom_t = [T_hov; 0.0; 0.0; 0.0];

% --- 1. シンボリック感度解析 ---
fprintf('--- 1. シンボリック感度 J_u の構造解析 (各入力への結合確認) ---\n');
sens_u1 = ~isequal(A_cbf_sym(1), sym(0));
sens_u2 = ~isequal(A_cbf_sym(2), sym(0));
sens_u3 = ~isequal(A_cbf_sym(3), sym(0));
sens_u4 = ~isequal(A_cbf_sym(4), sym(0));

fprintf('  d(psi)/du1 (推力 T 感度)         : %s\n', mat2str(sens_u1));
fprintf('  d(psi)/du2 (Rollトルク taux 感度) : %s\n', mat2str(sens_u2));
fprintf('  d(psi)/du3 (Pitchトルク tauy 感度): %s\n', mat2str(sens_u3));
fprintf('  d(psi)/du4 (Yawトルク tauz 感度)   : %s\n', mat2str(sens_u4));

% --- 2. 数値テスト ---
fprintf('\n--- 2. 数値テスト環境による代数符号＆感度の評価 ---\n');

% 障害物: 半径 0.5m の球 (A = eye(3)/0.5^2)
ro_t = 0.5;
A_sph = diag([1/ro_t^2, 1/ro_t^2, 1/ro_t^2]);
A_flat = [A_sph(1,1); A_sph(1,2); A_sph(1,3); A_sph(2,2); A_sph(2,3); A_sph(3,3)];

% Case A: 安全 (障害物が後方 X = -3.0m にあり離脱中)
mu_safe = [-3.0; 0.0; 1.0];
splat_safe = [mu_safe; A_flat; 1.0];
[A_safe, b_safe, h0_safe, ~] = FastBridge_SingleDrone_ECBF(...
    z_t, splat_safe, phys_t, gains_t, T_hov, T_hov, dt_t);
A_safe = double(A_safe); b_safe = double(b_safe);
viol_safe = A_safe * u_nom_t - b_safe;

fprintf('  [Case A: 安全 (障害物が後方にあり衝突コース外)]\n');
fprintf('    h_cc (衝突円錐マージン): %+7.4e (正であるべき)\n', double(h0_safe));
fprintf('    A_cbf (感度 1x4)       : [%+.2e, %+.2e, %+.2e, %+.2e]\n', A_safe);
fprintf('    b_cbf (ドリフト)       : %+.4e\n', b_safe);
fprintf('    A*u_nom - b            : %+.4e (負であるべき: 安全合格)\n\n', viol_safe);

% Case B: 危険 (前方 X = 1.5m に障害物があり、3.0 m/s で突入中)
mu_dang = [1.5; 0.0; 1.0];
splat_dang = [mu_dang; A_flat; 1.0];
[A_dang, b_dang, h0_dang, ~] = FastBridge_SingleDrone_ECBF(...
    z_t, splat_dang, phys_t, gains_t, T_hov, T_hov, dt_t);
A_dang = double(A_dang); b_dang = double(b_dang);
viol_dang = A_dang * u_nom_t - b_dang;

fprintf('  [Case B: 衝突コース突入状態 (前方1.5mに突進中)]\n');
fprintf('    h_cc (衝突円錐マージン): %+7.4e (負であるべき: 衝突コース内)\n', double(h0_dang));
fprintf('    A_cbf (感度 1x4)       : [%+.2e, %+.2e, %+.2e, %+.2e]\n', A_dang);
fprintf('    b_cbf (ドリフト)       : %+.4e\n', b_dang);
fprintf('    A*u_nom - b            : %+.4e (正であるべき: 制約違反検知)\n\n', viol_dang);

% --- 3. 符号整合性の自動判定結果 ---
fprintf('--- 3. 符号整合性の自動判定結果 ---\n');
test_pass = true;

if viol_safe > 0
    warning('【符号異常】Case A: 安全圏なのに A*u - b > 0 (制約違反) と判定されています。');
    test_pass = false;
end

if viol_dang <= 0
    warning('【符号異常】Case B: 衝突コースなのに A*u - b <= 0 (安全) と誤判定されています。');
    test_pass = false;
end

if test_pass && sens_u1 && (sens_u2 || sens_u3)
    fprintf('  [PASS] すべてのテストに合格しました！\n');
    fprintf('         - トルクおよび推力へのヤコビアン結合を確認\n');
    fprintf('         - 安全時に負 (マージン内)、衝突突入時に正 (制約違反) の符号整合性を確認\n');
else
    fprintf('  [FAIL] 判定に不整合が残っています。\n');
end
fprintf('==============================================================\n');
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
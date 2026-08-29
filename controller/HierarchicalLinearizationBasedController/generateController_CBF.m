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

% %% ========================================================================= %%
% %% FastBridge 完全準拠: 衝突円錐 ECBF (動的拡大状態 z in R^14, 相対次数3)
% %% ========================================================================= %%
% fprintf('FastBridge Collision Cone ECBF 制約の導出を開始します...\n');
% 
% %% 1. 拡大状態量とパラメータの定義
% syms aT daT d2aT real             % 正規化推力 aT=T/m, 1階微分, 2階微分 (決定変数)
% syms taux tauy tauz real          % トルク入力 (決定変数)
% syms mu1 mu2 mu3 real             % 障害物中心座標
% syms A11 A12 A13 A22 A23 A33 real % 楕円共分散逆行列
% syms c_scale real                 % 信頼区間スケール
% syms lambda_cbf positive real     % ECBF 極配置パラメータ
% 
% e3 = [0; 0; 1];
% J  = diag([jx, jy, jz]);
% 
% %% 2. 衝突円錐バリア関数 h(r, v) の定義 (論文 Eq. 4, 8, 10)
% mu_vec = [mu1; mu2; mu3];
% A_mat  = [A11, A12, A13; A12, A22, A23; A13, A23, A33];
% 
% r_vec     = mu_vec - p;
% gamma_val = r_vec.' * A_mat * r_vec - c_scale^2;
% beta_val  = dp.' * A_mat * dp;
% delta_val = r_vec.' * A_mat * dp;
% 
% h_cc = simplify(beta_val * gamma_val - delta_val^2);
% 
% %% 3. 拡大状態系 z in R^14 における ドリフト f_ext(z) と 入力行列 G_ext(z)
% % 拡大状態ベクトル: z = [q(4); p(3); dp(3); ob(3); aT; daT]
% z_ext = [q; p; dp; ob; aT; daT];
% 
% % クォータニオン回転行列 R
% R_mat = [q0^2+q1^2-q2^2-q3^2,   2*(q1*q2-q0*q3),       2*(q1*q3+q0*q2);
%          2*(q1*q2+q0*q3),       q0^2-q1^2+q2^2-q3^2,   2*(q2*q3-q0*q1);
%          2*(q1*q3-q0*q2),       2*(q2*q3+q0*q1),       q0^2-q1^2-q2^2+q3^2];
% b3 = R_mat * e3;
% 
% % クォータニオン運動学
% Omega_w = [  0, -o1, -o2, -o3;
%             o1,   0,  o3, -o2;
%             o2, -o3,   0,  o1;
%             o3,  o2, -o1,   0];
% q_dot = 0.5 * Omega_w * q;
% 
% % ダイナミクス展開
% p_dot  = dp;
% dp_dot = -gravity * e3 + aT * b3;
% 
% gyro_drift  = -J \ cross(ob, J * ob);
% G_w_tau     = J \ eye(3);
% aT_dot      = daT;
% daT_drift   = sym(0);
% 
% f_ext = [q_dot;
%          p_dot;
%          dp_dot;
%          gyro_drift;
%          aT_dot;
%          daT_drift];
% 
% % 入力 eta = [d2aT; taux; tauy; tauz] に対する入力行列
% g_d2aT = [zeros(4,1); zeros(3,1); zeros(3,1); zeros(3,1); 0; 1];
% g_tx   = [zeros(4,1); zeros(3,1); zeros(3,1); G_w_tau(:,1); 0; 0];
% g_ty   = [zeros(4,1); zeros(3,1); zeros(3,1); G_w_tau(:,2); 0; 0];
% g_tz   = [zeros(4,1); zeros(3,1); zeros(3,1); G_w_tau(:,3); 0; 0];
% G_ext  = [g_d2aT, g_tx, g_ty, g_tz];
% 
% %% 4. Lie微分と相対次数3のECBF制約構築 (論文 Eq. 15)
% Lfh  = simplify(jacobian(h_cc, z_ext) * f_ext);
% Lf2h = simplify(jacobian(Lfh,  z_ext) * f_ext);
% Lf3h = simplify(jacobian(Lf2h, z_ext) * f_ext);
% 
% LgLf2h = simplify(jacobian(Lf2h, z_ext) * G_ext);
% 
% % 不等式制約: A_cbf * eta <= b_cbf
% A_qp = simplify(-LgLf2h);
% b_qp = simplify(Lf3h + 3*lambda_cbf*Lf2h + 3*(lambda_cbf^2)*Lfh + (lambda_cbf^3)*h_cc);
% 
% %% 5. MATLAB 関数の生成・エクスポート
% fprintf('MATLAB 関数を生成中...\n');
% splat_params = [mu1; mu2; mu3; A11; A12; A13; A22; A23; A33; c_scale];
% 
% % 1. バリア関数値の計算
% matlabFunction(h_cc, 'file', 'FastBridge_CBF_h.m', ...
%     'vars', {z_ext, splat_params}, 'outputs', {'h_val'});
% 
% % 2. Lie微分の診断関数
% matlabFunction(Lfh, Lf2h, Lf3h, 'file', 'FastBridge_CBF_Lie.m', ...
%     'vars', {z_ext, splat_params, physicalParam}, 'outputs', {'Lfh', 'Lf2h', 'Lf3h'});
% 
% % 3. QP不等式制約行列 (A_cbf * eta <= b_cbf)
% matlabFunction(A_qp, b_qp, 'file', 'FastBridge_CBF_QP.m', ...
%     'vars', {z_ext, splat_params, physicalParam, lambda_cbf}, 'outputs', {'A_cbf', 'b_cbf'});
% 
% fprintf('====================================================\n');
% fprintf('  [OK] FastBridge 完全準拠 ECBF 関数群 生成完了\n');
% fprintf('====================================================\n\n');
% 
% %% ========================================================================= %%
% %% 生成後 動作確認テスト & QPシミュレーション
% %% ========================================================================= %%
% fprintf('========================================\n');
% fprintf('  生成済み FastBridge ECBF 関数の動作テスト\n');
% fprintf('========================================\n');
% 
% % 1. 物理パラメータ設定 (m=0.461kg, 慣性モーメント)
% m_val   = 0.461;
% g_val   = 9.81;
% jx_val  = 2.5e-3; jy_val = 2.5e-3; jz_val = 4.0e-3;
% phys_test = [m_val, 0.1, 0.1, 0.05, 0.05, jx_val, jy_val, jz_val, g_val, ...
%              1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0];
% 
% % 2. クアッドローター拡大状態 z_test (14次元): 前方(+x)へ 3.0 m/s で直進
% q_test   = [1; 0; 0; 0];
% p_test   = [0.0; 0.0; 1.0];
% dp_test  = [3.0; 0.0; 0.0];
% ob_test  = [0.0; 0.0; 0.0];
% aT_test  = g_val;                 % ホバリング推力 (T/m = g)
% daT_test = 0.0;
% z_test   = [q_test; p_test; dp_test; ob_test; aT_test; daT_test];
% 
% % 3. 3DGS 障害物 (3.0m 前方に配置)
% mu_test = [3.0; 0.0; 1.0];
% Sigma_inv = diag([1/(0.5^2), 1/(0.3^2), 1/(0.8^2)]);
% A_test = [Sigma_inv(1,1); Sigma_inv(1,2); Sigma_inv(1,3); ...
%           Sigma_inv(2,2); Sigma_inv(2,3); Sigma_inv(3,3)];
% c_scale_test = 1.0;
% splat_test = [mu_test; A_test; c_scale_test];
% 
% % 4. 制御ゲイン
% lambda_test = 3.0;
% 
% % 5. 関数の評価
% h_val = FastBridge_CBF_h(z_test, splat_test);
% [Lfh_val, Lf2h_val, Lf3h_val] = FastBridge_CBF_Lie(z_test, splat_test, phys_test);
% [A_cbf_val, b_cbf_val] = FastBridge_CBF_QP(z_test, splat_test, phys_test, lambda_test);
% 
% fprintf('  バリア関数値 h_cc        : % .4f\n', double(h_val));
% fprintf('  Lf h, Lf^2 h, Lf^3 h      : [% .2e, % .2e, % .2e]\n', double(Lfh_val), double(Lf2h_val), double(Lf3h_val));
% fprintf('  A_cbf 行列サイズ         : [%d x %d] (期待値: [1 x 4])\n', size(A_cbf_val, 1), size(A_cbf_val, 2));
% fprintf('  A_cbf (d2aT, tx, ty, tz)  : [% .3e, % .3e, % .3e, % .3e]\n', double(A_cbf_val));
% fprintf('  b_cbf の値               : % .3e\n', double(b_cbf_val));
% 
% % 6. QP 検証 (quadprog): ノミナル入力 eta_nom に対するフィルタリング
% eta_nom = [0.0; 0.0; 0.0; 0.0];
% H_qp = eye(4);
% f_qp = -eta_nom;
% 
% lb = [-50.0; -0.5; -0.5; -0.1];
% ub = [ 50.0;  0.5;  0.5;  0.1];
% 
% opts = optimoptions('quadprog', 'Display', 'off');
% [eta_opt, ~, exitflag] = quadprog(H_qp, f_qp, double(A_cbf_val), double(b_cbf_val), [], [], lb, ub, [], opts);
% 
% if exitflag == 1
%     fprintf('\n  [OK] QP Safety Filter 最適化成功\n');
%     fprintf('  最適修正入力 eta*:\n');
%     fprintf('    d2aT  (推力加加速度) : % .4f m/s^4\n', eta_opt(1));
%     fprintf('    tau_x (ロールトルク) : % .4f Nm\n', eta_opt(2));
%     fprintf('    tau_y (ピッチトルク) : % .4f Nm\n', eta_opt(3));
%     fprintf('    tau_z (ヨートルク)   : % .4f Nm\n', eta_opt(4));
%     fprintf('========================================\n\n');
% else
%     warning('QPが正常に終了しませんでした (exitflag = %d)', exitflag);
% end
% %% =========================================================================
% %% FastBridge: Finite-Difference (FD) ECBF Derivation (方針B 完全準拠)
% %% Decision variable: u = [T_curr; taux; tauy; tauz] in R^4
% %% =========================================================================
% fprintf('====================================================\n');
% fprintf(' FastBridge Finite-Difference ECBF Function Generator\n');
% fprintf(' Target: A_cbf * [T; taux; tauy; tauz] <= b_cbf\n');
% fprintf('====================================================\n');
% 
% %% 1. シンボリック変数の定義
% syms q0 q1 q2 q3 real               % クォータニオン [q0; q1; q2; q3]
% syms px py pz real                   % 位置 [px; py; pz]
% syms vx vy vz real                   % 速度 [vx; vy; vz]
% syms wx wy wz real                   % 機体角速度 [wx; wy; wz]
% syms m jx jy jz gravity real         % 物理パラメータ
% 
% % 推力履歴と時間刻み
% syms T_curr T_prev1 T_prev2 dt real  % 現在の推力(QP変数), 1ステップ前, 2ステップ前, dt
% syms taux tauy tauz real             % ボディトルク(QP変数)
% 
% % 障害物 (3DGS 楕円体)
% syms mu1 mu2 mu3 real
% syms A11 A12 A13 A22 A23 A33 real
% syms c_scale real
% syms lambda_cbf positive real
% 
% q = [q0; q1; q2; q3];
% p = [px; py; pz];
% v = [vx; vy; vz];
% w = [wx; wy; wz];
% J = diag([jx; jy; jz]);
% e3 = [0; 0; 1];
% 
% %% 2. 幾何・運動学モデル
% % クォータニオン回転行列 R(q)
% R = [q0^2+q1^2-q2^2-q3^2,   2*(q1*q2-q0*q3),       2*(q1*q3+q0*q2);
%      2*(q1*q2+q0*q3),       q0^2-q1^2+q2^2-q3^2,   2*(q2*q3-q0*q1);
%      2*(q1*q3-q0*q2),       2*(q2*q3+q0*q1),       q0^2-q1^2-q2^2+q3^2];
% b3 = R * e3;
% 
% % クォータニオン微分
% Omega_w = [ 0,  -wx, -wy, -wz;
%            wx,    0,  wz, -wy;
%            wy,  -wz,   0,  wx;
%            wz,   wy, -wx,   0];
% q_dot = 0.5 * Omega_w * q;
% 
% %% 3. 衝突円錐バリア関数 h(r, v)
% mu = [mu1; mu2; mu3];
% A  = [A11, A12, A13;
%       A12, A22, A23;
%       A13, A23, A33];
% 
% r     = mu - p;
% gamma = r.' * A * r - c_scale^2;
% beta  = v.' * A * v;
% delta = r.' * A * v;
% h     = simplify(beta * gamma - delta^2);
% 
% %% 4. 加速度・ジャーク・スナップ (有限差分カスケードの代入)
% % 正規化推力の有限差分展開: aT = T/m
% aT_curr  = T_curr / m;
% aT_prev1 = T_prev1 / m;
% aT_prev2 = T_prev2 / m;
% 
% % 差分による1階・2階微分表現
% daT_fd  = (aT_curr - aT_prev1) / dt;
% d2aT_fd = (aT_curr - 2*aT_prev1 + aT_prev2) / (dt^2);
% 
% % 加速度 a = -g*e3 + aT*b3
% acc = -gravity * e3 + aT_curr * b3;
% 
% % ジャーク j = daT*b3 + aT*db3
% hat_e3 = [0, -1, 0; 1, 0, 0; 0, 0, 0];
% db3 = -R * hat_e3 * w;
% jerk = daT_fd * b3 + aT_curr * db3;
% 
% % スナップ p^(4) のアフィン分解: p^(4) = F_p + G_T*d2aT + G_tau*tau_m
% invJ = diag([1/jx, 1/jy, 1/jz]);
% dw_drift = -invJ * cross(w, J * w);
% hat_w = [0, -w(3), w(2); w(3), 0, -w(1); -w(2), w(1), 0];
% ddb3_drift = -R * hat_e3 * dw_drift + R * (hat_w * hat_w) * e3;
% 
% F_p   = 2 * daT_fd * db3 + aT_curr * ddb3_drift;
% G_T   = b3;
% G_tau = -aT_curr * R * hat_e3 * invJ;
% 
% %% 5. 相対次数3 ECBF 式の展開と整理
% % h の時間微分 (幾何ダイナミクスに沿った導出)
% dh_dt  = jacobian(h, [p; v]) * [v; acc];
% ddh_dt = jacobian(dh_dt, [p; v; w]) * [v; acc; dw_drift] + diff(dh_dt, T_curr) * (m * daT_fd);
% 
% % スナップの感度ベクトル w_snap
% w_snap = 2 * A * (gamma * v - delta * r);
% 
% % 3階微分ドリフト項 Phi
% Phi_drift = jacobian(ddh_dt, [p; v; w]) * [v; acc; dw_drift] + diff(ddh_dt, T_curr) * (m * daT_fd);
% 
% % ECBF 全体式 (h^(3) + 3*lambda*ddh + 3*lambda^2*dh + lambda^3*h >= 0)
% % h^(3) = w_snap.' * (F_p + G_T * d2aT_fd + G_tau * tau_m) + Phi_drift
% snap_total = F_p + G_T * d2aT_fd + G_tau * [taux; tauy; tauz];
% ECBF_total = w_snap.' * snap_total + Phi_drift ...
%            + 3 * lambda_cbf * ddh_dt + 3 * (lambda_cbf^2) * dh_dt + (lambda_cbf^3) * h;
% 
% %% 6. QP決定変数 u = [T_curr; taux; tauy; tauz] に対するアフィン抽出
% u_vec = [T_curr; taux; tauy; tauz];
% 
% % ECBF_total >= 0  <=>  -ECBF_total <= 0
% % -ECBF_total = A_cbf * u - b_cbf <= 0  <=>  A_cbf * u <= b_cbf
% A_cbf_sym = jacobian(-ECBF_total, u_vec);
% b_cbf_sym = simplify(A_cbf_sym * u_vec - (-ECBF_total));
% 
% A_cbf = simplify(A_cbf_sym);
% b_cbf = simplify(b_cbf_sym);
% 
% %% 7. MATLAB 関数のエクスポート
% fprintf('MATLAB 関数を生成中...\n');
% 
% z_base = [q; p; v; w];
% splat_params = [mu1; mu2; mu3; A11; A12; A13; A22; A23; A33; c_scale];
% phys_params  = [m; Lx; Ly; lx; ly; jx; jy; jz; gravity; km1; km2; km3; km4; k1; k2; k3; k4]; % 17要素対応
% 
% matlabFunction(A_cbf, b_cbf, 'file', 'FastBridge_CBF_QP_FD.m', ...
%     'vars', {z_base, splat_params, phys_params, lambda_cbf, T_prev1, T_prev2, dt}, ...
%     'outputs', {'A_cbf', 'b_cbf'});
% 
% fprintf('====================================================\n');
% fprintf(' [OK] FastBridge_CBF_QP_FD.m 正常生成完了\n');
% fprintf('====================================================\n\n');
%% =========================================================================
%% FastBridge: Finite-Difference (FD) ECBF Derivation
%%
%% Target:
%%     A_cbf * u <= b_cbf
%%
%%     u = [T_curr; tau_x; tau_y; tau_z]
%%
%% Collision Cone ECBF + Finite-Difference Thrust Derivatives
%%
%% IMPORTANT:
%%    This implementation uses finite differences for
%%
%%        aT_dot  ~= (T_k - T_{k-1}) / (m*dt)
%%
%%        aT_ddot ~= (T_k - 2*T_{k-1} + T_{k-2}) / (m*dt^2)
%%
%%    Therefore this is a discrete/FD implementation of the continuous-time
%%    ECBF condition, not an exact continuous-time Lie-derivative system
%%    with T as a direct relative-degree-3 input.
%% =========================================================================

clear;
clc;

fprintf('====================================================\n');
fprintf(' FastBridge Finite-Difference ECBF Function Generator\n');
fprintf(' Target: A_cbf * [T; taux; tauy; tauz] <= b_cbf\n');
fprintf('====================================================\n');

%% =========================================================================
%% 1. Symbolic Variables
%% =========================================================================

% -------------------------------------------------------------------------
% Quadrotor state
% -------------------------------------------------------------------------

syms q0 q1 q2 q3 real               % quaternion [q0;q1;q2;q3]

syms px py pz real                   % position
syms vx vy vz real                   % velocity
syms wx wy wz real                   % body angular velocity

q = [q0; q1; q2; q3];
p = [px; py; pz];
v = [vx; vy; vz];
w = [wx; wy; wz];

% -------------------------------------------------------------------------
% Physical parameters (17 elements)
% -------------------------------------------------------------------------

syms m Lx Ly lx ly jx jy jz gravity real
syms km1 km2 km3 km4 k1 k2 k3 k4 real

J    = diag([jx; jy; jz]);
invJ = diag([1/jx, 1/jy, 1/jz]);

% -------------------------------------------------------------------------
% Actual QP input
%
% u = [T_curr; taux; tauy; tauz]
% -------------------------------------------------------------------------

syms T_curr real
syms taux tauy tauz real

tau = [taux; tauy; tauz];

% -------------------------------------------------------------------------
% Thrust history and sampling time
%
% T_prev1 = T_{k-1}
% T_prev2 = T_{k-2}
% -------------------------------------------------------------------------

syms T_prev1 T_prev2 dt positive real

% -------------------------------------------------------------------------
% Obstacle parameters
% -------------------------------------------------------------------------

syms mu1 mu2 mu3 real

syms A11 A12 A13 A22 A23 A33 real

syms c_scale positive real

% -------------------------------------------------------------------------
% ECBF parameter
% -------------------------------------------------------------------------

syms lambda_cbf positive real

% -------------------------------------------------------------------------
% Constants
% -------------------------------------------------------------------------

e3 = [0;0;1];

%% =========================================================================
%% 2. Geometry and Quadrotor Kinematics
%% =========================================================================

% Quaternion -> rotation matrix
%
% Assumes quaternion convention compatible with the rest of the project.

R = [q0^2+q1^2-q2^2-q3^2,   2*(q1*q2-q0*q3),       2*(q1*q3+q0*q2);
     2*(q1*q2+q0*q3),       q0^2-q1^2+q2^2-q3^2,   2*(q2*q3-q0*q1);
     2*(q1*q3-q0*q2),       2*(q2*q3+q0*q1),       q0^2-q1^2-q2^2+q3^2];

% Body z-axis in world coordinates
b3 = R * e3;

% Skew matrices

hat_e3 = [0, -1, 0;
          1,  0, 0;
          0,  0, 0];

hat_w = [  0, -w(3),  w(2);
         w(3),    0, -w(1);
        -w(2),  w(1),    0];

%% =========================================================================
%% 3. Collision Cone Barrier Function
%%
%% h(r,v) = beta*gamma - delta^2
%%
%% gamma = r'*A*r - c^2
%% beta  = v'*A*v
%% delta = r'*A*v
%% =========================================================================

mu = [mu1; mu2; mu3];

A = [A11, A12, A13;
     A12, A22, A23;
     A13, A23, A33];

% Relative position:
%
% r = obstacle_center - vehicle_position

r = mu - p;

gamma = simplify(r.' * A * r - c_scale^2);

beta  = simplify(v.' * A * v);

delta = simplify(r.' * A * v);

h = simplify(beta * gamma - delta^2);

%% =========================================================================
%% 4. Base Continuous-Time Derivatives
%%
%% The previous thrust T_{k-1} is used as the operating-point thrust.
%%
%% aT_base = T_{k-1}/m
%%
%% The current decision variable T_k is introduced later through
%% finite-difference approximations of aT_dot and aT_ddot.
%% =========================================================================

% -------------------------------------------------------------------------
% Base normalized thrust
% -------------------------------------------------------------------------

aT_base = T_prev1 / m;

% -------------------------------------------------------------------------
% Translational acceleration
%
% p_ddot = -g e3 + aT*b3
% -------------------------------------------------------------------------

acc_base = -gravity * e3 + aT_base * b3;

% -------------------------------------------------------------------------
% Angular velocity drift
%
% w_dot = -J^{-1}(w x Jw) + J^{-1} tau
%
% Here only the drift part is used.
% -------------------------------------------------------------------------

dw_drift = -invJ * cross(w, J*w);

% -------------------------------------------------------------------------
% b3 derivatives
%
% b3_dot = -R*hat(e3)*w
% -------------------------------------------------------------------------

db3 = simplify(-R * hat_e3 * w);

% -------------------------------------------------------------------------
% Drift part of b3_ddot
%
% b3_ddot
% =
%   -R*hat(e3)*w_dot
%   + R*hat(w)^2*e3
%
% Torque-dependent part is introduced separately.
% -------------------------------------------------------------------------

ddb3_drift = simplify( ...
    -R * hat_e3 * dw_drift ...
    + R * (hat_w * hat_w) * e3 ...
);

% -------------------------------------------------------------------------
% First derivative of h
% -------------------------------------------------------------------------

dh_dt = simplify( ...
    jacobian(h, [p; v]) * ...
    [v; acc_base] ...
);

% -------------------------------------------------------------------------
% Second derivative of h
%
% Operating-point differentiation:
% T_prev1 is treated as fixed during this derivative evaluation.
% -------------------------------------------------------------------------

ddh_dt = simplify( ...
    jacobian(dh_dt, [p; v; w]) * ...
    [v; acc_base; dw_drift] ...
);

%% =========================================================================
%% 5. FD Snap Model and Input-Affine Decomposition
%%
%% Quadrotor translational snap:
%%
%% p^(4)
%% =
%%    aT_ddot*b3
%% + 2*aT_dot*b3_dot
%% + aT*b3_ddot
%%
%% Finite differences:
%%
%% aT_dot
%% ~= (T_k - T_{k-1})/(m*dt)
%%
%% aT_ddot
%% ~= (T_k - 2*T_{k-1} + T_{k-2})/(m*dt^2)
%%
%% The torque contribution is linearized using T_{k-1}/m as the
%% operating-point thrust.
%% =========================================================================

% -------------------------------------------------------------------------
% Collision-cone snap sensitivity
%
% Coefficient multiplying p^(4) in h^(3)
% -------------------------------------------------------------------------

w_snap = simplify( ...
    2 * A * (gamma*v - delta*r) ...
);

% -------------------------------------------------------------------------
% T_k coefficient in p^(4)
%
% IMPORTANT:
%
% The T_k-dependent terms are only
%
%    (1/(m dt^2))*b3
%    (2/(m dt))*b3_dot
%
% The aT*b3_ddot drift is evaluated at T_{k-1}/m.
% -------------------------------------------------------------------------

G_T_FD = simplify( ...
    (1/(m*dt^2)) * b3 ...
  + (2/(m*dt))   * db3 ...
);

% -------------------------------------------------------------------------
% Torque coefficient in p^(4)
%
% b3_ddot torque part:
%
% -R*hat(e3)*J^{-1}*tau
%
% multiplied by operating-point aT = T_{k-1}/m
% -------------------------------------------------------------------------

G_tau = simplify( ...
    -(T_prev1/m) * R * hat_e3 * invJ ...
);

% -------------------------------------------------------------------------
% T-independent snap drift
%
% p^(4)
% =
%    G_T_FD*T_curr
% + G_tau*tau
% + F_snap_drift
%
% where
%
% F_snap_drift =
%
%    (T_prev1/m)*b3_ddot_drift
% - (2*T_prev1/(m*dt))*b3_dot
% + (-2*T_prev1+T_prev2)/(m*dt^2)*b3
% -------------------------------------------------------------------------

F_snap_drift = simplify( ...
    (T_prev1/m) * ddb3_drift ...
  - (2*T_prev1/(m*dt)) * db3 ...
  + ((-2*T_prev1 + T_prev2)/(m*dt^2)) * b3 ...
);

% -------------------------------------------------------------------------
% Remaining h^(3) drift
%
% This represents the state-evolution contribution not explicitly contained
% in the snap input channel.
% -------------------------------------------------------------------------

Phi_drift = simplify( ...
    jacobian(ddh_dt, [p; v; w]) * ...
    [v; acc_base; dw_drift] ...
);

% -------------------------------------------------------------------------
% h^(3) input coefficients
%
% h^(3)
% =
%    Lg_T*T_curr
% + Lg_tau*tau
% + drift_h3
% -------------------------------------------------------------------------

Lg_T = simplify( ...
    w_snap.' * G_T_FD ...
);

Lg_tau = simplify( ...
    w_snap.' * G_tau ...
);

drift_h3 = simplify( ...
    w_snap.' * F_snap_drift ...
  + Phi_drift ...
);

%% =========================================================================
%% 6. Relative-Degree-3 ECBF Constraint
%%
%% ECBF condition:
%%
%% h^(3)
%% + 3 lambda h^(2)
%% + 3 lambda^2 h^(1)
%% + lambda^3 h
%% >= 0
%%
%% Substitute:
%%
%% h^(3)
%% =
%%    Lg_T*T
%% + Lg_tau*tau
%% + drift_h3
%%
%% Then:
%%
%% Lg*u + ECBF_drift >= 0
%%
%% Convert to quadprog form:
%%
%% A_cbf*u <= b_cbf
%% =========================================================================

ecbf_drift = simplify( ...
    drift_h3 ...
  + 3*lambda_cbf*ddh_dt ...
  + 3*(lambda_cbf^2)*dh_dt ...
  + (lambda_cbf^3)*h ...
);

% Actual QP variable:
%
% u = [T_curr; taux; tauy; tauz]

A_cbf = simplify( ...
    [-Lg_T, -Lg_tau] ...
);

b_cbf = simplify( ...
    ecbf_drift ...
);

%% =========================================================================
%% 7. MATLAB Function Export
%% =========================================================================

fprintf('MATLAB 関数を生成中...\n');

% -------------------------------------------------------------------------
% State vector
%
% z_base = [q(4); p(3); v(3); w(3)]
%
% Dimension = 13
% -------------------------------------------------------------------------

z_base = [q; p; v; w];

% -------------------------------------------------------------------------
% Obstacle parameters
% -------------------------------------------------------------------------

splat_params = [ ...
    mu1;
    mu2;
    mu3;
    A11;
    A12;
    A13;
    A22;
    A23;
    A33;
    c_scale];

% -------------------------------------------------------------------------
% Physical parameters
% -------------------------------------------------------------------------

phys_params = [ ...
    m;
    Lx;
    Ly;
    lx;
    ly;
    jx;
    jy;
    jz;
    gravity;
    km1;
    km2;
    km3;
    km4;
    k1;
    k2;
    k3;
    k4];

% -------------------------------------------------------------------------
% Export
% -------------------------------------------------------------------------

matlabFunction( ...
    A_cbf, ...
    b_cbf, ...
    'file', 'FastBridge_CBF_QP_FD.m', ...
    'vars', { ...
        z_base, ...
        splat_params, ...
        phys_params, ...
        lambda_cbf, ...
        T_prev1, ...
        T_prev2, ...
        dt}, ...
    'outputs', {'A_cbf', 'b_cbf'} ...
);

fprintf('====================================================\n');
fprintf(' [OK] FastBridge_CBF_QP_FD.m generated successfully\n');
fprintf('====================================================\n\n');

%% =========================================================================
%% 8. Runtime Test and QP Verification
%% =========================================================================

fprintf('========================================\n');
fprintf('  Generated FD-ECBF Function Test\n');
fprintf('========================================\n');

% -------------------------------------------------------------------------
% Physical parameters
% -------------------------------------------------------------------------

m_val  = 0.461;
g_val  = 9.81;

jx_val = 2.5e-3;
jy_val = 2.5e-3;
jz_val = 4.0e-3;

phys_test = [ ...
    m_val;
    0.1;
    0.1;
    0.05;
    0.05;
    jx_val;
    jy_val;
    jz_val;
    g_val;
    1.0;
    1.0;
    1.0;
    1.0;
    1.0;
    1.0;
    1.0;
    1.0];

% -------------------------------------------------------------------------
% State
%
% Vehicle moving forward at 3 m/s
% -------------------------------------------------------------------------

q_test = [1;0;0;0];

p_test = [0.0;
          0.0;
          1.0];

v_test = [3.0;
          0.0;
          0.0];

w_test = [0.0;
          0.0;
          0.0];

z_test = [q_test;
          p_test;
          v_test;
          w_test];

% -------------------------------------------------------------------------
% Obstacle
%
% Ellipsoidal obstacle located 3 m ahead
% -------------------------------------------------------------------------

mu_test = [3.0;
           0.0;
           1.0];

Sigma_inv = diag([ ...
    1/(0.5^2), ...
    1/(0.3^2), ...
    1/(0.8^2)]);

A_test = [ ...
    Sigma_inv(1,1);
    Sigma_inv(1,2);
    Sigma_inv(1,3);
    Sigma_inv(2,2);
    Sigma_inv(2,3);
    Sigma_inv(3,3)];

c_scale_test = 1.0;

splat_test = [ ...
    mu_test;
    A_test;
    c_scale_test];

% -------------------------------------------------------------------------
% Thrust history
% -------------------------------------------------------------------------

T_hov = m_val * g_val;

T_prev1_test = T_hov;
T_prev2_test = T_hov;

% -------------------------------------------------------------------------
% Sampling time
% -------------------------------------------------------------------------

dt_test = 0.02;

if dt_test <= 0
    error('dt_test must be positive.');
end

% -------------------------------------------------------------------------
% ECBF gain
% -------------------------------------------------------------------------

lambda_test = 2.0;

% -------------------------------------------------------------------------
% Evaluate CBF
% -------------------------------------------------------------------------

[A_val, b_val] = FastBridge_CBF_QP_FD( ...
    z_test, ...
    splat_test, ...
    phys_test, ...
    lambda_test, ...
    T_prev1_test, ...
    T_prev2_test, ...
    dt_test);

A_val = real(double(A_val));
b_val = real(double(b_val));

fprintf('  A_cbf size             : [%d x %d] (expected [1 x 4])\n', ...
    size(A_val,1), size(A_val,2));

fprintf('  A_cbf raw              : [% .3e, % .3e, % .3e, % .3e]\n', ...
    A_val);

fprintf('  b_cbf raw              : % .3e\n', b_val);

% -------------------------------------------------------------------------
% Numerical validity check
% -------------------------------------------------------------------------

if any(~isfinite(A_val)) || any(~isfinite(b_val))
    error('CBF constraint contains NaN or Inf.');
end

% -------------------------------------------------------------------------
% Constraint normalization
%
% Positive scaling does NOT change the feasible set:
%
% A*u <= b
%
% -> (A/s)*u <= b/s
% -------------------------------------------------------------------------

constraint_scale = max([norm(A_val, 2), abs(b_val), 1.0]);

A_qp_cbf = A_val / constraint_scale;
b_qp_cbf = b_val / constraint_scale;

fprintf('  Constraint scale       : % .3e\n', constraint_scale);

fprintf('  A_cbf normalized       : [% .3e, % .3e, % .3e, % .3e]\n', ...
    A_qp_cbf);

fprintf('  b_cbf normalized       : % .3e\n', b_qp_cbf);

%% =========================================================================
%% 9. QP Safety Filter Test
%% =========================================================================

% -------------------------------------------------------------------------
% Nominal input
% -------------------------------------------------------------------------

u_nom = [T_hov;
         0.0;
         0.0;
         0.0];

% -------------------------------------------------------------------------
% QP cost
%
% 1/2*(u-u_nom)'*H*(u-u_nom)
% -------------------------------------------------------------------------

H_qp = diag([ ...
    5.0;    % thrust tracking
    50.0;   % roll torque tracking
    50.0;   % pitch torque tracking
    10.0]); % yaw torque tracking

f_qp = -H_qp * u_nom;

% -------------------------------------------------------------------------
% Physical bounds
%
% IMPORTANT:
% Replace these values with the actual actuator limits of the vehicle.
% -------------------------------------------------------------------------

T_min = 0.0;
T_max = 20.0;

tau_x_max = 1.0;
tau_y_max = 1.0;
tau_z_max = 1.0;

lb = [ ...
    T_min;
   -tau_x_max;
   -tau_y_max;
   -tau_z_max];

ub = [ ...
    T_max;
    tau_x_max;
    tau_y_max;
    tau_z_max];

% -------------------------------------------------------------------------
% Solve QP
% -------------------------------------------------------------------------

opts = optimoptions( ...
    'quadprog', ...
    'Display', 'off', ...
    'Algorithm', 'interior-point-convex');

[u_opt, fval, exitflag] = quadprog( ...
    H_qp, ...
    f_qp, ...
    A_qp_cbf, ...
    b_qp_cbf, ...
    [], ...
    [], ...
    lb, ...
    ub, ...
    [], ...
    opts);

% -------------------------------------------------------------------------
% Result
% -------------------------------------------------------------------------

if exitflag == 1

    fprintf('\n');
    fprintf('  [OK] QP Safety Filter solved successfully\n');

    fprintf('  Safe input:\n');
    fprintf('    T     : % .6f N\n',  u_opt(1));
    fprintf('    tau_x : % .6f Nm\n', u_opt(2));
    fprintf('    tau_y : % .6f Nm\n', u_opt(3));
    fprintf('    tau_z : % .6f Nm\n', u_opt(4));

    % Verify original, unnormalized constraint
    constraint_value = A_val * u_opt - b_val;

    fprintf('\n');
    fprintf('  Original constraint residual A*u-b: % .6e\n', ...
        constraint_value);

    if constraint_value <= 1e-6
        fprintf('  [OK] Original ECBF inequality satisfied\n');
    else
        warning('QP solution does not satisfy original ECBF constraint within tolerance.');
    end

    fprintf('========================================\n\n');

else

    warning( ...
        'QP did not converge normally. exitflag = %d', ...
        exitflag);

end

%% =========================================================================
%% 10. Optional Diagnostic Output
%% =========================================================================

% Evaluate the nominal input against the original constraint.

nominal_residual = A_val * u_nom - b_val;

fprintf('----------------------------------------\n');
fprintf('  Diagnostic\n');
fprintf('----------------------------------------\n');

fprintf('  Nominal constraint residual: % .6e\n', ...
    nominal_residual);

if nominal_residual <= 0
    fprintf('  Nominal input is ECBF-feasible.\n');
else
    fprintf('  Nominal input violates the ECBF constraint.\n');
end

fprintf('----------------------------------------\n');
fprintf(' FD-ECBF test completed.\n');
fprintf('========================================\n');

%% =========================================================================
%% 8. 生成関数の実動作テスト & QP検証
%% =========================================================================
fprintf('========================================\n');
fprintf('  生成済み FD-ECBF 関数の動作テスト\n');
fprintf('========================================\n');

m_val = 0.461; g_val = 9.81;
jx_val = 2.5e-3; jy_val = 2.5e-3; jz_val = 4.0e-3;
phys_test = [m_val, 0.1, 0.1, 0.05, 0.05, jx_val, jy_val, jz_val, g_val, ...
             1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0];

% 状態: 前方に 3.0 m/s で突入中
q_test = [1; 0; 0; 0];
p_test = [0.0; 0.0; 1.0];
v_test = [3.0; 0.0; 0.0];
w_test = [0.0; 0.0; 0.0];
z_test = [q_test; p_test; v_test; w_test];

% 障害物: 3m前方
mu_test = [3.0; 0.0; 1.0];
Sigma_inv = diag([1/(0.5^2), 1/(0.3^2), 1/(0.8^2)]);
A_test = [Sigma_inv(1,1); Sigma_inv(1,2); Sigma_inv(1,3); ...
          Sigma_inv(2,2); Sigma_inv(2,3); Sigma_inv(3,3)];
splat_test = [mu_test; A_test; 1.0];

T_hov = m_val * g_val;
dt_test = 0.02;
lambda_test = 2.0;

[A_val, b_val] = FastBridge_CBF_QP_FD(z_test, splat_test, phys_test, lambda_test, T_hov, T_hov, dt_test);

fprintf('  A_cbf サイズ          : [%d x %d] (期待値: [1 x 4])\n', size(A_val,1), size(A_val,2));
fprintf('  A_cbf (T, tx, ty, tz) : [% .3e, % .3e, % .3e, % .3e]\n', double(A_val));
fprintf('  b_cbf の値            : % .3e\n', double(b_val));

% QP テスト
u_nom = [T_hov; 0; 0; 0];
H_qp = diag([5.0, 50.0, 50.0, 10.0]);
f_qp = -H_qp * u_nom;
lb = [0.7*T_hov; -1.0; -1.0; -1.0];
ub = [20.0;       1.0;  1.0;  1.0];

opts = optimoptions('quadprog', 'Display', 'off');
[u_opt, ~, exitflag] = quadprog(H_qp, f_qp, double(A_val), double(b_val), [], [], lb, ub, [], opts);

if exitflag == 1
    fprintf('\n  [OK] QP Safety Filter 最適化成功\n');
    fprintf('  安全制御入力 u* (T, tau): [%.3f N, %.3f Nm, %.3f Nm, %.3f Nm]\n', u_opt);
    fprintf('========================================\n\n');
else
    warning('QPが正常に終了しませんでした (exitflag = %d)', exitflag);
end
%% =========================================================================
%% 8. 生成関数の実動作テスト & QP検証
%% =========================================================================
fprintf('========================================\n');
fprintf('  生成済み FD-ECBF 関数の動作テスト\n');
fprintf('========================================\n');

m_val = 0.461; g_val = 9.81;
jx_val = 2.5e-3; jy_val = 2.5e-3; jz_val = 4.0e-3;
phys_test = [m_val, 0.1, 0.1, 0.05, 0.05, jx_val, jy_val, jz_val, g_val, ...
             1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0];

% 状態: 前方に 3.0 m/s で突入中
q_test = [1; 0; 0; 0];
p_test = [0.0; 0.0; 1.0];
v_test = [3.0; 0.0; 0.0];
w_test = [0.0; 0.0; 0.0];
z_test = [q_test; p_test; v_test; w_test];

% 障害物: 3m前方
mu_test = [3.0; 0.0; 1.0];
Sigma_inv = diag([1/(0.5^2), 1/(0.3^2), 1/(0.8^2)]);
A_test = [Sigma_inv(1,1); Sigma_inv(1,2); Sigma_inv(1,3); ...
          Sigma_inv(2,2); Sigma_inv(2,3); Sigma_inv(3,3)];
splat_test = [mu_test; A_test; 1.0];

% テスト入力履歴 (ホバリング推力)
T_hov = m_val * g_val;
dt_test = 0.02;
lambda_test = 2.0;

[A_val, b_val] = FastBridge_CBF_QP_FD(z_test, splat_test, phys_test, lambda_test, T_hov, T_hov, dt_test);

fprintf('  A_cbf サイズ          : [%d x %d] (期待値: [1 x 4])\n', size(A_val,1), size(A_val,2));
fprintf('  A_cbf (T, tx, ty, tz) : [% .3e, % .3e, % .3e, % .3e]\n', double(A_val));
fprintf('  b_cbf の値            : % .3e\n', double(b_val));

% QP テスト (ノミナル: ホバリング)
u_nom = [T_hov; 0; 0; 0];
H_qp = diag([5.0, 50.0, 50.0, 10.0]);
f_qp = -H_qp * u_nom;
lb = [0.7*T_hov; -1.0; -1.0; -1.0];
ub = [20.0;       1.0;  1.0;  1.0];

opts = optimoptions('quadprog', 'Display', 'off');
[u_opt, ~, exitflag] = quadprog(H_qp, f_qp, double(A_val), double(b_val), [], [], lb, ub, [], opts);

if exitflag == 1
    fprintf('\n  [OK] QP Safety Filter 最適化成功\n');
    fprintf('  安全制御入力 u* (T, tau): [%.3f N, %.3f Nm, %.3f Nm, %.3f Nm]\n', u_opt);
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
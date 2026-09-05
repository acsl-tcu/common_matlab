% generate_Sphere_SuspendedLoad_TetherMid_CBF.m
mfile_path = fileparts(mfilename('fullpath'));
cd(mfile_path);

disp("==============================================================");
disp(" Start: 単機完全準拠・牽引紐中点 動的拡張 ECBF (相対次数4)");
disp(" Target: A_cbf * [ddT; tau] <= b_cbf");
disp("==============================================================");

%% 1. シンボリック変数の定義 (単機記法に完全準拠)
% ドローン側
syms q0 q1 q2 q3 real
syms w1 w2 w3 real
syms T dT ddT real
syms tau1 tau2 tau3 real
syms m jx jy jz g real

% 荷物・紐側
syms pl1 pl2 pl3 vl1 vl2 vl3 real
syms pT1 pT2 pT3 ol1 ol2 ol3 real
syms mL cableL real

% 障害物 & ゲイン
syms p_obs1 p_obs2 p_obs3 r_obs real
syms k0 k1 k2 k3 real

q = [q0; q1; q2; q3];
w = [w1; w2; w3];
tau = [tau1; tau2; tau3];
J = diag([jx, jy, jz]);
e3 = [0; 0; 1];

pl = [pl1; pl2; pl3];
vl = [vl1; vl2; vl3];
pT = [pT1; pT2; pT3];
ol = [ol1; ol2; ol3];

p_obs = [p_obs1; p_obs2; p_obs3];
u_ext = [ddT; tau];

%% 2. 幾何関係と回転行列 R(q)
R = [q0^2+q1^2-q2^2-q3^2, 2*(q1*q2-q0*q3),     2*(q1*q3+q0*q2);
     2*(q1*q2+q0*q3),     q0^2-q1^2+q2^2-q3^2, 2*(q2*q3-q0*q1);
     2*(q1*q3-q0*q2),     2*(q2*q3+q0*q1),     q0^2-q1^2-q2^2+q3^2];

%% 3. 牽引系ダイナミクス連鎖の定義 (陽的運動方程式)
% 紐方向ベクトル微分
dpT = cross(ol, pT);

% 荷物の運動方程式 (張力結合): dvl = a_L
% ドローン推力ベクトル F_T = T * R * e3
F_T = T * (R * e3);
a_load = - g * e3 + ((pT.' * F_T - m * cableL * (dpT.' * dpT)) / (m + mL)) * pT;

% 紐回転角加速度 dol
a_ol = cross(pT, - F_T) / (m * cableL);

% ドローン姿勢キネマティクス & ダイナミクス (単機と同一)
dq = 0.5 * [-q1, -q2, -q3;
             q0, -q3,  q2;
             q3,  q0, -q1;
            -q2,  q1,  q0] * w;
dw = J \ (tau - cross(w, J * w));

% 全状態ベクトル x_all とその時間微分 dx_all
x_all = [pl; vl; pT; ol; q; w; T; dT];
dx_all = [vl; a_load; dpT; a_ol; dq; dw; dT; ddT];

%% 4. バリア関数の定義 (牽引紐の中点 p_mid)
% ドローン位置 p = pl - cableL * pT
% 紐中点 p_mid = pl - 0.5 * cableL * pT
p_mid = pl - 0.5 * cableL * pT;

% 単機と同一の球体バリア関数
h = (p_mid - p_obs).' * (p_mid - p_obs) - r_obs^2;

%% 5. 単機と完全に同一の Lie微分チェーン
disp('Calculating dh...');
dh = jacobian(h, x_all) * dx_all;

disp('Calculating ddh...');
ddh = jacobian(dh, x_all) * dx_all;

disp('Calculating dddh...');
dddh = jacobian(ddh, x_all) * dx_all;

disp('Calculating h4...');
h4 = jacobian(dddh, x_all) * dx_all;

%% 6. 入力アフィン分離 (A_cbf * u_ext <= b_cbf)
disp('Extracting affine components...');
A_cbf = - jacobian(h4, u_ext);
h4_drift = subs(h4, u_ext, [0; 0; 0; 0]);

% 独立ゲイン多項式
b_cbf = h4_drift + k3 * dddh + k2 * ddh + k1 * dh + k0 * h;

%% 7. エクスポート
z_state = [pl; vl; pT; ol; q; w];
T_state = [T; dT];
params = [m; mL; cableL; jx; jy; jz; g];
obs_params = [p_obs; r_obs];
cbf_gains = [k0; k1; k2; k3];

disp('Exporting to Sphere_SuspendedLoad_TetherMid_CBF.m...');
matlabFunction(A_cbf, b_cbf, h, dh, ddh, dddh, ...
    'file', 'Sphere_SuspendedLoad_TetherMid_CBF.m', ...
    'vars', {z_state, T_state, params, obs_params, cbf_gains}, ...
    'outputs', {'A_cbf', 'b_cbf', 'h', 'dh', 'ddh', 'dddh'}, ...
    'Optimize', true);

disp('Sphere_SuspendedLoad_TetherMid_CBF.m generated successfully!');

%% =========================================================================
%% [Verification Test] 厳密検証テストコード
%% =========================================================================
fprintf('\n==============================================================\n');
fprintf(' [Verification Test] 生成関数の検証テストを開始します\n');
fprintf('==============================================================\n');

m_t = 0.461; mL_t = 0.050; L_t = 0.60;
jx_t = 2.5e-3; jy_t = 2.5e-3; jz_t = 4.0e-3; g_t = 9.81;
params_t = [m_t; mL_t; L_t; jx_t; jy_t; jz_t; g_t];

% 状態設定: 前方(+X)へ荷物と中点が 1.5 m/s で直進中 (紐真下)
pl_t  = [0.0; 0.0; 0.5];
vl_t  = [1.5; 0.0; 0.0];
pT_t  = [0.0; 0.0; -1.0];
ol_t  = [0.0; 0.0; 0.0];
q_t   = [1; 0; 0; 0];
w_t   = [0.0; 0.0; 0.0];
z_t   = [pl_t; vl_t; pT_t; ol_t; q_t; w_t];

% 紐中点の実際の位置: [0; 0; 0.5 - 0.5*0.6*(-1)] = [0; 0; 0.8]
p_mid_t = pl_t - 0.5 * L_t * pT_t;

T_hov = (m_t + mL_t) * g_t;
T_t   = [T_hov; 0.0];
gains_t = [80.0; 10.0; 3.0; 1.5]; % [k0; k1; k2; k3]
u_ext_nom = [0.0; 0.0; 0.0; 0.0];

% 1. シンボリック感度チェック
sens_ddT = ~isequal(A_cbf(1), sym(0));
sens_tx  = ~isequal(A_cbf(2), sym(0));
sens_ty  = ~isequal(A_cbf(3), sym(0));
sens_tz  = ~isequal(A_cbf(4), sym(0));

fprintf('  d(h4)/d(ddT)  : %s\n', mat2str(sens_ddT));
fprintf('  d(h4)/d(taux) : %s\n', mat2str(sens_tx));
fprintf('  d(h4)/d(tauy) : %s\n', mat2str(sens_ty));
fprintf('  d(h4)/d(tauz) : %s\n', mat2str(sens_tz));

% 2. 数値テスト
r_obs_t = 0.4;

% Case A: 後方離脱 (安全)
obs_safe = [-2.0; 0.0; p_mid_t(3); r_obs_t];
[A_s, b_s, h_s] = Sphere_SuspendedLoad_TetherMid_CBF(z_t, T_t, params_t, obs_safe, gains_t);
viol_safe = double(A_s) * u_ext_nom - double(b_s);

fprintf('\n  [Case A: 安全 (障害物後方)]\n');
fprintf('    h      : %+7.4f (正であるべき)\n', double(h_s));
fprintf('    A*u - b: %+.4e (負であるべき)\n', viol_safe);

% Case B: 前方突進 (衝突コース)
obs_dang = [0.5; 0.0; p_mid_t(3); r_obs_t];
[A_d, b_d, h_d] = Sphere_SuspendedLoad_TetherMid_CBF(z_t, T_t, params_t, obs_dang, gains_t);
viol_dang = double(A_d) * u_ext_nom - double(b_d);

fprintf('\n  [Case B: 衝突侵入コース]\n');
fprintf('    h      : %+7.4f\n', double(h_d));
fprintf('    A*u - b: %+.4e (正であるべき: 違反検知)\n', viol_dang);

if (viol_safe <= 0) && (viol_dang > 0) && sens_ddT && (sens_tx || sens_ty)
    fprintf('\n  [PASS] 単機と完全に整合した牽引紐中点ECBFの動作を確認しました！\n');
else
    fprintf('\n  [FAIL] 符号または感度に不整合があります。\n');
end
fprintf("==============================================================\n");
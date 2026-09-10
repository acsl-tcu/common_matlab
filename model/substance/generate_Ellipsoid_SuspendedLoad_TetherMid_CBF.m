% generate_Ellipsoid_SuspendedLoad_TetherMid_CBF.m
mfile_path = fileparts(mfilename('fullpath'));
cd(mfile_path);

disp("==============================================================");
disp(" Start: 自機楕円体 × 分離超平面 動的拡張 ECBF (相対次数4)");
disp(" Target: A_cbf * [ddT; tau] <= b_cbf");
disp("==============================================================");

%% 1. シンボリック変数の定義
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

% 楕円体と分離超平面パラメータ
syms a_rad b_rad real % a_rad: 横半径, b_rad: 縦半径
syms n1 n2 n3 d_plane real % 分離平面の法線ベクトル(ノルム1)と距離
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

n_vec = [n1; n2; n3];
u_ext = [ddT; tau];

%% 2. 幾何関係と回転行列 R(q)
R = [q0^2+q1^2-q2^2-q3^2, 2*(q1*q2-q0*q3),     2*(q1*q3+q0*q2);
     2*(q1*q2+q0*q3),     q0^2-q1^2+q2^2-q3^2, 2*(q2*q3-q0*q1);
     2*(q1*q3-q0*q2),     2*(q2*q3+q0*q1),     q0^2-q1^2-q2^2+q3^2];

%% 3. 牽引系ダイナミクス連鎖の定義
dpT = cross(ol, pT);
F_T = T * (R * e3);
a_load = - g * e3 + ((pT.' * F_T - m * cableL * (dpT.' * dpT)) / (m + mL)) * pT;
a_ol = cross(pT, - F_T) / (m * cableL);

dq = 0.5 * [-q1, -q2, -q3;
             q0, -q3,  q2;
             q3,  q0, -q1;
            -q2,  q1,  q0] * w;
dw = J \ (tau - cross(w, J * w));

x_all = [pl; vl; pT; ol; q; w; T; dT];
dx_all = [vl; a_load; dpT; a_ol; dq; dw; dT; ddT];

%% 4. バリア関数の定義 (自機楕円体と分離超平面の距離)
% 紐中点 p_mid
p_mid = pl - 0.5 * cableL * pT;

% 分離超平面 n^T x - d_plane = 0 までの距離 (安全条件)
% 楕円体の法線方向の張り出し量 = sqrt(a^2 + (b^2 - a^2)*(n^T p_T)^2)
r_extend = sqrt(a_rad^2 + (b_rad^2 - a_rad^2)*(n_vec.' * pT)^2);
h = n_vec.' * p_mid - d_plane - r_extend;

%% 5. 微分チェーン
disp('Calculating dh...');
dh = jacobian(h, x_all) * dx_all;

disp('Calculating ddh...');
ddh = jacobian(dh, x_all) * dx_all;

disp('Calculating dddh...');
dddh = jacobian(ddh, x_all) * dx_all;

disp('Calculating h4...');
h4 = jacobian(dddh, x_all) * dx_all;
%% =========================================================================
%% [DEBUG] 入力 ddT (および T, dT) への依存性と係数のチェック (修正版)
%% =========================================================================
disp(' ');
disp('------------------ [CBF Symbolic Debug Start] ------------------');

% ゼロ判定用ヘルパー関数（シンボリック用）
chk_zero = @(expr) isequal(simplify(expr), sym(0)) || isequal(expr, sym(0));

% --- 1. 各階微分における u_ext ([ddT; tau]) へのヤコビアン（直接的な依存性） ---
disp('[1] Checking direct Jacobian wrt u_ext = [ddT; tau]:');
J_dh_u   = jacobian(dh, u_ext);
J_ddh_u  = jacobian(ddh, u_ext);
J_dddh_u = jacobian(dddh, u_ext);
J_h4_u   = jacobian(h4, u_ext);

fprintf(' - dh   wrt u_ext is zero? : %d\n', chk_zero(J_dh_u));
fprintf(' - ddh  wrt u_ext is zero? : %d\n', chk_zero(J_ddh_u));
fprintf(' - dddh wrt u_ext is zero? : %d\n', chk_zero(J_dddh_u));
fprintf(' - h4   wrt u_ext is zero? : %d\n', chk_zero(J_h4_u));

% --- 2. h4 における各入力成分 (ddT, tau1, tau2, tau3) の個別の係数チェック ---
disp(' ');
disp('[2] Checking coefficients in h4 (Jacobian wrt individual inputs):');
h4_coeff_ddT  = jacobian(h4, ddT);
h4_coeff_tau1 = jacobian(h4, tau1);
h4_coeff_tau2 = jacobian(h4, tau2);
h4_coeff_tau3 = jacobian(h4, tau3);

fprintf(' - A_cbf Thrust row (d(h4)/d(ddT))  is zero? : %d\n', chk_zero(h4_coeff_ddT));
fprintf(' - A_cbf Tau1 row   (d(h4)/d(tau1)) is zero? : %d\n', chk_zero(h4_coeff_tau1));
fprintf(' - A_cbf Tau2 row   (d(h4)/d(tau2)) is zero? : %d\n', chk_zero(h4_coeff_tau2));
fprintf(' - A_cbf Tau3 row   (d(h4)/d(tau3)) is zero? : %d\n', chk_zero(h4_coeff_tau3));

% --- 3. 代数的に項が残っているか（ゼロでない場合、構造を表示） ---
if ~chk_zero(h4_coeff_ddT)
    disp(' ');
    disp('[3] d(h4)/d(ddT) is NOT symbolically zero. Expression structure:');
    disp(simplify(h4_coeff_ddT));
else
    disp(' ');
    disp('[3] WARNING: d(h4)/d(ddT) is SYMBOLICALLY ZERO!');
    disp('    -> Dynamic chain for thrust (ddT) is broken in h4.');
end

disp('------------------- [CBF Symbolic Debug End] -------------------');
disp(' ');
% generate_Ellipsoid_SuspendedLoad_TetherMid_CBF.m
mfile_path = fileparts(mfilename('fullpath'));
cd(mfile_path);

disp("==============================================================");
disp(" Start: 自機楕円体 × 分離超平面 動的拡張 ECBF (相対次数4)");
disp(" Target: A_cbf * [ddT; tau] <= b_cbf");
disp("==============================================================");

%% 1. シンボリック変数の定義
syms q0 q1 q2 q3 real
syms w1 w2 w3 real
syms T dT ddT real
syms tau1 tau2 tau3 real
syms m jx jy jz g real
syms pl1 pl2 pl3 vl1 vl2 vl3 real
syms pT1 pT2 pT3 ol1 ol2 ol3 real
syms mL cableL real
syms a_rad b_rad real 
syms n1 n2 n3 d_plane real 
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
n_vec = [n1; n2; n3];
u_ext = [ddT; tau];

%% 2. 幾何関係と回転行列 R(q)
R = [q0^2+q1^2-q2^2-q3^2, 2*(q1*q2-q0*q3),     2*(q1*q3+q0*q2);
     2*(q1*q2+q0*q3),     q0^2-q1^2+q2^2-q3^2, 2*(q2*q3-q0*q1);
     2*(q1*q3-q0*q2),     2*(q2*q3+q0*q1),     q0^2-q1^2-q2^2+q3^2];

%% 3. 牽引系ダイナミクス連鎖の定義
dpT = cross(ol, pT);
F_T = T * (R * e3);
a_load = - g * e3 + ((pT.' * F_T - m * cableL * (dpT.' * dpT)) / (m + mL)) * pT;
a_ol = cross(pT, - F_T) / (m * cableL);

dq = 0.5 * [-q1, -q2, -q3;
             q0, -q3,  q2;
             q3,  q0, -q1;
            -q2,  q1,  q0] * w;
dw = J \ (tau - cross(w, J * w));

x_all = [pl; vl; pT; ol; q; w; T; dT];
dx_all = [vl; a_load; dpT; a_ol; dq; dw; dT; ddT];

%% 4. バリア関数の定義
p_mid = pl - 0.5 * cableL * pT;
r_extend = sqrt(a_rad^2 + (b_rad^2 - a_rad^2)*(n_vec.' * pT)^2);
h = n_vec.' * p_mid - d_plane - r_extend;

%% 5. 微分チェーン
disp('Calculating dh...');
dh = jacobian(h, x_all) * dx_all;
disp('Calculating ddh...');
ddh = jacobian(dh, x_all) * dx_all;
disp('Calculating dddh...');
dddh = jacobian(ddh, x_all) * dx_all;
disp('Calculating h4...');
h4 = jacobian(dddh, x_all) * dx_all;

%% =========================================================================
%% 【多角パターン検証デバッグブロック】
%% =========================================================================
disp(' ');
disp('==============================================================');
disp(' [DEBUG] 状態・物理パラメータ代入テスト（原因特定）');
disp('==============================================================');

% 1. 各微分段階における状態変数（特に T, dT）への依存性
disp('[Check 1] 状態変数 T, dT に対する偏微分構造');
fprintf('  dh   / dT  : %s\n', char(simplify(jacobian(dh, T))));
fprintf('  ddh  / dT  : %s\n', char(simplify(jacobian(ddh, T))));
fprintf('  dddh / dT  : %s\n', char(simplify(jacobian(dddh, T))));
fprintf('  h4   / dT  : %s\n', char(simplify(jacobian(h4, T))));
fprintf('  h4   / ddT : %s\n', char(simplify(jacobian(h4, ddT))));

% 2. 数値パターンを代入して A_cbf の第1列（推力加速度係数）を評価
A_cbf_sym = - jacobian(h4, u_ext);

% 共通ベース数値設定
sub_base = [m==1.5, mL==0.5, cableL==1.0, jx==0.02, jy==0.02, jz==0.04, g==9.81, ...
            a_rad==0.35, b_rad==0.85, d_plane==2.0, T==19.62, dT==0];

% パターン A: 水平姿勢・静止状態（法線: +X 方向）
sub_patA = [sub_base, ...
            q0==1, q1==0, q2==0, q3==0, w1==0, w2==0, w3==0, ...
            pl1==0, pl2==0, pl3==0, vl1==0, vl2==0, vl3==0, ...
            pT1==0, pT2==0, pT3==-1, ol1==0, ol2==0, ol3==0, ...
            n1==1, n2==0, n3==0];

% パターン B: 水平姿勢・静止状態（法線: +Z 方向 / 直上）
sub_patB = [sub_base, ...
            q0==1, q1==0, q2==0, q3==0, w1==0, w2==0, w3==0, ...
            pl1==0, pl2==0, pl3==0, vl1==0, vl2==0, vl3==0, ...
            pT1==0, pT2==0, pT3==-1, ol1==0, ol2==0, ol3==0, ...
            n1==0, n2==0, n3==1];

% パターン C: 45度傾斜姿勢・紐振り子運動あり（法線: 斜め方向）
sub_patC = [sub_base, ...
            q0==cos(pi/8), q1==sin(pi/8), q2==0, q3==0, w1==0.1, w2==0.5, w3==0, ...
            pl1==1.0, pl2==0.5, pl3==-0.5, vl1==0.2, vl2==-0.1, vl3==0.0, ...
            pT1==0.2, pT2==0.1, pT3==-0.95, ol1==0.3, ol2==-0.2, ol3==0.0, ...
            n1==1/sqrt(3), n2==1/sqrt(3), n3==1/sqrt(3)];

disp(' ');
disp('[Check 2] 多様パターンにおける A_cbf 行列（数値評価）');
A_num_A = double(subs(A_cbf_sym, sub_patA));
A_num_B = double(subs(A_cbf_sym, sub_patB));
A_num_C = double(subs(A_cbf_sym, sub_patC));

fprintf(' Pattern A (法線+X) A_cbf Thrust Col: %f | Tau Cols: [%f, %f, %f]\n', ...
    A_num_A(1), A_num_A(2), A_num_A(3), A_num_A(4));
fprintf(' Pattern B (法線+Z) A_cbf Thrust Col: %f | Tau Cols: [%f, %f, %f]\n', ...
    A_num_B(1), A_num_B(2), A_num_B(3), A_num_B(4));
fprintf(' Pattern C (傾斜・斜め) A_cbf Thrust Col: %f | Tau Cols: [%f, %f, %f]\n', ...
    A_num_C(1), A_num_C(2), A_num_C(3), A_num_C(4));

disp('==============================================================');
disp(' ');

%% 6. 入力アフィン分離 (A_cbf * u_ext <= b_cbf)
disp('Extracting affine components...');
A_cbf = - jacobian(h4, u_ext);
h4_drift = subs(h4, u_ext, [0; 0; 0; 0]);

b_cbf = h4_drift + k3 * dddh + k2 * ddh + k1 * dh + k0 * h;

%% 7. エクスポート
z_state = [pl; vl; pT; ol; q; w];
T_state = [T; dT];
params = [m; mL; cableL; jx; jy; jz; g];
ellipsoid_params = [a_rad; b_rad];
plane_params = [n_vec; d_plane];
cbf_gains = [k0; k1; k2; k3];

disp('Exporting to Ellipsoid_SuspendedLoad_TetherMid_CBF.m...');
matlabFunction(A_cbf, b_cbf, h, dh, ddh, dddh, ...
    'file', 'Ellipsoid_SuspendedLoad_TetherMid_CBF.m', ...
    'vars', {z_state, T_state, params, ellipsoid_params, plane_params, cbf_gains}, ...
    'outputs', {'A_cbf', 'b_cbf', 'h', 'dh', 'ddh', 'dddh'}, ...
    'Optimize', true);
disp('Ellipsoid_SuspendedLoad_TetherMid_CBF.m generated successfully!');
%% =========================================================================
%% =========================================================================
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
ellipsoid_params = [a_rad; b_rad];
plane_params = [n_vec; d_plane];
cbf_gains = [k0; k1; k2; k3];

disp('Exporting to Ellipsoid_SuspendedLoad_TetherMid_CBF.m...');
matlabFunction(A_cbf, b_cbf, h, dh, ddh, dddh, ...
    'file', 'Ellipsoid_SuspendedLoad_TetherMid_CBF.m', ...
    'vars', {z_state, T_state, params, ellipsoid_params, plane_params, cbf_gains}, ...
    'outputs', {'A_cbf', 'b_cbf', 'h', 'dh', 'ddh', 'dddh'}, ...
    'Optimize', true);

disp('Ellipsoid_SuspendedLoad_TetherMid_CBF.m generated successfully!');

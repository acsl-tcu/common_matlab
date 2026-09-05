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

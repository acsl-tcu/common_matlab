% generateController_VirtualCBF.m
% 仮想入力（スナップ空間）に対する 3DGS 楕円体 Collision Cone CBF の導出

% スクリプトがあるフォルダに移動
mfile_path = fileparts(mfilename('fullpath'));
cd(mfile_path);

syms p1 p2 p3 v1 v2 v3 a1 a2 a3 j1 j2 j3 real
syms mu1 mu2 mu3 A11 A12 A13 A22 A23 A33 c_scale real
syms mu_x mu_y mu_z real % 決定変数（スナップ空間の入力）
syms k0 k1 k2 real

p = [p1; p2; p3];
v_vel = [v1; v2; v3];
a = [a1; a2; a3];
j = [j1; j2; j3];
snap = [mu_x; mu_y; mu_z];

mu_obs = [mu1; mu2; mu3];
A  = [A11, A12, A13;
      A12, A22, A23;
      A13, A23, A33];
r = mu_obs - p;

% Collision Cone CBF: h_cc(r, v) >= 0
beta_term  = v_vel.' * A * v_vel;
gamma_term = r.' * A * r - c_scale^2;
delta_term = r.' * A * v_vel;
h_cc = simplify(beta_term * gamma_term - delta_term^2);

% Lie微分のためのヤコビアン計算ヘルパー
pdiff_tmp = @(flist, vars) arrayfun(@(func) arrayfun(@(x) diff(func, x), vars)', flist, 'UniformOutput', false);
jacobian_mat = @(f, x) cell2sym(pdiff_tmp(f, x));

disp('Calculating 1st derivative...');
dh = simplify(jacobian_mat(h_cc, [p; v_vel]) * [v_vel; a]);

disp('Calculating 2nd derivative...');
ddh = simplify(jacobian_mat(dh, [p; v_vel; a]) * [v_vel; a; j]);

disp('Calculating 3rd derivative...');
dddh = simplify(jacobian_mat(ddh, [p; v_vel; a; j]) * [v_vel; a; j; snap]);

% 3階微分からスナップ (mu_x, mu_y, mu_z) に関するアフィン係数 A_v を抽出
disp('Extracting affine coefficients...');
A_cbf = simplify(-jacobian_mat(dddh, snap));

% 零入力ドリフト b_v = dddh(snap=0) + k2*ddh + k1*dh + k0*h
dddh_drift = simplify(subs(dddh, snap, [0;0;0]));
b_cbf = simplify(dddh_drift + k2 * ddh + k1 * dh + k0 * h_cc);

% MATLAB関数としてエクスポート
z_state = [p; v_vel; a; j]; % 12次元状態ベクトル (位置, 速度, 加速度, 躍度)
splat_params = [mu1; mu2; mu3; A11; A12; A13; A22; A23; A33; c_scale]; % 10次元
cbf_gains = [k0; k1; k2]; % 3次元

disp('Exporting to Virtual_Snap_ECBF.m...');
matlabFunction(A_cbf, b_cbf, h_cc, dh, ddh, ...
    'file', 'Virtual_Snap_ECBF.m', ...
    'vars', {z_state, splat_params, cbf_gains}, ...
    'outputs', {'A_v', 'b_v', 'h0', 'h1', 'h2'});

disp('Virtual_Snap_ECBF.m generated successfully!');

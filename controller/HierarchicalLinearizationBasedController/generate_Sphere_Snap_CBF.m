% generate_Sphere_Snap_CBF.m
% 位置ベース（球体）HOCBF のスナップ空間（4階微分）における制約導出

mfile_path = fileparts(mfilename('fullpath'));
cd(mfile_path);

syms p1 p2 p3 v1 v2 v3 a1 a2 a3 j1 j2 j3 real
syms p_obs1 p_obs2 p_obs3 r_obs real
syms mu_x mu_y mu_z real % スナップ入力
syms k0 k1 k2 k3 real

p = [p1; p2; p3];
v = [v1; v2; v3];
a = [a1; a2; a3];
j = [j1; j2; j3];
snap = [mu_x; mu_y; mu_z];
p_obs = [p_obs1; p_obs2; p_obs3];

% 純粋な球体の距離バリア関数
h = (p - p_obs).' * (p - p_obs) - r_obs^2;

pdiff_tmp = @(flist, vars) arrayfun(@(func) arrayfun(@(x) diff(func, x), vars)', flist, 'UniformOutput', false);
jacobian_mat = @(f, x) cell2sym(pdiff_tmp(f, x));

dh = simplify(jacobian_mat(h, p) * v);
ddh = simplify(jacobian_mat(dh, [p; v]) * [v; a]);
dddh = simplify(jacobian_mat(ddh, [p; v; a]) * [v; a; j]);
h4 = simplify(jacobian_mat(dddh, [p; v; a; j]) * [v; a; j; snap]);

% 相対次数4の HOCBF: h4 + k3*dddh + k2*ddh + k1*dh + k0*h >= 0
% A_cbf * snap <= b_cbf に変形
A_cbf = simplify(-jacobian_mat(h4, snap));
h4_drift = simplify(subs(h4, snap, [0;0;0]));
b_cbf = simplify(h4_drift + k3*dddh + k2*ddh + k1*dh + k0*h);

z_state = [p; v; a; j];
obs_params = [p_obs; r_obs];
cbf_gains = [k0; k1; k2; k3];

matlabFunction(A_cbf, b_cbf, h, dh, ddh, dddh, ...
    'file', 'Sphere_Snap_CBF.m', ...
    'vars', {z_state, obs_params, cbf_gains}, ...
    'outputs', {'A_cbf', 'b_cbf', 'h', 'dh', 'ddh', 'dddh'});

disp('Sphere_Snap_CBF.m generated successfully!');

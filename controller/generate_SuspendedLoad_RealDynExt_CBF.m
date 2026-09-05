% generate_SuspendedLoad_RealDynExt_CBF.m
% 懸架システムの非線形ダイナミクスに基づく紐中点の HOCBF 導出

mfile_path = fileparts(mfilename('fullpath'));
cd(mfile_path);

syms p_q1 p_q2 p_q3 v_q1 v_q2 v_q3 real
syms p_L1 p_L2 p_L3 v_L1 v_L2 v_L3 real
syms q0 q1 q2 q3 real
syms w1 w2 w3 real
syms T dT ddT real
syms tau1 tau2 tau3 real
syms m_q m_L L jx jy jz g real
syms p_obs1 p_obs2 p_obs3 r_obs real
syms k0 k1 k2 k3 real

p_q = [p_q1; p_q2; p_q3];
v_q = [v_q1; v_q2; v_q3];
p_L = [p_L1; p_L2; p_L3];
v_L = [v_L1; v_L2; v_L3];
q_quat = [q0; q1; q2; q3];
w = [w1; w2; w3];
tau = [tau1; tau2; tau3];
p_obs = [p_obs1; p_obs2; p_obs3];

J = diag([jx, jy, jz]);
e3 = [0; 0; 1];

% Quaternion to Rotation Matrix
R = [q0^2+q1^2-q2^2-q3^2, 2*(q1*q2-q0*q3),   2*(q1*q3+q0*q2);
     2*(q1*q2+q0*q3),     q0^2-q1^2+q2^2-q3^2, 2*(q2*q3-q0*q1);
     2*(q1*q3-q0*q2),     2*(q2*q3+q0*q1),     q0^2-q1^2-q2^2+q3^2];

% ケーブルの方向と微分
p_T = (p_L - p_q) / L;
v_T = (v_L - v_q) / L;

% 張力 T_c の導出 (拘束条件 ||p_L - p_q|| = L より)
% Tc = (m_q * m_L) / (m_q + m_L) * ( - (T/m_q)*p_T' * R * e3 + L * ||v_T||^2 )
Tc = (m_q * m_L) / (m_q + m_L) * ( - (T/m_q) * (p_T.' * R * e3) + L * (v_T.' * v_T) );

% 加速度
a_q = -g*e3 + (T/m_q) * R * e3 + (Tc/m_q) * p_T;
a_L = -g*e3 - (Tc/m_L) * p_T;

% ドローン本体の角加速度とクォータニオン微分
dw = J \ (tau - cross(w, J*w));
dq = 0.5 * [-q1, -q2, -q3;
             q0, -q3,  q2;
             q3,  q0, -q1;
            -q2,  q1,  q0] * w;

% 全状態ベクトル
x_all = [p_q; v_q; p_L; v_L; q_quat; w; T; dT];
dx_all = [v_q; a_q; v_L; a_L; dq; dw; dT; ddT];

% 紐中点の定義とバリア関数
p_m = (p_q + p_L) / 2.0;
h = (p_m - p_obs).' * (p_m - p_obs) - r_obs^2;

disp('Calculating dh...');
dh = simplify(jacobian(h, x_all) * dx_all);

disp('Calculating ddh...');
ddh = simplify(jacobian(dh, x_all) * dx_all);

disp('Calculating dddh (This may take a while)...');
dddh = simplify(jacobian(ddh, x_all) * dx_all);

disp('Calculating h4 (This may take a LONG while)...');
h4 = simplify(jacobian(dddh, x_all) * dx_all);

u_ext = [ddT; tau];
A_cbf = simplify(-jacobian(h4, u_ext));
h4_drift = simplify(subs(h4, u_ext, [0;0;0;0]));
b_cbf = simplify(h4_drift + k3*dddh + k2*ddh + k1*dh + k0*h);

z_state = [p_q; v_q; p_L; v_L; q_quat; w];
T_state = [T; dT];
drone_params = [m_q; m_L; L; jx; jy; jz; g];
obs_params = [p_obs; r_obs];
cbf_gains = [k0; k1; k2; k3];

disp('Exporting to SuspendedLoad_RealDynExt_CBF.m...');
matlabFunction(A_cbf, b_cbf, h, dh, ddh, dddh, ...
    'file', 'SuspendedLoad_RealDynExt_CBF.m', ...
    'vars', {z_state, T_state, drone_params, obs_params, cbf_gains}, ...
    'outputs', {'A_cbf', 'b_cbf', 'h', 'dh', 'ddh', 'dddh'});

disp('SuspendedLoad_RealDynExt_CBF.m generated successfully!');

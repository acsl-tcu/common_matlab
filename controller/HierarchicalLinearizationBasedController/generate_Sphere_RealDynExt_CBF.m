% generate_Sphere_RealDynExt_CBF.m
mfile_path = fileparts(mfilename('fullpath'));
cd(mfile_path);

syms p1 p2 p3 v1 v2 v3 real
syms q0 q1 q2 q3 real
syms w1 w2 w3 real
syms T dT ddT real
syms tau1 tau2 tau3 real
syms m jx jy jz g real
syms p_obs1 p_obs2 p_obs3 r_obs real
syms k0 k1 k2 k3 real

p = [p1; p2; p3];
v = [v1; v2; v3];
q = [q0; q1; q2; q3];
w = [w1; w2; w3];
tau = [tau1; tau2; tau3];
p_obs = [p_obs1; p_obs2; p_obs3];

J = diag([jx, jy, jz]);

% Quaternion to Rotation Matrix
R = [q0^2+q1^2-q2^2-q3^2, 2*(q1*q2-q0*q3),   2*(q1*q3+q0*q2);
     2*(q1*q2+q0*q3),     q0^2-q1^2+q2^2-q3^2, 2*(q2*q3-q0*q1);
     2*(q1*q3-q0*q2),     2*(q2*q3+q0*q1),     q0^2-q1^2-q2^2+q3^2];

e3 = [0; 0; 1];

% System Dynamics
dp = v;
dv = -g*e3 + (T/m) * R * e3;
% Quaternion kinematics
dq = 0.5 * [-q1, -q2, -q3;
             q0, -q3,  q2;
             q3,  q0, -q1;
            -q2,  q1,  q0] * w;
dw = J \ (tau - cross(w, J*w));

x_all = [p; v; q; w; T; dT];
dx_all = [dp; dv; dq; dw; dT; ddT];

% Barrier function (Sphere)
h = (p - p_obs).' * (p - p_obs) - r_obs^2;

disp('Calculating dh...');
dh = simplify(jacobian(h, x_all) * dx_all);
disp('Calculating ddh...');
ddh = simplify(jacobian(dh, x_all) * dx_all);
disp('Calculating dddh...');
dddh = simplify(jacobian(ddh, x_all) * dx_all);
disp('Calculating h4...');
h4 = simplify(jacobian(dddh, x_all) * dx_all);

u_ext = [ddT; tau];
A_cbf = simplify(-jacobian(h4, u_ext));
h4_drift = simplify(subs(h4, u_ext, [0;0;0;0]));
b_cbf = simplify(h4_drift + k3*dddh + k2*ddh + k1*dh + k0*h);

z_state = [p; v; q; w];
T_state = [T; dT];
drone_params = [m; jx; jy; jz; g];
obs_params = [p_obs; r_obs];
cbf_gains = [k0; k1; k2; k3];

disp('Exporting to Sphere_RealDynExt_CBF.m...');
matlabFunction(A_cbf, b_cbf, h, dh, ddh, dddh, ...
    'file', 'Sphere_RealDynExt_CBF.m', ...
    'vars', {z_state, T_state, drone_params, obs_params, cbf_gains}, ...
    'outputs', {'A_cbf', 'b_cbf', 'h', 'dh', 'ddh', 'dddh'});

disp('Sphere_RealDynExt_CBF.m generated successfully!');

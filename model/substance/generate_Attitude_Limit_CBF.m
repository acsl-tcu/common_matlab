% generate_Attitude_Limit_CBF.m
mfile_path = fileparts(mfilename('fullpath'));
cd(mfile_path);

syms q0 q1 q2 q3 real
syms w1 w2 w3 real
syms tau1 tau2 tau3 real
syms ddT real
syms jx jy jz real
syms cos_gamma_max real
syms k0 k1 real

q = [q0; q1; q2; q3];
w = [w1; w2; w3];
tau = [tau1; tau2; tau3];
u_ext = [ddT; tau1; tau2; tau3];
J = diag([jx, jy, jz]);

R33 = q0^2 - q1^2 - q2^2 + q3^2;
h = R33 - cos_gamma_max;

dq = 0.5 * [-q1, -q2, -q3;
             q0, -q3,  q2;
             q3,  q0, -q1;
            -q2,  q1,  q0] * w;
dw = J \ (tau - cross(w, J * w));

x_att = [q; w];
dx_att = [dq; dw];

dh = jacobian(h, x_att) * dx_att;
ddh = jacobian(dh, x_att) * dx_att;

A_att = - jacobian(ddh, u_ext);
ddh_drift = subs(ddh, [ddT, tau1, tau2, tau3], [0, 0, 0, 0]);

b_att = ddh_drift + k1 * dh + k0 * h;

z_state = [q; w];
params = [jx; jy; jz];
cbf_gains = [k0; k1];

matlabFunction(A_att, b_att, h, ...
    'file', 'Attitude_Limit_CBF.m', ...
    'vars', {z_state, params, cos_gamma_max, cbf_gains}, ...
    'outputs', {'A_att', 'b_att', 'h_att'}, ...
    'Optimize', true);

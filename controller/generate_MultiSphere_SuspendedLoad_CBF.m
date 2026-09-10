syms q0 q1 q2 q3 w1 w2 w3 T dT ddT tau1 tau2 tau3 m jx jy jz g real
syms pl1 pl2 pl3 vl1 vl2 vl3 pT1 pT2 pT3 ol1 ol2 ol3 mL cableL real
syms xo yo zo R_safe c_ratio real
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
p_obs = [xo; yo; zo];
u_ext = [ddT; tau];

R = [q0^2+q1^2-q2^2-q3^2, 2*(q1*q2-q0*q3),     2*(q1*q3+q0*q2);
     2*(q1*q2+q0*q3),     q0^2-q1^2+q2^2-q3^2, 2*(q2*q3-q0*q1);
     2*(q1*q3-q0*q2),     2*(q2*q3+q0*q1),     q0^2-q1^2-q2^2+q3^2];

dpT = cross(ol, pT);
F_T = T * (R * e3);
a_load = - g * e3 + ((pT.' * F_T - m * cableL * (dpT.' * dpT)) / (m + mL)) * pT;
a_ol = cross(pT, - F_T) / (m * cableL);

dq = 0.5 * [-q1, -q2, -q3; q0, -q3, q2; q3, q0, -q1; -q2, q1, q0] * w;
dw = J \ (tau - cross(w, J * w));

x_all = [pl; vl; pT; ol; q; w; T; dT];
dx_all = [vl; a_load; dpT; a_ol; dq; dw; dT; ddT];

% Target Point
p_sys = pl + c_ratio * cableL * pT;
h = (p_sys - p_obs).' * (p_sys - p_obs) - R_safe^2;

disp('dh...'); dh = jacobian(h, x_all) * dx_all;
disp('ddh...'); ddh = jacobian(dh, x_all) * dx_all;
disp('dddh...'); dddh = jacobian(ddh, x_all) * dx_all;
disp('h4...'); h4 = jacobian(dddh, x_all) * dx_all;

disp('A_cbf...');
A_cbf = -jacobian(h4, u_ext);
h4_drift = subs(h4, u_ext, [0;0;0;0]);
b_cbf = h4_drift + k3 * dddh + k2 * ddh + k1 * dh + k0 * h;

z_state = [pl; vl; pT; ol; q; w];
T_state = [T; dT];
params = [m; mL; cableL; jx; jy; jz; g];

disp('Exporting...');
matlabFunction(A_cbf, b_cbf, h, 'file', 'MultiSphere_SuspendedLoad_CBF.m', ...
    'vars', {z_state, T_state, params, p_obs, R_safe, c_ratio, [k0;k1;k2;k3]}, ...
    'outputs', {'A_cbf', 'b_cbf', 'h'}, 'Optimize', true);
disp('Done');

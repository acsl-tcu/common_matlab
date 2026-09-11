syms r1 r2 r3 real
syms p1 p2 p3 real
syms R_safe gamma sigma real
syms u_nom1 u_nom2 u_nom3 real

r = [r1; r2; r3];
p_obs = [p1; p2; p3];
u_nom = [u_nom1; u_nom2; u_nom3];

h = (r - p_obs).' * (r - p_obs) - R_safe^2;
Lgh = 2 * (r - p_obs).';
a = Lgh * u_nom + gamma * h;
b = Lgh * Lgh.';

lambda_sontag = (sqrt(a^2 + sigma * b^2) - a) / (2 * b);
u_safe = u_nom + lambda_sontag * Lgh.';

du_sontag = jacobian(u_safe, r);
dot_u_safe = du_sontag * u_safe;

ddot_u_safe = jacobian(dot_u_safe, r) * u_safe;
len_dddu = length(char(ddot_u_safe(1)));
fprintf("Length of 3rd derivative: %d\n", len_dddu);

dddot_u_safe = jacobian(ddot_u_safe, r) * u_safe;
len_4 = length(char(dddot_u_safe(1)));
fprintf("Length of 4th derivative: %d\n", len_4);


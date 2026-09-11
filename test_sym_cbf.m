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

% Half-Sontag: lambda = (sqrt(a^2 + b * (sigma * b)) - a) / (2*b)
% Cohen's Smooth Sontag formula: 
% F_Sontag = b*lambda^2 + a*lambda - 1/4 * sigma * b = 0
lambda_sontag = (sqrt(a^2 + sigma * b^2) - a) / (2 * b);

u_safe = u_nom + lambda_sontag * Lgh.';

disp("Symbolic definition done. Testing derivation length...");
du_sontag = jacobian(u_safe, r);
len_du_sontag = length(char(du_sontag(1,1)));
fprintf("Length of sontag jacobian element: %d\n", len_du_sontag);

% Try 2nd derivative (acceleration of r_safe)
% dot_u_safe = du_sontag * u_safe
% ddot_r_safe = dot_u_safe
dot_u_safe = du_sontag * u_safe;
len_ddu = length(char(dot_u_safe(1)));
fprintf("Length of 2nd derivative: %d\n", len_ddu);


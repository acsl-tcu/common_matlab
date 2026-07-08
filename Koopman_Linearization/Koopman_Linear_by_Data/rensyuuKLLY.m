clear;clc;

p = 8;          % Psiの行数
p_theta = 6;    % Theta_plusの行数
q = 100;        % データ数

Psi = randn(p, q);
Theta_plus = randn(p_theta, q);

G = (1/q) * Theta_plus * Psi';
H = (1/q) * Psi * Psi';
% c = (1/q) * Theta_plus * Theta_plus';
c = (1/q) * trace(Theta_plus * Theta_plus');

disp('G');
disp(size(G));
disp('H');
disp(size(H));
disp('c');
disp(c);

[Q,S,~] = svd(Psi, 'econ');
L = (1/sqrt(q)) * Q * S;

disp('L')
disp(size(L))

check_L=norm(H - L*L', 'fro');

fprintf('Lが適しているか');
disp(check_L);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

yalmip('clear');

U = sdpvar(p_theta, p, 'full');
W = sdpvar(p_theta, p_theta, 'symmetric');
nu_var = sdpvar(1,1);

Constraints = [];

Constraints = [Constraints, trace(W) <= nu_var];
Constraints = [Constraints, W >= 1e-6*eye(p_theta)];

M = [W, U*L;
     (U*L)', eye(size(L,2))];

Constraints = [Constraints, M >= 1e-6*eye(size(M,1))];

Objective = c - 2*trace(U*G') + nu_var;

options = sdpsettings('solver','sdpt3','verbose',1);
diagnostics = optimize(Constraints, Objective, options);

disp(diagnostics.problem)
disp(diagnostics.info)

U_val = value(U);
disp(size(U_val))
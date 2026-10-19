clear; clc;

KL = load('errore_02_spline__KL_err_gain_controllable_dlqr_20261015_171844_339.mat');
LYKL = load('errore_02_spline__LYKL_err_gain_controllable_dlqr_20261015_172827_218.mat');

models = {KL, LYKL};
names = {'KL', 'LYKL'};

for i = 1:2

    M = models{i};

    fprintf('\n========== %s ==========\n', names{i});

    fprintf('可制御次元 = %d\n', M.nCont);

    fprintf('||A||F = %.6e\n', norm(M.A,'fro'));
    fprintf('||B||F = %.6e\n', norm(M.B,'fro'));
    fprintf('||K||F = %.6e\n', norm(M.K_full,'fro'));

    fprintf('max|eig(A)| = %.6f\n', ...
        max(abs(eig(M.A))));

    fprintf('max|eig(A-BK)| = %.6f\n', ...
        max(abs(eig(M.A-M.B*M.K_full))));

    fprintf('\n各入力のゲインノルム\n');
    disp(vecnorm(M.K_full,2,2));

    fprintf('Bの各入力列ノルム\n');
    disp(vecnorm(M.B,2,1));

end

fprintf('\n========== KLとLYKLの差 ==========\n');

fprintf('||A_KL-A_LYKL||F = %.6e\n', ...
    norm(KL.A-LYKL.A,'fro'));

fprintf('||B_KL-B_LYKL||F = %.6e\n', ...
    norm(KL.B-LYKL.B,'fro'));

fprintf('||K_KL-K_LYKL||F = %.6e\n', ...
    norm(KL.K_full-LYKL.K_full,'fro'));
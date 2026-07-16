%% test_solve_koopman_alternating.m
% solve_koopman_alternating が正常に動作するか確認するテストコード

clear;
clc;
close all;

%% 乱数を固定
rng(1);

%% テストデータのサイズ
p_theta = 6;      % 観測量の数
n_u = 2;          % 入力の数
q = 100;         % データ数

%% 真の安定な行列Aを作成
A_random = randn(p_theta);

% 最大固有値の絶対値が0.7になるように調整
A_true = 0.7 * A_random / max(abs(eig(A_random)));

%% 真の入力行列Bを作成
B_true = 0.3 * randn(p_theta, n_u);

%% 現時刻の観測量データ
Theta = randn(p_theta, q);

%% 入力データ
Input = randn(n_u, q);

%% 次時刻の観測量データ
noise_level = 0.01;

Theta_plus = ...
    A_true * Theta ...
    + B_true * Input ...
    + noise_level * randn(p_theta, q);

%% Psi = [観測量; 入力]
Psi = [Theta;
       Input];

%% データサイズを表示
fprintf('入力データのサイズ\n');
fprintf('size(Psi)        = %d × %d\n', ...
    size(Psi,1), size(Psi,2));

fprintf('size(Theta_plus) = %d × %d\n\n', ...
    size(Theta_plus,1), size(Theta_plus,2));

%% 最適化条件
rho_bar = 0.99;
max_iter = 30;
tolerance = 1e-5;

%% 関数を実行
try
    [U_val, A_val, B_val, P_val, result] = ...
        solve_koopman_alternating( ...
            Psi, ...
            Theta_plus, ...
            rho_bar, ...
            max_iter, ...
            tolerance);

catch ME
    fprintf(2, '\n計算中にエラーが発生しました。\n');
    fprintf(2, '%s\n', ME.message);

    % エラーが発生した位置を表示
    for k = 1:length(ME.stack)
        fprintf(2, ...
            'ファイル: %s, 行: %d\n', ...
            ME.stack(k).name, ...
            ME.stack(k).line);
    end

    return;
end

%% 推定結果を表示
fprintf('\n====================================\n');
fprintf('真値と推定値の比較\n');
fprintf('====================================\n');

fprintf('\nA_true\n');
disp(A_true);

fprintf('A_val\n');
disp(A_val);

fprintf('B_true\n');
disp(B_true);

fprintf('B_val\n');
disp(B_val);

%% 推定誤差
A_error = norm(A_val - A_true, 'fro');
B_error = norm(B_val - B_true, 'fro');

A_relative_error = ...
    A_error / max(norm(A_true, 'fro'), eps);

B_relative_error = ...
    B_error / max(norm(B_true, 'fro'), eps);

fprintf('AのFrobeniusノルム誤差      = %.6e\n', ...
    A_error);

fprintf('Aの相対誤差                 = %.6e\n', ...
    A_relative_error);

fprintf('BのFrobeniusノルム誤差      = %.6e\n', ...
    B_error);

fprintf('Bの相対誤差                 = %.6e\n', ...
    B_relative_error);

fprintf('真のAのmax|eig(A)|          = %.8f\n', ...
    max(abs(eig(A_true))));

fprintf('推定したAのmax|eig(A)|      = %.8f\n', ...
    result.max_abs_eig);

fprintf('収束したか                  = %d\n', ...
    result.converged);

fprintf('反復回数                    = %d\n', ...
    result.iteration);

%% 履歴をプロット
figure;

semilogy( ...
    1:result.iteration, ...
    result.objective_history, ...
    'o-', ...
    'LineWidth', 1.5);

grid on;
xlabel('Iteration');
ylabel('Objective');
title('目的関数の履歴');

figure;

semilogy( ...
    1:result.iteration, ...
    result.U_change_history, ...
    'o-', ...
    'LineWidth', 1.5);

hold on;

semilogy( ...
    1:result.iteration, ...
    result.P_change_history, ...
    's-', ...
    'LineWidth', 1.5);

grid on;
xlabel('Iteration');
ylabel('Relative change');
legend('U change', 'P change');
title('変数の変化量');

figure;

plot( ...
    1:result.iteration, ...
    result.max_eig_history, ...
    'o-', ...
    'LineWidth', 1.5);

hold on;

yline( ...
    rho_bar, ...
    '--', ...
    'rho\_bar', ...
    'LineWidth', 1.5);

grid on;
xlabel('Iteration');
ylabel('max |eig(A)|');
title('Aの最大固有値絶対値');

%% 最終確認
fprintf('\n====================================\n');

if result.max_abs_eig < rho_bar
    fprintf('安定性条件を満たしています。\n');
else
    fprintf(2, ...
        '警告：max|eig(A)|がrho_bar以上です。\n');
end

if all(isfinite(U_val), 'all') && ...
        all(isfinite(P_val), 'all')
    fprintf('UおよびPにNaNやInfはありません。\n');
else
    fprintf(2, ...
        '警告：UまたはPにNaNもしくはInfがあります。\n');
end

fprintf('テスト終了\n');
fprintf('====================================\n');
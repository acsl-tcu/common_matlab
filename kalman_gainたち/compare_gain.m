%% ============================================================
%  2つのフィードバックゲイン・閉ループ系の比較
%
%  各.matファイルに
%     A
%     B
%     K_full
%  が保存されていることを想定
% ============================================================

clear;
clc;
close all;

%% ===== ファイル読み込み =====


%% ===== ゲインファイルを選択 =====

% 1つ目
[file1, path1] = uigetfile('*.mat', '1つ目のゲインファイルを選択');

if isequal(file1,0)
    error('1つ目のファイル選択がキャンセルされました。');
end

fullpath1 = fullfile(path1, file1);


% 2つ目
[file2, path2] = uigetfile('*.mat', '2つ目のゲインファイルを選択');

if isequal(file2,0)
    error('2つ目のファイル選択がキャンセルされました。');
end

fullpath2 = fullfile(path2, file2);


%% ===== 読み込み =====

data1 = load(fullpath1);
data2 = load(fullpath2);

A1 = data1.A;
B1 = data1.B;
K1 = data1.K_full;

A2 = data2.A;
B2 = data2.B;
K2 = data2.K_full;


%% ===== 選択したファイル名を表示 =====

fprintf("===== 選択ファイル =====\n");
fprintf("Gain 1 : %s\n", fullpath1);
fprintf("Gain 2 : %s\n\n", fullpath2);

%% ============================================================
%  0. サイズ確認
% ============================================================

fprintf("===== 行列サイズ =====\n");

fprintf("Model 1 : A = %d x %d, B = %d x %d, K = %d x %d\n", ...
    size(A1,1), size(A1,2), ...
    size(B1,1), size(B1,2), ...
    size(K1,1), size(K1,2));

fprintf("Model 2 : A = %d x %d, B = %d x %d, K = %d x %d\n\n", ...
    size(A2,1), size(A2,2), ...
    size(B2,1), size(B2,2), ...
    size(K2,1), size(K2,2));


%% ============================================================
%  1. A, B, K の Frobenius ノルム比較
% ============================================================

fprintf("===== Frobeniusノルム比較 =====\n");

% --- A ---
A1_norm = norm(A1,'fro');
A2_norm = norm(A2,'fro');

A_diff = norm(A1-A2,'fro');

% 対称な相対差
A_rel_diff = A_diff / ((A1_norm + A2_norm)/2);


% --- B ---
B1_norm = norm(B1,'fro');
B2_norm = norm(B2,'fro');

B_diff = norm(B1-B2,'fro');
B_rel_diff = B_diff / ((B1_norm + B2_norm)/2);


% --- K ---
K1_norm = norm(K1,'fro');
K2_norm = norm(K2,'fro');

K_diff = norm(K1-K2,'fro');
K_rel_diff = K_diff / ((K1_norm + K2_norm)/2);


fprintf("A:\n");
fprintf("  ||A1||_F          = %.6g\n", A1_norm);
fprintf("  ||A2||_F          = %.6g\n", A2_norm);
fprintf("  ||A1-A2||_F       = %.6g\n", A_diff);
fprintf("  相対差             = %.3f %%\n\n", 100*A_rel_diff);

fprintf("B:\n");
fprintf("  ||B1||_F          = %.6g\n", B1_norm);
fprintf("  ||B2||_F          = %.6g\n", B2_norm);
fprintf("  ||B1-B2||_F       = %.6g\n", B_diff);
fprintf("  相対差             = %.3f %%\n\n", 100*B_rel_diff);

fprintf("K_full:\n");
fprintf("  ||K1||_F          = %.6g\n", K1_norm);
fprintf("  ||K2||_F          = %.6g\n", K2_norm);
fprintf("  ||K1-K2||_F       = %.6g\n", K_diff);
fprintf("  相対差             = %.3f %%\n\n", 100*K_rel_diff);


%% ============================================================
%  2. 各モデル＋各ゲインで閉ループを比較
%
%     Model 1 : A1 - B1*K1
%     Model 2 : A2 - B2*K2
% ============================================================

Acl1 = A1 - B1*K1;
Acl2 = A2 - B2*K2;

eig_cl1 = eig(Acl1);
eig_cl2 = eig(Acl2);

rho1 = max(abs(eig_cl1));
rho2 = max(abs(eig_cl2));

Acl_diff = norm(Acl1-Acl2,'fro');
Acl_rel_diff = Acl_diff / ...
    ((norm(Acl1,'fro') + norm(Acl2,'fro'))/2);

fprintf("===== 各モデルでの閉ループ比較 =====\n");

fprintf("Model 1:\n");
fprintf("  max |eig(A1-B1*K1)| = %.8f\n", rho1);

fprintf("Model 2:\n");
fprintf("  max |eig(A2-B2*K2)| = %.8f\n", rho2);

fprintf("\n閉ループ行列の差:\n");
fprintf("  ||Acl1-Acl2||_F      = %.6g\n", Acl_diff);
fprintf("  相対差                = %.3f %%\n\n", ...
    100*Acl_rel_diff);


%% ============================================================
%  3. 同じモデルに K1, K2 を適用
%     → Kの違いだけが閉ループに与える影響を見る
% ============================================================

fprintf("===== 同一モデル上でのK比較 =====\n");

if isequal(size(A1),size(A2)) && ...
        isequal(size(B1),size(B2)) && ...
        isequal(size(K1),size(K2))

    % ---------------------------------------------------------
    % A1, B1を共通プラントとする
    % ---------------------------------------------------------
    Acl_11 = A1 - B1*K1;
    Acl_12 = A1 - B1*K2;

    eig_11 = eig(Acl_11);
    eig_12 = eig(Acl_12);

    rho_11 = max(abs(eig_11));
    rho_12 = max(abs(eig_12));

    rel_Acl_model1 = norm(Acl_11-Acl_12,'fro') / ...
        ((norm(Acl_11,'fro') + norm(Acl_12,'fro'))/2);

    fprintf("--- A1, B1 に適用 ---\n");
    fprintf("K1 : max|eig| = %.8f\n", rho_11);
    fprintf("K2 : max|eig| = %.8f\n", rho_12);
    fprintf("閉ループ行列の相対差 = %.3f %%\n\n", ...
        100*rel_Acl_model1);


    % ---------------------------------------------------------
    % A2, B2を共通プラントとする
    % ---------------------------------------------------------
    Acl_21 = A2 - B2*K1;
    Acl_22 = A2 - B2*K2;

    eig_21 = eig(Acl_21);
    eig_22 = eig(Acl_22);

    rho_21 = max(abs(eig_21));
    rho_22 = max(abs(eig_22));

    rel_Acl_model2 = norm(Acl_21-Acl_22,'fro') / ...
        ((norm(Acl_21,'fro') + norm(Acl_22,'fro'))/2);

    fprintf("--- A2, B2 に適用 ---\n");
    fprintf("K1 : max|eig| = %.8f\n", rho_21);
    fprintf("K2 : max|eig| = %.8f\n", rho_22);
    fprintf("閉ループ行列の相対差 = %.3f %%\n\n", ...
        100*rel_Acl_model2);

else
    warning("行列サイズが異なるため、同一モデル上でのK比較は行いません。");
end


%% ============================================================
%  4. 閉ループ極のプロット
% ============================================================

theta = linspace(0,2*pi,500);

figure;
hold on;
grid on;
axis equal;

% 単位円
plot(cos(theta), sin(theta), 'k--', 'LineWidth', 1);

% 各閉ループ極
plot(real(eig_cl1), imag(eig_cl1), 'o', ...
    'MarkerSize', 8, 'LineWidth', 1.5);

plot(real(eig_cl2), imag(eig_cl2), 'x', ...
    'MarkerSize', 8, 'LineWidth', 1.5);

xlabel('Real');
ylabel('Imaginary');

title('Closed-loop eigenvalues');

legend( ...
    'Unit circle', ...
    'A_1-B_1K_1', ...
    'A_2-B_2K_2', ...
    'Location','best');
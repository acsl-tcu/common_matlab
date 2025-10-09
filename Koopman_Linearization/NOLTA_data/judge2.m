%% --- 主分析程序 (增强版) ---
% 功能: 1. 分析库普曼矩阵A的特征值，识别振荡模式及其主要影响的观测量。
%      2. 分析系统的可控性矩阵，包括秩、奇异值，并明确指出最难和
%         最容易控制的观测量。

clear; clc; close all;

%% 1. 加载和设置
% -------------------------------------------------------------------------
% 确保您的 'system_matrices.mat' 文件在此文件夹中
% 该文件应包含 A, B, C 三个矩阵
try
    load('2025-10-08_exp_ob25_1_code00_randompp.mat');
catch
    error('请确保 2025-10-08_exp_ob25_1_code00_randompp.mat 文件在当前文件夹中。');
end

% 假设的时间步长 (对于计算物理频率是必要的)
dt = 0.025; % 单位: 秒
A =est.A;
B =est.B;
C =est.C;
% 获取系统维度
n = size(A, 1);
fprintf('系统已加载，状态维度 n = %d\n', n);
fprintf('--------------------------------------------------\n\n');


%% 2. Koopman矩阵特征值分析
% -------------------------------------------------------------------------
disp('### 1. Koopman矩阵特征值分析 ###');

[V, D] = eig(A);
eigenvalues_A = diag(D);
oscillatory_indices = find(imag(eigenvalues_A) ~= 0);

if isempty(oscillatory_indices)
    disp('未发现引起振荡的特征值 (所有特征值均为实数)。');
else
    fprintf('发现 %d 个引起振荡的特征值。\n\n', length(oscillatory_indices));
    positive_imag_indices = oscillatory_indices(imag(eigenvalues_A(oscillatory_indices)) > 0);
    
    for i = 1:length(positive_imag_indices)
        idx = positive_imag_indices(i);
        lambda = eigenvalues_A(idx);
        frequency_rad_s = abs(angle(lambda)) / dt;
        frequency_hz = frequency_rad_s / (2*pi);
        
        fprintf('振荡模式 %d:\n', i);
        fprintf('  - 特征值: %f + %fi\n', real(lambda), imag(lambda));
        fprintf('  - 模: |λ| = %f (若<1则稳定, >1则不稳定)\n', abs(lambda));
        fprintf('  - 物理频率: %.2f Hz (%.2f rad/s)\n', frequency_hz, frequency_rad_s);
        
        eigenvector = V(:, idx);
        [max_participation, max_idx] = max(abs(eigenvector));
        
        fprintf('  - 在此振荡模式下，第 %d 个观测量(状态)的参与程度最大。\n\n', max_idx);
    end
end
fprintf('--------------------------------------------------\n\n');


%% 3. 可控性矩阵分析 (增强部分)
% -------------------------------------------------------------------------
disp('### 2. 可控性矩阵分析 ###');

% 构建可控性矩阵
CtrbMat = ctrb(A, B);

% --- 秩分析 ---
rank_C = rank(CtrbMat);
fprintf('可控性矩阵的秩为: %d\n', rank_C);
if rank_C == n
    fprintf('结论: 秩 = 系统维度 (%d)。系统是完全可控的。\n\n', n);
else
    fprintf('结论: 秩 < 系统维度 (%d)。系统不是完全可控的。\n\n', n);
end

% --- 奇异值与奇异向量分析 ---
% **【修复/增强】** 这里我们获取完整的SVD分解，特别是U矩阵
[U, S_mat, ~] = svd(CtrbMat);
S_vec = diag(S_mat); % 提取奇异值为向量

fprintf('奇异值分析 (控制的难易程度):\n');
fprintf('  - 奇异值 (从大到小): ');
fprintf('%.3f  ', S_vec);
fprintf('\n');
fprintf('  - 最大奇异值 (σ_max = %.3f): 对应最容易控制的方向。\n', S_vec(1));
fprintf('  - 最小奇异值 (σ_min = %.3f): 对应最难控制的方向。\n', S_vec(end));

% **【新增功能】** 通过分析U矩阵的列向量来确定具体的观测量
fprintf('\n奇异向量分析 (揭示具体的观测量):\n');

% 找到最难控制方向上的主导观测量
u_hard = U(:, end); % 对应最小奇异值的左奇异向量
[~, idx_hard] = max(abs(u_hard));
fprintf('  - **最难控制**的方向主要由【第 %d 个观测量】主导。\n', idx_hard);
fprintf('    (在该方向的向量中，此观测量的系数绝对值最大)\n');

% 找到最容易控制方向上的主导观测量
u_easy = U(:, 1); % 对应最大奇异值的左奇异向量
[~, idx_easy] = max(abs(u_easy));
fprintf('  - **最容易控制**的方向主要由【第 %d 个观测量】主导。\n', idx_easy);
fprintf('    (在该方向的向量中，此观测量的系数绝对值最大)\n');

fprintf('--------------------------------------------------\n');
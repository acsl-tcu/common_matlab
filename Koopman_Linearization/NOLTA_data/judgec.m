% Koopman 模型分析程序
% 从 mat 文件中加载 est.abc (假设里面有 est.A, est.B, est.C)
clear; clc;

% === 修改这里为你的文件名 ===
load('2026-01-30_exp_ob26_code00_randompp', 'est');  

A = est.A;
B = est.B;
C = est.C;

%% 1. 固有值分析
[eigVec, eigVal] = eig(A);
eigVals = diag(eigVal);

% 固有值模长、频率、阻尼（离散时间下）
magnitudes = abs(eigVals);          % 幅值（模长）
angles = angle(eigVals);            % 相角（弧度）
frequencies = angles / (2*pi);      % 频率 (周期化)

% 排序（模长从大到小）
[~, idx_sort] = sort(magnitudes, 'descend');
eigVals_sorted = eigVals(idx_sort);
magnitudes_sorted = magnitudes(idx_sort);
frequencies_sorted = frequencies(idx_sort);

fprintf('=== 固有值分析 ===\n');
for i = 1:length(eigVals)
    fprintf('第 %d 个固有值: %.4f + %.4fi | 幅值=%.4f | 频率=%.4f Hz\n', ...
        i, real(eigVals_sorted(i)), imag(eigVals_sorted(i)), ...
        magnitudes_sorted(i), frequencies_sorted(i));
end

%% 2. 模态对观测量的影响 (模态投影)
% 观测矩阵 * 模态向量
mode_influence = C * eigVec;  

% 取模长表示影响强弱
influence_strength = vecnorm(mode_influence, 2, 1);  

% 排序，找到影响最大的模态
[~, idx_infl] = sort(influence_strength, 'descend');

fprintf('\n=== 模态对观测量的影响排序 ===\n');
for i = 1:length(idx_infl)
    fprintf('第 %d 强的模态对应固有值 %.4f + %.4fi, 影响强度=%.4f\n', ...
        i, real(eigVals(idx_infl(i))), imag(eigVals(idx_infl(i))), influence_strength(idx_infl(i)));
end

%% 3. 可控性分析
Co = ctrb(A,B);
rank_Co = rank(Co);

fprintf('\n=== 可控性矩阵分析 ===\n');
fprintf('可控性矩阵秩: %d / %d\n', rank_Co, size(A,1));

% 奇异值分解
singVals = svd(Co);

% 按从大到小排序
[singVals_sorted, idx_sv] = sort(singVals, 'descend');

fprintf('\n奇异值大小排序:\n');
for i = 1:length(singVals_sorted)
    fprintf('第 %d 个奇异值: %.6f\n', i, singVals_sorted(i));
end

% 指出最大/最小奇异值
fprintf('\n最大奇异值: %.6f (编号 %d)\n', singVals_sorted(1), idx_sv(1));
fprintf('最小奇异值: %.6f (编号 %d)\n', singVals_sorted(end), idx_sv(end));

%% 4. 输出总结
fprintf('\n=== 总结 ===\n');
fprintf('系统共有 %d 个固有值，其中模长 > 1 的为不稳定因子。\n', length(eigVals));
unstable_idx = find(abs(eigVals) > 1);
if isempty(unstable_idx)
    fprintf('所有模态均在单位圆内，系统渐进稳定。\n');
else
    fprintf('以下模态不稳定:\n');
    for i = unstable_idx'
        fprintf('固有值 %.4f + %.4fi (模长 %.4f)\n', real(eigVals(i)), imag(eigVals(i)), abs(eigVals(i)));
    end
end

Co = ctrb(A,B); fprintf('rank Co = %d / %d\n', rank(Co), size(A,1));

% LQR params (tune these)
Q = diag(min(1, 1./(1e-6 + diag(C'*C)))); % example: state weighting from observability
R = 1e-2 * eye(size(B,2));

[K,S,eigs_cl] = dlqr(A,B,Q,R);  % discrete-time LQR
Acl = A - B*K;
fprintf('Closed-loop eigenvalues:\n'); disp(eig(Acl));

% simulate
x0 = randn(size(A,1),1)*0.1;
N = 300;
x = zeros(size(A,1),N); x(:,1)=x0;
for k=1:N-1
    x(:,k+1) = Acl * x(:,k);
end
figure; plot(0:N-1, vecnorm(x)); title('State norm under LQR closed-loop');
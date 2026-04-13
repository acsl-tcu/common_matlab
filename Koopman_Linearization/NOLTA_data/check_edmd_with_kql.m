%% Koopman 开环验证程序 (修复 Phase 102 第二段读取逻辑)
clear; clc; close all;

%% 1. 文件设置 (请修改此处文件名)
% -------------------------------------------------------------------------
logFileName   = 'kqlmpcdata_Log(26-Jan-2026_17_47_35).mat';  % <--- 修改为你的 Log 文件名
modelFileName = '2026-01-30_exp_ob26_code00_randompp.mat'; % <--- 修改为你的 Model 文件名
F_obs = @quaternions_all_kyo; 
% -------------------------------------------------------------------------

%% 2. 加载数据
fprintf('>>> 正在加载 Log 文件: %s ...\n', logFileName);
if ~isfile(logFileName), error('找不到 Log 文件'); end
load(logFileName); 

fprintf('>>> 正在加载 Model 文件: %s ...\n', modelFileName);
if ~isfile(modelFileName), error('找不到 Model 文件'); end
load(modelFileName); 

%% 3. 数据提取与对齐 (核心修改部分)
fprintf('>>> 正在解析飞行数据 (Phase 102 - Target: 2nd Segment) ...\n');

% 3.1 智能定位 Phase 102 的第二段
phases = log.Data.phase;
all_102_indices = find(phases == 102);

if isempty(all_102_indices)
    error('Log 中完全没有找到 Phase 102');
end

% 计算索引的差分，寻找不连续点 (Gap)
% 如果是连续的，diff 应该是 1。如果 diff > 1，说明中间断开了
gaps = find(diff(all_102_indices) > 1);

if isempty(gaps)
    % 情况 A: 只有一段连续的 102
    warning('Log 中只发现了一段连续的 Phase 102。将使用这一段。');
    start_k = all_102_indices(1);
    end_k   = all_102_indices(end);
else
    % 情况 B: 发现多段 102
    fprintf('    检测到 Phase 102 存在中断 (ATFL 初始化检查)。\n');
    fprintf('    正在锁定第二段 (实际飞行段)...\n');
    
    % 第一段结束于: gaps(1)
    % 第二段开始于: gaps(1) + 1
    idx_in_subset_start = gaps(1) + 1;
    
    % 检查是否有第三段
    if length(gaps) > 1
        % 如果有第三段，第二段结束于第二个 Gap 之前
        idx_in_subset_end = gaps(2);
    else
        % 如果没有第三段，第二段一直持续到最后
        idx_in_subset_end = length(all_102_indices);
    end
    
    % 映射回全局索引 k
    start_k = all_102_indices(idx_in_subset_start);
    end_k   = all_102_indices(idx_in_subset_end);
end

total_steps = end_k - start_k + 1;
fprintf('    [锁定成功] 提取区间: Index %d -> %d (共 %d 步)\n', start_k, end_k, total_steps);

% 3.2 初始化物理状态矩阵
X_phys_extracted = zeros(16, total_steps); 
Time_extracted   = zeros(1, total_steps);

% 3.3 循环提取数据
estimator_cell = log.Data.agent.estimator.result;
input_array    = log.Data.agent.input; 
time_array     = log.Data.t;

% 记录基准时间 (flight start time)
base_time = time_array(start_k);

for i = 1:total_steps
    % 全局索引
    k = start_k + i - 1;
    
    % --- 提取状态 ---
    % 必须使用 {1, k} 格式
    res_struct = estimator_cell{1, k};
    
    p_raw = res_struct.state.p; % [x; y; z]
    v_raw = res_struct.state.v; % [vx; vy; vz]
    w_raw = res_struct.state.w; % [wx; wy; wz]
    
    % 直接使用欧拉角 (根据你的说明 state.q 已经是 Roll Pitch Yaw)
    euler_angles = res_struct.state.q;
    if size(euler_angles, 1) == 1, euler_angles = euler_angles'; end
    
    % --- 提取输入 ---
    if iscell(input_array)
        u_curr = input_array{1, k}; % 或者 input_array{k}，取决于维度
    else
        u_curr = input_array(:, k); % 如果它偶尔是矩阵，保留这个兼容性
    end
    
    % --- 组装物理状态 X ---
    % [P; Q; V; W; U]
    X_phys_extracted(1:3, i)   = p_raw;
    X_phys_extracted(4:6, i)   = euler_angles;
    X_phys_extracted(7:9, i)   = v_raw;
    X_phys_extracted(10:12, i) = w_raw;
    X_phys_extracted(13:16, i) = u_curr;
    
    % --- 提取时间并归零 ---
    Time_extracted(i) = time_array(k) - base_time;
end

%% 4. 转换为 Koopman 状态 (Ground Truth)
fprintf('>>> 正在转换为 Koopman 45维状态 ...\n');

z_sample = F_obs(X_phys_extracted(:, 1));
n_koop = length(z_sample);
Z_true = zeros(n_koop, total_steps);

for i = 1:total_steps
    Z_true(:, i) = F_obs(X_phys_extracted(:, i));
end

%% 5. 执行开环预测 (Open-Loop Prediction)
fprintf('>>> 开始开环预测 (使用 Model A, B) ...\n');

% 验证全段
verify_len = total_steps; 

Z_pred = zeros(n_koop, verify_len);
Z_pred(:, 1) = Z_true(:, 1); % 初始状态对齐

for k = 1 : verify_len - 1
    z_k = Z_pred(:, k);
    
    % 使用从 Log 中提取的真实输入
    u_k = X_phys_extracted(13:16, k);
    
    % 离散预测
    z_next = est.A * z_k + est.B * u_k;
    
    Z_pred(:, k+1) = z_next;
end

%% 6. 绘图分析
fprintf('>>> 正在绘图 ...\n');
t_axis = Time_extracted(1:verify_len);

figure('Name', 'Log Phase 102 (2nd Segment) Verification', 'Color', 'w', 'Position', [100, 100, 1200, 800]);

% [1] 3D 轨迹
subplot(2, 2, 1);
plot3(Z_true(1,:), Z_true(2,:), Z_true(3,:), 'k', 'LineWidth', 1.5); hold on;
plot3(Z_pred(1,:), Z_pred(2,:), Z_pred(3,:), 'r--', 'LineWidth', 1.5);
title('3D 轨迹 (p1)'); legend('真实(Log)', '预测(Model)');
grid on; axis equal; xlabel('X'); ylabel('Y'); zlabel('Z');

% [2] 耦合项 p2 (X分量) - 螺旋/圆运动特征
subplot(2, 2, 2);
plot(t_axis, Z_true(4, :), 'k'); hold on;
plot(t_axis, Z_pred(4, :), 'r--');
title('耦合项 p2 (X) - 检查角速度耦合');
xlabel('Time (s) from flight start'); grid on;

% [3] 姿态 z1 (R11)
subplot(2, 2, 3);
plot(t_axis, Z_true(28, :), 'k'); hold on;
plot(t_axis, Z_pred(28, :), 'r--');
title('姿态 z1 (R11)');
grid on;

% [4] 误差范数
subplot(2, 2, 4);
err_vec = Z_true(:, 1:verify_len) - Z_pred;
plot(t_axis, vecnorm(err_vec), 'b', 'LineWidth', 1.5);
title('预测误差 (L2 Norm)');
grid on;

fprintf('验证完成。图表显示的是 Phase 102 的第二段(实际飞行段)。\n');
%% 7. 系统可控性分析 (Controllability Analysis)
% 目的：检查数据驱动得到的 A, B 矩阵是否能够完全控制所有 45 个状态
% 理论基础：Kalman Rank Condition & PBH Test

fprintf('\n===== 7. 系统可控性分析 (Controllability Analysis) =====\n');

if ~isfield(est, 'A') || ~isfield(est, 'B')
    error('未找到 est.A 或 est.B，无法进行分析。');
end

A = est.A;
B = est.B;
[n, m] = size(B);

fprintf('=== Koopman 系统分析报告 ===\n');
fprintf('状态维数 n: %d\n', n);
fprintf('输入维数 m: %d\n', m);

%% 2. 可控性分析 (Controllability)
% 核心公式: Rank([B, AB, ..., A^(n-1)B]) == n ?

disp('正在计算可控性矩阵...');
Co = ctrb(A, B); 
rank_Co = rank(Co, 1e-10); % 使用容差防止数值误差

fprintf('\n--- [1] 可控性分析 ---\n');
fprintf('A 矩阵的秩: %d (如果不满秩，仅代表信息压缩，不代表不可控)\n', rank(A));
fprintf('可控性矩阵秩: %d / %d\n', rank_Co, n);

if rank_Co == n
    fprintf('>> 结论: 系统 [完全可控]。\n');
    fprintf('   这意味着 MPC 理论上可以控制所有 45 个状态维度。\n');
else
    uncontrollable = n - rank_Co;
    fprintf('>> 结论: 系统 [不可控]！缺失 %d 个维度。\n', uncontrollable);
    fprintf('   这意味着有些 Koopman 状态（可能是高阶项）完全不受输入 U 影响。\n');
    fprintf('   建议: 检查训练数据是否包含了充分的激励 (Excitation)。\n');
end

%% 3. 稳定性分析 (Stability)
% 核心公式: abs(eig(A)) <= 1 ?

eigenvalues = eig(A);
mag_eig = abs(eigenvalues);
max_eig = max(mag_eig);

fprintf('\n--- [2] 稳定性分析 (预测模型) ---\n');
fprintf('A 矩阵最大特征值模长: %.6f\n', max_eig);

figure('Name', 'Eigenvalue Spectrum', 'Color', 'w');
theta = linspace(0, 2*pi, 100);
plot(cos(theta), sin(theta), 'k--'); hold on; % 单位圆
scatter(real(eigenvalues), imag(eigenvalues), 'r', 'filled');
axis equal; grid on;
xlabel('Real Axis'); ylabel('Imaginary Axis');
title(['A 矩阵特征值分布 (Max |?| = ' num2str(max_eig, '%.4f') ')']);
legend('单位圆 (Unit Circle)', '特征值 (Eigenvalues)');

if max_eig > 1.0001
    fprintf('>> [危险警告]: 模型 [不稳定] (Unstable)。\n');
    fprintf('   红点跑到了单位圆外面。这解释了为什么开环预测误差会指数爆炸。\n');
    fprintf('   MPC 可能会利用这个不稳定性产生错误的激进控制。\n');
    fprintf('   建议: 重新训练时加入正则化，或强制缩放 A = A / (max_eig + epsilon)。\n');
elseif max_eig > 0.99
    fprintf('>> [结论]: 模型 [边缘稳定]。这是 Koopman 算子的理想状态。\n');
else
    fprintf('>> [结论]: 模型 [稳定]。预测误差会随时间收敛。\n');
end

%% 4. PBH 测试 (定位不可控模态)
% 如果不可控，到底是哪个物理量不可控？

if rank_Co < n
    fprintf('\n--- [3] PBH 测试 (定位不可控模态) ---\n');
    [V, D] = eig(A);
    for i = 1:n
        lambda = D(i,i);
        % PBH 矩阵秩检测
        if rank([A - lambda*eye(n), B], 1e-6) < n
            fprintf('   不可控模式索引: %d (特征值: %.4f)\n', i, abs(lambda));
            % 简单推断物理意义
            [~, max_idx] = max(abs(V(:,i)));
            fprintf('   -> 主要关联状态索引: %d\n', max_idx);
        end
    end
end

%% Koopman 观测量分布诊断 (Diagnostics based on Observable Structure)
% 假设 n = 45
n = size(est.A, 1);

fprintf('\n===== 观测量分布可控性诊断 =====\n');

% 1. 计算 PBH 并分类
[V, D] = eig(est.A);
eigenvalues = diag(D);
tol = 1e-6; % 秩判定的容差

for i = 1:n
    lambda = eigenvalues(i);
    % PBH Test: Rank([A - lambda*I, B])
    if rank([est.A - lambda*eye(n), est.B], tol) < n
        
        % 找到该模式主要影响哪个物理状态
        [~, max_dim] = max(abs(V(:, i))); 
        
        % --- 你的观测量分布映射 ---
        if max_dim <= 3
            state_name = 'p1 (位置)'; severity = '【致命】不可忽略！';
        elseif max_dim <= 6
            state_name = 'p2 (Pos x Omega)'; severity = '【警告】需检查稳定性';
        elseif max_dim <= 9
            state_name = 'p3 (高阶耦合)'; severity = '【可忽略】';
        elseif max_dim <= 12
            state_name = 'y1 (速度)'; severity = '【致命】不可忽略！';
        elseif max_dim <= 15
            state_name = 'y2 (Vel x Omega)'; severity = '【警告】需检查稳定性';
        elseif max_dim <= 18
            state_name = 'y3 (高阶耦合)'; severity = '【可忽略】';
        elseif max_dim <= 27
            state_name = 'h (重力项)'; severity = '【可忽略】';
        elseif max_dim <= 36
            state_name = 'z1 (姿态 R)'; severity = '【致命】不可忽略！';
        elseif max_dim <= 45
            state_name = 'z2 (AngVel R*W)'; severity = '【通常可忽略】';
        else
            state_name = '未知状态'; severity = '?';
        end
        
        % --- 输出诊断结果 ---
        fprintf('  模式 #%d 不可控 | 模长: %.4f | 主要影响: Index %d (%s)\n', ...
            i, abs(lambda), max_dim, state_name);
        
        if contains(severity, '致命')
            fprintf('    -> %s 必须重新训练模型！\n', severity);
        elseif abs(lambda) > 1.001
            fprintf('    -> [危险] 虽然是高阶项，但它是不稳定的 (|eig|>1)。必须加正则化重训！\n');
        else
            fprintf('    -> [安全] 它是稳定的。建议将 Q 矩阵中 Index %d 的权重设为 0。\n', max_dim);
        end
        fprintf('-----------------------------------------------------\n');
    end
end
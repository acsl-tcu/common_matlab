
    % 输入 est 结构体需包含 est.A (45x45) 和 est.B (45x4)
    if ~isfield(est, 'A') || ~isfield(est, 'B')
        error('输入结构体必须包含 A 和 B 矩阵');
    end
    
    A = est.A;
    B = est.B;
    n = size(A, 1);
    
    % --- 1. 特征分解与 PBH 测试准备 ---
    [V, D] = eig(A);      % 右特征向量 (模态形状)
    [W, ~] = eig(A');     % 左特征向量 (用于 PBH 投影)
    eigenvalues = diag(D);
    
    % --- 2. 状态分块索引定义 (M=3, N=2) ---
    % 低次项 (关键物理量)
    idx_p1 = 1:3;         % 真实位置
    idx_y1 = 10:12;       % 真实速度
    idx_z1 = 28:36;       % 真实姿态 (vec(R))
    idx_z2 = 37:45;       % 真实角速度 (vec(R*Omega))
    
    % 高次项 (数学辅助项)
    idx_p_high = 4:9;     % p2, p3
    idx_y_high = 13:18;   % y2, y3
    idx_h_all  = 19:27;   % h1, h2, h3 (重力/漂移)
    
    fprintf('\n==================================================================================\n');
    fprintf('                  Koopman PBH 分析: 高次项 vs 低次项诊断\n');
    fprintf('==================================================================================\n');
    fprintf('%-4s | %-9s | %-10s | %-12s | %-s\n', ...
            'ID', 'Eigenvalue', 'Controllable', 'Risk Level', 'Dominant Component');
    fprintf('----------------------------------------------------------------------------------\n');

    tol = 1e-6; % 能控性判定阈值
    count_critical = 0;

    for k = 1:n
        lambda = eigenvalues(k);
        w = W(:, k); w = w / norm(w); % 归一化左特征向量
        v = V(:, k); v = v / norm(v); % 归一化右特征向量
        
        % --- PBH 测试 ---
        % 判据: |w' * B| > 0
        ctrl_val = norm(w' * B);
        is_controllable = ctrl_val > tol;
        
        ctrl_str = 'YES';
        if ~is_controllable
            ctrl_str = 'NO';
        end
        
        % 如果能控，直接跳过 (或根据需要显示)
        if is_controllable
            continue; 
        end
        
        % --- 3. 成分分析: 是高次项还是低次项? ---
        mag = abs(v);
        
        % 计算能量分布
        E_p1 = sum(mag(idx_p1));
        E_y1 = sum(mag(idx_y1));
        E_z1 = sum(mag(idx_z1));
        E_z2 = sum(mag(idx_z2));
        
        E_p_high = sum(mag(idx_p_high));
        E_y_high = sum(mag(idx_y_high));
        E_h_all  = sum(mag(idx_h_all));
        
        % 找出能量最大的部分
        [~, max_id] = max([E_p1, E_y1, E_z1, E_z2, E_p_high, E_y_high, E_h_all]);
        
        risk_str = '[SAFE]'; % 默认安全
        comp_str = '';
        
        switch max_id
            case 1 % p1 (低次位置)
                risk_str = '[CRITICAL]';
                comp_str = 'Low-order Pos (p1) -> 无法控制位置!';
                count_critical = count_critical + 1;
            case 2 % y1 (低次速度)
                risk_str = '[CRITICAL]';
                comp_str = 'Low-order Vel (y1) -> 无法控制速度!';
                count_critical = count_critical + 1;
            case 3 % z1 (姿态)
                risk_str = '[CRITICAL]';
                comp_str = 'Attitude (z1) -> 姿态不可控!';
                count_critical = count_critical + 1;
            case 4 % z2 (角速度)
                risk_str = '[CRITICAL]';
                comp_str = 'Ang Velocity (z2) -> 力矩失效!';
                count_critical = count_critical + 1;
            case 5 % p_high (高次位置)
                risk_str = '[SAFE]';
                comp_str = 'High-order Pos (p2, p3)';
            case 6 % y_high (高次速度)
                risk_str = '[SAFE]';
                comp_str = 'High-order Vel (y2, y3)';
            case 7 % h (重力/漂移)
                risk_str = '[SAFE]';
                comp_str = 'Gravity/Bias Terms (h)';
        end
        
        % 打印不可控模态详情
        if strcmp(risk_str, '[CRITICAL]')
            fprintf(2, '%-4d | %-9.4f | %-10s | %-12s | %s\n', ...
                k, abs(lambda), ctrl_str, risk_str, comp_str);
        else
            fprintf('%-4d | %-9.4f | %-10s | %-12s | %s\n', ...
                k, abs(lambda), ctrl_str, risk_str, comp_str);
        end
    end
    
    fprintf('----------------------------------------------------------------------------------\n');
    if count_critical == 0
        fprintf('\n[SUCCESS] 通过检查！所有不可控模态均为高次项或辅助项。\n');
        fprintf('          此模型可用于 MPC 控制。\n');
    else
        fprintf(2, '\n[FAILURE] 发现 %d 个关键低次项不可控！\n', count_critical);
        fprintf(2, '          这通常意味着 B 矩阵在悬停线性化时丢失了水平耦合，\n');
        fprintf(2, '          或者 EDMD 数据缺乏特定轴的激励。\n');
    end

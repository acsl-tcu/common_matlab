function xd_safe = cbf_reference_filter(agent, xd_nominal)
% cbf_reference_filter: 目標の加速度変化を累積積分し、LQRが追従可能な滑らかな回避軌道を生成する
% 【数理的追従不整合の完全解消版】

    xd_safe = xd_nominal; 

    if isempty(xd_nominal) || ~isfield(agent.estimator, 'result') || isempty(agent.estimator.result)
        return;
    end

    % 📈 安全な位置と速度を滑らかに累積していくための永続変数
    persistent p_safe_integral v_safe_integral;
    dt = 0.025; % タイムステップ

    model = agent.estimator.result;
    pL = model.state.pL; 
    vL = model.state.vL; 
    
    obs_array = ENVIRONMENT_OBSTACLE();
    A_matrix = []; b_vector = [];
    
    p_ref_nom = xd_nominal(1:3); 
    v_ref_nom = xd_nominal(5:7);  % 公称の目標速度
    a_ref_nom = xd_nominal(9:11); % 公称の目標加速度
    
    % 初回ステップ時のみ、現在の公称目標値で積分器を初期化
    if isempty(p_safe_integral)
        p_safe_integral = p_ref_nom;
        v_safe_integral = v_ref_nom;
    end
    
    % 論文準拠パラメータ (Table 1: gamma=2) [cite: 545]
    gamma_cbf = 2.0; alpha1 = 8.0; alpha2 = 8.0; 
    
    is_avoiding = false;
    for idx = 1:length(obs_array)
        p_obs  = obs_array(idx).p_obs;
        R_safe = obs_array(idx).R_safe; 
        
        dp = pL - p_obs;
        h_ij = sum(dp.^2) - R_safe^2; % [cite: 268]
        
        % 障害物の手前 3.0m（影響圏内）に入ったら安全バリア制約を適用
        if h_ij < 3.0 
            B_cbf = 2 * sum(vL.^2) + (alpha1 + alpha2) * (2 * dp' * vL) + alpha1 * alpha2 * h_ij;
            A_row = [-2*dp(1), -2*dp(2), -2*dp(3)]; 
            
            A_matrix = [A_matrix; A_row]; %#ok<AGROW>
            b_vector = [b_vector; B_cbf]; %#ok<AGROW>
            is_avoiding = true;
        end
    end
    
    if is_avoiding && ~isempty(A_matrix)
        % 🎯 論文式(29a)の思想：公称目標加速度からの変更量を最小化する [cite: 321]
        [a_ref_safe, ~, exitflag] = quadprog(eye(3), -a_ref_nom, A_matrix, b_vector, [], [], [-10;-10;-4], [10;10;4], [], optimoptions('quadprog', 'Display', 'off'));
        
        if exitflag == 1
            % 💥【核心】変更された安全加速度をオイラー積分して、滑らかな安全軌道を「累積生成」する
            v_safe_integral = v_safe_integral + a_ref_safe * dt;
            p_safe_integral = p_safe_integral + v_safe_integral * dt;
            
            % HLCに引き渡す軌道配列の「位置」「速度」「加速度」をすべて安全な一連の曲線へ置換
            xd_safe(1:3)   = p_safe_integral; 
            xd_safe(5:7)   = v_safe_integral;
            xd_safe(9:11)  = a_ref_safe;
            
            fprintf('🛡️ [CBF ACTIVE] TIME: %.3f | pL_y: %.3f | 補正安全目標 xd_y: %.3f\n', agent.time.t, pL(2), xd_safe(2));
            return;
        end
    end
    
    % 障害物圏外、またはQPが解けなかった場合は、公称軌道に滑らかに同期させて追従
    p_safe_integral = p_ref_nom;
    v_safe_integral = v_ref_nom;
end
function generate_ROM_CBF()
    % generate_ROM_CBF.m
    % Cohen, Molnar & Ames (2024) [CBF via Reduced-Order Models] に基づき、
    % 単積分器モデル (ROM) に対する解析的 CBF 速度修正関数を生成する。
    
    syms q1 q2 q3 real
    syms v_nom1 v_nom2 v_nom3 real
    syms p_obs1 p_obs2 p_obs3 real
    syms R_safe alpha_gain real
    
    q = [q1; q2; q3];
    v_nom = [v_nom1; v_nom2; v_nom3];
    p_obs = [p_obs1; p_obs2; p_obs3];
    
    % 安全関数 (インフレ化された半径 R_safe を使用)
    h_0 = (q - p_obs).' * (q - p_obs) - R_safe^2;
    
    % Lie微分
    Lg_h0 = 2 * (q - p_obs).';
    
    % CBF 制約: A * v <= b
    % Lg_h0 * v + alpha_gain * h_0 >= 0 
    % => -Lg_h0 * v <= alpha_gain * h_0
    A = -Lg_h0;
    b = alpha_gain * h_0;
    
    % 解析的 QP 解 (1つの不等式制約)
    % 制約違反度合い
    violation = A * v_nom - b;
    
    % 違反している場合のみ修正
    % smooth_max(x) = x * (x > 0) に相当するが、ここでは if-else を出力コードで書けるように
    % MATLAB function の中では単純に計算して、あとで max(0, violation) とする
    
    % denominator
    A_norm_sq = A * A.';
    
    % v_safe = v_nom - max(0, violation) / (A_norm_sq + eps) * A.'
    % ゼロ割りを防ぐ微小項
    eps_val = 1e-6;
    
    % 生成する式本体
    % violation をそのまま出力して、ラッパー内で max(0, violation) を取る形にする
    
    v_correct_dir = -A.' / (A_norm_sq + eps_val);
    
    filename = 'Analytical_ROM_CBF_Velocity.m';
    matlabFunction(violation, v_correct_dir, h_0, ...
        'File', filename, ...
        'Vars', {q, v_nom, p_obs, R_safe, alpha_gain}, ...
        'Outputs', {'violation', 'v_correct_dir', 'h_0'});
        
    disp('Successfully generated Analytical_ROM_CBF_Velocity.m');
end

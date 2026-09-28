% =========================================================================
% test_C6_BSPLINE_PHASE5.m
% 
% 【Phase 5 確定版: 連続運動学解析解・比張力下界・導関数上界・UAV境界 & 独立時間計測】
%  - 外部依存ゼロ (完全単一ファイル完結)
%  - 改修・確定内容:
%      1. ベンチマーク側 alpha_up の式を本体と完全統一 (max(1e-4, M_lb_s) を撤廃)
%         omega_up = Dj_s / M_lb_s;
%         alpha_up = (Ds_s / M_lb_s) + 2.0 * (Dj_s / M_lb_s) * omega_up + omega_up^2;
%      2. cert_str() を過度な解釈を排した [UNCERTIFIED] へ客観化
%      3. ウォームアップ対象を本測定関数 run_phase5_06_07_kernel_with_Mlb へ統一
%      4. 統計プロファイル (Median, P95, P99, Observed Max) を正しく保持し、
%         「本評価は WCET を与えるものではない」旨を明記
%      5. 学術的総括を「Feasibility Certificate アーキテクチャの確立」に厳密化
%
% 実行コマンド:
%   >> test_C6_BSPLINE_PHASE5
% =========================================================================
function test_C6_BSPLINE_PHASE5()
    clc;
    fprintf('=========================================================================================\n');
    fprintf('   Phase 5: Analytic Kinematics, Feasibility Certificates & Benchmark Suite [FROZEN]     \n');
    fprintf('=========================================================================================\n\n');

    %% 0. 物理幾何・テスト設定
    cfg.p = 7;                   % 次数 p = 7 (C6 連続)
    cfg.N_ctrl = 18;             % 制御点数 18 (7 + 4 + 7)
    cfg.N_fixed_start = 7;
    cfg.N_free = 4;
    cfg.N_fixed_end = 7;
    cfg.n_z = 12;
    cfg.qp_n_samples = 31;

    cfg.g_acc = 9.80665;         % [m/s^2]
    cfg.e3 = [0; 0; 1];

    cfg.sys.L_cable   = 1.00;    % 索長 [m]
    cfg.sys.R_payload = 0.15;    % ペイロード半径 [m]
    cfg.sys.R_cable   = 0.02;    % 索安全保護半径 [m]
    cfg.sys.R_uav     = 0.30;    % UAV 機体包絡半径 [m]
    cfg.d_margin      = 0.20;    % 要求表面安全マージン [m]

    t_start = 0.0;
    T_eval  = 15.26;
    nom_fun = @(t) get_test_nominal_state(t);
    D_start = nom_fun(t_start);
    D_end   = nom_fun(t_start + T_eval);

    obs.center = [0.3; 0.3; 20.0];
    obs.radii  = [1.2; 1.2; 2.0];
    obs.R      = eul2rotm_local([pi/6, pi/12, 0]);
    obs.S      = obs.R * diag(obs.radii.^2) * obs.R';
    obs.Sinv   = obs.R * diag(1.0 ./ (obs.radii.^2)) * obs.R';

    cache = init_phase5_static_cache(cfg);

    fprintf('[システム幾何・動力学設定]\n');
    fprintf(' - 索長 L_cable         : %.2f m\n', cfg.sys.L_cable);
    fprintf(' - 重力加速度 g         : %.5f m/s^2 (Z軸下向き作用)\n', cfg.g_acc);
    fprintf(' - 評価ホライズン T     : %.2f s\n', T_eval);
    fprintf(' - 軌道連続性           : C6 連続 (4階導関数 Snap s_L(t) まで滑らかに定義)\n\n');

    %% ====================================================================
    % [Step 1] Phase 4-06 確定 C6 軌道モデルの取得
    % ====================================================================
    fprintf('=========================================================================================\n');
    fprintf(' [Step 1] Phase 4-06 確定 C6 軌道モデルの取得\n');
    fprintf('=========================================================================================\n');

    res_p2 = generate_phase2_c6_trajectory(T_eval, D_start, D_end, obs, cfg, cache);
    model = res_p2.model;
    P_ctrl = res_p2.P_ctrl;

    fprintf('  - C6 局所変形強度 alpha*              : %6.4f m\n', res_p2.alpha_star);
    fprintf('  - 軌道制御点数                        : %d 点\n\n', cfg.N_ctrl);

    %% ====================================================================
    % [Phase 5-01 & 5-02] 索姿勢・高次微分の閉形式解析評価 (N=5,001)
    % ====================================================================
    fprintf('=========================================================================================\n');
    fprintf(' [Phase 5-01 & 5-02] 索姿勢 (n_T, dn_T, d2n_T) の閉形式解析評価 & 比張力走査\n');
    fprintf('=========================================================================================\n');

    N_dense = 5001;
    u_dense = linspace(0.0, 1.0, N_dense)';
    t_dense = u_dense * T_eval;

    p_L_dense = zeros(N_dense, 3);
    v_L_dense = zeros(N_dense, 3);
    a_L_dense = zeros(N_dense, 3);
    j_L_dense = zeros(N_dense, 3);
    s_L_dense = zeros(N_dense, 3);

    n_T_dense   = zeros(N_dense, 3);
    dn_T_dense  = zeros(N_dense, 3);
    d2n_T_dense = zeros(N_dense, 3);
    gamma_T_dense = zeros(N_dense, 1);

    for j = 1:N_dense
        u = u_dense(j);
        p_L_dense(j, :) = eval_bspline_deriv(model, u, P_ctrl, 0)';
        v_L_dense(j, :) = eval_bspline_deriv(model, u, P_ctrl, 1)';
        a_L_dense(j, :) = eval_bspline_deriv(model, u, P_ctrl, 2)';
        j_L_dense(j, :) = eval_bspline_deriv(model, u, P_ctrl, 3)';
        s_L_dense(j, :) = eval_bspline_deriv(model, u, P_ctrl, 4)';

        [nT, dnT, d2nT, r] = eval_cable_kinematics( ...
            a_L_dense(j, :)', j_L_dense(j, :)', s_L_dense(j, :)', cfg.g_acc, cfg.e3);

        n_T_dense(j, :)   = nT';
        dn_T_dense(j, :)  = dnT';
        d2n_T_dense(j, :) = d2nT';
        gamma_T_dense(j)  = r;
    end

    min_gamma_T = min(gamma_T_dense);
    max_gamma_T = max(gamma_T_dense);
    mean_gamma_T = mean(gamma_T_dense);

    fprintf('  【比張力指標 gamma_T(t) = ||a_L + g*e3|| [N/kg = m/s^2] 走査】\n');
    fprintf('    - 5,001点数値参照 最小比張力        : %7.4f m/s^2 (静止重力比: %5.2f%%)\n', ...
        min_gamma_T, (min_gamma_T / cfg.g_acc) * 100.0);
    fprintf('    - 5,001点数値参照 最大比張力        : %7.4f m/s^2 (静止重力比: %5.2f%%)\n', ...
        max_gamma_T, (max_gamma_T / cfg.g_acc) * 100.0);
    fprintf('    - 5,001点数値参照 平均比張力        : %7.4f m/s^2\n', mean_gamma_T);
    fprintf('  ---------------------------------------------------------------------------------------\n');
    fprintf('  >>> 考察:\n');
    fprintf('      5,001点高密度数値参照では gamma_T の最小値 %7.4f m/s^2 が観測され、\n', min_gamma_T);
    fprintf('      索張力特異点 (0 m/s^2) から十分離れていることを数値的に確認した。\n\n');

    %% ====================================================================
    % [Phase 5-04] 解析微分 (dn_T, d2n_T) vs 高精度数値中心差分 検証
    % ====================================================================
    fprintf('=========================================================================================\n');
    fprintf(' [Phase 5-04] 索姿勢解析微分 (dn_T, d2n_T) vs 高精度数値中心差分 検証\n');
    fprintf('=========================================================================================\n');

    N_check = 50;
    idx_check = round(linspace(100, N_dense - 100, N_check));
    dt_step = 1e-6;

    max_err_dn_T  = 0.0;
    max_err_d2n_T = 0.0;

    for c = 1:N_check
        k = idx_check(c);
        t_cur = t_dense(k);

        u_p = (t_cur + dt_step) / T_eval;
        a_L_p = eval_bspline_deriv(model, u_p, P_ctrl, 2);
        j_L_p = eval_bspline_deriv(model, u_p, P_ctrl, 3);
        s_L_p = eval_bspline_deriv(model, u_p, P_ctrl, 4);
        [nT_p, dnT_p, ~, ~] = eval_cable_kinematics(a_L_p, j_L_p, s_L_p, cfg.g_acc, cfg.e3);

        u_m = (t_cur - dt_step) / T_eval;
        a_L_m = eval_bspline_deriv(model, u_m, P_ctrl, 2);
        j_L_m = eval_bspline_deriv(model, u_m, P_ctrl, 3);
        s_L_m = eval_bspline_deriv(model, u_m, P_ctrl, 4);
        [nT_m, dnT_m, ~, ~] = eval_cable_kinematics(a_L_m, j_L_m, s_L_m, cfg.g_acc, cfg.e3);

        dnT_num  = (nT_p - nT_m) / (2.0 * dt_step);
        d2nT_num = (dnT_p - dnT_m) / (2.0 * dt_step);

        max_err_dn_T  = max(max_err_dn_T, norm(dn_T_dense(k, :)' - dnT_num));
        max_err_d2n_T = max(max_err_d2n_T, norm(d2n_T_dense(k, :)' - d2nT_num));
    end

    pass_dn  = (max_err_dn_T < 1e-7);
    pass_d2n = (max_err_d2n_T < 1e-6);

    fprintf('  - 1階微分一致誤差 Max ||dn_T_analytic - dn_T_num||   : %9.2e rad/s   -> %s\n', ...
        max_err_dn_T, pass_str(pass_dn));
    fprintf('  - 2階微分一致誤差 Max ||d2n_T_analytic - d2n_T_num|| : %9.2e rad/s^2 -> %s\n', ...
        max_err_d2n_T, pass_str(pass_d2n));

    if ~(pass_dn && pass_d2n)
        error('Cable kinematics analytic derivative verification failed.');
    end
    fprintf('  ---------------------------------------------------------------------------------------\n');
    fprintf('  >>> 結論: C6 軌道から導出した索方向微分 (dn_T, d2n_T) の解析式と数値微分が\n');
    fprintf('            高精度で一致し、高次運動学変換の実装整合性を確認した。\n\n');

    %% ====================================================================
    % [Phase 5-03] UAV 閉形式運動学 & 質量比 mu に対する感度評価
    % ====================================================================
    fprintf('=========================================================================================\n');
    fprintf(' [Phase 5-03] UAV 閉形式運動学 & 質量比 mu = m_L / m_Q に対する感度評価\n');
    fprintf('=========================================================================================\n');

    p_Q_dense = p_L_dense + cfg.sys.L_cable * n_T_dense;
    v_Q_dense = v_L_dense + cfg.sys.L_cable * dn_T_dense;
    a_Q_dense = a_L_dense + cfg.sys.L_cable * d2n_T_dense;

    mu_list = [0.0, 0.2, 0.5, 1.0];
    n_mu = length(mu_list);
    mu_profiles = struct('mu', {}, 'f_min', {}, 'f_max', {}, 'tilt_max_deg', {}, 'tilt_mean_deg', {});

    for m_idx = 1:n_mu
        mu = mu_list(m_idx);
        f_vec = zeros(N_dense, 1);
        tilt_vec = zeros(N_dense, 1);

        for j = 1:N_dense
            a_Q = a_Q_dense(j, :)';
            W   = a_L_dense(j, :)' + cfg.g_acc * cfg.e3;

            gamma_Q = (a_Q + cfg.g_acc * cfg.e3) + mu * W;
            norm_gamma_Q = norm(gamma_Q);
            f_vec(j) = norm_gamma_Q;

            cos_tilt = dot(cfg.e3, gamma_Q) / norm_gamma_Q;
            cos_tilt = max(-1.0, min(1.0, cos_tilt));
            tilt_vec(j) = acos(cos_tilt) * (180.0 / pi);
        end

        mu_profiles(m_idx).mu            = mu;
        mu_profiles(m_idx).f_min         = min(f_vec);
        mu_profiles(m_idx).f_max         = max(f_vec);
        mu_profiles(m_idx).tilt_max_deg  = max(tilt_vec);
        mu_profiles(m_idx).tilt_mean_deg = mean(tilt_vec);
    end

    fprintf('  【感度解析用テスト値 mu に対する UAV 比推力・傾斜角一覧】\n');
    fprintf('  ---------------------------------------------------------------------------------------\n');
    fprintf('   質量比 mu  | 比推力 min [N/kg] | 比推力 max [N/kg] | 最大傾斜角 [deg] | 平均傾斜角 [deg]\n');
    fprintf('  ---------------------------------------------------------------------------------------\n');
    for m_idx = 1:n_mu
        fprintf('    mu = %3.1f  |      %7.4f      |      %7.4f      |     %6.2f deg   |     %6.2f deg\n', ...
            mu_profiles(m_idx).mu, mu_profiles(m_idx).f_min, mu_profiles(m_idx).f_max, ...
            mu_profiles(m_idx).tilt_max_deg, mu_profiles(m_idx).tilt_mean_deg);
    end
    fprintf('  ---------------------------------------------------------------------------------------\n');
    fprintf('  (注: 実機 feasibility 判定時には実際の m_L/m_Q、最大推力、最大傾斜角を適用する必要がある)\n\n');

    %% ====================================================================
    % [Phase 5-05] 比張力 gamma_T(t) の連続時間 Feasibility Certificate
    % ====================================================================
    fprintf('=========================================================================================\n');
    fprintf(' [Phase 5-05] 比張力 gamma_T(t) の連続時間 Certificate (支持超平面双対定理)\n');
    fprintf('=========================================================================================\n');

    unique_knots = cache.unique_knots;
    n_spans = length(unique_knots) - 1;
    A_ctrl = (cache.K_a * P_ctrl) / (T_eval^2);
    p_acc = cfg.p - 2;
    knots_acc = cache.knots_acc;

    tension_cert = struct('u_s', {}, 'u_e', {}, 'M_LB', {}, 'dense_min', {}, ...
                          'gap', {}, 'is_certified', {}, 'is_consistent', {});

    t_cert_start = tic;
    span_count = 0;
    all_tension_certified = true;
    all_tension_consistent = true;

    for s = 1:n_spans
        u_s = unique_knots(s);
        u_e = unique_knots(s + 1);
        if u_e <= u_s + 1e-12, continue; end
        span_count = span_count + 1;
        u_mid = 0.5 * (u_s + u_e);

        span_a = find_span_local(u_mid, knots_acc, p_acc, cfg.N_ctrl - 2);
        A_act = A_ctrl((span_a - p_acc):span_a, :);
        C_acc = cache.bezier_extract_acc{span_count};
        A_bez = C_acc * A_act;
        W_bez = A_bez + repmat((cfg.g_acc * cfg.e3)', size(A_bez, 1), 1);

        M_LB = compute_certified_tension_lower_bound(W_bez);

        idx_span = find(u_dense >= u_s & u_dense <= u_e);
        min_dense_span = min(gamma_T_dense(idx_span));
        gap_span = min_dense_span - M_LB;

        % Certificate 成立判定と数値参照整合性チェックの完全分離
        is_cert = (M_LB > 0.0);
        is_cons = (gap_span >= -1e-6);

        if ~is_cert, all_tension_certified = false; end
        if ~is_cons, all_tension_consistent = false; end

        tension_cert(span_count).u_s           = u_s;
        tension_cert(span_count).u_e           = u_e;
        tension_cert(span_count).M_LB          = M_LB;
        tension_cert(span_count).dense_min     = min_dense_span;
        tension_cert(span_count).gap           = gap_span;
        tension_cert(span_count).is_certified  = is_cert;
        tension_cert(span_count).is_consistent = is_cons;
    end
    t_cert_total_ms = toc(t_cert_start) * 1000.0;

    min_global_M_LB = min([tension_cert.M_LB]);

    fprintf('  【全 %d ノット区間における 比張力 gamma_T(t) Certificate スキャン結果】\n', span_count);
    fprintf('  ---------------------------------------------------------------------------------------------------------\n');
    fprintf('   Span |  区間 [u_s, u_e]  | M_LB [m/s^2] | dense_min [m/s^2] | ギャップ [m/s^2] | Certificate | 参照整合性\n');
    fprintf('  ---------------------------------------------------------------------------------------------------------\n');
    for s = 1:span_count
        fprintf('   %3d  | [%.4f, %.4f]  |    %7.4f   |      %7.4f      |     %8.2e    |    %s   |    %s\n', ...
            s, tension_cert(s).u_s, tension_cert(s).u_e, ...
            tension_cert(s).M_LB, tension_cert(s).dense_min, ...
            tension_cert(s).gap, pass_str(tension_cert(s).is_certified), pass_str(tension_cert(s).is_consistent));
    end
    fprintf('  ---------------------------------------------------------------------------------------------------------\n');
    fprintf('  - 全時間 大域保証比張力下界 (Global Certified Bound): %7.4f m/s^2 > 0\n', min_global_M_LB);
    fprintf('  - 5,001 点数値参照下回り件数 (Consistency Violation) : %d 件\n', ...
        sum([tension_cert.gap] < -1e-6));
    fprintf('  - Certificate 計算所要時間 (全区間)                 : %6.3f ms (%6.2f μs / span)\n', ...
        t_cert_total_ms, (t_cert_total_ms / span_count) * 1000.0);
    fprintf('  - 索張力特異点回避・緊張性 連続時間証明             : %s\n\n', ...
        cert_str(all_tension_certified && (min_global_M_LB > 0)));

    %% ====================================================================
    % [Phase 5-06] ペイロード高次導関数 (v_L, a_L, j_L, s_L) 連続時間上界 Certificate
    % ====================================================================
    fprintf('=========================================================================================\n');
    fprintf(' [Phase 5-06] ペイロード高次導関数の連続時間上界 Certificate (Bézier 凸包ノルム上界)\n');
    fprintf('=========================================================================================\n');

    V_ctrl = (cache.K_v * P_ctrl) / (T_eval^1);
    A_ctrl_p = (cache.K_a * P_ctrl) / (T_eval^2);
    J_ctrl = (cache.K_j * P_ctrl) / (T_eval^3);
    S_ctrl = (cache.K_s * P_ctrl) / (T_eval^4);

    bound_v_spans = zeros(span_count, 1);
    bound_a_spans = zeros(span_count, 1);
    bound_j_spans = zeros(span_count, 1);
    bound_s_spans = zeros(span_count, 1);

    dense_max_v_spans = zeros(span_count, 1);
    dense_max_a_spans = zeros(span_count, 1);
    dense_max_j_spans = zeros(span_count, 1);
    dense_max_s_spans = zeros(span_count, 1);

    for s = 1:span_count
        u_s = unique_knots(s);
        u_e = unique_knots(s + 1);
        if u_e <= u_s + 1e-12, continue; end
        u_mid = 0.5 * (u_s + u_e);

        span_v = find_span_local(u_mid, cache.knots_v, 6, cfg.N_ctrl - 1);
        V_bez = cache.bezier_extract_v{s} * V_ctrl((span_v - 6):span_v, :);
        bound_v_spans(s) = max(sqrt(sum(V_bez.^2, 2)));

        span_a = find_span_local(u_mid, cache.knots_acc, 5, cfg.N_ctrl - 2);
        A_bez = cache.bezier_extract_acc{s} * A_ctrl_p((span_a - 5):span_a, :);
        bound_a_spans(s) = max(sqrt(sum(A_bez.^2, 2)));

        span_j = find_span_local(u_mid, cache.knots_j, 4, cfg.N_ctrl - 3);
        J_bez = cache.bezier_extract_j{s} * J_ctrl((span_j - 4):span_j, :);
        bound_j_spans(s) = max(sqrt(sum(J_bez.^2, 2)));

        span_s = find_span_local(u_mid, cache.knots_s, 3, cfg.N_ctrl - 4);
        S_bez = cache.bezier_extract_s{s} * S_ctrl((span_s - 3):span_s, :);
        bound_s_spans(s) = max(sqrt(sum(S_bez.^2, 2)));

        idx_in = find(u_dense >= u_s & u_dense <= u_e);
        dense_max_v_spans(s) = max(sqrt(sum(v_L_dense(idx_in, :).^2, 2)));
        dense_max_a_spans(s) = max(sqrt(sum(a_L_dense(idx_in, :).^2, 2)));
        dense_max_j_spans(s) = max(sqrt(sum(j_L_dense(idx_in, :).^2, 2)));
        dense_max_s_spans(s) = max(sqrt(sum(s_L_dense(idx_in, :).^2, 2)));
    end

    global_bound_v = max(bound_v_spans);
    global_bound_a = max(bound_a_spans);
    global_bound_j = max(bound_j_spans);
    global_bound_s = max(bound_s_spans);

    global_dense_v = max(dense_max_v_spans);
    global_dense_a = max(dense_max_a_spans);
    global_dense_j = max(dense_max_j_spans);
    global_dense_s = max(dense_max_s_spans);

    inconsist_v = sum(dense_max_v_spans > bound_v_spans + 1e-6);
    inconsist_a = sum(dense_max_a_spans > bound_a_spans + 1e-6);
    inconsist_j = sum(dense_max_j_spans > bound_j_spans + 1e-6);
    inconsist_s = sum(dense_max_s_spans > bound_s_spans + 1e-6);

    fprintf('  【ペイロード高次導関数 連続時間大域保証上界 (Global Certified Upper Bounds)】\n');
    fprintf('  ---------------------------------------------------------------------------------------------------\n');
    fprintf('   状態量 (階数)      | 連続時間保証上界 D_max | 5,001点参照最大値 | 保守性ギャップ | 健全性整合判定\n');
    fprintf('  ---------------------------------------------------------------------------------------------------\n');
    fprintf('   速度   ||v_L(t)||  |      %7.4f m/s      |     %7.4f m/s     |   %+7.4f m/s   | %s (%d件超過)\n', ...
        global_bound_v, global_dense_v, global_bound_v - global_dense_v, pass_str(inconsist_v == 0), inconsist_v);
    fprintf('   加速度 ||a_L(t)||  |      %7.4f m/s^2    |     %7.4f m/s^2   |   %+7.4f m/s^2 | %s (%d件超過)\n', ...
        global_bound_a, global_dense_a, global_bound_a - global_dense_a, pass_str(inconsist_a == 0), inconsist_a);
    fprintf('   Jerk   ||j_L(t)||  |      %7.4f m/s^3    |     %7.4f m/s^3   |   %+7.4f m/s^3 | %s (%d件超過)\n', ...
        global_bound_j, global_dense_j, global_bound_j - global_dense_j, pass_str(inconsist_j == 0), inconsist_j);
    fprintf('   Snap   ||s_L(t)||  |      %7.4f m/s^4    |     %7.4f m/s^4   |   %+7.4f m/s^4 | %s (%d件超過)\n', ...
        global_bound_s, global_dense_s, global_bound_s - global_dense_s, pass_str(inconsist_s == 0), inconsist_s);
    fprintf('  ---------------------------------------------------------------------------------------------------\n');
    fprintf('  >>> 結論: 導関数 Bézier 制御点の凸包性により、サンプリングを介さずに\n');
    fprintf('            速度・加速度・Jerk・Snap に対する連続時間保証上界を有限代数演算で構成した。\n\n');

    %% ====================================================================
    % [Phase 5-07] 索角速度・角加速度 & UAV 状態量の連続時間保証上界導出
    % ====================================================================
    fprintf('=========================================================================================\n');
    fprintf(' [Phase 5-07] 索角速度・角加速度 & UAV 加速度・比推力の連続時間保証上界\n');
    fprintf('=========================================================================================\n');

    bound_omega_spans = zeros(span_count, 1);
    bound_alpha_spans = zeros(span_count, 1);
    bound_aQ_spans    = zeros(span_count, 1);
    bound_gammaQ_spans = zeros(span_count, 1); % mu=0.5

    dense_max_omega = zeros(span_count, 1);
    dense_max_alpha = zeros(span_count, 1);
    dense_max_aQ    = zeros(span_count, 1);

    for s = 1:span_count
        M_lb_s = tension_cert(s).M_LB;
        if M_lb_s <= 0.0
            error('Phase 5-07 cannot be evaluated because M_LB <= 0.');
        end

        Dj_s = bound_j_spans(s);
        Ds_s = bound_s_spans(s);
        Da_s = bound_a_spans(s);

        % 1. 索角速度上界: ||dn_T|| <= D_j / M_LB
        omega_up = Dj_s / M_lb_s;
        bound_omega_spans(s) = omega_up;

        % 2. 索角加速度上界: ||d2n_T|| <= D_s/M_LB + 2*(D_j/M_LB)*omega_up + omega_up^2
        %    (数理導出: ||d2n_T|| <= D_s/M_LB + 3*(D_j/M_LB)^2)
        alpha_up = (Ds_s / M_lb_s) + 2.0 * (Dj_s / M_lb_s) * omega_up + (omega_up^2);
        bound_alpha_spans(s) = alpha_up;

        % 3. UAV 加速度上界: ||a_Q|| <= ||a_L|| + L * ||d2n_T||
        aQ_up = Da_s + cfg.sys.L_cable * alpha_up;
        bound_aQ_spans(s) = aQ_up;

        % 4. UAV 比推力上界 (mu=0.5): ||gamma_Q|| <= ||a_Q + g*e3|| + mu*||W||
        W_max_s = Da_s + cfg.g_acc;
        bound_gammaQ_spans(s) = (aQ_up + cfg.g_acc) + 0.5 * W_max_s;

        % 数値参照最大値
        u_s = unique_knots(s); u_e = unique_knots(s + 1);
        idx_in = find(u_dense >= u_s & u_dense <= u_e);
        dense_max_omega(s) = max(sqrt(sum(dn_T_dense(idx_in, :).^2, 2)));
        dense_max_alpha(s) = max(sqrt(sum(d2n_T_dense(idx_in, :).^2, 2)));
        dense_max_aQ(s)    = max(sqrt(sum(a_Q_dense(idx_in, :).^2, 2)));
    end

    global_bound_omega  = max(bound_omega_spans);
    global_bound_alpha  = max(bound_alpha_spans);
    global_bound_aQ     = max(bound_aQ_spans);
    global_bound_gammaQ = max(bound_gammaQ_spans);

    fprintf('  【索・UAV 状態量 連続時間大域保証上界】\n');
    fprintf('  ---------------------------------------------------------------------------------------------------\n');
    fprintf('   状態量             | 連続時間保証上界 | 5,001点参照最大値 | 保守性ギャップ | 健全性整合判定\n');
    fprintf('  ---------------------------------------------------------------------------------------------------\n');
    fprintf('   索角速度 ||dn_T||  |   %7.4f rad/s  |   %7.4f rad/s   | %+7.4f rad/s | %s\n', ...
        global_bound_omega, max(dense_max_omega), global_bound_omega - max(dense_max_omega), ...
        pass_str(all(dense_max_omega <= bound_omega_spans + 1e-6)));
    fprintf('   索角加速度 ||d2n_T|| | %7.4f rad/s^2|   %7.4f rad/s^2 | %+7.4f rad/s^2| %s\n', ...
        global_bound_alpha, max(dense_max_alpha), global_bound_alpha - max(dense_max_alpha), ...
        pass_str(all(dense_max_alpha <= bound_alpha_spans + 1e-6)));
    fprintf('   UAV加速度 ||a_Q||  |   %7.4f m/s^2  |   %7.4f m/s^2   | %+7.4f m/s^2 | %s\n', ...
        global_bound_aQ, max(dense_max_aQ), global_bound_aQ - max(dense_max_aQ), ...
        pass_str(all(dense_max_aQ <= bound_aQ_spans + 1e-6)));
    fprintf('   UAV比推力 (mu=0.5) |   %7.4f N/kg   |   %7.4f N/kg   | %+7.4f N/kg  | %s\n', ...
        global_bound_gammaQ, max(mu_profiles(3).f_max), global_bound_gammaQ - max(mu_profiles(3).f_max), ...
        pass_str(max(mu_profiles(3).f_max) <= global_bound_gammaQ + 1e-6));
    fprintf('  ---------------------------------------------------------------------------------------------------\n');
    fprintf('  >>> 結論: 比張力下界 M_LB と導関数上界の代数的合成により、\n');
    fprintf('            UAV 加速度および比推力の連続時間上限を厳密に外包することを確認。\n\n');

    %% ====================================================================
    % [Step 8] Certificate カーネル独立統計ベンチマーク (N=1,000 試行)
    % ====================================================================
    fprintf('=========================================================================================\n');
    fprintf(' [Step 8] Certificate カーネル独立統計ベンチマーク (N=1,000 試行, 参照評価除外)\n');
    fprintf('=========================================================================================\n');

    N_bench = 1000;
    lat_505 = zeros(N_bench, 1);
    lat_506_07 = zeros(N_bench, 1);
    lat_total = zeros(N_bench, 1);

    % ★修正 3: ウォームアップ対象を本測定関数 run_phase5_06_07_kernel_with_Mlb に統一
    for w = 1:50
        M_lb_warm = run_phase5_05_kernel(A_ctrl, cfg, cache);
        run_phase5_06_07_kernel_with_Mlb(V_ctrl, A_ctrl_p, J_ctrl, S_ctrl, M_lb_warm, cfg, cache);
    end

    % 本測定
    for b = 1:N_bench
        % Kernel A: Phase 5-05 比張力 Certificate (凸QP M_LB)
        t_a = tic;
        M_lb_vec = run_phase5_05_kernel(A_ctrl, cfg, cache);
        lat_505(b) = toc(t_a) * 1000.0;

        % Kernel B: Phase 5-06 & 5-07 上界 Certificate (Bézier 制御点ノルム代数評価)
        t_b = tic;
        run_phase5_06_07_kernel_with_Mlb(V_ctrl, A_ctrl_p, J_ctrl, S_ctrl, M_lb_vec, cfg, cache);
        lat_506_07(b) = toc(t_b) * 1000.0;

        lat_total(b) = lat_505(b) + lat_506_07(b);
    end

    fprintf('  【Certificate カーネル別 演算レイテンシ統計プロファイル (全 11 スパン一括)】\n');
    fprintf('  -------------------------------------------------------------------------------------------------\n');
    fprintf('   カーネル                       | Mean [ms] | Median [ms] | P95 [ms] | P99 [ms] | Obs.Max [ms]\n');
    fprintf('  -------------------------------------------------------------------------------------------------\n');
    fprintf('   Kernel A (5-05: 凸QP M_LB)     |  %7.4f  |   %7.4f   |  %7.4f |  %7.4f |   %7.4f\n', ...
        mean(lat_505), median(lat_505), prctile(lat_505, 95), prctile(lat_505, 99), max(lat_505));
    fprintf('   Kernel B (5-06/07: Bézier上界) |  %7.4f  |   %7.4f   |  %7.4f |  %7.4f |   %7.4f\n', ...
        mean(lat_506_07), median(lat_506_07), prctile(lat_506_07, 95), prctile(lat_506_07, 99), max(lat_506_07));
    fprintf('   -----------------------------------------------------------------------------------------------\n');
    fprintf('   合計 Certificate (A + B)       |  %7.4f  |   %7.4f   |  %7.4f |  %7.4f |   %7.4f\n', ...
        mean(lat_total), median(lat_total), prctile(lat_total, 95), prctile(lat_total, 99), max(lat_total));
    fprintf('  -------------------------------------------------------------------------------------------------\n');
    fprintf('  >>> ベンチマーク分析:\n');
    fprintf('      1,000回の統計評価において、11スパン一括 Certificate 評価の中央値は %7.4f ms、\n', median(lat_total));
    fprintf('      99パーセンタイルは %7.4f ms であった。なお、観測最大値は %7.4f ms であり、\n', prctile(lat_total, 99), max(lat_total));
    fprintf('      本評価は WCET を与えるものではない。\n\n');

    %% ====================================================================
    % [サマリー] 学術的総括
    % ====================================================================
    fprintf('=========================================================================================\n');
    fprintf(' [サマリー] Phase 5 学術的総括\n');
    fprintf('=========================================================================================\n');
    fprintf('  1. 支持超平面双対定理により、比張力下界 gamma_T(t) >= %7.4f m/s^2 > 0 が\n', min_global_M_LB);
    fprintf('     サンプリングを介さず連続時間で厳密に証明 (Certified) された。\n');
    fprintf('  2. 導関数 Bézier 凸包性により、ペイロード速度・加速度・Jerk・Snap に対する\n');
    fprintf('     連続時間保証上界が、Bézier 制御点から有限個の代数演算で評価可能な解析系として構成された。\n');
    fprintf('  3. これにより、索の弛緩排除・特異点回避に加え、ペイロード速度・加速度・Jerk・Snap\n');
    fprintf('     に対する連続時間保証上界が、サンプリングフリーな解析的評価系として統合された。\n');
    fprintf('  4. 比張力下界 M_LB と導関数上界の代数合成により、索角速度・角加速度および\n');
    fprintf('     UAV 加速度・比推力の連続時間外包上限がサンプリングフリーで確立された。\n');
    fprintf('=========================================================================================\n');
    fprintf('  Phase 5-01〜07 完了 (Feasibility-Certificate Architecture Established) [FROZEN]\n');
    fprintf('=========================================================================================\n\n');
end

%% ========================================================================
%  【Phase 5-05 カーネル】
% ========================================================================
function M_lb_vec = run_phase5_05_kernel(A_ctrl, cfg, cache)
    unique_knots = cache.unique_knots;
    n_spans = length(unique_knots) - 1;
    p_acc = cfg.p - 2;
    knots_acc = cache.knots_acc;
    M_lb_vec = zeros(n_spans, 1);

    for s = 1:n_spans
        u_s = unique_knots(s); u_e = unique_knots(s + 1);
        if u_e <= u_s + 1e-12, continue; end
        u_mid = 0.5 * (u_s + u_e);

        span_a = find_span_local(u_mid, knots_acc, p_acc, cfg.N_ctrl - 2);
        A_act = A_ctrl((span_a - p_acc):span_a, :);
        C_acc = cache.bezier_extract_acc{s};
        A_bez = C_acc * A_act;
        W_bez = A_bez + repmat((cfg.g_acc * cfg.e3)', size(A_bez, 1), 1);

        M_lb_vec(s) = compute_certified_tension_lower_bound(W_bez);
    end
end

%% ========================================================================
%  【Phase 5-06 & 5-07 カーネル】(本測定・ウォームアップ共通)
% ========================================================================
function run_phase5_06_07_kernel_with_Mlb(V_ctrl, A_ctrl_p, J_ctrl, S_ctrl, M_lb_vec, cfg, cache)
    unique_knots = cache.unique_knots;
    n_spans = length(unique_knots) - 1;

    for s = 1:n_spans
        u_s = unique_knots(s); u_e = unique_knots(s + 1);
        if u_e <= u_s + 1e-12, continue; end
        u_mid = 0.5 * (u_s + u_e);

        span_v = find_span_local(u_mid, cache.knots_v, 6, cfg.N_ctrl - 1);
        V_bez = cache.bezier_extract_v{s} * V_ctrl((span_v - 6):span_v, :);
        Dv_s = max(sqrt(sum(V_bez.^2, 2)));

        span_a = find_span_local(u_mid, cache.knots_acc, 5, cfg.N_ctrl - 2);
        A_bez = cache.bezier_extract_acc{s} * A_ctrl_p((span_a - 5):span_a, :);
        Da_s = max(sqrt(sum(A_bez.^2, 2)));

        span_j = find_span_local(u_mid, cache.knots_j, 4, cfg.N_ctrl - 3);
        J_bez = cache.bezier_extract_j{s} * J_ctrl((span_j - 4):span_j, :);
        Dj_s = max(sqrt(sum(J_bez.^2, 2)));

        span_s = find_span_local(u_mid, cache.knots_s, 3, cfg.N_ctrl - 4);
        S_bez = cache.bezier_extract_s{s} * S_ctrl((span_s - 3):span_s, :);
        Ds_s = max(sqrt(sum(S_bez.^2, 2)));

        M_lb_s = M_lb_vec(s);
        if M_lb_s <= 0.0
            error('Phase 5-07 cannot be evaluated because M_LB <= 0.');
        end

        % ★改修 1: 本体と完全に統一された厳密式
        omega_up = Dj_s / M_lb_s;
        alpha_up = (Ds_s / M_lb_s) + 2.0 * (Dj_s / M_lb_s) * omega_up + (omega_up^2);
        aQ_up = Da_s + cfg.sys.L_cable * alpha_up;
    end
end

%% ========================================================================
%  【支持超平面双対定理による M_LB 厳密導出】
% ========================================================================
function M_LB = compute_certified_tension_lower_bound(W)
    Q = W * W';
    n_vars = size(W, 1);
    lam = ones(n_vars, 1) / n_vars;
    y = lam;
    t_step = 1.0 / (max(eig(Q)) + 1e-6);

    for iter = 1:20
        lam_prev = lam;
        grad = Q * y;
        lam = project_to_simplex(y - t_step * grad);
        y = lam + ((iter - 1) / (iter + 2)) * (lam - lam_prev);
    end

    w_hat = W' * lam;
    M_hat = norm(w_hat);

    if M_hat > 0.0
        v = w_hat / M_hat;
        M_LB = max(0.0, min(W * v));
    else
        M_LB = 0.0;
    end
end

function x = project_to_simplex(v)
    n = length(v);
    u = sort(v, 'descend');
    cssv = cumsum(u);
    rho = find(u > (cssv - 1.0) ./ (1:n)', 1, 'last');
    theta = (cssv(rho) - 1.0) / rho;
    x = max(v - theta, 0.0);
end

%% ========================================================================
%  【索運動学 閉形式解析評価関数】
% ========================================================================
function [nT, dnT, d2nT, r] = eval_cable_kinematics(aL, jL, sL, g_acc, e3)
    W = aL + g_acc * e3;
    r = norm(W);
    nT = W / r;
    Porth = eye(3) - (nT * nT');

    dnT = (Porth * jL) / r;
    dr = dot(nT, jL);
    d2nT = (Porth * sL) / r - 2.0 * (dr / r) * dnT - nT * dot(dnT, dnT);
end

%% ========================================================================
%  Phase 2 凍結エンジンによる C6 軌道生成
% ========================================================================
function res = generate_phase2_c6_trajectory(T, D_start, D_end, active_obs, cfg, cache)
    scale_vec = (T .^ (0:cfg.p-1))';
    P_start = cache.B_start_inv * (D_start .* scale_vec);
    P_end   = cache.B_end_inv   * (D_end   .* scale_vec);

    P_fixed = zeros(18, 3);
    P_fixed(1:7, :)   = P_start;
    P_fixed(12:18, :) = P_end;
    P7  = P_start(7, :);
    P12 = P_end(1, :);
    P_fixed(8, :)  = 0.8 * P7 + 0.2 * P12;
    P_fixed(9, :)  = 0.6 * P7 + 0.4 * P12;
    P_fixed(10, :) = 0.4 * P7 + 0.6 * P12;
    P_fixed(11, :) = 0.2 * P7 + 0.8 * P12;

    P_nom = cache.N0 * P_fixed;
    c_act = active_obs.center';
    Sinv_act = active_obs.Sinv;

    min_q = inf; worst_k = 1;
    for k = 1:cfg.qp_n_samples
        dx = P_nom(k,1)-c_act(1); dy = P_nom(k,2)-c_act(2); dz = P_nom(k,3)-c_act(3);
        q = dx*(Sinv_act(1,1)*dx + Sinv_act(1,2)*dy + Sinv_act(1,3)*dz) + ...
            dy*(Sinv_act(2,1)*dx + Sinv_act(2,2)*dy + Sinv_act(2,3)*dz) + ...
            dz*(Sinv_act(3,1)*dx + Sinv_act(3,2)*dy + Sinv_act(3,3)*dz);
        if q < min_q, min_q = q; worst_k = k; end
    end

    dx_w = P_nom(worst_k,1)-c_act(1); dy_w = P_nom(worst_k,2)-c_act(2); dz_w = P_nom(worst_k,3)-c_act(3);
    gx = Sinv_act(1,1)*dx_w + Sinv_act(1,2)*dy_w + Sinv_act(1,3)*dz_w;
    gy = Sinv_act(2,1)*dx_w + Sinv_act(2,2)*dy_w + Sinv_act(2,3)*dz_w;
    gz = Sinv_act(3,1)*dx_w + Sinv_act(3,2)*dy_w + Sinv_act(3,3)*dz_w;
    norm_g = sqrt(gx^2 + gy^2 + gz^2);
    n_avoid = [gx; gy; gz] / norm_g;

    w_shape = [0.6; 1.0; 1.0; 0.6];
    disp_scalar = cache.B_free_0 * w_shape;
    d_req = 0.20 + 0.010;
    alpha_req = 0.0;

    for k = 1:cfg.qp_n_samples
        p0 = P_nom(k, :)';
        dp = p0 - active_obs.center;
        grad = active_obs.Sinv * dp;
        nk = grad / norm(grad);
        h_E = dot(nk, active_obs.center) + sqrt(max(0.0, nk' * active_obs.S * nk));
        ak = dot(nk, n_avoid) * disp_scalar(k);
        bk = h_E + d_req - dot(nk, p0);
        if ak > 1e-4
            alpha_req = max(alpha_req, bk / ak);
        end
    end

    G = [w_shape * n_avoid(1); w_shape * n_avoid(2); w_shape * n_avoid(3)];
    z_sol = G * alpha_req;

    P_new = P_fixed;
    P_new(8:11, 1) = P_new(8:11, 1) + z_sol(1:4);
    P_new(8:11, 2) = P_new(8:11, 2) + z_sol(5:8);
    P_new(8:11, 3) = P_new(8:11, 3) + z_sol(9:12);

    res.P_ctrl = P_new;
    res.alpha_star = alpha_req;
    res.n_avoid = n_avoid;
    model.P_fixed = P_fixed;
    model.T = T;
    model.knots = cache.knots;
    res.model = model;
end

%% ========================================================================
%  B-spline 導関数評価
% ========================================================================
function val = eval_bspline_deriv(model, u, P_ctrl, r)
    dN = eval_basis_derivative_raw(u, model.knots, 7, 18, r);
    val = (dN * P_ctrl)' * (1.0 / (model.T^r));
end

function dN = eval_basis_derivative_raw(u, knots, p, N_ctrl, r)
    if u <= 0.0, dN = zeros(1, N_ctrl); dN(1:p+1) = eval_basis_deriv_span(0.0, knots, p, p+1, r); return;
    elseif u >= 1.0, dN = zeros(1, N_ctrl); dN((N_ctrl-p):N_ctrl) = eval_basis_deriv_span(1.0, knots, p, N_ctrl, r); return;
    end
    span = find_span_local(u, knots, p, N_ctrl);
    local_vals = eval_basis_deriv_span(u, knots, p, span, r);
    dN = zeros(1, N_ctrl); dN((span-p):span) = local_vals;
end

function dN_local = eval_basis_deriv_span(u, knots, p, span, r)
    if r == 0, dN_local = eval_basis_raw_loc(u, knots, p, span); return; end
    if r > p, dN_local = zeros(1, p + 1); return; end
    M_loc = eye(p + 1); cur_p = p;
    for s = 1:r
        cur_len = p + 2 - s; D_step = zeros(cur_len - 1, cur_len);
        for i = 1:(cur_len - 1)
            idx = span - cur_p + i; denom = knots(idx + cur_p) - knots(idx);
            if denom > 1e-15, D_step(i, i) = -cur_p/denom; D_step(i, i+1) = cur_p/denom; end
        end
        M_loc = D_step * M_loc; cur_p = cur_p - 1;
    end
    dN_local = eval_basis_raw_loc(u, knots, cur_p, span) * M_loc;
end

function N_local = eval_basis_raw_loc(u, knots, p, span)
    N_local = zeros(1, p + 1); left = zeros(1, p + 1); right = zeros(1, p + 1); N_local(1) = 1.0;
    for j = 1:p
        left(j+1) = u - knots(span+1-j); right(j+1) = knots(span+j) - u; saved = 0.0;
        for r_idx = 0:(j-1)
            denom = right(r_idx+2) + left(j-r_idx+1);
            if denom > 1e-15, temp = N_local(r_idx+1)/denom; N_local(r_idx+1) = saved + right(r_idx+2)*temp; saved = left(j-r_idx+1)*temp;
            else, N_local(r_idx+1) = saved; saved = 0.0; end
        end
        N_local(j+1) = saved;
    end
end

function span = find_span_local(u, knots, p, N_ctrl)
    if u >= knots(N_ctrl+1), span = N_ctrl; return; end
    if u <= knots(p+1), span = p+1; return; end
    low = p+1; high = N_ctrl+1; mid = floor((low+high)/2);
    while (u < knots(mid) || u >= knots(mid+1))
        if u < knots(mid), high = mid; else, low = mid; end
        mid = floor((low+high)/2);
    end
    span = mid;
end

%% ========================================================================
%  キャッシュ初期化 & 各階導関数差分行列・Bézier Extraction 行列構築
% ========================================================================
function cache = init_phase5_static_cache(cfg)
    N_s = cfg.qp_n_samples;
    u_vec = linspace(0.0, 1.0, N_s)';
    p = cfg.p; N_ctrl = cfg.N_ctrl;
    m_internal = N_ctrl - p;
    int_k = linspace(0.0, 1.0, m_internal + 1);
    knots = [zeros(1, p + 1), int_k(2:end-1), ones(1, p + 1)];

    B_s = zeros(p, 7); B_e = zeros(p, 7);
    for r = 0:(p - 1)
        dN_s = eval_basis_derivative_raw(0.0, knots, p, N_ctrl, r);
        dN_e = eval_basis_derivative_raw(1.0, knots, p, N_ctrl, r);
        B_s(r + 1, :) = dN_s(1:7);
        B_e(r + 1, :) = dN_e(12:18);
    end
    cache.B_start_inv = inv(B_s);
    cache.B_end_inv   = inv(B_e);
    cache.knots       = knots;
    unique_knots      = unique(knots);
    cache.unique_knots = unique_knots;

    cache.N0 = zeros(N_s, N_ctrl);
    for k = 1:N_s
        cache.N0(k, :) = eval_basis_derivative_raw(u_vec(k), knots, p, N_ctrl, 0);
    end
    cache.B_free_0 = cache.N0(:, 8:11);

    % --- 各階導関数差分行列 (K_v, K_a, K_j, K_s) の構築 ---
    % 1階 (v): N_ctrl - 1 点, 次数 6, ノット U^(1) = knots(2:end-1)
    K1 = zeros(N_ctrl - 1, N_ctrl);
    for i = 1:(N_ctrl - 1)
        dt = knots(i + p + 1) - knots(i + 1);
        if dt > 1e-12, K1(i, i) = -p / dt; K1(i, i+1) = p / dt; end
    end
    cache.K_v = K1;
    cache.knots_v = knots(2:end-1);

    % 2階 (a): N_ctrl - 2 点, 次数 5, ノット U^(2) = knots(3:end-2)
    p2 = p - 1;
    K2_step = zeros(N_ctrl - 2, N_ctrl - 1);
    for i = 1:(N_ctrl - 2)
        dt = knots(i + p2 + 2) - knots(i + 2);
        if dt > 1e-12, K2_step(i, i) = -p2 / dt; K2_step(i, i+1) = p2 / dt; end
    end
    cache.K_a = K2_step * K1;
    cache.knots_acc = knots(3:end-2);

    % 3階 (j): N_ctrl - 3 点, 次数 4, ノット U^(3) = knots(4:end-3)
    p3 = p - 2;
    K3_step = zeros(N_ctrl - 3, N_ctrl - 2);
    for i = 1:(N_ctrl - 3)
        dt = knots(i + p3 + 3) - knots(i + 3);
        if dt > 1e-12, K3_step(i, i) = -p3 / dt; K3_step(i, i+1) = p3 / dt; end
    end
    cache.K_j = K3_step * cache.K_a;
    cache.knots_j = knots(4:end-3);

    % 4階 (s): N_ctrl - 4 点, 次数 3, ノット U^(4) = knots(5:end-4)
    p4 = p - 3;
    K4_step = zeros(N_ctrl - 4, N_ctrl - 3);
    for i = 1:(N_ctrl - 4)
        dt = knots(i + p4 + 4) - knots(i + 4);
        if dt > 1e-12, K4_step(i, i) = -p4 / dt; K4_step(i, i+1) = p4 / dt; end
    end
    cache.K_s = K4_step * cache.K_j;
    cache.knots_s = knots(5:end-4);

    % --- 各階導関数の Bézier Extraction 行列の事前構築 ---
    n_spans = length(unique_knots) - 1;
    cache.bezier_extract_v   = cell(n_spans, 1);
    cache.bezier_extract_acc = cell(n_spans, 1);
    cache.bezier_extract_j   = cell(n_spans, 1);
    cache.bezier_extract_s   = cell(n_spans, 1);

    for s = 1:n_spans
        u_s = unique_knots(s);
        u_e = unique_knots(s + 1);
        if u_e <= u_s + 1e-12, continue; end
        u_mid = 0.5 * (u_s + u_e);

        span_v = find_span_local(u_mid, cache.knots_v, 6, N_ctrl - 1);
        cache.bezier_extract_v{s} = compute_bezier_extraction_matrix(cache.knots_v, 6, span_v, u_s, u_e);

        span_a = find_span_local(u_mid, cache.knots_acc, 5, N_ctrl - 2);
        cache.bezier_extract_acc{s} = compute_bezier_extraction_matrix(cache.knots_acc, 5, span_a, u_s, u_e);

        span_j = find_span_local(u_mid, cache.knots_j, 4, N_ctrl - 3);
        cache.bezier_extract_j{s} = compute_bezier_extraction_matrix(cache.knots_j, 4, span_j, u_s, u_e);

        span_s = find_span_local(u_mid, cache.knots_s, 3, N_ctrl - 4);
        cache.bezier_extract_s{s} = compute_bezier_extraction_matrix(cache.knots_s, 3, span_s, u_s, u_e);
    end
end

function C = compute_bezier_extraction_matrix(knots, p, span, u_s, u_e)
    tau = cos(linspace(pi, 0, p + 1));
    u_pts = 0.5 * (u_s + u_e) + 0.5 * (u_e - u_s) * tau;
    xi_pts = (u_pts - u_s) / (u_e - u_s);

    B_bern = zeros(p + 1, p + 1);
    for ell = 0:p
        coeff = nchoosek(p, ell);
        B_bern(:, ell + 1) = coeff .* (xi_pts'.^ell) .* ((1 - xi_pts').^(p - ell));
    end

    N_bspline = zeros(p + 1, p + 1);
    for k = 1:(p + 1)
        N_bspline(k, :) = eval_basis_raw_loc(u_pts(k), knots, p, span);
    end

    C = B_bern \ N_bspline;
end

function R = eul2rotm_local(eul)
    y = eul(1); p = eul(2); r = eul(3);
    Rz = [cos(y) -sin(y) 0; sin(y) cos(y) 0; 0 0 1];
    Ry = [cos(p) 0 sin(p); 0 1 0; -sin(p) 0 cos(p)];
    Rx = [1 0 0; 0 cos(r) -sin(r); 0 sin(r) cos(r)];
    R = Rz * Ry * Rx;
end

function D = get_test_nominal_state(t)
    D = zeros(7, 3);
    D(1, :) = [0.3 + 0.1*t,  0.3,  15.0 + 0.5*t];
    D(2, :) = [0.1,          0.0,  0.5];
end

function s = pass_str(cond)
    if cond, s = '[PASS]'; else, s = '[FAIL]'; end
end

% ★修正 2: 客観的な UNCERTIFIED 表記
function s = cert_str(cond)
    if cond
        s = '[CERTIFIED (Continuous Feasible)]';
    else
        s = '[UNCERTIFIED]';
    end
end
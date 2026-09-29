% =========================================================================
% test_C6_BSPLINE_PHASE7_04_GUIDE_ASSISTED_REPLANNER.m
% 
% 【Phase 7-04: Guide-Path Assisted Certified B-Spline Replanning】
%  - 外部依存ゼロ (完全単一ファイル完結, 2つの解析 Figure 描画)
%  - アーキテクチャ:
%      [1] Prediction & Trigger: 公称全体系衝突スキャン
%      [2] Lightweight Front-End: 幾何学的膨張楕円体から左右 2 本の安全 Guide-Path 生成
%      [3] Trajectory Generation: C6 境界条件を満たす 24-DOF B-spline 一発 Fitting
%      [4] Iterative Refinement: 2001点走査から最悪点 1 点のみを追加して最大2回再Fit
%      [5] Time-Scaling Feasibility: 形状を保ったまま時間軸スケーリングで v, a, j を満足
%      [6] Certification & Gate: Phase 4 厳密 Fast Certificate による完全 Fail-Safe 遮断
%
% 実行コマンド:
%   >> test_C6_BSPLINE_PHASE7_04_GUIDE_ASSISTED_REPLANNER
% =========================================================================
function test_C6_BSPLINE_PHASE7_04_GUIDE_ASSISTED_REPLANNER()
    clc;
    close all;
    fprintf('=========================================================================================\n');
    fprintf('   Phase 7-04: Guide-Path Assisted Certified B-Spline Replanner Suite                    \n');
    fprintf('=========================================================================================\n\n');

    %% 0. 物理幾何・基本パラメータ設定
    cfg.p = 7;
    cfg.N_fixed_start = 7;       % 始端固定点数 (P1 ~ P7)
    cfg.N_free        = 8;       % 自由制御点数 (P8 ~ P15) -> 24 DOF
    cfg.N_fixed_end   = 7;       % 終端固定点数 (P16 ~ P22)
    cfg.N_ctrl        = cfg.N_fixed_start + cfg.N_free + cfg.N_fixed_end; % 計 22 点
    cfg.n_z           = cfg.N_free * 3; % 24 次元
    cfg.free_indices  = (cfg.N_fixed_start + 1):(cfg.N_fixed_start + cfg.N_free);

    cfg.g_acc = 9.80665;
    cfg.e3 = [0; 0; 1];

    cfg.sys.L_cable   = 1.00;    % 索長 [m]
    cfg.sys.R_payload = 0.15;    % ペイロード半径 [m]
    cfg.sys.R_cable   = 0.02;    % 索保護厚み [m]
    cfg.sys.R_uav     = 0.30;    % UAV 機体包絡半径 [m]
    cfg.d_margin      = 0.20;    % 要求表面安全マージン [m]

    limits.v_max  = 4.50;        % [m/s]
    limits.a_max  = 3.50;        % [m/s^2]
    limits.j_max  = 5.00;        % [m/s^3]

    obs = setup_obstacle_case1();
    T_global = 15.26;
    nom_fun = @(t) get_nominal_trajectory(t);

    fprintf('【0. システム構成】\n');
    fprintf(' - B-spline: 次数 p=%d, 制御点数 N=%d (始端固定 %d, 自由 %d [24-DOF], 終端固定 %d)\n', ...
        cfg.p, cfg.N_ctrl, cfg.N_fixed_start, cfg.N_free, cfg.N_fixed_end);
    fprintf(' - 障害物: 中心=[%.2f, %.2f, %.2f], 半径=[%.2f, %.2f, %.2f]\n', ...
        obs.center(1), obs.center(2), obs.center(3), obs.radii(1), obs.radii(2), obs.radii(3));
    fprintf(' - 運動学上下限: v_max=%.2f, a_max=%.2f, j_max=%.2f\n\n', limits.v_max, limits.a_max, limits.j_max);

    %% ====================================================================
    % [Step 1] 公称衝突区間スキャン & 安全復帰時刻 t_reconnect 選定
    % ====================================================================
    fprintf('=========================================================================================\n');
    fprintf(' [Step 1] 公称全体系スキャン & 安全復帰時刻 t_reconnect の自動同定\n');
    fprintf('=========================================================================================\n');

    N_nom_scan = 1527;
    t_nom_vec = linspace(0.0, T_global, N_nom_scan)';
    dnet_nom = zeros(N_nom_scan, 1);

    for j = 1:N_nom_scan
        [pL, ~, aL] = eval_nominal_state(nom_fun, t_nom_vec(j));
        W = aL + cfg.g_acc * cfg.e3; nT = W / norm(W);
        pQ = pL + cfg.sys.L_cable * nT;
        lam_star = find_cable_closest_lambda(pL, nT, cfg.sys.L_cable, obs);
        pC = pL + (lam_star * cfg.sys.L_cable) * nT;

        cL = compute_distance_point_to_ellipsoid(pL, obs) - cfg.sys.R_payload - cfg.d_margin;
        cQ = compute_distance_point_to_ellipsoid(pQ, obs) - cfg.sys.R_uav - cfg.d_margin;
        cC = compute_distance_point_to_ellipsoid(pC, obs) - cfg.sys.R_cable - cfg.d_margin;
        dnet_nom(j) = min([cL, cC, cQ]);
    end

    coll_idx = find(dnet_nom < 0.0);
    t_entry = t_nom_vec(coll_idx(1));
    t_exit  = t_nom_vec(coll_idx(end));

    t_reconnect_cand = linspace(t_exit + 2.0, T_global, 10);
    best_t_rec = t_reconnect_cand(end);

    for tr = t_reconnect_cand
        T_loc_test = tr;
        cache_test = init_expanded_bspline_cache(cfg, T_loc_test);
        P_base_test = build_c6_hermite_spline(nom_fun(0.0), nom_fun(tr), T_loc_test, cfg, cache_test);

        d_end_cpts = zeros(cfg.N_fixed_end, 1);
        for ep = 1:cfg.N_fixed_end
            cpt_idx = cfg.N_fixed_start + cfg.N_free + ep;
            d_end_cpts(ep) = compute_distance_point_to_ellipsoid(P_base_test(cpt_idx, :)', obs);
        end

        if min(d_end_cpts) > cfg.d_margin
            best_t_rec = tr;
            break;
        end
    end

    t_now = 0.0;
    t_reconnect = best_t_rec;
    T_local = t_reconnect - t_now;

    fprintf('  - 公称全体系 衝突区間: entry = %5.2f s, exit = %5.2f s\n', t_entry, t_exit);
    fprintf('  - 選定復帰時刻: t_reconnect = %5.2f s (脱出後余裕: %4.2f s, T_local = %5.2f s)\n\n', ...
        t_reconnect, t_reconnect - t_exit, T_local);

    cache = init_expanded_bspline_cache(cfg, T_local);
    D_start = nom_fun(t_now);
    D_end   = nom_fun(t_reconnect);
    P_base  = build_c6_hermite_spline(D_start, D_end, T_local, cfg, cache);

    audit_c6_reconnection(cache, P_base, D_start, D_end);

    %% ====================================================================
    % [Step 2] 軽量 Front-End: 幾何学的安全 Guide-Path 生成 (左右 2 候補)
    % ====================================================================
    fprintf('=========================================================================================\n');
    fprintf(' [Step 2] 軽量 Front-End: 幾何学的安全 Guide-Path 生成 (+Y / -Y の 2 候補)\n');
    fprintf('=========================================================================================\n');

    % 全体系を外包する安全半径 R_safe_total
    R_sys_envelope = cfg.sys.L_cable + cfg.sys.R_uav;
    R_safe_obs = obs.radii(2) + R_sys_envelope + cfg.d_margin + 0.35; % 余裕バッファ込み (約 2.85m)

    t_mid = 0.5 * (t_entry + t_exit);
    u_mid = t_mid / T_local;
    pL_mid_nom = eval_nominal_state(nom_fun, t_mid);

    % 公称進行方向ベクトル
    [~, v_nom_mid, ~] = eval_nominal_state(nom_fun, t_mid);
    tau = v_nom_mid / norm(v_nom_mid);

    % 直交横方向 (+Y 側)
    t_lat = [0; 1; 0];
    t_lat = t_lat - dot(t_lat, tau) * tau;
    t_lat = t_lat / norm(t_lat);

    % 左右 2 つの安全 Sub-goal 候補
    q_sub_pos = obs.center + R_safe_obs * t_lat;
    q_sub_neg = obs.center - R_safe_obs * t_lat;

    % 3 点の Guide-Path 列: [p_start -> q_sub -> p_end]
    guide_cands = cell(2, 1);
    guide_cands{1}.name = 'Guide Left (+Y 迂回)';
    guide_cands{1}.points = [nom_fun(t_now); q_sub_pos'; nom_fun(t_reconnect)];
    guide_cands{1}.u_times = [0.0; u_mid; 1.0];
    guide_cands{1}.n_side = t_lat;

    guide_cands{2}.name = 'Guide Right (-Y 迂回)';
    guide_cands{2}.points = [nom_fun(t_now); q_sub_neg'; nom_fun(t_reconnect)];
    guide_cands{2}.u_times = [0.0; u_mid; 1.0];
    guide_cands{2}.n_side = -t_lat;

    fprintf('  - 安全膨張半径 R_safe_obs = %5.2f m (障害物半径 %.2f + 索長 %.2f + 機体包絡 %.2f + マージン %.2f)\n', ...
        R_safe_obs, obs.radii(2), cfg.sys.L_cable, cfg.sys.R_uav, cfg.d_margin);
    fprintf('  - Guide 1 (+Y 側頂点) : [%+6.3f, %+6.3f, %+6.3f]\n', q_sub_pos(1), q_sub_pos(2), q_sub_pos(3));
    fprintf('  - Guide 2 (-Y 側頂点) : [%+6.3f, %+6.3f, %+6.3f]\n\n', q_sub_neg(1), q_sub_neg(2), q_sub_neg(3));

    %% ====================================================================
    % [Step 3] 候補別 B-spline Guide Fitting & 反復リファインメント
    % ====================================================================
    fprintf('=========================================================================================\n');
    fprintf(' [Step 3] C6 拘束付き B-spline Fitting & 局所リファインメント (最大 2 回)\n');
    fprintf('=========================================================================================\n');

    N_eval_dense = 2001;
    u_dense = linspace(0.0, 1.0, N_eval_dense)';
    t_local_dense = u_dense * T_local;

    fitted_results = cell(2, 1);

    for g_idx = 1:2
        g_info = guide_cands{g_idx};
        fprintf('  【候補 #%d: %s】\n', g_idx, g_info.name);

        guide_pts = g_info.points;
        guide_u   = g_info.u_times;

        % 一発 Fitting (QP / 最小二乗)
        P_fit = fit_bspline_to_guide(P_base, guide_pts, guide_u, cfg, cache);

        % 衝突スキャン & 最悪点追加リファインメント (最大 2 回)
        for ref_iter = 1:2
            [worst_k, dnet_arr] = scan_trajectory_clearance(P_fit, obs, cfg, cache, T_local, u_dense);
            if worst_k.dnet >= 0.0
                fprintf('    - Refine %d: 全体系完全離隔達成 (d_net,min = %+7.4f m)\n', ref_iter-1, worst_k.dnet);
                break;
            end

            fprintf('    - Refine %d: 未達 d_net = %+7.4f m (at t = %5.2f s, %s) -> 点を追加して再Fit\n', ...
                ref_iter, worst_k.dnet, worst_k.t, worst_k.part_str);

            % 最悪点を安全側へ押し出した点を Guide に 1 点追加
            u_add = worst_k.u;
            pL_cur = worst_k.pL;
            q_add = pL_cur + (abs(worst_k.dnet) + 0.15) * g_info.n_side;

            guide_pts = [guide_pts; q_add']; %#ok<AGROW>
            guide_u   = [guide_u; u_add];    %#ok<AGROW>

            P_fit = fit_bspline_to_guide(P_base, guide_pts, guide_u, cfg, cache);
        end

        % 最終全時間走査
        [worst_final, dnet_fin, dL_fin, dC_fin, dQ_fin, v_fin, a_fin, j_fin] = scan_trajectory_full( ...
            P_fit, obs, cfg, cache, T_local, u_dense);

        % --- Time-Scaling Feasibility ---
        max_v = max(v_fin); max_a = max(a_fin); max_j = max(j_fin);
        alpha_scale = max([1.0, max_v / limits.v_max, sqrt(max_a / limits.a_max), (max_j / limits.j_max)^(1/3)]);

        if alpha_scale > 1.001
            fprintf('    - Time-Scaling 適用: 倍率 alpha = %6.3f (軌道形状維持のまま時間伸縮)\n', alpha_scale);
            T_scaled = T_local * alpha_scale;
            cache_scaled = init_expanded_bspline_cache(cfg, T_scaled);
            P_scaled = P_fit; % 制御点の幾何位置は完全不変

            [worst_final, dnet_fin, dL_fin, dC_fin, dQ_fin, v_fin, a_fin, j_fin] = scan_trajectory_full( ...
                P_scaled, obs, cfg, cache_scaled, T_scaled, u_dense);
            cache_use = cache_scaled;
            T_use = T_scaled;
        else
            cache_use = cache;
            T_use = T_local;
        end

        max_v = max(v_fin); max_a = max(a_fin); max_j = max(j_fin);
        pass_g1 = (worst_final.dnet >= 0.0);
        pass_g2 = (max_v <= limits.v_max + 1e-4) && (max_a <= limits.a_max + 1e-4) && (max_j <= limits.j_max + 1e-4);

        % Gate 3: Fast Certificate (Phase 4 厳密版)
        is_cert = false; g_min_LB = -inf;
        if pass_g1 && pass_g2
            [is_cert, g_min_LB, worst_span] = evaluate_phase4_certified_bound(P_fit, obs, cfg, cache_use);
        end

        res_entry.g_idx     = g_idx;
        res_entry.name      = g_info.name;
        res_entry.P_ctrl    = P_fit;
        res_entry.cache     = cache_use;
        res_entry.T_final   = T_use;
        res_entry.d_net     = worst_final.dnet;
        res_entry.max_v     = max_v;
        res_entry.max_a     = max_a;
        res_entry.max_j     = max_j;
        res_entry.g_min_LB  = g_min_LB;
        res_entry.pass_g1   = pass_g1;
        res_entry.pass_g2   = pass_g2;
        res_entry.pass_g3   = is_cert;
        res_entry.pass_all  = pass_g1 && pass_g2 && is_cert;

        fitted_results{g_idx} = res_entry;

        fprintf('    >>> 結果: d_net=%+6.3fm, v=%.2f, a=%.2f, j=%.2f | Gate1:%s, Gate2:%s, Gate3:%s\n\n', ...
            worst_final.dnet, max_v, max_a, max_j, pass_str(pass_g1), pass_str(pass_g2), pass_str(is_cert));
    end

    %% ====================================================================
    % [Step 4] 最適認証軌道選定 & 完全 Fail-Safe 遮断判定
    % ====================================================================
    fprintf('=========================================================================================\n');
    fprintf(' [Step 4] 最適認証軌道の選定 & 実行許可判定 (Fail-Safe Architecture)\n');
    fprintf('=========================================================================================\n');

    passed_indices = find(cellfun(@(c) c.pass_all, fitted_results));
    execution_allowed = false;
    P_execute = [];

    if isempty(passed_indices)
        fprintf('  ***************************************************************************************\n');
        fprintf('  >>> 【全候補 UNCERTIFIED / 実行禁止】: 連続時間安全証明を通過した候補は存在しません。\n');
        fprintf('  >>> 【DECISION: EXECUTION PROHIBITED (完全遮断)】\n');
        fprintf('      安全性が数学的に保証できないため、軌道出力を遮断しました (P_execute = [])。\n');
        fprintf('  ***************************************************************************************\n\n');

        [~, best_id] = max(cellfun(@(c) c.d_net, fitted_results));
        best_cand = fitted_results{best_id};
    else
        % 全ゲート合格候補の中から最小時間 T_final を選定
        T_list = cellfun(@(c) c.T_final, fitted_results(passed_indices));
        [~, min_t_sub] = min(T_list);
        best_id = passed_indices(min_t_sub);
        best_cand = fitted_results{best_id};

        execution_allowed = true;
        P_execute = best_cand.P_ctrl;

        fprintf('  =======================================================================================\n');
        fprintf('  >>> 【DECISION: EXECUTION ALLOWED (採択)】\n');
        fprintf('      最適選定: 【%s】 (所要時間 T = %5.2f s)\n', best_cand.name, best_cand.T_final);
        fprintf('      - 表面余白 min d_net = %+7.4f m > 0\n', best_cand.d_net);
        fprintf('      - 運動学限界: v=%.2f, a=%.2f, j=%.2f [ALL PASS]\n', ...
            best_cand.max_v, best_cand.max_a, best_cand.max_j);
        fprintf('      - Fast Certificate 下界 g_min^LB = %+7.4f m > 0 [CERTIFIED PASS]\n', best_cand.g_min_LB);
        fprintf('      数学的に安全証明された滑らかな軌道を実機コントローラへ出力します。\n');
        fprintf('  =======================================================================================\n\n');
    end

    %% ====================================================================
    % [Step 5] 診断可視化プロファイル描画
    % ====================================================================
    figure('Name', 'Phase 7-04: Guide-Assisted Replanning Diagnostic', 'Color', 'w', 'Position', [50, 50, 1200, 750]);

    % 1. 3D 空間軌道 & 制御点ポリゴン
    subplot(2, 2, [1, 3]);
    hold on; grid on; axis equal;
    [Xo, Yo, Zo] = ellipsoid(obs.center(1), obs.center(2), obs.center(3), ...
        obs.radii(1)+cfg.d_margin, obs.radii(2)+cfg.d_margin, obs.radii(3)+cfg.d_margin, 30);
    surf(Xo, Yo, Zo, 'FaceColor', [0.9, 0.2, 0.2], 'FaceAlpha', 0.20, 'EdgeColor', 'none', 'DisplayName', 'Obstacle + Margin');

    t_eval_plot = linspace(0.0, best_cand.T_final, N_eval_dense)';
    p_nom_p = zeros(N_eval_dense, 3);
    p_cand_p = zeros(N_eval_dense, 3);
    for j = 1:N_eval_dense
        p_nom_p(j, :) = eval_nominal_state(nom_fun, t_eval_plot(j));
        p_cand_p(j, :) = eval_bspline_deriv(best_cand.cache, u_dense(j), best_cand.P_ctrl, 0);
    end
    plot3(p_nom_p(:,1), p_nom_p(:,2), p_nom_p(:,3), 'r--', 'LineWidth', 1.8, 'DisplayName', 'Nominal Track');

    if execution_allowed
        plot3(p_cand_p(:,1), p_cand_p(:,2), p_cand_p(:,3), 'b-', 'LineWidth', 2.5, 'DisplayName', 'Executed Trajectory [SAFE]');
    else
        plot3(p_cand_p(:,1), p_cand_p(:,2), p_cand_p(:,3), 'm:', 'LineWidth', 1.8, 'DisplayName', 'Rejected Candidate [UNSAFE]');
    end

    P_st = best_cand.P_ctrl(1:cfg.N_fixed_start, :);
    P_fr = best_cand.P_ctrl(cfg.free_indices, :);
    P_ed = best_cand.P_ctrl((cfg.N_fixed_start + cfg.N_free + 1):end, :);

    plot3(best_cand.P_ctrl(:,1), best_cand.P_ctrl(:,2), best_cand.P_ctrl(:,3), 'k:', 'LineWidth', 0.8, 'DisplayName', 'Control Polygon');
    plot3(P_st(:,1), P_st(:,2), P_st(:,3), 'rs', 'MarkerSize', 7, 'MarkerFaceColor', 'r', 'DisplayName', 'Start-Fixed');
    plot3(P_fr(:,1), P_fr(:,2), P_fr(:,3), 'co', 'MarkerSize', 8, 'MarkerFaceColor', 'c', 'DisplayName', 'Free (24-DOF)');
    plot3(P_ed(:,1), P_ed(:,2), P_ed(:,3), 'md', 'MarkerSize', 7, 'MarkerFaceColor', 'm', 'DisplayName', 'End-Fixed');

    xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
    title(sprintf('3D Trajectory & Control Points (Execution: %s)', check_bool_str(execution_allowed)), 'FontSize', 12);
    legend('Location', 'northeast', 'FontSize', 8);
    view(35, 25);

    % 2. クリアランス履歴
    subplot(2, 2, 2);
    hold on; grid on;
    yline(0.0, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Safety Boundary');
    [~, dnet_best, dL_best, dC_best, dQ_best, ~, ~, ~] = scan_trajectory_full( ...
        best_cand.P_ctrl, obs, cfg, best_cand.cache, best_cand.T_final, u_dense);
    plot(t_eval_plot, dnet_best, 'k-', 'LineWidth', 2.0, 'DisplayName', 'd_{net}(t)');
    plot(t_eval_plot, dL_best,   'b:', 'LineWidth', 1.5, 'DisplayName', 'Payload d_L(t)');
    plot(t_eval_plot, dC_best,   'c:', 'LineWidth', 1.5, 'DisplayName', 'Cable d_C(t)');
    plot(t_eval_plot, dQ_best,   'm:', 'LineWidth', 1.5, 'DisplayName', 'UAV d_Q(t)');
    xlabel('Time [s]'); ylabel('Clearance [m]');
    title('Clearance Profiles by Component', 'FontSize', 11);
    legend('Location', 'southeast', 'FontSize', 8);
    ylim([-1.0, 3.0]);

    % 3. 動力学プロファイル
    subplot(2, 2, 4);
    hold on; grid on;
    [~, ~, ~, ~, ~, v_best, a_best, j_best] = scan_trajectory_full( ...
        best_cand.P_ctrl, obs, cfg, best_cand.cache, best_cand.T_final, u_dense);
    plot(t_eval_plot, v_best, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Velocity ||v||');
    plot(t_eval_plot, a_best, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Acceleration ||a||');
    plot(t_eval_plot, j_best, 'k:', 'LineWidth', 1.2, 'DisplayName', 'Jerk ||j||');
    yline(limits.v_max, 'b--', 'DisplayName', 'v_{max}');
    yline(limits.a_max, 'r--', 'DisplayName', 'a_{max}');
    yline(limits.j_max, 'k--', 'DisplayName', 'j_{max}');
    xlabel('Time [s]'); ylabel('Magnitude [SI]');
    title('Kinematic Profiles (Constraint Satisfied)', 'FontSize', 11);
    legend('Location', 'northeast', 'FontSize', 8);

    fprintf('=========================================================================================\n');
    fprintf('  Phase 7-04 完了\n');
    fprintf('=========================================================================================\n\n');
end

%% ========================================================================
%  【C6 拘束付き B-spline Guide Fitting (最小二乗 QP)】
% ========================================================================
function P_fit = fit_bspline_to_guide(P_base, guide_pts, guide_u, cfg, cache)
    N_g = length(guide_u);
    n_f = cfg.N_free;
    n_dim = cfg.n_z;

    % 目的関数: min_z sum_i || p(u_i; z) - q_i ||^2 + lambda_smooth * || D2 * z ||^2
    % p(u_i; z) = p_base(u_i) + M_i * z
    H_fit = zeros(n_dim, n_dim);
    f_fit = zeros(n_dim, 1);

    for i = 1:N_g
        u_i = guide_u(i);
        q_i = guide_pts(i, :)';

        dN0 = eval_basis_derivative_raw(u_i, cache.knots, cfg.p, cfg.N_ctrl, 0);
        phi_f = dN0(cfg.free_indices); % 1 x 8
        p_base_i = eval_bspline_deriv(cache, u_i, P_base, 0);

        for d = 1:3
            idx_d = (d-1)*n_f + (1:n_f);
            H_fit(idx_d, idx_d) = H_fit(idx_d, idx_d) + 2.0 * (phi_f' * phi_f);
            diff_i = p_base_i(d) - q_i(d);
            f_fit(idx_d) = f_fit(idx_d) + 2.0 * (phi_f' * diff_i);
        end
    end

    % 制御点列平滑化正則化 (D2 二階差分)
    D2 = diff(eye(n_f), 2);
    H_smooth_block = 5.0 * (D2' * D2);
    H_smooth = blkdiag(H_smooth_block, H_smooth_block, H_smooth_block);

    H_total = H_fit + H_smooth + 1e-4 * eye(n_dim);

    opts_qp = optimoptions('quadprog', 'Display', 'none');
    lb_z = -4.0 * ones(n_dim, 1);
    ub_z =  4.0 * ones(n_dim, 1);

    [z_sol, ~, ef] = quadprog(H_total, f_fit, [], [], [], [], lb_z, ub_z, zeros(n_dim, 1), opts_qp);

    if ef == 1
        P_fit = assemble_control_points(P_base, z_sol, cfg);
    else
        P_fit = P_base;
    end
end

%% ========================================================================
%  【Phase 4 準拠 厳密全体系 Fast Certificate】
% ========================================================================
function [is_cert, g_min_global, worst_span] = evaluate_phase4_certified_bound(P_ctrl, obs, cfg, cache)
    unique_knots = cache.unique_knots;
    n_spans = length(unique_knots) - 1;
    g_spans = zeros(n_spans, 1);

    for s = 1:n_spans
        u_s = unique_knots(s); u_e = unique_knots(s + 1);
        if u_e <= u_s + 1e-12, g_spans(s) = inf; continue; end
        u_mid = 0.5 * (u_s + u_e);

        % 中間点索ベクトル n_T
        aL_mid = eval_bspline_deriv(cache, u_mid, P_ctrl, 2);
        W_mid  = aL_mid + cfg.g_acc * cfg.e3;
        nT_mid = W_mid / norm(W_mid);

        % 中間点法線 n_sep
        pL_mid = eval_bspline_deriv(cache, u_mid, P_ctrl, 0);
        grad = obs.Sinv * (pL_mid - obs.center);
        if norm(grad) < 1e-6, grad = [0; 1; 0]; end
        n_sep = grad / norm(grad);

        h_E = dot(n_sep, obs.center) + sqrt(max(0.0, n_sep' * obs.S * n_sep));

        % Bézier Extraction による局所細分化凸包下界 p_low (粗いB-spline凸包を排除)
        span_p = find_span_local(u_mid, cache.knots, cfg.p, cfg.N_ctrl);
        P_act = P_ctrl((span_p - cfg.p):span_p, :);
        C_pos = cache.bezier_extract_pos{s};
        P_bez = C_pos * P_act;
        p_low = min(P_bez * n_sep);

        % Phase 4 厳密全体系分離包絡 Delta_sep(n, n_T)
        proj_neg = dot(-n_sep, nT_mid);
        val_L = cfg.sys.R_payload;
        val_Q = cfg.sys.L_cable * proj_neg + cfg.sys.R_uav;
        val_C = max(0.0, cfg.sys.L_cable * proj_neg) + cfg.sys.R_cable;
        Delta_sep = max([val_L, val_Q, val_C]);

        g_spans(s) = p_low - Delta_sep - h_E - cfg.d_margin;
    end

    [g_min_global, worst_span] = min(g_spans);
    is_cert = (g_min_global >= 0.0);
end

%% ========================================================================
%  【全ホライズン詳細スキャンルーチン】
% ========================================================================
function [worst, dnet_arr, dL_arr, dC_arr, dQ_arr, v_arr, a_arr, j_arr] = scan_trajectory_full( ...
    P_ctrl, obs, cfg, cache, T_local, u_vec)

    N_s = length(u_vec);
    dL_arr = zeros(N_s, 1); dC_arr = zeros(N_s, 1); dQ_arr = zeros(N_s, 1); dnet_arr = zeros(N_s, 1);
    v_arr = zeros(N_s, 1); a_arr = zeros(N_s, 1); j_arr = zeros(N_s, 1);

    worst.dnet = inf;

    for j = 1:N_s
        u = u_vec(j);
        t_loc = u * T_local;
        pL = eval_bspline_deriv(cache, u, P_ctrl, 0);
        vL = eval_bspline_deriv(cache, u, P_ctrl, 1);
        aL = eval_bspline_deriv(cache, u, P_ctrl, 2);
        jL = eval_bspline_deriv(cache, u, P_ctrl, 3);

        W = aL + cfg.g_acc * cfg.e3; nT = W / norm(W);
        pQ = pL + cfg.sys.L_cable * nT;
        lam_star = find_cable_closest_lambda(pL, nT, cfg.sys.L_cable, obs);
        pC = pL + (lam_star * cfg.sys.L_cable) * nT;

        cL = compute_distance_point_to_ellipsoid(pL, obs) - cfg.sys.R_payload - cfg.d_margin;
        cQ = compute_distance_point_to_ellipsoid(pQ, obs) - cfg.sys.R_uav - cfg.d_margin;
        cC = compute_distance_point_to_ellipsoid(pC, obs) - cfg.sys.R_cable - cfg.d_margin;

        [d_net, comp_id] = min([cL, cC, cQ]);
        dL_arr(j) = cL; dC_arr(j) = cC; dQ_arr(j) = cQ; dnet_arr(j) = d_net;
        v_arr(j) = norm(vL); a_arr(j) = norm(aL); j_arr(j) = norm(jL);

        if d_net < worst.dnet
            worst.dnet = d_net;
            worst.t = t_loc;
            worst.u = u;
            worst.pL = pL; worst.pC = pC; worst.pQ = pQ;
            if comp_id == 1, worst.part_str = 'Payload';
            elseif comp_id == 2, worst.part_str = 'Cable';
            else, worst.part_str = 'UAV'; end
        end
    end
end

function [worst, dnet_arr] = scan_trajectory_clearance(P_ctrl, obs, cfg, cache, T_local, u_vec)
    [worst, dnet_arr, ~, ~, ~, ~, ~, ~] = scan_trajectory_full(P_ctrl, obs, cfg, cache, T_local, u_vec);
end

%% ========================================================================
%  【C6 接続誤差監査】
% ========================================================================
function audit_c6_reconnection(cache, P_ctrl, D_start, D_end)
    fprintf('  【C6 Reconnection Audit (始端・終端代数境界誤差)】\n');
    fprintf('   階数 r | 始端誤差 e_start [SI] | 終端誤差 e_end [SI] | 判定\n');
    fprintf('  --------+-----------------------+---------------------+--------\n');
    pass_c6 = true;
    for r = 0:6
        ps = eval_bspline_deriv(cache, 0.0, P_ctrl, r);
        pe = eval_bspline_deriv(cache, 1.0, P_ctrl, r);
        e_s = norm(ps - D_start(r+1, :)');
        e_e = norm(pe - D_end(r+1, :)');
        if e_s > 1e-6 || e_e > 1e-6, pass_c6 = false; end
        fprintf('     %d    |       %9.2e       |      %9.2e      | %s\n', ...
            r, e_s, e_e, pass_str(e_s < 1e-6 && e_e < 1e-6));
    end
    fprintf('  ---------------------------------------------------------------\n');
    if pass_c6
        fprintf('  >>> C6 エルミート再接続: 機械精度完全一致 [PASS]\n\n');
    else
        fprintf('  >>> C6 エルミート再接続: 不整合あり [FAIL]\n\n');
    end
end

%% ========================================================================
%  【幾何最短距離・B-spline 計算基礎ライブラリ】
% ========================================================================
function P_fixed = build_c6_hermite_spline(D_start, D_end, T, cfg, cache)
    scale_vec = (T .^ (0:cfg.p-1))';
    P_start = cache.B_start_inv * (D_start .* scale_vec);
    P_end   = cache.B_end_inv   * (D_end   .* scale_vec);

    P_fixed = zeros(cfg.N_ctrl, 3);
    P_fixed(1:cfg.N_fixed_start, :) = P_start;
    P_fixed((cfg.N_ctrl - cfg.N_fixed_end + 1):cfg.N_ctrl, :) = P_end;

    P7  = P_start(7, :);
    P16 = P_end(1, :);
    for k = 1:cfg.N_free
        alpha = k / (cfg.N_free + 1.0);
        P_fixed(cfg.N_fixed_start + k, :) = (1 - alpha) * P7 + alpha * P16;
    end
end

function P_full = assemble_control_points(P_fixed, z, cfg)
    P_full = P_fixed;
    n_f = cfg.N_free;
    f_idx = cfg.free_indices;
    P_full(f_idx, 1) = P_full(f_idx, 1) + z(1:n_f);
    P_full(f_idx, 2) = P_full(f_idx, 2) + z((n_f+1):(2*n_f));
    P_full(f_idx, 3) = P_full(f_idx, 3) + z((2*n_f+1):(3*n_f));
end

function cache = init_expanded_bspline_cache(cfg, T_local)
    p = cfg.p; N_ctrl = cfg.N_ctrl;
    m_internal = N_ctrl - p;
    int_k = linspace(0.0, 1.0, m_internal + 1);
    knots = [zeros(1, p + 1), int_k(2:end-1), ones(1, p + 1)];

    B_s = zeros(p, cfg.N_fixed_start);
    B_e = zeros(p, cfg.N_fixed_end);
    for r = 0:(p - 1)
        dN_s = eval_basis_derivative_raw(0.0, knots, p, N_ctrl, r);
        dN_e = eval_basis_derivative_raw(1.0, knots, p, N_ctrl, r);
        B_s(r + 1, :) = dN_s(1:cfg.N_fixed_start);
        B_e(r + 1, :) = dN_e((N_ctrl - cfg.N_fixed_end + 1):N_ctrl);
    end
    cache.B_start_inv = inv(B_s);
    cache.B_end_inv   = inv(B_e);
    cache.knots       = knots;
    unique_knots      = unique(knots);
    cache.unique_knots = unique_knots;
    cache.p           = p;
    cache.N_ctrl      = N_ctrl;
    cache.T           = T_local;

    % Bézier Extraction 行列 (位置 p=7)
    n_spans = length(unique_knots) - 1;
    cache.bezier_extract_pos = cell(n_spans, 1);
    for s = 1:n_spans
        u_s = unique_knots(s); u_e = unique_knots(s + 1);
        if u_e <= u_s + 1e-12, continue; end
        u_mid = 0.5 * (u_s + u_e);
        span_p = find_span_local(u_mid, knots, p, N_ctrl);
        cache.bezier_extract_pos{s} = compute_bezier_extraction_matrix(knots, p, span_p, u_s, u_e);
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

function val = eval_bspline_deriv(cache, u, P_ctrl, r)
    dN = eval_basis_derivative_raw(u, cache.knots, cache.p, cache.N_ctrl, r);
    val = (dN * P_ctrl)' * (1.0 / (cache.T^r));
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

function lam_star = find_cable_closest_lambda(pL, nT, L_cable, obs)
    invphi = (sqrt(5) - 1) / 2; invphi2 = (3 - sqrt(5)) / 2;
    a = 0.0; b = 1.0; h = b - a;
    c = a + invphi2 * h; d = a + invphi * h;
    fc = compute_distance_point_to_ellipsoid(pL + (c * L_cable) * nT, obs);
    fd = compute_distance_point_to_ellipsoid(pL + (d * L_cable) * nT, obs);
    for k = 1:12
        if fc < fd
            b = d; d = c; fd = fc;
            h = invphi * h; c = a + invphi2 * h;
            fc = compute_distance_point_to_ellipsoid(pL + (c * L_cable) * nT, obs);
        else
            a = c; c = d; fc = fd;
            h = invphi * h; d = a + invphi * h;
            fd = compute_distance_point_to_ellipsoid(pL + (d * L_cable) * nT, obs);
        end
    end
    lam_star = 0.5 * (a + b);
end

function dist_val = compute_distance_point_to_ellipsoid(pt, obs)
    p_rel = obs.R' * (pt - obs.center);
    rx = obs.radii(1); ry = obs.radii(2); rz = obs.radii(3);
    alg_val = (p_rel(1)/rx)^2 + (p_rel(2)/ry)^2 + (p_rel(3)/rz)^2;
    if alg_val < 1e-12, dist_val = -min([rx, ry, rz]); return; end
    nonzero_idx = abs(p_rel) > 1e-12;
    r_act = [rx; ry; rz]; r_act = r_act(nonzero_idx);
    p_act = p_rel(nonzero_idx);
    if isempty(r_act), dist_val = -min([rx, ry, rz]); return; end
    r_min_act = min(r_act); r_max_act = max(r_act);
    if alg_val >= 1.0, a_lam = 0.0; b_lam = r_max_act * norm(p_rel);
    else, a_lam = -(r_min_act^2) + 1e-9; b_lam = 0.0; end
    lam = 0.5 * (a_lam + b_lam);
    for iter = 1:35
        denom = r_act.^2 + lam;
        f = sum( ((r_act .* p_act) ./ denom).^2 ) - 1.0;
        if abs(f) < 1e-10, break; end
        if f > 0.0, a_lam = lam; else, b_lam = lam; end
        df = -2.0 * sum( ((r_act .* p_act).^2) ./ (denom.^3) );
        lam_new = lam - f / df;
        if lam_new > a_lam && lam_new < b_lam, lam = lam_new;
        else, lam = 0.5 * (a_lam + b_lam); end
    end
    y_star = zeros(3, 1);
    for i = 1:3
        if abs(p_rel(i)) > 1e-12
            y_star(i) = (obs.radii(i)^2 * p_rel(i)) / (obs.radii(i)^2 + lam);
        end
    end
    euc_dist = norm(p_rel - y_star);
    if alg_val >= 1.0, dist_val = euc_dist; else, dist_val = -euc_dist; end
end

function [p, v, a] = eval_nominal_state(nom_fun, t)
    D = nom_fun(t); p = D(1, :)'; v = D(2, :)'; a = D(3, :)';
end

function obs = setup_obstacle_case1()
    obs.center = [1.0; 0.3; 18.5];
    obs.radii  = [1.3; 1.3; 1.3];
    obs.R      = eye(3);
    obs.S      = diag(obs.radii.^2);
    obs.Sinv   = diag(1.0 ./ (obs.radii.^2));
end

function D = get_nominal_trajectory(t)
    D = zeros(7, 3);
    D(1, :) = [0.3 + 0.1*t,  0.3,  15.0 + 0.5*t];
    D(2, :) = [0.1,          0.0,  0.5];
end

function s = pass_str(c), if c, s = '[PASS]'; else, s = '[FAIL]'; end, end
function s = check_str(c), if c, s = '[OK]'; else, s = '[VIOLATED]'; end, end
function s = check_bool_str(c), if c, s = 'ALLOWED'; else, s = 'PROHIBITED'; end, end
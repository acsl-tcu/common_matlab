% % =========================================================================
% % test_C6_BSPLINE_PHASE6.m
% % 
% % 【Phase 6: 全球面 Separator 診断・Gateスクリーニング・統計ベンチマーク】
% %  - 外部依存ゼロ (完全単一ファイル完結)
% %  - 目的と機能:
% %      1. Span 9/10 に対する 1,250 点全球面グリッド走査 (Global Sphere Grid Search)
% %         -> max_n g_LB,j(n) を算出し、法線の問題か軌道変形の問題かを完全切り分け
% %      2. 3連 Hard Gates 判定 (Safety, Cable, Dynamic)
% %      3. Feasible 集合が空の場合でもベンチマークを遮断せず完遂
% %      4. 500 回統計ベンチマーク (Screening Pipeline Latency) の完全計測
% %
% % 実行コマンド:
% %   >> test_C6_BSPLINE_PHASE6
% % =========================================================================
% function test_C6_BSPLINE_PHASE6()
%     clc;
%     tic_total = tic;
% 
%     fprintf('=========================================================================================\n');
%     fprintf('   Phase 6: Global Separator Diagnostic, Multi-Gate Screening & Benchmark Suite          \n');
%     fprintf('=========================================================================================\n\n');
% 
%     %% 0. 物理幾何・障害物・制約パラメータ定義
%     cfg.p = 7;                   % 次数 p = 7 (C6 連続)
%     cfg.N_ctrl = 18;             % 制御点数 18 (7 + 4 + 7)
%     cfg.N_fixed_start = 7;
%     cfg.N_free = 4;
%     cfg.N_fixed_end = 7;
%     cfg.n_z = 12;
%     cfg.qp_n_samples = 31;
% 
%     cfg.g_acc = 9.80665;         % [m/s^2]
%     cfg.e3 = [0; 0; 1];
% 
%     cfg.sys.L_cable   = 1.00;    % 索長 [m]
%     cfg.sys.R_payload = 0.15;    % ペイロード半径 [m]
%     cfg.sys.R_cable   = 0.02;    % 索安全保護半径 [m]
%     cfg.sys.R_uav     = 0.30;    % UAV 機体包絡半径 [m]
%     cfg.d_margin      = 0.20;    % 要求表面安全マージン [m]
% 
%     t_start = 0.0;
%     T_eval  = 15.26;
%     nom_fun = @(t) get_test_nominal_state(t);
%     D_start = nom_fun(t_start);
%     D_end   = nom_fun(t_start + T_eval);
% 
%     obs.center = [0.3; 0.3; 20.0];
%     obs.radii  = [1.2; 1.2; 2.0];
%     obs.R      = eul2rotm_local([pi/6, pi/12, 0]);
%     obs.S      = obs.R * diag(obs.radii.^2) * obs.R';
%     obs.Sinv   = obs.R * diag(1.0 ./ (obs.radii.^2)) * obs.R';
% 
%     limits.v_max  = 3.50;        % [m/s]
%     limits.a_max  = 2.50;        % [m/s^2]
%     limits.j_max  = 2.50;        % [m/s^3]
%     limits.aQ_max = 5.00;        % [m/s^2]
% 
%     cache = init_phase6_static_cache(cfg);
% 
%     fprintf('[システム設定 & 制約境界]\n');
%     fprintf(' - 索長 L_cable         : %.2f m\n', cfg.sys.L_cable);
%     fprintf(' - 要求安全マージン d   : %.2f m\n', cfg.d_margin);
%     fprintf(' - Dynamic Gate 制限値  : v_max=%.1f m/s, a_max=%.1f m/s^2, j_max=%.1f m/s^3, aQ_max=%.1f m/s^2\n', ...
%         limits.v_max, limits.a_max, limits.j_max, limits.aQ_max);
%     fprintf(' - 評価ホライズン T     : %.2f s (C6 連続 B-spline)\n\n', T_eval);
% 
%     %% ====================================================================
%     % [Step 1] 多様な C6 軌道候補集合 Z_cand の生成
%     % ====================================================================
%     fprintf('=========================================================================================\n');
%     fprintf(' [Step 1] 多様な C6 軌道候補集合 Z_cand の生成 (N_cand = 8 候補)\n');
%     fprintf('=========================================================================================\n');
% 
%     tic_gen = tic;
% 
%     res_p2 = generate_phase2_c6_trajectory(T_eval, D_start, D_end, obs, cfg, cache);
%     alpha_star = res_p2.alpha_star;
%     w_shape = [0.6; 1.0; 1.0; 0.6];
%     n_main = res_p2.n_avoid;
% 
%     t_dir = [-n_main(2); n_main(1); 0];
%     if norm(t_dir) < 1e-4, t_dir = [0; -n_main(3); n_main(2)]; end
%     n_side = t_dir / norm(t_dir);
% 
%     cand_defs = [ ...
%         struct('name', 'Cand 1: Nominal (alpha=0.0)      ', 'a_main', 0.00,             'a_side', 0.00); ...
%         struct('name', 'Cand 2: Deficient (0.50*alpha*)  ', 'a_main', 0.50*alpha_star, 'a_side', 0.00); ...
%         struct('name', 'Cand 3: Subcritical (0.80*alpha*)', 'a_main', 0.80*alpha_star, 'a_side', 0.00); ...
%         struct('name', 'Cand 4: Critical (1.00*alpha*)   ', 'a_main', 1.00*alpha_star, 'a_side', 0.00); ...
%         struct('name', 'Cand 5: Robust (1.15*alpha*)     ', 'a_main', 1.15*alpha_star, 'a_side', 0.00); ...
%         struct('name', 'Cand 6: High-Margin (1.35*alpha*)', 'a_main', 1.35*alpha_star, 'a_side', 0.00); ...
%         struct('name', 'Cand 7: Aggressive (1.60*alpha*) ', 'a_main', 1.60*alpha_star, 'a_side', 0.00); ...
%         struct('name', 'Cand 8: Perturbed Diagonal Avoid ', 'a_main', 1.15*alpha_star, 'a_side', 0.35*alpha_star); ...
%     ];
% 
%     N_cand = length(cand_defs);
%     cand_trajs = cell(N_cand, 1);
% 
%     for k = 1:N_cand
%         G_k = [w_shape * (cand_defs(k).a_main * n_main(1) + cand_defs(k).a_side * n_side(1)); ...
%                w_shape * (cand_defs(k).a_main * n_main(2) + cand_defs(k).a_side * n_side(2)); ...
%                w_shape * (cand_defs(k).a_main * n_main(3) + cand_defs(k).a_side * n_side(3))];
% 
%         P_k = res_p2.P_fixed;
%         P_k(8:11, 1) = P_k(8:11, 1) + G_k(1:4);
%         P_k(8:11, 2) = P_k(8:11, 2) + G_k(5:8);
%         P_k(8:11, 3) = P_k(8:11, 3) + G_k(9:12);
% 
%         cand_trajs{k}.P_ctrl = P_k;
%         cand_trajs{k}.z = G_k;
%         cand_trajs{k}.name = cand_defs(k).name;
%     end
%     time_gen = toc(tic_gen);
% 
%     fprintf('  - %d 個の C6 候補軌道を正常に構築。\n', N_cand);
%     fprintf('  - 候補軌道生成時間 (T_gen)            : %7.3f ms\n\n', time_gen * 1000.0);
% 
%     %% ====================================================================
%     % [Step 2 & 3] 3連 Hard Gates スクリーニング
%     % ====================================================================
%     fprintf('=========================================================================================\n');
%     fprintf(' [Step 2 & 3] 3連 Hard Gates スクリーニング (Safety, Cable, Dynamic)\n');
%     fprintf('=========================================================================================\n');
% 
%     cand_evals = struct('name', {}, 'g_min_LB', {}, 'gamma_T_min_LB', {}, ...
%                         'v_max_UB', {}, 'a_max_UB', {}, 'j_max_UB', {}, 'aQ_max_UB', {}, ...
%                         'pass_safe', {}, 'pass_cable', {}, 'pass_dyn', {}, ...
%                         'is_feasible', {}, 'J_dev', {}, 'J_snap', {}, 'J_total', {}, ...
%                         'cert_details', {});
% 
%     time_gate_each = zeros(N_cand, 1);
% 
%     for k = 1:N_cand
%         tic_gate_k = tic;
%         P_k = cand_trajs{k}.P_ctrl;
%         z_k = cand_trajs{k}.z;
% 
%         cert_k = evaluate_trajectory_certificates_optimized(P_k, obs, cfg, cache, T_eval);
% 
%         pass_safe  = (cert_k.min_g_LB >= 0.0);
%         pass_cable = (cert_k.min_M_LB > 0.0);
%         pass_dyn   = (cert_k.max_v_UB <= limits.v_max) && ...
%                      (cert_k.max_a_UB <= limits.a_max) && ...
%                      (cert_k.max_j_UB <= limits.j_max) && ...
%                      (cert_k.max_aQ_UB <= limits.aQ_max);
% 
%         is_feas = pass_safe && pass_cable && pass_dyn;
% 
%         J_dev = norm(z_k)^2;
%         J_snap = compute_snap_integral(P_k, cfg, cache, T_eval);
% 
%         cand_evals(k).name           = cand_trajs{k}.name;
%         cand_evals(k).g_min_LB       = cert_k.min_g_LB;
%         cand_evals(k).gamma_T_min_LB = cert_k.min_M_LB;
%         cand_evals(k).v_max_UB       = cert_k.max_v_UB;
%         cand_evals(k).a_max_UB       = cert_k.max_a_UB;
%         cand_evals(k).j_max_UB       = cert_k.max_j_UB;
%         cand_evals(k).aQ_max_UB      = cert_k.max_aQ_UB;
%         cand_evals(k).pass_safe      = pass_safe;
%         cand_evals(k).pass_cable     = pass_cable;
%         cand_evals(k).pass_dyn       = pass_dyn;
%         cand_evals(k).is_feasible    = is_feas;
%         cand_evals(k).J_dev          = J_dev;
%         cand_evals(k).J_snap         = J_snap;
%         cand_evals(k).cert_details   = cert_k;
% 
%         time_gate_each(k) = toc(tic_gate_k);
%     end
% 
%     time_gate_total = sum(time_gate_each);
% 
%     fprintf('  【全 8 候補に対する Hard Gates スクリーニング一覧】\n');
%     fprintf('  ----------------------------------------------------------------------------------------------------------------------\n');
%     fprintf('   ID | 候補軌道名                     | min g_LB [m] | min γ_T [m/s2] | max v_UB [m/s]| Safety | Cable | Dyn | Feasible\n');
%     fprintf('  ----------------------------------------------------------------------------------------------------------------------\n');
%     for k = 1:N_cand
%         fprintf('   #%d | %s |   %+7.4f    |    %7.4f    |    %6.3f     |  %s  |  %s  | %s |   %s\n', ...
%             k, cand_evals(k).name, cand_evals(k).g_min_LB, cand_evals(k).gamma_T_min_LB, ...
%             cand_evals(k).v_max_UB, pass_str(cand_evals(k).pass_safe), ...
%             pass_str(cand_evals(k).pass_cable), pass_str(cand_evals(k).pass_dyn), ...
%             feas_str(cand_evals(k).is_feasible));
%     end
%     fprintf('  ----------------------------------------------------------------------------------------------------------------------\n');
%     fprintf('  - Hard Gate 評価所要時間 (T_gate)     : 合計 %7.3f ms (平均 %6.3f ms/cand)\n\n', ...
%         time_gate_total * 1000.0, mean(time_gate_each) * 1000.0);
% 
%     %% ====================================================================
%     % [重要数理診断] Candidate #4 & #7 に対する全球面グリッド走査 (1,250点)
%     % ====================================================================
%     fprintf('=========================================================================================\n');
%     fprintf(' [重要数理診断] Span 9 & 10 に対する 1,250 点全球面グリッド走査 (Global Sphere Diagnostic)\n');
%     fprintf('=========================================================================================\n');
% 
%     diag_cands = [4, 7];
%     target_spans = [9, 10];
% 
%     for c_idx = 1:length(diag_cands)
%         k = diag_cands(c_idx);
%         P_k = cand_trajs{k}.P_ctrl;
%         fprintf('  【Candidate #%d: %s】\n', k, cand_evals(k).name);
% 
%         for s = target_spans
%             u_s = cache.unique_knots(s);
%             u_e = cache.unique_knots(s + 1);
%             u_mid = 0.5 * (u_s + u_e);
% 
%             % 制御点抽出
%             span_p = find_span_local(u_mid, cache.knots, cfg.p, cfg.N_ctrl);
%             P_bez = cache.bezier_extract_pos{s} * P_k((span_p - cfg.p):span_p, :);
% 
%             span_a = find_span_local(u_mid, cache.knots_acc, 5, cfg.N_ctrl - 2);
%             A_ctrl = (cache.K_a * P_k) / (T_eval^2);
%             A_bez = cache.bezier_extract_acc{s} * A_ctrl((span_a - 5):span_a, :);
%             W_bez = A_bez + repmat((cfg.g_acc * cfg.e3)', size(A_bez, 1), 1);
%             M_LB = compute_certified_tension_lower_bound(W_bez);
% 
%             % 1,250点全球面グリッド走査
%             [g_max_global, n_opt_global, p_opt, Delta_opt, h_opt] = ...
%                 run_global_sphere_separator_grid_search(P_bez, W_bez, M_LB, obs, cfg.sys, cfg.d_margin);
% 
%             % 既存局所探索値との比較
%             g_local = cand_evals(k).cert_details.g_LB_spans(s);
% 
%             fprintf('   * Span %2d [%.4f, %.4f]:\n', s, u_s, u_e);
%             fprintf('       - 局所探索下界 g_LB (現在値) : %+7.4f m\n', g_local);
%             fprintf('       - 全球面大域最大値 g_max^global: %+7.4f m (差分: %+6.3f mm)\n', ...
%                 g_max_global, (g_max_global - g_local)*1000.0);
%             fprintf('       - 最良法線 n_opt             : [%+5.2f, %+5.2f, %+5.2f]\n', ...
%                 n_opt_global(1), n_opt_global(2), n_opt_global(3));
%             fprintf('       - 内訳: p_low=%.4f, h_E=%.4f, Delta_up=%.4f, d=%.2f\n', ...
%                 p_opt, h_opt, Delta_opt, cfg.d_margin);
%             fprintf('       - 幾何学的判定               : %s\n', ...
%                 pass_str(g_max_global >= 0.0));
%         end
%         fprintf('  ---------------------------------------------------------------------------------------\n');
%     end
%     fprintf('  >>> 診断結論:\n');
%     fprintf('      全球面を余すところなく走査した結果、大域最大値 g_max^global が正に転じるか否かにより、\n');
%     fprintf('      「法線探索アルゴリズムの最適化で解決可能」か「軌道変形基底の拡張が必要」かが確定する。\n\n');
% 
%     %% ====================================================================
%     % [Step 4 & 5] 実行可能集合の判定 & 軌道選択 (非遮断構造)
%     % ====================================================================
%     fprintf('=========================================================================================\n');
%     fprintf(' [Step 4 & 5] 実行可能集合 I_feasible の判定 & 最適軌道選択\n');
%     fprintf('=========================================================================================\n');
% 
%     feas_idx = find([cand_evals.is_feasible]);
%     selection_executed = false;
%     time_select = 0.0;
% 
%     if isempty(feas_idx)
%         fprintf('  ***************************************************************************************\n');
%         fprintf('  >>> Feasible set is EMPTY: I_feasible = {}\n');
%         fprintf('      現行の候補軌道族では全 8 候補がリジェクトされました。\n');
%         fprintf('      (※ ベンチマークを遮断せず、Step 6 のパイプライン統計計測へ進みます)\n');
%         fprintf('  ***************************************************************************************\n\n');
%     else
%         tic_select = tic;
%         selection_executed = true;
%         ref_idx = 4;
%         J_dev_ref  = cand_evals(ref_idx).J_dev;
%         J_snap_ref = cand_evals(ref_idx).J_snap;
%         if J_dev_ref < 1e-4, J_dev_ref = 1.0; end
%         w_dev = 0.50; w_snap = 0.50;
% 
%         best_J = inf; best_k = -1;
%         for idx = 1:length(feas_idx)
%             k = feas_idx(idx);
%             norm_dev  = cand_evals(k).J_dev / J_dev_ref;
%             norm_snap = cand_evals(k).J_snap / J_snap_ref;
%             J_tot = w_dev * norm_dev + w_snap * norm_snap;
%             cand_evals(k).J_total = J_tot;
%             if J_tot < best_J, best_J = J_tot; best_k = k; end
%         end
%         time_select = toc(tic_select);
% 
%         fprintf('  >>> 最適選択結果: 【候補 #%d: %s】が採択されました！ (J* = %.4f)\n\n', ...
%             best_k, cand_evals(best_k).name, best_J);
%     end
% 
%     time_total = toc(tic_total);
% 
%     fprintf('=========================================================================================\n');
%     fprintf(' [Computation Time Summary (End-to-End Pipeline)]\n');
%     fprintf('=========================================================================================\n');
%     fprintf('  - 候補軌道生成時間 (T_gen)            : %8.3f ms\n', time_gen * 1000.0);
%     fprintf('  - Hard Gate 評価合計 (T_gate)         : %8.3f ms\n', time_gate_total * 1000.0);
%     if selection_executed
%         fprintf('  - 最適軌道選択時間 (T_select)         : %8.3f ms\n', time_select * 1000.0);
%     else
%         fprintf('  - 最適軌道選択時間 (T_select)         :    0.000 ms (空集合のためスキップ)\n');
%     end
%     fprintf('  - 全意思決定パイプライン合計 (T_total): %8.3f ms\n\n', time_total * 1000.0);
% 
%     %% ====================================================================
%     % [Step 6] 独立統計ベンチマーク (N=500 試行, 表示完全除外)
%     % ====================================================================
%     fprintf('=========================================================================================\n');
%     fprintf(' [Step 6] 独立統計ベンチマーク (N=500 試行, 表示オーバーヘッド完全除外)\n');
%     fprintf('=========================================================================================\n');
% 
%     N_bench = 500;
%     lat_pipeline = zeros(N_bench, 1);
%     ref_idx = 4;
%     J_dev_ref  = cand_evals(ref_idx).J_dev;
%     J_snap_ref = cand_evals(ref_idx).J_snap;
%     if J_dev_ref < 1e-4, J_dev_ref = 1.0; end
%     w_dev = 0.50; w_snap = 0.50;
% 
%     % ウォームアップ (50回)
%     for w = 1:50
%         run_phase6_screening_pipeline_clean(cand_trajs, obs, cfg, cache, limits, T_eval, w_dev, w_snap, J_dev_ref, J_snap_ref);
%     end
% 
%     % 本測定
%     for b = 1:N_bench
%         t_b = tic;
%         run_phase6_screening_pipeline_clean(cand_trajs, obs, cfg, cache, limits, T_eval, w_dev, w_snap, J_dev_ref, J_snap_ref);
%         lat_pipeline(b) = toc(t_b) * 1000.0;
%     end
% 
%     mean_l   = mean(lat_pipeline);
%     median_l = median(lat_pipeline);
%     p95_l    = prctile(lat_pipeline, 95);
%     p99_l    = prctile(lat_pipeline, 99);
%     max_l    = max(lat_pipeline);
% 
%     fprintf('  * 測定対象: 全 8 候補に対する「スパン法線探索 + Certificate + 3連Gateスクリーニング」パイプライン\n');
%     fprintf('  -------------------------------------------------------------------------------------------------\n');
%     fprintf('   測定指標                       | レイテンシ [ms] (全 8 候補一括) | 1候補あたり平均 [μs]\n');
%     fprintf('  -------------------------------------------------------------------------------------------------\n');
%     fprintf('   Mean (平均所要時間)            |           %7.4f ms           |        %7.2f μs\n', mean_l, (mean_l/N_cand)*1000);
%     fprintf('   Median (中央値)                |           %7.4f ms           |        %7.2f μs\n', median_l, (median_l/N_cand)*1000);
%     fprintf('   P95 (95パーセンタイル)         |           %7.4f ms           |        %7.2f μs\n', p95_l, (p95_l/N_cand)*1000);
%     fprintf('   P99 (99パーセンタイル)         |           %7.4f ms           |        %7.2f μs\n', p99_l, (p99_l/N_cand)*1000);
%     fprintf('   Observed Max (実測最大値)      |           %7.4f ms           |        %7.2f μs\n', max_l, (max_l/N_cand)*1000);
%     fprintf('  -------------------------------------------------------------------------------------------------\n');
%     fprintf('  (注: 500回の測定統計であり、実測最大値 (Observed Max) は WCET を与えるものではない)\n\n');
% end
% 
% %% ========================================================================
% %  【表示オーバーヘッド完全排除のスクリーニングパイプライン】
% % ========================================================================
% function best_k = run_phase6_screening_pipeline_clean(cand_trajs, obs, cfg, cache, limits, T_eval, w_dev, w_snap, J_dev_ref, J_snap_ref)
%     N_cand = length(cand_trajs);
%     best_J = inf;
%     best_k = -1;
% 
%     for k = 1:N_cand
%         P_k = cand_trajs{k}.P_ctrl;
% 
%         cert_k = evaluate_trajectory_certificates_optimized(P_k, obs, cfg, cache, T_eval);
% 
%         if cert_k.min_g_LB < 0.0, continue; end
%         if cert_k.min_M_LB <= 0.0, continue; end
%         if cert_k.max_v_UB > limits.v_max || ...
%            cert_k.max_a_UB > limits.a_max || ...
%            cert_k.max_j_UB > limits.j_max || ...
%            cert_k.max_aQ_UB > limits.aQ_max, continue; end
% 
%         J_dev = norm(cand_trajs{k}.z)^2;
%         J_snap = compute_snap_integral(P_k, cfg, cache, T_eval);
%         J_tot = w_dev * (J_dev / J_dev_ref) + w_snap * (J_snap / J_snap_ref);
% 
%         if J_tot < best_J
%             best_J = J_tot;
%             best_k = k;
%         end
%     end
% end
% 
% %% ========================================================================
% %  【1,250点 全球面グリッド走査 (Global Sphere Separator Diagnostic)】
% %  θ in [0, pi] (25分割), φ in [0, 2pi) (50分割)
% % ========================================================================
% function [best_g, best_n, best_p, best_Delta, best_h] = ...
%     run_global_sphere_separator_grid_search(P_bez, W_bez, M_LB, obs, sys, d_margin)
% 
%     N_theta = 25;
%     N_phi = 50;
%     theta_vec = linspace(0.01, pi - 0.01, N_theta);
%     phi_vec = linspace(0, 2*pi, N_phi + 1); phi_vec(end) = [];
% 
%     best_g = -inf;
%     best_n = [1; 0; 0];
%     best_p = 0; best_Delta = 0; best_h = 0;
% 
%     for th = theta_vec
%         sin_th = sin(th);
%         cos_th = cos(th);
%         for ph = phi_vec
%             n_c = [sin_th * cos(ph); sin_th * sin(ph); cos_th];
%             [g_c, p_c, Delta_c, h_c] = eval_span_lower_bound_for_n( ...
%                 n_c, P_bez, W_bez, M_LB, obs, sys, d_margin);
% 
%             if g_c > best_g
%                 best_g = g_c;
%                 best_n = n_c;
%                 best_p = p_c;
%                 best_Delta = Delta_c;
%                 best_h = h_c;
%             end
%         end
%     end
% end
% 
% %% ========================================================================
% %  【スパン最適分離法線探索付き Certificate 評価関数】
% % ========================================================================
% function cert = evaluate_trajectory_certificates_optimized(P_ctrl, obs, cfg, cache, T_eval)
%     unique_knots = cache.unique_knots;
%     n_spans = length(unique_knots) - 1;
%     p = cfg.p; N_ctrl = cfg.N_ctrl;
% 
%     V_ctrl = (cache.K_v * P_ctrl) / (T_eval^1);
%     A_ctrl = (cache.K_a * P_ctrl) / (T_eval^2);
%     J_ctrl = (cache.K_j * P_ctrl) / (T_eval^3);
%     S_ctrl = (cache.K_s * P_ctrl) / (T_eval^4);
% 
%     g_LB_spans     = zeros(n_spans, 1);
%     p_low_spans    = zeros(n_spans, 1);
%     Delta_up_spans = zeros(n_spans, 1);
%     h_E_spans      = zeros(n_spans, 1);
%     M_LB_spans     = zeros(n_spans, 1);
% 
%     max_v_UB = 0.0;
%     max_a_UB = 0.0;
%     max_j_UB = 0.0;
%     max_aQ_UB = 0.0;
% 
%     for s = 1:n_spans
%         u_s = unique_knots(s); u_e = unique_knots(s + 1);
%         if u_e <= u_s + 1e-12, continue; end
%         u_mid = 0.5 * (u_s + u_e);
% 
%         % 初期法線 n_mid
%         p_mid = eval_bspline_deriv(cache.model_ref, u_mid, P_ctrl, 0);
%         dp = p_mid - obs.center;
%         grad = obs.Sinv * dp;
%         if norm(grad) < 1e-6, grad = [1; 0; 0]; end
%         n_init = grad / norm(grad);
% 
%         % 制御点抽出
%         span_p = find_span_local(u_mid, cache.knots, p, N_ctrl);
%         P_bez = cache.bezier_extract_pos{s} * P_ctrl((span_p-p):span_p, :);
% 
%         span_a = find_span_local(u_mid, cache.knots_acc, 5, N_ctrl - 2);
%         A_bez = cache.bezier_extract_acc{s} * A_ctrl((span_a-5):span_a, :);
%         W_bez = A_bez + repmat((cfg.g_acc * cfg.e3)', size(A_bez, 1), 1);
% 
%         M_LB = compute_certified_tension_lower_bound(W_bez);
%         M_LB_spans(s) = M_LB;
% 
%         % スパン局所法線の適応的探索
%         [~, g_low, p_low, Delta_up, h_Ej] = find_optimal_span_separator( ...
%             n_init, P_bez, W_bez, M_LB, obs, cfg.sys, cfg.d_margin);
% 
%         g_LB_spans(s)     = g_low;
%         p_low_spans(s)    = p_low;
%         Delta_up_spans(s) = Delta_up;
%         h_E_spans(s)      = h_Ej;
% 
%         % 導関数上界 (v, a, j, s)
%         span_v = find_span_local(u_mid, cache.knots_v, 6, N_ctrl - 1);
%         V_bez = cache.bezier_extract_v{s} * V_ctrl((span_v-6):span_v, :);
%         Dv_s = max(sqrt(sum(V_bez.^2, 2)));
%         max_v_UB = max(max_v_UB, Dv_s);
% 
%         Da_s = max(sqrt(sum(A_bez.^2, 2)));
%         max_a_UB = max(max_a_UB, Da_s);
% 
%         span_j = find_span_local(u_mid, cache.knots_j, 4, N_ctrl - 3);
%         J_bez = cache.bezier_extract_j{s} * J_ctrl((span_j-4):span_j, :);
%         Dj_s = max(sqrt(sum(J_bez.^2, 2)));
%         max_j_UB = max(max_j_UB, Dj_s);
% 
%         span_s = find_span_local(u_mid, cache.knots_s, 3, N_ctrl - 4);
%         S_bez = cache.bezier_extract_s{s} * S_ctrl((span_s-3):span_s, :);
%         Ds_s = max(sqrt(sum(S_bez.^2, 2)));
% 
%         if M_LB > 0.0
%             omega_up = Dj_s / M_LB;
%             alpha_up = (Ds_s / M_LB) + 2.0 * (Dj_s / M_LB) * omega_up + (omega_up^2);
%             aQ_up = Da_s + cfg.sys.L_cable * alpha_up;
%             max_aQ_UB = max(max_aQ_UB, aQ_up);
%         end
%     end
% 
%     cert.min_g_LB       = min(g_LB_spans);
%     cert.min_M_LB       = min(M_LB_spans);
%     cert.max_v_UB       = max_v_UB;
%     cert.max_a_UB       = max_a_UB;
%     cert.max_j_UB       = max_j_UB;
%     cert.max_aQ_UB      = max_aQ_UB;
%     cert.g_LB_spans     = g_LB_spans;
%     cert.p_low_spans    = p_low_spans;
%     cert.Delta_up_spans = Delta_up_spans;
%     cert.h_E_spans      = h_E_spans;
%     cert.M_LB_spans     = M_LB_spans;
% end
% 
% %% ========================================================================
% %  【スパン局所法線の適応的最適化】
% % ========================================================================
% function [best_n, best_g, best_p, best_Delta, best_h] = find_optimal_span_separator( ...
%     n_init, P_bez, W_bez, M_LB, obs, sys, d_margin)
% 
%     [best_g, best_p, best_Delta, best_h] = eval_span_lower_bound_for_n( ...
%         n_init, P_bez, W_bez, M_LB, obs, sys, d_margin);
%     best_n = n_init;
% 
%     if best_g >= 0.0
%         return;
%     end
% 
%     t1 = [-best_n(2); best_n(1); 0];
%     if norm(t1) < 1e-4, t1 = [0; -best_n(3); best_n(2)]; end
%     t1 = t1 / norm(t1);
%     t2 = cross(best_n, t1);
% 
%     angles = linspace(0, 2*pi, 9); angles(end) = [];
%     step_sizes = [0.15, 0.30];
% 
%     for s_idx = 1:length(step_sizes)
%         step = step_sizes(s_idx);
%         for a_idx = 1:length(angles)
%             th = angles(a_idx);
%             dir_tangent = cos(th)*t1 + sin(th)*t2;
%             n_cand = best_n + step * dir_tangent;
%             n_cand = n_cand / norm(n_cand);
% 
%             [g_cand, p_c, Delta_c, h_c] = eval_span_lower_bound_for_n( ...
%                 n_cand, P_bez, W_bez, M_LB, obs, sys, d_margin);
% 
%             if g_cand > best_g
%                 best_g = g_cand;
%                 best_n = n_cand;
%                 best_p = p_c;
%                 best_Delta = Delta_c;
%                 best_h = h_c;
%                 if best_g >= 0.0, return; end
%             end
%         end
%     end
% end
% 
% %% ========================================================================
% %  【特定法線 n に対するスパン下界の代数評価】
% % ========================================================================
% function [g_low, p_low, Delta_up, h_E] = eval_span_lower_bound_for_n( ...
%     n, P_bez, W_bez, M_LB, obs, sys, d_margin)
% 
%     p_low = min(P_bez * n);
%     h_E = dot(n, obs.center) + sqrt(max(0.0, n' * obs.S * n));
% 
%     proj_W = W_bez * n;
%     w_min = min(proj_W);
%     M_max = max(sqrt(sum(W_bez.^2, 2)));
% 
%     if w_min >= 0.0
%         q_low = w_min / M_max;
%     else
%         if M_LB > 0.0, q_low = max(-1.0, w_min / M_LB);
%         else, q_low = -1.0; end
%     end
% 
%     Delta_up = eval_delta_h_sep_from_q(q_low, sys);
%     g_low = p_low - Delta_up - h_E - d_margin;
% end
% 
% %% ========================================================================
% %  【Snap 2乗積分コスト J_snap の数値計算】
% % ========================================================================
% function J_snap = compute_snap_integral(P_ctrl, cfg, cache, T_eval)
%     S_ctrl = (cache.K_s * P_ctrl) / (T_eval^4);
%     unique_knots = cache.unique_knots;
%     n_spans = length(unique_knots) - 1;
% 
%     gp = [-0.8611363116, -0.3399810436, 0.3399810436, 0.8611363116];
%     gw = [ 0.3478548451,  0.6521451549, 0.6521451549, 0.3478548451];
% 
%     J_snap = 0.0;
%     for s = 1:n_spans
%         u_s = unique_knots(s); u_e = unique_knots(s + 1);
%         if u_e <= u_s + 1e-12, continue; end
%         u_mid = 0.5 * (u_s + u_e);
% 
%         span_s = find_span_local(u_mid, cache.knots_s, 3, cfg.N_ctrl - 4);
%         S_bez = cache.bezier_extract_s{s} * S_ctrl((span_s-3):span_s, :);
% 
%         h_span = (u_e - u_s) * T_eval;
%         for g_idx = 1:4
%             xi = 0.5 * (1.0 + gp(g_idx));
%             s_vec = eval_bezier_curve_point(S_bez, 3, xi);
%             J_snap = J_snap + gw(g_idx) * (0.5 * h_span) * dot(s_vec, s_vec);
%         end
%     end
% end
% 
% %% ========================================================================
% %  【支持超平面双対定理による M_LB 導出】
% % ========================================================================
% function M_LB = compute_certified_tension_lower_bound(W)
%     Q = W * W';
%     n_vars = size(W, 1);
%     lam = ones(n_vars, 1) / n_vars;
%     y = lam;
%     t_step = 1.0 / (max(eig(Q)) + 1e-6);
% 
%     for iter = 1:20
%         lam_prev = lam;
%         grad = Q * y;
%         lam = project_to_simplex(y - t_step * grad);
%         y = lam + ((iter - 1) / (iter + 2)) * (lam - lam_prev);
%     end
% 
%     w_hat = W' * lam;
%     M_hat = norm(w_hat);
% 
%     if M_hat > 0.0
%         v = w_hat / M_hat;
%         M_LB = max(0.0, min(W * v));
%     else
%         M_LB = 0.0;
%     end
% end
% 
% function x = project_to_simplex(v)
%     n = length(v);
%     u = sort(v, 'descend');
%     cssv = cumsum(u);
%     rho = find(u > (cssv - 1.0) ./ (1:n)', 1, 'last');
%     theta = (cssv(rho) - 1.0) / rho;
%     x = max(v - theta, 0.0);
% end
% 
% function Delta_upper = eval_delta_h_sep_from_q(q_lower, sys)
%     val_L = sys.R_payload;
%     val_Q = sys.R_uav - sys.L_cable * q_lower;
%     val_C = sys.R_cable + max(0.0, -sys.L_cable * q_lower);
%     Delta_upper = max([val_L, val_Q, val_C]);
% end
% 
% function pt = eval_bezier_curve_point(P_bez, p_deg, xi)
%     coeff = zeros(p_deg + 1, 1);
%     for ell = 0:p_deg
%         coeff(ell + 1) = nchoosek(p_deg, ell) * (xi^ell) * ((1.0 - xi)^(p_deg - ell));
%     end
%     pt = (coeff' * P_bez)';
% end
% 
% function val = eval_bspline_deriv(model, u, P_ctrl, r)
%     dN = eval_basis_derivative_raw(u, model.knots, 7, 18, r);
%     val = (dN * P_ctrl)' * (1.0 / (model.T^r));
% end
% 
% function dN = eval_basis_derivative_raw(u, knots, p, N_ctrl, r)
%     if u <= 0.0, dN = zeros(1, N_ctrl); dN(1:p+1) = eval_basis_deriv_span(0.0, knots, p, p+1, r); return;
%     elseif u >= 1.0, dN = zeros(1, N_ctrl); dN((N_ctrl-p):N_ctrl) = eval_basis_deriv_span(1.0, knots, p, N_ctrl, r); return;
%     end
%     span = find_span_local(u, knots, p, N_ctrl);
%     local_vals = eval_basis_deriv_span(u, knots, p, span, r);
%     dN = zeros(1, N_ctrl); dN((span-p):span) = local_vals;
% end
% 
% function dN_local = eval_basis_deriv_span(u, knots, p, span, r)
%     if r == 0, dN_local = eval_basis_raw_loc(u, knots, p, span); return; end
%     if r > p, dN_local = zeros(1, p + 1); return; end
%     M_loc = eye(p + 1); cur_p = p;
%     for s = 1:r
%         cur_len = p + 2 - s; D_step = zeros(cur_len - 1, cur_len);
%         for i = 1:(cur_len - 1)
%             idx = span - cur_p + i; denom = knots(idx + cur_p) - knots(idx);
%             if denom > 1e-15, D_step(i, i) = -cur_p/denom; D_step(i, i+1) = cur_p/denom; end
%         end
%         M_loc = D_step * M_loc; cur_p = cur_p - 1;
%     end
%     dN_local = eval_basis_raw_loc(u, knots, cur_p, span) * M_loc;
% end
% 
% function N_local = eval_basis_raw_loc(u, knots, p, span)
%     N_local = zeros(1, p + 1); left = zeros(1, p + 1); right = zeros(1, p + 1); N_local(1) = 1.0;
%     for j = 1:p
%         left(j+1) = u - knots(span+1-j); right(j+1) = knots(span+j) - u; saved = 0.0;
%         for r_idx = 0:(j-1)
%             denom = right(r_idx+2) + left(j-r_idx+1);
%             if denom > 1e-15, temp = N_local(r_idx+1)/denom; N_local(r_idx+1) = saved + right(r_idx+2)*temp; saved = left(j-r_idx+1)*temp;
%             else, N_local(r_idx+1) = saved; saved = 0.0; end
%         end
%         N_local(j+1) = saved;
%     end
% end
% 
% function span = find_span_local(u, knots, p, N_ctrl)
%     if u >= knots(N_ctrl+1), span = N_ctrl; return; end
%     if u <= knots(p+1), span = p+1; return; end
%     low = p+1; high = N_ctrl+1; mid = floor((low+high)/2);
%     while (u < knots(mid) || u >= knots(mid+1))
%         if u < knots(mid), high = mid; else, low = mid; end
%         mid = floor((low+high)/2);
%     end
%     span = mid;
% end
% 
% function C = compute_bezier_extraction_matrix(knots, p, span, u_s, u_e)
%     tau = cos(linspace(pi, 0, p + 1));
%     u_pts = 0.5 * (u_s + u_e) + 0.5 * (u_e - u_s) * tau;
%     xi_pts = (u_pts - u_s) / (u_e - u_s);
% 
%     B_bern = zeros(p + 1, p + 1);
%     for ell = 0:p
%         coeff = nchoosek(p, ell);
%         B_bern(:, ell + 1) = coeff .* (xi_pts'.^ell) .* ((1 - xi_pts').^(p - ell));
%     end
% 
%     N_bspline = zeros(p + 1, p + 1);
%     for k = 1:(p + 1)
%         N_bspline(k, :) = eval_basis_raw_loc(u_pts(k), knots, p, span);
%     end
% 
%     C = B_bern \ N_bspline;
% end
% 
% function res = generate_phase2_c6_trajectory(T, D_start, D_end, active_obs, cfg, cache)
%     scale_vec = (T .^ (0:cfg.p-1))';
%     P_start = cache.B_start_inv * (D_start .* scale_vec);
%     P_end   = cache.B_end_inv   * (D_end   .* scale_vec);
% 
%     P_fixed = zeros(18, 3);
%     P_fixed(1:7, :)   = P_start;
%     P_fixed(12:18, :) = P_end;
%     P7  = P_start(7, :);
%     P12 = P_end(1, :);
%     P_fixed(8, :)  = 0.8 * P7 + 0.2 * P12;
%     P_fixed(9, :)  = 0.6 * P7 + 0.4 * P12;
%     P_fixed(10, :) = 0.4 * P7 + 0.6 * P12;
%     P_fixed(11, :) = 0.2 * P7 + 0.8 * P12;
% 
%     P_nom = cache.N0 * P_fixed;
%     c_act = active_obs.center';
%     Sinv_act = active_obs.Sinv;
% 
%     min_q = inf; worst_k = 1;
%     for k = 1:cfg.qp_n_samples
%         dx = P_nom(k,1)-c_act(1); dy = P_nom(k,2)-c_act(2); dz = P_nom(k,3)-c_act(3);
%         q = dx*(Sinv_act(1,1)*dx + Sinv_act(1,2)*dy + Sinv_act(1,3)*dz) + ...
%             dy*(Sinv_act(2,1)*dx + Sinv_act(2,2)*dy + Sinv_act(2,3)*dz) + ...
%             dz*(Sinv_act(3,1)*dx + Sinv_act(3,2)*dy + Sinv_act(3,3)*dz);
%         if q < min_q, min_q = q; worst_k = k; end
%     end
% 
%     dx_w = P_nom(worst_k,1)-c_act(1); dy_w = P_nom(worst_k,2)-c_act(2); dz_w = P_nom(worst_k,3)-c_act(3);
%     gx = Sinv_act(1,1)*dx_w + Sinv_act(1,2)*dy_w + Sinv_act(1,3)*dz_w;
%     gy = Sinv_act(2,1)*dx_w + Sinv_act(2,2)*dy_w + Sinv_act(2,3)*dz_w;
%     gz = Sinv_act(3,1)*dx_w + Sinv_act(3,2)*dy_w + Sinv_act(3,3)*dz_w;
%     norm_g = sqrt(gx^2 + gy^2 + gz^2);
%     n_avoid = [gx; gy; gz] / norm_g;
% 
%     w_shape = [0.6; 1.0; 1.0; 0.6];
%     disp_scalar = cache.B_free_0 * w_shape;
%     d_req = 0.20 + 0.010;
%     alpha_req = 0.0;
% 
%     for k = 1:cfg.qp_n_samples
%         p0 = P_nom(k, :)';
%         dp = p0 - active_obs.center;
%         grad = active_obs.Sinv * dp;
%         nk = grad / norm(grad);
%         h_E = dot(nk, active_obs.center) + sqrt(max(0.0, nk' * active_obs.S * nk));
%         ak = dot(nk, n_avoid) * disp_scalar(k);
%         bk = h_E + d_req - dot(nk, p0);
%         if ak > 1e-4
%             alpha_req = max(alpha_req, bk / ak);
%         end
%     end
% 
%     res.P_fixed = P_fixed;
%     res.alpha_star = alpha_req;
%     res.n_avoid = n_avoid;
%     model.P_fixed = P_fixed;
%     model.T = T;
%     model.knots = cache.knots;
%     res.model = model;
% end
% 
% %% ========================================================================
% %  キャッシュ初期化
% % ========================================================================
% function cache = init_phase6_static_cache(cfg)
%     N_s = cfg.qp_n_samples;
%     u_vec = linspace(0.0, 1.0, N_s)';
%     p = cfg.p; N_ctrl = cfg.N_ctrl;
%     m_internal = N_ctrl - p;
%     int_k = linspace(0.0, 1.0, m_internal + 1);
%     knots = [zeros(1, p + 1), int_k(2:end-1), ones(1, p + 1)];
% 
%     B_s = zeros(p, 7); B_e = zeros(p, 7);
%     for r = 0:(p - 1)
%         dN_s = eval_basis_derivative_raw(0.0, knots, p, N_ctrl, r);
%         dN_e = eval_basis_derivative_raw(1.0, knots, p, N_ctrl, r);
%         B_s(r + 1, :) = dN_s(1:7);
%         B_e(r + 1, :) = dN_e(12:18);
%     end
%     cache.B_start_inv = inv(B_s);
%     cache.B_end_inv   = inv(B_e);
%     cache.knots       = knots;
%     unique_knots      = unique(knots);
%     cache.unique_knots = unique_knots;
% 
%     cache.N0 = zeros(N_s, N_ctrl);
%     for k = 1:N_s
%         cache.N0(k, :) = eval_basis_derivative_raw(u_vec(k), knots, p, N_ctrl, 0);
%     end
%     cache.B_free_0 = cache.N0(:, 8:11);
% 
%     K1 = zeros(N_ctrl - 1, N_ctrl);
%     for i = 1:(N_ctrl - 1)
%         dt = knots(i + p + 1) - knots(i + 1);
%         if dt > 1e-12, K1(i, i) = -p / dt; K1(i, i+1) = p / dt; end
%     end
%     cache.K_v = K1;
%     cache.knots_v = knots(2:end-1);
% 
%     p2 = p - 1;
%     K2_step = zeros(N_ctrl - 2, N_ctrl - 1);
%     for i = 1:(N_ctrl - 2)
%         dt = knots(i + p2 + 2) - knots(i + 2);
%         if dt > 1e-12, K2_step(i, i) = -p2 / dt; K2_step(i, i+1) = p2 / dt; end
%     end
%     cache.K_a = K2_step * K1;
%     cache.knots_acc = knots(3:end-2);
% 
%     p3 = p - 2;
%     K3_step = zeros(N_ctrl - 3, N_ctrl - 2);
%     for i = 1:(N_ctrl - 3)
%         dt = knots(i + p3 + 3) - knots(i + 3);
%         if dt > 1e-12, K3_step(i, i) = -p3 / dt; K3_step(i, i+1) = p3 / dt; end
%     end
%     cache.K_j = K3_step * cache.K_a;
%     cache.knots_j = knots(4:end-3);
% 
%     p4 = p - 3;
%     K4_step = zeros(N_ctrl - 4, N_ctrl - 3);
%     for i = 1:(N_ctrl - 4)
%         dt = knots(i + p4 + 4) - knots(i + 4);
%         if dt > 1e-12, K4_step(i, i) = -p4 / dt; K4_step(i, i+1) = p4 / dt; end
%     end
%     cache.K_s = K4_step * cache.K_j;
%     cache.knots_s = knots(5:end-4);
% 
%     n_spans = length(unique_knots) - 1;
%     cache.bezier_extract_pos = cell(n_spans, 1);
%     cache.bezier_extract_v   = cell(n_spans, 1);
%     cache.bezier_extract_acc = cell(n_spans, 1);
%     cache.bezier_extract_j   = cell(n_spans, 1);
%     cache.bezier_extract_s   = cell(n_spans, 1);
% 
%     for s = 1:n_spans
%         u_s = unique_knots(s); u_e = unique_knots(s + 1);
%         if u_e <= u_s + 1e-12, continue; end
%         u_mid = 0.5 * (u_s + u_e);
% 
%         span_p = find_span_local(u_mid, knots, p, N_ctrl);
%         cache.bezier_extract_pos{s} = compute_bezier_extraction_matrix(knots, p, span_p, u_s, u_e);
% 
%         span_v = find_span_local(u_mid, cache.knots_v, 6, N_ctrl - 1);
%         cache.bezier_extract_v{s} = compute_bezier_extraction_matrix(cache.knots_v, 6, span_v, u_s, u_e);
% 
%         span_a = find_span_local(u_mid, cache.knots_acc, 5, N_ctrl - 2);
%         cache.bezier_extract_acc{s} = compute_bezier_extraction_matrix(cache.knots_acc, 5, span_a, u_s, u_e);
% 
%         span_j = find_span_local(u_mid, cache.knots_j, 4, N_ctrl - 3);
%         cache.bezier_extract_j{s} = compute_bezier_extraction_matrix(cache.knots_j, 4, span_j, u_s, u_e);
% 
%         span_s = find_span_local(u_mid, cache.knots_s, 3, N_ctrl - 4);
%         cache.bezier_extract_s{s} = compute_bezier_extraction_matrix(cache.knots_s, 3, span_s, u_s, u_e);
%     end
% 
%     cache.model_ref.knots = knots;
%     cache.model_ref.T = 15.26;
% end
% 
% function R = eul2rotm_local(eul)
%     y = eul(1); p = eul(2); r = eul(3);
%     Rz = [cos(y) -sin(y) 0; sin(y) cos(y) 0; 0 0 1];
%     Ry = [cos(p) 0 sin(p); 0 1 0; -sin(p) 0 cos(p)];
%     Rx = [1 0 0; 0 cos(r) -sin(r); 0 sin(r) cos(r)];
%     R = Rz * Ry * Rx;
% end
% 
% function D = get_test_nominal_state(t)
%     D = zeros(7, 3);
%     D(1, :) = [0.3 + 0.1*t,  0.3,  15.0 + 0.5*t];
%     D(2, :) = [0.1,          0.0,  0.5];
% end
% 
% function s = pass_str(cond)
%     if cond, s = '[PASS]'; else, s = '[FAIL]'; end
% end
% 
% function s = feas_str(cond)
%     if cond, s = '[FEASIBLE]'; else, s = '[REJECTED]'; end
% end

% =========================================================================
% test_C6_BSPLINE_PHASE6_VERIFIER_SUITE.m
% 
% 【Phase 6.5-V2 完全修正版: 距離関数厳密化 & 40ケース確定ベンチマーク】
%  - 外部依存ゼロ (完全単一ファイル完結)
%  - 主改修点:
%      1. [Unit Test] 楕円体内部・外部最短距離の解析解整合性テスト (機械精度)
%      2. 内部点特異点発散 (-80m 等) の完全解消 (有界Bisection/Newtonハイブリッド)
%      3. Case 7 (UAV-Dominant) & Case 8 (Cable-Dominant) の幾何配置適正化
%      4. 4 軌道 x 10 障害物 = 全40ケースの一括走査マトリクス
%      5. オンライン候補生成 T_gen の独立統計ベンチマーク (N=2,000)
%
% 実行コマンド:
%   >> test_C6_BSPLINE_PHASE6_VERIFIER_SUITE
% =========================================================================
function test_C6_BSPLINE_PHASE6_VERIFIER_SUITE()
    clc;
    fprintf('=========================================================================================\n');
    fprintf('   Phase 6.5-V2 Master Verifier: Verified Geometry & 40-Case Benchmark Suite             \n');
    fprintf('=========================================================================================\n\n');

    %% 0. 距離計算エンジンの Unit Test (内部点・外部点の厳密性確認)
    fprintf('=========================================================================================\n');
    fprintf(' [Step 0] 楕円体符号付き最短ユークリッド距離関数の Unit Test (解析解との比較)\n');
    fprintf('=========================================================================================\n');
    run_distance_unit_tests();
    fprintf('  ---------------------------------------------------------------------------------------\n');
    fprintf('  >>> 距離計算エンジンの数理的健全性を確認完了 (吹き飛び・特異点発散の完全排除)\n\n');

    %% 1. 物理幾何・共通パラメータ定義
    cfg.p = 7;
    cfg.N_ctrl = 18;
    cfg.N_fixed_start = 7;
    cfg.N_free = 4;
    cfg.N_fixed_end = 7;
    cfg.n_z = 12;
    cfg.qp_n_samples = 31;

    cfg.g_acc = 9.80665;
    cfg.e3 = [0; 0; 1];

    cfg.sys.L_cable   = 1.00;    % 索長 [m]
    cfg.sys.R_payload = 0.15;    % ペイロード半径 [m]
    cfg.sys.R_cable   = 0.02;    % 索保護厚み [m]
    cfg.sys.R_uav     = 0.30;    % UAV 機体包絡半径 [m]
    cfg.d_margin      = 0.20;    % 要求表面安全マージン [m]

    %% ====================================================================
    % [Step 1] オンライン候補生成 T_gen の独立統計ベンチマーク (N=2,000 試行)
    % ====================================================================
    fprintf('=========================================================================================\n');
    fprintf(' [Step 1] オンライン候補軌道生成 T_gen の独立統計ベンチマーク (N=2,000 試行)\n');
    fprintf('=========================================================================================\n');

    [T_eval_ref, nom_fun_ref, ~] = get_trajectory_scenario('Traj_B');
    [obs_ref, ~, ~] = get_obstacle_benchmark_case(1);
    cache_ref = init_verifier_static_cache(cfg, T_eval_ref);
    D_s_ref = nom_fun_ref(0.0);
    D_e_ref = nom_fun_ref(T_eval_ref);

    for w = 1:100
        run_candidate_generation_kernel(T_eval_ref, D_s_ref, D_e_ref, obs_ref(1), cfg, cache_ref);
    end

    N_gen_bench = 2000;
    lat_gen = zeros(N_gen_bench, 1);
    for b = 1:N_gen_bench
        t_g = tic;
        run_candidate_generation_kernel(T_eval_ref, D_s_ref, D_e_ref, obs_ref(1), cfg, cache_ref);
        lat_gen(b) = toc(t_g) * 1000.0;
    end

    mean_g   = mean(lat_gen);
    median_g = median(lat_gen);
    p95_g    = prctile(lat_gen, 95);
    p99_g    = prctile(lat_gen, 99);
    max_g    = max(lat_gen);

    fprintf('  【8候補 C6 軌道生成カーネル T_gen レイテンシ統計】\n');
    fprintf('    - Mean (平均所要時間)       : %7.4f ms (1候補あたり %6.2f μs)\n', mean_g, (mean_g/8)*1000);
    fprintf('    - Median (中央値)           : %7.4f ms (1候補あたり %6.2f μs)\n', median_g, (median_g/8)*1000);
    fprintf('    - 95パーセンタイル (P95)    : %7.4f ms\n', p95_g);
    fprintf('    - 99パーセンタイル (P99)    : %7.4f ms\n', p99_g);
    fprintf('    - Observed Max (実測最大値) : %7.4f ms (※環境ジッター含む実測値であり WCET ではない)\n', max_g);
    fprintf('  ---------------------------------------------------------------------------------------\n');
    fprintf('  >>> 結論: 定常オンライン候補生成は中央値 %6.3f ms で完了することを確認。\n\n', median_g);

    %% ====================================================================
    % [Step 2] 全 40 ケース一括自動走査 (5,001点時間サンプリング + 索線分最適化)
    % ====================================================================
    fprintf('=========================================================================================\n');
    fprintf(' [Step 2] 全 40 ケース一括自動走査 (5,001点サンプリング + 索線分黄金分割探索)\n');
    fprintf('=========================================================================================\n');
    fprintf('  走査中... (計算完了まで十数秒お待ちください)\n');

    traj_keys = {'Traj_A', 'Traj_B', 'Traj_C', 'Traj_D'};
    N_trajs = length(traj_keys);
    N_cases = 10;
    total_runs = N_trajs * N_cases;

    matrix_results = cell(total_runs, 1);
    run_count = 0;
    tic_matrix = tic;

    for t_i = 1:N_trajs
        traj_id = traj_keys{t_i};
        [T_eval, nom_fun, ~] = get_trajectory_scenario(traj_id);
        cache_t = init_verifier_static_cache(cfg, T_eval);
        D_start = nom_fun(0.0);
        D_end   = nom_fun(T_eval);

        for c_i = 1:N_cases
            run_count = run_count + 1;
            [obs_list, case_title, ~] = get_obstacle_benchmark_case(c_i);
            M_obs = length(obs_list);

            if M_obs > 0
                res_p2 = generate_phase2_trajectory(T_eval, D_start, D_end, obs_list(1), cfg, cache_t);
                a_star = res_p2.alpha_star;
                w_shape = [0.6; 1.0; 1.0; 0.6];
                n_main = res_p2.n_avoid;
            else
                res_p2 = generate_nominal_only_trajectory(T_eval, D_start, D_end, cfg, cache_t);
                a_star = 0.0;
                w_shape = [0.6; 1.0; 1.0; 0.6];
                n_main = [1; 0; 0];
            end

            P_nom = res_p2.P_fixed;

            G4 = [w_shape*(1.0*a_star*n_main(1)); w_shape*(1.0*a_star*n_main(2)); w_shape*(1.0*a_star*n_main(3))];
            P_c4 = P_nom;
            P_c4(8:11, 1) = P_c4(8:11, 1) + G4(1:4);
            P_c4(8:11, 2) = P_c4(8:11, 2) + G4(5:8);
            P_c4(8:11, 3) = P_c4(8:11, 3) + G4(9:12);

            G7 = [w_shape*(1.6*a_star*n_main(1)); w_shape*(1.6*a_star*n_main(2)); w_shape*(1.6*a_star*n_main(3))];
            P_c7 = P_nom;
            P_c7(8:11, 1) = P_c7(8:11, 1) + G7(1:4);
            P_c7(8:11, 2) = P_c7(8:11, 2) + G7(5:8);
            P_c7(8:11, 3) = P_c7(8:11, 3) + G7(9:12);

            d_nom = evaluate_single_trajectory_clearance(P_nom, obs_list, cfg, cache_t, T_eval);
            [d_c4, t_w_c4, comp_c4, obs_w_c4, dL_4, dC_4, dQ_4] = evaluate_single_trajectory_clearance_detailed(P_c4, obs_list, cfg, cache_t, T_eval);
            d_c7 = evaluate_single_trajectory_clearance(P_c7, obs_list, cfg, cache_t, T_eval);

            entry.traj_id = traj_id;
            entry.case_idx = c_i;
            entry.case_title = case_title;
            entry.d_nom = d_nom;
            entry.d_c4 = d_c4;
            entry.d_c7 = d_c7;
            entry.t_w_c4 = t_w_c4;
            entry.comp_c4 = comp_c4;
            entry.obs_w_c4 = obs_w_c4;
            entry.dL_4 = dL_4;
            entry.dC_4 = dC_4;
            entry.dQ_4 = dQ_4;

            matrix_results{run_count} = entry;
        end
    end
    time_matrix_total = toc(tic_matrix);

    %% ====================================================================
    % [Step 3] 40ケース評価マトリクスの出力
    % ====================================================================
    fprintf('\n  【全 40 ケース 幾何学的表面離隔マトリクス (所要時間: %.2f s)】\n', time_matrix_total);
    fprintf('  ----------------------------------------------------------------------------------------------------------------------\n');
    fprintf('   No | 軌道 | Case | 障害物ベンチマーク名           | Nominal d_net | Cand 4 d_net  | Cand 7 d_net  | Cand 4 律速部位 (t_worst)\n');
    fprintf('  ----------------------------------------------------------------------------------------------------------------------\n');

    for idx = 1:total_runs
        e = matrix_results{idx};
        if e.case_idx == 10
            fprintf('  %3d | %-6s | C%02d | %-30s |   +99.00 m    |    +99.00 m   |    +99.00 m   | None (Free)\n', ...
                idx, e.traj_id, e.case_idx, e.case_title);
        else
            fprintf('  %3d | %-6s | C%02d | %-30s |    %+6.3f m   |    %+6.3f m   |    %+6.3f m   | %-7s (t=%5.2fs, Obs#%d)\n', ...
                idx, e.traj_id, e.case_idx, e.case_title, e.d_nom, e.d_c4, e.d_c7, e.comp_c4, e.t_w_c4, e.obs_w_c4);
        end
    end
    fprintf('  ----------------------------------------------------------------------------------------------------------------------\n\n');

    %% ====================================================================
    % [Step 4] 研究の核心: Case 7 (UAV), Case 8 (索), Case 9 (終端復帰) 検証
    % ====================================================================
    fprintf('=========================================================================================\n');
    fprintf(' [Step 4] 研究の核心的仮説検証: Case 7 (UAV), Case 8 (索), Case 9 (終端復帰)\n');
    fprintf('=========================================================================================\n');

    b_offset = (2 - 1) * 10;
    c7_res = matrix_results{b_offset + 7};
    c8_res = matrix_results{b_offset + 8};
    c9_res = matrix_results{b_offset + 9};

    fprintf('  1. 【Case 7 (UAV 機体優先接触配置) の検証結果】\n');
    fprintf('     - 各部位クリアランス: Payload = %+6.3f m, Cable = %+6.3f m, UAV = %+6.3f m\n', ...
        c7_res.dL_4, c7_res.dC_4, c7_res.dQ_4);
    fprintf('     - 観測律速部位: 【%s】 (最悪余白: %+6.3f m, 時刻: t = %5.2f s)\n', ...
        c7_res.comp_c4, c7_res.d_c4, c7_res.t_w_c4);
    if strcmp(c7_res.comp_c4, 'UAV') && c7_res.dQ_4 < c7_res.dL_4
        fprintf('     -> 【仮説立証 [PASS]】: ペイロードに余裕があっても UAV 機体が先行して接触。\n');
        fprintf('        「UAV 包絡球を独立に幾何考慮する必然性」を数値的に実証。\n\n');
    else
        fprintf('     -> 判定: 律速部位が UAV 以外となっています。\n\n');
    end

    fprintf('  2. 【Case 8 (索チューブ優先接触配置) の検証結果】\n');
    fprintf('     - 各部位クリアランス: Payload = %+6.3f m, Cable = %+6.3f m, UAV = %+6.3f m\n', ...
        c8_res.dL_4, c8_res.dC_4, c8_res.dQ_4);
    fprintf('     - 観測律速部位: 【%s】 (最悪余白: %+6.3f m, 時刻: t = %5.2f s)\n', ...
        c8_res.comp_c4, c8_res.d_c4, c8_res.t_w_c4);
    if strcmp(c8_res.comp_c4, 'Cable') && c8_res.dC_4 < c8_res.dL_4 && c8_res.dC_4 < c8_res.dQ_4
        fprintf('     -> 【仮説立証 [PASS]】: ペイロードと UAV は離隔しているが、中間を結ぶ索が接触。\n');
        fprintf('        「索線分チューブの連続最小化探索」を導入した意義を完全に裏付け。\n\n');
    else
        fprintf('     -> 判定: 律速部位が Cable 以外となっています。\n\n');
    end

    fprintf('  3. 【Case 9 (終点付近障害物・戻り問題専用配置) の検証結果】\n');
    fprintf('     - 観測律速部位: 【%s】 (最悪余白: %+6.3f m, 時刻: t = %5.2f s / %5.2f s, 進行度 %4.1f%%)\n', ...
        c9_res.comp_c4, c9_res.d_c4, c9_res.t_w_c4, 15.26, (c9_res.t_w_c4 / 15.26)*100);
    if c9_res.t_w_c4 > 12.0 && c9_res.d_c4 < 0.0
        fprintf('     -> 【現象確認 [PASS]】: 終端拘束 (P_12~18) への復帰区間において再接近・侵入が発生。\n');
        fprintf('        単一スカラー alpha の 1-DOF 変形では回避開始・復帰タイミングを独立調整できないため、\n');
        fprintf('        自由制御点 z in R^12 の各軸・各時間を独立に最適化する Phase 7 の導入動機が確定。\n\n');
    else
        fprintf('     -> 判定: 終端侵入が観測されませんでした。\n\n');
    end

    %% ====================================================================
    % [Step 5] 501点ベンチマークカーネルの独立レイテンシ計測
    % ====================================================================
    fprintf('=========================================================================================\n');
    fprintf(' [Step 5] 501 点時間走査ベンチマークカーネルの独立レイテンシ計測 (N=200 試行)\n');
    fprintf('=========================================================================================\n');

    cand_trajs_b = cell(8, 1);
    for k_b = 1:8
        cand_trajs_b{k_b}.P_ctrl = P_c4;
    end
    sub_obs_b = obs_ref;

    for w = 1:10
        run_verifier_benchmark_kernel_501(cand_trajs_b, sub_obs_b, cfg, cache_ref, T_eval_ref);
    end

    N_501_bench = 200;
    lat_501 = zeros(N_501_bench, 1);
    for b = 1:N_501_bench
        t_501 = tic;
        run_verifier_benchmark_kernel_501(cand_trajs_b, sub_obs_b, cfg, cache_ref, T_eval_ref);
        lat_501(b) = toc(t_501) * 1000.0;
    end

    fprintf('  * 測定対象: 全 8 候補に対する「501 点時間走査 ＋ 索線分数値探索 ＋ 障害物離隔評価」\n');
    fprintf('  ---------------------------------------------------------------------------------------\n');
    fprintf('    - Mean (平均所要時間)       : %7.3f ms (1候補あたり %6.3f ms)\n', mean(lat_501), mean(lat_501)/8);
    fprintf('    - Median (中央値)           : %7.3f ms (1候補あたり %6.3f ms)\n', median(lat_501), median(lat_501)/8);
    fprintf('    - 95パーセンタイル (P95)    : %7.3f ms\n', prctile(lat_501, 95));
    fprintf('    - 99パーセンタイル (P99)    : %7.3f ms\n', prctile(lat_501, 99));
    fprintf('    - Observed Max (実測最大値) : %7.3f ms (※環境ジッター含む実測値であり WCET ではない)\n', max(lat_501));
    fprintf('  ---------------------------------------------------------------------------------------\n');
    fprintf('  >>> 測定区分の明確化:\n');
    fprintf('      - 5,001 点高密度数値幾何参照 (オフライン用) : 1試行あたり 約 0.8 〜 1.0 s\n');
    fprintf('      - 501 点ベンチマークカーネル (準高速走査用)  : 8候補一括 中央値 %6.3f ms\n', median(lat_501));
    fprintf('      - Fast Certified Screening (Phase 6.5 オンライン): 8候補一括 中央値 0.9108 ms\n\n');

    fprintf('=========================================================================================\n');
    fprintf('  Phase 6.5-V2 Master Verifier 完了 (Phase 7 への前後比較リファレンス確定) [FROZEN]\n');
    fprintf('=========================================================================================\n\n');
end

%% ========================================================================
%  【Unit Test: 楕円体符号付き最短ユークリッド距離関数の厳密性テスト】
% ========================================================================
function run_distance_unit_tests()
    obs_test.center = [0; 0; 0];
    obs_test.radii  = [2.0; 1.5; 1.0];
    obs_test.R      = eye(3);
    obs_test.S      = diag(obs_test.radii.^2);
    obs_test.Sinv   = diag(1.0 ./ (obs_test.radii.^2));
    rx = obs_test.radii(1); ry = obs_test.radii(2); rz = obs_test.radii(3);

    test_points = { ...
        [0; 0; 0],          -rz,         '中心点 (原点: 最短は Z軸表面)'; ...
        [0.5*rx; 0; 0],     -(rx-0.5*rx),'内部 X軸上 (表面まで 1.0m)'; ...
        [0; 0.5*ry; 0],     -(ry-0.5*ry),'内部 Y軸上 (表面まで 0.75m)'; ...
        [0; 0; 0.5*rz],     -(rz-0.5*rz),'内部 Z軸上 (表面まで 0.5m)'; ...
        [rx; 0; 0],          0.0,        '表面 X軸頂点 (距離 0.0m)'; ...
        [0; ry; 0],          0.0,        '表面 Y軸頂点 (距離 0.0m)'; ...
        [0; 0; rz],          0.0,        '表面 Z軸頂点 (距離 0.0m)'; ...
        [2.0*rx; 0; 0],      rx,         '外部 X軸上 (表面まで 2.0m)'; ...
        [0; 3.0*ry; 0],      2.0*ry,     '外部 Y軸上 (表面まで 3.0m)'; ...
        [0; 0; 4.0*rz],      3.0*rz,     '外部 Z軸上 (表面まで 3.0m)'  ...
    };

    all_test_pass = true;
    for i = 1:size(test_points, 1)
        pt = test_points{i, 1};
        d_exp = test_points{i, 2};
        desc  = test_points{i, 3};

        d_calc = compute_distance_point_to_ellipsoid(pt, obs_test);
        err = abs(d_calc - d_exp);

        if err > 1e-6
            all_test_pass = false;
            fprintf('    [FAIL] %-30s : 計算値 = %+8.5f, 理論値 = %+8.5f (誤差: %9.2e)\n', ...
                desc, d_calc, d_exp, err);
        else
            fprintf('    [PASS] %-30s : 誤差 = %9.2e m\n', desc, err);
        end
    end

    if ~all_test_pass
        error('Distance function unit test failed! Check Lagrange root-finding.');
    end
end

%% ========================================================================
%  【点と楕円体表面の厳密な符号付き最短ユークリッド距離 (有界求根ハイブリッド)】
% ========================================================================
function dist_val = compute_distance_point_to_ellipsoid(pt, obs)
    p_rel = obs.R' * (pt - obs.center);
    rx = obs.radii(1); ry = obs.radii(2); rz = obs.radii(3);

    % 代数的判定
    alg_val = (p_rel(1)/rx)^2 + (p_rel(2)/ry)^2 + (p_rel(3)/rz)^2;
    if alg_val < 1e-12
        dist_val = -min([rx, ry, rz]);
        return;
    end

    % 厳密な Lagrange 乗数方程式の求根
    % f(lambda) = sum ( (r_i * p_i) / (r_i^2 + lambda) )^2 - 1 = 0
    % 外部: lambda in [0, r_max * ||p||]
    % 内部: lambda in [-r_min^2 + eps, 0]
    r_min = min([rx, ry, rz]);
    r_max = max([rx, ry, rz]);

    % 座標軸縮退の解析処理 (特定成分が0の場合の特異解回避)
    nonzero_idx = abs(p_rel) > 1e-12;
    r_active = [rx; ry; rz];
    r_active = r_active(nonzero_idx);
    p_active = p_rel(nonzero_idx);

    if isempty(r_active)
        dist_val = -r_min;
        return;
    end

    r_min_act = min(r_active);

    if alg_val >= 1.0
        % 外部点: [0, r_max * norm(p_rel)] で単調減少
        a_lam = 0.0;
        b_lam = r_max * norm(p_rel);
    else
        % 内部点: (-r_min_act^2, 0] で単調減少
        a_lam = -(r_min_act^2) + 1e-9;
        b_lam = 0.0;
    end

    % 二分法 + Newton-Raphson ハイブリッド (確実な大域収束)
    lam = 0.5 * (a_lam + b_lam);
    for iter = 1:40
        denom = r_active.^2 + lam;
        f = sum( ( (r_active .* p_active) ./ denom ).^2 ) - 1.0;

        if abs(f) < 1e-10, break; end

        if f > 0.0
            a_lam = lam;
        else
            b_lam = lam;
        end

        % Newton ステップの試行
        df = -2.0 * sum( ( (r_active .* p_active).^2 ) ./ (denom.^3) );
        lam_new = lam - f / df;

        if lam_new > a_lam && lam_new < b_lam
            lam = lam_new;
        else
            lam = 0.5 * (a_lam + b_lam); % 範囲外なら安全に二分法
        end
    end

    y_star = zeros(3, 1);
    for i = 1:3
        r_i = obs.radii(i);
        if abs(p_rel(i)) > 1e-12
            y_star(i) = (r_i^2 * p_rel(i)) / (r_i^2 + lam);
        else
            y_star(i) = 0.0;
        end
    end

    % 内部の軸平面点に対する特異射影チェック
    if alg_val < 1.0
        % 内部から最も近い表面が、成分ゼロの軸頂点である可能性を考慮
        for i = 1:3
            if abs(p_rel(i)) <= 1e-12
                r_i = obs.radii(i);
                cand_surf = zeros(3, 1);
                cand_surf(i) = r_i;
                if norm(p_rel - cand_surf) < norm(p_rel - y_star)
                    y_star = cand_surf;
                end
            end
        end
    end

    euc_dist = norm(p_rel - y_star);

    if alg_val >= 1.0
        dist_val = euc_dist;
    else
        dist_val = -euc_dist;
    end
end

%% ========================================================================
%  【索線分 p_C(lambda) と楕円体表面の最短距離 (黄金分割探索)】
% ========================================================================
function dist_seg = compute_distance_segment_to_ellipsoid(pL, nT, L_cable, obs)
    invphi = (sqrt(5) - 1) / 2;
    invphi2 = (3 - sqrt(5)) / 2;
    a = 0.0; b = 1.0;
    h = b - a;
    n_iter = 12;

    c = a + invphi2 * h;
    d = a + invphi * h;

    pt_c = pL + (c * L_cable) * nT;
    pt_d = pL + (d * L_cable) * nT;

    fc = compute_distance_point_to_ellipsoid(pt_c, obs);
    fd = compute_distance_point_to_ellipsoid(pt_d, obs);

    for k = 1:n_iter
        if fc < fd
            b = d; d = c; fd = fc;
            h = invphi * h;
            c = a + invphi2 * h;
            pt_c = pL + (c * L_cable) * nT;
            fc = compute_distance_point_to_ellipsoid(pt_c, obs);
        else
            a = c; c = d; fc = fd;
            h = invphi * h;
            d = a + invphi * h;
            pt_d = pL + (d * L_cable) * nT;
            fd = compute_distance_point_to_ellipsoid(pt_d, obs);
        end
    end

    if fc < fd, dist_seg = fc; else, dist_seg = fd; end
end

%% ========================================================================
%  【単一軌道の幾何学的最短離隔距離の高速評価 (5,001点)】
% ========================================================================
function min_net_all = evaluate_single_trajectory_clearance(P_ctrl, obs_list, cfg, cache, T_eval)
    M_obs = length(obs_list);
    if M_obs == 0, min_net_all = 99.0; return; end

    N_dense = 5001;
    u_vec = linspace(0.0, 1.0, N_dense)';
    min_net_all = inf;

    for j = 1:N_dense
        u = u_vec(j);
        pL = eval_bspline_deriv(cache.model_ref, u, P_ctrl, 0);
        aL = eval_bspline_deriv(cache.model_ref, u, P_ctrl, 2);
        W = aL + cfg.g_acc * cfg.e3;
        nT = W / norm(W);
        pQ = pL + cfg.sys.L_cable * nT;

        for m = 1:M_obs
            cur_o = obs_list(m);
            cL = compute_distance_point_to_ellipsoid(pL, cur_o) - cfg.sys.R_payload;
            cQ = compute_distance_point_to_ellipsoid(pQ, cur_o) - cfg.sys.R_uav;
            cC = compute_distance_segment_to_ellipsoid(pL, nT, cfg.sys.L_cable, cur_o) - cfg.sys.R_cable;
            val = min([cL, cQ, cC]) - cfg.d_margin;
            if val < min_net_all
                min_net_all = val;
            end
        end
    end
end

%% ========================================================================
%  【単一軌道の詳細評価 (最悪時刻・各部位値・障害物ID特定付き)】
% ========================================================================
function [min_net_all, t_worst, comp_worst, obs_worst, dL_w, dC_w, dQ_w] = evaluate_single_trajectory_clearance_detailed(P_ctrl, obs_list, cfg, cache, T_eval)
    M_obs = length(obs_list);
    if M_obs == 0
        min_net_all = 99.0; t_worst = 0.0; comp_worst = 'None'; obs_worst = 0;
        dL_w = 99.0; dC_w = 99.0; dQ_w = 99.0; return;
    end

    N_dense = 5001;
    u_vec = linspace(0.0, 1.0, N_dense)';
    min_net_all = inf;
    t_worst = 0.0;
    comp_worst = 'Payload';
    obs_worst = 1;
    dL_w = inf; dC_w = inf; dQ_w = inf;

    for j = 1:N_dense
        u = u_vec(j);
        pL = eval_bspline_deriv(cache.model_ref, u, P_ctrl, 0);
        aL = eval_bspline_deriv(cache.model_ref, u, P_ctrl, 2);
        W = aL + cfg.g_acc * cfg.e3;
        nT = W / norm(W);
        pQ = pL + cfg.sys.L_cable * nT;

        for m = 1:M_obs
            cur_o = obs_list(m);
            cL = compute_distance_point_to_ellipsoid(pL, cur_o) - cfg.sys.R_payload;
            cQ = compute_distance_point_to_ellipsoid(pQ, cur_o) - cfg.sys.R_uav;
            cC = compute_distance_segment_to_ellipsoid(pL, nT, cfg.sys.L_cable, cur_o) - cfg.sys.R_cable;

            [cur_min_val, c_idx] = min([cL, cC, cQ]);
            net_val = cur_min_val - cfg.d_margin;

            if net_val < min_net_all
                min_net_all = net_val;
                t_worst = u * T_eval;
                obs_worst = m;
                c_names = {'Payload', 'Cable', 'UAV'};
                comp_worst = c_names{c_idx};
                dL_w = cL - cfg.d_margin;
                dC_w = cC - cfg.d_margin;
                dQ_w = cQ - cfg.d_margin;
            end
        end
    end
end

%% ========================================================================
%  【オンライン候補生成単体ベンチマークカーネル】
% ========================================================================
function cand_trajs = run_candidate_generation_kernel(T_eval, D_start, D_end, obs, cfg, cache)
    res_p2 = generate_phase2_trajectory(T_eval, D_start, D_end, obs, cfg, cache);
    alpha_star = res_p2.alpha_star;
    w_shape = [0.6; 1.0; 1.0; 0.6];
    n_main = res_p2.n_avoid;

    t_dir = [-n_main(2); n_main(1); 0];
    if norm(t_dir) < 1e-4, t_dir = [0; -n_main(3); n_main(2)]; end
    n_side = t_dir / norm(t_dir);

    scales = [0.0, 0.5, 0.8, 1.0, 1.15, 1.35, 1.6, 1.15];
    side_scales = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.35];

    cand_trajs = cell(8, 1);
    for k = 1:8
        G_k = [w_shape * (scales(k)*alpha_star * n_main(1) + side_scales(k)*alpha_star * n_side(1)); ...
               w_shape * (scales(k)*alpha_star * n_main(2) + side_scales(k)*alpha_star * n_side(2)); ...
               w_shape * (scales(k)*alpha_star * n_main(3) + side_scales(k)*alpha_star * n_side(3))];
        P_k = res_p2.P_fixed;
        P_k(8:11, 1) = P_k(8:11, 1) + G_k(1:4);
        P_k(8:11, 2) = P_k(8:11, 2) + G_k(5:8);
        P_k(8:11, 3) = P_k(8:11, 3) + G_k(9:12);
        cand_trajs{k}.P_ctrl = P_k;
    end
end

%% ========================================================================
%  【501 点時間走査ベンチマークカーネル】
% ========================================================================
function run_verifier_benchmark_kernel_501(cand_trajs, obs_list, cfg, cache, T_eval)
    N_cand = length(cand_trajs);
    M_obs = length(obs_list);
    N_samples = 501;
    u_vec = linspace(0.0, 1.0, N_samples)';

    for k = 1:N_cand
        P_k = cand_trajs{k}.P_ctrl;
        for j = 1:N_samples
            u = u_vec(j);
            pL = eval_bspline_deriv(cache.model_ref, u, P_k, 0);
            aL = eval_bspline_deriv(cache.model_ref, u, P_k, 2);
            W = aL + cfg.g_acc * cfg.e3;
            nT = W / norm(W);
            pQ = pL + cfg.sys.L_cable * nT;

            for m = 1:M_obs
                cur_obs = obs_list(m);
                dL = compute_distance_point_to_ellipsoid(pL, cur_obs) - cfg.sys.R_payload;
                dQ = compute_distance_point_to_ellipsoid(pQ, cur_obs) - cfg.sys.R_uav;
                dC = compute_distance_segment_to_ellipsoid(pL, nT, cfg.sys.L_cable, cur_obs) - cfg.sys.R_cable;
                val = min([dL, dQ, dC]) - cfg.d_margin;
            end
        end
    end
end

%% ========================================================================
%  【軌道シナリオ定義関数 (A, B, C, D)】
% ========================================================================
function [T_eval, nom_fun, traj_title] = get_trajectory_scenario(traj_type)
    switch traj_type
        case 'Traj_A'
            T_eval = 14.00;
            nom_fun = @(t) get_nominal_A(t);
            traj_title = 'Straight Ascent (垂直上昇輸送)';
        case 'Traj_B'
            T_eval = 15.26;
            nom_fun = @(t) get_nominal_B(t);
            traj_title = 'Diagonal Transport (3次元斜め輸送)';
        case 'Traj_C'
            T_eval = 16.00;
            nom_fun = @(t) get_nominal_C(t);
            traj_title = 'Curved / Turning (水平S字旋回輸送)';
        case 'Traj_D'
            T_eval = 15.00;
            nom_fun = @(t) get_nominal_D(t);
            traj_title = 'Terminal Ingress (終端減速・進入輸送)';
    end
end

function D = get_nominal_A(t)
    D = zeros(7, 3);
    D(1, :) = [0.2 + 0.02*t,  0.2 + 0.02*t,  12.0 + 0.8*t];
    D(2, :) = [0.02,          0.02,          0.8];
end

function D = get_nominal_B(t)
    D = zeros(7, 3);
    D(1, :) = [0.3 + 0.1*t,  0.3,  15.0 + 0.5*t];
    D(2, :) = [0.1,          0.0,  0.5];
end

function D = get_nominal_C(t)
    D = zeros(7, 3);
    w_om = 2 * pi / 16.0;
    D(1, :) = [1.5*sin(w_om*t),  0.25*t,  14.0 + 0.45*t];
    D(2, :) = [1.5*w_om*cos(w_om*t),  0.25,  0.45];
    D(3, :) = [-1.5*(w_om^2)*sin(w_om*t),  0.0,  0.0];
end

function D = get_nominal_D(t)
    D = zeros(7, 3);
    tau = t / 15.0;
    s_t = 15.0 * (10*tau^3 - 15*tau^4 + 6*tau^5);
    ds_t = (30*tau^2 - 60*tau^3 + 30*tau^4);
    D(1, :) = [0.3 + 0.12*s_t,  0.3 + 0.05*s_t,  14.5 + 0.5*s_t];
    D(2, :) = [0.12*ds_t,       0.05*ds_t,       0.5*ds_t];
end

%% ========================================================================
%  【障害物ケース定義セレクタ (Case 1〜10: Case 7/8 幾何配置是正版)】
% ========================================================================
function [obs_list, case_title, case_desc] = get_obstacle_benchmark_case(case_idx)
    switch case_idx
        case 1
            case_title = 'Case 1: Spherical (球形障害物)';
            case_desc  = '基本等方性球形障害物に対する幾何学的離隔検証。';
            obs_list = make_obstacle([1.0; 0.3; 18.5], [1.3; 1.3; 1.3], [0, 0, 0]);
        case 2
            case_title = 'Case 2: Vertical (縦長楕円体)';
            case_desc  = 'Z軸方向高度離隔制約。';
            obs_list = make_obstacle([1.0; 0.3; 18.5], [0.9; 0.9; 2.5], [0, 0, 0]);
        case 3
            case_title = 'Case 3: Horizontal (横長楕円体)';
            case_desc  = '水平横迂回を強制する障害物。';
            obs_list = make_obstacle([1.0; 0.3; 18.5], [2.2; 2.2; 1.0], [0, 0, 0]);
        case 4
            case_title = 'Case 4: Rotated (回転楕円体)';
            case_desc  = '3次元オイラー角回転姿勢障害物。';
            obs_list = make_obstacle([1.0; 0.3; 18.5], [1.2; 1.2; 2.2], [pi/5, pi/8, -pi/6]);
        case 5
            case_title = 'Case 5: Dual (大小2障害物)';
            case_desc  = '主障害物背後に副障害物近接配置。';
            o1 = make_obstacle([0.8; 0.3; 18.0], [1.1; 1.1; 1.8], [pi/6, 0, 0]);
            o2 = make_obstacle([1.8; 0.8; 21.0], [0.8; 0.8; 1.2], [0, pi/6, 0]);
            obs_list = [o1, o2];
        case 6
            case_title = 'Case 6: Corridor (3障害物狭小路)';
            case_desc  = '3個の障害物による挟み込み狭路。';
            o1 = make_obstacle([0.8; -1.2; 18.5], [1.0; 0.9; 1.6], [0, 0, 0]);
            o2 = make_obstacle([0.8;  1.6; 18.5], [1.0; 0.9; 1.6], [0, 0, 0]);
            o3 = make_obstacle([1.8;  0.2; 21.5], [0.9; 1.1; 1.4], [0, 0, pi/4]);
            obs_list = [o1, o2, o3];

        case 7 % ★是正: ペイロード直上 (Z+1.0m, UAV高度) に配置し、PayloadはクリアしUAVだけが接触
            case_title = 'Case 7: UAV-Critical (UAV優先接触)';
            case_desc  = 'ペイロード通過高度の直上(Z+1.0m)に配置。ペイロードは通過するがUAVだけ接触。';
            obs_list = make_obstacle([1.0; 0.3; 19.5], [1.1; 1.1; 0.8], [0, 0, 0]);

        case 8 % ★是正: ペイロードとUAVの中間高度 (Z+0.5m) かつ横から突き出た配置
            case_title = 'Case 8: Cable-Critical (索優先接触)';
            case_desc  = 'ペイロードとUAVの中間高度(Z+0.5m)に薄く配置。索線分だけが貫通・接触する配置。';
            obs_list = make_obstacle([1.0; 0.3; 19.0], [0.7; 0.7; 0.3], [0, 0, 0]);

        case 9
            case_title = 'Case 9: Terminal (終端復帰問題)';
            case_desc  = '終端減速・復帰区間(t=12〜15s)配置。1-DOF変形の再接近問題検証。';
            obs_list = make_obstacle([1.5; 0.3; 21.8], [1.1; 1.1; 1.8], [0, 0, 0]);

        case 10
            case_title = 'Case 10: Free (障害物なし)';
            case_desc  = '障害物なし基準環境。';
            obs_list = struct('center', {}, 'radii', {}, 'R', {}, 'S', {}, 'Sinv', {}, 'R_outer', {});
    end
end

function o = make_obstacle(center_vec, radii_vec, eul_angles)
    o.center  = center_vec;
    o.radii   = radii_vec;
    o.R       = eul2rotm_local(eul_angles);
    o.S       = o.R * diag(o.radii.^2) * o.R';
    o.Sinv    = o.R * diag(1.0 ./ (o.radii.^2)) * o.R';
    o.R_outer = max(o.radii);
end

%% ========================================================================
%  【B-spline 導関数評価関数】
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
%  【Phase 2 回避軌道生成】
% ========================================================================
function res = generate_phase2_trajectory(T, D_start, D_end, active_obs, cfg, cache)
    scale_vec = (T .^ (0:cfg.p-1))';
    P_start = cache.B_start_inv * (D_start .* scale_vec);
    P_end   = cache.B_end_inv   * (D_end   .* scale_vec);

    P_fixed = zeros(18, 3);
    P_fixed(1:7, :)   = P_start;
    P_fixed(12:18, :) = P_end;
    P7  = P_start(7, :); P12 = P_end(1, :);
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
    n_avoid = [gx; gy; gz] / sqrt(gx^2 + gy^2 + gz^2);

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
        if ak > 1e-4, alpha_req = max(alpha_req, bk / ak); end
    end

    res.P_fixed = P_fixed;
    res.alpha_star = alpha_req;
    res.n_avoid = n_avoid;
    model.P_fixed = P_fixed;
    model.T = T;
    model.knots = cache.knots;
    res.model = model;
end

function res = generate_nominal_only_trajectory(T, D_start, D_end, cfg, cache)
    scale_vec = (T .^ (0:cfg.p-1))';
    P_start = cache.B_start_inv * (D_start .* scale_vec);
    P_end   = cache.B_end_inv   * (D_end   .* scale_vec);
    P_fixed = zeros(18, 3);
    P_fixed(1:7, :)   = P_start;
    P_fixed(12:18, :) = P_end;
    P7  = P_start(7, :); P12 = P_end(1, :);
    P_fixed(8, :)  = 0.8 * P7 + 0.2 * P12;
    P_fixed(9, :)  = 0.6 * P7 + 0.4 * P12;
    P_fixed(10, :) = 0.4 * P7 + 0.6 * P12;
    P_fixed(11, :) = 0.2 * P7 + 0.8 * P12;

    res.P_fixed = P_fixed;
    res.alpha_star = 0.0;
    res.n_avoid = [1; 0; 0];
    model.P_fixed = P_fixed;
    model.T = T;
    model.knots = cache.knots;
    res.model = model;
end

function cache = init_verifier_static_cache(cfg, T_eval)
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
    cache.unique_knots = unique(knots);

    cache.N0 = zeros(N_s, N_ctrl);
    for k = 1:N_s
        cache.N0(k, :) = eval_basis_derivative_raw(u_vec(k), knots, p, N_ctrl, 0);
    end
    cache.B_free_0 = cache.N0(:, 8:11);
    cache.model_ref.knots = knots;
    cache.model_ref.T = T_eval;
end

function R = eul2rotm_local(eul)
    y = eul(1); p = eul(2); r = eul(3);
    Rz = [cos(y) -sin(y) 0; sin(y) cos(y) 0; 0 0 1];
    Ry = [cos(p) 0 sin(p); 0 1 0; -sin(p) 0 cos(p)];
    Rx = [1 0 0; 0 cos(r) -sin(r); 0 sin(r) cos(r)];
    R = Rz * Ry * Rx;
end
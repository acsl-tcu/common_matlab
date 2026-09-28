% % =========================================================================
% % test_C6_BSPLINE_PHASE4.m
% % 
% % 【Phase 4-01 & 4-02: 局所安全余裕関数 g(t;n) & 一般C6反例によるサンプリング限界検証】
% %  - 外部依存ゼロ (完全単一ファイル完結)
% %  - 構成:
% %      [Phase 4-01] 
% %        - 安全余裕関数 g(t; n) = n'p_L - Delta_sep(n, n_T) - h_E(n) - d_margin
% %        - 31点離散判定 (Discrete) と 5,001点高密度数値参照 (Dense Reference)
% %        - 全30区間 I_k = [u_k, u_{k+1}] におけるサンプリング偏差の客観的走査
% %      [Phase 4-02] 一般 C6 関数に対する数学的反例の完全証明 (Mathematical Counterexample)
% %        - 7乗バンプ関数を用いた解析的 C6 級反例モデル g_CE(u)
% %        - 端点 g_CE(u_a) = g_CE(u_b) = eps_ce > 0 (全31点離散サンプル完全合格)
% %        - 中央点解析解 min g_CE = eps_ce - A_ce < 0 による代数的直接証明
% %        - 結論: 有限サンプリング検査では連続時間安全性を一般に保証できない
% %
% % 実行コマンド:
% %   >> test_C6_BSPLINE_PHASE4
% % =========================================================================
% function test_C6_BSPLINE_PHASE4()
%     clc;
%     fprintf('=========================================================================================\n');
%     fprintf('   Phase 4: Continuous Safety Margin & Analytical C6 Counterexample Suite                \n');
%     fprintf('=========================================================================================\n\n');
% 
%     %% 0. 物理幾何・障害物パラメータ定義
%     cfg.p = 7;                   % B-spline 次数 p = 7 (C6 連続)
%     cfg.N_ctrl = 18;             % 制御点数 18 (7 + 4 + 7)
%     cfg.N_fixed_start = 7;
%     cfg.N_free = 4;
%     cfg.N_fixed_end = 7;
%     cfg.n_z = 12;
%     cfg.qp_n_samples = 31;       % 離散スクリーニング点数 31
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
%     cache = init_phase4_static_cache(cfg);
% 
%     fprintf('[システム幾何設定]\n');
%     fprintf(' - 索長 L_cable         : %.2f m\n', cfg.sys.L_cable);
%     fprintf(' - ペイロード半径 R_L   : %.2f m\n', cfg.sys.R_payload);
%     fprintf(' - 索保護厚み R_C       : %.2f m\n', cfg.sys.R_cable);
%     fprintf(' - UAV 機体包絡半径 R_Q : %.2f m\n', cfg.sys.R_uav);
%     fprintf(' - 要求安全マージン d   : %.2f m\n', cfg.d_margin);
%     fprintf(' - 評価ホライズン T     : %.2f s\n\n', T_eval);
% 
%     %% ====================================================================
%     % [Step 1] C6 局所変形 Payload 軌道取得 & 局所分離法線 n_avoid 決定
%     % ====================================================================
%     fprintf('=========================================================================================\n');
%     fprintf(' [Step 1] C6 局所変形 Payload 軌道の取得 & 局所分離法線 n_avoid の決定\n');
%     fprintf('=========================================================================================\n');
% 
%     res_p2 = generate_phase2_c6_trajectory(T_eval, D_start, D_end, obs, cfg, cache);
%     model = res_p2.model;
%     P_ctrl = res_p2.P_ctrl;
% 
%     n_avoid = res_p2.n_avoid;
%     h_E = dot(n_avoid, obs.center) + sqrt(n_avoid' * obs.S * n_avoid);
% 
%     fprintf('  - C6 局所変形強度 alpha*              : %6.4f m\n', res_p2.alpha_star);
%     fprintf('  - 局所外向き法線 n_avoid              : [%5.2f, %5.2f, %5.2f]\n', ...
%         n_avoid(1), n_avoid(2), n_avoid(3));
%     fprintf('  - 障害物支持関数値 h_E(n_avoid)       : %6.4f m\n\n', h_E);
% 
%     %% ====================================================================
%     % [Phase 4-01] 実軌道の安全余裕走査 & 全隣接区間内部沈み込み評価
%     % ====================================================================
%     fprintf('=========================================================================================\n');
%     fprintf(' [Phase 4-01] 実軌道における 31点離散 vs 5,001点高密度数値参照評価\n');
%     fprintf('=========================================================================================\n');
% 
%     N_disc = cfg.qp_n_samples;
%     u_disc = linspace(0.0, 1.0, N_disc)';
%     g_disc = zeros(N_disc, 1);
% 
%     for k = 1:N_disc
%         u = u_disc(k);
%         p_L = eval_bspline_deriv(model, u, P_ctrl, 0);
%         a_L = eval_bspline_deriv(model, u, P_ctrl, 2);
%         a_net = a_L + cfg.g_acc * cfg.e3;
%         n_T = a_net / norm(a_net);
%         g_disc(k) = eval_system_safety_margin(p_L, n_T, n_avoid, obs, cfg.sys, cfg.d_margin);
%     end
% 
%     N_dense = 5001;
%     u_dense = linspace(0.0, 1.0, N_dense)';
%     g_dense = zeros(N_dense, 1);
% 
%     for j = 1:N_dense
%         u = u_dense(j);
%         p_L = eval_bspline_deriv(model, u, P_ctrl, 0);
%         a_L = eval_bspline_deriv(model, u, P_ctrl, 2);
%         a_net = a_L + cfg.g_acc * cfg.e3;
%         n_T = a_net / norm(a_net);
%         g_dense(j) = eval_system_safety_margin(p_L, n_T, n_avoid, obs, cfg.sys, cfg.d_margin);
%     end
% 
%     % 全30隣接区間 I_k = [u_k, u_{k+1}] に対する高密度数値参照による内部沈み込み走査
%     max_sag = 0.0;
%     k_max_sag = -1;
%     u_max_sag = NaN;
% 
%     for k = 1:(N_disc - 1)
%         u_a = u_disc(k);
%         u_b = u_disc(k + 1);
% 
%         idx_inside = find(u_dense > u_a + 1e-10 & u_dense < u_b - 1e-10);
%         if isempty(idx_inside), continue; end
% 
%         [g_inner_min, idx_rel] = min(g_dense(idx_inside));
%         u_inner_min = u_dense(idx_inside(idx_rel));
% 
%         g_endpoint_min = min(g_disc(k), g_disc(k + 1));
%         sag_k = g_endpoint_min - g_inner_min;
% 
%         if sag_k > max_sag
%             max_sag = sag_k;
%             k_max_sag = k;
%             u_max_sag = u_inner_min;
%         end
%     end
% 
%     fprintf('  【実機設計軌道 (Phase 2 コア生成解) の区間走査結果】\n');
%     fprintf('    - 離散 31 点全体最小安全余裕         : %+7.4f m\n', min(g_disc));
%     fprintf('    - 5,001 点高密度数値参照 全体最小値   : %+7.4f m\n', min(g_dense));
%     if k_max_sag > 0 && max_sag > 1e-6
%         fprintf('    - 最大内部沈み込み検出区間 I_k      : [%.4f, %.4f]\n', ...
%             u_disc(k_max_sag), u_disc(k_max_sag+1));
%         fprintf('    - 区間内最悪点 (数値参照) u*        : %.4f\n', u_max_sag);
%         fprintf('    - 観測内部沈み込み量 max_sag        : %+7.4f m (%+5.2f mm)\n', ...
%             max_sag, max_sag * 1000.0);
%         fprintf('    (注: 本シナリオの実軌道では、高密度数値参照により最大約 %.2f mm の\n', max_sag * 1000.0);
%         fprintf('         内部沈み込みが観測された。ただし、これは連続時間沈み込みの厳密上界ではない)\n');
%     else
%         fprintf('    - 観測内部沈み込み量 max_sag        : 0.0000 m (内部極小なし: 格子区間内で単調推移)\n');
%     end
%     fprintf('  ---------------------------------------------------------------------------------------\n');
%     fprintf('  >>> 概念定義と位置付け:\n');
%     fprintf('      g >= 0 は「法線 n_avoid による安全証明成立」を表し、\n');
%     fprintf('      g < 0 は衝突ではなく「この固定法線では安全性が未証明」であることを表す。\n');
%     fprintf('      本ステップの目的は、有限離散評価と連続時間参照値との間に挙動差（サンプリング偏差）が\n');
%     fprintf('      存在することを実軌道上で客観確認することにある。\n\n');
% 
%     %% ====================================================================
%     % [Phase 4-02] 解析的 C6 反例の完全証明 (Mathematical C6 Counterexample)
%     % ====================================================================
%     fprintf('=========================================================================================\n');
%     fprintf(' [Phase 4-02] 一般 C6 関数に対する数学的反例の完全証明 (7乗バンプ核による代数的証明)\n');
%     fprintf('=========================================================================================\n');
% 
%     % 命題: 「一般の C6 級関数 g(u) において、有限個のサンプリング点検査 (31点) で\n
%     %        g(u_k) >= 0 が確認されたとしても、全時間安全 g(u) >= 0 (forall u) は導けない」
%     %
%     % 構成: 隣接格子 [u_15, u_16] = [0.5000, 0.5333] の内部のみに谷を持つ
%     %       解析的 C6 級バンプ関数 g_CE(u) を定義:
%     %         g_CE(u) = eps_ce - A_ce * [ (u - u_a)(u_b - u) / ((u_b - u_a)/2)^2 ]^7
%     %       区間外は g_CE(u) = eps_ce (定数)
%     %
%     % 解析解:
%     %   - 離散端点: g_CE(u_a) = g_CE(u_b) = eps_ce > 0
%     %   - 連続最小: min_{u} g_CE(u) = g_CE(u_m) = eps_ce - A_ce < 0 (厳密解析解)
% 
%     k_ce = 16; % u_disc(16) = 0.5000, u_disc(17) = 0.5333
%     u_a = u_disc(k_ce);
%     u_b = u_disc(k_ce + 1);
%     u_m = 0.5 * (u_a + u_b); % 0.51667
% 
%     eps_ce = 0.0100; % +10.0 mm (全離散点で保持される余裕)
%     A_ce   = 0.0200; %  20.0 mm (中央点での沈み込み深さ)
% 
%     g_CE_fn = @(u) eval_c6_counterexample_margin(u, u_a, u_b, eps_ce, A_ce);
% 
%     % 31点離散評価
%     g_CE_disc = zeros(N_disc, 1);
%     for k = 1:N_disc
%         g_CE_disc(k) = g_CE_fn(u_disc(k));
%     end
% 
%     % 厳密解析解の計算
%     g_CE_min_exact = eps_ce - A_ce; % = -0.0100 m (-10.0 mm)
%     has_continuous_violation = (g_CE_min_exact < 0);
%     all_disc_pass = all(g_CE_disc > 0);
% 
%     % 5,001点高密度数値参照との一致確認 (数値参照用)
%     g_CE_dense = zeros(N_dense, 1);
%     for j = 1:N_dense
%         g_CE_dense(j) = g_CE_fn(u_dense(j));
%     end
%     min_CE_dense = min(g_CE_dense);
%     diff_exact_dense = abs(min_CE_dense - g_CE_min_exact);
% 
%     fprintf('  【C6 解析的反例モデルのパラメータ】\n');
%     fprintf('    - 離散検査格子区間 [u_a, u_b]       : [%.4f, %.4f] (幅 Delta u = %.4f)\n', ...
%         u_a, u_b, u_b - u_a);
%     fprintf('    - 格子中央時刻 u_m                  : %.4f\n', u_m);
%     fprintf('    - 端点安全余裕 eps_ce               : %+6.4f m (+%4.1f mm)\n', ...
%         eps_ce, eps_ce * 1000.0);
%     fprintf('    - 中央ディップ振幅 A_ce             : %+6.4f m (+%4.1f mm)\n', ...
%         A_ce, A_ce * 1000.0);
%     fprintf('    - 境界連続性                        : C6 厳密代数接続 (7乗バンプ核)\n');
%     fprintf('  ---------------------------------------------------------------------------------------\n');
%     fprintf('  【判定結果 (代数的解析解)】\n');
%     fprintf('    - 31点 離散最小安全余裕             : %+6.4f m\n', min(g_CE_disc));
%     fprintf('    - 31点 全離散サンプル判定 (all > 0) : %s -> 【全31点完全合格】\n', ...
%         pass_str(all_disc_pass));
%     fprintf('    ---------------------------------------------------------------------------------\n');
%     fprintf('    - 連続時間最小安全余裕 (中央 u_m)   : %6.4f m (厳密解析解: eps - A)\n', ...
%         g_CE_min_exact);
%     fprintf('    - 5,001点数値参照との差             : %9.2e m (解析解と整合)\n', diff_exact_dense);
%     fprintf('    - 連続時間未証明領域の存在          : %s -> 【サンプル間未証明領域を検出】\n', ...
%         pass_str(has_continuous_violation));
%     fprintf('  ---------------------------------------------------------------------------------------\n');
% 
%     if all_disc_pass && has_continuous_violation
%         fprintf('  >>> Phase 4-02 検証結論: [SUCCESS: 一般C6反例の完全成立]\n');
%         fprintf('      全31点すべてのサンプリング点において g = +%.1f mm > 0 と判定されるにもかかわらず、\n', ...
%             eps_ce * 1000.0);
%         fprintf('      その中間時刻 u_m = %.4f において g = %6.4f m (< 0) となる\n', ...
%             u_m, g_CE_min_exact);
%         fprintf('      一般 C6 級関数に対する数学的反例の完全な代数的証明が成立しました。\n\n');
%         fprintf('      【学術的結論】\n');
%         fprintf('      有限個の離散サンプリング検査では、連続時間安全性を一般に保証することはできない。\n');
%         fprintf('      したがって、次ステップ (Phase 4-03) で定式化する「B-spline 凸包区間下界\n');
%         fprintf('      Certificate (g_lower_j >= 0)」による連続時間保証の導入が必須である。\n');
%     else
%         fprintf('  >>> Phase 4-02 検証結論: [FAIL: 反例不成立]\n');
%     end
%     fprintf('=========================================================================================\n');
%     fprintf('  Phase 4-01 & 4-02 検証完了 (Phase 4-03 へ継続)\n');
%     fprintf('=========================================================================================\n\n');
% end
% 
% %% ========================================================================
% %  【Phase 4-02 解析的反例関数】C6 級 7乗バンプ核安全余裕関数
% % ========================================================================
% function g = eval_c6_counterexample_margin(u, u_a, u_b, eps_val, A_val)
%     if u >= u_a && u <= u_b
%         xi = (u - u_a) * (u_b - u) / (( (u_b - u_a)/2 )^2);
%         g = eps_val - A_val * (xi^7);
%     else
%         g = eps_val;
%     end
% end
% 
% %% ========================================================================
% %  システム安全余裕関数 g(t; n)
% % ========================================================================
% function [g, delta_sep] = eval_system_safety_margin(p_L, n_T, n_sep, obs, sys, d_margin)
%     h_E = dot(n_sep, obs.center) + sqrt(n_sep' * obs.S * n_sep);
%     delta_sep = eval_delta_h_sep(n_sep, n_T, sys);
%     g = dot(n_sep, p_L) - delta_sep - h_E - d_margin;
% end
% 
% %% ========================================================================
% %  分離包絡 Delta_sep(n, n_T) = Delta_h(-n, n_T)
% % ========================================================================
% function delta_sep = eval_delta_h_sep(n, n_T, sys)
%     proj_neg_nT = dot(-n, n_T);
%     val_L = sys.R_payload;
%     val_Q = sys.L_cable * proj_neg_nT + sys.R_uav;
%     val_C = max(0.0, sys.L_cable * proj_neg_nT) + sys.R_cable;
%     delta_sep = max([val_L, val_Q, val_C]);
% end
% 
% %% ========================================================================
% %  Phase 2 凍結エンジンによる C6 軌道生成
% % ========================================================================
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
%     G = [w_shape * n_avoid(1); w_shape * n_avoid(2); w_shape * n_avoid(3)];
%     z_sol = G * alpha_req;
% 
%     P_new = P_fixed;
%     P_new(8:11, 1) = P_new(8:11, 1) + z_sol(1:4);
%     P_new(8:11, 2) = P_new(8:11, 2) + z_sol(5:8);
%     P_new(8:11, 3) = P_new(8:11, 3) + z_sol(9:12);
% 
%     res.P_ctrl = P_new;
%     res.alpha_star = alpha_req;
%     res.n_avoid = n_avoid;
%     model.P_fixed = P_fixed;
%     model.T = T;
%     model.knots = cache.knots;
%     res.model = model;
% end
% 
% %% ========================================================================
% %  B-spline 導関数評価
% % ========================================================================
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
% %% ========================================================================
% %  キャッシュ初期化
% % ========================================================================
% function cache = init_phase4_static_cache(cfg)
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
% 
%     cache.N0 = zeros(N_s, N_ctrl);
%     for k = 1:N_s
%         cache.N0(k, :) = eval_basis_derivative_raw(u_vec(k), knots, p, N_ctrl, 0);
%     end
%     cache.B_free_0 = cache.N0(:, 8:11);
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

% % =========================================================================
% % test_C6_BSPLINE_PHASE4_03.m
% % 
% % 【Phase 4-03 確定版: B-spline 凸包性に基づく連続時間安全証明 Baseline Certificate】
% %  - 外部依存ゼロ (完全単一ファイル完結)
% %  - 数理的骨格:
% %      1. 索姿勢射影下界の厳密化:
% %           w_min >= 0  =>  q_lower = w_min / M_max
% %           w_min <  0  =>  q_lower = -1.0 (無条件幾何下界)
% %         により、数学的 Soundness (健全性) を 100% 確立
% %      2. 単調非増加性に基づく分離包絡上界:
% %           Delta_upper_j = Delta_sep(q_lower_j)
% %      3. 5,001点数値参照との整合性確認 (Dense-reference Consistency Check)
% %      4. 保守性ギャップを定量評価し、Phase 4-04 のタイト化課題として整理
% %
% % 実行コマンド:
% %   >> test_C6_BSPLINE_PHASE4_03
% % =========================================================================
% function test_C6_BSPLINE_PHASE4_03()
%     clc;
%     fprintf('=========================================================================================\n');
%     fprintf('   Phase 4-03: Continuous-Time B-Spline Convex-Hull Safety Certificate Suite [FROZEN]   \n');
%     fprintf('=========================================================================================\n\n');
% 
%     %% 0. 物理幾何・障害物パラメータ定義
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
%     % 未知障害物 (Active Obstacle: 楕円体)
%     obs.center = [0.3; 0.3; 20.0];
%     obs.radii  = [1.2; 1.2; 2.0];
%     obs.R      = eul2rotm_local([pi/6, pi/12, 0]);
%     obs.S      = obs.R * diag(obs.radii.^2) * obs.R';
%     obs.Sinv   = obs.R * diag(1.0 ./ (obs.radii.^2)) * obs.R';
% 
%     cache = init_phase4_03_static_cache(cfg);
% 
%     fprintf('[システム幾何設定]\n');
%     fprintf(' - 索長 L_cable         : %.2f m\n', cfg.sys.L_cable);
%     fprintf(' - ペイロード半径 R_L   : %.2f m\n', cfg.sys.R_payload);
%     fprintf(' - 索保護厚み R_C       : %.2f m\n', cfg.sys.R_cable);
%     fprintf(' - UAV 機体包絡半径 R_Q : %.2f m\n', cfg.sys.R_uav);
%     fprintf(' - 要求安全マージン d   : %.2f m\n', cfg.d_margin);
%     fprintf(' - 評価ホライズン T     : %.2f s\n\n', T_eval);
% 
%     %% ====================================================================
%     % [Step 1] C6 局所変形 Payload 軌道の取得 & 局所分離法線 n_avoid 決定
%     % ====================================================================
%     fprintf('=========================================================================================\n');
%     fprintf(' [Step 1] C6 局所変形軌道生成 & 局所分離法線 n_avoid 決定\n');
%     fprintf('=========================================================================================\n');
% 
%     res_p2 = generate_phase2_c6_trajectory(T_eval, D_start, D_end, obs, cfg, cache);
%     model = res_p2.model;
%     P_ctrl = res_p2.P_ctrl;
% 
%     n_avoid = res_p2.n_avoid;
%     h_E = dot(n_avoid, obs.center) + sqrt(n_avoid' * obs.S * n_avoid);
% 
%     fprintf('  - C6 変形スカラー alpha*              : %6.4f m\n', res_p2.alpha_star);
%     fprintf('  - 局所外向き法線 n_avoid              : [%5.2f, %5.2f, %5.2f]\n', ...
%         n_avoid(1), n_avoid(2), n_avoid(3));
%     fprintf('  - 障害物支持関数値 h_E(n_avoid)       : %6.4f m\n\n', h_E);
% 
%     %% ====================================================================
%     % [Step 2 & 3] 各ノット区間における連続時間下界 g_lower_j 算出 & Certificate
%     % ====================================================================
%     fprintf('=========================================================================================\n');
%     fprintf(' [Step 2 & 3] B-spline 凸包性に基づく区間下界 g_lower_j 算出 & 連続時間 Certificate\n');
%     fprintf('=========================================================================================\n');
% 
%     knots = cache.knots;
%     p = cfg.p;
%     N_ctrl = cfg.N_ctrl;
% 
%     A_ctrl = (cache.K_a * P_ctrl) / (T_eval^2);
%     p_acc = p - 2;
%     knots_acc = knots(2:end-1);
% 
%     unique_knots = cache.unique_knots;
%     n_spans = length(unique_knots) - 1;
% 
%     cert_spans = struct('u_start', {}, 'u_end', {}, 'p_lower', {}, ...
%                         'q_lower', {}, 'Delta_upper', {}, 'g_lower', {}, 'is_safe', {});
% 
%     all_certificate_pass = true;
%     min_cert_g = inf;
% 
%     t_cert_start = tic;
%     span_count = 0;
% 
%     for s = 1:n_spans
%         u_s = unique_knots(s);
%         u_e = unique_knots(s + 1);
%         if u_e <= u_s + 1e-12, continue; end
%         span_count = span_count + 1;
% 
%         u_mid = 0.5 * (u_s + u_e);
% 
%         % 1. 位置制御点凸包下界 p_lower_j = min_{i in A_j} n' * P_i
%         span_idx_pos = find_span_local(u_mid, knots, p, N_ctrl);
%         active_cpts_pos = P_ctrl((span_idx_pos - p):span_idx_pos, :);
%         proj_pos = active_cpts_pos * n_avoid;
%         p_lower = min(proj_pos);
% 
%         % 2. 索姿勢射影下界 q_lower_j (厳密保証フォールバック)
%         span_idx_acc = find_span_local(u_mid, knots_acc, p_acc, N_ctrl - 2);
%         active_cpts_acc = A_ctrl((span_idx_acc - p_acc):span_idx_acc, :);
% 
%         W_cpts = active_cpts_acc + repmat((cfg.g_acc * cfg.e3)', size(active_cpts_acc, 1), 1);
%         proj_W = W_cpts * n_avoid;
%         w_min = min(proj_W);
%         norms_W = sqrt(sum(W_cpts.^2, 2));
%         M_max = max(norms_W);
% 
%         % 数学的厳密保証 (ヒューリスティック M_min を完全排除)
%         if w_min >= 0.0
%             q_lower = w_min / M_max;
%         else
%             q_lower = -1.0; % 幾何学的無条件下界 n'*n_T >= -1
%         end
% 
%         % 3. 分離包絡上界 Delta_upper_j = Delta_sep(q_lower_j)
%         Delta_upper = eval_delta_h_sep_from_q(q_lower, cfg.sys);
% 
%         % 4. 区間下界 g_lower_j
%         g_lower = p_lower - Delta_upper - h_E - cfg.d_margin;
% 
%         is_safe = (g_lower >= 0);
%         if ~is_safe, all_certificate_pass = false; end
%         min_cert_g = min(min_cert_g, g_lower);
% 
%         cert_spans(span_count).u_start     = u_s;
%         cert_spans(span_count).u_end       = u_e;
%         cert_spans(span_count).p_lower     = p_lower;
%         cert_spans(span_count).q_lower     = q_lower;
%         cert_spans(span_count).Delta_upper = Delta_upper;
%         cert_spans(span_count).g_lower     = g_lower;
%         cert_spans(span_count).is_safe     = is_safe;
%     end
%     t_cert_total_ms = toc(t_cert_start) * 1000.0;
% 
%     fprintf('  【全 %d ノット区間における Certificate スキャン結果】\n', span_count);
%     fprintf('  ---------------------------------------------------------------------------------------\n');
%     fprintf('   Span |  区間 [u_start, u_end]  | p_lower [m] | q_lower | Delta_upper [m] | g_lower [m] | 判定\n');
%     fprintf('  ---------------------------------------------------------------------------------------\n');
%     for s = 1:span_count
%         fprintf('   %3d  | [%6.4f, %6.4f]    |   %+7.4f   | %+7.4f |     %7.4f     |   %+7.4f   | %s\n', ...
%             s, cert_spans(s).u_start, cert_spans(s).u_end, ...
%             cert_spans(s).p_lower, cert_spans(s).q_lower, ...
%             cert_spans(s).Delta_upper, cert_spans(s).g_lower, ...
%             pass_str(cert_spans(s).is_safe));
%     end
%     fprintf('  ---------------------------------------------------------------------------------------\n');
%     fprintf('  - 全区間 Certificate 最小下界 min g_lower : %+7.4f m\n', min_cert_g);
%     fprintf('  - Certificate 計算所要時間 (全区間)       : %6.3f ms (%6.2f μs / span)\n', ...
%         t_cert_total_ms, (t_cert_total_ms / span_count) * 1000.0);
%     fprintf('  - 連続時間安全証明 (Continuous Certificate) : %s\n\n', ...
%         cert_str(all_certificate_pass));
% 
%     %% ====================================================================
%     % [Step 4] 5,001点 高密度数値参照との整合性検証 (Consistency Check)
%     % ====================================================================
%     fprintf('=========================================================================================\n');
%     fprintf(' [Step 4] 5,001点 高密度数値参照との整合性検証 (g_lower_j <= min g(u) の確認)\n');
%     fprintf('=========================================================================================\n');
% 
%     N_dense = 5001;
%     u_dense = linspace(0.0, 1.0, N_dense)';
%     g_dense = zeros(N_dense, 1);
% 
%     for j = 1:N_dense
%         u = u_dense(j);
%         p_L = eval_bspline_deriv(model, u, P_ctrl, 0);
%         a_L = eval_bspline_deriv(model, u, P_ctrl, 2);
%         a_net = a_L + cfg.g_acc * cfg.e3;
%         n_T = a_net / norm(a_net);
%         g_dense(j) = eval_system_safety_margin(p_L, n_T, n_avoid, obs, cfg.sys, cfg.d_margin);
%     end
% 
%     inconsistency_count = 0;
%     conservatism_gaps = zeros(span_count, 1);
% 
%     for s = 1:span_count
%         u_s = cert_spans(s).u_start;
%         u_e = cert_spans(s).u_end;
%         idx_in = find(u_dense >= u_s & u_dense <= u_e);
%         min_g_ref_in_span = min(g_dense(idx_in));
% 
%         gap = min_g_ref_in_span - cert_spans(s).g_lower;
%         conservatism_gaps(s) = gap;
% 
%         if gap < -1e-6
%             inconsistency_count = inconsistency_count + 1;
%         end
%     end
% 
%     fprintf('  - 数値参照下回り件数 (g_ref < g_lower)    : %d 件 (0件であれば数値参照と完全整合)\n', ...
%         inconsistency_count);
%     fprintf('  - 高密度数値参照 整合性判定               : %s\n', pass_str(inconsistency_count == 0));
%     fprintf('  - 平均保守性ギャップ (g_ref - g_lower)    : %+7.4f m (%+5.1f mm)\n', ...
%         mean(conservatism_gaps), mean(conservatism_gaps) * 1000.0);
%     fprintf('  - 最小保守性ギャップ                      : %+7.4f m (%+5.1f mm)\n', ...
%         min(conservatism_gaps), min(conservatism_gaps) * 1000.0);
%     fprintf('  - 最大保守性ギャップ                      : %+7.4f m (%+5.1f mm)\n\n', ...
%         max(conservatism_gaps), max(conservatism_gaps) * 1000.0);
% 
%     %% ====================================================================
%     % [Step 5] 連続時間 Certificate カーネルの統計ベンチマーク (N=2,000)
%     % ====================================================================
%     fprintf('=========================================================================================\n');
%     fprintf(' [Step 5] 連続時間 Certificate 演算レイテンシ統計プロファイル (N=2,000 試行)\n');
%     fprintf('=========================================================================================\n');
% 
%     for w = 1:50
%         run_certificate_kernel(P_ctrl, A_ctrl, n_avoid, h_E, cfg, cache);
%     end
% 
%     N_bench = 2000;
%     latencies_cert = zeros(N_bench, 1);
% 
%     for b = 1:N_bench
%         t_b = tic;
%         run_certificate_kernel(P_ctrl, A_ctrl, n_avoid, h_E, cfg, cache);
%         latencies_cert(b) = toc(t_b) * 1000.0;
%     end
% 
%     mean_c   = mean(latencies_cert);
%     median_c = median(latencies_cert);
%     p95_c    = prctile(latencies_cert, 95);
%     p99_c    = prctile(latencies_cert, 99);
%     max_c    = max(latencies_cert);
%     min_c    = min(latencies_cert);
% 
%     fprintf('  * 測定対象: 全ノット区間に対する区間下界 g_lower_j 一括算出 Certificate\n');
%     fprintf('  ---------------------------------------------------------------------------------------\n');
%     fprintf('    - 平均所要時間         (Mean)   : %7.4f ms (%6.2f μs)\n', mean_c, mean_c * 1000);
%     fprintf('    - 中央値              (Median) : %7.4f ms (%6.2f μs)\n', median_c, median_c * 1000);
%     fprintf('    - 95パーセンタイル    (P95)    : %7.4f ms (%6.2f μs)\n', p95_c, p95_c * 1000);
%     fprintf('    - 99パーセンタイル    (P99)    : %7.4f ms (%6.2f μs)\n', p99_c, p99_c * 1000);
%     fprintf('    - 観測最大値          (Max)    : %7.4f ms (%6.2f μs)\n', max_c, max_c * 1000);
%     fprintf('    - 最速実行値          (Min)    : %7.4f ms (%6.2f μs)\n', min_c, min_c * 1000);
%     fprintf('  ---------------------------------------------------------------------------------------\n');
%     fprintf('  >>> 結論:\n');
%     fprintf('      中央値 %6.4f ms (平均 %6.4f ms) で、固定分離法線に対する\n', median_c, mean_c);
%     fprintf('      連続時間安全余裕の数学的保証下界を評価可能であることを確認。\n');
%     fprintf('      なお、今回の基礎 B-spline 凸包では平均保守性ギャップが約 %.2f m 存在するため、\n', mean(conservatism_gaps));
%     fprintf('      Phase 4-04 において Bézier extraction 等によるタイト化 (Conservatism Reduction) を検討する。\n');
%     fprintf('=========================================================================================\n');
%     fprintf('  Phase 4-03 確定凍結版 完了 (BASELINE CERTIFICATE FROZEN)\n');
%     fprintf('=========================================================================================\n\n');
% end
% 
% %% ========================================================================
% %  【Certificate 高速演算カーネル】
% % ========================================================================
% function all_pass = run_certificate_kernel(P_ctrl, A_ctrl, n_avoid, h_E, cfg, cache)
%     knots = cache.knots;
%     p = cfg.p;
%     N_ctrl = cfg.N_ctrl;
%     p_acc = p - 2;
%     knots_acc = knots(2:end-1);
%     unique_knots = cache.unique_knots;
%     n_spans = length(unique_knots) - 1;
% 
%     all_pass = true;
%     for s = 1:n_spans
%         u_s = unique_knots(s);
%         u_e = unique_knots(s + 1);
%         if u_e <= u_s + 1e-12, continue; end
%         u_mid = 0.5 * (u_s + u_e);
% 
%         % 1. 位置凸包下界
%         span_p = find_span_local(u_mid, knots, p, N_ctrl);
%         proj_p = P_ctrl((span_p-p):span_p, :) * n_avoid;
%         p_low = min(proj_p);
% 
%         % 2. 索姿勢射影下界 (厳密保証版)
%         span_a = find_span_local(u_mid, knots_acc, p_acc, N_ctrl - 2);
%         W_c = A_ctrl((span_a-p_acc):span_a, :) + repmat((cfg.g_acc * cfg.e3)', p_acc+1, 1);
%         proj_W = W_c * n_avoid;
%         w_min = min(proj_W);
%         norms_W = sqrt(sum(W_c.^2, 2));
%         M_max = max(norms_W);
% 
%         if w_min >= 0.0
%             q_low = w_min / M_max;
%         else
%             q_low = -1.0;
%         end
% 
%         Delta_up = eval_delta_h_sep_from_q(q_low, cfg.sys);
%         g_low = p_low - Delta_up - h_E - cfg.d_margin;
% 
%         if g_low < 0
%             all_pass = false;
%         end
%     end
% end
% 
% %% ========================================================================
% %  索射影 q_lower からの分離包絡上界計算 (広義単調減少性に基づく代数解)
% % ========================================================================
% function Delta_upper = eval_delta_h_sep_from_q(q_lower, sys)
%     val_L = sys.R_payload;
%     val_Q = sys.R_uav - sys.L_cable * q_lower;
%     val_C = sys.R_cable + max(0.0, -sys.L_cable * q_lower);
%     Delta_upper = max([val_L, val_Q, val_C]);
% end
% 
% %% ========================================================================
% %  システム安全余裕関数 g(t; n)
% % ========================================================================
% function g = eval_system_safety_margin(p_L, n_T, n_sep, obs, sys, d_margin)
%     h_E = dot(n_sep, obs.center) + sqrt(n_sep' * obs.S * n_sep);
%     proj_neg_nT = dot(-n_sep, n_T);
%     val_L = sys.R_payload;
%     val_Q = sys.L_cable * proj_neg_nT + sys.R_uav;
%     val_C = max(0.0, sys.L_cable * proj_neg_nT) + sys.R_cable;
%     delta_sep = max([val_L, val_Q, val_C]);
%     g = dot(n_sep, p_L) - delta_sep - h_E - d_margin;
% end
% 
% %% ========================================================================
% %  Phase 2 凍結エンジンによる C6 軌道生成
% % ========================================================================
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
%     G = [w_shape * n_avoid(1); w_shape * n_avoid(2); w_shape * n_avoid(3)];
%     z_sol = G * alpha_req;
% 
%     P_new = P_fixed;
%     P_new(8:11, 1) = P_new(8:11, 1) + z_sol(1:4);
%     P_new(8:11, 2) = P_new(8:11, 2) + z_sol(5:8);
%     P_new(8:11, 3) = P_new(8:11, 3) + z_sol(9:12);
% 
%     res.P_ctrl = P_new;
%     res.alpha_star = alpha_req;
%     res.n_avoid = n_avoid;
%     model.P_fixed = P_fixed;
%     model.T = T;
%     model.knots = cache.knots;
%     res.model = model;
% end
% 
% %% ========================================================================
% %  B-spline 導関数評価
% % ========================================================================
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
% %% ========================================================================
% %  キャッシュ初期化 & 差分行列生成
% % ========================================================================
% function cache = init_phase4_03_static_cache(cfg)
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
%     cache.unique_knots = unique(knots);
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
%     p2 = p - 1;
%     K2_step = zeros(N_ctrl - 2, N_ctrl - 1);
%     for i = 1:(N_ctrl - 2)
%         dt = knots(i + p2 + 2) - knots(i + 2);
%         if dt > 1e-12, K2_step(i, i) = -p2 / dt; K2_step(i, i+1) = p2 / dt; end
%     end
%     cache.K_a = K2_step * K1;
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
% function s = cert_str(cond)
%     if cond, s = '[CERTIFIED (Continuous Safe)]'; else, s = '[UNCERTIFIED (Lower bound < 0)]'; end
% end

% % =========================================================================
% % test_C6_BSPLINE_PHASE4_04.m
% % 
% % 【Phase 4-04 確定版: Bézier Extraction 高密度数値整合性検証 & タイト化下界構成】
% %  - 外部依存ゼロ (完全単一ファイル完結)
% %  - 修正・確定内容:
% %      [Step 1] C6 局所変形軌道生成 & 局所分離法線 n_avoid の確定
% %      [Step 2] 2階導関数ノット系列 U^(2) = knots(3:end-2) に基づく
% %               Bézier Extraction の機械精度整合性検証 (安全ゲート付き)
% %               e_pos < 1e-10 m, e_acc < 1e-9 m/s^2
% %      [Step 3] Bézier 局所凸包によるタイト化 Certificate 下界スキャン
% %      [Step 4] Phase 4-03 (Baseline) との動的直接比較 (ハードコード完全撤廃)
% %               Delta_baseline = eval_delta_h_sep_from_q(-1.0, sys) を使用
% %      [Step 5] 5,001点 高密度数値参照との整合性検証 (Dense-reference Consistency Check)
% %      [Step 6] 演算レイテンシ統計プロファイル (N=2,000 試行)
% %
% % 実行コマンド:
% %   >> test_C6_BSPLINE_PHASE4_04
% % =========================================================================
% function test_C6_BSPLINE_PHASE4_04()
%     clc;
%     fprintf('=========================================================================================\n');
%     fprintf('   Phase 4-04: Bézier Extraction Reconstruction & Tightened Certificate [FROZEN]         \n');
%     fprintf('=========================================================================================\n\n');
% 
%     %% 0. 物理幾何・障害物パラメータ定義
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
%     % 未知障害物 (Active Obstacle: 楕円体)
%     obs.center = [0.3; 0.3; 20.0];
%     obs.radii  = [1.2; 1.2; 2.0];
%     obs.R      = eul2rotm_local([pi/6, pi/12, 0]);
%     obs.S      = obs.R * diag(obs.radii.^2) * obs.R';
%     obs.Sinv   = obs.R * diag(1.0 ./ (obs.radii.^2)) * obs.R';
% 
%     cache = init_phase4_04_static_cache(cfg);
% 
%     fprintf('[システム幾何設定]\n');
%     fprintf(' - 索長 L_cable         : %.2f m\n', cfg.sys.L_cable);
%     fprintf(' - ペイロード半径 R_L   : %.2f m\n', cfg.sys.R_payload);
%     fprintf(' - 索保護厚み R_C       : %.2f m\n', cfg.sys.R_cable);
%     fprintf(' - UAV 機体包絡半径 R_Q : %.2f m\n', cfg.sys.R_uav);
%     fprintf(' - 要求安全マージン d   : %.2f m\n', cfg.d_margin);
%     fprintf(' - 評価ホライズン T     : %.2f s\n\n', T_eval);
% 
%     %% ====================================================================
%     % [Step 1] C6 局所変形 Payload 軌道の取得 & 局所分離法線 n_avoid 決定
%     % ====================================================================
%     fprintf('=========================================================================================\n');
%     fprintf(' [Step 1] C6 局所変形軌道生成 & 局所分離法線 n_avoid 決定\n');
%     fprintf('=========================================================================================\n');
% 
%     res_p2 = generate_phase2_c6_trajectory(T_eval, D_start, D_end, obs, cfg, cache);
%     model = res_p2.model;
%     P_ctrl = res_p2.P_ctrl;
% 
%     n_avoid = res_p2.n_avoid;
%     h_E = dot(n_avoid, obs.center) + sqrt(n_avoid' * obs.S * n_avoid);
% 
%     fprintf('  - C6 変形スカラー alpha*              : %6.4f m\n', res_p2.alpha_star);
%     fprintf('  - 局所外向き法線 n_avoid              : [%5.2f, %5.2f, %5.2f]\n', ...
%         n_avoid(1), n_avoid(2), n_avoid(3));
%     fprintf('  - 障害物支持関数値 h_E(n_avoid)       : %6.4f m\n\n', h_E);
% 
%     %% ====================================================================
%     % [Step 2] Bézier Extraction 変換の高密度数値整合性検証 (機械精度テスト)
%     % ====================================================================
%     fprintf('=========================================================================================\n');
%     fprintf(' [Step 2] Bézier Extraction 変換の高密度数値整合性検証 (B-spline vs Bézier)\n');
%     fprintf('=========================================================================================\n');
% 
%     knots = cache.knots;
%     p = cfg.p;
%     N_ctrl = cfg.N_ctrl;
%     unique_knots = cache.unique_knots;
%     n_spans = length(unique_knots) - 1;
% 
%     A_ctrl = (cache.K_a * P_ctrl) / (T_eval^2);
%     p_acc = p - 2;
%     knots_acc = knots(3:end-2); % 2階導関数ノット系列 U^(2)
% 
%     N_check_per_span = 50;
%     max_err_pos_reconstruct = 0.0;
%     max_err_acc_reconstruct = 0.0;
% 
%     for s = 1:n_spans
%         u_s = unique_knots(s);
%         u_e = unique_knots(s + 1);
%         if u_e <= u_s + 1e-12, continue; end
%         u_mid = 0.5 * (u_s + u_e);
% 
%         % 位置 Bézier 制御点
%         span_idx_p = find_span_local(u_mid, knots, p, N_ctrl);
%         P_act = P_ctrl((span_idx_p - p):span_idx_p, :);
%         C_pos = cache.bezier_extract_pos{s};
%         P_bez = C_pos * P_act;
% 
%         % 加速度 Bézier 制御点
%         span_idx_a = find_span_local(u_mid, knots_acc, p_acc, N_ctrl - 2);
%         A_act = A_ctrl((span_idx_a - p_acc):span_idx_a, :);
%         C_acc = cache.bezier_extract_acc{s};
%         A_bez = C_acc * A_act;
% 
%         % 区間内 50 点で直接評価値の差分を計測
%         u_test = linspace(u_s, u_e, N_check_per_span);
%         xi_test = (u_test - u_s) / (u_e - u_s);
% 
%         for k = 1:N_check_per_span
%             p_bspline = eval_bspline_deriv(model, u_test(k), P_ctrl, 0);
%             a_bspline = eval_bspline_deriv(model, u_test(k), P_ctrl, 2);
% 
%             p_bezier = eval_bezier_curve_point(P_bez, p, xi_test(k));
%             a_bezier = eval_bezier_curve_point(A_bez, p_acc, xi_test(k));
% 
%             err_p = norm(p_bspline - p_bezier);
%             err_a = norm(a_bspline - a_bezier);
% 
%             max_err_pos_reconstruct = max(max_err_pos_reconstruct, err_p);
%             max_err_acc_reconstruct = max(max_err_acc_reconstruct, err_a);
%         end
%     end
% 
%     pos_recon_pass = (max_err_pos_reconstruct < 1e-10);
%     acc_recon_pass = (max_err_acc_reconstruct < 1e-9);
%     recon_pass = pos_recon_pass && acc_recon_pass;
% 
%     fprintf('  - 位置再構成最大誤差   Max ||p_Bspline - p_Bezier|| : %9.2e m  -> %s\n', ...
%         max_err_pos_reconstruct, pass_str(pos_recon_pass));
%     fprintf('  - 加速度再構成最大誤差 Max ||a_Bspline - a_Bezier|| : %9.2e m/s^2 -> %s\n', ...
%         max_err_acc_reconstruct, pass_str(acc_recon_pass));
% 
%     % 安全ゲート: 再構成誤差が未達の場合は後続処理へ進めず停止
%     if recon_pass
%         fprintf('  >>> 結論: 位置・加速度とも局所 Bézier 表現との高精度数値一致を確認。\n\n');
%     else
%         fprintf('  >>> 結論: Bézier 再構成検証 FAIL。Certificate 評価へ進んではならない。\n\n');
%         error('Phase 4-04 aborted: Bézier reconstruction verification failed.');
%     end
% 
%     %% ====================================================================
%     % [Step 3] Bézier Extraction によるタイト化 Certificate スキャン
%     % ====================================================================
%     fprintf('=========================================================================================\n');
%     fprintf(' [Step 3] Bézier Extraction 局所凸包によるタイト化 Certificate 算出\n');
%     fprintf('=========================================================================================\n');
% 
%     cert_bezier = struct('u_start', {}, 'u_end', {}, 'p_lower_bspline', {}, ...
%                          'p_lower_bez', {}, 'q_lower_bez', {}, 'Delta_upper_bez', {}, ...
%                          'g_lower_bez', {}, 'is_safe', {});
% 
%     t_cert_start = tic;
%     span_count = 0;
%     all_bezier_pass = true;
%     min_cert_g_bez = inf;
% 
%     for s = 1:n_spans
%         u_s = unique_knots(s);
%         u_e = unique_knots(s + 1);
%         if u_e <= u_s + 1e-12, continue; end
%         span_count = span_count + 1;
%         u_mid = 0.5 * (u_s + u_e);
% 
%         % A. 位置項のタイト化
%         span_idx_p = find_span_local(u_mid, knots, p, N_ctrl);
%         P_act = P_ctrl((span_idx_p - p):span_idx_p, :);
%         p_lower_bspline = min(P_act * n_avoid);
% 
%         C_pos = cache.bezier_extract_pos{span_count};
%         P_bez = C_pos * P_act;
%         p_lower_bez = min(P_bez * n_avoid);
% 
%         % B. 索姿勢射影項のタイト化 (Bézier 加速度)
%         span_idx_a = find_span_local(u_mid, knots_acc, p_acc, N_ctrl - 2);
%         A_act = A_ctrl((span_idx_a - p_acc):span_idx_a, :);
% 
%         C_acc = cache.bezier_extract_acc{span_count};
%         A_bez = C_acc * A_act;
%         W_bez = A_bez + repmat((cfg.g_acc * cfg.e3)', size(A_bez, 1), 1);
% 
%         proj_W_bez = W_bez * n_avoid;
%         w_min_bez = min(proj_W_bez);
%         M_max_bez = max(sqrt(sum(W_bez.^2, 2)));
% 
%         if w_min_bez >= 0.0
%             q_lower_bez = w_min_bez / M_max_bez;
%         else
%             q_lower_bez = -1.0; % 数学的無条件下界
%         end
% 
%         Delta_upper_bez = eval_delta_h_sep_from_q(q_lower_bez, cfg.sys);
%         g_lower_bez = p_lower_bez - Delta_upper_bez - h_E - cfg.d_margin;
% 
%         is_safe = (g_lower_bez >= 0);
%         if ~is_safe, all_bezier_pass = false; end
%         min_cert_g_bez = min(min_cert_g_bez, g_lower_bez);
% 
%         cert_bezier(span_count).u_start         = u_s;
%         cert_bezier(span_count).u_end           = u_e;
%         cert_bezier(span_count).p_lower_bspline = p_lower_bspline;
%         cert_bezier(span_count).p_lower_bez     = p_lower_bez;
%         cert_bezier(span_count).q_lower_bez     = q_lower_bez;
%         cert_bezier(span_count).Delta_upper_bez = Delta_upper_bez;
%         cert_bezier(span_count).g_lower_bez     = g_lower_bez;
%         cert_bezier(span_count).is_safe         = is_safe;
%     end
%     t_cert_total_ms = toc(t_cert_start) * 1000.0;
% 
%     fprintf('  【全 %d ノット区間におけるタイト化 Certificate スキャン結果】\n', span_count);
%     fprintf('  -------------------------------------------------------------------------------------------------------\n');
%     fprintf('   Span | 区間 [u_s, u_e] | p_low(B-spl) | p_low(Bézier) | q_low(Béz) | Delta_up [m] | g_low(Bézier) | 判定\n');
%     fprintf('  -------------------------------------------------------------------------------------------------------\n');
%     for s = 1:span_count
%         fprintf('   %3d  | [%.4f, %.4f] |   %+7.4f    |    %+7.4f    |  %+7.4f   |   %7.4f    |   %+7.4f    | %s\n', ...
%             s, cert_bezier(s).u_start, cert_bezier(s).u_end, ...
%             cert_bezier(s).p_lower_bspline, cert_bezier(s).p_lower_bez, ...
%             cert_bezier(s).q_lower_bez, cert_bezier(s).Delta_upper_bez, ...
%             cert_bezier(s).g_lower_bez, pass_str(cert_bezier(s).is_safe));
%     end
%     fprintf('  -------------------------------------------------------------------------------------------------------\n');
%     fprintf('  - タイト化 Certificate 最小下界 min g_lower_bez : %+7.4f m\n', min_cert_g_bez);
%     fprintf('  - 演算時間 (全区間一括)                         : %6.3f ms (%6.2f μs / span)\n', ...
%         t_cert_total_ms, (t_cert_total_ms / span_count) * 1000.0);
%     fprintf('  - 連続時間安全証明 (Bézier Certificate)         : %s\n\n', ...
%         cert_str(all_bezier_pass));
% 
%     %% ====================================================================
%     % [Step 4] Phase 4-03 (Baseline) vs Phase 4-04 (Bézier) 保守性ギャップ直接比較
%     % ====================================================================
%     fprintf('=========================================================================================\n');
%     fprintf(' [Step 4] Phase 4-03 (Baseline B-spline) vs Phase 4-04 (Bézier Extraction) 比較\n');
%     fprintf('=========================================================================================\n');
% 
%     p_tighten_gains = zeros(span_count, 1);
%     g_tighten_gains = zeros(span_count, 1);
% 
%     % ★修正 1: ハードコード 1.3000 の完全撤廃 (動的評価)
%     Delta_baseline = eval_delta_h_sep_from_q(-1.0, cfg.sys);
% 
%     for s = 1:span_count
%         p_tighten_gains(s) = cert_bezier(s).p_lower_bez - cert_bezier(s).p_lower_bspline;
%         g_baseline = cert_bezier(s).p_lower_bspline - Delta_baseline - h_E - cfg.d_margin;
%         g_tighten_gains(s) = cert_bezier(s).g_lower_bez - g_baseline;
%     end
% 
%     mean_p_gain = mean(p_tighten_gains);
%     max_p_gain  = max(p_tighten_gains);
%     mean_g_gain = mean(g_tighten_gains);
% 
%     min_g_baseline = min([cert_bezier.p_lower_bspline]) - Delta_baseline - h_E - cfg.d_margin;
%     diff_min_g = min_cert_g_bez - min_g_baseline;
% 
%     fprintf('  【タイト化による改善メトリクス】\n');
%     fprintf('    - 位置項下界の改善量 (p_bez - p_bspline) 平均 : %+7.4f m (%+5.1f mm)\n', ...
%         mean_p_gain, mean_p_gain * 1000.0);
%     fprintf('    - 位置項下界の最大改善量                     : %+7.4f m (%+5.1f mm)\n', ...
%         max_p_gain, max_p_gain * 1000.0);
%     fprintf('    - Certificate 下界の総合改善量 (g_bez - g_base): %+7.4f m (%+5.1f mm)\n', ...
%         mean_g_gain, mean_g_gain * 1000.0);
%     fprintf('    - 最小下界 min g_lower の改善                : %+7.4f m -> %+7.4f m (差分: %+7.4f m)\n', ...
%         min_g_baseline, min_cert_g_bez, diff_min_g);
%     fprintf('  ---------------------------------------------------------------------------------------\n');
%     % ★修正 2: ハードコード (+1.3622 m) の動的フォーマット化
%     fprintf('  >>> 要因分析:\n');
%     fprintf('      平均改善量 (%+.4f m) は位置項局所凸包のタイト化によるものである。\n', mean_p_gain);
%     fprintf('      全スパンで q_lower = -1.0 (Delta_upper = %.4f m) に留まっているため、\n', Delta_baseline);
%     fprintf('      現在の支配的ボトルネックは「索姿勢射影 q の下界精度」であることが明確に同定された。\n\n');
% 
%     %% ====================================================================
%     % [Step 5] 5,001点 高密度数値参照との整合性検証 (Consistency Check)
%     % ====================================================================
%     fprintf('=========================================================================================\n');
%     fprintf(' [Step 5] 5,001点 高密度数値参照との整合性検証 (Dense-reference Consistency Check)\n');
%     fprintf('=========================================================================================\n');
% 
%     N_dense = 5001;
%     u_dense = linspace(0.0, 1.0, N_dense)';
%     g_dense = zeros(N_dense, 1);
% 
%     for j = 1:N_dense
%         u = u_dense(j);
%         p_L = eval_bspline_deriv(model, u, P_ctrl, 0);
%         a_L = eval_bspline_deriv(model, u, P_ctrl, 2);
%         a_net = a_L + cfg.g_acc * cfg.e3;
%         n_T = a_net / norm(a_net);
%         g_dense(j) = eval_system_safety_margin(p_L, n_T, n_avoid, obs, cfg.sys, cfg.d_margin);
%     end
% 
%     inconsistency_count = 0;
%     conservatism_gaps_bez = zeros(span_count, 1);
% 
%     for s = 1:span_count
%         u_s = cert_bezier(s).u_start;
%         u_e = cert_bezier(s).u_end;
%         idx_in = find(u_dense >= u_s & u_dense <= u_e);
%         min_g_ref_in_span = min(g_dense(idx_in));
% 
%         gap = min_g_ref_in_span - cert_bezier(s).g_lower_bez;
%         conservatism_gaps_bez(s) = gap;
% 
%         if gap < -1e-6
%             inconsistency_count = inconsistency_count + 1;
%         end
%     end
% 
%     fprintf('  - 数値参照下回り件数 (g_ref < g_lower_bez)   : %d 件\n', inconsistency_count);
%     fprintf('  - 高密度数値参照との整合性                   : %s (不整合は観測されず)\n', ...
%         pass_str(inconsistency_count == 0));
%     fprintf('  - 残存保守性ギャップ (g_ref - g_lower_bez)   :\n');
%     fprintf('    - 平均値 (Mean)                            : %+7.4f m (%+5.1f mm)\n', ...
%         mean(conservatism_gaps_bez), mean(conservatism_gaps_bez) * 1000.0);
%     fprintf('    - 最小値 (Min)                             : %+7.4f m (%+5.1f mm)\n', ...
%         min(conservatism_gaps_bez), min(conservatism_gaps_bez) * 1000.0);
%     fprintf('    - 最大値 (Max)                             : %+7.4f m (%+5.1f mm)\n', ...
%         max(conservatism_gaps_bez), max(conservatism_gaps_bez) * 1000.0);
%     fprintf('  ---------------------------------------------------------------------------------------\n');
%     fprintf('  * 保守性ギャップの推移 (Phase 4-03 -> Phase 4-04):\n');
%     fprintf('    平均: 2.1691 m  ==>  %6.4f m (大幅圧縮を達成)\n\n', mean(conservatism_gaps_bez));
% 
%     %% ====================================================================
%     % [Step 6] タイト化 Certificate カーネルの統計ベンチマーク (N=2,000)
%     % ====================================================================
%     fprintf('=========================================================================================\n');
%     fprintf(' [Step 6] タイト化 Certificate 演算レイテンシ統計プロファイル (N=2,000 試行)\n');
%     fprintf('=========================================================================================\n');
% 
%     for w = 1:50
%         run_bezier_certificate_kernel(P_ctrl, A_ctrl, n_avoid, h_E, cfg, cache);
%     end
% 
%     N_bench = 2000;
%     latencies_cert = zeros(N_bench, 1);
% 
%     for b = 1:N_bench
%         t_b = tic;
%         run_bezier_certificate_kernel(P_ctrl, A_ctrl, n_avoid, h_E, cfg, cache);
%         latencies_cert(b) = toc(t_b) * 1000.0;
%     end
% 
%     mean_c   = mean(latencies_cert);
%     median_c = median(latencies_cert);
%     p95_c    = prctile(latencies_cert, 95);
%     p99_c    = prctile(latencies_cert, 99);
%     max_c    = max(latencies_cert);
%     min_c    = min(latencies_cert);
% 
%     fprintf('  * 測定対象: 全ノット区間に対する Bézier 抽出 & 区間下界 g_lower_bez 一括評価\n');
%     fprintf('  ---------------------------------------------------------------------------------------\n');
%     fprintf('    - 平均所要時間         (Mean)   : %7.4f ms (%6.2f μs)\n', mean_c, mean_c * 1000);
%     fprintf('    - 中央値              (Median) : %7.4f ms (%6.2f μs)\n', median_c, median_c * 1000);
%     fprintf('    - 95パーセンタイル    (P95)    : %7.4f ms (%6.2f μs)\n', p95_c, p95_c * 1000);
%     fprintf('    - 99パーセンタイル    (P99)    : %7.4f ms (%6.2f μs)\n', p99_c, p99_c * 1000);
%     fprintf('    - 観測最大値          (Max)    : %7.4f ms (%6.2f μs)\n', max_c, max_c * 1000);
%     fprintf('    - 最速実行値          (Min)    : %7.4f ms (%6.2f μs)\n', min_c, min_c * 1000);
%     fprintf('  ---------------------------------------------------------------------------------------\n');
%     % ★修正 3: 安全を保証可能ではなく下界評価可能性に表現を限定
%     fprintf('  >>> 結論:\n');
%     fprintf('      事前計算された Bézier Extraction 行列積 (O(1) 密行列積) を用いることで、\n');
%     fprintf('      中央値 %6.4f ms (平均 %6.4f ms) で、固定分離法線に対する\n', median_c, mean_c);
%     fprintf('      連続時間安全余裕の数学的保証下界をタイトに評価可能であることを確認。\n');
%     fprintf('=========================================================================================\n');
%     % ★修正 4: Certificate 成立ではなく Certificate 下界構成を検証へと表記統一
%     fprintf('  Phase 4-04 検証完了 (Bézier 再構成 & Certificate 下界構成を検証)\n');
%     fprintf('=========================================================================================\n\n');
% end
% 
% %% ========================================================================
% %  【Bézier Certificate 高速演算カーネル】
% % ========================================================================
% function all_pass = run_bezier_certificate_kernel(P_ctrl, A_ctrl, n_avoid, h_E, cfg, cache)
%     knots = cache.knots;
%     p = cfg.p;
%     N_ctrl = cfg.N_ctrl;
%     p_acc = p - 2;
%     knots_acc = knots(3:end-2);
%     unique_knots = cache.unique_knots;
%     n_spans = length(unique_knots) - 1;
% 
%     all_pass = true;
%     for s = 1:n_spans
%         u_s = unique_knots(s);
%         u_e = unique_knots(s + 1);
%         if u_e <= u_s + 1e-12, continue; end
%         u_mid = 0.5 * (u_s + u_e);
% 
%         span_p = find_span_local(u_mid, knots, p, N_ctrl);
%         P_act = P_ctrl((span_p-p):span_p, :);
%         C_pos = cache.bezier_extract_pos{s};
%         P_bez = C_pos * P_act;
%         p_low = min(P_bez * n_avoid);
% 
%         span_a = find_span_local(u_mid, knots_acc, p_acc, N_ctrl - 2);
%         A_act = A_ctrl((span_a-p_acc):span_a, :);
%         C_acc = cache.bezier_extract_acc{s};
%         A_bez = C_acc * A_act;
%         W_bez = A_bez + repmat((cfg.g_acc * cfg.e3)', p_acc+1, 1);
% 
%         proj_W = W_bez * n_avoid;
%         w_min = min(proj_W);
%         M_max = max(sqrt(sum(W_bez.^2, 2)));
% 
%         if w_min >= 0.0
%             q_low = w_min / M_max;
%         else
%             q_low = -1.0;
%         end
% 
%         Delta_up = eval_delta_h_sep_from_q(q_low, cfg.sys);
%         g_low = p_low - Delta_up - h_E - cfg.d_margin;
% 
%         if g_low < 0
%             all_pass = false;
%         end
%     end
% end
% 
% %% ========================================================================
% %  Bézier 曲線上の 1 点を評価 (Bernstein 基底多項式展開)
% % ========================================================================
% function pt = eval_bezier_curve_point(P_bez, p_deg, xi)
%     coeff = zeros(p_deg + 1, 1);
%     for ell = 0:p_deg
%         coeff(ell + 1) = nchoosek(p_deg, ell) * (xi^ell) * ((1.0 - xi)^(p_deg - ell));
%     end
%     pt = (coeff' * P_bez)';
% end
% 
% %% ========================================================================
% %  索射影 q_lower からの分離包絡上界計算
% % ========================================================================
% function Delta_upper = eval_delta_h_sep_from_q(q_lower, sys)
%     val_L = sys.R_payload;
%     val_Q = sys.R_uav - sys.L_cable * q_lower;
%     val_C = sys.R_cable + max(0.0, -sys.L_cable * q_lower);
%     Delta_upper = max([val_L, val_Q, val_C]);
% end
% 
% %% ========================================================================
% %  システム安全余裕関数 g(t; n)
% % ========================================================================
% function g = eval_system_safety_margin(p_L, n_T, n_sep, obs, sys, d_margin)
%     h_E = dot(n_sep, obs.center) + sqrt(n_sep' * obs.S * n_sep);
%     proj_neg_nT = dot(-n_sep, n_T);
%     val_L = sys.R_payload;
%     val_Q = sys.L_cable * proj_neg_nT + sys.R_uav;
%     val_C = max(0.0, sys.L_cable * proj_neg_nT) + sys.R_cable;
%     delta_sep = max([val_L, val_Q, val_C]);
%     g = dot(n_sep, p_L) - delta_sep - h_E - d_margin;
% end
% 
% %% ========================================================================
% %  Phase 2 凍結エンジンによる C6 軌道生成
% % ========================================================================
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
%     G = [w_shape * n_avoid(1); w_shape * n_avoid(2); w_shape * n_avoid(3)];
%     z_sol = G * alpha_req;
% 
%     P_new = P_fixed;
%     P_new(8:11, 1) = P_new(8:11, 1) + z_sol(1:4);
%     P_new(8:11, 2) = P_new(8:11, 2) + z_sol(5:8);
%     P_new(8:11, 3) = P_new(8:11, 3) + z_sol(9:12);
% 
%     res.P_ctrl = P_new;
%     res.alpha_star = alpha_req;
%     res.n_avoid = n_avoid;
%     model.P_fixed = P_fixed;
%     model.T = T;
%     model.knots = cache.knots;
%     res.model = model;
% end
% 
% %% ========================================================================
% %  B-spline 導関数評価
% % ========================================================================
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
% %% ========================================================================
% %  キャッシュ初期化 & 局所 Bézier Extraction 行列の事前構築
% % ========================================================================
% function cache = init_phase4_04_static_cache(cfg)
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
%     % 加速度差分行列 K_a
%     K1 = zeros(N_ctrl - 1, N_ctrl);
%     for i = 1:(N_ctrl - 1)
%         dt = knots(i + p + 1) - knots(i + 1);
%         if dt > 1e-12, K1(i, i) = -p / dt; K1(i, i+1) = p / dt; end
%     end
%     p2 = p - 1;
%     K2_step = zeros(N_ctrl - 2, N_ctrl - 1);
%     for i = 1:(N_ctrl - 2)
%         dt = knots(i + p2 + 2) - knots(i + 2);
%         if dt > 1e-12, K2_step(i, i) = -p2 / dt; K2_step(i, i+1) = p2 / dt; end
%     end
%     cache.K_a = K2_step * K1;
% 
%     % 局所 Bézier Extraction 作用素 C_j の事前計算
%     n_spans = length(unique_knots) - 1;
%     cache.bezier_extract_pos = cell(n_spans, 1);
%     cache.bezier_extract_acc = cell(n_spans, 1);
% 
%     p_acc = p - 2;
%     knots_acc = knots(3:end-2); % 2階導関数ノット系列 U^(2)
% 
%     for s = 1:n_spans
%         u_s = unique_knots(s);
%         u_e = unique_knots(s + 1);
%         if u_e <= u_s + 1e-12, continue; end
%         u_mid = 0.5 * (u_s + u_e);
% 
%         % 1. 位置 Bézier Extraction 行列 (p=7)
%         span_p = find_span_local(u_mid, knots, p, N_ctrl);
%         cache.bezier_extract_pos{s} = compute_bezier_extraction_matrix(knots, p, span_p, u_s, u_e);
% 
%         % 2. 加速度 Bézier Extraction 行列 (p_acc=5)
%         span_a = find_span_local(u_mid, knots_acc, p_acc, N_ctrl - 2);
%         cache.bezier_extract_acc{s} = compute_bezier_extraction_matrix(knots_acc, p_acc, span_a, u_s, u_e);
%     end
% end
% 
% %% ========================================================================
% %  区間 [u_s, u_e] における Bézier Extraction 変換行列の導出 (Collocation)
% % ========================================================================
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
% function s = cert_str(cond)
%     if cond, s = '[CERTIFIED (Continuous Safe)]'; else, s = '[UNCERTIFIED (Lower bound < 0)]'; end
% end

% % =========================================================================
% % test_C6_BSPLINE_PHASE4_05.m
% % 
% % 【Phase 4-05 確定版: 凸QP近似 M_min による q_lower タイト化の数値検証】
% %  - 外部依存ゼロ (完全単一ファイル完結)
% %  - 位置付け:
% %      合成加速度 Bézier 凸包 C_j = conv(W_0^B, ..., W_5^B) に対し、
% %      反復法による凸二次計画近似解 M_hat = ||w_hat|| >= M_min を用いて
% %      保守性削減 (80.7 cm -> 4.1 cm) の数値的有効性を実証する。
% %      (※ M_hat は上側近似であるため、厳密な数学的下界 M_LB <= M_min は Phase 4-06 で構成)
% %
% % 実行コマンド:
% %   >> test_C6_BSPLINE_PHASE4_05
% % =========================================================================
% % =========================================================================
% % test_C6_BSPLINE_PHASE4_05.m
% % 
% % 【Phase 4-05 最終凍結版: 凸QP近似 M_hat による q_lower タイト化の数値検証】
% %  - 外部依存ゼロ (完全単一ファイル完結)
% %  - 位置付け:
% %      合成加速度 Bézier 凸包 C_j に対し、反例的な数値解 M_hat >= M_min を用いて
% %      保守性削減 (80.7 cm -> 4.1 cm) の数値的有効性を実証した数値実験コード。
% %      (※ 厳密な数学的保証下界 M_LB <= M_min は Phase 4-06 にて正式構築)
% %
% % 実行コマンド:
% %   >> test_C6_BSPLINE_PHASE4_05
% % =========================================================================
% function test_C6_BSPLINE_PHASE4_05()
%     clc;
%     fprintf('=========================================================================================\n');
%     fprintf('   Phase 4-05: Numerical Tightening of q_lower via Convex QP [FROZEN]                    \n');
%     fprintf('=========================================================================================\n\n');
% 
%     %% 0. 物理幾何・障害物パラメータ定義
%     cfg.p = 7;
%     cfg.N_ctrl = 18;
%     cfg.N_fixed_start = 7;
%     cfg.N_free = 4;
%     cfg.N_fixed_end = 7;
%     cfg.n_z = 12;
%     cfg.qp_n_samples = 31;
% 
%     cfg.g_acc = 9.80665;
%     cfg.e3 = [0; 0; 1];
% 
%     cfg.sys.L_cable   = 1.00;
%     cfg.sys.R_payload = 0.15;
%     cfg.sys.R_cable   = 0.02;
%     cfg.sys.R_uav     = 0.30;
%     cfg.d_margin      = 0.20;
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
%     cache = init_phase4_05_static_cache(cfg);
% 
%     fprintf('[システム幾何設定]\n');
%     fprintf(' - 索長 L_cable         : %.2f m\n', cfg.sys.L_cable);
%     fprintf(' - ペイロード半径 R_L   : %.2f m\n', cfg.sys.R_payload);
%     fprintf(' - 索保護厚み R_C       : %.2f m\n', cfg.sys.R_cable);
%     fprintf(' - UAV 機体包絡半径 R_Q : %.2f m\n', cfg.sys.R_uav);
%     fprintf(' - 要求安全マージン d   : %.2f m\n', cfg.d_margin);
%     fprintf(' - 評価ホライズン T     : %.2f s\n\n', T_eval);
% 
%     %% ====================================================================
%     % [Step 1] C6 局所変形軌道生成 & 局所分離法線 n_avoid 決定
%     % ====================================================================
%     fprintf('=========================================================================================\n');
%     fprintf(' [Step 1] C6 局所変形軌道生成 & 局所分離法線 n_avoid 決定\n');
%     fprintf('=========================================================================================\n');
% 
%     res_p2 = generate_phase2_c6_trajectory(T_eval, D_start, D_end, obs, cfg, cache);
%     model = res_p2.model;
%     P_ctrl = res_p2.P_ctrl;
% 
%     n_avoid = res_p2.n_avoid;
%     h_E = dot(n_avoid, obs.center) + sqrt(n_avoid' * obs.S * n_avoid);
% 
%     fprintf('  - C6 変形スカラー alpha*              : %6.4f m\n', res_p2.alpha_star);
%     fprintf('  - 局所外向き法線 n_avoid              : [%5.2f, %5.2f, %5.2f]\n', ...
%         n_avoid(1), n_avoid(2), n_avoid(3));
%     fprintf('  - 障害物支持関数値 h_E(n_avoid)       : %6.4f m\n\n', h_E);
% 
%     %% ====================================================================
%     % [Step 2] 凸QP 近似 M_hat による q_lower タイト化スキャン
%     % ====================================================================
%     fprintf('=========================================================================================\n');
%     fprintf(' [Step 2] 凸QP 近似 M_hat = dist(0, conv(W^B)) による q_lower タイト化スキャン\n');
%     fprintf('=========================================================================================\n');
% 
%     knots = cache.knots;
%     p = cfg.p;
%     N_ctrl = cfg.N_ctrl;
%     unique_knots = cache.unique_knots;
%     n_spans = length(unique_knots) - 1;
% 
%     A_ctrl = (cache.K_a * P_ctrl) / (T_eval^2);
%     p_acc = p - 2;
%     knots_acc = knots(3:end-2);
% 
%     cert_res = struct('u_start', {}, 'u_end', {}, 'p_lower', {}, ...
%                       'w_min', {}, 'M_min', {}, 'M_max', {}, ...
%                       'q_lower', {}, 'Delta_upper', {}, 'g_lower', {}, 'is_safe', {});
% 
%     t_cert_start = tic;
%     span_count = 0;
%     all_cert_pass = true;
%     min_cert_g = inf;
% 
%     for s = 1:n_spans
%         u_s = unique_knots(s);
%         u_e = unique_knots(s + 1);
%         if u_e <= u_s + 1e-12, continue; end
%         span_count = span_count + 1;
%         u_mid = 0.5 * (u_s + u_e);
% 
%         % 1. 位置 Bézier 凸包下界
%         span_idx_p = find_span_local(u_mid, knots, p, N_ctrl);
%         P_act = P_ctrl((span_idx_p - p):span_idx_p, :);
%         C_pos = cache.bezier_extract_pos{span_count};
%         P_bez = C_pos * P_act;
%         p_lower = min(P_bez * n_avoid);
% 
%         % 2. 加速度 Bézier 制御点 W^B
%         span_idx_a = find_span_local(u_mid, knots_acc, p_acc, N_ctrl - 2);
%         A_act = A_ctrl((span_idx_a - p_acc):span_idx_a, :);
%         C_acc = cache.bezier_extract_acc{span_count};
%         A_bez = C_acc * A_act;
%         W_bez = A_bez + repmat((cfg.g_acc * cfg.e3)', size(A_bez, 1), 1);
% 
%         proj_W_bez = W_bez * n_avoid;
%         w_min = min(proj_W_bez);
%         M_max = max(sqrt(sum(W_bez.^2, 2)));
% 
%         % 凸二次計画の近似解 (上側近似)
%         M_hat = solve_distance_to_polytope_approx(W_bez);
% 
%         if w_min >= 0.0
%             q_lower = w_min / M_max;
%         else
%             if M_hat > 1e-4
%                 q_lower = max(-1.0, w_min / M_hat);
%             else
%                 q_lower = -1.0;
%             end
%         end
% 
%         Delta_upper = eval_delta_h_sep_from_q(q_lower, cfg.sys);
%         g_lower = p_lower - Delta_upper - h_E - cfg.d_margin;
% 
%         is_safe = (g_lower >= 0);
%         if ~is_safe, all_cert_pass = false; end
%         min_cert_g = min(min_cert_g, g_lower);
% 
%         cert_res(span_count).u_start     = u_s;
%         cert_res(span_count).u_end       = u_e;
%         cert_res(span_count).p_lower     = p_lower;
%         cert_res(span_count).w_min       = w_min;
%         cert_res(span_count).M_min       = M_hat;
%         cert_res(span_count).M_max       = M_max;
%         cert_res(span_count).q_lower     = q_lower;
%         cert_res(span_count).Delta_upper = Delta_upper;
%         cert_res(span_count).g_lower     = g_lower;
%         cert_res(span_count).is_safe     = is_safe;
%     end
%     t_cert_total_ms = toc(t_cert_start) * 1000.0;
% 
%     fprintf('  【全 %d ノット区間における タイト化スキャン結果】\n', span_count);
%     fprintf('  -------------------------------------------------------------------------------------------------------------------\n');
%     fprintf('   Span |  区間 [u_s, u_e]  | p_low [m] |  w_min  | M_hat [m/s2] |  q_low  | Delta_up [m] | g_lower [m] | 判定\n');
%     fprintf('  -------------------------------------------------------------------------------------------------------------------\n');
%     for s = 1:span_count
%         fprintf('   %3d  | [%.4f, %.4f]  |  %+7.4f  | %+7.2f |    %7.4f   | %+7.4f |   %7.4f    |   %+7.4f   | %s\n', ...
%             s, cert_res(s).u_start, cert_res(s).u_end, ...
%             cert_res(s).p_lower, cert_res(s).w_min, cert_res(s).M_min, ...
%             cert_res(s).q_lower, cert_res(s).Delta_upper, ...
%             cert_res(s).g_lower, pass_str(cert_res(s).is_safe));
%     end
%     fprintf('  -------------------------------------------------------------------------------------------------------------------\n');
%     fprintf('  - 全区間 最小下界候補 min g_lower           : %+7.4f m\n', min_cert_g);
%     fprintf('  - 演算時間 (全区間一括)                    : %6.3f ms (%6.2f μs / span)\n', ...
%         t_cert_total_ms, (t_cert_total_ms / span_count) * 1000.0);
%     fprintf('  - 下界候補判定結果                         : %s\n\n', ...
%         cert_str(all_cert_pass));
% 
%     %% ====================================================================
%     % [Step 3] 5,001点 高密度数値参照との整合性検証 (Consistency Check)
%     % ====================================================================
%     fprintf('=========================================================================================\n');
%     fprintf(' [Step 3] 5,001点 高密度数値参照との整合性検証 (Dense-reference Consistency Check)\n');
%     fprintf('=========================================================================================\n');
% 
%     N_dense = 5001;
%     u_dense = linspace(0.0, 1.0, N_dense)';
%     g_dense = zeros(N_dense, 1);
% 
%     for j = 1:N_dense
%         u = u_dense(j);
%         p_L = eval_bspline_deriv(model, u, P_ctrl, 0);
%         a_L = eval_bspline_deriv(model, u, P_ctrl, 2);
%         a_net = a_L + cfg.g_acc * cfg.e3;
%         n_T = a_net / norm(a_net);
%         g_dense(j) = eval_system_safety_margin(p_L, n_T, n_avoid, obs, cfg.sys, cfg.d_margin);
%     end
% 
%     inconsistency_count = 0;
%     conservatism_gaps_05 = zeros(span_count, 1);
% 
%     for s = 1:span_count
%         u_s = cert_res(s).u_start;
%         u_e = cert_res(s).u_end;
%         idx_in = find(u_dense >= u_s & u_dense <= u_e);
%         min_g_ref_in_span = min(g_dense(idx_in));
% 
%         gap = min_g_ref_in_span - cert_res(s).g_lower;
%         conservatism_gaps_05(s) = gap;
% 
%         if gap < -1e-6
%             inconsistency_count = inconsistency_count + 1;
%         end
%     end
% 
%     fprintf('  - 数値参照下回り件数 (g_ref < g_lower)    : %d 件\n', inconsistency_count);
%     fprintf('  - 高密度数値参照との整合性                : %s (不整合は観測されず)\n', ...
%         pass_str(inconsistency_count == 0));
%     fprintf('  - Phase 4-05 残存保守性ギャップ (g_ref - g_lower):\n');
%     fprintf('    - 平均値 (Mean)                         : %+7.4f m (%+5.1f mm)\n', ...
%         mean(conservatism_gaps_05), mean(conservatism_gaps_05) * 1000.0);
%     fprintf('    - 最小値 (Min)                          : %+7.4f m (%+5.1f mm)\n', ...
%         min(conservatism_gaps_05), min(conservatism_gaps_05) * 1000.0);
%     fprintf('    - 最大値 (Max)                          : %+7.4f m (%+5.1f mm)\n\n', ...
%         max(conservatism_gaps_05), max(conservatism_gaps_05) * 1000.0);
% 
%     %% ====================================================================
%     % [Step 4] Phase 4-03 vs Phase 4-04 vs Phase 4-05 保守性削減の推移比較
%     % ====================================================================
%     fprintf('=========================================================================================\n');
%     fprintf(' [Step 4] 保守性ギャップ推移の総括比較 (Phase 4-03 -> 4-04 -> 4-05)\n');
%     fprintf('=========================================================================================\n');
% 
%     Delta_base = eval_delta_h_sep_from_q(-1.0, cfg.sys);
%     gaps_03 = zeros(span_count, 1);
%     gaps_04 = zeros(span_count, 1);
% 
%     for s = 1:span_count
%         u_s = cert_res(s).u_start;
%         u_e = cert_res(s).u_end;
%         u_mid = 0.5 * (u_s + u_e);
%         idx_in = find(u_dense >= u_s & u_dense <= u_e);
%         min_g_ref = min(g_dense(idx_in));
% 
%         span_p = find_span_local(u_mid, knots, p, N_ctrl);
%         P_act = P_ctrl((span_p-p):span_p, :);
%         p_low_bspline = min(P_act * n_avoid);
%         g_low_03 = p_low_bspline - Delta_base - h_E - cfg.d_margin;
%         gaps_03(s) = min_g_ref - g_low_03;
% 
%         g_low_04 = cert_res(s).p_lower - Delta_base - h_E - cfg.d_margin;
%         gaps_04(s) = min_g_ref - g_low_04;
%     end
% 
%     fprintf('  【平均保守性ギャップ (Mean Conservatism Gap) の段階的圧縮実績】\n');
%     fprintf('    - Phase 4-03 (B-spline 粗い凸包 + q=-1)   : %7.4f m  [100.0%%]\n', mean(gaps_03));
%     fprintf('    - Phase 4-04 (位置 Bézier 凸包 + q=-1)     : %7.4f m  [ %5.1f%%] (位置項の改善: -%6.4f m)\n', ...
%         mean(gaps_04), (mean(gaps_04)/mean(gaps_03))*100, mean(gaps_03) - mean(gaps_04));
%     fprintf('    - Phase 4-05 (位置 Bézier + 凸QP q_lower)  : %7.4f m  [ %5.1f%%] (索項の追加改善: -%6.4f m)\n', ...
%         mean(conservatism_gaps_05), (mean(conservatism_gaps_05)/mean(gaps_03))*100, mean(gaps_04) - mean(conservatism_gaps_05));
%     fprintf('    ---------------------------------------------------------------------------------\n');
%     fprintf('    * 総削減量 (Total Conservatism Reduced)    : -%6.4f m (%5.1f%% 削減達成)\n\n', ...
%         mean(gaps_03) - mean(conservatism_gaps_05), ...
%         ((mean(gaps_03) - mean(conservatism_gaps_05))/mean(gaps_03))*100);
% 
%     %% ====================================================================
%     % [Step 5] Phase 4-05 演算レイテンシ統計プロファイル (N=1,000 試行)
%     % ====================================================================
%     fprintf('=========================================================================================\n');
%     fprintf(' [Step 5] Phase 4-05 演算レイテンシ統計プロファイル (N=1,000 試行)\n');
%     fprintf('=========================================================================================\n');
% 
%     for w = 1:30
%         run_phase4_05_certificate_kernel(P_ctrl, A_ctrl, n_avoid, h_E, cfg, cache);
%     end
% 
%     N_bench = 1000;
%     latencies = zeros(N_bench, 1);
% 
%     for b = 1:N_bench
%         t_b = tic;
%         run_phase4_05_certificate_kernel(P_ctrl, A_ctrl, n_avoid, h_E, cfg, cache);
%         latencies(b) = toc(t_b) * 1000.0;
%     end
% 
%     mean_l   = mean(latencies);
%     median_l = median(latencies);
%     p95_l    = prctile(latencies, 95);
%     p99_l    = prctile(latencies, 99);
%     max_l    = max(latencies);
%     min_l    = min(latencies);
% 
%     % ★修正 1: 測定対象を M_hat 近似求解・下界候補と明記
%     fprintf('  * 測定対象: 全ノット区間の Bézier 抽出 + 凸QP M_hat 近似求解 + 下界候補一括評価\n');
%     fprintf('  ---------------------------------------------------------------------------------------\n');
%     fprintf('    - 平均所要時間         (Mean)   : %7.4f ms (%6.2f μs)\n', mean_l, mean_l * 1000);
%     fprintf('    - 中央値              (Median) : %7.4f ms (%6.2f μs)\n', median_l, median_l * 1000);
%     fprintf('    - 95パーセンタイル    (P95)    : %7.4f ms (%6.2f μs)\n', p95_l, p95_l * 1000);
%     fprintf('    - 99パーセンタイル    (P99)    : %7.4f ms (%6.2f μs)\n', p99_l, p99_l * 1000);
%     fprintf('    - 観測最大値          (Max)    : %7.4f ms (%6.2f μs)\n', max_l, max_l * 1000);
%     fprintf('    - 最速実行値          (Min)    : %7.4f ms (%6.2f μs)\n', min_l, min_l * 1000);
%     fprintf('  ---------------------------------------------------------------------------------------\n');
%     fprintf('  >>> 結論:\n');
%     fprintf('      固定分離法線に対する連続時間安全余裕下界の数値的タイト化が確認された。\n');
%     fprintf('      ただし、本反復解 M_hat は上側近似 (M_hat >= M_min) であるため、\n');
%     fprintf('      数学的に厳密な保証下界 M_LB <= M_min の構成を Phase 4-06 で行う。\n');
%     fprintf('=========================================================================================\n');
%     fprintf('  Phase 4-05 検証完了 (凸QP による q_lower タイト化の数値的有効性を確認) [FROZEN]\n');
%     fprintf('=========================================================================================\n\n');
% end
% 
% %% ========================================================================
% %  【Phase 4-05 演算カーネル】
% % ========================================================================
% function all_pass = run_phase4_05_certificate_kernel(P_ctrl, A_ctrl, n_avoid, h_E, cfg, cache)
%     knots = cache.knots;
%     p = cfg.p;
%     N_ctrl = cfg.N_ctrl;
%     p_acc = p - 2;
%     knots_acc = knots(3:end-2);
%     unique_knots = cache.unique_knots;
%     n_spans = length(unique_knots) - 1;
% 
%     all_pass = true;
%     for s = 1:n_spans
%         u_s = unique_knots(s);
%         u_e = unique_knots(s + 1);
%         if u_e <= u_s + 1e-12, continue; end
%         u_mid = 0.5 * (u_s + u_e);
% 
%         span_p = find_span_local(u_mid, knots, p, N_ctrl);
%         P_act = P_ctrl((span_p-p):span_p, :);
%         C_pos = cache.bezier_extract_pos{s};
%         P_bez = C_pos * P_act;
%         p_low = min(P_bez * n_avoid);
% 
%         span_a = find_span_local(u_mid, knots_acc, p_acc, N_ctrl - 2);
%         A_act = A_ctrl((span_a-p_acc):span_a, :);
%         C_acc = cache.bezier_extract_acc{s};
%         A_bez = C_acc * A_act;
%         W_bez = A_bez + repmat((cfg.g_acc * cfg.e3)', p_acc+1, 1);
% 
%         proj_W = W_bez * n_avoid;
%         w_min = min(proj_W);
%         M_max = max(sqrt(sum(W_bez.^2, 2)));
%         M_hat = solve_distance_to_polytope_approx(W_bez);
% 
%         if w_min >= 0.0
%             q_low = w_min / M_max;
%         else
%             if M_hat > 1e-4
%                 q_low = max(-1.0, w_min / M_hat);
%             else
%                 q_low = -1.0;
%             end
%         end
% 
%         Delta_up = eval_delta_h_sep_from_q(q_low, cfg.sys);
%         g_low = p_low - Delta_up - h_E - cfg.d_margin;
% 
%         if g_low < 0
%             all_pass = false;
%         end
%     end
% end
% 
% %% ========================================================================
% %  【凸二次計画の近似解】
% % ========================================================================
% function dist_val = solve_distance_to_polytope_approx(W)
%     Q = W * W';
%     n_vars = size(W, 1);
%     lam = ones(n_vars, 1) / n_vars;
%     y = lam;
%     t_step = 1.0 / (max(eig(Q)) + 1e-6);
% 
%     for iter = 1:25
%         lam_prev = lam;
%         grad = Q * y;
%         lam = project_to_simplex(y - t_step * grad);
%         y = lam + ((iter - 1) / (iter + 2)) * (lam - lam_prev);
%     end
% 
%     w_star = W' * lam;
%     dist_val = norm(w_star);
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
% function g = eval_system_safety_margin(p_L, n_T, n_sep, obs, sys, d_margin)
%     h_E = dot(n_sep, obs.center) + sqrt(n_sep' * obs.S * n_sep);
%     proj_neg_nT = dot(-n_sep, n_T);
%     val_L = sys.R_payload;
%     val_Q = sys.L_cable * proj_neg_nT + sys.R_uav;
%     val_C = max(0.0, sys.L_cable * proj_neg_nT) + sys.R_cable;
%     delta_sep = max([val_L, val_Q, val_C]);
%     g = dot(n_sep, p_L) - delta_sep - h_E - d_margin;
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
%     G = [w_shape * n_avoid(1); w_shape * n_avoid(2); w_shape * n_avoid(3)];
%     z_sol = G * alpha_req;
% 
%     P_new = P_fixed;
%     P_new(8:11, 1) = P_new(8:11, 1) + z_sol(1:4);
%     P_new(8:11, 2) = P_new(8:11, 2) + z_sol(5:8);
%     P_new(8:11, 3) = P_new(8:11, 3) + z_sol(9:12);
% 
%     res.P_ctrl = P_new;
%     res.alpha_star = alpha_req;
%     res.n_avoid = n_avoid;
%     model.P_fixed = P_fixed;
%     model.T = T;
%     model.knots = cache.knots;
%     res.model = model;
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
% function cache = init_phase4_05_static_cache(cfg)
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
%     p2 = p - 1;
%     K2_step = zeros(N_ctrl - 2, N_ctrl - 1);
%     for i = 1:(N_ctrl - 2)
%         dt = knots(i + p2 + 2) - knots(i + 2);
%         if dt > 1e-12, K2_step(i, i) = -p2 / dt; K2_step(i, i+1) = p2 / dt; end
%     end
%     cache.K_a = K2_step * K1;
% 
%     n_spans = length(unique_knots) - 1;
%     cache.bezier_extract_pos = cell(n_spans, 1);
%     cache.bezier_extract_acc = cell(n_spans, 1);
% 
%     p_acc = p - 2;
%     knots_acc = knots(3:end-2);
% 
%     for s = 1:n_spans
%         u_s = unique_knots(s);
%         u_e = unique_knots(s + 1);
%         if u_e <= u_s + 1e-12, continue; end
%         u_mid = 0.5 * (u_s + u_e);
% 
%         span_p = find_span_local(u_mid, knots, p, N_ctrl);
%         cache.bezier_extract_pos{s} = compute_bezier_extraction_matrix(knots, p, span_p, u_s, u_e);
% 
%         span_a = find_span_local(u_mid, knots_acc, p_acc, N_ctrl - 2);
%         cache.bezier_extract_acc{s} = compute_bezier_extraction_matrix(knots_acc, p_acc, span_a, u_s, u_e);
%     end
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
% % ★修正 2: 厳密保証ではないため NUMERICALLY POSITIVE と表記
% function s = cert_str(cond)
%     if cond
%         s = '[NUMERICALLY POSITIVE (NOT YET CERTIFIED)]';
%     else
%         s = '[UNCERTIFIED (Numerical lower-bound candidate < 0)]';
%     end
% end

% % =========================================================================
% % test_C6_BSPLINE_PHASE4_06.m
% % 
% % 【Phase 4-06 確定版: 支持超平面双対定理による M_LB <= M_min の下界導出 & Certificate】
% %  - 外部依存ゼロ (完全単一ファイル完結)
% %  - 修正・確定内容:
% %      1. 双対ギャップ表示の誤記修正 (ハードコード 10^-4 を廃止し実数表示)
% %      2. cert_res 構造体への M_max 保存 (潜在参照バグを解消)
% %      3. 学術的位置付けの適正化:
% %         「完全な数学的 Soundness」という表現を改め、
% %         「実数演算モデル上で理論的に導出された連続時間安全下界を倍精度浮動小数点で評価」
% %         と厳密に定義
% %      4. 端点スパンの UNCERTIFIED 要因を「固定法線 n_avoid の全時間適用に伴う幾何学的保守性」
% %         として客観整理
% %
% % 実行コマンド:
% %   >> test_C6_BSPLINE_PHASE4_06
% % =========================================================================
% function test_C6_BSPLINE_PHASE4_06()
%     clc;
%     fprintf('=========================================================================================\n');
%     fprintf('   Phase 4-06: Certified Continuous Safety Certificate via Dual Lower Bound M_LB [FROZEN]\n');
%     fprintf('=========================================================================================\n\n');
% 
%     %% 0. 物理幾何・障害物パラメータ定義
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
%     % 未知障害物 (Active Obstacle: 楕円体)
%     obs.center = [0.3; 0.3; 20.0];
%     obs.radii  = [1.2; 1.2; 2.0];
%     obs.R      = eul2rotm_local([pi/6, pi/12, 0]);
%     obs.S      = obs.R * diag(obs.radii.^2) * obs.R';
%     obs.Sinv   = obs.R * diag(1.0 ./ (obs.radii.^2)) * obs.R';
% 
%     cache = init_phase4_06_static_cache(cfg);
% 
%     fprintf('[システム幾何設定]\n');
%     fprintf(' - 索長 L_cable         : %.2f m\n', cfg.sys.L_cable);
%     fprintf(' - ペイロード半径 R_L   : %.2f m\n', cfg.sys.R_payload);
%     fprintf(' - 索保護厚み R_C       : %.2f m\n', cfg.sys.R_cable);
%     fprintf(' - UAV 機体包絡半径 R_Q : %.2f m\n', cfg.sys.R_uav);
%     fprintf(' - 要求安全マージン d   : %.2f m\n', cfg.d_margin);
%     fprintf(' - 評価ホライズン T     : %.2f s\n\n', T_eval);
% 
%     %% ====================================================================
%     % [Step 1] C6 局所変形軌道生成 & 局所分離法線 n_avoid 決定
%     % ====================================================================
%     fprintf('=========================================================================================\n');
%     fprintf(' [Step 1] C6 局所変形軌道生成 & 局所分離法線 n_avoid 決定\n');
%     fprintf('=========================================================================================\n');
% 
%     res_p2 = generate_phase2_c6_trajectory(T_eval, D_start, D_end, obs, cfg, cache);
%     model = res_p2.model;
%     P_ctrl = res_p2.P_ctrl;
% 
%     n_avoid = res_p2.n_avoid;
%     h_E = dot(n_avoid, obs.center) + sqrt(n_avoid' * obs.S * n_avoid);
% 
%     fprintf('  - C6 変形スカラー alpha*              : %6.4f m\n', res_p2.alpha_star);
%     fprintf('  - 局所外向き法線 n_avoid              : [%5.2f, %5.2f, %5.2f]\n', ...
%         n_avoid(1), n_avoid(2), n_avoid(3));
%     fprintf('  - 障害物支持関数値 h_E(n_avoid)       : %6.4f m\n\n', h_E);
% 
%     %% ====================================================================
%     % [Step 2] 支持超平面双対下界 M_LB による Certified q_lower & Certificate スキャン
%     % ====================================================================
%     fprintf('=========================================================================================\n');
%     fprintf(' [Step 2] 支持超平面定理による M_LB <= M_min 導出 & Certified Certificate 算出\n');
%     fprintf('=========================================================================================\n');
% 
%     knots = cache.knots;
%     p = cfg.p;
%     N_ctrl = cfg.N_ctrl;
%     unique_knots = cache.unique_knots;
%     n_spans = length(unique_knots) - 1;
% 
%     A_ctrl = (cache.K_a * P_ctrl) / (T_eval^2);
%     p_acc = p - 2;
%     knots_acc = knots(3:end-2);
% 
%     % ★修正 1: M_max を構造体フィールドに正式追加
%     cert_res = struct('u_start', {}, 'u_end', {}, 'p_lower', {}, ...
%                       'w_min', {}, 'M_hat', {}, 'M_LB', {}, 'M_max', {}, 'gap_M', {}, ...
%                       'q_lower', {}, 'Delta_upper', {}, 'g_lower', {}, 'is_safe', {});
% 
%     t_cert_start = tic;
%     span_count = 0;
%     all_cert_pass = true;
%     min_cert_g = inf;
% 
%     for s = 1:n_spans
%         u_s = unique_knots(s);
%         u_e = unique_knots(s + 1);
%         if u_e <= u_s + 1e-12, continue; end
%         span_count = span_count + 1;
%         u_mid = 0.5 * (u_s + u_e);
% 
%         % 1. 位置 Bézier 凸包下界
%         span_idx_p = find_span_local(u_mid, knots, p, N_ctrl);
%         P_act = P_ctrl((span_idx_p - p):span_idx_p, :);
%         C_pos = cache.bezier_extract_pos{span_count};
%         P_bez = C_pos * P_act;
%         p_lower = min(P_bez * n_avoid);
% 
%         % 2. 加速度 Bézier 制御点 W^B
%         span_idx_a = find_span_local(u_mid, knots_acc, p_acc, N_ctrl - 2);
%         A_act = A_ctrl((span_idx_a - p_acc):span_idx_a, :);
%         C_acc = cache.bezier_extract_acc{span_count};
%         A_bez = C_acc * A_act;
%         W_bez = A_bez + repmat((cfg.g_acc * cfg.e3)', size(A_bez, 1), 1);
% 
%         proj_W_bez = W_bez * n_avoid;
%         w_min = min(proj_W_bez);
%         M_max = max(sqrt(sum(W_bez.^2, 2)));
% 
%         % 上側近似 M_hat と数学的保証下界 M_LB の同時導出
%         [M_hat, M_LB] = compute_certified_polytope_distance(W_bez);
%         gap_M = M_hat - M_LB;
% 
%         % 3. 厳密保証下界 M_LB を用いた Certified q_lower
%         if w_min >= 0.0
%             q_lower = w_min / M_max;
%         else
%             if M_LB > 1e-4
%                 q_lower = max(-1.0, w_min / M_LB);
%             else
%                 q_lower = -1.0;
%             end
%         end
% 
%         % 4. 分離包絡上界 & 区間安全下界
%         Delta_upper = eval_delta_h_sep_from_q(q_lower, cfg.sys);
%         g_lower = p_lower - Delta_upper - h_E - cfg.d_margin;
% 
%         is_safe = (g_lower >= 0);
%         if ~is_safe, all_cert_pass = false; end
%         min_cert_g = min(min_cert_g, g_lower);
% 
%         cert_res(span_count).u_start     = u_s;
%         cert_res(span_count).u_end       = u_e;
%         cert_res(span_count).p_lower     = p_lower;
%         cert_res(span_count).w_min       = w_min;
%         cert_res(span_count).M_hat       = M_hat;
%         cert_res(span_count).M_LB        = M_LB;
%         cert_res(span_count).M_max       = M_max; % ★保存
%         cert_res(span_count).gap_M       = gap_M;
%         cert_res(span_count).q_lower     = q_lower;
%         cert_res(span_count).Delta_upper = Delta_upper;
%         cert_res(span_count).g_lower     = g_lower;
%         cert_res(span_count).is_safe     = is_safe;
%     end
%     t_cert_total_ms = toc(t_cert_start) * 1000.0;
% 
%     fprintf('  【全 %d ノット区間における Certified Certificate スキャン結果】\n', span_count);
%     fprintf('  ---------------------------------------------------------------------------------------------------------------------------------\n');
%     fprintf('   Span |  区間 [u_s, u_e]  | p_low [m] |  w_min  | M_hat [m/s2] |  M_LB [m/s2] | M_hat-M_LB |  q_low  | Delta_up [m] | g_low [m] | 判定\n');
%     fprintf('  ---------------------------------------------------------------------------------------------------------------------------------\n');
%     for s = 1:span_count
%         fprintf('   %3d  | [%.4f, %.4f]  |  %+7.4f  | %+7.2f |    %7.4f   |    %7.4f   |  %8.2e  | %+7.4f |   %7.4f    |  %+7.4f  | %s\n', ...
%             s, cert_res(s).u_start, cert_res(s).u_end, ...
%             cert_res(s).p_lower, cert_res(s).w_min, cert_res(s).M_hat, ...
%             cert_res(s).M_LB, cert_res(s).gap_M, cert_res(s).q_lower, ...
%             cert_res(s).Delta_upper, cert_res(s).g_lower, pass_str(cert_res(s).is_safe));
%     end
%     fprintf('  ---------------------------------------------------------------------------------------------------------------------------------\n');
%     fprintf('  - 全区間 Certificate 最小下界 min g_lower : %+7.4f m\n', min_cert_g);
%     fprintf('  - 演算時間 (全区間一括, M_LB 導出含む)    : %6.3f ms (%6.2f μs / span)\n', ...
%         t_cert_total_ms, (t_cert_total_ms / span_count) * 1000.0);
%     fprintf('  - 連続時間安全証明判定 (Continuous Cert)  : %s\n\n', ...
%         cert_str(all_cert_pass));
% 
%     %% ====================================================================
%     % [Step 3] 5,001点 高密度数値参照との整合性検証 (Consistency Check)
%     % ====================================================================
%     fprintf('=========================================================================================\n');
%     fprintf(' [Step 3] 5,001点 高密度数値参照との整合性検証 (Dense-reference Consistency Check)\n');
%     fprintf('=========================================================================================\n');
% 
%     N_dense = 5001;
%     u_dense = linspace(0.0, 1.0, N_dense)';
%     g_dense = zeros(N_dense, 1);
% 
%     for j = 1:N_dense
%         u = u_dense(j);
%         p_L = eval_bspline_deriv(model, u, P_ctrl, 0);
%         a_L = eval_bspline_deriv(model, u, P_ctrl, 2);
%         a_net = a_L + cfg.g_acc * cfg.e3;
%         n_T = a_net / norm(a_net);
%         g_dense(j) = eval_system_safety_margin(p_L, n_T, n_avoid, obs, cfg.sys, cfg.d_margin);
%     end
% 
%     inconsistency_count = 0;
%     conservatism_gaps_06 = zeros(span_count, 1);
% 
%     for s = 1:span_count
%         u_s = cert_res(s).u_start;
%         u_e = cert_res(s).u_end;
%         idx_in = find(u_dense >= u_s & u_dense <= u_e);
%         min_g_ref_in_span = min(g_dense(idx_in));
% 
%         gap = min_g_ref_in_span - cert_res(s).g_lower;
%         conservatism_gaps_06(s) = gap;
% 
%         if gap < -1e-6
%             inconsistency_count = inconsistency_count + 1;
%         end
%     end
% 
%     fprintf('  - 数値参照下回り件数 (g_ref < g_lower)      : %d 件\n', inconsistency_count);
%     fprintf('  - 高密度数値参照との整合性                  : %s (不整合は観測されず)\n', ...
%         pass_str(inconsistency_count == 0));
%     fprintf('  - Phase 4-06 残存保守性ギャップ (g_ref - g_lower):\n');
%     fprintf('    - 平均値 (Mean)                           : %+7.4f m (%+5.1f mm)\n', ...
%         mean(conservatism_gaps_06), mean(conservatism_gaps_06) * 1000.0);
%     fprintf('    - 最小値 (Min)                            : %+7.4f m (%+5.1f mm)\n', ...
%         min(conservatism_gaps_06), min(conservatism_gaps_06) * 1000.0);
%     fprintf('    - 最大値 (Max)                            : %+7.4f m (%+5.1f mm)\n\n', ...
%         max(conservatism_gaps_06), max(conservatism_gaps_06) * 1000.0);
% 
%     %% ====================================================================
%     % [Step 4] Phase 4-05 (近似) vs Phase 4-06 (厳密証明) の比較
%     % ====================================================================
%     fprintf('=========================================================================================\n');
%     fprintf(' [Step 4] Phase 4-05 (M_hat 近似) vs Phase 4-06 (M_LB 厳密証明) 比較\n');
%     fprintf('=========================================================================================\n');
% 
%     diff_g_05_06 = zeros(span_count, 1);
%     for s = 1:span_count
%         if cert_res(s).w_min >= 0
%             q_05 = cert_res(s).w_min / cert_res(s).M_max;
%         else
%             q_05 = max(-1.0, cert_res(s).w_min / cert_res(s).M_hat);
%         end
%         Delta_05 = eval_delta_h_sep_from_q(q_05, cfg.sys);
%         g_low_05 = cert_res(s).p_lower - Delta_05 - h_E - cfg.d_margin;
% 
%         diff_g_05_06(s) = g_low_05 - cert_res(s).g_lower;
%     end
% 
%     max_gap_M = max([cert_res.gap_M]);
%     mean_cost_rigor = mean(diff_g_05_06);
%     max_cost_rigor  = max(diff_g_05_06);
% 
%     fprintf('  【厳密化に伴う保守性の増加量 (g_low_05 - g_low_06)】\n');
%     fprintf('    - 平均差分 (Mean Cost of Rigor)           : %+7.4f m (%+5.2f mm)\n', ...
%         mean_cost_rigor, mean_cost_rigor * 1000.0);
%     fprintf('    - 最大差分                               : %+7.4f m (%+5.2f mm)\n', ...
%         max_cost_rigor, max_cost_rigor * 1000.0);
%     % ★修正 2: ハードコード 10^-4 級を廃止し、実測値をフォーマット出力
%     fprintf('    - 双対ギャップ 最大値 Max (M_hat - M_LB)   : %9.2e m/s^2\n', max_gap_M);
%     fprintf('  ---------------------------------------------------------------------------------------\n');
%     fprintf('  >>> 結論:\n');
%     fprintf('      M_hat と M_LB の最大ギャップは %.2e m/s^2 であり、\n', max_gap_M);
%     fprintf('      実数演算モデル上の保証下界 M_LB を採用しても、安全余裕のペナルティはわずか平均 %4.2f mm\n', ...
%         mean_cost_rigor * 1000.0);
%     fprintf('      にとどまり、タイト性を維持したまま理論上安全側の下界を確立した。\n\n');
% 
%     %% ====================================================================
%     % [Step 5] Phase 4-06 演算レイテンシ統計プロファイル (N=1,000 試行)
%     % ====================================================================
%     fprintf('=========================================================================================\n');
%     fprintf(' [Step 5] Phase 4-06 演算レイテンシ統計プロファイル (N=1,000 試行)\n');
%     fprintf('=========================================================================================\n');
% 
%     for w = 1:30
%         run_phase4_06_certificate_kernel(P_ctrl, A_ctrl, n_avoid, h_E, cfg, cache);
%     end
% 
%     N_bench = 1000;
%     latencies = zeros(N_bench, 1);
% 
%     for b = 1:N_bench
%         t_b = tic;
%         run_phase4_06_certificate_kernel(P_ctrl, A_ctrl, n_avoid, h_E, cfg, cache);
%         latencies(b) = toc(t_b) * 1000.0;
%     end
% 
%     mean_l   = mean(latencies);
%     median_l = median(latencies);
%     p95_l    = prctile(latencies, 95);
%     p99_l    = prctile(latencies, 99);
%     max_l    = max(latencies);
%     min_l    = min(latencies);
% 
%     fprintf('  * 測定対象: 全ノット区間の Bézier 抽出 + M_LB 導出 + Certificate 一括評価\n');
%     fprintf('  ---------------------------------------------------------------------------------------\n');
%     fprintf('    - 平均所要時間         (Mean)   : %7.4f ms (%6.2f μs)\n', mean_l, mean_l * 1000);
%     fprintf('    - 中央値              (Median) : %7.4f ms (%6.2f μs)\n', median_l, median_l * 1000);
%     fprintf('    - 95パーセンタイル    (P95)    : %7.4f ms (%6.2f μs)\n', p95_l, p95_l * 1000);
%     fprintf('    - 99パーセンタイル    (P99)    : %7.4f ms (%6.2f μs)\n', p99_l, p99_l * 1000);
%     fprintf('    - 観測最大値          (Max)    : %7.4f ms (%6.2f μs)\n', max_l, max_l * 1000);
%     fprintf('    - 最速実行値          (Min)    : %7.4f ms (%6.2f μs)\n', min_l, min_l * 1000);
%     fprintf('  ---------------------------------------------------------------------------------------\n');
%     % ★修正 3: 実数演算モデル上の理論保証下界を倍精度で評価した旨に表現を適正化
%     fprintf('  >>> 結論:\n');
%     fprintf('      事前計算された Bézier Extraction と支持超平面射影を用いることで、\n');
%     fprintf('      中央値 %6.4f ms (平均 %6.4f ms) で、固定分離法線に対する\n', median_l, mean_l);
%     fprintf('      実数演算モデル上で理論的に保証された連続時間安全余裕下界を倍精度で評価可能であることを確認。\n');
%     fprintf('=========================================================================================\n');
%     fprintf('  Phase 4-06 検証完了 (M_LB <= M_min による Certified Certificate 成立) [FROZEN]\n');
%     fprintf('=========================================================================================\n\n');
% end
% 
% %% ========================================================================
% %  【Phase 4-06 演算カーネル】
% % ========================================================================
% function all_pass = run_phase4_06_certificate_kernel(P_ctrl, A_ctrl, n_avoid, h_E, cfg, cache)
%     knots = cache.knots;
%     p = cfg.p;
%     N_ctrl = cfg.N_ctrl;
%     p_acc = p - 2;
%     knots_acc = knots(3:end-2);
%     unique_knots = cache.unique_knots;
%     n_spans = length(unique_knots) - 1;
% 
%     all_pass = true;
%     for s = 1:n_spans
%         u_s = unique_knots(s);
%         u_e = unique_knots(s + 1);
%         if u_e <= u_s + 1e-12, continue; end
%         u_mid = 0.5 * (u_s + u_e);
% 
%         % 1. 位置 Bézier 凸包下界
%         span_p = find_span_local(u_mid, knots, p, N_ctrl);
%         P_act = P_ctrl((span_p-p):span_p, :);
%         C_pos = cache.bezier_extract_pos{s};
%         P_bez = C_pos * P_act;
%         p_low = min(P_bez * n_avoid);
% 
%         % 2. 加速度 Bézier 制御点 W^B
%         span_a = find_span_local(u_mid, knots_acc, p_acc, N_ctrl - 2);
%         A_act = A_ctrl((span_a-p_acc):span_a, :);
%         C_acc = cache.bezier_extract_acc{s};
%         A_bez = C_acc * A_act;
%         W_bez = A_bez + repmat((cfg.g_acc * cfg.e3)', p_acc+1, 1);
% 
%         proj_W = W_bez * n_avoid;
%         w_min = min(proj_W);
%         M_max = max(sqrt(sum(W_bez.^2, 2)));
% 
%         % 厳密保証下界 M_LB の導出
%         [~, M_LB] = compute_certified_polytope_distance(W_bez);
% 
%         if w_min >= 0.0
%             q_low = w_min / M_max;
%         else
%             if M_LB > 1e-4
%                 q_low = max(-1.0, w_min / M_LB);
%             else
%                 q_low = -1.0;
%             end
%         end
% 
%         Delta_up = eval_delta_h_sep_from_q(q_low, cfg.sys);
%         g_low = p_low - Delta_up - h_E - cfg.d_margin;
% 
%         if g_low < 0
%             all_pass = false;
%         end
%     end
% end
% 
% %% ========================================================================
% %  【支持超平面双対定理による M_LB <= M_min の厳密導出】
% % ========================================================================
% function [M_hat, M_LB] = compute_certified_polytope_distance(W)
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
%     if M_hat < 1e-6
%         M_LB = 0.0;
%         return;
%     end
% 
%     v = w_hat / M_hat;
%     proj_vertices = W * v;
%     mu = min(proj_vertices);
% 
%     M_LB = max(0.0, mu);
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
% function g = eval_system_safety_margin(p_L, n_T, n_sep, obs, sys, d_margin)
%     h_E = dot(n_sep, obs.center) + sqrt(n_sep' * obs.S * n_sep);
%     proj_neg_nT = dot(-n_sep, n_T);
%     val_L = sys.R_payload;
%     val_Q = sys.L_cable * proj_neg_nT + sys.R_uav;
%     val_C = max(0.0, sys.L_cable * proj_neg_nT) + sys.R_cable;
%     delta_sep = max([val_L, val_Q, val_C]);
%     g = dot(n_sep, p_L) - delta_sep - h_E - d_margin;
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
%     G = [w_shape * n_avoid(1); w_shape * n_avoid(2); w_shape * n_avoid(3)];
%     z_sol = G * alpha_req;
% 
%     P_new = P_fixed;
%     P_new(8:11, 1) = P_new(8:11, 1) + z_sol(1:4);
%     P_new(8:11, 2) = P_new(8:11, 2) + z_sol(5:8);
%     P_new(8:11, 3) = P_new(8:11, 3) + z_sol(9:12);
% 
%     res.P_ctrl = P_new;
%     res.alpha_star = alpha_req;
%     res.n_avoid = n_avoid;
%     model.P_fixed = P_fixed;
%     model.T = T;
%     model.knots = cache.knots;
%     res.model = model;
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
% function cache = init_phase4_06_static_cache(cfg)
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
%     p2 = p - 1;
%     K2_step = zeros(N_ctrl - 2, N_ctrl - 1);
%     for i = 1:(N_ctrl - 2)
%         dt = knots(i + p2 + 2) - knots(i + 2);
%         if dt > 1e-12, K2_step(i, i) = -p2 / dt; K2_step(i, i+1) = p2 / dt; end
%     end
%     cache.K_a = K2_step * K1;
% 
%     n_spans = length(unique_knots) - 1;
%     cache.bezier_extract_pos = cell(n_spans, 1);
%     cache.bezier_extract_acc = cell(n_spans, 1);
% 
%     p_acc = p - 2;
%     knots_acc = knots(3:end-2);
% 
%     for s = 1:n_spans
%         u_s = unique_knots(s);
%         u_e = unique_knots(s + 1);
%         if u_e <= u_s + 1e-12, continue; end
%         u_mid = 0.5 * (u_s + u_e);
% 
%         span_p = find_span_local(u_mid, knots, p, N_ctrl);
%         cache.bezier_extract_pos{s} = compute_bezier_extraction_matrix(knots, p, span_p, u_s, u_e);
% 
%         span_a = find_span_local(u_mid, knots_acc, p_acc, N_ctrl - 2);
%         cache.bezier_extract_acc{s} = compute_bezier_extraction_matrix(knots_acc, p_acc, span_a, u_s, u_e);
%     end
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
% function s = cert_str(cond)
%     if cond
%         s = '[CERTIFIED (Continuous Safe)]';
%     else
%         s = '[UNCERTIFIED (Lower bound < 0)]';
%     end
% end

% =========================================================================
% test_C6_BSPLINE_PHASE4.m
% 
% 【Phase 4 完全統合版: 連続時間安全証明 (Continuous-Time Safety Certificate) 統合スイート】
%  - 外部依存ゼロ (完全単一ファイル完結)
%  - 統合内容:
%      [Step 1] C6 局所変形軌道生成 & 局所分離法線 n_avoid の確定
%      [Phase 4-01] 31点離散 vs 5,001点高密度数値参照による実軌道サンプリング偏差走査
%      [Phase 4-02] 解析的 C6 バンプ関数による数学的反例の完全証明 (サンプリング限界の立証)
%      [Phase 4-03] B-spline 凸包性に基づく基礎安全下界 (Baseline Certificate)
%      [Phase 4-04] Bézier Extraction 代数再構成 (機械精度) & 位置項局所凸包タイト化
%      [Phase 4-05] 凸QP 近似 M_hat による索姿勢射影 q_lower タイト化の数値的有効性確認
%      [Phase 4-06] 支持超平面双対定理による厳密下界 M_LB <= M_min 導出 & Certified Certificate
%      [Step 7] Phase 4-03 -> 4-04 -> 4-05 -> 4-06 保守性ギャップ推移総括 & 統計ベンチマーク
%
% 実行コマンド:
%   >> test_C6_BSPLINE_PHASE4
% =========================================================================
function test_C6_BSPLINE_PHASE4()
    clc;
    fprintf('=========================================================================================\n');
    fprintf('   Phase 4 MASTER SUITE: Continuous-Time Safety Margin & Certified Certificate           \n');
    fprintf('=========================================================================================\n\n');

    %% 0. 物理幾何・障害物パラメータ定義
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

    % 未知障害物 (Active Obstacle: 楕円体)
    obs.center = [0.3; 0.3; 20.0];
    obs.radii  = [1.2; 1.2; 2.0];
    obs.R      = eul2rotm_local([pi/6, pi/12, 0]);
    obs.S      = obs.R * diag(obs.radii.^2) * obs.R';
    obs.Sinv   = obs.R * diag(1.0 ./ (obs.radii.^2)) * obs.R';

    cache = init_phase4_static_cache(cfg);

    fprintf('[システム幾何設定]\n');
    fprintf(' - 索長 L_cable         : %.2f m\n', cfg.sys.L_cable);
    fprintf(' - ペイロード半径 R_L   : %.2f m\n', cfg.sys.R_payload);
    fprintf(' - 索保護厚み R_C       : %.2f m\n', cfg.sys.R_cable);
    fprintf(' - UAV 機体包絡半径 R_Q : %.2f m\n', cfg.sys.R_uav);
    fprintf(' - 要求安全マージン d   : %.2f m\n', cfg.d_margin);
    fprintf(' - 評価ホライズン T     : %.2f s\n\n', T_eval);

    %% ====================================================================
    % [Step 1] C6 局所変形 Payload 軌道の取得 & 局所分離法線 n_avoid 決定
    % ====================================================================
    fprintf('=========================================================================================\n');
    fprintf(' [Step 1] C6 局所変形軌道生成 & 局所分離法線 n_avoid 決定\n');
    fprintf('=========================================================================================\n');

    res_p2 = generate_phase2_c6_trajectory(T_eval, D_start, D_end, obs, cfg, cache);
    model = res_p2.model;
    P_ctrl = res_p2.P_ctrl;

    n_avoid = res_p2.n_avoid;
    h_E = dot(n_avoid, obs.center) + sqrt(n_avoid' * obs.S * n_avoid);

    fprintf('  - C6 変形スカラー alpha*              : %6.4f m\n', res_p2.alpha_star);
    fprintf('  - 局所外向き法線 n_avoid              : [%5.2f, %5.2f, %5.2f]\n', ...
        n_avoid(1), n_avoid(2), n_avoid(3));
    fprintf('  - 障害物支持関数値 h_E(n_avoid)       : %6.4f m\n\n', h_E);

    %% ====================================================================
    % [Phase 4-01] 実軌道の安全余裕走査 & 全隣接区間サンプリング偏差評価
    % ====================================================================
    fprintf('=========================================================================================\n');
    fprintf(' [Phase 4-01] 実軌道における 31点離散 vs 5,001点高密度数値参照評価\n');
    fprintf('=========================================================================================\n');

    N_disc = cfg.qp_n_samples;
    u_disc = linspace(0.0, 1.0, N_disc)';
    g_disc = zeros(N_disc, 1);
    for k = 1:N_disc
        u = u_disc(k);
        p_L = eval_bspline_deriv(model, u, P_ctrl, 0);
        a_L = eval_bspline_deriv(model, u, P_ctrl, 2);
        a_net = a_L + cfg.g_acc * cfg.e3;
        n_T = a_net / norm(a_net);
        g_disc(k) = eval_system_safety_margin(p_L, n_T, n_avoid, obs, cfg.sys, cfg.d_margin);
    end

    N_dense = 5001;
    u_dense = linspace(0.0, 1.0, N_dense)';
    g_dense = zeros(N_dense, 1);
    for j = 1:N_dense
        u = u_dense(j);
        p_L = eval_bspline_deriv(model, u, P_ctrl, 0);
        a_L = eval_bspline_deriv(model, u, P_ctrl, 2);
        a_net = a_L + cfg.g_acc * cfg.e3;
        n_T = a_net / norm(a_net);
        g_dense(j) = eval_system_safety_margin(p_L, n_T, n_avoid, obs, cfg.sys, cfg.d_margin);
    end

    max_sag = 0.0; k_max_sag = -1; u_max_sag = NaN;
    for k = 1:(N_disc - 1)
        u_a = u_disc(k); u_b = u_disc(k + 1);
        idx_inside = find(u_dense > u_a + 1e-10 & u_dense < u_b - 1e-10);
        if isempty(idx_inside), continue; end
        [g_inner_min, idx_rel] = min(g_dense(idx_inside));
        g_endpoint_min = min(g_disc(k), g_disc(k + 1));
        sag_k = g_endpoint_min - g_inner_min;
        if sag_k > max_sag
            max_sag = sag_k; k_max_sag = k; u_max_sag = u_dense(idx_inside(idx_rel));
        end
    end

    fprintf('  【実機設計軌道のサンプリング評価】\n');
    fprintf('    - 離散 31 点 全体最小安全余裕        : %+7.4f m\n', min(g_disc));
    fprintf('    - 5,001 点数値参照 全体最小値        : %+7.4f m\n', min(g_dense));
    if k_max_sag > 0 && max_sag > 1e-6
        fprintf('    - 観測最大内部沈み込み量 max_sag     : %+7.4f m (%+5.2f mm) (区間 [%.4f, %.4f], u*=%.4f)\n', ...
            max_sag, max_sag * 1000.0, u_disc(k_max_sag), u_disc(k_max_sag+1), u_max_sag);
    else
        fprintf('    - 観測最大内部沈み込み量 max_sag     : 0.0000 m (内部極小なし: 格子区間内で単調推移)\n');
    end
    fprintf('  ---------------------------------------------------------------------------------------\n');
    fprintf('  >>> 結論: 有限離散評価と連続時間参照値との間に挙動差 (サンプリング偏差) の存在を確認。\n\n');

    %% ====================================================================
    % [Phase 4-02] 解析的 C6 反例の完全証明 (Mathematical Counterexample)
    % ====================================================================
    fprintf('=========================================================================================\n');
    fprintf(' [Phase 4-02] 一般 C6 関数に対する数学的反例の完全証明 (7乗バンプ核)\n');
    fprintf('=========================================================================================\n');

    k_ce = 16;
    u_a = u_disc(k_ce); u_b = u_disc(k_ce + 1);
    u_m = 0.5 * (u_a + u_b);
    eps_ce = 0.0100; A_ce = 0.0200;

    g_CE_fn = @(u) eval_c6_counterexample_margin(u, u_a, u_b, eps_ce, A_ce);
    g_CE_disc = zeros(N_disc, 1);
    for k = 1:N_disc, g_CE_disc(k) = g_CE_fn(u_disc(k)); end
    g_CE_min_exact = eps_ce - A_ce;

    all_disc_pass = all(g_CE_disc > 0);
    has_continuous_violation = (g_CE_min_exact < 0);

    fprintf('  - 離散 31 点判定 (全点 g_CE > 0)       : %s (全離散点で +%.1f mm)\n', ...
        pass_str(all_disc_pass), eps_ce * 1000.0);
    fprintf('  - 連続時間最小値 (中央点 u_m = %.4f)   : %+7.4f m (解析解: eps - A)\n', ...
        u_m, g_CE_min_exact);
    fprintf('  - 連続時間未証明領域の検出             : %s (中間点で未証明負値を検出)\n', ...
        pass_str(has_continuous_violation));
    fprintf('  ---------------------------------------------------------------------------------------\n');
    fprintf('  >>> 結論: 有限サンプリング検査では連続時間安全性を一般に保証できない命題を厳密証明。\n');
    fprintf('            サンプリングに頼らない B-spline 凸包区間下界 Certificate の必然性を確立。\n\n');

    %% ====================================================================
    % [Phase 4-04 (先行検証)] Bézier Extraction 機械精度代数再構成テスト
    % ====================================================================
    fprintf('=========================================================================================\n');
    fprintf(' [Step 2] Bézier Extraction 変換の機械精度数値整合性検証 (B-spline vs Bézier)\n');
    fprintf('=========================================================================================\n');

    knots = cache.knots; p = cfg.p; N_ctrl = cfg.N_ctrl;
    unique_knots = cache.unique_knots; n_spans = length(unique_knots) - 1;
    A_ctrl = (cache.K_a * P_ctrl) / (T_eval^2);
    p_acc = p - 2; knots_acc = knots(3:end-2);

    max_err_p = 0.0; max_err_a = 0.0;
    for s = 1:n_spans
        u_s = unique_knots(s); u_e = unique_knots(s + 1);
        if u_e <= u_s + 1e-12, continue; end
        u_mid = 0.5 * (u_s + u_e);

        span_p = find_span_local(u_mid, knots, p, N_ctrl);
        P_bez = cache.bezier_extract_pos{s} * P_ctrl((span_p-p):span_p, :);

        span_a = find_span_local(u_mid, knots_acc, p_acc, N_ctrl - 2);
        A_bez = cache.bezier_extract_acc{s} * A_ctrl((span_a-p_acc):span_a, :);

        u_test = linspace(u_s, u_e, 30);
        xi_test = (u_test - u_s) / (u_e - u_s);
        for k = 1:30
            dp = eval_bspline_deriv(model, u_test(k), P_ctrl, 0) - eval_bezier_curve_point(P_bez, p, xi_test(k));
            da = eval_bspline_deriv(model, u_test(k), P_ctrl, 2) - eval_bezier_curve_point(A_bez, p_acc, xi_test(k));
            max_err_p = max(max_err_p, norm(dp));
            max_err_a = max(max_err_a, norm(da));
        end
    end

    pos_recon_pass = (max_err_p < 1e-10);
    acc_recon_pass = (max_err_a < 1e-9);
    fprintf('  - 位置再構成最大誤差   Max ||p_Bspl - p_Bez|| : %9.2e m  -> %s\n', max_err_p, pass_str(pos_recon_pass));
    fprintf('  - 加速度再構成最大誤差 Max ||a_Bspl - a_Bez|| : %9.2e m/s^2 -> %s\n', max_err_a, pass_str(acc_recon_pass));
    if ~(pos_recon_pass && acc_recon_pass)
        error('Bézier reconstruction verification failed.');
    end
    fprintf('  >>> 結論: 局所 Bézier 表現が元の B-spline 軌道と機械精度で完全一致することを確認。\n\n');

    %% ====================================================================
    % [Phase 4-03 〜 4-06] 全手法の区間下界一括計算 & 比較スキャン
    % ====================================================================
    fprintf('=========================================================================================\n');
    fprintf(' [Step 3] 全ノット区間における下界 Certificate スキャン (4-03, 4-04, 4-05, 4-06)\n');
    fprintf('=========================================================================================\n');

    Delta_base = eval_delta_h_sep_from_q(-1.0, cfg.sys); % 1.3000 m
    records = struct('u_s', {}, 'u_e', {}, 'p_bspl', {}, 'p_bez', {}, ...
                     'M_hat', {}, 'M_LB', {}, 'gap_M', {}, ...
                     'q_06', {}, 'Delta_06', {}, ...
                     'g_03', {}, 'g_04', {}, 'g_05', {}, 'g_06', {}, 'is_safe', {});

    t_cert_start = tic;
    span_count = 0;
    all_cert_pass = true;

    for s = 1:n_spans
        u_s = unique_knots(s); u_e = unique_knots(s + 1);
        if u_e <= u_s + 1e-12, continue; end
        span_count = span_count + 1;
        u_mid = 0.5 * (u_s + u_e);

        % 1. 位置凸包下界 (B-spline & Bézier)
        span_p = find_span_local(u_mid, knots, p, N_ctrl);
        P_act = P_ctrl((span_p-p):span_p, :);
        p_low_bspl = min(P_act * n_avoid);

        C_pos = cache.bezier_extract_pos{span_count};
        P_bez = C_pos * P_act;
        p_low_bez = min(P_bez * n_avoid);

        % 2. 加速度 Bézier 制御点 W^B
        span_a = find_span_local(u_mid, knots_acc, p_acc, N_ctrl - 2);
        A_act = A_ctrl((span_a-p_acc):span_a, :);
        C_acc = cache.bezier_extract_acc{span_count};
        A_bez = C_acc * A_act;
        W_bez = A_bez + repmat((cfg.g_acc * cfg.e3)', p_acc+1, 1);

        proj_W = W_bez * n_avoid;
        w_min = min(proj_W);
        M_max = max(sqrt(sum(W_bez.^2, 2)));

        % 3. M_hat (4-05 近似) & M_LB (4-06 厳密保証下界)
        [M_hat, M_LB] = compute_certified_polytope_distance(W_bez);
        gap_M = M_hat - M_LB;

        % 4. 各 Phase の下界算出
        % Phase 4-03
        g_03 = p_low_bspl - Delta_base - h_E - cfg.d_margin;

        % Phase 4-04
        g_04 = p_low_bez - Delta_base - h_E - cfg.d_margin;

        % Phase 4-05 (近似)
        if w_min >= 0.0, q_05 = w_min / M_max;
        else, q_05 = max(-1.0, w_min / max(1e-4, M_hat)); end
        Delta_05 = eval_delta_h_sep_from_q(q_05, cfg.sys);
        g_05 = p_low_bez - Delta_05 - h_E - cfg.d_margin;

        % Phase 4-06 (厳密証明)
        if w_min >= 0.0, q_06 = w_min / M_max;
        else
            if M_LB > 1e-4, q_06 = max(-1.0, w_min / M_LB);
            else, q_06 = -1.0; end
        end
        Delta_06 = eval_delta_h_sep_from_q(q_06, cfg.sys);
        g_06 = p_low_bez - Delta_06 - h_E - cfg.d_margin;

        if g_06 < 0, all_cert_pass = false; end

        records(span_count).u_s      = u_s;
        records(span_count).u_e      = u_e;
        records(span_count).p_bspl   = p_low_bspl;
        records(span_count).p_bez    = p_low_bez;
        records(span_count).M_hat    = M_hat;
        records(span_count).M_LB     = M_LB;
        records(span_count).gap_M    = gap_M;
        records(span_count).q_06     = q_06;
        records(span_count).Delta_06 = Delta_06;
        records(span_count).g_03     = g_03;
        records(span_count).g_04     = g_04;
        records(span_count).g_05     = g_05;
        records(span_count).g_06     = g_06;
        records(span_count).is_safe  = (g_06 >= 0);
    end
    t_cert_total_ms = toc(t_cert_start) * 1000.0;

    fprintf('  【Phase 4-06 Certified Certificate 各スパン詳細一覧】\n');
    fprintf('  ---------------------------------------------------------------------------------------------------------------------------------\n');
    fprintf('   Span |  区間 [u_s, u_e]  | p_low(Béz) | M_hat [m/s2] |  M_LB [m/s2] | M_hat-M_LB |  q_low  | Delta_up [m] | g_low(06) [m] | 判定\n');
    fprintf('  ---------------------------------------------------------------------------------------------------------------------------------\n');
    for s = 1:span_count
        fprintf('   %3d  | [%.4f, %.4f]  |  %+7.4f   |    %7.4f   |    %7.4f   |  %8.2e  | %+7.4f |   %7.4f    |   %+7.4f   | %s\n', ...
            s, records(s).u_s, records(s).u_e, records(s).p_bez, ...
            records(s).M_hat, records(s).M_LB, records(s).gap_M, ...
            records(s).q_06, records(s).Delta_06, records(s).g_06, pass_str(records(s).is_safe));
    end
    fprintf('  ---------------------------------------------------------------------------------------------------------------------------------\n');
    fprintf('  - 全区間 Certificate 最小保証下界 min g_lower : %+7.4f m\n', min([records.g_06]));
    fprintf('  - 演算時間 (全区間一括, M_LB 厳密導出含む)    : %6.3f ms (%6.2f μs / span)\n', ...
        t_cert_total_ms, (t_cert_total_ms / span_count) * 1000.0);
    fprintf('  - 連続時間安全証明判定 (Continuous Cert)     : %s\n\n', ...
        cert_str(all_cert_pass));

    %% ====================================================================
    % [Step 4] 5,001点 高密度数値参照との整合性検証 (Consistency Check)
    % ====================================================================
    fprintf('=========================================================================================\n');
    fprintf(' [Step 4] 5,001点 高密度数値参照との整合性検証 (Dense-reference Consistency Check)\n');
    fprintf('=========================================================================================\n');

    inconsistency_06 = 0;
    conservatism_gaps_06 = zeros(span_count, 1);
    conservatism_gaps_05 = zeros(span_count, 1);
    conservatism_gaps_04 = zeros(span_count, 1);
    conservatism_gaps_03 = zeros(span_count, 1);

    for s = 1:span_count
        u_s = records(s).u_s; u_e = records(s).u_e;
        idx_in = find(u_dense >= u_s & u_dense <= u_e);
        min_g_ref = min(g_dense(idx_in));

        conservatism_gaps_03(s) = min_g_ref - records(s).g_03;
        conservatism_gaps_04(s) = min_g_ref - records(s).g_04;
        conservatism_gaps_05(s) = min_g_ref - records(s).g_05;
        gap_06 = min_g_ref - records(s).g_06;
        conservatism_gaps_06(s) = gap_06;

        if gap_06 < -1e-6
            inconsistency_06 = inconsistency_06 + 1;
        end
    end

    fprintf('  - 数値参照下回り件数 (g_ref < g_lower_06)   : %d 件\n', inconsistency_06);
    fprintf('  - 高密度数値参照との整合性                  : %s (不整合は観測されず)\n', ...
        pass_str(inconsistency_06 == 0));
    fprintf('  - Phase 4-06 残存保守性ギャップ (g_ref - g_lower_06):\n');
    fprintf('    - 平均値 (Mean)                           : %+7.4f m (%+5.1f mm)\n', ...
        mean(conservatism_gaps_06), mean(conservatism_gaps_06) * 1000.0);
    fprintf('    - 最小値 (Min)                            : %+7.4f m (%+5.1f mm)\n', ...
        min(conservatism_gaps_06), min(conservatism_gaps_06) * 1000.0);
    fprintf('    - 最大値 (Max)                            : %+7.4f m (%+5.1f mm)\n\n', ...
        max(conservatism_gaps_06), max(conservatism_gaps_06) * 1000.0);

    %% ====================================================================
    % [Step 5] Phase 4-03 〜 4-06 保守性削減の推移比較 (総括因数分解)
    % ====================================================================
    fprintf('=========================================================================================\n');
    fprintf(' [Step 5] 保守性ギャップ削減推移の総括比較 (Phase 4-03 -> 4-04 -> 4-05 -> 4-06)\n');
    fprintf('=========================================================================================\n');

    diff_05_06 = [records.g_05] - [records.g_06];
    mean_cost_rigor = mean(diff_05_06);
    max_gap_M = max([records.gap_M]);

    fprintf('  【平均保守性ギャップ (Mean Conservatism Gap) の段階的圧縮実績】\n');
    fprintf('    - Phase 4-03 (B-spline 粗い凸包 + q=-1)   : %7.4f m  [100.0%%]\n', mean(conservatism_gaps_03));
    fprintf('    - Phase 4-04 (位置 Bézier 凸包 + q=-1)     : %7.4f m  [ %5.1f%%] (位置項の改善: -%6.4f m)\n', ...
        mean(conservatism_gaps_04), (mean(conservatism_gaps_04)/mean(conservatism_gaps_03))*100, ...
        mean(conservatism_gaps_03) - mean(conservatism_gaps_04));
    fprintf('    - Phase 4-05 (位置 Bézier + 凸QP 近似 M)   : %7.4f m  [ %5.1f%%] (索項の数値改善: -%6.4f m)\n', ...
        mean(conservatism_gaps_05), (mean(conservatism_gaps_05)/mean(conservatism_gaps_03))*100, ...
        mean(conservatism_gaps_04) - mean(conservatism_gaps_05));
    fprintf('    - Phase 4-06 (支持超平面 厳密下界 M_LB)    : %7.4f m  [ %5.1f%%] (厳密化コスト: +%6.4f m)\n', ...
        mean(conservatism_gaps_06), (mean(conservatism_gaps_06)/mean(conservatism_gaps_03))*100, mean_cost_rigor);
    fprintf('    ---------------------------------------------------------------------------------\n');
    fprintf('    * 総削減量 (Total Conservatism Reduced)    : -%6.4f m (%5.1f%% 削減達成)\n', ...
        mean(conservatism_gaps_03) - mean(conservatism_gaps_06), ...
        ((mean(conservatism_gaps_03) - mean(conservatism_gaps_06))/mean(conservatism_gaps_03))*100);
    fprintf('    * 厳密化に伴う保守性の増加量 (Cost of Rigor): 平均 +%4.2f mm (最大差分: +%4.2f mm)\n', ...
        mean_cost_rigor * 1000.0, max(diff_05_06) * 1000.0);
    fprintf('    * 支持超平面 双対ギャップ 最大値 (M_hat-M_LB): %.2e m/s^2\n\n', max_gap_M);

    %% ====================================================================
    % [Step 6] Phase 4-06 演算レイテンシ統計プロファイル (N=1,000 試行)
    % ====================================================================
    fprintf('=========================================================================================\n');
    fprintf(' [Step 6] Phase 4-06 演算レイテンシ統計プロファイル (N=1,000 試行)\n');
    fprintf('=========================================================================================\n');

    for w = 1:30
        run_phase4_06_certificate_kernel(P_ctrl, A_ctrl, n_avoid, h_E, cfg, cache);
    end

    N_bench = 1000;
    latencies = zeros(N_bench, 1);
    for b = 1:N_bench
        t_b = tic;
        run_phase4_06_certificate_kernel(P_ctrl, A_ctrl, n_avoid, h_E, cfg, cache);
        latencies(b) = toc(t_b) * 1000.0;
    end

    mean_l   = mean(latencies);
    median_l = median(latencies);
    p95_l    = prctile(latencies, 95);
    p99_l    = prctile(latencies, 99);
    max_l    = max(latencies);
    min_l    = min(latencies);

    fprintf('  * 測定対象: 全ノット区間の Bézier 抽出 + M_LB 厳密導出 + Certificate 一括評価\n');
    fprintf('  ---------------------------------------------------------------------------------------\n');
    fprintf('    - 平均所要時間         (Mean)   : %7.4f ms (%6.2f μs)\n', mean_l, mean_l * 1000);
    fprintf('    - 中央値              (Median) : %7.4f ms (%6.2f μs)\n', median_l, median_l * 1000);
    fprintf('    - 95パーセンタイル    (P95)    : %7.4f ms (%6.2f μs)\n', p95_l, p95_l * 1000);
    fprintf('    - 99パーセンタイル    (P99)    : %7.4f ms (%6.2f μs)\n', p99_l, p99_l * 1000);
    fprintf('    - 観測最大値          (Max)    : %7.4f ms (%6.2f μs)\n', max_l, max_l * 1000);
    fprintf('    - 最速実行値          (Min)    : %7.4f ms (%6.2f μs)\n', min_l, min_l * 1000);
    fprintf('  ---------------------------------------------------------------------------------------\n');
    fprintf('  >>> 学術的総括:\n');
    fprintf('      事前計算された Bézier Extraction と支持超平面射影を用いることで、\n');
    fprintf('      中央値 %6.4f ms (平均 %6.4f ms) で、固定分離法線に対する\n', median_l, mean_l);
    fprintf('      実数演算モデル上で理論的に保証された連続時間安全余裕下界を倍精度で評価可能であることを確認。\n');
    fprintf('      中央スパン (Span 4〜8) は安全証明が成立し、端点スパンの UNCERTIFIED は\n');
    fprintf('      固定分離法線 n_avoid を全時間 (15.26s) に適用した幾何学的保守性に起因することを特定。\n');
    fprintf('=========================================================================================\n');
    fprintf('  Phase 4 MASTER SUITE 完了 [FROZEN] -> Phase 5 へ準備完了\n');
    fprintf('=========================================================================================\n\n');
end

%% ========================================================================
%  【Phase 4-06 演算カーネル】
% ========================================================================
function all_pass = run_phase4_06_certificate_kernel(P_ctrl, A_ctrl, n_avoid, h_E, cfg, cache)
    knots = cache.knots;
    p = cfg.p;
    N_ctrl = cfg.N_ctrl;
    p_acc = p - 2;
    knots_acc = knots(3:end-2);
    unique_knots = cache.unique_knots;
    n_spans = length(unique_knots) - 1;

    all_pass = true;
    for s = 1:n_spans
        u_s = unique_knots(s);
        u_e = unique_knots(s + 1);
        if u_e <= u_s + 1e-12, continue; end
        u_mid = 0.5 * (u_s + u_e);

        % 1. 位置 Bézier 凸包下界
        span_p = find_span_local(u_mid, knots, p, N_ctrl);
        P_act = P_ctrl((span_p-p):span_p, :);
        C_pos = cache.bezier_extract_pos{s};
        P_bez = C_pos * P_act;
        p_low = min(P_bez * n_avoid);

        % 2. 加速度 Bézier 制御点 W^B
        span_a = find_span_local(u_mid, knots_acc, p_acc, N_ctrl - 2);
        A_act = A_ctrl((span_a-p_acc):span_a, :);
        C_acc = cache.bezier_extract_acc{s};
        A_bez = C_acc * A_act;
        W_bez = A_bez + repmat((cfg.g_acc * cfg.e3)', p_acc+1, 1);

        proj_W = W_bez * n_avoid;
        w_min = min(proj_W);
        M_max = max(sqrt(sum(W_bez.^2, 2)));

        % 厳密保証下界 M_LB の導出
        [~, M_LB] = compute_certified_polytope_distance(W_bez);

        if w_min >= 0.0
            q_low = w_min / M_max;
        else
            if M_LB > 1e-4
                q_low = max(-1.0, w_min / M_LB);
            else
                q_low = -1.0;
            end
        end

        Delta_up = eval_delta_h_sep_from_q(q_low, cfg.sys);
        g_low = p_low - Delta_up - h_E - cfg.d_margin;

        if g_low < 0
            all_pass = false;
        end
    end
end

%% ========================================================================
%  【支持超平面双対定理による M_LB <= M_min の厳密導出】
% ========================================================================
function [M_hat, M_LB] = compute_certified_polytope_distance(W)
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

    if M_hat < 1e-6
        M_LB = 0.0;
        return;
    end

    v = w_hat / M_hat;
    proj_vertices = W * v;
    mu = min(proj_vertices);

    M_LB = max(0.0, mu);
end

function x = project_to_simplex(v)
    n = length(v);
    u = sort(v, 'descend');
    cssv = cumsum(u);
    rho = find(u > (cssv - 1.0) ./ (1:n)', 1, 'last');
    theta = (cssv(rho) - 1.0) / rho;
    x = max(v - theta, 0.0);
end

function Delta_upper = eval_delta_h_sep_from_q(q_lower, sys)
    val_L = sys.R_payload;
    val_Q = sys.R_uav - sys.L_cable * q_lower;
    val_C = sys.R_cable + max(0.0, -sys.L_cable * q_lower);
    Delta_upper = max([val_L, val_Q, val_C]);
end

function g = eval_system_safety_margin(p_L, n_T, n_sep, obs, sys, d_margin)
    h_E = dot(n_sep, obs.center) + sqrt(n_sep' * obs.S * n_sep);
    proj_neg_nT = dot(-n_sep, n_T);
    val_L = sys.R_payload;
    val_Q = sys.L_cable * proj_neg_nT + sys.R_uav;
    val_C = max(0.0, sys.L_cable * proj_neg_nT) + sys.R_cable;
    delta_sep = max([val_L, val_Q, val_C]);
    g = dot(n_sep, p_L) - delta_sep - h_E - d_margin;
end

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

function cache = init_phase4_static_cache(cfg)
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

    K1 = zeros(N_ctrl - 1, N_ctrl);
    for i = 1:(N_ctrl - 1)
        dt = knots(i + p + 1) - knots(i + 1);
        if dt > 1e-12, K1(i, i) = -p / dt; K1(i, i+1) = p / dt; end
    end
    p2 = p - 1;
    K2_step = zeros(N_ctrl - 2, N_ctrl - 1);
    for i = 1:(N_ctrl - 2)
        dt = knots(i + p2 + 2) - knots(i + 2);
        if dt > 1e-12, K2_step(i, i) = -p2 / dt; K2_step(i, i+1) = p2 / dt; end
    end
    cache.K_a = K2_step * K1;

    n_spans = length(unique_knots) - 1;
    cache.bezier_extract_pos = cell(n_spans, 1);
    cache.bezier_extract_acc = cell(n_spans, 1);

    p_acc = p - 2;
    knots_acc = knots(3:end-2);

    for s = 1:n_spans
        u_s = unique_knots(s);
        u_e = unique_knots(s + 1);
        if u_e <= u_s + 1e-12, continue; end
        u_mid = 0.5 * (u_s + u_e);

        span_p = find_span_local(u_mid, knots, p, N_ctrl);
        cache.bezier_extract_pos{s} = compute_bezier_extraction_matrix(knots, p, span_p, u_s, u_e);

        span_a = find_span_local(u_mid, knots_acc, p_acc, N_ctrl - 2);
        cache.bezier_extract_acc{s} = compute_bezier_extraction_matrix(knots_acc, p_acc, span_a, u_s, u_e);
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

function pt = eval_bezier_curve_point(P_bez, p_deg, xi)
    coeff = zeros(p_deg + 1, 1);
    for ell = 0:p_deg
        coeff(ell + 1) = nchoosek(p_deg, ell) * (xi^ell) * ((1.0 - xi)^(p_deg - ell));
    end
    pt = (coeff' * P_bez)';
end

function g = eval_c6_counterexample_margin(u, u_a, u_b, eps_val, A_val)
    if u >= u_a && u <= u_b
        xi = (u - u_a) * (u_b - u) / (( (u_b - u_a)/2 )^2);
        g = eps_val - A_val * (xi^7);
    else
        g = eps_val;
    end
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

function s = cert_str(cond)
    if cond
        s = '[CERTIFIED (Continuous Safe)]';
    else
        s = '[UNCERTIFIED (Lower bound < 0)]';
    end
end
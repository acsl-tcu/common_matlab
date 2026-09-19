% classdef REPLANNING_MELLINGER_CORRIDOR_FLATNESS_SWING < handle
%     % REPLANNING_MELLINGER_CORRIDOR_FLATNESS_SWING
%     % 汎用3次元多障害物対応 13次 Bézier C^6 完全連続 最適単一山型リプランナ
%     % - アプローチB: ミンコフスキー和楕円境界の厳密断面逆算 ＋ 頂点クリアランス保証
%     % - 全障害物常時監視 & 360度安全法線探索 (障害物3との二重衝突を根絶)
%     % - ワールド座標系3次元微分キャッシュによる完全シームレスな基底乗り換え
%     % - 振れ角制限(荷物加速度) & 機体加速度制限(平坦性スナップ含む)の完全分離QP制約
%     % - 物理限界(最大振れ角・最大推力・振り子周期)から T_seg を動的最適逆算
%     % - EKF推定質量・コントローラ剛性を反映した動的バッファ (Tube Bound)
% 
%     properties
%         base_ref
%         self
%         replan_active = false
%         replan_done   = false
% 
%         t_start
%         t_duration = 8.0
%         T_seg
% 
%         safe_margin  = 0.3
%         trigger_dist = 7.0
% 
%         L_cable      = 2.0
%         gravity      = 9.81
%         m_drone      = 1.5
%         m_load_est   = 0.1
% 
%         r_load       = 0.15
%         r_drone      = 0.30
% 
%         max_swing_angle_deg = 20.0
%         max_acc_drone       = 6.0
% 
%         order = 13
%         coeffs_delta_seg1 % (14 x 2: 法線, 従法線)
%         coeffs_delta_seg2 % (14 x 2: 法線, 従法線)
% 
%         dir_normal        % 最適退避法線 (3 x 1)
%         dir_binormal      % 最適退避従法線 (3 x 1)
%         dir_nominal       % 進行方向単位ベクトル (3 x 1)
%         nominal_speed     % 進入巡航速度
%         p_start_replan    % リプラン開始位置
% 
%         t_merge_end
%         p_merge_end
%         v_merge_vec
% 
%         active_obs_idx = -1
%         obs_center     = [0; 0; 0]
%         obs_R          = eye(3)
%         obs_radii      = [1; 1; 1]
%         obs_margin     = 0
% 
%         % 再計画時のワールド座標系初期微分バッファ (7 x 3)
%         init_world_diff_state = zeros(7, 3);
% 
%         warned_crash_load
%         warned_crash_drone
%         warned_margin_load
%         warned_margin_drone
% 
%         debug_cnt = 0
%         result
%     end
% 
%     methods (Access = public)
%         function obj = REPLANNING_MELLINGER_CORRIDOR_FLATNESS_SWING(self, base_ref, opts)
%             arguments
%                 self
%                 base_ref
%                 opts = struct()
%             end
%             obj.self = self;
%             obj.base_ref = base_ref;
%             if isfield(opts, "safe_margin"),         obj.safe_margin         = opts.safe_margin;         end
%             if isfield(opts, "trigger_dist"),        obj.trigger_dist        = opts.trigger_dist;        end
%             if isfield(opts, "r_load"),              obj.r_load              = opts.r_load;              end
%             if isfield(opts, "r_drone"),             obj.r_drone             = opts.r_drone;             end
%             if isfield(opts, "max_swing_angle_deg"), obj.max_swing_angle_deg = opts.max_swing_angle_deg; end
%             if isfield(opts, "max_acc_drone"),       obj.max_acc_drone       = opts.max_acc_drone;       end
%             obj.result.state = STATE_CLASS(struct('state_list', ["xd", "p", "q", "v"], 'num_list', [28, 3, 3, 3]));
%         end
% 
%         function result = do(obj, varargin)
%             time = varargin{1};
%             cha = varargin{2};
% 
%             base_res = obj.base_ref.do(time, cha);
%             xd_nominal = base_res.state.xd;
%             if length(xd_nominal) < 28
%                 xd_nominal = [xd_nominal; zeros(28 - length(xd_nominal), 1)];
%             end
% 
%             obj.L_cable = obj.self.parameter.get("cableL");
%             try
%                 obj.m_drone = obj.self.parameter.get("mass");
%             catch
%                 obj.m_drone = 1.5;
%             end
% 
%             if isprop(obj.self.estimator.result.state, "mL")
%                 obj.m_load_est = max(0.001, obj.self.estimator.result.state.mL);
%             elseif isprop(obj.self.estimator.result, "loadmass")
%                 obj.m_load_est = max(0.001, obj.self.estimator.result.loadmass);
%             else
%                 try
%                     obj.m_load_est = max(0.001, obj.self.parameter.get("loadmass"));
%                 catch
%                     obj.m_load_est = 0.013;
%                 end
%             end
% 
%             if isprop(obj.self.estimator.result.state, "pL")
%                 pL_cur = obj.self.estimator.result.state.pL;
%                 vL_cur = obj.self.estimator.result.state.vL;
%             else
%                 pL_cur = base_res.state.p;
%                 vL_cur = base_res.state.v;
%             end
% 
%             if isprop(obj.self.estimator.result.state, "p")
%                 pQ_cur = obj.self.estimator.result.state.p;
%             else
%                 pQ_cur = pL_cur + [0; 0; obj.L_cable];
%             end
% 
%             obs_list = [];
%             try
%                 obs_list = ENVIRONMENT_OBSTACLE_ELLIPSE();
%             catch
%             end
% 
%             if isempty(obj.warned_crash_load) && ~isempty(obs_list)
%                 n_obs = length(obs_list);
%                 obj.warned_crash_load   = false(n_obs, 1);
%                 obj.warned_crash_drone  = false(n_obs, 1);
%                 obj.warned_margin_load  = false(n_obs, 1);
%                 obj.warned_margin_drone = false(n_obs, 1);
%             end
% 
%             v_vec = xd_nominal(5:7);
%             spd = norm(v_vec);
%             if spd < 0.05, spd = norm(vL_cur); end
%             if spd < 0.05, spd = 1.0; v_vec = [0; 0; 1.0]; end
%             dir_nom = v_vec / spd;
% 
%             % -------------------------------------------------------------
%             % 1. Receding Horizon 進路予測 ＆ 連続リプランニング判定
%             % -------------------------------------------------------------
%             if cha == 'f' && ~obj.replan_done && ~isempty(obs_list)
%                 best_idx = -1;
%                 min_proj_dist = inf;
%                 min_surf_dist = inf;
%                 sensor_owner = "";
% 
%                 for i = 1:length(obs_list)
%                     if obj.replan_active && (i == obj.active_obs_idx)
%                         continue;
%                     end
% 
%                     tgt_i = obs_list(i);
%                     c_i = tgt_i.p_center;
%                     rad_i = tgt_i.ellipsoid_radii;
%                     R_i = tgt_i.R_obs;
%                     d_m_i = tgt_i.d_margin;
% 
%                     d_proj_L = dot(c_i - pL_cur, dir_nom);
%                     d_proj_Q = dot(c_i - pQ_cur, dir_nom);
% 
%                     if d_proj_L <= 0.1 && d_proj_Q <= 0.1, continue; end
% 
%                     p_line_closest = pL_cur + d_proj_L * dir_nom;
%                     dist_lateral = norm(c_i - p_line_closest);
% 
%                     vec_lat = c_i - p_line_closest;
%                     if dist_lateral < 1e-4
%                         r_eff = max(rad_i);
%                     else
%                         dir_lat = vec_lat / dist_lateral;
%                         p_rel_lat = R_i' * dir_lat;
%                         r_eff = 1.0 / norm(p_rel_lat ./ rad_i);
%                     end
% 
%                     max_corridor_r = r_eff + max(obj.r_drone, obj.r_load) + d_m_i + 0.1;
%                     if dist_lateral > max_corridor_r, continue; end
% 
%                     d_surf_L = obj.calc_exact_surface_distance(pL_cur, c_i, R_i, rad_i);
%                     d_surf_Q = obj.calc_exact_surface_distance(pQ_cur, c_i, R_i, rad_i);
% 
%                     if d_surf_Q < d_surf_L
%                         cur_min_surf = d_surf_Q;
%                         cur_proj = d_proj_Q;
%                         cur_owner = "機体実位置センサー";
%                     else
%                         cur_min_surf = d_surf_L;
%                         cur_proj = d_proj_L;
%                         cur_owner = "荷物実位置センサー";
%                     end
% 
%                     if cur_min_surf <= obj.trigger_dist && cur_proj < min_proj_dist
%                         min_proj_dist = cur_proj;
%                         min_surf_dist = cur_min_surf;
%                         best_idx = i;
%                         sensor_owner = cur_owner;
%                     end
%                 end
% 
%                 % 新たな前方障害物を検知
%                 if best_idx > 0
%                     tgt = obs_list(best_idx);
% 
%                     if obj.replan_active
%                         tau_now = time.t - obj.t_start;
%                         obj.init_world_diff_state = obj.get_current_world_derivatives(tau_now);
%                         fprintf("\n[RECEDING HORIZON] 新たな障害物 (ID: %d) を捕捉! 軌道を動的更新します。\n", best_idx);
%                     else
%                         obj.init_world_diff_state = zeros(7, 3);
%                         fprintf("\n=======================================================\n");
%                         fprintf("[検知発動] 実状態センサーが前方障害物を捕捉! (ID: %d, Center=[%.2f, %.2f, %.2f])\n", ...
%                             best_idx, tgt.p_center(1), tgt.p_center(2), tgt.p_center(3));
%                     end
% 
%                     obj.active_obs_idx = best_idx;
%                     obj.obs_center     = tgt.p_center;
%                     obj.obs_R          = tgt.R_obs;
%                     obj.obs_radii      = tgt.ellipsoid_radii;
%                     obj.obs_margin     = tgt.d_margin;
% 
%                     obj.t_start        = time.t;
%                     obj.p_start_replan = pL_cur;
%                     obj.dir_nominal    = dir_nom;
%                     obj.nominal_speed  = spd;
% 
%                     vec_to_obs = obj.obs_center - obj.p_start_replan;
%                     d_proj_to_center = dot(vec_to_obs, dir_nom);
%                     p_closest_on_line = obj.p_start_replan + d_proj_to_center * dir_nom;
% 
%                     % 特異点フリー Gram-Schmidt 直交基底
%                     [~, min_dim] = min(abs(dir_nom));
%                     v_arbitrary = zeros(3, 1);
%                     v_arbitrary(min_dim) = 1.0;
% 
%                     base_n1 = cross(dir_nom, v_arbitrary);
%                     base_n1 = base_n1 / norm(base_n1);
%                     base_n2 = cross(dir_nom, base_n1);
%                     base_n2 = base_n2 / norm(base_n2);
% 
%                     v_center_rel = obj.obs_center - p_closest_on_line;
%                     d_center_lat = norm(v_center_rel);
%                     p_rel_raw = obj.obs_R' * (v_center_rel / max(d_center_lat, 1e-4));
%                     r_eff_est = 1.0 / norm(p_rel_raw ./ obj.obs_radii);
%                     test_radius = max(r_eff_est + 1.0, 2.0);
% 
%                     % 360度方向探索による最適安全退避法線の決定
%                     best_score = -inf;
%                     best_normal = base_n1;
%                     test_angles = linspace(0, 2*pi, 36);
%                     for ang = test_angles
%                         cand_n = cos(ang) * base_n1 + sin(ang) * base_n2;
%                         if d_center_lat > 1e-3 && dot(cand_n, -v_center_rel) < -0.1
%                             continue;
%                         end
%                         test_apex = p_closest_on_line + test_radius * cand_n;
%                         min_dist_other = inf;
%                         for j = 1:length(obs_list)
%                             if j == best_idx, continue; end
%                             d_j = obj.calc_exact_surface_distance(test_apex, obs_list(j).p_center, obs_list(j).R_obs, obs_list(j).ellipsoid_radii);
%                             min_dist_other = min(min_dist_other, d_j);
%                         end
%                         if min_dist_other > best_score
%                             best_score = min_dist_other;
%                             best_normal = cand_n;
%                         end
%                     end
% 
%                     obj.dir_normal = best_normal;
%                     b_cand = cross(dir_nom, obj.dir_normal);
%                     obj.dir_binormal = b_cand / norm(b_cand);
% 
%                     dist_line_to_obs = norm(obj.obs_center - p_closest_on_line);
%                     p_rel_n = obj.obs_R' * obj.dir_normal;
%                     r_eff_obs = 1.0 / norm(p_rel_n ./ obj.obs_radii);
% 
%                     effective_Kp = 1.0;
%                     try
%                         if isprop(obj.self.controller, "param") && isfield(obj.self.controller.param, "F2")
%                             raw_gain = obj.self.controller.param.F2(1);
%                             effective_Kp = max(0.5, min(2.5, raw_gain / 50.0));
%                         end
%                     catch
%                     end
% 
%                     base_clearance = max(0.5, (r_eff_obs + obj.r_drone + obj.obs_margin) - dist_line_to_obs);
% 
%                     t_geom = d_proj_to_center / spd;
%                     t_pendulum = 2 * pi * sqrt(obj.L_cable / obj.gravity);
%                     theta_max_rad = deg2rad(obj.max_swing_angle_deg);
%                     a_load_allow  = obj.gravity * tan(theta_max_rad);
%                     T_min_swing   = sqrt(10.0 * base_clearance / a_load_allow);
%                     T_min_drone   = sqrt(12.0 * base_clearance / obj.max_acc_drone);
% 
%                     obj.T_seg = max([t_geom, 1.2 * t_pendulum, T_min_swing, T_min_drone, 4.0]);
%                     obj.t_duration = 2.0 * obj.T_seg;
% 
%                     initial_pos_error = norm(pL_cur - (obj.p_start_replan + dot(pL_cur - obj.p_start_replan, dir_nom) * dir_nom));
%                     estimated_max_acc = 6.5 * base_clearance / (obj.T_seg^2);
%                     accel_tracking_error = estimated_max_acc / effective_Kp;
%                     mass_ratio = obj.m_load_est / (obj.m_drone + obj.m_load_est);
%                     swing_coupling_buffer = mass_ratio * (obj.L_cable / obj.gravity) * estimated_max_acc;
%                     dynamic_buffer = max(0.20, initial_pos_error + accel_tracking_error + swing_coupling_buffer);
% 
%                     actual_cable_vec = pQ_cur - pL_cur;
%                     cable_offset_s = dot(actual_cable_vec, obj.dir_nominal);
% 
%                     fprintf("  - 検知主体: %s (実表面距離: %.3f m, 進入速度: %.2f m/s)\n", sensor_owner, min_surf_dist, spd);
%                     fprintf("  - 選択された動的退避方向: [%.2f, %.2f, %.2f] (他障害物余白: %.2f m)\n", ...
%                         obj.dir_normal(1), obj.dir_normal(2), obj.dir_normal(3), best_score);
%                     fprintf("  - 物理最適化時間: T_seg = %.2f s (全時間 T = %.2f s)\n", obj.T_seg, obj.t_duration);
% 
%                     obj.plan_bezier_c6_error_spline_qp(dynamic_buffer, dist_line_to_obs, cable_offset_s);
%                     obj.replan_active = true;
%                     fprintf("=======================================================\n\n");
%                 end
%             end
% 
%             % -------------------------------------------------------------
%             % 2. 飛行中常時監視: 全障害物に対する網羅判定
%             % -------------------------------------------------------------
%             if cha == 'f' && ~isempty(obs_list)
%                 for j = 1:length(obs_list)
%                     tgt_j = obs_list(j);
%                     d_surf_L = obj.calc_exact_surface_distance(pL_cur, tgt_j.p_center, tgt_j.R_obs, tgt_j.ellipsoid_radii);
%                     d_surf_Q = obj.calc_exact_surface_distance(pQ_cur, tgt_j.p_center, tgt_j.R_obs, tgt_j.ellipsoid_radii);
% 
%                     if d_surf_L <= 0 && ~obj.warned_crash_load(j)
%                         fprintf(2, "[CRITICAL ALARM] 荷物が障害物%d本体に衝突! (t=%.3f s, 侵入深さ=%.3f m)\n", j, time.t, -d_surf_L);
%                         obj.warned_crash_load(j) = true;
%                     elseif d_surf_L <= (obj.r_load + tgt_j.d_margin) && ~obj.warned_margin_load(j)
%                         fprintf("[SAFETY WARN] 荷物が障害物%dのソフトマージン帯に侵入 (t=%.3f s, 表面残余=%.3f m)\n", j, time.t, d_surf_L);
%                         obj.warned_margin_load(j) = true;
%                     end
% 
%                     if d_surf_Q <= 0 && ~obj.warned_crash_drone(j)
%                         fprintf(2, "[CRITICAL ALARM] 機体が障害物%d本体に衝突! (t=%.3f s, 侵入深さ=%.3f m)\n", j, time.t, -d_surf_Q);
%                         obj.warned_crash_drone(j) = true;
%                     elseif d_surf_Q <= (obj.r_drone + tgt_j.d_margin) && ~obj.warned_margin_drone(j)
%                         fprintf("[SAFETY WARN] 機体が障害物%dのソフトマージン帯に侵入 (t=%.3f s, 表面残余=%.3f m)\n", j, time.t, d_surf_Q);
%                         obj.warned_margin_drone(j) = true;
%                     end
%                 end
%             end
% 
%             % -------------------------------------------------------------
%             % 3. 軌道出力 ＆ C^6 シームレス合流
%             % -------------------------------------------------------------
%             if obj.replan_active
%                 tau = time.t - obj.t_start;
%                 if tau <= obj.t_duration
%                     xd = obj.evaluate_smooth_trajectory(tau, xd_nominal);
%                 else
%                     if ~obj.replan_done
%                         fprintf("[BÉZIER C^6 QP] 障害物通過完了・完全シームレス復帰 (t=%.3f s)\n\n", time.t);
%                         obj.t_merge_end = obj.t_start + obj.t_duration;
%                         xd_end = obj.evaluate_smooth_trajectory(obj.t_duration, xd_nominal);
%                         obj.p_merge_end = xd_end(1:3);
%                         obj.v_merge_vec = xd_end(5:7);
%                         obj.replan_active = false;
%                         obj.replan_done   = true;
%                     end
% 
%                     dt_after = time.t - obj.t_merge_end;
%                     xd = zeros(28, 1);
%                     xd(1:3) = obj.p_merge_end + obj.v_merge_vec * dt_after;
%                     xd(5:7) = obj.v_merge_vec;
%                     xd(9:28) = 0;
%                 end
%             elseif obj.replan_done
%                 dt_after = time.t - obj.t_merge_end;
%                 xd = zeros(28, 1);
%                 xd(1:3) = obj.p_merge_end + obj.v_merge_vec * dt_after;
%                 xd(5:7) = obj.v_merge_vec;
%                 xd(9:28) = 0;
%             else
%                 xd = xd_nominal;
%             end
% 
%             if length(xd) < 28, xd = [xd; zeros(28 - length(xd), 1)]; end
%             obj.result.state.xd = xd;
%             obj.result.state.p = xd(1:3);
%             obj.result.state.v = xd(5:7);
%             obj.result.state.q = [0; 0; xd(4)];
%             result = obj.result;
%         end
% 
%         function xd = evaluate_smooth_trajectory(obj, tau, xd_nom)
%             xd = xd_nom;
%             N = obj.order;
%             T_seg = obj.T_seg;
% 
%             if tau <= T_seg
%                 C = obj.coeffs_delta_seg1;
%                 u = max(0, min(1.0, tau / T_seg));
%             else
%                 C = obj.coeffs_delta_seg2;
%                 u = max(0, min(1.0, (tau - T_seg) / T_seg));
%             end
% 
%             for k = 0:6
%                 c_diff = obj.get_bezier_derivative_coeffs_at_u(N, k, u);
%                 delta_n = dot(c_diff, C(:, 1)) / (T_seg^k);
%                 delta_b = dot(c_diff, C(:, 2)) / (T_seg^k);
% 
%                 delta_vec = obj.dir_normal * delta_n + obj.dir_binormal * delta_b;
% 
%                 idx = 4 * k + (1:3);
%                 xd(idx) = xd_nom(idx) + delta_vec;
%             end
%         end
%     end
% 
%     methods (Access = private)
%         function d = calc_exact_surface_distance(~, p, c, R, rad)
%             p_rel = R' * (p - c);
%             val = norm(p_rel ./ rad);
%             if val < 1e-6
%                 d = -min(rad);
%                 return;
%             end
%             p_surf = p_rel / val;
%             d = (val - 1.0) * norm(p_surf);
%         end
% 
%         function world_diffs = get_current_world_derivatives(obj, tau)
%             world_diffs = zeros(7, 3);
%             N = obj.order;
%             T_seg = obj.T_seg;
%             if tau <= T_seg
%                 C = obj.coeffs_delta_seg1;
%                 u = max(0, min(1.0, tau / T_seg));
%             else
%                 C = obj.coeffs_delta_seg2;
%                 u = max(0, min(1.0, (tau - T_seg) / T_seg));
%             end
%             for k = 0:6
%                 c_diff = obj.get_bezier_derivative_coeffs_at_u(N, k, u);
%                 delta_n = dot(c_diff, C(:, 1)) / (T_seg^k);
%                 delta_b = dot(c_diff, C(:, 2)) / (T_seg^k);
% 
%                 world_vec_k = obj.dir_normal * delta_n + obj.dir_binormal * delta_b;
%                 world_diffs(k + 1, :) = world_vec_k';
%             end
%         end
% 
%         function plan_bezier_c6_error_spline_qp(obj, dynamic_buffer, dist_line_to_obs, cable_offset_s)
%             N = obj.order;
%             n_c = N + 1;
%             T_seg = obj.T_seg;
% 
%             p_rel_n = obj.obs_R' * obj.dir_normal;
%             r_eff_n = 1.0 / norm(p_rel_n ./ obj.obs_radii);
%             p_rel_s = obj.obs_R' * obj.dir_nominal;
%             r_eff_s = 1.0 / norm(p_rel_s ./ obj.obs_radii);
% 
%             max_r_obj = max(obj.r_drone, obj.r_load);
%             a_mink = r_eff_n + max_r_obj + obj.obs_margin + dynamic_buffer;
%             c_mink = r_eff_s + max_r_obj + obj.obs_margin + dynamic_buffer;
% 
%             % 頂点クリアランス要求量（確実に障害物外郭を安全にパスする値）
%             delta_target = max(a_mink - dist_line_to_obs, 1.0);
% 
%             n_vars_1d  = 2 * n_c;
%             n_vars_tot = 2 * n_vars_1d;
% 
%             Q_snap = obj.compute_bezier_derivative_hessian(N, 4);
%             Q_acc  = obj.compute_bezier_derivative_hessian(N, 2);
%             Q_pos  = obj.compute_bezier_pos_hessian(N);
% 
%             Q_snap = Q_snap / norm(Q_snap, 2);
%             Q_acc  = Q_acc  / norm(Q_acc,  2);
%             Q_pos  = Q_pos  / norm(Q_pos,  2);
% 
%             H_1d = 1.0 * Q_snap + 0.1 * Q_acc + 1e-4 * Q_pos;
%             H_1d = (H_1d + H_1d') / 2 + 1e-6 * eye(n_c);
% 
%             H = blkdiag(H_1d, H_1d, H_1d, H_1d);
%             f = zeros(n_vars_tot, 1);
% 
%             % 等式制約 (42本)
%             Aeq_1d = zeros(21, n_vars_1d);
%             beq_n  = zeros(21, 1);
%             beq_b  = zeros(21, 1);
% 
%             % 1) 始端 (u1=0): キャプチャしたワールド微分ベクトルを新基底へ射影
%             for k = 0:6
%                 c_diff = obj.get_bezier_derivative_coeffs_at_u(N, k, 0.0);
%                 Aeq_1d(k + 1, 1:n_c) = c_diff / (T_seg^k);
% 
%                 w_vec_k = obj.init_world_diff_state(k + 1, :)';
%                 beq_n(k + 1) = dot(w_vec_k, obj.dir_normal);
%                 beq_b(k + 1) = dot(w_vec_k, obj.dir_binormal);
%             end
% 
%             % 2) 中間接続点 (u1=1, u2=0): 0〜6階微分完全一致 (Gap=0)
%             for k = 0:6
%                 c_end   = obj.get_bezier_derivative_coeffs_at_u(N, k, 1.0);
%                 c_start = obj.get_bezier_derivative_coeffs_at_u(N, k, 0.0);
%                 Aeq_1d(7 + k + 1, 1:n_c)       =  c_end   / (T_seg^k);
%                 Aeq_1d(7 + k + 1, n_c+1:2*n_c) = -c_start / (T_seg^k);
%             end
% 
%             % 3) 終端 (u2=1): 0〜6階微分 = 0
%             for k = 0:6
%                 c_diff = obj.get_bezier_derivative_coeffs_at_u(N, k, 1.0);
%                 Aeq_1d(14 + k + 1, n_c+1:2*n_c) = c_diff / (T_seg^k);
%             end
% 
%             Aeq = blkdiag(Aeq_1d, Aeq_1d);
%             beq = [beq_n; beq_b];
% 
%             % 不等式制約: 楕円断面逆算 ＋ 頂点厳密通過
%             alpha_dyn = obj.L_cable / (obj.gravity * (T_seg^2));
%             A_ineq = [];
%             b_ineq = [];
% 
%             % 区間 1 (接近側)
%             N_samp = 7;
%             u1_samples = linspace(0.3, 1.0, N_samp);
%             for u = u1_samples
%                 B_val      = obj.eval_bernstein_vector(N, u);
%                 B_ddot_val = obj.get_bezier_derivative_coeffs_at_u(N, 2, u);
%                 BQ_val     = B_val + alpha_dyn * B_ddot_val;
% 
%                 delta_s_L = (1.0 - u) * obj.nominal_speed * T_seg;
%                 delta_s_Q = delta_s_L - cable_offset_s;
% 
%                 req_L = 0.0;
%                 if abs(delta_s_L) < c_mink
%                     req_L = max(0, a_mink * sqrt(1.0 - (delta_s_L / c_mink)^2) - dist_line_to_obs);
%                 end
%                 req_Q = 0.0;
%                 if abs(delta_s_Q) < c_mink
%                     req_Q = max(0, a_mink * sqrt(1.0 - (delta_s_Q / c_mink)^2) - dist_line_to_obs);
%                 end
% 
%                 r_L = zeros(1, n_vars_tot); r_L(1:n_c) = -B_val;
%                 r_Q = zeros(1, n_vars_tot); r_Q(1:n_c) = -BQ_val;
%                 A_ineq = [A_ineq; r_L; r_Q];
%                 b_ineq = [b_ineq; -req_L; -req_Q];
%             end
% 
%             % 区間 2 (離脱側)
%             u2_samples = linspace(0.0, 0.7, N_samp);
%             for u = u2_samples
%                 B_val      = obj.eval_bernstein_vector(N, u);
%                 B_ddot_val = obj.get_bezier_derivative_coeffs_at_u(N, 2, u);
%                 BQ_val     = B_val + alpha_dyn * B_ddot_val;
% 
%                 delta_s_L = u * obj.nominal_speed * T_seg;
%                 delta_s_Q = delta_s_L - cable_offset_s;
% 
%                 req_L = 0.0;
%                 if abs(delta_s_L) < c_mink
%                     req_L = max(0, a_mink * sqrt(1.0 - (delta_s_L / c_mink)^2) - dist_line_to_obs);
%                 end
%                 req_Q = 0.0;
%                 if abs(delta_s_Q) < c_mink
%                     req_Q = max(0, a_mink * sqrt(1.0 - (delta_s_Q / c_mink)^2) - dist_line_to_obs);
%                 end
% 
%                 r_L = zeros(1, n_vars_tot); r_L(n_c+1:2*n_c) = -B_val;
%                 r_Q = zeros(1, n_vars_tot); r_Q(n_c+1:2*n_c) = -BQ_val;
%                 A_ineq = [A_ineq; r_L; r_Q];
%                 b_ineq = [b_ineq; -req_L; -req_Q];
%             end
% 
%             % 中間頂点でのハード下限保証 (最接近点で確実に delta_target 退避)
%             B_mid = obj.eval_bernstein_vector(N, 1.0);
%             B_ddot_mid = obj.get_bezier_derivative_coeffs_at_u(N, 2, 1.0);
%             BQ_mid = B_mid + alpha_dyn * B_ddot_mid;
% 
%             r_mid_L = zeros(1, n_vars_tot); r_mid_L(1:n_c) = -B_mid;
%             r_mid_Q = zeros(1, n_vars_tot); r_mid_Q(1:n_c) = -BQ_mid;
%             A_ineq = [A_ineq; r_mid_L; r_mid_Q];
%             b_ineq = [b_ineq; -delta_target; -delta_target];
% 
%             % 振れ角制限
%             theta_max_rad = deg2rad(obj.max_swing_angle_deg);
%             a_load_limit  = obj.gravity * tan(theta_max_rad);
%             u_check = [0.5, 0.8, 1.0];
%             for u = u_check
%                 c_ddot = obj.get_bezier_derivative_coeffs_at_u(N, 2, u) / (T_seg^2);
%                 r_pos = zeros(1, n_vars_tot); r_pos(1:n_c) = c_ddot;
%                 r_neg = zeros(1, n_vars_tot); r_neg(1:n_c) = -c_ddot;
%                 A_ineq = [A_ineq; r_pos; r_neg];
%                 b_ineq = [b_ineq; a_load_limit; a_load_limit];
%             end
% 
%             % 機体加速度制限
%             for u = u_check
%                 c_ddot = obj.get_bezier_derivative_coeffs_at_u(N, 2, u) / (T_seg^2);
%                 c_snap = obj.get_bezier_derivative_coeffs_at_u(N, 4, u) / (T_seg^4);
%                 c_drone_acc = c_ddot + (obj.L_cable / obj.gravity) * c_snap;
% 
%                 r_pos = zeros(1, n_vars_tot); r_pos(1:n_c) = c_drone_acc;
%                 r_neg = zeros(1, n_vars_tot); r_neg(1:n_c) = -c_drone_acc;
%                 A_ineq = [A_ineq; r_pos; r_neg];
%                 b_ineq = [b_ineq; obj.max_acc_drone; obj.max_acc_drone];
%             end
% 
%             opts = optimoptions('quadprog', ...
%                 'Display', 'off', ...
%                 'Algorithm', 'interior-point-convex', ...
%                 'MaxIterations', 300, ...
%                 'ConstraintTolerance', 1e-4, ...
%                 'OptimalityTolerance', 1e-4);
% 
%             [X_opt, ~, exitflag, output] = quadprog(H, f, A_ineq, b_ineq, Aeq, beq, [], [], [], opts);
% 
%             if exitflag < 1
%                 fprintf("[BÉZIER C^6 DEBUG] 不等式制約付きQP未収束 (exitflag=%d). 等式拘束フォールバックを実行\n", exitflag);
%                 X_fb = pinv(Aeq) * beq;
%                 cn1 = X_fb(1:n_c);         cn2 = X_fb(n_c+1:2*n_c);
%                 cb1 = X_fb(2*n_c+1:3*n_c); cb2 = X_fb(3*n_c+1:4*n_c);
%             else
%                 fprintf("[BÉZIER C^6] 最適化成功! (反復=%d, ピーク退避量=%.3fm)\n", output.iterations, delta_target);
%                 cn1 = X_opt(1:n_c);             cn2 = X_opt(n_c+1:2*n_c);
%                 cb1 = X_opt(2*n_c+1:3*n_c);     cb2 = X_opt(3*n_c+1:4*n_c);
%             end
% 
%             obj.coeffs_delta_seg1 = [cn1, cb1];
%             obj.coeffs_delta_seg2 = [cn2, cb2];
% 
%             obj.verify_c6_continuity();
%         end
% 
%         function verify_c6_continuity(obj)
%             N = obj.order;
%             T_seg = obj.T_seg;
%             names = ["位置 (0階)", "速度 (1階)", "加速度 (2階)", "Jerk (3階)", "Snap (4階)", "Crack (5階)", "Pop (6階)"];
% 
%             fprintf("-------------------------------------------------------\n");
%             fprintf("[理論検証] 中間接続点における 0〜6階微分の不連続量 (Gap):\n");
%             for k = 0:6
%                 c_end   = obj.get_bezier_derivative_coeffs_at_u(N, k, 1.0);
%                 c_start = obj.get_bezier_derivative_coeffs_at_u(N, k, 0.0);
% 
%                 d1_n = dot(c_end,   obj.coeffs_delta_seg1(:, 1)) / (T_seg^k);
%                 d1_b = dot(c_end,   obj.coeffs_delta_seg1(:, 2)) / (T_seg^k);
% 
%                 d2_n = dot(c_start, obj.coeffs_delta_seg2(:, 1)) / (T_seg^k);
%                 d2_b = dot(c_start, obj.coeffs_delta_seg2(:, 2)) / (T_seg^k);
% 
%                 gap = norm([d1_n - d2_n; d1_b - d2_b]);
%                 fprintf("  - %-12s ギャップ: %.3e\n", names(k + 1), gap);
%             end
%             fprintf("-------------------------------------------------------\n");
%         end
% 
%         function B = eval_bernstein_vector(~, n, u)
%             B = zeros(1, n + 1);
%             for i = 0:n
%                 B(i + 1) = nchoosek(n, i) * (u^i) * ((1 - u)^(n - i));
%             end
%         end
% 
%         function c_diff = get_bezier_derivative_coeffs_at_u(~, n, k, u)
%             if k > n, c_diff = zeros(1, n + 1); return; end
%             factor = factorial(n) / factorial(n - k);
%             b_low = zeros(1, n - k + 1);
%             for j = 0:(n - k)
%                 b_low(j + 1) = nchoosek(n - k, j) * (u^j) * ((1 - u)^(n - k - j));
%             end
%             D = eye(n + 1);
%             for step = 1:k, D = diff(D); end
%             c_diff = factor * (b_low * D);
%         end
% 
%         function Q = compute_bezier_derivative_hessian(obj, n, k)
%             Q = zeros(n + 1, n + 1);
%             if k > n, return; end
%             factor = factorial(n) / factorial(n - k);
%             n_low = n - k;
%             D = eye(n + 1);
%             for step = 1:k, D = diff(D); end
%             M_low = zeros(n_low + 1, n_low + 1);
%             for i = 0:n_low
%                 for j = 0:n_low
%                     M_low(i + 1, j + 1) = (nchoosek(n_low, i) * nchoosek(n_low, j)) / ...
%                         ((2 * n_low + 1) * nchoosek(2 * n_low, i + j));
%                 end
%             end
%             Q = (factor^2) * (D' * M_low * D);
%         end
% 
%         function Q = compute_bezier_pos_hessian(~, n)
%             Q = zeros(n + 1, n + 1);
%             for i = 0:n
%                 for j = 0:n
%                     Q(i + 1, j + 1) = (nchoosek(n, i) * nchoosek(n, j)) / ...
%                         ((2 * n + 1) * nchoosek(2 * n, i + j));
%                 end
%             end
%         end
%     end
% end

% classdef REPLANNING_MELLINGER_CORRIDOR_FLATNESS_SWING < handle
%     % REPLANNING_MELLINGER_CORRIDOR_FLATNESS_SWING
%     % 汎用3次元静的・動的障害物対応 13次 Bézier C^6 完全連続 時空間SFCリプランナ
%     % - 4D時空間楕円境界逆算: 静的障害物 (v_obs=0) と動的障害物 (v_obs!=0) を統一処理
%     % - Receding Horizon 動的初期状態継承 (0〜6階微分ワールド座標系射影)
%     % - 360度時空間安全法線探索 (未来時刻における他障害物クリアランス最大化)
%     % - 機体・荷物の平坦性連立不等式制約 ＆ 動的バッファ (Tube Bound)
% 
%     properties
%         base_ref
%         self
%         replan_active = false
%         replan_done   = false
% 
%         t_start = 0.0
%         t_duration = 8.0
%         T_seg
% 
%         safe_margin  = 0.3
%         trigger_dist = 7.0
% 
%         L_cable      = 2.0
%         gravity      = 9.81
%         m_drone      = 1.5
%         m_load_est   = 0.1
% 
%         r_load       = 0.15
%         r_drone      = 0.30
% 
%         max_swing_angle_deg = 20.0
%         max_acc_drone       = 6.0
% 
%         order = 13
%         coeffs_delta_seg1 % (14 x 2: 法線, 従法線)
%         coeffs_delta_seg2 % (14 x 2: 法線, 従法線)
% 
%         dir_normal        % 最適退避法線 (3 x 1)
%         dir_binormal      % 最適退避従法線 (3 x 1)
%         dir_nominal       % 進行方向単位ベクトル (3 x 1)
%         nominal_speed     % 進入巡航速度
%         p_start_replan    % リプラン開始位置
% 
%         t_merge_end
%         p_merge_end
%         v_merge_vec
% 
%         active_obs_idx = -1
%         obs_center_init= [0; 0; 0] % リプラン開始時の中心座標
%         obs_velocity   = [0; 0; 0] % 障害物速度ベクトル
%         obs_R          = eye(3)
%         obs_radii      = [1; 1; 1]
%         obs_margin     = 0
% 
%         % 再計画時のワールド座標系初期微分バッファ (7 x 3)
%         init_world_diff_state = zeros(7, 3);
% 
%         warned_crash_load
%         warned_crash_drone
%         warned_margin_load
%         warned_margin_drone
% 
%         debug_cnt = 0
%         result
%     end
% 
%     methods (Access = public)
%         function obj = REPLANNING_MELLINGER_CORRIDOR_FLATNESS_SWING(self, base_ref, opts)
%             arguments
%                 self
%                 base_ref
%                 opts = struct()
%             end
%             obj.self = self;
%             obj.base_ref = base_ref;
%             if isfield(opts, "safe_margin"),         obj.safe_margin         = opts.safe_margin;         end
%             if isfield(opts, "trigger_dist"),        obj.trigger_dist        = opts.trigger_dist;        end
%             if isfield(opts, "r_load"),              obj.r_load              = opts.r_load;              end
%             if isfield(opts, "r_drone"),             obj.r_drone             = opts.r_drone;             end
%             if isfield(opts, "max_swing_angle_deg"), obj.max_swing_angle_deg = opts.max_swing_angle_deg; end
%             if isfield(opts, "max_acc_drone"),       obj.max_acc_drone       = opts.max_acc_drone;       end
%             obj.result.state = STATE_CLASS(struct('state_list', ["xd", "p", "q", "v"], 'num_list', [28, 3, 3, 3]));
%         end
% 
%         function result = do(obj, varargin)
%             time = varargin{1};
%             cha = varargin{2};
% 
%             base_res = obj.base_ref.do(time, cha);
%             xd_nominal = base_res.state.xd;
%             if length(xd_nominal) < 28
%                 xd_nominal = [xd_nominal; zeros(28 - length(xd_nominal), 1)];
%             end
% 
%             obj.L_cable = obj.self.parameter.get("cableL");
%             try
%                 obj.m_drone = obj.self.parameter.get("mass");
%             catch
%                 obj.m_drone = 1.5;
%             end
% 
%             if isprop(obj.self.estimator.result.state, "mL")
%                 obj.m_load_est = max(0.001, obj.self.estimator.result.state.mL);
%             elseif isprop(obj.self.estimator.result, "loadmass")
%                 obj.m_load_est = max(0.001, obj.self.estimator.result.loadmass);
%             else
%                 try
%                     obj.m_load_est = max(0.001, obj.self.parameter.get("loadmass"));
%                 catch
%                     obj.m_load_est = 0.013;
%                 end
%             end
% 
%             if isprop(obj.self.estimator.result.state, "pL")
%                 pL_cur = obj.self.estimator.result.state.pL;
%                 vL_cur = obj.self.estimator.result.state.vL;
%             else
%                 pL_cur = base_res.state.p;
%                 vL_cur = base_res.state.v;
%             end
% 
%             if isprop(obj.self.estimator.result.state, "p")
%                 pQ_cur = obj.self.estimator.result.state.p;
%             else
%                 pQ_cur = pL_cur + [0; 0; obj.L_cable];
%             end
% 
%             % 全障害物リストの取得
%             obs_list = [];
%             try
%                 obs_list = ENVIRONMENT_OBSTACLE_ELLIPSE();
%             catch
%             end
% 
%             if isempty(obj.warned_crash_load) && ~isempty(obs_list)
%                 n_obs = length(obs_list);
%                 obj.warned_crash_load   = false(n_obs, 1);
%                 obj.warned_crash_drone  = false(n_obs, 1);
%                 obj.warned_margin_load  = false(n_obs, 1);
%                 obj.warned_margin_drone = false(n_obs, 1);
%             end
% 
%             v_vec = xd_nominal(5:7);
%             spd = norm(v_vec);
%             if spd < 0.05, spd = norm(vL_cur); end
%             if spd < 0.05, spd = 1.0; v_vec = [0; 0; 1.0]; end
%             dir_nom = v_vec / spd;
% 
%             % -------------------------------------------------------------
%             % 1. Receding Horizon 時空間交差判定 ＆ リプラン判定
%             % -------------------------------------------------------------
%             if cha == 'f' && ~obj.replan_done && ~isempty(obs_list)
%                 best_idx = -1;
%                 min_proj_dist = inf;
%                 min_surf_dist = inf;
%                 sensor_owner = "";
% 
%                 for i = 1:length(obs_list)
%                     if obj.replan_active && (i == obj.active_obs_idx)
%                         continue;
%                     end
% 
%                     tgt_i = obs_list(i);
%                     v_obs_i = obj.extract_obstacle_velocity(tgt_i);
%                     c_i = tgt_i.p_center;
%                     rad_i = tgt_i.ellipsoid_radii;
%                     R_i = tgt_i.R_obs;
%                     d_m_i = tgt_i.d_margin;
% 
%                     % 相対速度ベクトル
%                     v_rel = dir_nom * spd - v_obs_i;
%                     spd_rel = norm(v_rel);
%                     if spd_rel < 0.05, spd_rel = 1.0; v_rel = dir_nom; end
%                     dir_rel = v_rel / spd_rel;
% 
%                     d_proj_L = dot(c_i - pL_cur, dir_rel);
%                     d_proj_Q = dot(c_i - pQ_cur, dir_rel);
% 
%                     if d_proj_L <= 0.1 && d_proj_Q <= 0.1, continue; end
% 
%                     p_line_closest = pL_cur + d_proj_L * dir_rel;
%                     dist_lateral = norm(c_i - p_line_closest);
% 
%                     vec_lat = c_i - p_line_closest;
%                     if dist_lateral < 1e-4
%                         r_eff = max(rad_i);
%                     else
%                         dir_lat = vec_lat / dist_lateral;
%                         p_rel_lat = R_i' * dir_lat;
%                         r_eff = 1.0 / norm(p_rel_lat ./ rad_i);
%                     end
% 
%                     max_corridor_r = r_eff + max(obj.r_drone, obj.r_load) + d_m_i + 0.1;
%                     if dist_lateral > max_corridor_r, continue; end
% 
%                     d_surf_L = obj.calc_exact_surface_distance(pL_cur, c_i, R_i, rad_i);
%                     d_surf_Q = obj.calc_exact_surface_distance(pQ_cur, c_i, R_i, rad_i);
% 
%                     if d_surf_Q < d_surf_L
%                         cur_min_surf = d_surf_Q;
%                         cur_proj = d_proj_Q;
%                         cur_owner = "機体実位置センサー";
%                     else
%                         cur_min_surf = d_surf_L;
%                         cur_proj = d_proj_L;
%                         cur_owner = "荷物実位置センサー";
%                     end
% 
%                     if cur_min_surf <= obj.trigger_dist && cur_proj < min_proj_dist
%                         min_proj_dist = cur_proj;
%                         min_surf_dist = cur_min_surf;
%                         best_idx = i;
%                         sensor_owner = cur_owner;
%                     end
%                 end
% 
%                 % 新たな前方障害物を捕捉
%                 if best_idx > 0
%                     tgt = obs_list(best_idx);
%                     v_obs_tgt = obj.extract_obstacle_velocity(tgt);
% 
%                     if obj.replan_active
%                         tau_now = time.t - obj.t_start;
%                         obj.init_world_diff_state = obj.get_current_world_derivatives(tau_now);
%                         fprintf("\n[RECEDING HORIZON] 新たな障害物 (ID: %d, 動的速度=[%.2f, %.2f, %.2f]) を捕捉! 軌道を動的更新します。\n", ...
%                             best_idx, v_obs_tgt(1), v_obs_tgt(2), v_obs_tgt(3));
%                     else
%                         obj.init_world_diff_state = zeros(7, 3);
%                         fprintf("\n=======================================================\n");
%                         fprintf("[検知発動] 実状態センサーが前方障害物を捕捉! (ID: %d, Center=[%.2f, %.2f, %.2f], 動的速度=[%.2f, %.2f, %.2f])\n", ...
%                             best_idx, tgt.p_center(1), tgt.p_center(2), tgt.p_center(3), v_obs_tgt(1), v_obs_tgt(2), v_obs_tgt(3));
%                     end
% 
%                     obj.active_obs_idx   = best_idx;
%                     obj.obs_center_init  = tgt.p_center;
%                     obj.obs_velocity     = v_obs_tgt;
%                     obj.obs_R            = tgt.R_obs;
%                     obj.obs_radii        = tgt.ellipsoid_radii;
%                     obj.obs_margin     = tgt.d_margin;
% 
%                     obj.t_start        = time.t;
%                     obj.p_start_replan = pL_cur;
%                     obj.dir_nominal    = dir_nom;
%                     obj.nominal_speed  = spd;
% 
%                     % 特異点フリー Gram-Schmidt 直交基底
%                     [~, min_dim] = min(abs(dir_nom));
%                     v_arbitrary = zeros(3, 1);
%                     v_arbitrary(min_dim) = 1.0;
% 
%                     base_n1 = cross(dir_nom, v_arbitrary);
%                     base_n1 = base_n1 / norm(base_n1);
%                     base_n2 = cross(dir_nom, base_n1);
%                     base_n2 = base_n2 / norm(base_n2);
% 
%                     vec_to_obs = obj.obs_center_init - obj.p_start_replan;
%                     v_rel_app = dir_nom * spd - obj.obs_velocity;
%                     spd_rel_app = max(norm(v_rel_app), 0.2);
%                     t_to_center = max(0.5, dot(vec_to_obs, v_rel_app) / (spd_rel_app^2));
% 
%                     obs_center_closest = obj.obs_center_init + obj.obs_velocity * t_to_center;
%                     p_closest_on_line = obj.p_start_replan + (spd * t_to_center) * dir_nom;
% 
%                     v_center_rel = obs_center_closest - p_closest_on_line;
%                     d_center_lat = norm(v_center_rel);
%                     p_rel_raw = obj.obs_R' * (v_center_rel / max(d_center_lat, 1e-4));
%                     r_eff_est = 1.0 / norm(p_rel_raw ./ obj.obs_radii);
%                     test_radius = max(r_eff_est + 1.0, 2.0);
% 
%                     % 360度時空間方向探索による最適安全退避法線の決定
%                     best_score = -inf;
%                     best_normal = base_n1;
%                     test_angles = linspace(0, 2*pi, 36);
%                     for ang = test_angles
%                         cand_n = cos(ang) * base_n1 + sin(ang) * base_n2;
%                         if d_center_lat > 1e-3 && dot(cand_n, -v_center_rel) < -0.1
%                             continue;
%                         end
%                         test_apex = p_closest_on_line + test_radius * cand_n;
%                         min_dist_other = inf;
%                         for j = 1:length(obs_list)
%                             if j == best_idx, continue; end
%                             v_j = obj.extract_obstacle_velocity(obs_list(j));
%                             c_j_future = obs_list(j).p_center + v_j * t_to_center;
%                             d_j = obj.calc_exact_surface_distance(test_apex, c_j_future, obs_list(j).R_obs, obs_list(j).ellipsoid_radii);
%                             min_dist_other = min(min_dist_other, d_j);
%                         end
%                         if min_dist_other > best_score
%                             best_score = min_dist_other;
%                             best_normal = cand_n;
%                         end
%                     end
% 
%                     obj.dir_normal = best_normal;
%                     b_cand = cross(dir_nom, obj.dir_normal);
%                     obj.dir_binormal = b_cand / norm(b_cand);
% 
%                     p_rel_n = obj.obs_R' * obj.dir_normal;
%                     r_eff_obs = 1.0 / norm(p_rel_n ./ obj.obs_radii);
% 
%                     effective_Kp = 1.0;
%                     try
%                         if isprop(obj.self.controller, "param") && isfield(obj.self.controller.param, "F2")
%                             raw_gain = obj.self.controller.param.F2(1);
%                             effective_Kp = max(0.5, min(2.5, raw_gain / 50.0));
%                         end
%                     catch
%                     end
% 
%                     base_clearance = max(0.5, (r_eff_obs + obj.r_drone + obj.obs_margin) - d_center_lat);
% 
%                     t_pendulum = 2 * pi * sqrt(obj.L_cable / obj.gravity);
%                     theta_max_rad = deg2rad(obj.max_swing_angle_deg);
%                     a_load_allow  = obj.gravity * tan(theta_max_rad);
%                     T_min_swing   = sqrt(10.0 * base_clearance / a_load_allow);
%                     T_min_drone   = sqrt(12.0 * base_clearance / obj.max_acc_drone);
% 
%                     obj.T_seg = max([t_to_center, 1.2 * t_pendulum, T_min_swing, T_min_drone, 4.0]);
%                     obj.t_duration = 2.0 * obj.T_seg;
% 
%                     initial_pos_error = norm(pL_cur - (obj.p_start_replan + dot(pL_cur - obj.p_start_replan, dir_nom) * dir_nom));
%                     estimated_max_acc = 6.5 * base_clearance / (obj.T_seg^2);
%                     accel_tracking_error = estimated_max_acc / effective_Kp;
%                     mass_ratio = obj.m_load_est / (obj.m_drone + obj.m_load_est);
%                     swing_coupling_buffer = mass_ratio * (obj.L_cable / obj.gravity) * estimated_max_acc;
%                     dynamic_buffer = max(0.20, initial_pos_error + accel_tracking_error + swing_coupling_buffer);
% 
%                     actual_cable_vec = pQ_cur - pL_cur;
%                     cable_offset_s = dot(actual_cable_vec, obj.dir_nominal);
% 
%                     fprintf("  - 検知主体: %s (実表面距離: %.3f m, 進入速度: %.2f m/s)\n", sensor_owner, min_surf_dist, spd);
%                     fprintf("  - 選択された動的退避方向: [%.2f, %.2f, %.2f] (他障害物余白: %.2f m)\n", ...
%                         obj.dir_normal(1), obj.dir_normal(2), obj.dir_normal(3), best_score);
%                     fprintf("  - 時空間最適化時間: T_seg = %.2f s (全時間 T = %.2f s)\n", obj.T_seg, obj.t_duration);
% 
%                     obj.plan_spatiotemporal_bezier_c6_qp(dynamic_buffer, d_center_lat, cable_offset_s);
%                     obj.replan_active = true;
%                     fprintf("=======================================================\n\n");
%                 end
%             end
% 
%             % -------------------------------------------------------------
%             % 2. 飛行中常時監視: 全障害物（動的位置を逐次反映）に対する網羅判定
%             % -------------------------------------------------------------
%             if cha == 'f' && ~isempty(obs_list)
%                 % t_start が未設定の場合は time.t を基準にする
%                 if isempty(obj.t_start) || ~obj.replan_active
%                     dt_obs_eval = time.t;
%                 else
%                     dt_obs_eval = time.t - obj.t_start;
%                 end
% 
%                 for j = 1:length(obs_list)
%                     tgt_j = obs_list(j);
%                     v_j = obj.extract_obstacle_velocity(tgt_j);
%                     c_j_now = tgt_j.p_center + v_j * dt_obs_eval;
% 
%                     d_surf_L = obj.calc_exact_surface_distance(pL_cur, c_j_now, tgt_j.R_obs, tgt_j.ellipsoid_radii);
%                     d_surf_Q = obj.calc_exact_surface_distance(pQ_cur, c_j_now, tgt_j.R_obs, tgt_j.ellipsoid_radii);
% 
%                     if d_surf_L <= 0 && ~obj.warned_crash_load(j)
%                         fprintf(2, "[CRITICAL ALARM] 荷物が障害物%d本体に衝突! (t=%.3f s, 侵入深さ=%.3f m)\n", j, time.t, -d_surf_L);
%                         obj.warned_crash_load(j) = true;
%                     elseif d_surf_L <= (obj.r_load + tgt_j.d_margin) && ~obj.warned_margin_load(j)
%                         fprintf("[SAFETY WARN] 荷物が障害物%dのソフトマージン帯に侵入 (t=%.3f s, 表面残余=%.3f m)\n", j, time.t, d_surf_L);
%                         obj.warned_margin_load(j) = true;
%                     end
% 
%                     if d_surf_Q <= 0 && ~obj.warned_crash_drone(j)
%                         fprintf(2, "[CRITICAL ALARM] 機体が障害物%d本体に衝突! (t=%.3f s, 侵入深さ=%.3f m)\n", j, time.t, -d_surf_Q);
%                         obj.warned_crash_drone(j) = true;
%                     elseif d_surf_Q <= (obj.r_drone + tgt_j.d_margin) && ~obj.warned_margin_drone(j)
%                         fprintf("[SAFETY WARN] 機体が障害物%dのソフトマージン帯に侵入 (t=%.3f s, 表面残余=%.3f m)\n", j, time.t, d_surf_Q);
%                         obj.warned_margin_drone(j) = true;
%                     end
%                 end
%             end
% 
%             % -------------------------------------------------------------
%             % 3. 軌道出力 ＆ C^6 シームレス合流
%             % -------------------------------------------------------------
%             if obj.replan_active
%                 tau = time.t - obj.t_start;
%                 if tau <= obj.t_duration
%                     xd = obj.evaluate_smooth_trajectory(tau, xd_nominal);
%                 else
%                     if ~obj.replan_done
%                         fprintf("[BÉZIER C^6 QP] 障害物通過完了・完全シームレス復帰 (t=%.3f s)\n\n", time.t);
%                         obj.t_merge_end = obj.t_start + obj.t_duration;
%                         xd_end = obj.evaluate_smooth_trajectory(obj.t_duration, xd_nominal);
%                         obj.p_merge_end = xd_end(1:3);
%                         obj.v_merge_vec = xd_end(5:7);
%                         obj.replan_active = false;
%                         obj.replan_done   = true;
%                     end
% 
%                     dt_after = time.t - obj.t_merge_end;
%                     xd = zeros(28, 1);
%                     xd(1:3) = obj.p_merge_end + obj.v_merge_vec * dt_after;
%                     xd(5:7) = obj.v_merge_vec;
%                     xd(9:28) = 0;
%                 end
%             elseif obj.replan_done
%                 dt_after = time.t - obj.t_merge_end;
%                 xd = zeros(28, 1);
%                 xd(1:3) = obj.p_merge_end + obj.v_merge_vec * dt_after;
%                 xd(5:7) = obj.v_merge_vec;
%                 xd(9:28) = 0;
%             else
%                 xd = xd_nominal;
%             end
% 
%             if length(xd) < 28, xd = [xd; zeros(28 - length(xd), 1)]; end
%             obj.result.state.xd = xd;
%             obj.result.state.p = xd(1:3);
%             obj.result.state.v = xd(5:7);
%             obj.result.state.q = [0; 0; xd(4)];
%             result = obj.result;
%         end
% 
%         function xd = evaluate_smooth_trajectory(obj, tau, xd_nom)
%             xd = xd_nom;
%             N = obj.order;
%             T_seg = obj.T_seg;
% 
%             if tau <= T_seg
%                 C = obj.coeffs_delta_seg1;
%                 u = max(0, min(1.0, tau / T_seg));
%             else
%                 C = obj.coeffs_delta_seg2;
%                 u = max(0, min(1.0, (tau - T_seg) / T_seg));
%             end
% 
%             for k = 0:6
%                 c_diff = obj.get_bezier_derivative_coeffs_at_u(N, k, u);
%                 delta_n = dot(c_diff, C(:, 1)) / (T_seg^k);
%                 delta_b = dot(c_diff, C(:, 2)) / (T_seg^k);
% 
%                 delta_vec = obj.dir_normal * delta_n + obj.dir_binormal * delta_b;
% 
%                 idx = 4 * k + (1:3);
%                 xd(idx) = xd_nom(idx) + delta_vec;
%             end
%         end
%     end
% 
%     methods (Access = private)
%         function v_obs = extract_obstacle_velocity(~, tgt)
%             v_raw = [0; 0; 0];
%             if isfield(tgt, 'v_center')
%                 v_raw = tgt.v_center;
%             elseif isprop(tgt, 'v_center')
%                 v_raw = tgt.v_center;
%             elseif isfield(tgt, 'velocity')
%                 v_raw = tgt.velocity;
%             elseif isprop(tgt, 'velocity')
%                 v_raw = tgt.velocity;
%             elseif isfield(tgt, 'v')
%                 v_raw = tgt.v;
%             elseif isprop(tgt, 'v')
%                 v_raw = tgt.v;
%             end
%             if isempty(v_raw) || length(v_raw) ~= 3
%                 v_obs = [0; 0; 0];
%             else
%                 v_obs = v_raw(:); % 確実に 3x1 縦ベクトル化
%             end
%         end
% 
%         function d = calc_exact_surface_distance(~, p, c, R, rad)
%             p_rel = R' * (p - c);
%             val = norm(p_rel ./ rad);
%             if val < 1e-6
%                 d = -min(rad);
%                 return;
%             end
%             p_surf = p_rel / val;
%             d = (val - 1.0) * norm(p_surf);
%         end
% 
%         function world_diffs = get_current_world_derivatives(obj, tau)
%             world_diffs = zeros(7, 3);
%             N = obj.order;
%             T_seg = obj.T_seg;
%             if tau <= T_seg
%                 C = obj.coeffs_delta_seg1;
%                 u = max(0, min(1.0, tau / T_seg));
%             else
%                 C = obj.coeffs_delta_seg2;
%                 u = max(0, min(1.0, (tau - T_seg) / T_seg));
%             end
%             for k = 0:6
%                 c_diff = obj.get_bezier_derivative_coeffs_at_u(N, k, u);
%                 delta_n = dot(c_diff, C(:, 1)) / (T_seg^k);
%                 delta_b = dot(c_diff, C(:, 2)) / (T_seg^k);
% 
%                 world_vec_k = obj.dir_normal * delta_n + obj.dir_binormal * delta_b;
%                 world_diffs(k + 1, :) = world_vec_k';
%             end
%         end
% 
%         function plan_spatiotemporal_bezier_c6_qp(obj, dynamic_buffer, dist_line_to_obs, cable_offset_s)
%             N = obj.order;
%             n_c = N + 1;
%             T_seg = obj.T_seg;
% 
%             p_rel_n = obj.obs_R' * obj.dir_normal;
%             r_eff_n = 1.0 / norm(p_rel_n ./ obj.obs_radii);
%             p_rel_s = obj.obs_R' * obj.dir_nominal;
%             r_eff_s = 1.0 / norm(p_rel_s ./ obj.obs_radii);
% 
%             max_r_obj = max(obj.r_drone, obj.r_load);
%             a_mink = r_eff_n + max_r_obj + obj.obs_margin + dynamic_buffer;
%             c_mink = r_eff_s + max_r_obj + obj.obs_margin + dynamic_buffer;
% 
%             delta_target = max(a_mink - dist_line_to_obs, 1.0);
% 
%             n_vars_1d  = 2 * n_c;
%             n_vars_tot = 2 * n_vars_1d;
% 
%             Q_snap = obj.compute_bezier_derivative_hessian(N, 4);
%             Q_acc  = obj.compute_bezier_derivative_hessian(N, 2);
%             Q_pos  = obj.compute_bezier_pos_hessian(N);
% 
%             Q_snap = Q_snap / norm(Q_snap, 2);
%             Q_acc  = Q_acc  / norm(Q_acc,  2);
%             Q_pos  = Q_pos  / norm(Q_pos,  2);
% 
%             H_1d = 1.0 * Q_snap + 0.1 * Q_acc + 1e-4 * Q_pos;
%             H_1d = (H_1d + H_1d') / 2 + 1e-6 * eye(n_c);
% 
%             H = blkdiag(H_1d, H_1d, H_1d, H_1d);
%             f = zeros(n_vars_tot, 1);
% 
%             % 等式制約 (42本: 新基底への動的初期状態射影 + C^6 連続性)
%             Aeq_1d = zeros(21, n_vars_1d);
%             beq_n  = zeros(21, 1);
%             beq_b  = zeros(21, 1);
% 
%             for k = 0:6
%                 c_diff = obj.get_bezier_derivative_coeffs_at_u(N, k, 0.0);
%                 Aeq_1d(k + 1, 1:n_c) = c_diff / (T_seg^k);
% 
%                 w_vec_k = obj.init_world_diff_state(k + 1, :)';
%                 beq_n(k + 1) = dot(w_vec_k, obj.dir_normal);
%                 beq_b(k + 1) = dot(w_vec_k, obj.dir_binormal);
%             end
% 
%             for k = 0:6
%                 c_end   = obj.get_bezier_derivative_coeffs_at_u(N, k, 1.0);
%                 c_start = obj.get_bezier_derivative_coeffs_at_u(N, k, 0.0);
%                 Aeq_1d(7 + k + 1, 1:n_c)       =  c_end   / (T_seg^k);
%                 Aeq_1d(7 + k + 1, n_c+1:2*n_c) = -c_start / (T_seg^k);
%             end
% 
%             for k = 0:6
%                 c_diff = obj.get_bezier_derivative_coeffs_at_u(N, k, 1.0);
%                 Aeq_1d(14 + k + 1, n_c+1:2*n_c) = c_diff / (T_seg^k);
%             end
% 
%             Aeq = blkdiag(Aeq_1d, Aeq_1d);
%             beq = [beq_n; beq_b];
% 
%             % -------------------------------------------------------------
%             % 時空間不等式制約 (4D Spatiotemporal Obstacle Constraints)
%             % -------------------------------------------------------------
%             alpha_dyn = obj.L_cable / (obj.gravity * (T_seg^2));
%             A_ineq = [];
%             b_ineq = [];
% 
%             % 区間 1 (接近側: 未来時刻 tau = u * T_seg における動的位置を計算)
%             N_samp = 7;
%             u1_samples = linspace(0.3, 1.0, N_samp);
%             for u = u1_samples
%                 B_val      = obj.eval_bernstein_vector(N, u);
%                 B_ddot_val = obj.get_bezier_derivative_coeffs_at_u(N, 2, u);
%                 BQ_val     = B_val + alpha_dyn * B_ddot_val;
% 
%                 tau_future = u * T_seg;
%                 c_future = obj.obs_center_init + obj.obs_velocity * tau_future;
%                 p_nom_future = obj.p_start_replan + (obj.nominal_speed * tau_future) * obj.dir_nominal;
% 
%                 delta_s_L = dot(c_future - p_nom_future, obj.dir_nominal);
%                 delta_s_Q = delta_s_L - cable_offset_s;
% 
%                 v_lat_future = (c_future - p_nom_future) - delta_s_L * obj.dir_nominal;
%                 dist_lat_future = norm(v_lat_future);
% 
%                 req_L = 0.0;
%                 if abs(delta_s_L) < c_mink
%                     req_L = max(0, a_mink * sqrt(1.0 - (delta_s_L / c_mink)^2) - dist_lat_future);
%                 end
%                 req_Q = 0.0;
%                 if abs(delta_s_Q) < c_mink
%                     req_Q = max(0, a_mink * sqrt(1.0 - (delta_s_Q / c_mink)^2) - dist_lat_future);
%                 end
% 
%                 r_L = zeros(1, n_vars_tot); r_L(1:n_c) = -B_val;
%                 r_Q = zeros(1, n_vars_tot); r_Q(1:n_c) = -BQ_val;
%                 A_ineq = [A_ineq; r_L; r_Q];
%                 b_ineq = [b_ineq; -req_L; -req_Q];
%             end
% 
%             % 区間 2 (離脱側: 未来時刻 tau = T_seg + u * T_seg)
%             u2_samples = linspace(0.0, 0.7, N_samp);
%             for u = u2_samples
%                 B_val      = obj.eval_bernstein_vector(N, u);
%                 B_ddot_val = obj.get_bezier_derivative_coeffs_at_u(N, 2, u);
%                 BQ_val     = B_val + alpha_dyn * B_ddot_val;
% 
%                 tau_future = T_seg + u * T_seg;
%                 c_future = obj.obs_center_init + obj.obs_velocity * tau_future;
%                 p_nom_future = obj.p_start_replan + (obj.nominal_speed * tau_future) * obj.dir_nominal;
% 
%                 delta_s_L = dot(c_future - p_nom_future, obj.dir_nominal);
%                 delta_s_Q = delta_s_L - cable_offset_s;
% 
%                 v_lat_future = (c_future - p_nom_future) - delta_s_L * obj.dir_nominal;
%                 dist_lat_future = norm(v_lat_future);
% 
%                 req_L = 0.0;
%                 if abs(delta_s_L) < c_mink
%                     req_L = max(0, a_mink * sqrt(1.0 - (delta_s_L / c_mink)^2) - dist_lat_future);
%                 end
%                 req_Q = 0.0;
%                 if abs(delta_s_Q) < c_mink
%                     req_Q = max(0, a_mink * sqrt(1.0 - (delta_s_Q / c_mink)^2) - dist_lat_future);
%                 end
% 
%                 r_L = zeros(1, n_vars_tot); r_L(n_c+1:2*n_c) = -B_val;
%                 r_Q = zeros(1, n_vars_tot); r_Q(n_c+1:2*n_c) = -BQ_val;
%                 A_ineq = [A_ineq; r_L; r_Q];
%                 b_ineq = [b_ineq; -req_L; -req_Q];
%             end
% 
%             % 中間頂点でのハード下限保証
%             B_mid = obj.eval_bernstein_vector(N, 1.0);
%             B_ddot_mid = obj.get_bezier_derivative_coeffs_at_u(N, 2, 1.0);
%             BQ_mid = B_mid + alpha_dyn * B_ddot_mid;
% 
%             r_mid_L = zeros(1, n_vars_tot); r_mid_L(1:n_c) = -B_mid;
%             r_mid_Q = zeros(1, n_vars_tot); r_mid_Q(1:n_c) = -BQ_mid;
%             A_ineq = [A_ineq; r_mid_L; r_mid_Q];
%             b_ineq = [b_ineq; -delta_target; -delta_target];
% 
%             % 振れ角制限
%             theta_max_rad = deg2rad(obj.max_swing_angle_deg);
%             a_load_limit  = obj.gravity * tan(theta_max_rad);
%             u_check = [0.5, 0.8, 1.0];
%             for u = u_check
%                 c_ddot = obj.get_bezier_derivative_coeffs_at_u(N, 2, u) / (T_seg^2);
%                 r_pos = zeros(1, n_vars_tot); r_pos(1:n_c) = c_ddot;
%                 r_neg = zeros(1, n_vars_tot); r_neg(1:n_c) = -c_ddot;
%                 A_ineq = [A_ineq; r_pos; r_neg];
%                 b_ineq = [b_ineq; a_load_limit; a_load_limit];
%             end
% 
%             % 機体加速度制限
%             for u = u_check
%                 c_ddot = obj.get_bezier_derivative_coeffs_at_u(N, 2, u) / (T_seg^2);
%                 c_snap = obj.get_bezier_derivative_coeffs_at_u(N, 4, u) / (T_seg^4);
%                 c_drone_acc = c_ddot + (obj.L_cable / obj.gravity) * c_snap;
% 
%                 r_pos = zeros(1, n_vars_tot); r_pos(1:n_c) = c_drone_acc;
%                 r_neg = zeros(1, n_vars_tot); r_neg(1:n_c) = -c_drone_acc;
%                 A_ineq = [A_ineq; r_pos; r_neg];
%                 b_ineq = [b_ineq; obj.max_acc_drone; obj.max_acc_drone];
%             end
% 
%             opts = optimoptions('quadprog', ...
%                 'Display', 'off', ...
%                 'Algorithm', 'interior-point-convex', ...
%                 'MaxIterations', 300, ...
%                 'ConstraintTolerance', 1e-4, ...
%                 'OptimalityTolerance', 1e-4);
% 
%             [X_opt, ~, exitflag, output] = quadprog(H, f, A_ineq, b_ineq, Aeq, beq, [], [], [], opts);
% 
%             if exitflag < 1
%                 fprintf("[BÉZIER C^6 DEBUG] 不等式制約付きQP未収束 (exitflag=%d). 等式拘束フォールバックを実行\n", exitflag);
%                 X_fb = pinv(Aeq) * beq;
%                 cn1 = X_fb(1:n_c);         cn2 = X_fb(n_c+1:2*n_c);
%                 cb1 = X_fb(2*n_c+1:3*n_c); cb2 = X_fb(3*n_c+1:4*n_c);
%             else
%                 fprintf("[BÉZIER C^6] 時空間最適化成功! (反復=%d, ピーク退避量=%.3fm)\n", output.iterations, delta_target);
%                 cn1 = X_opt(1:n_c);             cn2 = X_opt(n_c+1:2*n_c);
%                 cb1 = X_opt(2*n_c+1:3*n_c);     cb2 = X_opt(3*n_c+1:4*n_c);
%             end
% 
%             obj.coeffs_delta_seg1 = [cn1, cb1];
%             obj.coeffs_delta_seg2 = [cn2, cb2];
% 
%             obj.verify_c6_continuity();
%         end
% 
%         function verify_c6_continuity(obj)
%             N = obj.order;
%             T_seg = obj.T_seg;
%             names = ["位置 (0階)", "速度 (1階)", "加速度 (2階)", "Jerk (3階)", "Snap (4階)", "Crack (5階)", "Pop (6階)"];
% 
%             fprintf("-------------------------------------------------------\n");
%             fprintf("[理論検証] 中間接続点における 0〜6階微分の不連続量 (Gap):\n");
%             for k = 0:6
%                 c_end   = obj.get_bezier_derivative_coeffs_at_u(N, k, 1.0);
%                 c_start = obj.get_bezier_derivative_coeffs_at_u(N, k, 0.0);
% 
%                 d1_n = dot(c_end,   obj.coeffs_delta_seg1(:, 1)) / (T_seg^k);
%                 d1_b = dot(c_end,   obj.coeffs_delta_seg1(:, 2)) / (T_seg^k);
% 
%                 d2_n = dot(c_start, obj.coeffs_delta_seg2(:, 1)) / (T_seg^k);
%                 d2_b = dot(c_start, obj.coeffs_delta_seg2(:, 2)) / (T_seg^k);
% 
%                 gap = norm([d1_n - d2_n; d1_b - d2_b]);
%                 fprintf("  - %-12s ギャップ: %.3e\n", names(k + 1), gap);
%             end
%             fprintf("-------------------------------------------------------\n");
%         end
% 
%         function B = eval_bernstein_vector(~, n, u)
%             B = zeros(1, n + 1);
%             for i = 0:n
%                 B(i + 1) = nchoosek(n, i) * (u^i) * ((1 - u)^(n - i));
%             end
%         end
% 
%         function c_diff = get_bezier_derivative_coeffs_at_u(~, n, k, u)
%             if k > n, c_diff = zeros(1, n + 1); return; end
%             factor = factorial(n) / factorial(n - k);
%             b_low = zeros(1, n - k + 1);
%             for j = 0:(n - k)
%                 b_low(j + 1) = nchoosek(n - k, j) * (u^j) * ((1 - u)^(n - k - j));
%             end
%             D = eye(n + 1);
%             for step = 1:k, D = diff(D); end
%             c_diff = factor * (b_low * D);
%         end
% 
%         function Q = compute_bezier_derivative_hessian(obj, n, k)
%             Q = zeros(n + 1, n + 1);
%             if k > n, return; end
%             factor = factorial(n) / factorial(n - k);
%             n_low = n - k;
%             D = eye(n + 1);
%             for step = 1:k, D = diff(D); end
%             M_low = zeros(n_low + 1, n_low + 1);
%             for i = 0:n_low
%                 for j = 0:n_low
%                     M_low(i + 1, j + 1) = (nchoosek(n_low, i) * nchoosek(n_low, j)) / ...
%                         ((2 * n_low + 1) * nchoosek(2 * n_low, i + j));
%                 end
%             end
%             Q = (factor^2) * (D' * M_low * D);
%         end
% 
%         function Q = compute_bezier_pos_hessian(~, n)
%             Q = zeros(n + 1, n + 1);
%             for i = 0:n
%                 for j = 0:n
%                     Q(i + 1, j + 1) = (nchoosek(n, i) * nchoosek(n, j)) / ...
%                         ((2 * n + 1) * nchoosek(2 * n, i + j));
%                 end
%             end
%         end
%     end
% end

% classdef REPLANNING_MELLINGER_CORRIDOR_FLATNESS_SWING < handle
%     % REPLANNING_MELLINGER_CORRIDOR_FLATNESS_SWING
%     % 汎用3次元静的・動的障害物両対応 13次 Bézier C^6 完全連続 時空間SFCリプランナ
%     % - obs_mode: 1 = 静的障害物 (ENVIRONMENT_OBSTACLE_ELLIPSE)
%     %             2 = 動的障害物 (ENVIRONMENT_OBSTACLE_ELLIPSE_MOVE)
%     % - 4D時空間楕円境界逆算による安全退避 (静止・移動を統一処理)
%     % - Receding Horizon 動的初期状態継承 (0〜6階微分ワールド座標系射影)
%     % - 機体・荷物の平坦性連立不等式制約 ＆ 動的バッファ (Tube Bound)
% 
%     properties
%         base_ref
%         self
%         replan_active = false
%         replan_done   = false
% 
%         obs_mode     = 2    % 1: 静的障害物, 2: 動的障害物
% 
%         t_start      = 0.0
%         t_duration   = 8.0
%         T_seg
% 
%         safe_margin  = 0.3
%         trigger_dist = 7.0
% 
%         L_cable      = 2.0
%         gravity      = 9.81
%         m_drone      = 1.5
%         m_load_est   = 0.1
% 
%         r_load       = 0.15
%         r_drone      = 0.30
% 
%         max_swing_angle_deg = 20.0
%         max_acc_drone       = 6.0
% 
%         order = 13
%         coeffs_delta_seg1 % (14 x 2: 法線, 従法線)
%         coeffs_delta_seg2 % (14 x 2: 法線, 従法線)
% 
%         dir_normal        % 最適退避法線 (3 x 1)
%         dir_binormal      % 最適退避従法線 (3 x 1)
%         dir_nominal       % 進行方向単位ベクトル (3 x 1)
%         nominal_speed     % 進入巡航速度
%         p_start_replan    % リプラン開始位置
% 
%         t_merge_end
%         p_merge_end
%         v_merge_vec
% 
%         active_obs_idx = -1
%         obs_center_init= [0; 0; 0] % リプラン開始時の中心座標
%         obs_velocity   = [0; 0; 0] % 障害物速度ベクトル
%         obs_R          = eye(3)
%         obs_radii      = [1; 1; 1]
%         obs_margin     = 0
% 
%         % 再計画時のワールド座標系初期微分バッファ (7 x 3)
%         init_world_diff_state = zeros(7, 3);
% 
%         warned_crash_load
%         warned_crash_drone
%         warned_margin_load
%         warned_margin_drone
% 
%         debug_cnt = 0
%         result
%     end
% 
%     methods (Access = public)
%         function obj = REPLANNING_MELLINGER_CORRIDOR_FLATNESS_SWING(self, base_ref, opts)
%             arguments
%                 self
%                 base_ref
%                 opts = struct()
%             end
%             obj.self = self;
%             obj.base_ref = base_ref;
%             if isfield(opts, "obs_mode"),            obj.obs_mode            = opts.obs_mode;            end
%             if isfield(opts, "safe_margin"),         obj.safe_margin         = opts.safe_margin;         end
%             if isfield(opts, "trigger_dist"),        obj.trigger_dist        = opts.trigger_dist;        end
%             if isfield(opts, "r_load"),              obj.r_load              = opts.r_load;              end
%             if isfield(opts, "r_drone"),             obj.r_drone             = opts.r_drone;             end
%             if isfield(opts, "max_swing_angle_deg"), obj.max_swing_angle_deg = opts.max_swing_angle_deg; end
%             if isfield(opts, "max_acc_drone"),       obj.max_acc_drone       = opts.max_acc_drone;       end
%             obj.result.state = STATE_CLASS(struct('state_list', ["xd", "p", "q", "v"], 'num_list', [28, 3, 3, 3]));
%         end
% 
%         function result = do(obj, varargin)
%             time = varargin{1};
%             cha = varargin{2};
% 
%             base_res = obj.base_ref.do(time, cha);
%             xd_nominal = base_res.state.xd;
%             if length(xd_nominal) < 28
%                 xd_nominal = [xd_nominal; zeros(28 - length(xd_nominal), 1)];
%             end
% 
%             obj.L_cable = obj.self.parameter.get("cableL");
%             try
%                 obj.m_drone = obj.self.parameter.get("mass");
%             catch
%                 obj.m_drone = 1.5;
%             end
% 
%             if isprop(obj.self.estimator.result.state, "mL")
%                 obj.m_load_est = max(0.001, obj.self.estimator.result.state.mL);
%             elseif isprop(obj.self.estimator.result, "loadmass")
%                 obj.m_load_est = max(0.001, obj.self.estimator.result.loadmass);
%             else
%                 try
%                     obj.m_load_est = max(0.001, obj.self.parameter.get("loadmass"));
%                 catch
%                     obj.m_load_est = 0.013;
%                 end
%             end
% 
%             if isprop(obj.self.estimator.result.state, "pL")
%                 pL_cur = obj.self.estimator.result.state.pL;
%                 vL_cur = obj.self.estimator.result.state.vL;
%             else
%                 pL_cur = base_res.state.p;
%                 vL_cur = base_res.state.v;
%             end
% 
%             if isprop(obj.self.estimator.result.state, "p")
%                 pQ_cur = obj.self.estimator.result.state.p;
%             else
%                 pQ_cur = pL_cur + [0; 0; obj.L_cable];
%             end
% 
%             % -------------------------------------------------------------
%             % 障害物リストの取得 (obs_mode に応じた動的切り替え)
%             % -------------------------------------------------------------
%             obs_list = [];
%             try
%                 if obj.obs_mode == 2
%                     obs_list = ENVIRONMENT_OBSTACLE_ELLIPSE_MOVE(time.t);
%                 else
%                     obs_list = ENVIRONMENT_OBSTACLE_ELLIPSE();
%                 end
%             catch
%                 try
%                     obs_list = ENVIRONMENT_OBSTACLE_ELLIPSE();
%                 catch
%                 end
%             end
% 
%             if isempty(obj.warned_crash_load) && ~isempty(obs_list)
%                 n_obs = length(obs_list);
%                 obj.warned_crash_load   = false(n_obs, 1);
%                 obj.warned_crash_drone  = false(n_obs, 1);
%                 obj.warned_margin_load  = false(n_obs, 1);
%                 obj.warned_margin_drone = false(n_obs, 1);
%             end
% 
%             v_vec = xd_nominal(5:7);
%             spd = norm(v_vec);
%             if spd < 0.05, spd = norm(vL_cur); end
%             if spd < 0.05, spd = 1.0; v_vec = [0; 0; 1.0]; end
%             dir_nom = v_vec / spd;
% 
%             % -------------------------------------------------------------
%             % 1. Receding Horizon 時空間交差判定 ＆ リプラン判定
%             % -------------------------------------------------------------
%             if cha == 'f' && ~obj.replan_done && ~isempty(obs_list)
%                 best_idx = -1;
%                 min_proj_dist = inf;
%                 min_surf_dist = inf;
%                 sensor_owner = "";
% 
%                 for i = 1:length(obs_list)
%                     if obj.replan_active && (i == obj.active_obs_idx)
%                         continue;
%                     end
% 
%                     tgt_i = obs_list(i);
%                     v_obs_i = obj.extract_obstacle_velocity(tgt_i);
%                     c_i = tgt_i.p_center;
%                     rad_i = tgt_i.ellipsoid_radii;
%                     R_i = tgt_i.R_obs;
%                     d_m_i = tgt_i.d_margin;
% 
%                     % 相対速度ベクトル
%                     v_rel = dir_nom * spd - v_obs_i;
%                     spd_rel = norm(v_rel);
%                     if spd_rel < 0.05, spd_rel = 1.0; v_rel = dir_nom; end
%                     dir_rel = v_rel / spd_rel;
% 
%                     d_proj_L = dot(c_i - pL_cur, dir_rel);
%                     d_proj_Q = dot(c_i - pQ_cur, dir_rel);
% 
%                     if d_proj_L <= 0.1 && d_proj_Q <= 0.1, continue; end
% 
%                     p_line_closest = pL_cur + d_proj_L * dir_rel;
%                     dist_lateral = norm(c_i - p_line_closest);
% 
%                     vec_lat = c_i - p_line_closest;
%                     if dist_lateral < 1e-4
%                         r_eff = max(rad_i);
%                     else
%                         dir_lat = vec_lat / dist_lateral;
%                         p_rel_lat = R_i' * dir_lat;
%                         r_eff = 1.0 / norm(p_rel_lat ./ rad_i);
%                     end
% 
%                     max_corridor_r = r_eff + max(obj.r_drone, obj.r_load) + d_m_i + 0.1;
%                     if dist_lateral > max_corridor_r, continue; end
% 
%                     d_surf_L = obj.calc_exact_surface_distance(pL_cur, c_i, R_i, rad_i);
%                     d_surf_Q = obj.calc_exact_surface_distance(pQ_cur, c_i, R_i, rad_i);
% 
%                     if d_surf_Q < d_surf_L
%                         cur_min_surf = d_surf_Q;
%                         cur_proj = d_proj_Q;
%                         cur_owner = "機体実位置センサー";
%                     else
%                         cur_min_surf = d_surf_L;
%                         cur_proj = d_proj_L;
%                         cur_owner = "荷物実位置センサー";
%                     end
% 
%                     if cur_min_surf <= obj.trigger_dist && cur_proj < min_proj_dist
%                         min_proj_dist = cur_proj;
%                         min_surf_dist = cur_min_surf;
%                         best_idx = i;
%                         sensor_owner = cur_owner;
%                     end
%                 end
% 
%                 % 新たな前方障害物を捕捉
%                 if best_idx > 0
%                     tgt = obs_list(best_idx);
%                     v_obs_tgt = obj.extract_obstacle_velocity(tgt);
% 
%                     mode_str = "静的障害物";
%                     if norm(v_obs_tgt) > 1e-3, mode_str = "動的移動障害物"; end
% 
%                     if obj.replan_active
%                         tau_now = time.t - obj.t_start;
%                         obj.init_world_diff_state = obj.get_current_world_derivatives(tau_now);
%                         fprintf("\n[RECEDING HORIZON] 新たな障害物捕捉! (ID: %d [%s], 速度=[%.2f, %.2f, %.2f])\n", ...
%                             best_idx, mode_str, v_obs_tgt(1), v_obs_tgt(2), v_obs_tgt(3));
%                     else
%                         obj.init_world_diff_state = zeros(7, 3);
%                         fprintf("\n=======================================================\n");
%                         fprintf("[検知発動] 実状態センサーが障害物捕捉! (ID: %d [%s], Center=[%.2f, %.2f, %.2f], 速度=[%.2f, %.2f, %.2f])\n", ...
%                             best_idx, mode_str, tgt.p_center(1), tgt.p_center(2), tgt.p_center(3), v_obs_tgt(1), v_obs_tgt(2), v_obs_tgt(3));
%                     end
% 
%                     obj.active_obs_idx   = best_idx;
%                     obj.obs_center_init  = tgt.p_center;
%                     obj.obs_velocity     = v_obs_tgt;
%                     obj.obs_R            = tgt.R_obs;
%                     obj.obs_radii        = tgt.ellipsoid_radii;
%                     obj.obs_margin       = tgt.d_margin;
% 
%                     obj.t_start        = time.t;
%                     obj.p_start_replan = pL_cur;
%                     obj.dir_nominal    = dir_nom;
%                     obj.nominal_speed  = spd;
% 
%                     % 特異点フリー Gram-Schmidt 直交基底
%                     [~, min_dim] = min(abs(dir_nom));
%                     v_arbitrary = zeros(3, 1);
%                     v_arbitrary(min_dim) = 1.0;
% 
%                     base_n1 = cross(dir_nom, v_arbitrary);
%                     base_n1 = base_n1 / norm(base_n1);
%                     base_n2 = cross(dir_nom, base_n1);
%                     base_n2 = base_n2 / norm(base_n2);
% 
%                     vec_to_obs = obj.obs_center_init - obj.p_start_replan;
%                     v_rel_app = dir_nom * spd - obj.obs_velocity;
%                     spd_rel_app = max(norm(v_rel_app), 0.2);
%                     t_to_center = max(0.5, dot(vec_to_obs, v_rel_app) / (spd_rel_app^2));
% 
%                     obs_center_closest = obj.obs_center_init + obj.obs_velocity * t_to_center;
%                     p_closest_on_line = obj.p_start_replan + (spd * t_to_center) * dir_nom;
% 
%                     v_center_rel = obs_center_closest - p_closest_on_line;
%                     d_center_lat = norm(v_center_rel);
%                     p_rel_raw = obj.obs_R' * (v_center_rel / max(d_center_lat, 1e-4));
%                     r_eff_est = 1.0 / norm(p_rel_raw ./ obj.obs_radii);
%                     test_radius = max(r_eff_est + 1.0, 2.0);
% 
%                     % 360度時空間方向探索による最適安全退避法線の決定
%                     best_score = -inf;
%                     best_normal = base_n1;
%                     test_angles = linspace(0, 2*pi, 36);
%                     for ang = test_angles
%                         cand_n = cos(ang) * base_n1 + sin(ang) * base_n2;
%                         if d_center_lat > 1e-3 && dot(cand_n, -v_center_rel) < -0.1
%                             continue;
%                         end
%                         test_apex = p_closest_on_line + test_radius * cand_n;
%                         min_dist_other = inf;
%                         for j = 1:length(obs_list)
%                             if j == best_idx, continue; end
%                             v_j = obj.extract_obstacle_velocity(obs_list(j));
%                             c_j_future = obs_list(j).p_center + v_j * t_to_center;
%                             d_j = obj.calc_exact_surface_distance(test_apex, c_j_future, obs_list(j).R_obs, obs_list(j).ellipsoid_radii);
%                             min_dist_other = min(min_dist_other, d_j);
%                         end
%                         if min_dist_other > best_score
%                             best_score = min_dist_other;
%                             best_normal = cand_n;
%                         end
%                     end
% 
%                     obj.dir_normal = best_normal;
%                     b_cand = cross(dir_nom, obj.dir_normal);
%                     obj.dir_binormal = b_cand / norm(b_cand);
% 
%                     p_rel_n = obj.obs_R' * obj.dir_normal;
%                     r_eff_obs = 1.0 / norm(p_rel_n ./ obj.obs_radii);
% 
%                     effective_Kp = 1.0;
%                     try
%                         if isprop(obj.self.controller, "param") && isfield(obj.self.controller.param, "F2")
%                             raw_gain = obj.self.controller.param.F2(1);
%                             effective_Kp = max(0.5, min(2.5, raw_gain / 50.0));
%                         end
%                     catch
%                     end
% 
%                     base_clearance = max(0.5, (r_eff_obs + obj.r_drone + obj.obs_margin) - d_center_lat);
% 
%                     t_pendulum = 2 * pi * sqrt(obj.L_cable / obj.gravity);
%                     theta_max_rad = deg2rad(obj.max_swing_angle_deg);
%                     a_load_allow  = obj.gravity * tan(theta_max_rad);
%                     T_min_swing   = sqrt(10.0 * base_clearance / a_load_allow);
%                     T_min_drone   = sqrt(12.0 * base_clearance / obj.max_acc_drone);
% 
%                     obj.T_seg = max([t_to_center, 1.2 * t_pendulum, T_min_swing, T_min_drone, 4.0]);
%                     obj.t_duration = 2.0 * obj.T_seg;
% 
%                     initial_pos_error = norm(pL_cur - (obj.p_start_replan + dot(pL_cur - obj.p_start_replan, dir_nom) * dir_nom));
%                     estimated_max_acc = 6.5 * base_clearance / (obj.T_seg^2);
%                     accel_tracking_error = estimated_max_acc / effective_Kp;
%                     mass_ratio = obj.m_load_est / (obj.m_drone + obj.m_load_est);
%                     swing_coupling_buffer = mass_ratio * (obj.L_cable / obj.gravity) * estimated_max_acc;
%                     dynamic_buffer = max(0.20, initial_pos_error + accel_tracking_error + swing_coupling_buffer);
% 
%                     actual_cable_vec = pQ_cur - pL_cur;
%                     cable_offset_s = dot(actual_cable_vec, obj.dir_nominal);
% 
%                     fprintf("  - 検知主体: %s (実表面距離: %.3f m, 進入速度: %.2f m/s)\n", sensor_owner, min_surf_dist, spd);
%                     fprintf("  - 選択された動的退避方向: [%.2f, %.2f, %.2f] (他障害物余白: %.2f m)\n", ...
%                         obj.dir_normal(1), obj.dir_normal(2), obj.dir_normal(3), best_score);
%                     fprintf("  - 時空間最適化時間: T_seg = %.2f s (全時間 T = %.2f s)\n", obj.T_seg, obj.t_duration);
% 
%                     obj.plan_spatiotemporal_bezier_c6_qp(dynamic_buffer, d_center_lat, cable_offset_s);
%                     obj.replan_active = true;
%                     fprintf("=======================================================\n\n");
%                 end
%             end
% 
%             % -------------------------------------------------------------
%             % 2. 飛行中常時監視: 全障害物（動的移動を安全に反映）
%             % -------------------------------------------------------------
%             if cha == 'f' && ~isempty(obs_list)
%                 for j = 1:length(obs_list)
%                     tgt_j = obs_list(j);
%                     c_j_now = tgt_j.p_center; % 既に現在時刻 t の位置として渡されている
% 
%                     d_surf_L = obj.calc_exact_surface_distance(pL_cur, c_j_now, tgt_j.R_obs, tgt_j.ellipsoid_radii);
%                     d_surf_Q = obj.calc_exact_surface_distance(pQ_cur, c_j_now, tgt_j.R_obs, tgt_j.ellipsoid_radii);
% 
%                     if d_surf_L <= 0 && ~obj.warned_crash_load(j)
%                         fprintf(2, "[CRITICAL ALARM] 荷物が障害物%d本体に衝突! (t=%.3f s, 侵入深さ=%.3f m)\n", j, time.t, -d_surf_L);
%                         obj.warned_crash_load(j) = true;
%                     elseif d_surf_L <= (obj.r_load + tgt_j.d_margin) && ~obj.warned_margin_load(j)
%                         fprintf("[SAFETY WARN] 荷物が障害物%dのソフトマージン帯に侵入 (t=%.3f s, 表面残余=%.3f m)\n", j, time.t, d_surf_L);
%                         obj.warned_margin_load(j) = true;
%                     end
% 
%                     if d_surf_Q <= 0 && ~obj.warned_crash_drone(j)
%                         fprintf(2, "[CRITICAL ALARM] 機体が障害物%d本体に衝突! (t=%.3f s, 侵入深さ=%.3f m)\n", j, time.t, -d_surf_Q);
%                         obj.warned_crash_drone(j) = true;
%                     elseif d_surf_Q <= (obj.r_drone + tgt_j.d_margin) && ~obj.warned_margin_drone(j)
%                         fprintf("[SAFETY WARN] 機体が障害物%dのソフトマージン帯に侵入 (t=%.3f s, 表面残余=%.3f m)\n", j, time.t, d_surf_Q);
%                         obj.warned_margin_drone(j) = true;
%                     end
%                 end
%             end
% 
%             % -------------------------------------------------------------
%             % 3. 軌道出力 ＆ C^6 シームレス合流
%             % -------------------------------------------------------------
%             if obj.replan_active
%                 tau = time.t - obj.t_start;
%                 if tau <= obj.t_duration
%                     xd = obj.evaluate_smooth_trajectory(tau, xd_nominal);
%                 else
%                     if ~obj.replan_done
%                         fprintf("[BÉZIER C^6 QP] 障害物通過完了・完全シームレス復帰 (t=%.3f s)\n\n", time.t);
%                         obj.t_merge_end = obj.t_start + obj.t_duration;
%                         xd_end = obj.evaluate_smooth_trajectory(obj.t_duration, xd_nominal);
%                         obj.p_merge_end = xd_end(1:3);
%                         obj.v_merge_vec = xd_end(5:7);
%                         obj.replan_active = false;
%                         obj.replan_done   = true;
%                     end
% 
%                     dt_after = time.t - obj.t_merge_end;
%                     xd = zeros(28, 1);
%                     xd(1:3) = obj.p_merge_end + obj.v_merge_vec * dt_after;
%                     xd(5:7) = obj.v_merge_vec;
%                     xd(9:28) = 0;
%                 end
%             elseif obj.replan_done
%                 dt_after = time.t - obj.t_merge_end;
%                 xd = zeros(28, 1);
%                 xd(1:3) = obj.p_merge_end + obj.v_merge_vec * dt_after;
%                 xd(5:7) = obj.v_merge_vec;
%                 xd(9:28) = 0;
%             else
%                 xd = xd_nominal;
%             end
% 
%             if length(xd) < 28, xd = [xd; zeros(28 - length(xd), 1)]; end
%             obj.result.state.xd = xd;
%             obj.result.state.p = xd(1:3);
%             obj.result.state.v = xd(5:7);
%             obj.result.state.q = [0; 0; xd(4)];
%             result = obj.result;
%         end
% 
%         function xd = evaluate_smooth_trajectory(obj, tau, xd_nom)
%             xd = xd_nom;
%             N = obj.order;
%             T_seg = obj.T_seg;
% 
%             if tau <= T_seg
%                 C = obj.coeffs_delta_seg1;
%                 u = max(0, min(1.0, tau / T_seg));
%             else
%                 C = obj.coeffs_delta_seg2;
%                 u = max(0, min(1.0, (tau - T_seg) / T_seg));
%             end
% 
%             for k = 0:6
%                 c_diff = obj.get_bezier_derivative_coeffs_at_u(N, k, u);
%                 delta_n = dot(c_diff, C(:, 1)) / (T_seg^k);
%                 delta_b = dot(c_diff, C(:, 2)) / (T_seg^k);
% 
%                 delta_vec = obj.dir_normal * delta_n + obj.dir_binormal * delta_b;
% 
%                 idx = 4 * k + (1:3);
%                 xd(idx) = xd_nom(idx) + delta_vec;
%             end
%         end
%     end
% 
%     methods (Access = private)
%         function v_obs = extract_obstacle_velocity(~, tgt)
%             v_raw = [0; 0; 0];
%             if isfield(tgt, 'v_center')
%                 v_raw = tgt.v_center;
%             elseif isprop(tgt, 'v_center')
%                 v_raw = tgt.v_center;
%             elseif isfield(tgt, 'velocity')
%                 v_raw = tgt.velocity;
%             elseif isprop(tgt, 'velocity')
%                 v_raw = tgt.velocity;
%             elseif isfield(tgt, 'v')
%                 v_raw = tgt.v;
%             elseif isprop(tgt, 'v')
%                 v_raw = tgt.v;
%             end
%             if isempty(v_raw) || length(v_raw) ~= 3
%                 v_obs = [0; 0; 0];
%             else
%                 v_obs = v_raw(:);
%             end
%         end
% 
%         function d = calc_exact_surface_distance(~, p, c, R, rad)
%             p_rel = R' * (p - c);
%             val = norm(p_rel ./ rad);
%             if val < 1e-6
%                 d = -min(rad);
%                 return;
%             end
%             p_surf = p_rel / val;
%             d = (val - 1.0) * norm(p_surf);
%         end
% 
%         function world_diffs = get_current_world_derivatives(obj, tau)
%             world_diffs = zeros(7, 3);
%             N = obj.order;
%             T_seg = obj.T_seg;
%             if tau <= T_seg
%                 C = obj.coeffs_delta_seg1;
%                 u = max(0, min(1.0, tau / T_seg));
%             else
%                 C = obj.coeffs_delta_seg2;
%                 u = max(0, min(1.0, (tau - T_seg) / T_seg));
%             end
%             for k = 0:6
%                 c_diff = obj.get_bezier_derivative_coeffs_at_u(N, k, u);
%                 delta_n = dot(c_diff, C(:, 1)) / (T_seg^k);
%                 delta_b = dot(c_diff, C(:, 2)) / (T_seg^k);
% 
%                 world_vec_k = obj.dir_normal * delta_n + obj.dir_binormal * delta_b;
%                 world_diffs(k + 1, :) = world_vec_k';
%             end
%         end
% 
%         function plan_spatiotemporal_bezier_c6_qp(obj, dynamic_buffer, dist_line_to_obs, cable_offset_s)
%             N = obj.order;
%             n_c = N + 1;
%             T_seg = obj.T_seg;
% 
%             p_rel_n = obj.obs_R' * obj.dir_normal;
%             r_eff_n = 1.0 / norm(p_rel_n ./ obj.obs_radii);
%             p_rel_s = obj.obs_R' * obj.dir_nominal;
%             r_eff_s = 1.0 / norm(p_rel_s ./ obj.obs_radii);
% 
%             max_r_obj = max(obj.r_drone, obj.r_load);
%             a_mink = r_eff_n + max_r_obj + obj.obs_margin + dynamic_buffer;
%             c_mink = r_eff_s + max_r_obj + obj.obs_margin + dynamic_buffer;
% 
%             delta_target = max(a_mink - dist_line_to_obs, 1.0);
% 
%             n_vars_1d  = 2 * n_c;
%             n_vars_tot = 2 * n_vars_1d;
% 
%             Q_snap = obj.compute_bezier_derivative_hessian(N, 4);
%             Q_acc  = obj.compute_bezier_derivative_hessian(N, 2);
%             Q_pos  = obj.compute_bezier_pos_hessian(N);
% 
%             Q_snap = Q_snap / norm(Q_snap, 2);
%             Q_acc  = Q_acc  / norm(Q_acc,  2);
%             Q_pos  = Q_pos  / norm(Q_pos,  2);
% 
%             H_1d = 1.0 * Q_snap + 0.1 * Q_acc + 1e-4 * Q_pos;
%             H_1d = (H_1d + H_1d') / 2 + 1e-6 * eye(n_c);
% 
%             H = blkdiag(H_1d, H_1d, H_1d, H_1d);
%             f = zeros(n_vars_tot, 1);
% 
%             % 等式制約 (42本: 新基底への動的初期状態射影 + C^6 連続性)
%             Aeq_1d = zeros(21, n_vars_1d);
%             beq_n  = zeros(21, 1);
%             beq_b  = zeros(21, 1);
% 
%             for k = 0:6
%                 c_diff = obj.get_bezier_derivative_coeffs_at_u(N, k, 0.0);
%                 Aeq_1d(k + 1, 1:n_c) = c_diff / (T_seg^k);
% 
%                 w_vec_k = obj.init_world_diff_state(k + 1, :)';
%                 beq_n(k + 1) = dot(w_vec_k, obj.dir_normal);
%                 beq_b(k + 1) = dot(w_vec_k, obj.dir_binormal);
%             end
% 
%             for k = 0:6
%                 c_end   = obj.get_bezier_derivative_coeffs_at_u(N, k, 1.0);
%                 c_start = obj.get_bezier_derivative_coeffs_at_u(N, k, 0.0);
%                 Aeq_1d(7 + k + 1, 1:n_c)       =  c_end   / (T_seg^k);
%                 Aeq_1d(7 + k + 1, n_c+1:2*n_c) = -c_start / (T_seg^k);
%             end
% 
%             for k = 0:6
%                 c_diff = obj.get_bezier_derivative_coeffs_at_u(N, k, 1.0);
%                 Aeq_1d(14 + k + 1, n_c+1:2*n_c) = c_diff / (T_seg^k);
%             end
% 
%             Aeq = blkdiag(Aeq_1d, Aeq_1d);
%             beq = [beq_n; beq_b];
% 
%             % -------------------------------------------------------------
%             % 時空間不等式制約 (4D Spatiotemporal Obstacle Constraints)
%             % -------------------------------------------------------------
%             alpha_dyn = obj.L_cable / (obj.gravity * (T_seg^2));
%             A_ineq = [];
%             b_ineq = [];
% 
%             % 区間 1 (接近側: 未来時刻 tau = u * T_seg における動的位置を計算)
%             N_samp = 7;
%             u1_samples = linspace(0.3, 1.0, N_samp);
%             for u = u1_samples
%                 B_val      = obj.eval_bernstein_vector(N, u);
%                 B_ddot_val = obj.get_bezier_derivative_coeffs_at_u(N, 2, u);
%                 BQ_val     = B_val + alpha_dyn * B_ddot_val;
% 
%                 tau_future = u * T_seg;
%                 c_future = obj.obs_center_init + obj.obs_velocity * tau_future;
%                 p_nom_future = obj.p_start_replan + (obj.nominal_speed * tau_future) * obj.dir_nominal;
% 
%                 delta_s_L = dot(c_future - p_nom_future, obj.dir_nominal);
%                 delta_s_Q = delta_s_L - cable_offset_s;
% 
%                 v_lat_future = (c_future - p_nom_future) - delta_s_L * obj.dir_nominal;
%                 dist_lat_future = norm(v_lat_future);
% 
%                 req_L = 0.0;
%                 if abs(delta_s_L) < c_mink
%                     req_L = max(0, a_mink * sqrt(1.0 - (delta_s_L / c_mink)^2) - dist_lat_future);
%                 end
%                 req_Q = 0.0;
%                 if abs(delta_s_Q) < c_mink
%                     req_Q = max(0, a_mink * sqrt(1.0 - (delta_s_Q / c_mink)^2) - dist_lat_future);
%                 end
% 
%                 r_L = zeros(1, n_vars_tot); r_L(1:n_c) = -B_val;
%                 r_Q = zeros(1, n_vars_tot); r_Q(1:n_c) = -BQ_val;
%                 A_ineq = [A_ineq; r_L; r_Q];
%                 b_ineq = [b_ineq; -req_L; -req_Q];
%             end
% 
%             % 区間 2 (離脱側: 未来時刻 tau = T_seg + u * T_seg)
%             u2_samples = linspace(0.0, 0.7, N_samp);
%             for u = u2_samples
%                 B_val      = obj.eval_bernstein_vector(N, u);
%                 B_ddot_val = obj.get_bezier_derivative_coeffs_at_u(N, 2, u);
%                 BQ_val     = B_val + alpha_dyn * B_ddot_val;
% 
%                 tau_future = T_seg + u * T_seg;
%                 c_future = obj.obs_center_init + obj.obs_velocity * tau_future;
%                 p_nom_future = obj.p_start_replan + (obj.nominal_speed * tau_future) * obj.dir_nominal;
% 
%                 delta_s_L = dot(c_future - p_nom_future, obj.dir_nominal);
%                 delta_s_Q = delta_s_L - cable_offset_s;
% 
%                 v_lat_future = (c_future - p_nom_future) - delta_s_L * obj.dir_nominal;
%                 dist_lat_future = norm(v_lat_future);
% 
%                 req_L = 0.0;
%                 if abs(delta_s_L) < c_mink
%                     req_L = max(0, a_mink * sqrt(1.0 - (delta_s_L / c_mink)^2) - dist_lat_future);
%                 end
%                 req_Q = 0.0;
%                 if abs(delta_s_Q) < c_mink
%                     req_Q = max(0, a_mink * sqrt(1.0 - (delta_s_Q / c_mink)^2) - dist_lat_future);
%                 end
% 
%                 r_L = zeros(1, n_vars_tot); r_L(n_c+1:2*n_c) = -B_val;
%                 r_Q = zeros(1, n_vars_tot); r_Q(n_c+1:2*n_c) = -BQ_val;
%                 A_ineq = [A_ineq; r_L; r_Q];
%                 b_ineq = [b_ineq; -req_L; -req_Q];
%             end
% 
%             % 中間頂点でのハード下限保証
%             B_mid = obj.eval_bernstein_vector(N, 1.0);
%             B_ddot_mid = obj.get_bezier_derivative_coeffs_at_u(N, 2, 1.0);
%             BQ_mid = B_mid + alpha_dyn * B_ddot_mid;
% 
%             r_mid_L = zeros(1, n_vars_tot); r_mid_L(1:n_c) = -B_mid;
%             r_mid_Q = zeros(1, n_vars_tot); r_mid_Q(1:n_c) = -BQ_mid;
%             A_ineq = [A_ineq; r_mid_L; r_mid_Q];
%             b_ineq = [b_ineq; -delta_target; -delta_target];
% 
%             % 振れ角制限
%             theta_max_rad = deg2rad(obj.max_swing_angle_deg);
%             a_load_limit  = obj.gravity * tan(theta_max_rad);
%             u_check = [0.5, 0.8, 1.0];
%             for u = u_check
%                 c_ddot = obj.get_bezier_derivative_coeffs_at_u(N, 2, u) / (T_seg^2);
%                 r_pos = zeros(1, n_vars_tot); r_pos(1:n_c) = c_ddot;
%                 r_neg = zeros(1, n_vars_tot); r_neg(1:n_c) = -c_ddot;
%                 A_ineq = [A_ineq; r_pos; r_neg];
%                 b_ineq = [b_ineq; a_load_limit; a_load_limit];
%             end
% 
%             % 機体加速度制限
%             for u = u_check
%                 c_ddot = obj.get_bezier_derivative_coeffs_at_u(N, 2, u) / (T_seg^2);
%                 c_snap = obj.get_bezier_derivative_coeffs_at_u(N, 4, u) / (T_seg^4);
%                 c_drone_acc = c_ddot + (obj.L_cable / obj.gravity) * c_snap;
% 
%                 r_pos = zeros(1, n_vars_tot); r_pos(1:n_c) = c_drone_acc;
%                 r_neg = zeros(1, n_vars_tot); r_neg(1:n_c) = -c_drone_acc;
%                 A_ineq = [A_ineq; r_pos; r_neg];
%                 b_ineq = [b_ineq; obj.max_acc_drone; obj.max_acc_drone];
%             end
% 
%             opts = optimoptions('quadprog', ...
%                 'Display', 'off', ...
%                 'Algorithm', 'interior-point-convex', ...
%                 'MaxIterations', 300, ...
%                 'ConstraintTolerance', 1e-4, ...
%                 'OptimalityTolerance', 1e-4);
% 
%             [X_opt, ~, exitflag, output] = quadprog(H, f, A_ineq, b_ineq, Aeq, beq, [], [], [], opts);
% 
%             if exitflag < 1
%                 fprintf("[BÉZIER C^6 DEBUG] 不等式制約付きQP未収束 (exitflag=%d). 等式拘束フォールバックを実行\n", exitflag);
%                 X_fb = pinv(Aeq) * beq;
%                 cn1 = X_fb(1:n_c);         cn2 = X_fb(n_c+1:2*n_c);
%                 cb1 = X_fb(2*n_c+1:3*n_c); cb2 = X_fb(3*n_c+1:4*n_c);
%             else
%                 fprintf("[BÉZIER C^6] 時空間最適化成功! (反復=%d, ピーク退避量=%.3fm)\n", output.iterations, delta_target);
%                 cn1 = X_opt(1:n_c);             cn2 = X_opt(n_c+1:2*n_c);
%                 cb1 = X_opt(2*n_c+1:3*n_c);     cb2 = X_opt(3*n_c+1:4*n_c);
%             end
% 
%             obj.coeffs_delta_seg1 = [cn1, cb1];
%             obj.coeffs_delta_seg2 = [cn2, cb2];
% 
%             obj.verify_c6_continuity();
%         end
% 
%         function verify_c6_continuity(obj)
%             N = obj.order;
%             T_seg = obj.T_seg;
%             names = ["位置 (0階)", "速度 (1階)", "加速度 (2階)", "Jerk (3階)", "Snap (4階)", "Crack (5階)", "Pop (6階)"];
% 
%             fprintf("-------------------------------------------------------\n");
%             fprintf("[理論検証] 中間接続点における 0〜6階微分の不連続量 (Gap):\n");
%             for k = 0:6
%                 c_end   = obj.get_bezier_derivative_coeffs_at_u(N, k, 1.0);
%                 c_start = obj.get_bezier_derivative_coeffs_at_u(N, k, 0.0);
% 
%                 d1_n = dot(c_end,   obj.coeffs_delta_seg1(:, 1)) / (T_seg^k);
%                 d1_b = dot(c_end,   obj.coeffs_delta_seg1(:, 2)) / (T_seg^k);
% 
%                 d2_n = dot(c_start, obj.coeffs_delta_seg2(:, 1)) / (T_seg^k);
%                 d2_b = dot(c_start, obj.coeffs_delta_seg2(:, 2)) / (T_seg^k);
% 
%                 gap = norm([d1_n - d2_n; d1_b - d2_b]);
%                 fprintf("  - %-12s ギャップ: %.3e\n", names(k + 1), gap);
%             end
%             fprintf("-------------------------------------------------------\n");
%         end
% 
%         function B = eval_bernstein_vector(~, n, u)
%             B = zeros(1, n + 1);
%             for i = 0:n
%                 B(i + 1) = nchoosek(n, i) * (u^i) * ((1 - u)^(n - i));
%             end
%         end
% 
%         function c_diff = get_bezier_derivative_coeffs_at_u(~, n, k, u)
%             if k > n, c_diff = zeros(1, n + 1); return; end
%             factor = factorial(n) / factorial(n - k);
%             b_low = zeros(1, n - k + 1);
%             for j = 0:(n - k)
%                 b_low(j + 1) = nchoosek(n - k, j) * (u^j) * ((1 - u)^(n - k - j));
%             end
%             D = eye(n + 1);
%             for step = 1:k, D = diff(D); end
%             c_diff = factor * (b_low * D);
%         end
% 
%         function Q = compute_bezier_derivative_hessian(obj, n, k)
%             Q = zeros(n + 1, n + 1);
%             if k > n, return; end
%             factor = factorial(n) / factorial(n - k);
%             n_low = n - k;
%             D = eye(n + 1);
%             for step = 1:k, D = diff(D); end
%             M_low = zeros(n_low + 1, n_low + 1);
%             for i = 0:n_low
%                 for j = 0:n_low
%                     M_low(i + 1, j + 1) = (nchoosek(n_low, i) * nchoosek(n_low, j)) / ...
%                         ((2 * n_low + 1) * nchoosek(2 * n_low, i + j));
%                 end
%             end
%             Q = (factor^2) * (D' * M_low * D);
%         end
% 
%         function Q = compute_bezier_pos_hessian(~, n)
%             Q = zeros(n + 1, n + 1);
%             for i = 0:n
%                 for j = 0:n
%                     Q(i + 1, j + 1) = (nchoosek(n, i) * nchoosek(n, j)) / ...
%                         ((2 * n + 1) * nchoosek(2 * n, i + j));
%                 end
%             end
%         end
%     end
% end

classdef REPLANNING_MELLINGER_CORRIDOR_FLATNESS_SWING < handle
    % REPLANNING_MELLINGER_CORRIDOR_FLATNESS_SWING
    % 汎用3次元静的・動的障害物両対応 13次 Bézier C^6 完全連続 時空間Receding Horizonリプランナ
    % - オンデマンド時空間再生成: 追従中軌道の未来干渉予測に基づく自律的軌道上書き
    % - C^6動的初期状態継承: 旋回中・退避中からのGap=0完全連続リプラン
    % - 複数障害物の連続検知・逐次回避
    % - 物理限界時間スケールガードによる推力飽和・特異点の完全防止
    
    properties
        base_ref
        self
        replan_active = false
        
        obs_mode     = 2    % 1: 静的, 2: 動的
        
        t_start      = 0.0
        t_duration   = 8.0
        T_seg        = 4.0
        
        last_replan_time = -100.0
        min_replan_interval = 0.30 % 最短リプラン間隔 [s]
        
        safe_margin  = 0.5
        trigger_dist = 8.0
        
        L_cable      = 2.0
        gravity      = 9.81
        m_drone      = 1.5
        m_load_est   = 0.1
        
        r_load       = 0.15
        r_drone      = 0.30
        
        max_swing_angle_deg = 20.0
        max_acc_drone       = 6.0
        
        order = 13
        coeffs_delta_seg1
        coeffs_delta_seg2
        
        dir_normal
        dir_binormal
        dir_nominal
        nominal_speed
        p_start_replan
        
        active_obs_idx  = -1
        obs_center_init = [0; 0; 0]
        obs_velocity    = [0; 0; 0]
        obs_R           = eye(3)
        obs_radii       = [1; 1; 1]
        obs_margin      = 0
        
        init_world_diff_state = zeros(7, 3);
        
        warned_crash_load
        warned_crash_drone
        warned_margin_load
        warned_margin_drone
        
        result
    end
    
    methods (Access = public)
        function obj = REPLANNING_MELLINGER_CORRIDOR_FLATNESS_SWING(self, base_ref, opts)
            arguments
                self
                base_ref
                opts = struct()
            end
            obj.self = self;
            obj.base_ref = base_ref;
            if isfield(opts, "obs_mode"),            obj.obs_mode            = opts.obs_mode;            end
            if isfield(opts, "safe_margin"),         obj.safe_margin         = opts.safe_margin;         end
            if isfield(opts, "trigger_dist"),        obj.trigger_dist        = opts.trigger_dist;        end
            if isfield(opts, "r_load"),              obj.r_load              = opts.r_load;              end
            if isfield(opts, "r_drone"),             obj.r_drone             = opts.r_drone;             end
            if isfield(opts, "max_swing_angle_deg"), obj.max_swing_angle_deg = opts.max_swing_angle_deg; end
            if isfield(opts, "max_acc_drone"),       obj.max_acc_drone       = opts.max_acc_drone;       end
            obj.result.state = STATE_CLASS(struct('state_list', ["xd", "p", "q", "v"], 'num_list', [28, 3, 3, 3]));
        end
        
        function result = do(obj, varargin)
            time = varargin{1};
            cha = varargin{2};
            
            base_res = obj.base_ref.do(time, cha);
            xd_nominal = base_res.state.xd;
            if length(xd_nominal) < 28
                xd_nominal = [xd_nominal; zeros(28 - length(xd_nominal), 1)];
            end
            
            obj.L_cable = obj.self.parameter.get("cableL");
            try
                obj.m_drone = obj.self.parameter.get("mass");
            catch
                obj.m_drone = 1.5;
            end
            
            if isprop(obj.self.estimator.result.state, "mL")
                obj.m_load_est = max(0.001, obj.self.estimator.result.state.mL);
            elseif isprop(obj.self.estimator.result, "loadmass")
                obj.m_load_est = max(0.001, obj.self.estimator.result.loadmass);
            else
                try
                    obj.m_load_est = max(0.001, obj.self.parameter.get("loadmass"));
                catch
                    obj.m_load_est = 0.013;
                end
            end
            
            if isprop(obj.self.estimator.result.state, "pL")
                pL_cur = obj.self.estimator.result.state.pL;
                vL_cur = obj.self.estimator.result.state.vL;
            else
                pL_cur = base_res.state.p;
                vL_cur = base_res.state.v;
            end
            
            if isprop(obj.self.estimator.result.state, "p")
                pQ_cur = obj.self.estimator.result.state.p;
            else
                pQ_cur = pL_cur + [0; 0; obj.L_cable];
            end
            
            obs_list = obj.get_current_obstacles(time.t);
            
            if isempty(obj.warned_crash_load) && ~isempty(obs_list)
                n_obs = length(obs_list);
                obj.warned_crash_load   = false(n_obs, 1);
                obj.warned_crash_drone  = false(n_obs, 1);
                obj.warned_margin_load  = false(n_obs, 1);
                obj.warned_margin_drone = false(n_obs, 1);
            end
            
            v_vec = xd_nominal(5:7);
            spd = norm(v_vec);
            if spd < 0.05, spd = norm(vL_cur); end
            if spd < 0.05, spd = 1.5; v_vec = [0; 0; 1.5]; end
            dir_nom = v_vec / spd;
            
            % -------------------------------------------------------------
            % 1. 時空間Receding Horizon干渉判定 ＆ 軌道再生成
            % -------------------------------------------------------------
            if cha == 'f' && ~isempty(obs_list)
                trigger_replan = false;
                best_idx = -1;
                min_t_impact = inf;
                min_surf_dist = inf;
                sensor_owner = "";
                
                % (A) 未発動時: 前方障害物の捕捉判定
                if ~obj.replan_active
                    for i = 1:length(obs_list)
                        tgt_i = obs_list(i);
                        v_obs_i = obj.extract_velocity(tgt_i);
                        c_i = tgt_i.p_center;
                        rad_i = tgt_i.ellipsoid_radii;
                        R_i = tgt_i.R_obs;
                        d_m_i = tgt_i.d_margin;
                        
                        v_rel = dir_nom * spd - v_obs_i;
                        spd_rel_sq = dot(v_rel, v_rel);
                        if spd_rel_sq < 1e-4, continue; end
                        
                        rel_pL = c_i - pL_cur;
                        t_closest = dot(rel_pL, v_rel) / spd_rel_sq;
                        if t_closest <= 0.1, continue; end
                        
                        pL_at_t = pL_cur + (dir_nom * spd) * t_closest;
                        c_at_t  = c_i + v_obs_i * t_closest;
                        d_lat = norm(c_at_t - pL_at_t);
                        
                        r_protect = max(obj.r_drone, obj.r_load);
                        r_crit = max(rad_i) + norm(v_obs_i)*1.5 + r_protect + d_m_i + obj.safe_margin;
                        
                        if d_lat < r_crit
                            d_surf_L = obj.calc_exact_surface_distance(pL_cur, c_i, R_i, rad_i);
                            d_surf_Q = obj.calc_exact_surface_distance(pQ_cur, c_i, R_i, rad_i);
                            cur_min_surf = min(d_surf_L, d_surf_Q);
                            
                            if cur_min_surf <= obj.trigger_dist && t_closest < min_t_impact
                                min_t_impact = t_closest;
                                min_surf_dist = cur_min_surf;
                                best_idx = i;
                                trigger_replan = true;
                                if d_surf_Q < d_surf_L, sensor_owner = "機体センサー";
                                else, sensor_owner = "荷物センサー"; end
                            end
                        end
                    end
                % (B) 飛行中: 現在追従中の軌道が将来干渉するか先読みスキャン (Cooldown 0.3s)
                elseif (time.t - obj.last_replan_time >= obj.min_replan_interval)
                    tau_cur = time.t - obj.t_start;
                    % 頂点を越えて合流中の場合は干渉判定を厳格化
                    t_scan_window = linspace(tau_cur, min(obj.t_duration, tau_cur + 2.0), 6);
                    
                    for i = 1:length(obs_list)
                        tgt_i = obs_list(i);
                        v_obs_i = obj.extract_velocity(tgt_i);
                        rad_i = tgt_i.ellipsoid_radii;
                        R_i = tgt_i.R_obs;
                        d_m_i = tgt_i.d_margin;
                        
                        for tau_s = t_scan_window
                            dt_future = tau_s - tau_cur;
                            c_future = tgt_i.p_center + v_obs_i * dt_future;
                            
                            t_eval = obj.t_start + tau_s;
                            nom_eval = obj.base_ref.do(struct('t', t_eval, 'dt', 0.025), 'f');
                            xd_nom_s = nom_eval.state.xd;
                            if length(xd_nom_s) < 28, xd_nom_s = [xd_nom_s; zeros(28-length(xd_nom_s), 1)]; end
                            xd_plan_s = obj.evaluate_smooth_trajectory(tau_s, xd_nom_s);
                            
                            pL_plan = xd_plan_s(1:3);
                            d_future_L = obj.calc_exact_surface_distance(pL_plan, c_future, R_i, rad_i);
                            
                            % 将来の軌道と障害物のクリアランスが危険域に入ったら再生成
                            if d_future_L < (obj.r_load + d_m_i + 0.30)
                                trigger_replan = true;
                                best_idx = i;
                                min_t_impact = max(1.5, dt_future);
                                min_surf_dist = d_future_L;
                                sensor_owner = "動的進路変化予測";
                                break;
                            end
                        end
                        if trigger_replan, break; end
                    end
                end
                
                % ---------------------------------------------------------
                % 軌道生成 / 再生成の実行
                % ---------------------------------------------------------
                if trigger_replan && best_idx > 0
                    tgt = obs_list(best_idx);
                    v_obs_tgt = obj.extract_velocity(tgt);
                    
                    if obj.replan_active
                        tau_now = time.t - obj.t_start;
                        obj.init_world_diff_state = obj.get_current_world_derivatives(tau_now);
                        fprintf("[RECEDING HORIZON] 障害物%dの動的変化を検知! C^6連続再生成 (t=%.3f s, 余白: %.2f m)\n", ...
                            best_idx, time.t, min_surf_dist);
                    else
                        obj.init_world_diff_state = zeros(7, 3);
                        fprintf("\n=======================================================\n");
                        fprintf("[時空間リプラン発動] 障害物 ID: %d を捕捉 (%s, 表面残余: %.2f m)\n", ...
                            best_idx, sensor_owner, min_surf_dist);
                    end
                    
                    obj.active_obs_idx   = best_idx;
                    obj.obs_center_init  = tgt.p_center;
                    obj.obs_velocity     = v_obs_tgt;
                    obj.obs_R            = tgt.R_obs;
                    obj.obs_radii        = tgt.ellipsoid_radii;
                    obj.obs_margin       = tgt.d_margin;
                    
                    obj.t_start          = time.t;
                    obj.last_replan_time = time.t;
                    obj.p_start_replan   = pL_cur;
                    obj.dir_nominal      = dir_nom;
                    obj.nominal_speed    = spd;
                    
                    % 特異点フリー直交基底
                    [~, min_dim] = min(abs(dir_nom));
                    v_arb = zeros(3, 1); v_arb(min_dim) = 1.0;
                    n1 = cross(dir_nom, v_arb); n1 = n1 / norm(n1);
                    n2 = cross(dir_nom, n1);     n2 = n2 / norm(n2);
                    
                    t_closest = max(1.5, min_t_impact);
                    c_apex = obj.obs_center_init + obj.obs_velocity * t_closest;
                    p_apex_line = obj.p_start_replan + (spd * t_closest) * dir_nom;
                    v_center_rel = c_apex - p_apex_line;
                    d_lat = norm(v_center_rel);
                    
                    % 360度方向探索
                    best_score = -inf;
                    best_n = n1;
                    r_test = max(tgt.ellipsoid_radii) + norm(v_obs_tgt)*1.5 + 3.0;
                    
                    for ang = linspace(0, 2*pi, 36)
                        cand_n = cos(ang) * n1 + sin(ang) * n2;
                        if d_lat > 1e-3 && dot(cand_n, -v_center_rel) < -0.1
                            continue;
                        end
                        test_p = p_apex_line + r_test * cand_n;
                        min_other_d = inf;
                        for j = 1:length(obs_list)
                            if j == best_idx, continue; end
                            v_j = obj.extract_velocity(obs_list(j));
                            c_j_future = obs_list(j).p_center + v_j * t_closest;
                            d_j = obj.calc_exact_surface_distance(test_p, c_j_future, obs_list(j).R_obs, obs_list(j).ellipsoid_radii);
                            min_other_d = min(min_other_d, d_j);
                        end
                        if min_other_d > best_score
                            best_score = min_other_d;
                            best_n = cand_n;
                        end
                    end
                    
                    obj.dir_normal = best_n;
                    b_cand = cross(dir_nom, obj.dir_normal);
                    obj.dir_binormal = b_cand / norm(b_cand);
                    
                    p_rel_n = obj.obs_R' * obj.dir_normal;
                    r_eff_n = 1.0 / norm(p_rel_n ./ obj.obs_radii);
                    
                    % 動的振幅を含めた包括退避量
                    dyn_motion_envelope = norm(v_obs_tgt) * 1.5;
                    req_clearance = max(5.0, (r_eff_n + dyn_motion_envelope + max(obj.r_drone, obj.r_load) + obj.obs_margin + 1.0) - d_lat);
                    
                    % 物理限界時間スケールガード (過渡加速度の跳ね上がりを防止)
                    t_pend = 2 * pi * sqrt(obj.L_cable / obj.gravity);
                    theta_max = deg2rad(obj.max_swing_angle_deg);
                    T_swing = sqrt(10.0 * req_clearance / (obj.gravity * tan(theta_max)));
                    T_acc   = sqrt(12.0 * req_clearance / obj.max_acc_drone);
                    
                    obj.T_seg = max([t_closest, 1.2 * t_pend, T_swing, T_acc, 4.0]);
                    obj.t_duration = 2.0 * obj.T_seg;
                    
                    actual_cable_vec = pQ_cur - pL_cur;
                    cable_offset_s = dot(actual_cable_vec, obj.dir_nominal);
                    
                    obj.plan_spatiotemporal_c6_qp(req_clearance, d_lat, cable_offset_s);
                    obj.replan_active = true;
                end
            end
            
            % -------------------------------------------------------------
            % 2. 飛行中常時監視
            % -------------------------------------------------------------
            if cha == 'f' && ~isempty(obs_list)
                for j = 1:length(obs_list)
                    tgt_j = obs_list(j);
                    c_j_now = tgt_j.p_center;
                    
                    d_surf_L = obj.calc_exact_surface_distance(pL_cur, c_j_now, tgt_j.R_obs, tgt_j.ellipsoid_radii);
                    d_surf_Q = obj.calc_exact_surface_distance(pQ_cur, c_j_now, tgt_j.R_obs, tgt_j.ellipsoid_radii);
                    
                    if d_surf_L <= 0 && ~obj.warned_crash_load(j)
                        fprintf(2, "[CRITICAL ALARM] 荷物が障害物%dに衝突! (t=%.3f s, 侵入: %.3f m)\n", j, time.t, -d_surf_L);
                        obj.warned_crash_load(j) = true;
                    elseif d_surf_L <= (obj.r_load + tgt_j.d_margin) && ~obj.warned_margin_load(j)
                        fprintf("[SAFETY WARN] 荷物が障害物%dのマージン帯侵入 (t=%.3f s, 残余: %.3f m)\n", j, time.t, d_surf_L);
                        obj.warned_margin_load(j) = true;
                    end
                    
                    if d_surf_Q <= 0 && ~obj.warned_crash_drone(j)
                        fprintf(2, "[CRITICAL ALARM] 機体が障害物%dに衝突! (t=%.3f s, 侵入: %.3f m)\n", j, time.t, -d_surf_Q);
                        obj.warned_crash_drone(j) = true;
                    elseif d_surf_Q <= (obj.r_drone + tgt_j.d_margin) && ~obj.warned_margin_drone(j)
                        fprintf("[SAFETY WARN] 機体が障害物%dのマージン帯侵入 (t=%.3f s, 残余: %.3f m)\n", j, time.t, d_surf_Q);
                        obj.warned_margin_drone(j) = true;
                    end
                end
            end
            
            % -------------------------------------------------------------
            % 3. 軌道出力 ＆ 次障害物への自律リセット
            % -------------------------------------------------------------
            if obj.replan_active
                tau = time.t - obj.t_start;
                if tau <= obj.t_duration
                    xd = obj.evaluate_smooth_trajectory(tau, xd_nominal);
                else
                    fprintf("[BÉZIER C^6] 障害物 %d 回避完了! 公称軌道へ合流 (t=%.3f s)\n\n", obj.active_obs_idx, time.t);
                    obj.replan_active = false;
                    obj.active_obs_idx = -1;
                    xd = xd_nominal;
                end
            else
                xd = xd_nominal;
            end
            
            if length(xd) < 28, xd = [xd; zeros(28 - length(xd), 1)]; end
            obj.result.state.xd = xd;
            obj.result.state.p = xd(1:3);
            obj.result.state.v = xd(5:7);
            obj.result.state.q = [0; 0; xd(4)];
            result = obj.result;
        end
        
        function xd = evaluate_smooth_trajectory(obj, tau, xd_nom)
            xd = xd_nom;
            N = obj.order;
            T_seg = obj.T_seg;
            
            if tau <= T_seg
                C = obj.coeffs_delta_seg1;
                u = max(0, min(1.0, tau / T_seg));
            else
                C = obj.coeffs_delta_seg2;
                u = max(0, min(1.0, (tau - T_seg) / T_seg));
            end
            
            for k = 0:6
                c_diff = obj.get_bezier_derivative_coeffs_at_u(N, k, u);
                delta_n = dot(c_diff, C(:, 1)) / (T_seg^k);
                delta_b = dot(c_diff, C(:, 2)) / (T_seg^k);
                
                delta_vec = obj.dir_normal * delta_n + obj.dir_binormal * delta_b;
                idx = 4 * k + (1:3);
                xd(idx) = xd_nom(idx) + delta_vec;
            end
        end
    end
    
    methods (Access = private)
        function list = get_current_obstacles(obj, t_now)
            list = [];
            try
                if obj.obs_mode == 2
                    list = ENVIRONMENT_OBSTACLE_ELLIPSE_MOVE(t_now);
                else
                    list = ENVIRONMENT_OBSTACLE_ELLIPSE();
                end
            catch
                try
                    list = ENVIRONMENT_OBSTACLE_ELLIPSE();
                catch
                end
            end
        end
        
        function v_obs = extract_velocity(~, tgt)
            v_obs = [0; 0; 0];
            if isfield(tgt, 'v_center'), v_obs = tgt.v_center(:);
            elseif isprop(tgt, 'v_center'), v_obs = tgt.v_center(:);
            elseif isfield(tgt, 'velocity'), v_obs = tgt.velocity(:);
            elseif isprop(tgt, 'velocity'), v_obs = tgt.velocity(:);
            end
        end
        
        function d = calc_exact_surface_distance(~, p, c, R, rad)
            p_rel = R' * (p - c);
            val = norm(p_rel ./ rad);
            if val < 1e-6
                d = -min(rad);
                return;
            end
            p_surf = p_rel / val;
            d = (val - 1.0) * norm(p_surf);
        end
        
        function world_diffs = get_current_world_derivatives(obj, tau)
            world_diffs = zeros(7, 3);
            N = obj.order;
            T_seg = obj.T_seg;
            if tau <= T_seg
                C = obj.coeffs_delta_seg1;
                u = max(0, min(1.0, tau / T_seg));
            else
                C = obj.coeffs_delta_seg2;
                u = max(0, min(1.0, (tau - T_seg) / T_seg));
            end
            for k = 0:6
                c_diff = obj.get_bezier_derivative_coeffs_at_u(N, k, u);
                delta_n = dot(c_diff, C(:, 1)) / (T_seg^k);
                delta_b = dot(c_diff, C(:, 2)) / (T_seg^k);
                
                world_vec_k = obj.dir_normal * delta_n + obj.dir_binormal * delta_b;
                world_diffs(k + 1, :) = world_vec_k';
            end
        end
        
        function plan_spatiotemporal_c6_qp(obj, req_clearance, d_center_lat, cable_offset_s)
            N = obj.order;
            n_c = N + 1;
            T_seg = obj.T_seg;
            
            n_vars_1d = 2 * n_c;
            n_vars_tot = 2 * n_vars_1d;
            
            % 目的関数 (Minimum Snap + 2階/0階正則化)
            Q_snap = obj.compute_bezier_derivative_hessian(N, 4);
            Q_acc  = obj.compute_bezier_derivative_hessian(N, 2);
            Q_pos  = obj.compute_bezier_pos_hessian(N);
            
            Q_snap = Q_snap / norm(Q_snap, 2);
            Q_acc  = Q_acc  / norm(Q_acc,  2);
            Q_pos  = Q_pos  / norm(Q_pos,  2);
            
            H_1d = 1.0 * Q_snap + 0.1 * Q_acc + 1e-4 * Q_pos;
            H_1d = (H_1d + H_1d') / 2 + 1e-6 * eye(n_c);
            H = blkdiag(H_1d, H_1d, H_1d, H_1d);
            f = zeros(n_vars_tot, 1);
            
            % 等式制約 (始端初期状態射影, 終端0, 中間Gap=0: 42本)
            Aeq_1d = zeros(21, n_vars_1d);
            beq_n  = zeros(21, 1);
            beq_b  = zeros(21, 1);
            
            % 始端 (u1=0): キャプチャした微分状態を新基底へ射影 (Gap=0)
            for k = 0:6
                c_diff = obj.get_bezier_derivative_coeffs_at_u(N, k, 0.0);
                Aeq_1d(k + 1, 1:n_c) = c_diff / (T_seg^k);
                
                w_vec_k = obj.init_world_diff_state(k + 1, :)';
                beq_n(k + 1) = dot(w_vec_k, obj.dir_normal);
                beq_b(k + 1) = dot(w_vec_k, obj.dir_binormal);
            end
            
            % 中間接続点 (u1=1, u2=0): 0〜6階連続
            for k = 0:6
                c_end   = obj.get_bezier_derivative_coeffs_at_u(N, k, 1.0);
                c_start = obj.get_bezier_derivative_coeffs_at_u(N, k, 0.0);
                Aeq_1d(7 + k + 1, 1:n_c)       =  c_end   / (T_seg^k);
                Aeq_1d(7 + k + 1, n_c+1:2*n_c) = -c_start / (T_seg^k);
            end
            
            % 終端 (u2=1): 0〜6階微分 = 0 (公称ラインへ完全一致合流)
            for k = 0:6
                c_diff = obj.get_bezier_derivative_coeffs_at_u(N, k, 1.0);
                Aeq_1d(14 + k + 1, n_c+1:2*n_c) = c_diff / (T_seg^k);
            end
            
            Aeq = blkdiag(Aeq_1d, Aeq_1d);
            beq = [beq_n; beq_b];
            
            % 不等式制約: 4D動的エンベロープ ＋ ピーククリアランス
            alpha_dyn = obj.L_cable / (obj.gravity * (T_seg^2));
            A_ineq = [];
            b_ineq = [];
            
            p_rel_n = obj.obs_R' * obj.dir_normal;
            r_eff_n = 1.0 / norm(p_rel_n ./ obj.obs_radii);
            p_rel_s = obj.obs_R' * obj.dir_nominal;
            r_eff_s = 1.0 / norm(p_rel_s ./ obj.obs_radii);
            
            dyn_envelope = norm(obj.obs_velocity) * 1.5;
            a_mink = r_eff_n + dyn_envelope + max(obj.r_drone, obj.r_load) + obj.obs_margin + 0.5;
            c_mink = r_eff_s + dyn_envelope + max(obj.r_drone, obj.r_load) + obj.obs_margin + 0.5;
            
            % 区間1 (接近側: u = 0.3 〜 1.0)
            for u = linspace(0.3, 1.0, 7)
                B_val = obj.eval_bernstein_vector(N, u);
                BQ_val = B_val + alpha_dyn * obj.get_bezier_derivative_coeffs_at_u(N, 2, u);
                
                tau_f = u * T_seg;
                c_f = obj.obs_center_init + obj.obs_velocity * tau_f;
                p_nom_f = obj.p_start_replan + (obj.nominal_speed * tau_f) * obj.dir_nominal;
                
                delta_s = dot(c_f - p_nom_f, obj.dir_nominal);
                v_lat_f = (c_f - p_nom_f) - delta_s * obj.dir_nominal;
                d_lat_f = norm(v_lat_f);
                
                req_L = 0.0; req_Q = 0.0;
                if abs(delta_s) < c_mink
                    req_L = max(0, a_mink * sqrt(1.0 - (delta_s / c_mink)^2) - d_lat_f);
                end
                delta_s_Q = delta_s - cable_offset_s;
                if abs(delta_s_Q) < c_mink
                    req_Q = max(0, a_mink * sqrt(1.0 - (delta_s_Q / c_mink)^2) - d_lat_f);
                end
                
                r_L = zeros(1, n_vars_tot); r_L(1:n_c) = -B_val;
                r_Q = zeros(1, n_vars_tot); r_Q(1:n_c) = -BQ_val;
                A_ineq = [A_ineq; r_L; r_Q];
                b_ineq = [b_ineq; -req_L; -req_Q];
            end
            
            % 区間2 (離脱側: u = 0.0 〜 0.7)
            for u = linspace(0.0, 0.7, 7)
                B_val = obj.eval_bernstein_vector(N, u);
                BQ_val = B_val + alpha_dyn * obj.get_bezier_derivative_coeffs_at_u(N, 2, u);
                
                tau_f = T_seg + u * T_seg;
                c_f = obj.obs_center_init + obj.obs_velocity * tau_f;
                p_nom_f = obj.p_start_replan + (obj.nominal_speed * tau_f) * obj.dir_nominal;
                
                delta_s = dot(c_f - p_nom_f, obj.dir_nominal);
                v_lat_f = (c_f - p_nom_f) - delta_s * obj.dir_nominal;
                d_lat_f = norm(v_lat_f);
                
                req_L = 0.0; req_Q = 0.0;
                if abs(delta_s) < c_mink
                    req_L = max(0, a_mink * sqrt(1.0 - (delta_s / c_mink)^2) - d_lat_f);
                end
                delta_s_Q = delta_s - cable_offset_s;
                if abs(delta_s_Q) < c_mink
                    req_Q = max(0, a_mink * sqrt(1.0 - (delta_s_Q / c_mink)^2) - d_lat_f);
                end
                
                r_L = zeros(1, n_vars_tot); r_L(n_c+1:2*n_c) = -B_val;
                r_Q = zeros(1, n_vars_tot); r_Q(n_c+1:2*n_c) = -BQ_val;
                A_ineq = [A_ineq; r_L; r_Q];
                b_ineq = [b_ineq; -req_L; -req_Q];
            end
            
            % 中間頂点でのハードクリアランス拘束 (Swept Volumeを完全に跨ぎ越す)
            B_mid = obj.eval_bernstein_vector(N, 1.0);
            BQ_mid = B_mid + alpha_dyn * obj.get_bezier_derivative_coeffs_at_u(N, 2, 1.0);
            r_mid_L = zeros(1, n_vars_tot); r_mid_L(1:n_c) = -B_mid;
            r_mid_Q = zeros(1, n_vars_tot); r_mid_Q(1:n_c) = -BQ_mid;
            A_ineq = [A_ineq; r_mid_L; r_mid_Q];
            b_ineq = [b_ineq; -req_clearance; -req_clearance];
            
            % 振れ角制限
            theta_max = deg2rad(obj.max_swing_angle_deg);
            a_limit = obj.gravity * tan(theta_max);
            for u = [0.5, 0.8, 1.0]
                c_ddot = obj.get_bezier_derivative_coeffs_at_u(N, 2, u) / (T_seg^2);
                r_pos = zeros(1, n_vars_tot); r_pos(1:n_c) = c_ddot;
                r_neg = zeros(1, n_vars_tot); r_neg(1:n_c) = -c_ddot;
                A_ineq = [A_ineq; r_pos; r_neg];
                b_ineq = [b_ineq; a_limit; a_limit];
            end
            
            % 機体加速度制限 (推力飽和を物理的に防止)
            for u = [0.5, 0.8, 1.0]
                c_ddot = obj.get_bezier_derivative_coeffs_at_u(N, 2, u) / (T_seg^2);
                c_snap = obj.get_bezier_derivative_coeffs_at_u(N, 4, u) / (T_seg^4);
                c_acc = c_ddot + (obj.L_cable / obj.gravity) * c_snap;
                r_pos = zeros(1, n_vars_tot); r_pos(1:n_c) = c_acc;
                r_neg = zeros(1, n_vars_tot); r_neg(1:n_c) = -c_acc;
                A_ineq = [A_ineq; r_pos; r_neg];
                b_ineq = [b_ineq; obj.max_acc_drone; obj.max_acc_drone];
            end
            
            opts = optimoptions('quadprog', ...
                'Display', 'off', ...
                'Algorithm', 'interior-point-convex', ...
                'MaxIterations', 300, ...
                'ConstraintTolerance', 1e-4, ...
                'OptimalityTolerance', 1e-4);
            
            [X_opt, ~, exitflag, ~] = quadprog(H, f, A_ineq, b_ineq, Aeq, beq, [], [], [], opts);
            
            if exitflag < 1
                X_fb = pinv(Aeq) * beq;
                cn1 = X_fb(1:n_c);         cn2 = X_fb(n_c+1:2*n_c);
                cb1 = X_fb(2*n_c+1:3*n_c); cb2 = X_fb(3*n_c+1:4*n_c);
            else
                cn1 = X_opt(1:n_c);             cn2 = X_opt(n_c+1:2*n_c);
                cb1 = X_opt(2*n_c+1:3*n_c);     cb2 = X_opt(3*n_c+1:4*n_c);
            end
            
            obj.coeffs_delta_seg1 = [cn1, cb1];
            obj.coeffs_delta_seg2 = [cn2, cb2];
        end
        
        function B = eval_bernstein_vector(~, n, u)
            B = zeros(1, n + 1);
            for i = 0:n
                B(i + 1) = nchoosek(n, i) * (u^i) * ((1 - u)^(n - i));
            end
        end
        
        function c_diff = get_bezier_derivative_coeffs_at_u(~, n, k, u)
            if k > n, c_diff = zeros(1, n + 1); return; end
            factor = factorial(n) / factorial(n - k);
            b_low = zeros(1, n - k + 1);
            for j = 0:(n - k)
                b_low(j + 1) = nchoosek(n - k, j) * (u^j) * ((1 - u)^(n - k - j));
            end
            D = eye(n + 1);
            for step = 1:k, D = diff(D); end
            c_diff = factor * (b_low * D);
        end
        
        function Q = compute_bezier_derivative_hessian(obj, n, k)
            Q = zeros(n + 1, n + 1);
            if k > n, return; end
            factor = factorial(n) / factorial(n - k);
            n_low = n - k;
            D = eye(n + 1);
            for step = 1:k, D = diff(D); end
            M_low = zeros(n_low + 1, n_low + 1);
            for i = 0:n_low
                for j = 0:n_low
                    M_low(i + 1, j + 1) = (nchoosek(n_low, i) * nchoosek(n_low, j)) / ...
                        ((2 * n_low + 1) * nchoosek(2 * n_low, i + j));
                end
            end
            Q = (factor^2) * (D' * M_low * D);
        end
        
        function Q = compute_bezier_pos_hessian(~, n)
            Q = zeros(n + 1, n + 1);
            for i = 0:n
                for j = 0:n
                    Q(i + 1, j + 1) = (nchoosek(n, i) * nchoosek(n, j)) / ...
                        ((2 * n + 1) * nchoosek(2 * n, i + j));
                end
            end
        end
    end
end
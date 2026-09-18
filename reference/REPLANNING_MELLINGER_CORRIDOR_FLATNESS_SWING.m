% classdef REPLANNING_MELLINGER_CORRIDOR_FLATNESS_SWING < handle
%     % REPLANNING_MELLINGER_CORRIDOR_FLATNESS_SWING
%     % 汎用3次元多障害物対応 13次 Bézier C^6 完全連続 最適単一山型リプランナ
%     % - アプローチB: ミンコフスキー和楕円境界の厳密断面逆算による滑らかな円弧回避
%     % - 振れ角制限(荷物加速度) & 機体加速度制限(平坦性スナップ含む)の完全分離QP制約
%     % - 物理限界(最大振れ角・最大推力・振り子周期)から T_seg を動的最適逆算
%     % - EKF推定質量・コントローラ剛性を反映した動的バッファ (Tube Bound)
%     % - 実機・実荷物の実状態センサー独立検知 (7.0m)
%     % - 外部描画クラスからのアクセス許可 (evaluate_smooth_trajectory を public 化)
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
%         m_drone      = 1.5   % ドローン機体質量 [kg]
%         m_load_est   = 0.1   % 推定荷物質量 [kg]
% 
%         r_load       = 0.15  % 荷物の外接球半径 [m]
%         r_drone      = 0.30  % 機体の外接球半径 [m]
% 
%         % --- 物理限界パラメータ ---
%         max_swing_angle_deg = 20.0  % 荷物の最大許容振れ角 [deg]
%         max_acc_drone       = 6.0   % ドローン機体の最大許容並進加速度 [m/s^2]
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
%         warned_margin_load  = false
%         warned_margin_drone = false
%         warned_crash_load   = false
%         warned_crash_drone  = false
% 
%         debug_cnt = 0
%         result
%     end
% 
%     % =========================================================================
%     % PUBLIC メソッド
%     % =========================================================================
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
%             % -------------------------------------------------------------
%             % 物理パラメータ・推定値の動的取得
%             % -------------------------------------------------------------
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
%             v_vec = xd_nominal(5:7);
%             spd = norm(v_vec);
%             if spd < 0.05, spd = norm(vL_cur); end
%             if spd < 0.05, spd = 1.0; v_vec = [0; 0; 1.0]; end
%             dir_nom = v_vec / spd;
% 
%             % -------------------------------------------------------------
%             % 1. 実状態センサーによる進路交差判定 ＆ トリガー
%             % -------------------------------------------------------------
%             if cha == 'f' && ~obj.replan_done && ~obj.replan_active && ~isempty(obs_list)
%                 best_idx = -1;
%                 min_proj_dist = inf;
%                 min_surf_dist = inf;
%                 sensor_owner = "";
% 
%                 for i = 1:length(obs_list)
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
%                 if best_idx > 0
%                     tgt = obs_list(best_idx);
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
%                     dist_line_to_obs = norm(obj.obs_center - p_closest_on_line);
% 
%                     if dist_line_to_obs < 1e-3
%                         if abs(dir_nom(3)) < 0.9
%                             n_cand = cross(dir_nom, [0; 0; 1]);
%                         else
%                             n_cand = cross(dir_nom, [1; 0; 0]);
%                         end
%                         obj.dir_normal = n_cand / norm(n_cand);
%                     else
%                         obj.dir_normal = -(obj.obs_center - p_closest_on_line) / dist_line_to_obs;
%                     end
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
%                     base_clearance = max(0.5, (r_eff_obs + obj.r_drone + obj.obs_margin) - dist_line_to_obs);
% 
%                     % 物理限界に基づく T_seg 最適逆算
%                     t_geom = d_proj_to_center / spd;
%                     t_pendulum = 2 * pi * sqrt(obj.L_cable / obj.gravity);
% 
%                     theta_max_rad = deg2rad(obj.max_swing_angle_deg);
%                     a_load_allow  = obj.gravity * tan(theta_max_rad);
%                     T_min_swing   = sqrt(10.0 * base_clearance / a_load_allow);
%                     T_min_drone   = sqrt(12.0 * base_clearance / obj.max_acc_drone);
% 
%                     obj.T_seg = max([t_geom, 1.2 * t_pendulum, T_min_swing, T_min_drone, 4.0]);
%                     obj.t_duration = 2.0 * obj.T_seg;
% 
%                     % 動的バッファ (Tube Bound)
%                     initial_pos_error = norm(pL_cur - (obj.p_start_replan + dot(pL_cur - obj.p_start_replan, dir_nom) * dir_nom));
%                     estimated_max_acc = 6.5 * base_clearance / (obj.T_seg^2);
%                     accel_tracking_error = estimated_max_acc / effective_Kp;
% 
%                     mass_ratio = obj.m_load_est / (obj.m_drone + obj.m_load_est);
%                     swing_coupling_buffer = mass_ratio * (obj.L_cable / obj.gravity) * estimated_max_acc;
% 
%                     dynamic_buffer = max(0.20, initial_pos_error + accel_tracking_error + swing_coupling_buffer);
% 
%                     fprintf("\n=======================================================\n");
%                     fprintf("[検知発動] 実状態センサーが前方障害物を捕捉! (ID: %d, Center=[%.2f, %.2f, %.2f])\n", ...
%                         best_idx, obj.obs_center(1), obj.obs_center(2), obj.obs_center(3));
%                     fprintf("  - 検知主体: %s (実表面距離: %.3f m, 進入速度: %.2f m/s)\n", ...
%                         sensor_owner, min_surf_dist, spd);
%                     fprintf("  - 制御パラメータ反映: 推定荷物質量 mL=%.4f kg, 実効剛性 Kp=%.2f\n", ...
%                         obj.m_load_est, effective_Kp);
%                     fprintf("  - 理論動的バッファ: %.3f m (加速遅れ:%.3fm, 荷物連成:%.3fm, 保証:0.20m)\n", ...
%                         dynamic_buffer, accel_tracking_error, swing_coupling_buffer);
%                     fprintf("  - 物理最適化時間: T_seg = %.2f s (全時間 T = %.2f s)\n", ...
%                         obj.T_seg, obj.t_duration);
% 
%                     obj.plan_bezier_c6_error_spline_qp(dynamic_buffer, dist_line_to_obs);
%                     obj.replan_active = true;
%                     fprintf("=======================================================\n\n");
%                 end
%             end
% 
%             % 2. 飛行中常時監視: マージン突入 / 実体衝突ログ
%             if cha == 'f' && obj.replan_active
%                 d_surf_L = obj.calc_exact_surface_distance(pL_cur, obj.obs_center, obj.obs_R, obj.obs_radii);
%                 d_surf_Q = obj.calc_exact_surface_distance(pQ_cur, obj.obs_center, obj.obs_R, obj.obs_radii);
% 
%                 if d_surf_L <= 0 && ~obj.warned_crash_load
%                     fprintf(2, "[CRITICAL ALARM] 荷物が障害物本体に衝突! (t=%.3f s, 侵入深さ=%.3f m)\n", time.t, -d_surf_L);
%                     obj.warned_crash_load = true;
%                 elseif d_surf_L <= (obj.r_load + obj.obs_margin) && ~obj.warned_margin_load
%                     fprintf("[SAFETY WARN] 荷物がソフトマージン帯に侵入 (t=%.3f s, 表面残余=%.3f m)\n", time.t, d_surf_L);
%                     obj.warned_margin_load = true;
%                 end
% 
%                 if d_surf_Q <= 0 && ~obj.warned_crash_drone
%                     fprintf(2, "[CRITICAL ALARM] 機体が障害物本体に衝突! (t=%.3f s, 侵入深さ=%.3f m)\n", time.t, -d_surf_Q);
%                     obj.warned_crash_drone = true;
%                 elseif d_surf_Q <= (obj.r_drone + obj.obs_margin) && ~obj.warned_margin_drone
%                     fprintf("[SAFETY WARN] 機体がソフトマージン帯に侵入 (t=%.3f s, 表面残余=%.3f m)\n", time.t, d_surf_Q);
%                     obj.warned_margin_drone = true;
%                 end
%             end
% 
%             % 3. 軌道出力 ＆ C^6 シームレス合流
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
%     % =========================================================================
%     % PRIVATE 内部計算法
%     % =========================================================================
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
%         function plan_bezier_c6_error_spline_qp(obj, dynamic_buffer, dist_line_to_obs)
%             N = obj.order;
%             n_c = N + 1;
%             T_seg = obj.T_seg;
% 
%             % -------------------------------------------------------------
%             % 【アプローチB】ミンコフスキー和による拡張楕円主軸半径
%             % -------------------------------------------------------------
%             p_rel_n = obj.obs_R' * obj.dir_normal;
%             r_eff_n = 1.0 / norm(p_rel_n ./ obj.obs_radii);
% 
%             p_rel_s = obj.obs_R' * obj.dir_nominal;
%             r_eff_s = 1.0 / norm(p_rel_s ./ obj.obs_radii);
% 
%             max_r_obj = max(obj.r_drone, obj.r_load);
%             a_mink = r_eff_n + max_r_obj + obj.obs_margin + dynamic_buffer;
%             c_mink = r_eff_s + max_r_obj + obj.obs_margin + dynamic_buffer;
% 
%             delta_target = max(a_mink - dist_line_to_obs, 0.8);
% 
%             n_vars_1d  = 2 * n_c;
%             n_vars_tot = 2 * n_vars_1d;
% 
%             % 目的関数
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
%             % 等式制約 (C^6 連続性: 42本)
%             Aeq_1d = zeros(21, n_vars_1d);
%             for k = 0:6
%                 c_diff = obj.get_bezier_derivative_coeffs_at_u(N, k, 0.0);
%                 Aeq_1d(k + 1, 1:n_c) = c_diff / (T_seg^k);
%             end
%             for k = 0:6
%                 c_end   = obj.get_bezier_derivative_coeffs_at_u(N, k, 1.0);
%                 c_start = obj.get_bezier_derivative_coeffs_at_u(N, k, 0.0);
%                 Aeq_1d(7 + k + 1, 1:n_c)       =  c_end   / (T_seg^k);
%                 Aeq_1d(7 + k + 1, n_c+1:2*n_c) = -c_start / (T_seg^k);
%             end
%             for k = 0:6
%                 c_diff = obj.get_bezier_derivative_coeffs_at_u(N, k, 1.0);
%                 Aeq_1d(14 + k + 1, n_c+1:2*n_c) = c_diff / (T_seg^k);
%             end
% 
%             Aeq = blkdiag(Aeq_1d, Aeq_1d);
%             beq = zeros(42, 1);
% 
%             % -------------------------------------------------------------
%             % 不等式制約: 楕円断面逆算 + 振れ角制限 + 機体加速度制限
%             % -------------------------------------------------------------
%             alpha_dyn = obj.L_cable / (obj.gravity * (T_seg^2));
%             cable_offset_s = dot([0; 0; obj.L_cable], obj.dir_nominal);
% 
%             A_ineq = [];
%             b_ineq = [];
% 
%             % 1) 楕円方程式から逆算した丸い壁（アプローチB）
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
% 
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
% 
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
%             % 2) 振れ角制限 (|a_load| <= g * tan(theta_max))
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
%             % 3) 機体加速度制限 (|a_drone| <= max_acc_drone)
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
%             % QP 最適化実行
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
%                 fprintf("[BÉZIER C^6] 最適化成功! (反復=%d, 厳密アプローチB退避量=%.3fm)\n", output.iterations, delta_target);
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
%         % --- Bézier 解析導出ヘルパー関数 ---
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
%     % 汎用3次元多障害物対応 13次 Bézier C^6 完全連続 最適単一山型リプランナ
%     % - アプローチB: ミンコフスキー和楕円境界の厳密断面逆算による滑らかな円弧回避
%     % - 機体先行オフセットの完全動的汎用化 (実相対ベクトル pQ - pL の射影)
%     % - 振れ角制限(荷物加速度) & 機体加速度制限(平坦性スナップ含む)の完全分離QP制約
%     % - 物理限界(最大振れ角・最大推力・振り子周期)から T_seg を動的最適逆算
%     % - EKF推定質量・コントローラ剛性を反映した動的バッファ (Tube Bound)
%     % - 実機・実荷物の実状態センサー独立検知 (7.0m)
%     % - 外部描画クラスからのアクセス許可 (evaluate_smooth_trajectory を public 化)
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
%         m_drone      = 1.5   % ドローン機体質量 [kg]
%         m_load_est   = 0.1   % 推定荷物質量 [kg]
% 
%         r_load       = 0.15  % 荷物の外接球半径 [m]
%         r_drone      = 0.30  % 機体の外接球半径 [m]
% 
%         % --- 物理限界パラメータ ---
%         max_swing_angle_deg = 20.0  % 荷物の最大許容振れ角 [deg]
%         max_acc_drone       = 6.0   % ドローン機体の最大許容並進加速度 [m/s^2]
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
%         warned_margin_load  = false
%         warned_margin_drone = false
%         warned_crash_load   = false
%         warned_crash_drone  = false
% 
%         debug_cnt = 0
%         result
%     end
% 
%     % =========================================================================
%     % PUBLIC メソッド
%     % =========================================================================
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
%             % -------------------------------------------------------------
%             % 物理パラメータ・推定値の動的取得
%             % -------------------------------------------------------------
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
%             v_vec = xd_nominal(5:7);
%             spd = norm(v_vec);
%             if spd < 0.05, spd = norm(vL_cur); end
%             if spd < 0.05, spd = 1.0; v_vec = [0; 0; 1.0]; end
%             dir_nom = v_vec / spd;
% 
%             % -------------------------------------------------------------
%             % 1. 実状態センサーによる進路交差判定 ＆ トリガー
%             % -------------------------------------------------------------
%             if cha == 'f' && ~obj.replan_done && ~obj.replan_active && ~isempty(obs_list)
%                 best_idx = -1;
%                 min_proj_dist = inf;
%                 min_surf_dist = inf;
%                 sensor_owner = "";
% 
%                 for i = 1:length(obs_list)
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
%                 if best_idx > 0
%                     tgt = obs_list(best_idx);
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
%                     dist_line_to_obs = norm(obj.obs_center - p_closest_on_line);
% 
%                     if dist_line_to_obs < 1e-3
%                         if abs(dir_nom(3)) < 0.9
%                             n_cand = cross(dir_nom, [0; 0; 1]);
%                         else
%                             n_cand = cross(dir_nom, [1; 0; 0]);
%                         end
%                         obj.dir_normal = n_cand / norm(n_cand);
%                     else
%                         obj.dir_normal = -(obj.obs_center - p_closest_on_line) / dist_line_to_obs;
%                     end
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
%                     base_clearance = max(0.5, (r_eff_obs + obj.r_drone + obj.obs_margin) - dist_line_to_obs);
% 
%                     % 物理限界に基づく T_seg 最適逆算
%                     t_geom = d_proj_to_center / spd;
%                     t_pendulum = 2 * pi * sqrt(obj.L_cable / obj.gravity);
% 
%                     theta_max_rad = deg2rad(obj.max_swing_angle_deg);
%                     a_load_allow  = obj.gravity * tan(theta_max_rad);
%                     T_min_swing   = sqrt(10.0 * base_clearance / a_load_allow);
%                     T_min_drone   = sqrt(12.0 * base_clearance / obj.max_acc_drone);
% 
%                     obj.T_seg = max([t_geom, 1.2 * t_pendulum, T_min_swing, T_min_drone, 4.0]);
%                     obj.t_duration = 2.0 * obj.T_seg;
% 
%                     % 動的バッファ (Tube Bound)
%                     initial_pos_error = norm(pL_cur - (obj.p_start_replan + dot(pL_cur - obj.p_start_replan, dir_nom) * dir_nom));
%                     estimated_max_acc = 6.5 * base_clearance / (obj.T_seg^2);
%                     accel_tracking_error = estimated_max_acc / effective_Kp;
% 
%                     mass_ratio = obj.m_load_est / (obj.m_drone + obj.m_load_est);
%                     swing_coupling_buffer = mass_ratio * (obj.L_cable / obj.gravity) * estimated_max_acc;
% 
%                     dynamic_buffer = max(0.20, initial_pos_error + accel_tracking_error + swing_coupling_buffer);
% 
%                     % 進行方向への動的先行オフセット算出 (実状態相対ベクトルから直接射影)
%                     actual_cable_vec = pQ_cur - pL_cur;
%                     cable_offset_s = dot(actual_cable_vec, obj.dir_nominal);
% 
%                     fprintf("\n=======================================================\n");
%                     fprintf("[検知発動] 実状態センサーが前方障害物を捕捉! (ID: %d, Center=[%.2f, %.2f, %.2f])\n", ...
%                         best_idx, obj.obs_center(1), obj.obs_center(2), obj.obs_center(3));
%                     fprintf("  - 検知主体: %s (実表面距離: %.3f m, 進入速度: %.2f m/s)\n", ...
%                         sensor_owner, min_surf_dist, spd);
%                     fprintf("  - 制御パラメータ反映: 推定荷物質量 mL=%.4f kg, 実効剛性 Kp=%.2f\n", ...
%                         obj.m_load_est, effective_Kp);
%                     fprintf("  - 幾何学的動的オフセット: 進行方向先行距離 = %.3f m\n", cable_offset_s);
%                     fprintf("  - 理論動的バッファ: %.3f m (加速遅れ:%.3fm, 荷物連成:%.3fm, 保証:0.20m)\n", ...
%                         dynamic_buffer, accel_tracking_error, swing_coupling_buffer);
%                     fprintf("  - 物理最適化時間: T_seg = %.2f s (全時間 T = %.2f s)\n", ...
%                         obj.T_seg, obj.t_duration);
% 
%                     obj.plan_bezier_c6_error_spline_qp(dynamic_buffer, dist_line_to_obs, cable_offset_s);
%                     obj.replan_active = true;
%                     fprintf("=======================================================\n\n");
%                 end
%             end
% 
%             % 2. 飛行中常時監視: マージン突入 / 実体衝突ログ
%             if cha == 'f' && obj.replan_active
%                 d_surf_L = obj.calc_exact_surface_distance(pL_cur, obj.obs_center, obj.obs_R, obj.obs_radii);
%                 d_surf_Q = obj.calc_exact_surface_distance(pQ_cur, obj.obs_center, obj.obs_R, obj.obs_radii);
% 
%                 if d_surf_L <= 0 && ~obj.warned_crash_load
%                     fprintf(2, "[CRITICAL ALARM] 荷物が障害物本体に衝突! (t=%.3f s, 侵入深さ=%.3f m)\n", time.t, -d_surf_L);
%                     obj.warned_crash_load = true;
%                 elseif d_surf_L <= (obj.r_load + obj.obs_margin) && ~obj.warned_margin_load
%                     fprintf("[SAFETY WARN] 荷物がソフトマージン帯に侵入 (t=%.3f s, 表面残余=%.3f m)\n", time.t, d_surf_L);
%                     obj.warned_margin_load = true;
%                 end
% 
%                 if d_surf_Q <= 0 && ~obj.warned_crash_drone
%                     fprintf(2, "[CRITICAL ALARM] 機体が障害物本体に衝突! (t=%.3f s, 侵入深さ=%.3f m)\n", time.t, -d_surf_Q);
%                     obj.warned_crash_drone = true;
%                 elseif d_surf_Q <= (obj.r_drone + obj.obs_margin) && ~obj.warned_margin_drone
%                     fprintf("[SAFETY WARN] 機体がソフトマージン帯に侵入 (t=%.3f s, 表面残余=%.3f m)\n", time.t, d_surf_Q);
%                     obj.warned_margin_drone = true;
%                 end
%             end
% 
%             % 3. 軌道出力 ＆ C^6 シームレス合流
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
%     % =========================================================================
%     % PRIVATE 内部計算法
%     % =========================================================================
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
%         function plan_bezier_c6_error_spline_qp(obj, dynamic_buffer, dist_line_to_obs, cable_offset_s)
%             N = obj.order;
%             n_c = N + 1;
%             T_seg = obj.T_seg;
% 
%             % -------------------------------------------------------------
%             % 【アプローチB】ミンコフスキー和による拡張楕円主軸半径
%             % -------------------------------------------------------------
%             p_rel_n = obj.obs_R' * obj.dir_normal;
%             r_eff_n = 1.0 / norm(p_rel_n ./ obj.obs_radii);
% 
%             p_rel_s = obj.obs_R' * obj.dir_nominal;
%             r_eff_s = 1.0 / norm(p_rel_s ./ obj.obs_radii);
% 
%             max_r_obj = max(obj.r_drone, obj.r_load);
%             a_mink = r_eff_n + max_r_obj + obj.obs_margin + dynamic_buffer;
%             c_mink = r_eff_s + max_r_obj + obj.obs_margin + dynamic_buffer;
% 
%             delta_target = max(a_mink - dist_line_to_obs, 0.8);
% 
%             n_vars_1d  = 2 * n_c;
%             n_vars_tot = 2 * n_vars_1d;
% 
%             % 目的関数
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
%             % 等式制約 (C^6 連続性: 42本)
%             Aeq_1d = zeros(21, n_vars_1d);
%             for k = 0:6
%                 c_diff = obj.get_bezier_derivative_coeffs_at_u(N, k, 0.0);
%                 Aeq_1d(k + 1, 1:n_c) = c_diff / (T_seg^k);
%             end
%             for k = 0:6
%                 c_end   = obj.get_bezier_derivative_coeffs_at_u(N, k, 1.0);
%                 c_start = obj.get_bezier_derivative_coeffs_at_u(N, k, 0.0);
%                 Aeq_1d(7 + k + 1, 1:n_c)       =  c_end   / (T_seg^k);
%                 Aeq_1d(7 + k + 1, n_c+1:2*n_c) = -c_start / (T_seg^k);
%             end
%             for k = 0:6
%                 c_diff = obj.get_bezier_derivative_coeffs_at_u(N, k, 1.0);
%                 Aeq_1d(14 + k + 1, n_c+1:2*n_c) = c_diff / (T_seg^k);
%             end
% 
%             Aeq = blkdiag(Aeq_1d, Aeq_1d);
%             beq = zeros(42, 1);
% 
%             % -------------------------------------------------------------
%             % 不等式制約: 楕円断面逆算 + 振れ角制限 + 機体加速度制限
%             % -------------------------------------------------------------
%             alpha_dyn = obj.L_cable / (obj.gravity * (T_seg^2));
% 
%             A_ineq = [];
%             b_ineq = [];
% 
%             % 1) 楕円方程式から逆算した丸い壁（アプローチB）
%             N_samp = 7;
%             u1_samples = linspace(0.3, 1.0, N_samp);
%             for u = u1_samples
%                 B_val      = obj.eval_bernstein_vector(N, u);
%                 B_ddot_val = obj.get_bezier_derivative_coeffs_at_u(N, 2, u);
%                 BQ_val     = B_val + alpha_dyn * B_ddot_val;
% 
%                 delta_s_L = (1.0 - u) * obj.nominal_speed * T_seg;
%                 delta_s_Q = delta_s_L - cable_offset_s; % 動的先行オフセットを反映
% 
%                 req_L = 0.0;
%                 if abs(delta_s_L) < c_mink
%                     req_L = max(0, a_mink * sqrt(1.0 - (delta_s_L / c_mink)^2) - dist_line_to_obs);
%                 end
% 
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
% 
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
%             % 2) 振れ角制限 (|a_load| <= g * tan(theta_max))
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
%             % 3) 機体加速度制限 (|a_drone| <= max_acc_drone)
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
%             % QP 最適化実行
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
%                 fprintf("[BÉZIER C^6] 最適化成功! (反復=%d, 厳密アプローチB退避量=%.3fm)\n", output.iterations, delta_target);
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
%         % --- Bézier 解析導出ヘルパー関数 ---
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
%     % 汎用3次元多障害物対応 13次 Bézier C^6 完全連続 最適単一山型リプランナ
%     % - 全障害物常時監視: どの障害物に触れても即座にアラーム出力
%     % - 競合回避法線選定: 他の障害物との干渉が最も少ない安全方向へ退避
%     % - アプローチB: ミンコフスキー和楕円境界の厳密断面逆算
%     % - 機体先行オフセット動的抽出 (pQ - pL の射影)
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
%         coeffs_delta_seg1
%         coeffs_delta_seg2
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
%         % 全障害物監視用のアラームフラグ配列
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
%                 obj.m_load_est = 0.013;
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
%             % 1. 実状態センサーによる進路交差判定 ＆ トリガー
%             % -------------------------------------------------------------
%             if cha == 'f' && ~obj.replan_done && ~obj.replan_active && ~isempty(obs_list)
%                 best_idx = -1;
%                 min_proj_dist = inf;
%                 min_surf_dist = inf;
%                 sensor_owner = "";
% 
%                 for i = 1:length(obs_list)
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
%                 if best_idx > 0
%                     tgt = obs_list(best_idx);
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
%                     % -----------------------------------------------------
%                     % 安全な退避法線 dir_normal の探索 (全障害物との干渉評価)
%                     % -----------------------------------------------------
%                     vec_to_obs = obj.obs_center - obj.p_start_replan;
%                     d_proj_to_center = dot(vec_to_obs, dir_nom);
%                     p_closest_on_line = obj.p_start_replan + d_proj_to_center * dir_nom;
% 
%                     % 直交平面上の基準直交ベクトルを構築
%                     if abs(dir_nom(3)) < 0.9
%                         base_n1 = cross(dir_nom, [0; 0; 1]);
%                     else
%                         base_n1 = cross(dir_nom, [1; 0; 0]);
%                     end
%                     base_n1 = base_n1 / norm(base_n1);
%                     base_n2 = cross(dir_nom, base_n1);
%                     base_n2 = base_n2 / norm(base_n2);
% 
%                     % 360度方向をサンプリングし、最も他の障害物から遠い安全方向を選定
%                     best_score = -inf;
%                     best_normal = base_n1;
% 
%                     test_angles = linspace(0, 2*pi, 36);
%                     for ang = test_angles
%                         cand_n = cos(ang) * base_n1 + sin(ang) * base_n2;
% 
%                         % 対象障害物から離れる方向であることを前提条件とする
%                         if dot(cand_n, -(obj.obs_center - p_closest_on_line)) < -0.1
%                             continue;
%                         end
% 
%                         % 他の障害物との最小クリアランスをスコア化
%                         test_apex = p_closest_on_line + 4.5 * cand_n;
%                         min_dist_other = inf;
%                         for j = 1:length(obs_list)
%                             if j == best_idx, continue; end
%                             d_j = obj.calc_exact_surface_distance(test_apex, obs_list(j).p_center, obs_list(j).R_obs, obs_list(j).ellipsoid_radii);
%                             min_dist_other = min(min_dist_other, d_j);
%                         end
% 
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
%                     fprintf("\n=======================================================\n");
%                     fprintf("[検知発動] 実状態センサーが前方障害物を捕捉! (ID: %d, Center=[%.2f, %.2f, %.2f])\n", ...
%                         best_idx, obj.obs_center(1), obj.obs_center(2), obj.obs_center(3));
%                     fprintf("  - 検知主体: %s (実表面距離: %.3f m, 進入速度: %.2f m/s)\n", sensor_owner, min_surf_dist, spd);
%                     fprintf("  - 選択された安全退避方向: [%.2f, %.2f, %.2f] (他障害物余白: %.2f m)\n", ...
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
%             % 2. 飛行中常時監視: 【全障害物】に対する完全網羅判定
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
%             delta_target = max(a_mink - dist_line_to_obs, 0.8);
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
%             % 0〜6階微分の完全一致等式制約 (42本)
%             Aeq_1d = zeros(21, n_vars_1d);
%             for k = 0:6
%                 c_diff = obj.get_bezier_derivative_coeffs_at_u(N, k, 0.0);
%                 Aeq_1d(k + 1, 1:n_c) = c_diff / (T_seg^k);
%             end
%             for k = 0:6
%                 c_end   = obj.get_bezier_derivative_coeffs_at_u(N, k, 1.0);
%                 c_start = obj.get_bezier_derivative_coeffs_at_u(N, k, 0.0);
%                 Aeq_1d(7 + k + 1, 1:n_c)       =  c_end   / (T_seg^k);
%                 Aeq_1d(7 + k + 1, n_c+1:2*n_c) = -c_start / (T_seg^k);
%             end
%             for k = 0:6
%                 c_diff = obj.get_bezier_derivative_coeffs_at_u(N, k, 1.0);
%                 Aeq_1d(14 + k + 1, n_c+1:2*n_c) = c_diff / (T_seg^k);
%             end
% 
%             Aeq = blkdiag(Aeq_1d, Aeq_1d);
%             beq = zeros(42, 1);
% 
%             % 不等式制約 (アプローチB 楕円逆算 ＋ 物理限界)
%             alpha_dyn = obj.L_cable / (obj.gravity * (T_seg^2));
%             A_ineq = [];
%             b_ineq = [];
% 
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
% 
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
% 
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
%                 fprintf("[BÉZIER C^6] 最適化成功! (反復=%d, 厳密アプローチB退避量=%.3fm)\n", output.iterations, delta_target);
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
%     % 汎用3次元多障害物対応 13次 Bézier C^6 完全連続 完全汎用Receding Horizonリプランナ
%     % - ワールド座標系3次元微分キャッシュによる完全シームレスな基底乗り換え (C^6保証)
%     % - 障害物スケール連動型 360度安全法線探索 (固定値4.5mの完全排除)
%     % - Gram-Schmidtによる特異点フリー直交基底生成
%     % - アプローチB: ミンコフスキー和楕円境界の厳密断面逆算
%     % - 機体先行オフセット動的抽出 (pQ - pL の射影)
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
%         coeffs_delta_seg1
%         coeffs_delta_seg2
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
%         % 再計画時のワールド座標系初期微分バッファ (7 x 3: 0〜6階微分の [dx, dy, dz])
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
%                 % ---------------------------------------------------------
%                 % 新たな前方障害物を検知：動的初期状態継承による即時リプラン
%                 % ---------------------------------------------------------
%                 if best_idx > 0
%                     tgt = obs_list(best_idx);
% 
%                     % ワールド座標系での現在微分ベクトルのキャプチャ
%                     if obj.replan_active
%                         tau_now = time.t - obj.t_start;
%                         obj.init_world_diff_state = obj.get_current_world_derivatives(tau_now);
%                         fprintf("\n[RECEDING HORIZON] 回避中に新たな前方障害物 (ID: %d) を捕捉! 軌道を動的更新します。\n", best_idx);
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
%                     % -----------------------------------------------------
%                     % 頑健な直交基底の生成 (特異点フリー Gram-Schmidt)
%                     % -----------------------------------------------------
%                     vec_to_obs = obj.obs_center - obj.p_start_replan;
%                     d_proj_to_center = dot(vec_to_obs, dir_nom);
%                     p_closest_on_line = obj.p_start_replan + d_proj_to_center * dir_nom;
% 
%                     [~, min_dim] = min(abs(dir_nom));
%                     v_arbitrary = zeros(3, 1);
%                     v_arbitrary(min_dim) = 1.0;
% 
%                     base_n1 = cross(dir_nom, v_arbitrary);
%                     base_n1 = base_n1 / norm(base_n1);
%                     base_n2 = cross(dir_nom, base_n1);
%                     base_n2 = base_n2 / norm(base_n2);
% 
%                     % 対象障害物の概略有効半径から動的テスト半径を算出
%                     v_center_rel = obj.obs_center - p_closest_on_line;
%                     d_center_lat = norm(v_center_rel);
%                     p_rel_raw = obj.obs_R' * (v_center_rel / max(d_center_lat, 1e-4));
%                     r_eff_est = 1.0 / norm(p_rel_raw ./ obj.obs_radii);
%                     test_radius = max(r_eff_est + 1.0, 1.5);
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
%         % 現在の退避多項式からワールド座標系における 3次元微分ベクトル (0〜6階) を厳密復元
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
%                 % 直交基底からワールド座標系ベクトルへ変換
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
%             delta_target = max(a_mink - dist_line_to_obs, 0.8);
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
%             % -------------------------------------------------------------
%             % 等式制約: 【新基底への射影による動的初期状態代入】
%             % -------------------------------------------------------------
%             Aeq_1d = zeros(21, n_vars_1d);
%             beq_n  = zeros(21, 1);
%             beq_b  = zeros(21, 1);
% 
%             % 1) 始端 (u1=0): キャプチャしたワールド微分ベクトルを新基底 (dir_normal, dir_binormal) へ射影
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
%             % 3) 終端 (u2=1): 0〜6階微分 = 0 (公称ラインへのシームレス復帰)
%             for k = 0:6
%                 c_diff = obj.get_bezier_derivative_coeffs_at_u(N, k, 1.0);
%                 Aeq_1d(14 + k + 1, n_c+1:2*n_c) = c_diff / (T_seg^k);
%             end
% 
%             Aeq = blkdiag(Aeq_1d, Aeq_1d);
%             beq = [beq_n; beq_b];
% 
%             % 不等式制約 (アプローチB 楕円逆算 ＋ 物理限界)
%             alpha_dyn = obj.L_cable / (obj.gravity * (T_seg^2));
%             A_ineq = [];
%             b_ineq = [];
% 
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
% 
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
% 
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
%                 fprintf("[BÉZIER C^6] 最適化成功! (反復=%d, 厳密アプローチB退避量=%.3fm)\n", output.iterations, delta_target);
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
    % 汎用3次元多障害物対応 13次 Bézier C^6 完全連続 最適単一山型リプランナ
    % - アプローチB: ミンコフスキー和楕円境界の厳密断面逆算 ＋ 頂点クリアランス保証
    % - 全障害物常時監視 & 360度安全法線探索 (障害物3との二重衝突を根絶)
    % - ワールド座標系3次元微分キャッシュによる完全シームレスな基底乗り換え
    % - 振れ角制限(荷物加速度) & 機体加速度制限(平坦性スナップ含む)の完全分離QP制約
    % - 物理限界(最大振れ角・最大推力・振り子周期)から T_seg を動的最適逆算
    % - EKF推定質量・コントローラ剛性を反映した動的バッファ (Tube Bound)
    
    properties
        base_ref
        self
        replan_active = false
        replan_done   = false
        
        t_start
        t_duration = 8.0
        T_seg
        
        safe_margin  = 0.3
        trigger_dist = 7.0
        
        L_cable      = 2.0
        gravity      = 9.81
        m_drone      = 1.5
        m_load_est   = 0.1
        
        r_load       = 0.15
        r_drone      = 0.30
        
        max_swing_angle_deg = 20.0
        max_acc_drone       = 6.0
        
        order = 13
        coeffs_delta_seg1 % (14 x 2: 法線, 従法線)
        coeffs_delta_seg2 % (14 x 2: 法線, 従法線)
        
        dir_normal        % 最適退避法線 (3 x 1)
        dir_binormal      % 最適退避従法線 (3 x 1)
        dir_nominal       % 進行方向単位ベクトル (3 x 1)
        nominal_speed     % 進入巡航速度
        p_start_replan    % リプラン開始位置
        
        t_merge_end
        p_merge_end
        v_merge_vec
        
        active_obs_idx = -1
        obs_center     = [0; 0; 0]
        obs_R          = eye(3)
        obs_radii      = [1; 1; 1]
        obs_margin     = 0
        
        % 再計画時のワールド座標系初期微分バッファ (7 x 3)
        init_world_diff_state = zeros(7, 3);
        
        warned_crash_load
        warned_crash_drone
        warned_margin_load
        warned_margin_drone
        
        debug_cnt = 0
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
            
            obs_list = [];
            try
                obs_list = ENVIRONMENT_OBSTACLE_ELLIPSE();
            catch
            end
            
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
            if spd < 0.05, spd = 1.0; v_vec = [0; 0; 1.0]; end
            dir_nom = v_vec / spd;
            
            % -------------------------------------------------------------
            % 1. Receding Horizon 進路予測 ＆ 連続リプランニング判定
            % -------------------------------------------------------------
            if cha == 'f' && ~obj.replan_done && ~isempty(obs_list)
                best_idx = -1;
                min_proj_dist = inf;
                min_surf_dist = inf;
                sensor_owner = "";
                
                for i = 1:length(obs_list)
                    if obj.replan_active && (i == obj.active_obs_idx)
                        continue;
                    end
                    
                    tgt_i = obs_list(i);
                    c_i = tgt_i.p_center;
                    rad_i = tgt_i.ellipsoid_radii;
                    R_i = tgt_i.R_obs;
                    d_m_i = tgt_i.d_margin;
                    
                    d_proj_L = dot(c_i - pL_cur, dir_nom);
                    d_proj_Q = dot(c_i - pQ_cur, dir_nom);
                    
                    if d_proj_L <= 0.1 && d_proj_Q <= 0.1, continue; end
                    
                    p_line_closest = pL_cur + d_proj_L * dir_nom;
                    dist_lateral = norm(c_i - p_line_closest);
                    
                    vec_lat = c_i - p_line_closest;
                    if dist_lateral < 1e-4
                        r_eff = max(rad_i);
                    else
                        dir_lat = vec_lat / dist_lateral;
                        p_rel_lat = R_i' * dir_lat;
                        r_eff = 1.0 / norm(p_rel_lat ./ rad_i);
                    end
                    
                    max_corridor_r = r_eff + max(obj.r_drone, obj.r_load) + d_m_i + 0.1;
                    if dist_lateral > max_corridor_r, continue; end
                    
                    d_surf_L = obj.calc_exact_surface_distance(pL_cur, c_i, R_i, rad_i);
                    d_surf_Q = obj.calc_exact_surface_distance(pQ_cur, c_i, R_i, rad_i);
                    
                    if d_surf_Q < d_surf_L
                        cur_min_surf = d_surf_Q;
                        cur_proj = d_proj_Q;
                        cur_owner = "機体実位置センサー";
                    else
                        cur_min_surf = d_surf_L;
                        cur_proj = d_proj_L;
                        cur_owner = "荷物実位置センサー";
                    end
                    
                    if cur_min_surf <= obj.trigger_dist && cur_proj < min_proj_dist
                        min_proj_dist = cur_proj;
                        min_surf_dist = cur_min_surf;
                        best_idx = i;
                        sensor_owner = cur_owner;
                    end
                end
                
                % 新たな前方障害物を検知
                if best_idx > 0
                    tgt = obs_list(best_idx);
                    
                    if obj.replan_active
                        tau_now = time.t - obj.t_start;
                        obj.init_world_diff_state = obj.get_current_world_derivatives(tau_now);
                        fprintf("\n[RECEDING HORIZON] 新たな障害物 (ID: %d) を捕捉! 軌道を動的更新します。\n", best_idx);
                    else
                        obj.init_world_diff_state = zeros(7, 3);
                        fprintf("\n=======================================================\n");
                        fprintf("[検知発動] 実状態センサーが前方障害物を捕捉! (ID: %d, Center=[%.2f, %.2f, %.2f])\n", ...
                            best_idx, tgt.p_center(1), tgt.p_center(2), tgt.p_center(3));
                    end
                    
                    obj.active_obs_idx = best_idx;
                    obj.obs_center     = tgt.p_center;
                    obj.obs_R          = tgt.R_obs;
                    obj.obs_radii      = tgt.ellipsoid_radii;
                    obj.obs_margin     = tgt.d_margin;
                    
                    obj.t_start        = time.t;
                    obj.p_start_replan = pL_cur;
                    obj.dir_nominal    = dir_nom;
                    obj.nominal_speed  = spd;
                    
                    vec_to_obs = obj.obs_center - obj.p_start_replan;
                    d_proj_to_center = dot(vec_to_obs, dir_nom);
                    p_closest_on_line = obj.p_start_replan + d_proj_to_center * dir_nom;
                    
                    % 特異点フリー Gram-Schmidt 直交基底
                    [~, min_dim] = min(abs(dir_nom));
                    v_arbitrary = zeros(3, 1);
                    v_arbitrary(min_dim) = 1.0;
                    
                    base_n1 = cross(dir_nom, v_arbitrary);
                    base_n1 = base_n1 / norm(base_n1);
                    base_n2 = cross(dir_nom, base_n1);
                    base_n2 = base_n2 / norm(base_n2);
                    
                    v_center_rel = obj.obs_center - p_closest_on_line;
                    d_center_lat = norm(v_center_rel);
                    p_rel_raw = obj.obs_R' * (v_center_rel / max(d_center_lat, 1e-4));
                    r_eff_est = 1.0 / norm(p_rel_raw ./ obj.obs_radii);
                    test_radius = max(r_eff_est + 1.0, 2.0);
                    
                    % 360度方向探索による最適安全退避法線の決定
                    best_score = -inf;
                    best_normal = base_n1;
                    test_angles = linspace(0, 2*pi, 36);
                    for ang = test_angles
                        cand_n = cos(ang) * base_n1 + sin(ang) * base_n2;
                        if d_center_lat > 1e-3 && dot(cand_n, -v_center_rel) < -0.1
                            continue;
                        end
                        test_apex = p_closest_on_line + test_radius * cand_n;
                        min_dist_other = inf;
                        for j = 1:length(obs_list)
                            if j == best_idx, continue; end
                            d_j = obj.calc_exact_surface_distance(test_apex, obs_list(j).p_center, obs_list(j).R_obs, obs_list(j).ellipsoid_radii);
                            min_dist_other = min(min_dist_other, d_j);
                        end
                        if min_dist_other > best_score
                            best_score = min_dist_other;
                            best_normal = cand_n;
                        end
                    end
                    
                    obj.dir_normal = best_normal;
                    b_cand = cross(dir_nom, obj.dir_normal);
                    obj.dir_binormal = b_cand / norm(b_cand);
                    
                    dist_line_to_obs = norm(obj.obs_center - p_closest_on_line);
                    p_rel_n = obj.obs_R' * obj.dir_normal;
                    r_eff_obs = 1.0 / norm(p_rel_n ./ obj.obs_radii);
                    
                    effective_Kp = 1.0;
                    try
                        if isprop(obj.self.controller, "param") && isfield(obj.self.controller.param, "F2")
                            raw_gain = obj.self.controller.param.F2(1);
                            effective_Kp = max(0.5, min(2.5, raw_gain / 50.0));
                        end
                    catch
                    end
                    
                    base_clearance = max(0.5, (r_eff_obs + obj.r_drone + obj.obs_margin) - dist_line_to_obs);
                    
                    t_geom = d_proj_to_center / spd;
                    t_pendulum = 2 * pi * sqrt(obj.L_cable / obj.gravity);
                    theta_max_rad = deg2rad(obj.max_swing_angle_deg);
                    a_load_allow  = obj.gravity * tan(theta_max_rad);
                    T_min_swing   = sqrt(10.0 * base_clearance / a_load_allow);
                    T_min_drone   = sqrt(12.0 * base_clearance / obj.max_acc_drone);
                    
                    obj.T_seg = max([t_geom, 1.2 * t_pendulum, T_min_swing, T_min_drone, 4.0]);
                    obj.t_duration = 2.0 * obj.T_seg;
                    
                    initial_pos_error = norm(pL_cur - (obj.p_start_replan + dot(pL_cur - obj.p_start_replan, dir_nom) * dir_nom));
                    estimated_max_acc = 6.5 * base_clearance / (obj.T_seg^2);
                    accel_tracking_error = estimated_max_acc / effective_Kp;
                    mass_ratio = obj.m_load_est / (obj.m_drone + obj.m_load_est);
                    swing_coupling_buffer = mass_ratio * (obj.L_cable / obj.gravity) * estimated_max_acc;
                    dynamic_buffer = max(0.20, initial_pos_error + accel_tracking_error + swing_coupling_buffer);
                    
                    actual_cable_vec = pQ_cur - pL_cur;
                    cable_offset_s = dot(actual_cable_vec, obj.dir_nominal);
                    
                    fprintf("  - 検知主体: %s (実表面距離: %.3f m, 進入速度: %.2f m/s)\n", sensor_owner, min_surf_dist, spd);
                    fprintf("  - 選択された動的退避方向: [%.2f, %.2f, %.2f] (他障害物余白: %.2f m)\n", ...
                        obj.dir_normal(1), obj.dir_normal(2), obj.dir_normal(3), best_score);
                    fprintf("  - 物理最適化時間: T_seg = %.2f s (全時間 T = %.2f s)\n", obj.T_seg, obj.t_duration);
                    
                    obj.plan_bezier_c6_error_spline_qp(dynamic_buffer, dist_line_to_obs, cable_offset_s);
                    obj.replan_active = true;
                    fprintf("=======================================================\n\n");
                end
            end
            
            % -------------------------------------------------------------
            % 2. 飛行中常時監視: 全障害物に対する網羅判定
            % -------------------------------------------------------------
            if cha == 'f' && ~isempty(obs_list)
                for j = 1:length(obs_list)
                    tgt_j = obs_list(j);
                    d_surf_L = obj.calc_exact_surface_distance(pL_cur, tgt_j.p_center, tgt_j.R_obs, tgt_j.ellipsoid_radii);
                    d_surf_Q = obj.calc_exact_surface_distance(pQ_cur, tgt_j.p_center, tgt_j.R_obs, tgt_j.ellipsoid_radii);
                    
                    if d_surf_L <= 0 && ~obj.warned_crash_load(j)
                        fprintf(2, "[CRITICAL ALARM] 荷物が障害物%d本体に衝突! (t=%.3f s, 侵入深さ=%.3f m)\n", j, time.t, -d_surf_L);
                        obj.warned_crash_load(j) = true;
                    elseif d_surf_L <= (obj.r_load + tgt_j.d_margin) && ~obj.warned_margin_load(j)
                        fprintf("[SAFETY WARN] 荷物が障害物%dのソフトマージン帯に侵入 (t=%.3f s, 表面残余=%.3f m)\n", j, time.t, d_surf_L);
                        obj.warned_margin_load(j) = true;
                    end
                    
                    if d_surf_Q <= 0 && ~obj.warned_crash_drone(j)
                        fprintf(2, "[CRITICAL ALARM] 機体が障害物%d本体に衝突! (t=%.3f s, 侵入深さ=%.3f m)\n", j, time.t, -d_surf_Q);
                        obj.warned_crash_drone(j) = true;
                    elseif d_surf_Q <= (obj.r_drone + tgt_j.d_margin) && ~obj.warned_margin_drone(j)
                        fprintf("[SAFETY WARN] 機体が障害物%dのソフトマージン帯に侵入 (t=%.3f s, 表面残余=%.3f m)\n", j, time.t, d_surf_Q);
                        obj.warned_margin_drone(j) = true;
                    end
                end
            end
            
            % -------------------------------------------------------------
            % 3. 軌道出力 ＆ C^6 シームレス合流
            % -------------------------------------------------------------
            if obj.replan_active
                tau = time.t - obj.t_start;
                if tau <= obj.t_duration
                    xd = obj.evaluate_smooth_trajectory(tau, xd_nominal);
                else
                    if ~obj.replan_done
                        fprintf("[BÉZIER C^6 QP] 障害物通過完了・完全シームレス復帰 (t=%.3f s)\n\n", time.t);
                        obj.t_merge_end = obj.t_start + obj.t_duration;
                        xd_end = obj.evaluate_smooth_trajectory(obj.t_duration, xd_nominal);
                        obj.p_merge_end = xd_end(1:3);
                        obj.v_merge_vec = xd_end(5:7);
                        obj.replan_active = false;
                        obj.replan_done   = true;
                    end
                    
                    dt_after = time.t - obj.t_merge_end;
                    xd = zeros(28, 1);
                    xd(1:3) = obj.p_merge_end + obj.v_merge_vec * dt_after;
                    xd(5:7) = obj.v_merge_vec;
                    xd(9:28) = 0;
                end
            elseif obj.replan_done
                dt_after = time.t - obj.t_merge_end;
                xd = zeros(28, 1);
                xd(1:3) = obj.p_merge_end + obj.v_merge_vec * dt_after;
                xd(5:7) = obj.v_merge_vec;
                xd(9:28) = 0;
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
        
        function plan_bezier_c6_error_spline_qp(obj, dynamic_buffer, dist_line_to_obs, cable_offset_s)
            N = obj.order;
            n_c = N + 1;
            T_seg = obj.T_seg;
            
            p_rel_n = obj.obs_R' * obj.dir_normal;
            r_eff_n = 1.0 / norm(p_rel_n ./ obj.obs_radii);
            p_rel_s = obj.obs_R' * obj.dir_nominal;
            r_eff_s = 1.0 / norm(p_rel_s ./ obj.obs_radii);
            
            max_r_obj = max(obj.r_drone, obj.r_load);
            a_mink = r_eff_n + max_r_obj + obj.obs_margin + dynamic_buffer;
            c_mink = r_eff_s + max_r_obj + obj.obs_margin + dynamic_buffer;
            
            % 頂点クリアランス要求量（確実に障害物外郭を安全にパスする値）
            delta_target = max(a_mink - dist_line_to_obs, 1.0);
            
            n_vars_1d  = 2 * n_c;
            n_vars_tot = 2 * n_vars_1d;
            
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
            
            % 等式制約 (42本)
            Aeq_1d = zeros(21, n_vars_1d);
            beq_n  = zeros(21, 1);
            beq_b  = zeros(21, 1);
            
            % 1) 始端 (u1=0): キャプチャしたワールド微分ベクトルを新基底へ射影
            for k = 0:6
                c_diff = obj.get_bezier_derivative_coeffs_at_u(N, k, 0.0);
                Aeq_1d(k + 1, 1:n_c) = c_diff / (T_seg^k);
                
                w_vec_k = obj.init_world_diff_state(k + 1, :)';
                beq_n(k + 1) = dot(w_vec_k, obj.dir_normal);
                beq_b(k + 1) = dot(w_vec_k, obj.dir_binormal);
            end
            
            % 2) 中間接続点 (u1=1, u2=0): 0〜6階微分完全一致 (Gap=0)
            for k = 0:6
                c_end   = obj.get_bezier_derivative_coeffs_at_u(N, k, 1.0);
                c_start = obj.get_bezier_derivative_coeffs_at_u(N, k, 0.0);
                Aeq_1d(7 + k + 1, 1:n_c)       =  c_end   / (T_seg^k);
                Aeq_1d(7 + k + 1, n_c+1:2*n_c) = -c_start / (T_seg^k);
            end
            
            % 3) 終端 (u2=1): 0〜6階微分 = 0
            for k = 0:6
                c_diff = obj.get_bezier_derivative_coeffs_at_u(N, k, 1.0);
                Aeq_1d(14 + k + 1, n_c+1:2*n_c) = c_diff / (T_seg^k);
            end
            
            Aeq = blkdiag(Aeq_1d, Aeq_1d);
            beq = [beq_n; beq_b];
            
            % 不等式制約: 楕円断面逆算 ＋ 頂点厳密通過
            alpha_dyn = obj.L_cable / (obj.gravity * (T_seg^2));
            A_ineq = [];
            b_ineq = [];
            
            % 区間 1 (接近側)
            N_samp = 7;
            u1_samples = linspace(0.3, 1.0, N_samp);
            for u = u1_samples
                B_val      = obj.eval_bernstein_vector(N, u);
                B_ddot_val = obj.get_bezier_derivative_coeffs_at_u(N, 2, u);
                BQ_val     = B_val + alpha_dyn * B_ddot_val;
                
                delta_s_L = (1.0 - u) * obj.nominal_speed * T_seg;
                delta_s_Q = delta_s_L - cable_offset_s;
                
                req_L = 0.0;
                if abs(delta_s_L) < c_mink
                    req_L = max(0, a_mink * sqrt(1.0 - (delta_s_L / c_mink)^2) - dist_line_to_obs);
                end
                req_Q = 0.0;
                if abs(delta_s_Q) < c_mink
                    req_Q = max(0, a_mink * sqrt(1.0 - (delta_s_Q / c_mink)^2) - dist_line_to_obs);
                end
                
                r_L = zeros(1, n_vars_tot); r_L(1:n_c) = -B_val;
                r_Q = zeros(1, n_vars_tot); r_Q(1:n_c) = -BQ_val;
                A_ineq = [A_ineq; r_L; r_Q];
                b_ineq = [b_ineq; -req_L; -req_Q];
            end
            
            % 区間 2 (離脱側)
            u2_samples = linspace(0.0, 0.7, N_samp);
            for u = u2_samples
                B_val      = obj.eval_bernstein_vector(N, u);
                B_ddot_val = obj.get_bezier_derivative_coeffs_at_u(N, 2, u);
                BQ_val     = B_val + alpha_dyn * B_ddot_val;
                
                delta_s_L = u * obj.nominal_speed * T_seg;
                delta_s_Q = delta_s_L - cable_offset_s;
                
                req_L = 0.0;
                if abs(delta_s_L) < c_mink
                    req_L = max(0, a_mink * sqrt(1.0 - (delta_s_L / c_mink)^2) - dist_line_to_obs);
                end
                req_Q = 0.0;
                if abs(delta_s_Q) < c_mink
                    req_Q = max(0, a_mink * sqrt(1.0 - (delta_s_Q / c_mink)^2) - dist_line_to_obs);
                end
                
                r_L = zeros(1, n_vars_tot); r_L(n_c+1:2*n_c) = -B_val;
                r_Q = zeros(1, n_vars_tot); r_Q(n_c+1:2*n_c) = -BQ_val;
                A_ineq = [A_ineq; r_L; r_Q];
                b_ineq = [b_ineq; -req_L; -req_Q];
            end
            
            % 中間頂点でのハード下限保証 (最接近点で確実に delta_target 退避)
            B_mid = obj.eval_bernstein_vector(N, 1.0);
            B_ddot_mid = obj.get_bezier_derivative_coeffs_at_u(N, 2, 1.0);
            BQ_mid = B_mid + alpha_dyn * B_ddot_mid;
            
            r_mid_L = zeros(1, n_vars_tot); r_mid_L(1:n_c) = -B_mid;
            r_mid_Q = zeros(1, n_vars_tot); r_mid_Q(1:n_c) = -BQ_mid;
            A_ineq = [A_ineq; r_mid_L; r_mid_Q];
            b_ineq = [b_ineq; -delta_target; -delta_target];
            
            % 振れ角制限
            theta_max_rad = deg2rad(obj.max_swing_angle_deg);
            a_load_limit  = obj.gravity * tan(theta_max_rad);
            u_check = [0.5, 0.8, 1.0];
            for u = u_check
                c_ddot = obj.get_bezier_derivative_coeffs_at_u(N, 2, u) / (T_seg^2);
                r_pos = zeros(1, n_vars_tot); r_pos(1:n_c) = c_ddot;
                r_neg = zeros(1, n_vars_tot); r_neg(1:n_c) = -c_ddot;
                A_ineq = [A_ineq; r_pos; r_neg];
                b_ineq = [b_ineq; a_load_limit; a_load_limit];
            end
            
            % 機体加速度制限
            for u = u_check
                c_ddot = obj.get_bezier_derivative_coeffs_at_u(N, 2, u) / (T_seg^2);
                c_snap = obj.get_bezier_derivative_coeffs_at_u(N, 4, u) / (T_seg^4);
                c_drone_acc = c_ddot + (obj.L_cable / obj.gravity) * c_snap;
                
                r_pos = zeros(1, n_vars_tot); r_pos(1:n_c) = c_drone_acc;
                r_neg = zeros(1, n_vars_tot); r_neg(1:n_c) = -c_drone_acc;
                A_ineq = [A_ineq; r_pos; r_neg];
                b_ineq = [b_ineq; obj.max_acc_drone; obj.max_acc_drone];
            end
            
            opts = optimoptions('quadprog', ...
                'Display', 'off', ...
                'Algorithm', 'interior-point-convex', ...
                'MaxIterations', 300, ...
                'ConstraintTolerance', 1e-4, ...
                'OptimalityTolerance', 1e-4);
            
            [X_opt, ~, exitflag, output] = quadprog(H, f, A_ineq, b_ineq, Aeq, beq, [], [], [], opts);
            
            if exitflag < 1
                fprintf("[BÉZIER C^6 DEBUG] 不等式制約付きQP未収束 (exitflag=%d). 等式拘束フォールバックを実行\n", exitflag);
                X_fb = pinv(Aeq) * beq;
                cn1 = X_fb(1:n_c);         cn2 = X_fb(n_c+1:2*n_c);
                cb1 = X_fb(2*n_c+1:3*n_c); cb2 = X_fb(3*n_c+1:4*n_c);
            else
                fprintf("[BÉZIER C^6] 最適化成功! (反復=%d, ピーク退避量=%.3fm)\n", output.iterations, delta_target);
                cn1 = X_opt(1:n_c);             cn2 = X_opt(n_c+1:2*n_c);
                cb1 = X_opt(2*n_c+1:3*n_c);     cb2 = X_opt(3*n_c+1:4*n_c);
            end
            
            obj.coeffs_delta_seg1 = [cn1, cb1];
            obj.coeffs_delta_seg2 = [cn2, cb2];
            
            obj.verify_c6_continuity();
        end
        
        function verify_c6_continuity(obj)
            N = obj.order;
            T_seg = obj.T_seg;
            names = ["位置 (0階)", "速度 (1階)", "加速度 (2階)", "Jerk (3階)", "Snap (4階)", "Crack (5階)", "Pop (6階)"];
            
            fprintf("-------------------------------------------------------\n");
            fprintf("[理論検証] 中間接続点における 0〜6階微分の不連続量 (Gap):\n");
            for k = 0:6
                c_end   = obj.get_bezier_derivative_coeffs_at_u(N, k, 1.0);
                c_start = obj.get_bezier_derivative_coeffs_at_u(N, k, 0.0);
                
                d1_n = dot(c_end,   obj.coeffs_delta_seg1(:, 1)) / (T_seg^k);
                d1_b = dot(c_end,   obj.coeffs_delta_seg1(:, 2)) / (T_seg^k);
                
                d2_n = dot(c_start, obj.coeffs_delta_seg2(:, 1)) / (T_seg^k);
                d2_b = dot(c_start, obj.coeffs_delta_seg2(:, 2)) / (T_seg^k);
                
                gap = norm([d1_n - d2_n; d1_b - d2_b]);
                fprintf("  - %-12s ギャップ: %.3e\n", names(k + 1), gap);
            end
            fprintf("-------------------------------------------------------\n");
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
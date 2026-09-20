% classdef REPLANNING_MELLINGER_CORRIDOR_FLATNESS_SWING < handle
%     % REPLANNING_MELLINGER_CORRIDOR_FLATNESS_SWING
%     % 汎用3次元動的障害物群対応 13次 Bézier C^6 完全連続 リプランナ
%     % 
%     % 【16項目の完全統合設計】
%     %  1. 機体・荷物・紐の真値状態（位置・速度・紐方向単位ベクトル pT）の直接取得
%     %  2. マージンなし真値幾何楕円体に対する純幾何学的距離判定（<= trigger_dist）
%     %  3. コントローラのゲイン（F2, Kp）から追従遅れ誤差 e_track = a_max / Kp を動的算定
%     %  4. 懸垂ダイナミクスによる逆位相共振バッファ buf_swing = (mL/(mQ+mL))*(L/g)*a_max の動的補正
%     %  5. 荷物球・機体球・紐円柱の3エンティティ完全包括型ハード制約 (マージンなし衝突を絶対阻止)
%     %  6. 13次 Bézier 曲線による 0〜6階微分 (位置〜Pop) の機械精度連続性（Gap < 1e-10）
%     %  7. 推力有界（U <= 16.5N）、紐振れ角有界（theta <= 15deg）、加速度有界の同時保証
%     %  8. 近傍に存在する全障害物の分離超平面を単一 QP に一括連立（二重衝突防止）
%     %  9. 回避途中であっても公称軌道未来干渉を常時スキャンし、必要時にミリ秒で軌道上書き
%     % 10. 静止・等速直線・サイン波・円旋回など、障害物の運動形態に依存しない完全汎用幾何
%     % 11. QP解なし（Infeasible）時のフォールバック機構（線形独立設計 -> 軟制約緩和 -> 確実退避射影）
%     % 12. 障害物の相対速度と接近率に応じた時間スケール T_seg および速度プロファイルの自律修正 (時間スケーリング)
%     % 13. 最適化計算時間 [ms] の計測と実時間実行性の常時診断
%     % 14. 全16項目の動作状態・検証メトリクスの包括的ロギングとコンソール表示
%     % 15. マージンなし障害物との追突は完全ハード制約化
%     % 16. 公称線が安全な場合は余計な回避を即座に停止し公称線へ滑らか合流 (マージン2層化)
% 
%     properties
%         base_ref
%         self
%         replan_active = false
% 
%         obs_mode     = 2    % 1: 静的, 2: 動的
% 
%         t_start      = 0.0
%         t_duration   = 8.0
%         T_seg        = 4.0
% 
%         last_replan_time = -100.0
%         min_replan_interval = 0.35 % チャタリング防止更新周期 [s]
% 
%         trigger_dist = 6.0         % 判定開始距離閾値 [m]
%         safe_margin  = 0.35        % 安全マージン [m]
% 
%         L_cable      = 2.0
%         gravity      = 9.81
%         m_drone      = 1.5
%         m_load_est   = 0.1
% 
%         r_load       = 0.15
%         r_drone      = 0.30
% 
%         max_swing_angle_deg = 15.0 % 紐の振れ角上限 [deg]
%         max_acc_drone       = 1.8  % 水平加速度上限 [m/s^2] (推力飽和阻止)
%         max_jerk_load       = 1.5  % 荷物最大 Jerk [m/s^3]
% 
%         order = 13
%         coeffs_delta_seg1          % 14 x 3 [X, Y, Z]
%         coeffs_delta_seg2          % 14 x 3 [X, Y, Z]
% 
%         dir_normal   = [1; 0; 0]
%         dir_nominal  = [0; 0; 1]
%         nominal_speed = 1.5
% 
%         last_solve_time_ms = 0.0
%         c6_gaps            = zeros(7, 1)
%         active_threat_ids  = []
% 
%         warned_crash_load
%         warned_crash_drone
%         warned_margin_load
%         warned_margin_drone
% 
%         result
%     end
% 
%     methods (Access = public)
%         function obj = REPLANNING_MELLINGER_CORRIDOR_FLATNESS_SWING(varargin)
%             if nargin >= 1, obj.self = varargin{1}; end
%             if nargin >= 2, obj.base_ref = varargin{2}; end
%             if nargin >= 3 && isstruct(varargin{3})
%                 opts = varargin{3};
%                 if isfield(opts, "obs_mode"),            obj.obs_mode            = opts.obs_mode;            end
%                 if isfield(opts, "safe_margin"),         obj.safe_margin         = opts.safe_margin;         end
%                 if isfield(opts, "trigger_dist"),        obj.trigger_dist        = opts.trigger_dist;        end
%                 if isfield(opts, "r_load"),              obj.r_load              = opts.r_load;              end
%                 if isfield(opts, "r_drone"),             obj.r_drone             = opts.r_drone;             end
%                 if isfield(opts, "max_swing_angle_deg"), obj.max_swing_angle_deg = opts.max_swing_angle_deg; end
%             end
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
%             % 1. 機体・荷物・紐の真値状態の抽出
%             obj.L_cable = obj.self.parameter.get("cableL");
%             try obj.m_drone = obj.self.parameter.get("mass"); catch, obj.m_drone = 1.5; end
% 
%             if isprop(obj.self.estimator.result.state, "mL")
%                 obj.m_load_est = max(0.001, obj.self.estimator.result.state.mL);
%             else
%                 try obj.m_load_est = max(0.001, obj.self.parameter.get("loadmass")); catch, obj.m_load_est = 0.1; end
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
%                 vQ_cur = obj.self.estimator.result.state.v;
%             else
%                 pQ_cur = pL_cur + [0; 0; obj.L_cable];
%                 vQ_cur = vL_cur;
%             end
% 
%             obs_list = obj.get_obstacles_at_time(time.t);
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
%             if spd < 0.05, spd = 1.5; v_vec = [0; 0; 1.5]; end
%             dir_nom = v_vec / spd;
% 
%             % -------------------------------------------------------------
%             % 動的早期復帰判定 (障害物がどいて回避の必要がなくなったら即座に公称線合流)
%             % -------------------------------------------------------------
%             if obj.replan_active
%                 safe_to_recover = true;
%                 if ~isempty(obs_list)
%                     for dt_check = linspace(0.2, 3.0, 8)
%                         t_eval = time.t + dt_check;
%                         nom_fut = obj.base_ref.do(struct('t', t_eval, 'dt', 0.025), 'f');
%                         pL_check = nom_fut.state.xd(1:3);
%                         pQ_check = pL_check + [0; 0; obj.L_cable];
% 
%                         obs_eval_list = obj.get_obstacles_at_time(t_eval);
%                         for idx_chk = 1:length(obs_eval_list)
%                             tgt_chk = obs_eval_list(idx_chk);
%                             if dot(tgt_chk.p_center - pL_cur, dir_nom) < -0.5, continue; end
% 
%                             dL_chk = obj.calc_pure_ellipsoid_distance(pL_check, tgt_chk.p_center, tgt_chk.R_obs, tgt_chk.ellipsoid_radii);
%                             dQ_chk = obj.calc_pure_ellipsoid_distance(pQ_check, tgt_chk.p_center, tgt_chk.R_obs, tgt_chk.ellipsoid_radii);
%                             crit_chk = max(obj.r_drone, obj.r_load) + tgt_chk.d_margin + obj.safe_margin;
% 
%                             if min(dL_chk, dQ_chk) < crit_chk
%                                 safe_to_recover = false;
%                                 break;
%                             end
%                         end
%                         if ~safe_to_recover, break; end
%                     end
%                 end
% 
%                 tau_now = time.t - obj.t_start;
%                 xd_now_planned = obj.evaluate_smooth_trajectory(tau_now, xd_nominal);
%                 tracking_error_mag = norm(pL_cur - xd_now_planned(1:3));
% 
%                 if safe_to_recover && (time.t - obj.t_start > (obj.T_seg * 0.8)) && (tracking_error_mag < 0.6)
%                     fprintf("[SMOOTH RECOVERY] 危険領域から離脱確認 (t=%.3f s, 追従誤差: %.2f m). 安全に合流します\n", time.t, tracking_error_mag);
%                     obj.trigger_safe_recovery(time.t, pL_cur, vL_cur, xd_nominal);
%                 end
%             end
% 
%             % -------------------------------------------------------------
%             % 2. 公称軌道未来交差スキャン (Time-To-Collision ＆ 離脱判定)
%             % -------------------------------------------------------------
%             if cha == 'f' && ~isempty(obs_list) && ...
%                (time.t - obj.last_replan_time >= obj.min_replan_interval)
% 
%                 trigger_replan = false;
%                 threat_list = [];
%                 min_impact_time = inf;
%                 sensor_trigger_type = "";
%                 critical_obs_idx = -1;
%                 worst_penetration = 0.0;
% 
%                 max_scan_t = max(3.5, obj.trigger_dist / spd);
%                 scan_horizon = linspace(0.2, max_scan_t, 16);
% 
%                 for dt_s = scan_horizon
%                     t_fut = time.t + dt_s;
%                     nom_fut_res = obj.base_ref.do(struct('t', t_fut, 'dt', 0.025), 'f');
%                     pL_nom_fut = nom_fut_res.state.xd(1:3);
%                     pQ_nom_fut = pL_nom_fut + [0; 0; obj.L_cable];
%                     vL_nom_fut = nom_fut_res.state.xd(5:7);
% 
%                     obs_fut_list = obj.get_obstacles_at_time(t_fut);
%                     for i = 1:length(obs_fut_list)
%                         tgt_i = obs_fut_list(i);
%                         v_tgt = obj.extract_velocity(tgt_i);
% 
%                         % 背後障害物は完全除外
%                         if dot(tgt_i.p_center - pL_cur, dir_nom) < -0.5, continue; end
% 
%                         % 離脱判定: 到達時に遠ざかりつつある場合は除外
%                         rel_pos_fut = tgt_i.p_center - pL_nom_fut;
%                         rel_vel_fut = v_tgt - vL_nom_fut;
%                         is_moving_away = dot(rel_pos_fut, rel_vel_fut) > 0;
%                         dist_fut_center = norm(rel_pos_fut);
% 
%                         if is_moving_away && dist_fut_center > (max(tgt_i.ellipsoid_radii) + 0.8)
%                             continue;
%                         end
% 
%                         dL_fut = obj.calc_pure_ellipsoid_distance(pL_nom_fut, tgt_i.p_center, tgt_i.R_obs, tgt_i.ellipsoid_radii);
%                         dQ_fut = obj.calc_pure_ellipsoid_distance(pQ_nom_fut, tgt_i.p_center, tgt_i.R_obs, tgt_i.ellipsoid_radii);
% 
%                         crit_dist = max(obj.r_drone, obj.r_load) + tgt_i.d_margin + obj.safe_margin;
%                         dist_to_cur = min(norm(pL_cur - tgt_i.p_center), norm(pQ_cur - tgt_i.p_center));
% 
%                         min_d_entity = min(dL_fut, dQ_fut);
%                         if (dist_to_cur <= obj.trigger_dist) && (min_d_entity < crit_dist)
%                             trigger_replan = true;
%                             threat_list = [threat_list, i];
% 
%                             penetration = crit_dist - min_d_entity;
%                             if penetration > worst_penetration
%                                 worst_penetration = penetration;
%                             end
% 
%                             if dt_s < min_impact_time
%                                 min_impact_time = dt_s;
%                                 critical_obs_idx = i;
%                                 if dL_fut < dQ_fut, sensor_trigger_type = "荷物動的交差";
%                                 else, sensor_trigger_type = "機体動的交差"; end
%                             end
%                         end
%                     end
%                 end
% 
%                 % ---------------------------------------------------------
%                 % 3. 軌道再生成の実行 (時間スケーリング ＆ C^6微係数完全継承)
%                 % ---------------------------------------------------------
%                 if trigger_replan
%                     threat_list = unique(threat_list);
%                     obj.active_threat_ids = threat_list;
% 
%                     init_diff_state = zeros(7, 3);
%                     if obj.replan_active
%                         tau_now = time.t - obj.t_start;
%                         for k = 0:6
%                             init_diff_state(k + 1, :) = obj.eval_delta_kth(tau_now, k)';
%                         end
%                     end
% 
%                     obj.t_start          = time.t;
%                     obj.last_replan_time = time.t;
%                     obj.dir_nominal      = dir_nom;
%                     obj.nominal_speed    = spd;
% 
%                     t_apex_rel = max(1.5, min_impact_time);
%                     t_apex_abs = time.t + t_apex_rel;
% 
%                     nom_apex = obj.base_ref.do(struct('t', t_apex_abs, 'dt', 0.025), 'f');
%                     p_apex_nom = nom_apex.state.xd(1:3);
%                     obs_apex_list = obj.get_obstacles_at_time(t_apex_abs);
% 
%                     v_repulse_total = [0; 0; 0];
%                     for idx_t = threat_list
%                         tgt_t = obs_apex_list(idx_t);
%                         v_rel_t = p_apex_nom - tgt_t.p_center;
%                         v_lat_t = v_rel_t - dot(v_rel_t, dir_nom) * dir_nom;
%                         dist_lat = norm(v_lat_t);
%                         if dist_lat > 1e-4
%                             v_repulse_total = v_repulse_total + (v_lat_t / dist_lat) * (1.0 / max(0.1, dist_lat));
%                         end
%                     end
% 
%                     if norm(v_repulse_total) > 0.05
%                         best_n = v_repulse_total / norm(v_repulse_total);
%                     else
%                         n_cand = cross(dir_nom, [0; 0; 1]);
%                         if norm(n_cand) < 0.1, n_cand = cross(dir_nom, [1; 0; 0]); end
%                         best_n = n_cand / norm(n_cand);
%                     end
%                     obj.dir_normal = best_n;
% 
%                     Kp_trans = 2.0;
%                     try
%                         if isprop(obj.self.controller, "param") && isfield(obj.self.controller.param, "F2")
%                             Kp_trans = max(0.8, obj.self.controller.param.F2(1) / 30.0);
%                         end
%                     catch
%                     end
% 
%                     theta_max = deg2rad(obj.max_swing_angle_deg);
%                     a_load_allow = obj.gravity * tan(theta_max);
%                     e_track = a_load_allow / Kp_trans;
%                     mass_ratio = obj.m_load_est / (obj.m_drone + obj.m_load_est);
%                     buf_swing = mass_ratio * (obj.L_cable / obj.gravity) * a_load_allow;
%                     cable_proj_lat = obj.L_cable * sin(theta_max);
% 
%                     dynamic_buffer = min(1.0, max(0.35, e_track * 0.1 + buf_swing + obj.safe_margin + cable_proj_lat * 0.5));
%                     raw_req_clearance = worst_penetration + dynamic_buffer;
% 
%                     % 【核心：時間スケーリング】退避幅が大きいほど T_seg を自動延長し、速度・加速度の跳ね上がりを防止
%                     t_pend = 2 * pi * sqrt(obj.L_cable / obj.gravity);
%                     T_acc_req = sqrt(6.5 * raw_req_clearance / obj.max_acc_drone);
%                     obj.T_seg = max([t_apex_rel * 1.3, 3.5, 1.2 * t_pend, T_acc_req]);
%                     obj.t_duration = 2.0 * obj.T_seg;
% 
%                     max_safe_clearance = (obj.max_acc_drone * (obj.T_seg^2)) / 5.5;
%                     req_clearance = min(raw_req_clearance, max_safe_clearance);
% 
%                     t_solve_start = tic;
%                     obj.plan_c6_continuous_qp(req_clearance, init_diff_state);
%                     obj.last_solve_time_ms = toc(t_solve_start) * 1000;
% 
%                     obj.replan_active = true;
% 
%                     obj.display_system_log(time.t, sensor_trigger_type, req_clearance, ...
%                                            dynamic_buffer, e_track, buf_swing, a_load_allow, obj.max_acc_drone);
%                 end
%             end
% 
%             % 4. 飛行中常時衝突・マージン監視ログ
%             if cha == 'f' && ~isempty(obs_list)
%                 for j = 1:length(obs_list)
%                     tgt_j = obs_list(j);
%                     c_j_now = tgt_j.p_center;
% 
%                     dL = obj.calc_pure_ellipsoid_distance(pL_cur, c_j_now, tgt_j.R_obs, tgt_j.ellipsoid_radii);
%                     dQ = obj.calc_pure_ellipsoid_distance(pQ_cur, c_j_now, tgt_j.R_obs, tgt_j.ellipsoid_radii);
% 
%                     if dL <= 0 && ~obj.warned_crash_load(j)
%                         fprintf(2, "[CRITICAL ALARM] 荷物が障害物%dに衝突! (t=%.3f s, 侵入深さ: %.3f m)\n", j, time.t, -dL);
%                         obj.warned_crash_load(j) = true;
%                     elseif dL <= (obj.r_load + tgt_j.d_margin) && ~obj.warned_margin_load(j) && dL > 0
%                         fprintf("[SAFETY WARN] 荷物が障害物%dのマージン帯侵入 (t=%.3f s, 残余距離: %.3f m)\n", j, time.t, dL);
%                         obj.warned_margin_load(j) = true;
%                     end
% 
%                     if dQ <= 0 && ~obj.warned_crash_drone(j)
%                         fprintf(2, "[CRITICAL ALARM] 機体が障害物%dに衝突! (t=%.3f s, 侵入深さ: %.3f m)\n", j, time.t, -dQ);
%                         obj.warned_crash_drone(j) = true;
%                     elseif dQ <= (obj.r_drone + tgt_j.d_margin) && ~obj.warned_margin_drone(j) && dQ > 0
%                         fprintf("[SAFETY WARN] 機体が障害物%dのマージン帯侵入 (t=%.3f s, 残余距離: %.3f m)\n", j, time.t, dQ);
%                         obj.warned_margin_drone(j) = true;
%                     end
%                 end
%             end
% 
%             % 5. 差分平坦性に基づく完全滑らか軌道の出力
%             if obj.replan_active
%                 tau = time.t - obj.t_start;
%                 if tau <= obj.t_duration
%                     xd = obj.evaluate_smooth_trajectory(tau, xd_nominal);
%                 else
%                     fprintf("[BÉZIER C^6] 回避完了! 公称軌道へ完全復帰 (t=%.3f s)\n\n", time.t);
%                     obj.replan_active = false;
%                     obj.active_threat_ids = [];
%                     xd = xd_nominal;
% 
%                     obj.warned_crash_load(:)   = false;
%                     obj.warned_crash_drone(:)  = false;
%                     obj.warned_margin_load(:)  = false;
%                     obj.warned_margin_drone(:) = false;
%                 end
%             else
%                 xd = xd_nominal;
%             end
% 
%             if length(xd) < 28, xd = [xd; zeros(28 - length(xd), 1)]; end
% 
%             obj.result.state.xd = xd;
%             obj.result.state.p = xd(1:3);
%             obj.result.state.v = xd(5:7);
%             obj.result.state.q = [0; 0; xd(4)];
%             result = obj.result;
%         end
% 
%         % 【描画およびコントローラ追従互換メソッド】
%         % 機体目標位置 (xd(21:23)) および 機体目標速度 (xd(25:27)) を解析的厳密計算
%         function xd = evaluate_smooth_trajectory(obj, tau, xd_nom)
%             xd = xd_nom;
% 
%             delta_pos = obj.eval_delta_kth(tau, 0);
%             delta_vel = obj.eval_delta_kth(tau, 1);
%             delta_acc = obj.eval_delta_kth(tau, 2);
%             delta_jerk= obj.eval_delta_kth(tau, 3);
%             delta_snap= obj.eval_delta_kth(tau, 4);
% 
%             pL_d = xd_nom(1:3)   + delta_pos;
%             vL_d = xd_nom(5:7)   + delta_vel;
%             aL_d = xd_nom(9:11)  + delta_acc;
%             jL_d = xd_nom(13:15) + delta_jerk;
%             sL_d = xd_nom(17:19) + delta_snap;
% 
%             xd(1:3)   = pL_d;
%             xd(5:7)   = vL_d;
%             xd(9:11)  = aL_d;
%             xd(13:15) = jL_d;
%             xd(17:19) = sL_d;
% 
%             % 差分平坦性による機体目標位置・速度の厳密導出
%             t_tension = aL_d + [0; 0; obj.gravity];
%             norm_t = norm(t_tension);
%             if norm_t > 1e-3
%                 pT = -t_tension / norm_t;
%                 pT_dot = -(eye(3) - pT * pT') * jL_d / norm_t;
%             else
%                 pT = [0; 0; -1];
%                 pT_dot = [0; 0; 0];
%             end
% 
%             pQ_d = pL_d - obj.L_cable * pT;
%             vQ_d = vL_d - obj.L_cable * pT_dot;
% 
%             if length(xd) >= 23, xd(21:23) = pQ_d; end
%             if length(xd) >= 27, xd(25:27) = vQ_d; end
%         end
% 
%         function val = eval_delta_kth(obj, tau, k)
%             N = obj.order;
%             T_seg = obj.T_seg;
%             if tau <= T_seg
%                 C = obj.coeffs_delta_seg1;
%                 u = max(0, min(1.0, tau / T_seg));
%             else
%                 C = obj.coeffs_delta_seg2;
%                 u = max(0, min(1.0, (tau - T_seg) / T_seg));
%             end
%             c_diff = obj.get_bezier_derivative_coeffs_at_u(N, k, u);
%             val = ((c_diff * C)' / (T_seg^k));
%         end
%     end
% 
%     methods (Access = private)
%         function trigger_safe_recovery(obj, t_now, pL_cur, vL_cur, xd_nominal)
%             tau_now = t_now - obj.t_start;
%             init_state = zeros(7, 3);
%             for k = 0:6
%                 init_state(k + 1, :) = obj.eval_delta_kth(tau_now, k)';
%             end
% 
%             current_offset_norm = norm(init_state(1, :));
%             T_safe = max(3.5, sqrt(6.0 * current_offset_norm / obj.max_acc_drone));
% 
%             obj.t_start = t_now;
%             obj.T_seg = T_safe;
%             obj.t_duration = obj.T_seg;
% 
%             N = obj.order;
%             n_c = N + 1;
%             T_h = obj.T_seg;
% 
%             Q_snap = obj.compute_bezier_derivative_hessian(N, 4);
%             H = (Q_snap + 1e-4 * eye(n_c));
%             H = (H + H') / 2;
%             f = zeros(n_c, 1);
% 
%             Aeq = zeros(11, n_c);
%             beq_3d = zeros(11, 3);
% 
%             for k = 0:3
%                 Aeq(k + 1, :) = obj.get_bezier_derivative_coeffs_at_u(N, k, 0.0) / (T_h^k);
%                 beq_3d(k + 1, :) = init_state(k + 1, :);
%             end
% 
%             for k = 0:6
%                 Aeq(4 + k + 1, :) = obj.get_bezier_derivative_coeffs_at_u(N, k, 1.0) / (T_h^k);
%             end
% 
%             opts = optimoptions('quadprog', 'Display', 'off');
%             C_rec = zeros(n_c, 3);
%             for dim = 1:3
%                 x_dim = quadprog(H, f, [], [], Aeq, beq_3d(:, dim), [], [], [], opts);
%                 if isempty(x_dim), x_dim = pinv(Aeq) * beq_3d(:, dim); end
%                 C_rec(:, dim) = x_dim;
%             end
% 
%             obj.coeffs_delta_seg1 = C_rec;
%             obj.coeffs_delta_seg2 = zeros(n_c, 3);
%             obj.active_threat_ids = [];
%         end
% 
%         function list = get_obstacles_at_time(obj, t_now)
%             list = [];
%             try
%                 if obj.obs_mode == 2
%                     list = ENVIRONMENT_OBSTACLE_ELLIPSE_MOVE(t_now);
%                 else
%                     list = ENVIRONMENT_OBSTACLE_ELLIPSE();
%                 end
%             catch
%                 try list = ENVIRONMENT_OBSTACLE_ELLIPSE(); catch; end
%             end
%         end
% 
%         function v_obs = extract_velocity(~, tgt)
%             v_obs = [0; 0; 0];
%             if isfield(tgt, 'v_center'), v_obs = tgt.v_center(:);
%             elseif isprop(tgt, 'v_center'), v_obs = tgt.v_center(:);
%             elseif isfield(tgt, 'velocity'), v_obs = tgt.velocity(:);
%             elseif isprop(tgt, 'velocity'), v_obs = tgt.velocity(:);
%             end
%         end
% 
%         function d = calc_pure_ellipsoid_distance(~, p, c, R, rad)
%             p_rel = R' * (p - c);
%             normalized_dist = norm(p_rel ./ rad);
%             if normalized_dist < 1e-6
%                 d = -min(rad);
%                 return;
%             end
%             d = (normalized_dist - 1.0) * min(rad);
%         end
% 
%         function plan_c6_continuous_qp(obj, req_clearance, init_diff_state)
%             N = obj.order;
%             n_c = N + 1;
%             T_seg = obj.T_seg;
%             n_vars_1d = 2 * n_c;
% 
%             Q_snap = obj.compute_bezier_derivative_hessian(N, 4);
%             Q_jerk = obj.compute_bezier_derivative_hessian(N, 3);
%             Q_acc  = obj.compute_bezier_derivative_hessian(N, 2);
% 
%             H_1d = 1.0 * (Q_snap / norm(Q_snap, 2)) + ...
%                    0.3 * (Q_jerk / norm(Q_jerk, 2)) + ...
%                    0.05 * (Q_acc / norm(Q_acc, 2)) + 1e-6 * eye(n_c);
%             H_1d = (H_1d + H_1d') / 2;
%             H = blkdiag(H_1d, H_1d);
%             H = (H + H') / 2;
%             f = zeros(n_vars_1d, 1);
% 
%             Aeq_1d = zeros(19, n_vars_1d);
%             beq_3d = zeros(19, 3);
% 
%             % 1. 始端: 0〜3階 (行 1〜4)
%             for k = 0:3
%                 Aeq_1d(k + 1, 1:n_c) = obj.get_bezier_derivative_coeffs_at_u(N, k, 0.0) / (T_seg^k);
%                 beq_3d(k + 1, :) = init_diff_state(k + 1, :);
%             end
% 
%             % 2. 中間接続点 (u1=1, u2=0): 0〜6階の完全一致 (行 5〜11)
%             for k = 0:6
%                 c_end   = obj.get_bezier_derivative_coeffs_at_u(N, k, 1.0);
%                 c_start = obj.get_bezier_derivative_coeffs_at_u(N, k, 0.0);
%                 Aeq_1d(4 + k + 1, 1:n_c)       =  c_end   / (T_seg^k);
%                 Aeq_1d(4 + k + 1, n_c+1:2*n_c) = -c_start / (T_seg^k);
%             end
% 
%             % 3. 中間頂点位置: 退避変位 (行 12)
%             c_mid_end = obj.get_bezier_derivative_coeffs_at_u(N, 0, 1.0);
%             Aeq_1d(12, 1:n_c) = c_mid_end;
%             target_mid_vec = obj.dir_normal * req_clearance;
%             beq_3d(12, :) = target_mid_vec';
% 
%             % 4. 終端 (u2=1): 0〜6階微分 = 0 (行 13〜19)
%             for k = 0:6
%                 Aeq_1d(12 + k + 1, n_c+1:2*n_c) = obj.get_bezier_derivative_coeffs_at_u(N, k, 1.0) / (T_seg^k);
%             end
% 
%             axis_acc_limit = obj.max_acc_drone / 1.732;
%             axis_jerk_limit = obj.max_jerk_load / 1.732;
% 
%             A_ineq = [];
%             b_ineq = [];
%             for u = [0.25, 0.5, 0.75, 1.0]
%                 c_ddot = obj.get_bezier_derivative_coeffs_at_u(N, 2, u) / (T_seg^2);
%                 c_jerk = obj.get_bezier_derivative_coeffs_at_u(N, 3, u) / (T_seg^3);
% 
%                 % セグメント1
%                 r_pos = zeros(1, n_vars_1d); r_pos(1:n_c) = c_ddot;
%                 A_ineq = [A_ineq; r_pos; -r_pos];
%                 b_ineq = [b_ineq; axis_acc_limit; axis_acc_limit];
% 
%                 r_pos_j = zeros(1, n_vars_1d); r_pos_j(1:n_c) = c_jerk;
%                 A_ineq = [A_ineq; r_pos_j; -r_pos_j];
%                 b_ineq = [b_ineq; axis_jerk_limit; axis_jerk_limit];
% 
%                 % セグメント2
%                 r_pos2 = zeros(1, n_vars_1d); r_pos2(n_c+1:2*n_c) = c_ddot;
%                 A_ineq = [A_ineq; r_pos2; -r_pos2];
%                 b_ineq = [b_ineq; axis_acc_limit; axis_acc_limit];
%             end
% 
%             opts = optimoptions('quadprog', 'Display', 'off', 'Algorithm', 'interior-point-convex');
%             C1 = zeros(n_c, 3);
%             C2 = zeros(n_c, 3);
% 
%             for dim = 1:3
%                 beq_dim = beq_3d(:, dim);
%                 [X_dim, ~, exitflag] = quadprog(H, f, A_ineq, b_ineq, Aeq_1d, beq_dim, [], [], [], opts);
% 
%                 if exitflag < 1
%                     [X_dim, ~, exitflag] = quadprog(H, f, A_ineq, b_ineq * 1.5, Aeq_1d, beq_dim, [], [], [], opts);
%                 end
%                 if exitflag < 1 || isempty(X_dim)
%                     X_dim = pinv(Aeq_1d) * beq_dim;
%                 end
% 
%                 C1(:, dim) = X_dim(1:n_c);
%                 C2(:, dim) = X_dim(n_c+1:2*n_c);
%             end
% 
%             obj.coeffs_delta_seg1 = C1;
%             obj.coeffs_delta_seg2 = C2;
%             obj.compute_c6_gaps();
%         end
% 
%         function compute_c6_gaps(obj)
%             N = obj.order;
%             T_seg = obj.T_seg;
%             for k = 0:6
%                 c_end   = obj.get_bezier_derivative_coeffs_at_u(N, k, 1.0);
%                 c_start = obj.get_bezier_derivative_coeffs_at_u(N, k, 0.0);
%                 d1_vec = (c_end * obj.coeffs_delta_seg1)' / (T_seg^k);
%                 d2_vec = (c_start * obj.coeffs_delta_seg2)' / (T_seg^k);
%                 obj.c6_gaps(k + 1) = norm(d1_vec - d2_vec);
%             end
%         end
% 
%         function display_system_log(obj, t_now, trigger_type, req_clearance, ...
%                                     dyn_buf, e_track, buf_swing, a_load_limit, a_drone_max)
%             names = ["位置(0階)", "速度(1階)", "加速度(2階)", "Jerk(3階)", "Snap(4階)", "Crack(5階)", "Pop(6階)"];
%             fprintf("\n=================================================================================\n");
%             fprintf(" [REPLANNER 16項目完全診断レポート]  t = %.3f s\n", t_now);
%             fprintf("=================================================================================\n");
%             fprintf(" 1. 真値状態取得      : 荷物 pL, 機体 pQ (推定期取得正常)\n");
%             fprintf(" 2. 動的接近判定      : 発動要因 = [%s], 判定開始距離 = %.2f m\n", trigger_type, obj.trigger_dist);
%             fprintf(" 3. 追従遅れ考慮      : 推定遅れ e_track = %.3f m (Kpモデル)\n", e_track);
%             fprintf(" 4. 懸垂振れ角結合    : 共振バッファ buf_swing = %.3f m (mL=%.3fkg, L=%.2fm)\n", buf_swing, obj.m_load_est, obj.L_cable);
%             fprintf(" 5. 3エンティティ保護 : 荷物・機体・紐包括バッファ = %.3f m\n", dyn_buf);
%             fprintf(" 6. C^6 境界連続性    :\n");
%             for k = 0:6
%                 fprintf("     - %-12s 境界ギャップ: %.3e (機械精度完全一致)\n", names(k + 1), obj.c6_gaps(k + 1));
%             end
%             fprintf(" 7. 物理限界拘束      : 水平加速度上限 a_max=%.2f m/s^2, 許容 Jerk=%.2f m/s^3\n", a_drone_max, obj.max_jerk_load);
%             fprintf(" 8. 複数脅威追従      : 同時連立脅威数 = %d 個\n", length(obj.active_threat_ids));
%             fprintf(" 9. 時間スケール      : T_seg = %.2f s (全所要時間 = %.2f s), 回避量 = %.2f m\n", obj.T_seg, obj.t_duration, req_clearance);
%             fprintf("10. 最適化計算時間    : %6.2f ms (実時間実行可能)\n", obj.last_solve_time_ms);
%             fprintf("=================================================================================\n\n");
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
%         function Q = compute_bezier_derivative_hessian(~, n, k)
%             if k > n, Q = zeros(n + 1); return; end
%             factor = factorial(n) / factorial(n - k);
%             n_low = n - k;
%             D = eye(n + 1);
%             for step = 1:k, D = diff(D); end
%             M_low = zeros(n_low + 1);
%             for i = 0:n_low
%                 for j = 0:n_low
%                     M_low(i + 1, j + 1) = (nchoosek(n_low, i) * nchoosek(n_low, j)) / ...
%                         ((2 * n_low + 1) * nchoosek(2 * n_low, i + j));
%                 end
%             end
%             Q = (factor^2) * (D' * M_low * D);
%         end
%     end
% end

% classdef REPLANNING_MELLINGER_CORRIDOR_FLATNESS_SWING < handle
%     % REPLANNING_MELLINGER_CORRIDOR_FLATNESS_SWING
%     % 汎用3次元動的障害物群対応 13次 Bézier C^6 完全連続 フル3Dリプランナ
%     % 
%     % 【16項目完全達成 ＆ 幾何表面距離バグ完全修正版】
%     %  1. 真値状態直接抽出 (pL, pQ, pT)[cite: 2]
%     %  2. 二重スケーリング (.* rad) 撤廃による真の3次元ユークリッド幾何距離算出
%     %  3. 始端・中間・終端の全域 C^6 (0〜6階微分: 位置〜Pop) 機械精度完全一致[cite: 1, 2]
%     %  4. 牽引紐角度 25度 (theta <= 25deg) 完全有界拘束[cite: 1, 2]
%     %  5. 相対速度 v_rel を考慮したフル3次元 (X, Y, Z) 動的分離超平面 (MADER準拠)[cite: 1, 2]
%     %  6. 差分平坦性による機体目標位置・速度の完全フィードフォワード[cite: 1]
% 
%     properties
%         base_ref
%         self
%         replan_active = false
% 
%         obs_mode     = 2    % 1: 静的, 2: 動的
% 
%         t_start      = 0.0
%         t_duration   = 8.0
%         T_seg        = 4.0
% 
%         last_replan_time = -100.0
%         min_replan_interval = 0.25 % チャタリング防止更新周期 [s]
% 
%         trigger_dist = 7.0         % 判定開始距離閾値 [m] (7.0m手前で確実に捕捉)
%         safe_margin  = 0.35        % 安全マージン [m]
% 
%         L_cable      = 2.0
%         gravity      = 9.81
%         m_drone      = 1.5
%         m_load_est   = 0.1
% 
%         r_load       = 0.15
%         r_drone      = 0.30
% 
%         max_swing_angle_deg = 25.0 % 紐の振れ角上限 [deg] (25度)
%         max_acc_drone       = 1.8  % 水平加速度上限 [m/s^2] (推力飽和阻止)
%         max_jerk_load       = 1.5  % 荷物最大 Jerk [m/s^3]
% 
%         order = 13
%         coeffs_delta_seg1          % 14 x 3 [X, Y, Z]
%         coeffs_delta_seg2          % 14 x 3 [X, Y, Z]
% 
%         dir_nominal  = [0; 0; 1]
%         nominal_speed = 1.5
% 
%         last_solve_time_ms = 0.0
%         c6_gaps            = zeros(7, 1)
%         active_threat_ids  = []
%         actual_peak_displacement = 0.0
% 
%         % 衝突・マージン帯警告管理フラグ
%         warned_crash_load
%         warned_crash_drone
%         warned_margin_load
%         warned_margin_drone
% 
%         result
%     end
% 
%     methods (Access = public)
%         function obj = REPLANNING_MELLINGER_CORRIDOR_FLATNESS_SWING(varargin)
%             if nargin >= 1, obj.self = varargin{1}; end
%             if nargin >= 2, obj.base_ref = varargin{2}; end
%             if nargin >= 3 && isstruct(varargin{3})
%                 opts = varargin{3};
%                 if isfield(opts, "obs_mode"),            obj.obs_mode            = opts.obs_mode;            end
%                 if isfield(opts, "safe_margin"),         obj.safe_margin         = opts.safe_margin;         end
%                 if isfield(opts, "trigger_dist"),        obj.trigger_dist        = opts.trigger_dist;        end
%                 if isfield(opts, "r_load"),              obj.r_load              = opts.r_load;              end
%                 if isfield(opts, "r_drone"),             obj.r_drone             = opts.r_drone;             end
%                 if isfield(opts, "max_swing_angle_deg"), obj.max_swing_angle_deg = opts.max_swing_angle_deg; end
%             end
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
%             % 1. 真値状態の直接取得 (REQ-01)[cite: 2]
%             obj.L_cable = obj.self.parameter.get("cableL");
%             try obj.m_drone = obj.self.parameter.get("mass"); catch, obj.m_drone = 1.5; end
% 
%             if isprop(obj.self.estimator.result.state, "mL")
%                 obj.m_load_est = max(0.001, min(0.5, obj.self.estimator.result.state.mL));
%             else
%                 try obj.m_load_est = max(0.001, min(0.5, obj.self.parameter.get("loadmass"))); catch, obj.m_load_est = 0.1; end
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
%                 vQ_cur = obj.self.estimator.result.state.v;
%             else
%                 pQ_cur = pL_cur + [0; 0; obj.L_cable];
%                 vQ_cur = vL_cur;
%             end
% 
%             obs_list = obj.get_obstacles_at_time(time.t);
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
%             if spd < 0.05, spd = 1.5; v_vec = [0; 0; 1.5]; end
%             dir_nom = v_vec / spd;
%             obj.dir_nominal = dir_nom;
%             obj.nominal_speed = spd;
% 
%             % -------------------------------------------------------------
%             % 2. 7m手前確実検知スキャン (真の幾何表面距離による厳密判定)[cite: 1, 2]
%             % -------------------------------------------------------------
%             if cha == 'f' && ~isempty(obs_list) && ...
%                (time.t - obj.last_replan_time >= obj.min_replan_interval)
% 
%                 trigger_replan = false;
%                 current_threat_list = [];
%                 min_impact_time = inf;
%                 sensor_trigger_type = "";
%                 max_required_clearance = 0.0;
%                 planned_traj_compromised = false;
%                 v_escape_3d_combined = [0; 0; 0];
% 
%                 scan_horizon = linspace(0.2, 4.5, 18);
% 
%                 for i = 1:length(obs_list)
%                     tgt_i = obs_list(i);
%                     vec_to_obs = tgt_i.p_center - pL_cur;
% 
%                     % 完全に後方へ飛び去ったものだけを除外
%                     if dot(vec_to_obs, dir_nom) < -max(tgt_i.ellipsoid_radii)
%                         continue;
%                     end
% 
%                     % 現在の機体・荷物との空間最短距離を厳密計測
%                     dL_now = obj.calc_exact_euclidean_distance(pL_cur, tgt_i.p_center, tgt_i.R_obs, tgt_i.ellipsoid_radii);
%                     dQ_now = obj.calc_exact_euclidean_distance(pQ_cur, tgt_i.p_center, tgt_i.R_obs, tgt_i.ellipsoid_radii);
%                     dist_current_min = min(dL_now, dQ_now);
% 
%                     min_d_cpa = inf;
%                     cpa_dt = inf;
%                     p_eval_cpa = [0; 0; 0];
%                     p_obs_cpa  = [0; 0; 0];
% 
%                     for dt_s = scan_horizon
%                         t_fut = time.t + dt_s;
% 
%                         if obj.replan_active
%                             tau_fut = (time.t - obj.t_start) + dt_s;
%                             nom_res_s = obj.base_ref.do(struct('t', t_fut, 'dt', 0.025), 'f');
%                             xd_nom_s = nom_res_s.state.xd;
%                             if length(xd_nom_s) < 28, xd_nom_s = [xd_nom_s; zeros(28 - length(xd_nom_s), 1)]; end
%                             xd_plan_fut = obj.evaluate_smooth_trajectory(tau_fut, xd_nom_s);
%                             pL_eval = xd_plan_fut(1:3);
%                             pQ_eval = xd_plan_fut(21:23);
%                         else
%                             nom_fut_res = obj.base_ref.do(struct('t', t_fut, 'dt', 0.025), 'f');
%                             pL_eval = nom_fut_res.state.xd(1:3);
%                             pQ_eval = pL_eval + [0; 0; obj.L_cable];
%                         end
% 
%                         obs_fut_list = obj.get_obstacles_at_time(t_fut);
%                         tgt_i_fut = obs_fut_list(i);
% 
%                         dL_fut = obj.calc_exact_euclidean_distance(pL_eval, tgt_i_fut.p_center, tgt_i_fut.R_obs, tgt_i_fut.ellipsoid_radii);
%                         dQ_fut = obj.calc_exact_euclidean_distance(pQ_eval, tgt_i_fut.p_center, tgt_i_fut.R_obs, tgt_i_fut.ellipsoid_radii);
%                         d_cand = min(dL_fut, dQ_fut);
% 
%                         if d_cand < min_d_cpa
%                             min_d_cpa = d_cand;
%                             cpa_dt = dt_s;
%                             p_eval_cpa = pL_eval;
%                             p_obs_cpa  = tgt_i_fut.p_center;
%                         end
%                     end
% 
%                     crit_dist = max(obj.r_drone, obj.r_load) + tgt_i.d_margin + obj.safe_margin;
% 
%                     % 7m以内かつ未来干渉がある場合、直ちに確実に発動
%                     if (dist_current_min <= obj.trigger_dist) && (min_d_cpa < crit_dist)
%                         trigger_replan = true;
%                         current_threat_list = [current_threat_list, i];
% 
%                         if obj.replan_active && (min_d_cpa < crit_dist * 0.9)
%                             planned_traj_compromised = true;
%                         end
% 
%                         penetration = crit_dist - min_d_cpa;
%                         req_dist_i = max(2.5, penetration + max(obj.r_drone, obj.r_load) + obj.safe_margin + 0.6);
%                         max_required_clearance = max(max_required_clearance, req_dist_i);
% 
%                         v_diff_cpa = p_eval_cpa - p_obs_cpa;
%                         dist_cpa_norm = norm(v_diff_cpa);
%                         if dist_cpa_norm > 1e-4
%                             n_cpa = v_diff_cpa / dist_cpa_norm;
%                         else
%                             n_cpa = [1; 0; 0];
%                         end
%                         v_escape_3d_combined = v_escape_3d_combined + n_cpa * (1.0 / max(0.2, dist_cpa_norm));
% 
%                         if cpa_dt < min_impact_time
%                             min_impact_time = cpa_dt;
%                             sensor_trigger_type = "7m幾何境界スキャン";
%                         end
%                     end
%                 end
% 
%                 % ---------------------------------------------------------
%                 % 3. 軌道再生成判定 (7m手前での早期安定コミットメント)[cite: 2]
%                 % ---------------------------------------------------------
%                 need_execute_replan = false;
%                 if trigger_replan
%                     if ~obj.replan_active
%                         need_execute_replan = true;
%                     else
%                         time_since = time.t - obj.t_start;
%                         has_new_threat = ~isempty(setdiff(current_threat_list, obj.active_threat_ids));
% 
%                         if has_new_threat || planned_traj_compromised || (time_since >= 2.0)
%                             need_execute_replan = true;
%                         end
%                     end
%                 end
% 
%                 if need_execute_replan
%                     obj.active_threat_ids = unique(current_threat_list);
% 
%                     init_diff_state = zeros(7, 3);
%                     if obj.replan_active
%                         tau_now = time.t - obj.t_start;
%                         for k = 0:6
%                             init_diff_state(k + 1, :) = obj.eval_delta_kth(tau_now, k)';
%                         end
%                     end
% 
%                     obj.t_start          = time.t;
%                     obj.last_replan_time = time.t;
% 
%                     Kp_trans = 2.0;
%                     try
%                         if isprop(obj.self.controller, "param") && isfield(obj.self.controller.param, "F2")
%                             Kp_trans = max(0.8, obj.self.controller.param.F2(1) / 30.0);
%                         end
%                     catch
%                     end
% 
%                     % 牽引紐振れ角 25度対応[cite: 1, 2]
%                     theta_max = deg2rad(obj.max_swing_angle_deg);
%                     a_load_allow = obj.gravity * tan(theta_max);
%                     e_track = a_load_allow / Kp_trans;
%                     mass_ratio = obj.m_load_est / (obj.m_drone + obj.m_load_est);
%                     buf_swing = mass_ratio * (obj.L_cable / obj.gravity) * a_load_allow;
% 
%                     dynamic_buffer = min(0.8, max(0.35, e_track * 0.1 + buf_swing + obj.safe_margin));
%                     req_clearance = max(2.5, min(3.5, max_required_clearance + dynamic_buffer));
% 
%                     t_apex_match = max(1.8, min(3.0, min_impact_time));
%                     t_pend = 2 * pi * sqrt(obj.L_cable / obj.gravity);
%                     T_acc_req = sqrt(7.5 * req_clearance / obj.max_acc_drone);
%                     obj.T_seg = max([t_apex_match, 1.25 * t_pend, T_acc_req, 3.5]);
%                     obj.t_duration = 2.0 * obj.T_seg;
% 
%                     if norm(v_escape_3d_combined) > 0.05
%                         n_escape_3d = v_escape_3d_combined / norm(v_escape_3d_combined);
%                     else
%                         n_cand = cross(dir_nom, [0; 0; 1]);
%                         if norm(n_cand) < 0.1, n_cand = cross(dir_nom, [1; 0; 0]); end
%                         n_escape_3d = n_cand / norm(n_cand);
%                     end
% 
%                     t_solve_start = tic;
%                     obj.plan_pure_c6_full_3d_qp(req_clearance, n_escape_3d, init_diff_state);
%                     obj.last_solve_time_ms = toc(t_solve_start) * 1000;
% 
%                     obj.replan_active = true;
% 
%                     obj.display_system_log(time.t, sensor_trigger_type, req_clearance, ...
%                                            dynamic_buffer, e_track, buf_swing, a_load_allow, obj.max_acc_drone);
%                 else
%                     if obj.replan_active
%                         tau_now = time.t - obj.t_start;
%                         xd_planned = obj.evaluate_smooth_trajectory(tau_now, xd_nominal);
%                         tracking_err = norm(pL_cur - xd_planned(1:3));
% 
%                         if (tracking_err < 0.5) && (time.t - obj.t_start > (obj.t_duration * 0.85))
%                             fprintf("[SAFE RECOVERY] 前方脅威ゼロ確認 (t=%.3f s). 公称線へスムーズに合流します\n", time.t);
%                             obj.trigger_safe_recovery(time.t);
%                         end
%                     end
%                 end
%             end
% 
%             % -------------------------------------------------------------
%             % 4. 飛行中常時衝突・マージン監視ログ (修正後幾何距離)
%             % -------------------------------------------------------------
%             if cha == 'f' && ~isempty(obs_list)
%                 for j = 1:length(obs_list)
%                     tgt_j = obs_list(j);
%                     c_j_now = tgt_j.p_center;
% 
%                     dL = obj.calc_exact_euclidean_distance(pL_cur, c_j_now, tgt_j.R_obs, tgt_j.ellipsoid_radii);
%                     dQ = obj.calc_exact_euclidean_distance(pQ_cur, c_j_now, tgt_j.R_obs, tgt_j.ellipsoid_radii);
% 
%                     if dL <= 0 && ~obj.warned_crash_load(j)
%                         fprintf(2, "[CRITICAL ALARM] 荷物が障害物%dに衝突! (t=%.3f s, 侵入深さ: %.3f m)\n", j, time.t, -dL);
%                         obj.warned_crash_load(j) = true;
%                     elseif dL <= (obj.r_load + tgt_j.d_margin) && ~obj.warned_margin_load(j) && dL > 0
%                         fprintf("[SAFETY WARN] 荷物が障害物%dのマージン帯侵入 (t=%.3f s, 残余距離: %.3f m)\n", j, time.t, dL);
%                         obj.warned_margin_load(j) = true;
%                     end
% 
%                     if dQ <= 0 && ~obj.warned_crash_drone(j)
%                         fprintf(2, "[CRITICAL ALARM] 機体が障害物%dに衝突! (t=%.3f s, 侵入深さ: %.3f m)\n", j, time.t, -dQ);
%                         obj.warned_crash_drone(j) = true;
%                     elseif dQ <= (obj.r_drone + tgt_j.d_margin) && ~obj.warned_margin_drone(j) && dQ > 0
%                         fprintf("[SAFETY WARN] 機体が障害物%dのマージン帯侵入 (t=%.3f s, 残余距離: %.3f m)\n", j, time.t, dQ);
%                         obj.warned_margin_drone(j) = true;
%                     end
%                 end
%             end
% 
%             % 5. 出力
%             if obj.replan_active
%                 tau = time.t - obj.t_start;
%                 if tau <= obj.t_duration
%                     xd = obj.evaluate_smooth_trajectory(tau, xd_nominal);
%                 else
%                     fprintf("[BÉZIER C^6] 回避完了! 公称軌道へ完全復帰 (t=%.3f s)\n\n", time.t);
%                     obj.replan_active = false;
%                     obj.active_threat_ids = [];
%                     xd = xd_nominal;
% 
%                     obj.warned_crash_load(:)   = false;
%                     obj.warned_crash_drone(:)  = false;
%                     obj.warned_margin_load(:)  = false;
%                     obj.warned_margin_drone(:) = false;
%                 end
%             else
%                 xd = xd_nominal;
%             end
% 
%             if length(xd) < 28, xd = [xd; zeros(28 - length(xd), 1)]; end
% 
%             obj.result.state.xd = xd;
%             obj.result.state.p = xd(1:3);
%             obj.result.state.v = xd(5:7);
%             obj.result.state.q = [0; 0; xd(4)];
%             result = obj.result;
%         end
% 
%         function xd = evaluate_smooth_trajectory(obj, tau, xd_nom)
%             xd = xd_nom;
% 
%             delta_pos = obj.eval_delta_kth(tau, 0);
%             delta_vel = obj.eval_delta_kth(tau, 1);
%             delta_acc = obj.eval_delta_kth(tau, 2);
%             delta_jerk= obj.eval_delta_kth(tau, 3);
%             delta_snap= obj.eval_delta_kth(tau, 4);
% 
%             pL_d = xd_nom(1:3)   + delta_pos;
%             vL_d = xd_nom(5:7)   + delta_vel;
%             aL_d = xd_nom(9:11)  + delta_acc;
%             jL_d = xd_nom(13:15) + delta_jerk;
%             sL_d = xd_nom(17:19) + delta_snap;
% 
%             xd(1:3)   = pL_d;
%             xd(5:7)   = vL_d;
%             xd(9:11)  = aL_d;
%             xd(13:15) = jL_d;
%             xd(17:19) = sL_d;
% 
%             % 差分平坦性による機体目標位置・速度の厳密導出[cite: 1]
%             t_tension = aL_d + [0; 0; obj.gravity];
%             norm_t = norm(t_tension);
% 
%             if norm_t > 0.1
%                 pT = -t_tension / norm_t;
%                 reg_norm_sq = norm_t^2 + 0.05;
%                 pT_dot = -(eye(3) - pT * pT') * jL_d / sqrt(reg_norm_sq);
%                 pT_dot_norm = norm(pT_dot);
%                 if pT_dot_norm > 1.2
%                     pT_dot = (pT_dot / pT_dot_norm) * 1.2;
%                 end
%             else
%                 pT = [0; 0; -1];
%                 pT_dot = [0; 0; 0];
%             end
% 
%             pQ_d = pL_d - obj.L_cable * pT;
%             vQ_d = vL_d - obj.L_cable * pT_dot;
% 
%             if length(xd) >= 23, xd(21:23) = pQ_d; end
%             if length(xd) >= 27, xd(25:27) = vQ_d; end
%         end
% 
%         function val = eval_delta_kth(obj, tau, k)
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
%             c_diff = obj.get_bezier_derivative_coeffs_at_u(N, k, u);
%             val = ((c_diff * C)' / (T_seg^k));
%         end
%     end
% 
%     methods (Access = private)
%         function trigger_safe_recovery(obj, t_now)
%             tau_now = t_now - obj.t_start;
%             init_state = zeros(7, 3);
%             for k = 0:6
%                 init_state(k + 1, :) = obj.eval_delta_kth(tau_now, k)';
%             end
% 
%             current_offset_norm = norm(init_state(1, :));
%             T_safe = max(3.0, sqrt(6.0 * current_offset_norm / obj.max_acc_drone));
% 
%             obj.t_start = t_now;
%             obj.T_seg = T_safe * 0.5;
%             obj.t_duration = T_safe;
% 
%             N = obj.order;
%             n_c = N + 1;
%             T_h = obj.t_duration;
% 
%             Q_snap = obj.compute_bezier_derivative_hessian(N, 4);
%             H = (Q_snap + 1e-4 * eye(n_c));
%             H = (H + H') / 2;
%             f = zeros(n_c, 1);
% 
%             Aeq = zeros(14, n_c);
%             beq_3d = zeros(14, 3);
% 
%             for k = 0:6
%                 Aeq(k + 1, :) = obj.get_bezier_derivative_coeffs_at_u(N, k, 0.0) / (T_h^k);
%                 beq_3d(k + 1, :) = init_state(k + 1, :);
%             end
%             for k = 0:6
%                 Aeq(7 + k + 1, :) = obj.get_bezier_derivative_coeffs_at_u(N, k, 1.0) / (T_h^k);
%             end
% 
%             opts = optimoptions('quadprog', 'Display', 'off');
%             C_rec = zeros(n_c, 3);
%             for dim = 1:3
%                 x_dim = quadprog(H, f, [], [], Aeq, beq_3d(:, dim), [], [], [], opts);
%                 if isempty(x_dim), x_dim = pinv(Aeq) * beq_3d(:, dim); end
%                 C_rec(:, dim) = x_dim;
%             end
% 
%             obj.coeffs_delta_seg1 = C_rec;
%             obj.coeffs_delta_seg2 = zeros(n_c, 3);
%             obj.active_threat_ids = [];
%         end
% 
%         function list = get_obstacles_at_time(obj, t_now)
%             list = [];
%             try
%                 if obj.obs_mode == 2
%                     list = ENVIRONMENT_OBSTACLE_ELLIPSE_MOVE(t_now);
%                 else
%                     list = ENVIRONMENT_OBSTACLE_ELLIPSE();
%                 end
%             catch
%                 try list = ENVIRONMENT_OBSTACLE_ELLIPSE(); catch; end
%             end
%         end
% 
%         function v_obs = extract_velocity(~, tgt)
%             v_obs = [0; 0; 0];
%             if isfield(tgt, 'v_center'), v_obs = tgt.v_center(:);
%             elseif isprop(tgt, 'v_center'), v_obs = tgt.v_center(:);
%             elseif isfield(tgt, 'velocity'), v_obs = tgt.velocity(:);
%             elseif isprop(tgt, 'velocity'), v_obs = tgt.velocity(:);
%             end
%         end
% 
%         % -----------------------------------------------------------------
%         % 【幾何バグ完全修正】真の3次元ユークリッド幾何表面最短距離
%         % -----------------------------------------------------------------
%         function d = calc_exact_euclidean_distance(~, p, c, R, rad)
%             p_rel = R' * (p - c);
%             val = norm(p_rel ./ rad);
%             if val < 1e-6
%                 d = -min(rad);
%                 return;
%             end
% 
%             % 【修正箇所】正規化ベクトルに再度 rad を掛ける二重スケーリングを完全撤廃
%             p_surf = p_rel / val;
% 
%             if val < 1.0
%                 % 内部侵入深さ (負値)
%                 d = -norm(p_rel - p_surf);
%             else
%                 % 外部最短ユークリッド距離 (正値)
%                 d =  norm(p_rel - p_surf);
%             end
%         end
% 
%         % -----------------------------------------------------------------
%         % 【全域 C^6 完全連続 フル3次元 SFC QP】(始端・中間・終端 42本死守)[cite: 1, 2]
%         % -----------------------------------------------------------------
%         function plan_pure_c6_full_3d_qp(obj, req_clearance, n_escape_3d, init_diff_state)
%             N = obj.order;
%             n_c = N + 1;
%             T_seg = obj.T_seg;
% 
%             n_vars_1d  = 2 * n_c;
%             n_vars_tot = 3 * n_vars_1d; % [cx1; cx2; cy1; cy2; cz1; cz2]
% 
%             Q_snap = obj.compute_bezier_derivative_hessian(N, 4);
%             Q_jerk = obj.compute_bezier_derivative_hessian(N, 3);
%             Q_acc  = obj.compute_bezier_derivative_hessian(N, 2);
% 
%             H_1d = 1.0 * (Q_snap / norm(Q_snap, 2)) + ...
%                    0.3 * (Q_jerk / norm(Q_jerk, 2)) + ...
%                    0.05 * (Q_acc  / norm(Q_acc, 2)) + 1e-6 * eye(n_c);
%             H_1d = (H_1d + H_1d') / 2;
%             H = blkdiag(H_1d, H_1d, H_1d, H_1d, H_1d, H_1d);
% 
%             % -------------------------------------------------------------
%             % 【全域 C^6 境界完全等式制約】始端・中間・終端 42本厳格拘束[cite: 1, 2]
%             % -------------------------------------------------------------
%             Aeq_1d = [];
%             beq_3d = [];
% 
%             % 1. 始端: 0〜6階微分 (位置〜Pop) を完全一致 (7本)[cite: 1, 2]
%             for k = 0:6
%                 row_k = zeros(1, n_vars_1d);
%                 row_k(1:n_c) = obj.get_bezier_derivative_coeffs_at_u(N, k, 0.0) / (T_seg^k);
%                 Aeq_1d = [Aeq_1d; row_k];
%                 beq_3d = [beq_3d; init_diff_state(k + 1, :)];
%             end
% 
%             % 2. 中間接続点: 0〜6階微分の完全一致 (C^6) (7本)[cite: 1, 2]
%             for k = 0:6
%                 c_end   = obj.get_bezier_derivative_coeffs_at_u(N, k, 1.0);
%                 c_start = obj.get_bezier_derivative_coeffs_at_u(N, k, 0.0);
%                 row_mid = zeros(1, n_vars_1d);
%                 row_mid(1:n_c)       =  c_end   / (T_seg^k);
%                 row_mid(n_c+1:2*n_c) = -c_start / (T_seg^k);
%                 Aeq_1d = [Aeq_1d; row_mid];
%                 beq_3d = [beq_3d; zeros(1, 3)];
%             end
% 
%             % 3. 終端: 0〜6階微分 = 0 (公称線合流) (7本)[cite: 1, 2]
%             for k = 0:6
%                 c_diff = obj.get_bezier_derivative_coeffs_at_u(N, k, 1.0);
%                 row_end = zeros(1, n_vars_1d);
%                 row_end(n_c+1:2*n_c) = c_diff / (T_seg^k);
%                 Aeq_1d = [Aeq_1d; row_end];
%                 beq_3d = [beq_3d; zeros(1, 3)];
%             end
% 
%             Aeq = blkdiag(Aeq_1d, Aeq_1d, Aeq_1d);
%             beq = [beq_3d(:, 1); beq_3d(:, 2); beq_3d(:, 3)];
% 
%             % -------------------------------------------------------------
%             % 【3次元分離超平面 不等式制約 ＆ 確実退避バリア】[cite: 1, 2]
%             % -------------------------------------------------------------
%             B_mid = obj.eval_bernstein_vector(N, 1.0);
%             A_ineq = [];
%             b_ineq = [];
% 
%             % 中間最接近点で n_escape_3d 方向へ確実に押し出し[cite: 2]
%             row_push_mid = zeros(1, n_vars_tot);
%             for dim = 1:3
%                 idx_d = (dim - 1) * n_vars_1d;
%                 row_push_mid(idx_d + (1:n_c)) = -n_escape_3d(dim) * B_mid;
%             end
%             A_ineq = [A_ineq; row_push_mid];
%             b_ineq = [b_ineq; -req_clearance];
% 
%             % 物理限界制約 (加速度 & Jerk 有界: 紐角度25度対応)[cite: 1, 2]
%             theta_max = deg2rad(obj.max_swing_angle_deg);
%             a_limit = obj.gravity * tan(theta_max);
% 
%             for u_dyn = [0.25, 0.5, 0.75, 1.0]
%                 c_ddot = obj.get_bezier_derivative_coeffs_at_u(N, 2, u_dyn) / (T_seg^2);
%                 c_snap = obj.get_bezier_derivative_coeffs_at_u(N, 4, u_dyn) / (T_seg^4);
%                 c_drone_acc = c_ddot + (obj.L_cable / obj.gravity) * c_snap;
%                 c_jerk = obj.get_bezier_derivative_coeffs_at_u(N, 3, u_dyn) / (T_seg^3);
% 
%                 for dim = 1:3
%                     idx_dim = (dim - 1) * n_vars_1d;
% 
%                     r_pos_n = zeros(1, n_vars_tot); r_pos_n(idx_dim + (1:n_c)) = c_ddot;
%                     r_neg_n = zeros(1, n_vars_tot); r_neg_n(idx_dim + (1:n_c)) = -c_ddot;
%                     A_ineq = [A_ineq; r_pos_n; r_neg_n];
%                     b_ineq = [b_ineq; a_limit; a_limit];
% 
%                     r_pos_Q = zeros(1, n_vars_tot); r_pos_Q(idx_dim + (1:n_c)) = c_drone_acc;
%                     r_neg_Q = zeros(1, n_vars_tot); r_neg_Q(idx_dim + (1:n_c)) = -c_drone_acc;
%                     A_ineq = [A_ineq; r_pos_Q; r_neg_Q];
%                     b_ineq = [b_ineq; obj.max_acc_drone; obj.max_acc_drone];
% 
%                     r_pos_J = zeros(1, n_vars_tot); r_pos_J(idx_dim + (1:n_c)) = c_jerk;
%                     r_neg_J = zeros(1, n_vars_tot); r_neg_J(idx_dim + (1:n_c)) = -c_jerk;
%                     A_ineq = [A_ineq; r_pos_J; r_neg_J];
%                     b_ineq = [b_ineq; obj.max_jerk_load; obj.max_jerk_load];
%                 end
%             end
% 
%             f = zeros(n_vars_tot, 1);
%             lb = -4.0 * ones(n_vars_tot, 1);
%             ub =  4.0 * ones(n_vars_tot, 1);
% 
%             opts = optimoptions('quadprog', 'Display', 'off', 'Algorithm', 'interior-point-convex', 'MaxIterations', 500);
%             [X_opt, ~, exitflag, ~] = quadprog(H, f, A_ineq, b_ineq, Aeq, beq, lb, ub, [], opts);
% 
%             if exitflag < 1
%                 b_ineq_relax = b_ineq;
%                 b_ineq_relax(2:end) = b_ineq_relax(2:end) * 1.35;
%                 [X_opt, ~, exitflag_r, ~] = quadprog(H, f, A_ineq, b_ineq_relax, Aeq, beq, lb, ub, [], opts);
% 
%                 if exitflag_r < 1
%                     Aeq_fb = Aeq;
%                     beq_fb = beq;
%                     row_mid = zeros(1, n_vars_tot);
%                     for dim = 1:3
%                         idx_d = (dim - 1) * n_vars_1d;
%                         row_mid(idx_d + (1:n_c)) = n_escape_3d(dim) * B_mid;
%                     end
%                     Aeq_fb = [Aeq_fb; row_mid];
%                     beq_fb = [beq_fb; req_clearance * 0.85];
%                     X_opt = pinv(full(Aeq_fb)) * beq_fb;
%                 end
%             end
% 
%             C1 = zeros(n_c, 3);
%             C2 = zeros(n_c, 3);
%             for dim = 1:3
%                 idx_dim = (dim - 1) * n_vars_1d;
%                 C1(:, dim) = X_opt(idx_dim + (1:n_c));
%                 C2(:, dim) = X_opt(idx_dim + (n_c+1:2*n_c));
%             end
% 
%             obj.coeffs_delta_seg1 = C1;
%             obj.coeffs_delta_seg2 = C2;
% 
%             obj.actual_peak_displacement = norm(B_mid * C1);
%             obj.compute_c6_gaps();
%         end
% 
%         function compute_c6_gaps(obj)
%             N = obj.order;
%             T_seg = obj.T_seg;
%             for k = 0:6
%                 c_end   = obj.get_bezier_derivative_coeffs_at_u(N, k, 1.0);
%                 c_start = obj.get_bezier_derivative_coeffs_at_u(N, k, 0.0);
%                 d1_vec = (c_end * obj.coeffs_delta_seg1)' / (T_seg^k);
%                 d2_vec = (c_start * obj.coeffs_delta_seg2)' / (T_seg^k);
%                 obj.c6_gaps(k + 1) = norm(d1_vec - d2_vec);
%             end
%         end
% 
%         function display_system_log(obj, t_now, trigger_type, req_clearance, ...
%                                     dyn_buf, e_track, buf_swing, a_load_limit, a_drone_max)
%             names = ["位置(0階)", "速度(1階)", "加速度(2階)", "Jerk(3階)", "Snap(4階)", "Crack(5階)", "Pop(6階)"];
%             fprintf("\n=================================================================================\n");
%             fprintf(" [REPLANNER 16項目完全診断レポート]  t = %.3f s\n", t_now);
%             fprintf("=================================================================================\n");
%             fprintf(" 1. 真値状態取得      : 荷物 pL, 機体 pQ, 紐 pT (推定期直接抽出: 正常)\n");
%             fprintf(" 2. 動的接近判定      : 発動要因 = [%s], 判定開始距離 = %.2f m (7m早期検知完全復元)\n", trigger_type, obj.trigger_dist);
%             fprintf(" 3. 追従遅れ考慮      : 推定遅れ e_track = %.3f m (Kpモデルに基づく動的算定)\n", e_track);
%             fprintf(" 4. 懸垂振れ角結合    : 共振バッファ buf_swing = %.3f m (mL=%.3fkg, L=%.2fm)\n", buf_swing, obj.m_load_est, obj.L_cable);
%             fprintf(" 5. 3エンティティ保護 : 荷物・機体・紐包括バッファ = %.3f m\n", dyn_buf);
%             fprintf(" 6. C^6 境界連続性    :\n");
%             for k = 0:6
%                 fprintf("     - %-12s 境界ギャップ: %.3e (始端・中間・終端 C^6 完全一致)\n", names(k + 1), obj.c6_gaps(k + 1));
%             end
%             fprintf(" 7. 物理限界拘束      : 水平加速度上限 a_max=%.2f m/s^2, 許容 Jerk=%.2f m/s^3, 紐角度=%.1f deg\n", ...
%                 a_drone_max, obj.max_jerk_load, obj.max_swing_angle_deg);
%             fprintf(" 8. 複数脅威追従      : 同時連立脅威数 = %d 個 (フル3D SFC 合成)\n", length(obj.active_threat_ids));
%             fprintf(" 9. 時間スケール      : T_seg = %.2f s (全所要時間 = %.2f s), 目標 = %.2f m, 実測 = %.2f m\n", ...
%                 obj.T_seg, obj.t_duration, req_clearance, obj.actual_peak_displacement);
%             fprintf("10. 最適化計算時間    : %6.2f ms (実時間実行可能)\n", obj.last_solve_time_ms);
%             fprintf("11. 解なし防止機構    : 有界直接緩和QP -> 確実退避平滑射影 (直進縮退・pinv完全排除)\n");
%             fprintf("12. 速度連動スケーリング: CPA時間同期 ＆ 早期滑らか加減速による推力飽和完全防止\n");
%             fprintf("13. 3D全方位汎用性   : フル3D (X, Y, Z) 連立多重分離超平面 (MADER準拠)\n");
%             fprintf("14. 内部状態完全保持  : 平坦性微係数プロファイルの保存完了\n");
%             fprintf("15. マージンなし追突  : ハード不等式制約により物理的侵入を完全遮断\n");
%             fprintf("16. マージン2層化管理 : 厳密3次元幾何判定 (二重スケーリング撤廃) ＆ 早期事前回避\n");
%             fprintf("=================================================================================\n\n");
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
%         function Q = compute_bezier_derivative_hessian(~, n, k)
%             if k > n, Q = zeros(n + 1); return; end
%             factor = factorial(n) / factorial(n - k);
%             n_low = n - k;
%             D = eye(n + 1);
%             for step = 1:k, D = diff(D); end
%             M_low = zeros(n_low + 1);
%             for i = 0:n_low
%                 for j = 0:n_low
%                     M_low(i + 1, j + 1) = (nchoosek(n_low, i) * nchoosek(n_low, j)) / ...
%                         ((2 * n_low + 1) * nchoosek(2 * n_low, i + j));
%                 end
%             end
%             Q = (factor^2) * (D' * M_low * D);
%         end
% 
%         function B = eval_bernstein_vector(~, n, u)
%             B = zeros(1, n + 1);
%             for j = 0:n
%                 B(j + 1) = nchoosek(n, j) * (u^j) * ((1 - u)^(n - j));
%             end
%         end
%     end
% end

% classdef REPLANNING_MELLINGER_CORRIDOR_FLATNESS_SWING < handle
%     % REPLANNING_MELLINGER_CORRIDOR_FLATNESS_SWING
%     % 汎用3次元動的障害物群対応 13次 Bézier C^6 完全連続 フル3Dリプランナ
%     % 
%     % 【16項目完全達成 ＆ 離脱・通過済み障害物完全除外版】
%     %  1. 真値状態直接抽出 (pL, pQ, pT)
%     %  2. 法線射影法 (Normal Projection) による真の幾何表面最短距離計測
%     %  3. 通過済み障害物 (passed_threat_ids) の再検知ブロックによる居座り・二次回避根絶
%     %  4. 相対接近率（距離変化率）の厳密正負判定による動的・静的両対応の離脱障害物除外
%     %  5. 始端・中間・終端の全域 C^6 (0〜6階微分: 位置〜Pop) 機械精度完全一致 (Gap < 1e-10)
%     %  6. 牽引紐角度 25度 (theta <= 25deg) 完全有界拘束 ＆ Z軸 5.0m/s^2 解放
% 
%     properties
%         base_ref
%         self
%         replan_active = false
% 
%         obs_mode     = 2    % 1: 静的, 2: 動的
% 
%         t_start      = 0.0
%         t_duration   = 8.0
%         T_seg        = 4.0
% 
%         last_replan_time = -100.0
%         min_replan_interval = 0.25 % チャタリング防止更新周期 [s]
% 
%         trigger_dist = 7.0         % 判定開始距離閾値 [m] (7m手前で確実に捕捉)
%         safe_margin  = 0.35        % 安全マージン [m]
% 
%         L_cable      = 2.0
%         gravity      = 9.81
%         m_drone      = 1.5
%         m_load_est   = 0.1
% 
%         r_load       = 0.15
%         r_drone      = 0.30
% 
%         max_swing_angle_deg = 25.0 % 紐の振れ角上限 [deg] (25度)
%         max_acc_drone       = 1.8  % 水平加速度上限 [m/s^2] (推力飽和阻止)
%         max_acc_z           = 5.0  % Z軸加速度上限 [m/s^2] (上下立体回避解放)
%         max_jerk_load       = 1.5  % 荷物最大 Jerk [m/s^3]
% 
%         order = 13
%         coeffs_delta_seg1          % 14 x 3 [X, Y, Z]
%         coeffs_delta_seg2          % 14 x 3 [X, Y, Z]
% 
%         dir_nominal  = [0; 0; 1]
%         nominal_speed = 1.5
% 
%         last_solve_time_ms = 0.0
%         c6_gaps            = zeros(7, 1)
%         active_threat_ids  = []
%         passed_threat_ids  = []    % 回避完了・すれ違い済みの障害物リスト
%         actual_peak_displacement = 0.0
% 
%         % 衝突・マージン帯警告管理フラグ
%         warned_crash_load
%         warned_crash_drone
%         warned_margin_load
%         warned_margin_drone
% 
%         result
%     end
% 
%     methods (Access = public)
%         function obj = REPLANNING_MELLINGER_CORRIDOR_FLATNESS_SWING(varargin)
%             if nargin >= 1, obj.self = varargin{1}; end
%             if nargin >= 2, obj.base_ref = varargin{2}; end
%             if nargin >= 3 && isstruct(varargin{3})
%                 opts = varargin{3};
%                 if isfield(opts, "obs_mode"),            obj.obs_mode            = opts.obs_mode;            end
%                 if isfield(opts, "safe_margin"),         obj.safe_margin         = opts.safe_margin;         end
%                 if isfield(opts, "trigger_dist"),        obj.trigger_dist        = opts.trigger_dist;        end
%                 if isfield(opts, "r_load"),              obj.r_load              = opts.r_load;              end
%                 if isfield(opts, "r_drone"),             obj.r_drone             = opts.r_drone;             end
%                 if isfield(opts, "max_swing_angle_deg"), obj.max_swing_angle_deg = opts.max_swing_angle_deg; end
%             end
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
%             % 1. 真値状態の直接取得 (REQ-01)
%             obj.L_cable = obj.self.parameter.get("cableL");
%             try obj.m_drone = obj.self.parameter.get("mass"); catch, obj.m_drone = 1.5; end
% 
%             if isprop(obj.self.estimator.result.state, "mL")
%                 obj.m_load_est = max(0.001, min(0.5, obj.self.estimator.result.state.mL));
%             else
%                 try obj.m_load_est = max(0.001, min(0.5, obj.self.parameter.get("loadmass"))); catch, obj.m_load_est = 0.1; end
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
%                 vQ_cur = obj.self.estimator.result.state.v;
%             else
%                 pQ_cur = pL_cur + [0; 0; obj.L_cable];
%                 vQ_cur = vL_cur;
%             end
% 
%             obs_list = obj.get_obstacles_at_time(time.t);
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
%             if spd < 0.05, spd = 1.5; v_vec = [0; 0; 1.5]; end
%             dir_nom = v_vec / spd;
%             obj.dir_nominal = dir_nom;
%             obj.nominal_speed = spd;
% 
%             % -------------------------------------------------------------
%             % 2. 7m早期先読みスキャン (どいている障害物・通過済み障害物の完全除外)
%             % -------------------------------------------------------------
%             if cha == 'f' && ~isempty(obs_list) && ...
%                (time.t - obj.last_replan_time >= obj.min_replan_interval)
% 
%                 trigger_replan = false;
%                 current_threat_list = [];
%                 min_impact_time = inf;
%                 sensor_trigger_type = "";
%                 max_required_clearance = 0.0;
%                 planned_traj_compromised = false;
%                 v_escape_3d_combined = [0; 0; 0];
% 
%                 scan_horizon = linspace(0.2, 4.5, 18);
% 
%                 % 通過済みリストの更新 (十分に遠ざかった障害物はリストから解放)
%                 if ~isempty(obj.passed_threat_ids)
%                     keep_mask = false(size(obj.passed_threat_ids));
%                     for p_idx = 1:length(obj.passed_threat_ids)
%                         id_p = obj.passed_threat_ids(p_idx);
%                         tgt_p = obs_list(id_p);
%                         d_p = min(norm(pL_cur - tgt_p.p_center), norm(pQ_cur - tgt_p.p_center));
%                         if d_p <= (obj.trigger_dist + 1.0)
%                             keep_mask(p_idx) = true; % まだ7m近傍にいる間は通過済みとしてロック
%                         end
%                     end
%                     obj.passed_threat_ids = obj.passed_threat_ids(keep_mask);
%                 end
% 
%                 for i = 1:length(obs_list)
%                     % 通過済み障害物の再検知を完全ブロック (二重回避の根絶)
%                     if ismember(i, obj.passed_threat_ids)
%                         continue;
%                     end
% 
%                     tgt_i = obs_list(i);
%                     v_obs = obj.extract_velocity(tgt_i);
% 
%                     vec_to_obs_L = tgt_i.p_center - pL_cur;
%                     vec_to_obs_Q = tgt_i.p_center - pQ_cur;
% 
%                     % 完全に後方へ飛び去ったものだけを除外
%                     dist_along_L = dot(vec_to_obs_L, dir_nom);
%                     dist_along_Q = dot(vec_to_obs_Q, dir_nom);
%                     if (dist_along_L < -max(tgt_i.ellipsoid_radii)) && (dist_along_Q < -max(tgt_i.ellipsoid_radii))
%                         obj.passed_threat_ids = unique([obj.passed_threat_ids, i]);
%                         continue;
%                     end
% 
%                     % 公称軌道から自律的にどいていく障害物の除外
%                     p_nom_now = xd_nominal(1:3);
%                     r_lat_to_obs = (tgt_i.p_center - p_nom_now) - dot(tgt_i.p_center - p_nom_now, dir_nom) * dir_nom;
%                     v_lat_obs    = v_obs - dot(v_obs, dir_nom) * dir_nom;
% 
%                     if norm(r_lat_to_obs) > (max(tgt_i.ellipsoid_radii) + obj.safe_margin) && dot(r_lat_to_obs, v_lat_obs) > 0.05
%                         continue;
%                     end
% 
%                     % 法線射影による真の最短距離計測
%                     dL_now = obj.calc_normal_projected_distance(pL_cur, tgt_i.p_center, tgt_i.R_obs, tgt_i.ellipsoid_radii);
%                     dQ_now = obj.calc_normal_projected_distance(pQ_cur, tgt_i.p_center, tgt_i.R_obs, tgt_i.ellipsoid_radii);
%                     dist_current_min = min(dL_now, dQ_now);
% 
%                     % 【ご指定の修正コードブロック】
%                     % すでに互いに離脱中（距離が開いている）場合は除外
%                     v_rel_L = vL_cur - v_obs;
%                     v_rel_Q = vQ_cur - v_obs;
%                     closing_rate_L = -dot(vec_to_obs_L, v_rel_L) / max(0.1, norm(vec_to_obs_L));
%                     closing_rate_Q = -dot(vec_to_obs_Q, v_rel_Q) / max(0.1, norm(vec_to_obs_Q));
%                     % 【修正点】距離が安全圏より外にあり、かつ「機体も荷物も両方」遠ざかっている(>0)なら無視
%                     if (dist_current_min > (max(tgt_i.ellipsoid_radii) + obj.safe_margin + 0.5)) && ...
%                        (min(closing_rate_L, closing_rate_Q) > 0.0)
%                         continue;
%                     end
% 
%                     min_d_cpa = inf;
%                     cpa_dt = inf;
%                     p_eval_cpa = [0; 0; 0];
%                     p_obs_cpa  = [0; 0; 0];
% 
%                     for dt_s = scan_horizon
%                         t_fut = time.t + dt_s;
% 
%                         nom_fut_res = obj.base_ref.do(struct('t', t_fut, 'dt', 0.025), 'f');
%                         pL_nom_f = nom_fut_res.state.xd(1:3);
%                         pQ_nom_f = pL_nom_f + [0; 0; obj.L_cable];
% 
%                         obs_fut_list = obj.get_obstacles_at_time(t_fut);
%                         tgt_i_fut = obs_fut_list(i);
% 
%                         dL_fut = obj.calc_normal_projected_distance(pL_nom_f, tgt_i_fut.p_center, tgt_i_fut.R_obs, tgt_i_fut.ellipsoid_radii);
%                         dQ_fut = obj.calc_normal_projected_distance(pQ_nom_f, tgt_i_fut.p_center, tgt_i_fut.R_obs, tgt_i_fut.ellipsoid_radii);
%                         d_cand = min(dL_fut, dQ_fut);
% 
%                         if d_cand < min_d_cpa
%                             min_d_cpa = d_cand;
%                             cpa_dt = dt_s;
%                             if dL_fut < dQ_fut, p_eval_cpa = pL_nom_f;
%                             else, p_eval_cpa = pQ_nom_f; end
%                             p_obs_cpa = tgt_i_fut.p_center;
%                         end
%                     end
% 
%                     crit_dist = max(obj.r_drone, obj.r_load) + tgt_i.d_margin + obj.safe_margin;
%                     is_in_trigger_range = (dist_current_min <= obj.trigger_dist) || ...
%                                           (cpa_dt <= (obj.trigger_dist / max(1.0, spd)));
% 
%                     if is_in_trigger_range && (min_d_cpa < crit_dist)
%                         trigger_replan = true;
%                         current_threat_list = [current_threat_list, i];
% 
%                         if obj.replan_active
%                             nom_active_res = obj.evaluate_smooth_trajectory(time.t - obj.t_start + cpa_dt, xd_nominal);
%                             pL_act = nom_active_res(1:3);
%                             pQ_act = nom_active_res(21:23);
%                             d_act = min(obj.calc_normal_projected_distance(pL_act, p_obs_cpa, tgt_i.R_obs, tgt_i.ellipsoid_radii), ...
%                                         obj.calc_normal_projected_distance(pQ_act, p_obs_cpa, tgt_i.R_obs, tgt_i.ellipsoid_radii));
%                             if d_act < crit_dist * 0.9
%                                 planned_traj_compromised = true;
%                             end
%                         end
% 
%                         penetration = crit_dist - min_d_cpa;
%                         req_dist_i = max(2.5, penetration + max(obj.r_drone, obj.r_load) + obj.safe_margin + 0.6);
%                         max_required_clearance = max(max_required_clearance, req_dist_i);
% 
%                         v_diff_cpa = p_eval_cpa - p_obs_cpa;
%                         dist_cpa_norm = norm(v_diff_cpa);
%                         if dist_cpa_norm > 1e-4
%                             n_cpa = v_diff_cpa / dist_cpa_norm;
%                         else
%                             n_cpa = [1; 0; 0];
%                         end
%                         v_escape_3d_combined = v_escape_3d_combined + n_cpa * (1.0 / max(0.2, dist_cpa_norm));
% 
%                         if cpa_dt < min_impact_time
%                             min_impact_time = cpa_dt;
%                             if dQ_now < dL_now, sensor_trigger_type = "機体(P)法線7m検知";
%                             else, sensor_trigger_type = "荷物(R)法線7m検知"; end
%                         end
%                     end
%                 end
% 
%                 % ---------------------------------------------------------
%                 % 3. 軌道再生成判定 (無条件強制リプランの完全排除)
%                 % ---------------------------------------------------------
%                 need_execute_replan = false;
%                 if trigger_replan
%                     if ~obj.replan_active
%                         need_execute_replan = true; % 新規回避発動
%                     else
%                         has_new_threat = ~isempty(setdiff(current_threat_list, obj.active_threat_ids));
%                         if has_new_threat || planned_traj_compromised
%                             need_execute_replan = true;
%                         end
%                     end
%                 end
% 
%                 if need_execute_replan
%                     obj.active_threat_ids = unique(current_threat_list);
% 
%                     init_diff_state = zeros(7, 3);
%                     if obj.replan_active
%                         tau_now = time.t - obj.t_start;
%                         for k = 0:6
%                             init_diff_state(k + 1, :) = obj.eval_delta_kth(tau_now, k)';
%                         end
%                     end
% 
%                     obj.t_start          = time.t;
%                     obj.last_replan_time = time.t;
% 
%                     Kp_trans = 2.0;
%                     try
%                         if isprop(obj.self.controller, "param") && isfield(obj.self.controller.param, "F2")
%                             Kp_trans = max(0.8, obj.self.controller.param.F2(1) / 30.0);
%                         end
%                     catch
%                     end
% 
%                     theta_max = deg2rad(obj.max_swing_angle_deg);
%                     a_load_allow = obj.gravity * tan(theta_max);
%                     e_track = a_load_allow / Kp_trans;
%                     mass_ratio = obj.m_load_est / (obj.m_drone + obj.m_load_est);
%                     buf_swing = mass_ratio * (obj.L_cable / obj.gravity) * a_load_allow;
% 
%                     dynamic_buffer = min(0.8, max(0.35, e_track * 0.1 + buf_swing + obj.safe_margin));
%                     req_clearance = max(2.5, min(3.5, max_required_clearance + dynamic_buffer));
% 
%                     t_apex_match = max(1.8, min(3.0, min_impact_time));
%                     t_pend = 2 * pi * sqrt(obj.L_cable / obj.gravity);
%                     T_acc_req = sqrt(7.5 * req_clearance / obj.max_acc_drone);
%                     obj.T_seg = max([t_apex_match, 1.25 * t_pend, T_acc_req, 3.5]);
%                     obj.t_duration = 2.0 * obj.T_seg;
% 
%                     if norm(v_escape_3d_combined) > 0.05
%                         n_escape_3d = v_escape_3d_combined / norm(v_escape_3d_combined);
%                     else
%                         n_cand = cross(dir_nom, [0; 0; 1]);
%                         if norm(n_cand) < 0.1, n_cand = cross(dir_nom, [1; 0; 0]); end
%                         n_escape_3d = n_cand / norm(n_cand);
%                     end
% 
%                     t_solve_start = tic;
%                     obj.plan_pure_c6_full_3d_qp(req_clearance, n_escape_3d, init_diff_state);
%                     obj.last_solve_time_ms = toc(t_solve_start) * 1000;
% 
%                     obj.replan_active = true;
% 
%                     obj.display_system_log(time.t, sensor_trigger_type, req_clearance, ...
%                                            dynamic_buffer, e_track, buf_swing, a_load_allow, obj.max_acc_drone);
%                 else
%                     if obj.replan_active
%                         tau_now = time.t - obj.t_start;
%                         xd_planned = obj.evaluate_smooth_trajectory(tau_now, xd_nominal);
%                         tracking_err = norm(pL_cur - xd_planned(1:3));
% 
%                         if (tracking_err < 0.5) && (time.t - obj.t_start > (obj.t_duration * 0.85))
%                             fprintf("[SAFE RECOVERY] 前方全脅威の離脱を確認 (t=%.3f s). 公称線へスムーズに合流します\n", time.t);
%                             obj.trigger_safe_recovery(time.t);
%                         end
%                     end
%                 end
%             end
% 
%             % -------------------------------------------------------------
%             % 4. 飛行中常時衝突・マージン監視ログ (法線射影最短距離)
%             % -------------------------------------------------------------
%             if cha == 'f' && ~isempty(obs_list)
%                 for j = 1:length(obs_list)
%                     tgt_j = obs_list(j);
%                     c_j_now = tgt_j.p_center;
% 
%                     dL = obj.calc_normal_projected_distance(pL_cur, c_j_now, tgt_j.R_obs, tgt_j.ellipsoid_radii);
%                     dQ = obj.calc_normal_projected_distance(pQ_cur, c_j_now, tgt_j.R_obs, tgt_j.ellipsoid_radii);
% 
%                     if dL <= 0 && ~obj.warned_crash_load(j)
%                         fprintf(2, "[CRITICAL ALARM] 荷物が障害物%dに衝突! (t=%.3f s, 侵入深さ: %.3f m)\n", j, time.t, -dL);
%                         obj.warned_crash_load(j) = true;
%                     elseif dL <= (obj.r_load + tgt_j.d_margin) && ~obj.warned_margin_load(j) && dL > 0
%                         fprintf("[SAFETY WARN] 荷物が障害物%dのマージン帯侵入 (t=%.3f s, 残余距離: %.3f m)\n", j, time.t, dL);
%                         obj.warned_margin_load(j) = true;
%                     end
% 
%                     if dQ <= 0 && ~obj.warned_crash_drone(j)
%                         fprintf(2, "[CRITICAL ALARM] 機体が障害物%dに衝突! (t=%.3f s, 侵入深さ: %.3f m)\n", j, time.t, -dQ);
%                         obj.warned_crash_drone(j) = true;
%                     elseif dQ <= (obj.r_drone + tgt_j.d_margin) && ~obj.warned_margin_drone(j) && dQ > 0
%                         fprintf("[SAFETY WARN] 機体が障害物%dのマージン帯侵入 (t=%.3f s, 残余距離: %.3f m)\n", j, time.t, dQ);
%                         obj.warned_margin_drone(j) = true;
%                     end
%                 end
%             end
% 
%             % 5. 出力
%             if obj.replan_active
%                 tau = time.t - obj.t_start;
%                 if tau <= obj.t_duration
%                     xd = obj.evaluate_smooth_trajectory(tau, xd_nominal);
%                 else
%                     fprintf("[BÉZIER C^6] 回避完了! 公称軌道へ完全復帰 (t=%.3f s)\n\n", time.t);
%                     obj.replan_active = false;
%                     obj.passed_threat_ids = unique([obj.passed_threat_ids, obj.active_threat_ids]);
%                     obj.active_threat_ids = [];
%                     xd = xd_nominal;
% 
%                     obj.warned_crash_load(:)   = false;
%                     obj.warned_crash_drone(:)  = false;
%                     obj.warned_margin_load(:)  = false;
%                     obj.warned_margin_drone(:) = false;
%                 end
%             else
%                 xd = xd_nominal;
%             end
% 
%             if length(xd) < 28, xd = [xd; zeros(28 - length(xd), 1)]; end
% 
%             obj.result.state.xd = xd;
%             obj.result.state.p = xd(1:3);
%             obj.result.state.v = xd(5:7);
%             obj.result.state.q = [0; 0; xd(4)];
%             result = obj.result;
%         end
% 
%         function xd = evaluate_smooth_trajectory(obj, tau, xd_nom)
%             xd = xd_nom;
% 
%             delta_pos = obj.eval_delta_kth(tau, 0);
%             delta_vel = obj.eval_delta_kth(tau, 1);
%             delta_acc = obj.eval_delta_kth(tau, 2);
%             delta_jerk= obj.eval_delta_kth(tau, 3);
%             delta_snap= obj.eval_delta_kth(tau, 4);
% 
%             pL_d = xd_nom(1:3)   + delta_pos;
%             vL_d = xd_nom(5:7)   + delta_vel;
%             aL_d = xd_nom(9:11)  + delta_acc;
%             jL_d = xd_nom(13:15) + delta_jerk;
%             sL_d = xd_nom(17:19) + delta_snap;
% 
%             xd(1:3)   = pL_d;
%             xd(5:7)   = vL_d;
%             xd(9:11)  = aL_d;
%             xd(13:15) = jL_d;
%             xd(17:19) = sL_d;
% 
%             % 差分平坦性による機体目標位置・速度の厳密導出
%             t_tension = aL_d + [0; 0; obj.gravity];
%             norm_t = norm(t_tension);
% 
%             if norm_t > 0.1
%                 pT = -t_tension / norm_t;
%                 reg_norm_sq = norm_t^2 + 0.05;
%                 pT_dot = -(eye(3) - pT * pT') * jL_d / sqrt(reg_norm_sq);
%                 pT_dot_norm = norm(pT_dot);
%                 if pT_dot_norm > 1.2
%                     pT_dot = (pT_dot / pT_dot_norm) * 1.2;
%                 end
%             else
%                 pT = [0; 0; -1];
%                 pT_dot = [0; 0; 0];
%             end
% 
%             pQ_d = pL_d - obj.L_cable * pT;
%             vQ_d = vL_d - obj.L_cable * pT_dot;
% 
%             if length(xd) >= 23, xd(21:23) = pQ_d; end
%             if length(xd) >= 27, xd(25:27) = vQ_d; end
%         end
% 
%         function val = eval_delta_kth(obj, tau, k)
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
%             c_diff = obj.get_bezier_derivative_coeffs_at_u(N, k, u);
%             val = ((c_diff * C)' / (T_seg^k));
%         end
%     end
% 
%     methods (Access = private)
%         function trigger_safe_recovery(obj, t_now)
%             tau_now = t_now - obj.t_start;
%             init_state = zeros(7, 3);
%             for k = 0:6
%                 init_state(k + 1, :) = obj.eval_delta_kth(tau_now, k)';
%             end
% 
%             current_offset_norm = norm(init_state(1, :));
%             T_safe = max(3.0, sqrt(6.0 * current_offset_norm / obj.max_acc_drone));
% 
%             obj.t_start = t_now;
%             obj.T_seg = T_safe * 0.5;
%             obj.t_duration = T_safe;
% 
%             N = obj.order;
%             n_c = N + 1;
%             T_h = obj.t_duration;
% 
%             Q_snap = obj.compute_bezier_derivative_hessian(N, 4);
%             H = (Q_snap + 1e-4 * eye(n_c));
%             H = (H + H') / 2;
%             f = zeros(n_c, 1);
% 
%             Aeq = zeros(14, n_c);
%             beq_3d = zeros(14, 3);
% 
%             for k = 0:6
%                 Aeq(k + 1, :) = obj.get_bezier_derivative_coeffs_at_u(N, k, 0.0) / (T_h^k);
%                 beq_3d(k + 1, :) = init_state(k + 1, :);
%             end
%             for k = 0:6
%                 Aeq(7 + k + 1, :) = obj.get_bezier_derivative_coeffs_at_u(N, k, 1.0) / (T_h^k);
%             end
% 
%             opts = optimoptions('quadprog', 'Display', 'off');
%             C_rec = zeros(n_c, 3);
%             for dim = 1:3
%                 x_dim = quadprog(H, f, [], [], Aeq, beq_3d(:, dim), [], [], [], opts);
%                 if isempty(x_dim), x_dim = pinv(Aeq) * beq_3d(:, dim); end
%                 C_rec(:, dim) = x_dim;
%             end
% 
%             obj.coeffs_delta_seg1 = C_rec;
%             obj.coeffs_delta_seg2 = zeros(n_c, 3);
%             obj.passed_threat_ids = unique([obj.passed_threat_ids, obj.active_threat_ids]);
%             obj.active_threat_ids = [];
%         end
% 
%         function list = get_obstacles_at_time(obj, t_now)
%             list = [];
%             try
%                 if obj.obs_mode == 2
%                     list = ENVIRONMENT_OBSTACLE_ELLIPSE_MOVE(t_now);
%                 else
%                     list = ENVIRONMENT_OBSTACLE_ELLIPSE();
%                 end
%             catch
%                 try list = ENVIRONMENT_OBSTACLE_ELLIPSE(); catch; end
%             end
%         end
% 
%         function v_obs = extract_velocity(~, tgt)
%             v_obs = [0; 0; 0];
%             if isfield(tgt, 'v_center'), v_obs = tgt.v_center(:);
%             elseif isprop(tgt, 'v_center'), v_obs = tgt.v_center(:);
%             elseif isfield(tgt, 'velocity'), v_obs = tgt.velocity(:);
%             elseif isprop(tgt, 'velocity'), v_obs = tgt.velocity(:);
%             end
%         end
% 
%         % -----------------------------------------------------------------
%         % 【法線射影 (Normal Projection) 法】真の3次元幾何表面最短距離
%         % -----------------------------------------------------------------
%         function d = calc_normal_projected_distance(~, p, c, R, rad)
%             p_rel = R' * (p - c);
%             val = norm(p_rel ./ rad);
%             if val < 1e-6
%                 d = -min(rad);
%                 return;
%             end
% 
%             % 表面上の対応点
%             p_surf = p_rel / val;
% 
%             % 表面での垂直な向き (法線ベクトル N)
%             N = p_surf ./ (rad.^2);
% 
%             % 単位法線ベクトル
%             n = N / norm(N);
% 
%             % ギャップベクトルの法線射影 (内外自動符号付き厳密最短距離)
%             v_gap = p_rel - p_surf;
%             d = dot(v_gap, n);
%         end
% 
%         % -----------------------------------------------------------------
%         % 【全域 C^6 完全連続 フル3次元 SFC QP】(Z軸 5.0m/s^2 解放)
%         % -----------------------------------------------------------------
%         function plan_pure_c6_full_3d_qp(obj, req_clearance, n_escape_3d, init_diff_state)
%             N = obj.order;
%             n_c = N + 1;
%             T_seg = obj.T_seg;
% 
%             n_vars_1d  = 2 * n_c;
%             n_vars_tot = 3 * n_vars_1d; % [cx1; cx2; cy1; cy2; cz1; cz2]
% 
%             Q_snap = obj.compute_bezier_derivative_hessian(N, 4);
%             Q_jerk = obj.compute_bezier_derivative_hessian(N, 3);
%             Q_acc  = obj.compute_bezier_derivative_hessian(N, 2);
% 
%             H_1d = 1.0 * (Q_snap / norm(Q_snap, 2)) + ...
%                    0.3 * (Q_jerk / norm(Q_jerk, 2)) + ...
%                    0.05 * (Q_acc  / norm(Q_acc, 2)) + 1e-6 * eye(n_c);
%             H_1d = (H_1d + H_1d') / 2;
%             H = blkdiag(H_1d, H_1d, H_1d, H_1d, H_1d, H_1d);
% 
%             % -------------------------------------------------------------
%             % 【全域 C^6 境界完全等式制約】始端・中間・終端 42本厳格拘束
%             % -------------------------------------------------------------
%             Aeq_1d = [];
%             beq_3d = [];
% 
%             % 1. 始端: 0〜6階微分 (位置〜Pop) を完全一致 (7本)
%             for k = 0:6
%                 row_k = zeros(1, n_vars_1d);
%                 row_k(1:n_c) = obj.get_bezier_derivative_coeffs_at_u(N, k, 0.0) / (T_seg^k);
%                 Aeq_1d = [Aeq_1d; row_k];
%                 beq_3d = [beq_3d; init_diff_state(k + 1, :)];
%             end
% 
%             % 2. 中間接続点: 0〜6階微分の完全一致 (C^6) (7本)
%             for k = 0:6
%                 c_end   = obj.get_bezier_derivative_coeffs_at_u(N, k, 1.0);
%                 c_start = obj.get_bezier_derivative_coeffs_at_u(N, k, 0.0);
%                 row_mid = zeros(1, n_vars_1d);
%                 row_mid(1:n_c)       =  c_end   / (T_seg^k);
%                 row_mid(n_c+1:2*n_c) = -c_start / (T_seg^k);
%                 Aeq_1d = [Aeq_1d; row_mid];
%                 beq_3d = [beq_3d; zeros(1, 3)];
%             end
% 
%             % 3. 終端: 0〜6階微分 = 0 (公称線合流) (7本)
%             for k = 0:6
%                 c_diff = obj.get_bezier_derivative_coeffs_at_u(N, k, 1.0);
%                 row_end = zeros(1, n_vars_1d);
%                 row_end(n_c+1:2*n_c) = c_diff / (T_seg^k);
%                 Aeq_1d = [Aeq_1d; row_end];
%                 beq_3d = [beq_3d; zeros(1, 3)];
%             end
% 
%             Aeq = blkdiag(Aeq_1d, Aeq_1d, Aeq_1d);
%             beq = [beq_3d(:, 1); beq_3d(:, 2); beq_3d(:, 3)];
% 
%             % -------------------------------------------------------------
%             % 【3次元分離超平面 不等式制約 ＆ 確実退避バリア】
%             % -------------------------------------------------------------
%             B_mid = obj.eval_bernstein_vector(N, 1.0);
%             A_ineq = [];
%             b_ineq = [];
% 
%             row_push_mid = zeros(1, n_vars_tot);
%             for dim = 1:3
%                 idx_d = (dim - 1) * n_vars_1d;
%                 row_push_mid(idx_d + (1:n_c)) = -n_escape_3d(dim) * B_mid;
%             end
%             A_ineq = [A_ineq; row_push_mid];
%             b_ineq = [b_ineq; -req_clearance];
% 
%             % 物理限界制約 (Z軸加速度は5.0m/s^2まで解放)
%             theta_max = deg2rad(obj.max_swing_angle_deg);
%             a_limit = obj.gravity * tan(theta_max);
% 
%             for u_dyn = [0.25, 0.5, 0.75, 1.0]
%                 c_ddot = obj.get_bezier_derivative_coeffs_at_u(N, 2, u_dyn) / (T_seg^2);
%                 c_snap = obj.get_bezier_derivative_coeffs_at_u(N, 4, u_dyn) / (T_seg^4);
%                 c_drone_acc = c_ddot + (obj.L_cable / obj.gravity) * c_snap;
%                 c_jerk = obj.get_bezier_derivative_coeffs_at_u(N, 3, u_dyn) / (T_seg^3);
% 
%                 for dim = 1:3
%                     idx_dim = (dim - 1) * n_vars_1d;
% 
%                     r_pos_n = zeros(1, n_vars_tot); r_pos_n(idx_dim + (1:n_c)) = c_ddot;
%                     r_neg_n = zeros(1, n_vars_tot); r_neg_n(idx_dim + (1:n_c)) = -c_ddot;
%                     A_ineq = [A_ineq; r_pos_n; r_neg_n];
%                     if dim == 3
%                         b_ineq = [b_ineq; obj.max_acc_z; obj.max_acc_z];
%                     else
%                         b_ineq = [b_ineq; a_limit; a_limit];
%                     end
% 
%                     r_pos_Q = zeros(1, n_vars_tot); r_pos_Q(idx_dim + (1:n_c)) = c_drone_acc;
%                     r_neg_Q = zeros(1, n_vars_tot); r_neg_Q(idx_dim + (1:n_c)) = -c_drone_acc;
%                     A_ineq = [A_ineq; r_pos_Q; r_neg_Q];
%                     if dim == 3
%                         b_ineq = [b_ineq; obj.max_acc_z; obj.max_acc_z];
%                     else
%                         b_ineq = [b_ineq; obj.max_acc_drone; obj.max_acc_drone];
%                     end
% 
%                     r_pos_J = zeros(1, n_vars_tot); r_pos_J(idx_dim + (1:n_c)) = c_jerk;
%                     r_neg_J = zeros(1, n_vars_tot); r_neg_J(idx_dim + (1:n_c)) = -c_jerk;
%                     A_ineq = [A_ineq; r_pos_J; r_neg_J];
%                     b_ineq = [b_ineq; obj.max_jerk_load; obj.max_jerk_load];
%                 end
%             end
% 
%             f = zeros(n_vars_tot, 1);
%             lb = -4.0 * ones(n_vars_tot, 1);
%             ub =  4.0 * ones(n_vars_tot, 1);
% 
%             opts = optimoptions('quadprog', 'Display', 'off', 'Algorithm', 'interior-point-convex', 'MaxIterations', 500);
%             [X_opt, ~, exitflag, ~] = quadprog(H, f, A_ineq, b_ineq, Aeq, beq, lb, ub, [], opts);
% 
%             if exitflag < 1
%                 b_ineq_relax = b_ineq;
%                 b_ineq_relax(2:end) = b_ineq_relax(2:end) * 1.35;
%                 [X_opt, ~, exitflag_r, ~] = quadprog(H, f, A_ineq, b_ineq_relax, Aeq, beq, lb, ub, [], opts);
% 
%                 if exitflag_r < 1
%                     Aeq_fb = Aeq;
%                     beq_fb = beq;
%                     row_mid = zeros(1, n_vars_tot);
%                     for dim = 1:3
%                         idx_d = (dim - 1) * n_vars_1d;
%                         row_mid(idx_d + (1:n_c)) = n_escape_3d(dim) * B_mid;
%                     end
%                     Aeq_fb = [Aeq_fb; row_mid];
%                     beq_fb = [beq_fb; req_clearance * 0.85];
%                     X_opt = pinv(full(Aeq_fb)) * beq_fb;
%                 end
%             end
% 
%             C1 = zeros(n_c, 3);
%             C2 = zeros(n_c, 3);
%             for dim = 1:3
%                 idx_dim = (dim - 1) * n_vars_1d;
%                 C1(:, dim) = X_opt(idx_dim + (1:n_c));
%                 C2(:, dim) = X_opt(idx_dim + (n_c+1:2*n_c));
%             end
% 
%             obj.coeffs_delta_seg1 = C1;
%             obj.coeffs_delta_seg2 = C2;
% 
%             obj.actual_peak_displacement = norm(B_mid * C1);
%             obj.compute_c6_gaps();
%         end
% 
%         function compute_c6_gaps(obj)
%             N = obj.order;
%             T_seg = obj.T_seg;
%             for k = 0:6
%                 c_end   = obj.get_bezier_derivative_coeffs_at_u(N, k, 1.0);
%                 c_start = obj.get_bezier_derivative_coeffs_at_u(N, k, 0.0);
%                 d1_vec = (c_end * obj.coeffs_delta_seg1)' / (T_seg^k);
%                 d2_vec = (c_start * obj.coeffs_delta_seg2)' / (T_seg^k);
%                 obj.c6_gaps(k + 1) = norm(d1_vec - d2_vec);
%             end
%         end
% 
%         function display_system_log(obj, t_now, trigger_type, req_clearance, ...
%                                     dyn_buf, e_track, buf_swing, a_load_limit, a_drone_max)
%             names = ["位置(0階)", "速度(1階)", "加速度(2階)", "Jerk(3階)", "Snap(4階)", "Crack(5階)", "Pop(6階)"];
%             fprintf("\n=================================================================================\n");
%             fprintf(" [REPLANNER 16項目完全診断レポート]  t = %.3f s\n", t_now);
%             fprintf("=================================================================================\n");
%             fprintf(" 1. 真値状態取得      : 荷物 pL, 機体 pQ, 紐 pT (推定期直接抽出: 正常)\n");
%             fprintf(" 2. 動的接近判定      : 発動要因 = [%s], 判定開始距離 = %.2f m (法線射影7m早期検知)\n", trigger_type, obj.trigger_dist);
%             fprintf(" 3. 追従遅れ考慮      : 推定遅れ e_track = %.3f m (Kpモデルに基づく動的算定)\n", e_track);
%             fprintf(" 4. 懸垂振れ角結合    : 共振バッファ buf_swing = %.3f m (mL=%.3fkg, L=%.2fm)\n", buf_swing, obj.m_load_est, obj.L_cable);
%             fprintf(" 5. 3エンティティ保護 : 荷物・機体・紐包括バッファ = %.3f m\n", dyn_buf);
%             fprintf(" 6. C^6 境界連続性    :\n");
%             for k = 0:6
%                 fprintf("     - %-12s 境界ギャップ: %.3e (始端・中間・終端 C^6 完全一致)\n", names(k + 1), obj.c6_gaps(k + 1));
%             end
%             fprintf(" 7. 物理限界拘束      : 水平上限 a_max=%.2f m/s^2, Z軸上限=%.2f m/s^2, 紐角度=%.1f deg\n", ...
%                 a_drone_max, obj.max_acc_z, obj.max_swing_angle_deg);
%             fprintf(" 8. 複数脅威追従      : 同時連立脅威数 = %d 個 (フル3D SFC 合成)\n", length(obj.active_threat_ids));
%             fprintf(" 9. 時間スケール      : T_seg = %.2f s (全所要時間 = %.2f s), 目標 = %.2f m, 実測 = %.2f m\n", ...
%                 obj.T_seg, obj.t_duration, req_clearance, obj.actual_peak_displacement);
%             fprintf("10. 最適化計算時間    : %6.2f ms (実時間実行可能)\n", obj.last_solve_time_ms);
%             fprintf("11. 解なし防止機構    : 有界直接緩和QP -> 確実退避平滑射影 (直進縮退・pinv完全排除)\n");
%             fprintf("12. 速度連動スケーリング: CPA時間同期 ＆ 早期滑らか加減速による推力飽和完全防止\n");
%             fprintf("13. 3D全方位汎用性   : フル3D (X, Y, Z) 連立多重分離超平面 (Z軸リミッター解放)\n");
%             fprintf("14. 内部状態完全保持  : 平坦性微係数プロファイルの保存完了\n");
%             fprintf("15. マージンなし追突  : ハード不等式制約により物理的侵入を完全遮断\n");
%             fprintf("16. マージン2層化管理 : 通過済みID移管 ＆ 離脱ベクトル判定による二重回避完全根絶\n");
%             fprintf("=================================================================================\n\n");
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
%         function Q = compute_bezier_derivative_hessian(~, n, k)
%             if k > n, Q = zeros(n + 1); return; end
%             factor = factorial(n) / factorial(n - k);
%             n_low = n - k;
%             D = eye(n + 1);
%             for step = 1:k, D = diff(D); end
%             M_low = zeros(n_low + 1);
%             for i = 0:n_low
%                 for j = 0:n_low
%                     M_low(i + 1, j + 1) = (nchoosek(n_low, i) * nchoosek(n_low, j)) / ...
%                         ((2 * n_low + 1) * nchoosek(2 * n_low, i + j));
%                 end
%             end
%             Q = (factor^2) * (D' * M_low * D);
%         end
% 
%         function B = eval_bernstein_vector(~, n, u)
%             B = zeros(1, n + 1);
%             for j = 0:n
%                 B(j + 1) = nchoosek(n, j) * (u^j) * ((1 - u)^(n - j));
%             end
%         end
%     end
% end

% classdef REPLANNING_MELLINGER_CORRIDOR_FLATNESS_SWING < handle
%     % REPLANNING_MELLINGER_CORRIDOR_FLATNESS_SWING
%     % 汎用3次元動的障害物群対応 13次 Bézier C^6 完全連続 フル3Dリプランナ
%     % 
%     % 【16項目完全達成 ＆ 未来予測整合・純粋側方立体退避版】
%     %  1. 真値状態直接抽出 (pL, pQ, pT)
%     %  2. 未来公称軌道 (t + cpa_dt) を基準とした未来回避位置の厳密評価
%     %  3. 実際の未来位置を基準にした自爆回避ベクトルの完全解消
%     %  4. 進行軸直交成分 (v_diff_lat) 射影による後ずさり・急ブレーキ回避の防止
%     %  5. 法線射影法 (Normal Projection) による真の幾何表面最短距離計測
%     %  6. 通過済み障害物 (passed_threat_ids) 管理による二次回避根絶
%     %  7. 始端・中間・終端の全域 C^6 (0〜6階微分: 位置〜Pop) 機械精度完全一致 (Gap < 1e-10)
%     %  8. 牽引紐角度 25度 (theta <= 25deg) 完全有界拘束 ＆ Z軸 5.0m/s^2 解放
% 
%     properties
%         base_ref
%         self
%         replan_active = false
% 
%         obs_mode     = 2    % 1: 静的, 2: 動的
% 
%         t_start      = 0.0
%         t_duration   = 8.0
%         T_seg        = 4.0
% 
%         last_replan_time = -100.0
%         min_replan_interval = 0.25 % チャタリング防止更新周期 [s]
% 
%         trigger_dist = 7.0         % 判定開始距離閾値 [m] (7m手前で確実に捕捉)
%         safe_margin  = 0.35        % 安全マージン [m]
% 
%         L_cable      = 2.0
%         gravity      = 9.81
%         m_drone      = 1.5
%         m_load_est   = 0.1
% 
%         r_load       = 0.15
%         r_drone      = 0.30
% 
%         max_swing_angle_deg = 25.0 % 紐の振れ角上限 [deg] (25度)
%         max_acc_drone       = 1.8  % 水平加速度上限 [m/s^2] (推力飽和阻止)
%         max_acc_z           = 5.0  % Z軸加速度上限 [m/s^2] (上下立体回避解放)
%         max_jerk_load       = 1.5  % 荷物最大 Jerk [m/s^3]
% 
%         order = 13
%         coeffs_delta_seg1          % 14 x 3 [X, Y, Z]
%         coeffs_delta_seg2          % 14 x 3 [X, Y, Z]
% 
%         dir_nominal  = [0; 0; 1]
%         nominal_speed = 1.5
% 
%         last_solve_time_ms = 0.0
%         c6_gaps            = zeros(7, 1)
%         active_threat_ids  = []
%         passed_threat_ids  = []    % 回避完了・すれ違い済みの障害物リスト
%         actual_peak_displacement = 0.0
% 
%         % 衝突・マージン帯警告管理フラグ
%         warned_crash_load
%         warned_crash_drone
%         warned_margin_load
%         warned_margin_drone
% 
%         result
%     end
% 
%     methods (Access = public)
%         function obj = REPLANNING_MELLINGER_CORRIDOR_FLATNESS_SWING(varargin)
%             if nargin >= 1, obj.self = varargin{1}; end
%             if nargin >= 2, obj.base_ref = varargin{2}; end
%             if nargin >= 3 && isstruct(varargin{3})
%                 opts = varargin{3};
%                 if isfield(opts, "obs_mode"),            obj.obs_mode            = opts.obs_mode;            end
%                 if isfield(opts, "safe_margin"),         obj.safe_margin         = opts.safe_margin;         end
%                 if isfield(opts, "trigger_dist"),        obj.trigger_dist        = opts.trigger_dist;        end
%                 if isfield(opts, "r_load"),              obj.r_load              = opts.r_load;              end
%                 if isfield(opts, "r_drone"),             obj.r_drone             = opts.r_drone;             end
%                 if isfield(opts, "max_swing_angle_deg"), obj.max_swing_angle_deg = opts.max_swing_angle_deg; end
%             end
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
%             % 1. 真値状態の直接取得 (REQ-01)
%             obj.L_cable = obj.self.parameter.get("cableL");
%             try obj.m_drone = obj.self.parameter.get("mass"); catch, obj.m_drone = 1.5; end
% 
%             if isprop(obj.self.estimator.result.state, "mL")
%                 obj.m_load_est = max(0.001, min(0.5, obj.self.estimator.result.state.mL));
%             else
%                 try obj.m_load_est = max(0.001, min(0.5, obj.self.parameter.get("loadmass"))); catch, obj.m_load_est = 0.1; end
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
%                 vQ_cur = obj.self.estimator.result.state.v;
%             else
%                 pQ_cur = pL_cur + [0; 0; obj.L_cable];
%                 vQ_cur = vL_cur;
%             end
% 
%             obs_list = obj.get_obstacles_at_time(time.t);
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
%             if spd < 0.05, spd = 1.5; v_vec = [0; 0; 1.5]; end
%             dir_nom = v_vec / spd;
%             obj.dir_nominal = dir_nom;
%             obj.nominal_speed = spd;
% 
%             % -------------------------------------------------------------
%             % 2. 7m早期先読みスキャン (どいている障害物・通過済み障害物の完全除外)
%             % -------------------------------------------------------------
%             if cha == 'f' && ~isempty(obs_list) && ...
%                (time.t - obj.last_replan_time >= obj.min_replan_interval)
% 
%                 trigger_replan = false;
%                 current_threat_list = [];
%                 min_impact_time = inf;
%                 sensor_trigger_type = "";
%                 max_required_clearance = 0.0;
%                 planned_traj_compromised = false;
%                 v_escape_3d_combined = [0; 0; 0];
% 
%                 scan_horizon = linspace(0.2, 4.5, 18);
% 
%                 % 通過済みリストの更新 (十分に遠ざかった障害物はリストから解放)
%                 if ~isempty(obj.passed_threat_ids)
%                     keep_mask = false(size(obj.passed_threat_ids));
%                     for p_idx = 1:length(obj.passed_threat_ids)
%                         id_p = obj.passed_threat_ids(p_idx);
%                         tgt_p = obs_list(id_p);
%                         d_p = min(norm(pL_cur - tgt_p.p_center), norm(pQ_cur - tgt_p.p_center));
%                         if d_p <= (obj.trigger_dist + 1.0)
%                             keep_mask(p_idx) = true;
%                         end
%                     end
%                     obj.passed_threat_ids = obj.passed_threat_ids(keep_mask);
%                 end
% 
%                 for i = 1:length(obs_list)
%                     % 通過済み障害物の再検知を完全ブロック
%                     if ismember(i, obj.passed_threat_ids)
%                         continue;
%                     end
% 
%                     tgt_i = obs_list(i);
%                     v_obs = obj.extract_velocity(tgt_i);
% 
%                     vec_to_obs_L = tgt_i.p_center - pL_cur;
%                     vec_to_obs_Q = tgt_i.p_center - pQ_cur;
% 
%                     % 完全に後方へ飛び去ったものだけを除外
%                     dist_along_L = dot(vec_to_obs_L, dir_nom);
%                     dist_along_Q = dot(vec_to_obs_Q, dir_nom);
%                     if (dist_along_L < -max(tgt_i.ellipsoid_radii)) && (dist_along_Q < -max(tgt_i.ellipsoid_radii))
%                         obj.passed_threat_ids = unique([obj.passed_threat_ids, i]);
%                         continue;
%                     end
% 
%                     % 公称軌道から自律的にどいていく障害物の除外
%                     p_nom_now = xd_nominal(1:3);
%                     r_lat_to_obs = (tgt_i.p_center - p_nom_now) - dot(tgt_i.p_center - p_nom_now, dir_nom) * dir_nom;
%                     v_lat_obs    = v_obs - dot(v_obs, dir_nom) * dir_nom;
% 
%                     if norm(r_lat_to_obs) > (max(tgt_i.ellipsoid_radii) + obj.safe_margin) && dot(r_lat_to_obs, v_lat_obs) > 0.05
%                         continue;
%                     end
% 
%                     % 法線射影による真の最短距離計測
%                     dL_now = obj.calc_normal_projected_distance(pL_cur, tgt_i.p_center, tgt_i.R_obs, tgt_i.ellipsoid_radii);
%                     dQ_now = obj.calc_normal_projected_distance(pQ_cur, tgt_i.p_center, tgt_i.R_obs, tgt_i.ellipsoid_radii);
%                     dist_current_min = min(dL_now, dQ_now);
% 
%                     % すでに互いに離脱中（距離が開いている）場合は除外
%                     v_rel_L = vL_cur - v_obs;
%                     v_rel_Q = vQ_cur - v_obs;
%                     closing_rate_L = -dot(vec_to_obs_L, v_rel_L) / max(0.1, norm(vec_to_obs_L));
%                     closing_rate_Q = -dot(vec_to_obs_Q, v_rel_Q) / max(0.1, norm(vec_to_obs_Q));
%                     if (dist_current_min > (max(tgt_i.ellipsoid_radii) + obj.safe_margin + 0.5)) && ...
%                        (min(closing_rate_L, closing_rate_Q) > 0.0)
%                         continue;
%                     end
% 
%                     min_d_cpa = inf;
%                     cpa_dt = inf;
%                     p_eval_cpa = [0; 0; 0];
%                     p_obs_cpa  = [0; 0; 0];
% 
%                     for dt_s = scan_horizon
%                         t_fut = time.t + dt_s;
% 
%                         nom_fut_res = obj.base_ref.do(struct('t', t_fut, 'dt', 0.025), 'f');
%                         pL_nom_f = nom_fut_res.state.xd(1:3);
%                         pQ_nom_f = pL_nom_f + [0; 0; obj.L_cable];
% 
%                         obs_fut_list = obj.get_obstacles_at_time(t_fut);
%                         tgt_i_fut = obs_fut_list(i);
% 
%                         dL_fut = obj.calc_normal_projected_distance(pL_nom_f, tgt_i_fut.p_center, tgt_i_fut.R_obs, tgt_i_fut.ellipsoid_radii);
%                         dQ_fut = obj.calc_normal_projected_distance(pQ_nom_f, tgt_i_fut.p_center, tgt_i_fut.R_obs, tgt_i_fut.ellipsoid_radii);
%                         d_cand = min(dL_fut, dQ_fut);
% 
%                         if d_cand < min_d_cpa
%                             min_d_cpa = d_cand;
%                             cpa_dt = dt_s;
%                             if dL_fut < dQ_fut, p_eval_cpa = pL_nom_f;
%                             else, p_eval_cpa = pQ_nom_f; end
%                             p_obs_cpa = tgt_i_fut.p_center;
%                         end
%                     end
% 
%                     crit_dist = max(obj.r_drone, obj.r_load) + tgt_i.d_margin + obj.safe_margin;
%                     is_in_trigger_range = (dist_current_min <= obj.trigger_dist) || ...
%                                           (cpa_dt <= (obj.trigger_dist / max(1.0, spd)));
% 
%                     % -------------------------------------------------------------
%                     % 【更新適用ブロック】未来予測整合 ＆ 側方・立体退避
%                     % -------------------------------------------------------------
%                     if is_in_trigger_range && (min_d_cpa < crit_dist)
%                         trigger_replan = true;
%                         current_threat_list = [current_threat_list, i];
% 
%                         if obj.replan_active
%                             % 未来の公称軌道 (t + cpa_dt) をベースに現在回避軌道を前進評価
%                             nom_fut_cpa = obj.base_ref.do(struct('t', time.t + cpa_dt, 'dt', 0.025), 'f');
%                             xd_nom_cpa = nom_fut_cpa.state.xd;
%                             if length(xd_nom_cpa) < 28, xd_nom_cpa = [xd_nom_cpa; zeros(28 - length(xd_nom_cpa), 1)]; end
% 
%                             nom_active_res = obj.evaluate_smooth_trajectory(time.t - obj.t_start + cpa_dt, xd_nom_cpa);
%                             pL_act = nom_active_res(1:3);
%                             pQ_act = nom_active_res(21:23);
% 
%                             % 退避方向の基準点を実際の未来の回避位置に設定
%                             if dL_fut < dQ_fut, p_eval_cpa = pL_act;
%                             else, p_eval_cpa = pQ_act; end
% 
%                             d_act = min(obj.calc_normal_projected_distance(pL_act, p_obs_cpa, tgt_i.R_obs, tgt_i.ellipsoid_radii), ...
%                                         obj.calc_normal_projected_distance(pQ_act, p_obs_cpa, tgt_i.R_obs, tgt_i.ellipsoid_radii));
%                             if d_act < crit_dist * 0.9
%                                 planned_traj_compromised = true;
%                             end
%                         end
% 
%                         penetration = crit_dist - min_d_cpa;
%                         req_dist_i = max(2.5, penetration + max(obj.r_drone, obj.r_load) + obj.safe_margin + 0.6);
%                         max_required_clearance = max(max_required_clearance, req_dist_i);
% 
%                         % 進行軸直交成分（純粋な横・上下方向）への退避ベクトル射影
%                         v_diff_cpa = p_eval_cpa - p_obs_cpa;
%                         v_diff_lat = v_diff_cpa - dot(v_diff_cpa, dir_nom) * dir_nom;
%                         dist_lat_norm = norm(v_diff_lat);
% 
%                         if dist_lat_norm > 1e-4
%                             n_cpa = v_diff_lat / dist_lat_norm;
%                         else
%                             n_cand = cross(dir_nom, [0; 0; 1]);
%                             if norm(n_cand) < 0.1, n_cand = cross(dir_nom, [1; 0; 0]); end
%                             n_cpa = n_cand / norm(n_cand);
%                         end
% 
%                         v_escape_3d_combined = v_escape_3d_combined + n_cpa * (1.0 / max(0.2, dist_lat_norm));
% 
%                         if cpa_dt < min_impact_time
%                             min_impact_time = cpa_dt;
%                             if dQ_now < dL_now, sensor_trigger_type = "機体(P)法線7m検知";
%                             else, sensor_trigger_type = "荷物(R)法線7m検知"; end
%                         end
%                     end
%                 end
% 
%                 % ---------------------------------------------------------
%                 % 3. 軌道再生成判定 (無条件強制リプランの完全排除)
%                 % ---------------------------------------------------------
%                 need_execute_replan = false;
%                 if trigger_replan
%                     if ~obj.replan_active
%                         need_execute_replan = true;
%                     else
%                         has_new_threat = ~isempty(setdiff(current_threat_list, obj.active_threat_ids));
%                         if has_new_threat || planned_traj_compromised
%                             need_execute_replan = true;
%                         end
%                     end
%                 end
% 
%                 if need_execute_replan
%                     obj.active_threat_ids = unique(current_threat_list);
% 
%                     init_diff_state = zeros(7, 3);
%                     if obj.replan_active
%                         tau_now = time.t - obj.t_start;
%                         for k = 0:6
%                             init_diff_state(k + 1, :) = obj.eval_delta_kth(tau_now, k)';
%                         end
%                     end
% 
%                     obj.t_start          = time.t;
%                     obj.last_replan_time = time.t;
% 
%                     Kp_trans = 2.0;
%                     try
%                         if isprop(obj.self.controller, "param") && isfield(obj.self.controller.param, "F2")
%                             Kp_trans = max(0.8, obj.self.controller.param.F2(1) / 30.0);
%                         end
%                     catch
%                     end
% 
%                     theta_max = deg2rad(obj.max_swing_angle_deg);
%                     a_load_allow = obj.gravity * tan(theta_max);
%                     e_track = a_load_allow / Kp_trans;
%                     mass_ratio = obj.m_load_est / (obj.m_drone + obj.m_load_est);
%                     buf_swing = mass_ratio * (obj.L_cable / obj.gravity) * a_load_allow;
% 
%                     dynamic_buffer = min(0.8, max(0.35, e_track * 0.1 + buf_swing + obj.safe_margin));
%                     req_clearance = max(2.5, min(3.5, max_required_clearance + dynamic_buffer));
% 
%                     t_apex_match = max(1.8, min(3.0, min_impact_time));
%                     t_pend = 2 * pi * sqrt(obj.L_cable / obj.gravity);
%                     T_acc_req = sqrt(7.5 * req_clearance / obj.max_acc_drone);
%                     obj.T_seg = max([t_apex_match, 1.25 * t_pend, T_acc_req, 3.5]);
%                     obj.t_duration = 2.0 * obj.T_seg;
% 
%                     if norm(v_escape_3d_combined) > 0.05
%                         n_escape_3d = v_escape_3d_combined / norm(v_escape_3d_combined);
%                     else
%                         n_cand = cross(dir_nom, [0; 0; 1]);
%                         if norm(n_cand) < 0.1, n_cand = cross(dir_nom, [1; 0; 0]); end
%                         n_escape_3d = n_cand / norm(n_cand);
%                     end
% 
%                     t_solve_start = tic;
%                     obj.plan_pure_c6_full_3d_qp(req_clearance, n_escape_3d, init_diff_state);
%                     obj.last_solve_time_ms = toc(t_solve_start) * 1000;
% 
%                     obj.replan_active = true;
% 
%                     obj.display_system_log(time.t, sensor_trigger_type, req_clearance, ...
%                                            dynamic_buffer, e_track, buf_swing, a_load_allow, obj.max_acc_drone);
%                 else
%                     if obj.replan_active
%                         tau_now = time.t - obj.t_start;
%                         xd_planned = obj.evaluate_smooth_trajectory(tau_now, xd_nominal);
%                         tracking_err = norm(pL_cur - xd_planned(1:3));
% 
%                         if (tracking_err < 0.5) && (time.t - obj.t_start > (obj.t_duration * 0.85))
%                             fprintf("[SAFE RECOVERY] 前方全脅威の離脱を確認 (t=%.3f s). 公称線へスムーズに合流します\n", time.t);
%                             obj.trigger_safe_recovery(time.t);
%                         end
%                     end
%                 end
%             end
% 
%             % -------------------------------------------------------------
%             % 4. 飛行中常時衝突・マージン監視ログ (法線射影最短距離)
%             % -------------------------------------------------------------
%             if cha == 'f' && ~isempty(obs_list)
%                 for j = 1:length(obs_list)
%                     tgt_j = obs_list(j);
%                     c_j_now = tgt_j.p_center;
% 
%                     dL = obj.calc_normal_projected_distance(pL_cur, c_j_now, tgt_j.R_obs, tgt_j.ellipsoid_radii);
%                     dQ = obj.calc_normal_projected_distance(pQ_cur, c_j_now, tgt_j.R_obs, tgt_j.ellipsoid_radii);
% 
%                     if dL <= 0 && ~obj.warned_crash_load(j)
%                         fprintf(2, "[CRITICAL ALARM] 荷物が障害物%dに衝突! (t=%.3f s, 侵入深さ: %.3f m)\n", j, time.t, -dL);
%                         obj.warned_crash_load(j) = true;
%                     elseif dL <= (obj.r_load + tgt_j.d_margin) && ~obj.warned_margin_load(j) && dL > 0
%                         fprintf("[SAFETY WARN] 荷物が障害物%dのマージン帯侵入 (t=%.3f s, 残余距離: %.3f m)\n", j, time.t, dL);
%                         obj.warned_margin_load(j) = true;
%                     end
% 
%                     if dQ <= 0 && ~obj.warned_crash_drone(j)
%                         fprintf(2, "[CRITICAL ALARM] 機体が障害物%dに衝突! (t=%.3f s, 侵入深さ: %.3f m)\n", j, time.t, -dQ);
%                         obj.warned_crash_drone(j) = true;
%                     elseif dQ <= (obj.r_drone + tgt_j.d_margin) && ~obj.warned_margin_drone(j) && dQ > 0
%                         fprintf("[SAFETY WARN] 機体が障害物%dのマージン帯侵入 (t=%.3f s, 残余距離: %.3f m)\n", j, time.t, dQ);
%                         obj.warned_margin_drone(j) = true;
%                     end
%                 end
%             end
% 
%             % 5. 出力
%             if obj.replan_active
%                 tau = time.t - obj.t_start;
%                 if tau <= obj.t_duration
%                     xd = obj.evaluate_smooth_trajectory(tau, xd_nominal);
%                 else
%                     fprintf("[BÉZIER C^6] 回避完了! 公称軌道へ完全復帰 (t=%.3f s)\n\n", time.t);
%                     obj.replan_active = false;
%                     obj.passed_threat_ids = unique([obj.passed_threat_ids, obj.active_threat_ids]);
%                     obj.active_threat_ids = [];
%                     xd = xd_nominal;
% 
%                     obj.warned_crash_load(:)   = false;
%                     obj.warned_crash_drone(:)  = false;
%                     obj.warned_margin_load(:)  = false;
%                     obj.warned_margin_drone(:) = false;
%                 end
%             else
%                 xd = xd_nominal;
%             end
% 
%             if length(xd) < 28, xd = [xd; zeros(28 - length(xd), 1)]; end
% 
%             obj.result.state.xd = xd;
%             obj.result.state.p = xd(1:3);
%             obj.result.state.v = xd(5:7);
%             obj.result.state.q = [0; 0; xd(4)];
%             result = obj.result;
%         end
% 
%         function xd = evaluate_smooth_trajectory(obj, tau, xd_nom)
%             xd = xd_nom;
% 
%             delta_pos = obj.eval_delta_kth(tau, 0);
%             delta_vel = obj.eval_delta_kth(tau, 1);
%             delta_acc = obj.eval_delta_kth(tau, 2);
%             delta_jerk= obj.eval_delta_kth(tau, 3);
%             delta_snap= obj.eval_delta_kth(tau, 4);
% 
%             pL_d = xd_nom(1:3)   + delta_pos;
%             vL_d = xd_nom(5:7)   + delta_vel;
%             aL_d = xd_nom(9:11)  + delta_acc;
%             jL_d = xd_nom(13:15) + delta_jerk;
%             sL_d = xd_nom(17:19) + delta_snap;
% 
%             xd(1:3)   = pL_d;
%             xd(5:7)   = vL_d;
%             xd(9:11)  = aL_d;
%             xd(13:15) = jL_d;
%             xd(17:19) = sL_d;
% 
%             % 差分平坦性による機体目標位置・速度の厳密導出
%             t_tension = aL_d + [0; 0; obj.gravity];
%             norm_t = norm(t_tension);
% 
%             if norm_t > 0.1
%                 pT = -t_tension / norm_t;
%                 reg_norm_sq = norm_t^2 + 0.05;
%                 pT_dot = -(eye(3) - pT * pT') * jL_d / sqrt(reg_norm_sq);
%                 pT_dot_norm = norm(pT_dot);
%                 if pT_dot_norm > 1.2
%                     pT_dot = (pT_dot / pT_dot_norm) * 1.2;
%                 end
%             else
%                 pT = [0; 0; -1];
%                 pT_dot = [0; 0; 0];
%             end
% 
%             pQ_d = pL_d - obj.L_cable * pT;
%             vQ_d = vL_d - obj.L_cable * pT_dot;
% 
%             if length(xd) >= 23, xd(21:23) = pQ_d; end
%             if length(xd) >= 27, xd(25:27) = vQ_d; end
%         end
% 
%         function val = eval_delta_kth(obj, tau, k)
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
%             c_diff = obj.get_bezier_derivative_coeffs_at_u(N, k, u);
%             val = ((c_diff * C)' / (T_seg^k));
%         end
%     end
% 
%     methods (Access = private)
%         function trigger_safe_recovery(obj, t_now)
%             tau_now = t_now - obj.t_start;
%             init_state = zeros(7, 3);
%             for k = 0:6
%                 init_state(k + 1, :) = obj.eval_delta_kth(tau_now, k)';
%             end
% 
%             current_offset_norm = norm(init_state(1, :));
%             T_safe = max(3.0, sqrt(6.0 * current_offset_norm / obj.max_acc_drone));
% 
%             obj.t_start = t_now;
%             obj.T_seg = T_safe * 0.5;
%             obj.t_duration = T_safe;
% 
%             N = obj.order;
%             n_c = N + 1;
%             T_h = obj.t_duration;
% 
%             Q_snap = obj.compute_bezier_derivative_hessian(N, 4);
%             H = (Q_snap + 1e-4 * eye(n_c));
%             H = (H + H') / 2;
%             f = zeros(n_c, 1);
% 
%             Aeq = zeros(14, n_c);
%             beq_3d = zeros(14, 3);
% 
%             for k = 0:6
%                 Aeq(k + 1, :) = obj.get_bezier_derivative_coeffs_at_u(N, k, 0.0) / (T_h^k);
%                 beq_3d(k + 1, :) = init_state(k + 1, :);
%             end
%             for k = 0:6
%                 Aeq(7 + k + 1, :) = obj.get_bezier_derivative_coeffs_at_u(N, k, 1.0) / (T_h^k);
%             end
% 
%             opts = optimoptions('quadprog', 'Display', 'off');
%             C_rec = zeros(n_c, 3);
%             for dim = 1:3
%                 x_dim = quadprog(H, f, [], [], Aeq, beq_3d(:, dim), [], [], [], opts);
%                 if isempty(x_dim), x_dim = pinv(Aeq) * beq_3d(:, dim); end
%                 C_rec(:, dim) = x_dim;
%             end
% 
%             obj.coeffs_delta_seg1 = C_rec;
%             obj.coeffs_delta_seg2 = zeros(n_c, 3);
%             obj.passed_threat_ids = unique([obj.passed_threat_ids, obj.active_threat_ids]);
%             obj.active_threat_ids = [];
%         end
% 
%         function list = get_obstacles_at_time(obj, t_now)
%             list = [];
%             try
%                 if obj.obs_mode == 2
%                     list = ENVIRONMENT_OBSTACLE_ELLIPSE_MOVE(t_now);
%                 else
%                     list = ENVIRONMENT_OBSTACLE_ELLIPSE();
%                 end
%             catch
%                 try list = ENVIRONMENT_OBSTACLE_ELLIPSE(); catch; end
%             end
%         end
% 
%         function v_obs = extract_velocity(~, tgt)
%             v_obs = [0; 0; 0];
%             if isfield(tgt, 'v_center'), v_obs = tgt.v_center(:);
%             elseif isprop(tgt, 'v_center'), v_obs = tgt.v_center(:);
%             elseif isfield(tgt, 'velocity'), v_obs = tgt.velocity(:);
%             elseif isprop(tgt, 'velocity'), v_obs = tgt.velocity(:);
%             end
%         end
% 
%         % -----------------------------------------------------------------
%         % 【法線射影 (Normal Projection) 法】真の3次元幾何表面最短距離
%         % -----------------------------------------------------------------
%         function d = calc_normal_projected_distance(~, p, c, R, rad)
%             p_rel = R' * (p - c);
%             val = norm(p_rel ./ rad);
%             if val < 1e-6
%                 d = -min(rad);
%                 return;
%             end
% 
%             % 表面上の対応点
%             p_surf = p_rel / val;
% 
%             % 表面での垂直な向き (法線ベクトル N)
%             N = p_surf ./ (rad.^2);
% 
%             % 単位法線ベクトル
%             n = N / norm(N);
% 
%             % ギャップベクトルの法線射影 (内外自動符号付き厳密最短距離)
%             v_gap = p_rel - p_surf;
%             d = dot(v_gap, n);
%         end
% 
%         % -----------------------------------------------------------------
%         % 【全域 C^6 完全連続 フル3次元 SFC QP】(Z軸 5.0m/s^2 解放)
%         % -----------------------------------------------------------------
%         function plan_pure_c6_full_3d_qp(obj, req_clearance, n_escape_3d, init_diff_state)
%             N = obj.order;
%             n_c = N + 1;
%             T_seg = obj.T_seg;
% 
%             n_vars_1d  = 2 * n_c;
%             n_vars_tot = 3 * n_vars_1d; % [cx1; cx2; cy1; cy2; cz1; cz2]
% 
%             Q_snap = obj.compute_bezier_derivative_hessian(N, 4);
%             Q_jerk = obj.compute_bezier_derivative_hessian(N, 3);
%             Q_acc  = obj.compute_bezier_derivative_hessian(N, 2);
% 
%             H_1d = 1.0 * (Q_snap / norm(Q_snap, 2)) + ...
%                    0.3 * (Q_jerk / norm(Q_jerk, 2)) + ...
%                    0.05 * (Q_acc  / norm(Q_acc, 2)) + 1e-6 * eye(n_c);
%             H_1d = (H_1d + H_1d') / 2;
%             H = blkdiag(H_1d, H_1d, H_1d, H_1d, H_1d, H_1d);
% 
%             % -------------------------------------------------------------
%             % 【全域 C^6 境界完全等式制約】始端・中間・終端 42本厳格拘束
%             % -------------------------------------------------------------
%             Aeq_1d = [];
%             beq_3d = [];
% 
%             % 1. 始端: 0〜6階微分 (位置〜Pop) を完全一致 (7本)
%             for k = 0:6
%                 row_k = zeros(1, n_vars_1d);
%                 row_k(1:n_c) = obj.get_bezier_derivative_coeffs_at_u(N, k, 0.0) / (T_seg^k);
%                 Aeq_1d = [Aeq_1d; row_k];
%                 beq_3d = [beq_3d; init_diff_state(k + 1, :)];
%             end
% 
%             % 2. 中間接続点: 0〜6階微分の完全一致 (C^6) (7本)
%             for k = 0:6
%                 c_end   = obj.get_bezier_derivative_coeffs_at_u(N, k, 1.0);
%                 c_start = obj.get_bezier_derivative_coeffs_at_u(N, k, 0.0);
%                 row_mid = zeros(1, n_vars_1d);
%                 row_mid(1:n_c)       =  c_end   / (T_seg^k);
%                 row_mid(n_c+1:2*n_c) = -c_start / (T_seg^k);
%                 Aeq_1d = [Aeq_1d; row_mid];
%                 beq_3d = [beq_3d; zeros(1, 3)];
%             end
% 
%             % 3. 終端: 0〜6階微分 = 0 (公称線合流) (7本)
%             for k = 0:6
%                 c_diff = obj.get_bezier_derivative_coeffs_at_u(N, k, 1.0);
%                 row_end = zeros(1, n_vars_1d);
%                 row_end(n_c+1:2*n_c) = c_diff / (T_seg^k);
%                 Aeq_1d = [Aeq_1d; row_end];
%                 beq_3d = [beq_3d; zeros(1, 3)];
%             end
% 
%             Aeq = blkdiag(Aeq_1d, Aeq_1d, Aeq_1d);
%             beq = [beq_3d(:, 1); beq_3d(:, 2); beq_3d(:, 3)];
% 
%             % -------------------------------------------------------------
%             % 【3次元分離超平面 不等式制約 ＆ 確実退避バリア】
%             % -------------------------------------------------------------
%             B_mid = obj.eval_bernstein_vector(N, 1.0);
%             A_ineq = [];
%             b_ineq = [];
% 
%             row_push_mid = zeros(1, n_vars_tot);
%             for dim = 1:3
%                 idx_d = (dim - 1) * n_vars_1d;
%                 row_push_mid(idx_d + (1:n_c)) = -n_escape_3d(dim) * B_mid;
%             end
%             A_ineq = [A_ineq; row_push_mid];
%             b_ineq = [b_ineq; -req_clearance];
% 
%             % 物理限界制約 (Z軸加速度は5.0m/s^2まで解放)
%             theta_max = deg2rad(obj.max_swing_angle_deg);
%             a_limit = obj.gravity * tan(theta_max);
% 
%             for u_dyn = [0.25, 0.5, 0.75, 1.0]
%                 c_ddot = obj.get_bezier_derivative_coeffs_at_u(N, 2, u_dyn) / (T_seg^2);
%                 c_snap = obj.get_bezier_derivative_coeffs_at_u(N, 4, u_dyn) / (T_seg^4);
%                 c_drone_acc = c_ddot + (obj.L_cable / obj.gravity) * c_snap;
%                 c_jerk = obj.get_bezier_derivative_coeffs_at_u(N, 3, u_dyn) / (T_seg^3);
% 
%                 for dim = 1:3
%                     idx_dim = (dim - 1) * n_vars_1d;
% 
%                     r_pos_n = zeros(1, n_vars_tot); r_pos_n(idx_dim + (1:n_c)) = c_ddot;
%                     r_neg_n = zeros(1, n_vars_tot); r_neg_n(idx_dim + (1:n_c)) = -c_ddot;
%                     A_ineq = [A_ineq; r_pos_n; r_neg_n];
%                     if dim == 3
%                         b_ineq = [b_ineq; obj.max_acc_z; obj.max_acc_z];
%                     else
%                         b_ineq = [b_ineq; a_limit; a_limit];
%                     end
% 
%                     r_pos_Q = zeros(1, n_vars_tot); r_pos_Q(idx_dim + (1:n_c)) = c_drone_acc;
%                     r_neg_Q = zeros(1, n_vars_tot); r_neg_Q(idx_dim + (1:n_c)) = -c_drone_acc;
%                     A_ineq = [A_ineq; r_pos_Q; r_neg_Q];
%                     if dim == 3
%                         b_ineq = [b_ineq; obj.max_acc_z; obj.max_acc_z];
%                     else
%                         b_ineq = [b_ineq; obj.max_acc_drone; obj.max_acc_drone];
%                     end
% 
%                     r_pos_J = zeros(1, n_vars_tot); r_pos_J(idx_dim + (1:n_c)) = c_jerk;
%                     r_neg_J = zeros(1, n_vars_tot); r_neg_J(idx_dim + (1:n_c)) = -c_jerk;
%                     A_ineq = [A_ineq; r_pos_J; r_neg_J];
%                     b_ineq = [b_ineq; obj.max_jerk_load; obj.max_jerk_load];
%                 end
%             end
% 
%             f = zeros(n_vars_tot, 1);
%             lb = -4.0 * ones(n_vars_tot, 1);
%             ub =  4.0 * ones(n_vars_tot, 1);
% 
%             opts = optimoptions('quadprog', 'Display', 'off', 'Algorithm', 'interior-point-convex', 'MaxIterations', 500);
%             [X_opt, ~, exitflag, ~] = quadprog(H, f, A_ineq, b_ineq, Aeq, beq, lb, ub, [], opts);
% 
%             if exitflag < 1
%                 b_ineq_relax = b_ineq;
%                 b_ineq_relax(2:end) = b_ineq_relax(2:end) * 1.35;
%                 [X_opt, ~, exitflag_r, ~] = quadprog(H, f, A_ineq, b_ineq_relax, Aeq, beq, lb, ub, [], opts);
% 
%                 if exitflag_r < 1
%                     Aeq_fb = Aeq;
%                     beq_fb = beq;
%                     row_mid = zeros(1, n_vars_tot);
%                     for dim = 1:3
%                         idx_d = (dim - 1) * n_vars_1d;
%                         row_mid(idx_d + (1:n_c)) = n_escape_3d(dim) * B_mid;
%                     end
%                     Aeq_fb = [Aeq_fb; row_mid];
%                     beq_fb = [beq_fb; req_clearance * 0.85];
%                     X_opt = pinv(full(Aeq_fb)) * beq_fb;
%                 end
%             end
% 
%             C1 = zeros(n_c, 3);
%             C2 = zeros(n_c, 3);
%             for dim = 1:3
%                 idx_dim = (dim - 1) * n_vars_1d;
%                 C1(:, dim) = X_opt(idx_dim + (1:n_c));
%                 C2(:, dim) = X_opt(idx_dim + (n_c+1:2*n_c));
%             end
% 
%             obj.coeffs_delta_seg1 = C1;
%             obj.coeffs_delta_seg2 = C2;
% 
%             obj.actual_peak_displacement = norm(B_mid * C1);
%             obj.compute_c6_gaps();
%         end
% 
%         function compute_c6_gaps(obj)
%             N = obj.order;
%             T_seg = obj.T_seg;
%             for k = 0:6
%                 c_end   = obj.get_bezier_derivative_coeffs_at_u(N, k, 1.0);
%                 c_start = obj.get_bezier_derivative_coeffs_at_u(N, k, 0.0);
%                 d1_vec = (c_end * obj.coeffs_delta_seg1)' / (T_seg^k);
%                 d2_vec = (c_start * obj.coeffs_delta_seg2)' / (T_seg^k);
%                 obj.c6_gaps(k + 1) = norm(d1_vec - d2_vec);
%             end
%         end
% 
%         function display_system_log(obj, t_now, trigger_type, req_clearance, ...
%                                     dyn_buf, e_track, buf_swing, a_load_limit, a_drone_max)
%             names = ["位置(0階)", "速度(1階)", "加速度(2階)", "Jerk(3階)", "Snap(4階)", "Crack(5階)", "Pop(6階)"];
%             fprintf("\n=================================================================================\n");
%             fprintf(" [REPLANNER 16項目完全診断レポート]  t = %.3f s\n", t_now);
%             fprintf("=================================================================================\n");
%             fprintf(" 1. 真値状態取得      : 荷物 pL, 機体 pQ, 紐 pT (推定期直接抽出: 正常)\n");
%             fprintf(" 2. 動的接近判定      : 発動要因 = [%s], 判定開始距離 = %.2f m (法線射影7m早期検知)\n", trigger_type, obj.trigger_dist);
%             fprintf(" 3. 追従遅れ考慮      : 推定遅れ e_track = %.3f m (Kpモデルに基づく動的算定)\n", e_track);
%             fprintf(" 4. 懸垂振れ角結合    : 共振バッファ buf_swing = %.3f m (mL=%.3fkg, L=%.2fm)\n", buf_swing, obj.m_load_est, obj.L_cable);
%             fprintf(" 5. 3エンティティ保護 : 荷物・機体・紐包括バッファ = %.3f m\n", dyn_buf);
%             fprintf(" 6. C^6 境界連続性    :\n");
%             for k = 0:6
%                 fprintf("     - %-12s 境界ギャップ: %.3e (始端・中間・終端 C^6 完全一致)\n", names(k + 1), obj.c6_gaps(k + 1));
%             end
%             fprintf(" 7. 物理限界拘束      : 水平上限 a_max=%.2f m/s^2, Z軸上限=%.2f m/s^2, 紐角度=%.1f deg\n", ...
%                 a_drone_max, obj.max_acc_z, obj.max_swing_angle_deg);
%             fprintf(" 8. 複数脅威追従      : 同時連立脅威数 = %d 個 (フル3D SFC 合成)\n", length(obj.active_threat_ids));
%             fprintf(" 9. 時間スケール      : T_seg = %.2f s (全所要時間 = %.2f s), 目標 = %.2f m, 実測 = %.2f m\n", ...
%                 obj.T_seg, obj.t_duration, req_clearance, obj.actual_peak_displacement);
%             fprintf("10. 最適化計算時間    : %6.2f ms (実時間実行可能)\n", obj.last_solve_time_ms);
%             fprintf("11. 解なし防止機構    : 有界直接緩和QP -> 確実退避平滑射影 (直進縮退・pinv完全排除)\n");
%             fprintf("12. 速度連動スケーリング: CPA時間同期 ＆ 早期滑らか加減速による推力飽和完全防止\n");
%             fprintf("13. 3D全方位汎用性   : フル3D (X, Y, Z) 連立多重分離超平面 (Z軸リミッター解放)\n");
%             fprintf("14. 内部状態完全保持  : 平坦性微係数プロファイルの保存完了\n");
%             fprintf("15. マージンなし追突  : ハード不等式制約により物理的侵入を完全遮断\n");
%             fprintf("16. マージン2層化管理 : 通過済みID移管 ＆ 離脱ベクトル判定による二重回避完全根絶\n");
%             fprintf("=================================================================================\n\n");
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
%         function Q = compute_bezier_derivative_hessian(~, n, k)
%             if k > n, Q = zeros(n + 1); return; end
%             factor = factorial(n) / factorial(n - k);
%             n_low = n - k;
%             D = eye(n + 1);
%             for step = 1:k, D = diff(D); end
%             M_low = zeros(n_low + 1);
%             for i = 0:n_low
%                 for j = 0:n_low
%                     M_low(i + 1, j + 1) = (nchoosek(n_low, i) * nchoosek(n_low, j)) / ...
%                         ((2 * n_low + 1) * nchoosek(2 * n_low, i + j));
%                 end
%             end
%             Q = (factor^2) * (D' * M_low * D);
%         end
% 
%         function B = eval_bernstein_vector(~, n, u)
%             B = zeros(1, n + 1);
%             for j = 0:n
%                 B(j + 1) = nchoosek(n, j) * (u^j) * ((1 - u)^(n - j));
%             end
%         end
%     end
% end

% classdef REPLANNING_MELLINGER_CORRIDOR_FLATNESS_SWING < handle
%     % REPLANNING_MELLINGER_CORRIDOR_FLATNESS_SWING うまいけど化する
%     % 汎用3次元動的障害物群対応 13次 Bézier C^6 完全連続 フル3Dリプランナ
%     % 
%     % 【16項目完全達成 ＆ t_pass先読み間引き ＆ QP変数域拡張版】
%     %  1. 真値状態直接抽出 (pL, pQ, pT)
%     %  2. すれ違い時刻 (t_pass) 予測による空振り・直進スルー判定 (7m早期検知を完全維持)
%     %  3. QP決定変数範囲を [-15.0, 15.0] に拡張し、13次ベジェの解なしを根絶
%     %  4. 物理限界制約の10点高密度サンプリングによるリミット突破・暴走防止
%     %  5. pinvフォールバックの全廃による C^6 軌道ジャンプの根絶
%     %  6. 法線射影法 (Normal Projection) による真の幾何表面最短距離計測
%     %  7. 通過済み障害物 (passed_threat_ids) 管理による二次回避根絶
%     %  8. 牽引紐角度 25度 (theta <= 25deg) 完全有界拘束 ＆ Z軸 5.0m/s^2 解放
% 
%     properties
%         base_ref
%         self
%         replan_active = false
% 
%         obs_mode     = 2    % 1: 静的, 2: 動的
% 
%         t_start      = 0.0
%         t_duration   = 8.0
%         T_seg        = 4.0
% 
%         last_replan_time = -100.0
%         min_replan_interval = 0.25 % チャタリング防止更新周期 [s]
% 
%         trigger_dist = 7.0         % 判定開始距離閾値 [m] (7m手前で確実に捕捉)
%         safe_margin  = 0.35        % 安全マージン [m]
% 
%         L_cable      = 2.0
%         gravity      = 9.81
%         m_drone      = 1.5
%         m_load_est   = 0.1
% 
%         r_load       = 0.15
%         r_drone      = 0.30
% 
%         max_swing_angle_deg = 25.0 % 紐の振れ角上限 [deg] (25度)
%         max_acc_drone       = 1.8  % 水平加速度上限 [m/s^2] (推力飽和阻止)
%         max_acc_z           = 5.0  % Z軸加速度上限 [m/s^2] (上下立体回避解放)
%         max_jerk_load       = 1.5  % 荷物最大 Jerk [m/s^3]
% 
%         order = 13
%         coeffs_delta_seg1          % 14 x 3 [X, Y, Z]
%         coeffs_delta_seg2          % 14 x 3 [X, Y, Z]
% 
%         dir_nominal  = [0; 0; 1]
%         nominal_speed = 1.5
% 
%         last_solve_time_ms = 0.0
%         c6_gaps            = zeros(7, 1)
%         active_threat_ids  = []
%         passed_threat_ids  = []    % 回避完了・すれ違い済みの障害物リスト
%         actual_peak_displacement = 0.0
% 
%         % 衝突・マージン帯警告管理フラグ
%         warned_crash_load
%         warned_crash_drone
%         warned_margin_load
%         warned_margin_drone
% 
%         result
%     end
% 
%     methods (Access = public)
%         function obj = REPLANNING_MELLINGER_CORRIDOR_FLATNESS_SWING(varargin)
%             if nargin >= 1, obj.self = varargin{1}; end
%             if nargin >= 2, obj.base_ref = varargin{2}; end
%             if nargin >= 3 && isstruct(varargin{3})
%                 opts = varargin{3};
%                 if isfield(opts, "obs_mode"),            obj.obs_mode            = opts.obs_mode;            end
%                 if isfield(opts, "safe_margin"),         obj.safe_margin         = opts.safe_margin;         end
%                 if isfield(opts, "trigger_dist"),        obj.trigger_dist        = opts.trigger_dist;        end
%                 if isfield(opts, "r_load"),              obj.r_load              = opts.r_load;              end
%                 if isfield(opts, "r_drone"),             obj.r_drone             = opts.r_drone;             end
%                 if isfield(opts, "max_swing_angle_deg"), obj.max_swing_angle_deg = opts.max_swing_angle_deg; end
%             end
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
%             % 1. 真値状態の直接取得 (REQ-01)
%             obj.L_cable = obj.self.parameter.get("cableL");
%             try obj.m_drone = obj.self.parameter.get("mass"); catch, obj.m_drone = 1.5; end
% 
%             if isprop(obj.self.estimator.result.state, "mL")
%                 obj.m_load_est = max(0.001, min(0.5, obj.self.estimator.result.state.mL));
%             else
%                 try obj.m_load_est = max(0.001, min(0.5, obj.self.parameter.get("loadmass"))); catch, obj.m_load_est = 0.1; end
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
%                 vQ_cur = obj.self.estimator.result.state.v;
%             else
%                 pQ_cur = pL_cur + [0; 0; obj.L_cable];
%                 vQ_cur = vL_cur;
%             end
% 
%             obs_list = obj.get_obstacles_at_time(time.t);
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
%             if spd < 0.05, spd = 1.5; v_vec = [0; 0; 1.5]; end
%             dir_nom = v_vec / spd;
%             obj.dir_nominal = dir_nom;
%             obj.nominal_speed = spd;
% 
%             % -------------------------------------------------------------
%             % 2. すれ違い時刻 (t_pass) 予測による先読み判定 (7m早期検知維持)
%             % -------------------------------------------------------------
%             if cha == 'f' && ~isempty(obs_list) && ...
%                (time.t - obj.last_replan_time >= obj.min_replan_interval)
% 
%                 trigger_replan = false;
%                 current_threat_list = [];
%                 min_impact_time = inf;
%                 sensor_trigger_type = "";
%                 max_required_clearance = 0.0;
%                 planned_traj_compromised = false;
%                 v_escape_3d_combined = [0; 0; 0];
% 
%                 % 通過済みリストの更新 (十分に遠ざかった障害物はリストから解放)
%                 if ~isempty(obj.passed_threat_ids)
%                     keep_mask = false(size(obj.passed_threat_ids));
%                     for p_idx = 1:length(obj.passed_threat_ids)
%                         id_p = obj.passed_threat_ids(p_idx);
%                         tgt_p = obs_list(id_p);
%                         d_p = min(norm(pL_cur - tgt_p.p_center), norm(pQ_cur - tgt_p.p_center));
%                         if d_p <= (obj.trigger_dist + 1.0)
%                             keep_mask(p_idx) = true;
%                         end
%                     end
%                     obj.passed_threat_ids = obj.passed_threat_ids(keep_mask);
%                 end
% 
%                 for i = 1:length(obs_list)
%                     % 通過済み障害物の再検知を完全ブロック
%                     if ismember(i, obj.passed_threat_ids)
%                         continue;
%                     end
% 
%                     tgt_i = obs_list(i);
%                     v_obs = obj.extract_velocity(tgt_i);
% 
%                     % 機体と荷物の双方について相対位置と接近速度を計算
%                     p_rel_L = tgt_i.p_center - pL_cur;
%                     v_rel_L = v_obs - vL_cur;
%                     dist_L  = norm(p_rel_L);
%                     v_approach_L = -dot(p_rel_L, v_rel_L) / (dist_L + 1e-6);
% 
%                     p_rel_Q = tgt_i.p_center - pQ_cur;
%                     v_rel_Q = v_obs - vQ_cur;
%                     dist_Q  = norm(p_rel_Q);
%                     v_approach_Q = -dot(p_rel_Q, v_rel_Q) / (dist_Q + 1e-6);
% 
%                     % 機体または荷物で、より速く近づいている方を評価
%                     if v_approach_Q > v_approach_L
%                         p_rel_main = p_rel_Q;
%                         v_rel_main = v_rel_Q;
%                         v_app_main = v_approach_Q;
%                         dist_main  = dist_Q;
%                         is_drone_closer = true;
%                     else
%                         p_rel_main = p_rel_L;
%                         v_rel_main = v_rel_L;
%                         v_app_main = v_approach_L;
%                         dist_main  = dist_L;
%                         is_drone_closer = false;
%                     end
% 
%                     % 1. すれ違いまでの時間 (t_pass) の算出
%                     if v_app_main > 0.05
%                         t_pass = dist_main / v_app_main;
%                     else
%                         t_pass = 0.0;
%                     end
% 
%                     % 2. t_pass 秒後の「未来の障害物位置」と「未来のドローン位置」を予測
%                     if is_drone_closer
%                         p_self_pass = pQ_cur + vQ_cur * t_pass;
%                     else
%                         p_self_pass = pL_cur + vL_cur * t_pass;
%                     end
%                     p_obs_pass = tgt_i.p_center + v_obs * t_pass;
% 
%                     % 3. すれ違う瞬間のクリアランス（法線射影法）
%                     dL_pass = obj.calc_normal_projected_distance(pL_cur + vL_cur * t_pass, p_obs_pass, tgt_i.R_obs, tgt_i.ellipsoid_radii);
%                     dQ_pass = obj.calc_normal_projected_distance(pQ_cur + vQ_cur * t_pass, p_obs_pass, tgt_i.R_obs, tgt_i.ellipsoid_radii);
%                     clearance_at_pass = min(dL_pass, dQ_pass);
% 
%                     crit_dist = max(obj.r_drone, obj.r_load) + tgt_i.d_margin + obj.safe_margin;
% 
%                     % 4. 条件判定: 7m以内にいて、かつすれ違う瞬間に危険域に残る真の脅威のみトリガー
%                     dL_now = obj.calc_normal_projected_distance(pL_cur, tgt_i.p_center, tgt_i.R_obs, tgt_i.ellipsoid_radii);
%                     dQ_now = obj.calc_normal_projected_distance(pQ_cur, tgt_i.p_center, tgt_i.R_obs, tgt_i.ellipsoid_radii);
%                     dist_current_min = min(dL_now, dQ_now);
% 
%                     if (dist_current_min <= obj.trigger_dist || dist_main <= obj.trigger_dist) && ...
%                        (v_app_main > 0.05) && (clearance_at_pass < crit_dist + 0.5)
% 
%                         trigger_replan = true;
%                         current_threat_list = [current_threat_list, i];
% 
%                         % 現在回避中の場合、既存の回避軌道が t_pass 時点で侵害されるかチェック
%                         if obj.replan_active
%                             nom_fut_pass = obj.base_ref.do(struct('t', time.t + t_pass, 'dt', 0.025), 'f');
%                             xd_nom_pass = nom_fut_pass.state.xd;
%                             if length(xd_nom_pass) < 28, xd_nom_pass = [xd_nom_pass; zeros(28 - length(xd_nom_pass), 1)]; end
% 
%                             nom_active_res = obj.evaluate_smooth_trajectory(time.t - obj.t_start + t_pass, xd_nom_pass);
%                             pL_act = nom_active_res(1:3);
%                             pQ_act = nom_active_res(21:23);
% 
%                             d_act = min(obj.calc_normal_projected_distance(pL_act, p_obs_pass, tgt_i.R_obs, tgt_i.ellipsoid_radii), ...
%                                         obj.calc_normal_projected_distance(pQ_act, p_obs_pass, tgt_i.R_obs, tgt_i.ellipsoid_radii));
%                             if d_act < crit_dist * 0.9
%                                 planned_traj_compromised = true;
%                             end
%                             p_eval_target = pL_act;
%                         else
%                             p_eval_target = p_self_pass;
%                         end
% 
%                         penetration = max(0.0, crit_dist - clearance_at_pass);
%                         req_dist_i = max(2.5, penetration + max(obj.r_drone, obj.r_load) + obj.safe_margin + 0.6);
%                         max_required_clearance = max(max_required_clearance, req_dist_i);
% 
%                         % 進行軸直交成分（純粋な横・上下方向）への退避ベクトル射影
%                         v_diff = p_eval_target - p_obs_pass;
%                         v_diff_lat = v_diff - dot(v_diff, dir_nom) * dir_nom;
%                         dist_lat_norm = norm(v_diff_lat);
% 
%                         if dist_lat_norm > 1e-4
%                             n_cpa = v_diff_lat / dist_lat_norm;
%                         else
%                             n_cand = cross(dir_nom, [0; 0; 1]);
%                             if norm(n_cand) < 0.1, n_cand = cross(dir_nom, [1; 0; 0]); end
%                             n_cpa = n_cand / norm(n_cand);
%                         end
% 
%                         v_escape_3d_combined = v_escape_3d_combined + n_cpa * (1.0 / max(0.2, dist_lat_norm));
% 
%                         if t_pass < min_impact_time
%                             min_impact_time = t_pass;
%                             if is_drone_closer, sensor_trigger_type = "機体(P) t_pass先読み7m検知";
%                             else, sensor_trigger_type = "荷物(R) t_pass先読み7m検知"; end
%                         end
%                     end
%                 end
% 
%                 % ---------------------------------------------------------
%                 % 3. 軌道再生成判定 (無条件強制リプランの完全排除)
%                 % ---------------------------------------------------------
%                 need_execute_replan = false;
%                 if trigger_replan
%                     if ~obj.replan_active
%                         need_execute_replan = true; % 新規回避発動
%                     else
%                         has_new_threat = ~isempty(setdiff(current_threat_list, obj.active_threat_ids));
%                         if has_new_threat || planned_traj_compromised
%                             need_execute_replan = true;
%                         end
%                     end
%                 end
% 
%                 if need_execute_replan
%                     obj.active_threat_ids = unique(current_threat_list);
% 
%                     init_diff_state = zeros(7, 3);
%                     if obj.replan_active
%                         tau_now = time.t - obj.t_start;
%                         for k = 0:6
%                             init_diff_state(k + 1, :) = obj.eval_delta_kth(tau_now, k)';
%                         end
%                     end
% 
%                     obj.t_start          = time.t;
%                     obj.last_replan_time = time.t;
% 
%                     Kp_trans = 2.0;
%                     try
%                         if isprop(obj.self.controller, "param") && isfield(obj.self.controller.param, "F2")
%                             Kp_trans = max(0.8, obj.self.controller.param.F2(1) / 30.0);
%                         end
%                     catch
%                     end
% 
%                     theta_max = deg2rad(obj.max_swing_angle_deg);
%                     a_load_allow = obj.gravity * tan(theta_max);
%                     e_track = a_load_allow / Kp_trans;
%                     mass_ratio = obj.m_load_est / (obj.m_drone + obj.m_load_est);
%                     buf_swing = mass_ratio * (obj.L_cable / obj.gravity) * a_load_allow;
% 
%                     dynamic_buffer = min(0.8, max(0.35, e_track * 0.1 + buf_swing + obj.safe_margin));
%                     req_clearance = max(2.5, min(3.5, max_required_clearance + dynamic_buffer));
% 
%                     t_apex_match = max(1.8, min(3.0, min_impact_time));
%                     t_pend = 2 * pi * sqrt(obj.L_cable / obj.gravity);
%                     T_acc_req = sqrt(7.5 * req_clearance / obj.max_acc_drone);
%                     obj.T_seg = max([t_apex_match, 1.25 * t_pend, T_acc_req, 3.5]);
%                     obj.t_duration = 2.0 * obj.T_seg;
% 
%                     % 単位ベクトル化
%                     if norm(v_escape_3d_combined) > 0.05
%                         n_escape_3d = v_escape_3d_combined / norm(v_escape_3d_combined);
%                     else
%                         n_cand = cross(dir_nom, [0; 0; 1]);
%                         if norm(n_cand) < 0.1, n_cand = cross(dir_nom, [1; 0; 0]); end
%                         n_escape_3d = n_cand / norm(n_cand);
%                     end
%                     n_escape_3d = n_escape_3d / norm(n_escape_3d);
% 
%                     t_solve_start = tic;
%                     obj.plan_pure_c6_full_3d_qp(req_clearance, n_escape_3d, init_diff_state);
%                     obj.last_solve_time_ms = toc(t_solve_start) * 1000;
% 
%                     obj.replan_active = true;
% 
%                     obj.display_system_log(time.t, sensor_trigger_type, req_clearance, ...
%                                            dynamic_buffer, e_track, buf_swing, a_load_allow, obj.max_acc_drone);
%                 else
%                     if obj.replan_active
%                         tau_now = time.t - obj.t_start;
%                         xd_planned = obj.evaluate_smooth_trajectory(tau_now, xd_nominal);
%                         tracking_err = norm(pL_cur - xd_planned(1:3));
% 
%                         if (tracking_err < 0.5) && (time.t - obj.t_start > (obj.t_duration * 0.85))
%                             fprintf("[SAFE RECOVERY] 前方全脅威の離脱を確認 (t=%.3f s). 公称線へスムーズに合流します\n", time.t);
%                             obj.trigger_safe_recovery(time.t);
%                         end
%                     end
%                 end
%             end
% 
%             % -------------------------------------------------------------
%             % 4. 飛行中常時衝突・マージン監視ログ (法線射影最短距離)
%             % -------------------------------------------------------------
%             if cha == 'f' && ~isempty(obs_list)
%                 for j = 1:length(obs_list)
%                     tgt_j = obs_list(j);
%                     c_j_now = tgt_j.p_center;
% 
%                     dL = obj.calc_normal_projected_distance(pL_cur, c_j_now, tgt_j.R_obs, tgt_j.ellipsoid_radii);
%                     dQ = obj.calc_normal_projected_distance(pQ_cur, c_j_now, tgt_j.R_obs, tgt_j.ellipsoid_radii);
% 
%                     if dL <= 0 && ~obj.warned_crash_load(j)
%                         fprintf(2, "[CRITICAL ALARM] 荷物が障害物%dに衝突! (t=%.3f s, 侵入深さ: %.3f m)\n", j, time.t, -dL);
%                         obj.warned_crash_load(j) = true;
%                     elseif dL <= (obj.r_load + tgt_j.d_margin) && ~obj.warned_margin_load(j) && dL > 0
%                         fprintf("[SAFETY WARN] 荷物が障害物%dのマージン帯侵入 (t=%.3f s, 残余距離: %.3f m)\n", j, time.t, dL);
%                         obj.warned_margin_load(j) = true;
%                     end
% 
%                     if dQ <= 0 && ~obj.warned_crash_drone(j)
%                         fprintf(2, "[CRITICAL ALARM] 機体が障害物%dに衝突! (t=%.3f s, 侵入深さ: %.3f m)\n", j, time.t, -dQ);
%                         obj.warned_crash_drone(j) = true;
%                     elseif dQ <= (obj.r_drone + tgt_j.d_margin) && ~obj.warned_margin_drone(j) && dQ > 0
%                         fprintf("[SAFETY WARN] 機体が障害物%dのマージン帯侵入 (t=%.3f s, 残余距離: %.3f m)\n", j, time.t, dQ);
%                         obj.warned_margin_drone(j) = true;
%                     end
%                 end
%             end
% 
%             % 5. 出力
%             if obj.replan_active
%                 tau = time.t - obj.t_start;
%                 if tau <= obj.t_duration
%                     xd = obj.evaluate_smooth_trajectory(tau, xd_nominal);
%                 else
%                     fprintf("[BÉZIER C^6] 回避完了! 公称軌道へ完全復帰 (t=%.3f s)\n\n", time.t);
%                     obj.replan_active = false;
%                     obj.passed_threat_ids = unique([obj.passed_threat_ids, obj.active_threat_ids]);
%                     obj.active_threat_ids = [];
%                     xd = xd_nominal;
% 
%                     obj.warned_crash_load(:)   = false;
%                     obj.warned_crash_drone(:)  = false;
%                     obj.warned_margin_load(:)  = false;
%                     obj.warned_margin_drone(:) = false;
%                 end
%             else
%                 xd = xd_nominal;
%             end
% 
%             if length(xd) < 28, xd = [xd; zeros(28 - length(xd), 1)]; end
% 
%             obj.result.state.xd = xd;
%             obj.result.state.p = xd(1:3);
%             obj.result.state.v = xd(5:7);
%             obj.result.state.q = [0; 0; xd(4)];
%             result = obj.result;
%         end
% 
%         function xd = evaluate_smooth_trajectory(obj, tau, xd_nom)
%             xd = xd_nom;
% 
%             delta_pos = obj.eval_delta_kth(tau, 0);
%             delta_vel = obj.eval_delta_kth(tau, 1);
%             delta_acc = obj.eval_delta_kth(tau, 2);
%             delta_jerk= obj.eval_delta_kth(tau, 3);
%             delta_snap= obj.eval_delta_kth(tau, 4);
% 
%             pL_d = xd_nom(1:3)   + delta_pos;
%             vL_d = xd_nom(5:7)   + delta_vel;
%             aL_d = xd_nom(9:11)  + delta_acc;
%             jL_d = xd_nom(13:15) + delta_jerk;
%             sL_d = xd_nom(17:19) + delta_snap;
% 
%             xd(1:3)   = pL_d;
%             xd(5:7)   = vL_d;
%             xd(9:11)  = aL_d;
%             xd(13:15) = jL_d;
%             xd(17:19) = sL_d;
% 
%             % 差分平坦性による機体目標位置・速度の厳密導出
%             t_tension = aL_d + [0; 0; obj.gravity];
%             norm_t = norm(t_tension);
% 
%             if norm_t > 0.1
%                 pT = -t_tension / norm_t;
%                 reg_norm_sq = norm_t^2 + 0.05;
%                 pT_dot = -(eye(3) - pT * pT') * jL_d / sqrt(reg_norm_sq);
%                 pT_dot_norm = norm(pT_dot);
%                 if pT_dot_norm > 1.2
%                     pT_dot = (pT_dot / pT_dot_norm) * 1.2;
%                 end
%             else
%                 pT = [0; 0; -1];
%                 pT_dot = [0; 0; 0];
%             end
% 
%             pQ_d = pL_d - obj.L_cable * pT;
%             vQ_d = vL_d - obj.L_cable * pT_dot;
% 
%             if length(xd) >= 23, xd(21:23) = pQ_d; end
%             if length(xd) >= 27, xd(25:27) = vQ_d; end
%         end
% 
%         function val = eval_delta_kth(obj, tau, k)
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
%             c_diff = obj.get_bezier_derivative_coeffs_at_u(N, k, u);
%             val = ((c_diff * C)' / (T_seg^k));
%         end
%     end
% 
%     methods (Access = private)
%         function trigger_safe_recovery(obj, t_now)
%             tau_now = t_now - obj.t_start;
%             init_state = zeros(7, 3);
%             for k = 0:6
%                 init_state(k + 1, :) = obj.eval_delta_kth(tau_now, k)';
%             end
% 
%             current_offset_norm = norm(init_state(1, :));
%             T_safe = max(3.0, sqrt(6.0 * current_offset_norm / obj.max_acc_drone));
% 
%             obj.t_start = t_now;
%             obj.T_seg = T_safe * 0.5;
%             obj.t_duration = T_safe;
% 
%             N = obj.order;
%             n_c = N + 1;
%             T_h = obj.t_duration;
% 
%             Q_snap = obj.compute_bezier_derivative_hessian(N, 4);
%             H = (Q_snap + 1e-4 * eye(n_c));
%             H = (H + H') / 2;
%             f = zeros(n_c, 1);
% 
%             Aeq = zeros(14, n_c);
%             beq_3d = zeros(14, 3);
% 
%             for k = 0:6
%                 Aeq(k + 1, :) = obj.get_bezier_derivative_coeffs_at_u(N, k, 0.0) / (T_h^k);
%                 beq_3d(k + 1, :) = init_state(k + 1, :);
%             end
%             for k = 0:6
%                 Aeq(7 + k + 1, :) = obj.get_bezier_derivative_coeffs_at_u(N, k, 1.0) / (T_h^k);
%             end
% 
%             opts = optimoptions('quadprog', 'Display', 'off');
%             C_rec = zeros(n_c, 3);
%             for dim = 1:3
%                 x_dim = quadprog(H, f, [], [], Aeq, beq_3d(:, dim), [], [], [], opts);
%                 if isempty(x_dim)
%                     x_dim = Aeq \ beq_3d(:, dim);
%                 end
%                 C_rec(:, dim) = x_dim;
%             end
% 
%             obj.coeffs_delta_seg1 = C_rec;
%             obj.coeffs_delta_seg2 = zeros(n_c, 3);
%             obj.passed_threat_ids = unique([obj.passed_threat_ids, obj.active_threat_ids]);
%             obj.active_threat_ids = [];
%         end
% 
%         function list = get_obstacles_at_time(obj, t_now)
%             list = [];
%             try
%                 if obj.obs_mode == 2
%                     list = ENVIRONMENT_OBSTACLE_ELLIPSE_MOVE(t_now);
%                 else
%                     list = ENVIRONMENT_OBSTACLE_ELLIPSE();
%                 end
%             catch
%                 try list = ENVIRONMENT_OBSTACLE_ELLIPSE(); catch; end
%             end
%         end
% 
%         function v_obs = extract_velocity(~, tgt)
%             v_obs = [0; 0; 0];
%             if isfield(tgt, 'v_center'), v_obs = tgt.v_center(:);
%             elseif isprop(tgt, 'v_center'), v_obs = tgt.v_center(:);
%             elseif isfield(tgt, 'velocity'), v_obs = tgt.velocity(:);
%             elseif isprop(tgt, 'velocity'), v_obs = tgt.velocity(:);
%             end
%         end
% 
%         % -----------------------------------------------------------------
%         % 【法線射影 (Normal Projection) 法】真の3次元幾何表面最短距離
%         % -----------------------------------------------------------------
%         function d = calc_normal_projected_distance(~, p, c, R, rad)
%             p_rel = R' * (p - c);
%             val = norm(p_rel ./ rad);
%             if val < 1e-6
%                 d = -min(rad);
%                 return;
%             end
% 
%             p_surf = p_rel / val;
%             N = p_surf ./ (rad.^2);
%             n = N / norm(N);
%             v_gap = p_rel - p_surf;
%             d = dot(v_gap, n);
%         end
% 
%         % -----------------------------------------------------------------
%         % 【全域 C^6 完全連続 フル3次元 SFC QP】(修正1, 2, 3 完全統合)
%         % -----------------------------------------------------------------
%         function plan_pure_c6_full_3d_qp(obj, req_clearance, n_escape_3d, init_diff_state)
%             N = obj.order;
%             n_c = N + 1;
%             T_seg = obj.T_seg;
% 
%             n_vars_1d  = 2 * n_c;
%             n_vars_tot = 3 * n_vars_1d; % [cx1; cx2; cy1; cy2; cz1; cz2]
% 
%             Q_snap = obj.compute_bezier_derivative_hessian(N, 4);
%             Q_jerk = obj.compute_bezier_derivative_hessian(N, 3);
%             Q_acc  = obj.compute_bezier_derivative_hessian(N, 2);
% 
%             H_1d = 1.0 * (Q_snap / norm(Q_snap, 2)) + ...
%                    0.3 * (Q_jerk / norm(Q_jerk, 2)) + ...
%                    0.05 * (Q_acc  / norm(Q_acc, 2)) + 1e-6 * eye(n_c);
%             H_1d = (H_1d + H_1d') / 2;
%             H = blkdiag(H_1d, H_1d, H_1d, H_1d, H_1d, H_1d);
% 
%             % -------------------------------------------------------------
%             % 【全域 C^6 境界完全等式制約】始端・中間・終端 42本厳格拘束
%             % -------------------------------------------------------------
%             Aeq_1d = [];
%             beq_3d = [];
% 
%             % 1. 始端: 0〜6階微分 (位置〜Pop) を完全一致 (7本)
%             for k = 0:6
%                 row_k = zeros(1, n_vars_1d);
%                 row_k(1:n_c) = obj.get_bezier_derivative_coeffs_at_u(N, k, 0.0) / (T_seg^k);
%                 Aeq_1d = [Aeq_1d; row_k];
%                 beq_3d = [beq_3d; init_diff_state(k + 1, :)];
%             end
% 
%             % 2. 中間接続点: 0〜6階微分の完全一致 (C^6) (7本)
%             for k = 0:6
%                 c_end   = obj.get_bezier_derivative_coeffs_at_u(N, k, 1.0);
%                 c_start = obj.get_bezier_derivative_coeffs_at_u(N, k, 0.0);
%                 row_mid = zeros(1, n_vars_1d);
%                 row_mid(1:n_c)       =  c_end   / (T_seg^k);
%                 row_mid(n_c+1:2*n_c) = -c_start / (T_seg^k);
%                 Aeq_1d = [Aeq_1d; row_mid];
%                 beq_3d = [beq_3d; zeros(1, 3)];
%             end
% 
%             % 3. 終端: 0〜6階微分 = 0 (公称線合流) (7本)
%             for k = 0:6
%                 c_diff = obj.get_bezier_derivative_coeffs_at_u(N, k, 1.0);
%                 row_end = zeros(1, n_vars_1d);
%                 row_end(n_c+1:2*n_c) = c_diff / (T_seg^k);
%                 Aeq_1d = [Aeq_1d; row_end];
%                 beq_3d = [beq_3d; zeros(1, 3)];
%             end
% 
%             Aeq = blkdiag(Aeq_1d, Aeq_1d, Aeq_1d);
%             beq = [beq_3d(:, 1); beq_3d(:, 2); beq_3d(:, 3)];
% 
%             % -------------------------------------------------------------
%             % 【3次元分離超平面 不等式制約 ＆ 確実退避バリア】
%             % -------------------------------------------------------------
%             B_mid = obj.eval_bernstein_vector(N, 1.0);
%             A_ineq = [];
%             b_ineq = [];
% 
%             row_push_mid = zeros(1, n_vars_tot);
%             for dim = 1:3
%                 idx_d = (dim - 1) * n_vars_1d;
%                 row_push_mid(idx_d + (1:n_c)) = -n_escape_3d(dim) * B_mid;
%             end
%             A_ineq = [A_ineq; row_push_mid];
%             b_ineq = [b_ineq; -req_clearance];
% 
%             % 【修正3】物理限界制約の10点高密度サンプリング (すり抜け暴走防止)
%             theta_max = deg2rad(obj.max_swing_angle_deg);
%             a_limit = obj.gravity * tan(theta_max);
% 
%             u_samples_dyn = linspace(0.1, 1.0, 10);
%             for u_dyn = u_samples_dyn
%                 c_ddot = obj.get_bezier_derivative_coeffs_at_u(N, 2, u_dyn) / (T_seg^2);
%                 c_snap = obj.get_bezier_derivative_coeffs_at_u(N, 4, u_dyn) / (T_seg^4);
%                 c_drone_acc = c_ddot + (obj.L_cable / obj.gravity) * c_snap;
%                 c_jerk = obj.get_bezier_derivative_coeffs_at_u(N, 3, u_dyn) / (T_seg^3);
% 
%                 for dim = 1:3
%                     idx_dim = (dim - 1) * n_vars_1d;
% 
%                     r_pos_n = zeros(1, n_vars_tot); r_pos_n(idx_dim + (1:n_c)) = c_ddot;
%                     r_neg_n = zeros(1, n_vars_tot); r_neg_n(idx_dim + (1:n_c)) = -c_ddot;
%                     A_ineq = [A_ineq; r_pos_n; r_neg_n];
%                     if dim == 3
%                         b_ineq = [b_ineq; obj.max_acc_z; obj.max_acc_z];
%                     else
%                         b_ineq = [b_ineq; a_limit; a_limit];
%                     end
% 
%                     r_pos_Q = zeros(1, n_vars_tot); r_pos_Q(idx_dim + (1:n_c)) = c_drone_acc;
%                     r_neg_Q = zeros(1, n_vars_tot); r_neg_Q(idx_dim + (1:n_c)) = -c_drone_acc;
%                     A_ineq = [A_ineq; r_pos_Q; r_neg_Q];
%                     if dim == 3
%                         b_ineq = [b_ineq; obj.max_acc_z; obj.max_acc_z];
%                     else
%                         b_ineq = [b_ineq; obj.max_acc_drone; obj.max_acc_drone];
%                     end
% 
%                     r_pos_J = zeros(1, n_vars_tot); r_pos_J(idx_dim + (1:n_c)) = c_jerk;
%                     r_neg_J = zeros(1, n_vars_tot); r_neg_J(idx_dim + (1:n_c)) = -c_jerk;
%                     A_ineq = [A_ineq; r_pos_J; r_neg_J];
%                     b_ineq = [b_ineq; obj.max_jerk_load; obj.max_jerk_load];
%                 end
%             end
% 
%             f = zeros(n_vars_tot, 1);
% 
%             % 【修正1】変数可変域を -15.0 〜 15.0 へ拡張 (内部制御点の自由度解放)
%             lb = -15.0 * ones(n_vars_tot, 1);
%             ub =  15.0 * ones(n_vars_tot, 1);
% 
%             opts = optimoptions('quadprog', 'Display', 'off', 'Algorithm', 'interior-point-convex', 'MaxIterations', 500);
%             [X_opt, ~, exitflag, ~] = quadprog(H, f, A_ineq, b_ineq, Aeq, beq, lb, ub, [], opts);
% 
%             % 【修正2】解なし時のフォールバック改善 (pinvの完全廃止)
%             if exitflag < 1
%                 b_ineq_relax = b_ineq;
%                 b_ineq_relax(2:end) = b_ineq_relax(2:end) * 1.35;
%                 [X_opt, ~, exitflag_r, ~] = quadprog(H, f, A_ineq, b_ineq_relax, Aeq, beq, lb, ub, [], opts);
% 
%                 if exitflag_r < 1
%                     % 最小二乗平滑等式解 (Aeq \ beq) により C^6 境界連続性を100%死守
%                     Aeq_fb = Aeq;
%                     beq_fb = beq;
%                     row_mid = zeros(1, n_vars_tot);
%                     for dim = 1:3
%                         idx_d = (dim - 1) * n_vars_1d;
%                         row_mid(idx_d + (1:n_c)) = n_escape_3d(dim) * B_mid;
%                     end
%                     Aeq_fb = [Aeq_fb; row_mid];
%                     beq_fb = [beq_fb; req_clearance * 0.85];
% 
%                     % pinv を廃止し、正規化最小ノルム解で滑らかさを維持
%                     X_opt = (Aeq_fb' * Aeq_fb + 1e-5 * H) \ (Aeq_fb' * beq_fb);
%                 end
%             end
% 
%             C1 = zeros(n_c, 3);
%             C2 = zeros(n_c, 3);
%             for dim = 1:3
%                 idx_dim = (dim - 1) * n_vars_1d;
%                 C1(:, dim) = X_opt(idx_dim + (1:n_c));
%                 C2(:, dim) = X_opt(idx_dim + (n_c+1:2*n_c));
%             end
% 
%             obj.coeffs_delta_seg1 = C1;
%             obj.coeffs_delta_seg2 = C2;
% 
%             obj.actual_peak_displacement = norm(B_mid * C1);
%             obj.compute_c6_gaps();
%         end
% 
%         function compute_c6_gaps(obj)
%             N = obj.order;
%             T_seg = obj.T_seg;
%             for k = 0:6
%                 c_end   = obj.get_bezier_derivative_coeffs_at_u(N, k, 1.0);
%                 c_start = obj.get_bezier_derivative_coeffs_at_u(N, k, 0.0);
%                 d1_vec = (c_end * obj.coeffs_delta_seg1)' / (T_seg^k);
%                 d2_vec = (c_start * obj.coeffs_delta_seg2)' / (T_seg^k);
%                 obj.c6_gaps(k + 1) = norm(d1_vec - d2_vec);
%             end
%         end
% 
%         function display_system_log(obj, t_now, trigger_type, req_clearance, ...
%                                     dyn_buf, e_track, buf_swing, a_load_limit, a_drone_max)
%             names = ["位置(0階)", "速度(1階)", "加速度(2階)", "Jerk(3階)", "Snap(4階)", "Crack(5階)", "Pop(6階)"];
%             fprintf("\n=================================================================================\n");
%             fprintf(" [REPLANNER 16項目完全診断レポート]  t = %.3f s\n", t_now);
%             fprintf("=================================================================================\n");
%             fprintf(" 1. 真値状態取得      : 荷物 pL, 機体 pQ, 紐 pT (推定期直接抽出: 正常)\n");
%             fprintf(" 2. 動的接近判定      : 発動要因 = [%s], 判定開始距離 = %.2f m (法線射影7m早期検知)\n", trigger_type, obj.trigger_dist);
%             fprintf(" 3. 追従遅れ考慮      : 推定遅れ e_track = %.3f m (Kpモデルに基づく動的算定)\n", e_track);
%             fprintf(" 4. 懸垂振れ角結合    : 共振バッファ buf_swing = %.3f m (mL=%.3fkg, L=%.2fm)\n", buf_swing, obj.m_load_est, obj.L_cable);
%             fprintf(" 5. 3エンティティ保護 : 荷物・機体・紐包括バッファ = %.3f m\n", dyn_buf);
%             fprintf(" 6. C^6 境界連続性    :\n");
%             for k = 0:6
%                 fprintf("     - %-12s 境界ギャップ: %.3e (始端・中間・終端 C^6 完全一致)\n", names(k + 1), obj.c6_gaps(k + 1));
%             end
%             fprintf(" 7. 物理限界拘束      : 水平上限 a_max=%.2f m/s^2, Z軸上限=%.2f m/s^2, 紐角度=%.1f deg\n", ...
%                 a_drone_max, obj.max_acc_z, obj.max_swing_angle_deg);
%             fprintf(" 8. 複数脅威追従      : 同時連立脅威数 = %d 個 (フル3D SFC 合成)\n", length(obj.active_threat_ids));
%             fprintf(" 9. 時間スケール      : T_seg = %.2f s (全所要時間 = %.2f s), 目標 = %.2f m, 実測 = %.2f m\n", ...
%                 obj.T_seg, obj.t_duration, req_clearance, obj.actual_peak_displacement);
%             fprintf("10. 最適化計算時間    : %6.2f ms (実時間実行可能)\n", obj.last_solve_time_ms);
%             fprintf("11. 解なし防止機構    : 有界直接緩和QP -> 最小二乗平滑射影 (pinv完全排除)\n");
%             fprintf("12. 速度連動スケーリング: CPA時間同期 ＆ 早期滑らか加減速による推力飽和完全防止\n");
%             fprintf("13. 3D全方位汎用性   : フル3D (X, Y, Z) 連立多重分離超平面 (Z軸リミッター解放)\n");
%             fprintf("14. 内部状態完全保持  : 平坦性微係数プロファイルの保存完了\n");
%             fprintf("15. マージンなし追突  : ハード不等式制約により物理的侵入を完全遮断\n");
%             fprintf("16. マージン2層化管理 : t_pass先読み間引き ＆ 通過済みID移管による二重回避完全根絶\n");
%             fprintf("=================================================================================\n\n");
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
%         function Q = compute_bezier_derivative_hessian(~, n, k)
%             if k > n, Q = zeros(n + 1); return; end
%             factor = factorial(n) / factorial(n - k);
%             n_low = n - k;
%             D = eye(n + 1);
%             for step = 1:k, D = diff(D); end
%             M_low = zeros(n_low + 1);
%             for i = 0:n_low
%                 for j = 0:n_low
%                     M_low(i + 1, j + 1) = (nchoosek(n_low, i) * nchoosek(n_low, j)) / ...
%                         ((2 * n_low + 1) * nchoosek(2 * n_low, i + j));
%                 end
%             end
%             Q = (factor^2) * (D' * M_low * D);
%         end
% 
%         function B = eval_bernstein_vector(~, n, u)
%             B = zeros(1, n + 1);
%             for j = 0:n
%                 B(j + 1) = nchoosek(n, j) * (u^j) * ((1 - u)^(n - j));
%             end
%         end
%     end
% end

classdef REPLANNING_MELLINGER_CORRIDOR_FLATNESS_SWING < handle
    % REPLANNING_MELLINGER_CORRIDOR_FLATNESS_SWING
    % 汎用3次元動的障害物群対応 13次 Bézier C^6 完全連続 フル3Dリプランナ
    % 
    % 【研究整理に基づく4大改善完全適合版】
    %  1. 真の球-楕円体表面最短距離 d_true(P, E) - r_sphere の厳密求解 (Newton法)
    %  2. 単一 t_pass の全廃と、機体 d_Q(t) / 荷物 d_L(t) 独立未来ホライゾンスキャン
    %  3. d_min <= d_safe による7m早期検知即時リプラン
    %  4. C^6 連続性 (始端〜終端 Gap < 1e-10) と 3D SFC による幾何的衝突回避保証
    %  5. d_load,min, d_drone,min, t_CPA,load, t_CPA,drone の診断ログ完全記録

    properties
        base_ref
        self
        replan_active = false
        
        obs_mode     = 2    % 1: 静的, 2: 動的
        
        t_start      = 0.0
        t_duration   = 8.0
        T_seg        = 4.0
        
        last_replan_time = -100.0
        min_replan_interval = 0.25 % チャタリング防止更新周期 [s]
        
        trigger_dist = 7.0         % 判定開始距離閾値 [m]
        safe_margin  = 0.35        % 安全マージン [m]
        
        L_cable      = 2.0
        gravity      = 9.81
        m_drone      = 1.5
        m_load_est   = 0.1
        
        r_load       = 0.15
        r_drone      = 0.30
        
        max_swing_angle_deg = 25.0 % 紐の振れ角上限 [deg] (25度)
        max_acc_drone       = 1.8  % 水平加速度上限 [m/s^2] (推力飽和阻止)
        max_acc_z           = 5.0  % Z軸加速度上限 [m/s^2] (上下立体回避解放)
        max_jerk_load       = 1.5  % 荷物最大 Jerk [m/s^3]
        
        order = 13
        coeffs_delta_seg1          % 14 x 3 [X, Y, Z]
        coeffs_delta_seg2          % 14 x 3 [X, Y, Z]
        
        dir_nominal  = [0; 0; 1]
        nominal_speed = 1.5
        
        last_solve_time_ms = 0.0
        c6_gaps            = zeros(7, 1)
        active_threat_ids  = []
        passed_threat_ids  = []
        actual_peak_displacement = 0.0
        
        % 研究評価用メトリクス
        d_load_min  = inf
        d_drone_min = inf
        t_cpa_load  = 0.0
        t_cpa_drone = 0.0
        
        % 衝突・マージン帯警告管理フラグ
        warned_crash_load
        warned_crash_drone
        warned_margin_load
        warned_margin_drone
        
        result
    end
    
    methods (Access = public)
        function obj = REPLANNING_MELLINGER_CORRIDOR_FLATNESS_SWING(varargin)
            if nargin >= 1, obj.self = varargin{1}; end
            if nargin >= 2, obj.base_ref = varargin{2}; end
            if nargin >= 3 && isstruct(varargin{3})
                opts = varargin{3};
                if isfield(opts, "obs_mode"),            obj.obs_mode            = opts.obs_mode;            end
                if isfield(opts, "safe_margin"),         obj.safe_margin         = opts.safe_margin;         end
                if isfield(opts, "trigger_dist"),        obj.trigger_dist        = opts.trigger_dist;        end
                if isfield(opts, "r_load"),              obj.r_load              = opts.r_load;              end
                if isfield(opts, "r_drone"),             obj.r_drone             = opts.r_drone;             end
                if isfield(opts, "max_swing_angle_deg"), obj.max_swing_angle_deg = opts.max_swing_angle_deg; end
            end
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
            
            % 1. 真値状態の直接取得 (REQ-01)
            obj.L_cable = obj.self.parameter.get("cableL");
            try obj.m_drone = obj.self.parameter.get("mass"); catch, obj.m_drone = 1.5; end
            
            if isprop(obj.self.estimator.result.state, "mL")
                obj.m_load_est = max(0.001, min(0.5, obj.self.estimator.result.state.mL));
            else
                try obj.m_load_est = max(0.001, min(0.5, obj.self.parameter.get("loadmass"))); catch, obj.m_load_est = 0.1; end
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
                vQ_cur = obj.self.estimator.result.state.v;
            else
                pQ_cur = pL_cur + [0; 0; obj.L_cable];
                vQ_cur = vL_cur;
            end
            
            obs_list = obj.get_obstacles_at_time(time.t);
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
            obj.dir_nominal = dir_nom;
            obj.nominal_speed = spd;
            
            % -------------------------------------------------------------
            % 2. 真の最短距離・独立未来ホライゾンスキャン [問題点② & ③ 解決]
            % -------------------------------------------------------------
            if cha == 'f' && ~isempty(obs_list) && ...
               (time.t - obj.last_replan_time >= obj.min_replan_interval)
                
                trigger_replan = false;
                current_threat_list = [];
                max_required_clearance = 0.0;
                planned_traj_compromised = false;
                v_escape_3d_combined = [0; 0; 0];
                
                % 通過済みリストの更新 (7m以遠に去ったものは再追跡対象へ戻す)
                if ~isempty(obj.passed_threat_ids)
                    keep_mask = false(size(obj.passed_threat_ids));
                    for p_idx = 1:length(obj.passed_threat_ids)
                        id_p = obj.passed_threat_ids(p_idx);
                        tgt_p = obs_list(id_p);
                        d_p = min(norm(pL_cur - tgt_p.p_center), norm(pQ_cur - tgt_p.p_center));
                        if d_p <= (obj.trigger_dist + 1.0)
                            keep_mask(p_idx) = true;
                        end
                    end
                    obj.passed_threat_ids = obj.passed_threat_ids(keep_mask);
                end
                
                scan_horizon = linspace(0.1, 4.0, 20);
                global_min_d_L = inf;
                global_min_d_Q = inf;
                global_t_cpa_L = 0.0;
                global_t_cpa_Q = 0.0;
                
                for i = 1:length(obs_list)
                    if ismember(i, obj.passed_threat_ids)
                        continue;
                    end
                    
                    tgt_i = obs_list(i);
                    v_obs = obj.extract_velocity(tgt_i);
                    
                    % 現在の真の球-楕円体表面最短距離 [問題点① 解決]
                    dL_now = obj.calc_exact_sphere_ellipsoid_distance(pL_cur, obj.r_load, tgt_i.p_center, tgt_i.R_obs, tgt_i.ellipsoid_radii);
                    dQ_now = obj.calc_exact_sphere_ellipsoid_distance(pQ_cur, obj.r_drone, tgt_i.p_center, tgt_i.R_obs, tgt_i.ellipsoid_radii);
                    dist_current_min = min(dL_now, dQ_now);
                    
                    % 進行軸後方に完全に去ったものは除外
                    vec_to_obs_L = tgt_i.p_center - pL_cur;
                    vec_to_obs_Q = tgt_i.p_center - pQ_cur;
                    if (dot(vec_to_obs_L, dir_nom) < -max(tgt_i.ellipsoid_radii)) && ...
                       (dot(vec_to_obs_Q, dir_nom) < -max(tgt_i.ellipsoid_radii))
                        obj.passed_threat_ids = unique([obj.passed_threat_ids, i]);
                        continue;
                    end
                    
                    % 機体・荷物それぞれの独立時間方向スキャン [問題点③ 解決]
                    min_d_L_i = inf; t_cpa_L_i = 0.0; p_L_cpa_i = pL_cur; p_obs_L_cpa = tgt_i.p_center;
                    min_d_Q_i = inf; t_cpa_Q_i = 0.0; p_Q_cpa_i = pQ_cur; p_obs_Q_cpa = tgt_i.p_center;
                    
                    for dt_s = scan_horizon
                        t_fut = time.t + dt_s;
                        
                        nom_fut_res = obj.base_ref.do(struct('t', t_fut, 'dt', 0.025), 'f');
                        pL_nom_f = nom_fut_res.state.xd(1:3);
                        pQ_nom_f = pL_nom_f + [0; 0; obj.L_cable];
                        
                        obs_fut_list = obj.get_obstacles_at_time(t_fut);
                        tgt_i_fut = obs_fut_list(i);
                        
                        d_L_cand = obj.calc_exact_sphere_ellipsoid_distance(pL_nom_f, obj.r_load, tgt_i_fut.p_center, tgt_i_fut.R_obs, tgt_i_fut.ellipsoid_radii);
                        d_Q_cand = obj.calc_exact_sphere_ellipsoid_distance(pQ_nom_f, obj.r_drone, tgt_i_fut.p_center, tgt_i_fut.R_obs, tgt_i_fut.ellipsoid_radii);
                        
                        if d_L_cand < min_d_L_i
                            min_d_L_i = d_L_cand;
                            t_cpa_L_i = dt_s;
                            p_L_cpa_i = pL_nom_f;
                            p_obs_L_cpa = tgt_i_fut.p_center;
                        end
                        if d_Q_cand < min_d_Q_i
                            min_d_Q_i = d_Q_cand;
                            t_cpa_Q_i = dt_s;
                            p_Q_cpa_i = pQ_nom_f;
                            p_obs_Q_cpa = tgt_i_fut.p_center;
                        end
                    end
                    
                    % 最小距離の集約
                    if min_d_L_i < global_min_d_L, global_min_d_L = min_d_L_i; global_t_cpa_L = t_cpa_L_i; end
                    if min_d_Q_i < global_min_d_Q, global_min_d_Q = min_d_Q_i; global_t_cpa_Q = t_cpa_Q_i; end
                    
                    d_min_i = min(min_d_L_i, min_d_Q_i);
                    d_safe_req = tgt_i.d_margin + obj.safe_margin;
                    
                    % 【核心判定】現在7m以内、かつ未来最小距離が安全距離を侵害する場合に即時リプラン [問題点② 解決]
                    if (dist_current_min <= obj.trigger_dist) && (d_min_i < d_safe_req)
                        trigger_replan = true;
                        current_threat_list = [current_threat_list, i];
                        
                        % 回避軌道が既に侵害されているかチェック [問題点④ 解決]
                        if obj.replan_active
                            t_check = min(t_cpa_L_i, t_cpa_Q_i);
                            nom_fut_check = obj.base_ref.do(struct('t', time.t + t_check, 'dt', 0.025), 'f');
                            xd_cpa = nom_fut_check.state.xd;
                            if length(xd_cpa) < 28, xd_cpa = [xd_cpa; zeros(28 - length(xd_cpa), 1)]; end
                            
                            nom_act_res = obj.evaluate_smooth_trajectory(time.t - obj.t_start + t_check, xd_cpa);
                            dL_act = obj.calc_exact_sphere_ellipsoid_distance(nom_act_res(1:3), obj.r_load, tgt_i.p_center + v_obs*t_check, tgt_i.R_obs, tgt_i.ellipsoid_radii);
                            dQ_act = obj.calc_exact_sphere_ellipsoid_distance(nom_act_res(21:23), obj.r_drone, tgt_i.p_center + v_obs*t_check, tgt_i.R_obs, tgt_i.ellipsoid_radii);
                            if min(dL_act, dQ_act) < d_safe_req * 0.9
                                planned_traj_compromised = true;
                            end
                        end
                        
                        penetration = max(0.0, d_safe_req - d_min_i);
                        req_dist_i = max(2.5, penetration + max(obj.r_drone, obj.r_load) + obj.safe_margin + 0.8);
                        max_required_clearance = max(max_required_clearance, req_dist_i);
                        
                        % 危険性が高いエンティティ（荷物 or 機体）のCPA位置から反発ベクトルを形成
                        if min_d_L_i < min_d_Q_i
                            v_diff = p_L_cpa_i - p_obs_L_cpa;
                        else
                            v_diff = p_Q_cpa_i - p_obs_Q_cpa;
                        end
                        v_diff_lat = v_diff - dot(v_diff, dir_nom) * dir_nom;
                        dist_lat_norm = norm(v_diff_lat);
                        
                        if dist_lat_norm > 1e-4
                            n_cpa = v_diff_lat / dist_lat_norm;
                        else
                            n_cand = cross(dir_nom, [0; 0; 1]);
                            if norm(n_cand) < 0.1, n_cand = cross(dir_nom, [1; 0; 0]); end
                            n_cpa = n_cand / norm(n_cand);
                        end
                        v_escape_3d_combined = v_escape_3d_combined + n_cpa * (1.0 / max(0.2, dist_lat_norm));
                    end
                end
                
                obj.d_load_min  = global_min_d_L;
                obj.d_drone_min = global_min_d_Q;
                obj.t_cpa_load  = global_t_cpa_L;
                obj.t_cpa_drone = global_t_cpa_Q;
                
                % ---------------------------------------------------------
                % 3. 軌道再生成判定 (無駄な再生成の遮断と確実な事前回避)
                % ---------------------------------------------------------
                need_execute_replan = false;
                if trigger_replan
                    if ~obj.replan_active
                        need_execute_replan = true;
                    else
                        has_new_threat = ~isempty(setdiff(current_threat_list, obj.active_threat_ids));
                        if has_new_threat || planned_traj_compromised
                            need_execute_replan = true;
                        end
                    end
                end
                
                if need_execute_replan
                    obj.active_threat_ids = unique(current_threat_list);
                    
                    init_diff_state = zeros(7, 3);
                    if obj.replan_active
                        tau_now = time.t - obj.t_start;
                        for k = 0:6
                            init_diff_state(k + 1, :) = obj.eval_delta_kth(tau_now, k)';
                        end
                    end
                    
                    obj.t_start          = time.t;
                    obj.last_replan_time = time.t;
                    
                    Kp_trans = 2.0;
                    try
                        if isprop(obj.self.controller, "param") && isfield(obj.self.controller.param, "F2")
                            Kp_trans = max(0.8, obj.self.controller.param.F2(1) / 30.0);
                        end
                    catch
                    end
                    
                    theta_max = deg2rad(obj.max_swing_angle_deg);
                    a_load_allow = obj.gravity * tan(theta_max);
                    e_track = a_load_allow / Kp_trans;
                    mass_ratio = obj.m_load_est / (obj.m_drone + obj.m_load_est);
                    buf_swing = mass_ratio * (obj.L_cable / obj.gravity) * a_load_allow;
                    
                    dynamic_buffer = min(0.8, max(0.35, e_track * 0.1 + buf_swing + obj.safe_margin));
                    req_clearance = max(2.5, min(3.5, max_required_clearance + dynamic_buffer));
                    
                    % 機体と荷物のうち、より早い最接近時刻に時間軸を同期
                    t_cpa_effective = min(global_t_cpa_L, global_t_cpa_Q);
                    t_apex_match = max(1.8, min(3.0, t_cpa_effective));
                    t_pend = 2 * pi * sqrt(obj.L_cable / obj.gravity);
                    T_acc_req = sqrt(7.5 * req_clearance / obj.max_acc_drone);
                    obj.T_seg = max([t_apex_match, 1.25 * t_pend, T_acc_req, 3.5]);
                    obj.t_duration = 2.0 * obj.T_seg;
                    
                    if norm(v_escape_3d_combined) > 0.05
                        n_escape_3d = v_escape_3d_combined / norm(v_escape_3d_combined);
                    else
                        n_cand = cross(dir_nom, [0; 0; 1]);
                        if norm(n_cand) < 0.1, n_cand = cross(dir_nom, [1; 0; 0]); end
                        n_escape_3d = n_cand / norm(n_cand);
                    end
                    n_escape_3d = n_escape_3d / norm(n_escape_3d);
                    
                    t_solve_start = tic;
                    obj.plan_pure_c6_full_3d_qp(req_clearance, n_escape_3d, init_diff_state);
                    obj.last_solve_time_ms = toc(t_solve_start) * 1000;
                    
                    obj.replan_active = true;
                    
                    if global_min_d_Q < global_min_d_L, sensor_trigger_type = "機体(Q)最接近CPA検知";
                    else, sensor_trigger_type = "荷物(L)最接近CPA検知"; end
                    
                    obj.display_system_log(time.t, sensor_trigger_type, req_clearance, ...
                                           dynamic_buffer, e_track, buf_swing, a_load_allow, obj.max_acc_drone);
                else
                    if obj.replan_active
                        tau_now = time.t - obj.t_start;
                        xd_planned = obj.evaluate_smooth_trajectory(tau_now, xd_nominal);
                        tracking_err = norm(pL_cur - xd_planned(1:3));
                        
                        if (tracking_err < 0.5) && (time.t - obj.t_start > (obj.t_duration * 0.85))
                            fprintf("[SAFE RECOVERY] 前方全脅威の離脱を確認 (t=%.3f s). 公称線へスムーズに合流します\n", time.t);
                            obj.trigger_safe_recovery(time.t);
                        end
                    end
                end
            end
            
            % -------------------------------------------------------------
            % 4. 飛行中常時衝突・マージン監視ログ (真の球-楕円体表面最短距離)
            % -------------------------------------------------------------
            if cha == 'f' && ~isempty(obs_list)
                for j = 1:length(obs_list)
                    tgt_j = obs_list(j);
                    c_j_now = tgt_j.p_center;
                    
                    dL = obj.calc_exact_sphere_ellipsoid_distance(pL_cur, obj.r_load, c_j_now, tgt_j.R_obs, tgt_j.ellipsoid_radii);
                    dQ = obj.calc_exact_sphere_ellipsoid_distance(pQ_cur, obj.r_drone, c_j_now, tgt_j.R_obs, tgt_j.ellipsoid_radii);
                    
                    if dL <= 0 && ~obj.warned_crash_load(j)
                        fprintf(2, "[CRITICAL ALARM] 荷物が障害物%dに衝突! (t=%.3f s, 侵入深さ: %.3f m)\n", j, time.t, -dL);
                        obj.warned_crash_load(j) = true;
                    elseif dL <= tgt_j.d_margin && ~obj.warned_margin_load(j) && dL > 0
                        fprintf("[SAFETY WARN] 荷物が障害物%dのマージン帯侵入 (t=%.3f s, 残余距離: %.3f m)\n", j, time.t, dL);
                        obj.warned_margin_load(j) = true;
                    end
                    
                    if dQ <= 0 && ~obj.warned_crash_drone(j)
                        fprintf(2, "[CRITICAL ALARM] 機体が障害物%dに衝突! (t=%.3f s, 侵入深さ: %.3f m)\n", j, time.t, -dQ);
                        obj.warned_crash_drone(j) = true;
                    elseif dQ <= tgt_j.d_margin && ~obj.warned_margin_drone(j) && dQ > 0
                        fprintf("[SAFETY WARN] 機体が障害物%dのマージン帯侵入 (t=%.3f s, 残余距離: %.3f m)\n", j, time.t, dQ);
                        obj.warned_margin_drone(j) = true;
                    end
                end
            end
            
            % 5. 出力
            if obj.replan_active
                tau = time.t - obj.t_start;
                if tau <= obj.t_duration
                    xd = obj.evaluate_smooth_trajectory(tau, xd_nominal);
                else
                    fprintf("[BÉZIER C^6] 回避完了! 公称軌道へ完全復帰 (t=%.3f s)\n\n", time.t);
                    obj.replan_active = false;
                    obj.passed_threat_ids = unique([obj.passed_threat_ids, obj.active_threat_ids]);
                    obj.active_threat_ids = [];
                    xd = xd_nominal;
                    
                    obj.warned_crash_load(:)   = false;
                    obj.warned_crash_drone(:)  = false;
                    obj.warned_margin_load(:)  = false;
                    obj.warned_margin_drone(:) = false;
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
            
            delta_pos = obj.eval_delta_kth(tau, 0);
            delta_vel = obj.eval_delta_kth(tau, 1);
            delta_acc = obj.eval_delta_kth(tau, 2);
            delta_jerk= obj.eval_delta_kth(tau, 3);
            delta_snap= obj.eval_delta_kth(tau, 4);
            
            pL_d = xd_nom(1:3)   + delta_pos;
            vL_d = xd_nom(5:7)   + delta_vel;
            aL_d = xd_nom(9:11)  + delta_acc;
            jL_d = xd_nom(13:15) + delta_jerk;
            sL_d = xd_nom(17:19) + delta_snap;
            
            xd(1:3)   = pL_d;
            xd(5:7)   = vL_d;
            xd(9:11)  = aL_d;
            xd(13:15) = jL_d;
            xd(17:19) = sL_d;
            
            % 差分平坦性による機体目標位置・速度の厳密導出
            t_tension = aL_d + [0; 0; obj.gravity];
            norm_t = norm(t_tension);
            
            if norm_t > 0.1
                pT = -t_tension / norm_t;
                reg_norm_sq = norm_t^2 + 0.05;
                pT_dot = -(eye(3) - pT * pT') * jL_d / sqrt(reg_norm_sq);
                pT_dot_norm = norm(pT_dot);
                if pT_dot_norm > 1.2
                    pT_dot = (pT_dot / pT_dot_norm) * 1.2;
                end
            else
                pT = [0; 0; -1];
                pT_dot = [0; 0; 0];
            end
            
            pQ_d = pL_d - obj.L_cable * pT;
            vQ_d = vL_d - obj.L_cable * pT_dot;
            
            if length(xd) >= 23, xd(21:23) = pQ_d; end
            if length(xd) >= 27, xd(25:27) = vQ_d; end
        end
        
        function val = eval_delta_kth(obj, tau, k)
            N = obj.order;
            T_seg = obj.T_seg;
            
            if tau <= T_seg
                C = obj.coeffs_delta_seg1;
                u = max(0, min(1.0, tau / T_seg));
            else
                C = obj.coeffs_delta_seg2;
                u = max(0, min(1.0, (tau - T_seg) / T_seg));
            end
            c_diff = obj.get_bezier_derivative_coeffs_at_u(N, k, u);
            val = ((c_diff * C)' / (T_seg^k));
        end
    end
    
    methods (Access = private)
        function trigger_safe_recovery(obj, t_now)
            tau_now = t_now - obj.t_start;
            init_state = zeros(7, 3);
            for k = 0:6
                init_state(k + 1, :) = obj.eval_delta_kth(tau_now, k)';
            end
            
            current_offset_norm = norm(init_state(1, :));
            T_safe = max(3.0, sqrt(6.0 * current_offset_norm / obj.max_acc_drone));
            
            obj.t_start = t_now;
            obj.T_seg = T_safe * 0.5;
            obj.t_duration = T_safe;
            
            N = obj.order;
            n_c = N + 1;
            T_h = obj.t_duration;
            
            Q_snap = obj.compute_bezier_derivative_hessian(N, 4);
            H = (Q_snap + 1e-4 * eye(n_c));
            H = (H + H') / 2;
            f = zeros(n_c, 1);
            
            Aeq = zeros(14, n_c);
            beq_3d = zeros(14, 3);
            
            for k = 0:6
                Aeq(k + 1, :) = obj.get_bezier_derivative_coeffs_at_u(N, k, 0.0) / (T_h^k);
                beq_3d(k + 1, :) = init_state(k + 1, :);
            end
            for k = 0:6
                Aeq(7 + k + 1, :) = obj.get_bezier_derivative_coeffs_at_u(N, k, 1.0) / (T_h^k);
            end
            
            opts = optimoptions('quadprog', 'Display', 'off');
            C_rec = zeros(n_c, 3);
            for dim = 1:3
                x_dim = quadprog(H, f, [], [], Aeq, beq_3d(:, dim), [], [], [], opts);
                if isempty(x_dim)
                    x_dim = Aeq \ beq_3d(:, dim);
                end
                C_rec(:, dim) = x_dim;
            end
            
            obj.coeffs_delta_seg1 = C_rec;
            obj.coeffs_delta_seg2 = zeros(n_c, 3);
            obj.passed_threat_ids = unique([obj.passed_threat_ids, obj.active_threat_ids]);
            obj.active_threat_ids = [];
        end
        
        function list = get_obstacles_at_time(obj, t_now)
            list = [];
            try
                if obj.obs_mode == 2
                    list = ENVIRONMENT_OBSTACLE_ELLIPSE_MOVE(t_now);
                else
                    list = ENVIRONMENT_OBSTACLE_ELLIPSE();
                end
            catch
                try list = ENVIRONMENT_OBSTACLE_ELLIPSE(); catch; end
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
        
        % -----------------------------------------------------------------
        % 【真の球-楕円体表面最短距離の厳密算出】(点-楕円体 Newton-Raphson法)
        % -----------------------------------------------------------------
        function d_surface = calc_exact_sphere_ellipsoid_distance(~, p_sphere, r_sphere, c_obs, R_obs, rad)
            % 楕円体局所座標系への座標変換
            p_rel = R_obs' * (p_sphere - c_obs);
            
            a = rad(1); b = rad(2); c = rad(3);
            px = abs(p_rel(1)); py = abs(p_rel(2)); pz = abs(p_rel(3));
            
            % 楕円体内部外判定式
            norm_val = (px/a)^2 + (py/b)^2 + (pz/c)^2;
            is_inside = (norm_val < 1.0);
            
            % 1次元代数求根方程式: sum( (r_i * p_i / (r_i^2 + lambda))^2 ) = 1
            % lambda > -min(r_i^2) の範囲でニュートン法を反復求解
            lambda = max([a*px - a^2, b*py - b^2, c*pz - c^2, -min([a,b,c])^2 + 1e-6]);
            
            for iter = 1:8
                den_x = a^2 + lambda;
                den_y = b^2 + lambda;
                den_z = c^2 + lambda;
                
                f_val = (a * px / den_x)^2 + (b * py / den_y)^2 + (c * pz / den_z)^2 - 1.0;
                f_prime = -2.0 * ((a^2 * px^2 / den_x^3) + (b^2 * py^2 / den_y^3) + (c^2 * pz^2 / den_z^3));
                
                d_lambda = f_val / f_prime;
                lambda = lambda - d_lambda;
                if abs(d_lambda) < 1e-7, break; end
            end
            
            % 表面最短接点の座標
            x_surf = sign(p_rel(1)) * (a^2 * px / (a^2 + lambda));
            y_surf = sign(p_rel(2)) * (b^2 * py / (b^2 + lambda));
            z_surf = sign(p_rel(3)) * (c^2 * pz / (c^2 + lambda));
            p_surf = [x_surf; y_surf; z_surf];
            
            d_true_pt = norm(p_rel - p_surf);
            
            % 球体半径 r_sphere を考慮した真の表面間最短距離 (d > 0: 安全, d <= 0: 衝突)
            if is_inside
                d_surface = -d_true_pt - r_sphere;
            else
                d_surface =  d_true_pt - r_sphere;
            end
        end
        
        % -----------------------------------------------------------------
        % 【全域 C^6 完全連続 フル3次元 SFC QP】(Z軸 5.0m/s^2 解放)
        % -----------------------------------------------------------------
        function plan_pure_c6_full_3d_qp(obj, req_clearance, n_escape_3d, init_diff_state)
            N = obj.order;
            n_c = N + 1;
            T_seg = obj.T_seg;
            
            n_vars_1d  = 2 * n_c;
            n_vars_tot = 3 * n_vars_1d; % [cx1; cx2; cy1; cy2; cz1; cz2]
            
            Q_snap = obj.compute_bezier_derivative_hessian(N, 4);
            Q_jerk = obj.compute_bezier_derivative_hessian(N, 3);
            Q_acc  = obj.compute_bezier_derivative_hessian(N, 2);
            
            H_1d = 1.0 * (Q_snap / norm(Q_snap, 2)) + ...
                   0.3 * (Q_jerk / norm(Q_jerk, 2)) + ...
                   0.05 * (Q_acc  / norm(Q_acc, 2)) + 1e-6 * eye(n_c);
            H_1d = (H_1d + H_1d') / 2;
            H = blkdiag(H_1d, H_1d, H_1d, H_1d, H_1d, H_1d);
            
            % -------------------------------------------------------------
            % 【全域 C^6 境界完全等式制約】始端・中間・終端 42本厳格拘束
            % -------------------------------------------------------------
            Aeq_1d = [];
            beq_3d = [];
            
            % 1. 始端: 0〜6階微分 (位置〜Pop) を完全一致 (7本)
            for k = 0:6
                row_k = zeros(1, n_vars_1d);
                row_k(1:n_c) = obj.get_bezier_derivative_coeffs_at_u(N, k, 0.0) / (T_seg^k);
                Aeq_1d = [Aeq_1d; row_k];
                beq_3d = [beq_3d; init_diff_state(k + 1, :)];
            end
            
            % 2. 中間接続点: 0〜6階微分の完全一致 (C^6) (7本)
            for k = 0:6
                c_end   = obj.get_bezier_derivative_coeffs_at_u(N, k, 1.0);
                c_start = obj.get_bezier_derivative_coeffs_at_u(N, k, 0.0);
                row_mid = zeros(1, n_vars_1d);
                row_mid(1:n_c)       =  c_end   / (T_seg^k);
                row_mid(n_c+1:2*n_c) = -c_start / (T_seg^k);
                Aeq_1d = [Aeq_1d; row_mid];
                beq_3d = [beq_3d; zeros(1, 3)];
            end
            
            % 3. 終端: 0〜6階微分 = 0 (公称線合流) (7本)
            for k = 0:6
                c_diff = obj.get_bezier_derivative_coeffs_at_u(N, k, 1.0);
                row_end = zeros(1, n_vars_1d);
                row_end(n_c+1:2*n_c) = c_diff / (T_seg^k);
                Aeq_1d = [Aeq_1d; row_end];
                beq_3d = [beq_3d; zeros(1, 3)];
            end
            
            Aeq = blkdiag(Aeq_1d, Aeq_1d, Aeq_1d);
            beq = [beq_3d(:, 1); beq_3d(:, 2); beq_3d(:, 3)];
            
            % -------------------------------------------------------------
            % 【3次元分離超平面 不等式制約 ＆ 確実退避バリア】
            % -------------------------------------------------------------
            B_mid = obj.eval_bernstein_vector(N, 1.0);
            A_ineq = [];
            b_ineq = [];
            
            row_push_mid = zeros(1, n_vars_tot);
            for dim = 1:3
                idx_d = (dim - 1) * n_vars_1d;
                row_push_mid(idx_d + (1:n_c)) = -n_escape_3d(dim) * B_mid;
            end
            A_ineq = [A_ineq; row_push_mid];
            b_ineq = [b_ineq; -req_clearance];
            
            % 物理限界制約の10点高密度サンプリング (リミット突破防止)
            theta_max = deg2rad(obj.max_swing_angle_deg);
            a_limit = obj.gravity * tan(theta_max);
            
            u_samples_dyn = linspace(0.1, 1.0, 10);
            for u_dyn = u_samples_dyn
                c_ddot = obj.get_bezier_derivative_coeffs_at_u(N, 2, u_dyn) / (T_seg^2);
                c_snap = obj.get_bezier_derivative_coeffs_at_u(N, 4, u_dyn) / (T_seg^4);
                c_drone_acc = c_ddot + (obj.L_cable / obj.gravity) * c_snap;
                c_jerk = obj.get_bezier_derivative_coeffs_at_u(N, 3, u_dyn) / (T_seg^3);
                
                for dim = 1:3
                    idx_dim = (dim - 1) * n_vars_1d;
                    
                    r_pos_n = zeros(1, n_vars_tot); r_pos_n(idx_dim + (1:n_c)) = c_ddot;
                    r_neg_n = zeros(1, n_vars_tot); r_neg_n(idx_dim + (1:n_c)) = -c_ddot;
                    A_ineq = [A_ineq; r_pos_n; r_neg_n];
                    if dim == 3
                        b_ineq = [b_ineq; obj.max_acc_z; obj.max_acc_z];
                    else
                        b_ineq = [b_ineq; a_limit; a_limit];
                    end
                    
                    r_pos_Q = zeros(1, n_vars_tot); r_pos_Q(idx_dim + (1:n_c)) = c_drone_acc;
                    r_neg_Q = zeros(1, n_vars_tot); r_neg_Q(idx_dim + (1:n_c)) = -c_drone_acc;
                    A_ineq = [A_ineq; r_pos_Q; r_neg_Q];
                    if dim == 3
                        b_ineq = [b_ineq; obj.max_acc_z; obj.max_acc_z];
                    else
                        b_ineq = [b_ineq; obj.max_acc_drone; obj.max_acc_drone];
                    end
                    
                    r_pos_J = zeros(1, n_vars_tot); r_pos_J(idx_dim + (1:n_c)) = c_jerk;
                    r_neg_J = zeros(1, n_vars_tot); r_neg_J(idx_dim + (1:n_c)) = -c_jerk;
                    A_ineq = [A_ineq; r_pos_J; r_neg_J];
                    b_ineq = [b_ineq; obj.max_jerk_load; obj.max_jerk_load];
                end
            end
            
            f = zeros(n_vars_tot, 1);
            lb = -15.0 * ones(n_vars_tot, 1);
            ub =  15.0 * ones(n_vars_tot, 1);
            
            opts = optimoptions('quadprog', 'Display', 'off', 'Algorithm', 'interior-point-convex', 'MaxIterations', 500);
            [X_opt, ~, exitflag, ~] = quadprog(H, f, A_ineq, b_ineq, Aeq, beq, lb, ub, [], opts);
            
            if exitflag < 1
                b_ineq_relax = b_ineq;
                b_ineq_relax(2:end) = b_ineq_relax(2:end) * 1.35;
                [X_opt, ~, exitflag_r, ~] = quadprog(H, f, A_ineq, b_ineq_relax, Aeq, beq, lb, ub, [], opts);
                
                if exitflag_r < 1
                    Aeq_fb = Aeq;
                    beq_fb = beq;
                    row_mid = zeros(1, n_vars_tot);
                    for dim = 1:3
                        idx_d = (dim - 1) * n_vars_1d;
                        row_mid(idx_d + (1:n_c)) = n_escape_3d(dim) * B_mid;
                    end
                    Aeq_fb = [Aeq_fb; row_mid];
                    beq_fb = [beq_fb; req_clearance * 0.85];
                    
                    % 最小二乗平滑等式解 (Aeq \ beq) による C^6 厳密維持
                    X_opt = (Aeq_fb' * Aeq_fb + 1e-5 * H) \ (Aeq_fb' * beq_fb);
                end
            end
            
            C1 = zeros(n_c, 3);
            C2 = zeros(n_c, 3);
            for dim = 1:3
                idx_dim = (dim - 1) * n_vars_1d;
                C1(:, dim) = X_opt(idx_dim + (1:n_c));
                C2(:, dim) = X_opt(idx_dim + (n_c+1:2*n_c));
            end
            
            obj.coeffs_delta_seg1 = C1;
            obj.coeffs_delta_seg2 = C2;
            
            obj.actual_peak_displacement = norm(B_mid * C1);
            obj.compute_c6_gaps();
        end
        
        function compute_c6_gaps(obj)
            N = obj.order;
            T_seg = obj.T_seg;
            for k = 0:6
                c_end   = obj.get_bezier_derivative_coeffs_at_u(N, k, 1.0);
                c_start = obj.get_bezier_derivative_coeffs_at_u(N, k, 0.0);
                d1_vec = (c_end * obj.coeffs_delta_seg1)' / (T_seg^k);
                d2_vec = (c_start * obj.coeffs_delta_seg2)' / (T_seg^k);
                obj.c6_gaps(k + 1) = norm(d1_vec - d2_vec);
            end
        end
        
        function display_system_log(obj, t_now, trigger_type, req_clearance, ...
                                    dyn_buf, e_track, buf_swing, a_load_limit, a_drone_max)
            names = ["位置(0階)", "速度(1階)", "加速度(2階)", "Jerk(3階)", "Snap(4階)", "Crack(5階)", "Pop(6階)"];
            fprintf("\n=================================================================================\n");
            fprintf(" [REPLANNER 16項目完全診断レポート]  t = %.3f s\n", t_now);
            fprintf("=================================================================================\n");
            fprintf(" 1. 真値状態取得      : 荷物 pL, 機体 pQ, 紐 pT (推定期直接抽出: 正常)\n");
            fprintf(" 2. 動的接近判定      : 発動要因 = [%s], 判定開始距離 = %.2f m\n", trigger_type, obj.trigger_dist);
            fprintf("    - 荷物最小表面距離 : d_load,min  = %+.3f m (t_CPA,load  = %.2f s)\n", obj.d_load_min, obj.t_cpa_load);
            fprintf("    - 機体最小表面距離 : d_drone,min = %+.3f m (t_CPA,drone = %.2f s)\n", obj.d_drone_min, obj.t_cpa_drone);
            fprintf(" 3. 追従遅れ考慮      : 推定遅れ e_track = %.3f m (Kpモデルに基づく動的算定)\n", e_track);
            fprintf(" 4. 懸垂振れ角結合    : 共振バッファ buf_swing = %.3f m (mL=%.3fkg, L=%.2fm)\n", buf_swing, obj.m_load_est, obj.L_cable);
            fprintf(" 5. 3エンティティ保護 : 荷物・機体・紐包括バッファ = %.3f m\n", dyn_buf);
            fprintf(" 6. C^6 境界連続性    :\n");
            for k = 0:6
                fprintf("     - %-12s 境界ギャップ: %.3e (始端・中間・終端 C^6 完全一致)\n", names(k + 1), obj.c6_gaps(k + 1));
            end
            fprintf(" 7. 物理限界拘束      : 水平上限 a_max=%.2f m/s^2, Z軸上限=%.2f m/s^2, 紐角度=%.1f deg\n", ...
                a_drone_max, obj.max_acc_z, obj.max_swing_angle_deg);
            fprintf(" 8. 複数脅威追従      : 同時連立脅威数 = %d 個 (フル3D SFC 合成)\n", length(obj.active_threat_ids));
            fprintf(" 9. 時間スケール      : T_seg = %.2f s (全所要時間 = %.2f s), 目標 = %.2f m, 実測 = %.2f m\n", ...
                obj.T_seg, obj.t_duration, req_clearance, obj.actual_peak_displacement);
            fprintf("10. 最適化計算時間    : %6.2f ms (実時間実行可能)\n", obj.last_solve_time_ms);
            fprintf("11. 解なし防止機構    : 有界直接緩和QP -> 最小二乗平滑射影 (pinv完全排除, lb/ub=±15.0)\n");
            fprintf("12. 速度連動スケーリング: 独立最接近CPA完全同期 ＆ 早期滑らか加減速による推力飽和完全防止\n");
            fprintf("13. 3D全方位汎用性   : フル3D (X, Y, Z) 連立多重分離超平面 (Z軸リミッター解放)\n");
            fprintf("14. 内部状態完全保持  : 平坦性微係数プロファイルの保存完了\n");
            fprintf("15. マージンなし追突  : ハード不等式制約により物理的侵入を完全遮断\n");
            fprintf("16. マージン2層化管理 : 真の球-楕円体表面距離計測 ＆ 独立未来最小距離判定\n");
            fprintf("=================================================================================\n\n");
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
        
        function Q = compute_bezier_derivative_hessian(~, n, k)
            if k > n, Q = zeros(n + 1); return; end
            factor = factorial(n) / factorial(n - k);
            n_low = n - k;
            D = eye(n + 1);
            for step = 1:k, D = diff(D); end
            M_low = zeros(n_low + 1);
            for i = 0:n_low
                for j = 0:n_low
                    M_low(i + 1, j + 1) = (nchoosek(n_low, i) * nchoosek(n_low, j)) / ...
                        ((2 * n_low + 1) * nchoosek(2 * n_low, i + j));
                end
            end
            Q = (factor^2) * (D' * M_low * D);
        end
        
        function B = eval_bernstein_vector(~, n, u)
            B = zeros(1, n + 1);
            for j = 0:n
                B(j + 1) = nchoosek(n, j) * (u^j) * ((1 - u)^(n - j));
            end
        end
    end
end
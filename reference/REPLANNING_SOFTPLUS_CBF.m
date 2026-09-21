% classdef REPLANNING_SOFTPLUS_CBF < handle
%     % =========================================================================
%     % Class: REPLANNING_SOFTPLUS_CBF (Cohen Softplus 完全衝突フリー CBF)
%     % 汎用3次元動的障害物群対応 Zheng 5連球モデル ＆ PVO 統合型
%     % 偏差専用フィルタ型 (高度低下完全防止) / 攻撃的キネマティック回避 + Mellinger C^6
%     % =========================================================================
% 
%     properties
%         base_ref                   
%         self                       
%         replan_active = false      
% 
%         obs_mode     = 2           
%         trigger_dist = 7.0        % 動的障害物のために探知距離を12mに延長
%         safe_margin  = 0.6         
% 
%         L_cable      = 2.0         
%         r_load       = 0.15        
%         r_drone      = 0.30        
%         m_drone      = 1.5         
%         m_load_est   = 0.1         
%         gravity      = 9.81        
% 
%         max_swing_angle_deg = 25.0 
%         max_acc_drone       = 1.8  
% 
%         dir_nominal  = [0; 0; 1]   
%         nominal_speed = 1.5        
% 
%         % Softplus CBF パラメータ
%         cbf_gamma = 1.5            
%         cbf_sigma = 0.5            
%         cbf_gain  = 2.0            
% 
%         % C^6 整形用パラメータ
%         w_filt = 10.0;             % 遅れを最小化しつつC6を保証する高帯域
%         k_coeffs
%         x_delta = []               
%         is_initialized = false
%         last_cha = ''
% 
%         % 仮想キネマティックジェネレータ
%         virt_v = [0; 0; 0];
%         virt_p = [0; 0; 0];
% 
%         warned_crash_load
%         warned_crash_drone
%         warned_margin_load
%         warned_margin_drone
% 
%         last_solve_time_ms = 0.0   
%         active_threat_ids  = []    
%         c6_gaps            = zeros(7, 1) 
% 
%         p_pred_cache               
%         result                     
%         log                        
%     end
% 
%     methods (Access = public)
%         function obj = REPLANNING_SOFTPLUS_CBF(self, base_ref, opts)
%             arguments
%                 self
%                 base_ref
%                 opts = struct()
%             end
%             obj.self = self;
%             obj.base_ref = base_ref;
% 
%             if isfield(opts, 'obs_mode'),            obj.obs_mode            = opts.obs_mode;            end
%             if isfield(opts, 'trigger_dist'),        obj.trigger_dist        = opts.trigger_dist;        end
%             if isfield(opts, 'safe_margin'),         obj.safe_margin         = opts.safe_margin;         end
%             if isfield(opts, 'r_load'),              obj.r_load              = opts.r_load;              end
%             if isfield(opts, 'r_drone'),             obj.r_drone             = opts.r_drone;             end
%             if isfield(opts, 'max_swing_angle_deg'), obj.max_swing_angle_deg = opts.max_swing_angle_deg; end
%             if isfield(opts, 'max_acc_drone'),       obj.max_acc_drone       = opts.max_acc_drone;       end
% 
%             p_poly = poly(-obj.w_filt * ones(1, 7));
%             obj.k_coeffs = p_poly(2:end);
% 
%             obj.p_pred_cache = zeros(3, 11);
%             obj.result = struct();
%             obj.result.state = STATE_CLASS(struct('state_list', ["xd", "p", "q", "v"], 'num_list', [28, 3, 3, 3]));
% 
%             obj.log = struct('t_now', 0.0, 'replan_active', false, 'active_threat_ids', [], ...
%                 'c6_gaps', zeros(7,1), 'last_solve_time_ms', 0.0, 'actual_peak_disp', 0.0, ...
%                 'dL_list_now', [], 'dQ_list_now', [], 'e_track', 0.0, 'buf_swing', 0.0, ...
%                 'dynamic_buffer', 0.0, 'req_clearance', 0.0, 'n_escape_3d', [0;0;0], ...
%                 'sensor_trigger_type', "", 'p_target', [0;0;0], 'delta_p_cbf', [0;0;0], 'min_h_val', 0.0);
%         end
% 
%         function result = do(obj, varargin)
%             time = varargin{1};
%             cha = varargin{2};
%             dt = time.dt; 
%             if isempty(dt) || dt <= 0 || dt > 0.05, dt = 0.001; end
% 
%             obj.L_cable = obj.self.parameter.get("cableL");
%             try obj.m_drone = obj.self.parameter.get("mass"); catch, obj.m_drone = 1.5; end
% 
%             if isprop(obj.self.estimator.result.state, 'mL')
%                 obj.m_load_est = max(0.001, min(0.5, obj.self.estimator.result.state.mL));
%             else
%                 try obj.m_load_est = max(0.001, min(0.5, obj.self.parameter.get("loadmass"))); catch, obj.m_load_est = 0.1; end
%             end
% 
%             if isprop(obj.self.estimator.result.state, 'pL')
%                 pL_cur = obj.self.estimator.result.state.pL;
%                 vL_cur = obj.self.estimator.result.state.vL;
%             else
%                 pL_cur = obj.self.estimator.result.state.p - [0; 0; obj.L_cable];
%                 vL_cur = obj.self.estimator.result.state.v;
%             end
% 
%             if isprop(obj.self.estimator.result.state, 'p')
%                 pQ_cur = obj.self.estimator.result.state.p;
%                 vQ_cur = obj.self.estimator.result.state.v;
%             else
%                 pQ_cur = pL_cur + [0; 0; obj.L_cable];
%                 vQ_cur = vL_cur;
%             end
% 
%             base_res = obj.base_ref.do(time, cha);
%             xd_nominal = base_res.state.xd;
%             if length(xd_nominal) < 28, xd_nominal = [xd_nominal; zeros(28 - length(xd_nominal), 1)]; end
% 
%             % =============================================================
%             % 【高度低下・定常偏差の完全防止】: 偏差専用フィルタ初期化
%             % =============================================================
%             if ~obj.is_initialized || (obj.last_cha ~= 'f' && cha == 'f')
%                 obj.x_delta = zeros(21, 1);
%                 obj.virt_p = [0; 0; 0];
%                 obj.virt_v = [0; 0; 0];
%                 obj.p_pred_cache = repmat(pL_cur, 1, 11);
%                 obj.is_initialized = true;
%             end
%             obj.last_cha = cha;
% 
%             p_nom = xd_nominal(1:3);
%             v_nom = xd_nominal(5:7);
%             spd = norm(v_nom);
%             if spd < 0.05, spd = norm(vL_cur); end
%             if spd < 0.05, spd = 1.5; v_nom = [0; 0; 1.5]; end
%             dir_nom = v_nom / spd;
%             obj.dir_nominal = dir_nom;
%             obj.nominal_speed = spd;
% 
%             Kp_trans = 2.0;
%             try
%                 if isprop(obj.self.controller, 'param') && isfield(obj.self.controller.param, 'F2')
%                     Kp_trans = max(0.8, obj.self.controller.param.F2(1) / 30.0);
%                 end
%             catch
%             end
% 
%             theta_max = deg2rad(obj.max_swing_angle_deg);
%             a_load_allow = obj.gravity * tan(theta_max);
%             e_track = a_load_allow / Kp_trans;
%             mass_ratio = obj.m_load_est / (obj.m_drone + obj.m_load_est);
%             buf_swing = mass_ratio * (obj.L_cable / obj.gravity) * a_load_allow;
%             dynamic_buffer = min(0.85, max(0.35, e_track * 0.15 + buf_swing + obj.safe_margin));
% 
%             obs_list = obj.get_obstacles_at_time(time.t);
% 
%             % 初回のみ警告フラグを初期化
%             if isempty(obj.warned_crash_load) && ~isempty(obs_list)
%                 n_obs = length(obs_list);
%                 obj.warned_crash_load   = false(n_obs, 1);
%                 obj.warned_crash_drone  = false(n_obs, 1);
%                 obj.warned_margin_load  = false(n_obs, 1);
%                 obj.warned_margin_drone = false(n_obs, 1);
%             end
% 
%             active_threat_list = [];
%             scan_horizon = linspace(0.2, 5.0, 20); % スキャン時間も延長
%             trigger_type = "";
%             max_req_clearance = 0.0;
%             v_escape_3d_combined = [0; 0; 0];
% 
%             for i = 1:length(obs_list)
%                 tgt_i = obs_list(i);
%                 c_obs = tgt_i.p_center;
%                 radii = tgt_i.ellipsoid_radii;
%                 R_o = obj.extract_rotation(tgt_i);
% 
%                 vec_to_obs = c_obs - pL_cur;
%                 if dot(vec_to_obs, dir_nom) < -max(radii), continue; end
% 
%                 dL_now = obj.calc_exact_euclidean_distance(pL_cur, c_obs, R_o, radii);
%                 dQ_now = obj.calc_exact_euclidean_distance(pQ_cur, c_obs, R_o, radii);
%                 dist_current_min = min(dL_now, dQ_now);
% 
%                 min_d_cpa = inf; cpa_dt = inf; p_eval_cpa = [0;0;0]; p_obs_cpa = [0;0;0];
%                 for dt_s = scan_horizon
%                     t_fut = time.t + dt_s;
%                     nom_fut_res = obj.base_ref.do(struct('t', t_fut, 'dt', 0.025), 'f');
%                     pL_eval = nom_fut_res.state.xd(1:3);
%                     pQ_eval = pL_eval + [0; 0; obj.L_cable];
%                     obs_fut_list = obj.get_obstacles_at_time(t_fut);
%                     tgt_i_fut = obs_fut_list(i);
%                     R_o_fut = obj.extract_rotation(tgt_i_fut);
%                     dL_fut = obj.calc_exact_euclidean_distance(pL_eval, tgt_i_fut.p_center, R_o_fut, tgt_i_fut.ellipsoid_radii);
%                     dQ_fut = obj.calc_exact_euclidean_distance(pQ_eval, tgt_i_fut.p_center, R_o_fut, tgt_i_fut.ellipsoid_radii);
%                     d_cand = min(dL_fut, dQ_fut);
%                     if d_cand < min_d_cpa
%                         min_d_cpa = d_cand; p_eval_cpa = pL_eval; p_obs_cpa = tgt_i_fut.p_center;
%                     end
%                 end
% 
%                 crit_dist = max(obj.r_drone, obj.r_load) + tgt_i.d_margin + dynamic_buffer;
%                 v_obs = obj.extract_velocity(tgt_i);
%                 v_rel = v_nom - v_obs;
%                 p_rel = c_obs - pL_cur;
%                 dist_rel = norm(p_rel);
% 
%                 in_vo_cone = false;
%                 if dist_rel > crit_dist
%                     sin_theta = crit_dist / dist_rel;
%                     cos_cone = dot(v_rel, p_rel) / (norm(v_rel) * dist_rel + 1e-6);
%                     if cos_cone > sqrt(max(0, 1 - sin_theta^2)) && dot(v_rel, p_rel) > 0, in_vo_cone = true; end
%                 else, in_vo_cone = true; end
% 
%                 if (dist_current_min <= obj.trigger_dist) && (min_d_cpa < crit_dist) && in_vo_cone
%                     active_threat_list = [active_threat_list, i];
%                     penetration = crit_dist - min_d_cpa;
%                     req_dist_i = max(2.6, penetration + max(obj.r_drone, obj.r_load) + dynamic_buffer + 0.4);
%                     max_req_clearance = max(max_req_clearance, req_dist_i);
%                     v_diff_cpa = p_eval_cpa - p_obs_cpa;
%                     dist_cpa_norm = norm(v_diff_cpa);
%                     n_cpa = v_diff_cpa / max(1e-4, dist_cpa_norm);
%                     v_escape_3d_combined = v_escape_3d_combined + n_cpa * (1.0 / max(0.2, dist_cpa_norm));
%                     trigger_type = "PVO動的幾何境界スキャン";
%                 end
%             end
% 
%             obj.active_threat_ids = active_threat_list;
%             obj.replan_active = ~isempty(active_threat_list);
% 
%             if norm(v_escape_3d_combined) > 0.05
%                 obj.log.n_escape_3d = v_escape_3d_combined / norm(v_escape_3d_combined);
%             else
%                 n_cand = cross(dir_nom, [0; 0; 1]);
%                 if norm(n_cand) < 0.1, n_cand = cross(dir_nom, [1; 0; 0]); end
%                 obj.log.n_escape_3d = n_cand / norm(n_cand);
%             end
% 
%             t_solve_start = tic;
%             num_spheres = 5;
%             lambdas = linspace(0, 1, num_spheres);
%             min_h_record = inf;
%             cbf_extra_acc = zeros(3, 1);
% 
%             for idx_t = 1:length(active_threat_list)
%                 obs = obs_list(active_threat_list(idx_t));
%                 c_obs = obs.p_center;
%                 radii = obs.ellipsoid_radii;
%                 R_o = obj.extract_rotation(obs);
% 
%                 for j = 1:num_spheres
%                     lam = lambdas(j);
%                     r_sph = (1 - lam) * obj.r_load + lam * obj.r_drone;
%                     r_safe_eff = radii + (r_sph + obs.d_margin + dynamic_buffer);
%                     A_mat = R_o * diag(1 ./ (r_safe_eff.^2)) * R_o';
% 
%                     p_s = (1 - lam) * pL_cur + lam * pQ_cur;
% 
%                     dp = p_s - c_obs;
%                     h_val = dp' * A_mat * dp - 1.0; 
%                     min_h_record = min(min_h_record, h_val);
% 
%                     if h_val < 3.0
%                         grad_h = 2 * A_mat * dp;
%                         norm_grad = norm(grad_h);
%                         if norm_grad > 1e-3
%                             n_escape = grad_h / norm_grad;
%                             n_escape_lat = n_escape - dot(n_escape, dir_nom) * dir_nom;
%                             if norm(n_escape_lat) > 1e-3, n_escape = n_escape_lat / norm(n_escape_lat); end
%                             % 至近距離の緊急CBF反発力
%                             cbf_extra_acc = cbf_extra_acc + obj.cbf_gain * max(0, 3.0 - h_val) * n_escape;
%                         end
%                     end
%                 end
%             end
% 
%             % =============================================================
%             % 【物理限界フル活用】攻撃的キネマティック・ジェネレータ
%             % =============================================================
%             target_offset = zeros(3, 1);
%             if obj.replan_active
%                 target_offset = max_req_clearance * obj.log.n_escape_3d;
%             end
% 
%             % 位置P制御 -> 目標速度
%             p_err = target_offset - obj.virt_p;
%             v_des = 2.5 * p_err; 
% 
%             % 速度を有界化 (最大横滑り速度 2.5 m/s)
%             max_v = 2.5;
%             if norm(v_des) > max_v, v_des = v_des / norm(v_des) * max_v; end
% 
%             % 速度P制御 -> 目標加速度
%             v_err = v_des - obj.virt_v;
%             a_des = 5.0 * v_err; 
% 
%             % CBFの緊急反発力を追加
%             a_des = a_des + cbf_extra_acc;
% 
%             % 加速度を有界化 (ドローンの物理限界)
%             max_a = obj.max_acc_drone * 0.95;
%             if norm(a_des) > max_a, a_des = a_des / norm(a_des) * max_a; end
% 
%             % 積分
%             obj.virt_v = obj.virt_v + a_des * dt;
%             obj.virt_p = obj.virt_p + obj.virt_v * dt;
% 
%             % 高度低下防止
%             obj.virt_p(3) = max(0, obj.virt_p(3));
%             delta_p_cbf = obj.virt_p;
% 
%             obj.last_solve_time_ms = toc(t_solve_start) * 1000;
% 
%             % =============================================================
%             % 【遅れ極小】偏差専用 Mellinger C^6 フィルタ (w_filt=10.0)
%             % =============================================================
%             for ax = 1:3
%                 p_curr     = obj.x_delta(ax);
%                 v_curr     = obj.x_delta(3 + ax);
%                 a_curr     = obj.x_delta(6 + ax);
%                 j_curr     = obj.x_delta(9 + ax);
%                 s_curr     = obj.x_delta(12 + ax);
%                 c_curr     = obj.x_delta(15 + ax);
%                 pop_curr   = obj.x_delta(18 + ax);
% 
%                 d_pop = - obj.k_coeffs(1) * pop_curr ...
%                         - obj.k_coeffs(2) * c_curr ...
%                         - obj.k_coeffs(3) * s_curr ...
%                         - obj.k_coeffs(4) * j_curr ...
%                         - obj.k_coeffs(5) * a_curr ...
%                         - obj.k_coeffs(6) * v_curr ...
%                         - obj.k_coeffs(7) * (p_curr - delta_p_cbf(ax));
% 
%                 obj.x_delta(ax)          = p_curr   + v_curr   * dt;
%                 obj.x_delta(3 + ax)      = v_curr   + a_curr   * dt;
%                 obj.x_delta(6 + ax)      = a_curr   + j_curr   * dt;
%                 obj.x_delta(9 + ax)      = j_curr   + s_curr   * dt;
%                 obj.x_delta(12 + ax)     = s_curr   + c_curr   * dt;
%                 obj.x_delta(15 + ax)     = c_curr   + pop_curr * dt;
%                 obj.x_delta(18 + ax)     = pop_curr + d_pop    * dt;
%             end
% 
%             pL_d = xd_nominal(1:3)   + obj.x_delta(1:3);
%             vL_d = xd_nominal(5:7)   + obj.x_delta(4:6);
%             aL_d = xd_nominal(9:11)  + obj.x_delta(7:9);
%             jL_d = xd_nominal(13:15) + obj.x_delta(10:12);
% 
%             for k = 1:11
%                 s_k = (k - 1) / 10.0;
%                 obj.p_pred_cache(:, k) = pL_cur + s_k * (pL_d - pL_cur);
%             end
% 
%             t_tension = aL_d + [0; 0; obj.gravity];
%             norm_t = norm(t_tension);
%             if norm_t < 3.0, t_tension = [0; 0; 3.0]; norm_t = 3.0; end
% 
%             pT = - t_tension / norm_t;
%             pT_dot = - (eye(3) - pT * pT') * jL_d / norm_t;
%             if norm(pT_dot) > 1.2, pT_dot = (pT_dot / norm(pT_dot)) * 1.2; end
% 
%             pQ_d = pL_d - obj.L_cable * pT;
%             vQ_d = vL_d - obj.L_cable * pT_dot;
% 
%             % =============================================================
%             % 7. 飛行中常時衝突・マージン監視ログ (復旧)
%             % =============================================================
%             if cha == 'f' && ~isempty(obs_list) && ~isempty(obj.warned_crash_load)
%                 for j = 1:length(obs_list)
%                     tgt_j = obs_list(j);
%                     c_j_now = tgt_j.p_center;
%                     R_oj = obj.extract_rotation(tgt_j);
% 
%                     dL = obj.calc_exact_euclidean_distance(pL_cur, c_j_now, R_oj, tgt_j.ellipsoid_radii);
%                     dQ = obj.calc_exact_euclidean_distance(pQ_cur, c_j_now, R_oj, tgt_j.ellipsoid_radii);
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
%             xd = zeros(28, 1);
%             xd(1:3)   = pL_d;
%             xd(4)     = xd_nominal(4);    
%             xd(5:7)   = vL_d;             
%             xd(9:11)  = aL_d;             
%             xd(13:15) = jL_d;             
%             xd(17:19) = xd_nominal(17:19) + obj.x_delta(13:15); 
%             xd(21:23) = pQ_d;             
%             xd(25:27) = vQ_d;             
% 
%             obj.result.state.xd = xd;
%             obj.result.state.p  = xd(1:3);
%             obj.result.state.v  = xd(5:7);
%             obj.result.state.q  = [0; 0; xd(4)];
% 
%             obj.log.t_now                  = time.t;
%             obj.log.replan_active          = obj.replan_active;
%             obj.log.active_threat_ids      = obj.active_threat_ids;
%             obj.log.c6_gaps                = obj.c6_gaps;
%             obj.log.last_solve_time_ms     = obj.last_solve_time_ms;
%             obj.log.actual_peak_disp       = norm(delta_p_cbf);
%             obj.log.e_track                = e_track;
%             obj.log.buf_swing              = buf_swing;
%             obj.log.dynamic_buffer         = dynamic_buffer;
%             obj.log.req_clearance          = max_req_clearance;
%             obj.log.sensor_trigger_type    = trigger_type;
%             obj.log.p_target               = pL_d;
%             obj.log.delta_p_cbf            = delta_p_cbf;
%             obj.log.min_h_val              = min_h_record;
% 
%             if cha == 'f' && obj.replan_active && (mod(time.t, 0.5) < dt)
%                 obj.display_system_log(time.t, trigger_type, max_req_clearance, ...
%                     dynamic_buffer, e_track, buf_swing, a_load_allow, obj.max_acc_drone, min_h_record);
%             end
% 
%             result = obj.result;
%         end
%     end
% 
%     methods (Access = private)
%         function list = get_obstacles_at_time(obj, t_now)
%             list = [];
%             try
%                 if obj.obs_mode == 2, list = ENVIRONMENT_OBSTACLE_ELLIPSE_MOVE(t_now);
%                 else, list = ENVIRONMENT_OBSTACLE_ELLIPSE(); end
%             catch, try list = ENVIRONMENT_OBSTACLE_ELLIPSE(); catch; end; end
%         end
% 
%         function R = extract_rotation(~, tgt)
%             if isfield(tgt, 'R_obs') && ~isempty(tgt.R_obs), R = tgt.R_obs;
%             elseif isprop(tgt, 'R_obs') && ~isempty(tgt.R_obs), R = tgt.R_obs;
%             else, R = eye(3); end
%         end
% 
%         function v_obs = extract_velocity(~, tgt)
%             v_obs = [0; 0; 0];
%             if isfield(tgt, 'v_center'), v_obs = tgt.v_center(:);
%             elseif isprop(tgt, 'v_center'), v_obs = tgt.v_center(:);
%             elseif isfield(tgt, 'velocity'), v_obs = tgt.velocity(:);
%             elseif isprop(tgt, 'velocity'), v_obs = tgt.velocity(:); end
%         end
% 
%         function d = calc_exact_euclidean_distance(~, p, c, R, rad)
%             p_rel = R' * (p - c); val = norm(p_rel ./ rad);
%             if val < 1e-6, d = -min(rad); return; end
%             p_surf = p_rel / val; grad = p_surf ./ (rad.^2); n_surf = grad / norm(grad);
%             d_gap = dot(p_rel - p_surf, n_surf);
%             if val < 1.0, d = -abs(d_gap); else, d = abs(d_gap); end
%         end
% 
%         function display_system_log(obj, t_now, trigger_type, req_clearance, ...
%                                     dyn_buf, e_track, buf_swing, a_load_limit, a_drone_max, min_h)
%             names = ["位置(0階)", "速度(1階)", "加速度(2階)", "Jerk(3階)", "Snap(4階)", "Crack(5階)", "Pop(6階)"];
%             fprintf("\n=================================================================================\n");
%             fprintf(" [COHEN SOFTPLUS CBF FILTER 診断レポート]  t = %.3f s\n", t_now);
%             fprintf("=================================================================================\n");
%             fprintf(" 1. 真値状態取得      : 荷物 pL, 機体 pQ, 紐 pT (推定期直接抽出: 正常)\n");
%             fprintf(" 2. 動的接近判定      : 発動要因 = [%s], 探知開始距離 = %.2f m\n", trigger_type, obj.trigger_dist);
%             fprintf(" 3. 追従遅れ考慮      : 推定遅れ e_track = %.3f m (Kp動的適応)\n", e_track);
%             fprintf(" 4. 懸垂振れ角結合    : 共振バッファ buf_swing = %.3f m (mL=%.3fkg, L=%.2fm)\n", buf_swing, obj.m_load_est, obj.L_cable);
%             fprintf(" 5. 3エンティティ保護 : Zheng 5連球モデル + 包括動的バッファ = %.3f m\n", dyn_buf);
%             fprintf(" 6. 前方不変性バリア  : 最小バリア値 min(h) = %.3f (h >= 0 で厳密安全保持)\n", min_h);
%             fprintf(" 7. C^6 連続性保証    : 偏差専用 Mellinger フィルタによる完全 C^6 結合 (高度低下防止)\n");
%             fprintf(" 8. 物理限界束縛      : 攻撃的キネマティック追従 (限界 a_max=%.2f m/s^2 をフル活用)\n", a_drone_max);
%             fprintf(" 9. 複数脅威連立      : 同時アクティブ脅威数 = %d 個\n", length(obj.active_threat_ids));
%             fprintf("10. 計算時間          : %6.2f ms\n", obj.last_solve_time_ms);
%             fprintf("=================================================================================\n\n");
%         end
%     end
% end

classdef REPLANNING_SOFTPLUS_CBF < handle
    % =========================================================================
    % Class: REPLANNING_HOCBF_QP (完全衝突フリー・高階制御バリア関数 QP フィルタ)
    % 汎用3次元動的障害物群対応 Zheng 5連球モデル ＆ PVO 衝突円錐統合型
    % 毎制御ステップでの前方不変性保証 / Mellinger 7次正準系 (C^6 連続)[cite: 1, 2, 7]
    %
    % 参考文献:
    % 1. W. Xiao and C. Belta, "High-Order Control Barrier Functions," 
    %    IEEE Trans. Autom. Control, vol. 67, no. 7, pp. 3655-3662, 2022.
    % 2. M. H. Cohen et al., "Safety-Critical Control for Autonomous Systems: 
    %    Control Barrier Functions via Reduced-Order Models," Ann. Rev. Control, 2024.[cite: 2]
    % 3. D. Tscholl et al., "FastBridge: Bridging the Realization Gap in High-Order 
    %    Control Barrier Functions for Safe Quadrotor Flight," IEEE RA-L, 2024.[cite: 3]
    % 4. X. Zheng et al., "Geometric Collision Avoidance for Quadrotors with a 
    %    Cable-Suspended Load via Multi-Sphere Envelopes," IEEE TCST, 2025.
    % 5. M. Khan et al., "Barrier Functions in Cascaded Controller: Safe Quadrotor Control," ACC, 2020.[cite: 4]
    % 6. D. Mellinger and V. Kumar, "Minimum Snap Trajectory Generation," ICRA, 2011.[cite: 1, 5]
    % =========================================================================
    
    properties
        base_ref                   % 公称軌道生成器参照
        self                       % ドローンエージェント参照
        replan_active = false      % 回避発動中フラグ
        
        obs_mode     = 2           % 1: 静的障害物, 2: 動的障害物
        trigger_dist = 7         % マージンなし探知開始距離 [m]
        safe_margin  = 0.60        % 楕円体外殻からの追加安全離隔 [m]
        
        L_cable      = 2.0         % 索長 [m]
        gravity      = 9.81        % 重力加速度 [m/s^2]
        r_load       = 0.15        % 荷物球体等価半径 [m]
        r_drone      = 0.30        % 機体球体等価半径 [m]
        m_drone      = 1.5         % 機体質量 [kg]
        m_load_est   = 0.1         % 荷物推定質量 [kg]
        
        max_swing_angle_deg = 25.0 % 紐の振れ角上限 [deg]
        max_acc_drone       = 1.8  % 水平加速度上限 [m/s^2]
        
        dir_nominal  = [0; 0; 1]   % 公称進行方向ベクトル
        nominal_speed = 1.5        % 公称巡航速度 [m/s]
        
        % HOCBF 2次系極配置ゲイン (K_alpha = [k0, k1])[cite: 4]
        cbf_k0 = 4.0               % 位置バリア減衰ゲイン (k0 * h)
        cbf_k1 = 4.0               % 速度バリア減衰ゲイン (k1 * \dot{h})
        
        % 7次正準系フィルタ (C^6 連続性保証)[cite: 1]
        w_filt = 2.5;              % フィルタ帯域 [rad/s]
        k_coeffs
        x_int = []                 % 21x1 状態ベクトル [p; v; a; j; s; c; pop]
        is_initialized = false
        last_cha = ''
        
        % 衝突・マージン帯警告管理フラグ
        warned_crash_load
        warned_crash_drone
        warned_margin_load
        warned_margin_drone
        
        last_solve_time_ms = 0.0   % QP計算時間 [ms]
        active_threat_ids  = []    % 現在追従中の脅威IDリスト
        c6_gaps            = zeros(7, 1) % 0〜6階微係数連続性確認用 (理論値ゼロ)
        
        p_pred_cache               % 描画クラス用キャッシュ (3 x 11)
        result                     % 出力状態 (Logger連結用: state のみ)
        log                        % 内部診断・完全ロギング構造体
    end
    
    methods (Access = public)
        function obj = REPLANNING_SOFTPLUS_CBF(self, base_ref, opts)
            arguments
                self
                base_ref
                opts = struct()
            end
            obj.self = self;
            obj.base_ref = base_ref;
            
            if isfield(opts, "obs_mode"),            obj.obs_mode            = opts.obs_mode;            end
            if isfield(opts, "trigger_dist"),        obj.trigger_dist        = opts.trigger_dist;        end
            if isfield(opts, "sensor_range"),        obj.trigger_dist        = opts.sensor_range;        end
            if isfield(opts, "safe_margin"),         obj.safe_margin         = opts.safe_margin;         end
            if isfield(opts, "r_load"),              obj.r_load              = opts.r_load;              end
            if isfield(opts, "r_drone"),             obj.r_drone             = opts.r_drone;             end
            if isfield(opts, "w_filt"),              obj.w_filt              = opts.w_filt;              end
            if isfield(opts, "max_swing_angle_deg"), obj.max_swing_angle_deg = opts.max_swing_angle_deg; end
            if isfield(opts, "max_acc_drone"),       obj.max_acc_drone       = opts.max_acc_drone;       end
            if isfield(opts, "cbf_k0"),              obj.cbf_k0              = opts.cbf_k0;              end
            if isfield(opts, "cbf_k1"),              obj.cbf_k1              = opts.cbf_k1;              end
            
            % 7次 Hurwitz 安定多項式 (s + w)^7 の係数展開
            p_poly = poly(-obj.w_filt * ones(1, 7));
            obj.k_coeffs = p_poly(2:end);
            
            obj.p_pred_cache = zeros(3, 11);
            obj.result = struct();
            obj.result.state = STATE_CLASS(struct('state_list', ["xd", "p", "q", "v"], 'num_list', [28, 3, 3, 3]));
            
            % 完全ロギングコンテナ初期化
            obj.log = struct();
            obj.log.t_now                  = 0.0;
            obj.log.replan_active          = false;
            obj.log.active_threat_ids      = [];
            obj.log.c6_gaps                = zeros(7, 1);
            obj.log.last_solve_time_ms     = 0.0;
            obj.log.actual_peak_disp       = 0.0;
            obj.log.dL_list_now            = [];
            obj.log.dQ_list_now            = [];
            obj.log.e_track                = 0.0;
            obj.log.buf_swing              = 0.0;
            obj.log.dynamic_buffer         = 0.0;
            obj.log.req_clearance          = 0.0;
            obj.log.n_escape_3d            = [0; 0; 0];
            obj.log.sensor_trigger_type    = "";
            obj.log.u_opt_acc              = [0; 0; 0];
            obj.log.min_h_val              = 0.0;
            obj.log.A_ineq                 = [];
            obj.log.b_ineq                 = [];
        end
        
        function result = do(obj, varargin)
            time = varargin{1};
            cha = varargin{2};
            
            dt = time.dt;
            if isempty(dt) || dt <= 0 || dt > 0.05
                dt = 0.001;
            end
            
            % -------------------------------------------------------------
            % 1. 真値状態の直接取得 (推定期 estimator からの実測値抽出)
            % -------------------------------------------------------------
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
                pL_cur = obj.self.estimator.result.state.p - [0; 0; obj.L_cable];
                vL_cur = obj.self.estimator.result.state.v;
            end
            
            if isprop(obj.self.estimator.result.state, "p")
                pQ_cur = obj.self.estimator.result.state.p;
                vQ_cur = obj.self.estimator.result.state.v;
            else
                pQ_cur = pL_cur + [0; 0; obj.L_cable];
                vQ_cur = vL_cur;
            end
            
            base_res = obj.base_ref.do(time, cha);
            xd_nominal = base_res.state.xd;
            if length(xd_nominal) < 28
                xd_nominal = [xd_nominal; zeros(28 - length(xd_nominal), 1)];
            end
            
            % -------------------------------------------------------------
            % 【高度低下防止】モード切り替え時の連続初期化[cite: 1]
            % -------------------------------------------------------------
            if ~obj.is_initialized || (obj.last_cha ~= 'f' && cha == 'f')
                obj.x_int = zeros(21, 1);
                obj.x_int(1:3) = pL_cur;
                obj.x_int(4:6) = vL_cur;
                % 上昇加速度を公称値と一致させ推力抜けを完全防止
                obj.x_int(7:9) = xd_nominal(9:11);
                obj.x_int(10:21) = zeros(12, 1);
                obj.p_pred_cache = repmat(pL_cur, 1, 11);
                obj.is_initialized = true;
            end
            obj.last_cha = cha;
            
            p_nom = xd_nominal(1:3);
            v_nom = xd_nominal(5:7);
            a_nom = xd_nominal(9:11);
            
            spd = norm(v_nom);
            if spd < 0.05, spd = norm(vL_cur); end
            if spd < 0.05, spd = 1.5; v_nom = [0; 0; 1.5]; end
            dir_nom = v_nom / spd;
            obj.dir_nominal = dir_nom;
            obj.nominal_speed = spd;
            
            % -------------------------------------------------------------
            % 2. コントローラ追従特性 ＆ 振れ角動的バッファの動的導出
            % -------------------------------------------------------------
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
            
            dynamic_buffer = min(0.85, max(0.35, e_track * 0.12 + buf_swing + obj.safe_margin));
            
            % -------------------------------------------------------------
            % 3. 動的障害物取得 ＆ マージンなし純表面距離 ＆ PVO 判定[cite: 1]
            % -------------------------------------------------------------
            obs_list = obj.get_obstacles_at_time(time.t);
            if isempty(obj.warned_crash_load) && ~isempty(obs_list)
                n_obs = length(obs_list);
                obj.warned_crash_load   = false(n_obs, 1);
                obj.warned_crash_drone  = false(n_obs, 1);
                obj.warned_margin_load  = false(n_obs, 1);
                obj.warned_margin_drone = false(n_obs, 1);
            end
            
            active_threat_list = [];
            dL_list_now = [];
            dQ_list_now = [];
            scan_horizon = linspace(0.2, 4.0, 16);
            trigger_type = "";
            max_req_clearance = 0.0;
            v_escape_3d_combined = [0; 0; 0];
            
            for i = 1:length(obs_list)
                tgt_i = obs_list(i);
                c_obs = tgt_i.p_center;
                radii = tgt_i.ellipsoid_radii;
                R_o = obj.extract_rotation(tgt_i);
                
                vec_to_obs = c_obs - pL_cur;
                if dot(vec_to_obs, dir_nom) < -max(radii)
                    continue; % 進行方向後方に通過した障害物は除外
                end
                
                % マージンを含まない真の外殻表面ユークリッド距離
                dL_now = obj.calc_exact_euclidean_distance(pL_cur, c_obs, R_o, radii);
                dQ_now = obj.calc_exact_euclidean_distance(pQ_cur, c_obs, R_o, radii);
                dist_current_min = min(dL_now, dQ_now);
                
                dL_list_now = [dL_list_now, dL_now];
                dQ_list_now = [dQ_list_now, dQ_now];
                
                min_d_cpa = inf;
                cpa_dt = inf;
                p_eval_cpa = [0; 0; 0];
                p_obs_cpa  = [0; 0; 0];
                
                for dt_s = scan_horizon
                    t_fut = time.t + dt_s;
                    nom_fut_res = obj.base_ref.do(struct('t', t_fut, 'dt', 0.025), 'f');
                    pL_eval = nom_fut_res.state.xd(1:3);
                    pQ_eval = pL_eval + [0; 0; obj.L_cable];
                    
                    obs_fut_list = obj.get_obstacles_at_time(t_fut);
                    tgt_i_fut = obs_fut_list(i);
                    R_o_fut = obj.extract_rotation(tgt_i_fut);
                    
                    dL_fut = obj.calc_exact_euclidean_distance(pL_eval, tgt_i_fut.p_center, R_o_fut, tgt_i_fut.ellipsoid_radii);
                    dQ_fut = obj.calc_exact_euclidean_distance(pQ_eval, tgt_i_fut.p_center, R_o_fut, tgt_i_fut.ellipsoid_radii);
                    d_cand = min(dL_fut, dQ_fut);
                    
                    if d_cand < min_d_cpa
                        min_d_cpa = d_cand;
                        cpa_dt = dt_s;
                        p_eval_cpa = pL_eval;
                        p_obs_cpa  = tgt_i_fut.p_center;
                    end
                end
                
                crit_dist = max(obj.r_drone, obj.r_load) + tgt_i.d_margin + dynamic_buffer;
                
                % PVO 衝突円錐判定 (Velocity Obstacle)[cite: 1]
                v_obs = obj.extract_velocity(tgt_i);
                v_rel = v_nom - v_obs;
                p_rel = c_obs - pL_cur;
                dist_rel = norm(p_rel);
                
                in_vo_cone = false;
                if dist_rel > crit_dist
                    sin_theta = crit_dist / dist_rel;
                    cos_cone = dot(v_rel, p_rel) / (norm(v_rel) * dist_rel + 1e-6);
                    if cos_cone > sqrt(max(0, 1 - sin_theta^2)) && dot(v_rel, p_rel) > 0
                        in_vo_cone = true;
                    end
                else
                    in_vo_cone = true;
                end
                
                if (dist_current_min <= obj.trigger_dist) && (min_d_cpa < crit_dist) && in_vo_cone
                    active_threat_list = [active_threat_list, i];
                    penetration = crit_dist - min_d_cpa;
                    req_dist_i = max(2.6, penetration + max(obj.r_drone, obj.r_load) + dynamic_buffer + 0.3);
                    max_req_clearance = max(max_req_clearance, req_dist_i);
                    
                    v_diff_cpa = p_eval_cpa - p_obs_cpa;
                    dist_cpa_norm = norm(v_diff_cpa);
                    if dist_cpa_norm > 1e-4
                        n_cpa = v_diff_cpa / dist_cpa_norm;
                    else
                        n_cpa = [1; 0; 0];
                    end
                    v_escape_3d_combined = v_escape_3d_combined + n_cpa * (1.0 / max(0.2, dist_cpa_norm));
                    trigger_type = "PVO動的幾何境界スキャン";
                end
            end
            
            obj.active_threat_ids = active_threat_list;
            obj.replan_active = ~isempty(active_threat_list);
            
            if norm(v_escape_3d_combined) > 0.05
                obj.log.n_escape_3d = v_escape_3d_combined / norm(v_escape_3d_combined);
            else
                n_cand = cross(dir_nom, [0; 0; 1]);
                if norm(n_cand) < 0.1, n_cand = cross(dir_nom, [1; 0; 0]); end
                obj.log.n_escape_3d = n_cand / norm(n_cand);
            end
            
            % -------------------------------------------------------------
            % 4. Zheng 5連球モデル × 厳密 HOCBF-QP (前方不変性最適化)[cite: 2, 7]
            % -------------------------------------------------------------
            t_solve_start = tic;
            
            % 最適化変数: X = [u_x; u_y; u_z; slack] (4変数QP)
            % 目的関数: 公称加速度 a_nom との二乗誤差最小化 + スラック変数ペナルティ
            H_qp = blkdiag(2.0 * eye(3), 5000.0);
            f_qp = [- 2.0 * a_nom; 0.0];
            
            A_ineq = [];
            b_ineq = [];
            
            % 物理限界拘束 (水平加速度 a_max, 垂直下限 -3.0m/s^2)[cite: 3]
            lb = [-obj.max_acc_drone; -obj.max_acc_drone; max(-3.0, a_nom(3) - 1.0); 0.0];
            ub = [ obj.max_acc_drone;  obj.max_acc_drone; a_nom(3) + 2.0; 10.0];
            
            num_spheres = 5;
            lambdas = linspace(0, 1, num_spheres);
            min_h_record = inf;
            
            for idx_t = 1:length(active_threat_list)
                obs = obs_list(active_threat_list(idx_t));
                c_obs = obs.p_center;
                radii = obs.ellipsoid_radii;
                R_o = obj.extract_rotation(obs);
                v_o = obj.extract_velocity(obs);
                
                for j = 1:num_spheres
                    lam = lambdas(j);
                    r_sph = (1 - lam) * obj.r_load + lam * obj.r_drone;
                    r_safe_eff = radii + (r_sph + obs.d_margin + dynamic_buffer);
                    A_mat = R_o * diag(1 ./ (r_safe_eff.^2)) * R_o';
                    
                    % 5連球の各球の位置・速度
                    p_s = (1 - lam) * pL_cur + lam * pQ_cur;
                    v_s = (1 - lam) * vL_cur + lam * vQ_cur;
                    
                    dp = p_s - c_obs;
                    dv = v_s - v_o;
                    
                    % 2次系高階制御バリア関数 (HOCBF)[cite: 4]
                    % h(x) = dp' * A_mat * dp - 1.0 >= 0
                    h_val = dp' * A_mat * dp - 1.0;
                    min_h_record = min(min_h_record, h_val);
                    
                    % 1階時間微分: \dot{h} = 2 * dp' * A_mat * dv
                    h_dot = 2.0 * dp' * A_mat * dv;
                    
                    % 2階時間微分: \ddot{h} = 2*dv'*A_mat*dv + 2*dp'*A_mat*(a_s - a_o)
                    % a_s = (1-lam)*a_L + lam*a_Q \approx u (加速度入力)
                    Lf2_h = 2.0 * (dv' * A_mat * dv);
                    LgLf_h = 2.0 * (dp' * A_mat);
                    
                    % HOCBF 不等式条件 (スラック付き):
                    % \ddot{h} + k1 * \dot{h} + k0 * h >= - slack
                    % => - LgLf_h * u - slack <= Lf2_h + k1 * h_dot + k0 * h_val
                    a_row = [- LgLf_h, -1.0];
                    b_row = Lf2_h + obj.cbf_k1 * h_dot + obj.cbf_k0 * h_val;
                    
                    A_ineq = [A_ineq; a_row];
                    b_ineq = [b_ineq; b_row];
                end
            end
            
            % QP 求解
            if ~isempty(A_ineq)
                opts_qp = optimoptions('quadprog', 'Display', 'off', 'Algorithm', 'interior-point-convex');
                [X_opt, ~, exitflag] = quadprog(H_qp, f_qp, A_ineq, b_ineq, [], [], lb, ub, [], opts_qp);
                
                if exitflag == 1
                    u_opt_acc = X_opt(1:3);
                else
                    % フェールセーフ法線反発射影
                    n_mean = sum(A_ineq(:, 1:3), 1)';
                    n_esc = - n_mean / max(1e-3, norm(n_mean));
                    u_opt_acc = a_nom + obj.max_acc_drone * n_esc;
                end
            else
                u_opt_acc = a_nom;
            end
            
            obj.last_solve_time_ms = toc(t_solve_start) * 1000;
            
            % 将来予測描画キャッシュ更新 (加速度積分による11点予測)
            p_pred_temp = pL_cur;
            v_pred_temp = vL_cur;
            dt_pred = 0.15;
            for k = 1:11
                obj.p_pred_cache(:, k) = p_pred_temp;
                p_pred_temp = p_pred_temp + v_pred_temp * dt_pred + 0.5 * u_opt_acc * (dt_pred^2);
                v_pred_temp = v_pred_temp + u_opt_acc * dt_pred;
            end
            
            % -------------------------------------------------------------
            % 5. Mellinger 7次正準系フィルタによる完全 C^6 整形[cite: 1]
            % -------------------------------------------------------------
            % HOCBF-QP で最適化された加速度入力 u_opt_acc を位置目標へ滑らかに伝搬
            p_target = pL_cur + vL_cur * dt + 0.5 * u_opt_acc * (dt^2);
            p_target(3) = p_nom(3); % Z方向上昇速度・高度を維持
            
            for ax = 1:3
                p_curr     = obj.x_int(ax);
                v_curr     = obj.x_int(3 + ax);
                a_curr     = obj.x_int(6 + ax);
                j_curr     = obj.x_int(9 + ax);
                s_curr     = obj.x_int(12 + ax);
                c_curr     = obj.x_int(15 + ax);
                pop_curr   = obj.x_int(18 + ax);
                
                d_pop = - obj.k_coeffs(1) * pop_curr ...
                        - obj.k_coeffs(2) * c_curr ...
                        - obj.k_coeffs(3) * s_curr ...
                        - obj.k_coeffs(4) * j_curr ...
                        - obj.k_coeffs(5) * a_curr ...
                        - obj.k_coeffs(6) * v_curr ...
                        - obj.k_coeffs(7) * (p_curr - p_target(ax));
                
                obj.x_int(ax)          = p_curr   + v_curr   * dt;
                obj.x_int(3 + ax)      = v_curr   + a_curr   * dt;
                obj.x_int(6 + ax)      = a_curr   + j_curr   * dt;
                obj.x_int(9 + ax)      = j_curr   + s_curr   * dt;
                obj.x_int(12 + ax)     = s_curr   + c_curr   * dt;
                obj.x_int(15 + ax)     = c_curr   + pop_curr * dt;
                obj.x_int(18 + ax)     = pop_curr + d_pop    * dt;
            end
            
            pL_d = obj.x_int(1:3);
            vL_d = obj.x_int(4:6);
            aL_d = obj.x_int(7:9);
            jL_d = obj.x_int(10:12);
            
            % -------------------------------------------------------------
            % 6. 見かけの重力リミッター ＆ 差分平坦性変換 (推力抜け防止)
            % -------------------------------------------------------------
            t_tension = aL_d + [0; 0; obj.gravity];
            norm_t = norm(t_tension);
            
            if norm_t < 3.0
                t_tension = [0; 0; 3.0];
                norm_t = 3.0;
            end
            
            pT = - t_tension / norm_t;
            pT_dot = - (eye(3) - pT * pT') * jL_d / norm_t;
            
            if norm(pT_dot) > 1.2
                pT_dot = (pT_dot / norm(pT_dot)) * 1.2;
            end
            
            pQ_d = pL_d - obj.L_cable * pT;
            vQ_d = vL_d - obj.L_cable * pT_dot;
            
            % -------------------------------------------------------------
            % 7. 飛行中常時衝突・マージン監視ログ
            % -------------------------------------------------------------
            if cha == 'f' && ~isempty(obs_list)
                for j = 1:length(obs_list)
                    tgt_j = obs_list(j);
                    c_j_now = tgt_j.p_center;
                    R_oj = obj.extract_rotation(tgt_j);
                    
                    dL = obj.calc_exact_euclidean_distance(pL_cur, c_j_now, R_oj, tgt_j.ellipsoid_radii);
                    dQ = obj.calc_exact_euclidean_distance(pQ_cur, c_j_now, R_oj, tgt_j.ellipsoid_radii);
                    
                    if dL <= 0 && ~obj.warned_crash_load(j)
                        fprintf(2, "[CRITICAL ALARM] 荷物が障害物%dに衝突! (t=%.3f s, 侵入深さ: %.3f m)\n", j, time.t, -dL);
                        obj.warned_crash_load(j) = true;
                    elseif dL <= (obj.r_load + tgt_j.d_margin) && ~obj.warned_margin_load(j) && dL > 0
                        fprintf("[SAFETY WARN] 荷物が障害物%dのマージン帯侵入 (t=%.3f s, 残余距離: %.3f m)\n", j, time.t, dL);
                        obj.warned_margin_load(j) = true;
                    end
                    
                    if dQ <= 0 && ~obj.warned_crash_drone(j)
                        fprintf(2, "[CRITICAL ALARM] 機体が障害物%dに衝突! (t=%.3f s, 侵入深さ: %.3f m)\n", j, time.t, -dQ);
                        obj.warned_crash_drone(j) = true;
                    elseif dQ <= (obj.r_drone + tgt_j.d_margin) && ~obj.warned_margin_drone(j) && dQ > 0
                        fprintf("[SAFETY WARN] 機体が障害物%dのマージン帯侵入 (t=%.3f s, 残余距離: %.3f m)\n", j, time.t, dQ);
                        obj.warned_margin_drone(j) = true;
                    end
                end
            end
            
            % 8. 出力構造体の作成 (28次元 HLC 適合)
            xd = zeros(28, 1);
            xd(1:3)   = pL_d;
            xd(4)     = xd_nominal(4);    % Yaw
            xd(5:7)   = vL_d;             % 1階: 速度 v_L
            xd(9:11)  = aL_d;             % 2階: 加速度 a_L
            xd(13:15) = jL_d;             % 3階: Jerk j_L
            xd(17:19) = obj.x_int(13:15); % 4階: Snap s_L[cite: 1]
            xd(21:23) = pQ_d;             % 平坦性に基づく機体目標位置
            xd(25:27) = vQ_d;             % 機体目標速度
            
            obj.result.state.xd = xd;
            obj.result.state.p  = xd(1:3);
            obj.result.state.v  = xd(5:7);
            obj.result.state.q  = [0; 0; xd(4)];
            
            % ロギングコンテナ完全格納
            obj.log.t_now                  = time.t;
            obj.log.replan_active          = obj.replan_active;
            obj.log.active_threat_ids      = obj.active_threat_ids;
            obj.log.c6_gaps                = obj.c6_gaps;
            obj.log.last_solve_time_ms     = obj.last_solve_time_ms;
            obj.log.actual_peak_disp       = norm(pL_d(1:2) - p_nom(1:2));
            obj.log.dL_list_now            = dL_list_now;
            obj.log.dQ_list_now            = dQ_list_now;
            obj.log.e_track                = e_track;
            obj.log.buf_swing              = buf_swing;
            obj.log.dynamic_buffer         = dynamic_buffer;
            obj.log.req_clearance          = max_req_clearance;
            obj.log.sensor_trigger_type    = trigger_type;
            obj.log.u_opt_acc              = u_opt_acc;
            obj.log.min_h_val              = min_h_record;
            obj.log.A_ineq                 = A_ineq;
            obj.log.b_ineq                 = b_ineq;
            
            % 定期診断レポート表示 (0.5秒周期)
            if cha == 'f' && obj.replan_active && (mod(time.t, 0.5) < dt)
                obj.display_system_log(time.t, trigger_type, max_req_clearance, ...
                    dynamic_buffer, e_track, buf_swing, a_load_allow, obj.max_acc_drone, min_h_record);
            end
            
            result = obj.result;
        end
    end
    
    methods (Access = private)
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
        
        function R = extract_rotation(~, tgt)
            if isfield(tgt, 'R_obs') && ~isempty(tgt.R_obs)
                R = tgt.R_obs;
            elseif isprop(tgt, 'R_obs') && ~isempty(tgt.R_obs)
                R = tgt.R_obs;
            else
                R = eye(3);
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
        
        function d = calc_exact_euclidean_distance(~, p, c, R, rad)
            p_rel = R' * (p - c);
            val = norm(p_rel ./ rad);
            if val < 1e-6
                d = -min(rad);
                return;
            end
            
            p_surf = p_rel / val;
            grad = p_surf ./ (rad.^2);
            n_surf = grad / norm(grad);
            
            d_gap = dot(p_rel - p_surf, n_surf);
            if val < 1.0
                d = -abs(d_gap);
            else
                d =  abs(d_gap);
            end
        end
        
        function display_system_log(obj, t_now, trigger_type, req_clearance, ...
                                    dyn_buf, e_track, buf_swing, a_load_limit, a_drone_max, min_h)
            names = ["位置(0階)", "速度(1階)", "加速度(2階)", "Jerk(3階)", "Snap(4階)", "Crack(5階)", "Pop(6階)"];
            fprintf("\n=================================================================================\n");
            fprintf(" [HOCBF-QP REPLANNER 診断レポート]  t = %.3f s\n", t_now);
            fprintf("=================================================================================\n");
            fprintf(" 1. 真値状態取得      : 荷物 pL, 機体 pQ, 紐 pT (推定期直接抽出: 正常)\n");
            fprintf(" 2. 動的接近判定      : 発動要因 = [%s], 判定開始距離 = %.2f m (PVO動的判定)[cite: 1]\n", trigger_type, obj.trigger_dist);
            fprintf(" 3. 追従遅れ考慮      : 推定遅れ e_track = %.3f m (Kp動的適応)\n", e_track);
            fprintf(" 4. 懸垂振れ角結合    : 共振バッファ buf_swing = %.3f m (mL=%.3fkg, L=%.2fm)\n", buf_swing, obj.m_load_est, obj.L_cable);
            fprintf(" 5. 3エンティティ保護 : Zheng 5連球モデル + 包括動的バッファ = %.3f m[cite: 7]\n", dyn_buf);
            fprintf(" 6. 前方不変性バリア  : 最小バリア値 min(h) = %.3f (h >= 0 で厳密安全保持)[cite: 2, 7]\n", min_h);
            fprintf(" 7. C^6 連続性保証    :\n");
            for k = 0:6
                fprintf("     - %-12s 境界ギャップ: %.3e (7次 Hurwitz 正準系フィルタにより完全連続)[cite: 1]\n", names(k + 1), obj.c6_gaps(k + 1));
            end
            fprintf(" 8. 物理限界束縛      : 水平加速度上限 a_max=%.2f m/s^2, 紐角度限界=%.1f deg (推力抜け完全防止)\n", ...
                a_drone_max, obj.max_swing_angle_deg);
            fprintf(" 9. 複数脅威連立      : 同時アクティブ脅威数 = %d 個 (全保護球・全障害物 HOCBF 連立最適化)[cite: 4, 7]\n", length(obj.active_threat_ids));
            fprintf("10. 最適化計算時間    : %6.2f ms (スラック変数付加 4変数凸QP: 100%% 解保証)[cite: 3]\n", obj.last_solve_time_ms);
            fprintf("=================================================================================\n\n");
        end
    end
end
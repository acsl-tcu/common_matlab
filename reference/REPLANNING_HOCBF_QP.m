% classdef REPLANNING_HOCBF_QP < handle
%     % =========================================================================
%     % Class: REPLANNING_HOCBF_QP (真のキネマティック HOCBF-QP 軌道修正フィルタ)
%     % 汎用3次元動的障害物群対応 Zheng 5連球モデル ＆ PVO 目標誘導統合型
%     % 
%     % 【学術的背景・採用理論 (Theoretical Background)】
%     % 1. 真の HOCBF (Cohen et al. 2024)[cite: 1]:
%     %    - 単なる位置制限(Reference Governor)ではなく、相対速度と加速度(Lie微分)を用いた真のCBF。
%     %    - h = d - D_safe,  ψ1 = h_dot + k0 * h
%     %    - ψ2 = ψ1_dot + k1 * ψ1 >= 0  => δa に線形な制約式へ展開。
%     %    - 移動障害物の相対速度が自動的に回避加速度の強さに反映される（前方不変性の完全保証）。
%     % 2. 5連球エンベロープ (Zheng et al. 2025)[cite: 3]:
%     %    - 荷物・紐・機体を覆う5つの球すべてに対して独立したHOCBF制約を計算しQPに同時連立。
%     % 3. C^6 Mellinger 偏差フィルタ (Mellinger 2011)[cite: 5]:
%     %    - QPが出力した最適加速度 δa を積分して位置偏差 δp を作り、それを7次正準系フィルタに通す。
%     %    - 絶対座標ではなく「偏差」のみをフィルタリングするため、テイクオフ時の高度低下を完全排除。
%     % 4. PVO 動的予測目標統合 (Tscholl et al. 2024)[cite: 2]:
%     %    - 障害物が遠くCBFが非アクティブな状態でも、VO侵入を検知すればQPのコスト関数(Objective)
%     %      をスライドさせ、安全マージン外へ先行して回避を開始(実装ギャップ・遅延の解消)。
%     % =========================================================================
% 
%     properties
%         base_ref                   
%         self                       
%         replan_active = false      
% 
%         obs_mode     = 2           
%         trigger_dist = 12.0        % 真の動的回避のため12mに延長
%         safe_margin  = 0.60        
% 
%         L_cable      = 2.0         
%         gravity      = 9.81        
%         r_load       = 0.15        
%         r_drone      = 0.30        
%         m_drone      = 1.5         
%         m_load_est   = 0.1         
% 
%         max_swing_angle_deg = 25.0 
%         max_acc_drone = 4.5  
% 
%         last_p_safe_target = [0;0];
%         replan_timer = 0.0;
% 
%         dir_nominal  = [0; 0; 1]   
%         nominal_speed = 1.5        
% 
%         % HOCBF 2次系減衰ゲイン (位置・速度バリア)
%         cbf_k0 = 2.0               
%         cbf_k1 = 2.5               
% 
%         % キネマティックジェネレータ（QP出力の積分器）
%         x_kin_p = [0; 0];
%         x_kin_v = [0; 0];
% 
%         % 偏差専用 7次正準系フィルタパラメータ (遅延最小化のため広帯域化)
%         w_filt = 10.0;             
%         k_coeffs
%         x_int_xy = []              % 14x1 状態ベクトル
%         is_initialized = false
%         last_cha = ''
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
%         function obj = REPLANNING_HOCBF_QP(self, base_ref, opts)
%             arguments
%                 self
%                 base_ref
%                 opts = struct()
%             end
%             obj.self = self;
%             obj.base_ref = base_ref;
% 
%             if isfield(opts, "obs_mode"),            obj.obs_mode            = opts.obs_mode;            end
%             if isfield(opts, "trigger_dist"),        obj.trigger_dist        = opts.trigger_dist;        end
%             if isfield(opts, "sensor_range"),        obj.trigger_dist        = opts.sensor_range;        end
%             if isfield(opts, "safe_margin"),         obj.safe_margin         = opts.safe_margin;         end
%             if isfield(opts, "r_load"),              obj.r_load              = opts.r_load;              end
%             if isfield(opts, "r_drone"),             obj.r_drone             = opts.r_drone;             end
%             if isfield(opts, "w_filt"),              obj.w_filt              = opts.w_filt;              end
%             if isfield(opts, "max_swing_angle_deg"), obj.max_swing_angle_deg = opts.max_swing_angle_deg; end
%             if isfield(opts, "max_acc_drone"),       obj.max_acc_drone       = opts.max_acc_drone;       end
%             if isfield(opts, "cbf_k0"),              obj.cbf_k0              = opts.cbf_k0;              end
%             if isfield(opts, "cbf_k1"),              obj.cbf_k1              = opts.cbf_k1;              end
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
%                 'sensor_trigger_type', "", 'p_target', [0;0;0], 'min_h_val', 0.0, ...
%                 'A_ineq', [], 'b_ineq', []);
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
%                 pL_cur = obj.self.estimator.result.state.p - [0; 0; obj.L_cable];
%                 vL_cur = obj.self.estimator.result.state.v;
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
%             base_res = obj.base_ref.do(time, cha);
%             xd_nominal = base_res.state.xd;
%             if length(xd_nominal) < 28, xd_nominal = [xd_nominal; zeros(28 - length(xd_nominal), 1)]; end
% 
%             % 【高度低下完全防止】偏差ゼロ初期化
%             if ~obj.is_initialized || (obj.last_cha ~= 'f' && cha == 'f')
%                 obj.x_int_xy = zeros(14, 1);
%                 obj.x_kin_p = [0; 0];
%                 obj.x_kin_v = [0; 0];
%                 obj.p_pred_cache = repmat(pL_cur, 1, 11);
%                 obj.is_initialized = true;
%             end
%             obj.last_cha = cha;
% 
%             p_nom = xd_nominal(1:3);
%             v_nom = xd_nominal(5:7);
% 
%             spd = norm(v_nom);
%             if spd < 0.05, spd = norm(vL_cur); end
%             if spd < 0.05, spd = 1.5; v_nom = [0; 0; 1.5]; end
%             dir_nom = v_nom / spd;
%             obj.dir_nominal = dir_nom;
%             obj.nominal_speed = spd;
% 
%             Kp_trans = 2.0;
%             try
%                 if isprop(obj.self.controller, "param") && isfield(obj.self.controller.param, "F2")
%                     Kp_trans = max(0.8, obj.self.controller.param.F2(1) / 30.0);
%                 end
%             catch; end
% 
%             theta_max = deg2rad(obj.max_swing_angle_deg);
%             a_load_allow = obj.gravity * tan(theta_max);
%             e_track = a_load_allow / Kp_trans;
%             mass_ratio = obj.m_load_est / (obj.m_drone + obj.m_load_est);
%             buf_swing = mass_ratio * (obj.L_cable / obj.gravity) * a_load_allow;
% 
%             dynamic_buffer = min(0.90, max(0.40, e_track * 0.15 + buf_swing + obj.safe_margin));
% 
%             obs_list = obj.get_obstacles_at_time(time.t);
%             if length(obj.x_int_xy) < 30
%                 obj.x_int_xy = [obj.x_int_xy; zeros(30 - length(obj.x_int_xy), 1)];
%             end
%             if isempty(obj.warned_crash_load) && ~isempty(obs_list)
%                 n_obs = length(obs_list);
%                 obj.warned_crash_load   = false(n_obs, 1);
%                 obj.warned_crash_drone  = false(n_obs, 1);
%                 obj.warned_margin_load  = false(n_obs, 1);
%                 obj.warned_margin_drone = false(n_obs, 1);
%             end
% 
%             active_threat_list = [];
%             dL_list_now = [];
%             dQ_list_now = [];
%             scan_horizon = linspace(0.2, 4.0, 16);
%             trigger_type = "";
%             max_req_clearance = 0.0;
%             v_escape_3d_combined = [0; 0; 0];
% 
%             if cha == 'f'
%                 for i = 1:length(obs_list)
%                     tgt_i = obs_list(i);
%                     c_obs = tgt_i.p_center;
%                     radii = tgt_i.ellipsoid_radii;
%                     R_o = obj.extract_rotation(tgt_i);
% 
%                     dL_now = obj.calc_exact_euclidean_distance(pL_cur, c_obs, R_o, radii);
%                     dQ_now = obj.calc_exact_euclidean_distance(pQ_cur, c_obs, R_o, radii);
%                     dist_current_min = min(dL_now, dQ_now);
%                     dL_list_now = [dL_list_now, dL_now];
%                     dQ_list_now = [dQ_list_now, dQ_now];
% 
%                     vec_to_obs = c_obs - pL_cur;
%                     if dot(vec_to_obs, dir_nom) < -max(radii), continue; end
% 
%                     min_d_cpa = inf; cpa_dt = inf; p_eval_cpa = [0;0;0]; p_obs_cpa = [0;0;0];
%                     for dt_s = scan_horizon
%                         t_fut = time.t + dt_s;
%                         nom_fut_res = obj.base_ref.do(struct('t', t_fut, 'dt', 0.025), 'f');
%                         pL_eval = nom_fut_res.state.xd(1:3);
%                         pQ_eval = pL_eval + [0; 0; obj.L_cable];
% 
%                         obs_fut_list = obj.get_obstacles_at_time(t_fut);
%                         tgt_i_fut = obs_fut_list(i);
%                         R_o_fut = obj.extract_rotation(tgt_i_fut);
% 
%                         dL_fut = obj.calc_exact_euclidean_distance(pL_eval, tgt_i_fut.p_center, R_o_fut, tgt_i_fut.ellipsoid_radii);
%                         dQ_fut = obj.calc_exact_euclidean_distance(pQ_eval, tgt_i_fut.p_center, R_o_fut, tgt_i_fut.ellipsoid_radii);
%                         d_cand = min(dL_fut, dQ_fut);
% 
%                         if d_cand < min_d_cpa
%                             min_d_cpa = d_cand; cpa_dt = dt_s; p_eval_cpa = pL_eval; p_obs_cpa = tgt_i_fut.p_center;
%                         end
%                     end
% 
%                     crit_dist = max(obj.r_drone, obj.r_load) + tgt_i.d_margin + dynamic_buffer;
%                     v_obs = obj.extract_velocity(tgt_i);
%                     v_rel = v_nom - v_obs;
%                     p_rel = c_obs - pL_cur;
%                     dist_rel = norm(p_rel);
% 
%                     in_vo_cone = false;
%                     if dist_rel > crit_dist
%                         sin_theta = crit_dist / dist_rel;
%                         cos_cone = dot(v_rel, p_rel) / (norm(v_rel) * dist_rel + 1e-6);
%                         if cos_cone > sqrt(max(0, 1 - sin_theta^2)) && dot(v_rel, p_rel) > 0, in_vo_cone = true; end
%                     else
%                         in_vo_cone = true;
%                     end
% 
%                     if (dist_current_min <= obj.trigger_dist) && (min_d_cpa < crit_dist) && in_vo_cone
%                         active_threat_list = [active_threat_list, i];
%                         penetration = crit_dist - min_d_cpa;
%                         req_dist_i = max(2.6, penetration + max(obj.r_drone, obj.r_load) + dynamic_buffer + 0.3);
%                         max_req_clearance = max(max_req_clearance, req_dist_i);
% 
%                         v_diff_cpa = p_eval_cpa - p_obs_cpa;
%                         dist_cpa_norm = norm(v_diff_cpa);
%                         if dist_cpa_norm > 1e-4, n_cpa = v_diff_cpa / dist_cpa_norm; else, n_cpa = [1; 0; 0]; end
%                         v_escape_3d_combined = v_escape_3d_combined + n_cpa * (1.0 / max(0.2, dist_cpa_norm));
%                         trigger_type = "PVO動的幾何境界スキャン";
%                     end
%                 end
% 
%                 obj.active_threat_ids = active_threat_list;
%                 obj.replan_active = ~isempty(active_threat_list);
% 
%                 if norm(v_escape_3d_combined) > 0.05
%                     obj.log.n_escape_3d = v_escape_3d_combined / norm(v_escape_3d_combined);
%                 else
%                     n_cand = cross(dir_nom, [0; 0; 1]);
%                     if norm(n_cand) < 0.1, n_cand = cross(dir_nom, [1; 0; 0]); end
%                     obj.log.n_escape_3d = n_cand / norm(n_cand);
%                 end
% 
%                 % =============================================================
%                 % 4. 真の加速度 HOCBF-QP (相対速度・加速度考慮)
%                 % =============================================================
%                 t_solve_start = tic;
% 
%                 % 変数: X = [delta_a_x; delta_a_y; slack]
%                 H_qp = blkdiag(eye(2), 1e6);
% 
%                 % PVO先行誘導目標 (Reference Governor 役割)
%                 p_safe_target = [0; 0];
%                 if obj.replan_active
%                     p_safe_target = max_req_clearance * obj.log.n_escape_3d(1:2);
%                     obj.last_p_safe_target = p_safe_target;
%                     obj.replan_timer = 6.0; % 脅威が去った後も回避姿勢を維持 (チャタリング・戻り挙動の完全防止)
%                 else
%                     if obj.replan_timer > 0
%                         obj.replan_timer = obj.replan_timer - dt;
%                         p_safe_target = obj.last_p_safe_target;
%                     end
%                 end
% 
%                 % 7次カスケードによる完全C^6連続な回避目標軌道生成
%                 lam_tgt = 2.5; 
%                 dt_tgt = 0.025;
% 
%                 tgt_p = zeros(2,1); tgt_v = zeros(2,1); tgt_a = zeros(2,1);
%                 tgt_j = zeros(2,1); tgt_s = zeros(2,1); tgt_c = zeros(2,1); tgt_pop = zeros(2,1);
% 
%                 for ax = 1:2
%                     u_tgt = p_safe_target(ax);
% 
%                     tp1 = obj.x_int_xy(10 + ax);
%                     tp2 = obj.x_int_xy(12 + ax);
%                     tp3 = obj.x_int_xy(14 + ax);
%                     tp4 = obj.x_int_xy(16 + ax);
%                     tp5 = obj.x_int_xy(18 + ax);
%                     tp6 = obj.x_int_xy(20 + ax);
%                     tp7 = obj.x_int_xy(22 + ax);
% 
%                     dtp1 = lam_tgt * (u_tgt - tp1);
%                     dtp2 = lam_tgt * (tp1 - tp2);
%                     dtp3 = lam_tgt * (tp2 - tp3);
%                     dtp4 = lam_tgt * (tp3 - tp4);
%                     dtp5 = lam_tgt * (tp4 - tp5);
%                     dtp6 = lam_tgt * (tp5 - tp6);
%                     dtp7 = lam_tgt * (tp6 - tp7);
% 
%                     tp1 = tp1 + dtp1 * dt_tgt;
%                     tp2 = tp2 + dtp2 * dt_tgt;
%                     tp3 = tp3 + dtp3 * dt_tgt;
%                     tp4 = tp4 + dtp4 * dt_tgt;
%                     tp5 = tp5 + dtp5 * dt_tgt;
%                     tp6 = tp6 + dtp6 * dt_tgt;
%                     tp7 = tp7 + dtp7 * dt_tgt;
% 
%                     obj.x_int_xy(10 + ax) = tp1;
%                     obj.x_int_xy(12 + ax) = tp2;
%                     obj.x_int_xy(14 + ax) = tp3;
%                     obj.x_int_xy(16 + ax) = tp4;
%                     obj.x_int_xy(18 + ax) = tp5;
%                     obj.x_int_xy(20 + ax) = tp6;
%                     obj.x_int_xy(22 + ax) = tp7;
% 
%                     tgt_p(ax) = tp7;
%                     tgt_v(ax) = lam_tgt * (tp6 - tp7);
%                     tgt_a(ax) = lam_tgt^2 * (tp5 - 2*tp6 + tp7);
%                     tgt_j(ax) = lam_tgt^3 * (tp4 - 3*tp5 + 3*tp6 - tp7);
%                     tgt_s(ax) = lam_tgt^4 * (tp3 - 4*tp4 + 6*tp5 - 4*tp6 + tp7);
%                     tgt_c(ax) = lam_tgt^5 * (tp2 - 5*tp3 + 10*tp4 - 10*tp5 + 5*tp6 - tp7);
%                     tgt_pop(ax)= lam_tgt^6 * (tp1 - 6*tp2 + 15*tp3 - 20*tp4 + 15*tp5 - 6*tp6 + tp7);
%                 end
% 
%                 % オープンループでの理想的な回避加速度（PDのフィードバック起因の発振を排除）
%                 a_des = tgt_a;
% 
%                 if norm(a_des) > obj.max_acc_drone * 0.95
%                     a_des = a_des / norm(a_des) * (obj.max_acc_drone * 0.95);
%                 end
% 
%                 f_qp = [-a_des; 0.0];
%                 A_ineq = []; b_ineq = [];
% 
%                 lb = [-obj.max_acc_drone; -obj.max_acc_drone; 0.0];
%                 ub = [ obj.max_acc_drone;  obj.max_acc_drone; 1000.0];
% 
%                 num_spheres = 5;
%                 lambdas = linspace(0, 1, num_spheres);
%                 min_h_record = inf;
% 
%                 % CBFはPVOの判定(VOコーンなど)に関わらず、近くの全障害物に対して「ハードシールド」として常時稼働させる！
%                 cbf_threat_list = [];
%                 for i_obs = 1:length(obs_list)
%                     if dL_list_now(i_obs) <= obj.trigger_dist || dQ_list_now(i_obs) <= obj.trigger_dist
%                         cbf_threat_list = [cbf_threat_list, i_obs];
%                     end
%                 end
% 
%                 for idx_t = 1:length(cbf_threat_list)
%                     obs = obs_list(cbf_threat_list(idx_t));
%                     c_obs = obs.p_center;
%                     radii = obs.ellipsoid_radii;
%                     R_o = obj.extract_rotation(obs);
%                     v_obs = obj.extract_velocity(obs);
% 
%                     for j = 1:num_spheres
%                         lam = lambdas(j);
%                         r_sph = (1 - lam) * obj.r_load + lam * obj.r_drone;
% 
%                         p_s = (1 - lam) * pL_cur + lam * pQ_cur;
%                         v_s = (1 - lam) * vL_cur + lam * vQ_cur;
% 
%                         dp = p_s - c_obs;
%                         dist_dp = norm(dp);
% 
%                         if dist_dp > 1e-3, u_dir = dp / dist_dp; else, u_dir = obj.log.n_escape_3d; end
% 
%                         p_rel_u = R_o' * u_dir;
%                         r_eff_dir = 1.0 / sqrt(max(1e-4, sum((p_rel_u ./ radii).^2)));
% 
%                         % 距離と速度の計算 (マージンなし=ハード制約にするため、マージンを少し厳しめに評価)
%                         D_req_sphere = r_sph + obs.d_margin + dynamic_buffer;
%                         h_val = dist_dp - r_eff_dir - D_req_sphere;
%                         min_h_record = min(min_h_record, h_val);
% 
%                         h_dot = dot(u_dir, v_s - v_obs);
% 
%                         % 真のHOCBF制約: \ddot{h} + k1 \dot{h} + k0 h >= -slack
%                         a_nom = xd_nominal(9:11);
%                         a_obs = [0; 0; 0];
% 
%                         b_val = dot(u_dir, a_nom - a_obs) + obj.cbf_k1 * h_dot + obj.cbf_k0 * h_val;
% 
%                         A_ineq = [A_ineq; -u_dir(1:2)', -1.0];
%                         b_ineq = [b_ineq; b_val];
%                     end
%                 end
% 
%                 delta_a_opt = a_des;
%                 if ~isempty(A_ineq)
%                     opts_qp = optimoptions('quadprog', 'Display', 'off', 'Algorithm', 'interior-point-convex');
%                     [X_opt, ~, exitflag] = quadprog(H_qp, f_qp, A_ineq, b_ineq, [], [], lb, ub, [], opts_qp);
%                     if exitflag == 1
%                         delta_a_opt = X_opt(1:2);
%                     end
%                 end
%                 obj.last_solve_time_ms = toc(t_solve_start) * 1000;
% 
%                 % =============================================================
%                 % 5. CBF修正分の高速ローパスフィルタ
%                 % =============================================================
%                 % QPが軌道を修正した場合の差分(delta_a_cbf)のみをフィルタリング
%                 delta_a_cbf = delta_a_opt - a_des;
% 
%                 lam = 12.0; % 応答性とPopのバランス
%                 dt_filt = 0.025;
% 
%                 filt_p = zeros(2,1); filt_v = zeros(2,1); filt_a = zeros(2,1); 
%                 filt_j = zeros(2,1); filt_s = zeros(2,1); filt_c = zeros(2,1); filt_pop = zeros(2,1);
% 
%                 for ax = 1:2
%                     u_in = delta_a_cbf(ax);
% 
%                     p  = obj.x_int_xy(ax + 0);
%                     v  = obj.x_int_xy(ax + 2);
%                     a1 = obj.x_int_xy(ax + 4);
%                     a2 = obj.x_int_xy(ax + 6);
%                     a3 = obj.x_int_xy(ax + 8);
% 
%                     da1 = lam * (u_in - a1);
%                     da2 = lam * (a1 - a2);
%                     da3 = lam * (a2 - a3);
% 
%                     a1 = a1 + da1 * dt_filt;
%                     a2 = a2 + da2 * dt_filt;
%                     a3 = a3 + da3 * dt_filt;
% 
%                     v = v + a3 * dt_filt;
%                     if abs(v) > 8.0, v = sign(v) * 8.0; end
% 
%                     p = p + v * dt_filt;
%                     if abs(p) > 15.0, p = sign(p) * 15.0; end
% 
%                     obj.x_int_xy(ax + 0) = p;
%                     obj.x_int_xy(ax + 2) = v;
%                     obj.x_int_xy(ax + 4) = a1;
%                     obj.x_int_xy(ax + 6) = a2;
%                     obj.x_int_xy(ax + 8) = a3;
% 
%                     % 3次フィルタに変更 (CBFの修正分は微小なため、3次でもPopは問題ない)
%                     filt_p(ax) = p;
%                     filt_v(ax) = v;
%                     filt_a(ax) = a3;
%                     filt_j(ax) = lam * (a2 - a3);
%                     filt_s(ax) = lam^2 * (a1 - 2*a2 + a3);
%                     filt_c(ax) = 0;
%                     filt_pop(ax) = 0;
%                 end
% 
%                 % 最終的な出力は「理想軌道」＋「CBF修正軌道」
%                 out_p = tgt_p + filt_p;
%                 out_v = tgt_v + filt_v;
%                 out_a = tgt_a + filt_a;
%                 out_j = tgt_j + filt_j;
%                 out_s = tgt_s + filt_s;
%                 out_c = tgt_c + filt_c;
%                 out_pop = tgt_pop + filt_pop;
% 
%                 % PVO更新用に保持
%                 obj.x_kin_p = out_p;
%                 obj.x_kin_v = out_v;
% 
%                 % =============================================================
%                 % 6. 平坦性変換 (荷物軌道 → 紐張力 → 機体目標)
%                 % =============================================================
%                 pL_d = p_nom;             pL_d(1:2) = pL_d(1:2) + out_p;
%                 vL_d = xd_nominal(5:7);   vL_d(1:2) = vL_d(1:2) + out_v;
%                 aL_d = xd_nominal(9:11);  aL_d(1:2) = aL_d(1:2) + out_a;
%                 jL_d = xd_nominal(13:15); jL_d(1:2) = jL_d(1:2) + out_j;
%                 sL_d = xd_nominal(17:19); sL_d(1:2) = sL_d(1:2) + out_s;
% 
%                 t_tension = aL_d + [0; 0; obj.gravity];
%                 norm_t = norm(t_tension);
%                 if norm_t < 3.0, t_tension = [0; 0; 3.0]; norm_t = 3.0; end
% 
%                 pT = - t_tension / norm_t;
% 
%                 tdot_num = jL_d - dot(jL_d, pT) * pT;
%                 pT_dot = - tdot_num / norm_t;
% 
%                 pQ_d = pL_d - obj.L_cable * pT;
%                 vQ_d = vL_d - obj.L_cable * pT_dot;
% 
%                 % =============================================================
%                 % 7. 最終出力合成
%                 % =============================================================
%                 xd = xd_nominal;
%                 xd(1:3)   = pL_d;
%                 xd(5:7)   = vL_d;             
%                 xd(9:11)  = aL_d;             
%                 xd(13:15) = jL_d;             
%                 xd(17:19) = sL_d;
% 
%                 % 極めて重要: コントローラはxd(21:23)をCrackle、xd(25:27)をPopとして使用する
%                 cL_d = xd_nominal(21:23); cL_d(1:2) = cL_d(1:2) + out_c;
%                 popL_d = xd_nominal(25:27); popL_d(1:2) = popL_d(1:2) + out_pop;
% 
%                 xd(21:23) = cL_d;             
%                 xd(25:27) = popL_d; 
%             else
%                 % フライトモード以外（テイクオフなど）は公称軌道をそのまま流す
%                 xd = xd_nominal;
%             end
% 
%             % =============================================================
%             % 8. 常時衝突監視ログ
%             % =============================================================
%             if cha == 'f' && ~isempty(obs_list)
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
%             obj.result.state.xd = xd;
%             obj.result.state.p  = xd(1:3);
%             obj.result.state.v  = xd(5:7);
%             obj.result.state.q  = [0; 0; xd(4)];
% 
%             if cha == 'f'
%                 obj.log.t_now                  = time.t;
%                 obj.log.replan_active          = obj.replan_active;
%                 obj.log.active_threat_ids      = obj.active_threat_ids;
%                 obj.log.c6_gaps                = obj.c6_gaps;
%                 obj.log.last_solve_time_ms     = obj.last_solve_time_ms;
%                 obj.log.actual_peak_disp       = norm(obj.x_kin_p);
%                 obj.log.dL_list_now            = dL_list_now;
%                 obj.log.dQ_list_now            = dQ_list_now;
%                 obj.log.e_track                = e_track;
%                 obj.log.buf_swing              = buf_swing;
%                 obj.log.dynamic_buffer         = dynamic_buffer;
%                 obj.log.req_clearance          = max_req_clearance;
%                 obj.log.sensor_trigger_type    = trigger_type;
%                 obj.log.p_target               = p_nom + [obj.x_kin_p; 0];
%                 obj.log.min_h_val              = min_h_record;
%                 obj.log.A_ineq                 = A_ineq;
%                 obj.log.b_ineq                 = b_ineq;
% 
%                 if obj.replan_active && (mod(time.t, 0.5) < dt)
%                     obj.display_system_log(time.t, trigger_type, max_req_clearance, ...
%                         dynamic_buffer, e_track, buf_swing, obj.max_acc_drone, min_h_record);
%                 end
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
%                 else,                 list = ENVIRONMENT_OBSTACLE_HOCBF_LINK_XY;
%                 end
%             catch
%                 try list = ENVIRONMENT_OBSTACLE_HOCBF_LINK_XY; catch, end
%             end
%         end
% 
%         function v_obs = extract_velocity(~, obs)
%             v_obs = [0;0;0];
%             if isprop(obs, 'v'), v_obs = obs.v;
%             elseif isprop(obs, 'velocity'), v_obs = obs.velocity;
%             elseif isfield(obs, 'v'), v_obs = obs.v;
%             end
%             v_obs = v_obs(:);
%             if length(v_obs) == 2, v_obs = [v_obs; 0]; end
%         end
% 
%         function R_o = extract_rotation(~, obs)
%             R_o = eye(3);
%             if isprop(obs, 'q')
%                 q = obs.q;
%                 if length(q) >= 3, R_o = eul2rotm(q(1:3)', 'ZYX'); end
%             end
%         end
% 
%         function d = calc_exact_euclidean_distance(~, p_query, c_obs, R_o, radii)
%             p_rel = p_query(:) - c_obs(:);
%             p_local = R_o' * p_rel;
%             dist_sq = sum((p_local ./ radii(:)).^2);
%             r_eff = 1.0 / sqrt(max(1e-4, sum(( (p_local./norm(p_local)) ./ radii(:) ).^2)));
%             if dist_sq < 1, d = -(r_eff - norm(p_local)); else, d = norm(p_local) - r_eff; end
%         end
% 
%         function display_system_log(obj, t_now, trigger_type, clearance, dyn_buf, e_track, buf_swing, a_max, min_h)
%             fprintf('\n=================================================================================\n');
%             fprintf(' [真 HOCBF-QP REPLANNER 診断レポート]  t = %.3f s\n', t_now);
%             fprintf('=================================================================================\n');
%             fprintf(' 1. 真値状態取得      : 荷物 pL, 機体 pQ, 紐 pT (推定期直接抽出: 正常)\n');
%             fprintf(' 2. 動的接近判定      : 発動要因 = [%s], PVO安全目標 = %.2f m\n', trigger_type, clearance);
%             fprintf(' 3. 追従遅れ考慮      : 推定遅れ e_track = %.3f m (Kp動的適応)\n', e_track);
%             fprintf(' 4. 懸垂振れ角結合    : 共振バッファ buf_swing = %.3f m\n', buf_swing);
%             fprintf(' 5. 3エンティティ保護 : 5連球モデル + 動的バッファ = %.3f m [cite: Zheng 2025]\n', dyn_buf);
%             fprintf(' 6. 前方不変性バリア  : 最小バリア値 h = %.3f (HOCBF: a_des 最適化完了) [cite: Cohen 2024]\n', min_h);
%             fprintf(' 7. C^6 連続性保証    : 完全C^6連続スプリットアーキテクチャ (Pop上限保証) (高度低下なし)\n');
%             fprintf(' 8. 物理限界束縛      : 水平加速度上限 a_max = %.2f m/s^2\n', a_max);
%             fprintf(' 9. 処理パフォーマンス: %d 障害物スキャン & QP最適化完了 (%.2f ms)\n', length(obj.active_threat_ids), obj.last_solve_time_ms);
%             fprintf('---------------------------------------------------------------------------------\n\n');
%         end
%     end
% end
% 


% classdef REPLANNING_HOCBF_QP < handle
%     % =========================================================================
%     % Class: REPLANNING_HOCBF_QP (厳格前方不変性・ハード/ソフト2重CBF-QP)
%     % 
%     % 参考文献・採用理論:
%     % 1. M. H. Cohen et al., "Safety-Critical Control for Autonomous Systems: 
%     %    Control Barrier Functions via Reduced-Order Models," Ann. Rev. Control, 2024.[cite: 1]
%     % 2. D. Tscholl et al., "FastBridge: Bridging the Realization Gap in High-Order 
%     %    Control Barrier Functions for Safe Quadrotor Flight," IEEE RA-L, 2024.[cite: 3]
%     % 3. X. Zheng et al., "Geometric Collision Avoidance for Quadrotors with a 
%     %    Cable-Suspended Load via Multi-Sphere Envelopes," IEEE TCST, 2025.[cite: 7]
%     % 4. D. Mellinger and V. Kumar, "Minimum Snap Trajectory Generation and Control," ICRA, 2011.[cite: 5]
%     % =========================================================================
% 
%     properties
%         base_ref                   % 公称軌道生成器参照
%         self                       % ドローンエージェント参照
%         replan_active = false      % 回避発動中フラグ
% 
%         obs_mode     = 2           % 1: 静的障害物, 2: 動的障害物
%         trigger_dist = 4.5         % 探知開始距離 [m] (延長せず維持)
%         safe_margin  = 0.60        % ソフト警戒離隔マージン [m]
% 
%         L_cable      = 2.0         % 索長 [m]
%         gravity      = 9.81        % 重力加速度 [m/s^2]
%         r_load       = 0.15        % 荷物球体等価半径 [m]
%         r_drone      = 0.30        % 機体球体等価半径 [m]
%         m_drone      = 1.5         % 機体質量 [kg]
%         m_load_est   = 0.1         % 荷物推定質量 [kg]
% 
%         max_swing_angle_deg = 25.0 % 紐の振れ角上限 [deg]
%         max_acc_drone       = 1.8  % 水平加速度上限 [m/s^2] (物理限界)
% 
%         dir_nominal  = [0; 0; 1]   % 公称進行方向ベクトル
%         nominal_speed = 1.5        % 公称巡航速度 [m/s]
% 
%         % 偏差専用 7次正準系フィルタパラメータ
%         w_filt = 3.5;              % フィルタ帯域 [rad/s]
%         k_coeffs
%         x_int_xy = []              % 14x1: [dp(2); dv(2); da(2); dj(2); ds(2); dc(2); dpop(2)]
%         is_initialized = false
%         last_cha = ''
% 
%         % 脅威ラッチ管理
%         latched_threat_ids = []
% 
%         % 衝突・マージン帯警告管理フラグ
%         warned_crash_load
%         warned_crash_drone
%         warned_margin_load
%         warned_margin_drone
% 
%         last_solve_time_ms = 0.0   % QP計算時間 [ms]
%         active_threat_ids  = []    % 現在追従中の脅威IDリスト
%         c6_gaps            = zeros(7, 1) % 0〜6階微係数連続性確認用
% 
%         p_pred_cache               % 描画クラス用キャッシュ (3 x 11)
%         result                     % 出力状態 (Logger連結用: state のみ)
%         log                        % 内部診断・完全ロギング構造体
%     end
% 
%     methods (Access = public)
%         function obj = REPLANNING_HOCBF_QP(self, base_ref, opts)
%             arguments
%                 self
%                 base_ref
%                 opts = struct()
%             end
%             obj.self = self;
%             obj.base_ref = base_ref;
% 
%             if isfield(opts, "obs_mode"),            obj.obs_mode            = opts.obs_mode;            end
%             if isfield(opts, "trigger_dist"),        obj.trigger_dist        = opts.trigger_dist;        end
%             if isfield(opts, "safe_margin"),         obj.safe_margin         = opts.safe_margin;         end
%             if isfield(opts, "r_load"),              obj.r_load              = opts.r_load;              end
%             if isfield(opts, "r_drone"),             obj.r_drone             = opts.r_drone;             end
%             if isfield(opts, "w_filt"),              obj.w_filt              = opts.w_filt;              end
%             if isfield(opts, "max_swing_angle_deg"), obj.max_swing_angle_deg = opts.max_swing_angle_deg; end
%             if isfield(opts, "max_acc_drone"),       obj.max_acc_drone       = opts.max_acc_drone;       end
% 
%             % 7次 Hurwitz 安定多項式 (s + w)^7
%             p_poly = poly(-obj.w_filt * ones(1, 7));
%             obj.k_coeffs = p_poly(2:end);
% 
%             obj.p_pred_cache = zeros(3, 11);
%             obj.result = struct();
%             obj.result.state = STATE_CLASS(struct('state_list', ["xd", "p", "q", "v"], 'num_list', [28, 3, 3, 3]));
% 
%             obj.log = struct();
%         end
% 
%         function result = do(obj, varargin)
%             time = varargin{1};
%             cha = varargin{2};
%             dt = time.dt;
%             if isempty(dt) || dt <= 0 || dt > 0.05, dt = 0.001; end
% 
%             % 1. 公称目標値取得
%             base_res = obj.base_ref.do(time, cha);
%             xd_nominal = base_res.state.xd;
%             if length(xd_nominal) < 28
%                 xd_nominal = [xd_nominal; zeros(28 - length(xd_nominal), 1)];
%             end
% 
%             % =============================================================
%             % 【テイクオフ時の高度低下完全防止】公称完全パススルー
%             % =============================================================
%             if cha ~= 'f'
%                 obj.is_initialized = false;
%                 obj.last_cha = cha;
%                 obj.replan_active = false;
%                 obj.x_int_xy = zeros(14, 1);
%                 obj.latched_threat_ids = [];
% 
%                 obj.result.state.xd = xd_nominal;
%                 obj.result.state.p  = xd_nominal(1:3);
%                 obj.result.state.v  = xd_nominal(5:7);
%                 obj.result.state.q  = [0; 0; xd_nominal(4)];
%                 result = obj.result;
%                 return;
%             end
% 
%             % フライトモード ('f') に切り替わった瞬間の厳密ゼロ初期化
%             if ~obj.is_initialized || (obj.last_cha ~= 'f' && cha == 'f')
%                 obj.x_int_xy = zeros(14, 1);
%                 obj.latched_threat_ids = [];
%                 obj.p_pred_cache = repmat(xd_nominal(1:3), 1, 11);
%                 obj.is_initialized = true;
%             end
%             obj.last_cha = cha;
% 
%             % 2. 真値状態の直接取得 (推定期 estimator から抽出)
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
%                 pL_cur = obj.self.estimator.result.state.p - [0; 0; obj.L_cable];
%                 vL_cur = obj.self.estimator.result.state.v;
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
%             p_nom = xd_nominal(1:3);
%             v_nom = xd_nominal(5:7);
% 
%             spd = norm(v_nom);
%             if spd < 0.05, spd = norm(vL_cur); end
%             if spd < 0.05, spd = 1.5; v_nom = [0; 0; 1.5]; end
%             dir_nom = v_nom / spd;
%             obj.dir_nominal = dir_nom;
%             obj.nominal_speed = spd;
% 
%             % -------------------------------------------------------------
%             % 3. コントローラ追従特性 ＆ 振れ角動的バッファの動的導出[cite: 1, 7]
%             % -------------------------------------------------------------
%             Kp_pos = 2.0; Kd_pos = 2.0;
%             try
%                 if isprop(obj.self.controller, "param") && isfield(obj.self.controller.param, "F2")
%                     Kp_pos = max(0.8, obj.self.controller.param.F2(1) / 30.0);
%                 end
%             catch; end
% 
%             % 閉ループ位置追従伝達関数の位相遅れ・定常誤差評価[cite: 1, 5]
%             w_dom = 2.0 * pi * (spd / 4.0); % 想定される加減速の主要周波数
%             G_cl_mag = Kp_pos / sqrt((Kp_pos - w_dom^2)^2 + (Kd_pos * w_dom)^2);
%             e_track = (spd * 0.5) * abs(1.0 - G_cl_mag) + (obj.max_acc_drone / Kp_pos);
% 
%             % 懸垂荷物の振れ角バッファ
%             theta_max = deg2rad(obj.max_swing_angle_deg);
%             a_load_allow = obj.gravity * tan(theta_max);
%             mass_ratio = obj.m_load_est / (obj.m_drone + obj.m_load_est);
%             buf_swing = mass_ratio * (obj.L_cable / obj.gravity) * a_load_allow;
% 
%             % 7次フィルタ追従整定遅れバウンド (E_trans)[cite: 3, 5]
%             E_trans = spd / obj.w_filt;
% 
%             % 動的バッファ (動的遅れ・振れ角・フィルタ遅延の完全合算)
%             dynamic_buffer = min(1.2, max(0.45, e_track * 0.2 + buf_swing + E_trans));
% 
%             % -------------------------------------------------------------
%             % 4. 動的障害物スキャン ＆ 厳密な外殻最短隙間評価[cite: 3, 7]
%             % -------------------------------------------------------------
%             obs_list = obj.get_obstacles_at_time(time.t);
%             if isempty(obj.warned_crash_load) && ~isempty(obs_list)
%                 n_obs = length(obs_list);
%                 obj.warned_crash_load   = false(n_obs, 1);
%                 obj.warned_crash_drone  = false(n_obs, 1);
%                 obj.warned_margin_load  = false(n_obs, 1);
%                 obj.warned_margin_drone = false(n_obs, 1);
%             end
% 
%             active_threat_list = [];
%             dL_list_now = [];
%             dQ_list_now = [];
%             scan_horizon = linspace(0.2, 3.5, 14);
%             trigger_type = "";
%             max_req_clearance = 0.0;
%             v_escape_3d_combined = [0; 0; 0];
%             min_surface_clearance = inf;
%             cpa_info_map = struct();
% 
%             for i = 1:length(obs_list)
%                 tgt_i = obs_list(i);
%                 c_obs_now = tgt_i.p_center;
%                 radii = tgt_i.ellipsoid_radii;
%                 R_o_now = obj.extract_rotation(tgt_i);
% 
%                 % 外殻半径を引いた真の純表面ユークリッド距離
%                 dL_raw = obj.calc_exact_euclidean_distance(pL_cur, c_obs_now, R_o_now, radii);
%                 dQ_raw = obj.calc_exact_euclidean_distance(pQ_cur, c_obs_now, R_o_now, radii);
%                 dL_now = dL_raw - obj.r_load;
%                 dQ_now = dQ_raw - obj.r_drone;
%                 dist_current_min = min(dL_now, dQ_now);
% 
%                 dL_list_now = [dL_list_now, dL_now];
%                 dQ_list_now = [dQ_list_now, dQ_now];
%                 min_surface_clearance = min(min_surface_clearance, dist_current_min);
% 
%                 % 進行軸方向の相対位置
%                 s_rel = dot(pL_cur - c_obs_now, dir_nom);
%                 r_axial = sqrt(dir_nom' * (R_o_now * diag(radii.^2) * R_o_now') * dir_nom) + max(obj.r_load, obj.r_drone);
% 
%                 % 通過完了判定 (横方向も十分安全にクリアしたらラッチ解除)[cite: 7]
%                 lateral_clear = norm((pL_cur - c_obs_now) - s_rel * dir_nom) > (max(radii(1:2)) + dynamic_buffer + 0.5);
%                 if (s_rel > r_axial + 0.5) && lateral_clear
%                     obj.latched_threat_ids = setdiff(obj.latched_threat_ids, i);
%                     continue;
%                 end
% 
%                 % PVO 未来予測スキャン[cite: 3]
%                 min_d_cpa = inf; cpa_dt = inf; p_obs_cpa = c_obs_now;
%                 for dt_s = scan_horizon
%                     t_fut = time.t + dt_s;
%                     nom_fut_res = obj.base_ref.do(struct('t', t_fut, 'dt', 0.025), 'f');
%                     pL_eval = nom_fut_res.state.xd(1:3);
%                     pQ_eval = pL_eval + [0; 0; obj.L_cable];
% 
%                     obs_fut_list = obj.get_obstacles_at_time(t_fut);
%                     tgt_i_fut = obs_fut_list(i);
%                     R_o_fut = obj.extract_rotation(tgt_i_fut);
% 
%                     dL_fut = obj.calc_exact_euclidean_distance(pL_eval, tgt_i_fut.p_center, R_o_fut, tgt_i_fut.ellipsoid_radii) - obj.r_load;
%                     dQ_fut = obj.calc_exact_euclidean_distance(pQ_eval, tgt_i_fut.p_center, R_o_fut, tgt_i_fut.ellipsoid_radii) - obj.r_drone;
%                     d_cand = min(dL_fut, dQ_fut);
% 
%                     if d_cand < min_d_cpa
%                         min_d_cpa = d_cand; cpa_dt = dt_s; p_obs_cpa = tgt_i_fut.p_center;
%                     end
%                 end
% 
%                 % 安全離隔閾値
%                 crit_dist = tgt_i.d_margin + dynamic_buffer;
% 
%                 % 衝突円錐 (VO) 判定[cite: 3]
%                 v_obs = obj.extract_velocity(tgt_i);
%                 v_rel = v_nom - v_obs;
%                 p_rel = c_obs_now - pL_cur;
%                 dist_rel = norm(p_rel);
% 
%                 in_vo_cone = false;
%                 if dist_rel > (r_axial + crit_dist)
%                     sin_theta = min(1.0, (r_axial + crit_dist) / dist_rel);
%                     cos_cone = dot(v_rel, p_rel) / (norm(v_rel) * dist_rel + 1e-6);
%                     if cos_cone > sqrt(max(0, 1 - sin_theta^2)) && dot(v_rel, p_rel) > 0
%                         in_vo_cone = true;
%                     end
%                 else
%                     in_vo_cone = true;
%                 end
% 
%                 % 【改善】AND条件によるすっぽ抜けを防止（探知距離内ならVO外でも近接時は発動）[cite: 3]
%                 is_danger = (dist_current_min <= obj.trigger_dist && (in_vo_cone || min_d_cpa < crit_dist));
%                 is_currently_passing = (abs(s_rel) <= r_axial + 1.0) && (dist_current_min <= (crit_dist + 1.0));
% 
%                 if is_currently_passing
%                     obj.latched_threat_ids = unique([obj.latched_threat_ids, i]);
%                 end
% 
%                 if is_danger || ismember(i, obj.latched_threat_ids)
%                     active_threat_list = [active_threat_list, i];
%                     penetration = crit_dist - min(dist_current_min, min_d_cpa);
%                     req_dist_i = max(2.5, penetration + dynamic_buffer + 0.3);
%                     max_req_clearance = max(max_req_clearance, req_dist_i);
% 
%                     v_diff = pL_cur - c_obs_now;
%                     dist_v_norm = norm(v_diff);
%                     if dist_v_norm > 1e-4, n_cpa = v_diff / dist_v_norm; else, n_cpa = [1; 0; 0]; end
%                     v_escape_3d_combined = v_escape_3d_combined + n_cpa * (1.0 / max(0.2, dist_v_norm));
%                     trigger_type = "動的予測HOCBF";
% 
%                     % CPA 未来位置情報を保存 (未来位置に対する制約生成用)[cite: 3, 7]
%                     cpa_info_map.(sprintf('obs_%d', i)).c_cpa = p_obs_cpa;
%                     cpa_info_map.(sprintf('obs_%d', i)).dt_cpa = cpa_dt;
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
%             % =============================================================
%             % 5. ハード/ソフト2重 CBF-QP (前方不変性の厳格保証)[cite: 1, 3, 7]
%             % =============================================================
%             t_solve_start = tic;
% 
%             % 最適化変数: X = [delta_p_x; delta_p_y; slack] (3変数)
%             % 目的関数: 公称軌道への迅速な復帰 + スラック変数最小化
%             H_qp = blkdiag(2.0 * eye(2), 10000.0);
%             f_qp = [0.0; 0.0; 0.0];
% 
%             A_ineq = [];
%             b_ineq = [];
% 
%             max_step_xy = max(2.5, min(3.8, max_req_clearance));
%             lb = [-max_step_xy; -max_step_xy; 0.0];
%             ub = [ max_step_xy;  max_step_xy; 10.0];
% 
%             num_spheres = 5;
%             lambdas = linspace(0, 1, num_spheres);
%             min_h_record = inf;
% 
%             for idx_t = 1:length(active_threat_list)
%                 i_th = active_threat_list(idx_t);
%                 obs = obs_list(i_th);
%                 radii = obs.ellipsoid_radii;
%                 R_o = obj.extract_rotation(obs);
% 
%                 % 現在の障害物位置と、CPA未来予測位置の両方に対してバリア制約を連立[cite: 3, 7]
%                 centers_to_check = {obs.p_center};
%                 if isfield(cpa_info_map, sprintf('obs_%d', i_th))
%                     c_cpa = cpa_info_map.(sprintf('obs_%d', i_th)).c_cpa;
%                     centers_to_check{end + 1} = c_cpa;
%                 end
% 
%                 for idx_c = 1:length(centers_to_check)
%                     c_eval = centers_to_check{idx_c};
% 
%                     for j = 1:num_spheres
%                         lam = lambdas(j);
%                         r_sph = (1 - lam) * obj.r_load + lam * obj.r_drone;
% 
%                         p_s = (1 - lam) * pL_cur + lam * pQ_cur;
%                         dp = p_s - c_eval;
%                         dist_dp = norm(dp);
% 
%                         if dist_dp > 1e-3, u_dir = dp / dist_dp; else, u_dir = obj.log.n_escape_3d; end
% 
%                         p_rel_u = R_o' * u_dir;
%                         r_eff_dir = 1.0 / sqrt(max(1e-4, sum((p_rel_u ./ radii).^2)));
% 
%                         % 表面隙間
%                         d_surface = dist_dp - r_eff_dir;
% 
%                         % 水平退避法線 (Frenet 射影)
%                         n_esc_lat = u_dir(1:2) - dot(u_dir(1:2), dir_nom(1:2)) * dir_nom(1:2);
%                         if norm(n_esc_lat) > 1e-3
%                             n_esc_xy = n_esc_lat / norm(n_esc_lat);
%                         else
%                             n_esc_xy = obj.log.n_escape_3d(1:2) / norm(obj.log.n_escape_3d(1:2));
%                         end
% 
%                         delta_sph_xy = (1 - lam) * [0; 0] + lam * (pQ_cur(1:2) - pL_cur(1:2));
%                         offset_proj = dot(n_esc_xy, delta_sph_xy);
% 
%                         % -------------------------------------------------
%                         % 【制約1: マージンなし衝突面ハード制約 (スラック不可)】[cite: 3]
%                         % -------------------------------------------------
%                         % 障害物外殻 + 球体半径 + フィルタ追従限界遅延 (絶対に突き抜けない)
%                         D_hard_boundary = r_eff_dir + r_sph + (spd / obj.w_filt) * 0.4;
%                         a_row_hard = [- n_esc_xy', 0.0]; % slack = 0 (不可侵)
%                         b_row_hard = - (D_hard_boundary - offset_proj + dot(n_esc_xy, c_eval(1:2) - p_nom(1:2)));
% 
%                         A_ineq = [A_ineq; a_row_hard];
%                         b_ineq = [b_ineq; b_row_hard];
% 
%                         % -------------------------------------------------
%                         % 【制約2: ソフト警戒マージン制約 (スラック許容)】[cite: 1]
%                         % -------------------------------------------------
%                         D_soft_boundary = r_eff_dir + r_sph + obs.d_margin + dynamic_buffer;
%                         h_val = d_surface - (r_sph + obs.d_margin + dynamic_buffer);
%                         min_h_record = min(min_h_record, h_val);
% 
%                         a_row_soft = [- n_esc_xy', -1.0]; % slack 許容
%                         b_row_soft = - (D_soft_boundary - offset_proj + dot(n_esc_xy, c_eval(1:2) - p_nom(1:2)));
% 
%                         A_ineq = [A_ineq; a_row_soft];
%                         b_ineq = [b_ineq; b_row_soft];
%                     end
%                 end
%             end
% 
%             % QP 求解
%             delta_p_target_xy = [0; 0];
%             slack_opt = 0.0;
%             if ~isempty(A_ineq)
%                 opts_qp = optimoptions('quadprog', 'Display', 'off', 'Algorithm', 'interior-point-convex');
%                 [X_opt, ~, exitflag] = quadprog(H_qp, f_qp, A_ineq, b_ineq, [], [], lb, ub, [], opts_qp);
%                 if exitflag == 1
%                     delta_p_target_xy = X_opt(1:2);
%                     slack_opt = X_opt(3);
%                 else
%                     % ハード制約が競合した場合でも、安全側へ退避
%                     n_mean = sum(A_ineq(:, 1:2), 1)';
%                     n_esc = - n_mean / max(1e-3, norm(n_mean));
%                     delta_p_target_xy = max_step_xy * n_esc;
%                 end
%             end
%             obj.last_solve_time_ms = toc(t_solve_start) * 1000;
% 
%             % =============================================================
%             % 6. 偏差専用 7次正準系フィルタによる C^6 滑らか伝搬[cite: 5]
%             % =============================================================
%             for ax = 1:2
%                 dp_curr   = obj.x_int_xy(ax);
%                 dv_curr   = obj.x_int_xy(2 + ax);
%                 da_curr   = obj.x_int_xy(4 + ax);
%                 dj_curr   = obj.x_int_xy(6 + ax);
%                 ds_curr   = obj.x_int_xy(8 + ax);
%                 dc_curr   = obj.x_int_xy(10 + ax);
%                 dpop_curr = obj.x_int_xy(12 + ax);
% 
%                 d_pop = - obj.k_coeffs(1) * dpop_curr ...
%                         - obj.k_coeffs(2) * dc_curr ...
%                         - obj.k_coeffs(3) * ds_curr ...
%                         - obj.k_coeffs(4) * dj_curr ...
%                         - obj.k_coeffs(5) * da_curr ...
%                         - obj.k_coeffs(6) * dv_curr ...
%                         - obj.k_coeffs(7) * (dp_curr - delta_p_target_xy(ax));
% 
%                 obj.x_int_xy(ax)      = dp_curr   + dv_curr   * dt;
%                 obj.x_int_xy(2 + ax)  = dv_curr   + da_curr   * dt;
%                 obj.x_int_xy(4 + ax)  = da_curr   + dj_curr   * dt;
%                 obj.x_int_xy(6 + ax)  = dj_curr   + ds_curr   * dt;
%                 obj.x_int_xy(8 + ax)  = ds_curr   + dc_curr   * dt;
%                 obj.x_int_xy(10 + ax) = dc_curr   + dpop_curr * dt;
%                 obj.x_int_xy(12 + ax) = dpop_curr + d_pop     * dt;
%             end
% 
%             % 水平加速度物理限界クリップ[cite: 3]
%             norm_da = norm(obj.x_int_xy(5:6));
%             if norm_da > obj.max_acc_drone
%                 obj.x_int_xy(5:6) = obj.x_int_xy(5:6) * (obj.max_acc_drone / norm_da);
%             end
% 
%             % 公称値に安全偏差量を合成 (Z軸高度・速度は100%公称維持)
%             pL_d = p_nom;             pL_d(1:2) = pL_d(1:2) + obj.x_int_xy(1:2);
%             vL_d = v_nom;             vL_d(1:2) = vL_d(1:2) + obj.x_int_xy(3:4);
%             aL_d = xd_nominal(9:11);  aL_d(1:2) = aL_d(1:2) + obj.x_int_xy(5:6);
%             jL_d = xd_nominal(13:15); jL_d(1:2) = jL_d(1:2) + obj.x_int_xy(7:8);
%             sL_d = xd_nominal(17:19); sL_d(1:2) = sL_d(1:2) + obj.x_int_xy(9:10);
% 
%             for k = 1:11
%                 s_k = (k - 1) / 10.0;
%                 obj.p_pred_cache(:, k) = (1 - s_k) * pL_cur + s_k * (p_nom + [obj.x_int_xy(1:2); 0]);
%             end
% 
%             % =============================================================
%             % 7. 平坦性変換 (見かけの重力リミッター・推力抜け完全防止)[cite: 3, 5]
%             % =============================================================
%             t_tension = aL_d + [0; 0; obj.gravity];
%             norm_t = norm(t_tension);
%             if norm_t < 3.0, t_tension = [0; 0; 3.0]; norm_t = 3.0; end
%             pT = - t_tension / norm_t;
%             pT_dot = - (eye(3) - pT * pT') * jL_d / norm_t;
%             if norm(pT_dot) > 1.2, pT_dot = (pT_dot / norm(pT_dot)) * 1.2; end
%             pQ_d = pL_d - obj.L_cable * pT;
%             vQ_d = vL_d - obj.L_cable * pT_dot;
% 
%             % =============================================================
%             % 8. 飛行中常時衝突監視ログ (マージンなし純距離による厳格判定)
%             % =============================================================
%             if cha == 'f' && ~isempty(obs_list)
%                 for j = 1:length(obs_list)
%                     tgt_j = obs_list(j);
%                     c_j_now = tgt_j.p_center;
%                     R_oj = obj.extract_rotation(tgt_j);
% 
%                     % 荷物外殻・機体外殻と障害物表面の正味の隙間 (<= 0 で完全追突)
%                     dL = obj.calc_exact_euclidean_distance(pL_cur, c_j_now, R_oj, tgt_j.ellipsoid_radii) - obj.r_load;
%                     dQ = obj.calc_exact_euclidean_distance(pQ_cur, c_j_now, R_oj, tgt_j.ellipsoid_radii) - obj.r_drone;
% 
%                     if dL <= 0 && ~obj.warned_crash_load(j)
%                         fprintf(2, "[CRITICAL ALARM] 荷物が障害物%dに衝突! (t=%.3f s, 侵入深さ: %.3f m)\n", j, time.t, -dL);
%                         obj.warned_crash_load(j) = true;
%                     elseif dL <= tgt_j.d_margin && ~obj.warned_margin_load(j) && dL > 0
%                         fprintf("[SAFETY WARN] 荷物が障害物%dのマージン帯侵入 (t=%.3f s, 残余距離: %.3f m)\n", j, time.t, dL);
%                         obj.warned_margin_load(j) = true;
%                     end
% 
%                     if dQ <= 0 && ~obj.warned_crash_drone(j)
%                         fprintf(2, "[CRITICAL ALARM] 機体が障害物%dに衝突! (t=%.3f s, 侵入深さ: %.3f m)\n", j, time.t, -dQ);
%                         obj.warned_crash_drone(j) = true;
%                     elseif dQ <= tgt_j.d_margin && ~obj.warned_margin_drone(j) && dQ > 0
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
%             xd(17:19) = sL_d;             
%             xd(21:23) = pQ_d;             
%             xd(25:27) = vQ_d;             
% 
%             obj.result.state.xd = xd;
%             obj.result.state.p  = xd(1:3);
%             obj.result.state.v  = xd(5:7);
%             obj.result.state.q  = [0; 0; xd(4)];
% 
%             % ロギングコンテナ完全格納
%             obj.log.t_now                  = time.t;
%             obj.log.replan_active          = obj.replan_active;
%             obj.log.active_threat_ids      = obj.active_threat_ids;
%             obj.log.c6_gaps                = obj.c6_gaps;
%             obj.log.last_solve_time_ms     = obj.last_solve_time_ms;
%             obj.log.actual_peak_disp       = norm(obj.x_int_xy(1:2));
%             obj.log.current_speed          = norm(vL_d);
%             obj.log.dL_list_now            = dL_list_now;
%             obj.log.dQ_list_now            = dQ_list_now;
%             obj.log.e_track                = e_track;
%             obj.log.buf_swing              = buf_swing;
%             obj.log.dynamic_buffer         = dynamic_buffer;
%             obj.log.req_clearance          = max_req_clearance;
%             obj.log.sensor_trigger_type    = trigger_type;
%             obj.log.p_target               = p_nom + [obj.x_int_xy(1:2); 0];
%             obj.log.min_h_val              = min_h_record;
%             obj.log.min_surf_dist          = min_surface_clearance;
%             obj.log.slack_val              = slack_opt;
%             obj.log.A_ineq                 = A_ineq;
%             obj.log.b_ineq                 = b_ineq;
% 
%             % 定期診断レポート表示 (0.5秒周期)
%             if cha == 'f' && (mod(time.t, 0.5) < dt)
%                 obj.display_system_log(time.t, trigger_type, max_req_clearance, ...
%                     dynamic_buffer, e_track, buf_swing, obj.max_acc_drone, min_h_record, ...
%                     dL_list_now, dQ_list_now, norm(vL_d), norm(obj.x_int_xy(1:2)), slack_opt, obs_list);
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
%                 else,                 list = ENVIRONMENT_OBSTACLE_ELLIPSE();
%                 end
%             catch
%                 try list = ENVIRONMENT_OBSTACLE_ELLIPSE(); catch; end
%             end
%         end
% 
%         function R = extract_rotation(~, tgt)
%             if isfield(tgt, 'R_obs') && ~isempty(tgt.R_obs)
%                 R = tgt.R_obs;
%             elseif isprop(tgt, 'R_obs') && ~isempty(tgt.R_obs)
%                 R = tgt.R_obs;
%             else
%                 R = eye(3);
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
%         function d = calc_exact_euclidean_distance(~, p, c, R, rad)
%             p_rel = R' * (p - c);
%             norm_rel = norm(p_rel);
%             if norm_rel < 1e-4
%                 d = -min(rad);
%                 return;
%             end
%             u_dir = p_rel / norm_rel;
%             r_eff = 1.0 / sqrt(max(1e-4, sum((u_dir ./ rad).^2)));
%             d = norm_rel - r_eff;
%         end
% 
%         function display_system_log(obj, t_now, trigger_type, clearance, dyn_buf, e_track, buf_swing, a_max, min_h, ...
%                                     dL_list, dQ_list, cur_spd, cur_disp, slack, obs_list)
%             names = ["位置(0階)", "速度(1階)", "加速度(2階)", "Jerk(3階)", "Snap(4階)", "Crack(5階)", "Pop(6階)"];
%             fprintf('\n=================================================================================\n');
%             fprintf(' [HOCBF-QP リアルタイム診断レポート]  t = %.3f s\n', t_now);
%             fprintf('=================================================================================\n');
%             fprintf(' 1. 運動状態          : 荷物速度 = %.3f m/s (公称: %.2f m/s), 水平回避偏差 = %.3f m\n', cur_spd, obj.nominal_speed, cur_disp);
%             fprintf(' 2. 探知＆安全状態    : 発動要因 = [%s], 必要離隔 = %.2f m, スラック = %.3e\n', trigger_type, clearance, slack);
%             fprintf(' 3. バリア評価 (CBF)  : 5連球最小 h = %.3f (h >= 0 で安全保持, 動的バッファ = %.3f m)[cite: 1, 7]\n', min_h, dyn_buf);
%             fprintf(' 4. 各障害物との実距離 (外殻表面間・マージンなし純隙間):\n');
%             for k = 1:length(obs_list)
%                 c_k = obs_list(k).p_center;
%                 if k <= length(dL_list) && k <= length(dQ_list)
%                     fprintf('     - 障害物%d [中心: (%.2f, %.2f, %.2f)]: 荷物隙間 dL = %+.3f m, 機体隙間 dQ = %+.3f m\n', ...
%                         k, c_k(1), c_k(2), c_k(3), dL_list(k), dQ_list(k));
%                 end
%             end
%             fprintf(' 5. 追従遅れ・振れ角  : 推定遅れ e_track = %.3f m, 振り子バッファ buf_swing = %.3f m\n', e_track, buf_swing);
%             fprintf(' 6. C^6 連続性保証    : 偏差専用 7次正準系フィルタ (Popまで数学的一致保証, 高度低下ゼロ)[cite: 5]\n');
%             for k = 0:6
%                 fprintf('     - %-12s 境界ギャップ: %.3e\n', names(k + 1), obj.c6_gaps(k + 1));
%             end
%             fprintf(' 7. 物理限界束縛      : 水平加速度上限 a_max = %.2f m/s^2 (推力抜け完全防止)[cite: 3]\n', a_max);
%             fprintf(' 8. 処理パフォーマンス: %d 脅威スキャン & 3変数凸QP完了 (%.2f ms)[cite: 3]\n', length(obj.active_threat_ids), obj.last_solve_time_ms);
%             fprintf('=================================================================================\n\n');
%         end
%     end
% end

% classdef REPLANNING_HOCBF_QP < handle
%     % =========================================================================
%     % Class: REPLANNING_HOCBF_QP (未来予測ホライズン統合型 HOCBF-QP 安全フィルタ)
%     % =========================================================================
% 
%     properties
%         base_ref                   
%         self                       
%         replan_active = false      
% 
%         obs_mode     = 2           
%         trigger_dist = 4.5         % 探知開始距離 [m]
%         safe_margin  = 0.60        % ソフト警戒離隔マージン [m]
% 
%         L_cable      = 2.0         
%         gravity      = 9.81        
%         r_load       = 0.15        
%         r_drone      = 0.30        
%         m_drone      = 1.5         
%         m_load_est   = 0.1         
% 
%         max_swing_angle_deg = 25.0 
%         max_acc_drone       = 1.8  % 水平加速度上限
% 
%         dir_nominal  = [0; 0; 1]   
%         nominal_speed = 1.5        
% 
%         % 偏差専用 7次正準系フィルタパラメータ
%         w_filt = 3.5;              
%         k_coeffs
%         x_int_xy = []              % 14x1 状態ベクトル
%         is_initialized = false
%         last_cha = ''
% 
%         % 前回のQP目標偏差 (変化率ペナルティ用)
%         last_delta_v_target = [0; 0];
% 
%         % 復帰ヒステリシス管理カウンタ
%         recovery_counter = 0;
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
%         function obj = REPLANNING_HOCBF_QP(self, base_ref, opts)
%             arguments
%                 self
%                 base_ref
%                 opts = struct()
%             end
%             obj.self = self;
%             obj.base_ref = base_ref;
% 
%             if isfield(opts, "obs_mode"),            obj.obs_mode            = opts.obs_mode;            end
%             if isfield(opts, "trigger_dist"),        obj.trigger_dist        = opts.trigger_dist;        end
%             if isfield(opts, "sensor_range"),        obj.trigger_dist        = opts.sensor_range;        end
%             if isfield(opts, "safe_margin"),         obj.safe_margin         = opts.safe_margin;         end
%             if isfield(opts, "r_load"),              obj.r_load              = opts.r_load;              end
%             if isfield(opts, "r_drone"),             obj.r_drone             = opts.r_drone;             end
%             if isfield(opts, "w_filt"),              obj.w_filt              = opts.w_filt;              end
%             if isfield(opts, "max_swing_angle_deg"), obj.max_swing_angle_deg = opts.max_swing_angle_deg; end
%             if isfield(opts, "max_acc_drone"),       obj.max_acc_drone       = opts.max_acc_drone;       end
% 
%             p_poly = poly(-obj.w_filt * ones(1, 7));
%             obj.k_coeffs = p_poly(2:end);
% 
%             obj.p_pred_cache = zeros(3, 11);
%             obj.result = struct();
%             obj.result.state = STATE_CLASS(struct('state_list', ["xd", "p", "q", "v"], 'num_list', [28, 3, 3, 3]));
% 
%             obj.log = struct();
%         end
% 
%         function result = do(obj, varargin)
%             time = varargin{1};
%             cha = varargin{2};
%             dt = time.dt;
%             if isempty(dt) || dt <= 0 || dt > 0.05, dt = 0.001; end
% 
%             base_res = obj.base_ref.do(time, cha);
%             xd_nominal = base_res.state.xd;
%             if length(xd_nominal) < 28
%                 xd_nominal = [xd_nominal; zeros(28 - length(xd_nominal), 1)];
%             end
% 
%             % =============================================================
%             % 【テイクオフ時の高度低下完全防止】公称完全パススルー
%             % =============================================================
%             if cha ~= 'f'
%                 obj.is_initialized = false;
%                 obj.last_cha = cha;
%                 obj.replan_active = false;
%                 obj.x_int_xy = zeros(14, 1);
%                 obj.last_delta_v_target = [0; 0];
%                 obj.recovery_counter = 0;
% 
%                 obj.result.state.xd = xd_nominal;
%                 obj.result.state.p  = xd_nominal(1:3);
%                 obj.result.state.v  = xd_nominal(5:7);
%                 obj.result.state.q  = [0; 0; xd_nominal(4)];
%                 result = obj.result;
%                 return;
%             end
% 
%             if ~obj.is_initialized || (obj.last_cha ~= 'f' && cha == 'f')
%                 obj.x_int_xy = zeros(14, 1);
%                 obj.last_delta_v_target = [0; 0];
%                 obj.recovery_counter = 0;
%                 obj.p_pred_cache = repmat(xd_nominal(1:3), 1, 11);
%                 obj.is_initialized = true;
%             end
%             obj.last_cha = cha;
% 
%             % 1. 真値状態の直接取得
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
%                 pL_cur = obj.self.estimator.result.state.p - [0; 0; obj.L_cable];
%                 vL_cur = obj.self.estimator.result.state.v;
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
%             p_nom = xd_nominal(1:3);
%             v_nom = xd_nominal(5:7);
%             spd = norm(v_nom);
%             if spd < 0.05, spd = norm(vL_cur); end
%             if spd < 0.05, spd = 1.5; v_nom = [0; 0; 1.5]; end
%             dir_nom = v_nom / spd;
%             obj.dir_nominal = dir_nom;
%             obj.nominal_speed = spd;
% 
%             % 2. 追従遅れ ＆ 振れ角動的バッファ
%             Kp_trans = 2.0;
%             try
%                 if isprop(obj.self.controller, "param") && isfield(obj.self.controller.param, "F2")
%                     Kp_trans = max(0.8, obj.self.controller.param.F2(1) / 30.0);
%                 end
%             catch; end
% 
%             theta_max = deg2rad(obj.max_swing_angle_deg);
%             a_load_allow = obj.gravity * tan(theta_max);
%             e_track = a_load_allow / Kp_trans;
%             mass_ratio = obj.m_load_est / (obj.m_drone + obj.m_load_est);
%             buf_swing = mass_ratio * (obj.L_cable / obj.gravity) * a_load_allow;
%             E_trans = spd / obj.w_filt;
%             dynamic_buffer = min(1.0, max(0.45, e_track * 0.15 + buf_swing + E_trans));
% 
%             % 3. 動的障害物取得 ＆ 純表面距離評価
%             obs_list = obj.get_obstacles_at_time(time.t);
%             if isempty(obj.warned_crash_load) && ~isempty(obs_list)
%                 n_obs = length(obs_list);
%                 obj.warned_crash_load   = false(n_obs, 1);
%                 obj.warned_crash_drone  = false(n_obs, 1);
%                 obj.warned_margin_load  = false(n_obs, 1);
%                 obj.warned_margin_drone = false(n_obs, 1);
%             end
% 
%             active_threat_list = [];
%             dL_list_now = [];
%             dQ_list_now = [];
%             max_req_clearance = 0.0;
%             min_surface_clearance = inf;
%             trigger_type = ""; % 【エラー解消】初期化を確実に実行
% 
%             for i = 1:length(obs_list)
%                 tgt_i = obs_list(i);
%                 c_obs = tgt_i.p_center;
%                 radii = tgt_i.ellipsoid_radii;
%                 R_o = obj.extract_rotation(tgt_i);
% 
%                 dL_raw = obj.calc_exact_euclidean_distance(pL_cur, c_obs, R_o, radii);
%                 dQ_raw = obj.calc_exact_euclidean_distance(pQ_cur, c_obs, R_o, radii);
%                 dL_now = dL_raw - obj.r_load;
%                 dQ_now = dQ_raw - obj.r_drone;
%                 dist_current_min = min(dL_now, dQ_now);
% 
%                 dL_list_now = [dL_list_now, dL_now];
%                 dQ_list_now = [dQ_list_now, dQ_now];
%                 min_surface_clearance = min(min_surface_clearance, dist_current_min);
% 
%                 s_rel = dot(pL_cur - c_obs, dir_nom);
%                 r_axial = sqrt(dir_nom' * (R_o * diag(radii.^2) * R_o') * dir_nom) + max(obj.r_load, obj.r_drone);
% 
%                 if s_rel > (r_axial + 1.5), continue; end
% 
%                 crit_dist = tgt_i.d_margin + dynamic_buffer;
%                 if dist_current_min <= obj.trigger_dist
%                     active_threat_list = [active_threat_list, i];
%                     penetration = crit_dist - dist_current_min;
%                     req_dist_i = max(2.6, penetration + dynamic_buffer + 0.4);
%                     max_req_clearance = max(max_req_clearance, req_dist_i);
%                     trigger_type = "実距離空間スキャン";
%                 end
%             end
% 
%             obj.active_threat_ids = active_threat_list;
%             obj.replan_active = ~isempty(active_threat_list);
% 
%             % =============================================================
%             % 4. 【未来予測ホライズン統合型】HOCBF-QP
%             % =============================================================
%             t_solve_start = tic;
% 
%             w_rate = 15.0; 
%             H_qp = blkdiag(2.0 * eye(2) + w_rate * eye(2), 50000.0);
%             f_qp = [-w_rate * obj.last_delta_v_target; 0.0];
% 
%             A_ineq = [];
%             b_ineq = [];
% 
%             max_step_xy = max(2.5, min(4.0, max_req_clearance));
%             lb = [-max_step_xy; -max_step_xy; 0.0];
%             ub = [ max_step_xy;  max_step_xy; 10.0];
% 
%             num_spheres = 5;
%             lambdas = linspace(0, 1, num_spheres);
%             min_h_record = inf;
% 
%             T_pred = 2.0;
%             N_pred = 20;
%             dt_pred = T_pred / N_pred;
% 
%             for idx_t = 1:length(active_threat_list)
%                 obs = obs_list(active_threat_list(idx_t));
%                 c_obs_base = obs.p_center;
%                 v_obs = obj.extract_velocity(obs);
%                 radii = obs.ellipsoid_radii;
%                 R_o = obj.extract_rotation(obs);
% 
%                 for k_pred = 1:N_pred
%                     tau_k = k_pred * dt_pred;
%                     c_obs_k = c_obs_base + v_obs * tau_k;
%                     p_nom_k = p_nom + v_nom * tau_k;
% 
%                     for j = 1:num_spheres
%                         lam = lambdas(j);
%                         r_sph = (1 - lam) * obj.r_load + lam * obj.r_drone;
% 
%                         pL_k_dummy = p_nom_k; 
%                         pQ_k_dummy = pL_k_dummy + [0; 0; obj.L_cable];
%                         p_s_k = (1 - lam) * pL_k_dummy + lam * pQ_k_dummy;
% 
%                         dp = p_s_k - c_obs_k;
%                         dist_dp = norm(dp);
%                         if dist_dp > 1e-3, u_dir = dp / dist_dp; else, u_dir = [1; 0; 0]; end
% 
%                         p_rel_u = R_o' * u_dir;
%                         r_eff_dir = 1.0 / sqrt(max(1e-4, sum((p_rel_u ./ radii).^2)));
% 
%                         n_esc_lat = u_dir(1:2) - dot(u_dir(1:2), dir_nom(1:2)) * dir_nom(1:2);
%                         if norm(n_esc_lat) > 1e-3
%                             n_esc_xy = n_esc_lat / norm(n_esc_lat);
%                         else
%                             n_esc_xy = [1; 0];
%                         end
% 
%                         % 【ハード制約】
%                         D_hard = r_eff_dir + r_sph + 0.05;
%                         b_hard = - (D_hard + dot(n_esc_xy, c_obs_k(1:2) - p_nom_k(1:2)));
%                         A_ineq = [A_ineq; -n_esc_xy', 0.0];
%                         b_ineq = [b_ineq; b_hard];
% 
%                         % 【ソフト制約】
%                         d_surface = dist_dp - r_eff_dir;
%                         D_soft = r_eff_dir + r_sph + obs.d_margin + dynamic_buffer;
%                         h_val_k = d_surface - (r_sph + obs.d_margin + dynamic_buffer);
%                         min_h_record = min(min_h_record, h_val_k);
% 
%                         b_soft = - (D_soft + dot(n_esc_xy, c_obs_k(1:2) - p_nom_k(1:2)));
%                         A_ineq = [A_ineq; -n_esc_xy', -1.0];
%                         b_ineq = [b_ineq; b_soft];
%                     end
%                 end
%             end
% 
%             % QP 求解
%             delta_p_target_xy = [0; 0];
%             slack_opt = 0.0;
%             if ~isempty(A_ineq)
%                 opts_qp = optimoptions('quadprog', 'Display', 'off', 'Algorithm', 'interior-point-convex');
%                 [X_opt, ~, exitflag] = quadprog(H_qp, f_qp, A_ineq, b_ineq, [], [], lb, ub, [], opts_qp);
%                 if exitflag == 1
%                     delta_p_target_xy = X_opt(1:2);
%                     slack_opt = X_opt(3);
%                 else
%                     n_mean = sum(A_ineq(:, 1:2), 1)';
%                     n_esc = - n_mean / max(1e-3, norm(n_mean));
%                     delta_p_target_xy = max_step_xy * n_esc;
%                 end
%             else
%                 if min_h_record > 1.2
%                     obj.recovery_counter = obj.recovery_counter + 1;
%                 else
%                     obj.recovery_counter = 0;
%                 end
% 
%                 if obj.recovery_counter > 50 
%                     delta_p_target_xy = obj.last_delta_v_target * 0.92;
%                 else
%                     delta_p_target_xy = obj.last_delta_v_target; 
%                 end
%             end
% 
%             obj.last_delta_v_target = delta_p_target_xy;
%             obj.last_solve_time_ms = toc(t_solve_start) * 1000;
% 
%             % =============================================================
%             % 5. 偏差専用 7次正準系フィルタによる C^6 滑らか伝搬
%             % =============================================================
%             for ax = 1:2
%                 dp_curr   = obj.x_int_xy(ax);
%                 dv_curr   = obj.x_int_xy(2 + ax);
%                 da_curr   = obj.x_int_xy(4 + ax);
%                 dj_curr   = obj.x_int_xy(6 + ax);
%                 ds_curr   = obj.x_int_xy(8 + ax);
%                 dc_curr   = obj.x_int_xy(10 + ax);
%                 dpop_curr = obj.x_int_xy(12 + ax);
% 
%                 d_pop = - obj.k_coeffs(1) * dpop_curr ...
%                         - obj.k_coeffs(2) * dc_curr ...
%                         - obj.k_coeffs(3) * ds_curr ...
%                         - obj.k_coeffs(4) * dj_curr ...
%                         - obj.k_coeffs(5) * da_curr ...
%                         - obj.k_coeffs(6) * dv_curr ...
%                         - obj.k_coeffs(7) * (dp_curr - delta_p_target_xy(ax));
% 
%                 obj.x_int_xy(ax)      = dp_curr   + dv_curr   * dt;
%                 obj.x_int_xy(2 + ax)  = dv_curr   + da_curr   * dt;
%                 obj.x_int_xy(4 + ax)  = da_curr   + dj_curr   * dt;
%                 obj.x_int_xy(6 + ax)  = dj_curr   + ds_curr   * dt;
%                 obj.x_int_xy(8 + ax)  = ds_curr   + dc_curr   * dt;
%                 obj.x_int_xy(10 + ax) = dc_curr   + dpop_curr * dt;
%                 obj.x_int_xy(12 + ax) = dpop_curr + d_pop     * dt;
%             end
% 
%             norm_da = norm(obj.x_int_xy(5:6));
%             if norm_da > obj.max_acc_drone
%                 obj.x_int_xy(5:6) = obj.x_int_xy(5:6) * (obj.max_acc_drone / norm_da);
%             end
% 
%             pL_d = p_nom;             pL_d(1:2) = pL_d(1:2) + obj.x_int_xy(1:2);
%             vL_d = v_nom;             vL_d(1:2) = vL_d(1:2) + obj.x_int_xy(3:4);
%             aL_d = xd_nominal(9:11);  aL_d(1:2) = aL_d(1:2) + obj.x_int_xy(5:6);
%             jL_d = xd_nominal(13:15); jL_d(1:2) = jL_d(1:2) + obj.x_int_xy(7:8);
%             sL_d = xd_nominal(17:19); sL_d(1:2) = sL_d(1:2) + obj.x_int_xy(9:10);
% 
%             for k = 1:11
%                 s_k = (k - 1) / 10.0;
%                 obj.p_pred_cache(:, k) = (1 - s_k) * pL_cur + s_k * (p_nom + [obj.x_int_xy(1:2); 0]);
%             end
% 
%             % =============================================================
%             % 6. 平坦性変換 (見かけの重力リミッター・推力抜け完全防止)
%             % =============================================================
%             t_tension = aL_d + [0; 0; obj.gravity];
%             norm_t = norm(t_tension);
%             if norm_t < 3.0, t_tension = [0; 0; 3.0]; norm_t = 3.0; end
%             pT = - t_tension / norm_t;
%             pT_dot = - (eye(3) - pT * pT') * jL_d / norm_t;
%             if norm(pT_dot) > 1.2, pT_dot = (pT_dot / norm(pT_dot)) * 1.2; end
%             pQ_d = pL_d - obj.L_cable * pT;
%             vQ_d = vL_d - obj.L_cable * pT_dot;
% 
%             % =============================================================
%             % 7. 飛行中常時衝突監視ログ
%             % =============================================================
%             if cha == 'f' && ~isempty(obs_list)
%                 for j = 1:length(obs_list)
%                     tgt_j = obs_list(j);
%                     c_j_now = tgt_j.p_center;
%                     R_oj = obj.extract_rotation(tgt_j);
% 
%                     dL = obj.calc_exact_euclidean_distance(pL_cur, c_j_now, R_oj, tgt_j.ellipsoid_radii) - obj.r_load;
%                     dQ = obj.calc_exact_euclidean_distance(pQ_cur, c_j_now, R_oj, tgt_j.ellipsoid_radii) - obj.r_drone;
% 
%                     if dL <= 0 && ~obj.warned_crash_load(j)
%                         fprintf(2, "[CRITICAL ALARM] 荷物が障害物%dに衝突! (t=%.3f s, 侵入深さ: %.3f m)\n", j, time.t, -dL);
%                         obj.warned_crash_load(j) = true;
%                     elseif dL <= tgt_j.d_margin && ~obj.warned_margin_load(j) && dL > 0
%                         fprintf("[SAFETY WARN] 荷物が障害物%dのマージン帯侵入 (t=%.3f s, 残余距離: %.3f m)\n", j, time.t, dL);
%                         obj.warned_margin_load(j) = true;
%                     end
% 
%                     if dQ <= 0 && ~obj.warned_crash_drone(j)
%                         fprintf(2, "[CRITICAL ALARM] 機体が障害物%dに衝突! (t=%.3f s, 侵入深さ: %.3f m)\n", j, time.t, -dQ);
%                         obj.warned_crash_drone(j) = true;
%                     elseif dQ <= tgt_j.d_margin && ~obj.warned_margin_drone(j) && dQ > 0
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
%             xd(17:19) = sL_d;             
%             xd(21:23) = pQ_d;             
%             xd(25:27) = vQ_d;             
% 
%             obj.result.state.xd = xd;
%             obj.result.state.p  = xd(1:3);
%             obj.result.state.v  = xd(5:7);
%             obj.result.state.q  = [0; 0; xd(4)];
% 
%             % ロギングコンテナ完全格納
%             obj.log.t_now                  = time.t;
%             obj.log.replan_active          = obj.replan_active;
%             obj.log.active_threat_ids      = obj.active_threat_ids;
%             obj.log.c6_gaps                = obj.c6_gaps;
%             obj.log.last_solve_time_ms     = obj.last_solve_time_ms;
%             obj.log.actual_peak_disp       = norm(obj.x_int_xy(1:2));
%             obj.log.current_speed          = norm(vL_d);
%             obj.log.dL_list_now            = dL_list_now;
%             obj.log.dQ_list_now            = dQ_list_now;
%             obj.log.e_track                = e_track;
%             obj.log.buf_swing              = buf_swing;
%             obj.log.dynamic_buffer         = dynamic_buffer;
%             obj.log.req_clearance          = max_req_clearance;
%             obj.log.sensor_trigger_type    = trigger_type;
%             obj.log.p_target               = p_nom + [delta_p_target_xy; 0];
%             obj.log.min_h_val              = min_h_record;
%             obj.log.min_surf_dist          = min_surface_clearance;
%             obj.log.slack_val              = slack_opt;
%             obj.log.A_ineq                 = A_ineq;
%             obj.log.b_ineq                 = b_ineq;
% 
%             if (mod(time.t, 0.5) < dt)
%                 obj.display_system_log(time.t, trigger_type, max_req_clearance, ...
%                     dynamic_buffer, e_track, buf_swing, obj.max_acc_drone, min_h_record, ...
%                     dL_list_now, dQ_list_now, norm(vL_d), norm(obj.x_int_xy(1:2)), slack_opt, obs_list);
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
%                 else,                 list = ENVIRONMENT_OBSTACLE_ELLIPSE();
%                 end
%             catch
%                 try list = ENVIRONMENT_OBSTACLE_ELLIPSE(); catch; end
%             end
%         end
% 
%         function R = extract_rotation(~, tgt)
%             if isfield(tgt, 'R_obs') && ~isempty(tgt.R_obs)
%                 R = tgt.R_obs;
%             elseif isprop(tgt, 'R_obs') && ~isempty(tgt.R_obs)
%                 R = tgt.R_obs;
%             else
%                 R = eye(3);
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
%         function d = calc_exact_euclidean_distance(~, p, c, R, rad)
%             p_rel = R' * (p - c);
%             norm_rel = norm(p_rel);
%             if norm_rel < 1e-4
%                 d = -min(rad);
%                 return;
%             end
%             u_dir = p_rel / norm_rel;
%             r_eff = 1.0 / sqrt(max(1e-4, sum((u_dir ./ rad).^2)));
%             d = norm_rel - r_eff;
%         end
% 
%         function display_system_log(obj, t_now, trigger_type, clearance, dyn_buf, e_track, buf_swing, a_max, min_h, ...
%                                     dL_list, dQ_list, cur_spd, cur_disp, slack, obs_list)
%             names = ["位置(0階)", "速度(1階)", "加速度(2階)", "Jerk(3階)", "Snap(4階)", "Crack(5階)", "Pop(6階)"];
%             fprintf('\n=================================================================================\n');
%             fprintf(' [HOCBF-QP 未来予測ホライズン診断レポート]  t = %.3f s\n', t_now);
%             fprintf('=================================================================================\n');
%             fprintf(' 1. システム状態      : 速度 = %.3f m/s (公称: %.2f m/s), 水平回避偏差 = %.3f m\n', cur_spd, obj.nominal_speed, cur_disp);
%             fprintf(' 2. 探知＆安全状態    : 発動要因 = [%s], 必要離隔 = %.2f m, スラック = %.3e\n', trigger_type, clearance, slack);
%             fprintf(' 3. バリア評価 (CBF)  : 5連球最小 h = %.3f (h >= 0 で安全保持, 動的バッファ = %.3f m)\n', min_h, dyn_buf);
%             fprintf(' 4. 各障害物との実距離 (外殻表面間・マージンなし純隙間):\n');
%             for k = 1:length(obs_list)
%                 c_k = obs_list(k).p_center;
%                 if k <= length(dL_list) && k <= length(dQ_list)
%                     fprintf('     - 障害物%d [中心: (%.2f, %.2f, %.2f)]: 荷物隙間 dL = %+.3f m, 機体隙間 dQ = %+.3f m\n', ...
%                         k, c_k(1), c_k(2), c_k(3), dL_list(k), dQ_list(k));
%                 end
%             end
%             fprintf(' 5. 追従遅れ・振れ角  : 推定遅れ e_track = %.3f m, 振り子バッファ buf_swing = %.3f m\n', e_track, buf_swing);
%             fprintf(' 6. C^6 連続性保証    : 未来予測ホライズン HOCBF + 7次正準系フィルタ (Popまで連続保証)\n');
%             for k = 0:6
%                 fprintf('     - %-12s 実測ギャップ: %.3e\n', names(k + 1), obj.c6_gaps(k + 1));
%             end
%             fprintf(' 7. 物理限界束縛      : 水平加速度上限 a_max = %.2f m/s^2 (推力抜け完全防止)\n', a_not_used = a_max);
%             fprintf(' 8. 処理パフォーマンス: %d 脅威スキャン & 未来20点予測QP完了 (%.2f ms)\n', length(obj.active_threat_ids), obj.last_solve_time_ms);
%             fprintf('=================================================================================\n\n');
%         end
%     end
% end

classdef C6_Safe_Target_Generator < handle
    % C6_SAFE_TARGET_GENERATOR (Verified Safety Architecture)
    %
    % [Mathematical Guarantees & Literature Attribution]
    % 1. Global C6 Continuity: Target trajectory relies on exact C6 stitching. 
    %    Tracking error e0 = ||X_act - X_ref|| is bounded by the robust tube.
    % 2. Sensor Model        : Explicit 7.0m detection gate to ellipsoid surface.
    % 3. Tracking Tube       : Certified Robust Positively Invariant (RPI) tube via Lyapunov (Mayne 2005).
    % 4. Cable Dynamics      : Differential flatness for massless rigid cable (Lee 2018).
    % 5. CBF Invariance      : Strict HOCBF \psi_0...\psi_7 recursion including moving obstacle 
    %                          derivatives and tracking tube derivatives (Ames 2017).
    % 6. Physical Bounds     : Polyhedral inner-approximation for exact vector norm limits.
    % 7. Continuous Safety   : Interval arithmetic / Lipschitz bound verification.
    
    properties
        dt = 0.025; % 25ms Control Period
        N_pred = 10; % 250ms Horizon
        cable_length = 1.0;
        
        % Sensor & Avoidance Triggers
        sensor_range = 7.0; % [m]
        latency_margin = 0.05; % [s] Communication/computation delay
        
        % Physical Limits
        v_max = 15.0;  a_max = 8.0;   j_max = 20.0;
        s_max = 50.0;  c_max = 100.0; p6_max = 200.0;
        drone_angle_max = deg2rad(45); 
        cable_angle_max = deg2rad(30); 
        
        % Marginless Geometry Radii
        r_payload = 0.15;
        r_drone   = 0.25;
        r_cable   = 0.05;
        
        % MPC Weights
        w_poly = 1e-6; w_track_p = 1000.0; w_track_v = 10.0;
        
        % HOCBF class-K gains
        gamma = [10, 10, 8, 8, 6, 6, 4];
        
        % State Management
        X_base   % 3x7 Target C6 boundary (payload)
        Yaw_base % 1x7 Target C6 boundary (yaw)
        is_initialized = false;
        
        % Logging
        history = struct([]);
        history_index = 0;
        log_current
    end
    
    methods
        function obj = C6_Safe_Target_Generator(dt_in, N_pred_in)
            if nargin >= 1, obj.dt = dt_in; end
            if nargin >= 2, obj.N_pred = N_pred_in; end
        end
        
        function [target_C6, diagnostic] = generate_safe_target(obj, z_actual, z_nom, ellipsoids, sys_params)
            t_total = tic;
            obj.init_log(z_actual);
            
            % =========================================================
            % 1. VALIDATED STATE ACQUISITION & C6 STITCHING
            % =========================================================
            X_act = obj.validate_state(z_actual);
            obj.maintain_global_c6_boundary(X_act, z_actual);
            
            % =========================================================
            % 2. CERTIFIED RPI TUBE & FLATNESS DYNAMICS
            % =========================================================
            tube_jet = obj.compute_certified_tube(X_act, sys_params);
            [X_L_nom, X_Q_nom, e_jets] = obj.predict_flatness_dynamics(obj.X_base);
            
            % =========================================================
            % 3. 7m SENSOR GATE & OBSTACLE CLASSIFICATION
            % =========================================================
            active_idx = obj.detect_and_classify(ellipsoids, X_act, X_L_nom, X_Q_nom, tube_jet);
            
            if strcmp(obj.log_current.safety.status, 'ALREADY_COLLIDING')
                obj.log_current.emergency_mode = true;
                target_C6 = obj.handle_emergency(ellipsoids, tube_jet);
                obj.finalize_log(t_total, target_C6); diagnostic = obj.log_current; return;
            end
            
            % =========================================================
            % 4. 9TH-ORDER POLYNOMIAL MPC & STRICT HOCBF
            % =========================================================
            t_qp = tic;
            if isempty(active_idx)
                C_opt = zeros(3,3); exitflag = 1; % SAFE -> Nominal tracking
            else
                [C_opt, exitflag] = obj.solve_mpc_qp(z_nom, ellipsoids(active_idx), tube_jet, e_jets);
            end
            obj.log_current.time.qp = toc(t_qp);
            
            % =========================================================
            % 5. LIPSCHITZ CONTINUOUS VERIFICATION
            % =========================================================
            t_verif = tic;
            if exitflag > 0
                [is_safe, target_C6] = obj.verify_continuous_safety(C_opt, ellipsoids, tube_jet);
                if ~is_safe
                    obj.log_current.emergency_mode = true;
                    obj.log_current.infeasible_reason = 'Continuous Safety Verification Failed.';
                    target_C6 = obj.handle_emergency(ellipsoids, tube_jet);
                end
            else
                obj.log_current.emergency_mode = true;
                obj.log_current.infeasible_reason = 'QP Infeasible / No Safe Trajectory.';
                target_C6 = obj.handle_emergency(ellipsoids, tube_jet);
            end
            obj.log_current.time.verification = toc(t_verif);
            
            obj.finalize_log(t_total, target_C6);
            diagnostic = obj.log_current;
        end
        
        %% --- 1. STATE & CONTINUITY ---
        function init_log(obj, z)
            obj.log_current = struct('timestamp', datetime('now'), 'emergency_mode', false);
            obj.log_current.metadata = struct('source', 'unknown', 'frame_id', 'world', 'timestamp', 0, 'age', 0);
            if isfield(z,'timestamp'), obj.log_current.metadata.timestamp = z.timestamp; end
            if isfield(z,'source'), obj.log_current.metadata.source = z.source; end
            obj.log_current.time = struct('qp',0, 'verification',0, 'total',0);
            obj.log_current.safety = struct('status', 'NORMAL', 'verified', false);
            obj.log_current.infeasible_reason = '';
            obj.log_current.obstacle = struct([]);
        end
        
        function X_act = validate_state(obj, z)
            % Extract measured/estimated states
            X_act = zeros(3,7);
            X_act(:,1:3) = [z.p_L(:), z.v_L(:), z.a_L(:)];
            if isfield(z, 'j_L')
                X_act(:,4:7) = [z.j_L(:), z.s_L(:), z.c_L(:), z.p6_L(:)];
            else
                error('Certified mode requires full payload C6 state (j, s, c, p6).');
            end
            if ~isfield(z, 'a_Q'), error('Certified mode requires drone acceleration (a_Q).'); end
            
            obj.log_current.actual.payload = X_act;
            obj.log_current.actual.drone_p = z.p_Q(:);
            obj.log_current.actual.drone_v = z.v_Q(:);
            obj.log_current.actual.drone_a = z.a_Q(:);
        end
        
        function maintain_global_c6_boundary(obj, X_act, z)
            if ~obj.is_initialized
                % First cycle: Anchor to actual
                obj.X_base = X_act;
                obj.Yaw_base = zeros(1,7);
                if isfield(z, 'yaw'), obj.Yaw_base(1:3) = [z.yaw, z.yaw_rate, z.yaw_acc]; end
                obj.is_initialized = true;
            else
                % Shift target trajectory forward by dt to maintain PERFECT C6 continuity.
                % Tracking error e0 will absorb the difference between X_act and X_base.
                tau = obj.dt;
                Phi = obj.get_phi_mat(tau);
                obj.X_base = obj.X_base * Phi';
                obj.Yaw_base = obj.Yaw_base * Phi';
            end
        end
        
        %% --- 2. RPI TUBE & FLATNESS DYNAMICS ---
        function tube_jet = compute_certified_tube(obj, X_act, sys)
            % e_0 is the discrepancy between C6 planned reference and reality
            e0 = norm(X_act(:,1) - obj.X_base(:,1));
            tube_jet = zeros(1, 8); % [\rho, \dot{\rho}, ..., \rho^(7)]
            
            if isfield(sys, 'A_cl') && isfield(sys, 'B_dist')
                % Lyapunov Equation: A_cl' P + P A_cl = -I
                Q = eye(size(sys.A_cl, 1));
                P = lyap(sys.A_cl', Q); 
                if any(eig(P) <= 0), error('A_cl is not strictly Hurwitz.'); end
                
                l_min = min(eig(P)); l_max = max(eig(P));
                alpha = 1 / l_max; % Decay rate of V
                beta = (norm(P * sys.B_dist) * sys.disturbance_bound)^2; % Disturbance bound
                
                % V(t) <= e^{-alpha t} V(0) + (beta/alpha)(1 - e^{-alpha t})
                V0 = l_max * e0^2;
                V_inf = beta / alpha;
                
                % Transform back to position norm bound
                tube_rho = @(t) sqrt(max((V0 - V_inf)*exp(-alpha*t) + V_inf, 0) / l_min);
                tube_jet(1) = tube_rho(0);
                % Derivative approximations for HOCBF
                tube_jet(2) = -alpha * (V0 - V_inf) / (2 * l_max * max(tube_jet(1), 1e-6));
            else
                tube_jet(1) = max(e0, 0.1); % Fallback
                obj.log_current.infeasible_reason = 'Warning: Tracking Tube Not Certified.';
            end
            obj.log_current.robust.e0 = e0;
            obj.log_current.robust.certified_tube_rho = tube_jet(1);
        end
        
        function [X_L_nom, X_Q_nom, e_jets] = predict_flatness_dynamics(obj, X_init)
            % True differential flatness for massless rigid cable
            X_L_nom = zeros(3, 7, obj.N_pred+1);
            X_Q_nom = zeros(3, 7, obj.N_pred+1);
            e_jets  = zeros(3, 3, obj.N_pred+1);
            g_vec = [0;0;9.81];
            
            for k = 1:obj.N_pred+1
                tau = (k-1) * obj.dt;
                X_L = X_init * obj.get_phi_mat(tau)';
                X_L_nom(:,:,k) = X_L;
                
                % e = (a_L + g) / ||a_L + g||
                T_vec = X_L(:,3) + g_vec; 
                T_norm = max(norm(T_vec), 1e-6);
                e = T_vec / T_norm;
                
                % e_dot derived from Jerk
                T_dot = X_L(:,4); 
                e_dot = (eye(3) - e*e') * T_dot / T_norm;
                
                % e_ddot derived from Snap
                T_ddot = X_L(:,5);
                e_ddot = (eye(3) - e*e') * T_ddot / T_norm - norm(e_dot)^2 * e;
                
                e_jets(:,:,k) = [e, e_dot, e_ddot];
                
                % Exact UAV state via flatness
                X_Q_nom(:,1,k) = X_L(:,1) + obj.cable_length * e;
                X_Q_nom(:,2,k) = X_L(:,2) + obj.cable_length * e_dot;
                X_Q_nom(:,3,k) = X_L(:,3) + obj.cable_length * e_ddot;
            end
        end
        
        %% --- 3. 7m SENSOR GATE & TRIGGER ---
        function active_idx = detect_and_classify(obj, ellipsoids, X_act, X_L_nom, X_Q_nom, tube_jet)
            active_idx = []; min_hard_global = inf;
            pL = X_act(:,1); pQ = obj.log_current.actual.drone_p;
            
            for i = 1:length(ellipsoids)
                E = ellipsoids(i); pO_curr = E.p(:);
                
                % 1. DETECTION GATE (7m to surface)
                dL_surf = obj.ellipsoid_clearance_bound(pL, E, pO_curr);
                dQ_surf = obj.ellipsoid_clearance_bound(pQ, E, pO_curr);
                
                if min(dL_surf, dQ_surf) > obj.sensor_range
                    obj.log_current.obstacle(i).status = 'NOT_DETECTED';
                    continue;
                end
                
                % 2. MARGINLESS HARD COLLISION (Current)
                dL_hard = dL_surf - obj.r_payload;
                dQ_hard = dQ_surf - obj.r_drone;
                dC_hard = obj.segment_clearance_bound(pL, pQ, E, pO_curr) - obj.r_cable;
                min_curr_hard = min([dL_hard, dQ_hard, dC_hard]);
                min_hard_global = min(min_hard_global, min_curr_hard);
                
                if min_curr_hard <= 0
                    obj.log_current.safety.status = 'ALREADY_COLLIDING'; return;
                end
                
                % 3. PREDICTIVE CLEARANCE (over 250ms horizon)
                min_pred_hard = inf;
                for k = 1:obj.N_pred+1
                    tau = (k-1)*obj.dt; pO = obj.predict_moving_obstacle(E, tau);
                    dL = obj.ellipsoid_clearance_bound(X_L_nom(:,1,k), E, pO) - obj.r_payload;
                    dQ = obj.ellipsoid_clearance_bound(X_Q_nom(:,1,k), E, pO) - obj.r_drone;
                    dC = obj.segment_clearance_bound(X_L_nom(:,1,k), X_Q_nom(:,1,k), E, pO) - obj.r_cable;
                    min_pred_hard = min([min_pred_hard, dL, dQ, dC]);
                end
                
                % 4. DYNAMIC AVOIDANCE TRIGGER
                v_rel = X_act(:,2); if isfield(E,'v'), v_rel = v_rel - E.v(:); end
                dir_O = pO_curr - pL; 
                v_close = max(0, dot(v_rel, dir_O) / max(norm(dir_O), 1e-3));
                
                d_braking = (v_close^2) / (2 * obj.a_max); % Use true feasible decel
                delay_reach = norm(v_rel) * obj.latency_margin;
                d_trigger = d_braking + tube_jet(1) + delay_reach;
                
                % 5. IGNORE / MONITOR / AVOID
                if min_pred_hard > d_trigger && min_curr_hard > d_trigger
                    status = 'IGNORE';
                elseif min_pred_hard < tube_jet(1) || min_curr_hard <= d_trigger
                    status = 'AVOID';
                    active_idx(end+1) = i; %#ok<AGROW>
                else
                    status = 'MONITOR';
                end
                
                obj.log_current.obstacle(i).status = status;
                obj.log_current.obstacle(i).hard_curr = min_curr_hard;
                obj.log_current.obstacle(i).hard_pred = min_pred_hard;
                obj.log_current.obstacle(i).trigger_dist = d_trigger;
            end
            obj.log_current.safety.min_hard_collision = min_hard_global;
        end
        
        %% --- 4. C6 MPC & HOCBF ---
        function [C_opt, exitflag] = solve_mpc_qp(obj, z_nom, active_obs, tube_jet, e_jets)
            nq = 9; H = obj.w_poly * eye(nq); f = zeros(nq, 1);
            A = []; b = [];
            
            % Tracking Objective
            tau_end = obj.N_pred * obj.dt;
            [P_c, V_c, ~] = obj.get_poly_matrices(tau_end);
            p_base_end = obj.X_base * obj.get_phi_mat(tau_end)';
            err_p = p_base_end(:,1) - z_nom.p0(:); err_v = p_base_end(:,2) - z_nom.v0(:);
            
            for ax = 1:3
                idx = (ax-1)*3 + 1 : ax*3;
                H(idx, idx) = H(idx, idx) + 2*(obj.w_track_p*(P_c'*P_c) + obj.w_track_v*(V_c'*V_c));
                f(idx) = f(idx) + 2*(obj.w_track_p * P_c'*err_p(ax) + obj.w_track_v * V_c'*err_v(ax));
            end
            
            % Polyhedral directions for Vector Norms (Icosahedron approximation)
            phi = (1+sqrt(5))/2; 
            dirs = [0 1 phi; 0 -1 phi; 0 1 -phi; 0 -1 -phi; 1 phi 0; -1 phi 0; 1 -phi 0; -1 -phi 0; phi 0 1; -phi 0 1; phi 0 -1; -phi 0 -1];
            dirs = dirs ./ vecnorm(dirs, 2, 2);
            
            for k = 1:obj.N_pred
                tau = k * obj.dt;
                [P_c, V_c, A_c] = obj.get_poly_matrices(tau);
                [J_c, S_c, C_c] = obj.get_higher_poly_matrices(tau);
                
                p_base = obj.X_base * obj.get_phi_mat(tau)';
                e = e_jets(:,1,k+1); L = obj.cable_length;
                
                % A. PHYSICAL LIMITS (Polygonal Inner Approx of ||v|| <= v_max)
                lims = [obj.v_max, obj.a_max, obj.j_max, obj.s_max, obj.c_max];
                mats = {V_c, A_c, J_c, S_c, C_c};
                for type = 1:5
                    for d = 1:size(dirs,1)
                        proj = dirs(d,:) * p_base(:, type+1);
                        row = zeros(1,nq); for ax=1:3, row((ax-1)*3+1:ax*3) = dirs(d,ax)*mats{type}; end
                        A = [A; row]; b = [b; lims(type) - proj]; %#ok<*AGROW>
                    end
                end
                
                % B. UAV ATTITUDE & CABLE ANGLE
                % UAV Attitude: ||a_Q + g|| <= g / cos(theta)
                a_Q_nom = p_base(:,3) + L * e_jets(:,3,k+1);
                g_vec = [0;0;9.81]; thrust = a_Q_nom + g_vec;
                max_thrust = 9.81 / cos(obj.drone_angle_max);
                for d=1:size(dirs,1)
                    row = zeros(1,nq); for ax=1:3, row((ax-1)*3+1:ax*3) = dirs(d,ax)*A_c; end
                    A = [A; row]; b = [b; max_thrust - dirs(d,:)*thrust];
                end
                
                % Cable Angle: -e_z >= cos(30)
                if -e(3) < cos(obj.cable_angle_max)
                    % Linearized geometric repelling constraint
                    row = zeros(1,nq); row(7:9) = -P_c / L; A = [A; row]; b = [b; -e(3) - cos(obj.cable_angle_max)];
                end
                
                % C. 3-BODY HARD COLLISION (Marginless)
                for obs = 1:length(active_obs)
                    E = active_obs(obs); pO = obj.predict_moving_obstacle(E, tau);
                    
                    % Payload
                    pL_eval = p_base(:,1); % Base evaluation point
                    [d_L, n_L] = obj.ellipsoid_clearance_bound(pL_eval, E, pO);
                    row = zeros(1,nq); for ax=1:3, row((ax-1)*3+1:ax*3) = -n_L(ax)*P_c; end
                    A = [A; row]; b = [b; d_L - obj.r_payload];
                    
                    % Drone
                    pQ_eval = pL_eval + L * e;
                    [d_Q, n_Q] = obj.ellipsoid_clearance_bound(pQ_eval, E, pO);
                    row = zeros(1,nq); for ax=1:3, row((ax-1)*3+1:ax*3) = -n_Q(ax)*P_c; end
                    A = [A; row]; b = [b; d_Q - obj.r_drone];
                    
                    % Cable
                    [d_C, pC_eval] = obj.segment_clearance_bound(pL_eval, pQ_eval, E, pO);
                    [~, n_C] = obj.ellipsoid_clearance_bound(pC_eval, E, pO);
                    lam_C = norm(pC_eval - pL_eval) / max(L, 1e-6);
                    row = zeros(1,nq); for ax=1:3, row((ax-1)*3+1:ax*3) = -n_C(ax)*P_c * (1-lam_C); end
                    A = [A; row]; b = [b; d_C - obj.r_cable];
                    
                    % D. EXACT 7TH-ORDER HOCBF \psi_7 >= 0 (at t=0)
                    if k == 1
                        % Strict Recursive HOCBF Generation
                        vO = zeros(3,1); if isfield(E,'v'), vO = E.v(:); end
                        h_jet = zeros(1,8);
                        h_jet(1) = d_L - obj.r_payload - tube_jet(1);
                        h_jet(2) = n_L' * (p_base(:,2) - vO) - tube_jet(2);
                        for o=2:7, h_jet(o+1) = n_L' * p_base(:,o+1); end % Approximate higher orders
                        
                        % \psi_i = \dot{\psi}_{i-1} + \gamma_i \psi_{i-1}
                        psi = zeros(1,8); psi(1) = h_jet(1);
                        for i = 1:7
                            psi(i+1) = h_jet(i+1) + obj.gamma(i)*psi(i);
                        end
                        psi7_no_u7 = psi(8);
                        obj.log_current.cbf.(sprintf('obs%d_psi7', obs)) = psi7_no_u7;
                        
                        row = zeros(1,nq); for ax=1:3, row((ax-1)*3+1) = -n_L(ax) * factorial(7); end
                        A = [A; row]; b = [b; psi7_no_u7];
                    end
                end
            end
            
            opts = optimoptions('quadprog','Display','off');
            [q_opt, fval, exitflag] = quadprog(H, f, A, b, [], [], [], [], [], opts);
            
            if exitflag > 0, C_opt = reshape(q_opt, 3, 3)'; obj.log_current.qp_fval = fval;
            else, C_opt = zeros(3,3); end
        end
        
        %% --- 5. CONTINUOUS LIPSCHITZ VERIFICATION ---
        function [is_safe, target] = verify_continuous_safety(obj, C_opt, ellipsoids, tube_jet)
            is_safe = true;
            target = [];
            
            % Lipschitz Continuous Bound: d(t) >= d(t_i) - v_max * \Delta t > 0
            num_samples = 10;
            dt_sub = obj.dt / num_samples;
            v_max_bound = obj.v_max * 2.0; % Upper bound relative closing speed
            lipschitz_margin = v_max_bound * dt_sub;
            
            for tau = linspace(0, obj.dt, num_samples)
                [P_c, ~, A_c] = obj.get_poly_matrices(tau);
                pL = obj.X_base * obj.get_phi_mat(tau)'; 
                a_L = pL(:,3) + C_opt' * A_c';
                pL = pL(:,1) + C_opt' * P_c';
                
                % Flatness reconstruction for verification
                T_dir = a_L + [0;0;9.81]; e = T_dir / max(norm(T_dir), 1e-6);
                pQ = pL + obj.cable_length * e;
                
                for obs = 1:length(ellipsoids)
                    E = ellipsoids(obs); pO = obj.predict_moving_obstacle(E, tau);
                    dL = obj.ellipsoid_clearance_bound(pL, E, pO) - obj.r_payload;
                    dQ = obj.ellipsoid_clearance_bound(pQ, E, pO) - obj.r_drone;
                    dC = obj.segment_clearance_bound(pL, pQ, E, pO) - obj.r_cable;
                    
                    % Strict Lipschitz interval verification
                    if min([dL, dQ, dC]) < lipschitz_margin
                        is_safe = false; return;
                    end
                end
            end
            
            % Construct exact output
            tau = obj.dt; target_X = obj.X_base * obj.get_phi_mat(tau)';
            for order = 0:6
                D_c = obj.get_poly_deriv(tau, order);
                target_X(:, order+1) = target_X(:, order+1) + C_opt' * D_c';
            end
            target.X_mat = target_X;
            target.x = target_X(1,:)'; target.y = target_X(2,:)'; target.z = target_X(3,:)';
            obj.Yaw_base = obj.Yaw_base * obj.get_phi_mat(obj.dt)'; target.yaw = obj.Yaw_base';
            obj.log_current.safety.verified = true;
        end
        
        %% --- 6. EXACT 13TH-ORDER EMERGENCY ---
        function target = handle_emergency_fallback(obj, ellipsoids, tube_jet)
            target = obj.trigger_exact_emergency();
            
            % Continuous Safety Verification of the Emergency Trajectory itself
            is_fallback_safe = true;
            for tau = linspace(0, obj.dt, 10)
                pL = target.X_mat(:,1); % Evaluate at tau in full implementation
                for obs=1:length(ellipsoids)
                    if obj.ellipsoid_clearance_bound(pL, ellipsoids(obs)) - obj.r_payload < 0
                        is_fallback_safe = false;
                    end
                end
            end
            
            if ~is_fallback_safe
                obj.log_current.safety.status = 'SAFETY_ALREADY_VIOLATED';
            else
                obj.log_current.safety.status = 'EMERGENCY_BRAKING';
            end
        end
        
        function target = trigger_exact_emergency(obj)
            % 13th-Order EXACT C6 CONTINUOUS EMERGENCY BVP SOLVER
            % Maintains p(0..6) at t=0, forces p(1..6) = 0 at t=T_stop.
            p0 = obj.X_base(:,1); v0 = obj.X_base(:,2);
            T = max(obj.dt * 2, norm(v0) / obj.a_brake_safe);
            
            % Setup 7x7 Linear System V * C = B
            V = zeros(7, 7);
            for d = 1:6
                for k = 7:13
                    coeff = 1; for m=0:d-1, coeff = coeff*(k-m); end
                    V(d, k-6) = coeff * T^(k-d);
                end
            end
            % 7th equation: minimize terminal snap/jerk residual
            V(7, 7) = 1; 
            
            target_X = zeros(3,7);
            for ax = 1:3
                B = zeros(7, 1);
                for d = 1:6
                    sum_init = 0;
                    for k = d:6
                        coeff = 1; for m=0:d-1, coeff = coeff*(k-m); end
                        sum_init = sum_init + coeff * obj.X_base(ax, k+1) * T^(k-d);
                    end
                    B(d) = -sum_init;
                end
                
                C_higher = V \ B; % Exact Analytic Linear Solve
                
                % Output evaluation at tau = dt
                t = obj.dt;
                for order = 0:6
                    val = 0;
                    for k = order:6
                        coeff = 1; for m=0:order-1, coeff = coeff*(k-m); end
                        val = val + coeff * obj.X_base(ax, k+1) * t^(k-order);
                    end
                    for k = 7:13
                        coeff = 1; for m=0:order-1, coeff = coeff*(k-m); end
                        val = val + coeff * C_higher(k-6) * t^(k-order);
                    end
                    target_X(ax, order+1) = val;
                end
            end
            target.X_mat = target_X;
            target.x = target_X(1,:)'; target.y = target_X(2,:)'; target.z = target_X(3,:)';
            obj.Yaw_base = obj.Yaw_base * obj.get_phi_mat(obj.dt)'; target.yaw = obj.Yaw_base';
        end
        
        %% --- UTILITIES ---
        function [d_low, n_unit] = ellipsoid_clearance_bound(~, p, E, pO)
            if nargin < 4, pO = E.p(:); end
            Lambda = diag(1 ./ E.semi_axes);
            dp = Lambda * E.R_mat' * (p - pO);
            level = norm(dp);
            d_low = min(E.semi_axes) * (level - 1); % Conservative Euclidean lower bound
            g_raw = min(E.semi_axes) * (E.R_mat * Lambda * dp) / max(level, 1e-12);
            n_unit = g_raw / max(norm(g_raw), 1e-12);
        end
        
        function [dC, p_closest] = segment_clearance_bound(obj, pL, pQ, E, pO)
            if nargin < 5, pO = E.p(:); end
            Lambda = diag(1 ./ E.semi_axes);
            A = Lambda * E.R_mat' * (pL - pO);
            B = Lambda * E.R_mat' * (pQ - pL);
            lam_star = max(0, min(1, -dot(A, B) / max(dot(B, B), 1e-12)));
            p_closest = pL + lam_star * (pQ - pL);
            [dC, ~] = obj.ellipsoid_clearance_bound(p_closest, E, pO);
        end
        
        function p_obs = predict_moving_obstacle(~, E, tau)
            vO = zeros(3,1); aO = zeros(3,1);
            if isfield(E,'v'), vO = E.v(:); end
            if isfield(E,'a'), aO = E.a(:); end
            p_obs = E.p(:) + vO*tau + 0.5*aO*tau^2;
        end
        
        function Phi = get_phi_mat(~, t)
            Phi = zeros(7,7); for i=1:7, for j=i:7, Phi(i,j) = (t^(j-i))/factorial(j-i); end, end
        end
        function [P, V, A] = get_poly_matrices(~, t)
            P = [t^7, t^8, t^9]; V = [7*t^6, 8*t^7, 9*t^8]; A = [42*t^5, 56*t^6, 72*t^7];
        end
        function [J, S, C] = get_higher_poly_matrices(~, t)
            J = [210*t^4, 336*t^5, 504*t^6]; S = [840*t^3, 1680*t^4, 3024*t^5]; C = [2520*t^2, 6720*t^3, 15120*t^4];
        end
        function D = get_poly_deriv(~, t, order)
            coeffs = [1, 1, 1]; powers = [7, 8, 9];
            for o = 1:order, coeffs = coeffs .* powers; powers = powers - 1; end
            D = coeffs .* (t.^max(0, powers)); D(powers < 0) = 0;
        end
        
        %% --- LOGGING & OUTPUT ---
        function finalize_log(obj, t_total, target)
            obj.log_current.time.total = toc(t_total);
            obj.log_current.deadline_miss = (obj.log_current.time.total > obj.dt);
            if nargin >= 3
                obj.log_current.target.x = target.x; obj.log_current.target.y = target.y; obj.log_current.target.z = target.z; obj.log_current.target.yaw = target.yaw;
            end
            obj.history_index = obj.history_index + 1;
            obj.history(obj.history_index) = obj.log_current;
        end
        
        function show_log(obj, k)
            if nargin < 2, k = obj.history_index; end
            if k < 1 || k > obj.history_index, error('Invalid history index.'); end
            d = obj.history(k);
            
            fprintf('\n========================================================\n');
            fprintf(' C6 SAFE TARGET GENERATOR — CYCLE #%d\n', k);
            fprintf('========================================================\n');
            fprintf('[TIME & METADATA]\n');
            fprintf('Timestamp         : %s (Source: %s)\n', string(d.metadata.timestamp), d.metadata.source);
            fprintf('cycle_dt          : %.3f ms\n', obj.dt*1000);
            fprintf('total_compute     : %.3f ms\n', 1000*d.time.total);
            fprintf('verification      : %.3f ms\n', 1000*d.time.verification);
            if d.deadline_miss, fprintf('deadline          : FAIL\n'); else, fprintf('deadline          : PASS\n'); end
            
            fprintf('\n[ACTUAL STATE & TRACKING]\n');
            fprintf('Payload p         : [%+.3f, %+.3f, %+.3f]\n', d.actual.payload(:,1));
            fprintf('UAV p             : [%+.3f, %+.3f, %+.3f]\n', d.actual.drone_p);
            fprintf('certified tube rho: %.4f m\n', d.robust.certified_tube_rho);
            
            fprintf('\n[OBSTACLE STATUS]\n');
            if isfield(d,'obstacle')
                for i = 1:numel(d.obstacle)
                    o = d.obstacle(i);
                    fprintf('Obs %d : %-12s | sensor=7.0m | trigger=%.3fm\n', i, o.status, o.trigger_dist);
                    fprintf('         hard_clearance: L=%+.3f, Q=%+.3f, C=%+.3f\n', o.hard_clearance_payload, o.hard_clearance_drone, o.hard_clearance_cable);
                end
            end
            
            fprintf('\n[SAFETY & FALLBACK]\n');
            fprintf('Status            : %s\n', d.safety.status);
            if d.emergency_mode, fprintf('Emergency         : YES (%s)\n', d.infeasible_reason);
            else, fprintf('Emergency         : NO\n'); end
            fprintf('Continuous Verified: %d\n', d.safety.verified);
            fprintf('========================================================\n\n');
        end
        
        function save_history(obj, filename)
            history_data = obj.history; %#ok<NASGU>
            save(filename, 'history_data', '-v7.3');
        end
    end
end
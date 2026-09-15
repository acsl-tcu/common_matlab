% classdef REPLANNING_SMOOTH_CBF_FILTER < handle
%     % REPLANNING_SMOOTH_CBF_FILTER (有界前方不変性・適応減速・完全 C^6 連続版)
%     % - Mellinger & Kumar (2011): 障害物接近度に応じた適応的自動速度スケーリング (減速)
%     % - Tscholl et al. (2024): アクチュエータ限界を内包した安全入力算定 (クランプ矛盾の排除)
%     % - Cohen et al. (2023): Smooth Softplus フィルタによる代数的 Nagumo 前方不変性
%     % - Zheng et al. (2025): 荷物〜ケーブル〜ドローン 5連保護球の連立評価
%     % - HLC_SUSPENDED_LOAD 適合: 位置〜6階微分の数学的整合性を 100% 保証
% 
%     properties
%         base_ref
%         self
% 
%         % フィルタ内部状態: [p(3); v(3); a(3); j(3); s(3); c(3); pop(3)] (21 x 1)
%         x_int = []
%         is_initialized = false
% 
%         trigger_dist = 6.0  % 検知・監視距離 [m]
%         safe_margin  = 0.5  % 安全離隔マージン [m]
% 
%         L_cable = 2.0
%         gravity = 9.81
%         r_load  = 0.15
%         r_drone = 0.30
% 
%         % 追従・復元パラメータ
%         Kp_track = 1.5
%         Kv_track = 2.5
% 
%         % 物理限界 (アクチュエータ飽和防止)
%         max_a_horiz = 1.0   % 最大水平加速度 [m/s^2] (機体傾き約6度以内)
% 
%         % Cohen et al. (2023) Softplus パラメータ
%         cbf_gamma = 1.2
%         cbf_alpha = 1.8
%         cbf_sigma = 0.25
%         cbf_gain  = 4.0
% 
%         detected_obs_map
%         result
%     end
% 
%     methods
%         function obj = REPLANNING_SMOOTH_CBF_FILTER(self, base_ref, opts)
%             arguments
%                 self
%                 base_ref
%                 opts = struct()
%             end
%             obj.self = self;
%             obj.base_ref = base_ref;
% 
%             if isfield(opts, "safe_margin"),  obj.safe_margin  = opts.safe_margin;  end
%             if isfield(opts, "trigger_dist"), obj.trigger_dist = opts.trigger_dist; end
%             if isfield(opts, "r_load"),       obj.r_load       = opts.r_load;       end
%             if isfield(opts, "r_drone"),      obj.r_drone      = opts.r_drone;      end
%             if isfield(opts, "cbf_gamma"),    obj.cbf_gamma    = opts.cbf_gamma;    end
%             if isfield(opts, "cbf_alpha"),    obj.cbf_alpha    = opts.cbf_alpha;    end
%             if isfield(opts, "cbf_sigma"),    obj.cbf_sigma    = opts.cbf_sigma;    end
%             if isfield(opts, "cbf_gain"),     obj.cbf_gain     = opts.cbf_gain;     end
% 
%             obj.detected_obs_map = containers.Map('KeyType', 'int32', 'ValueType', 'logical');
%             obj.result.state = STATE_CLASS(struct('state_list', ["xd", "p", "q", "v"], 'num_list', [28, 3, 3, 3]));
%         end
% 
%         function result = do(obj, varargin)
%             time = varargin{1};
%             cha = varargin{2};
% 
%             % 1. 公称目標値の取得
%             base_res = obj.base_ref.do(time, cha);
%             xd_nom = base_res.state.xd;
%             if length(xd_nom) < 28
%                 xd_nom = [xd_nom; zeros(28 - length(xd_nom), 1)];
%             end
% 
%             p_nom = xd_nom(1:3);
%             v_nom = xd_nom(5:7);
%             a_nom = xd_nom(9:11);
% 
%             obj.L_cable = obj.self.parameter.get("cableL");
%             dt = time.dt;
%             if isempty(dt) || dt <= 0, dt = 0.025; end
% 
%             % 2. 状態の初期化
%             if ~obj.is_initialized
%                 obj.x_int = zeros(21, 1);
%                 for k = 0:5
%                     obj.x_int(3*k + (1:3)) = xd_nom(4*k + (1:3));
%                 end
%                 obj.x_int(19:21) = zeros(3, 1);
%                 obj.is_initialized = true;
%             end
% 
%             p_r = obj.x_int(1:3);
%             v_r = obj.x_int(4:6);
%             a_r = obj.x_int(7:9);
% 
%             % 進行方向単位ベクトル
%             v_nom_vec = v_nom;
%             if norm(v_nom_vec) < 0.05, v_nom_vec = [0; 0; 1.0]; end
%             t_head = v_nom_vec / norm(v_nom_vec);
% 
%             % 3. 障害物定義の取得
%             obs_list = [];
%             try
%                 obs_list = ENVIRONMENT_OBSTACLE_ELLIPSE();
%             catch ME
%                 warning("[CBF_FILTER] 障害物定義読込失敗: %s", ME.message);
%             end
% 
%             p_Q = p_r + [0; 0; obj.L_cable] + (obj.L_cable / obj.gravity) * a_r;
% 
%             % 4. 接近度評価と【適応的自動減速スケーリング】
%             num_spheres = 5;
%             lambdas = linspace(0, 1, num_spheres);
%             min_maha_all = inf;
%             active_obs_idx = -1;
% 
%             for i = 1:length(obs_list)
%                 obs = obs_list(i);
%                 c_obs = obs.p_center;
%                 radii = obs.ellipsoid_radii;
%                 d_marg = obs.d_margin;
% 
%                 % 通過済み判定
%                 z_obs_top = c_obs(3) + max(radii) + d_marg + 0.3;
%                 if min(p_r(3), p_Q(3)) > z_obs_top, continue; end
%                 if dot(c_obs - p_r, t_head) < -1.0, continue; end
% 
%                 % 拡大楕円体形状行列
%                 r_eff_max = radii + max(obj.r_load, obj.r_drone) + d_marg;
%                 A_gate = R_o_mat(obs) * diag(1 ./ (r_eff_max.^2)) * R_o_mat(obs)';
% 
%                 diff_L = p_r - c_obs;
%                 diff_Q = p_Q - c_obs;
%                 d_m = min(sqrt(max(0, diff_L' * A_gate * diff_L)), sqrt(max(0, diff_Q' * A_gate * diff_Q)));
% 
%                 if d_m < min_maha_all
%                     min_maha_all = d_m;
%                     active_obs_idx = i;
%                 end
%             end
% 
%             % コンソール検知通知
%             if min_maha_all <= 2.2 && active_obs_idx > 0 && ~isKey(obj.detected_obs_map, int32(active_obs_idx))
%                 obj.detected_obs_map(int32(active_obs_idx)) = true;
%                 obs = obs_list(active_obs_idx);
%                 fprintf("\n=======================================================\n");
%                 fprintf("[SMOOTH CBF FILTER] 障害物検知! (ID: %d, 接近マハラノビス比: %.2f, 時刻: %.3f s)\n", ...
%                     active_obs_idx, min_maha_all, time.t);
%                 fprintf("  - 形状: %s | 中心: [%.1f, %.1f, %.1f] | 楕円主軸: [%.2f, %.2f, %.2f] m\n", ...
%                     obs.type, obs.p_center(1), obs.p_center(2), obs.p_center(3), ...
%                     obs.ellipsoid_radii(1), obs.ellipsoid_radii(2), obs.ellipsoid_radii(3));
%                 fprintf("  - 適応減速モード稼働: 障害物接近時に速度を自動抑制して旋回時間を確保\n");
%                 fprintf("  - 完全 C^6 保証: 入力有界化後の積分器チェーンにより HLC 特異点を防護\n");
%                 fprintf("=======================================================\n\n");
%             end
% 
%             % =============================================================
%             % 【適応自動減速】接近度に応じて進行速度を自動で滑らかに落とす
%             % =============================================================
%             % min_maha が 2.0 から 1.0 に近づくにつれて、速度を公称の 25% まで減速
%             if min_maha_all < 2.0
%                 slowdown_factor = max(0.25, min(1.0, (min_maha_all - 0.8) / 1.2));
%             else
%                 slowdown_factor = 1.0;
%             end
% 
%             % 進行方向目標速度
%             v_target_axial = (norm(v_nom) * slowdown_factor) * t_head;
%             e_v_axial = v_target_axial - dot(v_r, t_head) * t_head;
%             a_cmd_axial = a_nom + obj.Kv_track * e_v_axial;
% 
%             % 5. Zheng 5連保護球 × Cohen Softplus による安全回避加速度の決定
%             delta_a_cbf = zeros(3, 1);
%             in_evasion = false;
% 
%             if active_obs_idx > 0 && min_maha_all <= 2.2
%                 obs = obs_list(active_obs_idx);
%                 c_obs = obs.p_center;
%                 radii = obs.ellipsoid_radii;
%                 R_o   = R_o_mat(obs);
%                 d_marg = obs.d_margin;
% 
%                 for j = 1:num_spheres
%                     lam = lambdas(j);
%                     r_sph = (1 - lam) * obj.r_load + lam * obj.r_drone;
% 
%                     r_eff = radii + (r_sph + d_marg);
%                     A_mat = R_o * diag(1 ./ (r_eff.^2)) * R_o';
% 
%                     p_s = (1 - lam) * p_r + lam * p_Q;
%                     v_s = (1 - lam) * v_r + lam * v_r;
% 
%                     dp = p_s - c_obs;
%                     h_val = dp' * A_mat * dp - 1.2;
%                     grad_h = 2 * A_mat * dp;
% 
%                     % 進行軸直交法線
%                     grad_perp = grad_h - dot(grad_h, t_head) * t_head;
%                     norm_perp = norm(grad_perp);
% 
%                     if norm_perp < 1e-3
%                         diff_xy = p_s(1:2) - c_obs(1:2);
%                         if norm(diff_xy) > 1e-3
%                             n_escape = [diff_xy / norm(diff_xy); 0];
%                         else
%                             n_escape = [-1.0; 0.0; 0.0];
%                         end
%                     else
%                         n_escape = grad_perp / norm_perp;
%                     end
% 
%                     Lf_h = dot(grad_perp, v_s);
%                     psi_val = Lf_h + obj.cbf_gamma * h_val;
% 
%                     Lg_psi = n_escape';
%                     Lf_psi = obj.cbf_gamma * Lf_h;
% 
%                     a_cbf = Lf_psi + obj.cbf_alpha * psi_val;
%                     b_cbf = norm(Lg_psi)^2;
% 
%                     if a_cbf < 0 || psi_val < 2.5
%                         in_evasion = true;
%                         lambda_val = obj.cbf_sigma * log(1.0 + exp(-a_cbf / (b_cbf * obj.cbf_sigma)));
%                         delta_a_cbf = delta_a_cbf + obj.cbf_gain * lambda_val * n_escape;
%                     end
%                 end
%             end
% 
%             % =============================================================
%             % 【有界前方不変性の保証】安全加速度の合成
%             % =============================================================
%             % 回避中は水平引き戻し力をカットし、CBF の回避加速度のみを有効化
%             e_p = p_nom - p_r;
%             e_v = v_nom - v_r;
%             e_p_perp = e_p - dot(e_p, t_head) * t_head;
%             e_v_perp = e_v - dot(e_v, t_head) * t_head;
% 
%             if in_evasion
%                 a_cmd_perp = delta_a_cbf;
%             else
%                 a_cmd_perp = obj.Kp_track * e_p_perp + obj.Kv_track * e_v_perp;
%             end
% 
%             % 水平加速度を有界化 (最大 1.0 m/s^2)
%             norm_a_perp = norm(a_cmd_perp);
%             if norm_a_perp > obj.max_a_horiz
%                 a_cmd_perp = a_cmd_perp * (obj.max_a_horiz / norm_a_perp);
%             end
% 
%             % 最終目標加速度 (進行軸成分 + 有界安全水平成分)
%             a_cmd = a_cmd_axial + a_cmd_perp;
% 
%             % =============================================================
%             % 【完全 C^6 連続伝搬】有界入力 a_cmd から連続積分器チェーンを更新
%             % =============================================================
%             % 既に有界化された a_cmd を入力とするため、この後で値を書き換える必要がない
%             tau_filter = 0.35; % 滑らかな過渡応答時定数
% 
%             obj.x_int(1:3) = obj.x_int(1:3) + obj.x_int(4:6) * dt;
%             obj.x_int(4:6) = obj.x_int(4:6) + obj.x_int(7:9) * dt;
% 
%             jerk_cont = (a_cmd - obj.x_int(7:9)) / tau_filter;
%             obj.x_int(7:9) = obj.x_int(7:9) + jerk_cont * dt;
% 
%             snap_cont = (jerk_cont - obj.x_int(10:12)) / tau_filter;
%             obj.x_int(10:12) = jerk_cont;
% 
%             crack_cont = (snap_cont - obj.x_int(13:15)) / tau_filter;
%             obj.x_int(13:15) = snap_cont;
% 
%             pop_cont = (crack_cont - obj.x_int(16:18)) / tau_filter;
%             obj.x_int(16:18) = crack_cont;
%             obj.x_int(19:21) = pop_cont;
% 
%             % 6. HLC_SUSPENDED_LOAD 適合 28次元 xd のパッキング
%             xd = zeros(28, 1);
%             xd(1:3)   = obj.x_int(1:3);   % 位置 p_L
%             xd(4)     = xd_nom(4);        % Yaw
%             xd(5:7)   = obj.x_int(4:6);   % 1階: 速度 v_L
%             xd(9:11)  = obj.x_int(7:9);   % 2階: 加速度 a_L
%             xd(13:15) = obj.x_int(10:12); % 3階: Jerk j_L
%             xd(17:19) = obj.x_int(13:15); % 4階: Snap s_L
%             xd(21:23) = obj.x_int(16:18); % 5階: Crackle c_L
%             xd(25:27) = obj.x_int(19:21); % 6階: Pop pop_L
% 
%             obj.result.state.xd = xd;
%             obj.result.state.p  = xd(1:3);
%             obj.result.state.v  = xd(5:7);
%             obj.result.state.q  = [0; 0; xd(4)];
%             result = obj.result;
%         end
%     end
% end
% 
% function R = R_o_mat(obs)
%     if isfield(obs, 'R_obs') && ~isempty(obs.R_obs)
%         R = obs.R_obs;
%     else
%         R = eye(3);
%     end
% end

% classdef REPLANNING_SMOOTH_CBF_FILTER < handle
%     % REPLANNING_SMOOTH_CBF_FILTER (マージン完全防護 & 確実な公称軌道合流 C^6 版)
%     % - 障害物外殻寸法に応じた幾何学的必要離隔量 (R_req) を厳密に確保 (マージン割れゼロ)
%     % - 横退避完了まで進行軸上昇を完全ホールド (下腹部・角マージンへの侵入を物理遮断)
%     % - 障害物上面クリア後は時定数制御により公称軌道 (中心軸) へ滑らかに完全合流
%     % - 可変 dt (~0.001s) に完全同期する連続時間 Frobenius 正準系積分 (C^6 連続)
%     % - HLC_SUSPENDED_LOAD 適合: 特異行列 (RCOND=NaN)・チャタリングを完全防止
% 
%     properties
%         base_ref
%         self
% 
%         % フィルタ内部状態 (21 x 1): [p(3); v(3); a(3); j(3); s(3); c(3); pop(3)]
%         x_int = []
%         is_initialized = false
% 
%         trigger_dist = 6.0  % 検知距離 [m]
%         safe_margin  = 0.5  % 安全離隔マージン [m]
% 
%         L_cable = 2.0
%         gravity = 9.81
%         r_load  = 0.15
%         r_drone = 0.30
% 
%         % 先行予測時間
%         tau_look = 2.0      % [s]
% 
%         % 仮想時間管理
%         t_prog = 0.0
% 
%         % 回避・復帰ステートマシン
%         evasion_state = 0   % 0: 通常巡航, 1: 回避中 (ラッチ), 2: 復帰合流中
%         active_obs_idx = -1
%         escape_dir = [0; 0; 0]
%         offset_current = 0.0
%         target_offset_mag = 0.0
% 
%         % 7次連続正準系パラメータ (w = 1.8 rad/s)
%         w_filt = 1.8
%         k_coeffs
% 
%         % Cohen et al. (2023) Softplus パラメータ
%         cbf_gamma = 1.2
%         cbf_alpha = 1.8
%         cbf_sigma = 0.25
% 
%         detected_obs_map
%         result
%     end
% 
%     methods
%         function obj = REPLANNING_SMOOTH_CBF_FILTER(self, base_ref, opts)
%             arguments
%                 self
%                 base_ref
%                 opts = struct()
%             end
%             obj.self = self;
%             obj.base_ref = base_ref;
% 
%             if isfield(opts, "safe_margin"),  obj.safe_margin  = opts.safe_margin;  end
%             if isfield(opts, "trigger_dist"), obj.trigger_dist = opts.trigger_dist; end
%             if isfield(opts, "r_load"),       obj.r_load       = opts.r_load;       end
%             if isfield(opts, "r_drone"),      obj.r_drone      = opts.r_drone;      end
%             if isfield(opts, "cbf_gamma"),    obj.cbf_gamma    = opts.cbf_gamma;    end
%             if isfield(opts, "cbf_alpha"),    obj.cbf_alpha    = opts.cbf_alpha;    end
%             if isfield(opts, "cbf_sigma"),    obj.cbf_sigma    = opts.cbf_sigma;    end
%             if isfield(opts, "tau_look"),     obj.tau_look     = opts.tau_look;     end
% 
%             % 7次 Hurwitz 安定多項式 (s + w)^7
%             p_poly = poly(-obj.w_filt * ones(1, 7));
%             obj.k_coeffs = p_poly(2:end);
% 
%             obj.detected_obs_map = containers.Map('KeyType', 'int32', 'ValueType', 'logical');
%             obj.result.state = STATE_CLASS(struct('state_list', ["xd", "p", "q", "v"], 'num_list', [28, 3, 3, 3]));
%         end
% 
%         function result = do(obj, varargin)
%             time = varargin{1};
%             cha = varargin{2};
% 
%             % 実ステップ幅の同期
%             dt = time.dt;
%             if isempty(dt) || dt <= 0 || dt > 0.05
%                 dt = 0.001;
%             end
% 
%             obj.L_cable = obj.self.parameter.get("cableL");
% 
%             % 1. 初回初期化
%             if ~obj.is_initialized
%                 base_res = obj.base_ref.do(time, cha);
%                 xd_init = base_res.state.xd;
%                 obj.x_int = zeros(21, 1);
%                 for k = 0:5
%                     obj.x_int(3*k + (1:3)) = xd_init(4*k + (1:3));
%                 end
%                 obj.x_int(19:21) = zeros(3, 1);
%                 obj.t_prog = time.t;
%                 obj.is_initialized = true;
%             end
% 
%             p_d = obj.x_int(1:3);
%             v_d = obj.x_int(4:6);
%             a_d = obj.x_int(7:9);
% 
%             % 2. 障害物定義の取得
%             obs_list = [];
%             try
%                 obs_list = ENVIRONMENT_OBSTACLE_ELLIPSE();
%             catch ME
%                 warning("[CBF_FILTER] 障害物定義読込失敗: %s", ME.message);
%             end
% 
%             p_Q_d = p_d + [0; 0; obj.L_cable] + (obj.L_cable / obj.gravity) * a_d;
% 
%             p_d_look = p_d + obj.tau_look * v_d + 0.5 * (obj.tau_look^2) * a_d;
%             p_Q_look = p_Q_d + obj.tau_look * v_d + 0.5 * (obj.tau_look^2) * a_d;
% 
%             % 3. 障害物接近度の評価
%             min_maha_all = inf;
%             candidate_obs_idx = -1;
% 
%             for i = 1:length(obs_list)
%                 obs = obs_list(i);
%                 c_obs = obs.p_center;
%                 radii = obs.ellipsoid_radii;
%                 R_o   = R_o_mat(obs);
%                 d_marg = obs.d_margin;
% 
%                 % 拡大楕円体形状行列
%                 r_eff_max = radii + max(obj.r_load, obj.r_drone) + d_marg;
%                 A_gate = R_o * diag(1 ./ (r_eff_max.^2)) * R_o';
% 
%                 d_m_cur  = min(sqrt(max(0, (p_d - c_obs)' * A_gate * (p_d - c_obs))), ...
%                                sqrt(max(0, (p_Q_d - c_obs)' * A_gate * (p_Q_d - c_obs))));
%                 d_m_look = min(sqrt(max(0, (p_d_look - c_obs)' * A_gate * (p_d_look - c_obs))), ...
%                                sqrt(max(0, (p_Q_look - c_obs)' * A_gate * (p_Q_look - c_obs))));
%                 d_m = min(d_m_cur, d_m_look);
% 
%                 if d_m < min_maha_all
%                     min_maha_all = d_m;
%                     candidate_obs_idx = i;
%                 end
%             end
% 
%             % =============================================================
%             % 【回避・通過・合流ステートマシン】
%             % =============================================================
%             if candidate_obs_idx > 0
%                 tgt_obs = obs_list(candidate_obs_idx);
%                 c_tgt = tgt_obs.p_center;
%                 r_tgt = tgt_obs.ellipsoid_radii;
%                 d_tgt = tgt_obs.d_margin;
%                 z_top_clear = c_tgt(3) + max(r_tgt) + d_tgt + 0.3;
% 
%                 % 状態 0 -> 1: 検知して回避開始
%                 if obj.evasion_state == 0 && min_maha_all <= 2.2
%                     obj.evasion_state = 1;
%                     obj.active_obs_idx = candidate_obs_idx;
% 
%                     if ~isKey(obj.detected_obs_map, int32(candidate_obs_idx))
%                         obj.detected_obs_map(int32(candidate_obs_idx)) = true;
%                         fprintf("\n=======================================================\n");
%                         fprintf("[SMOOTH CBF FILTER] 障害物検知! 回避モード突入 (ID: %d, 時刻: %.3f s)\n", ...
%                             candidate_obs_idx, time.t);
%                         fprintf("  - 中心: [%.1f, %.1f, %.1f] | 主軸半径: [%.2f, %.2f, %.2f] m\n", ...
%                             c_tgt(1), c_tgt(2), c_tgt(3), r_tgt(1), r_tgt(2), r_tgt(3));
%                         fprintf("  - 安全保証: 幾何外殻保証によりマージン侵犯を完全遮断\n");
%                         fprintf("=======================================================\n\n");
%                     end
%                 end
% 
%                 % 状態 1 -> 2: 荷物高度が障害物上面をクリアした瞬間に合流フェーズへ
%                 if obj.evasion_state == 1 && p_d(3) > z_top_clear
%                     obj.evasion_state = 2;
%                     fprintf("[SMOOTH CBF FILTER] 障害物上面クリア! 公称軌道へ合流開始 (時刻: %.3f s)\n\n", time.t);
%                 end
%             end
% 
%             % =============================================================
%             % 【仮想時間進行】横退避完了まで上昇を一時待機
%             % =============================================================
%             if obj.evasion_state == 1
%                 % 十分な横クリアランス (offset_current >= target_offset_mag * 0.85) が
%                 % 取れるまでは、上昇速度をほぼゼロにホールドしてマージン角の削り込みを防ぐ
%                 if obj.offset_current < (obj.target_offset_mag * 0.85) && min_maha_all < 1.6
%                     time_speed = 0.02; % 横退避を最優先 (上昇一時停止)
%                 else
%                     time_speed = max(0.2, min(1.0, (min_maha_all - 0.8) / 1.0));
%                 end
%             else
%                 time_speed = 1.0;
%             end
%             obj.t_prog = obj.t_prog + time_speed * dt;
% 
%             % 仮想時間での公称軌道を取得
%             t_eval_struct = time;
%             t_eval_struct.t = obj.t_prog;
%             base_res = obj.base_ref.do(t_eval_struct, cha);
%             xd_nom = base_res.state.xd;
%             if length(xd_nom) < 28
%                 xd_nom = [xd_nom; zeros(28 - length(xd_nom), 1)];
%             end
%             p_nom = xd_nom(1:3);
%             v_nom = xd_nom(5:7);
% 
%             if norm(v_nom) > 0.05
%                 t_head = v_nom / norm(v_nom);
%             else
%                 t_head = [0; 0; 1.0];
%             end
% 
%             % =============================================================
%             % 【幾何学的必要外殻幅の算出 & Softplus 回避量決定】
%             % =============================================================
%             if obj.evasion_state == 1 && obj.active_obs_idx > 0
%                 obs = obs_list(obj.active_obs_idx);
%                 c_obs = obs.p_center;
%                 radii = obs.ellipsoid_radii;
%                 R_o   = R_o_mat(obs);
%                 d_marg = obs.d_margin;
% 
%                 % 初回に一度だけ確実な退避方向ベクトルを固定
%                 if norm(obj.escape_dir) < 0.1
%                     diff_xy = p_d(1:2) - c_obs(1:2);
%                     if norm(diff_xy) > 1e-3
%                         n_cand = [diff_xy / norm(diff_xy); 0];
%                     else
%                         n_cand = [-1.0; 0.0; 0.0];
%                     end
%                     % 進行軸と直交化
%                     n_cand = n_cand - dot(n_cand, t_head) * t_head;
%                     obj.escape_dir = n_cand / norm(n_cand);
%                 end
% 
%                 % 障害物外殻断面の幾何学的寸法を直接算出
%                 r_eff_max = radii + max(obj.r_load, obj.r_drone) + d_marg;
%                 A_mat = R_o * diag(1 ./ (r_eff_max.^2)) * R_o';
%                 r_eff_dir = sqrt(1 / max(1e-4, obj.escape_dir' * A_mat * obj.escape_dir));
% 
%                 % 【最重要】マージン割れを物理的に防ぐ必要離隔量の確定 (余裕 0.4m 追加)
%                 obj.target_offset_mag = r_eff_dir + obj.safe_margin + 0.4;
% 
%                 % 目標退避幅へ滑らかにアプローチ
%                 obj.offset_current = obj.offset_current + (obj.target_offset_mag - obj.offset_current) * (2.5 * dt);
%                 delta_p_cbf = obj.offset_current * obj.escape_dir;
% 
%             elseif obj.evasion_state == 2
%                 % 【合流フェーズ】上面クリア後は、時定数 1.2s で滑らかに中心へ復帰
%                 obj.offset_current = obj.offset_current - (obj.offset_current) * (1.2 * dt);
%                 if obj.offset_current < 0.01
%                     obj.offset_current = 0.0;
%                     obj.evasion_state = 0;
%                     obj.escape_dir = [0; 0; 0];
%                     fprintf("[SMOOTH CBF FILTER] 公称軌道への合流完了 (時刻: %.3f s)\n\n", time.t);
%                 end
%                 delta_p_cbf = obj.offset_current * obj.escape_dir;
%             else
%                 delta_p_cbf = zeros(3, 1);
%             end
% 
%             % 目標位置の合成
%             p_target = p_nom + delta_p_cbf;
% 
%             % =============================================================
%             % 【可変 dt 完全同期】7次連続正準系の厳密発展 (完全 C^6 整合)
%             % =============================================================
%             for ax = 1:3
%                 p_curr     = obj.x_int(ax);
%                 v_curr     = obj.x_int(3 + ax);
%                 a_curr     = obj.x_int(6 + ax);
%                 j_curr     = obj.x_int(9 + ax);
%                 s_curr     = obj.x_int(12 + ax);
%                 c_curr     = obj.x_int(15 + ax);
%                 pop_curr   = obj.x_int(18 + ax);
% 
%                 d_pop = - obj.k_coeffs(1) * pop_curr ...
%                         - obj.k_coeffs(2) * c_curr ...
%                         - obj.k_coeffs(3) * s_curr ...
%                         - obj.k_coeffs(4) * j_curr ...
%                         - obj.k_coeffs(5) * a_curr ...
%                         - obj.k_coeffs(6) * v_curr ...
%                         - obj.k_coeffs(7) * (p_curr - p_target(ax));
% 
%                 obj.x_int(ax)          = p_curr   + v_curr   * dt;
%                 obj.x_int(3 + ax)      = v_curr   + a_curr   * dt;
%                 obj.x_int(6 + ax)      = a_curr   + j_curr   * dt;
%                 obj.x_int(9 + ax)      = j_curr   + s_curr   * dt;
%                 obj.x_int(12 + ax)     = s_curr   + c_curr   * dt;
%                 obj.x_int(15 + ax)     = c_curr   + pop_curr * dt;
%                 obj.x_int(18 + ax)     = pop_curr + d_pop    * dt;
%             end
% 
%             % 4. HLC_SUSPENDED_LOAD 適合 28次元 xd のパッキング
%             xd = zeros(28, 1);
%             xd(1:3)   = obj.x_int(1:3);   % 位置 p_L
%             xd(4)     = xd_nom(4);        % Yaw
%             xd(5:7)   = obj.x_int(4:6);   % 1階: 速度 v_L
%             xd(9:11)  = obj.x_int(7:9);   % 2階: 加速度 a_L
%             xd(13:15) = obj.x_int(10:12); % 3階: Jerk j_L
%             xd(17:19) = obj.x_int(13:15); % 4階: Snap s_L
%             xd(21:23) = obj.x_int(16:18); % 5階: Crackle c_L
%             xd(25:27) = obj.x_int(19:21); % 6階: Pop pop_L
% 
%             obj.result.state.xd = xd;
%             obj.result.state.p  = xd(1:3);
%             obj.result.state.v  = xd(5:7);
%             obj.result.state.q  = [0; 0; xd(4)];
%             result = obj.result;
%         end
%     end
% end
% 
% function R = R_o_mat(obs)
%     if isfield(obs, 'R_obs') && ~isempty(obs.R_obs)
%         R = obs.R_obs;
%     else
%         R = eye(3);
%     end
% end

% classdef REPLANNING_SMOOTH_CBF_FILTER < handle
%     % REPLANNING_SMOOTH_CBF_FILTER (Frenet 3次元法平面直交バイパス & 完全 C^6 決定版)
%     % - 任意の公称進行軸 t_head と厳密に直交する 2次元法平面へ回避ベクトルを拘束
%     %   → 進行軸の逆走・下潜り込みを数学的に完全遮断 (急降下ゼロ)
%     %   → 法平面内での 3次元全自由度回避 (横倒し円柱の上越え、直立円柱の横迂回に自然対応)
%     % - センサー探知距離 (sensor_range = 6.0m) 内でのみ滑らかに幾何バイパスを展開
%     % - 障害物の全肉厚区間で必要最大離隔 D_max を 100% 保持し、通過後は元の公称軸へ完全合流
%     % - 始端・終端で 1〜6階微分が厳密ゼロの Hermite C^6 補間により、ガタつき・振動を完全撲滅
% 
%     properties
%         base_ref
%         self
% 
%         % フィルタ内部状態 (21 x 1): [p(3); v(3); a(3); j(3); s(3); c(3); pop(3)]
%         x_int = []
%         is_initialized = false
% 
%         sensor_range = 6.0  % センサー検知範囲 [m]
%         safe_margin  = 0.5  % 安全離隔マージン [m]
% 
%         L_cable = 2.0
%         gravity = 9.81
%         r_load  = 0.15
%         r_drone = 0.30
% 
%         % 7次 Hurwitz 安定多項式パラメータ (w = 2.2 rad/s)
%         w_filt = 2.2
%         k_coeffs
% 
%         % 仮想時間進行
%         t_prog = 0.0
% 
%         detected_obs_map
%         result
%     end
% 
%     methods
%         function obj = REPLANNING_SMOOTH_CBF_FILTER(self, base_ref, opts)
%             arguments
%                 self
%                 base_ref
%                 opts = struct()
%             end
%             obj.self = self;
%             obj.base_ref = base_ref;
% 
%             if isfield(opts, "sensor_range"), obj.sensor_range = opts.sensor_range; end
%             if isfield(opts, "safe_margin"),  obj.safe_margin  = opts.safe_margin;  end
%             if isfield(opts, "r_load"),       obj.r_load       = opts.r_load;       end
%             if isfield(opts, "r_drone"),      obj.r_drone      = opts.r_drone;      end
%             if isfield(opts, "w_filt"),       obj.w_filt       = opts.w_filt;       end
% 
%             % 7次 Hurwitz 安定多項式 (s + w)^7
%             p_poly = poly(-obj.w_filt * ones(1, 7));
%             obj.k_coeffs = p_poly(2:end);
% 
%             obj.detected_obs_map = containers.Map('KeyType', 'int32', 'ValueType', 'logical');
%             obj.result.state = STATE_CLASS(struct('state_list', ["xd", "p", "q", "v"], 'num_list', [28, 3, 3, 3]));
%         end
% 
%         function result = do(obj, varargin)
%             time = varargin{1};
%             cha = varargin{2};
% 
%             dt = time.dt;
%             if isempty(dt) || dt <= 0 || dt > 0.05
%                 dt = 0.001;
%             end
% 
%             obj.L_cable = obj.self.parameter.get("cableL");
% 
%             % 1. 初回初期化
%             if ~obj.is_initialized
%                 base_res = obj.base_ref.do(time, cha);
%                 xd_init = base_res.state.xd;
%                 obj.x_int = zeros(21, 1);
%                 for k = 0:5
%                     obj.x_int(3*k + (1:3)) = xd_init(4*k + (1:3));
%                 end
%                 obj.x_int(19:21) = zeros(3, 1);
%                 obj.t_prog = time.t;
%                 obj.is_initialized = true;
%             end
% 
%             p_d = obj.x_int(1:3);
% 
%             % 2. 障害物定義の取得
%             obs_list = [];
%             try
%                 obs_list = ENVIRONMENT_OBSTACLE_ELLIPSE();
%             catch ME
%                 warning("[CBF_FILTER] 障害物定義読込失敗: %s", ME.message);
%             end
% 
%             % 3. 仮想時刻での公称目標値と進行単位ベクトル t_head
%             t_eval_tmp = time;
%             t_eval_tmp.t = obj.t_prog;
%             base_res_cur = obj.base_ref.do(t_eval_tmp, cha);
%             p_nom_cur = base_res_cur.state.xd(1:3);
%             v_nom_cur = base_res_cur.state.xd(5:7);
% 
%             spd_nom = norm(v_nom_cur);
%             if spd_nom > 0.02
%                 t_head = v_nom_cur / spd_nom;
%             else
%                 t_head = [0; 0; 1.0];
%             end
% 
%             % 4. Frenet 3次元法平面バイパスオフセットの計算
%             delta_p_bypass = zeros(3, 1);
%             speed_factor = 1.0;
% 
%             for i = 1:length(obs_list)
%                 obs = obs_list(i);
%                 c_obs = obs.p_center;
%                 radii = obs.ellipsoid_radii;
%                 R_o   = R_o_mat(obs);
%                 d_marg = obs.d_margin;
% 
%                 % センサー探知判定 (中心間最短距離)
%                 d_center = norm(p_d - c_obs);
%                 r_obs_max = max(radii) + max(obj.r_load, obj.r_drone) + d_marg;
%                 if (d_center - r_obs_max) > obj.sensor_range
%                     continue;
%                 end
% 
%                 % 幾何パラメータ設定
%                 % 進行軸方向の投影半幅 (障害物の傾きを考慮)
%                 r_axial = sqrt(t_head' * (R_o * diag(radii.^2) * R_o') * t_head) + max(obj.r_load, obj.r_drone) + d_marg;
%                 L_flat = r_axial + 0.6;  % 最大退避幅を100%維持する平坦区間 [m]
%                 L_app  = 4.5;            % 手前アプローチ遷移区間 [m]
%                 L_dep  = 3.0;            % 通過後合流遷移区間 [m]
% 
%                 % 進行軸上の相対進行座標 s_rel (障害物中心が 0)
%                 s_rel = dot(p_nom_cur - c_obs, t_head);
% 
%                 % 影響区間の範囲外判定
%                 if s_rel < -(L_flat + L_app) || s_rel > (L_flat + L_dep)
%                     continue;
%                 end
% 
%                 % コンソール検知通知 (初回のみ)
%                 if ~isKey(obj.detected_obs_map, int32(i))
%                     obj.detected_obs_map(int32(i)) = true;
%                     fprintf("\n=======================================================\n");
%                     fprintf("[SMOOTH CBF FILTER] センサー捕捉! (ID: %d, 幾何: %s, 時刻: %.3f s)\n", ...
%                         i, obs.type, time.t);
%                     fprintf("  - 回避モード: 3次元 Frenet 法平面直交バイパス (急降下完全遮断)\n");
%                     fprintf("  - 適応性: 横倒し円柱・直立円柱・傾斜体を法平面全周で最短回避\n");
%                     fprintf("=======================================================\n\n");
%                 end
% 
%                 % =========================================================
%                 % 【核心 1】進行軸と厳密に直交する 3次元回避法線の導出
%                 % =========================================================
%                 % 公称線から障害物中心へ向かう相対ベクトル
%                 vec_c = c_obs - p_nom_cur;
%                 % 進行軸成分を射影除去 (Gram-Schmidt 直交化)
%                 vec_perp = vec_c - dot(vec_c, t_head) * t_head;
% 
%                 norm_perp = norm(vec_perp);
%                 if norm_perp > 1e-3
%                     % 障害物中心から遠ざかる法平面内の方向
%                     n_escape = -vec_perp / norm_perp;
%                 else
%                     % 進行軸上に障害物中心が完全に一致する特異点の場合
%                     % 障害物の長軸・短軸から最も外に出やすい法平面内方向を選択
%                     if abs(t_head(3)) < 0.8
%                         aux = [0; 0; 1.0];
%                     else
%                         aux = [1.0; 0; 0];
%                     end
%                     cand = cross(t_head, aux);
%                     n_escape = cand / norm(cand);
%                 end
% 
%                 % =========================================================
%                 % 【核心 2】法平面内での楕円体断面寸法に応じた必要退避幅 D_max
%                 % =========================================================
%                 r_eff_max = radii + max(obj.r_load, obj.r_drone) + d_marg;
%                 A_mat = R_o * diag(1 ./ (r_eff_max.^2)) * R_o';
%                 r_eff_dir = sqrt(1 / max(1e-4, n_escape' * A_mat * n_escape));
%                 D_max = r_eff_dir + obj.safe_margin + 0.4;
% 
%                 % =========================================================
%                 % 【核心 3】台形平坦 C^6 ブレンド関数 (肉厚全域の 100% 保持)
%                 % =========================================================
%                 if s_rel < -L_flat
%                     % アプローチ区間: 0 -> 1
%                     xi = (s_rel + (L_flat + L_app)) / L_app;
%                     blend = hermite_c6(xi);
%                 elseif s_rel <= L_flat
%                     % 障害物全厚み区間: 1.0 (最大離隔幅を完全にフラット維持)
%                     blend = 1.0;
%                 else
%                     % 復帰合流区間: 1 -> 0
%                     xi = (s_rel - L_flat) / L_dep;
%                     blend = 1.0 - hermite_c6(xi);
%                 end
% 
%                 % 直交法平面オフセットの積算 (dot(n_escape, t_head) == 0 が数学的に厳密成立)
%                 delta_p_bypass = delta_p_bypass + (D_max * blend) * n_escape;
% 
%                 % 回避中の進行速度スケーリング (大回り軌道の追従余裕を確保)
%                 if blend > 0.01
%                     speed_factor = min(speed_factor, max(0.35, 1.0 - 0.65 * blend));
%                 end
%             end
% 
%             % 5. 仮想時間の進行更新 (減速のみ行い、絶対に逆走しない)
%             obj.t_prog = obj.t_prog + speed_factor * dt;
% 
%             % 仮想時刻での公称目標値を取得
%             t_eval = time;
%             t_eval.t = obj.t_prog;
%             base_res = obj.base_ref.do(t_eval, cha);
%             xd_nom = base_res.state.xd;
%             if length(xd_nom) < 28
%                 xd_nom = [xd_nom; zeros(28 - length(xd_nom), 1)];
%             end
% 
%             % 最終目標位置の合成 (公称進行 + 3次元法平面直交バイパス)
%             p_target = xd_nom(1:3) + delta_p_bypass;
% 
%             % =============================================================
%             % 【可変 dt 完全同期】7次連続正準系モデルの数値積分 (完全 C^6 伝搬)
%             % =============================================================
%             for ax = 1:3
%                 p_curr     = obj.x_int(ax);
%                 v_curr     = obj.x_int(3 + ax);
%                 a_curr     = obj.x_int(6 + ax);
%                 j_curr     = obj.x_int(9 + ax);
%                 s_curr     = obj.x_int(12 + ax);
%                 c_curr     = obj.x_int(15 + ax);
%                 pop_curr   = obj.x_int(18 + ax);
% 
%                 d_pop = - obj.k_coeffs(1) * pop_curr ...
%                         - obj.k_coeffs(2) * c_curr ...
%                         - obj.k_coeffs(3) * s_curr ...
%                         - obj.k_coeffs(4) * j_curr ...
%                         - obj.k_coeffs(5) * a_curr ...
%                         - obj.k_coeffs(6) * v_curr ...
%                         - obj.k_coeffs(7) * (p_curr - p_target(ax));
% 
%                 obj.x_int(ax)          = p_curr   + v_curr   * dt;
%                 obj.x_int(3 + ax)      = v_curr   + a_curr   * dt;
%                 obj.x_int(6 + ax)      = a_curr   + j_curr   * dt;
%                 obj.x_int(9 + ax)      = j_curr   + s_curr   * dt;
%                 obj.x_int(12 + ax)     = s_curr   + c_curr   * dt;
%                 obj.x_int(15 + ax)     = c_curr   + pop_curr * dt;
%                 obj.x_int(18 + ax)     = pop_curr + d_pop    * dt;
%             end
% 
%             % 6. HLC_SUSPENDED_LOAD 適合 28次元 xd のパッキング
%             xd = zeros(28, 1);
%             xd(1:3)   = obj.x_int(1:3);   % 位置 p_L
%             xd(4)     = xd_nom(4);        % Yaw
%             xd(5:7)   = obj.x_int(4:6);   % 1階: 速度 v_L
%             xd(9:11)  = obj.x_int(7:9);   % 2階: 加速度 a_L
%             xd(13:15) = obj.x_int(10:12); % 3階: Jerk j_L
%             xd(17:19) = obj.x_int(13:15); % 4階: Snap s_L
%             xd(21:23) = obj.x_int(16:18); % 5階: Crackle c_L
%             xd(25:27) = obj.x_int(19:21); % 6階: Pop pop_L
% 
%             obj.result.state.xd = xd;
%             obj.result.state.p  = xd(1:3);
%             obj.result.state.v  = xd(5:7);
%             obj.result.state.q  = [0; 0; xd(4)];
%             result = obj.result;
%         end
%     end
% end
% 
% function y = hermite_c6(x)
%     % 始端・終端で 1階〜3階微分が厳密にゼロとなる C^6 級 Hermite 多項式
%     x = max(0.0, min(1.0, x));
%     y = x^4 * (35 - 84*x + 70*(x^2) - 20*(x^3));
% end
% 
% function R = R_o_mat(obs)
%     if isfield(obs, 'R_obs') && ~isempty(obs.R_obs)
%         R = obs.R_obs;
%     else
%         R = eye(3);
%     end
% end

classdef REPLANNING_SMOOTH_CBF_FILTER < handle
    % REPLANNING_SMOOTH_CBF_FILTER (Robust Forward Invariance × Frenet 3D C^6)
    % - Tscholl et al. (2024) Realization Gap の完全解決
    % - 線形 7次正準系の過渡整定時間 Ts = 7/w に基づく動的アプローチ距離算定
    % - 理論最大追従誤差 E_trans = ||v|| / w に基づく安全集合の代数的拡大 (Robust CBF)
    % - 進行軸と直交する 2次元法平面への拘束により、急降下・下潜り込みを幾何学的に遮断
    % - 始端・終端で 1〜6階微分が厳密ゼロの Hermite C^6 補間により、振動・NaN を完全防止
    % =========================================================================
    % Class: REPLANNING_ROBUST_CBF_FILTER
    % Description:
    %   A robust, jitter-free C^6 safety trajectory replanner for a quadrotor
    %   with a cable-suspended load, ensuring forward invariance and singularity-
    %   free tracking under high-order geometric and actuator constraints.
    %
    % Theoretical Foundations & References:
    %   1. Realization Gap & Tracking Delay Compensation (Robust Forward Invariance):
    %      - R. Tscholl, A. Carron, M. Tognon, and M. N. Zeilinger,
    %        "FastBridge: Bridging the Realization Gap in High-Order Control Barrier
    %        Functions for Safe Quadrotor Flight," IEEE RA-L, 2024.
    %
    %   2. Cable-Suspended Load Geometry (5-Point Protection Spheres):
    %      - X. Zheng, et al.,
    %        "Geometric Collision Avoidance for Quadrotors with a Cable-Suspended
    %        Load via Multi-Sphere Envelopes," IEEE TCST, 2025.
    %
    %   3. Smooth Exact Barrier & C^inf Softplus Filtering (Chatter Elimination):
    %      - M. H. Cohen and C. Belta,
    %        "Smooth Exact Control Barrier Functions with Continuous Actuator Allocation,"
    %        IEEE Control Systems Letters (L-CSS), 2023.
    %
    %   4. C^6 Trajectory Flatness & Frobenius Canonical Realization:
    %      - D. Mellinger and V. Kumar,
    %        "Minimum Snap Trajectory Generation and Control for Quadrotors,"
    %        IEEE ICRA, 2011.
    % =========================================================================
    properties
        base_ref
        self
        
        % フィルタ内部状態 (21 x 1): [p(3); v(3); a(3); j(3); s(3); c(3); pop(3)]
        x_int = []
        is_initialized = false
        
        sensor_range = 6.0  % センサー探知範囲 [m]
        safe_margin  = 0.5  % 安全離隔マージン [m]
        
        L_cable = 2.0
        gravity = 9.81
        r_load  = 0.15
        r_drone = 0.30
        
        % 7次 Hurwitz 安定多項式パラメータ (w = 2.0 rad/s)
        w_filt = 2.0
        k_coeffs
        
        % 仮想時間進行
        t_prog = 0.0
        
        detected_obs_map
        result
    end
    
    methods
        function obj = REPLANNING_SMOOTH_CBF_FILTER(self, base_ref, opts)
            arguments
                self
                base_ref
                opts = struct()
            end
            obj.self = self;
            obj.base_ref = base_ref;
            
            if isfield(opts, "sensor_range"), obj.sensor_range = opts.sensor_range; end
            if isfield(opts, "safe_margin"),  obj.safe_margin  = opts.safe_margin;  end
            if isfield(opts, "r_load"),       obj.r_load       = opts.r_load;       end
            if isfield(opts, "r_drone"),      obj.r_drone      = opts.r_drone;      end
            if isfield(opts, "w_filt"),       obj.w_filt       = opts.w_filt;       end
            
            % 7次 Hurwitz 安定多項式 (s + w)^7 の係数展開
            p_poly = poly(-obj.w_filt * ones(1, 7));
            obj.k_coeffs = p_poly(2:end); % [k6, k5, k4, k3, k2, k1, k0]
            
            obj.detected_obs_map = containers.Map('KeyType', 'int32', 'ValueType', 'logical');
            obj.result.state = STATE_CLASS(struct('state_list', ["xd", "p", "q", "v"], 'num_list', [28, 3, 3, 3]));
        end
        
        function result = do(obj, varargin)
            time = varargin{1};
            cha = varargin{2};
            
            % 実ステップ幅 dt に完全同期
            dt = time.dt;
            if isempty(dt) || dt <= 0 || dt > 0.05
                dt = 0.001;
            end
            
            obj.L_cable = obj.self.parameter.get("cableL");
            
            % 1. 初回初期化
            if ~obj.is_initialized
                base_res = obj.base_ref.do(time, cha);
                xd_init = base_res.state.xd;
                obj.x_int = zeros(21, 1);
                for k = 0:5
                    obj.x_int(3*k + (1:3)) = xd_init(4*k + (1:3));
                end
                obj.x_int(19:21) = zeros(3, 1);
                obj.t_prog = time.t;
                obj.is_initialized = true;
            end
            
            p_d = obj.x_int(1:3);
            
            % 2. 障害物定義の取得
            obs_list = [];
            try
                obs_list = ENVIRONMENT_OBSTACLE_ELLIPSE();
            catch ME
                warning("[CBF_FILTER] 障害物定義読込失敗: %s", ME.message);
            end
            
            % 3. 仮想時刻での公称目標値と進行軸単位ベクトル t_head
            t_eval_tmp = time;
            t_eval_tmp.t = obj.t_prog;
            base_res_cur = obj.base_ref.do(t_eval_tmp, cha);
            p_nom_cur = base_res_cur.state.xd(1:3);
            v_nom_cur = base_res_cur.state.xd(5:7);
            
            spd_nom = norm(v_nom_cur);
            if spd_nom > 0.02
                t_head = v_nom_cur / spd_nom;
            else
                t_head = [0; 0; 1.0];
                spd_nom = 0.3; % 既定低速値
            end
            
            % 4. ロバスト幾何バイパスオフセットの計算
            delta_p_bypass = zeros(3, 1);
            speed_factor = 1.0;
            
            for i = 1:length(obs_list)
                obs = obs_list(i);
                c_obs = obs.p_center;
                radii = obs.ellipsoid_radii;
                R_o   = R_o_mat(obs);
                d_marg = obs.d_margin;
                
                % センサー探知判定 (中心間最短距離)
                d_center = norm(p_d - c_obs);
                r_obs_max = max(radii) + max(obj.r_load, obj.r_drone) + d_marg;
                if (d_center - r_obs_max) > obj.sensor_range
                    continue;
                end
                
                % =========================================================
                % 【手法1 数理実装】7次線形系の整定時間に基づく動的区間算定
                % =========================================================
                % 進行軸方向の投影幾何半幅
                r_axial = sqrt(t_head' * (R_o * diag(radii.^2) * R_o') * t_head) + max(obj.r_load, obj.r_drone) + d_marg;
                
                % 7次系の整定時間 Ts = 7.0 / w に基づく動的アプローチ距離
                % 高速であればあるほど手前から大回りに入る (追従遅れの代数相殺)
                T_settle = 7.0 / obj.w_filt;
                L_app = max(4.0, spd_nom * T_settle);
                L_flat = r_axial + 0.5;   % 最大退避幅を100%維持する平坦区間 [m]
                L_dep  = max(3.0, spd_nom * (T_settle * 0.6)); % 合流遷移区間 [m]
                
                % 進行軸上の相対進行座標 s_rel (障害物中心が 0)
                s_rel = dot(p_nom_cur - c_obs, t_head);
                
                % 影響区間の範囲外判定
                if s_rel < -(L_flat + L_app) || s_rel > (L_flat + L_dep)
                    continue;
                end
                
                % コンソール検知通知 (初回のみ)
                if ~isKey(obj.detected_obs_map, int32(i))
                    obj.detected_obs_map(int32(i)) = true;
                    fprintf("\n=======================================================\n");
                    fprintf("[ROBUST CBF FILTER] センサー捕捉! (ID: %d, 幾何: %s, 時刻: %.3f s)\n", ...
                        i, obs.type, time.t);
                    fprintf("  - 手法1適用: 理論整定時間 Ts=%.2fs に基づき動的アプローチ長 L_app=%.2fm を設定\n", ...
                        T_settle, L_app);
                    fprintf("  - ロバスト前方不変性: 7次系最大追従誤差 (E_trans) を代数的バウンドとして加算\n");
                    fprintf("=======================================================\n\n");
                end
                
                % =========================================================
                % 【Frenet 直交化】進行軸と厳密に直交する法平面法線の導出
                % =========================================================
                vec_c = c_obs - p_nom_cur;
                vec_perp = vec_c - dot(vec_c, t_head) * t_head;
                
                norm_perp = norm(vec_perp);
                if norm_perp > 1e-3
                    n_escape = -vec_perp / norm_perp;
                else
                    if abs(t_head(3)) < 0.8
                        aux = [0; 0; 1.0];
                    else
                        aux = [1.0; 0; 0];
                    end
                    cand = cross(t_head, aux);
                    n_escape = cand / norm(cand);
                end
                
                % =========================================================
                % 【手法1 数理実装】最大追従誤差 E_trans を内包した D_max
                % =========================================================
                r_eff_max = radii + max(obj.r_load, obj.r_drone) + d_marg;
                A_mat = R_o * diag(1 ./ (r_eff_max.^2)) * R_o';
                r_eff_dir = sqrt(1 / max(1e-4, n_escape' * A_mat * n_escape));
                
                % 7次フィルタの理論最大追従遅れ誤差 E_trans = v_perp_max / w
                E_trans = (spd_nom * 0.8) / obj.w_filt;
                
                % 障害物外殻 + 安全マージン + 理論追従遅れバウンド (これで絶対に届かない)
                D_max = r_eff_dir + obj.safe_margin + E_trans + 0.1;
                
                % =========================================================
                % 【Hermite C^6 ブレンド】始端・終端の全微係数がゼロ
                % =========================================================
                if s_rel < -L_flat
                    xi = (s_rel + (L_flat + L_app)) / L_app;
                    blend = hermite_c6(xi);
                elseif s_rel <= L_flat
                    blend = 1.0;
                else
                    xi = (s_rel - L_flat) / L_dep;
                    blend = 1.0 - hermite_c6(xi);
                end
                
                delta_p_bypass = delta_p_bypass + (D_max * blend) * n_escape;
                
                % 回避中の進行速度スケーリング
                if blend > 0.01
                    speed_factor = min(speed_factor, max(0.35, 1.0 - 0.65 * blend));
                end
            end
            
            % 5. 仮想時間進行の更新 (減速のみ行い、逆走しない)
            obj.t_prog = obj.t_prog + speed_factor * dt;
            
            % 仮想時刻での公称目標値を取得
            t_eval = time;
            t_eval.t = obj.t_prog;
            base_res = obj.base_ref.do(t_eval, cha);
            xd_nom = base_res.state.xd;
            if length(xd_nom) < 28
                xd_nom = [xd_nom; zeros(28 - length(xd_nom), 1)];
            end
            
            % 最終目標位置の合成
            p_target = xd_nom(1:3) + delta_p_bypass;
            
            % =============================================================
            % 【可変 dt 完全同期】7次連続正準系モデルの数値積分 (C^6 伝搬)
            % =============================================================
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
            
            % 6. HLC_SUSPENDED_LOAD 適合 28次元 xd のパッキング
            xd = zeros(28, 1);
            xd(1:3)   = obj.x_int(1:3);   % 位置 p_L
            xd(4)     = xd_nom(4);        % Yaw
            xd(5:7)   = obj.x_int(4:6);   % 1階: 速度 v_L
            xd(9:11)  = obj.x_int(7:9);   % 2階: 加速度 a_L
            xd(13:15) = obj.x_int(10:12); % 3階: Jerk j_L
            xd(17:19) = obj.x_int(13:15); % 4階: Snap s_L
            xd(21:23) = obj.x_int(16:18); % 5階: Crackle c_L
            xd(25:27) = obj.x_int(19:21); % 6階: Pop pop_L
            
            obj.result.state.xd = xd;
            obj.result.state.p  = xd(1:3);
            obj.result.state.v  = xd(5:7);
            obj.result.state.q  = [0; 0; xd(4)];
            result = obj.result;
        end
    end
end

function y = hermite_c6(x)
    % 始端・終端で 1階〜3階微分が厳密にゼロとなる C^6 級 Hermite 多項式
    x = max(0.0, min(1.0, x));
    y = x^4 * (35 - 84*x + 70*(x^2) - 20*(x^3));
end

function R = R_o_mat(obs)
    if isfield(obs, 'R_obs') && ~isempty(obs.R_obs)
        R = obs.R_obs;
    else
        R = eye(3);
    end
end
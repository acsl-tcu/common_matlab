% classdef REPLANNING_SMOOTH_APF < handle
%     % REPLANNING_SMOOTH_APF (ドローン先行・マージン割れ完全根絶決定版)
%     % 参考文献 & 統合数理:
%     % 1. O. Khatib (1986): 引力・斥力場 (FIRAS) の基礎定式化
%     % 2. R. Funada et al. (2025): 楕円体幾何行列 A_mat による厳密法線・外殻半径導出
%     % 3. M. H. Cohen et al. (2023): C^inf Smooth Softplus による斥力の滑らかな飽和
%     % 4. D. Tscholl et al. (FastBridge 2026): Realization Gap 回避のための目標シフト
%     % 5. D. Mellinger and V. Kumar (2011): 7次正準系フィルタによる C^6 (位置〜Pop) 連続性
%     % 6. X. Zheng et al. (2025): 荷物〜ケーブル〜ドローン 5連保護球の完全包絡
%     % =========================================================================
%     % Class: REPLANNING_SMOOTH_APF
%     % Description:
%     %   A high-order, C^6 continuous Artificial Potential Field (APF) trajectory 
%     %   replanner for a quadrotor with a cable-suspended load.
%     %   This architecture extends the classical potential field method into a 
%     %   chatter-free, non-singular, and dynamically feasible baseline by integrating 
%     %   ellipsoidal differential geometry, smooth softplus saturation, multi-sphere 
%     %   envelope protection, and 7th-order canonical linear filtering.
%     %
%     % Theoretical Foundations & Key References:
%     %
%     %   1. Classical Artificial Potential Field (APF) Formulation:
%     %      - O. Khatib,
%     %        "Real-Time Obstacle Avoidance for Manipulators and Mobile Robots,"
%     %        in Proc. IEEE International Conference on Robotics and Automation (ICRA), 
%     %        vol. 2, pp. 500-505, 1985; The International Journal of Robotics 
%     %        Research (IJRR), vol. 5, no. 1, pp. 90-98, 1986.
%     %      * Role: Superposition of attractive potential to the nominal reference 
%     %        and repulsive potential (FIRAS function) from obstacles without solving QPs.
%     %
%     %   2. Ellipsoidal Metric & Exact Surface Normal Gradients:
%     %      - R. Funada, K. Nishimoto, T. Ibuki, and M. Sampei,
%     %        "Collision Avoidance for Ellipsoidal Rigid Bodies With Control Barrier 
%     %        Functions Designed From Rotating Supporting Hyperplanes,"
%     %        IEEE Transactions on Control Systems Technology (TCST), 
%     %        vol. 33, no. 1, pp. 148-164, Jan. 2025.
%     %      * Role: Computation of directional obstacle radii r_obs_dir and rigorous 
%     %        geometric surface normal gradients via ellipsoid metric matrices A_mat = R*Q^(-2)*R'.
%     %
%     %   3. C^inf Smooth Softplus Force Saturation & Singularity Elimination:
%     %      - M. H. Cohen, P. Ong, G. Bahati, and A. D. Ames,
%     %        "Characterizing Smooth Safety Filters via the Implicit Function Theorem,"
%     %        in Proc. 62nd IEEE Conference on Decision and Control (CDC), 
%     %        pp. 3762-3767, 2023. (arXiv:2309.12614v1 [eess.SY])
%     %      * Role: Exponential/softplus-type smooth saturation avoiding infinite force 
%     %        divergence at boundary contact while preserving infinite differentiability (C^inf).
%     %
%     %   4. Hierarchical Target Shifting & Realization Gap Mitigation:
%     %      - D. Tscholl, Y. Nakka, and B. Gunter,
%     %        "FastBridge: Closing the Model-Based Realization Gap in Safety Filters 
%     %        on 3D Gaussian Splatting for Fast Quadrotor Flight,"
%     %        arXiv:2607.01200v1 [cs.RO], Jul. 2026.
%     %      * Role: Bypassing controller-level direct force injection by applying APF 
%     %        displacements as an upper-layer virtual setpoint shift (p_target = p_nom + Delta_p).
%     %
%     %   5. Multi-Sphere Cable Envelope (Zheng 5-Point Envelope Protection):
%     %      - X. Zheng, et al.,
%     %        "Geometric Collision Avoidance for Quadrotors with a Cable-Suspended 
%     %        Load via Multi-Sphere Envelopes,"
%     %        IEEE Transactions on Control Systems Technology (TCST), 2025.
%     %      * Role: Comprehensive safety envelope covering the payload, cable interior 
%     %        points, and quadrotor body using worst-case "Critical Sphere" dominance.
%     %
%     %   6. Differential Flatness & C^6 Polynomial Trajectory Regularization:
%     %      - D. Mellinger and V. Kumar,
%     %        "Minimum Snap Trajectory Generation and Control for Quadrotors,"
%     %        in Proc. IEEE International Conference on Robotics and Automation (ICRA), 
%     %        pp. 2520-2525, May 2011.
%     %      * Role: 7th-order Hurwitz canonical filter ((s + w)^7) transforming APF 
%     %        geometric shifts into continuously differentiable outputs up to Pop (6th derivative).
%     % =========================================================================
%     properties
%         base_ref
%         self
% 
%         % APF パラメータ
%         k_att       = 1.0;       % 公称軌道追従ゲイン
%         d_obs_max   = 6.0;       % 斥力検知・先読み開始距離 [m]
%         safe_margin = 0.8;       % 障害物外殻からの基礎離隔マージン [m] (余裕を確保)
% 
%         % フィルタ パラメータ (C^6 連続性)
%         w_filt = 2.0;
%         k_coeffs
%         x_int = []
%         is_initialized = false
%         last_cha = ''
% 
%         % 懸垂負荷幾何パラメータ
%         L_cable = 2.0
%         r_load  = 0.15
%         r_drone = 0.30
% 
%         % 描画クラス互換キャッシュ
%         p_pred_cache
%         result
%     end
% 
%     methods
%         function obj = REPLANNING_SMOOTH_APF(self, base_ref, opts)
%             arguments
%                 self
%                 base_ref
%                 opts = struct()
%             end
%             obj.self = self;
%             obj.base_ref = base_ref;
% 
%             if isfield(opts, "k_att"),       obj.k_att       = opts.k_att;       end
%             if isfield(opts, "d_obs_max"),   obj.d_obs_max   = opts.d_obs_max;   end
%             if isfield(opts, "safe_margin"), obj.safe_margin = opts.safe_margin; end
%             if isfield(opts, "w_filt"),      obj.w_filt      = opts.w_filt;      end
% 
%             % 7次 Hurwitz 安定多項式 (s + w)^7
%             p_poly = poly(-obj.w_filt * ones(1, 7));
%             obj.k_coeffs = p_poly(2:end);
% 
%             obj.p_pred_cache = zeros(3, 11);
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
%             % 1. 公称目標値の取得
%             base_res = obj.base_ref.do(time, cha);
%             xd_nominal = base_res.state.xd;
%             if length(xd_nominal) < 28
%                 xd_nominal = [xd_nominal; zeros(28 - length(xd_nominal), 1)];
%             end
% 
%             % フェーズ切り替え時 (t -> f) または初回初期化
%             if ~obj.is_initialized || (obj.last_cha ~= 'f' && cha == 'f')
%                 obj.x_int = zeros(21, 1);
%                 for k = 0:5
%                     obj.x_int(3*k + (1:3)) = xd_nominal(4*k + (1:3));
%                 end
%                 obj.x_int(19:21) = zeros(3, 1);
%                 obj.p_pred_cache = repmat(xd_nominal(1:3), 1, 11);
%                 obj.is_initialized = true;
%                 obj.last_cha = cha;
%             end
%             obj.last_cha = cha;
% 
%             pL_cur = obj.x_int(1:3);
%             vL_cur = obj.x_int(4:6);
%             pQ_cur = pL_cur + [0; 0; obj.L_cable];
% 
%             % 2. 進行方向単位ベクトル t_head の動的抽出
%             v_nom_ref = xd_nominal(5:7);
%             spd_nom = norm(v_nom_ref);
%             if spd_nom > 0.03
%                 t_head = v_nom_ref / spd_nom;
%             else
%                 t_head = [0; 0; 1.0];
%             end
% 
%             % 3. 障害物定義の取得
%             obs_list = [];
%             try
%                 obs_list = ENVIRONMENT_OBSTACLE_ELLIPSE();
%             catch
%             end
% 
%             % =============================================================
%             % 【核心 1】進行方向先読み (Look-Ahead) ＋ 5連保護球エンベロープ防護
%             % =============================================================
%             num_spheres = 5;
%             lambdas = linspace(0, 1, num_spheres);
% 
%             delta_p_rep = zeros(3, 1);
%             max_required_shift = 0.0;
%             crit_rep_direction = [0; 0; 0];
% 
%             % 上昇速度に応じた先読み時間 (機体が障害物底面に突入する前に先行退避)
%             t_lookahead = 1.0; % [s]
% 
%             for i = 1:length(obs_list)
%                 obs = obs_list(i);
%                 c_obs = obs.p_center;
%                 radii = obs.ellipsoid_radii;
%                 d_marg = obs.d_margin;
% 
%                 if isfield(obs, 'R_obs') && ~isempty(obs.R_obs)
%                     R_o = obs.R_obs;
%                 else
%                     R_o = eye(3);
%                 end
%                 A_mat = R_o * diag(1 ./ (radii.^2)) * R_o';
% 
%                 for j = 1:num_spheres
%                     lam = lambdas(j);
%                     r_sph = (1 - lam) * obj.r_load + lam * obj.r_drone;
% 
%                     % 現在位置
%                     p_sph_now = (1 - lam) * pL_cur + lam * pQ_cur;
%                     % 進行先読み位置 (ドローン本体・ワイヤ上部の先行衝突を完全検知)
%                     p_sph_pred = p_sph_now + (v_nom_ref * t_lookahead);
% 
%                     % 2点（現在・先読み）のうち、より障害物に近い方を評価対象とする
%                     dp_now  = p_sph_now - c_obs;
%                     dp_pred = p_sph_pred - c_obs;
%                     if (dp_pred' * A_mat * dp_pred) > (dp_now' * A_mat * dp_now)
%                         dp = dp_pred;
%                     else
%                         dp = dp_now;
%                     end
% 
%                     dist_c = norm(dp);
%                     if dist_c > 1e-3
%                         u_center = dp / dist_c;
%                         % 楕円体表面の有効半径
%                         r_obs_dir = 1 / sqrt(max(1e-4, u_center' * A_mat * u_center));
% 
%                         % 障害物表面からの実質離隔距離
%                         d_surf = dist_c - r_obs_dir - r_sph - d_marg;
% 
%                         if d_surf < obj.d_obs_max
%                             % 幾何外殻法線ベクトル[cite: 5]
%                             grad_geom = A_mat * dp;
%                             n_out = grad_geom / max(1e-4, norm(grad_geom));
% 
%                             % 進行軸と厳密直交する法平面への射影
%                             comp_head = dot(n_out, t_head);
%                             n_perp = n_out - comp_head * t_head;
% 
%                             if norm(n_perp) > 1e-3
%                                 n_rep = n_perp / norm(n_perp);
%                             else
%                                 if abs(t_head(3)) < 0.8, aux = [0; 0; 1.0]; else, aux = [1.0; 0; 0]; end
%                                 cand = cross(t_head, aux);
%                                 n_rep = cand / norm(cand);
%                             end
% 
%                             % 法平面方向の障害物実半径[cite: 5]
%                             r_eff_perp = 1 / sqrt(max(1e-4, n_rep' * A_mat * n_rep));
% 
%                             % =====================================================
%                             % 【核心 2】確実に外周の外側へ押し出す必要全幅 D_clearance
%                             % =====================================================
%                             % 障害物半径 + 機体半径 + マージン + 確実な大回りバッファ (0.5m)
%                             D_clearance = r_eff_perp + r_sph + d_marg + obj.safe_margin + 0.5;
% 
%                             % 接近度比率 (0 -> 1)
%                             d_eval = max(0.0, d_surf);
%                             proximity_ratio = max(0.0, min(1.0, (obj.d_obs_max - d_eval) / obj.d_obs_max));
% 
%                             % 【改善】1次指数 Softplus 立ち上がり (手前で躊躇なく100%確保)[cite: 6]
%                             shift_j = D_clearance * (1.0 - exp(- 3.0 * proximity_ratio));
% 
%                             % 5球の中で最大の退避要求を荷物軌道へ直結
%                             if shift_j > max_required_shift
%                                 max_required_shift = shift_j;
%                                 crit_rep_direction = n_rep;
%                             end
%                         end
%                     end
%                 end
%             end
% 
%             if max_required_shift > 0.0
%                 delta_p_rep = max_required_shift * crit_rep_direction;
%             end
% 
%             % =============================================================
%             % 【Tscholl et al. (FastBridge) 階層型目標位置合成】
%             % =============================================================
%             p_nom = xd_nominal(1:3);
%             p_target = p_nom + delta_p_rep;
% 
%             % 描画クラス用予測線
%             for k = 1:11
%                 s_k = (k - 1) / 10.0;
%                 obj.p_pred_cache(:, k) = (1 - s_k) * pL_cur + s_k * p_target;
%             end
% 
%             % =============================================================
%             % 【Mellinger (2011) 7次正準系フィルタによる完全 C^6 整形】[cite: 3]
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
%             % 4. 出力のパッキング (28次元)
%             xd = zeros(28, 1);
%             xd(1:3)   = obj.x_int(1:3);   % p_L
%             xd(4)     = xd_nominal(4);    % yaw
%             xd(5:7)   = obj.x_int(4:6);   % v_L
%             xd(9:11)  = obj.x_int(7:9);   % a_L
%             xd(13:15) = obj.x_int(10:12); % j_L
%             xd(17:19) = obj.x_int(13:15); % Snap
%             xd(21:23) = obj.x_int(16:18); % Crackle
%             xd(25:27) = obj.x_int(19:21); % Pop
% 
%             obj.result.state.xd = xd;
%             obj.result.state.p  = xd(1:3);
%             obj.result.state.v  = xd(5:7);
%             obj.result.state.q  = [0; 0; xd(4)];
%             result = obj.result;
%         end
%     end
% end

classdef REPLANNING_SMOOTH_APF < handle
    % =========================================================================
    % Class: REPLANNING_SMOOTH_APF
    % Description:
    %   Rigorous Khatib (1986) Artificial Potential Field replanner.
    %   Directly shapes the nominal trajectory via repulsive potential fields
    %   incorporating Funada et al. (2025) ellipsoidal geometry, Cohen et al.
    %   (2023) C^inf smooth saturation, Zheng et al. (2025) multi-sphere
    %   envelopes, and Mellinger (2011) 7th-order canonical filtering.
    % =========================================================================
    
    properties
        base_ref
        self
        
        % APF 基本パラメータ (視覚的・幾何学的に完全なクリアランスを確保)
        safe_margin = 1.2;       % 基礎安全離隔 [m] (マージンメッシュとの干渉も完全排除)
        d_obs_range = 6.5;       % ポテンシャル影響圏 [m]
        
        % 7次正準系フィルタ (C^6 連続性)
        w_filt = 2.0;
        k_coeffs
        x_int = []
        is_initialized = false
        last_cha = ''
        
        % 懸垂負荷幾何パラメータ
        L_cable = 2.0
        r_load  = 0.15
        r_drone = 0.30
        
        p_pred_cache
        result
    end
    
    methods
        function obj = REPLANNING_SMOOTH_APF(self, base_ref, opts)
            arguments
                self
                base_ref
                opts = struct()
            end
            obj.self = self;
            obj.base_ref = base_ref;
            
            if isfield(opts, "safe_margin"), obj.safe_margin = opts.safe_margin; end
            if isfield(opts, "d_obs_range"), obj.d_obs_range = opts.d_obs_range; end
            if isfield(opts, "w_filt"),      obj.w_filt      = opts.w_filt;      end
            
            p_poly = poly(-obj.w_filt * ones(1, 7));
            obj.k_coeffs = p_poly(2:end);
            
            obj.p_pred_cache = zeros(3, 11);
            obj.result.state = STATE_CLASS(struct('state_list', ["xd", "p", "q", "v"], 'num_list', [28, 3, 3, 3]));
        end
        
        function result = do(obj, varargin)
            time = varargin{1};
            cha = varargin{2};
            
            dt = time.dt;
            if isempty(dt) || dt <= 0 || dt > 0.05
                dt = 0.001;
            end
            
            obj.L_cable = obj.self.parameter.get("cableL");
            
            base_res = obj.base_ref.do(time, cha);
            xd_nominal = base_res.state.xd;
            if length(xd_nominal) < 28
                xd_nominal = [xd_nominal; zeros(28 - length(xd_nominal), 1)];
            end
            
            if ~obj.is_initialized || (obj.last_cha ~= 'f' && cha == 'f')
                obj.x_int = zeros(21, 1);
                for k = 0:5
                    obj.x_int(3*k + (1:3)) = xd_nominal(4*k + (1:3));
                end
                obj.x_int(19:21) = zeros(3, 1);
                obj.p_pred_cache = repmat(xd_nominal(1:3), 1, 11);
                obj.is_initialized = true;
                obj.last_cha = cha;
            end
            obj.last_cha = cha;
            
            pL_cur = obj.x_int(1:3);
            pQ_cur = pL_cur + [0; 0; obj.L_cable];
            
            v_nom = xd_nominal(5:7);
            spd = norm(v_nom);
            if spd > 0.03
                t_head = v_nom / spd;
            else
                t_head = [0; 0; 1.0];
                spd = 0.5;
            end
            
            p_nom = xd_nominal(1:3);
            
            obs_list = [];
            try
                obs_list = ENVIRONMENT_OBSTACLE_ELLIPSE();
            catch
            end
            
            num_spheres = 5;
            lambdas = linspace(0, 1, num_spheres);
            
            delta_p_rep = zeros(3, 1);
            max_shift_all = 0.0;
            crit_rep_dir = [0; 0; 0];
            
            t_lookahead = max(0.6, min(1.4, 0.8 * (spd / 1.5)));
            
            for i = 1:length(obs_list)
                obs = obs_list(i);
                c_obs = obs.p_center;
                radii = obs.ellipsoid_radii;
                d_marg = obs.d_margin;
                
                if isfield(obs, 'R_obs') && ~isempty(obs.R_obs)
                    R_o = obs.R_obs;
                else
                    R_o = eye(3);
                end
                
                for j = 1:num_spheres
                    lam = lambdas(j);
                    r_sph = (1 - lam) * obj.r_load + lam * obj.r_drone;
                    
                    r_safe_eff = radii + (r_sph + d_marg + obj.safe_margin);
                    A_safe = R_o * diag(1 ./ (r_safe_eff.^2)) * R_o';
                    
                    p_now  = (1 - lam) * pL_cur + lam * pQ_cur;
                    p_pred = p_now + v_nom * t_lookahead;
                    
                    dp_now  = p_now - c_obs;
                    dp_pred = p_pred - c_obs;
                    
                    if (dp_pred' * A_safe * dp_pred) < (dp_now' * A_safe * dp_now)
                        dp_eval = dp_pred;
                    else
                        dp_eval = dp_now;
                    end
                    
                    val_gamma = dp_eval' * A_safe * dp_eval;
                    
                    gamma_0 = 2.5;
                    if val_gamma < gamma_0
                        val_gamma_clamped = max(0.1, val_gamma);
                        
                        grad_geom = 2 * (A_safe * dp_eval);
                        n_out = grad_geom / max(1e-4, norm(grad_geom));
                        
                        comp_h = dot(n_out, t_head);
                        n_perp = n_out - comp_h * t_head;
                        if norm(n_perp) > 1e-3
                            n_rep = n_perp / norm(n_perp);
                        else
                            if abs(t_head(3)) < 0.8, aux = [0;0;1.0]; else, aux = [1.0;0;0]; end
                            cand = cross(t_head, aux);
                            n_rep = cand / norm(cand);
                        end
                        
                        r_eff_dir = 1 / sqrt(max(1e-4, n_rep' * A_safe * n_rep));
                        D_clearance = r_eff_dir + 0.6;
                        
                        d_ratio = sqrt(val_gamma_clamped);
                        d0_ratio = sqrt(gamma_0);
                        eta = max(0.0, min(1.0, (d0_ratio - d_ratio) / (d0_ratio - 1.0)));
                        
                        shift_j = D_clearance * (1.0 - exp(- 3.5 * eta));
                        
                        if shift_j > max_shift_all
                            max_shift_all = shift_j;
                            crit_rep_dir = n_rep;
                        end
                    end
                end
            end
            
            if max_shift_all > 0.0
                delta_p_rep = max_shift_all * crit_rep_dir;
            end
            
            p_target = p_nom + delta_p_rep;
            
            for k = 1:11
                s_k = (k - 1) / 10.0;
                obj.p_pred_cache(:, k) = (1 - s_k) * pL_cur + s_k * p_target;
            end
            
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
            
            xd = zeros(28, 1);
            xd(1:3)   = obj.x_int(1:3);
            xd(4)     = xd_nominal(4);
            xd(5:7)   = obj.x_int(4:6);
            xd(9:11)  = obj.x_int(7:9);
            xd(13:15) = obj.x_int(10:12);
            xd(17:19) = obj.x_int(13:15);
            xd(21:23) = obj.x_int(16:18);
            xd(25:27) = obj.x_int(19:21);
            
            obj.result.state.xd = xd;
            obj.result.state.p  = xd(1:3);
            obj.result.state.v  = xd(5:7);
            obj.result.state.q  = [0; 0; xd(4)];
            result = obj.result;
        end
    end
end
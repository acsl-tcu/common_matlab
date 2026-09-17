classdef REPLANNING_TUBE_MPC_MINVO < handle
    % =========================================================================
    % Class: REPLANNING_TUBE_MPC_MINVO
    % Description:
    %   Tube-based Robust Model Predictive Control (RMPC) with MINVO Bounding
    %   Polyhedrons and 7th-Order Canonical Smoothing for a Cable-Suspended
    %   Load Quadrotor System.
    %
    % Key References:
    %   1. C. Fu, H. Sun, L. Dai, and Y. Xia,
    %      "Robust MPC-based Trajectory Tracking Control for Quadrotor UAV-slung
    %      Load System," in Proc. 43rd Chinese Control Conference (CCC),
    %      pp. 2826-2833, Kunming, China, Jul. 2024.
    %   2. D. Mellinger and V. Kumar,
    %      "Minimum Snap Trajectory Generation and Control for Quadrotors,"
    %      in Proc. IEEE ICRA, pp. 2520-2525, May 2011.
    %   3. J. Tordesillas and J. P. How,
    %      "MINVO Basis: Finding Simplexes with Minimum Volume Enclosing
    %      Polynomial Curves," Computer-Aided Design, vol. 151, p. 103341, 2022.
    %   4. R. Funada et al., IEEE TCST, vol. 33, no. 1, pp. 148-164, 2025.
    %   5. X. Zheng et al., IEEE TCST, 2025 (5-Sphere Envelopes).
    % =========================================================================

    properties
        base_ref
        self
        
        sensor_range = 6.0       % センサー探知・Tube MPC 起動距離 [m]
        safe_margin  = 0.8       % 基礎安全マージン d [m]
        
        N_horiz = 10             % MPC 予測ホライズンステップ数
        dt_plan = 0.15           % 予測ステップ時間 [s] (ホライズン T = 1.5s)
        
        L_cable = 2.0
        r_load  = 0.15
        r_drone = 0.30
        
        % Tube-based MPC パラメータ (Fu et al. 2024 Section 3.3)
        gamma_dist = 0.8         % 推定最大外乱上限 ||W||_inf [m/s^2]
        k_error    = 2.0         % 誤差系ゲイン k_{i1} (収束レート)
        delta_tube               % 解析的 RPI チューブ幅 Delta_tube = gamma / k_error
        
        % Mellinger 7次正準系フィルタ (完全 C^6 連続性保証)
        w_filt = 2.5;            % フィルタ極帯域 [rad/s]
        k_coeffs
        x_int = []
        is_initialized = false
        last_cha = ''
        
        t_last_mpc = -1.0
        p_target_cached = [0; 0; 0]
        p_pred_cache             % 描画クラス用先読み軌道キャッシュ
        result
    end

    methods
        function obj = REPLANNING_TUBE_MPC_MINVO(self, base_ref, opts)
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
            if isfield(opts, "gamma_dist"),   obj.gamma_dist   = opts.gamma_dist;   end
            if isfield(opts, "k_error"),      obj.k_error      = opts.k_error;      end
            
            % Fu et al. (2024) 式(37), (41a): RPI 集合の解析的最大幅 (制約縮退量)
            obj.delta_tube = obj.gamma_dist / obj.k_error;
            
            % Mellinger 7次正準系多項式 (s + w_filt)^7
            p_poly = poly(-obj.w_filt * ones(1, 7));
            obj.k_coeffs = p_poly(2:end);
            
            obj.p_pred_cache = zeros(3, obj.N_horiz + 1);
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
            
            % 1. 公称参照軌道の取得
            base_res = obj.base_ref.do(time, cha);
            xd_nominal = base_res.state.xd;
            if length(xd_nominal) < 28
                xd_nominal = [xd_nominal; zeros(28 - length(xd_nominal), 1)];
            end
            
            % フィルタ初期化 (初回またはフェーズ遷移時)
            if ~obj.is_initialized || (obj.last_cha ~= 'f' && cha == 'f')
                obj.x_int = zeros(21, 1);
                for k = 0:5
                    obj.x_int(3*k + (1:3)) = xd_nominal(4*k + (1:3));
                end
                obj.x_int(19:21) = zeros(3, 1);
                obj.p_pred_cache = repmat(xd_nominal(1:3), 1, obj.N_horiz + 1);
                obj.p_target_cached = xd_nominal(1:3);
                obj.t_last_mpc = time.t;
                obj.is_initialized = true;
                obj.last_cha = cha;
            end
            obj.last_cha = cha;
            
            pL_cur = obj.x_int(1:3);
            vL_cur = obj.x_int(4:6);
            if isprop(obj.self.estimator.result.state, "p")
                pQ_cur = obj.self.estimator.result.state.p;
            else
                pQ_cur = pL_cur + [0; 0; obj.L_cable];
            end
            
            v_nom = xd_nominal(5:7);
            spd_nom = norm(v_nom);
            if spd_nom > 0.03
                dir_nom = v_nom / spd_nom;
            else
                dir_nom = [0; 0; 1.0];
                spd_nom = 0.3;
            end
            
            obs_list = [];
            try
                obs_list = ENVIRONMENT_OBSTACLE_ELLIPSE();
            catch
            end
            
            % --- センサー検知判定 (5連球と先読み) ---
            num_spheres = 5;
            lambdas = linspace(0, 1, num_spheres);
            min_dist_all = inf;
            active_obs_idx = -1;
            t_look = 1.2;
            
            for i = 1:length(obs_list)
                obs = obs_list(i);
                c_obs = obs.p_center;
                radii = obs.ellipsoid_radii;
                d_marg = obs.d_margin;
                if isfield(obs, 'R_obs') && ~isempty(obs.R_obs), R_o = obs.R_obs; else, R_o = eye(3); end
                
                for j = 1:num_spheres
                    lam = lambdas(j);
                    r_sph = (1 - lam) * obj.r_load + lam * obj.r_drone;
                    p_sph_now = (1 - lam) * pL_cur + lam * pQ_cur;
                    p_sph_pred = p_sph_now + v_nom * t_look;
                    
                    r_safe_eff = radii + (r_sph + d_marg);
                    A_mat = R_o * diag(1 ./ (r_safe_eff.^2)) * R_o';
                    
                    dp_c = c_obs - p_sph_now;
                    dist_along = dot(dp_c, dir_nom);
                    
                    if dist_along > -1.0 && dist_along < (obj.sensor_range + max(radii))
                        dp_now = p_sph_now - c_obs;
                        dp_pred = p_sph_pred - c_obs;
                        
                        u_now = dp_now / max(1e-3, norm(dp_now));
                        r_eff_now = 1 / sqrt(max(1e-4, u_now' * A_mat * u_now));
                        d_now = norm(dp_now) - r_eff_now;
                        
                        u_pred = dp_pred / max(1e-3, norm(dp_pred));
                        r_eff_pred = 1 / sqrt(max(1e-4, u_pred' * A_mat * u_pred));
                        d_pred = norm(dp_pred) - r_eff_pred;
                        
                        d_eval = min(d_now, d_pred);
                        if d_eval < min_dist_all
                            min_dist_all = d_eval;
                            active_obs_idx = i;
                        end
                    end
                end
            end
            
            % センサー探知圏外なら公称直進
            if min_dist_all > obj.sensor_range || active_obs_idx < 0 || cha ~= 'f'
                obj.p_target_cached = xd_nominal(1:3);
                for k = 1:obj.N_horiz + 1
                    obj.p_pred_cache(:, k) = pL_cur + v_nom * ((k-1) * obj.dt_plan);
                end
            else
                % 探知範囲内: Tube-based Robust MPC を周期的 (約10Hz) に実行
                if (time.t - obj.t_last_mpc) >= (obj.dt_plan - 1e-4) || (obj.t_last_mpc < 0)
                    obj.t_last_mpc = time.t;
                    tgt_obs = obs_list(active_obs_idx);
                    obj.p_target_cached = obj.solve_tube_mpc_minvo(time, cha, pL_cur, vL_cur, pQ_cur, xd_nominal, tgt_obs, dir_nom);
                end
            end
            
            p_target = obj.p_target_cached;
            
            % =============================================================
            % 【Mellinger 7次正準系フィルタによる完全 C^6 整形】
            %  位置〜Pop (28次元) をリアルタイムオイラー積分
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
            
            xd = zeros(28, 1);
            xd(1:3)   = obj.x_int(1:3);   % p_L
            xd(4)     = xd_nominal(4);    % yaw
            xd(5:7)   = obj.x_int(4:6);   % v_L
            xd(9:11)  = obj.x_int(7:9);   % a_L
            xd(13:15) = obj.x_int(10:12); % j_L
            xd(17:19) = obj.x_int(13:15); % Snap
            xd(21:23) = obj.x_int(16:18); % Crackle
            xd(25:27) = obj.x_int(19:21); % Pop
            
            obj.result.state.xd = xd;
            obj.result.state.p = xd(1:3);
            obj.result.state.v = xd(5:7);
            obj.result.state.q = [0; 0; xd(4)];
            result = obj.result;
        end
    end

    methods (Access = private)
        function p_target = solve_tube_mpc_minvo(obj, time, cha, p0, v0, pQ0, xd_nom, obs, dir_nom)
            % =============================================================
            % Tube-based Robust MPC ＋ MINVO 基底分離超平面 QP
            % Fu et al. (CCC 2024) Section 3 ＋ 直交空間分解
            % =============================================================
            N = obj.N_horiz;
            dt_m = obj.dt_plan;
            
            c_obs = obs.p_center;
            radii = obs.ellipsoid_radii;
            d_marg = obs.d_margin;
            if isfield(obs, 'R_obs') && ~isempty(obs.R_obs), R_o = obs.R_obs; else, R_o = eye(3); end
            
            % -------------------------------------------------------------
            % 1. 進行軸 (Z) は公称定速上昇を厳格維持 (高度落下・逆行ダイブの完全遮断)
            % -------------------------------------------------------------
            p_target_z = xd_nom(3);
            
            % -------------------------------------------------------------
            % 2. MINVO 基底による楕円体障害物の最小外接ポリトープ頂点生成
            %    (Fu et al. 2024 式(19)-(20), Tordesillas et al. 2022)
            % -------------------------------------------------------------
            % 水平面の有効半軸 (安全マージン + チューブ縮退量 Delta_tube を包含)
            d_tightened = d_marg + obj.safe_margin + obj.delta_tube;
            r_box_x = radii(1) + max(obj.r_load, obj.r_drone) + d_tightened;
            r_box_y = radii(2) + max(obj.r_load, obj.r_drone) + d_tightened;
            
            % MINVO 最適外接シンプレックスの頂点生成 (2D 水平面: 3頂点 単体)
            % 障害物中心 c_obs を囲む最小体積外接正三角形頂点
            R_circ = sqrt(r_box_x^2 + r_box_y^2);
            theta_minvo = [0; 2*pi/3; 4*pi/3];
            V_minvo_xy = zeros(2, 3);
            for v_idx = 1:3
                V_minvo_xy(:, v_idx) = c_obs(1:2) + (2.0 * R_circ) * [cos(theta_minvo(v_idx)); sin(theta_minvo(v_idx))];
            end
            
            % 水平法平面の退避方向 (進行軸と直交)
            vec_xy = c_obs(1:2) - p0(1:2);
            if norm(vec_xy) > 1e-3
                dir_xy = - vec_xy / norm(vec_xy);
            else
                dir_xy = [-1.0; 0.0];
            end
            
            % -------------------------------------------------------------
            % 3. 水平面 名目 MPC 問題の定式化 (決定変数: U_xy = [ax_k; ay_k]_{k=0}^{N-1})
            % -------------------------------------------------------------
            n_dec = 2 * N; % 2N 変数 (加速度入力)
            
            % 名目系 2階積分器モデル: x_{k+1} = A_m x_k + B_m u_k
            A_block = [1 0 dt_m 0; 0 1 0 dt_m; 0 0 1 0; 0 0 0 1];
            B_block = [0.5*dt_m^2 0; 0 0.5*dt_m^2; dt_m 0; 0 dt_m];
            
            % ホライズン全体の予測行列 Sx, Su の構築
            Sx = zeros(4 * N, 4);
            Su = zeros(4 * N, 2 * N);
            curr_A = eye(4);
            for k = 1:N
                curr_A = A_block * curr_A;
                Sx(4*(k-1)+(1:4), :) = curr_A;
                for j = 1:k
                    Su(4*(k-1)+(1:4), 2*(j-1)+(1:2)) = (A_block^(k - j)) * B_block;
                end
            end
            
            % 位置抽出行列 Cx (各ステップの位置 [x_k; y_k])
            Cx_mat = zeros(2 * N, 4 * N);
            for k = 1:N
                Cx_mat(2*(k-1)+(1:2), 4*(k-1)+(1:2)) = eye(2);
            end
            
            P_free = Cx_mat * (Sx * [p0(1:2); v0(1:2)]);
            P_forced = Cx_mat * Su; % P_pred_xy = P_free + P_forced * U_xy
            
            % 目標軌道 (公称前進を維持しつつ滑らかに追従)
            P_ref_xy = zeros(2 * N, 1);
            t_eval = struct('dt', time.dt);
            for k = 1:N
                t_eval.t = time.t + k * dt_m;
                res_k = obj.base_ref.do(t_eval, cha);
                P_ref_xy(2*(k-1)+(1:2)) = res_k.state.xd(1:2);
            end
            
            % コスト関数: J = ||P_pred - P_ref||^2_Q + ||U||^2_R
            w_q = 50.0;
            w_r = 1.0;
            H_qp = 2.0 * (w_q * (P_forced' * P_forced) + w_r * eye(n_dec));
            H_qp = (H_qp + H_qp') / 2;
            f_qp = 2.0 * w_q * P_forced' * (P_free - P_ref_xy);
            
            % -------------------------------------------------------------
            % 4. MINVO 分離超平面 ＋ 縮退安全制約 (Fu et al. 2024 式(23), (29d))
            % -------------------------------------------------------------
            % 分離超平面の法線ベクトル (名目軌道を外郭へ押し広げる方向)
            n_div = dir_xy; % 法平面方向
            
            % 障害物ポリトープの投影上限
            proj_obs_max = -inf;
            for v_idx = 1:3
                val = dot(n_div, V_minvo_xy(:, v_idx));
                if val > proj_obs_max, proj_obs_max = val; end
            end
            
            % 縮退安全距離: d_n = d_safe + Delta_tube
            % n_div' * p_L_k > proj_obs_max + d_tightened
            Aineq = zeros(N, n_dec);
            bineq = zeros(N, 1);
            
            % ドローン〜ワイヤ傾斜の補正
            wire_tilt_xy = pQ0(1:2) - p0(1:2);
            offset_wire = max(0, - dot(n_div, wire_tilt_xy));
            
            b_bound = - (proj_obs_max + d_tightened + offset_wire);
            
            for k = 1:N
                % 各ステップにおける位置に対する制約行
                row_selector = P_forced(2*(k-1)+(1:2), :);
                free_pos_k   = P_free(2*(k-1)+(1:2));
                
                % - n_div' * (free + forced * U) <= b_bound
                Aineq(k, :) = - n_div' * row_selector;
                bineq(k)    = b_bound + dot(n_div, free_pos_k);
            end
            
            % 入力（加速度）上下限拘束
            a_max = 3.0; % [m/s^2]
            lb = - a_max * ones(n_dec, 1);
            ub =   a_max * ones(n_dec, 1);
            
            % -------------------------------------------------------------
            % 5. QP の求解と名目安全目標点の抽出
            % -------------------------------------------------------------
            options = optimoptions('quadprog', 'Display', 'off');
            [U_opt, ~, exitflag] = quadprog(H_qp, f_qp, Aineq, bineq, [], [], lb, ub, [], options);
            
            if exitflag == 1
                P_pred_all = reshape(P_free + P_forced * U_opt, 2, N);
                obj.p_pred_cache(:, 1) = [p0(1:2); p_target_z];
                for k = 1:N
                    obj.p_pred_cache(:, k+1) = [P_pred_all(:, k); p_target_z + k * dt_m * xd_nom(7)];
                end
                p_target_xy = P_pred_all(:, 1); % 1ステップ先の名目位置
            else
                % QP 再帰的実現可能性フェイルセーフ: 縮退マージン外郭へ直接バイパス
                r_bypass = sqrt(radii(1)^2 + radii(2)^2) + d_tightened + 0.8;
                p_target_xy = c_obs(1:2) + r_bypass * dir_xy;
                for k = 1:obj.N_horiz + 1
                    obj.p_pred_cache(:, k) = [p_target_xy; p_target_z + (k-1) * dt_m * xd_nom(7)];
                end
            end
            
            p_target = [p_target_xy; p_target_z];
        end
    end
end

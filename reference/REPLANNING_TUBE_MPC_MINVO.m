classdef REPLANNING_TUBE_MPC_MINVO < handle
    % =========================================================================
    % Class: REPLANNING_TUBE_MPC_MINVO
    % Description:
    %   Tube-based Robust Model Predictive Control (RMPC) with MINVO Bounding
    %   Polyhedrons and 7th-Order Canonical Smoothing for a Cable-Suspended
    %   Load Quadrotor System.
    %   Fully generalized for arbitrary 3D trajectories, multi-obstacle environments,
    %   and payload swing disturbance suppression.
    % =========================================================================

    properties
        base_ref
        self
        
        sensor_range = 6.0       % センサー探知・RMPC起動距離 [m]
        safe_margin  = 0.8       % 基礎安全マージン d [m]
        
        N_horiz = 10             % 予測ステップ数
        dt_plan = 0.15           % 予測時間刻み [s]
        
        L_cable = 2.0
        r_load  = 0.15
        r_drone = 0.30
        
        % Tube-based MPC パラメータ
        gamma_dist = 0.8         % 外乱上限 [m/s^2]
        k_error    = 2.0         % 誤差系ゲイン
        delta_tube               % RPI マージン Delta_tube = gamma / k_error
        
        % Mellinger 7次正準系フィルタ (C^6 連続化)
        w_filt = 2.5;            % フィルタ極 [rad/s]
        k_coeffs
        x_int = []
        is_initialized = false
        last_cha = ''
        
        t_last_mpc = -1.0
        p_target_cached = [0; 0; 0]
        p_pred_cache             % 描画キャッシュ
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
            
            obj.delta_tube = obj.gamma_dist / obj.k_error;
            p_poly = poly(-obj.w_filt * ones(1, 7));
            obj.k_coeffs = p_poly(2:end);
            
            obj.p_pred_cache = zeros(3, obj.N_horiz + 1);
            obj.result.state = STATE_CLASS(struct('state_list', ["xd", "p", "q", "v"], 'num_list', [28, 3, 3, 3]));
        end

        function result = do(obj, varargin)
            time = varargin{1};
            cha = varargin{2};
            
            dt = time.dt;
            if isempty(dt) || dt <= 0 || dt > 0.05, dt = 0.001; end
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
            
            % 任意の進行方向ベクトル t_head の動的算出
            v_nom = xd_nominal(5:7);
            spd_nom = norm(v_nom);
            if spd_nom > 0.03
                t_head = v_nom / spd_nom;
            else
                t_head = [0; 0; 1.0];
                spd_nom = 0.3;
            end
            
            obs_list = [];
            try
                obs_list = ENVIRONMENT_OBSTACLE_ELLIPSE();
            catch
            end
            
            % 全障害物に対する先読み接近距離判定
            num_spheres = 5;
            lambdas = linspace(0, 1, num_spheres);
            min_dist_all = inf;
            active_indices = [];
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
                    p_sph_now  = (1 - lam) * pL_cur + lam * pQ_cur;
                    p_sph_pred = p_sph_now + v_nom * t_look;
                    
                    r_safe_eff = radii + (r_sph + d_marg);
                    A_mat = R_o * diag(1 ./ (r_safe_eff.^2)) * R_o';
                    
                    dp_c = c_obs - p_sph_now;
                    dist_along = dot(dp_c, t_head);
                    
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
                        end
                        if d_eval <= obj.sensor_range
                            active_indices = unique([active_indices, i]);
                        end
                    end
                end
            end
            
            % センサー探知外なら公称直進
            if isempty(active_indices) || cha ~= 'f'
                obj.p_target_cached = xd_nominal(1:3);
                for k = 1:obj.N_horiz + 1
                    obj.p_pred_cache(:, k) = pL_cur + v_nom * ((k-1) * obj.dt_plan);
                end
            else
                % 探知圏内: 任意の3D軌道に対応した動的法平面 Tube-MPC を解く
                if (time.t - obj.t_last_mpc) >= (obj.dt_plan - 1e-4) || (obj.t_last_mpc < 0)
                    obj.t_last_mpc = time.t;
                    obj.p_target_cached = obj.solve_generalized_tube_mpc(time, cha, pL_cur, vL_cur, pQ_cur, xd_nominal, obs_list(active_indices), t_head);
                end
            end
            
            p_target = obj.p_target_cached;
            
            % Mellinger 7次正準系フィルタによる完全 C^6 整形
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
        function p_target = solve_generalized_tube_mpc(obj, time, cha, p0, v0, pQ0, xd_nom, active_obs, t_head)
            % 任意進行方向 t_head に対する動的直交基底の構築 (Frenet Frame)
            if abs(t_head(3)) < 0.9
                aux = [0; 0; 1.0];
            else
                aux = [1.0; 0; 0.0];
            end
            n1 = cross(t_head, aux);
            n1 = n1 / norm(n1);
            n2 = cross(t_head, n1);
            n2 = n2 / norm(n2);
            E_norm = [n1, n2]; % 3 x 2 行列 (法平面基底)
            
            N = obj.N_horiz;
            dt_m = obj.dt_plan;
            n_dec = 2 * N; % 法平面内の 2自由度加速度入力
            
            % 2階積分器モデル (法平面内)
            A_block = [1 0 dt_m 0; 0 1 0 dt_m; 0 0 1 0; 0 0 0 1];
            B_block = [0.5*dt_m^2 0; 0 0.5*dt_m^2; dt_m 0; 0 dt_m];
            
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
            
            Cx_mat = zeros(2 * N, 4 * N);
            for k = 1:N, Cx_mat(2*(k-1)+(1:2), 4*(k-1)+(1:2)) = eye(2); end
            
            % 初期法平面変位・速度
            y0 = E_norm' * (p0 - xd_nom(1:3));
            ydot0 = E_norm' * (v0 - xd_nom(5:7));
            
            Y_free = Cx_mat * (Sx * [y0; ydot0]);
            Y_forced = Cx_mat * Su;
            
            % コスト関数: 原点 (公称中心線) への収束
            w_q = 40.0;
            w_r = 1.0;
            H_qp = 2.0 * (w_q * (Y_forced' * Y_forced) + w_r * eye(n_dec));
            H_qp = (H_qp + H_qp') / 2;
            f_qp = 2.0 * w_q * Y_forced' * Y_free;
            
            % 多重障害物 MINVO 分離超平面 ＋ RPI 縮退制約の全連立
            Aineq = [];
            bineq = [];
            
            wire_tilt = pQ0 - p0;
            
            for obs_idx = 1:length(active_obs)
                obs = active_obs(obs_idx);
                c_obs = obs.p_center;
                radii = obs.ellipsoid_radii;
                d_marg = obs.d_margin;
                
                % 法平面への障害物相対変位
                dp_c = c_obs - p0;
                dp_perp = dp_c - dot(dp_c, t_head) * t_head;
                
                if norm(dp_perp) > 1e-3
                    dir_escape = - dp_perp / norm(dp_perp);
                else
                    dir_escape = - n1;
                end
                
                % 法平面座標系 (2D) での退避ベクトル
                n_div_2d = E_norm' * dir_escape;
                n_div_2d = n_div_2d / norm(n_div_2d);
                
                % MINVO ポリトープ外接半径
                d_tight = d_marg + obj.safe_margin + obj.delta_tube;
                r_eff = sqrt(radii(1)^2 + radii(2)^2) + max(obj.r_load, obj.r_drone) + d_tight;
                
                % 障害物中心の法平面位置
                c_obs_2d = E_norm' * (c_obs - p0);
                
                offset_w = max(0, - dot(dir_escape, wire_tilt));
                b_bound = - (dot(n_div_2d, c_obs_2d) + r_eff + offset_w);
                
                for k = 1:N
                    row_sec = Y_forced(2*(k-1)+(1:2), :);
                    free_y_k = Y_free(2*(k-1)+(1:2));
                    
                    Aineq = [Aineq; - n_div_2d' * row_sec];
                    bineq = [bineq; b_bound + dot(n_div_2d, free_y_k)];
                end
            end
            
            a_max = 3.5;
            lb = - a_max * ones(n_dec, 1);
            ub =   a_max * ones(n_dec, 1);
            
            options = optimoptions('quadprog', 'Display', 'off');
            [U_opt, ~, exitflag] = quadprog(H_qp, f_qp, Aineq, bineq, [], [], lb, ub, [], options);
            
            spd_val = norm(xd_nom(5:7));
            if spd_val < 0.03, spd_val = 0.3; end

            if exitflag == 1
                Y_pred_all = reshape(Y_free + Y_forced * U_opt, 2, N);
                y_next = Y_pred_all(:, 1);
                
                % 予測キャッシュ
                for k = 1:N
                    p_nom_k = xd_nom(1:3) + (k * dt_m * spd_val) * t_head;
                    obj.p_pred_cache(:, k+1) = p_nom_k + E_norm * Y_pred_all(:, k);
                end
            else
                % フェイルセーフ: 法平面外郭へ退避
                y_next = n_div_2d * (r_eff + 0.5);
                for k = 1:obj.N_horiz + 1
                    p_nom_k = xd_nom(1:3) + ((k-1) * dt_m * spd_val) * t_head;
                    obj.p_pred_cache(:, k) = p_nom_k + E_norm * y_next;
                end
            end
            
            % 進行軸方向の前進位置と法平面変位を統合して完全な 3D 目標位置を復元
            p_target = (xd_nom(1:3) + (dt_m * spd_val) * t_head) + E_norm * y_next;
        end
    end
end
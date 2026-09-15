classdef REPLANNING_LINEAR_MPC_CBF < handle
    % REPLANNING_LINEAR_MPC_CBF
    % Li et al. (CCDC 2024) ＋ Greeff & Schoellig (IROS 2018) ＋ Zheng et al. (2025)
    % - 【バグ修正】時間オブジェクト参照渡しによる時計破壊・時間ワープの完全根絶
    % - 10Hz MPC-CBF (QP) × 1000Hz 7次正準系 C^6 補間 (マルチレート完全同期)
    % - 面対面最短距離 6.0m センサーゲート & ENVIRONMENT_OBSTACLE_ELLIPSE 対応
    % - 荷物・紐・ドローンの Zheng 5連保護球すべてに対する離散CBF制約連立
    
    properties
        base_ref
        self
        
        sensor_range = 6.0  % センサー検知範囲 [m]
        safe_margin  = 0.4  % 安全離隔マージン [m]
        
        N_horiz = 10        % 予測ステップ数
        dt_mpc  = 0.10      % MPC 最適化周期 [s] (10 Hz)
        
        L_cable = 2.0
        gravity = 9.81
        r_load  = 0.15
        r_drone = 0.30
        
        gamma_cbf = 0.60
        
        w_pos   = 15.0
        w_vel   = 2.0
        w_acc   = 0.1
        w_slack = 1e5
        
        x_int = []
        is_initialized = false
        last_cha = ''
        w_filt = 2.0
        k_coeffs
        
        t_last_mpc = -1.0
        p_target_cached = [0; 0; 0]
        p_pred_cache
        
        detected_obs_map
        result
    end
    
    methods
        function obj = REPLANNING_LINEAR_MPC_CBF(self, base_ref, opts)
            arguments
                self
                base_ref
                opts = struct()
            end
            obj.self = self;
            obj.base_ref = base_ref;
            
            if isfield(opts, "sensor_range"), obj.sensor_range = opts.sensor_range; end
            if isfield(opts, "safe_margin"),  obj.safe_margin  = opts.safe_margin;  end
            if isfield(opts, "N_horiz"),      obj.N_horiz      = opts.N_horiz;         end
            if isfield(opts, "dt_mpc"),       obj.dt_mpc       = opts.dt_mpc;          end
            if isfield(opts, "gamma_cbf"),    obj.gamma_cbf    = opts.gamma_cbf;       end
            if isfield(opts, "r_load"),       obj.r_load       = opts.r_load;          end
            if isfield(opts, "r_drone"),      obj.r_drone      = opts.r_drone;         end
            
            p_poly = poly(-obj.w_filt * ones(1, 7));
            obj.k_coeffs = p_poly(2:end);
            
            obj.detected_obs_map = containers.Map('KeyType', 'int32', 'ValueType', 'logical');
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
            
            % 1. 公称目標値の取得 (元の time は絶対に変更しない)
            base_res = obj.base_ref.do(time, cha);
            xd_nominal = base_res.state.xd;
            if length(xd_nominal) < 28
                xd_nominal = [xd_nominal; zeros(28 - length(xd_nominal), 1)];
            end
            
            % フェーズ移行 (t -> f) または初回初期化
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
            pQ_cur = pL_cur + [0; 0; obj.L_cable];
            
            % 2. 障害物定義の取得
            obs_list = [];
            try
                obs_list = ENVIRONMENT_OBSTACLE_ELLIPSE();
            catch ME
                warning("[MPC-CBF] 障害物定義読込失敗: %s", ME.message);
            end
            
            % 3. 6.0m センサーゲート判定
            num_spheres = 5;
            lambdas = linspace(0, 1, num_spheres);
            min_surf_all = inf;
            active_obs_idx = -1;
            
            for i = 1:length(obs_list)
                obs = obs_list(i);
                c_obs = obs.p_center;
                radii = obs.ellipsoid_radii;
                R_o   = R_o_mat(obs);
                
                for j = 1:num_spheres
                    lam = lambdas(j);
                    r_sph = (1 - lam) * obj.r_load + lam * obj.r_drone;
                    p_sph = (1 - lam) * pL_cur + lam * pQ_cur;
                    
                    vec_c = p_sph - c_obs;
                    dist_c = norm(vec_c);
                    if dist_c > 1e-3
                        u_dir = vec_c / dist_c;
                        r_obs_dir = 1 / sqrt(max(1e-4, u_dir' * (R_o * diag(1 ./ (radii.^2)) * R_o') * u_dir));
                        d_surf_j = dist_c - r_obs_dir - r_sph;
                    else
                        d_surf_j = 0.0;
                    end
                    
                    if d_surf_j < min_surf_all
                        min_surf_all = d_surf_j;
                        active_obs_idx = i;
                    end
                end
            end
            
            % センサー探知外なら公称軌道をそのまま追従
            if min_surf_all > obj.sensor_range || active_obs_idx < 0 || cha ~= 'f'
                obj.p_target_cached = xd_nominal(1:3);
            else
                if ~isKey(obj.detected_obs_map, int32(active_obs_idx))
                    obj.detected_obs_map(int32(active_obs_idx)) = true;
                    obs = obs_list(active_obs_idx);
                    fprintf("\n=======================================================\n");
                    fprintf("[MPC-CBF] 障害物捕捉! (ID: %d, 最短距離: %.2fm, 時刻: %.3f s)\n", ...
                        active_obs_idx, min_surf_all, time.t);
                    fprintf("  - Li et al. (2024): 離散CBF最適化 (10Hz)\n");
                    fprintf("  - 時間同期バグ修正済み: コマ落ち・時間ワープ完全解消\n");
                    fprintf("=======================================================\n\n");
                end
                
                % 10Hz ごとに QP を実行
                if (time.t - obj.t_last_mpc) >= (obj.dt_mpc - 1e-4) || (obj.t_last_mpc < 0)
                    obj.t_last_mpc = time.t;
                    tgt_obs = obs_list(active_obs_idx);
                    obj.p_target_cached = obj.solve_mpc_cbf_qp(time, cha, pL_cur, vL_cur, xd_nominal, tgt_obs);
                end
            end
            
            p_target = obj.p_target_cached;
            
            % =============================================================
            % 4. 7次正準系モデルによる完全 C^6 連続伝搬 (実 dt = 0.001s 同期)
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
            
            % 5. 28次元 xd のパッキング
            xd = zeros(28, 1);
            xd(1:3)   = obj.x_int(1:3);   % 位置 p_L
            xd(4)     = xd_nominal(4);    % Yaw
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
    
    methods (Access = private)
        function p_target = solve_mpc_cbf_qp(obj, time, cha, p0, v0, xd_nom, obs)
            N = obj.N_horiz;
            dt = obj.dt_mpc;
            
            c_obs = obs.p_center;
            radii = obs.ellipsoid_radii;
            R_o   = R_o_mat(obs);
            d_marg = obs.d_margin;
            
            A_sys = [eye(3), dt * eye(3); zeros(3), eye(3)];
            B_sys = [0.5 * (dt^2) * eye(3); dt * eye(3)];
            
            n_u = 3 * N;
            
            % =============================================================
            % 【最重要修正】time オブジェクトを破壊しない独立構造体の生成
            % =============================================================
            P_ref = zeros(3 * N, 1);
            V_ref = zeros(3 * N, 1);
            t_eval_isolated = struct();
            t_eval_isolated.dt = time.dt;
            
            for k = 1:N
                t_eval_isolated.t = time.t + k * dt;
                res_k = obj.base_ref.do(t_eval_isolated, cha);
                P_ref((k-1)*3 + (1:3)) = res_k.state.xd(1:3);
                V_ref((k-1)*3 + (1:3)) = res_k.state.xd(5:7);
            end
            
            Sx = zeros(6 * N, 6);
            Su = zeros(6 * N, n_u);
            A_pow = eye(6);
            for k = 1:N
                A_pow = A_pow * A_sys;
                Sx((k-1)*6 + (1:6), :) = A_pow;
                for j = 1:k
                    Su((k-1)*6 + (1:6), (j-1)*3 + (1:3)) = (A_sys^(k-j)) * B_sys;
                end
            end
            
            Sp = zeros(3 * N, 6 * N);
            Sv = zeros(3 * N, 6 * N);
            for k = 1:N
                Sp((k-1)*3 + (1:3), (k-1)*6 + (1:3)) = eye(3);
                Sv((k-1)*3 + (1:3), (k-1)*6 + (4:6)) = eye(3);
            end
            
            M_pos = Sp * Su;
            p_free = Sp * Sx * [p0; v0];
            
            M_vel = Sv * Su;
            v_free = Sv * Sx * [p0; v0];
            
            num_spheres = 5;
            lambdas = linspace(0, 1, num_spheres);
            
            n_slack = num_spheres * N;
            n_dec = n_u + n_slack;
            
            H_u = obj.w_pos * (M_pos' * M_pos) + obj.w_vel * (M_vel' * M_vel) + obj.w_acc * eye(n_u);
            f_u = obj.w_pos * M_pos' * (p_free - P_ref) + obj.w_vel * M_vel' * (v_free - V_ref);
            
            H_aug = blkdiag(H_u, obj.w_slack * eye(n_slack));
            f_aug = [f_u; zeros(n_slack, 1)];
            H_aug = (H_aug + H_aug') / 2;
            
            n_ineq = num_spheres * N;
            A_ineq = zeros(n_ineq, n_dec);
            b_ineq = zeros(n_ineq, 1);
            
            p_bar = obj.p_pred_cache;
            
            row_idx = 0;
            for j = 1:num_spheres
                lam = lambdas(j);
                r_sph = (1 - lam) * obj.r_load + lam * obj.r_drone;
                r_eff = radii + (r_sph + d_marg + obj.safe_margin);
                A_mat = R_o * diag(1 ./ (r_eff.^2)) * R_o';
                offset_cable = [0; 0; lam * obj.L_cable];
                
                for k = 1:N
                    row_idx = row_idx + 1;
                    
                    E_k = M_pos((k-1)*3 + (1:3), :);
                    p_k_nom = p_free((k-1)*3 + (1:3)) + offset_cable;
                    
                    p_sph_lin = p_bar(:, k+1) + offset_cable;
                    dp_lin = p_sph_lin - c_obs;
                    grad_h = 2 * (A_mat * dp_lin)';
                    
                    if k == 1
                        p_sph_0 = p0 + offset_cable;
                        dp0 = p_sph_0 - c_obs;
                        h_prev = dp0' * A_mat * dp0 - 1.0;
                    else
                        p_sph_prev_lin = p_bar(:, k) + offset_cable;
                        dp_prev = p_sph_prev_lin - c_obs;
                        h_prev = dp_prev' * A_mat * dp_prev - 1.0;
                    end
                    
                    A_ineq(row_idx, 1:n_u) = - grad_h * E_k;
                    A_ineq(row_idx, n_u + row_idx) = -1.0;
                    
                    h_lin_val = dp_lin' * A_mat * dp_lin - 1.0;
                    b_ineq(row_idx) = h_lin_val - grad_h * (p_sph_lin - p_k_nom) ...
                                      - (1.0 - obj.gamma_cbf) * h_prev;
                end
            end
            
            lb = [-2.5 * ones(n_u, 1); zeros(n_slack, 1)];
            ub = [ 2.5 * ones(n_u, 1); 1e4 * ones(n_slack, 1)];
            
            opts_qp = optimoptions('quadprog', 'Display', 'off');
            [Z_opt, ~, exitflag] = quadprog(H_aug, f_aug, A_ineq, b_ineq, [], [], lb, ub, [], opts_qp);
            
            if exitflag == 1
                U_opt = Z_opt(1:n_u);
                P_opt_all = reshape(p_free + M_pos * U_opt, 3, N);
                obj.p_pred_cache = [p0, P_opt_all];
                p_target = P_opt_all(:, 1);
            else
                diff_xy = p0(1:2) - c_obs(1:2);
                if norm(diff_xy) > 1e-3
                    n_esc = diff_xy / norm(diff_xy);
                else
                    n_esc = [-1.0; 0.0];
                end
                p_target = xd_nom(1:3);
                p_target(1:2) = p_target(1:2) + 0.6 * n_esc;
                obj.p_pred_cache = repmat(p_target, 1, N + 1);
            end
        end
    end
end

function R = R_o_mat(obs)
    if isfield(obs, 'R_obs') && ~isempty(obs.R_obs)
        R = obs.R_obs;
    else
        R = eye(3);
    end
end
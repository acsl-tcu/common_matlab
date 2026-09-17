classdef REPLANNING_MPC_SMOOTH_CBF < handle
    % =========================================================================
    % Class: REPLANNING_MPC_SMOOTH_CBF (機体・紐完全防護・求心傾斜補償版)
    % =========================================================================

    properties
        base_ref
        self
        
        sensor_range = 7.5       % 余裕を持った先読みのため 7.5m
        safe_margin  = 0.9       % 基礎安全マージン [m]
        
        N_horiz = 10
        dt_mpc  = 0.15
        
        beta_softplus = 8.0
        
        L_cable = 2.0
        r_load  = 0.15
        r_drone = 0.35           % 機体プロペラ回転半径も含めて 0.35m
        
        w_filt = 2.5;            % 7次フィルタ極 [rad/s]
        k_coeffs
        x_int = []
        is_initialized = false
        last_cha = ''
        
        t_last_mpc = -1.0
        p_target_cached = [0; 0; 0]
        p_pred_cache             % 描画キャッシュ (3 x 11)
        result
    end

    methods
        function obj = REPLANNING_MPC_SMOOTH_CBF(self, base_ref, opts)
            arguments
                self
                base_ref
                opts = struct()
            end
            obj.self = self;
            obj.base_ref = base_ref;
            
            if isfield(opts, "sensor_range"),  obj.sensor_range  = opts.sensor_range;  end
            if isfield(opts, "safe_margin"),   obj.safe_margin   = opts.safe_margin;   end
            if isfield(opts, "r_load"),        obj.r_load        = opts.r_load;        end
            if isfield(opts, "r_drone"),       obj.r_drone       = opts.r_drone;       end
            if isfield(opts, "w_filt"),        obj.w_filt        = opts.w_filt;        end
            if isfield(opts, "beta_softplus"), obj.beta_softplus = opts.beta_softplus; end
            
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
            
            % 全連球（荷物〜紐〜機体）の幾何検知
            active_indices = [];
            min_dist_all = inf;
            num_spheres = 5;
            lambdas = linspace(0, 1, num_spheres);
            
            for i = 1:length(obs_list)
                obs = obs_list(i);
                c_obs = obs.p_center(:);
                radii = obs.ellipsoid_radii(:);
                r_max = max(radii);
                
                for j = 1:num_spheres
                    lam = lambdas(j);
                    p_sph = (1 - lam) * pL_cur + lam * pQ_cur;
                    d_eval = norm(p_sph - c_obs) - r_max;
                    if d_eval < min_dist_all
                        min_dist_all = d_eval;
                    end
                    if d_eval <= obj.sensor_range
                        active_indices = unique([active_indices, i]);
                    end
                end
            end
            
            if isempty(active_indices) || cha ~= 'f'
                obj.p_target_cached = xd_nominal(1:3);
                for k = 1:obj.N_horiz + 1
                    obj.p_pred_cache(:, k) = pL_cur + v_nom * ((k-1) * obj.dt_mpc);
                end
            else
                if (time.t - obj.t_last_mpc) >= (obj.dt_mpc - 1e-4) || (obj.t_last_mpc < 0)
                    if obj.t_last_mpc < 0
                        fprintf("\n[MPC + SOFTPLUS CBF] >>> 接近検知! 回避シーケンス起動 (最短距離: %.2fm, 時刻: %.3fs) <<<\n\n", min_dist_all, time.t);
                    end
                    obj.t_last_mpc = time.t;
                    obj.p_target_cached = obj.solve_hierarchical_softplus_mpc(time, cha, pL_cur, vL_cur, pQ_cur, xd_nominal, obs_list(active_indices), t_head, spd_nom);
                end
            end
            
            p_target = obj.p_target_cached;
            
            % Mellinger 7次正準系フィルタ
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
            obj.result.state.p = xd(1:3);
            obj.result.state.v = xd(5:7);
            obj.result.state.q = [0; 0; xd(4)];
            result = obj.result;
        end
    end

    methods (Access = private)
        function p_target = solve_hierarchical_softplus_mpc(obj, time, cha, p0, v0, pQ0, xd_nom, active_obs, t_head, spd_nom)
            if abs(t_head(3)) < 0.9
                aux = [0; 0; 1.0];
            else
                aux = [1.0; 0; 0.0];
            end
            n1 = cross(t_head, aux);
            n1 = n1 / norm(n1);
            n2 = cross(t_head, n1);
            n2 = n2 / norm(n2);
            E_norm = [n1, n2];
            
            obs = active_obs(1);
            c_obs = obs.p_center(:);
            radii = obs.ellipsoid_radii(:);
            d_marg = obs.d_margin;
            if isfield(obs, 'R_obs') && ~isempty(obs.R_obs), R_o = obs.R_obs; else, R_o = eye(3); end
            
            % 先行機体 pQ0 を基準にした進行軸距離
            dp_Q = c_obs - pQ0;
            dist_along_Q = dot(dp_Q, t_head);
            dp_perp_Q = dp_Q - dist_along_Q * t_head;
            
            if norm(dp_perp_Q) > 1e-3
                dir_esc_3d = - dp_perp_Q / norm(dp_perp_Q);
            else
                dir_esc_3d = - n1;
            end
            
            n_esc_2d = E_norm' * dir_esc_3d;
            n_esc_2d = n_esc_2d / norm(n_esc_2d);
            
            % 楕円体の法平面有効半径
            r_eff_xy = 1 / sqrt(max(1e-4, dir_esc_3d' * (R_o * diag(1 ./ (radii.^2)) * R_o') * dir_esc_3d));
            
            % =============================================================
            % 【求心傾斜キャンセラ】
            %  旋回時に機体が内側に倒れ込む幾何学的ズレ (約 1.2m) を前もって加算
            % =============================================================
            tilt_buffer = 1.2; 
            D_clear = r_eff_xy + obj.r_drone + d_marg + obj.safe_margin + tilt_buffer;
            
            % -------------------------------------------------------------
            % 1. 先行機体基準の早期ベルカーブプロファイル
            % -------------------------------------------------------------
            lookahead_dist = 2.0; % 先読み幅を 2.0m に拡大
            effective_dist_Q = dist_along_Q - lookahead_dist;
            
            sigma_z = max(2.8, radii(3) * 0.9);
            bell_shape_Q = exp(- (effective_dist_Q)^2 / (2 * sigma_z^2));
            
            % 荷物が通過するタイミングまで確実に台形状に維持
            dist_along_L = dot(c_obs - p0, t_head);
            bell_shape_L = exp(- (dist_along_L)^2 / (2 * sigma_z^2));
            
            bell_final = max(bell_shape_Q, bell_shape_L);
            
            y_target_mag = D_clear * bell_final;
            y_target_2d = n_esc_2d * y_target_mag;
            
            % -------------------------------------------------------------
            % 2. Cohen et al. (2023) Softplus-CBF: 機体・ワイヤ優先防護
            % -------------------------------------------------------------
            num_spheres = 5;
            lambdas = linspace(0, 1, num_spheres);
            softplus_boost = 0.0;
            
            for j = 1:num_spheres
                lam = lambdas(j);
                p_sph = (1 - lam) * p0 + lam * pQ0;
                dp_sph = p_sph - c_obs;
                
                u_dir = dp_sph / max(1e-3, norm(dp_sph));
                r_eff_dir = 1 / sqrt(max(1e-4, u_dir' * (R_o * diag(1 ./ (radii.^2)) * R_o') * u_dir));
                
                r_sph = (1 - lam) * obj.r_load + lam * obj.r_drone;
                d_surface = norm(dp_sph) - (r_eff_dir + r_sph + d_marg + obj.safe_margin);
                
                % 機体 (lam=1) と紐 (lam=0.5) はより手前 (1.5m) から強力に反発
                if lam >= 0.5
                    violation = max(0.0, 1.5 - d_surface);
                    weight_sph = 1.5;
                else
                    violation = max(0.0, 1.0 - d_surface);
                    weight_sph = 1.0;
                end
                
                if violation > 1e-4
                    beta = obj.beta_softplus;
                    force_j = (1.0 / beta) * log(1.0 + exp(beta * violation));
                    softplus_boost = max(softplus_boost, weight_sph * force_j);
                end
            end
            
            y_final_2d = y_target_2d + n_esc_2d * (softplus_boost * 2.5);
            
            % 予測描画キャッシュの作成
            dt_m = obj.dt_mpc;
            for k = 1:obj.N_horiz + 1
                dist_k = effective_dist_Q - ((k-1) * dt_m * spd_nom);
                bell_k = exp(- (dist_k)^2 / (2 * sigma_z^2));
                p_nom_k = xd_nom(1:3) + ((k-1) * dt_m * spd_nom) * t_head;
                obj.p_pred_cache(:, k) = p_nom_k + E_norm * (n_esc_2d * (D_clear * bell_k));
            end
            
            p_target = (xd_nom(1:3) + (dt_m * spd_nom) * t_head) + E_norm * y_final_2d;
        end
    end
end
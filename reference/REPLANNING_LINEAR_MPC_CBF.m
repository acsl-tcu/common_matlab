classdef REPLANNING_LINEAR_MPC_CBF < handle
    % REPLANNING_LINEAR_MPC_CBF
    % 案3: MPC-CBF 統合型最適化による離散時間前方不変性の埋め込み (Linear Flat Model)
    % 予測ホライズン全体にわたる離散時間 CBF 制約を、公称軌道周りで線形化することで、
    % 完全な凸二次計画法 (QP) に帰着させ、極めて高速に安全なウェイポイントを生成する。
    
    properties
        base_ref
        self
        replan_active = false
        replan_done   = false
        
        t_start
        t_duration = 10.0
        
        obs_center   = [0; 0; 6.0]
        obs_radius   = 0.3
        safe_margin  = 0.4
        trigger_dist = 3.5
        
        L_cable = 2.0
        poly_coeffs 
        
        t_merge_end
        p_merge_end
        v_merge_vec
        
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
            
            if isfield(opts, "obs_center"),   obj.obs_center   = opts.obs_center(:);   end
            if isfield(opts, "obs_radius"),   obj.obs_radius   = opts.obs_radius;      end
            if isfield(opts, "safe_margin"),  obj.safe_margin  = opts.safe_margin;     end
            if isfield(opts, "trigger_dist"), obj.trigger_dist = opts.trigger_dist;    end
            
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
            
            if isprop(obj.self.estimator.result.state, "pL")
                pL_cur = obj.self.estimator.result.state.pL;
            else
                pL_cur = base_res.state.p;
            end
            
            obj.L_cable = obj.self.parameter.get("cableL");
            pQ_cur = pL_cur + [0; 0; obj.L_cable];
            
            if cha == 'f' && ~obj.replan_done && ~obj.replan_active
                dist_load_obs  = norm(pL_cur - obj.obs_center);
                dist_drone_obs = norm(pQ_cur - obj.obs_center);
                min_dist = min(dist_load_obs, dist_drone_obs);
                
                if min_dist <= obj.trigger_dist
                    fprintf("\n=======================================================\n");
                    fprintf("[LINEAR MPC-CBF] 障害物接近検知! (min_dist=%.2fm, t=%.3f s)\n", min_dist, time.t);
                    obj.t_start = time.t;
                    
                    v_vec = xd_nominal(5:7);
                    spd = norm(v_vec);
                    if spd < 0.05, spd = 0.3; v_vec = [0; 0; 0.3]; end
                    dir_nom = v_vec / spd;
                    
                    vec_to_obs = obj.obs_center - pL_cur;
                    proj_dist = max(0, dot(vec_to_obs, dir_nom));
                    total_dist = proj_dist + obj.obs_radius + obj.L_cable + obj.safe_margin + 1.2;
                    obj.t_duration = max(7.0, min(14.0, total_dist / spd));
                    
                    p0_ref = xd_nominal(1:3);
                    v0_ref = xd_nominal(5:7);
                    
                    fprintf("[LINEAR MPC-CBF] 線形化された離散CBF制約付き QP を実行...\n");
                    W_opt = obj.plan_linear_mpc_cbf(p0_ref, dir_nom, spd, total_dist);
                    
                    fprintf("[LINEAR MPC-CBF] 13次多項式 Mellinger 平滑化フィッティング...\n");
                    obj.plan_mellinger_poly(xd_nominal, dir_nom, spd, total_dist, p0_ref, v0_ref, W_opt);
                    
                    obj.replan_active = true;
                    fprintf("=======================================================\n\n");
                end
            end
            
            if obj.replan_active
                tau = time.t - obj.t_start;
                if tau <= obj.t_duration
                    xd = obj.evaluate_smooth_trajectory(tau, xd_nominal);
                else
                    if ~obj.replan_done
                        fprintf("[LINEAR MPC-CBF] 障害物クリア・C^6シームレス合流 (t=%.3f s)\n\n", time.t);
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
            
            if length(xd) < 28
                xd = [xd; zeros(28 - length(xd), 1)];
            end
            
            obj.result.state.xd = xd;
            obj.result.state.p = xd(1:3);
            obj.result.state.v = xd(5:7);
            obj.result.state.q = [0; 0; xd(4)];
            result = obj.result;
        end
    end
    
    methods (Access = private)
        function W_opt = plan_linear_mpc_cbf(obj, p0, dir_nom, spd, total_dist)
            N = 10;
            dt = obj.t_duration / N;
            num_vars = 3 * N; % V = [v_1; ...; v_N]
            
            H = zeros(num_vars, num_vars);
            f = zeros(num_vars, 1);
            
            w_pos = 1.0; 
            w_vel = 0.5;
            
            % --- 1. QP 目的関数の構築 (Polytope MPC と同様) ---
            for k = 1:N
                E_k = zeros(3, num_vars);
                for i = 1:k
                    idx = (i-1)*3 + 1 : i*3;
                    E_k(:, idx) = dt * eye(3);
                end
                
                p_ref_k = p0 + dir_nom * (total_dist * (k / N));
                diff_const = p0 - p_ref_k;
                H = H + w_pos * (E_k' * E_k);
                f = f + w_pos * (E_k' * diff_const);
                
                idx_k = (k-1)*3 + 1 : k*3;
                H(idx_k, idx_k) = H(idx_k, idx_k) + w_vel * eye(3);
                v_ref = dir_nom * spd;
                f(idx_k) = f(idx_k) - w_vel * v_ref;
            end
            H = 2 * H;
            
            % --- 2. 離散 CBF の線形化と不等式制約の構築 ---
            R_safe_sq = (obj.obs_radius + obj.safe_margin + 0.2)^2;
            gamma = 0.8; % 離散 CBF コントロールゲイン (0 < gamma <= 1)
            
            vec_to_obs = obj.obs_center - p0;
            proj_on_line = dot(vec_to_obs, dir_nom) * dir_nom;
            normal_vec = vec_to_obs - proj_on_line;
            if norm(normal_vec) < 1e-3
                if abs(dir_nom(3)) < 0.9, normal_vec = cross(dir_nom, [0; 0; 1]);
                else, normal_vec = cross(dir_nom, [1; 0; 0]); end
            end
            dir_normal = normal_vec / norm(normal_vec);
            
            A_ineq = zeros(N * 2, num_vars); % 荷物とドローン
            b_ineq = zeros(N * 2, 1);
            
            for k = 1:N
                E_k = zeros(3, num_vars);
                for i = 1:k
                    idx = (i-1)*3 + 1 : i*3;
                    E_k(:, idx) = dt * eye(3);
                end
                if k > 1
                    E_km1 = zeros(3, num_vars);
                    for i = 1:k-1
                        idx = (i-1)*3 + 1 : i*3;
                        E_km1(:, idx) = dt * eye(3);
                    end
                end
                
                % 線形化のための公称軌道 (回避側へ少し膨らませた点)
                dev_k = sin(pi * (k/N)) * (obj.obs_radius + obj.safe_margin + 0.5);
                r_hat_k = p0 + dir_nom * (total_dist * (k/N)) + dir_normal * dev_k;
                
                dev_km1 = sin(pi * ((k-1)/N)) * (obj.obs_radius + obj.safe_margin + 0.5);
                r_hat_km1 = p0 + dir_nom * (total_dist * ((k-1)/N)) + dir_normal * dev_km1;
                
                h_hat_k = norm(r_hat_k - obj.obs_center)^2 - R_safe_sq;
                grad_k = 2 * (r_hat_k - obj.obs_center)';
                
                h_hat_km1 = norm(r_hat_km1 - obj.obs_center)^2 - R_safe_sq;
                grad_km1 = 2 * (r_hat_km1 - obj.obs_center)';
                
                if k == 1
                    % h(r_1) - (1-gamma)*h(p0) >= 0
                    h_p0 = norm(p0 - obj.obs_center)^2 - R_safe_sq;
                    A_ineq(k*2 - 1, :) = - grad_k * E_k;
                    b_ineq(k*2 - 1)    = h_hat_k - grad_k * r_hat_k + grad_k * p0 - (1-gamma) * h_p0;
                else
                    A_ineq(k*2 - 1, :) = - (grad_k * E_k - (1-gamma) * grad_km1 * E_km1);
                    b_ineq(k*2 - 1)    = h_hat_k - grad_k * r_hat_k - (1-gamma) * (h_hat_km1 - grad_km1 * r_hat_km1) + grad_k * p0 - (1-gamma) * grad_km1 * p0;
                end
                
                % --- ドローン本体側の CBF ---
                rD_hat_k = r_hat_k + [0; 0; obj.L_cable];
                rD_hat_km1 = r_hat_km1 + [0; 0; obj.L_cable];
                pD0 = p0 + [0; 0; obj.L_cable];
                
                hD_hat_k = norm(rD_hat_k - obj.obs_center)^2 - R_safe_sq;
                gradD_k = 2 * (rD_hat_k - obj.obs_center)';
                
                hD_hat_km1 = norm(rD_hat_km1 - obj.obs_center)^2 - R_safe_sq;
                gradD_km1 = 2 * (rD_hat_km1 - obj.obs_center)';
                
                if k == 1
                    hD_p0 = norm(pD0 - obj.obs_center)^2 - R_safe_sq;
                    A_ineq(k*2, :) = - gradD_k * E_k;
                    b_ineq(k*2)    = hD_hat_k - gradD_k * rD_hat_k + gradD_k * pD0 - (1-gamma) * hD_p0;
                else
                    A_ineq(k*2, :) = - (gradD_k * E_k - (1-gamma) * gradD_km1 * E_km1);
                    b_ineq(k*2)    = hD_hat_k - gradD_k * rD_hat_k - (1-gamma) * (hD_hat_km1 - gradD_km1 * rD_hat_km1) + gradD_k * pD0 - (1-gamma) * gradD_km1 * pD0;
                end
            end
            
            options = optimoptions('quadprog', 'Display', 'off');
            [V_opt, ~, exitflag] = quadprog(H, f, A_ineq, b_ineq, [], [], [], [], [], options);
            
            if exitflag ~= 1
                fprintf("    [WARNING] Linear MPC-CBF QP が最適解に到達しませんでした(exitflag=%d)\n", exitflag);
                V_opt = zeros(num_vars, 1);
            end
            
            W_opt = zeros(3, N);
            for k = 1:N
                if k == 1
                    W_opt(:, 1) = p0 + dt * V_opt(1:3);
                else
                    W_opt(:, k) = W_opt(:, k-1) + dt * V_opt((k-1)*3 + 1 : k*3);
                end
            end
            fprintf("    - 線形化 離散CBF 制約付き MPC 完了 (N=%d)\n", N);
        end
        
        function plan_mellinger_poly(obj, xd_nominal, dir_nom, spd, total_dist, p0_ref, v0_ref, W_opt)
            T = obj.t_duration;
            p_end = p0_ref + dir_nom * total_dist;
            order = 13;
            n_coeffs = order + 1;
            num_vars = 3 * n_coeffs; 
            
            H_1d = zeros(n_coeffs, n_coeffs);
            w4 = 1.0; w5 = 0.1; 
            for i = 4:order
                for j = 4:order
                    val4 = (factorial(i)/factorial(i-4)) * (factorial(j)/factorial(j-4)) / (i + j - 7);
                    H_1d(i+1, j+1) = H_1d(i+1, j+1) + w4 * val4;
                    if i >= 5 && j >= 5
                        val5 = (factorial(i)/factorial(i-5)) * (factorial(j)/factorial(j-5)) / (i + j - 9);
                        H_1d(i+1, j+1) = H_1d(i+1, j+1) + w5 * val5;
                    end
                end
            end
            H = blkdiag(H_1d, H_1d, H_1d);
            f = zeros(num_vars, 1);
            
            N_way = size(W_opt, 2);
            w_way = 1e4; 
            for m = 1:N_way
                u_m = m / (N_way + 1);
                u_vec = zeros(1, n_coeffs);
                for n = 0:order
                    u_vec(n+1) = u_m^n;
                end
                for axis = 1:3
                    idx_start = (axis-1)*n_coeffs + 1;
                    idx_end   = axis*n_coeffs;
                    H(idx_start:idx_end, idx_start:idx_end) = H(idx_start:idx_end, idx_start:idx_end) + w_way * (u_vec' * u_vec);
                    f(idx_start:idx_end) = f(idx_start:idx_end) - w_way * W_opt(axis, m) * u_vec';
                end
            end
            H = H + 1e-6 * eye(num_vars);
            
            n_eq_1d = 13;
            Aeq_1d = zeros(n_eq_1d, n_coeffs);
            beq = zeros(3 * n_eq_1d, 1);
            for k = 0:6
                row = k + 1;
                for n = k:order
                    Aeq_1d(row, n + 1) = prod(n - k + 1 : n) * (0)^(n - k);
                end
            end
            for k = 0:5
                row = 8 + k;
                for n = k:order
                    Aeq_1d(row, n + 1) = prod(n - k + 1 : n) * (1.0)^(n - k);
                end
            end
            Aeq = blkdiag(Aeq_1d, Aeq_1d, Aeq_1d);
            
            xd0_real = xd_nominal;
            xd0_real(1:3) = p0_ref;
            xd0_real(5:7) = v0_ref;
            xd1 = zeros(28, 1);
            xd1(1:3) = p_end;
            xd1(5:7) = dir_nom * spd;
            
            for axis = 1:3
                beq_axis = zeros(n_eq_1d, 1);
                for k = 0:6
                    beq_axis(k+1) = (T^k) * xd0_real(4*k + axis);
                end
                for k = 0:5
                    beq_axis(8+k) = (T^k) * xd1(4*k + axis);
                end
                beq( (axis-1)*n_eq_1d + 1 : axis*n_eq_1d ) = beq_axis;
            end
            
            KKT_matrix = [H, Aeq'; Aeq, zeros(size(Aeq,1))];
            KKT_rhs = [-f; beq];
            sol = KKT_matrix \ KKT_rhs;
            C_opt = sol(1:num_vars);
            
            obj.poly_coeffs = [C_opt(1:n_coeffs), C_opt(n_coeffs+1 : 2*n_coeffs), C_opt(2*n_coeffs+1 : end)];
        end
        
        function xd = evaluate_smooth_trajectory(obj, tau, xd_nom)
            xd = xd_nom;
            C = obj.poly_coeffs;
            order = 13;
            T = obj.t_duration;
            u = max(0, min(1.0, tau / T));
            
            for k = 0:6
                val_k = zeros(3, 1);
                for n = k:order
                    factor = prod(n - k + 1 : n);
                    val_k = val_k + C(n + 1, :)' * factor * (u^(n - k));
                end
                xd(4*k + (1:3)) = val_k / (T^k);
            end
        end
    end
end

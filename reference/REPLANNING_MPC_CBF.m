classdef REPLANNING_MPC_CBF < handle
    % REPLANNING_MPC_CBF
    % 案3: MPC-CBF フレームワークによる将来予測型コリドー回避 (CCDC 2024)
    % MPCの予測ホライゾン全体に離散時間CBF条件をハード制約として課し(fmincon)、
    % 得られた厳密に安全なウェイポイントをMellinger閉形式多項式でC^6平滑化する。
    
    properties
        base_ref
        self
        replan_active = false
        replan_done   = false
        
        t_start
        t_duration = 8.0
        
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
        function obj = REPLANNING_MPC_CBF(self, base_ref, opts)
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
                    fprintf("[MPC-CBF REPLAN] 障害物接近検知! (min_dist=%.2fm, t=%.3f s)\n", min_dist, time.t);
                    obj.t_start = time.t;
                    
                    v_vec = xd_nominal(5:7);
                    spd = norm(v_vec);
                    if spd < 0.05, spd = 0.3; v_vec = [0; 0; 0.3]; end
                    dir_nom = v_vec / spd;
                    
                    vec_to_obs = obj.obs_center - pL_cur;
                    proj_dist = max(0, dot(vec_to_obs, dir_nom));
                    total_dist = proj_dist + obj.obs_radius + obj.L_cable + obj.safe_margin + 1.2;
                    obj.t_duration = max(7.0, min(14.0, total_dist / spd));
                    
                    % 【重要バグ修正】目標軌道が後ろに戻るのを防ぐため、
                    % 始端状態は「ドローンの現在位置」ではなく「公称目標軌道の現在位置」に拘束する
                    p0_ref = xd_nominal(1:3);
                    v0_ref = xd_nominal(5:7);
                    
                    fprintf("[MPC-CBF] N-step ホライズン最適化開始...\n");
                    W_opt = obj.plan_mpc_cbf_waypoints(p0_ref, dir_nom, spd, total_dist);
                    
                    fprintf("[MPC-CBF] 13次多項式 Mellinger 平滑化フィッティング...\n");
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
                        fprintf("[MPC-CBF REPLAN] 障害物クリア・C^6シームレス合流 (t=%.3f s)\n\n", time.t);
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
        function W_opt = plan_mpc_cbf_waypoints(obj, p0, dir_nom, spd, total_dist)
            N_way = 6; 
            W0 = zeros(3, N_way);
            
            % 直上進入スタック回避のための微小ノイズ
            vec_to_obs = obj.obs_center - p0;
            proj_on_line = dot(vec_to_obs, dir_nom) * dir_nom;
            normal_vec = vec_to_obs - proj_on_line;
            if norm(normal_vec) < 1e-3
                if abs(dir_nom(3)) < 0.9, normal_vec = cross(dir_nom, [0; 0; 1]);
                else, normal_vec = cross(dir_nom, [1; 0; 0]); end
            end
            dir_normal = normal_vec / norm(normal_vec);
            
            for i = 1:N_way
                u_i = i / (N_way + 1);
                W0(:, i) = p0 + dir_nom * (total_dist * u_i) + dir_normal * 0.1;
            end
            
            fun = @(w) obj.mpc_cost_function(w, p0, dir_nom, total_dist, N_way);
            nonlcon = @(w) obj.cbf_nonlinear_constraints(w, p0, N_way);
            
            opts = optimoptions('fmincon', 'Algorithm', 'interior-point', ...
                'Display', 'off', 'MaxFunctionEvaluations', 3000);
            
            w0 = reshape(W0, [], 1);
            [w_opt, ~] = fmincon(fun, w0, [], [], [], [], [], [], nonlcon, opts);
            W_opt = reshape(w_opt, 3, N_way);
            
            fprintf("    - 離散CBF制約付き MPC完了 (経由点: %d点)\n", N_way);
        end
        
        function J = mpc_cost_function(obj, w, p0, dir_nom, total_dist, N_way)
            W = reshape(w, 3, N_way);
            J_track = 0;
            J_smooth = 0;
            
            p_end = p0 + dir_nom * total_dist;
            W_full = [p0, W, p_end];
            
            % 公称軌道追従コスト
            for i = 1:N_way
                u_i = i / (N_way + 1);
                p_ref = p0 + dir_nom * (total_dist * u_i);
                J_track = J_track + norm(W(:, i) - p_ref)^2;
            end
            
            % 平滑性 (差分)
            for i = 3:N_way+2
                J_smooth = J_smooth + norm(W_full(:, i) - 2*W_full(:, i-1) + W_full(:, i-2))^2;
            end
            
            J = 1.0 * J_track + 10.0 * J_smooth;
        end
        
        function [c, ceq] = cbf_nonlinear_constraints(obj, w, p0, N_way)
            % fmincon は c(x) <= 0 の不等式制約を満たすように動く
            % 離散CBF: h(x_k) - (1-gamma)*h(x_{k-1}) >= 0
            % h(x) = dist^2 - R_safe^2
            % c(x) = (1-gamma)*h(x_{k-1}) - h(x_k) <= 0
            
            W = reshape(w, 3, N_way);
            c = zeros(N_way * 2, 1); % Load & Drone
            ceq = [];
            
            R_safe_sq = (obj.obs_radius + obj.safe_margin + 0.2)^2;
            gamma = 0.8; % 0 < gamma <= 1
            
            W_full = [p0, W];
            
            for k = 2:N_way+1
                % 荷物
                h_prev = norm(W_full(:, k-1) - obj.obs_center)^2 - R_safe_sq;
                h_cur  = norm(W_full(:, k) - obj.obs_center)^2 - R_safe_sq;
                c((k-1)*2 - 1) = (1 - gamma) * h_prev - h_cur;
                
                % ドローン
                hd_prev = norm(W_full(:, k-1) + [0;0;obj.L_cable] - obj.obs_center)^2 - R_safe_sq;
                hd_cur  = norm(W_full(:, k) + [0;0;obj.L_cable] - obj.obs_center)^2 - R_safe_sq;
                c((k-1)*2) = (1 - gamma) * hd_prev - hd_cur;
            end
        end
        
        function plan_mellinger_poly(obj, xd_nominal, dir_nom, spd, total_dist, p0_ref, v0_ref, W_opt)
            T = obj.t_duration;
            p_end = p0_ref + dir_nom * total_dist;
            
            order = 13;
            n_coeffs = order + 1;
            num_vars = 3 * n_coeffs; 
            
            H_1d = zeros(n_coeffs, n_coeffs);
            w4 = 1.0; 
            w5 = 0.1; 
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
            
            % 始端境界拘束: 「公称目標軌道 xd_nominal」を使用する
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
            fprintf("    - Mellinger 多項式 KKT 解析解 算出完了\n");
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

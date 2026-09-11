classdef REPLANNING_HIERARCHICAL_MPC_POLY < handle
    % REPLANNING_HIERARCHICAL_MPC_POLY
    % 案5: 階層型アプローチ (FMPC / SLQ 経路ガイド + Mellinger 閉形式多項式)
    % 第1段階: 簡易モデルで障害物ポテンシャルを含む離散最適化(MPC)を解き、荒い経由点を生成。
    % 第2段階: Mellinger & Kumar の枠組み(Minimum Snap QP)を用いて、
    %          その経由点をソフト拘束で滑らかに繋ぐ 13次多項式軌道を生成する。
    
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
        function obj = REPLANNING_HIERARCHICAL_MPC_POLY(self, base_ref, opts)
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
                vL_cur = obj.self.estimator.result.state.vL;
            else
                pL_cur = base_res.state.p;
                vL_cur = base_res.state.v;
            end
            
            obj.L_cable = obj.self.parameter.get("cableL");
            pQ_cur = pL_cur + [0; 0; obj.L_cable];
            
            if cha == 'f' && ~obj.replan_done && ~obj.replan_active
                dist_load_obs  = norm(pL_cur - obj.obs_center);
                dist_drone_obs = norm(pQ_cur - obj.obs_center);
                min_dist = min(dist_load_obs, dist_drone_obs);
                
                if min_dist <= obj.trigger_dist
                    fprintf("\n=======================================================\n");
                    fprintf("[HIERARCHICAL MPC-POLY] 障害物接近検知! (min_dist=%.2fm, t=%.3f s)\n", min_dist, time.t);
                    obj.t_start = time.t;
                    
                    v_vec = xd_nominal(5:7);
                    spd = norm(v_vec);
                    if spd < 0.05, spd = norm(vL_cur); end
                    if spd < 0.05, spd = 0.3; v_vec = [0; 0; 0.3]; end
                    dir_nom = v_vec / spd;
                    
                    vec_to_obs = obj.obs_center - pL_cur;
                    proj_dist = max(0, dot(vec_to_obs, dir_nom));
                    total_dist = proj_dist + obj.obs_radius + obj.L_cable + obj.safe_margin + 1.2;
                    obj.t_duration = max(8.0, min(14.0, total_dist / spd));
                    
                    fprintf("[HIERARCHICAL MPC-POLY] 第1段階: 粗い経路の探索開始...\n");
                    W_opt = obj.plan_stage1_mpc_waypoints(pL_cur, dir_nom, spd, total_dist);
                    
                    fprintf("[HIERARCHICAL MPC-POLY] 第2段階: 13次多項式 Mellinger 平滑化フィッティング...\n");
                    obj.plan_stage2_mellinger_poly(xd_nominal, dir_nom, spd, total_dist, pL_cur, vL_cur, W_opt);
                    
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
                        fprintf("[HIERARCHICAL MPC-POLY] 障害物クリア・C^6シームレス合流 (t=%.3f s)\n\n", time.t);
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
        function W_opt = plan_stage1_mpc_waypoints(obj, p0, dir_nom, spd, total_dist)
            % 粗い離散ウェイポイント数
            N_way = 5; 
            T = obj.t_duration;
            
            W0 = zeros(3, N_way);
            for i = 1:N_way
                u_i = i / (N_way + 1);
                W0(:, i) = p0 + dir_nom * (total_dist * u_i);
                % 勾配消失を防ぐ特異点ノイズ
                W0(1, i) = W0(1, i) + 1e-3;
            end
            
            fun = @(w) obj.mpc_cost_function(w, p0, dir_nom, total_dist, N_way);
            
            opts = optimoptions('fminunc', 'Algorithm', 'quasi-newton', ...
                'Display', 'off', 'OptimalityTolerance', 1e-3);
            
            [w_opt, ~] = fminunc(fun, reshape(W0, [], 1), opts);
            W_opt = reshape(w_opt, 3, N_way);
            
            fprintf("    - MPC ガイドポイント生成完了 (中心点: [%.2f, %.2f, %.2f])\n", W_opt(:, 3));
        end
        
        function J = mpc_cost_function(obj, w, p0, dir_nom, total_dist, N_way)
            W = reshape(w, 3, N_way);
            J_track = 0;
            J_obs = 0;
            J_smooth = 0;
            
            R_clear = obj.obs_radius + obj.safe_margin + obj.L_cable * 0.5 + 0.3;
            p_end = p0 + dir_nom * total_dist;
            
            W_full = [p0, W, p_end];
            
            for i = 1:N_way
                u_i = i / (N_way + 1);
                p_ref = p0 + dir_nom * (total_dist * u_i);
                
                % 追従コスト
                J_track = J_track + norm(W(:, i) - p_ref)^2;
                
                % 障害物ポテンシャル (SLQ-MPC風斥力)
                dist_sq = norm(W(:, i) - obj.obs_center)^2;
                dist = sqrt(dist_sq + 1e-4);
                if dist < R_clear + 1.0
                    J_obs = J_obs + 100.0 * (1.0 / (dist) - 1.0 / (R_clear + 1.0))^2;
                end
            end
            
            for i = 3:N_way+2
                J_smooth = J_smooth + norm(W_full(:, i) - 2*W_full(:, i-1) + W_full(:, i-2))^2;
            end
            
            J = 1.0 * J_track + 1.0 * J_obs + 10.0 * J_smooth;
        end
        
        function plan_stage2_mellinger_poly(obj, xd0, dir_nom, spd, total_dist, pL_cur, vL_cur, W_opt)
            T = obj.t_duration;
            p0 = pL_cur;
            p_end = pL_cur + dir_nom * total_dist;
            
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
            w_way = 1e4; % ソフト制約重み
            
            for m = 1:N_way
                u_m = m / (N_way + 1);
                u_vec = zeros(1, n_coeffs);
                for n = 0:order
                    u_vec(n+1) = u_m^n;
                end
                
                for axis = 1:3
                    idx_start = (axis-1)*n_coeffs + 1;
                    idx_end   = axis*n_coeffs;
                    
                    % (u_vec * C - W_opt)^2 を H, f に加算
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
            
            xd0_real = xd0;
            xd0_real(1:3) = pL_cur;
            xd0_real(5:7) = vL_cur;
            
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
            
            % H は正定値、Aeq の等式制約付き最適化
            % C_opt = quadprog(H, f, [], [], Aeq, beq);
            % 行列反転で解析的に解く (KKTシステム)
            KKT_matrix = [H, Aeq'; Aeq, zeros(size(Aeq,1))];
            KKT_rhs = [-f; beq];
            sol = KKT_matrix \ KKT_rhs;
            C_opt = sol(1:num_vars);
            
            obj.poly_coeffs = [C_opt(1:n_coeffs), C_opt(n_coeffs+1 : 2*n_coeffs), C_opt(2*n_coeffs+1 : end)];
            fprintf("    - Mellinger 多項式 QP 解析解 算出完了\n");
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

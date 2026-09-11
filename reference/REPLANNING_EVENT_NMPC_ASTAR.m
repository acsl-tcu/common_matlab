classdef REPLANNING_EVENT_NMPC_ASTAR < handle
    % REPLANNING_EVENT_NMPC_ASTAR
    % 案4: イベント駆動型NMPCと大域A*の結合 (Tasooji 2025 + URAI 2017)
    % 障害物接近の「イベント」発火時のみ、A*ライクな空間探索で大域的な初期経路を見つけ、
    % それをウォームスタートとして非線形MPC(障害物ポテンシャル)で局所最適化。
    % 最後にMellinger13次多項式で6階微分まで平滑化フィッティングする統合アーキテクチャ。
    
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
        function obj = REPLANNING_EVENT_NMPC_ASTAR(self, base_ref, opts)
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
            
            % イベント駆動: 障害物接近時に1回だけ発火
            if cha == 'f' && ~obj.replan_done && ~obj.replan_active
                dist_load_obs  = norm(pL_cur - obj.obs_center);
                dist_drone_obs = norm(pQ_cur - obj.obs_center);
                min_dist = min(dist_load_obs, dist_drone_obs);
                
                if min_dist <= obj.trigger_dist
                    fprintf("\n=======================================================\n");
                    fprintf("[EVENT NMPC-A*] 障害物検知イベント発火! (min_dist=%.2fm, t=%.3f s)\n", min_dist, time.t);
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
                    
                    fprintf("[EVENT NMPC-A*] 第1段階: A* 大域探索による初期ウェイポイント生成...\n");
                    W_astar = obj.plan_stage1_astar(p0_ref, dir_nom, total_dist);
                    
                    fprintf("[EVENT NMPC-A*] 第2段階: 局所非線形MPC (障害物ポテンシャル) による最適化...\n");
                    W_nmpc = obj.plan_stage2_nmpc(W_astar, p0_ref, dir_nom, total_dist);
                    
                    fprintf("[EVENT NMPC-A*] 第3段階: 13次多項式 Mellinger 平滑化フィッティング...\n");
                    obj.plan_stage3_mellinger_poly(xd_nominal, dir_nom, spd, total_dist, p0_ref, v0_ref, W_nmpc);
                    
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
                        fprintf("[EVENT NMPC-A*] 障害物クリア・C^6シームレス合流 (t=%.3f s)\n\n", time.t);
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
        function W_astar = plan_stage1_astar(obj, p0, dir_nom, total_dist)
            % 粗い A* 空間探索
            N_steps = 5;
            step_len = total_dist / (N_steps + 1);
            p_end = p0 + dir_nom * total_dist;
            
            R_clear = obj.obs_radius + obj.safe_margin + obj.L_cable * 0.5 + 0.3;
            
            vec_to_obs = obj.obs_center - p0;
            proj_on_line = dot(vec_to_obs, dir_nom) * dir_nom;
            normal_vec = vec_to_obs - proj_on_line;
            if norm(normal_vec) < 1e-3
                if abs(dir_nom(3)) < 0.9, normal_vec = cross(dir_nom, [0; 0; 1]);
                else, normal_vec = cross(dir_nom, [1; 0; 0]); end
            end
            dir_normal = normal_vec / norm(normal_vec);
            dir_side = cross(dir_nom, dir_normal);
            
            % 動的計画法 / A* の貪欲展開
            W_astar = zeros(3, N_steps);
            current_p = p0;
            
            % 横方向の探索アクション
            actions = [0; 0.5; -0.5; 1.0; -1.0; 1.5; -1.5];
            
            for k = 1:N_steps
                best_cost = inf;
                best_p = current_p;
                
                for a = 1:length(actions)
                    % 進行方向に1ステップ進み、法線面内で横にずれる
                    p_cand = current_p + dir_nom * step_len + dir_normal * actions(a) * R_clear;
                    
                    % 衝突チェック
                    dist_obs = norm(p_cand - obj.obs_center);
                    if dist_obs < R_clear
                        continue;
                    end
                    
                    % コスト: ゴールへの距離 + 直進からのズレ
                    cost_g = norm(actions(a)) * 2.0; 
                    cost_h = norm(p_end - p_cand);
                    cost_total = cost_g + cost_h;
                    
                    if cost_total < best_cost
                        best_cost = cost_total;
                        best_p = p_cand;
                    end
                end
                
                % もし全スタックしたら強制押し出し
                if isinf(best_cost)
                    best_p = current_p + dir_nom * step_len + dir_normal * (R_clear + 0.2);
                end
                
                W_astar(:, k) = best_p;
                current_p = best_p;
            end
        end
        
        function W_nmpc = plan_stage2_nmpc(obj, W_astar, p0, dir_nom, total_dist)
            N_way = size(W_astar, 2);
            fun = @(w) obj.nmpc_cost_function(w, W_astar, p0, dir_nom, total_dist, N_way);
            
            % ウォームスタート (A* の結果を初期値にするため一瞬で収束する)
            w0 = reshape(W_astar, [], 1);
            opts = optimoptions('fminunc', 'Algorithm', 'quasi-newton', ...
                'Display', 'off', 'OptimalityTolerance', 1e-3, 'MaxIterations', 50);
            
            [w_opt, ~] = fminunc(fun, w0, opts);
            W_nmpc = reshape(w_opt, 3, N_way);
        end
        
        function J = nmpc_cost_function(obj, w, W_astar, p0, dir_nom, total_dist, N_way)
            W = reshape(w, 3, N_way);
            J_track = 0;
            J_smooth = 0;
            J_obs = 0;
            
            R_clear = obj.obs_radius + obj.safe_margin + obj.L_cable * 0.5 + 0.1;
            p_end = p0 + dir_nom * total_dist;
            
            W_full = [p0, W, p_end];
            
            for i = 1:N_way
                % A* の大域ルートへの追従（局所解への落下防止）
                J_track = J_track + norm(W(:, i) - W_astar(:, i))^2;
                
                % 障害物斥力ポテンシャル
                dist_sq = norm(W(:, i) - obj.obs_center)^2;
                dist = sqrt(dist_sq + 1e-4);
                if dist < R_clear + 1.5
                    % 侵入ペナルティ
                    J_obs = J_obs + 100.0 * max(0, 1.0/dist - 1.0/(R_clear + 1.5))^2;
                end
            end
            
            for i = 2:N_way+1
                % 曲率(差分)ペナルティ
                J_smooth = J_smooth + norm(W_full(:, i) - 2*W_full(:, i-1) + W_full(:, i-2))^2;
            end
            
            J = 5.0 * J_track + 20.0 * J_obs + 10.0 * J_smooth;
        end
        
        function plan_stage3_mellinger_poly(obj, xd_nominal, dir_nom, spd, total_dist, p0_ref, v0_ref, W_opt)
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

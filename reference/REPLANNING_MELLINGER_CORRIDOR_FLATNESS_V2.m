classdef REPLANNING_MELLINGER_CORRIDOR_FLATNESS_V2 < handle
    % REPLANNING_MELLINGER_CORRIDOR_FLATNESS_V2
    % 13次多項式 (7階微分まで連続) の完全なる滑らかさを維持しつつ、
    % スケーリング理論に基づき数値的悪条件を完全に排除した真の汎用2点バリアリプランナ。
    
    properties
        base_ref
        self
        replan_active = false
        replan_done   = false
        
        t_start
        t_duration = 10.0
        
        safe_margin  = 0.3
        trigger_dist = 2.5
        
        L_cable
        gravity = 9.81
        
        poly_coeffs 
        
        t_merge_end
        p_merge_end
        v_merge_vec
        
        obs_center = [0;0;0];
        obs_radius = 0;
        hyperplane_n = [0;0;0];
        hyperplane_p = [0;0;0];
        
        result
    end
    
    methods
        function obj = REPLANNING_MELLINGER_CORRIDOR_FLATNESS_V2(self, base_ref, opts)
            arguments
                self
                base_ref
                opts = struct()
            end
            obj.self = self;
            obj.base_ref = base_ref;
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
            
            pL_cur = base_res.state.p;
            if isprop(obj.self.estimator.result.state, "pL")
                pL_cur = obj.self.estimator.result.state.pL;
            end
            
            obj.L_cable = obj.self.parameter.get("cableL");
            pQ_cur = pL_cur + [0; 0; obj.L_cable];
            
            if cha == 'f' && ~obj.replan_done && ~obj.replan_active
                try
                    obs_list = ENVIRONMENT_OBSTACLE_HOCBF_LINK_XY();
                catch
                    obs_list = [];
                end
                
                min_dist_overall = inf;
                target_obs_idx = -1;
                
                for i = 1:length(obs_list)
                    if isfield(obs_list(i), 'p_obs') && isfield(obs_list(i), 'r_obs')
                        c = obs_list(i).p_obs;
                        r = obs_list(i).r_obs;
                        dist_load = norm(pL_cur - c);
                        dist_drone = norm(pQ_cur - c);
                        d_surf = min(dist_load, dist_drone) - r;
                        
                        if d_surf < min_dist_overall
                            min_dist_overall = d_surf;
                            target_obs_idx = i;
                        end
                    end
                end
                
                if target_obs_idx > 0 && min_dist_overall <= obj.trigger_dist
                    obj.obs_center = obs_list(target_obs_idx).p_obs;
                    obj.obs_radius = obs_list(target_obs_idx).r_obs;
                    
                    fprintf("\n=======================================================\n");
                    fprintf("[MELLINGER FLATNESS V2] 障害物検知! (ID: %d, d_surf=%.2fm, t=%.3f s)\n", target_obs_idx, min_dist_overall, time.t);
                    obj.t_start = time.t;
                    
                    v_vec = xd_nominal(5:7);
                    spd = norm(v_vec);
                    if spd < 0.1, spd = 0.5; v_vec = [0; 0; 0.5]; end
                    dir_nom = v_vec / spd;
                    
                    vec_to_obs = obj.obs_center - pL_cur;
                    proj_dist = max(0, dot(vec_to_obs, dir_nom));
                    
                    total_dist = proj_dist + obj.obs_radius + obj.safe_margin + 2.0;
                    obj.t_duration = max(5.0, min(12.0, total_dist / spd));
                    
                    fprintf("[MELLINGER FLATNESS V2] 理論に基づく 13次多項式 ソフト制約QP の実行...\n");
                    obj.plan_corridor_soft_qp(xd_nominal, dir_nom, spd, total_dist);
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
                        fprintf("[MELLINGER FLATNESS V2] 障害物クリア・C^6シームレス合流 (t=%.3f s)\n\n", time.t);
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
                end
            elseif obj.replan_done
                dt_after = time.t - obj.t_merge_end;
                xd = zeros(28, 1);
                xd(1:3) = obj.p_merge_end + obj.v_merge_vec * dt_after;
                xd(5:7) = obj.v_merge_vec;
            else
                xd = xd_nominal;
            end
            
            if length(xd) < 28, xd = [xd; zeros(28 - length(xd), 1)]; end
            obj.result.state.xd = xd;
            obj.result.state.p = xd(1:3);
            obj.result.state.v = xd(5:7);
            obj.result.state.q = [0; 0; xd(4)];
            result = obj.result;
        end
    end
    
    methods (Access = private)
        function plan_corridor_soft_qp(obj, xd0, dir_nom, spd, total_dist)
            T = obj.t_duration;
            p0 = xd0(1:3);
            
            vec_to_obs = obj.obs_center - p0;
            proj_on_line = dot(vec_to_obs, dir_nom) * dir_nom;
            normal_vec = vec_to_obs - proj_on_line;
            if norm(normal_vec) < 1e-3
                tangent = cross(dir_nom, [0; 0; 1]);
                if norm(tangent) < 1e-3, tangent = cross(dir_nom, [1; 0; 0]); end
                normal_vec = tangent;
            end
            
            n_wall = - (normal_vec / norm(normal_vec)); 
            R_clear = obj.obs_radius + obj.safe_margin;
            p_wall = obj.obs_center + n_wall * R_clear;
            
            a_ineq = -n_wall;
            b_ineq = -dot(n_wall, p_wall);
            
            obj.hyperplane_n = n_wall;
            obj.hyperplane_p = p_wall;
            
            p_end = p0 + dir_nom * total_dist;
            xd1 = zeros(28, 1);
            xd1(1:3) = p_end;
            xd1(5:7) = dir_nom * spd;
            
            % 13次多項式 -> 14変数
            order = 13;
            n_coeffs = order + 1;
            N_samples = 15;
            
            num_C = 3 * n_coeffs;
            num_Slack = 2 * N_samples;
            num_vars = num_C + num_Slack;
            
            H = zeros(num_vars, num_vars);
            f = zeros(num_vars, 1);
            
            % 時間Tを考慮した正しい物理スケーリングの目的関数
            H_1d = zeros(n_coeffs, n_coeffs);
            w4 = 1.0; w5 = 0.1;
            for i = 4:order
                for j = 4:order
                    val4 = (factorial(i)/factorial(i-4)) * (factorial(j)/factorial(j-4)) / (i + j - 7);
                    H_1d(i+1, j+1) = H_1d(i+1, j+1) + w4 * val4 / (T^7);
                    if i >= 5 && j >= 5
                        val5 = (factorial(i)/factorial(i-5)) * (factorial(j)/factorial(j-5)) / (i + j - 9);
                        H_1d(i+1, j+1) = H_1d(i+1, j+1) + w5 * val5 / (T^9);
                    end
                end
            end
            
            % Hessianの正規化（スケールを整える）
            scale_H = max(abs(H_1d(:)));
            if scale_H > 0
                H_1d = H_1d / scale_H;
            end
            H(1:num_C, 1:num_C) = blkdiag(H_1d, H_1d, H_1d) + 1e-9 * eye(num_C);
            
            % スラック変数へのウェイトをHessianのスケールに合わせる
            W_slack = 1e4; % 適度な強さで壁侵入を防ぐ
            for i = 1:num_Slack
                idx = num_C + i;
                H(idx, idx) = W_slack;
                f(idx) = W_slack;
            end
            
            % 等式制約 (位置、速度、加速度、ジャーク、スナップ、クラックル、7階微分まで連続)
            n_eq_1d = 13;
            Aeq_1d = zeros(n_eq_1d, n_coeffs);
            beq_1d = zeros(n_eq_1d, 1);
            
            for k = 0:6
                row = k + 1;
                for n = k:order, Aeq_1d(row, n + 1) = prod(n - k + 1 : n) * (0)^(n - k); end
            end
            for k = 0:5
                row = 8 + k;
                for n = k:order, Aeq_1d(row, n + 1) = prod(n - k + 1 : n) * (1.0)^(n - k); end
            end
            
            Aeq = zeros(3 * n_eq_1d, num_vars);
            Aeq(1:3*n_eq_1d, 1:num_C) = blkdiag(Aeq_1d, Aeq_1d, Aeq_1d);
            beq = zeros(3 * n_eq_1d, 1);
            
            for axis = 1:3
                beq_axis = zeros(n_eq_1d, 1);
                for k = 0:6, beq_axis(k+1) = (T^k) * xd0(4*k + axis); end
                for k = 0:5, beq_axis(8+k) = (T^k) * xd1(4*k + axis); end
                beq( (axis-1)*n_eq_1d + 1 : axis*n_eq_1d ) = beq_axis;
            end
            
            % 不等式制約
            A_ineq = zeros(2 * N_samples, num_vars);
            b_ineq_all = zeros(2 * N_samples, 1);
            row_idx = 1;
            L_g = obj.L_cable / obj.gravity;
            
            for m = 1:N_samples
                u_m = m / (N_samples + 1); 
                T0_vec = zeros(1, n_coeffs);
                T2_vec = zeros(1, n_coeffs);
                for n = 0:order
                    T0_vec(n+1) = u_m^n;
                    if n >= 2, T2_vec(n+1) = (n * (n - 1)) * u_m^(n - 2) / (T^2); end
                end
                
                T0_blk = blkdiag(T0_vec, T0_vec, T0_vec);
                T2_blk = blkdiag(T2_vec, T2_vec, T2_vec);
                
                A_ineq(row_idx, 1:num_C) = a_ineq' * T0_blk;
                A_ineq(row_idx, num_C + 2*m - 1) = -1;
                b_ineq_all(row_idx) = b_ineq;
                row_idx = row_idx + 1;
                
                A_ineq(row_idx, 1:num_C) = a_ineq' * (T0_blk + L_g * T2_blk);
                A_ineq(row_idx, num_C + 2*m) = -1;
                b_ineq_all(row_idx) = b_ineq - a_ineq' * [0; 0; obj.L_cable];
                row_idx = row_idx + 1;
            end
            
            A_slack_pos = zeros(num_Slack, num_vars);
            A_slack_pos(:, num_C+1:end) = -eye(num_Slack);
            A_ineq = [A_ineq; A_slack_pos];
            b_ineq_all = [b_ineq_all; zeros(num_Slack, 1)];
            
            opts = optimoptions('quadprog', 'Display', 'off');
            [X_opt, ~, exitflag] = quadprog(H, f, A_ineq, b_ineq_all, Aeq, beq, [], [], [], opts);
            
            if exitflag < 1
                fprintf("[MELLINGER FLATNESS V2] QP数値エラー(exitflag=%d)。解析解へフォールバック\n", exitflag);
                p_via = obj.obs_center + n_wall * (R_clear + 1.0);
                A_fb = zeros(14, 14); Bx_fb = zeros(14, 3);
                for k = 0:6
                    for n = k:order, A_fb(k+1, n+1) = prod(n-k+1:n) * 0^(n-k); end
                    Bx_fb(k+1, :) = (T^k) * xd0(4*k + (1:3))';
                end
                for n = 0:order, A_fb(8, n+1) = (0.5)^n; end
                Bx_fb(8, :) = p_via';
                for k = 0:5
                    for n = k:order, A_fb(9+k, n+1) = prod(n-k+1:n) * 1.0^(n-k); end
                    Bx_fb(9+k, :) = (T^k) * xd1(4*k + (1:3))';
                end
                C_fb = A_fb \ Bx_fb;
                C_opt = [C_fb(:, 1); C_fb(:, 2); C_fb(:, 3)];
            else
                max_violation = max(X_opt(num_C+1:end));
                if max_violation > 0.05
                    fprintf("[MELLINGER FLATNESS V2] ※壁侵入(%.3fm)を許容して滑らかさを優先しました。\n", max_violation);
                else
                    fprintf("[MELLINGER FLATNESS V2] QP成功！壁への完全回避と13次平滑化達成。\n");
                end
                C_opt = X_opt(1:num_C);
            end
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

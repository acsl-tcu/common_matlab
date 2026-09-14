classdef REPLANNING_MELLINGER_CORRIDOR_FLATNESS_SWING < handle
    % REPLANNING_MELLINGER_CORRIDOR_FLATNESS_SWING
    % 任意姿勢楕円体 (Ellipsoid) 障害物対応 Mellinger Corridor QP リプランナ
    
    properties
        base_ref
        self
        replan_active = false
        replan_done   = false
        
        t_start
        t_duration = 8.0
        
        safe_margin  = 0.5
        trigger_dist = 5.0
        
        L_cable      = 2.0
        gravity      = 9.81
        
        r_load       = 0.15
        r_drone      = 0.30
        
        poly_coeffs
        
        t_merge_end
        p_merge_end
        v_merge_vec
        
        obs_center = [0; 0; 0]
        obs_R      = eye(3)
        obs_radii  = [1; 1; 1]
        obs_margin = 0
        
        debug_cnt  = 0 % 診断ログ用カウンタ
        result
    end
    
    methods
        function obj = REPLANNING_MELLINGER_CORRIDOR_FLATNESS_SWING(self, base_ref, opts)
            arguments
                self
                base_ref
                opts = struct()
            end
            obj.self = self;
            obj.base_ref = base_ref;
            if isfield(opts, "safe_margin"),  obj.safe_margin  = opts.safe_margin;  end
            if isfield(opts, "trigger_dist"), obj.trigger_dist = opts.trigger_dist; end
            if isfield(opts, "r_load"),       obj.r_load       = opts.r_load;       end
            if isfield(opts, "r_drone"),      obj.r_drone      = opts.r_drone;      end
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
            
            if isprop(obj.self.estimator.result.state, "p")
                pQ_cur = obj.self.estimator.result.state.p;
            else
                pQ_cur = pL_cur + [0; 0; obj.L_cable];
            end
            
            % -------------------------------------------------------------
            % 1. 動的障害物検知 (デバッグ診断機能付き)
            % -------------------------------------------------------------
            if cha == 'f' && ~obj.replan_done && ~obj.replan_active
                obs_list = [];
                try
                    obs_list = ENVIRONMENT_OBSTACLE_ELLIPSE();
                catch ME
                    if obj.debug_cnt == 0
                        fprintf("[DEBUG ERROR] ENVIRONMENT_OBSTACLE_ELLIPSEの呼び出しに失敗: %s\n", ME.message);
                    end
                end
                
                min_dist_overall = inf;
                target_obs_idx = -1;
                
                for i = 1:length(obs_list)
                    % p_center と p_obs の両方を許容
                    if isfield(obs_list(i), 'p_center') && ~isempty(obs_list(i).p_center)
                        c = obs_list(i).p_center;
                    elseif isfield(obs_list(i), 'p_obs') && ~isempty(obs_list(i).p_obs)
                        c = obs_list(i).p_obs;
                    else
                        continue;
                    end
                    
                    % 楕円半径 or スカラー半径
                    if isfield(obs_list(i), 'ellipsoid_radii') && ~isempty(obs_list(i).ellipsoid_radii)
                        r_max = max(obs_list(i).ellipsoid_radii);
                    elseif isfield(obs_list(i), 'r_obs') && ~isempty(obs_list(i).r_obs)
                        r_max = obs_list(i).r_obs;
                    elseif isfield(obs_list(i), 'Q_obs') && ~isempty(obs_list(i).Q_obs)
                        % Q = diag(1/a^2, 1/b^2, 1/c^2) から復元
                        r_max = max(1 ./ sqrt(diag(obs_list(i).Q_obs)));
                    else
                        r_max = 0.5;
                    end
                    
                    if isfield(obs_list(i), 'd_margin')
                        d_marg = obs_list(i).d_margin;
                    else
                        d_marg = obj.safe_margin;
                    end
                    
                    dist_load  = norm(pL_cur - c) - (r_max + obj.r_load + d_marg);
                    dist_drone = norm(pQ_cur - c) - (r_max + obj.r_drone + d_marg);
                    d_surf = min(dist_load, dist_drone);
                    
                    if d_surf < min_dist_overall
                        min_dist_overall = d_surf;
                        target_obs_idx = i;
                    end
                end
                
                % 1秒に1回、現在の検知状態をコンソールへ表示（診断ログ）
                obj.debug_cnt = obj.debug_cnt + 1;
                if mod(obj.debug_cnt, 40) == 1
                    if isempty(obs_list)
                        fprintf("[DIAGNOSTIC] t=%.2f: 障害物リストが空です (ENVIRONMENT_OBSTACLE_ELLIPSE未読込)\n", time.t);
                    else
                        fprintf("[DIAGNOSTIC] t=%.2f: 最接近障害物 ID=%d, d_surf=%.2fm (トリガー閾値: %.2fm)\n", ...
                            time.t, target_obs_idx, min_dist_overall, obj.trigger_dist);
                    end
                end
                
                % トリガー判定
                if target_obs_idx > 0 && min_dist_overall <= obj.trigger_dist
                    tgt = obs_list(target_obs_idx);
                    
                    % 中心座標
                    if isfield(tgt, 'p_center'), obj.obs_center = tgt.p_center;
                    else, obj.obs_center = tgt.p_obs; end
                    
                    % 姿勢
                    if isfield(tgt, 'R_obs'), obj.obs_R = tgt.R_obs;
                    else, obj.obs_R = eye(3); end
                    
                    % 楕円主軸長
                    if isfield(tgt, 'ellipsoid_radii')
                        obj.obs_radii = tgt.ellipsoid_radii;
                    elseif isfield(tgt, 'Q_obs')
                        obj.obs_radii = 1 ./ sqrt(diag(tgt.Q_obs));
                    elseif isfield(tgt, 'r_obs')
                        obj.obs_radii = [tgt.r_obs; tgt.r_obs; tgt.r_obs];
                    else
                        obj.obs_radii = [0.71; 0.71; 0.71];
                    end
                    
                    if isfield(tgt, 'd_margin'), obj.obs_margin = tgt.d_margin;
                    else, obj.obs_margin = obj.safe_margin; end
                    
                    obj.t_start = time.t;
                    
                    v_vec = xd_nominal(5:7);
                    spd = norm(v_vec);
                    if spd < 0.05, spd = norm(vL_cur); end
                    if spd < 0.05, spd = 0.5; v_vec = [0; 0; 0.5]; end
                    dir_nom = v_vec / spd;
                    
                    vec_to_obs = obj.obs_center - pL_cur;
                    d_proj = dot(vec_to_obs, dir_nom);
                    if d_proj < 0.5, d_proj = 0.5; end
                    
                    t_cross = d_proj / spd;
                    obj.t_duration = max(5.0, 2.0 * t_cross);
                    total_dist = spd * obj.t_duration;
                    
                    fprintf("\n=======================================================\n");
                    fprintf("[ELLIPSE FLATNESS QP] 検知成功・リプラン開始! (ID: %d, d_surf=%.2fm, t=%.3f s)\n", target_obs_idx, min_dist_overall, time.t);
                    fprintf("  - 進行速度: %.2f m/s, 射影距離: %.2f m, 計画時間 T = %.2f s\n", spd, d_proj, obj.t_duration);
                    fprintf("  - 楕円中心: [%.2f, %.2f, %.2f]\n", obj.obs_center(1), obj.obs_center(2), obj.obs_center(3));
                    fprintf("  - 楕円主軸半径: [a=%.2f, b=%.2f, c=%.2f] m\n", obj.obs_radii(1), obj.obs_radii(2), obj.obs_radii(3));
                    fprintf("  - 荷物半径: %.2f m / ドローン保護半径: %.2f m / マージン帯: %.2f m\n", obj.r_load, obj.r_drone, obj.obs_margin);
                    
                    obj.plan_mellinger_mahalanobis_qp(xd_nominal, dir_nom, spd, total_dist);
                    obj.replan_active = true;
                    fprintf("=======================================================\n\n");
                end
            end
            
            % 2. 軌道出力 ＆ C^6シームレス合流
            if obj.replan_active
                tau = time.t - obj.t_start;
                if tau <= obj.t_duration
                    xd = obj.evaluate_smooth_trajectory(tau, xd_nominal);
                else
                    if ~obj.replan_done
                        fprintf("[ELLIPSE FLATNESS QP] 障害物通過完了・C^6シームレス合流 (t=%.3f s)\n\n", time.t);
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
            
            if length(xd) < 28, xd = [xd; zeros(28 - length(xd), 1)]; end
            obj.result.state.xd = xd;
            obj.result.state.p = xd(1:3);
            obj.result.state.v = xd(5:7);
            obj.result.state.q = [0; 0; xd(4)];
            result = obj.result;
        end
    end
    
    methods (Access = private)
        function plan_mellinger_mahalanobis_qp(obj, xd0, dir_nom, spd, total_dist)
            T = obj.t_duration;
            p0 = xd0(1:3);
            order = 13;
            n_coeffs = order + 1;
            
            % 1. マハラノビス空間での最適接平面
            D_inv = diag(1 ./ obj.obs_radii);
            z0 = D_inv * obj.obs_R' * (p0 - obj.obs_center);
            vz = D_inv * obj.obs_R' * dir_nom;
            
            tau_star = -dot(z0, vz) / max(1e-6, dot(vz, vz));
            z_closest = z0 + tau_star * vz;
            
            if norm(z_closest) < 1e-4
                if abs(vz(3)) < 0.9, z_perp = cross(vz, [0; 0; 1]);
                else, z_perp = cross(vz, [1; 0; 0]); end
                z_hat = z_perp / norm(z_perp);
            else
                z_hat = z_closest / norm(z_closest);
            end
            
            n_opt = obj.obs_R * D_inv * z_hat;
            norm_n_opt = norm(n_opt);
            dir_normal = n_opt / norm_n_opt;
            r_eff_obs  = 1.0 / norm_n_opt;
            
            R_hard_load  = r_eff_obs + obj.r_load;
            R_hard_drone = r_eff_obs + obj.r_drone;
            
            r_eff_long = sqrt(dot(dir_nom, obj.obs_R * diag(obj.obs_radii.^2) * obj.obs_R' * dir_nom));
            t_span = (r_eff_long + obj.obs_margin) / spd;
            delta_u = max(0.12, min(0.35, t_span / T));
            
            u_start = max(0.15, 0.50 - delta_u * 1.3);
            u_end   = min(0.85, 0.50 + delta_u * 1.3);
            
            N_samples = 15;
            u_samples = linspace(u_start, u_end, N_samples);
            num_C = 3 * n_coeffs;
            num_vars = num_C + N_samples;
            
            % 2. 目的関数
            H_1d = zeros(n_coeffs, n_coeffs);
            w2 = 0.05; w4 = 1.0; w5 = 0.1;
            for i = 2:order
                for j = 2:order
                    val2 = (factorial(i)/factorial(i-2)) * (factorial(j)/factorial(j-2)) / (i + j - 3);
                    H_1d(i+1, j+1) = H_1d(i+1, j+1) + w2 * val2;
                    if i >= 4 && j >= 4
                        val4 = (factorial(i)/factorial(i-4)) * (factorial(j)/factorial(j-4)) / (i + j - 7);
                        H_1d(i+1, j+1) = H_1d(i+1, j+1) + w4 * val4;
                    end
                    if i >= 5 && j >= 5
                        val5 = (factorial(i)/factorial(i-5)) * (factorial(j)/factorial(j-5)) / (i + j - 9);
                        H_1d(i+1, j+1) = H_1d(i+1, j+1) + w5 * val5;
                    end
                end
            end
            
            norm_H = norm(H_1d, 2);
            H_1d_scaled = H_1d / norm_H;
            
            H = zeros(num_vars, num_vars);
            H(1:num_C, 1:num_C) = blkdiag(H_1d_scaled, H_1d_scaled, H_1d_scaled) + 1e-6 * eye(num_C);
            
            w_slack_quad = 1000000.0;
            w_slack_lin  = 100000.0;
            for s_idx = 1:N_samples
                H(num_C + s_idx, num_C + s_idx) = w_slack_quad;
            end
            f = zeros(num_vars, 1);
            f(num_C + 1 : end) = w_slack_lin;
            
            % 3. 等式制約
            p_end = p0 + dir_nom * total_dist;
            xd1 = zeros(28, 1);
            xd1(1:3) = p_end;
            xd1(5:7) = dir_nom * spd;
            xd1(9:28) = 0;
            
            n_eq = 13;
            Aeq_1d = zeros(n_eq, n_coeffs);
            for k = 0:6
                for n = k:order, Aeq_1d(k+1, n+1) = prod(n-k+1:n) * 0^(n-k); end
            end
            for k = 0:5
                for n = k:order, Aeq_1d(8+k, n+1) = prod(n-k+1:n) * 1.0^(n-k); end
            end
            
            Aeq = zeros(3 * n_eq, num_vars);
            Aeq(:, 1:num_C) = blkdiag(Aeq_1d, Aeq_1d, Aeq_1d);
            beq = zeros(3 * n_eq, 1);
            
            for ax = 1:3
                beq_ax = zeros(n_eq, 1);
                for k = 0:6, beq_ax(k+1) = (T^k) * xd0(4*k + ax); end
                for k = 0:5, beq_ax(8+k) = (T^k) * xd1(4*k + ax); end
                beq((ax-1)*n_eq + 1 : ax*n_eq) = beq_ax;
            end
            
            % 4. 不等式制約
            alpha_dyn = obj.L_cable / (obj.gravity * (T^2));
            p_base_proj = dot(dir_normal, obj.obs_center);
            
            hard_mask = (u_samples >= (0.50 - delta_u)) & (u_samples <= (0.50 + delta_u));
            n_hard = sum(hard_mask);
            
            num_ineq = n_hard + n_hard + N_samples + N_samples;
            A_ineq = zeros(num_ineq, num_vars);
            b_ineq = zeros(num_ineq, 1);
            
            row = 1;
            for m = find(hard_mask)
                u_m = u_samples(m);
                T0 = zeros(1, n_coeffs);
                for n = 0:order, T0(n+1) = u_m^n; end
                
                A_ineq(row, 1:n_coeffs)               = -dir_normal(1) * T0;
                A_ineq(row, n_coeffs+1 : 2*n_coeffs)   = -dir_normal(2) * T0;
                A_ineq(row, 2*n_coeffs+1 : 3*n_coeffs) = -dir_normal(3) * T0;
                b_ineq(row) = -(p_base_proj + R_hard_load);
                row = row + 1;
            end
            
            for m = find(hard_mask)
                u_m = u_samples(m);
                T0 = zeros(1, n_coeffs);
                T2 = zeros(1, n_coeffs);
                for n = 0:order, T0(n+1) = u_m^n; end
                for n = 2:order, T2(n+1) = n * (n - 1) * (u_m^(n - 2)); end
                
                TQ = T0 + alpha_dyn * T2;
                
                A_ineq(row, 1:n_coeffs)               = -dir_normal(1) * TQ;
                A_ineq(row, n_coeffs+1 : 2*n_coeffs)   = -dir_normal(2) * TQ;
                A_ineq(row, 2*n_coeffs+1 : 3*n_coeffs) = -dir_normal(3) * TQ;
                
                z_offset_proj = dir_normal(3) * obj.L_cable;
                b_ineq(row) = -(p_base_proj + R_hard_drone - z_offset_proj);
                row = row + 1;
            end
            
            for m = 1:N_samples
                u_m = u_samples(m);
                T0 = zeros(1, n_coeffs);
                for n = 0:order, T0(n+1) = u_m^n; end
                
                phase = (u_m - u_start) / (u_end - u_start);
                envelope = sin(pi * max(0, min(1.0, phase)));
                R_soft_local = R_hard_load + obj.obs_margin * envelope;
                
                A_ineq(row, 1:n_coeffs)               = -dir_normal(1) * T0;
                A_ineq(row, n_coeffs+1 : 2*n_coeffs)   = -dir_normal(2) * T0;
                A_ineq(row, 2*n_coeffs+1 : 3*n_coeffs) = -dir_normal(3) * T0;
                A_ineq(row, num_C + m)                = -1;
                b_ineq(row) = -(p_base_proj + R_soft_local);
                row = row + 1;
            end
            
            for m = 1:N_samples
                A_ineq(row, num_C + m) = -1;
                b_ineq(row) = 0;
                row = row + 1;
            end
            
            % 5. 最適化実行
            opts = optimoptions('quadprog', ...
                'Display', 'off', ...
                'Algorithm', 'interior-point-convex', ...
                'MaxIterations', 200, ...
                'ConstraintTolerance', 1e-4, ...
                'OptimalityTolerance', 1e-4, ...
                'StepTolerance', 1e-8);
            
            [X_opt, ~, exitflag, output] = quadprog(H, f, A_ineq, b_ineq, Aeq, beq, [], [], [], opts);
            
            if exitflag < 1
                fprintf("[RIGID ELLIPSOID QP DEBUG] quadprog未収束 (exitflag=%d: %s)\n", exitflag, output.message);
                C_1 = Aeq_1d \ beq(1:n_eq);
                C_2 = Aeq_1d \ beq(n_eq+1:2*n_eq);
                C_3 = Aeq_1d \ beq(2*n_eq+1:3*n_eq);
                obj.poly_coeffs = [C_1, C_2, C_3]';
            else
                slack_vals = X_opt(num_C + 1 : end);
                center_mask = (u_samples >= (0.50 - delta_u/2)) & (u_samples <= (0.50 + delta_u/2));
                max_slack_center = max(slack_vals(center_mask));
                
                fprintf("[RIGID ELLIPSOID QP] 最適化成功! (反復=%d, 最接近時マージン侵入=%.3fm)\n", ...
                    output.iterations, max_slack_center);
                
                C_opt = X_opt(1:num_C);
                obj.poly_coeffs = [C_opt(1:n_coeffs), C_opt(n_coeffs+1:2*n_coeffs), C_opt(2*n_coeffs+1:end)]';
            end
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
                    val_k = val_k + C(:, n + 1) * factor * (u^(n - k));
                end
                xd(4*k + (1:3)) = val_k / (T^k);
            end
        end
    end
end
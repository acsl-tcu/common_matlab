classdef REPLANNING_MELLINGER_CORRIDOR_FLATNESS_SWING < handle
    % REPLANNING_MELLINGER_CORRIDOR_FLATNESS_SWING
    % 任意姿勢楕円体・動的速度・動的距離適応 Mellinger Corridor QP リプランナ
    % - 障害物までの射影距離と速度から通過時刻・計画時間 T を完全自動適応
    % - 障害物サイズに応じて制約サンプリング区間を自動スケーリング
    % - 荷物 (Load) と ドローン実位置 (Drone) の双方に安全壁を連立
    
    properties
        base_ref
        self
        replan_active = false
        replan_done   = false
        
        t_start
        t_duration = 8.0           % 自動決定される所要時間 [s]
        
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
            
            % 1. 動的障害物検知
            if cha == 'f' && ~obj.replan_done && ~obj.replan_active
                try
                    obs_list = ENVIRONMENT_OBSTACLE_ELLIPSE();
                catch
                    try
                        obs_list = ENVIRONMENT_OBSTACLE_HOCBF_LINK_XY();
                    catch
                        obs_list = [];
                    end
                end
                
                min_dist_overall = inf;
                target_obs_idx = -1;
                
                for i = 1:length(obs_list)
                    if isfield(obs_list(i), 'p_center')
                        c = obs_list(i).p_center;
                    elseif isfield(obs_list(i), 'p_obs')
                        c = obs_list(i).p_obs;
                    else
                        continue;
                    end
                    
                    if isfield(obs_list(i), 'ellipsoid_radii')
                        radii = obs_list(i).ellipsoid_radii;
                        r_equiv = max(radii);
                    elseif isfield(obs_list(i), 'r_obs')
                        r_equiv = obs_list(i).r_obs;
                    else
                        r_equiv = 0.5;
                    end
                    
                    if isfield(obs_list(i), 'd_margin')
                        d_marg = obs_list(i).d_margin;
                    else
                        d_marg = obj.safe_margin;
                    end
                    
                    dist_load  = norm(pL_cur - c) - (r_equiv + obj.r_load + d_marg);
                    dist_drone = norm(pQ_cur - c) - (r_equiv + obj.r_drone + d_marg);
                    d_surf = min(dist_load, dist_drone);
                    
                    if d_surf < min_dist_overall
                        min_dist_overall = d_surf;
                        target_obs_idx = i;
                    end
                end
                
                if target_obs_idx > 0 && min_dist_overall <= obj.trigger_dist
                    if isfield(obs_list(target_obs_idx), 'p_center')
                        obj.obs_center = obs_list(target_obs_idx).p_center;
                    else
                        obj.obs_center = obs_list(target_obs_idx).p_obs;
                    end
                    
                    if isfield(obs_list(target_obs_idx), 'R_obs')
                        obj.obs_R = obs_list(target_obs_idx).R_obs;
                    else
                        obj.obs_R = eye(3);
                    end
                    
                    if isfield(obs_list(target_obs_idx), 'ellipsoid_radii')
                        obj.obs_radii = obs_list(target_obs_idx).ellipsoid_radii;
                    else
                        r = obs_list(target_obs_idx).r_obs;
                        obj.obs_radii = [r; r; r];
                    end
                    
                    if isfield(obs_list(target_obs_idx), 'd_margin')
                        obj.obs_margin = obs_list(target_obs_idx).d_margin;
                    else
                        obj.obs_margin = obj.safe_margin;
                    end
                    
                    obj.t_start = time.t;
                    
                    % 速度と方向ベクトルの同定
                    v_vec = xd_nominal(5:7);
                    spd = norm(v_vec);
                    if spd < 0.05, spd = norm(vL_cur); end
                    if spd < 0.05, spd = 0.5; v_vec = [0; 0; 0.5]; end
                    dir_nom = v_vec / spd;
                    
                    % -------------------------------------------------------------
                    % 【完全自動連動】射影距離と速度から T と通過タイミングを厳密決定
                    % -------------------------------------------------------------
                    vec_to_obs = obj.obs_center - pL_cur;
                    d_proj = dot(vec_to_obs, dir_nom);
                    if d_proj < 0.5, d_proj = 0.5; end % 最低助走マージン
                    
                    % 障害物中心までの到達時間
                    t_cross = d_proj / spd;
                    % 助走と復帰を対称にするため T = 2 * t_cross (常に u=0.5 で最接近)
                    obj.t_duration = max(5.0, 2.0 * t_cross);
                    
                    % 終端合流距離
                    total_dist = spd * obj.t_duration;
                    
                    fprintf("\n=======================================================\n");
                    fprintf("[ADAPTIVE QP] 障害物検知! (ID: %d, d_surf=%.2fm, t=%.3f s)\n", target_obs_idx, min_dist_overall, time.t);
                    fprintf("  - 進行速度: %.2f m/s, 障害物射影距離: %.2f m\n", spd, d_proj);
                    fprintf("  - 通過予定時間: %.2f s後 (u=0.50に自動ロック), 計画時間 T = %.2f s\n", t_cross, obj.t_duration);
                    fprintf("  - 楕円主軸半径: [a=%.2f, b=%.2f, c=%.2f] m\n", obj.obs_radii(1), obj.obs_radii(2), obj.obs_radii(3));
                    fprintf("  - 荷物半径: %.2f m / ドローン保護半径: %.2f m / マージン帯: %.2f m\n", obj.r_load, obj.r_drone, obj.obs_margin);
                    
                    obj.plan_mellinger_adaptive_qp(xd_nominal, dir_nom, spd, total_dist, d_proj);
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
                        fprintf("[ADAPTIVE QP] 障害物通過完了・C^6シームレス合流 (t=%.3f s)\n\n", time.t);
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
        function plan_mellinger_adaptive_qp(obj, xd0, dir_nom, spd, total_dist, d_proj)
            T = obj.t_duration;
            p0 = xd0(1:3);
            order = 13;
            n_coeffs = order + 1;
            
            % 1. 法線ベクトル（障害物から自機を外側へ押し出す向き）
            vec_from_obs = p0 - obj.obs_center;
            proj_on_line = dot(vec_from_obs, dir_nom) * dir_nom;
            normal_vec   = vec_from_obs - proj_on_line;
            
            if norm(normal_vec) < 1e-3
                if abs(dir_nom(3)) < 0.9
                    normal_vec = cross(dir_nom, [0; 0; 1]);
                else
                    normal_vec = cross(dir_nom, [1; 0; 0]);
                end
            end
            dir_normal = normal_vec / norm(normal_vec);
            
            % 2. 楕円体の実効横半径
            n_local = obj.obs_R' * dir_normal;
            r_eff_obs = sqrt((obj.obs_radii(1) * n_local(1))^2 + ...
                             (obj.obs_radii(2) * n_local(2))^2 + ...
                             (obj.obs_radii(3) * n_local(3))^2);
            
            R_hard_load  = r_eff_obs + obj.r_load;
            R_hard_drone = r_eff_obs + obj.r_drone;
            
            % 進行方向における障害物の実効長さ r_longitudinal
            dir_local = obj.obs_R' * dir_nom;
            r_eff_long = sqrt((obj.obs_radii(1) * dir_local(1))^2 + ...
                              (obj.obs_radii(2) * dir_local(2))^2 + ...
                              (obj.obs_radii(3) * dir_local(3))^2);
            
            % -------------------------------------------------------------
            % 【自動連動】障害物サイズと速度に応じた無次元制約幅 delta_u の計算
            % -------------------------------------------------------------
            % 障害物の前後 (r_long + margin) を通過する時間幅
            t_span = (r_eff_long + obj.obs_margin) / spd;
            delta_u = max(0.12, min(0.35, t_span / T));
            
            % ハード制約およびソフト制約の区間を [0.5 - delta_u, 0.5 + delta_u] に完全同期
            u_start = max(0.15, 0.50 - delta_u * 1.3);
            u_end   = min(0.85, 0.50 + delta_u * 1.3);
            
            N_samples = 15;
            u_samples = linspace(u_start, u_end, N_samples);
            num_C = 3 * n_coeffs;
            num_vars = num_C + N_samples;
            
            % 3. 目的関数 H
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
            
            w_slack_quad = 1000000.0; % マージン侵入ペナルティを大幅強化
            w_slack_lin  = 100000.0;
            for s_idx = 1:N_samples
                H(num_C + s_idx, num_C + s_idx) = w_slack_quad;
            end
            f = zeros(num_vars, 1);
            f(num_C + 1 : end) = w_slack_lin;
            
            % 4. 等式制約
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
            
            % 5. 不等式制約（適応型サンプリング）
            alpha_dyn = obj.L_cable / (obj.gravity * (T^2));
            p_base_proj = dot(dir_normal, obj.obs_center);
            
            % 障害物の実体が存在するコア区間 [0.5 - delta_u, 0.5 + delta_u]
            hard_mask = (u_samples >= (0.50 - delta_u)) & (u_samples <= (0.50 + delta_u));
            n_hard = sum(hard_mask);
            
            num_ineq = n_hard + n_hard + N_samples + N_samples;
            A_ineq = zeros(num_ineq, num_vars);
            b_ineq = zeros(num_ineq, 1);
            
            row = 1;
            % a) 荷物ハード
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
            
            % b) ドローン実位置ハード
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
            
            % c) 荷物ソフト制約（u=0.5 をピークとする山なりエンベロープ）
            for m = 1:N_samples
                u_m = u_samples(m);
                T0 = zeros(1, n_coeffs);
                for n = 0:order, T0(n+1) = u_m^n; end
                
                % u_start から u_end にかけて u=0.5 で最大となるエンベロープ
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
            
            % d) スラック非負
            for m = 1:N_samples
                A_ineq(row, num_C + m) = -1;
                b_ineq(row) = 0;
                row = row + 1;
            end
            
            % 6. 最適化実行
            opts = optimoptions('quadprog', ...
                'Display', 'off', ...
                'Algorithm', 'interior-point-convex', ...
                'MaxIterations', 200, ...
                'ConstraintTolerance', 1e-4, ...
                'OptimalityTolerance', 1e-4, ...
                'StepTolerance', 1e-8);
            
            [X_opt, ~, exitflag, output] = quadprog(H, f, A_ineq, b_ineq, Aeq, beq, [], [], [], opts);
            
            if exitflag < 1
                fprintf("[ADAPTIVE QP DEBUG] quadprog未収束 (exitflag=%d: %s)\n", exitflag, output.message);
                C_1 = Aeq_1d \ beq(1:n_eq);
                C_2 = Aeq_1d \ beq(n_eq+1:2*n_eq);
                C_3 = Aeq_1d \ beq(2*n_eq+1:3*n_eq);
                obj.poly_coeffs = [C_1, C_2, C_3]';
            else
                slack_vals = X_opt(num_C + 1 : end);
                center_mask = (u_samples >= (0.50 - delta_u/2)) & (u_samples <= (0.50 + delta_u/2));
                max_slack_center = max(slack_vals(center_mask));
                
                fprintf("[ADAPTIVE QP] 最適化成功! (反復=%d, 最接近時マージン侵入=%.3fm)\n", ...
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
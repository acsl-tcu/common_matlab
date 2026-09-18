classdef REPLANNING_MELLINGER_CORRIDOR_FLATNESS_SWING < handle
    % REPLANNING_MELLINGER_CORRIDOR_FLATNESS_SWING
    % 論文準拠 2区間 C^6 完全連続 偏差多項式 (Error-Spline) Mellinger QP
    % - 始端・中間接続・終端のすべてで位置〜6階微分 (C^6) を完全一致拘束
    % - 公称直進軌道からの「横方向偏差」のみを最適化することで過剰膨らみを原理的に撲滅
    % - 楕円マハラノビス空間分離超平面 & 微分平坦性ドローン連立防護
    
    properties
        base_ref
        self
        replan_active = false
        replan_done   = false
        
        t_start
        t_duration = 8.0
        T_seg
        
        safe_margin  = 0.5
        trigger_dist = 5.0
        
        L_cable      = 2.0
        gravity      = 9.81
        
        r_load       = 0.15
        r_drone      = 0.30
        
        order = 13        % 13次多項式 (係数14個: 始端7+中間7=14を完全に受け止める最適次数)
        coeffs_delta_seg1 % 区間1 偏差係数 (1 x 14)
        coeffs_delta_seg2 % 区間2 偏差係数 (1 x 14)
        
        dir_normal        % 最適回避法線単位ベクトル (3 x 1)
        dir_nominal       % 進行方向単位ベクトル (3 x 1)
        nominal_speed     % 進入巡航速度
        p_start_replan    % リプラン開始位置
        
        t_merge_end
        p_merge_end
        v_merge_vec
        
        obs_center = [0; 0; 0]
        obs_R      = eye(3)
        obs_radii  = [1; 1; 1]
        obs_margin = 0
        
        debug_cnt  = 0
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
                obs_list = [];
                try
                    obs_list = ENVIRONMENT_OBSTACLE_ELLIPSE();
                catch ME
                    if obj.debug_cnt == 0
                        fprintf(2, "[ERROR in ENVIRONMENT_OBSTACLE_ELLIPSE]: %s (Line: %d)\n", ...
                            ME.message, ME.stack(1).line);
                    end
                end
                
                min_dist_overall = inf;
                target_obs_idx = -1;
                
                for i = 1:length(obs_list)
                    if isfield(obs_list(i), 'p_center') && ~isempty(obs_list(i).p_center)
                        c = obs_list(i).p_center;
                    elseif isfield(obs_list(i), 'p_obs') && ~isempty(obs_list(i).p_obs)
                        c = obs_list(i).p_obs;
                    else
                        continue;
                    end
                    
                    if isfield(obs_list(i), 'ellipsoid_radii') && ~isempty(obs_list(i).ellipsoid_radii)
                        r_max = max(obs_list(i).ellipsoid_radii);
                    elseif isfield(obs_list(i), 'r_obs') && ~isempty(obs_list(i).r_obs)
                        r_max = obs_list(i).r_obs;
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
                
                if target_obs_idx > 0 && min_dist_overall <= obj.trigger_dist
                    tgt = obs_list(target_obs_idx);
                    
                    if isfield(tgt, 'p_center'), obj.obs_center = tgt.p_center;
                    else, obj.obs_center = tgt.p_obs; end
                    
                    if isfield(tgt, 'R_obs'), obj.obs_R = tgt.R_obs;
                    else, obj.obs_R = eye(3); end
                    
                    if isfield(tgt, 'ellipsoid_radii'), obj.obs_radii = tgt.ellipsoid_radii;
                    else, obj.obs_radii = [0.71; 0.71; 0.71]; end
                    
                    if isfield(tgt, 'd_margin'), obj.obs_margin = tgt.d_margin;
                    else, obj.obs_margin = obj.safe_margin; end
                    
                    obj.t_start = time.t;
                    obj.p_start_replan = pL_cur;
                    
                    v_vec = xd_nominal(5:7);
                    spd = norm(v_vec);
                    if spd < 0.05, spd = norm(vL_cur); end
                    if spd < 0.05, spd = 0.5; v_vec = [0; 0; 0.5]; end
                    obj.dir_nominal = v_vec / spd;
                    obj.nominal_speed = spd;
                    
                    vec_to_obs = obj.obs_center - pL_cur;
                    d_proj = dot(vec_to_obs, obj.dir_nominal);
                    if d_proj < 0.5, d_proj = 0.5; end
                    
                    t_cross = d_proj / spd;
                    obj.T_seg = max(3.0, t_cross);
                    obj.t_duration = 2.0 * obj.T_seg;
                    
                    fprintf("\n=======================================================\n");
                    fprintf("[C^6 PIECEWISE QP] 障害物検知! (ID: %d, d_surf=%.2fm, t=%.3f s)\n", target_obs_idx, min_dist_overall, time.t);
                    fprintf("  - モード: 2区間 C^6 完全一致 偏差スプライン (ルンゲ発振根絶)\n");
                    fprintf("  - 計画時間 T = %.2f s (区間1: %.2f s, 区間2: %.2f s, 中間点=障害物真横)\n", ...
                        obj.t_duration, obj.T_seg, obj.T_seg);
                    fprintf("  - 楕円主軸半径: [a=%.2f, b=%.2f, c=%.2f] m\n", obj.obs_radii(1), obj.obs_radii(2), obj.obs_radii(3));
                    fprintf("  - 荷物半径: %.2f m / ドローン保護半径: %.2f m / マージン帯: %.2f m\n", obj.r_load, obj.r_drone, obj.obs_margin);
                    
                    obj.plan_mellinger_c6_error_spline_qp();
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
                        fprintf("[C^6 PIECEWISE QP] 障害物通過完了・完全シームレス復帰 (t=%.3f s)\n\n", time.t);
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
        function plan_mellinger_c6_error_spline_qp(obj)
            N = obj.order;           % 13次
            n_c = N + 1;             % 14個
            n_vars = 2 * n_c;        % 28個 (区間1: 14個, 区間2: 14個)
            T_seg = obj.T_seg;
            
            % 1. マハラノビス空間での幾何最短法線導出
            D_inv = diag(1 ./ obj.obs_radii);
            z0 = D_inv * obj.obs_R' * (obj.p_start_replan - obj.obs_center);
            vz = D_inv * obj.obs_R' * obj.dir_nominal;
            
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
            obj.dir_normal = n_opt / norm_n_opt;
            r_eff_obs  = 1.0 / norm_n_opt;
            
            % 公称中心線から障害物中心までの法線方向オフセット
            p_center_proj = dot(obj.dir_normal, obj.obs_center - obj.p_start_replan);
            
            % 必要な最大横逃げ量 delta_target
            R_hard_load  = p_center_proj + r_eff_obs + obj.r_load;
            R_hard_drone = p_center_proj + r_eff_obs + obj.r_drone - (obj.dir_normal(3) * obj.L_cable);
            delta_target = max(R_hard_load, R_hard_drone) + obj.obs_margin;
            
            % サンプル点の配置 (無次元 u in [0, 1])
            N1 = 6; N2 = 6;
            u1_samples = linspace(0.5, 1.0, N1);
            u2_samples = linspace(0.0, 0.5, N2);
            N_samples = N1 + N2;
            num_opt_vars = n_vars + N_samples;
            
            % 2. 目的関数 H: 無次元 Snap (4階) ＆ 加速度 (2階) 最小化
            H_1d = zeros(n_c, n_c);
            w0 = 0.01; % 偏差そのものの抑制 (不要な大回りを厳密防止)
            w2 = 0.10; % 加速度
            w4 = 1.00; % Snap (滑らかさ)
            for i = 0:N
                for j = 0:N
                    H_1d(i+1, j+1) = H_1d(i+1, j+1) + w0 / (i + j + 1);
                    if i >= 2 && j >= 2
                        val2 = prod(i-1:i) * prod(j-1:j) / (i + j - 3);
                        H_1d(i+1, j+1) = H_1d(i+1, j+1) + w2 * val2;
                    end
                    if i >= 4 && j >= 4
                        val4 = prod(i-3:i) * prod(j-3:j) / (i + j - 7);
                        H_1d(i+1, j+1) = H_1d(i+1, j+1) + w4 * val4;
                    end
                end
            end
            H_1d = H_1d / norm(H_1d, 2);
            
            H = zeros(num_opt_vars, num_opt_vars);
            H(1:n_vars, 1:n_vars) = blkdiag(H_1d, H_1d) + 1e-6 * eye(n_vars);
            
            w_slack_quad = 100000.0;
            w_slack_lin  = 10000.0;
            for s_idx = 1:N_samples
                H(n_vars + s_idx, n_vars + s_idx) = w_slack_quad;
            end
            f = zeros(num_opt_vars, 1);
            f(n_vars + 1 : end) = w_slack_lin;
            
            % -------------------------------------------------------------
            % 3. 等式制約: 始端(7: C^6) + 中間接続(7: C^6完全一致) + 終端(7: C^6) = 21本
            % -------------------------------------------------------------
            n_eq = 21;
            Aeq = zeros(n_eq, num_opt_vars);
            beq = zeros(n_eq, 1);
            
            % a) 始端拘束 (区間1の u = 0): k = 0..6階微分が全て 0
            for k = 0:6
                row = k + 1;
                for n = k:N
                    Aeq(row, n+1) = prod(n-k+1:n) * 0^(n-k);
                end
                beq(row) = 0; % 公称軌道から完全にゼロで分岐
            end
            
            % b) 中間C^6完全接続条件 (区間1の u=1 と 区間2の u=0): k = 0..6階微分完全一致
            for k = 0:6
                row = 7 + k + 1;
                for n = k:N, Aeq(row, n+1) = prod(n-k+1:n) * 1.0^(n-k); end
                for n = k:N, Aeq(row, n_c + n+1) = -prod(n-k+1:n) * 0^(n-k); end
                beq(row) = 0; % 完璧な連続性
            end
            
            % c) 終端拘束 (区間2の u = 1): k = 0..6階微分が全て 0
            for k = 0:6
                row = 14 + k + 1;
                for n = k:N
                    Aeq(row, n_c + n+1) = prod(n-k+1:n) * 1.0^(n-k);
                end
                beq(row) = 0; % 公称軌道へ完全にゼロで合流
            end
            
            % -------------------------------------------------------------
            % 4. 不等式制約: 障害物マージン外殻の維持
            % -------------------------------------------------------------
            alpha_dyn = obj.L_cable / (obj.gravity * (T_seg^2));
            
            num_ineq = N_samples + N_samples + N_samples;
            A_ineq = zeros(num_ineq, num_opt_vars);
            b_ineq = zeros(num_ineq, 1);
            
            row = 1;
            % 荷物のソフトマージン（区間1: u1=0.5 -> 1.0）
            for m = 1:N1
                u_m = u1_samples(m);
                T0 = u_m .^ (0:N);
                env = sin((pi/2) * (u_m - 0.5) / 0.5);
                req_delta = delta_target * env;
                
                A_ineq(row, 1:n_c) = -T0; % delta(u) >= req_delta
                A_ineq(row, n_vars + m) = -1;
                b_ineq(row) = -req_delta;
                row = row + 1;
            end
            % 荷物のソフトマージン（区間2: u2=0.0 -> 0.5）
            for m = 1:N2
                u_m = u2_samples(m);
                T0 = u_m .^ (0:N);
                env = cos((pi/2) * u_m / 0.5);
                req_delta = delta_target * env;
                
                A_ineq(row, n_c + 1 : n_vars) = -T0;
                A_ineq(row, n_vars + N1 + m) = -1;
                b_ineq(row) = -req_delta;
                row = row + 1;
            end
            
            % ドローン実位置の連立ハード制約 (最接近点 u1=1.0 と u2=0.0)
            % delta_drone = delta + alpha_dyn * delta'' >= delta_target
            T0_mid = 1.0 .^ (0:N);
            T2_mid = zeros(1, n_c);
            for n = 2:N, T2_mid(n+1) = n * (n - 1) * (1.0^(n - 2)); end
            TQ_mid = T0_mid + alpha_dyn * T2_mid;
            
            A_ineq(row, 1:n_c) = -TQ_mid;
            b_ineq(row) = -delta_target;
            row = row + 1;
            
            % スラック非負制約
            for m = 1:N_samples
                A_ineq(row, n_vars + m) = -1;
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
                fprintf("[C^6 PIECEWISE DEBUG] quadprog未収束 (exitflag=%d: %s)\n", exitflag, output.message);
                X_fb = Aeq(:, 1:n_vars) \ beq;
                obj.coeffs_delta_seg1 = X_fb(1:n_c)';
                obj.coeffs_delta_seg2 = X_fb(n_c+1:n_vars)';
            else
                slack_vals = X_opt(n_vars + 1 : end);
                max_slack = max(slack_vals);
                
                fprintf("[C^6 PIECEWISE] 最適化成功! (反復=%d, 最接近時マージン侵入=%.3fm)\n", ...
                    output.iterations, max_slack);
                
                X_c = X_opt(1:n_vars);
                obj.coeffs_delta_seg1 = X_c(1:n_c)';
                obj.coeffs_delta_seg2 = X_c(n_c+1:n_vars)';
            end
        end
        
        function xd = evaluate_smooth_trajectory(obj, tau, xd_nom)
            xd = xd_nom;
            N = obj.order;
            T_seg = obj.T_seg;
            
            % 1. 現在の区間と偏差多項式の選択
            if tau <= T_seg
                C = obj.coeffs_delta_seg1;
                u = max(0, min(1.0, tau / T_seg));
            else
                C = obj.coeffs_delta_seg2;
                u = max(0, min(1.0, (tau - T_seg) / T_seg));
            end
            
            % 2. 偏差 delta とその 1〜6 階微分の計算
            delta_k = zeros(7, 1);
            for k = 0:6
                val_k = 0;
                for n = k:N
                    factor = prod(n - k + 1 : n);
                    val_k = val_k + C(n + 1) * factor * (u^(n - k));
                end
                delta_k(k + 1) = val_k / (T_seg^k);
            end
            
            % 3. 公称軌道に法線方向の滑らかな偏差 delta を加算 (C^6 級)
            for k = 0:6
                idx = 4 * k + (1:3);
                xd(idx) = xd_nom(idx) + obj.dir_normal * delta_k(k + 1);
            end
        end
    end
end
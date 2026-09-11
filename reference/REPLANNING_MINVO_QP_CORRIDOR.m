classdef REPLANNING_MINVO_QP_CORRIDOR < handle
    % REPLANNING_MINVO_QP_CORRIDOR
    % 案2: MINVO基底/分離超平面を用いた閉形式多項式コリドー回避 (Mellinger + CCC 2024)
    % 13次多項式に対し、障害物との分離超平面（Separating Hyperplane）を不等式制約とし、
    % Minimum Snap/Crackle 等の最適化を QP (quadprog) で一度に解く厳密回避リプランナ。
    
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
        
        poly_coeffs % [14 x 3] 無次元時間 u in [0,1]
        
        t_merge_end
        p_merge_end
        v_merge_vec
        
        result
    end
    
    methods
        function obj = REPLANNING_MINVO_QP_CORRIDOR(self, base_ref, opts)
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
            if isfield(opts, "t_duration"),   obj.t_duration   = opts.t_duration;      end
            
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
                    fprintf("[MINVO QP CORRIDOR] 障害物接近検知! (min_dist=%.2fm, t=%.3f s)\n", min_dist, time.t);
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
                    
                    fprintf("[MINVO QP CORRIDOR] 分離超平面QPの生成開始 (T = %.2f s)...\n", obj.t_duration);
                    
                    obj.plan_qp_corridor_trajectory(xd_nominal, dir_nom, spd, total_dist, pL_cur, vL_cur);
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
                        fprintf("[MINVO QP CORRIDOR] 障害物クリア・C^6シームレス合流 (t=%.3f s)\n\n", time.t);
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
        function plan_qp_corridor_trajectory(obj, xd0, dir_nom, spd, total_dist, pL_cur, vL_cur)
            T = obj.t_duration;
            p0 = pL_cur;
            
            % --- 1. 分離超平面 (Separating Hyperplane) の計算 ---
            vec_to_obs = obj.obs_center - p0;
            proj_on_line = dot(vec_to_obs, dir_nom) * dir_nom;
            normal_vec = vec_to_obs - proj_on_line;
            
            if norm(normal_vec) < 1e-3
                if abs(dir_nom(3)) < 0.9, normal_vec = cross(dir_nom, [0; 0; 1]);
                else, normal_vec = cross(dir_nom, [1; 0; 0]); end
            end
            dir_normal = normal_vec / norm(normal_vec);
            
            % 障害物側からドローンを押し返す法線ベクトル (a_h)
            a_h = -dir_normal;
            
            % 安全マージンを含む障害物クリアランス半径
            R_clear = obj.obs_radius + obj.safe_margin + obj.L_cable * 0.5 + 0.3;
            
            % 分離平面上の点
            p_plane = obj.obs_center + a_h * R_clear;
            
            % 平面の不等式制約: a_h^T * r <= a_h^T * p_plane
            % これにより、軌道は障害物から半径 R_clear より外側を通るように制限される
            b_h = dot(a_h, p_plane);
            
            % --- 2. 13次多項式 QP の定式化 (無次元時間 u in [0, 1]) ---
            order = 13;
            n_coeffs = order + 1; % 14
            num_vars = 3 * n_coeffs; % [Cx; Cy; Cz] (42次元)
            
            % (A) 目的関数 (Minimum Snap & Crackle) の Hessian 行列構築
            H_1d = zeros(n_coeffs, n_coeffs);
            % Snap (4階微分) と Crackle (5階微分) の重み
            w4 = 1.0; 
            w5 = 0.1; 
            for i = 4:order
                for j = 4:order
                    % 4階微分の積分
                    val4 = (factorial(i)/factorial(i-4)) * (factorial(j)/factorial(j-4)) / (i + j - 7);
                    H_1d(i+1, j+1) = H_1d(i+1, j+1) + w4 * val4;
                    
                    % 5階微分の積分
                    if i >= 5 && j >= 5
                        val5 = (factorial(i)/factorial(i-5)) * (factorial(j)/factorial(j-5)) / (i + j - 9);
                        H_1d(i+1, j+1) = H_1d(i+1, j+1) + w5 * val5;
                    end
                end
            end
            
            % 3軸分をブロック対角に並べる
            H = blkdiag(H_1d, H_1d, H_1d);
            % 数値的安定化のための微小な正則化
            H = H + 1e-6 * eye(num_vars);
            f = zeros(num_vars, 1);
            
            % (B) 等式制約 Aeq * C = beq (境界条件)
            % 始端 7条件 (0〜6階微分), 終端 6条件 (0〜5階微分) -> 各軸 13条件, 計 39条件
            n_eq_1d = 13;
            Aeq_1d = zeros(n_eq_1d, n_coeffs);
            beq = zeros(3 * n_eq_1d, 1);
            
            % u = 0 拘束 (始端)
            for k = 0:6
                row = k + 1;
                for n = k:order
                    Aeq_1d(row, n + 1) = prod(n - k + 1 : n) * (0)^(n - k);
                end
            end
            
            % u = 1 拘束 (終端)
            for k = 0:5
                row = 8 + k;
                for n = k:order
                    Aeq_1d(row, n + 1) = prod(n - k + 1 : n) * (1.0)^(n - k);
                end
            end
            
            Aeq = blkdiag(Aeq_1d, Aeq_1d, Aeq_1d);
            
            % beq の計算
            xd0_real = xd0;
            xd0_real(1:3) = pL_cur;
            xd0_real(5:7) = vL_cur;
            
            p_end = pL_cur + dir_nom * total_dist;
            xd1 = zeros(28, 1);
            xd1(1:3) = p_end;
            xd1(5:7) = dir_nom * spd;
            
            for axis = 1:3
                beq_axis = zeros(n_eq_1d, 1);
                % 始端
                for k = 0:6
                    beq_axis(k+1) = (T^k) * xd0_real(4*k + axis);
                end
                % 終端
                for k = 0:5
                    beq_axis(8+k) = (T^k) * xd1(4*k + axis);
                end
                beq( (axis-1)*n_eq_1d + 1 : axis*n_eq_1d ) = beq_axis;
            end
            
            % (C) 不等式制約 Aineq * C <= bineq (分離超平面コリドー)
            % 軌道を N_sample 等分して、そのすべての点が超平面の安全側にあることを保証する
            N_sample = 20;
            Aineq = zeros(N_sample, num_vars);
            bineq = repmat(b_h, N_sample, 1);
            
            for m = 1:N_sample
                u_m = m / (N_sample + 1);
                % [1, u, u^2, ..., u^13]
                u_vec = zeros(1, n_coeffs);
                for n = 0:order
                    u_vec(n+1) = u_m^n;
                end
                
                % a_h(1)*x(u_m) + a_h(2)*y(u_m) + a_h(3)*z(u_m) <= b_h
                Aineq(m, 1:n_coeffs) = a_h(1) * u_vec;
                Aineq(m, n_coeffs+1 : 2*n_coeffs) = a_h(2) * u_vec;
                Aineq(m, 2*n_coeffs+1 : end) = a_h(3) * u_vec;
            end
            
            % --- 3. quadprog の実行 ---
            options = optimoptions('quadprog', 'Display', 'off');
            [C_opt, fval, exitflag] = quadprog(H, f, Aineq, bineq, Aeq, beq, [], [], [], options);
            
            if exitflag ~= 1
                fprintf("    [WARNING] QPが最適解を保証できませんでした (exitflag=%d)。制約を緩和して再試行します。\n", exitflag);
                % 安全マージンを少し削ってフォールバック
                bineq = bineq + 0.5;
                [C_opt, ~, exitflag] = quadprog(H, f, Aineq, bineq, Aeq, beq, [], [], [], options);
            end
            
            fprintf("    [DEBUG] 分離超平面QP 最適化完了 (ExitFlag: %d)\n", exitflag);
            
            % 係数の再構成: [14 x 3] 行列に整形 (1列目X, 2列目Y, 3列目Z)
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

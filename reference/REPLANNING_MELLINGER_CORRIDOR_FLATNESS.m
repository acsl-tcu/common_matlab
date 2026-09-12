classdef REPLANNING_MELLINGER_CORRIDOR_FLATNESS < handle
    % REPLANNING_MELLINGER_CORRIDOR_FLATNESS
    % Mellingerの原著に忠実な「安全回廊(Corridor/分離超平面)」を用いた不等式制約QPと、
    % 微分平坦性(Differential Flatness)を利用してドローン・紐・荷物を全て守る2点バリア型リプランナ。
    
    properties
        base_ref
        self
        replan_active = false
        replan_done   = false
        
        t_start
        t_duration = 10.0
        
        % 障害物情報 (実行時に ENVIRONMENT_OBSTACLE_HOCBF_LINK_XY から取得して保持)
        obs_center   = [];
        obs_radius   = [];
        
        % 幾何学マージン
        safe_margin  = 0.4
        trigger_dist = 3.5
        
        L_cable = 2.0
        gravity = 9.81
        
        poly_coeffs 
        
        t_merge_end
        p_merge_end
        v_merge_vec
        
        % 描画・可視化用の保存データ
        hyperplane_n = [1;0;0];
        hyperplane_p = [0;0;0];
        
        result
    end
    
    methods
        function obj = REPLANNING_MELLINGER_CORRIDOR_FLATNESS(self, base_ref, opts)
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
            
            % 環境クラスから障害物を読み取る仕組みがある場合はここで上書き可能
            if isprop(self, 'environment') && isfield(self.environment, 'obstacles') && ~isempty(self.environment.obstacles)
                % 仮実装: 最初に見つかった障害物をターゲットにする
                % obj.obs_center = self.environment.obstacles{1}.param.center;
                % obj.obs_radius = self.environment.obstacles{1}.param.r;
            end
            
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
                % 環境関数から障害物リストを読み込む
                try
                    obs_list = ENVIRONMENT_OBSTACLE_HOCBF_LINK_XY();
                catch
                    obs_list = [];
                end
                
                % 最も近い障害物を探す
                min_dist_overall = inf;
                target_obs_idx = -1;
                
                for i = 1:length(obs_list)
                    % ここでは Bounding Sphere のパラメータを使用 (param: [r])
                    % もし p_center が無ければスキップ
                    if isfield(obs_list(i), 'p_obs') && isfield(obs_list(i), 'r_obs')
                        c = obs_list(i).p_obs;
                        r = obs_list(i).r_obs; % Bounding Sphere の半径
                        
                        dist_load = norm(pL_cur - c);
                        dist_drone = norm(pQ_cur - c);
                        % 表面までの距離を近似的に測る
                        d = min(dist_load, dist_drone) - r;
                        
                        if d < min_dist_overall
                            min_dist_overall = d;
                            target_obs_idx = i;
                        end
                    end
                end
                
                % もし trigger_dist (表面からの距離) 以内に近づいたら回避開始
                if target_obs_idx > 0 && min_dist_overall <= obj.trigger_dist
                    obj.obs_center = obs_list(target_obs_idx).p_obs;
                    obj.obs_radius = obs_list(target_obs_idx).r_obs;
                    
                    fprintf("\n=======================================================\n");
                    fprintf("[MELLINGER FLATNESS] 障害物検知! (ID: %d, d_surf=%.2fm, t=%.3f s)\n", target_obs_idx, min_dist_overall, time.t);

                    obj.t_start = time.t;
                    
                    v_vec = xd_nominal(5:7);
                    spd = norm(v_vec);
                    if spd < 0.05, spd = norm(vL_cur); end
                    if spd < 0.05, spd = 0.3; v_vec = [0; 0; 0.3]; end
                    dir_nom = v_vec / spd;
                    
                    vec_to_obs = obj.obs_center - pL_cur;
                    proj_dist = max(0, dot(vec_to_obs, dir_nom));
                    
                    % ※今回はマージンにケーブル長を足しません！(凸性と平坦性で守るため)
                    total_dist = proj_dist + obj.obs_radius + obj.safe_margin + 1.5;
                    obj.t_duration = max(8.0, min(14.0, total_dist / spd));
                    
                    p0_ref = xd_nominal(1:3);
                    v0_ref = xd_nominal(5:7);
                    
                    fprintf("[MELLINGER FLATNESS] 安全回廊(Corridor)の計算と2点バリア型QPの実行...\n");
                    
                    obj.plan_corridor_flatness_qp(xd_nominal, dir_nom, spd, total_dist);
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
                        fprintf("[MELLINGER FLATNESS] 障害物クリア・C^6シームレス合流 (t=%.3f s)\n\n", time.t);
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
        function plan_corridor_flatness_qp(obj, xd0, dir_nom, spd, total_dist)
            T = obj.t_duration;
            p0 = xd0(1:3);
            
            % 1. 障害物を避ける分離超平面(Corridor Wall)の構築
            vec_to_obs = obj.obs_center - p0;
            proj_on_line = dot(vec_to_obs, dir_nom) * dir_nom;
            normal_vec = vec_to_obs - proj_on_line;
            if norm(normal_vec) < 1e-3
                if abs(dir_nom(3)) < 0.9, normal_vec = cross(dir_nom, [0; 0; 1]);
                else, normal_vec = cross(dir_nom, [1; 0; 0]); end
            end
            
            % 壁の法線(障害物から逃げる方向)
            n_wall = - (normal_vec / norm(normal_vec)); 
            
            % 壁の通過点(障害物中心から半径+マージン分だけ逃げた位置)
            R_clear = obj.obs_radius + obj.safe_margin;
            p_wall = obj.obs_center + n_wall * R_clear;
            
            % 壁の不等式: n_wall^T x >= n_wall^T p_wall
            % つまり -n_wall^T x <= -n_wall^T p_wall
            a_ineq = -n_wall;
            b_ineq = -dot(n_wall, p_wall);
            
            % 描画用保存
            obj.hyperplane_n = n_wall;
            obj.hyperplane_p = p_wall;
            
            p_end = p0 + dir_nom * total_dist;
            xd1 = zeros(28, 1);
            xd1(1:3) = p_end;
            xd1(5:7) = dir_nom * spd;
            
            % 2. 13次多項式 QP の目的関数 (Minimum Snap等)
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
            H = blkdiag(H_1d, H_1d, H_1d) + 1e-6 * eye(num_vars);
            f = zeros(num_vars, 1);
            
            % 3. 等式制約 (始端と終端の滑らかさ)
            n_eq_1d = 13;
            Aeq_1d = zeros(n_eq_1d, n_coeffs);
            beq = zeros(3 * n_eq_1d, 1);
            for k = 0:6
                row = k + 1;
                for n = k:order, Aeq_1d(row, n + 1) = prod(n - k + 1 : n) * (0)^(n - k); end
            end
            for k = 0:5
                row = 8 + k;
                for n = k:order, Aeq_1d(row, n + 1) = prod(n - k + 1 : n) * (1.0)^(n - k); end
            end
            Aeq = blkdiag(Aeq_1d, Aeq_1d, Aeq_1d);
            
            for axis = 1:3
                beq_axis = zeros(n_eq_1d, 1);
                for k = 0:6, beq_axis(k+1) = (T^k) * xd0(4*k + axis); end
                for k = 0:5, beq_axis(8+k) = (T^k) * xd1(4*k + axis); end
                beq( (axis-1)*n_eq_1d + 1 : axis*n_eq_1d ) = beq_axis;
            end
            
            % 4. 微分平坦性を用いた 2点バリア不等式制約 (Corridor)
            % ==== 制約の緩和 ====
            % 手前や奥では壁の延長線上に引っかかる可能性があるため、
            % 障害物の真横を通過する時間帯 (u = 0.3 ~ 0.7) にのみ Corridor 制約を課す。
            % また、確実に避けるためのガイドとして u=0.5 での経由点 p_via をソフト制約で H に足す。
            
            p_via = obj.obs_center + n_wall * (R_clear + 0.2);
            u_mid = 0.5;
            T0_mid = zeros(1, n_coeffs); for n=0:order, T0_mid(n+1)=u_mid^n; end
            w_via = 1e4;
            for axis = 1:3
                idx_s = (axis-1)*n_coeffs + 1; idx_e = axis*n_coeffs;
                H(idx_s:idx_e, idx_s:idx_e) = H(idx_s:idx_e, idx_s:idx_e) + w_via * (T0_mid' * T0_mid);
                f(idx_s:idx_e) = f(idx_s:idx_e) - w_via * p_via(axis) * T0_mid';
            end

            N_samples = 15; % 軌道上のサンプル点
            % 障害物付近のサンプル点 (m = 5~11 つまり u=0.3 ~ 0.7 付近) にのみ壁制約を課す
            valid_m = 5:11;
            A_ineq_all = zeros(2 * length(valid_m), num_vars);
            b_ineq_all = zeros(2 * length(valid_m), 1);
            
            row_idx = 1;
            for m = valid_m
                u_m = m / (N_samples + 1);
                
                % T_0 (位置) と T_2 (加速度) の基底ベクトル
                T0_vec = zeros(1, n_coeffs);
                T2_vec = zeros(1, n_coeffs);
                for n = 0:order
                    T0_vec(n+1) = u_m^n;
                    if n >= 2
                        T2_vec(n+1) = (n * (n - 1)) * u_m^(n - 2) / (T^2); % T^2で実時間微分にスケール
                    end
                end
                
                % ブロック行列化 (X, Y, Z軸)
                T0_blk = [T0_vec, zeros(1, n_coeffs), zeros(1, n_coeffs);
                          zeros(1, n_coeffs), T0_vec, zeros(1, n_coeffs);
                          zeros(1, n_coeffs), zeros(1, n_coeffs), T0_vec];
                          
                T2_blk = [T2_vec, zeros(1, n_coeffs), zeros(1, n_coeffs);
                          zeros(1, n_coeffs), T2_vec, zeros(1, n_coeffs);
                          zeros(1, n_coeffs), zeros(1, n_coeffs), T2_vec];
                
                % --- (1) 荷物(Load)の保護 ---
                % a_ineq^T * P_load <= b_ineq
                A_ineq_all(row_idx, :) = a_ineq' * T0_blk;
                b_ineq_all(row_idx)    = b_ineq;
                row_idx = row_idx + 1;
                
                % --- (2) ドローン(Drone)の保護 (微分平坦性) ---
                % P_drone = P_load + (L/g) * ddot{P}_load + [0;0;L]
                % a_ineq^T * P_drone <= b_ineq
                % a_ineq^T (P_load + (L/g) * ddot{P}_load) <= b_ineq - a_ineq^T * [0;0;L]
                
                L_g = obj.L_cable / obj.gravity;
                A_drone = a_ineq' * (T0_blk + L_g * T2_blk);
                b_drone = b_ineq - a_ineq' * [0; 0; obj.L_cable];
                
                A_ineq_all(row_idx, :) = A_drone;
                b_ineq_all(row_idx)    = b_drone;
                row_idx = row_idx + 1;
            end
            
            % 5. quadprog で解く
            opts = optimoptions('quadprog', 'Display', 'off');
            [C_opt, ~, exitflag] = quadprog(H, f, A_ineq_all, b_ineq_all, Aeq, beq, [], [], [], opts);
            
            if exitflag < 1
                fprintf("[MELLINGER FLATNESS] QPの実行可能解が見つかりません。安全のため解析解(大回り)にフォールバックします。\n");
                % 14元連立方程式 (ベースラインの解析解) によるフォールバック
                A_fb = zeros(14, 14); Bx_fb = zeros(14, 3);
                for k = 0:6
                    for n = k:order, A_fb(k+1, n+1) = prod(n-k+1:n) * 0^(n-k); end
                    Bx_fb(k+1, :) = (T^k) * xd0(4*k + (1:3))';
                end
                for n = 0:order, A_fb(8, n+1) = u_mid^n; end
                Bx_fb(8, :) = p_via';
                for k = 0:5
                    for n = k:order, A_fb(9+k, n+1) = prod(n-k+1:n) * 1.0^(n-k); end
                    Bx_fb(9+k, :) = (T^k) * xd1(4*k + (1:3))';
                end
                C_fb = A_fb \ Bx_fb;
                C_opt = [C_fb(:, 1); C_fb(:, 2); C_fb(:, 3)];
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




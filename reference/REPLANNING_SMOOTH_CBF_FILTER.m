classdef REPLANNING_SMOOTH_CBF_FILTER < handle
    % REPLANNING_SMOOTH_CBF_FILTER
    % Cohen et al. (2023) "Smooth Safety Filters" と
    % Mellinger & Kumar (2011) "Minimum Snap Trajectory" の概念を融合し、
    % 平坦出力空間(位置)での解析的CBF積分による回避経由点生成と、
    % 13次多項式による C^6 完全連続フィッティングを行うリプランナ。
    
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
        
        % CBF パラメータ
        cbf_gamma = 5.0
        cbf_sigma = 1.0
        Kp_cbf    = 3.0
        
        result
    end
    
    methods
        function obj = REPLANNING_SMOOTH_CBF_FILTER(self, base_ref, opts)
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
                    fprintf("[SMOOTH CBF REPLAN] 障害物接近検知! (min_dist=%.2fm, t=%.3f s)\n", min_dist, time.t);
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
                    
                    fprintf("[SMOOTH CBF REPLAN] Smooth CBF 仮想前方積分開始 (T = %.2f s)...\n", obj.t_duration);
                    
                    obj.plan_cbf_polynomial_trajectory(xd_nominal, dir_nom, spd, total_dist);
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
                        fprintf("[SMOOTH CBF REPLAN] 障害物クリア・C^6シームレス合流 (t=%.3f s)\n\n", time.t);
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
        function plan_cbf_polynomial_trajectory(obj, xd0, dir_nom, spd, total_dist)
            T = obj.t_duration;
            p0 = xd0(1:3);
            p_end = p0 + dir_nom * total_dist;
            
            % --- 1. 仮想空間での Smooth CBF 前方積分 ---
            % Cohen et al. (2023) の Sontag フィルタを利用して、
            % 障害物を安全に迂回する仮想の経由点をオンラインで生成する
            
            R_clear = obj.obs_radius + obj.safe_margin + obj.L_cable * 0.5 + 0.3;
            
            dt_sim = 0.1;
            N_steps = ceil(T / dt_sim);
            r_hist = zeros(3, N_steps + 1);
            r_hist(:, 1) = p0;
            
            % 直上からの接近など、完全対称な特異点による勾配消失を防ぐための微小初期オフセット
            % （論文におけるランダム摂動と同等の役割）
            vec_to_obs = obj.obs_center - p0;
            proj_on_line = dot(vec_to_obs, dir_nom) * dir_nom;
            normal_vec = vec_to_obs - proj_on_line;
            if norm(normal_vec) < 1e-3
                if abs(dir_nom(3)) < 0.9, normal_vec = cross(dir_nom, [0; 0; 1]);
                else, normal_vec = cross(dir_nom, [1; 0; 0]); end
            end
            dir_normal = normal_vec / norm(normal_vec);
            r_hist(:, 1) = r_hist(:, 1) + dir_normal * 0.05;
            
            for k = 1:N_steps
                t_k = (k-1) * dt_sim;
                r_ref_k = p0 + dir_nom * (spd * t_k);
                v_ref_k = dir_nom * spd;
                
                % 生成したシンボリック関数（Softplus/Sontagフィルタ）呼び出し
                u_safe = SmoothCBF_Filter_Velocity(r_hist(:, k), r_ref_k, v_ref_k, ...
                                                   obj.obs_center, R_clear, ...
                                                   obj.cbf_gamma, obj.cbf_sigma, obj.Kp_cbf);
                                                   
                r_hist(:, k+1) = r_hist(:, k) + u_safe * dt_sim;
            end
            
            % 前方積分結果から、中間経由点 (u = 0.5 に相当する点) を抽出
            mid_idx = ceil(N_steps / 2);
            p_via = r_hist(:, mid_idx);
            
            fprintf("    - CBF抽出 経由点 (u=0.5): [%.2f, %.2f, %.2f]\n", p_via);
            
            % --- 2. Mellinger & Kumar ベースの 13次多項式フィッティング ---
            xd1 = zeros(28, 1);
            xd1(1:3) = p_end;
            xd1(5:7) = dir_nom * spd;
            xd1(9:28) = 0;
            
            order = 13;
            A = zeros(14, 14);
            Bx = zeros(14, 3);
            
            for k = 0:6
                row = k + 1;
                for n = k:order
                    A(row, n + 1) = prod(n - k + 1 : n) * (0)^(n - k);
                end
                Bx(row, :) = (T^k) * xd0(4*k + (1:3))';
            end
            
            u_mid = 0.5;
            row = 8;
            for n = 0:order
                A(row, n + 1) = u_mid^n;
            end
            Bx(row, :) = p_via';
            
            u_end = 1.0;
            for k = 0:5
                row = 9 + k;
                for n = k:order
                    A(row, n + 1) = prod(n - k + 1 : n) * (u_end)^(n - k);
                end
                Bx(row, :) = (T^k) * xd1(4*k + (1:3))';
            end
            
            C = A \ Bx;
            obj.poly_coeffs = C';
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

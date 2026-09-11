classdef REPLANNING_LATENTCBF_BSPLINE < handle
    % REPLANNING_LATENTCBF_BSPLINE
    % 案5: Nakamura et al. (2025) [LatentCBF] + Teach-Repeat-Replan (2020)
    % 障害物領域をリプシッツ連続な境界（Margin function）として定義し、
    % Softplus と特異点平滑化を組み合わせた勾配ペナルティ付きバリア関数を用いて、
    % 7次 B-spline 制御点の Elastic Band 局所最適化を滑らかに行うリプランナ。
    
    properties
        base_ref
        self
        replan_active = false
        replan_done   = false
        
        t_start
        t_duration = 5.0
        
        obs_center   = [0; 0; 6.0]
        obs_radius   = 0.3
        safe_margin  = 0.4
        trigger_dist = 3.5
        
        p = 7;
        num_free_pts = 7;
        n;
        U;
        Q;
        
        L_cable = 2.0;
        
        N_sample = 50;
        M_sample; 
        
        % LatentCBF 風の平滑化パラメータ
        beta_softplus = 10.0; % Softplusの鋭さ
        eps_smooth    = 1e-2; % 距離関数の中心特異点除去（リプシッツ定数制限）
        
        result
    end
    
    methods
        function obj = REPLANNING_LATENTCBF_BSPLINE(self, base_ref, opts)
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
            if isfield(opts, "num_free_pts"), obj.num_free_pts = opts.num_free_pts;    end
            
            obj.n = 2 * obj.p + obj.num_free_pts - 1; 
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
                    fprintf("[LATENT CBF REPLAN] 障害物接近検知! (min_dist=%.2fm, t=%.3f s)\n", min_dist, time.t);
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
                    
                    fprintf("[LATENT CBF REPLAN] 平滑化バリア最適化開始 (T = %.2f s)...\n", obj.t_duration);
                    
                    state_start = zeros(3, 7); 
                    state_start(:, 1) = xd_nominal(1:3);
                    state_start(:, 2) = xd_nominal(5:7);
                    
                    t_end = obj.t_start + obj.t_duration;
                    p_end = pL_cur + dir_nom * total_dist;
                    state_end = zeros(3, 7);
                    state_end(:, 1) = p_end;
                    state_end(:, 2) = dir_nom * spd;
                    
                    obj.plan_bspline_trajectory(state_start, state_end);
                    obj.replan_active = true;
                    fprintf("=======================================================\n\n");
                end
            end
            
            if obj.replan_active
                tau = time.t - obj.t_start;
                
                if tau <= obj.t_duration
                    xd = obj.evaluate_bspline_trajectory(tau);
                else
                    fprintf("[LATENT CBF REPLAN] 障害物クリア・公称軌道へ合流 (t=%.3f s)\n\n", time.t);
                    obj.replan_active = false;
                    obj.replan_done   = true;
                    xd = xd_nominal;
                end
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
        
        function plan_bspline_trajectory(obj, state_start, state_end)
            S = obj.n - obj.p + 1;
            obj.U = zeros(1, obj.n + obj.p + 2);
            obj.U(1 : obj.p+1) = 0;
            obj.U(end-obj.p : end) = obj.t_duration;
            for i = 1:S-1
                obj.U(obj.p + 1 + i) = i * (obj.t_duration / S);
            end
            
            dt_knot = obj.t_duration / S;
            obj.Q = zeros(3, obj.n + 1);
            
            % 境界制御点を等速直線外挿で安定して配置
            for k = 0:obj.p-1
                obj.Q(:, k+1) = state_start(:, 1) + state_start(:, 2) * (k * dt_knot / obj.p);
                obj.Q(:, obj.n - k + 1) = state_end(:, 1) - state_end(:, 2) * (k * dt_knot / obj.p);
            end
            
            start_free = obj.p;         
            end_free   = obj.n - obj.p; 
            
            % 中間制御点の初期化（直線補間）
            for i = start_free : end_free
                ratio = (i - start_free + 1) / (end_free - start_free + 2);
                obj.Q(:, i+1) = (1 - ratio) * obj.Q(:, start_free) + ratio * obj.Q(:, end_free+2);
                % 勾配消失特異点を防ぐための微小初期ノイズ
                obj.Q(1, i+1) = obj.Q(1, i+1) + 1e-3;
            end
            
            % --- トンネリング対策のための軌道評価用重み行列 M_sample の事前計算 ---
            obj.M_sample = zeros(obj.n + 1, obj.N_sample);
            t_samples = linspace(0, obj.t_duration, obj.N_sample);
            for j = 0 : obj.n
                dummy_Q = zeros(3, obj.n + 1);
                dummy_Q(1, j+1) = 1.0;
                for k = 1 : obj.N_sample
                    t_eval = max(0, min(obj.t_duration - 1e-6, t_samples(k)));
                    k_span = obj.p + 1;
                    for i_knot = obj.p+1 : obj.n
                        if t_eval >= obj.U(i_knot) && t_eval < obj.U(i_knot+1)
                            k_span = i_knot;
                            break;
                        end
                    end
                    pt = obj.evaluate_derivative_internal(t_eval, k_span, 0, dummy_Q);
                    obj.M_sample(j+1, k) = pt(1);
                end
            end
            % -----------------------------------------------------------------
            
            x0 = reshape(obj.Q(:, start_free+1 : end_free+1), [], 1);
            
            [J_init, J_sm, J_ob] = obj.objective_function_debug(x0, start_free, end_free);
            fprintf("\n    [DEBUG] 初期コスト: Total=%.4f (Smooth=%.4f, LatentObs=%.4f)\n", J_init, J_sm, J_ob);
            
            fun = @(x) obj.objective_function(x, start_free, end_free);
            
            opts = optimoptions('fminunc', 'Algorithm', 'quasi-newton', ...
                'Display', 'iter', 'MaxIterations', 300, 'MaxFunctionEvaluations', 20000, ...
                'OptimalityTolerance', 1e-4, 'StepTolerance', 1e-4);
            
            try
                tic;
                [x_opt, fval, exitflag, output] = fminunc(fun, x0, opts);
                t_opt = toc;
                fprintf("    [DEBUG] 最適化完了: %.1f ms (ExitFlag: %d, Iters: %d)\n", t_opt * 1000, exitflag, output.iterations);
            catch ME
                fprintf("    [DEBUG] fminunc failed: %s\n", ME.message);
                opts_search = optimset('Display', 'iter', 'MaxIter', 400, 'MaxFunEvals', 20000);
                [x_opt, fval, exitflag, output] = fminsearch(fun, x0, opts_search);
            end
            
            [J_opt, J_sm_opt, J_ob_opt] = obj.objective_function_debug(x_opt, start_free, end_free);
            fprintf("    [DEBUG] 最終コスト: Total=%.4f (Smooth=%.4f, LatentObs=%.4f)\n\n", J_opt, J_sm_opt, J_ob_opt);
            
            obj.Q(:, start_free+1 : end_free+1) = reshape(x_opt, 3, []);
        end
        
        function J = objective_function(obj, x, start_free, end_free)
            Q_temp = obj.Q;
            Q_temp(:, start_free+1 : end_free+1) = reshape(x, 3, []);
            
            % 1. ジャーク最小化による滑らかさコスト (3階差分)
            w_smooth = 1.0;
            J_smooth = 0;
            for i = 1 : (obj.n + 1 - 3)
                diff3 = Q_temp(:, i+3) - 3*Q_temp(:, i+2) + 3*Q_temp(:, i+1) - Q_temp(:, i);
                J_smooth = J_smooth + sum(diff3.^2);
            end
            
            % 2. LatentCBF 風の勾配ペナルティ付き平滑化バリア関数コスト
            % 従来の ReLU(d_safe - dist)^3 の代わりに、全域で C^infty であり、
            % リプシッツ連続性を保証する Softplus マージン関数を用いる
            w_obs = 5000.0;
            J_obs = 0;
            d_safe = obj.obs_radius + obj.safe_margin;
            
            P_samples = Q_temp * obj.M_sample;
            
            for k = 1 : obj.N_sample
                % 荷物のペナルティ
                dist_load_sq = norm(P_samples(:, k) - obj.obs_center)^2;
                % 特異点除去（微小なepsを加えて原点での微分不可能をなくす）
                dist_load = sqrt(dist_load_sq + obj.eps_smooth^2);
                h_load = dist_load - d_safe; 
                % h < 0 (侵入) のときペナルティが線形増加し、勾配が有界
                J_obs = J_obs + (1 / obj.beta_softplus) * log(1 + exp(-obj.beta_softplus * h_load));
                
                % ドローン本体のペナルティ
                dist_drone_sq = norm(P_samples(:, k) + [0; 0; obj.L_cable] - obj.obs_center)^2;
                dist_drone = sqrt(dist_drone_sq + obj.eps_smooth^2);
                h_drone = dist_drone - d_safe;
                J_obs = J_obs + (1 / obj.beta_softplus) * log(1 + exp(-obj.beta_softplus * h_drone));
            end
            
            J = w_smooth * J_smooth + w_obs * J_obs;
        end
        
        function [J, J_smooth, J_obs] = objective_function_debug(obj, x, start_free, end_free)
            Q_temp = obj.Q;
            Q_temp(:, start_free+1 : end_free+1) = reshape(x, 3, []);
            
            w_smooth = 1.0;
            J_smooth = 0;
            for i = 1 : (obj.n + 1 - 3)
                diff3 = Q_temp(:, i+3) - 3*Q_temp(:, i+2) + 3*Q_temp(:, i+1) - Q_temp(:, i);
                J_smooth = J_smooth + sum(diff3.^2);
            end
            
            w_obs = 5000.0;
            J_obs = 0;
            d_safe = obj.obs_radius + obj.safe_margin;
            
            P_samples = Q_temp * obj.M_sample;
            
            for k = 1 : obj.N_sample
                dist_load_sq = norm(P_samples(:, k) - obj.obs_center)^2;
                dist_load = sqrt(dist_load_sq + obj.eps_smooth^2);
                h_load = dist_load - d_safe;
                J_obs = J_obs + (1 / obj.beta_softplus) * log(1 + exp(-obj.beta_softplus * h_load));
                
                dist_drone_sq = norm(P_samples(:, k) + [0; 0; obj.L_cable] - obj.obs_center)^2;
                dist_drone = sqrt(dist_drone_sq + obj.eps_smooth^2);
                h_drone = dist_drone - d_safe;
                J_obs = J_obs + (1 / obj.beta_softplus) * log(1 + exp(-obj.beta_softplus * h_drone));
            end
            
            J_smooth = w_smooth * J_smooth;
            J_obs = w_obs * J_obs;
            J = J_smooth + J_obs;
        end
        
        function xd = evaluate_bspline_trajectory(obj, t)
            xd = zeros(28, 1);
            t = max(0, min(obj.t_duration - 1e-6, t)); 
            
            k_span = obj.p + 1;
            for i = obj.p+1 : obj.n
                if t >= obj.U(i) && t < obj.U(i+1)
                    k_span = i;
                    break;
                end
            end
            
            for d = 0:6
                pt_d = obj.evaluate_derivative_internal(t, k_span, d, obj.Q);
                idx = 4 * d + 1;
                xd(idx : idx+2) = pt_d;
                xd(idx+3) = 0; 
            end
        end
        
        function pt = evaluate_derivative_internal(obj, t, k_span, d, Q_in)
            p_cur = obj.p - d;
            Q_d = zeros(3, p_cur + 1);
            
            for i = 0 : p_cur
                ctrl_idx = k_span - p_cur + i;
                Q_d(:, i+1) = obj.get_derivative_control_point(ctrl_idx, d, Q_in);
            end
            
            for r = 1 : p_cur
                for i = p_cur : -1 : r
                    knot_idx = k_span - p_cur + i;
                    denom = obj.U(knot_idx + p_cur - r + d + 1) - obj.U(knot_idx + d);
                    if denom == 0
                        alpha = 0;
                    else
                        alpha = (t - obj.U(knot_idx + d)) / denom;
                    end
                    Q_d(:, i+1) = (1 - alpha) * Q_d(:, i) + alpha * Q_d(:, i+1);
                end
            end
            pt = Q_d(:, p_cur + 1);
        end
        
        function Qd = get_derivative_control_point(obj, idx, d, Q_in)
            if d == 0
                Qd = Q_in(:, idx);
            else
                Qd_prev_i   = obj.get_derivative_control_point(idx, d-1, Q_in);
                Qd_prev_im1 = obj.get_derivative_control_point(idx-1, d-1, Q_in);
                denom = obj.U(idx + obj.p + 1) - obj.U(idx + d);
                if denom == 0
                    Qd = zeros(3,1);
                else
                    Qd = (obj.p - d + 1) / denom * (Qd_prev_i - Qd_prev_im1);
                end
            end
        end
        
    end
end

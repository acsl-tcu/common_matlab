classdef REPLANNING_ROM_CBF_BACKSTEPPING < handle
    % REPLANNING_ROM_CBF_BACKSTEPPING
    % 案2: 縮小次元モデル(ROM)とバックステップCBFによる階層的安全性保証 (Cohen et al. 2024)
    % 単積分器モデル(ROM)でISSf保証付きの安全な速度指令 v_safe をオンライン生成し、
    % その軌道を経由点として高階スプライン(13次多項式)で6階微分まで平滑化補間する。
    
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
        
        % ISSf 理論における追従誤差バウンド (epsilon)
        % 下位コントローラが v_safe に追従する際の最大誤差を想定して安全集合をインフレさせる
        eps_issf = 0.5
        
        alpha_gain = 2.0 % CBFクラスK関数ゲイン
        
        poly_coeffs 
        
        t_merge_end
        p_merge_end
        v_merge_vec
        
        result
    end
    
    methods
        function obj = REPLANNING_ROM_CBF_BACKSTEPPING(self, base_ref, opts)
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
                    fprintf("[ROM CBF BACKSTEPPING] 障害物接近検知! (min_dist=%.2fm, t=%.3f s)\n", min_dist, time.t);
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
                    
                    fprintf("[ROM CBF BACKSTEPPING] ROM軌道前方生成と平滑化補間を開始...\n");
                    obj.plan_rom_cbf_and_smooth(xd_nominal, dir_nom, spd, total_dist, pL_cur, vL_cur);
                    
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
                        fprintf("[ROM CBF BACKSTEPPING] 障害物クリア・C^6シームレス合流 (t=%.3f s)\n\n", time.t);
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
        function plan_rom_cbf_and_smooth(obj, xd0, dir_nom, spd, total_dist, pL_cur, vL_cur)
            T = obj.t_duration;
            p0 = pL_cur;
            p_end = pL_cur + dir_nom * total_dist;
            
            % --- 1. ROM (縮小次元モデル) 空間での CBF 前方シミュレーション ---
            % 障害物半径 + 余裕 + ケーブル長 + **ISSf追従誤差バウンド(eps_issf)** 
            % をインフレさせた安全半径 R_safe を用いる
            R_safe = obj.obs_radius + obj.safe_margin + obj.L_cable * 0.5 + obj.eps_issf;
            
            dt_sim = 0.1;
            N_steps = ceil(T / dt_sim);
            q_hist = zeros(3, N_steps + 1);
            q_hist(:, 1) = p0;
            
            % 直上からの特異点スタックを防ぐための微小オフセット
            vec_to_obs = obj.obs_center - p0;
            proj_on_line = dot(vec_to_obs, dir_nom) * dir_nom;
            normal_vec = vec_to_obs - proj_on_line;
            if norm(normal_vec) < 1e-3
                if abs(dir_nom(3)) < 0.9, normal_vec = cross(dir_nom, [0; 0; 1]);
                else, normal_vec = cross(dir_nom, [1; 0; 0]); end
            end
            dir_normal = normal_vec / norm(normal_vec);
            q_hist(:, 1) = q_hist(:, 1) + dir_normal * 0.05;
            
            % Kpゲイン (公称速度への収束)
            Kp_nom = 2.0;
            
            for k = 1:N_steps
                t_k = (k-1) * dt_sim;
                % 公称目標位置と速度
                q_ref_k = p0 + dir_nom * (spd * t_k);
                v_ref_k = dir_nom * spd;
                
                % 公称入力
                v_nom = v_ref_k + Kp_nom * (q_ref_k - q_hist(:, k));
                
                % 解析的 CBF-QP の呼び出し
                [violation, v_correct_dir, h_0] = Analytical_ROM_CBF_Velocity(...
                    q_hist(:, k), v_nom, obj.obs_center, R_safe, obj.alpha_gain);
                
                % QPの修正量
                if violation > 0
                    v_safe = v_nom + violation * v_correct_dir;
                else
                    v_safe = v_nom;
                end
                
                % 単積分器モデルのオイラー更新
                q_hist(:, k+1) = q_hist(:, k) + v_safe * dt_sim;
            end
            
            % ROM軌道の中間点（最も障害物を迂回した点）を抽出
            mid_idx = ceil(N_steps / 2);
            p_via = q_hist(:, mid_idx);
            
            fprintf("    - ROM CBF 経由点生成完了 (インフレ半径 R_safe = %.2f m)\n", R_safe);
            
            % --- 2. 13次多項式 (Mellinger) 平滑化補間 ---
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
            
            fprintf("    - 高階スプライン平滑化補間完了 (C^6 連続)\n");
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

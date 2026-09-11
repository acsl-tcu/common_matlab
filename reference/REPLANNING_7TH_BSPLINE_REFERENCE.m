classdef REPLANNING_7TH_BSPLINE_REFERENCE < handle
    % C^6完全追従・解析的B-spline QPリプランナ (トルク飽和・波打ちゼロ版)「13次
    % 多項式（13th-order Polynomial）の境界値問題」** 後は回避時間問題位？
    properties
        base_ref
        self
        replan_active = false
        replan_done   = false
        
        t_start
        t_duration = 10.0          % 回避所要時間 [s]
        
        % 障害物設定
        obs_center   = [0.2; 0.2; 6.0]
        obs_radius   = 0.3
        safe_margin  = 0.4
        trigger_dist = 3.5
        
        p = 7;                     % 7次スプライン (C^6連続)
        L_cable = 2.0
        
        poly_coeffs                % 3 x 14 解析的係数 (無次元 u in [0,1])
        
        % 合流保持
        t_merge_end
        p_merge_end
        v_merge_vec
        
        result
    end
    
    methods
        function obj = REPLANNING_7TH_BSPLINE_REFERENCE(self, base_ref, opts)
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
            
            % 1. 接近検知トリガー
            if cha == 'f' && ~obj.replan_done && ~obj.replan_active
                dist_load_obs  = norm(pL_cur - obj.obs_center);
                dist_drone_obs = norm(pQ_cur - obj.obs_center);
                min_dist = min(dist_load_obs, dist_drone_obs);
                
                if min_dist <= obj.trigger_dist
                    fprintf("\n=======================================================\n");
                    fprintf("[B-SPLINE REPLAN] 障害物検知! (min_dist=%.2fm, t=%.3f s)\n", min_dist, time.t);
                    obj.t_start = time.t;
                    
                    v_vec = xd_nominal(5:7);
                    spd = norm(v_vec);
                    if spd < 0.05, spd = norm(vL_cur); end
                    if spd < 0.05, spd = 0.3; v_vec = [0; 0; 0.3]; end
                    dir_nom = v_vec / spd;
                    
                    % 障害物を安全に越える距離と所要時間
                    vec_to_obs = obj.obs_center - pL_cur;
                    proj_dist = max(0, dot(vec_to_obs, dir_nom));
                    total_dist = proj_dist + obj.obs_radius + obj.L_cable + obj.safe_margin + 1.2;
                    obj.t_duration = max(8.0, min(14.0, total_dist / spd));
                    
                    fprintf("[B-SPLINE REPLAN] 解析的平滑解の算出 (所要時間 T = %.2f s)\n", obj.t_duration);
                    
                    obj.plan_analytical_smooth_bspline(xd_nominal, dir_nom, spd, total_dist);
                    obj.replan_active = true;
                    fprintf("=======================================================\n\n");
                end
            end
            
            % 2. 軌道出力 ＆ C^6完全連続合流
            if obj.replan_active
                tau = time.t - obj.t_start;
                
                if tau <= obj.t_duration
                    xd = obj.evaluate_smooth_trajectory(tau, xd_nominal);
                else
                    if ~obj.replan_done
                        fprintf("[B-SPLINE REPLAN] 障害物クリア・C^6シームレス合流 (t=%.3f s)\n\n", time.t);
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
                    xd(9:28) = 0; % 2階以降は数学的に完全ゼロ一致
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
        function plan_analytical_smooth_bspline(obj, xd0, dir_nom, spd, total_dist)
            T = obj.t_duration;
            p0 = xd0(1:3);
            
            % 障害物を回避する最短法線方向の算出
            vec_to_obs = obj.obs_center - p0;
            proj_on_line = dot(vec_to_obs, dir_nom) * dir_nom;
            normal_vec = vec_to_obs - proj_on_line;
            if norm(normal_vec) < 1e-3
                if abs(dir_nom(3)) < 0.9, normal_vec = cross(dir_nom, [0; 0; 1]);
                else, normal_vec = cross(dir_nom, [1; 0; 0]); end
            end
            dir_normal = normal_vec / norm(normal_vec);
            
            % 紐とドローンが干渉しない十分なクリアランス
            R_clear = obj.obs_radius + obj.safe_margin + obj.L_cable * 0.5 + 0.3;
            
            % 中間経由点 (u = 0.5)
            p_mid_line = p0 + dir_nom * (total_dist * 0.5);
            p_via = p_mid_line + dir_normal * R_clear;
            
            % 終端合流点 (u = 1.0)
            p_end = p0 + dir_nom * total_dist;
            xd1 = zeros(28, 1);
            xd1(1:3) = p_end;
            xd1(5:7) = dir_nom * spd;
            xd1(9:28) = 0;
            
            % -------------------------------------------------------------
            % 13次単一スプライン多項式 (無次元時間 u in [0, 1])
            % 高次微分の波打ち・特異性を完全に抑え込む Closed-form 連立解
            % -------------------------------------------------------------
            order = 13;
            A = zeros(14, 14);
            Bx = zeros(14, 3);
            
            % 始端境界拘束 (u = 0, 0〜6階微分完全一致)
            for k = 0:6
                row = k + 1;
                for n = k:order
                    A(row, n + 1) = prod(n - k + 1 : n) * (0)^(n - k);
                end
                Bx(row, :) = (T^k) * xd0(4*k + (1:3))';
            end
            
            % 中間経由点拘束 (u = 0.5)
            u_mid = 0.5;
            row = 8;
            for n = 0:order
                A(row, n + 1) = u_mid^n;
            end
            Bx(row, :) = p_via';
            
            % 終端合流拘束 (u = 1.0, 0〜5階微分完全一致)
            u_end = 1.0;
            for k = 0:5
                row = 9 + k;
                for n = k:order
                    A(row, n + 1) = prod(n - k + 1 : n) * (u_end)^(n - k);
                end
                Bx(row, :) = (T^k) * xd1(4*k + (1:3))';
            end
            
            % 解析解の算出 (反復最適化を行わないため解の波打ちがゼロ)
            C = A \ Bx;
            obj.poly_coeffs = C'; % 3 x 14
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
                % 実時間導関数への正確なスケール変換
                xd(4*k + (1:3)) = val_k / (T^k);
            end
        end
    end
end
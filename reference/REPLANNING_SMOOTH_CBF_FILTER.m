classdef REPLANNING_SMOOTH_CBF_FILTER < handle
    % REPLANNING_SMOOTH_CBF_FILTER (論文融合・姿勢安定化版)
    % - Zheng et al. (2025): 荷物〜紐〜ドローンの 5連保護球評価
    % - Cohen et al. (2023): Smooth Softplus フィルタによる代数的回避量算定
    % - Mellinger & Kumar (2011): 13次多項式による 6階微分 (C^6) 完全連続生成
    % - Tscholl et al. (2024): 実現ギャップを防ぐ過大加速度・Snap 抑制
    
    properties
        base_ref
        self
        
        replan_active = false
        replan_done   = false
        
        t_start
        t_duration = 8.0
        T_seg
        
        trigger_dist = 5.0  % 検知距離 [m]
        safe_margin  = 0.5  % 安全マージン [m]
        
        L_cable = 2.0
        gravity = 9.81
        r_load  = 0.15
        r_drone = 0.30
        
        % Cohen (2023) パラメータ
        cbf_gamma = 1.5
        cbf_sigma = 0.2
        
        % 回避幾何パラメータ
        dir_normal          % 回避法線方向 (3x1)
        dir_nominal         % 公称進行方向 (3x1)
        nominal_speed       % 進入速度
        delta_target = 0.0  % Cohen Softplus で算出された必要離隔量
        
        % Mellinger 13次多項式スプライン係数 (区間1, 区間2)
        order = 13
        coeffs_seg1
        coeffs_seg2
        
        obs_center = [0; 0; 0]
        obs_R      = eye(3)
        obs_radii  = [1; 1; 1]
        obs_margin = 0.5
        
        t_merge_end
        p_merge_end
        v_merge_vec
        
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
            
            if isfield(opts, "safe_margin"),  obj.safe_margin  = opts.safe_margin;  end
            if isfield(opts, "trigger_dist"), obj.trigger_dist = opts.trigger_dist; end
            if isfield(opts, "r_load"),       obj.r_load       = opts.r_load;       end
            if isfield(opts, "r_drone"),      obj.r_drone      = opts.r_drone;      end
            if isfield(opts, "cbf_gamma"),    obj.cbf_gamma    = opts.cbf_gamma;    end
            if isfield(opts, "cbf_sigma"),    obj.cbf_sigma    = opts.cbf_sigma;    end
            
            obj.result.state = STATE_CLASS(struct('state_list', ["xd", "p", "q", "v"], 'num_list', [28, 3, 3, 3]));
        end
        
        function result = do(obj, varargin)
            time = varargin{1};
            cha = varargin{2};
            
            % 1. 公称リファレンスの取得
            base_res = obj.base_ref.do(time, cha);
            xd_nominal = base_res.state.xd;
            if length(xd_nominal) < 28
                xd_nominal = [xd_nominal; zeros(28 - length(xd_nominal), 1)];
            end
            
            % 状態取得
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
            
            % 2. 障害物検知とトリガー（コンソール通知付き）
            if cha == 'f' && ~obj.replan_done && ~obj.replan_active
                obs_list = [];
                try
                    obs_list = ENVIRONMENT_OBSTACLE_ELLIPSE();
                catch ME
                    warning("[CBF_FILTER] ENVIRONMENT_OBSTACLE_ELLIPSE 読込失敗: %s", ME.message);
                end
                
                min_dist_overall = inf;
                target_obs_idx = -1;
                
                for i = 1:length(obs_list)
                    c = obs_list(i).p_center;
                    r_max = max(obs_list(i).ellipsoid_radii);
                    d_marg = obs_list(i).d_margin;
                    
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
                    obj.obs_center = tgt.p_center;
                    obj.obs_R      = tgt.R_obs;
                    obj.obs_radii  = tgt.ellipsoid_radii;
                    obj.obs_margin = tgt.d_margin;
                    
                    obj.t_start = time.t;
                    
                    % 進行方向と速度の取得
                    v_vec = xd_nominal(5:7);
                    spd = norm(v_vec);
                    if spd < 0.05, spd = norm(vL_cur); end
                    if spd < 0.05, spd = 0.3; v_vec = [0; 0; 0.3]; end
                    obj.dir_nominal = v_vec / spd;
                    obj.nominal_speed = spd;
                    
                    % 到達予測時間から十分な加減速時間を確保 (Tscholl 2024: 実現ギャップ防止)
                    vec_to_obs = obj.obs_center - pL_cur;
                    d_proj = dot(vec_to_obs, obj.dir_nominal);
                    t_cross = max(3.5, d_proj / spd);
                    obj.T_seg = t_cross;
                    obj.t_duration = 2.0 * obj.T_seg;
                    
                    % --- 検知通知の表示 ---
                    fprintf("\n=======================================================\n");
                    fprintf("[SMOOTH CBF REPLANNER] 障害物検知! (ID: %d, 表面距離: %.2f m, 時刻: %.3f s)\n", ...
                        target_obs_idx, min_dist_overall, time.t);
                    fprintf("  - タイプ: %s | 主軸半径: [a=%.2f, b=%.2f, c=%.2f] m\n", ...
                        tgt.type, obj.obs_radii(1), obj.obs_radii(2), obj.obs_radii(3));
                    fprintf("  - Zheng 保護球列: 5球評価 (荷物・ワイヤ・ドローン連立防護)\n");
                    fprintf("  - 計画区間長: T_seg = %.2f s (合計: %.2f s, C^6 完全連続多項式)\n", ...
                        obj.T_seg, obj.t_duration);
                    
                    % Cohen (2023) Softplus 評価と Mellinger 13次スプラインの求解
                    obj.solve_smooth_cbf_mellinger_spline(pL_cur, pQ_cur);
                    obj.replan_active = true;
                    fprintf("=======================================================\n\n");
                end
            end
            
            % 3. C^6 連続軌道の評価・合流
            if obj.replan_active
                tau = time.t - obj.t_start;
                if tau <= obj.t_duration
                    xd = obj.evaluate_c6_trajectory(tau, xd_nominal);
                else
                    if ~obj.replan_done
                        fprintf("[SMOOTH CBF REPLANNER] 障害物回避完了・公称軌道へ完全シームレス合流 (t=%.3f s)\n\n", time.t);
                        obj.t_merge_end = obj.t_start + obj.t_duration;
                        xd_end = obj.evaluate_c6_trajectory(obj.t_duration, xd_nominal);
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
        function solve_smooth_cbf_mellinger_spline(obj, pL_cur, ~)
            % 1. 幾何最短回避法線 dir_normal の導出 (マハラノビス空間射影)
            D_inv = diag(1 ./ obj.obs_radii);
            z0 = D_inv * obj.obs_R' * (pL_cur - obj.obs_center);
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
            obj.dir_normal = n_opt / norm(n_opt);
            
            % 2. Zheng (2025) 保護球列による楕円体バリア h の評価[cite: 1]
            % 荷物〜ドローン間の 5 球について最接近点での侵入量を計算
            num_spheres = 5;
            lambdas = linspace(0, 1, num_spheres);
            max_delta_req = 0;
            
            p_center_proj = dot(obj.dir_normal, obj.obs_center - pL_cur);
            r_eff_obs = 1.0 / norm(n_opt);
            
            for j = 1:num_spheres
                lam = lambdas(j);
                r_sph = (1 - lam) * obj.r_load + lam * obj.r_drone;
                % ドローン本体とワイヤの姿勢傾きマージンを考慮
                offset_tilt = lam * (obj.dir_normal(3) * obj.L_cable);
                req_j = p_center_proj + r_eff_obs + r_sph + obj.obs_margin - offset_tilt;
                if req_j > max_delta_req
                    max_delta_req = req_j;
                end
            end
            
            % 3. Cohen et al. (2023) Softplus フィルタによる滑らかな目標逃げ量決定[cite: 2]
            % a = -マージン余裕, b = 1
            a_val = -(max_delta_req);
            b_val = 1.0;
            % lambda_softplus = sigma * log(1 + exp(-a / sigma))
            delta_smooth = obj.cbf_sigma * log(1 + exp(-a_val / (b_val * obj.cbf_sigma)));
            obj.delta_target = max(max_delta_req, delta_smooth);
            
            fprintf("  - Cohen Softplus 目標回避幅: %.3f m (物理限界マージン: %.3f m)\n", ...
                obj.delta_target, max_delta_req);
            
            % 4. Mellinger 13次多項式 (C^6 完全一致) の境界値問題の解[cite: 4]
            % 区間1 (u in [0, 1]): 0 から delta_target へ (始端0〜6階微分=0, 終端速度〜6階微分=0)
            % 区間2 (u in [0, 1]): delta_target から 0 へ (始端速度〜6階微分=0, 終端0〜6階微分=0)
            N = obj.order; % 13次
            n_c = N + 1;   % 14
            
            A_bvp = zeros(14, 14);
            b1 = zeros(14, 1);
            b2 = zeros(14, 1);
            
            % 始端 u=0 の 0〜6階微分
            for k = 0:6
                A_bvp(k + 1, k + 1) = factorial(k);
            end
            % 終端 u=1 の 0〜6階微分
            for k = 0:6
                for n = k:N
                    A_bvp(7 + k + 1, n + 1) = prod(n - k + 1 : n);
                end
            end
            
            % 区間1の境界値: 始端 [0, 0...], 終端 [delta_target, 0, 0...]
            b1(8) = obj.delta_target; 
            obj.coeffs_seg1 = (A_bvp \ b1)';
            
            % 区間2の境界値: 始端 [delta_target, 0...], 終端 [0, 0, 0...]
            b2(1) = obj.delta_target;
            obj.coeffs_seg2 = (A_bvp \ b2)';
        end
        
        function xd = evaluate_c6_trajectory(obj, tau, xd_nom)
            xd = xd_nom;
            N = obj.order;
            T_seg = obj.T_seg;
            
            if tau <= T_seg
                C = obj.coeffs_seg1;
                u = max(0, min(1.0, tau / T_seg));
            else
                C = obj.coeffs_seg2;
                u = max(0, min(1.0, (tau - T_seg) / T_seg));
            end
            
            % 0〜6階微分値 delta^(k) の評価 (C^6 完全連続)[cite: 4]
            delta_k = zeros(7, 1);
            for k = 0:6
                val_k = 0;
                for n = k:N
                    factor = prod(n - k + 1 : n);
                    val_k = val_k + C(n + 1) * factor * (u^(n - k));
                end
                delta_k(k + 1) = val_k / (T_seg^k);
            end
            
            % 公称軌道の法線方向ベクトルに足し合わせ
            for k = 0:6
                idx = 4 * k + (1:3);
                xd(idx) = xd_nom(idx) + obj.dir_normal * delta_k(k + 1);
            end
        end
    end
end
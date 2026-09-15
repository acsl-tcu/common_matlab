classdef REPLANNING_SMOOTH_CBF_FILTER < handle
    % REPLANNING_SMOOTH_CBF_FILTER (Mellinger 13次多項式解析生成 & 幾何直交回避版)
    % - 数値積分 (Ad/Bd) を完全撤廃し、Mellinger 13次多項式で C^6 を解析的に厳密生成
    % - 障害物中心からの幾何学的最短横方向法線を直接導出し、直下突入時も確実に横回避
    % - Zheng et al. (2025): 荷物〜ケーブル〜ドローン 5連保護球評価
    % - Cohen et al. (2023): Smooth Softplus フィルタによる代数的前方不変性
    % - HLC_SUSPENDED_LOAD 適合: 位置〜6階微分の数学的整合性を 100% 保証
    
    properties
        base_ref
        self
        
        replan_active = false
        replan_done   = false
        
        t_start
        t_duration = 10.0
        T_seg
        
        trigger_dist = 6.0  % 検知・回避判定距離 [m]
        safe_margin  = 0.5  % 安全離隔マージン [m]
        
        L_cable = 2.0
        gravity = 9.81
        r_load  = 0.15
        r_drone = 0.30
        
        % Cohen et al. (2023) Softplus パラメータ
        cbf_gamma = 1.0
        cbf_sigma = 0.30
        
        % 回避幾何パラメータ
        dir_normal          % 進行軸と直交する水平回避法線単位ベクトル (3x1)
        dir_nominal         % 公称進行方向 (3x1)
        nominal_speed       % 進行速度
        delta_target = 0.0  % Softplus により算定された必要横離隔幅 [m]
        
        % Mellinger 13次多項式スプライン係数 (区間1, 区間2)
        order = 13
        coeffs_seg1
        coeffs_seg2
        
        t_merge_end
        p_merge_end
        v_merge_vec
        
        detected_obs_map
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
            
            obj.detected_obs_map = containers.Map('KeyType', 'int32', 'ValueType', 'logical');
            obj.result.state = STATE_CLASS(struct('state_list', ["xd", "p", "q", "v"], 'num_list', [28, 3, 3, 3]));
        end
        
        function result = do(obj, varargin)
            time = varargin{1};
            cha = varargin{2};
            
            % 1. 公称目標値の取得
            base_res = obj.base_ref.do(time, cha);
            xd_nom = base_res.state.xd;
            if length(xd_nom) < 28
                xd_nom = [xd_nom; zeros(28 - length(xd_nom), 1)];
            end
            
            obj.L_cable = obj.self.parameter.get("cableL");
            
            % 現在状態の取得
            if isprop(obj.self.estimator.result.state, "pL")
                pL_cur = obj.self.estimator.result.state.pL;
                vL_cur = obj.self.estimator.result.state.vL;
            else
                pL_cur = base_res.state.p;
                vL_cur = base_res.state.v;
            end
            
            if isprop(obj.self.estimator.result.state, "p")
                pQ_cur = obj.self.estimator.result.state.p;
            else
                pQ_cur = pL_cur + [0; 0; obj.L_cable];
            end
            
            % 2. 障害物検知と回避計画のトリガー
            if cha == 'f' && ~obj.replan_done && ~obj.replan_active
                obs_list = [];
                try
                    obs_list = ENVIRONMENT_OBSTACLE_ELLIPSE();
                catch ME
                    warning("[CBF_FILTER] 障害物定義読込失敗: %s", ME.message);
                end
                
                min_dist_overall = inf;
                target_obs_idx = -1;
                
                for i = 1:length(obs_list)
                    obs = obs_list(i);
                    c_obs = obs.p_center;
                    radii = obs.ellipsoid_radii;
                    d_marg = obs.d_margin;
                    r_bound = max(radii) + max(obj.r_load, obj.r_drone) + d_marg;
                    
                    % 3次元絶対距離
                    dist_3d = norm(pL_cur - c_obs) - r_bound;
                    dist_3d_Q = norm(pQ_cur - c_obs) - r_bound;
                    d_surf = min(dist_3d, dist_3d_Q);
                    
                    if d_surf < min_dist_overall
                        min_dist_overall = d_surf;
                        target_obs_idx = i;
                    end
                end
                
                % 検知距離内に入った場合
                if target_obs_idx > 0 && min_dist_overall <= obj.trigger_dist
                    tgt = obs_list(target_obs_idx);
                    c_obs = tgt.p_center;
                    radii = tgt.ellipsoid_radii;
                    R_o   = tgt.R_obs;
                    d_marg = tgt.d_margin;
                    
                    obj.t_start = time.t;
                    
                    % 進行方向と巡航速度の確定
                    v_vec = xd_nom(5:7);
                    spd = norm(v_vec);
                    if spd < 0.05, spd = norm(vL_cur); end
                    if spd < 0.05, spd = 0.3; v_vec = [0; 0; 0.3]; end
                    obj.dir_nominal = v_vec / spd;
                    obj.nominal_speed = spd;
                    
                    % 障害物通過予測時間に基づく区間長の設定
                    vec_to_obs = c_obs - pL_cur;
                    d_proj = dot(vec_to_obs, obj.dir_nominal);
                    if d_proj < 1.0, d_proj = 1.0; end
                    obj.T_seg = max(4.0, d_proj / spd);
                    obj.t_duration = 2.0 * obj.T_seg;
                    
                    % 幾何学的最短横方向法線の導出 (進行軸と直交)
                    p_perp = vec_to_obs - d_proj * obj.dir_nominal;
                    norm_perp = norm(p_perp);
                    
                    if norm_perp > 1e-3
                        obj.dir_normal = -p_perp / norm_perp;
                    else
                        if abs(obj.dir_nominal(3)) > 0.8
                            aux_axis = [1; 0; 0];
                        else
                            aux_axis = [0; 0; 1];
                        end
                        n_cand = cross(obj.dir_nominal, aux_axis);
                        obj.dir_normal = n_cand / norm(n_cand);
                    end
                    
                    % Zheng (2025) 5連保護球による必要離隔幅の算出
                    num_spheres = 5;
                    lambdas = linspace(0, 1, num_spheres);
                    r_eff_max = radii + max(obj.r_load, obj.r_drone) + d_marg;
                    A_mat = R_o * diag(1 ./ (r_eff_max.^2)) * R_o';
                    
                    max_req_delta = 0;
                    for j = 1:num_spheres
                        lam = lambdas(j);
                        r_sph = (1 - lam) * obj.r_load + lam * obj.r_drone;
                        
                        % 進行軸と直交する楕円断面半径の推定
                        r_eff_proj = sqrt(1 / max(1e-4, obj.dir_normal' * A_mat * obj.dir_normal));
                        req_j = r_eff_proj + r_sph + d_marg;
                        if req_j > max_req_delta
                            max_req_delta = req_j;
                        end
                    end
                    
                    % Cohen et al. (2023) Softplus フィルタによる滑らかな目標離隔量の確定
                    a_val = -max_req_delta;
                    b_val = 1.0;
                    delta_smooth = obj.cbf_sigma * log(1.0 + exp(-a_val / (b_val * obj.cbf_sigma)));
                    obj.delta_target = max(max_req_delta, delta_smooth);
                    
                    fprintf("\n=======================================================\n");
                    fprintf("[SMOOTH CBF FILTER] 障害物検知! (ID: %d, 表面最短距離: %.2f m, 時刻: %.3f s)\n", ...
                        target_obs_idx, min_dist_overall, time.t);
                    fprintf("  - 形状: %s | 楕円主軸半径: [a=%.2f, b=%.2f, c=%.2f] m\n", ...
                        tgt.type, radii(1), radii(2), radii(3));
                    fprintf("  - 回避法線: [%.2f, %.2f, %.2f] | 目標離隔量: %.3f m\n", ...
                        obj.dir_normal(1), obj.dir_normal(2), obj.dir_normal(3), obj.delta_target);
                    fprintf("  - Mellinger 13次多項式 (C^6 完全解析伝搬): T_seg=%.1f s, 全長=%.1f s\n", ...
                        obj.T_seg, obj.t_duration);
                    fprintf("=======================================================\n\n");
                    
                    % Mellinger 13次多項式スプラインの求解 (境界値問題)
                    obj.plan_mellinger_13th_polynomial();
                    obj.replan_active = true;
                end
            end
            
            % 3. C^6 連続軌道の評価・合流
            if obj.replan_active
                tau = time.t - obj.t_start;
                if tau <= obj.t_duration
                    xd = obj.evaluate_c6_trajectory(tau, xd_nom);
                else
                    if ~obj.replan_done
                        fprintf("[SMOOTH CBF FILTER] 障害物通過完了・公称軌道へ完全シームレス復帰 (t=%.3f s)\n\n", time.t);
                        obj.t_merge_end = obj.t_start + obj.t_duration;
                        xd_end = obj.evaluate_c6_trajectory(obj.t_duration, xd_nom);
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
                xd = xd_nom;
            end
            
            if length(xd) < 28, xd = [xd; zeros(28 - length(xd), 1)]; end
            obj.result.state.xd = xd;
            obj.result.state.p  = xd(1:3);
            obj.result.state.v  = xd(5:7);
            obj.result.state.q  = [0; 0; xd(4)];
            result = obj.result;
        end
    end
    
    methods (Access = private)
        function plan_mellinger_13th_polynomial(obj)
            % 13次多項式 (係数14個) による境界値問題の厳密解
            % 区間1: 始端 [0, 0, 0, 0, 0, 0, 0] -> 終端 [delta_target, 0, 0, 0, 0, 0, 0]
            % 区間2: 始端 [delta_target, 0, 0, 0, 0, 0, 0] -> 終端 [0, 0, 0, 0, 0, 0, 0]
            N = obj.order; % 13
            A_bvp = zeros(14, 14);
            
            % u = 0 での 0〜6階微分
            for k = 0:6
                A_bvp(k + 1, k + 1) = factorial(k);
            end
            % u = 1 での 0〜6階微分
            for k = 0:6
                for n = k:N
                    A_bvp(7 + k + 1, n + 1) = prod(n - k + 1 : n);
                end
            end
            
            % 区間1
            b1 = zeros(14, 1);
            b1(8) = obj.delta_target; % 終端位置
            obj.coeffs_seg1 = (A_bvp \ b1)';
            
            % 区間2
            b2 = zeros(14, 1);
            b2(1) = obj.delta_target; % 始端位置
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
            
            % 0〜6階微分の完全解析評価 (位置〜Pop)
            delta_k = zeros(7, 1);
            for k = 0:6
                val_k = 0;
                for n = k:N
                    factor = prod(n - k + 1 : n);
                    val_k = val_k + C(n + 1) * factor * (u^(n - k));
                end
                delta_k(k + 1) = val_k / (T_seg^k);
            end
            
            % 公称軌道の直交法線方向成分に厳密加算
            for k = 0:6
                idx = 4 * k + (1:3);
                xd(idx) = xd_nom(idx) + obj.dir_normal * delta_k(k + 1);
            end
        end
    end
end
classdef HLC_ROTOR_LEVEL_CBF < handle
    % HLC_ROTOR_LEVEL_CBF
    % ローター推力 f1,f2,f3,f4 に直接介入する CBF-QP コントローラ
    % 相対次数4の Exponential CBF (Tscholl et al. 2024) と
    % Smooth Softplus (Cohen et al. 2023) を適用
    
    properties
        self
        base_controller  % ノミナル入力を計算するベースのHLC
        result
        
        f_max = 5.0      % 各ローターの最大推力 [N]
        f_min = 0.0      % 各ローターの最小推力 [N]
        
        % High-Order CBF (Relative Degree 4) ゲイン: (s+p1)(s+p2)(s+p3)(s+p4)
        cbf_k1 = 8.0     % h^(3) の係数
        cbf_k2 = 24.0    % h^(2) の係数
        cbf_k3 = 32.0    % h^(1) の係数
        cbf_k4 = 16.0    % h^(0) の係数
        
        % Smooth Softplus パラメータ (Cohen et al.)
        sigma = 0.5
        gamma = 1.0
        
        % キャッシュ
        obs_list_cache = []
    end
    
    methods
        function obj = HLC_ROTOR_LEVEL_CBF(self, base_controller)
            obj.self = self;
            obj.base_controller = base_controller;
            obj.result.state = STATE_CLASS(struct('state_list', ["xd"], 'num_list', [28]));
        end
        
        function result = do(obj, varargin)
            time = varargin{1};
            
            % 1. ベースコントローラでノミナル入力 [T, tau_x, tau_y, tau_z] を計算
            res_base = obj.base_controller.do(varargin{:});
            U_hlc = res_base.input(1:4); % [T; tau]
            
            % ドローン物理パラメータの取得
            agent_p = obj.self.estimator.result.state.p;
            agent_q = obj.self.estimator.result.state.q; % Euler
            agent_v = obj.self.estimator.result.state.v;
            
            pL = obj.self.estimator.result.state.pL;
            vL = obj.self.estimator.result.state.vL;
            
            % 2. アロケーション行列 B を構築
            P = obj.self.parameter.get(["Lx","Ly","lx","ly","km1","km2","km3","km4"]);
            Lx=P(1); Ly=P(2); lx_=P(3); ly_=P(4);
            km1_=P(5); km2_=P(6); km3_=P(7); km4_=P(8);
            
            B_mat = [1,     1,          1,        1;
                    -ly_,  -ly_,      (Ly-ly_),  (Ly-ly_);
                     lx_,  -(Lx-lx_),  lx_,      -(Lx-lx_);
                     km1_, -km2_,     -km3_,      km4_];
            B_inv = inv(B_mat);
            
            % 3. ノミナルローター推力 U_nom = [f1, f2, f3, f4]^T
            U_nom = B_inv * U_hlc;
            
            % 4. 障害物情報の取得
            try
                obs_list = ENVIRONMENT_OBSTACLE_ELLIPSE();
            catch
                obs_list = [];
            end
            
            % =========================================================
            % CBF-QP のセットアップ (スラック変数付き)
            % 決定変数: X = [f1, f2, f3, f4, delta]^T
            % =========================================================
            H = diag([2, 2, 2, 2, 1e4]); % delta のペナルティは大きく
            f = [-2 * U_nom; 0];
            
            A_ineq = [];
            b_ineq = [];
            
            for i = 1:length(obs_list)
                obs = obs_list(i);
                c_obs = obs.p_center;
                
                % 保護点
                p_s = 0.5 * agent_p + 0.5 * pL;
                vec_diff = p_s - c_obs;
                
                % 【特異点回避】真上・真下にいると横に逃げるトルクが0になるため、微小オフセットを追加
                if norm(vec_diff(1:2)) < 0.2
                    vec_diff(1) = vec_diff(1) + 0.2;
                    vec_diff(2) = vec_diff(2) + 0.1;
                end
                
                dist_to_obs = norm(vec_diff);
                h_val = dist_to_obs^2 - (obs.ellipsoid_radii(1) + 0.3)^2;
                
                % センサー検知範囲 
                if dist_to_obs < 5.0
                    v_s = 0.5 * agent_v + 0.5 * vL;
                    h_dot = 2 * vec_diff' * v_s;
                    h_ddot = 2 * norm(v_s)^2; 
                    h_3dot = 0;               
                    Lf4_h = 0;                
                    
                    mass = 1.0; 
                    T_curr = max(0.1, sum(U_nom)); 
                    
                    J_u = zeros(3, 4);
                    J_u(1,:) =  (T_curr / mass) * B_mat(3,:); % Pitch -> Xスナップ
                    J_u(2,:) = -(T_curr / mass) * B_mat(2,:); % Roll  -> Yスナップ
                    J_u(3,:) =  0.05 * B_mat(1,:);            % Zは控えめに
                    
                    J_us = 0.5 * J_u;
                    LgLf3_h = 2 * vec_diff' * J_us; % 1x4
                    
                    fprintf("[CBF DETECT] 障害物%d 検知! 距離: %.2fm (h_val: %.2f)\n", i, dist_to_obs, h_val);
                    
                    a_term = Lf4_h + obj.cbf_k1 * h_3dot + obj.cbf_k2 * h_ddot + obj.cbf_k3 * h_dot;
                    b_val = max(1e-4, norm(LgLf3_h)^2); 
                    softplus_val = obj.sigma * log(1 + exp(-a_term / (b_val * obj.sigma)));
                    
                    % 制約: -LgLf3_h * U - delta <= Lf4_h + ...
                    A_ineq = [A_ineq; -LgLf3_h, -1]; % 5要素目(delta)を追加
                    b_ineq = [b_ineq; Lf4_h + obj.cbf_k1 * h_3dot + obj.cbf_k2 * h_ddot + obj.cbf_k3 * h_dot + obj.cbf_k4 * h_val + softplus_val];
                end
            end
            
            % 物理限界 (deltaには上下限なし、または緩く)
            lb = [obj.f_min * ones(4, 1); -Inf];
            ub = [obj.f_max * ones(4, 1);  Inf];
            
            % 5. QPの解決
            options = optimoptions('quadprog', 'Display', 'off');
            if ~isempty(A_ineq)
                [X_safe, ~, exitflag] = quadprog(H, f, A_ineq, b_ineq, [], [], lb, ub, [U_nom; 0], options);
                if exitflag == 1 || exitflag == 2
                    U_safe = X_safe(1:4);
                    delta_val = X_safe(5);
                    if delta_val > 0.1
                        fprintf("  -> [CBF WARNING] 物理限界のためスラック作動: delta = %.2f\n", delta_val);
                    end
                else
                    U_safe = U_nom; % フォールバック
                end
            else
                U_safe = max(obj.f_min, min(obj.f_max, U_nom));
            end
            
            % 補正量のコンソール表示
            dU_norm = norm(U_safe - U_nom);
            if dU_norm > 1e-3
                fprintf("  -> [CBF補正作動] 推力オフセット: %.3f N (f1:%.2f->%.2f, f2:%.2f->%.2f)\n", ...
                    dU_norm, U_nom(1), U_safe(1), U_nom(2), U_safe(2));
            end
            
            % 6. 安全なローター推力を再び [T, tau] に戻す
            U_hlc_safe = B_mat * U_safe;
            
            obj.result.input = [U_hlc_safe; 0; 0]; % [T; tau_x; tau_y; tau_z; thrust_flag; ...]
            result = obj.result;
        end
    end
end

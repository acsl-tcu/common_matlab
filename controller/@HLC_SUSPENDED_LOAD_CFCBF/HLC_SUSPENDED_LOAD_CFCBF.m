classdef HLC_SUSPENDED_LOAD_CFCBF < handle
    % クアッドコプター用階層型線形化 + 仮想加速度 Command Filtered CBF (CFCBF)
    % Z方向（推力）とXY方向（仮想加速度）を同時に1つのQPで解き、
    % フィルタを通して微分の整合性を保ちながらHLC下位層に渡す手法
properties
    self
    result
    param
    % Command Filter 用の状態変数
    vs_f            % フィルタされた仮想加速度 [3x1]
end

methods
    function obj = HLC_SUSPENDED_LOAD_CFCBF(self, param)
        obj.self  = self;
        obj.param = param;
        obj.result.min_clearance = Inf;
        obj.vs_f = [];
    end
    
    function result = do(obj, varargin)
        Param = obj.param; 
        model = obj.self.estimator.result; 
        ref   = obj.self.reference.result; 
        
        if isprop(ref.state, 'xd')
            xd = ref.state.xd; 
        else
            xd = ref.state.get();
        end
        
        pL = model.state.pL;
        if isprop(model.state, "pT")
            pT = model.state.pT;
        else
            delta = pL - model.state.p;
            if norm(delta) > 1e-9
                pT = delta / norm(delta);
            else
                pT = [0; 0; -1];
            end
        end
        vL = model.state.vL;
        wL = model.state.wL;
        
        P = [obj.self.parameter.get(["mass", "jx", "jy", "jz", "gravity", "loadmass", "cableL"]), 0, 0, 0];
        m = P(1);
        L_cable = P(7);
        x = [model.state.getq('compact'); model.state.w; pL; vL; pT; wL];
        
        % Yaw 角の誤差修正
        yaw      = wrapToPi(model.state.q(3)); 
        yawd     = xd(4); 
        yawUnit  = [cos(yaw); sin(yaw); 0]; 
        yawdUnit = [cos(yawd); sin(yawd); 0]; 
        deltaYaw = sign(cross(yawdUnit, yawUnit)) * acos(yawdUnit' * yawUnit); 
        xd(4)    = -deltaYaw(3) + yaw; 
        xd       = [xd; zeros(28 - size(xd, 1), 1)];
        
        tic_start = tic;
        
        %% =========================================================================
        %% 1. ノミナル制御入力 (第1層) の算定
        %% =========================================================================
        F1 = Param.F1; F2 = Param.F2; F3 = Param.F3; F4 = Param.F4; 
        vf = obj.Vfd_SuspendedLoadxyDst(Param.dt, x, xd', F1); 
        
        % vs は 1x3 の行ベクトルとして出力される
        vs_row = obj.Vs_SuspendedLoadxyDst(x, xd', vf, P, F2, F3, F4); 
        uf = obj.Uf_SuspendedLoadxyDst(x, xd', vf, P); 
        beta2 = obj.Beta2_SuspendedLoadxyDst(x, xd', vf, P); 
        
        % 扱いやすいように 3x1 の列ベクトルに変換
        vs_nom = vs_row(:);
        
        % フィルタの初期化
        if isempty(obj.vs_f)
            obj.vs_f = vs_nom;
        end
        
        %% =========================================================================
        %% 2. 幾何情報・クリアランス計算
        %% =========================================================================
        min_surf_dist = Inf;
        obs_env = ENVIRONMENT_OBSTACLE_HOCBF_LINK_XY();
        num_obs = length(obs_env);
        p_mid   = pL - 0.5 * L_cable * pT;
        
        % 被覆球体のパラメータ
        num_spheres = 3;
        lambda_list  = [0.0, 0.5, 1.0];
        rl_hard_list = [0.15, 0.15, 0.15]; 
        rl_soft_list = [0.4, 0.4, 0.4];
        
        log_p_obs = cell(1, max(1, num_obs));
        log_r_minimal = cell(1, max(1, num_obs));
        
        A_qp_soft = []; b_qp_soft = [];
        A_qp_hard = []; b_qp_hard = [];
        
        %% =========================================================================
        %% 3. 仮想加速度による MRD CBF 制約構築 (3次元回避)
        %% =========================================================================
        for i = 1:num_obs
            p_obs = obs_env(i).p_obs(:);
            ro = obs_env(i).r_obs;
            
            obs_min_dist = Inf;
            for j = 1:num_spheres
                lambda_j = lambda_list(j);
                x_c_j = pL - lambda_j * L_cable * pT;
                d_surf = norm(x_c_j - p_obs) - (ro + rl_hard_list(j));
                
                min_surf_dist = min(min_surf_dist, d_surf);
                obs_min_dist = min(obs_min_dist, d_surf);

                % 3.0m以内でCBF有効化
                if d_surf < 3.0
                    % --- ソフト制約 ---
                    gamma_params_soft = [1.5; 0.5]; 
                    [A_f, b_core, ~, ~] = CBF_Constraints_MRD_Core(pL, vL, pT, wL, p_obs, ro, rl_soft_list(j), lambda_j, P(1), P(6), P(7), P(5), gamma_params_soft(1), gamma_params_soft(2));
                    
                    % 最適化変数は [u1; vs_x; vs_y]
                    % A_f(3)*u1 + m*A_f(1)*vs_x + m*A_f(2)*vs_y >= b_core
                    A_cbf_vs_s = [A_f(3), m * A_f(1), m * A_f(2)];
                    
                    if all(~isnan(A_cbf_vs_s)) && ~isnan(b_core) && all(~isinf(A_cbf_vs_s))
                        A_qp_soft = [A_qp_soft; -A_cbf_vs_s];
                        b_qp_soft = [b_qp_soft; -b_core];
                    end

                    % --- ハード制約 ---
                    gamma_params_hard = [2.0; 1.0]; 
                    [A_f_h, b_core_h, ~, ~] = CBF_Constraints_MRD_Core(pL, vL, pT, wL, p_obs, ro, rl_hard_list(j), lambda_j, P(1), P(6), P(7), P(5), gamma_params_hard(1), gamma_params_hard(2));
                    A_cbf_vs_h = [A_f_h(3), m * A_f_h(1), m * A_f_h(2)];
                    
                    if all(~isnan(A_cbf_vs_h)) && ~isnan(b_core_h) && all(~isinf(A_cbf_vs_h))
                        A_qp_hard = [A_qp_hard; -A_cbf_vs_h];
                        b_qp_hard = [b_qp_hard; -b_core_h];
                    end
                end
            end
            log_p_obs{i} = p_obs; 
            log_r_minimal{i} = obs_min_dist;
        end
        
        %% =========================================================================
        %% 4. QP の構築と求解
        %% =========================================================================
        % 変数: x_qp = [u1; vs_x; vs_y; delta]
        num_vars = 4;
        
        % ペナルティ重み: 仮想加速度の変更を許容し、deltaのコストを大きくする
        H = diag([1.0, 1.0, 1.0, 1e5]); 
        tmp_vs = [uf(1); vs_nom(1); vs_nom(2)];
        tmp_ext = [tmp_vs; 0];
        f = -H * tmp_ext; 
        
        A_qp_ext = []; b_qp_ext = [];
        if ~isempty(A_qp_soft)
            A_qp_ext = [A_qp_ext; [A_qp_soft, -ones(size(A_qp_soft, 1), 1)]];
            b_qp_ext = [b_qp_ext; b_qp_soft];
        end
        if ~isempty(A_qp_hard)
            A_qp_ext = [A_qp_ext; [A_qp_hard, zeros(size(A_qp_hard, 1), 1)]];
            b_qp_ext = [b_qp_ext; b_qp_hard];
        end
        
        % 仮想加速度の制限 (vs_limit)。これが機体の傾き上限として機能し、裏返りを防ぐ
        vs_limit = 5.0; 
        lb = [0.0; -vs_limit; -vs_limit; 0.0];
        ub = [20.0;  vs_limit;  vs_limit; Inf];
        
        options = optimoptions('quadprog', 'Display', 'off');
        if isempty(A_qp_ext)
            u_safe_vs = tmp_vs;
        else
            [x_qp_safe, ~, exitflag] = quadprog(H, f, A_qp_ext, b_qp_ext, [], [], lb, ub, tmp_ext, options);
            if exitflag ~= 1
                u_safe_vs = tmp_vs;
            else
                u_safe_vs = x_qp_safe(1:3);
            end
        end
        
        %% =========================================================================
        %% 5. Command Filter による滑らかな仮想加速度の生成
        %% =========================================================================
        % QP が要求する目標の仮想加速度 (3x1)
        vs_safe_target = [u_safe_vs(2); u_safe_vs(3); vs_nom(3)];
        
        % 1次遅れフィルタ (Command Filter)
        % 帯域幅 omega_n: ドローンの姿勢制御応答速度に近い値（例: 10〜20 rad/s）
        omega_n = 15.0; 
        
        % フィルタの微分方程式
        vs_f_dot = omega_n * (vs_safe_target - obj.vs_f);
        
        % フィルタ状態の更新 (オイラー法)
        dt = Param.dt;
        if isempty(dt) || dt <= 0, dt = 0.01; end
        obj.vs_f = obj.vs_f + vs_f_dot * dt;
        
        %% =========================================================================
        %% 6. 第2層 HLC への受け渡し
        %% =========================================================================
        % フィルタされた仮想加速度 (3x1 の列ベクトルとして V2_alpha2_... に渡す)
        vs_safe = obj.vs_f;
        
        % 逆変換
        % 元のコード `vs_alpha2 = obj.V2_alpha2_SuspendedLoadxyDst(..., vs', ...)` と
        % 全く同じ形式（3x1の列ベクトル）で渡す。
        vs_alpha2_safe = obj.V2_alpha2_SuspendedLoadxyDst(x, xd', vf, vs_safe, P); 
        us_safe = beta2 \ vs_alpha2_safe; 
        
        % 最終出力の統合と物理クリップ
        u_final = [u_safe_vs(1); us_safe];
        u_final(1) = max(0.0, min(20.0, u_final(1)));
        u_final(2) = max(-1.0, min(1.0, u_final(2)));
        u_final(3) = max(-1.0, min(1.0, u_final(3)));
        u_final(4) = max(-1.0, min(1.0, u_final(4)));
        
        %% =========================================================================
        %% 7. ログ保存
        %% =========================================================================
        obj.result.min_clearance = min_surf_dist;
        obj.result.p_obs = log_p_obs;
        obj.result.r_minimal = log_r_minimal;
        obj.result.controllertime = toc(tic_start);
        
        obj.result.input = u_final;
        obj.result.xd = xd;
        obj.result.x = x;
        result = obj.result;
    end
    
    function show(obj)
        obj.result
    end
end
end
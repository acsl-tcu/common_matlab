classdef HLC_SUSPENDED_LOAD_CBF < HLC_SUSPENDED_LOAD
  % 懸架システムのノミナルHLCをベースに、紐中点(p_m)を対象としたActuator CBF (案1)を適用する
  properties
    T_val
    dT_val
    initialized
  end
  
  methods
    function obj = HLC_SUSPENDED_LOAD_CBF(self, param)
        obj@HLC_SUSPENDED_LOAD(self, param);
        if ~isfield(obj.result, 'min_clearance')
            obj.result.min_clearance = Inf;
        end
        obj.T_val = 0;
        obj.dT_val = 0;
        obj.initialized = false;
    end
    
    function result = do(obj, varargin)
        % 1. ベースクラスのdoを呼んでノミナル入力を計算
        res_nom = do@HLC_SUSPENDED_LOAD(obj, varargin{:});
        u_nom = res_nom.tmp; % 制限前の実入力 [T; tau_x; tau_y; tau_z]
        
        T_nom = u_nom(1);
        tau_nom = u_nom(2:4);
        
        if ~obj.initialized
            obj.T_val = T_nom;
            obj.dT_val = 0;
            obj.initialized = true;
        end
        
        % 2. 状態の取得
        model = obj.self.estimator.result;
        P_vec = obj.self.parameter.get(["mass", "jx", "jy", "jz", "gravity", "loadmass", "cableL"]);
        dt = obj.param.dt;
        
        % ドローンと負荷の位置・速度など
        p_load = model.state.pL;
        v_load = model.state.vL;
        pT = model.state.pT;
        wL = model.state.wL;
        
        q_quat = R2q(model.state.getq("rotmat"));
        w_vec = model.state.w;
        
        % ユーザーの簡潔なジェネレータ仕様に基づく状態ベクトル
        % z_state = [pl; vl; pT; ol; q; w];
        z_state = [p_load(:); v_load(:); pT(:); wL(:); q_quat(:); w_vec(:)];
        T_state = [obj.T_val; obj.dT_val];
        
        % 3. 動的拡張とQPのセットアップ
        M_total = P_vec(1) + P_vec(6); 
        T_hover = M_total * P_vec(5); % ホバリング推力

        if ~obj.initialized || isnan(obj.T_val)
            obj.T_val = T_hover;
            obj.dT_val = 0;
            obj.initialized = true;
        end
        
        Kp_T = 400.0; Kd_T = 40.0;
        ddT_nom = -Kp_T * (obj.T_val - T_nom) - Kd_T * obj.dT_val;
        mu_nom = [ddT_nom; tau_nom];

        % 単機と完全に同じ重み
        % H_qp = diag([1.0, 50.0, 50.0, 50.0]); 
        % H_qp = diag([0.01, 0.5, 0.5, 0.5]);
        H_qp = diag([0.1, 5, 5, 5]);
        f_qp = -H_qp * mu_nom;

        A_obs = [];
        b_obs = [];
        min_h_this_step = Inf;

        % 4. 障害物CBF
        % params = [m; mL; cableL; jx; jy; jz; g];
        drone_params = [P_vec(1); P_vec(6); P_vec(7); P_vec(2); P_vec(3); P_vec(4); P_vec(5)]; 
        
        % ★ システム全体（ドローン・紐・負荷）を覆う安全マージン半径
        r_system = 1.0;
        % r_system = 0.35;
        
        obs_list = ENVIRONMENT_OBSTACLE_HOCBF_LINK_XY();
        for i = 1:length(obs_list)
            % 障害物本来の半径 (r_obs) に、システム側のマージン (r_system) を足す
            r_barrier = obs_list(i).r_obs + r_system;
            obs_params = [obs_list(i).p_obs; r_barrier]; 
            
            % 単機と完全に同じ極配置 (s+2)^4
            % cbf_gains = [16; 32; 24; 8]; 
            cbf_gains = [1; 4; 6; 4];
            
            [A_cbf, b_cbf, h_val] = Sphere_SuspendedLoad_TetherMid_CBF(z_state, T_state, drone_params, obs_params, cbf_gains);
            
            if h_val < min_h_this_step
                min_h_this_step = h_val;
            end
            if h_val < obj.result.min_clearance
                obj.result.min_clearance = h_val;
            end
            
            if ~any(isnan(A_cbf)) && ~any(isnan(b_cbf))
                A_obs = [A_obs; A_cbf];
                b_obs = [b_obs; b_cbf];
            end
        end
        obj.result.h_val = min_h_this_step; % ロガー用
        
        % 5. Solve QP (単機と完全に同一の定式化、スラック無し・推力バリア無し)
        mu_safe = mu_nom;
        if ~isempty(A_obs)
            lb = [-1000; -1.0; -1.0; -1.0];
            ub = [ 1000;  1.0;  1.0;  1.0];
            options = optimoptions('quadprog', 'Display', 'off');
            [mu_opt, ~, exitflag] = quadprog(H_qp, f_qp, A_obs, b_obs, [], [], lb, ub, [], options);
            if exitflag == 1
                mu_safe = mu_opt;
            end
        end
        
        % 6. 動的拡張の積分
        obj.dT_val = obj.dT_val + mu_safe(1) * dt;
        obj.T_val  = obj.T_val  + obj.dT_val * dt;
        
        % 自由落下防止の推力下限・上限ガード (事後クリッピング)
        T_min = T_hover * 0.1; 
        T_max = max(20.0, T_hover * 2.0);
        obj.T_val = max(T_min, min(T_max, obj.T_val)); 
        
        % 7. 最終出力を厳密にクリッピング
        tau_safe = max(-1.0, min(1.0, mu_safe(2:4)));
        obj.result.input = [obj.T_val; tau_safe];
        result = obj.result;
    end
  end
end

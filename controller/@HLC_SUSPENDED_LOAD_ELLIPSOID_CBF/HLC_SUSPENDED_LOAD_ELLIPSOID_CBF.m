classdef HLC_SUSPENDED_LOAD_ELLIPSOID_CBF < HLC_SUSPENDED_LOAD
  properties
    T_val
    dT_val
    initialized
  end
  
  methods
    function obj = HLC_SUSPENDED_LOAD_ELLIPSOID_CBF(self, param)
        obj@HLC_SUSPENDED_LOAD(self, param);
        if ~isfield(obj.result, 'min_clearance')
            obj.result.min_clearance = Inf;
        end
        obj.T_val = 0;
        obj.dT_val = 0;
        obj.initialized = false;
    end
    
    function result = do(obj, varargin)
        res_nom = do@HLC_SUSPENDED_LOAD(obj, varargin{:});
        u_nom = res_nom.tmp; 
        
        T_nom = u_nom(1);
        tau_nom = u_nom(2:4);
        
        if ~obj.initialized
            obj.T_val = T_nom;
            obj.dT_val = 0;
            obj.initialized = true;
        end
        
        model = obj.self.estimator.result;
        P_vec = obj.self.parameter.get(["mass", "jx", "jy", "jz", "gravity", "loadmass", "cableL"]);
        dt = obj.param.dt;
        
        p_load = model.state.pL;
        v_load = model.state.vL;
        pT = model.state.pT;
        wL = model.state.wL;
        q_quat = R2q(model.state.getq("rotmat"));
        w_vec = model.state.w;
        
        z_state = [p_load(:); v_load(:); pT(:); wL(:); q_quat(:); w_vec(:)];
        T_state = [obj.T_val; obj.dT_val];
        
        M_total = P_vec(1) + P_vec(6); 
        T_hover = M_total * P_vec(5); 

        if ~obj.initialized || isnan(obj.T_val)
            obj.T_val = T_hover;
            obj.dT_val = 0;
            obj.initialized = true;
        end
        
        Kp_T = 400.0; Kd_T = 40.0;
        ddT_nom = -Kp_T * (obj.T_val - T_nom) - Kd_T * obj.dT_val;
        mu_nom = [ddT_nom; tau_nom];

        H_qp = diag([5, 5, 5, 5]); 
        f_qp = -H_qp * mu_nom;

        A_obs = [];
        b_obs = [];
        min_h_this_step = Inf;
        min_d_surf_this_step = Inf;

        drone_params = [P_vec(1); P_vec(6); P_vec(7); P_vec(2); P_vec(3); P_vec(4); P_vec(5)]; 
        p_mid = p_load - 0.5 * P_vec(7) * pT;
        
        % 自機楕円体パラメータ
        a_rad = 0.3;
        b_rad = P_vec(7) / 2.0 + 0.1;
        ellipsoid_params = [a_rad; b_rad];
        
        % 新しい楕円体環境を読み込み
        obs_list = ENVIRONMENT_OBSTACLE_ELLIPSOID();
        for i = 1:length(obs_list)
            p_obs = obs_list(i).p_obs;
            R_obs = obs_list(i).R_obs;
            Q_obs = obs_list(i).Q_obs;
            d_margin = obs_list(i).d_margin;
            
            Q2_obs = R_obs * (Q_obs^2) * R_obs';
            
            % 最適な分離超平面の法線 n_vec を勾配法で探索
            n_vec = p_mid - p_obs; 
            if norm(n_vec) > 1e-6, n_vec = n_vec / norm(n_vec); else, n_vec = [0;0;1]; end
            
            for k = 1:5
                r_sys_ext = sqrt(a_rad^2 + (b_rad^2 - a_rad^2)*(n_vec'*pT)^2);
                r_obs_ext = sqrt(n_vec' * Q2_obs * n_vec);
                
                grad_r_sys = ((b_rad^2 - a_rad^2)*(n_vec'*pT) / max(1e-6, r_sys_ext)) * pT;
                grad_r_obs = (Q2_obs * n_vec) / max(1e-6, r_obs_ext);
                
                grad_n = (p_mid - p_obs) - grad_r_sys - grad_r_obs;
                n_vec = n_vec + 0.2 * grad_n;
                n_vec = n_vec / norm(n_vec);
            end
            
            % 障害物側の表面接平面の距離 d_plane (安全マージンを含む)
            r_obs_ext = sqrt(n_vec' * Q2_obs * n_vec);
            d_plane = n_vec' * p_obs + r_obs_ext + d_margin;
            plane_params = [n_vec; d_plane];
            
            cbf_gains = [16; 40; 33; 10]; 
            
            [A_cbf, b_cbf, h_val, dh, ddh, dddh] = Ellipsoid_SuspendedLoad_TetherMid_CBF(z_state, T_state, drone_params, ellipsoid_params, plane_params, cbf_gains);
            
            d_surf = h_val;
            if h_val < min_h_this_step, min_h_this_step = h_val; end
            if d_surf < min_d_surf_this_step, min_d_surf_this_step = d_surf; end
            if h_val < obj.result.min_clearance, obj.result.min_clearance = h_val; end
            
            if d_surf < 4.0
                if ~any(isnan(A_cbf)) && ~any(isnan(b_cbf))
                    A_obs = [A_obs; A_cbf];
                    b_obs = [b_obs; b_cbf];
                end
            end
        end
        obj.result.h_val = min_h_this_step; 
        obj.result.d_surf = min_d_surf_this_step; 
        
        % ★ 姿勢ハード制約 (Attitude Limit CBF)
        % ロール角・ピッチ角が35度(約0.61rad)以上傾かないようにする
        cos_gamma_max = cos(35 * pi / 180);
        cbf_gains_att = [25; 10]; % (s+5)^2 の極配置
        [A_att, b_att, h_att] = Attitude_Limit_CBF([q_quat(:); w_vec(:)], [P_vec(2); P_vec(3); P_vec(4)], cos_gamma_max, cbf_gains_att);
        
        if ~any(isnan(A_att)) && ~any(isnan(b_att))
            A_obs = [A_obs; A_att];
            b_obs = [b_obs; b_att];
        end
        
        % 5. Solve QP
        mu_safe = mu_nom;
        if ~isempty(A_obs)
            % lb = [-1000; -1.0; -1.0; -1.0];
            % ub = [ 1000;  1.0;  1.0;  1.0];
            lb = [-1000; -0.5; -0.5; -0.5];
            ub = [ 1000;  0.5;  0.5;  0.5];
            options = optimoptions('quadprog', 'Display', 'off');
            [mu_opt, ~, exitflag] = quadprog(H_qp, f_qp, A_obs, b_obs, [], [], lb, ub, [], options);
            if exitflag == 1
                mu_safe = mu_opt;
            end
        end
        
        obj.dT_val = obj.dT_val + mu_safe(1) * dt;
        obj.T_val  = obj.T_val  + obj.dT_val * dt;
        
        T_min = T_hover * 0.1; 
        T_max = max(20.0, T_hover * 2.0);
        obj.T_val = max(T_min, min(T_max, obj.T_val)); 
        
        tau_safe = max(-1.0, min(1.0, mu_safe(2:4)));
        obj.result.input = [obj.T_val; tau_safe];
        result = obj.result;
    end
  end
end

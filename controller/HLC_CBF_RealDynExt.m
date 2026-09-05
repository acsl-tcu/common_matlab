classdef HLC_CBF_RealDynExt < handle
  % Hierarchical Linearization Controller with Real Input Dynamic Extension
  % 決定変数: mu = [ddT; tau_x; tau_y; tau_z]
  % 3軸（推力・トルク）を平等にスナップ空間で最適化する究極のCBFコントローラ

  properties
    self
    result
    param
    parameter_order = ["mass","Lx","Ly","lx","ly","jx","jy","jz","gravity","km1","km2","km3","km4","k1","k2","k3","k4"];
    T_val
    dT_val
    initialized
  end

  methods
    function obj = HLC_CBF_RealDynExt(self, param)
      obj.self = self;
      obj.param = param;
      obj.param.P = self.parameter.get(obj.parameter_order);
      obj.result.input = zeros(4,1);
      obj.result.min_clearance = Inf;
      obj.T_val = 0;
      obj.dT_val = 0;
      obj.initialized = false;
    end

    function result = do(obj, varargin)
      model = obj.self.estimator.result;
      ref = obj.self.reference.result;
      xd = ref.state.xd;
      P = obj.param.P;
      % Pを行ベクトルに統一 (Vsなどの関数は行ベクトルを想定しているため)
      if iscolumn(P)
          P = P';
      end
      F1 = obj.param.F1; F2 = obj.param.F2; F3 = obj.param.F3; F4 = obj.param.F4;
      xd = [xd; zeros(max(0, 20-size(xd,1)), 1)];

      % Yaw alignment
      Rb0 = RodriguesQuaternion(Eul2Quat([0;0;xd(4)]));
      q_rotmat = model.state.getq("rotmat");
      q_quat = R2q(Rb0'*q_rotmat);
      x = [q_quat; Rb0'*model.state.p; Rb0'*model.state.v; model.state.w];
      xd(1:3) = Rb0'*xd(1:3); xd(4) = 0; xd(5:7) = Rb0'*xd(5:7);

      dt = obj.param.dt;
      if ~isempty(varargin) && isfield(varargin{1}, 'dt')
        dt = varargin{1}.dt;
      end

      % 1. HLCによるノミナル制御入力の計算
      vf_nom = Vf(x, xd', P, F1);
      vs_nom = Vs(x, xd', vf_nom, P, F2, F3, F4);
      u_nom = Uf(x, xd', vf_nom, P) + Us(x, xd', vf_nom, vs_nom', P);
      
      T_nom = u_nom(1);
      tau_nom = u_nom(2:4);

      % 初期化
      if ~obj.initialized
        obj.T_val = T_nom;
        obj.dT_val = 0;
        obj.initialized = true;
      end

      % 2. 動的拡張: T_val を T_nom に追従させるためのノミナル推力加速度 ddT
      % 強いPD制御でノミナル推力に引き戻す (omega_n = 20, zeta = 1.0)
      Kp_T = 400.0;
      Kd_T = 40.0;
      ddT_nom = -Kp_T * (obj.T_val - T_nom) - Kd_T * obj.dT_val;

      mu_nom = [ddT_nom; tau_nom];

      % 3. QP Setup
      % [ddT, tau_x, tau_y, tau_z] に対するペナルティ。
      % 3軸平等に動けるよう、推力加速度とトルクの重みをバランスさせる。
      H_qp = diag([1.0, 50.0, 50.0, 50.0]); 
      f_qp = -H_qp * mu_nom;

      A_cbf_all = [];
      b_cbf_all = [];

      % 状態ベクトルとパラメータの準備
      p_vec = x(5:7);
      v_vec = x(8:10);
      w_vec = x(11:13);
      z_state = [p_vec; v_vec; q_quat; w_vec];
      T_state = [obj.T_val; obj.dT_val];
      
      m_drone = P(1);
      drone_params = [m_drone; P(6); P(7); P(8); P(9)]; % m, jx, jy, jz, g

      % 障害物の読み込み
      obs_list = ENVIRONMENT_OBSTACLE_HOCBF_LINK_XY();
      for i = 1:length(obs_list)
        p_obs = obs_list(i).p_obs;
        r_obs = obs_list(i).r_obs_margin;
        
        p_obs_rel = Rb0' * p_obs;
        obs_params = [p_obs_rel; r_obs];

        % 4次系 HOCBF の極配置 (重根 lambda = 2.0)
        % (s+2)^4 = s^4 + 8s^3 + 24s^2 + 32s + 16
        cbf_gains = [16; 32; 24; 8];

        [A_cbf, b_cbf, h_val] = Sphere_RealDynExt_CBF(z_state, T_state, drone_params, obs_params, cbf_gains);
        
        if h_val < obj.result.min_clearance
            obj.result.min_clearance = h_val;
        end

        % 不正値チェック
        if ~any(isnan(A_cbf)) && ~any(isnan(b_cbf))
            A_cbf_all = [A_cbf_all; A_cbf];
            b_cbf_all = [b_cbf_all; b_cbf];
        end
      end

      % 4. Solve QP
      mu_safe = mu_nom;
      if ~isempty(A_cbf_all)
        % トルクと推力加速度の物理限界（必要に応じて設定）
        lb = [-1000; -2; -2; -2];
        ub = [ 1000;  2;  2;  2];
        options = optimoptions('quadprog', 'Display', 'off');
        [mu_opt, ~, exitflag] = quadprog(H_qp, f_qp, A_cbf_all, b_cbf_all, [], [], lb, ub, [], options);
        if exitflag == 1
          mu_safe = mu_opt;
        end
      end

      % 5. Dynamic Extension (Integration)
      ddT_safe = mu_safe(1);
      tau_safe = mu_safe(2:4);

      obj.dT_val = obj.dT_val + ddT_safe * dt;
      obj.T_val  = obj.T_val  + obj.dT_val * dt;

      % 自由落下防止の推力下限ガード
      f_hover = m_drone * P(9);
      if obj.T_val < 0.2 * f_hover
          obj.T_val = 0.2 * f_hover;
          obj.dT_val = 0;
      end
      if obj.T_val > 30.0
          obj.T_val = 30.0;
          obj.dT_val = 0;
      end

      % 6. Final Output
      obj.result.input = [obj.T_val; tau_safe];
      result = obj.result;
    end
  end
end

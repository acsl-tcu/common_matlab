classdef HLC_CBF_FiniteDifference < handle
  % 案3: 有限差分近似 (決定変数を [T, tau] に戻し、ddT を差分近似で代入)
  % 決定変数: mu = [T; tau_x; tau_y; tau_z]

  properties
    self
    result
    param
    parameter_order = ["mass","Lx","Ly","lx","ly","jx","jy","jz","gravity","km1","km2","km3","km4","k1","k2","k3","k4"];
    T_prev1
    T_prev2
    initialized
  end

  methods
    function obj = HLC_CBF_FiniteDifference(self, param)
      obj.self = self;
      obj.param = param;
      obj.param.P = self.parameter.get(obj.parameter_order);
      obj.result.input = zeros(4,1);
      obj.result.min_clearance = Inf;
      obj.T_prev1 = 0;
      obj.T_prev2 = 0;
      obj.initialized = false;
    end

    function result = do(obj, varargin)
      model = obj.self.estimator.result;
      ref = obj.self.reference.result;
      xd = ref.state.xd;
      P = obj.param.P;
      if iscolumn(P), P = P'; end
      F1 = obj.param.F1; F2 = obj.param.F2; F3 = obj.param.F3; F4 = obj.param.F4;
      xd = [xd; zeros(max(0, 20-size(xd,1)), 1)];

      Rb0 = RodriguesQuaternion(Eul2Quat([0;0;xd(4)]));
      q_rotmat = model.state.getq("rotmat");
      q_quat = R2q(Rb0'*q_rotmat);
      x = [q_quat; Rb0'*model.state.p; Rb0'*model.state.v; model.state.w];
      xd(1:3) = Rb0'*xd(1:3); xd(4) = 0; xd(5:7) = Rb0'*xd(5:7);

      dt = obj.param.dt;
      if ~isempty(varargin) && isfield(varargin{1}, 'dt')
        dt = varargin{1}.dt;
      end

      vf_nom = Vf(x, xd', P, F1);
      vs_nom = Vs(x, xd', vf_nom, P, F2, F3, F4);
      u_nom = Uf(x, xd', vf_nom, P) + Us(x, xd', vf_nom, vs_nom', P);
      
      if ~obj.initialized
        obj.T_prev1 = u_nom(1);
        obj.T_prev2 = u_nom(1);
        obj.initialized = true;
      end

      % 差分近似による現在の推力速度
      dT_curr = (obj.T_prev1 - obj.T_prev2) / dt;

      mu_nom = u_nom;
      H_qp = diag([1.0, 50.0, 50.0, 50.0]); 
      f_qp = -H_qp * mu_nom;

      A_cbf_all = [];
      b_cbf_all = [];

      % 障害物CBF
      z_state = [x(5:7); x(8:10); q_quat; x(11:13)];
      T_state = [obj.T_prev1; dT_curr];
      drone_params = [P(1); P(6); P(7); P(8); P(9)];
      cbf_gains = [16; 32; 24; 8];

      obs_list = ENVIRONMENT_OBSTACLE_HOCBF_LINK_XY();
      for i = 1:length(obs_list)
        obs_params = [Rb0' * obs_list(i).p_obs; obs_list(i).r_obs_margin];
        [A_ext, b_ext, h_val] = Sphere_RealDynExt_CBF(z_state, T_state, drone_params, obs_params, cbf_gains);
        if h_val < obj.result.min_clearance, obj.result.min_clearance = h_val; end
        
        if ~any(isnan(A_ext)) && ~any(isnan(b_ext))
            % A_ext は [ddT, tau_x, tau_y, tau_z] に対する係数
            % ddT = (T - 2*T_prev1 + T_prev2) / dt^2
            % A_ext(1) * ddT <= b_ext
            % => (A_ext(1) / dt^2) * T <= b_ext + A_ext(1) * (2*T_prev1 - T_prev2) / dt^2
            A_cbf = [(A_ext(:,1) / dt^2), A_ext(:, 2:4)];
            b_cbf = b_ext + A_ext(:,1) * (2*obj.T_prev1 - obj.T_prev2) / dt^2;
            
            A_cbf_all = [A_cbf_all; A_cbf];
            b_cbf_all = [b_cbf_all; b_cbf];
        end
      end

      % 直接の下限・上限
      T_min = 1.0;
      T_max = 20.0;
      lb = [T_min; -2; -2; -2];
      ub = [T_max;  2;  2;  2];

      mu_safe = mu_nom;
      if ~isempty(A_cbf_all)
        options = optimoptions('quadprog', 'Display', 'off');
        [mu_opt, ~, exitflag] = quadprog(H_qp, f_qp, A_cbf_all, b_cbf_all, [], [], lb, ub, [], options);
        if exitflag == 1
          mu_safe = mu_opt;
        end
      end

      obj.T_prev2 = obj.T_prev1;
      obj.T_prev1 = mu_safe(1);

      obj.result.input = mu_safe;
      result = obj.result;
    end
  end
end

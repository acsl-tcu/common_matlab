classdef HLC_CBF_SimpleMPC < handle
  % 案2: 簡易MPC (T_next を決定変数に含め、等式制約で繋ぐ)
  % 決定変数: mu = [ddT; tau_x; tau_y; tau_z; T_next]

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
    function obj = HLC_CBF_SimpleMPC(self, param)
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
      
      T_nom = u_nom(1);
      tau_nom = u_nom(2:4);

      if ~obj.initialized
        obj.T_val = T_nom;
        obj.dT_val = 0;
        obj.initialized = true;
      end

      Kp_T = 400.0; Kd_T = 40.0;
      ddT_nom = -Kp_T * (obj.T_val - T_nom) - Kd_T * obj.dT_val;
      
      % mu = [ddT; tau; T_next]
      mu_nom = [ddT_nom; tau_nom; T_nom];
      H_qp = diag([1.0, 50.0, 50.0, 50.0, 0.01]); 
      f_qp = -H_qp * mu_nom;

      A_cbf_all = [];
      b_cbf_all = [];

      % 障害物CBF
      z_state = [x(5:7); x(8:10); q_quat; x(11:13)];
      T_state = [obj.T_val; obj.dT_val];
      drone_params = [P(1); P(6); P(7); P(8); P(9)];
      cbf_gains = [16; 32; 24; 8];

      obs_list = ENVIRONMENT_OBSTACLE_HOCBF_LINK_XY();
      for i = 1:length(obs_list)
        obs_params = [Rb0' * obs_list(i).p_obs; obs_list(i).r_obs_margin];
        [A_cbf, b_cbf, h_val] = Sphere_RealDynExt_CBF(z_state, T_state, drone_params, obs_params, cbf_gains);
        if h_val < obj.result.min_clearance, obj.result.min_clearance = h_val; end
        if ~any(isnan(A_cbf)) && ~any(isnan(b_cbf))
            % 5列目に 0 を追加
            A_cbf_all = [A_cbf_all; [A_cbf, 0]];
            b_cbf_all = [b_cbf_all; b_cbf];
        end
      end

      % 等式制約: T_next - ddT*dt^2/2 = T_curr + dT*dt
      Aeq = [-0.5*dt^2, 0, 0, 0, 1];
      beq = obj.T_val + obj.dT_val * dt;

      % 直接の下限・上限
      T_min = 1.0;
      T_max = 20.0;
      lb = [-1000; -2; -2; -2; T_min];
      ub = [ 1000;  2;  2;  2; T_max];

      mu_safe = mu_nom;
      options = optimoptions('quadprog', 'Display', 'off');
      [mu_opt, ~, exitflag] = quadprog(H_qp, f_qp, A_cbf_all, b_cbf_all, Aeq, beq, lb, ub, [], options);
      if exitflag == 1
        mu_safe = mu_opt;
      end

      obj.dT_val = obj.dT_val + mu_safe(1) * dt;
      obj.T_val  = mu_safe(5); % 直接 T_next を使う

      obj.result.input = [obj.T_val; mu_safe(2:4)];
      result = obj.result;
    end
  end
end

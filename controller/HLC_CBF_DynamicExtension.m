classdef HLC_CBF_DynamicExtension < handle
  % Hierarchical Linearization Controller with Dynamic Extension & Snap-space CBF
  properties
    self
    result
    param
    parameter_order = ["mass","Lx","Ly","lx","ly","jx","jy","jz","gravity","km1","km2","km3","km4","k1","k2","k3","k4"];
    v1_val
    dv1_val
    initialized
  end

  methods
    function obj = HLC_CBF_DynamicExtension(self, param)
      obj.self = self;
      obj.param = param;
      obj.param.P = self.parameter.get(obj.parameter_order);
      obj.result.input = zeros(4,1);
      obj.result.min_clearance = Inf;
      obj.v1_val = 0;
      obj.dv1_val = 0;
      obj.initialized = false;
    end

    function result = do(obj, varargin)
      model = obj.self.estimator.result;
      ref = obj.self.reference.result;
      xd = ref.state.xd;
      P = obj.param.P(:);
      F1 = obj.param.F1; F2 = obj.param.F2; F3 = obj.param.F3; F4 = obj.param.F4;
      xd = [xd; zeros(max(0, 20-size(xd,1)), 1)];

      % Yaw alignment
      Rb0 = RodriguesQuaternion(Eul2Quat([0;0;xd(4)]));
      q_rotmat = model.state.getq("rotmat");
      x = [R2q(Rb0'*q_rotmat); Rb0'*model.state.p; Rb0'*model.state.v; model.state.w];
      xd(1:3) = Rb0'*xd(1:3); xd(4) = 0; xd(5:7) = Rb0'*xd(5:7);

      dt = obj.param.dt;
      if ~isempty(varargin) && isfield(varargin{1}, 'dt')
        dt = varargin{1}.dt;
      end

      % 1. Nominal Virtual Inputs (from LQR)
      vf_nom = Vf(x, xd', P, F1);
      vs_nom = Vs(x, xd', vf_nom, P, F2, F3, F4);

      if ~obj.initialized
        obj.v1_val = vf_nom(1);
        obj.dv1_val = vf_nom(2);
        obj.initialized = true;
      end

      % 2. State Estimation for Snap CBF (a and j)
      m = P(1); g = P(9);
      % Approximate T from current virtual v1 to get acceleration
      cos_term = q_rotmat(3,3); 
      if cos_term < 0.1, cos_term = 0.1; end
      T_approx = (g + obj.v1_val) * m / cos_term;
      if T_approx < 0, T_approx = 0; end
      a_vec = [0;0;-g] + (T_approx/m) * q_rotmat(:,3);
      j_vec = [0;0;0]; % 躍度は簡易的に0とする

      z_state = [x(5:7); x(8:10); a_vec; j_vec];

      % 3. QP Setup for Snap variables mu = [v2; v3; ddv1]
      % ここで XYZ のペナルティを均等 (平等) にする
      H_qp = diag([1.0, 1.0, 1.0]); 
      
      % ノミナル値へ追従する力を設定
      % 動的拡張により v1 が発散しないよう、ノミナル v1, dv1 との差を ddv1_nom にフィードバック
      K_v1 = 4.0; K_dv1 = 4.0;
      ddv1_nom_adjusted = vf_nom(3) - K_v1*(obj.v1_val - vf_nom(1)) - K_dv1*(obj.dv1_val - vf_nom(2));
      
      mu_nom = [vs_nom(1); vs_nom(2); ddv1_nom_adjusted];
      f_qp = -H_qp * mu_nom;

      A_cbf_all = [];
      b_cbf_all = [];

      % 球体障害物の読み込み
      obs_list = ENVIRONMENT_OBSTACLE_HOCBF_LINK_XY();
      for i = 1:length(obs_list)
        p_obs = obs_list(i).p_obs;
        r_obs = obs_list(i).r_obs_margin;
        obs_params = [p_obs; r_obs];

        % 4次系 HOCBF の極配置 (重根 lambda = 2.0)
        % (s+2)^4 = s^4 + 8s^3 + 24s^2 + 32s + 16
        cbf_gains = [16; 32; 24; 8];

        [A_cbf, b_cbf, h_val] = Sphere_Snap_CBF(z_state, obs_params, cbf_gains);
        
        if h_val < obj.result.min_clearance
            obj.result.min_clearance = h_val;
        end

        A_cbf_all = [A_cbf_all; A_cbf];
        b_cbf_all = [b_cbf_all; b_cbf];
      end

      % 4. Solve QP
      mu_safe = mu_nom;
      if ~isempty(A_cbf_all)
        options = optimoptions('quadprog', 'Display', 'off');
        [mu_opt, ~, exitflag] = quadprog(H_qp, f_qp, A_cbf_all, b_cbf_all, [], [], [], [], [], options);
        if exitflag == 1
          mu_safe = mu_opt;
        end
      end

      % 5. Dynamic Extension (Integration for Z-axis virtual inputs)
      ddv1_safe = mu_safe(3);
      obj.dv1_val = obj.dv1_val + ddv1_safe * dt;
      obj.v1_val = obj.v1_val + obj.dv1_val * dt;

      % 6. Compute Real Inputs using HLC inverse mapping
      vf_safe = [obj.v1_val; obj.dv1_val; ddv1_safe; vf_nom(4)];
      vs_safe = [mu_safe(1); mu_safe(2); vs_nom(3)];

      u_final = Uf(x, xd', vf_safe, P) + Us(x, xd', vf_safe, vs_safe', P);

      % 安全リミット
      obj.result.input = [max(0, min(20, u_final(1))); ...
                          max(-1, min(1, u_final(2))); ...
                          max(-1, min(1, u_final(3))); ...
                          max(-1, min(1, u_final(4)))];
      result = obj.result;
    end
  end
end

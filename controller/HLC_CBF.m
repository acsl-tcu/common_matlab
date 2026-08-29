% classdef HLC_CBF < handle
%   % Hierarchical linearization based controller for a quadcopter
%   properties
%     self
%     result
%     param
%     parameter_order = ["mass","Lx","Ly","lx","ly","jx","jy","jz","gravity","km1","km2","km3","km4","k1","k2","k3","k4"];
%   end
% 
%   methods
%     function obj = HLC_CBF(self,param)
%       obj.self = self;
%       obj.param = param;
%       obj.param.P = self.parameter.get(obj.parameter_order);
%       obj.result.input = zeros(4,1);
%     end
% 
%     function result = do(obj,varargin)
%       model = obj.self.estimator.result;
%       ref = obj.self.reference.result;
%       xd = ref.state.xd;
%       P = obj.param.P;
%       F1 = obj.param.F1;
%       F2 = obj.param.F2;
%       F3 = obj.param.F3;
%       F4 = obj.param.F4;
%       xd=[xd;zeros(20-size(xd,1),1)];% 足りない分は０で埋める．
% 
%       % yaw 角についてボディ座標に合わせることで目標姿勢と現在姿勢の間の2pi問題を緩和
%       % TODO : 本質的にはx-xdを受け付ける関数にして，x-xdの状態で2pi問題を解決すれば良い．
%       Rb0 = RodriguesQuaternion(Eul2Quat([0;0;xd(4)]));
%       x = [R2q(Rb0'*model.state.getq("rotmat"));Rb0'*model.state.p;Rb0'*model.state.v;model.state.w]; % [q, p, v, w]に並べ替え
%       xd(1:3)=Rb0'*xd(1:3);
%       xd(4) = 0;
%       xd(5:7)=Rb0'*xd(5:7);
%       xd(9:11)=Rb0'*xd(9:11);
%       xd(13:15)=Rb0'*xd(13:15);
%       xd(17:19)=Rb0'*xd(17:19);
%       %if isfield(obj.param,'dt')
%       if isfield(varargin{1},'dt') && varargin{1}.dt <= obj.param.dt
%         dt = varargin{1}.dt;
%          vf = Vfd(dt,x,xd',P,F1);
%         vs = Vsd(dt,x,xd',vf,P,F2,F3,F4);
%       else
%         vf = Vf(x,xd',P,F1);
%         vs = Vs(x,xd',vf,P,F2,F3,F4);
%       end
%       tmp = Uf(x,xd',vf,P) + Us(x,xd',vf,vs',P);
%       % max,min are applied for the safty
%       % ---------------------CBF--------------------%
%       obs_list = ENVIRONMENT_OBSTACLE_HOCBF_LINK_XY();
%       if ~isempty(obs_list)
%         if isempty(obj.result.input) || obj.result.input(1) <= 0
%           f_T_curr = P(1) * P(9); % mass * gravity
%         else
%           f_T_curr = obj.result.input(1);
%         end
% 
%         lambda_cbf = 1.0; 
%         A_cbf_all = [];
%         b_cbf_all = [];
% 
%         p_drone = x(5:7);
%         v_drone = x(8:10);
% 
%         % ドローン姿勢の推力軸
%         q_curr = x(1:4);
%         R_curr = RodriguesQuaternion(q_curr);
%         z_B_curr = R_curr(:, 3);
% 
%         for i = 1:length(obs_list)
%           p_obs_world = obs_list(i).p_obs;
%           r_obs_val   = obs_list(i).r_obs_margin;
%           if isfield(obs_list(i), 'v_obs') && ~isempty(obs_list(i).v_obs)
%             v_obs_world = obs_list(i).v_obs;
%           else
%             v_obs_world = zeros(3, 1);
%           end
% 
%           p_obs_rel = Rb0' * p_obs_world;
%           v_obs_rel = Rb0' * v_obs_world;
% 
%           p_rel_vec = p_obs_rel - p_drone;
%           v_rel_vec = v_obs_rel - v_drone;
%           dist_p    = norm(p_rel_vec);
%           speed_v   = norm(v_rel_vec);
% 
%           % 警戒距離外なら完全にスキップ
%           if dist_p > (r_obs_val + 2.0)
%             continue;
%           end
% 
%           % -----------------------------------------------------------
%           % 1. 距離ベースのバックアップCBF (低速時の突入防止ガード)
%           % h_pos = ||p_rel||^2 - r^2 >= 0
%           % dh_pos/dt = -2 * p_rel' * dp >= -gamma * h_pos
%           % -----------------------------------------------------------
%           h_pos = dist_p^2 - r_obs_val^2;
%           % 速度が低い、または障害物至近距離では距離制約を強制印加
%           if speed_v < 0.2 || dist_p < (r_obs_val + 0.5)
%             % 並進加速度への制約: 2 * p_rel' * ( (1/m)*z_B*u1 - g ) >= -gamma1*h_pos - 2*v_rel'*v_rel
%             gamma_pos = 2.0;
%             A_pos = [-2 * (p_rel_vec' * z_B_curr) / P(1),  0,  0,  0];
%             b_pos = 2 * (p_rel_vec' * [0; 0; -P(9)]) + 2 * (v_rel_vec' * v_rel_vec) + gamma_pos * h_pos;
% 
%             A_cbf_all = [A_cbf_all; A_pos];
%             b_cbf_all = [b_cbf_all; b_pos];
%           end
% 
%           % -----------------------------------------------------------
%           % 2. 衝突円錐 ECBF (移動時の回避動作)
%           % -----------------------------------------------------------
%           is_approaching = (dot(p_rel_vec, v_rel_vec) < 0);
%           if (speed_v >= 0.1) && is_approaching
%             % 根号内負値ガード
%             r_cbf_eval = r_obs_val;
%             if dist_p <= r_obs_val
%               r_cbf_eval = dist_p - 1e-4;
%             end
% 
%             obsParam = [p_obs_rel', v_obs_rel', r_cbf_eval];
% 
%             A_cone = real(ECBF_Acbf(x, obsParam, P, f_T_curr));
%             b_cone = real(ECBF_bcbf(x, obsParam, P, f_T_curr, lambda_cbf));
% 
%             if ~any(isnan(A_cone)) && ~any(isnan(b_cone)) && ~any(isinf(A_cone)) && ~any(isinf(b_cone))
%               A_cbf_all = [A_cbf_all; A_cone];
%               b_cbf_all = [b_cbf_all; b_cone];
%             end
%           end
%         end
% 
%         if ~isempty(A_cbf_all)
%           lb = [0; -1; -1; -1];
%           ub = [20; 1;  1;  1];
% 
%           % 姿勢の過度なバタつきを抑える重み
%           H_qp = diag([1.0, 1.0, 1.0, 1.0]);
%           f_qp = -H_qp * real(tmp);
% 
%           options = optimoptions('quadprog', 'Display', 'off', 'Algorithm', 'interior-point-convex');
%           [u_safe, ~, exitflag] = quadprog(real(H_qp), real(f_qp), real(A_cbf_all), real(b_cbf_all), [], [], lb, ub, [], options);
% 
%           if exitflag == 1
%             tmp = u_safe;
%           end
%         end
%       end
%       % --- 最小クリアランス（余裕距離）のログ保存 ---
%       min_dist_to_surface = Inf;
%       if ~isempty(obs_list)
%           p_drone_curr = x(5:7);
%           for i = 1:length(obs_list)
%               p_obs_w = obs_list(i).p_obs;
%               r_m_val = obs_list(i).r_obs_margin;
% 
%               % 中心間距離 - マージン球半径 = 表面までの余裕距離 (>0なら安全領域内)
%               dist_surf = norm(p_obs_w - p_drone_curr) - r_m_val;
%               if dist_surf < min_dist_to_surface
%                   min_dist_to_surface = dist_surf;
%               end
%           end
%       end
%       obj.result.min_clearance = min_dist_to_surface;
%       % --------------------------------------------%
%       obj.result.input = [max(0,min(20,tmp(1)));max(-1,min(1,tmp(2)));max(-1,min(1,tmp(3)));max(-1,min(1,tmp(4)))];
%       result = obj.result;
%     end
%   end
% end
% 



% classdef HLC_CBF < handle
%   % Hierarchical linearization based controller for a quadcopter
%   properties
%     self
%     result
%     param
%     parameter_order = ["mass","Lx","Ly","lx","ly","jx","jy","jz","gravity","km1","km2","km3","km4","k1","k2","k3","k4"];
%   end
% 
%   methods
%     function obj = HLC_CBF(self,param)
%       obj.self = self;
%       obj.param = param;
%       obj.param.P = self.parameter.get(obj.parameter_order);
%       obj.result.input = zeros(4,1);
%     end
% 
%     function result = do(obj,varargin)
%       model = obj.self.estimator.result;
%       ref = obj.self.reference.result;
%       xd = ref.state.xd;
%       P = obj.param.P;
%       F1 = obj.param.F1;
%       F2 = obj.param.F2;
%       F3 = obj.param.F3;
%       F4 = obj.param.F4;
%       xd=[xd;zeros(20-size(xd,1),1)];% 足りない分は０で埋める．
% 
%       % yaw 角についてボディ座標に合わせることで目標姿勢と現在姿勢の間の2pi問題を緩和
%       % TODO : 本質的にはx-xdを受け付ける関数にして，x-xdの状態で2pi問題を解決すれば良い．
%       Rb0 = RodriguesQuaternion(Eul2Quat([0;0;xd(4)]));
%       x = [R2q(Rb0'*model.state.getq("rotmat"));Rb0'*model.state.p;Rb0'*model.state.v;model.state.w]; % [q, p, v, w]に並べ替え
%       xd(1:3)=Rb0'*xd(1:3);
%       xd(4) = 0;
%       xd(5:7)=Rb0'*xd(5:7);
%       xd(9:11)=Rb0'*xd(9:11);
%       xd(13:15)=Rb0'*xd(13:15);
%       xd(17:19)=Rb0'*xd(17:19);
%       %if isfield(obj.param,'dt')
%       if isfield(varargin{1},'dt') && varargin{1}.dt <= obj.param.dt
%         dt = varargin{1}.dt;
%          vf = Vfd(dt,x,xd',P,F1);
%         vs = Vsd(dt,x,xd',vf,P,F2,F3,F4);
%       else
%         vf = Vf(x,xd',P,F1);
%         vs = Vs(x,xd',vf,P,F2,F3,F4);
%       end
%       tmp = Uf(x,xd',vf,P) + Us(x,xd',vf,vs',P);
%       % max,min are applied for the safty
%       % ---------------------CBF--------------------%
%       obs_list = ENVIRONMENT_OBSTACLE_HOCBF_LINK_XY();
%       if ~isempty(obs_list)
%         f_hover = P(1) * P(9);
% 
%         p_drone = x(5:7);
%         v_drone = x(8:10);
% 
%         q_curr   = x(1:4);
%         R_curr   = RodriguesQuaternion(q_curr);
%         z_B_curr = R_curr(:, 3);
% 
%         A_cbf_all = [];
%         b_cbf_all = [];
%         evasion_bias = zeros(4, 1);
% 
%         for i = 1:length(obs_list)
%           p_obs_world = obs_list(i).p_obs;
%           r_obs_val   = obs_list(i).r_obs_margin;
%           if isfield(obs_list(i), 'v_obs') && ~isempty(obs_list(i).v_obs)
%             v_obs_world = obs_list(i).v_obs;
%           else
%             v_obs_world = zeros(3, 1);
%           end
% 
%           p_obs_rel = Rb0' * p_obs_world;
%           v_obs_rel = Rb0' * v_obs_world;
% 
%           p_rel_vec = p_obs_rel - p_drone;
%           v_rel_vec = v_obs_rel - v_drone;
%           dist_p    = norm(p_rel_vec);
%           speed_v   = norm(v_rel_vec);
% 
%           if dist_p > (r_obs_val + 3.0)
%             continue;
%           end
% 
%           % -----------------------------------------------------------
%           % 1. 衝突円錐幾何ベクトル w の算出 (C3BF)
%           % -----------------------------------------------------------
%           r_eval = r_obs_val;
%           if dist_p <= r_obs_val
%             r_eval = dist_p - 1e-3;
%           end
% 
%           norm_v_safe = max(speed_v, 1e-4);
%           cos_phi = sqrt(max(0, dist_p^2 - r_eval^2)) / dist_p;
%           h_cone  = dot(p_rel_vec, v_rel_vec) + dist_p * norm_v_safe * cos_phi;
% 
%           is_approaching = (dot(p_rel_vec, v_rel_vec) < 0);
%           if is_approaching
%             w_vec = p_rel_vec + v_rel_vec * (sqrt(max(0, dist_p^2 - r_eval^2)) / norm_v_safe);
%             gamma_cone = 1.5;
% 
%             % --- 修正: Y軸前進時の横回避 (X変位) は ty (u3: ピッチ) を主軸とする ---
%             A_u1 = - (w_vec' * z_B_curr) / P(1);
%             A_u2 = - w_vec(2) * 5.0;  % ロール (Y方向制御)
%             A_u3 = - w_vec(1) * 10.0; % ピッチ (X方向横回避を強化)
%             A_u4 = 0;
% 
%             A_cone_row = [A_u1, A_u2, A_u3, A_u4];
%             b_cone_val = w_vec' * [0; 0; -P(9)] + gamma_cone * h_cone;
% 
%             A_cbf_all = [A_cbf_all; A_cone_row];
%             b_cbf_all = [b_cbf_all; b_cone_val];
% 
%             % 回避バイアス: X正方向（右）へ機体を傾けるピッチバイアス
%             evasion_bias = evasion_bias + [0; 0; 0.6; 0];
%           end
% 
%           % -----------------------------------------------------------
%           % 2. 距離ハードバリア
%           % -----------------------------------------------------------
%           if dist_p < (r_obs_val + 0.8)
%             h_dist = dist_p^2 - r_obs_val^2;
%             gamma_dist = 1.5;
% 
%             A_d1 = - 2 * (p_rel_vec' * z_B_curr) / P(1);
%             A_d2 = - 2 * p_rel_vec(2) * 5.0;
%             A_d3 = - 2 * p_rel_vec(1) * 10.0;
%             A_d4 = 0;
% 
%             A_dist_row = [A_d1, A_d2, A_d3, A_d4];
%             b_dist_val = 2 * dot(v_rel_vec, v_rel_vec) + 2 * (p_rel_vec' * [0; 0; -P(9)]) + gamma_dist * h_dist;
% 
%             A_cbf_all = [A_cbf_all; A_dist_row];
%             b_cbf_all = [b_cbf_all; b_dist_val];
%           end
%         end
% 
%         % --- 3. Slack付き QP 最適化 ---
%         if ~isempty(A_cbf_all)
%           num_cons = size(A_cbf_all, 1);
% 
%           H_u = diag([1.0, 0.5, 0.1, 1.0]); % ピッチ(u3)の重みを大幅に下げて傾きやすくする
%           w_xi = 1000.0;
% 
%           H_qp = blkdiag(H_u, w_xi * eye(num_cons));
% 
%           u_nominal_clamped = [tmp(1); max(-1, min(1, tmp(2))); max(-1, min(1, tmp(3))); max(-1, min(1, tmp(4)))];
%           u_target = u_nominal_clamped + evasion_bias;
%           f_qp = [-H_u * u_target; zeros(num_cons, 1)];
% 
%           A_qp = [A_cbf_all, -eye(num_cons)];
%           b_qp = b_cbf_all;
% 
%           lb = [0.70 * f_hover; -1.0; -1.0; -1.0; zeros(num_cons, 1)];
%           ub = [20.0;            1.0;  1.0;  1.0; inf(num_cons, 1)];
% 
%           options = optimoptions('quadprog', 'Display', 'off', 'Algorithm', 'interior-point-convex');
%           [z_safe, ~, exitflag] = quadprog(real(H_qp), real(f_qp), real(A_qp), real(b_qp), [], [], lb, ub, [], options);
% 
%           % --- 診断用 printf ---
%           fprintf('[CBF 診断] 距離: %0.2fm | h_cone: %+0.3f | QP Status: %d\n', dist_p, h_cone, exitflag);
%           fprintf('  公称入力 tmp : [T=%0.2f, tx=%+0.2f, ty=%+0.2f, tz=%+0.2f]\n', tmp(1), tmp(2), tmp(3), tmp(4));
%           if exitflag == 1
%             u_safe = z_safe(1:4);
%             fprintf('  安全入力 u*  : [T=%0.2f, tx=%+0.2f, ty=%+0.2f, tz=%+0.2f] (回避適用)\n', u_safe(1), u_safe(2), u_safe(3), u_safe(4));
%             tmp = u_safe;
%           else
%             fprintf('  ⚠️ QP解なし (exitflag=%d) -> 公称入力をそのまま維持\n', exitflag);
%           end
%         end
%       end
%       % --- 最小クリアランス（余裕距離）のログ保存 ---
%       min_dist_to_surface = Inf;
%       if ~isempty(obs_list)
%           p_drone_curr = x(5:7);
%           for i = 1:length(obs_list)
%               p_obs_w = obs_list(i).p_obs;
%               r_m_val = obs_list(i).r_obs_margin;
% 
%               % 中心間距離 - マージン球半径 = 表面までの余裕距離 (>0なら安全領域内)
%               dist_surf = norm(p_obs_w - p_drone_curr) - r_m_val;
%               if dist_surf < min_dist_to_surface
%                   min_dist_to_surface = dist_surf;
%               end
%           end
%       end
%       obj.result.min_clearance = min_dist_to_surface;
%       % --------------------------------------------%
%       obj.result.input = [max(0,min(20,tmp(1)));max(-1,min(1,tmp(2)));max(-1,min(1,tmp(3)));max(-1,min(1,tmp(4)))];
%       result = obj.result;
%     end
%   end
% end
% 


classdef HLC_CBF < handle
  % Hierarchical linearization based controller for a quadcopter
  properties
    self
    result
    param
    parameter_order = ["mass","Lx","Ly","lx","ly","jx","jy","jz","gravity","km1","km2","km3","km4","k1","k2","k3","k4"];
  end

  methods
    function obj = HLC_CBF(self,param)
      obj.self = self;
      obj.param = param;
      obj.param.P = self.parameter.get(obj.parameter_order);
      obj.result.input = zeros(4,1);
    end

    function result = do(obj,varargin)
      model = obj.self.estimator.result;
      ref = obj.self.reference.result;
      xd = ref.state.xd;
      P = obj.param.P;
      F1 = obj.param.F1;
      F2 = obj.param.F2;
      F3 = obj.param.F3;
      F4 = obj.param.F4;
      xd=[xd;zeros(20-size(xd,1),1)];% 足りない分は０で埋める．

      % yaw 角についてボディ座標に合わせることで目標姿勢と現在姿勢の間の2pi問題を緩和
      % TODO : 本質的にはx-xdを受け付ける関数にして，x-xdの状態で2pi問題を解決すれば良い．
      Rb0 = RodriguesQuaternion(Eul2Quat([0;0;xd(4)]));
      x = [R2q(Rb0'*model.state.getq("rotmat"));Rb0'*model.state.p;Rb0'*model.state.v;model.state.w]; % [q, p, v, w]に並べ替え
      xd(1:3)=Rb0'*xd(1:3);
      xd(4) = 0;
      xd(5:7)=Rb0'*xd(5:7);
      xd(9:11)=Rb0'*xd(9:11);
      xd(13:15)=Rb0'*xd(13:15);
      xd(17:19)=Rb0'*xd(17:19);
      %if isfield(obj.param,'dt')
      if isfield(varargin{1},'dt') && varargin{1}.dt <= obj.param.dt
        dt = varargin{1}.dt;
         vf = Vfd(dt,x,xd',P,F1);
        vs = Vsd(dt,x,xd',vf,P,F2,F3,F4);
      else
        vf = Vf(x,xd',P,F1);
        vs = Vs(x,xd',vf,P,F2,F3,F4);
      end
      tmp = Uf(x,xd',vf,P) + Us(x,xd',vf,vs',P);
      % max,min are applied for the safty
      % ---------------------CBF--------------------%
      % --- 1. ノミナル入力の事前クリッピング (0~20N, 各トルク -1~1 Nm) ---
      u_nom = [max(0.0, min(20.0, tmp(1))); ...
               max(-1.0, min(1.0, tmp(2))); ...
               max(-1.0, min(1.0, tmp(3))); ...
               max(-1.0, min(1.0, tmp(4)))];

      f_hover = P(1) * P(9);
      if isempty(obj.result.input) || obj.result.input(1) <= 0
        f_T_curr = f_hover;
      else
        f_T_curr = obj.result.input(1);
      end

      obs_list = ENVIRONMENT_OBSTACLE_HOCBF_LINK_XY();
      if ~isempty(obs_list)
        p_drone = x(5:7);
        v_drone = x(8:10);

        lambda_cbf = 2.0; 
        A_obs_all = [];
        b_obs_all = [];

        for i = 1:length(obs_list)
          p_obs_world = obs_list(i).p_obs;
          r_obs_val   = obs_list(i).r_obs_margin;
          if isfield(obs_list(i), 'v_obs') && ~isempty(obs_list(i).v_obs)
            v_obs_world = obs_list(i).v_obs;
          else
            v_obs_world = zeros(3, 1);
          end

          p_obs_rel = Rb0' * p_obs_world;
          v_obs_rel = Rb0' * v_obs_world;
          
          p_rel_vec = p_obs_rel - p_drone;
          v_rel_vec = v_obs_rel - v_drone;
          dist_p    = norm(p_rel_vec);

          if dist_p > (r_obs_val + 3.0)
            continue;
          end

          % 衝突円錐 ECBF
          if dot(p_rel_vec, v_rel_vec) < 0
            r_cbf_eval = r_obs_val;
            if dist_p <= r_obs_val
              r_cbf_eval = dist_p - 1e-4;
            end

            obsParam = [p_obs_rel', v_obs_rel', r_cbf_eval];
            A_cone = real(ECBF_Acbf(x, obsParam, P, f_T_curr));
            b_cone = real(ECBF_bcbf(x, obsParam, P, f_T_curr, lambda_cbf));

            if ~any(isnan(A_cone)) && ~any(isnan(b_cone)) && ~any(isinf(A_cone)) && ~any(isinf(b_cone))
              % --- スケーリング正規化 (トルク項のオーダー 10^5 を推力と同等スケールに調整) ---
              scale_tau = norm(A_cone(2:4)) + 1e-6;
              A_cone_scaled = [A_cone(1), A_cone(2:4) * (5.0 / scale_tau)];
              
              A_obs_all = [A_obs_all; A_cone_scaled];
              b_obs_all = [b_obs_all; b_cone];
            end
          end
        end

        % --- 2. 姿勢角制限 CBF (ロール・ピッチ ±25 deg) ---
        max_tilt  = deg2rad(25); 
        gamma_att = 3.0;
        A_att = real(Attitude_CBF_Acbf(x, P));
        b_att = real(Attitude_CBF_bcbf(x, P, max_tilt, gamma_att));

        % --- 3. 階層型 QP 最適化 (推力下限は 0.0) ---
        n_obs = size(A_obs_all, 1);

        if n_obs > 0
          A_qp = [A_att,     zeros(size(A_att, 1), n_obs); ...
                  A_obs_all, -eye(n_obs)];
          b_qp = [b_att; b_obs_all];

          H_u  = eye(4); 
          w_xi = 500.0;
          H_qp = blkdiag(H_u, w_xi * eye(n_obs));
          f_qp = [-H_u * u_nom; zeros(n_obs, 1)];

          % 推力下限は 0.0
          lb = [0.0;  -1.0; -1.0; -1.0; zeros(n_obs, 1)];
          ub = [20.0;  1.0;  1.0;  1.0; inf(n_obs, 1)];

          options = optimoptions('quadprog', 'Display', 'off', 'Algorithm', 'interior-point-convex');
          [z_safe, ~, exitflag] = quadprog(real(H_qp), real(f_qp), real(A_qp), real(b_qp), [], [], lb, ub, [], options);

          if exitflag == 1
            u_safe = z_safe(1:4);
            tmp = u_safe;
          else
            [u_att_safe, ~, ef_att] = quadprog(H_u, -H_u*u_nom, A_att, b_att, [], [], lb(1:4), ub(1:4), [], options);
            if ef_att == 1
              tmp = u_att_safe;
            else
              tmp = u_nom;
            end
          end
        else
          [u_safe, ~, exitflag] = quadprog(eye(4), -u_nom, A_att, b_att, [], [], [0.0; -1; -1; -1], [20; 1; 1; 1], [], optimoptions('quadprog','Display','off'));
          if exitflag == 1
            tmp = u_safe;
          else
            tmp = u_nom;
          end
        end

        % --- 診断用 printf ---
        if n_obs > 0
          h_diag = real(ECBF_h(x, obsParam, P));
          fprintf('[CBF 診断] 距離: %0.2fm | h_cone: %+0.3f | QP Status: %d\n', dist_p, h_diag, exitflag);
          fprintf('  ノミナル入力 : [T=%0.2f, tx=%+0.2f, ty=%+0.2f, tz=%+0.2f]\n', u_nom(1), u_nom(2), u_nom(3), u_nom(4));
          fprintf('  安全入力 u*  : [T=%0.2f, tx=%+0.2f, ty=%+0.2f, tz=%+0.2f]\n', tmp(1), tmp(2), tmp(3), tmp(4));
        end
      else
        tmp = u_nom;
      end

      % --- 最小クリアランス（余裕距離）のログ保存 ---
      min_dist_to_surface = Inf;
      if ~isempty(obs_list)
        p_drone_curr = x(5:7);
        for i = 1:length(obs_list)
          p_obs_w = obs_list(i).p_obs;
          r_m_val = obs_list(i).r_obs_margin;
          dist_surf = norm(p_obs_w - p_drone_curr) - r_m_val;
          if dist_surf < min_dist_to_surface
            min_dist_to_surface = dist_surf;
          end
        end
      end
      obj.result.min_clearance = min_dist_to_surface;

      % 最終入力
      obj.result.input = [max(0.0, min(20.0, tmp(1))); ...
                          max(-1.0, min(1.0,  tmp(2))); ...
                          max(-1.0, min(1.0,  tmp(3))); ...
                          max(-1.0, min(1.0,  tmp(4)))];
      result = obj.result;
    end
  end
end
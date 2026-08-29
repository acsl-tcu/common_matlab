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
%       % --- 1. ノミナル入力の事前クリッピング (0~20N, 各トルク -1~1 Nm) ---
%       u_nom = [max(0.0, min(20.0, tmp(1))); ...
%                max(-1.0, min(1.0, tmp(2))); ...
%                max(-1.0, min(1.0, tmp(3))); ...
%                max(-1.0, min(1.0, tmp(4)))];
% 
%       f_hover = P(1) * P(9);
%       if isempty(obj.result.input) || obj.result.input(1) <= 0
%         f_T_curr = f_hover;
%       else
%         f_T_curr = obj.result.input(1);
%       end
% 
%       obs_list = ENVIRONMENT_OBSTACLE_HOCBF_LINK_XY();
%       if ~isempty(obs_list)
%         p_drone = x(5:7);
%         v_drone = x(8:10);
% 
%         lambda_cbf = 2.0; 
%         A_obs_all = [];
%         b_obs_all = [];
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
% 
%           if dist_p > (r_obs_val + 3.0)
%             continue;
%           end
% 
%           % 衝突円錐 ECBF
%           if dot(p_rel_vec, v_rel_vec) < 0
%             r_cbf_eval = r_obs_val;
%             if dist_p <= r_obs_val
%               r_cbf_eval = dist_p - 1e-4;
%             end
% 
%             obsParam = [p_obs_rel', v_obs_rel', r_cbf_eval];
%             A_cone = real(ECBF_Acbf(x, obsParam, P, f_T_curr));
%             b_cone = real(ECBF_bcbf(x, obsParam, P, f_T_curr, lambda_cbf));
% 
%             if ~any(isnan(A_cone)) && ~any(isnan(b_cone)) && ~any(isinf(A_cone)) && ~any(isinf(b_cone))
%               % --- スケーリング正規化 (トルク項のオーダー 10^5 を推力と同等スケールに調整) ---
%               scale_tau = norm(A_cone(2:4)) + 1e-6;
%               A_cone_scaled = [A_cone(1), A_cone(2:4) * (5.0 / scale_tau)];
% 
%               A_obs_all = [A_obs_all; A_cone_scaled];
%               b_obs_all = [b_obs_all; b_cone];
%             end
%           end
%         end
% 
%         % --- 2. 姿勢角制限 CBF (ロール・ピッチ ±25 deg) ---
%         max_tilt  = deg2rad(25); 
%         gamma_att = 3.0;
%         A_att = real(Attitude_CBF_Acbf(x, P));
%         b_att = real(Attitude_CBF_bcbf(x, P, max_tilt, gamma_att));
% 
%         % --- 3. 階層型 QP 最適化 (推力下限は 0.0) ---
%         n_obs = size(A_obs_all, 1);
% 
%         if n_obs > 0
%           A_qp = [A_att,     zeros(size(A_att, 1), n_obs); ...
%                   A_obs_all, -eye(n_obs)];
%           b_qp = [b_att; b_obs_all];
% 
%           % H_u  = eye(4); 
%           H_u  = diag([50.0, 0.01, 0.01, 1.0]);
%           w_xi = 500.0;
%           H_qp = blkdiag(H_u, w_xi * eye(n_obs));
%           f_qp = [-H_u * u_nom; zeros(n_obs, 1)];
% 
%           % 推力下限は 0.0
%           lb = [0.0;  -1.0; -1.0; -1.0; zeros(n_obs, 1)];
%           ub = [20.0;  1.0;  1.0;  1.0; inf(n_obs, 1)];
% 
%           options = optimoptions('quadprog', 'Display', 'off', 'Algorithm', 'interior-point-convex');
%           [z_safe, ~, exitflag] = quadprog(real(H_qp), real(f_qp), real(A_qp), real(b_qp), [], [], lb, ub, [], options);
% 
%           if exitflag == 1
%             u_safe = z_safe(1:4);
%             tmp = u_safe;
%           else
%             [u_att_safe, ~, ef_att] = quadprog(H_u, -H_u*u_nom, A_att, b_att, [], [], lb(1:4), ub(1:4), [], options);
%             if ef_att == 1
%               tmp = u_att_safe;
%             else
%               tmp = u_nom;
%             end
%           end
%         else
%           [u_safe, ~, exitflag] = quadprog(eye(4), -u_nom, A_att, b_att, [], [], [0.0; -1; -1; -1], [20; 1; 1; 1], [], optimoptions('quadprog','Display','off'));
%           if exitflag == 1
%             tmp = u_safe;
%           else
%             tmp = u_nom;
%           end
%         end
% 
%         % --- 診断用 printf ---
%         if n_obs > 0
%           h_diag = real(ECBF_h(x, obsParam, P));
%           fprintf('[CBF 診断] 距離: %0.2fm | h_cone: %+0.3f | QP Status: %d\n', dist_p, h_diag, exitflag);
%           fprintf('  ノミナル入力 : [T=%0.2f, tx=%+0.2f, ty=%+0.2f, tz=%+0.2f]\n', u_nom(1), u_nom(2), u_nom(3), u_nom(4));
%           fprintf('  安全入力 u*  : [T=%0.2f, tx=%+0.2f, ty=%+0.2f, tz=%+0.2f]\n', tmp(1), tmp(2), tmp(3), tmp(4));
%         end
%       else
%         tmp = u_nom;
%       end
% 
%       % --- 最小クリアランス（余裕距離）のログ保存 ---
%       min_dist_to_surface = Inf;
%       if ~isempty(obs_list)
%         p_drone_curr = x(5:7);
%         for i = 1:length(obs_list)
%           p_obs_w = obs_list(i).p_obs;
%           r_m_val = obs_list(i).r_obs_margin;
%           dist_surf = norm(p_obs_w - p_drone_curr) - r_m_val;
%           if dist_surf < min_dist_to_surface
%             min_dist_to_surface = dist_surf;
%           end
%         end
%       end
%       obj.result.min_clearance = min_dist_to_surface;
% 
%       % 最終入力
%       obj.result.input = [max(0.0, min(20.0, tmp(1))); ...
%                           max(-1.0, min(1.0,  tmp(2))); ...
%                           max(-1.0, min(1.0,  tmp(3))); ...
%                           max(-1.0, min(1.0,  tmp(4)))];
%       result = obj.result;
%     end
%   end
% end

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
%       % max,min are applied for the safty
%       % ここに入れるCBFを
% 
%       % --- 1. ノミナル入力の事前クリッピング ---
%       u_nom = [max(0.0, min(20.0, tmp(1))); ...
%                max(-1.0, min(1.0, tmp(2))); ...
%                max(-1.0, min(1.0, tmp(3))); ...
%                max(-1.0, min(1.0, tmp(4)))];
%       f_hover = P(1) * P(9);
%       if isempty(obj.result.input) || obj.result.input(1) <= 0
%         f_T_curr = f_hover;
%       else
%         f_T_curr = obj.result.input(1);
%       end
%       obs_list = ENVIRONMENT_OBSTACLE_HOCBF_LINK_XY();
%       if ~isempty(obs_list)
%         p_drone = x(5:7);
%         v_drone = x(8:10);
%         w_drone = x(11:13);
%         q_curr   = x(1:4);
%         phi_deg   = rad2deg(atan2(2*(q_curr(1)*q_curr(2) + q_curr(3)*q_curr(4)), q_curr(1)^2 - q_curr(2)^2 - q_curr(3)^2 + q_curr(4)^2));
%         theta_deg = rad2deg(asin(max(-1.0, min(1.0, 2*(q_curr(1)*q_curr(3) - q_curr(2)*q_curr(4))))));
%         % =========================================================================
%         % 【3次元回避チューニングパラメータ】ここを調整してバランスを取る
%         % =========================================================================
%         lambda_cbf = 3.0;      % 接近時の制約立ち上がり強度 (標準: 2.0 ~ 3.5)
%         gain_T     = 0.30;     % 推力(Z)の制約寄与度 (0.05=ほぼZ無効 ~ 0.50=Zも強めに使う)
%         gain_tau   = 40.0;     % トルク(XY)の制約寄与度 (25.0=穏やか ~ 60.0=深く傾く)
% 
%         w_T        = 25.0;      % コスト関数での推力変更ペナルティ (小さいほどZが動きやすい)
%         w_tau      = 0.05;     % コスト関数でのトルク変更ペナルティ (小さいほどXYに傾きやすい)
%         % =========================================================================
%         A_obs_all = [];
%         b_obs_all = [];
%         for i = 1:length(obs_list)
%           p_obs_world = obs_list(i).p_obs;
%           r_obs_val   = obs_list(i).r_obs_margin;
%           if isfield(obs_list(i), 'v_obs') && ~isempty(obs_list(i).v_obs)
%             v_obs_world = obs_list(i).v_obs;
%           else
%             v_obs_world = zeros(3, 1);
%           end
%           p_obs_rel = Rb0' * p_obs_world;
%           v_obs_rel = Rb0' * v_obs_world;
% 
%           p_rel_vec = p_obs_rel - p_drone;
%           v_rel_vec = v_obs_rel - v_drone;
%           dist_p    = norm(p_rel_vec);
%           if dist_p > (r_obs_val + 3.0)
%             continue;
%           end
% 
%           % --- 1. 衝突円錐 ECBF (接近時) ---
%           if dot(p_rel_vec, v_rel_vec) < 0
%             r_cbf_eval = r_obs_val;
%             if dist_p <= r_obs_val
%               r_cbf_eval = dist_p - 1e-4;
%             end
%             obsParam = [p_obs_rel', v_obs_rel', r_cbf_eval];
%             A_cone = real(ECBF_Acbf(x, obsParam, P, f_T_curr));
%             b_cone = real(ECBF_bcbf(x, obsParam, P, f_T_curr, lambda_cbf));
%             if ~any(isnan(A_cone)) && ~any(isnan(b_cone)) && ~any(isinf(A_cone)) && ~any(isinf(b_cone))
%               scale_tau = norm(A_cone(2:4)) + 1e-6;
% 
%               A_cone_scaled = [gain_T * A_cone(1), A_cone(2:4) * (gain_tau / scale_tau)];
% 
%               A_obs_all = [A_obs_all; A_cone_scaled];
%               b_obs_all = [b_obs_all; b_cone];
%             end
%           end
% 
%           % --- 2. 位置依存の距離 ECBF (速度の向きに関わらず常時併用) ---
%           h_dist = dist_p^2 - r_obs_val^2;
%           gamma_pos = 2.0;
%           R_curr   = RodriguesQuaternion(q_curr);
%           z_B_curr = R_curr(:, 3);
% 
%           A_dist_1 = -2.0 * (p_rel_vec' * z_B_curr) / P(1);
%           A_dist_2 = -2.0 * p_rel_vec(2);
%           A_dist_3 = -2.0 * p_rel_vec(1);
%           A_dist_4 = 0.0;
% 
%           A_dist_raw = [A_dist_1, A_dist_2, A_dist_3, A_dist_4];
%           b_dist = 2.0 * dot(v_rel_vec, v_rel_vec) + 2.0 * (p_rel_vec' * [0; 0; -P(9)]) ...
%                    + 2.0 * gamma_pos * (-2.0 * dot(p_rel_vec, v_rel_vec)) + (gamma_pos^2) * h_dist;
% 
%           scale_tau_dist = norm(A_dist_raw(2:4)) + 1e-6;
%           A_dist_scaled = [gain_T * A_dist_raw(1), A_dist_raw(2:4) * (gain_tau / scale_tau_dist)];
% 
%           A_obs_all = [A_obs_all; A_dist_scaled];
%           b_obs_all = [b_obs_all; b_dist];
%         end
%         % --- 2. 姿勢角制限 CBF (ロール・ピッチ ±40 deg) ---
%         max_tilt  = deg2rad(40); 
%         gamma_att = 3.0;
%         A_att = real(Attitude_CBF_Acbf(x, P));
%         b_att = real(Attitude_CBF_bcbf(x, P, max_tilt, gamma_att));
% 
%         % --- 3. 階層型 完全スラック付き QP 最適化 ---
%         n_obs = size(A_obs_all, 1);
%         n_att = size(A_att, 1);
% 
%         if n_obs > 0
%           % 最適化変数 z = [u (4x1); xi_obs (n_obs x 1); xi_att (n_att x 1)]
%           num_slacks = n_obs + n_att;
%           A_qp = [A_obs_all, -eye(n_obs), zeros(n_obs, n_att); ...
%                   A_att,     zeros(n_att, n_obs), -eye(n_att)];
%           b_qp = [b_obs_all; b_att];
% 
%           H_u      = diag([w_T, w_tau, w_tau, 1.0]); 
%           w_xi_obs = 500.0;
%           w_xi_att = 50000.0; % 姿勢スラックは超高ペナルティ
%           H_qp     = blkdiag(H_u, w_xi_obs * eye(n_obs), w_xi_att * eye(n_att));
%           f_qp     = [-H_u * u_nom; zeros(num_slacks, 1)];
% 
%           lb = [0.0;  -1.0; -1.0; -1.0; zeros(num_slacks, 1)];
%           ub = [20.0;  1.0;  1.0;  1.0; inf(num_slacks, 1)];
% 
%           options = optimoptions('quadprog', 'Display', 'off', 'Algorithm', 'interior-point-convex');
%           [z_safe, ~, exitflag] = quadprog(real(H_qp), real(f_qp), real(A_qp), real(b_qp), [], [], lb, ub, [], options);
%           if exitflag == 1
%             u_safe = z_safe(1:4);
%             tmp = u_safe;
%           else
%             tmp = u_nom;
%           end
%         else
%           % 警戒範囲外: 姿勢制約にもスラックを付与して最適化
%           A_qp_att = [A_att, -eye(n_att)];
%           b_qp_att = b_att;
%           H_qp_att = blkdiag(eye(4), 50000.0 * eye(n_att));
%           f_qp_att = [-u_nom; zeros(n_att, 1)];
%           lb_att   = [0.0; -1.0; -1.0; -1.0; zeros(n_att, 1)];
%           ub_att   = [20.0; 1.0;  1.0;  1.0; inf(n_att, 1)];
% 
%           [z_safe, ~, exitflag] = quadprog(real(H_qp_att), real(f_qp_att), real(A_qp_att), real(b_qp_att), [], [], lb_att, ub_att, [], optimoptions('quadprog','Display','off'));
%           if exitflag == 1
%             tmp = z_safe(1:4);
%           else
%             tmp = u_nom;
%           end
%         end
%         % --- 診断 printf ---
%         if n_obs > 0
%           c_T  = A_obs_all(1, 1) * tmp(1);
%           c_tx = A_obs_all(1, 2) * tmp(2);
%           c_ty = A_obs_all(1, 3) * tmp(3);
%           total_lhs = c_T + c_tx + c_ty;
%           fprintf('[CBF 診断] 距離: %0.2fm | 制約数: %d | QP: %d\n', dist_p, n_obs, exitflag);
%           fprintf('  姿勢実測 : ロール=%+0.1f deg, ピッチ=%+0.1f deg | 角速度=[%+0.2f, %+0.2f]\n', phi_deg, theta_deg, w_drone(1), w_drone(2));
%           fprintf('  制御入力 : T=%0.2f, tx=%+0.2f, ty=%+0.2f\n', tmp(1), tmp(2), tmp(3));
%           fprintf('  制約寄与 : [推力T: %+0.2f, ロールtx: %+0.2f, ピッチty: %+0.2f] => 合計=%+0.2f <= b=%+0.2f\n\n', ...
%                   c_T, c_tx, c_ty, total_lhs, b_obs_all(1));
%         end
%       else
%         tmp = u_nom;
%       end
%       % --- 最小クリアランス（余裕距離）のログ保存 ---
%       min_dist_to_surface = Inf;
%       if ~isempty(obs_list)
%         p_drone_curr = x(5:7);
%         for i = 1:length(obs_list)
%           p_obs_w = obs_list(i).p_obs;
%           r_m_val = obs_list(i).r_obs_margin;
%           dist_surf = norm(p_obs_w - p_drone_curr) - r_m_val;
%           if dist_surf < min_dist_to_surface
%             min_dist_to_surface = dist_surf;
%           end
%         end
%       end
%       obj.result.min_clearance = min_dist_to_surface;
%       % 最終入力
%       obj.result.input = [max(0.0, min(20.0, tmp(1))); ...
%                           max(-1.0, min(1.0,  tmp(2))); ...
%                           max(-1.0, min(1.0,  tmp(3))); ...
%                           max(-1.0, min(1.0,  tmp(4)))];
%       result = obj.result;
%     end
%   end
% end

% classdef HLC_CBF < handle
%   % Hierarchical linearization based controller for a quadcopter
%   properties
%     self
%     result
%     param
%     parameter_order = ["mass","Lx","Ly","lx","ly","jx","jy","jz","gravity","km1","km2","km3","km4","k1","k2","k3","k4"];
%   end
%   methods
%     function obj = HLC_CBF(self,param)
%       obj.self = self;
%       obj.param = param;
%       obj.param.P = self.parameter.get(obj.parameter_order);
%       obj.result.input = zeros(4,1);
%     end
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
%       % max,min are applied for the safty
%       % ここに入れるCBFを
% 
%       f_hover = P(1) * P(9);
%       T_min_flight = 0.70 * f_hover; % 【重要】空中での自由落下・失速を防ぐ最低推力下限 (~5.05 N)
% 
%       % --- 1. ノミナル入力の事前クリッピング ---
%       u_nom = [max(T_min_flight, min(20.0, tmp(1))); ...
%                max(-1.0, min(1.0, tmp(2))); ...
%                max(-1.0, min(1.0, tmp(3))); ...
%                max(-1.0, min(1.0, tmp(4)))];
% 
%       if isempty(obj.result.input) || obj.result.input(1) <= 0
%         f_T_curr = f_hover;
%       else
%         f_T_curr = obj.result.input(1);
%       end
%       obs_list = ENVIRONMENT_OBSTACLE_HOCBF_LINK_XY();
%       if ~isempty(obs_list)
%         p_drone = x(5:7);
%         v_drone = x(8:10);
%         w_drone = x(11:13);
%         q_curr   = x(1:4);
%         phi_deg   = rad2deg(atan2(2*(q_curr(1)*q_curr(2) + q_curr(3)*q_curr(4)), q_curr(1)^2 - q_curr(2)^2 - q_curr(3)^2 + q_curr(4)^2));
%         theta_deg = rad2deg(asin(max(-1.0, min(1.0, 2*(q_curr(1)*q_curr(3) - q_curr(2)*q_curr(4))))));
%         % =========================================================================
%         % 【3次元回避チューニングパラメータ】ここを調整してバランスを取る
%         % =========================================================================
%         lambda_cbf = 3.0;      % 接近時の制約立ち上がり強度 (標準: 2.0 ~ 3.5)
%         gain_T     = 0.30;     % 推力(Z)の制約寄与度 (0.05=ほぼZ無効 ~ 0.50=Zも強めに使う)
%         gain_tau   = 40.0;     % トルク(XY)の制約寄与度 (25.0=穏やか ~ 60.0=深く傾く)
% 
%         w_T        = 25.0;      % コスト関数での推力変更ペナルティ (小さいほどZが動きやすい)
%         w_tau      = 0.05;     % コスト関数でのトルク変更ペナルティ (小さいほどXYに傾きやすい)
%         % =========================================================================
%         A_obs_all = [];
%         b_obs_all = [];
%         for i = 1:length(obs_list)
%           p_obs_world = obs_list(i).p_obs;
%           r_obs_val   = obs_list(i).r_obs_margin;
%           if isfield(obs_list(i), 'v_obs') && ~isempty(obs_list(i).v_obs)
%             v_obs_world = obs_list(i).v_obs;
%           else
%             v_obs_world = zeros(3, 1);
%           end
%           p_obs_rel = Rb0' * p_obs_world;
%           v_obs_rel = Rb0' * v_obs_world;
% 
%           p_rel_vec = p_obs_rel - p_drone;
%           v_rel_vec = v_obs_rel - v_drone;
%           dist_p    = norm(p_rel_vec);
%           if dist_p > (r_obs_val + 3.0)
%             continue;
%           end
%           % --- 1. 衝突円錐 ECBF (接近時) ---
%           if dot(p_rel_vec, v_rel_vec) < 0
%             r_cbf_eval = r_obs_val;
%             if dist_p <= r_obs_val
%               r_cbf_eval = dist_p - 1e-4;
%             end
%             obsParam = [p_obs_rel', v_obs_rel', r_cbf_eval];
%             A_cone = real(ECBF_Acbf(x, obsParam, P, f_T_curr));
%             b_cone = real(ECBF_bcbf(x, obsParam, P, f_T_curr, lambda_cbf));
%             if ~any(isnan(A_cone)) && ~any(isnan(b_cone)) && ~any(isinf(A_cone)) && ~any(isinf(b_cone))
%               scale_tau = norm(A_cone(2:4)) + 1e-6;
% 
%               A_cone_scaled = [gain_T * A_cone(1), A_cone(2:4) * (gain_tau / scale_tau)];
% 
%               A_obs_all = [A_obs_all; A_cone_scaled];
%               b_obs_all = [b_obs_all; b_cone];
%             end
%           end
%           % --- 2. 位置依存の距離 ECBF (速度の向きに関わらず常時併用) ---
%           h_dist = dist_p^2 - r_obs_val^2;
%           gamma_pos = 2.0;
%           R_curr   = RodriguesQuaternion(q_curr);
%           z_B_curr = R_curr(:, 3);
% 
%           A_dist_1 = -2.0 * (p_rel_vec' * z_B_curr) / P(1);
%           A_dist_2 = -2.0 * p_rel_vec(2);
%           A_dist_3 = -2.0 * p_rel_vec(1);
%           A_dist_4 = 0.0;
% 
%           A_dist_raw = [A_dist_1, A_dist_2, A_dist_3, A_dist_4];
%           b_dist = 2.0 * dot(v_rel_vec, v_rel_vec) + 2.0 * (p_rel_vec' * [0; 0; -P(9)]) ...
%                    + 2.0 * gamma_pos * (-2.0 * dot(p_rel_vec, v_rel_vec)) + (gamma_pos^2) * h_dist;
% 
%           scale_tau_dist = norm(A_dist_raw(2:4)) + 1e-6;
%           A_dist_scaled = [gain_T * A_dist_raw(1), A_dist_raw(2:4) * (gain_tau / scale_tau_dist)];
% 
%           A_obs_all = [A_obs_all; A_dist_scaled];
%           b_obs_all = [b_obs_all; b_dist];
%         end
%         % --- 2. 姿勢角制限 CBF (ロール・ピッチ ±40 deg) ---
%         max_tilt  = deg2rad(40); 
%         gamma_att = 3.0;
%         A_att = real(Attitude_CBF_Acbf(x, P));
%         b_att = real(Attitude_CBF_bcbf(x, P, max_tilt, gamma_att));
% 
%         % --- 3. 階層型 完全スラック付き QP 最適化 ---
%         n_obs = size(A_obs_all, 1);
%         n_att = size(A_att, 1);
% 
%         if n_obs > 0
%           % 最適化変数 z = [u (4x1); xi_obs (n_obs x 1); xi_att (n_att x 1)]
%           num_slacks = n_obs + n_att;
%           A_qp = [A_obs_all, -eye(n_obs), zeros(n_obs, n_att); ...
%                   A_att,     zeros(n_att, n_obs), -eye(n_att)];
%           b_qp = [b_obs_all; b_att];
% 
%           H_u      = diag([w_T, w_tau, w_tau, 1.0]); 
%           w_xi_obs = 500.0;
%           w_xi_att = 50000.0; % 姿勢スラックは超高ペナルティ
%           H_qp     = blkdiag(H_u, w_xi_obs * eye(n_obs), w_xi_att * eye(n_att));
%           f_qp     = [-H_u * u_nom; zeros(num_slacks, 1)];
% 
%           % 推力下限を T_min_flight に設定
%           lb = [T_min_flight;  -1.0; -1.0; -1.0; zeros(num_slacks, 1)];
%           ub = [20.0;           1.0;  1.0;  1.0; inf(num_slacks, 1)];
% 
%           options = optimoptions('quadprog', 'Display', 'off', 'Algorithm', 'interior-point-convex');
%           [z_safe, ~, exitflag] = quadprog(real(H_qp), real(f_qp), real(A_qp), real(b_qp), [], [], lb, ub, [], options);
%           if exitflag == 1
%             u_safe = z_safe(1:4);
%             tmp = u_safe;
%           else
%             tmp = u_nom;
%           end
%         else
%           % 警戒範囲外: 姿勢制約にもスラックを付与して最適化
%           A_qp_att = [A_att, -eye(n_att)];
%           b_qp_att = b_att;
%           H_qp_att = blkdiag(eye(4), 50000.0 * eye(n_att));
%           f_qp_att = [-u_nom; zeros(n_att, 1)];
%           lb_att   = [T_min_flight; -1.0; -1.0; -1.0; zeros(n_att, 1)];
%           ub_att   = [20.0;          1.0;  1.0;  1.0; inf(n_att, 1)];
% 
%           [z_safe, ~, exitflag] = quadprog(real(H_qp_att), real(f_qp_att), real(A_qp_att), real(b_qp_att), [], [], lb_att, ub_att, [], optimoptions('quadprog','Display','off'));
%           if exitflag == 1
%             tmp = z_safe(1:4);
%           else
%             tmp = u_nom;
%           end
%         end
%         % --- 診断 printf ---
%         if n_obs > 0
%           c_T  = A_obs_all(1, 1) * tmp(1);
%           c_tx = A_obs_all(1, 2) * tmp(2);
%           c_ty = A_obs_all(1, 3) * tmp(3);
%           total_lhs = c_T + c_tx + c_ty;
%           fprintf('[CBF 診断] 距離: %0.2fm | 制約数: %d | QP: %d\n', dist_p, n_obs, exitflag);
%           fprintf('  姿勢実測 : ロール=%+0.1f deg, ピッチ=%+0.1f deg | 角速度=[%+0.2f, %+0.2f]\n', phi_deg, theta_deg, w_drone(1), w_drone(2));
%           fprintf('  制御入力 : T=%0.2f (下限 %0.2f), tx=%+0.2f, ty=%+0.2f\n', tmp(1), T_min_flight, tmp(2), tmp(3));
%           fprintf('  制約寄与 : [推力T: %+0.2f, ロールtx: %+0.2f, ピッチty: %+0.2f] => 合計=%+0.2f <= b=%+0.2f\n\n', ...
%                   c_T, c_tx, c_ty, total_lhs, b_obs_all(1));
%         end
%       else
%         tmp = u_nom;
%       end
%       % --- 最小クリアランス（余裕距離）のログ保存 ---
%       min_dist_to_surface = Inf;
%       if ~isempty(obs_list)
%         p_drone_curr = x(5:7);
%         for i = 1:length(obs_list)
%           p_obs_w = obs_list(i).p_obs;
%           r_m_val = obs_list(i).r_obs_margin;
%           dist_surf = norm(p_obs_w - p_drone_curr) - r_m_val;
%           if dist_surf < min_dist_to_surface
%             min_dist_to_surface = dist_surf;
%           end
%         end
%       end
%       obj.result.min_clearance = min_dist_to_surface;
%       % 最終入力
%       obj.result.input = [max(T_min_flight, min(20.0, tmp(1))); ...
%                           max(-1.0,          min(1.0,  tmp(2))); ...
%                           max(-1.0,          min(1.0,  tmp(3))); ...
%                           max(-1.0,          min(1.0,  tmp(4)))];
%       result = obj.result;
%     end
%   end
% end

% classdef HLC_CBF < handle
%   % Hierarchical Linearization Controller with Mathematically Consistent C3BF
%   properties
%     self
%     result
%     param
%     parameter_order = ["mass","Lx","Ly","lx","ly","jx","jy","jz","gravity","km1","km2","km3","km4","k1","k2","k3","k4"];
%   end
%   methods
%     function obj = HLC_CBF(self,param)
%       obj.self = self;
%       obj.param = param;
%       obj.param.P = self.parameter.get(obj.parameter_order);
%       obj.result.input = zeros(4,1);
%     end
%     function result = do(obj,varargin)
%       model = obj.self.estimator.result;
%       ref = obj.self.reference.result;
%       xd = ref.state.xd;
%       P = obj.param.P;
%       F1 = obj.param.F1;
%       F2 = obj.param.F2;
%       F3 = obj.param.F3;
%       F4 = obj.param.F4;
%       xd=[xd;zeros(20-size(xd,1),1)];
% 
%       Rb0 = RodriguesQuaternion(Eul2Quat([0;0;xd(4)]));
%       x = [R2q(Rb0'*model.state.getq("rotmat"));Rb0'*model.state.p;Rb0'*model.state.v;model.state.w];
%       xd(1:3)=Rb0'*xd(1:3);
%       xd(4) = 0;
%       xd(5:7)=Rb0'*xd(5:7);
%       xd(9:11)=Rb0'*xd(9:11);
%       xd(13:15)=Rb0'*xd(13:15);
%       xd(17:19)=Rb0'*xd(17:19);
% 
%       if isfield(varargin{1},'dt') && varargin{1}.dt <= obj.param.dt
%         dt = varargin{1}.dt;
%         vf = Vfd(dt,x,xd',P,F1);
%         vs = Vsd(dt,x,xd',vf,P,F2,F3,F4);
%       else
%         vf = Vf(x,xd',P,F1);
%         vs = Vs(x,xd',vf,P,F2,F3,F4);
%       end
%       tmp = Uf(x,xd',vf,P) + Us(x,xd',vf,vs',P);
% 
%       % --------------------- 物理基準・パラメータ設定 -------------------- %
%       f_hover = P(1) * P(9);
%       T_min_flight = 0.70 * f_hover; % 自由落下防止の推力下限 (~5.05 N)
%       r_drone = 0.15;                % ドローン外接球半径 [m]
% 
%       % ノミナル入力の事前クリッピング
%       u_nom = [max(T_min_flight, min(20.0, tmp(1))); ...
%                max(-1.0,          min(1.0,  tmp(2))); ...
%                max(-1.0,          min(1.0,  tmp(3))); ...
%                max(-1.0,          min(1.0,  tmp(4)))];
% 
%       if isempty(obj.result.input) || obj.result.input(1) <= 0
%         f_T_curr = f_hover;
%       else
%         f_T_curr = obj.result.input(1);
%       end
% 
%       obs_list = ENVIRONMENT_OBSTACLE_HOCBF_LINK_XY();
%       if ~isempty(obs_list)
%         p_drone = x(5:7);
%         v_drone = x(8:10);
%         w_drone = x(11:13);
%         q_curr   = x(1:4);
%         phi_deg   = rad2deg(atan2(2*(q_curr(1)*q_curr(2) + q_curr(3)*q_curr(4)), q_curr(1)^2 - q_curr(2)^2 - q_curr(3)^2 + q_curr(4)^2));
%         theta_deg = rad2deg(asin(max(-1.0, min(1.0, 2*(q_curr(1)*q_curr(3) - q_curr(2)*q_curr(4))))));
% 
%         % % =========================================================================
%         % % 【正統 C3BF チューニングパラメータ】
%         % % =========================================================================
%         % lambda_cbf = 1.8;      % 衝突円錐極配置ゲイン
%         % margin_pad = 0.20;     % 理論的に正当な安全集合拡大マージン [m] (真横での安全確保)
%         % 
%         % % QP 目的関数での軸別優先度 (制約 A を歪めず、コスト H で回避方針を決定)
%         % w_T        = 300.0;    % 推力変更ペナルティ (高度抜けを最優先で抑止)
%         % w_tau_xy   = 0.01;     % ロール・ピッチ変更ペナルティ (積極的なバンクを許可)
%         % w_tau_z    = 1.0;      % ヨートルク変更ペナルティ
%         % % =========================================================================
%         % =========================================================================
%         % 【正統 C3BF チューニングパラメータ】
%         % =========================================================================
%         lambda_cbf = 1.8;      % 衝突円錐極配置ゲイン
%         margin_pad = 0.20;     % 理論的に正当な安全集合拡大マージン [m] (真横での安全確保)
% 
%         % QP 目的関数での軸別優先度 (制約 A を歪めず、コスト H で回避方針を決定)
%         w_T        = 0.001;    % 推力変更ペナルティ (高度抜けを最優先で抑止)
%         w_tau_xy   = 0.01;     % ロール・ピッチ変更ペナルティ (積極的なバンクを許可)
%         w_tau_z    = 1.0;      % ヨートルク変更ペナルティ
%         % =========================================================================
% 
%         A_obs_all = [];
%         b_obs_all = [];
% 
%         for i = 1:length(obs_list)
%           p_obs_world = obs_list(i).p_obs;
%           r_obs_val   = obs_list(i).r_obs_margin;
%           if isfield(obs_list(i), 'v_obs') && ~isempty(obs_list(i).v_obs)
%             v_obs_world = obs_list(i).v_obs;
%           else
%             v_obs_world = zeros(3, 1);
%           end
%           p_obs_rel = Rb0' * p_obs_world;
%           v_obs_rel = Rb0' * v_obs_world;
% 
%           p_rel_vec = p_obs_rel - p_drone;
%           dist_p    = norm(p_rel_vec);
% 
%           % 警戒距離: 手前 4.5m
%           if dist_p > (r_obs_val + r_drone + margin_pad + 4.0)
%             continue;
%           end
% 
%           % --- 拡大安全半径による C3BF 評価 (安全集合の正統拡大) ---
%           % --- 警戒距離判定 ---
%           r_effective = r_obs_val + r_drone + margin_pad;
% 
%           if dist_p > (r_effective + 4.0)
%             continue;
%           end
% 
%           % 特異領域ガード (平方根の非負性を保証)
%           if dist_p > (r_effective + 0.02)
%             % 【通常時: C3BF (衝突円錐)】
%             obsParam_c3bf = [p_obs_rel', v_obs_rel', r_effective];
%             A_raw = real(ECBF_Acbf(x, obsParam_c3bf, P, f_T_curr));
%             b_raw = real(ECBF_bcbf(x, obsParam_c3bf, P, f_T_curr, lambda_cbf));
%           else
%             % 【至近距離バックアップ: 純粋位置距離バリア】
%             obsParam_pos = [p_obs_rel', v_obs_rel', r_obs_val, r_drone];
%             A_raw = real(Pos_ECBF_Acbf(x, obsParam_pos, P, f_T_curr));
%             b_raw = real(Pos_ECBF_bcbf(x, obsParam_pos, P, f_T_curr, lambda_cbf));
%           end
% 
%           if ~any(isnan(A_raw)) && ~any(isnan(b_raw)) && ~any(isinf(A_raw)) && ~any(isinf(b_raw))
%             A_obs_all = [A_obs_all; A_raw];
%             b_obs_all = [b_obs_all; b_raw];
%           end
%         end
% 
%         % --- 2. 姿勢角制限 CBF (ロール・ピッチ ±40 deg) ---
%         max_tilt  = deg2rad(40); 
%         gamma_att = 3.0;
%         A_att = real(Attitude_CBF_Acbf(x, P));
%         b_att = real(Attitude_CBF_bcbf(x, P, max_tilt, gamma_att));
% 
%         % --- 3. ハード障害物制約 + ソフト姿勢制約 QP 最適化 ---
%         n_obs = size(A_obs_all, 1);
%         n_att = size(A_att, 1);
% 
%         if n_obs > 0
%           % 決定変数: z = [u (4x1); xi_att (n_att x 1)] (障害物制約にはスラックを許さない)
%           num_vars = 4 + n_att;
% 
%           % 制約式:
%           % [A_obs]    * u          <= b_obs   (ハード制約)
%           % [A_att]    * u - xi_att <= b_att   (ソフト制約)
%           A_qp = [A_obs_all, zeros(n_obs, n_att); ...
%                   A_att,     -eye(n_att)];
%           b_qp = [b_obs_all; b_att];
% 
%           H_u      = diag([w_T, w_tau_xy, w_tau_xy, w_tau_z]); 
%           w_xi_att = 50000.0; % 姿勢限界緩和への高ペナルティ
%           H_qp     = blkdiag(H_u, w_xi_att * eye(n_att));
%           f_qp     = [-H_u * u_nom; zeros(n_att, 1)];
% 
%           lb = [T_min_flight;  -1.0; -1.0; -1.0; zeros(n_att, 1)];
%           ub = [20.0;           1.0;  1.0;  1.0; inf(n_att, 1)];
% 
%           options = optimoptions('quadprog', 'Display', 'off', 'Algorithm', 'interior-point-convex');
%           [z_safe, ~, exitflag] = quadprog(real(H_qp), real(f_qp), real(A_qp), real(b_qp), [], [], lb, ub, [], options);
%           if exitflag == 1
%             tmp = z_safe(1:4);
%           else
%             % 万が一ハード制約で infeasible になった場合のみ公称値でフォールバック
%             tmp = u_nom;
%           end
%         else
%           A_qp_att = [A_att, -eye(n_att)];
%           b_qp_att = b_att;
%           H_qp_att = blkdiag(eye(4), 50000.0 * eye(n_att));
%           f_qp_att = [-u_nom; zeros(n_att, 1)];
%           lb_att   = [T_min_flight; -1.0; -1.0; -1.0; zeros(n_att, 1)];
%           ub_att   = [20.0;          1.0;  1.0;  1.0; inf(n_att, 1)];
% 
%           [z_safe, ~, exitflag] = quadprog(real(H_qp_att), real(f_qp_att), real(A_qp_att), real(b_qp_att), [], [], lb_att, ub_att, [], optimoptions('quadprog','Display','off'));
%           if exitflag == 1
%             tmp = z_safe(1:4);
%           else
%             tmp = u_nom;
%           end
%         end
% 
%         % --- 診断 printf ---
%         if n_obs > 0
%           h_diag = dist_p - (r_obs_val + r_drone);
%           c_T  = A_obs_all(1, 1) * tmp(1);
%           c_tx = A_obs_all(1, 2) * tmp(2);
%           c_ty = A_obs_all(1, 3) * tmp(3);
%           total_lhs = c_T + c_tx + c_ty;
%           fprintf('[理論純化 C3BF] 表面クリアランス: %+0.3fm | 距離: %0.2fm | QP: %d\n', h_diag, dist_p, exitflag);
%           fprintf('  姿勢実測 : ロール=%+0.1f deg, ピッチ=%+0.1f deg | 角速度=[%+0.2f, %+0.2f]\n', phi_deg, theta_deg, w_drone(1), w_drone(2));
%           fprintf('  制御入力 : T=%0.2f, tx=%+0.2f, ty=%+0.2f\n', tmp(1), tmp(2), tmp(3));
%           fprintf('  制約寄与 : [推力T: %+0.2e, ロールtx: %+0.2e, ピッチty: %+0.2e] => 合計=%+0.2e <= b=%+0.2e\n\n', ...
%                   c_T, c_tx, c_ty, total_lhs, b_obs_all(1));
%         end
%       else
%         tmp = u_nom;
%       end
% 
%       % --- 最小クリアランス（球体表面同士の最短距離）のログ保存 ---
%       min_dist_to_surface = Inf;
%       if ~isempty(obs_list)
%         p_drone_curr = x(5:7);
%         for i = 1:length(obs_list)
%           p_obs_w = obs_list(i).p_obs;
%           r_m_val = obs_list(i).r_obs_margin;
% 
%           % ドローン外接球表面 〜 障害物球表面 の最短距離
%           dist_surf = norm(p_obs_w - p_drone_curr) - (r_m_val + r_drone);
%           if dist_surf < min_dist_to_surface
%             min_dist_to_surface = dist_surf;
%           end
%         end
%       end
%       obj.result.min_clearance = min_dist_to_surface;
% 
%       % 最終入力
%       obj.result.input = [max(T_min_flight, min(20.0, tmp(1))); ...
%                           max(-1.0,          min(1.0,  tmp(2))); ...
%                           max(-1.0,          min(1.0,  tmp(3))); ...
%                           max(-1.0,          min(1.0,  tmp(4)))];
%       result = obj.result;
%     end
%   end
% end

% classdef HLC_CBF < handle
%   % Hierarchical Linearization Controller with Tiered CBF (Hard Position + Soft C3BF/Attitude)
%   properties
%     self
%     result
%     param
%     parameter_order = ["mass","Lx","Ly","lx","ly","jx","jy","jz","gravity","km1","km2","km3","km4","k1","k2","k3","k4"];
%   end
%   methods
%     function obj = HLC_CBF(self,param)
%       obj.self = self;
%       obj.param = param;
%       obj.param.P = self.parameter.get(obj.parameter_order);
%       obj.result.input = zeros(4,1);
%     end
%     function result = do(obj,varargin)
%       model = obj.self.estimator.result;
%       ref = obj.self.reference.result;
%       xd = ref.state.xd;
%       P = obj.param.P;
%       F1 = obj.param.F1;
%       F2 = obj.param.F2;
%       F3 = obj.param.F3;
%       F4 = obj.param.F4;
%       xd=[xd;zeros(20-size(xd,1),1)];
% 
%       Rb0 = RodriguesQuaternion(Eul2Quat([0;0;xd(4)]));
%       x = [R2q(Rb0'*model.state.getq("rotmat"));Rb0'*model.state.p;Rb0'*model.state.v;model.state.w];
%       xd(1:3)=Rb0'*xd(1:3);
%       xd(4) = 0;
%       xd(5:7)=Rb0'*xd(5:7);
%       xd(9:11)=Rb0'*xd(9:11);
%       xd(13:15)=Rb0'*xd(13:15);
%       xd(17:19)=Rb0'*xd(17:19);
% 
%       if isfield(varargin{1},'dt') && varargin{1}.dt <= obj.param.dt
%         dt = varargin{1}.dt;
%         vf = Vfd(dt,x,xd',P,F1);
%         vs = Vsd(dt,x,xd',vf,P,F2,F3,F4);
%       else
%         vf = Vf(x,xd',P,F1);
%         vs = Vs(x,xd',vf,P,F2,F3,F4);
%       end
%       tmp = Uf(x,xd',vf,P) + Us(x,xd',vf,vs',P);
% 
%       % --------------------- 物理基準・パラメータ設定 -------------------- %
%       f_hover = P(1) * P(9);
%       T_min_flight = 0.70 * f_hover; % 自由落下防止の推力下限 (~5.05 N)
%       r_drone = 0.15;                % ドローン外接球半径 [m]
% 
%       % ノミナル入力の事前クリッピング
%       u_nom = [max(T_min_flight, min(20.0, tmp(1))); ...
%                max(-1.0,          min(1.0,  tmp(2))); ...
%                max(-1.0,          min(1.0,  tmp(3))); ...
%                max(-1.0,          min(1.0,  tmp(4)))];
% 
%       if isempty(obj.result.input) || obj.result.input(1) <= 0
%         f_T_curr = f_hover;
%       else
%         f_T_curr = obj.result.input(1);
%       end
% 
%       obs_list = ENVIRONMENT_OBSTACLE_HOCBF_LINK_XY();
%       if ~isempty(obs_list)
%         p_drone = x(5:7);
%         v_drone = x(8:10);
%         w_drone = x(11:13);
%         q_curr   = x(1:4);
%         phi_deg   = rad2deg(atan2(2*(q_curr(1)*q_curr(2) + q_curr(3)*q_curr(4)), q_curr(1)^2 - q_curr(2)^2 - q_curr(3)^2 + q_curr(4)^2));
%         theta_deg = rad2deg(asin(max(-1.0, min(1.0, 2*(q_curr(1)*q_curr(3) - q_curr(2)*q_curr(4))))));
% 
%         % =========================================================================
%         % 【階層型 CBF チューニングパラメータ】
%         % =========================================================================
%         lambda_c3bf = 1.8;     % 衝突円錐極配置ゲイン
%         lambda_pos  = 1.5;     % 位置距離バリア極配置ゲイン
%         margin_pad  = 0.15;    % 衝突円錐拡大マージン [m]
%         d_detect    = 4.5;     % 検知・警戒範囲マージン [m]
% 
%         % QP 目的関数での軸別入力追従重み
%         w_T        = 5.0;      % 推力変更ペナルティ
%         w_tau_xy   = 50.0;     % ロール・ピッチ変更ペナルティ
%         w_tau_z    = 10.0;     % ヨートルクペナルティ
% 
%         % スラック変数ペナルティ重み
%         w_xi_c3bf  = 2000.0;   % C3BF（早期回避誘導）緩和ペナルティ
%         w_xi_att   = 50000.0;  % 姿勢限界緩和ペナルティ
%         % =========================================================================
% 
%         A_pos_all  = [];
%         b_pos_all  = [];
%         A_c3bf_all = [];
%         b_c3bf_all = [];
% 
%         for i = 1:length(obs_list)
%           p_obs_world = obs_list(i).p_obs;
%           r_obs_val   = obs_list(i).r_obs_margin;
%           if isfield(obs_list(i), 'v_obs') && ~isempty(obs_list(i).v_obs)
%             v_obs_world = obs_list(i).v_obs;
%           else
%             v_obs_world = zeros(3, 1);
%           end
%           p_obs_rel = Rb0' * p_obs_world;
%           v_obs_rel = Rb0' * v_obs_world;
% 
%           p_rel_vec = p_obs_rel - p_drone;
%           dist_p    = norm(p_rel_vec);
% 
%           % --- 1. 検知範囲判定 ---
%           r_effective = r_obs_val + r_drone + margin_pad;
%           if dist_p > (r_effective + d_detect)
%             continue;
%           end
% 
%           % --- 2. 純粋位置 ECBF (ハード制約候補: 絶対安全距離の死守) ---
%           obsParam_pos = [p_obs_rel', v_obs_rel', r_obs_val, r_drone];
%           A_pos = real(Pos_ECBF_Acbf(x, obsParam_pos, P, f_T_curr));
%           b_pos = real(Pos_ECBF_bcbf(x, obsParam_pos, P, f_T_curr, lambda_pos));
% 
%           if ~any(isnan(A_pos)) && ~any(isnan(b_pos)) && ~any(isinf(A_pos)) && ~any(isinf(b_pos))
%             A_pos_all = [A_pos_all; A_pos];
%             b_pos_all = [b_pos_all; b_pos];
%           end
% 
%           % --- 3. 衝突円錐 C3BF (ソフト制約候補: 早期バンク誘導) ---
%           if dist_p > (r_effective + 0.01)
%             obsParam_c3bf = [p_obs_rel', v_obs_rel', r_effective];
%             A_c3bf = real(ECBF_Acbf(x, obsParam_c3bf, P, f_T_curr));
%             b_c3bf = real(ECBF_bcbf(x, obsParam_c3bf, P, f_T_curr, lambda_c3bf));
% 
%             if ~any(isnan(A_c3bf)) && ~any(isnan(b_c3bf)) && ~any(isinf(A_c3bf)) && ~any(isinf(b_c3bf))
%               A_c3bf_all = [A_c3bf_all; A_c3bf];
%               b_c3bf_all = [b_c3bf_all; b_c3bf];
%             end
%           end
%         end
% 
%         % --- 4. 姿勢角制限 CBF (ソフト制約: ロール・ピッチ ±40 deg) ---
%         max_tilt  = deg2rad(40); 
%         gamma_att = 3.0;
%         A_att = real(Attitude_CBF_Acbf(x, P));
%         b_att = real(Attitude_CBF_bcbf(x, P, max_tilt, gamma_att));
% 
%         % --- 5. 階層型 QP 最適化 (Hard Pos + Soft C3BF + Soft Att) ---
%         n_pos  = size(A_pos_all, 1);
%         n_c3bf = size(A_c3bf_all, 1);
%         n_att  = size(A_att, 1);
% 
%         num_slacks = n_c3bf + n_att;
%         num_vars   = 4 + num_slacks;
% 
%         % 制約ブロック構築:
%         % [ A_pos   0       0     ] [u      ]   <= [ b_pos  ]  (ハード)
%         % [ A_c3bf -I_c3bf  0     ] [xi_c3bf]   <= [ b_c3bf ]  (ソフト)
%         % [ A_att   0      -I_att ] [xi_att ]   <= [ b_att  ]  (ソフト)
%         A_qp = [A_pos_all,  zeros(n_pos, n_c3bf),  zeros(n_pos, n_att); ...
%                 A_c3bf_all, -eye(n_c3bf),          zeros(n_c3bf, n_att); ...
%                 A_att,      zeros(n_att, n_c3bf),  -eye(n_att)];
%         b_qp = [b_pos_all; b_c3bf_all; b_att];
% 
%         H_u  = diag([w_T, w_tau_xy, w_tau_xy, w_tau_z]);
%         H_qp = blkdiag(H_u, w_xi_c3bf * eye(n_c3bf), w_xi_att * eye(n_att));
%         f_qp = [-H_u * u_nom; zeros(num_slacks, 1)];
% 
%         lb = [T_min_flight; -1.0; -1.0; -1.0; zeros(num_slacks, 1)];
%         ub = [20.0;          1.0;  1.0;  1.0; inf(num_slacks, 1)];
% 
%         options = optimoptions('quadprog', 'Display', 'off', 'Algorithm', 'interior-point-convex');
%         [z_safe, ~, exitflag] = quadprog(real(H_qp), real(f_qp), real(A_qp), real(b_qp), [], [], lb, ub, [], options);
% 
%         if exitflag == 1
%           tmp = z_safe(1:4);
%         else
%           % ハード位置制約でさえ競合した場合の緊急フォールバック (位置制約にもスラックを許可)
%           A_qp_em = [A_pos_all,  -eye(n_pos),           zeros(n_pos, n_c3bf + n_att); ...
%                      A_c3bf_all, zeros(n_c3bf, n_pos),  -eye(n_c3bf), zeros(n_c3bf, n_att); ...
%                      A_att,      zeros(n_att, n_pos + n_c3bf), -eye(n_att)];
%           b_qp_em = b_qp;
%           H_qp_em = blkdiag(H_u, 1e6 * eye(n_pos), w_xi_c3bf * eye(n_c3bf), w_xi_att * eye(n_att));
%           f_qp_em = [-H_u * u_nom; zeros(n_pos + num_slacks, 1)];
%           lb_em   = [T_min_flight; -1.0; -1.0; -1.0; zeros(n_pos + num_slacks, 1)];
%           ub_em   = [20.0;          1.0;  1.0;  1.0; inf(n_pos + num_slacks, 1)];
% 
%           [z_em, ~, exitflag_em] = quadprog(real(H_qp_em), real(f_qp_em), real(A_qp_em), real(b_qp_em), [], [], lb_em, ub_em, [], options);
%           if exitflag_em == 1
%             tmp = z_em(1:4);
%           else
%             tmp = u_nom;
%           end
%         end
% 
%         % --- 診断 printf ---
%         if (n_pos + n_c3bf) > 0
%           h_diag = dist_p - (r_obs_val + r_drone);
%           c_T  = A_pos_all(1, 1) * tmp(1);
%           c_tx = A_pos_all(1, 2) * tmp(2);
%           c_ty = A_pos_all(1, 3) * tmp(3);
%           total_lhs = c_T + c_tx + c_ty;
%           fprintf('[Tiered CBF 診断] 表面クリアランス: %+0.3fm | 距離: %0.2fm | QP: %d\n', h_diag, dist_p, exitflag);
%           fprintf('  姿勢実測 : ロール=%+0.1f deg, ピッチ=%+0.1f deg | 角速度=[%+0.2f, %+0.2f]\n', phi_deg, theta_deg, w_drone(1), w_drone(2));
%           fprintf('  制御入力 : T=%0.2f, tx=%+0.2f, ty=%+0.2f\n', tmp(1), tmp(2), tmp(3));
%           fprintf('  位置制約寄与 : [推力T: %+0.2e, ロールtx: %+0.2e, ピッチty: %+0.2e] => 合計=%+0.2e <= b=%+0.2e\n\n', ...
%                   c_T, c_tx, c_ty, total_lhs, b_pos_all(1));
%         end
%       else
%         tmp = u_nom;
%       end
% 
%       % --- 最小クリアランス（球体表面同士の最短距離）のログ保存 ---
%       min_dist_to_surface = Inf;
%       if ~isempty(obs_list)
%         p_drone_curr = x(5:7);
%         for i = 1:length(obs_list)
%           p_obs_w = obs_list(i).p_obs;
%           r_m_val = obs_list(i).r_obs_margin;
% 
%           % ドローン外接球表面 〜 障害物球表面 の最短距離
%           dist_surf = norm(p_obs_w - p_drone_curr) - (r_m_val + r_drone);
%           if dist_surf < min_dist_to_surface
%             min_dist_to_surface = dist_surf;
%           end
%         end
%       end
%       obj.result.min_clearance = min_dist_to_surface;
% 
%       % 最終入力
%       obj.result.input = [max(T_min_flight, min(20.0, tmp(1))); ...
%                           max(-1.0,          min(1.0,  tmp(2))); ...
%                           max(-1.0,          min(1.0,  tmp(3))); ...
%                           max(-1.0,          min(1.0,  tmp(4)))];
%       result = obj.result;
%     end
%   end
% end

% classdef HLC_CBF < handle
%   % FastBridge Nonlinear Collision Cone ECBF Controller (Relative Degree 3)
%   % Full Quadrotor Dynamics with 3D Gaussian Splatting / Ellipsoidal Obstacles
%   properties
%     self
%     result
%     param
%     parameter_order = ["mass","Lx","Ly","lx","ly","jx","jy","jz","gravity","km1","km2","km3","km4","k1","k2","k3","k4"];
% 
%     % FastBridge 動的拡大状態 (正規化推力 a_T = T/m, da_T = d(T/m)/dt)
%     aT_state  = [];
%     daT_state = 0.0;
%   end
% 
%   methods
%     function obj = HLC_CBF(self, param)
%       obj.self = self;
%       obj.param = param;
%       obj.param.P = self.parameter.get(obj.parameter_order);
%       obj.result.input = zeros(4,1);
% 
%       % ホバリング正規化推力 (aT = g) で初期化
%       g_val = obj.param.P(9);
%       obj.aT_state  = g_val;
%       obj.daT_state = 0.0;
%     end
% 
%     function result = do(obj, varargin)
%       model = obj.self.estimator.result;
%       ref   = obj.self.reference.result;
%       xd    = ref.state.xd;
%       P     = obj.param.P;
%       F1    = obj.param.F1;
%       F2    = obj.param.F2;
%       F3    = obj.param.F3;
%       F4    = obj.param.F4;
%       xd    = [xd; zeros(20 - size(xd,1), 1)];
% 
%       % ヨー基準回転座標系への変換
%       Rb0 = RodriguesQuaternion(Eul2Quat([0; 0; xd(4)]));
%       x = [R2q(Rb0' * model.state.getq("rotmat")); ...
%            Rb0' * model.state.p; ...
%            Rb0' * model.state.v; ...
%            model.state.w];
%       xd(1:3)   = Rb0' * xd(1:3);
%       xd(4)     = 0;
%       xd(5:7)   = Rb0' * xd(5:7);
%       xd(9:11)  = Rb0' * xd(9:11);
%       xd(13:15) = Rb0' * xd(13:15);
%       xd(17:19) = Rb0' * xd(17:19);
% 
%       % 制御サンプリング周期 dt の取得
%       if isfield(varargin{1}, 'dt') && varargin{1}.dt <= obj.param.dt
%         dt = varargin{1}.dt;
%         vf = Vfd(dt, x, xd', P, F1);
%         vs = Vsd(dt, x, xd', vf, P, F2, F3, F4);
%       else
%         dt = obj.param.dt;
%         vf = Vf(x, xd', P, F1);
%         vs = Vs(x, xd', vf, P, F2, F3, F4);
%       end
% 
%       % ノミナル制御入力の計算 (DFLコントローラ)
%       tmp_nom = Uf(x, xd', vf, P) + Us(x, xd', vf, vs', P);
% 
%       % 物理定数
%       m_drone = P(1);
%       g_drone = P(9);
%       f_hover = m_drone * g_drone;
%       T_min_flight = 0.70 * f_hover;
%       T_max_flight = 20.0;
%       tau_max      = 1.0;
% 
%       % ノミナル推力・トルクのクリッピング
%       u_nom_clamped = [max(T_min_flight, min(T_max_flight, tmp_nom(1))); ...
%                        max(-tau_max,     min(tau_max,      tmp_nom(2))); ...
%                        max(-tau_max,     min(tau_max,      tmp_nom(3))); ...
%                        max(-tau_max,     min(tau_max,      tmp_nom(4)))];
% 
%       % 拡大状態 aT の初期化ガード
%       if isempty(obj.aT_state) || obj.aT_state <= 0
%         obj.aT_state = u_nom_clamped(1) / m_drone;
%       end
% 
%       % 14次元拡大状態ベクトル: z = [q(4); p(3); v(3); w(3); aT; daT]
%       z_ext = [x(1:4); x(5:7); x(8:10); x(11:13); obj.aT_state; obj.daT_state];
% 
%       % ノミナル入力 eta_nom = [d2aT_nom; tau_x_nom; tau_y_nom; tau_z_nom]
%       % (目標推力へスムーズに追従させるための2階微分のPゲイン誘導)
%       aT_nom_target = u_nom_clamped(1) / m_drone;
%       d2aT_nom = 40.0 * (aT_nom_target - obj.aT_state) - 10.0 * obj.daT_state;
%       eta_nom  = [d2aT_nom; u_nom_clamped(2:4)];
% 
%       % ------------------- 障害物リストと制約構築 ------------------- %
%       obs_list = ENVIRONMENT_OBSTACLE_HOCBF_LINK_XY();
%       A_cbf_all = [];
%       b_cbf_all = [];
% 
%       if ~isempty(obs_list)
%         p_drone = x(5:7);
%         v_drone = x(8:10);
% 
%         lambda_cbf = 2.0;    % FastBridge ECBF 極配置パラメータ (triple pole)
%         d_detect   = 5.0;    % 検知・安全フィルタ適用範囲 [m]
% 
%         for i = 1:length(obs_list)
%           p_obs_world = obs_list(i).p_obs;
%           r_obs_val   = obs_list(i).r_obs_margin;
% 
%           % 座標変換 (機体ヨー基準)
%           p_obs_rel = Rb0' * p_obs_world;
%           dist_p    = norm(p_obs_rel - p_drone);
% 
%           % 1. 距離ゲート (Distance Gate)
%           if dist_p > (r_obs_val + d_detect)
%             continue;
%           end
% 
%           % 障害物形状行列 A = Sigma^(-1) の構成 (球形/楕円体)
%           % ※ 球体障害物の場合は等方性、3DGSの場合は共分散の逆行列を設定
%           Sigma_inv = diag([1/(r_obs_val^2), 1/(r_obs_val^2), 1/(r_obs_val^2)]);
%           A_vec = [Sigma_inv(1,1); Sigma_inv(1,2); Sigma_inv(1,3); ...
%                    Sigma_inv(2,2); Sigma_inv(2,3); Sigma_inv(3,3)];
%           c_scale_val = 1.0;
%           splat_params = [p_obs_rel; A_vec; c_scale_val];
% 
%           % 2. アプローチゲート (Approach Gate: r'*A*v >= 0)
%           r_vec = p_obs_rel - p_drone;
%           A_mat = [A_vec(1), A_vec(2), A_vec(3); ...
%                    A_vec(2), A_vec(4), A_vec(5); ...
%                    A_vec(3), A_vec(5), A_vec(6)];
%           if (r_vec' * A_mat * v_drone >= -0.1) % 接近方向にある場合のみ制約生成
%             [A_i, b_i] = FastBridge_CBF_QP(z_ext, splat_params, P, lambda_cbf);
%             if ~any(isnan(A_i)) && ~any(isnan(b_i)) && ~any(isinf(A_i)) && ~any(isinf(b_i))
%               A_cbf_all = [A_cbf_all; real(A_i)];
%               b_cbf_all = [b_cbf_all; real(b_i)];
%             end
%           end
%         end
%       end
% 
%       % --- 姿勢角制限 CBF (ロール・ピッチ ±40 deg) ---
%       max_tilt  = deg2rad(40);
%       gamma_att = 3.0;
%       A_att = real(Attitude_CBF_Acbf(x, P));
%       b_att = real(Attitude_CBF_bcbf(x, P, max_tilt, gamma_att));
% 
%       % --- 階層型 QP 最適化 (Soft Slacks 構成) ---
%       n_cbf = size(A_cbf_all, 1);
%       n_att = size(A_att, 1);
% 
%       w_d2aT     = 1.0;      % 推力変化率ペナルティ
%       w_tau_xy   = 50.0;     % ロール・ピッチトルクペナルティ
%       w_tau_z    = 10.0;     % ヨートルクペナルティ
%       w_xi_cbf   = 5000.0;   % セーフティフィルタ違反ペナルティ (高優先度)
%       w_xi_att   = 50000.0;  % 姿勢限界スラックペナルティ
% 
%       H_u  = diag([w_d2aT, w_tau_xy, w_tau_xy, w_tau_z]);
%       f_u  = -H_u * eta_nom;
% 
%       num_slacks = n_cbf + n_att;
%       H_qp = blkdiag(H_u, w_xi_cbf * eye(n_cbf), w_xi_att * eye(n_att));
%       f_qp = [f_u; zeros(num_slacks, 1)];
% 
%       % 制約ブロック: A_cbf*eta - xi_cbf <= b_cbf,  A_att(:,2:4)*tau - xi_att <= b_att
%       A_att_4var = [zeros(n_att, 1), A_att(:, 2:4)]; % d2aT には姿勢制約はかからない
%       A_qp = [A_cbf_all,   -eye(n_cbf),         zeros(n_cbf, n_att); ...
%               A_att_4var,  zeros(n_att, n_cbf), -eye(n_att)];
%       b_qp = [b_cbf_all; b_att];
% 
%       % eta = [d2aT; taux; tauy; tauz] の探索範囲
%       lb = [-80.0; -tau_max; -tau_max; -tau_max; zeros(num_slacks, 1)];
%       ub = [ 80.0;  tau_max;  tau_max;  tau_max; inf(num_slacks, 1)];
% 
%       options = optimoptions('quadprog', 'Display', 'off', 'Algorithm', 'interior-point-convex');
%       [eta_opt, ~, exitflag] = quadprog(real(H_qp), real(f_qp), real(A_qp), real(b_qp), [], [], lb, ub, [], options);
% 
%       if exitflag == 1
%         eta_cmd = eta_opt(1:4);
%       else
%         % QP 非実行可能時の安全フォールバック (論文 Backup Policy: ホバリング制動)
%         k_brake = 1.5;
%         eta_cmd = [ -5.0 * obj.daT_state; ...
%                     -2.0 * x(11) + k_brake * x(9); ...
%                     -2.0 * x(12) - k_brake * x(8); ...
%                     -1.0 * x(13) ];
%       end
% 
%       % ----------------- 動的拡大状態の積分更新 & 実推力換算 ----------------- %
%       d2aT_safe = eta_cmd(1);
%       tau_safe  = eta_cmd(2:4);
% 
%       % オイラー積分による状態更新
%       obj.daT_state = obj.daT_state + d2aT_safe * dt;
%       obj.aT_state  = obj.aT_state  + obj.daT_state * dt;
% 
%       % 機体実推力 [N]
%       T_actual = m_drone * obj.aT_state;
% 
%       % 最終出力のクリッピングガード
%       T_out = max(T_min_flight, min(T_max_flight, T_actual));
%       obj.aT_state = T_out / m_drone; % 飽和時の積分ワインドアップ防止
% 
%       obj.result.input = [T_out; ...
%                           max(-tau_max, min(tau_max, tau_safe(1))); ...
%                           max(-tau_max, min(tau_max, tau_safe(2))); ...
%                           max(-tau_max, min(tau_max, tau_safe(3)))];
% 
%       % --- 最小表面クリアランスの記録 ---
%       min_dist_to_surface = Inf;
%       if ~isempty(obs_list)
%         p_drone_curr = x(5:7);
%         for i = 1:length(obs_list)
%           p_obs_w = obs_list(i).p_obs;
%           r_m_val = obs_list(i).r_obs_margin;
%           dist_surf = norm(p_obs_w - p_drone_curr) - r_m_val;
%           if dist_surf < min_dist_to_surface
%             min_dist_to_surface = dist_surf;
%           end
%         end
%       end
%       obj.result.min_clearance = min_dist_to_surface;
% 
%       result = obj.result;
%     end
%   end
% end

% classdef HLC_CBF < handle
%   % FastBridge Nonlinear Collision Cone ECBF Controller (Relative Degree 3)
%   % Full Quadrotor Dynamics with 3D Gaussian Splatting / Ellipsoidal Obstacles
% 
%   properties
%     self
%     result
%     param
% 
%     parameter_order = ["mass","Lx","Ly","lx","ly", ...
%                        "jx","jy","jz","gravity", ...
%                        "km1","km2","km3","km4", ...
%                        "k1","k2","k3","k4"];
% 
%     % Dynamic extension:
%     % aT  = T/m
%     % daT = d(T/m)/dt
%     aT_state  = [];
%     daT_state = 0.0;
%   end
% 
%   methods
% 
%     function obj = HLC_CBF(self, param)
%       obj.self = self;
%       obj.param = param;
% 
%       obj.param.P = self.parameter.get(obj.parameter_order);
% 
%       obj.result.input = zeros(4,1);
% 
%       % Hover initialization
%       g_val = obj.param.P(9);
%       obj.aT_state  = g_val;
%       obj.daT_state = 0.0;
%     end
% 
%     function result = do(obj, varargin)
% 
%       %% ================================================================
%       % 1. State / Reference
%       %% ================================================================
%       model = obj.self.estimator.result;
%       ref   = obj.self.reference.result;
% 
%       xd = ref.state.xd;
% 
%       P  = obj.param.P;
%       F1 = obj.param.F1;
%       F2 = obj.param.F2;
%       F3 = obj.param.F3;
%       F4 = obj.param.F4;
% 
%       xd = [xd; zeros(max(0,20-size(xd,1)),1)];
% 
%       %% ================================================================
%       % 2. dt safe acquisition
%       %% ================================================================
%       dt = obj.param.dt;
% 
%       if ~isempty(varargin)
%         if isstruct(varargin{1}) && isfield(varargin{1},'dt')
%           dt_candidate = varargin{1}.dt;
%           if ~isempty(dt_candidate) && ...
%              isfinite(dt_candidate) && ...
%              dt_candidate > 0 && ...
%              dt_candidate <= obj.param.dt
%             dt = dt_candidate;
%           end
%         end
%       end
% 
%       %% ================================================================
%       % 3. Yaw reference coordinate transformation
%       %% ================================================================
%       Rb0 = RodriguesQuaternion(Eul2Quat([0;0;xd(4)]));
% 
%       q_raw = R2q(Rb0' * model.state.getq("rotmat"));
% 
%       % Quaternion normalization
%       q_norm = norm(q_raw);
%       if q_norm < 1e-8 || ~isfinite(q_norm)
%         q_safe = [1;0;0;0];
%       else
%         q_safe = q_raw / q_norm;
%       end
% 
%       x = [q_safe; ...
%            Rb0' * model.state.p; ...
%            Rb0' * model.state.v; ...
%            model.state.w];
% 
%       xd(1:3)   = Rb0' * xd(1:3);
%       xd(4)     = 0;
%       xd(5:7)   = Rb0' * xd(5:7);
%       xd(9:11)  = Rb0' * xd(9:11);
%       xd(13:15) = Rb0' * xd(13:15);
%       xd(17:19) = Rb0' * xd(17:19);
% 
%       %% ================================================================
%       % 4. Nominal DFL controller
%       %% ================================================================
%       if dt <= obj.param.dt
%         vf = Vfd(dt, x, xd', P, F1);
%         vs = Vsd(dt, x, xd', vf, P, F2, F3, F4);
%       else
%         vf = Vf(x, xd', P, F1);
%         vs = Vs(x, xd', vf, P, F2, F3, F4);
%       end
% 
%       tmp_nom = Uf(x, xd', vf, P) + ...
%                 Us(x, xd', vf, vs', P);
% 
%       %% ================================================================
%       % 5. Physical limits
%       %% ================================================================
%       m_drone = P(1);
%       g_drone = P(9);
% 
%       f_hover = m_drone * g_drone;
% 
%       T_min_flight = 0.70 * f_hover;
%       T_max_flight = 20.0;
% 
%       tau_max = 1.0;
% 
%       % Drone collision radius
%       r_drone = 0.15;
% 
%       %% ================================================================
%       % 6. Nominal input clipping
%       %% ================================================================
%       u_nom_clamped = [ ...
%           max(T_min_flight, min(T_max_flight, tmp_nom(1))); ...
%           max(-tau_max,     min(tau_max, tmp_nom(2))); ...
%           max(-tau_max,     min(tau_max, tmp_nom(3))); ...
%           max(-tau_max,     min(tau_max, tmp_nom(4))) ];
% 
%       %% ================================================================
%       % 7. Dynamic thrust extension initialization
%       %% ================================================================
%       if isempty(obj.aT_state) || ...
%          ~isfinite(obj.aT_state) || ...
%          obj.aT_state <= 0
%         obj.aT_state = u_nom_clamped(1) / m_drone;
%       end
% 
%       if isempty(obj.daT_state) || ...
%          ~isfinite(obj.daT_state)
%         obj.daT_state = 0.0;
%       end
% 
%       %% ================================================================
%       % 8. Extended state z in R^14: [q(4); p(3); v(3); w(3); aT; daT]
%       %% ================================================================
%       z_ext = [ ...
%           x(1:4); ...
%           x(5:7); ...
%           x(8:10); ...
%           x(11:13); ...
%           obj.aT_state; ...
%           obj.daT_state ];
% 
%       %% ================================================================
%       % 9. Nominal eta: eta = [d2aT; taux; tauy; tauz]
%       %% ================================================================
%       aT_nom_target = u_nom_clamped(1) / m_drone;
% 
%       kp_aT = 40.0;
%       kd_aT = 10.0;
% 
%       d2aT_nom = kp_aT * (aT_nom_target - obj.aT_state) ...
%                - kd_aT * obj.daT_state;
% 
%       eta_nom = [ ...
%           d2aT_nom; ...
%           u_nom_clamped(2:4) ];
% 
%       %% ================================================================
%       % 10. Collision Cone ECBF constraints
%       %% ================================================================
%       obs_list = ENVIRONMENT_OBSTACLE_HOCBF_LINK_XY();
% 
%       A_cbf_all = [];
%       b_cbf_all = [];
% 
%       if ~isempty(obs_list)
%         p_drone = x(5:7);
%         v_drone = x(8:10);
% 
%         lambda_cbf = 2.0;
%         d_detect = 5.0;
% 
%         for i = 1:length(obs_list)
%           p_obs_world = obs_list(i).p_obs;
%           r_obs_val   = obs_list(i).r_obs_margin;
% 
%           % Coordinate transformation
%           p_obs_rel = Rb0' * p_obs_world;
% 
%           % Effective collision radius
%           r_effective = r_obs_val + r_drone;
%           dist_p = norm(p_obs_rel - p_drone);
% 
%           % Distance gate
%           if dist_p > r_effective + d_detect
%             continue;
%           end
% 
%           % Ellipsoid matrix Sigma^(-1)
%           r_safe = max(r_effective, 1e-3);
% 
%           Sigma_inv = diag([ ...
%               1/(r_safe^2), ...
%               1/(r_safe^2), ...
%               1/(r_safe^2)]);
% 
%           A_vec = [ ...
%               Sigma_inv(1,1); ...
%               Sigma_inv(1,2); ...
%               Sigma_inv(1,3); ...
%               Sigma_inv(2,2); ...
%               Sigma_inv(2,3); ...
%               Sigma_inv(3,3) ];
% 
%           c_scale_val = 1.0;
% 
%           splat_params = [ ...
%               p_obs_rel; ...
%               A_vec; ...
%               c_scale_val ];
% 
%           % Approach gate
%           r_vec = p_obs_rel - p_drone;
% 
%           A_mat = [ ...
%               A_vec(1), A_vec(2), A_vec(3); ...
%               A_vec(2), A_vec(4), A_vec(5); ...
%               A_vec(3), A_vec(5), A_vec(6) ];
% 
%           approach_metric = r_vec' * A_mat * v_drone;
% 
%           if approach_metric >= -0.1
%             [A_i, b_i] = FastBridge_CBF_QP( ...
%                             z_ext, ...
%                             splat_params, ...
%                             P, ...
%                             lambda_cbf);
% 
%             A_i = real(A_i);
%             b_i = real(b_i);
% 
%             if all(isfinite(A_i(:))) && all(isfinite(b_i(:)))
%               A_cbf_all = [A_cbf_all; A_i];
%               b_cbf_all = [b_cbf_all; b_i];
%             end
%           end
%         end
%       end
% 
%       %% ================================================================
%       % 11. Attitude CBF
%       %% ================================================================
%       max_tilt  = deg2rad(40);
%       gamma_att = 3.0;
% 
%       A_att = real(Attitude_CBF_Acbf(x, P));
%       b_att = real(Attitude_CBF_bcbf(x, P, max_tilt, gamma_att));
% 
%       n_cbf = size(A_cbf_all, 1);
%       n_att = size(A_att, 1);
% 
%       %% ================================================================
%       % 12. HARD Collision CBF + SOFT Attitude CBF QP
%       %% ================================================================
%       w_d2aT   = 1.0;
%       w_tau_xy = 50.0;
%       w_tau_z  = 10.0;
%       w_xi_att = 50000.0;
% 
%       H_u = diag([ ...
%           w_d2aT, ...
%           w_tau_xy, ...
%           w_tau_xy, ...
%           w_tau_z ]);
% 
%       f_u = -H_u * eta_nom;
% 
%       % Decision variable: z_qp = [eta(4); xi_att(n_att)]
%       H_qp = blkdiag(H_u, w_xi_att * eye(n_att));
%       f_qp = [f_u; zeros(n_att, 1)];
% 
%       % Attitude constraint mapping to eta = [d2aT, tx, ty, tz]
%       A_att_4var = [zeros(n_att, 1), A_att(:, 2:4)];
% 
%       % Constraints: Hard CBF + Soft Attitude
%       A_qp = [ ...
%           A_cbf_all,   zeros(n_cbf, n_att); ...
%           A_att_4var,  -eye(n_att) ];
% 
%       b_qp = [ ...
%           b_cbf_all; ...
%           b_att ];
% 
%       %% ================================================================
%       % 13. Input bounds
%       %% ================================================================
%       lb_eta = [ ...
%           -80.0; ...
%           -tau_max; ...
%           -tau_max; ...
%           -tau_max ];
% 
%       ub_eta = [ ...
%            80.0; ...
%            tau_max; ...
%            tau_max; ...
%            tau_max ];
% 
%       lb = [lb_eta; zeros(n_att, 1)];
%       ub = [ub_eta; inf(n_att, 1)];
% 
%       %% ================================================================
%       % 14. QP solve
%       %% ================================================================
%       options = optimoptions( ...
%           'quadprog', ...
%           'Display', 'off', ...
%           'Algorithm', 'interior-point-convex');
% 
%       [z_qp, ~, exitflag] = quadprog( ...
%           real(H_qp), ...
%           real(f_qp), ...
%           real(A_qp), ...
%           real(b_qp), ...
%           [], [], ...
%           lb, ub, [], ...
%           options);
% 
%       %% ================================================================
%       % 15. Safety fallback
%       %% ================================================================
%       if exitflag == 1
%         eta_cmd = z_qp(1:4);
%       else
%         % Conservative braking policy
%         k_brake = 1.5;
%         eta_cmd = [ ...
%             -5.0 * obj.daT_state; ...
%             -2.0 * x(11) + k_brake * x(9); ...
%             -2.0 * x(12) - k_brake * x(8); ...
%             -1.0 * x(13) ];
%       end
% 
%       %% ================================================================
%       % 16. Dynamic thrust state update (Semi-implicit Euler)
%       %% ================================================================
%       d2aT_safe = eta_cmd(1);
%       tau_safe  = eta_cmd(2:4);
% 
%       daT_new = obj.daT_state + d2aT_safe * dt;
%       aT_new  = obj.aT_state  + daT_new   * dt;
% 
%       %% ================================================================
%       % 17. Thrust saturation + anti-windup
%       %% ================================================================
%       T_actual = m_drone * aT_new;
% 
%       T_out = max( ...
%           T_min_flight, ...
%           min(T_max_flight, T_actual));
% 
%       aT_sat = T_out / m_drone;
% 
%       saturated_high = (T_actual > T_max_flight);
%       saturated_low  = (T_actual < T_min_flight);
% 
%       if saturated_high
%         daT_new = min(daT_new, 0.0);
%       elseif saturated_low
%         daT_new = max(daT_new, 0.0);
%       end
% 
%       obj.aT_state  = aT_sat;
%       obj.daT_state = daT_new;
% 
%       %% ================================================================
%       % 18. Final actuator command
%       %% ================================================================
%       obj.result.input = [ ...
%           T_out; ...
%           max(-tau_max, min(tau_max, tau_safe(1))); ...
%           max(-tau_max, min(tau_max, tau_safe(2))); ...
%           max(-tau_max, min(tau_max, tau_safe(3))) ];
% 
%       %% ================================================================
%       % 19. Minimum surface clearance
%       %% ================================================================
%       min_dist_to_surface = Inf;
% 
%       if ~isempty(obs_list)
%         p_drone_curr = x(5:7);
% 
%         for i = 1:length(obs_list)
%           p_obs_w = Rb0' * obs_list(i).p_obs;
%           r_obs_val = obs_list(i).r_obs_margin;
% 
%           dist_surf = norm(p_obs_w - p_drone_curr) - (r_obs_val + r_drone);
% 
%           if dist_surf < min_dist_to_surface
%             min_dist_to_surface = dist_surf;
%           end
%         end
%       end
% 
%       obj.result.min_clearance = min_dist_to_surface;
% 
%       %% ================================================================
%       % 20. Diagnostics
%       %% ================================================================
%       obj.result.aT       = obj.aT_state;
%       obj.result.daT      = obj.daT_state;
%       obj.result.exitflag = exitflag;
%       obj.result.n_cbf    = n_cbf;
% 
%       result = obj.result;
%     end
% 
%   end
% end

classdef HLC_CBF < handle
  % =========================================================================
  % FastBridge Collision Cone ECBF Controller (Finite-Difference Formulation)
  %
  % Decision variable:
  %     u = [T; tau_x; tau_y; tau_z] \in R^4
  %
  % Thrust dynamics:
  %     Finite-difference cascade based on actual thrust history (T_prev1, T_prev2)
  %
  % Optimization:
  %     Primary:   Hard Collision ECBF + Soft Attitude CBF
  %     Secondary: Slack-relaxed Emergency QP (Feasibility Guarantee)
  % =========================================================================

  properties
    self
    result
    param

    parameter_order = [ ...
      "mass","Lx","Ly","lx","ly", ...
      "jx","jy","jz","gravity", ...
      "km1","km2","km3","km4", ...
      "k1","k2","k3","k4"];

    % 実推力履歴 (確定した実入力推力を格納)
    T_prev1 = [];
    T_prev2 = [];

    initialized = false;
  end

  methods

    % =======================================================================
    % Constructor
    % =======================================================================
    function obj = HLC_CBF(self, param)
      obj.self  = self;
      obj.param = param;

      obj.param.P = self.parameter.get(obj.parameter_order);

      obj.result.input = zeros(4, 1);
      obj.result.min_clearance = Inf;

      obj.T_prev1 = [];
      obj.T_prev2 = [];
      obj.initialized = false;
    end

    % =======================================================================
    % Main Controller Execution
    % =======================================================================
    function result = do(obj, varargin)

      % =====================================================================
      % 1. State / Reference Acquisition
      % =====================================================================
      model = obj.self.estimator.result;
      ref   = obj.self.reference.result;

      xd = ref.state.xd;
      P  = obj.param.P(:); % 確実に列ベクトル化 (インデックスエラー防止)

      F1 = obj.param.F1;
      F2 = obj.param.F2;
      F3 = obj.param.F3;
      F4 = obj.param.F4;

      xd = [xd; zeros(max(0, 20 - size(xd, 1)), 1)];

      % =====================================================================
      % 2. Sampling Time Acquisition & Safety Guard
      % =====================================================================
      dt = obj.param.dt;
      if ~isempty(varargin)
        if isstruct(varargin{1}) && isfield(varargin{1}, 'dt')
          dt_candidate = varargin{1}.dt;
          if dt_candidate > 0 && dt_candidate <= obj.param.dt
            dt = dt_candidate;
          end
        end
      end
      dt = max(dt, 1e-4); % ゼロ除算・数値不安定性防止

      % =====================================================================
      % 3. Coordinate Transformation & Quaternion Normalization
      % =====================================================================
      Rb0 = RodriguesQuaternion(Eul2Quat([0; 0; xd(4)]));

      q_raw = R2q(Rb0' * model.state.getq("rotmat"));
      q_norm = norm(q_raw);
      if q_norm < 1e-8 || ~isfinite(q_norm)
        q = [1; 0; 0; 0];
      else
        q = q_raw / q_norm;
      end

      p = Rb0' * model.state.p;
      v = Rb0' * model.state.v;
      w = model.state.w;

      x = [q; p; v; w];

      % Reference transformation
      xd(1:3)   = Rb0' * xd(1:3);
      xd(4)     = 0;
      xd(5:7)   = Rb0' * xd(5:7);
      xd(9:11)  = Rb0' * xd(9:11);
      xd(13:15) = Rb0' * xd(13:15);
      xd(17:19) = Rb0' * xd(17:19);

      % =====================================================================
      % 4. Nominal DFL Controller
      % =====================================================================
      if dt <= obj.param.dt
        vf = Vfd(dt, x, xd', P', F1);
        vs = Vsd(dt, x, xd', vf, P', F2, F3, F4);
      else
        vf = Vf(x, xd', P', F1);
        vs = Vs(x, xd', vf, P', F2, F3, F4);
      end

      tmp_nom = Uf(x, xd', vf, P') + Us(x, xd', vf, vs', P');

      % =====================================================================
      % 5. Physical Limits & Nominal Input Clamping
      % =====================================================================
      m_drone = P(1);
      g_drone = P(9);

      T_hover = m_drone * g_drone;
      T_min   = 0.70 * T_hover;
      T_max   = 20.0;
      tau_max = 1.0;
      r_drone = 0.15; % ドローン外接半径 [m]

      u_nom = [ ...
        max(T_min,   min(T_max,   tmp_nom(1))); ...
        max(-tau_max, min(tau_max, tmp_nom(2))); ...
        max(-tau_max, min(tau_max, tmp_nom(3))); ...
        max(-tau_max, min(tau_max, tmp_nom(4))) ...
      ];

      % =====================================================================
      % 6. Thrust History Initialization
      % =====================================================================
      if isempty(obj.T_prev1) || ~isfinite(obj.T_prev1)
        obj.T_prev1 = u_nom(1);
      end
      if isempty(obj.T_prev2) || ~isfinite(obj.T_prev2)
        obj.T_prev2 = obj.T_prev1;
      end

      % =====================================================================
      % 7. FastBridge Collision Cone ECBF Constraints (FD Formulation)
      % =====================================================================
      obs_list = ENVIRONMENT_OBSTACLE_HOCBF_LINK_XY();

      A_cbf_all = [];
      b_cbf_all = [];

      lambda_cbf   = 1.2;  % 急峻なトルク要求を抑制するチューニング値
      d_detect     = 5.0;  % 検知範囲 [m]
      r_margin     = 0.5; % 安全マージン [m]
      approach_eps = 1e-3; % 接近判定許容差

      if ~isempty(obs_list)
        for i = 1:length(obs_list)
          p_obs_world = obs_list(i).p_obs;
          r_obs       = obs_list(i).r_obs_margin;

          % Yaw基準座標系へ変換
          p_obs_rel = Rb0' * p_obs_world;

          r_effective = r_obs + r_drone + r_margin;
          dist_center = norm(p_obs_rel - p);

          % 1. Distance gate
          if dist_center > r_effective + d_detect
            continue;
          end

          % 2. 楕円体共分散逆行列の構築 (球体膨張)
          A_mat = eye(3) / (r_effective^2);
          A_vec = [ ...
            A_mat(1,1); A_mat(1,2); A_mat(1,3); ...
            A_mat(2,2); A_mat(2,3); A_mat(3,3) ...
          ];

          c_scale_val  = 1.0;
          splat_params = [p_obs_rel; A_vec; c_scale_val];

          % 3. Approach gate (r = mu - p に対し r'*A*v >= -eps で接近中)
          r_vec = p_obs_rel - p;
          delta_val = r_vec' * A_mat * v;

          if delta_val >= -approach_eps
            z_base = x; % [q; p; v; w] (13次元)

            [A_i, b_i] = FastBridge_CBF_QP_FD( ...
              z_base(:), ...
              splat_params(:), ...
              P, ...
              lambda_cbf, ...
              obj.T_prev1, ...
              obj.T_prev2, ...
              dt);

            A_i = real(double(A_i));
            b_i = real(double(b_i));

            if all(isfinite(A_i(:))) && all(isfinite(b_i(:)))
              A_cbf_all = [A_cbf_all; A_i];
              b_cbf_all = [b_cbf_all; b_i];
            end
          end
          % % --- 2. 純粋位置距離バリア (至近距離での絶対防衛ライン) ---
          % % 既存の位置ECBF関数があれば連立してハード制約化
          % if exist('Pos_ECBF_Acbf', 'file') == 2
          %     obsParam_pos = [p_obs_rel', [0,0,0], r_obs, r_drone + r_margin];
          %     A_pos = real(Pos_ECBF_Acbf(x, obsParam_pos, P', obj.T_prev1));
          %     b_pos = real(Pos_ECBF_bcbf(x, obsParam_pos, P', obj.T_prev1, 1.5));
          %     if all(isfinite(A_pos(:))) && all(isfinite(b_pos(:)))
          %         A_cbf_all = [A_cbf_all; A_pos];
          %         b_cbf_all = [b_cbf_all; b_pos];
          %     end
          % end
        end
      end

      % =====================================================================
      % 8. Attitude CBF Constraints (Soft Constraint)
      % =====================================================================
      max_tilt  = deg2rad(35);
      gamma_att = 3.0;

      A_att = real(Attitude_CBF_Acbf(x, P'));
      b_att = real(Attitude_CBF_bcbf(x, P', max_tilt, gamma_att));

      n_cbf = size(A_cbf_all, 1);
      n_att = size(A_att, 1);

      % =====================================================================
      % 9. Primary QP Setup (Hard Collision CBF + Soft Attitude CBF)
      % =====================================================================
      w_T      = 10.0;
      w_tau_xy = 1.0;
      w_tau_z  = 200.0;
      w_xi_att = 50000.0;

      H_u = diag([w_T, w_tau_xy, w_tau_xy, w_tau_z]);
      f_u = -H_u * u_nom;

      H_qp = blkdiag(H_u, w_xi_att * eye(n_att));
      f_qp = [f_u; zeros(n_att, 1)];

      % 姿勢制約の4変数マッピング
      if size(A_att, 2) == 4
        A_att_u = A_att;
      elseif size(A_att, 2) == 3
        A_att_u = [zeros(n_att, 1), A_att];
      else
        error('Attitude_CBF_Acbf output dimension is invalid.');
      end

      if n_cbf > 0
        A_collision = [A_cbf_all, zeros(n_cbf, n_att)];
        b_collision = b_cbf_all;
      else
        A_collision = zeros(0, 4 + n_att);
        b_collision = zeros(0, 1);
      end

      A_att_qp = [A_att_u, -eye(n_att)];
      b_att_qp = b_att;

      A_qp = [A_collision; A_att_qp];
      b_qp = [b_collision; b_att_qp];

      lb = [T_min; -tau_max; -tau_max; -tau_max; zeros(n_att, 1)];
      ub = [T_max;  tau_max;  tau_max;  tau_max; inf(n_att, 1)];

      options = optimoptions( ...
        'quadprog', ...
        'Display', 'off', ...
        'Algorithm', 'interior-point-convex');

      [z_opt, ~, exitflag] = quadprog( ...
        real(H_qp), real(f_qp), real(A_qp), real(b_qp), ...
        [], [], lb, ub, [], options);

      % =====================================================================
      % 10. Feasibility Fallback (2段階緩和QP & 緊急制動)
      % =====================================================================
      if exitflag == 1
        u_safe = z_opt(1:4);
      else
        % 【第2段階】ハード制約が競合した場合のスラック緩和QP (安全最優先ペナルティ)
        w_xi_cbf = 1e6;
        num_slacks_em = n_cbf + n_att;

        H_em = blkdiag(H_u, w_xi_cbf * eye(n_cbf), w_xi_att * eye(n_att));
        f_em = [f_u; zeros(num_slacks_em, 1)];

        A_em = [ ...
          A_cbf_all,  -eye(n_cbf),          zeros(n_cbf, n_att); ...
          A_att_u,    zeros(n_att, n_cbf), -eye(n_att) ...
        ];
        b_em = [b_cbf_all; b_att];

        lb_em = [T_min; -tau_max; -tau_max; -tau_max; zeros(num_slacks_em, 1)];
        ub_em = [T_max;  tau_max;  tau_max;  tau_max; inf(num_slacks_em, 1)];

        [z_em, ~, exitflag_em] = quadprog( ...
          real(H_em), real(f_em), real(A_em), real(b_em), ...
          [], [], lb_em, ub_em, [], options);

        if exitflag_em == 1
          u_safe = z_em(1:4);
        else
          % 【第3段階】万が一の完全緊急ホバリング制動 (FastBridge Backup Policy)
          k_brake = 1.5;
          u_safe = [ ...
            T_hover; ...
            max(-tau_max, min(tau_max, -2.0 * w(1) + k_brake * v(2))); ...
            max(-tau_max, min(tau_max, -2.0 * w(2) - k_brake * v(1))); ...
            max(-tau_max, min(tau_max, -1.0 * w(3))) ...
          ];
        end
      end

      % =====================================================================
      % 11. Actuator Saturation & Thrust History Update
      % =====================================================================
      T_out     = max(T_min,   min(T_max,   u_safe(1)));
      tau_x_out = max(-tau_max, min(tau_max, u_safe(2)));
      tau_y_out = max(-tau_max, min(tau_max, u_safe(3)));
      tau_z_out = max(-tau_max, min(tau_max, u_safe(4)));

      % 実際にアクチュエータへ送られた推力を履歴に保存
      obj.T_prev2 = obj.T_prev1;
      obj.T_prev1 = T_out;
      obj.initialized = true;

      % =====================================================================
      % 12. Output & Clearance Calculation
      % =====================================================================
      obj.result.input = [ ...
        T_out; ...
        tau_x_out; ...
        tau_y_out; ...
        tau_z_out ...
      ];

      min_clearance = Inf;
      if ~isempty(obs_list)
        p_drone_curr = x(5:7);
        for i = 1:length(obs_list)
          p_obs_rel = Rb0' * obs_list(i).p_obs;
          r_obs     = obs_list(i).r_obs_margin;

          dist_center = norm(p_obs_rel - p_drone_curr);
          clearance   = dist_center - (r_obs + r_drone);

          if clearance < min_clearance
            min_clearance = clearance;
          end
        end
      end

      obj.result.min_clearance       = min_clearance;
      obj.result.qp_exitflag         = exitflag;
      obj.result.num_cbf_constraints = n_cbf;
      obj.result.thrust_history      = [obj.T_prev1; obj.T_prev2];

      result = obj.result;
    end

  end
end
% classdef HLC_SUSPENDED_LOAD_ELLIPSOID_CBF < HLC_SUSPENDED_LOAD
%   properties
%     T_val
%     dT_val
%     initialized
%   end
% 
%   methods
%     function obj = HLC_SUSPENDED_LOAD_ELLIPSOID_CBF(self, param)
%         obj@HLC_SUSPENDED_LOAD(self, param);
%         if ~isfield(obj.result, 'min_clearance')
%             obj.result.min_clearance = Inf;
%         end
%         obj.T_val = 0;
%         obj.dT_val = 0;
%         obj.initialized = false;
%     end
% 
%     function result = do(obj, varargin)
%         res_nom = do@HLC_SUSPENDED_LOAD(obj, varargin{:});
%         u_nom = res_nom.tmp; 
% 
%         T_nom = u_nom(1);
%         tau_nom = u_nom(2:4);
% 
%         if ~obj.initialized
%             obj.T_val = T_nom;
%             obj.dT_val = 0;
%             obj.initialized = true;
%         end
% 
%         model = obj.self.estimator.result;
%         P_vec = obj.self.parameter.get(["mass", "jx", "jy", "jz", "gravity", "loadmass", "cableL"]);
%         dt = obj.param.dt;
% 
%         p_load = model.state.pL;
%         v_load = model.state.vL;
%         pT = model.state.pT;
%         wL = model.state.wL;
%         q_quat = R2q(model.state.getq("rotmat"));
%         w_vec = model.state.w;
% 
%         z_state = [p_load(:); v_load(:); pT(:); wL(:); q_quat(:); w_vec(:)];
%         T_state = [obj.T_val; obj.dT_val];
% 
%         M_total = P_vec(1) + P_vec(6); 
%         T_hover = M_total * P_vec(5); 
% 
%         if ~obj.initialized || isnan(obj.T_val)
%             obj.T_val = T_hover;
%             obj.dT_val = 0;
%             obj.initialized = true;
%         end
% 
%         Kp_T = 400.0; Kd_T = 40.0;
%         ddT_nom = -Kp_T * (obj.T_val - T_nom) - Kd_T * obj.dT_val;
%         mu_nom = [ddT_nom; tau_nom];
% 
%         H_qp = diag([5, 5, 5, 5]); 
%         f_qp = -H_qp * mu_nom;
% 
%         A_obs = [];
%         b_obs = [];
%         min_h_this_step = Inf;
%         min_d_surf_this_step = Inf;
% 
%         drone_params = [P_vec(1); P_vec(6); P_vec(7); P_vec(2); P_vec(3); P_vec(4); P_vec(5)]; 
%         p_mid = p_load - 0.5 * P_vec(7) * pT;
% 
%         % 自機楕円体パラメータ
%         a_rad = 0.3;
%         b_rad = P_vec(7) / 2.0 + 0.1;
%         ellipsoid_params = [a_rad; b_rad];
% 
%         % 新しい楕円体環境を読み込み
%         obs_list = ENVIRONMENT_OBSTACLE_ELLIPSOID();
%         for i = 1:length(obs_list)
%             p_obs = obs_list(i).p_obs;
%             R_obs = obs_list(i).R_obs;
%             Q_obs = obs_list(i).Q_obs;
%             d_margin = obs_list(i).d_margin;
% 
%             Q2_obs = R_obs * (Q_obs^2) * R_obs';
% 
%             % 最適な分離超平面の法線 n_vec を勾配法で探索
%             n_vec = p_mid - p_obs; 
%             if norm(n_vec) > 1e-6, n_vec = n_vec / norm(n_vec); else, n_vec = [0;0;1]; end
% 
%             for k = 1:5
%                 r_sys_ext = sqrt(a_rad^2 + (b_rad^2 - a_rad^2)*(n_vec'*pT)^2);
%                 r_obs_ext = sqrt(n_vec' * Q2_obs * n_vec);
% 
%                 grad_r_sys = ((b_rad^2 - a_rad^2)*(n_vec'*pT) / max(1e-6, r_sys_ext)) * pT;
%                 grad_r_obs = (Q2_obs * n_vec) / max(1e-6, r_obs_ext);
% 
%                 grad_n = (p_mid - p_obs) - grad_r_sys - grad_r_obs;
%                 n_vec = n_vec + 0.2 * grad_n;
%                 n_vec = n_vec / norm(n_vec);
%             end
% 
%             % 障害物側の表面接平面の距離 d_plane (安全マージンを含む)
%             r_obs_ext = sqrt(n_vec' * Q2_obs * n_vec);
%             d_plane = n_vec' * p_obs + r_obs_ext + d_margin;
%             plane_params = [n_vec; d_plane];
% 
%             cbf_gains = [16; 40; 33; 10]; 
% 
%             [A_cbf, b_cbf, h_val, dh, ddh, dddh] = Ellipsoid_SuspendedLoad_TetherMid_CBF(z_state, T_state, drone_params, ellipsoid_params, plane_params, cbf_gains);
% 
%             d_surf = h_val;
%             if h_val < min_h_this_step, min_h_this_step = h_val; end
%             if d_surf < min_d_surf_this_step, min_d_surf_this_step = d_surf; end
%             if h_val < obj.result.min_clearance, obj.result.min_clearance = h_val; end
% 
%             if d_surf < 4.0
%                 if ~any(isnan(A_cbf)) && ~any(isnan(b_cbf))
%                     A_obs = [A_obs; A_cbf];
%                     b_obs = [b_obs; b_cbf];
%                 end
%             end
%         end
%         obj.result.h_val = min_h_this_step; 
%         obj.result.d_surf = min_d_surf_this_step; 
% 
%         % ★ 姿勢ハード制約 (Attitude Limit CBF)
%         % ロール角・ピッチ角が35度(約0.61rad)以上傾かないようにする
%         cos_gamma_max = cos(35 * pi / 180);
%         cbf_gains_att = [25; 10]; % (s+5)^2 の極配置
%         [A_att, b_att, h_att] = Attitude_Limit_CBF([q_quat(:); w_vec(:)], [P_vec(2); P_vec(3); P_vec(4)], cos_gamma_max, cbf_gains_att);
% 
%         if ~any(isnan(A_att)) && ~any(isnan(b_att))
%             A_obs = [A_obs; A_att];
%             b_obs = [b_obs; b_att];
%         end
% 
%         % 5. Solve QP
%         mu_safe = mu_nom;
%         if ~isempty(A_obs)
%             % lb = [-1000; -1.0; -1.0; -1.0];
%             % ub = [ 1000;  1.0;  1.0;  1.0];
%             lb = [-1000; -0.5; -0.5; -0.5];
%             ub = [ 1000;  0.5;  0.5;  0.5];
%             options = optimoptions('quadprog', 'Display', 'off');
%             [mu_opt, ~, exitflag] = quadprog(H_qp, f_qp, A_obs, b_obs, [], [], lb, ub, [], options);
%             if exitflag == 1
%                 mu_safe = mu_opt;
%             end
%         end
% 
%         obj.dT_val = obj.dT_val + mu_safe(1) * dt;
%         obj.T_val  = obj.T_val  + obj.dT_val * dt;
% 
%         T_min = T_hover * 0.1; 
%         T_max = max(20.0, T_hover * 2.0);
%         obj.T_val = max(T_min, min(T_max, obj.T_val)); 
% 
%         tau_safe = max(-1.0, min(1.0, mu_safe(2:4)));
%         obj.result.input = [obj.T_val; tau_safe];
%         result = obj.result;
%     end
%   end
% end

% classdef HLC_SUSPENDED_LOAD_ELLIPSOID_CBF < HLC_SUSPENDED_LOAD
%     % =========================================================================
%     % 動的超平面法に基づく 4階高次制御バリア関数 (HOCBF-QP)
%     % (3次元接線循環流・スラック変数・行列次元自動判定 確定版)
%     % =========================================================================
%     properties
%         T_val
%         dT_val
%         initialized
%     end
% 
%     methods
%         function obj = HLC_SUSPENDED_LOAD_ELLIPSOID_CBF(self, param)
%             obj@HLC_SUSPENDED_LOAD(self, param);
%             if ~isfield(obj.result, 'min_clearance')
%                 obj.result.min_clearance = Inf;
%             end
%             obj.T_val = 0;
%             obj.dT_val = 0;
%             obj.initialized = false;
%         end
% 
%         function result = do(obj, varargin)
%             res_nom = do@HLC_SUSPENDED_LOAD(obj, varargin{:});
%             u_nom = res_nom.tmp; 
% 
%             T_nom = u_nom(1);
%             tau_nom = u_nom(2:4);
% 
%             model = obj.self.estimator.result;
%             P_vec = obj.self.parameter.get(["mass", "jx", "jy", "jz", "gravity", "loadmass", "cableL"]);
%             dt = obj.param.dt;
% 
%             M_total = P_vec(1) + P_vec(6); 
%             T_hover = M_total * P_vec(5); 
% 
%             if ~obj.initialized || isnan(obj.T_val)
%                 obj.T_val = T_nom;
%                 obj.dT_val = 0;
%                 obj.initialized = true;
%             end
% 
%             p_load = model.state.pL;
%             v_load = model.state.vL;
%             pT = model.state.pT;
%             wL = model.state.wL;
%             q_quat = R2q(model.state.getq("rotmat"));
%             w_vec = model.state.w;
% 
%             z_state = [p_load(:); v_load(:); pT(:); wL(:); q_quat(:); w_vec(:)];
%             T_state = [obj.T_val; obj.dT_val];
% 
%             Kp_T = 400.0; Kd_T = 40.0;
%             ddT_nom = -Kp_T * (obj.T_val - T_nom) - Kd_T * obj.dT_val;
% 
%             mu_nom = [ddT_nom; tau_nom(:)];
% 
%             A_obs = zeros(0, 5); % 5列 (ddT, tau_x, tau_y, tau_z, slack) で初期化
%             b_obs = zeros(0, 1);
%             tau_circ = zeros(3, 1);
% 
%             min_h_this_step = Inf;
%             min_d_surf_this_step = Inf;
%             drone_params = [P_vec(1); P_vec(6); P_vec(7); P_vec(2); P_vec(3); P_vec(4); P_vec(5)]; 
%             p_mid = p_load - 0.5 * P_vec(7) * pT;
% 
%             a_rad = 0.35;
%             b_rad = P_vec(7) / 2.0 + 0.15;
%             ellipsoid_params = [a_rad; b_rad];
% 
%             obs_list = ENVIRONMENT_OBSTACLE_ELLIPSOID();
% 
%             for i = 1:length(obs_list)
%                 p_obs = obs_list(i).p_obs;
%                 R_obs = obs_list(i).R_obs;
%                 Q_obs = obs_list(i).Q_obs;
%                 d_margin = obs_list(i).d_margin;
% 
%                 Q2_obs = R_obs * (Q_obs^2) * R_obs';
% 
%                 diff_center = p_mid - p_obs;
%                 if norm(diff_center) > 1e-4
%                     n_vec = diff_center / norm(diff_center);
%                 else
%                     n_vec = [1; 0; 0];
%                 end
% 
%                 if abs(n_vec(3)) > 0.90
%                     n_vec = n_vec + [0.15; 0.05; 0];
%                     n_vec = n_vec / norm(n_vec);
%                 end
% 
%                 for k = 1:8
%                     r_sys_ext = sqrt(a_rad^2 + (b_rad^2 - a_rad^2) * (n_vec' * pT)^2);
%                     r_obs_ext = sqrt(n_vec' * Q2_obs * n_vec);
% 
%                     grad_r_sys = ((b_rad^2 - a_rad^2) * (n_vec' * pT) / max(1e-6, r_sys_ext)) * pT;
%                     grad_r_obs = (Q2_obs * n_vec) / max(1e-6, r_obs_ext);
% 
%                     grad_n = diff_center - grad_r_sys - grad_r_obs;
%                     n_vec = n_vec + 0.15 * grad_n;
%                     n_vec = n_vec / norm(n_vec);
%                 end
% 
%                 r_obs_ext = sqrt(n_vec' * Q2_obs * n_vec);
%                 d_plane = n_vec' * p_obs + r_obs_ext + d_margin;
%                 plane_params = [n_vec; d_plane];
% 
%                 cbf_gains = [39.0; 62.5; 37.5; 10.0]; 
% 
%                 [A_cbf, b_cbf, h_val, ~, ~, ~] = Ellipsoid_SuspendedLoad_TetherMid_CBF( ...
%                     z_state, T_state, drone_params, ellipsoid_params, plane_params, cbf_gains);
% 
%                 if h_val < min_h_this_step; min_h_this_step = h_val; end
%                 if h_val < min_d_surf_this_step; min_d_surf_this_step = h_val; end
%                 if h_val < obj.result.min_clearance; obj.result.min_clearance = h_val; end
% 
%                 if h_val < 4.0
%                     v_nom_dir = [0; 0; 1.0];
%                     tangent_dir = cross(n_vec, cross(v_nom_dir, n_vec));
%                     if norm(tangent_dir) < 0.1 || abs(n_vec(3)) > 0.85
%                         tangent_dir = cross(n_vec, [0; 1; 0]);
%                         if norm(tangent_dir) < 0.1; tangent_dir = cross(n_vec, [1; 0; 0]); end
%                     end
%                     tangent_dir = tangent_dir / norm(tangent_dir);
% 
%                     tau_circ = tau_circ + 0.3 * [tangent_dir(2); -tangent_dir(1); 0];
% 
%                     if ~any(isnan(A_cbf(:))) && ~any(isnan(b_cbf(:)))
%                         % A_cbf を 1x4 行ベクトルとして確実に整形し、-1.0 を結合 (1x5)
%                         A_cbf_row = reshape(A_cbf, 1, 4);
%                         A_obs = [A_obs; [A_cbf_row, -1.0]];
%                         b_obs = [b_obs; b_cbf(:)];
%                     end
%                 end
%             end
% 
%             obj.result.h_val = min_h_this_step; 
%             obj.result.d_surf = min_d_surf_this_step; 
% 
%             % 姿勢制約 (Attitude Limit CBF)
%             cos_gamma_max = cos(35 * pi / 180);
%             cbf_gains_att = [25; 10];
%             [A_att, b_att, ~] = Attitude_Limit_CBF([q_quat(:); w_vec(:)], [P_vec(2); P_vec(3); P_vec(4)], cos_gamma_max, cbf_gains_att);
% 
%             if ~any(isnan(A_att(:))) && ~any(isnan(b_att(:)))
%                 % 要素数チェックと自動 5 列パディング
%                 n_rows = size(A_att, 1);
%                 n_cols = numel(A_att) / n_rows;
%                 A_att_mat = reshape(A_att, n_rows, n_cols);
% 
%                 if n_cols == 3
%                     A_att_5col = [zeros(n_rows, 1), A_att_mat, zeros(n_rows, 1)];
%                 elseif n_cols == 4
%                     A_att_5col = [A_att_mat, zeros(n_rows, 1)];
%                 else
%                     A_att_flat = A_att(:)';
%                     if length(A_att_flat) >= 4
%                         A_att_5col = [A_att_flat(1:4), 0.0];
%                     else
%                         A_att_5col = [0.0, A_att_flat, zeros(1, 4 - length(A_att_flat))];
%                     end
%                 end
% 
%                 A_obs = [A_obs; A_att_5col];
%                 b_obs = [b_obs; b_att(:)];
%             end
% 
%             % スラック付き 5変数 QP の定義
%             w_mu = 5.0;
%             w_slack = 1e6;
%             H_qp = blkdiag(w_mu * eye(4), w_slack);
% 
%             mu_target = mu_nom + [0; tau_circ];
%             f_qp = [-w_mu * mu_target; 0];
% 
%             lb = [-1000; -0.8; -0.8; -0.8; 0];
%             ub = [ 1000;  0.8;  0.8;  0.8; inf];
% 
%             mu_safe = mu_nom;
%             if ~isempty(A_obs)
%                 options = optimoptions('quadprog', 'Display', 'off');
%                 [mu_opt, ~, exitflag] = quadprog(H_qp, f_qp, A_obs, b_obs, [], [], lb, ub, [], options);
%                 if exitflag == 1
%                     mu_safe = mu_opt(1:4);
%                 end
%             end
% 
%             obj.dT_val = obj.dT_val + mu_safe(1) * dt;
%             obj.T_val  = obj.T_val  + obj.dT_val * dt;
% 
%             T_min = T_hover * 0.4;
%             T_max = max(20.0, T_hover * 2.2);
%             obj.T_val = max(T_min, min(T_max, obj.T_val)); 
% 
%             tau_safe = max(-1.0, min(1.0, mu_safe(2:4)));
%             obj.result.input = [obj.T_val; tau_safe];
%             result = obj.result;
%         end
%     end
% end

% classdef HLC_SUSPENDED_LOAD_ELLIPSOID_CBF < HLC_SUSPENDED_LOAD
%     % =========================================================================
%     % 動的超平面 4階高次制御バリア関数 (HOCBF-QP)
%     % 【推力・トルク物理限界厳密射影 & パニック時受動制動フォールバック版】
%     % =========================================================================
%     properties
%         T_val
%         dT_val
%         initialized
%     end
% 
%     methods
%         function obj = HLC_SUSPENDED_LOAD_ELLIPSOID_CBF(self, param)
%             obj@HLC_SUSPENDED_LOAD(self, param);
%             if ~isfield(obj.result, 'min_clearance')
%                 obj.result.min_clearance = Inf;
%             end
%             obj.T_val = 0;
%             obj.dT_val = 0;
%             obj.initialized = false;
%         end
% 
%         function result = do(obj, varargin)
%             res_nom = do@HLC_SUSPENDED_LOAD(obj, varargin{:});
%             u_nom = res_nom.tmp; 
% 
%             T_nom = u_nom(1);
%             tau_nom = u_nom(2:4);
% 
%             model = obj.self.estimator.result;
%             P_vec = obj.self.parameter.get(["mass", "jx", "jy", "jz", "gravity", "loadmass", "cableL"]);
%             dt = obj.param.dt;
% 
%             m_drone = P_vec(1);
%             m_load  = P_vec(6);
%             M_total = m_drone + m_load;
%             g_acc   = P_vec(5);
%             T_hover = M_total * g_acc; 
%             J_mat   = diag([P_vec(2); P_vec(3); P_vec(4)]);
% 
%             if ~obj.initialized || isnan(obj.T_val)
%                 obj.T_val = T_nom;
%                 obj.dT_val = 0;
%                 obj.initialized = true;
%             end
% 
%             p_load = model.state.pL;
%             v_load = model.state.vL;
%             pT = model.state.pT;
%             wL = model.state.wL;
%             q_quat = R2q(model.state.getq("rotmat"));
%             w_vec = model.state.w;
% 
%             z_state = [p_load(:); v_load(:); pT(:); wL(:); q_quat(:); w_vec(:)];
%             T_state = [obj.T_val; obj.dT_val];
% 
%             % ノミナル2階微分指令
%             Kp_T = 400.0; Kd_T = 40.0;
%             ddT_nom = -Kp_T * (obj.T_val - T_nom) - Kd_T * obj.dT_val;
%             mu_nom = [ddT_nom; tau_nom(:)];
% 
%             A_obs = zeros(0, 5);
%             b_obs = zeros(0, 1);
%             tau_circ = zeros(3, 1);
% 
%             min_h_this_step = Inf;
%             min_d_surf_this_step = Inf;
%             drone_params = [m_drone; m_load; P_vec(7); P_vec(2); P_vec(3); P_vec(4); g_acc]; 
%             p_mid = p_load - 0.5 * P_vec(7) * pT;
% 
%             a_rad = 0.35;
%             b_rad = P_vec(7) / 2.0 + 0.15;
%             ellipsoid_params = [a_rad; b_rad];
% 
%             obs_list = ENVIRONMENT_OBSTACLE_ELLIPSOID();
% 
%             for i = 1:length(obs_list)
%                 p_obs = obs_list(i).p_obs;
%                 R_obs = obs_list(i).R_obs;
%                 Q_obs = obs_list(i).Q_obs;
%                 d_margin = obs_list(i).d_margin;
% 
%                 Q2_obs = R_obs * (Q_obs^2) * R_obs';
% 
%                 diff_center = p_mid - p_obs;
%                 if norm(diff_center) > 1e-4
%                     n_vec = diff_center / norm(diff_center);
%                 else
%                     n_vec = [1; 0; 0];
%                 end
% 
%                 if abs(n_vec(3)) > 0.90
%                     n_vec = n_vec + [0.15; 0.05; 0];
%                     n_vec = n_vec / norm(n_vec);
%                 end
% 
%                 for k = 1:8
%                     r_sys_ext = sqrt(a_rad^2 + (b_rad^2 - a_rad^2) * (n_vec' * pT)^2);
%                     r_obs_ext = sqrt(n_vec' * Q2_obs * n_vec);
% 
%                     grad_r_sys = ((b_rad^2 - a_rad^2) * (n_vec' * pT) / max(1e-6, r_sys_ext)) * pT;
%                     grad_r_obs = (Q2_obs * n_vec) / max(1e-6, r_obs_ext);
% 
%                     grad_n = diff_center - grad_r_sys - grad_r_obs;
%                     n_vec = n_vec + 0.15 * grad_n;
%                     n_vec = n_vec / norm(n_vec);
%                 end
% 
%                 r_obs_ext = sqrt(n_vec' * Q2_obs * n_vec);
%                 d_plane = n_vec' * p_obs + r_obs_ext + d_margin;
%                 plane_params = [n_vec; d_plane];
% 
%                 cbf_gains = [39.0; 62.5; 37.5; 10.0]; 
% 
%                 [A_cbf, b_cbf, h_val, ~, ~, ~] = Ellipsoid_SuspendedLoad_TetherMid_CBF( ...
%                     z_state, T_state, drone_params, ellipsoid_params, plane_params, cbf_gains);
% 
%                 if h_val < min_h_this_step; min_h_this_step = h_val; end
%                 if h_val < min_d_surf_this_step; min_d_surf_this_step = h_val; end
%                 if h_val < obj.result.min_clearance; obj.result.min_clearance = h_val; end
% 
%                 if h_val < 4.0
%                     v_nom_dir = [0; 0; 1.0];
%                     tangent_dir = cross(n_vec, cross(v_nom_dir, n_vec));
%                     if norm(tangent_dir) < 0.1 || abs(n_vec(3)) > 0.85
%                         tangent_dir = cross(n_vec, [0; 1; 0]);
%                         if norm(tangent_dir) < 0.1; tangent_dir = cross(n_vec, [1; 0; 0]); end
%                     end
%                     tangent_dir = tangent_dir / norm(tangent_dir);
% 
%                     tau_circ = tau_circ + 0.3 * [tangent_dir(2); -tangent_dir(1); 0];
% 
%                     if ~any(isnan(A_cbf(:))) && ~any(isnan(b_cbf(:)))
%                         A_cbf_row = reshape(A_cbf, 1, 4);
%                         A_obs = [A_obs; [A_cbf_row, -1.0]];
%                         b_obs = [b_obs; b_cbf(:)];
%                     end
%                 end
%             end
% 
%             obj.result.h_val = min_h_this_step; 
%             obj.result.d_surf = min_d_surf_this_step; 
% 
%             % 姿勢制約
%             cos_gamma_max = cos(35 * pi / 180);
%             cbf_gains_att = [25; 10];
%             [A_att, b_att, ~] = Attitude_Limit_CBF([q_quat(:); w_vec(:)], [P_vec(2); P_vec(3); P_vec(4)], cos_gamma_max, cbf_gains_att);
% 
%             if ~any(isnan(A_att(:))) && ~any(isnan(b_att(:)))
%                 n_rows = size(A_att, 1);
%                 n_cols = numel(A_att) / n_rows;
%                 A_att_mat = reshape(A_att, n_rows, n_cols);
% 
%                 if n_cols == 3
%                     A_att_5col = [zeros(n_rows, 1), A_att_mat, zeros(n_rows, 1)];
%                 elseif n_cols == 4
%                     A_att_5col = [A_att_mat, zeros(n_rows, 1)];
%                 else
%                     A_att_flat = A_att(:)';
%                     if length(A_att_flat) >= 4
%                         A_att_5col = [A_att_flat(1:4), 0.0];
%                     else
%                         A_att_5col = [0.0, A_att_flat, zeros(1, 4 - length(A_att_flat))];
%                     end
%                 end
%                 A_obs = [A_obs; A_att_5col];
%                 b_obs = [b_obs; b_att(:)];
%             end
% 
%             % ========================================================
%             % 推力 [0, 20] N と トルク [-1, 1] Nm の厳密な物理限界バウンド
%             % ========================================================
%             % T_{k+1} = T_k + dT_k*dt + 0.5*ddT*dt^2
%             dt2_half = 0.5 * (dt^2);
%             T_pred_base = obj.T_val + obj.dT_val * dt;
% 
%             T_limit_min = 2.0;  % 最小推力 (特異点回避・完全落下防止)
%             T_limit_max = 20.0; % 最大推力
% 
%             ddT_lb = (T_limit_min - T_pred_base) / dt2_half;
%             ddT_ub = (T_limit_max - T_pred_base) / dt2_half;
% 
%             % 数値安定性のためのクランプ
%             ddT_lb = max(-1500.0, min(1500.0, ddT_lb));
%             ddT_ub = max(-1500.0, min(1500.0, ddT_ub));
%             if ddT_lb > ddT_ub; tmp = ddT_lb; ddT_lb = ddT_ub; ddT_ub = tmp; end
% 
%             % トルクは厳密に [-1.0, 1.0] Nm
%             lb = [ddT_lb; -1.0; -1.0; -1.0; 0.0];
%             ub = [ddT_ub;  1.0;  1.0;  1.0; inf];
% 
%             % QP コスト設定
%             w_mu = 5.0;
%             w_slack = 1e6;
%             H_qp = blkdiag(w_mu * eye(4), w_slack);
% 
%             mu_target = mu_nom + [0; tau_circ];
%             f_qp = [-w_mu * mu_target; 0];
% 
%             options = optimoptions('quadprog', 'Display', 'off');
%             exitflag = -1;
% 
%             if ~isempty(A_obs)
%                 [mu_opt, ~, exitflag] = quadprog(H_qp, f_qp, A_obs, b_obs, [], [], lb, ub, [], options);
%             else
%                 exitflag = 1;
%                 mu_opt = [min(max(mu_target(1), ddT_lb), ddT_ub); ...
%                           min(max(mu_target(2:4), -1.0), 1.0); ...
%                           0.0];
%             end
% 
%             % ========================================================
%             % 【解なし（Infeasible）時の受動制動フォールバック】
%             % ========================================================
%             if exitflag == 1
%                 mu_safe = mu_opt(1:4);
%             else
%                 % 1. 推力系統: 自重ホバリングに向けて臨界制動 (急激な跳ね上がり・落下を抑制)
%                 ddT_fallback = -100.0 * (obj.T_val - T_hover) - 20.0 * obj.dT_val;
%                 ddT_fallback = min(max(ddT_fallback, ddT_lb), ddT_ub);
% 
%                 % 2. トルク系統: 角速度ダンパにより機体回転エネルギーを急速吸収 (水平姿勢復帰)
%                 tau_fallback = -2.5 * J_mat * w_vec;
%                 tau_fallback = min(max(tau_fallback, -1.0), 1.0);
% 
%                 mu_safe = [ddT_fallback; tau_fallback];
%             end
% 
%             % 状態更新
%             obj.dT_val = obj.dT_val + mu_safe(1) * dt;
%             obj.T_val  = obj.T_val  + obj.dT_val * dt;
% 
%             % 最終物理リミットガード
%             obj.T_val = max(T_limit_min, min(T_limit_max, obj.T_val));
%             tau_safe  = max(-1.0, min(1.0, mu_safe(2:4)));
% 
%             obj.result.input = [obj.T_val; tau_safe];
%             result = obj.result;
%         end
%     end
% end

% classdef HLC_SUSPENDED_LOAD_ELLIPSOID_CBF < HLC_SUSPENDED_LOAD
%     % =========================================================================
%     % 動的超平面 4階高次制御バリア関数 (HOCBF-QP)
%     % 【幾何学的包含完全保証 & 物理限界厳密射影 & パニック時受動制動版】
%     % =========================================================================
%     properties
%         T_val
%         dT_val
%         initialized
%     end
% 
%     methods
%         function obj = HLC_SUSPENDED_LOAD_ELLIPSOID_CBF(self, param)
%             obj@HLC_SUSPENDED_LOAD(self, param);
%             if ~isfield(obj.result, 'min_clearance')
%                 obj.result.min_clearance = Inf;
%             end
%             obj.T_val = 0;
%             obj.dT_val = 0;
%             obj.initialized = false;
%         end
% 
%         function result = do(obj, varargin)
%             res_nom = do@HLC_SUSPENDED_LOAD(obj, varargin{:});
%             u_nom = res_nom.tmp; 
% 
%             T_nom = u_nom(1);
%             tau_nom = u_nom(2:4);
% 
%             model = obj.self.estimator.result;
%             P_vec = obj.self.parameter.get(["mass", "jx", "jy", "jz", "gravity", "loadmass", "cableL"]);
%             dt = obj.param.dt;
% 
%             m_drone = P_vec(1);
%             m_load  = P_vec(6);
%             M_total = m_drone + m_load;
%             g_acc   = P_vec(5);
%             T_hover = M_total * g_acc; 
%             J_mat   = diag([P_vec(2); P_vec(3); P_vec(4)]);
% 
%             if ~obj.initialized || isnan(obj.T_val)
%                 obj.T_val = T_nom;
%                 obj.dT_val = 0;
%                 obj.initialized = true;
%             end
% 
%             p_drone = model.state.p;
%             p_load  = model.state.pL;
%             v_load  = model.state.vL;
%             pT      = model.state.pT;
%             wL      = model.state.wL;
%             q_quat  = R2q(model.state.getq("rotmat"));
%             w_vec   = model.state.w;
% 
%             z_state = [p_load(:); v_load(:); pT(:); wL(:); q_quat(:); w_vec(:)];
%             T_state = [obj.T_val; obj.dT_val];
% 
%             % ノミナル2階微分指令
%             Kp_T = 400.0; Kd_T = 40.0;
%             ddT_nom = -Kp_T * (obj.T_val - T_nom) - Kd_T * obj.dT_val;
%             mu_nom = [ddT_nom; tau_nom(:)];
% 
%             A_obs = zeros(0, 5);
%             b_obs = zeros(0, 1);
%             tau_circ = zeros(3, 1);
% 
%             min_h_this_step = Inf;
%             min_d_surf_this_step = Inf;
%             drone_params = [m_drone; m_load; P_vec(7); P_vec(2); P_vec(3); P_vec(4); g_acc]; 
% 
%             % ★ 修正1: 機体位置と荷物位置から直接中点を算出し、符号の誤認を排除
%             p_mid = 0.5 * (p_drone + p_load);
% 
%             % ★ 修正2: 長軸マージンを +0.15 -> +0.35 に拡張（プロペラ・天面フレーム完全包含）
%             a_rad = 0.35;                      % 短軸半径 [m]
%             b_rad = (P_vec(7) / 2.0) + 0.35;   % 長軸半径 [m]
%             ellipsoid_params = [a_rad; b_rad];
% 
%             obs_list = ENVIRONMENT_OBSTACLE_ELLIPSOID();
% 
%             for i = 1:length(obs_list)
%                 p_obs = obs_list(i).p_obs;
%                 R_obs = obs_list(i).R_obs;
%                 Q_obs = obs_list(i).Q_obs;
%                 d_margin = obs_list(i).d_margin;
% 
%                 Q2_obs = R_obs * (Q_obs^2) * R_obs';
% 
%                 diff_center = p_mid - p_obs;
%                 if norm(diff_center) > 1e-4
%                     n_vec = diff_center / norm(diff_center);
%                 else
%                     n_vec = [1; 0; 0];
%                 end
% 
%                 % 真下・真上アプローチ時の特異点破壊バイアス
%                 if abs(n_vec(3)) > 0.90
%                     n_vec = n_vec + [0.05; 0.05; 0];
%                     n_vec = n_vec / norm(n_vec);
%                 end
% 
%                 % サポート関数差分法（8回反復による真の最短法線探索）
%                 for k = 1:8
%                     r_sys_ext = sqrt(a_rad^2 + (b_rad^2 - a_rad^2) * (n_vec' * pT)^2);
%                     r_obs_ext = sqrt(n_vec' * Q2_obs * n_vec);
% 
%                     grad_r_sys = ((b_rad^2 - a_rad^2) * (n_vec' * pT) / max(1e-6, r_sys_ext)) * pT;
%                     grad_r_obs = (Q2_obs * n_vec) / max(1e-6, r_obs_ext);
% 
%                     grad_n = diff_center - grad_r_sys - grad_r_obs;
%                     n_vec = n_vec + 0.15 * grad_n;
%                     n_vec = n_vec / norm(n_vec);
%                 end
% 
%                 r_sys_ext = sqrt(a_rad^2 + (b_rad^2 - a_rad^2) * (n_vec' * pT)^2);
%                 r_obs_ext = sqrt(n_vec' * Q2_obs * n_vec);
% 
%                 % ★ 修正3: 物理表面間最短ユークリッド距離を厳密に計算
%                 dist_surface = n_vec' * diff_center - r_obs_ext - r_sys_ext;
%                 if dist_surface < min_d_surf_this_step
%                     min_d_surf_this_step = dist_surface;
%                 end
% 
%                 % 動的分離超平面パラメータ (CBF用)
%                 d_plane = n_vec' * p_obs + r_obs_ext + d_margin;
%                 plane_params = [n_vec; d_plane];
% 
%                 % cbf_gains = [39.0; 62.5; 37.5; 10.0]; 
%                 % cbf_gains = [2.07; 6.91; 8.64; 4.8];
%                 cbf_gains = [61.47; 87.81; 47.04; 11.2];
% 
%                 [A_cbf, b_cbf, h_val, ~, ~, ~] = Ellipsoid_SuspendedLoad_TetherMid_CBF( ...
%                     z_state, T_state, drone_params, ellipsoid_params, plane_params, cbf_gains);
% 
%                 if h_val < min_h_this_step; min_h_this_step = h_val; end
%                 if h_val < obj.result.min_clearance; obj.result.min_clearance = h_val; end
% 
%                 if h_val < 4.0
%                     v_nom_dir = [0; 0; 1.0];
%                     tangent_dir = cross(n_vec, cross(v_nom_dir, n_vec));
%                     if norm(tangent_dir) < 0.1 || abs(n_vec(3)) > 0.85
%                         tangent_dir = cross(n_vec, [0; 1; 0]);
%                         if norm(tangent_dir) < 0.1; tangent_dir = cross(n_vec, [1; 0; 0]); end
%                     end
%                     tangent_dir = tangent_dir / norm(tangent_dir);
% 
%                     % tau_circ = tau_circ + 0.3 * [tangent_dir(2); -tangent_dir(1); 0];
%                     tau_circ = tau_circ + 0.2 * [tangent_dir(2); -tangent_dir(1); 0];
% 
%                     if ~any(isnan(A_cbf(:))) && ~any(isnan(b_cbf(:)))
%                         A_cbf_row = reshape(A_cbf, 1, 4);
%                         A_obs = [A_obs; [A_cbf_row, -1.0]];
%                         b_obs = [b_obs; b_cbf(:)];
%                     end
%                 end
%             end
% 
%             obj.result.h_val = min_h_this_step; 
%             obj.result.d_surf = min_d_surf_this_step; % 真の物理表面間最短距離
% 
%             % 姿勢制約
%             cos_gamma_max = cos(35 * pi / 180);
%             cbf_gains_att = [25; 10];
%             [A_att, b_att, ~] = Attitude_Limit_CBF([q_quat(:); w_vec(:)], [P_vec(2); P_vec(3); P_vec(4)], cos_gamma_max, cbf_gains_att);
% 
%             if ~any(isnan(A_att(:))) && ~any(isnan(b_att(:)))
%                 n_rows = size(A_att, 1);
%                 n_cols = numel(A_att) / n_rows;
%                 A_att_mat = reshape(A_att, n_rows, n_cols);
% 
%                 if n_cols == 3
%                     A_att_5col = [zeros(n_rows, 1), A_att_mat, zeros(n_rows, 1)];
%                 elseif n_cols == 4
%                     A_att_5col = [A_att_mat, zeros(n_rows, 1)];
%                 else
%                     A_att_flat = A_att(:)';
%                     if length(A_att_flat) >= 4
%                         A_att_5col = [A_att_flat(1:4), 0.0];
%                     else
%                         A_att_5col = [0.0, A_att_flat, zeros(1, 4 - length(A_att_flat))];
%                     end
%                 end
%                 A_obs = [A_obs; A_att_5col];
%                 b_obs = [b_obs; b_att(:)];
%             end
% 
%             % ========================================================
%             % 推力 [2.0, 20.0] N と トルク [-1, 1] Nm の物理限界バウンド
%             % ========================================================
%             dt2_half = 0.5 * (dt^2);
%             T_pred_base = obj.T_val + obj.dT_val * dt;
% 
%             T_limit_min = 2.0;
%             T_limit_max = 20.0;
% 
%             ddT_lb = (T_limit_min - T_pred_base) / dt2_half;
%             ddT_ub = (T_limit_max - T_pred_base) / dt2_half;
% 
%             ddT_lb = max(-1500.0, min(1500.0, ddT_lb));
%             ddT_ub = max(-1500.0, min(1500.0, ddT_ub));
%             if ddT_lb > ddT_ub; tmp = ddT_lb; ddT_lb = ddT_ub; ddT_ub = tmp; end
% 
%             lb = [ddT_lb; -1.0; -1.0; -1.0; 0.0];
%             ub = [ddT_ub;  1.0;  1.0;  1.0; inf];
% 
%             w_mu = 5.0;
%             w_slack = 1e6;
%             H_qp = blkdiag(w_mu * eye(4), w_slack);
% 
%             mu_target = mu_nom + [0; tau_circ];
%             f_qp = [-w_mu * mu_target; 0];
% 
%             options = optimoptions('quadprog', 'Display', 'off');
%             exitflag = -1;
% 
%             if ~isempty(A_obs)
%                 [mu_opt, ~, exitflag] = quadprog(H_qp, f_qp, A_obs, b_obs, [], [], lb, ub, [], options);
%             else
%                 exitflag = 1;
%                 mu_opt = [min(max(mu_target(1), ddT_lb), ddT_ub); ...
%                           min(max(mu_target(2:4), -1.0), 1.0); ...
%                           0.0];
%             end
% 
%             % ========================================================
%             % 【解なし（Infeasible）時の受動制動フォールバック】
%             % ========================================================
%             if exitflag == 1
%                 mu_safe = mu_opt(1:4);
%             else
%                 ddT_fallback = -100.0 * (obj.T_val - T_hover) - 20.0 * obj.dT_val;
%                 ddT_fallback = min(max(ddT_fallback, ddT_lb), ddT_ub);
% 
%                 tau_fallback = -2.5 * J_mat * w_vec;
%                 tau_fallback = min(max(tau_fallback, -1.0), 1.0);
% 
%                 mu_safe = [ddT_fallback; tau_fallback];
%             end
% 
%             % 状態更新
%             obj.dT_val = obj.dT_val + mu_safe(1) * dt;
%             obj.T_val  = obj.T_val  + obj.dT_val * dt;
% 
%             obj.T_val = max(T_limit_min, min(T_limit_max, obj.T_val));
%             tau_safe  = max(-1.0, min(1.0, mu_safe(2:4)));
% 
%             obj.result.input = [obj.T_val; tau_safe];
%             result = obj.result;
%         end
%     end
% end

% classdef HLC_SUSPENDED_LOAD_ELLIPSOID_CBF < HLC_SUSPENDED_LOAD
%     % =========================================================================
%     % 動的超平面 4階高次制御バリア関数 (HOCBF-QP)
%     % 【元ゲイン設定 & XYZ全方位3次元外積サーキュレーション完全版】
%     % =========================================================================
%     properties
%         T_val
%         dT_val
%         initialized
%     end
% 
%     methods
%         function obj = HLC_SUSPENDED_LOAD_ELLIPSOID_CBF(self, param)
%             obj@HLC_SUSPENDED_LOAD(self, param);
%             if ~isfield(obj.result, 'min_clearance')
%                 obj.result.min_clearance = Inf;
%             end
%             obj.T_val = 0;
%             obj.dT_val = 0;
%             obj.initialized = false;
%         end
% 
%         function result = do(obj, varargin)
%             res_nom = do@HLC_SUSPENDED_LOAD(obj, varargin{:});
%             u_nom = res_nom.tmp; 
% 
%             T_nom = u_nom(1);
%             tau_nom = u_nom(2:4);
% 
%             model = obj.self.estimator.result;
%             P_vec = obj.self.parameter.get(["mass", "jx", "jy", "jz", "gravity", "loadmass", "cableL"]);
%             dt = obj.param.dt;
% 
%             m_drone = P_vec(1);
%             m_load  = P_vec(6);
%             M_total = m_drone + m_load;
%             g_acc   = P_vec(5);
%             T_hover = M_total * g_acc; 
%             J_mat   = diag([P_vec(2); P_vec(3); P_vec(4)]);
% 
%             if ~obj.initialized || isnan(obj.T_val)
%                 obj.T_val = T_nom;
%                 obj.dT_val = 0;
%                 obj.initialized = true;
%             end
% 
%             p_drone = model.state.p;
%             p_load  = model.state.pL;
%             v_load  = model.state.vL;
%             pT      = model.state.pT;
%             wL      = model.state.wL;
%             R_curr  = model.state.getq("rotmat");
%             q_quat  = R2q(R_curr);
%             w_vec   = model.state.w;
% 
%             z_state = [p_load(:); v_load(:); pT(:); wL(:); q_quat(:); w_vec(:)];
%             T_state = [obj.T_val; obj.dT_val];
% 
%             % 元の推力追従PDゲイン
%             Kp_T = 400.0; 
%             Kd_T = 40.0;
%             ddT_nom = -Kp_T * (obj.T_val - T_nom) - Kd_T * obj.dT_val;
%             mu_nom = [ddT_nom; tau_nom(:)];
% 
%             A_obs = zeros(0, 5);
%             b_obs = zeros(0, 1);
%             tau_circ = zeros(3, 1);
% 
%             min_h_this_step = Inf;
%             min_d_surf_this_step = Inf;
%             drone_params = [m_drone; m_load; P_vec(7); P_vec(2); P_vec(3); P_vec(4); g_acc]; 
% 
%             % 機体と荷物の直接中点
%             p_mid = 0.5 * (p_drone + p_load);
% 
%             % 機体天面・プロペラ・荷物底面を包含する長軸マージン
%             a_rad = 0.35;                      % 短軸半径 [m]
%             b_rad = (P_vec(7) / 2.0) + 0.35;   % 長軸半径 [m]
%             ellipsoid_params = [a_rad; b_rad];
% 
%             obs_list = ENVIRONMENT_OBSTACLE_ELLIPSOID();
% 
%             for i = 1:length(obs_list)
%                 p_obs = obs_list(i).p_obs;
%                 R_obs = obs_list(i).R_obs;
%                 Q_obs = obs_list(i).Q_obs;
%                 d_margin = obs_list(i).d_margin;
% 
%                 Q2_obs = R_obs * (Q_obs^2) * R_obs';
% 
%                 diff_center = p_mid - p_obs;
%                 if norm(diff_center) > 1e-4
%                     n_vec = diff_center / norm(diff_center);
%                 else
%                     n_vec = [1; 0; 0];
%                 end
% 
%                 % 真下・真上アプローチ時の特異点破壊バイアス
%                 if abs(n_vec(3)) > 0.90
%                     n_vec = n_vec + [0.15; 0.05; 0];
%                     n_vec = n_vec / norm(n_vec);
%                 end
% 
%                 % サポート関数差分法（8回反復による最短法線探索）
%                 for k = 1:8
%                     r_sys_ext = sqrt(a_rad^2 + (b_rad^2 - a_rad^2) * (n_vec' * pT)^2);
%                     r_obs_ext = sqrt(n_vec' * Q2_obs * n_vec);
% 
%                     grad_r_sys = ((b_rad^2 - a_rad^2) * (n_vec' * pT) / max(1e-6, r_sys_ext)) * pT;
%                     grad_r_obs = (Q2_obs * n_vec) / max(1e-6, r_obs_ext);
% 
%                     grad_n = diff_center - grad_r_sys - grad_r_obs;
%                     n_vec = n_vec + 0.15 * grad_n;
%                     n_vec = n_vec / norm(n_vec);
%                 end
% 
%                 r_sys_ext = sqrt(a_rad^2 + (b_rad^2 - a_rad^2) * (n_vec' * pT)^2);
%                 r_obs_ext = sqrt(n_vec' * Q2_obs * n_vec);
% 
%                 % 物理表面間最短ユークリッド距離
%                 dist_surface = n_vec' * diff_center - r_obs_ext - r_sys_ext;
%                 if dist_surface < min_d_surf_this_step
%                     min_d_surf_this_step = dist_surface;
%                 end
% 
%                 % 動的分離超平面パラメータ (CBF用)
%                 d_plane = n_vec' * p_obs + r_obs_ext + d_margin;
%                 plane_params = [n_vec; d_plane];
% 
%                 % 元のCBFゲイン設定
%                 cbf_gains = [39.0; 62.5; 37.5; 10.0]; 
% 
%                 [A_cbf, b_cbf, h_val, ~, ~, ~] = Ellipsoid_SuspendedLoad_TetherMid_CBF( ...
%                     z_state, T_state, drone_params, ellipsoid_params, plane_params, cbf_gains);
% 
%                 if h_val < min_h_this_step; min_h_this_step = h_val; end
%                 if h_val < obj.result.min_clearance; obj.result.min_clearance = h_val; end
% 
%                 % 介入判定（元の 4.0m 設定）
%                 if h_val < 4.0
%                     % ========================================================
%                     % XYZ全方位 3次元幾何サーキュレーション
%                     % ========================================================
%                     % 1. 現在の荷物速度から実進行方向単位ベクトルを取得 (Z固定廃止)
%                     if norm(v_load) > 0.1
%                         v_dir = v_load / norm(v_load);
%                     else
%                         v_dir = [0; 0; 1.0];
%                     end
% 
%                     % 2. 進行方向 v_dir と法線 n_vec の外積による接線平面の決定
%                     cross_vn = cross(v_dir, n_vec);
%                     if norm(cross_vn) > 1e-3
%                         % 障害物表面に沿って自然に迂回する外周接線ベクトル
%                         tangent_dir = cross(n_vec, cross_vn) / norm(cross_vn);
%                     else
%                         % 完全正対時の特異点回避：法線に直交する水平接線
%                         tangent_dir = [-n_vec(2); n_vec(1); 0];
%                         if norm(tangent_dir) > 1e-3
%                             tangent_dir = tangent_dir / norm(tangent_dir);
%                         else
%                             tangent_dir = [1; 0; 0];
%                         end
%                     end
% 
%                     % 3. 機体Z軸を接線ベクトル方向へ倒す目標傾斜軸（XYZ全方位モーメント）
%                     b_z = R_curr * [0; 0; 1];
%                     desired_tilt_axis = cross(b_z, tangent_dir);
% 
%                     % 元のスケール 0.3 でトルクバイアスを重畳
%                     tau_circ = tau_circ + 0.3 * desired_tilt_axis;
% 
%                     if ~any(isnan(A_cbf(:))) && ~any(isnan(b_cbf(:)))
%                         A_cbf_row = reshape(A_cbf, 1, 4);
%                         A_obs = [A_obs; [A_cbf_row, -1.0]];
%                         b_obs = [b_obs; b_cbf(:)];
%                     end
%                 end
%             end
% 
%             obj.result.h_val = min_h_this_step; 
%             obj.result.d_surf = min_d_surf_this_step;
% 
%             % 姿勢制約
%             cos_gamma_max = cos(35 * pi / 180);
%             cbf_gains_att = [25; 10];
%             [A_att, b_att, ~] = Attitude_Limit_CBF([q_quat(:); w_vec(:)], [P_vec(2); P_vec(3); P_vec(4)], cos_gamma_max, cbf_gains_att);
% 
%             if ~any(isnan(A_att(:))) && ~any(isnan(b_att(:)))
%                 n_rows = size(A_att, 1);
%                 n_cols = numel(A_att) / n_rows;
%                 A_att_mat = reshape(A_att, n_rows, n_cols);
% 
%                 if n_cols == 3
%                     A_att_5col = [zeros(n_rows, 1), A_att_mat, zeros(n_rows, 1)];
%                 elseif n_cols == 4
%                     A_att_5col = [A_att_mat, zeros(n_rows, 1)];
%                 else
%                     A_att_flat = A_att(:)';
%                     if length(A_att_flat) >= 4
%                         A_att_5col = [A_att_flat(1:4), 0.0];
%                     else
%                         A_att_5col = [0.0, A_att_flat, zeros(1, 4 - length(A_att_flat))];
%                     end
%                 end
%                 A_obs = [A_obs; A_att_5col];
%                 b_obs = [b_obs; b_att(:)];
%             end
% 
%             % ========================================================
%             % 推力 [2.0, 20.0] N と トルク [-1, 1] Nm の物理限界バウンド
%             % ========================================================
%             dt2_half = 0.5 * (dt^2);
%             T_pred_base = obj.T_val + obj.dT_val * dt;
% 
%             T_limit_min = 2.0;
%             T_limit_max = 20.0;
% 
%             ddT_lb = (T_limit_min - T_pred_base) / dt2_half;
%             ddT_ub = (T_limit_max - T_pred_base) / dt2_half;
% 
%             ddT_lb = max(-1500.0, min(1500.0, ddT_lb));
%             ddT_ub = max(-1500.0, min(1500.0, ddT_ub));
%             if ddT_lb > ddT_ub; tmp = ddT_lb; ddT_lb = ddT_ub; ddT_ub = tmp; end
% 
%             lb = [ddT_lb; -1.0; -1.0; -1.0; 0.0];
%             ub = [ddT_ub;  1.0;  1.0;  1.0; inf];
% 
%             % 元のQPコスト重み設定
%             w_mu = 5.0;
%             w_slack = 1e6;
%             H_qp = blkdiag(w_mu * eye(4), w_slack);
% 
%             mu_target = mu_nom + [0; tau_circ];
%             f_qp = [-w_mu * mu_target; 0];
% 
%             options = optimoptions('quadprog', 'Display', 'off');
%             exitflag = -1;
% 
%             if ~isempty(A_obs)
%                 [mu_opt, ~, exitflag] = quadprog(H_qp, f_qp, A_obs, b_obs, [], [], lb, ub, [], options);
%             else
%                 exitflag = 1;
%                 mu_opt = [min(max(mu_target(1), ddT_lb), ddT_ub); ...
%                           min(max(mu_target(2:4), -1.0), 1.0); ...
%                           0.0];
%             end
% 
%             % ========================================================
%             % 【解なし（Infeasible）時の受動制動フォールバック】
%             % ========================================================
%             if exitflag == 1
%                 mu_safe = mu_opt(1:4);
%             else
%                 ddT_fallback = -100.0 * (obj.T_val - T_hover) - 20.0 * obj.dT_val;
%                 ddT_fallback = min(max(ddT_fallback, ddT_lb), ddT_ub);
% 
%                 tau_fallback = -2.5 * J_mat * w_vec;
%                 tau_fallback = min(max(tau_fallback, -1.0), 1.0);
% 
%                 mu_safe = [ddT_fallback; tau_fallback];
%             end
% 
%             % 状態更新
%             obj.dT_val = obj.dT_val + mu_safe(1) * dt;
%             obj.T_val  = obj.T_val  + obj.dT_val * dt;
% 
%             obj.T_val = max(T_limit_min, min(T_limit_max, obj.T_val));
%             tau_safe  = max(-1.0, min(1.0, mu_safe(2:4)));
% 
%             obj.result.input = [obj.T_val; tau_safe];
%             result = obj.result;
%         end
%     end
% end

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
            t_start = tic;
            
            res_nom = do@HLC_SUSPENDED_LOAD(obj, varargin{:});
            u_nom = res_nom.tmp; 
            
            T_nom = u_nom(1);
            tau_nom = u_nom(2:4);
            
            model = obj.self.estimator.result;
            P_vec = obj.self.parameter.get(["mass", "jx", "jy", "jz", "gravity", "loadmass", "cableL"]);
            dt = obj.param.dt;
            
            m_drone = P_vec(1);
            m_load  = P_vec(6);
            M_total = m_drone + m_load;
            g_acc   = P_vec(5);
            T_hover = M_total * g_acc; 
            J_mat   = diag([P_vec(2); P_vec(3); P_vec(4)]);
            
            if ~obj.initialized || isnan(obj.T_val)
                obj.T_val = T_nom;
                obj.dT_val = 0;
                obj.initialized = true;
            end
            
            p_drone = model.state.p;
            p_load  = model.state.pL;
            v_load  = model.state.vL;
            pT      = model.state.pT;
            wL      = model.state.wL;
            R_curr  = model.state.getq("rotmat");
            q_quat  = R2q(R_curr);
            w_vec   = model.state.w;
            
            z_state = [p_load(:); v_load(:); pT(:); wL(:); q_quat(:); w_vec(:)];
            T_state = [obj.T_val; obj.dT_val];
            
            % 推力追従ゲイン（急落を防ぐためマイルドに）
            Kp_T = 200.0; 
            Kd_T = 30.0;
            ddT_nom = -Kp_T * (obj.T_val - T_nom) - Kd_T * obj.dT_val;
            mu_nom = [ddT_nom; tau_nom(:)];
            
            % 決定変数: x_qp = [ddT; tau (3次元); delta (スラック)] (計5次元)
            A_ineq = zeros(0, 5);
            b_ineq = zeros(0, 1);
            tau_circ = zeros(3, 1);
            
            min_h_this_step = Inf;
            min_d_surf_this_step = Inf;
            drone_params = [m_drone; m_load; P_vec(7); P_vec(2); P_vec(3); P_vec(4); g_acc]; 
            p_mid = 0.5 * (p_drone + p_load);
            
            a_rad = 0.35;
            b_rad = (P_vec(7) / 2.0) + 0.35;
            ellipsoid_params = [a_rad; b_rad];
            
            obs_list = ENVIRONMENT_OBSTACLE_ELLIPSOID();
            
            for i = 1:length(obs_list)
                p_obs = obs_list(i).p_obs;
                R_obs = obs_list(i).R_obs;
                Q_obs = obs_list(i).Q_obs;
                d_margin = obs_list(i).d_margin;
                
                Q2_obs = R_obs * (Q_obs^2) * R_obs';
                diff_center = p_mid - p_obs;
                if norm(diff_center) > 1e-4
                    n_vec = diff_center / norm(diff_center);
                else
                    n_vec = [1; 0; 0];
                end
                
                if abs(n_vec(3)) > 0.90
                    n_vec = n_vec + [0.15; 0.05; 0];
                    n_vec = n_vec / norm(n_vec);
                end
                
                for k = 1:8
                    r_sys_ext = sqrt(a_rad^2 + (b_rad^2 - a_rad^2) * (n_vec' * pT)^2);
                    r_obs_ext = sqrt(n_vec' * Q2_obs * n_vec);
                    grad_r_sys = ((b_rad^2 - a_rad^2) * (n_vec' * pT) / max(1e-6, r_sys_ext)) * pT;
                    grad_r_obs = (Q2_obs * n_vec) / max(1e-6, r_obs_ext);
                    grad_n = diff_center - grad_r_sys - grad_r_obs;
                    n_vec = n_vec + 0.15 * grad_n;
                    n_vec = n_vec / norm(n_vec);
                end
                
                r_sys_ext = sqrt(a_rad^2 + (b_rad^2 - a_rad^2) * (n_vec' * pT)^2);
                r_obs_ext = sqrt(n_vec' * Q2_obs * n_vec);
                
                dist_surface = n_vec' * diff_center - r_obs_ext - r_sys_ext;
                if dist_surface < min_d_surf_this_step
                    min_d_surf_this_step = dist_surface;
                end
                
                cbf_gains = [39.0; 62.5; 37.5; 10.0]; 
                
                % ① 障害物コア (完全ハード)
                d_plane_core = n_vec' * p_obs + r_obs_ext;
                plane_core = [n_vec; d_plane_core];
                [A_core, b_core, ~, ~, ~, ~] = Ellipsoid_SuspendedLoad_TetherMid_CBF( ...
                    z_state, T_state, drone_params, ellipsoid_params, plane_core, cbf_gains);
                if ~any(isnan(A_core(:))) && ~any(isnan(b_core(:)))
                    A_ineq = [A_ineq; [reshape(A_core, 1, 4), 0.0]];
                    b_ineq = [b_ineq; b_core];
                end
                
                % ② マージン外殻 (セミハード)
                d_plane_margin = n_vec' * p_obs + r_obs_ext + d_margin;
                plane_margin = [n_vec; d_plane_margin];
                [A_mrg, b_mrg, h_mrg, ~, ~, ~] = Ellipsoid_SuspendedLoad_TetherMid_CBF( ...
                    z_state, T_state, drone_params, ellipsoid_params, plane_margin, cbf_gains);
                if h_mrg < min_h_this_step; min_h_this_step = h_mrg; end
                if h_mrg < obj.result.min_clearance; obj.result.min_clearance = h_mrg; end
                
                if ~any(isnan(A_mrg(:))) && ~any(isnan(b_mrg(:)))
                    A_ineq = [A_ineq; [reshape(A_mrg, 1, 4), -1.0]];
                    b_ineq = [b_ineq; b_mrg];
                end
                
                % 接線サーキュレーション
                if h_mrg < 4.0
                    if norm(v_load) > 0.1
                        v_dir = v_load / norm(v_load);
                    else
                        v_dir = [0; 0; 1.0];
                    end
                    cross_vn = cross(v_dir, n_vec);
                    if norm(cross_vn) > 1e-3
                        tangent_dir = cross(n_vec, cross_vn) / norm(cross_vn);
                    else
                        tangent_dir = [-n_vec(2); n_vec(1); 0];
                        if norm(tangent_dir) > 1e-3; tangent_dir = tangent_dir / norm(tangent_dir); else; tangent_dir = [1; 0; 0]; end
                    end
                    b_z = R_curr * [0; 0; 1];
                    tau_circ = tau_circ + 0.25 * cross(b_z, tangent_dir);
                end
            end
            
            % 機体姿勢制約（35 deg）
            cos_gamma_max = cos(35 * pi / 180);
            cbf_gains_att = [25; 10];
            [A_att, b_att, ~] = Attitude_Limit_CBF([q_quat(:); w_vec(:)], [P_vec(2); P_vec(3); P_vec(4)], cos_gamma_max, cbf_gains_att);
            if ~any(isnan(A_att(:))) && ~any(isnan(b_att(:)))
                A_att_row = reshape(A_att, 1, numel(A_att));
                if length(A_att_row) >= 4
                    A_ineq = [A_ineq; [A_att_row(1:4), 0.0]];
                else
                    A_ineq = [A_ineq; [0.0, A_att_row, zeros(1, 3 - length(A_att_row)), 0.0]];
                end
                b_ineq = [b_ineq; b_att(1)];
            end
            
            % 推力・トルク物理限界
            dt2_half = 0.5 * (dt^2);
            T_pred_base = obj.T_val + obj.dT_val * dt;
            
            % ★ 落下防止のため推力下限を 6.0N に引き上げ
            T_limit_min = 6.0;  
            T_limit_max = 20.0;
            
            ddT_lb = (T_limit_min - T_pred_base) / dt2_half;
            ddT_ub = (T_limit_max - T_pred_base) / dt2_half;
            ddT_lb = max(-1000.0, min(1000.0, ddT_lb));
            ddT_ub = max(-1000.0, min(1000.0, ddT_ub));
            if ddT_lb > ddT_ub; tmp = ddT_lb; ddT_lb = ddT_ub; ddT_ub = tmp; end
            
            lb = [ddT_lb; -1.0; -1.0; -1.0; 0.0];
            ub = [ddT_ub;  1.0;  1.0;  1.0; inf];
            
            % ★ 最も安定していた単一スラック重み (1e6)
            w_mu = 5.0;
            w_slack = 1e6;
            % H_qp = blkdiag(w_mu * eye(4), w_slack);
            w_ddT = 5;         % 推力加速度のペナルティを大幅に軽くする (推力を動きやすくする)
            w_tau = 5.0;          % トルク追従ペナルティ
            W_u   = diag([w_ddT, w_tau, w_tau, w_tau]);
            H_qp  = blkdiag(W_u, w_slack);
            
            mu_target = mu_nom + [0; tau_circ];
            f_qp = [-w_mu * mu_target; 0];
            
            options = optimoptions('quadprog', 'Display', 'off');
            exitflag = -1;
            
            if ~isempty(A_ineq)
                [mu_opt, ~, exitflag] = quadprog(H_qp, f_qp, A_ineq, b_ineq, [], [], lb, ub, [], options);
            else
                exitflag = 1;
                mu_opt = [min(max(mu_target(1), ddT_lb), ddT_ub); ...
                          min(max(mu_target(2:4), -1.0), 1.0); ...
                          0.0];
            end
            
            if exitflag == 1
                mu_safe = mu_opt(1:4);
            else
                ddT_fallback = -100.0 * (obj.T_val - T_hover) - 20.0 * obj.dT_val;
                ddT_fallback = min(max(ddT_fallback, ddT_lb), ddT_ub);
                tau_fallback = -2.5 * J_mat * w_vec;
                tau_fallback = min(max(tau_fallback, -1.0), 1.0);
                mu_safe = [ddT_fallback; tau_fallback];
            end
            
            obj.dT_val = obj.dT_val + mu_safe(1) * dt;
            obj.T_val  = obj.T_val  + obj.dT_val * dt;
            obj.T_val  = max(T_limit_min, min(T_limit_max, obj.T_val));
            tau_safe   = max(-1.0, min(1.0, mu_safe(2:4)));
            u_final    = [obj.T_val; tau_safe];
            
            % ログ格納
            obj.result.input = u_final;
            obj.result.u_nom = u_nom;
            obj.result.u_diff = u_final - u_nom;
            obj.result.controllertime = toc(t_start) * 1000.0;
            obj.result.drone_att_deg = Quat2Eul(q_quat) * (180/pi);
            pT_unit = pT / max(1e-6, norm(pT));
            obj.result.cable_tilt_deg = acos(max(-1.0, min(1.0, abs(pT_unit(3))))) * (180.0 / pi);
            obj.result.cbf_active = double(min_h_this_step < 4.0);
            obj.result.qp_status  = double(exitflag == 1);
            obj.result.h_val = min_h_this_step;
            obj.result.d_surf = min_d_surf_this_step;
            
            result = obj.result;
        end
    end
end
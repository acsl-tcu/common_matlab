% 
% 
% classdef HLC_SUSPENDED_LOAD_HOCBF_LINK_XYZ < handle
%     % クアッドコプター用階層型線形化（実入力 u1 保持型 HOCBF + 姿勢角・紐振れ角制約 統合版）
%     % 第1層: Z方向障害物回避 CBF (u1 補正)
%     % 第2層: 機体姿勢角 (Roll/Pitch) & 牽引紐振れ角 CBF (u2, u3 補正)
% properties
%     self
%     result
%     param
% end
% 
% methods
%     function obj = HLC_SUSPENDED_LOAD_HOCBF_LINK_XYZ(self, param)
%         obj.self = self;
%         obj.param = param;
%         obj.result.min_clearance = Inf;
%     end
% 
%     function result = do(obj, varargin)
%         Param = obj.param; 
%         model = obj.self.estimator.result; 
%         ref   = obj.self.reference.result; 
% 
%         if isprop(ref.state, 'xd')
%             xd = ref.state.xd; 
%         else
%             xd = ref.state.get();
%         end
% 
%         pL = model.state.pL;
%         if isprop(model.state, "pT")
%             pT = model.state.pT;
%         else
%             delta = pL - model.state.p;
%             if norm(delta) > 1e-9
%                 pT = delta / norm(delta);
%             else
%                 pT = [0; 0; -1];
%             end
%         end
% 
%         P = [obj.self.parameter.get(["mass", "jx", "jy", "jz", "gravity", "loadmass", "cableL"]), 0, 0];
%         x = [model.state.getq('compact'); model.state.w; pL; model.state.vL; pT; model.state.wL];
% 
%         yaw      = wrapToPi(model.state.q(3)); 
%         yawd     = xd(4); 
%         yawUnit  = [cos(yaw); sin(yaw); 0]; 
%         yawdUnit = [cos(yawd); sin(yawd); 0]; 
%         deltaYaw = sign(cross(yawdUnit, yawUnit)) * acos(yawdUnit' * yawUnit); 
%         xd(4)    = -deltaYaw(3) + yaw; 
%         xd = [xd; zeros(28 - size(xd, 1), 1)];
% 
%         tic_start = tic;
% 
%         %% =========================================================================
%         %% ノミナル入力の算定 (階層型線形化)
%         %% =========================================================================
%         F1 = Param.F1; F2 = Param.F2; F3 = Param.F3; F4 = Param.F4; 
%         vf = obj.Vfd_SuspendedLoadxyDst(Param.dt, x, xd', F1); 
%         vs = obj.Vs_SuspendedLoadxyDst(x, xd', vf, P, F2, F3, F4); 
%         uf = obj.Uf_SuspendedLoadxyDst(x, xd', vf, P); 
%         beta2 = obj.Beta2_SuspendedLoadxyDst(x, xd', vf, P); 
%         vs_alpha2 = obj.V2_alpha2_SuspendedLoadxyDst(x, xd', vf, vs', P); 
%         us = beta2 \ vs_alpha2; 
% 
%         tmp = [uf(1); us]; % ノミナル入力 [u1_nom; u2_nom; u3_nom; u4_nom]
%         obj.result.tmp = tmp; 
% 
%         %% =========================================================================
%         %% 【ステップ 1】 ロープ中点 p_mid クリアランス計算 ＆ フィールド初期化
%         %% =========================================================================
%         min_surf_dist = Inf;
%         obs_env = ENVIRONMENT_OBSTACLE_HOCBF_LINK_XY();
%         num_obs = length(obs_env);
%         L_cable = P(7); 
%         p_mid = pL - 0.5 * L_cable * pT;
%         rl_sys = 1.2;
% 
%         obj.result.rl    = rl_sys;
%         obj.result.p_mid = p_mid;
% 
%         log_p_obs               = cell(1, max(1, num_obs));
%         log_r_obs_margin        = cell(1, max(1, num_obs));
%         log_r_obs               = cell(1, max(1, num_obs));
%         log_d_margin            = cell(1, max(1, num_obs));
%         log_r_minimal           = cell(1, max(1, num_obs));
%         log_r_minimal_no_margin = cell(1, max(1, num_obs));
% 
%         for i = 1:num_obs
%             xo = obs_env(i).p_obs(1); yo = obs_env(i).p_obs(2); zo = obs_env(i).p_obs(3);
%             ro = obs_env(i).r_obs_margin;
%             ro_no_margin = obs_env(i).r_obs;
%             p_obs = [xo; yo; zo];
% 
%             d_surf = norm(p_mid - p_obs) - (ro + rl_sys);
%             d_surf_no_margin = norm(p_mid - p_obs) - (ro_no_margin + rl_sys);
%             if d_surf < min_surf_dist
%                 min_surf_dist = d_surf;
%             end
%             log_p_obs{i}               = p_obs;
%             log_r_obs_margin{i}        = ro;
%             log_r_obs{i}               = obs_env(i).r_obs;
%             log_d_margin{i}            = obs_env(i).d_margin;
%             log_r_minimal{i}           = d_surf;
%             log_r_minimal_no_margin{i} = d_surf_no_margin;
%         end
%         obj.result.min_clearance           = min_surf_dist;
%         obj.result.p_obs                   = log_p_obs;
%         obj.result.r_obs_margin            = log_r_obs_margin;
%         obj.result.r_obs                   = log_r_obs;
%         obj.result.d_margin                = log_d_margin;
%         obj.result.r_minimal               = log_r_minimal;
%         obj.result.r_minimal_no_margin     = log_r_minimal_no_margin;
% 
%         %% =========================================================================
%         %% 【ステップ 2】 第1層: z 方向障害物回避 QP (u1 推力の補正)
%         %% =========================================================================
%         u1_nominal = tmp(1);
%         gamma_params_z = [1; 5];
%         A_z_qp_list = [];
%         b_z_qp_list = [];
%         log_h1_z = zeros(num_obs, 1);
%         log_h2_z = zeros(num_obs, 1);
% 
%         for i = 1:num_obs
%             obs_params = [obs_env(i).p_obs; obs_env(i).r_obs_margin];
%             sys_params = rl_sys;
% 
%             [A_z_single, b_z_single, h1_val, h2_val] = CBF_Constraints_HOCBF_zlink(...
%                 obj, x, xd, obs_params, gamma_params_z, sys_params, P);
% 
%             A_z_qp_list = [A_z_qp_list; A_z_single]; %#ok<AGROW>
%             b_z_qp_list = [b_z_qp_list; b_z_single]; %#ok<AGROW>
%             log_h1_z(i) = h1_val;
%             log_h2_z(i) = h2_val;
%         end
% 
%         H_qp_z  = 1;
%         f_qp_z  = -H_qp_z * u1_nominal;
%         lb_qp_z = 0;
%         ub_qp_z = 20;
%         options_z = optimoptions('quadprog', 'Display', 'off', 'ConstraintTolerance', 1e-4);
%         [u1_safe, ~, exitflag_z] = quadprog(H_qp_z, f_qp_z, A_z_qp_list, b_z_qp_list, [], [], lb_qp_z, ub_qp_z, [], options_z);
% 
%         if exitflag_z == 1 && ~isempty(u1_safe)
%             tmp(1) = u1_safe(1);
%         else
%             tmp(1) = max(0, min(20, u1_nominal(1)));
%         end
% 
%         slack_nom_z = A_z_qp_list * u1_nominal - b_z_qp_list;
%         slack_safe_z = b_z_qp_list - A_z_qp_list * tmp(1);
%         num_violated_z = sum(slack_nom_z > 0);
% 
%         %% =========================================================================
%         %% 【ステップ 3】 第2層: 姿勢角 (Roll/Pitch) ＆ 紐振れ角 QP (u2, u3 トルクの補正)
%         %% =========================================================================
%         u_torque_nom = tmp(2:3); % [u2_nom; u3_nom]
%         U1_val       = tmp(1);   % 第1層で確定した推力
%         V4           = tmp(4);   % Yaw 入力
% 
%         % 制限値: Roll 最大 30deg, Pitch 最大 30deg, 紐最大振れ角 30deg
%         phi_max_val  = deg2rad(10);
%         th_max_val   = deg2rad(10);
%         cos_cb_max   = cos(deg2rad(10));
%         limit_params = [phi_max_val; th_max_val; cos_cb_max];
% 
%         % クラスK関数ゲイン [Roll(2); Pitch(2); Cable(4)]
%         gamma_attidude_cable = [15; 15; 15; 15; 5; 5; 5; 5];
% 
%         % 🌟 姿勢角＆振れ角の各階層 CBF 制約式の計算
%         [A_xy_qp, b_xy_qp, h_layers_att_cb] = CBF_Constraints_Attitude_Cable_Layers( ...
%             obj, x, xd, U1_val, V4, limit_params, gamma_attidude_cable, P);
% 
%         % 2次元 QP の求解 (Roll, Pitch トルクの最適化)
%         H_qp_xy  = eye(2);
%         f_qp_xy  = -u_torque_nom;
%         lb_qp_xy = [-1.0; -1.0];
%         ub_qp_xy = [ 1.0;  1.0];
%         options_xy = optimoptions('quadprog', 'Display', 'off', 'ConstraintTolerance', 1e-4);
%         [u_torque_safe, ~, exitflag_xy] = quadprog(H_qp_xy, f_qp_xy, A_xy_qp, b_xy_qp, [], [], lb_qp_xy, ub_qp_xy, [], options_xy);
% 
%         if exitflag_xy == 1 && ~isempty(u_torque_safe)
%             tmp(2:3) = u_torque_safe;
%         else
%             tmp(2) = max(-1.0, min(1.0, u_torque_nom(1)));
%             tmp(3) = max(-1.0, min(1.0, u_torque_nom(2)));
%         end
% 
%         slack_nom_xy  = A_xy_qp * u_torque_nom - b_xy_qp;
%         slack_safe_xy = b_xy_qp - A_xy_qp * tmp(2:3);
%         num_violated_xy = sum(slack_nom_xy > 0);
% 
%         %% =========================================================================
%         %% 🌟 デバッグ用コンソール出力 (Z回避 ＆ 姿勢・振れ角 CBF)
%         %% =========================================================================
%         if num_violated_z > 0 || num_violated_xy > 0
%             fprintf('[HOCBF Status] Violations -> Z-Obs: %d | Att/Cable: %d\n', num_violated_z, num_violated_xy);
% 
%             % Z 軸方向の表示
%             if num_violated_z > 0
%                 [max_viol_z, idx_z] = max(slack_nom_z);
%                 fprintf('  [Z-Obs CBF] Obs idx:%d | Viol:%.3f | h1:%.4f | h2:%.4f | u1: %.2f -> %.2f\n', ...
%                     idx_z, max_viol_z, log_h1_z(idx_z), log_h2_z(idx_z), u1_nominal, tmp(1));
%             end
% 
%             % 姿勢・振れ角の表示
%             if num_violated_xy > 0
%                 labels = {'Roll Limit ', 'Pitch Limit', 'Cable Limit'};
%                 for k = 1:3
%                     if slack_nom_xy(k) > 0
%                         fprintf('  [%s] Viol:%.3f | A*u_nom:%.2f -> A*u_opt:%.2f (b=%.2f)\n', ...
%                             labels{k}, slack_nom_xy(k), A_xy_qp(k,:)*u_torque_nom, A_xy_qp(k,:)*tmp(2:3), b_xy_qp(k));
%                     end
%                 end
%                 fprintf('    ├─ Layer(Roll)  : h1=%.4f, h2=%.4f\n', h_layers_att_cb(1), h_layers_att_cb(2));
%                 fprintf('    ├─ Layer(Pitch) : h1=%.4f, h2=%.4f\n', h_layers_att_cb(3), h_layers_att_cb(4));
%                 fprintf('    ├─ Layer(Cable) : h1=%.4f, h2=%.4f, h3=%.4f, h4=%.4f\n', ...
%                     h_layers_att_cb(5), h_layers_att_cb(6), h_layers_att_cb(7), h_layers_att_cb(8));
%                 fprintf('    └─ Torque Output: u2:[%.3f -> %.3f], u3:[%.3f -> %.3f]\n', ...
%                     u_torque_nom(1), tmp(2), u_torque_nom(2), tmp(3));
%             end
%         end
% 
%         %% =========================================================================
%         %% 🌟 結果の完全ロギング・保存
%         %% =========================================================================
%         % Z方向回避データ
%         obj.result.A_z_qp_list     = A_z_qp_list;
%         obj.result.b_z_qp_list     = b_z_qp_list;
%         obj.result.slack_check     = slack_nom_z;
%         obj.result.num_violated    = num_violated_z;
%         obj.result.log_h1          = log_h1_z;
%         obj.result.log_h2          = log_h2_z;
%         obj.result.slack_nom       = slack_nom_z;
%         obj.result.slack_safe      = slack_safe_z;
% 
%         % 姿勢角＆振れ角 CBF データ (新規追加)
%         obj.result.A_xy_qp         = A_xy_qp;
%         obj.result.b_xy_qp         = b_xy_qp;
%         obj.result.h_layers_att_cb = h_layers_att_cb;
%         obj.result.slack_nom_xy    = slack_nom_xy;
%         obj.result.slack_safe_xy   = slack_safe_xy;
%         obj.result.num_violated_xy = num_violated_xy;
% 
%         % 入力・位置・クリアランスデータ
%         obj.result.tmp_fix         = tmp; 
%         obj.result.controllertime  = toc(tic_start);
% 
%         % 重みづけのデータ
%         obj.result.gamma_params_z         = gamma_params_z;
%         obj.result.H_qp_z         = H_qp_z;
%         obj.result.f_qp_z         = f_qp_z;
%         obj.result.lb_qp_z         = lb_qp_z;
%         obj.result.ub_qp_z         = ub_qp_z;
%         obj.result.limit_params         = limit_params;
%         obj.result.H_qp_xy         = H_qp_xy;
%         obj.result.f_qp_xy         = f_qp_xy;
%         obj.result.lb_qp_xy        = lb_qp_xy;
%         obj.result.ub_qp_xy         = ub_qp_xy;
%         obj.result.gamma_attidude_cable         = gamma_attidude_cable;
% 
%         % 最終制御出力
%         obj.result.input = [max(0.0, min(20.0, tmp(1))); ... % u1 (Thrust)
%                             max(-1.0, min(1.0,  tmp(2))); ... % u2 (Roll)
%                             max(-1.0, min(1.0,  tmp(3))); ... % u3 (Pitch)
%                             max(-1.0, min(1.0,  tmp(4)))];   % u4 (Yaw)
%         obj.result.xd    = xd;
%         obj.result.x     = x;
%         result           = obj.result;
%     end
% 
%     function show(obj)
%         obj.result
%     end
% end
% end

% classdef HLC_SUSPENDED_LOAD_HOCBF_LINK_XYZ < handle
%     % スリング負荷付きクアッドコプター用 Non-cascaded ECBF 統合コントローラ
%     % 4入力 u = [u1; u2; u3; u4] を 1 つの QP で同時最適化
%     % 制約: 3次元球状障害物回避 (中点 p_mid) + 機体姿勢角 (Roll/Pitch) + 紐振れ角
% properties
%     self
%     result
%     param
% end
% methods
%     function obj = HLC_SUSPENDED_LOAD_HOCBF_LINK_XYZ(self, param)
%         obj.self = self;
%         obj.param = param;
%         obj.result.min_clearance = Inf;
%     end
% 
%     function result = do(obj, varargin)
%         Param = obj.param; 
%         model = obj.self.estimator.result; 
%         ref   = obj.self.reference.result; 
%         if isprop(ref.state, 'xd')
%             xd = ref.state.xd; 
%         else
%             xd = ref.state.get();
%         end
%         pL = model.state.pL;
%         if isprop(model.state, "pT")
%             pT = model.state.pT;
%         else
%             delta = pL - model.state.p;
%             if norm(delta) > 1e-9
%                 pT = delta / norm(delta);
%             else
%                 pT = [0; 0; -1];
%             end
%         end
%         P = [obj.self.parameter.get(["mass", "jx", "jy", "jz", "gravity", "loadmass", "cableL"]), 0, 0];
%         x = [model.state.getq('compact'); model.state.w; pL; model.state.vL; pT; model.state.wL];
%         yaw      = wrapToPi(model.state.q(3)); 
%         yawd     = xd(4); 
%         yawUnit  = [cos(yaw); sin(yaw); 0]; 
%         yawdUnit = [cos(yawd); sin(yawd); 0]; 
%         deltaYaw = sign(cross(yawdUnit, yawUnit)) * acos(yawdUnit' * yawUnit); 
%         xd(4)    = -deltaYaw(3) + yaw; 
%         xd = [xd; zeros(28 - size(xd, 1), 1)];
%         tic_start = tic;
% 
%         %% =========================================================================
%         %% 1. ノミナル制御入力の算定 (階層型線形化)
%         %% =========================================================================
%         F1 = Param.F1; F2 = Param.F2; F3 = Param.F3; F4 = Param.F4; 
%         vf = obj.Vfd_SuspendedLoadxyDst(Param.dt, x, xd', F1); 
%         vs = obj.Vs_SuspendedLoadxyDst(x, xd', vf, P, F2, F3, F4); 
%         uf = obj.Uf_SuspendedLoadxyDst(x, xd', vf, P); 
%         beta2 = obj.Beta2_SuspendedLoadxyDst(x, xd', vf, P); 
%         vs_alpha2 = obj.V2_alpha2_SuspendedLoadxyDst(x, xd', vf, vs', P); 
%         us = beta2 \ vs_alpha2; 
%         u_nominal = [uf(1); us]; % ノミナル入力 [u1_nom; u2_nom; u3_nom; u4_nom]
%         obj.result.tmp = u_nominal; 
% 
%         %% =========================================================================
%         %% 2. 幾何情報・クリアランス計算
%         %% =========================================================================
%         min_surf_dist = Inf;
%         obs_env = ENVIRONMENT_OBSTACLE_HOCBF_LINK_XY();
%         num_obs = length(obs_env);
%         L_cable = P(7); 
%         p_mid = pL - 0.5 * L_cable * pT;
%         rl_sys = 1.2;
%         obj.result.rl    = rl_sys;
%         obj.result.p_mid = p_mid;
% 
%         log_p_obs        = cell(1, max(1, num_obs));
%         log_r_obs_margin = cell(1, max(1, num_obs));
%         log_r_minimal    = cell(1, max(1, num_obs));
% 
%         for i = 1:num_obs
%             p_obs = obs_env(i).p_obs(:);
%             ro = obs_env(i).r_obs_margin;
%             d_surf = norm(p_mid - p_obs) - (ro + rl_sys);
%             if d_surf < min_surf_dist
%                 min_surf_dist = d_surf;
%             end
%             log_p_obs{i}        = p_obs;
%             log_r_obs_margin{i} = ro;
%             log_r_minimal{i}    = d_surf;
%         end
%         obj.result.min_clearance = min_surf_dist;
%         obj.result.p_obs         = log_p_obs;
%         obj.result.r_minimal     = log_r_minimal;
% 
%         %% =========================================================================
%         %% 3. Non-cascaded CBF 制約行列の構築 (全入力 u に対して A*u <= b)
%         %% =========================================================================
%         % 動作点としての基準推力 (ノミナル推力または前ステップ値)
%         U1_ref = max(0.1, u_nominal(1)); 
% 
%         % (A) 障害物回避 CBF (各障害物に対して 1x4 の行制約)
%         gamma_obs = [2; 4; 8; 16]; % [gamma1; gamma2; gamma3; gamma4]
%         A_obs_qp = [];
%         b_obs_qp = [];
%         log_h_obs = zeros(num_obs, 4);
% 
%         for i = 1:num_obs
%             obs_params = [obs_env(i).p_obs(:); obs_env(i).r_obs_margin];
%             sys_params = rl_sys;
%             [A_single, b_single, h1_v, h2_v, h3_v, h4_v] = CBF_Constraints_NonCascaded_Obstacle(...
%                 obj, x, xd, U1_ref, obs_params, gamma_obs, sys_params, P);
% 
%             A_obs_qp = [A_obs_qp; A_single]; %#ok<AGROW> % (num_obs x 4)
%             b_obs_qp = [b_obs_qp; b_single]; %#ok<AGROW>
%             log_h_obs(i, :) = [h1_v, h2_v, h3_v, h4_v];
%         end
% 
%         % (B) 姿勢角 ＆ 紐振れ角 CBF (3x4 の制約行列として統合)
%         phi_max_val  = deg2rad(15);
%         th_max_val   = deg2rad(15);
%         cos_cb_max   = cos(deg2rad(15));
%         limit_params = [phi_max_val; th_max_val; cos_cb_max];
%         gamma_att_cb = [15; 15; 15; 15; 5; 5; 5; 5];
% 
%         [A_att_cb_23, b_att_cb, h_layers_att_cb] = CBF_Constraints_Attitude_Cable_Layers(...
%             obj, x, xd, U1_ref, u_nominal(4), limit_params, gamma_att_cb, P);
%         % A_att_cb_23 は [3 x 2] (u2, u3 に作用) -> 4次元 [u1, u2, u3, u4] に拡張
%         A_att_cb_qp = [zeros(3, 1), A_att_cb_23, zeros(3, 1)];
% 
%         % 全制約行列の結合 (A_all * u <= b_all)
%         A_qp_all = [A_obs_qp; A_att_cb_qp];
%         b_qp_all = [b_obs_qp; b_att_cb];
% 
%         %% =========================================================================
%         %% 4. 4入力統合 QP の求解 (min ||u - u_nominal||_W^2)
%         %% =========================================================================
%         % 入力重み行列 (推力と各軸トルクの調整感度)
%         W_u = diag([1.0, 10.0, 10.0, 1.0]); 
%         H_qp = W_u;
%         f_qp = -W_u * u_nominal;
% 
%         % 入力上下限 [u1; u2; u3; u4]
%         lb_qp = [ 0.0; -1.0; -1.0; -0.5];
%         ub_qp = [25.0;  1.0;  1.0;  0.5];
% 
%         options_qp = optimoptions('quadprog', 'Display', 'off', 'ConstraintTolerance', 1e-4);
%         [u_safe, ~, exitflag] = quadprog(H_qp, f_qp, A_qp_all, b_qp_all, [], [], lb_qp, ub_qp, [], options_qp);
% 
%         if exitflag == 1 && ~isempty(u_safe)
%             u_final = u_safe;
%         else
%             % ソルバー失敗時のフォールバック (クリップ処理)
%             u_final = max(lb_qp, min(ub_qp, u_nominal));
%         end
% 
%         slack_nom  = A_qp_all * u_nominal - b_qp_all;
%         slack_safe = b_qp_all - A_qp_all * u_final;
%         num_violated = sum(slack_nom > 0);
% 
%         %% =========================================================================
%         %% 5. デバッグ表示 ＆ 結果ロギング
%         %% =========================================================================
%         if num_violated > 0
%             fprintf('[Non-cascaded CBF] Violations: %d | u_nom -> u_opt: [%.2f, %.2f, %.2f, %.2f] -> [%.2f, %.2f, %.2f, %.2f]\n', ...
%                 num_violated, u_nominal(1), u_nominal(2), u_nominal(3), u_nominal(4), ...
%                               u_final(1), u_final(2), u_final(3), u_final(4));
%             [max_viol, idx_v] = max(slack_nom);
%             fprintf('  Max Violation Constraint idx:%d (Viol: %.3f)\n', idx_v, max_viol);
%         end
% 
%         obj.result.A_qp_all        = A_qp_all;
%         obj.result.b_qp_all        = b_qp_all;
%         obj.result.slack_nom       = slack_nom;
%         obj.result.slack_safe      = slack_safe;
%         obj.result.num_violated    = num_violated;
%         obj.result.log_h_obs       = log_h_obs;
%         obj.result.h_layers_att_cb = h_layers_att_cb;
%         obj.result.u_nominal       = u_nominal;
%         obj.result.tmp_fix         = u_final;
%         obj.result.controllertime  = toc(tic_start);
% 
%         % 最終出力
%         obj.result.input = u_final;
%         obj.result.xd    = xd;
%         obj.result.x     = x;
%         result           = obj.result;
%     end
% 
%     function show(obj)
%         obj.result
%     end
% end
% end


classdef HLC_SUSPENDED_LOAD_HOCBF_LINK_XYZ < handle
    % スリング負荷付きクアッドコプター用 Non-cascaded ECBF 単一 QP コントローラ
    % 最適化対象: u = [u1; u2; u3; u4] (推力 + 3軸トルク)
    % 制約: 障害物回避 CBF + 入力上下限 (lb <= u <= ub)
properties
    self
    result
    param
end
methods
    function obj = HLC_SUSPENDED_LOAD_HOCBF_LINK_XYZ(self, param)
        obj.self = self;
        obj.param = param;
        obj.result.min_clearance = Inf;
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
        P = [obj.self.parameter.get(["mass", "jx", "jy", "jz", "gravity", "loadmass", "cableL"]), 0, 0];
        x = [model.state.getq('compact'); model.state.w; pL; model.state.vL; pT; model.state.wL];
        yaw      = wrapToPi(model.state.q(3)); 
        yawd     = xd(4); 
        yawUnit  = [cos(yaw); sin(yaw); 0]; 
        yawdUnit = [cos(yawd); sin(yawd); 0]; 
        deltaYaw = sign(cross(yawdUnit, yawUnit)) * acos(yawdUnit' * yawUnit); 
        xd(4)    = -deltaYaw(3) + yaw; 
        xd = [xd; zeros(28 - size(xd, 1), 1)];
        tic_start = tic;

        %% =========================================================================
        %% 1. ノミナル制御入力の算定 (階層型線形化)
        %% =========================================================================
        F1 = Param.F1; F2 = Param.F2; F3 = Param.F3; F4 = Param.F4; 
        vf = obj.Vfd_SuspendedLoadxyDst(Param.dt, x, xd', F1); 
        vs = obj.Vs_SuspendedLoadxyDst(x, xd', vf, P, F2, F3, F4); 
        uf = obj.Uf_SuspendedLoadxyDst(x, xd', vf, P); 
        beta2 = obj.Beta2_SuspendedLoadxyDst(x, xd', vf, P); 
        vs_alpha2 = obj.V2_alpha2_SuspendedLoadxyDst(x, xd', vf, vs', P); 
        us = beta2 \ vs_alpha2; 
        u_nominal = [uf(1); us]; % ノミナル入力 [u1_nom; u2_nom; u3_nom; u4_nom]
        obj.result.tmp = u_nominal; 

        %% =========================================================================
        %% 2. 幾何情報・クリアランス計算
        %% =========================================================================
        min_surf_dist = Inf;
        obs_env = ENVIRONMENT_OBSTACLE_HOCBF_LINK_XY();
        num_obs = length(obs_env);
        L_cable = P(7); 
        p_mid = pL - 0.5 * L_cable * pT;
        rl_sys = 1.2;
        obj.result.rl    = rl_sys;
        obj.result.p_mid = p_mid;
        
        log_p_obs        = cell(1, max(1, num_obs));
        log_r_obs_margin = cell(1, max(1, num_obs));
        log_r_minimal    = cell(1, max(1, num_obs));

        for i = 1:num_obs
            p_obs = obs_env(i).p_obs(:);
            ro = obs_env(i).r_obs_margin;
            d_surf = norm(p_mid - p_obs) - (ro + rl_sys);
            if d_surf < min_surf_dist
                min_surf_dist = d_surf;
            end
            log_p_obs{i}        = p_obs;
            log_r_obs_margin{i} = ro;
            log_r_minimal{i}    = d_surf;
        end
        obj.result.min_clearance = min_surf_dist;
        obj.result.p_obs         = log_p_obs;
        obj.result.r_minimal     = log_r_minimal;

        %% =========================================================================
        %% 3. 障害物回避 Non-cascaded CBF 制約行列の構築
        %% =========================================================================
        % トルク効果を保持するための動作点推力 U1_ref
        U1_ref = max(0.1, u_nominal(1)); 

        % 各階層ゲイン (必要に応じて調整)
        gamma_obs = [2; 4; 8; 16]; % [gamma1; gamma2; gamma3; gamma4]
        A_qp_all = [];
        b_qp_all = [];
        log_h_obs = zeros(num_obs, 4);

        for i = 1:num_obs
            obs_params = [obs_env(i).p_obs(:); obs_env(i).r_obs_margin];
            sys_params = rl_sys;
            [A_single, b_single, h1_v, h2_v, h3_v, h4_v] = CBF_Constraints_NonCascaded_Obstacle(...
                obj, x, xd, U1_ref, obs_params, gamma_obs, sys_params, P);
            
            A_qp_all = [A_qp_all; A_single]; %#ok<AGROW> % (num_obs x 4)
            b_qp_all = [b_qp_all; b_single]; %#ok<AGROW>
            log_h_obs(i, :) = [h1_v, h2_v, h3_v, h4_v];
        end

        %% =========================================================================
        %% 4. 単一 4次元 QP の求解 (min ||u - u_nominal||_W^2)
        %% =========================================================================
        % 入力重み行列 (推力とトルクのスケール差を調整)
        W_u = diag([1.0, 0.1, 0.1, 1.0]); 
        H_qp = W_u;
        f_qp = -W_u * u_nominal;

        % 入力上下限 [u1 (Thrust); u2 (Roll); u3 (Pitch); u4 (Yaw)]
        lb_qp = [ 0.0; -1.0; -1.0; -0.5];
        ub_qp = [20.0;  1.0;  1.0;  0.5];

        options_qp = optimoptions('quadprog', 'Display', 'off', 'ConstraintTolerance', 1e-4);
        [u_safe, ~, exitflag] = quadprog(H_qp, f_qp, A_qp_all, b_qp_all, [], [], lb_qp, ub_qp, [], options_qp);

        if exitflag == 1 && ~isempty(u_safe)
            u_final = u_safe;
        else
            % ソルバー失敗時のクリップ処理
            u_final = max(lb_qp, min(ub_qp, u_nominal));
        end

        slack_nom  = A_qp_all * u_nominal - b_qp_all;
        slack_safe = b_qp_all - A_qp_all * u_final;
        num_violated = sum(slack_nom > 0);

        %% =========================================================================
        %% 5. デバッグ表示 ＆ ロギング
        %% =========================================================================
        if num_violated > 0
            fprintf('[Non-cascaded CBF] 介入発生 (違反障害物数: %d)\n', num_violated);
            fprintf('  Nominal: [u1=%.2f, u2=%.3f, u3=%.3f, u4=%.3f]\n', ...
                u_nominal(1), u_nominal(2), u_nominal(3), u_nominal(4));
            fprintf('  Safe   : [u1=%.2f, u2=%.3f, u3=%.3f, u4=%.3f]\n', ...
                u_final(1), u_final(2), u_final(3), u_final(4));
            [max_viol, idx_v] = max(slack_nom);
            fprintf('  Max Viol (Obs #%d): %.3f | A*u_nom=%.2f, A*u_safe=%.2f (b=%.2f)\n', ...
                idx_v, max_viol, A_qp_all(idx_v,:)*u_nominal, A_qp_all(idx_v,:)*u_final, b_qp_all(idx_v));
        end

        obj.result.A_qp_all        = A_qp_all;
        obj.result.b_qp_all        = b_qp_all;
        obj.result.slack_nom       = slack_nom;
        obj.result.slack_safe      = slack_safe;
        obj.result.num_violated    = num_violated;
        obj.result.log_h_obs       = log_h_obs;
        obj.result.u_nominal       = u_nominal;
        obj.result.tmp_fix         = u_final;
        obj.result.controllertime  = toc(tic_start);

        % 最終制御出力
        obj.result.input = u_final;
        obj.result.xd    = xd;
        obj.result.x     = x;
        result           = obj.result;
    end

    function show(obj)
        obj.result
    end
end
end
% classdef HLC_SUSPENDED_LOAD_HOCBF_LINK_Z < handle
%     % クアッドコプター用階層型線形化（実入力 u1 保持型 HOCBF 単一球体検証版）
%     % 条件：障害物＝球体(円)、ロープ中点 p_mid 基準（単一球体）
% properties
%     self
%     result
%     param
% end
% 
% methods
%     function obj = HLC_SUSPENDED_LOAD_HOCBF_LINK_Z(self, param)
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
%         % 仮想・ノミナル入力の算定
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
%         %% 【ステップ 1】 ロープ中点 p_mid (単一球体) クリアランス計算 ＆ フィールド初期化
%         %% =========================================================================
%         min_surf_dist = Inf;
%         obs_env = ENVIRONMENT_OBSTACLE_HOCBF_LINK_XY(); % これは共通化　XY次回からは共通化のために外す
%         num_obs = length(obs_env);
% 
%         pL = model.state.pL;  
%         L_cable = P(7); 
%         p_mid = pL - 0.5 * L_cable * pT; % ロープ（リンク）の中点
% 
%         % システム側単一球体の半径 (rl)
%         rl_sys = 1.2;
%         obj.result.rl = rl_sys;
% 
%         % アニメーション描画用フィールドのセット
%         obj.result.p_mid = p_mid;
% 
%         log_p_obs     = cell(1, max(1, num_obs));
%         log_r_obs_margin     = cell(1, max(1, num_obs));
%         log_r_obs     = cell(1, max(1, num_obs));
%         log_d_margin     = cell(1, max(1, num_obs));
%         log_r_minimal = cell(1, max(1, num_obs));
%         log_r_minimal_no_margin = cell(1, max(1, num_obs));
% 
%         for i = 1:num_obs
%             xo = obs_env(i).p_obs(1); yo = obs_env(i).p_obs(2); zo = obs_env(i).p_obs(3);
%             ro = obs_env(i).r_obs_margin;
%             ro_no_margin = obs_env(i).r_obs;
%             p_obs = [xo; yo; zo];
% 
%             % 単一球体 (p_mid) からマージンを含めた障害物表面までの距離
%             d_surf = norm(p_mid - p_obs) - (ro + rl_sys);
%             d_surf_no_margin = norm(p_mid - p_obs) - (ro_no_margin + rl_sys);
% 
%             if d_surf < min_surf_dist
%                 min_surf_dist = d_surf;
%             end
% 
%             log_p_obs{i}     = p_obs;
%             log_r_obs_margin{i}     = ro;
%             log_r_obs{i}     = obs_env(i).r_obs;
%             log_d_margin{i}     = obs_env(i).d_margin;
%             log_r_minimal{i} = d_surf;
%             log_r_minimal_no_margin{i} = d_surf_no_margin;
%         end
% 
%         obj.result.min_clearance = min_surf_dist;
%         obj.result.p_obs         = log_p_obs;
%         obj.result.r_obs         = log_r_obs_margin;
%         obj.result.r_minimal     = log_r_minimal;
%         obj.result.r_minimal_no_margin     = log_r_minimal_no_margin;
% 
%         %% =========================================================================
%         %% 【ステップ 2】  CBF_Constraints_HOCBF_zlink による xy 方向 QP 安全補正
%         %% =========================================================================
%         u1_nominal = tmp(1); % u1_nom
% 
%         gamma_params_z = [1; 5];
% 
%         A_z_qp_list = [];
%         b_z_qp_list = [];
% 
%         % 🌟 階層ログ配列の事前割り当て（事前初期化）
%         log_h1 = zeros(num_obs, 1);
%         log_h2 = zeros(num_obs, 1);
% 
%         % 全障害物に対する CBF 制約の積載 (単一球体モデル)
%         for i = 1:num_obs
%             obs_params = [obs_env(i).p_obs; obs_env(i).r_obs_margin]; % [xo; yo; zo; ro]
%             sys_params = rl_sys;                                 % rl
% 
%             % 🌟 CBF_Constraints_HOCBF_zlink.m から A, b と各階層 h1~h2 を取得
%             [A_z_single, b_z_single, h1_val, h2_val] = CBF_Constraints_HOCBF_zlink(...
%                 obj, x, xd, obs_params, gamma_params_z, sys_params, P);
% 
%             A_z_qp_list = [A_z_qp_list; A_z_single]; %#ok<AGROW>
%             b_z_qp_list = [b_z_qp_list; b_z_single]; %#ok<AGROW>
% 
%             % 各階層の数値を保存
%             log_h1(i) = h1_val;
%             log_h2(i) = h2_val;
%         end
% 
%         % 🌟 2次元 QP (quadprog) の実行
%         H_qp  = 0.8;
%         f_qp  = -H_qp * u1_nominal;
%         lb_qp = 0;
%         ub_qp = 20;
%         options = optimoptions('quadprog', 'Display', 'off', 'ConstraintTolerance', 1e-4);
%         [u1_safe, ~, exitflag] = quadprog(H_qp, f_qp, A_z_qp_list, b_z_qp_list, [], [], lb_qp, ub_qp, [], options);
% 
%         % 安全ガード付き更新処理
%         if exitflag == 1 && ~isempty(u1_safe)
%             tmp(1) = u1_safe(1); % u1
%         else
%             % 解なし時等のフォールバック (ノミナル値をクリッピング)
%             tmp(1) = max(0, min(20, u1_nominal(1)));
%         end
% 
%         % 🌟 スラック（制約余裕度）の計算
%         slack_nom  = A_z_qp_list * u1_nominal - b_z_qp_list; % ノミナル入力での違反量 (>0 で違反)
%         if exitflag == 1 && ~isempty(u1_safe)
%             slack_safe = b_z_qp_list - A_z_qp_list * u1_safe; % 安全入力適用後の余裕 (>=0 で安全)
%         else
%             slack_safe = -slack_nom;
%         end
% 
%         % 🌟 デバッグ用コンソール出力 (最悪条件の障害物と各階層の数値を表示)
%         num_violated = sum(slack_nom > 0);
%         if num_violated > 0
%             [max_viol, idx] = max(slack_nom);
%             fprintf('[HOCBF 違反検知] 違反数:%d/%d | 最悪障害物 Index:%d | 最大違反量:%.3f\n', ...
%                 num_violated, length(b_z_qp_list), idx, max_viol);
%             fprintf('  ├─ h1 (距離余裕度 h)       : %.4f (正なら安全領域内)\n', log_h1(idx));
%             fprintf('  ├─ h2 (速度・1次応答 h2)    : %.4f (これが負に近づくとQP介入)\n', log_h2(idx));
%             fprintf('  入力補正: u1_nom:[%.2f, %.2f] -> u1_out:[%.2f, %.2f] (exitflag:%d)\n', ...
%                 u1_nominal(1), tmp(1), exitflag);
%         end
% 
%         % 🌟 結果の格納（元のフィールド名もすべて維持）
%         obj.result.A_z_qp_list   = A_z_qp_list;
%         obj.result.b_z_qp_list   = b_z_qp_list;
%         obj.result.slack_check    = slack_nom; % slack_check も維持
%         obj.result.num_violated   = num_violated;
%         obj.result.tmp_fix        = tmp; 
%         obj.result.p_mid          = p_mid;
%         obj.result.min_clearance  = min_surf_dist;
%         obj.result.p_obs          = log_p_obs;
%         obj.result.r_obs_margin          = log_r_obs_margin;
%         obj.result.r_obs          = log_r_obs;
%         obj.result.d_margin          = log_d_margin;
%         obj.result.r_minimal      = log_r_minimal;
%         obj.result.r_minimal_no_margin      = log_r_minimal_no_margin;
%         obj.result.controllertime = toc(tic_start);
% 
%         % 🌟 階層ログデータの保存
%         obj.result.log_h1     = log_h1;
%         obj.result.log_h2     = log_h2;
%         obj.result.slack_nom  = slack_nom;
%         obj.result.slack_safe = slack_safe;
% 
%         % 最終制御入力 (物理範囲 [-1, 1] 内に収めて出力)
%         obj.result.input = [max(0.0, min(20.0, tmp(1))); ... % u1 (推力)
%                             max(-1.0, min(1.0,  tmp(2))); ... % u2 (Roll)
%                             max(-1.0, min(1.0,  tmp(3))); ... % u3 (Pitch)
%                             max(-1.0, min(1.0,  tmp(4)))];   % u4 (Yaw)
%         obj.result.xd = xd;
%         obj.result.x  = x;
%         result        = obj.result;
%     end
%     function show(obj)
%         obj.result
%     end
% end
% end


% classdef HLC_SUSPENDED_LOAD_HOCBF_LINK_Z < handle
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
%     function obj = HLC_SUSPENDED_LOAD_HOCBF_LINK_Z(self, param)
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

classdef HLC_SUSPENDED_LOAD_HOCBF_LINK_Z < handle
    % クアッドコプター用階層型線形化
    % 第1層: Z方向障害物回避 CBF (u1 補正)
    % 第2層: スラック付きソフト制約 QP による姿勢・紐振れ角制御 (Parwana et al. 方式)
    %Feasible Space Monitoring for Multiple Control Barrier Functions with application to Large Scale Indoor Navigation
properties
    self
    result
    param
end

methods
    function obj = HLC_SUSPENDED_LOAD_HOCBF_LINK_Z(self, param)
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

        %% ノミナル入力の算定 (階層型線形化)
        F1 = Param.F1; F2 = Param.F2; F3 = Param.F3; F4 = Param.F4; 
        vf = obj.Vfd_SuspendedLoadxyDst(Param.dt, x, xd', F1); 
        vs = obj.Vs_SuspendedLoadxyDst(x, xd', vf, P, F2, F3, F4); 
        uf = obj.Uf_SuspendedLoadxyDst(x, xd', vf, P); 
        beta2 = obj.Beta2_SuspendedLoadxyDst(x, xd', vf, P); 
        vs_alpha2 = obj.V2_alpha2_SuspendedLoadxyDst(x, xd', vf, vs', P); 
        us = beta2 \ vs_alpha2; 

        tmp = [uf(1); us]; 
        obj.result.tmp = tmp; 

        %% 【ステップ 1】 ロープ中点 p_mid クリアランス計算
        min_surf_dist = Inf;
        obs_env = ENVIRONMENT_OBSTACLE_HOCBF_LINK_XY();
        num_obs = length(obs_env);
        L_cable = P(7); 
        p_mid = pL - 0.5 * L_cable * pT;
        rl_sys = 1.2;

        obj.result.rl    = rl_sys;
        obj.result.p_mid = p_mid;

        log_p_obs               = cell(1, max(1, num_obs));
        log_r_obs_margin        = cell(1, max(1, num_obs));
        log_r_obs               = cell(1, max(1, num_obs));
        log_d_margin            = cell(1, max(1, num_obs));
        log_r_minimal           = cell(1, max(1, num_obs));
        log_r_minimal_no_margin = cell(1, max(1, num_obs));

        for i = 1:num_obs
            xo = obs_env(i).p_obs(1); yo = obs_env(i).p_obs(2); zo = obs_env(i).p_obs(3);
            ro = obs_env(i).r_obs_margin;
            ro_no_margin = obs_env(i).r_obs;
            p_obs = [xo; yo; zo];

            d_surf = norm(p_mid - p_obs) - (ro + rl_sys);
            d_surf_no_margin = norm(p_mid - p_obs) - (ro_no_margin + rl_sys);
            if d_surf < min_surf_dist
                min_surf_dist = d_surf;
            end
            log_p_obs{i}               = p_obs;
            log_r_obs_margin{i}        = ro;
            log_r_obs{i}               = obs_env(i).r_obs;
            log_d_margin{i}            = obs_env(i).d_margin;
            log_r_minimal{i}           = d_surf;
            log_r_minimal_no_margin{i} = d_surf_no_margin;
        end
        obj.result.min_clearance           = min_surf_dist;
        obj.result.p_obs                   = log_p_obs;
        obj.result.r_obs_margin            = log_r_obs_margin;
        obj.result.r_obs                   = log_r_obs;
        obj.result.d_margin                = log_d_margin;
        obj.result.r_minimal               = log_r_minimal;
        obj.result.r_minimal_no_margin     = log_r_minimal_no_margin;

        %% 【ステップ 2】 第1層: z 方向障害物回避 QP (u1 推力補正)
        u1_nominal = tmp(1);
        gamma_params_z = [1; 5];
        A_z_qp_list = [];
        b_z_qp_list = [];
        log_h1_z = zeros(num_obs, 1);
        log_h2_z = zeros(num_obs, 1);
        for i = 1:num_obs
            obs_params = [obs_env(i).p_obs; obs_env(i).r_obs_margin];
            sys_params = rl_sys;

            [A_z_single, b_z_single, h1_val, h2_val] = CBF_Constraints_HOCBF_zlink(...
                obj, x, xd, obs_params, gamma_params_z, sys_params, P);

            A_z_qp_list = [A_z_qp_list; A_z_single]; %#ok<AGROW>
            b_z_qp_list = [b_z_qp_list; b_z_single]; %#ok<AGROW>
            log_h1_z(i) = h1_val;
            log_h2_z(i) = h2_val;
        end
        H_qp_z  = 1;
        f_qp_z  = -H_qp_z * u1_nominal;
        lb_qp_z = 0;
        ub_qp_z = 20;
        options_z = optimoptions('quadprog', 'Display', 'off', 'ConstraintTolerance', 1e-4);
        [u1_safe, ~, exitflag_z] = quadprog(H_qp_z, f_qp_z, A_z_qp_list, b_z_qp_list, [], [], lb_qp_z, ub_qp_z, [], options_z);

        if exitflag_z == 1 && ~isempty(u1_safe)
            tmp(1) = u1_safe(1);
        else
            tmp(1) = max(0, min(20, u1_nominal(1)));
        end
        slack_nom_z = A_z_qp_list * u1_nominal - b_z_qp_list;
        slack_safe_z = b_z_qp_list - A_z_qp_list * tmp(1);
        num_violated_z = sum(slack_nom_z > 0);

        %% 【ステップ 3】 第2層: スラック付きソフト制約 QP (u2, u3 トルク補正)
        tic_xy = tic;
        u_torque_nom = tmp(2:3); 
        U1_val       = tmp(1);   
        V4           = tmp(4);   

        limit_deg = 10;
        limit_params = [deg2rad(limit_deg); deg2rad(limit_deg); cos(deg2rad(limit_deg))];
        gamma_attidude_cable = [15; 15; 15; 15; 5; 5; 5; 5];

        [A_xy_qp, b_xy_qp, h_layers_att_cb] = CBF_Constraints_Attitude_Cable_Layers( ...
            obj, x, xd, U1_val, V4, limit_params, gamma_attidude_cable, P);

        % 🌟 スラック変数付き QP の定式化
        n_u = 2;
        n_c = size(A_xy_qp, 1); % 3 (Roll, Pitch, Cable)
        W_slack = 1e4 * eye(n_c); 

        H_soft = blkdiag(eye(n_u), W_slack);
        f_soft = [-u_torque_nom; zeros(n_c, 1)];

        % A*u - delta <= b  ->  [A, -I] * [u; delta] <= b
        A_soft = [A_xy_qp, -eye(n_c)];
        b_soft = b_xy_qp;

        lb_soft = [-1.0; -1.0; zeros(n_c, 1)];
        ub_soft = [ 1.0;  1.0; Inf * ones(n_c, 1)];

        options_xy = optimoptions('quadprog', 'Display', 'off', 'ConstraintTolerance', 1e-4);
        [sol, ~, exitflag_xy] = quadprog(H_soft, f_soft, A_soft, b_soft, [], [], lb_soft, ub_soft, [], options_xy);

        slack_delta_xy = zeros(3, 1);
        if exitflag_xy == 1 && ~isempty(sol)
            tmp(2:3) = sol(1:2);
            slack_delta_xy = sol(3:5); % 各制約の緩和量
        else
            tmp(2) = max(-1.0, min(1.0, u_torque_nom(1)));
            tmp(3) = max(-1.0, min(1.0, u_torque_nom(2)));
        end
        time_xy_calc = toc(tic_xy);

        slack_nom_xy  = A_xy_qp * u_torque_nom - b_xy_qp;
        slack_safe_xy = b_xy_qp - A_xy_qp * tmp(2:3);
        num_violated_xy = sum(slack_nom_xy > 0);

        %% 🌟 デバッグ用コンソール出力
        if num_violated_z > 0 || num_violated_xy > 0
            fprintf('[HOCBF SOFT] Violations -> Z: %d | Att/Cable: %d (Flag:%d, Time:%.2f ms)\n', ...
                num_violated_z, num_violated_xy, exitflag_xy, time_xy_calc*1000);
            if num_violated_xy > 0
                fprintf('  ├─ [Slack Deltas] Roll:%.4f, Pitch:%.4f, Cable:%.4f\n', ...
                    slack_delta_xy(1), slack_delta_xy(2), slack_delta_xy(3));
                fprintf('  └─ Torque Output: u2:[%.3f -> %.3f], u3:[%.3f -> %.3f]\n', ...
                    u_torque_nom(1), tmp(2), u_torque_nom(2), tmp(3));
            end
        end

        %% 結果の格納
        obj.result.A_z_qp_list     = A_z_qp_list;
        obj.result.b_z_qp_list     = b_z_qp_list;
        obj.result.slack_check     = slack_nom_z;
        obj.result.num_violated    = num_violated_z;
        obj.result.log_h1          = log_h1_z;
        obj.result.log_h2          = log_h2_z;
        obj.result.slack_nom       = slack_nom_z;
        obj.result.slack_safe      = slack_safe_z;

        obj.result.A_xy_qp         = A_xy_qp;
        obj.result.b_xy_qp         = b_xy_qp;
        obj.result.h_layers_att_cb = h_layers_att_cb;
        obj.result.slack_nom_xy    = slack_nom_xy;
        obj.result.slack_safe_xy   = slack_safe_xy;
        obj.result.num_violated_xy = num_violated_xy;
        obj.result.slack_delta_xy  = slack_delta_xy;

        obj.result.controllertime_xy = time_xy_calc;
        obj.result.controllertime    = toc(tic_start);
        obj.result.tmp_fix           = tmp;

        obj.result.input = [max(0.0, min(20.0, tmp(1))); ... 
                            max(-1.0, min(1.0,  tmp(2))); ... 
                            max(-1.0, min(1.0,  tmp(3))); ... 
                            max(-1.0, min(1.0,  tmp(4)))];   
        obj.result.xd    = xd;
        obj.result.x     = x;
        result           = obj.result;
    end

    function show(obj)
        obj.result
    end
end
end

% classdef HLC_SUSPENDED_LOAD_HOCBF_LINK_Z < handle
%     % クアッドコプター用階層型線形化
%     % 第1層: Z方向障害物回避 CBF (u1 補正)
%     % 第2層: Composite CBF (Softmin合成) による姿勢・紐振れ角制御 (Harms et al. 方式)
%     %Safe Quadrotor Navigation using Composite Control Barrier Functions
% properties
%     self
%     result
%     param
% end
% 
% methods
%     function obj = HLC_SUSPENDED_LOAD_HOCBF_LINK_Z(self, param)
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
%         %% ノミナル入力の算定
%         F1 = Param.F1; F2 = Param.F2; F3 = Param.F3; F4 = Param.F4; 
%         vf = obj.Vfd_SuspendedLoadxyDst(Param.dt, x, xd', F1); 
%         vs = obj.Vs_SuspendedLoadxyDst(x, xd', vf, P, F2, F3, F4); 
%         uf = obj.Uf_SuspendedLoadxyDst(x, xd', vf, P); 
%         beta2 = obj.Beta2_SuspendedLoadxyDst(x, xd', vf, P); 
%         vs_alpha2 = obj.V2_alpha2_SuspendedLoadxyDst(x, xd', vf, vs', P); 
%         us = beta2 \ vs_alpha2; 
% 
%         tmp = [uf(1); us]; 
%         obj.result.tmp = tmp; 
% 
%         %% 【ステップ 1】 ロープ中点 p_mid クリアランス計算
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
%         %% 【ステップ 2】 第1層: z 方向障害物回避 QP
%         u1_nominal = tmp(1);
%         gamma_params_z = [1; 5];
%         A_z_qp_list = [];
%         b_z_qp_list = [];
%         log_h1_z = zeros(num_obs, 1);
%         log_h2_z = zeros(num_obs, 1);
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
%         slack_nom_z = A_z_qp_list * u1_nominal - b_z_qp_list;
%         slack_safe_z = b_z_qp_list - A_z_qp_list * tmp(1);
%         num_violated_z = sum(slack_nom_z > 0);
% 
%         %% 【ステップ 3】 第2層: Composite CBF (Softmin合成) による QP
%         tic_xy = tic;
%         u_torque_nom = tmp(2:3); 
%         U1_val       = tmp(1);   
%         V4           = tmp(4);   
% 
%         limit_deg = 10;
%         limit_params = [deg2rad(limit_deg); deg2rad(limit_deg); cos(deg2rad(limit_deg))];
%         gamma_attidude_cable = [15; 15; 15; 15; 5; 5; 5; 5];
% 
%         [A_xy_qp, b_xy_qp, h_layers_att_cb] = CBF_Constraints_Attitude_Cable_Layers( ...
%             obj, x, xd, U1_val, V4, limit_params, gamma_attidude_cable, P);
% 
%         % 🌟 Composite CBF (Softmin 重み計算)
%         h_candidates = [h_layers_att_cb(1); h_layers_att_cb(3); h_layers_att_cb(5)];
%         kappa_comp = 20.0;
% 
%         shift_h = h_candidates - min(h_candidates);
%         exp_w   = exp(-kappa_comp * shift_h);
%         composite_weights = exp_w / sum(exp_w);
% 
%         % 単一制約へ合成 (1x2 行列, 1x1 スカラー)
%         A_comp = composite_weights' * A_xy_qp;
%         b_comp = composite_weights' * b_xy_qp;
% 
%         options_xy = optimoptions('quadprog', 'Display', 'off', 'ConstraintTolerance', 1e-4);
%         [u_torque_safe, ~, exitflag_xy] = quadprog(eye(2), -u_torque_nom, A_comp, b_comp, ...
%             [], [], [-1.0; -1.0], [1.0; 1.0], [], options_xy);
% 
%         if exitflag_xy == 1 && ~isempty(u_torque_safe)
%             tmp(2:3) = u_torque_safe;
%         else
%             tmp(2) = max(-1.0, min(1.0, u_torque_nom(1)));
%             tmp(3) = max(-1.0, min(1.0, u_torque_nom(2)));
%         end
%         time_xy_calc = toc(tic_xy);
% 
%         slack_nom_xy  = A_xy_qp * u_torque_nom - b_xy_qp;
%         slack_safe_xy = b_xy_qp - A_xy_qp * tmp(2:3);
%         num_violated_xy = sum(slack_nom_xy > 0);
% 
%         %% 🌟 デバッグ用コンソール出力
%         if num_violated_z > 0 || num_violated_xy > 0
%             fprintf('[HOCBF COMPOSITE] Violations -> Z: %d | Att/Cable: %d (Flag:%d, Time:%.2f ms)\n', ...
%                 num_violated_z, num_violated_xy, exitflag_xy, time_xy_calc*1000);
%             if num_violated_xy > 0
%                 fprintf('  ├─ [Composite Weights] Roll:%.2f, Pitch:%.2f, Cable:%.2f\n', ...
%                     composite_weights(1), composite_weights(2), composite_weights(3));
%                 fprintf('  └─ Torque Output: u2:[%.3f -> %.3f], u3:[%.3f -> %.3f]\n', ...
%                     u_torque_nom(1), tmp(2), u_torque_nom(2), tmp(3));
%             end
%         end
% 
%         %% 結果の格納
%         obj.result.A_z_qp_list     = A_z_qp_list;
%         obj.result.b_z_qp_list     = b_z_qp_list;
%         obj.result.slack_check     = slack_nom_z;
%         obj.result.num_violated    = num_violated_z;
%         obj.result.log_h1          = log_h1_z;
%         obj.result.log_h2          = log_h2_z;
%         obj.result.slack_nom       = slack_nom_z;
%         obj.result.slack_safe      = slack_safe_z;
% 
%         obj.result.A_xy_qp         = A_xy_qp;
%         obj.result.b_xy_qp         = b_xy_qp;
%         obj.result.h_layers_att_cb = h_layers_att_cb;
%         obj.result.slack_nom_xy    = slack_nom_xy;
%         obj.result.slack_safe_xy   = slack_safe_xy;
%         obj.result.num_violated_xy = num_violated_xy;
%         obj.result.composite_weights = composite_weights;
% 
%         obj.result.controllertime_xy = time_xy_calc;
%         obj.result.controllertime    = toc(tic_start);
%         obj.result.tmp_fix           = tmp;
% 
%         obj.result.input = [max(0.0, min(20.0, tmp(1))); ... 
%                             max(-1.0, min(1.0,  tmp(2))); ... 
%                             max(-1.0, min(1.0,  tmp(3))); ... 
%                             max(-1.0, min(1.0,  tmp(4)))];   
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
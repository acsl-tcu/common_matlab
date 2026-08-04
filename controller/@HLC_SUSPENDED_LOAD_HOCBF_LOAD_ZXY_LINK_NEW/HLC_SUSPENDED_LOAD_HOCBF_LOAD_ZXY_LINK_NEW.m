% classdef HLC_SUSPENDED_LOAD_HOCBF_LOAD_ZXY_LINK_NEW < handle
%     % クアッドコプター用階層型線形化（z解析的CBF ＋ xy実入力結合型HOCBF ＋ ケーブル角度制限付き）
%     % 条件：障害物＝球体(円)、システム＝マルチセグメント球体被覆モデル（HOCBF版）
%     % よけようとしてやるが、姿勢を考えないため、墜落
% properties
%     self
%     result
%     param
% end
% 
% methods
%     function obj = HLC_SUSPENDED_LOAD_HOCBF_LOAD_ZXY_LINK_NEW(self, param)
%         obj.self = self;
%         obj.param = param;
%         % 初期化時のログ参照エラー防止（安全ガード）
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
% 
%         xd = [xd; zeros(28 - size(xd, 1), 1)];
%         tic_start = tic;
% 
%         % 仮想入力のゲイン
%         F1 = Param.F1; 
%         F2 = Param.F2; 
%         F3 = Param.F3; 
%         F4 = Param.F4; 
%         time_log = cell(1, 6);
% 
%         tic;
%         time_log{1} = datetime('now', 'Format', 'HH:mm:ss.SSSSSS');
%         vf = obj.Vfd_SuspendedLoadxyDst(Param.dt, x, xd', F1); 
% 
%         time_log{2} = datetime('now', 'Format', 'HH:mm:ss.SSSSSS');
%         vs = obj.Vs_SuspendedLoadxyDst(x, xd', vf, P, F2, F3, F4); 
% 
%         time_log{3} = datetime('now', 'Format', 'HH:mm:ss.SSSSSS');
%         uf = obj.Uf_SuspendedLoadxyDst(x, xd', vf, P); 
% 
%         time_log{4} = datetime('now', 'Format', 'HH:mm:ss.SSSSSS');
%         beta2 = obj.Beta2_SuspendedLoadxyDst(x, xd', vf, P); 
% 
%         time_log{5} = datetime('now', 'Format', 'HH:mm:ss.SSSSSS');
%         vs_alpha2 = obj.V2_alpha2_SuspendedLoadxyDst(x, xd', vf, vs', P); 
% 
%         time_log{6} = datetime('now', 'Format', 'HH:mm:ss.SSSSSS');
%         us = beta2 \ vs_alpha2; 
% 
%         total_time = toc;
% 
%         if total_time * 1000 > 50 
%             fprintf('\n🚨======== 制御フリーズ検出 (処理時間: %.2f ms) ========🚨\n', total_time * 1000);
%             fprintf('1. vf 開始時     : %s\n', time_log{1});
%             fprintf('2. vs 開始時     : %s\n', time_log{2});
%             fprintf('3. uf 開始時     : %s\n', time_log{3});
%             fprintf('4. beta 開始時   : %s\n', time_log{4});
%             fprintf('5. alpha 開始時  : %s\n', time_log{5});
%             fprintf('6. 左除算 開始時 : %s\n', time_log{6});
%             fprintf('7. 全体終了時    : %s\n', datetime('now', 'Format', 'HH:mm:ss.SSSSSS'));
%             fprintf('====================================================\n');
%         end
% 
%         tmp = [uf(1); us]; % ノミナル入力 [u1_nom; u2_nom; u3_nom; u4_nom]
%         obj.result.tmp = tmp; 
% 
%         %% =========================================================================
%         %% 【ステップ 1】 マルチセグメント球体 HOCBF による解析的安全推力 u1_safe の算出
%         %% =========================================================================
%         min_surf_dist = Inf;
%         obj.result.min_clearance = Inf; 
% 
%         obs_env = ENVIRONMENT_OBSTACLE_HOCBF_Z(); 
%         num_obs = length(obs_env);
% 
%         pL = model.state.pL;  
%         L_cable = P(7); 
%         p_mid = pL - 0.5 * L_cable * pT; % ケーブル中央位置
% 
%         % 🌟 ケーブル・システムを被覆する球体モデルの設定
%         % lambda_list: 荷物(0.0) 〜 ケーブル中央(0.5) 〜 機体(1.0) の位置比率
%         % rl_list: 各球体の半径 (例: 荷物=0.20m, ケーブル中央=0.15m, 機体=0.35m)
%         lambda_list = [0.0,  0.5,  1.0]; 
%         % rl_list     = [0.20, 0.15, 0.35]; 
%         rl_list     = [0.5, 0.5, 0.5];
%         num_spheres = length(lambda_list);
% 
%         obj.result.lambda_list = lambda_list;
%         obj.result.rl_list     = rl_list;
% 
%         log_p_obs     = cell(1, max(1, num_obs));
%         log_r_obs     = cell(1, max(1, num_obs));
%         log_r_minimal = cell(1, max(1, num_obs));
% 
%         % z方向 CBF 解析解用の上下限境界
%         u1_lower_bound = 0.0;
%         u1_upper_bound = 20.0;
%         gamma_params_z = [1.0; 5.0]; 
% 
%         for i = 1:num_obs
%             xo = obs_env(i).p_obs(1);
%             yo = obs_env(i).p_obs(2);
%             zo = obs_env(i).p_obs(3);
%             ro = obs_env(i).r_obs;
% 
%             p_obs = [xo; yo; zo];
%             obs_params = [xo; yo; zo; ro];
% 
%             obs_min_dist = Inf; % 障害物 i に対する最小クリアランス
% 
%             % システムを被覆する全ての球体 j についてループ評価
%             for j = 1:num_spheres
%                 lambda_j = lambda_list(j);
%                 rl_j     = rl_list(j);
%                 sphere_params = [lambda_j; rl_j];
% 
%                 % 第 j 被覆球体の中心位置 x_c^j
%                 x_c_j = pL - lambda_j * L_cable * pT;
% 
%                 % 障害物（球体）表面までの距離計算（クリアランス）
%                 dist_center = norm(x_c_j - p_obs);
%                 d_surf      = dist_center - (ro + rl_j);
% 
%                 if d_surf < min_surf_dist
%                     min_surf_dist = d_surf;
%                 end
%                 if d_surf < obs_min_dist
%                     obs_min_dist = d_surf;
%                 end
% 
%                 % z方向 CBF 制約（A_z * u1 <= b_z）を導出関数から取得
%                 [A_z_single, b_z_single] = CBF_Constraints_zlink4(...
%                     obj, x, xd, vf, obs_params, sphere_params, gamma_params_z, P);
% 
%                 % A_i * u1 >= B_i に変換 (符号反転)
%                 A_i = -A_z_single;
%                 B_i = -b_z_single;
% 
%                 if A_i > 1e-9
%                     u1_lower_bound = max(u1_lower_bound, B_i / A_i);
%                 elseif A_i < -1e-9
%                     u1_upper_bound = min(u1_upper_bound, B_i / A_i);
%                 end
%             end
% 
%             log_p_obs{i}     = p_obs;
%             log_r_obs{i}     = ro;
%             log_r_minimal{i} = obs_min_dist;
%         end
% 
%         u1_nominal = tmp(1);
%         u1_safe    = max(u1_lower_bound, min(u1_upper_bound, u1_nominal));
% 
%         % ドローンの物理アクチュエータ限界(0N〜20N)に直接合わせる
%         u1_safe    = max(0.0, min(20.0, u1_safe));
% 
%         % 確定した z 方向安全推力をセット
%         tmp(1) = u1_safe;
% 
%         % 結果の格納
%         obj.result.p_mid          = p_mid;
%         obj.result.min_clearance  = min_surf_dist;
%         obj.result.p_obs          = log_p_obs;
%         obj.result.r_obs          = log_r_obs;
%         obj.result.r_minimal      = log_r_minimal;
%         obj.result.controllertime = toc(tic_start);
% 
%         obj.result.input = [max(0, min(20, tmp(1))); ... % u1 (解析的CBFで確定された推力)
%                             max(-1, min(1, tmp(2))); ...  % u2 (安全化されたroll)
%                             max(-1, min(1, tmp(3))); ...  % u3 (安全化されたpitch)
%                             max(-1, min(1, tmp(4)))];    % u4 (yaw)
% 
%         obj.result.xd = xd;
%         obj.result.x  = x;
%         result        = obj.result;
%     end
% 
%     function show(obj)
%         obj.result
%     end
% end
% end

% classdef HLC_SUSPENDED_LOAD_HOCBF_LOAD_ZXY_LINK_NEW < handle
%     % クアッドコプター用階層型線形化（z解析的CBF ＋ xy実入力結合型HOCBF ＋ ケーブル角度制限付き）
%     % 条件：障害物＝球体(円)、システム＝マルチセグメント球体被覆モデル（HOCBF版）
% properties
%     self
%     result
%     param
% end
% 
% methods
%     function obj = HLC_SUSPENDED_LOAD_HOCBF_LOAD_ZXY_LINK_NEW(self, param)
%         obj.self = self;
%         obj.param = param;
%         % 初期化時のログ参照エラー防止（安全ガード）
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
%         % 仮想入力のゲイン
%         F1 = Param.F1; 
%         F2 = Param.F2; 
%         F3 = Param.F3; 
%         F4 = Param.F4; 
%         time_log = cell(1, 6);
% 
%         tic;
%         time_log{1} = datetime('now', 'Format', 'HH:mm:ss.SSSSSS');
%         vf = obj.Vfd_SuspendedLoadxyDst(Param.dt, x, xd', F1); 
% 
%         time_log{2} = datetime('now', 'Format', 'HH:mm:ss.SSSSSS');
%         vs = obj.Vs_SuspendedLoadxyDst(x, xd', vf, P, F2, F3, F4); 
% 
%         time_log{3} = datetime('now', 'Format', 'HH:mm:ss.SSSSSS');
%         uf = obj.Uf_SuspendedLoadxyDst(x, xd', vf, P); 
% 
%         time_log{4} = datetime('now', 'Format', 'HH:mm:ss.SSSSSS');
%         beta2 = obj.Beta2_SuspendedLoadxyDst(x, xd', vf, P); 
% 
%         time_log{5} = datetime('now', 'Format', 'HH:mm:ss.SSSSSS');
%         vs_alpha2 = obj.V2_alpha2_SuspendedLoadxyDst(x, xd', vf, vs', P); 
% 
%         time_log{6} = datetime('now', 'Format', 'HH:mm:ss.SSSSSS');
%         us = beta2 \ vs_alpha2; 
% 
%         total_time = toc;
% 
%         if total_time * 1000 > 50 
%             fprintf('\n🚨======== 制御フリーズ検出 (処理時間: %.2f ms) ========🚨\n', total_time * 1000);
%             fprintf('1. vf 開始時     : %s\n', time_log{1});
%             fprintf('2. vs 開始時     : %s\n', time_log{2});
%             fprintf('3. uf 開始時     : %s\n', time_log{3});
%             fprintf('4. beta 開始時   : %s\n', time_log{4});
%             fprintf('5. alpha 開始時  : %s\n', time_log{5});
%             fprintf('6. 左除算 開始時 : %s\n', time_log{6});
%             fprintf('7. 全体終了時    : %s\n', datetime('now', 'Format', 'HH:mm:ss.SSSSSS'));
%             fprintf('====================================================\n');
%         end
% 
%         tmp = [uf(1); us]; % ノミナル入力 [u1_nom; u2_nom; u3_nom; u4_nom]
%         obj.result.tmp = tmp; 
% 
%         %% =========================================================================
%         %% 【ステップ 1】 マルチセグメント球体 HOCBF による解析的安全推力 u1_safe の算出
%         %% =========================================================================
%         min_surf_dist = Inf;
%         obj.result.min_clearance = Inf; 
% 
%         obs_env = ENVIRONMENT_OBSTACLE_HOCBF_Z(); 
%         num_obs = length(obs_env);
% 
%         pL = model.state.pL;  
%         L_cable = P(7); 
%         p_mid = pL - 0.5 * L_cable * pT; % ケーブル中央位置
% 
%         % 🌟 ケーブル・システムを被覆する球体モデルの設定
%         lambda_list = [0.0,  0.5,  1.0]; 
%         rl_list     = [0.5,  0.5,  0.5];
%         num_spheres = length(lambda_list);
% 
%         obj.result.lambda_list = lambda_list;
%         obj.result.rl_list     = rl_list;
% 
%         log_p_obs     = cell(1, max(1, num_obs));
%         log_r_obs     = cell(1, max(1, num_obs));
%         log_r_minimal = cell(1, max(1, num_obs));
% 
%         % z方向 CBF 解析解用の上下限境界
%         u1_lower_bound = 0.0;
%         u1_upper_bound = 20.0;
%         gamma_params_z = [1.0; 5.0]; 
% 
%         for i = 1:num_obs
%             xo = obs_env(i).p_obs(1);
%             yo = obs_env(i).p_obs(2);
%             zo = obs_env(i).p_obs(3);
%             ro = obs_env(i).r_obs;
% 
%             p_obs = [xo; yo; zo];
%             obs_params = [xo; yo; zo; ro];
%             obs_min_dist = Inf; 
% 
%             % システムを被覆する全ての球体 j についてループ評価
%             for j = 1:num_spheres
%                 lambda_j = lambda_list(j);
%                 rl_j     = rl_list(j);
%                 sphere_params = [lambda_j; rl_j];
% 
%                 % 第 j 被覆球体の中心位置 x_c^j
%                 x_c_j = pL - lambda_j * L_cable * pT;
% 
%                 % 障害物（球体）表面までの距離計算（クリアランス）
%                 dist_center = norm(x_c_j - p_obs);
%                 d_surf      = dist_center - (ro + rl_j);
% 
%                 if d_surf < min_surf_dist
%                     min_surf_dist = d_surf;
%                 end
%                 if d_surf < obs_min_dist
%                     obs_min_dist = d_surf;
%                 end
% 
%                 % z方向 CBF 制約（A_z * u1 <= b_z）を取得
%                 [A_z_single, b_z_single] = CBF_Constraints_zlink4(...
%                     obj, x, xd, vf, obs_params, sphere_params, gamma_params_z, P);
% 
%                 % A_i * u1 >= B_i に変換 (符号反転)
%                 A_i = -A_z_single;
%                 B_i = -b_z_single;
% 
%                 if A_i > 1e-9
%                     u1_lower_bound = max(u1_lower_bound, B_i / A_i);
%                 elseif A_i < -1e-9
%                     u1_upper_bound = min(u1_upper_bound, B_i / A_i);
%                 end
%             end
% 
%             log_p_obs{i}     = p_obs;
%             log_r_obs{i}     = ro;
%             log_r_minimal{i} = obs_min_dist;
%         end
% 
%         u1_nominal = tmp(1);
%         u1_safe    = max(u1_lower_bound, min(u1_upper_bound, u1_nominal));
%         u1_safe    = max(0.0, min(20.0, u1_safe));
% 
%         % 確定した z 方向安全推力をセット
%         tmp(1) = u1_safe;
% 
%         %% =========================================================================
%         %% 【ステップ 2】 xy 方向 HOCBF (u2, u3) による安全補正 (2次元 QP)
%         %% =========================================================================
%         u23_nominal = [tmp(2); tmp(3)]; % [u2_nom; u3_nom] (Roll, Pitch)
%         u1_val = u1_safe;                % z側で求まった安全推力
% 
%         gamma_params_xy = [1.0; 3.0; 4.0; 5.0]; % 相対次数4用のゲイン(4要素)
% 
%         A_xy_qp_list = [];
%         b_xy_qp_list = [];
% 
%         for i = 1:num_obs
%             xo = obs_env(i).p_obs(1);
%             yo = obs_env(i).p_obs(2);
%             zo = obs_env(i).p_obs(3);
%             ro = obs_env(i).r_obs;
%             obs_params = [xo; yo; zo; ro];
% 
%             for j = 1:num_spheres
%                 sphere_params = [lambda_list(j); rl_list(j)];
% 
%                 % xy方向 CBF 制約（A_xy * [u2; u3] <= b_xy）を取得
%                 [A_xy_single, b_xy_single] = CBF_Constraints_xylink(...
%                     obj, x, xd, vf, u1_val, obs_params, sphere_params, gamma_params_xy, P);
% 
%                 % 制約行列・ベクトルの積み上げ
%                 A_xy_qp_list = [A_xy_qp_list; A_xy_single]; %#ok<AGROW> % [N x 2]
%                 b_xy_qp_list = [b_xy_qp_list; b_xy_single]; %#ok<AGROW> % [N x 1]
%             end
%         end
% 
%         % 🌟 QP (quadprog) により [u2; u3] の安全値を決定
%         % min 0.5 * || u23 - u23_nominal ||^2
%         % s.t. A_xy_qp_list * u23 <= b_xy_qp_list
%         %      -1.0 <= u23 <= 1.0 (物理限界)
%         H_qp = eye(2);
%         f_qp = -u23_nominal;
%         lb_qp = [-1.0; -1.0];
%         ub_qp = [ 1.0;  1.0];
% 
%         options = optimoptions('quadprog', 'Display', 'off', 'ConstraintTolerance', 1e-4);
%         [u23_safe, ~, exitflag] = quadprog(H_qp, f_qp, A_xy_qp_list, b_xy_qp_list, [], [], lb_qp, ub_qp, [], options);
% 
%         if exitflag == 1
%             tmp(2) = u23_safe(1); % u2 (Roll)
%             tmp(3) = u23_safe(2); % u3 (Pitch)
%         else
%             % 解なし(Infeasible)等の場合はノミナル値をクリッピングして代入
%             tmp(2) = max(-1.0, min(1.0, u23_nominal(1)));
%             tmp(3) = max(-1.0, min(1.0, u23_nominal(2)));
%         end
% 
%         % 🌟 結果の格納
%         obj.result.p_mid          = p_mid;
%         obj.result.min_clearance  = min_surf_dist;
%         obj.result.p_obs          = log_p_obs;
%         obj.result.r_obs          = log_r_obs;
%         obj.result.r_minimal      = log_r_minimal;
%         obj.result.controllertime = toc(tic_start);
% 
%         obj.result.input = [max(0, min(20, tmp(1))); ... % u1 (解析的CBFで確定された推力)
%                             max(-1, min(1, tmp(2))); ... % u2 (安全化されたroll)
%                             max(-1, min(1, tmp(3))); ... % u3 (安全化されたpitch)
%                             max(-1, min(1, tmp(4)))];   % u4 (yaw)
% 
%         obj.result.xd = xd;
%         obj.result.x  = x;
%         result        = obj.result;
%     end
% 
%     function show(obj)
%         obj.result
%     end
% end
% end

% classdef HLC_SUSPENDED_LOAD_HOCBF_LOAD_ZXY_LINK_NEW < handle
%     % クアッドコプター用階層型線形化（z解析的CBF ＋ xy実入力結合型HOCBF ＋ ケーブル角度制限付き）
%     % 条件：障害物＝球体(円)、システム＝マルチセグメント球体被覆モデル（HOCBF版）
% properties
%     self
%     result
%     param
% end
% 
% methods
%     function obj = HLC_SUSPENDED_LOAD_HOCBF_LOAD_ZXY_LINK_NEW(self, param)
%         obj.self = self;
%         obj.param = param;
%         % 初期化時のログ参照エラー防止（安全ガード）
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
%         % 仮想入力のゲイン
%         F1 = Param.F1; 
%         F2 = Param.F2; 
%         F3 = Param.F3; 
%         F4 = Param.F4; 
%         time_log = cell(1, 6);
% 
%         tic;
%         time_log{1} = datetime('now', 'Format', 'HH:mm:ss.SSSSSS');
%         vf = obj.Vfd_SuspendedLoadxyDst(Param.dt, x, xd', F1); 
% 
%         time_log{2} = datetime('now', 'Format', 'HH:mm:ss.SSSSSS');
%         vs = obj.Vs_SuspendedLoadxyDst(x, xd', vf, P, F2, F3, F4); 
% 
%         time_log{3} = datetime('now', 'Format', 'HH:mm:ss.SSSSSS');
%         uf = obj.Uf_SuspendedLoadxyDst(x, xd', vf, P); 
% 
%         time_log{4} = datetime('now', 'Format', 'HH:mm:ss.SSSSSS');
%         beta2 = obj.Beta2_SuspendedLoadxyDst(x, xd', vf, P); 
% 
%         time_log{5} = datetime('now', 'Format', 'HH:mm:ss.SSSSSS');
%         vs_alpha2 = obj.V2_alpha2_SuspendedLoadxyDst(x, xd', vf, vs', P); 
% 
%         time_log{6} = datetime('now', 'Format', 'HH:mm:ss.SSSSSS');
%         us = beta2 \ vs_alpha2; 
% 
%         total_time = toc;
% 
%         if total_time * 1000 > 50 
%             fprintf('\n🚨======== 制御フリーズ検出 (処理時間: %.2f ms) ========🚨\n', total_time * 1000);
%             fprintf('1. vf 開始時     : %s\n', time_log{1});
%             fprintf('2. vs 開始時     : %s\n', time_log{2});
%             fprintf('3. uf 開始時     : %s\n', time_log{3});
%             fprintf('4. beta 開始時   : %s\n', time_log{4});
%             fprintf('5. alpha 開始時  : %s\n', time_log{5});
%             fprintf('6. 左除算 開始時 : %s\n', time_log{6});
%             fprintf('7. 全体終了時    : %s\n', datetime('now', 'Format', 'HH:mm:ss.SSSSSS'));
%             fprintf('====================================================\n');
%         end
% 
%         tmp = [uf(1); us]; % ノミナル入力 [u1_nom; u2_nom; u3_nom; u4_nom]
%         obj.result.tmp = tmp; 
% 
%         %% =========================================================================
%         %% 【ステップ 1】 マルチセグメント球体 HOCBF による解析的安全推力 u1_safe の算出
%         %% =========================================================================
%         min_surf_dist = Inf;
%         obj.result.min_clearance = Inf; 
% 
%         obs_env = ENVIRONMENT_OBSTACLE_HOCBF_Z(); 
%         num_obs = length(obs_env);
% 
%         pL = model.state.pL;  
%         L_cable = P(7); 
%         p_mid = pL - 0.5 * L_cable * pT; % ケーブル中央位置
% 
%         % 🌟 ケーブル・システムを被覆する球体モデルの設定
%         lambda_list = [0.0,  0.5,  1.0]; 
%         rl_list     = [0.5,  0.5,  0.5];
%         num_spheres = length(lambda_list);
% 
%         obj.result.lambda_list = lambda_list;
%         obj.result.rl_list     = rl_list;
% 
%         log_p_obs     = cell(1, max(1, num_obs));
%         log_r_obs     = cell(1, max(1, num_obs));
%         log_r_minimal = cell(1, max(1, num_obs));
% 
%         % % z方向 CBF 解析解用の上下限境界
%         % u1_lower_bound = 0.0;
%         % u1_upper_bound = 20.0;
%         % gamma_params_z = [1.0; 5.0]; 
%         % 
%         % for i = 1:num_obs
%         %     xo = obs_env(i).p_obs(1);
%         %     yo = obs_env(i).p_obs(2);
%         %     zo = obs_env(i).p_obs(3);
%         %     ro = obs_env(i).r_obs;
%         % 
%         %     p_obs = [xo; yo; zo];
%         %     obs_params = [xo; yo; zo; ro];
%         %     obs_min_dist = Inf; 
%         % 
%         %     % システムを被覆する全ての球体 j についてループ評価
%         %     for j = 1:num_spheres
%         %         lambda_j = lambda_list(j);
%         %         rl_j     = rl_list(j);
%         %         sphere_params = [lambda_j; rl_j];
%         % 
%         %         % 第 j 被覆球体の中心位置 x_c^j
%         %         x_c_j = pL - lambda_j * L_cable * pT;
%         % 
%         %         % 障害物（球体）表面までの距離計算（クリアランス）
%         %         dist_center = norm(x_c_j - p_obs);
%         %         d_surf      = dist_center - (ro + rl_j);
%         % 
%         %         if d_surf < min_surf_dist
%         %             min_surf_dist = d_surf;
%         %         end
%         %         if d_surf < obs_min_dist
%         %             obs_min_dist = d_surf;
%         %         end
%         % 
%         %         % z方向 CBF 制約（A_z * u1 <= b_z）を取得
%         %         [A_z_single, b_z_single] = CBF_Constraints_zlink4(...
%         %             obj, x, xd, vf, obs_params, sphere_params, gamma_params_z, P);
%         % 
%         %         % A_i * u1 >= B_i に変換 (符号反転)
%         %         A_i = -A_z_single;
%         %         B_i = -b_z_single;
%         % 
%         %         if A_i > 1e-9
%         %             u1_lower_bound = max(u1_lower_bound, B_i / A_i);
%         %         elseif A_i < -1e-9
%         %             u1_upper_bound = min(u1_upper_bound, B_i / A_i);
%         %         end
%         %     end
%         % 
%         %     log_p_obs{i}     = p_obs;
%         %     log_r_obs{i}     = ro;
%         %     log_r_minimal{i} = obs_min_dist;
%         % end
%         % 
%         % u1_nominal = tmp(1);
%         % u1_safe    = max(u1_lower_bound, min(u1_upper_bound, u1_nominal));
%         % u1_safe    = max(0.0, min(20.0, u1_safe));
%         % 
%         % % 確定した z 方向安全推力をセット
%         % tmp(1) = u1_safe;
% 
%         %% =========================================================================
%         %% 【ステップ 2】 xy 方向 HOCBF (u2, u3) ＋ ケーブル角度制限による QP 安全補正
%         %% =========================================================================
%         u23_nominal = [tmp(2); tmp(3)]; % [u2_nom; u3_nom] (Roll, Pitch)
%         u1_val = tmp(1);                % z側で求まった安全推力
% 
%         gamma_params_xy = [5.0; 5.0; 5.0; 5.0]; % 相対次数4用のゲイン(4要素)
% 
%         A_xy_qp_list = [];
%         b_xy_qp_list = [];
% 
%         % --- A. 障害物回避制約（全障害物 × 全被覆球体）の積載 ---
%         for i = 1:num_obs
%             xo = obs_env(i).p_obs(1);
%             yo = obs_env(i).p_obs(2);
%             zo = obs_env(i).p_obs(3);
%             ro = obs_env(i).r_obs;
%             obs_params = [xo; yo; zo; ro];
% 
%             for j = 1:num_spheres
%                 sphere_params = [lambda_list(j); rl_list(j)];
% 
%                 % 🌟 ご提示の導出スクリプト（全9引数）に完全対応させて呼び出し
%                 [A_xy_single, b_xy_single] = CBF_Constraints_xylink(...
%                     obj, x, xd, vf, u1_val, obs_params, sphere_params, gamma_params_xy, P);
% 
%                 A_xy_qp_list = [A_xy_qp_list; A_xy_single]; %#ok<AGROW> % [1 x 2]
%                 b_xy_qp_list = [b_xy_qp_list; b_xy_single]; %#ok<AGROW> % [1 x 1]
%             end
%         end
% 
%         % --- B. 🌟 ケーブル角度制限 CBF 制約の積載 (相対次数4) ---
%         max_cable_angle    = deg2rad(30);             % 許容最大傾斜角 30 deg
%         gamma_cable_params = [1.0; 2.0; 3.0; 4.0];   % 相対次数4用のゲイン(4要素)
% 
%         % 🌟 ご提示の導出スクリプト（全8引数）に完全対応させて呼び出し
%         % ※ 生成用コード末尾のファイル名を CBF_Constraints_Cable.m に直して生成した関数を呼ぶ
%         [A_cable_single, b_cable_single] = CBF_Constraints_Cable(...
%             obj, x, xd, vf, u1_val, max_cable_angle, gamma_cable_params, P);
% 
%         % 全 CBF 制約（回避 ＋ ケーブル角度）の合体
%         A_all_qp = [A_xy_qp_list; A_cable_single];
%         b_all_qp = [b_xy_qp_list; b_cable_single];
% 
%         % --- C. 🌟 QP (quadprog) により [u2; u3] の安全値を一括決定 ---
%         % H_qp  = eye(2);
%         H_qp = diag([0.4, 0.4]);
%         f_qp  = -H_qp * u23_nominal;
%         lb_qp = [-1.0; -1.0];
%         ub_qp = [ 1.0;  1.0];
% 
%         options = optimoptions('quadprog', 'Display', 'off', 'ConstraintTolerance', 1e-4);
%         [u23_safe, ~, exitflag] = quadprog(H_qp, f_qp, A_all_qp, b_all_qp, [], [], lb_qp, ub_qp, [], options);
% 
%         % 🌟 デバッグ用: xy CBF 制約が発動しているかのモニタリング
%         slack_check = A_all_qp * u23_nominal - b_all_qp; % ノミナル入力での制約違反量
%         num_violated = sum(slack_check > 0);            % 違反している（介入が必要な）制約数
% 
%         if num_violated > 0
%             fprintf('[xy-CBF発動] 違反制約数: %d/%d | 最大違反量: %.4f | u23_nom:[%.2f, %.2f] -> u23_safe:[%.2f, %.2f] (exitflag:%d)\n', ...
%                 num_violated, length(b_all_qp), max(slack_check), ...
%                 u23_nominal(1), u23_nominal(2), tmp(2), tmp(3), exitflag);
%         end
%         if exitflag == 1
%             tmp(2) = u23_safe(1); % u2 (Roll)
%             tmp(3) = u23_safe(2); % u3 (Pitch)
%         else
%             % 制約が干渉した場合のフォールバック (クリッピング)
%             tmp(2) = max(-1.0, min(1.0, u23_nominal(1)));
%             tmp(3) = max(-1.0, min(1.0, u23_nominal(2)));
%         end
% 
%         % 🌟 結果の格納
%         obj.result.p_mid          = p_mid;
%         obj.result.min_clearance  = min_surf_dist;
%         obj.result.p_obs          = log_p_obs;
%         obj.result.r_obs          = log_r_obs;
%         obj.result.r_minimal      = log_r_minimal;
%         obj.result.controllertime = toc(tic_start);
% 
%         obj.result.input = [max(0, min(20, tmp(1))); ... % u1 (確定推力)
%                             max(-1, min(1, tmp(2))); ... % u2 (安全化Roll)
%                             max(-1, min(1, tmp(3))); ... % u3 (安全化Pitch)
%                             max(-1, min(1, tmp(4)))];   % u4 (Yaw)
% 
%         obj.result.xd = xd;
%         obj.result.x  = x;
%         result        = obj.result;
%     end
% 
%     function show(obj)
%         obj.result
%     end
% end
% end

% classdef HLC_SUSPENDED_LOAD_HOCBF_LOAD_ZXY_LINK_NEW < handle
%     % クアッドコプター用階層型線形化（実入力 u1 保持型 HOCBF 単一球体検証版）
%     % 条件：障害物＝球体(円)、ロープ中点 p_mid 基準（単一球体）
% properties
%     self
%     result
%     param
% end
% 
% methods
%     function obj = HLC_SUSPENDED_LOAD_HOCBF_LOAD_ZXY_LINK_NEW(self, param)
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
%         obs_env = ENVIRONMENT_OBSTACLE_HOCBF_Z(); 
%         num_obs = length(obs_env);
% 
%         pL = model.state.pL;  
%         L_cable = P(7); 
%         p_mid = pL - 0.5 * L_cable * pT; % ロープ（リンク）の中点
% 
%         % システム側単一球体の半径 (rl)
%         rl_sys = 1;
%         obj.result.rl = rl_sys;
% 
%         % アニメーション描画用フィールドのセット
%         obj.result.p_mid = p_mid;
% 
%         log_p_obs     = cell(1, max(1, num_obs));
%         log_r_obs     = cell(1, max(1, num_obs));
%         log_r_minimal = cell(1, max(1, num_obs));
% 
%         for i = 1:num_obs
%             xo = obs_env(i).p_obs(1); yo = obs_env(i).p_obs(2); zo = obs_env(i).p_obs(3);
%             ro = obs_env(i).r_obs;
%             p_obs = [xo; yo; zo];
% 
%             % 単一球体 (p_mid) から障害物表面までの距離
%             d_surf = norm(p_mid - p_obs) - (ro + rl_sys);
% 
%             if d_surf < min_surf_dist
%                 min_surf_dist = d_surf;
%             end
% 
%             log_p_obs{i}     = p_obs;
%             log_r_obs{i}     = ro;
%             log_r_minimal{i} = d_surf;
%         end
% 
%         obj.result.min_clearance = min_surf_dist;
%         obj.result.p_obs         = log_p_obs;
%         obj.result.r_obs         = log_r_obs;
%         obj.result.r_minimal     = log_r_minimal;
% 
%         %% =========================================================================
%         %% 【ステップ 2】 実入力 u1 保持型 CBF_Constraints_xyotamesi による xy 方向 QP 安全補正
%         %% =========================================================================
%         u23_nominal = [tmp(2); tmp(3)]; % [u2_nom; u3_nom]
%         u1_val      = tmp(1);           % U1_val (実入力推力 u1 の確定値)
%         u4_val      = tmp(4);           % V4 (Yaw軸制御入力 u4)
% 
%         % 🌟 4階 HOCBF ゲインパラメータ (gamma1 〜 gamma4)
%         gamma_params_xy = [8.0; 5.0; 5.0; 5.0]; 
% 
%         A_xy_qp_list = [];
%         b_xy_qp_list = [];
% 
%         % 全障害物に対する CBF 制約の積載 (単一球体モデル)
%         for i = 1:num_obs
%             obs_params = [obs_env(i).p_obs; obs_env(i).r_obs]; % [xo; yo; zo; ro]
%             sys_params = rl_sys;                                 % rl
% 
%             % 🌟 CBF_Constraints_xyotamesi.m の 9 引数に完全対応させて呼び出し
%             % 引数順: {obj, x, XD_sym, U1_val, V4, obs_params, gamma_params, sys_params, physicalParam}
%             [A_xy_single, b_xy_single] = CBF_Constraints_xyotamesi(...
%                 obj, x, xd, u1_val, u4_val, obs_params, gamma_params_xy, sys_params, P);
% 
%             A_xy_qp_list = [A_xy_qp_list; A_xy_single]; %#ok<AGROW>
%             b_xy_qp_list = [b_xy_qp_list; b_xy_single]; %#ok<AGROW>
%         end
% 
%         % 🌟 2次元 QP (quadprog) の実行
%         H_qp  = diag([1.0, 1.0]);
%         f_qp  = -H_qp * u23_nominal;
%         lb_qp = [-2.0; -2.0];
%         ub_qp = [ 2.0;  2.0];
% 
%         options = optimoptions('quadprog', 'Display', 'off', 'ConstraintTolerance', 1e-4);
%         [u23_safe, ~, exitflag] = quadprog(H_qp, f_qp, A_xy_qp_list, b_xy_qp_list, [], [], lb_qp, ub_qp, [], options);
% 
%         % 安全ガード付き更新処理
%         if exitflag == 1 && ~isempty(u23_safe)
%             tmp(2) = u23_safe(1); % u2 (Roll)
%             tmp(3) = u23_safe(2); % u3 (Pitch)
%         else
%             % 解なし時等のフォールバック (ノミナル値をクリッピング)
%             tmp(2) = max(-1.0, min(1.0, u23_nominal(1)));
%             tmp(3) = max(-1.0, min(1.0, u23_nominal(2)));
%         end
% 
%         % デバッグ用コンソール出力 (制約アクティブ状態の確認)
%         slack_check  = A_xy_qp_list * u23_nominal - b_xy_qp_list;
%         num_violated = sum(slack_check > 0);
%         if num_violated > 0
%             fprintf('[xyotamesi 1球体CBF] 違反数:%d/%d | 最大違反量:%.2f | u23_nom:[%.2f, %.2f] -> u23_out:[%.2f, %.2f] (exitflag:%d)\n', ...
%                 num_violated, length(b_xy_qp_list), max(slack_check), ...
%                 u23_nominal(1), u23_nominal(2), tmp(2), tmp(3), exitflag);
%         end
% 
%         % 結果の格納
%         obj.result.p_mid          = p_mid;
%         obj.result.min_clearance  = min_surf_dist;
%         obj.result.p_obs          = log_p_obs;
%         obj.result.r_obs          = log_r_obs;
%         obj.result.r_minimal      = log_r_minimal;
%         obj.result.controllertime = toc(tic_start);
% 
%         % 最終制御入力 (物理範囲 [-1, 1] 内に収めて出力)
%         obj.result.input = [max(0.0, min(20.0, tmp(1))); ... % u1 (推力)
%                             max(-1.0, min(1.0,  tmp(2))); ... % u2 (Roll)
%                             max(-1.0, min(1.0,  tmp(3))); ... % u3 (Pitch)
%                             max(-1.0, min(1.0,  tmp(4)))];   % u4 (Yaw)
% 
%         obj.result.xd = xd;
%         obj.result.x  = x;
%         result        = obj.result;
%     end
% 
%     function show(obj)
%         obj.result
%     end
% end
% end

classdef HLC_SUSPENDED_LOAD_HOCBF_LOAD_ZXY_LINK_NEW < handle
    % クアッドコプター用階層型線形化（実入力 u1 保持型 HOCBF マルチセグメント球体検証版）
    % 条件：障害物＝球体(円)、システム＝マルチセグメント球体被覆モデル（マージン付き）
properties
    self
    result
    param
end

methods
    function obj = HLC_SUSPENDED_LOAD_HOCBF_LOAD_ZXY_LINK_NEW(self, param)
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

        % 仮想・ノミナル入力の算定
        F1 = Param.F1; F2 = Param.F2; F3 = Param.F3; F4 = Param.F4; 
        vf = obj.Vfd_SuspendedLoadxyDst(Param.dt, x, xd', F1); 
        vs = obj.Vs_SuspendedLoadxyDst(x, xd', vf, P, F2, F3, F4); 
        uf = obj.Uf_SuspendedLoadxyDst(x, xd', vf, P); 
        beta2 = obj.Beta2_SuspendedLoadxyDst(x, xd', vf, P); 
        vs_alpha2 = obj.V2_alpha2_SuspendedLoadxyDst(x, xd', vf, vs', P); 
        us = beta2 \ vs_alpha2; 

        tmp = [uf(1); us]; % ノミナル入力 [u1_nom; u2_nom; u3_nom; u4_nom]
        obj.result.tmp = tmp; 

        %% =========================================================================
        %% 【ステップ 1】 クリアランス計算 ＆ フィールド初期化
        %% =========================================================================
        min_surf_dist = Inf;
        obs_env = ENVIRONMENT_OBSTACLE_HOCBF_Z(); 
        num_obs = length(obs_env);
        
        pL = model.state.pL;  
        L_cable = P(7); 
        p_mid = pL - 0.5 * L_cable * pT;

        % 🌟 マルチセグメント球体モデルの設定 ＆ めり込み防止安全マージン (+0.5m)
        lambda_list = [0.0, 0.5, 1.0]; 
        rl_base     = [0.5, 0.5, 0.5]; 
        rl_margin   = 0.5;                             % めり込み防止マージン [m]
        rl_list     = rl_base + rl_margin;             % 実際の回避計算用半径 [1.0, 1.0, 1.0]
        num_spheres = length(lambda_list);

        % アニメーション描画用にオリジナル物理半径とマージン後半径をセット
        obj.result.lambda_list = lambda_list;
        obj.result.rl_list     = rl_base;              % 描画・判定用は物理半径
        obj.result.p_mid       = p_mid;

        log_p_obs     = cell(1, max(1, num_obs));
        log_r_obs     = cell(1, max(1, num_obs));
        log_r_minimal = cell(1, max(1, num_obs));

        for i = 1:num_obs
            xo = obs_env(i).p_obs(1); yo = obs_env(i).p_obs(2); zo = obs_env(i).p_obs(3);
            ro = obs_env(i).r_obs;
            p_obs = [xo; yo; zo];
            obs_min_dist = Inf;

            for j = 1:num_spheres
                x_c_j  = pL - lambda_list(j) * L_cable * pT;
                % 物理的な表面クリアランス評価（ログ・最小クリアランス判定用）
                d_surf = norm(x_c_j - p_obs) - (ro + rl_base(j));
                if d_surf < min_surf_dist, min_surf_dist = d_surf; end
                if d_surf < obs_min_dist,  obs_min_dist = d_surf;  end
            end

            log_p_obs{i}     = p_obs;
            log_r_obs{i}     = ro;
            log_r_minimal{i} = obs_min_dist;
        end

        obj.result.min_clearance = min_surf_dist;
        obj.result.p_obs         = log_p_obs;
        obj.result.r_obs         = log_r_obs;
        obj.result.r_minimal     = log_r_minimal;

        %% =========================================================================
        %% 【ステップ 2】 CBF_Constraints_xyotamesi によるマルチセグメント xy QP 安全補正
        %% =========================================================================
        %% =========================================================================
        %% 【ステップ 2】 正規化 Soft-QP による HOCBF (xy方向) 安全補正
        %% =========================================================================
        u23_nominal = [tmp(2); tmp(3)]; % [u2_nom; u3_nom]
        u1_val      = tmp(1);           % U1_val
        u4_val      = tmp(4);           % V4

        % 🌟 HOCBF ゲインパラメータ (積 = 12 程度にマイルド化して過剰要求を防ぐ)
        gamma_params_xy = [1.0; 2.0; 2.0; 3.0]; 

        A_raw_list = [];
        b_raw_list = [];

        % 1. 各被覆球体の CBF 制約を計算
        for i = 1:num_obs
            obs_params = [obs_env(i).p_obs; obs_env(i).r_obs];
            for j = 1:num_spheres
                sys_params = [lambda_list(j); rl_list(j)];

                [A_single, b_single] = CBF_Constraints_xyotamesi(...
                    obj, x, xd, u1_val, u4_val, obs_params, gamma_params_xy, sys_params, P);

                % 🌟 行列の正規化 (A_k のノルムで両辺を割って桁数を揃える)
                norm_A = norm(A_single);
                if norm_A > 1e-6
                    A_norm = A_single / norm_A;
                    b_norm = b_single / norm_A;
                else
                    A_norm = A_single;
                    b_norm = b_single;
                end

                A_raw_list = [A_raw_list; A_norm]; %#ok<AGROW>
                b_raw_list = [b_raw_list; b_norm]; %#ok<AGROW>
            end
        end

        num_constraints = length(b_raw_list);

        % 🌟 2. スラック変数付き Soft QP で物理限界内の「最大回避出力」を保証
        if num_constraints > 0
            % 最適化変数 U_qp = [u2; u3; slack_1; ...; slack_m]
            w_slack = 1e4; % スラック変数のペナルティ
            H_qp    = diag([0.1, 0.1, w_slack * ones(1, num_constraints)]);
            f_qp    = [-0.1 * u23_nominal(1); -0.1 * u23_nominal(2); zeros(num_constraints, 1)];

            % 制約: A * [u2; u3] - slack <= b
            A_soft = [A_raw_list, -eye(num_constraints)];
            b_soft = b_raw_list;

            % 物理入力限界: Roll, Pitch in [-2.0, 2.0], slack >= 0
            lb_qp = [-2.0; -2.0; zeros(num_constraints, 1)];
            ub_qp = [ 2.0;  2.0; Inf * ones(num_constraints, 1)];

            options = optimoptions('quadprog', 'Display', 'off', 'ConstraintTolerance', 1e-4);
            [U_safe, ~, exitflag] = quadprog(H_qp, f_qp, A_soft, b_soft, [], [], lb_qp, ub_qp, [], options);

            if (exitflag == 1 || exitflag == 0) && ~isempty(U_safe)
                tmp(2) = U_safe(1); % 安全 Roll
                tmp(3) = U_safe(2); % 安全 Pitch
            else
                % 万一のフォールバック
                tmp(2) = max(-1.0, min(1.0, u23_nominal(1)));
                tmp(3) = max(-1.0, min(1.0, u23_nominal(2)));
            end

            % デバッグ出力
            slack_check  = A_raw_list * u23_nominal - b_raw_list;
            num_violated = sum(slack_check > 0);
            
            if num_violated > 0
                max_slack = 0;
                if length(U_safe) > 2, max_slack = max(U_safe(3:end)); end
                fprintf('[正規化Soft-QP] 違反数:%d/%d | 最大違反量:%.2f | 発生スラック:%.3f | u23_out:[%.2f, %.2f] (exitflag:%d)\n', ...
                    num_violated, num_constraints, max(slack_check), max_slack, tmp(2), tmp(3), exitflag);
            end
        end

        % 結果の格納（物理限界 [-1, 1] 内にクリッピングして出力）
        obj.result.p_mid          = p_mid;
        obj.result.min_clearance  = min_surf_dist;
        obj.result.p_obs          = log_p_obs;
        obj.result.r_obs          = log_r_obs;
        obj.result.r_minimal      = log_r_minimal;
        obj.result.controllertime = toc(tic_start);

        obj.result.input = [max(0.0, min(20.0, tmp(1))); ... % u1 (推力)
                            max(-1.0, min(1.0,  tmp(2))); ... % u2 (Roll)
                            max(-1.0, min(1.0,  tmp(3))); ... % u3 (Pitch)
                            max(-1.0, min(1.0,  tmp(4)))];   % u4 (Yaw)

        obj.result.xd = xd;
        obj.result.x  = x;
        result        = obj.result;
    end

    function show(obj)
        obj.result
    end
end
end
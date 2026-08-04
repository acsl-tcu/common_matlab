% classdef HLC_SUSPENDED_LOAD_HOCBF_LOAD_Z_LINK_NEW < handle
%     % クアッドコプター用階層型線形化（z解析的CBF ＋ xy実入力結合型HOCBF ＋ ケーブル
%     % 角度制限付き）制約から漏れた場合は20or0として算出　xyに動けないので姿勢変化
%     % 不可能　大きな球体のため複数機体時に問題になりそう　これをもとに楕円と複数球をためす
% properties
%     self
%     result
%     param
% end
% methods
%     function obj = HLC_SUSPENDED_LOAD_HOCBF_LOAD_Z_LINK_NEW(self, param)
%         obj.self = self;
%         obj.param = param;
%         % 🌟 初期化時のログ参照エラー防止（安全ガード）
%         obj.result.min_clearance = Inf;
%     end
% 
%     function result = do(obj, varargin)
%         Param = obj.param; 
%         model = obj.self.estimator.result; 
%         ref = obj.self.reference.result; 
% 
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
%         yaw = wrapToPi(model.state.q(3)); 
%         yawd = xd(4); 
%         yawUnit = [cos(yaw); sin(yaw); 0]; 
%         yawdUnit = [cos(yawd); sin(yawd); 0]; 
%         deltaYaw = sign(cross(yawdUnit, yawUnit)) * acos(yawdUnit' * yawUnit); 
%         xd(4) = -deltaYaw(3) + yaw; 
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
%         %% 【ステップ 1】 z 方向 HOCBF による解析的安全推力 u1_safe の算出
%         %% =========================================================================
%         tic_cbf_setup = tic;
% 
%         % 🌟 ループ前デフォルト値の初期化
%         min_surf_dist = Inf;
%         obj.result.min_clearance = Inf; 
% 
%         obs_env = ENVIRONMENT_OBSTACLE_HOCBF_Z(); 
%         num_obs = length(obs_env);
% 
%         % rl_val = 0.5; 
%         % obj.result.rl = rl_val;
%         pL = model.state.pL;  
%         L_cable = P(7); 
%         rl_val = L_cable/2+0.2;
%         obj.result.rl = rl_val;
%         p_mid = pL - 0.5 * L_cable * pT;
% 
%         log_p_obs = cell(1, max(1, num_obs));
%         log_r_obs = cell(1, max(1, num_obs));
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
%             p_obs = [xo; yo; zo];
%             obs_params = [xo; yo; zo; ro];
% 
%             % 🌟 障害物（球体）表面までの距離計算
%             dist_center = norm(p_mid - p_obs);   % 中心間距離
%             d_surf = dist_center - (ro + rl_val); % 表面間距離（クリアランス）
% 
%             if d_surf < min_surf_dist
%                 min_surf_dist = d_surf;
%             end
% 
%             % z方向 CBF 制約（A_z * u1 <= b_z）を取得
%             [A_z_single, b_z_single] = CBF_Constraints_zlink(obj, x, xd, vf, obs_params, gamma_params_z, rl_val, P);
% 
%             % A_i * u1 >= B_i に変換 (符号反転)
%             A_i = -A_z_single;
%             B_i = -b_z_single;
%             if A_i > 1e-9
%                 u1_lower_bound = max(u1_lower_bound, B_i / A_i);
%             elseif A_i < -1e-9
%                 u1_upper_bound = min(u1_upper_bound, B_i / A_i);
%             end
%         end
% 
%         u1_nominal = tmp(1);
%         u1_safe = max(u1_lower_bound, min(u1_upper_bound, u1_nominal));
%         % 2. ドローンの物理アクチュエータ限界(0N〜20N)に直接合わせる
%         u1_safe = max(0.0, min(20.0, u1_safe));
% 
%         % 🌟 確定した z 方向安全推力をセット
%         tmp(1) = u1_safe;
%         % 🌟 結果を確実に格納
%         obj.result.p_mid = p_mid;
%         obj.result.min_clearance = min_surf_dist;
%         obj.result.p_obs     = log_p_obs;
%         obj.result.r_obs     = log_r_obs;
%         obj.result.r_minimal = log_r_minimal;
%         % obj.result.exitflag  = exitflag;
%         % obj.result.t_qp      = toc(tic_qp);
%         obj.result.controllertime = toc(tic_start);
% 
%         obj.result.input = [max(0, min(20, tmp(1))); ... % u1 (解析的CBFで確定された推力)
%                             max(-1, min(1, tmp(2))); ...  % u2 (安全化されたroll)
%                             max(-1, min(1, tmp(3))); ...  % u3 (安全化されたpitch)
%                             max(-1, min(1, tmp(4)))];    % u4 (yaw)
% 
%         obj.result.xd = xd;
%         obj.result.x = x;
%         result = obj.result;
%     end
% 
%     function show(obj)
%         obj.result
%     end
% end
% end

% classdef HLC_SUSPENDED_LOAD_HOCBF_LOAD_Z_LINK_NEW < handle
%     % クアッドコプター用階層型線形化（z解析的CBF ＋ xy実入力結合型HOCBF ＋ ケーブル
%     % 角度制限付き）制約から漏れた場合は20or0として算出　xyに動けないので姿勢変化
%     % xy方向の範囲を広げたが姿勢を崩したので、牽引角度の制約つけてやってみるあとで
% properties
%     self
%     result
%     param
% end
% methods
%     function obj = HLC_SUSPENDED_LOAD_HOCBF_LOAD_Z_LINK_NEW(self, param)
%         obj.self = self;
%         obj.param = param;
%         % 🌟 初期化時のログ参照エラー防止（安全ガード）
%         obj.result.min_clearance = Inf;
%     end
% 
%     function result = do(obj, varargin)
%         Param = obj.param; 
%         model = obj.self.estimator.result; 
%         ref = obj.self.reference.result; 
% 
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
%         yaw = wrapToPi(model.state.q(3)); 
%         yawd = xd(4); 
%         yawUnit = [cos(yaw); sin(yaw); 0]; 
%         yawdUnit = [cos(yawd); sin(yawd); 0]; 
%         deltaYaw = sign(cross(yawdUnit, yawUnit)) * acos(yawdUnit' * yawUnit); 
%         xd(4) = -deltaYaw(3) + yaw; 
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
%         %% 【ステップ 1】 z 方向 HOCBF による解析的安全推力 u1_safe の算出
%         %% =========================================================================
%         tic_cbf_setup = tic;
% 
%         % 🌟 ループ前デフォルト値の初期化
%         min_surf_dist = Inf;
%         obj.result.min_clearance = Inf; 
% 
%         obs_env = ENVIRONMENT_OBSTACLE_HOCBF_Z(); 
%         num_obs = length(obs_env);
% 
%         % rl_val = 0.5; 
%         % obj.result.rl = rl_val;
%         pL = model.state.pL;  
%         L_cable = P(7); 
%         rl_val = L_cable/2+0.2;
%         obj.result.rl = rl_val;
%         p_mid = pL - 0.5 * L_cable * pT;
% 
%         log_p_obs = cell(1, max(1, num_obs));
%         log_r_obs = cell(1, max(1, num_obs));
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
%             p_obs = [xo; yo; zo];
%             obs_params = [xo; yo; zo; ro];
% 
%             % 🌟 障害物（球体）表面までの距離計算
%             dist_center = norm(p_mid - p_obs);   % 中心間距離
%             d_surf = dist_center - (ro + rl_val); % 表面間距離（クリアランス）
% 
%             if d_surf < min_surf_dist
%                 min_surf_dist = d_surf;
%             end
% 
%             % z方向 CBF 制約（A_z * u1 <= b_z）を取得
%             [A_z_single, b_z_single] = CBF_Constraints_zlink2(obj, x, xd, vf, obs_params, gamma_params_z, rl_val, P);
% 
%             % A_i * u1 >= B_i に変換 (符号反転)
%             A_i = -A_z_single;
%             B_i = -b_z_single;
%             if A_i > 1e-9
%                 u1_lower_bound = max(u1_lower_bound, B_i / A_i);
%             elseif A_i < -1e-9
%                 u1_upper_bound = min(u1_upper_bound, B_i / A_i);
%             end
%         end
% 
%         u1_nominal = tmp(1);
%         u1_safe = max(u1_lower_bound, min(u1_upper_bound, u1_nominal));
%         % 2. ドローンの物理アクチュエータ限界(0N〜20N)に直接合わせる
%         u1_safe = max(0.0, min(20.0, u1_safe));
% 
%         % 🌟 確定した z 方向安全推力をセット
%         tmp(1) = u1_safe;
%         % 🌟 結果を確実に格納
%         obj.result.p_mid = p_mid;
%         obj.result.min_clearance = min_surf_dist;
%         obj.result.p_obs     = log_p_obs;
%         obj.result.r_obs     = log_r_obs;
%         obj.result.r_minimal = log_r_minimal;
%         % obj.result.exitflag  = exitflag;
%         % obj.result.t_qp      = toc(tic_qp);
%         obj.result.controllertime = toc(tic_start);
% 
%         obj.result.input = [max(0, min(20, tmp(1))); ... % u1 (解析的CBFで確定された推力)
%                             max(-1, min(1, tmp(2))); ...  % u2 (安全化されたroll)
%                             max(-1, min(1, tmp(3))); ...  % u3 (安全化されたpitch)
%                             max(-1, min(1, tmp(4)))];    % u4 (yaw)
% 
%         obj.result.xd = xd;
%         obj.result.x = x;
%         result = obj.result;
%     end
% 
%     function show(obj)
%         obj.result
%     end
% end
% end

% classdef HLC_SUSPENDED_LOAD_HOCBF_LOAD_Z_LINK_NEW < handle
%     % クアッドコプター用階層型線形化（z解析的CBF ＋ xy実入力結合型HOCBF ＋ ケーブル角度制限付き）
%     % 条件：障害物＝球体(円)、システム＝機体姿勢連動の回転楕円体（クリアランス厳密計算版）
% properties
%     self
%     result
%     param
% end
% methods
%     function obj = HLC_SUSPENDED_LOAD_HOCBF_LOAD_Z_LINK_NEW(self, param)
%         obj.self = self;
%         obj.param = param;
%         obj.result.min_clearance = Inf;
%     end
% 
%     function result = do(obj, varargin)
%         Param = obj.param; 
%         model = obj.self.estimator.result; 
%         ref = obj.self.reference.result; 
% 
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
%         yaw = wrapToPi(model.state.q(3)); 
%         yawd = xd(4); 
%         yawUnit = [cos(yaw); sin(yaw); 0]; 
%         yawdUnit = [cos(yawd); sin(yawd); 0]; 
%         deltaYaw = sign(cross(yawdUnit, yawUnit)) * acos(yawdUnit' * yawUnit); 
%         xd(4) = -deltaYaw(3) + yaw; 
% 
%         xd = [xd; zeros(28 - size(xd, 1), 1)];
%         tic_start = tic;
% 
%         % 仮想入力のゲイン
%         F1 = Param.F1; F2 = Param.F2; F3 = Param.F3; F4 = Param.F4; 
%         time_log = cell(1, 6);
% 
%         vf = obj.Vfd_SuspendedLoadxyDst(Param.dt, x, xd', F1); 
%         vs = obj.Vs_SuspendedLoadxyDst(x, xd', vf, P, F2, F3, F4); 
%         uf = obj.Uf_SuspendedLoadxyDst(x, xd', vf, P); 
%         beta2 = obj.Beta2_SuspendedLoadxyDst(x, xd', vf, P); 
%         vs_alpha2 = obj.V2_alpha2_SuspendedLoadxyDst(x, xd', vf, vs', P); 
%         us = beta2 \ vs_alpha2; 
% 
%         tmp = [uf(1); us]; % ノミナル入力
%         obj.result.tmp = tmp; 
% 
%         %% =========================================================================
%         %% 【ステップ 1】 姿勢連動型楕円 vs 球体 の厳密クリアランス ＆ HOCBF 評価
%         %% =========================================================================
%         min_surf_dist = Inf;
%         obj.result.min_clearance = Inf; 
% 
%         obs_env = ENVIRONMENT_OBSTACLE_HOCBF_Z(); 
%         num_obs = length(obs_env);
% 
%         pL = model.state.pL;  
%         L_cable = P(7); 
% 
%         % 機体・システム楕円体の軸長設定 (a: 長軸(Z方向), b: 短軸(X/Y方向))
%         a_sys = L_cable / 2 + 0.3; 
%         b_sys = 0.4;               
%         obj.result.a_sys = a_sys;
%         obj.result.b_sys = b_sys;
%         sys_params_ellipsoid = [a_sys; b_sys];
% 
%         % システム中心位置 p_mid
%         p_mid = pL - 0.5 * L_cable * pT;
% 
%         % 機体の回転行列 R_sys の計算 (x から姿勢クオータニオンを取得)
%         q0 = x(1); q1 = x(2); q2 = x(3); q3 = x(4);
%         R_sys = [1 - 2*(q2^2 + q3^2),  2*(q1*q2 - q0*q3),  2*(q1*q3 + q0*q2);
%                  2*(q1*q2 + q0*q3),  1 - 2*(q1^2 + q3^2),  2*(q2*q3 - q0*q1);
%                  2*(q1*q3 - q0*q2),  2*(q2*q3 + q0*q1),  1 - 2*(q1^2 + q2^2)];
% 
%         log_p_obs = cell(1, max(1, num_obs));
%         log_r_obs = cell(1, max(1, num_obs));
%         log_r_minimal = cell(1, max(1, num_obs));
% 
%         u1_lower_bound = 0.0;
%         u1_upper_bound = 20.0;
%         gamma_params_z = [1.0; 6.0]; 
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
%             % 🌟【正確な幾何学クリアランス計算】機体姿勢R_sysと相対方向を考慮
%             delta_p = p_mid - p_obs;
%             dist_center = norm(delta_p);
% 
%             if dist_center > 1e-6
%                 % 障害物方向のロープ座標系ベクトル
%                 p_loc = R_sys.' * delta_p; 
% 
%                 % 相対方向における楕円体の有効半径 R_sys_dir の算出
%                 % (X/Y軸は b_sys, Z軸は a_sys)
%                 denom = sqrt((p_loc(1)/b_sys)^2 + (p_loc(2)/b_sys)^2 + (p_loc(3)/a_sys)^2);
%                 R_sys_dir = dist_center / denom;
% 
%                 % 厳密な表面間クリアランス
%                 d_surf = dist_center - (R_sys_dir + ro);
%             else
%                 d_surf = -(ro + min(a_sys, b_sys));
%             end
% 
%             if d_surf < min_surf_dist
%                 min_surf_dist = d_surf;
%             end
% 
%             % CBF 制約（CBF_Constraints_zlink3）の計算
%             [A_z_single, b_z_single] = CBF_Constraints_zlink3(obj, x, xd, vf, obs_params, gamma_params_z, sys_params_ellipsoid, P);
% 
%             A_i = -A_z_single;
%             B_i = -b_z_single;
% 
%             if A_i > 1e-9
%                 u1_lower_bound = max(u1_lower_bound, B_i / A_i);
%             elseif A_i < -1e-9
%                 u1_upper_bound = min(u1_upper_bound, B_i / A_i);
%             end
% 
%             log_p_obs{i}     = p_obs;
%             log_r_obs{i}     = ro;
%             log_r_minimal{i} = d_surf;
%         end
% 
%         u1_nominal = tmp(1);
%         u1_safe = max(u1_lower_bound, min(u1_upper_bound, u1_nominal));
%         u1_safe = max(0.0, min(20.0, u1_safe));
% 
%         tmp(1) = u1_safe;
% 
%         obj.result.p_mid         = p_mid;
%         obj.result.min_clearance = min_surf_dist; % 🌟 姿勢連動の正しいクリアランスが格納される
%         obj.result.p_obs         = log_p_obs;
%         obj.result.r_obs         = log_r_obs;
%         obj.result.r_minimal     = log_r_minimal;
%         obj.result.controllertime = toc(tic_start);
% 
%         obj.result.input = [max(0, min(20, tmp(1))); ...
%                             max(-1, min(1, tmp(2))); ...
%                             max(-1, min(1, tmp(3))); ...
%                             max(-1, min(1, tmp(4)))];
% 
%         obj.result.xd = xd;
%         obj.result.x = x;
%         result = obj.result;
%     end
% 
%     function show(obj)
%         obj.result
%     end
% end
% end

classdef HLC_SUSPENDED_LOAD_HOCBF_LOAD_ZXY_LINK_NEW < handle
    % クアッドコプター用階層型線形化（z解析的CBF ＋ xy実入力結合型HOCBF ＋ ケーブル角度制限付き）
    % 条件：障害物＝球体(円)、システム＝マルチセグメント球体被覆モデル（HOCBF版）
    % よけようとしてやるが、姿勢を考えないため、墜落
properties
    self
    result
    param
end

methods
    function obj = HLC_SUSPENDED_LOAD_HOCBF_LOAD_ZXY_LINK_NEW(self, param)
        obj.self = self;
        obj.param = param;
        % 初期化時のログ参照エラー防止（安全ガード）
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

        % 仮想入力のゲイン
        F1 = Param.F1; 
        F2 = Param.F2; 
        F3 = Param.F3; 
        F4 = Param.F4; 
        time_log = cell(1, 6);

        tic;
        time_log{1} = datetime('now', 'Format', 'HH:mm:ss.SSSSSS');
        vf = obj.Vfd_SuspendedLoadxyDst(Param.dt, x, xd', F1); 

        time_log{2} = datetime('now', 'Format', 'HH:mm:ss.SSSSSS');
        vs = obj.Vs_SuspendedLoadxyDst(x, xd', vf, P, F2, F3, F4); 

        time_log{3} = datetime('now', 'Format', 'HH:mm:ss.SSSSSS');
        uf = obj.Uf_SuspendedLoadxyDst(x, xd', vf, P); 

        time_log{4} = datetime('now', 'Format', 'HH:mm:ss.SSSSSS');
        beta2 = obj.Beta2_SuspendedLoadxyDst(x, xd', vf, P); 

        time_log{5} = datetime('now', 'Format', 'HH:mm:ss.SSSSSS');
        vs_alpha2 = obj.V2_alpha2_SuspendedLoadxyDst(x, xd', vf, vs', P); 

        time_log{6} = datetime('now', 'Format', 'HH:mm:ss.SSSSSS');
        us = beta2 \ vs_alpha2; 

        total_time = toc;

        if total_time * 1000 > 50 
            fprintf('\n🚨======== 制御フリーズ検出 (処理時間: %.2f ms) ========🚨\n', total_time * 1000);
            fprintf('1. vf 開始時     : %s\n', time_log{1});
            fprintf('2. vs 開始時     : %s\n', time_log{2});
            fprintf('3. uf 開始時     : %s\n', time_log{3});
            fprintf('4. beta 開始時   : %s\n', time_log{4});
            fprintf('5. alpha 開始時  : %s\n', time_log{5});
            fprintf('6. 左除算 開始時 : %s\n', time_log{6});
            fprintf('7. 全体終了時    : %s\n', datetime('now', 'Format', 'HH:mm:ss.SSSSSS'));
            fprintf('====================================================\n');
        end

        tmp = [uf(1); us]; % ノミナル入力 [u1_nom; u2_nom; u3_nom; u4_nom]
        obj.result.tmp = tmp; 

        %% =========================================================================
        %% 【ステップ 1】 マルチセグメント球体 HOCBF による解析的安全推力 u1_safe の算出
        %% =========================================================================
        min_surf_dist = Inf;
        obj.result.min_clearance = Inf; 

        obs_env = ENVIRONMENT_OBSTACLE_HOCBF_Z(); 
        num_obs = length(obs_env);

        pL = model.state.pL;  
        L_cable = P(7); 
        p_mid = pL - 0.5 * L_cable * pT; % ケーブル中央位置

        % 🌟 ケーブル・システムを被覆する球体モデルの設定
        % lambda_list: 荷物(0.0) 〜 ケーブル中央(0.5) 〜 機体(1.0) の位置比率
        % rl_list: 各球体の半径 (例: 荷物=0.20m, ケーブル中央=0.15m, 機体=0.35m)
        lambda_list = [0.0,  0.5,  1.0]; 
        % rl_list     = [0.20, 0.15, 0.35]; 
        rl_list     = [0.5, 0.5, 0.5];
        num_spheres = length(lambda_list);

        obj.result.lambda_list = lambda_list;
        obj.result.rl_list     = rl_list;

        log_p_obs     = cell(1, max(1, num_obs));
        log_r_obs     = cell(1, max(1, num_obs));
        log_r_minimal = cell(1, max(1, num_obs));

        % z方向 CBF 解析解用の上下限境界
        u1_lower_bound = 0.0;
        u1_upper_bound = 20.0;
        gamma_params_z = [1.0; 5.0]; 

        for i = 1:num_obs
            xo = obs_env(i).p_obs(1);
            yo = obs_env(i).p_obs(2);
            zo = obs_env(i).p_obs(3);
            ro = obs_env(i).r_obs;
            
            p_obs = [xo; yo; zo];
            obs_params = [xo; yo; zo; ro];

            obs_min_dist = Inf; % 障害物 i に対する最小クリアランス

            % システムを被覆する全ての球体 j についてループ評価
            for j = 1:num_spheres
                lambda_j = lambda_list(j);
                rl_j     = rl_list(j);
                sphere_params = [lambda_j; rl_j];

                % 第 j 被覆球体の中心位置 x_c^j
                x_c_j = pL - lambda_j * L_cable * pT;

                % 障害物（球体）表面までの距離計算（クリアランス）
                dist_center = norm(x_c_j - p_obs);
                d_surf      = dist_center - (ro + rl_j);

                if d_surf < min_surf_dist
                    min_surf_dist = d_surf;
                end
                if d_surf < obs_min_dist
                    obs_min_dist = d_surf;
                end

                % z方向 CBF 制約（A_z * u1 <= b_z）を導出関数から取得
                [A_z_single, b_z_single] = CBF_Constraints_zlink4(...
                    obj, x, xd, vf, obs_params, sphere_params, gamma_params_z, P);

                % A_i * u1 >= B_i に変換 (符号反転)
                A_i = -A_z_single;
                B_i = -b_z_single;

                if A_i > 1e-9
                    u1_lower_bound = max(u1_lower_bound, B_i / A_i);
                elseif A_i < -1e-9
                    u1_upper_bound = min(u1_upper_bound, B_i / A_i);
                end
            end

            log_p_obs{i}     = p_obs;
            log_r_obs{i}     = ro;
            log_r_minimal{i} = obs_min_dist;
        end

        u1_nominal = tmp(1);
        u1_safe    = max(u1_lower_bound, min(u1_upper_bound, u1_nominal));
        
        % ドローンの物理アクチュエータ限界(0N〜20N)に直接合わせる
        u1_safe    = max(0.0, min(20.0, u1_safe));

        % 確定した z 方向安全推力をセット
        tmp(1) = u1_safe;

        % 結果の格納
        obj.result.p_mid          = p_mid;
        obj.result.min_clearance  = min_surf_dist;
        obj.result.p_obs          = log_p_obs;
        obj.result.r_obs          = log_r_obs;
        obj.result.r_minimal      = log_r_minimal;
        obj.result.controllertime = toc(tic_start);

        obj.result.input = [max(0, min(20, tmp(1))); ... % u1 (解析的CBFで確定された推力)
                            max(-1, min(1, tmp(2))); ...  % u2 (安全化されたroll)
                            max(-1, min(1, tmp(3))); ...  % u3 (安全化されたpitch)
                            max(-1, min(1, tmp(4)))];    % u4 (yaw)

        obj.result.xd = xd;
        obj.result.x  = x;
        result        = obj.result;
    end

    function show(obj)
        obj.result
    end
end
end
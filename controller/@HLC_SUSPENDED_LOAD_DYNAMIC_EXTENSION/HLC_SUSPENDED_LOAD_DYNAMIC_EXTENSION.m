% classdef HLC_SUSPENDED_LOAD_DYNAMIC_EXTENSION < handle
% % クアッドコプター用階層型線形化を使った入力算出
% properties
%     self
%     result
%     param
% 
%     % FastBridge 推力差分カスケード用バッファ
%     u_prev          % 1ステップ前の確定入力 [u1; u2; u3; u4]
%     u_prev2         % 2ステップ前の確定入力 [u1; u2; u3; u4]
%     is_initialized  % バッファ初期化フラグ
% end
% 
% methods
% 
%     function obj = HLC_SUSPENDED_LOAD_DYNAMIC_EXTENSION(self, param)
%         obj.self  = self;
%         obj.param = param;
%         obj.result.min_clearance = Inf;
% 
%         % カスケードバッファの初期化
%         obj.u_prev         = [];
%         obj.u_prev2        = [];
%         obj.is_initialized = false;
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
%         P = [obj.self.parameter.get(["mass", "jx", "jy", "jz", "gravity", "loadmass", "cableL"]), 0, 0, 0];
%         x = [model.state.getq('compact'); model.state.w; pL; model.state.vL; pT; model.state.wL];
% 
%         % Yaw 角の誤差修正
%         yaw      = wrapToPi(model.state.q(3)); 
%         yawd     = xd(4); 
%         yawUnit  = [cos(yaw); sin(yaw); 0]; 
%         yawdUnit = [cos(yawd); sin(yawd); 0]; 
%         deltaYaw = sign(cross(yawdUnit, yawUnit)) * acos(yawdUnit' * yawUnit); 
%         xd(4)    = -deltaYaw(3) + yaw; 
%         xd       = [xd; zeros(28 - size(xd, 1), 1)];
% 
%         tic_start = tic;
% 
%         %% =========================================================================
%         %% 1. ノミナル制御入力の算定
%         %% =========================================================================
%         F1 = Param.F1; F2 = Param.F2; F3 = Param.F3; F4 = Param.F4; 
%         vf = obj.Vfd_SuspendedLoadxyDst(Param.dt, x, xd', F1); 
%         vs = obj.Vs_SuspendedLoadxyDst(x, xd', vf, P, F2, F3, F4); 
%         uf = obj.Uf_SuspendedLoadxyDst(x, xd', vf, P); 
%         beta2 = obj.Beta2_SuspendedLoadxyDst(x, xd', vf, P); 
%         vs_alpha2 = obj.V2_alpha2_SuspendedLoadxyDst(x, xd', vf, vs', P); 
%         us = beta2 \ vs_alpha2; 
%         u_raw = [uf(1); us]; % 生ノミナル入力
% 
%         lb_u = [0; -1; -1; -1];
%         ub_u = [20;  1;  1;  1];
%         u_nominal = u_raw;
%         obj.result.tmp = u_nominal;
% 
%         %% =========================================================================
%         %% 2. FastBridge: 推力微分の有限差分カスケード計算 (フィルタ・リミッター付き)
%         %% =========================================================================
%         dt = Param.dt;
%         if isempty(dt) || dt <= 0
%             dt = 0.025;
%         end
% 
%         if ~obj.is_initialized || isempty(obj.u_prev)
%             obj.u_prev         = u_nominal;
%             obj.u_prev2        = u_nominal;
%             obj.is_initialized = true;
%         end
% 
%         % 1. 生の差分計算
%         raw_du1  = (obj.u_prev(1) - obj.u_prev2(1)) / dt;
%         raw_d2u1 = (obj.u_prev(1) - 2 * obj.u_prev(1) + obj.u_prev2(1)) / (dt^2);
% 
%         % 2. 物理リミッター (モータ・機体の限界: 例 ±40 N/s, ±200 N/s^2)
%         max_du1  = 40.0;
%         max_d2u1 = 200.0;
%         du1_sat  = max(-max_du1,  min(max_du1,  raw_du1));
%         d2u1_sat = max(-max_d2u1, min(max_d2u1, raw_d2u1));
% 
%         % 3. 一次遅れローパスフィルタ (急激な数値微分ノイズをカット)
%         alpha_lpf = 0.2; % 0 < alpha <= 1 (小さいほど滑らか)
%         if ~isprop(obj, 'du1_filt') || isempty(obj.du1_filt)
%             obj.du1_filt  = 0;
%             obj.d2u1_filt = 0;
%         end
%         obj.du1_filt  = (1 - alpha_lpf) * obj.du1_filt  + alpha_lpf * du1_sat;
%         obj.d2u1_filt = (1 - alpha_lpf) * obj.d2u1_filt + alpha_lpf * d2u1_sat;
% 
%         cascade_vars = [obj.du1_filt; obj.d2u1_filt];
% 
%         %% =========================================================================
%         %% 3. 幾何情報・クリアランス計算
%         %% =========================================================================
%         min_surf_dist = Inf;
%         obs_env = ENVIRONMENT_OBSTACLE_HOCBF_LINK_XY();
%         num_obs = length(obs_env);
%         L_cable = P(7); 
%         p_mid   = pL - 0.5 * L_cable * pT;
%         rl_sys  = 1; % 紐・荷物半径マージン
%         obj.result.rl    = rl_sys;
%         obj.result.p_mid = p_mid;
% 
%         log_p_obs        = cell(1, max(1, num_obs));
%         log_r_obs_margin = cell(1, max(1, num_obs));
%         log_r_minimal    = cell(1, max(1, num_obs));
% 
%         for i = 1:num_obs
%             p_obs = obs_env(i).p_obs(:);
%             ro_margin = obs_env(i).r_obs_margin;
%             d_surf = norm(p_mid - p_obs) - (ro_margin + rl_sys);
%             if d_surf < min_surf_dist
%                 min_surf_dist = d_surf;
%             end
%             log_p_obs{i}        = p_obs;
%             log_r_obs_margin{i} = ro_margin;
%             log_r_minimal{i}    = d_surf;
%         end
%         obj.result.min_clearance = min_surf_dist;
%         obj.result.p_obs         = log_p_obs;
%         obj.result.r_minimal     = log_r_minimal;
% 
%         %% =========================================================================
%         %% 4. FastBridge ECBF 制約行列の構築
%         %% =========================================================================
%         % --- [A] 障害物回避 4次 ECBF (FastBridge 式 (15) 準拠) ---
%         lambda_cbf = 3; % Hurwitz 4重極ゲイン
%         A_hard = zeros(num_obs, 4);
%         b_hard = zeros(num_obs, 1);
%         A_soft = zeros(num_obs, 4);
%         b_soft = zeros(num_obs, 1);
%         log_h_obs = zeros(num_obs, 4);
% 
%         for i = 1:num_obs
%             p_obs = obs_env(i).p_obs(:);
% 
%             % 1) ハード制約: マージンなし (r_obs)
%             ro_hard = obs_env(i).r_obs;
%             obs_params_hard = [p_obs; ro_hard];
%             [A_h, b_h, ~, ~, ~, ~] = FastBridge_ECBF_SlungLoad_MidPoint(...
%                 obj, x, xd, cascade_vars, obs_params_hard, lambda_cbf, rl_sys, P);
%             A_hard(i, :) = real(A_h);
%             b_hard(i)    = real(b_h);
% 
%             % 2) ソフト制約: マージンあり (r_obs_margin)
%             ro_soft = obs_env(i).r_obs_margin;
%             obs_params_soft = [p_obs; ro_soft];
%             [A_s, b_s, h0_v, h1_v, h2_v, h3_v] = FastBridge_ECBF_SlungLoad_MidPoint(...
%                 obj, x, xd, cascade_vars, obs_params_soft, lambda_cbf, rl_sys, P);
%             A_soft(i, :) = real(A_s);
%             b_soft(i)    = real(b_s);
%             log_h_obs(i, :) = [h0_v, h1_v, h2_v, h3_v];
%         end
% 
%         % --- [B] 機体姿勢角 ＆ 紐傾斜角 CBF (2階微分) ---
%         u0_k = u_nominal;
%         ang_limits = [deg2rad(35); deg2rad(35)];
%         gamma_ang  = [5.0; 10.0; 5.0; 10.0];
%         [A_att, b_att, h_att, A_cable, b_cable, h_cable] = CBF_Constraints_Attitude_Cable_Taylor(...
%             obj, x, xd, u0_k, ang_limits, gamma_ang, rl_sys, P);
%         A_att   = real(A_att);   b_att   = real(b_att);
%         A_cable = real(A_cable); b_cable = real(b_cable);
% 
%         %% =========================================================================
%         %% 5. ソフト制約付き単一 QP の構築と求解
%         %% =========================================================================
%         n_u   = 4;
%         n_obs = num_obs;
%         n_ang = 2; 
%         n_in  = 4; 
%         n_vars = n_u + n_obs + n_ang + 2 * n_in;
% 
%         W_u = diag([10, 0.01, 0.01, 2.0]);
%         rho_obs_quad = 1e2;  rho_obs_lin = 1e4;
%         rho_ang_quad = 1e2;  rho_ang_lin = 1e4;
%         rho_in_quad  = 1e3;  rho_in_lin  = 1e5;
% 
%         H_qp = blkdiag(W_u, ...
%                        rho_obs_quad * eye(n_obs), ...
%                        rho_ang_quad * eye(n_ang), ...
%                        rho_in_quad  * eye(n_in), ...
%                        rho_in_quad  * eye(n_in));
%         f_qp = [-W_u * u_nominal; ...
%                 rho_obs_lin * ones(n_obs, 1); ...
%                 rho_ang_lin * ones(n_ang, 1); ...
%                 rho_in_lin  * ones(n_in, 1); ...
%                 rho_in_lin  * ones(n_in, 1)];
% 
%         A_ineq = [
%             A_hard,    zeros(n_obs, n_obs), zeros(n_obs, n_ang), zeros(n_obs, n_in), zeros(n_obs, n_in); ...
%             A_soft,   -eye(n_obs),          zeros(n_obs, n_ang), zeros(n_obs, n_in), zeros(n_obs, n_in); ...
%             A_att,     zeros(1, n_obs),    -1,  0,               zeros(1, n_in),     zeros(1, n_in);     ...
%             A_cable,   zeros(1, n_obs),     0, -1,               zeros(1, n_in),     zeros(1, n_in);     ...
%             -eye(n_u), zeros(n_u, n_obs),   zeros(n_u, n_ang),  -eye(n_in),          zeros(n_u, n_in);   ...
%              eye(n_u), zeros(n_u, n_obs),   zeros(n_u, n_ang),   zeros(n_u, n_in),  -eye(n_in)           ...
%         ];
%         b_ineq = [b_hard; b_soft; b_att; b_cable; -lb_u; ub_u];
% 
%         lb_qp = [-Inf(n_u, 1); zeros(n_obs + n_ang + 2 * n_in, 1)];
%         ub_qp = Inf(n_vars, 1);
% 
%         options_qp = optimoptions('quadprog', 'Display', 'off', 'ConstraintTolerance', 1e-4);
%         [z_opt, ~, exitflag] = quadprog(H_qp, f_qp, A_ineq, b_ineq, [], [], lb_qp, ub_qp, [], options_qp);
% 
%         if (exitflag == 1 || exitflag == 2) && ~isempty(z_opt)
%             u_opt = z_opt(1:4);
%         else
%             u_opt = u_nominal;
%         end
% 
%         u_final = max(lb_u, min(ub_u, u_opt));
% 
%         %% =========================================================================
%         %% 6. カスケードバッファの更新 (次回ステップ用)
%         %% =========================================================================
%         obj.u_prev2 = obj.u_prev;
%         obj.u_prev  = u_final;
% 
%         %% =========================================================================
%         %% 7. ロギング ＆ 結果格納
%         %% =========================================================================
%         slack_nom  = A_soft * u_nominal - b_soft;
%         slack_safe = b_soft - A_soft * u_final;
%         num_violated = sum(slack_nom > 0);
% 
%         if num_violated > 0
%             fprintf('[FastBridge Cascade ECBF] 介入発生 (接近数: %d)\n', num_violated);
%             fprintf('  du1_est = %+.3f, d2u1_est = %+.3f\n', du1_est, d2u1_est);
%             fprintf('  Nominal : [u1=%.2f, u2=%.3f, u3=%.3f, u4=%.3f]\n', ...
%                 u_nominal(1), u_nominal(2), u_nominal(3), u_nominal(4));
%             fprintf('  Safe    : [u1=%.2f, u2=%.3f, u3=%.3f, u4=%.3f]\n', ...
%                 u_final(1), u_final(2), u_final(3), u_final(4));
%         end
% 
%         obj.result.A_qp_all        = A_soft;
%         obj.result.b_qp_all        = b_soft;
%         obj.result.A_hard          = A_hard;
%         obj.result.b_hard          = b_hard;
%         obj.result.A_att           = A_att;
%         obj.result.b_att           = b_att;
%         obj.result.h_att           = h_att;
%         obj.result.A_cable         = A_cable;
%         obj.result.b_cable         = b_cable;
%         obj.result.h_cable         = h_cable;
%         obj.result.slack_nom       = slack_nom;
%         obj.result.slack_safe      = slack_safe;
%         obj.result.num_violated    = num_violated;
%         obj.result.log_h_obs       = log_h_obs;
%         obj.result.u_nominal       = u_nominal;
%         obj.result.tmp_fix         = u_final;
%         obj.result.cascade_vars    = cascade_vars;
%         obj.result.controllertime  = toc(tic_start);
% 
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

% classdef HLC_SUSPENDED_LOAD_DYNAMIC_EXTENSION < handle
% % クアッドコプター用階層型線形化を使った入力算出 (FastBridge Cascade ECBF 安定化版)
% properties
%     self
%     result
%     param
%     % FastBridge 推力差分カスケード用バッファ
%     u_prev          % 1ステップ前の確定入力 [u1; u2; u3; u4]
%     u_prev2         % 2ステップ前の確定入力 [u1; u2; u3; u4]
%     is_initialized  % バッファ初期化フラグ
%     du1_filt        % 🌟 推力1階差分フィルタ値
%     d2u1_filt       % 🌟 推力2階差分フィルタ値
% end
% methods
%     function obj = HLC_SUSPENDED_LOAD_DYNAMIC_EXTENSION(self, param)
%         obj.self  = self;
%         obj.param = param;
%         obj.result.min_clearance = Inf;
% 
%         % カスケードバッファの初期化
%         obj.u_prev         = [];
%         obj.u_prev2        = [];
%         obj.is_initialized = false;
%         obj.du1_filt       = 0;
%         obj.d2u1_filt      = 0;
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
%         P = [obj.self.parameter.get(["mass", "jx", "jy", "jz", "gravity", "loadmass", "cableL"]), 0, 0, 0];
%         x = [model.state.getq('compact'); model.state.w; pL; model.state.vL; pT; model.state.wL];
% 
%         % Yaw 角の誤差修正
%         yaw      = wrapToPi(model.state.q(3)); 
%         yawd     = xd(4); 
%         yawUnit  = [cos(yaw); sin(yaw); 0]; 
%         yawdUnit = [cos(yawd); sin(yawd); 0]; 
%         deltaYaw = sign(cross(yawdUnit, yawUnit)) * acos(yawdUnit' * yawUnit); 
%         xd(4)    = -deltaYaw(3) + yaw; 
%         xd       = [xd; zeros(28 - size(xd, 1), 1)];
% 
%         tic_start = tic;
% 
%         %% =========================================================================
%         %% 1. ノミナル制御入力の算定
%         %% =========================================================================
%         F1 = Param.F1; F2 = Param.F2; F3 = Param.F3; F4 = Param.F4; 
%         vf = obj.Vfd_SuspendedLoadxyDst(Param.dt, x, xd', F1); 
%         vs = obj.Vs_SuspendedLoadxyDst(x, xd', vf, P, F2, F3, F4); 
%         uf = obj.Uf_SuspendedLoadxyDst(x, xd', vf, P); 
%         beta2 = obj.Beta2_SuspendedLoadxyDst(x, xd', vf, P); 
%         vs_alpha2 = obj.V2_alpha2_SuspendedLoadxyDst(x, xd', vf, vs', P); 
%         us = beta2 \ vs_alpha2; 
%         u_raw = [uf(1); us]; 
% 
%         lb_u = [0; -1; -1; -1];
%         ub_u = [20;  1;  1;  1];
%         u_nominal = u_raw;
%         obj.result.tmp = u_nominal;
% 
%         %% =========================================================================
%         %% 2. FastBridge: 推力微分の有限差分カスケード計算 (LPF & リミッター)
%         %% =========================================================================
%         dt = Param.dt;
%         if isempty(dt) || dt <= 0
%             dt = 0.025;
%         end
% 
%         if ~obj.is_initialized || isempty(obj.u_prev)
%             obj.u_prev         = u_nominal;
%             obj.u_prev2        = u_nominal;
%             obj.du1_filt       = 0;
%             obj.d2u1_filt      = 0;
%             obj.is_initialized = true;
%         end
% 
%         % 1. 生差分計算
%         raw_du1  = (obj.u_prev(1) - obj.u_prev2(1)) / dt;
%         raw_d2u1 = (obj.u_prev(1) - 2 * obj.u_prev(1) + obj.u_prev2(1)) / (dt^2);
% 
%         % 2. 物理リミッター (実機推力変化率の上下限に制限)
%         max_du1  = 25.0;   % 最大 ±25 N/s
%         max_d2u1 = 100.0;  % 最大 ±100 N/s^2
%         du1_sat  = max(-max_du1,  min(max_du1,  raw_du1));
%         d2u1_sat = max(-max_d2u1, min(max_d2u1, raw_d2u1));
% 
%         % 3. 一次遅れローパスフィルタ (確実な保持)
%         alpha_lpf = 0.15; 
%         obj.du1_filt  = (1 - alpha_lpf) * obj.du1_filt  + alpha_lpf * du1_sat;
%         obj.d2u1_filt = (1 - alpha_lpf) * obj.d2u1_filt + alpha_lpf * d2u1_sat;
% 
%         cascade_vars = [obj.du1_filt; obj.d2u1_filt];
% 
%         %% =========================================================================
%         %% 3. 幾何情報・クリアランス計算
%         %% =========================================================================
%         min_surf_dist = Inf;
%         obs_env = ENVIRONMENT_OBSTACLE_HOCBF_LINK_XY();
%         num_obs = length(obs_env);
%         L_cable = P(7); 
%         p_mid   = pL - 0.5 * L_cable * pT;
%         rl_sys  = 1; 
%         obj.result.rl    = rl_sys;
%         obj.result.p_mid = p_mid;
% 
%         log_p_obs        = cell(1, max(1, num_obs));
%         log_r_obs_margin = cell(1, max(1, num_obs));
%         log_r_minimal    = cell(1, max(1, num_obs));
% 
%         for i = 1:num_obs
%             p_obs = obs_env(i).p_obs(:);
%             ro_margin = obs_env(i).r_obs_margin;
%             d_surf = norm(p_mid - p_obs) - (ro_margin + rl_sys);
%             if d_surf < min_surf_dist
%                 min_surf_dist = d_surf;
%             end
%             log_p_obs{i}        = p_obs;
%             log_r_obs_margin{i} = ro_margin;
%             log_r_minimal{i}    = d_surf;
%         end
%         obj.result.min_clearance = min_surf_dist;
%         obj.result.p_obs         = log_p_obs;
%         obj.result.r_minimal     = log_r_minimal;
% 
%         %% =========================================================================
%         %% 4. FastBridge ECBF 制約行列の構築 (Distance & Approach Gate 付き)
%         %% =========================================================================
%         lambda_cbf = 1.2; 
%         A_hard = [];
%         b_hard = [];
%         A_soft = [];
%         b_soft = [];
%         log_h_obs = [];
% 
%         v_mid = model.state.vL; % 中点の代表速度 (接近判定用)
%         active_obs_count = 0;
% 
%         for i = 1:num_obs
%             p_obs = obs_env(i).p_obs(:);
%             ro_margin = obs_env(i).r_obs_margin;
% 
%             % 🌟 1. Distance Gate (論文 Sec. IV-C): 障害物が遠すぎる場合はスキップ
%             dist_to_obs = norm(p_mid - p_obs);
%             detection_radius = ro_margin + rl_sys + 2.5; % 検知範囲: 障害物半径+2.5m
%             if dist_to_obs > detection_radius
%                 continue; % 遠い障害物は制約に含めない
%             end
% 
%             % 🌟 2. Approach Gate (論文 式(9) & Sec. IV-C): 遠ざかっている場合はスキップ
%             % 相対位置ベクトル (機体から障害物へ向かうベクトル)
%             r_rel = p_obs - p_mid;
%             % 速度ベクトルとの内積が負（障害物から離れる運動）なら除外
%             if dot(r_rel, v_mid) < -0.1 && dist_to_obs > (ro_margin + rl_sys + 0.3)
%                 continue; 
%             end
% 
%             active_obs_count = active_obs_count + 1;
% 
%             % --- 1) ハード制約 ---
%             ro_hard = obs_env(i).r_obs;
%             obs_params_hard = [p_obs; ro_hard];
%             [A_h, b_h, ~, ~, ~, ~] = FastBridge_ECBF_SlungLoad_MidPoint(...
%                 obj, x, xd, cascade_vars, obs_params_hard, lambda_cbf, rl_sys, P);
%             A_hard = [A_hard; real(A_h)];
%             b_hard = [b_hard; real(b_h)];
% 
%             % --- 2) ソフト制約 ---
%             ro_soft = ro_margin;
%             obs_params_soft = [p_obs; ro_soft];
%             [A_s, b_s, h0_v, h1_v, h2_v, h3_v] = FastBridge_ECBF_SlungLoad_MidPoint(...
%                 obj, x, xd, cascade_vars, obs_params_soft, lambda_cbf, rl_sys, P);
%             A_soft = [A_soft; real(A_s)];
%             b_soft = [b_soft; real(b_s)];
%             log_h_obs = [log_h_obs; [h0_v, h1_v, h2_v, h3_v]];
%         end
% 
%         n_obs = active_obs_count; % 有効な障害物数に更新
% 
%         %% =========================================================================
%         %% 5. ソフト制約付き単一 QP の構築と求解 (推力変化率抑制付き)
%         %% =========================================================================
%         n_u   = 4;
%         n_obs = num_obs;
%         n_ang = 2; 
%         n_in  = 4; 
%         n_vars = n_u + n_obs + n_ang + 2 * n_in;
% 
%         % 🌟 推力変化を滑らかにする正則化
%         w_thrust_nom  = 1.0;   % ノミナル追従重み
%         w_thrust_rate = 2.0;   % 前回入力追従（急変抑制）重み
%         W_u = diag([w_thrust_nom, 0.05, 0.05, 1.0]);
% 
%         rho_obs_quad = 1e2;  rho_obs_lin = 1e4;
%         rho_ang_quad = 1e2;  rho_ang_lin = 1e4;
%         rho_in_quad  = 1e3;  rho_in_lin  = 1e5;
% 
%         H_u = W_u + diag([w_thrust_rate, 0, 0, 0]);
%         H_qp = blkdiag(H_u, ...
%                        rho_obs_quad * eye(n_obs), ...
%                        rho_ang_quad * eye(n_ang), ...
%                        rho_in_quad  * eye(n_in), ...
%                        rho_in_quad  * eye(n_in));
% 
%         f_u  = - (W_u * u_nominal + [w_thrust_rate * obj.u_prev(1); 0; 0; 0]);
%         f_qp = [f_u; ...
%                 rho_obs_lin * ones(n_obs, 1); ...
%                 rho_ang_lin * ones(n_ang, 1); ...
%                 rho_in_lin  * ones(n_in, 1); ...
%                 rho_in_lin  * ones(n_in, 1)];
% 
%         A_ineq = [
%             A_hard,    zeros(n_obs, n_obs), zeros(n_obs, n_ang), zeros(n_obs, n_in), zeros(n_obs, n_in); ...
%             A_soft,   -eye(n_obs),          zeros(n_obs, n_ang), zeros(n_obs, n_in), zeros(n_obs, n_in); ...
%             A_att,     zeros(1, n_obs),    -1,  0,               zeros(1, n_in),     zeros(1, n_in);     ...
%             A_cable,   zeros(1, n_obs),     0, -1,               zeros(1, n_in),     zeros(1, n_in);     ...
%             -eye(n_u), zeros(n_u, n_obs),   zeros(n_u, n_ang),  -eye(n_in),          zeros(n_u, n_in);   ...
%              eye(n_u), zeros(n_u, n_obs),   zeros(n_u, n_ang),   zeros(n_u, n_in),  -eye(n_in)           ...
%         ];
%         b_ineq = [b_hard; b_soft; b_att; b_cable; -lb_u; ub_u];
% 
%         lb_qp = [-Inf(n_u, 1); zeros(n_obs + n_ang + 2 * n_in, 1)];
%         ub_qp = Inf(n_vars, 1);
% 
%         options_qp = optimoptions('quadprog', 'Display', 'off', 'ConstraintTolerance', 1e-4);
%         [z_opt, ~, exitflag] = quadprog(H_qp, f_qp, A_ineq, b_ineq, [], [], lb_qp, ub_qp, [], options_qp);
% 
%         if (exitflag == 1 || exitflag == 2) && ~isempty(z_opt)
%             u_opt = z_opt(1:4);
%         else
%             u_opt = u_nominal;
%         end
% 
%         u_final = max(lb_u, min(ub_u, u_opt));
% 
%         %% =========================================================================
%         %% 6. カスケードバッファの更新
%         %% =========================================================================
%         obj.u_prev2 = obj.u_prev;
%         obj.u_prev  = u_final;
% 
%         %% =========================================================================
%         %% 7. ロギング ＆ 結果格納
%         %% =========================================================================
%         slack_nom  = A_soft * u_nominal - b_soft;
%         slack_safe = b_soft - A_soft * u_final;
%         num_violated = sum(slack_nom > 0);
% 
%         if num_violated > 0
%             fprintf('[FastBridge Cascade ECBF] 介入 (接近数: %d)\n', num_violated);
%             fprintf('  du1_filt = %+.3f, d2u1_filt = %+.3f\n', obj.du1_filt, obj.d2u1_filt);
%             fprintf('  Nominal : [u1=%.2f, u2=%.3f, u3=%.3f, u4=%.3f]\n', ...
%                 u_nominal(1), u_nominal(2), u_nominal(3), u_nominal(4));
%             fprintf('  Safe    : [u1=%.2f, u2=%.3f, u3=%.3f, u4=%.3f]\n', ...
%                 u_final(1), u_final(2), u_final(3), u_final(4));
%         end
% 
%         obj.result.A_qp_all        = A_soft;
%         obj.result.b_qp_all        = b_soft;
%         obj.result.A_hard          = A_hard;
%         obj.result.b_hard          = b_hard;
%         obj.result.A_att           = A_att;
%         obj.result.b_att           = b_att;
%         obj.result.h_att           = h_att;
%         obj.result.A_cable         = A_cable;
%         obj.result.b_cable         = b_cable;
%         obj.result.h_cable         = h_cable;
%         obj.result.slack_nom       = slack_nom;
%         obj.result.slack_safe      = slack_safe;
%         obj.result.num_violated    = num_violated;
%         obj.result.log_h_obs       = log_h_obs;
%         obj.result.u_nominal       = u_nominal;
%         obj.result.tmp_fix         = u_final;
%         obj.result.cascade_vars    = cascade_vars;
%         obj.result.controllertime  = toc(tic_start);
% 
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

classdef HLC_SUSPENDED_LOAD_DYNAMIC_EXTENSION < handle
% クアッドコプター用階層型線形化を使った入力算出
% (階層的ハード/ソフトCBFスラック分離・推力下限保護・トルクレンジ制限版)
properties
    self
    result
    param
    
    % FastBridge 推力差分カスケード用バッファ
    u_prev          % 1ステップ前の確定入力 [u1; u2; u3; u4]
    u_prev2         % 2ステップ前の確定入力 [u1; u2; u3; u4]
    is_initialized  % バッファ初期化フラグ
    du1_filt        % 推力1階差分フィルタ値
    d2u1_filt       % 推力2階差分フィルタ値
end

methods
    function obj = HLC_SUSPENDED_LOAD_DYNAMIC_EXTENSION(self, param)
        obj.self  = self;
        obj.param = param;
        obj.result.min_clearance = Inf;
        
        obj.u_prev         = [];
        obj.u_prev2        = [];
        obj.is_initialized = false;
        obj.du1_filt       = 0;
        obj.d2u1_filt      = 0;
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
        
        P = [obj.self.parameter.get(["mass", "jx", "jy", "jz", "gravity", "loadmass", "cableL"]), 0, 0, 0];
        x = [model.state.getq('compact'); model.state.w; pL; model.state.vL; pT; model.state.wL];
        
        % Yaw 角の誤差修正
        yaw      = wrapToPi(model.state.q(3)); 
        yawd     = xd(4); 
        yawUnit  = [cos(yaw); sin(yaw); 0]; 
        yawdUnit = [cos(yawd); sin(yawd); 0]; 
        deltaYaw = sign(cross(yawdUnit, yawUnit)) * acos(yawdUnit' * yawUnit); 
        xd(4)    = -deltaYaw(3) + yaw; 
        xd       = [xd; zeros(28 - size(xd, 1), 1)];
        
        tic_start = tic;
        
        %% =========================================================================
        %% 1. ノミナル制御入力の算定
        %% =========================================================================
        F1 = Param.F1; F2 = Param.F2; F3 = Param.F3; F4 = Param.F4; 
        vf = obj.Vfd_SuspendedLoadxyDst(Param.dt, x, xd', F1); 
        vs = obj.Vs_SuspendedLoadxyDst(x, xd', vf, P, F2, F3, F4); 
        uf = obj.Uf_SuspendedLoadxyDst(x, xd', vf, P); 
        beta2 = obj.Beta2_SuspendedLoadxyDst(x, xd', vf, P); 
        vs_alpha2 = obj.V2_alpha2_SuspendedLoadxyDst(x, xd', vf, vs', P); 
        us = beta2 \ vs_alpha2; 
        u_raw = [uf(1); us]; 
        
        % 物理ハードリミット (最終段のクリッピングで使用)
        lb_phys = [0; -1.0; -1.0; -1.0];
        ub_phys = [20;  1.0;  1.0;  1.0];
        
        u_nominal = u_raw;
        obj.result.tmp = u_nominal;
        
        %% =========================================================================
        %% 2. FastBridge: 推力微分の有限差分カスケード計算 (LPF & リミッター)
        %% =========================================================================
        dt = Param.dt;
        if isempty(dt) || dt <= 0
            dt = 0.025;
        end
        
        if ~obj.is_initialized || isempty(obj.u_prev)
            obj.u_prev         = u_nominal;
            obj.u_prev2        = u_nominal;
            obj.du1_filt       = 0;
            obj.d2u1_filt      = 0;
            obj.is_initialized = true;
        end
        
        raw_du1  = (obj.u_prev(1) - obj.u_prev2(1)) / dt;
        raw_d2u1 = (obj.u_prev(1) - 2 * obj.u_prev(1) + obj.u_prev2(1)) / (dt^2);
        
        max_du1  = 25.0;   
        max_d2u1 = 100.0;  
        du1_sat  = max(-max_du1,  min(max_du1,  raw_du1));
        d2u1_sat = max(-max_d2u1, min(max_d2u1, raw_d2u1));
        
        alpha_lpf = 0.15; 
        obj.du1_filt  = (1 - alpha_lpf) * obj.du1_filt  + alpha_lpf * du1_sat;
        obj.d2u1_filt = (1 - alpha_lpf) * obj.d2u1_filt + alpha_lpf * d2u1_sat;
        
        cascade_vars = [obj.du1_filt; obj.d2u1_filt; u_nominal(1)];
        
        %% =========================================================================
        %% 3. 幾何情報・クリアランス計算
        %% =========================================================================
        min_surf_dist = Inf;
        obs_env = ENVIRONMENT_OBSTACLE_HOCBF_LINK_XY();
        num_obs_total = length(obs_env);
        L_cable = P(7); 
        p_mid   = pL - 0.5 * L_cable * pT;
        
        rl_sys  = 0.45; 
        obj.result.rl    = rl_sys;
        obj.result.p_mid = p_mid;
        
        log_p_obs        = cell(1, max(1, num_obs_total));
        log_r_obs_margin = cell(1, max(1, num_obs_total));
        log_r_minimal    = cell(1, max(1, num_obs_total));
        
        for i = 1:num_obs_total
            p_obs = obs_env(i).p_obs(:);
            ro_margin = obs_env(i).r_obs_margin;
            d_surf = norm(p_mid - p_obs) - (ro_margin + rl_sys);
            if d_surf < min_surf_dist
                min_surf_dist = d_surf;
            end
            log_p_obs{i}        = p_obs;
            log_r_obs_margin{i} = ro_margin;
            log_r_minimal{i}    = d_surf;
        end
        obj.result.min_clearance = min_surf_dist;
        obj.result.p_obs         = log_p_obs;
        obj.result.r_minimal     = log_r_minimal;
        
        %% =========================================================================
        %% 4. FastBridge ECBF 制約行列の構築 (障害物・姿勢・紐)
        %% =========================================================================
        k0 = 2.0;
        k1 = 4.0;
        k2 = 8.0;
        k3 = 4.0;
        cbf_gains = [k0; k1; k2; k3];
        
        A_hard = zeros(0, 4);  b_hard = zeros(0, 1);
        A_soft = zeros(0, 4);  b_soft = zeros(0, 1);
        log_h_obs = zeros(0, 4);
        
        for i = 1:num_obs_total
            p_obs = obs_env(i).p_obs(:);
            ro_margin = obs_env(i).r_obs_margin;
            ro_hard   = obs_env(i).r_obs;
            
            % 水平面(XY)での距離判定（高度差で障害物を見失う欠陥を防止）
            dist_xy = norm(p_mid(1:2) - p_obs(1:2));
            if dist_xy > (ro_margin + rl_sys + 3.0)
                continue;
            end
            
            % 1) ハード制約
            obs_params_hard = [p_obs; ro_hard];
            [A_h, b_h, ~, ~, ~, ~] = FastBridge_ECBF_SlungLoad_MidPoint(...
                obj, x, xd, cascade_vars, obs_params_hard, cbf_gains, rl_sys, P);
            A_hard = [A_hard; real(A_h)];
            b_hard = [b_hard; real(b_h)];
            
            % 2) ソフト制約
            obs_params_soft = [p_obs; ro_margin];
            [A_s, b_s, h0_v, h1_v, h2_v, h3_v] = FastBridge_ECBF_SlungLoad_MidPoint(...
                obj, x, xd, cascade_vars, obs_params_soft, cbf_gains, rl_sys, P);
            A_soft = [A_soft; real(A_s)];
            b_soft = [b_soft; real(b_s)];
            log_h_obs = [log_h_obs; [h0_v, h1_v, h2_v, h3_v]];
        end
        
        n_obs = size(A_soft, 1);
        
        % 🌟【復元】姿勢角 ＆ 紐傾斜角 CBF 制約の算出
        u0_k = u_nominal;
        ang_limits = [deg2rad(35); deg2rad(35)];
        gamma_ang  = [3.0; 6.0; 3.0; 6.0];
        [A_att, b_att, h_att, A_cable, b_cable, h_cable] = CBF_Constraints_Attitude_Cable_Taylor(...
            obj, x, xd, u0_k, ang_limits, gamma_ang, rl_sys, P);
        A_att   = real(A_att);   b_att   = real(b_att);
        A_cable = real(A_cable); b_cable = real(b_cable);
        
        %% =========================================================================
        %% 5. ソフト/ハード制約付き単一 QP の構築と求解
        %% =========================================================================
        n_u   = 4;
        n_ang = 2; 
        n_in  = 4; 
        n_vars = n_u + 2 * n_obs + n_ang + 2 * n_in;
        
        % QP 内部での入力安全レンジ
        lb_qp_u = [4.0;  -0.35; -0.35; -0.8];
        ub_qp_u = [15.0;  0.35;  0.35;  0.8];
        
        W_u_nom  = diag([1.0, 0.005, 0.005, 1.0]);
        W_u_rate = diag([2.0, 0.05,  0.05,  0.1]); 
        H_u = W_u_nom + W_u_rate;
        f_u = - (W_u_nom * u_nominal + W_u_rate * obj.u_prev);
        
        % スラック変数ペナルティ
        rho_hard_quad = 1e6;  rho_hard_lin = 1e8;
        rho_soft_quad = 1e5;  rho_soft_lin = 1e7;
        rho_ang_quad  = 1e4;  rho_ang_lin  = 1e6;
        rho_in_quad   = 1e4;  rho_in_lin   = 1e6;
        
        if n_obs > 0
            H_qp = blkdiag(H_u, ...
                           rho_hard_quad * eye(n_obs), ...
                           rho_soft_quad * eye(n_obs), ...
                           rho_ang_quad  * eye(n_ang), ...
                           rho_in_quad   * eye(n_in), ...
                           rho_in_quad   * eye(n_in));
            f_qp = [f_u; ...
                    rho_hard_lin * ones(n_obs, 1); ...
                    rho_soft_lin * ones(n_obs, 1); ...
                    rho_ang_lin  * ones(n_ang, 1); ...
                    rho_in_lin   * ones(n_in, 1); ...
                    rho_in_lin   * ones(n_in, 1)];
            
            A_ineq = [
                A_hard,    -eye(n_obs),          zeros(n_obs, n_obs), zeros(n_obs, n_ang), zeros(n_obs, n_in), zeros(n_obs, n_in); ...
                A_soft,     zeros(n_obs, n_obs), -eye(n_obs),          zeros(n_obs, n_ang), zeros(n_obs, n_in), zeros(n_obs, n_in); ...
                A_att,      zeros(1, n_obs),     zeros(1, n_obs),     -1,  0,               zeros(1, n_in),     zeros(1, n_in);     ...
                A_cable,    zeros(1, n_obs),     zeros(1, n_obs),      0, -1,               zeros(1, n_in),     zeros(1, n_in);     ...
                -eye(n_u),  zeros(n_u, n_obs),   zeros(n_u, n_obs),   zeros(n_u, n_ang),  -eye(n_in),          zeros(n_u, n_in);   ...
                 eye(n_u),  zeros(n_u, n_obs),   zeros(n_u, n_obs),   zeros(n_u, n_ang),   zeros(n_u, n_in),  -eye(n_in)           ...
            ];
            b_ineq = [b_hard; b_soft; b_att; b_cable; -lb_qp_u; ub_qp_u];
            lb_qp = [-Inf(n_u, 1); zeros(2 * n_obs + n_ang + 2 * n_in, 1)];
            ub_qp = Inf(n_vars, 1);
        else
            H_qp = blkdiag(H_u, ...
                           rho_ang_quad * eye(n_ang), ...
                           rho_in_quad  * eye(n_in), ...
                           rho_in_quad  * eye(n_in));
            f_qp = [f_u; ...
                    rho_ang_lin * ones(n_ang, 1); ...
                    rho_in_lin  * ones(n_in, 1); ...
                    rho_in_lin  * ones(n_in, 1)];
            
            A_ineq = [
                A_att,    -1,  0,               zeros(1, n_in),     zeros(1, n_in);     ...
                A_cable,   0, -1,               zeros(1, n_in),     zeros(1, n_in);     ...
                -eye(n_u), zeros(n_u, n_ang),  -eye(n_in),          zeros(n_u, n_in);   ...
                 eye(n_u), zeros(n_u, n_ang),   zeros(n_u, n_in),  -eye(n_in)           ...
            ];
            b_ineq = [b_att; b_cable; -lb_qp_u; ub_qp_u];
            lb_qp = [-Inf(n_u, 1); zeros(n_ang + 2 * n_in, 1)];
            ub_qp = Inf(n_vars, 1);
        end
        
        options_qp = optimoptions('quadprog', 'Display', 'off', 'ConstraintTolerance', 1e-4);
        [z_opt, ~, exitflag] = quadprog(H_qp, f_qp, A_ineq, b_ineq, [], [], lb_qp, ub_qp, [], options_qp);
        
        if (exitflag == 1 || exitflag == 2) && ~isempty(z_opt)
            u_opt = z_opt(1:4);
        else
            u_opt = u_nominal;
        end
        
        % 最終段の物理限界クリッピング
        u_final = max(lb_phys, min(ub_phys, u_opt));
        
        %% =========================================================================
        %% 6. カスケードバッファの更新
        %% =========================================================================
        obj.u_prev2 = obj.u_prev;
        obj.u_prev  = u_final;
        
        %% =========================================================================
        %% 7. 介入ロギング ＆ 結果格納
        %% =========================================================================
        if n_obs > 0
            slack_nom  = A_soft * u_nominal - b_soft;
            slack_safe = b_soft - A_soft * u_final;
            num_violated = sum(slack_nom > 0);
            
            if num_violated > 0
                fprintf('[FastBridge ECBF] 介入中 (min_dist: %+.3fm)\n', min_surf_dist);
                fprintf('  Nominal : [u1=%5.2f, u2=%+6.3f, u3=%+6.3f, u4=%+6.3f]\n', ...
                    u_nominal(1), u_nominal(2), u_nominal(3), u_nominal(4));
                fprintf('  Safe    : [u1=%5.2f, u2=%+6.3f, u3=%+6.3f, u4=%+6.3f]\n', ...
                    u_final(1), u_final(2), u_final(3), u_final(4));
            end
        else
            slack_nom = [];
            slack_safe = [];
            num_violated = 0;
        end
        
        obj.result.A_qp_all        = A_soft;
        obj.result.b_qp_all        = b_soft;
        obj.result.A_hard          = A_hard;
        obj.result.b_hard          = b_hard;
        obj.result.A_att           = A_att;
        obj.result.b_att           = b_att;
        obj.result.h_att           = h_att;
        obj.result.A_cable         = A_cable;
        obj.result.b_cable         = b_cable;
        obj.result.h_cable         = h_cable;
        obj.result.slack_nom       = slack_nom;
        obj.result.slack_safe      = slack_safe;
        obj.result.num_violated    = num_violated;
        obj.result.log_h_obs       = log_h_obs;
        obj.result.u_nominal       = u_nominal;
        obj.result.tmp_fix         = u_final;
        obj.result.cascade_vars    = cascade_vars;
        obj.result.controllertime  = toc(tic_start);
        
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


% classdef HLC_SUSPENDED_LOAD_HOCBF_LINK_XYZ < handle
%     % スリング負荷付きクアッドコプター用 4入力完全テイラー展開 HOCBF コントローラ
%     % 最適化変数: z = [u; delta_obs; delta_in] in R^(4 + N_obs + 8)
%     % 制約: 
%     %   - ハードCBF (マージンなし r_obs): スラックなし
%     %   - ソフトCBF (マージンあり r_obs_margin): スラック delta_obs >= 0
%     %   - ソフト入力上下限: スラック delta_in >= 0
% properties
%     self
%     result
%     param
%     u_prev % 前回ステップの安全入力 (1-step SQP / テイラー展開中心 u0)
% end
% methods
%     function obj = HLC_SUSPENDED_LOAD_HOCBF_LINK_XYZ(self, param)
%         obj.self = self;
%         obj.param = param;
%         obj.result.min_clearance = Inf;
%         obj.u_prev = [];
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
% 
%         P = [obj.self.parameter.get(["mass", "jx", "jy", "jz", "gravity", "loadmass", "cableL"]), 0, 0, 0];
%         x = [model.state.getq('compact'); model.state.w; pL; model.state.vL; pT; model.state.wL];
% 
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
%         %% 1. ノミナル制御入力の算定 ＆ 事前クリップ
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
%         % 入力上下限の定義 [u1 (推力); u2 (Mx); u3 (My); u4 (Mz)]
%         f_hover = (P(1) + P(6)) * P(5);
%         lb_u = [0.85 * f_hover; -0.50; -0.50; -0.20];
%         ub_u = [1.40 * f_hover;  0.50;  0.50;  0.20];
% 
%         % 🌟 事前クリッピング
%         u_nominal = max(lb_u, min(ub_u, u_raw));
%         obj.result.tmp = u_nominal; 
% 
%         %% =========================================================================
%         %% 2. 幾何情報・クリアランス計算
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
%         %% 3. テイラー展開 HOCBF 制約行列の構築 (ハード & ソフト)
%         %% =========================================================================
%         % 展開中心 u0 の設定 (初回はノミナル、以降は前回の最適化出力)
%         if isempty(obj.u_prev)
%             u0_k = u_nominal;
%         else
%             u0_k = obj.u_prev;
%         end
% 
%         % gamma_obs = [4*2.5; 6*2.5^2; 4*2.5^3; 2.5^4]; % lambda = 2.5
%         gamma_obs = [2; 4; 8; 16];
% 
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
%             [A_h, b_h, ~, ~, ~, ~] = CBF_Constraints_HOCBF_Taylor_4Inputs(...
%                 obj, x, xd, u0_k, obs_params_hard, gamma_obs, rl_sys, P);
%             A_hard(i, :) = real(A_h);
%             b_hard(i)    = real(b_h);
% 
%             % 2) ソフト制約: マージンあり (r_obs_margin)
%             ro_soft = obs_env(i).r_obs_margin;
%             obs_params_soft = [p_obs; ro_soft];
%             [A_s, b_s, h0_v, h1_v, h2_v, h3_v] = CBF_Constraints_HOCBF_Taylor_4Inputs(...
%                 obj, x, xd, u0_k, obs_params_soft, gamma_obs, rl_sys, P);
%             A_soft(i, :) = real(A_s);
%             b_soft(i)    = real(b_s);
%             log_h_obs(i, :) = [h0_v, h1_v, h2_v, h3_v];
%         end
% 
%         %% =========================================================================
%         %% 4. ソフト制約付き単一 QP の構築と求解
%         %% =========================================================================
%         % 最適化決定変数: z = [u (4x1); delta_obs (num_obs x 1); delta_lb (4x1); delta_ub (4x1)]
%         n_u = 4;
%         n_obs = num_obs;
%         n_in = 4;
%         n_vars = n_u + n_obs + 2 * n_in;
% 
%         W_u = diag([10.0, 0.01, 0.01, 2.0]); % 制御入力偏差ペナルティ
%         rho_obs_quad = 1e2;  rho_obs_lin = 1e4;  % 障害物マージンスラック重み
%         rho_in_quad  = 1e3;  rho_in_lin  = 1e5;  % 入力上下限スラック重み
% 
%         % コスト関数: 0.5 * z' * H_qp * z + f_qp' * z
%         H_qp = blkdiag(W_u, ...
%                        rho_obs_quad * eye(n_obs), ...
%                        rho_in_quad * eye(n_in), ...
%                        rho_in_quad * eye(n_in));
%         f_qp = [-W_u * u_nominal; ...
%                 rho_obs_lin * ones(n_obs, 1); ...
%                 rho_in_lin * ones(n_in, 1); ...
%                 rho_in_lin * ones(n_in, 1)];
% 
%         % 不等式制約: A_ineq * z <= b_ineq
%         % [1] ハードCBF:      A_hard * u <= b_hard
%         % [2] ソフトCBF:      A_soft * u - delta_obs <= b_soft
%         % [3] ソフト下限:    -u - delta_lb <= -lb_u  ==> u >= lb_u - delta_lb
%         % [4] ソフト上限:     u - delta_ub <=  ub_u  ==> u <= ub_u + delta_ub
%         A_ineq = [
%             A_hard,  zeros(n_obs, n_obs),  zeros(n_obs, n_in),  zeros(n_obs, n_in); ...
%             A_soft, -eye(n_obs),           zeros(n_obs, n_in),  zeros(n_obs, n_in); ...
%             -eye(n_u), zeros(n_u, n_obs), -eye(n_in),           zeros(n_u, n_in);   ...
%              eye(n_u), zeros(n_u, n_obs),  zeros(n_u, n_in),   -eye(n_in)           ...
%         ];
%         b_ineq = [b_hard; b_soft; -lb_u; ub_u];
% 
%         % 変数の上下限: スラック変数は非負 (delta >= 0)
%         lb_qp = [-Inf(n_u, 1); zeros(n_obs + 2 * n_in, 1)];
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
%         % 🌟 事後クリッピング (物理上下限の確実なクリップ)
%         u_final = max(lb_u, min(ub_u, u_opt));
% 
%         % 次回ステップのテイラー展開中心として保存
%         obj.u_prev = u_final;
% 
%         %% =========================================================================
%         %% 5. デバッグ表示 ＆ ロギング
%         %% =========================================================================
%         slack_nom  = A_soft * u_nominal - b_soft;
%         slack_safe = b_soft - A_soft * u_final;
%         num_violated = sum(slack_nom > 0);
% 
%         if num_violated > 0
%             fprintf('[Taylor HOCBF] 介入発生 (マージン接近障害物数: %d)\n', num_violated);
%             fprintf('  Nominal : [u1=%.2f, u2=%.3f, u3=%.3f, u4=%.3f]\n', ...
%                 u_nominal(1), u_nominal(2), u_nominal(3), u_nominal(4));
%             fprintf('  Safe    : [u1=%.2f, u2=%.3f, u3=%.3f, u4=%.3f]\n', ...
%                 u_final(1), u_final(2), u_final(3), u_final(4));
%             [max_viol, idx_v] = max(slack_nom);
%             fprintf('  Max Soft-Viol (Obs #%d): %.3f | A*u_nom=%.2f, A*u_safe=%.2f (b=%.2f)\n', ...
%                 idx_v, max_viol, A_soft(idx_v,:)*u_nominal, A_soft(idx_v,:)*u_final, b_soft(idx_v));
%         end
% 
%         obj.result.A_qp_all        = A_soft;
%         obj.result.b_qp_all        = b_soft;
%         obj.result.A_hard          = A_hard;
%         obj.result.b_hard          = b_hard;
%         obj.result.slack_nom       = slack_nom;
%         obj.result.slack_safe      = slack_safe;
%         obj.result.num_violated    = num_violated;
%         obj.result.log_h_obs       = log_h_obs;
%         obj.result.u_nominal       = u_nominal;
%         obj.result.tmp_fix         = u_final;
%         obj.result.controllertime  = toc(tic_start);
% 
%         % 最終制御出力
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

% classdef HLC_SUSPENDED_LOAD_HOCBF_LINK_XYZ < handle
%     % スリング負荷付きクアッドコプター用 4入力テイラー展開 HOCBF コントローラ
%     % 展開中心 u0: 今周期の事前クリップ済みノミナル入力 u_nominal
%     % 最適化変数: z = [u; delta_obs; delta_lb; delta_ub] in R^(4 + N_obs + 8)
%     % 制約: 
%     %   - ハードCBF (マージンなし r_obs): スラックなし
%     %   - ソフトCBF (マージンあり r_obs_margin): スラック delta_obs >= 0
%     %   - ソフト入力上下限: スラック delta_in >= 0
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
% 
%         P = [obj.self.parameter.get(["mass", "jx", "jy", "jz", "gravity", "loadmass", "cableL"]), 0, 0, 0];
%         x = [model.state.getq('compact'); model.state.w; pL; model.state.vL; pT; model.state.wL];
% 
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
%         %% 1. ノミナル制御入力の算定 ＆ 事前クリップ
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
%         % 入力上下限の定義 [u1 (推力); u2 (Mx); u3 (My); u4 (Mz)]
%         f_hover = (P(1) + P(6)) * P(5);
%         lb_u = [0.85 * f_hover; -0.50; -0.50; -0.20];
%         ub_u = [1.40 * f_hover;  0.50;  0.50;  0.20];
% 
%         % 🌟 事前クリッピング (今周期のノミナル入力)
%         u_nominal = max(lb_u, min(ub_u, u_raw));
%         obj.result.tmp = u_nominal; 
% 
%         %% =========================================================================
%         %% 2. 幾何情報・クリアランス計算
%         %% =========================================================================
%         min_surf_dist = Inf;
%         obs_env = ENVIRONMENT_OBSTACLE_HOCBF_LINK_XY();
%         num_obs = length(obs_env);
%         L_cable = P(7); 
%         p_mid   = pL - 0.5 * L_cable * pT;
%         rl_sys  = 1.0; % 紐・荷物半径マージン
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
%         %% 3. テイラー展開 HOCBF 制約行列の構築 (ハード & ソフト)
%         %% =========================================================================
%         % 🌟 展開中心 u0 として今周期のクリッピング後ノミナル入力を直接使用
%         u0_k = u_nominal;
% 
%         gamma_obs = [2; 4; 8; 16];
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
%             [A_h, b_h, ~, ~, ~, ~] = CBF_Constraints_HOCBF_Taylor_4Inputs(...
%                 obj, x, xd, u0_k, obs_params_hard, gamma_obs, rl_sys, P);
%             A_hard(i, :) = real(A_h);
%             b_hard(i)    = real(b_h);
% 
%             % 2) ソフト制約: マージンあり (r_obs_margin)
%             ro_soft = obs_env(i).r_obs_margin;
%             obs_params_soft = [p_obs; ro_soft];
%             [A_s, b_s, h0_v, h1_v, h2_v, h3_v] = CBF_Constraints_HOCBF_Taylor_4Inputs(...
%                 obj, x, xd, u0_k, obs_params_soft, gamma_obs, rl_sys, P);
%             A_soft(i, :) = real(A_s);
%             b_soft(i)    = real(b_s);
%             log_h_obs(i, :) = [h0_v, h1_v, h2_v, h3_v];
%         end
% 
%         %% =========================================================================
%         %% 4. ソフト制約付き単一 QP の構築と求解
%         %% =========================================================================
%         % 最適化決定変数: z = [u (4x1); delta_obs (num_obs x 1); delta_lb (4x1); delta_ub (4x1)]
%         n_u = 4;
%         n_obs = num_obs;
%         n_in = 4;
%         n_vars = n_u + n_obs + 2 * n_in;
% 
%         W_u = diag([10.0, 0.01, 0.01, 2.0]); % 制御入力偏差ペナルティ
%         rho_obs_quad = 1e2;  rho_obs_lin = 1e4;  % 障害物マージンスラック重み
%         rho_in_quad  = 1e3;  rho_in_lin  = 1e5;  % 入力上下限スラック重み
% 
%         % コスト関数: 0.5 * z' * H_qp * z + f_qp' * z
%         H_qp = blkdiag(W_u, ...
%                        rho_obs_quad * eye(n_obs), ...
%                        rho_in_quad * eye(n_in), ...
%                        rho_in_quad * eye(n_in));
%         f_qp = [-W_u * u_nominal; ...
%                 rho_obs_lin * ones(n_obs, 1); ...
%                 rho_in_lin * ones(n_in, 1); ...
%                 rho_in_lin * ones(n_in, 1)];
% 
%         % 不等式制約: A_ineq * z <= b_ineq
%         % [1] ハードCBF:      A_hard * u <= b_hard
%         % [2] ソフトCBF:      A_soft * u - delta_obs <= b_soft
%         % [3] ソフト下限:    -u - delta_lb <= -lb_u  ==> u >= lb_u - delta_lb
%         % [4] ソフト上限:     u - delta_ub <=  ub_u  ==> u <= ub_u + delta_ub
%         A_ineq = [
%             A_hard,  zeros(n_obs, n_obs),  zeros(n_obs, n_in),  zeros(n_obs, n_in); ...
%             A_soft, -eye(n_obs),           zeros(n_obs, n_in),  zeros(n_obs, n_in); ...
%             -eye(n_u), zeros(n_u, n_obs), -eye(n_in),           zeros(n_u, n_in);   ...
%              eye(n_u), zeros(n_u, n_obs),  zeros(n_u, n_in),   -eye(n_in)           ...
%         ];
%         b_ineq = [b_hard; b_soft; -lb_u; ub_u];
% 
%         % 変数の上下限: スラック変数は非負 (delta >= 0)
%         lb_qp = [-Inf(n_u, 1); zeros(n_obs + 2 * n_in, 1)];
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
%         % 🌟 事後クリッピング (物理上下限の確実なクリップ)
%         u_final = max(lb_u, min(ub_u, u_opt));
% 
%         %% =========================================================================
%         %% 5. デバッグ表示 ＆ ロギング
%         %% =========================================================================
%         slack_nom  = A_soft * u_nominal - b_soft;
%         slack_safe = b_soft - A_soft * u_final;
%         num_violated = sum(slack_nom > 0);
% 
%         if num_violated > 0
%             fprintf('[Taylor HOCBF] 介入発生 (マージン接近障害物数: %d)\n', num_violated);
%             fprintf('  Nominal : [u1=%.2f, u2=%.3f, u3=%.3f, u4=%.3f]\n', ...
%                 u_nominal(1), u_nominal(2), u_nominal(3), u_nominal(4));
%             fprintf('  Safe    : [u1=%.2f, u2=%.3f, u3=%.3f, u4=%.3f]\n', ...
%                 u_final(1), u_final(2), u_final(3), u_final(4));
%             [max_viol, idx_v] = max(slack_nom);
%             fprintf('  Max Soft-Viol (Obs #%d): %.3f | A*u_nom=%.2f, A*u_safe=%.2f (b=%.2f)\n', ...
%                 idx_v, max_viol, A_soft(idx_v,:)*u_nominal, A_soft(idx_v,:)*u_final, b_soft(idx_v));
%         end
% 
%         obj.result.A_qp_all        = A_soft;
%         obj.result.b_qp_all        = b_soft;
%         obj.result.A_hard          = A_hard;
%         obj.result.b_hard          = b_hard;
%         obj.result.slack_nom       = slack_nom;
%         obj.result.slack_safe      = slack_safe;
%         obj.result.num_violated    = num_violated;
%         obj.result.log_h_obs       = log_h_obs;
%         obj.result.u_nominal       = u_nominal;
%         obj.result.tmp_fix         = u_final;
%         obj.result.controllertime  = toc(tic_start);
% 
%         % 最終制御出力
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
    % スリング負荷付きクアッドコプター用 統合 HOCBF 単一 QP コントローラ
    % 最適化変数: z = [u (4x1); delta_obs (num_obs x 1); delta_att (1x1); delta_cb (1x1); delta_lb (4x1); delta_ub (4x1)]
    % 制約:
    %   - ハードCBF (障害物マージンなし r_obs): スラックなし
    %   - ソフトCBF (障害物マージンあり r_obs_margin): スラック delta_obs >= 0
    %   - ソフトCBF (機体姿勢角制限): スラック delta_att >= 0
    %   - ソフトCBF (紐傾斜角制限): スラック delta_cb >= 0
    %   - ソフト入力上下限: スラック delta_lb, delta_ub >= 0
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
        
        P = [obj.self.parameter.get(["mass", "jx", "jy", "jz", "gravity", "loadmass", "cableL"]), 0, 0, 0];
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
        %% 1. ノミナル制御入力の算定 ＆ 事前クリップ
        %% =========================================================================
        F1 = Param.F1; F2 = Param.F2; F3 = Param.F3; F4 = Param.F4; 
        vf = obj.Vfd_SuspendedLoadxyDst(Param.dt, x, xd', F1); 
        vs = obj.Vs_SuspendedLoadxyDst(x, xd', vf, P, F2, F3, F4); 
        uf = obj.Uf_SuspendedLoadxyDst(x, xd', vf, P); 
        beta2 = obj.Beta2_SuspendedLoadxyDst(x, xd', vf, P); 
        vs_alpha2 = obj.V2_alpha2_SuspendedLoadxyDst(x, xd', vf, vs', P); 
        us = beta2 \ vs_alpha2; 
        u_raw = [uf(1); us]; % 生ノミナル入力

        % 入力上下限の定義 [u1 (推力); u2 (Mx); u3 (My); u4 (Mz)]
        f_hover = (P(1) + P(6)) * P(5);
        lb_u = [0.85 * f_hover; -0.50; -0.50; -0.20];
        ub_u = [1.40 * f_hover;  0.50;  0.50;  0.20];

        % 事前クリッピング (今周期のノミナル入力)
        u_nominal = max(lb_u, min(ub_u, u_raw));
        obj.result.tmp = u_nominal; 

        %% =========================================================================
        %% 2. 幾何情報・クリアランス計算
        %% =========================================================================
        min_surf_dist = Inf;
        obs_env = ENVIRONMENT_OBSTACLE_HOCBF_LINK_XY();
        num_obs = length(obs_env);
        L_cable = P(7); 
        p_mid   = pL - 0.5 * L_cable * pT;
        rl_sys  = 1; % 紐・荷物半径マージン
        obj.result.rl    = rl_sys;
        obj.result.p_mid = p_mid;
        
        log_p_obs        = cell(1, max(1, num_obs));
        log_r_obs_margin = cell(1, max(1, num_obs));
        log_r_minimal    = cell(1, max(1, num_obs));
        
        for i = 1:num_obs
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
        %% 3. テイラー展開 HOCBF 制約行列の構築
        %% =========================================================================
        % 展開中心 u0 として今周期のクリッピング後ノミナル入力を使用
        u0_k = u_nominal;

        % --- [A] 障害物回避 CBF (4階微分) ---
        gamma_obs = [2; 4; 8; 16];
        A_hard = zeros(num_obs, 4);
        b_hard = zeros(num_obs, 1);
        A_soft = zeros(num_obs, 4);
        b_soft = zeros(num_obs, 1);
        log_h_obs = zeros(num_obs, 4);

        for i = 1:num_obs
            p_obs = obs_env(i).p_obs(:);
            
            % 1) ハード制約: マージンなし (r_obs)
            ro_hard = obs_env(i).r_obs;
            obs_params_hard = [p_obs; ro_hard];
            [A_h, b_h, ~, ~, ~, ~] = CBF_Constraints_HOCBF_Taylor_4Inputs(...
                obj, x, xd, u0_k, obs_params_hard, gamma_obs, rl_sys, P);
            A_hard(i, :) = real(A_h);
            b_hard(i)    = real(b_h);

            % 2) ソフト制約: マージンあり (r_obs_margin)
            ro_soft = obs_env(i).r_obs_margin;
            obs_params_soft = [p_obs; ro_soft];
            [A_s, b_s, h0_v, h1_v, h2_v, h3_v] = CBF_Constraints_HOCBF_Taylor_4Inputs(...
                obj, x, xd, u0_k, obs_params_soft, gamma_obs, rl_sys, P);
            A_soft(i, :) = real(A_s);
            b_soft(i)    = real(b_s);
            log_h_obs(i, :) = [h0_v, h1_v, h2_v, h3_v];
        end

        % --- [B] 機体姿勢角 ＆ 紐傾斜角 CBF (2階微分) ---
        ang_limits = [deg2rad(35); deg2rad(25)]; % [機体最大許容角; 紐最大許容角]
        gamma_ang  = [5.0; 10.0; 5.0; 10.0];      % [att1; att2; cb1; cb2]
        
        [A_att, b_att, h_att, A_cable, b_cable, h_cable] = CBF_Constraints_Attitude_Cable_Taylor(...
            obj, x, xd, u0_k, ang_limits, gamma_ang, rl_sys, P);
        
        A_att   = real(A_att);   b_att   = real(b_att);
        A_cable = real(A_cable); b_cable = real(b_cable);

        %% =========================================================================
        %% 4. ソフト制約付き単一 QP の構築と求解
        %% =========================================================================
        % 最適化決定変数: z = [u (4x1); delta_obs (num_obs x 1); delta_att (1x1); delta_cb (1x1); delta_lb (4x1); delta_ub (4x1)]
        n_u   = 4;
        n_obs = num_obs;
        n_ang = 2; % [delta_att; delta_cb]
        n_in  = 4; % [delta_lb, delta_ub 各4次元]
        n_vars = n_u + n_obs + n_ang + 2 * n_in;

        W_u = diag([10.0, 0.01, 0.01, 2.0]);     % 制御入力偏差ペナルティ
        rho_obs_quad = 1e2;  rho_obs_lin = 1e4;  % 障害物マージンスラック重み
        rho_ang_quad = 1e2;  rho_ang_lin = 1e2;  % 姿勢・紐角度スラック重み
        rho_in_quad  = 1e3;  rho_in_lin  = 1e5;  % 入力上下限スラック重み

        % コスト関数: 0.5 * z' * H_qp * z + f_qp' * z
        H_qp = blkdiag(W_u, ...
                       rho_obs_quad * eye(n_obs), ...
                       rho_ang_quad * eye(n_ang), ...
                       rho_in_quad  * eye(n_in), ...
                       rho_in_quad  * eye(n_in));
        f_qp = [-W_u * u_nominal; ...
                rho_obs_lin * ones(n_obs, 1); ...
                rho_ang_lin * ones(n_ang, 1); ...
                rho_in_lin  * ones(n_in, 1); ...
                rho_in_lin  * ones(n_in, 1)];

        % 不等式制約: A_ineq * z <= b_ineq
        % [1] ハードCBF (障害物): A_hard * u <= b_hard
        % [2] ソフトCBF (障害物): A_soft * u - delta_obs <= b_soft
        % [3] ソフトCBF (姿勢角): A_att  * u - delta_att <= b_att
        % [4] ソフトCBF (紐角度): A_cable* u - delta_cb  <= b_cable
        % [5] ソフト下限:        -u - delta_lb <= -lb_u
        % [6] ソフト上限:         u - delta_ub <=  ub_u
        A_ineq = [
            A_hard,    zeros(n_obs, n_obs), zeros(n_obs, n_ang), zeros(n_obs, n_in), zeros(n_obs, n_in); ...
            A_soft,   -eye(n_obs),          zeros(n_obs, n_ang), zeros(n_obs, n_in), zeros(n_obs, n_in); ...
            A_att,     zeros(1, n_obs),    -1,  0,               zeros(1, n_in),     zeros(1, n_in);     ...
            A_cable,   zeros(1, n_obs),     0, -1,               zeros(1, n_in),     zeros(1, n_in);     ...
            -eye(n_u), zeros(n_u, n_obs),   zeros(n_u, n_ang),  -eye(n_in),          zeros(n_u, n_in);   ...
             eye(n_u), zeros(n_u, n_obs),   zeros(n_u, n_ang),   zeros(n_u, n_in),  -eye(n_in)           ...
        ];
        b_ineq = [b_hard; b_soft; b_att; b_cable; -lb_u; ub_u];

        % 変数の上下限: スラック変数は非負 (delta >= 0)
        lb_qp = [-Inf(n_u, 1); zeros(n_obs + n_ang + 2 * n_in, 1)];
        ub_qp = Inf(n_vars, 1);

        options_qp = optimoptions('quadprog', 'Display', 'off', 'ConstraintTolerance', 1e-4);
        [z_opt, ~, exitflag] = quadprog(H_qp, f_qp, A_ineq, b_ineq, [], [], lb_qp, ub_qp, [], options_qp);

        if (exitflag == 1 || exitflag == 2) && ~isempty(z_opt)
            u_opt = z_opt(1:4);
        else
            u_opt = u_nominal;
        end

        % 事後クリッピング (物理上下限の確実なクリップ)
        u_final = max(lb_u, min(ub_u, u_opt));

        %% =========================================================================
        %% 5. デバッグ表示 ＆ ロギング
        %% =========================================================================
        slack_nom  = A_soft * u_nominal - b_soft;
        slack_safe = b_soft - A_soft * u_final;
        num_violated = sum(slack_nom > 0);

        if num_violated > 0
            fprintf('[Integrated HOCBF] 介入発生 (障害物マージン接近数: %d)\n', num_violated);
            fprintf('  Nominal : [u1=%.2f, u2=%.3f, u3=%.3f, u4=%.3f]\n', ...
                u_nominal(1), u_nominal(2), u_nominal(3), u_nominal(4));
            fprintf('  Safe    : [u1=%.2f, u2=%.3f, u3=%.3f, u4=%.3f]\n', ...
                u_final(1), u_final(2), u_final(3), u_final(4));
            [max_viol, idx_v] = max(slack_nom);
            fprintf('  Max Soft-Viol (Obs #%d): %.3f | A*u_nom=%.2f, A*u_safe=%.2f (b=%.2f)\n', ...
                idx_v, max_viol, A_soft(idx_v,:)*u_nominal, A_soft(idx_v,:)*u_final, b_soft(idx_v));
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
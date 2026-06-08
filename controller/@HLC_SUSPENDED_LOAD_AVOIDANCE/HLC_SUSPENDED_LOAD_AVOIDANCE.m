% 
% 
% classdef HLC_SUSPENDED_LOAD_AVOIDANCE < handle
% properties
%     self
%     result
%     param
% 
%     % 🌟【論文仕様】未知の外乱を一括蓄積するための離散時間積分メモリ [cite: 23, 37]
%     input_integral = [0; 0; 0]; 
% end
% methods
%     function obj = HLC_SUSPENDED_LOAD_AVOIDANCE(self, param)
%         obj.self = self;
%         obj.param = param;
%     end
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
% 
%         yaw = wrapToPi(model.state.q(3)); 
%         yawd = xd(4); 
%         yawUnit = [cos(yaw); sin(yaw); 0]; 
%         yawdUnit = [cos(yawd); sin(yawd); 0]; 
%         deltaYaw = sign(cross(yawdUnit, yawUnit)) * acos(yawdUnit' * yawUnit); 
%         xd(4) = -deltaYaw(3) + yaw; 
%         xd = [xd; zeros(28 - size(xd, 1), 1)]; 
% 
%         F1 = Param.F1; F2 = Param.F2; F3 = Param.F3; F4 = Param.F4; 
% 
%         % -----------------------------------------------------------------
%         % フェーズ①：公称名目入力（目標加速度）の計算
%         % -----------------------------------------------------------------
%         vf_nominal = obj.Vfd_SuspendedLoadxyDst(Param.dt, x, xd', F1); 
%         vs_nominal = obj.Vs_SuspendedLoadxyDst(x, xd', vf_nominal, P, F2, F3, F4); 
% 
%         u_nominal = [vs_nominal(1); vs_nominal(2); vf_nominal(1)]; 
%         u_nominal = u_nominal(:); 
% 
%         % -----------------------------------------------------------------
%         % 🌟【論文式(37)】UDE (外乱オブザーバ) によるリアルタイム外乱推定 [cite: 23, 78]
%         % -----------------------------------------------------------------
%         v_drone = model.state.v; 
%         T_ude = 0.4;             % 論文Table 1パラメータ [cite: 545]
% 
%         persistent last_u_safe;
%         if isempty(last_u_safe); last_u_safe = u_nominal; end
%         obj.input_integral = obj.input_integral + last_u_safe * Param.dt;
% 
%         % 加速度次元の一括外乱推定値の計算
%         d_hat = (1 / T_ude) * (v_drone - obj.input_integral); 
% 
%         % -----------------------------------------------------------------
%         % フェーズ②：安全制約（CBF）パラメータの準備
%         % -----------------------------------------------------------------
%         obs_env = ENVIRONMENT_OBSTACLE();
%         ox = obs_env(1).p_obs(1);
%         oy = obs_env(1).p_obs(2);
%         oz = obs_env(1).p_obs(3);
%         ro = obs_env(1).r_obs;
% 
%         r_drone = 0.3; r_load = 0.2;  
% 
%         c1 = 0.8; c2 = 2.5; 
%         d1 = 1.0; d2 = 2;
% 
%         cbfParam_load  = [ox; oy; oz; ro; r_load;  c1; c2; P(:)];
%         cbfParam_drone = [ox; oy; oz; ro; r_drone; d1; d2; P(:)];
% 
%         is_flight_now = false;
%         if length(varargin) >= 2
%             if strcmp(varargin{2}, 'f') || string(varargin{2}) == "f"
%                 is_flight_now = true;
%             end
%         end
% 
%         current_h_load_eval = 0.5 * ((pL(1) - ox)^2 + (pL(2) - oy)^2 + (pL(3) - oz)^2 - (ro + r_load)^2);
% 
%         if is_flight_now && (current_h_load_eval <= 10.0)
% 
%             t_current = 0.0; 
%             [A_load_raw,  b_load_raw]  = CBF_Constraints_Load(obj,  x, xd', u_nominal, cbfParam_load,  t_current);
%             [A_drone_raw, b_drone_raw] = CBF_Constraints_Drone(obj, x, xd', u_nominal, cbfParam_drone, t_current);
% 
%             % 🌟【論文式(43)に準拠】入力状態安全(ISSf)を保証する動的緩和項 Δ [cite: 79, 375]
%             sigma_param = 1.0;    % kappa_d = 1.0 [cite: 545]
%             epsilon_param = 0.5;  
%             gamma_param = 2.0;    % Extended class Kゲイン [cite: 324]
%             LgH_load_norm2 = norm(A_load_raw(1:3))^2;
%             Delta_load = LgH_load_norm2 / (4 * sigma_param * (epsilon_param - gamma_param/2 + 1e-5)); 
% 
%             % 🌟【数理修正の核心：マイナス符号の強制適用】
%             % quadprog(A_qp * u <= b_qp)の標準形にするため、一括でマイナス化を行い整合性を100%確保します。
%             A_qp = -[A_load_raw(1:3); A_drone_raw(1:3)];
%             b_qp = -[b_load_raw(1); b_drone_raw(1)];
% 
%             % 論文の定理2に従い、外乱残差マージン Delta を境界から減算（防衛線を障害物手前に引き締める） [cite: 372, 373]
%             b_qp(1) = b_qp(1) - max(0, Delta_load); 
% 
%             % 状況モニタ表示
%             qp_margin = b_qp(1) - A_qp(1,:) * u_nominal;
%             fprintf('--- MONITOR -> t: %.3f | h_eval: %.2f | QP_Margin: %.2f | STATUS: [CBF-QP ON]\n', xd(1), current_h_load_eval, qp_margin);
% 
%             H_mat = eye(3)* 0.01;
%             f_vec = -u_nominal* 0.01;
%             options = optimoptions('quadprog', 'Display', 'off');
% 
%             [u_safe, ~, exitflag] = quadprog(H_mat, f_vec, A_qp, b_qp, [], [], [], [], [], options);
%             u_safe = u_safe(:);
% 
%             if exitflag ~= 1
%                 % 万が一のフォールバック退避モード
%                 p_to_obs = [ox; oy; oz] - pL;
%                 escape_dir = -p_to_obs / (norm(p_to_obs) + 1e-5);
%                 u_safe = u_nominal + escape_dir * 4.0; 
%             end
% 
%             current_h_load = current_h_load_eval;
%         else
%             fprintf('--- MONITOR -> t: %.3f | h_eval: %.2f | STATUS: [CBF OFF (安全直進中)]\n', xd(1), current_h_load_eval);
%             u_safe = u_nominal;
%             exitflag = 1;
%             current_h_load = 999.0;
%         end
% 
%         last_u_safe = u_safe; 
% 
%         % -----------------------------------------------------------------
%         % 🌟【論文式(31)】UDEによるアクティブ外乱相殺 [cite: 23, 343]
%         % -----------------------------------------------------------------
%         u_robust = u_safe - d_hat; 
% 
%         % -----------------------------------------------------------------
%         % フェーズ④：堅牢な加速度の格納 ＆ 高次微分の同調再合成
%         % -----------------------------------------------------------------
%         vs_safe = vs_nominal;
%         vf_safe = vf_nominal;
% 
%         vs_safe(1) = u_robust(1); 
%         vs_safe(2) = u_robust(2); 
%         vf_safe(1) = u_robust(3); 
% 
%         ratio = vf_safe(1) / (vf_nominal(1) + 1e-6);
%         vf_safe(2:6) = vf_nominal(2:6) * ratio; 
% 
%         % -----------------------------------------------------------------
%         % フェーズ⑤：既存の非線形相殺・実物理入力への最終コンバート
%         % -----------------------------------------------------------------
%         uf = obj.Uf_SuspendedLoadxyDst(x, xd', vf_safe, P);
%         beta2 = obj.Beta2_SuspendedLoadxyDst(x, xd', vf_safe, P);
%         vs_alpha2 = obj.V2_alpha2_SuspendedLoadxyDst(x, xd', vf_safe, vs_safe', P);
% 
%         us = beta2 \ vs_alpha2; 
% 
%         tmp = [uf(1); us]; 
%         obj.result.tmp = tmp; 
%         obj.result.input = [max(0, min(20, tmp(1))); max(-1, min(1, tmp(2))); max(-1, min(1, tmp(3))); max(-1, min(1, tmp(4)))]; 
%         obj.result.xd = xd;
%         obj.result.x = x;
%         result = obj.result;
%     end
%     function result = show(obj)
%         obj.result
%     end
% end
% end




% classdef HLC_SUSPENDED_LOAD_AVOIDANCE < handle
% properties
%     self
%     result
%     param
% 
%     % 🌟【論文仕様】未知の外乱を一括蓄積するための離散時間積分メモリ [cite: 23, 37]
%     input_integral = [0; 0; 0]; 
% end
% methods
%     function obj = HLC_SUSPENDED_LOAD_AVOIDANCE(self, param)
%         obj.self = self;
%         obj.param = param;
%     end
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
% 
%         yaw = wrapToPi(model.state.q(3)); 
%         yawd = xd(4); 
%         yawUnit = [cos(yaw); sin(yaw); 0]; 
%         yawdUnit = [cos(yawd); sin(yawd); 0]; 
%         deltaYaw = sign(cross(yawdUnit, yawUnit)) * acos(yawdUnit' * yawUnit); 
%         xd(4) = -deltaYaw(3) + yaw; 
%         xd = [xd; zeros(28 - size(xd, 1), 1)]; 
% 
%         F1 = Param.F1; F2 = Param.F2; F3 = Param.F3; F4 = Param.F4; 
% 
%         % -----------------------------------------------------------------
%         % フェーズ①：公称名目入力（目標加速度）の計算
%         % -----------------------------------------------------------------
%         vf_nominal = obj.Vfd_SuspendedLoadxyDst(Param.dt, x, xd', F1); 
%         vs_nominal = obj.Vs_SuspendedLoadxyDst(x, xd', vf_nominal, P, F2, F3, F4); 
% 
%         u_nominal = [vs_nominal(1); vs_nominal(2); vf_nominal(1)]; 
%         u_nominal = u_nominal(:); 
% 
%         % -----------------------------------------------------------------
%         % 🌟【論文式(37)】UDE (外乱オブザーバ) によるリアルタイム外乱推定 [cite: 23, 78]
%         % -----------------------------------------------------------------
%         v_drone = model.state.v; 
%         T_ude = 0.4;             % 論文Table 1パラメータ [cite: 545]
% 
%         persistent last_u_safe;
%         if isempty(last_u_safe); last_u_safe = u_nominal; end
%         obj.input_integral = obj.input_integral + last_u_safe * Param.dt;
% 
%         % 加速度次元の一括外乱推定値の計算
%         d_hat = (1 / T_ude) * (v_drone - obj.input_integral); 
% 
%         % -----------------------------------------------------------------
%         % フェーズ②：安全制約（CBF）パラメータの準備
%         % -----------------------------------------------------------------
%         obs_env = ENVIRONMENT_OBSTACLE();
%         ox = obs_env(1).p_obs(1);
%         oy = obs_env(1).p_obs(2);
%         oz = obs_env(1).p_obs(3);
%         ro = obs_env(1).r_obs;
% 
%         r_drone = 0.3; r_load = 0.2;  
% 
%         c1 = 0.8; c2 = 2.5; 
%         d1 = 1.0; d2 = 2;
% 
%         cbfParam_load  = [ox; oy; oz; ro; r_load;  c1; c2; P(:)];
%         cbfParam_drone = [ox; oy; oz; ro; r_drone; d1; d2; P(:)];
% 
%         is_flight_now = false;
%         if length(varargin) >= 2
%             if strcmp(varargin{2}, 'f') || string(varargin{2}) == "f"
%                 is_flight_now = true;
%             end
%         end
% 
%         current_h_load_eval = 0.5 * ((pL(1) - ox)^2 + (pL(2) - oy)^2 + (pL(3) - oz)^2 - (ro + r_load)^2);
% 
%         if is_flight_now && (current_h_load_eval <= 10.0)
% 
%             t_current = 0.0; 
%             [A_load_raw,  b_load_raw]  = CBF_Constraints_Load(obj,  x, xd', u_nominal, cbfParam_load,  t_current);
%             [A_drone_raw, b_drone_raw] = CBF_Constraints_Drone(obj, x, xd', u_nominal, cbfParam_drone, t_current);
% 
%             % 🌟【論文式(43)に準拠】入力状態安全(ISSf)を保証する動的緩和項 Δ [cite: 79, 375]
%             sigma_param = 1.0;    % kappa_d = 1.0 [cite: 545]
%             epsilon_param = 0.5;  
%             gamma_param = 2.0;    % Extended class Kゲイン [cite: 324]
%             LgH_load_norm2 = norm(A_load_raw(1:3))^2;
%             Delta_load = LgH_load_norm2 / (4 * sigma_param * (epsilon_param - gamma_param/2 + 1e-5)); 
% 
%             % 🌟【数理修正の核心：マイナス符号の強制適用】
%             % quadprog(A_qp * u <= b_qp)の標準形にするため、一括でマイナス化を行い整合性を100%確保します。
%             A_qp = -[A_load_raw(1:3); A_drone_raw(1:3)];
%             b_qp = -[b_load_raw(1); b_drone_raw(1)];
% 
%             % 論文の定理2に従い、外乱残差マージン Delta を境界から減算（防衛線を障害物手前に引き締める） [cite: 372, 373]
%             b_qp(1) = b_qp(1) - max(0, Delta_load); 
% 
%             % 状況モニタ表示
%             qp_margin = b_qp(1) - A_qp(1,:) * u_nominal;
%             fprintf('--- MONITOR -> t: %.3f | h_eval: %.2f | QP_Margin: %.2f | STATUS: [CBF-QP ON]\n', xd(1), current_h_load_eval, qp_margin);
% 
%             H_mat = eye(3)* 0.01;
%             f_vec = -u_nominal* 0.01;
%             options = optimoptions('quadprog', 'Display', 'off');
% 
%             [u_safe, ~, exitflag] = quadprog(H_mat, f_vec, A_qp, b_qp, [], [], [], [], [], options);
%             u_safe = u_safe(:);
% 
%             if exitflag ~= 1
%                 % 万が一のフォールバック退避モード
%                 p_to_obs = [ox; oy; oz] - pL;
%                 escape_dir = -p_to_obs / (norm(p_to_obs) + 1e-5);
%                 u_safe = u_nominal + escape_dir * 4.0; 
%             end
% 
%             current_h_load = current_h_load_eval;
%         else
%             fprintf('--- MONITOR -> t: %.3f | h_eval: %.2f | STATUS: [CBF OFF (安全直進中)]\n', xd(1), current_h_load_eval);
%             u_safe = u_nominal;
%             exitflag = 1;
%             current_h_load = 999.0;
%         end
% 
%         last_u_safe = u_safe; 
% 
%         % -----------------------------------------------------------------
%         % 🌟【論文式(31)】UDEによるアクティブ外乱相殺 [cite: 23, 343]
%         % -----------------------------------------------------------------
%         u_robust = u_safe - d_hat; 
% 
%         % -----------------------------------------------------------------
%         % フェーズ④：堅牢な加速度の格納 ＆ 高次微分の同調再合成
%         % -----------------------------------------------------------------
%         vs_safe = vs_nominal;
%         vf_safe = vf_nominal;
% 
%         vs_safe(1) = u_robust(1); 
%         vs_safe(2) = u_robust(2); 
%         vf_safe(1) = u_robust(3); 
% 
%         ratio = vf_safe(1) / (vf_nominal(1) + 1e-6);
%         vf_safe(2:6) = vf_nominal(2:6) * ratio; 
% 
%         % -----------------------------------------------------------------
%         % フェーズ⑤：既存の非線形相殺・実物理入力への最終コンバート
%         % -----------------------------------------------------------------
%         uf = obj.Uf_SuspendedLoadxyDst(x, xd', vf_safe, P);
%         beta2 = obj.Beta2_SuspendedLoadxyDst(x, xd', vf_safe, P);
%         vs_alpha2 = obj.V2_alpha2_SuspendedLoadxyDst(x, xd', vf_safe, vs_safe', P);
% 
%         us = beta2 \ vs_alpha2; 
% 
%         tmp = [uf(1); us]; 
%         obj.result.tmp = tmp; 
%         obj.result.input = [max(0, min(20, tmp(1))); max(-1, min(1, tmp(2))); max(-1, min(1, tmp(3))); max(-1, min(1, tmp(4)))]; 
%         obj.result.xd = xd;
%         obj.result.x = x;
%         result = obj.result;
%     end
%     function result = show(obj)
%         obj.result
%     end
% end
% end

% classdef HLC_SUSPENDED_LOAD_AVOIDANCE < handle
% properties
%     self
%     result
%     param
%     % 🌟 未知の外乱を一括蓄積するための離散時間積分メモリ [cite: 23, 37]
%     input_integral = [0; 0; 0]; 
% end
% methods
%     function obj = HLC_SUSPENDED_LOAD_AVOIDANCE(self, param)
%         obj.self = self;
%         obj.param = param;
%     end
%     function result = do(obj, varargin)
%         Param = obj.param; 
%         model = obj.self.estimator.result; 
%         ref = obj.self.reference.result;   
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
%         % 🌟 スクリプトの定義に完全同期した24次元フル状態ベクトル x の展開
%         x = [model.state.getq('compact'); model.state.w; pL; model.state.vL; pT; model.state.wL]; 
% 
%         yaw = wrapToPi(model.state.q(3)); 
%         yawd = xd(4); 
%         yawUnit = [cos(yaw); sin(yaw); 0]; 
%         yawdUnit = [cos(yawd); sin(yawd); 0]; 
%         deltaYaw = sign(cross(yawdUnit, yawUnit)) * acos(yawdUnit' * yawUnit); 
%         xd(4) = -deltaYaw(3) + yaw; 
%         xd = [xd; zeros(28 - size(xd, 1), 1)]; 
%         F1 = Param.F1; F2 = Param.F2; F3 = Param.F3; F4 = Param.F4; 
% 
%         % -----------------------------------------------------------------
%         % フェーズ①：公称名目入力（目標加速度）の計算
%         % -----------------------------------------------------------------
%         vf_nominal = obj.Vfd_SuspendedLoadxyDst(Param.dt, x, xd', F1); 
%         vs_nominal = obj.Vs_SuspendedLoadxyDst(x, xd', vf_nominal, [obj.self.parameter.get(["mass", "jx", "jy", "jz", "gravity", "loadmass", "cableL"]), 0, 0], F2, F3, F4); 
%         u_nominal = [vs_nominal(1); vs_nominal(2); vf_nominal(1)]; 
%         u_nominal = u_nominal(:); 
% 
%         % -----------------------------------------------------------------
%         % 🌟 UDE (外乱オブザーバ) による加速度空間でのリアルタイム外乱推定 [cite: 23, 78]
%         % -----------------------------------------------------------------
%         v_drone = model.state.v; 
%         T_ude = 0.4;             % 論文Table 1 [cite: 545]
% 
%         % 🔍【デバッグ・持続リセット】タイムステップが初手(t=0付近)なら積分メモリを強制ゼロクリア
%         if xd(1) <= Param.dt + 1e-5
%             obj.input_integral = [0; 0; 0];
%         end
% 
%         persistent last_u_safe;
%         if isempty(last_u_safe); last_u_safe = u_nominal; end
% 
%         % 🌟 前回の安全ろ過された加速度コマンドを適切に時間積分
%         obj.input_integral = obj.input_integral + last_u_safe * Param.dt;
%         % 加速度次元の一括外乱推定値（風・非線形ズレ・張力影響）を計算 [cite: 23, 137]
%         d_hat = (1 / T_ude) * (v_drone - obj.input_integral); 
% 
%         % -----------------------------------------------------------------
%         % フェーズ②：安全制約（CBF）パラメータの準備＆新関数インプット
%         % -----------------------------------------------------------------
%         obs_env = ENVIRONMENT_OBSTACLE();
%         ox = obs_env(1).p_obs(1);
%         oy = obs_env(1).p_obs(2);
%         oz = obs_env(1).p_obs(3);
%         ro = obs_env(1).r_obs;
% 
%         r_drone = 0.3; r_load = 0.2;  
%         c1 = 0.8; c2 = 2.5; 
%         d1 = 1.0; d2 = 2.0;
%         P_param = [obj.self.parameter.get(["mass", "jx", "jy", "jz", "gravity", "loadmass", "cableL"]), 0, 0];
%         cbfParam_load  = [ox; oy; oz; ro; r_load;  c1; c2; P_param(:)];
%         cbfParam_drone = [ox; oy; oz; ro; r_drone; d1; d2; P_param(:)];
% 
%         is_flight_now = false;
%         if length(varargin) >= 2
%             if strcmp(varargin{2}, 'f') || string(varargin{2}) == "f"
%                 is_flight_now = true;
%             end
%         end
% 
%         % 衝突評価用の真の隙間（hのスカラー評価）
%         current_h_load_eval = 0.5 * ((pL(1) - ox)^2 + (pL(2) - oy)^2 + (pL(3) - oz)^2 - (ro + r_load)^2);
% 
%         if is_flight_now && (current_h_load_eval <= 15.0) % 評価半径を少し広げて監視
%             t_current = 0.0; 
% 
%             % 🌟 3次元化された新しいCBF制約生成関数の呼び出し
%             [A_load_raw,  b_load_raw]  = CBF_Constraints_Load(obj,  x, xd', u_nominal, cbfParam_load,  t_current);
%             [A_drone_raw, b_drone_raw] = CBF_Constraints_Drone(obj, x, xd', u_nominal, cbfParam_drone, t_current);
% 
%             % 🌟【論文式(43)】ISSfを保証する残差補償マージン Delta の計算 [cite: 79, 375]
%             sigma_param = 1.0;    
%             epsilon_param = 0.5;  
%             gamma_param = 2.0;    
%             LgH_load_norm2 = norm(A_load_raw)^2; % 🌟 3成分すべてを用いた真のL2ノルム
%             Delta_load = LgH_load_norm2 / (4 * sigma_param * (epsilon_param - gamma_param/2 + 1e-5)); 
% 
%             % 🌟 二次計画法標準形（A_qp * u_safe <= b_qp）へのパッキング（サイズミスマッチの完全解消）
%             A_qp = -[A_load_raw; A_drone_raw]; % 🌟 [1x3; 1x3] -> 2x3 マトリクス
%             b_qp = -[b_load_raw; b_drone_raw]; % 🌟 [2x1] ベクトル
% 
%             % 論文の定理2に従い、Delta分だけ防衛線を手前に引き締める [cite: 372, 373]
%             b_qp(1) = b_qp(1) - max(0, Delta_load); 
% 
%             % 🔍 コスト関数の重み（公称値からの乖離の最小化）
%             H_mat = eye(3) * 2.0;
%             f_vec = -2.0 * u_nominal;
% 
%             options = optimoptions('quadprog', 'Display', 'off');
%             [u_safe, ~, exitflag] = quadprog(H_mat, f_vec, A_qp, b_qp, [], [], [], [], [], options);
%             u_safe = u_safe(:);
% 
%             % 🔍 【強力デバッグロギング】QPソルバーの内部状態を完全可視化
%             qp_margin_load  = b_qp(1) - A_qp(1,:) * u_safe;
%             qp_margin_drone = b_qp(2) - A_qp(2,:) * u_safe;
%             u_diff = norm(u_safe - u_nominal);
% 
%             fprintf('⚙️ [CBF-QP ON] t: %.3f | h_load: %5.2f | LoadMargin: %5.2f | DroneMargin: %5.2f | QP_Exit: %d | Intervene(修正量): %.4f\n', ...
%                 xd(1), current_h_load_eval, qp_margin_load, qp_margin_drone, exitflag, u_diff);
% 
%             if exitflag ~= 1
%                 warning('⚠️ QP Solved with anomaly! Emergency fallback triggered.');
%                 p_to_obs = [ox; oy; oz] - pL;
%                 escape_dir = -p_to_obs / (norm(p_to_obs) + 1e-5);
%                 u_safe = u_nominal + escape_dir * 3.0; % 障害物の反対方向へ一歩退避
%             end
%         else
%             % 安全圏内、またはテイクオフ中
%             if is_flight_now
%                 fprintf('🟢 [CBF FREE] t: %.3f | h_load: %5.2f | UDE_d_hat: [%.2f, %.2f, %.2f]\n', ...
%                     xd(1), current_h_load_eval, d_hat(1), d_hat(2), d_hat(3));
%             end
%             u_safe = u_nominal;
%             exitflag = 1;
%         end
% 
%         last_u_safe = u_safe; 
% 
%         % -----------------------------------------------------------------
%         % 🌟【論文式(31)】UDEによるアクティブ外乱相殺 [cite: 23, 343]
%         % -----------------------------------------------------------------
%         u_robust = u_safe - d_hat; 
% 
%         % -----------------------------------------------------------------
%         % フェーズ④：堅牢な加速度の格納 ＆ 高次微分の同調再合成
%         % -----------------------------------------------------------------
%         vs_safe = vs_nominal;
%         vf_safe = vf_nominal;
%         vs_safe(1) = u_robust(1); 
%         vs_safe(2) = u_robust(2); 
%         vf_safe(1) = u_robust(3); 
%         ratio = vf_safe(1) / (vf_nominal(1) + 1e-6);
%         vf_safe(2:6) = vf_nominal(2:6) * ratio; 
% 
%         % -----------------------------------------------------------------
%         % フェーズ⑤：既存の非線形相殺・実物理入力への最終コンバート
%         % -----------------------------------------------------------------
%         uf = obj.Uf_SuspendedLoadxyDst(x, xd', vf_safe, P_param);
%         beta2 = obj.Beta2_SuspendedLoadxyDst(x, xd', vf_safe, P_param);
%         vs_alpha2 = obj.V2_alpha2_SuspendedLoadxyDst(x, xd', vf_safe, vs_safe', P_param);
%         us = beta2 \ vs_alpha2; 
%         tmp = [uf(1); us]; 
%         obj.result.tmp = tmp; 
%         obj.result.input = [max(0, min(20, tmp(1))); max(-1, min(1, tmp(2))); max(-1, min(1, tmp(3))); max(-1, min(1, tmp(4)))]; 
%         obj.result.xd = xd;
%         obj.result.x = x;
%         result = obj.result;
%     end
%     function result = show(obj)
%         obj.result
%     end
% end
% end

% classdef HLC_SUSPENDED_LOAD_AVOIDANCE < handle
% properties
%     self
%     result
%     param
%     % 🌟 積分メモリは明確にプロパティとして管理
%     input_integral = [0; 0; 0]; 
% end
% methods
%     function obj = HLC_SUSPENDED_LOAD_AVOIDANCE(self, param)
%         obj.self = self;
%         obj.param = param;
%     end
%     function result = do(obj, varargin)
%         Param = obj.param; 
%         model = obj.self.estimator.result; 
%         ref = obj.self.reference.result;   
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
%         % 🌟 1. パラメータ P は、元コードと全く同じ「行ベクトル」形状を厳格に死守
%         P = [obj.self.parameter.get(["mass", "jx", "jy", "jz", "gravity", "loadmass", "cableL"]), 0, 0];
%         x = [model.state.getq('compact'); model.state.w; pL; model.state.vL; pT; model.state.wL]; 
% 
%         yaw = wrapToPi(model.state.q(3)); 
%         yawd = xd(4); 
%         yawUnit = [cos(yaw); sin(yaw); 0]; 
%         yawdUnit = [cos(yawd); sin(yawd); 0]; 
%         deltaYaw = sign(cross(yawdUnit, yawUnit)) * acos(yawdUnit' * yawUnit); 
%         xd(4) = -deltaYaw(3) + yaw; 
%         xd = [xd; zeros(28 - size(xd, 1), 1)]; 
%         F1 = Param.F1; F2 = Param.F2; F3 = Param.F3; F4 = Param.F4; 
% 
%         % -----------------------------------------------------------------
%         % フェーズ①：公称名目入力（目標加速度）の計算 (元コードと完全同一)
%         % -----------------------------------------------------------------
%         vf_nominal = obj.Vfd_SuspendedLoadxyDst(Param.dt, x, xd', F1); 
%         vs_nominal = obj.Vs_SuspendedLoadxyDst(x, xd', vf_nominal, P, F2, F3, F4); 
%         u_nominal = [vs_nominal(1); vs_nominal(2); vf_nominal(1)]; 
%         u_nominal = u_nominal(:); 
% 
%         % -----------------------------------------------------------------
%         % 🌟 2. UDE外乱推定の同期・リセット処理の厳格化
%         % -----------------------------------------------------------------
%         v_drone = model.state.v; 
%         T_ude = 0.4; 
% 
%         % 初期化時、またはテイクオフ開始時は積分器を綺麗にリセット
%         if xd(1) <= Param.dt + 1e-5
%             obj.input_integral = [0; 0; 0];
%         end
% 
%         % -----------------------------------------------------------------
%         % フェーズ②：安全制約（CBF）フィルターの適用
%         % -----------------------------------------------------------------
%         obs_env = ENVIRONMENT_OBSTACLE();
%         ox = obs_env(1).p_obs(1);
%         oy = obs_env(1).p_obs(2);
%         oz = obs_env(1).p_obs(3);
%         ro = obs_env(1).r_obs;
% 
%         r_drone = 0.3; r_load = 0.2;  
%         c1 = 0.8; c2 = 2.5; 
%         d1 = 1.0; d2 = 2.0;
% 
%         % CBF関数専用の縦ベクトルパラメータを、元の P とは「別変数」として安全に分離
%         cbfParam_load  = [ox; oy; oz; ro; r_load;  c1; c2; P(:)];
%         cbfParam_drone = [ox; oy; oz; ro; r_drone; d1; d2; P(:)];
% 
%         is_flight_now = false;
%         if length(varargin) >= 2
%             if strcmp(varargin{2}, 'f') || string(varargin{2}) == "f"
%                 is_flight_now = true;
%             end
%         end
% 
%         current_h_load_eval = 0.5 * ((pL(1) - ox)^2 + (pL(2) - oy)^2 + (pL(3) - oz)^2 - (ro + r_load)^2);
% 
%         if is_flight_now && (current_h_load_eval <= 15.0)
%             t_current = 0.0; 
%             [A_load_raw,  b_load_raw]  = CBF_Constraints_Load(obj,  x, xd', u_nominal, cbfParam_load,  t_current);
%             [A_drone_raw, b_drone_raw] = CBF_Constraints_Drone(obj, x, xd', u_nominal, cbfParam_drone, t_current);
% 
%             % 🌟 3. 論文パラメータ関係に完全準拠した正負の調停
%             sigma_param = 1.0;    
%             epsilon_param = 2.0;   % γ(2.0) < 2*ε(4.0) となるよう論文Table 1に沿ってεを変更 [cite: 362, 372]
%             gamma_param = 2.0;    
%             denom = 4 * sigma_param * (epsilon_param - gamma_param/2 + 1e-5); % 正の値になる
% 
%             LgH_load_norm2 = norm(A_load_raw)^2;
%             Delta_load = LgH_load_norm2 / denom; 
% 
%             % 不等式 A_raw * u >= b_raw を quadprog の標準形 A_qp * u <= b_qp に変換
%             A_qp = -[A_load_raw; A_drone_raw]; 
%             b_qp = -[b_load_raw; b_drone_raw]; 
% 
%             % マージン Delta を安全側に適用
%             b_qp(1) = b_qp(1) - max(0, Delta_load); 
% 
%             % コスト関数 (公称加速度 u_nominal にできるだけ追従する)
%             H_mat = eye(3) * 2.0;
%             f_vec = -2.0 * u_nominal;
% 
%             options = optimoptions('quadprog', 'Display', 'off');
%             [u_safe, ~, exitflag] = quadprog(H_mat, f_vec, A_qp, b_qp, [], [], [], [], [], options);
%             u_safe = u_safe(:);
% 
%             if exitflag ~= 1
%                 % 緊急フォールバック
%                 u_safe = u_nominal; 
%             end
% 
%             % ⚙️ 詳細数値監視ログ（問題の根本原因追跡用）
%             fprintf('⚙️ [CBF ON] t: %.3f | h: %.2f | u_nom: [%.2f, %.2f, %.2f] | u_safe: [%.2f, %.2f, %.2f] | Delta: %.3e\n', ...
%                 xd(1), current_h_load_eval, u_nominal(1), u_nominal(2), u_nominal(3), u_safe(1), u_safe(2), u_safe(3), Delta_load);
%         else
%             if is_flight_now
%                 fprintf('🟢 [CBF FREE] t: %.3f | h: %.2f | u_nom: [%.2f, %.2f, %.2f]\n', xd(1), current_h_load_eval, u_nominal(1), u_nominal(2), u_nominal(3));
%             end
%             u_safe = u_nominal;
%         end
% 
%         % 🌟 4. UDEの積分メモリ更新タイミングを安全化
%         obj.input_integral = obj.input_integral + u_safe * Param.dt;
%         d_hat = (1 / T_ude) * (v_drone - obj.input_integral); 
% 
%         % UDEによる堅牢相殺入力を計算
%         u_robust = u_safe - d_hat; 
% 
%         % -----------------------------------------------------------------
%         % フェーズ④：加速度の格納 ＆ 高次微分の同調再合成 (元コードの手法をリスペクト)
%         % -----------------------------------------------------------------
%         vs_safe = vs_nominal;
%         vf_safe = vf_nominal;
%         vs_safe(1) = u_robust(1); 
%         vs_safe(2) = u_robust(2); 
%         vf_safe(1) = u_robust(3); 
%         ratio = vf_safe(1) / (vf_nominal(1) + 1e-6);
%         vf_safe(2:6) = vf_nominal(2:6) * ratio; 
% 
%         % -----------------------------------------------------------------
%         % フェーズ⑤：既存の非線形相殺・実物理入力への最終コンバート (元コードと完全同一)
%         % -----------------------------------------------------------------
%         uf = obj.Uf_SuspendedLoadxyDst(x, xd', vf_safe, P);
%         beta2 = obj.Beta2_SuspendedLoadxyDst(x, xd', vf_safe, P);
%         vs_alpha2 = obj.V2_alpha2_SuspendedLoadxyDst(x, xd', vf_safe, vs_safe', P);
%         us = beta2 \ vs_alpha2; 
%         tmp = [uf(1); us]; 
%         obj.result.tmp = tmp; 
% 
%         % 元コードと全く同じ実入力サチュレーション
%         obj.result.input = [max(0, min(20, tmp(1))); max(-1, min(1, tmp(2))); max(-1, min(1, tmp(3))); max(-1, min(1, tmp(4)))]; 
%         obj.result.xd = xd;
%         obj.result.x = x;
%         result = obj.result;
%     end
%     function result = show(obj)
%         obj.result
%     end
% end
% end


% classdef HLC_SUSPENDED_LOAD_AVOIDANCE < handle
% properties
%     self
%     result
%     param
%     % 🌟 積分メモリは明確にプロパティとして管理
%     input_integral = [0; 0; 0]; 
% end
% methods
%     function obj = HLC_SUSPENDED_LOAD_AVOIDANCE(self, param)
%         obj.self = self;
%         obj.param = param;
%     end
%     function result = do(obj, varargin)
%         Param = obj.param; 
%         model = obj.self.estimator.result; 
%         ref = obj.self.reference.result;   
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
%         % 🌟 1. パラメータ P は、元コードと全く同じ「行ベクトル」形状を厳格に死守
%         P = [obj.self.parameter.get(["mass", "jx", "jy", "jz", "gravity", "loadmass", "cableL"]), 0, 0];
%         x = [model.state.getq('compact'); model.state.w; pL; model.state.vL; pT; model.state.wL]; 
% 
%         yaw = wrapToPi(model.state.q(3)); 
%         yawd = xd(4); 
%         yawUnit = [cos(yaw); sin(yaw); 0]; 
%         yawdUnit = [cos(yawd); sin(yawd); 0]; 
%         deltaYaw = sign(cross(yawdUnit, yawUnit)) * acos(yawdUnit' * yawUnit); 
%         xd(4) = -deltaYaw(3) + yaw; 
%         xd = [xd; zeros(28 - size(xd, 1), 1)]; 
%         F1 = Param.F1; F2 = Param.F2; F3 = Param.F3; F4 = Param.F4; 
% 
%         % -----------------------------------------------------------------
%         % フェーズ①：公称名目入力（目標加速度）の計算 (元コードと完全同一)
%         % -----------------------------------------------------------------
%         vf_nominal = obj.Vfd_SuspendedLoadxyDst(Param.dt, x, xd', F1); 
%         vs_nominal = obj.Vs_SuspendedLoadxyDst(x, xd', vf_nominal, P, F2, F3, F4); 
%         u_nominal = [vs_nominal(1); vs_nominal(2); vf_nominal(1)]; 
%         u_nominal = u_nominal(:); 
% 
%         % -----------------------------------------------------------------
%         % 🌟 2. UDE外乱推定の同期・リセット処理の厳格化
%         % -----------------------------------------------------------------
%         v_drone = model.state.v; 
%         T_ude = 0.4; 
% 
%         % 初期化時、またはテイクオフ開始時は積分器を綺麗にリセット
%         if xd(1) <= Param.dt + 1e-5
%             obj.input_integral = [0; 0; 0];
%         end
% 
%         % -----------------------------------------------------------------
%         % フェーズ②：安全制約（CBF）フィルターの適用
%         % -----------------------------------------------------------------
%         obs_env = ENVIRONMENT_OBSTACLE();
%         ox = obs_env(1).p_obs(1);
%         oy = obs_env(1).p_obs(2);
%         oz = obs_env(1).p_obs(3);
%         ro = obs_env(1).r_obs;
% 
%         r_drone = 0.3; r_load = 0.2;  
%         c1 = 0.8; c2 = 2.5; 
%         d1 = 1.0; d2 = 2.0;
% 
%         % CBF関数専用の縦ベクトルパラメータを、元の P とは「別変数」として安全に分離
%         cbfParam_load  = [ox; oy; oz; ro; r_load;  c1; c2; P(:)];
%         cbfParam_drone = [ox; oy; oz; ro; r_drone; d1; d2; P(:)];
% 
%         is_flight_now = false;
%         if length(varargin) >= 2
%             if strcmp(varargin{2}, 'f') || string(varargin{2}) == "f"
%                 is_flight_now = true;
%             end
%         end
% 
%         current_h_load_eval = 0.5 * ((pL(1) - ox)^2 + (pL(2) - oy)^2 + (pL(3) - oz)^2 - (ro + r_load)^2);
% 
%         if is_flight_now && (current_h_load_eval <= 15.0)
%             t_current = 0.0; 
%             [A_load_raw,  b_load_raw]  = CBF_Constraints_Load(obj,  x, xd', u_nominal, cbfParam_load,  t_current);
%             [A_drone_raw, b_drone_raw] = CBF_Constraints_Drone(obj, x, xd', u_nominal, cbfParam_drone, t_current);
% 
%             % 🌟 3. 論文パラメータ関係に完全準拠した正負の調停
%             sigma_param = 1.0;    
%             epsilon_param = 2.0;   % γ(2.0) < 2*ε(4.0) となるよう論文Table 1に沿ってεを変更 [cite: 362, 372]
%             gamma_param = 2.0;    
%             denom = 4 * sigma_param * (epsilon_param - gamma_param/2 + 1e-5); % 正の値になる
% 
%             LgH_load_norm2 = norm(A_load_raw)^2;
%             Delta_load = LgH_load_norm2 / denom; 
% 
%             % 不等式 A_raw * u >= b_raw を quadprog の標準形 A_qp * u <= b_qp に変換
%             A_qp = -[A_load_raw; A_drone_raw]; 
%             b_qp = -[b_load_raw; b_drone_raw]; 
% 
%             % マージン Delta を安全側に適用
%             b_qp(1) = b_qp(1) - max(0, Delta_load); 
% 
%             % コスト関数 (公称加速度 u_nominal にできるだけ追従する)
%             H_mat = eye(3) * 2.0;
%             f_vec = -2.0 * u_nominal;
% 
%             options = optimoptions('quadprog', 'Display', 'off');
%             [u_safe, ~, exitflag] = quadprog(H_mat, f_vec, A_qp, b_qp, [], [], [], [], [], options);
%             u_safe = u_safe(:);
% 
%             if exitflag ~= 1
%                 % 緊急フォールバック
%                 u_safe = u_nominal; 
%             end
% 
%             % ⚙️ 詳細数値監視ログ（問題の根本原因追跡用）
%             fprintf('⚙️ [CBF ON] t: %.3f | h: %.2f | u_nom: [%.2f, %.2f, %.2f] | u_safe: [%.2f, %.2f, %.2f] | Delta: %.3e\n', ...
%                 xd(1), current_h_load_eval, u_nominal(1), u_nominal(2), u_nominal(3), u_safe(1), u_safe(2), u_safe(3), Delta_load);
%         else
%             if is_flight_now
%                 fprintf('🟢 [CBF FREE] t: %.3f | h: %.2f | u_nom: [%.2f, %.2f, %.2f]\n', xd(1), current_h_load_eval, u_nominal(1), u_nominal(2), u_nominal(3));
%             end
%             u_safe = u_nominal;
%         end
% 
%         % 🌟 4. UDEの積分メモリ更新タイミングを安全化
%         obj.input_integral = obj.input_integral + u_safe * Param.dt;
%         d_hat = (1 / T_ude) * (v_drone - obj.input_integral); 
% 
%         % UDEによる堅牢相殺入力を計算
%         u_robust = u_safe - d_hat; 
% 
%         % -----------------------------------------------------------------
%         % フェーズ④：加速度の格納 ＆ 高次微分の同調再合成（微分破綻の防止版）
%         % -----------------------------------------------------------------
%         vs_safe = vs_nominal;
%         vf_safe = vf_nominal;
% 
%         % 🌟 加速度コマンド（1成分目）は、QPが計算した安全な値（u_robust）を適用
%         vs_safe(1) = u_robust(1); 
%         vs_safe(2) = u_robust(2); 
%         vf_safe(1) = u_robust(3); 
% 
%         % 🌟【核心の修正】高次微分（2〜6成分目）を、比率（ratio）で無理に弄らず、
%         % 階層型線形化の相殺が崩れないよう、綺麗に連続している公称値（nominal）をそのまま維持する。
%         vf_safe(2:6) = vf_nominal(2:6);
% 
%         % -----------------------------------------------------------------
%         % フェーズ⑤：既存の非線形相殺・実物理入力への最終コンバート (元コードと完全同一)
%         % -----------------------------------------------------------------
%         uf = obj.Uf_SuspendedLoadxyDst(x, xd', vf_safe, P);
%         beta2 = obj.Beta2_SuspendedLoadxyDst(x, xd', vf_safe, P);
%         vs_alpha2 = obj.V2_alpha2_SuspendedLoadxyDst(x, xd', vf_safe, vs_safe', P);
%         us = beta2 \ vs_alpha2; 
%         tmp = [uf(1); us]; 
%         obj.result.tmp = tmp; 
% 
%         % 元コードと全く同じ実入力サチュレーション
%         obj.result.input = [max(0, min(20, tmp(1))); max(-1, min(1, tmp(2))); max(-1, min(1, tmp(3))); max(-1, min(1, tmp(4)))]; 
%         obj.result.xd = xd;
%         obj.result.x = x;
%         result = obj.result;
%     end
%     function result = show(obj)
%         obj.result
%     end
% end
% end

classdef HLC_SUSPENDED_LOAD_AVOIDANCE < handle
properties
    self
    result
    param
    % 🌟 積分メモリは明確にプロパティとして管理
    input_integral = [0; 0; 0]; 
end
methods
    function obj = HLC_SUSPENDED_LOAD_AVOIDANCE(self, param)
        obj.self = self;
        obj.param = param;
    end
    function result = do(obj, varargin)
        Param = obj.param; 
        model = obj.self.estimator.result; 
        ref = obj.self.reference.result;   
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
        
        % 🌟 1. パラメータ P は、元コードと全く同じ「行ベクトル」形状を厳格に死守
        P = [obj.self.parameter.get(["mass", "jx", "jy", "jz", "gravity", "loadmass", "cableL"]), 0, 0];
        x = [model.state.getq('compact'); model.state.w; pL; model.state.vL; pT; model.state.wL]; 
        
        yaw = wrapToPi(model.state.q(3)); 
        yawd = xd(4); 
        yawUnit = [cos(yaw); sin(yaw); 0]; 
        yawdUnit = [cos(yawd); sin(yawd); 0]; 
        deltaYaw = sign(cross(yawdUnit, yawUnit)) * acos(yawdUnit' * yawUnit); 
        xd(4) = -deltaYaw(3) + yaw; 
        xd = [xd; zeros(28 - size(xd, 1), 1)]; 
        F1 = Param.F1; F2 = Param.F2; F3 = Param.F3; F4 = Param.F4; 
        
        % -----------------------------------------------------------------
        % フェーズ①：公称名目入力（目標加速度）の計算 (元コードと完全同一)
        % -----------------------------------------------------------------
        vf_nominal = obj.Vfd_SuspendedLoadxyDst(Param.dt, x, xd', F1); 
        vs_nominal = obj.Vs_SuspendedLoadxyDst(x, xd', vf_nominal, P, F2, F3, F4); 
        u_nominal = [vs_nominal(1); vs_nominal(2); vf_nominal(1)]; 
        u_nominal = u_nominal(:); 
        
        % -----------------------------------------------------------------
        % 🌟 2. UDE外乱推定の同期・リセット処理の厳格化
        % -----------------------------------------------------------------
        v_drone = model.state.v; 
        T_ude = 0.4; 
        
        % 初期化時、またはテイクオフ開始時は積分器を綺麗にリセット
        if xd(1) <= Param.dt + 1e-5
            obj.input_integral = [0; 0; 0];
        end
        
        % -----------------------------------------------------------------
        % フェーズ②：安全制約（CBF）フィルターの適用
        % -----------------------------------------------------------------
        obs_env = ENVIRONMENT_OBSTACLE();
        ox = obs_env(1).p_obs(1);
        oy = obs_env(1).p_obs(2);
        oz = obs_env(1).p_obs(3);
        ro = obs_env(1).r_obs;
        
        r_drone = 0.3; r_load = 0.2;  
        c1 = 0.8; c2 = 2.5; 
        d1 = 1.0; d2 = 2.0;
        
        % CBF関数専用の縦ベクトルパラメータを、元の P とは「別変数」として安全に分離
        cbfParam_load  = [ox; oy; oz; ro; r_load;  c1; c2; P(:)];
        cbfParam_drone = [ox; oy; oz; ro; r_drone; d1; d2; P(:)];
        
        is_flight_now = false;
        if length(varargin) >= 2
            if strcmp(varargin{2}, 'f') || string(varargin{2}) == "f"
                is_flight_now = true;
            end
        end
        
        current_h_load_eval = 0.5 * ((pL(1) - ox)^2 + (pL(2) - oy)^2 + (pL(3) - oz)^2 - (ro + r_load)^2);
        
        if is_flight_now && (current_h_load_eval <= 15.0)
            t_current = 0.0; 
            [A_load_raw,  b_load_raw]  = CBF_Constraints_Load(obj,  x, xd', u_nominal, cbfParam_load,  t_current);
            [A_drone_raw, b_drone_raw] = CBF_Constraints_Drone(obj, x, xd', u_nominal, cbfParam_drone, t_current);
            
            % 🌟 3. 論文パラメータ関係に完全準拠した正負の調停
            sigma_param = 1.0;    
            epsilon_param = 2.0;   % γ(2.0) < 2*ε(4.0) [cite: 364]
            gamma_param = 2.0;    
            denom = 4 * sigma_param * (epsilon_param - gamma_param/2 + 1e-5); 
            
            LgH_load_norm2 = norm(A_load_raw)^2;
            Delta_load = LgH_load_norm2 / denom; 
            
            % 不等式 A_raw * u >= b_raw を quadprog の標準形 A_qp * u <= b_qp に変換
            A_qp = -[A_load_raw; A_drone_raw]; 
            b_qp = -[b_load_raw; b_drone_raw]; 
            
            % マージン Delta を安全側に適用
            b_qp(1) = b_qp(1) - max(0, Delta_load); 
            
            % コスト関数 (公称加速度 u_nominal にできるだけ追従する)
            H_mat = eye(3) * 2.0;
            f_vec = -2.0 * u_nominal;
            
            options = optimoptions('quadprog', 'Display', 'off');
            [u_safe, ~, exitflag] = quadprog(H_mat, f_vec, A_qp, b_qp, [], [], [], [], [], options);
            u_safe = u_safe(:);
            
            if exitflag ~= 1
                u_safe = u_nominal; 
            end
        else
            u_safe = u_nominal;
        end
        
        % 🌟 4. UDEの積分メモリ更新タイミングを安全化
        obj.input_integral = obj.input_integral + u_safe * Param.dt;
        d_hat = (1 / T_ude) * (v_drone - obj.input_integral); 
        
        % UDEによる堅牢相殺入力を計算
        u_robust = u_safe - d_hat; 
        
        % -----------------------------------------------------------------
        % フェーズ④：加速度の格納 ＆ 高次微分の同調再合成（微分破綻の防止版）
        % -----------------------------------------------------------------
        vs_safe = vs_nominal;
        vf_safe = vf_nominal;
        
        vs_safe(1) = u_robust(1); 
        vs_safe(2) = u_robust(2); 
        vf_safe(1) = u_robust(3); 
        
        % 高次微分は、相殺が崩れないよう綺麗に連続している公称値をそのまま維持
        vf_safe(2:6) = vf_nominal(2:6);
        
        % -----------------------------------------------------------------
        % フェーズ⑤：既存の非線形相殺・実物理入力への最終コンバート
        % -----------------------------------------------------------------
        uf = obj.Uf_SuspendedLoadxyDst(x, xd', vf_safe, P);
        beta2 = obj.Beta2_SuspendedLoadxyDst(x, xd', vf_safe, P);
        vs_alpha2 = obj.V2_alpha2_SuspendedLoadxyDst(x, xd', vf_safe, vs_safe', P);
        
        % 🔍【追加：特異点解析デバッグコード】
        norm_beta2 = norm(beta2);
        cond_beta2 = cond(beta2); % 行列の悪条件度（特異点判定）
        norm_vs_alpha2 = norm(vs_alpha2);
        
        us = beta2 \ vs_alpha2; 
        tmp = [uf(1); us]; 
        obj.result.tmp = tmp; 
        
        % 🔍【追加：内部状態完全可視化ロギング】
        if is_flight_now && (current_h_load_eval <= 15.0)
            fprintf('\n================== 🔍 CONVERT MATRIX ANALYSIS ==================\n');
            fprintf('◆ [Time]         t             : %.3f\n', xd(1));
            fprintf('◆ [QP Output]    u_safe        : [%.2f, %.2f, %.2f]\n', u_safe(1), u_safe(2), u_safe(3));
            fprintf('◆ [Linearization]norm(vs_a2)   : %.3e\n', norm_vs_alpha2);
            fprintf('◆ [Matrix State] norm(beta2)   : %.3e  | Cond(beta2): %.3e\n', norm_beta2, cond_beta2);
            fprintf('◆ [Raw Hardware] 生Thrust(uf)  : %5.2f  | 生Torque(us): [%.2f, %.2f, %.2f]\n', tmp(1), tmp(2), tmp(3), tmp(4));
            fprintf('========================================================================\n\n');
        else
            if is_flight_now
                fprintf('🟢 [CBF FREE] t: %.3f | h: %.2f | u_nom: [%.2f, %.2f, %.2f]\n', xd(1), current_h_load_eval, u_nominal(1), u_nominal(2), u_nominal(3));
            end
        end
        
        % 元コードと全く同じ実入力サチュレーション
        obj.result.input = [max(0, min(20, tmp(1))); max(-1, min(1, tmp(2))); max(-1, min(1, tmp(3))); max(-1, min(1, tmp(4)))]; 
        obj.result.xd = xd;
        obj.result.x = x;
        result = obj.result;
    end
    function result = show(obj)
        obj.result
    end
end
end
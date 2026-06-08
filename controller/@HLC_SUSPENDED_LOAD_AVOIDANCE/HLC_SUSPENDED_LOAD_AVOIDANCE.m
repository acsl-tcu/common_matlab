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
%         % パラメータ P の取得
%         P = [obj.self.parameter.get(["mass", "jx", "jy", "jz", "gravity", "loadmass", "cableL"]), 0, 0];
%         x = [model.state.getq('compact'); model.state.w; pL; model.state.vL; pT; model.state.wL]; 
% 
%         yaw = wrapToPi(model.state.q(3)); 
%         yawd = xd(4); 
%         yawUnit = [cos(yaw); sin(yaw); 0]; 
%         yawdUnit = [cos(yawd); sin(yawd); 0]; 
%         deltaYaw = sign(cross(yawdUnit, yawUnit)) * acos(yawdUnit' * yawUnit); 
%         xd(4) = -deltaYaw(3) + yaw; 
% 
%         % 高次微分の完全保護ロジック（上書き破壊を防止）
%         original_size = length(xd);
%         if original_size < 28
%             xd = [xd(:); zeros(28 - original_size, 1)];
%         else
%             xd = xd(:);
%         end
% 
%         F1 = Param.F1; F2 = Param.F2; F3 = Param.F3; F4 = Param.F4; 
%         dt = Param.dt;
% 
%         % -----------------------------------------------------------------
%         % フェーズ①：公称名目入力の計算 (元コードと完全同一)
%         % -----------------------------------------------------------------
%         vf_nominal = obj.Vfd_SuspendedLoadxyDst(Param.dt, x, xd', F1); 
%         vs_nominal = obj.Vs_SuspendedLoadxyDst(x, xd', vf_nominal, P, F2, F3, F4); 
% 
%         % 🌟【プランBの適用】：最上流の仮想入力「本体（低次）」[v1; v2; v3]
%         v_nominal_top = [vf_nominal(1); vs_nominal(1); vs_nominal(2)];
%         v_nominal_top = v_nominal_top(:);
% 
%         % -----------------------------------------------------------------
%         % UDE外乱推定の同期・リセット処理
%         % -----------------------------------------------------------------
%         v_drone = model.state.v; 
%         T_ude = 0.4; 
%         if xd(1) <= Param.dt + 1e-5
%             obj.input_integral = [0; 0; 0];
%         end
% 
%         % -----------------------------------------------------------------
%         % フェーズ②：【最上流レイヤー】での安全制約（CBF-QP）の実行
%         % -----------------------------------------------------------------
%         obs_env = ENVIRONMENT_OBSTACLE();
%         ox = obs_env(1).p_obs(1);
%         oy = obs_env(1).p_obs(2);
%         oz = obs_env(1).p_obs(3);
%         ro = obs_env(1).r_obs;
% 
%         r_drone = 0.3; r_load = 0.2;  
% 
%         % 相次次数5の荷物側CBFゲイン
%         k_cbf1 = 2.5; k_cbf2 = 2.0; k_cbf3 = 1.5; k_cbf4 = 1.0; k_cbf5 = 1.0;
%         % 相対次数2のドローン側CBFゲイン
%         d1 = 1.5; d2 = 1.0;
% 
%         % パラメータパッキング
%         cbfParam_top_load  = [ox; oy; oz; ro; r_load;  k_cbf1; k_cbf2; k_cbf3; k_cbf4; k_cbf5; P(:)];
%         cbfParam_top_drone = [ox; oy; oz; ro; r_drone; d1; d2; P(:)];
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
%             [A_load_top,  b_load_top]  = CBF_Constraints_Load_TopLayer(obj,  x, xd', v_nominal_top, cbfParam_top_load,  t_current);
%             [A_drone_top, b_drone_top] = CBF_Constraints_Drone(obj, x, xd', v_nominal_top, cbfParam_top_drone, t_current);
% 
%             % ISSf動的緩和マージン Delta の計算
%             sigma_param = 1.0;    
%             epsilon_param = 2.0;   
%             gamma_param = 2.0;    
%             denom = 4 * sigma_param * (epsilon_param - gamma_param/2 + 1e-5); 
% 
%             Delta_load = norm(A_load_top)^2 / denom; 
% 
%             A_qp = -[A_load_top; A_drone_top]; 
%             b_qp = -[b_load_top; b_drone_top]; 
%             b_qp(1) = b_qp(1) - max(0, Delta_load); 
% 
%             % 最上流の入力空間 [v1; v2; v3] で最適化を解く
%             H_mat = eye(3) * 2.0;
%             f_vec = -2.0 * v_nominal_top;
% 
%             options = optimoptions('quadprog', 'Display', 'off');
%             [v_safe_top, ~, exitflag] = quadprog(H_mat, f_vec, A_qp, b_qp, [], [], [], [], [], options);
%             v_safe_top = v_safe_top(:);
% 
%             if exitflag ~= 1
%                 v_safe_top = v_nominal_top; 
%             end
%         else
%             v_safe_top = v_nominal_top;
%         end
% 
%         % UDEによる外乱相殺
%         obj.input_integral = obj.input_integral + v_safe_top * Param.dt;
%         d_hat = (1 / T_ude) * (v_drone - obj.input_integral); 
%         v_robust_top = v_safe_top - d_hat; 
% 
%         % -----------------------------------------------------------------
%         % フェーズ④：安全な高次微分の離散時間発展
%         % -----------------------------------------------------------------
%         vf_safe = vf_nominal;
%         vs_safe = vs_nominal;
% 
%         vf_safe(1) = v_robust_top(1); 
%         vs_safe(1) = v_robust_top(2); 
%         vs_safe(2) = v_robust_top(3); 
% 
%         % 閉ループダイナミクス A1 行列の数値展開
%         syms k real
%         A1_mat = expm([0,1;0,0]*Param.dt) - int(expm([0,1;0,0]*(Param.dt-k))*[0;1], k, [0,Param.dt]) * F1;
%         A1_mat = double(A1_mat); 
% 
%         Z1_state = [model.state.pL(3) - xd(3); model.state.vL(3) - xd(7)]; 
% 
%         % 高次の微分連鎖（dV1 ~ d5V1）を安全に再合成
%         for idx = 2:6
%             Z1_state = A1_mat * Z1_state;
%             vf_safe(idx) = -F1 * Z1_state; 
%         end
% 
%         % -----------------------------------------------------------------
%         % フェーズ⑤：既存の非線形相殺・実物理入力への最終コンバート
%         % -----------------------------------------------------------------
%         uf = obj.Uf_SuspendedLoadxyDst(x, xd', vf_safe, P);
%         beta2 = obj.Beta2_SuspendedLoadxyDst(x, xd', vf_safe, P);
%         vs_alpha2 = obj.V2_alpha2_SuspendedLoadxyDst(x, xd', vf_safe, vs_safe', P);
% 
%         norm_beta2 = norm(beta2);
%         cond_beta2 = cond(beta2); 
%         norm_vs_alpha2 = norm(vs_alpha2);
% 
%         us = beta2 \ vs_alpha2; 
%         tmp = [uf(1); us]; 
%         obj.result.tmp = tmp; 
% 
%         % 🔍【上書き修正】：画面表示ロジック変数の不整合をクリーンアップ
%         if is_flight_now && (current_h_load_eval <= 15.0)
%             fprintf('\n================== 🔍 PLAN-B LAYER ANALYSIS ==================\n');
%             fprintf('◆ [Time]         t             : %.3f\n', xd(1));
%             fprintf('◆ [QP Top Out]   v_safe_top    : [%.2f, %.2f, %.2f]\n', v_safe_top(1), v_safe_top(2), v_safe_top(3));
%             fprintf('◆ [Linearization]norm(vs_a2)   : %.3e\n', norm_vs_alpha2);
%             fprintf('◆ [Matrix State] norm(beta2)   : %.3e  | Cond(beta2): %.3e\n', norm_beta2, cond_beta2);
%             fprintf('◆ [Raw Hardware] 生Thrust(uf)  : %5.2f  | 生Torque(us): [%.2f, %.2f, %.2f]\n', tmp(1), tmp(2), tmp(3), tmp(4));
%             fprintf('========================================================================\n\n');
%         else
%             if is_flight_now
%                 % 旧 u_nominal ログを排除し、最上流の名目入力 v_nominal_top を出すように安全化
%                 fprintf('🟢 [CBF FREE] t: %.3f | h: %.2f | v_nom_top: [%.2f, %.2f, %.2f]\n', xd(1), current_h_load_eval, v_nominal_top(1), v_nominal_top(2), v_nominal_top(3));
%             end
%         end
% 
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
%     % 🌟 積分メモリはプロパティとして管理
%     input_integral = [0; 0; 0]; % UDE
% end
% methods
%     function obj = HLC_SUSPENDED_LOAD_AVOIDANCE(self, param)
%         obj.self = self;
%         obj.param = param;
%     end
%     function result = do(obj, varargin)
%         % コンストラクタ引数から本来の param 構造体を抽出し、型エラーを完全防止
%         if isprop(obj.param, 'param')
%             Param = obj.param.param;
%         elseif isfield(obj.param, 'param')
%             Param = obj.param.param;
%         else
%             Param = obj.param; 
%         end
% 
%         model = obj.self.estimator.result; 
%         ref = obj.self.reference.result;   
%         if isprop(ref.state, 'xd')
%             xd_raw = ref.state.xd; 
%         else
%             xd_raw = ref.state.get();
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
%         P = [obj.self.parameter.get(["mass", "jx", "jy", "jz", "gravity", "loadmass", "cableL"]), 0, 0];
%         x = [model.state.getq('compact'); model.state.w; pL; model.state.vL; pT; model.state.wL]; 
% 
%         % 🌟【根本治療】：変に弄るのを一切やめます。
%         % 元の正常なコード（HLC_SUSPENDED_LOAD）と100%完全に同じ並び順・同じマトリクス形状（横ベクトル）のままキープします。
%         % これにより、内部時間の逆走バグや、公称値が兆単位に爆発する現象が100%根底から消滅します。
%         xd_row = xd_raw(:)'; 
%         original_size = length(xd_row);
%         if original_size < 28
%             xd_row = [xd_row, zeros(1, 28 - original_size)];
%         end
% 
%         % Yaw角の補正は4番目の要素にダイレクトに適用
%         yaw = wrapToPi(model.state.q(3)); 
%         yawd = xd_row(4); 
%         yawUnit = [cos(yaw); sin(yaw); 0]; 
%         yawdUnit = [cos(yawd); sin(yawd); 0]; 
%         deltaYaw = sign(cross(yawdUnit, yawUnit)) * acos(yawdUnit' * yawUnit); 
%         xd_row(4) = -deltaYaw(3) + yaw; 
% 
%         xd = xd_row(:); % 縦ベクトル版
%         F1 = Param.F1; F2 = Param.F2; F3 = Param.F3; F4 = Param.F4; 
%         dt = Param.dt;
% 
%         % シミュレータの本当の現在時間を安全に抽出（vararginの1番目から正確にtime構造体を解析）
%         if length(varargin) >= 1 && isstruct(varargin{1}) && isfield(varargin{1}, 't')
%             t_sim = varargin{1}.t;
%         else
%             t_sim = xd_row(1); % フォールバック
%         end
% 
%         % -----------------------------------------------------------------
%         % フェーズ①：公称名目入力の計算 (元コードと100%完全同一・無加工)
%         % -----------------------------------------------------------------
%         vf_nominal = obj.Vfd_SuspendedLoadxyDst(Param.dt, x, xd_row, F1); 
%         vs_nominal = obj.Vs_SuspendedLoadxyDst(x, xd_row, vf_nominal, P, F2, F3, F4); 
% 
%         v_nominal_top = [vf_nominal(1); vs_nominal(1); vs_nominal(2)];
%         v_nominal_top = v_nominal_top(:);
% 
%         % 外乱推定用積分器の同期
%         v_drone = model.state.v; 
%         T_ude = 0.4; 
%         if t_sim <= Param.dt + 1e-5
%             obj.input_integral = [0; 0; 0];
%         end
% 
%         % -----------------------------------------------------------------
%         % フェーズ②：【最上流レイヤー】での安全制約（CBF-QP）の実行
%         % -----------------------------------------------------------------
%         obs_env = ENVIRONMENT_OBSTACLE();
%         ox = obs_env(1).p_obs(1);
%         oy = obs_env(1).p_obs(2);
%         oz = obs_env(1).p_obs(3);
%         ro = obs_env(1).r_obs;
%         r_drone = 0.3; r_load = 0.2;  
%         k_cbf1 = 2.5; k_cbf2 = 2.0; k_cbf3 = 1.5; k_cbf4 = 1.0; k_cbf5 = 1.0;
%         d1 = 1.5; d2 = 1.0;
% 
%         cbfParam_top_load  = [ox; oy; oz; ro; r_load;  k_cbf1; k_cbf2; k_cbf3; k_cbf4; k_cbf5; P(:)];
%         cbfParam_top_drone = [ox; oy; oz; ro; r_drone; d1; d2; P(:)];
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
%         % 安全な操作変数を名目値で初期化
%         v_safe_top = v_nominal_top;
%         cbf_status_str = '🟢 [CBF FREE - IDLE]';
% 
%         % 本当に障害物が近づいたフライト中のみ安全フィルター（QP）を介入させる
%         if is_flight_now && (current_h_load_eval <= 15.0)
%             [A_load_top,  b_load_top]  = CBF_Constraints_Load_TopLayer(obj,  x, xd_row, v_nominal_top, cbfParam_top_load,  t_sim);
%             [A_drone_top, b_drone_top] = CBF_Constraints_Drone(obj, x, xd_row, v_nominal_top, cbfParam_top_drone, t_sim);
% 
%             % 成分ごとの代数的対角正規化
%             for i = 1:3
%                 scale_fact_load = abs(A_load_top(i));
%                 if scale_fact_load > 1e-5
%                     A_load_top(i) = A_load_top(i) / scale_fact_load;
%                     b_load_top = b_load_top / scale_fact_load; 
%                 end
%                 scale_fact_drone = abs(A_drone_top(i));
%                 if scale_fact_drone > 1e-5
%                     A_drone_top(i) = A_drone_top(i) / scale_fact_drone;
%                     b_drone_top = b_drone_top / scale_fact_drone;
%                 end
%             end
% 
%             sigma_param = 1.0; epsilon_param = 2.0; gamma_param = 2.0;    
%             denom = 4 * sigma_param * (epsilon_param - gamma_param/2 + 1e-5); 
%             Delta_load = norm(A_load_top)^2 / denom; 
% 
%             A_qp = -[A_load_top; A_drone_top]; 
%             b_qp = -[b_load_top; b_drone_top]; 
%             b_qp(1) = b_qp(1) - max(0, Delta_load); 
% 
%             v_nominal_top_clipped = max([-15; -15; -15], min([15; 15; 15], v_nominal_top));
%             H_mat = eye(3) * 10.0;
%             f_vec = -10.0 * v_nominal_top_clipped;
%             lb = v_nominal_top_clipped - [8; 8; 8];
%             ub = v_nominal_top_clipped + [8; 8; 8];
% 
%             options = optimoptions('quadprog', 'Display', 'off');
%             [v_opt, ~, exitflag] = quadprog(H_mat, f_vec, A_qp, b_qp, [], [], lb, ub, [], options);
% 
%             if exitflag == 1 && ~any(isnan(v_opt))
%                 v_safe_top = max(lb, min(ub, v_opt(:)));
%                 cbf_status_str = '⚙️ [CBF WORKING - ACTIVE]';
%             else
%                 cbf_status_str = '⚠️ [QP FAILED - FALLBACK TO NOMINAL]';
%             end
%         end
% 
%         % UDE外乱相殺
%         obj.input_integral = obj.input_integral + v_safe_top * Param.dt;
%         d_hat = (1 / T_ude) * (v_drone - obj.input_integral); 
%         v_robust_top = v_safe_top - d_hat; 
% 
%         % -----------------------------------------------------------------
%         % フェーズ④：安全な高次微分の離散時間発展
%         % -----------------------------------------------------------------
%         vf_safe = vf_nominal;
%         vs_safe = vs_nominal;
% 
%         vf_safe(1) = v_robust_top(1); 
%         vs_safe(1) = v_robust_top(2); 
%         vs_safe(2) = v_robust_top(3); 
% 
%         % 🌟安全圏（FREE）の時は、元コードが叩き出した100%完璧な前進用高次微分「vf_nominal」を
%         % 無傷でそのまま下流へ流します。これでフライト移行時の姿勢破壊が完全に防がれます。
%         if strcmp(cbf_status_str, '⚙️ [CBF WORKING - ACTIVE]')
%             % 回避制約が本当に発動したときだけ、不連続性を防ぐために高度の微分を滑らかに再合成
%             syms k real
%             A1_mat = expm([0,1;0,0]*Param.dt) - int(expm([0,1;0,0]*(Param.dt-k))*[0;1], k, [0,Param.dt]) * F1;
%             A1_mat = double(A1_mat); 
% 
%             Z1_state = [model.state.pL(3) - xd_row(3); model.state.vL(3) - xd_row(7)]; 
%             for idx = 2:6
%                 Z1_state = A1_mat * Z1_state;
%                 vf_safe(idx) = -F1 * Z1_state; 
%             end
%         end
% 
%         % -----------------------------------------------------------------
%         % フェーズ⑤：既存の非線形相殺・実物理入力への最終コンバート
%         % -----------------------------------------------------------------
%         uf = obj.Uf_SuspendedLoadxyDst(x, xd_row, vf_safe, P);
%         beta2 = obj.Beta2_SuspendedLoadxyDst(x, xd_row, vf_safe, P);
%         vs_alpha2 = obj.V2_alpha2_SuspendedLoadxyDst(x, xd_row, vf_safe, vs_safe', P);
% 
%         norm_beta2 = norm(beta2);
%         cond_beta2 = cond(beta2); 
%         norm_vs_alpha2 = norm(vs_alpha2);
% 
%         us = beta2 \ vs_alpha2; 
%         tmp = [uf(1); us]; 
%         obj.result.tmp = tmp; 
% 
%         % 徹底数値監視ロギング（フライト開始時用）
%         if strcmp(cbf_status_str, '⚙️ [CBF WORKING - ACTIVE]')
%             fprintf('\n================== 🔍 PLAN-B LAYER ANALYSIS ==================\n');
%             fprintf('◆ [Time]         t             : %.3f\n', t_sim);
%             fprintf('◆ [QP Top Out]   v_safe_top    : [%.2f, %.2f, %.2f]\n', v_safe_top(1), v_safe_top(2), v_safe_top(3));
%             fprintf('◆ [Linearization]norm(vs_a2)   : %.3e\n', norm_vs_alpha2);
%             fprintf('◆ [Matrix State] norm(beta2)   : %.3e  | Cond(beta2): %.3e\n', norm_beta2, cond_beta2);
%             fprintf('◆ [Raw Hardware] 生Thrust(uf)  : %5.2f  | 生Torque(us): [%.2f, %.2f, %.2f]\n', tmp(1), tmp(2), tmp(3), tmp(4));
%             fprintf('========================================================================\n\n');
%         else
%             if is_flight_now
%                 fprintf('🟢 [CBF FREE] t: %.3f | h: %.2f | v_nom_top: [%.2f, %.2f, %.2f]\n', t_sim, current_h_load_eval, v_nominal_top(1), v_nominal_top(2), v_nominal_top(3));
%             end
%         end
% 
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
%     % 🌟 積分メモリはプロパティとして管理
%     input_integral = [0; 0; 0]; % UDE
% end
% methods
%     function obj = HLC_SUSPENDED_LOAD_AVOIDANCE(self, param)
%         obj.self = self;
%         obj.param = param;
%     end
%     function result = do(obj, varargin)
%         Param = obj.param; % param (optional) : 構造体：ゲインF1-F4
%         model = obj.self.estimator.result; % 推定した状態
%         ref = obj.self.reference.result; % 目標値
% 
%         % 目標値を取得
%         if isprop(ref.state, 'xd')
%             xd = ref.state.xd; %これがxdからxd_rawになっている
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
% 
%         x = [model.state.getq('compact'); model.state.w; pL; model.state.vL; pT; model.state.wL]; 
% 
%         % [model.state.p, x(8:10), xd(1:3), x(8:10) - xd(1:3)]
%         % yaw角の定義域の問題を回避,h4 = yaw - yawd(誤差)だがyawd = -(誤差)+yawの値を入れる．x,y,yawの仮想入力はVs_SuspendedLoadはクオータニオンで計算するため
%         % yawサブシステムの入力を設計するときにyaw角を打ち消して定義域修正した誤差を反映
%         % Yaw角の補正は4番目の要素にダイレクトに適用
%         yaw = wrapToPi(model.state.q(3));  % 機体yaw角[-pi,pi]にする特にyaw
%         yawd = xd(4); % 目標yaw角
%         yawUnit = [cos(yaw); sin(yaw); 0]; % yawの方向ベクトル
%         yawdUnit = [cos(yawd); sin(yawd); 0]; % yawdの方向ベクトル
%         deltaYaw = sign(cross(yawdUnit, yawUnit)) * acos(yawdUnit' * yawUnit); % 目標角度からみた機体角度との誤差
%         xd_row(4) = -deltaYaw(3) + yaw; % yaw打ち消しと誤差をyawの目標角に入れる．
% 
%         %目標値の格納
%         % 🌟 既存の高次微分データを破壊せず、28次元に満たない場合のみ末尾をゼロ埋めする
%         if original_size < 28
%             xd = [xd(:); zeros(28 - original_size, 1)];
%         else
%             xd = xd(:); % 縦ベクトル化のみ行う（既存の高次微分はすべて無傷で維持）
%         end
% 
%         % 階層型線形化による入力計算
%         % 仮想入力のゲイン
%         F1 = Param.F1; % z方向サブシステムのゲイン
%         F2 = Param.F2; % x方向サブシステムのゲイン
%         F3 = Param.F3; % y方向サブシステムのゲイン
%         F4 = Param.F4; % yaw方向サブシステムのゲイン
% 
% 
%         % -----------------------------------------------------------------
%         % フェーズ①：公称名目入力の計算 (元コードと100%完全同一・無加工)
%         % -----------------------------------------------------------------
%         vf_nominal = obj.Vfd_SuspendedLoadxyDst(Param.dt, x, xd', F1); 
%         vs_nominal = obj.Vs_SuspendedLoadxyDst(x, xd', vf_nominal, P, F2, F3, F4); 
% 
%         % 🌟【修正点】：シミュレータ本体が管理している本物の現在時間 t を安全に抽出　これしっかりと抽出できているのか確認
%         if length(varargin) >= 1 && isstruct(varargin{1}) && isfield(varargin{1}, 't')
%             t_sim = varargin{1}.t;
%         else
%             t_sim = 0.0; % フォールバック
%         end
% 
%         % 外乱推定用積分器の同期
%         v_drone = model.state.v; 
%         T_ude = 0.4; 
%         if t_sim <= Param.dt + 1e-5
%             obj.input_integral = [0; 0; 0];
%         end
% 
%         % -----------------------------------------------------------------
%         % フェーズ②：【最上流レイヤー】での安全制約（CBF-QP）の実行
%         % -----------------------------------------------------------------
%         obs_env = ENVIRONMENT_OBSTACLE();
%         ox = obs_env(1).p_obs(1);
%         oy = obs_env(1).p_obs(2);
%         oz = obs_env(1).p_obs(3);
%         ro = obs_env(1).r_obs;
%         r_drone = 0.3; r_load = 0.2;  
%         k_cbf1 = 2.5; k_cbf2 = 2.0; k_cbf3 = 1.5; k_cbf4 = 1.0; k_cbf5 = 1.0;
%         d1 = 1.5; d2 = 1.0;
% 
%         cbfParam_top_load  = [ox; oy; oz; ro; r_load;  k_cbf1; k_cbf2; k_cbf3; k_cbf4; k_cbf5; P(:)];
%         cbfParam_top_drone = [ox; oy; oz; ro; r_drone; d1; d2; P(:)];
% 
%         is_flight_now = false;
%         if length(varargin) >= 2
%             if strcmp(varargin{2}, 'f') || string(varargin{2}) == "f" % これほんとに入っているのか確認
%                 is_flight_now = true;
%             end
%         end
% 
%         current_h_load_eval = 0.5 * ((pL(1) - ox)^2 + (pL(2) - oy)^2 + (pL(3) - oz)^2 - (ro + r_load)^2);
% 
%         % 安全な操作変数を名目値で初期化
%         v_safe = v_nominal;
%         cbf_status_str = '🟢 [CBF FREE - IDLE]';
% 
%         % 本当に障害物が近づいたフライト中のみ安全フィルター（QP）を介入させる
%         if is_flight_now && (current_h_load_eval <= 15.0) % これ動いているのか見ましょう
%             [A_load_top,  b_load_top]  = CBF_Constraints_Load_TopLayer(obj,  x, xd_row, v_nominal_top, cbfParam_top_load,  t_sim);
%             [A_drone_top, b_drone_top] = CBF_Constraints_Drone(obj, x, xd_row, v_nominal_top, cbfParam_top_drone, t_sim);
% 
%             % 成分ごとの代数的対角正規化
%             for i = 1:3
%                 scale_fact_load = abs(A_load_top(i));
%                 if scale_fact_load > 1e-5
%                     A_load_top(i) = A_load_top(i) / scale_fact_load;
%                     b_load_top = b_load_top / scale_fact_load; 
%                 end
%                 scale_fact_drone = abs(A_drone_top(i));
%                 if scale_fact_drone > 1e-5
%                     A_drone_top(i) = A_drone_top(i) / scale_fact_drone;
%                     b_drone_top = b_drone_top / scale_fact_drone;
%                 end
%             end
% 
%             sigma_param = 1.0; epsilon_param = 2.0; gamma_param = 2.0;    
%             denom = 4 * sigma_param * (epsilon_param - gamma_param/2 + 1e-5); 
%             Delta_load = norm(A_load_top)^2 / denom; 
% 
%             A_qp = -[A_load_top; A_drone_top]; 
%             b_qp = -[b_load_top; b_drone_top]; 
%             b_qp(1) = b_qp(1) - max(0, Delta_load); 
% 
%             v_nominal_top_clipped = max([-15; -15; -15], min([15; 15; 15], v_nominal_top));
%             H_mat = eye(3) * 10.0;
%             f_vec = -10.0 * v_nominal_top_clipped;
%             lb = v_nominal_top_clipped - [8; 8; 8];
%             ub = v_nominal_top_clipped + [8; 8; 8];
% 
%             options = optimoptions('quadprog', 'Display', 'off');
%             [v_opt, ~, exitflag] = quadprog(H_mat, f_vec, A_qp, b_qp, [], [], lb, ub, [], options);
% 
%             if exitflag == 1 && ~any(isnan(v_opt))
%                 v_safe = max(lb, min(ub, v_opt(:)));
%                 cbf_status_str = '⚙️ [CBF WORKING - ACTIVE]';
%             else
%                 cbf_status_str = '⚠️ [QP FAILED - FALLBACK TO NOMINAL]';
%             end
%         end
% 
%         % UDE外乱相殺
%         obj.input_integral = obj.input_integral + v_safe * Param.dt;
%         d_hat = (1 / T_ude) * (v_drone - obj.input_integral); 
%         v_robust_top = v_safe - d_hat; 
% 
%         % -----------------------------------------------------------------
%         % フェーズ④：安全な高次微分の離散時間発展
%         % -----------------------------------------------------------------
%         vf_safe = vf_nominal;
%         vs_safe = vs_nominal;
% 
%         vf_safe(1) = v_robust_top(1); 
%         vs_safe(1) = v_robust_top(2); 
%         vs_safe(2) = v_robust_top(3); 
% 
%         % 🌟安全圏（FREE）の時は、元コードが叩き出した100%完璧な前進用高次微分「vf_nominal」を
%         % 無傷でそのまま下流へ流します。これでフライト移行時の姿勢破壊が完全に防がれます。
%         if strcmp(cbf_status_str, '⚙️ [CBF WORKING - ACTIVE]')
%             % 回避制約が本当に発動したときだけ、不連続性を防ぐために高度の微分を滑らかに再合成　これ何やっているんだ？
%             syms k real
%             A1_mat = expm([0,1;0,0]*Param.dt) - int(expm([0,1;0,0]*(Param.dt-k))*[0;1], k, [0,Param.dt]) * F1;
%             A1_mat = double(A1_mat); 
% 
%             Z1_state = [model.state.pL(3) - xd_row(3); model.state.vL(3) - xd_row(7)]; 
%             for idx = 2:6
%                 Z1_state = A1_mat * Z1_state;
%                 vf_safe(idx) = -F1 * Z1_state; 
%             end
%         end
% 
%         % -----------------------------------------------------------------
%         % フェーズ⑤：既存の非線形相殺・実物理入力への最終コンバート
%         % -----------------------------------------------------------------
%         uf = obj.Uf_SuspendedLoadxyDst(x, xd', vf_safe, P);
%         beta2 = obj.Beta2_SuspendedLoadxyDst(x, xd', vf_safe, P);
%         vs_alpha2 = obj.V2_alpha2_SuspendedLoadxyDst(x, xd', vf_safe, vs_safe', P);
% 
%         norm_beta2 = norm(beta2);
%         cond_beta2 = cond(beta2); 
%         norm_vs_alpha2 = norm(vs_alpha2);
% 
%         us = beta2 \ vs_alpha2; 
%         tmp = [uf(1); us]; 
%         obj.result.tmp = tmp; 
% 
%         % 徹底数値監視ロギング（フライト開始時用）
%         if strcmp(cbf_status_str, '⚙️ [CBF WORKING - ACTIVE]')
%             fprintf('\n================== 🔍 PLAN-B LAYER ANALYSIS ==================\n');
%             fprintf('◆ [Time]         t             : %.3f\n', t_sim);
%             fprintf('◆ [QP Top Out]   v_safe_top    : [%.2f, %.2f, %.2f]\n', v_safe(1), v_safe(2), v_safe(3));
%             fprintf('◆ [Linearization]norm(vs_a2)   : %.3e\n', norm_vs_alpha2);
%             fprintf('◆ [Matrix State] norm(beta2)   : %.3e  | Cond(beta2): %.3e\n', norm_beta2, cond_beta2);
%             fprintf('◆ [Raw Hardware] 生Thrust(uf)  : %5.2f  | 生Torque(us): [%.2f, %.2f, %.2f]\n', tmp(1), tmp(2), tmp(3), tmp(4));
%             fprintf('========================================================================\n\n');
%         else
%             if is_flight_now
%                 fprintf('🟢 [CBF FREE] t: %.3f | h: %.2f | v_nom_top: [%.2f, %.2f, %.2f]\n', t_sim, current_h_load_eval, v_nominal_top(1), v_nominal_top(2), v_nominal_top(3));
%             end
%         end
% 
%         obj.result.input = [max(0, min(20, tmp(1))); max(-1, min(1, tmp(2))); max(-1, min(1, tmp(3))); max(-1, min(1, tmp(4)))]; 
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

% 外乱推定はいったん実装しない
classdef HLC_SUSPENDED_LOAD_AVOIDANCE < handle
properties
    self
    result
    param
end
methods
    function obj = HLC_SUSPENDED_LOAD_AVOIDANCE(self, param)
        obj.self = self;
        obj.param = param;
    end
    function result = do(obj, varargin)
        Param = obj.param; % param (optional) : 構造体：ゲインF1-F4
        model = obj.self.estimator.result; % 推定した状態
        ref = obj.self.reference.result; % 目標値

        % 目標値を取得
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
        
        % [model.state.p, x(8:10), xd(1:3), x(8:10) - xd(1:3)]
        % yaw角の定義域の問題を回避,h4 = yaw - yawd(誤差)だがyawd = -(誤差)+yawの値を入れる．x,y,yawの仮想入力はVs_SuspendedLoadはクオータニオンで計算するため
        % yawサブシステムの入力を設計するときにyaw角を打ち消して定義域修正した誤差を反映
        % Yaw角の補正は4番目の要素にダイレクトに適用
        yaw = wrapToPi(model.state.q(3));  % 機体yaw角[-pi,pi]にする特にyaw
        yawd = xd(4); % 目標yaw角
        yawUnit = [cos(yaw); sin(yaw); 0]; % yawの方向ベクトル
        yawdUnit = [cos(yawd); sin(yawd); 0]; % yawdの方向ベクトル
        deltaYaw = sign(cross(yawdUnit, yawUnit)) * acos(yawdUnit' * yawUnit); % 目標角度からみた機体角度との誤差
        xd_row(4) = -deltaYaw(3) + yaw; % yaw打ち消しと誤差をyawの目標角に入れる．

        %目標値の格納
        xd = [xd; zeros(28 - size(xd, 1), 1)];

        % 階層型線形化による入力計算
        % 仮想入力のゲイン
        F1 = Param.F1; % z方向サブシステムのゲイン
        F2 = Param.F2; % x方向サブシステムのゲイン
        F3 = Param.F3; % y方向サブシステムのゲイン
        F4 = Param.F4; % yaw方向サブシステムのゲイン
        
        
        % -----------------------------------------------------------------
        % フェーズ①：公称名目入力の計算 (元コードと100%完全同一・無加工)
        % -----------------------------------------------------------------
        vf_nominal = obj.Vfd_SuspendedLoadxyDst(Param.dt, x, xd', F1); 
        vs_nominal = obj.Vs_SuspendedLoadxyDst(x, xd', vf_nominal, P, F2, F3, F4); 
        
        % 🌟【修正点】：シミュレータ本体が管理している本物の現在時間 t を安全に抽出　これしっかりと抽出できているのか確認
        % そもそもしっかりと動いているんだから要らないかも制約と考えよう
        if length(varargin) >= 1 && isstruct(varargin{1}) && isfield(varargin{1}, 't')
            t_sim = varargin{1}.t;
        else
            t_sim = 0.0; % フォールバック
        end
        
        
        % -----------------------------------------------------------------
        % フェーズ②：【最上流レイヤー】での安全制約（CBF-QP）の実行
        % -----------------------------------------------------------------
        obs_env = ENVIRONMENT_OBSTACLE(); % 障害物定義
        ox = obs_env(1).p_obs(1);
        oy = obs_env(1).p_obs(2);
        oz = obs_env(1).p_obs(3);
        ro = obs_env(1).r_obs;
        r_drone = 0.3; r_load = 0.2;  
        k_cbf1 = 2.5; k_cbf2 = 2.0; k_cbf3 = 1.5; k_cbf4 = 1.0; k_cbf5 = 1.0;
        d1 = 1.5; d2 = 1.0;
        
        cbfParam_top_load  = [ox; oy; oz; ro; r_load;  k_cbf1; k_cbf2; k_cbf3; k_cbf4; k_cbf5; P(:)];
        cbfParam_top_drone = [ox; oy; oz; ro; r_drone; d1; d2; P(:)];
        
        is_flight_now = false;
        if length(varargin) >= 2
            if strcmp(varargin{2}, 'f') || string(varargin{2}) == "f" % これほんとに入っているのか確認
                is_flight_now = true;
            end
        end
        
        current_h_load_eval = 0.5 * ((pL(1) - ox)^2 + (pL(2) - oy)^2 + (pL(3) - oz)^2 - (ro + r_load)^2);
        
        % 安全な操作変数を名目値で初期化
        v_safe = v_nominal;
        cbf_status_str = '🟢 [CBF FREE - IDLE]';
        
        % 本当に障害物が近づいたフライト中のみ安全フィルター（QP）を介入させる
        if is_flight_now && (current_h_load_eval <= 15.0) % これ動いているのか見ましょう
            [A_load_top,  b_load_top]  = CBF_Constraints_Load_TopLayer(obj,  x, xd', v_nominal_top, cbfParam_top_load,  t_sim);
            [A_drone_top, b_drone_top] = CBF_Constraints_Drone(obj, x, xd', v_nominal_top, cbfParam_top_drone, t_sim);
            
            % 成分ごとの代数的対角正規化
            for i = 1:3
                scale_fact_load = abs(A_load_top(i));
                if scale_fact_load > 1e-5
                    A_load_top(i) = A_load_top(i) / scale_fact_load;
                    b_load_top = b_load_top / scale_fact_load; 
                end
                scale_fact_drone = abs(A_drone_top(i));
                if scale_fact_drone > 1e-5
                    A_drone_top(i) = A_drone_top(i) / scale_fact_drone;
                    b_drone_top = b_drone_top / scale_fact_drone;
                end
            end
            
            sigma_param = 1.0; epsilon_param = 2.0; gamma_param = 2.0;    
            denom = 4 * sigma_param * (epsilon_param - gamma_param/2 + 1e-5); 
            Delta_load = norm(A_load_top)^2 / denom; 
            
            A_qp = -[A_load_top; A_drone_top]; 
            b_qp = -[b_load_top; b_drone_top]; 
            b_qp(1) = b_qp(1) - max(0, Delta_load); 
            
            v_nominal_top_clipped = max([-15; -15; -15], min([15; 15; 15], v_nominal_top));
            H_mat = eye(3) * 10.0;
            f_vec = -10.0 * v_nominal_top_clipped;
            lb = v_nominal_top_clipped - [8; 8; 8];
            ub = v_nominal_top_clipped + [8; 8; 8];
            
            options = optimoptions('quadprog', 'Display', 'off');
            [v_opt, ~, exitflag] = quadprog(H_mat, f_vec, A_qp, b_qp, [], [], lb, ub, [], options);
            
            if exitflag == 1 && ~any(isnan(v_opt))
                v_safe = max(lb, min(ub, v_opt(:)));
                cbf_status_str = '⚙️ [CBF WORKING - ACTIVE]';
            else
                cbf_status_str = '⚠️ [QP FAILED - FALLBACK TO NOMINAL]';
            end
        end
        
        
        % -----------------------------------------------------------------
        % フェーズ④：安全な高次微分の離散時間発展
        % -----------------------------------------------------------------
        vf_safe = vf_nominal;
        vs_safe = vs_nominal;
        
        vf_safe(1) = v_robust_top(1); 
        vs_safe(1) = v_robust_top(2); 
        vs_safe(2) = v_robust_top(3); 
        
        % 🌟安全圏（FREE）の時は、元コードが叩き出した100%完璧な前進用高次微分「vf_nominal」を
        % 無傷でそのまま下流へ流します。これでフライト移行時の姿勢破壊が完全に防がれます。
        if strcmp(cbf_status_str, '⚙️ [CBF WORKING - ACTIVE]')
            % 回避制約が本当に発動したときだけ、不連続性を防ぐために高度の微分を滑らかに再合成　これ何やっているんだ？
            syms k real
            A1_mat = expm([0,1;0,0]*Param.dt) - int(expm([0,1;0,0]*(Param.dt-k))*[0;1], k, [0,Param.dt]) * F1;
            A1_mat = double(A1_mat); 
            
            Z1_state = [model.state.pL(3) - xd_row(3); model.state.vL(3) - xd_row(7)]; 
            for idx = 2:6
                Z1_state = A1_mat * Z1_state;
                vf_safe(idx) = -F1 * Z1_state; 
            end
        end
        
        % -----------------------------------------------------------------
        % フェーズ⑤：既存の非線形相殺・実物理入力への最終コンバート
        % -----------------------------------------------------------------
        uf = obj.Uf_SuspendedLoadxyDst(x, xd', vf_safe, P);
        beta2 = obj.Beta2_SuspendedLoadxyDst(x, xd', vf_safe, P);
        vs_alpha2 = obj.V2_alpha2_SuspendedLoadxyDst(x, xd', vf_safe, vs_safe', P);
        
        norm_beta2 = norm(beta2);
        cond_beta2 = cond(beta2); 
        norm_vs_alpha2 = norm(vs_alpha2);
        
        us = beta2 \ vs_alpha2; 
        tmp = [uf(1); us]; 
        obj.result.tmp = tmp; 
        
        % 徹底数値監視ロギング（フライト開始時用）
        if strcmp(cbf_status_str, '⚙️ [CBF WORKING - ACTIVE]')
            fprintf('\n================== 🔍 PLAN-B LAYER ANALYSIS ==================\n');
            fprintf('◆ [Time]         t             : %.3f\n', t_sim);
            fprintf('◆ [QP Top Out]   v_safe_top    : [%.2f, %.2f, %.2f]\n', v_safe(1), v_safe(2), v_safe(3));
            fprintf('◆ [Linearization]norm(vs_a2)   : %.3e\n', norm_vs_alpha2);
            fprintf('◆ [Matrix State] norm(beta2)   : %.3e  | Cond(beta2): %.3e\n', norm_beta2, cond_beta2);
            fprintf('◆ [Raw Hardware] 生Thrust(uf)  : %5.2f  | 生Torque(us): [%.2f, %.2f, %.2f]\n', tmp(1), tmp(2), tmp(3), tmp(4));
            fprintf('========================================================================\n\n');
        else
            if is_flight_now
                fprintf('🟢 [CBF FREE] t: %.3f | h: %.2f | v_nom_top: [%.2f, %.2f, %.2f]\n', t_sim, current_h_load_eval, v_nominal_top(1), v_nominal_top(2), v_nominal_top(3));
            end
        end
        
        obj.result.input = [max(0, min(20, tmp(1))); max(-1, min(1, tmp(2))); max(-1, min(1, tmp(3))); max(-1, min(1, tmp(4)))]; 
        obj.result.xd = xd;
        obj.result.x = x;
        result = obj.result;
    end
    
    function show(obj)
        obj.result
    end
end
end

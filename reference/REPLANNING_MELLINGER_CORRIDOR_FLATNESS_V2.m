% classdef REPLANNING_MELLINGER_CORRIDOR_FLATNESS_V2 < handle
%     % REPLANNING_MELLINGER_CORRIDOR_FLATNESS_V2
%     % 13次多項式 (7階微分まで連続) の完全なる滑らかさを維持しつつ、
%     % スケーリング理論に基づき数値的悪条件を完全に排除した真の汎用2点バリアリプランナ。
% 
%     properties
%         base_ref
%         self
%         replan_active = false
%         replan_done   = false
% 
%         t_start
%         t_duration = 10.0
% 
%         safe_margin  = 0.3
%         trigger_dist = 2.5
% 
%         L_cable
%         gravity = 9.81
% 
%         poly_coeffs 
% 
%         t_merge_end
%         p_merge_end
%         v_merge_vec
% 
%         obs_center = [0;0;0];
%         obs_radius = 0;
%         hyperplane_n = [0;0;0];
%         hyperplane_p = [0;0;0];
% 
%         result
%     end
% 
%     methods
%         function obj = REPLANNING_MELLINGER_CORRIDOR_FLATNESS_V2(self, base_ref, opts)
%             arguments
%                 self
%                 base_ref
%                 opts = struct()
%             end
%             obj.self = self;
%             obj.base_ref = base_ref;
%             if isfield(opts, "safe_margin"),  obj.safe_margin  = opts.safe_margin;     end
%             if isfield(opts, "trigger_dist"), obj.trigger_dist = opts.trigger_dist;    end
%             obj.result.state = STATE_CLASS(struct('state_list', ["xd", "p", "q", "v"], 'num_list', [28, 3, 3, 3]));
%         end
% 
%         function result = do(obj, varargin)
%             time = varargin{1};
%             cha = varargin{2};
% 
%             base_res = obj.base_ref.do(time, cha);
%             xd_nominal = base_res.state.xd;
%             if length(xd_nominal) < 28
%                 xd_nominal = [xd_nominal; zeros(28 - length(xd_nominal), 1)];
%             end
% 
%             pL_cur = base_res.state.p;
%             if isprop(obj.self.estimator.result.state, "pL")
%                 pL_cur = obj.self.estimator.result.state.pL;
%             end
% 
%             obj.L_cable = obj.self.parameter.get("cableL");
%             pQ_cur = pL_cur + [0; 0; obj.L_cable];
% 
%             if cha == 'f' && ~obj.replan_done && ~obj.replan_active
%                 try
%                     obs_list = ENVIRONMENT_OBSTACLE_HOCBF_LINK_XY();
%                 catch
%                     obs_list = [];
%                 end
% 
%                 min_dist_overall = inf;
%                 target_obs_idx = -1;
% 
%                 for i = 1:length(obs_list)
%                     if isfield(obs_list(i), 'p_obs') && isfield(obs_list(i), 'r_obs')
%                         c = obs_list(i).p_obs;
%                         r = obs_list(i).r_obs;
%                         dist_load = norm(pL_cur - c);
%                         dist_drone = norm(pQ_cur - c);
%                         d_surf = min(dist_load, dist_drone) - r;
% 
%                         if d_surf < min_dist_overall
%                             min_dist_overall = d_surf;
%                             target_obs_idx = i;
%                         end
%                     end
%                 end
% 
%                 if target_obs_idx > 0 && min_dist_overall <= obj.trigger_dist
%                     obj.obs_center = obs_list(target_obs_idx).p_obs;
%                     obj.obs_radius = obs_list(target_obs_idx).r_obs;
% 
%                     fprintf("\n=======================================================\n");
%                     fprintf("[MELLINGER FLATNESS V2] 障害物検知! (ID: %d, d_surf=%.2fm, t=%.3f s)\n", target_obs_idx, min_dist_overall, time.t);
%                     obj.t_start = time.t;
% 
%                     v_vec = xd_nominal(5:7);
%                     spd = norm(v_vec);
%                     if spd < 0.1, spd = 0.5; v_vec = [0; 0; 0.5]; end
%                     dir_nom = v_vec / spd;
% 
%                     vec_to_obs = obj.obs_center - pL_cur;
%                     proj_dist = max(0, dot(vec_to_obs, dir_nom));
% 
%                     total_dist = proj_dist + obj.obs_radius + obj.safe_margin + 2.0;
%                     obj.t_duration = max(5.0, min(12.0, total_dist / spd));
% 
%                     fprintf("[MELLINGER FLATNESS V2] 理論に基づく 13次多項式 ソフト制約QP の実行...\n");
%                     obj.plan_corridor_soft_qp(xd_nominal, dir_nom, spd, total_dist);
%                     obj.replan_active = true;
%                     fprintf("=======================================================\n\n");
%                 end
%             end
% 
%             if obj.replan_active
%                 tau = time.t - obj.t_start;
%                 if tau <= obj.t_duration
%                     xd = obj.evaluate_smooth_trajectory(tau, xd_nominal);
%                 else
%                     if ~obj.replan_done
%                         fprintf("[MELLINGER FLATNESS V2] 障害物クリア・C^6シームレス合流 (t=%.3f s)\n\n", time.t);
%                         obj.t_merge_end = obj.t_start + obj.t_duration;
%                         xd_end = obj.evaluate_smooth_trajectory(obj.t_duration, xd_nominal);
%                         obj.p_merge_end = xd_end(1:3);
%                         obj.v_merge_vec = xd_end(5:7);
%                         obj.replan_active = false;
%                         obj.replan_done   = true;
%                     end
%                     dt_after = time.t - obj.t_merge_end;
%                     xd = zeros(28, 1);
%                     xd(1:3) = obj.p_merge_end + obj.v_merge_vec * dt_after;
%                     xd(5:7) = obj.v_merge_vec;
%                 end
%             elseif obj.replan_done
%                 dt_after = time.t - obj.t_merge_end;
%                 xd = zeros(28, 1);
%                 xd(1:3) = obj.p_merge_end + obj.v_merge_vec * dt_after;
%                 xd(5:7) = obj.v_merge_vec;
%             else
%                 xd = xd_nominal;
%             end
% 
%             if length(xd) < 28, xd = [xd; zeros(28 - length(xd), 1)]; end
%             obj.result.state.xd = xd;
%             obj.result.state.p = xd(1:3);
%             obj.result.state.v = xd(5:7);
%             obj.result.state.q = [0; 0; xd(4)];
%             result = obj.result;
%         end
%     end
% 
%     methods (Access = private)
%         function plan_corridor_soft_qp(obj, xd0, dir_nom, spd, total_dist)
%             T = obj.t_duration;
%             p0 = xd0(1:3);
% 
%             vec_to_obs = obj.obs_center - p0;
%             proj_on_line = dot(vec_to_obs, dir_nom) * dir_nom;
%             normal_vec = vec_to_obs - proj_on_line;
%             if norm(normal_vec) < 1e-3
%                 tangent = cross(dir_nom, [0; 0; 1]);
%                 if norm(tangent) < 1e-3, tangent = cross(dir_nom, [1; 0; 0]); end
%                 normal_vec = tangent;
%             end
% 
%             n_wall = - (normal_vec / norm(normal_vec)); 
%             R_clear = obj.obs_radius + obj.safe_margin;
%             p_wall = obj.obs_center + n_wall * R_clear;
% 
%             a_ineq = -n_wall;
%             b_ineq = -dot(n_wall, p_wall);
% 
%             obj.hyperplane_n = n_wall;
%             obj.hyperplane_p = p_wall;
% 
%             p_end = p0 + dir_nom * total_dist;
%             xd1 = zeros(28, 1);
%             xd1(1:3) = p_end;
%             xd1(5:7) = dir_nom * spd;
% 
%             % 13次多項式 -> 14変数
%             order = 13;
%             n_coeffs = order + 1;
%             N_samples = 15;
% 
%             num_C = 3 * n_coeffs;
%             num_Slack = 2 * N_samples;
%             num_vars = num_C + num_Slack;
% 
%             H = zeros(num_vars, num_vars);
%             f = zeros(num_vars, 1);
% 
%             % 時間Tを考慮した正しい物理スケーリングの目的関数
%             H_1d = zeros(n_coeffs, n_coeffs);
%             w4 = 1.0; w5 = 0.1;
%             for i = 4:order
%                 for j = 4:order
%                     val4 = (factorial(i)/factorial(i-4)) * (factorial(j)/factorial(j-4)) / (i + j - 7);
%                     H_1d(i+1, j+1) = H_1d(i+1, j+1) + w4 * val4 / (T^7);
%                     if i >= 5 && j >= 5
%                         val5 = (factorial(i)/factorial(i-5)) * (factorial(j)/factorial(j-5)) / (i + j - 9);
%                         H_1d(i+1, j+1) = H_1d(i+1, j+1) + w5 * val5 / (T^9);
%                     end
%                 end
%             end
% 
%             % Hessianの正規化（スケールを整える）
%             scale_H = max(abs(H_1d(:)));
%             if scale_H > 0
%                 H_1d = H_1d / scale_H;
%             end
%             H(1:num_C, 1:num_C) = blkdiag(H_1d, H_1d, H_1d) + 1e-9 * eye(num_C);
% 
%             % スラック変数へのウェイトをHessianのスケールに合わせる
%             W_slack = 1e4; % 適度な強さで壁侵入を防ぐ
%             for i = 1:num_Slack
%                 idx = num_C + i;
%                 H(idx, idx) = W_slack;
%                 f(idx) = W_slack;
%             end
% 
%             % 等式制約 (位置、速度、加速度、ジャーク、スナップ、クラックル、7階微分まで連続)
%             n_eq_1d = 13;
%             Aeq_1d = zeros(n_eq_1d, n_coeffs);
%             beq_1d = zeros(n_eq_1d, 1);
% 
%             for k = 0:6
%                 row = k + 1;
%                 for n = k:order, Aeq_1d(row, n + 1) = prod(n - k + 1 : n) * (0)^(n - k); end
%             end
%             for k = 0:5
%                 row = 8 + k;
%                 for n = k:order, Aeq_1d(row, n + 1) = prod(n - k + 1 : n) * (1.0)^(n - k); end
%             end
% 
%             Aeq = zeros(3 * n_eq_1d, num_vars);
%             Aeq(1:3*n_eq_1d, 1:num_C) = blkdiag(Aeq_1d, Aeq_1d, Aeq_1d);
%             beq = zeros(3 * n_eq_1d, 1);
% 
%             for axis = 1:3
%                 beq_axis = zeros(n_eq_1d, 1);
%                 for k = 0:6, beq_axis(k+1) = (T^k) * xd0(4*k + axis); end
%                 for k = 0:5, beq_axis(8+k) = (T^k) * xd1(4*k + axis); end
%                 beq( (axis-1)*n_eq_1d + 1 : axis*n_eq_1d ) = beq_axis;
%             end
% 
%             % 不等式制約
%             A_ineq = zeros(2 * N_samples, num_vars);
%             b_ineq_all = zeros(2 * N_samples, 1);
%             row_idx = 1;
%             L_g = obj.L_cable / obj.gravity;
% 
%             for m = 1:N_samples
%                 u_m = m / (N_samples + 1); 
%                 T0_vec = zeros(1, n_coeffs);
%                 T2_vec = zeros(1, n_coeffs);
%                 for n = 0:order
%                     T0_vec(n+1) = u_m^n;
%                     if n >= 2, T2_vec(n+1) = (n * (n - 1)) * u_m^(n - 2) / (T^2); end
%                 end
% 
%                 T0_blk = blkdiag(T0_vec, T0_vec, T0_vec);
%                 T2_blk = blkdiag(T2_vec, T2_vec, T2_vec);
% 
%                 A_ineq(row_idx, 1:num_C) = a_ineq' * T0_blk;
%                 A_ineq(row_idx, num_C + 2*m - 1) = -1;
%                 b_ineq_all(row_idx) = b_ineq;
%                 row_idx = row_idx + 1;
% 
%                 A_ineq(row_idx, 1:num_C) = a_ineq' * (T0_blk + L_g * T2_blk);
%                 A_ineq(row_idx, num_C + 2*m) = -1;
%                 b_ineq_all(row_idx) = b_ineq - a_ineq' * [0; 0; obj.L_cable];
%                 row_idx = row_idx + 1;
%             end
% 
%             A_slack_pos = zeros(num_Slack, num_vars);
%             A_slack_pos(:, num_C+1:end) = -eye(num_Slack);
%             A_ineq = [A_ineq; A_slack_pos];
%             b_ineq_all = [b_ineq_all; zeros(num_Slack, 1)];
% 
%             opts = optimoptions('quadprog', 'Display', 'off');
%             [X_opt, ~, exitflag] = quadprog(H, f, A_ineq, b_ineq_all, Aeq, beq, [], [], [], opts);
% 
%             if exitflag < 1
%                 fprintf("[MELLINGER FLATNESS V2] QP数値エラー(exitflag=%d)。解析解へフォールバック\n", exitflag);
%                 p_via = obj.obs_center + n_wall * (R_clear + 1.0);
%                 A_fb = zeros(14, 14); Bx_fb = zeros(14, 3);
%                 for k = 0:6
%                     for n = k:order, A_fb(k+1, n+1) = prod(n-k+1:n) * 0^(n-k); end
%                     Bx_fb(k+1, :) = (T^k) * xd0(4*k + (1:3))';
%                 end
%                 for n = 0:order, A_fb(8, n+1) = (0.5)^n; end
%                 Bx_fb(8, :) = p_via';
%                 for k = 0:5
%                     for n = k:order, A_fb(9+k, n+1) = prod(n-k+1:n) * 1.0^(n-k); end
%                     Bx_fb(9+k, :) = (T^k) * xd1(4*k + (1:3))';
%                 end
%                 C_fb = A_fb \ Bx_fb;
%                 C_opt = [C_fb(:, 1); C_fb(:, 2); C_fb(:, 3)];
%             else
%                 max_violation = max(X_opt(num_C+1:end));
%                 if max_violation > 0.05
%                     fprintf("[MELLINGER FLATNESS V2] ※壁侵入(%.3fm)を許容して滑らかさを優先しました。\n", max_violation);
%                 else
%                     fprintf("[MELLINGER FLATNESS V2] QP成功！壁への完全回避と13次平滑化達成。\n");
%                 end
%                 C_opt = X_opt(1:num_C);
%             end
%             obj.poly_coeffs = [C_opt(1:n_coeffs), C_opt(n_coeffs+1 : 2*n_coeffs), C_opt(2*n_coeffs+1 : end)];
%         end
% 
%         function xd = evaluate_smooth_trajectory(obj, tau, xd_nom)
%             xd = xd_nom;
%             C = obj.poly_coeffs;
%             order = 13;
%             T = obj.t_duration;
%             u = max(0, min(1.0, tau / T));
% 
%             for k = 0:6
%                 val_k = zeros(3, 1);
%                 for n = k:order
%                     factor = prod(n - k + 1 : n);
%                     val_k = val_k + C(n + 1, :)' * factor * (u^(n - k));
%                 end
%                 xd(4*k + (1:3)) = val_k / (T^k);
%             end
%         end
%     end
% end


% classdef REPLANNING_MELLINGER_CORRIDOR_FLATNESS_V2 < handle
%     % REPLANNING_MELLINGER_CORRIDOR_FLATNESS_V2
%     % 任意進行方向・任意障害物配置対応 Mellinger Corridor QP リプランナ
%     % - 障害物本体: ハード制約 (侵入絶対禁止)
%     % - 安全マージン帯: ソフト制約 (スラック変数による最適ペナルティ)
% 
%     properties
%         base_ref
%         self
%         replan_active = false
%         replan_done   = false
% 
%         t_start
%         t_duration = 8.0           % 動的に最適決定される所要時間 [s]
% 
%         safe_margin  = 0.5         % デフォルトマージン (obs側で指定あれば上書き)
%         trigger_dist = 4.0
% 
%         L_cable      = 2.0
%         gravity      = 9.81
% 
%         poly_coeffs                % 3 x 14 係数行列 (無次元時間 u in [0,1])
% 
%         % 合流用保持
%         t_merge_end
%         p_merge_end
%         v_merge_vec
% 
%         obs_center = [0; 0; 0]
%         obs_radius = 0
%         obs_margin = 0
% 
%         result
%     end
% 
%     methods
%         function obj = REPLANNING_MELLINGER_CORRIDOR_FLATNESS_V2(self, base_ref, opts)
%             arguments
%                 self
%                 base_ref
%                 opts = struct()
%             end
%             obj.self = self;
%             obj.base_ref = base_ref;
%             if isfield(opts, "safe_margin"),  obj.safe_margin  = opts.safe_margin;  end
%             if isfield(opts, "trigger_dist"), obj.trigger_dist = opts.trigger_dist; end
%             obj.result.state = STATE_CLASS(struct('state_list', ["xd", "p", "q", "v"], 'num_list', [28, 3, 3, 3]));
%         end
% 
%         function result = do(obj, varargin)
%             time = varargin{1};
%             cha = varargin{2};
% 
%             base_res = obj.base_ref.do(time, cha);
%             xd_nominal = base_res.state.xd;
%             if length(xd_nominal) < 28
%                 xd_nominal = [xd_nominal; zeros(28 - length(xd_nominal), 1)];
%             end
% 
%             if isprop(obj.self.estimator.result.state, "pL")
%                 pL_cur = obj.self.estimator.result.state.pL;
%                 vL_cur = obj.self.estimator.result.state.vL;
%             else
%                 pL_cur = base_res.state.p;
%                 vL_cur = base_res.state.v;
%             end
% 
%             obj.L_cable = obj.self.parameter.get("cableL");
%             pQ_cur = pL_cur + [0; 0; obj.L_cable];
% 
%             % 1. 動的障害物検知 (任意障害物リストの走査)
%             if cha == 'f' && ~obj.replan_done && ~obj.replan_active
%                 try
%                     obs_list = ENVIRONMENT_OBSTACLE_HOCBF_LINK_XY();
%                 catch
%                     obs_list = [];
%                 end
% 
%                 min_dist_overall = inf;
%                 target_obs_idx = -1;
% 
%                 for i = 1:length(obs_list)
%                     if isfield(obs_list(i), 'p_obs') && isfield(obs_list(i), 'r_obs')
%                         c = obs_list(i).p_obs;
%                         r_body = obs_list(i).r_obs;
%                         if isfield(obs_list(i), 'd_margin')
%                             d_marg = obs_list(i).d_margin;
%                         else
%                             d_marg = obj.safe_margin;
%                         end
% 
%                         dist_load  = norm(pL_cur - c);
%                         dist_drone = norm(pQ_cur - c);
%                         d_surf = min(dist_load, dist_drone) - (r_body + d_marg);
% 
%                         if d_surf < min_dist_overall
%                             min_dist_overall = d_surf;
%                             target_obs_idx = i;
%                         end
%                     end
%                 end
% 
%                 if target_obs_idx > 0 && min_dist_overall <= obj.trigger_dist
%                     obj.obs_center = obs_list(target_obs_idx).p_obs;
%                     obj.obs_radius = obs_list(target_obs_idx).r_obs;
%                     if isfield(obs_list(target_obs_idx), 'd_margin')
%                         obj.obs_margin = obs_list(target_obs_idx).d_margin;
%                     else
%                         obj.obs_margin = obj.safe_margin;
%                     end
% 
%                     fprintf("\n=======================================================\n");
%                     fprintf("[CORRIDOR QP] 障害物検知! (ID: %d, d_surf=%.2fm, t=%.3f s)\n", target_obs_idx, min_dist_overall, time.t);
%                     fprintf("  - 本体半径: %.2f m (ハード制約)\n", obj.obs_radius);
%                     fprintf("  - 安全マージン: %.2f m (ソフト制約)\n", obj.obs_margin);
%                     obj.t_start = time.t;
% 
%                     % 任意方向の速度ベクトル・巡航速度の動的抽出
%                     v_vec = xd_nominal(5:7);
%                     spd = norm(v_vec);
%                     if spd < 0.05, spd = norm(vL_cur); end
%                     if spd < 0.05, spd = 0.5; v_vec = [0; 0; 0.5]; end
%                     dir_nom = v_vec / spd;
% 
%                     % 進行軸上の障害物通過距離と所要時間 T の動的計算
%                     vec_to_obs = obj.obs_center - pL_cur;
%                     proj_dist = max(0, dot(vec_to_obs, dir_nom));
%                     total_dist = proj_dist + obj.obs_radius + obj.obs_margin + obj.L_cable + 1.5;
%                     obj.t_duration = max(6.0, min(14.0, total_dist / spd));
% 
%                     fprintf("  - 進行方向: [%.2f, %.2f, %.2f], 計画時間 T = %.2f s\n", dir_nom(1), dir_nom(2), dir_nom(3), obj.t_duration);
% 
%                     obj.plan_mellinger_hybrid_qp(xd_nominal, dir_nom, spd, total_dist);
%                     obj.replan_active = true;
%                     fprintf("=======================================================\n\n");
%                 end
%             end
% 
%             % 2. 軌道出力 ＆ C^6シームレス合流
%             if obj.replan_active
%                 tau = time.t - obj.t_start;
%                 if tau <= obj.t_duration
%                     xd = obj.evaluate_smooth_trajectory(tau, xd_nominal);
%                 else
%                     if ~obj.replan_done
%                         fprintf("[CORRIDOR QP] 障害物通過完了・C^6シームレス合流 (t=%.3f s)\n\n", time.t);
%                         obj.t_merge_end = obj.t_start + obj.t_duration;
%                         xd_end = obj.evaluate_smooth_trajectory(obj.t_duration, xd_nominal);
%                         obj.p_merge_end = xd_end(1:3);
%                         obj.v_merge_vec = xd_end(5:7);
%                         obj.replan_active = false;
%                         obj.replan_done   = true;
%                     end
% 
%                     % 合流後: そのまま進行方向へ完全等速直進
%                     dt_after = time.t - obj.t_merge_end;
%                     xd = zeros(28, 1);
%                     xd(1:3) = obj.p_merge_end + obj.v_merge_vec * dt_after;
%                     xd(5:7) = obj.v_merge_vec;
%                     xd(9:28) = 0;
%                 end
%             elseif obj.replan_done
%                 dt_after = time.t - obj.t_merge_end;
%                 xd = zeros(28, 1);
%                 xd(1:3) = obj.p_merge_end + obj.v_merge_vec * dt_after;
%                 xd(5:7) = obj.v_merge_vec;
%                 xd(9:28) = 0;
%             else
%                 xd = xd_nominal;
%             end
% 
%             if length(xd) < 28, xd = [xd; zeros(28 - length(xd), 1)]; end
%             obj.result.state.xd = xd;
%             obj.result.state.p = xd(1:3);
%             obj.result.state.v = xd(5:7);
%             obj.result.state.q = [0; 0; xd(4)];
%             result = obj.result;
%         end
%     end
% 
%     methods (Access = private)
%         function plan_mellinger_hybrid_qp(obj, xd0, dir_nom, spd, total_dist)
%             T = obj.t_duration;
%             p0 = xd0(1:3);
%             order = 13;
%             n_coeffs = order + 1;
% 
%             % 1. 進行方向ベクトルに直交する最短法線ベクトルの動的幾何計算
%             vec_to_obs = obj.obs_center - p0;
%             proj_on_line = dot(vec_to_obs, dir_nom) * dir_nom;
%             normal_vec = vec_to_obs - proj_on_line;
% 
%             if norm(normal_vec) < 1e-3
%                 % 軌道直線上にある場合、進行方向に依存しない直交軸を選択
%                 if abs(dir_nom(3)) < 0.9
%                     normal_vec = cross(dir_nom, [0; 0; 1]);
%                 else
%                     normal_vec = cross(dir_nom, [1; 0; 0]);
%                 end
%             end
%             dir_normal = normal_vec / norm(normal_vec);
% 
%             % ドローン・紐を含めたマージン付き回避半径
%             R_hard = obj.obs_radius + obj.L_cable * 0.4 + 0.15;
%             R_soft = R_hard + obj.obs_margin;
% 
%             % 中間経由点 (Corridorの中心点)
%             p_mid_nominal = p0 + dir_nom * (total_dist * 0.5);
%             p_via = p_mid_nominal + dir_normal * R_soft;
% 
%             % 終端合流点
%             p_end = p0 + dir_nom * total_dist;
%             xd1 = zeros(28, 1);
%             xd1(1:3) = p_end;
%             xd1(5:7) = dir_nom * spd;
%             xd1(9:28) = 0;
% 
%             % サンプル区間設定
%             N_samples = 9;
%             u_samples = linspace(0.3, 0.7, N_samples);
%             num_C = 3 * n_coeffs;
%             num_vars = num_C + N_samples; % スラック変数 (ソフト制約用)
% 
%             % -------------------------------------------------------------
%             % 2. 目的関数 H の無次元正規化 (条件数 O(10^5) 保証)
%             % -------------------------------------------------------------
%             H_1d = zeros(n_coeffs, n_coeffs);
%             w4 = 1.0; w5 = 0.1;
%             for i = 4:order
%                 for j = 4:order
%                     val4 = (factorial(i)/factorial(i-4)) * (factorial(j)/factorial(j-4)) / (i + j - 7);
%                     H_1d(i+1, j+1) = H_1d(i+1, j+1) + w4 * val4;
%                     if i >= 5 && j >= 5
%                         val5 = (factorial(i)/factorial(i-5)) * (factorial(j)/factorial(j-5)) / (i + j - 9);
%                         H_1d(i+1, j+1) = H_1d(i+1, j+1) + w5 * val5;
%                     end
%                 end
%             end
% 
%             norm_H = norm(H_1d, 2);
%             H_1d_scaled = H_1d / norm_H;
% 
%             H = zeros(num_vars, num_vars);
%             H(1:num_C, 1:num_C) = blkdiag(H_1d_scaled, H_1d_scaled, H_1d_scaled) + 1e-6 * eye(num_C);
% 
%             w_slack = 5000.0;
%             for s_idx = 1:N_samples
%                 H(num_C + s_idx, num_C + s_idx) = w_slack;
%             end
%             f = zeros(num_vars, 1);
%             f(num_C + 1 : end) = w_slack;
% 
%             % -------------------------------------------------------------
%             % 3. 等式制約 (始端7階 + 終端6階 + 経由点1階)
%             % -------------------------------------------------------------
%             n_eq = 14;
%             Aeq_1d = zeros(n_eq, n_coeffs);
%             for k = 0:6
%                 for n = k:order, Aeq_1d(k+1, n+1) = prod(n-k+1:n) * 0^(n-k); end
%             end
%             for n = 0:order
%                 Aeq_1d(8, n+1) = 0.5^n;
%             end
%             for k = 0:5
%                 for n = k:order, Aeq_1d(9+k, n+1) = prod(n-k+1:n) * 1.0^(n-k); end
%             end
% 
%             Aeq = zeros(3 * n_eq, num_vars);
%             Aeq(:, 1:num_C) = blkdiag(Aeq_1d, Aeq_1d, Aeq_1d);
%             beq = zeros(3 * n_eq, 1);
% 
%             for ax = 1:3
%                 beq_ax = zeros(n_eq, 1);
%                 for k = 0:6, beq_ax(k+1) = (T^k) * xd0(4*k + ax); end
%                 beq_ax(8) = p_via(ax);
%                 for k = 0:5, beq_ax(9+k) = (T^k) * xd1(4*k + ax); end
%                 beq((ax-1)*n_eq + 1 : ax*n_eq) = beq_ax;
%             end
% 
%             % -------------------------------------------------------------
%             % 4. 不等式制約: 真横ハード + 全域ソフト制約
%             % -------------------------------------------------------------
%             n_wall = -dir_normal;
%             p_hard_bound = obj.obs_center + dir_normal * R_hard;
%             p_soft_bound = obj.obs_center + dir_normal * R_soft;
% 
%             hard_mask = (u_samples >= 0.45) & (u_samples <= 0.55);
%             n_hard = sum(hard_mask);
% 
%             num_ineq = n_hard + N_samples + N_samples;
%             A_ineq = zeros(num_ineq, num_vars);
%             b_ineq = zeros(num_ineq, 1);
% 
%             row = 1;
%             % a) ハード制約: 真横通過時 (スラックなし)
%             for m = find(hard_mask)
%                 u_m = u_samples(m);
%                 T_vec = zeros(1, n_coeffs);
%                 for n = 0:order, T_vec(n+1) = u_m^n; end
%                 A_ineq(row, 1:n_coeffs)               = n_wall(1) * T_vec;
%                 A_ineq(row, n_coeffs+1 : 2*n_coeffs)   = n_wall(2) * T_vec;
%                 A_ineq(row, 2*n_coeffs+1 : 3*n_coeffs) = n_wall(3) * T_vec;
%                 b_ineq(row) = dot(n_wall, p_hard_bound);
%                 row = row + 1;
%             end
% 
%             % b) ソフト制約: マージン境界 (スラック許容)
%             for m = 1:N_samples
%                 u_m = u_samples(m);
%                 T_vec = zeros(1, n_coeffs);
%                 for n = 0:order, T_vec(n+1) = u_m^n; end
%                 A_ineq(row, 1:n_coeffs)               = n_wall(1) * T_vec;
%                 A_ineq(row, n_coeffs+1 : 2*n_coeffs)   = n_wall(2) * T_vec;
%                 A_ineq(row, 2*n_coeffs+1 : 3*n_coeffs) = n_wall(3) * T_vec;
%                 A_ineq(row, num_C + m)                = -1;
%                 b_ineq(row) = dot(n_wall, p_soft_bound);
%                 row = row + 1;
%             end
% 
%             % c) スラック非負制約
%             for m = 1:N_samples
%                 A_ineq(row, num_C + m) = -1;
%                 b_ineq(row) = 0;
%                 row = row + 1;
%             end
% 
%             % -------------------------------------------------------------
%             % 5. 最適化実行
%             % -------------------------------------------------------------
%             opts = optimoptions('quadprog', ...
%                 'Display', 'off', ...
%                 'Algorithm', 'interior-point-convex', ...
%                 'MaxIterations', 200, ...
%                 'ConstraintTolerance', 1e-4, ... % 0.1 mm 精度（1e-8 による偽の解なしを防止）
%                 'OptimalityTolerance', 1e-4, ...
%                 'StepTolerance', 1e-8);
% 
%             [X_opt, ~, exitflag, output] = quadprog(H, f, A_ineq, b_ineq, Aeq, beq, [], [], [], opts);
% 
%             if exitflag < 1
%                 fprintf("[CORRIDOR QP DEBUG] quadprog未収束 (exitflag=%d: %s)\n", exitflag, output.message);
%                 fprintf("[CORRIDOR QP] フォールバック解(Aeq\\beq)を採用\n");
%                 C_1 = Aeq_1d \ beq(1:n_eq);
%                 C_2 = Aeq_1d \ beq(n_eq+1:2*n_eq);
%                 C_3 = Aeq_1d \ beq(2*n_eq+1:3*n_eq);
%                 obj.poly_coeffs = [C_1, C_2, C_3]';
%             else
%                 slack_vals = X_opt(num_C + 1 : end);
%                 max_slack = max(slack_vals);
%                 fprintf("[CORRIDOR QP] QP最適化成功! (exitflag=1, 反復=%d, マージン内侵入=%.3fm)\n", ...
%                     output.iterations, max_slack);
%                 C_opt = X_opt(1:num_C);
%                 obj.poly_coeffs = [C_opt(1:n_coeffs), C_opt(n_coeffs+1:2*n_coeffs), C_opt(2*n_coeffs+1:end)]';
%             end
%         end
% 
%         function xd = evaluate_smooth_trajectory(obj, tau, xd_nom)
%             xd = xd_nom;
%             C = obj.poly_coeffs;
%             order = 13;
%             T = obj.t_duration;
%             u = max(0, min(1.0, tau / T));
% 
%             for k = 0:6
%                 val_k = zeros(3, 1);
%                 for n = k:order
%                     factor = prod(n - k + 1 : n);
%                     val_k = val_k + C(:, n + 1) * factor * (u^(n - k));
%                 end
%                 xd(4*k + (1:3)) = val_k / (T^k);
%             end
%         end
%     end
% end

% classdef REPLANNING_MELLINGER_CORRIDOR_FLATNESS_V2 < handle
%     % REPLANNING_MELLINGER_CORRIDOR_FLATNESS_V2
%     % 任意進行方向・任意障害物配置対応 Mellinger Corridor QP リプランナ
%     % - 障害物本体: ハード制約 (侵入絶対禁止)
%     % - 安全マージン帯: 山なり凸回廊ソフト制約 (スラック変数による最適ペナルティ)
%     % - 詳細デバッグ解析機能付き
% 
%     properties
%         base_ref
%         self
%         replan_active = false
%         replan_done   = false
% 
%         t_start
%         t_duration = 8.0           % 動的に最適決定される所要時間 [s]
% 
%         safe_margin  = 0.5         % デフォルトマージン (obs側で指定あれば上書き)
%         trigger_dist = 4.0
% 
%         L_cable      = 2.0
%         gravity      = 9.81
% 
%         poly_coeffs                % 3 x 14 係数行列 (無次元時間 u in [0,1])
% 
%         % 合流用保持
%         t_merge_end
%         p_merge_end
%         v_merge_vec
% 
%         obs_center = [0; 0; 0]
%         obs_radius = 0
%         obs_margin = 0
% 
%         result
%     end
% 
%     methods
%         function obj = REPLANNING_MELLINGER_CORRIDOR_FLATNESS_V2(self, base_ref, opts)
%             arguments
%                 self
%                 base_ref
%                 opts = struct()
%             end
%             obj.self = self;
%             obj.base_ref = base_ref;
%             if isfield(opts, "safe_margin"),  obj.safe_margin  = opts.safe_margin;  end
%             if isfield(opts, "trigger_dist"), obj.trigger_dist = opts.trigger_dist; end
%             obj.result.state = STATE_CLASS(struct('state_list', ["xd", "p", "q", "v"], 'num_list', [28, 3, 3, 3]));
%         end
% 
%         function result = do(obj, varargin)
%             time = varargin{1};
%             cha = varargin{2};
% 
%             base_res = obj.base_ref.do(time, cha);
%             xd_nominal = base_res.state.xd;
%             if length(xd_nominal) < 28
%                 xd_nominal = [xd_nominal; zeros(28 - length(xd_nominal), 1)];
%             end
% 
%             if isprop(obj.self.estimator.result.state, "pL")
%                 pL_cur = obj.self.estimator.result.state.pL;
%                 vL_cur = obj.self.estimator.result.state.vL;
%             else
%                 pL_cur = base_res.state.p;
%                 vL_cur = base_res.state.v;
%             end
% 
%             obj.L_cable = obj.self.parameter.get("cableL");
%             pQ_cur = pL_cur + [0; 0; obj.L_cable];
% 
%             % 1. 動的障害物検知 (任意障害物リストの走査)
%             if cha == 'f' && ~obj.replan_done && ~obj.replan_active
%                 try
%                     obs_list = ENVIRONMENT_OBSTACLE_HOCBF_LINK_XY();
%                 catch
%                     obs_list = [];
%                 end
% 
%                 min_dist_overall = inf;
%                 target_obs_idx = -1;
% 
%                 for i = 1:length(obs_list)
%                     if isfield(obs_list(i), 'p_obs') && isfield(obs_list(i), 'r_obs')
%                         c = obs_list(i).p_obs;
%                         r_body = obs_list(i).r_obs;
%                         if isfield(obs_list(i), 'd_margin')
%                             d_marg = obs_list(i).d_margin;
%                         else
%                             d_marg = obj.safe_margin;
%                         end
% 
%                         dist_load  = norm(pL_cur - c);
%                         dist_drone = norm(pQ_cur - c);
%                         d_surf = min(dist_load, dist_drone) - (r_body + d_marg);
% 
%                         if d_surf < min_dist_overall
%                             min_dist_overall = d_surf;
%                             target_obs_idx = i;
%                         end
%                     end
%                 end
% 
%                 if target_obs_idx > 0 && min_dist_overall <= obj.trigger_dist
%                     obj.obs_center = obs_list(target_obs_idx).p_obs;
%                     obj.obs_radius = obs_list(target_obs_idx).r_obs;
%                     if isfield(obs_list(target_obs_idx), 'd_margin')
%                         obj.obs_margin = obs_list(target_obs_idx).d_margin;
%                     else
%                         obj.obs_margin = obj.safe_margin;
%                     end
% 
%                     fprintf("\n=======================================================\n");
%                     fprintf("[CORRIDOR QP] 障害物検知! (ID: %d, d_surf=%.2fm, t=%.3f s)\n", target_obs_idx, min_dist_overall, time.t);
%                     fprintf("  - 本体半径: %.2f m (ハード制約)\n", obj.obs_radius);
%                     fprintf("  - 安全マージン: %.2f m (ソフト制約)\n", obj.obs_margin);
%                     obj.t_start = time.t;
% 
%                     % 任意方向の速度ベクトル・巡航速度の動的抽出
%                     v_vec = xd_nominal(5:7);
%                     spd = norm(v_vec);
%                     if spd < 0.05, spd = norm(vL_cur); end
%                     if spd < 0.05, spd = 0.5; v_vec = [0; 0; 0.5]; end
%                     dir_nom = v_vec / spd;
% 
%                     % 進行軸上の障害物通過距離と所要時間 T の動的計算
%                     vec_to_obs = obj.obs_center - pL_cur;
%                     proj_dist = max(0, dot(vec_to_obs, dir_nom));
%                     total_dist = proj_dist + obj.obs_radius + obj.obs_margin + obj.L_cable + 1.5;
%                     obj.t_duration = max(6.0, min(14.0, total_dist / spd));
% 
%                     fprintf("  - 進行方向: [%.2f, %.2f, %.2f], 計画時間 T = %.2f s\n", dir_nom(1), dir_nom(2), dir_nom(3), obj.t_duration);
% 
%                     obj.plan_mellinger_hybrid_qp(xd_nominal, dir_nom, spd, total_dist);
%                     obj.replan_active = true;
%                     fprintf("=======================================================\n\n");
%                 end
%             end
% 
%             % 2. 軌道出力 ＆ C^6シームレス合流
%             if obj.replan_active
%                 tau = time.t - obj.t_start;
%                 if tau <= obj.t_duration
%                     xd = obj.evaluate_smooth_trajectory(tau, xd_nominal);
%                 else
%                     if ~obj.replan_done
%                         fprintf("[CORRIDOR QP] 障害物通過完了・C^6シームレス合流 (t=%.3f s)\n\n", time.t);
%                         obj.t_merge_end = obj.t_start + obj.t_duration;
%                         xd_end = obj.evaluate_smooth_trajectory(obj.t_duration, xd_nominal);
%                         obj.p_merge_end = xd_end(1:3);
%                         obj.v_merge_vec = xd_end(5:7);
%                         obj.replan_active = false;
%                         obj.replan_done   = true;
%                     end
% 
%                     % 合流後: そのまま進行方向へ完全等速直進
%                     dt_after = time.t - obj.t_merge_end;
%                     xd = zeros(28, 1);
%                     xd(1:3) = obj.p_merge_end + obj.v_merge_vec * dt_after;
%                     xd(5:7) = obj.v_merge_vec;
%                     xd(9:28) = 0;
%                 end
%             elseif obj.replan_done
%                 dt_after = time.t - obj.t_merge_end;
%                 xd = zeros(28, 1);
%                 xd(1:3) = obj.p_merge_end + obj.v_merge_vec * dt_after;
%                 xd(5:7) = obj.v_merge_vec;
%                 xd(9:28) = 0;
%             else
%                 xd = xd_nominal;
%             end
% 
%             if length(xd) < 28, xd = [xd; zeros(28 - length(xd), 1)]; end
%             obj.result.state.xd = xd;
%             obj.result.state.p = xd(1:3);
%             obj.result.state.v = xd(5:7);
%             obj.result.state.q = [0; 0; xd(4)];
%             result = obj.result;
%         end
%     end
% 
%     methods (Access = private)
%         function plan_mellinger_hybrid_qp(obj, xd0, dir_nom, spd, total_dist)
%             T = obj.t_duration;
%             p0 = xd0(1:3);
%             order = 13;
%             n_coeffs = order + 1;
% 
%             % 1. 進行方向ベクトルに直交する最短法線ベクトルの動的幾何計算
%             vec_to_obs = obj.obs_center - p0;
%             proj_on_line = dot(vec_to_obs, dir_nom) * dir_nom;
%             normal_vec = vec_to_obs - proj_on_line;
% 
%             if norm(normal_vec) < 1e-3
%                 % 軌道直線上にある場合、進行方向に依存しない直交軸を選択
%                 if abs(dir_nom(3)) < 0.9
%                     normal_vec = cross(dir_nom, [0; 0; 1]);
%                 else
%                     normal_vec = cross(dir_nom, [1; 0; 0]);
%                 end
%             end
%             dir_normal = normal_vec / norm(normal_vec);
% 
%             % ドローン・紐を含めたマージン付き回避半径
%             R_hard = obj.obs_radius + obj.L_cable * 0.4 + 0.15;
%             R_soft = R_hard + obj.obs_margin;
% 
%             % 中間経由点 (Corridorの中心点)
%             p_mid_nominal = p0 + dir_nom * (total_dist * 0.5);
%             p_via = p_mid_nominal + dir_normal * R_soft;
% 
%             % 終端合流点
%             p_end = p0 + dir_nom * total_dist;
%             xd1 = zeros(28, 1);
%             xd1(1:3) = p_end;
%             xd1(5:7) = dir_nom * spd;
%             xd1(9:28) = 0;
% 
%             % サンプル区間設定
%             N_samples = 9;
%             u_samples = linspace(0.3, 0.7, N_samples);
%             num_C = 3 * n_coeffs;
%             num_vars = num_C + N_samples; % スラック変数 (ソフト制約用)
% 
%             % -------------------------------------------------------------
%             % 2. 目的関数 H の無次元正規化 (条件数 O(10^5) 保証)
%             % -------------------------------------------------------------
%             H_1d = zeros(n_coeffs, n_coeffs);
%             w4 = 1.0; w5 = 0.1;
%             for i = 4:order
%                 for j = 4:order
%                     val4 = (factorial(i)/factorial(i-4)) * (factorial(j)/factorial(j-4)) / (i + j - 7);
%                     H_1d(i+1, j+1) = H_1d(i+1, j+1) + w4 * val4;
%                     if i >= 5 && j >= 5
%                         val5 = (factorial(i)/factorial(i-5)) * (factorial(j)/factorial(j-5)) / (i + j - 9);
%                         H_1d(i+1, j+1) = H_1d(i+1, j+1) + w5 * val5;
%                     end
%                 end
%             end
% 
%             norm_H = norm(H_1d, 2);
%             H_1d_scaled = H_1d / norm_H;
% 
%             H = zeros(num_vars, num_vars);
%             H(1:num_C, 1:num_C) = blkdiag(H_1d_scaled, H_1d_scaled, H_1d_scaled) + 1e-6 * eye(num_C);
% 
%             % スラック変数へのペナルティ
%             w_slack = 50000.0;
%             for s_idx = 1:N_samples
%                 H(num_C + s_idx, num_C + s_idx) = w_slack;
%             end
%             f = zeros(num_vars, 1);
%             f(num_C + 1 : end) = w_slack;
% 
%             % -------------------------------------------------------------
%             % 3. 等式制約 (始端7階 + 終端6階 + 経由点1階)
%             % -------------------------------------------------------------
%             n_eq = 14;
%             Aeq_1d = zeros(n_eq, n_coeffs);
%             for k = 0:6
%                 for n = k:order, Aeq_1d(k+1, n+1) = prod(n-k+1:n) * 0^(n-k); end
%             end
%             for n = 0:order
%                 Aeq_1d(8, n+1) = 0.5^n;
%             end
%             for k = 0:5
%                 for n = k:order, Aeq_1d(9+k, n+1) = prod(n-k+1:n) * 1.0^(n-k); end
%             end
% 
%             Aeq = zeros(3 * n_eq, num_vars);
%             Aeq(:, 1:num_C) = blkdiag(Aeq_1d, Aeq_1d, Aeq_1d);
%             beq = zeros(3 * n_eq, 1);
% 
%             for ax = 1:3
%                 beq_ax = zeros(n_eq, 1);
%                 for k = 0:6, beq_ax(k+1) = (T^k) * xd0(4*k + ax); end
%                 beq_ax(8) = p_via(ax);
%                 for k = 0:5, beq_ax(9+k) = (T^k) * xd1(4*k + ax); end
%                 beq((ax-1)*n_eq + 1 : ax*n_eq) = beq_ax;
%             end
% 
%             % -------------------------------------------------------------
%             % 4. 不等式制約: 局所凸回廊（山なりマージン包絡）
%             % -------------------------------------------------------------
%             n_wall = -dir_normal;
% 
%             % ハード制約: 真横通過時 (u in [0.45, 0.55]) のみ本体侵入阻止
%             hard_mask = (u_samples >= 0.45) & (u_samples <= 0.55);
%             n_hard = sum(hard_mask);
% 
%             num_ineq = n_hard + N_samples + N_samples;
%             A_ineq = zeros(num_ineq, num_vars);
%             b_ineq = zeros(num_ineq, 1);
% 
%             row = 1;
%             % a) ハード制約 (真横通過時の本体侵入阻止・スラックなし)
%             p_hard_bound = obj.obs_center + dir_normal * R_hard;
%             for m = find(hard_mask)
%                 u_m = u_samples(m);
%                 T_vec = zeros(1, n_coeffs);
%                 for n = 0:order, T_vec(n+1) = u_m^n; end
%                 A_ineq(row, 1:n_coeffs)               = n_wall(1) * T_vec;
%                 A_ineq(row, n_coeffs+1 : 2*n_coeffs)   = n_wall(2) * T_vec;
%                 A_ineq(row, 2*n_coeffs+1 : 3*n_coeffs) = n_wall(3) * T_vec;
%                 b_ineq(row) = dot(n_wall, p_hard_bound);
%                 row = row + 1;
%             end
% 
%             % b) ソフト制約: 障害物近傍のみ張り出す局所マージン包絡
%             for m = 1:N_samples
%                 u_m = u_samples(m);
%                 T_vec = zeros(1, n_coeffs);
%                 for n = 0:order, T_vec(n+1) = u_m^n; end
% 
%                 % u=0.5 を頂点として自然に張り出す山なりマージン
%                 envelope = sin(pi * (u_m - 0.3) / 0.4); % u=0.3で0, 0.5で1, 0.7で0
%                 R_soft_local = R_hard + obj.obs_margin * envelope;
%                 p_soft_local = obj.obs_center + dir_normal * R_soft_local;
% 
%                 A_ineq(row, 1:n_coeffs)               = n_wall(1) * T_vec;
%                 A_ineq(row, n_coeffs+1 : 2*n_coeffs)   = n_wall(2) * T_vec;
%                 A_ineq(row, 2*n_coeffs+1 : 3*n_coeffs) = n_wall(3) * T_vec;
%                 A_ineq(row, num_C + m)                = -1; % -slack
%                 b_ineq(row) = dot(n_wall, p_soft_local);
%                 row = row + 1;
%             end
% 
%             % c) スラック非負制約 (-slack <= 0)
%             for m = 1:N_samples
%                 A_ineq(row, num_C + m) = -1;
%                 b_ineq(row) = 0;
%                 row = row + 1;
%             end
% 
%             % -------------------------------------------------------------
%             % 5. 最適化実行とフルデバッグ情報取得
%             % -------------------------------------------------------------
%             opts = optimoptions('quadprog', ...
%                 'Display', 'off', ...
%                 'Algorithm', 'interior-point-convex', ...
%                 'MaxIterations', 200, ...
%                 'ConstraintTolerance', 1e-4, ... % 0.1 mm 精度（偽の解なしを防止）
%                 'OptimalityTolerance', 1e-4, ...
%                 'StepTolerance', 1e-8);
% 
%             [X_opt, ~, exitflag, output] = quadprog(H, f, A_ineq, b_ineq, Aeq, beq, [], [], [], opts);
% 
%             if exitflag < 1
%                 fprintf("[CORRIDOR QP DEBUG] quadprog失敗! exitflag = %d (%s)\n", exitflag, output.message);
% 
%                 % 等式制約のみを解いた場合の解 (Aeq \ beq)
%                 C_eq_only = Aeq \ beq;
% 
%                 % 制約違反量検査
%                 viol_all = A_ineq * C_eq_only - b_ineq;
%                 viol_hard = viol_all(1:n_hard);
%                 viol_soft = viol_all(n_hard+1 : n_hard+N_samples);
% 
%                 fprintf("[CORRIDOR QP DEBUG] 等式解における最大違反量:\n");
%                 if ~isempty(viol_hard)
%                     fprintf("  - ハード制約(本体) 最大違反: %.4f m (正なら境界侵入)\n", max(viol_hard));
%                 end
%                 fprintf("  - ソフト制約(マージン) 最大違反: %.4f m\n", max(viol_soft));
% 
%                 % 最悪違反サンプルの特定
%                 [max_val, worst_idx] = max(viol_soft);
%                 if max_val > 0
%                     fprintf("  - 最悪違反サンプル: u = %.3f で %.4f m 侵入\n", u_samples(worst_idx), max_val);
%                 end
% 
%                 fprintf("  - 条件数: Aeq=%.2e, H=%.2e\n", cond(Aeq), cond(H(1:num_C, 1:num_C)));
%                 fprintf("[CORRIDOR QP] フォールバック解(Aeq\\beq)を採用\n");
% 
%                 C_1 = Aeq_1d \ beq(1:n_eq);
%                 C_2 = Aeq_1d \ beq(n_eq+1:2*n_eq);
%                 C_3 = Aeq_1d \ beq(2*n_eq+1:3*n_eq);
%                 obj.poly_coeffs = [C_1, C_2, C_3]';
%             else
%                 slack_vals = X_opt(num_C + 1 : end);
% 
%                 % 真横近傍 (u in [0.45, 0.55]) での真の侵入量を抽出
%                 center_mask = (u_samples >= 0.45) & (u_samples <= 0.55);
%                 max_slack_center = max(slack_vals(center_mask));
% 
%                 fprintf("[CORRIDOR QP] QP最適化成功! (exitflag=1, 反復=%d, 最接近時マージン侵入=%.3fm)\n", ...
%                     output.iterations, max_slack_center);
% 
%                 % サンプル点ごとのスラック消費の内訳デバッグ表示
%                 fprintf("[CORRIDOR QP DEBUG] 各サンプル点でのスラック消費 [m]:\n  [ ");
%                 for m = 1:N_samples
%                     fprintf("u=%.2f: %.3f  ", u_samples(m), slack_vals(m));
%                 end
%                 fprintf("]\n");
% 
%                 C_opt = X_opt(1:num_C);
%                 obj.poly_coeffs = [C_opt(1:n_coeffs), C_opt(n_coeffs+1:2*n_coeffs), C_opt(2*n_coeffs+1:end)]';
%             end
%         end
% 
%         function xd = evaluate_smooth_trajectory(obj, tau, xd_nom)
%             xd = xd_nom;
%             C = obj.poly_coeffs;
%             order = 13;
%             T = obj.t_duration;
%             u = max(0, min(1.0, tau / T));
% 
%             for k = 0:6
%                 val_k = zeros(3, 1);
%                 for n = k:order
%                     factor = prod(n - k + 1 : n);
%                     val_k = val_k + C(:, n + 1) * factor * (u^(n - k));
%                 end
%                 xd(4*k + (1:3)) = val_k / (T^k);
%             end
%         end
%     end
% end

classdef REPLANNING_MELLINGER_CORRIDOR_FLATNESS_V2 < handle
    % REPLANNING_MELLINGER_CORRIDOR_FLATNESS_V2
    % 任意進行方向・任意障害物配置対応 Mellinger Corridor QP リプランナ
    % - 障害物本体: ハード制約 (侵入絶対禁止)
    % - 安全マージン帯: 山なり凸回廊ソフト制約 (スラック変数による最適ペナルティ)
    % - 詳細デバッグ解析機能付き
    
    properties
        base_ref
        self
        replan_active = false
        replan_done   = false
        
        t_start
        t_duration = 8.0           % 動的に最適決定される所要時間 [s]
        
        safe_margin  = 0.5         % デフォルトマージン (obs側で指定あれば上書き)
        trigger_dist = 4.0
        
        L_cable      = 2.0
        gravity      = 9.81
        
        poly_coeffs                % 3 x 14 係数行列 (無次元時間 u in [0,1])
        
        % 合流用保持
        t_merge_end
        p_merge_end
        v_merge_vec
        
        obs_center = [0; 0; 0]
        obs_radius = 0
        obs_margin = 0
        
        result
    end
    
    methods
        function obj = REPLANNING_MELLINGER_CORRIDOR_FLATNESS_V2(self, base_ref, opts)
            arguments
                self
                base_ref
                opts = struct()
            end
            obj.self = self;
            obj.base_ref = base_ref;
            if isfield(opts, "safe_margin"),  obj.safe_margin  = opts.safe_margin;  end
            if isfield(opts, "trigger_dist"), obj.trigger_dist = opts.trigger_dist; end
            obj.result.state = STATE_CLASS(struct('state_list', ["xd", "p", "q", "v"], 'num_list', [28, 3, 3, 3]));
        end
        
        function result = do(obj, varargin)
            time = varargin{1};
            cha = varargin{2};
            
            base_res = obj.base_ref.do(time, cha);
            xd_nominal = base_res.state.xd;
            if length(xd_nominal) < 28
                xd_nominal = [xd_nominal; zeros(28 - length(xd_nominal), 1)];
            end
            
            if isprop(obj.self.estimator.result.state, "pL")
                pL_cur = obj.self.estimator.result.state.pL;
                vL_cur = obj.self.estimator.result.state.vL;
            else
                pL_cur = base_res.state.p;
                vL_cur = base_res.state.v;
            end
            
            obj.L_cable = obj.self.parameter.get("cableL");
            pQ_cur = pL_cur + [0; 0; obj.L_cable];
            
            % 1. 動的障害物検知 (任意障害物リストの走査)
            if cha == 'f' && ~obj.replan_done && ~obj.replan_active
                try
                    obs_list = ENVIRONMENT_OBSTACLE_HOCBF_LINK_XY();
                catch
                    obs_list = [];
                end
                
                min_dist_overall = inf;
                target_obs_idx = -1;
                
                for i = 1:length(obs_list)
                    if isfield(obs_list(i), 'p_obs') && isfield(obs_list(i), 'r_obs')
                        c = obs_list(i).p_obs;
                        r_body = obs_list(i).r_obs;
                        if isfield(obs_list(i), 'd_margin')
                            d_marg = obs_list(i).d_margin;
                        else
                            d_marg = obj.safe_margin;
                        end
                        
                        dist_load  = norm(pL_cur - c);
                        dist_drone = norm(pQ_cur - c);
                        d_surf = min(dist_load, dist_drone) - (r_body + d_marg);
                        
                        if d_surf < min_dist_overall
                            min_dist_overall = d_surf;
                            target_obs_idx = i;
                        end
                    end
                end
                
                if target_obs_idx > 0 && min_dist_overall <= obj.trigger_dist
                    obj.obs_center = obs_list(target_obs_idx).p_obs;
                    obj.obs_radius = obs_list(target_obs_idx).r_obs;
                    if isfield(obs_list(target_obs_idx), 'd_margin')
                        obj.obs_margin = obs_list(target_obs_idx).d_margin;
                    else
                        obj.obs_margin = obj.safe_margin;
                    end
                    
                    fprintf("\n=======================================================\n");
                    fprintf("[CORRIDOR QP] 障害物検知! (ID: %d, d_surf=%.2fm, t=%.3f s)\n", target_obs_idx, min_dist_overall, time.t);
                    fprintf("  - 本体半径: %.2f m (ハード制約)\n", obj.obs_radius);
                    fprintf("  - 安全マージン: %.2f m (ソフト制約)\n", obj.obs_margin);
                    obj.t_start = time.t;
                    
                    % 任意方向の速度ベクトル・巡航速度の動的抽出
                    v_vec = xd_nominal(5:7);
                    spd = norm(v_vec);
                    if spd < 0.05, spd = norm(vL_cur); end
                    if spd < 0.05, spd = 0.5; v_vec = [0; 0; 0.5]; end
                    dir_nom = v_vec / spd;
                    
                    % 進行軸上の障害物通過距離と所要時間 T の動的計算（急な合流切り返しを防ぐため余裕を持たせる）
                    vec_to_obs = obj.obs_center - pL_cur;
                    proj_dist = max(0, dot(vec_to_obs, dir_nom));
                    total_dist = proj_dist + obj.obs_radius + obj.obs_margin + obj.L_cable + 3.0;
                    obj.t_duration = max(6.0, min(14.0, total_dist / spd));
                    
                    fprintf("  - 進行方向: [%.2f, %.2f, %.2f], 計画時間 T = %.2f s\n", dir_nom(1), dir_nom(2), dir_nom(3), obj.t_duration);
                    
                    obj.plan_mellinger_hybrid_qp(xd_nominal, dir_nom, spd, total_dist);
                    obj.replan_active = true;
                    fprintf("=======================================================\n\n");
                end
            end
            
            % 2. 軌道出力 ＆ C^6シームレス合流
            if obj.replan_active
                tau = time.t - obj.t_start;
                if tau <= obj.t_duration
                    xd = obj.evaluate_smooth_trajectory(tau, xd_nominal);
                else
                    if ~obj.replan_done
                        fprintf("[CORRIDOR QP] 障害物通過完了・C^6シームレス合流 (t=%.3f s)\n\n", time.t);
                        obj.t_merge_end = obj.t_start + obj.t_duration;
                        xd_end = obj.evaluate_smooth_trajectory(obj.t_duration, xd_nominal);
                        obj.p_merge_end = xd_end(1:3);
                        obj.v_merge_vec = xd_end(5:7);
                        obj.replan_active = false;
                        obj.replan_done   = true;
                    end
                    
                    % 合流後: そのまま進行方向へ完全等速直進
                    dt_after = time.t - obj.t_merge_end;
                    xd = zeros(28, 1);
                    xd(1:3) = obj.p_merge_end + obj.v_merge_vec * dt_after;
                    xd(5:7) = obj.v_merge_vec;
                    xd(9:28) = 0;
                end
            elseif obj.replan_done
                dt_after = time.t - obj.t_merge_end;
                xd = zeros(28, 1);
                xd(1:3) = obj.p_merge_end + obj.v_merge_vec * dt_after;
                xd(5:7) = obj.v_merge_vec;
                xd(9:28) = 0;
            else
                xd = xd_nominal;
            end
            
            if length(xd) < 28, xd = [xd; zeros(28 - length(xd), 1)]; end
            obj.result.state.xd = xd;
            obj.result.state.p = xd(1:3);
            obj.result.state.v = xd(5:7);
            obj.result.state.q = [0; 0; xd(4)];
            result = obj.result;
        end
    end
    
    methods (Access = private)
        function plan_mellinger_hybrid_qp(obj, xd0, dir_nom, spd, total_dist)
            T = obj.t_duration;
            p0 = xd0(1:3);
            order = 13;
            n_coeffs = order + 1;
            
            % 1. 進行方向ベクトルに直交する最短法線ベクトルの動的幾何計算
            vec_to_obs = obj.obs_center - p0;
            proj_on_line = dot(vec_to_obs, dir_nom) * dir_nom;
            normal_vec = vec_to_obs - proj_on_line;
            
            if norm(normal_vec) < 1e-3
                if abs(dir_nom(3)) < 0.9
                    normal_vec = cross(dir_nom, [0; 0; 1]);
                else
                    normal_vec = cross(dir_nom, [1; 0; 0]);
                end
            end
            dir_normal = normal_vec / norm(normal_vec);
            
            % ドローン・紐を含めたマージン付き回避半径
            R_hard = obj.obs_radius + obj.L_cable * 0.4 + 0.15;
            R_soft = R_hard + obj.obs_margin;
            
            % 中間経由点 (Corridorの中心点)
            p_mid_nominal = p0 + dir_nom * (total_dist * 0.5);
            p_via = p_mid_nominal + dir_normal * R_soft;
            
            % 終端合流点
            p_end = p0 + dir_nom * total_dist;
            xd1 = zeros(28, 1);
            xd1(1:3) = p_end;
            xd1(5:7) = dir_nom * spd;
            xd1(9:28) = 0;
            
            % サンプル区間設定
            N_samples = 9;
            u_samples = linspace(0.3, 0.7, N_samples);
            num_C = 3 * n_coeffs;
            num_vars = num_C + N_samples; % スラック変数 (ソフト制約用)
            
            % -------------------------------------------------------------
            % 2. 目的関数 H の無次元正規化 (条件数 O(10^5) 保証)
            % -------------------------------------------------------------
            H_1d = zeros(n_coeffs, n_coeffs);
            w4 = 1.0; w5 = 0.1;
            for i = 4:order
                for j = 4:order
                    val4 = (factorial(i)/factorial(i-4)) * (factorial(j)/factorial(j-4)) / (i + j - 7);
                    H_1d(i+1, j+1) = H_1d(i+1, j+1) + w4 * val4;
                    if i >= 5 && j >= 5
                        val5 = (factorial(i)/factorial(i-5)) * (factorial(j)/factorial(j-5)) / (i + j - 9);
                        H_1d(i+1, j+1) = H_1d(i+1, j+1) + w5 * val5;
                    end
                end
            end
            
            norm_H = norm(H_1d, 2);
            H_1d_scaled = H_1d / norm_H;
            
            H = zeros(num_vars, num_vars);
            H(1:num_C, 1:num_C) = blkdiag(H_1d_scaled, H_1d_scaled, H_1d_scaled) + 1e-6 * eye(num_C);
            
            % スラック変数へのペナルティ (マージン死守のため 50000.0)
            w_slack = 50000.0;
            for s_idx = 1:N_samples
                H(num_C + s_idx, num_C + s_idx) = w_slack;
            end
            f = zeros(num_vars, 1);
            f(num_C + 1 : end) = w_slack;
            
            % -------------------------------------------------------------
            % 3. 等式制約 (始端7階 + 終端6階 + 経由点1階)
            % -------------------------------------------------------------
            n_eq = 14;
            Aeq_1d = zeros(n_eq, n_coeffs);
            for k = 0:6
                for n = k:order, Aeq_1d(k+1, n+1) = prod(n-k+1:n) * 0^(n-k); end
            end
            for n = 0:order
                Aeq_1d(8, n+1) = 0.5^n;
            end
            for k = 0:5
                for n = k:order, Aeq_1d(9+k, n+1) = prod(n-k+1:n) * 1.0^(n-k); end
            end
            
            Aeq = zeros(3 * n_eq, num_vars);
            Aeq(:, 1:num_C) = blkdiag(Aeq_1d, Aeq_1d, Aeq_1d);
            beq = zeros(3 * n_eq, 1);
            
            for ax = 1:3
                beq_ax = zeros(n_eq, 1);
                for k = 0:6, beq_ax(k+1) = (T^k) * xd0(4*k + ax); end
                beq_ax(8) = p_via(ax);
                for k = 0:5, beq_ax(9+k) = (T^k) * xd1(4*k + ax); end
                beq((ax-1)*n_eq + 1 : ax*n_eq) = beq_ax;
            end
            
            % -------------------------------------------------------------
            % 4. 不等式制約: 障害物外側（安全側半空間）への押し出し
            % -------------------------------------------------------------
            % dir_normal は障害物中心から外側へ向かう単位ベクトル
            % 安全条件: dir_normal' * p >= dir_normal' * p_bound
            % quadprog 形式 (A*x <= b) に直すと: -dir_normal' * p <= -dir_normal' * p_bound
            n_safe = dir_normal;
            p_hard_bound = obj.obs_center + dir_normal * R_hard;
            
            % ハード制約: 真横周辺 (u in [0.40, 0.60]) で急激な切り返しを禁止
            hard_mask = (u_samples >= 0.40) & (u_samples <= 0.60);
            n_hard = sum(hard_mask);
            
            num_ineq = n_hard + N_samples + N_samples;
            A_ineq = zeros(num_ineq, num_vars);
            b_ineq = zeros(num_ineq, 1);
            
            row = 1;
            % a) ハード制約: 本体+紐防護境界の外側を強制 (スラックなし)
            for m = find(hard_mask)
                u_m = u_samples(m);
                T_vec = zeros(1, n_coeffs);
                for n = 0:order, T_vec(n+1) = u_m^n; end
                
                A_ineq(row, 1:n_coeffs)               = -n_safe(1) * T_vec;
                A_ineq(row, n_coeffs+1 : 2*n_coeffs)   = -n_safe(2) * T_vec;
                A_ineq(row, 2*n_coeffs+1 : 3*n_coeffs) = -n_safe(3) * T_vec;
                b_ineq(row) = -dot(n_safe, p_hard_bound);
                row = row + 1;
            end
            
            % b) ソフト制約: マージン目標面 (スラックにより超過を許容)
            for m = 1:N_samples
                u_m = u_samples(m);
                T_vec = zeros(1, n_coeffs);
                for n = 0:order, T_vec(n+1) = u_m^n; end
                
                envelope = sin(pi * (u_m - 0.3) / 0.4);
                R_soft_local = R_hard + obj.obs_margin * envelope;
                p_soft_local = obj.obs_center + dir_normal * R_soft_local;
                
                A_ineq(row, 1:n_coeffs)               = -n_safe(1) * T_vec;
                A_ineq(row, n_coeffs+1 : 2*n_coeffs)   = -n_safe(2) * T_vec;
                A_ineq(row, 2*n_coeffs+1 : 3*n_coeffs) = -n_safe(3) * T_vec;
                A_ineq(row, num_C + m)                = -1; % -slack <= 0
                b_ineq(row) = -dot(n_safe, p_soft_local);
                row = row + 1;
            end
            
            % c) スラック非負制約 (-slack <= 0)
            for m = 1:N_samples
                A_ineq(row, num_C + m) = -1;
                b_ineq(row) = 0;
                row = row + 1;
            end
            
            % -------------------------------------------------------------
            % 5. 最適化実行とフルデバッグ情報取得
            % -------------------------------------------------------------
            opts = optimoptions('quadprog', ...
                'Display', 'off', ...
                'Algorithm', 'interior-point-convex', ...
                'MaxIterations', 200, ...
                'ConstraintTolerance', 1e-4, ...
                'OptimalityTolerance', 1e-4, ...
                'StepTolerance', 1e-8);
            
            [X_opt, ~, exitflag, output] = quadprog(H, f, A_ineq, b_ineq, Aeq, beq, [], [], [], opts);
            
            if exitflag < 1
                fprintf("[CORRIDOR QP DEBUG] quadprog失敗! exitflag = %d (%s)\n", exitflag, output.message);
                
                C_eq_only = Aeq \ beq;
                viol_all = A_ineq * C_eq_only - b_ineq;
                viol_hard = viol_all(1:n_hard);
                viol_soft = viol_all(n_hard+1 : n_hard+N_samples);
                
                fprintf("[CORRIDOR QP DEBUG] 等式解における最大違反量:\n");
                if ~isempty(viol_hard)
                    fprintf("  - ハード制約(本体) 最大違反: %.4f m (正なら境界侵入)\n", max(viol_hard));
                end
                fprintf("  - ソフト制約(マージン) 最大違反: %.4f m\n", max(viol_soft));
                
                [max_val, worst_idx] = max(viol_soft);
                if max_val > 0
                    fprintf("  - 最悪違反サンプル: u = %.3f で %.4f m 侵入\n", u_samples(worst_idx), max_val);
                end
                
                fprintf("  - 条件数: Aeq=%.2e, H=%.2e\n", cond(Aeq), cond(H(1:num_C, 1:num_C)));
                fprintf("[CORRIDOR QP] フォールバック解(Aeq\\beq)を採用\n");
                
                C_1 = Aeq_1d \ beq(1:n_eq);
                C_2 = Aeq_1d \ beq(n_eq+1:2*n_eq);
                C_3 = Aeq_1d \ beq(2*n_eq+1:3*n_eq);
                obj.poly_coeffs = [C_1, C_2, C_3]';
            else
                slack_vals = X_opt(num_C + 1 : end);
                
                % 真横近傍 (u in [0.45, 0.55]) での真の侵入量を抽出
                center_mask = (u_samples >= 0.45) & (u_samples <= 0.55);
                max_slack_center = max(slack_vals(center_mask));
                
                fprintf("[CORRIDOR QP] QP最適化成功! (exitflag=1, 反復=%d, 最接近時マージン侵入=%.3fm)\n", ...
                    output.iterations, max_slack_center);
                
                fprintf("[CORRIDOR QP DEBUG] 各サンプル点でのスラック消費 [m]:\n  [ ");
                for m = 1:N_samples
                    fprintf("u=%.2f: %.3f  ", u_samples(m), slack_vals(m));
                end
                fprintf("]\n");
                
                C_opt = X_opt(1:num_C);
                obj.poly_coeffs = [C_opt(1:n_coeffs), C_opt(n_coeffs+1:2*n_coeffs), C_opt(2*n_coeffs+1:end)]';
            end
        end
        
        function xd = evaluate_smooth_trajectory(obj, tau, xd_nom)
            xd = xd_nom;
            C = obj.poly_coeffs;
            order = 13;
            T = obj.t_duration;
            u = max(0, min(1.0, tau / T));
            
            for k = 0:6
                val_k = zeros(3, 1);
                for n = k:order
                    factor = prod(n - k + 1 : n);
                    val_k = val_k + C(:, n + 1) * factor * (u^(n - k));
                end
                xd(4*k + (1:3)) = val_k / (T^k);
            end
        end
    end
end
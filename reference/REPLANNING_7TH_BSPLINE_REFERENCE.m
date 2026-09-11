% classdef REPLANNING_7TH_BSPLINE_REFERENCE < handle
%     % REPLANNING_7TH_BSPLINE_REFERENCE (デバッグ・純粋アルゴリズム版・1つ前の状態)
% 
%     properties
%         base_ref
%         self
%         replan_active = false
%         replan_done   = false
% 
%         t_start
%         t_duration = 5.0           % 回避時間 [s]
% 
%         obs_center = [0; 0; 4.0]
%         obs_radius = 0.3
%         safe_margin = 0.8
%         trigger_dist = 4.0
% 
%         p = 7;
%         num_free_pts = 7;
%         n;
%         U;
%         Q;
% 
%         L_cable = 2.0;
%         result
%     end
% 
%     methods
%         function obj = REPLANNING_7TH_BSPLINE_REFERENCE(self, base_ref, opts)
%             arguments
%                 self
%                 base_ref
%                 opts = struct()
%             end
%             obj.self = self;
%             obj.base_ref = base_ref;
% 
%             if isfield(opts, "obs_center"),   obj.obs_center   = opts.obs_center;   end
%             if isfield(opts, "obs_radius"),   obj.obs_radius   = opts.obs_radius;   end
%             if isfield(opts, "safe_margin"),  obj.safe_margin  = opts.safe_margin;  end
%             if isfield(opts, "trigger_dist"), obj.trigger_dist = opts.trigger_dist; end
%             if isfield(opts, "t_duration"),   obj.t_duration   = opts.t_duration;   end
%             if isfield(opts, "num_free_pts"), obj.num_free_pts = opts.num_free_pts; end
% 
%             obj.n = 2 * obj.p + obj.num_free_pts - 1; 
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
%             else
%                 pL_cur = base_res.state.p;
%             end
% 
%             obj.L_cable = obj.self.parameter.get("cableL");
%             pQ_cur = pL_cur + [0; 0; obj.L_cable]; 
% 
%             if cha == 'f' && ~obj.replan_done && ~obj.replan_active
%                 dist_load_obs = norm(pL_cur - obj.obs_center);
%                 dist_drone_obs = norm(pQ_cur - obj.obs_center);
%                 min_dist = min(dist_load_obs, dist_drone_obs);
% 
%                 if min_dist <= obj.trigger_dist
%                     fprintf("\n=======================================================\n");
%                     fprintf("[DEBUG] 障害物接近検知 (dist=%.2fm). リプランニング開始 at t=%.3f s\n", min_dist, time.t);
%                     obj.t_start = time.t;
% 
%                     state_start = zeros(3, 7); 
%                     state_start(:, 1) = pL_cur;
%                     if isprop(obj.self.estimator.result.state, "vL")
%                         state_start(:, 2) = obj.self.estimator.result.state.vL;
%                     end
% 
%                     t_end = obj.t_start + obj.t_duration;
%                     time_end = time; time_end.t = t_end;
%                     base_res_end = obj.base_ref.do(time_end, cha);
%                     xd_end = base_res_end.state.xd;
%                     if length(xd_end) < 28
%                         xd_end = [xd_end; zeros(28 - length(xd_end), 1)];
%                     end
%                     state_end = zeros(3, 7);
%                     for k = 1:7
%                         state_end(:, k) = xd_end( 4*(k-1) + (1:3) );
%                     end
% 
%                     obj.plan_bspline_trajectory(state_start, state_end);
%                     obj.replan_active = true;
%                     fprintf("=======================================================\n\n");
%                 end
%             end
% 
%             if obj.replan_active
%                 t_tau = time.t - obj.t_start;
%                 if t_tau <= obj.t_duration
%                     xd = obj.evaluate_bspline_trajectory(t_tau);
%                 else
%                     fprintf("[DEBUG] 回避完了・公称軌道へ合流 at t=%.3f s\n", time.t);
%                     obj.replan_active = false;
%                     obj.replan_done = true;
%                     xd = xd_nominal;
%                 end
%             else
%                 xd = xd_nominal;
%             end
% 
%             obj.result.state.xd = xd;
%             obj.result.state.p = xd(1:3);
%             obj.result.state.v = xd(5:7);
%             obj.result.state.q = [0; 0; xd(4)];
%             result = obj.result;
%         end
%     end
% 
%     methods (Access = private)
% 
%         function plan_bspline_trajectory(obj, state_start, state_end)
%             S = obj.n - obj.p + 1;
%             obj.U = zeros(1, obj.n + obj.p + 2);
%             obj.U(1 : obj.p+1) = 0;
%             obj.U(end-obj.p : end) = obj.t_duration;
%             for i = 1:S-1
%                 obj.U(obj.p + 1 + i) = i * (obj.t_duration / S);
%             end
% 
%             dt_knot = obj.t_duration / S;
%             obj.Q = zeros(3, obj.n + 1);
% 
%             for k = 0:obj.p-1
%                 obj.Q(:, k+1) = state_start(:, 1) + state_start(:, 2) * (k * dt_knot / obj.p);
%                 obj.Q(:, obj.n - k + 1) = state_end(:, 1) - state_end(:, 2) * (k * dt_knot / obj.p);
%             end
% 
%             start_free = obj.p;         
%             end_free   = obj.n - obj.p; 
% 
%             for i = start_free : end_free
%                 ratio = (i - start_free + 1) / (end_free - start_free + 2);
%                 obj.Q(:, i+1) = (1 - ratio) * obj.Q(:, start_free) + ratio * obj.Q(:, end_free+2);
% 
%                 % 特異点回避のための微小ノイズ（X, Y方向0.001m）
%                 obj.Q(1, i+1) = obj.Q(1, i+1) + 1e-3;
%             end
% 
%             x0 = reshape(obj.Q(:, start_free+1 : end_free+1), [], 1);
% 
%             [J_init, J_smooth_init, J_obs_init] = obj.objective_function_debug(x0, start_free, end_free);
%             fprintf("\n[DEBUG] 最適化前 コスト: Total=%.4f, Smooth=%.4f, Obs=%.4f\n", J_init, J_smooth_init, J_obs_init);
% 
%             fun = @(x) obj.objective_function(x, start_free, end_free);
% 
%             opts = optimoptions('fminunc', 'Algorithm', 'quasi-newton', ...
%                 'Display', 'iter', 'MaxIterations', 200, 'MaxFunctionEvaluations', 2100, ...
%                 'OptimalityTolerance', 1e-4, 'StepTolerance', 1e-4);
% 
%             try
%                 tic;
%                 [x_opt, fval, exitflag, output] = fminunc(fun, x0, opts);
%                 t_opt = toc;
%                 fprintf("[DEBUG] 最適化完了: %.1f ms (ExitFlag: %d, Iters: %d)\n", t_opt * 1000, exitflag, output.iterations);
%             catch
%                 opts_search = optimset('Display', 'iter', 'MaxIter', 400, 'MaxFunEvals', 2100);
%                 [x_opt, fval, exitflag, output] = fminsearch(fun, x0, opts_search);
%                 fprintf("[DEBUG] fminsearch 完了 (ExitFlag: %d)\n", exitflag);
%             end
% 
%             [J_opt, J_smooth_opt, J_obs_opt] = obj.objective_function_debug(x_opt, start_free, end_free);
%             fprintf("[DEBUG] 最適化後 コスト: Total=%.4f, Smooth=%.4f, Obs=%.4f\n\n", J_opt, J_smooth_opt, J_obs_opt);
% 
%             obj.Q(:, start_free+1 : end_free+1) = reshape(x_opt, 3, []);
%         end
% 
%         function J = objective_function(obj, x, start_free, end_free)
%             Q_temp = obj.Q;
%             Q_temp(:, start_free+1 : end_free+1) = reshape(x, 3, []);
% 
%             w_smooth = 1.0;
%             J_smooth = 0;
%             for i = 1 : (obj.n + 1 - 3)
%                 diff3 = Q_temp(:, i+3) - 3*Q_temp(:, i+2) + 3*Q_temp(:, i+1) - Q_temp(:, i);
%                 J_smooth = J_smooth + sum(diff3.^2);
%             end
% 
%             w_obs = 10000.0;
%             J_obs = 0;
%             d_safe = obj.obs_radius + obj.safe_margin;
% 
%             for i = 1 : obj.n + 1
%                 dist_load = norm(Q_temp(:, i) - obj.obs_center);
%                 if dist_load < d_safe
%                     J_obs = J_obs + (d_safe - dist_load)^3;
%                 end
% 
%                 dist_drone = norm(Q_temp(:, i) + [0; 0; obj.L_cable] - obj.obs_center);
%                 if dist_drone < d_safe
%                     J_obs = J_obs + (d_safe - dist_drone)^3;
%                 end
%             end
% 
%             J = w_smooth * J_smooth + w_obs * J_obs;
%         end
% 
%         function [J, J_smooth, J_obs] = objective_function_debug(obj, x, start_free, end_free)
%             Q_temp = obj.Q;
%             Q_temp(:, start_free+1 : end_free+1) = reshape(x, 3, []);
% 
%             w_smooth = 1.0;
%             J_smooth = 0;
%             for i = 1 : (obj.n + 1 - 3)
%                 diff3 = Q_temp(:, i+3) - 3*Q_temp(:, i+2) + 3*Q_temp(:, i+1) - Q_temp(:, i);
%                 J_smooth = J_smooth + sum(diff3.^2);
%             end
% 
%             w_obs = 10000.0;
%             J_obs = 0;
%             d_safe = obj.obs_radius + obj.safe_margin;
% 
%             fprintf("  [Obs Check] d_safe = %.2f\n", d_safe);
%             for i = 1 : obj.n + 1
%                 dist_load = norm(Q_temp(:, i) - obj.obs_center);
%                 if dist_load < d_safe
%                     J_obs = J_obs + (d_safe - dist_load)^3;
%                     fprintf("    - CtrlPt %d (Load): Z=%.2f, dist=%.3f (< %.2f) -> Violation!\n", i, Q_temp(3,i), dist_load, d_safe);
%                 end
% 
%                 pQ = Q_temp(:, i) + [0; 0; obj.L_cable];
%                 dist_drone = norm(pQ - obj.obs_center);
%                 if dist_drone < d_safe
%                     J_obs = J_obs + (d_safe - dist_drone)^3;
%                     fprintf("    - CtrlPt %d (Drone): Z=%.2f, dist=%.3f (< %.2f) -> Violation!\n", i, pQ(3), dist_drone, d_safe);
%                 end
%             end
% 
%             J_smooth = w_smooth * J_smooth;
%             J_obs = w_obs * J_obs;
%             J = J_smooth + J_obs;
%         end
% 
%         function xd = evaluate_bspline_trajectory(obj, t)
%             xd = zeros(28, 1);
%             t = max(0, min(obj.t_duration - 1e-6, t)); 
% 
%             k_span = obj.p + 1;
%             for i = obj.p+1 : obj.n
%                 if t >= obj.U(i) && t < obj.U(i+1)
%                     k_span = i;
%                     break;
%                 end
%             end
% 
%             for d = 0:6
%                 pt_d = obj.evaluate_derivative(t, k_span, d);
%                 idx = 4 * d + 1;
%                 xd(idx : idx+2) = pt_d;
%                 xd(idx+3) = 0; 
%             end
%         end
% 
%         function pt = evaluate_derivative(obj, t, k_span, d)
%             p_cur = obj.p - d;
%             Q_d = zeros(3, p_cur + 1);
% 
%             for i = 0 : p_cur
%                 ctrl_idx = k_span - p_cur + i;
%                 Q_d(:, i+1) = obj.get_derivative_control_point(ctrl_idx, d);
%             end
% 
%             for r = 1 : p_cur
%                 for i = p_cur : -1 : r
%                     knot_idx = k_span - p_cur + i;
%                     denom = obj.U(knot_idx + p_cur - r + d + 1) - obj.U(knot_idx + d);
%                     if denom == 0
%                         alpha = 0;
%                     else
%                         alpha = (t - obj.U(knot_idx + d)) / denom;
%                     end
%                     Q_d(:, i+1) = (1 - alpha) * Q_d(:, i) + alpha * Q_d(:, i+1);
%                 end
%             end
%             pt = Q_d(:, p_cur + 1);
%         end
% 
%         function Qd = get_derivative_control_point(obj, idx, d)
%             if d == 0
%                 Qd = obj.Q(:, idx);
%             else
%                 Qd_prev_i   = obj.get_derivative_control_point(idx, d-1);
%                 Qd_prev_im1 = obj.get_derivative_control_point(idx-1, d-1);
%                 denom = obj.U(idx + obj.p + 1) - obj.U(idx + d);
%                 if denom == 0
%                     Qd = zeros(3,1);
%                 else
%                     Qd = (obj.p - d + 1) / denom * (Qd_prev_i - Qd_prev_im1);
%                 end
%             end
%         end
% 
%     end
% end


% classdef REPLANNING_7TH_BSPLINE_REFERENCE < handle
%     % 厳密C^6連続・システム全体包絡QP多項式リプランナ
%     properties
%         base_ref
%         self
%         replan_active = false
%         replan_done   = false
% 
%         t_start
%         t_duration = 10.0          % 回避・通過時間 [s] (無理のない加速度)
% 
%         obs_center = [0.2; 0.2; 10.0]
%         obs_radius = 0.3
%         safe_margin = 0.5
%         trigger_dist = 3.5
% 
%         L_cable = 2.0
%         poly_coeffs                % 3 x 14 係数行列
%         t_merge_abs                % 合流先の公称時間
%         result
%     end
% 
%     methods
%         function obj = REPLANNING_7TH_BSPLINE_REFERENCE(self, base_ref, opts)
%             arguments
%                 self
%                 base_ref
%                 opts = struct()
%             end
%             obj.self = self;
%             obj.base_ref = base_ref;
% 
%             if isfield(opts, "obs_center"),   obj.obs_center   = opts.obs_center;   end
%             if isfield(opts, "obs_radius"),   obj.obs_radius   = opts.obs_radius;   end
%             if isfield(opts, "safe_margin"),  obj.safe_margin  = opts.safe_margin;  end
%             if isfield(opts, "trigger_dist"), obj.trigger_dist = opts.trigger_dist; end
%             if isfield(opts, "t_duration"),   obj.t_duration   = opts.t_duration;   end
% 
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
%             % 1. 接近検知トリガー (荷物・ドローン両方の距離を監視)
%             if cha == 'f' && ~obj.replan_done && ~obj.replan_active
%                 dist_load_obs  = norm(pL_cur - obj.obs_center);
%                 dist_drone_obs = norm(pQ_cur - obj.obs_center);
%                 min_dist = min(dist_load_obs, dist_drone_obs);
% 
%                 if min_dist <= obj.trigger_dist
%                     fprintf("\n=======================================================\n");
%                     fprintf("[REPLAN] 障害物接近検知! (min_dist=%.2fm, t=%.3f s)\n", min_dist, time.t);
%                     obj.t_start = time.t;
% 
%                     % 速度から適正な回避時間を動的決定
%                     spd = norm(vL_cur);
%                     if spd < 0.1, spd = 0.3; end
%                     % 障害物を通過して十分先まで行く時間を設定
%                     dist_to_pass = (obj.obs_center(3) + obj.obs_radius + obj.L_cable + 1.0) - pL_cur(3);
%                     obj.t_duration = max(8.0, dist_to_pass / spd);
% 
%                     fprintf("[REPLAN] 動的計算された安全回避時間 T = %.2f 秒\n", obj.t_duration);
% 
%                     obj.plan_exact_avoidance(time.t, xd_nominal, pL_cur);
%                     obj.replan_active = true;
%                     fprintf("=======================================================\n\n");
%                 end
%             end
% 
%             % 2. 軌道出力 ＆ 空間通過判定
%             if obj.replan_active
%                 tau = time.t - obj.t_start;
% 
%                 % 荷物もドローンも障害物の頂点 + マージンを通過したか
%                 obs_top_z = obj.obs_center(3) + obj.obs_radius + obj.safe_margin;
%                 is_passed = (pL_cur(3) >= obs_top_z) && (tau >= obj.t_duration * 0.8);
% 
%                 if tau <= obj.t_duration && ~is_passed
%                     xd = obj.evaluate_poly_xd(tau, xd_nominal);
%                 else
%                     if ~obj.replan_done
%                         fprintf("[REPLAN] 障害物を安全クリア! 公称軌道へ合流 (t=%.3f s)\n\n", time.t);
%                     end
%                     obj.replan_active = false;
%                     obj.replan_done = true;
% 
%                     % 合流時は時間シフトした公称軌道を呼び出す
%                     t_shifted = obj.t_merge_abs + (time.t - (obj.t_start + obj.t_duration));
%                     time_s = time; time_s.t = t_shifted;
%                     res_s = obj.base_ref.do(time_s, cha);
%                     xd = res_s.state.xd;
%                 end
%             else
%                 xd = xd_nominal;
%             end
% 
%             if length(xd) < 28
%                 xd = [xd; zeros(28 - length(xd), 1)];
%             end
% 
%             obj.result.state.xd = xd;
%             obj.result.state.p = xd(1:3);
%             obj.result.state.v = xd(5:7);
%             obj.result.state.q = [0; 0; xd(4)];
%             result = obj.result;
%         end
%     end
% 
%     methods (Access = private)
%         function plan_exact_avoidance(obj, t0, xd0, pL_cur)
%             T = obj.t_duration;
% 
%             % 経由点（Via-point: 障害物の横を通過する点）の幾何計算
%             % 障害物中心から法線方向（外側）へドローン＋紐全体を逃がす
%             R_clear = obj.obs_radius + obj.safe_margin + obj.L_cable * 0.5 + 0.3;
% 
%             % 障害物の位置に応じた逃げ方向（原点から外側へ）
%             d_xy = [obj.obs_center(1); obj.obs_center(2)];
%             if norm(d_xy) < 0.05
%                 dir_avoid = [1; 0];
%             else
%                 dir_avoid = d_xy / norm(d_xy);
%             end
% 
%             p_via = zeros(3, 1);
%             p_via(1:2) = obj.obs_center(1:2) + dir_avoid * R_clear;
%             p_via(3)   = obj.obs_center(3); % 障害物の真横の高さ
% 
%             % 合流目標 (t0 + T における安全な公称位置)
%             % 高さ z が障害物頂点より確実に上に来る未来時刻を計算
%             obs_top_z = obj.obs_center(3) + obj.obs_radius + obj.safe_margin;
%             z_target = obs_top_z + obj.L_cable + 1.0;
% 
%             % gen_ref_line (z = p0_z + v*t) から逆算
%             v_nom = norm(xd0(5:7)); if v_nom < 0.1, v_nom = 0.3; end
%             p0_z = 3.0; % メイン設定の開始高度
%             t_needed = (z_target - p0_z) / v_nom;
%             obj.t_merge_abs = max(t0 + T, t_needed);
% 
%             ref_f = obj.base_ref.func;
%             xd1 = ref_f(obj.t_merge_abs);
%             if length(xd1) < 28
%                 xd1 = [xd1; zeros(28 - length(xd1), 1)];
%             end
% 
%             % -------------------------------------------------------------
%             % 13次多項式 QP（Closed-form 解）
%             % t=0  : 0〜6階微分 (7拘束) -> 直前軌道と厳密一致
%             % t=T/2: 位置 = p_via (1拘束) -> 障害物の外側を確実に通過
%             % t=T  : 0〜5階微分 (6拘束) -> 合流先軌道と厳密一致
%             % 計14拘束 = 14次正方行列の一発逆行列計算（反復なし・破綻なし）
%             % -------------------------------------------------------------
%             order = 13;
%             A = zeros(14, 14);
%             Bx = zeros(14, 3);
% 
%             % 1. t = 0 拘束 (現在状態を完全引き継ぎ)
%             state0 = zeros(7, 3);
%             for k = 0:6
%                 state0(k+1, :) = xd0(4*k + (1:3))';
%             end
%             state0(1, :) = pL_cur'; % 位置は実測でリセットしてジャンプ防止
% 
%             for k = 0:6
%                 row = k + 1;
%                 for n = k:order
%                     A(row, n + 1) = prod(n - k + 1 : n) * (0)^(n - k);
%                 end
%                 Bx(row, :) = state0(k+1, :);
%             end
% 
%             % 2. t = T/2 経由点拘束
%             t_mid = T / 2;
%             row = 8;
%             for n = 0:order
%                 A(row, n + 1) = t_mid^n;
%             end
%             Bx(row, :) = p_via';
% 
%             % 3. t = T 合流拘束
%             for k = 0:5
%                 row = 9 + k;
%                 for n = k:order
%                     A(row, n + 1) = prod(n - k + 1 : n) * (T)^(n - k);
%                 end
%                 Bx(row, :) = xd1(4*k + (1:3))';
%             end
% 
%             % 厳密解の算出
%             C = A \ Bx;
%             obj.poly_coeffs = C'; % 3 x 14
%         end
% 
%         function xd = evaluate_poly_xd(obj, tau, xd_nom)
%             xd = xd_nom;
%             C = obj.poly_coeffs;
%             order = 13;
% 
%             for k = 0:6
%                 val_k = zeros(3, 1);
%                 for n = k:order
%                     factor = prod(n - k + 1 : n);
%                     val_k = val_k + C(:, n + 1) * factor * (tau^(n - k));
%                 end
%                 xd(4*k + (1:3)) = val_k;
%             end
%         end
%     end
% end

% classdef REPLANNING_7TH_BSPLINE_REFERENCE < handle
%     % 厳密C^6連続・段差ゼロ合流QP多項式リプランナ
%     properties
%         base_ref
%         self
%         replan_active = false
%         replan_done   = false
% 
%         t_start
%         t_duration = 10.0          % 回避・通過時間 [s]
% 
%         obs_center = [0.2; 0.2; 10.0]
%         obs_radius = 0.3
%         safe_margin = 0.5
%         trigger_dist = 3.5
% 
%         L_cable = 2.0
%         poly_coeffs                % 3 x 14 係数行列
% 
%         % 合流用状態保持
%         t_merged_actual            % 実際に合流処理に入った時刻
%         p_merged_init              % 合流瞬間の位置 [3x1]
%         v_nominal_dir              % 公称進行方向 [3x1]
%         v_nominal_speed = 0.3      % 公称速度 [m/s]
% 
%         result
%     end
% 
%     methods
%         function obj = REPLANNING_7TH_BSPLINE_REFERENCE(self, base_ref, opts)
%             arguments
%                 self
%                 base_ref
%                 opts = struct()
%             end
%             obj.self = self;
%             obj.base_ref = base_ref;
% 
%             if isfield(opts, "obs_center"),   obj.obs_center   = opts.obs_center;   end
%             if isfield(opts, "obs_radius"),   obj.obs_radius   = opts.obs_radius;   end
%             if isfield(opts, "safe_margin"),  obj.safe_margin  = opts.safe_margin;  end
%             if isfield(opts, "trigger_dist"), obj.trigger_dist = opts.trigger_dist; end
%             if isfield(opts, "t_duration"),   obj.t_duration   = opts.t_duration;   end
% 
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
%             % 1. 接近検知トリガー
%             if cha == 'f' && ~obj.replan_done && ~obj.replan_active
%                 dist_load_obs  = norm(pL_cur - obj.obs_center);
%                 dist_drone_obs = norm(pQ_cur - obj.obs_center);
%                 min_dist = min(dist_load_obs, dist_drone_obs);
% 
%                 if min_dist <= obj.trigger_dist
%                     fprintf("\n=======================================================\n");
%                     fprintf("[REPLAN] 障害物接近検知! (min_dist=%.2fm, t=%.3f s)\n", min_dist, time.t);
%                     obj.t_start = time.t;
% 
%                     spd = norm(vL_cur);
%                     if spd < 0.1, spd = 0.3; end
%                     obj.v_nominal_speed = spd;
% 
%                     % 障害物を通過して余裕を持った距離までの所要時間を算出
%                     dist_to_pass = (obj.obs_center(3) + obj.obs_radius + obj.L_cable + 1.5) - pL_cur(3);
%                     obj.t_duration = max(8.0, dist_to_pass / spd);
% 
%                     fprintf("[REPLAN] 安全回避計画時間 T = %.2f 秒\n", obj.t_duration);
% 
%                     obj.plan_exact_avoidance(time.t, xd_nominal, pL_cur);
%                     obj.replan_active = true;
%                     fprintf("=======================================================\n\n");
%                 end
%             end
% 
%             % 2. 軌道出力 ＆ 段差ゼロ合流
%             if obj.replan_active
%                 tau = time.t - obj.t_start;
% 
%                 obs_top_z = obj.obs_center(3) + obj.obs_radius + obj.safe_margin;
%                 % 荷物・ドローン双方が安全高度を通過し、かつ計画時間の85%以上を経過しているか
%                 is_passed = (pL_cur(3) >= obs_top_z) && (tau >= obj.t_duration * 0.85);
% 
%                 if tau < obj.t_duration && ~is_passed
%                     xd = obj.evaluate_poly_xd(tau, xd_nominal);
%                 else
%                     % 合流瞬間の初期化処理
%                     if ~obj.replan_done
%                         fprintf("[REPLAN] 障害物通過完了・段差ゼロで継続上昇へ移行 (t=%.3f s)\n\n", time.t);
%                         obj.t_merged_actual = time.t;
% 
%                         % 直前フレームの目標位置を取得して接続点とする
%                         xd_prev = obj.evaluate_poly_xd(min(tau, obj.t_duration), xd_nominal);
%                         obj.p_merged_init = xd_prev(1:3);
% 
%                         % 進行方向ベクトルの決定 (公称速度方向)
%                         if norm(xd_nominal(5:7)) > 0.01
%                             obj.v_nominal_dir = xd_nominal(5:7) / norm(xd_nominal(5:7));
%                         else
%                             obj.v_nominal_dir = [0; 0; 1];
%                         end
%                         obj.replan_active = false;
%                         obj.replan_done   = true;
%                     end
% 
%                     % 合流後の目標軌道（現在値から完全に途切れなく等速上昇）
%                     dt_after = time.t - obj.t_merged_actual;
%                     xd = zeros(28, 1);
% 
%                     % 位置: 接続点から進行方向へ等速前進
%                     % XY方向の膨らみは4秒かけて滑らかにゼロへ戻す減衰フィルタ
%                     decay = exp(-dt_after / 1.5);
%                     p_xy_decay = [obj.p_merged_init(1) * decay; obj.p_merged_init(2) * decay];
%                     p_z_linear = obj.p_merged_init(3) + obj.v_nominal_speed * dt_after;
% 
%                     xd(1:3) = [p_xy_decay(1); p_xy_decay(2); p_z_linear];
%                     xd(5:7) = obj.v_nominal_speed * obj.v_nominal_dir;
%                     xd(9:28) = 0; % 加速度以降はゼロ（等速直進）
%                 end
%             elseif obj.replan_done
%                 % 合流完了後の継続区間
%                 dt_after = time.t - obj.t_merged_actual;
%                 xd = zeros(28, 1);
%                 decay = exp(-dt_after / 1.5);
%                 p_xy_decay = [obj.p_merged_init(1) * decay; obj.p_merged_init(2) * decay];
%                 p_z_linear = obj.p_merged_init(3) + obj.v_nominal_speed * dt_after;
% 
%                 xd(1:3) = [p_xy_decay(1); p_xy_decay(2); p_z_linear];
%                 xd(5:7) = obj.v_nominal_speed * obj.v_nominal_dir;
%                 xd(9:28) = 0;
%             else
%                 xd = xd_nominal;
%             end
% 
%             if length(xd) < 28
%                 xd = [xd; zeros(28 - length(xd), 1)];
%             end
% 
%             obj.result.state.xd = xd;
%             obj.result.state.p = xd(1:3);
%             obj.result.state.v = xd(5:7);
%             obj.result.state.q = [0; 0; xd(4)];
%             result = obj.result;
%         end
%     end
% 
%     methods (Access = private)
%         function plan_exact_avoidance(obj, t0, xd0, pL_cur)
%             T = obj.t_duration;
% 
%             % 経由点（Via-point）: 障害物の外側へドローン・紐全体を逃がす
%             R_clear = obj.obs_radius + obj.safe_margin + obj.L_cable * 0.5 + 0.3;
% 
%             d_xy = [obj.obs_center(1); obj.obs_center(2)];
%             if norm(d_xy) < 0.05
%                 dir_avoid = [1; 0];
%             else
%                 dir_avoid = d_xy / norm(d_xy);
%             end
% 
%             p_via = zeros(3, 1);
%             p_via(1:2) = obj.obs_center(1:2) + dir_avoid * R_clear;
%             p_via(3)   = obj.obs_center(3);
% 
%             % 終端合流点: 障害物の頭上を十分に超えた高度
%             obs_top_z = obj.obs_center(3) + obj.obs_radius + obj.safe_margin;
%             z_end = obs_top_z + obj.L_cable + 1.5;
% 
%             % 終端状態 xd1 の手動構築（公称関数の時刻ズレに依存しない）
%             xd1 = zeros(28, 1);
%             xd1(1:3) = [p_via(1) * 0.3; p_via(2) * 0.3; z_end]; % XYは徐々に戻りつつ上昇
%             xd1(5:7) = [0; 0; obj.v_nominal_speed];             % 等速上昇速度
%             xd1(9:28) = 0;
% 
%             % 13次多項式 QP
%             order = 13;
%             A = zeros(14, 14);
%             Bx = zeros(14, 3);
% 
%             % 1. t = 0 拘束 (現在状態を完全引き継ぎ)
%             state0 = zeros(7, 3);
%             for k = 0:6
%                 state0(k+1, :) = xd0(4*k + (1:3))';
%             end
%             state0(1, :) = pL_cur'; 
% 
%             for k = 0:6
%                 row = k + 1;
%                 for n = k:order
%                     A(row, n + 1) = prod(n - k + 1 : n) * (0)^(n - k);
%                 end
%                 Bx(row, :) = state0(k+1, :);
%             end
% 
%             % 2. t = T/2 経由点拘束
%             t_mid = T / 2;
%             row = 8;
%             for n = 0:order
%                 A(row, n + 1) = t_mid^n;
%             end
%             Bx(row, :) = p_via';
% 
%             % 3. t = T 合流拘束
%             for k = 0:5
%                 row = 9 + k;
%                 for n = k:order
%                     A(row, n + 1) = prod(n - k + 1 : n) * (T)^(n - k);
%                 end
%                 Bx(row, :) = xd1(4*k + (1:3))';
%             end
% 
%             C = A \ Bx;
%             obj.poly_coeffs = C';
%         end
% 
%         function xd = evaluate_poly_xd(obj, tau, xd_nom)
%             xd = xd_nom;
%             C = obj.poly_coeffs;
%             order = 13;
% 
%             for k = 0:6
%                 val_k = zeros(3, 1);
%                 for n = k:order
%                     factor = prod(n - k + 1 : n);
%                     val_k = val_k + C(:, n + 1) * factor * (tau^(n - k));
%                 end
%                 xd(4*k + (1:3)) = val_k;
%             end
%         end
%     end
% end

% classdef REPLANNING_7TH_BSPLINE_REFERENCE < handle
%     % 数値正規化・段差ゼロ 13次多項式QPリプランナ
%     properties
%         base_ref
%         self
%         replan_active = false
%         replan_done   = false
% 
%         t_start
%         t_duration = 10.0          % 回避・通過時間 [s]
% 
%         obs_center = [0.2; 0.2; 10.0]
%         obs_radius = 0.3
%         safe_margin = 0.5
%         trigger_dist = 3.5
% 
%         L_cable = 2.0
%         poly_coeffs                % 3 x 14 係数行列 (無次元時間 u in [0,1])
% 
%         % 合流用状態
%         t_merged_actual
%         p_merged_init
%         v_nominal_dir
%         v_nominal_speed = 0.3
% 
%         result
%     end
% 
%     methods
%         function obj = REPLANNING_7TH_BSPLINE_REFERENCE(self, base_ref, opts)
%             arguments
%                 self
%                 base_ref
%                 opts = struct()
%             end
%             obj.self = self;
%             obj.base_ref = base_ref;
% 
%             if isfield(opts, "obs_center"),   obj.obs_center   = opts.obs_center;   end
%             if isfield(opts, "obs_radius"),   obj.obs_radius   = opts.obs_radius;   end
%             if isfield(opts, "safe_margin"),  obj.safe_margin  = opts.safe_margin;  end
%             if isfield(opts, "trigger_dist"), obj.trigger_dist = opts.trigger_dist; end
%             if isfield(opts, "t_duration"),   obj.t_duration   = opts.t_duration;   end
% 
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
%             % 1. 接近検知トリガー
%             if cha == 'f' && ~obj.replan_done && ~obj.replan_active
%                 dist_load_obs  = norm(pL_cur - obj.obs_center);
%                 dist_drone_obs = norm(pQ_cur - obj.obs_center);
%                 min_dist = min(dist_load_obs, dist_drone_obs);
% 
%                 if min_dist <= obj.trigger_dist
%                     fprintf("\n=======================================================\n");
%                     fprintf("[REPLAN] 障害物接近検知! (min_dist=%.2fm, t=%.3f s)\n", min_dist, time.t);
%                     obj.t_start = time.t;
% 
%                     spd = norm(vL_cur);
%                     if spd < 0.1, spd = 0.3; end
%                     obj.v_nominal_speed = spd;
% 
%                     % 障害物通過までの所要時間を算出 (上限を12秒に制限して無駄な伸長を防ぐ)
%                     dist_to_pass = (obj.obs_center(3) + obj.obs_radius + obj.L_cable + 1.2) - pL_cur(3);
%                     obj.t_duration = max(7.0, min(12.0, dist_to_pass / spd));
% 
%                     fprintf("[REPLAN] 安全回避計画時間 T = %.2f 秒\n", obj.t_duration);
% 
%                     obj.plan_exact_avoidance(time.t, xd_nominal);
%                     obj.replan_active = true;
%                     fprintf("=======================================================\n\n");
%                 end
%             end
% 
%             % 2. 軌道出力 ＆ 合流
%             if obj.replan_active
%                 tau = time.t - obj.t_start;
%                 obs_top_z = obj.obs_center(3) + obj.obs_radius + obj.safe_margin;
%                 is_passed = (pL_cur(3) >= obs_top_z) && (tau >= obj.t_duration * 0.85);
% 
%                 if tau < obj.t_duration && ~is_passed
%                     xd = obj.evaluate_poly_xd(tau, xd_nominal);
%                 else
%                     if ~obj.replan_done
%                         fprintf("[REPLAN] 障害物通過完了・段差ゼロで継続上昇へ移行 (t=%.3f s)\n\n", time.t);
%                         obj.t_merged_actual = time.t;
% 
%                         xd_prev = obj.evaluate_poly_xd(min(tau, obj.t_duration), xd_nominal);
%                         obj.p_merged_init = xd_prev(1:3);
% 
%                         if norm(xd_nominal(5:7)) > 0.01
%                             obj.v_nominal_dir = xd_nominal(5:7) / norm(xd_nominal(5:7));
%                         else
%                             obj.v_nominal_dir = [0; 0; 1];
%                         end
%                         obj.replan_active = false;
%                         obj.replan_done   = true;
%                     end
% 
%                     dt_after = time.t - obj.t_merged_actual;
%                     xd = zeros(28, 1);
% 
%                     decay = exp(-dt_after / 2.0);
%                     p_xy_decay = [obj.p_merged_init(1) * decay; obj.p_merged_init(2) * decay];
%                     p_z_linear = obj.p_merged_init(3) + obj.v_nominal_speed * dt_after;
% 
%                     xd(1:3) = [p_xy_decay(1); p_xy_decay(2); p_z_linear];
%                     xd(5:7) = obj.v_nominal_speed * obj.v_nominal_dir;
%                     xd(9:28) = 0;
%                 end
%             elseif obj.replan_done
%                 dt_after = time.t - obj.t_merged_actual;
%                 xd = zeros(28, 1);
%                 decay = exp(-dt_after / 2.0);
%                 p_xy_decay = [obj.p_merged_init(1) * decay; obj.p_merged_init(2) * decay];
%                 p_z_linear = obj.p_merged_init(3) + obj.v_nominal_speed * dt_after;
% 
%                 xd(1:3) = [p_xy_decay(1); p_xy_decay(2); p_z_linear];
%                 xd(5:7) = obj.v_nominal_speed * obj.v_nominal_dir;
%                 xd(9:28) = 0;
%             else
%                 xd = xd_nominal;
%             end
% 
%             if length(xd) < 28
%                 xd = [xd; zeros(28 - length(xd), 1)];
%             end
% 
%             obj.result.state.xd = xd;
%             obj.result.state.p = xd(1:3);
%             obj.result.state.v = xd(5:7);
%             obj.result.state.q = [0; 0; xd(4)];
%             result = obj.result;
%         end
%     end
% 
%     methods (Access = private)
%         function plan_exact_avoidance(obj, t0, xd0)
%             T = obj.t_duration;
% 
%             % 経由点 (Via-point): 障害物の外側
%             R_clear = obj.obs_radius + obj.safe_margin + obj.L_cable * 0.5 + 0.3;
%             d_xy = [obj.obs_center(1); obj.obs_center(2)];
%             if norm(d_xy) < 0.05
%                 dir_avoid = [1; 0];
%             else
%                 dir_avoid = d_xy / norm(d_xy);
%             end
% 
%             p_via = zeros(3, 1);
%             p_via(1:2) = obj.obs_center(1:2) + dir_avoid * R_clear;
%             % 経由点の高さ: 開始目標位置と通過目標の中間
%             obs_top_z = obj.obs_center(3) + obj.obs_radius + obj.safe_margin;
%             z_end = obs_top_z + obj.L_cable + 1.2;
%             p_via(3) = (xd0(3) + z_end) / 2;
% 
%             % 終端状態 xd1
%             xd1 = zeros(28, 1);
%             xd1(1:3) = [p_via(1) * 0.2; p_via(2) * 0.2; z_end];
%             xd1(5:7) = [0; 0; obj.v_nominal_speed];
%             xd1(9:28) = 0;
% 
%             % -------------------------------------------------------------
%             % 正規化時間 u = tau / T in [0, 1] による 13次多項式 QP
%             % d^k/dtau^k = (1 / T^k) * d^k/du^k
%             % -------------------------------------------------------------
%             order = 13;
%             A = zeros(14, 14);
%             Bx = zeros(14, 3);
% 
%             % 1. u = 0 拘束 (直前の目標状態 xd0 と厳密一致 -> 段差ゼロ)
%             for k = 0:6
%                 row = k + 1;
%                 for n = k:order
%                     A(row, n + 1) = prod(n - k + 1 : n) * (0)^(n - k);
%                 end
%                 % 導関数スケーリング: d^k p / du^k = T^k * d^k p / dtau^k
%                 Bx(row, :) = (T^k) * xd0(4*k + (1:3))';
%             end
% 
%             % 2. u = 0.5 経由点拘束 (位置のみ)
%             u_mid = 0.5;
%             row = 8;
%             for n = 0:order
%                 A(row, n + 1) = u_mid^n;
%             end
%             Bx(row, :) = p_via';
% 
%             % 3. u = 1.0 合流拘束
%             u_end = 1.0;
%             for k = 0:5
%                 row = 9 + k;
%                 for n = k:order
%                     A(row, n + 1) = prod(n - k + 1 : n) * (u_end)^(n - k);
%                 end
%                 Bx(row, :) = (T^k) * xd1(4*k + (1:3))';
%             end
% 
%             % 連立一次方程式の求解 (正規化により条件数は常に健全)
%             C = A \ Bx;
%             obj.poly_coeffs = C'; % 3 x 14
%         end
% 
%         function xd = evaluate_poly_xd(obj, tau, xd_nom)
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
%                 % 実時間微分に戻す: d^k p / dtau^k = (1 / T^k) * d^k p / du^k
%                 xd(4*k + (1:3)) = val_k / (T^k);
%             end
%         end
%     end
% end

classdef REPLANNING_7TH_BSPLINE_REFERENCE < handle
    % 任意軌道・任意障害物完全適応型 C^6 滑らかQPリプランナ 13次多項式ユニバーサルリプランナ
    properties
        base_ref
        self
        replan_active = false
        replan_done   = false
        
        t_start
        t_duration                 % 動的に自動決定される所要時間 [s]
        
        % 障害物設定 (外から任意に指定可能)
        obs_center   = [0; 0; 5.0]  % 障害物中心 [x; y; z]
        obs_radius   = 0.3          % 障害物半径 [m]
        safe_margin  = 0.4          % 安全余裕 [m]
        trigger_dist = 3.5          % 検知距離 [m]
        
        L_cable      = 2.0          % ケーブル長
        poly_coeffs                 % 3 x 14 係数行列 (無次元時間 u in [0,1])
        
        % 合流用状態保持
        t_merge_end
        p_merge_end                 % 3x1
        v_merge_vec                 % 3x1
        
        result
    end
    
    methods
        function obj = REPLANNING_7TH_BSPLINE_REFERENCE(self, base_ref, opts)
            arguments
                self
                base_ref
                opts = struct()
            end
            obj.self = self;
            obj.base_ref = base_ref;
            
            if isfield(opts, "obs_center"),   obj.obs_center   = opts.obs_center(:);   end
            if isfield(opts, "obs_radius"),   obj.obs_radius   = opts.obs_radius;      end
            if isfield(opts, "safe_margin"),  obj.safe_margin  = opts.safe_margin;     end
            if isfield(opts, "trigger_dist"), obj.trigger_dist = opts.trigger_dist;    end
            if isfield(opts, "t_duration"),   obj.t_duration   = opts.t_duration;      end
            
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
            
            % ドローン本体の推算位置 (公称直線の傾きから上空オフセットを汎用計算)
            pQ_cur = pL_cur + [0; 0; obj.L_cable]; 
            
            % 1. 接近検知トリガー (荷物・ドローン両方の距離を全方向監視)
            if cha == 'f' && ~obj.replan_done && ~obj.replan_active
                dist_load_obs  = norm(pL_cur - obj.obs_center);
                dist_drone_obs = norm(pQ_cur - obj.obs_center);
                min_dist = min(dist_load_obs, dist_drone_obs);
                
                if min_dist <= obj.trigger_dist
                    fprintf("\n=======================================================\n");
                    fprintf("[REPLAN] 障害物接近を動的検知! (min_dist=%.2fm, t=%.3f s)\n", min_dist, time.t);
                    obj.t_start = time.t;
                    
                    % 任意方向の現在速度・進行方向を自動抽出
                    v_vec = xd_nominal(5:7);
                    spd = norm(v_vec);
                    if spd < 0.05, spd = norm(vL_cur); end
                    if spd < 0.05, spd = 0.3; v_vec = [0; 0; 0.3]; end
                    dir_nom = v_vec / spd;
                    
                    % 障害物を安全にクリアして軸に戻るまでの必要移動距離を幾何計算
                    % (荷物〜障害物ベクトルを進行方向軸へ射影)
                    vec_to_obs = obj.obs_center - pL_cur;
                    proj_dist = dot(vec_to_obs, dir_nom);
                    
                    % 障害物中心からさらに「半径 + ケーブル長 + マージン」先まで進む
                    total_travel_dist = proj_dist + obj.obs_radius + obj.L_cable + obj.safe_margin + 0.8;
                    total_travel_dist = max(total_travel_dist, 2.5); % 最低保証距離
                    
                    % 無理のない加速度で抜ける回避所要時間を決定
                    obj.t_duration = max(7.0, min(14.0, total_travel_dist / spd));
                    
                    fprintf("[REPLAN] 進行速度=%.2f m/s, 回避所要時間 T = %.2f 秒\n", spd, obj.t_duration);
                    
                    obj.plan_universal_avoidance(xd_nominal, dir_nom, spd, total_travel_dist);
                    obj.replan_active = true;
                    fprintf("=======================================================\n\n");
                end
            end
            
            % 2. 軌道出力 ＆ C^6完全連続合流
            if obj.replan_active
                tau = time.t - obj.t_start;
                
                if tau <= obj.t_duration
                    xd = obj.evaluate_poly_xd(tau, xd_nominal);
                else
                    if ~obj.replan_done
                        fprintf("[REPLAN] 完全連続合流完了・等速直進へシームレス移行 (t=%.3f s)\n\n", time.t);
                        obj.t_merge_end = obj.t_start + obj.t_duration;
                        xd_end = obj.evaluate_poly_xd(obj.t_duration, xd_nominal);
                        obj.p_merge_end = xd_end(1:3);
                        obj.v_merge_vec = xd_end(5:7);
                        obj.replan_active = false;
                        obj.replan_done   = true;
                    end
                    
                    % 合流後: 終端状態からその速度ベクトルのまま完全等速直進
                    dt_after = time.t - obj.t_merge_end;
                    xd = zeros(28, 1);
                    xd(1:3) = obj.p_merge_end + obj.v_merge_vec * dt_after;
                    xd(5:7) = obj.v_merge_vec;
                    xd(9:28) = 0; % 2階微分以降はゼロ接続
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
            
            if length(xd) < 28
                xd = [xd; zeros(28 - length(xd), 1)];
            end
            
            obj.result.state.xd = xd;
            obj.result.state.p = xd(1:3);
            obj.result.state.v = xd(5:7);
            obj.result.state.q = [0; 0; xd(4)];
            result = obj.result;
        end
    end
    
    methods (Access = private)
        function plan_universal_avoidance(obj, xd0, dir_nom, spd, total_dist)
            T = obj.t_duration;
            p0 = xd0(1:3);
            
            % 1. 進行方向に直交する「最短回避方向ベクトル（法線ベクトル）」を自動計算
            vec_line_to_obs = obj.obs_center - p0;
            proj_on_line = dot(vec_line_to_obs, dir_nom) * dir_nom;
            normal_vec = vec_line_to_obs - proj_on_line; % 軌道直線から障害物へ向かうベクトル
            
            if norm(normal_vec) < 1e-3
                % 障害物が軌道直線上にある場合、任意の直交ベクトルを選択
                if abs(dir_nom(3)) < 0.9
                    normal_vec = cross(dir_nom, [0; 0; 1]);
                else
                    normal_vec = cross(dir_nom, [1; 0; 0]);
                end
            end
            dir_normal = normal_vec / norm(normal_vec);
            
            % 障害物の反対側（あるいは外側）へ回避するクリアランス半径
            R_clear = obj.obs_radius + obj.safe_margin + obj.L_cable * 0.5 + 0.3;
            
            % 2. 経由点 (Via-point: u = 0.5): 
            % 進行方向には半分の距離を進み、法線方向には障害物を確実に外側へ迂回
            p_mid_line = p0 + dir_nom * (total_dist * 0.5);
            p_via = p_mid_line + dir_normal * R_clear;
            
            % 3. 終端点 (u = 1.0): 
            % 障害物を抜けた公称直線軸上へ完全着地
            p_end = p0 + dir_nom * total_dist;
            
            xd1 = zeros(28, 1);
            xd1(1:3) = p_end;            % 軌道軸に着地
            xd1(5:7) = dir_nom * spd;    % 進行方向の定常速度
            xd1(9:28) = 0;               % 2階〜6階微分は完全ゼロ
            
            % -------------------------------------------------------------
            % 正規化時間 u in [0, 1] による 13次多項式境界値問題
            % -------------------------------------------------------------
            order = 13;
            A = zeros(14, 14);
            Bx = zeros(14, 3);
            
            % u = 0 拘束 (0〜6階微分完全一致)
            for k = 0:6
                row = k + 1;
                for n = k:order
                    A(row, n + 1) = prod(n - k + 1 : n) * (0)^(n - k);
                end
                Bx(row, :) = (T^k) * xd0(4*k + (1:3))';
            end
            
            % u = 0.5 経由点拘束 (最大迂回位置)
            u_mid = 0.5;
            row = 8;
            for n = 0:order
                A(row, n + 1) = u_mid^n;
            end
            Bx(row, :) = p_via';
            
            % u = 1.0 合流拘束 (0〜5階微分完全一致で着地)
            u_end = 1.0;
            for k = 0:5
                row = 9 + k;
                for n = k:order
                    A(row, n + 1) = prod(n - k + 1 : n) * (u_end)^(n - k);
                end
                Bx(row, :) = (T^k) * xd1(4*k + (1:3))';
            end
            
            C = A \ Bx;
            obj.poly_coeffs = C'; % 3 x 14
        end
        
        function xd = evaluate_poly_xd(obj, tau, xd_nom)
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
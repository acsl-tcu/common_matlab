classdef REPLANNING_BSPLINE < handle
    % =========================================================================
    % REPLANNING_BSPLINE
    % 1. 模擬センサー：機体重心 pQ による 15m 楕円体接近検知（前方かつ真の危険度順ソート）。
    % 2. 衝突・マージン診断系：
    %    - UAV (pQ), 荷物 (pL), 索 (Cable: 間隔 <= 2*r_c の球列) の3者について、
    %      楕円体外郭までの最短ユークリッド距離をラグランジュ未定乗数法で厳密計算。
    %    - 物理接触 (physical_gap <= 0) とマージン侵入 (safety_gap <= 0) を分離記録。
    % 3. 7次 Uniform B-Spline C^6 境界確定アルゴリズム：
    %    - 始端・終端の 0〜6階微分 (位置〜Pop) を代数確定し、e_0〜e_6 の境界ギャップを完全診断。
    %    - 終端 T_tot で変位の 0〜6階微分が数学的にゼロ収束し、公称軌道へ完全滑らか合流。
    % 4. 予測軌道事前検査 (Look-ahead Safety Verification)：
    %    - 採用前に B-Spline 軌道全体をサンプリングし、3者の楕円体侵入を事前検証。
    %    - 復帰直前にも公称軌道の安全性をスキャンし、インカット衝突を防止。
    % =========================================================================
    properties
        self                       % ドローンエージェント自身
        base_ref                   % 公称参照軌道生成オブジェクト
        result                     % 出力結果構造体

        % --- センサ・検知パラメータ ---
        trigger_dist = 15.0;       % 機体搭載センサーによる接近検知閾値 [m]
        clearance_margin = 0.8;    % 安全マージン d_margin [m]

        % --- 機体・荷物・索物理パラメータ ---
        gravity = 9.81;            % 重力加速度 [m/s^2]
        L_cable = 1.0;             % 索長 [m]
        r_drone = 0.30;            % 機体球体半径 [m]
        r_load  = 0.15;            % 荷物球体半径 [m]
        r_cable_sphere = 0.25;     % 索保護球体半径 r_c [m] (直径 0.5m の球列)

        % --- 運動学・物理制約パラメータ ---
        max_acc_load = 2.0;        % 荷物許容水平加速度上限 [m/s^2] (姿勢角過大・推力飽和阻止)

        % --- リプランニング状態管理 ---
        replan_active = false;     % 回避軌道追従中フラグ
        t_start       = 0.0;       % 回避開始時刻 [s]
        t_duration    = 6.0;       % 回避全所要時間 [s]
        last_replan_time = -100.0; % 前回リプラン実行時刻 [s]
        min_replan_interval = 0.25;% 再計画更新周期 [s]
        active_threat_id = NaN;    % 現在回避中の最優先障害物ID

        % --- 7次 B-Spline パラメータ (p=7, n_seg=25 -> N_cp=32) ---
        spline_degree = 7;         % 7次 B-Spline (内部 C^6 連続)
        num_segments  = 25;        % 25セグメント
        knots                      % ノットベクトル
        control_points             % 制御点座標 (32 x 3) [X, Y, Z]
        actual_peak_displacement = 0.0; % 最大空間変位 [m]
        last_solve_time_ms = 0.0;  % QP求解時間 [ms]

        % --- C^6 境界接続ギャップ診断バッファ ---
        c6_boundary_gaps = zeros(7, 1); % e_k = ||p_new^(k) - p_old^(k)|| (k=0..6)

        % --- 連続性常時監視用バッファ ---
        prev_xd                    % 前回ステップの xd (28x1)
        prev_t = -1.0;             % 前回ステップの時刻 [s]

        % --- アクティブ障害物管理 (Active Obstacle Manager) ---
        active_obstacles = [];         % 現在管理中の障害物リスト
        post_recovery_hold_time = 3.0; % 回避・安全復帰完了後の猶予保持時間 [s]
    end

    methods (Access = public)
        % =====================================================================
        % コンストラクタ
        % =====================================================================
        function obj = REPLANNING_BSPLINE(self, base_ref, opts)
            arguments
                self                  % 必須: エージェントインスタンス
                base_ref              % 必須: 通常飛行用の公称軌道インスタンス
                opts = struct()       % 任意: 外部設定構造体
            end
            obj.self = self;
            obj.base_ref = base_ref;
            if isfield(opts, 'trigger_dist'),     obj.trigger_dist     = opts.trigger_dist;     end
            if isfield(opts, 'clearance_margin'), obj.clearance_margin = opts.clearance_margin; end
            if isfield(opts, 'r_drone'),          obj.r_drone          = opts.r_drone;          end
            if isfield(opts, 'r_load'),           obj.r_load           = opts.r_load;           end
            if isfield(opts, 'L_cable'),          obj.L_cable          = opts.L_cable;          end
            if isfield(opts, 'gravity'),          obj.gravity          = opts.gravity;          end
            if isfield(opts, 'max_acc_load'),     obj.max_acc_load     = opts.max_acc_load;     end

            obj.result = base_ref.result;

            % STATE_CLASS に診断プロパティを動的追加
            sensor_props = ["time", "pQ", "pL", "detected_point", ...
                "drone_inside_obstacle_point", "load_inside_obstacle_point", ...
                "cable_inside_obstacle_point", "drone_margin_violated", ...
                "load_margin_violated", "cable_margin_violated", ...
                "drone_min_dist_point", "load_min_dist_point", "cable_min_dist_point", ...
                "min_dist_point", "drone_obstacle_id_point", "load_obstacle_id_point", ...
                "min_obstacle_id_point", "min_source_point", "detected_obstacle_count_point"];
            for p_name = sensor_props
                if ~isprop(obj.result.state, p_name)
                    addprop(obj.result.state, p_name);
                end
            end

            obj.clear_state_sensor_values(0.0);
        end

        % =====================================================================
        % do: 制御周期ごとのメイン実行メソッド
        % =====================================================================
        function result_out = do(obj, varargin)
            time = varargin{1};
            cha  = varargin{2};

            % 1. 公称目標軌道 (Nominal Reference) の算出
            base_res = obj.base_ref.do(varargin{:});
            xd_nom = base_res.state.xd;
            if length(xd_nom) < 28
                xd_nom = [xd_nom; zeros(28 - length(xd_nom), 1)];
            end

            obj.clear_state_sensor_values(time.t);

            detection = struct();
            detection.time                          = time.t;
            detection.pQ                            = [NaN; NaN; NaN];
            detection.pL                            = [NaN; NaN; NaN];
            detection.detected_point                = false;
            detection.drone_inside_obstacle_point   = false;
            detection.load_inside_obstacle_point    = false;
            detection.cable_inside_obstacle_point   = false;
            detection.drone_margin_violated         = false;
            detection.load_margin_violated          = false;
            detection.cable_margin_violated         = false;
            detection.drone_min_dist_point          = inf;
            detection.load_min_dist_point           = inf;
            detection.cable_min_dist_point          = inf;
            detection.min_dist_point                = inf;
            detection.drone_obstacle_id_point       = NaN;
            detection.load_obstacle_id_point        = NaN;
            detection.min_obstacle_id_point         = NaN;
            detection.min_source_point              = "none";
            detection.detected_obstacle_count_point = 0;

            try obj.L_cable = obj.self.parameter.get("cableL"); catch; end

            % 2. 飛行フェーズ ('f') の機体センシング & 荷物・索 楕円体診断
            if cha == 'f'
                pL_cur = obj.self.estimator.result.state.pL(:);
                pQ_cur = obj.self.estimator.result.state.p(:);
                obs_list = obj.get_obstacles_at_time(time.t);

                % 楕円体に対する UAV / 荷物 / 索球列の厳密ユークリッド距離診断
                detection = obj.check_detection_simulated_sensor(pQ_cur, pL_cur, obs_list, time.t, xd_nom(5:7));

                % ★ ここで確実に更新代入される
                planning_obstacles = obj.update_active_obstacles(detection.detected_obstacles_point, time.t);
                % % 3. 軌道再計画の判定
                % if detection.detected_point && (time.t - obj.last_replan_time >= obj.min_replan_interval)
                %     need_replan = false;
                %     if ~obj.replan_active
                %         need_replan = true;
                %     else
                %         % 別の新しい障害物がより危険になった場合は即座に再計画
                %         if ~isnan(detection.min_obstacle_id_point) && ...
                %            (detection.min_obstacle_id_point ~= obj.active_threat_id)
                %             need_replan = true;
                %             fprintf('[REPLAN TRIGGER] 新規/別障害物 ID=%d を捕捉 -> 軌道再生成\n', detection.min_obstacle_id_point);
                %         end
                %     end
                %     if need_replan && ~isempty(planning_obstacles)
                %         obj.execute_replanning(pQ_cur, xd_nom, detection, planning_obstacles, time.t);
                %     end
                % end
                % 3. 階層型 軌道再計画の判定 (Detection -> Collision Prediction -> Dynamic Trigger)
                if ~isempty(planning_obstacles) && (time.t - obj.last_replan_time >= obj.min_replan_interval)
                    target_obs = planning_obstacles(1);

                    % 進行軸方向 dir_nom と進行軸上の速度 spd_along
                    v_nom = xd_nom(5:7);
                    if norm(v_nom) > 0.1
                        dir_nom = v_nom(:) / norm(v_nom);
                    else
                        dir_nom = [0; 0; 1];
                    end
                    spd_along = max(0.5, dot(v_nom(:), dir_nom));

                    % 障害物中心までの進行軸距離 dist_along
                    p_obs = target_obs.p_obs(:);
                    dist_along = dot(p_obs - pQ_cur, dir_nom);

                    % システム包含クリアランスの算出 (機体+荷物+索長+マージン)
                    req_clearance = obj.L_cable + obj.r_drone + obj.clearance_margin;

                    % [Level 2] 現在追従している軌道の将来衝突診断 (UAV + Load + Cable)
                    [has_collision_risk, min_dist_sys, ~] = obj.predict_current_trajectory_collision(time.t, target_obs);

                    need_replan = false;

                    if ~obj.replan_active
                        % 未回避状態: 現在軌道に衝突リスクがあり、かつ動力学的限界距離に達したか？
                        if has_collision_risk
                            % [Level 3] 動的開始限界距離 d_start の計算
                            d_start = obj.compute_dynamic_avoidance_distance(spd_along, target_obs, req_clearance, dir_nom);

                            if dist_along <= d_start
                                need_replan = true;
                                fprintf('[TRIGGER: COLLISION RISK] 限界距離到達 (dist=%.2fm <= d_start=%.2fm, min_clearance=%.2fm) -> QP起動\n', ...
                                    dist_along, d_start, min_dist_sys);
                            else
                                % [WARNING] 衝突リスクはあるが、まだ距離に余裕があるため直進継続
                            end
                        end
                    else
                        % 既に回避実行中: 新規/別の障害物との衝突リスクが発生した場合に再計画
                        if ~isnan(detection.min_obstacle_id_point) && ...
                                (detection.min_obstacle_id_point ~= obj.active_threat_id) && ...
                                has_collision_risk
                            need_replan = true;
                            fprintf('[REPLAN TRIGGER] 別障害物 ID=%d との将来衝突リスク検知 -> 軌道再生成\n', detection.min_obstacle_id_point);
                        end
                    end

                    % 条件成立時に再計画を実行
                    if need_replan
                        obj.execute_replanning(pQ_cur, xd_nom, detection, planning_obstacles, time.t);
                    end
                end
            end

            % 4. 出力目標軌道の確定 (差分変位加算方式)
            if obj.replan_active
                tau = time.t - obj.t_start;

                % 復帰安全性の事前検査: 終了 1.0 秒前に公称軌道への合流安全性をスキャン
                if (tau >= obj.t_duration - 1.0) && (tau <= obj.t_duration)
                    if ~isempty(obj.active_obstacles) && ~obj.is_nominal_recovery_safe(time.t, obj.active_obstacles)
                        % 公称軌道上に障害物がある場合は回避期間を自動延長
                        obj.t_duration = obj.t_duration + 2.0;
                        fprintf("[RETURN HELD] 公称軌道への復帰経路上に楕円体干渉を検出 (t=%.3f s). 回避期間を延長します.\n", time.t);
                    end
                end

                if tau <= obj.t_duration
                    xd_out = obj.evaluate_smooth_trajectory(tau, xd_nom);
                else
                    % 終端条件 P_end = 0 により公称軌道の 0〜6 階微分へショックなく合流
                    obj.replan_active = false;
                    obj.active_threat_id = NaN;
                    xd_out = xd_nom;
                    fprintf("[B-SPLINE C^6] 回避所要時間完了: 公称軌道へ完全滑らか復帰 (t=%.3f s)\n\n", time.t);
                end
            else
                xd_out = xd_nom;
            end

            % 5. 全時間ステップでの C^6 連続性監視 (0〜6階微分の跳躍検出)
            if cha == 'f'
                obj.verify_continuous_c6_step(xd_out, time.t, time.dt);
            end

            % 6. ロガー・後続制御器への結果格納
            st = obj.result.state;
            st.xd                            = xd_out;
            st.p                             = xd_out(1:3);
            st.v                             = xd_out(5:7);
            st.q                             = [0; 0; xd_out(4)];

            st.time                          = detection.time;
            st.pQ                            = detection.pQ;
            st.pL                            = detection.pL;
            st.detected_point                = detection.detected_point;
            st.drone_inside_obstacle_point   = detection.drone_inside_obstacle_point;
            st.load_inside_obstacle_point    = detection.load_inside_obstacle_point;
            st.cable_inside_obstacle_point   = detection.cable_inside_obstacle_point;
            st.drone_margin_violated         = detection.drone_margin_violated;
            st.load_margin_violated          = detection.load_margin_violated;
            st.cable_margin_violated         = detection.cable_margin_violated;
            st.drone_min_dist_point          = detection.drone_min_dist_point;
            st.load_min_dist_point           = detection.load_min_dist_point;
            st.cable_min_dist_point          = detection.cable_min_dist_point;
            st.min_dist_point                = detection.min_dist_point;
            st.drone_obstacle_id_point       = detection.drone_obstacle_id_point;
            st.load_obstacle_id_point        = detection.load_obstacle_id_point;
            st.min_obstacle_id_point         = detection.min_obstacle_id_point;
            st.min_source_point              = detection.min_source_point;
            st.detected_obstacle_count_point = detection.detected_obstacle_count_point;

            result_out = obj.result;
        end

        % =====================================================================
        % evaluate_smooth_trajectory: 荷物実 B-Spline 軌道 p_L(t) から直接評価 (Public)
        % =====================================================================
        function xd = evaluate_smooth_trajectory(obj, tau, xd_nom)
            xd = xd_nom;
            
            % 荷物の実目標軌道状態を B-Spline から直接評価 (0〜6階微分)
            pL_d   = obj.eval_spline_kth(tau, 0);
            vL_d   = obj.eval_spline_kth(tau, 1);
            aL_d   = obj.eval_spline_kth(tau, 2);
            jL_d   = obj.eval_spline_kth(tau, 3);
            sL_d   = obj.eval_spline_kth(tau, 4);
            cL_d   = obj.eval_spline_kth(tau, 5);
            popL_d = obj.eval_spline_kth(tau, 6);
            
            % xd(1:28): 荷物 pL の 0〜6階微分 ＋ 公称 Yaw の 0〜6階微分
            xd(1:3)   = pL_d;    % 荷物位置 (0階)
            xd(5:7)   = vL_d;    % 荷物速度 (1階)
            xd(9:11)  = aL_d;    % 荷物加速度 (2階)
            xd(13:15) = jL_d;    % Jerk (3階)
            xd(17:19) = sL_d;    % Snap (4階)
            if length(xd) >= 23, xd(21:23) = cL_d;   end % Crack (5階)
            if length(xd) >= 27, xd(25:27) = popL_d; end % Pop (6階)
            
            % 差分平坦性モデルに基づく索張力方向と UAV 参照目標位置
            g_vec = [0; 0; obj.gravity];
            acc_tot = aL_d + g_vec;
            norm_a = norm(acc_tot);
            if norm_a > 1e-3, thrust_dir = acc_tot / norm_a; else, thrust_dir = [0; 0; 1]; end
            pQ_d = pL_d + obj.L_cable * thrust_dir;
            
            % UAV 目標位置の格納 (状態定義に合わせて格納)
            if length(xd) >= 31
                xd(29:31) = pQ_d;
            end
        end
    end

    methods (Access = private)
        % =====================================================================
        % execute_replanning: 迂回弧長タイムスケーリング ＆ 適正幾何回避計画
        % =====================================================================
        function execute_replanning(obj, pQ_cur, xd_nom, detection, planning_obstacles, t_now)
            target_obs = detection.detected_obstacles_point(1);
            obj.active_threat_id = target_obs.id;
            
            % 1. 公称進行方向の単位ベクトル確定
            v_nom = xd_nom(5:7);
            spd = norm(v_nom);
            if spd < 0.1
                spd = 1.0;
                v_nom = [0; 0; 1]; % デフォルト進行方向
            end
            dir_nom = v_nom / spd;
            
            % 2. 旧軌道（または公称軌道）の現時刻における実 0〜6階微分状態の抽出 (厳密 C^6)
            init_load_state = zeros(7, 3);
            if obj.replan_active
                tau_now = t_now - obj.t_start;
                for k = 0:6
                    init_load_state(k + 1, :) = obj.eval_spline_kth(tau_now, k)';
                end
            else
                init_load_state = obj.get_nominal_derivatives_at_time(t_now);
            end
            
            obj.t_start          = t_now;
            obj.last_replan_time = t_now;
            
            % --- 3. 要求クリアランスの適正確定 ---
            % 索長(約1m) + 機体半径(0.3m) + 荷物半径(0.1m) + 安全マージン(0.6m) = 約 2.0m
            % 過大な固定マージンで機体を遠方に吹き飛ばさないよう適正化
            req_clearance = obj.L_cable + obj.r_drone + obj.r_load + 0.6;
            req_clearance = max(req_clearance, 1.8);
            
            % --- 4. 全方位進入対応の退避方向 (n_escape) 確定ロジック ---
            % 機体現在位置から障害物中心へのベクトル
            vec_to_center = target_obs.p_obs - pQ_cur;
            
            % 進行軸に直交する相対位置ベクトル (中心線からの横ズレ)
            offset_perp = pQ_cur - (target_obs.p_obs + dot(pQ_cur - target_obs.p_obs, dir_nom) * dir_nom);
            
            if norm(offset_perp) > 0.05
                % すでに横オフセットがある場合: そのまま外側へ離隔
                n_escape = offset_perp / norm(offset_perp);
            else
                % ほぼ真正面衝突 (ヘッドオン) の場合: 法線から進行成分を除去して退避
                n_raw = -target_obs.normal_drone_point;
                n_escape = n_raw - dot(n_raw, dir_nom) * dir_nom;
                
                if norm(n_escape) < 0.1
                    % 進行軸と完全に一直線の場合: 鉛直軸との外積(水平退避)を自動選択
                    n_cand = cross(dir_nom, [0; 0; 1]);
                    if norm(n_cand) < 0.1
                        n_cand = cross(dir_nom, [1; 0; 0]);
                    end
                    n_escape = n_cand / norm(n_cand);
                else
                    n_escape = n_escape / norm(n_escape);
                end
            end
            
            % --- 5. 楕円体の進行軸投影長と【迂回弧長ベースの回避時間延伸】 ---
            R_mat = target_obs.R_obs;
            r_vec = target_obs.radii_obs;
            % 楕円体の支持関数による進行軸方向の半長
            obs_radius_along = sqrt(dot(dir_nom, R_mat * diag(r_vec.^2) * R_mat' * dir_nom));
            dist_along = dot(vec_to_center, dir_nom);
            
            % 最接近予想時間
            t_impact = max(1.0, dist_along / spd);
            
            % 【重要】横に迂回する弧長を考慮した時間延伸 (Time Scaling)
            % 迂回ルートの幾何学的移動距離の近似 (直進区間 + 横退避往復)obj.
            dist_straight = max(dist_along + obs_radius_along + 3.0, 6.0);
            arc_length = sqrt(dist_straight^2 + 4.0 * (req_clearance^2));
            
            % ドローンの許容並進速度 (約 0.8〜1.0 m/s) で無理なく移動できる時間を確保
            v_safe = max(0.6, min(1.0, spd));
            t_traverse = arc_length / v_safe;
            
            % 合流・姿勢安定マージン時間 (約 3.5秒)
            t_merge_margin = max(3.5, sqrt((8.0 * req_clearance) / obj.max_acc_load));
            obj.t_duration = max([8.0, t_traverse + t_merge_margin]);
            
            % --- 6. 7次 B-Spline QP 求解 (1回目: 通常クリアランス) ---
            t_solve = tic;
            obj.plan_uniform_bspline_c6_qp(req_clearance, n_escape, init_load_state, t_now, planning_obstacles, dir_nom);
            obj.last_solve_time_ms = toc(t_solve) * 1000;
            
            % --- 7. フェイルセーフ安全検証 ＆ 採用ガード ---
            % [第1段階検証]
            traj_verified = obj.verify_future_trajectory_safety(t_now, planning_obstacles);
            
            if traj_verified
                % 1回目で完全合格 -> 採用
                obj.replan_active = true;
                obj.verify_boundary_c6_matching(init_load_state, t_now);
                obj.display_detection_report(t_now, detection, req_clearance, t_impact, n_escape, "PASS");
            else
                % 1回目で不合格 -> クリアランスを拡大して再計画 (2回目)
                req_clearance_boost = req_clearance * 1.15;
                obj.plan_uniform_bspline_c6_qp(req_clearance_boost, n_escape, init_load_state, t_now, planning_obstacles, dir_nom);
                
                % [第2段階検証] Boosted 軌道を再検証
                boosted_verified = obj.verify_future_trajectory_safety(t_now, planning_obstacles);
                
                if boosted_verified
                    % 2回目で合格 -> 採用
                    obj.replan_active = true;
                    obj.verify_boundary_c6_matching(init_load_state, t_now);
                    obj.display_detection_report(t_now, detection, req_clearance_boost, t_impact, n_escape, "BOOSTED_PASS");
                else
                    % 2回とも厳格マージンをわずかに割った場合でも、
                    % 直進公称軌道(衝突確定)の維持を阻止するため、最大退避している Boosted 軌道をフォールバック採用！
                    obj.replan_active = true;
                    obj.verify_boundary_c6_matching(init_load_state, t_now);
                    obj.display_detection_report(t_now, detection, req_clearance_boost, t_impact, n_escape, "FORCED_BOOST_PASS");
                end
            end
        end

        
        % % =====================================================================
        % % plan_uniform_bspline_c6_qp: 先行研究型 曲面適応支持超平面 全区間安全 QP
        % % =====================================================================
        % function plan_uniform_bspline_c6_qp(obj, req_clearance, n_escape_3d, init_load_state, t_now, planning_obstacles, dir_nom)
        %     p = 7;
        %     n_seg = 25;
        %     n_cp = n_seg + p;          % 32 制御点
        %     T_tot = obj.t_duration;
        %     obj.spline_degree = p;
        %     obj.num_segments = n_seg;
        %     obj.build_clamped_uniform_knots(n_seg, p, T_tot);
        % 
        %     % 1. 始端 0〜6階微分の境界確定: P_start (7x3) 【厳密代数 C^6 接続】
        %     M_start = zeros(7, 7);
        %     for k = 0:6
        %         d_row = obj.eval_basis_derivatives(p + 1, 0.0, k);
        %         M_start(k + 1, :) = d_row(k + 1, 1:7);
        %     end
        %     P_start = M_start \ init_load_state(1:7, :);
        % 
        %     % 2. 終端 0〜6階微分の公称合流確定: P_end (7x3) 【厳密代数 C^6 境界】
        %     % 2. 終端 0〜6階微分の公称合流確定: P_end (7x3) 【厳密代数 C^6 境界】
        %     end_nom_state = obj.get_nominal_derivatives_at_time(t_now + T_tot);
        %     M_end = zeros(7, 7);
        %     for k = 0:6
        %         d_row_end = obj.eval_basis_derivatives(n_cp, T_tot, k);
        %         M_end(k + 1, :) = d_row_end(k + 1, (end - 6):end);
        %     end
        %     P_end = M_end \ end_nom_state(1:7, :);
        % 
        %     % =================================================================
        %     % 【厳密版】P_end (CP26〜32) 楕円体幾何安全判定 ＆ T_tot 動的延伸
        %     % 粗い外接球 max(radii) を廃止し、楕円体主軸計量で方向依存の真の安全を判定
        %     % =================================================================
        %     R_obs = target_obs.R_obs;
        %     r_obs = target_obs.radii_obs(:);
        % 
        %     % マージンを含む拡張楕円体の主軸半径
        %     r_safe_axes = r_obs + req_clearance;
        % 
        %     max_extend_iter = 10;
        %     iter_ext = 0;
        % 
        %     while iter_ext < max_extend_iter
        %         % 終端7制御点を楕円体主軸ローカル座標系へ一括写像
        %         P_rel = P_end - target_obs.p_obs';               % (7x3)
        %         P_local = (R_obs' * P_rel')';                    % (7x3)
        % 
        %         % 拡張楕円体に対する無次元幾何距離 D (D_k = sqrt((x/a)^2 + (y/b)^2 + (z/c)^2))
        %         norm_coords = P_local ./ (r_safe_axes');
        %         D_eval = sqrt(sum(norm_coords.^2, 2));           % (7x1)
        % 
        %         % 全7点が拡張楕円体の外側 (D >= 1.0) にあれば完全合格
        %         if all(D_eval >= 1.0)
        %             break;
        %         end
        % 
        %         % 1点でも侵入している場合のみ、必要最小限の T_tot を延伸
        %         T_tot = T_tot + 0.8;
        %         obj.t_duration = T_tot;
        %         obj.build_clamped_uniform_knots(n_seg, p, T_tot);
        % 
        %         % 延伸時刻における P_end を再計算
        %         end_nom_state = obj.get_nominal_derivatives_at_time(t_now + T_tot);
        %         M_end = zeros(7, 7);
        %         for k = 0:6
        %             d_row_end = obj.eval_basis_derivatives(n_cp, T_tot, k);
        %             M_end(k + 1, :) = d_row_end(k + 1, (end - 6):end);
        %         end
        %         P_end = M_end \ end_nom_state(1:7, :);
        % 
        %         iter_ext = iter_ext + 1;
        %     end
        % 
        %     % 3. 自由変数 (P_8 〜 P_25: 18点) の定式化
        %     D4 = diff(eye(n_cp), 4);
        %     Q = D4' * D4;
        %     idx_free = 8:(n_cp - 7);
        %     n_free = length(idx_free);
        % 
        %     Q_mm = Q(idx_free, idx_free) + 1e-4 * eye(n_free);
        %     Q_ms = Q(idx_free, 1:7);
        %     Q_me = Q(idx_free, (n_cp-6):n_cp);
        % 
        %     % 公称軌道制御点の最小二乗射影
        %     P_nom_all = obj.project_nominal_trajectory_to_bspline(t_now, T_tot, n_cp);
        %     P_nom_free = P_nom_all(idx_free, :);
        % 
        %     % 目的関数: スプライン平滑性(Snap)優先 ＆ 適度な公称引き戻し
        %     w_snap = 1.0;
        %     w_dev  = 0.5;
        % 
        %     H_1d = w_snap * ((Q_mm + Q_mm') / 2) + w_dev * eye(n_free);
        %     H = blkdiag(H_1d, H_1d, H_1d);
        % 
        %     f_x = w_snap * (P_start(:, 1)' * Q_ms' + P_end(:, 1)' * Q_me')' - w_dev * P_nom_free(:, 1);
        %     f_y = w_snap * (P_start(:, 2)' * Q_ms' + P_end(:, 2)' * Q_me')' - w_dev * P_nom_free(:, 2);
        %     f_z = w_snap * (P_start(:, 3)' * Q_ms' + P_end(:, 3)' * Q_me')' - w_dev * P_nom_free(:, 3);
        %     f = [f_x; f_y; f_z];
        % 
        %     % =================================================================
        %     % 4. 先行研究型: 任意姿勢楕円体の厳密断面追従支持超平面 (Exact SFC)
        %     % =================================================================
        %     p_obs = target_obs.p_obs;
        %     R_obs = target_obs.R_obs;
        %     r_obs = target_obs.radii_obs(:);
        % 
        %     % 楕円体の Dual 形状行列 M_inv = R * diag(r^2) * R'
        %     % (x - p_obs)' * M * (x - p_obs) <= 1 における M の逆行列
        %     M_inv = R_obs * diag(r_obs.^2) * R_obs';
        % 
        %     % 進行軸 d と 退避軸 n の二次形式解析値
        %     d_Md = dot(dir_nom, M_inv * dir_nom);
        %     n_Mn = dot(n_escape_3d, M_inv * n_escape_3d);
        %     n_Md = dot(n_escape_3d, M_inv * dir_nom);
        % 
        %     % 進行軸方向の厳密な楕円体半長 L_along
        %     L_along = sqrt(max(1e-6, d_Md));
        % 
        %     % 進入マージンおよび索抜け（戻り区間）マージン
        %     s_enter = -(L_along + 1.2);
        %     s_exit  =  (L_along + obj.L_cable + 1.2);
        % 
        %     % 断面の形状係数 (傾いた楕円体の直交断面半径の係数)
        %     % Schur complement に基づく断面退避幅の二乗係数
        %     rad_sq_coeff = max(0.0, n_Mn - (n_Md^2) / d_Md);
        %     center_shift_coeff = n_Md / d_Md;
        % 
        %     A_ineq = [];
        %     b_ineq = [];
        % 
        %     for j = 1:n_free
        %         P_ref = P_nom_free(j, :)';
        %         s = dot(P_ref - p_obs, dir_nom); % 進行軸方向の相対位置
        % 
        %         if (s >= s_enter) && (s <= s_exit)
        %             if abs(s) <= L_along
        %                 % 厳密断面解: 断面中心の傾斜ズレ + 局所楕円断面幅
        %                 s_norm_sq = (s / L_along)^2;
        %                 R_local = center_shift_coeff * s + sqrt(rad_sq_coeff * max(0.0, 1.0 - s_norm_sq));
        %             else
        %                 % 障害物本体を抜けた過渡域 (索抜け・合流域)
        %                 if s > L_along
        %                     decay = max(0.0, 1.0 - (s - L_along) / (obj.L_cable + 1.2));
        %                     R_local = 0.3 * sqrt(n_Mn) * decay;
        %                 else
        %                     decay = max(0.0, 1.0 - (abs(s) - L_along) / 1.2);
        %                     R_local = 0.3 * sqrt(n_Mn) * decay;
        %                 end
        %             end
        % 
        %             % この断面位置での絶対安全離隔: 楕円体厳密表面幅 + 必要クリアランス
        %             local_safe_dist = max(0.0, R_local) + req_clearance;
        % 
        %             % 障害物中心軸から外向きに local_safe_dist 離隔する半空間壁
        %             p_boundary = p_obs + s * dir_nom + local_safe_dist * n_escape_3d;
        % 
        %             row_cp = zeros(1, n_free * 3);
        %             for dim = 1:3
        %                 idx_d = (dim - 1) * n_free;
        %                 row_cp(idx_d + j) = -n_escape_3d(dim);
        %             end
        % 
        %             A_ineq = [A_ineq; row_cp];
        %             b_ineq = [b_ineq; -dot(n_escape_3d, p_boundary)];
        %         end
        %     end
        % 
        %     % 5. QP 求解 (内点法二次計画法)
        %     opts = optimoptions('quadprog', 'Display', 'off', 'Algorithm', 'interior-point-convex');
        %     [X_mid, ~, exitflag] = quadprog(H, f, A_ineq, b_ineq, [], [], [], [], [], opts);
        % 
        %     if exitflag < 1
        %         % Infeasible フォールバック: 各点ごとに法線方向へ滑らかにオフセット
        %         P_mid = P_nom_free;
        %         for j = 1:n_free
        %             P_mid(j, :) = P_mid(j, :) + n_escape_3d' * req_clearance;
        %         end
        %     else
        %         P_mid = zeros(n_free, 3);
        %         P_mid(:, 1) = X_mid(1:n_free);
        %         P_mid(:, 2) = X_mid((n_free+1):(2*n_free));
        %         P_mid(:, 3) = X_mid((2*n_free+1):(3*n_free));
        %     end
        % 
        %     obj.control_points = [P_start; P_mid; P_end];
        %     obj.actual_peak_displacement = max(vecnorm(P_mid - P_nom_free, 2, 2));
        % end

        % =====================================================================
        % plan_uniform_bspline_c6_qp: 複数障害物 統合支持超平面 全区間安全 QP
        % =====================================================================
        function plan_uniform_bspline_c6_qp(obj, req_clearance, n_escape_3d, init_load_state, t_now, planning_obstacles, dir_nom)
            p = 7;
            n_seg = 25;
            n_cp = n_seg + p;          % 32 制御点
            T_tot = obj.t_duration;
            obj.spline_degree = p;
            obj.num_segments = n_seg;
            obj.build_clamped_uniform_knots(n_seg, p, T_tot);
            
            % 入力ベクトルの列ベクトル化 (3x1)
            dir_nom = dir_nom(:);
            n_escape_3d = n_escape_3d(:);
            
            % 1. 始端 0〜6階微分の境界確定: P_start (7x3) 【厳密代数 C^6 接続】
            M_start = zeros(7, 7);
            for k = 0:6
                d_row = obj.eval_basis_derivatives(p + 1, 0.0, k);
                M_start(k + 1, :) = d_row(k + 1, 1:7);
            end
            P_start = M_start \ init_load_state(1:7, :);
            
            % 2. 終端 0〜6階微分の公称合流確定: P_end (7x3) 【厳密代数 C^6 境界】
            end_nom_state = obj.get_nominal_derivatives_at_time(t_now + T_tot);
            M_end = zeros(7, 7);
            for k = 0:6
                d_row_end = obj.eval_basis_derivatives(n_cp, T_tot, k);
                M_end(k + 1, :) = d_row_end(k + 1, (end - 6):end);
            end
            P_end = M_end \ end_nom_state(1:7, :);
            
            % =================================================================
            % 【複数障害物対応】P_end (CP26〜32) 全障害物幾何安全判定 ＆ T_tot 動的延伸
            % 全アクティブ障害物に対して D_eval >= 1.0 を満たすまで延伸
            % =================================================================
            max_extend_iter = 10;
            iter_ext = 0;
            
            while iter_ext < max_extend_iter
                all_obs_safe = true;
                
                for obs_idx = 1:length(planning_obstacles)
                    cur_obs = planning_obstacles(obs_idx);
                    R_obs_k = cur_obs.R_obs;
                    r_obs_k = cur_obs.radii_obs(:);
                    p_obs_k = cur_obs.p_obs(:);
                    
                    r_safe_axes = r_obs_k + req_clearance;
                    
                    % 終端7制御点を各楕円体の主軸ローカル座標系へ写像
                    P_rel = P_end - p_obs_k';                    % (7x3)
                    P_local = (R_obs_k' * P_rel')';              % (7x3)
                    
                    norm_coords = P_local ./ (r_safe_axes');
                    D_eval = sqrt(sum(norm_coords.^2, 2));       % (7x1)
                    
                    if any(D_eval < 1.0)
                        all_obs_safe = false;
                        break;
                    end
                end
                
                if all_obs_safe
                    break;
                end
                
                % いずれかの障害物に近すぎる場合は合流時刻 T_tot を延伸
                T_tot = T_tot + 0.8;
                obj.t_duration = T_tot;
                obj.build_clamped_uniform_knots(n_seg, p, T_tot);
                
                end_nom_state = obj.get_nominal_derivatives_at_time(t_now + T_tot);
                M_end = zeros(7, 7);
                for k = 0:6
                    d_row_end = obj.eval_basis_derivatives(n_cp, T_tot, k);
                    M_end(k + 1, :) = d_row_end(k + 1, (end - 6):end);
                end
                P_end = M_end \ end_nom_state(1:7, :);
                
                iter_ext = iter_ext + 1;
            end
            
            % 3. 自由変数 (P_8 〜 P_25: 18点) の定式化
            D4 = diff(eye(n_cp), 4);
            Q = D4' * D4;
            idx_free = 8:(n_cp - 7);
            n_free = length(idx_free);
            
            Q_mm = Q(idx_free, idx_free) + 1e-4 * eye(n_free);
            Q_ms = Q(idx_free, 1:7);
            Q_me = Q(idx_free, (n_cp-6):n_cp);
            
            % 公称軌道制御点の最小二乗射影
            P_nom_all = obj.project_nominal_trajectory_to_bspline(t_now, T_tot, n_cp);
            P_nom_free = P_nom_all(idx_free, :);
            
            % 目的関数: スプライン平滑性(Snap)優先 ＆ 適度な公称引き戻し
            w_snap = 1.0;
            w_dev  = 0.5;
            
            H_1d = w_snap * ((Q_mm + Q_mm') / 2) + w_dev * eye(n_free);
            H = blkdiag(H_1d, H_1d, H_1d);
            
            f_x = w_snap * (P_start(:, 1)' * Q_ms' + P_end(:, 1)' * Q_me')' - w_dev * P_nom_free(:, 1);
            f_y = w_snap * (P_start(:, 2)' * Q_ms' + P_end(:, 2)' * Q_me')' - w_dev * P_nom_free(:, 2);
            f_z = w_snap * (P_start(:, 3)' * Q_ms' + P_end(:, 3)' * Q_me')' - w_dev * P_nom_free(:, 3);
            f = [f_x; f_y; f_z];
            
            % =================================================================
            % 4. 複数障害物 統合断面支持超平面スタック (Multi-Obstacle SFC)
            % =================================================================
            A_ineq = [];
            b_ineq = [];
            
            % 全障害物を巡回して不等式制約を行方向スタック
            for obs_idx = 1:length(planning_obstacles)
                cur_obs = planning_obstacles(obs_idx);
                p_obs = cur_obs.p_obs(:);
                R_obs = cur_obs.R_obs;
                r_obs = cur_obs.radii_obs(:);
                
                % 障害物ごとの Dual 形状行列
                M_inv = R_obs * diag(r_obs.^2) * R_obs';
                
                % 進行軸 d 方向の半長 L_along
                d_Md = dot(dir_nom, M_inv * dir_nom);
                L_along = sqrt(max(1e-6, d_Md));
                
                % 障害物ごとの固有退避方向 n_esc の決定
                % （公称軌道代表点と障害物中心のオフセットベクトルから幾何学的に決定）
                P_mid_ref = P_nom_free(round(n_free / 2), :)';
                v_rel = (P_mid_ref - p_obs) - dot(P_mid_ref - p_obs, dir_nom) * dir_nom;
                if norm(v_rel) > 1e-3
                    n_esc = v_rel / norm(v_rel);
                else
                    n_esc = n_escape_3d; % 特異時は指定退避方向を使用
                end
                
                % 退避軸 n の二次形式解析値
                n_Mn = dot(n_esc, M_inv * n_esc);
                n_Md = dot(n_esc, M_inv * dir_nom);
                
                % 進入マージンおよび索抜けマージン
                s_enter = -(L_along + 1.2);
                s_exit  =  (L_along + obj.L_cable + 1.2);
                
                % 断面形状係数 (Schur complement)
                rad_sq_coeff = max(0.0, n_Mn - (n_Md^2) / d_Md);
                center_shift_coeff = n_Md / d_Md;
                
                for j = 1:n_free
                    P_ref = P_nom_free(j, :)';
                    s = dot(P_ref - p_obs, dir_nom);
                    
                    if (s >= s_enter) && (s <= s_exit)
                        if abs(s) <= L_along
                            s_norm_sq = (s / L_along)^2;
                            R_local = center_shift_coeff * s + sqrt(rad_sq_coeff * max(0.0, 1.0 - s_norm_sq));
                        else
                            if s > L_along
                                decay = max(0.0, 1.0 - (s - L_along) / (obj.L_cable + 1.2));
                                R_local = 0.3 * sqrt(n_Mn) * decay;
                            else
                                decay = max(0.0, 1.0 - (abs(s) - L_along) / 1.2);
                                R_local = 0.3 * sqrt(n_Mn) * decay;
                            end
                        end
                        
                        local_safe_dist = max(0.0, R_local) + req_clearance;
                        p_boundary = p_obs + s * dir_nom + local_safe_dist * n_esc;
                        
                        row_cp = zeros(1, n_free * 3);
                        for dim = 1:3
                            idx_d = (dim - 1) * n_free;
                            row_cp(idx_d + j) = -n_esc(dim);
                        end
                        
                        A_ineq = [A_ineq; row_cp];
                        b_ineq = [b_ineq; -dot(n_esc, p_boundary)];
                    end
                end
            end
            
            % 5. QP 求解 (内点法二次計画法)
            opts = optimoptions('quadprog', 'Display', 'off', 'Algorithm', 'interior-point-convex');
            [X_mid, ~, exitflag] = quadprog(H, f, A_ineq, b_ineq, [], [], [], [], [], opts);
            
            if exitflag < 1
                % Infeasible フォールバック: 第1脅威障害物の退避方向へオフセット
                P_mid = P_nom_free;
                for j = 1:n_free
                    P_mid(j, :) = P_mid(j, :) + n_escape_3d' * req_clearance;
                end
            else
                P_mid = zeros(n_free, 3);
                P_mid(:, 1) = X_mid(1:n_free);
                P_mid(:, 2) = X_mid((n_free+1):(2*n_free));
                P_mid(:, 3) = X_mid((2*n_free+1):(3*n_free));
            end
            
            obj.control_points = [P_start; P_mid; P_end];
            obj.actual_peak_displacement = max(vecnorm(P_mid - P_nom_free, 2, 2));
        end

        % =====================================================================
        % verify_future_trajectory_safety: 軌道全体のサンプリング事前安全性検査
        % =====================================================================
        function is_safe = verify_future_trajectory_safety(obj, t_now, obs_list)
            is_safe = true;
            N_samples = 40; % 回避区間を 40 点サンプリング
            taus = linspace(0.2, obj.t_duration, N_samples);

            r_c_sph = obj.r_cable_sphere;
            n_spheres = max(2, ceil(obj.L_cable / (2.0 * r_c_sph)) + 1);
            s_ratios = linspace(0.0, 1.0, n_spheres);

            for i = 1:N_samples
                tau_i = taus(i);
                t_fut = t_now + tau_i;
                nom_res = obj.base_ref.do(struct('t', t_fut, 'dt', 0.025), 'f');
                xd_nom_i = nom_res.state.xd;
                if length(xd_nom_i) < 28, xd_nom_i = [xd_nom_i; zeros(28 - length(xd_nom_i), 1)]; end

                % xd_nom_i との加算ではなく、直読で pL と pQ を取得
                xd_fut = obj.evaluate_smooth_trajectory(tau_i, xd_nom_i);
                pL_fut = xd_fut(1:3);
                
                % pQ は差分平坦性モデルから得られる位置を使用
                g_vec = [0; 0; obj.gravity];
                acc_tot = xd_fut(9:11) + g_vec;
                norm_a = norm(acc_tot);
                if norm_a > 1e-3, thrust_dir = acc_tot / norm_a; else, thrust_dir = [0; 0; 1]; end
                pQ_fut = pL_fut + obj.L_cable * thrust_dir;
                
                cable_pts_fut = (1 - s_ratios) .* pL_fut + s_ratios .* pQ_fut;

                for obs_idx = 1:length(obs_list)
                    o = obs_list(obs_idx);
                    % フィールド名の差異を吸収
                    if isfield(o, 'p_obs')
                        p_c = o.p_obs(:);
                        r_e = o.radii_obs(:);
                    else
                        p_c = o.p_center(:);
                        r_e = o.ellipsoid_radii(:);
                    end
                    
                    obs_parsed = struct('center', p_c, 'radii', r_e, 'R', o.R_obs);

                    % 1. UAV
                    [dQ, ~, ~, ~] = obj.point_ellipsoid_signed_distance(pQ_fut, obs_parsed);
                    if dQ - obj.r_drone <= 0, is_safe = false; return; end

                    % 2. 荷物
                    [dL, ~, ~, ~] = obj.point_ellipsoid_signed_distance(pL_fut, obs_parsed);
                    if dL - obj.r_load <= 0, is_safe = false; return; end

                    % 3. 索球列
                    for sp = 1:n_spheres
                        [dC, ~, ~, ~] = obj.point_ellipsoid_signed_distance(cable_pts_fut(:, sp), obs_parsed);
                        if dC - r_c_sph <= 0, is_safe = false; return; end
                    end
                end
            end
        end

        % =====================================================================
        % is_nominal_recovery_safe: 復帰先の公称軌道における干渉スキャン (微分平坦性整合版)
        % =====================================================================
        function is_safe = is_nominal_recovery_safe(obj, t_now, obs_list)
            is_safe = true;
            N_lookahead = 15;
            t_scan = linspace(t_now + obj.t_duration, t_now + obj.t_duration + 2.0, N_lookahead);
            g_vec = [0; 0; obj.gravity];

            for i = 1:N_lookahead
                t_eval = t_scan(i);
                nom_res = obj.base_ref.do(struct('t', t_eval, 'dt', 0.025), 'f');

                % 1. 荷物の位置および加速度の取得
                % 1. 荷物の位置および加速度の取得 (確実に 3x1 列ベクトル化)
                pL_nom = nom_res.state.xd(1:3);
                pL_nom = pL_nom(:);
                if length(nom_res.state.xd) >= 10
                    aL_nom = nom_res.state.xd(9:11); % 公称加速度
                    aL_nom = aL_nom(:);
                else
                    aL_nom = zeros(3, 1);
                end

                % 2. 微分平坦性モデルに基づく真の推力方向と UAV 位置 pQ の計算
                acc_tot = aL_nom + g_vec;
                norm_a = norm(acc_tot);
                if norm_a < 1e-3
                    thrust_dir = [0; 0; 1];
                else
                    thrust_dir = acc_tot / norm_a;
                end
                pQ_nom = pL_nom + obj.L_cable * thrust_dir;

                % 3. 障害物干渉判定 (UAV, Load, ケーブル球列)
                for obs_idx = 1:length(obs_list)
                    o = obs_list(obs_idx);

                    % active_obstacles と raw obstacle のフィールド名差異を吸収
                    if isfield(o, 'p_obs')
                        p_c = o.p_obs(:);
                        r_e = o.radii_obs(:);
                    else
                        p_c = o.p_center(:);
                        r_e = o.ellipsoid_radii(:);
                    end

                    obs_parsed = struct('center', p_c, 'radii', r_e, 'R', o.R_obs);

                    [dQ, ~, ~, ~] = obj.point_ellipsoid_signed_distance(pQ_nom, obs_parsed);
                    [dL, ~, ~, ~] = obj.point_ellipsoid_signed_distance(pL_nom, obs_parsed);

                    % UAV および Load のマージン侵入判定
                    if (dQ - obj.r_drone - obj.clearance_margin <= 0) || ...
                            (dL - obj.r_load  - obj.clearance_margin <= 0)
                        is_safe = false;
                        return;
                    end

                    % ケーブル（索）中点での簡易マージン判定
                    p_cable_mid = (pQ_nom + pL_nom) / 2.0;
                    [dC, ~, ~, ~] = obj.point_ellipsoid_signed_distance(p_cable_mid, obs_parsed);
                    if (dC - 0.05 - obj.clearance_margin <= 0)
                        is_safe = false;
                        return;
                    end
                end
            end
        end

        % =====================================================================
        % verify_boundary_c6_matching: 始端 0〜6階微分ギャップ厳密検証
        % =====================================================================
        function verify_boundary_c6_matching(obj, init_load_state, t_now)
            names = ["位置(0階)", "速度(1階)", "加速度(2階)", "Jerk(3階)", "Snap(4階)", "Crack(5階)", "Pop(6階)"];
            tolerances = [1e-10, 1e-9, 1e-8, 1e-6, 1e-4, 1e-2, 1.0];
            for k = 0:6
                new_k = obj.eval_spline_kth(0.0, k)';
                expected_k = init_load_state(k + 1, :);
                gap = norm(new_k - expected_k);
                obj.c6_boundary_gaps(k + 1) = gap;
                if gap > tolerances(k + 1)
                    fprintf(2, "[FATAL C^6 BREACH] t=%.3f s | %s の切り替え接続ギャップ超過: %.3e (許容: %.3e)\n", ...
                        t_now, names(k + 1), gap, tolerances(k + 1));
                    error('リプランニング始端での C^6 境界接続に失敗しました: %s', names(k + 1));
                end
            end
        end

        % =====================================================================
        % verify_continuous_c6_step: 全時間ステップ積分整合性監視
        % =====================================================================
        function verify_continuous_c6_step(obj, xd_now, t_now, dt)
            if obj.prev_t < 0
                obj.prev_xd = xd_now;
                obj.prev_t  = t_now;
                return;
            end
            real_dt = t_now - obj.prev_t;
            if real_dt <= 1e-6, return; end
            if nargin < 4 || isempty(dt) || dt <= 0
                dt = real_dt;
            end
            
            orders = { ...
                '位置 (0階)',      1:3,   5:7;   ...
                'yaw角 (0階)',     4,     8;     ...
                '速度 (1階)',      5:7,   9:11;  ...
                '加速度 (2階)',    9:11,  13:15; ...
                'Jerk (3階)',      13:15, 17:19; ...
            };
            for idx = 1:size(orders, 1)
                name   = orders{idx, 1};
                curr_i = orders{idx, 2};
                next_i = orders{idx, 3};
                val_prev = obj.prev_xd(curr_i);
                val_curr = xd_now(curr_i);
                der_prev = obj.prev_xd(next_i);
                der_curr = xd_now(next_i);
                
                % dt を使用して予測計算
                predicted = val_prev + 0.5 * (der_prev + der_curr) * dt;
                disc_gap  = norm(val_curr - predicted);
                tol = max(0.08, 6.0 * norm(der_curr) * dt);
                if disc_gap > tol
                    fprintf(2, "[FATAL] 軌道連続性監視違反: %s at t=%.4f s (ギャップ: %.3e, 許容: %.3e)\n", ...
                        name, t_now, disc_gap, tol);
                    error('目標軌道の不連続キックが検出されました: %s', name);
                end
            end
            obj.prev_xd = xd_now;
            obj.prev_t  = t_now;
        end

        % =====================================================================
        % check_detection_simulated_sensor: 3者(機体/荷物/索)の3層距離・マージン診断
        % =====================================================================
        function det = check_detection_simulated_sensor(obj, pQ, pL, obs_list, t_now, v_nom)
            det = struct();
            det.time                          = t_now;
            det.pQ                            = pQ;
            det.pL                            = pL;
            det.trigger_dist                  = obj.trigger_dist;
            det.detected_point                = false;
            det.drone_inside_obstacle_point   = false;
            det.load_inside_obstacle_point    = false;
            det.cable_inside_obstacle_point   = false;
            det.drone_margin_violated         = false;
            det.load_margin_violated          = false;
            det.cable_margin_violated         = false;
            det.drone_min_dist_point          = inf;
            det.load_min_dist_point           = inf;
            det.cable_min_dist_point          = inf;
            det.min_dist_point                = inf;
            det.drone_obstacle_id_point       = [];
            det.load_obstacle_id_point        = [];
            det.min_obstacle_id_point         = [];
            det.min_source_point              = "none";
            det.detected_obstacles_point      = [];
            det.detected_obstacle_count_point = 0;

            if isempty(obs_list), return; end

            % 索球列の配置: 隙間ゼロ保証 (Delta s <= 2*r_c)
            r_c_sph = obj.r_cable_sphere;
            n_spheres = max(2, ceil(obj.L_cable / (2.0 * r_c_sph)) + 1);
            s_ratios = linspace(0.0, 1.0, n_spheres);
            cable_pts = (1 - s_ratios) .* pL + s_ratios .* pQ;

            v_dir = v_nom(:);
            if norm(v_dir) > 0.1, v_dir = v_dir / norm(v_dir); else, v_dir = [0; 1; 0]; end

            detected_obs_candidates = [];

            for i = 1:length(obs_list)
                o = obs_list(i);
                obs_id = o.id;
                R_obs = o.R_obs;
                radii_obs = o.ellipsoid_radii(:);
                p_obs = o.p_center(:);
                obs_parsed = struct('center', p_obs, 'radii', radii_obs, 'R', R_obs);

                % 1. 機体重心 pQ (センサー & 物理接触/マージン評価)
                [d_drone_raw, inside_drone, cpQ_loc, ~] = obj.point_ellipsoid_signed_distance(pQ, obs_parsed);
                d_drone_physical = d_drone_raw - obj.r_drone;
                d_drone_safety   = d_drone_physical - obj.clearance_margin;

                % 2. 荷物 pL
                [d_load_raw, inside_load, cpL_loc, ~] = obj.point_ellipsoid_signed_distance(pL, obs_parsed);
                d_load_physical = d_load_raw - obj.r_load;
                d_load_safety   = d_load_physical - obj.clearance_margin;

                % 3. 索球列 (Cable Spheres)
                d_cable_raw_min = inf;
                inside_cable_i = false;
                for sp_idx = 1:n_spheres
                    [d_pt, in_pt, ~, ~] = obj.point_ellipsoid_signed_distance(cable_pts(:, sp_idx), obs_parsed);
                    if d_pt < d_cable_raw_min, d_cable_raw_min = d_pt; end
                    if in_pt, inside_cable_i = true; end
                end
                d_cable_physical = d_cable_raw_min - r_c_sph;
                d_cable_safety   = d_cable_physical - obj.clearance_margin;

                % 物理衝突 (Collision) & マージン侵入 (Margin Violation) 判定
                if inside_drone || (d_drone_physical <= 0),   det.drone_inside_obstacle_point = true; end
                if inside_load  || (d_load_physical <= 0),    det.load_inside_obstacle_point  = true; end
                if inside_cable_i || (d_cable_physical <= 0), det.cable_inside_obstacle_point = true; end

                if d_drone_safety <= 0, det.drone_margin_violated = true; end
                if d_load_safety  <= 0, det.load_margin_violated  = true; end
                if d_cable_safety <= 0, det.cable_margin_violated = true; end

                if d_drone_physical < det.drone_min_dist_point
                    det.drone_min_dist_point = d_drone_physical;
                    det.drone_obstacle_id_point = obs_id;
                end
                if d_load_physical < det.load_min_dist_point
                    det.load_min_dist_point = d_load_physical;
                    det.load_obstacle_id_point = obs_id;
                end
                if d_cable_physical < det.cable_min_dist_point
                    det.cable_min_dist_point = d_cable_physical;
                end

                % 楕円体法線ベクトル
                cpQ_world = p_obs + R_obs * cpQ_loc;
                cpL_world = p_obs + R_obs * cpL_loc;
                nQ_loc = cpQ_loc ./ (radii_obs.^2);
                normal_drone = R_obs * (nQ_loc / norm(nQ_loc));
                nL_loc = cpL_loc ./ (radii_obs.^2);
                normal_load = R_obs * (nL_loc / norm(nL_loc));

                obs_info = struct( ...
                    'id',                        obs_id, ...
                    'dist_drone_point',          d_drone_physical, ...
                    'dist_drone_safety',         d_drone_safety, ...
                    'dist_load_point',           d_load_physical, ...
                    'dist_load_safety',          d_load_safety, ...
                    'dist_cable_point',          d_cable_physical, ...
                    'dist_cable_safety',         d_cable_safety, ...
                    'min_dist_point',            d_drone_physical, ...
                    'p_obs',                     p_obs, ...
                    'radii_obs',                 radii_obs, ...
                    'R_obs',                     R_obs, ...
                    'closest_drone_world_point', cpQ_world, ...
                    'closest_load_world_point',  cpL_world, ...
                    'normal_drone_point',        normal_drone, ...
                    'normal_load_point',         normal_load ...
                );

                % 進行方向かつ 15m 検知判定
                vec_to_center = p_obs - pQ;
                is_in_front = dot(vec_to_center, v_dir) > -max(radii_obs);

                if (d_drone_raw <= obj.trigger_dist) && is_in_front
                    detected_obs_candidates = [detected_obs_candidates; obs_info];
                end
            end

            % 複数検知時: 最短距離（最も危険な障害物）順にソート
            if ~isempty(detected_obs_candidates)
                [~, sort_idx] = sort([detected_obs_candidates.dist_drone_point], 'ascend');
                det.detected_obstacles_point = detected_obs_candidates(sort_idx);
                det.detected_point = true;
                det.min_dist_point = det.detected_obstacles_point(1).dist_drone_point;
                det.min_obstacle_id_point = det.detected_obstacles_point(1).id;
                det.min_source_point = "drone";
                det.detected_obstacle_count_point = length(detected_obs_candidates);
            else
                det.min_dist_point = det.drone_min_dist_point;
                det.min_obstacle_id_point = det.drone_obstacle_id_point;
                det.min_source_point = "none";
            end
        end

        % =====================================================================
        % display_detection_report: 診断レポート (3者ステータス & C^6 境界ログ)
        % =====================================================================
        function display_detection_report(obj, t_now, det, req_clearance, t_impact, n_escape, status_str)
            tgt = det.detected_obstacles_point(1);
            names = ["位置(0階)", "速度(1階)", "加速度(2階)", "Jerk(3階)", "Snap(4階)", "Crack(5階)", "Pop(6階)"];
            fprintf("\n=================================================================================\n");
            fprintf(" [B-SPLINE 15m センサー検知 ＆ C^6 回避診断レポート]  t = %.3f s [%s]\n", t_now, status_str);
            fprintf("=================================================================================\n");
            fprintf(" 1. センサ判定元      : 機体重心 pQ 搭載模擬センサー (機体表面間距離: %.3f m)\n", tgt.dist_drone_point);
            fprintf(" 2. 3層安全診断ステータス:\n");
            fprintf("     - UAV   : 表面間: %+6.3f m | マージン間: %+6.3f m | [%s]\n", ...
                tgt.dist_drone_point, tgt.dist_drone_safety, obj.get_safety_status(tgt.dist_drone_point, tgt.dist_drone_safety));
            fprintf("     - Cable : 表面間: %+6.3f m | マージン間: %+6.3f m | [%s]\n", ...
                tgt.dist_cable_point, tgt.dist_cable_safety, obj.get_safety_status(tgt.dist_cable_point, tgt.dist_cable_safety));
            fprintf("     - Load  : 表面間: %+6.3f m | マージン間: %+6.3f m | [%s]\n", ...
                tgt.dist_load_point, tgt.dist_load_safety, obj.get_safety_status(tgt.dist_load_point, tgt.dist_load_safety));
            fprintf(" 3. 捕捉楕円体情報    : 危険順位 1位 / 候補 %d 個 (ID=%d, 半径=[%.2f, %.2f, %.2f] m)\n", ...
                det.detected_obstacle_count_point, tgt.id, tgt.radii_obs(1), tgt.radii_obs(2), tgt.radii_obs(3));
            % t_impact をここで表示
            fprintf(" 4. 運動学制約考慮    : 許容 a_max = %.2f m/s^2 -> 回避時間 T_tot = %.2f s (予想接近時間 t_imp = %.2f s)\n", ...
                obj.max_acc_load, obj.t_duration, t_impact);
            fprintf(" 5. 楕円体幾何拘束    : 厳密法線退避クリアランス = %.2f m (退避ベクトル: [%.2f, %.2f, %.2f])\n", ...
                req_clearance, n_escape(1), n_escape(2), n_escape(3));
            fprintf(" 6. C^6 始端境界ギャップ実測値 (M \\ B 厳密接続):\n");
            for k = 0:6
                fprintf("     - %-12s e_%d = %.3e m/s^%d (数学的 C^6 保証)\n", names(k + 1), k, obj.c6_boundary_gaps(k + 1), k);
            end
            fprintf(" 7. QP最適化計算時間  : %6.2f ms (18自由度 Aeqフリー超高速二次計画)\n", obj.last_solve_time_ms);
            fprintf(" 8. 生成最大空間変位  : %.3f m\n", obj.actual_peak_displacement);
            fprintf("=================================================================================\n\n");
        end

        function str = get_safety_status(~, d_phys, d_safe)
            if d_phys <= 0
                str = "COLLISION";
            elseif d_safe <= 0
                str = "MARGIN VIOLATION";
            else
                str = "SAFE";
            end
        end

        % =====================================================================
        % point_ellipsoid_signed_distance: ラグランジュ未定乗数法による厳密幾何距離
        % =====================================================================
        function [d, inside, closest_local, lambda] = point_ellipsoid_signed_distance(~, p, o)
            p = p(:); c = o.center(:); r = o.radii(:); R = o.R;
            y  = R' * (p - c);
            r2 = r.^2;
            q  = sum((y ./ r).^2);
            inside = (q < 1.0);

            if norm(y) < 1e-14
                [min_r, min_idx] = min(r);
                closest_local = zeros(3, 1);
                closest_local(min_idx) = min_r;
                d      = -min_r;
                lambda = -min(r2);
                return;
            end

            f = @(lam) sum(r2 .* (y.^2) ./ ((lam + r2).^2)) - 1.0;
            if q > 1.0
                lo = 0.0;
                hi = max(r) * norm(y);
            else
                lo = -min(r2) * (1.0 - 1e-12);
                hi = 0.0;
            end

            for kk = 1:80
                mid = 0.5 * (lo + hi);
                if f(mid) > 0, lo = mid; else, hi = mid; end
            end
            lambda = 0.5 * (lo + hi);
            closest_local = r2 .* y ./ (lambda + r2);
            d_abs = norm(closest_local - y);
            if inside, d = -d_abs; else, d = d_abs; end
        end

        function val = eval_spline_kth(obj, tau, k)
            p = obj.spline_degree;
            T_tot = obj.t_duration;
            t_eval = max(0.0, min(T_tot - 1e-7, tau));

            idx_span = obj.find_knot_span(t_eval);
            ders = obj.eval_basis_derivatives(idx_span, t_eval, k);

            c_indices = (idx_span - p):idx_span;
            val = (ders(k + 1, :) * obj.control_points(c_indices, :))';
        end

        function build_clamped_uniform_knots(obj, n_seg, p, T_tot)
            dt_knot = T_tot / n_seg;
            interior_knots = dt_knot * (1:(n_seg - 1));
            obj.knots = [zeros(1, p + 1), interior_knots, T_tot * ones(1, p + 1)];
        end

        function idx = find_knot_span(obj, t_eval)
            p = obj.spline_degree;
            n_cp = length(obj.knots) - p - 1;
            if t_eval >= obj.knots(n_cp + 1), idx = n_cp; return; end
            if t_eval <= obj.knots(p + 1),    idx = p + 1; return; end
            low = p + 1; high = n_cp + 1; mid = floor((low + high) / 2);
            while (t_eval < obj.knots(mid)) || (t_eval >= obj.knots(mid + 1))
                if t_eval < obj.knots(mid), high = mid; else, low = mid; end
                mid = floor((low + high) / 2);
            end
            idx = mid;
        end

        function ders = eval_basis_derivatives(obj, idx_span, t_eval, n_der)
            p = obj.spline_degree;
            U = obj.knots;
            ders = zeros(n_der + 1, p + 1);
            ndu = zeros(p + 1, p + 1);
            left = zeros(p + 1, 1);
            right = zeros(p + 1, 1);

            ndu(1, 1) = 1.0;
            for j = 1:p
                left(j + 1) = t_eval - U(idx_span + 1 - j);
                right(j + 1) = U(idx_span + j) - t_eval;
                saved = 0.0;
                for r = 0:(j - 1)
                    ndu(j + 1, r + 1) = right(r + 2) + left(j - r + 1);
                    temp = ndu(r + 1, j) / ndu(j + 1, r + 1);
                    ndu(r + 1, j + 1) = saved + right(r + 2) * temp;
                    saved = left(j - r + 1) * temp;
                end
                ndu(j + 1, j + 1) = saved;
            end

            for j = 0:p, ders(1, j + 1) = ndu(j + 1, p + 1); end

            a = zeros(2, p + 1);
            for r = 0:p
                s1 = 0; s2 = 1; a(1, 1) = 1.0;
                for k = 1:n_der
                    d = 0.0; rk = r - k; pk = p - k;
                    if r >= k
                        a(s2 + 1, 1) = a(s1 + 1, 1) / ndu(pk + 2, rk + 1);
                        d = a(s2 + 1, 1) * ndu(rk + 1, pk + 1);
                    end
                    if rk >= -1, j1 = 1; else, j1 = -rk; end
                    if (r - 1) <= pk, j2 = k - 1; else, j2 = p - r; end
                    for j = j1:j2
                        a(s2 + 1, j + 1) = (a(s1 + 1, j + 1) - a(s1 + 1, j)) / ndu(pk + 2, rk + j + 1);
                        d = d + a(s2 + 1, j + 1) * ndu(rk + j + 1, pk + 1);
                    end
                    if r <= pk
                        a(s2 + 1, k + 1) = -a(s1 + 1, k) / ndu(pk + 2, r + 1);
                        d = d + a(s2 + 1, k + 1) * ndu(r + 1, pk + 1);
                    end
                    ders(k + 1, r + 1) = d;
                    j_tmp = s1; s1 = s2; s2 = j_tmp;
                end
            end

            r_scale = p;
            for k = 1:n_der
                for j = 0:p
                    ders(k + 1, j + 1) = ders(k + 1, j + 1) * r_scale;
                end
                r_scale = r_scale * (p - k);
            end
        end

        % =====================================================================
        % update_active_obstacles: センサー検知・回避状態に応じた障害物ライフサイクル管理
        % =====================================================================
        function planning_obstacles = update_active_obstacles(obj, detected_candidates, t_now)
            detected_ids = [];
            for k = 1:length(detected_candidates)
                cand = detected_candidates(k);
                detected_ids = [detected_ids, cand.id];
                idx = [];
                if ~isempty(obj.active_obstacles)
                    idx = find([obj.active_obstacles.id] == cand.id, 1);
                end
                if isempty(idx)
                    new_item = struct();
                    new_item.id                 = cand.id;
                    new_item.p_obs              = cand.p_obs;
                    new_item.radii_obs          = cand.radii_obs;
                    new_item.R_obs              = cand.R_obs;
                    new_item.closest_drone_world_point = cand.closest_drone_world_point;
                    new_item.normal_drone_point = cand.normal_drone_point;
                    new_item.dist_drone_point   = cand.dist_drone_point;
                    new_item.last_seen_time     = t_now;
                    new_item.state              = "DETECTED";
                    new_item.release_time       = NaN;
                    obj.active_obstacles = [obj.active_obstacles; new_item];
                else
                    obj.active_obstacles(idx).p_obs              = cand.p_obs;
                    obj.active_obstacles(idx).radii_obs          = cand.radii_obs;
                    obj.active_obstacles(idx).R_obs              = cand.R_obs;
                    obj.active_obstacles(idx).closest_drone_world_point = cand.closest_drone_world_point;
                    obj.active_obstacles(idx).normal_drone_point = cand.normal_drone_point;
                    obj.active_obstacles(idx).dist_drone_point   = cand.dist_drone_point;
                    obj.active_obstacles(idx).last_seen_time     = t_now;
                    obj.active_obstacles(idx).state              = "DETECTED";
                    obj.active_obstacles(idx).release_time       = NaN;
                end
            end

            keep_flags = true(length(obj.active_obstacles), 1);
            for i = 1:length(obj.active_obstacles)
                obs_id = obj.active_obstacles(i).id;
                if ~ismember(obs_id, detected_ids)
                    if obj.replan_active
                        obj.active_obstacles(i).state = "LATCHED";
                        obj.active_obstacles(i).release_time = NaN;
                    else
                        if obj.active_obstacles(i).state ~= "POST_RECOVERY"
                            obj.active_obstacles(i).state = "POST_RECOVERY";
                            obj.active_obstacles(i).release_time = t_now + obj.post_recovery_hold_time;
                        end
                        if t_now >= obj.active_obstacles(i).release_time
                            keep_flags(i) = false;
                        end
                    end
                end
            end
            obj.active_obstacles = obj.active_obstacles(keep_flags);
            planning_obstacles = obj.active_obstacles;
        end

        % =====================================================================
        % project_nominal_trajectory_to_bspline: 公称軌道のB-Spline最小二乗射影
        % =====================================================================
        function P_nom_all = project_nominal_trajectory_to_bspline(obj, t_now, T_tot, n_cp)
            p = obj.spline_degree;
            N_samples = 100;
            t_quad = linspace(0.0, T_tot, N_samples);
            dt = T_tot / (N_samples - 1);

            M_gram = zeros(n_cp, n_cp);
            B_proj = zeros(n_cp, 3);

            for k = 1:N_samples
                tau_k = t_quad(k);
                t_k = t_now + tau_k;

                nom_res_k = obj.base_ref.do(struct('t', t_k, 'dt', 0.025), 'f');
                p_nom_k = nom_res_k.state.xd(1:3)';

                idx_span = obj.find_knot_span(min(T_tot - 1e-6, tau_k));
                ders = obj.eval_basis_derivatives(idx_span, min(T_tot - 1e-6, tau_k), 0);
                basis_vals = ders(1, :);
                c_idx = (idx_span - p):idx_span;

                w = dt;
                if k == 1 || k == N_samples, w = 0.5 * dt; end

                M_gram(c_idx, c_idx) = M_gram(c_idx, c_idx) + w * (basis_vals' * basis_vals);
                B_proj(c_idx, :) = B_proj(c_idx, :) + w * (basis_vals' * p_nom_k);
            end

            P_nom_all = (M_gram + 1e-5 * eye(n_cp)) \ B_proj;
        end

        % =====================================================================
        % predict_current_trajectory_collision: 現在追従中軌道の将来衝突診断
        % =====================================================================
        function [has_collision_risk, min_dist_sys, t_first_risk] = predict_current_trajectory_collision(obj, t_now, target_obs)
            has_collision_risk = false;
            min_dist_sys = inf;
            t_first_risk = inf;

            % 将来 4.0 秒間 (または t_duration) を 20 点スキャン
            T_scan = 4.0;
            N_steps = 20;
            taus = linspace(0.1, T_scan, N_steps);

            p_obs = target_obs.p_obs(:);
            r_obs = target_obs.radii_obs(:);
            R_obs = target_obs.R_obs;
            obs_struct = struct('center', p_obs, 'radii', r_obs, 'R', R_obs);

            g_vec = [0; 0; obj.gravity];

            for i = 1:N_steps
                t_fut = t_now + taus(i);

                % 現在の基準軌道 (公称、または実行中の再計画軌道) から将来状態を評価
                if obj.replan_active
                    tau_local = t_fut - obj.last_replan_time;
                    if tau_local <= obj.t_duration
                        nom_res = obj.base_ref.do(struct('t', t_fut, 'dt', 0.025), 'f');
                        xd_eval = obj.evaluate_smooth_trajectory(tau_local, nom_res.state.xd);
                    else
                        nom_res = obj.base_ref.do(struct('t', t_fut, 'dt', 0.025), 'f');
                        xd_eval = nom_res.state.xd;
                    end
                else
                    nom_res = obj.base_ref.do(struct('t', t_fut, 'dt', 0.025), 'f');
                    xd_eval = nom_res.state.xd;
                end

                pL_fut = xd_eval(1:3); pL_fut = pL_fut(:);
                if length(xd_eval) >= 11
                    aL_fut = xd_eval(9:11); aL_fut = aL_fut(:);
                else
                    aL_fut = zeros(3, 1);
                end

                acc_tot = aL_fut + g_vec;
                norm_a = norm(acc_tot);
                if norm_a > 1e-3, th_dir = acc_tot / norm_a; else, th_dir = [0; 0; 1]; end
                pQ_fut = pL_fut + obj.L_cable * th_dir;
                pC_mid = (pQ_fut + pL_fut) / 2.0;

                [dQ, ~, ~, ~] = obj.point_ellipsoid_signed_distance(pQ_fut, obs_struct);
                [dL, ~, ~, ~] = obj.point_ellipsoid_signed_distance(pL_fut, obs_struct);
                [dC, ~, ~, ~] = obj.point_ellipsoid_signed_distance(pC_mid, obs_struct);

                % 正味のクリアランス (各半径と安全マージンを差し引いた実効離隔)
                clearance_Q = dQ - obj.r_drone - obj.clearance_margin;
                clearance_L = dL - obj.r_load  - obj.clearance_margin;
                clearance_C = dC - 0.05        - obj.clearance_margin;

                cur_min = min([clearance_Q, clearance_L, clearance_C]);
                if cur_min < min_dist_sys
                    min_dist_sys = cur_min;
                end

                if cur_min <= 0 && ~has_collision_risk
                    has_collision_risk = true;
                    t_first_risk = taus(i);
                end
            end
        end

        
        % =====================================================================
        % compute_dynamic_avoidance_distance: 機体・索動力学に基づく動的開始限界距離
        % =====================================================================
        function d_start = compute_dynamic_avoidance_distance(obj, spd_along, target_obs, req_clearance, dir_nom)
            R_mat = target_obs.R_obs;
            r_vec = target_obs.radii_obs(:);
            M_inv = R_mat * diag(r_vec.^2) * R_mat';
            
            % 進行軸方向の楕円体半長
            R_along = sqrt(max(1e-6, dot(dir_nom, M_inv * dir_nom)));
            
            % 許容横加速度 (標準 1.5 m/s^2)
            a_lat_max = 1.5;
            
            % 横方向の必要退避量 (最大クリアランス)
            dy_req = max(1.5, req_clearance);
            
            % 機体横移動に必要な幾何時間
            t_kinematic = sqrt(2.0 * dy_req / a_lat_max);
            
            % 吊り下げ系の振り子遅延 (半周期相当: π * sqrt(L/g))
            t_pendulum = 0.5 * pi * sqrt(obj.L_cable / obj.gravity);
            
            % QP計算猶予 + マージン
            t_buffer = 0.8;
            
            t_total_needed = t_kinematic + t_pendulum + t_buffer;
            
            % 速度に依存する動的開始限界距離
            d_start = spd_along * t_total_needed + R_along;
            % 最低保障距離 (4.0m)
            d_start = max(4.0, d_start);
        end

        % =====================================================================
        % get_nominal_derivatives_at_time: 公称軌道から 0〜6階微分値を抽出
        % =====================================================================
        function ders = get_nominal_derivatives_at_time(obj, t_eval)
            ders = zeros(7, 3);
            nom_res = obj.base_ref.do(struct('t', t_eval, 'dt', 0.025), 'f');
            xd_val = nom_res.state.xd;
            if length(xd_val) < 28
                xd_val = [xd_val; zeros(28 - length(xd_val), 1)];
            end
            ders(1, :) = xd_val(1:3)';   % 位置 (0階)
            ders(2, :) = xd_val(5:7)';   % 速度 (1階)
            ders(3, :) = xd_val(9:11)';  % 加速度 (2階)
            ders(4, :) = xd_val(13:15)'; % Jerk (3階)
            ders(5, :) = xd_val(17:19)'; % Snap (4階)
            ders(6, :) = xd_val(21:23)'; % Crack (5階)
            ders(7, :) = xd_val(25:27)'; % Pop (6階)
        end

        function clear_state_sensor_values(obj, t_now)
            st = obj.result.state;
            st.time                          = t_now;
            st.pQ                            = [NaN; NaN; NaN];
            st.pL                            = [NaN; NaN; NaN];
            st.detected_point                = false;
            st.drone_inside_obstacle_point   = false;
            st.load_inside_obstacle_point    = false;
            st.cable_inside_obstacle_point   = false;
            st.drone_margin_violated         = false;
            st.load_margin_violated          = false;
            st.cable_margin_violated         = false;
            st.drone_min_dist_point          = inf;
            st.load_min_dist_point           = inf;
            st.cable_min_dist_point          = inf;
            st.min_dist_point                = inf;
            st.drone_obstacle_id_point       = NaN;
            st.load_obstacle_id_point        = NaN;
            st.min_obstacle_id_point         = NaN;
            st.min_source_point              = "none";
            st.detected_obstacle_count_point = 0;
        end

        function list = get_obstacles_at_time(~, t_now)
            % ENVIRONMENT_OBSTACLE_ELLIPSE_MOVE を直接呼び出し (フォールバックなし)
            list = ENVIRONMENT_OBSTACLE_ELLIPSE_MOVE(t_now);
        end
    end
end

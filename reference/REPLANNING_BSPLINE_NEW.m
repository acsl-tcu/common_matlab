classdef REPLANNING_BSPLINE_NEW < handle
    % =========================================================================
    % REPLANNING_BSPLINE_NEW (純粋センサ検知器：15m 全方位球面レンジモデル)
    %
    % 【責務定義】
    %  - UAV機体重心 pQ を中心とする半径 15m 球面レンジ内に存在する障害物を検知。
    %  - 静的障害物マップを初期化時に一度だけ生成・キャッシュ。
    %  - 最上部のフラグ (1: 表示, 0: 非表示) でログ出力を一発切り替え。
    % =========================================================================
    properties
        % =====================================================================
        % ★★★ ログ表示スイッチ (1: 表示する, 0: 一切表示しない) ★★★
        % ここを 1 または 0 に書き換えるだけで切り替えできます！
        % =====================================================================
        ENABLE_ALL_LOG = 1;        % 1: ON (ログ表示), 0: OFF (非表示・最高速)
        
        % ---------------------------------------------------------------------
        % モジュール別フラグ (上記が 1 のときに個別で OFF にしたい場合に使用)
        % ---------------------------------------------------------------------
        LOG_SENSOR    = 1;         % センサ検知ログ
        LOG_RISK      = 1;         % (今後用) 危険度評価ログ
        LOG_PLANNING  = 1;         % (今後用) 回避計画・QPログ
        % =====================================================================

        self                       % ドローンエージェント自身
        base_ref                   % 公称参照軌道生成オブジェクト
        result                     % 出力結果構造体
        
        % --- センサ検知パラメータ ---
        trigger_dist = 15.0;       % センサ検知球半径 R_sense [m]
        
        % --- 静的障害物マップ (キャッシュ) ---
        static_obstacle_map = [];  % 初期化時に事前計算・保持する障害物リスト
        
        % --- 性能プロファイリング (計測バッファ) ---
        profiling = struct( ...
            't_base_ref_ms',       0.0, ... % 公称軌道生成所要時間 [ms]
            't_sensor_total_ms',   0.0, ... % センサ検知合計時間 [ms]
            't_sensor_stage1_ms',  0.0, ... % Stage 1 (外接球Reject) [ms]
            't_sensor_stage2_ms',  0.0, ... % Stage 2 (表面距離反復計算) [ms]
            't_sensor_sort_ms',    0.0, ... % ソート・パッキング [ms]
            'candidate_count',     0);      % Stage 2 に進んだ候補障害物数
    end

    methods (Access = public)
        % =====================================================================
        % コンストラクタ
        % =====================================================================
        function obj = REPLANNING_BSPLINE_NEW(self, base_ref, opts)
            arguments
                self                  % 必須: エージェントインスタンス
                base_ref              % 必須: 通常飛行用の公称軌道インスタンス
                opts = struct()       % 任意: 外部設定構造体
            end
            obj.self = self;
            obj.base_ref = base_ref;
            if isfield(opts, 'trigger_dist'), obj.trigger_dist = opts.trigger_dist; end
            if isfield(opts, 'ENABLE_ALL_LOG'), obj.ENABLE_ALL_LOG = opts.ENABLE_ALL_LOG; end

            obj.result = base_ref.result;

            % 1. 静的障害物マップの初期化・キャッシュ (t=0 で読み込み)
            raw_obs = ENVIRONMENT_OBSTACLE_ELLIPSE_MOVE(0.0);
            obj.static_obstacle_map = repmat(struct( ...
                'id', [], 'p_obs', [], 'R_obs', [], 'radii_obs', [], 'r_max', []), 0, 1);
            
            for i = 1:numel(raw_obs)
                o = raw_obs(i);
                r = o.ellipsoid_radii(:);
                item.id        = o.id;
                item.p_obs     = o.p_center(:);
                item.R_obs     = o.R_obs;
                item.radii_obs = r;
                item.r_max     = max(r); % 外接球半径を事前計算
                obj.static_obstacle_map(end+1, 1) = item;
            end

            % 2. STATE_CLASS に検知プロパティおよびプロファイリング情報を追加
            sensor_props = ["time", "pQ", "detected_point", "detected_obstacles", ...
                            "min_obstacle_id_point", "min_dist_point", "detected_obstacle_count_point", ...
                            "profiling"];
            for p_name = sensor_props
                if ~isprop(obj.result.state, p_name)
                    addprop(obj.result.state, p_name);
                end
            end
            obj.clear_state_sensor_values(0.0);
        end

        % =====================================================================
        % do: 制御周期ごとのセンシング実行 & プロファイリング
        % =====================================================================
        function result_out = do(obj, varargin)
            time = varargin{1};
            cha  = varargin{2};

            % 1. 公称目標軌道の取得 (計測)
            t_base_start = tic;
            base_res = obj.base_ref.do(varargin{:});
            xd_nom = base_res.state.xd;
            obj.profiling.t_base_ref_ms = toc(t_base_start) * 1000.0;

            obj.clear_state_sensor_values(time.t);

            detection = struct();
            detection.time                          = time.t;
            detection.pQ                            = [NaN; NaN; NaN];
            detection.detected_point                = false;
            detection.detected_obstacles            = [];
            detection.min_obstacle_id_point         = NaN;
            detection.min_dist_point                = inf;
            detection.detected_obstacle_count_point = 0;

            % 2. 飛行フェーズ ('f') の 15m 球面レンジ検知
            if cha == 'f'
                pQ_cur = obj.self.estimator.result.state.p(:);

                % センサ検知の実行 (内部ステージごとの計測を含む)
                [detection, sensor_timing] = obj.detect_obstacles_fast(pQ_cur, time.t);

                % プロファイラへ結果を登録
                obj.profiling.t_sensor_total_ms  = sensor_timing.total_ms;
                obj.profiling.t_sensor_stage1_ms = sensor_timing.stage1_ms;
                obj.profiling.t_sensor_stage2_ms = sensor_timing.stage2_ms;
                obj.profiling.t_sensor_sort_ms   = sensor_timing.sort_ms;
                obj.profiling.candidate_count    = sensor_timing.candidate_count;

                % ★★★ ログ表示スイッチ判定 (一番上のフラグが 1 の時だけ出力) ★★★
                if (obj.ENABLE_ALL_LOG == 1) && (obj.LOG_SENSOR == 1) && detection.detected_point
                    tgt = detection.detected_obstacles(1);
                    fprintf("[SENSOR DETECT] t=%.3f s | 捕捉数: %d | 最寄ID: %d | 表面距離: %6.3f m | 総計算: %.3f ms (Stage1: %.3f ms, Stage2: %.3f ms, 候補数: %d)\n", ...
                        time.t, ...
                        detection.detected_obstacle_count_point, ...
                        tgt.id, ...
                        tgt.range, ...
                        sensor_timing.total_ms, ...
                        sensor_timing.stage1_ms, ...
                        sensor_timing.stage2_ms, ...
                        sensor_timing.candidate_count);
                end
            end

            % 3. 軌道出力 (本クラスはセンサのため公称軌道をそのまま出力)
            xd_out = xd_nom;

            % 4. 状態格納 (後段の判定・計画モジュールへ引き渡す)
            st = obj.result.state;
            st.xd                            = xd_out;
            st.p                             = xd_out(1:3);
            st.v                             = xd_out(5:7);
            st.q                             = [0; 0; xd_out(4)];
            st.time                          = detection.time;
            st.pQ                            = detection.pQ;
            st.detected_point                = detection.detected_point;
            st.detected_obstacles            = detection.detected_obstacles;
            st.min_obstacle_id_point         = detection.min_obstacle_id_point;
            st.min_dist_point                = detection.min_dist_point;
            st.detected_obstacle_count_point = detection.detected_obstacle_count_point;
            st.profiling                     = obj.profiling;

            result_out = obj.result;
        end

        % =====================================================================
        % benchmark_stage2: timeit による厳密アルゴリズムベンチマーク (公開関数)
        % =====================================================================
        function [t_mean_ms, t_std_ms] = benchmark_stage2(obj, pQ_test)
            if nargin < 2 || isempty(pQ_test)
                pQ_test = [0.3; 0.3; 5.0];
            end
            if isempty(obj.static_obstacle_map)
                error('静的障害物マップが空です。');
            end

            test_obs = obj.static_obstacle_map(1);
            p = pQ_test(:);
            c = test_obs.p_obs;
            r = test_obs.radii_obs;
            R = test_obs.R_obs;

            core_fun = @() obj.point_ellipsoid_signed_distance_fast(p, c, r, R);

            % ウォームアップ
            for w = 1:50
                core_fun();
            end

            % timeit 測定
            t_sec = timeit(core_fun);
            t_mean_ms = t_sec * 1000.0;

            % サンプリング測定
            N_samples = 1000;
            t_samples_ms = zeros(N_samples, 1);
            for s = 1:N_samples
                t_s = tic;
                core_fun();
                t_samples_ms(s) = toc(t_s) * 1000.0;
            end
            t_std_ms = std(t_samples_ms);

            fprintf("\n=======================================================================\n");
            fprintf(" [STAGE 2: 楕円体表面距離計算 (40回反復) timeit ベンチマーク結果]\n");
            fprintf("=======================================================================\n");
            fprintf("  - timeit 代表実行時間 (平均): %.6f ms (約 %.2f マイクロ秒)\n", t_mean_ms, t_mean_ms * 1000.0);
            fprintf("  - 1000回単発 tic/toc 標本平均 : %.6f ms\n", mean(t_samples_ms));
            fprintf("  - 1000回単発 tic/toc 標準偏差 : %.6f ms (ジッタ幅)\n", t_std_ms);
            fprintf("  - 1000回単発 tic/toc 最小/最大: %.6f ms / %.6f ms\n", min(t_samples_ms), max(t_samples_ms));
            fprintf("=======================================================================\n\n");
        end
    end

    methods (Access = private)
        % =====================================================================
        % detect_obstacles_fast: 事前キャッシュマップに対する 15m 球面検知 (詳細計測)
        % =====================================================================
        function [det, timing] = detect_obstacles_fast(obj, pQ, t_now)
            t_total_start = tic;

            det = struct();
            det.time                          = t_now;
            det.pQ                            = pQ;
            det.detected_point                = false;
            det.detected_obstacles            = [];
            det.min_obstacle_id_point         = NaN;
            det.min_dist_point                = inf;
            det.detected_obstacle_count_point = 0;

            timing = struct('stage1_ms', 0.0, 'stage2_ms', 0.0, 'sort_ms', 0.0, 'total_ms', 0.0, 'candidate_count', 0);

            obs_map = obj.static_obstacle_map;
            if isempty(obs_map)
                timing.total_ms = toc(t_total_start) * 1000.0;
                return;
            end

            % -----------------------------------------------------------------
            % Stage 1: 外接球 Reject 判定 (全障害物に対するスクリーニング)
            % -----------------------------------------------------------------
            t_stage1_start = tic;
            passed_indices = [];
            for i = 1:numel(obs_map)
                dc = pQ - obs_map(i).p_obs;
                d2c = dot(dc, dc);
                if d2c <= (obj.trigger_dist + obs_map(i).r_max)^2
                    passed_indices(end+1) = i;
                end
            end
            timing.stage1_ms = toc(t_stage1_start) * 1000.0;
            timing.candidate_count = length(passed_indices);

            if isempty(passed_indices)
                timing.total_ms = toc(t_total_start) * 1000.0;
                return;
            end

            % -----------------------------------------------------------------
            % Stage 2: 候補に対する楕円体表面最短距離判定 (反復法)
            % -----------------------------------------------------------------
            t_stage2_start = tic;
            candidates = repmat(struct( ...
                'id', [], 'p_obs', [], 'R_obs', [], 'radii_obs', [], 'range', []), 0, 1);

            for idx = passed_indices
                o = obs_map(idx);
                d_surface = obj.point_ellipsoid_signed_distance_fast(pQ, o.p_obs, o.radii_obs, o.R_obs);

                if d_surface <= obj.trigger_dist
                    tmp.id        = o.id;
                    tmp.p_obs     = o.p_obs;
                    tmp.R_obs     = o.R_obs;
                    tmp.radii_obs = o.radii_obs;
                    tmp.range     = d_surface;
                    candidates(end+1, 1) = tmp;
                end
            end
            timing.stage2_ms = toc(t_stage2_start) * 1000.0;

            if isempty(candidates)
                timing.total_ms = toc(t_total_start) * 1000.0;
                return;
            end

            % -----------------------------------------------------------------
            % Stage 3: 最寄り順ソートと出力格納
            % -----------------------------------------------------------------
            t_sort_start = tic;
            [~, sort_idx] = sort([candidates.range], 'ascend');
            det.detected_obstacles            = candidates(sort_idx);
            det.detected_point                = true;
            det.min_dist_point                = det.detected_obstacles(1).range;
            det.min_obstacle_id_point         = det.detected_obstacles(1).id;
            det.detected_obstacle_count_point = length(candidates);
            timing.sort_ms = toc(t_sort_start) * 1000.0;

            timing.total_ms = toc(t_total_start) * 1000.0;
        end

        % =====================================================================
        % point_ellipsoid_signed_distance_fast: ラグランジュ未定乗数法 (表面距離)
        % =====================================================================
        function d = point_ellipsoid_signed_distance_fast(~, p, c, r, R)
            p = p(:);
            c = c(:);
            r = r(:);

            y  = R' * (p - c);
            r2 = r.^2;
            q  = sum((y ./ r).^2);
            inside = (q < 1.0);

            if norm(y) < 1e-14
                d = -min(r);
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

            % 固定反復数による表面距離計算
            for kk = 1:40
                mid = 0.5 * (lo + hi);
                if f(mid) > 0
                    lo = mid;
                else
                    hi = mid;
                end
            end

            lambda = 0.5 * (lo + hi);
            closest_local = r2 .* y ./ (lambda + r2);
            d_abs = norm(closest_local - y);

            if inside
                d = -d_abs;
            else
                d = d_abs;
            end
        end

        function clear_state_sensor_values(obj, t_now)
            st = obj.result.state;
            st.time                          = t_now;
            st.pQ                            = [NaN; NaN; NaN];
            st.detected_point                = false;
            st.detected_obstacles            = [];
            st.min_obstacle_id_point         = NaN;
            st.min_dist_point                = inf;
            st.detected_obstacle_count_point = 0;
        end
    end
end
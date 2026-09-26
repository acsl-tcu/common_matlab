classdef REPLANNING_BSPLINE_NEW < handle
    % =========================================================================
    % detect_obstacles_fast (純粋センサ検知器：15m 全方位球面レンジモデル)
    %
    % 【責務定義】
    %  - UAV機体重心 pQ を中心とする半径 15m 球面レンジ内に存在する障害物を検知。
    %  - 静的障害物マップを初期化時に一度だけ生成・キャッシュ。
    %  - 最上部のフラグ (1: 表示, 0: 非表示) でログ出力を一発切り替え。
    % =========================================================================
    properties
        % =====================================================================
        % ★★★ ログ表示スイッチ (1: 表示する, 0: 一切表示しない) ★★★
        % この部分の表示でコマンドウィンドウに追加のlogを表示するのか決定
        % =====================================================================
        ENABLE_ALL_LOG = 1;        % 1: ON (ログ表示), 0: OFF (非表示・最高速) 
        
        % ---------------------------------------------------------------------
        % モジュール別フラグ (上記が 1 のときに個別で OFF にしたい場合に使用)
        % ---------------------------------------------------------------------
        LOG_SENSOR    = 1;         % センサ検知ログ
        % LOG_RISK      = 1;         % (今後用) 危険度評価ログ
        % LOG_PLANNING  = 1;         % (今後用) 回避計画・QPログ
        % =====================================================================

        self                       % ドローンエージェント自身
        base_ref                   % 公称参照軌道生成オブジェクト
        result                     % 出力結果構造体
        
        % --- センサ検知パラメータ ---
        trigger_dist = 15.0;       % センサ検知球半径 R_sense [m]
        
        % --- 静的障害物マップ (キャッシュ) ---
        static_obstacle_map = [];  % 初期化時に事前計算・保持する障害物リスト 全体の障害物で実環境として取得
        
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
        % コンストラクタ：初期に一度だけ実行
        % =====================================================================
        function obj = REPLANNING_BSPLINE_NEW(self, base_ref, opts)
            arguments
                self                  % 必須: エージェントインスタンス(状態推定値やパラメータ取得元)
                base_ref              % 必須: 通常飛行用の公称軌道インスタンス
                opts = struct()       % 任意: 外部設定構造体(検知半径やログスイッチ等)
            end
            % --- 基本インスタンス・パラメータの初期設定 ---
            obj.self = self;
            obj.base_ref = base_ref;
            % 外部オプションによるパラメータ上書き　外部から渡された場合上書き
            if isfield(opts, 'trigger_dist'), obj.trigger_dist = opts.trigger_dist; end % センサ検知球半径 R_sense [m]
            if isfield(opts, 'ENABLE_ALL_LOG'), obj.ENABLE_ALL_LOG = opts.ENABLE_ALL_LOG; end % 追加ログの表示

            % 出力構造体の初期テンプレートを公称軌道オブジェクトから継承
            obj.result = base_ref.result;

            % --- 2. シミュレーション環境全体の静的障害物マップ生成・キャッシュ ---
            % 【設計意図】
            %   実世界における「物理空間そのもの（Ground Truth）」をシミュレータ上に1度だけ構築
            %   ※ 本データは「センサが検知した情報」ではなく，ワールド全体の「正解配置データ」
            %   毎制御周期 (doメソッド内) で環境関数を呼び直すオーバーヘッドを完全排除し，
            %   飛行中は本キャッシュマップに対して「15m以内に存在するか？」の照会 (Query) のみを行う
            raw_obs = ENVIRONMENT_OBSTACLE_ELLIPSE_MOVE(0.0); % 静的環境として t=0 の初期配置を取得
            % 空の構造体配列を事前定義 (フィールド構造を固定してアクセスを高速化)
            obj.static_obstacle_map = repmat(struct( ...
                'id', [], 'p_obs', [], 'R_obs', [], 'radii_obs', [], 'r_max', []), 0, 1);
            
            for i = 1:numel(raw_obs)
                o = raw_obs(i);
                r = o.ellipsoid_radii(:);
                item.id        = o.id; % 障害物固有識別ID
                item.p_obs     = o.p_center(:); % 楕円体中心座標 [x; y; z] (ワールド座標系)
                item.R_obs     = o.R_obs; % 姿勢回転行列 (ワールド -> 局所主軸座標系)
                item.radii_obs = r; % 楕円体の3主軸半長 [rx; ry; rz]
                % 【高速化事前計算】
                % 楕円体中心から最遠表面までの距離（外接球半径 r_max）をあらかじめ保持
                % これにより，オンライン実行時の Stage 1 (外接球 Reject) で max() 計算を毎回行わずに済む　
                item.r_max     = max(r); % 外接球半径を事前計算
                obj.static_obstacle_map(end+1, 1) = item;
            end

            % --- STATE_CLASS への診断・検知プロパティの動的登録 ---
            % 後続の制御器，ロガー，可視化ツールが参照できるように，
            % センサ検知結果や計算負荷プロファイリング用の変数を状態構造体 (state) に追加
            sensor_props = [ ...
                "time", ...                          % センシング実行時刻 [s]
                "pQ", ...                            % センシング時のUAV機体重心座標 [m]
                "detected_point", ...                % 15m以内に障害物が1つ以上存在するか (true/false)
                "detected_obstacles", ...            % 15m以内で検知された障害物リスト (最寄り順ソート済み)
                "min_obstacle_id_point", ...         % 最も近接している障害物のID
                "min_dist_point", ...                % 最も近接している障害物表面までの距離 [m]
                "detected_obstacle_count_point", ... % 15m以内で検知された障害物の総数
                "profiling" ...                      % センサ処理各ステージの計算時間プロファイラ [ms]
            ];
            for p_name = sensor_props
                % 未定義プロパティのみを動的に追加 (二重定義エラーを防止)
                if ~isprop(obj.result.state, p_name)
                    addprop(obj.result.state, p_name);
                end
            end
            % --- 状態バッファの初期化 ---
            % 初期時刻 (t=0) における各センサ出力プロパティをデフォルト値 (NaN, false, inf) で初期化
            obj.clear_state_sensor_values(0.0);
        end

        % =====================================================================
        % do: 制御周期ごとのセンシング実行 & プロファイリング
        % ---------------------------------------------------------------------
        % 【概要】
        %   シミュレーションのメイン制御ループから毎ステップ（制御周期 dt ごと）
        %   自動的に呼び出される実行メソッド。
        %
        % 【責務】
        %   1. 公称目標軌道 (Nominal Reference) の取得と処理時間の計測
        %   2. 飛行フェーズ ('f') における UAV 搭載 15m 球面レンジセンサ検知の実行
        %   3. センサ内部ステージ（外接球足切り、厳密距離計算、ソート等）の性能計測
        %   4. 後続の制御器・ロガー・可視化系へ向けた状態データ (state) の更新・格納
        %
        % 【入出力引数】
        %   - 入力 varargin{1} : time 構造体 (time.t: 現在時刻 [s], time.dt: 制御周期 [s])
        %   - 入力 varargin{2} : cha 文字 (飛行フェーズ文字: 'f'=Flight, 't'=Takeoff, 'l'=Landing 等)
        %   - 出力 result_out  : 状態量 (xd, 検知情報, プロファイリング結果等) を格納した結果構造体
        % =====================================================================
        function result_out = do(obj, varargin)
            % --- 引数のアンパック ---
            time = varargin{1}; % 時間管理構造体 (time.t: シミュレーション現在時刻 [s])
            cha  = varargin{2}; % 現在のフライトモード文字 (例: 'f'=飛行中, 't'=離陸中)

            % =================================================================
            % 公称目標軌道の取得 (Nominal Reference 透過処理)
            % =================================================================
            % 【設計意図】
            %   本クラスは現段階では「純粋センサ検知モジュール」であるため、
            %   公称軌道生成器 (base_ref) の出力をそのまま後段へバイパス（透過）します。
            %   センサ自体の処理時間と公称軌道の生成時間を完全に分離して計測するため、
            %   呼び出しの前後を tic / toc で囲んで独立計測します。
            t_base_start = tic;
            base_res = obj.base_ref.do(varargin{:}); % 公称目標軌道の計算実行
            xd_nom = base_res.state.xd; % 公称目標状態ベクトル (28×1 など)
            obj.profiling.t_base_ref_ms = toc(t_base_start) * 1000.0; % 公称目標軌道の計算に要した時間 [ms] をプロファイラに記録
            % 前回ステップの検知プロパティ値をクリア（初期化）
            obj.clear_state_sensor_values(time.t);

            % =================================================================
            % 局所検知結果構造体 (detection) の定義と初期化
            % =================================================================
            % 非飛行フェーズ（待機中や着陸中）または障害物非検知時でも
            % 変数の未定義エラーを防ぐため、デフォルト値（安全側・非検知側）で初期化します。
            detection = struct();
            detection.time                          = time.t;            % センシング実行時刻 [s]
            detection.pQ                            = [NaN; NaN; NaN];   % センシング時のUAV機体重心位置 [x; y; z] [m] (非飛行時は未定義)
            detection.detected_point                = false;             % センサー範囲検知フラグ (true: 1つ以上の障害物を捕捉, false: 未捕捉)
            detection.detected_obstacles            = [];                % センサー範囲以内で捕捉された障害物情報の配列 (未捕捉時は空)
            detection.min_obstacle_id_point         = NaN;               % センサー範囲以内で最寄りの障害物ID (未捕捉時は NaN)
            detection.min_dist_point                = inf;               % センサー範囲以内で最寄りの障害物表面までの距離 [m] (未捕捉時は無限大 inf)
            detection.detected_obstacle_count_point = 0;                 % センサー範囲以内で捕捉された障害物の個数 [個]

            % =================================================================
            % 飛行フェーズ ('f') における センサー範囲 球面レンジ検知
            % =================================================================
            if cha == 'f'
                % エスティメータ（状態推定器）から現在のUAV機体重心位置 pQ [m] (3×1) を取得
                pQ_cur = obj.self.estimator.result.state.p(:);

                % --- センサ検知本体の実行 ---
                % 【引数】pQ_cur: UAV位置, time.t: 現在時刻
                % 【戻り値】
                %   - detection    : 検知結果が格納された構造体
                %   - sensor_timing: 内部ステージごとの実行時間詳細 [ms]
                [detection, sensor_timing] = obj.detect_obstacles_fast(pQ_cur, time.t);

                % --- 計測時間・性能情報のプロファイラ構造体への格納 ---
                obj.profiling.t_sensor_total_ms  = sensor_timing.total_ms;         % センサ処理全体の合計時間 [ms]
                obj.profiling.t_sensor_stage1_ms = sensor_timing.stage1_ms;        % Stage 1: 外接球による足切り判定の所要時間 [ms]
                obj.profiling.t_sensor_stage2_ms = sensor_timing.stage2_ms;        % Stage 2: 楕円体表面最短距離の反復計算時間 [ms]
                obj.profiling.t_sensor_sort_ms   = sensor_timing.sort_ms;          % Stage 3: 最寄り順ソート・構造体詰替時間 [ms]
                obj.profiling.candidate_count    = sensor_timing.candidate_count;  % Stage 1 の足切りを通過して Stage 2 に進んだ候補障害物数 [個]

                % =============================================================
                % リアルタイムログ出力判定
                % =============================================================
                % 【条件】
                %   1. ENABLE_ALL_LOG == 1 : クラス全体のマスターログスイッチが有効
                %   2. LOG_SENSOR == 1     : センサモジュールのログスイッチが有効
                %   3. detected_point      : 実際に1つ以上の障害物を検知している
                % これらすべてを満たす場合のみ fprintf を実行し、平常時の I/O 負荷を完全に遮断します。
                if (obj.ENABLE_ALL_LOG == 1) && (obj.LOG_SENSOR == 1) && detection.detected_point
                    tgt = detection.detected_obstacles(1); % ソート済みのため第1要素が最寄り障害物
                    fprintf("[SENSOR DETECT] t=%.3f s | 捕捉数: %d | 最寄ID: %d | 表面距離: %6.3f m | 総計算: %.3f ms (Stage1: %.3f ms, Stage2: %.3f ms, 候補数: %d)\n", ...
                        time.t, ...                               % 現在時刻 [s]
                        detection.detected_obstacle_count_point, ... % 検知した障害物の総数
                        tgt.id, ...                               % 最寄り障害物の識別番号
                        tgt.range, ...                            % 最寄り障害物表面までのユークリッド距離 [m]
                        sensor_timing.total_ms, ...               % センサ検知の総計算時間 [ms]
                        sensor_timing.stage1_ms, ...              % 外接球足切り時間 [ms]
                        sensor_timing.stage2_ms, ...              % 厳密反復計算時間 [ms]
                        sensor_timing.candidate_count);           % 厳密計算を実施した候補数
                end
            end

            % =================================================================
            % 目標軌道出力の決定
            % =================================================================
            % 現段階では軌道回避を行わないため、公称軌道をそのまま制御目標としてバイパス出力
            xd_out = xd_nom;

            % =================================================================
            % 状態格納 (後段の制御器・ロガー・プランナー向け出力パッキング)
            % =================================================================
            st = obj.result.state;
            
            % --- 制御目標値 ---
            st.xd = xd_out;         % 目標全状態ベクトル [28×1] (位置, 速度, 加速度, 躍度, 姿勢等)
            st.p  = xd_out(1:3);     % 目標位置 [x; y; z] [m]
            st.v  = xd_out(5:7);     % 目標並進速度 [vx; vy; vz] [m/s]
            st.q  = [0; 0; xd_out(4)]; % 目標姿勢角 [roll; pitch; yaw] [rad] (yawのみ公称追従)

            % --- センサ検知・診断情報 ---
            st.time                          = detection.time;                          % センシング基準時刻 [s]
            st.pQ                            = detection.pQ;                            % 検知時の機体重心位置 [m]
            st.detected_point                = detection.detected_point;                % 障害物検知フラグ (true/false)
            st.detected_obstacles            = detection.detected_obstacles;            % 15m以内の障害物リスト構造体配列
            st.min_obstacle_id_point         = detection.min_obstacle_id_point;         % 最も近接している障害物ID
            st.min_dist_point                = detection.min_dist_point;                % 最も近接している障害物表面までの距離 [m]
            st.detected_obstacle_count_point = detection.detected_obstacle_count_point; % 検知障害物数 [個]
            
            % --- プロファイリング情報 ---
            st.profiling                     = obj.profiling;                           % 各ステージの処理時間内訳構造体 [ms]

            % 最終結果構造体を呼び出し元へ返却
            result_out = obj.result;
        end

        % =====================================================================
        % benchmark_stage2: timeit による厳密アルゴリズムベンチマーク (公開関数)
        % ---------------------------------------------------------------------
        % 【概要】
        %   MATLAB 公式の性能評価関数 `timeit` および 1000 回サンプリングを用い、
        %   Stage 2（楕円体表面最短距離計算コア関数）の純粋な計算時間・ジッタ幅を精密測定します。
        %
        % 【背景と目的】
        %   制御ループ内の単発 tic/toc 計測では、OS のコンテキストスイッチや CPU 周波数スケーリング
        %   などの外部ノイズが乗りやすく、見かけ上の値が跳ねる場合があります。
        %   本メソッドはウォームアップ＋複数回反復により、JIT コンパイル後の「真の処理性能」を明らかにします。
        %
        % 【入出力引数】
        %   - 入力 pQ_test   : テスト用の UAV 評価位置 [x; y; z] [m]（省略時は [0.3; 0.3; 5.0]）
        %   - 出力 t_mean_ms : timeit により得られた代表実行時間（平均処理時間）[ms]
        %   - 出力 t_std_ms  : 1000 回単発実行における標準偏差（測定ジッタ・ばらつき幅）[ms]
        % =====================================================================
        function [t_mean_ms, t_std_ms] = benchmark_stage2(obj, pQ_test)
            % --- テスト位置および事前キャッシュマップの確認 ---
            if nargin < 2 || isempty(pQ_test)
                pQ_test = [0.3; 0.3; 5.0]; % デフォルト評価位置 [m]
            end
            if isempty(obj.static_obstacle_map)
                error('静的障害物マップが空です。初期化されているか確認してください。');
            end

            % --- 評価対象パラメータの抽出 ---
            test_obs = obj.static_obstacle_map(1); % マップ内の先頭障害物をテスト対象として選定
            p = pQ_test(:);                        % UAV の位置ベクトル [3×1] [m]
            c = test_obs.p_obs;                    % 楕円体中心座標 [3×1] [m]
            r = test_obs.radii_obs;                % 楕円体3主軸半径 [rx; ry; rz] [m]
            R = test_obs.R_obs;                    % 姿勢回転行列 [3×3]

            % 計測対象となるコア関数の関数ハンドルを作成（struct 生成等の余分な処理を含まない純粋処理）
            core_fun = @() obj.point_ellipsoid_signed_distance_fast(p, c, r, R);

            % --- ウォームアップ実行 (50回) ---
            % MATLAB インタプリタの JIT 最適化を確実に完了させ、初回実行時のオーバーヘッドを排除
            for w = 1:50
                core_fun();
            end

            % --- timeit による代表実行時間（統計的中央値ベース）の厳密測定 ---
            t_sec = timeit(core_fun); % 秒単位で取得
            t_mean_ms = t_sec * 1000.0; % ミリ秒単位 [ms] に換算

            % --- 1000回単発 tic/toc によるジッタ（ばらつき）サンプリング ---
            N_samples = 1000; % サンプル数
            t_samples_ms = zeros(N_samples, 1); % 各回の計測値格納配列 [ms]
            for s = 1:N_samples
                t_s = tic;
                core_fun();
                t_samples_ms(s) = toc(t_s) * 1000.0;
            end
            t_std_ms = std(t_samples_ms); % サンプル標準偏差 [ms]

            % --- 6. ベンチマーク結果のコンソール表示 ---
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
        % ---------------------------------------------------------------------
        % 【概要】
        %   UAV 機体重心 pQ を中心とする半径 15m の全方位球面レンジ内に、
        %   楕円体障害物の表面が存在するかどうかを 2 段階スクリーニングで高速検知します。
        %
        % 【アルゴリズムの 3 ステージ構成】
        %   - Stage 1: 外接球による高速 Reject（遠方の障害物を乗算・比較のみで瞬時に除外）
        %   - Stage 2: 残った候補に対する厳密な楕円体表面距離判定（40回反復二分法）
        %   - Stage 3: 最寄り順（表面距離昇順）ソートと出力構造体パッキング
        %
        % 【入出力引数】
        %   - 入力 pQ     : UAV 機体重心位置 [x; y; z] [m]
        %   - 入力 t_now  : センシング現在時刻 [s]
        %   - 出力 det    : 検知情報構造体（検知フラグ、最寄ID、リスト等）
        %   - 出力 timing : 各ステージの処理時間 [ms] および通過候補数を格納した構造体
        % =====================================================================
        function [det, timing] = detect_obstacles_fast(obj, pQ, t_now)
            t_total_start = tic; % センサ検知全体の計測開始

            % --- 出力用構造体 (det) の初期化 ---
            det = struct();
            det.time                          = t_now;            % センシング基準時刻 [s]
            det.pQ                            = pQ;               % センシング時のUAV位置 [m]
            det.detected_point                = false;            % 検知フラグ（1つでも検知すれば true）
            det.detected_obstacles            = [];               % 15m以内で検知された障害物配列
            det.min_obstacle_id_point         = NaN;              % 最寄り障害物の識別番号 ID
            det.min_dist_point                = inf;              % 最寄り障害物表面までの距離 [m]
            det.detected_obstacle_count_point = 0;                % 15m以内で検知された障害物の個数

            % --- 計測時間・性能統計用構造体 (timing) の初期化 ---
            timing = struct( ...
                'stage1_ms',       0.0, ... % Stage 1: 外接球足切り時間 [ms]
                'stage2_ms',       0.0, ... % Stage 2: 楕円体表面距離反復計算時間 [ms]
                'sort_ms',         0.0, ... % Stage 3: ソート・パッキング時間 [ms]
                'total_ms',        0.0, ... % センサ検知の総処理時間 [ms]
                'candidate_count', 0);      % Stage 1 を通過した候補障害物数

            obs_map = obj.static_obstacle_map;
            if isempty(obs_map)
                timing.total_ms = toc(t_total_start) * 1000.0;
                return;
            end

            % -----------------------------------------------------------------
            % Stage 1: 事前計算外接球による高速 Reject 判定 (全障害物スクリーニング)
            % -----------------------------------------------------------------
            % 【原理】
            %   中心間距離の2乗 ||pQ - c||^2 が (15m + r_max)^2 より大きければ、
            %   楕円体がどの方向を向いていようとも表面が 15m 圏内に入ることは幾何学的にあり得ない。
            %   平方根 sqrt を取らず、dot 積による 2 乗比較で計算コストを最小化。
            t_stage1_start = tic;
            passed_indices = []; % 足切りをクリアした障害物のインデックス配列
            for i = 1:numel(obs_map)
                dc = pQ - obs_map(i).p_obs; % 中心間相対位置ベクトル [m]
                d2c = dot(dc, dc); % 中心間距離の2乗 [m^2]
                % 外接球判定: 中心距離 <= センサ半径(15m) + 最大半径 r_max
                if d2c <= (obj.trigger_dist + obs_map(i).r_max)^2
                    passed_indices(end+1) = i; % 候補として通過
                end
            end
            timing.stage1_ms = toc(t_stage1_start) * 1000.0;
            timing.candidate_count = length(passed_indices);

            % 候補が 1 つも無ければ Stage 2 を行わずに即座に早期リターン
            if isempty(passed_indices)
                timing.total_ms = toc(t_total_start) * 1000.0;
                return;
            end

            % -----------------------------------------------------------------
            % Stage 2: 候補に対する楕円体表面最短距離判定 (反復法)
            % -----------------------------------------------------------------
            % 【原理】
            %   Stage 1 を通過した近傍の障害物のみを対象に、楕円体方程式に対する
            %   厳密な最短ユークリッド距離を計算し、真に 15m 圏内にあるかを確定する。
            t_stage2_start = tic;
            % 15m 検知候補を格納する構造体配列テンプレート
            candidates = repmat(struct( ...
                'id',        [], ... % 障害物ID
                'p_obs',     [], ... % 中心座標 [m]
                'R_obs',     [], ... % 回転行列 [3×3]
                'radii_obs', [], ... % 3主軸半径 [m]
                'range',     []), ...% 表面までの真のユークリッド距離 [m]
                0, 1);

            for idx = passed_indices
                o = obs_map(idx);
                % struct 生成を避け、引数を直接渡して表面距離を計算
                d_surface = obj.point_ellipsoid_signed_distance_fast(pQ, o.p_obs, o.radii_obs, o.R_obs);
                % 表面距離が 15m 以内の場合のみ最終検知リストへ登録
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
            % 15m 以内に障害物が無ければ終了
            if isempty(candidates)
                timing.total_ms = toc(t_total_start) * 1000.0;
                return;
            end

            % -----------------------------------------------------------------
            % Stage 3: 最寄り順ソートと出力格納
            % -----------------------------------------------------------------
            % 表面距離 (range) が近い順（昇順）に並べ替え、配列の先頭を「最優先・最寄り」とする
            t_sort_start = tic;
            [~, sort_idx] = sort([candidates.range], 'ascend');
            
            det.detected_obstacles            = candidates(sort_idx);              % ソート済み障害物配列
            det.detected_point                = true;                               % 検知フラグ成立
            det.min_dist_point                = det.detected_obstacles(1).range;   % 最寄り障害物の表面距離 [m]
            det.min_obstacle_id_point         = det.detected_obstacles(1).id;      % 最寄り障害物のID
            det.detected_obstacle_count_point = length(candidates);                % 検知総数 [個]
            
            timing.sort_ms = toc(t_sort_start) * 1000.0;
            timing.total_ms = toc(t_total_start) * 1000.0;
        end

        % =====================================================================
        % point_ellipsoid_signed_distance_fast: ラグランジュ未定乗数法 (表面距離)
        % ---------------------------------------------------------------------
        % 【概要】
        %   3次元空間の点 p から楕円体表面までの「符号付き最短ユークリッド距離」を、
        %   楕円体方程式に対するラグランジュ未定乗数法と二分法（40回反復）で厳密に計算します。
        %
        % 【数理モデル】
        %   楕円体主軸局所座標 y = R^T * (p - c) に対し、表面上の最寄り点 y* は
        %   未定乗数 lambda を用いて y*_i = (r_i^2 * y_i) / (lambda + r_i^2) と表される。
        %   これが楕円面方程式 sum((y*_i / r_i)^2) = 1 を満たす単調減少関数 f(lambda) = 0
        %   となる lambda を二分探索で決定し、||y - y*|| を算出する。
        %
        % 【入出力引数】
        %   - 入力 p : UAV 機体重心座標 [3×1] [m]
        %   - 入力 c : 楕円体中心座標 [3×1] [m]
        %   - 入力 r : 楕円体3主軸半径 [rx; ry; rz] [m]
        %   - 入力 R : 楕円体姿勢回転行列 [3×3]（ワールド -> 局所座標系）
        %   - 出力 d : 符号付き表面距離 [m] (外部: d > 0, 表面: d = 0, 内部: d < 0)
        % =====================================================================
        function d = point_ellipsoid_signed_distance_fast(~, p, c, r, R)
            p = p(:);
            c = c(:);
            r = r(:);

            % --- 1. 楕円体の局所（主軸）座標系への変換 ---
            y  = R' * (p - c);              % 局所座標系での相対位置ベクトル [m]
            r2 = r.^2;                      % 各主軸半径の2乗 [rx^2; ry^2; rz^2]
            q  = sum((y ./ r).^2);          % 楕円体二次形式の値
            inside = (q < 1.0);             % q < 1 なら内部、q >= 1 なら外部/表面

            % 特異点処理: 点が中心近傍 (y ≈ 0) にある場合、最短距離は最小主軸半径
            if norm(y) < 1e-14
                d = -min(r);
                return;
            end

            % --- 2. ラグランジュ方程式 f(lambda) の定義 ---
            % f(lambda) = sum[ r_i^2 * y_i^2 / (lambda + r_i^2)^2 ] - 1 = 0
            f = @(lam) sum(r2 .* (y.^2) ./ ((lam + r2).^2)) - 1.0;

            % --- 3. 二分探索の初期探索区間 [lo, hi] の設定 ---
            if q > 1.0
                % 外部点の場合: 解 lambda は 0 以上
                lo = 0.0;
                hi = max(r) * norm(y);
            else
                % 内部点の場合: 解 lambda は -min(r^2) よりわずかに大きい負の値
                lo = -min(r2) * (1.0 - 1e-12);
                hi = 0.0;
            end

            % --- 4. 固定反復数（40回）による二分探索 ---
            % 探索区間幅が (hi - lo) * (1/2)^40 ≈ 10^(-12) 程度まで収束
            for kk = 1:40
                mid = 0.5 * (lo + hi);
                if f(mid) > 0
                    lo = mid; % 単調減少関数のため、f(mid) > 0 なら解は右側
                else
                    hi = mid; % f(mid) <= 0 なら解は左側
                end
            end

            % --- 5. 最短表面点およびユークリッド距離の算出 ---
            lambda = 0.5 * (lo + hi);                     % 確定した未定乗数 lambda
            closest_local = r2 .* y ./ (lambda + r2);      % 局所座標系における楕円体表面最近接点 [m]
            d_abs = norm(closest_local - y);              % 幾何ユークリッド距離の絶対値 [m]

            % 内部の場合は負値、外部の場合は正値として符号を付与
            if inside
                d = -d_abs;
            else
                d = d_abs;
            end
        end

        % =====================================================================
        % clear_state_sensor_values: 制御周期開始時のセンサ状態バッファ初期化
        % ---------------------------------------------------------------------
        % 【概要】
        %   毎ステップのセンシング処理前に状態構造体 (state) 内のセンサプロパティを
        %   非検知状態（安全側デフォルト値）にリセットし、過去ステップの残存値をクリアします。
        %
        % 【引数】
        %   - 入力 t_now : 現在時刻 [s]
        % =====================================================================
        function clear_state_sensor_values(obj, t_now)
            st = obj.result.state;
            st.time                          = t_now;            % 現在時刻 [s]
            st.pQ                            = [NaN; NaN; NaN];   % 機体位置（未測定状態）
            st.detected_point                = false;             % 非検知
            st.detected_obstacles            = [];                % 空配列
            st.min_obstacle_id_point         = NaN;               % 未定義
            st.min_dist_point                = inf;               % 距離無限大
            st.detected_obstacle_count_point = 0;                 % 0個
        end
    end
end
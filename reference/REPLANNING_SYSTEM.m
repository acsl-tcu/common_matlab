classdef REPLANNING_SYSTEM < handle
    % =========================================================================
    % REPLANNING_SYSTEM
    %
    % 【概要】
    %   索吊り下げ型ドローン (Quadrotor with Suspended Load) 向け
    %   超高速MEXトリガー＆20mセンサ検知統括クラス。
    %
    % 【設計思想】
    %   1. スプラインモデル生成および重い公称軌道サンプリングループを完全撤廃。
    %   2. 制御周期ごとの計算負荷をマイクロ秒（0.01ms級）に極小化。
    %   3. 20m圏内センサ検知および現在状態（荷物・機体・索）の接触判定を
    %      新C++ MEX カーネル (trigger_packet_kernel_mex) で瞬時に一括実行。
    %   4. 制御周期 dt、重力加速度 gravity、索長 L_cable を本体・パラメータから
    %      厳格に取得（未設定時は即座に assert エラー停止）。
    %   5. 牽引物質量 loadmass は推定器の推定値 (estimator.result.state.mL) を厳格取得。
    %      推定値が取得できない場合は即座に assert エラー停止（フォールバック完全撤廃）。
    %   6. プロパティ名の衝突を防ぐため、索保護球半径を r_cable_sphere に改名。
    %   7. ENVIRONMENT_OBSTACLE_ELLIPSE_MOVE の全障害物形状 (球、円柱、直方体、
    %      三角柱、円錐、角錐、楕円体、カスタム) に完全対応。外接球半径 r_obs を
    %      環境側から直接受け取り、マージン無しの幾何楕円体近似をベースとして
    %      システム側マージン (初期値 0.0) を付与。
    %   8. ログ表示フラグ (ENABLE_ALL_LOG, LOG_SENSOR, LOG_TRIGGER) により
    %      20m内障害物情報や接触判定の表示／非表示を自由に切り替え可能。
    %      すべての検知・判定結果は状態構造体 (result.state) に毎周期記録され、
    %      LOGGER クラスによって自動保存される。
    % =========================================================================

    properties (Access = public)
        % =====================================================================
        % ログ表示マスタースイッチ & モジュール別スイッチ
        % =====================================================================
        ENABLE_ALL_LOG = 1;        % 1: ON (ログ表示), 0: OFF (非表示・最高速)
        LOG_SENSOR     = 1;        % 20m以内の障害物検知ログ表示 (1: ON, 0: OFF)
        LOG_TRIGGER    = 1;        % 接触判定 (回避トリガー) ログ表示 (1: ON, 0: OFF)

        % =====================================================================
        % 基本オブジェクト参照
        % =====================================================================
        self                       % ドローンエージェント自身 (パラメータ・状態取得元)
        base_ref                   % 公称参照軌道オブジェクト
        result                     % 出力結果構造体

        % =====================================================================
        % センサ検知パラメータ
        % =====================================================================
        trigger_dist   = 20.0;     % センサ検知球半径 R_sense [m] (デフォルト: 20m)
        prediction_dt  = 0.025;    % 制御周期 [s] (本体から厳格取得)

        % =====================================================================
        % 幾何保護半径 & 安全マージン [m]
        % =====================================================================
        d_margin       = 0.0;      % システム離隔マージン [m] (デフォルト: 0.0)
        r_drone        = 0.5;      % 機体等価球半径 [m]
        r_load         = 0.5;      % 荷物球半径 [m]
        r_cable_sphere = 0.5;      % 索保護球半径 [m] (旧 r_cable から改名し衝突回避)

        % =====================================================================
        % 物理モデルパラメータ
        % =====================================================================
        gravity        = 9.81;     % 重力加速度 [m/s^2] (self.parameter から厳格取得)
        L_cable        = 2.0;      % 索長 [m] (self.parameter から厳格取得)
        loadmass       = 0.0;      % 牽引物質量 [kg] (推定器から厳格取得・未推定時エラー)

        % =====================================================================
        % 静的障害物マップ & MEX用事前幾何パケット
        % =====================================================================
        static_obstacle_map = [];  % 生の環境障害物データ (ENVIRONMENT_OBSTACLE)
        obstacle_packet     = [];  % MEX向け幾何前計算済み障害物パケット

        % =====================================================================
        % MEX ランタイム管理
        % =====================================================================
        mex_name       = 'trigger_packet_kernel_mex'; % MEX関数名
        mex_ready      = false;    % MEX準備完了フラグ

        % =====================================================================
        % 互換用ダミープロパティ (外部参照でのエラー防止)
        % =====================================================================
        current_bspline = [];      % スプラインモデル生成廃止に伴う空プレースホルダー

        % =====================================================================
        % 計算負荷プロファイリング [ms]
        % =====================================================================
        profiling = struct( ...
            't_base_ref_ms', 0.0, ... % 公称軌道透過取得時間 [ms]
            't_trigger_ms',  0.0, ... % センサ＆接触判定合計時間 [ms]
            't_kernel_ms',   0.0);    % 純粋MEXカーネル時間 [ms]
    end

    methods (Access = public)
        % =====================================================================
        % コンストラクタ：初期化・パラメータバインド・MEX自動コンパイル
        % =====================================================================
        function obj = REPLANNING_SYSTEM(self, base_ref, opts)
            arguments
                self                  % 必須: エージェントインスタンス
                base_ref              % 必須: 公称参照軌道オブジェクト
                opts = struct()       % 任意: 外部設定構造体
            end

            % --- 1. 基本インスタンスの保持 ---
            obj.self     = self;
            obj.base_ref = base_ref;

            % --- 2. 外部オプション (opts) によるパラメータ上書き ---
            if isfield(opts, 'ENABLE_ALL_LOG'), obj.ENABLE_ALL_LOG = opts.ENABLE_ALL_LOG; end
            if isfield(opts, 'LOG_SENSOR'),     obj.LOG_SENSOR     = opts.LOG_SENSOR;     end
            if isfield(opts, 'LOG_TRIGGER'),    obj.LOG_TRIGGER    = opts.LOG_TRIGGER;    end

            % センサ検知半径 (デフォルト 20m)
            if isfield(opts, 'trigger_dist'),   obj.trigger_dist   = opts.trigger_dist;   end
            if isfield(opts, 'sensor_range_m'), obj.trigger_dist   = opts.sensor_range_m; end

            % 安全マージン (システム側のマージン: デフォルト 0.0)
            if isfield(opts, 'd_margin'),           obj.d_margin   = opts.d_margin;           end
            if isfield(opts, 'D_margin'),           obj.d_margin   = opts.D_margin;           end
            if isfield(opts, 'd_collision_thresh'), obj.d_margin   = opts.d_collision_thresh; end
            if isfield(opts, 'D_trigger'),          obj.d_margin   = opts.D_trigger;          end

            % 各部等価球半径 (r_cable プロパティ名衝突回避版)
            if isfield(opts, 'r_drone'),        obj.r_drone        = opts.r_drone;        end
            if isfield(opts, 'r_load'),         obj.r_load         = opts.r_load;         end
            if isfield(opts, 'r_cable_sphere'), obj.r_cable_sphere = opts.r_cable_sphere; end
            if isfield(opts, 'r_cable'),        obj.r_cable_sphere = opts.r_cable;        end % 旧オプション互換
            if isfield(opts, 'cable'),          obj.r_cable_sphere = opts.cable;          end % 旧オプション互換

            % --- 3. 制御周期 dt の厳格バインド (本体 self.plant から取得、無ければ停止) ---
            dt_val = [];
            if isprop(obj.self, 'plant') && ~isempty(obj.self.plant) && isprop(obj.self.plant, 'dt') && ~isempty(obj.self.plant.dt)
                dt_val = obj.self.plant.dt;
            elseif isprop(obj.self, 'parameter') && ~isempty(obj.self.parameter)
                try
                    dt_val = obj.self.parameter.get("dt");
                catch
                end
            end
            if isempty(dt_val) && isfield(opts, 'dt')
                dt_val = opts.dt;
            end
            assert(~isempty(dt_val) && isnumeric(dt_val) && dt_val > 0, ...
                '[REPLANNER ERROR] 制御周期 dt の取得に失敗しました。本体 (self.plant.dt 等) に設定されている必要があります。');
            obj.prediction_dt = dt_val;

            % --- 4. 物理パラメータの厳格バインド (self.parameter から直接取得) ---
            obj.gravity = obj.self.parameter.get("gravity");
            obj.L_cable = obj.self.parameter.get("cableL");

            assert(~isempty(obj.gravity) && isnumeric(obj.gravity) && obj.gravity > 0, ...
                '[REPLANNER ERROR] self.parameter から gravity の取得に失敗しました。');
            assert(~isempty(obj.L_cable) && isnumeric(obj.L_cable) && obj.L_cable > 0, ...
                '[REPLANNER ERROR] self.parameter から cableL の取得に失敗しました。');

            % 牽引物質量の取得 (推定値優先)
            obj.loadmass = obj.get_load_mass();

            % 出力構造体の初期テンプレートを公称軌道から継承
            obj.result = base_ref.result;

            % --- 5. MEXファイルの存在確認 & 自動ビルド ---
            obj.ensure_mex_compiled();

            % --- 6. 静的障害物マップの取得 & MEXパケット事前構築 ---
            raw_obs = ENVIRONMENT_OBSTACLE_ELLIPSE_MOVE(0.0);
            obj.static_obstacle_map = raw_obs;
            obj.obstacle_packet     = obj.prepare_obstacle_packet(raw_obs);

            % --- 7. 状態構造体 (state) へのプロパティ動的登録 (LOGGER 保存用) ---
            sensor_props = [ ...
                "time", ...                          % 実行時刻 [s]
                "pQ", ...                            % 機体重心座標 [m]
                "detected_point", ...                % 20m内検知フラグ (true/false)
                "detected_obstacles", ...            % 20m内障害物リスト
                "need_replan", ...                   % 接触・回避トリガーフラグ (true/false)
                "relevant_obstacles", ...            % 接触対象障害物情報
                "min_obstacle_id_point", ...         % 最寄障害物ID
                "min_dist_point", ...                % 最短離隔距離 [m]
                "detected_obstacle_count_point", ... % 20m内障害物数
                "loadmass", ...                      % 牽引物質量 [kg] (推定値)
                "profiling", ...                     % 計算時間プロファイル構造体
                "current_bspline" ...                % 互換用ダミー
            ];
            for p_name = sensor_props
                if ~isprop(obj.result.state, p_name)
                    addprop(obj.result.state, p_name);
                end
            end

            % --- 8. 状態バッファの初期化 ---
            obj.clear_state_sensor_values(0.0);
        end

        % =====================================================================
        % do: 制御周期ごとの公称軌道透過取得・20mセンサ検知・接触トリガー判定
        % =====================================================================
        function result_out = do(obj, varargin)
            % 引数アンパック: time (時刻構造体), cha (フライトモード文字)
            time = varargin{1};
            cha  = varargin{2};

            % 制御周期の動的追従 (time.dt が存在する場合)
            if isfield(time, 'dt') && ~isempty(time.dt) && time.dt > 0
                obj.prediction_dt = time.dt;
            end

            % 牽引物質量の最新値取得 (推定値優先)
            obj.loadmass = obj.get_load_mass();

            % --- 1. 公称目標軌道の取得 (Nominal Reference 透過処理) ---
            t_base_start = tic;
            base_res = obj.base_ref.do(varargin{:});
            xd_nom   = base_res.state.xd;
            obj.profiling.t_base_ref_ms = toc(t_base_start) * 1000.0;

            % プロファイリングバッファのリセット
            obj.profiling.t_trigger_ms = 0.0;
            obj.profiling.t_kernel_ms  = 0.0;

            % 前ステップの状態値をクリア
            obj.clear_state_sensor_values(time.t);

            % --- 2. 出力変数のデフォルト初期化 ---
            detection = struct();
            detection.time                          = time.t;
            detection.pQ                            = [NaN; NaN; NaN];
            detection.detected_point                = false;
            detection.detected_obstacles            = [];
            detection.min_obstacle_id_point         = NaN;
            detection.min_dist_point                = inf;
            detection.detected_obstacle_count_point = 0;

            need_replan        = false;
            relevant_obstacles = [];

            % --- 3. 飛行フェーズ ('f') における超高速20mセンサ＆接触判定 ---
            if cha == 'f'
                t_trig_start = tic;

                % (a) 現在の状態ベクトルを取得 (推定器または公称軌道からO(1)で直接取得)
                if isprop(obj.self.estimator.result.state, "p")
                    pQ_cur = obj.self.estimator.result.state.p(:);
                else
                    pQ_cur = xd_nom(1:3);
                end
                detection.pQ = pQ_cur;

                % 荷物位置 pL
                if isprop(obj.self.estimator.result.state, "pL")
                    pL_cur = obj.self.estimator.result.state.pL(:);
                else
                    pL_cur = pQ_cur - [0; 0; obj.L_cable];
                end

                % 索姿勢単位ベクトル q (UAV -> Load 方向)
                if isprop(obj.self.estimator.result.state, "pT")
                    q_cur = obj.self.estimator.result.state.pT(:);
                    norm_q = norm(q_cur);
                    if norm_q > 1e-6, q_cur = q_cur / norm_q; else, q_cur = [0; 0; -1]; end
                else
                    if length(xd_nom) >= 11, aL = xd_nom(9:11); else, aL = [0; 0; 0]; end
                    cable_vec = aL + [0; 0; obj.gravity];
                    norm_cable = norm(cable_vec);
                    if norm_cable > 1e-4, q_cur = -cable_vec / norm_cable; else, q_cur = [0; 0; -1]; end
                end

                % (b) C++ MEX カーネル呼び出し (全形状対応・距離計算もC++内で一括実行)
                pack = obj.obstacle_packet;
                t_k_start = tic;

                [triggered, ~, obs_id, comp_code, vis_count, min_dist, min_id, all_dists] = ...
                    trigger_packet_kernel_mex( ...
                        1, ...                % 開始インデックス
                        0.0, ...              % ホライズン H = 0 (瞬時判定)
                        obj.prediction_dt, ...% dt
                        obj.trigger_dist, ... % センサ検知半径 (20m)
                        obj.L_cable, ...      % 索長 L
                        pL_cur, ...           % 3×1 現在荷物位置
                        q_cur, ...            % 3×1 現在索姿勢
                        pack.C, ...           % 障害物中心 (3×M)
                        pack.r_obs, ...       % 外接球半径 (1×M) (ENVIRONMENTから直接取得)
                        pack.raw_h, ...       % 補助パラメータ (1×M)
                        pack.R, ...           % 回転行列 (9×M)
                        pack.Apay, ...        % 荷物保護楕円体 (3×M)
                        pack.Auav, ...        % UAV保護楕円体 (3×M)
                        pack.Acable, ...      % 索保護楕円体 (3×M)
                        pack.ids);            % 障害物ID (1×M)

                t_kernel = toc(t_k_start) * 1000.0;
                obj.profiling.t_kernel_ms = t_kernel;

                % (c) 20m以内の障害物リスト生成 (MEXから返された距離配列を利用し超高速パッキング)
                detected_obs = [];
                if vis_count > 0
                    vis_indices = find(all_dists <= obj.trigger_dist);
                    num_vis = numel(vis_indices);
                    detected_obs = repmat(struct('id', 0, 'range', 0), num_vis, 1);
                    for k = 1:num_vis
                        idx = vis_indices(k);
                        detected_obs(k).id    = pack.ids(idx);
                        detected_obs(k).range = all_dists(idx);
                    end
                end

                % (d) 判定結果のパッキング
                detection.detected_point                = (vis_count > 0);
                detection.detected_obstacles            = detected_obs;
                detection.min_obstacle_id_point         = min_id;
                detection.min_dist_point                = min_dist;
                detection.detected_obstacle_count_point = vis_count;

                need_replan = logical(triggered);

                if need_replan
                    comp_name = obj.get_component_name(comp_code);
                    rel = struct();
                    rel.id            = obs_id;
                    rel.threat_source = comp_name;
                    rel.pL_collision  = pL_cur;
                    rel.q_collision   = q_cur;
                    rel.pQ_collision  = pQ_cur;
                    relevant_obstacles = rel;
                end

                obj.profiling.t_trigger_ms = toc(t_trig_start) * 1000.0;

                % =============================================================
                % リアルタイムログ出力 (表示切替フラグ対応)
                % =============================================================
                if obj.ENABLE_ALL_LOG == 1
                    % 1. 20m内障害物検知ログ (LOG_SENSOR == 1)
                    if (obj.LOG_SENSOR == 1) && detection.detected_point
                        fprintf("[SENSOR 20m] t=%.3f s | 捕捉数: %d | 最寄ID: %d | 表面距離: %6.3f m | 判定計算: %.3f ms (MEX: %.3f ms)\n", ...
                            time.t, vis_count, min_id, min_dist, obj.profiling.t_trigger_ms, t_kernel);
                    end

                    % 2. 接触・衝突トリガーログ (LOG_TRIGGER == 1)
                    if (obj.LOG_TRIGGER == 1) && need_replan
                        fprintf("[CONTACT HIT -> REPLAN] t=%.3f s | 障害物 ID=%d 接触検知！ (接触部位: %s) | 判定計算: %.3f ms\n", ...
                            time.t, obs_id, comp_name, obj.profiling.t_trigger_ms);
                    end
                end
            end

            % --- 4. 状態構造体への格納 & 出力返却 (LOGGER が自動保存) ---
            st = obj.result.state;

            % 制御目標値 (公称軌道をそのまま透過出力)
            st.xd = xd_nom;
            st.p  = xd_nom(1:3);
            st.v  = xd_nom(5:7);
            st.q  = [0; 0; xd_nom(4)];

            % センサ・接触判定結果
            st.time                          = detection.time;
            st.pQ                            = detection.pQ;
            st.detected_point                = detection.detected_point;
            st.detected_obstacles            = detection.detected_obstacles;
            st.need_replan                   = need_replan;
            st.relevant_obstacles            = relevant_obstacles;
            st.min_obstacle_id_point         = detection.min_obstacle_id_point;
            st.min_dist_point                = detection.min_dist_point;
            st.detected_obstacle_count_point = detection.detected_obstacle_count_point;
            st.loadmass                      = obj.loadmass;
            st.current_bspline               = [];

            % プロファイリング情報
            st.profiling                     = obj.profiling;

            result_out = obj.result;
        end

        % =====================================================================
        % get_load_mass: 牽引物質量の厳格取得 (推定器 mL の取得を必須化)
        % =====================================================================
        function mL = get_load_mass(obj)
            % 牽引物質量の取得: 推定器の推定値 (estimator.result.state.mL) を厳格取得
            % ※ 推定器から mL の推定値が取得できない場合は即座にエラー停止 (フォールバック完全撤廃)
            st_est = [];
            if (isprop(obj.self, 'estimator') || isfield(obj.self, 'estimator')) && ~isempty(obj.self.estimator)
                est_obj = obj.self.estimator;
                if (isprop(est_obj, 'result') || isfield(est_obj, 'result')) && ~isempty(est_obj.result)
                    res = est_obj.result;
                    if (isprop(res, 'state') || isfield(res, 'state')) && ~isempty(res.state)
                        st_est = res.state;
                    end
                end
            end

            has_mL = ~isempty(st_est) && (isprop(st_est, 'mL') || isfield(st_est, 'mL'));

            assert(has_mL, ...
                '[REPLANNER ERROR] 推定器 (self.estimator.result.state) に牽引物質量の推定プロパティ "mL" が存在しません。本システムは牽引物質量の推定値に基づく安全検証を必須としています。');

            mL = st_est.mL;

            assert(~isempty(mL) && isnumeric(mL) && ~isnan(mL) && mL > 0, ...
                '[REPLANNER ERROR] 牽引物質量の推定値 (self.estimator.result.state.mL) が有効な正の数値ではありません (値: %s)。', mat2str(mL));
        end
    end

    methods (Access = private)
        % =====================================================================
        % ensure_mex_compiled: C++ソース生成 & MEX自動コンパイル
        % =====================================================================
        function ensure_mex_compiled(obj)
            root = fileparts(mfilename('fullpath'));
            cpp_path = fullfile(root, [obj.mex_name '.cpp']);
            mex_path = fullfile(root, [obj.mex_name '.' mexext]);

            % 1. C++ ソースコードファイルが存在しなければ生成
            if exist(cpp_path, 'file') ~= 2
                obj.write_mex_source(cpp_path);
                if obj.ENABLE_ALL_LOG
                    fprintf('[REPLANNING_SYSTEM] MEXソースコードを生成しました: %s\n', cpp_path);
                end
            end

            % 2. MEX バイナリが存在しない場合はコンパイル実行
            if exist(mex_path, 'file') ~= 3
                if obj.ENABLE_ALL_LOG
                    fprintf('[REPLANNING_SYSTEM] MEXバイナリが存在しないため自動コンパイルを実行します...\n');
                end
                old_dir = pwd;
                cleanup = onCleanup(@() cd(old_dir));
                cd(root);

                mex('-R2018a', cpp_path, '-output', obj.mex_name);
                rehash;

                if obj.ENABLE_ALL_LOG
                    fprintf('[REPLANNING_SYSTEM] MEXコンパイルが正常に完了しました: %s\n', mex_path);
                end
            end

            obj.mex_ready = true;
        end

        % =====================================================================
        % prepare_obstacle_packet: ENVIRONMENTの全形状に対応した幾何パケット構築
        % =====================================================================
        function pack = prepare_obstacle_packet(obj, OBS)
            if isempty(OBS)
                M = 0;
            else
                M = numel(OBS);
            end

            pack = struct();
            pack.count  = M;
            pack.C      = zeros(3, M);
            pack.r_obs  = zeros(1, M);
            pack.raw_h  = zeros(1, M);
            pack.R      = zeros(9, M);
            pack.Apay   = zeros(3, M);
            pack.Auav   = zeros(3, M);
            pack.Acable = zeros(3, M);
            pack.ids    = zeros(1, M);

            for i = 1:M
                o = OBS(i);
                pack.ids(i)  = o.id;
                pack.C(:, i) = o.p_center(:);
                pack.R(:, i) = o.R_obs(:);

                % 1. 外接球半径 (r_obs) の取得: ENVIRONMENT側から直接取得
                if isfield(o, 'r_obs') && ~isempty(o.r_obs)
                    r_bound = o.r_obs;
                elseif isfield(o, 'ellipsoid_radii') && ~isempty(o.ellipsoid_radii)
                    r_bound = max(o.ellipsoid_radii);
                else
                    r_bound = 1.0;
                end
                pack.r_obs(i) = r_bound;

                % 2. マージンなしの外接楕円体半軸 [a; b; c] の取得
                if isfield(o, 'ellipsoid_radii') && ~isempty(o.ellipsoid_radii)
                    radii = o.ellipsoid_radii(:);
                else
                    radii = obj.compute_raw_ellipsoid_radii(o);
                end

                % 3. システム側マージン (初期値ゼロ) ＋ 各部位の等価球半径による拡張
                % ※ 障害物側のマージン (o.d_margin) は加算せず，マージンなしの楕円近似を渡す
                Dp = obj.d_margin + obj.r_load;
                Dq = obj.d_margin + obj.r_drone;
                Dc = obj.d_margin + obj.r_cable_sphere;

                pack.Apay(:, i)   = radii + Dp;
                pack.Auav(:, i)   = radii + Dq;
                pack.Acable(:, i) = radii + Dc;

                % 補助パラメータ (円柱等の高さが存在する場合は格納)
                if isfield(o, 'raw_param') && numel(o.raw_param) >= 2
                    pack.raw_h(i) = o.raw_param(2);
                else
                    pack.raw_h(i) = 2.0 * radii(3);
                end
            end
        end

        % =====================================================================
        % compute_raw_ellipsoid_radii: 障害物形状に応じた外接楕円体寸法の計算
        % (ENVIRONMENT_OBSTACLE_ELLIPSE_MOVE と完全整合)
        % =====================================================================
        function radii = compute_raw_ellipsoid_radii(~, o)
            type = 'unknown';
            if isfield(o, 'type'), type = lower(o.type); end
            raw = [1.0, 1.0];
            if isfield(o, 'raw_param'), raw = o.raw_param; end

            switch type
                case 'sphere'
                    r = raw(1);
                    radii = [r; r; r];
                case 'cylinder'
                    r = raw(1); h = raw(2);
                    radii = [r * sqrt(2); r * sqrt(2); (h / 2) * sqrt(2)];
                case 'box'
                    dx = raw(1); dy = raw(2); dz = raw(3);
                    radii = [dx/2; dy/2; dz/2] * sqrt(3);
                case 'prism'
                    r_tri = raw(1) / sqrt(3); h = raw(2);
                    radii = [r_tri * sqrt(2); r_tri * sqrt(2); (h / 2) * sqrt(2)];
                case 'cone'
                    r = raw(1); h = raw(2);
                    r_bound = max(h / 2, sqrt(r^2 + (h / 2)^2));
                    radii = [r_bound; r_bound; r_bound];
                case 'pyramid'
                    r_bot = sqrt(2) * (raw(1) / 2); h = raw(2);
                    r_bound = max(h / 2, sqrt(r_bot^2 + (h / 2)^2));
                    radii = [r_bound; r_bound; r_bound];
                case 'ellipsoid'
                    radii = raw(:);
                case 'custom'
                    diffs = raw - mean(raw, 2);
                    a = max(abs(diffs(1, :))) * sqrt(3);
                    b = max(abs(diffs(2, :))) * sqrt(3);
                    c = max(abs(diffs(3, :))) * sqrt(3);
                    radii = [max(a, 0.1); max(b, 0.1); max(c, 0.1)];
                otherwise
                    radii = [1.0; 1.0; 1.0];
            end
        end

        % =====================================================================
        % get_component_name: 接触部位コードを文字列へ変換
        % =====================================================================
        function name = get_component_name(~, code)
            switch code
                case 1, name = 'PAYLOAD';
                case 2, name = 'UAV';
                case 3, name = 'CABLE';
                otherwise, name = 'NONE';
            end
        end

        % =====================================================================
        % clear_state_sensor_values: 毎周期開始時の状態バッファ初期化
        % =====================================================================
        function clear_state_sensor_values(obj, t_now)
            st = obj.result.state;
            st.time                          = t_now;
            st.pQ                            = [NaN; NaN; NaN];
            st.detected_point                = false;
            st.detected_obstacles            = [];
            st.need_replan                   = false;
            st.relevant_obstacles            = [];
            st.min_obstacle_id_point         = NaN;
            st.min_dist_point                = inf;
            st.detected_obstacle_count_point = 0;
            st.loadmass                      = obj.loadmass;
            st.current_bspline               = [];
        end

        % =====================================================================
        % write_mex_source: C++ MEX ソースコードの自己生成
        % =====================================================================
        function write_mex_source(~, target_path)
            fid = fopen(target_path, 'w');
            assert(fid >= 0, '[REPLANNER ERROR] MEXソースファイルの作成に失敗しました: %s', target_path);
            cleanup = onCleanup(@() fclose(fid));

            lines = {
                '// Trigger packet kernel for replanning system'
                '// Auto-generated by REPLANNING_SYSTEM'
                '#include "mex.h"'
                '#include <cmath>'
                '#include <algorithm>'
                '#include <limits>'
                ''
                'struct V3 { double x, y, z; };'
                ''
                'static inline V3 sub(V3 a, V3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }'
                'static inline V3 add(V3 a, V3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }'
                'static inline V3 mul(double s, V3 a) { return {s * a.x, s * a.y, s * a.z}; }'
                'static inline double dot(V3 a, V3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }'
                'static inline double nrm(V3 a) { return std::sqrt(dot(a, a)); }'
                ''
                'static inline V3 col3(const double* A, mwSize j) {'
                '    return {A[3 * j], A[3 * j + 1], A[3 * j + 2]};'
                '}'
                ''
                'static inline V3 Rt(const double* R, V3 v) {'
                '    return {'
                '        R[0] * v.x + R[1] * v.y + R[2] * v.z,'
                '        R[3] * v.x + R[4] * v.y + R[5] * v.z,'
                '        R[6] * v.x + R[7] * v.y + R[8] * v.z};'
                '}'
                ''
                'static double pointLevel(V3 p, V3 c, const double* R, V3 a) {'
                '    V3 z = Rt(R, sub(p, c));'
                '    z.x /= a.x; z.y /= a.y; z.z /= a.z;'
                '    return nrm(z);'
                '}'
                ''
                'static double segmentLevel(V3 p0, V3 p1, V3 c, const double* R, V3 a) {'
                '    V3 z0 = Rt(R, sub(p0, c));'
                '    V3 z1 = Rt(R, sub(p1, c));'
                '    z0.x /= a.x; z0.y /= a.y; z0.z /= a.z;'
                '    z1.x /= a.x; z1.y /= a.y; z1.z /= a.z;'
                '    V3 d = sub(z1, z0);'
                '    double den = dot(d, d);'
                '    if (den <= std::numeric_limits<double>::epsilon()) return nrm(z0);'
                '    double u = -dot(z0, d) / den;'
                '    u = std::max(0.0, std::min(1.0, u));'
                '    return nrm(add(z0, mul(u, d)));'
                '}'
                ''
                'void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {'
                '    if (nrhs != 15) mexErrMsgIdAndTxt("TRIGGER:MEX", "Expected 15 inputs.");'
                '    if (nlhs > 8)   mexErrMsgIdAndTxt("TRIGGER:MEX", "At most 8 outputs.");'
                ''
                '    mwSize i0m = (mwSize)mxGetScalar(prhs[0]);'
                '    double H   = mxGetScalar(prhs[1]);'
                '    double dt  = mxGetScalar(prhs[2]);'
                '    double rangeMax = mxGetScalar(prhs[3]);'
                '    double L   = mxGetScalar(prhs[4]);'
                ''
                '    const mxArray* pLmx = prhs[5];'
                '    const mxArray* qmx  = prhs[6];'
                '    const mxArray* Cmx  = prhs[7];'
                '    const mxArray* rbmx = prhs[8];'
                '    const mxArray* rhmx = prhs[9];'
                '    const mxArray* Rmx  = prhs[10];'
                '    const mxArray* Apmx = prhs[11];'
                '    const mxArray* Aqmx = prhs[12];'
                '    const mxArray* Acmx = prhs[13];'
                '    const mxArray* Idmx = prhs[14];'
                ''
                '    mwSize N = mxGetN(pLmx);'
                '    mwSize M = mxGetN(Cmx);'
                ''
                '    const double* pL = mxGetPr(pLmx);'
                '    const double* q  = mxGetPr(qmx);'
                '    const double* C  = mxGetPr(Cmx);'
                '    const double* rb = mxGetPr(rbmx);'
                '    const double* Rf = mxGetPr(Rmx);'
                '    const double* Ap = mxGetPr(Apmx);'
                '    const double* Aq = mxGetPr(Aqmx);'
                '    const double* Ac = mxGetPr(Acmx);'
                '    const double* ids = mxGetPr(Idmx);'
                ''
                '    mwSize i0 = i0m - 1;'
                '    V3 pL0 = col3(pL, i0);'
                '    V3 q0  = col3(q, i0);'
                '    V3 pQ0 = sub(pL0, mul(L, q0));'
                ''
                '    unsigned char* active = (unsigned char*)mxCalloc(M, sizeof(unsigned char));'
                '    mxArray* distsMx = mxCreateDoubleMatrix(1, M, mxREAL);'
                '    double* dists = mxGetPr(distsMx);'
                ''
                '    bool any = false;'
                '    mwSize visibleCount = 0;'
                '    double minDist = std::numeric_limits<double>::infinity();'
                '    double minId = mxGetNaN();'
                ''
                '    for (mwSize i = 0; i < M; i++) {'
                '        V3 c = {C[3 * i], C[3 * i + 1], C[3 * i + 2]};'
                '        double d_center = nrm(sub(pQ0, c));'
                '        double d_surf = std::max(0.0, d_center - rb[i]);'
                '        dists[i] = d_surf;'
                ''
                '        if (d_surf <= rangeMax) {'
                '            active[i] = 1;'
                '            any = true;'
                '            visibleCount++;'
                '            if (d_surf < minDist) {'
                '                minDist = d_surf;'
                '                minId = ids[i];'
                '            }'
                '        }'
                '    }'
                ''
                '    bool found = false;'
                '    double jhit = mxGetNaN();'
                '    double oid  = mxGetNaN();'
                '    double comp = mxGetNaN();'
                ''
                '    if (any) {'
                '        mwSize nH = (mwSize)std::floor(H / dt + 1e-12);'
                '        mwSize i1 = std::min(N - 1, i0 + nH);'
                ''
                '        for (mwSize j = i0; j <= i1; j++) {'
                '            V3 pLj = col3(pL, j);'
                '            V3 qj  = col3(q, j);'
                '            V3 pQj = sub(pLj, mul(L, qj));'
                ''
                '            double best = std::numeric_limits<double>::infinity();'
                '            double bestComp = mxGetNaN();'
                '            double bestOid  = mxGetNaN();'
                ''
                '            for (mwSize i = 0; i < M; i++) {'
                '                if (!active[i]) continue;'
                '                V3 c = {C[3 * i], C[3 * i + 1], C[3 * i + 2]};'
                '                const double* R = Rf + 9 * i;'
                '                V3 ap = {Ap[3 * i], Ap[3 * i + 1], Ap[3 * i + 2]};'
                '                V3 aq = {Aq[3 * i], Aq[3 * i + 1], Aq[3 * i + 2]};'
                '                V3 ac = {Ac[3 * i], Ac[3 * i + 1], Ac[3 * i + 2]};'
                ''
                '                double sp = pointLevel(pLj, c, R, ap) - 1.0;'
                '                if (sp < best) { best = sp; bestComp = 1; bestOid = ids[i]; }'
                ''
                '                double sq = pointLevel(pQj, c, R, aq) - 1.0;'
                '                if (sq < best) { best = sq; bestComp = 2; bestOid = ids[i]; }'
                ''
                '                double sc = segmentLevel(pLj, pQj, c, R, ac) - 1.0;'
                '                if (sc < best) { best = sc; bestComp = 3; bestOid = ids[i]; }'
                '            }'
                ''
                '            if (best <= 0.0) {'
                '                found = true;'
                '                jhit = (double)(j + 1);'
                '                oid  = bestOid;'
                '                comp = bestComp;'
                '                break;'
                '            }'
                '        }'
                '    }'
                ''
                '    mxFree(active);'
                ''
                '    if (nlhs > 0) plhs[0] = mxCreateLogicalScalar(found);'
                '    if (nlhs > 1) plhs[1] = mxCreateDoubleScalar(jhit);'
                '    if (nlhs > 2) plhs[2] = mxCreateDoubleScalar(oid);'
                '    if (nlhs > 3) plhs[3] = mxCreateDoubleScalar(comp);'
                '    if (nlhs > 4) plhs[4] = mxCreateDoubleScalar((double)visibleCount);'
                '    if (nlhs > 5) plhs[5] = mxCreateDoubleScalar(minDist);'
                '    if (nlhs > 6) plhs[6] = mxCreateDoubleScalar(minId);'
                '    if (nlhs > 7) plhs[7] = distsMx;'
                '}'
            };

            for k = 1:numel(lines)
                fprintf(fid, '%s\n', lines{k});
            end
        end
    end
end

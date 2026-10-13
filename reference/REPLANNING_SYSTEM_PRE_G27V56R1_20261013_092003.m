classdef REPLANNING_SYSTEM < handle
% =========================================================================
% REPLANNING_SYSTEM
%
% 【概要】
%   索吊り下げ型ドローン (Quadrotor with Suspended Load) 向け
%   G26 将来予測衝突回避トリガー統合クラス（本番運用最適化版・完全解説付き）。
%
% 【システム構成・運動学モデル】
%   - 状態定義:
%       pL : 牽引荷物 (Payload) の位置ベクトル [x; y; z] (3×1)
%       pQ : ドローン機体 (UAV Quadrotor) の位置ベクトル [x; y; z] (3×1)
%       q  : 機体から荷物への索姿勢単位ベクトル (UAV -> Load, ||q|| = 1)
%       L  : 索長 (cableL)
%     幾何学的拘束関係:
%       pQ = pL - L * q
%
% 【G27-V54 fail-closed GENERAL C6 POSITION + INDEPENDENT C6 YAW contract】
%   - External output requirement remains position C6 + yaw C6.
%   - Position safety is certified independently by the unchanged continuous
%     geometry / dynamics / HLC / backup / finite-sensing authorities.
%   - Yaw is an independent C6 reference.  Exact horizontal travel-direction
%     alignment is NOT a production safety gate.
%   - The generator may prefer forward-looking yaw through a mathematical
%     optimization objective, but that preference cannot create or revoke a
%     safety certificate.
%   - Zero horizontal speed requires no atan2(0,0), speed epsilon, or
%     yaw-freeze safety branch because yaw C6 exists independently.
%   - path_position_c7 remains an internal generator/representation
%     regularity contract.
%   - Sensor-direction/FoV certification is NOT a mandatory yaw certificate.
%     The existing simulated-sensor finite-sensing certificate remains
%     separate and unchanged.
%   - Time law must be C6, nondecreasing, and join a constant terminal hold
%     with derivatives 1..6 equal to zero.
%   - Every production candidate certificate is bound to the next REPAIRED
%     reference version and the current live dependency/source signature.
%   - Test-only certificate tokens are explicitly rejected.
%   - hard_modify_deadline_s is fail-closed at <= 4 ms.
%   - MEX is an acceleration mechanism only; no MEX result bypasses the
%     certificate/backup/atomic-commit authority.
%
% 【G27-V39R2 initialization-order correction】
%   REPLANNING_SYSTEM is instantiated before agent.controller.hlc_suspended
%   exists in run_flight_headless.  Command limits are therefore certified
%   from HLC_SUSPENDED_LOAD.m source during construction, then the actual
%   live controller handle is bound/verified later and is mandatory by Flight.
%
% 【役割と責務】
%   - 本クラスは制御周期 (dt) ごとに「球状センサによる障害物検知」および
%     「将来予測ホライズン H [s] 内の衝突予測トリガー判定」を担当します。
%   - 将来衝突が予測された場合、need_replan = true とし、
%     衝突余裕時間 TTC (Time-To-Collision)、衝突予測時刻、衝突障害物ID、
%     接触予測部位 (PAYLOAD / UAV / CABLE) を特定して制御器・リプランナーへ通知します。
%   - 本クラス自身は軌道候補を生成しません。後続 generator が作る候補に対し、
%     G27-V54 以降は position C6 + independent yaw C6/連続時間安全/動力学/backup/
%     4-ms deadline/dependency binding を fail-closed に検証し、原子的 commit を担当します。
%
% =========================================================================
% 【極めて重要な設計思想：実行時間スパイク（Latency Spike）の完全排除】
% =========================================================================
% 実機ドローン制御やミリ秒単位のリアルタイムシミュレーションにおいて、
% 制御ループ内の実行時間ピーク（スパイク）は制御破綻や通信タイムアウトの原因となります。
% 本クラスでは、以下の5大最適化手法によりスパイクを原理的に根絶しています：
%
% 1. 公称軌道テーブルの一括事前生成 (Precomputed Nominal Table)
%    - 飛行開始前（Setup / Armingフェーズ）に、飛行全区間 [t0, te + H] の
%      公称荷物位置 pL(t) および公称索姿勢 q(t) を一括計算して行列テーブルへ展開。
%    - 飛行中 (Flightフェーズ) は、時刻 t_now に対応する配列インデックスを O(1) で
%      算出し、連続メモリブロックをスライス参照するのみ。
%    - これにより、定常制御ループ内での微分方程式反復評価や関数呼び出しオーバーヘッド、
%      一時メモリ割り当てを完全にゼロ化。
%
% 2. 離陸前コールドパス完全事前燃焼 (Pre-motion Prime)
%    - 機体が動く前の静止段階で、オンライン判定ルーチン (warmup_online_path) を実行。
%    - 障害物マスクの全組み合わせ (0個〜M個)、強制衝突パス (現在接触 j=1、将来衝突 j=2)、
%      全衝突部位コード (1: PAYLOAD, 2: UAV, 3: CABLE) のデコード処理、
%      さらに本番と同一引数による do() の実呼び出しを 12 回反復実行 (setup_exact_do_prime)。
%    - これにより、MATLAB JIT コンパイラの初回コンパイル、動的メソッドディスパッチ、
%      CPU命令キャッシュミス、OSメモリページフォールトを離陸前に全て消化。
%
% 3. 状態ロガーの固定長・直接上書き (Fixed-field In-place Logging)
%    - 毎周期の clear -> reallocate や、可変長 struct 配列 (detected_obstacles 等) の
%      動的生成をホットループから完全撤廃。
%    - 事前定義済みの固定長スカラーおよび固定長ベクトルフィールド (min_dist_point,
%      trigger_ttc, trigger_pL_collision 等) に対し、インプレース代入で直接更新。
%    - MATLAB のガベージコレクション (GC) 誘発とヒープ断片化を防止。
%
% 4. 時間クエリの非破壊メモリ保護 (Memory-Isolated Time Query)
%    - time が TIME < handle の場合、参照クラス呼び出し時に本体の time.t を破壊しないよう
%      独立した軽量時間構造体・オブジェクトを生成して引数渡し。
%    - 意図しないシミュレーション時刻ジャンプやメモリ再割り当てを防止。
%
% 5. C++ MEX 高速カーネルによる SIMD 級判定 (C++ MEX Kernel)
%    - 将来区間の全 N サンプリング点および全アクティブ障害物に対する楕円体包囲レベルセット計算を、
%      C++ MEX (g26_future_trigger_kernel_mex) により一括並列実行。
%    - 起動時にソースコード改変を検知して自動コンパイルし、10 回の実パラメータウォームアップを実施。
%
% =========================================================================
% 【コンソールログの意味と表示切り替え】
% =========================================================================
%   ENABLE_ALL_LOG : ログ表示マスタースイッチ (0: SILENT, 1: 全ログ表示)
%   LOG_SENSOR     : センサ検知ログ表示スイッチ (20m球内に障害物を捕捉した周期で表示)
%   LOG_TRIGGER    : 将来衝突検知ログ表示スイッチ (将来ホライズン内で衝突を予測した周期で表示)
%
%   各ログの意味:
%   - [REPLANNING_SYSTEM] initialized
%       初期化完了ログ。センサ検知範囲、予測ホライズン H、周期 dt、安全マージン、初期ウォームアップ時間を表示。
%   - [REPLANNING_SYSTEM] nominal table ready
%       公称軌道事前計算テーブル生成完了ログ。生成区間 [t0, tend]、サンプリング点数、生成所要時間を表示。
%   - [REPLANNING_SYSTEM] pre-motion prime ready
%       離陸前 JIT プライム完了ログ。網羅テストしたマスク数、強制衝突ケース数、事前 do() 実行回数、所要時間を表示。
%   - [SENSOR RAW 20.0m] t=... | count=... | nearest=... | d=...
%       球状センサ検知ログ。現在時刻 t、検知範囲内の障害物数 count、最寄障害物 ID、最寄真形状表面距離 d [m] を表示。
%   - [FUTURE HIT] t=... | TTC=... | obs=... | comp=...
%       将来衝突予測検知ログ。現在時刻 t、衝突までの余裕時間 TTC [s]、衝突障害物 ID、接触予測部位 comp を表示。
% =========================================================================


    properties (Access = public)
        % -----------------------------------------------------------------
        % 【コンソールログ表示・プロファイリング制御スイッチ】
        % -----------------------------------------------------------------
        ENABLE_ALL_LOG   = 1;  % ログ表示マスタースイッチ (1: 有効, 0: 無効)
        LOG_SENSOR       = 0;  % 球状センサ障害物捕捉ログの個別スイッチ (20m内に障害物検知時)
        LOG_TRIGGER      = 0;  % 将来衝突検知ログの個別スイッチ (衝突予測トリガー発火時)
        ENABLE_PROFILING = 1;  % 各モジュール実行時間計測スイッチ (通常運用時はスパイク防止のため0推奨)

        % -----------------------------------------------------------------
        % 【離陸前コールドパス完全事前燃焼 (Pre-motion Prime) 設定】
        % -----------------------------------------------------------------
        ONLINE_PRIME_CALLS           = 3;   % 各合成テストケース (マスク・衝突パターン) の反復実行回数
        SETUP_EXACT_DO_PRIME_CALLS   = 12;  % 本番引数による do() の事前呼び出し回数 (JIT・キャッシュ完全温め)
        REQUIRE_PRIME_BEFORE_TAKEOFF = 1;   % 離陸開始前に完全プライム完了を必須とする安全ガードフラグ
        takeoff_end_s                = NaN; % 離陸完了時刻 [s] (自動解決または手動設定)

        % -----------------------------------------------------------------
        % 【基本オブジェクト参照】
        % -----------------------------------------------------------------
        self       % 所属エージェント本体 (agent)
        base_ref   % ベース公称軌道オブジェクト
        result     % 出力結果構造体 (result.state に固定長フィールドを格納)

        % -----------------------------------------------------------------
        % 【トリガー判定幾何パラメータ】
        % -----------------------------------------------------------------
        trigger_dist  = 20.0;   % 球状センサ検知半径 [m] (raw-shape 表面距離ゲート)
        horizon_s     = 4.0;    % 衝突予測ホライズン H [s]
        prediction_dt = 0.025;  % 予測および制御周期 dt [s] (本体 plant/time から厳格取得)
        d_margin      = 0.0;    % 追加システム安全マージン [m] (物理保護半径とは独立、デフォルト 0.0)

        % -----------------------------------------------------------------
        % 【物理保護半径 [m]】
        % -----------------------------------------------------------------
        r_drone = 0.5;  % ドローン機体の等価保護球半径
        r_load  = 0.5;  % 牽引荷物の等価保護球半径
        r_cable = 0.5;  % 索セグメントの等価保護球半径 (r_cable_sphere と同等)

        % -----------------------------------------------------------------
        % 【物理定数・状態推定パラメータ】
        % -----------------------------------------------------------------
        gravity  = 9.81;  % 重力加速度 [m/s^2]
        L_cable  = 2.0;   % 索長 [m]
        loadmass = 0.0;   % 牽引物質量 [kg] (推定器 self.estimator.result.state.mL から必須取得)

        % -----------------------------------------------------------------
        % 【環境障害物データ・楕円パケット】
        % -----------------------------------------------------------------
        static_obstacle_map = [];  % ENVIRONMENT から取得した静的障害物配列
        obstacle_packet     = [];  % MEX 用に事前計算された保守的外接楕円体パラメータパケット

        % -----------------------------------------------------------------
        % 【将来参照軌道パケット・ローリングキャッシュ】
        % -----------------------------------------------------------------
        reference_packet_fn   = [];      % 外部指定の将来軌道関数 @(tvec)
        reference_cache       = struct(); % フォールバック用ローリングキャッシュ
        reference_cache_valid = false;   % キャッシュ有効フラグ

        % -----------------------------------------------------------------
        % 【公称軌道事前計算テーブル (Precomputed Nominal Table)】
        % -----------------------------------------------------------------
        % ※ 飛行全区間の pL(t), q(t) を離陸前に一括生成して保持。定常周期では
        %    O(1) の直接スライス参照のみを行うため、計算スパイクが原理的に発生しません。
        nominal_table_valid      = false;  % テーブル有効フラグ
        nominal_table_t0         = NaN;    % テーブル開始時刻 [s]
        nominal_table_tend       = NaN;    % テーブル終了時刻 [s]
        nominal_table_dt         = NaN;    % テーブル時間刻み [s]
        nominal_table_pL         = [];     % 事前計算済み荷物位置行列 (3×K)
        nominal_table_q          = [];     % 事前計算済み索姿勢ベクトル行列 (3×K)
        nominal_table_build_ms   = NaN;    % テーブル事前生成所要時間 [ms]
        nominal_table_eval_count = 0;      % 飛行中のテーブル参照カウント

        % -----------------------------------------------------------------
        % 【C++ MEX 高速カーネル設定】
        % -----------------------------------------------------------------
        mex_name            = 'g26_future_trigger_kernel_mex'; % MEX 関数名
        mex_ready           = false;                           % コンパイル済みフラグ
        mex_source_revision = 'G26_FUTURE_TRIGGER_R2_20261012'; % ソースコードリビジョン識別子

        % -----------------------------------------------------------------
        % 【初期ウォームアップ・事前プライム実行状態】
        % -----------------------------------------------------------------
        warmup_calls      = 10;     % 起動時 MEX ウォームアップ試行回数
        warmup_done       = false;  % 初期ウォームアップ完了フラグ
        warmup_runtime_ms = NaN;    % 初期ウォームアップ所要時間 [ms]

        online_prime_done          = false;  % オンラインパステスト完了フラグ
        online_prime_runtime_ms    = NaN;    % オンラインパステスト所要時間 [ms]
        online_prime_variant_count = 0;      % テストしたマスク組み合わせ総数
        online_prime_hit_cases     = 0;      % テストした強制衝突ケース総数

        setup_exact_do_prime_active     = false;  % do() 事前プライム実行中フラグ
        setup_exact_do_prime_done       = false;  % do() 事前プライム完了フラグ
        setup_exact_do_prime_runtime_ms = NaN;    % do() 事前プライム所要時間 [ms]
        setup_exact_do_prime_calls      = 0;      % do() 事前呼び出し実行回数
        setup_exact_force_hit           = false;  % プライム用強制衝突テストフラグ
        setup_exact_force_hit_calls     = 0;      % プライム用強制衝突テスト実行回数

        % -----------------------------------------------------------------
        % 【将来 C6 軌道修復連携用フック】
        % -----------------------------------------------------------------
        active_reference_mode    = 'NOMINAL';  % 参照モード ('NOMINAL' または 'REPAIRED')
        active_reference_version = 0;          % 軌道バージョン番号

        % -----------------------------------------------------------------
        % 【連続時間 future-trigger 用 version-bound 速度証明】
        % -----------------------------------------------------------------
        % センサ raw_surface_distance には使用しない。
        % MEX 用外接楕円体の intersample sweep inflation のみに使用する。
        active_speed_certificate = struct();
        trigger_sweep_inflation  = struct();

        % -----------------------------------------------------------------
        % 【G27-V38 dependency-aware runtime certificate manager】
        % -----------------------------------------------------------------
        % 4 ms is a design requirement and remains externally configurable.
        hard_modify_deadline_s      = 4.0e-3;

        sensor_horizon_certificate  = struct();
        stop_geometry_4ms_certificate = struct();
        command_limit_certificate   = struct();
        actuator_dynamics_certificate = struct();
        hlc_robust_certificate      = struct();

        runtime_dependency_signature = struct();
        runtime_source_signature     = struct();
        certificate_stale            = struct();
        dependency_refresh_count      = 0;
        mass_certificate_rebuild_count = 0;

        % -----------------------------------------------------------------
        % 【G27-V39R1 current repaired-reference backend】
        % -----------------------------------------------------------------
        % Old reference-folder planners are NOT used here.  A repaired
        % trajectory is accepted only through commit_repaired_reference()
        % after all required certificates are already closed.
        active_repaired_reference   = struct();
        active_repaired_certificates = struct();
        replan_commit_count          = 0;
        last_replan_commit_time      = NaN;
        last_replan_status           = struct();

        % 互換性保持用プレースホルダー
        current_bspline = [];

        % -----------------------------------------------------------------
        % 【最小プロファイリング情報構造体 (ENABLE_PROFILING=1 時のみ更新)】
        % -----------------------------------------------------------------
        profiling = struct( ...
            't_base_ref_ms',            NaN, ... % 公称軌道計算時間 [ms]
            't_sensor_ms',              NaN, ... % センサ検知ゲート時間 [ms]
            't_reference_packet_ms',    NaN, ... % 将来参照パケット取得時間 [ms]
            't_kernel_ms',              NaN, ... % C++ MEX カーネル実行時間 [ms]
            't_trigger_ms',             NaN, ... % トリガー判定全体時間 [ms]
            't_total_ms',               NaN, ... % 周期内総処理時間 [ms]
            't_startup_warmup_ms',      NaN, ... % 初期ウォームアップ時間 [ms]
            't_nominal_table_build_ms', NaN, ... % 公称テーブル生成時間 [ms]
            't_setup_prime_ms',         NaN);    % 離陸前プライム時間 [ms]
    end

    properties (Dependent)
        r_cable_sphere
    end


    methods
        function val = get.r_cable_sphere(obj)
            val = obj.r_cable;
        end

        function obj = set.r_cable_sphere(obj,val)
            obj.r_cable = val;
        end

        function obj = set.horizon_s(obj,val)
            assert(isnumeric(val) && isscalar(val) && isfinite(val) && val>0, ...
                '[REPLANNER ERROR] horizon_s must be a finite positive scalar.');
            if ~isempty(obj.prediction_dt) && obj.prediction_dt>0
                ratio=val/obj.prediction_dt;
                assert(abs(ratio-round(ratio)) <= 1e-9*max(1,abs(ratio)), ...
                    '[REPLANNER ERROR] horizon_s/prediction_dt must be integer.');
            end
            obj.horizon_s=val;
            obj.reference_cache_valid=false;
            obj.nominal_table_valid=false;
            obj.online_prime_done=false;
            obj.setup_exact_do_prime_done=false;
        end
    end



    methods (Access = public)


        function set_horizon(obj,H)
            obj.horizon_s=H;
        end


        function install_active_speed_certificate(obj,cert)
            % =============================================================
            % 将来 REPAIRED 軌道 commit 用の共通インターフェース。
            %
            % current active_reference_mode/version と完全一致する
            % certified speed certificate だけを受理し、future-trigger
            % obstacle packet を同じ version で原子的に再構築する。
            %
            % raw sensor geometry は変更しない。
            % =============================================================
            obj.validate_active_speed_certificate(cert);
            obj.active_speed_certificate=cert;
            obj.trigger_sweep_inflation= ...
                obj.build_trigger_sweep_inflation(cert);

            if ~isempty(obj.static_obstacle_map)
                obj.obstacle_packet= ...
                    obj.prepare_obstacle_packet(obj.static_obstacle_map);
            end

            obj.sensor_horizon_certificate= ...
                obj.build_sensor_horizon_certificate();

            if strcmpi(char(obj.active_reference_mode),'NOMINAL')
                obj.stop_geometry_4ms_certificate= ...
                    obj.build_stop_geometry_4ms_certificate();
                obj.actuator_dynamics_certificate= ...
                    obj.build_current_actuator_dynamics_certificate();
            else
                obj.stop_geometry_4ms_certificate=struct( ...
                    'certified',false, ...
                    'reason','REPAIRED stop geometry certificate not installed');
                obj.actuator_dynamics_certificate=struct( ...
                    'certified',false, ...
                    'reason','REPAIRED actuator dynamics certificate not installed');
            end

            obj.runtime_dependency_signature= ...
                obj.capture_runtime_dependency_signature_fast();
        end



        function refresh_runtime_certificates(obj)
            % =============================================================
            % Explicit heavy/source-bound refresh entry point.
            %
            % Intended for setup / configuration change / trajectory commit.
            % It is NOT called every Flight cycle.
            % =============================================================
            if strcmpi(char(obj.active_reference_mode),'NOMINAL')
                obj.active_speed_certificate= ...
                    obj.build_nominal_active_speed_certificate();
            else
                obj.validate_active_speed_certificate(obj.active_speed_certificate);
            end

            obj.trigger_sweep_inflation= ...
                obj.build_trigger_sweep_inflation(obj.active_speed_certificate);

            obj.command_limit_certificate= ...
                obj.build_active_command_limit_certificate();
            obj.verify_live_controller_binding(false);

            obj.sensor_horizon_certificate= ...
                obj.build_sensor_horizon_certificate();

            obj.loadmass=obj.get_load_mass();

            if strcmpi(char(obj.active_reference_mode),'NOMINAL')
                obj.stop_geometry_4ms_certificate= ...
                    obj.build_stop_geometry_4ms_certificate();
                obj.actuator_dynamics_certificate= ...
                    obj.build_current_actuator_dynamics_certificate();
            else
                obj.stop_geometry_4ms_certificate=struct( ...
                    'certified',false, ...
                    'reason','REPAIRED stop geometry certificate must be installed at commit');
                obj.actuator_dynamics_certificate=struct( ...
                    'certified',false, ...
                    'reason','REPAIRED actuator dynamics certificate must be installed at commit');
            end

            obj.hlc_robust_certificate=struct( ...
                'certified',false, ...
                'stale',true, ...
                'reason','HLC robust tracking-error certificate not installed yet');

            if ~isempty(obj.static_obstacle_map)
                obj.obstacle_packet= ...
                    obj.prepare_obstacle_packet(obj.static_obstacle_map);
            end

            obj.runtime_source_signature= ...
                obj.capture_runtime_source_signature();
            obj.runtime_dependency_signature= ...
                obj.capture_runtime_dependency_signature_fast();

            obj.certificate_stale=obj.empty_certificate_stale_state();
            obj.certificate_stale.HLC_ROBUST_CERT=true;
            obj.dependency_refresh_count=obj.dependency_refresh_count+1;
        end


        function status = get_runtime_certificate_status(obj)
            status=struct();
            status.signature=obj.runtime_dependency_signature;
            status.source_signature=obj.runtime_source_signature;
            status.stale=obj.certificate_stale;
            status.speed_trigger=obj.active_speed_certificate;
            status.trigger_sweep=obj.trigger_sweep_inflation;
            status.sensor_horizon=obj.sensor_horizon_certificate;
            status.stop_geometry_4ms=obj.stop_geometry_4ms_certificate;
            status.command_limit=obj.command_limit_certificate;
            if isfield(obj.command_limit_certificate,'live_controller_verified')
                status.live_controller_verified= ...
                    logical(obj.command_limit_certificate.live_controller_verified);
            else
                status.live_controller_verified=false;
            end
            status.actuator_dynamics=obj.actuator_dynamics_certificate;
            status.hlc_robust=obj.hlc_robust_certificate;
            status.dependency_refresh_count=obj.dependency_refresh_count;
            status.mass_certificate_rebuild_count= ...
                obj.mass_certificate_rebuild_count;
        end



        function sig = get_next_repaired_dependency_signature(obj,eval_fn)
            % =============================================================
            % Public read-only binding helper for external candidate
            % generators.  No certificate is created here.
            %
            % The returned signature is the exact live/source dependency
            % snapshot that a candidate targeting the NEXT REPAIRED version
            % must carry.  This prevents generator-side hard-coded physical
            % parameters and stale certificate reuse.
            % =============================================================
            assert(isa(eval_fn,'function_handle'), ...
                '[REPLANNER ERROR] eval_fn must be a function handle.');
            obj.assert_hard_modify_deadline_limit();
            targetVersion=double(obj.active_reference_version)+1;
            sig=obj.capture_candidate_dependency_signature( ...
                eval_fn,targetVersion);
        end


        function commit_repaired_reference(obj,candidate,t_commit)
            % =============================================================
            % Atomic REPAIRED-reference commit interface.
            %
            % This method DOES NOT generate a trajectory.  It only accepts a
            % candidate already certified by the current planner/theory.
            % Old reference-folder planners are intentionally not called.
            %
            % Required candidate fields:
            %   certified, backend, eval_fn, valid_from, valid_until,
            %   speed_certificate,
            %   smoothness_certificate,
            %   geometry_certificate,
            %   dynamics_certificate,
            %   hlc_robust_certificate,
            %   backup_certificate,
            %   position_c6_certificate,
            %   yaw_c6_certificate,
            %   yaw_policy='C6_INDEPENDENT',
            %   path_parameterization,
            %   path_position_c7_certificate,
            %   time_law_c6_certificate,
            %   time_law_monotonic_certificate,
            %   terminal_time_law_hold_certificate,
            %   finite_sensing_certificate,
            %   compute_deadline_certificate,
            %   dependency_certificate.
            %
            % The candidate is rejected unless every certificate is true and
            % the reference remains valid over the whole future-trigger
            % horizon from commit.
            % =============================================================
            if nargin<3
                error('[REPLANNER ERROR] commit_repaired_reference requires t_commit.');
            end

            obj.assert_hard_modify_deadline_limit();
            obj.validate_repaired_candidate(candidate,double(t_commit));

            newVersion=double(obj.active_reference_version)+1;
            sp=candidate.speed_certificate;

            assert(strcmpi(char(sp.reference_mode),'REPAIRED') && ...
                double(sp.reference_version)==newVersion, ...
                ['[REPLANNER ERROR] repaired speed certificate must already ' ...
                 'target the next REPAIRED reference version.']);

            % Save complete rollback state first.
            old=struct();
            old.mode=obj.active_reference_mode;
            old.version=obj.active_reference_version;
            old.reference=obj.active_repaired_reference;
            old.certs=obj.active_repaired_certificates;
            old.speed=obj.active_speed_certificate;
            old.sweep=obj.trigger_sweep_inflation;
            old.packet=obj.obstacle_packet;
            old.sensor=obj.sensor_horizon_certificate;
            old.stop=obj.stop_geometry_4ms_certificate;
            old.act=obj.actuator_dynamics_certificate;
            old.hlc=obj.hlc_robust_certificate;
            old.dep=obj.runtime_dependency_signature;
            old.stale=obj.certificate_stale;
            old.bspline=obj.current_bspline;
            old.count=obj.replan_commit_count;
            old.time=obj.last_replan_commit_time;

            try
                R=struct();
                R.valid=true;
                R.backend=char(candidate.backend);
                R.eval_fn=candidate.eval_fn;
                R.valid_from=double(candidate.valid_from);
                R.valid_until=double(candidate.valid_until);
                R.reference_version=newVersion;
                R.source='CURRENT_REPLANNING_SYSTEM_CERTIFIED_COMMIT';
                R.yaw_policy=char(candidate.yaw_policy);
                R.path_parameterization=candidate.path_parameterization;
                if isfield(candidate,'rejoin_time')
                    R.rejoin_time=double(candidate.rejoin_time);
                else
                    R.rejoin_time=NaN;
                end
                if isfield(candidate,'representation')
                    R.representation=candidate.representation;
                end

                obj.active_repaired_reference=R;
                obj.active_repaired_certificates=struct( ...
                    'smoothness',candidate.smoothness_certificate, ...
                    'geometry',candidate.geometry_certificate, ...
                    'dynamics',candidate.dynamics_certificate, ...
                    'hlc_robust',candidate.hlc_robust_certificate, ...
                    'backup',candidate.backup_certificate, ...
                    'position_c6',candidate.position_c6_certificate, ...
                    'yaw_c6',candidate.yaw_c6_certificate, ...
                    'path_position_c7',candidate.path_position_c7_certificate, ...
                    'time_law_c6',candidate.time_law_c6_certificate, ...
                    'time_law_monotonic',candidate.time_law_monotonic_certificate, ...
                    'terminal_time_law_hold',candidate.terminal_time_law_hold_certificate, ...
                    'finite_sensing',candidate.finite_sensing_certificate, ...
                    'compute_deadline',candidate.compute_deadline_certificate, ...
                    'dependency',candidate.dependency_certificate);

                obj.active_reference_mode='REPAIRED';
                obj.active_reference_version=newVersion;

                % Version-bound speed/sweep and obstacle packet are rebuilt
                % only after the trajectory/certificates are staged.
                obj.active_speed_certificate=sp;
                obj.validate_active_speed_certificate(obj.active_speed_certificate);
                obj.trigger_sweep_inflation= ...
                    obj.build_trigger_sweep_inflation(obj.active_speed_certificate);

                if ~isempty(obj.static_obstacle_map)
                    obj.obstacle_packet= ...
                        obj.prepare_obstacle_packet(obj.static_obstacle_map);
                end

                obj.sensor_horizon_certificate= ...
                    obj.build_sensor_horizon_certificate();

                % For REPAIRED mode these fields represent the certified
                % backup attached to this active repaired candidate.
                B=candidate.backup_certificate;
                assert(isfield(B,'stop_geometry_certificate') && ...
                    isfield(B,'actuator_dynamics_certificate') && ...
                    isstruct(B.stop_geometry_certificate) && ...
                    isstruct(B.actuator_dynamics_certificate) && ...
                    isfield(B.stop_geometry_certificate,'certified') && ...
                    logical(B.stop_geometry_certificate.certified) && ...
                    isfield(B.actuator_dynamics_certificate,'certified') && ...
                    logical(B.actuator_dynamics_certificate.certified), ...
                    '[REPLANNER ERROR] repaired backup certificate is incomplete.');

                obj.stop_geometry_4ms_certificate= ...
                    B.stop_geometry_certificate;
                obj.actuator_dynamics_certificate= ...
                    B.actuator_dynamics_certificate;
                obj.hlc_robust_certificate= ...
                    candidate.hlc_robust_certificate;

                obj.certificate_stale=obj.empty_certificate_stale_state();
                obj.runtime_dependency_signature= ...
                    obj.capture_runtime_dependency_signature_fast();

                if strcmpi(char(candidate.backend),'BSPLINE_D8_C7') && ...
                        isfield(candidate,'representation')
                    obj.current_bspline=candidate.representation;
                else
                    obj.current_bspline=[];
                end

                obj.replan_commit_count=obj.replan_commit_count+1;
                obj.last_replan_commit_time=double(t_commit);
                obj.last_replan_status=struct( ...
                    'requested',true, ...
                    'committed',true, ...
                    'reference_version',newVersion, ...
                    'backend',char(candidate.backend), ...
                    'commit_time',double(t_commit));

                % Final fail-closed verification after every field changed.
                obj.assert_trigger_packet_certificate_current();
                obj.assert_repaired_reference_current();

            catch ME
                obj.active_reference_mode=old.mode;
                obj.active_reference_version=old.version;
                obj.active_repaired_reference=old.reference;
                obj.active_repaired_certificates=old.certs;
                obj.active_speed_certificate=old.speed;
                obj.trigger_sweep_inflation=old.sweep;
                obj.obstacle_packet=old.packet;
                obj.sensor_horizon_certificate=old.sensor;
                obj.stop_geometry_4ms_certificate=old.stop;
                obj.actuator_dynamics_certificate=old.act;
                obj.hlc_robust_certificate=old.hlc;
                obj.runtime_dependency_signature=old.dep;
                obj.certificate_stale=old.stale;
                obj.current_bspline=old.bspline;
                obj.replan_commit_count=old.count;
                obj.last_replan_commit_time=old.time;

                obj.last_replan_status=struct( ...
                    'requested',true, ...
                    'committed',false, ...
                    'reason',ME.message, ...
                    'commit_time',double(t_commit));
                rethrow(ME);
            end
        end


        function obj = REPLANNING_SYSTEM(self,base_ref,opts)
            arguments
                self
                base_ref
                opts = struct()
            end

            obj.self=self;
            obj.base_ref=base_ref;

            if isfield(opts,'ENABLE_ALL_LOG'), obj.ENABLE_ALL_LOG=logical(opts.ENABLE_ALL_LOG); end
            if isfield(opts,'LOG_SENSOR'), obj.LOG_SENSOR=logical(opts.LOG_SENSOR); end
            if isfield(opts,'LOG_TRIGGER'), obj.LOG_TRIGGER=logical(opts.LOG_TRIGGER); end
            if isfield(opts,'ENABLE_PROFILING'), obj.ENABLE_PROFILING=logical(opts.ENABLE_PROFILING); end

            if isfield(opts,'sensor_range_m'), obj.trigger_dist=opts.sensor_range_m; end
            if isfield(opts,'trigger_dist'), obj.trigger_dist=opts.trigger_dist; end
            if isfield(opts,'horizon_s'), obj.horizon_s=opts.horizon_s; end
            if isfield(opts,'d_margin'), obj.d_margin=opts.d_margin; end

            if isfield(opts,'r_drone'), obj.r_drone=opts.r_drone; end
            if isfield(opts,'r_load'), obj.r_load=opts.r_load; end
            if isfield(opts,'r_cable'), obj.r_cable=opts.r_cable; end
            if isfield(opts,'R_cable_m'), obj.r_cable=opts.R_cable_m; end
            if isfield(opts,'cable'), obj.r_cable=opts.cable; end
            if isfield(opts,'r_cable_sphere'), obj.r_cable=opts.r_cable_sphere; end

            if isfield(opts,'warmup_calls'), obj.warmup_calls=opts.warmup_calls; end
            if isfield(opts,'ONLINE_PRIME_CALLS'), obj.ONLINE_PRIME_CALLS=opts.ONLINE_PRIME_CALLS; end
            if isfield(opts,'SETUP_EXACT_DO_PRIME_CALLS')
                obj.SETUP_EXACT_DO_PRIME_CALLS=opts.SETUP_EXACT_DO_PRIME_CALLS;
            end
            if isfield(opts,'REQUIRE_PRIME_BEFORE_TAKEOFF')
                obj.REQUIRE_PRIME_BEFORE_TAKEOFF=logical(opts.REQUIRE_PRIME_BEFORE_TAKEOFF);
            end
            if isfield(opts,'takeoff_end_s'), obj.takeoff_end_s=opts.takeoff_end_s; end
            if isfield(opts,'reference_packet_fn'), obj.reference_packet_fn=opts.reference_packet_fn; end
            if isfield(opts,'hard_modify_deadline_s')
                obj.hard_modify_deadline_s=opts.hard_modify_deadline_s;
            end

            assert(isnumeric(obj.hard_modify_deadline_s) && ...
                isscalar(obj.hard_modify_deadline_s) && ...
                isfinite(obj.hard_modify_deadline_s) && ...
                obj.hard_modify_deadline_s>0 && ...
                obj.hard_modify_deadline_s<=4.0e-3, ...
                ['[REPLANNER ERROR] hard_modify_deadline_s must be a positive ' ...
                 'finite scalar not exceeding the project hard maximum 4 ms.']);

            assert(isnumeric(obj.SETUP_EXACT_DO_PRIME_CALLS) && ...
                isscalar(obj.SETUP_EXACT_DO_PRIME_CALLS) && ...
                isfinite(obj.SETUP_EXACT_DO_PRIME_CALLS) && ...
                obj.SETUP_EXACT_DO_PRIME_CALLS>=1 && ...
                obj.SETUP_EXACT_DO_PRIME_CALLS==round(obj.SETUP_EXACT_DO_PRIME_CALLS), ...
                '[REPLANNER ERROR] SETUP_EXACT_DO_PRIME_CALLS must be a positive integer.');

            dt_val=[];
            if obj.has_prop_or_field(obj.self,'plant') && ~isempty(obj.self.plant)
                if obj.has_prop_or_field(obj.self.plant,'dt') && ~isempty(obj.self.plant.dt)
                    dt_val=obj.self.plant.dt;
                end
            end
            if isempty(dt_val) && obj.has_prop_or_field(obj.self,'parameter') && ~isempty(obj.self.parameter)
                try
                    dt_val=obj.self.parameter.get("dt");
                catch
                end
            end
            if isempty(dt_val) && isfield(opts,'dt'), dt_val=opts.dt; end

            assert(~isempty(dt_val) && isnumeric(dt_val) && isscalar(dt_val) && ...
                isfinite(dt_val) && dt_val>0, ...
                '[REPLANNER ERROR] control/prediction dt is unavailable.');

            if isfield(opts,'prediction_dt')
                obj.prediction_dt=opts.prediction_dt;
            else
                obj.prediction_dt=dt_val;
            end

            ratio=obj.horizon_s/obj.prediction_dt;
            assert(abs(ratio-round(ratio)) <= 1e-9*max(1,abs(ratio)), ...
                '[REPLANNER ERROR] horizon_s/prediction_dt must be integer.');

            obj.gravity=obj.self.parameter.get("gravity");
            obj.L_cable=obj.self.parameter.get("cableL");

            assert(isnumeric(obj.gravity) && isscalar(obj.gravity) && isfinite(obj.gravity) && obj.gravity>0, ...
                '[REPLANNER ERROR] invalid gravity.');
            assert(isnumeric(obj.L_cable) && isscalar(obj.L_cable) && isfinite(obj.L_cable) && obj.L_cable>0, ...
                '[REPLANNER ERROR] invalid cableL.');
            assert(isnumeric(obj.trigger_dist) && isscalar(obj.trigger_dist) && isfinite(obj.trigger_dist) && obj.trigger_dist>0, ...
                '[REPLANNER ERROR] invalid trigger_dist.');
            assert(isnumeric(obj.d_margin) && isscalar(obj.d_margin) && isfinite(obj.d_margin) && obj.d_margin>=0, ...
                '[REPLANNER ERROR] invalid d_margin.');
            assert(obj.r_drone>0 && obj.r_load>0 && obj.r_cable>0, ...
                '[REPLANNER ERROR] physical protection radii must be positive.');

            obj.loadmass=obj.get_load_mass();
            obj.result=base_ref.result;

            % -------------------------------------------------------------
            % Current NOMINAL/version=0 の連続時間 speed certificate を
            % base_ref.func から厳密生成する。サンプル max は使わない。
            % -------------------------------------------------------------
            obj.active_speed_certificate= ...
                obj.build_nominal_active_speed_certificate();
            obj.trigger_sweep_inflation= ...
                obj.build_trigger_sweep_inflation( ...
                    obj.active_speed_certificate);

            raw_obs=ENVIRONMENT_OBSTACLE_ELLIPSE_MOVE(0.0);
            obj.static_obstacle_map=raw_obs;
            obj.obstacle_packet=obj.prepare_obstacle_packet(raw_obs);

            % Dependency-aware theorem certificates are built from CURRENT
            % live/source-truth values.  No observed numeric configuration
            % is copied into the implementation as a fixed assumption.
            obj.initialize_runtime_certificate_manager();

            obj.ensure_mex_compiled();
            obj.warmup_runtime();

            sensor_props=[ ...
                "time", ...
                "pQ", ...
                "detected_point", ...
                "detected_obstacles", ...
                "need_replan", ...
                "relevant_obstacles", ...
                "min_obstacle_id_point", ...
                "min_dist_point", ...
                "detected_obstacle_count_point", ...
                "loadmass", ...
                "profiling", ...
                "current_bspline", ...
                "active_repaired_reference", ...
                "active_repaired_certificates", ...
                "last_replan_status", ...
                "replan_commit_count", ...
                "last_replan_commit_time", ...
                "trigger_ttc", ...
                "trigger_collision_time", ...
                "trigger_obstacle_id", ...
                "trigger_component", ...
                "trigger_component_code", ...
                "trigger_pL_collision", ...
                "trigger_q_collision", ...
                "trigger_pQ_collision", ...
                "trigger_effective_horizon", ...
                "trigger_reference_points", ...
                "trigger_reference_source", ...
                "trigger_sensor_origin", ...
                "trigger_cable_residual", ...
                "trigger_active_mask", ...
                "trigger_raw_distances", ...
                "trigger_hit_index", ...
                "trigger_kernel_called", ...
                "active_reference_mode", ...
                "active_reference_version", ...
                "active_speed_certificate", ...
                "trigger_sweep_inflation", ...
                "sensor_horizon_certificate", ...
                "stop_geometry_4ms_certificate", ...
                "command_limit_certificate", ...
                "command_limit_live_verified", ...
                "actuator_dynamics_certificate", ...
                "certificate_stale", ...
                "runtime_dependency_signature" ...
            ];

            for p_name=sensor_props
                if ~isprop(obj.result.state,p_name)
                    addprop(obj.result.state,p_name);
                end
            end

            obj.clear_state_sensor_values(0.0);

            if obj.ENABLE_ALL_LOG
                fprintf(['[REPLANNING_SYSTEM] initialized | sensor=%.3f m | H=%.3f s | ' ...
                    'dt=%.6f s | d_margin=%.3f m | warmup=%.3f ms\n'], ...
                    obj.trigger_dist,obj.horizon_s,obj.prediction_dt,obj.d_margin, ...
                    obj.warmup_runtime_ms);
            end
        end


        function result_out = do(obj,varargin)
            % =================================================================
            % 【メイン実行エントリーポイント: do】
            %   制御周期 (dt) ごとに呼び出され、衝突判定および状態出力を実行します。
            %
            % 【引数】
            %   varargin{1} : time (シミュレーション時間構造体またはオブジェクト: t, dt 等)
            %   varargin{2} : cha  (動作フェーズ文字: 'a'=Arming, 't'=Takeoff, 'f'=Flight 等)
            %   varargin{3} : state (オプション: 現在の状態量)
            %
            % 【スパイク防止処理フロー】
            %   1. 制御周期 dt の整合性チェック
            %   2. 牽引物質量推定値 mL の必須検証 (取得不可時は安全のため即座にエラー停止)
            %   3. ベース公称軌道 base_ref.do() の評価
            %   4. フェーズ判別:
            %      - 非保護フェーズ ('a', 't'):
            %        静止中 (Arming/Takeoff初期) に公称テーブル生成と JIT プライムを一括完遂。
            %        センサ・トリガー状態をゼロリセットし、公称指令値をパススルー出力。
            %      - 保護フェーズ ('f'):
            %        事前プライムの完了を厳格検証。
            %        機体位置 pQ を中心とする球状センサ (20m) で近接障害物をフィルタリング。
            %        公称テーブルから将来 H 区間の [pL, q] を O(1) スライス取得。
            %        C++ MEX カーネルを呼び出し、将来衝突判定を高速実行。
            %        結果を事前確保済み固定長フィールドへ直接上書き。
            % =================================================================
            time=varargin{1};
            cha=varargin{2};

            if isfield(time,'dt') && ~isempty(time.dt) && time.dt>0
                if abs(time.dt-obj.prediction_dt)>1e-12
                    obj.prediction_dt=time.dt;
                    ratio=obj.horizon_s/obj.prediction_dt;
                    assert(abs(ratio-round(ratio)) <= 1e-9*max(1,abs(ratio)), ...
                        '[REPLANNER ERROR] horizon_s/prediction_dt must be integer.');
                    obj.reference_cache_valid=false;
                    obj.nominal_table_valid=false;
                    obj.online_prime_done=false;
                    obj.setup_exact_do_prime_done=false;
                end
            end

            obj.loadmass=obj.get_load_mass();

            % The replanner is constructed before the live HLC controller
            % object exists in run_flight_headless.  Command limits are
            % therefore source-bound in the constructor and the actual live
            % controller handle is verified lazily once it exists.  Flight
            % may not proceed without that live verification.
            obj.verify_live_controller_binding(isequal(cha,'f'));

            % Fast dependency check: scalar/string comparisons only.
            % No source-file parsing and no symbolic derivation is performed
            % in the Flight control cycle.
            obj.update_runtime_dependency_state_fast(cha);

            if obj.ENABLE_PROFILING
                t_total=tic;
                t0=tic;
            end

            base_res=obj.base_ref.do(varargin{:});
            xd_nom=base_res.state.xd;

            if obj.ENABLE_PROFILING
                obj.profiling.t_base_ref_ms=1e3*toc(t0);
            end

            protected_phase=isequal(cha,'f');

            if ~protected_phase
                obj.clear_state_sensor_values(time.t);

                preflight_context=obj.is_flight_warmup_context();
                try
                    t_now_pre=double(time.t);
                catch
                    t_now_pre=NaN;
                end
                if isequal(cha,'a') && isfinite(t_now_pre) && ...
                        abs(t_now_pre)<=0.5*obj.prediction_dt
                    preflight_context=true;
                end

                if preflight_context && (~obj.nominal_table_valid || ~obj.online_prime_done)
                    if ~obj.nominal_table_valid
                        obj.build_nominal_reference_table(time,varargin,xd_nom);
                    end
                    if obj.nominal_table_valid && ~obj.online_prime_done
                        obj.warmup_online_path(time,varargin,xd_nom);
                    end
                end

                obj.write_nominal_output(xd_nom);
                result_out=obj.result;
                return;
            end

            if obj.REQUIRE_PRIME_BEFORE_TAKEOFF && ~obj.setup_exact_do_prime_active
                assert(obj.nominal_table_valid && obj.online_prime_done && ...
                    obj.setup_exact_do_prime_done, ...
                    ['[REPLANNER PREFLIGHT ERROR] Flight reached before pre-motion exact prime completed. ' ...
                     'Prime must finish before vehicle motion.']);
                assert(~obj.setup_exact_force_hit, ...
                    '[REPLANNER ERROR] setup-only forced-hit flag leaked into Flight.');
            end

            obj.assert_runtime_core_certificates_current();

            xd_active_now=obj.evaluate_active_reference_xd(time.t,xd_nom);
            [pQ_cur,~,~,cable_residual]=obj.get_current_physical_geometry(xd_active_now);

            if obj.ENABLE_PROFILING, ts=tic; end
            [active,min_dist,min_id,all_dists]= ...
                obj.raw_sensor_gate(pQ_cur,obj.static_obstacle_map);
            if obj.ENABLE_PROFILING
                obj.profiling.t_sensor_ms=1e3*toc(ts);
                tr=tic;
            end

            vis_count=nnz(active);

            [pL_ref,q_ref,ref_source]= ...
                obj.get_active_reference_packet(time,cha,varargin,xd_nom);

            if obj.ENABLE_PROFILING
                obj.profiling.t_reference_packet_ms=1e3*toc(tr);
                tt=tic;
            end

            N=size(pL_ref,2);
            effective_horizon=(N-1)*obj.prediction_dt;

            [need_replan,ttc,collision_time,comp_name, ...
                pL_collision,q_collision,pQ_collision,jhit, ...
                obs_id,comp_code,kernel_called,kernel_ms] = ...
                obj.trigger_core(active,pL_ref,q_ref,time.t);

            if obj.setup_exact_do_prime_active && obj.setup_exact_force_hit
                force_idx=max(1,obj.setup_exact_force_hit_calls);
                if N>=2
                    j_force=1+mod(force_idx-1,2);   % current/future-hit downstream paths
                else
                    j_force=1;
                end
                obs_force=obj.obstacle_packet.ids(1);
                comp_force=1+mod(force_idx-1,3);   % PAYLOAD/UAV/CABLE result paths
                [need_replan,ttc,collision_time,comp_name, ...
                    pL_collision,q_collision,pQ_collision,jhit] = ...
                    obj.decode_trigger_result( ...
                        true,j_force,obs_force,comp_force,N,pL_ref,q_ref,time.t);
                obs_id=obs_force;
                comp_code=comp_force;
                kernel_called=true;
            end

            if obj.ENABLE_PROFILING
                obj.profiling.t_kernel_ms=kernel_ms;
                obj.profiling.t_trigger_ms=1e3*toc(tt);
            end

            if need_replan
                obj.last_replan_status=struct( ...
                    'requested',true, ...
                    'committed',false, ...
                    'reason','CURRENT primary generator not yet installed', ...
                    'request_time',double(time.t), ...
                    'trigger_ttc',double(ttc), ...
                    'trigger_obstacle_id',double(obs_id), ...
                    'trigger_component',char(comp_name));
            end

            st=obj.result.state;
            obj.write_active_output(time.t,xd_nom);

            st.time=time.t;
            st.pQ=pQ_cur;
            st.detected_point=vis_count>0;

            % Variable-size detected/relevant structs are compatibility
            % placeholders only. Fixed scalar/vector fields below are the
            % production LOGGER interface.
            st.min_obstacle_id_point=min_id;
            st.min_dist_point=min_dist;
            st.detected_obstacle_count_point=vis_count;

            st.need_replan=need_replan;
            st.trigger_ttc=ttc;
            st.trigger_collision_time=collision_time;
            st.trigger_obstacle_id=obs_id;
            st.trigger_component=comp_name;
            st.trigger_component_code=comp_code;
            st.trigger_pL_collision=pL_collision;
            st.trigger_q_collision=q_collision;
            st.trigger_pQ_collision=pQ_collision;
            st.trigger_effective_horizon=effective_horizon;
            st.trigger_reference_points=N;
            st.trigger_reference_source=ref_source;
            st.trigger_sensor_origin=pQ_cur;
            st.trigger_cable_residual=cable_residual;
            st.trigger_active_mask=double(active);
            st.trigger_raw_distances=all_dists;
            st.trigger_hit_index=jhit;
            st.trigger_kernel_called=double(kernel_called);
            st.active_reference_mode=obj.active_reference_mode;
            st.active_reference_version=obj.active_reference_version;
            st.active_speed_certificate=obj.active_speed_certificate;
            st.trigger_sweep_inflation=obj.trigger_sweep_inflation;
            st.sensor_horizon_certificate=obj.sensor_horizon_certificate;
            st.stop_geometry_4ms_certificate=obj.stop_geometry_4ms_certificate;
            st.command_limit_certificate=obj.command_limit_certificate;
            if isfield(obj.command_limit_certificate,'live_controller_verified')
                st.command_limit_live_verified= ...
                    logical(obj.command_limit_certificate.live_controller_verified);
            else
                st.command_limit_live_verified=false;
            end
            st.actuator_dynamics_certificate=obj.actuator_dynamics_certificate;
            st.certificate_stale=obj.certificate_stale;
            st.runtime_dependency_signature=obj.runtime_dependency_signature;
            st.loadmass=obj.loadmass;
            st.current_bspline=obj.current_bspline;
            st.active_repaired_reference=obj.repaired_reference_metadata();
            st.active_repaired_certificates=obj.active_repaired_certificates;
            st.last_replan_status=obj.last_replan_status;
            st.replan_commit_count=obj.replan_commit_count;
            st.last_replan_commit_time=obj.last_replan_commit_time;

            if obj.ENABLE_PROFILING
                obj.profiling.t_total_ms=1e3*toc(t_total);
                st.profiling=obj.profiling;
            end

            if obj.ENABLE_ALL_LOG
                if obj.LOG_SENSOR && vis_count>0
                    fprintf('[SENSOR RAW %.1fm] t=%.3f | count=%d | nearest=%g | d=%.3f m\n', ...
                        obj.trigger_dist,time.t,vis_count,min_id,min_dist);
                end
                if obj.LOG_TRIGGER && need_replan
                    fprintf('[FUTURE HIT] t=%.3f | TTC=%.3f | obs=%g | comp=%s\n', ...
                        time.t,ttc,obs_id,comp_name);
                end
            end

            result_out=obj.result;
        end


        function mL = get_load_mass(obj)
            % =================================================================
            % 【牽引物質量推定値取得関数】
            %   状態推定器 (self.estimator.result.state.mL) から牽引物質量の最新推定値を取得します。
            %
            % 【安全性に関する厳格な制約（フォールバック禁止）】
            %   本制御システムにおいて牽引物質量のオンライン推定値は安全保証の中核です。
            %   推定値が存在しない場合、あるいは非正値・不正値の場合は安全のため即座にエラー停止します。
            %   （設計方針に基づき、安易なデフォルト値や固定値へのフォールバックは一切行いません）
            % =================================================================
            st_est = [];
            if obj.has_prop_or_field(obj.self,'estimator') && ~isempty(obj.self.estimator)
                est_obj = obj.self.estimator;
                if obj.has_prop_or_field(est_obj,'result') && ~isempty(est_obj.result)
                    res = est_obj.result;
                    if obj.has_prop_or_field(res,'state') && ~isempty(res.state)
                        st_est = res.state;
                    end
                end
            end
            has_mL = ~isempty(st_est) && obj.has_prop_or_field(st_est,'mL');
            assert(has_mL, ...
                '[REPLANNER ERROR] 推定器 (self.estimator.result.state) に牽引物質量の推定プロパティ "mL" が存在しません。本システムは牽引物質量の推定値に基づく安全検証を必須としています。');
            mL = st_est.mL;
            assert(isnumeric(mL) && isscalar(mL) && isfinite(mL) && mL>0, ...
                '[REPLANNER ERROR] 牽引物質量の推定値 (self.estimator.result.state.mL) が有効な正の数値ではありません (値: %s)。', mat2str(mL));
        end

    end



    methods (Access = private)


        function cert = build_nominal_active_speed_certificate(obj)
            % =============================================================
            % Current TIME_VARYING_REFERENCE.func から exact nominal
            % continuous-time V_L, V_Q を導出する。
            %
            % 現段階では速度ノルムが時間一定、かつ aL==0 のケースを
            % 厳密に閉じる。一般化できない reference では推測せず停止。
            % =============================================================
            assert(strcmpi(char(obj.active_reference_mode),'NOMINAL'), ...
                ['[REPLANNER ERROR] nominal speed certificate requested ' ...
                 'while active_reference_mode is not NOMINAL.']);
            assert(isa(obj.base_ref,'TIME_VARYING_REFERENCE'), ...
                ['[REPLANNER ERROR] exact nominal speed certificate requires ' ...
                 'TIME_VARYING_REFERENCE base_ref.']);
            assert(isprop(obj.base_ref,'func') && ...
                isa(obj.base_ref.func,'function_handle'), ...
                '[REPLANNER ERROR] nominal base_ref.func is unavailable.');

            fstr=func2str(obj.base_ref.func);
            [xd_sym,t]=obj.function_handle_to_symbolic_vector(fstr);

            assert(numel(xd_sym)>=11, ...
                '[REPLANNER ERROR] nominal xd must contain at least 11 states.');

            pL=simplify(xd_sym(1:3));
            vL=simplify(xd_sym(5:7));
            aL=simplify(xd_sym(9:11));

            assert(obj.symbolic_vector_equal(simplify(diff(pL,t)),vL), ...
                '[REPLANNER ERROR] nominal xd(5:7) is not d/dt xd(1:3).');
            assert(obj.symbolic_vector_equal(simplify(diff(vL,t)),aL), ...
                '[REPLANNER ERROR] nominal xd(9:11) is not d/dt xd(5:7).');

            speed2=simplify(vL.'*vL);
            assert(obj.symbolic_zero(simplify(diff(speed2,t))), ...
                ['[REPLANNER ERROR] current nominal speed is not constant. ' ...
                 'Install a theorem-backed global speed bound instead of ' ...
                 'using a sampled maximum.']);

            VL=double(simplify(sqrt(speed2)));
            assert(isscalar(VL) && isfinite(VL) && VL>=0);

            aZero=true;
            for k=1:numel(aL)
                if ~obj.symbolic_zero(aL(k))
                    aZero=false;
                    break;
                end
            end
            assert(aZero, ...
                ['[REPLANNER ERROR] current nominal aL is not identically zero. ' ...
                 'General qdot certification is required before trigger use.']);

            % aL==0 => q=-e3, qdot=0 => vQ=vL exactly.
            VQ=VL;

            cert=struct();
            cert.certified=true;
            cert.reference_mode='NOMINAL';
            cert.reference_version=double(obj.active_reference_version);
            cert.reference_class=class(obj.base_ref);
            cert.func_text=fstr;
            cert.payload_speed_UB_m_s=VL;
            cert.uav_speed_UB_m_s=VQ;
            cert.payload_velocity_vector_m_s=double(vL(:));
            cert.nominal_acceleration_zero=true;
            cert.qdot_UB_rad_s=0;
            cert.source='EXACT_CURRENT_NOMINAL_FUNC';
        end


        function sweep = build_trigger_sweep_inflation(obj,cert)
            % =============================================================
            % Sampled future trigger -> continuous time lift:
            %
            % delta_L = V_L*dt/2
            % delta_Q = V_Q*dt/2
            % delta_C = max(V_L,V_Q)*dt/2
            %
            % Sensor sampling is別問題であり raw sensorには加えない。
            % =============================================================
            obj.validate_active_speed_certificate(cert);

            VL=cert.payload_speed_UB_m_s;
            VQ=cert.uav_speed_UB_m_s;
            dt=obj.prediction_dt;

            sweep=struct();
            sweep.certified=true;
            sweep.reference_mode=char(cert.reference_mode);
            sweep.reference_version=double(cert.reference_version);
            sweep.payload_m=VL*dt/2;
            sweep.uav_m=VQ*dt/2;
            sweep.cable_m=max(VL,VQ)*dt/2;
            sweep.sensor_one_period_m=VQ*dt;
            sweep.dt=dt;
            sweep.source='VERSION_BOUND_ACTIVE_SPEED_CERTIFICATE';
        end


        function validate_active_speed_certificate(obj,cert)
            assert(isstruct(cert) && isfield(cert,'certified') && ...
                logical(cert.certified), ...
                '[REPLANNER ERROR] active speed certificate is not certified.');

            req={'reference_mode','reference_version', ...
                 'payload_speed_UB_m_s','uav_speed_UB_m_s'};
            for k=1:numel(req)
                assert(isfield(cert,req{k}), ...
                    '[REPLANNER ERROR] speed certificate missing field %s.', ...
                    req{k});
            end

            assert(strcmpi(char(cert.reference_mode), ...
                char(obj.active_reference_mode)), ...
                ['[REPLANNER ERROR] speed certificate reference_mode is stale ' ...
                 'for current active reference.']);
            assert(double(cert.reference_version)== ...
                double(obj.active_reference_version), ...
                ['[REPLANNER ERROR] speed certificate reference_version is stale ' ...
                 'for current active reference.']);

            if strcmpi(char(obj.active_reference_mode),'NOMINAL')
                assert(isfield(cert,'func_text') && ...
                    isa(obj.base_ref,'TIME_VARYING_REFERENCE') && ...
                    isprop(obj.base_ref,'func') && ...
                    isa(obj.base_ref.func,'function_handle') && ...
                    strcmp(char(cert.func_text),func2str(obj.base_ref.func)), ...
                    ['[REPLANNER ERROR] nominal reference function changed ' ...
                     'without rebuilding its speed certificate.']);
            end

            VL=cert.payload_speed_UB_m_s;
            VQ=cert.uav_speed_UB_m_s;
            assert(isnumeric(VL)&&isscalar(VL)&&isfinite(VL)&&VL>=0);
            assert(isnumeric(VQ)&&isscalar(VQ)&&isfinite(VQ)&&VQ>=0);
        end


        function assert_trigger_packet_certificate_current(obj)
            % =============================================================
            % 古い NOMINAL packet を REPAIRED/version更新後に誤使用する
            % ことを禁止する fail-closed guard。
            % =============================================================
            obj.validate_active_speed_certificate( ...
                obj.active_speed_certificate);

            s=obj.trigger_sweep_inflation;
            assert(isstruct(s)&&isfield(s,'certified')&&logical(s.certified), ...
                '[REPLANNER ERROR] trigger sweep certificate is unavailable.');
            assert(strcmpi(char(s.reference_mode), ...
                char(obj.active_reference_mode)) && ...
                double(s.reference_version)== ...
                double(obj.active_reference_version), ...
                '[REPLANNER ERROR] trigger sweep certificate is stale.');
            assert(isfield(s,'dt') && ...
                double(s.dt)==double(obj.prediction_dt), ...
                ['[REPLANNER ERROR] prediction_dt changed without ' ...
                 'rebuilding trigger sweep certificate.']);

            p=obj.obstacle_packet;
            assert(isstruct(p) && ...
                isfield(p,'reference_mode') && ...
                isfield(p,'reference_version') && ...
                isfield(p,'sweep_certified'), ...
                '[REPLANNER ERROR] obstacle packet lacks version-bound sweep metadata.');
            assert(logical(p.sweep_certified) && ...
                strcmpi(char(p.reference_mode), ...
                    char(obj.active_reference_mode)) && ...
                double(p.reference_version)== ...
                    double(obj.active_reference_version), ...
                '[REPLANNER ERROR] obstacle packet sweep certificate is stale.');
            assert(isfield(p,'sweep_dt') && ...
                double(p.sweep_dt)==double(obj.prediction_dt), ...
                '[REPLANNER ERROR] obstacle packet sweep dt is stale.');
        end




        function validate_repaired_candidate(obj,candidate,t_commit)
            obj.assert_hard_modify_deadline_limit();

            assert(isstruct(candidate) && ...
                isfield(candidate,'certified') && ...
                logical(candidate.certified), ...
                '[REPLANNER ERROR] repaired candidate is not certified.');

            if isfield(candidate,'test_only')
                assert(~logical(candidate.test_only), ...
                    '[REPLANNER ERROR] test-only repaired candidates are forbidden in production commit.');
            end

            req={'backend','eval_fn','valid_from','valid_until', ...
                 'speed_certificate','smoothness_certificate', ...
                 'geometry_certificate','dynamics_certificate', ...
                 'hlc_robust_certificate','backup_certificate', ...
                 'position_c6_certificate','yaw_c6_certificate', ...
                 'yaw_policy','path_parameterization', ...
                 'path_position_c7_certificate', ...
                 'time_law_c6_certificate', ...
                 'time_law_monotonic_certificate', ...
                 'terminal_time_law_hold_certificate', ...
                 'finite_sensing_certificate', ...
                 'compute_deadline_certificate', ...
                 'dependency_certificate'};
            for kk=1:numel(req)
                assert(isfield(candidate,req{kk}), ...
                    '[REPLANNER ERROR] repaired candidate missing field %s.',req{kk});
            end

            assert(ischar(candidate.backend) || isstring(candidate.backend), ...
                '[REPLANNER ERROR] repaired backend name is invalid.');
            assert(isa(candidate.eval_fn,'function_handle'), ...
                '[REPLANNER ERROR] repaired candidate eval_fn must be a function handle.');

            vf=double(candidate.valid_from);
            vu=double(candidate.valid_until);
            assert(isscalar(vf) && isfinite(vf) && ...
                isscalar(vu) && (isfinite(vu)||isinf(vu)) && vu>=vf, ...
                '[REPLANNER ERROR] repaired candidate validity interval is invalid.');
            assert(t_commit>=vf && ...
                t_commit+double(obj.horizon_s)<=vu, ...
                ['[REPLANNER ERROR] repaired candidate does not cover the ' ...
                 'full future-trigger horizon from commit.']);

            targetVersion=double(obj.active_reference_version)+1;

            % -------------------------------------------------------------
            % Version-bound continuous-time production certificates.
            % -------------------------------------------------------------
            certNames={ ...
                'smoothness_certificate', ...
                'geometry_certificate', ...
                'dynamics_certificate', ...
                'hlc_robust_certificate', ...
                'backup_certificate', ...
                'position_c6_certificate', ...
                'yaw_c6_certificate', ...
                'path_position_c7_certificate', ...
                'time_law_c6_certificate', ...
                'time_law_monotonic_certificate', ...
                'terminal_time_law_hold_certificate', ...
                'finite_sensing_certificate'};
            for kk=1:numel(certNames)
                obj.validate_repaired_bound_certificate( ...
                    candidate.(certNames{kk}),certNames{kk}, ...
                    targetVersion,vf,vu);
            end

            % -------------------------------------------------------------
            % General independent C6 yaw.
            %
            % Yaw may be optimized to look along the travel direction, but
            % alignment is a generator preference rather than a safety gate.
            % Production requires only the exact C6 yaw contract here.
            % -------------------------------------------------------------
            assert(strcmpi(char(candidate.yaw_policy), ...
                'C6_INDEPENDENT'), ...
                ['[REPLANNER ERROR] production yaw_policy must be ' ...
                 'C6_INDEPENDENT.']);

            P=candidate.path_parameterization;
            assert(isstruct(P) && ...
                isfield(P,'kind') && ...
                strcmpi(char(P.kind),'PATH_TIME_SEPARATED') && ...
                isfield(P,'path_derivative_order') && ...
                double(P.path_derivative_order)>=7 && ...
                isfield(P,'time_derivative_order') && ...
                double(P.time_derivative_order)>=6, ...
                ['[REPLANNER ERROR] path_parameterization must be ' ...
                 'PATH_TIME_SEPARATED with path order >=7 and time order >=6.']);

            PC6=candidate.position_c6_certificate;
            YC6=candidate.yaw_c6_certificate;
            PC7=candidate.path_position_c7_certificate;
            TC6=candidate.time_law_c6_certificate;
            TM=candidate.time_law_monotonic_certificate;
            TH=candidate.terminal_time_law_hold_certificate;
            FS=candidate.finite_sensing_certificate;

            obj.assert_continuous_time_certificate(PC6,'position_c6_certificate');
            obj.assert_continuous_time_certificate(YC6,'yaw_c6_certificate');
            obj.assert_continuous_time_certificate(PC7,'path_position_c7_certificate');
            obj.assert_continuous_time_certificate(TC6,'time_law_c6_certificate');
            obj.assert_continuous_time_certificate(TM,'time_law_monotonic_certificate');
            obj.assert_continuous_time_certificate(TH,'terminal_time_law_hold_certificate');
            obj.assert_continuous_time_certificate(FS,'finite_sensing_certificate');

            assert(isfield(PC6,'continuity_order') && ...
                double(PC6.continuity_order)>=6, ...
                '[REPLANNER ERROR] position C6 continuity order is not certified.');
            assert(isfield(YC6,'continuity_order') && ...
                double(YC6.continuity_order)>=6, ...
                '[REPLANNER ERROR] yaw C6 continuity order is not certified.');
            assert(isfield(PC7,'continuity_order') && ...
                double(PC7.continuity_order)>=7, ...
                ['[REPLANNER ERROR] internal path-position continuity ' ...
                 'order >=7 is required by the retained four-backend generator contract.']);

            assert(isfield(TC6,'continuity_order') && ...
                double(TC6.continuity_order)>=6, ...
                '[REPLANNER ERROR] time law C6 continuity order is not certified.');

            assert(isfield(TM,'sdot_lower_bound') && ...
                isnumeric(TM.sdot_lower_bound) && ...
                isscalar(TM.sdot_lower_bound) && ...
                isfinite(TM.sdot_lower_bound) && ...
                double(TM.sdot_lower_bound)>=0, ...
                '[REPLANNER ERROR] time law monotonicity certificate is invalid.');

            assert(isfield(TH,'zero_derivative_order') && ...
                double(TH.zero_derivative_order)>=6 && ...
                isfield(TH,'constant_hold_after_terminal') && ...
                logical(TH.constant_hold_after_terminal), ...
                ['[REPLANNER ERROR] terminal time-law hold must set ' ...
                 'derivatives 1..6 to zero and continue as a constant hold.']);

            assert(isfield(FS,'sensor_range_m') && ...
                double(FS.sensor_range_m)==double(obj.trigger_dist) && ...
                isfield(FS,'hard_modify_deadline_s') && ...
                double(FS.hard_modify_deadline_s)== ...
                    double(obj.hard_modify_deadline_s) && ...
                isfield(FS,'backup_available') && ...
                logical(FS.backup_available), ...
                ['[REPLANNER ERROR] finite-sensing certificate is not bound ' ...
                 'to the live sensor/deadline/backup contract.']);

            % -------------------------------------------------------------
            % Speed certificate is already version-bound by its own schema.
            % Validate target version before mutating live reference version.
            % -------------------------------------------------------------
            sp=candidate.speed_certificate;
            assert(isstruct(sp) && isfield(sp,'certified') && ...
                logical(sp.certified) && ...
                isfield(sp,'reference_mode') && ...
                strcmpi(char(sp.reference_mode),'REPAIRED') && ...
                isfield(sp,'reference_version') && ...
                double(sp.reference_version)==targetVersion && ...
                isfield(sp,'payload_speed_UB_m_s') && ...
                isnumeric(sp.payload_speed_UB_m_s) && ...
                isscalar(sp.payload_speed_UB_m_s) && ...
                isfinite(sp.payload_speed_UB_m_s) && ...
                double(sp.payload_speed_UB_m_s)>=0 && ...
                isfield(sp,'uav_speed_UB_m_s') && ...
                isnumeric(sp.uav_speed_UB_m_s) && ...
                isscalar(sp.uav_speed_UB_m_s) && ...
                isfinite(sp.uav_speed_UB_m_s) && ...
                double(sp.uav_speed_UB_m_s)>=0, ...
                '[REPLANNER ERROR] repaired speed certificate is invalid/stale.');

            % -------------------------------------------------------------
            % End-to-end compute deadline record.  This is deadline
            % ENFORCEMENT, not a formal OS/WCET theorem.
            % -------------------------------------------------------------
            DC=candidate.compute_deadline_certificate;
            obj.validate_compute_deadline_certificate( ...
                DC,targetVersion,double(t_commit));

            % -------------------------------------------------------------
            % Live/source dependency binding.  This prevents stale candidate
            % certificates after dt/sensor/radius/mass/HLC/deadline changes.
            % -------------------------------------------------------------
            DEP=candidate.dependency_certificate;
            obj.validate_candidate_dependency_certificate( ...
                DEP,candidate.eval_fn,targetVersion);

            % -------------------------------------------------------------
            % Recursively safe repaired-state backup contract.
            % No nominal-only vertical-stop certificate can satisfy this
            % schema unless it has separately proved the general conditions.
            % -------------------------------------------------------------
            B=candidate.backup_certificate;
            assert(isfield(B,'general_repaired_state') && ...
                logical(B.general_repaired_state) && ...
                isfield(B,'recursively_safe') && ...
                logical(B.recursively_safe) && ...
                isfield(B,'continuous_time') && ...
                logical(B.continuous_time) && ...
                isfield(B,'backup_kind') && ...
                (ischar(B.backup_kind)||isstring(B.backup_kind)), ...
                ['[REPLANNER ERROR] backup certificate is not a general ' ...
                 'continuous-time recursively-safe REPAIRED-state backup.']);

            backupReq={'stop_geometry_certificate', ...
                       'actuator_dynamics_certificate', ...
                       'hlc_robust_certificate', ...
                       'terminal_hold_certificate', ...
                       'finite_sensing_certificate'};
            for kk=1:numel(backupReq)
                name=backupReq{kk};
                assert(isfield(B,name), ...
                    '[REPLANNER ERROR] repaired backup missing field %s.',name);
                obj.validate_repaired_bound_certificate( ...
                    B.(name),['backup.' name],targetVersion,vf,vu);
            end

            assert(isfield(B.terminal_hold_certificate, ...
                'constant_hold_after_terminal') && ...
                logical(B.terminal_hold_certificate.constant_hold_after_terminal), ...
                '[REPLANNER ERROR] repaired backup terminal hold is not closed.');

            % -------------------------------------------------------------
            % Evaluate both commit instant and future-trigger horizon end
            % before any live state is mutated.
            % -------------------------------------------------------------
            tcheck=[t_commit, t_commit+double(obj.horizon_s)];
            for kk=1:numel(tcheck)
                xd=candidate.eval_fn(tcheck(kk));
                xd=xd(:);
                assert(numel(xd)>=28 && all(isfinite(xd(1:28))), ...
                    ['[REPLANNER ERROR] repaired candidate eval_fn must ' ...
                     'return a finite HLC d0..d6 28-state reference.']);
                aL=xd(9:11);
                r=aL+[0;0;obj.gravity];
                assert(norm(r)>0, ...
                    '[REPLANNER ERROR] repaired candidate violates flatness nonzero-vector condition.');
            end
        end


        function validate_repaired_bound_certificate(obj,C,name,targetVersion,vf,vu)
            assert(isstruct(C) && isfield(C,'certified') && ...
                logical(C.certified), ...
                '[REPLANNER ERROR] candidate certificate %s is not closed.',name);

            if isfield(C,'test_only')
                assert(~logical(C.test_only), ...
                    '[REPLANNER ERROR] test-only certificate %s is forbidden in production.',name);
            end

            req={'reference_mode','reference_version','valid_from','valid_until'};
            for kk=1:numel(req)
                assert(isfield(C,req{kk}), ...
                    '[REPLANNER ERROR] certificate %s missing field %s.', ...
                    name,req{kk});
            end

            assert(strcmpi(char(C.reference_mode),'REPAIRED') && ...
                double(C.reference_version)==double(targetVersion), ...
                '[REPLANNER ERROR] certificate %s is bound to a stale reference.',name);

            cvf=double(C.valid_from);
            cvu=double(C.valid_until);
            assert(isscalar(cvf)&&isfinite(cvf) && ...
                isscalar(cvu)&&(isfinite(cvu)||isinf(cvu)) && ...
                cvf<=vf && cvu>=vu, ...
                '[REPLANNER ERROR] certificate %s does not cover candidate validity.',name);
        end


        function assert_continuous_time_certificate(~,C,name)
            assert(isfield(C,'continuous_time') && ...
                logical(C.continuous_time), ...
                ['[REPLANNER ERROR] %s must be a continuous-time ' ...
                 'certificate; sampled-only evidence is insufficient.'],name);
        end


        function validate_compute_deadline_certificate(obj,C,targetVersion,t_commit)
            assert(isstruct(C) && isfield(C,'certified') && ...
                logical(C.certified), ...
                '[REPLANNER ERROR] compute deadline certificate is not closed.');

            if isfield(C,'test_only')
                assert(~logical(C.test_only), ...
                    '[REPLANNER ERROR] test-only compute deadline certificate is forbidden.');
            end

            req={'reference_mode','reference_version','scope', ...
                 'elapsed_s','hard_limit_s','completed_before_commit', ...
                 'commit_time'};
            for kk=1:numel(req)
                assert(isfield(C,req{kk}), ...
                    '[REPLANNER ERROR] compute deadline certificate missing field %s.', ...
                    req{kk});
            end

            assert(strcmpi(char(C.reference_mode),'REPAIRED') && ...
                double(C.reference_version)==double(targetVersion), ...
                '[REPLANNER ERROR] compute deadline certificate is version-stale.');
            assert(strcmpi(char(C.scope), ...
                'END_TO_END_ONLINE_TRAJECTORY_MODIFICATION'), ...
                '[REPLANNER ERROR] compute deadline scope is not end-to-end.');
            assert(isnumeric(C.elapsed_s)&&isscalar(C.elapsed_s)&& ...
                isfinite(C.elapsed_s)&&double(C.elapsed_s)>=0, ...
                '[REPLANNER ERROR] compute elapsed_s is invalid.');
            assert(isnumeric(C.hard_limit_s)&&isscalar(C.hard_limit_s)&& ...
                isfinite(C.hard_limit_s)&& ...
                double(C.hard_limit_s)==double(obj.hard_modify_deadline_s) && ...
                double(C.hard_limit_s)<=4.0e-3, ...
                '[REPLANNER ERROR] compute hard limit is not the live <=4 ms limit.');
            assert(double(C.elapsed_s)<=double(C.hard_limit_s), ...
                '[REPLANNER ERROR] repaired candidate missed the hard online modification deadline.');
            assert(logical(C.completed_before_commit), ...
                '[REPLANNER ERROR] compute deadline measurement did not complete before commit.');
            assert(double(C.commit_time)==double(t_commit), ...
                '[REPLANNER ERROR] compute deadline certificate commit_time is stale.');
        end


        function validate_candidate_dependency_certificate(obj,D,eval_fn,targetVersion)
            assert(isstruct(D) && isfield(D,'certified') && ...
                logical(D.certified) && isfield(D,'signature'), ...
                '[REPLANNER ERROR] candidate dependency certificate is unavailable.');

            if isfield(D,'test_only')
                assert(~logical(D.test_only), ...
                    '[REPLANNER ERROR] test-only dependency certificate is forbidden.');
            end

            expected=obj.capture_candidate_dependency_signature( ...
                eval_fn,targetVersion);
            assert(isequaln(D.signature,expected), ...
                ['[REPLANNER ERROR] candidate dependency/source signature ' ...
                 'does not match current live configuration.']);
        end


        function sig = capture_candidate_dependency_signature(obj,eval_fn,targetVersion)
            obj.assert_hard_modify_deadline_limit();

            sig=struct();
            sig.reference_mode='REPAIRED';
            sig.reference_version=double(targetVersion);
            sig.reference_eval_signature=func2str(eval_fn);

            sig.dt=double(obj.prediction_dt);
            sig.sensor_range=double(obj.trigger_dist);
            sig.horizon=double(obj.horizon_s);
            sig.r_drone=double(obj.r_drone);
            sig.r_load=double(obj.r_load);
            sig.r_cable=double(obj.r_cable);
            sig.cable_length=double(obj.L_cable);
            sig.gravity=double(obj.gravity);
            sig.mQ=obj.get_vehicle_mass();
            sig.mL_snapshot=double(obj.loadmass);
            sig.hard_modify_deadline_s=double(obj.hard_modify_deadline_s);

            sig.command_limit_source_signature= ...
                obj.runtime_source_signature.command_limit_source_signature;
            sig.command_limits=obj.runtime_source_signature.command_limits;
            sig.hlc_source=obj.runtime_source_signature.hlc_source;
            sig.hlc_settings_signature= ...
                obj.runtime_source_signature.hlc_settings_signature;
        end


        function assert_hard_modify_deadline_limit(obj)
            assert(isnumeric(obj.hard_modify_deadline_s) && ...
                isscalar(obj.hard_modify_deadline_s) && ...
                isfinite(obj.hard_modify_deadline_s) && ...
                obj.hard_modify_deadline_s>0 && ...
                obj.hard_modify_deadline_s<=4.0e-3, ...
                ['[REPLANNER ERROR] hard_modify_deadline_s exceeds the ' ...
                 'project hard maximum of 4 ms.']);
        end


        function initialize_runtime_certificate_manager(obj)
            % Constructor-safe: the live HLC object may not exist yet.
            % Build from the authoritative class source first.
            obj.command_limit_certificate= ...
                obj.build_active_command_limit_certificate();
            obj.verify_live_controller_binding(false);

            obj.sensor_horizon_certificate= ...
                obj.build_sensor_horizon_certificate();

            obj.stop_geometry_4ms_certificate= ...
                obj.build_stop_geometry_4ms_certificate();

            obj.actuator_dynamics_certificate= ...
                obj.build_current_actuator_dynamics_certificate();

            obj.hlc_robust_certificate=struct( ...
                'certified',false, ...
                'stale',true, ...
                'reason','HLC robust tracking-error certificate not installed yet');

            obj.runtime_source_signature= ...
                obj.capture_runtime_source_signature();
            obj.runtime_dependency_signature= ...
                obj.capture_runtime_dependency_signature_fast();

            obj.certificate_stale=obj.empty_certificate_stale_state();
            obj.certificate_stale.HLC_ROBUST_CERT=true;
            obj.dependency_refresh_count=obj.dependency_refresh_count+1;
        end


        function stale = empty_certificate_stale_state(~)
            stale=struct();
            stale.SPEED_TRIGGER_CERT=false;
            stale.SENSOR_HORIZON_CERT=false;
            stale.STOP_GEOMETRY_4MS_CERT=false;
            stale.ACTUATOR_DYNAMICS_CERT=false;
            stale.HLC_ROBUST_CERT=false;
        end


        function sig = capture_runtime_dependency_signature_fast(obj)
            obj.assert_hard_modify_deadline_limit();
            sig=struct();
            sig.reference_mode=char(obj.active_reference_mode);
            sig.reference_version=double(obj.active_reference_version);

            if isa(obj.base_ref,'TIME_VARYING_REFERENCE') && ...
                    isprop(obj.base_ref,'func') && ...
                    isa(obj.base_ref.func,'function_handle')
                sig.reference_func_signature=func2str(obj.base_ref.func);
            else
                sig.reference_func_signature='';
            end

            sig.dt=double(obj.prediction_dt);
            sig.sensor_range=double(obj.trigger_dist);
            sig.horizon=double(obj.horizon_s);
            sig.r_drone=double(obj.r_drone);
            sig.r_load=double(obj.r_load);
            sig.r_cable=double(obj.r_cable);
            sig.cable_length=double(obj.L_cable);
            sig.gravity=double(obj.gravity);
            sig.mQ=obj.get_vehicle_mass();
            sig.mL_snapshot=double(obj.loadmass);
            sig.hard_modify_deadline_s=double(obj.hard_modify_deadline_s);
        end


        function sig = capture_runtime_source_signature(obj)
            sig=struct();
            sig.command_limit_source_signature= ...
                obj.command_limit_certificate.source_line_signature;
            sig.command_limits=obj.command_limit_certificate.limits;

            S=Controller_HL(obj.prediction_dt);
            sig.hlc_source=which('Controller_HL');
            sig.hlc_settings_signature=obj.serialize_numeric_struct(S);
        end


        function update_runtime_dependency_state_fast(obj,cha)
            cur=obj.capture_runtime_dependency_signature_fast();

            if isempty(fieldnames(obj.runtime_dependency_signature))
                obj.runtime_dependency_signature=cur;
                return;
            end

            old=obj.runtime_dependency_signature;

            refChanged= ...
                ~strcmp(char(old.reference_mode),char(cur.reference_mode)) || ...
                double(old.reference_version)~=double(cur.reference_version) || ...
                ~strcmp(char(old.reference_func_signature), ...
                        char(cur.reference_func_signature));

            dtChanged=double(old.dt)~=double(cur.dt);

            sensorGeomChanged= ...
                double(old.sensor_range)~=double(cur.sensor_range) || ...
                double(old.horizon)~=double(cur.horizon) || ...
                double(old.r_drone)~=double(cur.r_drone) || ...
                double(old.r_load)~=double(cur.r_load) || ...
                double(old.r_cable)~=double(cur.r_cable) || ...
                double(old.cable_length)~=double(cur.cable_length);

            gravityChanged=double(old.gravity)~=double(cur.gravity);
            deadlineChanged= ...
                double(old.hard_modify_deadline_s)~= ...
                double(cur.hard_modify_deadline_s);

            massChanged= ...
                double(old.mQ)~=double(cur.mQ) || ...
                double(old.mL_snapshot)~=double(cur.mL_snapshot);

            if refChanged
                obj.certificate_stale.SPEED_TRIGGER_CERT=true;
                obj.certificate_stale.SENSOR_HORIZON_CERT=true;
                obj.certificate_stale.STOP_GEOMETRY_4MS_CERT=true;
                obj.certificate_stale.ACTUATOR_DYNAMICS_CERT=true;
                obj.certificate_stale.HLC_ROBUST_CERT=true;
            end

            if dtChanged
                obj.certificate_stale.SPEED_TRIGGER_CERT=true;
                obj.certificate_stale.SENSOR_HORIZON_CERT=true;
                obj.certificate_stale.STOP_GEOMETRY_4MS_CERT=true;
                obj.certificate_stale.HLC_ROBUST_CERT=true;
            end

            if sensorGeomChanged
                obj.certificate_stale.SENSOR_HORIZON_CERT=true;
                obj.certificate_stale.STOP_GEOMETRY_4MS_CERT=true;
            end

            if gravityChanged
                obj.certificate_stale.STOP_GEOMETRY_4MS_CERT=true;
                obj.certificate_stale.ACTUATOR_DYNAMICS_CERT=true;
                obj.certificate_stale.HLC_ROBUST_CERT=true;
            end

            if deadlineChanged
                obj.certificate_stale.STOP_GEOMETRY_4MS_CERT=true;
            end

            % Mass changes are the high-rate expected dependency change.
            % Rebuild only the mass-dependent nominal actuator certificate.
            % Geometry/sensor certificates are intentionally untouched.
            if massChanged && ~refChanged && ~gravityChanged
                if strcmpi(char(obj.active_reference_mode),'NOMINAL')
                    obj.actuator_dynamics_certificate= ...
                        obj.build_current_actuator_dynamics_certificate();
                    obj.certificate_stale.ACTUATOR_DYNAMICS_CERT= ...
                        ~logical(obj.actuator_dynamics_certificate.certified);
                    obj.certificate_stale.HLC_ROBUST_CERT=true;
                    obj.mass_certificate_rebuild_count= ...
                        obj.mass_certificate_rebuild_count+1;
                else
                    % A general REPAIRED trajectory can have nonzero cable
                    % motion/attitude dynamics.  The nominal vertical-stop
                    % formula must NEVER be reused after a mass change.
                    obj.certificate_stale.ACTUATOR_DYNAMICS_CERT=true;
                    obj.certificate_stale.HLC_ROBUST_CERT=true;
                end
            end

            obj.runtime_dependency_signature=cur;

            % Configuration/reference changes are supported fail-closed.
            % Heavy/source/symbolic rebuild must occur through
            % refresh_runtime_certificates() outside the Flight cycle.
            if isequal(cha,'f')
                assert(~obj.certificate_stale.SPEED_TRIGGER_CERT, ...
                    ['[REPLANNER DEPENDENCY ERROR] speed/trigger certificate ' ...
                     'became stale during Flight. Rebuild outside the Flight cycle.']);
                assert(~obj.certificate_stale.SENSOR_HORIZON_CERT, ...
                    ['[REPLANNER DEPENDENCY ERROR] sensor/horizon certificate ' ...
                     'became stale during Flight. Rebuild outside the Flight cycle.']);
                assert(~obj.certificate_stale.STOP_GEOMETRY_4MS_CERT, ...
                    ['[REPLANNER DEPENDENCY ERROR] 4-ms stop-geometry certificate ' ...
                     'became stale during Flight. Rebuild outside the Flight cycle.']);

                if strcmpi(char(obj.active_reference_mode),'REPAIRED')
                    assert(~obj.certificate_stale.ACTUATOR_DYNAMICS_CERT, ...
                        ['[REPLANNER DEPENDENCY ERROR] REPAIRED actuator/dynamics ' ...
                         'certificate became stale during Flight.']);
                    assert(~obj.certificate_stale.HLC_ROBUST_CERT, ...
                        ['[REPLANNER DEPENDENCY ERROR] REPAIRED HLC robust ' ...
                         'certificate became stale during Flight.']);
                end
            end
        end


        function assert_runtime_core_certificates_current(obj)
            obj.assert_trigger_packet_certificate_current();

            assert(isstruct(obj.sensor_horizon_certificate) && ...
                isfield(obj.sensor_horizon_certificate,'certified') && ...
                logical(obj.sensor_horizon_certificate.certified) && ...
                ~obj.certificate_stale.SENSOR_HORIZON_CERT, ...
                '[REPLANNER ERROR] sensor/horizon certificate is unavailable or stale.');

            switch upper(char(obj.active_reference_mode))
                case 'NOMINAL'
                    assert(isstruct(obj.stop_geometry_4ms_certificate) && ...
                        isfield(obj.stop_geometry_4ms_certificate,'certified') && ...
                        logical(obj.stop_geometry_4ms_certificate.certified) && ...
                        ~obj.certificate_stale.STOP_GEOMETRY_4MS_CERT, ...
                        '[REPLANNER ERROR] NOMINAL 4-ms stop-geometry certificate is unavailable or stale.');

                case 'REPAIRED'
                    obj.assert_repaired_reference_current();
                    assert(~obj.certificate_stale.STOP_GEOMETRY_4MS_CERT && ...
                        ~obj.certificate_stale.ACTUATOR_DYNAMICS_CERT && ...
                        ~obj.certificate_stale.HLC_ROBUST_CERT, ...
                        '[REPLANNER ERROR] active REPAIRED safety certificate became stale.');

                otherwise
                    error('[REPLANNER ERROR] unknown active_reference_mode.');
            end
        end


        function cert = build_sensor_horizon_certificate(obj)
            obj.validate_active_speed_certificate(obj.active_speed_certificate);

            VQ=double(obj.active_speed_certificate.uav_speed_UB_m_s);
            VL=double(obj.active_speed_certificate.payload_speed_UB_m_s);
            dt=double(obj.prediction_dt);
            H=double(obj.horizon_s);
            Rs=double(obj.trigger_dist);
            rho=max([double(obj.r_drone), ...
                     double(obj.L_cable)+double(obj.r_load), ...
                     double(obj.L_cable)+double(obj.r_cable)]);

            horizonMargin=Rs-(rho+VQ*H);
            dDetLB=Rs-VQ*dt;
            safeClear=dDetLB-rho;

            if VQ>0
                collisionLead=safeClear/VQ;
                beforeHorizon=collisionLead-H;
            else
                collisionLead=Inf;
                beforeHorizon=Inf;
            end

            cert=struct();
            cert.certified= ...
                horizonMargin>0 && safeClear>0 && beforeHorizon>0;
            cert.reference_mode=char(obj.active_reference_mode);
            cert.reference_version=double(obj.active_reference_version);
            cert.VL=VL;
            cert.VQ=VQ;
            cert.dt=dt;
            cert.sensor_range=Rs;
            cert.horizon=H;
            cert.rho_body=rho;
            cert.sensor_horizon_margin_m=horizonMargin;
            cert.first_detection_distance_LB_m=dDetLB;
            cert.first_collision_lead_LB_s=collisionLead;
            cert.before_horizon_entry_LB_s=beforeHorizon;
            cert.source='LIVE_SPEED_SENSOR_GEOMETRY_THEOREM';
        end


        function cert = build_stop_geometry_4ms_certificate(obj)
            obj.validate_active_speed_certificate(obj.active_speed_certificate);

            v=double(obj.active_speed_certificate.payload_speed_UB_m_s);
            VQ=double(obj.active_speed_certificate.uav_speed_UB_m_s);
            dt=double(obj.prediction_dt);
            Rs=double(obj.trigger_dist);
            g=double(obj.gravity);
            L=double(obj.L_cable);
            rho=max([double(obj.r_drone), ...
                     L+double(obj.r_load), ...
                     L+double(obj.r_cable)]);

            C1=3003/1024;
            k=v*C1;
            dDetLB=Rs-VQ*dt;
            clearanceAfterDeadline= ...
                dDetLB-rho-VQ*double(obj.hard_modify_deadline_s);

            INT=obj.geometric_stop_T_intervals( ...
                v,k,g,L,clearanceAfterDeadline);

            cert=struct();
            cert.certified=clearanceAfterDeadline>0 && INT.nonempty;
            cert.reference_mode=char(obj.active_reference_mode);
            cert.reference_version=double(obj.active_reference_version);
            cert.C1=C1;
            cert.VQ=VQ;
            cert.VL=v;
            cert.dt=dt;
            cert.hard_modify_deadline_s=double(obj.hard_modify_deadline_s);
            cert.rho_body=rho;
            cert.first_detection_distance_LB_m=dDetLB;
            cert.clearance_after_deadline_m=clearanceAfterDeadline;
            cert.T_flat=k/g;
            cert.INTERVALS=INT;
            cert.source='LIVE_4MS_ORIENTATION_INDEPENDENT_STOP_GEOMETRY';
        end


        function cert = build_active_command_limit_certificate(obj)
            % =============================================================
            % Source-truth command-limit certificate.
            %
            % IMPORTANT INITIALIZATION ORDER:
            %   run_flight_headless constructs REPLANNING_SYSTEM before the
            %   live HLC_SUSPENDED_LOAD object is attached to agent.controller.
            %   Therefore this constructor-safe builder MUST NOT require the
            %   live controller object.
            %
            %   1) Parse the authoritative HLC_SUSPENDED_LOAD class source.
            %   2) Later verify that the live controller object is really the
            %      same class/source via verify_live_controller_binding().
            % =============================================================

            expected_class='HLC_SUSPENDED_LOAD';
            fp=which(expected_class);

            assert(~isempty(fp) && isfile(fp), ...
                '[REPLANNER ERROR] HLC_SUSPENDED_LOAD source file unresolved.');

            txt=fileread(fp);
            L=regexp(txt,'\r\n|\n|\r','split');
            hit=[];

            for i=1:numel(L)
                s=strtrim(L{i});
                if isempty(s) || startsWith(s,'%')
                    continue;
                end
                if contains(s,'obj.result.input') && ...
                        contains(s,'tmp(1)') && contains(s,'tmp(2)') && ...
                        contains(s,'tmp(3)') && contains(s,'tmp(4)')
                    hit(end+1)=i; %#ok<AGROW>
                end
            end

            assert(numel(hit)==1, ...
                '[REPLANNER ERROR] active command clamp source line is not unique.');

            lineNo=hit(1);
            lineText=strtrim(L{lineNo});
            LIM=zeros(4,2);

            for j=1:4
                num='([-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?)';
                pat=['max\(\s*' num '\s*,\s*min\(\s*' num ...
                     '\s*,\s*tmp\(\s*' num2str(j) '\s*\)\s*\)\s*\)'];
                tok=regexp(lineText,pat,'tokens','once');
                assert(~isempty(tok), ...
                    '[REPLANNER ERROR] cannot parse active clamp tmp(%d).',j);
                LIM(j,:)=[str2double(tok{1}),str2double(tok{2})];
            end

            cert=struct();
            cert.certified=true;
            cert.controller_class=expected_class;
            cert.controller_source=fp;
            cert.source_line_number=lineNo;
            cert.source_line_signature=lineText;
            cert.command_order={'F','Mx','My','Mz'};
            cert.limits=LIM;
            cert.source='HLC_COMMAND_CLAMP_SOURCE_TRUTH';
            cert.live_controller_verified=false;
            cert.live_controller_path='';
            cert.initialization_binding='CLASS_SOURCE_BEFORE_LIVE_CONTROLLER';
        end


        function verify_live_controller_binding(obj,required_now)
            % =============================================================
            % Verify the runtime controller handle WITHOUT reparsing source.
            %
            % During constructor / early setup the controller may legitimately
            % not exist yet.  This is not a safety failure.
            %
            % Before Flight, however, the live controller MUST exist and MUST
            % match the class/source used by command_limit_certificate.
            % =============================================================
            if nargin<2
                required_now=false;
            end

            cert=obj.command_limit_certificate;
            assert(isstruct(cert) && isfield(cert,'certified') && ...
                logical(cert.certified), ...
                '[REPLANNER ERROR] command-limit source certificate unavailable.');

            if isfield(cert,'live_controller_verified') && ...
                    logical(cert.live_controller_verified)
                return;
            end

            available=false;
            ctrl=[];

            if obj.has_prop_or_field(obj.self,'controller') && ...
                    ~isempty(obj.self.controller) && ...
                    obj.has_prop_or_field(obj.self.controller,'hlc_suspended') && ...
                    ~isempty(obj.self.controller.hlc_suspended)
                ctrl=obj.self.controller.hlc_suspended;
                available=true;
            end

            if ~available
                if required_now
                    error(['[REPLANNER ERROR] Flight reached before live ' ...
                        'HLC_SUSPENDED_LOAD controller binding became available.']);
                end
                return;
            end

            assert(isa(ctrl,cert.controller_class), ...
                '[REPLANNER ERROR] live controller class differs from command-limit certificate.');

            live_source=which(class(ctrl));
            assert(~isempty(live_source) && isfile(live_source), ...
                '[REPLANNER ERROR] live controller source file unresolved.');

            assert(strcmp(live_source,cert.controller_source), ...
                ['[REPLANNER ERROR] live HLC controller source differs from ' ...
                 'the source used for command-limit certification.']);

            cert.live_controller_verified=true;
            cert.live_controller_path='self.controller.hlc_suspended';
            cert.live_controller_class=class(ctrl);
            cert.live_controller_source=live_source;
            obj.command_limit_certificate=cert;
        end


        function cert = build_current_actuator_dynamics_certificate(obj)
            cmd=obj.command_limit_certificate;
            assert(isstruct(cmd) && isfield(cmd,'certified') && ...
                logical(cmd.certified), ...
                '[REPLANNER ERROR] command limit certificate unavailable.');

            sp=obj.active_speed_certificate;
            obj.validate_active_speed_certificate(sp);

            cert=struct();
            cert.certified=false;
            cert.reference_mode=char(obj.active_reference_mode);
            cert.reference_version=double(obj.active_reference_version);
            cert.mQ=obj.get_vehicle_mass();
            cert.mL_snapshot=double(obj.loadmass);
            cert.invalidate_on_mL_change=true;
            cert.command_limit_source_signature=cmd.source_line_signature;
            cert.command_limits=cmd.limits;
            cert.production_T_selected=false;

            if ~strcmpi(char(obj.active_reference_mode),'NOMINAL') || ...
                    ~isfield(sp,'payload_velocity_vector_m_s') || ...
                    ~isfield(sp,'nominal_acceleration_zero') || ...
                    ~logical(sp.nominal_acceleration_zero)
                cert.reason='current exact vertical NOMINAL stop theorem not applicable';
                return;
            end

            vv=double(sp.payload_velocity_vector_m_s(:));
            if numel(vv)~=3 || vv(1)~=0 || vv(2)~=0 || vv(3)<=0
                cert.reason='current reference is not the certified positive-vertical constant-velocity case';
                return;
            end

            g=double(obj.gravity);
            v=norm(vv);
            C1=3003/1024;
            k=v*C1;
            mt=cert.mQ+cert.mL_snapshot;

            fmin=cmd.limits(1,1);
            fmax=cmd.limits(1,2);
            fReqMax=mt*g;

            den=g-fmin/mt;
            if den>0
                Tthrust=k/den;
            else
                Tthrust=Inf;
            end

            Tflat=k/g;
            Tlower=max(Tflat,Tthrust);

            zeroMomentOK= ...
                cmd.limits(2,1)<=0 && 0<=cmd.limits(2,2) && ...
                cmd.limits(3,1)<=0 && 0<=cmd.limits(3,2) && ...
                cmd.limits(4,1)<=0 && 0<=cmd.limits(4,2);

            cert.total_mass=mt;
            cert.hover_thrust=fReqMax;
            cert.upper_thrust_feasible=fReqMax<=fmax;
            cert.zero_moment_feasible=zeroMomentOK;
            cert.C1=C1;
            cert.T_flat=Tflat;
            cert.T_thrust_lower=Tthrust;
            cert.T_actuator_theorem_lower=Tlower;

            % This certificate is deliberately independent of sensor range,
            % horizon, radii and the 4-ms geometric clearance.  The
            % intersection with STOP_GEOMETRY_4MS_CERT belongs to the later
            % backup/HLC certificate.
            cert.certified= ...
                cert.upper_thrust_feasible && ...
                cert.zero_moment_feasible && isfinite(Tlower);
            cert.source='CURRENT_MASS_ACTIVE_CLAMP_VERTICAL_STOP_DYNAMICS';

            if cert.certified
                cert.reason='';
            else
                cert.reason='current mass/command limits do not admit the certified vertical-stop dynamics';
            end
        end


        function mQ = get_vehicle_mass(obj)
            P=obj.self.parameter;
            mQ=[];

            if isobject(P) && isprop(P,'mass')
                mQ=P.mass;
            elseif isstruct(P) && isfield(P,'mass')
                mQ=P.mass;
            end

            assert(isnumeric(mQ) && isscalar(mQ) && ...
                isfinite(mQ) && mQ>0, ...
                '[REPLANNER ERROR] fixed vehicle mass parameter is unavailable.');
            mQ=double(mQ);
        end


        function sig = serialize_numeric_struct(~,S)
            fn=sort(fieldnames(S));
            parts={};

            for i=1:numel(fn)
                n=fn{i};
                v=S.(n);

                if isnumeric(v) || islogical(v)
                    parts{end+1}=[n '=' mat2str(v,17)]; %#ok<AGROW>
                elseif ischar(v) || isstring(v)
                    parts{end+1}=[n '=' char(string(v))]; %#ok<AGROW>
                end
            end

            sig=strjoin(parts,'|');
        end


        function INT = geometric_stop_T_intervals(~,v,k,g,L,S)
            INT=struct();
            INT.branch1=zeros(0,2);
            INT.branch1_right_closed=false(0,1);
            INT.branch2=zeros(0,2);
            INT.nonempty=false;

            if v==0
                INT.branch1=[0,Inf];
                INT.branch1_right_closed=true;
                INT.nonempty=S>0;
                return;
            end

            Tf=k/g;
            Tb=2*Tf;

            b1upper=2*(S-2*L)/v;
            hi=min(Tb,b1upper);
            if hi>Tf
                INT.branch1=[Tf,hi];
                INT.branch1_right_closed=(Tb<b1upper);
            end

            qa=0.5*v*g;
            qb=-(0.5*v*k+S*g);
            qc=S*k+2*L*k;
            disc=qb^2-4*qa*qc;

            if disc>0
                r1=(-qb-sqrt(disc))/(2*qa);
                r2=(-qb+sqrt(disc))/(2*qa);
                lo=max(Tb,r1);
                hi=r2;

                if hi>lo
                    INT.branch2=[lo,hi];
                end
            end

            INT.nonempty=~isempty(INT.branch1) || ~isempty(INT.branch2);
        end


        function INT2 = intersect_T_intervals_with_lower(~,INT,Tlo)
            INT2=struct();
            INT2.branch1=zeros(0,2);
            INT2.branch1_right_closed=false(0,1);
            INT2.branch2=zeros(0,2);
            INT2.nonempty=false;

            for i=1:size(INT.branch1,1)
                lo=max(INT.branch1(i,1),Tlo);
                hi=INT.branch1(i,2);

                if hi>lo
                    INT2.branch1(end+1,:)=[lo,hi]; %#ok<AGROW>
                    INT2.branch1_right_closed(end+1,1)= ...
                        INT.branch1_right_closed(i); %#ok<AGROW>
                end
            end

            for i=1:size(INT.branch2,1)
                lo=max(INT.branch2(i,1),Tlo);
                hi=INT.branch2(i,2);

                if hi>lo
                    INT2.branch2(end+1,:)=[lo,hi]; %#ok<AGROW>
                end
            end

            INT2.nonempty= ...
                ~isempty(INT2.branch1) || ~isempty(INT2.branch2);
        end


        function [xd,t] = function_handle_to_symbolic_vector(~,fstr)
            prefix='@(t)';
            assert(startsWith(strtrim(fstr),prefix), ...
                ['[REPLANNER ERROR] nominal func must be an anonymous ' ...
                 'scalar-time function @(t)...']);

            body=strtrim(extractAfter(string(strtrim(fstr)), ...
                strlength(prefix)));
            body=strrep(body,'.^','^');
            body=strrep(body,'.*','*');
            body=strrep(body,'./','/');

            syms t real
            xd=str2sym(char(body));
            xd=xd(:);
        end


        function tf = symbolic_vector_equal(obj,a,b)
            r=simplify(a-b);
            tf=true;
            for k=1:numel(r)
                if ~obj.symbolic_zero(r(k))
                    tf=false;
                    return;
                end
            end
        end


        function tf = symbolic_zero(~,x)
            tf=isAlways(simplify(x)==0,'Unknown','false');
        end


        function tf = has_prop_or_field(~,x,name)
            tf = false;
            if isobject(x)
                tf = isprop(x,name);
            elseif isstruct(x)
                tf = isfield(x,name);
            end
        end


        function xd = evaluate_active_reference_xd(obj,t_now,xd_nom)
            switch upper(char(obj.active_reference_mode))
                case 'NOMINAL'
                    xd=xd_nom(:);

                case 'REPAIRED'
                    R=obj.active_repaired_reference;
                    assert(isstruct(R) && isfield(R,'valid') && logical(R.valid), ...
                        '[REPLANNER ERROR] REPAIRED mode has no valid repaired reference.');
                    assert(isfield(R,'eval_fn') && isa(R.eval_fn,'function_handle'), ...
                        '[REPLANNER ERROR] repaired reference eval_fn is unavailable.');
                    assert(isfield(R,'valid_from') && isfield(R,'valid_until') && ...
                        isfinite(R.valid_from) && ...
                        (isfinite(R.valid_until) || isinf(R.valid_until)) && ...
                        double(t_now)>=double(R.valid_from) && ...
                        double(t_now)<=double(R.valid_until), ...
                        '[REPLANNER ERROR] repaired reference queried outside certified validity interval.');

                    xd=R.eval_fn(double(t_now));
                    xd=xd(:);
                    assert(numel(xd)>=11 && all(isfinite(xd)), ...
                        '[REPLANNER ERROR] repaired reference returned invalid xd.');

                otherwise
                    error('[REPLANNER ERROR] unknown active_reference_mode: %s', ...
                        char(obj.active_reference_mode));
            end
        end


        function write_active_output(obj,t_now,xd_nom)
            xd=obj.evaluate_active_reference_xd(t_now,xd_nom);
            st=obj.result.state;
            st.xd=xd;

            if numel(xd)>=3
                st.p=xd(1:3);
            end
            if numel(xd)>=7
                st.v=xd(5:7);
            end
            if numel(xd)>=4
                st.q=[0;0;xd(4)];
            end
        end


        function [pL_ref,q_ref,source] = repaired_reference_packet(obj,t_now)
            obj.assert_repaired_reference_current();

            N=round(obj.horizon_s/obj.prediction_dt)+1;
            tq=double(t_now)+(0:N-1)*double(obj.prediction_dt);

            pL_ref=zeros(3,N);
            q_ref=zeros(3,N);

            for k=1:N
                xd=obj.active_repaired_reference.eval_fn(tq(k));
                xd=xd(:);
                assert(numel(xd)>=11 && all(isfinite(xd)), ...
                    '[REPLANNER ERROR] repaired reference packet returned invalid xd.');

                pL_ref(:,k)=xd(1:3);
                aL=xd(9:11);
                r=aL+[0;0;obj.gravity];
                nr=norm(r);
                assert(nr>0, ...
                    '[REPLANNER ERROR] repaired reference has zero flatness vector.');
                q_ref(:,k)=-r/nr;
            end

            obj.validate_reference_packet(pL_ref,q_ref,N);
            source=sprintf('REPAIRED:v%d:%s', ...
                obj.active_reference_version, ...
                char(obj.active_repaired_reference.backend));
        end


        function assert_repaired_reference_current(obj)
            R=obj.active_repaired_reference;
            C=obj.active_repaired_certificates;

            obj.assert_hard_modify_deadline_limit();

            assert(strcmpi(char(obj.active_reference_mode),'REPAIRED'), ...
                '[REPLANNER ERROR] repaired reference assertion used outside REPAIRED mode.');
            assert(isstruct(R) && isfield(R,'valid') && logical(R.valid), ...
                '[REPLANNER ERROR] repaired reference is unavailable.');
            assert(isfield(R,'reference_version') && ...
                double(R.reference_version)==double(obj.active_reference_version), ...
                '[REPLANNER ERROR] repaired reference version is stale.');
            assert(isfield(R,'eval_fn') && isa(R.eval_fn,'function_handle'), ...
                '[REPLANNER ERROR] repaired reference eval_fn is unavailable.');
            assert(isfield(R,'yaw_policy') && ...
                strcmpi(char(R.yaw_policy),'C6_INDEPENDENT'), ...
                '[REPLANNER ERROR] active repaired yaw policy is not C6_INDEPENDENT.');
            assert(isfield(R,'path_parameterization') && ...
                isstruct(R.path_parameterization), ...
                '[REPLANNER ERROR] active repaired path parameterization is unavailable.');

            required={'smoothness','geometry','dynamics','hlc_robust','backup', ...
                'position_c6','yaw_c6','path_position_c7', ...
                'time_law_c6','time_law_monotonic','terminal_time_law_hold', ...
                'finite_sensing','compute_deadline','dependency'};
            for kk=1:numel(required)
                name=required{kk};
                assert(isstruct(C) && isfield(C,name) && ...
                    isstruct(C.(name)) && ...
                    isfield(C.(name),'certified') && ...
                    logical(C.(name).certified), ...
                    '[REPLANNER ERROR] repaired reference certificate %s is unavailable/stale.', ...
                    name);
                if isfield(C.(name),'test_only')
                    assert(~logical(C.(name).test_only), ...
                        '[REPLANNER ERROR] active repaired certificate %s is test-only.',name);
                end
            end

            targetVersion=double(obj.active_reference_version);
            vf=double(R.valid_from);
            vu=double(R.valid_until);

            boundNames={'smoothness','geometry','dynamics','hlc_robust','backup', ...
                'position_c6','yaw_c6','path_position_c7', ...
                'time_law_c6','time_law_monotonic','terminal_time_law_hold', ...
                'finite_sensing'};
            for kk=1:numel(boundNames)
                name=boundNames{kk};
                obj.validate_repaired_bound_certificate( ...
                    C.(name),name,targetVersion,vf,vu);
            end

            assert(isfield(C.time_law_monotonic,'sdot_lower_bound') && ...
                double(C.time_law_monotonic.sdot_lower_bound)>=0, ...
                '[REPLANNER ERROR] active time-law monotonicity certificate is invalid.');
            assert(isfield(C.terminal_time_law_hold,'zero_derivative_order') && ...
                double(C.terminal_time_law_hold.zero_derivative_order)>=6 && ...
                isfield(C.terminal_time_law_hold,'constant_hold_after_terminal') && ...
                logical(C.terminal_time_law_hold.constant_hold_after_terminal), ...
                '[REPLANNER ERROR] active terminal time-law hold certificate is invalid.');

            obj.validate_candidate_dependency_certificate( ...
                C.dependency,R.eval_fn,targetVersion);

            obj.validate_active_speed_certificate(obj.active_speed_certificate);
        end


        function meta = repaired_reference_metadata(obj)
            R=obj.active_repaired_reference;
            if isempty(fieldnames(R))
                meta=struct('valid',false);
                return;
            end

            meta=struct();
            meta.valid=isfield(R,'valid') && logical(R.valid);
            if isfield(R,'backend'), meta.backend=R.backend; end
            if isfield(R,'reference_version'), ...
                    meta.reference_version=R.reference_version; end
            if isfield(R,'valid_from'), meta.valid_from=R.valid_from; end
            if isfield(R,'valid_until'), meta.valid_until=R.valid_until; end
            if isfield(R,'rejoin_time'), meta.rejoin_time=R.rejoin_time; end
            if isfield(R,'source'), meta.source=R.source; end
            if isfield(R,'yaw_policy'), meta.yaw_policy=R.yaw_policy; end
        end


        function write_nominal_output(obj,xd_nom)
            st = obj.result.state;
            st.xd = xd_nom;
            if numel(xd_nom)>=3
                st.p = xd_nom(1:3);
            end
            if numel(xd_nom)>=7
                st.v = xd_nom(5:7);
            end
            % 既存フレームワークとの互換性のため state.q にはヨー角情報を格納
            % （索姿勢ベクトル変数としては使用しない）
            if numel(xd_nom)>=4
                st.q = [0;0;xd_nom(4)];
            end
        end

        function [pQ_cur,pL_cur,q_cur,residual] = ...
                get_current_physical_geometry(obj,xd_nom)

            est_state=obj.self.estimator.result.state;

            if obj.has_prop_or_field(est_state,'p') && ~isempty(est_state.p)
                pQ_cur=est_state.p(:);
            else
                error('[REPLANNER ERROR] estimated UAV position p is unavailable.');
            end

            if obj.has_prop_or_field(est_state,'pL') && ~isempty(est_state.pL)
                pL_cur=est_state.pL(:);
            else
                pL_cur=[];
            end

            if obj.has_prop_or_field(est_state,'pT') && ~isempty(est_state.pT)
                q_cur=est_state.pT(:);
                nq=norm(q_cur);
                assert(nq>1e-12, ...
                    '[REPLANNER ERROR] estimated cable direction pT has zero norm.');
                q_cur=q_cur/nq;
            else
                assert(numel(xd_nom)>=11, ...
                    '[REPLANNER ERROR] pT unavailable and nominal acceleration is insufficient.');
                aL=xd_nom(9:11);
                r=aL+[0;0;obj.gravity];
                nr=norm(r);
                assert(nr>1e-12, ...
                    '[REPLANNER ERROR] nominal cable direction reconstruction is invalid.');
                q_cur=-r/nr;
            end

            if isempty(pL_cur)
                pL_cur=pQ_cur+obj.L_cable*q_cur;
            end

            residual=norm(pL_cur-pQ_cur-obj.L_cable*q_cur);
        end


        function [pL_ref,q_ref,source] = ...
                update_reference_cache(obj,time,cha,varargin_in,xd_now)

            t_entry=time.t;
            N=round(obj.horizon_s/obj.prediction_dt)+1;

            % Fast path: precomputed nominal table.
            use_nominal_table=isequal(cha,'f') && obj.nominal_table_valid && ...
                abs(obj.nominal_table_dt-obj.prediction_dt)<=1e-12 && ...
                t_entry>=obj.nominal_table_t0-1e-10 && ...
                (t_entry+obj.horizon_s)<=obj.nominal_table_tend+1e-10;

            if use_nominal_table
                idx0=round((t_entry-obj.nominal_table_t0)/obj.nominal_table_dt)+1;
                idx1=idx0+N-1;
                use_nominal_table=idx0>=1 && idx1<=size(obj.nominal_table_pL,2);
            end

            if use_nominal_table
                pL_ref=obj.nominal_table_pL(:,idx0:idx1);
                q_ref=obj.nominal_table_q(:,idx0:idx1);
                [pL0,q0]=obj.nominal_payload_and_q_from_xd(xd_now);
                pL_ref(:,1)=pL0;
                q_ref(:,1)=q0;
                source='precomputed_nominal_table';
                return;
            end

            tvec=t_entry+(0:N-1)*obj.prediction_dt;

            if ~isempty(obj.reference_packet_fn)
                callback_ok=false;
                try
                    [pL_ref,q_ref]=obj.reference_packet_fn(tvec);
                    callback_ok=isnumeric(pL_ref) && isnumeric(q_ref);
                catch
                    callback_ok=false;
                end

                if ~callback_ok
                    out=obj.reference_packet_fn(tvec);
                    if isstruct(out)
                        assert(isfield(out,'pL') && isfield(out,'q'), ...
                            '[REPLANNER ERROR] reference_packet_fn struct must contain pL and q.');
                        pL_ref=out.pL;
                        q_ref=out.q;
                    elseif iscell(out) && numel(out)>=2
                        pL_ref=out{1};
                        q_ref=out{2};
                    else
                        error('[REPLANNER ERROR] invalid reference_packet_fn output.');
                    end
                end

                obj.validate_reference_packet(pL_ref,q_ref,N);
                source='reference_packet_fn';
                return;
            end

            can_shift=obj.reference_cache_valid && ...
                abs(obj.reference_cache.dt-obj.prediction_dt)<=1e-12 && ...
                abs(obj.reference_cache.H-obj.horizon_s)<=1e-12 && ...
                isfield(obj.reference_cache,'cha') && isequal(obj.reference_cache.cha,cha) && ...
                abs(t_entry-(obj.reference_cache.t0+obj.prediction_dt)) <= ...
                    1e-5*max(1,abs(obj.prediction_dt));

            if can_shift
                pL_ref=[obj.reference_cache.pL(:,2:end),zeros(3,1)];
                q_ref=[obj.reference_cache.q(:,2:end),zeros(3,1)];

                [pL_new,q_new]=obj.evaluate_nominal_payload_and_q( ...
                    tvec(end),cha,varargin_in,[]);
                pL_ref(:,end)=pL_new;
                q_ref(:,end)=q_new;

                [pL0,q0]=obj.nominal_payload_and_q_from_xd(xd_now);
                pL_ref(:,1)=pL0;
                q_ref(:,1)=q0;
                source='rolling_base_ref_do_1_new_endpoint';
            else
                pL_ref=zeros(3,N);
                q_ref=zeros(3,N);
                [pL0,q0]=obj.nominal_payload_and_q_from_xd(xd_now);
                pL_ref(:,1)=pL0;
                q_ref(:,1)=q0;

                for j=2:N
                    [pL_ref(:,j),q_ref(:,j)]= ...
                        obj.evaluate_nominal_payload_and_q( ...
                            tvec(j),cha,varargin_in,[]);
                end
                source='full_base_ref_do_fill';
            end

            obj.validate_reference_packet(pL_ref,q_ref,N);

            assert(abs(time.t-t_entry)<=1e-12, ...
                '[REPLANNER BUG] time.t changed during future reference evaluation.');

            obj.base_ref.do(varargin_in{:});

            obj.reference_cache=struct( ...
                't0',t_entry, ...
                'dt',obj.prediction_dt, ...
                'H',obj.horizon_s, ...
                'cha',cha, ...
                'pL',pL_ref, ...
                'q',q_ref);
            obj.reference_cache_valid=true;
        end


        function [pL,q] = evaluate_nominal_payload_and_q( ...
                obj,t_query,cha,varargin_in,~)
            args = varargin_in;
            time_orig = args{1};
            % time は TIME < handle の場合があるため、本体の time.t を絶対に破壊しないよう
            % 独立した時間クエリ用オブジェクトまたは構造体を生成して渡す
            if isa(time_orig, 'TIME')
                tlocal = TIME(t_query, time_orig.dt, time_orig.te);
                if isprop(time_orig, 'k') || isfield(time_orig, 'k')
                    tlocal.k = time_orig.k;
                end
            elseif isobject(time_orig)
                try
                    tlocal = copy(time_orig);
                    tlocal.t = t_query;
                catch
                    tlocal = struct('t', t_query, 'dt', obj.prediction_dt);
                end
            elseif isstruct(time_orig)
                tlocal = time_orig;
                tlocal.t = t_query;
            else
                tlocal = struct('t', t_query, 'dt', obj.prediction_dt);
            end
            args{1} = tlocal;
            args{2} = cha;
            r = obj.base_ref.do(args{:});
            xd = r.state.xd;
            [pL,q] = obj.nominal_payload_and_q_from_xd(xd);
        end

        function [pL,q] = nominal_payload_and_q_from_xd(obj,xd)
            assert(numel(xd)>=11, ...
                ['[REPLANNER ERROR] 公称軌道フォールバックは xd(1:3)=pL および ' ...
                 'xd(9:11)=ddot(pL) を仮定しています。それ以外の場合は opts.reference_packet_fn を指定してください。']);
            pL = xd(1:3);
            aL = xd(9:11);
            r = aL + [0;0;obj.gravity];
            nr = norm(r);
            assert(nr>1e-12, ...
                '[REPLANNER ERROR] 将来の索姿勢 q の再構成計算が無効です。');
            q = -r/nr;
        end

        function validate_reference_packet(~,pL,q,N)
            assert(isnumeric(pL) && isequal(size(pL),[3 N]) && ...
                all(isfinite(pL(:))), ...
                '[REPLANNER ERROR] pL 参照パケットは有限な 3×N 行列である必要があります。');
            assert(isnumeric(q) && isequal(size(q),[3 N]) && ...
                all(isfinite(q(:))), ...
                '[REPLANNER ERROR] q 参照パケットは有限な 3×N 行列である必要があります。');
            qn = vecnorm(q,2,1);
            assert(max(abs(qn-1))<=1e-8, ...
                '[REPLANNER ERROR] 将来 q パケットに単位ベクトルでないものが含まれています。');
        end

        function [active,min_dist,min_id,dists] = raw_sensor_gate(obj,pQ,OBS)
            M=numel(OBS);
            active=false(1,M);
            dists=inf(1,M);
            min_dist=inf;
            min_id=NaN;

            for i=1:M
                d=obj.raw_surface_distance(pQ,OBS(i));
                dists(i)=d;
                if d<=obj.trigger_dist
                    active(i)=true;
                    if d<min_dist
                        min_dist=d;
                        min_id=OBS(i).id;
                    end
                end
            end
        end


        function d = raw_surface_distance(obj,pW,o)
            type = lower(o.type);
            c = o.p_center(:);
            R = o.R_obs;
            z = R.'*(pW-c);
            switch type
                case 'sphere'
                    r = o.raw_param(1);
                    d = max(0,norm(z)-r);
                case 'cylinder'
                    r = o.raw_param(1);
                    h = o.raw_param(2);
                    dr = max(hypot(z(1),z(2))-r,0);
                    dz = max(abs(z(3))-h/2,0);
                    d = hypot(dr,dz);
                case 'box'
                    dims = o.raw_param(:);
                    assert(numel(dims)==3);
                    q = abs(z)-dims/2;
                    d = norm(max(q,0));
                case 'ellipsoid'
                    a = o.raw_param(:);
                    assert(numel(a)==3 && all(a>0));
                    d = obj.point_ellipsoid_distance_local(z,a);
                case 'cone'
                    r = o.raw_param(1);
                    h = o.raw_param(2);
                    rho = hypot(z(1),z(2));
                    % 円錐断面 (底面 z=-h/2, 頂点 z=+h/2)
                    P = [0, h/2; r,-h/2; 0,-h/2];
                    if obj.point_in_triangle_2d([rho,z(3)],P)
                        d=0;
                    else
                        d=min([ ...
                            obj.point_segment_distance_2d([rho,z(3)],P(1,:),P(2,:)), ...
                            obj.point_segment_distance_2d([rho,z(3)],P(2,:),P(3,:)), ...
                            obj.point_segment_distance_2d([rho,z(3)],P(3,:),P(1,:))]);
                    end
                case {'prism','pyramid'}
                    [V,F] = obj.poly_vertices_faces_world(o);
                    d = obj.point_convex_polyhedron_distance(pW,V,F);
                case 'custom'
                    V = o.raw_param;
                    assert(isnumeric(V) && size(V,1)==3 && size(V,2)>=4, ...
                        '[REPLANNER ERROR] カスタム障害物の raw_param は 3×N の頂点座標である必要があります。');
                    try
                        F = convhulln(V.');
                    catch ME
                        error('[REPLANNER ERROR] カスタム障害物の凸包生成に失敗しました: %s',ME.message);
                    end
                    d = obj.point_convex_polyhedron_distance(pW,V,F);
                otherwise
                    error('[REPLANNER ERROR] 未対応の障害物タイプです: %s',o.type);
            end
        end

        function d = point_ellipsoid_distance_local(~,z,a)
            z = z(:);
            a = a(:);
            if sum((z./a).^2) <= 1
                d=0;
                return;
            end
            a2 = a.^2;
            F = @(lam) sum((a2.*z.^2)./(lam+a2).^2)-1;
            lo=0;
            hi=max(a2)*max(1,norm(z./a));
            while F(hi)>0
                hi=2*hi+max(a2);
            end
            for k=1:80
                mid=0.5*(lo+hi);
                if F(mid)>0
                    lo=mid;
                else
                    hi=mid;
                end
            end
            lam=0.5*(lo+hi);
            x=(a2.*z)./(lam+a2);
            d=norm(z-x);
        end

        function tf = point_in_triangle_2d(~,p,T)
            a=T(1,:); b=T(2,:); c=T(3,:);
            u=b-a; w=p-a;
            s1=u(1)*w(2)-u(2)*w(1);
            u=c-b; w=p-b;
            s2=u(1)*w(2)-u(2)*w(1);
            u=a-c; w=p-c;
            s3=u(1)*w(2)-u(2)*w(1);
            tol=1e-12;
            tf = (s1>=-tol && s2>=-tol && s3>=-tol) || ...
                 (s1<= tol && s2<= tol && s3<= tol);
        end

        function d = point_segment_distance_2d(~,p,a,b)
            v=b-a;
            den=dot(v,v);
            if den<=eps
                d=norm(p-a);
                return;
            end
            u=dot(p-a,v)/den;
            u=min(1,max(0,u));
            d=norm(p-(a+u*v));
        end

        function [V,F] = poly_vertices_faces_world(~,o)
            c=o.p_center(:);
            R=o.R_obs;
            type=lower(o.type);
            switch type
                case 'prism'
                    s=o.raw_param(1);
                    h=o.raw_param(2);
                    rc=s/sqrt(3);
                    ang=[pi/2,pi/2+2*pi/3,pi/2+4*pi/3];
                    xy=[rc*cos(ang);rc*sin(ang)];
                    Vloc=[ ...
                        xy,xy; ...
                        -h/2*ones(1,3),h/2*ones(1,3)];
                case 'pyramid'
                    s=o.raw_param(1);
                    h=o.raw_param(2);
                    Vloc=[ ...
                        -s/2, s/2, s/2,-s/2,0; ...
                        -s/2,-s/2, s/2, s/2,0; ...
                        -h/2,-h/2,-h/2,-h/2,h/2];
                otherwise
                    error('poly_vertices_faces_world: 未対応のタイプです。');
            end
            V=c+R*Vloc;
            F=convhulln(V.');
        end

        function d = point_convex_polyhedron_distance(obj,p,V,F)
            p=p(:);
            center=mean(V,2);
            inside=true;
            minFace=inf;
            for i=1:size(F,1)
                tri=V(:,F(i,:));
                n=cross(tri(:,2)-tri(:,1),tri(:,3)-tri(:,1));
                nn=norm(n);
                if nn<=1e-14
                    continue;
                end
                n=n/nn;
                % 外向き法線ベクトルの向き合わせ
                if dot(n,center-tri(:,1))>0
                    n=-n;
                end
                if dot(n,p-tri(:,1))>1e-10
                    inside=false;
                end
                dt=obj.point_triangle_distance_3d(p,tri(:,1),tri(:,2),tri(:,3));
                minFace=min(minFace,dt);
            end
            if inside
                d=0;
            else
                d=minFace;
            end
        end

        function d = point_triangle_distance_3d(~,p,a,b,c)
            ab=b-a;
            ac=c-a;
            ap=p-a;
            d1=dot(ab,ap);
            d2=dot(ac,ap);
            if d1<=0 && d2<=0
                d=norm(ap);
                return;
            end
            bp=p-b;
            d3=dot(ab,bp);
            d4=dot(ac,bp);
            if d3>=0 && d4<=d3
                d=norm(bp);
                return;
            end
            vc=d1*d4-d3*d2;
            if vc<=0 && d1>=0 && d3<=0
                v=d1/(d1-d3);
                d=norm(p-(a+v*ab));
                return;
            end
            cp=p-c;
            d5=dot(ab,cp);
            d6=dot(ac,cp);
            if d6>=0 && d5<=d6
                d=norm(cp);
                return;
            end
            vb=d5*d2-d1*d6;
            if vb<=0 && d2>=0 && d6<=0
                w=d2/(d2-d6);
                d=norm(p-(a+w*ac));
                return;
            end
            va=d3*d6-d5*d4;
            if va<=0 && (d4-d3)>=0 && (d5-d6)>=0
                w=(d4-d3)/((d4-d3)+(d5-d6));
                d=norm(p-(b+w*(c-b)));
                return;
            end
            denom=1/(va+vb+vc);
            v=vb*denom;
            w=vc*denom;
            x=a+ab*v+ac*w;
            d=norm(p-x);
        end

        function pack = prepare_obstacle_packet(obj,OBS)
            M=numel(OBS);
            pack=struct();
            pack.count=M;
            pack.C=zeros(3,M);
            pack.R=zeros(9,M);
            pack.Apay=zeros(3,M);
            pack.Auav=zeros(3,M);
            pack.Acable=zeros(3,M);
            pack.ids=zeros(1,M);
            pack.types=cell(1,M);

            obj.validate_active_speed_certificate( ...
                obj.active_speed_certificate);
            sweep=obj.trigger_sweep_inflation;
            assert(isstruct(sweep) && isfield(sweep,'certified') && ...
                logical(sweep.certified), ...
                '[REPLANNER ERROR] trigger sweep inflation is not certified.');

            pack.reference_mode=char(obj.active_reference_mode);
            pack.reference_version=double(obj.active_reference_version);
            pack.sweep_certified=true;
            pack.sweep_dt=sweep.dt;
            pack.delta_payload_m=sweep.payload_m;
            pack.delta_uav_m=sweep.uav_m;
            pack.delta_cable_m=sweep.cable_m;

            for i=1:M
                o=OBS(i);
                obj.validate_obstacle(o,i);
                pack.ids(i)=o.id;
                pack.C(:,i)=o.p_center(:);
                pack.R(:,i)=o.R_obs(:);
                pack.types{i}=lower(o.type);
                [Ap,Aq,Ac]=obj.component_outer_ellipsoids(o);
                pack.Apay(:,i)=Ap;
                pack.Auav(:,i)=Aq;
                pack.Acable(:,i)=Ac;
                if isfield(o,'v_center') && norm(o.v_center(:))>0
                    error(['[REPLANNER ERROR] 動的障害物 (ID=%d) が検知されました。' ...
                           '現在のトリガー予測器は静的障害物マップのみ対応しています。動きを勝手に凍結して保証することはありません。'],o.id);
                end
            end
        end

        function validate_obstacle(~,o,i)
            req={'id','type','raw_param','p_center','R_obs'};
            for k=1:numel(req)
                assert(isfield(o,req{k}), ...
                    '[REPLANNER ERROR] 障害物 %d の必須フィールド %s が存在しません。',i,req{k});
            end
            assert(numel(o.p_center)==3 && all(isfinite(o.p_center(:))));
            assert(isequal(size(o.R_obs),[3 3]) && all(isfinite(o.R_obs(:))));
            assert(norm(o.R_obs.'*o.R_obs-eye(3),'fro')<=1e-9 && ...
                abs(det(o.R_obs)-1)<=1e-9, ...
                '[REPLANNER ERROR] 障害物 %d の回転行列 R_obs が無効です。',i);
            supported={'sphere','cylinder','box','prism','cone','pyramid','ellipsoid','custom'};
            assert(any(strcmpi(o.type,supported)), ...
                '[REPLANNER ERROR] 未対応の障害物タイプです: %s',o.type);
        end

        function [Ap,Aq,Ac] = component_outer_ellipsoids(obj,o)
            % =============================================================
            % Sensor raw geometry is untouched.
            %
            % Future-trigger conservative packet only:
            % physical protection radius + existing d_margin
            % + theorem-backed intersample sweep radius.
            % =============================================================
            sweep=obj.trigger_sweep_inflation;
            assert(isstruct(sweep) && isfield(sweep,'certified') && ...
                logical(sweep.certified), ...
                '[REPLANNER ERROR] trigger intersample sweep is uncertified.');

            Dp=obj.r_load+obj.d_margin+sweep.payload_m;
            Dq=obj.r_drone+obj.d_margin+sweep.uav_m;
            Dc=obj.r_cable+obj.d_margin+sweep.cable_m;

            Ap=obj.outer_ellipsoid_with_inflation(o,Dp);
            Aq=obj.outer_ellipsoid_with_inflation(o,Dq);
            Ac=obj.outer_ellipsoid_with_inflation(o,Dc);
        end

        function A = outer_ellipsoid_with_inflation(obj,o,D)
            type=lower(o.type);
            % 円柱に対する緊密な外接楕円体 (V223/V224準拠)
            if strcmp(type,'cylinder')
                r=o.raw_param(1);
                h=o.raw_param(2);
                A=[ ...
                    sqrt(3/2)*(r+D); ...
                    sqrt(3/2)*(r+D); ...
                    sqrt(3)*(h/2+D)];
                return;
            end
            % 球に対する厳密な拡大
            if strcmp(type,'sphere')
                r=o.raw_param(1);
                A=(r+D)*ones(3,1);
                return;
            end
            base=obj.raw_enclosing_ellipsoid_axes(o);
            % 一般形状に対するミンコフスキー和包囲拡大理論:
            % E(base) (+) B(D) ⊆ lambda * E(base)
            % lambda = 1 + D / min(base)
            amin=min(base);
            assert(amin>0);
            lambda=1+D/amin;
            A=lambda*base;
        end

        function radii = raw_enclosing_ellipsoid_axes(~,o)
            type=lower(o.type);
            raw=o.raw_param;
            switch type
                case 'sphere'
                    r=raw(1);
                    radii=[r;r;r];
                case 'cylinder'
                    r=raw(1);
                    h=raw(2);
                    radii=[sqrt(3/2)*r;sqrt(3/2)*r;sqrt(3)*h/2];
                case 'box'
                    dims=raw(:);
                    radii=sqrt(3)*(dims/2);
                case 'prism'
                    s=raw(1);
                    h=raw(2);
                    rc=s/sqrt(3);
                    radii=[sqrt(2)*rc;sqrt(2)*rc;sqrt(2)*h/2];
                case 'cone'
                    r=raw(1);
                    h=raw(2);
                    rb=sqrt(r^2+(h/2)^2);
                    radii=rb*ones(3,1);
                case 'pyramid'
                    s=raw(1);
                    h=raw(2);
                    rbase=s/sqrt(2);
                    rb=sqrt(rbase^2+(h/2)^2);
                    radii=rb*ones(3,1);
                case 'ellipsoid'
                    radii=raw(:);
                case 'custom'
                    V=raw;
                    assert(size(V,1)==3 && size(V,2)>=4);
                    c=o.p_center(:);
                    Z=o.R_obs.'*(V-c);
                    mx=max(abs(Z),[],2);
                    radii=sqrt(3)*mx;
                otherwise
                    error('[REPLANNER ERROR] 未対応の障害物タイプです: %s',o.type);
            end
            assert(numel(radii)==3 && all(isfinite(radii)) && all(radii>0));
        end

        function name = get_component_name(~,code)
            switch code
                case 1
                    name='PAYLOAD';
                case 2
                    name='UAV';
                case 3
                    name='CABLE';
                otherwise
                    name='NONE';
            end
        end

        function clear_state_sensor_values(obj,t_now)
            st=obj.result.state;

            st.time=t_now;
            st.pQ=[NaN;NaN;NaN];
            st.detected_point=false;
            st.detected_obstacles=[];
            st.need_replan=false;
            st.relevant_obstacles=[];
            st.min_obstacle_id_point=NaN;
            st.min_dist_point=inf;
            st.detected_obstacle_count_point=0;

            st.trigger_ttc=NaN;
            st.trigger_collision_time=NaN;
            st.trigger_obstacle_id=NaN;
            st.trigger_component='NONE';
            st.trigger_component_code=NaN;
            st.trigger_pL_collision=[NaN;NaN;NaN];
            st.trigger_q_collision=[NaN;NaN;NaN];
            st.trigger_pQ_collision=[NaN;NaN;NaN];
            st.trigger_effective_horizon=NaN;
            st.trigger_reference_points=0;
            st.trigger_reference_source='';
            st.trigger_sensor_origin=[NaN;NaN;NaN];
            st.trigger_cable_residual=NaN;
            st.trigger_active_mask=zeros(1,numel(obj.static_obstacle_map));
            st.trigger_raw_distances=inf(1,numel(obj.static_obstacle_map));
            st.trigger_hit_index=NaN;
            st.trigger_kernel_called=0;
            st.active_reference_mode=obj.active_reference_mode;
            st.active_reference_version=obj.active_reference_version;
            st.active_speed_certificate=obj.active_speed_certificate;
            st.trigger_sweep_inflation=obj.trigger_sweep_inflation;
            st.loadmass=obj.loadmass;
            st.current_bspline=obj.current_bspline;
            st.active_repaired_reference=obj.repaired_reference_metadata();
            st.active_repaired_certificates=obj.active_repaired_certificates;
            st.last_replan_status=obj.last_replan_status;
            st.replan_commit_count=obj.replan_commit_count;
            st.last_replan_commit_time=obj.last_replan_commit_time;
            st.profiling=obj.profiling;
        end


        function [pL_ref,q_ref,source] = ...
                get_active_reference_packet(obj,time,cha,varargin_in,xd_now)

            switch upper(char(obj.active_reference_mode))
                case 'NOMINAL'
                    [pL_ref,q_ref,source]= ...
                        obj.update_reference_cache(time,cha,varargin_in,xd_now);
                case 'REPAIRED'
                    [pL_ref,q_ref,source]= ...
                        obj.repaired_reference_packet(time.t);
                otherwise
                    error('[REPLANNER ERROR] unknown active_reference_mode: %s', ...
                        char(obj.active_reference_mode));
            end
        end


        function [need_replan,ttc,collision_time,comp_name, ...
                pL_collision,q_collision,pQ_collision,jhit, ...
                obs_id,comp_code,kernel_called,kernel_ms] = ...
                trigger_core(obj,active,pL_ref,q_ref,t_now)

            kernel_called=any(active);
            triggered=false;
            jhit=NaN;
            obs_id=NaN;
            comp_code=NaN;
            kernel_ms=NaN;

            if kernel_called
                % Fail closed if active reference mode/version changed without
                % installing the matching continuous-time speed certificate
                % and rebuilding the obstacle packet.
                obj.assert_trigger_packet_certificate_current();

                if obj.ENABLE_PROFILING, tk=tic; end
                [triggered,jhit,obs_id,comp_code]= ...
                    g26_future_trigger_kernel_mex( ...
                        obj.horizon_s,obj.prediction_dt,obj.L_cable, ...
                        pL_ref,q_ref, ...
                        obj.obstacle_packet.C,obj.obstacle_packet.R, ...
                        obj.obstacle_packet.Apay,obj.obstacle_packet.Auav, ...
                        obj.obstacle_packet.Acable,obj.obstacle_packet.ids, ...
                        double(active));
                if obj.ENABLE_PROFILING, kernel_ms=1e3*toc(tk); end
            end

            N=size(pL_ref,2);
            [need_replan,ttc,collision_time,comp_name, ...
                pL_collision,q_collision,pQ_collision,jhit]= ...
                obj.decode_trigger_result( ...
                    triggered,jhit,obs_id,comp_code,N,pL_ref,q_ref,t_now);
        end


        function [need_replan,ttc,collision_time,comp_name, ...
                pL_collision,q_collision,pQ_collision,jhit] = ...
                decode_trigger_result(obj,triggered,jhit,obs_id,comp_code, ...
                N,pL_ref,q_ref,t_now)

            need_replan=logical(triggered);
            ttc=NaN;
            collision_time=NaN;
            comp_name='NONE';
            pL_collision=[NaN;NaN;NaN];
            q_collision=[NaN;NaN;NaN];
            pQ_collision=[NaN;NaN;NaN];

            if need_replan
                jhit=round(jhit);
                assert(jhit>=1 && jhit<=N, ...
                    '[REPLANNER ERROR] MEX returned invalid collision index.');

                ttc=(jhit-1)*obj.prediction_dt;
                collision_time=t_now+ttc;
                comp_name=obj.get_component_name(comp_code);
                pL_collision=pL_ref(:,jhit);
                q_collision=q_ref(:,jhit);
                pQ_collision=pL_collision-obj.L_cable*q_collision;
            end
        end


        function ref = get_reference_object(obj, ref_name)
            % =================================================================
            % 【参照オブジェクト取得関数】
            %   agent.reference から指定された名前 (ref_name) の参照軌道オブジェクトを取得します。
            %   agent.reference が struct の場合も dynamicprops (FUNCTION_CLASS_PROPERTY 等)
            %   の場合も、MATLAB では共通してドット動的構文 R.(ref_name) で直接安全にプロパティや
            %   フィールドを取得可能です。冗長な if isstruct(R) 分岐を解消し、高速かつ簡潔に参照を取得します。
            % =================================================================
            assert(obj.has_prop_or_field(obj.self, 'reference'), ...
                '[REPLANNER ERROR] agent.reference が存在しません。');
            R = obj.self.reference;
            assert(obj.has_prop_or_field(R, ref_name), ...
                '[REPLANNER ERROR] reference.%s が存在しません。', ref_name);
            ref = R.(ref_name);
        end

        function te = resolve_takeoff_end_time(obj)
            if isfinite(obj.takeoff_end_s) && obj.takeoff_end_s>0
                te=obj.takeoff_end_s;
                return;
            end

            ref=obj.get_reference_object('takeoff');
            te=NaN;

            candidates={'te','t_end','tend','duration','T'};
            for k=1:numel(candidates)
                nm=candidates{k};
                try
                    if isprop(ref,nm)
                        v=ref.(nm);
                        if isnumeric(v) && isscalar(v) && isfinite(v) && v>0
                            te=double(v);
                            break;
                        end
                    elseif isstruct(ref) && isfield(ref,nm)
                        v=ref.(nm);
                        if isnumeric(v) && isscalar(v) && isfinite(v) && v>0
                            te=double(v);
                            break;
                        end
                    end
                catch
                end
            end

            if ~isfinite(te)
                for holder={'param','parameter','params','option','opts'}
                    hn=holder{1};
                    try
                        if obj.has_prop_or_field(ref,hn)
                            H=ref.(hn);
                            for k=1:numel(candidates)
                                nm=candidates{k};
                                if isstruct(H) && isfield(H,nm)
                                    v=H.(nm);
                                    if isnumeric(v) && isscalar(v) && isfinite(v) && v>0
                                        te=double(v);
                                        break;
                                    end
                                end
                            end
                        end
                    catch
                    end
                    if isfinite(te), break; end
                end
            end

            assert(isfinite(te) && te>0, ...
                ['[REPLANNER ERROR] TAKEOFF_REFERENCE の終了時刻 te を取得できません。' ...
                 ' opts.takeoff_end_s に run と同一の値を明示してください。']);
            obj.takeoff_end_s=te;
        end

        function tf = is_flight_warmup_context(obj)
            tf=false;
            try
                if obj.has_prop_or_field(obj.self,'cha')
                    c=obj.self.cha;
                    if isstring(c), c=char(c); end
                    tf=ischar(c) && ~isempty(c) && c(1)=='f';
                end
            catch
                tf=false;
            end
        end

        function build_nominal_reference_table(obj,time,varargin_in,~)
            % =================================================================
            % 【公称軌道テーブル一括事前生成関数】
            %   飛行開始前 (Setup/Arming) に飛行全区間 [t0, tend] の公称軌道 [pL, q] を
            %   一括評価して行列にキャッシュします。
            %
            % 【スパイク防止の工夫】
            %   定常飛行中に公称軌道を毎周期再サンプリング・微分計算すると大きな計算時間
            %   スパイクが発生するため、離陸前に全区間を一括計算しておくことで、飛行中は
            %   O(1) の高速メモリスライス参照のみで即座に参照パケットを取得できるようにします。
            % =================================================================
            if obj.nominal_table_valid || ~isempty(obj.reference_packet_fn)
                return;
            end

            if ~isa(obj.base_ref,'TIME_VARYING_REFERENCE')
                return;
            end

            te=[];
            if isstruct(time)
                if isfield(time,'te'), te=time.te; end
            elseif isobject(time)
                if isprop(time,'te'), te=time.te; end
            end
            if isempty(te) || ~isscalar(te) || ~isfinite(te)
                return;
            end

            t0=time.t;
            tend=te+obj.horizon_s;
            if tend<t0
                return;
            end

            Ntab=round((tend-t0)/obj.prediction_dt)+1;
            assert(Ntab>=2,'[REPLANNER ERROR] invalid nominal table size.');

            pLtab=zeros(3,Ntab);
            qtab=zeros(3,Ntab);

            tb=tic;
            for j=1:Ntab
                tq=t0+(j-1)*obj.prediction_dt;
                [pLtab(:,j),qtab(:,j)]= ...
                    obj.evaluate_nominal_payload_and_q( ...
                        tq,'f',varargin_in,[]);
            end

            obj.base_ref.do(varargin_in{:});

            obj.nominal_table_t0=t0;
            obj.nominal_table_tend=t0+(Ntab-1)*obj.prediction_dt;
            obj.nominal_table_dt=obj.prediction_dt;
            obj.nominal_table_pL=pLtab;
            obj.nominal_table_q=qtab;
            obj.nominal_table_eval_count=Ntab;
            obj.nominal_table_build_ms=1e3*toc(tb);
            obj.nominal_table_valid=true;
            obj.profiling.t_nominal_table_build_ms=obj.nominal_table_build_ms;

            if obj.ENABLE_ALL_LOG
                fprintf('[REPLANNING_SYSTEM] nominal table ready | %.3f--%.3f s | points=%d | %.3f ms\n', ...
                    obj.nominal_table_t0,obj.nominal_table_tend,Ntab,obj.nominal_table_build_ms);
            end
        end


        function tq = make_time_query(obj,time_orig,t_query)
            % =================================================================
            % 【非破壊時間クエリオブジェクト生成関数】
            %   公称軌道の将来時刻評価を行う際、シミュレーション本体の時間オブジェクト (time)
            %   の内部時刻 t を絶対に書き換えないよう、独立した時間クエリ構造体・オブジェクトを生成します。
            %
            % 【重要性】
            %   time が TIME < handle の場合、直接 time.t = t_query と代入すると本体の時計が狂い、
            %   シミュレーションが即座に破綻します。本関数により本体時刻の非破壊性を厳格に保証します。
            % =================================================================
            if isa(time_orig,'TIME')
                tq=TIME(t_query,time_orig.dt,time_orig.te);
                try
                    if isprop(time_orig,'k'), tq.k=time_orig.k; end
                catch
                end
            elseif isstruct(time_orig)
                tq=time_orig;
                tq.t=t_query;
            elseif isobject(time_orig)
                try
                    tq=copy(time_orig);
                    tq.t=t_query;
                catch
                    tq=struct('t',t_query,'dt',obj.prediction_dt);
                end
            else
                tq=struct('t',t_query,'dt',obj.prediction_dt);
            end
        end


        function warmup_online_path(obj,time,varargin_in,xd_now)
            % =================================================================
            % 【離陸前コールドパス完全事前燃焼関数 (Pre-motion Prime)】
            %   機体が動く前の静止段階で、飛行中に実行されうる全パターンの分岐・判定を
            %   網羅的に事前実行し、MATLAB JIT コンパイラと CPU キャッシュを完全に温めます。
            %
            % 【事前テストする網羅ケース】
            %   1. 障害物マスクの全パターン (0個アクティブ 〜 M個全アクティブ)
            %   2. 合成された現在衝突パス (j=1) および将来衝突パス (j=2)
            %   3. 全衝突部位コード (1: PAYLOAD, 2: UAV, 3: CABLE) のデコード処理
            %   4. 本番引数による do() の実呼び出しを 12 回連続実行 (setup_exact_do_prime)
            %
            % 【スパイク防止の工夫】
            %   MATLAB の JIT コンパイルや初回の関数バインディングは 10〜100 ms 規模の
            %   巨大な遅延スパイクを引き起こします。これを飛行開始前に 100% 消化しておくことで、
            %   定常飛行での初周遅延スパイクを根絶します。
            % =================================================================
            if obj.online_prime_done
                return;
            end

            assert(obj.nominal_table_valid, ...
                '[REPLANNER ERROR] pre-motion prime requires nominal table.');
            assert(obj.obstacle_packet.count>=1, ...
                '[REPLANNER ERROR] pre-motion prime requires at least one obstacle.');

            cache_saved=obj.reference_cache;
            cache_valid_saved=obj.reference_cache_valid;
            profile_saved=obj.profiling;

            log_master=obj.ENABLE_ALL_LOG;
            log_sensor=obj.LOG_SENSOR;
            log_trigger=obj.LOG_TRIGGER;
            obj.ENABLE_ALL_LOG=0;
            obj.LOG_SENSOR=0;
            obj.LOG_TRIGGER=0;

            M=obj.obstacle_packet.count;
            N=round(obj.horizon_s/obj.prediction_dt)+1;
            tf=obj.resolve_takeoff_end_time();

            % Prime reference-table slice exactly at Flight start.
            tprime=struct('t',tf,'dt',obj.prediction_dt);
            args_prime=varargin_in;
            args_prime{1}=tprime;
            args_prime{2}='f';

            [pL_ref,q_ref,~]= ...
                obj.get_active_reference_packet(tprime,'f',args_prime,xd_now);

            [pQ_cur,~,~,~]=obj.get_current_physical_geometry(xd_now);
            [~,~,~,all_dists]=obj.raw_sensor_gate(pQ_cur,obj.static_obstacle_map);

            masks=false(M+1,M);
            for nactive=1:M
                masks(nactive+1,1:nactive)=true;
            end

            qsyn=repmat([0;0;-1],1,N);
            C1=obj.obstacle_packet.C(:,1);
            far=C1+[1000;1000;1000];

            pL_hit_now=repmat(far,1,N);
            pL_hit_now(:,1)=C1;

            pL_hit_future=repmat(far,1,N);
            if N>=2
                pL_hit_future(:,2)=C1;
            else
                pL_hit_future(:,1)=C1;
            end

            active1=false(1,M);
            active1(1)=true;

            t0=tic;
            variant_count=0;
            hit_case_count=0;

            for rep=1:max(1,obj.ONLINE_PRIME_CALLS)
                obj.get_current_physical_geometry(xd_now);
                obj.raw_sensor_gate(pQ_cur,obj.static_obstacle_map);

                for im=1:size(masks,1)
                    active=masks(im,:);
                    obj.trigger_core(active,pL_ref,q_ref,tf);
                    variant_count=variant_count+1;
                end

                [hit_now,~,~,~,~,~,~,j_now]= ...
                    obj.trigger_core(active1,pL_hit_now,qsyn,tf);
                assert(hit_now && j_now==1, ...
                    '[REPLANNER PRIME ERROR] forced current-hit path failed.');
                hit_case_count=hit_case_count+1;

                [hit_future,~,~,~,~,~,~,j_future]= ...
                    obj.trigger_core(active1,pL_hit_future,qsyn,tf);
                assert(hit_future && j_future>=1 && j_future<=min(2,N), ...
                    '[REPLANNER PRIME ERROR] forced future-hit path failed.');
                hit_case_count=hit_case_count+1;

                for cprime=1:3
                    obj.decode_trigger_result( ...
                        true,1,obj.obstacle_packet.ids(1),cprime, ...
                        N,pL_hit_now,qsyn,tf);
                end
            end

            % Exact Flight do() prime: no-hit and hit downstream paths.
            exact_args=varargin_in;
            exact_args{1}=obj.make_time_query(varargin_in{1},tf);
            exact_args{2}='f';

            obj.setup_exact_do_prime_active=true;
            obj.setup_exact_force_hit=false;
            obj.setup_exact_force_hit_calls=0;

            te=tic;
            try
                for k=1:round(obj.SETUP_EXACT_DO_PRIME_CALLS)
                    obj.setup_exact_force_hit=mod(k,2)==0;
                    if obj.setup_exact_force_hit
                        obj.setup_exact_force_hit_calls= ...
                            obj.setup_exact_force_hit_calls+1;
                    end
                    obj.do(exact_args{:});
                end
                obj.setup_exact_force_hit=false;
                obj.setup_exact_do_prime_runtime_ms=1e3*toc(te);
                obj.setup_exact_do_prime_calls=round(obj.SETUP_EXACT_DO_PRIME_CALLS);
                obj.setup_exact_do_prime_done=true;
                obj.setup_exact_do_prime_active=false;
            catch ME
                obj.setup_exact_force_hit=false;
                obj.setup_exact_do_prime_active=false;
                obj.setup_exact_do_prime_done=false;
                obj.ENABLE_ALL_LOG=log_master;
                obj.LOG_SENSOR=log_sensor;
                obj.LOG_TRIGGER=log_trigger;
                rethrow(ME);
            end

            % Restore normal state after synthetic setup execution.
            try
                obj.base_ref.do(varargin_in{:});
            catch
            end
            obj.reference_cache=cache_saved;
            obj.reference_cache_valid=cache_valid_saved;
            obj.profiling=profile_saved;

            obj.clear_state_sensor_values(time.t);
            obj.write_nominal_output(xd_now);

            obj.online_prime_runtime_ms=1e3*toc(t0);
            obj.online_prime_variant_count=variant_count;
            obj.online_prime_hit_cases=hit_case_count;
            obj.online_prime_done=true;
            obj.profiling.t_setup_prime_ms=obj.online_prime_runtime_ms;

            obj.ENABLE_ALL_LOG=log_master;
            obj.LOG_SENSOR=log_sensor;
            obj.LOG_TRIGGER=log_trigger;

            if obj.ENABLE_ALL_LOG
                fprintf(['[REPLANNING_SYSTEM] pre-motion prime ready | ' ...
                    'variants=%d | hitCases=%d | exactDo=%d | forcedHit=%d | %.3f ms\n'], ...
                    obj.online_prime_variant_count,obj.online_prime_hit_cases, ...
                    obj.setup_exact_do_prime_calls,obj.setup_exact_force_hit_calls, ...
                    obj.online_prime_runtime_ms);
            end
        end


        function warmup_runtime(obj)
            t0=tic;
            N=round(obj.horizon_s/obj.prediction_dt)+1;
            pL=zeros(3,N);
            q=repmat([0;0;-1],1,N);
            % 本番と同じ障害物数・全格子点 N・全 active・実楕円パラメータによる MEX ウォームアップ
            if ~isempty(obj.obstacle_packet) && isfield(obj.obstacle_packet,'count') && obj.obstacle_packet.count>0
                M=obj.obstacle_packet.count;
                C_warm   = obj.obstacle_packet.C;
                R_warm   = obj.obstacle_packet.R;
                Ap_warm  = obj.obstacle_packet.Apay;
                Aq_warm  = obj.obstacle_packet.Auav;
                Ac_warm  = obj.obstacle_packet.Acable;
                ids_warm = obj.obstacle_packet.ids;
                act_warm = ones(1,M);
            else
                M=1;
                C_warm   = [100;100;100];
                R_mat    = eye(3);
                R_warm   = R_mat(:);
                Ap_warm  = [1;1;1];
                Aq_warm  = [1;1;1];
                Ac_warm  = [1;1;1];
                ids_warm = 1;
                act_warm = 1;
            end
            for k=1:obj.warmup_calls
                g26_future_trigger_kernel_mex( ...
                    obj.horizon_s,obj.prediction_dt,obj.L_cable, ...
                    pL,q,C_warm,R_warm,Ap_warm,Aq_warm,Ac_warm,ids_warm,act_warm);
            end
            % 実障害物を用いた生形状幾何計算ルーチンの JIT ウォームアップ
            if ~isempty(obj.static_obstacle_map)
                p=[0;0;0];
                for rep=1:max(1,min(3,obj.warmup_calls))
                    for i=1:numel(obj.static_obstacle_map)
                        obj.raw_surface_distance(p,obj.static_obstacle_map(i));
                    end
                end
            end
            obj.warmup_runtime_ms=1e3*toc(t0);
            obj.warmup_done=true;
            obj.profiling.t_startup_warmup_ms=obj.warmup_runtime_ms;
        end

        function ensure_mex_compiled(obj)
            root=fileparts(mfilename('fullpath'));
            cpp_path=fullfile(root,[obj.mex_name '.cpp']);
            mex_path=fullfile(root,[obj.mex_name '.' mexext]);
            lines=obj.mex_source_lines();
            src=sprintf('%s\n',lines{:});
            source_changed=true;
            if exist(cpp_path,'file')==2
                try
                    old=fileread(cpp_path);
                    source_changed=~strcmp(old,src);
                catch
                    source_changed=true;
                end
            end
            if source_changed
                fid=fopen(cpp_path,'w');
                assert(fid>=0,'[REPLANNER ERROR] MEX ソースファイルの書き込みに失敗しました。');
                fprintf(fid,'%s',src);
                fclose(fid);
            end
            rebuild=source_changed || exist(mex_path,'file')~=3;
            if rebuild
                try
                    eval(['clear ',obj.mex_name]);
                catch
                end
                old_dir=pwd;
                cleanup=onCleanup(@()cd(old_dir)); %#ok<NASGU>
                cd(root);
                mex('-R2018a',cpp_path,'-output',obj.mex_name);
                rehash;
            end
            obj.mex_ready = exist(mex_path,'file')==3 || exist(obj.mex_name,'file')==3;
            assert(obj.mex_ready,'[REPLANNER ERROR] 将来予測トリガー MEX カーネルが利用できません。');
        end

        function lines = mex_source_lines(obj)
            rev=obj.mex_source_revision;
            lines={
                ['// ' rev]
                '// Auto-generated by REPLANNING_SYSTEM'
                '#include "mex.h"'
                '#include <cmath>'
                '#include <algorithm>'
                '#include <limits>'
                ''
                'struct V3 { double x,y,z; };'
                ''
                'static inline V3 sub(V3 a,V3 b){ return {a.x-b.x,a.y-b.y,a.z-b.z}; }'
                'static inline V3 add(V3 a,V3 b){ return {a.x+b.x,a.y+b.y,a.z+b.z}; }'
                'static inline V3 mul(double s,V3 a){ return {s*a.x,s*a.y,s*a.z}; }'
                'static inline double dot(V3 a,V3 b){ return a.x*b.x+a.y*b.y+a.z*b.z; }'
                'static inline double nrm(V3 a){ return std::sqrt(dot(a,a)); }'
                ''
                'static inline V3 col3(const double* A,mwSize j){'
                '    return {A[3*j],A[3*j+1],A[3*j+2]};'
                '}'
                ''
                'static inline V3 Rt(const double* R,V3 v){'
                '    return {'
                '        R[0]*v.x+R[1]*v.y+R[2]*v.z,'
                '        R[3]*v.x+R[4]*v.y+R[5]*v.z,'
                '        R[6]*v.x+R[7]*v.y+R[8]*v.z};'
                '}'
                ''
                'static double pointLevel(V3 p,V3 c,const double* R,V3 a){'
                '    V3 z=Rt(R,sub(p,c));'
                '    z.x/=a.x; z.y/=a.y; z.z/=a.z;'
                '    return nrm(z);'
                '}'
                ''
                'static double segmentLevel(V3 p0,V3 p1,V3 c,const double* R,V3 a){'
                '    V3 z0=Rt(R,sub(p0,c));'
                '    V3 z1=Rt(R,sub(p1,c));'
                '    z0.x/=a.x; z0.y/=a.y; z0.z/=a.z;'
                '    z1.x/=a.x; z1.y/=a.y; z1.z/=a.z;'
                '    V3 d=sub(z1,z0);'
                '    double den=dot(d,d);'
                '    if(den<=std::numeric_limits<double>::epsilon()) return nrm(z0);'
                '    double u=-dot(z0,d)/den;'
                '    u=std::max(0.0,std::min(1.0,u));'
                '    return nrm(add(z0,mul(u,d)));'
                '}'
                ''
                'void mexFunction(int nlhs,mxArray* plhs[],int nrhs,const mxArray* prhs[]){'
                '    if(nrhs!=12) mexErrMsgIdAndTxt("G26TRIGGER:MEX","Expected 12 inputs.");'
                '    if(nlhs>4) mexErrMsgIdAndTxt("G26TRIGGER:MEX","At most 4 outputs.");'
                ''
                '    double H=mxGetScalar(prhs[0]);'
                '    double dt=mxGetScalar(prhs[1]);'
                '    double L=mxGetScalar(prhs[2]);'
                ''
                '    const mxArray* pLmx=prhs[3];'
                '    const mxArray* qmx=prhs[4];'
                '    const mxArray* Cmx=prhs[5];'
                '    const mxArray* Rmx=prhs[6];'
                '    const mxArray* Apmx=prhs[7];'
                '    const mxArray* Aqmx=prhs[8];'
                '    const mxArray* Acmx=prhs[9];'
                '    const mxArray* Idmx=prhs[10];'
                '    const mxArray* Actmx=prhs[11];'
                ''
                '    if(mxGetM(pLmx)!=3 || mxGetM(qmx)!=3)'
                '        mexErrMsgIdAndTxt("G26TRIGGER:MEX","pL,q must be 3xN.");'
                ''
                '    mwSize N=mxGetN(pLmx);'
                '    if(mxGetN(qmx)!=N)'
                '        mexErrMsgIdAndTxt("G26TRIGGER:MEX","pL/q length mismatch.");'
                ''
                '    mwSize M=mxGetN(Cmx);'
                ''
                '    if(mxGetM(Cmx)!=3 || mxGetM(Rmx)!=9 ||'
                '       mxGetM(Apmx)!=3 || mxGetM(Aqmx)!=3 || mxGetM(Acmx)!=3)'
                '        mexErrMsgIdAndTxt("G26TRIGGER:MEX","bad obstacle matrix shape.");'
                ''
                '    if(mxGetN(Rmx)!=M || mxGetN(Apmx)!=M || mxGetN(Aqmx)!=M ||'
                '       mxGetN(Acmx)!=M || mxGetNumberOfElements(Idmx)!=M ||'
                '       mxGetNumberOfElements(Actmx)!=M)'
                '        mexErrMsgIdAndTxt("G26TRIGGER:MEX","obstacle size mismatch.");'
                ''
                '    const double* pL=mxGetPr(pLmx);'
                '    const double* q=mxGetPr(qmx);'
                '    const double* C=mxGetPr(Cmx);'
                '    const double* Rf=mxGetPr(Rmx);'
                '    const double* Ap=mxGetPr(Apmx);'
                '    const double* Aq=mxGetPr(Aqmx);'
                '    const double* Ac=mxGetPr(Acmx);'
                '    const double* ids=mxGetPr(Idmx);'
                '    const double* active=mxGetPr(Actmx);'
                ''
                '    mwSize nH=(mwSize)std::floor(H/dt+1e-12);'
                '    mwSize i1=std::min(N-1,nH);'
                ''
                '    bool found=false;'
                '    double jhit=mxGetNaN();'
                '    double oid=mxGetNaN();'
                '    double comp=mxGetNaN();'
                ''
                '    for(mwSize j=0;j<=i1;j++){'
                '        V3 pLj=col3(pL,j);'
                '        V3 qj=col3(q,j);'
                '        V3 pQj=sub(pLj,mul(L,qj));'
                ''
                '        double best=std::numeric_limits<double>::infinity();'
                '        double bestComp=mxGetNaN();'
                '        double bestOid=mxGetNaN();'
                ''
                '        for(mwSize i=0;i<M;i++){'
                '            if(!(active[i]>0.5)) continue;'
                ''
                '            V3 c={C[3*i],C[3*i+1],C[3*i+2]};'
                '            const double* R=Rf+9*i;'
                '            V3 ap={Ap[3*i],Ap[3*i+1],Ap[3*i+2]};'
                '            V3 aq={Aq[3*i],Aq[3*i+1],Aq[3*i+2]};'
                '            V3 ac={Ac[3*i],Ac[3*i+1],Ac[3*i+2]};'
                ''
                '            double sp=pointLevel(pLj,c,R,ap)-1.0;'
                '            if(sp<best){ best=sp; bestComp=1; bestOid=ids[i]; }'
                ''
                '            double sq=pointLevel(pQj,c,R,aq)-1.0;'
                '            if(sq<best){ best=sq; bestComp=2; bestOid=ids[i]; }'
                ''
                '            double sc=segmentLevel(pLj,pQj,c,R,ac)-1.0;'
                '            if(sc<best){ best=sc; bestComp=3; bestOid=ids[i]; }'
                '        }'
                ''
                '        if(best<=0.0){'
                '            found=true;'
                '            jhit=(double)(j+1);'
                '            oid=bestOid;'
                '            comp=bestComp;'
                '            break;'
                '        }'
                '    }'
                ''
                '    if(nlhs>0) plhs[0]=mxCreateLogicalScalar(found);'
                '    if(nlhs>1) plhs[1]=mxCreateDoubleScalar(jhit);'
                '    if(nlhs>2) plhs[2]=mxCreateDoubleScalar(oid);'
                '    if(nlhs>3) plhs[3]=mxCreateDoubleScalar(comp);'
                '}'
            };
        end

    end
end
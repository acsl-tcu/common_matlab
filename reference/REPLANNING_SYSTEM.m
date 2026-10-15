classdef REPLANNING_SYSTEM < handle
% G27-V74R41R2: caller-compatible initialization order preserved.
% G27-V74R30D2R41Z-DIRECT: suspended-load HLC gain authority is
% Controller_HL_Suspended_Load(dt,agent); live F1..F4 are source-bound.
% Constructor MUST NOT require agent.reference.takeoff.
% Flight time origin is bound later during preflight warmup/table generation.
% G27-V74R41R1: constructor-safe Flight-reference time-origin binding.
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
        ENABLE_PROFILING = 0;  % 詳細多段 profiling は通常運用ではOFF
        ENABLE_LIGHT_RUNTIME_AUDIT = 1; % 外側1個の軽量 wall timer のみ有効
        SPIKE_CAPTURE_THRESHOLD_MS = 2.0; % rare-stage snapshot threshold [ms]

        % -----------------------------------------------------------------
        % 【離陸前コールドパス完全事前燃焼 (Pre-motion Prime) 設定】
        % -----------------------------------------------------------------
        ONLINE_PRIME_CALLS           = 3;   % 各合成テストケース (マスク・衝突パターン) の反復実行回数
        SETUP_EXACT_DO_PRIME_CALLS   = 12;  % 本番引数による do() の事前呼び出し回数 (JIT・キャッシュ完全温め)
        SEQUENTIAL_FLIGHT_PRIME_CALLS = 8;   % tf,tf+dt,... 本番同順序の protected-path 事前プライム
        REQUIRE_PRIME_BEFORE_TAKEOFF = 1;   % 離陸開始前に完全プライム完了を必須とする安全ガードフラグ
        takeoff_end_s                = NaN; % 離陸完了時刻 [s] (自動解決または手動設定)

        % G27-V74R41 Flight-reference time-origin contract:
        %   global simulation time tG remains unchanged;
        %   TIME_VARYING_REFERENCE is evaluated at tau=tG-tFlight0 in Flight.
        % This prevents Takeoff duration from advancing the Flight reference.
        flight_reference_time_origin = NaN; % absolute/global Flight start time [s]

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
        nominal_table_xd         = [];     % 同じ base_ref.do() で事前計算した完全 xd 行列 (D×K)
        nominal_acceleration_fn  = [];     % exact d^2/dt^2 of base_ref position law
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

        % Optional GENERAL continuous-time bound provider for a NOMINAL
        % reference whose speed/acceleration/jerk norms are not provably
        % constant from the symbolic source.
        %
        % Contract:
        %   B = fn(CTX)
        % where B must contain certified=true, continuous_time=true,
        % sampled_acceptance=false and hard upper bounds
        %   payload_speed_UB_m_s
        %   payload_accel_UB_m_s2
        %   payload_jerk_UB_m_s3
        % Optional:
        %   specific_force_norm_LB_m_s2
        %
        % This is a proof-interface, not a trajectory-specific threshold.
        NOMINAL_DERIVATIVE_BOUND_FN = [];

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
        runtime_reference_func_handle = []; % cached handle for allocation-free dependency fast path
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
        replan_request_active        = false;  % request-edge latch

        % -----------------------------------------------------------------
        % G27-V74R41 actual Flight trigger-event latch
        % -----------------------------------------------------------------
        % Captured ONLY on a real Flight request edge.  Setup/prime forced-hit
        % execution is excluded.  No file I/O / assignin occurs in Flight.
        % The full event is retrieved after the run by
        % get_last_trigger_event()/get_first_trigger_event().
        CAPTURE_REAL_TRIGGER_EVENT   = true;
        trigger_event_count          = 0;
        first_trigger_event          = struct();
        last_trigger_event           = struct();
        trigger_event_capture_ms     = NaN;

        % -----------------------------------------------------------------
        % 【G27-V74R16 four-backend repaired-candidate generator router】
        % -----------------------------------------------------------------
        % Callback contract:
        %   OUT = repaired_candidate_generator_fn(CTX)
        %
        % OUT may be:
        %   []                              -> no candidate
        %   candidate struct                -> candidate directly
        %   struct('candidate',C,...)        -> candidate + diagnostics
        %
        % R16 is SHADOW ONLY: candidate is passed through the exact
        % production validator but is NOT committed.
        repaired_candidate_generator_fn = []; % legacy/global fallback callback
        generator_shadow_enabled         = true;

        % -----------------------------------------------------------------
        % Four-backend router.
        %
        % Numeric production flag:
        %   1 = BEZIER_D15_C7
        %   2 = POLY_D15_C7
        %   3 = MINCO_STYLE_D15_C7
        %   4 = BSPLINE_D8_C7
        %
        % Selection is LATCHED ONCE before Flight:
        %   A) generator_backend_flag
        %   B) generator_backend_external_flag_fn(), evaluated ONCE in constructor
        %
        % Flight-time backend switching is forbidden.
        % -----------------------------------------------------------------
        generator_backend_flag             = 1;
        generator_backend_external_flag_fn = [];
        generator_backend_callbacks        = struct();
        generator_backend_locked_code      = 1;
        generator_backend_locked_name      = 'BEZIER_D15_C7';
        generator_backend_locked_source_code = 1; % 1=direct initial flag, 2=external initial provider
        generator_backend_selection_locked = false;
        generator_backend_locked_callback  = [];
        generator_backend_configuration_frozen = false;
        generator_backend_last_code        = 0;
        generator_backend_last_source_code = 0;
        generator_backend_last_callback_installed = false;

        % Generator integration helper is primed before Flight.
        generator_integration_prime_active = false;
        generator_integration_prime_done   = false;
        generator_integration_prime_ms     = NaN;
        generator_integration_prime_variants = 0;
        generator_context_detected_only    = true;

        % -----------------------------------------------------------------
        % G27-V74R21 fast analytic BEZIER_D15_C7 shadow proposal kernel
        % -----------------------------------------------------------------
        generator_backend_kernel_prime_done     = false;
        generator_backend_kernel_prime_ms       = NaN;
        generator_backend_kernel_prime_variants = 0;

        generator_shadow_proposal_count       = 0;
        bezier_shadow_generated_count         = 0;
        bezier_shadow_peak_safe_count         = 0;
        bezier_shadow_last_reason_code        = 0;
        bezier_shadow_last_t0                 = NaN;
        bezier_shadow_last_tm                 = NaN;
        bezier_shadow_last_t1                 = NaN;
        bezier_shadow_last_obstacle_id        = NaN;
        bezier_shadow_last_component_code     = NaN;
        bezier_shadow_last_displacement_m     = NaN;
        bezier_shadow_last_metric_before      = NaN;
        bezier_shadow_last_metric_after       = NaN;
        bezier_shadow_last_peak_min_metric    = NaN;
        bezier_shadow_last_peak_all_safe      = false;
        bezier_shadow_last_c7_exact           = false;
        bezier_shadow_last_d1_ub              = NaN;
        bezier_shadow_last_d2_ub              = NaN;
        bezier_shadow_last_d3_ub              = NaN;
        bezier_shadow_last_d4_ub              = NaN;
        bezier_shadow_last_d5_ub              = NaN;
        bezier_shadow_last_d6_ub              = NaN;
        bezier_shadow_last_dx                 = NaN;
        bezier_shadow_last_dy                 = NaN;
        bezier_shadow_last_dz                 = NaN;

        % G27-V74R22 continuous-time whole-body geometry shadow certificate.
        geometry_shadow_pass                   = false;
        geometry_shadow_reason_code            = 0;
        geometry_shadow_continuous_time        = false;
        geometry_shadow_ieee754_outward_closed = false;
        geometry_shadow_min_margin_m           = NaN;
        geometry_shadow_min_payload_margin_m   = NaN;
        geometry_shadow_min_uav_margin_m       = NaN;
        geometry_shadow_min_cable_margin_m     = NaN;
        geometry_shadow_leaf_count             = 0;
        geometry_shadow_split_count            = 0;
        geometry_shadow_max_depth              = 0;
        geometry_shadow_min_alpha_lb           = NaN;
        geometry_shadow_max_qdot_ub            = NaN;
        geometry_shadow_max_q_arc_radius       = NaN;
        geometry_shadow_max_q_arc_inflation_m  = NaN;
        geometry_shadow_resource_exhausted     = false;
        geometry_shadow_capture_done           = false;

        % Lean request-edge timing. Updated only on a real request edge.
        generator_last_elapsed_before_ms   = NaN;
        generator_last_budget_remaining_ms = NaN;
        generator_last_context_ms          = NaN;
        generator_last_callback_ms         = NaN;
        generator_last_validation_ms       = NaN;
        generator_last_critical_ms         = NaN;
        generator_last_cycle_critical_ms   = NaN;

        generator_request_count          = 0;
        generator_candidate_count        = 0;
        generator_validation_pass_count  = 0;
        generator_validation_fail_count  = 0;
        generator_deadline_fail_count    = 0;
        generator_last_runtime_ms        = NaN;
        generator_last_status_code       = 0;
        generator_last_status            = struct();

        % 互換性保持用プレースホルダー
        current_bspline = [];

        % -----------------------------------------------------------------
        % AF7R7 observer hard-support add-on (production, self-contained)
        % -----------------------------------------------------------------
        % Default ON.  Set false to bypass the add-on completely without
        % changing the existing estimator/reference/controller/plant path.
        ENABLE_OBSERVER_HARD_SUPPORT = true;
        OBSERVER_HARD_SUPPORT_MODE = 'SIM_NUMERIC_TRUTH';
        % Simulation-only hard authority for the TRUE plant payload mass.
        % Must be passed explicitly by the simulation setup because the
        % live agent.parameter.loadmass is later overwritten with the EKF
        % initial guess and is therefore not true-plant authority.
        OBSERVER_SIM_TRUE_ML = NaN;
        OBSERVER_REAL_NU_ABS_UB = [];
        OBSERVER_REAL_SOURCE_TAG = '';
        OBSERVER_REAL_INITIAL_SUPPORT = struct();

        % All add-on runtime state is kept in one internal struct.
        % No observer diagnostic fields are appended to result.state.
        observer_hard_support_internal = struct();

        % -----------------------------------------------------------------
        % 【最小プロファイリング情報構造体 (ENABLE_PROFILING=1 時のみ更新)】
        % -----------------------------------------------------------------
        profiling = struct( ...
            't_live_binding_ms',        NaN, ... % live HLC binding check [ms]
            't_dependency_ms',          NaN, ... % runtime dependency check [ms]
            't_cert_guard_ms',          NaN, ... % runtime certificate guard [ms]
            't_geometry_eval_ms',       NaN, ... % active-reference/current-geometry eval [ms]
            't_replan_status_ms',       NaN, ... % last_replan_status update [ms]
            't_output_write_ms',        NaN, ... % write_active_output [ms]
            't_state_trigger_write_ms', NaN, ... % scalar/vector trigger state write [ms]
            't_state_cert_write_ms',    NaN, ... % certificate/dependency state write [ms]
            't_state_meta_write_ms',    NaN, ... % repaired/meta state write [ms]
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

            % R41Z-DIRECT: bind the actual suspended-load gain generator before
            % the live HLC handle is verified.  This remains setup/refresh only.
            obj.runtime_source_signature= ...
                obj.capture_runtime_source_signature();
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

            obj.runtime_dependency_signature= ...
                obj.capture_runtime_dependency_signature_fast();

            obj.certificate_stale=obj.empty_certificate_stale_state();
            obj.certificate_stale.HLC_ROBUST_CERT=true;
            obj.dependency_refresh_count=obj.dependency_refresh_count+1;
        end


        function set_observer_hard_support_enabled(obj,enabled)
            % Runtime switch. Default ON. OFF is a pure add-on bypass.
            assert((islogical(enabled)||isnumeric(enabled)) && ...
                isscalar(enabled) && isfinite(double(enabled)), ...
                '[REPLANNER OBSERVER ERROR] enabled must be a finite logical scalar.');
            obj.ENABLE_OBSERVER_HARD_SUPPORT=logical(enabled);
            obj.observer_hard_support_internal=struct();
        end


        function enabled = get_observer_hard_support_enabled(obj)
            enabled=logical(obj.ENABLE_OBSERVER_HARD_SUPPORT);
        end


        function S = get_observer_hard_support_status(obj)
            % Read-only diagnostic/status accessor. Not used by safety logic.
            I=obj.observer_hard_support_internal;
            S=struct();
            S.schema='REPLANNING_SYSTEM_OBSERVER_HARD_SUPPORT_STATUS_V2';
            S.enabled=logical(obj.ENABLE_OBSERVER_HARD_SUPPORT);
            S.mode=char(obj.OBSERVER_HARD_SUPPORT_MODE);
            S.sim_true_mL_configured=double(obj.OBSERVER_SIM_TRUE_ML);
            S.initialized=false;
            S.selector_verified=false;
            S.last_event_time=NaN;
            S.last_phase='';
            S.phase_transition_count=0;
            S.update_count=0;
            S.fail_count=0;
            S.observer_support=struct();
            S.hlc_state_mass_bridge=struct();
            S.runtime_last_ms=NaN;
            S.runtime_max_ms=NaN;
            S.runtime_mean_ms=NaN;
            S.current_saturation_closed=false;
            S.fixed_phase_switch_time_required=false;
            S.realized_error_comparison_required=false;
            S.Q_R_covariance_are_not_hard_authority=true;
            S.external_AF7R7_runtime_dependency=false;

            if isempty(I) || ~isstruct(I)
                return;
            end
            names={'initialized','selector_verified','last_event_time','last_phase', ...
                'phase_transition_count','update_count','fail_count','observer_support', ...
                'hlc_state_mass_bridge','runtime_last_ms','runtime_max_ms'};
            for kk=1:numel(names)
                if isfield(I,names{kk})
                    S.(names{kk})=I.(names{kk});
                end
            end
            if isfield(I,'runtime_count') && I.runtime_count>0 && ...
                    isfield(I,'runtime_sum_ms')
                S.runtime_mean_ms=I.runtime_sum_ms/I.runtime_count;
            end
        end


        function E = get_last_trigger_event(obj)
            E=obj.last_trigger_event;
        end


        function E = get_first_trigger_event(obj)
            E=obj.first_trigger_event;
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
            if isfield(obj.command_limit_certificate,'live_gain_verified')
                status.live_gain_verified= ...
                    logical(obj.command_limit_certificate.live_gain_verified);
            else
                status.live_gain_verified=false;
            end
            status.actuator_dynamics=obj.actuator_dynamics_certificate;
            status.hlc_robust=obj.hlc_robust_certificate;
            status.dependency_refresh_count=obj.dependency_refresh_count;
            status.mass_certificate_rebuild_count= ...
                obj.mass_certificate_rebuild_count;
            status.trigger_event_count=double(obj.trigger_event_count);
            status.trigger_event_capture_ms=double(obj.trigger_event_capture_ms);
            status.flight_reference_time_origin=double(obj.flight_reference_time_origin);
            status.trigger_event_latched=obj.trigger_event_count>0;
            if obj.trigger_event_count>0
                status.trigger_event_time=double(obj.last_trigger_event.event_time);
            else
                status.trigger_event_time=NaN;
            end
        end



        function S = get_nominal_stop_geometry_actuator_T_set(obj)
            % =============================================================
            % Read-only certified T-set helper for the CURRENT NOMINAL
            % backup-stop theorem.
            %
            % Scope is deliberately limited to the intersection of:
            %   - STOP_GEOMETRY_4MS_CERT, and
            %   - current vertical-stop ACTUATOR_DYNAMICS_CERT.
            %
            % It is NOT the general obstacle-replan T optimizer and does not
            % include general HLC/rejoin/new-obstacle feasibility.
            % =============================================================
            assert(strcmpi(char(obj.active_reference_mode),'NOMINAL'), ...
                '[REPLANNER ERROR] nominal stop T-set requested outside NOMINAL mode.');

            G=obj.stop_geometry_4ms_certificate;
            A=obj.actuator_dynamics_certificate;

            assert(isstruct(G) && isfield(G,'certified') && logical(G.certified), ...
                '[REPLANNER ERROR] nominal stop-geometry certificate is unavailable.');
            assert(isstruct(A) && isfield(A,'certified') && logical(A.certified), ...
                '[REPLANNER ERROR] nominal stop actuator/dynamics certificate is unavailable.');
            assert(isfield(A,'T_actuator_theorem_lower') && ...
                isfinite(A.T_actuator_theorem_lower), ...
                '[REPLANNER ERROR] actuator T lower bound is unavailable.');

            I=obj.intersect_T_intervals_with_lower( ...
                G.INTERVALS,double(A.T_actuator_theorem_lower));

            intervals=zeros(0,2);
            loClosed=false(0,1);
            hiClosed=false(0,1);

            if ~isempty(I.branch1)
                intervals=[intervals;I.branch1]; %#ok<AGROW>
                loClosed=[loClosed;I.branch1_left_closed(:)]; %#ok<AGROW>
                hiClosed=[hiClosed;I.branch1_right_closed(:)]; %#ok<AGROW>
            end
            if ~isempty(I.branch2)
                intervals=[intervals;I.branch2]; %#ok<AGROW>
                loClosed=[loClosed;I.branch2_left_closed(:)]; %#ok<AGROW>
                hiClosed=[hiClosed;I.branch2_right_closed(:)]; %#ok<AGROW>
            end

            if ~isempty(intervals)
                [~,ord]=sort(intervals(:,1),'ascend');
                intervals=intervals(ord,:);
                loClosed=loClosed(ord);
                hiClosed=hiClosed(ord);
            end

            S=struct();
            S.certified_subset=true;
            S.scope='NOMINAL_STOP_GEOMETRY_INTERSECT_ACTUATOR_ONLY';
            S.intervals=intervals;
            S.lo_closed=loClosed;
            S.hi_closed=hiClosed;
            S.nonempty=~isempty(intervals);
            S.selected_minimum_exists=false;
            S.T_selected=NaN;
            S.reason='';

            if ~isempty(intervals)
                if loClosed(1)
                    S.selected_minimum_exists=true;
                    S.T_selected=intervals(1,1);
                    S.reason='ATTAINED_MINIMUM_OF_CERTIFIED_NOMINAL_STOP_SUBSET';
                else
                    S.reason='INFIMUM_NOT_ATTAINED_FAIL_CLOSED';
                end
            else
                S.reason='EMPTY_CERTIFIED_NOMINAL_STOP_SUBSET';
            end
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
                obj.replan_request_active=false;

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
                obj.replan_request_active=false;
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
            if isfield(opts,'ENABLE_LIGHT_RUNTIME_AUDIT')
                obj.ENABLE_LIGHT_RUNTIME_AUDIT=logical(opts.ENABLE_LIGHT_RUNTIME_AUDIT);
            end
            if isfield(opts,'SPIKE_CAPTURE_THRESHOLD_MS')
                obj.SPIKE_CAPTURE_THRESHOLD_MS=double(opts.SPIKE_CAPTURE_THRESHOLD_MS);
            end

            if isfield(opts,'ENABLE_OBSERVER_HARD_SUPPORT')
                obj.ENABLE_OBSERVER_HARD_SUPPORT=logical(opts.ENABLE_OBSERVER_HARD_SUPPORT);
            end
            if isfield(opts,'OBSERVER_HARD_SUPPORT_MODE')
                obj.OBSERVER_HARD_SUPPORT_MODE=char(string(opts.OBSERVER_HARD_SUPPORT_MODE));
            end
            if isfield(opts,'OBSERVER_SIM_TRUE_ML')
                obj.OBSERVER_SIM_TRUE_ML=double(opts.OBSERVER_SIM_TRUE_ML);
            end
            if isfield(opts,'OBSERVER_REAL_NU_ABS_UB')
                obj.OBSERVER_REAL_NU_ABS_UB=double(opts.OBSERVER_REAL_NU_ABS_UB(:));
            end
            if isfield(opts,'OBSERVER_REAL_SOURCE_TAG')
                obj.OBSERVER_REAL_SOURCE_TAG=char(string(opts.OBSERVER_REAL_SOURCE_TAG));
            end
            if isfield(opts,'OBSERVER_REAL_INITIAL_SUPPORT')
                obj.OBSERVER_REAL_INITIAL_SUPPORT=opts.OBSERVER_REAL_INITIAL_SUPPORT;
            end

            % IMPORTANT: do not initialize the add-on in the constructor.
            % Existing MOTIVE / estimator / reference initialization order is
            % preserved exactly. Lazy initialization occurs only when this
            % reference object receives the first real a/t/f control event.
            obj.observer_hard_support_internal=struct();

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
            if isfield(opts,'SEQUENTIAL_FLIGHT_PRIME_CALLS')
                obj.SEQUENTIAL_FLIGHT_PRIME_CALLS=opts.SEQUENTIAL_FLIGHT_PRIME_CALLS;
            end
            if isfield(opts,'REQUIRE_PRIME_BEFORE_TAKEOFF')
                obj.REQUIRE_PRIME_BEFORE_TAKEOFF=logical(opts.REQUIRE_PRIME_BEFORE_TAKEOFF);
            end
            if isfield(opts,'takeoff_end_s'), obj.takeoff_end_s=opts.takeoff_end_s; end
            if isfield(opts,'reference_packet_fn'), obj.reference_packet_fn=opts.reference_packet_fn; end
            if isfield(opts,'NOMINAL_DERIVATIVE_BOUND_FN')
                obj.NOMINAL_DERIVATIVE_BOUND_FN=opts.NOMINAL_DERIVATIVE_BOUND_FN;
            end
            if isfield(opts,'hard_modify_deadline_s')
                obj.hard_modify_deadline_s=opts.hard_modify_deadline_s;
            end
            if isfield(opts,'repaired_candidate_generator_fn')
                obj.repaired_candidate_generator_fn=opts.repaired_candidate_generator_fn;
            end
            if isfield(opts,'generator_shadow_enabled')
                obj.generator_shadow_enabled=logical(opts.generator_shadow_enabled);
            end
            if isfield(opts,'generator_backend_flag')
                obj.generator_backend_flag=opts.generator_backend_flag;
            end
            if isfield(opts,'generator_backend_external_flag_fn')
                obj.generator_backend_external_flag_fn=opts.generator_backend_external_flag_fn;
            end
            if isfield(opts,'generator_backend_callbacks')
                obj.generator_backend_callbacks=opts.generator_backend_callbacks;
            end

            if ~isempty(obj.repaired_candidate_generator_fn)
                assert(isa(obj.repaired_candidate_generator_fn,'function_handle'), ...
                    '[REPLANNER ERROR] repaired_candidate_generator_fn must be a function handle.');
            end
            if ~isempty(obj.generator_backend_external_flag_fn)
                assert(isa(obj.generator_backend_external_flag_fn,'function_handle'), ...
                    '[REPLANNER ERROR] generator_backend_external_flag_fn must be a function handle.');
            end

            % -------------------------------------------------------------
            % Backend selection is resolved exactly ONCE at construction.
            % An external provider is therefore only an INITIAL flag source;
            % it is never called during Flight or on a replan edge.
            % -------------------------------------------------------------
            backend_source_code=1;
            backend_raw=obj.generator_backend_flag;
            if ~isempty(obj.generator_backend_external_flag_fn)
                backend_raw=obj.generator_backend_external_flag_fn();
                backend_source_code=2;
            end
            [code0,name0,~]=obj.normalize_generator_backend_flag(backend_raw);
            obj.generator_backend_flag=code0;
            obj.generator_backend_locked_code=code0;
            obj.generator_backend_locked_name=name0;
            obj.generator_backend_locked_source_code=backend_source_code;
            obj.generator_backend_selection_locked=true;

            % Do not retain a request-time selector provider.
            obj.generator_backend_external_flag_fn=[];

            obj.generator_backend_callbacks= ...
                obj.normalize_generator_backend_callbacks(obj.generator_backend_callbacks);

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
            assert(isnumeric(obj.SEQUENTIAL_FLIGHT_PRIME_CALLS) && ...
                isscalar(obj.SEQUENTIAL_FLIGHT_PRIME_CALLS) && ...
                isfinite(obj.SEQUENTIAL_FLIGHT_PRIME_CALLS) && ...
                obj.SEQUENTIAL_FLIGHT_PRIME_CALLS>=5 && ...
                obj.SEQUENTIAL_FLIGHT_PRIME_CALLS==round(obj.SEQUENTIAL_FLIGHT_PRIME_CALLS), ...
                '[REPLANNER ERROR] SEQUENTIAL_FLIGHT_PRIME_CALLS must be an integer >=5.');

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
                "trigger_event_latched", ...
                "trigger_event_count", ...
                "trigger_event_time_latched", ...
                "trigger_event_ttc_latched", ...
                "trigger_event_collision_time_latched", ...
                "trigger_event_obstacle_id_latched", ...
                "trigger_event_component_code_latched", ...
                "trigger_event_capture_ms", ...
                "active_reference_mode", ...
                "active_reference_version", ...
                "nominal_xd_table_used", ...
                "certificate_guard_passed", ...
                "certificate_stale_mask", ...
                "hlc_optional_stale", ...
                "command_limit_live_verified", ...
                "dependency_refresh_count", ...
                "mass_certificate_rebuild_count", ...
                "replan_commit_count", ...
                "last_replan_commit_time", ...
                "generator_shadow_enabled", ...
                "generator_callback_installed", ...
                "generator_request_count", ...
                "generator_candidate_count", ...
                "generator_validation_pass_count", ...
                "generator_validation_fail_count", ...
                "generator_deadline_fail_count", ...
                "generator_last_runtime_ms", ...
                "generator_last_status_code", ...
                "generator_backend_flag", ...
                "generator_backend_last_code", ...
                "generator_backend_last_source_code", ...
                "generator_backend_last_callback_installed", ...
                "generator_backend_locked_code", ...
                "generator_backend_selection_locked", ...
                "generator_backend_configuration_frozen", ...
                "generator_integration_prime_done", ...
                "generator_integration_prime_ms", ...
                "generator_integration_prime_variants", ...
                "generator_context_detected_only", ...
                "generator_backend_kernel_prime_done", ...
                "generator_backend_kernel_prime_ms", ...
                "generator_backend_kernel_prime_variants", ...
                "generator_shadow_proposal_count", ...
                "bezier_shadow_generated_count", ...
                "bezier_shadow_peak_safe_count", ...
                "bezier_shadow_last_reason_code", ...
                "bezier_shadow_last_t0", ...
                "bezier_shadow_last_tm", ...
                "bezier_shadow_last_t1", ...
                "bezier_shadow_last_obstacle_id", ...
                "bezier_shadow_last_component_code", ...
                "bezier_shadow_last_displacement_m", ...
                "bezier_shadow_last_metric_before", ...
                "bezier_shadow_last_metric_after", ...
                "bezier_shadow_last_peak_min_metric", ...
                "bezier_shadow_last_peak_all_safe", ...
                "bezier_shadow_last_c7_exact", ...
                "bezier_shadow_last_d1_ub", ...
                "bezier_shadow_last_d2_ub", ...
                "bezier_shadow_last_d3_ub", ...
                "bezier_shadow_last_d4_ub", ...
                "bezier_shadow_last_d5_ub", ...
                "bezier_shadow_last_d6_ub", ...
                "bezier_shadow_last_dx", ...
                "bezier_shadow_last_dy", ...
                "bezier_shadow_last_dz", ...
                "geometry_shadow_pass", ...
                "geometry_shadow_reason_code", ...
                "geometry_shadow_continuous_time", ...
                "geometry_shadow_ieee754_outward_closed", ...
                "geometry_shadow_min_margin_m", ...
                "geometry_shadow_min_payload_margin_m", ...
                "geometry_shadow_min_uav_margin_m", ...
                "geometry_shadow_min_cable_margin_m", ...
                "geometry_shadow_leaf_count", ...
                "geometry_shadow_split_count", ...
                "geometry_shadow_max_depth", ...
                "geometry_shadow_min_alpha_lb", ...
                "geometry_shadow_max_qdot_ub", ...
                "geometry_shadow_max_q_arc_radius", ...
                "geometry_shadow_max_q_arc_inflation_m", ...
                "geometry_shadow_resource_exhausted", ...
                "generator_last_elapsed_before_ms", ...
                "generator_last_budget_remaining_ms", ...
                "generator_last_context_ms", ...
                "generator_last_callback_ms", ...
                "generator_last_validation_ms", ...
                "generator_last_critical_ms", ...
                "generator_last_cycle_critical_ms" ...
            ];

            for p_name=sensor_props
                if ~isprop(obj.result.state,p_name)
                    addprop(obj.result.state,p_name);
                end
            end

            % -------------------------------------------------------------
            % LOGGER-safe scalar timing fields.
            % Keep these separate from sensor_props to avoid changing the
            % existing production sensor/state interface.
            % -------------------------------------------------------------
            timing_props=[ ...
                "rt_light_e2e_ms", ...
                "rt_observer_ms", ...
                "rt_spike_capture_time", ...
                "rt_spike_total_ms", ...
                "rt_spike_stage_front_ms", ...
                "rt_spike_stage_loadmass_ms", ...
                "rt_spike_stage_dependency_ms", ...
                "rt_spike_stage_base_cert_geom_ms", ...
                "rt_spike_stage_sensor_trigger_ms", ...
                "rt_spike_stage_output_state_ms", ...
                "rt_spike_base_ref_ms", ...
                "rt_spike_flight_guard_ms", ...
                "rt_spike_cert_guard_ms", ...
                "rt_spike_active_ref_ms", ...
                "rt_spike_geometry_ms", ...
                "rt_spike_state_handle_ms", ...
                "rt_spike_write_active_output_ms", ...
                "rt_spike_state_basic_ms", ...
                "rt_spike_state_trigger_scalar_ms", ...
                "rt_spike_state_trigger_heavy_ms", ...
                "rt_spike_state_cert_ms", ...
                "rt_spike_state_meta_ms", ...
                "rt_spike_active_count", ...
                "rt_spike_need_replan", ...
                "rt_live_binding_ms", ...
                "rt_dependency_ms", ...
                "rt_cert_guard_ms", ...
                "rt_geometry_eval_ms", ...
                "rt_replan_status_ms", ...
                "rt_output_write_ms", ...
                "rt_state_trigger_write_ms", ...
                "rt_state_cert_write_ms", ...
                "rt_state_meta_write_ms", ...
                "rt_base_ref_ms", ...
                "rt_sensor_ms", ...
                "rt_reference_packet_ms", ...
                "rt_kernel_ms", ...
                "rt_trigger_ms", ...
                "rt_total_ms" ...
            ];

            for p_name=timing_props
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

            % Single lightweight production wall timer for A/B diagnosis.
            % This remains active while the detailed multi-stage profiling
            % below is disabled.
            if obj.ENABLE_LIGHT_RUNTIME_AUDIT
                t_light_e2e=tic;
            end

            if isfield(time,'dt') && ~isempty(time.dt) && time.dt>0
                if abs(time.dt-obj.prediction_dt)>1e-12
                    obj.prediction_dt=time.dt;
                    ratio=obj.horizon_s/obj.prediction_dt;
                    assert(abs(ratio-round(ratio)) <= 1e-9*max(1,abs(ratio)), ...
                        '[REPLANNER ERROR] horizon_s/prediction_dt must be integer.');
                    obj.reference_cache_valid=false;
                    obj.nominal_table_valid=false;
                    obj.nominal_table_xd=[];
                    obj.online_prime_done=false;
                    obj.setup_exact_do_prime_done=false;
                end
            end

            if obj.ENABLE_LIGHT_RUNTIME_AUDIT
                cp_front_ms=1e3*toc(t_light_e2e);
            end

            obj.loadmass=obj.get_load_mass();

            if obj.ENABLE_LIGHT_RUNTIME_AUDIT
                cp_loadmass_ms=1e3*toc(t_light_e2e);
            end

            % -------------------------------------------------------------
            % AF7R7 observer hard-support add-on.
            % This REPLANNING_SYSTEM call occurs after sensor+EKF and before
            % controller+plant. Constructor/startup order is untouched.
            % Existing rt_light_e2e_ms automatically includes this cost.
            % -------------------------------------------------------------
            dt_observer_ms=NaN;
            if obj.ENABLE_OBSERVER_HARD_SUPPORT && ...
                    obj.observer_is_real_control_event(cha)
                t_observer_addon=tic;
                obj.observer_update_same_event(time,cha);
                dt_observer_ms=1e3*toc(t_observer_addon);
                I=obj.observer_hard_support_internal;
                I.runtime_last_ms=dt_observer_ms;
                if ~isfield(I,'runtime_max_ms') || ~isfinite(I.runtime_max_ms)
                    I.runtime_max_ms=dt_observer_ms;
                else
                    I.runtime_max_ms=max(I.runtime_max_ms,dt_observer_ms);
                end
                if ~isfield(I,'runtime_sum_ms') || ~isfinite(I.runtime_sum_ms)
                    I.runtime_sum_ms=0;
                end
                if ~isfield(I,'runtime_count') || ~isfinite(I.runtime_count)
                    I.runtime_count=0;
                end
                I.runtime_sum_ms=I.runtime_sum_ms+dt_observer_ms;
                I.runtime_count=I.runtime_count+1;
                obj.observer_hard_support_internal=I;
            end

            % The replanner is constructed before the live HLC controller
            % object exists in run_flight_headless.  Command limits are
            % therefore source-bound in the constructor and the actual live
            % controller handle is verified lazily once it exists.  Flight
            % may not proceed without that live verification.
            if obj.ENABLE_PROFILING
                t_live_binding=tic;
            end
            obj.verify_live_controller_binding(isequal(cha,'f'));
            if obj.ENABLE_PROFILING
                obj.profiling.t_live_binding_ms=1e3*toc(t_live_binding);
                t_dependency=tic;
            end

            % Fast dependency check: scalar/string comparisons only.
            % No source-file parsing and no symbolic derivation is performed
            % in the Flight control cycle.
            obj.update_runtime_dependency_state_fast(cha);
            if obj.ENABLE_LIGHT_RUNTIME_AUDIT
                cp_dependency_ms=1e3*toc(t_light_e2e);
            end
            if obj.ENABLE_PROFILING
                obj.profiling.t_dependency_ms=1e3*toc(t_dependency);
            end

            if obj.ENABLE_PROFILING
                t_total=tic;
                t0=tic;
            end

            protected_phase=isequal(cha,'f');

            if protected_phase
                % NOMINAL current xd has already been evaluated exactly by
                % base_ref.do() during preflight table construction.
                % Reading it here removes repeated Flight-time reference
                % evaluation without changing the reference mathematics.
                xd_nom=obj.get_nominal_xd_from_table(time.t);
            else
                base_res=obj.base_ref.do(varargin{:});
                xd_nom=base_res.state.xd;
            end

            if obj.ENABLE_LIGHT_RUNTIME_AUDIT
                cp_base_ref_ms=1e3*toc(t_light_e2e);
            end

            if obj.ENABLE_PROFILING
                obj.profiling.t_base_ref_ms=1e3*toc(t0);
            end

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
                    obj.setup_exact_do_prime_done && ...
                    obj.generator_backend_configuration_frozen && ...
                    obj.generator_integration_prime_done && ...
                    obj.generator_backend_kernel_prime_done, ...
                    ['[REPLANNER PREFLIGHT ERROR] Flight reached before pre-motion exact/generator prime completed. ' ...
                     'Prime must finish before vehicle motion.']);
                assert(~obj.setup_exact_force_hit, ...
                    '[REPLANNER ERROR] setup-only forced-hit flag leaked into Flight.');
            end

            if obj.ENABLE_LIGHT_RUNTIME_AUDIT
                cp_flight_guard_ms=1e3*toc(t_light_e2e);
            end

            % Hard real-time accounting for sensor/trigger/generator integration.
            % This timer is independent of optional profiling/light-audit flags.
            t_hard_cycle=tic;

            if obj.ENABLE_PROFILING
                t_cert_guard=tic;
            end
            obj.assert_runtime_core_certificates_current();
            if obj.ENABLE_LIGHT_RUNTIME_AUDIT
                cp_cert_guard_ms=1e3*toc(t_light_e2e);
            end

            if obj.ENABLE_PROFILING
                obj.profiling.t_cert_guard_ms=1e3*toc(t_cert_guard);
                t_geometry_eval=tic;
            end

            xd_active_now=obj.evaluate_active_reference_xd(time.t,xd_nom);

            if obj.ENABLE_LIGHT_RUNTIME_AUDIT
                cp_active_ref_ms=1e3*toc(t_light_e2e);
            end
            [pQ_cur,pL_cur,q_cur,cable_residual]=obj.get_current_physical_geometry(xd_active_now);

            if obj.ENABLE_LIGHT_RUNTIME_AUDIT
                cp_geometry_ms=1e3*toc(t_light_e2e);
                cp_base_cert_geom_ms=cp_geometry_ms;
            end

            if obj.ENABLE_PROFILING
                obj.profiling.t_geometry_eval_ms=1e3*toc(t_geometry_eval);
                ts=tic;
            end
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

            if obj.ENABLE_PROFILING
                t_replan_status=tic;
            end
            % Replan generator is invoked only on a REAL Flight request edge.
            % Setup/warmup forced-hit paths never invoke the external generator.
            if need_replan
                if ~obj.replan_request_active

                    % -----------------------------------------------------
                    % G27-V74R41 REAL trigger-event authority.
                    %
                    % This is the unique rising edge used by the online
                    % generator request.  Synthetic setup/prime forced-hit
                    % executions are forbidden from entering the latch.
                    % No file I/O / assignin is performed here.
                    % -----------------------------------------------------
                    if obj.CAPTURE_REAL_TRIGGER_EVENT && ...
                            ~obj.setup_exact_do_prime_active && ...
                            ~obj.generator_integration_prime_active
                        tevent=tic;
                        obj.latch_real_trigger_event( ...
                            time.t,xd_nom,xd_active_now, ...
                            pQ_cur,pL_cur,q_cur,cable_residual, ...
                            active,all_dists,ref_source, ...
                            ttc,collision_time,obs_id,comp_name,comp_code, ...
                            pL_collision,q_collision,pQ_collision,jhit, ...
                            effective_horizon,N);
                        obj.trigger_event_capture_ms=1e3*toc(tevent);
                        obj.last_trigger_event.capture_runtime_ms= ...
                            double(obj.trigger_event_capture_ms);
                        if obj.trigger_event_count==1
                            obj.first_trigger_event.capture_runtime_ms= ...
                                double(obj.trigger_event_capture_ms);
                        end
                    end

                    if ~obj.setup_exact_do_prime_active && obj.generator_shadow_enabled
                        obj.invoke_replan_generator_shadow( ...
                            t_hard_cycle,time.t,xd_nom,xd_active_now, ...
                            pQ_cur,pL_cur,q_cur,cable_residual, ...
                            active,all_dists,pL_ref,q_ref,ref_source, ...
                            ttc,collision_time,obs_id,comp_name,comp_code, ...
                            pL_collision,q_collision,pQ_collision,jhit);
                    else
                        obj.last_replan_status=struct( ...
                            'requested',true, ...
                            'committed',false, ...
                            'reason','GENERATOR_SKIPPED_DURING_PRIME_OR_DISABLED', ...
                            'request_time',double(time.t), ...
                            'trigger_ttc',double(ttc), ...
                            'trigger_obstacle_id',double(obs_id), ...
                            'trigger_component',char(comp_name));
                    end
                    obj.replan_request_active=true;
                end
            else
                obj.replan_request_active=false;
            end
            if obj.ENABLE_PROFILING
                obj.profiling.t_replan_status_ms=1e3*toc(t_replan_status);
                t_output_write=tic;
            end

            if obj.ENABLE_LIGHT_RUNTIME_AUDIT
                cp_sensor_trigger_ms=1e3*toc(t_light_e2e);
            end

            st=obj.result.state;

            if obj.ENABLE_LIGHT_RUNTIME_AUDIT
                cp_state_handle_ms=1e3*toc(t_light_e2e);
            end

            obj.write_active_output(time.t,xd_nom);

            if obj.ENABLE_LIGHT_RUNTIME_AUDIT
                cp_write_active_output_ms=1e3*toc(t_light_e2e);
            end

            if obj.ENABLE_PROFILING
                obj.profiling.t_output_write_ms=1e3*toc(t_output_write);
                t_state_trigger_write=tic;
            end

            st.time=time.t;
            st.pQ=pQ_cur;
            st.detected_point=vis_count>0;

            % Variable-size detected/relevant structs are compatibility
            % placeholders only. Fixed scalar/vector fields below are the
            % production LOGGER interface.
            st.min_obstacle_id_point=min_id;
            st.min_dist_point=min_dist;
            st.detected_obstacle_count_point=vis_count;

            if obj.ENABLE_LIGHT_RUNTIME_AUDIT
                cp_state_basic_ms=1e3*toc(t_light_e2e);
            end

            % Fixed scalar trigger fields first.
            st.need_replan=need_replan;
            st.trigger_ttc=ttc;
            st.trigger_collision_time=collision_time;
            st.trigger_obstacle_id=obs_id;
            st.trigger_component_code=comp_code;
            st.trigger_effective_horizon=effective_horizon;
            st.trigger_reference_points=N;
            st.trigger_cable_residual=cable_residual;
            st.trigger_hit_index=jhit;
            st.trigger_kernel_called=double(kernel_called);

            if obj.ENABLE_LIGHT_RUNTIME_AUDIT
                cp_state_trigger_scalar_ms=1e3*toc(t_light_e2e);
            end

            % Small vectors / strings / mask-distance arrays are separated
            % because these assignments can create/copy MATLAB data.
            st.trigger_component=comp_name;
            st.trigger_pL_collision=pL_collision;
            st.trigger_q_collision=q_collision;
            st.trigger_pQ_collision=pQ_collision;
            st.trigger_reference_source=ref_source;
            st.trigger_sensor_origin=pQ_cur;
            st.trigger_active_mask=double(active);
            st.trigger_raw_distances=all_dists;

            if obj.ENABLE_LIGHT_RUNTIME_AUDIT
                cp_state_trigger_heavy_ms=1e3*toc(t_light_e2e);
            end

            if obj.ENABLE_PROFILING
                obj.profiling.t_state_trigger_write_ms=1e3*toc(t_state_trigger_write);
                t_state_cert_write=tic;
            end

            st.active_reference_mode=obj.active_reference_mode;
            st.active_reference_version=obj.active_reference_version;
            st.nominal_xd_table_used=protected_phase;

            % Full certificate structs remain authoritative in obj.*.
            % The fail-closed certificate guard above is unchanged.
            % LOGGER receives fixed-size summaries only.
            st.certificate_guard_passed=true;
            st.certificate_stale_mask = ...
                double(obj.certificate_stale.SPEED_TRIGGER_CERT) + ...
                2*double(obj.certificate_stale.SENSOR_HORIZON_CERT) + ...
                4*double(obj.certificate_stale.STOP_GEOMETRY_4MS_CERT) + ...
                8*double(obj.certificate_stale.ACTUATOR_DYNAMICS_CERT);
            st.hlc_optional_stale=logical(obj.certificate_stale.HLC_ROBUST_CERT);
            if isfield(obj.command_limit_certificate,'live_controller_verified')
                st.command_limit_live_verified= ...
                    logical(obj.command_limit_certificate.live_controller_verified);
            else
                st.command_limit_live_verified=false;
            end
            st.dependency_refresh_count=obj.dependency_refresh_count;
            st.mass_certificate_rebuild_count=obj.mass_certificate_rebuild_count;

            if obj.ENABLE_LIGHT_RUNTIME_AUDIT
                cp_state_cert_ms=1e3*toc(t_light_e2e);
            end

            if obj.ENABLE_PROFILING
                obj.profiling.t_state_cert_write_ms=1e3*toc(t_state_cert_write);
                t_state_meta_write=tic;
            end

            st.loadmass=obj.loadmass;
            st.replan_commit_count=obj.replan_commit_count;
            st.last_replan_commit_time=obj.last_replan_commit_time;
            st.generator_shadow_enabled=logical(obj.generator_shadow_enabled);
            st.generator_callback_installed=~isempty(obj.repaired_candidate_generator_fn);
            st.generator_request_count=obj.generator_request_count;
            st.generator_candidate_count=obj.generator_candidate_count;
            st.generator_validation_pass_count=obj.generator_validation_pass_count;
            st.generator_validation_fail_count=obj.generator_validation_fail_count;
            st.generator_deadline_fail_count=obj.generator_deadline_fail_count;
            st.generator_last_runtime_ms=obj.generator_last_runtime_ms;
            st.generator_last_status_code=obj.generator_last_status_code;
            [flagCode,~,~]=obj.normalize_generator_backend_flag(obj.generator_backend_flag);
            st.generator_backend_flag=double(flagCode);
            st.generator_backend_last_code=double(obj.generator_backend_last_code);
            st.generator_backend_last_source_code=double(obj.generator_backend_last_source_code);
            st.generator_backend_last_callback_installed= ...
                logical(obj.generator_backend_last_callback_installed);
            st.generator_backend_locked_code=double(obj.generator_backend_locked_code);
            st.generator_backend_selection_locked=logical(obj.generator_backend_selection_locked);
            st.generator_backend_configuration_frozen= ...
                logical(obj.generator_backend_configuration_frozen);
            st.generator_integration_prime_done=logical(obj.generator_integration_prime_done);
            st.generator_integration_prime_ms=obj.generator_integration_prime_ms;
            st.generator_integration_prime_variants=obj.generator_integration_prime_variants;
            st.generator_context_detected_only=logical(obj.generator_context_detected_only);
            st.generator_backend_kernel_prime_done=logical(obj.generator_backend_kernel_prime_done);
            st.generator_backend_kernel_prime_ms=obj.generator_backend_kernel_prime_ms;
            st.generator_backend_kernel_prime_variants=obj.generator_backend_kernel_prime_variants;
            st.generator_shadow_proposal_count=obj.generator_shadow_proposal_count;
            st.bezier_shadow_generated_count=obj.bezier_shadow_generated_count;
            st.bezier_shadow_peak_safe_count=obj.bezier_shadow_peak_safe_count;
            st.bezier_shadow_last_reason_code=obj.bezier_shadow_last_reason_code;
            st.bezier_shadow_last_t0=obj.bezier_shadow_last_t0;
            st.bezier_shadow_last_tm=obj.bezier_shadow_last_tm;
            st.bezier_shadow_last_t1=obj.bezier_shadow_last_t1;
            st.bezier_shadow_last_obstacle_id=obj.bezier_shadow_last_obstacle_id;
            st.bezier_shadow_last_component_code=obj.bezier_shadow_last_component_code;
            st.bezier_shadow_last_displacement_m=obj.bezier_shadow_last_displacement_m;
            st.bezier_shadow_last_metric_before=obj.bezier_shadow_last_metric_before;
            st.bezier_shadow_last_metric_after=obj.bezier_shadow_last_metric_after;
            st.bezier_shadow_last_peak_min_metric=obj.bezier_shadow_last_peak_min_metric;
            st.bezier_shadow_last_peak_all_safe=logical(obj.bezier_shadow_last_peak_all_safe);
            st.bezier_shadow_last_c7_exact=logical(obj.bezier_shadow_last_c7_exact);
            st.bezier_shadow_last_d1_ub=obj.bezier_shadow_last_d1_ub;
            st.bezier_shadow_last_d2_ub=obj.bezier_shadow_last_d2_ub;
            st.bezier_shadow_last_d3_ub=obj.bezier_shadow_last_d3_ub;
            st.bezier_shadow_last_d4_ub=obj.bezier_shadow_last_d4_ub;
            st.bezier_shadow_last_d5_ub=obj.bezier_shadow_last_d5_ub;
            st.bezier_shadow_last_d6_ub=obj.bezier_shadow_last_d6_ub;
            st.bezier_shadow_last_dx=obj.bezier_shadow_last_dx;
            st.bezier_shadow_last_dy=obj.bezier_shadow_last_dy;
            st.bezier_shadow_last_dz=obj.bezier_shadow_last_dz;
            st.geometry_shadow_pass=logical(obj.geometry_shadow_pass);
            st.geometry_shadow_reason_code=obj.geometry_shadow_reason_code;
            st.geometry_shadow_continuous_time=logical(obj.geometry_shadow_continuous_time);
            st.geometry_shadow_ieee754_outward_closed=logical(obj.geometry_shadow_ieee754_outward_closed);
            st.geometry_shadow_min_margin_m=obj.geometry_shadow_min_margin_m;
            st.geometry_shadow_min_payload_margin_m=obj.geometry_shadow_min_payload_margin_m;
            st.geometry_shadow_min_uav_margin_m=obj.geometry_shadow_min_uav_margin_m;
            st.geometry_shadow_min_cable_margin_m=obj.geometry_shadow_min_cable_margin_m;
            st.geometry_shadow_leaf_count=obj.geometry_shadow_leaf_count;
            st.geometry_shadow_split_count=obj.geometry_shadow_split_count;
            st.geometry_shadow_max_depth=obj.geometry_shadow_max_depth;
            st.geometry_shadow_min_alpha_lb=obj.geometry_shadow_min_alpha_lb;
            st.geometry_shadow_max_qdot_ub=obj.geometry_shadow_max_qdot_ub;
            st.geometry_shadow_max_q_arc_radius=obj.geometry_shadow_max_q_arc_radius;
            st.geometry_shadow_max_q_arc_inflation_m=obj.geometry_shadow_max_q_arc_inflation_m;
            st.geometry_shadow_resource_exhausted=logical(obj.geometry_shadow_resource_exhausted);
            st.generator_last_elapsed_before_ms=obj.generator_last_elapsed_before_ms;
            st.generator_last_budget_remaining_ms=obj.generator_last_budget_remaining_ms;
            st.generator_last_context_ms=obj.generator_last_context_ms;
            st.generator_last_callback_ms=obj.generator_last_callback_ms;
            st.generator_last_validation_ms=obj.generator_last_validation_ms;
            st.generator_last_critical_ms=obj.generator_last_critical_ms;
            st.generator_last_cycle_critical_ms=obj.generator_last_cycle_critical_ms;

            if obj.ENABLE_LIGHT_RUNTIME_AUDIT
                cp_state_meta_ms=1e3*toc(t_light_e2e);
            end

            if obj.ENABLE_PROFILING
                obj.profiling.t_state_meta_write_ms=1e3*toc(t_state_meta_write);
                obj.profiling.t_total_ms=1e3*toc(t_total);

                % LOGGER-safe scalar timing mirror.
                % Diagnostic only: these values do not feed any safety or
                % replanning decision.
                st.rt_live_binding_ms=obj.profiling.t_live_binding_ms;
                st.rt_dependency_ms=obj.profiling.t_dependency_ms;
                st.rt_cert_guard_ms=obj.profiling.t_cert_guard_ms;
                st.rt_geometry_eval_ms=obj.profiling.t_geometry_eval_ms;
                st.rt_replan_status_ms=obj.profiling.t_replan_status_ms;
                st.rt_output_write_ms=obj.profiling.t_output_write_ms;
                st.rt_state_trigger_write_ms=obj.profiling.t_state_trigger_write_ms;
                st.rt_state_cert_write_ms=obj.profiling.t_state_cert_write_ms;
                st.rt_state_meta_write_ms=obj.profiling.t_state_meta_write_ms;
                st.rt_base_ref_ms=obj.profiling.t_base_ref_ms;
                st.rt_sensor_ms=obj.profiling.t_sensor_ms;
                st.rt_reference_packet_ms=obj.profiling.t_reference_packet_ms;
                st.rt_kernel_ms=obj.profiling.t_kernel_ms;
                st.rt_trigger_ms=obj.profiling.t_trigger_ms;
                st.rt_total_ms=obj.profiling.t_total_ms;

                % Nested profiling is intentionally not attached to result.state.
                % Direct rt_* scalars are the runtime diagnostic interface.
            end

            if obj.ENABLE_LIGHT_RUNTIME_AUDIT
                cp_output_state_ms=1e3*toc(t_light_e2e);
                st.rt_light_e2e_ms=cp_output_state_ms;
                st.rt_observer_ms=dt_observer_ms;

                % Capture only rare slow cycles. The production total above is
                % measured before these diagnostic writes.
                if cp_output_state_ms > obj.SPIKE_CAPTURE_THRESHOLD_MS
                    st.rt_spike_capture_time=double(time.t);
                    st.rt_spike_total_ms=cp_output_state_ms;
                    st.rt_spike_stage_front_ms=cp_front_ms;
                    st.rt_spike_stage_loadmass_ms=cp_loadmass_ms-cp_front_ms;
                    st.rt_spike_stage_dependency_ms=cp_dependency_ms-cp_loadmass_ms;
                    st.rt_spike_stage_base_cert_geom_ms= ...
                        cp_base_cert_geom_ms-cp_dependency_ms;
                    st.rt_spike_stage_sensor_trigger_ms= ...
                        cp_sensor_trigger_ms-cp_base_cert_geom_ms;
                    st.rt_spike_stage_output_state_ms= ...
                        cp_output_state_ms-cp_sensor_trigger_ms;

                    % Fine split of BASE_CERT_GEOM.
                    st.rt_spike_base_ref_ms= ...
                        cp_base_ref_ms-cp_dependency_ms;
                    st.rt_spike_flight_guard_ms= ...
                        cp_flight_guard_ms-cp_base_ref_ms;
                    st.rt_spike_cert_guard_ms= ...
                        cp_cert_guard_ms-cp_flight_guard_ms;
                    st.rt_spike_active_ref_ms= ...
                        cp_active_ref_ms-cp_cert_guard_ms;
                    st.rt_spike_geometry_ms= ...
                        cp_geometry_ms-cp_active_ref_ms;

                    % Fine split of OUTPUT_STATE.
                    st.rt_spike_state_handle_ms= ...
                        cp_state_handle_ms-cp_sensor_trigger_ms;
                    st.rt_spike_write_active_output_ms= ...
                        cp_write_active_output_ms-cp_state_handle_ms;
                    st.rt_spike_state_basic_ms= ...
                        cp_state_basic_ms-cp_write_active_output_ms;
                    st.rt_spike_state_trigger_scalar_ms= ...
                        cp_state_trigger_scalar_ms-cp_state_basic_ms;
                    st.rt_spike_state_trigger_heavy_ms= ...
                        cp_state_trigger_heavy_ms-cp_state_trigger_scalar_ms;
                    st.rt_spike_state_cert_ms= ...
                        cp_state_cert_ms-cp_state_trigger_heavy_ms;
                    st.rt_spike_state_meta_ms= ...
                        cp_state_meta_ms-cp_state_cert_ms;

                    st.rt_spike_active_count=double(vis_count);
                    st.rt_spike_need_replan=double(need_replan);
                end
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
            % Fast fail-closed live mass read.
            % The estimator schema is fixed after construction; direct access
            % avoids repeated isprop/isfield traversal in the control cycle.
            try
                mL=obj.self.estimator.result.state.mL;
            catch
                error(['[REPLANNER ERROR] estimator live mass path ' ...
                    'self.estimator.result.state.mL is unavailable.']);
            end

            assert(isnumeric(mL) && isscalar(mL) && isfinite(mL) && mL>0, ...
                ['[REPLANNER ERROR] live load-mass estimate must be a finite ' ...
                 'positive scalar.']);
            mL=double(mL);
        end


        function cert = build_nominal_active_speed_certificate(obj)
            % =============================================================
            % GENERAL continuous-time NOMINAL kinematic bound certificate.
            %
            % The theorem does NOT require constant speed.
            %
            % From the exact position law pL(t):
            %   vL = pLdot, aL = pLddot, jL = pLdddot.
            %
            % Let hard continuous-time bounds be
            %   ||vL|| <= V_L,
            %   ||aL|| <= A_L,
            %   ||jL|| <= J_L.
            %
            % Define s = aL + g e3 and pT* = -s/||s||.  If
            %   sigma = inf_t ||s(t)|| > 0,
            % then
            %   ||pTdot|| <= J_L / sigma
            % because
            %   pTdot = -(I-pT pT') jL / ||s||.
            %
            % Hence for pQ = pL - L pT,
            %   ||vQ|| <= V_L + L J_L/sigma.
            %
            % Automatic setup prover:
            %   If the relevant squared norm is symbolically constant, use
            %   its exact value.  Otherwise use the GENERAL certified bound
            %   provider NOMINAL_DERIVATIVE_BOUND_FN.  Therefore
            %   time-invariant speed/acceleration/jerk is NOT a production
            %   requirement.
            %
            % No sampled maxima are accepted.
            % =============================================================

            assert(strcmpi(char(obj.active_reference_mode),'NOMINAL'), ...
                ['[REPLANNER ERROR] nominal speed certificate requested ' ...
                 'while active_reference_mode is not NOMINAL.']);

            assert(isa(obj.base_ref,'TIME_VARYING_REFERENCE'), ...
                ['[REPLANNER ERROR] exact nominal kinematic certificate ' ...
                 'requires TIME_VARYING_REFERENCE base_ref.']);

            assert(isprop(obj.base_ref,'func') && ...
                isa(obj.base_ref.func,'function_handle'), ...
                '[REPLANNER ERROR] nominal base_ref.func is unavailable.');

            fstr=func2str(obj.base_ref.func);
            [xd_sym,t]=obj.function_handle_to_symbolic_vector(fstr);

            assert(numel(xd_sym)>=7, ...
                '[REPLANNER ERROR] nominal xd must contain at least 7 states.');

            pL=simplify(xd_sym(1:3));
            vL_packet=simplify(xd_sym(5:7));

            vL=simplify(diff(pL,t));
            assert(obj.symbolic_vector_equal(vL,vL_packet), ...
                '[REPLANNER ERROR] nominal xd(5:7) is not d/dt xd(1:3).');

            aL=simplify(diff(vL,t));
            jL=simplify(diff(aL,t));

            % Setup-only exact acceleration evaluator used by future
            % reference packet construction.  This removes legacy reliance
            % on xd(9:11), which is not guaranteed to be the true second
            % derivative for every TIME_VARYING_REFERENCE generator.
            obj.nominal_acceleration_fn=matlabFunction(aL,'Vars',t);

            [VL,vConst]=obj.exact_constant_vector_norm_ub(vL,t);
            [AL,aConst]=obj.exact_constant_vector_norm_ub(aL,t);
            [JL,jConst]=obj.exact_constant_vector_norm_ub(jL,t);

            providerUsed=false;
            provider=struct();

            needProvider=~isfinite(VL) || ~isfinite(AL) || ~isfinite(JL);
            if needProvider
                fn=obj.NOMINAL_DERIVATIVE_BOUND_FN;
                assert(isa(fn,'function_handle'), ...
                    ['[REPLANNER ERROR] current NOMINAL reference has a ' ...
                     'nonconstant derivative norm. Install the GENERAL ' ...
                     'continuous-time NOMINAL_DERIVATIVE_BOUND_FN; sampled ' ...
                     'maxima are forbidden.']);

                CTX=struct();
                CTX.schema='REPLANNING_NOMINAL_DERIVATIVE_BOUND_REQUEST_V1';
                CTX.reference_class=class(obj.base_ref);
                CTX.func_text=fstr;
                CTX.symbolic_time=t;
                CTX.symbolic_position=pL;
                CTX.symbolic_velocity=vL;
                CTX.symbolic_acceleration=aL;
                CTX.symbolic_jerk=jL;
                CTX.gravity=double(obj.gravity);
                CTX.cable_length=double(obj.L_cable);
                CTX.required_continuous_time=true;
                CTX.sampled_acceptance_forbidden=true;

                provider=fn(CTX);
                obj.validate_nominal_derivative_bound_provider(provider);

                if ~isfinite(VL)
                    VL=double(provider.payload_speed_UB_m_s);
                end
                if ~isfinite(AL)
                    AL=double(provider.payload_accel_UB_m_s2);
                end
                if ~isfinite(JL)
                    JL=double(provider.payload_jerk_UB_m_s3);
                end
                providerUsed=true;
            end

            assert(all(isfinite([VL AL JL])) && all([VL AL JL]>=0), ...
                '[REPLANNER ERROR] nominal continuous derivative bounds invalid.');

            % -------------------------------------------------------------
            % General cable-direction lower bound.
            % -------------------------------------------------------------
            % Default theorem:
            %   ||aL + g e3|| >= g - ||aL|| >= g - A_L.
            % A certified provider may supply a tighter direct lower bound.
            g=double(obj.gravity);
            sigmaLB=g-AL;

            if providerUsed && isfield(provider,'specific_force_norm_LB_m_s2')
                q=double(provider.specific_force_norm_LB_m_s2);
                if isscalar(q) && isfinite(q) && q>0
                    sigmaLB=max(sigmaLB,q);
                end
            end

            assert(isfinite(sigmaLB) && sigmaLB>0, ...
                ['[REPLANNER ERROR] cannot certify ||aL+g e3|| away from zero. ' ...
                 'Provide a certified positive specific_force_norm_LB_m_s2 ' ...
                 'or a tighter continuous acceleration bound.']);

            qdotUB=JL/sigmaLB;
            VQ=VL+double(obj.L_cable)*qdotUB;

            assert(isfinite(qdotUB)&&qdotUB>=0&&isfinite(VQ)&&VQ>=0, ...
                '[REPLANNER ERROR] derived cable/UAV speed bound invalid.');

            cert=struct();
            cert.certified=true;
            cert.continuous_time=true;
            cert.sampled_acceptance=false;
            cert.reference_mode='NOMINAL';
            cert.reference_version=double(obj.active_reference_version);
            cert.reference_class=class(obj.base_ref);
            cert.func_text=fstr;

            cert.payload_speed_UB_m_s=VL;
            cert.payload_accel_UB_m_s2=AL;
            cert.payload_jerk_UB_m_s3=JL;
            cert.specific_force_norm_LB_m_s2=sigmaLB;
            cert.qdot_UB_rad_s=qdotUB;
            cert.uav_speed_UB_m_s=VQ;

            cert.payload_speed_constant=logical(vConst);
            cert.payload_accel_norm_constant=logical(aConst);
            cert.payload_jerk_norm_constant=logical(jConst);
            cert.constant_norm_required=false;
            cert.general_bound_provider_used=logical(providerUsed);

            % Compatibility diagnostics only.  They are NOT prerequisites
            % for any final/general theorem.
            aL_zero=obj.symbolic_vector_equal(aL,sym(zeros(3,1)));
            cert.nominal_acceleration_zero=logical(aL_zero);
            if aL_zero
                vL_const=double(simplify(vL));
                if numel(vL_const)==3 && all(isfinite(vL_const(:)))
                    cert.payload_velocity_vector_m_s=vL_const(:);
                end
            end

            cert.xd_9_11_promoted=false;
            cert.source= ...
                'GENERAL_CONTINUOUS_REFERENCE_DERIVATIVE_BOUNDS_AND_CABLE_KINEMATICS';
            cert.symbolic_objects_retained=false;
        end


        function [ub,isConst] = exact_constant_vector_norm_ub(obj,v,t)
            q=simplify(v.'*v);
            isConst=obj.symbolic_zero(simplify(diff(q,t)));
            ub=NaN;
            if ~isConst
                return;
            end
            qn=double(simplify(q));
            assert(isscalar(qn)&&isfinite(qn)&&qn>=0, ...
                '[REPLANNER ERROR] exact constant vector norm is invalid.');
            ub=sqrt(max(0,qn));
        end


        function validate_nominal_derivative_bound_provider(~,B)
            assert(isstruct(B) && isscalar(B), ...
                '[REPLANNER ERROR] nominal derivative-bound provider must return a scalar struct.');
            req={'certified','continuous_time','sampled_acceptance', ...
                'payload_speed_UB_m_s','payload_accel_UB_m_s2', ...
                'payload_jerk_UB_m_s3'};
            for k=1:numel(req)
                assert(isfield(B,req{k}), ...
                    '[REPLANNER ERROR] nominal derivative-bound provider missing %s.',req{k});
            end
            assert(logical(B.certified) && logical(B.continuous_time) && ...
                ~logical(B.sampled_acceptance), ...
                ['[REPLANNER ERROR] nominal derivative-bound provider is ' ...
                 'not a continuous-time non-sampled certificate.']);
            z=[double(B.payload_speed_UB_m_s), ...
               double(B.payload_accel_UB_m_s2), ...
               double(B.payload_jerk_UB_m_s3)];
            assert(all(isfinite(z)) && all(z>=0), ...
                '[REPLANNER ERROR] nominal derivative-bound provider returned invalid bounds.');
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
                 'payload_speed_UB_m_s','uav_speed_UB_m_s','qdot_UB_rad_s'};
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
            WQ=cert.qdot_UB_rad_s;
            assert(isnumeric(VL)&&isscalar(VL)&&isfinite(VL)&&VL>=0);
            assert(isnumeric(VQ)&&isscalar(VQ)&&isfinite(VQ)&&VQ>=0);
            assert(isnumeric(WQ)&&isscalar(WQ)&&isfinite(WQ)&&WQ>=0);
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
                double(sp.uav_speed_UB_m_s)>=0 && ...
                isfield(sp,'qdot_UB_rad_s') && ...
                isnumeric(sp.qdot_UB_rad_s) && ...
                isscalar(sp.qdot_UB_rad_s) && ...
                isfinite(sp.qdot_UB_rad_s) && ...
                double(sp.qdot_UB_rad_s)>=0, ...
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
            sig.d_margin=double(obj.d_margin);
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
            sig.hlc_gain_generator= ...
                obj.runtime_source_signature.hlc_gain_generator;
            sig.hlc_gain_generator_source= ...
                obj.runtime_source_signature.hlc_gain_generator_source;
            sig.hlc_gain_packet_signature= ...
                obj.runtime_source_signature.hlc_gain_packet_signature;
            sig.hlc_binding_schema= ...
                obj.runtime_source_signature.hlc_binding_schema;
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

            % R41Z-DIRECT: bind the actual suspended-load gain generator before
            % the live HLC handle is verified.  This remains setup/refresh only.
            obj.runtime_source_signature= ...
                obj.capture_runtime_source_signature();
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


        function Jdep = get_inertia_dependency_fast(obj)
            % Read live inertia without constructing a string-array query in
            % the common path when direct parameter properties are available.
            P=obj.self.parameter;

            if isobject(P) && isprop(P,'jx') && isprop(P,'jy') && isprop(P,'jz')
                Jdep=[P.jx;P.jy;P.jz];
            elseif isstruct(P) && isfield(P,'jx') && isfield(P,'jy') && isfield(P,'jz')
                Jdep=[P.jx;P.jy;P.jz];
            else
                Jdep=P.get(["jx","jy","jz"]);
            end

            Jdep=double(Jdep(:));
            assert(numel(Jdep)==3 && all(isfinite(Jdep)) && all(Jdep>0), ...
                '[REPLANNER ERROR] invalid live inertia parameter dependency.');
        end


        function sig = capture_runtime_dependency_signature_fast(obj)
            obj.assert_hard_modify_deadline_limit();
            sig=struct();
            sig.reference_mode=char(obj.active_reference_mode);
            sig.reference_version=double(obj.active_reference_version);

            % Flight-reference time origin is part of the reference authority,
            % but it is NOT constructor-mandatory.  In the existing system
            % run_flight_headless may bind takeoff_end_s only after the
            % REPLANNING_SYSTEM object is constructed.  Therefore construction
            % must tolerate an unresolved origin and bind it later, before the
            % Flight nominal table is generated.
            [tf0,tf0ok]=obj.try_resolve_takeoff_end_time_nonfatal();
            if tf0ok
                obj.flight_reference_time_origin=double(tf0);
                sig.flight_reference_time_origin=double(tf0);
            else
                sig.flight_reference_time_origin=NaN;
            end

            if isa(obj.base_ref,'TIME_VARYING_REFERENCE') && ...
                    isprop(obj.base_ref,'func') && ...
                    isa(obj.base_ref.func,'function_handle')
                obj.runtime_reference_func_handle=obj.base_ref.func;
                sig.reference_func_signature=func2str(obj.base_ref.func);
            else
                obj.runtime_reference_func_handle=[];
                sig.reference_func_signature='';
            end

            sig.dt=double(obj.prediction_dt);
            sig.sensor_range=double(obj.trigger_dist);
            sig.horizon=double(obj.horizon_s);
            sig.r_drone=double(obj.r_drone);
            sig.r_load=double(obj.r_load);
            sig.r_cable=double(obj.r_cable);
            sig.d_margin=double(obj.d_margin);
            sig.cable_length=double(obj.L_cable);
            sig.gravity=double(obj.gravity);
            sig.mQ=obj.get_vehicle_mass();
            Jdep=obj.get_inertia_dependency_fast();
            sig.jx=Jdep(1);
            sig.jy=Jdep(2);
            sig.jz=Jdep(3);
            sig.mL_snapshot=double(obj.loadmass);
            sig.hard_modify_deadline_s=double(obj.hard_modify_deadline_s);
        end


        function sig = capture_runtime_source_signature(obj)
            % =============================================================
            % R41Z-DIRECT actual suspended-load HLC gain authority.
            %
            % run_flight_headless constructs the live controller as
            %   HLC_SUSPENDED_LOAD(agent,Controller_HL_Suspended_Load(dt,agent))
            % so Controller_HL(dt) is NOT the gain authority for this controller.
            % This heavy/source-bound capture is setup/refresh only.
            % =============================================================
            sig=struct();
            sig.command_limit_source_signature= ...
                obj.command_limit_certificate.source_line_signature;
            sig.command_limits=obj.command_limit_certificate.limits;

            gainName='Controller_HL_Suspended_Load';
            gainSource=which(gainName);
            assert(~isempty(gainSource) && isfile(gainSource), ...
                '[REPLANNER ERROR] suspended-load HLC gain generator source unresolved.');

            % Suppress diagnostic console output produced inside the current
            % gain generator while preserving its exact numerical result.
            evalc('S=Controller_HL_Suspended_Load(obj.prediction_dt,obj.self);');
            P=obj.extract_hlc_gain_packet(S);

            % Preserve legacy dependency-field names, but bind them to the
            % ACTUAL suspended-load gain authority.
            sig.hlc_source=gainSource;
            sig.hlc_settings_signature=obj.serialize_numeric_struct(S);

            sig.hlc_gain_generator=gainName;
            sig.hlc_gain_generator_source=gainSource;
            sig.hlc_gain_packet=P;
            sig.hlc_gain_packet_signature=obj.serialize_numeric_struct(P);
            sig.hlc_controller_source=which('HLC_SUSPENDED_LOAD');
            sig.hlc_binding_schema='R41Z_DIRECT_ACTUAL_SUSPENDED_LOAD_GAIN_BINDING_V1';
        end

        function update_runtime_dependency_state_fast(obj,cha)
            % =============================================================
            % Allocation-light live dependency watcher.
            %
            % Common unchanged cycle:
            %   - NO new dependency struct
            %   - NO func2str()
            %   - direct scalar comparisons against the certified snapshot
            %
            % Change cycle:
            %   - preserve the previous fail-closed invalidation/rebuild rules
            %   - rebuild the snapshot only after the detected change
            % =============================================================
            old=obj.runtime_dependency_signature;

            if ~isstruct(old) || ~isfield(old,'dt')
                obj.runtime_dependency_signature= ...
                    obj.capture_runtime_dependency_signature_fast();
                old=obj.runtime_dependency_signature;
            end

            % Reference mode/version are cheap scalar/string dependencies.
            refChanged = ...
                ~strcmp(char(old.reference_mode),char(obj.active_reference_mode)) || ...
                double(old.reference_version)~=double(obj.active_reference_version);

            [tfNow,tfNowOk]=obj.try_resolve_takeoff_end_time_nonfatal();
            if tfNowOk
                obj.flight_reference_time_origin=double(tfNow);
            end

            oldOrigin=NaN;
            if isfield(old,'flight_reference_time_origin')
                oldOrigin=double(old.flight_reference_time_origin);
            end

            if tfNowOk
                timeOriginChanged = ...
                    ~isfinite(oldOrigin) || oldOrigin~=double(tfNow);
            else
                % Unresolved-at-construction is a legitimate preflight state.
                % NaN->NaN is NOT a dependency change.
                timeOriginChanged=false;
            end

            % Avoid func2str in the common cycle. Only stringify a newly
            % installed function handle, preserving the previous textual
            % signature semantics.
            funcChanged=false;
            hasFunc=isa(obj.base_ref,'TIME_VARYING_REFERENCE') && ...
                isprop(obj.base_ref,'func') && ...
                isa(obj.base_ref.func,'function_handle');

            if hasFunc
                fh=obj.base_ref.func;
                if isempty(obj.runtime_reference_func_handle) || ...
                        ~isequal(fh,obj.runtime_reference_func_handle)
                    sigNow=func2str(fh);
                    funcChanged=~strcmp(char(old.reference_func_signature),sigNow);
                    if ~funcChanged
                        obj.runtime_reference_func_handle=fh;
                    end
                end
            else
                funcChanged=~isempty(char(old.reference_func_signature));
            end
            refChanged=refChanged || funcChanged;

            dtChanged=double(old.dt)~=double(obj.prediction_dt);

            sensorGeomChanged= ...
                double(old.sensor_range)~=double(obj.trigger_dist) || ...
                double(old.horizon)~=double(obj.horizon_s) || ...
                double(old.r_drone)~=double(obj.r_drone) || ...
                double(old.r_load)~=double(obj.r_load) || ...
                double(old.r_cable)~=double(obj.r_cable) || ...
                ~isfield(old,'d_margin') || ...
                double(old.d_margin)~=double(obj.d_margin) || ...
                double(old.cable_length)~=double(obj.L_cable);

            gravityChanged=double(old.gravity)~=double(obj.gravity);
            deadlineChanged= ...
                double(old.hard_modify_deadline_s)~= ...
                double(obj.hard_modify_deadline_s);

            mQNow=obj.get_vehicle_mass();
            massChanged= ...
                double(old.mQ)~=double(mQNow) || ...
                double(old.mL_snapshot)~=double(obj.loadmass);

            JNow=obj.get_inertia_dependency_fast();
            inertiaChanged= ...
                double(old.jx)~=JNow(1) || ...
                double(old.jy)~=JNow(2) || ...
                double(old.jz)~=JNow(3);

            anyChanged=refChanged || timeOriginChanged || ...
                dtChanged || sensorGeomChanged || ...
                gravityChanged || deadlineChanged || massChanged || inertiaChanged;

            if anyChanged
                if refChanged
                    obj.certificate_stale.SPEED_TRIGGER_CERT=true;
                    obj.certificate_stale.SENSOR_HORIZON_CERT=true;
                    obj.certificate_stale.STOP_GEOMETRY_4MS_CERT=true;
                    obj.certificate_stale.ACTUATOR_DYNAMICS_CERT=true;
                    obj.certificate_stale.HLC_ROBUST_CERT=true;
                end

                if timeOriginChanged
                    % Time-origin binding changes only the mapping
                    % t_global -> tau_reference.  It does NOT change the
                    % nominal law, speed bound, obstacle geometry, dynamics,
                    % or command limits.  Rebuild only cached reference data
                    % and preflight prime state.
                    obj.nominal_table_valid=false;
                    obj.nominal_table_t0=NaN;
                    obj.nominal_table_tend=NaN;
                    obj.nominal_table_xd=[];
                    obj.nominal_table_pL=[];
                    obj.nominal_table_q=[];
                    obj.reference_cache_valid=false;
                    obj.online_prime_done=false;
                    obj.setup_exact_do_prime_done=false;
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

                if inertiaChanged
                    if strcmpi(char(obj.active_reference_mode),'NOMINAL')
                        obj.actuator_dynamics_certificate= ...
                            obj.build_current_actuator_dynamics_certificate();
                        obj.certificate_stale.ACTUATOR_DYNAMICS_CERT= ...
                            ~logical(obj.actuator_dynamics_certificate.certified);
                    else
                        obj.certificate_stale.ACTUATOR_DYNAMICS_CERT=true;
                    end
                    obj.certificate_stale.HLC_ROBUST_CERT=true;
                end

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
                        obj.certificate_stale.ACTUATOR_DYNAMICS_CERT=true;
                        obj.certificate_stale.HLC_ROBUST_CERT=true;
                    end
                end

                % Build a fresh authoritative snapshot only when a dependency
                % actually changed.
                obj.runtime_dependency_signature= ...
                    obj.capture_runtime_dependency_signature_fast();
            end

            % Same fail-closed Flight rules as before.
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
            cert.live_gain_verified=false;
            cert.live_controller_path='';
            cert.initialization_binding='CLASS_SOURCE_BEFORE_LIVE_CONTROLLER';
        end


        function verify_live_controller_binding(obj,required_now)
            % =============================================================
            % R41Z-DIRECT live suspended-load HLC binding.
            %
            % One-time binding verifies class/source + actual F1..F4 against
            % Controller_HL_Suspended_Load(dt,agent).  After binding, each call
            % performs only an exact comparison of the small cached F1..F4
            % packet; the gain generator/lqrd is never rerun in the hot loop.
            % =============================================================
            if nargin<2
                required_now=false;
            end

            cert=obj.command_limit_certificate;
            assert(isstruct(cert) && isfield(cert,'certified') && ...
                logical(cert.certified), ...
                '[REPLANNER ERROR] command-limit source certificate unavailable.');

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

            assert(isstruct(obj.runtime_source_signature) && ...
                isfield(obj.runtime_source_signature,'hlc_gain_packet') && ...
                isfield(obj.runtime_source_signature,'hlc_gain_generator') && ...
                strcmp(char(obj.runtime_source_signature.hlc_gain_generator), ...
                    'Controller_HL_Suspended_Load'), ...
                ['[REPLANNER ERROR] actual suspended-load HLC source signature ' ...
                 'is unavailable/stale; refresh outside Flight.']);

            assert(isa(ctrl,cert.controller_class), ...
                '[REPLANNER ERROR] live controller class differs from command-limit certificate.');
            assert(isprop(ctrl,'param') && isstruct(ctrl.param), ...
                '[REPLANNER ERROR] live HLC parameter packet is unavailable.');

            expected=obj.runtime_source_signature.hlc_gain_packet;
            gainOK=obj.hlc_gain_packets_exact_equal(ctrl.param,expected);
            if ~gainOK
                cert.live_controller_verified=false;
                cert.live_gain_verified=false;
                cert.live_binding_reason= ...
                    'LIVE_HLC_GAIN_PACKET_DIFFERS_FROM_SUSPENDED_LOAD_GENERATOR';
                obj.command_limit_certificate=cert;
                if isstruct(obj.certificate_stale) && ...
                        isfield(obj.certificate_stale,'HLC_ROBUST_CERT')
                    obj.certificate_stale.HLC_ROBUST_CERT=true;
                end
                error(['[REPLANNER ERROR] live HLC F1/F2/F3/F4 differs from ' ...
                    'Controller_HL_Suspended_Load source-bound packet.']);
            end

            % Cheap steady-state path: exact gain equality above is the only
            % recurring check.  No which(), fileread(), lqrd(), or generator
            % reevaluation occurs after the one-time source binding.
            if isfield(cert,'live_controller_verified') && ...
                    logical(cert.live_controller_verified) && ...
                    isfield(cert,'live_gain_verified') && ...
                    logical(cert.live_gain_verified)
                return;
            end

            live_source=which(class(ctrl));
            assert(~isempty(live_source) && isfile(live_source), ...
                '[REPLANNER ERROR] live controller source file unresolved.');
            assert(strcmp(live_source,cert.controller_source), ...
                ['[REPLANNER ERROR] live HLC controller source differs from ' ...
                 'the source used for command-limit certification.']);

            gain_source=which('Controller_HL_Suspended_Load');
            assert(~isempty(gain_source) && isfile(gain_source), ...
                '[REPLANNER ERROR] suspended-load gain generator source unresolved.');
            assert(strcmp(gain_source, ...
                    obj.runtime_source_signature.hlc_gain_generator_source), ...
                ['[REPLANNER ERROR] suspended-load gain-generator source path ' ...
                 'changed; refresh certificates outside Flight.']);

            cert.live_controller_verified=true;
            cert.live_gain_verified=true;
            cert.live_controller_path='self.controller.hlc_suspended';
            cert.live_controller_class=class(ctrl);
            cert.live_controller_source=live_source;
            cert.live_gain_generator='Controller_HL_Suspended_Load';
            cert.live_gain_generator_source=gain_source;
            cert.live_gain_signature=obj.serialize_numeric_struct( ...
                obj.extract_hlc_gain_packet(ctrl.param));
            cert.live_binding_reason= ...
                'CLASS_SOURCE_AND_ACTUAL_SUSPENDED_LOAD_GAIN_PACKET_MATCH';
            obj.command_limit_certificate=cert;
        end

        function cert = build_current_actuator_dynamics_certificate(obj)
            % =============================================================
            % GENERAL actuator/dynamics certificate entry point.
            %
            % The old positive-vertical constant-velocity theorem is
            % deliberately disabled as production authority.  General
            % authority must be generated from the SAME active-reference
            % state/reference uncertainty image, HLC map, actuator limits
            % and tracking fixed point used for the final repaired
            % trajectory.
            %
            % Until that interval-HLC theorem is installed this certificate
            % fails closed.  No trajectory-specific fallback is allowed.
            % =============================================================
            cmd=obj.command_limit_certificate;
            assert(isstruct(cmd) && isfield(cmd,'certified') && ...
                logical(cmd.certified), ...
                '[REPLANNER ERROR] command limit certificate unavailable.');

            obj.validate_active_speed_certificate(obj.active_speed_certificate);

            cert=struct();
            cert.certified=false;
            cert.general_reference_required=true;
            cert.trajectory_specific_fallback_allowed=false;
            cert.legacy_positive_vertical_theorem_enabled=false;
            cert.reference_mode=char(obj.active_reference_mode);
            cert.reference_version=double(obj.active_reference_version);
            cert.mQ=obj.get_vehicle_mass();
            cert.mL_snapshot=double(obj.loadmass);
            cert.invalidate_on_mL_change=true;
            cert.command_limit_source_signature=cmd.source_line_signature;
            cert.command_limits=cmd.limits;
            cert.reason= ...
                'GENERAL_HLC_TRACKING_AND_ACTUATOR_INTERVAL_CERTIFICATE_NOT_YET_INSTALLED';
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


        function P = extract_hlc_gain_packet(~,S)
            req={'F1','F2','F3','F4'};
            assert(isstruct(S) && all(isfield(S,req)), ...
                '[REPLANNER ERROR] suspended-load HLC gain packet is incomplete.');

            P=struct();
            for kk=1:numel(req)
                v=S.(req{kk});
                assert(isnumeric(v) && ~isempty(v) && all(isfinite(v(:))), ...
                    '[REPLANNER ERROR] suspended-load HLC gain %s is invalid.',req{kk});
                P.(req{kk})=double(v);
            end
            if isfield(S,'dt')
                vdt=double(S.dt);
                assert(isscalar(vdt) && isfinite(vdt) && vdt>0, ...
                    '[REPLANNER ERROR] suspended-load HLC gain-packet dt is invalid.');
                P.dt=vdt;
            end
        end


        function tf = hlc_gain_packets_exact_equal(~,A,B)
            % Exact equality is intentional: the live controller and cached
            % packet are created from the same deterministic generator call.
            % No empirical/tuned tolerance is introduced into authority.
            req={'F1','F2','F3','F4'};
            tf=isstruct(A) && isstruct(B) && ...
                all(isfield(A,req)) && all(isfield(B,req));
            if ~tf
                return;
            end

            for kk=1:numel(req)
                a=A.(req{kk});
                b=B.(req{kk});
                if ~isnumeric(a) || ~isnumeric(b) || ...
                        ~isequal(size(a),size(b)) || ...
                        any(~isfinite(a(:))) || any(~isfinite(b(:))) || ...
                        ~isequaln(double(a),double(b))
                    tf=false;
                    return;
                end
            end
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
            % =============================================================
            % Certified stop-geometry T intervals with explicit endpoint
            % topology.
            %
            % branch1:
            %   lower Tf belongs to the first analytic branch.
            %   upper Tb is closed only when the strict clearance boundary
            %   b1upper lies beyond Tb. If b1upper is active (including
            %   b1upper==Tb), the strict-clearance endpoint is open.
            %
            % branch2:
            %   the quadratic clearance condition is strict between r1,r2.
            %   Root endpoints are therefore open. If Tb lies strictly
            %   inside (r1,r2), the branch switch at Tb is closed.
            %
            % No epsilon is used to emulate an open boundary.
            % =============================================================
            INT=struct();
            INT.branch1=zeros(0,2);
            INT.branch1_left_closed=false(0,1);
            INT.branch1_right_closed=false(0,1);
            INT.branch2=zeros(0,2);
            INT.branch2_left_closed=false(0,1);
            INT.branch2_right_closed=false(0,1);
            INT.nonempty=false;

            if v==0
                INT.branch1=[0,Inf];
                INT.branch1_left_closed=true;
                INT.branch1_right_closed=false;
                INT.nonempty=S>0;
                return;
            end

            Tf=k/g;
            Tb=2*Tf;

            b1upper=2*(S-2*L)/v;
            hi=min(Tb,b1upper);
            if hi>Tf
                INT.branch1=[Tf,hi];
                INT.branch1_left_closed=true;
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
                    INT.branch2_left_closed=(Tb>r1);
                    INT.branch2_right_closed=false;
                end
            end

            INT.nonempty=~isempty(INT.branch1) || ~isempty(INT.branch2);
        end


        function INT2 = intersect_T_intervals_with_lower(~,INT,Tlo)
            % Intersect with the closed lower-bound set [Tlo,Inf).
            INT2=struct();
            INT2.branch1=zeros(0,2);
            INT2.branch1_left_closed=false(0,1);
            INT2.branch1_right_closed=false(0,1);
            INT2.branch2=zeros(0,2);
            INT2.branch2_left_closed=false(0,1);
            INT2.branch2_right_closed=false(0,1);
            INT2.nonempty=false;

            for i=1:size(INT.branch1,1)
                oldlo=INT.branch1(i,1);
                lo=max(oldlo,Tlo);
                hi=INT.branch1(i,2);

                if lo>oldlo
                    lclosed=true;
                else
                    lclosed=INT.branch1_left_closed(i);
                end
                hclosed=INT.branch1_right_closed(i);

                nonempty=(hi>lo) || (hi==lo && lclosed && hclosed);
                if nonempty
                    INT2.branch1(end+1,:)=[lo,hi]; %#ok<AGROW>
                    INT2.branch1_left_closed(end+1,1)=lclosed; %#ok<AGROW>
                    INT2.branch1_right_closed(end+1,1)=hclosed; %#ok<AGROW>
                end
            end

            for i=1:size(INT.branch2,1)
                oldlo=INT.branch2(i,1);
                lo=max(oldlo,Tlo);
                hi=INT.branch2(i,2);

                if lo>oldlo
                    lclosed=true;
                else
                    lclosed=INT.branch2_left_closed(i);
                end
                hclosed=INT.branch2_right_closed(i);

                nonempty=(hi>lo) || (hi==lo && lclosed && hclosed);
                if nonempty
                    INT2.branch2(end+1,:)=[lo,hi]; %#ok<AGROW>
                    INT2.branch2_left_closed(end+1,1)=lclosed; %#ok<AGROW>
                    INT2.branch2_right_closed(end+1,1)=hclosed; %#ok<AGROW>
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


        function set_generator_backend_flag(obj,flag)
            %#ok<INUSD>
            error(['[REPLANNER ERROR] generator backend is immutable after construction. ' ...
                'Pass generator_backend_flag in opts before Flight.']);
        end


        function set_generator_backend_external_flag_fn(obj,fn)
            %#ok<INUSD>
            error(['[REPLANNER ERROR] generator backend provider is constructor-only. ' ...
                'Pass generator_backend_external_flag_fn in opts; it is sampled once.']);
        end


        function set_generator_backend_callback(obj,flag,fn)
            % Register/replace one backend callback BEFORE preflight freeze.
            assert(~obj.generator_backend_configuration_frozen, ...
                ['[REPLANNER ERROR] generator backend callbacks are frozen before Flight. ' ...
                 'Register callbacks before simulation/preflight prime.']);
            [code,name,~]=obj.normalize_generator_backend_flag(flag);
            if ~isempty(fn)
                assert(isa(fn,'function_handle'), ...
                    '[REPLANNER ERROR] backend callback must be a function handle or [].');
            end
            C=obj.normalize_generator_backend_callbacks(obj.generator_backend_callbacks);
            C.(name)=fn;
            obj.generator_backend_callbacks=C;
            %#ok<NASGU> code is validation only; locked backend never changes.
        end


        function C = get_generator_backend_catalog(obj)
            C=struct();
            C.codes=[1 2 3 4];
            C.names={'BEZIER_D15_C7','POLY_D15_C7', ...
                'MINCO_STYLE_D15_C7','BSPLINE_D8_C7'};
            C.current_internal_flag=double(obj.generator_backend_flag);
            C.locked_code=double(obj.generator_backend_locked_code);
            C.locked_name=char(obj.generator_backend_locked_name);
            C.locked_source_code=double(obj.generator_backend_locked_source_code);
            C.selection_locked=logical(obj.generator_backend_selection_locked);
            C.configuration_frozen=logical(obj.generator_backend_configuration_frozen);
            C.selected_callback_installed=~isempty(obj.generator_backend_locked_callback);
            C.external_selector_installed=false;
            C.callbacks=obj.normalize_generator_backend_callbacks( ...
                obj.generator_backend_callbacks);
        end


        function [code,name,source_code] = resolve_generator_backend_selection(obj)
            assert(obj.generator_backend_selection_locked, ...
                '[REPLANNER ERROR] generator backend selection is not locked before Flight.');
            code=double(obj.generator_backend_locked_code);
            name=char(obj.generator_backend_locked_name);
            source_code=double(obj.generator_backend_locked_source_code);
        end


        function freeze_generator_backend_configuration(obj)
            if obj.generator_backend_configuration_frozen
                return;
            end
            assert(obj.generator_backend_selection_locked, ...
                '[REPLANNER ERROR] generator backend must be locked before callback freeze.');

            C=obj.normalize_generator_backend_callbacks(obj.generator_backend_callbacks);
            name=char(obj.generator_backend_locked_name);
            fn=C.(name);
            if isempty(fn)
                fn=obj.repaired_candidate_generator_fn;
            end

            if isempty(fn) && double(obj.generator_backend_locked_code)==1
                % Self-contained default backend.  Production runtime must
                % not depend on G27/Rxx audit/test files.
                fn=@(CTX)obj.builtin_bezier_d15_c7_proposal(CTX);
            end

            if ~isempty(fn)
                assert(isa(fn,'function_handle'), ...
                    '[REPLANNER ERROR] selected generator callback is invalid.');
            end

            obj.generator_backend_locked_callback=fn;
            obj.generator_backend_configuration_frozen=true;
            obj.generator_backend_last_callback_installed=~isempty(fn);
        end


        function C = normalize_generator_backend_callbacks(~,C)
            names={'BEZIER_D15_C7','POLY_D15_C7', ...
                'MINCO_STYLE_D15_C7','BSPLINE_D8_C7'};

            if isempty(C)
                C=struct();
            end
            assert(isstruct(C) && isscalar(C), ...
                '[REPLANNER ERROR] generator_backend_callbacks must be a scalar struct.');

            for k=1:numel(names)
                n=names{k};
                if ~isfield(C,n)
                    C.(n)=[];
                elseif ~isempty(C.(n))
                    assert(isa(C.(n),'function_handle'), ...
                        '[REPLANNER ERROR] backend callback %s must be a function handle or [].',n);
                end
            end
        end


        function [code,name,canonical] = normalize_generator_backend_flag(~,flag)
            names={'BEZIER_D15_C7','POLY_D15_C7', ...
                'MINCO_STYLE_D15_C7','BSPLINE_D8_C7'};

            if isnumeric(flag) || islogical(flag)
                assert(isscalar(flag) && isfinite(double(flag)) && ...
                    double(flag)==round(double(flag)) && ...
                    double(flag)>=1 && double(flag)<=4, ...
                    '[REPLANNER ERROR] generator backend numeric flag must be integer 1..4.');
                code=double(flag);
                name=names{code};
                canonical=name;
                return;
            end

            assert(ischar(flag) || (isstring(flag)&&isscalar(flag)), ...
                '[REPLANNER ERROR] generator backend flag must be numeric 1..4 or a backend name.');

            s=upper(strtrim(char(flag)));
            switch s
                case {'1','BEZIER','BEZIER_D15_C7'}
                    code=1;
                case {'2','POLY','POLYNOMIAL','POLY_D15_C7'}
                    code=2;
                case {'3','MINCO','MINCO_STYLE','MINCO_STYLE_D15_C7'}
                    code=3;
                case {'4','BSPLINE','B-SPLINE','BSPLINE_D8_C7'}
                    code=4;
                otherwise
                    error('[REPLANNER ERROR] unknown generator backend flag/name: %s',s);
            end
            name=names{code};
            canonical=name;
        end


        function invoke_replan_generator_shadow(obj,t_hard_cycle,t_now, ...
                xd_nom,xd_active_now,pQ_cur,pL_cur,q_cur,cable_residual, ...
                active,all_dists,pL_ref,q_ref,ref_source, ...
                ttc,collision_time,obs_id,comp_name,comp_code, ...
                pL_collision,q_collision,pQ_collision,jhit)
            % =============================================================
            % R19 compact detected-only request path.
            %
            % IMPORTANT SENSOR CONTRACT:
            %   The generator receives ONLY currently detected obstacles.
            %   The full simulation obstacle map is never exposed.
            %
            % Backend selection/callback are frozen before Flight.
            % =============================================================

            ti=tic;
            obj.generator_request_count=obj.generator_request_count+1;

            elapsed_before=toc(t_hard_cycle);
            budget_remaining=double(obj.hard_modify_deadline_s)-elapsed_before;
            obj.generator_last_elapsed_before_ms=1e3*elapsed_before;
            obj.generator_last_budget_remaining_ms=1e3*budget_remaining;

            if budget_remaining<=0
                obj.generator_deadline_fail_count=obj.generator_deadline_fail_count+1;
                obj.generator_last_status_code=-2;
                obj.generator_last_context_ms=0;
                obj.generator_last_callback_ms=0;
                obj.generator_last_validation_ms=0;
                obj.generator_last_critical_ms=1e3*toc(ti);
                obj.generator_last_cycle_critical_ms=1e3*toc(t_hard_cycle);
                return;
            end

            assert(obj.generator_backend_configuration_frozen, ...
                '[REPLANNER ERROR] generator backend configuration not frozen before Flight.');

            CTX=obj.build_generator_request_context_compact( ...
                t_now,elapsed_before,budget_remaining, ...
                xd_nom,xd_active_now,pQ_cur,pL_cur,q_cur,cable_residual, ...
                active,all_dists,pL_ref,q_ref,ref_source, ...
                ttc,collision_time,obs_id,comp_name,comp_code, ...
                pL_collision,q_collision,pQ_collision,jhit);

            cp_context=1e3*toc(ti);
            obj.generator_last_context_ms=cp_context;

            % Fixed callback contract:
            %   []               -> no candidate
            %   certified struct -> candidate
            candidate=[];
            callback_ok=true;

            if isempty(obj.generator_backend_locked_callback)
                % Exact real no-generator branch. This SAME branch is primed
                % before Flight for every detected-obstacle count.
                candidate=[];
            else
                try
                    candidate=obj.generator_backend_locked_callback(CTX);
                catch
                    callback_ok=false;
                end
            end

            cp_callback=1e3*toc(ti);
            obj.generator_last_callback_ms=cp_callback-cp_context;
            obj.generator_last_runtime_ms=obj.generator_last_callback_ms;

            if ~callback_ok
                obj.generator_validation_fail_count=obj.generator_validation_fail_count+1;
                obj.generator_last_status_code=-1;
                obj.generator_last_validation_ms=0;
                obj.generator_last_critical_ms=cp_callback;
                cycle_ms=1e3*toc(t_hard_cycle);
                obj.generator_last_cycle_critical_ms=cycle_ms;
                if cycle_ms>1e3*obj.hard_modify_deadline_s
                    obj.generator_deadline_fail_count=obj.generator_deadline_fail_count+1;
                end
                return;
            end

            if isempty(candidate)
                obj.generator_last_status_code=1;
                obj.generator_last_validation_ms=0;
                obj.generator_last_critical_ms=cp_callback;
                cycle_ms=1e3*toc(t_hard_cycle);
                obj.generator_last_cycle_critical_ms=cycle_ms;
                if cycle_ms>1e3*obj.hard_modify_deadline_s
                    obj.generator_deadline_fail_count=obj.generator_deadline_fail_count+1;
                    obj.generator_last_status_code=-7;
                end
                return;
            end

            % Real trajectory SHADOW proposal: generated shape, but incomplete
            % production proof stack. It is logged and never committed.
            if isstruct(candidate) && isscalar(candidate) && ...
                    isfield(candidate,'shadow_proposal') && ...
                    logical(candidate.shadow_proposal)
                obj.record_bezier_shadow_proposal(candidate);
                obj.generator_last_validation_ms=0;
                obj.generator_last_critical_ms=cp_callback;
                cycle_ms=1e3*toc(t_hard_cycle);
                obj.generator_last_cycle_critical_ms=cycle_ms;

                if cycle_ms>1e3*obj.hard_modify_deadline_s
                    obj.generator_deadline_fail_count=obj.generator_deadline_fail_count+1;
                    obj.generator_last_status_code=-20;
                elseif isfield(candidate,'success') && logical(candidate.success)
                    obj.generator_last_status_code=20;
                else
                    obj.generator_last_status_code=21;
                end
                return;
            end

            if ~isstruct(candidate) || ~isscalar(candidate) || ...
                    ~isfield(candidate,'certified')
                obj.generator_validation_fail_count=obj.generator_validation_fail_count+1;
                obj.generator_last_status_code=-3;
                obj.generator_last_validation_ms=0;
                obj.generator_last_critical_ms=cp_callback;
                cycle_ms=1e3*toc(t_hard_cycle);
                obj.generator_last_cycle_critical_ms=cycle_ms;
                if cycle_ms>1e3*obj.hard_modify_deadline_s
                    obj.generator_deadline_fail_count=obj.generator_deadline_fail_count+1;
                end
                return;
            end

            obj.generator_candidate_count=obj.generator_candidate_count+1;

            try
                obj.validate_repaired_candidate(candidate,double(t_now));
                validation_ok=true;
            catch
                validation_ok=false;
            end

            cp_validation=1e3*toc(ti);
            obj.generator_last_validation_ms=cp_validation-cp_callback;
            obj.generator_last_critical_ms=cp_validation;

            cycle_ms=1e3*toc(t_hard_cycle);
            obj.generator_last_cycle_critical_ms=cycle_ms;

            if cycle_ms>1e3*obj.hard_modify_deadline_s
                obj.generator_deadline_fail_count=obj.generator_deadline_fail_count+1;
                obj.generator_last_status_code=-6;
                return;
            end

            if ~validation_ok
                obj.generator_validation_fail_count=obj.generator_validation_fail_count+1;
                obj.generator_last_status_code=-5;
                return;
            end

            obj.generator_validation_pass_count=obj.generator_validation_pass_count+1;
            obj.generator_last_status_code=4; % valid candidate, SHADOW only
        end


        function OUT = builtin_bezier_d15_c7_proposal(obj,CTX)
            % =============================================================
            % Self-contained degree-15 C7 endpoint-repeated BEZIER proposal.
            %
            % This is the in-class production copy of the formerly external
            % fast proposal logic.  It computes an outward displacement w
            % that clears all currently DETECTED conservative payload/UAV/
            % cable ellipsoids at the correction peak.
            %
            % This function is a proposal generator only.  Continuous
            % all-span/whole-body certification remains a separate authority
            % before any repaired reference is committed.
            % =============================================================
            OUT=obj.empty_bezier_d15_c7_proposal();

            if ~isstruct(CTX) || ~isfield(CTX,'static') || ...
                    ~isfield(CTX,'request') || ...
                    ~isfield(CTX,'obstacles') || ...
                    ~isfield(CTX,'live')
                OUT.reason_code=-100;
                return;
            end

            S=CTX.static;
            RQ=CTX.request;
            O=CTX.obstacles;

            if double(S.backend_code)~=1 || ...
                    ~strcmpi(char(S.backend_name),'BEZIER_D15_C7')
                OUT.reason_code=-101;
                return;
            end

            OUT.t0=double(RQ.time);
            OUT.trigger_collision_time=double(RQ.trigger_collision_time);
            OUT.t1=double(RQ.time)+double(CTX.live.horizon);
            OUT.obstacle_id=double(RQ.trigger_obstacle_id);
            OUT.component_code=double(RQ.trigger_component_code);

            if O.count<1
                OUT.reason_code=1;
                return;
            end
            if ~isfinite(double(RQ.trigger_ttc)) || double(RQ.trigger_ttc)<0
                OUT.reason_code=2;
                return;
            end

            % Trigger witness time and BEZIER peak time are different
            % concepts.  A first/last prediction-grid hit gives
            % collision_time==t0/t1 and must not collapse a C7 segment.
            %
            % Proposal timing is projected to the interior control-grid
            % interval [t0+dt,t1-dt].  This timing choice is NOT a safety
            % certificate; the continuous whole-body certificate below is
            % the authority.  No obstacle/trajectory-specific threshold is
            % used.
            dtLive=double(CTX.live.dt);
            assert(isfinite(dtLive)&&dtLive>0, ...
                '[REPLANNER ERROR] live dt invalid in BEZIER proposal.');
            left=OUT.t0+dtLive;
            right=OUT.t1-dtLive;
            if ~(right>left)
                OUT.reason_code=3;
                return;
            end
            tc=OUT.trigger_collision_time;
            if ~isfinite(tc)
                OUT.reason_code=3;
                return;
            end
            OUT.tm=min(max(tc,left),right);
            OUT.timing_projected=(OUT.tm~=tc);

            if ~(OUT.tm>OUT.t0) || ~(OUT.t1>OUT.tm)
                OUT.reason_code=3;
                return;
            end

            kt=find(double(O.ids)==OUT.obstacle_id,1,'first');
            if isempty(kt)
                OUT.reason_code=4;
                return;
            end

            Rt=reshape(double(O.rotations_flat(:,kt)),3,3);
            ct=double(O.centers(:,kt));

            switch round(OUT.component_code)
                case 1
                    at=double(O.payload_axes(:,kt));
                    xw=double(RQ.trigger_pL_collision(:));
                    before=obj.bezier_point_metric(xw,ct,Rt,at);
                case 2
                    at=double(O.uav_axes(:,kt));
                    xw=double(RQ.trigger_pQ_collision(:));
                    before=obj.bezier_point_metric(xw,ct,Rt,at);
                case 3
                    at=double(O.cable_axes(:,kt));
                    pQ0=double(RQ.trigger_pQ_collision(:));
                    pL0=double(RQ.trigger_pL_collision(:));
                    [before,~,xw]=obj.bezier_segment_metric( ...
                        pQ0,pL0,ct,Rt,at);
                otherwise
                    OUT.reason_code=5;
                    return;
            end

            if any(~isfinite(at)) || any(at<=0) || ...
                    any(~isfinite(Rt(:))) || any(~isfinite(xw))
                OUT.reason_code=6;
                return;
            end
            OUT.trigger_metric_before=before;

            yl=Rt.'*(xw-ct);
            gw=Rt*(yl./(at.^2));
            ng=norm(gw);
            if ~isfinite(ng) || ng<=0
                OUT.reason_code=7;
                return;
            end
            dir=gw/ng;

            % Numerical strictness only; not a physical margin.
            target=1+32*eps(1);

            pLbase=double(RQ.trigger_pL_collision(:));
            pQbase=double(RQ.trigger_pQ_collision(:));

            alphaReq=0;
            for j=1:double(O.count)
                Rj=reshape(double(O.rotations_flat(:,j)),3,3);
                cj=double(O.centers(:,j));

                [ap,okp]=obj.bezier_point_last_exit_alpha( ...
                    pLbase,cj,Rj,double(O.payload_axes(:,j)),dir,target);
                [aq,okq]=obj.bezier_point_last_exit_alpha( ...
                    pQbase,cj,Rj,double(O.uav_axes(:,j)),dir,target);
                [ac,okc]=obj.bezier_segment_last_exit_alpha( ...
                    pQbase,pLbase,cj,Rj,double(O.cable_axes(:,j)),dir,target);

                if ~(okp && okq && okc)
                    OUT.reason_code=8;
                    return;
                end
                alphaReq=max([alphaReq,ap,aq,ac]);
            end

            alpha=alphaReq + 64*eps(max(1,alphaReq));
            w=alpha*dir;

            Tup=OUT.tm-OUT.t0;
            Tdn=OUT.t1-OUT.tm;
            if ~(Tup>0) || ~(Tdn>0)
                OUT.reason_code=9;
                return;
            end

            K=obj.bezier_c7_basis_derivative_constants();
            dub=norm(w)*K(:).*max(Tup.^-(1:6)',Tdn.^-(1:6)');

            pLpeak=pLbase+w;
            pQpeak=pQbase+w;

            switch round(OUT.component_code)
                case 1
                    after=obj.bezier_point_metric(pLpeak,ct,Rt,at);
                case 2
                    after=obj.bezier_point_metric(pQpeak,ct,Rt,at);
                case 3
                    after=obj.bezier_segment_metric(pQpeak,pLpeak,ct,Rt,at);
            end

            minPeak=inf;
            for j=1:double(O.count)
                Rj=reshape(double(O.rotations_flat(:,j)),3,3);
                cj=double(O.centers(:,j));
                mp=obj.bezier_point_metric( ...
                    pLpeak,cj,Rj,double(O.payload_axes(:,j)));
                mq=obj.bezier_point_metric( ...
                    pQpeak,cj,Rj,double(O.uav_axes(:,j)));
                mc=obj.bezier_segment_metric( ...
                    pQpeak,pLpeak,cj,Rj,double(O.cable_axes(:,j)));
                minPeak=min([minPeak,mp,mq,mc]);
            end

            OUT.generated=true;
            OUT.c7_exact=true;
            OUT.displacement=w;
            OUT.displacement_norm_m=norm(w);
            OUT.direction=dir;
            OUT.trigger_metric_after=after;
            OUT.peak_min_metric=minPeak;
            OUT.peak_all_safe=isfinite(minPeak) && minPeak>1;
            OUT.correction_derivative_ub=dub;
            OUT.representation_code=1;
            OUT.degree=15;

            % Continuous-time static-obstacle whole-body theorem.
            G=obj.build_static_continuous_wholebody_shadow_certificate(CTX,OUT);
            OUT.geometry_shadow_certificate=G;

            if OUT.c7_exact && logical(G.mathematical_pass)
                OUT.success=true;
                OUT.reason_code=0;
            elseif ~OUT.peak_all_safe
                % Peak-only failure is retained as a useful diagnostic, but
                % the continuous certificate is the actual geometry gate.
                OUT.reason_code=12;
            else
                OUT.reason_code=13; % continuous support-tube certificate failed
            end
        end


        function G = build_static_continuous_wholebody_shadow_certificate(obj,CTX,P)
            % =============================================================
            % STATIC ELLIPSOID CONTINUOUS-TIME WHOLE-BODY CERTIFICATE
            %
            % Obstacles:
            %   E_i = { c_i + R_i diag(a_i) u : ||u||_2 <= 1 }.
            %
            % Candidate:
            %   The degree-15 C7 correction translates pL and pQ by the
            %   same w*b(t), so cable direction q is unchanged by the
            %   correction itself.
            %
            % Finite-cover theorem:
            %   If ||xdot|| <= V on every time cell and samples are spaced
            %   h, any continuous point in the cell lies inside a Euclidean
            %   ball of radius V*h/2 around its nearest sample.
            %
            % For ANY nonzero separating normal n, positivity of
            %
            %   n'x - [n'c + ||diag(a) R'n||] - rho||n||
            %
            % proves separation of point x from E_i (+) B_rho.
            %
            % For a cable segment conv{pQ,pL}, replace n'x by
            % min(n'pQ,n'pL).  This proves the WHOLE segment is on the safe
            % side of the support plane.
            %
            % Therefore checking every grid center with
            %   rho = body_radius + d_margin + V*h/2
            % proves the complete continuous-time interval, not merely the
            % sampled instants.
            %
            % The normal-search affects completeness only.  Any positive
            % returned support margin is a valid mathematical certificate.
            %
            % NOTE: binary64 directed-rounding closure is deliberately kept
            % false here until the fixed-dimension MEX/outward kernel is
            % installed.  Thus this is a mathematical continuous-time
            % geometry certificate, not yet final numerical authority.
            % =============================================================

            G=struct( ...
                'mathematical_pass',false, ...
                'certified',false, ...
                'continuous_time',true, ...
                'whole_body',true, ...
                'all_detected_obstacles',true, ...
                'static_obstacles_only',true, ...
                'sampled_acceptance',false, ...
                'finite_cover_speed_tube',true, ...
                'support_function_separation',true, ...
                'ieee754_outward_closed',false, ...
                'reason_code',0, ...
                'min_margin_m',NaN, ...
                'min_payload_margin_m',NaN, ...
                'min_uav_margin_m',NaN, ...
                'min_cable_margin_m',NaN, ...
                'leaf_count',0, ...
                'accepted_leaf_count',0, ...
                'split_count',0, ...
                'max_depth_used',0, ...
                'max_depth_cap',0, ...
                'min_alpha_lb',NaN, ...
                'max_qdot_ub',NaN, ...
                'max_q_arc_radius',NaN, ...
                'max_q_arc_inflation_m',NaN, ...
                'resource_exhausted',false, ...
                'tracking_inflation_m',0, ...
                'final_system_authority',false);

            try
                RQ=CTX.request;
                O=CTX.obstacles;
                LIV=CTX.live;

                pL0=double(RQ.future_pL);
                q0=double(RQ.future_q);
                N=size(pL0,2);

                if N<2 || size(q0,2)~=N || double(O.count)<1
                    G.reason_code=101;
                    return;
                end

                dt=double(LIV.dt);
                VL0=double(LIV.active_payload_speed_ub_m_s);
                VQ0=double(LIV.active_uav_speed_ub_m_s);
                qdot0=double(LIV.active_qdot_ub_rad_s);
                vcorr=double(P.correction_derivative_ub(1));

                if any(~isfinite([dt VL0 VQ0 qdot0 vcorr])) || ...
                        dt<=0 || min([VL0 VQ0 qdot0 vcorr])<0
                    G.reason_code=102;
                    return;
                end

                VL=VL0+vcorr;
                VQ=VQ0+vcorr;
                VC=max(VL,VQ);

                % Static whole-body radii plus continuous-time temporal tube.
                rhoL=double(LIV.r_load)+double(LIV.d_margin)+VL*dt/2;
                rhoQ=double(LIV.r_drone)+double(LIV.d_margin)+VQ*dt/2;
                rhoC=double(LIV.r_cable)+double(LIV.d_margin)+VC*dt/2;

                tgrid=double(RQ.time)+(0:N-1)*dt;
                corr=zeros(3,N);
                for jj=1:N
                    b=obj.bezier_c7_bump_scalar( ...
                        tgrid(jj),double(P.t0),double(P.tm),double(P.t1));
                    corr(:,jj)=double(P.displacement(:))*b;
                end

                pL=pL0+corr;
                pQ=pL0-double(LIV.cable_length)*q0+corr;

                minL=inf;
                minQ=inf;
                minC=inf;
                checks=0;

                for jj=1:N
                    for ii=1:double(O.count)
                        c=double(O.centers(:,ii));
                        R=reshape(double(O.rotations_flat(:,ii)),3,3);
                        a=double(O.base_axes(:,ii));

                        if any(~isfinite([c;R(:);a])) || any(a<=0)
                            G.reason_code=103;
                            return;
                        end

                        [mL,okL]=obj.best_point_support_margin( ...
                            pL(:,jj),c,R,a,rhoL);
                        [mQ,okQ]=obj.best_point_support_margin( ...
                            pQ(:,jj),c,R,a,rhoQ);
                        [mC,okC]=obj.best_segment_support_margin( ...
                            pQ(:,jj),pL(:,jj),c,R,a,rhoC);

                        checks=checks+3;
                        if ~(okL && okQ && okC)
                            G.reason_code=104;
                            G.leaf_count=checks;
                            return;
                        end

                        minL=min(minL,mL);
                        minQ=min(minQ,mQ);
                        minC=min(minC,mC);
                    end
                end

                G.min_payload_margin_m=minL;
                G.min_uav_margin_m=minQ;
                G.min_cable_margin_m=minC;
                G.min_margin_m=min([minL,minQ,minC]);
                G.leaf_count=checks;
                G.accepted_leaf_count=checks;
                G.max_qdot_ub=qdot0;
                G.max_q_arc_radius=qdot0*dt/2;
                G.max_q_arc_inflation_m=double(LIV.cable_length)*qdot0*dt/2;

                if isfinite(G.min_margin_m) && G.min_margin_m>0
                    G.mathematical_pass=true;
                    G.certified=true;
                    G.reason_code=0;
                else
                    G.reason_code=105;
                end
            catch
                G.reason_code=199;
                G.mathematical_pass=false;
                G.certified=false;
            end
        end


        function b = bezier_c7_bump_scalar(obj,t,t0,tm,t1)
            if t<=t0 || t>=t1
                b=0;
                return;
            end

            if t<=tm
                u=(t-t0)/(tm-t0);
                c=[zeros(1,8),ones(1,8)];
            else
                u=(t-tm)/(t1-tm);
                c=[ones(1,8),zeros(1,8)];
            end

            u=max(0,min(1,u));
            b=obj.decasteljau_scalar_local(c,u);
        end


        function [margin,ok] = best_point_support_margin(obj,x,c,R,a,rho)
            y=R.'*(x-c);
            normals=[ ...
                R*(y./(a.^2)), ...
                (x-c) ...
                ];

            margin=-inf;
            ok=false;
            for kk=1:size(normals,2)
                [m,v]=obj.support_margin_point( ...
                    x,c,R,a,rho,normals(:,kk));
                if v && m>margin
                    margin=m;
                    ok=true;
                end
            end
            ok=ok && isfinite(margin) && margin>0;
        end


        function [margin,ok] = best_segment_support_margin(obj,p0,p1,c,R,a,rho)
            [~,~,xstar]=obj.bezier_segment_metric(p0,p1,c,R,a);

            Y=[xstar,p0,p1];
            normals=zeros(3,0);
            for kk=1:size(Y,2)
                y=R.'*(Y(:,kk)-c);
                normals(:,end+1)=R*(y./(a.^2)); %#ok<AGROW>
                normals(:,end+1)=Y(:,kk)-c; %#ok<AGROW>
            end

            margin=-inf;
            ok=false;
            for kk=1:size(normals,2)
                [m,v]=obj.support_margin_segment( ...
                    p0,p1,c,R,a,rho,normals(:,kk));
                if v && m>margin
                    margin=m;
                    ok=true;
                end
            end
            ok=ok && isfinite(margin) && margin>0;
        end


        function [margin_m,ok] = support_margin_point(~,x,c,R,a,rho,n)
            n=double(n(:));
            nn=norm(n);
            if ~(isfinite(nn)&&nn>0)
                margin_m=-inf;
                ok=false;
                return;
            end

            h=dot(n,c)+norm(a.*(R.'*n));
            raw=dot(n,x)-h-rho*nn;
            margin_m=raw/nn;
            ok=isfinite(margin_m);
        end


        function [margin_m,ok] = support_margin_segment(~,p0,p1,c,R,a,rho,n)
            n=double(n(:));
            nn=norm(n);
            if ~(isfinite(nn)&&nn>0)
                margin_m=-inf;
                ok=false;
                return;
            end

            h=dot(n,c)+norm(a.*(R.'*n));
            proj=min(dot(n,p0),dot(n,p1));
            raw=proj-h-rho*nn;
            margin_m=raw/nn;
            ok=isfinite(margin_m);
        end


        function OUT = empty_bezier_d15_c7_proposal(~)
            OUT=struct( ...
                'shadow_proposal',true, ...
                'backend','BEZIER_D15_C7', ...
                'generated',false, ...
                'success',false, ...
                'reason_code',0, ...
                't0',NaN,'tm',NaN,'t1',NaN, ...
                'trigger_collision_time',NaN, ...
                'timing_projected',false, ...
                'obstacle_id',NaN,'component_code',NaN, ...
                'displacement',nan(3,1), ...
                'displacement_norm_m',NaN, ...
                'direction',nan(3,1), ...
                'trigger_metric_before',NaN, ...
                'trigger_metric_after',NaN, ...
                'peak_min_metric',NaN, ...
                'peak_all_safe',false, ...
                'c7_exact',false, ...
                'correction_derivative_ub',nan(6,1), ...
                'representation_code',1, ...
                'degree',15, ...
                'geometry_shadow_certificate',struct(), ...
                'external_test_dependency',false);
        end


        function K = bezier_c7_basis_derivative_constants(~)
            persistent KK
            if isempty(KK)
                c=[zeros(1,8),ones(1,8)];
                D=c;
                KK=zeros(6,1);
                for r=1:6
                    D=(numel(D)-1)*diff(D);
                    KK(r)=max(abs(D));
                end
            end
            K=KK;
        end


        function m = bezier_point_metric(~,x,c,R,a)
            z=(R.'*(x-c))./a;
            m=norm(z);
        end


        function [m,lambda,xstar] = bezier_segment_metric(~,p0,p1,c,R,a)
            z0=(R.'*(p0-c))./a;
            z1=(R.'*(p1-c))./a;
            sv=z1-z0;
            ss=dot(sv,sv);
            if ss<=0
                lambda=0;
            else
                lambda=max(0,min(1,-dot(z0,sv)/ss));
            end
            z=z0+lambda*sv;
            m=norm(z);
            xstar=p0+lambda*(p1-p0);
        end


        function [alpha,ok] = bezier_point_last_exit_alpha(obj,x,c,R,a,d,target)
            y=(R.'*(x-c))./a;
            v=(R.'*d)./a;

            A=dot(v,v);
            B=2*dot(y,v);
            C=dot(y,y)-target^2;

            if ~(A>0) || any(~isfinite([A B C]))
                alpha=NaN;
                ok=false;
                return;
            end

            rr=obj.bezier_quad_roots_nonnegative(A,B,C);
            if isempty(rr)
                if C>0
                    alpha=0;
                    ok=true;
                else
                    alpha=NaN;
                    ok=false;
                end
            else
                alpha=max(rr);
                ok=true;
            end
        end


        function [alpha,ok] = bezier_segment_last_exit_alpha(obj,p0,p1,c,R,a,d,target)
            z0=(R.'*(p0-c))./a;
            z1=(R.'*(p1-c))./a;
            sv=z1-z0;
            v=(R.'*d)./a;
            ss=dot(sv,sv);

            if any(~isfinite([z0;z1;v])) || dot(v,v)<=0
                alpha=NaN;
                ok=false;
                return;
            end

            if ss<=0
                [alpha,ok]=obj.bezier_point_last_exit_alpha( ...
                    p0,c,R,a,d,target);
                return;
            end

            cand=zeros(1,6);
            nc=0;

            rr=obj.bezier_quad_roots_nonnegative( ...
                dot(v,v),2*dot(z0,v),dot(z0,z0)-target^2);
            for k=1:numel(rr)
                lam=-(dot(sv,z0)+rr(k)*dot(sv,v))/ss;
                if lam<=64*eps(max(1,abs(lam)))
                    nc=nc+1;
                    cand(nc)=rr(k);
                end
            end

            rr=obj.bezier_quad_roots_nonnegative( ...
                dot(v,v),2*dot(z1,v),dot(z1,z1)-target^2);
            for k=1:numel(rr)
                lam=-(dot(sv,z0)+rr(k)*dot(sv,v))/ss;
                if lam>=1-64*eps(max(1,abs(lam)))
                    nc=nc+1;
                    cand(nc)=rr(k);
                end
            end

            Pz=z0-sv*(dot(sv,z0)/ss);
            Pv=v-sv*(dot(sv,v)/ss);
            rr=obj.bezier_quad_roots_nonnegative( ...
                dot(Pv,Pv),2*dot(Pz,Pv),dot(Pz,Pz)-target^2);
            for k=1:numel(rr)
                lam=-(dot(sv,z0)+rr(k)*dot(sv,v))/ss;
                tol=64*eps(max(1,abs(lam)));
                if lam>=-tol && lam<=1+tol
                    nc=nc+1;
                    cand(nc)=rr(k);
                end
            end

            if nc>0
                alpha=max(cand(1:nc));
                ok=isfinite(alpha) && alpha>=0;
                return;
            end

            m0=obj.bezier_segment_metric(p0,p1,c,R,a);
            if isfinite(m0) && m0>target
                alpha=0;
                ok=true;
            else
                alpha=NaN;
                ok=false;
            end
        end


        function b = decasteljau_scalar_local(~,c,u)
            q=double(c(:).');
            u=double(u);
            n=numel(q)-1;
            for r=1:n
                q(1:n-r+1)=(1-u).*q(1:n-r+1)+u.*q(2:n-r+2);
            end
            b=q(1);
        end


        function rr = bezier_quad_roots_nonnegative(~,A,B,C)
            rr=zeros(1,0);
            if ~(isfinite(A)&&isfinite(B)&&isfinite(C))
                return;
            end

            if abs(A)<=eps(max(1,abs(B)))
                if abs(B)<=eps(max(1,abs(C)))
                    return;
                end
                r=-C/B;
                if isfinite(r) && r>=0
                    rr=r;
                end
                return;
            end

            disc=B*B-4*A*C;
            tol=128*eps(max(1,max(abs([B*B,4*A*C]))));
            if disc < -tol
                return;
            end
            disc=max(0,disc);
            sd=sqrt(disc);

            if B>=0
                q=-0.5*(B+sd);
            else
                q=-0.5*(B-sd);
            end

            if q==0
                roots=[-B/(2*A),-B/(2*A)];
            else
                roots=[q/A,C/q];
            end

            roots=roots(isfinite(roots) & roots>=0);
            if ~isempty(roots)
                rr=unique(roots);
            end
        end


        function record_bezier_shadow_proposal(obj,P)
            % Scalar-only hot-path record. The full 3x16 control arrays are
            % implied by (w,t0,tm,t1) and are not copied into obj/state.
            obj.generator_shadow_proposal_count=obj.generator_shadow_proposal_count+1;

            if isfield(P,'generated') && logical(P.generated)
                obj.bezier_shadow_generated_count=obj.bezier_shadow_generated_count+1;
            end
            if isfield(P,'peak_all_safe') && logical(P.peak_all_safe)
                obj.bezier_shadow_peak_safe_count=obj.bezier_shadow_peak_safe_count+1;
            end

            if isfield(P,'reason_code'), obj.bezier_shadow_last_reason_code=double(P.reason_code); end
            if isfield(P,'t0'), obj.bezier_shadow_last_t0=double(P.t0); end
            if isfield(P,'tm'), obj.bezier_shadow_last_tm=double(P.tm); end
            if isfield(P,'t1'), obj.bezier_shadow_last_t1=double(P.t1); end
            if isfield(P,'obstacle_id'), obj.bezier_shadow_last_obstacle_id=double(P.obstacle_id); end
            if isfield(P,'component_code'), obj.bezier_shadow_last_component_code=double(P.component_code); end
            if isfield(P,'displacement_norm_m'), obj.bezier_shadow_last_displacement_m=double(P.displacement_norm_m); end
            if isfield(P,'trigger_metric_before'), obj.bezier_shadow_last_metric_before=double(P.trigger_metric_before); end
            if isfield(P,'trigger_metric_after'), obj.bezier_shadow_last_metric_after=double(P.trigger_metric_after); end
            if isfield(P,'peak_min_metric'), obj.bezier_shadow_last_peak_min_metric=double(P.peak_min_metric); end
            if isfield(P,'peak_all_safe'), obj.bezier_shadow_last_peak_all_safe=logical(P.peak_all_safe); end
            if isfield(P,'c7_exact'), obj.bezier_shadow_last_c7_exact=logical(P.c7_exact); end

            if isfield(P,'displacement') && isnumeric(P.displacement) && numel(P.displacement)==3
                w=double(P.displacement(:));
                obj.bezier_shadow_last_dx=w(1);
                obj.bezier_shadow_last_dy=w(2);
                obj.bezier_shadow_last_dz=w(3);
            end

            if isfield(P,'correction_derivative_ub') && numel(P.correction_derivative_ub)>=6
                u=double(P.correction_derivative_ub(:));
                obj.bezier_shadow_last_d1_ub=u(1);
                obj.bezier_shadow_last_d2_ub=u(2);
                obj.bezier_shadow_last_d3_ub=u(3);
                obj.bezier_shadow_last_d4_ub=u(4);
                obj.bezier_shadow_last_d5_ub=u(5);
                obj.bezier_shadow_last_d6_ub=u(6);
            end

            if isfield(P,'geometry_shadow_certificate') && ...
                    isstruct(P.geometry_shadow_certificate)
                G=P.geometry_shadow_certificate;
                if isfield(G,'mathematical_pass'), obj.geometry_shadow_pass=logical(G.mathematical_pass); end
                if isfield(G,'reason_code'), obj.geometry_shadow_reason_code=double(G.reason_code); end
                if isfield(G,'continuous_time'), obj.geometry_shadow_continuous_time=logical(G.continuous_time); end
                if isfield(G,'ieee754_outward_closed'), obj.geometry_shadow_ieee754_outward_closed=logical(G.ieee754_outward_closed); end
                if isfield(G,'min_margin_m'), obj.geometry_shadow_min_margin_m=double(G.min_margin_m); end
                if isfield(G,'min_payload_margin_m'), obj.geometry_shadow_min_payload_margin_m=double(G.min_payload_margin_m); end
                if isfield(G,'min_uav_margin_m'), obj.geometry_shadow_min_uav_margin_m=double(G.min_uav_margin_m); end
                if isfield(G,'min_cable_margin_m'), obj.geometry_shadow_min_cable_margin_m=double(G.min_cable_margin_m); end
                if isfield(G,'leaf_count'), obj.geometry_shadow_leaf_count=double(G.leaf_count); end
                if isfield(G,'split_count'), obj.geometry_shadow_split_count=double(G.split_count); end
                if isfield(G,'max_depth_used'), obj.geometry_shadow_max_depth=double(G.max_depth_used); end
                if isfield(G,'min_alpha_lb'), obj.geometry_shadow_min_alpha_lb=double(G.min_alpha_lb); end
                if isfield(G,'max_qdot_ub'), obj.geometry_shadow_max_qdot_ub=double(G.max_qdot_ub); end
                if isfield(G,'max_q_arc_radius'), obj.geometry_shadow_max_q_arc_radius=double(G.max_q_arc_radius); end
                if isfield(G,'max_q_arc_inflation_m'), obj.geometry_shadow_max_q_arc_inflation_m=double(G.max_q_arc_inflation_m); end
                if isfield(G,'resource_exhausted'), obj.geometry_shadow_resource_exhausted=logical(G.resource_exhausted); end
            end
        end


        function latch_real_trigger_event(obj,t_now, ...
                xd_nom,xd_active_now,pQ_cur,pL_cur,q_cur,cable_residual, ...
                active,all_dists,ref_source, ...
                ttc,collision_time,obs_id,comp_name,comp_code, ...
                pL_collision,q_collision,pQ_collision,jhit, ...
                effective_horizon,N)
            % =============================================================
            % Persistent authority snapshot of the ACTUAL Flight request edge.
            %
            % Important:
            %   - called only from do() when need_replan rises false->true,
            %   - setup/prime paths are excluded by the caller,
            %   - only currently DETECTED obstacles are copied,
            %   - no file I/O, no base-workspace mutation,
            %   - no trigger time is inferred; event_time=t_now exactly.
            % =============================================================

            assert(~obj.setup_exact_do_prime_active, ...
                '[REPLANNER ERROR] prime path attempted to latch a real trigger event.');
            assert(~obj.generator_integration_prime_active, ...
                '[REPLANNER ERROR] generator prime attempted to latch a real trigger event.');

            idx=find(active);
            P=obj.obstacle_packet;

            OBS=struct( ...
                'count',double(numel(idx)), ...
                'ids',P.ids(idx), ...
                'surface_distances',all_dists(idx), ...
                'centers',P.C(:,idx), ...
                'rotations_flat',P.R(:,idx), ...
                'base_axes',P.Araw(:,idx), ...
                'payload_axes',P.Apay(:,idx), ...
                'uav_axes',P.Auav(:,idx), ...
                'cable_axes',P.Acable(:,idx));

            STATE=struct( ...
                'pQ_current',pQ_cur(:), ...
                'pL_current',pL_cur(:), ...
                'q_current',q_cur(:), ...
                'cable_residual',double(cable_residual), ...
                'xd_nominal',xd_nom(:), ...
                'xd_active',xd_active_now(:));

            TRIG=struct( ...
                'ttc',double(ttc), ...
                'collision_time',double(collision_time), ...
                'obstacle_id',double(obs_id), ...
                'component',char(comp_name), ...
                'component_code',double(comp_code), ...
                'pL_collision',pL_collision(:), ...
                'q_collision',q_collision(:), ...
                'pQ_collision',pQ_collision(:), ...
                'hit_index',double(jhit), ...
                'effective_horizon',double(effective_horizon), ...
                'reference_points',double(N));

            tf0=obj.flight_reference_time_origin;
            if ~(isnumeric(tf0)&&isscalar(tf0)&&isfinite(tf0))
                tf0=obj.resolve_takeoff_end_time();
                obj.flight_reference_time_origin=double(tf0);
            end

            REF=struct( ...
                'mode',char(obj.active_reference_mode), ...
                'version',double(obj.active_reference_version), ...
                'source',char(ref_source), ...
                'global_time',double(t_now), ...
                'flight_time_origin',double(tf0), ...
                'reference_time',double(t_now)-double(tf0));

            E=struct();
            E.schema='REPLANNING_TRIGGER_EVENT_V1_TIME_ORIGIN_BOUND';
            E.valid=true;
            E.need_replan=true;
            E.real_flight_request_edge=true;
            E.event_time=double(t_now);
            E.phase='FLIGHT';
            E.setup_prime=false;
            E.generator_prime=false;
            E.state=STATE;
            E.trigger=TRIG;
            E.detected_obstacles=OBS;
            E.reference=REF;
            E.dependency_signature=obj.runtime_dependency_signature;
            E.speed_certificate=obj.active_speed_certificate;
            E.sensor_horizon_certificate=obj.sensor_horizon_certificate;
            E.capture_runtime_ms=NaN;

            obj.trigger_event_count=obj.trigger_event_count+1;
            obj.last_trigger_event=E;

            if obj.trigger_event_count==1
                obj.first_trigger_event=E;
            end

            % LOGGER-safe persistent summary.  This is written once per real
            % request edge, not every control cycle.
            st=obj.result.state;
            st.trigger_event_latched=true;
            st.trigger_event_count=double(obj.trigger_event_count);
            st.trigger_event_time_latched=double(t_now);
            st.trigger_event_ttc_latched=double(ttc);
            st.trigger_event_collision_time_latched=double(collision_time);
            st.trigger_event_obstacle_id_latched=double(obs_id);
            st.trigger_event_component_code_latched=double(comp_code);
            st.trigger_event_capture_ms=NaN;
        end


        function CTX = build_generator_request_context_compact(obj,t_now, ...
                elapsed_before,budget_remaining, ...
                xd_nom,xd_active_now,pQ_cur,pL_cur,q_cur,cable_residual, ...
                active,all_dists,pL_ref,q_ref,ref_source, ...
                ttc,collision_time,obs_id,comp_name,comp_code, ...
                pL_collision,q_collision,pQ_collision,jhit)
            % =============================================================
            % Compact context:
            %   STATIC  : small immutable interface/backend facts
            %   REQUEST : current state/reference/trigger only
            %   OBS     : DETECTED obstacles only
            %   LIVE    : compact live physical/dependency values
            %
            % Full static_obstacle_map and large certificate structs are
            % intentionally NOT passed to the generator.
            % =============================================================

            idx=find(active);
            P=obj.obstacle_packet;

            OBS=struct( ...
                'count',double(numel(idx)), ...
                'ids',P.ids(idx), ...
                'surface_distances',all_dists(idx), ...
                'centers',P.C(:,idx), ...
                'rotations_flat',P.R(:,idx), ...
                'base_axes',P.Araw(:,idx), ...
                'payload_axes',P.Apay(:,idx), ...
                'uav_axes',P.Auav(:,idx), ...
                'cable_axes',P.Acable(:,idx), ...
                'raw',obj.static_obstacle_map(idx));

            REQ=struct( ...
                'time',double(t_now), ...
                'deadline_remaining_s',double(budget_remaining), ...
                'elapsed_before_generator_s',double(elapsed_before), ...
                'next_reference_version',double(obj.active_reference_version)+1, ...
                'active_reference_mode',obj.active_reference_mode, ...
                'active_reference_version',double(obj.active_reference_version), ...
                'xd_nominal',xd_nom(:), ...
                'xd_active',xd_active_now(:), ...
                'pQ_current',pQ_cur(:), ...
                'pL_current',pL_cur(:), ...
                'q_current',q_cur(:), ...
                'cable_residual',double(cable_residual), ...
                'future_pL',pL_ref, ...
                'future_q',q_ref, ...
                'future_reference_source',ref_source, ...
                'trigger_ttc',double(ttc), ...
                'trigger_collision_time',double(collision_time), ...
                'trigger_obstacle_id',double(obs_id), ...
                'trigger_component',comp_name, ...
                'trigger_component_code',double(comp_code), ...
                'trigger_pL_collision',pL_collision(:), ...
                'trigger_q_collision',q_collision(:), ...
                'trigger_pQ_collision',pQ_collision(:), ...
                'trigger_hit_index',double(jhit));

            D=obj.runtime_dependency_signature;

            % General ACTIVE-reference kinematic bounds.
            SC=obj.active_speed_certificate;
            obj.validate_active_speed_certificate(SC);

            LIVE=struct( ...
                'dependency_refresh_count',double(obj.dependency_refresh_count), ...
                'dt',double(D.dt), ...
                'sensor_range',double(D.sensor_range), ...
                'horizon',double(D.horizon), ...
                'r_drone',double(D.r_drone), ...
                'r_load',double(D.r_load), ...
                'r_cable',double(D.r_cable), ...
                'd_margin',double(D.d_margin), ...
                'cable_length',double(D.cable_length), ...
                'gravity',double(D.gravity), ...
                'mQ',double(D.mQ), ...
                'jx',double(D.jx), ...
                'jy',double(D.jy), ...
                'jz',double(D.jz), ...
                'mL',double(D.mL_snapshot), ...
                'active_speed_certified', ...
                    isstruct(SC) && isfield(SC,'certified') && logical(SC.certified), ...
                'active_payload_speed_ub_m_s',double(SC.payload_speed_UB_m_s), ...
                'active_uav_speed_ub_m_s',double(SC.uav_speed_UB_m_s), ...
                'active_qdot_ub_rad_s',double(SC.qdot_UB_rad_s), ...
                'active_constant_speed_required',false, ...
                'hard_modify_deadline_s',double(D.hard_modify_deadline_s), ...
                'command_limits',obj.command_limit_certificate.limits);

            STATIC=struct( ...
                'interface_version','REPLANNING_GENERATOR_CONTEXT_V1', ...
                'shadow_only',true, ...
                'backend_code',double(obj.generator_backend_locked_code), ...
                'backend_name',obj.generator_backend_locked_name, ...
                'backend_source_code',double(obj.generator_backend_locked_source_code), ...
                'detected_only_obstacle_contract',true);

            CTX=struct('static',STATIC,'request',REQ,'obstacles',OBS,'live',LIVE);
        end


        function prime_generator_integration_variants(obj,t_now, ...
                xd_nom,xd_active_now,pQ_cur,pL_cur,q_cur,cable_residual, ...
                all_dists,pL_ref,q_ref,ref_source,masks)
            if obj.generator_integration_prime_done
                return;
            end

            % Save externally visible counters/timing.
            saved=struct( ...
                'request_count',obj.generator_request_count, ...
                'candidate_count',obj.generator_candidate_count, ...
                'vpass',obj.generator_validation_pass_count, ...
                'vfail',obj.generator_validation_fail_count, ...
                'dfail',obj.generator_deadline_fail_count, ...
                'runtime',obj.generator_last_runtime_ms, ...
                'scode',obj.generator_last_status_code, ...
                'before',obj.generator_last_elapsed_before_ms, ...
                'budget',obj.generator_last_budget_remaining_ms, ...
                'ctx',obj.generator_last_context_ms, ...
                'cb',obj.generator_last_callback_ms, ...
                'val',obj.generator_last_validation_ms, ...
                'critical',obj.generator_last_critical_ms, ...
                'cycle',obj.generator_last_cycle_critical_ms, ...
                'callback',obj.generator_backend_locked_callback);

            % Prime the exact REAL empty-callback branch, not a special probe.
            obj.generator_backend_locked_callback=[];
            obj.generator_integration_prime_active=true;
            tp=tic;
            nvar=0;

            try
                for rep=1:2
                    for im=1:size(masks,1)
                        active=masks(im,:);
                        thard=tic;
                        obj.invoke_replan_generator_shadow( ...
                            thard,t_now,xd_nom,xd_active_now, ...
                            pQ_cur,pL_cur,q_cur,cable_residual, ...
                            active,all_dists,pL_ref,q_ref,ref_source, ...
                            0,double(t_now),1,'PAYLOAD',1, ...
                            pL_cur,q_cur,pQ_cur,1);
                        nvar=nvar+1;
                    end
                end
                obj.generator_integration_prime_ms=1e3*toc(tp);
                obj.generator_integration_prime_variants=nvar;
                obj.generator_integration_prime_done=true;
                obj.generator_integration_prime_active=false;
            catch ME
                obj.generator_integration_prime_active=false;
                obj.generator_backend_locked_callback=saved.callback;
                rethrow(ME);
            end

            obj.generator_backend_locked_callback=saved.callback;

            % Restore counters/timing after pure warmup.
            obj.generator_request_count=saved.request_count;
            obj.generator_candidate_count=saved.candidate_count;
            obj.generator_validation_pass_count=saved.vpass;
            obj.generator_validation_fail_count=saved.vfail;
            obj.generator_deadline_fail_count=saved.dfail;
            obj.generator_last_runtime_ms=saved.runtime;
            obj.generator_last_status_code=saved.scode;
            obj.generator_last_elapsed_before_ms=saved.before;
            obj.generator_last_budget_remaining_ms=saved.budget;
            obj.generator_last_context_ms=saved.ctx;
            obj.generator_last_callback_ms=saved.cb;
            obj.generator_last_validation_ms=saved.val;
            obj.generator_last_critical_ms=saved.critical;
            obj.generator_last_cycle_critical_ms=saved.cycle;
        end


        function prime_locked_backend_kernel_variants(obj,t_now, ...
                xd_nom,xd_active_now,pQ_cur,pL_cur,q_cur,cable_residual, ...
                all_dists,pL_ref,q_ref,ref_source,masks)
            if obj.generator_backend_kernel_prime_done
                return;
            end

            if isempty(obj.generator_backend_locked_callback)
                obj.generator_backend_kernel_prime_done=true;
                obj.generator_backend_kernel_prime_ms=0;
                obj.generator_backend_kernel_prime_variants=0;
                return;
            end

            P=obj.obstacle_packet;
            tp=tic;
            nvar=0;

            for rep=1:2
                for im=1:size(masks,1)
                    active=masks(im,:);
                    idx=find(active,1,'first');
                    if isempty(idx)
                        continue;
                    end

                    Robs=reshape(P.R(:,idx),3,3);
                    axes=P.Apay(:,idx);
                    xcol=P.C(:,idx)+Robs*[0.5*axes(1);0;0];

                    ttcPrime=0.5*obj.horizon_s;
                    pLcol=xcol;
                    qcol=q_cur(:);
                    pQcol=pLcol-obj.L_cable*qcol;

                    CTX=obj.build_generator_request_context_compact( ...
                        t_now,0,obj.hard_modify_deadline_s, ...
                        xd_nom,xd_active_now,pQ_cur,pL_cur,q_cur,cable_residual, ...
                        active,all_dists,pL_ref,q_ref,ref_source, ...
                        ttcPrime,t_now+ttcPrime,P.ids(idx),'PAYLOAD',1, ...
                        pLcol,qcol,pQcol,1);

                    obj.generator_backend_locked_callback(CTX);
                    nvar=nvar+1;
                end
            end

            obj.generator_backend_kernel_prime_ms=1e3*toc(tp);
            obj.generator_backend_kernel_prime_variants=nvar;
            obj.generator_backend_kernel_prime_done=true;
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
                error(['[REPLANNER ERROR] estimator pT is unavailable. ' ...
                    'Do not reconstruct current physical cable direction ' ...
                    'from legacy xd(9:11); current geometry must come from ' ...
                    'the live state estimator/sensor path.']);
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
                tau0=obj.global_to_nominal_tau(t_entry);
                [pL0,q0]=obj.nominal_payload_and_q_from_xd(xd_now,tau0);
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

                tau0=obj.global_to_nominal_tau(t_entry);
                [pL0,q0]=obj.nominal_payload_and_q_from_xd(xd_now,tau0);
                pL_ref(:,1)=pL0;
                q_ref(:,1)=q0;
                source='rolling_base_ref_do_1_new_endpoint';
            else
                pL_ref=zeros(3,N);
                q_ref=zeros(3,N);
                tau0=obj.global_to_nominal_tau(t_entry);
                [pL0,q0]=obj.nominal_payload_and_q_from_xd(xd_now,tau0);
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


        function [pL,q,xd_eval] = evaluate_nominal_payload_and_q( ...
                obj,t_query_global,cha,varargin_in,~)
            args = varargin_in;
            time_orig = args{1};

            % -------------------------------------------------------------
            % G27-V74R41 Flight-reference time-origin correction.
            %
            % Simulation/controller time remains absolute/global.
            % Only the base Flight reference is evaluated in Flight-relative
            % time:
            %       tau = t_global - t_flight_start.
            %
            % This is intentionally applied at the unique base_ref future
            % evaluation boundary so trigger TTC/collision_time, logs, commit
            % validity, and the control-cycle clock stay in global time.
            % -------------------------------------------------------------
            t_eval=double(t_query_global);
            te_eval=[];

            if isequal(cha,'f')
                tf0=obj.flight_reference_time_origin;
                if ~(isnumeric(tf0)&&isscalar(tf0)&&isfinite(tf0))
                    tf0=obj.resolve_takeoff_end_time();
                    obj.flight_reference_time_origin=double(tf0);
                end

                t_eval=double(t_query_global)-double(tf0);
                tol=1e-10*max(1,abs(double(t_query_global)));
                assert(t_eval>=-tol, ...
                    '[REPLANNER ERROR] Flight reference queried before Flight time origin.');
                if t_eval<0
                    t_eval=0;
                end

                try
                    if isstruct(time_orig) && isfield(time_orig,'te')
                        te_eval=double(time_orig.te)-double(tf0);
                    elseif isobject(time_orig) && isprop(time_orig,'te')
                        te_eval=double(time_orig.te)-double(tf0);
                    end
                catch
                    te_eval=[];
                end
            end

            % time may be TIME < handle; never mutate the main simulation
            % clock. Construct an independent query object/struct.
            if isa(time_orig, 'TIME')
                if isempty(te_eval) || ~isfinite(te_eval)
                    te_local=time_orig.te;
                else
                    te_local=te_eval;
                end
                tlocal = TIME(t_eval, time_orig.dt, te_local);
                if isprop(time_orig, 'k') || isfield(time_orig, 'k')
                    tlocal.k = time_orig.k;
                end
            elseif isobject(time_orig)
                try
                    tlocal = copy(time_orig);
                    tlocal.t = t_eval;
                    if ~isempty(te_eval) && isfinite(te_eval) && isprop(tlocal,'te')
                        tlocal.te=te_eval;
                    end
                catch
                    tlocal = struct('t', t_eval, 'dt', obj.prediction_dt);
                    if ~isempty(te_eval) && isfinite(te_eval)
                        tlocal.te=te_eval;
                    end
                end
            elseif isstruct(time_orig)
                tlocal = time_orig;
                tlocal.t = t_eval;
                if ~isempty(te_eval) && isfinite(te_eval) && isfield(tlocal,'te')
                    tlocal.te=te_eval;
                end
            else
                tlocal = struct('t', t_eval, 'dt', obj.prediction_dt);
            end

            args{1} = tlocal;
            args{2} = cha;
            r = obj.base_ref.do(args{:});
            xd_eval = r.state.xd;
            [pL,q] = obj.nominal_payload_and_q_from_xd(xd_eval,t_eval);
        end

        function [pL,q] = nominal_payload_and_q_from_xd(obj,xd,tau)
            assert(numel(xd)>=3, ...
                '[REPLANNER ERROR] nominal reference packet lacks position.');
            assert(nargin>=3 && isnumeric(tau) && isscalar(tau) && isfinite(tau), ...
                '[REPLANNER ERROR] exact nominal reference time tau is required.');
            assert(isa(obj.nominal_acceleration_fn,'function_handle'), ...
                ['[REPLANNER ERROR] exact nominal acceleration function is ' ...
                 'not initialized. Refresh the nominal kinematic certificate.']);

            pL=double(xd(1:3));
            aL=double(obj.nominal_acceleration_fn(double(tau)));
            aL=aL(:);
            assert(numel(aL)==3 && all(isfinite(aL)), ...
                '[REPLANNER ERROR] exact nominal acceleration evaluation failed.');

            specific=aL+[0;0;obj.gravity];
            ns=norm(specific);
            assert(isfinite(ns)&&ns>0, ...
                '[REPLANNER ERROR] nominal specific-force vector is singular.');
            q=-specific/ns;
        end


        function tau = global_to_nominal_tau(obj,t_global)
            tf0=double(obj.flight_reference_time_origin);
            if ~(isfinite(tf0))
                tf0=double(obj.resolve_takeoff_end_time());
                obj.flight_reference_time_origin=tf0;
            end
            tau=double(t_global)-tf0;
            tol=1e-10*max(1,abs(double(t_global)));
            assert(tau>=-tol, ...
                '[REPLANNER ERROR] nominal reference queried before Flight origin.');
            if tau<0
                tau=0;
            end
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
            pack.Araw=zeros(3,M);
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
                pack.Araw(:,i)=obj.raw_enclosing_ellipsoid_axes(o);
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

            % Persistent real-event summary.  During ordinary preflight this
            % is zero/NaN.  Once a real Flight request edge is latched, the
            % helper writes these fields and clear_state_sensor_values() mirrors
            % (rather than erases) that persistent object state.
            st.trigger_event_latched=obj.trigger_event_count>0;
            st.trigger_event_count=double(obj.trigger_event_count);
            if obj.trigger_event_count>0 && isstruct(obj.last_trigger_event) && ...
                    isfield(obj.last_trigger_event,'event_time')
                st.trigger_event_time_latched=double(obj.last_trigger_event.event_time);
                st.trigger_event_ttc_latched=double(obj.last_trigger_event.trigger.ttc);
                st.trigger_event_collision_time_latched= ...
                    double(obj.last_trigger_event.trigger.collision_time);
                st.trigger_event_obstacle_id_latched= ...
                    double(obj.last_trigger_event.trigger.obstacle_id);
                st.trigger_event_component_code_latched= ...
                    double(obj.last_trigger_event.trigger.component_code);
                st.trigger_event_capture_ms=double(obj.trigger_event_capture_ms);
            else
                st.trigger_event_time_latched=NaN;
                st.trigger_event_ttc_latched=NaN;
                st.trigger_event_collision_time_latched=NaN;
                st.trigger_event_obstacle_id_latched=NaN;
                st.trigger_event_component_code_latched=NaN;
                st.trigger_event_capture_ms=NaN;
            end

            st.active_reference_mode=obj.active_reference_mode;
            st.active_reference_version=obj.active_reference_version;
            st.nominal_xd_table_used=false;
            st.certificate_guard_passed=false;
            st.certificate_stale_mask = ...
                double(obj.certificate_stale.SPEED_TRIGGER_CERT) + ...
                2*double(obj.certificate_stale.SENSOR_HORIZON_CERT) + ...
                4*double(obj.certificate_stale.STOP_GEOMETRY_4MS_CERT) + ...
                8*double(obj.certificate_stale.ACTUATOR_DYNAMICS_CERT);
            st.hlc_optional_stale=logical(obj.certificate_stale.HLC_ROBUST_CERT);
            if isfield(obj.command_limit_certificate,'live_controller_verified')
                st.command_limit_live_verified= ...
                    logical(obj.command_limit_certificate.live_controller_verified);
            else
                st.command_limit_live_verified=false;
            end
            st.dependency_refresh_count=obj.dependency_refresh_count;
            st.mass_certificate_rebuild_count=obj.mass_certificate_rebuild_count;
            st.loadmass=obj.loadmass;
            st.replan_commit_count=obj.replan_commit_count;
            st.last_replan_commit_time=obj.last_replan_commit_time;
            st.generator_shadow_enabled=logical(obj.generator_shadow_enabled);
            st.generator_callback_installed=~isempty(obj.repaired_candidate_generator_fn);
            st.generator_request_count=obj.generator_request_count;
            st.generator_candidate_count=obj.generator_candidate_count;
            st.generator_validation_pass_count=obj.generator_validation_pass_count;
            st.generator_validation_fail_count=obj.generator_validation_fail_count;
            st.generator_deadline_fail_count=obj.generator_deadline_fail_count;
            st.generator_last_runtime_ms=obj.generator_last_runtime_ms;
            st.generator_last_status_code=obj.generator_last_status_code;
            [flagCode,~,~]=obj.normalize_generator_backend_flag(obj.generator_backend_flag);
            st.generator_backend_flag=double(flagCode);
            st.generator_backend_last_code=double(obj.generator_backend_last_code);
            st.generator_backend_last_source_code=double(obj.generator_backend_last_source_code);
            st.generator_backend_last_callback_installed= ...
                logical(obj.generator_backend_last_callback_installed);
            st.generator_backend_locked_code=double(obj.generator_backend_locked_code);
            st.generator_backend_selection_locked=logical(obj.generator_backend_selection_locked);
            st.generator_backend_configuration_frozen= ...
                logical(obj.generator_backend_configuration_frozen);
            st.generator_integration_prime_done=logical(obj.generator_integration_prime_done);
            st.generator_integration_prime_ms=obj.generator_integration_prime_ms;
            st.generator_integration_prime_variants=obj.generator_integration_prime_variants;
            st.generator_context_detected_only=logical(obj.generator_context_detected_only);
            st.generator_backend_kernel_prime_done=logical(obj.generator_backend_kernel_prime_done);
            st.generator_backend_kernel_prime_ms=obj.generator_backend_kernel_prime_ms;
            st.generator_backend_kernel_prime_variants=obj.generator_backend_kernel_prime_variants;
            st.generator_shadow_proposal_count=obj.generator_shadow_proposal_count;
            st.bezier_shadow_generated_count=obj.bezier_shadow_generated_count;
            st.bezier_shadow_peak_safe_count=obj.bezier_shadow_peak_safe_count;
            st.bezier_shadow_last_reason_code=obj.bezier_shadow_last_reason_code;
            st.bezier_shadow_last_t0=obj.bezier_shadow_last_t0;
            st.bezier_shadow_last_tm=obj.bezier_shadow_last_tm;
            st.bezier_shadow_last_t1=obj.bezier_shadow_last_t1;
            st.bezier_shadow_last_obstacle_id=obj.bezier_shadow_last_obstacle_id;
            st.bezier_shadow_last_component_code=obj.bezier_shadow_last_component_code;
            st.bezier_shadow_last_displacement_m=obj.bezier_shadow_last_displacement_m;
            st.bezier_shadow_last_metric_before=obj.bezier_shadow_last_metric_before;
            st.bezier_shadow_last_metric_after=obj.bezier_shadow_last_metric_after;
            st.bezier_shadow_last_peak_min_metric=obj.bezier_shadow_last_peak_min_metric;
            st.bezier_shadow_last_peak_all_safe=logical(obj.bezier_shadow_last_peak_all_safe);
            st.bezier_shadow_last_c7_exact=logical(obj.bezier_shadow_last_c7_exact);
            st.bezier_shadow_last_d1_ub=obj.bezier_shadow_last_d1_ub;
            st.bezier_shadow_last_d2_ub=obj.bezier_shadow_last_d2_ub;
            st.bezier_shadow_last_d3_ub=obj.bezier_shadow_last_d3_ub;
            st.bezier_shadow_last_d4_ub=obj.bezier_shadow_last_d4_ub;
            st.bezier_shadow_last_d5_ub=obj.bezier_shadow_last_d5_ub;
            st.bezier_shadow_last_d6_ub=obj.bezier_shadow_last_d6_ub;
            st.bezier_shadow_last_dx=obj.bezier_shadow_last_dx;
            st.bezier_shadow_last_dy=obj.bezier_shadow_last_dy;
            st.bezier_shadow_last_dz=obj.bezier_shadow_last_dz;
            st.geometry_shadow_pass=logical(obj.geometry_shadow_pass);
            st.geometry_shadow_reason_code=obj.geometry_shadow_reason_code;
            st.geometry_shadow_continuous_time=logical(obj.geometry_shadow_continuous_time);
            st.geometry_shadow_ieee754_outward_closed=logical(obj.geometry_shadow_ieee754_outward_closed);
            st.geometry_shadow_min_margin_m=obj.geometry_shadow_min_margin_m;
            st.geometry_shadow_min_payload_margin_m=obj.geometry_shadow_min_payload_margin_m;
            st.geometry_shadow_min_uav_margin_m=obj.geometry_shadow_min_uav_margin_m;
            st.geometry_shadow_min_cable_margin_m=obj.geometry_shadow_min_cable_margin_m;
            st.geometry_shadow_leaf_count=obj.geometry_shadow_leaf_count;
            st.geometry_shadow_split_count=obj.geometry_shadow_split_count;
            st.geometry_shadow_max_depth=obj.geometry_shadow_max_depth;
            st.geometry_shadow_min_alpha_lb=obj.geometry_shadow_min_alpha_lb;
            st.geometry_shadow_max_qdot_ub=obj.geometry_shadow_max_qdot_ub;
            st.geometry_shadow_max_q_arc_radius=obj.geometry_shadow_max_q_arc_radius;
            st.geometry_shadow_max_q_arc_inflation_m=obj.geometry_shadow_max_q_arc_inflation_m;
            st.geometry_shadow_resource_exhausted=logical(obj.geometry_shadow_resource_exhausted);
            st.generator_last_elapsed_before_ms=obj.generator_last_elapsed_before_ms;
            st.generator_last_budget_remaining_ms=obj.generator_last_budget_remaining_ms;
            st.generator_last_context_ms=obj.generator_last_context_ms;
            st.generator_last_callback_ms=obj.generator_last_callback_ms;
            st.generator_last_validation_ms=obj.generator_last_validation_ms;
            st.generator_last_critical_ms=obj.generator_last_critical_ms;
            st.generator_last_cycle_critical_ms=obj.generator_last_cycle_critical_ms;

            % LOGGER-safe direct scalar timing mirrors.
            st.rt_light_e2e_ms=NaN;
            st.rt_spike_capture_time=NaN;
            st.rt_spike_total_ms=NaN;
            st.rt_spike_stage_front_ms=NaN;
            st.rt_spike_stage_loadmass_ms=NaN;
            st.rt_spike_stage_dependency_ms=NaN;
            st.rt_spike_stage_base_cert_geom_ms=NaN;
            st.rt_spike_stage_sensor_trigger_ms=NaN;
            st.rt_spike_stage_output_state_ms=NaN;
            st.rt_spike_base_ref_ms=NaN;
            st.rt_spike_flight_guard_ms=NaN;
            st.rt_spike_cert_guard_ms=NaN;
            st.rt_spike_active_ref_ms=NaN;
            st.rt_spike_geometry_ms=NaN;
            st.rt_spike_state_handle_ms=NaN;
            st.rt_spike_write_active_output_ms=NaN;
            st.rt_spike_state_basic_ms=NaN;
            st.rt_spike_state_trigger_scalar_ms=NaN;
            st.rt_spike_state_trigger_heavy_ms=NaN;
            st.rt_spike_state_cert_ms=NaN;
            st.rt_spike_state_meta_ms=NaN;
            st.rt_spike_active_count=NaN;
            st.rt_spike_need_replan=NaN;
            st.rt_live_binding_ms=NaN;
            st.rt_dependency_ms=NaN;
            st.rt_cert_guard_ms=NaN;
            st.rt_geometry_eval_ms=NaN;
            st.rt_replan_status_ms=NaN;
            st.rt_output_write_ms=NaN;
            st.rt_state_trigger_write_ms=NaN;
            st.rt_state_cert_write_ms=NaN;
            st.rt_state_meta_write_ms=NaN;
            st.rt_base_ref_ms=NaN;
            st.rt_sensor_ms=NaN;
            st.rt_reference_packet_ms=NaN;
            st.rt_kernel_ms=NaN;
            st.rt_trigger_ms=NaN;
            st.rt_total_ms=NaN;

        end


        function tf = observer_is_real_control_event(obj,cha)
            %#ok<INUSD>
            % Only actual a/t/f calls delivered to THIS reference object
            % advance the certifier. In the current allocation the replanner
            % is normally dispatched only in Flight, so SIM mode initializes
            % exactly at the observed Flight-entry event. Synthetic prime
            % calls are excluded.
            if obj.setup_exact_do_prime_active || obj.generator_integration_prime_active
                tf=false;
                return;
            end
            ph=char(string(cha));
            tf=any(strcmp(ph,{'a','t','f'}));
        end


        function observer_initialize_lazy(obj,t,phase)
            % Lazy initialization intentionally occurs after MOTIVE and EKF
            % have already executed for a real control event.
            I=struct();
            I.initialized=false;
            I.selector_verified=false;
            I.measured_scalar_indices=[];
            I.previous_measurement=struct();
            I.observer_support=struct();
            I.hlc_state_mass_bridge=struct();
            I.last_event_time=NaN;
            I.last_phase='';
            I.phase_transition_count=0;
            I.update_count=0;
            I.fail_count=0;
            I.runtime_last_ms=NaN;
            I.runtime_max_ms=NaN;
            I.runtime_sum_ms=0;
            I.runtime_count=0;

            [selectorOK,expected]=obj.observer_validate_measurement_selector();
            assert(selectorOK, ...
                '[REPLANNER OBSERVER ERROR] EKF selector is not certified p,q,pL,pT order.');
            I.selector_verified=true;
            I.measured_scalar_indices=expected;

            mode=upper(strtrim(char(obj.OBSERVER_HARD_SUPPORT_MODE)));
            if strcmp(mode,'SIM_NUMERIC_TRUTH')
                st=obj.self.plant.state;
                wc=double(st.w(:));
                vc=double(st.vL(:));
                lc=double(st.wL(:));
                qe=double(st.q(:));
                pT=double(st.pT(:));
                assert(numel(wc)==3&&numel(vc)==3&&numel(lc)==3&& ...
                    numel(qe)==3&&numel(pT)==3&& ...
                    all(isfinite([wc;vc;lc;qe;pT])), ...
                    '[REPLANNER OBSERVER ERROR] SIM initial hidden state invalid.');
                pn=norm(pT);
                assert(abs(pn-1)<=16*eps(max(1,pn)), ...
                    '[REPLANNER OBSERVER ERROR] SIM pT is not unit.');
                mL0=double(obj.OBSERVER_SIM_TRUE_ML);
                assert(isscalar(mL0)&&isfinite(mL0)&&mL0>=0, ...
                    ['[REPLANNER OBSERVER ERROR] SIM true payload mass authority is missing. ' ...
                     'Pass opts.OBSERVER_SIM_TRUE_ML from the value used BEFORE plant construction.']);

                I.hidden=struct( ...
                    'w_center',wc,'w_radius',0,'w_norm_ub',norm(wc), ...
                    'vL_center',vc,'vL_radius',0,'vL_norm_ub',norm(vc), ...
                    'wL_center',lc,'wL_radius',0,'wL_norm_ub',norm(lc));
                I.true_mL_interval=[mL0 mL0];
                I.q_abs_ub_current=abs(qe);
                I.pT_norm_ub=1;

            elseif strcmp(mode,'REAL_HARD_BOUNDS')
                B0=obj.OBSERVER_REAL_INITIAL_SUPPORT;
                req={'w_center','w_radius','vL_center','vL_radius', ...
                    'wL_center','wL_radius','mL_interval','q_abs_ub','pT_norm_ub'};
                assert(isstruct(B0), ...
                    '[REPLANNER OBSERVER ERROR] REAL initial support must be a struct.');
                for kk=1:numel(req)
                    assert(isfield(B0,req{kk}), ...
                        '[REPLANNER OBSERVER ERROR] REAL initial support missing %s.',req{kk});
                end
                wc=double(B0.w_center(:)); vc=double(B0.vL_center(:));
                lc=double(B0.wL_center(:));
                wr=double(B0.w_radius); vr=double(B0.vL_radius); lr=double(B0.wL_radius);
                mi=double(B0.mL_interval(:)).'; qa=double(B0.q_abs_ub(:));
                pn=double(B0.pT_norm_ub);
                assert(numel(wc)==3&&numel(vc)==3&&numel(lc)==3&& ...
                    all(isfinite([wc;vc;lc;wr;vr;lr]))&&all([wr;vr;lr]>=0), ...
                    '[REPLANNER OBSERVER ERROR] REAL hidden initial support invalid.');
                assert(numel(mi)==2&&all(isfinite(mi))&&mi(1)>=0&&mi(2)>=mi(1), ...
                    '[REPLANNER OBSERVER ERROR] REAL mL interval invalid.');
                assert(numel(qa)==3&&all(isfinite(qa))&&all(qa>=0)&& ...
                    isscalar(pn)&&isfinite(pn)&&pn>0, ...
                    '[REPLANNER OBSERVER ERROR] REAL q/pT support invalid.');
                nu=double(obj.OBSERVER_REAL_NU_ABS_UB(:));
                assert(numel(nu)==12&&all(isfinite(nu))&&all(nu>=0), ...
                    '[REPLANNER OBSERVER ERROR] REAL 12-D measurement hard bounds invalid.');
                assert(~isempty(strtrim(char(obj.OBSERVER_REAL_SOURCE_TAG))), ...
                    '[REPLANNER OBSERVER ERROR] REAL hard-bound source tag required.');

                I.hidden=struct( ...
                    'w_center',wc,'w_radius',wr,'w_norm_ub',norm(wc)+wr, ...
                    'vL_center',vc,'vL_radius',vr,'vL_norm_ub',norm(vc)+vr, ...
                    'wL_center',lc,'wL_radius',lr,'wL_norm_ub',norm(lc)+lr);
                I.true_mL_interval=mi;
                I.q_abs_ub_current=qa;
                I.pT_norm_ub=pn;
            else
                error('[REPLANNER OBSERVER ERROR] unsupported mode: %s',mode);
            end

            I.initialized=true;
            I.initialized_at_time=double(t);
            I.initialized_at_phase=char(phase);
            obj.observer_hard_support_internal=I;
        end


        function observer_update_same_event(obj,time,cha)
            t=double(time.t);
            phase=char(string(cha));

            I=obj.observer_hard_support_internal;
            if isempty(I)||~isstruct(I)||~isfield(I,'initialized')||~I.initialized
                obj.observer_initialize_lazy(t,phase);
                I=obj.observer_hard_support_internal;
            end

            if isfinite(I.last_event_time)
                if t==I.last_event_time
                    return;
                end
                assert(t>I.last_event_time, ...
                    '[REPLANNER OBSERVER ERROR] control-event time moved backwards.');
            end

            if ~isempty(I.last_phase)&&~strcmp(I.last_phase,phase)
                I.phase_transition_count=I.phase_transition_count+1;
            end

            C=obj.observer_build_measurement_event(t,phase,I.selector_verified);
            if ~C.closed
                I.fail_count=I.fail_count+1;
                obj.observer_hard_support_internal=I;
                error('[REPLANNER OBSERVER ERROR] measurement hard box failed: %s',C.reason);
            end

            xhat=obj.self.estimator.result.state;
            hasPrev=isstruct(I.previous_measurement)&& ...
                isfield(I.previous_measurement,'closed')&&logical(I.previous_measurement.closed);

            if hasPrev
                h=t-double(I.previous_measurement.event_time);
                assert(isfinite(h)&&h>0, ...
                    '[REPLANNER OBSERVER ERROR] consecutive event interval is not positive.');
                uPrev=obj.observer_get_previous_held_input();
                F=obj.observer_one_step_support(h,uPrev,I);
                assert(F.closed, ...
                    '[REPLANNER OBSERVER ERROR] continuous hidden-state support left certified chart.');
                K=obj.observer_contract_two_events(I.previous_measurement,C,F,xhat);

                I.hidden.w_center=K.w.center;
                I.hidden.w_radius=K.w.radius;
                I.hidden.w_norm_ub=K.w.true_norm_ub;
                I.hidden.vL_center=K.vL.center;
                I.hidden.vL_radius=K.vL.radius;
                I.hidden.vL_norm_ub=K.vL.true_norm_ub;
                I.hidden.wL_center=K.wL.center;
                I.hidden.wL_radius=K.wL.radius;
                I.hidden.wL_norm_ub=K.wL.true_norm_ub;
                measured=K.measured_error;
                ew=K.w.observer_error_norm_ub;
                evL=K.vL.observer_error_norm_ub;
                ewL=K.wL.observer_error_norm_ub;
                contractionUsed=true;
                I.last_forward_support=F;
            else
                measured=obj.observer_measured_error_box(xhat,C);
                ew=norm(double(xhat.w(:))-I.hidden.w_center)+I.hidden.w_radius;
                evL=norm(double(xhat.vL(:))-I.hidden.vL_center)+I.hidden.vL_radius;
                ewL=norm(double(xhat.wL(:))-I.hidden.wL_center)+I.hidden.wL_radius;
                contractionUsed=false;
                I.last_forward_support=struct();
            end

            mi=I.true_mL_interval;
            hm=double(xhat.mL);
            emL=max(abs(hm-mi(1)),abs(hm-mi(2)));

            B=struct();
            B.schema='REPLANNING_SYSTEM_OBSERVER_HARD_SUPPORT_EVENT_V2';
            B.closed=true;
            B.event_time=t;
            B.phase=phase;
            B.protection_eligible=strcmp(phase,'f');
            B.measured=measured;
            B.w_error_norm_ub=ew;
            B.vL_error_norm_ub=evL;
            B.wL_error_norm_ub=ewL;
            B.mL_error_abs_ub=emL;
            B.true_mL_interval=mi;
            B.hidden_true_support=I.hidden;
            B.consecutive_measurement_contraction_used=contractionUsed;
            B.numeric_plant_to_continuous_authority=false;
            B.realized_error_compared=false;
            B.Q_promoted=false;
            B.R_promoted=false;
            B.covariance_promoted=false;
            B.sampled_acceptance=false;

            I.q_abs_ub_current=abs(C.y(4:6))+C.nu_abs_ub(4:6);
            I.previous_measurement=C;
            I.observer_support=B;
            I.hlc_state_mass_bridge=obj.observer_build_state_mass_bridge(B);
            I.last_event_time=t;
            I.last_phase=phase;
            I.update_count=I.update_count+1;
            obj.observer_hard_support_internal=I;
        end


        function C = observer_build_measurement_event(obj,t,phase,selectorVerified)
            C=struct('schema','REPLANNING_SYSTEM_OBSERVER_MEASUREMENT_EVENT_V2', ...
                'closed',false,'event_time',t,'phase',phase,'y',[], ...
                'nu_abs_ub',[],'reason','UNINITIALIZED');
            assert(selectorVerified, ...
                '[REPLANNER OBSERVER ERROR] measurement selector not verified.');

            y=double(obj.self.sensor.result.output(:));
            if numel(y)~=12||any(~isfinite(y))
                C.reason='MEASUREMENT_NOT_FINITE_12D';
                return;
            end

            mode=upper(strtrim(char(obj.OBSERVER_HARD_SUPPORT_MODE)));
            if strcmp(mode,'SIM_NUMERIC_TRUTH')
                st=obj.self.plant.state;
                x=[double(st.p(:));double(st.q(:));double(st.pL(:));double(st.pT(:))];
                if numel(x)~=12||any(~isfinite(x))
                    C.reason='SIM_NUMERIC_TRUTH_NOT_FINITE_12D';
                    return;
                end
                ur=eps(1)/2;
                d=double(y-x);
                rho=(ur/(1-ur))*(abs(y)+abs(x));
                raw=abs(d)+rho;
                nu=zeros(12,1);
                for ii=1:12
                    nu(ii)=obj.observer_nextfloat_up(raw(ii));
                end
                C.y=y; C.nu_abs_ub=nu; C.raw_difference=d;
                C.relative_to='CURRENT_NUMERICAL_PLANT_STATE';
                C.numeric_plant_to_continuous_authority=false;
                C.closed=all(isfinite(nu))&&all(nu>=0);
                C.reason='SIM_SAME_EVENT_HARD_BOX_CLOSED';
            elseif strcmp(mode,'REAL_HARD_BOUNDS')
                nu=double(obj.OBSERVER_REAL_NU_ABS_UB(:));
                tag=strtrim(char(obj.OBSERVER_REAL_SOURCE_TAG));
                if numel(nu)~=12||any(~isfinite(nu))||any(nu<0)||isempty(tag)
                    C.reason='REAL_HARD_BOUND_AUTHORITY_INVALID';
                    return;
                end
                C.y=y; C.nu_abs_ub=nu; C.source_tag=tag;
                C.relative_to='PHYSICAL_TRUE_MEASURED_COORDINATES';
                C.numeric_plant_to_continuous_authority=false;
                C.closed=true;
                C.reason='REAL_HARD_BOX_CLOSED';
            else
                C.reason='UNSUPPORTED_OBSERVER_MODE';
            end
            C.Q_promoted=false; C.R_promoted=false;
            C.covariance_promoted=false; C.logged_max_promoted=false;
            C.sampled_acceptance=false; C.realized_error_compared=false;
        end


        function [ok,expected] = observer_validate_measurement_selector(obj)
            ok=false; expected=[];
            ekf=obj.self.estimator.ekf;
            Csel=double(ekf.output_param);
            stlist=string(ekf.model.state.list(:)).';
            nlist=double(ekf.model.state.num_list(:)).';
            IDX=struct(); p0=1;
            for ii=1:numel(stlist)
                n=nlist(ii); IDX.(char(stlist(ii)))=p0:(p0+n-1); p0=p0+n;
            end
            req={'p','q','pL','pT'};
            if ~all(isfield(IDX,req)),return;end
            expected=[IDX.p IDX.q IDX.pL IDX.pT];
            ok=size(Csel,1)==12&&size(Csel,2)==sum(nlist)&& ...
                all(sum(Csel~=0,2)==1)&&all(Csel(:)==0|Csel(:)==1);
            if ok
                [rr,cc]=find(Csel==1); [rr,ord]=sort(rr); cc=cc(ord);
                ok=isequal(rr(:).',(1:12))&&isequal(cc(:).',expected);
            end
        end


        function M = observer_measured_error_box(~,xhat,C)
            y=double(C.y(:)); nu=double(C.nu_abs_ub(:)); M=struct();
            M.p=struct('center',double(xhat.p(:))-y(1:3),'radius',nu(1:3),'abs_ub',abs(double(xhat.p(:))-y(1:3))+nu(1:3));
            M.q=struct('center',double(xhat.q(:))-y(4:6),'radius',nu(4:6),'abs_ub',abs(double(xhat.q(:))-y(4:6))+nu(4:6));
            M.pL=struct('center',double(xhat.pL(:))-y(7:9),'radius',nu(7:9),'abs_ub',abs(double(xhat.pL(:))-y(7:9))+nu(7:9));
            M.pT=struct('center',double(xhat.pT(:))-y(10:12),'radius',nu(10:12),'abs_ub',abs(double(xhat.pT(:))-y(10:12))+nu(10:12));
        end


        function u = observer_get_previous_held_input(obj)
            u=double(obj.self.controller.result.input(:));
            assert(numel(u)==4&&all(isfinite(u)), ...
                '[REPLANNER OBSERVER ERROR] previous held controller input invalid.');
        end


        function S = observer_one_step_support(obj,h,u,I)
            H=I.hidden;
            Jd=double(obj.self.parameter.get(["jx","jy","jz"])); Jd=Jd(:);
            m=double(obj.self.parameter.get("mass"));
            g=double(obj.self.parameter.get("gravity"));
            L=double(obj.self.parameter.get("cableL"));
            assert(numel(Jd)==3&&all(isfinite(Jd))&&all(Jd>0)&& ...
                isfinite(m)&&m>0&&isfinite(g)&&g>=0&&isfinite(L)&&L>0, ...
                '[REPLANNER OBSERVER ERROR] physical parameter domain invalid.');

            Jmin=min(Jd); Jmax=max(Jd);
            tauNorm=norm(u(2:4)); u1abs=abs(u(1));
            W0=double(H.w_norm_ub); WL0=double(H.wL_norm_ub);
            VL0=double(H.vL_norm_ub); q0=double(I.q_abs_ub_current(:));
            Pnorm=double(I.pT_norm_ub); mLint=double(I.true_mL_interval(:));
            mLlb=mLint(1);

            V0=0.5*Jmax*W0^2;
            sqrtVend=sqrt(V0)+tauNorm*h/sqrt(2*Jmin);
            VwEnd=sqrtVend^2;
            Wabs0=sqrt(2*V0/Jmin); WabsEnd=sqrt(2*VwEnd/Jmin);
            Iw=Wabs0*h+tauNorm*h^2/(2*Jmin);

            Theta=q0(2)+Iw;
            pitchClosed=Theta<pi/2;
            qAbs=[Inf;Theta;Inf]; fullClosed=false;
            if pitchClosed
                qAbs(1)=q0(1)+(1+tan(Theta))*Iw;
                qAbs(3)=q0(3)+(1/cos(Theta))*Iw;
                fullClosed=qAbs(1)<pi&&qAbs(3)<pi;
            end

            aWL=Pnorm*u1abs/(m*L);
            WLEnd=WL0+aWL*h;
            IWL=WL0*h+0.5*aWL*h^2;
            pTSpan=Pnorm*IWL;
            den=m+mLlb;
            assert(den>0,'[REPLANNER OBSERVER ERROR] total mass lower bound invalid.');
            Aconst=g+(Pnorm^2*u1abs)/den;
            Aquad=(Pnorm^3)*m*L/den;
            intWL2=WL0^2*h+WL0*aWL*h^2+(aWL^2*h^3)/3;
            VLEnd=VL0+Aconst*h+Aquad*intWL2;
            ALmax=Aconst+Aquad*(WLEnd^2);

            S=struct();
            S.closed=pitchClosed&&fullClosed; S.h=h; S.held_input=u;
            S.Jdiag=Jd; S.w_norm_ub_end=WabsEnd;
            S.q_abs_ub_interval=qAbs;
            S.wL_accel_norm_ub=aWL; S.wL_norm_ub_end=WLEnd;
            S.vL_accel_norm_ub=ALmax; S.vL_norm_ub_end=VLEnd; S.pT_norm_ub=Pnorm;
            S.sampled_acceptance=false; S.guessed_bound=false;
        end


        function O = observer_contract_two_events(obj,C0,C1,F,xhat)
            y0=double(C0.y(:)); y1=double(C1.y(:));
            n0=double(C0.nu_abs_ub(:)); n1=double(C1.nu_abs_ub(:));
            h=double(C1.event_time-C0.event_time);
            assert(h>0&&isfinite(h),'[REPLANNER OBSERVER ERROR] contraction h invalid.');
            iq=4:6; ipL=7:9; ipT=10:12;
            MEAS=obj.observer_measured_error_box(xhat,C1);

            VLF=double(F.vL_norm_ub_end);
            ALmax=double(F.vL_accel_norm_ub);
            dPL=(y1(ipL)-y0(ipL))/h;
            rVL=0.5*ALmax*h+(norm(n0(ipL))+norm(n1(ipL)))/h;
            cVL=dPL;
            VLtrue=min(VLF,norm(cVL)+rVL);
            eVL=min(norm(double(xhat.vL(:)))+VLF, ...
                norm(double(xhat.vL(:))-cVL)+rVL);

            WLF=double(F.wL_norm_ub_end); Pbar=double(F.pT_norm_ub);
            aWL=double(F.wL_accel_norm_ub);
            pTddot=Pbar*(aWL+WLF^2);
            dPT=(y1(ipT)-y0(ipT))/h;
            rPTdot=0.5*pTddot*h+(norm(n0(ipT))+norm(n1(ipT)))/h;
            cWL=cross(y1(ipT),dPT);
            pTdotUb=Pbar*WLF;
            rWL=norm(n1(ipT))*pTdotUb+norm(y1(ipT))*rPTdot;
            WLtrue=min(WLF,norm(cWL)+rWL);
            eWL=min(norm(double(xhat.wL(:)))+WLF, ...
                norm(double(xhat.wL(:))-cWL)+rWL);

            Wf=double(F.w_norm_ub_end); Jd=double(F.Jdiag(:));
            Jmin=min(Jd); Jmax=max(Jd); tauNorm=norm(double(F.held_input(2:4)));
            Theta=double(F.q_abs_ub_interval(2));
            assert(isfinite(Theta)&&Theta<pi/2, ...
                '[REPLANNER OBSERVER ERROR] Euler chart is not closed.');
            tanT=tan(Theta); secT=1/cos(Theta); Kphi=1+tanT;
            Aomega=tauNorm/Jmin+(Jmax/Jmin)*Wf^2;
            adot=Aomega+Kphi*Wf^2;
            qddotUb=norm([Aomega+secT^2*Wf^2+tanT*adot; ...
                Aomega+Kphi*Wf^2; secT*(tanT*Wf^2+adot)]);
            dq=(y1(iq)-y0(iq))/h;
            rQdot=0.5*qddotUb*h+(norm(n0(iq))+norm(n1(iq)))/h;
            lim=[pi;pi/2;pi];
            assert(all(abs(y0(iq))+n0(iq)<lim)&&all(abs(y1(iq))+n1(iq)<lim), ...
                '[REPLANNER OBSERVER ERROR] measured Euler box reaches chart boundary.');
            Ey=obj.observer_euler_rate_inverse_ZYX(y1(iq));
            cW=Ey*dq;
            dphi=n1(4); dtheta=n1(5);
            LE=sqrt(dtheta^2+2*dphi^2+2*(dphi+dtheta)^2);
            Kq=sqrt(Kphi^2+1+secT^2); qdotUb=Kq*Wf;
            rW=LE*qdotUb+norm(Ey,2)*rQdot;
            Wtrue=min(Wf,norm(cW)+rW);
            eW=min(norm(double(xhat.w(:)))+Wf,norm(double(xhat.w(:))-cW)+rW);

            O=struct(); O.closed=true; O.h=h; O.measured_error=MEAS;
            O.vL=struct('center',cVL,'radius',rVL,'true_norm_ub',VLtrue, ...
                'observer_error_norm_ub',eVL);
            O.wL=struct('center',cWL,'radius',rWL,'true_norm_ub',WLtrue, ...
                'observer_error_norm_ub',eWL);
            O.w=struct('center',cW,'radius',rW,'true_norm_ub',Wtrue, ...
                'observer_error_norm_ub',eW);
        end


        function E = observer_euler_rate_inverse_ZYX(~,q)
            phi=q(1); theta=q(2);
            E=[1,0,-sin(theta);0,cos(phi),sin(phi)*cos(theta); ...
                0,-sin(phi),cos(phi)*cos(theta)];
        end


        function R = observer_build_state_mass_bridge(~,B)
            M=B.measured;
            eq=double(M.q.abs_ub(:));
            thetaRelUb=min(pi,sum(eq));
            R=struct();
            R.schema='REPLANNING_SYSTEM_OBSERVER_TO_HLC_STATE_MASS_BRIDGE_V2';
            R.closed=true;
            R.event_time=B.event_time;
            R.phase=B.phase;
            R.model_state=struct( ...
                'euler_abs_ub',eq, ...
                'compact_quaternion_chord_norm_ub',2*sin(thetaRelUb/4), ...
                'w_norm_ub',double(B.w_error_norm_ub), ...
                'pL_abs_ub',double(M.pL.abs_ub(:)), ...
                'vL_norm_ub',double(B.vL_error_norm_ub), ...
                'pT_abs_ub',double(M.pT.abs_ub(:)), ...
                'wL_norm_ub',double(B.wL_error_norm_ub));
            R.live_mL=struct('error_abs_ub',double(B.mL_error_abs_ub), ...
                'true_interval',double(B.true_mL_interval(:)).');
            % Current controller command has not executed yet at reference stage.
            R.current_saturation_closed=false;
            R.total_virtual_disturbance_closed=false;
            R.phi_fixed_point_closed=false;
            R.realized_error_compared=false;
        end


        function y = observer_nextfloat_up(~,x)
            x=double(x);
            if isnan(x)||x==Inf,y=x;return;end
            assert(x>=0,'[REPLANNER OBSERVER ERROR] nextfloat input must be nonnegative.');
            if x==0,y=realmin*eps(1);return;end
            u=typecast(x,'uint64'); u=u+uint64(1); y=typecast(u,'double');
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

        function [te,ok] = try_resolve_takeoff_end_time_nonfatal(obj)
            % Constructor-safe takeoff-end lookup.
            %
            % The legacy run sequence may set obj.takeoff_end_s AFTER the
            % REPLANNING_SYSTEM constructor returns.  Therefore absence of
            % reference.takeoff during construction is not an error.
            %
            % No guessed/default time is introduced.
            te=NaN;
            ok=false;

            if isnumeric(obj.takeoff_end_s) && isscalar(obj.takeoff_end_s) && ...
                    isfinite(obj.takeoff_end_s) && obj.takeoff_end_s>0
                te=double(obj.takeoff_end_s);
                ok=true;
                return;
            end

            % Optional legacy source: agent.reference.takeoff.
            try
                if ~obj.has_prop_or_field(obj.self,'reference')
                    return;
                end
                R=obj.self.reference;
                if ~obj.has_prop_or_field(R,'takeoff')
                    return;
                end
                ref=R.takeoff;
            catch
                return;
            end

            candidates={'te','t_end','tend','duration','T'};

            for k=1:numel(candidates)
                nm=candidates{k};
                try
                    if isobject(ref) && isprop(ref,nm)
                        v=ref.(nm);
                    elseif isstruct(ref) && isfield(ref,nm)
                        v=ref.(nm);
                    else
                        continue;
                    end

                    if isnumeric(v) && isscalar(v) && isfinite(v) && v>0
                        te=double(v);
                        ok=true;
                        obj.takeoff_end_s=te;
                        return;
                    end
                catch
                end
            end

            for holder={'param','parameter','params','option','opts'}
                hn=holder{1};
                try
                    if ~obj.has_prop_or_field(ref,hn)
                        continue;
                    end
                    H=ref.(hn);

                    for k=1:numel(candidates)
                        nm=candidates{k};
                        if isstruct(H) && isfield(H,nm)
                            v=H.(nm);
                            if isnumeric(v) && isscalar(v) && ...
                                    isfinite(v) && v>0
                                te=double(v);
                                ok=true;
                                obj.takeoff_end_s=te;
                                return;
                            end
                        end
                    end
                catch
                end
            end
        end


        function te = resolve_takeoff_end_time(obj)
            [te,ok]=obj.try_resolve_takeoff_end_time_nonfatal();

            assert(ok && isfinite(te) && te>0, ...
                ['[REPLANNER ERROR] Flight reference time origin is still unresolved. ' ...
                 'The existing run must bind replanner_instance.takeoff_end_s ' ...
                 '(or opts.takeoff_end_s) before nominal-table generation. ' ...
                 'reference.takeoff is optional and is not required by construction.']);

            obj.takeoff_end_s=double(te);
            obj.flight_reference_time_origin=double(te);
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

            % Global table index starts at actual Flight start.  The table
            % values themselves are evaluated at tau=t_global-t0.
            t0=obj.resolve_takeoff_end_time();
            obj.flight_reference_time_origin=double(t0);
            tend=te+obj.horizon_s;
            if tend<t0
                return;
            end

            Ntab=round((tend-t0)/obj.prediction_dt)+1;
            assert(Ntab>=2,'[REPLANNER ERROR] invalid nominal table size.');

            pLtab=zeros(3,Ntab);
            qtab=zeros(3,Ntab);

            % Evaluate the first point once to determine the exact xd width.
            % The values come from the same base_ref.do() path used previously.
            tb=tic;
            [pL1,q1,xd1]=obj.evaluate_nominal_payload_and_q( ...
                t0,'f',varargin_in,[]);
            xd1=xd1(:);
            D=numel(xd1);
            assert(D>=11 && all(isfinite(xd1)), ...
                '[REPLANNER ERROR] nominal full-state table received invalid xd.');

            xdtab=zeros(D,Ntab);
            pLtab(:,1)=pL1;
            qtab(:,1)=q1;
            xdtab(:,1)=xd1;

            for j=2:Ntab
                tq=t0+(j-1)*obj.prediction_dt;
                [pLj,qj,xdj]=obj.evaluate_nominal_payload_and_q( ...
                    tq,'f',varargin_in,[]);
                xdj=xdj(:);
                assert(numel(xdj)==D && all(isfinite(xdj)), ...
                    '[REPLANNER ERROR] nominal xd dimension/value changed during table build.');
                pLtab(:,j)=pLj;
                qtab(:,j)=qj;
                xdtab(:,j)=xdj;
            end

            obj.base_ref.do(varargin_in{:});

            obj.nominal_table_t0=t0;
            obj.nominal_table_tend=t0+(Ntab-1)*obj.prediction_dt;
            obj.nominal_table_dt=obj.prediction_dt;
            obj.nominal_table_pL=pLtab;
            obj.nominal_table_q=qtab;
            obj.nominal_table_xd=xdtab;
            obj.nominal_table_eval_count=Ntab;
            obj.nominal_table_build_ms=1e3*toc(tb);
            obj.nominal_table_valid=true;
            obj.profiling.t_nominal_table_build_ms=obj.nominal_table_build_ms;

            if obj.ENABLE_ALL_LOG
                fprintf(['[REPLANNING_SYSTEM] nominal table ready | global %.3f--%.3f s | ' ...
                    'reference tau %.3f--%.3f s | points=%d | %.3f ms\n'], ...
                    obj.nominal_table_t0,obj.nominal_table_tend, ...
                    0.0,obj.nominal_table_tend-obj.flight_reference_time_origin, ...
                    Ntab,obj.nominal_table_build_ms);
            end
        end


        function xd = get_nominal_xd_from_table(obj,t_now)
            % Exact O(1) current-reference lookup on the same dt grid used by
            % build_nominal_reference_table(). No interpolation is permitted.
            assert(obj.nominal_table_valid && ~isempty(obj.nominal_table_xd), ...
                '[REPLANNER ERROR] nominal full-state table unavailable in Flight.');
            assert(abs(obj.nominal_table_dt-obj.prediction_dt)<=1e-12, ...
                '[REPLANNER ERROR] nominal full-state table dt is stale.');

            idx=round((double(t_now)-obj.nominal_table_t0)/obj.nominal_table_dt)+1;
            assert(idx>=1 && idx<=size(obj.nominal_table_xd,2), ...
                '[REPLANNER ERROR] current Flight time lies outside nominal full-state table.');

            tq=obj.nominal_table_t0+(idx-1)*obj.nominal_table_dt;
            tol=1e-10*max(1,abs(double(t_now)));
            assert(abs(tq-double(t_now))<=tol, ...
                '[REPLANNER ERROR] current Flight time is not on the certified nominal dt grid.');

            xd=obj.nominal_table_xd(:,idx);
            assert(numel(xd)>=11 && all(isfinite(xd)), ...
                '[REPLANNER ERROR] nominal full-state table returned invalid xd.');
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

            [pQ_cur,pL_cur,q_cur,cable_residual_cur]= ...
                obj.get_current_physical_geometry(xd_now);
            [~,~,~,all_dists]= ...
                obj.raw_sensor_gate(pQ_cur,obj.static_obstacle_map);

            % Freeze selected callback before Flight.
            obj.freeze_generator_backend_configuration();

            % Prime all detected-obstacle cardinalities: 0,1,...,M.
            masks=false(M+1,M);
            for nactive=1:M
                masks(nactive+1,1:nactive)=true;
            end
            obj.prime_generator_integration_variants( ...
                tf,xd_now,xd_now,pQ_cur,pL_cur,q_cur,cable_residual_cur, ...
                all_dists,pL_ref,q_ref,'NOMINAL_TABLE',masks);

            % Prime the real selected backend kernel as JIT/first-use prep.
            % Outputs are discarded; no runtime decision/commit is made.
            obj.prime_locked_backend_kernel_variants( ...
                tf,xd_now,xd_now,pQ_cur,pL_cur,q_cur,cable_residual_cur, ...
                all_dists,pL_ref,q_ref,'NOMINAL_TABLE',masks);

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

            % =========================================================
            % Exact Flight do() prime.
            %
            % Stage 1: sequential real-time-age prime
            %   tf, tf+dt, tf+2dt, ...
            % This reproduces the first protected cycles with changing table
            % indices before Flight. It moves sequence-dependent MATLAB/JIT/
            % memory first-use effects out of the runtime control cycle.
            %
            % Stage 2: preserve existing forced-hit/no-hit downstream branch
            % coverage at tf.
            % =========================================================
            exact_args=varargin_in;
            exact_args{2}='f';

            replan_request_active_saved=obj.replan_request_active;
            last_replan_status_saved=obj.last_replan_status;

            obj.setup_exact_do_prime_active=true;
            obj.setup_exact_force_hit=false;
            obj.setup_exact_force_hit_calls=0;

            nseq=round(obj.SEQUENTIAL_FLIGHT_PRIME_CALLS);
            nbranch=round(obj.SETUP_EXACT_DO_PRIME_CALLS);

            te=tic;
            try
                % Stage 1: genuine no-forced-hit protected sequence.
                for k=1:nseq
                    tq=tf+(k-1)*obj.prediction_dt;
                    exact_args{1}=obj.make_time_query(varargin_in{1},tq);
                    obj.setup_exact_force_hit=false;
                    obj.do(exact_args{:});
                end

                % Stage 2: existing branch-coverage prime.
                exact_args{1}=obj.make_time_query(varargin_in{1},tf);
                for k=1:nbranch
                    obj.setup_exact_force_hit=mod(k,2)==0;
                    if obj.setup_exact_force_hit
                        obj.setup_exact_force_hit_calls= ...
                            obj.setup_exact_force_hit_calls+1;
                    end
                    obj.do(exact_args{:});
                end

                obj.setup_exact_force_hit=false;
                obj.setup_exact_do_prime_runtime_ms=1e3*toc(te);
                obj.setup_exact_do_prime_calls=nseq+nbranch;
                obj.setup_exact_do_prime_done=true;
                obj.setup_exact_do_prime_active=false;

                % Warmup is not allowed to leave request-edge state behind.
                obj.replan_request_active=replan_request_active_saved;
                obj.last_replan_status=last_replan_status_saved;
            catch ME
                obj.setup_exact_force_hit=false;
                obj.setup_exact_do_prime_active=false;
                obj.setup_exact_do_prime_done=false;
                obj.replan_request_active=replan_request_active_saved;
                obj.last_replan_status=last_replan_status_saved;
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
                    'variants=%d | hitCases=%d | sequentialDo=%d | branchDo=%d | ' ...
                    'totalExactDo=%d | forcedHit=%d | generatorPrime=%.3f ms (%d variants) | ' ...
                    'backendPrime=%.3f ms (%d variants) | %.3f ms\n'], ...
                    obj.online_prime_variant_count,obj.online_prime_hit_cases, ...
                    round(obj.SEQUENTIAL_FLIGHT_PRIME_CALLS), ...
                    round(obj.SETUP_EXACT_DO_PRIME_CALLS), ...
                    obj.setup_exact_do_prime_calls,obj.setup_exact_force_hit_calls, ...
                    obj.generator_integration_prime_ms,obj.generator_integration_prime_variants, ...
                    obj.generator_backend_kernel_prime_ms,obj.generator_backend_kernel_prime_variants, ...
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
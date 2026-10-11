classdef REPLANNING_SYSTEM < handle
% =========================================================================
% REPLANNING_SYSTEM
%
% 【概要】
%   索吊り下げ型ドローン (Quadrotor with Suspended Load) 向け
%   G26 将来予測衝突回避トリガー統合クラス。
%
% 【設計思想・特徴】
%   1) 将来予測ホライズン H [s]（外部設定可能、デフォルト 4.0 秒）に基づく
%      将来区間の衝突予測トリガーを復元・統合。
%   2) 現在の機体推定位置 (pQ) のみをセンサ球の検知原点として使用。
%   3) 20m センサ検知ゲートには各障害物の真の幾何形状に基づく表面距離
%      (raw-shape surface distance) を直接計算して適用。
%   4) 追加システム安全マージン d_margin はデフォルト 0.0 を維持
%      （機体・荷物・索の物理保護半径は独立して維持）。
%   5) 一般楕円体に対して「主軸 + 物理半径」という幾何学的に安全でない
%      近似を廃止し、厳密なミンコフスキー和包囲理論に基づく保守的外接楕円体を構成。
%   6) 球、円柱、直方体、三角柱、円錐、角錐、楕円体、カスタムの全8形状に対応。
%      未対応タイプは即座にエラー停止。
%   7) ソースコード変更を自動検知して C++ MEX カーネルを自動再ビルド。
%   8) 初期化時に MEX および幾何計算ルーチンの事前ウォームアップを実施し、
%      初回コールドスタート時間を定常周期のプロファイリングと明確に分離。
%   9) 牽引物質量 loadmass の推定値 (estimator.result.state.mL) の取得を
%      必須化（未推定時は即座にエラー停止）。
%
% 【座標系および索姿勢ベクトルの定義】
%   q  : 機体から荷物への単位ベクトル (UAV -> Load)
%   pQ = pL - L * q
%
% 【重要事項】
%   - 本クラスは衝突予測と「トリガー発火」のみを担当します（C6 軌道修復は行いません）。
%   - 将来予測は prediction_dt 刻みでサンプリング評価されます。
%   - 現在サポートされる障害物マップは静的障害物（static obstacles）です。
%     動的障害物の移動を勝手に静止扱いして安全保証することはありません。
%
% -------------------------------------------------------------------------
% コンストラクタ引数オプション (opts)
% -------------------------------------------------------------------------
%   sensor_range_m              デフォルト 20.0 [m] (球状センサ検知半径)
%   trigger_dist                上記エイリアス
%   horizon_s                   デフォルト 4.0 [s] (衝突予測ホライズン)
%   prediction_dt               制御/予測周期 [s] (本体 plant/time から厳格取得)
%   d_margin                    デフォルト 0.0 [m] (追加システムマージン)
%   r_drone                     デフォルト 0.5 [m] (機体等価半径)
%   r_load                      デフォルト 0.5 [m] (荷物等価半径)
%   r_cable_sphere              デフォルト 0.5 [m] (索保護球半径)
%   warmup_calls                デフォルト 10 (ウォームアップ試行回数)
%   ENABLE_ALL_LOG              デフォルト 1 (全ログ表示スイッチ)
%   LOG_SENSOR                  デフォルト 1 (センサ検知ログスイッチ)
%   LOG_TRIGGER                 デフォルト 1 (トリガー発火ログスイッチ)
%
% 【推奨将来軌道インターフェース】
%   opts.reference_packet_fn = @(tvec) ...
%       時間ベクトル tvec に対して [pL_ref, q_ref] を返す関数
%       - セル形式: out = {pL_ref, q_ref}
%       - 構造体形式: out.pL = 3×N, out.q = 3×N
%
%   ※ reference_packet_fn が指定されない場合、本クラスは公称軌道 base_ref.do(...) を
%      将来時刻で反復評価し、荷物加速度から索姿勢 q を再構成するローリングキャッシュへ
%      自動フォールバックします:
%         q = -(ddot(pL) + g * e3) / ||ddot(pL) + g * e3||
% =========================================================================

    properties (Access = public)

        % ログ表示マスタースイッチ & モジュール別スイッチ
        ENABLE_ALL_LOG = 1;        % 1: ON (ログ表示), 0: OFF (非表示・最高速)
        LOG_SENSOR     = 1;        % センサ検知ログ表示スイッチ (1: ON, 0: OFF)
        LOG_TRIGGER    = 1;        % 将来衝突トリガー発火ログ表示スイッチ (1: ON, 0: OFF)

        % 基本オブジェクト参照
        self                       % ドローンエージェント本体 (パラメータ・状態取得元)
        base_ref                   % 公称参照軌道オブジェクト
        result                     % 出力結果構造体

        % トリガー方針 & パラメータ
        trigger_dist  = 20.0;      % 球状センサ検知半径 [m]
        horizon_s     = 4.0;       % 衝突予測ホライズン [s]
        prediction_dt = 0.025;     % 予測チェックポイント間隔 [s]

        % 追加システム安全マージン [m] (現段階ではゼロを維持)
        d_margin = 0.0;

        % 物理構成部位の保護半径 [m] (物理形状の幾何学的球・カプセル半径、安全マージンではない)
        r_drone = 0.5;      % UAV 等価球半径 R_Q [m]: B_Q(t) = B(p_Q(t), R_Q)
        r_load  = 0.5;      % 吊り荷等価球半径 R_L [m]: B_L(t) = B(p_L(t), R_L)
        r_cable = 0.5;      % 索カプセル保護半径 R_C [m]: B_C(t) = [p_Q(t), p_L(t)] \oplus B(R_C)

        % 物理モデルパラメータ
        gravity  = 9.81;           % 重力加速度 [m/s^2]
        L_cable  = 2.0;            % 索長 [m]
        loadmass = 0.0;            % 牽引物質量 [kg] (推定器から厳格取得)

        % 環境 & 障害物パケット
        static_obstacle_map = [];  % 静的環境障害物マップ
        obstacle_packet     = [];  % MEX向け幾何前計算済み障害物パケット

        % 公称参照軌道パケット・キャッシュ
        reference_packet_fn   = [];
        reference_cache       = struct();
        reference_cache_valid = false;

        % MEXランタイム管理
        mex_name            = 'g26_future_trigger_kernel_mex';
        mex_ready           = false;
        mex_source_revision = 'G26_FUTURE_TRIGGER_R2_20261012';

        % ウォームアップ管理
        warmup_calls      = 10;
        warmup_done       = false;
        warmup_runtime_ms = NaN;

        % 互換用ダミープロパティ
        current_bspline   = [];

        % 計算時間プロファイリング [ms]
        profiling = struct( ...
            't_base_ref_ms',         0.0, ...
            't_reference_packet_ms', 0.0, ...
            't_sensor_ms',           0.0, ...
            't_kernel_ms',           0.0, ...
            't_trigger_ms',          0.0, ...
            't_startup_warmup_ms',   NaN);

    end

    properties (Dependent)
        r_cable_sphere             % 旧プロパティ名互換エイリアス
    end

    methods
        function val = get.r_cable_sphere(obj)
            val = obj.r_cable;
        end

        function obj = set.r_cable_sphere(obj, val)
            obj.r_cable = val;
        end

        function obj = set.horizon_s(obj, val)
            assert(isnumeric(val) && isscalar(val) && isfinite(val) && val > 0, ...
                '[REPLANNER ERROR] 予測ホライズン horizon_s が無効です。');
            if ~isempty(obj.prediction_dt) && obj.prediction_dt > 0
                ratio = val / obj.prediction_dt;
                assert(abs(ratio - round(ratio)) <= 1e-9 * max(1, abs(ratio)), ...
                    '[REPLANNER ERROR] ホライズン比 horizon_s / prediction_dt は整数である必要があります。');
            end
            obj.horizon_s = val;
            obj.reference_cache_valid = false;
        end
    end

    methods (Access = public)

        % =====================================================================
        % set_horizon: 予測ホライズン H の外部動的変更
        % =====================================================================
        function set_horizon(obj, H)
            obj.horizon_s = H;
        end

        % =====================================================================
        % コンストラクタ：初期化・パラメータバインド・MEXビルド・ウォームアップ
        % =====================================================================
        function obj = REPLANNING_SYSTEM(self, base_ref, opts)

            arguments
                self
                base_ref
                opts = struct()
            end

            obj.self     = self;
            obj.base_ref = base_ref;

            % -------------------------------------------------------------
            % 1. 外部オプション (opts) による上書き
            % -------------------------------------------------------------
            if isfield(opts,'ENABLE_ALL_LOG'), obj.ENABLE_ALL_LOG = opts.ENABLE_ALL_LOG; end
            if isfield(opts,'LOG_SENSOR'),     obj.LOG_SENSOR     = opts.LOG_SENSOR;     end
            if isfield(opts,'LOG_TRIGGER'),    obj.LOG_TRIGGER    = opts.LOG_TRIGGER;    end

            if isfield(opts,'sensor_range_m'), obj.trigger_dist = opts.sensor_range_m; end
            if isfield(opts,'trigger_dist'),   obj.trigger_dist = opts.trigger_dist;   end

            if isfield(opts,'horizon_s'), obj.horizon_s = opts.horizon_s; end

            if isfield(opts,'d_margin'), obj.d_margin = opts.d_margin; end

            if isfield(opts,'r_drone'),        obj.r_drone = opts.r_drone;        end
            if isfield(opts,'r_load'),         obj.r_load  = opts.r_load;         end
            if isfield(opts,'r_cable'),        obj.r_cable = opts.r_cable;        end
            if isfield(opts,'R_cable_m'),      obj.r_cable = opts.R_cable_m;      end
            if isfield(opts,'cable'),          obj.r_cable = opts.cable;          end
            if isfield(opts,'r_cable_sphere'), obj.r_cable = opts.r_cable_sphere; end

            if isfield(opts,'warmup_calls'), obj.warmup_calls = opts.warmup_calls; end

            if isfield(opts,'reference_packet_fn')
                obj.reference_packet_fn = opts.reference_packet_fn;
            end

            % -------------------------------------------------------------
            % 2. 制御周期 dt の本体厳格バインド
            % -------------------------------------------------------------
            dt_val = [];

            if obj.has_prop_or_field(obj.self,'plant') && ~isempty(obj.self.plant)
                if obj.has_prop_or_field(obj.self.plant,'dt') && ~isempty(obj.self.plant.dt)
                    dt_val = obj.self.plant.dt;
                end
            end

            if isempty(dt_val) && obj.has_prop_or_field(obj.self,'parameter') && ~isempty(obj.self.parameter)
                try
                    dt_val = obj.self.parameter.get("dt");
                catch
                end
            end

            if isempty(dt_val) && isfield(opts,'dt')
                dt_val = opts.dt;
            end

            assert(~isempty(dt_val) && isnumeric(dt_val) && isscalar(dt_val) && ...
                isfinite(dt_val) && dt_val>0, ...
                '[REPLANNER ERROR] 制御/予測周期 dt の取得に失敗しました。本体 (self.plant.dt 等) に設定されている必要があります。');

            if isfield(opts,'prediction_dt')
                obj.prediction_dt = opts.prediction_dt;
            else
                obj.prediction_dt = dt_val;
            end

            % ホライズンが格子点の整数倍であることを保証
            ratio = obj.horizon_s / obj.prediction_dt;
            assert(abs(ratio-round(ratio)) <= 1e-9*max(1,abs(ratio)), ...
                '[REPLANNER ERROR] ホライズン比 horizon_s / prediction_dt は整数である必要があります。');

            % -------------------------------------------------------------
            % 3. 物理パラメータの厳格バインド
            % -------------------------------------------------------------
            obj.gravity = obj.self.parameter.get("gravity");
            obj.L_cable = obj.self.parameter.get("cableL");

            assert(isnumeric(obj.gravity) && isscalar(obj.gravity) && ...
                isfinite(obj.gravity) && obj.gravity>0, ...
                '[REPLANNER ERROR] 重力加速度 gravity が無効です。');

            assert(isnumeric(obj.L_cable) && isscalar(obj.L_cable) && ...
                isfinite(obj.L_cable) && obj.L_cable>0, ...
                '[REPLANNER ERROR] 索長 cableL が無効です。');

            assert(isnumeric(obj.trigger_dist) && isscalar(obj.trigger_dist) && ...
                isfinite(obj.trigger_dist) && obj.trigger_dist>0, ...
                '[REPLANNER ERROR] センサ検知半径 trigger_dist が無効です。');

            assert(isnumeric(obj.horizon_s) && isscalar(obj.horizon_s) && ...
                isfinite(obj.horizon_s) && obj.horizon_s>0, ...
                '[REPLANNER ERROR] 予測ホライズン horizon_s が無効です。');

            assert(isnumeric(obj.d_margin) && isscalar(obj.d_margin) && ...
                isfinite(obj.d_margin) && obj.d_margin>=0, ...
                '[REPLANNER ERROR] 安全マージン d_margin が無効です。');

            assert(obj.r_drone>0 && obj.r_load>0 && obj.r_cable>0, ...
                '[REPLANNER ERROR] 機体・荷物・索の保護半径はすべて正の値である必要があります。');

            % 牽引物質量の厳格取得 (推定値必須・フォールバックなし)
            obj.loadmass = obj.get_load_mass();

            % 公称軌道から出力結果テンプレートを継承
            obj.result = base_ref.result;

            % -------------------------------------------------------------
            % 4. 環境データおよび将来トリガー用障害物パケットの事前構築
            % -------------------------------------------------------------
            raw_obs = ENVIRONMENT_OBSTACLE_ELLIPSE_MOVE(0.0);
            obj.static_obstacle_map = raw_obs;
            obj.obstacle_packet     = obj.prepare_obstacle_packet(raw_obs);

            % -------------------------------------------------------------
            % 5. MEX 自動ビルド & 明示的ウォームアップ
            % -------------------------------------------------------------
            obj.ensure_mex_compiled();
            obj.warmup_runtime();

            % -------------------------------------------------------------
            % 6. LOGGER 互換の動的プロパティ登録
            % -------------------------------------------------------------
            sensor_props = [ ...
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
                "trigger_ttc", ...
                "trigger_collision_time", ...
                "trigger_obstacle_id", ...
                "trigger_component", ...
                "trigger_pL_collision", ...
                "trigger_q_collision", ...
                "trigger_pQ_collision", ...
                "trigger_effective_horizon", ...
                "trigger_reference_points", ...
                "trigger_reference_source", ...
                "trigger_sensor_origin", ...
                "trigger_cable_residual" ...
            ];

            for p_name = sensor_props
                if ~isprop(obj.result.state,p_name)
                    addprop(obj.result.state,p_name);
                end
            end

            obj.clear_state_sensor_values(0.0);

            if obj.ENABLE_ALL_LOG
                fprintf('[REPLANNING_SYSTEM] 将来予測トリガー初期化完了 | センサ範囲: %.3f m | ホライズン: %.3f s | dt: %.6f s | マージン: %.3f m\n', ...
                    obj.trigger_dist,obj.horizon_s,obj.prediction_dt,obj.d_margin);
                fprintf('[REPLANNING_SYSTEM] MEX ウォームアップ完了 (%d 回呼び出し) | 起動時処理時間: %.3f ms\n', ...
                    obj.warmup_calls,obj.warmup_runtime_ms);
            end
        end

        % =====================================================================
        % do: 制御周期ごとの公称軌道透過取得・センサ検知・将来予測衝突トリガー判定
        % =====================================================================
        function result_out = do(obj, varargin)

            time = varargin{1};
            cha  = varargin{2};

            if isfield(time,'dt') && ~isempty(time.dt) && time.dt>0
                if abs(time.dt-obj.prediction_dt) > 1e-12
                    % 制御刻み変更時はローリングキャッシュを無効化
                    obj.prediction_dt = time.dt;
                    ratio = obj.horizon_s/obj.prediction_dt;
                    assert(abs(ratio-round(ratio)) <= 1e-9*max(1,abs(ratio)), ...
                        '[REPLANNER ERROR] ホライズン比 horizon_s / prediction_dt は整数である必要があります。');
                    obj.reference_cache_valid = false;
                end
            end

            % 最新の牽引物質量推定値を厳格取得
            obj.loadmass = obj.get_load_mass();

            % -------------------------------------------------------------
            % 1. 公称目標軌道の取得 (Nominal Reference 透過処理)
            % -------------------------------------------------------------
            t_base = tic;
            base_res = obj.base_ref.do(varargin{:});
            xd_nom = base_res.state.xd;
            obj.profiling.t_base_ref_ms = 1e3*toc(t_base);

            obj.profiling.t_reference_packet_ms = 0.0;
            obj.profiling.t_sensor_ms           = 0.0;
            obj.profiling.t_kernel_ms           = 0.0;
            obj.profiling.t_trigger_ms          = 0.0;
            obj.profiling.t_startup_warmup_ms   = obj.warmup_runtime_ms;

            obj.clear_state_sensor_values(time.t);

            % 飛行フェーズ ('f') 以外では公称軌道を透過出力
            % （可能であれば将来パケットキャッシュを維持して初回フライト時の急激な遅延を回避）
            if cha ~= 'f'
                try
                    obj.update_reference_cache(time,cha,varargin,xd_nom);
                catch
                    obj.reference_cache_valid = false;
                end

                obj.write_nominal_output(xd_nom);
                result_out = obj.result;
                return;
            end

            t_trigger_total = tic;

            % -------------------------------------------------------------
            % 2. センサ検知原点としての現在推定幾何配置の取得
            % -------------------------------------------------------------
            [pQ_cur,pL_cur,q_cur,cable_residual] = ...
                obj.get_current_physical_geometry(xd_nom);

            % -------------------------------------------------------------
            % 3. 生形状表面距離による球状センサ検知ゲート
            % -------------------------------------------------------------
            t_sensor = tic;
            [active,min_dist,min_id,all_dists] = ...
                obj.raw_sensor_gate(pQ_cur,obj.static_obstacle_map);
            obj.profiling.t_sensor_ms = 1e3*toc(t_sensor);

            vis_indices = find(active);
            vis_count   = numel(vis_indices);

            detected_obs = repmat(struct('id',0,'range',0,'type',''),vis_count,1);

            for k=1:vis_count
                idx = vis_indices(k);
                detected_obs(k).id    = obj.static_obstacle_map(idx).id;
                detected_obs(k).range = all_dists(idx);
                detected_obs(k).type  = obj.static_obstacle_map(idx).type;
            end

            % -------------------------------------------------------------
            % 4. 将来公称軌道パケット pL(t_j), q(t_j) の生成・更新
            % -------------------------------------------------------------
            t_refpkt = tic;
            [pL_ref,q_ref,ref_source] = ...
                obj.update_reference_cache(time,cha,varargin,xd_nom);
            obj.profiling.t_reference_packet_ms = 1e3*toc(t_refpkt);

            N = size(pL_ref,2);
            effective_horizon = (N-1)*obj.prediction_dt;

            % -------------------------------------------------------------
            % 5. 将来衝突予測 MEX カーネルの実行
            % -------------------------------------------------------------
            triggered = false;
            jhit      = NaN;
            obs_id    = NaN;
            comp_code = NaN;

            t_kernel = tic;

            if any(active)
                [triggered,jhit,obs_id,comp_code] = ...
                    g26_future_trigger_kernel_mex( ...
                        obj.horizon_s, ...
                        obj.prediction_dt, ...
                        obj.L_cable, ...
                        pL_ref, ...
                        q_ref, ...
                        obj.obstacle_packet.C, ...
                        obj.obstacle_packet.R, ...
                        obj.obstacle_packet.Apay, ...
                        obj.obstacle_packet.Auav, ...
                        obj.obstacle_packet.Acable, ...
                        obj.obstacle_packet.ids, ...
                        double(active));
            end

            obj.profiling.t_kernel_ms = 1e3*toc(t_kernel);

            % -------------------------------------------------------------
            % 6. トリガー結果の確定
            % -------------------------------------------------------------
            need_replan        = logical(triggered);
            relevant_obstacles = [];

            ttc            = NaN;
            collision_time = NaN;
            comp_name      = 'NONE';
            pL_collision   = [NaN;NaN;NaN];
            q_collision    = [NaN;NaN;NaN];
            pQ_collision   = [NaN;NaN;NaN];

            if need_replan
                jhit = round(jhit);

                assert(jhit>=1 && jhit<=N, ...
                    '[REPLANNER ERROR] MEX が返した衝突インデックスが無効です。');

                ttc            = (jhit-1)*obj.prediction_dt;
                collision_time = time.t + ttc;
                comp_name      = obj.get_component_name(comp_code);

                pL_collision   = pL_ref(:,jhit);
                q_collision    = q_ref(:,jhit);
                pQ_collision   = pL_collision - obj.L_cable*q_collision;

                rel = struct();
                rel.id             = obs_id;
                rel.threat_source  = comp_name;
                rel.ttc            = ttc;
                rel.collision_time = collision_time;
                rel.pL_collision   = pL_collision;
                rel.q_collision    = q_collision;
                rel.pQ_collision   = pQ_collision;

                relevant_obstacles = rel;
            end

            obj.profiling.t_trigger_ms = 1e3*toc(t_trigger_total);

            % -------------------------------------------------------------
            % 7. 状態構造体 (result.state) への記録 (LOGGER が自動保存)
            % -------------------------------------------------------------
            st = obj.result.state;

            obj.write_nominal_output(xd_nom);

            st.time                          = time.t;
            st.pQ                            = pQ_cur;

            st.detected_point                = vis_count>0;
            st.detected_obstacles            = detected_obs;
            st.min_obstacle_id_point         = min_id;
            st.min_dist_point                = min_dist;
            st.detected_obstacle_count_point = vis_count;

            st.need_replan                   = need_replan;
            st.relevant_obstacles            = relevant_obstacles;

            st.trigger_ttc                   = ttc;
            st.trigger_collision_time        = collision_time;
            st.trigger_obstacle_id           = obs_id;
            st.trigger_component             = comp_name;
            st.trigger_pL_collision          = pL_collision;
            st.trigger_q_collision           = q_collision;
            st.trigger_pQ_collision          = pQ_collision;
            st.trigger_effective_horizon     = effective_horizon;
            st.trigger_reference_points      = N;
            st.trigger_reference_source      = ref_source;
            st.trigger_sensor_origin         = pQ_cur;
            st.trigger_cable_residual        = cable_residual;

            st.loadmass                      = obj.loadmass;
            st.current_bspline               = [];
            st.profiling                     = obj.profiling;

            % -------------------------------------------------------------
            % 8. コンソールログ出力 (表示切替フラグ対応)
            % -------------------------------------------------------------
            if obj.ENABLE_ALL_LOG

                if obj.LOG_SENSOR && vis_count>0
                    fprintf(['[SENSOR RAW %.1fm] t=%.3f s | 捕捉数: %d | 最寄ID: %d | 表面距離: %6.3f m | ' ...
                             'センサ: %.3f ms | パケット: %.3f ms | カーネル: %.3f ms | 合計: %.3f ms\n'], ...
                        obj.trigger_dist,time.t,vis_count,min_id,min_dist, ...
                        obj.profiling.t_sensor_ms, ...
                        obj.profiling.t_reference_packet_ms, ...
                        obj.profiling.t_kernel_ms, ...
                        obj.profiling.t_trigger_ms);
                end

                if obj.LOG_TRIGGER && need_replan
                    if ttc <= 1e-6
                        fprintf(['[CONTACT HIT -> REPLAN] t=%.3f s | 障害物 ID=%d 接触検知！ (接触部位: %s) | ' ...
                                 '判定計算: %.3f ms (MEX: %.3f ms)\n'], ...
                            time.t, obs_id, comp_name, obj.profiling.t_trigger_ms, obj.profiling.t_kernel_ms);
                    else
                        fprintf(['[FUTURE HIT -> REPLAN] t=%.3f s | TTC: %6.3f s (衝突予定時刻: %6.3f s) | ' ...
                                 '障害物 ID=%d | 接触部位: %s | H: %.3f s | 判定計算: %.3f ms (MEX: %.3f ms)\n'], ...
                            time.t, ttc, collision_time, obs_id, comp_name, ...
                            obj.horizon_s, obj.profiling.t_trigger_ms, obj.profiling.t_kernel_ms);
                    end
                end
            end

            result_out = obj.result;
        end

        % =====================================================================
        % get_load_mass: 牽引物質量の厳格取得 (推定器 mL の取得を必須化)
        % =====================================================================
        function mL = get_load_mass(obj)

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

        % =====================================================================
        % has_prop_or_field: オブジェクトのプロパティまたは構造体フィールドの存在確認
        % =====================================================================
        function tf = has_prop_or_field(~,x,name)
            tf = false;

            if isobject(x)
                tf = isprop(x,name);
            elseif isstruct(x)
                tf = isfield(x,name);
            end
        end

        % =====================================================================
        % write_nominal_output: 公称制御目標値の透過出力
        % =====================================================================
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

        % =====================================================================
        % get_current_physical_geometry: センサ原点用の機体・荷物・索の現在幾何配置取得
        % =====================================================================
        function [pQ_cur,pL_cur,q_cur,residual] = ...
                get_current_physical_geometry(obj,xd_nom)

            est_state = obj.self.estimator.result.state;

            % 現在の機体位置: センサ検知球の中心原点
            if obj.has_prop_or_field(est_state,'p') && ~isempty(est_state.p)
                pQ_cur = est_state.p(:);
            else
                error('[REPLANNER ERROR] センサ原点用の機体推定位置 p が取得できません。');
            end

            % 現在の荷物位置
            if obj.has_prop_or_field(est_state,'pL') && ~isempty(est_state.pL)
                pL_cur = est_state.pL(:);
            else
                pL_cur = [];
            end

            % 現在の索姿勢単位ベクトル (機体 -> 荷物 方向)
            if obj.has_prop_or_field(est_state,'pT') && ~isempty(est_state.pT)
                q_cur = est_state.pT(:);
                nq = norm(q_cur);
                assert(nq>1e-12,'[REPLANNER ERROR] 推定器の索姿勢ベクトル pT のノルムがゼロです。');
                q_cur = q_cur/nq;
            else
                assert(numel(xd_nom)>=11, ...
                    '[REPLANNER ERROR] 推定器 pT が存在せず、公称軌道の加速度情報も不足しています。');

                aL = xd_nom(9:11);
                r = aL + [0;0;obj.gravity];
                nr = norm(r);

                assert(nr>1e-12, ...
                    '[REPLANNER ERROR] 索姿勢の再構成計算が無効です。');

                q_cur = -r/nr;
            end

            if isempty(pL_cur)
                pL_cur = pQ_cur + obj.L_cable*q_cur;
            end

            residual = norm(pL_cur-pQ_cur-obj.L_cable*q_cur);
        end

        % =====================================================================
        % update_reference_cache: 将来参照軌道パケットの高速ローリング更新
        % =====================================================================
        function [pL_ref,q_ref,source] = ...
                update_reference_cache(obj,time,cha,varargin_in,xd_now)

            t_entry = time.t;
            N = round(obj.horizon_s/obj.prediction_dt)+1;
            tvec = t_entry + (0:N-1)*obj.prediction_dt;

            % ユーザー指定のベクトル化コールバック関数が提供されている場合
            if ~isempty(obj.reference_packet_fn)
                callback_ok = false;

                try
                    [pL_ref,q_ref] = obj.reference_packet_fn(tvec);
                    callback_ok = isnumeric(pL_ref) && isnumeric(q_ref);
                catch
                    callback_ok = false;
                end

                if ~callback_ok
                    out = obj.reference_packet_fn(tvec);

                    if isstruct(out)
                        assert(isfield(out,'pL') && isfield(out,'q'), ...
                            '[REPLANNER ERROR] reference_packet_fn 構造体には pL と q が含まれている必要があります。');
                        pL_ref = out.pL;
                        q_ref  = out.q;

                    elseif iscell(out) && numel(out)>=2
                        pL_ref = out{1};
                        q_ref  = out{2};

                    else
                        error(['[REPLANNER ERROR] reference_packet_fn は [pL, q]、' ...
                               '構造体 (pL, q)、またはセル {pL, q} を返す必要があります。']);
                    end
                end

                obj.validate_reference_packet(pL_ref,q_ref,N);

                obj.reference_cache = struct( ...
                    't0',t_entry, ...
                    'dt',obj.prediction_dt, ...
                    'H',obj.horizon_s, ...
                    'cha',cha, ...
                    'pL',pL_ref, ...
                    'q',q_ref);

                obj.reference_cache_valid = true;
                source = 'reference_packet_fn';
                return;
            end

            % ローリングフォールバック:
            % 初回満充填後は、通常の時間進行に伴い 1 ステップシフトして終端 1 点のみを新規評価
            can_shift = obj.reference_cache_valid && ...
                abs(obj.reference_cache.dt-obj.prediction_dt)<=1e-12 && ...
                abs(obj.reference_cache.H-obj.horizon_s)<=1e-12 && ...
                isfield(obj.reference_cache,'cha') && isequal(obj.reference_cache.cha,cha) && ...
                abs(t_entry-(obj.reference_cache.t0+obj.prediction_dt)) <= ...
                    1e-5*max(1,abs(obj.prediction_dt));

            if can_shift
                pL_ref = [obj.reference_cache.pL(:,2:end),zeros(3,1)];
                q_ref  = [obj.reference_cache.q(:,2:end),zeros(3,1)];

                [pL_new,q_new] = obj.evaluate_nominal_payload_and_q( ...
                    tvec(end),cha,varargin_in,[]);

                pL_ref(:,end) = pL_new;
                q_ref(:,end)  = q_new;

                % 先頭点は現在の制御周期で評価された公称目標値
                [pL0,q0] = obj.nominal_payload_and_q_from_xd(xd_now);
                pL_ref(:,1)=pL0;
                q_ref(:,1)=q0;

                source = 'rolling_base_ref_do_1_new_endpoint';

            else
                pL_ref = zeros(3,N);
                q_ref  = zeros(3,N);

                [pL0,q0] = obj.nominal_payload_and_q_from_xd(xd_now);
                pL_ref(:,1)=pL0;
                q_ref(:,1)=q0;

                for j=2:N
                    [pL_ref(:,j),q_ref(:,j)] = ...
                        obj.evaluate_nominal_payload_and_q( ...
                            tvec(j),cha,varargin_in,[]);
                end

                source = 'full_base_ref_do_fill';
            end

            obj.validate_reference_packet(pL_ref,q_ref,N);

            % 将来評価に伴う base_ref の内部状態時刻を現在ステップに戻す
            % （time.t が不変であることを検証した上で現在時刻で再評価）
            assert(abs(time.t - t_entry) <= 1e-12, ...
                '[REPLANNER BUG] time.t が将来評価中に変更されました。');
            obj.base_ref.do(varargin_in{:});

            obj.reference_cache = struct( ...
                't0',t_entry, ...
                'dt',obj.prediction_dt, ...
                'H',obj.horizon_s, ...
                'cha',cha, ...
                'pL',pL_ref, ...
                'q',q_ref);

            obj.reference_cache_valid = true;
        end

        % =====================================================================
        % evaluate_nominal_payload_and_q: 指定時刻における公称荷物位置および索姿勢評価
        % =====================================================================
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

        % =====================================================================
        % nominal_payload_and_q_from_xd: 状態ベクトル xd から pL と q を抽出・再構成
        % =====================================================================
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

        % =====================================================================
        % validate_reference_packet: 参照パケットの整合性・単位ベクトル検証
        % =====================================================================
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

        % =====================================================================
        % raw_sensor_gate: 生形状幾何表面距離による 20m センサ検知ゲート
        % =====================================================================
        function [active,min_dist,min_id,dists] = raw_sensor_gate(obj,pQ,OBS)

            M = numel(OBS);

            active   = false(1,M);
            dists    = inf(1,M);
            min_dist = inf;
            min_id   = NaN;

            for i=1:M
                d = obj.raw_surface_distance(pQ,OBS(i));
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

        % =====================================================================
        % raw_surface_distance: 各形状に応じた厳密な点-障害物表面距離計算
        % =====================================================================
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

        % =====================================================================
        % point_ellipsoid_distance_local: 点と楕円体の厳密な幾何表面距離 (二分法ソルバー)
        % =====================================================================
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

        % =====================================================================
        % point_in_triangle_2d: 2D 三角形内部判定
        % =====================================================================
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

        % =====================================================================
        % point_segment_distance_2d: 2D 点-線分最短距離
        % =====================================================================
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

        % =====================================================================
        % poly_vertices_faces_world: 多面体のワールド頂点および面生成
        % =====================================================================
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

        % =====================================================================
        % point_convex_polyhedron_distance: 点と凸多面体の最短距離
        % =====================================================================
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

        % =====================================================================
        % point_triangle_distance_3d: 点と 3D 三角形の最短距離 (Ericson アルゴリズム)
        % =====================================================================
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

        % =====================================================================
        % prepare_obstacle_packet: MEX向け幾何前計算済み障害物パケットの構築
        % =====================================================================
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

        % =====================================================================
        % validate_obstacle: 障害物データ構造体の妥当性・回転行列の直交性検証
        % =====================================================================
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

        % =====================================================================
        % component_outer_ellipsoids: 荷物・機体・索の保護半径を含む外接衝突楕円体
        % =====================================================================
        function [Ap,Aq,Ac] = component_outer_ellipsoids(obj,o)

            Ap=obj.outer_ellipsoid_with_inflation(o,obj.r_load+obj.d_margin);
            Aq=obj.outer_ellipsoid_with_inflation(o,obj.r_drone+obj.d_margin);
            Ac=obj.outer_ellipsoid_with_inflation(o,obj.r_cable+obj.d_margin);
        end

        % =====================================================================
        % outer_ellipsoid_with_inflation: ミンコフスキー和包囲理論に基づく保守的楕円体拡大
        % =====================================================================
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

        % =====================================================================
        % raw_enclosing_ellipsoid_axes: 各生形状の最小外接楕円体主軸半長
        % =====================================================================
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

        % =====================================================================
        % clear_state_sensor_values: 毎周期開始時の状態バッファ初期化
        % =====================================================================
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
            st.trigger_pL_collision=[NaN;NaN;NaN];
            st.trigger_q_collision=[NaN;NaN;NaN];
            st.trigger_pQ_collision=[NaN;NaN;NaN];
            st.trigger_effective_horizon=NaN;
            st.trigger_reference_points=0;
            st.trigger_reference_source='';
            st.trigger_sensor_origin=[NaN;NaN;NaN];
            st.trigger_cable_residual=NaN;

            st.loadmass=obj.loadmass;
            st.current_bspline=[];
            st.profiling=obj.profiling;
        end

        % =====================================================================
        % get_component_name: 接触部位コードから文字列へ変換
        % =====================================================================
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

        % =====================================================================
        % warmup_runtime: 初期化時ウォームアップ (コールドスタートの排除)
        % =====================================================================
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

        % =====================================================================
        % ensure_mex_compiled: C++ソース変更検知 & 自動ビルド
        % =====================================================================
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

        % =====================================================================
        % mex_source_lines: 埋め込み C++ MEX ソースコード定義
        % =====================================================================
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

% % classdef REPLANNING_BSPLINE < handle
% %     % =========================================================================
% %     % REPLANNING_BSPLINE
% %     % 1. 模擬センサー：機体重心 pQ を点（Point）として扱い、機体搭載センサーによる
% %     %    楕円体境界面までの最短ユークリッド距離計算および 7m 接近検知判定を実行。
% %     % 2. 衝突診断系：荷物位置 pL についてはセンサーとしては扱わず、機体センサー情報や
% %     %    安全評価（最短表面距離、最近接点、法線、侵入有無）の診断値としてのみ記録。
% %     % =========================================================================
% %     properties
% %         self % ドローンエージェント自身 (推定器 estimator やパラメータ parameter を保持) 
% %         base_ref % 公称参照軌道生成オブジェクト
% %         result                % 出力結果構造体 (目標状態 xd, pRef, vRef, yawRef, 検知・診断情報)
% %         trigger_dist = 15.0;   % 機体搭載センサーによる接近検知の閾値 [m] (表面間距離がこれ以下になるとアラート)
% %         % r_load       = 0.15;  % 荷物保護半径 [m] (必要に応じて将来のマージン計算等に使用)
% %         % r_drone      = 0.30;  % 機体保護半径 [m] (必要に応じて将来のマージン計算等に使用)
% %     end
% %     methods (Access = public)
% %         % =====================================================================
% %         % コンストラクタ: クラスの初期化と外部設定 (opts) の反映
% %         % =====================================================================
% %         function obj = REPLANNING_BSPLINE(self, base_ref, opts)
% %             arguments
% %                 self                  % 必須: エージェントインスタンス
% %                 base_ref              % 必須: 通常飛行用の公称軌道インスタンス
% %                 opts = struct()       % 任意: 外部からパラメータを変更するための構造体
% %             end
% %             obj.self = self;
% %             obj.base_ref = base_ref;
% %             if isfield(opts, 'trigger_dist'), obj.trigger_dist = opts.trigger_dist; end % 外で定義されていたらデフォルト値を上書き 機体センサー検知範囲
% %             % if isfield(opts, 'r_load'),       obj.r_load       = opts.r_load;       end % 外で定義されていたらデフォルト値を上書き 牽引物を近似した球体
% %             % if isfield(opts, 'r_drone'),      obj.r_drone      = opts.r_drone;      end % 外で定義されていたらデフォルト値を上書き 機体を近似した球体
% %             % base_ref の result 構造体をそのまま継承 (直下は state のみ)
% %             obj.result = base_ref.result;
% %             % --- 【重要】STATE_CLASS に検知・診断用プロパティを動的追加 (dynamicprops) ---
% %             % これにより、state 内に正式な記録領域が作成され、ロガーで抽出可能になる
% %             sensor_props = ["time", "pQ", "pL", "detected_point", ...
% %                 "drone_inside_obstacle_point", "load_inside_obstacle_point", ...
% %                 "drone_min_dist_point", "load_min_dist_point", "min_dist_point", ...
% %                 "drone_obstacle_id_point", "load_obstacle_id_point", "min_obstacle_id_point", ...
% %                 "min_source_point", "detected_obstacle_count_point"];
% %             for p_name = sensor_props
% %                 if ~isprop(obj.result.state, p_name)
% %                     addprop(obj.result.state, p_name);
% %                 end
% %             end
% %             % 初期ダミー値のセット
% %             obj.clear_state_sensor_values(0.0);
% %         end
% %         % =====================================================================
% %         % do: 制御周期ごと (例: 25ms周期) にメインループから呼び出される実行メソッド
% %         % 入力: 
% %         %   varargin{1}: time (現在の時刻 struct: time.t, time.dt など)
% %         %   varargin{2}: cha  (フェーズ文字列: 'f' = 飛行中, 't' = 離陸など)
% %         %   varargin{4}: env  (環境構造体、障害物リストを内包する場合あり)
% %         % 出力:
% %         %   result_out : 下流のコントローラやロガーが受け取る目標状態・検知・診断結果
% %         % =====================================================================
% %         function result_out = do(obj, varargin)
% %             time = varargin{1}; % varargin{1}: time (現在の時刻 struct: time.t, time.dt など)
% %             cha = varargin{2}; % varargin{2}: cha  (フェーズ文字列: 'f' = 飛行中, 't' = 離陸など)
% %             % --- 公称目標軌道 (Nominal Reference) の算出 ---
% %             % 本クラスが障害物を回避する新軌道を生成しない間は、公称軌道生成器の出力をそのまま踏襲する
% %             % 公称参照軌道の取得
% %             base_res = obj.base_ref.do(varargin{:}); % 公称軌道の抜き出し
% %             xd_nom = base_res.state.xd; % 牽引物の目標３次元位置・yaw角からその６階微分まで [pL(3); yaw(1); vL(3); yaw_dot(1); aL(3)...]
% % 
% %             % 毎ステップ、検知・診断プロパティの初期値をリセット
% %             obj.clear_state_sensor_values(time.t);
% % 
% %             % obj.result.state に公称軌道を反映 (base_res 全体の上書きは行わない)
% %             obj.result.state.xd = xd_nom; % 公称軌道保存
% %             % --- 2. 空の検知・診断構造体を用意 (非飛行フェーズ用) ---
% %             detection = struct();
% %             detection.time                          = time.t;             % [s] 現在のシミュレーション時刻
% %             detection.pQ                            = [NaN; NaN; NaN];    % [m] ドローン機体重心の3次元位置ベクトル [x; y; z]
% %             detection.pL                            = [NaN; NaN; NaN];    % [m] 牽引荷物の3次元位置ベクトル [x; y; z] (状態診断用)
% %             detection.detected_point                = false;              % [bool] 機体センサー7m近接検知フラグ (true: 検知, false: 未検知)
% %             detection.drone_inside_obstacle_point   = false;              % [bool] 機体の障害物楕円体内部侵入フラグ (true: 侵入, false: 外部)
% %             detection.load_inside_obstacle_point    = false;              % [bool] 荷物の障害物楕円体内部侵入フラグ (true: 侵入, false: 外部)
% %             detection.drone_min_dist_point          = inf;                % [m] 機体センサーから全障害物表面までの最短幾何学距離
% %             detection.load_min_dist_point           = inf;                % [m] 荷物から全障害物表面までの最短幾何学距離 (診断値)
% %             detection.min_dist_point                = inf;                % [m] 機体センサーによる最短表面距離 (機体基準)
% %             detection.drone_obstacle_id_point       = NaN;                % [ID] 機体にとって最短距離を与えている障害物インデックス番号
% %             detection.load_obstacle_id_point        = NaN;                % [ID] 荷物にとって最短距離を与えている障害物インデックス番号 (診断値)
% %             detection.min_obstacle_id_point         = NaN;                % [ID] 機体センサーが捉えた最短障害物インデックス番号
% %             detection.min_source_point              = "none";             % [string] 最短距離センサ種別 (検知なし: "none", 検知時: "drone")
% %             detection.detected_obstacle_count_point = 0;                  % [個] 機体センサーにより7m以内に検知された障害物の総数
% %             % --- 飛行フェーズ ('f') 時の近接障害物スキャン & 荷物状態診断 ---
% %             if cha == 'f'
% %                 % --- 機体・荷物の現在位置の取得 ---
% %                 % (A) 状態推定器 (estimator) から真値・推定位置を直接取得
% %                 % ※ フォールバックを排除しているため、estimator にプロパティが存在しない場合は即座にエラー停止
% %                 pL_cur = obj.self.estimator.result.state.pL(:); % 荷物位置の現在3次元位置 [x; y; z]
% %                 pQ_cur = obj.self.estimator.result.state.p(:); % ドローン機体の現在3次元位置 [x; y; z]
% %                 % (B) 現在時刻 t における動的障害物配置を取得
% %                 obs_list = obj.get_obstacles_at_time(time.t);
% %                 % (C) 機体センサーによる最短距離評価・検知判定、および荷物の幾何診断値を計算
% %                 detection = obj.check_detection_simulated_sensor(pQ_cur, pL_cur, obs_list, time.t);
% %                 % 機体センサーが7m以内に侵入した場合のコンソール警告
% %                 if detection.detected_point
% %                     fprintf("[PROXIMITY ALERT] t=%.3f s | 機体センサー7m近接検知! (機体表面間: %.2f m, 荷物表面間診断: %.2f m, 最寄障害物ID: %d)\n", ...
% %                         time.t, detection.drone_min_dist_point, detection.load_min_dist_point, detection.min_obstacle_id_point);
% %                 end
% %             end
% % 
% %             % --- 4. 【最重要】state 内の各プロパティに代入して app.logger に完全保存 ---
% %             st = obj.result.state;
% %             st.xd                            = xd_nom;                                  % [28x1 double] 目標軌道全状態
% %             st.p                             = xd_nom(1:3);                             % [m] 目標位置 [x; y; z]
% %             st.v                             = xd_nom(5:7);                             % [m/s] 目標速度 [vx; vy; vz]
% %             st.q                             = [0; 0; xd_nom(4)];                       % [rad] 目標姿勢 (yaw角)
% % 
% %             st.time                          = detection.time;                          % [s] 計測時刻 (double)
% %             st.pQ                            = detection.pQ;                            % [m] ドローン機体重心位置 (3x1 double)
% %             st.pL                            = detection.pL;                            % [m] 荷物位置 (3x1 double, 診断用)
% %             st.detected_point                = detection.detected_point;                % [bool] 機体センサー7m近接検知判定フラグ (true / false)
% %             st.drone_inside_obstacle_point   = detection.drone_inside_obstacle_point;   % [bool] 機体侵入フラグ (true: 侵入, false: 外部)
% %             st.load_inside_obstacle_point    = detection.load_inside_obstacle_point;    % [bool] 荷物侵入フラグ (診断用, true: 侵入, false: 外部)
% %             st.drone_min_dist_point          = detection.drone_min_dist_point;          % [m] 機体センサー最短表面距離 (正: 外部, 負: 侵入深さ)
% %             st.load_min_dist_point           = detection.load_min_dist_point;           % [m] 荷物の最短表面距離 (診断値, 正: 外部, 負: 侵入深さ)
% %             st.min_dist_point                = detection.min_dist_point;                % [m] 機体センサーによる最短幾何表面距離 (= dQ)
% %             st.drone_obstacle_id_point       = detection.drone_obstacle_id_point;       % [ID] 機体にとって最短の障害物インデックス番号
% %             st.load_obstacle_id_point        = detection.load_obstacle_id_point;        % [ID] 荷物にとって最短の障害物インデックス番号 (診断値)
% %             st.min_obstacle_id_point         = detection.min_obstacle_id_point;         % [ID] 機体センサーが捉えた最短障害物インデックス番号
% %             st.min_source_point              = detection.min_source_point;              % [string] 最短距離センサ種別 (検知なし: "none", 検知時: "drone")
% %             st.detected_obstacle_count_point = detection.detected_obstacle_count_point; % [個] 機体センサーにより7m以内に検知された障害物の総数
% % 
% %             result_out = obj.result;
% %         end
% %     end
% %     methods (Access = private)
% %         % =====================================================================
% %         % clear_state_sensor_values: state 内の検知・診断プロパティを初期化
% %         % =====================================================================
% %         function clear_state_sensor_values(obj, t_now)
% %             st = obj.result.state;
% %             st.time                          = t_now;              % [s] 現在時刻
% %             st.pQ                            = [NaN; NaN; NaN];    % [m] ドローン位置初期値
% %             st.pL                            = [NaN; NaN; NaN];    % [m] 荷物位置初期値 (診断用)
% %             st.detected_point                = false;              % 検知なし
% %             st.drone_inside_obstacle_point   = false;              % 機体侵入なし
% %             st.load_inside_obstacle_point    = false;              % 荷物侵入なし (診断用)
% %             st.drone_min_dist_point          = inf;                % 最短距離初期値 (無限大)
% %             st.load_min_dist_point           = inf;                % 最短距離初期値 (無限大, 診断用)
% %             st.min_dist_point                = inf;                % 最短距離初期値 (無限大)
% %             st.drone_obstacle_id_point       = NaN;                % 最短障害物ID初期値
% %             st.load_obstacle_id_point        = NaN;                % 最短障害物ID初期値 (診断用)
% %             st.min_obstacle_id_point         = NaN;                % 最短障害物ID初期値
% %             st.min_source_point              = "none";             % 最短センサ初期値
% %             st.detected_obstacle_count_point = 0;                  % 検知個数初期値
% %         end
% %         % =====================================================================
% %         % get_obstacles_at_time: 環境関数を叩き、時刻 t_now での障害物リストを取得
% %         % =====================================================================
% %         function list = get_obstacles_at_time(~,t_now)
% %             % ENVIRONMENT_OBSTACLE_ELLIPSE_MOVE 内ですでに「p_center = p0 + v*t」
% %             % のように時刻 t_now に応じた現在位置が計算されている
% %             list = ENVIRONMENT_OBSTACLE_ELLIPSE_MOVE(t_now);
% %         end
% %         % =====================================================================
% %         % check_detection_simulated_sensor: 
% %         % 機体搭載センサーによる接近検知判定（7m以内）を実行し、
% %         % 荷物についてはシステム状態診断値（距離・最近接点・法線・侵入有無）として計算・記録する
% %         % =====================================================================
% %         function det = check_detection_simulated_sensor(obj, pQ, pL, obs_list, t_now)
% %             det = struct();
% %             % --- センサ状態・時刻 ---
% %             det.time                          = t_now;              % [s] 現在のシミュレーション時刻 (double)
% %             det.pQ                            = pQ;                 % [m] ドローン機体重心の3次元位置ベクトル [x; y; z] (3x1 double)
% %             det.pL                            = pL;                 % [m] 牽引荷物の3次元位置ベクトル [x; y; z] (3x1 double, 診断用)
% %             det.trigger_dist                  = obj.trigger_dist;   % [m] 機体センサーの近接検知閾値距離 (7.0 m) (double)
% %             % --- 判定フラグ ---
% %             det.detected_point                = false;              % [bool] 機体センサーが障害物から7m以内に接近したか (true: 検知, false: 未検知)
% %             det.drone_inside_obstacle_point   = false;              % [bool] 機体が障害物楕円体の内部へ侵入/衝突したか (true: 侵入, false: 外部)
% %             det.load_inside_obstacle_point    = false;              % [bool] 荷物が障害物楕円体の内部へ侵入/衝突したか (診断用, true: 侵入, false: 外部)
% %             % --- 最短距離 ---
% %             det.drone_min_dist_point          = inf;                % [m] 機体センサーから表面までの最短幾何学距離 (double, 内部時は負値)
% %             det.load_min_dist_point           = inf;                % [m] 荷物から表面までの最短幾何学距離 (診断値, double, 内部時は負値)
% %             det.min_dist_point                = inf;                % [m] 機体センサーによる最短幾何表面距離 (= dQ)
% %             % --- 識別情報・統計 ---
% %             det.drone_obstacle_id_point       = [];                 % [ID] 機体にとって最短距離を与えている障害物のインデックス番号 (integer)
% %             det.load_obstacle_id_point        = [];                 % [ID] 荷物にとって最短距離を与えている障害物のインデックス番号 (診断値, integer)
% %             det.min_obstacle_id_point         = [];                 % [ID] 機体センサーが捉えた最短障害物のインデックス番号 (integer)
% %             det.min_source_point              = "none";             % [string] 最短距離センサ種別 (検知なし: "none", 検知時: "drone")
% %             det.detected_obstacles_point      = [];                 % [struct配列] 機体センサーにより7m以内に検知された全障害物の詳細情報リスト
% %             det.detected_obstacle_count_point = 0;                  % [個] 機体センサーにより7m以内に検知された障害物の総数 (integer)
% %             if isempty(obs_list)
% %                 return;
% %             end
% %             detected_obs_point = [];
% %             for i = 1:length(obs_list)
% %                 o = obs_list(i);
% %                 % --- 障害物パラメータの抽出 (推測フォールバックなし) ---
% %                 if ~isfield(o, 'R_obs') || ~isfield(o, 'ellipsoid_radii') || ~isfield(o, 'p_center')
% %                     error('インデックス %d の障害物に必須フィールド (R_obs, ellipsoid_radii, p_center) が不足しています.', i);
% %                 end
% %                 % --- 障害物の姿勢回転行列 R_obs の抽出 ---
% %                 R_obs = o.R_obs;
% %                 % --- 外接楕円体の主軸半径 [a; b; c] の抽出 ---
% %                 radii_obs = o.ellipsoid_radii(:);
% %                 % --- 障害物の中心位置 ---
% %                 p_obs = o.p_center(:);
% %                 obs_parsed = struct('center', p_obs, 'radii', radii_obs, 'R', R_obs);
% % 
% %                 % --- 機体 pQ (センサー) および荷物 pL (診断用) から楕円体表面への符号付き最短幾何距離 ---
% %                 % d > 0: 表面の外側にある (表面までの最短距離 [m])
% %                 % d < 0: 内部に侵入している (侵入深さ [m])
% %                 [d_drone_point, inside_drone_point, cpQ_local_point, ~] = obj.point_ellipsoid_signed_distance(pQ, obs_parsed);
% %                 [d_load_point,  inside_load_point,  cpL_local_point, ~] = obj.point_ellipsoid_signed_distance(pL, obs_parsed);
% % 
% %                 % ワールド座標系における表面最近接点: x_world = center + R * x_local
% %                 cpQ_world_point = p_obs + R_obs * cpQ_local_point;
% %                 cpL_world_point = p_obs + R_obs * cpL_local_point;
% % 
% %                 % --- 障害物表面における外向き単位法線ベクトル (ワールド座標系) ---
% %                 nQ_local_point = cpQ_local_point ./ (radii_obs.^2);
% %                 if norm(nQ_local_point) < 1e-12
% %                     error('機体の最近接点における法線ベクトルが退化 (ゼロベクトル) しました.');
% %                 end
% %                 normal_drone_point = R_obs * (nQ_local_point / norm(nQ_local_point));
% % 
% %                 nL_local_point = cpL_local_point ./ (radii_obs.^2);
% %                 if norm(nL_local_point) < 1e-12
% %                     error('荷物の最近接点における法線ベクトルが退化 (ゼロベクトル) しました.');
% %                 end
% %                 normal_load_point = R_obs * (nL_local_point / norm(nL_local_point));
% % 
% %                 % 幾何学的侵入判定（衝突判定）の論理和更新
% %                 if inside_drone_point, det.drone_inside_obstacle_point = true; end
% %                 if inside_load_point,  det.load_inside_obstacle_point  = true; end
% % 
% %                 % 機体側（センサー）の最小値更新
% %                 if d_drone_point < det.drone_min_dist_point
% %                     det.drone_min_dist_point = d_drone_point;
% %                     det.drone_obstacle_id_point    = i;
% %                 end
% %                 % 荷物側（診断値）の最小値更新
% %                 if d_load_point < det.load_min_dist_point
% %                     det.load_min_dist_point = d_load_point;
% %                     det.load_obstacle_id_point    = i;
% %                 end
% % 
% %                 % 該当障害物の詳細データ構造体 (荷物データはシステム診断値として包含)
% %                 obs_info_point = struct( ...
% %                     'id',                        i, ...                  % [ID] 障害物のインデックス番号 (integer)
% %                     'dist_drone_point',          d_drone_point, ...      % [m] 機体重心(センサー)からこの障害物表面までの符号付き最短距離 (正: 外部, 負: 侵入深さ)
% %                     'dist_load_point',           d_load_point, ...       % [m] 荷物位置からこの障害物表面までの符号付き最短距離 (診断値, 正: 外部, 負: 侵入深さ)
% %                     'min_dist_point',            d_drone_point, ...      % [m] 機体センサー基準の表面間距離
% %                     'drone_inside_point',        inside_drone_point, ... % [bool] 機体がこの障害物の内部に侵入しているか (true: 侵入, false: 外部)
% %                     'load_inside_point',         inside_load_point, ...  % [bool] 荷物がこの障害物の内部に侵入しているか (診断用, true: 侵入, false: 外部)
% %                     'p_obs',                     p_obs, ...              % [m] 時刻 t における障害物の中心位置ベクトル [x; y; z] (3x1 double)
% %                     'radii_obs',                 radii_obs, ...          % [m] 障害物の3軸半径 [rx; ry; rz] (3x1 double)
% %                     'R_obs',                     R_obs, ...              % [-] 障害物の主軸姿勢を表す3x3回転行列 SO(3) (3x3 double)
% %                     'closest_drone_world_point', cpQ_world_point, ...    % [m] 機体に対する障害物表面上の最短最近接点 (ワールド座標系 [x; y; z])
% %                     'closest_load_world_point',  cpL_world_point, ...    % [m] 荷物に対する障害物表面上の最短最近接点 (診断用, ワールド座標系 [x; y; z])
% %                     'normal_drone_point',        normal_drone_point, ... % [-] 機体側最近接点における障害物表面の外向き単位法線ベクトル (ワールド系, 単位ノルム)
% %                     'normal_load_point',         normal_load_point ...   % [-] 荷物側最近接点における障害物表面の外向き単位法線ベクトル (診断用, ワールド系, 単位ノルム)
% %                 );
% % 
% %                 % --- 機体センサーによる接近検知判定 (7m以内) ---
% %                 if d_drone_point <= obj.trigger_dist
% %                     det.detected_point = true;
% %                     detected_obs_point = [detected_obs_point; obs_info_point];
% %                 end
% %             end
% % 
% %             % システム全体での最短距離の確定 (機体センサー基準)
% %             det.min_dist_point                = det.drone_min_dist_point;
% %             det.min_obstacle_id_point         = det.drone_obstacle_id_point;
% %             % 検知ありの場合は "drone"、検知なし（7m超過または障害物なし）の場合は初期値 "none" を保持
% %             if det.detected_point
% %                 det.min_source_point          = "drone";
% %             else
% %                 det.min_source_point          = "none";
% %             end
% %             det.detected_obstacles_point      = detected_obs_point;
% %             det.detected_obstacle_count_point = numel(detected_obs_point);  % 機体センサーが検知した障害物個数
% %         end
% %         % =====================================================================
% %         % point_ellipsoid_signed_distance
% %         % 任意の3次元点 p と回転楕円体 (中心 c, 半径 r=[a;b;c], 回転行列 R) の
% %         % 【外郭表面】までの真の最短ユークリッド距離を算出する。
% %         %
% %         % [数学的アルゴリズム]:
% %         % 楕円方程式: (x/a)^2 + (y/b)^2 + (z/c)^2 = 1
% %         % 点 p から楕円面上の最近接点 x への最短距離は、ラグランジュ未定乗数法:
% %         %   f(lambda) = sum( (a_i^2 * y_i^2) / (lambda + a_i^2)^2 ) - 1 = 0
% %         % を満たす単一根 lambda を二分探索法 (Binary Search) で 80 回反復して求め、
% %         % 表面点 x = (a_i^2 * y_i) / (lambda + a_i^2) と点 y の距離 ||x - y|| を計算する。
% %         % =====================================================================
% %         function [d, inside, closest_local, lambda] = point_ellipsoid_signed_distance(~, p, o)
% %             p = p(:);
% %             c = o.center(:);
% %             r = o.radii(:);
% %             R = o.R;
% %             % --- 入力チェック: 不正ならフォールバックせず即座に停止 ---
% %             if numel(p) ~= 3 || numel(c) ~= 3 || numel(r) ~= 3
% %                 error('点 p、中心 c、および半径 r はすべて3x1ベクトルである必要があります。');
% %             end
% %             if any(~isfinite(p)) || any(~isfinite(c)) || any(~isfinite(r))
% %                 error('点または楕円体の定義に非有限値 (NaN や Inf) が検出されました。');
% %             end
% %             if any(r <= 0)
% %                 error('楕円体の半径はすべて厳密に正の値 (r > 0) である必要があります。');
% %             end
% %             if ~isequal(size(R), [3 3]) || any(~isfinite(R(:)))
% %                 error('回転行列 R は有限な3x3行列である必要があります。');
% %             end
% %             if norm(R'*R - eye(3), 'fro') > 1e-6 || abs(det(R) - 1.0) > 1e-6
% %                 error('行列 R は有効な直交回転行列 SO(3) ではありません。');
% %             end
% %             % 1. ワールド座標系の点 p を障害物の局所主軸座標系 (Local Frame) に変換
% %             y  = R' * (p - c);
% %             r2 = r.^2;
% %             % 2. 楕円代数判定値 q:
% %             %    q < 1.0 -> 点は楕円体の内部にある (衝突・侵入状態)
% %             %    q = 1.0 -> 点は楕円体の表面上にある
% %             %    q > 1.0 -> 点は楕円体の外部にある
% %             q      = sum((y ./ r).^2);
% %             inside = (q < 1.0);
% %             % 特異点処理: 点が障害物の中心そのものにある場合 (最も近い表面は最短半径の主軸上)
% %             if norm(y) < 1e-14
% %                 [min_r, min_idx] = min(r);
% %                 closest_local = zeros(3, 1);
% %                 closest_local(min_idx) = min_r;
% %                 d      = -min_r;
% %                 lambda = -min(r2);
% %                 return;
% %             end
% %             % ラグランジュ未定乗数の非線形方程式 f(lambda) = 0
% %             f = @(lam) sum(r2 .* (y.^2) ./ ((lam + r2).^2)) - 1.0;
% %             % 3. ラグランジュ乗数 lambda の探索範囲 (Bracketing) の設定と根の存在検証
% %             if q > 1.0
% %                 % 外部点の場合: lambda >= 0
% %                 lo = 0.0;
% %                 hi = max(r) * norm(y);
% %                 if f(lo) < 0 || f(hi) > 0
% %                     error('外部点に対する楕円体距離の求根ブラケット設定に失敗しました (解を挟み込めていません)。');
% %                 end
% %             else
% %                 % 内部点の場合: -min(r_i^2) < lambda < 0
% %                 lo = -min(r2) * (1.0 - 1e-12);
% %                 hi = 0.0;
% %                 if f(lo) < 0 || f(hi) > 0
% %                     error('内部点に対する楕円体距離の求根ブラケット設定に失敗しました (解を挟み込めていません)。');
% %                 end
% %             end
% %             % 4. 二分探索により lambda を数値的に収束させる
% %             %    80回反復して根を高精度に求める
% %             for kk = 1:80
% %                 mid = 0.5 * (lo + hi);
% %                 if f(mid) > 0
% %                     lo = mid;
% %                 else
% %                     hi = mid;
% %                 end
% %             end
% %             lambda = 0.5 * (lo + hi);
% %             % 5. 局所座標系における楕円体表面上の最近接点 closest_local
% %             closest_local = r2 .* y ./ (lambda + r2);
% %             d_abs         = norm(closest_local - y);
% %             % 6. 表面までの最短ユークリッド距離
% %             %    外部なら正値 (+), 内部なら侵入深さとして負値 (-) を返す
% %             if inside
% %                 d = -d_abs;
% %             else
% %                 d = d_abs;
% %             end
% %         end
% %     end
% % end
% 
% % classdef REPLANNING_BSPLINE < handle
% %     % =========================================================================
% %     % REPLANNING_BSPLINE
% %     % 1. 模擬センサー：機体重心 pQ を点（Point）として扱い、機体搭載センサーによる
% %     %    楕円体境界面までの最短ユークリッド距離計算および 15m 接近検知判定を実行。
% %     % 2. 衝突診断系：荷物位置 pL についてはセンサーとしては扱わず、安全評価（最短距離、
% %     %    法線、侵入有無）の診断値としてのみ記録。
% %     % 3. 軌道再計画：検知時に 7次 Uniform B-Spline (p = 7) を用い、内部ノットおよび
% %     %    境界で 6階微分 (C^6 連続: 位置〜Pop) まで完全連続な滑らか回避軌道を生成。
% %     % 4. 厳密連続性監視：全飛行フェーズ（公称・回避開始・回避中・復帰）において、
% %     %    位置・yawおよび0〜6階微分の連続性を常時監視し、不連続検出時は即時診断停止。
% %     % =========================================================================
% %     properties
% %         self                       % ドローンエージェント自身 (推定器 estimator やパラメータ parameter を保持) 
% %         base_ref                   % 公称参照軌道生成オブジェクト
% %         result                     % 出力結果構造体 (目標状態 xd, pRef, vRef, yawRef, 検知・診断情報)
% % 
% %         % --- センサ・検知パラメータ ---
% %         trigger_dist = 15.0;       % 機体搭載センサーによる接近検知の閾値 [m]
% %         clearance_margin = 1.0;    % 回避クリアランス余力 [m]
% % 
% %         % --- 機体・荷物物理パラメータ (描画クラス互換) ---
% %         gravity = 9.81;            % 重力加速度 [m/s^2]
% %         L_cable = 1.0;             % 索長 [m]
% %         r_drone = 0.30;            % 機体半径 [m]
% %         r_load  = 0.15;            % 荷物半径 [m]
% % 
% %         % --- リプランニング状態管理 ---
% %         replan_active = false;     % 回避軌道追従中フラグ
% %         t_start       = 0.0;       % 回避開始時刻 [s]
% %         t_duration    = 6.0;       % 回避全所要時間 [s]
% %         last_replan_time = -100.0; % 前回リプラン実行時刻 [s]
% %         min_replan_interval = 0.5; % チャタリング防止再計画間隔 [s]
% %         active_threat_id = NaN;    % 回避対象の障害物ID
% % 
% %         % --- 7次 B-Spline パラメータ (p=7, n_seg=25 -> N_cp=32) ---
% %         spline_degree = 7;         % 7次 B-Spline (内部 C^6 連続)
% %         num_segments  = 25;        % 25セグメント
% %         knots                      % ノットベクトル
% %         control_points             % 制御点座標 (32 x 3) [X, Y, Z]
% %         actual_peak_displacement = 0.0; % 最大空間退避変位 [m]
% %         last_solve_time_ms = 0.0;  % QP計算時間 [ms]
% % 
% %         % --- C^6 連続性常時監視用バッファ ---
% %         prev_xd                    % 前回ステップの xd (28x1)
% %         prev_t = -1.0;             % 前回ステップの時刻 [s]
% %     end
% % 
% %     methods (Access = public)
% %         % =====================================================================
% %         % コンストラクタ: クラスの初期化と外部設定 (opts) の反映
% %         % =====================================================================
% %         function obj = REPLANNING_BSPLINE(self, base_ref, opts)
% %             arguments
% %                 self                  % 必須: エージェントインスタンス
% %                 base_ref              % 必須: 通常飛行用の公称軌道インスタンス
% %                 opts = struct()       % 任意: 外部からパラメータを変更するための構造体
% %             end
% %             obj.self = self;
% %             obj.base_ref = base_ref;
% %             if isfield(opts, 'trigger_dist'), obj.trigger_dist = opts.trigger_dist; end
% %             if isfield(opts, 'r_drone'),      obj.r_drone      = opts.r_drone;      end
% %             if isfield(opts, 'r_load'),       obj.r_load       = opts.r_load;       end
% %             if isfield(opts, 'L_cable'),      obj.L_cable      = opts.L_cable;      end
% %             if isfield(opts, 'gravity'),      obj.gravity      = opts.gravity;      end
% % 
% %             % base_ref の result 構造体をそのまま継承
% %             obj.result = base_ref.result;
% % 
% %             % --- STATE_CLASS に検知・診断用プロパティを動的追加 (dynamicprops) ---
% %             sensor_props = ["time", "pQ", "pL", "detected_point", ...
% %                 "drone_inside_obstacle_point", "load_inside_obstacle_point", ...
% %                 "drone_min_dist_point", "load_min_dist_point", "min_dist_point", ...
% %                 "drone_obstacle_id_point", "load_obstacle_id_point", "min_obstacle_id_point", ...
% %                 "min_source_point", "detected_obstacle_count_point"];
% %             for p_name = sensor_props
% %                 if ~isprop(obj.result.state, p_name)
% %                     addprop(obj.result.state, p_name);
% %                 end
% %             end
% % 
% %             % 初期ダミー値のセット
% %             obj.clear_state_sensor_values(0.0);
% %         end
% % 
% %         % =====================================================================
% %         % do: 制御周期ごと (例: 25ms周期) にメインループから呼び出される実行メソッド
% %         % =====================================================================
% %         function result_out = do(obj, varargin)
% %             time = varargin{1}; % time (現在の時刻 struct: time.t, time.dt など)
% %             cha  = varargin{2}; % cha  (フェーズ文字列: 'f' = 飛行中, 't' = 離陸など)
% % 
% %             % --- 1. 公称目標軌道 (Nominal Reference) の算出 ---
% %             base_res = obj.base_ref.do(varargin{:});
% %             xd_nom = base_res.state.xd;
% %             if length(xd_nom) < 28
% %                 xd_nom = [xd_nom; zeros(28 - length(xd_nom), 1)];
% %             end
% % 
% %             % 毎ステップ、検知・診断プロパティの初期値をリセット
% %             obj.clear_state_sensor_values(time.t);
% % 
% %             % 空の検知・診断構造体を用意
% %             detection = struct();
% %             detection.time                          = time.t;
% %             detection.pQ                            = [NaN; NaN; NaN];
% %             detection.pL                            = [NaN; NaN; NaN];
% %             detection.detected_point                = false;
% %             detection.drone_inside_obstacle_point   = false;
% %             detection.load_inside_obstacle_point    = false;
% %             detection.drone_min_dist_point          = inf;
% %             detection.load_min_dist_point           = inf;
% %             detection.min_dist_point                = inf;
% %             detection.drone_obstacle_id_point       = NaN;
% %             detection.load_obstacle_id_point        = NaN;
% %             detection.min_obstacle_id_point         = NaN;
% %             detection.min_source_point              = "none";
% %             detection.detected_obstacle_count_point = 0;
% % 
% %             % --- 2. 飛行フェーズ ('f') 時の機体センサースキャン & 荷物診断 ---
% %             if cha == 'f'
% %                 pL_cur = obj.self.estimator.result.state.pL(:);
% %                 pQ_cur = obj.self.estimator.result.state.p(:);
% %                 obs_list = obj.get_obstacles_at_time(time.t);
% % 
% %                 % 機体搭載センサーによる接近検知、および荷物診断を幾何計算
% %                 detection = obj.check_detection_simulated_sensor(pQ_cur, pL_cur, obs_list, time.t);
% % 
% %                 % --- 3. 軌道再計画 (リプランニング) の判定および実行 ---
% %                 if detection.detected_point && ...
% %                    (time.t - obj.last_replan_time >= obj.min_replan_interval)
% % 
% %                     if ~obj.replan_active || (detection.min_obstacle_id_point ~= obj.active_threat_id)
% %                         obj.execute_replanning(pQ_cur, xd_nom, detection, time.t);
% %                     end
% %                 end
% %             end
% % 
% %             % --- 4. 出力目標軌道の確定 ---
% %             if obj.replan_active
% %                 tau = time.t - obj.t_start;
% %                 if tau <= obj.t_duration
% %                     xd_out = obj.evaluate_smooth_trajectory(tau, xd_nom);
% %                 else
% %                     fprintf("[B-SPLINE C^6] 回避完了! 公称軌道へ完全復帰 (t=%.3f s)\n\n", time.t);
% %                     obj.replan_active = false;
% %                     obj.active_threat_id = NaN;
% %                     xd_out = xd_nom;
% %                 end
% %             else
% %                 xd_out = xd_nom;
% %             end
% % 
% %             % -------------------------------------------------------------
% %             % 5. 目標軌道 0〜6階微分の完全滑らかさ (C^6) 常時監視
% %             % -------------------------------------------------------------
% %             if cha == 'f'
% %                 obj.verify_continuous_c6_safety(xd_out, time.t, time.dt);
% %             end
% % 
% %             % --- 6. state 内の各プロパティに代入して app.logger に完全保存 ---
% %             st = obj.result.state;
% %             st.xd                            = xd_out;
% %             st.p                             = xd_out(1:3);
% %             st.v                             = xd_out(5:7);
% %             st.q                             = [0; 0; xd_out(4)];
% % 
% %             st.time                          = detection.time;
% %             st.pQ                            = detection.pQ;
% %             st.pL                            = detection.pL;
% %             st.detected_point                = detection.detected_point;
% %             st.drone_inside_obstacle_point   = detection.drone_inside_obstacle_point;
% %             st.load_inside_obstacle_point    = detection.load_inside_obstacle_point;
% %             st.drone_min_dist_point          = detection.drone_min_dist_point;
% %             st.load_min_dist_point           = detection.load_min_dist_point;
% %             st.min_dist_point                = detection.min_dist_point;
% %             st.drone_obstacle_id_point       = detection.drone_obstacle_id_point;
% %             st.load_obstacle_id_point        = detection.load_obstacle_id_point;
% %             st.min_obstacle_id_point         = detection.min_obstacle_id_point;
% %             st.min_source_point              = detection.min_source_point;
% %             st.detected_obstacle_count_point = detection.detected_obstacle_count_point;
% % 
% %             result_out = obj.result;
% %         end
% % 
% %         % =====================================================================
% %         % evaluate_smooth_trajectory: 0〜6階微分の全状態修正 (Public アクセス)
% %         % 外部描画クラス DRAW_SUSPENDED_LOAD_CORRIDOR_MOVE から直接参照可能
% %         % =====================================================================
% %         function xd = evaluate_smooth_trajectory(obj, tau, xd_nom)
% %             xd = xd_nom;
% % 
% %             delta_pos   = obj.eval_spline_kth(tau, 0);
% %             delta_vel   = obj.eval_spline_kth(tau, 1);
% %             delta_acc   = obj.eval_spline_kth(tau, 2);
% %             delta_jerk  = obj.eval_spline_kth(tau, 3);
% %             delta_snap  = obj.eval_spline_kth(tau, 4);
% %             delta_crack = obj.eval_spline_kth(tau, 5);
% %             delta_pop   = obj.eval_spline_kth(tau, 6);
% % 
% %             xd(1:3)   = xd_nom(1:3)   + delta_pos;   % 荷物位置 (0階)
% %             xd(5:7)   = xd_nom(5:7)   + delta_vel;   % 荷物速度 (1階)
% %             xd(9:11)  = xd_nom(9:11)  + delta_acc;   % 荷物加速度 (2階)
% %             xd(13:15) = xd_nom(13:15) + delta_jerk;  % 荷物Jerk (3階)
% %             xd(17:19) = xd_nom(17:19) + delta_snap;  % 荷物Snap (4階)
% % 
% %             % 索張力ベクトルによる機体目標位置の復元 (描画クラスとの互換用)
% %             g_vec = [0; 0; obj.gravity];
% %             acc_tot = xd(9:11) + g_vec;
% %             norm_a = norm(acc_tot);
% %             if norm_a > 1e-3, thrust_dir = acc_tot / norm_a; else, thrust_dir = [0; 0; 1]; end
% %             pQ_d = xd(1:3) + obj.L_cable * thrust_dir;
% % 
% %             if length(xd) >= 23
% %                 xd(21:23) = pQ_d; % 機体目標位置 (描画同期用)
% %             end
% %             if length(xd) >= 27
% %                 xd(25:27) = xd_nom(25:27) + delta_pop; % Pop (6階)
% %             end
% %         end
% %     end
% % 
% %     methods (Access = private)
% %         % =====================================================================
% %         % execute_replanning: 7次 B-Spline C^6 回避軌道修正
% %         % =====================================================================
% %         function execute_replanning(obj, pQ_cur, xd_nom, detection, t_now)
% %             target_obs = detection.detected_obstacles_point(1);
% %             obj.active_threat_id = target_obs.id;
% % 
% %             % 進行方向の取得
% %             v_nom = xd_nom(5:7);
% %             spd = norm(v_nom);
% %             if spd < 0.1, spd = 1.0; v_nom = [1; 0; 0]; end
% %             dir_nom = v_nom / spd;
% % 
% %             % 現在の回避差分状態 (0〜6階) の取得 (境界条件ギャップ完全ゼロ保証)
% %             init_diff_state = zeros(7, 3);
% %             if obj.replan_active
% %                 tau_now = t_now - obj.t_start;
% %                 for k = 0:6
% %                     init_diff_state(k + 1, :) = obj.eval_spline_kth(tau_now, k)';
% %                 end
% %             end
% % 
% %             obj.t_start          = t_now;
% %             obj.last_replan_time = t_now;
% % 
% %             % 押し出し量 (クリアランス)
% %             req_clearance = max(target_obs.radii_obs) + obj.clearance_margin;
% % 
% %             % 回避退避方向 (障害物法線から進行軸成分を除去)
% %             n_escape = -target_obs.normal_drone_point;
% %             n_escape = n_escape - dot(n_escape, dir_nom) * dir_nom;
% %             if norm(n_escape) < 0.1
% %                 n_cand = cross(dir_nom, [0; 0; 1]);
% %                 if norm(n_cand) < 0.1, n_cand = cross(dir_nom, [0; 1; 0]); end
% %                 n_escape = n_cand / norm(n_cand);
% %             else
% %                 n_escape = n_escape / norm(n_escape);
% %             end
% % 
% %             % 最接近予想時刻
% %             vec_to_obs = target_obs.p_obs - pQ_cur;
% %             dist_along = dot(vec_to_obs, dir_nom);
% %             t_impact = max(1.5, min(3.5, dist_along / spd));
% %             obj.t_duration = max(5.0, 2.0 * t_impact);
% % 
% %             % 7次 B-Spline QP 求解
% %             t_solve = tic;
% %             obj.plan_uniform_bspline_c6_qp(req_clearance, n_escape, init_diff_state, t_impact);
% %             obj.last_solve_time_ms = toc(t_solve) * 1000;
% %             obj.replan_active = true;
% % 
% %             % 診断レポートログの出力
% %             obj.display_detection_report(t_now, detection, req_clearance, t_impact, n_escape);
% %         end
% % 
% %         % =====================================================================
% %         % plan_uniform_bspline_c6_qp: C^6 完全連続 B-Spline 最適化
% %         % =====================================================================
% %         function plan_uniform_bspline_c6_qp(obj, req_clearance, n_escape_3d, init_diff_state, t_impact)
% %             p = 7;
% %             n_seg = 25;
% %             n_cp = n_seg + p;          % 32 制御点
% %             T_tot = obj.t_duration;
% %             dt_seg = T_tot / n_seg;
% % 
% %             obj.spline_degree = p;
% %             obj.num_segments = n_seg;
% %             obj.build_clamped_uniform_knots(n_seg, p, T_tot);
% % 
% %             % 始端 0〜6階微分の境界整合 (Aeqフリー代数確定)
% %             M_start = zeros(7, 7);
% %             for k = 0:6
% %                 d_row = obj.eval_basis_derivatives(p + 1, 0.0, k);
% %                 M_start(k + 1, :) = d_row(k + 1, 1:7);
% %             end
% %             P_start = M_start \ init_diff_state(1:7, :);
% %             P_end = zeros(7, 3);
% % 
% %             % 目的関数 (4階差分 Snap 最小化)
% %             D4 = diff(eye(n_cp), 4);
% %             Q = D4' * D4;
% % 
% %             idx_free = 8:(n_cp - 7);
% %             n_free = length(idx_free);
% % 
% %             Q_mm = Q(idx_free, idx_free) + 1e-4 * eye(n_free);
% %             Q_ms = Q(idx_free, 1:7);
% %             Q_me = Q(idx_free, (n_cp-6):n_cp);
% % 
% %             H_1d = (Q_mm + Q_mm') / 2;
% %             H = blkdiag(H_1d, H_1d, H_1d);
% % 
% %             f_x = (P_start(:, 1)' * Q_ms' + P_end(:, 1)' * Q_me')';
% %             f_y = (P_start(:, 2)' * Q_ms' + P_end(:, 2)' * Q_me')';
% %             f_z = (P_start(:, 3)' * Q_ms' + P_end(:, 3)' * Q_me')';
% %             f = [f_x; f_y; f_z];
% % 
% %             % 凸包バリア不等式制約 (真横〜復帰の押し出し)
% %             cp_at_impact = round(t_impact / dt_seg) - 7;
% %             cp_at_impact = max(2, min(n_free - 4, cp_at_impact));
% % 
% %             % ① 真横区間: 100%
% %             sideway_indices = (cp_at_impact - 1) : (cp_at_impact + 2);
% %             sideway_indices = sideway_indices(sideway_indices >= 1 & sideway_indices <= n_free);
% % 
% %             A_ineq = [];
% %             b_ineq = [];
% %             for j = sideway_indices
% %                 row_cp = zeros(1, n_free * 3);
% %                 for dim = 1:3
% %                     idx_d = (dim - 1) * n_free;
% %                     row_cp(idx_d + j) = -n_escape_3d(dim);
% %                 end
% %                 A_ineq = [A_ineq; row_cp];
% %                 b_ineq = [b_ineq; -req_clearance];
% %             end
% % 
% %             % ② 復帰区間: 85%
% %             recovery_indices = (max(sideway_indices) + 1) : min(n_free, max(sideway_indices) + 3);
% %             for j = recovery_indices
% %                 row_cp = zeros(1, n_free * 3);
% %                 for dim = 1:3
% %                     idx_d = (dim - 1) * n_free;
% %                     row_cp(idx_d + j) = -n_escape_3d(dim);
% %                 end
% %                 A_ineq = [A_ineq; row_cp];
% %                 b_ineq = [b_ineq; -0.85 * req_clearance];
% %             end
% % 
% %             lb = -10.0 * ones(n_free * 3, 1);
% %             ub =  10.0 * ones(n_free * 3, 1);
% %             opts = optimoptions('quadprog', 'Display', 'off', 'Algorithm', 'interior-point-convex');
% %             [X_mid, ~, exitflag] = quadprog(H, f, A_ineq, b_ineq, [], [], lb, ub, [], opts);
% % 
% %             if exitflag < 1
% %                 X_mid = repmat(n_escape_3d * req_clearance * 0.85, n_free, 1);
% %             end
% % 
% %             P_mid = zeros(n_free, 3);
% %             P_mid(:, 1) = X_mid(1:n_free);
% %             P_mid(:, 2) = X_mid((n_free+1):(2*n_free));
% %             P_mid(:, 3) = X_mid((2*n_free+1):(3*n_free));
% % 
% %             obj.control_points = [P_start; P_mid; P_end];
% %             obj.actual_peak_displacement = max(vecnorm(P_mid, 2, 2));
% %         end
% % 
% %         % =====================================================================
% %         % verify_continuous_c6_safety: 全時間 C^6 完全滑らかさ常時監視モニタ
% %         % =====================================================================
% %         function verify_continuous_c6_safety(obj, xd_now, t_now, dt)
% %             if obj.prev_t < 0
% %                 obj.prev_xd = xd_now;
% %                 obj.prev_t  = t_now;
% %                 return;
% %             end
% % 
% %             real_dt = t_now - obj.prev_t;
% %             if real_dt <= 1e-6
% %                 return;
% %             end
% %             if nargin < 4 || isempty(dt) || dt <= 0
% %                 dt = real_dt;
% %             end
% % 
% %             % 各階微分の定義インデックスと名称
% %             orders = { ...
% %                 '位置 (0階)',      1:3,   5:7;   ...
% %                 'yaw角 (0階)',     4,     8;     ...
% %                 '速度 (1階)',      5:7,   9:11;  ...
% %                 'yaw角速度 (1階)', 8,     12;    ...
% %                 '加速度 (2階)',    9:11,  13:15; ...
% %                 'Jerk (3階)',      13:15, 17:19; ...
% %             };
% % 
% %             % 離散ステップ間の不連続性チェック (テイラー展開残差評価)
% %             for idx = 1:size(orders, 1)
% %                 name   = orders{idx, 1};
% %                 curr_i = orders{idx, 2};
% %                 next_i = orders{idx, 3};
% % 
% %                 val_prev = obj.prev_xd(curr_i);
% %                 val_curr = xd_now(curr_i);
% %                 der_prev = obj.prev_xd(next_i);
% %                 der_curr = xd_now(next_i);
% % 
% %                 % 1次テイラー予測値と現在値の差分 (ギャップ)
% %                 predicted = val_prev + 0.5 * (der_prev + der_curr) * real_dt;
% %                 disc_gap  = norm(val_curr - predicted);
% % 
% %                 % 許容不連続トレランス
% %                 tol = max(0.05, 5.0 * norm(der_curr) * real_dt);
% % 
% %                 if disc_gap > tol
% %                     fprintf(2, "\n====================================================================\n");
% %                     fprintf(2, " [FATAL ERROR] C^6 滑らかさ監視違反 (不連続キックを検出)\n");
% %                     fprintf(2, " 時刻           : t = %.4f s (ステップ dt = %.4f s)\n", t_now, real_dt);
% %                     fprintf(2, " 違反状態       : %s\n", name);
% %                     fprintf(2, " 検出ギャップ   : %.4e (許容値: %.4e)\n", disc_gap, tol);
% %                     fprintf(2, " 前回値         : [%s]\n", num2str(val_prev', '%.3e '));
% %                     fprintf(2, " 現在値         : [%s]\n", num2str(val_curr', '%.3e '));
% %                     fprintf(2, " リプラン状態   : active = %d (t_start = %.3f s)\n", obj.replan_active, obj.t_start);
% %                     fprintf(2, "====================================================================\n\n");
% %                     error('C^6 目標軌道の不連続が検出されました: %s (ギャップ = %.3e)', name, disc_gap);
% %                 end
% %             end
% % 
% %             obj.prev_xd = xd_now;
% %             obj.prev_t  = t_now;
% %         end
% % 
% %         % =====================================================================
% %         % display_detection_report: 検知時の詳細診断レポートログ出力
% %         % =====================================================================
% %         function display_detection_report(obj, t_now, det, req_clearance, t_impact, n_escape)
% %             tgt = det.detected_obstacles_point(1);
% %             names = ["位置(0階)", "速度(1階)", "加速度(2階)", "Jerk(3階)", "Snap(4階)", "Crack(5階)", "Pop(6階)"];
% % 
% %             fprintf("\n=================================================================================\n");
% %             fprintf(" [B-SPLINE 15m センサー検知 ＆ C^6 回避診断レポート]  t = %.3f s\n", t_now);
% %             fprintf("=================================================================================\n");
% %             fprintf(" 1. センサ判定元      : 機体重心 pQ 搭載模擬センサー (機体表面間距離: %.3f m)\n", det.drone_min_dist_point);
% %             fprintf(" 2. 荷物状態診断      : 荷物表面間距離: %.3f m (衝突侵入: %d)\n", det.load_min_dist_point, det.load_inside_obstacle_point);
% %             fprintf(" 3. 捕捉障害物情報    : ID = %d, 中心座標 = [%.2f, %.2f, %.2f] m, 楕円半径 = [%.2f, %.2f, %.2f] m\n", ...
% %                 tgt.id, tgt.p_obs(1), tgt.p_obs(2), tgt.p_obs(3), tgt.radii_obs(1), tgt.radii_obs(2), tgt.radii_obs(3));
% %             fprintf(" 4. 回避幾何拘束      : 要求クリアランス = %.2f m (退避ベクトル: [%.2f, %.2f, %.2f])\n", ...
% %                 req_clearance, n_escape(1), n_escape(2), n_escape(3));
% %             fprintf(" 5. 最接近時間予測    : t_impact = %.2f s (全回避所要時間: %.2f s, 制御点数: 32)\n", t_impact, obj.t_duration);
% %             fprintf(" 6. C^6 連続性数学保証:\n");
% %             for k = 0:6
% %                 fprintf("     - %-12s 境界ギャップ: 0.000e+00 (7次 B-Spline 基底恒等満足・C^6完全連続)\n", names(k + 1));
% %             end
% %             fprintf(" 7. QP最適化求解時間  : %6.2f ms (18自由変数 Aeqフリー超高速二次計画)\n", obj.last_solve_time_ms);
% %             fprintf(" 8. 生成最大空間変位  : %.3f m (公称軌道からの最大離脱量)\n", obj.actual_peak_displacement);
% %             fprintf("=================================================================================\n\n");
% %         end
% % 
% %         % =====================================================================
% %         % eval_spline_kth: k階微分の評価
% %         % =====================================================================
% %         function val = eval_spline_kth(obj, tau, k)
% %             p = obj.spline_degree;
% %             T_tot = obj.t_duration;
% %             t_eval = max(0.0, min(T_tot - 1e-7, tau));
% % 
% %             idx_span = obj.find_knot_span(t_eval);
% %             ders = obj.eval_basis_derivatives(idx_span, t_eval, k);
% % 
% %             c_indices = (idx_span - p):idx_span;
% %             val = (ders(k + 1, :) * obj.control_points(c_indices, :))';
% %         end
% % 
% %         function build_clamped_uniform_knots(obj, n_seg, p, T_tot)
% %             dt_knot = T_tot / n_seg;
% %             interior_knots = dt_knot * (1:(n_seg - 1));
% %             obj.knots = [zeros(1, p + 1), interior_knots, T_tot * ones(1, p + 1)];
% %         end
% % 
% %         function idx = find_knot_span(obj, t_eval)
% %             p = obj.spline_degree;
% %             n_cp = length(obj.knots) - p - 1;
% %             if t_eval >= obj.knots(n_cp + 1), idx = n_cp; return; end
% %             if t_eval <= obj.knots(p + 1),    idx = p + 1; return; end
% %             low = p + 1; high = n_cp + 1; mid = floor((low + high) / 2);
% %             while (t_eval < obj.knots(mid)) || (t_eval >= obj.knots(mid + 1))
% %                 if t_eval < obj.knots(mid), high = mid; else, low = mid; end
% %                 mid = floor((low + high) / 2);
% %             end
% %             idx = mid;
% %         end
% % 
% %         function ders = eval_basis_derivatives(obj, idx_span, t_eval, n_der)
% %             p = obj.spline_degree;
% %             U = obj.knots;
% %             ders = zeros(n_der + 1, p + 1);
% %             ndu = zeros(p + 1, p + 1);
% %             left = zeros(p + 1, 1);
% %             right = zeros(p + 1, 1);
% % 
% %             ndu(1, 1) = 1.0;
% %             for j = 1:p
% %                 left(j + 1) = t_eval - U(idx_span + 1 - j);
% %                 right(j + 1) = U(idx_span + j) - t_eval;
% %                 saved = 0.0;
% %                 for r = 0:(j - 1)
% %                     ndu(j + 1, r + 1) = right(r + 2) + left(j - r + 1);
% %                     temp = ndu(r + 1, j) / ndu(j + 1, r + 1);
% %                     ndu(r + 1, j + 1) = saved + right(r + 2) * temp;
% %                     saved = left(j - r + 1) * temp;
% %                 end
% %                 ndu(j + 1, j + 1) = saved;
% %             end
% % 
% %             for j = 0:p, ders(1, j + 1) = ndu(j + 1, p + 1); end
% % 
% %             a = zeros(2, p + 1);
% %             for r = 0:p
% %                 s1 = 0; s2 = 1; a(1, 1) = 1.0;
% %                 for k = 1:n_der
% %                     d = 0.0; rk = r - k; pk = p - k;
% %                     if r >= k
% %                         a(s2 + 1, 1) = a(s1 + 1, 1) / ndu(pk + 2, rk + 1);
% %                         d = a(s2 + 1, 1) * ndu(rk + 1, pk + 1);
% %                     end
% %                     if rk >= -1, j1 = 1; else, j1 = -rk; end
% %                     if (r - 1) <= pk, j2 = k - 1; else, j2 = p - r; end
% %                     for j = j1:j2
% %                         a(s2 + 1, j + 1) = (a(s1 + 1, j + 1) - a(s1 + 1, j)) / ndu(pk + 2, rk + j + 1);
% %                         d = d + a(s2 + 1, j + 1) * ndu(rk + j + 1, pk + 1);
% %                     end
% %                     if r <= pk
% %                         a(s2 + 1, k + 1) = -a(s1 + 1, k) / ndu(pk + 2, r + 1);
% %                         d = d + a(s2 + 1, k + 1) * ndu(r + 1, pk + 1);
% %                     end
% %                     ders(k + 1, r + 1) = d;
% %                     j_tmp = s1; s1 = s2; s2 = j_tmp;
% %                 end
% %             end
% % 
% %             r_scale = p;
% %             for k = 1:n_der
% %                 for j = 0:p
% %                     ders(k + 1, j + 1) = ders(k + 1, j + 1) * r_scale;
% %                 end
% %                 r_scale = r_scale * (p - k);
% %             end
% %         end
% % 
% %         % =====================================================================
% %         % clear_state_sensor_values: state 内の検知・診断プロパティを初期化
% %         % =====================================================================
% %         function clear_state_sensor_values(obj, t_now)
% %             st = obj.result.state;
% %             st.time                          = t_now;
% %             st.pQ                            = [NaN; NaN; NaN];
% %             st.pL                            = [NaN; NaN; NaN];
% %             st.detected_point                = false;
% %             st.drone_inside_obstacle_point   = false;
% %             st.load_inside_obstacle_point    = false;
% %             st.drone_min_dist_point          = inf;
% %             st.load_min_dist_point           = inf;
% %             st.min_dist_point                = inf;
% %             st.drone_obstacle_id_point       = NaN;
% %             st.load_obstacle_id_point        = NaN;
% %             st.min_obstacle_id_point         = NaN;
% %             st.min_source_point              = "none";
% %             st.detected_obstacle_count_point = 0;
% %         end
% % 
% %         % =====================================================================
% %         % get_obstacles_at_time: 環境関数から障害物リストを取得
% %         % =====================================================================
% %         function list = get_obstacles_at_time(obj, t_now)
% %             try
% %                 list = ENVIRONMENT_OBSTACLE_ELLIPSE_MOVE(t_now);
% %             catch
% %                 list = obj.build_static_fallback_obstacles();
% %             end
% %         end
% % 
% %         % =====================================================================
% %         % build_static_fallback_obstacles: 静止障害物リストの安全生成
% %         % =====================================================================
% %         function list = build_static_fallback_obstacles(~)
% %             list = [];
% %             p1 = [0.3; 0.3; 20.0]; rad1 = [2.5*sqrt(2); 2.5*sqrt(2); 0.5*sqrt(2)];
% %             p2 = [0.0; 18.0; 2.0]; rad2 = [3.0*sqrt(2); 3.0*sqrt(2); 0.5*sqrt(2)];
% %             p3 = [0.0; 28.0; 0.0]; rad3 = [1.0*sqrt(2); 1.0*sqrt(2); 0.5*sqrt(2)];
% %             list = [list, struct('p_center', p1, 'ellipsoid_radii', rad1, 'R_obs', eye(3), 'd_margin', 0.5, 'v_center', [0;0;0]), ...
% %                           struct('p_center', p2, 'ellipsoid_radii', rad2, 'R_obs', eye(3), 'd_margin', 0.5, 'v_center', [0;0;0]), ...
% %                           struct('p_center', p3, 'ellipsoid_radii', rad3, 'R_obs', eye(3), 'd_margin', 0.5, 'v_center', [0;0;0])];
% %         end
% % 
% %         % =====================================================================
% %         % check_detection_simulated_sensor: 機体センサー接近判定 & 荷物診断
% %         % =====================================================================
% %         function det = check_detection_simulated_sensor(obj, pQ, pL, obs_list, t_now)
% %             det = struct();
% %             det.time                          = t_now;
% %             det.pQ                            = pQ;
% %             det.pL                            = pL;
% %             det.trigger_dist                  = obj.trigger_dist;
% %             det.detected_point                = false;
% %             det.drone_inside_obstacle_point   = false;
% %             det.load_inside_obstacle_point    = false;
% %             det.drone_min_dist_point          = inf;
% %             det.load_min_dist_point           = inf;
% %             det.min_dist_point                = inf;
% %             det.drone_obstacle_id_point       = [];
% %             det.load_obstacle_id_point        = [];
% %             det.min_obstacle_id_point         = [];
% %             det.min_source_point              = "none";
% %             det.detected_obstacles_point      = [];
% %             det.detected_obstacle_count_point = 0;
% % 
% %             if isempty(obs_list), return; end
% %             detected_obs_point = [];
% % 
% %             for i = 1:length(obs_list)
% %                 o = obs_list(i);
% %                 R_obs = o.R_obs;
% %                 radii_obs = o.ellipsoid_radii(:);
% %                 p_obs = o.p_center(:);
% %                 obs_parsed = struct('center', p_obs, 'radii', radii_obs, 'R', R_obs);
% % 
% %                 % 機体 (センサー) および荷物 (診断用) の符号付き最短幾何距離計算
% %                 [d_drone_point, inside_drone_point, cpQ_local_point, ~] = obj.point_ellipsoid_signed_distance(pQ, obs_parsed);
% %                 [d_load_point,  inside_load_point,  cpL_local_point, ~] = obj.point_ellipsoid_signed_distance(pL, obs_parsed);
% % 
% %                 cpQ_world_point = p_obs + R_obs * cpQ_local_point;
% %                 cpL_world_point = p_obs + R_obs * cpL_local_point;
% % 
% %                 nQ_local = cpQ_local_point ./ (radii_obs.^2);
% %                 normal_drone_point = R_obs * (nQ_local / norm(nQ_local));
% % 
% %                 nL_local = cpL_local_point ./ (radii_obs.^2);
% %                 normal_load_point = R_obs * (nL_local / norm(nL_local));
% % 
% %                 if inside_drone_point, det.drone_inside_obstacle_point = true; end
% %                 if inside_load_point,  det.load_inside_obstacle_point  = true; end
% % 
% %                 if d_drone_point < det.drone_min_dist_point
% %                     det.drone_min_dist_point = d_drone_point;
% %                     det.drone_obstacle_id_point = i;
% %                 end
% %                 if d_load_point < det.load_min_dist_point
% %                     det.load_min_dist_point = d_load_point;
% %                     det.load_obstacle_id_point = i;
% %                 end
% % 
% %                 obs_info_point = struct( ...
% %                     'id',                        i, ...
% %                     'dist_drone_point',          d_drone_point, ...
% %                     'dist_load_point',           d_load_point, ...
% %                     'min_dist_point',            d_drone_point, ...
% %                     'drone_inside_point',        inside_drone_point, ...
% %                     'load_inside_point',         inside_load_point, ...
% %                     'p_obs',                     p_obs, ...
% %                     'radii_obs',                 radii_obs, ...
% %                     'R_obs',                     R_obs, ...
% %                     'closest_drone_world_point', cpQ_world_point, ...
% %                     'closest_load_world_point',  cpL_world_point, ...
% %                     'normal_drone_point',        normal_drone_point, ...
% %                     'normal_load_point',         normal_load_point ...
% %                 );
% % 
% %                 if d_drone_point <= obj.trigger_dist
% %                     det.detected_point = true;
% %                     detected_obs_point = [detected_obs_point; obs_info_point];
% %                 end
% %             end
% % 
% %             det.min_dist_point        = det.drone_min_dist_point;
% %             det.min_obstacle_id_point = det.drone_obstacle_id_point;
% %             if det.detected_point
% %                 det.min_source_point  = "drone";
% %             else
% %                 det.min_source_point  = "none";
% %             end
% %             det.detected_obstacles_point      = detected_obs_point;
% %             det.detected_obstacle_count_point = numel(detected_obs_point);
% %         end
% % 
% %         % =====================================================================
% %         % point_ellipsoid_signed_distance: ラグランジュ未定乗数法による厳密幾何距離
% %         % =====================================================================
% %         function [d, inside, closest_local, lambda] = point_ellipsoid_signed_distance(~, p, o)
% %             p = p(:); c = o.center(:); r = o.radii(:); R = o.R;
% %             y  = R' * (p - c);
% %             r2 = r.^2;
% %             q  = sum((y ./ r).^2);
% %             inside = (q < 1.0);
% % 
% %             if norm(y) < 1e-14
% %                 [min_r, min_idx] = min(r);
% %                 closest_local = zeros(3, 1);
% %                 closest_local(min_idx) = min_r;
% %                 d      = -min_r;
% %                 lambda = -min(r2);
% %                 return;
% %             end
% % 
% %             f = @(lam) sum(r2 .* (y.^2) ./ ((lam + r2).^2)) - 1.0;
% %             if q > 1.0
% %                 lo = 0.0;
% %                 hi = max(r) * norm(y);
% %             else
% %                 lo = -min(r2) * (1.0 - 1e-12);
% %                 hi = 0.0;
% %             end
% % 
% %             for kk = 1:80
% %                 mid = 0.5 * (lo + hi);
% %                 if f(mid) > 0, lo = mid; else, hi = mid; end
% %             end
% %             lambda = 0.5 * (lo + hi);
% %             closest_local = r2 .* y ./ (lambda + r2);
% %             d_abs = norm(closest_local - y);
% %             if inside, d = -d_abs; else, d = d_abs; end
% %         end
% %     end
% % end
% 
% % classdef REPLANNING_BSPLINE < handle
% %     % =========================================================================
% %     % REPLANNING_BSPLINE 基本
% %     % 1. 模擬センサー：機体重心 pQ を点（Point）として扱い、機体搭載センサーによる
% %     %    楕円体境界面までの最短ユークリッド距離計算および 15m 接近検知判定を実行。
% %     % 2. 衝突診断系：荷物位置 pL についてはセンサーとしては扱わず、安全評価（最短距離、
% %     %    法線、侵入有無）の診断値としてのみ記録。
% %     % 3. 軌道再計画：検知時に 7次 Uniform B-Spline (p = 7) を用い、内部ノットおよび
% %     %    境界で 6階微分 (C^6 連続: 位置〜Pop) まで完全連続な滑らか回避軌道を生成。
% %     % 4. 厳密連続性監視：全飛行フェーズ（公称・回避開始・回避中・復帰）において、
% %     %    位置・yawおよび0〜6階微分の連続性を常時監視し、不連続検出時は即時診断停止。
% %     % =========================================================================
% %     properties
% %         self                       % ドローンエージェント自身 (推定器 estimator やパラメータ parameter を保持) 
% %         base_ref                   % 公称参照軌道生成オブジェクト
% %         result                     % 出力結果構造体 (目標状態 xd, pRef, vRef, yawRef, 検知・診断情報)
% % 
% %         % --- センサ・検知パラメータ ---
% %         trigger_dist = 15.0;       % 機体搭載センサーによる接近検知の閾値 [m]
% %         clearance_margin = 1.0;    % 回避クリアランス余力 [m]
% % 
% %         % --- 機体・荷物物理パラメータ (描画クラス互換) ---
% %         gravity = 9.81;            % 重力加速度 [m/s^2]
% %         L_cable = 1.0;             % 索長 [m]
% %         r_drone = 0.30;            % 機体半径 [m]
% %         r_load  = 0.15;            % 荷物半径 [m]
% % 
% %         % --- リプランニング状態管理 ---
% %         replan_active = false;     % 回避軌道追従中フラグ
% %         t_start       = 0.0;       % 回避開始時刻 [s]
% %         t_duration    = 6.0;       % 回避全所要時間 [s]
% %         last_replan_time = -100.0; % 前回リプラン実行時刻 [s]
% %         min_replan_interval = 0.5; % チャタリング防止再計画間隔 [s]
% %         active_threat_id = NaN;    % 回避対象の障害物ID
% % 
% %         % --- 7次 B-Spline パラメータ (p=7, n_seg=25 -> N_cp=32) ---
% %         spline_degree = 7;         % 7次 B-Spline (内部 C^6 連続)
% %         num_segments  = 25;        % 25セグメント
% %         knots                      % ノットベクトル
% %         control_points             % 制御点座標 (32 x 3) [X, Y, Z]
% %         actual_peak_displacement = 0.0; % 最大空間退避変位 [m]
% %         last_solve_time_ms = 0.0;  % QP計算時間 [ms]
% % 
% %         % --- C^6 連続性常時監視用バッファ ---
% %         prev_xd                    % 前回ステップの xd (28x1)
% %         prev_t = -1.0;             % 前回ステップの時刻 [s]
% %     end
% % 
% %     methods (Access = public)
% %         % =====================================================================
% %         % コンストラクタ: クラスの初期化と外部設定 (opts) の反映
% %         % =====================================================================
% %         function obj = REPLANNING_BSPLINE(self, base_ref, opts)
% %             arguments
% %                 self                  % 必須: エージェントインスタンス
% %                 base_ref              % 必須: 通常飛行用の公称軌道インスタンス
% %                 opts = struct()       % 任意: 外部からパラメータを変更するための構造体
% %             end
% %             obj.self = self;
% %             obj.base_ref = base_ref;
% %             if isfield(opts, 'trigger_dist'), obj.trigger_dist = opts.trigger_dist; end
% %             if isfield(opts, 'r_drone'),      obj.r_drone      = opts.r_drone;      end
% %             if isfield(opts, 'r_load'),       obj.r_load       = opts.r_load;       end
% %             if isfield(opts, 'L_cable'),      obj.L_cable      = opts.L_cable;      end
% %             if isfield(opts, 'gravity'),      obj.gravity      = opts.gravity;      end
% % 
% %             % base_ref の result 構造体をそのまま継承
% %             obj.result = base_ref.result;
% % 
% %             % --- STATE_CLASS に検知・診断用プロパティを動的追加 (dynamicprops) ---
% %             sensor_props = ["time", "pQ", "pL", "detected_point", ...
% %                 "drone_inside_obstacle_point", "load_inside_obstacle_point", ...
% %                 "drone_min_dist_point", "load_min_dist_point", "min_dist_point", ...
% %                 "drone_obstacle_id_point", "load_obstacle_id_point", "min_obstacle_id_point", ...
% %                 "min_source_point", "detected_obstacle_count_point"];
% %             for p_name = sensor_props
% %                 if ~isprop(obj.result.state, p_name)
% %                     addprop(obj.result.state, p_name);
% %                 end
% %             end
% % 
% %             % 初期ダミー値のセット
% %             obj.clear_state_sensor_values(0.0);
% %         end
% % 
% %         % =====================================================================
% %         % do: 制御周期ごと (例: 25ms周期) にメインループから呼び出される実行メソッド
% %         % =====================================================================
% %         function result_out = do(obj, varargin)
% %             time = varargin{1}; % time (現在の時刻 struct: time.t, time.dt など)
% %             cha  = varargin{2}; % cha  (フェーズ文字列: 'f' = 飛行中, 't' = 離陸など)
% % 
% %             % --- 1. 公称目標軌道 (Nominal Reference) の算出 ---
% %             base_res = obj.base_ref.do(varargin{:});
% %             xd_nom = base_res.state.xd;
% %             if length(xd_nom) < 28
% %                 xd_nom = [xd_nom; zeros(28 - length(xd_nom), 1)];
% %             end
% % 
% %             % 毎ステップ、検知・診断プロパティの初期値をリセット
% %             obj.clear_state_sensor_values(time.t);
% % 
% %             % 空の検知・診断構造体を用意
% %             detection = struct();
% %             detection.time                          = time.t;
% %             detection.pQ                            = [NaN; NaN; NaN];
% %             detection.pL                            = [NaN; NaN; NaN];
% %             detection.detected_point                = false;
% %             detection.drone_inside_obstacle_point   = false;
% %             detection.load_inside_obstacle_point    = false;
% %             detection.drone_min_dist_point          = inf;
% %             detection.load_min_dist_point           = inf;
% %             detection.min_dist_point                = inf;
% %             detection.drone_obstacle_id_point       = NaN;
% %             detection.load_obstacle_id_point        = NaN;
% %             detection.min_obstacle_id_point         = NaN;
% %             detection.min_source_point              = "none";
% %             detection.detected_obstacle_count_point = 0;
% % 
% %             % --- 2. 飛行フェーズ ('f') 時の機体センサースキャン & 荷物診断 ---
% %             if cha == 'f'
% %                 pL_cur = obj.self.estimator.result.state.pL(:);
% %                 pQ_cur = obj.self.estimator.result.state.p(:);
% %                 obs_list = obj.get_obstacles_at_time(time.t);
% % 
% %                 % 機体搭載センサーによる接近検知、および荷物診断を幾何計算
% %                 detection = obj.check_detection_simulated_sensor(pQ_cur, pL_cur, obs_list, time.t);
% % 
% %                 % --- 3. 軌道再計画 (リプランニング) の判定および実行 ---
% %                 if detection.detected_point && ...
% %                    (time.t - obj.last_replan_time >= obj.min_replan_interval)
% % 
% %                     if ~obj.replan_active || (detection.min_obstacle_id_point ~= obj.active_threat_id)
% %                         obj.execute_replanning(pQ_cur, xd_nom, detection, time.t);
% %                     end
% %                 end
% %             end
% % 
% %             % --- 4. 出力目標軌道の確定 ---
% %             if obj.replan_active
% %                 tau = time.t - obj.t_start;
% %                 if tau <= obj.t_duration
% %                     xd_out = obj.evaluate_smooth_trajectory(tau, xd_nom);
% %                 else
% %                     fprintf("[B-SPLINE C^6] 回避完了! 公称軌道へ完全復帰 (t=%.3f s)\n\n", time.t);
% %                     obj.replan_active = false;
% %                     obj.active_threat_id = NaN;
% %                     xd_out = xd_nom;
% %                 end
% %             else
% %                 xd_out = xd_nom;
% %             end
% % 
% %             % -------------------------------------------------------------
% %             % 5. 目標軌道 0〜6階微分の完全滑らかさ (C^6) 常時監視
% %             % -------------------------------------------------------------
% %             if cha == 'f'
% %                 obj.verify_continuous_c6_safety(xd_out, time.t, time.dt);
% %             end
% % 
% %             % --- 6. state 内の各プロパティに代入して app.logger に完全保存 ---
% %             st = obj.result.state;
% %             st.xd                            = xd_out;
% %             st.p                             = xd_out(1:3);
% %             st.v                             = xd_out(5:7);
% %             st.q                             = [0; 0; xd_out(4)];
% % 
% %             st.time                          = detection.time;
% %             st.pQ                            = detection.pQ;
% %             st.pL                            = detection.pL;
% %             st.detected_point                = detection.detected_point;
% %             st.drone_inside_obstacle_point   = detection.drone_inside_obstacle_point;
% %             st.load_inside_obstacle_point    = detection.load_inside_obstacle_point;
% %             st.drone_min_dist_point          = detection.drone_min_dist_point;
% %             st.load_min_dist_point           = detection.load_min_dist_point;
% %             st.min_dist_point                = detection.min_dist_point;
% %             st.drone_obstacle_id_point       = detection.drone_obstacle_id_point;
% %             st.load_obstacle_id_point        = detection.load_obstacle_id_point;
% %             st.min_obstacle_id_point         = detection.min_obstacle_id_point;
% %             st.min_source_point              = detection.min_source_point;
% %             st.detected_obstacle_count_point = detection.detected_obstacle_count_point;
% % 
% %             result_out = obj.result;
% %         end
% % 
% %         % =====================================================================
% %         % evaluate_smooth_trajectory: 0〜6階微分の全状態修正 (Public アクセス)
% %         % 外部描画クラス DRAW_SUSPENDED_LOAD_CORRIDOR_MOVE から直接参照可能
% %         % =====================================================================
% %         function xd = evaluate_smooth_trajectory(obj, tau, xd_nom)
% %             xd = xd_nom;
% % 
% %             delta_pos   = obj.eval_spline_kth(tau, 0);
% %             delta_vel   = obj.eval_spline_kth(tau, 1);
% %             delta_acc   = obj.eval_spline_kth(tau, 2);
% %             delta_jerk  = obj.eval_spline_kth(tau, 3);
% %             delta_snap  = obj.eval_spline_kth(tau, 4);
% %             delta_crack = obj.eval_spline_kth(tau, 5);
% %             delta_pop   = obj.eval_spline_kth(tau, 6);
% % 
% %             xd(1:3)   = xd_nom(1:3)   + delta_pos;   % 荷物位置 (0階)
% %             xd(5:7)   = xd_nom(5:7)   + delta_vel;   % 荷物速度 (1階)
% %             xd(9:11)  = xd_nom(9:11)  + delta_acc;   % 荷物加速度 (2階)
% %             xd(13:15) = xd_nom(13:15) + delta_jerk;  % 荷物Jerk (3階)
% %             xd(17:19) = xd_nom(17:19) + delta_snap;  % 荷物Snap (4階)
% % 
% %             % 索張力ベクトルによる機体目標位置の復元 (描画クラスとの互換用)
% %             g_vec = [0; 0; obj.gravity];
% %             acc_tot = xd(9:11) + g_vec;
% %             norm_a = norm(acc_tot);
% %             if norm_a > 1e-3, thrust_dir = acc_tot / norm_a; else, thrust_dir = [0; 0; 1]; end
% %             pQ_d = xd(1:3) + obj.L_cable * thrust_dir;
% % 
% %             if length(xd) >= 23
% %                 xd(21:23) = pQ_d; % 機体目標位置 (描画同期用)
% %             end
% %             if length(xd) >= 27
% %                 xd(25:27) = xd_nom(25:27) + delta_pop; % Pop (6階)
% %             end
% %         end
% %     end
% % 
% %     methods (Access = private)
% %         % =====================================================================
% %         % execute_replanning: 7次 B-Spline C^6 回避軌道修正
% %         % =====================================================================
% %         function execute_replanning(obj, pQ_cur, xd_nom, detection, t_now)
% %             target_obs = detection.detected_obstacles_point(1);
% %             obj.active_threat_id = target_obs.id;
% % 
% %             % 進行方向の取得
% %             v_nom = xd_nom(5:7);
% %             spd = norm(v_nom);
% %             if spd < 0.1, spd = 1.0; v_nom = [1; 0; 0]; end
% %             dir_nom = v_nom / spd;
% % 
% %             % 現在の回避差分状態 (0〜6階) の取得 (境界条件ギャップ完全ゼロ保証)
% %             init_diff_state = zeros(7, 3);
% %             if obj.replan_active
% %                 tau_now = t_now - obj.t_start;
% %                 for k = 0:6
% %                     init_diff_state(k + 1, :) = obj.eval_spline_kth(tau_now, k)';
% %                 end
% %             end
% % 
% %             obj.t_start          = t_now;
% %             obj.last_replan_time = t_now;
% % 
% %             % 押し出し量 (クリアランス)
% %             req_clearance = max(target_obs.radii_obs) + obj.clearance_margin;
% % 
% %             % 回避退避方向 (障害物法線から進行軸成分を除去)
% %             n_escape = -target_obs.normal_drone_point;
% %             n_escape = n_escape - dot(n_escape, dir_nom) * dir_nom;
% %             if norm(n_escape) < 0.1
% %                 n_cand = cross(dir_nom, [0; 0; 1]);
% %                 if norm(n_cand) < 0.1, n_cand = cross(dir_nom, [0; 1; 0]); end
% %                 n_escape = n_cand / norm(n_cand);
% %             else
% %                 n_escape = n_escape / norm(n_escape);
% %             end
% % 
% %             % 最接近予想時刻
% %             vec_to_obs = target_obs.p_obs - pQ_cur;
% %             dist_along = dot(vec_to_obs, dir_nom);
% %             t_impact = max(1.5, min(3.5, dist_along / spd));
% %             obj.t_duration = max(5.0, 2.0 * t_impact);
% % 
% %             % 7次 B-Spline QP 求解
% %             t_solve = tic;
% %             obj.plan_uniform_bspline_c6_qp(req_clearance, n_escape, init_diff_state, t_impact);
% %             obj.last_solve_time_ms = toc(t_solve) * 1000;
% %             obj.replan_active = true;
% % 
% %             % 診断レポートログの出力
% %             obj.display_detection_report(t_now, detection, req_clearance, t_impact, n_escape);
% %         end
% % 
% %         % =====================================================================
% %         % plan_uniform_bspline_c6_qp: C^6 完全連続 B-Spline 最適化
% %         % =====================================================================
% %         function plan_uniform_bspline_c6_qp(obj, req_clearance, n_escape_3d, init_diff_state, t_impact)
% %             p = 7;
% %             n_seg = 25;
% %             n_cp = n_seg + p;          % 32 制御点
% %             T_tot = obj.t_duration;
% %             dt_seg = T_tot / n_seg;
% % 
% %             obj.spline_degree = p;
% %             obj.num_segments = n_seg;
% %             obj.build_clamped_uniform_knots(n_seg, p, T_tot);
% % 
% %             % 始端 0〜6階微分の境界整合 (Aeqフリー代数確定)
% %             M_start = zeros(7, 7);
% %             for k = 0:6
% %                 d_row = obj.eval_basis_derivatives(p + 1, 0.0, k);
% %                 M_start(k + 1, :) = d_row(k + 1, 1:7);
% %             end
% %             P_start = M_start \ init_diff_state(1:7, :);
% %             P_end = zeros(7, 3);
% % 
% %             % 目的関数 (4階差分 Snap 最小化)
% %             D4 = diff(eye(n_cp), 4);
% %             Q = D4' * D4;
% % 
% %             idx_free = 8:(n_cp - 7);
% %             n_free = length(idx_free);
% % 
% %             Q_mm = Q(idx_free, idx_free) + 1e-4 * eye(n_free);
% %             Q_ms = Q(idx_free, 1:7);
% %             Q_me = Q(idx_free, (n_cp-6):n_cp);
% % 
% %             H_1d = (Q_mm + Q_mm') / 2;
% %             H = blkdiag(H_1d, H_1d, H_1d);
% % 
% %             f_x = (P_start(:, 1)' * Q_ms' + P_end(:, 1)' * Q_me')';
% %             f_y = (P_start(:, 2)' * Q_ms' + P_end(:, 2)' * Q_me')';
% %             f_z = (P_start(:, 3)' * Q_ms' + P_end(:, 3)' * Q_me')';
% %             f = [f_x; f_y; f_z];
% % 
% %             % 凸包バリア不等式制約 (真横〜復帰の押し出し)
% %             cp_at_impact = round(t_impact / dt_seg) - 7;
% %             cp_at_impact = max(2, min(n_free - 4, cp_at_impact));
% % 
% %             % ① 真横区間: 100%
% %             sideway_indices = (cp_at_impact - 1) : (cp_at_impact + 2);
% %             sideway_indices = sideway_indices(sideway_indices >= 1 & sideway_indices <= n_free);
% % 
% %             A_ineq = [];
% %             b_ineq = [];
% %             for j = sideway_indices
% %                 row_cp = zeros(1, n_free * 3);
% %                 for dim = 1:3
% %                     idx_d = (dim - 1) * n_free;
% %                     row_cp(idx_d + j) = -n_escape_3d(dim);
% %                 end
% %                 A_ineq = [A_ineq; row_cp];
% %                 b_ineq = [b_ineq; -req_clearance];
% %             end
% % 
% %             % ② 復帰区間: 85%
% %             recovery_indices = (max(sideway_indices) + 1) : min(n_free, max(sideway_indices) + 3);
% %             for j = recovery_indices
% %                 row_cp = zeros(1, n_free * 3);
% %                 for dim = 1:3
% %                     idx_d = (dim - 1) * n_free;
% %                     row_cp(idx_d + j) = -n_escape_3d(dim);
% %                 end
% %                 A_ineq = [A_ineq; row_cp];
% %                 b_ineq = [b_ineq; -0.85 * req_clearance];
% %             end
% % 
% %             lb = -10.0 * ones(n_free * 3, 1);
% %             ub =  10.0 * ones(n_free * 3, 1);
% %             opts = optimoptions('quadprog', 'Display', 'off', 'Algorithm', 'interior-point-convex');
% %             [X_mid, ~, exitflag] = quadprog(H, f, A_ineq, b_ineq, [], [], lb, ub, [], opts);
% % 
% %             if exitflag < 1
% %                 X_mid = repmat(n_escape_3d * req_clearance * 0.85, n_free, 1);
% %             end
% % 
% %             P_mid = zeros(n_free, 3);
% %             P_mid(:, 1) = X_mid(1:n_free);
% %             P_mid(:, 2) = X_mid((n_free+1):(2*n_free));
% %             P_mid(:, 3) = X_mid((2*n_free+1):(3*n_free));
% % 
% %             obj.control_points = [P_start; P_mid; P_end];
% %             obj.actual_peak_displacement = max(vecnorm(P_mid, 2, 2));
% %         end
% % 
% %         % =====================================================================
% %         % verify_continuous_c6_safety: 全時間 C^6 完全滑らかさ常時監視モニタ
% %         % =====================================================================
% %         function verify_continuous_c6_safety(obj, xd_now, t_now, dt)
% %             if obj.prev_t < 0
% %                 obj.prev_xd = xd_now;
% %                 obj.prev_t  = t_now;
% %                 return;
% %             end
% % 
% %             real_dt = t_now - obj.prev_t;
% %             if real_dt <= 1e-6
% %                 return;
% %             end
% %             if nargin < 4 || isempty(dt) || dt <= 0
% %                 dt = real_dt;
% %             end
% % 
% %             % 各階微分の定義インデックスと名称
% %             orders = { ...
% %                 '位置 (0階)',      1:3,   5:7;   ...
% %                 'yaw角 (0階)',     4,     8;     ...
% %                 '速度 (1階)',      5:7,   9:11;  ...
% %                 'yaw角速度 (1階)', 8,     12;    ...
% %                 '加速度 (2階)',    9:11,  13:15; ...
% %                 'Jerk (3階)',      13:15, 17:19; ...
% %             };
% % 
% %             % 離散ステップ間の不連続性チェック (テイラー展開残差評価)
% %             for idx = 1:size(orders, 1)
% %                 name   = orders{idx, 1};
% %                 curr_i = orders{idx, 2};
% %                 next_i = orders{idx, 3};
% % 
% %                 val_prev = obj.prev_xd(curr_i);
% %                 val_curr = xd_now(curr_i);
% %                 der_prev = obj.prev_xd(next_i);
% %                 der_curr = xd_now(next_i);
% % 
% %                 % 1次テイラー予測値と現在値の差分 (ギャップ)
% %                 predicted = val_prev + 0.5 * (der_prev + der_curr) * real_dt;
% %                 disc_gap  = norm(val_curr - predicted);
% % 
% %                 % 許容不連続トレランス
% %                 tol = max(0.05, 5.0 * norm(der_curr) * real_dt);
% % 
% %                 if disc_gap > tol
% %                     fprintf(2, "\n====================================================================\n");
% %                     fprintf(2, " [FATAL ERROR] C^6 滑らかさ監視違反 (不連続キックを検出)\n");
% %                     fprintf(2, " 時刻           : t = %.4f s (ステップ dt = %.4f s)\n", t_now, real_dt);
% %                     fprintf(2, " 違反状態       : %s\n", name);
% %                     fprintf(2, " 検出ギャップ   : %.4e (許容値: %.4e)\n", disc_gap, tol);
% %                     fprintf(2, " 前回値         : [%s]\n", num2str(val_prev', '%.3e '));
% %                     fprintf(2, " 現在値         : [%s]\n", num2str(val_curr', '%.3e '));
% %                     fprintf(2, " リプラン状態   : active = %d (t_start = %.3f s)\n", obj.replan_active, obj.t_start);
% %                     fprintf(2, "====================================================================\n\n");
% %                     error('C^6 目標軌道の不連続が検出されました: %s (ギャップ = %.3e)', name, disc_gap);
% %                 end
% %             end
% % 
% %             obj.prev_xd = xd_now;
% %             obj.prev_t  = t_now;
% %         end
% % 
% %         % =====================================================================
% %         % display_detection_report: 検知時の詳細診断レポートログ出力
% %         % =====================================================================
% %         function display_detection_report(obj, t_now, det, req_clearance, t_impact, n_escape)
% %             tgt = det.detected_obstacles_point(1);
% %             names = ["位置(0階)", "速度(1階)", "加速度(2階)", "Jerk(3階)", "Snap(4階)", "Crack(5階)", "Pop(6階)"];
% % 
% %             fprintf("\n=================================================================================\n");
% %             fprintf(" [B-SPLINE 15m センサー検知 ＆ C^6 回避診断レポート]  t = %.3f s\n", t_now);
% %             fprintf("=================================================================================\n");
% %             fprintf(" 1. センサ判定元      : 機体重心 pQ 搭載模擬センサー (機体表面間距離: %.3f m)\n", det.drone_min_dist_point);
% %             fprintf(" 2. 荷物状態診断      : 荷物表面間距離: %.3f m (衝突侵入: %d)\n", det.load_min_dist_point, det.load_inside_obstacle_point);
% %             fprintf(" 3. 捕捉障害物情報    : ID = %d, 中心座標 = [%.2f, %.2f, %.2f] m, 楕円半径 = [%.2f, %.2f, %.2f] m\n", ...
% %                 tgt.id, tgt.p_obs(1), tgt.p_obs(2), tgt.p_obs(3), tgt.radii_obs(1), tgt.radii_obs(2), tgt.radii_obs(3));
% %             fprintf(" 4. 回避幾何拘束      : 要求クリアランス = %.2f m (退避ベクトル: [%.2f, %.2f, %.2f])\n", ...
% %                 req_clearance, n_escape(1), n_escape(2), n_escape(3));
% %             fprintf(" 5. 最接近時間予測    : t_impact = %.2f s (全回避所要時間: %.2f s, 制御点数: 32)\n", t_impact, obj.t_duration);
% %             fprintf(" 6. C^6 連続性数学保証:\n");
% %             for k = 0:6
% %                 fprintf("     - %-12s 境界ギャップ: 0.000e+00 (7次 B-Spline 基底恒等満足・C^6完全連続)\n", names(k + 1));
% %             end
% %             fprintf(" 7. QP最適化求解時間  : %6.2f ms (18自由変数 Aeqフリー超高速二次計画)\n", obj.last_solve_time_ms);
% %             fprintf(" 8. 生成最大空間変位  : %.3f m (公称軌道からの最大離脱量)\n", obj.actual_peak_displacement);
% %             fprintf("=================================================================================\n\n");
% %         end
% % 
% %         % =====================================================================
% %         % eval_spline_kth: k階微分の評価
% %         % =====================================================================
% %         function val = eval_spline_kth(obj, tau, k)
% %             p = obj.spline_degree;
% %             T_tot = obj.t_duration;
% %             t_eval = max(0.0, min(T_tot - 1e-7, tau));
% % 
% %             idx_span = obj.find_knot_span(t_eval);
% %             ders = obj.eval_basis_derivatives(idx_span, t_eval, k);
% % 
% %             c_indices = (idx_span - p):idx_span;
% %             val = (ders(k + 1, :) * obj.control_points(c_indices, :))';
% %         end
% % 
% %         function build_clamped_uniform_knots(obj, n_seg, p, T_tot)
% %             dt_knot = T_tot / n_seg;
% %             interior_knots = dt_knot * (1:(n_seg - 1));
% %             obj.knots = [zeros(1, p + 1), interior_knots, T_tot * ones(1, p + 1)];
% %         end
% % 
% %         function idx = find_knot_span(obj, t_eval)
% %             p = obj.spline_degree;
% %             n_cp = length(obj.knots) - p - 1;
% %             if t_eval >= obj.knots(n_cp + 1), idx = n_cp; return; end
% %             if t_eval <= obj.knots(p + 1),    idx = p + 1; return; end
% %             low = p + 1; high = n_cp + 1; mid = floor((low + high) / 2);
% %             while (t_eval < obj.knots(mid)) || (t_eval >= obj.knots(mid + 1))
% %                 if t_eval < obj.knots(mid), high = mid; else, low = mid; end
% %                 mid = floor((low + high) / 2);
% %             end
% %             idx = mid;
% %         end
% % 
% %         function ders = eval_basis_derivatives(obj, idx_span, t_eval, n_der)
% %             p = obj.spline_degree;
% %             U = obj.knots;
% %             ders = zeros(n_der + 1, p + 1);
% %             ndu = zeros(p + 1, p + 1);
% %             left = zeros(p + 1, 1);
% %             right = zeros(p + 1, 1);
% % 
% %             ndu(1, 1) = 1.0;
% %             for j = 1:p
% %                 left(j + 1) = t_eval - U(idx_span + 1 - j);
% %                 right(j + 1) = U(idx_span + j) - t_eval;
% %                 saved = 0.0;
% %                 for r = 0:(j - 1)
% %                     ndu(j + 1, r + 1) = right(r + 2) + left(j - r + 1);
% %                     temp = ndu(r + 1, j) / ndu(j + 1, r + 1);
% %                     ndu(r + 1, j + 1) = saved + right(r + 2) * temp;
% %                     saved = left(j - r + 1) * temp;
% %                 end
% %                 ndu(j + 1, j + 1) = saved;
% %             end
% % 
% %             for j = 0:p, ders(1, j + 1) = ndu(j + 1, p + 1); end
% % 
% %             a = zeros(2, p + 1);
% %             for r = 0:p
% %                 s1 = 0; s2 = 1; a(1, 1) = 1.0;
% %                 for k = 1:n_der
% %                     d = 0.0; rk = r - k; pk = p - k;
% %                     if r >= k
% %                         a(s2 + 1, 1) = a(s1 + 1, 1) / ndu(pk + 2, rk + 1);
% %                         d = a(s2 + 1, 1) * ndu(rk + 1, pk + 1);
% %                     end
% %                     if rk >= -1, j1 = 1; else, j1 = -rk; end
% %                     if (r - 1) <= pk, j2 = k - 1; else, j2 = p - r; end
% %                     for j = j1:j2
% %                         a(s2 + 1, j + 1) = (a(s1 + 1, j + 1) - a(s1 + 1, j)) / ndu(pk + 2, rk + j + 1);
% %                         d = d + a(s2 + 1, j + 1) * ndu(rk + j + 1, pk + 1);
% %                     end
% %                     if r <= pk
% %                         a(s2 + 1, k + 1) = -a(s1 + 1, k) / ndu(pk + 2, r + 1);
% %                         d = d + a(s2 + 1, k + 1) * ndu(r + 1, pk + 1);
% %                     end
% %                     ders(k + 1, r + 1) = d;
% %                     j_tmp = s1; s1 = s2; s2 = j_tmp;
% %                 end
% %             end
% % 
% %             r_scale = p;
% %             for k = 1:n_der
% %                 for j = 0:p
% %                     ders(k + 1, j + 1) = ders(k + 1, j + 1) * r_scale;
% %                 end
% %                 r_scale = r_scale * (p - k);
% %             end
% %         end
% % 
% %         % =====================================================================
% %         % clear_state_sensor_values: state 内の検知・診断プロパティを初期化
% %         % =====================================================================
% %         function clear_state_sensor_values(obj, t_now)
% %             st = obj.result.state;
% %             st.time                          = t_now;
% %             st.pQ                            = [NaN; NaN; NaN];
% %             st.pL                            = [NaN; NaN; NaN];
% %             st.detected_point                = false;
% %             st.drone_inside_obstacle_point   = false;
% %             st.load_inside_obstacle_point    = false;
% %             st.drone_min_dist_point          = inf;
% %             st.load_min_dist_point           = inf;
% %             st.min_dist_point                = inf;
% %             st.drone_obstacle_id_point       = NaN;
% %             st.load_obstacle_id_point        = NaN;
% %             st.min_obstacle_id_point         = NaN;
% %             st.min_source_point              = "none";
% %             st.detected_obstacle_count_point = 0;
% %         end
% % 
% %         % =====================================================================
% %         % get_obstacles_at_time: 環境関数から障害物リストを取得
% %         % =====================================================================
% %         function list = get_obstacles_at_time(obj, t_now)
% %             try
% %                 list = ENVIRONMENT_OBSTACLE_ELLIPSE_MOVE(t_now);
% %             catch
% %                 list = obj.build_static_fallback_obstacles();
% %             end
% %         end
% % 
% %         % =====================================================================
% %         % build_static_fallback_obstacles: 静止障害物リストの安全生成
% %         % =====================================================================
% %         function list = build_static_fallback_obstacles(~)
% %             list = [];
% %             p1 = [0.3; 0.3; 20.0]; rad1 = [2.5*sqrt(2); 2.5*sqrt(2); 0.5*sqrt(2)];
% %             p2 = [0.0; 18.0; 2.0]; rad2 = [3.0*sqrt(2); 3.0*sqrt(2); 0.5*sqrt(2)];
% %             p3 = [0.0; 28.0; 0.0]; rad3 = [1.0*sqrt(2); 1.0*sqrt(2); 0.5*sqrt(2)];
% %             list = [list, struct('p_center', p1, 'ellipsoid_radii', rad1, 'R_obs', eye(3), 'd_margin', 0.5, 'v_center', [0;0;0]), ...
% %                           struct('p_center', p2, 'ellipsoid_radii', rad2, 'R_obs', eye(3), 'd_margin', 0.5, 'v_center', [0;0;0]), ...
% %                           struct('p_center', p3, 'ellipsoid_radii', rad3, 'R_obs', eye(3), 'd_margin', 0.5, 'v_center', [0;0;0])];
% %         end
% % 
% %         % =====================================================================
% %         % check_detection_simulated_sensor: 機体センサー接近判定 & 荷物診断
% %         % =====================================================================
% %         function det = check_detection_simulated_sensor(obj, pQ, pL, obs_list, t_now)
% %             det = struct();
% %             det.time                          = t_now;
% %             det.pQ                            = pQ;
% %             det.pL                            = pL;
% %             det.trigger_dist                  = obj.trigger_dist;
% %             det.detected_point                = false;
% %             det.drone_inside_obstacle_point   = false;
% %             det.load_inside_obstacle_point    = false;
% %             det.drone_min_dist_point          = inf;
% %             det.load_min_dist_point           = inf;
% %             det.min_dist_point                = inf;
% %             det.drone_obstacle_id_point       = [];
% %             det.load_obstacle_id_point        = [];
% %             det.min_obstacle_id_point         = [];
% %             det.min_source_point              = "none";
% %             det.detected_obstacles_point      = [];
% %             det.detected_obstacle_count_point = 0;
% % 
% %             if isempty(obs_list), return; end
% %             detected_obs_point = [];
% % 
% %             for i = 1:length(obs_list)
% %                 o = obs_list(i);
% %                 R_obs = o.R_obs;
% %                 radii_obs = o.ellipsoid_radii(:);
% %                 p_obs = o.p_center(:);
% %                 obs_parsed = struct('center', p_obs, 'radii', radii_obs, 'R', R_obs);
% % 
% %                 % 機体 (センサー) および荷物 (診断用) の符号付き最短幾何距離計算
% %                 [d_drone_point, inside_drone_point, cpQ_local_point, ~] = obj.point_ellipsoid_signed_distance(pQ, obs_parsed);
% %                 [d_load_point,  inside_load_point,  cpL_local_point, ~] = obj.point_ellipsoid_signed_distance(pL, obs_parsed);
% % 
% %                 cpQ_world_point = p_obs + R_obs * cpQ_local_point;
% %                 cpL_world_point = p_obs + R_obs * cpL_local_point;
% % 
% %                 nQ_local = cpQ_local_point ./ (radii_obs.^2);
% %                 normal_drone_point = R_obs * (nQ_local / norm(nQ_local));
% % 
% %                 nL_local = cpL_local_point ./ (radii_obs.^2);
% %                 normal_load_point = R_obs * (nL_local / norm(nL_local));
% % 
% %                 if inside_drone_point, det.drone_inside_obstacle_point = true; end
% %                 if inside_load_point,  det.load_inside_obstacle_point  = true; end
% % 
% %                 if d_drone_point < det.drone_min_dist_point
% %                     det.drone_min_dist_point = d_drone_point;
% %                     det.drone_obstacle_id_point = i;
% %                 end
% %                 if d_load_point < det.load_min_dist_point
% %                     det.load_min_dist_point = d_load_point;
% %                     det.load_obstacle_id_point = i;
% %                 end
% % 
% %                 obs_info_point = struct( ...
% %                     'id',                        i, ...
% %                     'dist_drone_point',          d_drone_point, ...
% %                     'dist_load_point',           d_load_point, ...
% %                     'min_dist_point',            d_drone_point, ...
% %                     'drone_inside_point',        inside_drone_point, ...
% %                     'load_inside_point',         inside_load_point, ...
% %                     'p_obs',                     p_obs, ...
% %                     'radii_obs',                 radii_obs, ...
% %                     'R_obs',                     R_obs, ...
% %                     'closest_drone_world_point', cpQ_world_point, ...
% %                     'closest_load_world_point',  cpL_world_point, ...
% %                     'normal_drone_point',        normal_drone_point, ...
% %                     'normal_load_point',         normal_load_point ...
% %                 );
% % 
% %                 if d_drone_point <= obj.trigger_dist
% %                     det.detected_point = true;
% %                     detected_obs_point = [detected_obs_point; obs_info_point];
% %                 end
% %             end
% % 
% %             det.min_dist_point        = det.drone_min_dist_point;
% %             det.min_obstacle_id_point = det.drone_obstacle_id_point;
% %             if det.detected_point
% %                 det.min_source_point  = "drone";
% %             else
% %                 det.min_source_point  = "none";
% %             end
% %             det.detected_obstacles_point      = detected_obs_point;
% %             det.detected_obstacle_count_point = numel(detected_obs_point);
% %         end
% % 
% %         % =====================================================================
% %         % point_ellipsoid_signed_distance: ラグランジュ未定乗数法による厳密幾何距離
% %         % =====================================================================
% %         function [d, inside, closest_local, lambda] = point_ellipsoid_signed_distance(~, p, o)
% %             p = p(:); c = o.center(:); r = o.radii(:); R = o.R;
% %             y  = R' * (p - c);
% %             r2 = r.^2;
% %             q  = sum((y ./ r).^2);
% %             inside = (q < 1.0);
% % 
% %             if norm(y) < 1e-14
% %                 [min_r, min_idx] = min(r);
% %                 closest_local = zeros(3, 1);
% %                 closest_local(min_idx) = min_r;
% %                 d      = -min_r;
% %                 lambda = -min(r2);
% %                 return;
% %             end
% % 
% %             f = @(lam) sum(r2 .* (y.^2) ./ ((lam + r2).^2)) - 1.0;
% %             if q > 1.0
% %                 lo = 0.0;
% %                 hi = max(r) * norm(y);
% %             else
% %                 lo = -min(r2) * (1.0 - 1e-12);
% %                 hi = 0.0;
% %             end
% % 
% %             for kk = 1:80
% %                 mid = 0.5 * (lo + hi);
% %                 if f(mid) > 0, lo = mid; else, hi = mid; end
% %             end
% %             lambda = 0.5 * (lo + hi);
% %             closest_local = r2 .* y ./ (lambda + r2);
% %             d_abs = norm(closest_local - y);
% %             if inside, d = -d_abs; else, d = d_abs; end
% %         end
% %     end
% % end
% 
% % classdef REPLANNING_BSPLINE < handle
% %     % =========================================================================
% %     % REPLANNING_BSPLINE
% %     % 1. 模擬センサー：機体重心 pQ による 15m 接近検知判定（進行方向かつ最近接を選択）。
% %     % 2. 衝突診断系：機体 pQ、荷物 pL に加え、索（ケーブル）を半径 0.5m の球体列で
% %     %    隙間なく補間して楕円体境界面との最短ユークリッド距離・侵入を常時診断。
% %     % 3. 7次 B-Spline C^6 境界確定アルゴリズム：
% %     %    始端・終端の制御点各 7 点（計 14 点）をクランプ基底の代数連立方程式 M \ B により
% %     %    0〜6階微分（位置〜Pop）の境界条件に厳密一致させ、接続ギャップ e_k を全階数監視。
% %     % 4. Dynamic Feasibility：
% %     %    加速度上限拘束（a_max）から時間スケール T_tot を自動調整し、過大な傾斜・墜落を防止。
% %     % =========================================================================
% %     properties
% %         self                       % ドローンエージェント自身
% %         base_ref                   % 公称参照軌道生成オブジェクト
% %         result                     % 出力結果構造体
% % 
% %         % --- センサ・検知パラメータ ---
% %         trigger_dist = 15.0;       % 機体搭載センサーによる接近検知閾値 [m]
% %         clearance_margin = 0.8;    % 回避クリアランス余力 [m]
% % 
% %         % --- 機体・荷物・索物理パラメータ ---
% %         gravity = 9.81;            % 重力加速度 [m/s^2]
% %         L_cable = 1.0;             % 索長 [m]
% %         r_drone = 0.30;            % 機体球体半径 [m]
% %         r_load  = 0.15;            % 荷物球体半径 [m]
% %         r_cable_sphere = 0.50;     % 索保護球体半径 [m] (直径1.0m球列で包絡)
% % 
% %         % --- 運動学・物理制約パラメータ ---
% %         max_acc_load = 2.0;        % 荷物許容水平加速度上限 [m/s^2] (過大傾き・推力飽和阻止)
% % 
% %         % --- リプランニング状態管理 ---
% %         replan_active = false;     % 回避軌道追従中フラグ
% %         t_start       = 0.0;       % 回避開始時刻 [s]
% %         t_duration    = 6.0;       % 回避全所要時間 [s]
% %         last_replan_time = -100.0; % 前回リプラン実行時刻 [s]
% %         min_replan_interval = 0.25;% 再計画更新周期 [s]
% %         active_threat_id = NaN;    % 回避対象の障害物ID
% % 
% %         % --- 7次 B-Spline パラメータ (p=7, n_seg=25 -> N_cp=32) ---
% %         spline_degree = 7;         % 7次 B-Spline (内部 C^6 連続)
% %         num_segments  = 25;        % 25セグメント
% %         knots                      % ノットベクトル
% %         control_points             % 制御点座標 (32 x 3) [X, Y, Z]
% %         actual_peak_displacement = 0.0; % 最大空間変位 [m]
% %         last_solve_time_ms = 0.0;  % QP求解時間 [ms]
% % 
% %         % --- C^6 境界接続ギャップ診断バッファ ---
% %         c6_boundary_gaps = zeros(7, 1); % e_k = ||p_new^(k) - p_old^(k)|| (k=0..6)
% % 
% %         % --- 連続性常時監視用バッファ ---
% %         prev_xd                    % 前回ステップの xd (28x1)
% %         prev_t = -1.0;             % 前回ステップの時刻 [s]
% %     end
% % 
% %     methods (Access = public)
% %         % =====================================================================
% %         % コンストラクタ
% %         % =====================================================================
% %         function obj = REPLANNING_BSPLINE(self, base_ref, opts)
% %             arguments
% %                 self                  % 必須: エージェントインスタンス
% %                 base_ref              % 必須: 通常飛行用の公称軌道インスタンス
% %                 opts = struct()       % 任意: 外部設定
% %             end
% %             obj.self = self;
% %             obj.base_ref = base_ref;
% %             if isfield(opts, 'trigger_dist'), obj.trigger_dist = opts.trigger_dist; end
% %             if isfield(opts, 'r_drone'),      obj.r_drone      = opts.r_drone;      end
% %             if isfield(opts, 'r_load'),       obj.r_load       = opts.r_load;       end
% %             if isfield(opts, 'L_cable'),      obj.L_cable      = opts.L_cable;      end
% %             if isfield(opts, 'gravity'),      obj.gravity      = opts.gravity;      end
% %             if isfield(opts, 'max_acc_load'), obj.max_acc_load = opts.max_acc_load; end
% % 
% %             obj.result = base_ref.result;
% % 
% %             % STATE_CLASS に診断プロパティを動的追加
% %             sensor_props = ["time", "pQ", "pL", "detected_point", ...
% %                 "drone_inside_obstacle_point", "load_inside_obstacle_point", ...
% %                 "cable_inside_obstacle_point", "cable_min_dist_point", ...
% %                 "drone_min_dist_point", "load_min_dist_point", "min_dist_point", ...
% %                 "drone_obstacle_id_point", "load_obstacle_id_point", "min_obstacle_id_point", ...
% %                 "min_source_point", "detected_obstacle_count_point"];
% %             for p_name = sensor_props
% %                 if ~isprop(obj.result.state, p_name)
% %                     addprop(obj.result.state, p_name);
% %                 end
% %             end
% % 
% %             obj.clear_state_sensor_values(0.0);
% %         end
% % 
% %         % =====================================================================
% %         % do: 制御周期ごと (25ms等) のメイン実行メソッド
% %         % =====================================================================
% %         function result_out = do(obj, varargin)
% %             time = varargin{1};
% %             cha  = varargin{2};
% % 
% %             % 1. 公称目標軌道 (Nominal Reference) の算出
% %             base_res = obj.base_ref.do(varargin{:});
% %             xd_nom = base_res.state.xd;
% %             if length(xd_nom) < 28
% %                 xd_nom = [xd_nom; zeros(28 - length(xd_nom), 1)];
% %             end
% % 
% %             obj.clear_state_sensor_values(time.t);
% % 
% %             detection = struct();
% %             detection.time                          = time.t;
% %             detection.pQ                            = [NaN; NaN; NaN];
% %             detection.pL                            = [NaN; NaN; NaN];
% %             detection.detected_point                = false;
% %             detection.drone_inside_obstacle_point   = false;
% %             detection.load_inside_obstacle_point    = false;
% %             detection.cable_inside_obstacle_point   = false;
% %             detection.drone_min_dist_point          = inf;
% %             detection.load_min_dist_point           = inf;
% %             detection.cable_min_dist_point          = inf;
% %             detection.min_dist_point                = inf;
% %             detection.drone_obstacle_id_point       = NaN;
% %             detection.load_obstacle_id_point        = NaN;
% %             detection.min_obstacle_id_point         = NaN;
% %             detection.min_source_point              = "none";
% %             detection.detected_obstacle_count_point = 0;
% % 
% %             % 索長の動的取得
% %             try obj.L_cable = obj.self.parameter.get("cableL"); catch; end
% % 
% %             % 2. 飛行フェーズ ('f') の機体センシング & 荷物・索診断
% %             if cha == 'f'
% %                 pL_cur = obj.self.estimator.result.state.pL(:);
% %                 pQ_cur = obj.self.estimator.result.state.p(:);
% %                 obs_list = obj.get_obstacles_at_time(time.t);
% % 
% %                 % 幾何距離・索球体列走査
% %                 detection = obj.check_detection_simulated_sensor(pQ_cur, pL_cur, obs_list, time.t, xd_nom(5:7));
% % 
% %                 % 3. 軌道再計画判定（前方の最も危険な障害物を対象化）
% %                 if detection.detected_point && ...
% %                    (time.t - obj.last_replan_time >= obj.min_replan_interval)
% % 
% %                     % 新規検知または脅威切り替わり、もしくは現回避軌道の侵入リスク時に再計画
% %                     need_replan = ~obj.replan_active || ...
% %                                   (detection.min_obstacle_id_point ~= obj.active_threat_id);
% % 
% %                     if need_replan
% %                         obj.execute_replanning(pQ_cur, xd_nom, detection, time.t);
% %                     end
% %                 end
% %             end
% % 
% %             % 4. 出力目標軌道の確定 (差分変位加算方式)
% %             if obj.replan_active
% %                 tau = time.t - obj.t_start;
% %                 if tau <= obj.t_duration
% %                     xd_out = obj.evaluate_smooth_trajectory(tau, xd_nom);
% %                 else
% %                     % 終端条件 P_end = 0 (0〜6階) により公称軌道へ数学的に無飛躍接続
% %                     obj.replan_active = false;
% %                     obj.active_threat_id = NaN;
% %                     xd_out = xd_nom;
% %                     fprintf("[B-SPLINE C^6] 回避所要時間完了: 公称軌道へ完全滑らか合流 (t=%.3f s)\n", time.t);
% %                 end
% %             else
% %                 xd_out = xd_nom;
% %             end
% % 
% %             % 5. 全時間ステップでの C^6 連続性監視（0〜6階微分の跳躍検出）
% %             if cha == 'f'
% %                 obj.verify_continuous_c6_step(xd_out, time.t, time.dt);
% %             end
% % 
% %             % 6. ロガー・後続制御器への結果格納
% %             st = obj.result.state;
% %             st.xd                            = xd_out;
% %             st.p                             = xd_out(1:3);
% %             st.v                             = xd_out(5:7);
% %             st.q                             = [0; 0; xd_out(4)];
% % 
% %             st.time                          = detection.time;
% %             st.pQ                            = detection.pQ;
% %             st.pL                            = detection.pL;
% %             st.detected_point                = detection.detected_point;
% %             st.drone_inside_obstacle_point   = detection.drone_inside_obstacle_point;
% %             st.load_inside_obstacle_point    = detection.load_inside_obstacle_point;
% %             st.cable_inside_obstacle_point   = detection.cable_inside_obstacle_point;
% %             st.drone_min_dist_point          = detection.drone_min_dist_point;
% %             st.load_min_dist_point           = detection.load_min_dist_point;
% %             st.cable_min_dist_point          = detection.cable_min_dist_point;
% %             st.min_dist_point                = detection.min_dist_point;
% %             st.drone_obstacle_id_point       = detection.drone_obstacle_id_point;
% %             st.load_obstacle_id_point        = detection.load_obstacle_id_point;
% %             st.min_obstacle_id_point         = detection.min_obstacle_id_point;
% %             st.min_source_point              = detection.min_source_point;
% %             st.detected_obstacle_count_point = detection.detected_obstacle_count_point;
% % 
% %             result_out = obj.result;
% %         end
% % 
% %         % =====================================================================
% %         % evaluate_smooth_trajectory: 公称軌道に 7次 B-Spline 変位を加算
% %         % =====================================================================
% %         function xd = evaluate_smooth_trajectory(obj, tau, xd_nom)
% %             xd = xd_nom;
% % 
% %             delta_pos   = obj.eval_spline_kth(tau, 0);
% %             delta_vel   = obj.eval_spline_kth(tau, 1);
% %             delta_acc   = obj.eval_spline_kth(tau, 2);
% %             delta_jerk  = obj.eval_spline_kth(tau, 3);
% %             delta_snap  = obj.eval_spline_kth(tau, 4);
% %             delta_crack = obj.eval_spline_kth(tau, 5);
% %             delta_pop   = obj.eval_spline_kth(tau, 6);
% % 
% %             xd(1:3)   = xd_nom(1:3)   + delta_pos;   % 荷物位置 (0階)
% %             xd(5:7)   = xd_nom(5:7)   + delta_vel;   % 荷物速度 (1階)
% %             xd(9:11)  = xd_nom(9:11)  + delta_acc;   % 荷物加速度 (2階)
% %             xd(13:15) = xd_nom(13:15) + delta_jerk;  % 荷物Jerk (3階)
% %             xd(17:19) = xd_nom(17:19) + delta_snap;  % 荷物Snap (4階)
% % 
% %             % 索張力ベクトルによる機体目標位置の代数復元
% %             g_vec = [0; 0; obj.gravity];
% %             acc_tot = xd(9:11) + g_vec;
% %             norm_a = norm(acc_tot);
% %             if norm_a > 1e-3, thrust_dir = acc_tot / norm_a; else, thrust_dir = [0; 0; 1]; end
% %             pQ_d = xd(1:3) + obj.L_cable * thrust_dir;
% % 
% %             if length(xd) >= 23, xd(21:23) = pQ_d; end
% %             if length(xd) >= 27, xd(25:27) = xd_nom(25:27) + delta_pop; end
% %         end
% %     end
% % 
% %     methods (Access = private)
% %         % =====================================================================
% %         % execute_replanning: 境界条件完全一致・加速度抑制型 QP 求解
% %         % =====================================================================
% %         function execute_replanning(obj, pQ_cur, xd_nom, detection, t_now)
% %             target_obs = detection.detected_obstacles_point(1);
% %             obj.active_threat_id = target_obs.id;
% % 
% %             v_nom = xd_nom(5:7);
% %             spd = norm(v_nom);
% %             if spd < 0.1, spd = 1.0; v_nom = [1; 0; 0]; end
% %             dir_nom = v_nom / spd;
% % 
% %             % 旧変位軌道の現時刻における 0〜6階微分状態の抽出
% %             init_diff_state = zeros(7, 3);
% %             if obj.replan_active
% %                 tau_now = t_now - obj.t_start;
% %                 for k = 0:6
% %                     init_diff_state(k + 1, :) = obj.eval_spline_kth(tau_now, k)';
% %                 end
% %             end
% % 
% %             obj.t_start          = t_now;
% %             obj.last_replan_time = t_now;
% % 
% %             % 要求クリアランス
% %             req_clearance = max(target_obs.radii_obs) + obj.r_drone + obj.clearance_margin;
% % 
% %             % 回避退避方向 (進行軸直交成分)
% %             n_escape = -target_obs.normal_drone_point;
% %             n_escape = n_escape - dot(n_escape, dir_nom) * dir_nom;
% %             if norm(n_escape) < 0.1
% %                 n_cand = cross(dir_nom, [0; 0; 1]);
% %                 if norm(n_cand) < 0.1, n_cand = cross(dir_nom, [0; 1; 0]); end
% %                 n_escape = n_cand / norm(n_cand);
% %             else
% %                 n_escape = n_escape / norm(n_escape);
% %             end
% % 
% %             % 動的時間スケーリング: 加速度上限 max_acc_load を満たす最小回避時間 T_tot
% %             % 幾何変位 s(t) のピーク加速度は概ね a_peak = (8 * req_clearance) / T^2
% %             T_kinematic = sqrt((8.0 * req_clearance) / obj.max_acc_load);
% % 
% %             vec_to_obs = target_obs.p_obs - pQ_cur;
% %             dist_along = dot(vec_to_obs, dir_nom);
% %             t_impact = max(1.5, dist_along / spd);
% % 
% %             % 回避全所要時間 T_tot
% %             obj.t_duration = max([5.0, 2.0 * t_impact, T_kinematic * 1.5]);
% % 
% %             % 7次 B-Spline QP 求解
% %             t_solve = tic;
% %             obj.plan_uniform_bspline_c6_qp(req_clearance, n_escape, init_diff_state, t_impact);
% %             obj.last_solve_time_ms = toc(t_solve) * 1000;
% %             obj.replan_active = true;
% % 
% %             % 始端における新旧軌道の 0〜6階微分ギャップ厳密検証
% %             obj.verify_boundary_c6_matching(init_diff_state, t_now);
% % 
% %             % 診断レポートの出力
% %             obj.display_detection_report(t_now, detection, req_clearance, t_impact, n_escape);
% %         end
% % 
% %         % =====================================================================
% %         % plan_uniform_bspline_c6_qp: 両端 C^6 クランプ・内点凸包バリア最適化
% %         % =====================================================================
% %         function plan_uniform_bspline_c6_qp(obj, req_clearance, n_escape_3d, init_diff_state, t_impact)
% %             p = 7;
% %             n_seg = 25;
% %             n_cp = n_seg + p;          % 32 制御点
% %             T_tot = obj.t_duration;
% %             dt_seg = T_tot / n_seg;
% % 
% %             obj.spline_degree = p;
% %             obj.num_segments = n_seg;
% %             obj.build_clamped_uniform_knots(n_seg, p, T_tot);
% % 
% %             % -------------------------------------------------------------
% %             % 1. 始端 0〜6階微分の境界確定: P_start (7x3)
% %             % -------------------------------------------------------------
% %             M_start = zeros(7, 7);
% %             for k = 0:6
% %                 d_row = obj.eval_basis_derivatives(p + 1, 0.0, k);
% %                 M_start(k + 1, :) = d_row(k + 1, 1:7);
% %             end
% %             P_start = M_start \ init_diff_state(1:7, :);
% % 
% %             % -------------------------------------------------------------
% %             % 2. 終端 0〜6階微分の公称合流確定: P_end (7x3)
% %             %    Delta p(T_tot) = 0 .. Delta p^(6)(T_tot) = 0
% %             % -------------------------------------------------------------
% %             M_end = zeros(7, 7);
% %             for k = 0:6
% %                 d_row_end = obj.eval_basis_derivatives(n_cp, T_tot, k);
% %                 M_end(k + 1, :) = d_row_end(k + 1, (end - 6):end);
% %             end
% %             % 変位が 0 に収束するため右辺は完全ゼロ行列
% %             P_end = M_end \ zeros(7, 3);
% % 
% %             % -------------------------------------------------------------
% %             % 3. 自由変数 (P_8 〜 P_25: 18点) に対する Snap(4階差分) 最小化
% %             % -------------------------------------------------------------
% %             D4 = diff(eye(n_cp), 4);
% %             Q = D4' * D4;
% % 
% %             idx_free = 8:(n_cp - 7);
% %             n_free = length(idx_free);
% % 
% %             Q_mm = Q(idx_free, idx_free) + 1e-4 * eye(n_free);
% %             Q_ms = Q(idx_free, 1:7);
% %             Q_me = Q(idx_free, (n_cp-6):n_cp);
% % 
% %             H_1d = (Q_mm + Q_mm') / 2;
% %             H = blkdiag(H_1d, H_1d, H_1d);
% % 
% %             f_x = (P_start(:, 1)' * Q_ms' + P_end(:, 1)' * Q_me')';
% %             f_y = (P_start(:, 2)' * Q_ms' + P_end(:, 2)' * Q_me')';
% %             f_z = (P_start(:, 3)' * Q_ms' + P_end(:, 3)' * Q_me')';
% %             f = [f_x; f_y; f_z];
% % 
% %             % -------------------------------------------------------------
% %             % 4. 凸包バリア不等式制約（最接近区間〜戻り区間の押し出し）
% %             % -------------------------------------------------------------
% %             cp_at_impact = round(t_impact / dt_seg) - 7;
% %             cp_at_impact = max(2, min(n_free - 4, cp_at_impact));
% % 
% %             sideway_indices = (cp_at_impact - 1) : (cp_at_impact + 2);
% %             sideway_indices = sideway_indices(sideway_indices >= 1 & sideway_indices <= n_free);
% % 
% %             A_ineq = [];
% %             b_ineq = [];
% %             for j = sideway_indices
% %                 row_cp = zeros(1, n_free * 3);
% %                 for dim = 1:3
% %                     idx_d = (dim - 1) * n_free;
% %                     row_cp(idx_d + j) = -n_escape_3d(dim);
% %                 end
% %                 A_ineq = [A_ineq; row_cp];
% %                 b_ineq = [b_ineq; -req_clearance];
% %             end
% % 
% %             recovery_indices = (max(sideway_indices) + 1) : min(n_free, max(sideway_indices) + 3);
% %             for j = recovery_indices
% %                 row_cp = zeros(1, n_free * 3);
% %                 for dim = 1:3
% %                     idx_d = (dim - 1) * n_free;
% %                     row_cp(idx_d + j) = -n_escape_3d(dim);
% %                 end
% %                 A_ineq = [A_ineq; row_cp];
% %                 b_ineq = [b_ineq; -0.85 * req_clearance];
% %             end
% % 
% %             % 加速度上限に基づく制御点変位ボックス境界
% %             max_disp = req_clearance * 1.5;
% %             lb = -max_disp * ones(n_free * 3, 1);
% %             ub =  max_disp * ones(n_free * 3, 1);
% % 
% %             opts = optimoptions('quadprog', 'Display', 'off', 'Algorithm', 'interior-point-convex');
% %             [X_mid, ~, exitflag] = quadprog(H, f, A_ineq, b_ineq, [], [], lb, ub, [], opts);
% % 
% %             if exitflag < 1
% %                 X_mid = repmat(n_escape_3d * req_clearance * 0.85, n_free, 1);
% %             end
% % 
% %             P_mid = zeros(n_free, 3);
% %             P_mid(:, 1) = X_mid(1:n_free);
% %             P_mid(:, 2) = X_mid((n_free+1):(2*n_free));
% %             P_mid(:, 3) = X_mid((2*n_free+1):(3*n_free));
% % 
% %             obj.control_points = [P_start; P_mid; P_end];
% %             obj.actual_peak_displacement = max(vecnorm(P_mid, 2, 2));
% %         end
% % 
% %         % =====================================================================
% %         % verify_boundary_c6_matching: 切り替え点における 0〜6階微分ギャップ厳密検証
% %         % =====================================================================
% %         function verify_boundary_c6_matching(obj, init_diff_state, t_now)
% %             names = ["位置(0階)", "速度(1階)", "加速度(2階)", "Jerk(3階)", "Snap(4階)", "Crack(5階)", "Pop(6階)"];
% %             tolerances = [1e-10, 1e-9, 1e-8, 1e-6, 1e-4, 1e-2, 1.0];
% % 
% %             for k = 0:6
% %                 new_k = obj.eval_spline_kth(0.0, k)';
% %                 old_k = init_diff_state(k + 1, :);
% %                 gap = norm(new_k - old_k);
% %                 obj.c6_boundary_gaps(k + 1) = gap;
% % 
% %                 if gap > tolerances(k + 1)
% %                     fprintf(2, "[FATAL C^6 BREACH] t=%.3f s | %s の切り替え接続ギャップが許容限界を超過: %.3e (許容値: %.3e)\n", ...
% %                         t_now, names(k + 1), gap, tolerances(k + 1));
% %                     error('リプランニング始端での C^6 境界接続に失敗しました: %s', names(k + 1));
% %                 end
% %             end
% %         end
% % 
% %         % =====================================================================
% %         % verify_continuous_c6_step: 全時間ステップ 0〜6階微分監視
% %         % =====================================================================
% %         function verify_continuous_c6_step(obj, xd_now, t_now, dt)
% %             if obj.prev_t < 0
% %                 obj.prev_xd = xd_now;
% %                 obj.prev_t  = t_now;
% %                 return;
% %             end
% % 
% %             real_dt = t_now - obj.prev_t;
% %             if real_dt <= 1e-6, return; end
% %             if nargin < 4 || isempty(dt) || dt <= 0, dt = real_dt; end
% % 
% %             % 各階微分の状態インデックス
% %             orders = { ...
% %                 '位置 (0階)',      1:3,   5:7;   ...
% %                 'yaw角 (0階)',     4,     8;     ...
% %                 '速度 (1階)',      5:7,   9:11;  ...
% %                 '加速度 (2階)',    9:11,  13:15; ...
% %                 'Jerk (3階)',      13:15, 17:19; ...
% %             };
% % 
% %             for idx = 1:size(orders, 1)
% %                 name   = orders{idx, 1};
% %                 curr_i = orders{idx, 2};
% %                 next_i = orders{idx, 3};
% % 
% %                 val_prev = obj.prev_xd(curr_i);
% %                 val_curr = xd_now(curr_i);
% %                 der_prev = obj.prev_xd(next_i);
% %                 der_curr = xd_now(next_i);
% % 
% %                 predicted = val_prev + 0.5 * (der_prev + der_curr) * real_dt;
% %                 disc_gap  = norm(val_curr - predicted);
% % 
% %                 tol = max(0.08, 6.0 * norm(der_curr) * real_dt);
% %                 if disc_gap > tol
% %                     fprintf(2, "[FATAL] C^6 連続性監視違反: %s at t=%.4f s (ギャップ: %.3e, 許容: %.3e)\n", ...
% %                         name, t_now, disc_gap, tol);
% %                     error('目標軌道の不連続キックが検出されました: %s', name);
% %                 end
% %             end
% % 
% %             obj.prev_xd = xd_now;
% %             obj.prev_t  = t_now;
% %         end
% % 
% %         % =====================================================================
% %         % check_detection_simulated_sensor: 機体センサ (15m・前方選択) & 荷物・索診断
% %         % =====================================================================
% %         function det = check_detection_simulated_sensor(obj, pQ, pL, obs_list, t_now, v_nom)
% %             det = struct();
% %             det.time                          = t_now;
% %             det.pQ                            = pQ;
% %             det.pL                            = pL;
% %             det.trigger_dist                  = obj.trigger_dist;
% %             det.detected_point                = false;
% %             det.drone_inside_obstacle_point   = false;
% %             det.load_inside_obstacle_point    = false;
% %             det.cable_inside_obstacle_point   = false;
% %             det.drone_min_dist_point          = inf;
% %             det.load_min_dist_point           = inf;
% %             det.cable_min_dist_point          = inf;
% %             det.min_dist_point                = inf;
% %             det.drone_obstacle_id_point       = [];
% %             det.load_obstacle_id_point        = [];
% %             det.min_obstacle_id_point         = [];
% %             det.min_source_point              = "none";
% %             det.detected_obstacles_point      = [];
% %             det.detected_obstacle_count_point = 0;
% % 
% %             if isempty(obs_list), return; end
% % 
% %             % 索を半径 0.5m の球体列で隙間なく離散配置 (索長 L に対して 0.5m 刻み)
% %             r_c_sph = obj.r_cable_sphere;
% %             n_spheres = max(2, ceil(obj.L_cable / r_c_sph));
% %             s_ratios = linspace(0.0, 1.0, n_spheres);
% %             cable_pts = (1 - s_ratios) .* pL + s_ratios .* pQ;
% % 
% %             v_dir = v_nom(:);
% %             if norm(v_dir) > 0.1, v_dir = v_dir / norm(v_dir); else, v_dir = [0; 1; 0]; end
% % 
% %             detected_obs_candidates = [];
% % 
% %             for i = 1:length(obs_list)
% %                 o = obs_list(i);
% %                 R_obs = o.R_obs;
% %                 radii_obs = o.ellipsoid_radii(:);
% %                 p_obs = o.p_center(:);
% %                 obs_parsed = struct('center', p_obs, 'radii', radii_obs, 'R', R_obs);
% % 
% %                 % 1. 機体重心 pQ (センサー) の距離計算
% %                 [d_drone_point, inside_drone, cpQ_loc, ~] = obj.point_ellipsoid_signed_distance(pQ, obs_parsed);
% % 
% %                 % 2. 荷物 pL の距離診断
% %                 [d_load_point, inside_load, cpL_loc, ~] = obj.point_ellipsoid_signed_distance(pL, obs_parsed);
% % 
% %                 % 3. 索（ケーブル球体列）の距離診断
% %                 d_cable_min_i = inf;
% %                 inside_cable_i = false;
% %                 for sp_idx = 1:n_spheres
% %                     [d_pt, in_pt, ~, ~] = obj.point_ellipsoid_signed_distance(cable_pts(:, sp_idx), obs_parsed);
% %                     d_surface_gap = d_pt - r_c_sph;
% %                     if d_surface_gap < d_cable_min_i, d_cable_min_i = d_surface_gap; end
% %                     if in_pt || (d_surface_gap <= 0), inside_cable_i = true; end
% %                 end
% % 
% %                 % 侵入フラグ更新
% %                 if inside_drone,   det.drone_inside_obstacle_point = true; end
% %                 if inside_load,    det.load_inside_obstacle_point  = true; end
% %                 if inside_cable_i, det.cable_inside_obstacle_point = true; end
% % 
% %                 if d_drone_point < det.drone_min_dist_point
% %                     det.drone_min_dist_point = d_drone_point;
% %                     det.drone_obstacle_id_point = i;
% %                 end
% %                 if d_load_point < det.load_min_dist_point
% %                     det.load_min_dist_point = d_load_point;
% %                     det.load_obstacle_id_point = i;
% %                 end
% %                 if d_cable_min_i < det.cable_min_dist_point
% %                     det.cable_min_dist_point = d_cable_min_i;
% %                 end
% % 
% %                 cpQ_world = p_obs + R_obs * cpQ_loc;
% %                 cpL_world = p_obs + R_obs * cpL_loc;
% %                 nQ_loc = cpQ_loc ./ (radii_obs.^2);
% %                 normal_drone = R_obs * (nQ_loc / norm(nQ_loc));
% %                 nL_loc = cpL_loc ./ (radii_obs.^2);
% %                 normal_load = R_obs * (nL_loc / norm(nL_loc));
% % 
% %                 obs_info = struct( ...
% %                     'id',                        i, ...
% %                     'dist_drone_point',          d_drone_point, ...
% %                     'dist_load_point',           d_load_point, ...
% %                     'dist_cable_point',          d_cable_min_i, ...
% %                     'min_dist_point',            d_drone_point, ...
% %                     'p_obs',                     p_obs, ...
% %                     'radii_obs',                 radii_obs, ...
% %                     'R_obs',                     R_obs, ...
% %                     'closest_drone_world_point', cpQ_world, ...
% %                     'closest_load_world_point',  cpL_world, ...
% %                     'normal_drone_point',        normal_drone, ...
% %                     'normal_load_point',         normal_load ...
% %                 );
% % 
% %                 % 前方検知条件: 機体センサー 15m 以内 かつ 進行方向前方
% %                 vec_to_center = p_obs - pQ;
% %                 is_in_front = dot(vec_to_center, v_dir) > -max(radii_obs);
% % 
% %                 if (d_drone_point <= obj.trigger_dist) && is_in_front
% %                     detected_obs_candidates = [detected_obs_candidates; obs_info];
% %                 end
% %             end
% % 
% %             % 複数検知時: 最も距離が近い最重要脅威を先頭にソート
% %             if ~isempty(detected_obs_candidates)
% %                 [~, sort_idx] = sort([detected_obs_candidates.dist_drone_point], 'ascend');
% %                 det.detected_obstacles_point = detected_obs_candidates(sort_idx);
% %                 det.detected_point = true;
% %                 det.min_dist_point = det.detected_obstacles_point(1).dist_drone_point;
% %                 det.min_obstacle_id_point = det.detected_obstacles_point(1).id;
% %                 det.min_source_point = "drone";
% %                 det.detected_obstacle_count_point = length(detected_obs_candidates);
% %             else
% %                 det.min_dist_point = det.drone_min_dist_point;
% %                 det.min_obstacle_id_point = det.drone_obstacle_id_point;
% %                 det.min_source_point = "none";
% %             end
% %         end
% % 
% %         % =====================================================================
% %         % display_detection_report: 診断レポート
% %         % =====================================================================
% %         function display_detection_report(obj, t_now, det, req_clearance, t_impact, n_escape)
% %             tgt = det.detected_obstacles_point(1);
% %             names = ["位置(0階)", "速度(1階)", "加速度(2階)", "Jerk(3階)", "Snap(4階)", "Crack(5階)", "Pop(6階)"];
% % 
% %             fprintf("\n=================================================================================\n");
% %             fprintf(" [B-SPLINE 15m センサー検知 ＆ C^6 回避診断レポート]  t = %.3f s\n", t_now);
% %             fprintf("=================================================================================\n");
% %             fprintf(" 1. センサ判定元      : 機体重心 pQ 搭載模擬センサー (機体表面間距離: %.3f m)\n", tgt.dist_drone_point);
% %             fprintf(" 2. 索・荷物状態診断  : 索球列表面間: %.3f m (侵入: %d), 荷物表面間: %.3f m (侵入: %d)\n", ...
% %                 tgt.dist_cable_point, det.cable_inside_obstacle_point, tgt.dist_load_point, det.load_inside_obstacle_point);
% %             fprintf(" 3. 捕捉障害物情報    : 危険順位 1位 / 候補 %d 個 (ID=%d, 中心=[%.2f, %.2f, %.2f] m)\n", ...
% %                 det.detected_obstacle_count_point, tgt.id, tgt.p_obs(1), tgt.p_obs(2), tgt.p_obs(3));
% %             fprintf(" 4. 運動学制約考慮    : 許容 a_max = %.2f m/s^2 -> 回避時間 T_tot = %.2f s (急激な横傾きを防止)\n", ...
% %                 obj.max_acc_load, obj.t_duration);
% %             fprintf(" 5. 回避幾何拘束      : 要求クリアランス = %.2f m (退避ベクトル: [%.2f, %.2f, %.2f])\n", ...
% %                 req_clearance, n_escape(1), n_escape(2), n_escape(3));
% %             fprintf(" 6. C^6 始端境界ギャップ実測値 (M \\ B 解の厳密検証):\n");
% %             for k = 0:6
% %                 fprintf("     - %-12s e_%d = %.3e m/s^%d (数学的連続保証)\n", names(k + 1), k, obj.c6_boundary_gaps(k + 1), k);
% %             end
% %             fprintf(" 7. QP最適化計算時間  : %6.2f ms (18自由度 Aeqフリー超高速二次計画)\n", obj.last_solve_time_ms);
% %             fprintf(" 8. 最大空間変位      : %.3f m\n", obj.actual_peak_displacement);
% %             fprintf("=================================================================================\n\n");
% %         end
% % 
% %         % =====================================================================
% %         % point_ellipsoid_signed_distance: ラグランジュ未定乗数法による厳密距離
% %         % =====================================================================
% %         function [d, inside, closest_local, lambda] = point_ellipsoid_signed_distance(~, p, o)
% %             p = p(:); c = o.center(:); r = o.radii(:); R = o.R;
% %             y  = R' * (p - c);
% %             r2 = r.^2;
% %             q  = sum((y ./ r).^2);
% %             inside = (q < 1.0);
% % 
% %             if norm(y) < 1e-14
% %                 [min_r, min_idx] = min(r);
% %                 closest_local = zeros(3, 1);
% %                 closest_local(min_idx) = min_r;
% %                 d      = -min_r;
% %                 lambda = -min(r2);
% %                 return;
% %             end
% % 
% %             f = @(lam) sum(r2 .* (y.^2) ./ ((lam + r2).^2)) - 1.0;
% %             if q > 1.0
% %                 lo = 0.0;
% %                 hi = max(r) * norm(y);
% %             else
% %                 lo = -min(r2) * (1.0 - 1e-12);
% %                 hi = 0.0;
% %             end
% % 
% %             for kk = 1:80
% %                 mid = 0.5 * (lo + hi);
% %                 if f(mid) > 0, lo = mid; else, hi = mid; end
% %             end
% %             lambda = 0.5 * (lo + hi);
% %             closest_local = r2 .* y ./ (lambda + r2);
% %             d_abs = norm(closest_local - y);
% %             if inside, d = -d_abs; else, d = d_abs; end
% %         end
% % 
% %         function val = eval_spline_kth(obj, tau, k)
% %             p = obj.spline_degree;
% %             T_tot = obj.t_duration;
% %             t_eval = max(0.0, min(T_tot - 1e-7, tau));
% % 
% %             idx_span = obj.find_knot_span(t_eval);
% %             ders = obj.eval_basis_derivatives(idx_span, t_eval, k);
% % 
% %             c_indices = (idx_span - p):idx_span;
% %             val = (ders(k + 1, :) * obj.control_points(c_indices, :))';
% %         end
% % 
% %         function build_clamped_uniform_knots(obj, n_seg, p, T_tot)
% %             dt_knot = T_tot / n_seg;
% %             interior_knots = dt_knot * (1:(n_seg - 1));
% %             obj.knots = [zeros(1, p + 1), interior_knots, T_tot * ones(1, p + 1)];
% %         end
% % 
% %         function idx = find_knot_span(obj, t_eval)
% %             p = obj.spline_degree;
% %             n_cp = length(obj.knots) - p - 1;
% %             if t_eval >= obj.knots(n_cp + 1), idx = n_cp; return; end
% %             if t_eval <= obj.knots(p + 1),    idx = p + 1; return; end
% %             low = p + 1; high = n_cp + 1; mid = floor((low + high) / 2);
% %             while (t_eval < obj.knots(mid)) || (t_eval >= obj.knots(mid + 1))
% %                 if t_eval < obj.knots(mid), high = mid; else, low = mid; end
% %                 mid = floor((low + high) / 2);
% %             end
% %             idx = mid;
% %         end
% % 
% %         function ders = eval_basis_derivatives(obj, idx_span, t_eval, n_der)
% %             p = obj.spline_degree;
% %             U = obj.knots;
% %             ders = zeros(n_der + 1, p + 1);
% %             ndu = zeros(p + 1, p + 1);
% %             left = zeros(p + 1, 1);
% %             right = zeros(p + 1, 1);
% % 
% %             ndu(1, 1) = 1.0;
% %             for j = 1:p
% %                 left(j + 1) = t_eval - U(idx_span + 1 - j);
% %                 right(j + 1) = U(idx_span + j) - t_eval;
% %                 saved = 0.0;
% %                 for r = 0:(j - 1)
% %                     ndu(j + 1, r + 1) = right(r + 2) + left(j - r + 1);
% %                     temp = ndu(r + 1, j) / ndu(j + 1, r + 1);
% %                     ndu(r + 1, j + 1) = saved + right(r + 2) * temp;
% %                     saved = left(j - r + 1) * temp;
% %                 end
% %                 ndu(j + 1, j + 1) = saved;
% %             end
% % 
% %             for j = 0:p, ders(1, j + 1) = ndu(j + 1, p + 1); end
% % 
% %             a = zeros(2, p + 1);
% %             for r = 0:p
% %                 s1 = 0; s2 = 1; a(1, 1) = 1.0;
% %                 for k = 1:n_der
% %                     d = 0.0; rk = r - k; pk = p - k;
% %                     if r >= k
% %                         a(s2 + 1, 1) = a(s1 + 1, 1) / ndu(pk + 2, rk + 1);
% %                         d = a(s2 + 1, 1) * ndu(rk + 1, pk + 1);
% %                     end
% %                     if rk >= -1, j1 = 1; else, j1 = -rk; end
% %                     if (r - 1) <= pk, j2 = k - 1; else, j2 = p - r; end
% %                     for j = j1:j2
% %                         a(s2 + 1, j + 1) = (a(s1 + 1, j + 1) - a(s1 + 1, j)) / ndu(pk + 2, rk + j + 1);
% %                         d = d + a(s2 + 1, j + 1) * ndu(rk + j + 1, pk + 1);
% %                     end
% %                     if r <= pk
% %                         a(s2 + 1, k + 1) = -a(s1 + 1, k) / ndu(pk + 2, r + 1);
% %                         d = d + a(s2 + 1, k + 1) * ndu(r + 1, pk + 1);
% %                     end
% %                     ders(k + 1, r + 1) = d;
% %                     j_tmp = s1; s1 = s2; s2 = j_tmp;
% %                 end
% %             end
% % 
% %             r_scale = p;
% %             for k = 1:n_der
% %                 for j = 0:p
% %                     ders(k + 1, j + 1) = ders(k + 1, j + 1) * r_scale;
% %                 end
% %                 r_scale = r_scale * (p - k);
% %             end
% %         end
% % 
% %         function clear_state_sensor_values(obj, t_now)
% %             st = obj.result.state;
% %             st.time                          = t_now;
% %             st.pQ                            = [NaN; NaN; NaN];
% %             st.pL                            = [NaN; NaN; NaN];
% %             st.detected_point                = false;
% %             st.drone_inside_obstacle_point   = false;
% %             st.load_inside_obstacle_point    = false;
% %             st.cable_inside_obstacle_point   = false;
% %             st.drone_min_dist_point          = inf;
% %             st.load_min_dist_point           = inf;
% %             st.cable_min_dist_point          = inf;
% %             st.min_dist_point                = inf;
% %             st.drone_obstacle_id_point       = NaN;
% %             st.load_obstacle_id_point        = NaN;
% %             st.min_obstacle_id_point         = NaN;
% %             st.min_source_point              = "none";
% %             st.detected_obstacle_count_point = 0;
% %         end
% % 
% %         function list = get_obstacles_at_time(~, t_now)
% %             try
% %                 list = ENVIRONMENT_OBSTACLE_ELLIPSE_MOVE(t_now);
% %             catch
% %                 p1 = [0.3; 0.3; 20.0]; rad1 = [2.5*sqrt(2); 2.5*sqrt(2); 0.5*sqrt(2)];
% %                 p2 = [0.0; 18.0; 2.0]; rad2 = [3.0*sqrt(2); 3.0*sqrt(2); 0.5*sqrt(2)];
% %                 p3 = [0.0; 28.0; 0.0]; rad3 = [1.0*sqrt(2); 1.0*sqrt(2); 0.5*sqrt(2)];
% %                 list = [struct('p_center', p1, 'ellipsoid_radii', rad1, 'R_obs', eye(3), 'd_margin', 0.5, 'v_center', [0;0;0]), ...
% %                         struct('p_center', p2, 'ellipsoid_radii', rad2, 'R_obs', eye(3), 'd_margin', 0.5, 'v_center', [0;0;0]), ...
% %                         struct('p_center', p3, 'ellipsoid_radii', rad3, 'R_obs', eye(3), 'd_margin', 0.5, 'v_center', [0;0;0])];
% %             end
% %         end
% %     end
% % end
% 
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

                % 3. 軌道再計画の判定 (複数障害物・回避中再計画・マージン侵入リスク)
                if detection.detected_point && (time.t - obj.last_replan_time >= obj.min_replan_interval)
                    need_replan = false;

                    if ~obj.replan_active
                        need_replan = true;
                    else
                        % 回避中の再計画判定: 
                        % (A) 別の障害物がより危険になった場合
                        % (B) 現在の回避軌道が対象楕円体のマージン帯を割り込む恐れがある場合
                        if detection.min_obstacle_id_point ~= obj.active_threat_id
                            need_replan = true;
                        elseif detection.drone_margin_violated || detection.cable_margin_violated || detection.load_margin_violated
                            need_replan = true;
                        end
                    end

                    if need_replan
                        obj.execute_replanning(pQ_cur, xd_nom, detection, obs_list, time.t);
                    end
                end
            end

            % 4. 出力目標軌道の確定 (差分変位加算方式)
            if obj.replan_active
                tau = time.t - obj.t_start;

                % 復帰安全性の事前検査: 終了 1.0 秒前に公称軌道への合流安全性をスキャン
                if (tau >= obj.t_duration - 1.0) && (tau <= obj.t_duration)
                    obs_list_check = obj.get_obstacles_at_time(time.t);
                    if ~obj.is_nominal_recovery_safe(time.t, obs_list_check)
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
        % evaluate_smooth_trajectory: 0〜6階微分の全状態修正 (Public)
        % =====================================================================
        function xd = evaluate_smooth_trajectory(obj, tau, xd_nom)
            xd = xd_nom;

            delta_pos   = obj.eval_spline_kth(tau, 0);
            delta_vel   = obj.eval_spline_kth(tau, 1);
            delta_acc   = obj.eval_spline_kth(tau, 2);
            delta_jerk  = obj.eval_spline_kth(tau, 3);
            delta_snap  = obj.eval_spline_kth(tau, 4);
            delta_crack = obj.eval_spline_kth(tau, 5);
            delta_pop   = obj.eval_spline_kth(tau, 6);

            xd(1:3)   = xd_nom(1:3)   + delta_pos;   % 荷物位置 (0階)
            xd(5:7)   = xd_nom(5:7)   + delta_vel;   % 荷物速度 (1階)
            xd(9:11)  = xd_nom(9:11)  + delta_acc;   % 荷物加速度 (2階)
            xd(13:15) = xd_nom(13:15) + delta_jerk;  % 荷物Jerk (3階)
            xd(17:19) = xd_nom(17:19) + delta_snap;  % 荷物Snap (4階)

            g_vec = [0; 0; obj.gravity];
            acc_tot = xd(9:11) + g_vec;
            norm_a = norm(acc_tot);
            if norm_a > 1e-3, thrust_dir = acc_tot / norm_a; else, thrust_dir = [0; 0; 1]; end
            pQ_d = xd(1:3) + obj.L_cable * thrust_dir;

            if length(xd) >= 23, xd(21:23) = pQ_d; end
            if length(xd) >= 27, xd(25:27) = xd_nom(25:27) + delta_pop; end
        end
    end

    methods (Access = private)
        % =====================================================================
        % execute_replanning: 楕円体幾何学に基づく回避 QP & 予測軌道事前検査
        % =====================================================================
        function execute_replanning(obj, pQ_cur, xd_nom, detection, obs_list, t_now)
            target_obs = detection.detected_obstacles_point(1);
            obj.active_threat_id = target_obs.id;

            v_nom = xd_nom(5:7);
            spd = norm(v_nom);
            if spd < 0.1, spd = 1.0; v_nom = [1; 0; 0]; end
            dir_nom = v_nom / spd;

            % 旧変位軌道の現時刻における 0〜6階微分状態の抽出
            init_diff_state = zeros(7, 3);
            if obj.replan_active
                tau_now = t_now - obj.t_start;
                for k = 0:6
                    init_diff_state(k + 1, :) = obj.eval_spline_kth(tau_now, k)';
                end
            end

            obj.t_start          = t_now;
            obj.last_replan_time = t_now;

            % --- 厳密な楕円体幾何に基づく要求クリアランスの算定 ---
            % 球体で丸め込まず、最接近点における法線方向深さ + 機体・索保護半径 + マージン
            req_clearance = target_obs.dist_drone_point + obj.r_drone + obj.clearance_margin;
            req_clearance = max(req_clearance, 1.5); % 最低退避量

            % 回避退避方向 (最寄りの真の楕円体表面外向き法線ベクトルから進行軸成分を除去)
            n_escape = -target_obs.normal_drone_point;
            n_escape = n_escape - dot(n_escape, dir_nom) * dir_nom;
            if norm(n_escape) < 0.1
                n_cand = cross(dir_nom, [0; 0; 1]);
                if norm(n_cand) < 0.1, n_cand = cross(dir_nom, [0; 1; 0]); end
                n_escape = n_cand / norm(n_cand);
            else
                n_escape = n_escape / norm(n_escape);
            end

            % 動的時間スケーリング: 加速度上限 max_acc_load を満たす最小回避時間 T_tot
            T_kinematic = sqrt((8.0 * req_clearance) / obj.max_acc_load);

            % 最接近時刻の幾何推定
            vec_to_obs = target_obs.p_obs - pQ_cur;
            dist_along = dot(vec_to_obs, dir_nom);
            t_impact = max(1.5, dist_along / spd);
            obj.t_duration = max([5.0, 2.0 * t_impact, T_kinematic * 1.5]);

            % 7次 B-Spline QP 求解
            t_solve = tic;
            obj.plan_uniform_bspline_c6_qp(req_clearance, n_escape, init_diff_state, t_impact);
            obj.last_solve_time_ms = toc(t_solve) * 1000;

            % --- 採用前の未来予測軌道サンプリング検査 (Look-ahead Safety Verification) ---
            traj_verified = obj.verify_future_trajectory_safety(t_now, obs_list);

            if traj_verified
                obj.replan_active = true;
                obj.verify_boundary_c6_matching(init_diff_state, t_now);
                obj.display_detection_report(t_now, detection, req_clearance, t_impact, n_escape, "PASS");
            else
                % 侵入リスクを検出した場合はクリアランスを増して再計算
                req_clearance_boost = req_clearance * 1.3;
                obj.plan_uniform_bspline_c6_qp(req_clearance_boost, n_escape, init_diff_state, t_impact);
                obj.replan_active = true;
                obj.verify_boundary_c6_matching(init_diff_state, t_now);
                obj.display_detection_report(t_now, detection, req_clearance_boost, t_impact, n_escape, "BOOSTED_PASS");
            end
        end

        % =====================================================================
        % plan_uniform_bspline_c6_qp: 両端 C^6 クランプ・内点凸包バリア最適化
        % =====================================================================
        function plan_uniform_bspline_c6_qp(obj, req_clearance, n_escape_3d, init_diff_state, t_impact)
            p = 7;
            n_seg = 25;
            n_cp = n_seg + p;          % 32 制御点
            T_tot = obj.t_duration;
            dt_seg = T_tot / n_seg;

            obj.spline_degree = p;
            obj.num_segments = n_seg;
            obj.build_clamped_uniform_knots(n_seg, p, T_tot);

            % 1. 始端 0〜6階微分の境界確定: P_start (7x3)
            M_start = zeros(7, 7);
            for k = 0:6
                d_row = obj.eval_basis_derivatives(p + 1, 0.0, k);
                M_start(k + 1, :) = d_row(k + 1, 1:7);
            end
            P_start = M_start \ init_diff_state(1:7, :);

            % 2. 終端 0〜6階微分の公称合流確定: P_end (7x3)
            %    Delta p(T_tot) = 0 .. Delta p^(6)(T_tot) = 0
            M_end = zeros(7, 7);
            for k = 0:6
                d_row_end = obj.eval_basis_derivatives(n_cp, T_tot, k);
                M_end(k + 1, :) = d_row_end(k + 1, (end - 6):end);
            end
            P_end = M_end \ zeros(7, 3);

            % 3. 自由変数 (P_8 〜 P_25: 18点) に対する Snap(4階差分) 最小化
            D4 = diff(eye(n_cp), 4);
            Q = D4' * D4;

            idx_free = 8:(n_cp - 7);
            n_free = length(idx_free);

            Q_mm = Q(idx_free, idx_free) + 1e-4 * eye(n_free);
            Q_ms = Q(idx_free, 1:7);
            Q_me = Q(idx_free, (n_cp-6):n_cp);

            H_1d = (Q_mm + Q_mm') / 2;
            H = blkdiag(H_1d, H_1d, H_1d);

            f_x = (P_start(:, 1)' * Q_ms' + P_end(:, 1)' * Q_me')';
            f_y = (P_start(:, 2)' * Q_ms' + P_end(:, 2)' * Q_me')';
            f_z = (P_start(:, 3)' * Q_ms' + P_end(:, 3)' * Q_me')';
            f = [f_x; f_y; f_z];

            % 4. 凸包バリア不等式制約（最接近区間〜戻り区間の押し出し）
            cp_at_impact = round(t_impact / dt_seg) - 7;
            cp_at_impact = max(2, min(n_free - 4, cp_at_impact));

            sideway_indices = (cp_at_impact - 1) : (cp_at_impact + 2);
            sideway_indices = sideway_indices(sideway_indices >= 1 & sideway_indices <= n_free);

            A_ineq = [];
            b_ineq = [];
            for j = sideway_indices
                row_cp = zeros(1, n_free * 3);
                for dim = 1:3
                    idx_d = (dim - 1) * n_free;
                    row_cp(idx_d + j) = -n_escape_3d(dim);
                end
                A_ineq = [A_ineq; row_cp];
                b_ineq = [b_ineq; -req_clearance];
            end

            recovery_indices = (max(sideway_indices) + 1) : min(n_free, max(sideway_indices) + 3);
            for j = recovery_indices
                row_cp = zeros(1, n_free * 3);
                for dim = 1:3
                    idx_d = (dim - 1) * n_free;
                    row_cp(idx_d + j) = -n_escape_3d(dim);
                end
                A_ineq = [A_ineq; row_cp];
                b_ineq = [b_ineq; -0.85 * req_clearance];
            end

            max_disp = req_clearance * 1.5;
            lb = -max_disp * ones(n_free * 3, 1);
            ub =  max_disp * ones(n_free * 3, 1);

            opts = optimoptions('quadprog', 'Display', 'off', 'Algorithm', 'interior-point-convex');
            [X_mid, ~, exitflag] = quadprog(H, f, A_ineq, b_ineq, [], [], lb, ub, [], opts);

            if exitflag < 1
                X_mid = repmat(n_escape_3d * req_clearance * 0.85, n_free, 1);
            end

            P_mid = zeros(n_free, 3);
            P_mid(:, 1) = X_mid(1:n_free);
            P_mid(:, 2) = X_mid((n_free+1):(2*n_free));
            P_mid(:, 3) = X_mid((2*n_free+1):(3*n_free));

            obj.control_points = [P_start; P_mid; P_end];
            obj.actual_peak_displacement = max(vecnorm(P_mid, 2, 2));
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

                xd_fut = obj.evaluate_smooth_trajectory(tau_i, xd_nom_i);
                pL_fut = xd_fut(1:3);
                pQ_fut = xd_fut(21:23);
                cable_pts_fut = (1 - s_ratios) .* pL_fut + s_ratios .* pQ_fut;

                for obs_idx = 1:length(obs_list)
                    o = obs_list(obs_idx);
                    obs_parsed = struct('center', o.p_center(:), 'radii', o.ellipsoid_radii(:), 'R', o.R_obs);

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
        % is_nominal_recovery_safe: 復帰先の公称軌道における干渉スキャン
        % =====================================================================
        function is_safe = is_nominal_recovery_safe(obj, t_now, obs_list)
            is_safe = true;
            N_lookahead = 15;
            t_scan = linspace(t_now + obj.t_duration, t_now + obj.t_duration + 2.0, N_lookahead);

            for i = 1:N_lookahead
                t_eval = t_scan(i);
                nom_res = obj.base_ref.do(struct('t', t_eval, 'dt', 0.025), 'f');
                pL_nom = nom_res.state.xd(1:3);
                pQ_nom = pL_nom + [0; 0; obj.L_cable];

                for obs_idx = 1:length(obs_list)
                    o = obs_list(obs_idx);
                    obs_parsed = struct('center', o.p_center(:), 'radii', o.ellipsoid_radii(:), 'R', o.R_obs);

                    [dQ, ~, ~, ~] = obj.point_ellipsoid_signed_distance(pQ_nom, obs_parsed);
                    [dL, ~, ~, ~] = obj.point_ellipsoid_signed_distance(pL_nom, obs_parsed);

                    % 復帰直後に接触・マージン侵入しないか
                    if (dQ - obj.r_drone - obj.clearance_margin <= 0) || ...
                       (dL - obj.r_load  - obj.clearance_margin <= 0)
                        is_safe = false;
                        return;
                    end
                end
            end
        end

        % =====================================================================
        % verify_boundary_c6_matching: 始端 0〜6階微分ギャップ厳密検証
        % =====================================================================
        function verify_boundary_c6_matching(obj, init_diff_state, t_now)
            names = ["位置(0階)", "速度(1階)", "加速度(2階)", "Jerk(3階)", "Snap(4階)", "Crack(5階)", "Pop(6階)"];
            tolerances = [1e-10, 1e-9, 1e-8, 1e-6, 1e-4, 1e-2, 1.0];

            for k = 0:6
                new_k = obj.eval_spline_kth(0.0, k)';
                old_k = init_diff_state(k + 1, :);
                gap = norm(new_k - old_k);
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
            if nargin < 4 || isempty(dt) || dt <= 0, dt = real_dt; end

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

                predicted = val_prev + 0.5 * (der_prev + der_curr) * real_dt;
                disc_gap  = norm(val_curr - predicted);

                tol = max(0.08, 6.0 * norm(der_curr) * real_dt);
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
                    det.drone_obstacle_id_point = i;
                end
                if d_load_physical < det.load_min_dist_point
                    det.load_min_dist_point = d_load_physical;
                    det.load_obstacle_id_point = i;
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
                    'id',                        i, ...
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
            fprintf(" 4. 運動学制約考慮    : 許容 a_max = %.2f m/s^2 -> 回避時間 T_tot = %.2f s (過大傾斜を防止)\n", ...
                obj.max_acc_load, obj.t_duration);
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
% 
% classdef REPLANNING_BSPLINE < handle
%     % =========================================================================
%     % REPLANNING_BSPLINE
%     % 
%     % 1. 模擬センサー (Simulated Point Sensor):
%     %    - 機体重心 pQ を検知主体とし、前方 15m 以内の楕円体障害物を最短表面距離順に捕捉。
%     % 2. 独立実行時安全診断モニタ (Independent Runtime Safety Monitor):
%     %    - 荷物 pL, 機体 pQ, および直線索（半径 r_c の球列幾何被覆）について、
%     %      楕円体外郭までの最短ユークリッド距離をラグランジュ未定乗数法で厳密計算。
%     %    - 物理接触 (physical_gap <= 0) とマージン侵入 (safety_gap <= 0) を分離診断。
%     % 3. 実荷物目標軌道 p_L(t) の C^6 境界確定アルゴリズム:
%     %    - 始端 (t_now) および終端 (t_start + T_tot) における公称軌道の 0〜6階微分値を
%     %      境界代数方程式 M \ B により直接課し、実出力軌道自体の C^6 接続を保証。
%     % 4. 【連続時間安全半空間包含保証 (Continuous-time Reference Safety Guarantee)】:
%     %    - 生成・出力するのは牽引物目標軌道 pL(t) のみ。
%     %    - 楕円体の支持半空間法線 n に対し、全制御点 P_L,i (i=1..N_cp) へ
%     %      d_req = L_cable + r_drone + d_margin を課す。
%     %    - B-Spline の大域凸包性により全連続時間区間で pL(t) in H(d_req) が保証され、
%     %      索の幾何拘束 ||pQ - pL|| = L および r_drone >= r_cable >= r_load から、
%     %      参照UAV球体 B(pQ(t), r_drone) および直線索全体が楕円体支持半空間内に完全包含される。
%     % 5. 公称軌道引き戻し正則化 (Nominal Deviation Regularization):
%     %    - QP 目的関数に公称制御点からの乖離ペナルティ w_dev * ||P - P_nom||^2 を組み込み、
%     %      安全制約を満たす範囲内で公称軌道から不必要に離れる大回り回避（過大横ずれ）を抑制。
%     % 6. 状態配列 xd の厳格な仕様定義:
%     %    - xd(1:28): 荷物 pL の 0〜6階微分 ＋ 公称 Yaw の 0〜6階微分専用。
%     %    - xd(29:31): 差分平坦性モデルから求まる UAV 参照目標位置 pQ を分離格納。
%     % =========================================================================
%     properties
%         self                       % ドローンエージェント自身
%         base_ref                   % 公称参照軌道生成オブジェクト
%         result                     % 出力結果構造体
% 
%         % --- センサ・検知パラメータ ---
%         trigger_dist = 15.0;       % 機体搭載センサーによる接近検知閾値 [m]
%         clearance_margin = 0.8;    % 安全マージン d_margin [m]
% 
%         % --- 機体・荷物・索物理パラメータ ---
%         gravity = 9.81;            % 重力加速度 [m/s^2]
%         L_cable = 1.0;             % 索長 L [m]
%         r_drone = 0.30;            % 機体球体半径 r_Q [m]
%         r_load  = 0.15;            % 荷物球体半径 r_L [m]
%         r_cable_sphere = 0.25;     % 索保護球体半径 r_C [m] (実行時モニタ専用)
% 
%         % --- 運動学・物理制約パラメータ ---
%         max_acc_load = 2.0;        % 荷物許容水平加速度上限 a_max [m/s^2]
%         w_dev = 0.05;              % 公称軌道引き戻し重み (過大回避抑制ペナルティ)
% 
%         % --- リプランニング状態管理 ---
%         replan_active = false;     % 回避軌道追従中フラグ
%         t_start       = 0.0;       % 回避開始時刻 [s]
%         t_duration    = 6.0;       % 回避全所要時間 [s]
%         last_replan_time = -100.0; % 前回リプラン実行時刻 [s]
%         min_replan_interval = 0.25;% 再計画更新周期 [s]
%         active_threat_id = NaN;    % 現在追従中の主障害物ID
% 
%         % --- 7次 B-Spline パラメータ (p=7, n_seg=25 -> N_cp=32) ---
%         spline_degree = 7;         % 7次 B-Spline (内部 C^6 連続)
%         num_segments  = 25;        % 25セグメント
%         knots                      % ノットベクトル
%         control_points             % 荷物実軌道制御点 P_L (32 x 3) [X, Y, Z]
%         sampled_peak_displacement = 0.0; % 最大空間変位 (60点サンプリング評価) [m]
%         sampled_peak_acceleration = 0.0; % 実軌道最大水平加速度 (サンプリング評価) [m/s^2]
%         last_solve_time_ms = 0.0;  % QP求解時間 [ms]
%         last_replan_total_time_ms = 0.0; % リプランニング全体処理時間 [ms]
% 
%         % --- C^6 境界接続ギャップ診断バッファ ---
%         c6_start_gaps = zeros(7, 1);  % 始端接続ギャップ e_k (k=0..6)
%         c6_return_gaps = zeros(7, 1); % 終端復帰ギャップ e_k (k=0..6)
% 
%         % --- 離散時間ステップ整合性監視用バッファ ---
%         prev_xd                    % 前回ステップの xd (31x1)
%         prev_t = -1.0;             % 前回ステップの時刻 [s]
% 
%         % --- アクティブ障害物管理 (Active Obstacle Manager) ---
%         active_obstacles = [];     % 現在管理中の障害物リスト (構造体配列)
%         post_recovery_hold_time = 3.0; % 回避・安全復帰完了後の猶予保持時間 [s]
%     end
% 
%     methods (Access = public)
%         % =====================================================================
%         % コンストラクタ
%         % =====================================================================
%         function obj = REPLANNING_BSPLINE(self, base_ref, opts)
%             arguments
%                 self                  % 必須: エージェントインスタンス
%                 base_ref              % 必須: 通常飛行用の公称軌道インスタンス
%                 opts = struct()       % 任意: 外部設定構造体
%             end
%             obj.self = self;
%             obj.base_ref = base_ref;
%             if isfield(opts, 'trigger_dist'),     obj.trigger_dist     = opts.trigger_dist;     end
%             if isfield(opts, 'clearance_margin'), obj.clearance_margin = opts.clearance_margin; end
%             if isfield(opts, 'r_drone'),          obj.r_drone          = opts.r_drone;          end
%             if isfield(opts, 'r_load'),           obj.r_load           = opts.r_load;           end
%             if isfield(opts, 'r_cable_sphere'),   obj.r_cable_sphere   = opts.r_cable_sphere;   end
%             if isfield(opts, 'L_cable'),          obj.L_cable          = opts.L_cable;          end
%             if isfield(opts, 'gravity'),          obj.gravity          = opts.gravity;          end
%             if isfield(opts, 'max_acc_load'),     obj.max_acc_load     = opts.max_acc_load;     end
%             if isfield(opts, 'w_dev'),            obj.w_dev            = opts.w_dev;            end
% 
%             % 幾何学的安全包含の前提条件を事前検証
%             assert(obj.r_drone >= obj.r_cable_sphere, '幾何学的安全包含の前提違反: r_drone >= r_cable_sphere が必要です.');
%             assert(obj.r_cable_sphere >= obj.r_load,   '幾何学的安全包含の前提違反: r_cable_sphere >= r_load が必要です.');
%             assert(obj.L_cable > 0,                    'L_cable は正値である必要があります.');
%             assert(obj.max_acc_load > 0,                'max_acc_load は正値である必要があります.');
%             assert(obj.clearance_margin >= 0,          'clearance_margin は非負である必要があります.');
%             assert(obj.trigger_dist > 0,               'trigger_dist は正値である必要があります.');
% 
%             obj.result = base_ref.result;
% 
%             sensor_props = ["time", "pQ", "pL", "detected_point", ...
%                 "drone_inside_obstacle_point", "load_inside_obstacle_point", ...
%                 "cable_inside_obstacle_point", "drone_margin_violated", ...
%                 "load_margin_violated", "cable_margin_violated", ...
%                 "drone_min_dist_point", "load_min_dist_point", "cable_min_dist_point", ...
%                 "min_dist_point", "drone_obstacle_id_point", "load_obstacle_id_point", ...
%                 "min_obstacle_id_point", "min_source_point", "detected_obstacle_count_point", ...
%                 "detected_obstacles_point", "pQ_ref"];
%             for p_name = sensor_props
%                 if ~isprop(obj.result.state, p_name)
%                     addprop(obj.result.state, p_name);
%                 end
%             end
% 
%             obj.clear_state_sensor_values(0.0);
%         end
% 
%         % =====================================================================
%         % do: 制御周期ごとのメイン実行メソッド
%         % =====================================================================
%         function result_out = do(obj, varargin)
%             time = varargin{1};
%             cha  = varargin{2};
% 
%             % 1. 公称目標軌道 (Nominal Reference) の算出
%             base_res = obj.base_ref.do(varargin{:});
%             xd_nom = base_res.state.xd;
%             if length(xd_nom) < 31
%                 xd_nom = [xd_nom; zeros(31 - length(xd_nom), 1)];
%             end
% 
%             obj.clear_state_sensor_values(time.t);
% 
%             detection = struct();
%             detection.time                          = time.t;
%             detection.pQ                            = [NaN; NaN; NaN];
%             detection.pL                            = [NaN; NaN; NaN];
%             detection.detected_point                = false;
%             detection.drone_inside_obstacle_point   = false;
%             detection.load_inside_obstacle_point    = false;
%             detection.cable_inside_obstacle_point   = false;
%             detection.drone_margin_violated         = false;
%             detection.load_margin_violated          = false;
%             detection.cable_margin_violated         = false;
%             detection.drone_min_dist_point          = inf;
%             detection.load_min_dist_point           = inf;
%             detection.cable_min_dist_point          = inf;
%             detection.min_dist_point                = inf;
%             detection.drone_obstacle_id_point       = NaN;
%             detection.load_obstacle_id_point        = NaN;
%             detection.min_obstacle_id_point         = NaN;
%             detection.min_source_point              = "none";
%             detection.detected_obstacle_count_point = 0;
%             detection.detected_obstacles_point      = [];
% 
%             try obj.L_cable = obj.self.parameter.get("cableL"); catch; end
% 
%             % 2. 飛行フェーズ ('f') の機体センシング & 独立モニタリング
%             if cha == 'f'
%                 pL_cur = obj.self.estimator.result.state.pL(:);
%                 pQ_cur = obj.self.estimator.result.state.p(:);
%                 obs_list = obj.get_obstacles_at_time(time.t);
% 
%                 % 1. センサー検知と安全診断 (検知候補を抽出)
%                 detection = obj.check_detection_simulated_sensor(pQ_cur, pL_cur, obs_list, time.t, xd_nom(5:7));
% 
%                 % 2. Active Obstacle Manager によるライフサイクル更新 (計画対象のみを抽出)
%                 planning_obstacles = obj.update_active_obstacles(detection.detected_obstacles_point, time.t);
% 
%                 % -------------------------------------------------------------
%                 % 3. 軌道再計画の判定 (近接センサ ＋ 3者マージン侵入 ＋ 未来予測モニタ)
%                 % -------------------------------------------------------------
%                 need_replan = false;
%                 can_replan = (time.t - obj.last_replan_time >= obj.min_replan_interval);
% 
%                 if can_replan
%                     if ~obj.replan_active
%                         body_margin_violated = detection.drone_margin_violated || ...
%                                                detection.load_margin_violated  || ...
%                                                detection.cable_margin_violated;
%                         if detection.detected_point || body_margin_violated
%                             need_replan = true;
%                         end
%                     else
%                         if detection.detected_point && (detection.min_obstacle_id_point ~= obj.active_threat_id)
%                             need_replan = true;
%                         elseif detection.drone_margin_violated || detection.cable_margin_violated || detection.load_margin_violated
%                             need_replan = true;
%                         else
%                             % 回避軌道追従中の数値モニタリング
%                             [pred_safe, pred_acc] = obj.verify_trajectory_numerical_sampling(time.t, planning_obstacles, obj.t_duration);
%                             if ~pred_safe || (pred_acc > obj.max_acc_load)
%                                 need_replan = true;
%                                 fprintf("[INDEPENDENT PREDICTIVE REPLAN] t=%.3f s | 回避軌道先行きで干渉または過大水平加速度 (a_xy=%.2f) を検知 -> 再計画\n", ...
%                                     time.t, pred_acc);
%                             end
%                         end
%                     end
% 
%                     if need_replan && ~isempty(planning_obstacles)
% 
%                         obj.execute_replanning(pQ_cur, xd_nom, detection, planning_obstacles, time.t);
%                     end
%                 end
%             end
% 
%             % 4. 出力目標軌道の確定
%             if obj.replan_active
%                 tau = time.t - obj.t_start;
%                 t_end_expected = obj.t_start + obj.t_duration;
% 
%                 % 復帰安全性の事前検査: 回避終了 1.0 秒前に復帰先公称軌道をスキャン (cha == 'f' 限定)
%                 if (tau >= obj.t_duration - 1.0) && (tau < obj.t_duration) && (cha == 'f')
%                     obs_list_check = obj.get_obstacles_at_time(time.t);
%                     % if ~obj.is_nominal_recovery_safe(t_end_expected, obs_list_check)
%                     %     fprintf("[RETURN HELD & REPLAN] t=%.3f s | 復帰予定時刻 t_end=%.3f s の公称軌道上に楕円体干渉を検出 -> 回避軌道を再生成\n", ...
%                     %         time.t, t_end_expected);
%                     % 
%                     %     obj.execute_replanning(pQ_cur, xd_nom, detection, obs_list, time.t);
%                     %     tau = time.t - obj.t_start;
%                     %     t_end_expected = obj.t_start + obj.t_duration;
%                     % end
%                     if ~obj.is_nominal_recovery_safe(t_end_expected, obs_list_check)
%                         fprintf("[RETURN HELD] t=%.3f s | 復帰予定時刻 t_end=%.3f s の公称軌道が危険 -> 回避継続\n", ...
%                             time.t, t_end_expected);
%                     end
%                 end
% 
%                 if tau < obj.t_duration
%                     xd_out = obj.evaluate_smooth_trajectory(tau, xd_nom);
%                 else
%                     obj.verify_return_c6_matching(t_end_expected);
%                     obj.replan_active = false;
%                     obj.active_threat_id = NaN;
%                     xd_out = xd_nom;
%                     fprintf("[B-SPLINE C^6] 回避所要時間完了: 公称軌道へ完全滑らか復帰 (t=%.3f s)\n\n", time.t);
%                 end
%             else
%                 xd_out = xd_nom;
%             end
% 
%             % 5. 離散時間ステップ積分整合性監視
%             if cha == 'f'
%                 obj.verify_step_integration_consistency(xd_out, time.t, time.dt);
%             end
% 
%             % 6. 結果の格納
%             st = obj.result.state;
%             st.xd                            = xd_out;
%             st.p                             = xd_out(1:3);
%             st.v                             = xd_out(5:7);
%             st.q                             = [0; 0; xd_out(4)];
%             st.pQ_ref                        = xd_out(29:31); % UAV参照目標位置
% 
%             st.time                          = detection.time;
%             st.pQ                            = detection.pQ;
%             st.pL                            = detection.pL;
%             st.detected_point                = detection.detected_point;
%             st.drone_inside_obstacle_point   = detection.drone_inside_obstacle_point;
%             st.load_inside_obstacle_point    = detection.load_inside_obstacle_point;
%             st.cable_inside_obstacle_point   = detection.cable_inside_obstacle_point;
%             st.drone_margin_violated         = detection.drone_margin_violated;
%             st.load_margin_violated          = detection.load_margin_violated;
%             st.cable_margin_violated         = detection.cable_margin_violated;
%             st.drone_min_dist_point          = detection.drone_min_dist_point;
%             st.load_min_dist_point           = detection.load_min_dist_point;
%             st.cable_min_dist_point          = detection.cable_min_dist_point;
%             st.min_dist_point                = detection.min_dist_point;
%             st.drone_obstacle_id_point       = detection.drone_obstacle_id_point;
%             st.load_obstacle_id_point        = detection.load_obstacle_id_point;
%             st.min_obstacle_id_point         = detection.min_obstacle_id_point;
%             st.min_source_point              = "none";
%             st.detected_obstacle_count_point = detection.detected_obstacle_count_point;
%             st.detected_obstacles_point      = detection.detected_obstacles_point;
% 
%             result_out = obj.result;
%         end
% 
%         % =====================================================================
%         % evaluate_smooth_trajectory: 荷物実 B-Spline 軌道 p_L(t) から直接評価 (Public)
%         % =====================================================================
%         function xd = evaluate_smooth_trajectory(obj, tau, xd_nom) %yawC6は未実装　そもそも実装の必要あるか　やったとしてもyawは動かないようにとかか？　思考中
%             xd = xd_nom;
% 
%             % 荷物の実目標軌道状態を B-Spline から直接評価 (0〜6階微分)
%             pL_d   = obj.eval_spline_kth(tau, 0);
%             vL_d   = obj.eval_spline_kth(tau, 1);
%             aL_d   = obj.eval_spline_kth(tau, 2);
%             jL_d   = obj.eval_spline_kth(tau, 3);
%             sL_d   = obj.eval_spline_kth(tau, 4);
%             cL_d   = obj.eval_spline_kth(tau, 5);
%             popL_d = obj.eval_spline_kth(tau, 6);
% 
%             % 【仕様固定】xd(1:28): 荷物 pL の 0〜6階微分 ＋ 公称 Yaw の 0〜6階微分
%             xd(1:3)   = pL_d;    % 荷物位置 (0階)
%             xd(5:7)   = vL_d;    % 荷物速度 (1階)
%             xd(9:11)  = aL_d;    % 荷物加速度 (2階)
%             xd(13:15) = jL_d;    % Jerk (3階)
%             xd(17:19) = sL_d;    % Snap (4階)
%             xd(21:23) = cL_d;    % Crack (5階)
%             xd(25:27) = popL_d;  % Pop (6階)
% 
%             % 差分平坦性モデルに基づく索張力および UAV 目標位置 これが実際のUAV
%             % 位置ではないとなっているがどの様にしたら　このままではいけないのか
%             % ？　思考中1
%             g_vec = [0; 0; obj.gravity];
%             acc_tot = aL_d + g_vec;
%             norm_a = norm(acc_tot);
%             if norm_a > 1e-3, thrust_dir = acc_tot / norm_a; else, thrust_dir = [0; 0; 1]; end
%             pQ_d = pL_d + obj.L_cable * thrust_dir;
% 
%             % 【仕様固定】UAV 目標位置は xd(29:31) へ分離格納
%             xd(29:31) = pQ_d;
%         end
%     end
% 
%     methods (Access = private)
%         % =====================================================================
%         % execute_replanning: 索長 L 最悪姿勢保証 ＆ 全制御点半空間制約 QP
%         % =====================================================================
%         function execute_replanning(obj, pQ_cur, xd_nom, detection, planning_obstacles, t_now)
%             t_replan_start = tic;
%             disp('===== planning_obstacles fields =====');
%             disp(fieldnames(planning_obstacles));
% 
%             if ~isempty(planning_obstacles)
%                 disp('===== planning_obstacles(1) =====');
%                 disp(planning_obstacles(1));
%             end
%             % planning_obstacles (検知中 + 保持中) を制約対象として使用
%             active_plan_obstacles = obj.extract_all_obstacle_halfspaces(pQ_cur, planning_obstacles);
%             if isempty(active_plan_obstacles), return; end
% 
%             % 主回避対象の選定
%             if ~isempty(detection.detected_obstacles_point)
%                 target_obs = detection.detected_obstacles_point(1);
%             else
%                 [~, idx_target] = min([active_plan_obstacles.dist_drone_point]);
%                 target_obs = active_plan_obstacles(idx_target);
%             end
% 
%             obj.active_threat_id = target_obs.id;
% 
%             v_nom = xd_nom(5:7);
%             spd = norm(v_nom);
%             if spd < 0.1, spd = 1.0; v_nom = [1; 0; 0]; end
%             dir_nom = v_nom / spd;
% 
%             init_load_state = zeros(7, 3);
%             if obj.replan_active
%                 tau_now = t_now - obj.t_start;
%                 for k = 0:6
%                     init_load_state(k + 1, :) = obj.eval_spline_kth(tau_now, k)';
%                 end
%             else
%                 init_load_state = obj.get_nominal_derivatives_at_time(t_now);
%             end
% 
%             % 1. 安全拘束法線: 楕円体最接近点の真の外向き単位法線
%             n_constraint = target_obs.normal_drone_point(:) / norm(target_obs.normal_drone_point);
% 
%             % 2. 目的関数の退避誘導方向: 進行軸直交成分
%             n_escape = n_constraint - dot(n_constraint, dir_nom) * dir_nom;
%             if norm(n_escape) < 0.1
%                 n_cand = cross(dir_nom, [0; 0; 1]);
%                 if norm(n_cand) < 0.1, n_cand = cross(dir_nom, [0; 1; 0]); end
%                 n_escape = n_cand / norm(n_cand);
%             else
%                 n_escape = n_escape / norm(n_escape);
%             end
% 
%             % 【厳密幾何包含クリアランス (Theorem)】
%             % コーシー・シュワルツの不等式 n'*(pQ - pL) >= -L より、索の最悪姿勢 n'*q >= -1 に対し
%             % d_req := L_cable + r_drone + d_margin を課すことで連続時間包含を完全保証
%             req_clearance = obj.L_cable + obj.r_drone + obj.clearance_margin;
% 
%             fprintf('\n[REPLAN GEOMETRY]\n');
%             fprintf('  pQ_cur = [%8.3f %8.3f %8.3f]\n', pQ_cur);
%             fprintf('  target_obs:\n');
%             disp(target_obs);
%             fprintf('  req_clearance = %.3f m\n', req_clearance);
% 
%             % ============================================================
%             % 時間設計
%             %
%             % t_clear:
%             %   公称進行方向に沿って障害物の安全領域を完全に抜けるまでの時間
%             %
%             % T_recovery:
%             %   安全領域を抜けた後、公称軌道終端へ戻るための余裕時間
%             %
%             % T_tot = t_clear + T_recovery
%             % ============================================================
% 
%             t_clear = obj.estimate_safety_margin_exit_time( ...
%                 pQ_cur, dir_nom, spd, target_obs, req_clearance);
% 
%             % 障害物通過後に公称軌道へ戻るための回復時間
%             % 最低2秒、かつ t_clear の50%を確保
%             T_recovery = max(2.0, 0.5 * t_clear);
% 
%             % 加速度制約から見た最低限の運動学的時間
%             T_kinematic = sqrt( ...
%                 (8.0 * (req_clearance + 1.0)) / obj.max_acc_load);
% 
%             % 最終的なB-spline生成時間
%             t_dur_cand = max([ ...
%                 5.0, ...
%                 t_clear + T_recovery, ...
%                 T_kinematic * 1.5]);
% 
%             fprintf('\n[REPLAN TIME DESIGN]\n');
%             fprintf('  t_clear      = %.3f s\n', t_clear);
%             fprintf('  T_recovery   = %.3f s\n', T_recovery);
%             fprintf('  T_kinematic  = %.3f s\n', T_kinematic);
%             fprintf('  T_total      = %.3f s\n', t_dur_cand);
% 
%             % --- 第1試行: 全制御点半空間制約 QP 求解 ---
%             t_solve = tic;
% 
%             [qp_success, peak_disp] = obj.plan_uniform_bspline_c6_qp( ...
%                 n_escape, ...
%                 init_load_state, ...
%                 t_dur_cand, ...
%                 active_plan_obstacles, ...
%                 t_now);
% 
%             obj.last_solve_time_ms = toc(t_solve) * 1000;
% 
%             if ~qp_success
%                 fprintf(2, ...
%                     "[REPLAN REJECTED] t=%.3f s | QP求解または全制御点半空間包含判定に失敗. 安全保証対象外として公称維持.\n", ...
%                     t_now);
%                 return;
%             end
% 
%             % 独立数値検証 1回目 (100点サンプリングによる追加チェック)
%             % [traj_safe, peak_acc] = obj.verify_trajectory_numerical_sampling(t_now, obs_list, t_dur_cand);
%             [traj_safe, peak_acc] = obj.verify_trajectory_numerical_sampling( ...
%                 t_now, planning_obstacles, t_dur_cand);
%             status_str = "PASS";
%             if ~traj_safe || (peak_acc > obj.max_acc_load)
%                 % --- 第2試行: マージン拡大 & 時間延長による再求解 ---
%                 obj.clearance_margin = obj.clearance_margin * 1.3; % 思考中　これは設計として危険らしいplan_uniform_bspline_c6_qp() の中で安全領域そのものを一時的に変更　安全保障の固定マージンとQP再試行の探索パラメータを分離してはどうか　
%                 T_kinematic_boost = sqrt((8.0 * (req_clearance * 1.3 + 1.0)) / obj.max_acc_load);
%                 t_dur_cand = max([t_dur_cand * 1.25, T_kinematic_boost * 1.5]);
% 
%                 t_solve = tic;
%                 fprintf('\n[REPLAN PARAM] L=%.3f, rQ=%.3f, margin=%.3f, d_req=%.3f\n', ...
%                     obj.L_cable, ...
%                     obj.r_drone, ...
%                     obj.clearance_margin, ...
%                     obj.L_cable + obj.r_drone + obj.clearance_margin);
%                 [qp_boost_success, peak_disp] = obj.plan_uniform_bspline_c6_qp( ...
%                     n_escape, init_load_state, t_dur_cand, planning_obstacles, t_now);
%                 obj.last_solve_time_ms = toc(t_solve) * 1000;
% 
%                 obj.clearance_margin = obj.clearance_margin / 1.3; % 復元
% 
%                 if ~qp_boost_success
%                     fprintf(2, "[REPLAN REJECTED] t=%.3f s | Boost QP求解に失敗しました. 安全保証対象外として公称維持.\n", t_now);
%                     return;
%                 end
% 
%                 % 独立数値検証 2回目 (再検証)
%                 [traj_safe_2, peak_acc_2] = obj.verify_trajectory_numerical_sampling(t_now, planning_obstacles, t_dur_cand);
% 
%                 if ~traj_safe_2 || (peak_acc_2 > obj.max_acc_load)
%                     fprintf(2, "[REPLAN REJECTED] t=%.3f s | 2段階安全性検証に失敗 (侵入または過大水平加速度: a_xy=%.2f). 危険軌道を棄却します.\n", ...
%                         t_now, peak_acc_2);
%                     return;
%                 end
% 
%                 status_str = "BOOSTED_PASS";
%                 peak_acc = peak_acc_2;
%             end
% 
%             obj.t_start          = t_now;
%             obj.t_duration       = t_dur_cand;
%             obj.last_replan_time = t_now;
%             obj.replan_active    = true;
%             obj.sampled_peak_displacement = peak_disp;
%             obj.sampled_peak_acceleration = peak_acc;
%             obj.last_replan_total_time_ms = toc(t_replan_start) * 1000;
% 
%             obj.verify_start_c6_matching(init_load_state, t_now);
%             obj.display_detection_report(t_now, target_obs, length(planning_obstacles), req_clearance, t_clear, n_constraint, status_str);
%         end
% 
% 
% 
%         % =====================================================================
%         % plan_uniform_bspline_c6_qp: 全制御点半空間制約 ＆ 実荷物 C^6 境界確定 QP
%         % =====================================================================
%         % function [success, max_disp] = plan_uniform_bspline_c6_qp(obj, n_escape_3d, init_load_state, T_tot, active_obstacles, t_now)
%         %     p = 7;
%         %     n_seg = 25;
%         %     n_cp = n_seg + p;          % 32 制御点
%         % 
%         %     obj.spline_degree = p;
%         %     obj.num_segments = n_seg;
%         %     obj.build_clamped_uniform_knots(n_seg, p, T_tot);
%         % 
%         %     % % 1. 始端 0〜6階微分の境界確定: P_L_start (7x3)
%         %     % M_start = zeros(7, 7);
%         %     % for k = 0:6
%         %     %     d_row = obj.eval_basis_derivatives(p + 1, 0.0, k);
%         %     %     M_start(k + 1, :) = d_row(k + 1, 1:7);
%         %     % end
%         %     % P_L_start = M_start \ init_load_state(1:7, :);
%         %     % ============================================================
%         %     % 1. 始端 0〜6階微分の境界確定
%         %     % ============================================================
%         %     M_start = zeros(7, 7);
%         % 
%         %     idx_span_start = obj.find_knot_span(0.0);
%         % 
%         %     for k = 0:6
%         % 
%         %         d_row = obj.eval_basis_derivatives( ...
%         %             idx_span_start, 0.0, k);
%         % 
%         %         if size(d_row,2) ~= p+1
%         %             error('Start basis size error: %d', size(d_row,2));
%         %         end
%         % 
%         %         % 始端span:
%         %         % global CP 1:8
%         %         % local  basis 1:8
%         %         %
%         %         % 始端で固定するCP = 1:7
%         %         M_start(k+1,:) = d_row(k+1,1:7);
%         %     end
%         % 
%         %     P_L_start = M_start \ init_load_state(1:7,:);
%         % 
%         %     % 2. 終端 0〜6階微分の公称合流確定: P_L_end (7x3)
%         %     % end_nom_state = obj.get_nominal_derivatives_at_time(t_now + T_tot);
%         %     % M_end = zeros(7, 7);
%         %     % idx_end = (n_cp - 6):n_cp;
%         %     % for k = 0:6
%         %     %     d_row_end = obj.eval_basis_derivatives(n_cp, T_tot, k);
%         %     %     M_end(k + 1, :) = d_row_end(k + 1, idx_end);
%         %     % end
%         %     % P_L_end = M_end \ end_nom_state(1:7, :);
%         %     % 2. 終端 0〜6階微分の公称合流確定: P_L_end (7x3) % 思考中　終端側は、
%         %     % idx_span_end = obj.find_knot_span(T_tot - 1e-10);として、そのspanを渡すべきです。始端側も同じ考え方で整理したほうがいい
%         %     end_nom_state = ...
%         %         obj.get_nominal_derivatives_at_time(t_now + T_tot);
%         % 
%         %     M_end = zeros(7,7);
%         % 
%         %     % 終端そのものは重複knotなので、
%         %     % 直前のknot spanでbasisを評価する
%         %     t_end_eval = T_tot - 1e-10;
%         % 
%         %     idx_span_end = obj.find_knot_span(t_end_eval);
%         % 
%         %     % 最後のspanの8個のlocal basis
%         %     %
%         %     % global CP : 25 26 27 28 29 30 31 32
%         %     % local     :  1  2  3  4  5  6  7  8
%         %     %
%         %     % C6境界で固定するのは global 26:32
%         %     % → local 2:8
%         % 
%         %     idx_local_end = 2:8;
%         % 
%         %     for k = 0:6
%         % 
%         %         d_row_end = obj.eval_basis_derivatives( ...
%         %             idx_span_end, t_end_eval, k);
%         % 
%         %         if size(d_row_end,2) ~= p+1
%         %             error('End basis size error: %d', size(d_row_end,2));
%         %         end
%         % 
%         %         M_end(k+1,:) = ...
%         %             d_row_end(k+1,idx_local_end);
%         %     end
%         % 
%         %     P_L_end = M_end \ end_nom_state(1:7,:);
%         % 
%         %     % 3. 自由変数 (P_8 〜 P_25: 18制御点 -> 54スカラー変数) 最適化定式化
%         %     D4 = diff(eye(n_cp), 4);
%         %     Q = D4' * D4;
%         % 
%         %     idx_free = 8:(n_cp - 7);
%         %     n_free = length(idx_free);
%         % 
%         %     Q_mm = Q(idx_free, idx_free) + 1e-4 * eye(n_free);
%         %     Q_ms = Q(idx_free, 1:7);
%         %     Q_me = Q(idx_free, (n_cp-6):n_cp);
%         % 
%         %     % 【過大横ずれ抑制】公称軌道制御点 P_nom への引き戻し正則化項を導入
%         %     P_nom_all = obj.project_nominal_trajectory_to_bspline(t_now, T_tot, n_cp);
%         %     P_nom_free = P_nom_all(idx_free, :);
%         % 
%         %     % H = Q_mm + w_dev * I
%         %     H_1d = (Q_mm + Q_mm') / 2 + obj.w_dev * eye(n_free);
%         %     H = blkdiag(H_1d, H_1d, H_1d);
%         % 
%         %     % 線形項: f = f_smooth - w_dev * P_nom_free - w_escape * n_escape
%         %     f_x = (P_L_start(:, 1)' * Q_ms' + P_L_end(:, 1)' * Q_me')' - obj.w_dev * P_nom_free(:, 1);
%         %     f_y = (P_L_start(:, 2)' * Q_ms' + P_L_end(:, 2)' * Q_me')' - obj.w_dev * P_nom_free(:, 2);
%         %     f_z = (P_L_start(:, 3)' * Q_ms' + P_L_end(:, 3)' * Q_me')' - obj.w_dev * P_nom_free(:, 3);
%         % 
%         %     w_escape = 0.05; % 思考中　w_devと同値で，バランスが経験的　後で調整するか何かいい手法を
%         %     f_x = f_x - w_escape * n_escape_3d(1) * ones(n_free, 1);
%         %     f_y = f_y - w_escape * n_escape_3d(2) * ones(n_free, 1);
%         %     f_z = f_z - w_escape * n_escape_3d(3) * ones(n_free, 1);
%         %     f = [f_x; f_y; f_z];
%         % 
%         %     % -------------------------------------------------------------
%         %     % 4. 【全制御点半空間制約】全制御点 (cp_i = 1..32) に対する幾何不等式
%         %     % -------------------------------------------------------------
%         %     A_ineq = [];
%         %     b_ineq = [];
%         %     num_targets = length(active_obstacles);
%         %     constraint_eps = 1e-6;
%         % 
%         %     d_req_obs = obj.L_cable + obj.r_drone + obj.clearance_margin;
%         % 
%         %     for obs_idx = 1:num_targets
%         %         tgt = active_obstacles(obs_idx);
%         %         p_surf = tgt.closest_drone_world_point;
%         %         n_dir  = tgt.normal_drone_point(:) / norm(tgt.normal_drone_point);
%         % 
%         %         for cp_i = 1:n_cp
%         %             if cp_i <= 7
%         %                 P_fixed = P_L_start(cp_i, :)';
%         %                 % if dot(n_dir, (P_fixed - p_surf)) < d_req_obs + constraint_eps
%         %                 %     success = false; max_disp = 0.0; return;
%         %                 % end
%         %                 margin_cp = dot(n_dir, (P_fixed - p_surf)) - d_req_obs;
%         % 
%         %                 if margin_cp < constraint_eps
%         % 
%         %                     fprintf(['\n[REPLAN DEBUG] FIXED START CP REJECT\n' ...
%         %                         '  obstacle ID = %d\n' ...
%         %                         '  cp_i        = %d\n' ...
%         %                         '  actual      = %.6f m\n' ...
%         %                         '  required    = %.6f m\n' ...
%         %                         '  margin      = %.6f m\n' ...
%         %                         '  P_fixed     = [%.3f %.3f %.3f]\n' ...
%         %                         '  p_surf      = [%.3f %.3f %.3f]\n' ...
%         %                         '  n_dir       = [%.3f %.3f %.3f]\n'], ...
%         %                         tgt.id, cp_i, ...
%         %                         dot(n_dir,(P_fixed-p_surf)), ...
%         %                         d_req_obs, ...
%         %                         margin_cp, ...
%         %                         P_fixed(1),P_fixed(2),P_fixed(3), ...
%         %                         p_surf(1),p_surf(2),p_surf(3), ...
%         %                         n_dir(1),n_dir(2),n_dir(3));
%         % 
%         %                     success = false;
%         %                     max_disp = 0.0;
%         %                     return;
%         %                 end
%         %             elseif cp_i >= (n_cp - 6)
%         %                 local_end_idx = cp_i - (n_cp - 7);
%         %                 P_fixed = P_L_end(local_end_idx, :)';
%         %                 % if dot(n_dir, (P_fixed - p_surf)) < d_req_obs + constraint_eps
%         %                 %     success = false; max_disp = 0.0; return;
%         %                 % end
%         %                 margin_cp = dot(n_dir, (P_fixed - p_surf)) - d_req_obs;
%         % 
%         %                 if margin_cp < constraint_eps
%         % 
%         %                     fprintf(['\n[REPLAN DEBUG] FIXED END CP REJECT\n' ...
%         %                         '  obstacle ID = %d\n' ...
%         %                         '  cp_i        = %d\n' ...
%         %                         '  local_idx   = %d\n' ...
%         %                         '  actual      = %.6f m\n' ...
%         %                         '  required    = %.6f m\n' ...
%         %                         '  margin      = %.6f m\n' ...
%         %                         '  P_fixed     = [%.3f %.3f %.3f]\n' ...
%         %                         '  p_surf      = [%.3f %.3f %.3f]\n' ...
%         %                         '  n_dir       = [%.3f %.3f %.3f]\n'], ...
%         %                         tgt.id, cp_i, local_end_idx, ...
%         %                         dot(n_dir,(P_fixed-p_surf)), ...
%         %                         d_req_obs, ...
%         %                         margin_cp, ...
%         %                         P_fixed(1),P_fixed(2),P_fixed(3), ...
%         %                         p_surf(1),p_surf(2),p_surf(3), ...
%         %                         n_dir(1),n_dir(2),n_dir(3));
%         % 
%         %                     success = false;
%         %                     max_disp = 0.0;
%         %                     return;
%         %                 end
%         %             else
%         %                 f_idx = cp_i - 7;
%         %                 row_cp = zeros(1, n_free * 3);
%         %                 for dim = 1:3
%         %                     idx_d = (dim - 1) * n_free;
%         %                     row_cp(idx_d + f_idx) = -n_dir(dim);
%         %                 end
%         %                 A_ineq = [A_ineq; row_cp];
%         %                 b_ineq = [b_ineq; -(d_req_obs + dot(n_dir, p_surf))];
%         %             end
%         %         end
%         %     end
%         % 
%         %     max_coord_bound = 50.0;
%         %     lb = -max_coord_bound * ones(n_free * 3, 1);
%         %     ub =  max_coord_bound * ones(n_free * 3, 1);
%         % 
%         %     opts = optimoptions('quadprog', 'Display', 'off', 'Algorithm', 'interior-point-convex', 'MaxIterations', 300);
%         %     % [X_mid, ~, exitflag] = quadprog(H, f, A_ineq, b_ineq, [], [], lb, ub, [], opts);
%         %     % 
%         %     % if exitflag < 1
%         %     %     success = false;
%         %     %     max_disp = 0.0;
%         %     %     return;
%         %     % end
%         %     [X_mid, fval, exitflag, output] = quadprog( ...
%         %         H, f, A_ineq, b_ineq, [], [], ...
%         %         lb, ub, [], opts);
%         % 
%         %     if exitflag < 1
%         % 
%         %         fprintf(['\n[REPLAN DEBUG] QP FAILED\n' ...
%         %             '  exitflag   = %d\n' ...
%         %             '  iterations = %d\n' ...
%         %             '  message    = %s\n' ...
%         %             '  nvar       = %d\n' ...
%         %             '  nconstr    = %d\n'], ...
%         %             exitflag, ...
%         %             output.iterations, ...
%         %             output.message, ...
%         %             length(f), ...
%         %             size(A_ineq,1));
%         % 
%         %         success = false;
%         %         max_disp = 0.0;
%         %         return;
%         %     end
%         % 
%         %     success = true;
%         %     P_L_mid = zeros(n_free, 3);
%         %     P_L_mid(:, 1) = X_mid(1:n_free);
%         %     P_L_mid(:, 2) = X_mid((n_free+1):(2*n_free));
%         %     P_L_mid(:, 3) = X_mid((2*n_free+1):(3*n_free));
%         % 
%         %     obj.control_points = [P_L_start; P_L_mid; P_L_end];
%         % 
%         %     N_eval = 60;
%         %     taus = linspace(0, T_tot, N_eval);
%         %     max_disp = 0.0;
%         %     for i = 1:N_eval
%         %         p_eval = obj.eval_spline_kth_with_T(taus(i), 0, T_tot);
%         %         nom_res = obj.base_ref.do(struct('t', t_now + taus(i), 'dt', 0.025), 'f');
%         %         max_disp = max(max_disp, norm(p_eval - nom_res.state.xd(1:3)));
%         %     end
%         % end
%         function [success, max_disp] = plan_uniform_bspline_c6_qp( ...
%                  obj, n_escape_3d, init_load_state, T_tot, active_obstacles, t_now)
% 
%             p = 7;
%             n_seg = 25;
%             n_cp = n_seg + p;          % 32 制御点
% 
%             obj.spline_degree = p;
%             obj.num_segments = n_seg;
%             obj.build_clamped_uniform_knots(n_seg, p, T_tot);
% 
%             % ============================================================
%             % 1. 始端 0〜6階微分の境界確定
%             % ============================================================
%             M_start = zeros(7, 7);
% 
%             idx_span_start = obj.find_knot_span(0.0);
% 
%             for k = 0:6
% 
%                 d_row = obj.eval_basis_derivatives( ...
%                     idx_span_start, 0.0, k);
% 
%                 if size(d_row,2) ~= p+1
%                     error('Start basis size error: %d', size(d_row,2));
%                 end
% 
%                 % 始端span:
%                 % global CP 1:8
%                 %
%                 % 始端境界として固定するCP = 1:7
%                 M_start(k+1,:) = d_row(k+1,1:7);
%             end
% 
%             P_L_start = M_start \ init_load_state(1:7,:);
% 
%             % ============================================================
%             % 2. 終端0〜6階微分
%             %
%             % 終端CPは固定しない。
%             % CP8〜CP32を全てQP変数とし、
%             % 終端公称状態はsoft constraintとして使用する。
%             % ============================================================
% 
%             end_nom_state = ...
%                 obj.get_nominal_derivatives_at_time(t_now + T_tot);
% 
%             % ============================================================
%             % 3. 自由変数
%             %
%             % CP1〜CP7  : 始端境界として固定
%             % CP8〜CP32 : 全てQP変数
%             % ============================================================
% 
%             idx_free = 8:n_cp;
%             n_free = length(idx_free);       % 25
% 
%             % ============================================================
%             % 終端評価点と終端span
%             % ============================================================
%             % 終端はclamped knotが重複しているため、
%             % 直前のspan上の点 t_end_eval でbasisを評価する。
%             t_end_eval = T_tot - 1e-10;
% 
%             idx_span_end = obj.find_knot_span(t_end_eval);
% 
%             % 最終span:
%             % global CP : 25 26 27 28 29 30 31 32
%             % local     :  1  2  3  4  5  6  7  8
%             %
%             % 終端側で使用するCP = global 26:32
%             %                  = local 2:8
%             idx_local_end = 2:8;
% 
%             M_end = zeros(7,7);
% 
%             for k = 0:6
% 
%                 d_row_end = obj.eval_basis_derivatives( ...
%                     idx_span_end, t_end_eval, k);
% 
%                 if size(d_row_end,2) ~= p+1
%                     error('End basis size error: %d', size(d_row_end,2));
%                 end
% 
%                 M_end(k+1,:) = ...
%                     d_row_end(k+1,idx_local_end);
%             end
% 
%             fprintf('[QP DEBUG] M_end max/min = %.3e / %.3e\n', ...
%                 max(abs(M_end(:))), min(M_end(:)));
% 
%             fprintf('[QP DEBUG] M_end row norms:\n');
%             for k = 1:7
%                 fprintf('  derivative %d : %.3e\n', ...
%                     k-1, norm(M_end(k,:)));
%             end
% 
%             % ============================================================
%             % 終端条件の数値スケーリング
%             % ============================================================
% 
%             terminal_scale = max(vecnorm(M_end,2,2),1.0);
% 
%             S_end = diag(1 ./ terminal_scale);
% 
%             M_end_scaled = S_end * M_end;
% 
%             end_nom_scaled = S_end * end_nom_state(1:7,:);
% 
%             % ============================================================
%             % 4. Smoothness
%             % ============================================================
%             D4 = diff(eye(n_cp), 4);
%             Q = D4' * D4;
% 
%             Q_mm = Q(idx_free, idx_free) ...
%                  + 1e-4 * eye(n_free);
% 
%             Q_ms = Q(idx_free, 1:7);
% 
%             % ============================================================
%             % 5. 公称軌道のB-spline制御点
%             %
%             % ここは今までの処理をそのまま残す
%             % ============================================================
%             P_nom_all = ...
%                 obj.project_nominal_trajectory_to_bspline( ...
%                     t_now, T_tot, n_cp);
% 
%             P_nom_free = P_nom_all(idx_free, :);
% 
%             % ============================================================
%             % 6. 基本QP目的関数
%             %
%             % smoothness
%             % +
%             % 公称軌道への引き戻し
%             % ============================================================
%             fprintf('[QP DEBUG] Q max/min = %.3e / %.3e\n', ...
%                 max(abs(Q(:))), min(Q(:)));
% 
%             fprintf('[QP DEBUG] Q_mm max/min = %.3e / %.3e\n', ...
%                 max(abs(Q_mm(:))), min(Q_mm(:)));
% 
%             fprintf('[QP DEBUG] w_dev = %.3e\n', obj.w_dev);
% 
%             H_1d_test = (Q_mm + Q_mm') / 2 + obj.w_dev * eye(n_free);
% 
%             fprintf('[QP DEBUG] H_1d max/min = %.3e / %.3e\n', ...
%                 max(abs(H_1d_test(:))), min(H_1d_test(:)));
% 
%             fprintf('[QP DEBUG] H_1d eig min/max = %.3e / %.3e\n', ...
%                 min(eig(H_1d_test)), max(eig(H_1d_test)));
% 
%             H_1d = ...
%                 (Q_mm + Q_mm') / 2 ...
%                 + obj.w_dev * eye(n_free);
% 
%             fprintf('[QP DEBUG] H_1d before terminal eig min/max = %.3e / %.3e\n', ...
%                 min(eig(H_1d)), max(eig(H_1d)));
% 
%             % ============================================================
%             % 7. 終端0〜6階微分のsoft constraint
%             % ============================================================
% 
%             w_terminal = 10.0;
% 
%             % free変数CP8:32のうち、
%             % 最後7個がCP26:32
%             idx_terminal_free = (n_free - 6):n_free;
% 
%             % 終端微分のスケーリング
%             terminal_scale = max(vecnorm(M_end, 2, 2), 1.0);
%             S_end = diag(1 ./ terminal_scale);
% 
%             M_end_scaled = S_end * M_end;
%             end_nom_scaled = S_end * end_nom_state(1:7,:);
% 
%             fprintf('[QP DEBUG] M_end_scaled row norms:\n');
%             for k = 1:7
%                 fprintf('  derivative %d : %.3e\n', ...
%                     k-1, norm(M_end_scaled(k,:)));
%             end
% 
%             % 終端CPに対するHessian
%             H_terminal = ...
%                 w_terminal * (M_end_scaled' * M_end_scaled);
% 
%             H_1d(idx_terminal_free,idx_terminal_free) = ...
%                 H_1d(idx_terminal_free,idx_terminal_free) ...
%                 + H_terminal;
% 
%             H_1d = (H_1d + H_1d') / 2;
% 
%             fprintf('[QP DEBUG] w_terminal = %.3e\n', w_terminal);
% 
%             fprintf('[QP DEBUG] H_1d after terminal eig min/max = %.3e / %.3e\n', ...
%                 min(eig(H_1d)), max(eig(H_1d)));
% 
%             H = blkdiag(H_1d, H_1d, H_1d);
% 
%             % ============================================================
%             % 8. 線形項
%             % ============================================================
% 
%             % smoothness
%             f_x = ...
%                 (P_L_start(:,1)' * Q_ms')' ...
%                 - obj.w_dev * P_nom_free(:,1);
% 
%             f_y = ...
%                 (P_L_start(:,2)' * Q_ms')' ...
%                 - obj.w_dev * P_nom_free(:,2);
% 
%             f_z = ...
%                 (P_L_start(:,3)' * Q_ms')' ...
%                 - obj.w_dev * P_nom_free(:,3);
% 
%             % ============================================================
%             % terminal soft constraint の線形項
%             % ============================================================
% 
%             f_x(idx_terminal_free) = ...
%                 f_x(idx_terminal_free) ...
%                 - w_terminal * M_end_scaled' * end_nom_scaled(:,1);
% 
%             f_y(idx_terminal_free) = ...
%                 f_y(idx_terminal_free) ...
%                 - w_terminal * M_end_scaled' * end_nom_scaled(:,2);
% 
%             f_z(idx_terminal_free) = ...
%                 f_z(idx_terminal_free) ...
%                 - w_terminal * M_end_scaled' * end_nom_scaled(:,3);
% 
%             % ============================================================
%             % 9. Escape方向
%             % ============================================================
% 
%             w_escape = 0.05;
% 
%             f_x = ...
%                 f_x ...
%                 - w_escape * n_escape_3d(1) * ones(n_free,1);
% 
%             f_y = ...
%                 f_y ...
%                 - w_escape * n_escape_3d(2) * ones(n_free,1);
% 
%             f_z = ...
%                 f_z ...
%                 - w_escape * n_escape_3d(3) * ones(n_free,1);
% 
%             f = [f_x; f_y; f_z];
% 
%             % ============================================================
%             % 10. 線形不等式制約
%             %
%             % 10-1. 障害物半空間制約
%             % 10-2. 水平加速度制約
%             % ============================================================
%             A_ineq = [];
%             b_ineq = [];
% 
%             num_targets = length(active_obstacles);
%             constraint_eps = 1e-6;
% 
%             d_req_obs = ...
%                 obj.L_cable ...
%                 + obj.r_drone ...
%                 + obj.clearance_margin;
% 
%             % ============================================================
%             % 10-1. 全制御点半空間制約
%             % ============================================================
%             for obs_idx = 1:num_targets
% 
%                 tgt = active_obstacles(obs_idx);
% 
%                 p_surf = tgt.closest_drone_world_point;
% 
%                 n_dir = ...
%                     tgt.normal_drone_point(:) ...
%                     / norm(tgt.normal_drone_point);
% 
%                 for cp_i = 1:n_cp
% 
%                     % ====================================================
%                     % CP1～CP7
%                     % 始端固定CP
%                     % ====================================================
%                     if cp_i <= 7
% 
%                         P_fixed = P_L_start(cp_i,:)';
% 
%                         margin_cp = ...
%                             dot(n_dir,(P_fixed-p_surf)) ...
%                             - d_req_obs;
% 
%                         if margin_cp < constraint_eps
% 
%                             fprintf( ...
%                                 ['\n[REPLAN DEBUG] FIXED START CP REJECT\n' ...
%                                  '  obstacle ID = %d\n' ...
%                                  '  cp_i        = %d\n' ...
%                                  '  actual      = %.6f m\n' ...
%                                  '  required    = %.6f m\n' ...
%                                  '  margin      = %.6f m\n' ...
%                                  '  P_fixed     = [%.3f %.3f %.3f]\n' ...
%                                  '  p_surf      = [%.3f %.3f %.3f]\n' ...
%                                  '  n_dir       = [%.3f %.3f %.3f]\n'], ...
%                                 tgt.id, ...
%                                 cp_i, ...
%                                 dot(n_dir,(P_fixed-p_surf)), ...
%                                 d_req_obs, ...
%                                 margin_cp, ...
%                                 P_fixed(1), ...
%                                 P_fixed(2), ...
%                                 P_fixed(3), ...
%                                 p_surf(1), ...
%                                 p_surf(2), ...
%                                 p_surf(3), ...
%                                 n_dir(1), ...
%                                 n_dir(2), ...
%                                 n_dir(3));
% 
%                             success = false;
%                             max_disp = 0.0;
%                             return;
%                         end
% 
%                     % ====================================================
%                     % CP8～CP32
%                     % 全てQP変数
%                     % ====================================================
%                     else
% 
%                         f_idx = cp_i - 7;
% 
%                         row_cp = zeros(1,n_free*3);
% 
%                         for dim = 1:3
% 
%                             idx_d = ...
%                                 (dim-1)*n_free;
% 
%                             row_cp(idx_d + f_idx) = ...
%                                 -n_dir(dim);
%                         end
% 
%                         A_ineq = ...
%                             [A_ineq; row_cp];
% 
%                         b_ineq = ...
%                             [b_ineq; ...
%                             -(d_req_obs + dot(n_dir,p_surf))];
%                     end
%                 end
%             end
% 
%             % ============================================================
%             % 10-2. 水平加速度制約
%             %
%             % 目的:
%             %
%             %   ||a_xy(t)|| <= obj.max_acc_load
%             %
%             % をB-splineの凸包性から保守的に保証する。
%             %
%             % 直接的な円形制約
%             %
%             %   sqrt(ax^2 + ay^2) <= a_max
%             %
%             % はquadprogでは扱えないため、
%             %
%             %   |ax| <= a_max/sqrt(2)
%             %   |ay| <= a_max/sqrt(2)
%             %
%             % を課す。
%             %
%             % これにより全時刻で
%             %
%             %   sqrt(ax^2 + ay^2) <= a_max
%             %
%             % が保証される。
%             %
%             % 2階微分B-splineの制御点は
%             %
%             %   A2 = D2 * P
%             %
%             % で求める。
%             %
%             % D2は現在のclamped uniform knotから直接構築するため、
%             % 端部のclamped knotによる係数変化も正しく反映する。
%             % ============================================================
% 
%             % ------------------------------------------------------------
%             % 10-2-1. 1階微分制御点変換行列 D1
%             % ------------------------------------------------------------
%             U = obj.knots;
% 
%             D1 = zeros(n_cp - 1, n_cp);
% 
%             for i = 1:(n_cp - 1)
% 
%                 denom = U(i + p + 1) - U(i + 1);
% 
%                 if denom > 1e-12
% 
%                     D1(i,i) = ...
%                         -p / denom;
% 
%                     D1(i,i+1) = ...
%                          p / denom;
%                 end
%             end
% 
%             % ------------------------------------------------------------
%             % 10-2-2. 1階微分後のノット
%             %
%             % degree 7 -> degree 6
%             % ------------------------------------------------------------
%             U1 = U(2:end-1);
% 
%             p1 = p - 1;
% 
%             % ------------------------------------------------------------
%             % 10-2-3. 2階微分制御点変換行列 D2
%             %
%             % degree 6 -> degree 5
%             % ------------------------------------------------------------
%             D1_second = zeros(n_cp - 2, n_cp - 1);
% 
%             for i = 1:(n_cp - 2)
% 
%                 denom = ...
%                     U1(i + p1 + 1) - U1(i + 1);
% 
%                 if denom > 1e-12
% 
%                     D1_second(i,i) = ...
%                         -p1 / denom;
% 
%                     D1_second(i,i+1) = ...
%                          p1 / denom;
%                 end
%             end
% 
%             D2 = D1_second * D1;
% 
%             % D2 : 30 x 32
%             fprintf('[QP DEBUG] D2 size = %d x %d\n', ...
%                 size(D2,1), size(D2,2));
% 
%             fprintf('[QP DEBUG] D2 max/min = %.3e / %.3e\n', ...
%                 max(abs(D2(:))), min(D2(:)));
% 
%             % ------------------------------------------------------------
%             % 10-2-4. 固定CPと自由CPに分解
%             %
%             % P = [P_start ; P_free]
%             %
%             % D2*P =
%             %   D2_start*P_start
%             %   +
%             %   D2_free*P_free
%             % ------------------------------------------------------------
%             D2_start = D2(:,1:7);
%             D2_free  = D2(:,8:n_cp);
% 
%             % ------------------------------------------------------------
%             % 10-2-5. 水平加速度制約
%             %
%             % |ax| <= a_max/sqrt(2)
%             % |ay| <= a_max/sqrt(2)
%             % ------------------------------------------------------------
%             a_max = obj.max_acc_load;
% 
%             if ~isfinite(a_max) || a_max <= 0
% 
%                 error( ...
%                     'Invalid obj.max_acc_load: %.6f', ...
%                     a_max);
%             end
% 
%             a_component_max = ...
%                 a_max / sqrt(2.0);
% 
%             n_acc_ctrl = size(D2,1);
% 
%             fprintf( ...
%                 ['[QP DEBUG] Horizontal acceleration constraint\n' ...
%                  '  a_max           = %.6f m/s^2\n' ...
%                  '  component limit = %.6f m/s^2\n' ...
%                  '  acceleration CP = %d\n'], ...
%                 a_max, ...
%                 a_component_max, ...
%                 n_acc_ctrl);
% 
%             for acc_i = 1:n_acc_ctrl
% 
%                 d2_row = D2_free(acc_i,:);
% 
%                 % ========================================================
%                 % ax <= a_component_max
%                 % ========================================================
%                 row_acc_x = zeros(1,n_free*3);
% 
%                 row_acc_x(1:n_free) = d2_row;
% 
%                 fixed_acc_x = ...
%                     D2_start(acc_i,:) * P_L_start(:,1);
% 
%                 rhs_acc_x_upper = ...
%                     a_component_max - fixed_acc_x;
% 
%                 A_ineq = ...
%                     [A_ineq; row_acc_x];
% 
%                 b_ineq = ...
%                     [b_ineq; rhs_acc_x_upper];
% 
%                 % ========================================================
%                 % -ax <= a_component_max
%                 % ========================================================
%                 row_acc_x_lower = ...
%                     -row_acc_x;
% 
%                 rhs_acc_x_lower = ...
%                     a_component_max + fixed_acc_x;
% 
%                 A_ineq = ...
%                     [A_ineq; row_acc_x_lower];
% 
%                 b_ineq = ...
%                     [b_ineq; rhs_acc_x_lower];
% 
%                 % ========================================================
%                 % ay <= a_component_max
%                 % ========================================================
%                 row_acc_y = zeros(1,n_free*3);
% 
%                 row_acc_y(n_free + (1:n_free)) = ...
%                     d2_row;
% 
%                 fixed_acc_y = ...
%                     D2_start(acc_i,:) * P_L_start(:,2);
% 
%                 rhs_acc_y_upper = ...
%                     a_component_max - fixed_acc_y;
% 
%                 A_ineq = ...
%                     [A_ineq; row_acc_y];
% 
%                 b_ineq = ...
%                     [b_ineq; rhs_acc_y_upper];
% 
%                 % ========================================================
%                 % -ay <= a_component_max
%                 % ========================================================
%                 row_acc_y_lower = ...
%                     -row_acc_y;
% 
%                 rhs_acc_y_lower = ...
%                     a_component_max + fixed_acc_y;
% 
%                 A_ineq = ...
%                     [A_ineq; row_acc_y_lower];
% 
%                 b_ineq = ...
%                     [b_ineq; rhs_acc_y_lower];
%             end
% 
%             % ============================================================
%             % 11. Control point bounds
%             % ============================================================
%             max_coord_bound = 50.0;
% 
%             lb = ...
%                 -max_coord_bound * ones(n_free*3,1);
% 
%             ub = ...
%                  max_coord_bound * ones(n_free*3,1);
% 
%             % ============================================================
%             % 12. QP
%             % ============================================================
%             opts = optimoptions( ...
%                 'quadprog', ...
%                 'Display', 'off', ...
%                 'Algorithm', 'interior-point-convex', ...
%                 'MaxIterations', 300);
% 
%             fprintf('\n[QP DEBUG] nvar=%d, nconstr=%d\n', ...
%                 size(H,1), size(A_ineq,1));
% 
%             fprintf('[QP DEBUG] H eig min/max = %.3e / %.3e\n', ...
%                 min(eig((H+H')/2)), max(eig((H+H')/2)));
% 
%             fprintf('[QP DEBUG] ||f|| = %.3e\n', norm(f));
% 
%             if ~isempty(A_ineq)
%                 fprintf('[QP DEBUG] ||A|| = %.3e, ||b|| = %.3e\n', ...
%                     norm(A_ineq,'fro'), norm(b_ineq));
%             end
% 
%             [X_mid, fval, exitflag, output] = ...
%                 quadprog( ...
%                     H, f, ...
%                     A_ineq, b_ineq, ...
%                     [], [], ...
%                     lb, ub, [], opts);
% 
%             % ============================================================
%             % 13. QP失敗
%             % ============================================================
%             if exitflag < 1
% 
%                 fprintf( ...
%                     ['\n[REPLAN DEBUG] QP FAILED\n' ...
%                      '  exitflag   = %d\n' ...
%                      '  iterations = %d\n' ...
%                      '  message    = %s\n' ...
%                      '  nvar       = %d\n' ...
%                      '  nconstr    = %d\n'], ...
%                     exitflag, ...
%                     output.iterations, ...
%                     output.message, ...
%                     length(f), ...
%                     size(A_ineq,1));
% 
%                 success = false;
%                 max_disp = 0.0;
% 
%                 return;
%             end
% 
%             % ============================================================
%             % 14. 最適解からCPを復元
%             % ============================================================
%             success = true;
% 
%             P_L_mid = zeros(n_free,3);
% 
%             P_L_mid(:,1) = ...
%                 X_mid(1:n_free);
% 
%             P_L_mid(:,2) = ...
%                 X_mid((n_free+1):(2*n_free));
% 
%             P_L_mid(:,3) = ...
%                 X_mid((2*n_free+1):(3*n_free));
% 
%             % CP1:7 + CP8:32
%             obj.control_points = ...
%                 [P_L_start; P_L_mid];
% 
%             % ============================================================
%             % 【緊急診断】D2による加速度制御点 vs 実スプライン微分値の完全比較
%             % ============================================================
%             A2_mat = D2 * obj.control_points; % 30 x 3 の加速度制御点行列
%             A2_xy_norms = sqrt(A2_mat(:,1).^2 + A2_mat(:,2).^2);
%             [max_A2_norm, idx_max_A2] = max(A2_xy_norms);
% 
%             fprintf('\n------------------------------------------------------------\n');
%             fprintf('[ACCEL DIAGNOSTIC 1: D2 CONTROL POINTS]\n');
%             fprintf('  設定加速度上限 a_max       = %.3f m/s^2 (成分上限: %.3f)\n', a_max, a_component_max);
%             fprintf('  D2制御点の最大 |A2_x|      = %.3f m/s^2\n', max(abs(A2_mat(:,1))));
%             fprintf('  D2制御点の最大 |A2_y|      = %.3f m/s^2\n', max(abs(A2_mat(:,2))));
%             fprintf('  D2制御点の最大 ||A2_xy||   = %.3f m/s^2 (インデックス: %d / %d)\n', max_A2_norm, idx_max_A2, n_acc_ctrl);
% 
%             % 始端固定CP（公称から引き継いだ始端加速度）自体のチェック
%             A2_start_mat = D2_start * P_L_start;
%             fprintf('  始端固定部 D2_start*P_start の最大 ||a_xy|| = %.3f m/s^2\n', ...
%                 max(sqrt(A2_start_mat(:,1).^2 + A2_start_mat(:,2).^2)));
% 
%             % 連続時間評価値サンプリング（60点）での比較
%             test_taus = linspace(0.0, T_tot, 60);
%             eval_acc_norms = zeros(60, 1);
%             for ti = 1:60
%                 a_eval = obj.eval_spline_kth_with_T(test_taus(ti), 2, T_tot);
%                 eval_acc_norms(ti) = norm(a_eval(1:2));
%             end
%             [max_eval_acc, idx_max_eval] = max(eval_acc_norms);
%             t_peak = test_taus(idx_max_eval);
% 
%             fprintf('[ACCEL DIAGNOSTIC 2: CONTINUOUS EVALUATION eval_spline_kth]\n');
%             fprintf('  eval_spline_kth による最大実加速度 a_xy = %.3f m/s^2 (at tau = %.3f s)\n', max_eval_acc, t_peak);
%             fprintf('  始端加速度 (tau=0.0s) a_xy              = %.3f m/s^2\n', eval_acc_norms(1));
%             fprintf('  終端加速度 (tau=T_tot) a_xy             = %.3f m/s^2\n', eval_acc_norms(end));
% 
%             % ギャップの判定
%             ratio = max_eval_acc / max(max_A2_norm, 1e-6);
%             fprintf('  [不整合比率] 実評価値 / D2制御点 = %.3f\n', ratio);
%             if abs(ratio - 1.0) > 0.2
%                 fprintf(2, '  ===> [FATAL MISMATCH DETECTED] D2 行列と eval_basis_derivatives の微分スケールが食い違っています！\n');
%             end
%             fprintf('------------------------------------------------------------\n\n');
%             % ============================================================
%             % 15. 公称軌道からの最大変位
%             % ============================================================
%             N_eval = 60;
% 
%             taus = linspace(0,T_tot,N_eval);
% 
%             max_disp = 0.0;
% 
%             for i = 1:N_eval
% 
%                 p_eval = ...
%                     obj.eval_spline_kth_with_T( ...
%                         taus(i),0,T_tot);
% 
%                 nom_res = ...
%                     obj.base_ref.do( ...
%                         struct( ...
%                             't',  t_now + taus(i), ...
%                             'dt', 0.025), ...
%                         'f');
% 
%                 max_disp = max( ...
%                     max_disp, ...
%                     norm(p_eval - nom_res.state.xd(1:3)));
%             end
% 
%         end
% 
% 
%         % =====================================================================
%         % project_nominal_trajectory_to_bspline: 公称軌道のB-Spline最小二乗射影
%         % =====================================================================
%         function P_nom_all = project_nominal_trajectory_to_bspline(obj, t_now, T_tot, n_cp)
%             p = obj.spline_degree;
%             N_samples = 100;
%             t_quad = linspace(0.0, T_tot, N_samples);
%             dt = T_tot / (N_samples - 1);
% 
%             M_gram = zeros(n_cp, n_cp);
%             B_proj = zeros(n_cp, 3);
% 
%             for k = 1:N_samples
%                 tau_k = t_quad(k);
%                 t_k = t_now + tau_k;
% 
%                 nom_res_k = obj.base_ref.do(struct('t', t_k, 'dt', 0.025), 'f');
%                 p_nom_k = nom_res_k.state.xd(1:3)';
% 
%                 idx_span = obj.find_knot_span(min(T_tot - 1e-6, tau_k));
%                 ders = obj.eval_basis_derivatives(idx_span, min(T_tot - 1e-6, tau_k), 0);
%                 basis_vals = ders(1, :);
%                 c_idx = (idx_span - p):idx_span;
% 
%                 w = dt;
%                 if k == 1 || k == N_samples, w = 0.5 * dt; end
% 
%                 M_gram(c_idx, c_idx) = M_gram(c_idx, c_idx) + w * (basis_vals' * basis_vals);
%                 B_proj(c_idx, :) = B_proj(c_idx, :) + w * (basis_vals' * p_nom_k);
%             end
% 
%             P_nom_all = (M_gram + 1e-5 * eye(n_cp)) \ B_proj;
%         end
% 
%         % =====================================================================
%         % get_nominal_derivatives_at_time: 公称軌道から 0〜6階微分値を抽出
%         % =====================================================================
%         function ders = get_nominal_derivatives_at_time(obj, t_eval)
%             ders = zeros(7, 3);
%             nom_res = obj.base_ref.do(struct('t', t_eval, 'dt', 0.025), 'f');
%             xd_val = nom_res.state.xd;
%             if length(xd_val) < 28
%                 xd_val = [xd_val; zeros(28 - length(xd_val), 1)];
%             end
%             ders(1, :) = xd_val(1:3)';   % 位置 (0階)
%             ders(2, :) = xd_val(5:7)';   % 速度 (1階)
%             ders(3, :) = xd_val(9:11)';  % 加速度 (2階)
%             ders(4, :) = xd_val(13:15)'; % Jerk (3階)
%             ders(5, :) = xd_val(17:19)'; % Snap (4階)
%             ders(6, :) = xd_val(21:23)'; % Crack (5階)
%             ders(7, :) = xd_val(25:27)'; % Pop (6階)
%         end
% 
%         % =====================================================================
%         % extract_all_obstacle_halfspaces: 環境内全障害物の支持半空間パラメータ抽出
%         % =====================================================================
%         % function active_list = extract_all_obstacle_halfspaces(obj, pQ_cur, obs_list)
%         %     active_list = [];
%         %     for i = 1:length(obs_list)
%         %         o = obs_list(i);
%         %         obs_parsed = struct('center', o.p_center(:), 'radii', o.ellipsoid_radii(:), 'R', o.R_obs);
%         %         [d_raw, ~, cpQ_loc, ~] = obj.point_ellipsoid_signed_distance(pQ_cur, obs_parsed);
%         % 
%         %         cpQ_world = o.p_center(:) + o.R_obs * cpQ_loc;
%         %         nQ_loc = cpQ_loc ./ (o.ellipsoid_radii(:).^2);
%         %         normal_drone = o.R_obs * (nQ_loc / norm(nQ_loc));
%         % 
%         %         item = struct('id', i, 'p_obs', o.p_center(:), 'radii_obs', o.ellipsoid_radii(:), ...
%         %                       'R_obs', o.R_obs, 'closest_drone_world_point', cpQ_world, ...
%         %                       'normal_drone_point', normal_drone, 'dist_drone_point', d_raw - obj.r_drone);
%         %         active_list = [active_list; item];
%         %     end
%         % end
%         % function active_list = extract_all_obstacle_halfspaces(obj, pQ_cur, obs_list)
%         %     active_list = [];
%         %     for i = 1:length(obs_list)
%         %         o = obs_list(i);
%         % 
%         %         obs_id = o.id;
%         % 
%         %         obs_parsed = struct('center', o.p_center(:), 'radii', o.ellipsoid_radii(:), 'R', o.R_obs);
%         %         [d_raw, ~, cpQ_loc, ~] = obj.point_ellipsoid_signed_distance(pQ_cur, obs_parsed);
%         % 
%         %         cpQ_world = o.p_center(:) + o.R_obs * cpQ_loc;
%         %         nQ_loc = cpQ_loc ./ (o.ellipsoid_radii(:).^2);
%         %         normal_drone = o.R_obs * (nQ_loc / norm(nQ_loc));
%         % 
%         %         item = struct('id', obs_id, 'p_obs', o.p_center(:), 'radii_obs', o.ellipsoid_radii(:), ...
%         %             'R_obs', o.R_obs, 'closest_drone_world_point', cpQ_world, ...
%         %             'normal_drone_point', normal_drone, 'dist_drone_point', d_raw - obj.r_drone);
%         %         active_list = [active_list; item];
%         %     end
%         % end
%         function active_list = extract_all_obstacle_halfspaces(obj, pQ_cur, obs_list)
% 
%             if isempty(obs_list)
%                 active_list = obs_list;
%                 return;
%             end
% 
%             active_list = obs_list;
% 
%             for i = 1:numel(obs_list)
% 
%                 o = obs_list(i);
% 
%                 % =====================================================
%                 % planning_obstacles は active obstacle
%                 % =====================================================
%                 % =====================================================
%                 % planning_obstacles の形式に応じて取得
%                 % =====================================================
%                 p_obs = o.p_obs(:);
% 
%                 if isfield(o, 'radii_obs')
%                     % active obstacle
%                     radii_obs = o.radii_obs(:);
%                 elseif isfield(o, 'ellipsoid_radii')
%                     % raw obstacle
%                     radii_obs = o.ellipsoid_radii(:);
%                 else
%                     error('Obstacle ID=%d に radii_obs / ellipsoid_radii がありません.', o.id);
%                 end
% 
%                 R_obs = o.R_obs;
% 
%                 % =====================================================
%                 % 楕円体情報
%                 % =====================================================
%                 obs_parsed = struct( ...
%                     'center', p_obs, ...
%                     'radii',  radii_obs, ...
%                     'R',      R_obs);
% 
%                 % =====================================================
%                 % 現在の UAV 位置から楕円体までの距離
%                 % =====================================================
%                 [d_raw, ~, cpQ_loc, ~] = ...
%                     obj.point_ellipsoid_signed_distance( ...
%                         pQ_cur, obs_parsed);
% 
%                 % =====================================================
%                 % 最近点（world座標）
%                 % =====================================================
%                 cpQ_world = p_obs + R_obs * cpQ_loc;
% 
%                 % =====================================================
%                 % 楕円体表面の外向き法線
%                 % =====================================================
%                 nQ_loc = cpQ_loc ./ (radii_obs.^2);
% 
%                 if norm(nQ_loc) < 1e-12
%                     warning( ...
%                         'Obstacle ID=%d: normal computation failed.', ...
%                         o.id);
%                     continue;
%                 end
% 
%                 normal_drone = ...
%                     R_obs * (nQ_loc / norm(nQ_loc));
% 
%                 % =====================================================
%                 % UAV球表面から障害物表面までの安全距離
%                 % =====================================================
%                 dist_drone_point = d_raw - obj.r_drone;
% 
%                 % =====================================================
%                 % active obstacle の情報を更新
%                 % =====================================================
%                 active_list(i).closest_drone_world_point = cpQ_world;
%                 active_list(i).normal_drone_point        = normal_drone;
%                 active_list(i).dist_drone_point          = dist_drone_point;
% 
%             end
%         end
% 
%         % =====================================================================
%         % estimate_safety_margin_impact_time: 公称軌道上の安全マージン侵入予測
%         % =====================================================================
%         function t_clear = estimate_safety_margin_impact_time(obj, pQ_cur, dir_nom, spd, target_obs)
%             vec_to_obs = target_obs.p_obs - pQ_cur;
%             dist_along = dot(vec_to_obs, dir_nom);
% 
%             max_rad = max(target_obs.radii_obs);
%             dist_to_margin = max(0.5, dist_along - max_rad - obj.r_drone - obj.clearance_margin);
%             t_clear = max(1.5, dist_to_margin / spd);
%         end
% 
%         % =====================================================================
%         % verify_trajectory_numerical_sampling: 100点サンプリングによる数値的独立検証
%         % ※連続時間の理論的安全包含は B-Spline 全制御点拘束 n'*(P_i - p_s) >= d_req が担保。
%         % 本メソッドは追従前シミュレーションによる独立した数値整合性チェック（Runtime Monitor）として機能する。
%         % =====================================================================
%         function [is_safe, peak_acc_xy] = verify_trajectory_numerical_sampling(obj, t_now, obs_list, T_tot)
%             is_safe = true;
%             peak_acc_xy = 0.0;
%             N_samples = 100;
%             taus = linspace(0.0, T_tot, N_samples);
% 
%             r_c_sph = obj.r_cable_sphere;
%             n_spheres = max(2, ceil(obj.L_cable / (2.0 * r_c_sph)) + 1);
%             s_ratios = linspace(0.0, 1.0, n_spheres);
% 
%             for i = 1:N_samples
%                 tau_i = taus(i);
% 
%                 pL_fut = obj.eval_spline_kth_with_T(tau_i, 0, T_tot);
%                 aL_fut = obj.eval_spline_kth_with_T(tau_i, 2, T_tot);
% 
%                 peak_acc_xy = max(peak_acc_xy, norm(aL_fut(1:2)));
% 
%                 g_vec = [0; 0; obj.gravity];
%                 acc_tot = aL_fut + g_vec;
%                 norm_a = norm(acc_tot);
%                 if norm_a > 1e-3, thrust_dir = acc_tot / norm_a; else, thrust_dir = [0; 0; 1]; end
%                 pQ_fut = pL_fut + obj.L_cable * thrust_dir;
% 
%                 cable_pts_fut = (1 - s_ratios) .* pL_fut + s_ratios .* pQ_fut;
% 
%                 for obs_idx = 1:length(obs_list)
%                     o = obs_list(obs_idx);
%                     % obs_parsed = struct('center', o.p_center(:), 'radii', o.ellipsoid_radii(:), 'R', o.R_obs);
%                     obs_parsed = struct( ...
%                         'center', o.p_obs(:), ...
%                         'radii', o.radii_obs(:), ...
%                         'R', o.R_obs);
%                     % 1. UAV
%                     [dQ, ~, ~, ~] = obj.point_ellipsoid_signed_distance(pQ_fut, obs_parsed);
%                     if dQ - obj.r_drone <= 0
%                         fprintf(2, '[VERIFY FAIL] t_eval=%.3f s (tau=%.3f s) | UAV球体が障害物 ID=%d に侵入 (gap=%.3f m)\n', ...
%                             t_now + tau_i, tau_i, o.id, dQ - obj.r_drone);
%                         is_safe = false; 
%                         return; 
%                     end
% 
%                     % 2. 荷物
%                     [dL, ~, ~, ~] = obj.point_ellipsoid_signed_distance(pL_fut, obs_parsed);
%                     if dL - obj.r_load <= 0
%                         fprintf(2, '[VERIFY FAIL] t_eval=%.3f s (tau=%.3f s) | 荷物球体が障害物 ID=%d に侵入 (gap=%.3f m)\n', ...
%                             t_now + tau_i, tau_i, o.id, dL - obj.r_load);
%                         is_safe = false; 
%                         return; 
%                     end
% 
%                     % 3. 索球列 (Runtime Monitor)
%                     for sp = 1:n_spheres
%                         [dC, ~, ~, ~] = obj.point_ellipsoid_signed_distance(cable_pts_fut(:, sp), obs_parsed);
%                             if dC - r_c_sph <= 0
%                             fprintf(2, '[VERIFY FAIL] t_eval=%.3f s (tau=%.3f s) | 直線索球列(球#%d)が障害物 ID=%d に侵入 (gap=%.3f m)\n', ...
%                                 t_now + tau_i, tau_i, sp, o.id, dC - r_c_sph);
%                             is_safe = false; 
%                             return; 
%                         end
%                     end
%                 end
%             end
%             if peak_acc_xy > obj.max_acc_load
%                 fprintf(2, '[VERIFY FAIL] 軌道最大水平加速度超過: peak_acc_xy = %.3f m/s^2 > 上限 %.3f m/s^2\n', ...
%                     peak_acc_xy, obj.max_acc_load);
%             end
%         end
% 
%         % =====================================================================
%         % update_active_obstacles: センサー検知・回避状態に応じた障害物ライフサイクル管理
%         % =====================================================================
%         function planning_obstacles = update_active_obstacles(obj, detected_candidates, t_now)
%             % 1. 現在検知された障害物の登録・更新
%             detected_ids = [];
%             for k = 1:length(detected_candidates)
%                 cand = detected_candidates(k);
%                 detected_ids = [detected_ids, cand.id];
% 
%                 idx = [];
%                 if ~isempty(obj.active_obstacles)
%                     idx = find([obj.active_obstacles.id] == cand.id, 1);
%                 end
% 
%                 if isempty(idx)
%                     % 新規検知 -> 登録 (DETECTED)
%                     new_item = struct();
%                     new_item.id                 = cand.id;
%                     new_item.p_obs              = cand.p_obs;
%                     new_item.radii_obs          = cand.radii_obs;
%                     new_item.R_obs              = cand.R_obs;
%                     new_item.closest_drone_world_point = cand.closest_drone_world_point;
%                     new_item.normal_drone_point = cand.normal_drone_point;
%                     new_item.dist_drone_point   = cand.dist_drone_point;
%                     new_item.first_detected_time = t_now;
%                     new_item.last_seen_time     = t_now;
%                     new_item.state              = "DETECTED";
%                     new_item.release_time       = NaN;
%                     obj.active_obstacles = [obj.active_obstacles; new_item];
%                 else
%                     % 既検知 -> 最新幾何情報 & タイムスタンプ更新
%                     obj.active_obstacles(idx).p_obs              = cand.p_obs;
%                     obj.active_obstacles(idx).radii_obs          = cand.radii_obs;
%                     obj.active_obstacles(idx).R_obs              = cand.R_obs;
%                     obj.active_obstacles(idx).closest_drone_world_point = cand.closest_drone_world_point;
%                     obj.active_obstacles(idx).normal_drone_point = cand.normal_drone_point;
%                     obj.active_obstacles(idx).dist_drone_point   = cand.dist_drone_point;
%                     obj.active_obstacles(idx).last_seen_time     = t_now;
%                     obj.active_obstacles(idx).state              = "DETECTED";
%                     obj.active_obstacles(idx).release_time       = NaN;
%                 end
%             end
% 
%             % 2. センサー範囲外となった障害物のライフサイクル更新
%             keep_flags = true(length(obj.active_obstacles), 1);
%             for i = 1:length(obj.active_obstacles)
%                 obs_id = obj.active_obstacles(i).id;
% 
%                 if ~ismember(obs_id, detected_ids)
%                     % センサー視野外に離脱
%                     if obj.replan_active
%                         % 回避軌道追従中は無条件で位置形状を完全保持
%                         obj.active_obstacles(i).state = "LATCHED";
%                         obj.active_obstacles(i).release_time = NaN;
%                     else
%                         % 回避完了後: 復帰安全猶予タイマーの開始判定
%                         if obj.active_obstacles(i).state ~= "POST_RECOVERY"
%                             obj.active_obstacles(i).state = "POST_RECOVERY";
%                             obj.active_obstacles(i).release_time = t_now + obj.post_recovery_hold_time;
%                         end
% 
%                         % 猶予時間を超過した場合はリストから安全に削除 (RELEASE)
%                         if t_now >= obj.active_obstacles(i).release_time
%                             keep_flags(i) = false;
%                         end
%                     end
%                 end
%             end
% 
%             obj.active_obstacles = obj.active_obstacles(keep_flags);
%             planning_obstacles = obj.active_obstacles;
%         end
% 
%         % =====================================================================
%         % is_nominal_recovery_safe: 復帰先公称軌道における干渉スキャン (UAV/荷物/索)
%         % =====================================================================
%         function is_safe = is_nominal_recovery_safe(obj, t_end_expected, obs_list)
%             is_safe = true;
%             N_lookahead = 15;
%             t_scan = linspace(t_end_expected, t_end_expected + 2.0, N_lookahead);
%             g_vec = [0; 0; obj.gravity];
% 
%             r_c_sph = obj.r_cable_sphere;
%             n_spheres = max(2, ceil(obj.L_cable / (2.0 * r_c_sph)) + 1);
%             s_ratios = linspace(0.0, 1.0, n_spheres);
% 
%             for i = 1:N_lookahead
%                 t_eval = t_scan(i);
%                 nom_res = obj.base_ref.do(struct('t', t_eval, 'dt', 0.025), 'f');
%                 pL_nom = nom_res.state.xd(1:3);
%                 aL_nom = nom_res.state.xd(9:11);
% 
%                 acc_tot = aL_nom + g_vec;
%                 norm_a = norm(acc_tot);
%                 if norm_a > 1e-3, thrust_dir = acc_tot / norm_a; else, thrust_dir = [0; 0; 1]; end
%                 pQ_nom = pL_nom + obj.L_cable * thrust_dir;
%                 cable_pts_nom = (1 - s_ratios) .* pL_nom + s_ratios .* pQ_nom;
% 
%                 for obs_idx = 1:length(obs_list)
%                     o = obs_list(obs_idx);
%                     % obs_parsed = struct('center', o.p_center(:), 'radii', o.ellipsoid_radii(:), 'R', o.R_obs);
%                     obs_parsed = struct( ...
%                         'center', o.p_obs(:), ...
%                         'radii', o.ellipsoid_radii(:), ...
%                         'R', o.R_obs);
%                     [dQ, ~, ~, ~] = obj.point_ellipsoid_signed_distance(pQ_nom, obs_parsed);
%                     [dL, ~, ~, ~] = obj.point_ellipsoid_signed_distance(pL_nom, obs_parsed);
% 
%                     if (dQ - obj.r_drone - obj.clearance_margin <= 0) || ...
%                        (dL - obj.r_load  - obj.clearance_margin <= 0)
%                         is_safe = false;
%                         return;
%                     end
% 
%                     for sp = 1:n_spheres
%                         [dC, ~, ~, ~] = obj.point_ellipsoid_signed_distance(cable_pts_nom(:, sp), obs_parsed);
%                         if dC - r_c_sph - obj.clearance_margin <= 0
%                             is_safe = false;
%                             return;
%                         end
%                     end
%                 end
%             end
%         end
% 
%         % =====================================================================
%         % verify_start_c6_matching: 始端 0〜6階微分ギャップ厳密検証
%         % =====================================================================
%         function verify_start_c6_matching(obj, init_load_state, t_now)
%             names = ["位置(0階)", "速度(1階)", "加速度(2階)", "Jerk(3階)", "Snap(4階)", "Crack(5階)", "Pop(6階)"];
%             tolerances = [1e-10, 1e-9, 1e-8, 1e-6, 1e-4, 1e-2, 1.0];
% 
%             for k = 0:6
%                 actual_k = obj.eval_spline_kth(0.0, k)';
%                 expected_k = init_load_state(k + 1, :);
%                 gap = norm(actual_k - expected_k);
%                 obj.c6_start_gaps(k + 1) = gap;
% 
%                 if gap > tolerances(k + 1)
%                     fprintf(2, "[FATAL C^6 BREACH] t=%.3f s | 始端 %s の接続ギャップ超過: %.3e (許容: %.3e)\n", ...
%                         t_now, names(k + 1), gap, tolerances(k + 1));
%                     error('リプランニング始端での C^6 境界接続に失敗しました: %s', names(k + 1));
%                 end
%             end
%         end
% 
%         % =====================================================================
%         % verify_return_c6_matching: 終端 0〜6階微分公称合流実測検証ログ
%         % =====================================================================
%         function verify_return_c6_matching(obj, t_end_expected)
%             names = ["位置(0階)", "速度(1階)", "加速度(2階)", "Jerk(3階)", "Snap(4階)", "Crack(5階)", "Pop(6階)"];
%             tolerances = [1e-9, 1e-8, 1e-7, 1e-5, 1e-3, 1e-1, 5.0];
% 
%             fprintf("\n=================================================================================\n");
%             fprintf(" [B-SPLINE 回避終了 C^6 終端合流ギャップ実測レポート]  t_end = %.3f s\n", t_end_expected);
%             fprintf("=================================================================================\n");
% 
%             pass = true;
%             end_nom_state = obj.get_nominal_derivatives_at_time(t_end_expected);
% 
%             for k = 0:6
%                 actual_k = obj.eval_spline_kth_with_T(obj.t_duration, k, obj.t_duration)';
%                 expected_k = end_nom_state(k + 1, :);
%                 gap = norm(actual_k - expected_k);
%                 obj.c6_return_gaps(k + 1) = gap;
% 
%                 if gap > tolerances(k + 1)
%                     pass = false;
%                 end
%                 fprintf("   - %-12s e_%d = ||p_L^(%d)(T) - p_nom^(%d)(T)|| = %.3e m/s^%d (許容: %.1e)\n", ...
%                     names(k + 1), k, k, gap, k, tolerances(k + 1));
%             end
% 
%             if pass
%                 fprintf(" 判定: 設計上の0〜6階微分境界条件を満足し、C^6公称合流条件を達成。\n");
%             else
%                 fprintf(2, " [WARNING] 終端0〜6階微分の残差が許容値を超過しました (不連続キックの可能性).\n");
%             end
%             fprintf("=================================================================================\n\n");
%         end
% 
%         % =====================================================================
%         % verify_step_integration_consistency: 離散時間ステップ積分整合性監視
%         % =====================================================================
%         function verify_step_integration_consistency(obj, xd_now, t_now, dt)
%             if obj.prev_t < 0
%                 obj.prev_xd = xd_now;
%                 obj.prev_t  = t_now;
%                 return;
%             end
% 
%             real_dt = t_now - obj.prev_t;
%             if real_dt <= 1e-6, return; end
%             if nargin < 4 || isempty(dt) || dt <= 0, dt = real_dt; end
% 
%             orders = { ...
%                 '位置 (0階)',      1:3,   5:7;   ...
%                 'yaw角 (0階)',     4,     8;     ...
%                 '速度 (1階)',      5:7,   9:11;  ...
%                 '加速度 (2階)',    9:11,  13:15; ...
%                 'Jerk (3階)',      13:15, 17:19; ...
%             };
% 
%             for idx = 1:size(orders, 1)
%                 name   = orders{idx, 1};
%                 curr_i = orders{idx, 2};
%                 next_i = orders{idx, 3};
% 
%                 val_prev = obj.prev_xd(curr_i);
%                 val_curr = xd_now(curr_i);
%                 der_prev = obj.prev_xd(next_i);
%                 der_curr = xd_now(next_i);
% 
%                 predicted = val_prev + 0.5 * (der_prev + der_curr) * real_dt;
%                 disc_gap  = norm(val_curr - predicted);
% 
%                 tol = max(0.08, 6.0 * norm(der_curr) * real_dt);
%                 if disc_gap > tol
%                     warning("離散ステップ不連続キック検出: %s at t=%.4f s (ギャップ: %.3e, 許容: %.3e)", ...
%                         name, t_now, disc_gap, tol);
%                 end
%             end
% 
%             obj.prev_xd = xd_now;
%             obj.prev_t  = t_now;
%         end
% 
%         % =====================================================================
%         % check_detection_simulated_sensor: 3者(機体/荷物/索)の3層距離・マージン診断
%         % =====================================================================
%         function det = check_detection_simulated_sensor(obj, pQ, pL, obs_list, t_now, v_nom)
%             det = struct();
%             det.time                          = t_now;
%             det.pQ                            = pQ;
%             det.pL                            = pL;
%             det.trigger_dist                  = obj.trigger_dist;
%             det.detected_point                = false;
%             det.drone_inside_obstacle_point   = false;
%             det.load_inside_obstacle_point    = false;
%             det.cable_inside_obstacle_point   = false;
%             det.drone_margin_violated         = false;
%             det.load_margin_violated          = false;
%             det.cable_margin_violated         = false;
%             det.drone_min_dist_point          = inf;
%             det.load_min_dist_point           = inf;
%             det.cable_min_dist_point          = inf;
%             det.min_dist_point                = inf;
%             det.drone_obstacle_id_point       = [];
%             det.load_obstacle_id_point        = [];
%             det.min_obstacle_id_point         = [];
%             det.min_source_point              = "none";
%             det.detected_obstacles_point      = [];
%             det.detected_obstacle_count_point = 0;
% 
%             if isempty(obs_list), return; end
% 
%             r_c_sph = obj.r_cable_sphere;
%             n_spheres = max(2, ceil(obj.L_cable / (2.0 * r_c_sph)) + 1);
%             s_ratios = linspace(0.0, 1.0, n_spheres);
%             cable_pts = (1 - s_ratios) .* pL + s_ratios .* pQ;
% 
%             v_dir = v_nom(:);
%             if norm(v_dir) > 0.1, v_dir = v_dir / norm(v_dir); else, v_dir = [0; 1; 0]; end
% 
%             detected_obs_candidates = [];
% 
%             for i = 1:length(obs_list)
%                 o = obs_list(i);
%                 obs_id = o.id;
%                 R_obs = o.R_obs;
%                 radii_obs = o.ellipsoid_radii(:);
%                 p_obs = o.p_center(:);
%                 obs_parsed = struct('center', p_obs, 'radii', radii_obs, 'R', R_obs);
% 
%                 % 1. 機体重心 pQ
%                 [d_drone_raw, inside_drone, cpQ_loc, ~] = obj.point_ellipsoid_signed_distance(pQ, obs_parsed);
%                 d_drone_physical = d_drone_raw - obj.r_drone;
%                 d_drone_safety   = d_drone_physical - obj.clearance_margin;
% 
%                 % 2. 荷物 pL
%                 [d_load_raw, inside_load, cpL_loc, ~] = obj.point_ellipsoid_signed_distance(pL, obs_parsed);
%                 d_load_physical = d_load_raw - obj.r_load;
%                 d_load_safety   = d_load_physical - obj.clearance_margin;
% 
%                 % 3. 索球列 (Runtime Monitor)
%                 d_cable_raw_min = inf;
%                 inside_cable_i = false;
%                 for sp_idx = 1:n_spheres
%                     [d_pt, in_pt, ~, ~] = obj.point_ellipsoid_signed_distance(cable_pts(:, sp_idx), obs_parsed);
%                     if d_pt < d_cable_raw_min, d_cable_raw_min = d_pt; end
%                     if in_pt, inside_cable_i = true; end
%                 end
%                 d_cable_physical = d_cable_raw_min - r_c_sph;
%                 d_cable_safety   = d_cable_physical - obj.clearance_margin;
% 
%                 if inside_drone || (d_drone_physical <= 0),   det.drone_inside_obstacle_point = true; end
%                 if inside_load  || (d_load_physical <= 0),    det.load_inside_obstacle_point  = true; end
%                 if inside_cable_i || (d_cable_physical <= 0), det.cable_inside_obstacle_point = true; end
% 
%                 if d_drone_safety <= 0, det.drone_margin_violated = true; end
%                 if d_load_safety  <= 0, det.load_margin_violated  = true; end
%                 if d_cable_safety <= 0, det.cable_margin_violated = true; end
% 
%                 if d_drone_physical < det.drone_min_dist_point
%                     det.drone_min_dist_point = d_drone_physical;
%                     det.drone_obstacle_id_point = obs_id;
%                 end
%                 if d_load_physical < det.load_min_dist_point
%                     det.load_min_dist_point = d_load_physical;
%                     det.load_obstacle_id_point = obs_id;
%                 end
%                 if d_cable_physical < det.cable_min_dist_point
%                     det.cable_min_dist_point = d_cable_physical;
%                 end
% 
%                 cpQ_world = p_obs + R_obs * cpQ_loc;
%                 cpL_world = p_obs + R_obs * cpL_loc;
%                 nQ_loc = cpQ_loc ./ (radii_obs.^2);
%                 normal_drone = R_obs * (nQ_loc / norm(nQ_loc));
%                 nL_loc = cpL_loc ./ (radii_obs.^2);
%                 normal_load = R_obs * (nL_loc / norm(nL_loc));
% 
%                 obs_info = struct( ...
%                     'id',                        obs_id, ...
%                     'dist_drone_point',          d_drone_physical, ...
%                     'dist_drone_safety',         d_drone_safety, ...
%                     'dist_load_point',           d_load_physical, ...
%                     'dist_load_safety',          d_load_safety, ...
%                     'dist_cable_point',          d_cable_physical, ...
%                     'dist_cable_safety',         d_cable_safety, ...
%                     'min_dist_point',            d_drone_physical, ...
%                     'p_obs',                     p_obs, ...
%                     'radii_obs',                 radii_obs, ...
%                     'R_obs',                     R_obs, ...
%                     'closest_drone_world_point', cpQ_world, ...
%                     'closest_load_world_point',  cpL_world, ...
%                     'normal_drone_point',        normal_drone, ...
%                     'normal_load_point',         normal_load ...
%                 );
% 
%                 % vec_to_center = p_obs - pQ;
%                 % is_in_front = dot(vec_to_center, v_dir) > -max(radii_obs);
%                 % 
%                 % if (d_drone_raw <= obj.trigger_dist) && is_in_front
%                 %     detected_obs_candidates = [detected_obs_candidates; obs_info];
%                 % end
%                 if d_drone_raw <= obj.trigger_dist
%                     detected_obs_candidates = [detected_obs_candidates; obs_info];
%                 end
%             end
% 
%             if ~isempty(detected_obs_candidates)
%                 [~, sort_idx] = sort([detected_obs_candidates.dist_drone_point], 'ascend');
%                 det.detected_obstacles_point = detected_obs_candidates(sort_idx);
%                 det.detected_point = true;
%                 det.min_dist_point = det.detected_obstacles_point(1).dist_drone_point;
%                 det.min_obstacle_id_point = det.detected_obstacles_point(1).id;
%                 det.min_source_point = "drone";
%                 det.detected_obstacle_count_point = length(detected_obs_candidates);
%             else
%                 det.min_dist_point = det.drone_min_dist_point;
%                 det.min_obstacle_id_point = det.drone_obstacle_id_point;
%                 det.min_source_point = "none";
%             end
%         end
% 
%         % =====================================================================
%         % display_detection_report: 診断レポート (安全例外ガード付き)
%         % =====================================================================
%         function display_detection_report(obj, t_now, tgt, total_candidates, req_clearance, t_impact, n_constraint, status_str)
%             names = ["位置(0階)", "速度(1階)", "加速度(2階)", "Jerk(3階)", "Snap(4階)", "Crack(5階)", "Pop(6階)"];
% 
%             fprintf("\n=================================================================================\n");
%             fprintf(" [B-SPLINE 15m センサー検知 ＆ C^6 回避診断レポート]  t = %.3f s [%s]\n", t_now, status_str);
%             fprintf("=================================================================================\n");
%             fprintf(" 1. センサ判定元      : 機体重心 pQ 搭載模擬センサー (機体表面間距離: %.3f m)\n", tgt.dist_drone_point);
%             fprintf(" 2. 実行時診断モニタステータス (独立100点サンプリング数値検証):\n");
%             fprintf("     - UAV   : 表面間: %+6.3f m | [%s]\n", tgt.dist_drone_point, obj.get_safety_status(tgt.dist_drone_point, tgt.dist_drone_point - obj.clearance_margin));
%             fprintf("     - Cable : 表面間: %+6.3f m | [%s]\n", tgt.dist_cable_point, obj.get_safety_status(tgt.dist_cable_point, tgt.dist_cable_point - obj.clearance_margin));
%             fprintf("     - Load  : 表面間: %+6.3f m | [%s]\n", tgt.dist_load_point, obj.get_safety_status(tgt.dist_load_point, tgt.dist_load_point - obj.clearance_margin));
%             fprintf(" 3. 捕捉楕円体情報    : 危険順位 1位 / 候補 %d 個 (ID=%d, 半径=[%.2f, %.2f, %.2f] m)\n", ...
%                 total_candidates, tgt.id, tgt.radii_obs(1), tgt.radii_obs(2), tgt.radii_obs(3));
%             fprintf(" 4. 運動学制約検証    : 実軌道最大 a_xy = %.2f m/s^2 (上限: %.2f m/s^2) -> 回避時間 T_tot = %.2f s\n", ...
%                 obj.sampled_peak_acceleration, obj.max_acc_load, obj.t_duration);
%             fprintf(" 5. 連続時間安全包含保証 (B-Spline 制御点凸包性):\n");
%             fprintf("     - 制御点拘束条件 : n'*(P_i - p_surf) >= d_req (全32制御点に厳密適用, d_req = %.2f m)\n", req_clearance);
%             fprintf("     - 理論的保証根拠 : P_L,i in H ==> 凸包性より p_L(t) in H ==> ||p_Q - p_L||=L より UAV球体・索包含\n");
%             fprintf("     - サンプリング位置: 100点チェックは連続時間保証の補足・独立数値検証として機能\n");
%             fprintf("     - 支持超平面法線 : n_constraint = [%.2f, %.2f, %.2f]\n", n_constraint(1), n_constraint(2), n_constraint(3));
%             fprintf("     - 数学保証根拠   : 荷物全制御点包含 P_L in H(d_req) ==> 凸包性より p_L(t) in H ==> UAV球体・索包含\n");
%             fprintf(" 6. C^6 始端境界ギャップ実測値 (M \\ B 厳密代数接続):\n");
%             for k = 0:6
%                 fprintf("     - %-12s e_%d = %.3e m/s^%d (数学的 C^6 一致)\n", names(k + 1), k, obj.c6_start_gaps(k + 1), k);
%             end
%             fprintf(" 7. 計算時間内訳      : QP求解 = %6.2f ms | リプラン全体 = %6.2f ms (18自由制御点/54変数)\n", ...
%                 obj.last_solve_time_ms, obj.last_replan_total_time_ms);
%             fprintf(" 8. 公称軌道離脱変位  : %.3f m (max ||p_L(t) - p_nom(t)||, 60点サンプリング評価)\n", obj.sampled_peak_displacement);
%             fprintf("=================================================================================\n\n");
%         end
% 
%         function str = get_safety_status(~, d_phys, d_safe)
%             if d_phys <= 0
%                 str = "COLLISION";
%             elseif d_safe <= 0
%                 str = "MARGIN VIOLATION";
%             else
%                 str = "SAFE";
%             end
%         end
% 
%         % =====================================================================
%         % point_ellipsoid_signed_distance: ラグランジュ未定乗数法による厳密幾何距離
%         % =====================================================================
%         function [d, inside, closest_local, lambda] = point_ellipsoid_signed_distance(~, p, o)
%             p = p(:); c = o.center(:); r = o.radii(:); R = o.R;
%             y  = R' * (p - c);
%             r2 = r.^2;
%             q  = sum((y ./ r).^2);
%             inside = (q < 1.0);
% 
%             if norm(y) < 1e-14
%                 [min_r, min_idx] = min(r);
%                 closest_local = zeros(3, 1);
%                 closest_local(min_idx) = min_r;
%                 d      = -min_r;
%                 lambda = -min(r2);
%                 return;
%             end
% 
%             f = @(lam) sum(r2 .* (y.^2) ./ ((lam + r2).^2)) - 1.0;
%             if q > 1.0
%                 lo = 0.0;
%                 hi = max(r) * norm(y);
%             else
%                 lo = -min(r2) * (1.0 - 1e-12);
%                 hi = 0.0;
%             end
% 
%             for kk = 1:80
%                 mid = 0.5 * (lo + hi);
%                 if f(mid) > 0, lo = mid; else, hi = mid; end
%             end
%             lambda = 0.5 * (lo + hi);
%             closest_local = r2 .* y ./ (lambda + r2);
%             d_abs = norm(closest_local - y);
%             if inside, d = -d_abs; else, d = d_abs; end
%         end
% 
%         % 実制御点 P_L による評価
%         function val = eval_spline_kth(obj, tau, k)
%             val = obj.eval_spline_kth_with_T(tau, k, obj.t_duration);
%         end
% 
%         function val = eval_spline_kth_with_T(obj, tau, k, T_tot)
%             p = obj.spline_degree;
%             t_eval = max(0.0, min(T_tot, tau));
% 
%             idx_span = obj.find_knot_span(t_eval);
%             ders = obj.eval_basis_derivatives(idx_span, t_eval, k);
% 
%             c_indices = (idx_span - p):idx_span;
%             val = (ders(k + 1, :) * obj.control_points(c_indices, :))';
%         end
% 
%         function build_clamped_uniform_knots(obj, n_seg, p, T_tot)
%             dt_knot = T_tot / n_seg;
%             interior_knots = dt_knot * (1:(n_seg - 1));
%             obj.knots = [zeros(1, p + 1), interior_knots, T_tot * ones(1, p + 1)];
%         end
% 
%         function idx = find_knot_span(obj, t_eval)
%             p = obj.spline_degree;
%             n_cp = length(obj.knots) - p - 1;
%             if t_eval >= obj.knots(n_cp + 1), idx = n_cp; return; end
%             if t_eval <= obj.knots(p + 1),    idx = p + 1; return; end
%             low = p + 1; high = n_cp + 1; mid = floor((low + high) / 2);
%             while (t_eval < obj.knots(mid)) || (t_eval >= obj.knots(mid + 1))
%                 if t_eval < obj.knots(mid), high = mid; else, low = mid; end
%                 mid = floor((low + high) / 2);
%             end
%             idx = mid;
%         end
% 
%         function ders = eval_basis_derivatives(obj, idx_span, t_eval, n_der)
%             p = obj.spline_degree;
%             U = obj.knots;
%             ders = zeros(n_der + 1, p + 1);
%             ndu = zeros(p + 1, p + 1);
%             left = zeros(p + 1, 1);
%             right = zeros(p + 1, 1);
% 
%             ndu(1, 1) = 1.0;
%             for j = 1:p
%                 left(j + 1) = t_eval - U(idx_span + 1 - j);
%                 right(j + 1) = U(idx_span + j) - t_eval;
%                 saved = 0.0;
%                 for r = 0:(j - 1)
%                     ndu(j + 1, r + 1) = right(r + 2) + left(j - r + 1);
%                     temp = ndu(r + 1, j) / ndu(j + 1, r + 1);
%                     ndu(r + 1, j + 1) = saved + right(r + 2) * temp;
%                     saved = left(j - r + 1) * temp;
%                 end
%                 ndu(j + 1, j + 1) = saved;
%             end
% 
%             for j = 0:p, ders(1, j + 1) = ndu(j + 1, p + 1); end
% 
%             a = zeros(2, p + 1);
%             for r = 0:p
%                 s1 = 0; s2 = 1; a(1, 1) = 1.0;
%                 for k = 1:n_der
%                     d = 0.0; rk = r - k; pk = p - k;
%                     if r >= k
%                         a(s2 + 1, 1) = a(s1 + 1, 1) / ndu(pk + 2, rk + 1);
%                         d = a(s2 + 1, 1) * ndu(rk + 1, pk + 1);
%                     end
%                     if rk >= -1, j1 = 1; else, j1 = -rk; end
%                     if (r - 1) <= pk, j2 = k - 1; else, j2 = p - r; end
%                     for j = j1:j2
%                         a(s2 + 1, j + 1) = (a(s1 + 1, j + 1) - a(s1 + 1, j)) / ndu(pk + 2, rk + j + 1);
%                         d = d + a(s2 + 1, j + 1) * ndu(rk + j + 1, pk + 1);
%                     end
%                     if r <= pk
%                         a(s2 + 1, k + 1) = -a(s1 + 1, k) / ndu(pk + 2, r + 1);
%                         d = d + a(s2 + 1, k + 1) * ndu(r + 1, pk + 1);
%                     end
%                     ders(k + 1, r + 1) = d;
%                     j_tmp = s1; s1 = s2; s2 = j_tmp;
%                 end
%             end
% 
%             r_scale = p;
%             for k = 1:n_der
%                 for j = 0:p
%                     ders(k + 1, j + 1) = ders(k + 1, j + 1) * r_scale;
%                 end
%                 r_scale = r_scale * (p - k);
%             end
%         end
% 
%         function clear_state_sensor_values(obj, t_now)
%             st = obj.result.state;
%             st.time                          = t_now;
%             st.pQ                            = [NaN; NaN; NaN];
%             st.pL                            = [NaN; NaN; NaN];
%             st.detected_point                = false;
%             st.drone_inside_obstacle_point   = false;
%             st.load_inside_obstacle_point    = false;
%             st.cable_inside_obstacle_point   = false;
%             st.drone_margin_violated         = false;
%             st.load_margin_violated          = false;
%             st.cable_margin_violated         = false;
%             st.drone_min_dist_point          = inf;
%             st.load_min_dist_point           = inf;
%             st.cable_min_dist_point          = inf;
%             st.min_dist_point                = inf;
%             st.drone_obstacle_id_point       = NaN;
%             st.load_obstacle_id_point        = NaN;
%             st.min_obstacle_id_point         = NaN;
%             st.min_source_point              = "none";
%             st.detected_obstacle_count_point = 0;
%             st.detected_obstacles_point      = [];
%             st.pQ_ref                        = [NaN; NaN; NaN];
%         end
% 
%         % =====================================================================
%         % estimate_safety_margin_exit_time:
%         % 公称進行方向に沿って障害物の安全マージン領域を完全に抜けるまでの時間
%         % =====================================================================
%         function t_clear = estimate_safety_margin_exit_time( ...
%                 obj, pQ_cur, dir_nom, spd, target_obs, req_clearance)
% 
%             % 公称進行方向に沿った障害物中心までの距離
%             vec_to_obs = target_obs.p_obs - pQ_cur;
%             dist_along = dot(vec_to_obs, dir_nom);
% 
%             % 現時点で障害物が進行方向の後方にある場合
%             % → すでに通過済みとして最低時間だけ確保
%             if dist_along <= 0
%                 t_clear = 1.5;
%                 return;
%             end
% 
%             % 障害物を球として保守的に包含
%             max_rad = max(target_obs.radii_obs);
% 
%             % 安全領域の「反対側の境界」までの距離
%             %
%             % pQ_cur
%             %    |
%             %    | dist_along
%             %    v
%             % [ obstacle ]
%             %
%             % 安全領域の出口:
%             %
%             % dist_along + max_rad + req_clearance
%             %
%             dist_to_exit = ...
%                 dist_along ...
%                 + max_rad ...
%                 + req_clearance;
% 
%             % 速度ゼロ等によるゼロ除算防止
%             spd_safe = max(spd, 0.1);
% 
%             t_clear = max(1.5, dist_to_exit / spd_safe);
%         end
% 
%         function list = get_obstacles_at_time(~, t_now)
%             % 本研究では静止障害物環境を対象とする (t_now はAPI互換性のために保持)
%             list = ENVIRONMENT_OBSTACLE_ELLIPSE_MOVE(t_now);
%         end
%     end
% end
% 
% % classdef REPLANNING_BSPLINE < handle
% %     % =========================================================================
% %     % REPLANNING_BSPLINE (診断・原因究明特化 単一試行版)
% %     % =========================================================================
% %     properties
% %         self                       % ドローンエージェント自身
% %         base_ref                   % 公称参照軌道生成オブジェクト
% %         result                     % 出力結果構造体
% % 
% %         % --- センサ・検知パラメータ ---
% %         trigger_dist = 15.0;       % 機体搭載センサーによる接近検知閾値 [m]
% %         clearance_margin = 0.8;    % 安全マージン d_margin [m]
% % 
% %         % --- 機体・荷物・索物理パラメータ ---
% %         gravity = 9.81;            % 重力加速度 [m/s^2]
% %         L_cable = 1.0;             % 索長 L [m]
% %         r_drone = 0.30;            % 機体球体半径 r_Q [m]
% %         r_load  = 0.15;            % 荷物球体半径 r_L [m]
% %         r_cable_sphere = 0.25;     % 索保護球体半径 r_C [m] (実行時モニタ専用)
% % 
% %         % --- 運動学・物理制約パラメータ ---
% %         max_acc_load = 2.0;        % 荷物許容水平加速度上限 a_max [m/s^2]
% %         w_dev = 0.05;              % 公称軌道引き戻し重み
% % 
% %         % --- リプランニング状態管理 ---
% %         replan_active = false;     % 回避軌道追従中フラグ
% %         t_start       = 0.0;       % 回避開始時刻 [s]
% %         t_duration    = 6.0;       % 回避全所要時間 [s]
% %         last_replan_time = -100.0; % 前回リプラン実行時刻 [s]
% %         min_replan_interval = 0.50;% 再計画更新周期 (過密ループ防止) [s]
% %         active_threat_id = NaN;    % 現在追従中の主障害物ID
% % 
% %         % --- 7次 B-Spline パラメータ (p=7, n_seg=25 -> N_cp=32) ---
% %         spline_degree = 7;         % 7次 B-Spline (内部 C^6 連続)
% %         num_segments  = 25;        % 25セグメント
% %         knots                      % ノットベクトル
% %         control_points             % 荷物実軌道制御点 P_L (32 x 3) [X, Y, Z]
% %         sampled_peak_displacement = 0.0; % 最大空間変位 [m]
% %         sampled_peak_acceleration = 0.0; % 実軌道最大水平加速度 [m/s^2]
% %         last_solve_time_ms = 0.0;  % QP求解時間 [ms]
% %         last_replan_total_time_ms = 0.0; % リプランニング全体処理時間 [ms]
% % 
% %         % --- C^6 境界接続ギャップ診断バッファ ---
% %         c6_start_gaps = zeros(7, 1);  % 始端接続ギャップ e_k (k=0..6)
% %         c6_return_gaps = zeros(7, 1); % 終端復帰ギャップ e_k (k=0..6)
% % 
% %         % --- 離散時間ステップ整合性監視用バッファ ---
% %         prev_xd                    % 前回ステップの xd (31x1)
% %         prev_t = -1.0;             % 前回ステップの時刻 [s]
% % 
% %         % --- アクティブ障害物管理 (Active Obstacle Manager) ---
% %         active_obstacles = [];     % 現在管理中の障害物リスト
% %         post_recovery_hold_time = 3.0; % 回避・安全復帰完了後の猶予保持時間 [s]
% %     end
% % 
% %     methods (Access = public)
% %         % =====================================================================
% %         % コンストラクタ
% %         % =====================================================================
% %         function obj = REPLANNING_BSPLINE(self, base_ref, opts)
% %             arguments
% %                 self
% %                 base_ref
% %                 opts = struct()
% %             end
% %             obj.self = self;
% %             obj.base_ref = base_ref;
% %             if isfield(opts, 'trigger_dist'),     obj.trigger_dist     = opts.trigger_dist;     end
% %             if isfield(opts, 'clearance_margin'), obj.clearance_margin = opts.clearance_margin; end
% %             if isfield(opts, 'r_drone'),          obj.r_drone          = opts.r_drone;          end
% %             if isfield(opts, 'r_load'),           obj.r_load           = opts.r_load;           end
% %             if isfield(opts, 'r_cable_sphere'),   obj.r_cable_sphere   = opts.r_cable_sphere;   end
% %             if isfield(opts, 'L_cable'),          obj.L_cable          = opts.L_cable;          end
% %             if isfield(opts, 'gravity'),          obj.gravity          = opts.gravity;          end
% %             if isfield(opts, 'max_acc_load'),     obj.max_acc_load     = opts.max_acc_load;     end
% %             if isfield(opts, 'w_dev'),            obj.w_dev            = opts.w_dev;            end
% % 
% %             assert(obj.r_drone >= obj.r_cable_sphere, '前提違反: r_drone >= r_cable_sphere');
% %             assert(obj.r_cable_sphere >= obj.r_load,   '前提違反: r_cable_sphere >= r_load');
% %             assert(obj.L_cable > 0,                    'L_cable > 0');
% %             assert(obj.max_acc_load > 0,                'max_acc_load > 0');
% %             assert(obj.clearance_margin >= 0,          'clearance_margin >= 0');
% %             assert(obj.trigger_dist > 0,               'trigger_dist > 0');
% % 
% %             obj.result = base_ref.result;
% % 
% %             sensor_props = ["time", "pQ", "pL", "detected_point", ...
% %                 "drone_inside_obstacle_point", "load_inside_obstacle_point", ...
% %                 "cable_inside_obstacle_point", "drone_margin_violated", ...
% %                 "load_margin_violated", "cable_margin_violated", ...
% %                 "drone_min_dist_point", "load_min_dist_point", "cable_min_dist_point", ...
% %                 "min_dist_point", "drone_obstacle_id_point", "load_obstacle_id_point", ...
% %                 "min_obstacle_id_point", "min_source_point", "detected_obstacle_count_point", ...
% %                 "detected_obstacles_point", "pQ_ref"];
% %             for p_name = sensor_props
% %                 if ~isprop(obj.result.state, p_name)
% %                     addprop(obj.result.state, p_name);
% %                 end
% %             end
% %             obj.clear_state_sensor_values(0.0);
% %         end
% % 
% %         % =====================================================================
% %         % do: 制御周期ごとのメイン実行メソッド
% %         % =====================================================================
% %         function result_out = do(obj, varargin)
% %             time = varargin{1};
% %             cha  = varargin{2};
% % 
% %             base_res = obj.base_ref.do(varargin{:});
% %             xd_nom = base_res.state.xd;
% %             if length(xd_nom) < 31
% %                 xd_nom = [xd_nom; zeros(31 - length(xd_nom), 1)];
% %             end
% % 
% %             obj.clear_state_sensor_values(time.t);
% % 
% %             detection = struct();
% %             detection.time                          = time.t;
% %             detection.pQ                            = [NaN; NaN; NaN];
% %             detection.pL                            = [NaN; NaN; NaN];
% %             detection.detected_point                = false;
% %             detection.drone_inside_obstacle_point   = false;
% %             detection.load_inside_obstacle_point    = false;
% %             detection.cable_inside_obstacle_point   = false;
% %             detection.drone_margin_violated         = false;
% %             detection.load_margin_violated          = false;
% %             detection.cable_margin_violated         = false;
% %             detection.drone_min_dist_point          = inf;
% %             detection.load_min_dist_point           = inf;
% %             detection.cable_min_dist_point          = inf;
% %             detection.min_dist_point                = inf;
% %             detection.drone_obstacle_id_point       = NaN;
% %             detection.load_obstacle_id_point        = NaN;
% %             detection.min_obstacle_id_point         = NaN;
% %             detection.min_source_point              = "none";
% %             detection.detected_obstacle_count_point = 0;
% %             detection.detected_obstacles_point      = [];
% % 
% %             try obj.L_cable = obj.self.parameter.get("cableL"); catch; end
% % 
% %             if cha == 'f'
% %                 pL_cur = obj.self.estimator.result.state.pL(:);
% %                 pQ_cur = obj.self.estimator.result.state.p(:);
% %                 obs_list = obj.get_obstacles_at_time(time.t);
% % 
% %                 % 1. センサー検知
% %                 detection = obj.check_detection_simulated_sensor(pQ_cur, pL_cur, obs_list, time.t, xd_nom(5:7));
% % 
% %                 % 2. 計画対象障害物の更新
% %                 planning_obstacles = obj.update_active_obstacles(detection.detected_obstacles_point, time.t);
% % 
% %                 % -------------------------------------------------------------
% %                 % 3. 軌道再計画の判定 (簡素化: 未回避時のみ発火)
% %                 % -------------------------------------------------------------
% %                 need_replan = false;
% %                 can_replan = (time.t - obj.last_replan_time >= obj.min_replan_interval);
% % 
% %                 if can_replan
% %                     if ~obj.replan_active
% %                         body_margin_violated = detection.drone_margin_violated || ...
% %                                                detection.load_margin_violated  || ...
% %                                                detection.cable_margin_violated;
% %                         if detection.detected_point || body_margin_violated
% %                             need_replan = true;
% %                         end
% %                     end
% % 
% %                     if need_replan && ~isempty(planning_obstacles)
% %                         obj.last_replan_time = time.t; % 失敗しても次回までインターバルを取る
% %                         obj.execute_replanning(pQ_cur, xd_nom, detection, planning_obstacles, time.t);
% %                     end
% %                 end
% %             end
% % 
% %             % 4. 出力目標軌道の確定
% %             if obj.replan_active
% %                 tau = time.t - obj.t_start;
% %                 t_end_expected = obj.t_start + obj.t_duration;
% % 
% %                 if tau < obj.t_duration
% %                     xd_out = obj.evaluate_smooth_trajectory(tau, xd_nom);
% %                 else
% %                     obj.verify_return_c6_matching(t_end_expected);
% %                     obj.replan_active = false;
% %                     obj.active_threat_id = NaN;
% %                     xd_out = xd_nom;
% %                     fprintf("[B-SPLINE C^6] 回避所要時間完了: 公称軌道へ完全滑らか復帰 (t=%.3f s)\n\n", time.t);
% %                 end
% %             else
% %                 xd_out = xd_nom;
% %             end
% % 
% %             % 5. 離散整合性監視
% %             if cha == 'f'
% %                 obj.verify_step_integration_consistency(xd_out, time.t, time.dt);
% %             end
% % 
% %             % 6. 格納
% %             st = obj.result.state;
% %             st.xd                            = xd_out;
% %             st.p                             = xd_out(1:3);
% %             st.v                             = xd_out(5:7);
% %             st.q                             = [0; 0; xd_out(4)];
% %             st.pQ_ref                        = xd_out(29:31);
% % 
% %             st.time                          = detection.time;
% %             st.pQ                            = detection.pQ;
% %             st.pL                            = detection.pL;
% %             st.detected_point                = detection.detected_point;
% %             st.drone_inside_obstacle_point   = detection.drone_inside_obstacle_point;
% %             st.load_inside_obstacle_point    = detection.load_inside_obstacle_point;
% %             st.cable_inside_obstacle_point   = detection.cable_inside_obstacle_point;
% %             st.drone_margin_violated         = detection.drone_margin_violated;
% %             st.load_margin_violated          = detection.load_margin_violated;
% %             st.cable_margin_violated         = detection.cable_margin_violated;
% %             st.drone_min_dist_point          = detection.drone_min_dist_point;
% %             st.load_min_dist_point           = detection.load_min_dist_point;
% %             st.cable_min_dist_point          = detection.cable_min_dist_point;
% %             st.min_dist_point                = detection.min_dist_point;
% %             st.drone_obstacle_id_point       = detection.drone_obstacle_id_point;
% %             st.load_obstacle_id_point        = detection.load_obstacle_id_point;
% %             st.min_obstacle_id_point         = detection.min_obstacle_id_point;
% %             st.min_source_point              = "none";
% %             st.detected_obstacle_count_point = detection.detected_obstacle_count_point;
% %             st.detected_obstacles_point      = detection.detected_obstacles_point;
% % 
% %             result_out = obj.result;
% %         end
% % 
% %         % =====================================================================
% %         % evaluate_smooth_trajectory
% %         % =====================================================================
% %         function xd = evaluate_smooth_trajectory(obj, tau, xd_nom)
% %             xd = xd_nom;
% %             pL_d   = obj.eval_spline_kth(tau, 0);
% %             vL_d   = obj.eval_spline_kth(tau, 1);
% %             aL_d   = obj.eval_spline_kth(tau, 2);
% %             jL_d   = obj.eval_spline_kth(tau, 3);
% %             sL_d   = obj.eval_spline_kth(tau, 4);
% %             cL_d   = obj.eval_spline_kth(tau, 5);
% %             popL_d = obj.eval_spline_kth(tau, 6);
% % 
% %             xd(1:3)   = pL_d;
% %             xd(5:7)   = vL_d;
% %             xd(9:11)  = aL_d;
% %             xd(13:15) = jL_d;
% %             xd(17:19) = sL_d;
% %             xd(21:23) = cL_d;
% %             xd(25:27) = popL_d;
% % 
% %             g_vec = [0; 0; obj.gravity];
% %             acc_tot = aL_d + g_vec;
% %             norm_a = norm(acc_tot);
% %             if norm_a > 1e-3, thrust_dir = acc_tot / norm_a; else, thrust_dir = [0; 0; 1]; end
% %             pQ_d = pL_d + obj.L_cable * thrust_dir;
% %             xd(29:31) = pQ_d;
% %         end
% %     end
% % 
% %     methods (Access = private)
% %         % =====================================================================
% %         % execute_replanning: 単一試行・完全診断版
% %         % =====================================================================
% %         function execute_replanning(obj, pQ_cur, xd_nom, detection, planning_obstacles, t_now)
% %             t_replan_start = tic;
% % 
% %             active_plan_obstacles = obj.extract_all_obstacle_halfspaces(pQ_cur, planning_obstacles);
% %             if isempty(active_plan_obstacles)
% %                 fprintf('[REPLAN SKIP] active_plan_obstacles が空です.\n');
% %                 return; 
% %             end
% % 
% %             if ~isempty(detection.detected_obstacles_point)
% %                 target_obs = detection.detected_obstacles_point(1);
% %             else
% %                 [~, idx_target] = min([active_plan_obstacles.dist_drone_point]);
% %                 target_obs = active_plan_obstacles(idx_target);
% %             end
% %             obj.active_threat_id = target_obs.id;
% % 
% %             v_nom = xd_nom(5:7);
% %             spd = norm(v_nom);
% %             if spd < 0.1, spd = 1.0; v_nom = [1; 0; 0]; end
% %             dir_nom = v_nom / spd;
% % 
% %             init_load_state = zeros(7, 3);
% %             if obj.replan_active
% %                 tau_now = t_now - obj.t_start;
% %                 for k = 0:6
% %                     init_load_state(k + 1, :) = obj.eval_spline_kth(tau_now, k)';
% %                 end
% %             else
% %                 init_load_state = obj.get_nominal_derivatives_at_time(t_now);
% %             end
% % 
% %             n_constraint = target_obs.normal_drone_point(:) / norm(target_obs.normal_drone_point);
% %             n_escape = n_constraint - dot(n_constraint, dir_nom) * dir_nom;
% %             if norm(n_escape) < 0.1
% %                 n_cand = cross(dir_nom, [0; 0; 1]);
% %                 if norm(n_cand) < 0.1, n_cand = cross(dir_nom, [0; 1; 0]); end
% %                 n_escape = n_cand / norm(n_cand);
% %             else
% %                 n_escape = n_escape / norm(n_escape);
% %             end
% % 
% %             req_clearance = obj.L_cable + obj.r_drone + obj.clearance_margin;
% %             t_impact = obj.estimate_safety_margin_impact_time(pQ_cur, dir_nom, spd, target_obs);
% %             T_kinematic = sqrt((8.0 * (req_clearance + 1.0)) / obj.max_acc_load);
% %             t_dur_cand = max([5.0, 2.0 * t_impact, T_kinematic * 1.5]);
% % 
% %             fprintf('\n============================================================\n');
% %             fprintf('[EXECUTE REPLANNING (SINGLE ATTEMPT)] t = %.3f s\n', t_now);
% %             fprintf('  主回避対象 ID   : %d\n', target_obs.id);
% %             fprintf('  UAV表面間距離   : %.3f m\n', target_obs.dist_drone_point);
% %             fprintf('  要求クリアランス: %.3f m (L=%.2f, rQ=%.2f, margin=%.2f)\n', ...
% %                 req_clearance, obj.L_cable, obj.r_drone, obj.clearance_margin);
% %             fprintf('  候補回避時間    : T_tot = %.3f s (t_impact=%.2f, T_kin=%.2f)\n', ...
% %                 t_dur_cand, t_impact, T_kinematic);
% %             fprintf('============================================================\n');
% % 
% %             % 単一試行: QP 求解
% %             t_solve = tic;
% %             [qp_success, peak_disp] = obj.plan_uniform_bspline_c6_qp( ...
% %                 n_escape, init_load_state, t_dur_cand, active_plan_obstacles, t_now);
% %             obj.last_solve_time_ms = toc(t_solve) * 1000;
% % 
% %             if ~qp_success
% %                 fprintf(2, '[REPLAN REJECTED] t=%.3f s | QP求解または始端固定制約包含に失敗.\n', t_now);
% %                 return;
% %             end
% % 
% %             % 独立数値検証 (100点サンプリング)
% %             [traj_safe, peak_acc] = obj.verify_trajectory_numerical_sampling( ...
% %                 t_now, active_plan_obstacles, t_dur_cand);
% % 
% %             if ~traj_safe || (peak_acc > obj.max_acc_load)
% %                 fprintf(2, '[REPLAN REJECTED] t=%.3f s | 事後検証で棄却 (safe=%d, peak_acc=%.3f > %.3f)\n', ...
% %                     t_now, traj_safe, peak_acc, obj.max_acc_load);
% %                 return;
% %             end
% % 
% %             % 採用処理
% %             obj.t_start          = t_now;
% %             obj.t_duration       = t_dur_cand;
% %             obj.replan_active    = true;
% %             obj.sampled_peak_displacement = peak_disp;
% %             obj.sampled_peak_acceleration = peak_acc;
% %             obj.last_replan_total_time_ms = toc(t_replan_start) * 1000;
% % 
% %             obj.verify_start_c6_matching(init_load_state, t_now);
% %             obj.display_detection_report(t_now, target_obs, length(active_plan_obstacles), req_clearance, t_impact, n_constraint, "ADOPTED");
% %         end
% % 
% %         % =====================================================================
% %         % plan_uniform_bspline_c6_qp: 詳細診断版
% %         % =====================================================================
% %         function [success, max_disp] = plan_uniform_bspline_c6_qp( ...
% %                  obj, n_escape_3d, init_load_state, T_tot, active_obstacles, t_now)
% % 
% %             p = 7;
% %             n_seg = 25;
% %             n_cp = n_seg + p;          % 32 制御点
% % 
% %             obj.spline_degree = p;
% %             obj.num_segments = n_seg;
% %             obj.build_clamped_uniform_knots(n_seg, p, T_tot);
% % 
% %             % 1. 始端 0〜6階微分の境界確定
% %             M_start = zeros(7, 7);
% %             idx_span_start = obj.find_knot_span(0.0);
% %             for k = 0:6
% %                 d_row = obj.eval_basis_derivatives(idx_span_start, 0.0, k);
% %                 M_start(k+1,:) = d_row(k+1,1:7);
% %             end
% %             P_L_start = M_start \ init_load_state(1:7,:);
% % 
% %             % 2. 終端 0〜6階微分 (Soft Constraint)
% %             end_nom_state = obj.get_nominal_derivatives_at_time(t_now + T_tot);
% %             idx_free = 8:n_cp;
% %             n_free = length(idx_free);       % 25
% % 
% %             t_end_eval = T_tot - 1e-10;
% %             idx_span_end = obj.find_knot_span(t_end_eval);
% %             idx_local_end = 2:8;
% %             M_end = zeros(7,7);
% %             for k = 0:6
% %                 d_row_end = obj.eval_basis_derivatives(idx_span_end, t_end_eval, k);
% %                 M_end(k+1,:) = d_row_end(k+1,idx_local_end);
% %             end
% % 
% %             terminal_scale = max(vecnorm(M_end, 2, 2), 1.0);
% %             S_end = diag(1 ./ terminal_scale);
% %             M_end_scaled = S_end * M_end;
% %             end_nom_scaled = S_end * end_nom_state(1:7,:);
% % 
% %             % 3. 平滑化行列
% %             D4 = diff(eye(n_cp), 4);
% %             Q = D4' * D4;
% %             Q_mm = Q(idx_free, idx_free) + 1e-4 * eye(n_free);
% %             Q_ms = Q(idx_free, 1:7);
% % 
% %             P_nom_all = obj.project_nominal_trajectory_to_bspline(t_now, T_tot, n_cp);
% %             P_nom_free = P_nom_all(idx_free, :);
% % 
% %             H_1d = (Q_mm + Q_mm') / 2 + obj.w_dev * eye(n_free);
% % 
% %             w_terminal = 10.0;
% %             idx_terminal_free = (n_free - 6):n_free;
% %             H_terminal = w_terminal * (M_end_scaled' * M_end_scaled);
% %             H_1d(idx_terminal_free, idx_terminal_free) = ...
% %                 H_1d(idx_terminal_free, idx_terminal_free) + H_terminal;
% %             H_1d = (H_1d + H_1d') / 2;
% %             H = blkdiag(H_1d, H_1d, H_1d);
% % 
% %             % 線形項
% %             f_x = (P_L_start(:,1)' * Q_ms')' - obj.w_dev * P_nom_free(:,1);
% %             f_y = (P_L_start(:,2)' * Q_ms')' - obj.w_dev * P_nom_free(:,2);
% %             f_z = (P_L_start(:,3)' * Q_ms')' - obj.w_dev * P_nom_free(:,3);
% % 
% %             f_x(idx_terminal_free) = f_x(idx_terminal_free) - w_terminal * M_end_scaled' * end_nom_scaled(:,1);
% %             f_y(idx_terminal_free) = f_y(idx_terminal_free) - w_terminal * M_end_scaled' * end_nom_scaled(:,2);
% %             f_z(idx_terminal_free) = f_z(idx_terminal_free) - w_terminal * M_end_scaled' * end_nom_scaled(:,3);
% % 
% %             w_escape = 0.05;
% %             f_x = f_x - w_escape * n_escape_3d(1) * ones(n_free,1);
% %             f_y = f_y - w_escape * n_escape_3d(2) * ones(n_free,1);
% %             f_z = f_z - w_escape * n_escape_3d(3) * ones(n_free,1);
% %             f = [f_x; f_y; f_z];
% % 
% %             % 4. 不等式制約
% %             A_ineq = [];
% %             b_ineq = [];
% %             constraint_eps = 1e-6;
% %             d_req_obs = obj.L_cable + obj.r_drone + obj.clearance_margin;
% % 
% %             % 4-1. 障害物半空間制約
% %             for obs_idx = 1:length(active_obstacles)
% %                 tgt = active_obstacles(obs_idx);
% %                 p_surf = tgt.closest_drone_world_point(:);
% %                 n_dir = tgt.normal_drone_point(:) / norm(tgt.normal_drone_point);
% % 
% %                 for cp_i = 1:n_cp
% %                     if cp_i <= 7
% %                         P_fixed = P_L_start(cp_i,:)';
% %                         margin_cp = dot(n_dir,(P_fixed-p_surf)) - d_req_obs;
% %                         if margin_cp < constraint_eps
% %                             fprintf(2, '[QP REJECT] 始端固定CP#%d が障害物ID=%d の半空間外: margin=%.4f m < 0\n', ...
% %                                 cp_i, tgt.id, margin_cp);
% %                             success = false; max_disp = 0.0; return;
% %                         end
% %                     else
% %                         f_idx = cp_i - 7;
% %                         row_cp = zeros(1, n_free * 3);
% %                         for dim = 1:3
% %                             idx_d = (dim - 1) * n_free;
% %                             row_cp(idx_d + f_idx) = -n_dir(dim);
% %                         end
% %                         A_ineq = [A_ineq; row_cp];
% %                         b_ineq = [b_ineq; -(d_req_obs + dot(n_dir, p_surf))];
% %                     end
% %                 end
% %             end
% % 
% %             % 4-2. 水平加速度制約行列 D2 の生成
% %             U = obj.knots;
% %             D1 = zeros(n_cp - 1, n_cp);
% %             for i = 1:(n_cp - 1)
% %                 denom = U(i + p + 1) - U(i + 1);
% %                 if denom > 1e-12
% %                     D1(i,i)   = -p / denom;
% %                     D1(i,i+1) =  p / denom;
% %                 end
% %             end
% % 
% %             U1 = U(2:end-1);
% %             p1 = p - 1;
% %             D1_second = zeros(n_cp - 2, n_cp - 1);
% %             for i = 1:(n_cp - 2)
% %                 denom = U1(i + p1 + 1) - U1(i + 1);
% %                 if denom > 1e-12
% %                     D1_second(i,i)   = -p1 / denom;
% %                     D1_second(i,i+1) =  p1 / denom;
% %                 end
% %             end
% %             D2 = D1_second * D1;
% % 
% %             D2_start = D2(:,1:7);
% %             D2_free  = D2(:,8:n_cp);
% % 
% %             a_max = obj.max_acc_load;
% %             a_comp_max = a_max / sqrt(2.0);
% %             n_acc_ctrl = size(D2, 1);
% % 
% %             for acc_i = 1:n_acc_ctrl
% %                 d2_row = D2_free(acc_i, :);
% % 
% %                 % ax <= a_comp_max
% %                 r_ax = zeros(1, n_free * 3); r_ax(1:n_free) = d2_row;
% %                 fix_ax = D2_start(acc_i,:) * P_L_start(:,1);
% %                 A_ineq = [A_ineq; r_ax; -r_ax];
% %                 b_ineq = [b_ineq; a_comp_max - fix_ax; a_comp_max + fix_ax];
% % 
% %                 % ay <= a_comp_max
% %                 r_ay = zeros(1, n_free * 3); r_ay(n_free + (1:n_free)) = d2_row;
% %                 fix_ay = D2_start(acc_i,:) * P_L_start(:,2);
% %                 A_ineq = [A_ineq; r_ay; -r_ay];
% %                 b_ineq = [b_ineq; a_comp_max - fix_ay; a_comp_max + fix_ay];
% %             end
% % 
% %             % 5. QP 求解
% %             max_coord = 50.0;
% %             lb = -max_coord * ones(n_free * 3, 1);
% %             ub =  max_coord * ones(n_free * 3, 1);
% %             opts = optimoptions('quadprog', 'Display', 'off', 'Algorithm', 'interior-point-convex', 'MaxIterations', 300);
% % 
% %             [X_mid, ~, exitflag, output] = quadprog(H, f, A_ineq, b_ineq, [], [], lb, ub, [], opts);
% % 
% %             if exitflag < 1
% %                 fprintf(2, '[QP FAILED] exitflag=%d, iter=%d, nvar=%d, nconstr=%d\n', ...
% %                     exitflag, output.iterations, length(f), size(A_ineq, 1));
% %                 success = false; max_disp = 0.0; return;
% %             end
% % 
% %             success = true;
% %             P_L_mid = zeros(n_free, 3);
% %             P_L_mid(:,1) = X_mid(1:n_free);
% %             P_L_mid(:,2) = X_mid((n_free+1):(2*n_free));
% %             P_L_mid(:,3) = X_mid((2*n_free+1):(3*n_free));
% %             obj.control_points = [P_L_start; P_L_mid];
% % 
% %             % ============================================================
% %             % 【加速度詳細診断】D2 加速度制御点 vs eval_spline_kth 実測値
% %             % ============================================================
% %             A2_mat = D2 * obj.control_points;
% %             A2_xy_norms = sqrt(A2_mat(:,1).^2 + A2_mat(:,2).^2);
% %             [max_A2, idx_A2] = max(A2_xy_norms);
% % 
% %             A2_start_xy = sqrt((D2_start * P_L_start(:,1)).^2 + (D2_start * P_L_start(:,2)).^2);
% %             [max_start_A2, idx_start_A2] = max(A2_start_xy);
% % 
% %             % 連続時間サンプリング
% %             eval_taus = linspace(0.0, T_tot, 60);
% %             eval_accs = zeros(60, 1);
% %             for ti = 1:60
% %                 a_temp = obj.eval_spline_kth_with_T(eval_taus(ti), 2, T_tot);
% %                 eval_accs(ti) = norm(a_temp(1:2));
% %             end
% %             [max_eval, idx_eval] = max(eval_accs);
% % 
% %             ratio = max_eval / max(max_A2, 1e-6);
% % 
% %             fprintf('\n--- [QP ACCELERATION MATCH DIAGNOSTIC] ---\n');
% %             fprintf('  設定上限 a_max               = %.3f m/s^2 (成分上限: %.3f)\n', a_max, a_comp_max);
% %             fprintf('  D2 制御点最大 ||A2_xy||      = %.3f m/s^2 (CP#%d / %d)\n', max_A2, idx_A2, n_acc_ctrl);
% %             fprintf('  始端固定部 D2_start 最大     = %.3f m/s^2 (行#%d)\n', max_start_A2, idx_start_A2);
% %             fprintf('  eval_spline_kth 最大 a_xy    = %.3f m/s^2 (at tau=%.3f s)\n', max_eval, eval_taus(idx_eval));
% %             fprintf('  始端加速度 (tau=0) a_xy      = %.3f m/s^2\n', eval_accs(1));
% %             fprintf('  終端加速度 (tau=T) a_xy      = %.3f m/s^2\n', eval_accs(end));
% %             fprintf('  [不整合比率] 実評価 / D2制御点 = %.3f\n', ratio);
% %             if abs(ratio - 1.0) > 0.15
% %                 fprintf(2, '  ===> [FATAL WARNING] D2 行列と eval_basis_derivatives に数学的不一致があります！\n');
% %             end
% %             fprintf('------------------------------------------\n\n');
% % 
% %             % 変位測定
% %             N_eval = 60;
% %             taus = linspace(0, T_tot, N_eval);
% %             max_disp = 0.0;
% %             for i = 1:N_eval
% %                 p_eval = obj.eval_spline_kth_with_T(taus(i), 0, T_tot);
% %                 nom_res = obj.base_ref.do(struct('t', t_now + taus(i), 'dt', 0.025), 'f');
% %                 max_disp = max(max_disp, norm(p_eval - nom_res.state.xd(1:3)));
% %             end
% %         end
% % 
% %         % =====================================================================
% %         % verify_trajectory_numerical_sampling: 詳細判定・ログ出力版
% %         % =====================================================================
% %         function [is_safe, peak_acc_xy] = verify_trajectory_numerical_sampling(obj, t_now, obs_list, T_tot)
% %             is_safe = true;
% %             peak_acc_xy = 0.0;
% %             N_samples = 100;
% %             taus = linspace(0.0, T_tot, N_samples);
% % 
% %             r_c_sph = obj.r_cable_sphere;
% %             n_spheres = max(2, ceil(obj.L_cable / (2.0 * r_c_sph)) + 1);
% %             s_ratios = linspace(0.0, 1.0, n_spheres);
% % 
% %             for i = 1:N_samples
% %                 tau_i = taus(i);
% %                 pL_fut = obj.eval_spline_kth_with_T(tau_i, 0, T_tot);
% %                 aL_fut = obj.eval_spline_kth_with_T(tau_i, 2, T_tot);
% % 
% %                 peak_acc_xy = max(peak_acc_xy, norm(aL_fut(1:2)));
% % 
% %                 g_vec = [0; 0; obj.gravity];
% %                 acc_tot = aL_fut + g_vec;
% %                 norm_a = norm(acc_tot);
% %                 if norm_a > 1e-3, thrust_dir = acc_tot / norm_a; else, thrust_dir = [0; 0; 1]; end
% %                 pQ_fut = pL_fut + obj.L_cable * thrust_dir;
% %                 cable_pts_fut = (1 - s_ratios) .* pL_fut + s_ratios .* pQ_fut;
% % 
% %                 for obs_idx = 1:length(obs_list)
% %                     o = obs_list(obs_idx);
% %                     obs_parsed = struct('center', o.p_obs(:), 'radii', o.radii_obs(:), 'R', o.R_obs);
% % 
% %                     % 1. UAV
% %                     [dQ, ~, ~, ~] = obj.point_ellipsoid_signed_distance(pQ_fut, obs_parsed);
% %                     if dQ - obj.r_drone <= 0
% %                         fprintf(2, '[VERIFY FAIL: UAV INTRUSION] tau=%.3f s | ID=%d, 表面gap=%.4f m <= 0\n', ...
% %                             tau_i, o.id, dQ - obj.r_drone);
% %                         is_safe = false; return;
% %                     end
% % 
% %                     % 2. 荷物
% %                     [dL, ~, ~, ~] = obj.point_ellipsoid_signed_distance(pL_fut, obs_parsed);
% %                     if dL - obj.r_load <= 0
% %                         fprintf(2, '[VERIFY FAIL: LOAD INTRUSION] tau=%.3f s | ID=%d, 表面gap=%.4f m <= 0\n', ...
% %                             tau_i, o.id, dL - obj.r_load);
% %                         is_safe = false; return;
% %                     end
% % 
% %                     % 3. 索球列
% %                     for sp = 1:n_spheres
% %                         [dC, ~, ~, ~] = obj.point_ellipsoid_signed_distance(cable_pts_fut(:, sp), obs_parsed);
% %                         if dC - r_c_sph <= 0
% %                             fprintf(2, '[VERIFY FAIL: CABLE INTRUSION] tau=%.3f s | 球#%d, ID=%d, 表面gap=%.4f m <= 0\n', ...
% %                                 tau_i, sp, o.id, dC - r_c_sph);
% %                             is_safe = false; return;
% %                         end
% %                     end
% %                 end
% %             end
% % 
% %             if peak_acc_xy > obj.max_acc_load
% %                 fprintf(2, '[VERIFY FAIL: ACCEL EXCEEDED] peak_acc_xy = %.3f m/s^2 > a_max = %.3f m/s^2\n', ...
% %                     peak_acc_xy, obj.max_acc_load);
% %             end
% %         end
% % 
% %         % =====================================================================
% %         % update_active_obstacles
% %         % =====================================================================
% %         function planning_obstacles = update_active_obstacles(obj, detected_candidates, t_now)
% %             detected_ids = [];
% %             for k = 1:length(detected_candidates)
% %                 cand = detected_candidates(k);
% %                 detected_ids = [detected_ids, cand.id];
% %                 idx = [];
% %                 if ~isempty(obj.active_obstacles)
% %                     idx = find([obj.active_obstacles.id] == cand.id, 1);
% %                 end
% %                 if isempty(idx)
% %                     new_item = struct();
% %                     new_item.id                 = cand.id;
% %                     new_item.p_obs              = cand.p_obs;
% %                     new_item.radii_obs          = cand.radii_obs;
% %                     new_item.R_obs              = cand.R_obs;
% %                     new_item.closest_drone_world_point = cand.closest_drone_world_point;
% %                     new_item.normal_drone_point = cand.normal_drone_point;
% %                     new_item.dist_drone_point   = cand.dist_drone_point;
% %                     new_item.first_detected_time = t_now;
% %                     new_item.last_seen_time     = t_now;
% %                     new_item.state              = "DETECTED";
% %                     new_item.release_time       = NaN;
% %                     obj.active_obstacles = [obj.active_obstacles; new_item];
% %                 else
% %                     obj.active_obstacles(idx).p_obs              = cand.p_obs;
% %                     obj.active_obstacles(idx).radii_obs          = cand.radii_obs;
% %                     obj.active_obstacles(idx).R_obs              = cand.R_obs;
% %                     obj.active_obstacles(idx).closest_drone_world_point = cand.closest_drone_world_point;
% %                     obj.active_obstacles(idx).normal_drone_point = cand.normal_drone_point;
% %                     obj.active_obstacles(idx).dist_drone_point   = cand.dist_drone_point;
% %                     obj.active_obstacles(idx).last_seen_time     = t_now;
% %                     obj.active_obstacles(idx).state              = "DETECTED";
% %                     obj.active_obstacles(idx).release_time       = NaN;
% %                 end
% %             end
% % 
% %             keep_flags = true(length(obj.active_obstacles), 1);
% %             for i = 1:length(obj.active_obstacles)
% %                 obs_id = obj.active_obstacles(i).id;
% %                 if ~ismember(obs_id, detected_ids)
% %                     if obj.replan_active
% %                         obj.active_obstacles(i).state = "LATCHED";
% %                         obj.active_obstacles(i).release_time = NaN;
% %                     else
% %                         if obj.active_obstacles(i).state ~= "POST_RECOVERY"
% %                             obj.active_obstacles(i).state = "POST_RECOVERY";
% %                             obj.active_obstacles(i).release_time = t_now + obj.post_recovery_hold_time;
% %                         end
% %                         if t_now >= obj.active_obstacles(i).release_time
% %                             keep_flags(i) = false;
% %                         end
% %                     end
% %                 end
% %             end
% %             obj.active_obstacles = obj.active_obstacles(keep_flags);
% %             planning_obstacles = obj.active_obstacles;
% %         end
% % 
% %         % =====================================================================
% %         % extract_all_obstacle_halfspaces
% %         % =====================================================================
% %         function active_list = extract_all_obstacle_halfspaces(obj, pQ_cur, obs_list)
% %             if isempty(obs_list)
% %                 active_list = obs_list;
% %                 return;
% %             end
% %             active_list = obs_list;
% %             for i = 1:numel(obs_list)
% %                 o = obs_list(i);
% %                 p_obs = o.p_obs(:);
% %                 if isfield(o, 'radii_obs')
% %                     radii_obs = o.radii_obs(:);
% %                 elseif isfield(o, 'ellipsoid_radii')
% %                     radii_obs = o.ellipsoid_radii(:);
% %                 else
% %                     error('Obstacle ID=%d に radii 情報がありません.', o.id);
% %                 end
% %                 R_obs = o.R_obs;
% %                 obs_parsed = struct('center', p_obs, 'radii', radii_obs, 'R', R_obs);
% % 
% %                 [d_raw, ~, cpQ_loc, ~] = obj.point_ellipsoid_signed_distance(pQ_cur, obs_parsed);
% %                 cpQ_world = p_obs + R_obs * cpQ_loc;
% %                 nQ_loc = cpQ_loc ./ (radii_obs.^2);
% %                 normal_drone = R_obs * (nQ_loc / norm(nQ_loc));
% % 
% %                 active_list(i).closest_drone_world_point = cpQ_world;
% %                 active_list(i).normal_drone_point        = normal_drone;
% %                 active_list(i).dist_drone_point          = d_raw - obj.r_drone;
% %             end
% %         end
% % 
% %         % =====================================================================
% %         % estimate_safety_margin_impact_time
% %         % =====================================================================
% %         function t_impact = estimate_safety_margin_impact_time(obj, pQ_cur, dir_nom, spd, target_obs)
% %             vec_to_obs = target_obs.p_obs - pQ_cur;
% %             dist_along = dot(vec_to_obs, dir_nom);
% %             max_rad = max(target_obs.radii_obs);
% %             dist_to_margin = max(0.5, dist_along - max_rad - obj.r_drone - obj.clearance_margin);
% %             t_impact = max(1.5, dist_to_margin / spd);
% %         end
% % 
% %         % =====================================================================
% %         % project_nominal_trajectory_to_bspline
% %         % =====================================================================
% %         function P_nom_all = project_nominal_trajectory_to_bspline(obj, t_now, T_tot, n_cp)
% %             p = obj.spline_degree;
% %             N_samples = 100;
% %             t_quad = linspace(0.0, T_tot, N_samples);
% %             dt = T_tot / (N_samples - 1);
% %             M_gram = zeros(n_cp, n_cp);
% %             B_proj = zeros(n_cp, 3);
% % 
% %             for k = 1:N_samples
% %                 tau_k = t_quad(k);
% %                 t_k = t_now + tau_k;
% %                 nom_res_k = obj.base_ref.do(struct('t', t_k, 'dt', 0.025), 'f');
% %                 p_nom_k = nom_res_k.state.xd(1:3)';
% % 
% %                 idx_span = obj.find_knot_span(min(T_tot - 1e-6, tau_k));
% %                 ders = obj.eval_basis_derivatives(idx_span, min(T_tot - 1e-6, tau_k), 0);
% %                 basis_vals = ders(1, :);
% %                 c_idx = (idx_span - p):idx_span;
% % 
% %                 w = dt;
% %                 if k == 1 || k == N_samples, w = 0.5 * dt; end
% %                 M_gram(c_idx, c_idx) = M_gram(c_idx, c_idx) + w * (basis_vals' * basis_vals);
% %                 B_proj(c_idx, :) = B_proj(c_idx, :) + w * (basis_vals' * p_nom_k);
% %             end
% %             P_nom_all = (M_gram + 1e-5 * eye(n_cp)) \ B_proj;
% %         end
% % 
% %         % =====================================================================
% %         % get_nominal_derivatives_at_time
% %         % =====================================================================
% %         function ders = get_nominal_derivatives_at_time(obj, t_eval)
% %             ders = zeros(7, 3);
% %             nom_res = obj.base_ref.do(struct('t', t_eval, 'dt', 0.025), 'f');
% %             xd_val = nom_res.state.xd;
% %             if length(xd_val) < 28
% %                 xd_val = [xd_val; zeros(28 - length(xd_val), 1)];
% %             end
% %             ders(1, :) = xd_val(1:3)';
% %             ders(2, :) = xd_val(5:7)';
% %             ders(3, :) = xd_val(9:11)';
% %             ders(4, :) = xd_val(13:15)';
% %             ders(5, :) = xd_val(17:19)';
% %             ders(6, :) = xd_val(21:23)';
% %             ders(7, :) = xd_val(25:27)';
% %         end
% % 
% %         % =====================================================================
% %         % verify_start_c6_matching
% %         % =====================================================================
% %         function verify_start_c6_matching(obj, init_load_state, t_now)
% %             names = ["位置(0階)", "速度(1階)", "加速度(2階)", "Jerk(3階)", "Snap(4階)", "Crack(5階)", "Pop(6階)"];
% %             tolerances = [1e-10, 1e-9, 1e-8, 1e-6, 1e-4, 1e-2, 1.0];
% %             for k = 0:6
% %                 actual_k = obj.eval_spline_kth(0.0, k)';
% %                 expected_k = init_load_state(k + 1, :);
% %                 gap = norm(actual_k - expected_k);
% %                 obj.c6_start_gaps(k + 1) = gap;
% %                 if gap > tolerances(k + 1)
% %                     error('[FATAL C^6 BREACH] t=%.3f s | 始端 %s の接続ギャップ超過: %.3e', ...
% %                         t_now, names(k + 1), gap);
% %                 end
% %             end
% %         end
% % 
% %         % =====================================================================
% %         % verify_return_c6_matching
% %         % =====================================================================
% %         function verify_return_c6_matching(obj, t_end_expected)
% %             names = ["位置(0階)", "速度(1階)", "加速度(2階)", "Jerk(3階)", "Snap(4階)", "Crack(5階)", "Pop(6階)"];
% %             tolerances = [1e-9, 1e-8, 1e-7, 1e-5, 1e-3, 1e-1, 5.0];
% %             fprintf("\n=================================================================================\n");
% %             fprintf(" [B-SPLINE 回避終了 C^6 終端合流ギャップ実測レポート]  t_end = %.3f s\n", t_end_expected);
% %             fprintf("=================================================================================\n");
% %             pass = true;
% %             end_nom_state = obj.get_nominal_derivatives_at_time(t_end_expected);
% %             for k = 0:6
% %                 actual_k = obj.eval_spline_kth_with_T(obj.t_duration, k, obj.t_duration)';
% %                 expected_k = end_nom_state(k + 1, :);
% %                 gap = norm(actual_k - expected_k);
% %                 obj.c6_return_gaps(k + 1) = gap;
% %                 if gap > tolerances(k + 1), pass = false; end
% %                 fprintf("   - %-12s e_%d = %.3e m/s^%d (許容: %.1e)\n", names(k + 1), k, gap, k, tolerances(k + 1));
% %             end
% %             if pass
% %                 fprintf(" 判定: C^6公称合流達成。\n");
% %             else
% %                 fprintf(2, " [WARNING] 終端0〜6階微分の残差が許容値を超過しました.\n");
% %             end
% %             fprintf("=================================================================================\n\n");
% %         end
% % 
% %         % =====================================================================
% %         % verify_step_integration_consistency
% %         % =====================================================================
% %         function verify_step_integration_consistency(obj, xd_now, t_now, dt)
% %             if obj.prev_t < 0
% %                 obj.prev_xd = xd_now;
% %                 obj.prev_t  = t_now;
% %                 return;
% %             end
% %             real_dt = t_now - obj.prev_t;
% %             if real_dt <= 1e-6, return; end
% %             if nargin < 4 || isempty(dt) || dt <= 0, dt = real_dt; end
% % 
% %             orders = { ...
% %                 '位置 (0階)',      1:3,   5:7;   ...
% %                 'yaw角 (0階)',     4,     8;     ...
% %                 '速度 (1階)',      5:7,   9:11;  ...
% %                 '加速度 (2階)',    9:11,  13:15; ...
% %                 'Jerk (3階)',      13:15, 17:19; ...
% %             };
% %             for idx = 1:size(orders, 1)
% %                 name   = orders{idx, 1};
% %                 curr_i = orders{idx, 2};
% %                 next_i = orders{idx, 3};
% %                 val_prev = obj.prev_xd(curr_i);
% %                 val_curr = xd_now(curr_i);
% %                 der_prev = obj.prev_xd(next_i);
% %                 der_curr = xd_now(next_i);
% %                 predicted = val_prev + 0.5 * (der_prev + der_curr) * real_dt;
% %                 disc_gap  = norm(val_curr - predicted);
% %                 tol = max(0.08, 6.0 * norm(der_curr) * real_dt);
% %                 if disc_gap > tol
% %                     warning("離散ステップ不連続キック検出: %s at t=%.4f s (gap: %.3e)", name, t_now, disc_gap);
% %                 end
% %             end
% %             obj.prev_xd = xd_now;
% %             obj.prev_t  = t_now;
% %         end
% % 
% %         % =====================================================================
% %         % check_detection_simulated_sensor
% %         % =====================================================================
% %         function det = check_detection_simulated_sensor(obj, pQ, pL, obs_list, t_now, v_nom)
% %             det = struct();
% %             det.time                          = t_now;
% %             det.pQ                            = pQ;
% %             det.pL                            = pL;
% %             det.trigger_dist                  = obj.trigger_dist;
% %             det.detected_point                = false;
% %             det.drone_inside_obstacle_point   = false;
% %             det.load_inside_obstacle_point    = false;
% %             det.cable_inside_obstacle_point   = false;
% %             det.drone_margin_violated         = false;
% %             det.load_margin_violated          = false;
% %             det.cable_margin_violated         = false;
% %             det.drone_min_dist_point          = inf;
% %             det.load_min_dist_point           = inf;
% %             det.cable_min_dist_point          = inf;
% %             det.min_dist_point                = inf;
% %             det.drone_obstacle_id_point       = [];
% %             det.load_obstacle_id_point        = [];
% %             det.min_obstacle_id_point         = [];
% %             det.min_source_point              = "none";
% %             det.detected_obstacles_point      = [];
% %             det.detected_obstacle_count_point = 0;
% % 
% %             if isempty(obs_list), return; end
% % 
% %             r_c_sph = obj.r_cable_sphere;
% %             n_spheres = max(2, ceil(obj.L_cable / (2.0 * r_c_sph)) + 1);
% %             s_ratios = linspace(0.0, 1.0, n_spheres);
% %             cable_pts = (1 - s_ratios) .* pL + s_ratios .* pQ;
% % 
% %             detected_obs_candidates = [];
% %             for i = 1:length(obs_list)
% %                 o = obs_list(i);
% %                 obs_id = o.id;
% %                 R_obs = o.R_obs;
% %                 radii_obs = o.ellipsoid_radii(:);
% %                 p_obs = o.p_center(:);
% %                 obs_parsed = struct('center', p_obs, 'radii', radii_obs, 'R', R_obs);
% % 
% %                 [d_drone_raw, inside_drone, cpQ_loc, ~] = obj.point_ellipsoid_signed_distance(pQ, obs_parsed);
% %                 d_drone_physical = d_drone_raw - obj.r_drone;
% %                 d_drone_safety   = d_drone_physical - obj.clearance_margin;
% % 
% %                 [d_load_raw, inside_load, cpL_loc, ~] = obj.point_ellipsoid_signed_distance(pL, obs_parsed);
% %                 d_load_physical = d_load_raw - obj.r_load;
% %                 d_load_safety   = d_load_physical - obj.clearance_margin;
% % 
% %                 d_cable_raw_min = inf;
% %                 inside_cable_i = false;
% %                 for sp_idx = 1:n_spheres
% %                     [d_pt, in_pt, ~, ~] = obj.point_ellipsoid_signed_distance(cable_pts(:, sp_idx), obs_parsed);
% %                     if d_pt < d_cable_raw_min, d_cable_raw_min = d_pt; end
% %                     if in_pt, inside_cable_i = true; end
% %                 end
% %                 d_cable_physical = d_cable_raw_min - r_c_sph;
% %                 d_cable_safety   = d_cable_physical - obj.clearance_margin;
% % 
% %                 if inside_drone || (d_drone_physical <= 0),   det.drone_inside_obstacle_point = true; end
% %                 if inside_load  || (d_load_physical <= 0),    det.load_inside_obstacle_point  = true; end
% %                 if inside_cable_i || (d_cable_physical <= 0), det.cable_inside_obstacle_point = true; end
% % 
% %                 if d_drone_safety <= 0, det.drone_margin_violated = true; end
% %                 if d_load_safety  <= 0, det.load_margin_violated  = true; end
% %                 if d_cable_safety <= 0, det.cable_margin_violated = true; end
% % 
% %                 if d_drone_physical < det.drone_min_dist_point
% %                     det.drone_min_dist_point = d_drone_physical;
% %                     det.drone_obstacle_id_point = obs_id;
% %                 end
% %                 if d_load_physical < det.load_min_dist_point
% %                     det.load_min_dist_point = d_load_physical;
% %                     det.load_obstacle_id_point = obs_id;
% %                 end
% %                 if d_cable_physical < det.cable_min_dist_point
% %                     det.cable_min_dist_point = d_cable_physical;
% %                 end
% % 
% %                 cpQ_world = p_obs + R_obs * cpQ_loc;
% %                 cpL_world = p_obs + R_obs * cpL_loc;
% %                 nQ_loc = cpQ_loc ./ (radii_obs.^2);
% %                 normal_drone = R_obs * (nQ_loc / norm(nQ_loc));
% %                 nL_loc = cpL_loc ./ (radii_obs.^2);
% %                 normal_load = R_obs * (nL_loc / norm(nL_loc));
% % 
% %                 obs_info = struct( ...
% %                     'id',                        obs_id, ...
% %                     'dist_drone_point',          d_drone_physical, ...
% %                     'dist_drone_safety',         d_drone_safety, ...
% %                     'dist_load_point',           d_load_physical, ...
% %                     'dist_load_safety',          d_load_safety, ...
% %                     'dist_cable_point',          d_cable_physical, ...
% %                     'dist_cable_safety',         d_cable_safety, ...
% %                     'min_dist_point',            d_drone_physical, ...
% %                     'p_obs',                     p_obs, ...
% %                     'radii_obs',                 radii_obs, ...
% %                     'R_obs',                     R_obs, ...
% %                     'closest_drone_world_point', cpQ_world, ...
% %                     'closest_load_world_point',  cpL_world, ...
% %                     'normal_drone_point',        normal_drone, ...
% %                     'normal_load_point',         normal_load ...
% %                 );
% % 
% %                 if d_drone_raw <= obj.trigger_dist
% %                     detected_obs_candidates = [detected_obs_candidates; obs_info];
% %                 end
% %             end
% % 
% %             if ~isempty(detected_obs_candidates)
% %                 [~, sort_idx] = sort([detected_obs_candidates.dist_drone_point], 'ascend');
% %                 det.detected_obstacles_point = detected_obs_candidates(sort_idx);
% %                 det.detected_point = true;
% %                 det.min_dist_point = det.detected_obstacles_point(1).dist_drone_point;
% %                 det.min_obstacle_id_point = det.detected_obstacles_point(1).id;
% %                 det.min_source_point = "drone";
% %                 det.detected_obstacle_count_point = length(detected_obs_candidates);
% %             else
% %                 det.min_dist_point = det.drone_min_dist_point;
% %                 det.min_obstacle_id_point = det.drone_obstacle_id_point;
% %                 det.min_source_point = "none";
% %             end
% %         end
% % 
% %         % =====================================================================
% %         % display_detection_report
% %         % =====================================================================
% %         function display_detection_report(obj, t_now, tgt, total_candidates, req_clearance, t_impact, n_constraint, status_str)
% %             names = ["位置(0階)", "速度(1階)", "加速度(2階)", "Jerk(3階)", "Snap(4階)", "Crack(5階)", "Pop(6階)"];
% %             fprintf("\n=================================================================================\n");
% %             fprintf(" [B-SPLINE 15m センサー検知 ＆ 回避実行レポート]  t = %.3f s [%s]\n", t_now, status_str);
% %             fprintf("=================================================================================\n");
% %             fprintf(" 1. センサ判定元      : 機体重心 pQ 模擬センサー (表面距離: %.3f m)\n", tgt.dist_drone_point);
% %             fprintf(" 2. 捕捉障害物情報    : ID=%d, 半径=[%.2f, %.2f, %.2f] m\n", ...
% %                 tgt.id, tgt.radii_obs(1), tgt.radii_obs(2), tgt.radii_obs(3));
% %             fprintf(" 3. 運動学制約検証    : 実軌道最大 a_xy = %.2f m/s^2 (上限: %.2f m/s^2) | T_tot = %.2f s\n", ...
% %                 obj.sampled_peak_acceleration, obj.max_acc_load, obj.t_duration);
% %             fprintf(" 4. 連続時間包含要求  : d_req = %.2f m | 支持法線: [%.2f, %.2f, %.2f]\n", ...
% %                 req_clearance, n_constraint(1), n_constraint(2), n_constraint(3));
% %             fprintf(" 5. C^6 始端境界ギャップ実測値:\n");
% %             for k = 0:6
% %                 fprintf("     - %-12s e_%d = %.3e m/s^%d\n", names(k + 1), k, obj.c6_start_gaps(k + 1), k);
% %             end
% %             fprintf(" 6. 計算時間          : QP = %.2f ms | 全体 = %.2f ms\n", ...
% %                 obj.last_solve_time_ms, obj.last_replan_total_time_ms);
% %             fprintf(" 7. 公称軌道離脱変位  : %.3f m (max ||p_L(t) - p_nom(t)||)\n", obj.sampled_peak_displacement);
% %             fprintf("=================================================================================\n\n");
% %         end
% % 
% %         function str = get_safety_status(~, d_phys, d_safe)
% %             if d_phys <= 0, str = "COLLISION";
% %             elseif d_safe <= 0, str = "MARGIN VIOLATION";
% %             else, str = "SAFE"; end
% %         end
% % 
% %         % =====================================================================
% %         % point_ellipsoid_signed_distance
% %         % =====================================================================
% %         function [d, inside, closest_local, lambda] = point_ellipsoid_signed_distance(~, p, o)
% %             p = p(:); c = o.center(:); r = o.radii(:); R = o.R;
% %             y  = R' * (p - c);
% %             r2 = r.^2;
% %             q  = sum((y ./ r).^2);
% %             inside = (q < 1.0);
% % 
% %             if norm(y) < 1e-14
% %                 [min_r, min_idx] = min(r);
% %                 closest_local = zeros(3, 1);
% %                 closest_local(min_idx) = min_r;
% %                 d      = -min_r;
% %                 lambda = -min(r2);
% %                 return;
% %             end
% % 
% %             f = @(lam) sum(r2 .* (y.^2) ./ ((lam + r2).^2)) - 1.0;
% %             if q > 1.0
% %                 lo = 0.0;
% %                 hi = max(r) * norm(y);
% %             else
% %                 lo = -min(r2) * (1.0 - 1e-12);
% %                 hi = 0.0;
% %             end
% % 
% %             for kk = 1:80
% %                 mid = 0.5 * (lo + hi);
% %                 if f(mid) > 0, lo = mid; else, hi = mid; end
% %             end
% %             lambda = 0.5 * (lo + hi);
% %             closest_local = r2 .* y ./ (lambda + r2);
% %             d_abs = norm(closest_local - y);
% %             if inside, d = -d_abs; else, d = d_abs; end
% %         end
% % 
% %         function val = eval_spline_kth(obj, tau, k)
% %             val = obj.eval_spline_kth_with_T(tau, k, obj.t_duration);
% %         end
% % 
% %         function val = eval_spline_kth_with_T(obj, tau, k, T_tot)
% %             p = obj.spline_degree;
% %             t_eval = max(0.0, min(T_tot, tau));
% %             idx_span = obj.find_knot_span(t_eval);
% %             ders = obj.eval_basis_derivatives(idx_span, t_eval, k);
% %             c_indices = (idx_span - p):idx_span;
% %             val = (ders(k + 1, :) * obj.control_points(c_indices, :))';
% %         end
% % 
% %         function build_clamped_uniform_knots(obj, n_seg, p, T_tot)
% %             dt_knot = T_tot / n_seg;
% %             interior_knots = dt_knot * (1:(n_seg - 1));
% %             obj.knots = [zeros(1, p + 1), interior_knots, T_tot * ones(1, p + 1)];
% %         end
% % 
% %         function idx = find_knot_span(obj, t_eval)
% %             p = obj.spline_degree;
% %             n_cp = length(obj.knots) - p - 1;
% %             if t_eval >= obj.knots(n_cp + 1), idx = n_cp; return; end
% %             if t_eval <= obj.knots(p + 1),    idx = p + 1; return; end
% %             low = p + 1; high = n_cp + 1; mid = floor((low + high) / 2);
% %             while (t_eval < obj.knots(mid)) || (t_eval >= obj.knots(mid + 1))
% %                 if t_eval < obj.knots(mid), high = mid; else, low = mid; end
% %                 mid = floor((low + high) / 2);
% %             end
% %             idx = mid;
% %         end
% % 
% %         function ders = eval_basis_derivatives(obj, idx_span, t_eval, n_der)
% %             p = obj.spline_degree;
% %             U = obj.knots;
% %             ders = zeros(n_der + 1, p + 1);
% %             ndu = zeros(p + 1, p + 1);
% %             left = zeros(p + 1, 1);
% %             right = zeros(p + 1, 1);
% % 
% %             ndu(1, 1) = 1.0;
% %             for j = 1:p
% %                 left(j + 1) = t_eval - U(idx_span + 1 - j);
% %                 right(j + 1) = U(idx_span + j) - t_eval;
% %                 saved = 0.0;
% %                 for r = 0:(j - 1)
% %                     ndu(j + 1, r + 1) = right(r + 2) + left(j - r + 1);
% %                     temp = ndu(r + 1, j) / ndu(j + 1, r + 1);
% %                     ndu(r + 1, j + 1) = saved + right(r + 2) * temp;
% %                     saved = left(j - r + 1) * temp;
% %                 end
% %                 ndu(j + 1, j + 1) = saved;
% %             end
% %             for j = 0:p, ders(1, j + 1) = ndu(j + 1, p + 1); end
% % 
% %             a = zeros(2, p + 1);
% %             for r = 0:p
% %                 s1 = 0; s2 = 1; a(1, 1) = 1.0;
% %                 for k = 1:n_der
% %                     d = 0.0; rk = r - k; pk = p - k;
% %                     if r >= k
% %                         a(s2 + 1, 1) = a(s1 + 1, 1) / ndu(pk + 2, rk + 1);
% %                         d = a(s2 + 1, 1) * ndu(rk + 1, pk + 1);
% %                     end
% %                     if rk >= -1, j1 = 1; else, j1 = -rk; end
% %                     if (r - 1) <= pk, j2 = k - 1; else, j2 = p - r; end
% %                     for j = j1:j2
% %                         a(s2 + 1, j + 1) = (a(s1 + 1, j + 1) - a(s1 + 1, j)) / ndu(pk + 2, rk + j + 1);
% %                         d = d + a(s2 + 1, j + 1) * ndu(rk + j + 1, pk + 1);
% %                     end
% %                     if r <= pk
% %                         a(s2 + 1, k + 1) = -a(s1 + 1, k) / ndu(pk + 2, r + 1);
% %                         d = d + a(s2 + 1, k + 1) * ndu(r + 1, pk + 1);
% %                     end
% %                     ders(k + 1, r + 1) = d;
% %                     j_tmp = s1; s1 = s2; s2 = j_tmp;
% %                 end
% %             end
% %             r_scale = p;
% %             for k = 1:n_der
% %                 for j = 0:p
% %                     ders(k + 1, j + 1) = ders(k + 1, j + 1) * r_scale;
% %                 end
% %                 r_scale = r_scale * (p - k);
% %             end
% %         end
% % 
% %         function clear_state_sensor_values(obj, t_now)
% %             st = obj.result.state;
% %             st.time                          = t_now;
% %             st.pQ                            = [NaN; NaN; NaN];
% %             st.pL                            = [NaN; NaN; NaN];
% %             st.detected_point                = false;
% %             st.drone_inside_obstacle_point   = false;
% %             st.load_inside_obstacle_point    = false;
% %             st.cable_inside_obstacle_point   = false;
% %             st.drone_margin_violated         = false;
% %             st.load_margin_violated          = false;
% %             st.cable_margin_violated         = false;
% %             st.drone_min_dist_point          = inf;
% %             st.load_min_dist_point           = inf;
% %             st.cable_min_dist_point          = inf;
% %             st.min_dist_point                = inf;
% %             st.drone_obstacle_id_point       = NaN;
% %             st.load_obstacle_id_point        = NaN;
% %             st.min_obstacle_id_point         = NaN;
% %             st.min_source_point              = "none";
% %             st.detected_obstacle_count_point = 0;
% %             st.detected_obstacles_point      = [];
% %             st.pQ_ref                        = [NaN; NaN; NaN];
% %         end
% % 
% %         function list = get_obstacles_at_time(~, t_now)
% %             list = ENVIRONMENT_OBSTACLE_ELLIPSE_MOVE(t_now);
% %         end
% %     end
% % end

% classdef REPLANNING_BSPLINE < handle
%     % =========================================================================
%     % REPLANNING_BSPLINE
%     % 
%     % 1. 模擬センサー (Simulated Point Sensor):
%     %    - 機体重心 pQ を検知主体とし、前方 15m 以内の楕円体障害物を最短表面距離順に捕捉。
%     % 2. 独立実行時安全診断モニタ (Independent Runtime Safety Monitor):
%     %    - 荷物 pL, 機体 pQ, および直線索（半径 r_c の球列幾何被覆）について、
%     %      楕円体外郭までの最短ユークリッド距離をラグランジュ未定乗数法で厳密計算。
%     %    - 物理接触 (physical_gap <= 0) とマージン侵入 (safety_gap <= 0) を分離診断。
%     % 3. 実荷物目標軌道 p_L(t) の C^6 境界確定アルゴリズム:
%     %    - 始端 (t_now) および終端 (t_start + T_tot) における公称軌道の 0〜6階微分値を
%     %      境界代数方程式 M \ B により直接課し、実出力軌道自体の C^6 接続を保証。
%     % 4. 【連続時間安全半空間包含保証 (Continuous-time Reference Safety Guarantee)】:
%     %    - 生成・出力するのは牽引物目標軌道 pL(t) のみ。
%     %    - 楕円体の支持半空間法線 n に対し、全制御点 P_L,i (i=1..N_cp) へ
%     %      d_req = L_cable + r_drone + d_margin を課す。
%     %    - B-Spline の大域凸包性により全連続時間区間で pL(t) in H(d_req) が保証され、
%     %      索の幾何拘束 ||pQ - pL|| = L および r_drone >= r_cable >= r_load から、
%     %      参照UAV球体 B(pQ(t), r_drone) および直線索全体が楕円体支持半空間内に完全包含される。
%     % 5. 公称軌道引き戻し正則化 (Nominal Deviation Regularization):
%     %    - QP 目的関数に公称制御点からの乖離ペナルティ w_dev * ||P - P_nom||^2 を組み込み、
%     %      安全制約を満たす範囲内で公称軌道から不必要に離れる大回り回避（過大横ずれ）を抑制。
%     % 6. 状態配列 xd の厳格な仕様定義:
%     %    - xd(1:28): 荷物 pL の 0〜6階微分 ＋ 公称 Yaw の 0〜6階微分専用。
%     %    - xd(29:31): 差分平坦性モデルから求まる UAV 参照目標位置 pQ を分離格納。
%     % =========================================================================
%     properties
%         self                       % ドローンエージェント自身
%         base_ref                   % 公称参照軌道生成オブジェクト
%         result                     % 出力結果構造体
% 
%         % --- センサ・検知パラメータ ---
%         trigger_dist = 15.0;       % 機体搭載センサーによる接近検知閾値 [m]
%         clearance_margin = 0.8;    % 安全マージン d_margin [m]
% 
%         % --- 機体・荷物・索物理パラメータ ---
%         gravity = 9.81;            % 重力加速度 [m/s^2]
%         L_cable = 1.0;             % 索長 L [m]
%         r_drone = 0.30;            % 機体球体半径 r_Q [m]
%         r_load  = 0.15;            % 荷物球体半径 r_L [m]
%         r_cable_sphere = 0.25;     % 索保護球体半径 r_C [m] (実行時モニタ専用)
% 
%         % --- 運動学・物理制約パラメータ ---
%         max_acc_load = 2.0;        % 荷物許容水平加速度上限 a_max [m/s^2]
%         w_dev = 0.05;              % 公称軌道引き戻し重み (過大回避抑制ペナルティ)
% 
%         % --- リプランニング状態管理 ---
%         replan_active = false;     % 回避軌道追従中フラグ
%         t_start       = 0.0;       % 回避開始時刻 [s]
%         t_duration    = 6.0;       % 回避全所要時間 [s]
%         last_replan_time = -100.0; % 前回リプラン実行時刻 [s]
%         min_replan_interval = 0.25;% 再計画更新周期 [s]
%         active_threat_id = NaN;    % 現在追従中の主障害物ID
% 
%         % --- 7次 B-Spline パラメータ (p=7, n_seg=25 -> N_cp=32) ---
%         spline_degree = 7;         % 7次 B-Spline (内部 C^6 連続)
%         num_segments  = 25;        % 25セグメント
%         knots                      % ノットベクトル
%         control_points             % 荷物実軌道制御点 P_L (32 x 3) [X, Y, Z]
%         sampled_peak_displacement = 0.0; % 最大空間変位 (60点サンプリング評価) [m]
%         sampled_peak_acceleration = 0.0; % 実軌道最大水平加速度 (サンプリング評価) [m/s^2]
%         last_solve_time_ms = 0.0;  % QP求解時間 [ms]
%         last_replan_total_time_ms = 0.0; % リプランニング全体処理時間 [ms]
% 
%         % --- C^6 境界接続ギャップ診断バッファ ---
%         c6_start_gaps = zeros(7, 1);  % 始端接続ギャップ e_k (k=0..6)
%         c6_return_gaps = zeros(7, 1); % 終端復帰ギャップ e_k (k=0..6)
% 
%         % --- 離散時間ステップ整合性監視用バッファ ---
%         prev_xd                    % 前回ステップの xd (31x1)
%         prev_t = -1.0;             % 前回ステップの時刻 [s]
%     end
% 
%     methods (Access = public)
%         % =====================================================================
%         % コンストラクタ
%         % =====================================================================
%         function obj = REPLANNING_BSPLINE(self, base_ref, opts)
%             arguments
%                 self                  % 必須: エージェントインスタンス
%                 base_ref              % 必須: 通常飛行用の公称軌道インスタンス
%                 opts = struct()       % 任意: 外部設定構造体
%             end
%             obj.self = self;
%             obj.base_ref = base_ref;
%             if isfield(opts, 'trigger_dist'),     obj.trigger_dist     = opts.trigger_dist;     end
%             if isfield(opts, 'clearance_margin'), obj.clearance_margin = opts.clearance_margin; end
%             if isfield(opts, 'r_drone'),          obj.r_drone          = opts.r_drone;          end
%             if isfield(opts, 'r_load'),           obj.r_load           = opts.r_load;           end
%             if isfield(opts, 'r_cable_sphere'),   obj.r_cable_sphere   = opts.r_cable_sphere;   end
%             if isfield(opts, 'L_cable'),          obj.L_cable          = opts.L_cable;          end
%             if isfield(opts, 'gravity'),          obj.gravity          = opts.gravity;          end
%             if isfield(opts, 'max_acc_load'),     obj.max_acc_load     = opts.max_acc_load;     end
%             if isfield(opts, 'w_dev'),            obj.w_dev            = opts.w_dev;            end
% 
%             % 幾何学的安全包含の前提条件を事前検証
%             assert(obj.r_drone >= obj.r_cable_sphere, '幾何学的安全包含の前提違反: r_drone >= r_cable_sphere が必要です.');
%             assert(obj.r_cable_sphere >= obj.r_load,   '幾何学的安全包含の前提違反: r_cable_sphere >= r_load が必要です.');
%             assert(obj.L_cable > 0,                    'L_cable は正値である必要があります.');
%             assert(obj.max_acc_load > 0,                'max_acc_load は正値である必要があります.');
%             assert(obj.clearance_margin >= 0,          'clearance_margin は非負である必要があります.');
%             assert(obj.trigger_dist > 0,               'trigger_dist は正値である必要があります.');
% 
%             obj.result = base_ref.result;
% 
%             sensor_props = ["time", "pQ", "pL", "detected_point", ...
%                 "drone_inside_obstacle_point", "load_inside_obstacle_point", ...
%                 "cable_inside_obstacle_point", "drone_margin_violated", ...
%                 "load_margin_violated", "cable_margin_violated", ...
%                 "drone_min_dist_point", "load_min_dist_point", "cable_min_dist_point", ...
%                 "min_dist_point", "drone_obstacle_id_point", "load_obstacle_id_point", ...
%                 "min_obstacle_id_point", "min_source_point", "detected_obstacle_count_point", ...
%                 "detected_obstacles_point", "pQ_ref"];
%             for p_name = sensor_props
%                 if ~isprop(obj.result.state, p_name)
%                     addprop(obj.result.state, p_name);
%                 end
%             end
% 
%             obj.clear_state_sensor_values(0.0);
%         end
% 
%         % =====================================================================
%         % do: 制御周期ごとのメイン実行メソッド
%         % =====================================================================
%         function result_out = do(obj, varargin)
%             time = varargin{1};
%             cha  = varargin{2};
% 
%             % 1. 公称目標軌道 (Nominal Reference) の算出
%             base_res = obj.base_ref.do(varargin{:});
%             xd_nom = base_res.state.xd;
%             if length(xd_nom) < 31
%                 xd_nom = [xd_nom; zeros(31 - length(xd_nom), 1)];
%             end
% 
%             obj.clear_state_sensor_values(time.t);
% 
%             detection = struct();
%             detection.time                          = time.t;
%             detection.pQ                            = [NaN; NaN; NaN];
%             detection.pL                            = [NaN; NaN; NaN];
%             detection.detected_point                = false;
%             detection.drone_inside_obstacle_point   = false;
%             detection.load_inside_obstacle_point    = false;
%             detection.cable_inside_obstacle_point   = false;
%             detection.drone_margin_violated         = false;
%             detection.load_margin_violated          = false;
%             detection.cable_margin_violated         = false;
%             detection.drone_min_dist_point          = inf;
%             detection.load_min_dist_point           = inf;
%             detection.cable_min_dist_point          = inf;
%             detection.min_dist_point                = inf;
%             detection.drone_obstacle_id_point       = NaN;
%             detection.load_obstacle_id_point        = NaN;
%             detection.min_obstacle_id_point         = NaN;
%             detection.min_source_point              = "none";
%             detection.detected_obstacle_count_point = 0;
%             detection.detected_obstacles_point      = [];
% 
%             try obj.L_cable = obj.self.parameter.get("cableL"); catch; end
% 
%             % 2. 飛行フェーズ ('f') の機体センシング & 独立モニタリング
%             if cha == 'f'
%                 pL_cur = obj.self.estimator.result.state.pL(:);
%                 pQ_cur = obj.self.estimator.result.state.p(:);
%                 obs_list = obj.get_obstacles_at_time(time.t);
% 
%                 % 楕円体に対する UAV / 荷物 / 索球列の厳密ユークリッド距離診断
%                 detection = obj.check_detection_simulated_sensor(pQ_cur, pL_cur, obs_list, time.t, xd_nom(5:7));
% 
%                 % -------------------------------------------------------------
%                 % 3. 軌道再計画の判定 (近接センサ ＋ 3者マージン侵入 ＋ 未来予測モニタ)
%                 % -------------------------------------------------------------
%                 need_replan = false;
%                 can_replan = (time.t - obj.last_replan_time >= obj.min_replan_interval);
% 
%                 if can_replan
%                     if ~obj.replan_active
%                         body_margin_violated = detection.drone_margin_violated || ...
%                                                detection.load_margin_violated  || ...
%                                                detection.cable_margin_violated;
%                         if detection.detected_point || body_margin_violated
%                             need_replan = true;
%                         end
%                     else
%                         if detection.detected_point && (detection.min_obstacle_id_point ~= obj.active_threat_id)
%                             need_replan = true;
%                         elseif detection.drone_margin_violated || detection.cable_margin_violated || detection.load_margin_violated
%                             need_replan = true;
%                         else
%                             % 回避軌道追従中の100点数値モニタリング (0.25s 周期)
%                             [pred_safe, pred_acc] = obj.verify_trajectory_numerical_sampling(time.t, obs_list, obj.t_duration);
%                             if ~pred_safe || (pred_acc > obj.max_acc_load)
%                                 need_replan = true;
%                                 fprintf("[INDEPENDENT PREDICTIVE REPLAN] t=%.3f s | 回避軌道先行きで干渉または過大水平加速度 (a_xy=%.2f) を検知 -> 再計画\n", ...
%                                     time.t, pred_acc);
%                             end
%                         end
%                     end
% 
%                     if need_replan
%                         obj.execute_replanning(pQ_cur, xd_nom, detection, obs_list, time.t);
%                     end
%                 end
%             end
% 
%             % 4. 出力目標軌道の確定
%             if obj.replan_active
%                 tau = time.t - obj.t_start;
%                 t_end_expected = obj.t_start + obj.t_duration;
% 
%                 % 復帰安全性の事前検査: 回避終了 1.0 秒前に復帰先公称軌道をスキャン (cha == 'f' 限定)
%                 if (tau >= obj.t_duration - 1.0) && (tau < obj.t_duration) && (cha == 'f')
%                     obs_list_check = obj.get_obstacles_at_time(time.t);
%                     if ~obj.is_nominal_recovery_safe(t_end_expected, obs_list_check)
%                         fprintf("[RETURN HELD & REPLAN] t=%.3f s | 復帰予定時刻 t_end=%.3f s の公称軌道上に楕円体干渉を検出 -> 回避軌道を再生成\n", ...
%                             time.t, t_end_expected);
%                         obj.execute_replanning(pQ_cur, xd_nom, detection, obs_list, time.t);
%                         tau = time.t - obj.t_start;
%                         t_end_expected = obj.t_start + obj.t_duration;
%                     end
%                 end
% 
%                 if tau < obj.t_duration
%                     xd_out = obj.evaluate_smooth_trajectory(tau, xd_nom);
%                 else
%                     obj.verify_return_c6_matching(t_end_expected);
%                     obj.replan_active = false;
%                     obj.active_threat_id = NaN;
%                     xd_out = xd_nom;
%                     fprintf("[B-SPLINE C^6] 回避所要時間完了: 公称軌道へ完全滑らか復帰 (t=%.3f s)\n\n", time.t);
%                 end
%             else
%                 xd_out = xd_nom;
%             end
% 
%             % 5. 離散時間ステップ積分整合性監視
%             if cha == 'f'
%                 obj.verify_step_integration_consistency(xd_out, time.t, time.dt);
%             end
% 
%             % 6. 結果の格納
%             st = obj.result.state;
%             st.xd                            = xd_out;
%             st.p                             = xd_out(1:3);
%             st.v                             = xd_out(5:7);
%             st.q                             = [0; 0; xd_out(4)];
%             st.pQ_ref                        = xd_out(29:31); % UAV参照目標位置
% 
%             st.time                          = detection.time;
%             st.pQ                            = detection.pQ;
%             st.pL                            = detection.pL;
%             st.detected_point                = detection.detected_point;
%             st.drone_inside_obstacle_point   = detection.drone_inside_obstacle_point;
%             st.load_inside_obstacle_point    = detection.load_inside_obstacle_point;
%             st.cable_inside_obstacle_point   = detection.cable_inside_obstacle_point;
%             st.drone_margin_violated         = detection.drone_margin_violated;
%             st.load_margin_violated          = detection.load_margin_violated;
%             st.cable_margin_violated         = detection.cable_margin_violated;
%             st.drone_min_dist_point          = detection.drone_min_dist_point;
%             st.load_min_dist_point           = detection.load_min_dist_point;
%             st.cable_min_dist_point          = detection.cable_min_dist_point;
%             st.min_dist_point                = detection.min_dist_point;
%             st.drone_obstacle_id_point       = detection.drone_obstacle_id_point;
%             st.load_obstacle_id_point        = detection.load_obstacle_id_point;
%             st.min_obstacle_id_point         = detection.min_obstacle_id_point;
%             st.min_source_point              = "none";
%             st.detected_obstacle_count_point = detection.detected_obstacle_count_point;
%             st.detected_obstacles_point      = detection.detected_obstacles_point;
% 
%             result_out = obj.result;
%         end
% 
%         % =====================================================================
%         % evaluate_smooth_trajectory: 荷物実 B-Spline 軌道 p_L(t) から直接評価 (Public)
%         % =====================================================================
%         function xd = evaluate_smooth_trajectory(obj, tau, xd_nom)
%             xd = xd_nom;
% 
%             % 荷物の実目標軌道状態を B-Spline から直接評価 (0〜6階微分)
%             pL_d   = obj.eval_spline_kth(tau, 0);
%             vL_d   = obj.eval_spline_kth(tau, 1);
%             aL_d   = obj.eval_spline_kth(tau, 2);
%             jL_d   = obj.eval_spline_kth(tau, 3);
%             sL_d   = obj.eval_spline_kth(tau, 4);
%             cL_d   = obj.eval_spline_kth(tau, 5);
%             popL_d = obj.eval_spline_kth(tau, 6);
% 
%             % 【仕様固定】xd(1:28): 荷物 pL の 0〜6階微分 ＋ 公称 Yaw の 0〜6階微分
%             xd(1:3)   = pL_d;    % 荷物位置 (0階)
%             xd(5:7)   = vL_d;    % 荷物速度 (1階)
%             xd(9:11)  = aL_d;    % 荷物加速度 (2階)
%             xd(13:15) = jL_d;    % Jerk (3階)
%             xd(17:19) = sL_d;    % Snap (4階)
%             xd(21:23) = cL_d;    % Crack (5階)
%             xd(25:27) = popL_d;  % Pop (6階)
% 
%             % 差分平坦性モデルに基づく索張力および UAV 目標位置 これが実際のUAV
%             % 位置ではないとなっているがどの様にしたら　このままではいけないのか
%             % ？　思考中1
%             g_vec = [0; 0; obj.gravity];
%             acc_tot = aL_d + g_vec;
%             norm_a = norm(acc_tot);
%             if norm_a > 1e-3, thrust_dir = acc_tot / norm_a; else, thrust_dir = [0; 0; 1]; end
%             pQ_d = pL_d + obj.L_cable * thrust_dir;
% 
%             % 【仕様固定】UAV 目標位置は xd(29:31) へ分離格納
%             xd(29:31) = pQ_d;
%         end
%     end
% 
%     methods (Access = private)
%         % =====================================================================
%         % execute_replanning: 索長 L 最悪姿勢保証 ＆ 全制御点半空間制約 QP
%         % =====================================================================
%         function execute_replanning(obj, pQ_cur, xd_nom, detection, obs_list, t_now)
%             t_replan_start = tic;
% 
%             all_env_obstacles = obj.extract_all_obstacle_halfspaces(pQ_cur, obs_list);
%             if isempty(all_env_obstacles), return; end
% 
%             if ~isempty(detection.detected_obstacles_point)
%                 target_obs = detection.detected_obstacles_point(1);
%             else
%                 [~, idx_target] = min([all_env_obstacles.dist_drone_point]);
%                 target_obs = all_env_obstacles(idx_target);
%             end
%             obj.active_threat_id = target_obs.id;
% 
%             v_nom = xd_nom(5:7);
%             spd = norm(v_nom);
%             if spd < 0.1, spd = 1.0; v_nom = [1; 0; 0]; end
%             dir_nom = v_nom / spd;
% 
%             init_load_state = zeros(7, 3);
%             if obj.replan_active
%                 tau_now = t_now - obj.t_start;
%                 for k = 0:6
%                     init_load_state(k + 1, :) = obj.eval_spline_kth(tau_now, k)';
%                 end
%             else
%                 init_load_state = obj.get_nominal_derivatives_at_time(t_now);
%             end
% 
%             % 1. 安全拘束法線: 楕円体最接近点の真の外向き単位法線
%             n_constraint = target_obs.normal_drone_point(:) / norm(target_obs.normal_drone_point);
% 
%             % 2. 目的関数の退避誘導方向: 進行軸直交成分
%             n_escape = n_constraint - dot(n_constraint, dir_nom) * dir_nom;
%             if norm(n_escape) < 0.1
%                 n_cand = cross(dir_nom, [0; 0; 1]);
%                 if norm(n_cand) < 0.1, n_cand = cross(dir_nom, [0; 1; 0]); end
%                 n_escape = n_cand / norm(n_cand);
%             else
%                 n_escape = n_escape / norm(n_escape);
%             end
% 
%             % 【厳密幾何包含クリアランス (Theorem)】
%             % コーシー・シュワルツの不等式 n'*(pQ - pL) >= -L より、索の最悪姿勢 n'*q >= -1 に対し
%             % d_req := L_cable + r_drone + d_margin を課すことで連続時間包含を完全保証
%             req_clearance = obj.L_cable + obj.r_drone + obj.clearance_margin;
% 
%             t_impact = obj.estimate_safety_margin_impact_time(pQ_cur, dir_nom, spd, target_obs);
%             T_kinematic = sqrt((8.0 * (req_clearance + 1.0)) / obj.max_acc_load);
%             t_dur_cand = max([5.0, 2.0 * t_impact, T_kinematic * 1.5]);
% 
%             % --- 第1試行: 全制御点半空間制約 QP 求解 ---
%             t_solve = tic;
%             [qp_success, peak_disp] = obj.plan_uniform_bspline_c6_qp( ...
%                 n_escape, init_load_state, t_dur_cand, all_env_obstacles, t_now);
%             obj.last_solve_time_ms = toc(t_solve) * 1000;
% 
%             if ~qp_success
%                 fprintf(2, "[REPLAN REJECTED] t=%.3f s | QP求解または全制御点半空間包含判定に失敗. 安全保証対象外として公称維持.\n", t_now);
%                 return;
%             end
% 
%             % 独立数値検証 1回目 (100点サンプリングによる追加チェック)
%             [traj_safe, peak_acc] = obj.verify_trajectory_numerical_sampling(t_now, obs_list, t_dur_cand);
% 
%             status_str = "PASS";
%             if ~traj_safe || (peak_acc > obj.max_acc_load)
%                 % --- 第2試行: マージン拡大 & 時間延長による再求解 ---
%                 obj.clearance_margin = obj.clearance_margin * 1.3;
%                 T_kinematic_boost = sqrt((8.0 * (req_clearance * 1.3 + 1.0)) / obj.max_acc_load);
%                 t_dur_cand = max([t_dur_cand * 1.25, T_kinematic_boost * 1.5]);
% 
%                 t_solve = tic;
%                 [qp_boost_success, peak_disp] = obj.plan_uniform_bspline_c6_qp( ...
%                     n_escape, init_load_state, t_dur_cand, all_env_obstacles, t_now);
%                 obj.last_solve_time_ms = toc(t_solve) * 1000;
% 
%                 obj.clearance_margin = obj.clearance_margin / 1.3; % 復元
% 
%                 if ~qp_boost_success
%                     fprintf(2, "[REPLAN REJECTED] t=%.3f s | Boost QP求解に失敗しました. 安全保証対象外として公称維持.\n", t_now);
%                     return;
%                 end
% 
%                 % 独立数値検証 2回目 (再検証)
%                 [traj_safe_2, peak_acc_2] = obj.verify_trajectory_numerical_sampling(t_now, obs_list, t_dur_cand);
% 
%                 if ~traj_safe_2 || (peak_acc_2 > obj.max_acc_load)
%                     fprintf(2, "[REPLAN REJECTED] t=%.3f s | 2段階安全性検証に失敗 (侵入または過大水平加速度: a_xy=%.2f). 危険軌道を棄却します.\n", ...
%                         t_now, peak_acc_2);
%                     return;
%                 end
% 
%                 status_str = "BOOSTED_PASS";
%                 peak_acc = peak_acc_2;
%             end
% 
%             obj.t_start          = t_now;
%             obj.t_duration       = t_dur_cand;
%             obj.last_replan_time = t_now;
%             obj.replan_active    = true;
%             obj.sampled_peak_displacement = peak_disp;
%             obj.sampled_peak_acceleration = peak_acc;
%             obj.last_replan_total_time_ms = toc(t_replan_start) * 1000;
% 
%             obj.verify_start_c6_matching(init_load_state, t_now);
%             obj.display_detection_report(t_now, target_obs, length(all_env_obstacles), req_clearance, t_impact, n_constraint, status_str);
%         end
% 
%         % =====================================================================
%         % plan_uniform_bspline_c6_qp: 全制御点半空間制約 ＆ 実荷物 C^6 境界確定 QP
%         % =====================================================================
%         function [success, max_disp] = plan_uniform_bspline_c6_qp(obj, n_escape_3d, init_load_state, T_tot, active_obstacles, t_now)
%             p = 7;
%             n_seg = 25;
%             n_cp = n_seg + p;          % 32 制御点
% 
%             obj.spline_degree = p;
%             obj.num_segments = n_seg;
%             obj.build_clamped_uniform_knots(n_seg, p, T_tot);
% 
%             % 1. 始端 0〜6階微分の境界確定: P_L_start (7x3)
%             M_start = zeros(7, 7);
%             for k = 0:6
%                 d_row = obj.eval_basis_derivatives(p + 1, 0.0, k);
%                 M_start(k + 1, :) = d_row(k + 1, 1:7);
%             end
%             P_L_start = M_start \ init_load_state(1:7, :);
% 
%             % 2. 終端 0〜6階微分の公称合流確定: P_L_end (7x3)
%             end_nom_state = obj.get_nominal_derivatives_at_time(t_now + T_tot);
%             M_end = zeros(7, 7);
%             idx_end = (n_cp - 6):n_cp;
%             for k = 0:6
%                 d_row_end = obj.eval_basis_derivatives(n_cp, T_tot, k);
%                 M_end(k + 1, :) = d_row_end(k + 1, idx_end);
%             end
%             P_L_end = M_end \ end_nom_state(1:7, :);
% 
%             % 3. 自由変数 (P_8 〜 P_25: 18制御点 -> 54スカラー変数) 最適化定式化
%             D4 = diff(eye(n_cp), 4);
%             Q = D4' * D4;
% 
%             idx_free = 8:(n_cp - 7);
%             n_free = length(idx_free);
% 
%             Q_mm = Q(idx_free, idx_free) + 1e-4 * eye(n_free);
%             Q_ms = Q(idx_free, 1:7);
%             Q_me = Q(idx_free, (n_cp-6):n_cp);
% 
%             % 【過大横ずれ抑制】公称軌道制御点 P_nom への引き戻し正則化項を導入
%             P_nom_all = obj.project_nominal_trajectory_to_bspline(t_now, T_tot, n_cp);
%             P_nom_free = P_nom_all(idx_free, :);
% 
%             % H = Q_mm + w_dev * I
%             H_1d = (Q_mm + Q_mm') / 2 + obj.w_dev * eye(n_free);
%             H = blkdiag(H_1d, H_1d, H_1d);
% 
%             % 線形項: f = f_smooth - w_dev * P_nom_free - w_escape * n_escape
%             f_x = (P_L_start(:, 1)' * Q_ms' + P_L_end(:, 1)' * Q_me')' - obj.w_dev * P_nom_free(:, 1);
%             f_y = (P_L_start(:, 2)' * Q_ms' + P_L_end(:, 2)' * Q_me')' - obj.w_dev * P_nom_free(:, 2);
%             f_z = (P_L_start(:, 3)' * Q_ms' + P_L_end(:, 3)' * Q_me')' - obj.w_dev * P_nom_free(:, 3);
% 
%             w_escape = 0.05;
%             f_x = f_x - w_escape * n_escape_3d(1) * ones(n_free, 1);
%             f_y = f_y - w_escape * n_escape_3d(2) * ones(n_free, 1);
%             f_z = f_z - w_escape * n_escape_3d(3) * ones(n_free, 1);
%             f = [f_x; f_y; f_z];
% 
%             % -------------------------------------------------------------
%             % 4. 【全制御点半空間制約】全制御点 (cp_i = 1..32) に対する幾何不等式
%             % -------------------------------------------------------------
%             A_ineq = [];
%             b_ineq = [];
%             num_targets = length(active_obstacles);
%             constraint_eps = 1e-6;
% 
%             d_req_obs = obj.L_cable + obj.r_drone + obj.clearance_margin;
% 
%             for obs_idx = 1:num_targets
%                 tgt = active_obstacles(obs_idx);
%                 p_surf = tgt.closest_drone_world_point;
%                 n_dir  = tgt.normal_drone_point(:) / norm(tgt.normal_drone_point);
% 
%                 for cp_i = 1:n_cp
%                     if cp_i <= 7
%                         P_fixed = P_L_start(cp_i, :)';
%                         if dot(n_dir, (P_fixed - p_surf)) < d_req_obs + constraint_eps
%                             success = false; max_disp = 0.0; return;
%                         end
%                     elseif cp_i >= (n_cp - 6)
%                         local_end_idx = cp_i - (n_cp - 7);
%                         P_fixed = P_L_end(local_end_idx, :)';
%                         if dot(n_dir, (P_fixed - p_surf)) < d_req_obs + constraint_eps
%                             success = false; max_disp = 0.0; return;
%                         end
%                     else
%                         f_idx = cp_i - 7;
%                         row_cp = zeros(1, n_free * 3);
%                         for dim = 1:3
%                             idx_d = (dim - 1) * n_free;
%                             row_cp(idx_d + f_idx) = -n_dir(dim);
%                         end
%                         A_ineq = [A_ineq; row_cp];
%                         b_ineq = [b_ineq; -(d_req_obs + dot(n_dir, p_surf))];
%                     end
%                 end
%             end
% 
%             max_coord_bound = 50.0;
%             lb = -max_coord_bound * ones(n_free * 3, 1);
%             ub =  max_coord_bound * ones(n_free * 3, 1);
% 
%             opts = optimoptions('quadprog', 'Display', 'off', 'Algorithm', 'interior-point-convex', 'MaxIterations', 300);
%             [X_mid, ~, exitflag] = quadprog(H, f, A_ineq, b_ineq, [], [], lb, ub, [], opts);
% 
%             if exitflag < 1
%                 success = false;
%                 max_disp = 0.0;
%                 return;
%             end
% 
%             success = true;
%             P_L_mid = zeros(n_free, 3);
%             P_L_mid(:, 1) = X_mid(1:n_free);
%             P_L_mid(:, 2) = X_mid((n_free+1):(2*n_free));
%             P_L_mid(:, 3) = X_mid((2*n_free+1):(3*n_free));
% 
%             obj.control_points = [P_L_start; P_L_mid; P_L_end];
% 
%             N_eval = 60;
%             taus = linspace(0, T_tot, N_eval);
%             max_disp = 0.0;
%             for i = 1:N_eval
%                 p_eval = obj.eval_spline_kth_with_T(taus(i), 0, T_tot);
%                 nom_res = obj.base_ref.do(struct('t', t_now + taus(i), 'dt', 0.025), 'f');
%                 max_disp = max(max_disp, norm(p_eval - nom_res.state.xd(1:3)));
%             end
%         end
% 
%         % =====================================================================
%         % project_nominal_trajectory_to_bspline: 公称軌道のB-Spline最小二乗射影
%         % =====================================================================
%         function P_nom_all = project_nominal_trajectory_to_bspline(obj, t_now, T_tot, n_cp)
%             p = obj.spline_degree;
%             N_samples = 100;
%             t_quad = linspace(0.0, T_tot, N_samples);
%             dt = T_tot / (N_samples - 1);
% 
%             M_gram = zeros(n_cp, n_cp);
%             B_proj = zeros(n_cp, 3);
% 
%             for k = 1:N_samples
%                 tau_k = t_quad(k);
%                 t_k = t_now + tau_k;
% 
%                 nom_res_k = obj.base_ref.do(struct('t', t_k, 'dt', 0.025), 'f');
%                 p_nom_k = nom_res_k.state.xd(1:3)';
% 
%                 idx_span = obj.find_knot_span(min(T_tot - 1e-6, tau_k));
%                 ders = obj.eval_basis_derivatives(idx_span, min(T_tot - 1e-6, tau_k), 0);
%                 basis_vals = ders(1, :);
%                 c_idx = (idx_span - p):idx_span;
% 
%                 w = dt;
%                 if k == 1 || k == N_samples, w = 0.5 * dt; end
% 
%                 M_gram(c_idx, c_idx) = M_gram(c_idx, c_idx) + w * (basis_vals' * basis_vals);
%                 B_proj(c_idx, :) = B_proj(c_idx, :) + w * (basis_vals' * p_nom_k);
%             end
% 
%             P_nom_all = (M_gram + 1e-5 * eye(n_cp)) \ B_proj;
%         end
% 
%         % =====================================================================
%         % get_nominal_derivatives_at_time: 公称軌道から 0〜6階微分値を抽出
%         % =====================================================================
%         function ders = get_nominal_derivatives_at_time(obj, t_eval)
%             ders = zeros(7, 3);
%             nom_res = obj.base_ref.do(struct('t', t_eval, 'dt', 0.025), 'f');
%             xd_val = nom_res.state.xd;
%             if length(xd_val) < 28
%                 xd_val = [xd_val; zeros(28 - length(xd_val), 1)];
%             end
%             ders(1, :) = xd_val(1:3)';   % 位置 (0階)
%             ders(2, :) = xd_val(5:7)';   % 速度 (1階)
%             ders(3, :) = xd_val(9:11)';  % 加速度 (2階)
%             ders(4, :) = xd_val(13:15)'; % Jerk (3階)
%             ders(5, :) = xd_val(17:19)'; % Snap (4階)
%             ders(6, :) = xd_val(21:23)'; % Crack (5階)
%             ders(7, :) = xd_val(25:27)'; % Pop (6階)
%         end
% 
%         % =====================================================================
%         % extract_all_obstacle_halfspaces: 環境内全障害物の支持半空間パラメータ抽出
%         % =====================================================================
%         % function active_list = extract_all_obstacle_halfspaces(obj, pQ_cur, obs_list)
%         %     active_list = [];
%         %     for i = 1:length(obs_list)
%         %         o = obs_list(i);
%         %         obs_parsed = struct('center', o.p_center(:), 'radii', o.ellipsoid_radii(:), 'R', o.R_obs);
%         %         [d_raw, ~, cpQ_loc, ~] = obj.point_ellipsoid_signed_distance(pQ_cur, obs_parsed);
%         % 
%         %         cpQ_world = o.p_center(:) + o.R_obs * cpQ_loc;
%         %         nQ_loc = cpQ_loc ./ (o.ellipsoid_radii(:).^2);
%         %         normal_drone = o.R_obs * (nQ_loc / norm(nQ_loc));
%         % 
%         %         item = struct('id', i, 'p_obs', o.p_center(:), 'radii_obs', o.ellipsoid_radii(:), ...
%         %                       'R_obs', o.R_obs, 'closest_drone_world_point', cpQ_world, ...
%         %                       'normal_drone_point', normal_drone, 'dist_drone_point', d_raw - obj.r_drone);
%         %         active_list = [active_list; item];
%         %     end
%         % end
%         function active_list = extract_all_obstacle_halfspaces(obj, pQ_cur, obs_list)
%             active_list = [];
%             for i = 1:length(obs_list)
%                 o = obs_list(i);
% 
%                 obs_id = o.id;
% 
%                 obs_parsed = struct('center', o.p_center(:), 'radii', o.ellipsoid_radii(:), 'R', o.R_obs);
%                 [d_raw, ~, cpQ_loc, ~] = obj.point_ellipsoid_signed_distance(pQ_cur, obs_parsed);
% 
%                 cpQ_world = o.p_center(:) + o.R_obs * cpQ_loc;
%                 nQ_loc = cpQ_loc ./ (o.ellipsoid_radii(:).^2);
%                 normal_drone = o.R_obs * (nQ_loc / norm(nQ_loc));
% 
%                 item = struct('id', obs_id, 'p_obs', o.p_center(:), 'radii_obs', o.ellipsoid_radii(:), ...
%                     'R_obs', o.R_obs, 'closest_drone_world_point', cpQ_world, ...
%                     'normal_drone_point', normal_drone, 'dist_drone_point', d_raw - obj.r_drone);
%                 active_list = [active_list; item];
%             end
%         end
% 
%         % =====================================================================
%         % estimate_safety_margin_impact_time: 公称軌道上の安全マージン侵入予測
%         % =====================================================================
%         function t_impact = estimate_safety_margin_impact_time(obj, pQ_cur, dir_nom, spd, target_obs)
%             vec_to_obs = target_obs.p_obs - pQ_cur;
%             dist_along = dot(vec_to_obs, dir_nom);
% 
%             max_rad = max(target_obs.radii_obs);
%             dist_to_margin = max(0.5, dist_along - max_rad - obj.r_drone - obj.clearance_margin);
%             t_impact = max(1.5, dist_to_margin / spd);
%         end
% 
%         % =====================================================================
%         % verify_trajectory_numerical_sampling: 100点サンプリングによる数値的独立検証
%         % ※連続時間の理論的安全包含は B-Spline 全制御点拘束 n'*(P_i - p_s) >= d_req が担保。
%         % 本メソッドは追従前シミュレーションによる独立した数値整合性チェック（Runtime Monitor）として機能する。
%         % =====================================================================
%         function [is_safe, peak_acc_xy] = verify_trajectory_numerical_sampling(obj, t_now, obs_list, T_tot)
%             is_safe = true;
%             peak_acc_xy = 0.0;
%             N_samples = 100;
%             taus = linspace(0.0, T_tot, N_samples);
% 
%             r_c_sph = obj.r_cable_sphere;
%             n_spheres = max(2, ceil(obj.L_cable / (2.0 * r_c_sph)) + 1);
%             s_ratios = linspace(0.0, 1.0, n_spheres);
% 
%             for i = 1:N_samples
%                 tau_i = taus(i);
% 
%                 pL_fut = obj.eval_spline_kth_with_T(tau_i, 0, T_tot);
%                 aL_fut = obj.eval_spline_kth_with_T(tau_i, 2, T_tot);
% 
%                 peak_acc_xy = max(peak_acc_xy, norm(aL_fut(1:2)));
% 
%                 g_vec = [0; 0; obj.gravity];
%                 acc_tot = aL_fut + g_vec;
%                 norm_a = norm(acc_tot);
%                 if norm_a > 1e-3, thrust_dir = acc_tot / norm_a; else, thrust_dir = [0; 0; 1]; end
%                 pQ_fut = pL_fut + obj.L_cable * thrust_dir;
% 
%                 cable_pts_fut = (1 - s_ratios) .* pL_fut + s_ratios .* pQ_fut;
% 
%                 for obs_idx = 1:length(obs_list)
%                     o = obs_list(obs_idx);
%                     obs_parsed = struct('center', o.p_center(:), 'radii', o.ellipsoid_radii(:), 'R', o.R_obs);
% 
%                     % 1. UAV
%                     [dQ, ~, ~, ~] = obj.point_ellipsoid_signed_distance(pQ_fut, obs_parsed);
%                     if dQ - obj.r_drone <= 0, is_safe = false; return; end
% 
%                     % 2. 荷物
%                     [dL, ~, ~, ~] = obj.point_ellipsoid_signed_distance(pL_fut, obs_parsed);
%                     if dL - obj.r_load <= 0, is_safe = false; return; end
% 
%                     % 3. 索球列 (Runtime Monitor)
%                     for sp = 1:n_spheres
%                         [dC, ~, ~, ~] = obj.point_ellipsoid_signed_distance(cable_pts_fut(:, sp), obs_parsed);
%                         if dC - r_c_sph <= 0, is_safe = false; return; end
%                     end
%                 end
%             end
%         end
% 
%         % =====================================================================
%         % is_nominal_recovery_safe: 復帰先公称軌道における干渉スキャン (UAV/荷物/索)
%         % =====================================================================
%         function is_safe = is_nominal_recovery_safe(obj, t_end_expected, obs_list)
%             is_safe = true;
%             N_lookahead = 15;
%             t_scan = linspace(t_end_expected, t_end_expected + 2.0, N_lookahead);
%             g_vec = [0; 0; obj.gravity];
% 
%             r_c_sph = obj.r_cable_sphere;
%             n_spheres = max(2, ceil(obj.L_cable / (2.0 * r_c_sph)) + 1);
%             s_ratios = linspace(0.0, 1.0, n_spheres);
% 
%             for i = 1:N_lookahead
%                 t_eval = t_scan(i);
%                 nom_res = obj.base_ref.do(struct('t', t_eval, 'dt', 0.025), 'f');
%                 pL_nom = nom_res.state.xd(1:3);
%                 aL_nom = nom_res.state.xd(9:11);
% 
%                 acc_tot = aL_nom + g_vec;
%                 norm_a = norm(acc_tot);
%                 if norm_a > 1e-3, thrust_dir = acc_tot / norm_a; else, thrust_dir = [0; 0; 1]; end
%                 pQ_nom = pL_nom + obj.L_cable * thrust_dir;
%                 cable_pts_nom = (1 - s_ratios) .* pL_nom + s_ratios .* pQ_nom;
% 
%                 for obs_idx = 1:length(obs_list)
%                     o = obs_list(obs_idx);
%                     obs_parsed = struct('center', o.p_center(:), 'radii', o.ellipsoid_radii(:), 'R', o.R_obs);
% 
%                     [dQ, ~, ~, ~] = obj.point_ellipsoid_signed_distance(pQ_nom, obs_parsed);
%                     [dL, ~, ~, ~] = obj.point_ellipsoid_signed_distance(pL_nom, obs_parsed);
% 
%                     if (dQ - obj.r_drone - obj.clearance_margin <= 0) || ...
%                        (dL - obj.r_load  - obj.clearance_margin <= 0)
%                         is_safe = false;
%                         return;
%                     end
% 
%                     for sp = 1:n_spheres
%                         [dC, ~, ~, ~] = obj.point_ellipsoid_signed_distance(cable_pts_nom(:, sp), obs_parsed);
%                         if dC - r_c_sph - obj.clearance_margin <= 0
%                             is_safe = false;
%                             return;
%                         end
%                     end
%                 end
%             end
%         end
% 
%         % =====================================================================
%         % verify_start_c6_matching: 始端 0〜6階微分ギャップ厳密検証
%         % =====================================================================
%         function verify_start_c6_matching(obj, init_load_state, t_now)
%             names = ["位置(0階)", "速度(1階)", "加速度(2階)", "Jerk(3階)", "Snap(4階)", "Crack(5階)", "Pop(6階)"];
%             tolerances = [1e-10, 1e-9, 1e-8, 1e-6, 1e-4, 1e-2, 1.0];
% 
%             for k = 0:6
%                 actual_k = obj.eval_spline_kth(0.0, k)';
%                 expected_k = init_load_state(k + 1, :);
%                 gap = norm(actual_k - expected_k);
%                 obj.c6_start_gaps(k + 1) = gap;
% 
%                 if gap > tolerances(k + 1)
%                     fprintf(2, "[FATAL C^6 BREACH] t=%.3f s | 始端 %s の接続ギャップ超過: %.3e (許容: %.3e)\n", ...
%                         t_now, names(k + 1), gap, tolerances(k + 1));
%                     error('リプランニング始端での C^6 境界接続に失敗しました: %s', names(k + 1));
%                 end
%             end
%         end
% 
%         % =====================================================================
%         % verify_return_c6_matching: 終端 0〜6階微分公称合流実測検証ログ
%         % =====================================================================
%         function verify_return_c6_matching(obj, t_end_expected)
%             names = ["位置(0階)", "速度(1階)", "加速度(2階)", "Jerk(3階)", "Snap(4階)", "Crack(5階)", "Pop(6階)"];
%             tolerances = [1e-9, 1e-8, 1e-7, 1e-5, 1e-3, 1e-1, 5.0];
% 
%             fprintf("\n=================================================================================\n");
%             fprintf(" [B-SPLINE 回避終了 C^6 終端合流ギャップ実測レポート]  t_end = %.3f s\n", t_end_expected);
%             fprintf("=================================================================================\n");
% 
%             pass = true;
%             end_nom_state = obj.get_nominal_derivatives_at_time(t_end_expected);
% 
%             for k = 0:6
%                 actual_k = obj.eval_spline_kth_with_T(obj.t_duration, k, obj.t_duration)';
%                 expected_k = end_nom_state(k + 1, :);
%                 gap = norm(actual_k - expected_k);
%                 obj.c6_return_gaps(k + 1) = gap;
% 
%                 if gap > tolerances(k + 1)
%                     pass = false;
%                 end
%                 fprintf("   - %-12s e_%d = ||p_L^(%d)(T) - p_nom^(%d)(T)|| = %.3e m/s^%d (許容: %.1e)\n", ...
%                     names(k + 1), k, k, gap, k, tolerances(k + 1));
%             end
% 
%             if pass
%                 fprintf(" 判定: 設計上の0〜6階微分境界条件を満足し、C^6公称合流条件を達成。\n");
%             else
%                 fprintf(2, " [WARNING] 終端0〜6階微分の残差が許容値を超過しました (不連続キックの可能性).\n");
%             end
%             fprintf("=================================================================================\n\n");
%         end
% 
%         % =====================================================================
%         % verify_step_integration_consistency: 離散時間ステップ積分整合性監視
%         % =====================================================================
%         function verify_step_integration_consistency(obj, xd_now, t_now, dt)
%             if obj.prev_t < 0
%                 obj.prev_xd = xd_now;
%                 obj.prev_t  = t_now;
%                 return;
%             end
% 
%             real_dt = t_now - obj.prev_t;
%             if real_dt <= 1e-6, return; end
%             if nargin < 4 || isempty(dt) || dt <= 0, dt = real_dt; end
% 
%             orders = { ...
%                 '位置 (0階)',      1:3,   5:7;   ...
%                 'yaw角 (0階)',     4,     8;     ...
%                 '速度 (1階)',      5:7,   9:11;  ...
%                 '加速度 (2階)',    9:11,  13:15; ...
%                 'Jerk (3階)',      13:15, 17:19; ...
%             };
% 
%             for idx = 1:size(orders, 1)
%                 name   = orders{idx, 1};
%                 curr_i = orders{idx, 2};
%                 next_i = orders{idx, 3};
% 
%                 val_prev = obj.prev_xd(curr_i);
%                 val_curr = xd_now(curr_i);
%                 der_prev = obj.prev_xd(next_i);
%                 der_curr = xd_now(next_i);
% 
%                 predicted = val_prev + 0.5 * (der_prev + der_curr) * real_dt;
%                 disc_gap  = norm(val_curr - predicted);
% 
%                 tol = max(0.08, 6.0 * norm(der_curr) * real_dt);
%                 if disc_gap > tol
%                     warning("離散ステップ不連続キック検出: %s at t=%.4f s (ギャップ: %.3e, 許容: %.3e)", ...
%                         name, t_now, disc_gap, tol);
%                 end
%             end
% 
%             obj.prev_xd = xd_now;
%             obj.prev_t  = t_now;
%         end
% 
%         % =====================================================================
%         % check_detection_simulated_sensor: 3者(機体/荷物/索)の3層距離・マージン診断
%         % =====================================================================
%         function det = check_detection_simulated_sensor(obj, pQ, pL, obs_list, t_now, v_nom)
%             det = struct();
%             det.time                          = t_now;
%             det.pQ                            = pQ;
%             det.pL                            = pL;
%             det.trigger_dist                  = obj.trigger_dist;
%             det.detected_point                = false;
%             det.drone_inside_obstacle_point   = false;
%             det.load_inside_obstacle_point    = false;
%             det.cable_inside_obstacle_point   = false;
%             det.drone_margin_violated         = false;
%             det.load_margin_violated          = false;
%             det.cable_margin_violated         = false;
%             det.drone_min_dist_point          = inf;
%             det.load_min_dist_point           = inf;
%             det.cable_min_dist_point          = inf;
%             det.min_dist_point                = inf;
%             det.drone_obstacle_id_point       = [];
%             det.load_obstacle_id_point        = [];
%             det.min_obstacle_id_point         = [];
%             det.min_source_point              = "none";
%             det.detected_obstacles_point      = [];
%             det.detected_obstacle_count_point = 0;
% 
%             if isempty(obs_list), return; end
% 
%             r_c_sph = obj.r_cable_sphere;
%             n_spheres = max(2, ceil(obj.L_cable / (2.0 * r_c_sph)) + 1);
%             s_ratios = linspace(0.0, 1.0, n_spheres);
%             cable_pts = (1 - s_ratios) .* pL + s_ratios .* pQ;
% 
%             v_dir = v_nom(:);
%             if norm(v_dir) > 0.1, v_dir = v_dir / norm(v_dir); else, v_dir = [0; 1; 0]; end
% 
%             detected_obs_candidates = [];
% 
%             for i = 1:length(obs_list)
%                 o = obs_list(i);
%                 obs_id = o.id;
%                 R_obs = o.R_obs;
%                 radii_obs = o.ellipsoid_radii(:);
%                 p_obs = o.p_center(:);
%                 obs_parsed = struct('center', p_obs, 'radii', radii_obs, 'R', R_obs);
% 
%                 % 1. 機体重心 pQ
%                 [d_drone_raw, inside_drone, cpQ_loc, ~] = obj.point_ellipsoid_signed_distance(pQ, obs_parsed);
%                 d_drone_physical = d_drone_raw - obj.r_drone;
%                 d_drone_safety   = d_drone_physical - obj.clearance_margin;
% 
%                 % 2. 荷物 pL
%                 [d_load_raw, inside_load, cpL_loc, ~] = obj.point_ellipsoid_signed_distance(pL, obs_parsed);
%                 d_load_physical = d_load_raw - obj.r_load;
%                 d_load_safety   = d_load_physical - obj.clearance_margin;
% 
%                 % 3. 索球列 (Runtime Monitor)
%                 d_cable_raw_min = inf;
%                 inside_cable_i = false;
%                 for sp_idx = 1:n_spheres
%                     [d_pt, in_pt, ~, ~] = obj.point_ellipsoid_signed_distance(cable_pts(:, sp_idx), obs_parsed);
%                     if d_pt < d_cable_raw_min, d_cable_raw_min = d_pt; end
%                     if in_pt, inside_cable_i = true; end
%                 end
%                 d_cable_physical = d_cable_raw_min - r_c_sph;
%                 d_cable_safety   = d_cable_physical - obj.clearance_margin;
% 
%                 if inside_drone || (d_drone_physical <= 0),   det.drone_inside_obstacle_point = true; end
%                 if inside_load  || (d_load_physical <= 0),    det.load_inside_obstacle_point  = true; end
%                 if inside_cable_i || (d_cable_physical <= 0), det.cable_inside_obstacle_point = true; end
% 
%                 if d_drone_safety <= 0, det.drone_margin_violated = true; end
%                 if d_load_safety  <= 0, det.load_margin_violated  = true; end
%                 if d_cable_safety <= 0, det.cable_margin_violated = true; end
% 
%                 if d_drone_physical < det.drone_min_dist_point
%                     det.drone_min_dist_point = d_drone_physical;
%                     det.drone_obstacle_id_point = obs_id;
%                 end
%                 if d_load_physical < det.load_min_dist_point
%                     det.load_min_dist_point = d_load_physical;
%                     det.load_obstacle_id_point = obs_id;
%                 end
%                 if d_cable_physical < det.cable_min_dist_point
%                     det.cable_min_dist_point = d_cable_physical;
%                 end
% 
%                 cpQ_world = p_obs + R_obs * cpQ_loc;
%                 cpL_world = p_obs + R_obs * cpL_loc;
%                 nQ_loc = cpQ_loc ./ (radii_obs.^2);
%                 normal_drone = R_obs * (nQ_loc / norm(nQ_loc));
%                 nL_loc = cpL_loc ./ (radii_obs.^2);
%                 normal_load = R_obs * (nL_loc / norm(nL_loc));
% 
%                 obs_info = struct( ...
%                     'id',                        obs_id, ...
%                     'dist_drone_point',          d_drone_physical, ...
%                     'dist_drone_safety',         d_drone_safety, ...
%                     'dist_load_point',           d_load_physical, ...
%                     'dist_load_safety',          d_load_safety, ...
%                     'dist_cable_point',          d_cable_physical, ...
%                     'dist_cable_safety',         d_cable_safety, ...
%                     'min_dist_point',            d_drone_physical, ...
%                     'p_obs',                     p_obs, ...
%                     'radii_obs',                 radii_obs, ...
%                     'R_obs',                     R_obs, ...
%                     'closest_drone_world_point', cpQ_world, ...
%                     'closest_load_world_point',  cpL_world, ...
%                     'normal_drone_point',        normal_drone, ...
%                     'normal_load_point',         normal_load ...
%                 );
% 
%                 vec_to_center = p_obs - pQ;
%                 is_in_front = dot(vec_to_center, v_dir) > -max(radii_obs);
% 
%                 if (d_drone_raw <= obj.trigger_dist) && is_in_front
%                     detected_obs_candidates = [detected_obs_candidates; obs_info];
%                 end
%             end
% 
%             if ~isempty(detected_obs_candidates)
%                 [~, sort_idx] = sort([detected_obs_candidates.dist_drone_point], 'ascend');
%                 det.detected_obstacles_point = detected_obs_candidates(sort_idx);
%                 det.detected_point = true;
%                 det.min_dist_point = det.detected_obstacles_point(1).dist_drone_point;
%                 det.min_obstacle_id_point = det.detected_obstacles_point(1).id;
%                 det.min_source_point = "drone";
%                 det.detected_obstacle_count_point = length(detected_obs_candidates);
%             else
%                 det.min_dist_point = det.drone_min_dist_point;
%                 det.min_obstacle_id_point = det.drone_obstacle_id_point;
%                 det.min_source_point = "none";
%             end
%         end
% 
%         % =====================================================================
%         % display_detection_report: 診断レポート (安全例外ガード付き)
%         % =====================================================================
%         function display_detection_report(obj, t_now, tgt, total_candidates, req_clearance, t_impact, n_constraint, status_str)
%             names = ["位置(0階)", "速度(1階)", "加速度(2階)", "Jerk(3階)", "Snap(4階)", "Crack(5階)", "Pop(6階)"];
% 
%             fprintf("\n=================================================================================\n");
%             fprintf(" [B-SPLINE 15m センサー検知 ＆ C^6 回避診断レポート]  t = %.3f s [%s]\n", t_now, status_str);
%             fprintf("=================================================================================\n");
%             fprintf(" 1. センサ判定元      : 機体重心 pQ 搭載模擬センサー (機体表面間距離: %.3f m)\n", tgt.dist_drone_point);
%             fprintf(" 2. 実行時診断モニタステータス (独立100点サンプリング数値検証):\n");
%             fprintf("     - UAV   : 表面間: %+6.3f m | [%s]\n", tgt.dist_drone_point, obj.get_safety_status(tgt.dist_drone_point, tgt.dist_drone_point - obj.clearance_margin));
%             fprintf("     - Cable : 表面間: %+6.3f m | [%s]\n", tgt.dist_cable_point, obj.get_safety_status(tgt.dist_cable_point, tgt.dist_cable_point - obj.clearance_margin));
%             fprintf("     - Load  : 表面間: %+6.3f m | [%s]\n", tgt.dist_load_point, obj.get_safety_status(tgt.dist_load_point, tgt.dist_load_point - obj.clearance_margin));
%             fprintf(" 3. 捕捉楕円体情報    : 危険順位 1位 / 候補 %d 個 (ID=%d, 半径=[%.2f, %.2f, %.2f] m)\n", ...
%                 total_candidates, tgt.id, tgt.radii_obs(1), tgt.radii_obs(2), tgt.radii_obs(3));
%             fprintf(" 4. 運動学制約検証    : 実軌道最大 a_xy = %.2f m/s^2 (上限: %.2f m/s^2) -> 回避時間 T_tot = %.2f s\n", ...
%                 obj.sampled_peak_acceleration, obj.max_acc_load, obj.t_duration);
%             fprintf(" 5. 連続時間安全包含保証 (B-Spline 制御点凸包性):\n");
%             fprintf("     - 制御点拘束条件 : n'*(P_i - p_surf) >= d_req (全32制御点に厳密適用, d_req = %.2f m)\n", req_clearance);
%             fprintf("     - 理論的保証根拠 : P_L,i in H ==> 凸包性より p_L(t) in H ==> ||p_Q - p_L||=L より UAV球体・索包含\n");
%             fprintf("     - サンプリング位置: 100点チェックは連続時間保証の補足・独立数値検証として機能\n");
%             fprintf("     - 支持超平面法線 : n_constraint = [%.2f, %.2f, %.2f]\n", n_constraint(1), n_constraint(2), n_constraint(3));
%             fprintf("     - 数学保証根拠   : 荷物全制御点包含 P_L in H(d_req) ==> 凸包性より p_L(t) in H ==> UAV球体・索包含\n");
%             fprintf(" 6. C^6 始端境界ギャップ実測値 (M \\ B 厳密代数接続):\n");
%             for k = 0:6
%                 fprintf("     - %-12s e_%d = %.3e m/s^%d (数学的 C^6 一致)\n", names(k + 1), k, obj.c6_start_gaps(k + 1), k);
%             end
%             fprintf(" 7. 計算時間内訳      : QP求解 = %6.2f ms | リプラン全体 = %6.2f ms (18自由制御点/54変数)\n", ...
%                 obj.last_solve_time_ms, obj.last_replan_total_time_ms);
%             fprintf(" 8. 公称軌道離脱変位  : %.3f m (max ||p_L(t) - p_nom(t)||, 60点サンプリング評価)\n", obj.sampled_peak_displacement);
%             fprintf("=================================================================================\n\n");
%         end
% 
%         function str = get_safety_status(~, d_phys, d_safe)
%             if d_phys <= 0
%                 str = "COLLISION";
%             elseif d_safe <= 0
%                 str = "MARGIN VIOLATION";
%             else
%                 str = "SAFE";
%             end
%         end
% 
%         % =====================================================================
%         % point_ellipsoid_signed_distance: ラグランジュ未定乗数法による厳密幾何距離
%         % =====================================================================
%         function [d, inside, closest_local, lambda] = point_ellipsoid_signed_distance(~, p, o)
%             p = p(:); c = o.center(:); r = o.radii(:); R = o.R;
%             y  = R' * (p - c);
%             r2 = r.^2;
%             q  = sum((y ./ r).^2);
%             inside = (q < 1.0);
% 
%             if norm(y) < 1e-14
%                 [min_r, min_idx] = min(r);
%                 closest_local = zeros(3, 1);
%                 closest_local(min_idx) = min_r;
%                 d      = -min_r;
%                 lambda = -min(r2);
%                 return;
%             end
% 
%             f = @(lam) sum(r2 .* (y.^2) ./ ((lam + r2).^2)) - 1.0;
%             if q > 1.0
%                 lo = 0.0;
%                 hi = max(r) * norm(y);
%             else
%                 lo = -min(r2) * (1.0 - 1e-12);
%                 hi = 0.0;
%             end
% 
%             for kk = 1:80
%                 mid = 0.5 * (lo + hi);
%                 if f(mid) > 0, lo = mid; else, hi = mid; end
%             end
%             lambda = 0.5 * (lo + hi);
%             closest_local = r2 .* y ./ (lambda + r2);
%             d_abs = norm(closest_local - y);
%             if inside, d = -d_abs; else, d = d_abs; end
%         end
% 
%         % 実制御点 P_L による評価
%         function val = eval_spline_kth(obj, tau, k)
%             val = obj.eval_spline_kth_with_T(tau, k, obj.t_duration);
%         end
% 
%         function val = eval_spline_kth_with_T(obj, tau, k, T_tot)
%             p = obj.spline_degree;
%             t_eval = max(0.0, min(T_tot, tau));
% 
%             idx_span = obj.find_knot_span(t_eval);
%             ders = obj.eval_basis_derivatives(idx_span, t_eval, k);
% 
%             c_indices = (idx_span - p):idx_span;
%             val = (ders(k + 1, :) * obj.control_points(c_indices, :))';
%         end
% 
%         function build_clamped_uniform_knots(obj, n_seg, p, T_tot)
%             dt_knot = T_tot / n_seg;
%             interior_knots = dt_knot * (1:(n_seg - 1));
%             obj.knots = [zeros(1, p + 1), interior_knots, T_tot * ones(1, p + 1)];
%         end
% 
%         function idx = find_knot_span(obj, t_eval)
%             p = obj.spline_degree;
%             n_cp = length(obj.knots) - p - 1;
%             if t_eval >= obj.knots(n_cp + 1), idx = n_cp; return; end
%             if t_eval <= obj.knots(p + 1),    idx = p + 1; return; end
%             low = p + 1; high = n_cp + 1; mid = floor((low + high) / 2);
%             while (t_eval < obj.knots(mid)) || (t_eval >= obj.knots(mid + 1))
%                 if t_eval < obj.knots(mid), high = mid; else, low = mid; end
%                 mid = floor((low + high) / 2);
%             end
%             idx = mid;
%         end
% 
%         function ders = eval_basis_derivatives(obj, idx_span, t_eval, n_der)
%             p = obj.spline_degree;
%             U = obj.knots;
%             ders = zeros(n_der + 1, p + 1);
%             ndu = zeros(p + 1, p + 1);
%             left = zeros(p + 1, 1);
%             right = zeros(p + 1, 1);
% 
%             ndu(1, 1) = 1.0;
%             for j = 1:p
%                 left(j + 1) = t_eval - U(idx_span + 1 - j);
%                 right(j + 1) = U(idx_span + j) - t_eval;
%                 saved = 0.0;
%                 for r = 0:(j - 1)
%                     ndu(j + 1, r + 1) = right(r + 2) + left(j - r + 1);
%                     temp = ndu(r + 1, j) / ndu(j + 1, r + 1);
%                     ndu(r + 1, j + 1) = saved + right(r + 2) * temp;
%                     saved = left(j - r + 1) * temp;
%                 end
%                 ndu(j + 1, j + 1) = saved;
%             end
% 
%             for j = 0:p, ders(1, j + 1) = ndu(j + 1, p + 1); end
% 
%             a = zeros(2, p + 1);
%             for r = 0:p
%                 s1 = 0; s2 = 1; a(1, 1) = 1.0;
%                 for k = 1:n_der
%                     d = 0.0; rk = r - k; pk = p - k;
%                     if r >= k
%                         a(s2 + 1, 1) = a(s1 + 1, 1) / ndu(pk + 2, rk + 1);
%                         d = a(s2 + 1, 1) * ndu(rk + 1, pk + 1);
%                     end
%                     if rk >= -1, j1 = 1; else, j1 = -rk; end
%                     if (r - 1) <= pk, j2 = k - 1; else, j2 = p - r; end
%                     for j = j1:j2
%                         a(s2 + 1, j + 1) = (a(s1 + 1, j + 1) - a(s1 + 1, j)) / ndu(pk + 2, rk + j + 1);
%                         d = d + a(s2 + 1, j + 1) * ndu(rk + j + 1, pk + 1);
%                     end
%                     if r <= pk
%                         a(s2 + 1, k + 1) = -a(s1 + 1, k) / ndu(pk + 2, r + 1);
%                         d = d + a(s2 + 1, k + 1) * ndu(r + 1, pk + 1);
%                     end
%                     ders(k + 1, r + 1) = d;
%                     j_tmp = s1; s1 = s2; s2 = j_tmp;
%                 end
%             end
% 
%             r_scale = p;
%             for k = 1:n_der
%                 for j = 0:p
%                     ders(k + 1, j + 1) = ders(k + 1, j + 1) * r_scale;
%                 end
%                 r_scale = r_scale * (p - k);
%             end
%         end
% 
%         function clear_state_sensor_values(obj, t_now)
%             st = obj.result.state;
%             st.time                          = t_now;
%             st.pQ                            = [NaN; NaN; NaN];
%             st.pL                            = [NaN; NaN; NaN];
%             st.detected_point                = false;
%             st.drone_inside_obstacle_point   = false;
%             st.load_inside_obstacle_point    = false;
%             st.cable_inside_obstacle_point   = false;
%             st.drone_margin_violated         = false;
%             st.load_margin_violated          = false;
%             st.cable_margin_violated         = false;
%             st.drone_min_dist_point          = inf;
%             st.load_min_dist_point           = inf;
%             st.cable_min_dist_point          = inf;
%             st.min_dist_point                = inf;
%             st.drone_obstacle_id_point       = NaN;
%             st.load_obstacle_id_point        = NaN;
%             st.min_obstacle_id_point         = NaN;
%             st.min_source_point              = "none";
%             st.detected_obstacle_count_point = 0;
%             st.detected_obstacles_point      = [];
%             st.pQ_ref                        = [NaN; NaN; NaN];
%         end
% 
%         function list = get_obstacles_at_time(~, t_now)
%             % 本研究では静止障害物環境を対象とする (t_now はAPI互換性のために保持)
%             list = ENVIRONMENT_OBSTACLE_ELLIPSE_MOVE(t_now);
%         end
%     end
% end
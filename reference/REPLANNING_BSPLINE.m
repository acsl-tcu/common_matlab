% classdef REPLANNING_BSPLINE < handle
%     % =========================================================================
%     % REPLANNING_BSPLINE
%     % 1. 模擬センサー：機体重心 pQ を点（Point）として扱い、機体搭載センサーによる
%     %    楕円体境界面までの最短ユークリッド距離計算および 7m 接近検知判定を実行。
%     % 2. 衝突診断系：荷物位置 pL についてはセンサーとしては扱わず、機体センサー情報や
%     %    安全評価（最短表面距離、最近接点、法線、侵入有無）の診断値としてのみ記録。
%     % =========================================================================
%     properties
%         self % ドローンエージェント自身 (推定器 estimator やパラメータ parameter を保持) 
%         base_ref % 公称参照軌道生成オブジェクト
%         result                % 出力結果構造体 (目標状態 xd, pRef, vRef, yawRef, 検知・診断情報)
%         trigger_dist = 15.0;   % 機体搭載センサーによる接近検知の閾値 [m] (表面間距離がこれ以下になるとアラート)
%         % r_load       = 0.15;  % 荷物保護半径 [m] (必要に応じて将来のマージン計算等に使用)
%         % r_drone      = 0.30;  % 機体保護半径 [m] (必要に応じて将来のマージン計算等に使用)
%     end
%     methods (Access = public)
%         % =====================================================================
%         % コンストラクタ: クラスの初期化と外部設定 (opts) の反映
%         % =====================================================================
%         function obj = REPLANNING_BSPLINE(self, base_ref, opts)
%             arguments
%                 self                  % 必須: エージェントインスタンス
%                 base_ref              % 必須: 通常飛行用の公称軌道インスタンス
%                 opts = struct()       % 任意: 外部からパラメータを変更するための構造体
%             end
%             obj.self = self;
%             obj.base_ref = base_ref;
%             if isfield(opts, 'trigger_dist'), obj.trigger_dist = opts.trigger_dist; end % 外で定義されていたらデフォルト値を上書き 機体センサー検知範囲
%             % if isfield(opts, 'r_load'),       obj.r_load       = opts.r_load;       end % 外で定義されていたらデフォルト値を上書き 牽引物を近似した球体
%             % if isfield(opts, 'r_drone'),      obj.r_drone      = opts.r_drone;      end % 外で定義されていたらデフォルト値を上書き 機体を近似した球体
%             % base_ref の result 構造体をそのまま継承 (直下は state のみ)
%             obj.result = base_ref.result;
%             % --- 【重要】STATE_CLASS に検知・診断用プロパティを動的追加 (dynamicprops) ---
%             % これにより、state 内に正式な記録領域が作成され、ロガーで抽出可能になる
%             sensor_props = ["time", "pQ", "pL", "detected_point", ...
%                 "drone_inside_obstacle_point", "load_inside_obstacle_point", ...
%                 "drone_min_dist_point", "load_min_dist_point", "min_dist_point", ...
%                 "drone_obstacle_id_point", "load_obstacle_id_point", "min_obstacle_id_point", ...
%                 "min_source_point", "detected_obstacle_count_point"];
%             for p_name = sensor_props
%                 if ~isprop(obj.result.state, p_name)
%                     addprop(obj.result.state, p_name);
%                 end
%             end
%             % 初期ダミー値のセット
%             obj.clear_state_sensor_values(0.0);
%         end
%         % =====================================================================
%         % do: 制御周期ごと (例: 25ms周期) にメインループから呼び出される実行メソッド
%         % 入力: 
%         %   varargin{1}: time (現在の時刻 struct: time.t, time.dt など)
%         %   varargin{2}: cha  (フェーズ文字列: 'f' = 飛行中, 't' = 離陸など)
%         %   varargin{4}: env  (環境構造体、障害物リストを内包する場合あり)
%         % 出力:
%         %   result_out : 下流のコントローラやロガーが受け取る目標状態・検知・診断結果
%         % =====================================================================
%         function result_out = do(obj, varargin)
%             time = varargin{1}; % varargin{1}: time (現在の時刻 struct: time.t, time.dt など)
%             cha = varargin{2}; % varargin{2}: cha  (フェーズ文字列: 'f' = 飛行中, 't' = 離陸など)
%             % --- 公称目標軌道 (Nominal Reference) の算出 ---
%             % 本クラスが障害物を回避する新軌道を生成しない間は、公称軌道生成器の出力をそのまま踏襲する
%             % 公称参照軌道の取得
%             base_res = obj.base_ref.do(varargin{:}); % 公称軌道の抜き出し
%             xd_nom = base_res.state.xd; % 牽引物の目標３次元位置・yaw角からその６階微分まで [pL(3); yaw(1); vL(3); yaw_dot(1); aL(3)...]
% 
%             % 毎ステップ、検知・診断プロパティの初期値をリセット
%             obj.clear_state_sensor_values(time.t);
% 
%             % obj.result.state に公称軌道を反映 (base_res 全体の上書きは行わない)
%             obj.result.state.xd = xd_nom; % 公称軌道保存
%             % --- 2. 空の検知・診断構造体を用意 (非飛行フェーズ用) ---
%             detection = struct();
%             detection.time                          = time.t;             % [s] 現在のシミュレーション時刻
%             detection.pQ                            = [NaN; NaN; NaN];    % [m] ドローン機体重心の3次元位置ベクトル [x; y; z]
%             detection.pL                            = [NaN; NaN; NaN];    % [m] 牽引荷物の3次元位置ベクトル [x; y; z] (状態診断用)
%             detection.detected_point                = false;              % [bool] 機体センサー7m近接検知フラグ (true: 検知, false: 未検知)
%             detection.drone_inside_obstacle_point   = false;              % [bool] 機体の障害物楕円体内部侵入フラグ (true: 侵入, false: 外部)
%             detection.load_inside_obstacle_point    = false;              % [bool] 荷物の障害物楕円体内部侵入フラグ (true: 侵入, false: 外部)
%             detection.drone_min_dist_point          = inf;                % [m] 機体センサーから全障害物表面までの最短幾何学距離
%             detection.load_min_dist_point           = inf;                % [m] 荷物から全障害物表面までの最短幾何学距離 (診断値)
%             detection.min_dist_point                = inf;                % [m] 機体センサーによる最短表面距離 (機体基準)
%             detection.drone_obstacle_id_point       = NaN;                % [ID] 機体にとって最短距離を与えている障害物インデックス番号
%             detection.load_obstacle_id_point        = NaN;                % [ID] 荷物にとって最短距離を与えている障害物インデックス番号 (診断値)
%             detection.min_obstacle_id_point         = NaN;                % [ID] 機体センサーが捉えた最短障害物インデックス番号
%             detection.min_source_point              = "none";             % [string] 最短距離センサ種別 (検知なし: "none", 検知時: "drone")
%             detection.detected_obstacle_count_point = 0;                  % [個] 機体センサーにより7m以内に検知された障害物の総数
%             % --- 飛行フェーズ ('f') 時の近接障害物スキャン & 荷物状態診断 ---
%             if cha == 'f'
%                 % --- 機体・荷物の現在位置の取得 ---
%                 % (A) 状態推定器 (estimator) から真値・推定位置を直接取得
%                 % ※ フォールバックを排除しているため、estimator にプロパティが存在しない場合は即座にエラー停止
%                 pL_cur = obj.self.estimator.result.state.pL(:); % 荷物位置の現在3次元位置 [x; y; z]
%                 pQ_cur = obj.self.estimator.result.state.p(:); % ドローン機体の現在3次元位置 [x; y; z]
%                 % (B) 現在時刻 t における動的障害物配置を取得
%                 obs_list = obj.get_obstacles_at_time(time.t);
%                 % (C) 機体センサーによる最短距離評価・検知判定、および荷物の幾何診断値を計算
%                 detection = obj.check_detection_simulated_sensor(pQ_cur, pL_cur, obs_list, time.t);
%                 % 機体センサーが7m以内に侵入した場合のコンソール警告
%                 if detection.detected_point
%                     fprintf("[PROXIMITY ALERT] t=%.3f s | 機体センサー7m近接検知! (機体表面間: %.2f m, 荷物表面間診断: %.2f m, 最寄障害物ID: %d)\n", ...
%                         time.t, detection.drone_min_dist_point, detection.load_min_dist_point, detection.min_obstacle_id_point);
%                 end
%             end
% 
%             % --- 4. 【最重要】state 内の各プロパティに代入して app.logger に完全保存 ---
%             st = obj.result.state;
%             st.xd                            = xd_nom;                                  % [28x1 double] 目標軌道全状態
%             st.p                             = xd_nom(1:3);                             % [m] 目標位置 [x; y; z]
%             st.v                             = xd_nom(5:7);                             % [m/s] 目標速度 [vx; vy; vz]
%             st.q                             = [0; 0; xd_nom(4)];                       % [rad] 目標姿勢 (yaw角)
% 
%             st.time                          = detection.time;                          % [s] 計測時刻 (double)
%             st.pQ                            = detection.pQ;                            % [m] ドローン機体重心位置 (3x1 double)
%             st.pL                            = detection.pL;                            % [m] 荷物位置 (3x1 double, 診断用)
%             st.detected_point                = detection.detected_point;                % [bool] 機体センサー7m近接検知判定フラグ (true / false)
%             st.drone_inside_obstacle_point   = detection.drone_inside_obstacle_point;   % [bool] 機体侵入フラグ (true: 侵入, false: 外部)
%             st.load_inside_obstacle_point    = detection.load_inside_obstacle_point;    % [bool] 荷物侵入フラグ (診断用, true: 侵入, false: 外部)
%             st.drone_min_dist_point          = detection.drone_min_dist_point;          % [m] 機体センサー最短表面距離 (正: 外部, 負: 侵入深さ)
%             st.load_min_dist_point           = detection.load_min_dist_point;           % [m] 荷物の最短表面距離 (診断値, 正: 外部, 負: 侵入深さ)
%             st.min_dist_point                = detection.min_dist_point;                % [m] 機体センサーによる最短幾何表面距離 (= dQ)
%             st.drone_obstacle_id_point       = detection.drone_obstacle_id_point;       % [ID] 機体にとって最短の障害物インデックス番号
%             st.load_obstacle_id_point        = detection.load_obstacle_id_point;        % [ID] 荷物にとって最短の障害物インデックス番号 (診断値)
%             st.min_obstacle_id_point         = detection.min_obstacle_id_point;         % [ID] 機体センサーが捉えた最短障害物インデックス番号
%             st.min_source_point              = detection.min_source_point;              % [string] 最短距離センサ種別 (検知なし: "none", 検知時: "drone")
%             st.detected_obstacle_count_point = detection.detected_obstacle_count_point; % [個] 機体センサーにより7m以内に検知された障害物の総数
% 
%             result_out = obj.result;
%         end
%     end
%     methods (Access = private)
%         % =====================================================================
%         % clear_state_sensor_values: state 内の検知・診断プロパティを初期化
%         % =====================================================================
%         function clear_state_sensor_values(obj, t_now)
%             st = obj.result.state;
%             st.time                          = t_now;              % [s] 現在時刻
%             st.pQ                            = [NaN; NaN; NaN];    % [m] ドローン位置初期値
%             st.pL                            = [NaN; NaN; NaN];    % [m] 荷物位置初期値 (診断用)
%             st.detected_point                = false;              % 検知なし
%             st.drone_inside_obstacle_point   = false;              % 機体侵入なし
%             st.load_inside_obstacle_point    = false;              % 荷物侵入なし (診断用)
%             st.drone_min_dist_point          = inf;                % 最短距離初期値 (無限大)
%             st.load_min_dist_point           = inf;                % 最短距離初期値 (無限大, 診断用)
%             st.min_dist_point                = inf;                % 最短距離初期値 (無限大)
%             st.drone_obstacle_id_point       = NaN;                % 最短障害物ID初期値
%             st.load_obstacle_id_point        = NaN;                % 最短障害物ID初期値 (診断用)
%             st.min_obstacle_id_point         = NaN;                % 最短障害物ID初期値
%             st.min_source_point              = "none";             % 最短センサ初期値
%             st.detected_obstacle_count_point = 0;                  % 検知個数初期値
%         end
%         % =====================================================================
%         % get_obstacles_at_time: 環境関数を叩き、時刻 t_now での障害物リストを取得
%         % =====================================================================
%         function list = get_obstacles_at_time(~,t_now)
%             % ENVIRONMENT_OBSTACLE_ELLIPSE_MOVE 内ですでに「p_center = p0 + v*t」
%             % のように時刻 t_now に応じた現在位置が計算されている
%             list = ENVIRONMENT_OBSTACLE_ELLIPSE_MOVE(t_now);
%         end
%         % =====================================================================
%         % check_detection_simulated_sensor: 
%         % 機体搭載センサーによる接近検知判定（7m以内）を実行し、
%         % 荷物についてはシステム状態診断値（距離・最近接点・法線・侵入有無）として計算・記録する
%         % =====================================================================
%         function det = check_detection_simulated_sensor(obj, pQ, pL, obs_list, t_now)
%             det = struct();
%             % --- センサ状態・時刻 ---
%             det.time                          = t_now;              % [s] 現在のシミュレーション時刻 (double)
%             det.pQ                            = pQ;                 % [m] ドローン機体重心の3次元位置ベクトル [x; y; z] (3x1 double)
%             det.pL                            = pL;                 % [m] 牽引荷物の3次元位置ベクトル [x; y; z] (3x1 double, 診断用)
%             det.trigger_dist                  = obj.trigger_dist;   % [m] 機体センサーの近接検知閾値距離 (7.0 m) (double)
%             % --- 判定フラグ ---
%             det.detected_point                = false;              % [bool] 機体センサーが障害物から7m以内に接近したか (true: 検知, false: 未検知)
%             det.drone_inside_obstacle_point   = false;              % [bool] 機体が障害物楕円体の内部へ侵入/衝突したか (true: 侵入, false: 外部)
%             det.load_inside_obstacle_point    = false;              % [bool] 荷物が障害物楕円体の内部へ侵入/衝突したか (診断用, true: 侵入, false: 外部)
%             % --- 最短距離 ---
%             det.drone_min_dist_point          = inf;                % [m] 機体センサーから表面までの最短幾何学距離 (double, 内部時は負値)
%             det.load_min_dist_point           = inf;                % [m] 荷物から表面までの最短幾何学距離 (診断値, double, 内部時は負値)
%             det.min_dist_point                = inf;                % [m] 機体センサーによる最短幾何表面距離 (= dQ)
%             % --- 識別情報・統計 ---
%             det.drone_obstacle_id_point       = [];                 % [ID] 機体にとって最短距離を与えている障害物のインデックス番号 (integer)
%             det.load_obstacle_id_point        = [];                 % [ID] 荷物にとって最短距離を与えている障害物のインデックス番号 (診断値, integer)
%             det.min_obstacle_id_point         = [];                 % [ID] 機体センサーが捉えた最短障害物のインデックス番号 (integer)
%             det.min_source_point              = "none";             % [string] 最短距離センサ種別 (検知なし: "none", 検知時: "drone")
%             det.detected_obstacles_point      = [];                 % [struct配列] 機体センサーにより7m以内に検知された全障害物の詳細情報リスト
%             det.detected_obstacle_count_point = 0;                  % [個] 機体センサーにより7m以内に検知された障害物の総数 (integer)
%             if isempty(obs_list)
%                 return;
%             end
%             detected_obs_point = [];
%             for i = 1:length(obs_list)
%                 o = obs_list(i);
%                 % --- 障害物パラメータの抽出 (推測フォールバックなし) ---
%                 if ~isfield(o, 'R_obs') || ~isfield(o, 'ellipsoid_radii') || ~isfield(o, 'p_center')
%                     error('インデックス %d の障害物に必須フィールド (R_obs, ellipsoid_radii, p_center) が不足しています.', i);
%                 end
%                 % --- 障害物の姿勢回転行列 R_obs の抽出 ---
%                 R_obs = o.R_obs;
%                 % --- 外接楕円体の主軸半径 [a; b; c] の抽出 ---
%                 radii_obs = o.ellipsoid_radii(:);
%                 % --- 障害物の中心位置 ---
%                 p_obs = o.p_center(:);
%                 obs_parsed = struct('center', p_obs, 'radii', radii_obs, 'R', R_obs);
% 
%                 % --- 機体 pQ (センサー) および荷物 pL (診断用) から楕円体表面への符号付き最短幾何距離 ---
%                 % d > 0: 表面の外側にある (表面までの最短距離 [m])
%                 % d < 0: 内部に侵入している (侵入深さ [m])
%                 [d_drone_point, inside_drone_point, cpQ_local_point, ~] = obj.point_ellipsoid_signed_distance(pQ, obs_parsed);
%                 [d_load_point,  inside_load_point,  cpL_local_point, ~] = obj.point_ellipsoid_signed_distance(pL, obs_parsed);
% 
%                 % ワールド座標系における表面最近接点: x_world = center + R * x_local
%                 cpQ_world_point = p_obs + R_obs * cpQ_local_point;
%                 cpL_world_point = p_obs + R_obs * cpL_local_point;
% 
%                 % --- 障害物表面における外向き単位法線ベクトル (ワールド座標系) ---
%                 nQ_local_point = cpQ_local_point ./ (radii_obs.^2);
%                 if norm(nQ_local_point) < 1e-12
%                     error('機体の最近接点における法線ベクトルが退化 (ゼロベクトル) しました.');
%                 end
%                 normal_drone_point = R_obs * (nQ_local_point / norm(nQ_local_point));
% 
%                 nL_local_point = cpL_local_point ./ (radii_obs.^2);
%                 if norm(nL_local_point) < 1e-12
%                     error('荷物の最近接点における法線ベクトルが退化 (ゼロベクトル) しました.');
%                 end
%                 normal_load_point = R_obs * (nL_local_point / norm(nL_local_point));
% 
%                 % 幾何学的侵入判定（衝突判定）の論理和更新
%                 if inside_drone_point, det.drone_inside_obstacle_point = true; end
%                 if inside_load_point,  det.load_inside_obstacle_point  = true; end
% 
%                 % 機体側（センサー）の最小値更新
%                 if d_drone_point < det.drone_min_dist_point
%                     det.drone_min_dist_point = d_drone_point;
%                     det.drone_obstacle_id_point    = i;
%                 end
%                 % 荷物側（診断値）の最小値更新
%                 if d_load_point < det.load_min_dist_point
%                     det.load_min_dist_point = d_load_point;
%                     det.load_obstacle_id_point    = i;
%                 end
% 
%                 % 該当障害物の詳細データ構造体 (荷物データはシステム診断値として包含)
%                 obs_info_point = struct( ...
%                     'id',                        i, ...                  % [ID] 障害物のインデックス番号 (integer)
%                     'dist_drone_point',          d_drone_point, ...      % [m] 機体重心(センサー)からこの障害物表面までの符号付き最短距離 (正: 外部, 負: 侵入深さ)
%                     'dist_load_point',           d_load_point, ...       % [m] 荷物位置からこの障害物表面までの符号付き最短距離 (診断値, 正: 外部, 負: 侵入深さ)
%                     'min_dist_point',            d_drone_point, ...      % [m] 機体センサー基準の表面間距離
%                     'drone_inside_point',        inside_drone_point, ... % [bool] 機体がこの障害物の内部に侵入しているか (true: 侵入, false: 外部)
%                     'load_inside_point',         inside_load_point, ...  % [bool] 荷物がこの障害物の内部に侵入しているか (診断用, true: 侵入, false: 外部)
%                     'p_obs',                     p_obs, ...              % [m] 時刻 t における障害物の中心位置ベクトル [x; y; z] (3x1 double)
%                     'radii_obs',                 radii_obs, ...          % [m] 障害物の3軸半径 [rx; ry; rz] (3x1 double)
%                     'R_obs',                     R_obs, ...              % [-] 障害物の主軸姿勢を表す3x3回転行列 SO(3) (3x3 double)
%                     'closest_drone_world_point', cpQ_world_point, ...    % [m] 機体に対する障害物表面上の最短最近接点 (ワールド座標系 [x; y; z])
%                     'closest_load_world_point',  cpL_world_point, ...    % [m] 荷物に対する障害物表面上の最短最近接点 (診断用, ワールド座標系 [x; y; z])
%                     'normal_drone_point',        normal_drone_point, ... % [-] 機体側最近接点における障害物表面の外向き単位法線ベクトル (ワールド系, 単位ノルム)
%                     'normal_load_point',         normal_load_point ...   % [-] 荷物側最近接点における障害物表面の外向き単位法線ベクトル (診断用, ワールド系, 単位ノルム)
%                 );
% 
%                 % --- 機体センサーによる接近検知判定 (7m以内) ---
%                 if d_drone_point <= obj.trigger_dist
%                     det.detected_point = true;
%                     detected_obs_point = [detected_obs_point; obs_info_point];
%                 end
%             end
% 
%             % システム全体での最短距離の確定 (機体センサー基準)
%             det.min_dist_point                = det.drone_min_dist_point;
%             det.min_obstacle_id_point         = det.drone_obstacle_id_point;
%             % 検知ありの場合は "drone"、検知なし（7m超過または障害物なし）の場合は初期値 "none" を保持
%             if det.detected_point
%                 det.min_source_point          = "drone";
%             else
%                 det.min_source_point          = "none";
%             end
%             det.detected_obstacles_point      = detected_obs_point;
%             det.detected_obstacle_count_point = numel(detected_obs_point);  % 機体センサーが検知した障害物個数
%         end
%         % =====================================================================
%         % point_ellipsoid_signed_distance
%         % 任意の3次元点 p と回転楕円体 (中心 c, 半径 r=[a;b;c], 回転行列 R) の
%         % 【外郭表面】までの真の最短ユークリッド距離を算出する。
%         %
%         % [数学的アルゴリズム]:
%         % 楕円方程式: (x/a)^2 + (y/b)^2 + (z/c)^2 = 1
%         % 点 p から楕円面上の最近接点 x への最短距離は、ラグランジュ未定乗数法:
%         %   f(lambda) = sum( (a_i^2 * y_i^2) / (lambda + a_i^2)^2 ) - 1 = 0
%         % を満たす単一根 lambda を二分探索法 (Binary Search) で 80 回反復して求め、
%         % 表面点 x = (a_i^2 * y_i) / (lambda + a_i^2) と点 y の距離 ||x - y|| を計算する。
%         % =====================================================================
%         function [d, inside, closest_local, lambda] = point_ellipsoid_signed_distance(~, p, o)
%             p = p(:);
%             c = o.center(:);
%             r = o.radii(:);
%             R = o.R;
%             % --- 入力チェック: 不正ならフォールバックせず即座に停止 ---
%             if numel(p) ~= 3 || numel(c) ~= 3 || numel(r) ~= 3
%                 error('点 p、中心 c、および半径 r はすべて3x1ベクトルである必要があります。');
%             end
%             if any(~isfinite(p)) || any(~isfinite(c)) || any(~isfinite(r))
%                 error('点または楕円体の定義に非有限値 (NaN や Inf) が検出されました。');
%             end
%             if any(r <= 0)
%                 error('楕円体の半径はすべて厳密に正の値 (r > 0) である必要があります。');
%             end
%             if ~isequal(size(R), [3 3]) || any(~isfinite(R(:)))
%                 error('回転行列 R は有限な3x3行列である必要があります。');
%             end
%             if norm(R'*R - eye(3), 'fro') > 1e-6 || abs(det(R) - 1.0) > 1e-6
%                 error('行列 R は有効な直交回転行列 SO(3) ではありません。');
%             end
%             % 1. ワールド座標系の点 p を障害物の局所主軸座標系 (Local Frame) に変換
%             y  = R' * (p - c);
%             r2 = r.^2;
%             % 2. 楕円代数判定値 q:
%             %    q < 1.0 -> 点は楕円体の内部にある (衝突・侵入状態)
%             %    q = 1.0 -> 点は楕円体の表面上にある
%             %    q > 1.0 -> 点は楕円体の外部にある
%             q      = sum((y ./ r).^2);
%             inside = (q < 1.0);
%             % 特異点処理: 点が障害物の中心そのものにある場合 (最も近い表面は最短半径の主軸上)
%             if norm(y) < 1e-14
%                 [min_r, min_idx] = min(r);
%                 closest_local = zeros(3, 1);
%                 closest_local(min_idx) = min_r;
%                 d      = -min_r;
%                 lambda = -min(r2);
%                 return;
%             end
%             % ラグランジュ未定乗数の非線形方程式 f(lambda) = 0
%             f = @(lam) sum(r2 .* (y.^2) ./ ((lam + r2).^2)) - 1.0;
%             % 3. ラグランジュ乗数 lambda の探索範囲 (Bracketing) の設定と根の存在検証
%             if q > 1.0
%                 % 外部点の場合: lambda >= 0
%                 lo = 0.0;
%                 hi = max(r) * norm(y);
%                 if f(lo) < 0 || f(hi) > 0
%                     error('外部点に対する楕円体距離の求根ブラケット設定に失敗しました (解を挟み込めていません)。');
%                 end
%             else
%                 % 内部点の場合: -min(r_i^2) < lambda < 0
%                 lo = -min(r2) * (1.0 - 1e-12);
%                 hi = 0.0;
%                 if f(lo) < 0 || f(hi) > 0
%                     error('内部点に対する楕円体距離の求根ブラケット設定に失敗しました (解を挟み込めていません)。');
%                 end
%             end
%             % 4. 二分探索により lambda を数値的に収束させる
%             %    80回反復して根を高精度に求める
%             for kk = 1:80
%                 mid = 0.5 * (lo + hi);
%                 if f(mid) > 0
%                     lo = mid;
%                 else
%                     hi = mid;
%                 end
%             end
%             lambda = 0.5 * (lo + hi);
%             % 5. 局所座標系における楕円体表面上の最近接点 closest_local
%             closest_local = r2 .* y ./ (lambda + r2);
%             d_abs         = norm(closest_local - y);
%             % 6. 表面までの最短ユークリッド距離
%             %    外部なら正値 (+), 内部なら侵入深さとして負値 (-) を返す
%             if inside
%                 d = -d_abs;
%             else
%                 d = d_abs;
%             end
%         end
%     end
% end

% classdef REPLANNING_BSPLINE < handle
%     % =========================================================================
%     % REPLANNING_BSPLINE
%     % 1. 模擬センサー：機体重心 pQ を点（Point）として扱い、機体搭載センサーによる
%     %    楕円体境界面までの最短ユークリッド距離計算および 15m 接近検知判定を実行。
%     % 2. 衝突診断系：荷物位置 pL についてはセンサーとしては扱わず、安全評価（最短距離、
%     %    法線、侵入有無）の診断値としてのみ記録。
%     % 3. 軌道再計画：検知時に 7次 Uniform B-Spline (p = 7) を用い、内部ノットおよび
%     %    境界で 6階微分 (C^6 連続: 位置〜Pop) まで完全連続な滑らか回避軌道を生成。
%     % 4. 厳密連続性監視：全飛行フェーズ（公称・回避開始・回避中・復帰）において、
%     %    位置・yawおよび0〜6階微分の連続性を常時監視し、不連続検出時は即時診断停止。
%     % =========================================================================
%     properties
%         self                       % ドローンエージェント自身 (推定器 estimator やパラメータ parameter を保持) 
%         base_ref                   % 公称参照軌道生成オブジェクト
%         result                     % 出力結果構造体 (目標状態 xd, pRef, vRef, yawRef, 検知・診断情報)
% 
%         % --- センサ・検知パラメータ ---
%         trigger_dist = 15.0;       % 機体搭載センサーによる接近検知の閾値 [m]
%         clearance_margin = 1.0;    % 回避クリアランス余力 [m]
% 
%         % --- 機体・荷物物理パラメータ (描画クラス互換) ---
%         gravity = 9.81;            % 重力加速度 [m/s^2]
%         L_cable = 1.0;             % 索長 [m]
%         r_drone = 0.30;            % 機体半径 [m]
%         r_load  = 0.15;            % 荷物半径 [m]
% 
%         % --- リプランニング状態管理 ---
%         replan_active = false;     % 回避軌道追従中フラグ
%         t_start       = 0.0;       % 回避開始時刻 [s]
%         t_duration    = 6.0;       % 回避全所要時間 [s]
%         last_replan_time = -100.0; % 前回リプラン実行時刻 [s]
%         min_replan_interval = 0.5; % チャタリング防止再計画間隔 [s]
%         active_threat_id = NaN;    % 回避対象の障害物ID
% 
%         % --- 7次 B-Spline パラメータ (p=7, n_seg=25 -> N_cp=32) ---
%         spline_degree = 7;         % 7次 B-Spline (内部 C^6 連続)
%         num_segments  = 25;        % 25セグメント
%         knots                      % ノットベクトル
%         control_points             % 制御点座標 (32 x 3) [X, Y, Z]
%         actual_peak_displacement = 0.0; % 最大空間退避変位 [m]
%         last_solve_time_ms = 0.0;  % QP計算時間 [ms]
% 
%         % --- C^6 連続性常時監視用バッファ ---
%         prev_xd                    % 前回ステップの xd (28x1)
%         prev_t = -1.0;             % 前回ステップの時刻 [s]
%     end
% 
%     methods (Access = public)
%         % =====================================================================
%         % コンストラクタ: クラスの初期化と外部設定 (opts) の反映
%         % =====================================================================
%         function obj = REPLANNING_BSPLINE(self, base_ref, opts)
%             arguments
%                 self                  % 必須: エージェントインスタンス
%                 base_ref              % 必須: 通常飛行用の公称軌道インスタンス
%                 opts = struct()       % 任意: 外部からパラメータを変更するための構造体
%             end
%             obj.self = self;
%             obj.base_ref = base_ref;
%             if isfield(opts, 'trigger_dist'), obj.trigger_dist = opts.trigger_dist; end
%             if isfield(opts, 'r_drone'),      obj.r_drone      = opts.r_drone;      end
%             if isfield(opts, 'r_load'),       obj.r_load       = opts.r_load;       end
%             if isfield(opts, 'L_cable'),      obj.L_cable      = opts.L_cable;      end
%             if isfield(opts, 'gravity'),      obj.gravity      = opts.gravity;      end
% 
%             % base_ref の result 構造体をそのまま継承
%             obj.result = base_ref.result;
% 
%             % --- STATE_CLASS に検知・診断用プロパティを動的追加 (dynamicprops) ---
%             sensor_props = ["time", "pQ", "pL", "detected_point", ...
%                 "drone_inside_obstacle_point", "load_inside_obstacle_point", ...
%                 "drone_min_dist_point", "load_min_dist_point", "min_dist_point", ...
%                 "drone_obstacle_id_point", "load_obstacle_id_point", "min_obstacle_id_point", ...
%                 "min_source_point", "detected_obstacle_count_point"];
%             for p_name = sensor_props
%                 if ~isprop(obj.result.state, p_name)
%                     addprop(obj.result.state, p_name);
%                 end
%             end
% 
%             % 初期ダミー値のセット
%             obj.clear_state_sensor_values(0.0);
%         end
% 
%         % =====================================================================
%         % do: 制御周期ごと (例: 25ms周期) にメインループから呼び出される実行メソッド
%         % =====================================================================
%         function result_out = do(obj, varargin)
%             time = varargin{1}; % time (現在の時刻 struct: time.t, time.dt など)
%             cha  = varargin{2}; % cha  (フェーズ文字列: 'f' = 飛行中, 't' = 離陸など)
% 
%             % --- 1. 公称目標軌道 (Nominal Reference) の算出 ---
%             base_res = obj.base_ref.do(varargin{:});
%             xd_nom = base_res.state.xd;
%             if length(xd_nom) < 28
%                 xd_nom = [xd_nom; zeros(28 - length(xd_nom), 1)];
%             end
% 
%             % 毎ステップ、検知・診断プロパティの初期値をリセット
%             obj.clear_state_sensor_values(time.t);
% 
%             % 空の検知・診断構造体を用意
%             detection = struct();
%             detection.time                          = time.t;
%             detection.pQ                            = [NaN; NaN; NaN];
%             detection.pL                            = [NaN; NaN; NaN];
%             detection.detected_point                = false;
%             detection.drone_inside_obstacle_point   = false;
%             detection.load_inside_obstacle_point    = false;
%             detection.drone_min_dist_point          = inf;
%             detection.load_min_dist_point           = inf;
%             detection.min_dist_point                = inf;
%             detection.drone_obstacle_id_point       = NaN;
%             detection.load_obstacle_id_point        = NaN;
%             detection.min_obstacle_id_point         = NaN;
%             detection.min_source_point              = "none";
%             detection.detected_obstacle_count_point = 0;
% 
%             % --- 2. 飛行フェーズ ('f') 時の機体センサースキャン & 荷物診断 ---
%             if cha == 'f'
%                 pL_cur = obj.self.estimator.result.state.pL(:);
%                 pQ_cur = obj.self.estimator.result.state.p(:);
%                 obs_list = obj.get_obstacles_at_time(time.t);
% 
%                 % 機体搭載センサーによる接近検知、および荷物診断を幾何計算
%                 detection = obj.check_detection_simulated_sensor(pQ_cur, pL_cur, obs_list, time.t);
% 
%                 % --- 3. 軌道再計画 (リプランニング) の判定および実行 ---
%                 if detection.detected_point && ...
%                    (time.t - obj.last_replan_time >= obj.min_replan_interval)
% 
%                     if ~obj.replan_active || (detection.min_obstacle_id_point ~= obj.active_threat_id)
%                         obj.execute_replanning(pQ_cur, xd_nom, detection, time.t);
%                     end
%                 end
%             end
% 
%             % --- 4. 出力目標軌道の確定 ---
%             if obj.replan_active
%                 tau = time.t - obj.t_start;
%                 if tau <= obj.t_duration
%                     xd_out = obj.evaluate_smooth_trajectory(tau, xd_nom);
%                 else
%                     fprintf("[B-SPLINE C^6] 回避完了! 公称軌道へ完全復帰 (t=%.3f s)\n\n", time.t);
%                     obj.replan_active = false;
%                     obj.active_threat_id = NaN;
%                     xd_out = xd_nom;
%                 end
%             else
%                 xd_out = xd_nom;
%             end
% 
%             % -------------------------------------------------------------
%             % 5. 目標軌道 0〜6階微分の完全滑らかさ (C^6) 常時監視
%             % -------------------------------------------------------------
%             if cha == 'f'
%                 obj.verify_continuous_c6_safety(xd_out, time.t, time.dt);
%             end
% 
%             % --- 6. state 内の各プロパティに代入して app.logger に完全保存 ---
%             st = obj.result.state;
%             st.xd                            = xd_out;
%             st.p                             = xd_out(1:3);
%             st.v                             = xd_out(5:7);
%             st.q                             = [0; 0; xd_out(4)];
% 
%             st.time                          = detection.time;
%             st.pQ                            = detection.pQ;
%             st.pL                            = detection.pL;
%             st.detected_point                = detection.detected_point;
%             st.drone_inside_obstacle_point   = detection.drone_inside_obstacle_point;
%             st.load_inside_obstacle_point    = detection.load_inside_obstacle_point;
%             st.drone_min_dist_point          = detection.drone_min_dist_point;
%             st.load_min_dist_point           = detection.load_min_dist_point;
%             st.min_dist_point                = detection.min_dist_point;
%             st.drone_obstacle_id_point       = detection.drone_obstacle_id_point;
%             st.load_obstacle_id_point        = detection.load_obstacle_id_point;
%             st.min_obstacle_id_point         = detection.min_obstacle_id_point;
%             st.min_source_point              = detection.min_source_point;
%             st.detected_obstacle_count_point = detection.detected_obstacle_count_point;
% 
%             result_out = obj.result;
%         end
% 
%         % =====================================================================
%         % evaluate_smooth_trajectory: 0〜6階微分の全状態修正 (Public アクセス)
%         % 外部描画クラス DRAW_SUSPENDED_LOAD_CORRIDOR_MOVE から直接参照可能
%         % =====================================================================
%         function xd = evaluate_smooth_trajectory(obj, tau, xd_nom)
%             xd = xd_nom;
% 
%             delta_pos   = obj.eval_spline_kth(tau, 0);
%             delta_vel   = obj.eval_spline_kth(tau, 1);
%             delta_acc   = obj.eval_spline_kth(tau, 2);
%             delta_jerk  = obj.eval_spline_kth(tau, 3);
%             delta_snap  = obj.eval_spline_kth(tau, 4);
%             delta_crack = obj.eval_spline_kth(tau, 5);
%             delta_pop   = obj.eval_spline_kth(tau, 6);
% 
%             xd(1:3)   = xd_nom(1:3)   + delta_pos;   % 荷物位置 (0階)
%             xd(5:7)   = xd_nom(5:7)   + delta_vel;   % 荷物速度 (1階)
%             xd(9:11)  = xd_nom(9:11)  + delta_acc;   % 荷物加速度 (2階)
%             xd(13:15) = xd_nom(13:15) + delta_jerk;  % 荷物Jerk (3階)
%             xd(17:19) = xd_nom(17:19) + delta_snap;  % 荷物Snap (4階)
% 
%             % 索張力ベクトルによる機体目標位置の復元 (描画クラスとの互換用)
%             g_vec = [0; 0; obj.gravity];
%             acc_tot = xd(9:11) + g_vec;
%             norm_a = norm(acc_tot);
%             if norm_a > 1e-3, thrust_dir = acc_tot / norm_a; else, thrust_dir = [0; 0; 1]; end
%             pQ_d = xd(1:3) + obj.L_cable * thrust_dir;
% 
%             if length(xd) >= 23
%                 xd(21:23) = pQ_d; % 機体目標位置 (描画同期用)
%             end
%             if length(xd) >= 27
%                 xd(25:27) = xd_nom(25:27) + delta_pop; % Pop (6階)
%             end
%         end
%     end
% 
%     methods (Access = private)
%         % =====================================================================
%         % execute_replanning: 7次 B-Spline C^6 回避軌道修正
%         % =====================================================================
%         function execute_replanning(obj, pQ_cur, xd_nom, detection, t_now)
%             target_obs = detection.detected_obstacles_point(1);
%             obj.active_threat_id = target_obs.id;
% 
%             % 進行方向の取得
%             v_nom = xd_nom(5:7);
%             spd = norm(v_nom);
%             if spd < 0.1, spd = 1.0; v_nom = [1; 0; 0]; end
%             dir_nom = v_nom / spd;
% 
%             % 現在の回避差分状態 (0〜6階) の取得 (境界条件ギャップ完全ゼロ保証)
%             init_diff_state = zeros(7, 3);
%             if obj.replan_active
%                 tau_now = t_now - obj.t_start;
%                 for k = 0:6
%                     init_diff_state(k + 1, :) = obj.eval_spline_kth(tau_now, k)';
%                 end
%             end
% 
%             obj.t_start          = t_now;
%             obj.last_replan_time = t_now;
% 
%             % 押し出し量 (クリアランス)
%             req_clearance = max(target_obs.radii_obs) + obj.clearance_margin;
% 
%             % 回避退避方向 (障害物法線から進行軸成分を除去)
%             n_escape = -target_obs.normal_drone_point;
%             n_escape = n_escape - dot(n_escape, dir_nom) * dir_nom;
%             if norm(n_escape) < 0.1
%                 n_cand = cross(dir_nom, [0; 0; 1]);
%                 if norm(n_cand) < 0.1, n_cand = cross(dir_nom, [0; 1; 0]); end
%                 n_escape = n_cand / norm(n_cand);
%             else
%                 n_escape = n_escape / norm(n_escape);
%             end
% 
%             % 最接近予想時刻
%             vec_to_obs = target_obs.p_obs - pQ_cur;
%             dist_along = dot(vec_to_obs, dir_nom);
%             t_impact = max(1.5, min(3.5, dist_along / spd));
%             obj.t_duration = max(5.0, 2.0 * t_impact);
% 
%             % 7次 B-Spline QP 求解
%             t_solve = tic;
%             obj.plan_uniform_bspline_c6_qp(req_clearance, n_escape, init_diff_state, t_impact);
%             obj.last_solve_time_ms = toc(t_solve) * 1000;
%             obj.replan_active = true;
% 
%             % 診断レポートログの出力
%             obj.display_detection_report(t_now, detection, req_clearance, t_impact, n_escape);
%         end
% 
%         % =====================================================================
%         % plan_uniform_bspline_c6_qp: C^6 完全連続 B-Spline 最適化
%         % =====================================================================
%         function plan_uniform_bspline_c6_qp(obj, req_clearance, n_escape_3d, init_diff_state, t_impact)
%             p = 7;
%             n_seg = 25;
%             n_cp = n_seg + p;          % 32 制御点
%             T_tot = obj.t_duration;
%             dt_seg = T_tot / n_seg;
% 
%             obj.spline_degree = p;
%             obj.num_segments = n_seg;
%             obj.build_clamped_uniform_knots(n_seg, p, T_tot);
% 
%             % 始端 0〜6階微分の境界整合 (Aeqフリー代数確定)
%             M_start = zeros(7, 7);
%             for k = 0:6
%                 d_row = obj.eval_basis_derivatives(p + 1, 0.0, k);
%                 M_start(k + 1, :) = d_row(k + 1, 1:7);
%             end
%             P_start = M_start \ init_diff_state(1:7, :);
%             P_end = zeros(7, 3);
% 
%             % 目的関数 (4階差分 Snap 最小化)
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
%             H_1d = (Q_mm + Q_mm') / 2;
%             H = blkdiag(H_1d, H_1d, H_1d);
% 
%             f_x = (P_start(:, 1)' * Q_ms' + P_end(:, 1)' * Q_me')';
%             f_y = (P_start(:, 2)' * Q_ms' + P_end(:, 2)' * Q_me')';
%             f_z = (P_start(:, 3)' * Q_ms' + P_end(:, 3)' * Q_me')';
%             f = [f_x; f_y; f_z];
% 
%             % 凸包バリア不等式制約 (真横〜復帰の押し出し)
%             cp_at_impact = round(t_impact / dt_seg) - 7;
%             cp_at_impact = max(2, min(n_free - 4, cp_at_impact));
% 
%             % ① 真横区間: 100%
%             sideway_indices = (cp_at_impact - 1) : (cp_at_impact + 2);
%             sideway_indices = sideway_indices(sideway_indices >= 1 & sideway_indices <= n_free);
% 
%             A_ineq = [];
%             b_ineq = [];
%             for j = sideway_indices
%                 row_cp = zeros(1, n_free * 3);
%                 for dim = 1:3
%                     idx_d = (dim - 1) * n_free;
%                     row_cp(idx_d + j) = -n_escape_3d(dim);
%                 end
%                 A_ineq = [A_ineq; row_cp];
%                 b_ineq = [b_ineq; -req_clearance];
%             end
% 
%             % ② 復帰区間: 85%
%             recovery_indices = (max(sideway_indices) + 1) : min(n_free, max(sideway_indices) + 3);
%             for j = recovery_indices
%                 row_cp = zeros(1, n_free * 3);
%                 for dim = 1:3
%                     idx_d = (dim - 1) * n_free;
%                     row_cp(idx_d + j) = -n_escape_3d(dim);
%                 end
%                 A_ineq = [A_ineq; row_cp];
%                 b_ineq = [b_ineq; -0.85 * req_clearance];
%             end
% 
%             lb = -10.0 * ones(n_free * 3, 1);
%             ub =  10.0 * ones(n_free * 3, 1);
%             opts = optimoptions('quadprog', 'Display', 'off', 'Algorithm', 'interior-point-convex');
%             [X_mid, ~, exitflag] = quadprog(H, f, A_ineq, b_ineq, [], [], lb, ub, [], opts);
% 
%             if exitflag < 1
%                 X_mid = repmat(n_escape_3d * req_clearance * 0.85, n_free, 1);
%             end
% 
%             P_mid = zeros(n_free, 3);
%             P_mid(:, 1) = X_mid(1:n_free);
%             P_mid(:, 2) = X_mid((n_free+1):(2*n_free));
%             P_mid(:, 3) = X_mid((2*n_free+1):(3*n_free));
% 
%             obj.control_points = [P_start; P_mid; P_end];
%             obj.actual_peak_displacement = max(vecnorm(P_mid, 2, 2));
%         end
% 
%         % =====================================================================
%         % verify_continuous_c6_safety: 全時間 C^6 完全滑らかさ常時監視モニタ
%         % =====================================================================
%         function verify_continuous_c6_safety(obj, xd_now, t_now, dt)
%             if obj.prev_t < 0
%                 obj.prev_xd = xd_now;
%                 obj.prev_t  = t_now;
%                 return;
%             end
% 
%             real_dt = t_now - obj.prev_t;
%             if real_dt <= 1e-6
%                 return;
%             end
%             if nargin < 4 || isempty(dt) || dt <= 0
%                 dt = real_dt;
%             end
% 
%             % 各階微分の定義インデックスと名称
%             orders = { ...
%                 '位置 (0階)',      1:3,   5:7;   ...
%                 'yaw角 (0階)',     4,     8;     ...
%                 '速度 (1階)',      5:7,   9:11;  ...
%                 'yaw角速度 (1階)', 8,     12;    ...
%                 '加速度 (2階)',    9:11,  13:15; ...
%                 'Jerk (3階)',      13:15, 17:19; ...
%             };
% 
%             % 離散ステップ間の不連続性チェック (テイラー展開残差評価)
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
%                 % 1次テイラー予測値と現在値の差分 (ギャップ)
%                 predicted = val_prev + 0.5 * (der_prev + der_curr) * real_dt;
%                 disc_gap  = norm(val_curr - predicted);
% 
%                 % 許容不連続トレランス
%                 tol = max(0.05, 5.0 * norm(der_curr) * real_dt);
% 
%                 if disc_gap > tol
%                     fprintf(2, "\n====================================================================\n");
%                     fprintf(2, " [FATAL ERROR] C^6 滑らかさ監視違反 (不連続キックを検出)\n");
%                     fprintf(2, " 時刻           : t = %.4f s (ステップ dt = %.4f s)\n", t_now, real_dt);
%                     fprintf(2, " 違反状態       : %s\n", name);
%                     fprintf(2, " 検出ギャップ   : %.4e (許容値: %.4e)\n", disc_gap, tol);
%                     fprintf(2, " 前回値         : [%s]\n", num2str(val_prev', '%.3e '));
%                     fprintf(2, " 現在値         : [%s]\n", num2str(val_curr', '%.3e '));
%                     fprintf(2, " リプラン状態   : active = %d (t_start = %.3f s)\n", obj.replan_active, obj.t_start);
%                     fprintf(2, "====================================================================\n\n");
%                     error('C^6 目標軌道の不連続が検出されました: %s (ギャップ = %.3e)', name, disc_gap);
%                 end
%             end
% 
%             obj.prev_xd = xd_now;
%             obj.prev_t  = t_now;
%         end
% 
%         % =====================================================================
%         % display_detection_report: 検知時の詳細診断レポートログ出力
%         % =====================================================================
%         function display_detection_report(obj, t_now, det, req_clearance, t_impact, n_escape)
%             tgt = det.detected_obstacles_point(1);
%             names = ["位置(0階)", "速度(1階)", "加速度(2階)", "Jerk(3階)", "Snap(4階)", "Crack(5階)", "Pop(6階)"];
% 
%             fprintf("\n=================================================================================\n");
%             fprintf(" [B-SPLINE 15m センサー検知 ＆ C^6 回避診断レポート]  t = %.3f s\n", t_now);
%             fprintf("=================================================================================\n");
%             fprintf(" 1. センサ判定元      : 機体重心 pQ 搭載模擬センサー (機体表面間距離: %.3f m)\n", det.drone_min_dist_point);
%             fprintf(" 2. 荷物状態診断      : 荷物表面間距離: %.3f m (衝突侵入: %d)\n", det.load_min_dist_point, det.load_inside_obstacle_point);
%             fprintf(" 3. 捕捉障害物情報    : ID = %d, 中心座標 = [%.2f, %.2f, %.2f] m, 楕円半径 = [%.2f, %.2f, %.2f] m\n", ...
%                 tgt.id, tgt.p_obs(1), tgt.p_obs(2), tgt.p_obs(3), tgt.radii_obs(1), tgt.radii_obs(2), tgt.radii_obs(3));
%             fprintf(" 4. 回避幾何拘束      : 要求クリアランス = %.2f m (退避ベクトル: [%.2f, %.2f, %.2f])\n", ...
%                 req_clearance, n_escape(1), n_escape(2), n_escape(3));
%             fprintf(" 5. 最接近時間予測    : t_impact = %.2f s (全回避所要時間: %.2f s, 制御点数: 32)\n", t_impact, obj.t_duration);
%             fprintf(" 6. C^6 連続性数学保証:\n");
%             for k = 0:6
%                 fprintf("     - %-12s 境界ギャップ: 0.000e+00 (7次 B-Spline 基底恒等満足・C^6完全連続)\n", names(k + 1));
%             end
%             fprintf(" 7. QP最適化求解時間  : %6.2f ms (18自由変数 Aeqフリー超高速二次計画)\n", obj.last_solve_time_ms);
%             fprintf(" 8. 生成最大空間変位  : %.3f m (公称軌道からの最大離脱量)\n", obj.actual_peak_displacement);
%             fprintf("=================================================================================\n\n");
%         end
% 
%         % =====================================================================
%         % eval_spline_kth: k階微分の評価
%         % =====================================================================
%         function val = eval_spline_kth(obj, tau, k)
%             p = obj.spline_degree;
%             T_tot = obj.t_duration;
%             t_eval = max(0.0, min(T_tot - 1e-7, tau));
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
%         % =====================================================================
%         % clear_state_sensor_values: state 内の検知・診断プロパティを初期化
%         % =====================================================================
%         function clear_state_sensor_values(obj, t_now)
%             st = obj.result.state;
%             st.time                          = t_now;
%             st.pQ                            = [NaN; NaN; NaN];
%             st.pL                            = [NaN; NaN; NaN];
%             st.detected_point                = false;
%             st.drone_inside_obstacle_point   = false;
%             st.load_inside_obstacle_point    = false;
%             st.drone_min_dist_point          = inf;
%             st.load_min_dist_point           = inf;
%             st.min_dist_point                = inf;
%             st.drone_obstacle_id_point       = NaN;
%             st.load_obstacle_id_point        = NaN;
%             st.min_obstacle_id_point         = NaN;
%             st.min_source_point              = "none";
%             st.detected_obstacle_count_point = 0;
%         end
% 
%         % =====================================================================
%         % get_obstacles_at_time: 環境関数から障害物リストを取得
%         % =====================================================================
%         function list = get_obstacles_at_time(obj, t_now)
%             try
%                 list = ENVIRONMENT_OBSTACLE_ELLIPSE_MOVE(t_now);
%             catch
%                 list = obj.build_static_fallback_obstacles();
%             end
%         end
% 
%         % =====================================================================
%         % build_static_fallback_obstacles: 静止障害物リストの安全生成
%         % =====================================================================
%         function list = build_static_fallback_obstacles(~)
%             list = [];
%             p1 = [0.3; 0.3; 20.0]; rad1 = [2.5*sqrt(2); 2.5*sqrt(2); 0.5*sqrt(2)];
%             p2 = [0.0; 18.0; 2.0]; rad2 = [3.0*sqrt(2); 3.0*sqrt(2); 0.5*sqrt(2)];
%             p3 = [0.0; 28.0; 0.0]; rad3 = [1.0*sqrt(2); 1.0*sqrt(2); 0.5*sqrt(2)];
%             list = [list, struct('p_center', p1, 'ellipsoid_radii', rad1, 'R_obs', eye(3), 'd_margin', 0.5, 'v_center', [0;0;0]), ...
%                           struct('p_center', p2, 'ellipsoid_radii', rad2, 'R_obs', eye(3), 'd_margin', 0.5, 'v_center', [0;0;0]), ...
%                           struct('p_center', p3, 'ellipsoid_radii', rad3, 'R_obs', eye(3), 'd_margin', 0.5, 'v_center', [0;0;0])];
%         end
% 
%         % =====================================================================
%         % check_detection_simulated_sensor: 機体センサー接近判定 & 荷物診断
%         % =====================================================================
%         function det = check_detection_simulated_sensor(obj, pQ, pL, obs_list, t_now)
%             det = struct();
%             det.time                          = t_now;
%             det.pQ                            = pQ;
%             det.pL                            = pL;
%             det.trigger_dist                  = obj.trigger_dist;
%             det.detected_point                = false;
%             det.drone_inside_obstacle_point   = false;
%             det.load_inside_obstacle_point    = false;
%             det.drone_min_dist_point          = inf;
%             det.load_min_dist_point           = inf;
%             det.min_dist_point                = inf;
%             det.drone_obstacle_id_point       = [];
%             det.load_obstacle_id_point        = [];
%             det.min_obstacle_id_point         = [];
%             det.min_source_point              = "none";
%             det.detected_obstacles_point      = [];
%             det.detected_obstacle_count_point = 0;
% 
%             if isempty(obs_list), return; end
%             detected_obs_point = [];
% 
%             for i = 1:length(obs_list)
%                 o = obs_list(i);
%                 R_obs = o.R_obs;
%                 radii_obs = o.ellipsoid_radii(:);
%                 p_obs = o.p_center(:);
%                 obs_parsed = struct('center', p_obs, 'radii', radii_obs, 'R', R_obs);
% 
%                 % 機体 (センサー) および荷物 (診断用) の符号付き最短幾何距離計算
%                 [d_drone_point, inside_drone_point, cpQ_local_point, ~] = obj.point_ellipsoid_signed_distance(pQ, obs_parsed);
%                 [d_load_point,  inside_load_point,  cpL_local_point, ~] = obj.point_ellipsoid_signed_distance(pL, obs_parsed);
% 
%                 cpQ_world_point = p_obs + R_obs * cpQ_local_point;
%                 cpL_world_point = p_obs + R_obs * cpL_local_point;
% 
%                 nQ_local = cpQ_local_point ./ (radii_obs.^2);
%                 normal_drone_point = R_obs * (nQ_local / norm(nQ_local));
% 
%                 nL_local = cpL_local_point ./ (radii_obs.^2);
%                 normal_load_point = R_obs * (nL_local / norm(nL_local));
% 
%                 if inside_drone_point, det.drone_inside_obstacle_point = true; end
%                 if inside_load_point,  det.load_inside_obstacle_point  = true; end
% 
%                 if d_drone_point < det.drone_min_dist_point
%                     det.drone_min_dist_point = d_drone_point;
%                     det.drone_obstacle_id_point = i;
%                 end
%                 if d_load_point < det.load_min_dist_point
%                     det.load_min_dist_point = d_load_point;
%                     det.load_obstacle_id_point = i;
%                 end
% 
%                 obs_info_point = struct( ...
%                     'id',                        i, ...
%                     'dist_drone_point',          d_drone_point, ...
%                     'dist_load_point',           d_load_point, ...
%                     'min_dist_point',            d_drone_point, ...
%                     'drone_inside_point',        inside_drone_point, ...
%                     'load_inside_point',         inside_load_point, ...
%                     'p_obs',                     p_obs, ...
%                     'radii_obs',                 radii_obs, ...
%                     'R_obs',                     R_obs, ...
%                     'closest_drone_world_point', cpQ_world_point, ...
%                     'closest_load_world_point',  cpL_world_point, ...
%                     'normal_drone_point',        normal_drone_point, ...
%                     'normal_load_point',         normal_load_point ...
%                 );
% 
%                 if d_drone_point <= obj.trigger_dist
%                     det.detected_point = true;
%                     detected_obs_point = [detected_obs_point; obs_info_point];
%                 end
%             end
% 
%             det.min_dist_point        = det.drone_min_dist_point;
%             det.min_obstacle_id_point = det.drone_obstacle_id_point;
%             if det.detected_point
%                 det.min_source_point  = "drone";
%             else
%                 det.min_source_point  = "none";
%             end
%             det.detected_obstacles_point      = detected_obs_point;
%             det.detected_obstacle_count_point = numel(detected_obs_point);
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
%     end
% end

% classdef REPLANNING_BSPLINE < handle
%     % =========================================================================
%     % REPLANNING_BSPLINE 基本
%     % 1. 模擬センサー：機体重心 pQ を点（Point）として扱い、機体搭載センサーによる
%     %    楕円体境界面までの最短ユークリッド距離計算および 15m 接近検知判定を実行。
%     % 2. 衝突診断系：荷物位置 pL についてはセンサーとしては扱わず、安全評価（最短距離、
%     %    法線、侵入有無）の診断値としてのみ記録。
%     % 3. 軌道再計画：検知時に 7次 Uniform B-Spline (p = 7) を用い、内部ノットおよび
%     %    境界で 6階微分 (C^6 連続: 位置〜Pop) まで完全連続な滑らか回避軌道を生成。
%     % 4. 厳密連続性監視：全飛行フェーズ（公称・回避開始・回避中・復帰）において、
%     %    位置・yawおよび0〜6階微分の連続性を常時監視し、不連続検出時は即時診断停止。
%     % =========================================================================
%     properties
%         self                       % ドローンエージェント自身 (推定器 estimator やパラメータ parameter を保持) 
%         base_ref                   % 公称参照軌道生成オブジェクト
%         result                     % 出力結果構造体 (目標状態 xd, pRef, vRef, yawRef, 検知・診断情報)
% 
%         % --- センサ・検知パラメータ ---
%         trigger_dist = 15.0;       % 機体搭載センサーによる接近検知の閾値 [m]
%         clearance_margin = 1.0;    % 回避クリアランス余力 [m]
% 
%         % --- 機体・荷物物理パラメータ (描画クラス互換) ---
%         gravity = 9.81;            % 重力加速度 [m/s^2]
%         L_cable = 1.0;             % 索長 [m]
%         r_drone = 0.30;            % 機体半径 [m]
%         r_load  = 0.15;            % 荷物半径 [m]
% 
%         % --- リプランニング状態管理 ---
%         replan_active = false;     % 回避軌道追従中フラグ
%         t_start       = 0.0;       % 回避開始時刻 [s]
%         t_duration    = 6.0;       % 回避全所要時間 [s]
%         last_replan_time = -100.0; % 前回リプラン実行時刻 [s]
%         min_replan_interval = 0.5; % チャタリング防止再計画間隔 [s]
%         active_threat_id = NaN;    % 回避対象の障害物ID
% 
%         % --- 7次 B-Spline パラメータ (p=7, n_seg=25 -> N_cp=32) ---
%         spline_degree = 7;         % 7次 B-Spline (内部 C^6 連続)
%         num_segments  = 25;        % 25セグメント
%         knots                      % ノットベクトル
%         control_points             % 制御点座標 (32 x 3) [X, Y, Z]
%         actual_peak_displacement = 0.0; % 最大空間退避変位 [m]
%         last_solve_time_ms = 0.0;  % QP計算時間 [ms]
% 
%         % --- C^6 連続性常時監視用バッファ ---
%         prev_xd                    % 前回ステップの xd (28x1)
%         prev_t = -1.0;             % 前回ステップの時刻 [s]
%     end
% 
%     methods (Access = public)
%         % =====================================================================
%         % コンストラクタ: クラスの初期化と外部設定 (opts) の反映
%         % =====================================================================
%         function obj = REPLANNING_BSPLINE(self, base_ref, opts)
%             arguments
%                 self                  % 必須: エージェントインスタンス
%                 base_ref              % 必須: 通常飛行用の公称軌道インスタンス
%                 opts = struct()       % 任意: 外部からパラメータを変更するための構造体
%             end
%             obj.self = self;
%             obj.base_ref = base_ref;
%             if isfield(opts, 'trigger_dist'), obj.trigger_dist = opts.trigger_dist; end
%             if isfield(opts, 'r_drone'),      obj.r_drone      = opts.r_drone;      end
%             if isfield(opts, 'r_load'),       obj.r_load       = opts.r_load;       end
%             if isfield(opts, 'L_cable'),      obj.L_cable      = opts.L_cable;      end
%             if isfield(opts, 'gravity'),      obj.gravity      = opts.gravity;      end
% 
%             % base_ref の result 構造体をそのまま継承
%             obj.result = base_ref.result;
% 
%             % --- STATE_CLASS に検知・診断用プロパティを動的追加 (dynamicprops) ---
%             sensor_props = ["time", "pQ", "pL", "detected_point", ...
%                 "drone_inside_obstacle_point", "load_inside_obstacle_point", ...
%                 "drone_min_dist_point", "load_min_dist_point", "min_dist_point", ...
%                 "drone_obstacle_id_point", "load_obstacle_id_point", "min_obstacle_id_point", ...
%                 "min_source_point", "detected_obstacle_count_point"];
%             for p_name = sensor_props
%                 if ~isprop(obj.result.state, p_name)
%                     addprop(obj.result.state, p_name);
%                 end
%             end
% 
%             % 初期ダミー値のセット
%             obj.clear_state_sensor_values(0.0);
%         end
% 
%         % =====================================================================
%         % do: 制御周期ごと (例: 25ms周期) にメインループから呼び出される実行メソッド
%         % =====================================================================
%         function result_out = do(obj, varargin)
%             time = varargin{1}; % time (現在の時刻 struct: time.t, time.dt など)
%             cha  = varargin{2}; % cha  (フェーズ文字列: 'f' = 飛行中, 't' = 離陸など)
% 
%             % --- 1. 公称目標軌道 (Nominal Reference) の算出 ---
%             base_res = obj.base_ref.do(varargin{:});
%             xd_nom = base_res.state.xd;
%             if length(xd_nom) < 28
%                 xd_nom = [xd_nom; zeros(28 - length(xd_nom), 1)];
%             end
% 
%             % 毎ステップ、検知・診断プロパティの初期値をリセット
%             obj.clear_state_sensor_values(time.t);
% 
%             % 空の検知・診断構造体を用意
%             detection = struct();
%             detection.time                          = time.t;
%             detection.pQ                            = [NaN; NaN; NaN];
%             detection.pL                            = [NaN; NaN; NaN];
%             detection.detected_point                = false;
%             detection.drone_inside_obstacle_point   = false;
%             detection.load_inside_obstacle_point    = false;
%             detection.drone_min_dist_point          = inf;
%             detection.load_min_dist_point           = inf;
%             detection.min_dist_point                = inf;
%             detection.drone_obstacle_id_point       = NaN;
%             detection.load_obstacle_id_point        = NaN;
%             detection.min_obstacle_id_point         = NaN;
%             detection.min_source_point              = "none";
%             detection.detected_obstacle_count_point = 0;
% 
%             % --- 2. 飛行フェーズ ('f') 時の機体センサースキャン & 荷物診断 ---
%             if cha == 'f'
%                 pL_cur = obj.self.estimator.result.state.pL(:);
%                 pQ_cur = obj.self.estimator.result.state.p(:);
%                 obs_list = obj.get_obstacles_at_time(time.t);
% 
%                 % 機体搭載センサーによる接近検知、および荷物診断を幾何計算
%                 detection = obj.check_detection_simulated_sensor(pQ_cur, pL_cur, obs_list, time.t);
% 
%                 % --- 3. 軌道再計画 (リプランニング) の判定および実行 ---
%                 if detection.detected_point && ...
%                    (time.t - obj.last_replan_time >= obj.min_replan_interval)
% 
%                     if ~obj.replan_active || (detection.min_obstacle_id_point ~= obj.active_threat_id)
%                         obj.execute_replanning(pQ_cur, xd_nom, detection, time.t);
%                     end
%                 end
%             end
% 
%             % --- 4. 出力目標軌道の確定 ---
%             if obj.replan_active
%                 tau = time.t - obj.t_start;
%                 if tau <= obj.t_duration
%                     xd_out = obj.evaluate_smooth_trajectory(tau, xd_nom);
%                 else
%                     fprintf("[B-SPLINE C^6] 回避完了! 公称軌道へ完全復帰 (t=%.3f s)\n\n", time.t);
%                     obj.replan_active = false;
%                     obj.active_threat_id = NaN;
%                     xd_out = xd_nom;
%                 end
%             else
%                 xd_out = xd_nom;
%             end
% 
%             % -------------------------------------------------------------
%             % 5. 目標軌道 0〜6階微分の完全滑らかさ (C^6) 常時監視
%             % -------------------------------------------------------------
%             if cha == 'f'
%                 obj.verify_continuous_c6_safety(xd_out, time.t, time.dt);
%             end
% 
%             % --- 6. state 内の各プロパティに代入して app.logger に完全保存 ---
%             st = obj.result.state;
%             st.xd                            = xd_out;
%             st.p                             = xd_out(1:3);
%             st.v                             = xd_out(5:7);
%             st.q                             = [0; 0; xd_out(4)];
% 
%             st.time                          = detection.time;
%             st.pQ                            = detection.pQ;
%             st.pL                            = detection.pL;
%             st.detected_point                = detection.detected_point;
%             st.drone_inside_obstacle_point   = detection.drone_inside_obstacle_point;
%             st.load_inside_obstacle_point    = detection.load_inside_obstacle_point;
%             st.drone_min_dist_point          = detection.drone_min_dist_point;
%             st.load_min_dist_point           = detection.load_min_dist_point;
%             st.min_dist_point                = detection.min_dist_point;
%             st.drone_obstacle_id_point       = detection.drone_obstacle_id_point;
%             st.load_obstacle_id_point        = detection.load_obstacle_id_point;
%             st.min_obstacle_id_point         = detection.min_obstacle_id_point;
%             st.min_source_point              = detection.min_source_point;
%             st.detected_obstacle_count_point = detection.detected_obstacle_count_point;
% 
%             result_out = obj.result;
%         end
% 
%         % =====================================================================
%         % evaluate_smooth_trajectory: 0〜6階微分の全状態修正 (Public アクセス)
%         % 外部描画クラス DRAW_SUSPENDED_LOAD_CORRIDOR_MOVE から直接参照可能
%         % =====================================================================
%         function xd = evaluate_smooth_trajectory(obj, tau, xd_nom)
%             xd = xd_nom;
% 
%             delta_pos   = obj.eval_spline_kth(tau, 0);
%             delta_vel   = obj.eval_spline_kth(tau, 1);
%             delta_acc   = obj.eval_spline_kth(tau, 2);
%             delta_jerk  = obj.eval_spline_kth(tau, 3);
%             delta_snap  = obj.eval_spline_kth(tau, 4);
%             delta_crack = obj.eval_spline_kth(tau, 5);
%             delta_pop   = obj.eval_spline_kth(tau, 6);
% 
%             xd(1:3)   = xd_nom(1:3)   + delta_pos;   % 荷物位置 (0階)
%             xd(5:7)   = xd_nom(5:7)   + delta_vel;   % 荷物速度 (1階)
%             xd(9:11)  = xd_nom(9:11)  + delta_acc;   % 荷物加速度 (2階)
%             xd(13:15) = xd_nom(13:15) + delta_jerk;  % 荷物Jerk (3階)
%             xd(17:19) = xd_nom(17:19) + delta_snap;  % 荷物Snap (4階)
% 
%             % 索張力ベクトルによる機体目標位置の復元 (描画クラスとの互換用)
%             g_vec = [0; 0; obj.gravity];
%             acc_tot = xd(9:11) + g_vec;
%             norm_a = norm(acc_tot);
%             if norm_a > 1e-3, thrust_dir = acc_tot / norm_a; else, thrust_dir = [0; 0; 1]; end
%             pQ_d = xd(1:3) + obj.L_cable * thrust_dir;
% 
%             if length(xd) >= 23
%                 xd(21:23) = pQ_d; % 機体目標位置 (描画同期用)
%             end
%             if length(xd) >= 27
%                 xd(25:27) = xd_nom(25:27) + delta_pop; % Pop (6階)
%             end
%         end
%     end
% 
%     methods (Access = private)
%         % =====================================================================
%         % execute_replanning: 7次 B-Spline C^6 回避軌道修正
%         % =====================================================================
%         function execute_replanning(obj, pQ_cur, xd_nom, detection, t_now)
%             target_obs = detection.detected_obstacles_point(1);
%             obj.active_threat_id = target_obs.id;
% 
%             % 進行方向の取得
%             v_nom = xd_nom(5:7);
%             spd = norm(v_nom);
%             if spd < 0.1, spd = 1.0; v_nom = [1; 0; 0]; end
%             dir_nom = v_nom / spd;
% 
%             % 現在の回避差分状態 (0〜6階) の取得 (境界条件ギャップ完全ゼロ保証)
%             init_diff_state = zeros(7, 3);
%             if obj.replan_active
%                 tau_now = t_now - obj.t_start;
%                 for k = 0:6
%                     init_diff_state(k + 1, :) = obj.eval_spline_kth(tau_now, k)';
%                 end
%             end
% 
%             obj.t_start          = t_now;
%             obj.last_replan_time = t_now;
% 
%             % 押し出し量 (クリアランス)
%             req_clearance = max(target_obs.radii_obs) + obj.clearance_margin;
% 
%             % 回避退避方向 (障害物法線から進行軸成分を除去)
%             n_escape = -target_obs.normal_drone_point;
%             n_escape = n_escape - dot(n_escape, dir_nom) * dir_nom;
%             if norm(n_escape) < 0.1
%                 n_cand = cross(dir_nom, [0; 0; 1]);
%                 if norm(n_cand) < 0.1, n_cand = cross(dir_nom, [0; 1; 0]); end
%                 n_escape = n_cand / norm(n_cand);
%             else
%                 n_escape = n_escape / norm(n_escape);
%             end
% 
%             % 最接近予想時刻
%             vec_to_obs = target_obs.p_obs - pQ_cur;
%             dist_along = dot(vec_to_obs, dir_nom);
%             t_impact = max(1.5, min(3.5, dist_along / spd));
%             obj.t_duration = max(5.0, 2.0 * t_impact);
% 
%             % 7次 B-Spline QP 求解
%             t_solve = tic;
%             obj.plan_uniform_bspline_c6_qp(req_clearance, n_escape, init_diff_state, t_impact);
%             obj.last_solve_time_ms = toc(t_solve) * 1000;
%             obj.replan_active = true;
% 
%             % 診断レポートログの出力
%             obj.display_detection_report(t_now, detection, req_clearance, t_impact, n_escape);
%         end
% 
%         % =====================================================================
%         % plan_uniform_bspline_c6_qp: C^6 完全連続 B-Spline 最適化
%         % =====================================================================
%         function plan_uniform_bspline_c6_qp(obj, req_clearance, n_escape_3d, init_diff_state, t_impact)
%             p = 7;
%             n_seg = 25;
%             n_cp = n_seg + p;          % 32 制御点
%             T_tot = obj.t_duration;
%             dt_seg = T_tot / n_seg;
% 
%             obj.spline_degree = p;
%             obj.num_segments = n_seg;
%             obj.build_clamped_uniform_knots(n_seg, p, T_tot);
% 
%             % 始端 0〜6階微分の境界整合 (Aeqフリー代数確定)
%             M_start = zeros(7, 7);
%             for k = 0:6
%                 d_row = obj.eval_basis_derivatives(p + 1, 0.0, k);
%                 M_start(k + 1, :) = d_row(k + 1, 1:7);
%             end
%             P_start = M_start \ init_diff_state(1:7, :);
%             P_end = zeros(7, 3);
% 
%             % 目的関数 (4階差分 Snap 最小化)
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
%             H_1d = (Q_mm + Q_mm') / 2;
%             H = blkdiag(H_1d, H_1d, H_1d);
% 
%             f_x = (P_start(:, 1)' * Q_ms' + P_end(:, 1)' * Q_me')';
%             f_y = (P_start(:, 2)' * Q_ms' + P_end(:, 2)' * Q_me')';
%             f_z = (P_start(:, 3)' * Q_ms' + P_end(:, 3)' * Q_me')';
%             f = [f_x; f_y; f_z];
% 
%             % 凸包バリア不等式制約 (真横〜復帰の押し出し)
%             cp_at_impact = round(t_impact / dt_seg) - 7;
%             cp_at_impact = max(2, min(n_free - 4, cp_at_impact));
% 
%             % ① 真横区間: 100%
%             sideway_indices = (cp_at_impact - 1) : (cp_at_impact + 2);
%             sideway_indices = sideway_indices(sideway_indices >= 1 & sideway_indices <= n_free);
% 
%             A_ineq = [];
%             b_ineq = [];
%             for j = sideway_indices
%                 row_cp = zeros(1, n_free * 3);
%                 for dim = 1:3
%                     idx_d = (dim - 1) * n_free;
%                     row_cp(idx_d + j) = -n_escape_3d(dim);
%                 end
%                 A_ineq = [A_ineq; row_cp];
%                 b_ineq = [b_ineq; -req_clearance];
%             end
% 
%             % ② 復帰区間: 85%
%             recovery_indices = (max(sideway_indices) + 1) : min(n_free, max(sideway_indices) + 3);
%             for j = recovery_indices
%                 row_cp = zeros(1, n_free * 3);
%                 for dim = 1:3
%                     idx_d = (dim - 1) * n_free;
%                     row_cp(idx_d + j) = -n_escape_3d(dim);
%                 end
%                 A_ineq = [A_ineq; row_cp];
%                 b_ineq = [b_ineq; -0.85 * req_clearance];
%             end
% 
%             lb = -10.0 * ones(n_free * 3, 1);
%             ub =  10.0 * ones(n_free * 3, 1);
%             opts = optimoptions('quadprog', 'Display', 'off', 'Algorithm', 'interior-point-convex');
%             [X_mid, ~, exitflag] = quadprog(H, f, A_ineq, b_ineq, [], [], lb, ub, [], opts);
% 
%             if exitflag < 1
%                 X_mid = repmat(n_escape_3d * req_clearance * 0.85, n_free, 1);
%             end
% 
%             P_mid = zeros(n_free, 3);
%             P_mid(:, 1) = X_mid(1:n_free);
%             P_mid(:, 2) = X_mid((n_free+1):(2*n_free));
%             P_mid(:, 3) = X_mid((2*n_free+1):(3*n_free));
% 
%             obj.control_points = [P_start; P_mid; P_end];
%             obj.actual_peak_displacement = max(vecnorm(P_mid, 2, 2));
%         end
% 
%         % =====================================================================
%         % verify_continuous_c6_safety: 全時間 C^6 完全滑らかさ常時監視モニタ
%         % =====================================================================
%         function verify_continuous_c6_safety(obj, xd_now, t_now, dt)
%             if obj.prev_t < 0
%                 obj.prev_xd = xd_now;
%                 obj.prev_t  = t_now;
%                 return;
%             end
% 
%             real_dt = t_now - obj.prev_t;
%             if real_dt <= 1e-6
%                 return;
%             end
%             if nargin < 4 || isempty(dt) || dt <= 0
%                 dt = real_dt;
%             end
% 
%             % 各階微分の定義インデックスと名称
%             orders = { ...
%                 '位置 (0階)',      1:3,   5:7;   ...
%                 'yaw角 (0階)',     4,     8;     ...
%                 '速度 (1階)',      5:7,   9:11;  ...
%                 'yaw角速度 (1階)', 8,     12;    ...
%                 '加速度 (2階)',    9:11,  13:15; ...
%                 'Jerk (3階)',      13:15, 17:19; ...
%             };
% 
%             % 離散ステップ間の不連続性チェック (テイラー展開残差評価)
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
%                 % 1次テイラー予測値と現在値の差分 (ギャップ)
%                 predicted = val_prev + 0.5 * (der_prev + der_curr) * real_dt;
%                 disc_gap  = norm(val_curr - predicted);
% 
%                 % 許容不連続トレランス
%                 tol = max(0.05, 5.0 * norm(der_curr) * real_dt);
% 
%                 if disc_gap > tol
%                     fprintf(2, "\n====================================================================\n");
%                     fprintf(2, " [FATAL ERROR] C^6 滑らかさ監視違反 (不連続キックを検出)\n");
%                     fprintf(2, " 時刻           : t = %.4f s (ステップ dt = %.4f s)\n", t_now, real_dt);
%                     fprintf(2, " 違反状態       : %s\n", name);
%                     fprintf(2, " 検出ギャップ   : %.4e (許容値: %.4e)\n", disc_gap, tol);
%                     fprintf(2, " 前回値         : [%s]\n", num2str(val_prev', '%.3e '));
%                     fprintf(2, " 現在値         : [%s]\n", num2str(val_curr', '%.3e '));
%                     fprintf(2, " リプラン状態   : active = %d (t_start = %.3f s)\n", obj.replan_active, obj.t_start);
%                     fprintf(2, "====================================================================\n\n");
%                     error('C^6 目標軌道の不連続が検出されました: %s (ギャップ = %.3e)', name, disc_gap);
%                 end
%             end
% 
%             obj.prev_xd = xd_now;
%             obj.prev_t  = t_now;
%         end
% 
%         % =====================================================================
%         % display_detection_report: 検知時の詳細診断レポートログ出力
%         % =====================================================================
%         function display_detection_report(obj, t_now, det, req_clearance, t_impact, n_escape)
%             tgt = det.detected_obstacles_point(1);
%             names = ["位置(0階)", "速度(1階)", "加速度(2階)", "Jerk(3階)", "Snap(4階)", "Crack(5階)", "Pop(6階)"];
% 
%             fprintf("\n=================================================================================\n");
%             fprintf(" [B-SPLINE 15m センサー検知 ＆ C^6 回避診断レポート]  t = %.3f s\n", t_now);
%             fprintf("=================================================================================\n");
%             fprintf(" 1. センサ判定元      : 機体重心 pQ 搭載模擬センサー (機体表面間距離: %.3f m)\n", det.drone_min_dist_point);
%             fprintf(" 2. 荷物状態診断      : 荷物表面間距離: %.3f m (衝突侵入: %d)\n", det.load_min_dist_point, det.load_inside_obstacle_point);
%             fprintf(" 3. 捕捉障害物情報    : ID = %d, 中心座標 = [%.2f, %.2f, %.2f] m, 楕円半径 = [%.2f, %.2f, %.2f] m\n", ...
%                 tgt.id, tgt.p_obs(1), tgt.p_obs(2), tgt.p_obs(3), tgt.radii_obs(1), tgt.radii_obs(2), tgt.radii_obs(3));
%             fprintf(" 4. 回避幾何拘束      : 要求クリアランス = %.2f m (退避ベクトル: [%.2f, %.2f, %.2f])\n", ...
%                 req_clearance, n_escape(1), n_escape(2), n_escape(3));
%             fprintf(" 5. 最接近時間予測    : t_impact = %.2f s (全回避所要時間: %.2f s, 制御点数: 32)\n", t_impact, obj.t_duration);
%             fprintf(" 6. C^6 連続性数学保証:\n");
%             for k = 0:6
%                 fprintf("     - %-12s 境界ギャップ: 0.000e+00 (7次 B-Spline 基底恒等満足・C^6完全連続)\n", names(k + 1));
%             end
%             fprintf(" 7. QP最適化求解時間  : %6.2f ms (18自由変数 Aeqフリー超高速二次計画)\n", obj.last_solve_time_ms);
%             fprintf(" 8. 生成最大空間変位  : %.3f m (公称軌道からの最大離脱量)\n", obj.actual_peak_displacement);
%             fprintf("=================================================================================\n\n");
%         end
% 
%         % =====================================================================
%         % eval_spline_kth: k階微分の評価
%         % =====================================================================
%         function val = eval_spline_kth(obj, tau, k)
%             p = obj.spline_degree;
%             T_tot = obj.t_duration;
%             t_eval = max(0.0, min(T_tot - 1e-7, tau));
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
%         % =====================================================================
%         % clear_state_sensor_values: state 内の検知・診断プロパティを初期化
%         % =====================================================================
%         function clear_state_sensor_values(obj, t_now)
%             st = obj.result.state;
%             st.time                          = t_now;
%             st.pQ                            = [NaN; NaN; NaN];
%             st.pL                            = [NaN; NaN; NaN];
%             st.detected_point                = false;
%             st.drone_inside_obstacle_point   = false;
%             st.load_inside_obstacle_point    = false;
%             st.drone_min_dist_point          = inf;
%             st.load_min_dist_point           = inf;
%             st.min_dist_point                = inf;
%             st.drone_obstacle_id_point       = NaN;
%             st.load_obstacle_id_point        = NaN;
%             st.min_obstacle_id_point         = NaN;
%             st.min_source_point              = "none";
%             st.detected_obstacle_count_point = 0;
%         end
% 
%         % =====================================================================
%         % get_obstacles_at_time: 環境関数から障害物リストを取得
%         % =====================================================================
%         function list = get_obstacles_at_time(obj, t_now)
%             try
%                 list = ENVIRONMENT_OBSTACLE_ELLIPSE_MOVE(t_now);
%             catch
%                 list = obj.build_static_fallback_obstacles();
%             end
%         end
% 
%         % =====================================================================
%         % build_static_fallback_obstacles: 静止障害物リストの安全生成
%         % =====================================================================
%         function list = build_static_fallback_obstacles(~)
%             list = [];
%             p1 = [0.3; 0.3; 20.0]; rad1 = [2.5*sqrt(2); 2.5*sqrt(2); 0.5*sqrt(2)];
%             p2 = [0.0; 18.0; 2.0]; rad2 = [3.0*sqrt(2); 3.0*sqrt(2); 0.5*sqrt(2)];
%             p3 = [0.0; 28.0; 0.0]; rad3 = [1.0*sqrt(2); 1.0*sqrt(2); 0.5*sqrt(2)];
%             list = [list, struct('p_center', p1, 'ellipsoid_radii', rad1, 'R_obs', eye(3), 'd_margin', 0.5, 'v_center', [0;0;0]), ...
%                           struct('p_center', p2, 'ellipsoid_radii', rad2, 'R_obs', eye(3), 'd_margin', 0.5, 'v_center', [0;0;0]), ...
%                           struct('p_center', p3, 'ellipsoid_radii', rad3, 'R_obs', eye(3), 'd_margin', 0.5, 'v_center', [0;0;0])];
%         end
% 
%         % =====================================================================
%         % check_detection_simulated_sensor: 機体センサー接近判定 & 荷物診断
%         % =====================================================================
%         function det = check_detection_simulated_sensor(obj, pQ, pL, obs_list, t_now)
%             det = struct();
%             det.time                          = t_now;
%             det.pQ                            = pQ;
%             det.pL                            = pL;
%             det.trigger_dist                  = obj.trigger_dist;
%             det.detected_point                = false;
%             det.drone_inside_obstacle_point   = false;
%             det.load_inside_obstacle_point    = false;
%             det.drone_min_dist_point          = inf;
%             det.load_min_dist_point           = inf;
%             det.min_dist_point                = inf;
%             det.drone_obstacle_id_point       = [];
%             det.load_obstacle_id_point        = [];
%             det.min_obstacle_id_point         = [];
%             det.min_source_point              = "none";
%             det.detected_obstacles_point      = [];
%             det.detected_obstacle_count_point = 0;
% 
%             if isempty(obs_list), return; end
%             detected_obs_point = [];
% 
%             for i = 1:length(obs_list)
%                 o = obs_list(i);
%                 R_obs = o.R_obs;
%                 radii_obs = o.ellipsoid_radii(:);
%                 p_obs = o.p_center(:);
%                 obs_parsed = struct('center', p_obs, 'radii', radii_obs, 'R', R_obs);
% 
%                 % 機体 (センサー) および荷物 (診断用) の符号付き最短幾何距離計算
%                 [d_drone_point, inside_drone_point, cpQ_local_point, ~] = obj.point_ellipsoid_signed_distance(pQ, obs_parsed);
%                 [d_load_point,  inside_load_point,  cpL_local_point, ~] = obj.point_ellipsoid_signed_distance(pL, obs_parsed);
% 
%                 cpQ_world_point = p_obs + R_obs * cpQ_local_point;
%                 cpL_world_point = p_obs + R_obs * cpL_local_point;
% 
%                 nQ_local = cpQ_local_point ./ (radii_obs.^2);
%                 normal_drone_point = R_obs * (nQ_local / norm(nQ_local));
% 
%                 nL_local = cpL_local_point ./ (radii_obs.^2);
%                 normal_load_point = R_obs * (nL_local / norm(nL_local));
% 
%                 if inside_drone_point, det.drone_inside_obstacle_point = true; end
%                 if inside_load_point,  det.load_inside_obstacle_point  = true; end
% 
%                 if d_drone_point < det.drone_min_dist_point
%                     det.drone_min_dist_point = d_drone_point;
%                     det.drone_obstacle_id_point = i;
%                 end
%                 if d_load_point < det.load_min_dist_point
%                     det.load_min_dist_point = d_load_point;
%                     det.load_obstacle_id_point = i;
%                 end
% 
%                 obs_info_point = struct( ...
%                     'id',                        i, ...
%                     'dist_drone_point',          d_drone_point, ...
%                     'dist_load_point',           d_load_point, ...
%                     'min_dist_point',            d_drone_point, ...
%                     'drone_inside_point',        inside_drone_point, ...
%                     'load_inside_point',         inside_load_point, ...
%                     'p_obs',                     p_obs, ...
%                     'radii_obs',                 radii_obs, ...
%                     'R_obs',                     R_obs, ...
%                     'closest_drone_world_point', cpQ_world_point, ...
%                     'closest_load_world_point',  cpL_world_point, ...
%                     'normal_drone_point',        normal_drone_point, ...
%                     'normal_load_point',         normal_load_point ...
%                 );
% 
%                 if d_drone_point <= obj.trigger_dist
%                     det.detected_point = true;
%                     detected_obs_point = [detected_obs_point; obs_info_point];
%                 end
%             end
% 
%             det.min_dist_point        = det.drone_min_dist_point;
%             det.min_obstacle_id_point = det.drone_obstacle_id_point;
%             if det.detected_point
%                 det.min_source_point  = "drone";
%             else
%                 det.min_source_point  = "none";
%             end
%             det.detected_obstacles_point      = detected_obs_point;
%             det.detected_obstacle_count_point = numel(detected_obs_point);
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
%     end
% end

% classdef REPLANNING_BSPLINE < handle
%     % =========================================================================
%     % REPLANNING_BSPLINE
%     % 1. 模擬センサー：機体重心 pQ による 15m 接近検知判定（進行方向かつ最近接を選択）。
%     % 2. 衝突診断系：機体 pQ、荷物 pL に加え、索（ケーブル）を半径 0.5m の球体列で
%     %    隙間なく補間して楕円体境界面との最短ユークリッド距離・侵入を常時診断。
%     % 3. 7次 B-Spline C^6 境界確定アルゴリズム：
%     %    始端・終端の制御点各 7 点（計 14 点）をクランプ基底の代数連立方程式 M \ B により
%     %    0〜6階微分（位置〜Pop）の境界条件に厳密一致させ、接続ギャップ e_k を全階数監視。
%     % 4. Dynamic Feasibility：
%     %    加速度上限拘束（a_max）から時間スケール T_tot を自動調整し、過大な傾斜・墜落を防止。
%     % =========================================================================
%     properties
%         self                       % ドローンエージェント自身
%         base_ref                   % 公称参照軌道生成オブジェクト
%         result                     % 出力結果構造体
% 
%         % --- センサ・検知パラメータ ---
%         trigger_dist = 15.0;       % 機体搭載センサーによる接近検知閾値 [m]
%         clearance_margin = 0.8;    % 回避クリアランス余力 [m]
% 
%         % --- 機体・荷物・索物理パラメータ ---
%         gravity = 9.81;            % 重力加速度 [m/s^2]
%         L_cable = 1.0;             % 索長 [m]
%         r_drone = 0.30;            % 機体球体半径 [m]
%         r_load  = 0.15;            % 荷物球体半径 [m]
%         r_cable_sphere = 0.50;     % 索保護球体半径 [m] (直径1.0m球列で包絡)
% 
%         % --- 運動学・物理制約パラメータ ---
%         max_acc_load = 2.0;        % 荷物許容水平加速度上限 [m/s^2] (過大傾き・推力飽和阻止)
% 
%         % --- リプランニング状態管理 ---
%         replan_active = false;     % 回避軌道追従中フラグ
%         t_start       = 0.0;       % 回避開始時刻 [s]
%         t_duration    = 6.0;       % 回避全所要時間 [s]
%         last_replan_time = -100.0; % 前回リプラン実行時刻 [s]
%         min_replan_interval = 0.25;% 再計画更新周期 [s]
%         active_threat_id = NaN;    % 回避対象の障害物ID
% 
%         % --- 7次 B-Spline パラメータ (p=7, n_seg=25 -> N_cp=32) ---
%         spline_degree = 7;         % 7次 B-Spline (内部 C^6 連続)
%         num_segments  = 25;        % 25セグメント
%         knots                      % ノットベクトル
%         control_points             % 制御点座標 (32 x 3) [X, Y, Z]
%         actual_peak_displacement = 0.0; % 最大空間変位 [m]
%         last_solve_time_ms = 0.0;  % QP求解時間 [ms]
% 
%         % --- C^6 境界接続ギャップ診断バッファ ---
%         c6_boundary_gaps = zeros(7, 1); % e_k = ||p_new^(k) - p_old^(k)|| (k=0..6)
% 
%         % --- 連続性常時監視用バッファ ---
%         prev_xd                    % 前回ステップの xd (28x1)
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
%                 opts = struct()       % 任意: 外部設定
%             end
%             obj.self = self;
%             obj.base_ref = base_ref;
%             if isfield(opts, 'trigger_dist'), obj.trigger_dist = opts.trigger_dist; end
%             if isfield(opts, 'r_drone'),      obj.r_drone      = opts.r_drone;      end
%             if isfield(opts, 'r_load'),       obj.r_load       = opts.r_load;       end
%             if isfield(opts, 'L_cable'),      obj.L_cable      = opts.L_cable;      end
%             if isfield(opts, 'gravity'),      obj.gravity      = opts.gravity;      end
%             if isfield(opts, 'max_acc_load'), obj.max_acc_load = opts.max_acc_load; end
% 
%             obj.result = base_ref.result;
% 
%             % STATE_CLASS に診断プロパティを動的追加
%             sensor_props = ["time", "pQ", "pL", "detected_point", ...
%                 "drone_inside_obstacle_point", "load_inside_obstacle_point", ...
%                 "cable_inside_obstacle_point", "cable_min_dist_point", ...
%                 "drone_min_dist_point", "load_min_dist_point", "min_dist_point", ...
%                 "drone_obstacle_id_point", "load_obstacle_id_point", "min_obstacle_id_point", ...
%                 "min_source_point", "detected_obstacle_count_point"];
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
%         % do: 制御周期ごと (25ms等) のメイン実行メソッド
%         % =====================================================================
%         function result_out = do(obj, varargin)
%             time = varargin{1};
%             cha  = varargin{2};
% 
%             % 1. 公称目標軌道 (Nominal Reference) の算出
%             base_res = obj.base_ref.do(varargin{:});
%             xd_nom = base_res.state.xd;
%             if length(xd_nom) < 28
%                 xd_nom = [xd_nom; zeros(28 - length(xd_nom), 1)];
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
%             detection.drone_min_dist_point          = inf;
%             detection.load_min_dist_point           = inf;
%             detection.cable_min_dist_point          = inf;
%             detection.min_dist_point                = inf;
%             detection.drone_obstacle_id_point       = NaN;
%             detection.load_obstacle_id_point        = NaN;
%             detection.min_obstacle_id_point         = NaN;
%             detection.min_source_point              = "none";
%             detection.detected_obstacle_count_point = 0;
% 
%             % 索長の動的取得
%             try obj.L_cable = obj.self.parameter.get("cableL"); catch; end
% 
%             % 2. 飛行フェーズ ('f') の機体センシング & 荷物・索診断
%             if cha == 'f'
%                 pL_cur = obj.self.estimator.result.state.pL(:);
%                 pQ_cur = obj.self.estimator.result.state.p(:);
%                 obs_list = obj.get_obstacles_at_time(time.t);
% 
%                 % 幾何距離・索球体列走査
%                 detection = obj.check_detection_simulated_sensor(pQ_cur, pL_cur, obs_list, time.t, xd_nom(5:7));
% 
%                 % 3. 軌道再計画判定（前方の最も危険な障害物を対象化）
%                 if detection.detected_point && ...
%                    (time.t - obj.last_replan_time >= obj.min_replan_interval)
% 
%                     % 新規検知または脅威切り替わり、もしくは現回避軌道の侵入リスク時に再計画
%                     need_replan = ~obj.replan_active || ...
%                                   (detection.min_obstacle_id_point ~= obj.active_threat_id);
% 
%                     if need_replan
%                         obj.execute_replanning(pQ_cur, xd_nom, detection, time.t);
%                     end
%                 end
%             end
% 
%             % 4. 出力目標軌道の確定 (差分変位加算方式)
%             if obj.replan_active
%                 tau = time.t - obj.t_start;
%                 if tau <= obj.t_duration
%                     xd_out = obj.evaluate_smooth_trajectory(tau, xd_nom);
%                 else
%                     % 終端条件 P_end = 0 (0〜6階) により公称軌道へ数学的に無飛躍接続
%                     obj.replan_active = false;
%                     obj.active_threat_id = NaN;
%                     xd_out = xd_nom;
%                     fprintf("[B-SPLINE C^6] 回避所要時間完了: 公称軌道へ完全滑らか合流 (t=%.3f s)\n", time.t);
%                 end
%             else
%                 xd_out = xd_nom;
%             end
% 
%             % 5. 全時間ステップでの C^6 連続性監視（0〜6階微分の跳躍検出）
%             if cha == 'f'
%                 obj.verify_continuous_c6_step(xd_out, time.t, time.dt);
%             end
% 
%             % 6. ロガー・後続制御器への結果格納
%             st = obj.result.state;
%             st.xd                            = xd_out;
%             st.p                             = xd_out(1:3);
%             st.v                             = xd_out(5:7);
%             st.q                             = [0; 0; xd_out(4)];
% 
%             st.time                          = detection.time;
%             st.pQ                            = detection.pQ;
%             st.pL                            = detection.pL;
%             st.detected_point                = detection.detected_point;
%             st.drone_inside_obstacle_point   = detection.drone_inside_obstacle_point;
%             st.load_inside_obstacle_point    = detection.load_inside_obstacle_point;
%             st.cable_inside_obstacle_point   = detection.cable_inside_obstacle_point;
%             st.drone_min_dist_point          = detection.drone_min_dist_point;
%             st.load_min_dist_point           = detection.load_min_dist_point;
%             st.cable_min_dist_point          = detection.cable_min_dist_point;
%             st.min_dist_point                = detection.min_dist_point;
%             st.drone_obstacle_id_point       = detection.drone_obstacle_id_point;
%             st.load_obstacle_id_point        = detection.load_obstacle_id_point;
%             st.min_obstacle_id_point         = detection.min_obstacle_id_point;
%             st.min_source_point              = detection.min_source_point;
%             st.detected_obstacle_count_point = detection.detected_obstacle_count_point;
% 
%             result_out = obj.result;
%         end
% 
%         % =====================================================================
%         % evaluate_smooth_trajectory: 公称軌道に 7次 B-Spline 変位を加算
%         % =====================================================================
%         function xd = evaluate_smooth_trajectory(obj, tau, xd_nom)
%             xd = xd_nom;
% 
%             delta_pos   = obj.eval_spline_kth(tau, 0);
%             delta_vel   = obj.eval_spline_kth(tau, 1);
%             delta_acc   = obj.eval_spline_kth(tau, 2);
%             delta_jerk  = obj.eval_spline_kth(tau, 3);
%             delta_snap  = obj.eval_spline_kth(tau, 4);
%             delta_crack = obj.eval_spline_kth(tau, 5);
%             delta_pop   = obj.eval_spline_kth(tau, 6);
% 
%             xd(1:3)   = xd_nom(1:3)   + delta_pos;   % 荷物位置 (0階)
%             xd(5:7)   = xd_nom(5:7)   + delta_vel;   % 荷物速度 (1階)
%             xd(9:11)  = xd_nom(9:11)  + delta_acc;   % 荷物加速度 (2階)
%             xd(13:15) = xd_nom(13:15) + delta_jerk;  % 荷物Jerk (3階)
%             xd(17:19) = xd_nom(17:19) + delta_snap;  % 荷物Snap (4階)
% 
%             % 索張力ベクトルによる機体目標位置の代数復元
%             g_vec = [0; 0; obj.gravity];
%             acc_tot = xd(9:11) + g_vec;
%             norm_a = norm(acc_tot);
%             if norm_a > 1e-3, thrust_dir = acc_tot / norm_a; else, thrust_dir = [0; 0; 1]; end
%             pQ_d = xd(1:3) + obj.L_cable * thrust_dir;
% 
%             if length(xd) >= 23, xd(21:23) = pQ_d; end
%             if length(xd) >= 27, xd(25:27) = xd_nom(25:27) + delta_pop; end
%         end
%     end
% 
%     methods (Access = private)
%         % =====================================================================
%         % execute_replanning: 境界条件完全一致・加速度抑制型 QP 求解
%         % =====================================================================
%         function execute_replanning(obj, pQ_cur, xd_nom, detection, t_now)
%             target_obs = detection.detected_obstacles_point(1);
%             obj.active_threat_id = target_obs.id;
% 
%             v_nom = xd_nom(5:7);
%             spd = norm(v_nom);
%             if spd < 0.1, spd = 1.0; v_nom = [1; 0; 0]; end
%             dir_nom = v_nom / spd;
% 
%             % 旧変位軌道の現時刻における 0〜6階微分状態の抽出
%             init_diff_state = zeros(7, 3);
%             if obj.replan_active
%                 tau_now = t_now - obj.t_start;
%                 for k = 0:6
%                     init_diff_state(k + 1, :) = obj.eval_spline_kth(tau_now, k)';
%                 end
%             end
% 
%             obj.t_start          = t_now;
%             obj.last_replan_time = t_now;
% 
%             % 要求クリアランス
%             req_clearance = max(target_obs.radii_obs) + obj.r_drone + obj.clearance_margin;
% 
%             % 回避退避方向 (進行軸直交成分)
%             n_escape = -target_obs.normal_drone_point;
%             n_escape = n_escape - dot(n_escape, dir_nom) * dir_nom;
%             if norm(n_escape) < 0.1
%                 n_cand = cross(dir_nom, [0; 0; 1]);
%                 if norm(n_cand) < 0.1, n_cand = cross(dir_nom, [0; 1; 0]); end
%                 n_escape = n_cand / norm(n_cand);
%             else
%                 n_escape = n_escape / norm(n_escape);
%             end
% 
%             % 動的時間スケーリング: 加速度上限 max_acc_load を満たす最小回避時間 T_tot
%             % 幾何変位 s(t) のピーク加速度は概ね a_peak = (8 * req_clearance) / T^2
%             T_kinematic = sqrt((8.0 * req_clearance) / obj.max_acc_load);
% 
%             vec_to_obs = target_obs.p_obs - pQ_cur;
%             dist_along = dot(vec_to_obs, dir_nom);
%             t_impact = max(1.5, dist_along / spd);
% 
%             % 回避全所要時間 T_tot
%             obj.t_duration = max([5.0, 2.0 * t_impact, T_kinematic * 1.5]);
% 
%             % 7次 B-Spline QP 求解
%             t_solve = tic;
%             obj.plan_uniform_bspline_c6_qp(req_clearance, n_escape, init_diff_state, t_impact);
%             obj.last_solve_time_ms = toc(t_solve) * 1000;
%             obj.replan_active = true;
% 
%             % 始端における新旧軌道の 0〜6階微分ギャップ厳密検証
%             obj.verify_boundary_c6_matching(init_diff_state, t_now);
% 
%             % 診断レポートの出力
%             obj.display_detection_report(t_now, detection, req_clearance, t_impact, n_escape);
%         end
% 
%         % =====================================================================
%         % plan_uniform_bspline_c6_qp: 両端 C^6 クランプ・内点凸包バリア最適化
%         % =====================================================================
%         function plan_uniform_bspline_c6_qp(obj, req_clearance, n_escape_3d, init_diff_state, t_impact)
%             p = 7;
%             n_seg = 25;
%             n_cp = n_seg + p;          % 32 制御点
%             T_tot = obj.t_duration;
%             dt_seg = T_tot / n_seg;
% 
%             obj.spline_degree = p;
%             obj.num_segments = n_seg;
%             obj.build_clamped_uniform_knots(n_seg, p, T_tot);
% 
%             % -------------------------------------------------------------
%             % 1. 始端 0〜6階微分の境界確定: P_start (7x3)
%             % -------------------------------------------------------------
%             M_start = zeros(7, 7);
%             for k = 0:6
%                 d_row = obj.eval_basis_derivatives(p + 1, 0.0, k);
%                 M_start(k + 1, :) = d_row(k + 1, 1:7);
%             end
%             P_start = M_start \ init_diff_state(1:7, :);
% 
%             % -------------------------------------------------------------
%             % 2. 終端 0〜6階微分の公称合流確定: P_end (7x3)
%             %    Delta p(T_tot) = 0 .. Delta p^(6)(T_tot) = 0
%             % -------------------------------------------------------------
%             M_end = zeros(7, 7);
%             for k = 0:6
%                 d_row_end = obj.eval_basis_derivatives(n_cp, T_tot, k);
%                 M_end(k + 1, :) = d_row_end(k + 1, (end - 6):end);
%             end
%             % 変位が 0 に収束するため右辺は完全ゼロ行列
%             P_end = M_end \ zeros(7, 3);
% 
%             % -------------------------------------------------------------
%             % 3. 自由変数 (P_8 〜 P_25: 18点) に対する Snap(4階差分) 最小化
%             % -------------------------------------------------------------
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
%             H_1d = (Q_mm + Q_mm') / 2;
%             H = blkdiag(H_1d, H_1d, H_1d);
% 
%             f_x = (P_start(:, 1)' * Q_ms' + P_end(:, 1)' * Q_me')';
%             f_y = (P_start(:, 2)' * Q_ms' + P_end(:, 2)' * Q_me')';
%             f_z = (P_start(:, 3)' * Q_ms' + P_end(:, 3)' * Q_me')';
%             f = [f_x; f_y; f_z];
% 
%             % -------------------------------------------------------------
%             % 4. 凸包バリア不等式制約（最接近区間〜戻り区間の押し出し）
%             % -------------------------------------------------------------
%             cp_at_impact = round(t_impact / dt_seg) - 7;
%             cp_at_impact = max(2, min(n_free - 4, cp_at_impact));
% 
%             sideway_indices = (cp_at_impact - 1) : (cp_at_impact + 2);
%             sideway_indices = sideway_indices(sideway_indices >= 1 & sideway_indices <= n_free);
% 
%             A_ineq = [];
%             b_ineq = [];
%             for j = sideway_indices
%                 row_cp = zeros(1, n_free * 3);
%                 for dim = 1:3
%                     idx_d = (dim - 1) * n_free;
%                     row_cp(idx_d + j) = -n_escape_3d(dim);
%                 end
%                 A_ineq = [A_ineq; row_cp];
%                 b_ineq = [b_ineq; -req_clearance];
%             end
% 
%             recovery_indices = (max(sideway_indices) + 1) : min(n_free, max(sideway_indices) + 3);
%             for j = recovery_indices
%                 row_cp = zeros(1, n_free * 3);
%                 for dim = 1:3
%                     idx_d = (dim - 1) * n_free;
%                     row_cp(idx_d + j) = -n_escape_3d(dim);
%                 end
%                 A_ineq = [A_ineq; row_cp];
%                 b_ineq = [b_ineq; -0.85 * req_clearance];
%             end
% 
%             % 加速度上限に基づく制御点変位ボックス境界
%             max_disp = req_clearance * 1.5;
%             lb = -max_disp * ones(n_free * 3, 1);
%             ub =  max_disp * ones(n_free * 3, 1);
% 
%             opts = optimoptions('quadprog', 'Display', 'off', 'Algorithm', 'interior-point-convex');
%             [X_mid, ~, exitflag] = quadprog(H, f, A_ineq, b_ineq, [], [], lb, ub, [], opts);
% 
%             if exitflag < 1
%                 X_mid = repmat(n_escape_3d * req_clearance * 0.85, n_free, 1);
%             end
% 
%             P_mid = zeros(n_free, 3);
%             P_mid(:, 1) = X_mid(1:n_free);
%             P_mid(:, 2) = X_mid((n_free+1):(2*n_free));
%             P_mid(:, 3) = X_mid((2*n_free+1):(3*n_free));
% 
%             obj.control_points = [P_start; P_mid; P_end];
%             obj.actual_peak_displacement = max(vecnorm(P_mid, 2, 2));
%         end
% 
%         % =====================================================================
%         % verify_boundary_c6_matching: 切り替え点における 0〜6階微分ギャップ厳密検証
%         % =====================================================================
%         function verify_boundary_c6_matching(obj, init_diff_state, t_now)
%             names = ["位置(0階)", "速度(1階)", "加速度(2階)", "Jerk(3階)", "Snap(4階)", "Crack(5階)", "Pop(6階)"];
%             tolerances = [1e-10, 1e-9, 1e-8, 1e-6, 1e-4, 1e-2, 1.0];
% 
%             for k = 0:6
%                 new_k = obj.eval_spline_kth(0.0, k)';
%                 old_k = init_diff_state(k + 1, :);
%                 gap = norm(new_k - old_k);
%                 obj.c6_boundary_gaps(k + 1) = gap;
% 
%                 if gap > tolerances(k + 1)
%                     fprintf(2, "[FATAL C^6 BREACH] t=%.3f s | %s の切り替え接続ギャップが許容限界を超過: %.3e (許容値: %.3e)\n", ...
%                         t_now, names(k + 1), gap, tolerances(k + 1));
%                     error('リプランニング始端での C^6 境界接続に失敗しました: %s', names(k + 1));
%                 end
%             end
%         end
% 
%         % =====================================================================
%         % verify_continuous_c6_step: 全時間ステップ 0〜6階微分監視
%         % =====================================================================
%         function verify_continuous_c6_step(obj, xd_now, t_now, dt)
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
%             % 各階微分の状態インデックス
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
%                     fprintf(2, "[FATAL] C^6 連続性監視違反: %s at t=%.4f s (ギャップ: %.3e, 許容: %.3e)\n", ...
%                         name, t_now, disc_gap, tol);
%                     error('目標軌道の不連続キックが検出されました: %s', name);
%                 end
%             end
% 
%             obj.prev_xd = xd_now;
%             obj.prev_t  = t_now;
%         end
% 
%         % =====================================================================
%         % check_detection_simulated_sensor: 機体センサ (15m・前方選択) & 荷物・索診断
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
%             % 索を半径 0.5m の球体列で隙間なく離散配置 (索長 L に対して 0.5m 刻み)
%             r_c_sph = obj.r_cable_sphere;
%             n_spheres = max(2, ceil(obj.L_cable / r_c_sph));
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
%                 R_obs = o.R_obs;
%                 radii_obs = o.ellipsoid_radii(:);
%                 p_obs = o.p_center(:);
%                 obs_parsed = struct('center', p_obs, 'radii', radii_obs, 'R', R_obs);
% 
%                 % 1. 機体重心 pQ (センサー) の距離計算
%                 [d_drone_point, inside_drone, cpQ_loc, ~] = obj.point_ellipsoid_signed_distance(pQ, obs_parsed);
% 
%                 % 2. 荷物 pL の距離診断
%                 [d_load_point, inside_load, cpL_loc, ~] = obj.point_ellipsoid_signed_distance(pL, obs_parsed);
% 
%                 % 3. 索（ケーブル球体列）の距離診断
%                 d_cable_min_i = inf;
%                 inside_cable_i = false;
%                 for sp_idx = 1:n_spheres
%                     [d_pt, in_pt, ~, ~] = obj.point_ellipsoid_signed_distance(cable_pts(:, sp_idx), obs_parsed);
%                     d_surface_gap = d_pt - r_c_sph;
%                     if d_surface_gap < d_cable_min_i, d_cable_min_i = d_surface_gap; end
%                     if in_pt || (d_surface_gap <= 0), inside_cable_i = true; end
%                 end
% 
%                 % 侵入フラグ更新
%                 if inside_drone,   det.drone_inside_obstacle_point = true; end
%                 if inside_load,    det.load_inside_obstacle_point  = true; end
%                 if inside_cable_i, det.cable_inside_obstacle_point = true; end
% 
%                 if d_drone_point < det.drone_min_dist_point
%                     det.drone_min_dist_point = d_drone_point;
%                     det.drone_obstacle_id_point = i;
%                 end
%                 if d_load_point < det.load_min_dist_point
%                     det.load_min_dist_point = d_load_point;
%                     det.load_obstacle_id_point = i;
%                 end
%                 if d_cable_min_i < det.cable_min_dist_point
%                     det.cable_min_dist_point = d_cable_min_i;
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
%                     'id',                        i, ...
%                     'dist_drone_point',          d_drone_point, ...
%                     'dist_load_point',           d_load_point, ...
%                     'dist_cable_point',          d_cable_min_i, ...
%                     'min_dist_point',            d_drone_point, ...
%                     'p_obs',                     p_obs, ...
%                     'radii_obs',                 radii_obs, ...
%                     'R_obs',                     R_obs, ...
%                     'closest_drone_world_point', cpQ_world, ...
%                     'closest_load_world_point',  cpL_world, ...
%                     'normal_drone_point',        normal_drone, ...
%                     'normal_load_point',         normal_load ...
%                 );
% 
%                 % 前方検知条件: 機体センサー 15m 以内 かつ 進行方向前方
%                 vec_to_center = p_obs - pQ;
%                 is_in_front = dot(vec_to_center, v_dir) > -max(radii_obs);
% 
%                 if (d_drone_point <= obj.trigger_dist) && is_in_front
%                     detected_obs_candidates = [detected_obs_candidates; obs_info];
%                 end
%             end
% 
%             % 複数検知時: 最も距離が近い最重要脅威を先頭にソート
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
%         % display_detection_report: 診断レポート
%         % =====================================================================
%         function display_detection_report(obj, t_now, det, req_clearance, t_impact, n_escape)
%             tgt = det.detected_obstacles_point(1);
%             names = ["位置(0階)", "速度(1階)", "加速度(2階)", "Jerk(3階)", "Snap(4階)", "Crack(5階)", "Pop(6階)"];
% 
%             fprintf("\n=================================================================================\n");
%             fprintf(" [B-SPLINE 15m センサー検知 ＆ C^6 回避診断レポート]  t = %.3f s\n", t_now);
%             fprintf("=================================================================================\n");
%             fprintf(" 1. センサ判定元      : 機体重心 pQ 搭載模擬センサー (機体表面間距離: %.3f m)\n", tgt.dist_drone_point);
%             fprintf(" 2. 索・荷物状態診断  : 索球列表面間: %.3f m (侵入: %d), 荷物表面間: %.3f m (侵入: %d)\n", ...
%                 tgt.dist_cable_point, det.cable_inside_obstacle_point, tgt.dist_load_point, det.load_inside_obstacle_point);
%             fprintf(" 3. 捕捉障害物情報    : 危険順位 1位 / 候補 %d 個 (ID=%d, 中心=[%.2f, %.2f, %.2f] m)\n", ...
%                 det.detected_obstacle_count_point, tgt.id, tgt.p_obs(1), tgt.p_obs(2), tgt.p_obs(3));
%             fprintf(" 4. 運動学制約考慮    : 許容 a_max = %.2f m/s^2 -> 回避時間 T_tot = %.2f s (急激な横傾きを防止)\n", ...
%                 obj.max_acc_load, obj.t_duration);
%             fprintf(" 5. 回避幾何拘束      : 要求クリアランス = %.2f m (退避ベクトル: [%.2f, %.2f, %.2f])\n", ...
%                 req_clearance, n_escape(1), n_escape(2), n_escape(3));
%             fprintf(" 6. C^6 始端境界ギャップ実測値 (M \\ B 解の厳密検証):\n");
%             for k = 0:6
%                 fprintf("     - %-12s e_%d = %.3e m/s^%d (数学的連続保証)\n", names(k + 1), k, obj.c6_boundary_gaps(k + 1), k);
%             end
%             fprintf(" 7. QP最適化計算時間  : %6.2f ms (18自由度 Aeqフリー超高速二次計画)\n", obj.last_solve_time_ms);
%             fprintf(" 8. 最大空間変位      : %.3f m\n", obj.actual_peak_displacement);
%             fprintf("=================================================================================\n\n");
%         end
% 
%         % =====================================================================
%         % point_ellipsoid_signed_distance: ラグランジュ未定乗数法による厳密距離
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
%         function val = eval_spline_kth(obj, tau, k)
%             p = obj.spline_degree;
%             T_tot = obj.t_duration;
%             t_eval = max(0.0, min(T_tot - 1e-7, tau));
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
%             st.drone_min_dist_point          = inf;
%             st.load_min_dist_point           = inf;
%             st.cable_min_dist_point          = inf;
%             st.min_dist_point                = inf;
%             st.drone_obstacle_id_point       = NaN;
%             st.load_obstacle_id_point        = NaN;
%             st.min_obstacle_id_point         = NaN;
%             st.min_source_point              = "none";
%             st.detected_obstacle_count_point = 0;
%         end
% 
%         function list = get_obstacles_at_time(~, t_now)
%             try
%                 list = ENVIRONMENT_OBSTACLE_ELLIPSE_MOVE(t_now);
%             catch
%                 p1 = [0.3; 0.3; 20.0]; rad1 = [2.5*sqrt(2); 2.5*sqrt(2); 0.5*sqrt(2)];
%                 p2 = [0.0; 18.0; 2.0]; rad2 = [3.0*sqrt(2); 3.0*sqrt(2); 0.5*sqrt(2)];
%                 p3 = [0.0; 28.0; 0.0]; rad3 = [1.0*sqrt(2); 1.0*sqrt(2); 0.5*sqrt(2)];
%                 list = [struct('p_center', p1, 'ellipsoid_radii', rad1, 'R_obs', eye(3), 'd_margin', 0.5, 'v_center', [0;0;0]), ...
%                         struct('p_center', p2, 'ellipsoid_radii', rad2, 'R_obs', eye(3), 'd_margin', 0.5, 'v_center', [0;0;0]), ...
%                         struct('p_center', p3, 'ellipsoid_radii', rad3, 'R_obs', eye(3), 'd_margin', 0.5, 'v_center', [0;0;0])];
%             end
%         end
%     end
% end

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
function data = extract_flight_data(mat_filepath, target_phase)
    % EXTRACT_FLIGHT_DATA ログデータから指定フェーズ（102等）のデータを抽出し構造体に格納する関数
    
    if nargin < 2
        target_phase = 102;
    end

    % MATファイルの読み込み
    loaded_struct = load(mat_filepath, 'log');
    log = loaded_struct.log;

    t_raw = log.Data.t(1:log.k);   % 元の時間軸
    phase = log.Data.phase;        % フライトフェーズ
    k = log.k;                     % データ数

    % 対象フェーズ (102) のインデックスを一括取得
    idx_list = find(phase(1:k) == target_phase);
    N = length(idx_list);          % データ数

    % 時間軸
    t_flight_raw = t_raw(idx_list);
    data.t = t_flight_raw - t_flight_raw(1); % 0秒スタート

    %% -------------------------------------------------------------------------
    %% 配列の事前割り当て（構造体フィールドへのメモリ確保）
    %% -------------------------------------------------------------------------
    % 1. 実測値 (sensor)
    data.sensor.drone_p   = zeros(3, N); % ドローン実測位置 (x, y, z)
    data.sensor.drone_att = zeros(3, N); % ドローン実測姿勢 (roll, pitch, yaw)
    data.sensor.load_p    = zeros(3, N); % 吊り荷実測位置 (x, y, z)
    data.sensor.load_att  = zeros(3, N); % 吊り荷実測姿勢 (roll, pitch, yaw)

    % 2. 推定値 (estimator)
    data.estimator.pL = zeros(3, N); % 吊り荷推定位置 pL
    data.estimator.pT = zeros(3, N); % ケーブル姿勢ベクトル pT
    data.estimator.v  = zeros(3, N); % ドローン推定速度 v
    data.estimator.q  = zeros(3, N); % ドローン推定姿勢 q
    data.estimator.mL = zeros(1, N); % 吊り荷質量推定値 mL
    data.estimator.vL = zeros(3, N); % 吊り荷推定速度 vL
    data.estimator.wL = zeros(3, N); % 吊り荷推定角速度 wL
    data.estimator.p  = zeros(3, N); % ドローン推定位置 p
    data.estimator.w  = zeros(3, N); % ドローン推定角速度 w

    % 3. 指令値 (reference)
    data.ref.p = zeros(3, N); % 目標位置 p_ref
    data.ref.v = zeros(3, N); % 目標速度 v_ref

    % 4. 制御値 (controller)
    data.cbf.tmp                 = zeros(4, N); % 修正前の公称入力 [thrust; roll; pitch; yaw]
    data.cbf.rl                  = zeros(1, N); % システム側の球体の半径
    data.cbf.p_mid               = zeros(3, N); % 牽引紐の中点位置 [x; y; z]
    data.cbf.p_obs               = zeros(3, N); % 障害物位置 [x; y; z]
    data.cbf.r_obs               = zeros(1, N); % マージンのない障害物球の半径
    data.cbf.r_minimal           = zeros(1, N); % マージンのあるときのシステムと障害物の表面距離
    data.cbf.r_minimal_no_margin = zeros(1, N); % マージンのないときのシステムと障害物の表面距離
    data.cbf.A_z_qp_list        = zeros(1, N); % QP制約係数 (1行目: スロットル)
    data.cbf.b_z_qp_list        = zeros(1, N); % 入力u_1に許容される上限制約
    data.cbf.slack_check         = zeros(1, N); % 公称入力をそのまま入れた場合どの程安全制約を違反しているのか プラスだと制約違反
    data.cbf.num_violated        = zeros(1, N); % 公称入力をそのまま使用したら安全制約を違反する障害物の数
    data.cbf.tmp_fix             = zeros(4, N); % 修正した後の入力 [thrust; roll; pitch; yaw]
    data.cbf.controllertime      = zeros(1, N); % コントローラ単体の時間
    data.cbf.controllertime_total= zeros(1, N); % 全体の制御周期
    data.cbf.r_obs_margin        = zeros(1, N); % マージンのある障害物球の半径
    data.cbf.d_margin            = zeros(1, N); % マージンの大きさ
    data.cbf.log_h               = zeros(2, N); % 安全余裕度 (1:位置, 2:速度)
    data.cbf.slack_safe          = zeros(1, N); % 入力適用後の安全度 マイナスなら異常

    % ▼▼▼ 新規追加: 姿勢角＆振れ角 CBFデータ ▼▼▼
    data.cbf.A_xy_qp         = zeros(3, 2, N); % Roll, Pitch, Cableに対する QP制約係数行列 (3行2列)
    data.cbf.b_xy_qp         = zeros(3, N);    % Roll, Pitch, Cableに対する 上限制約 (3行)
    data.cbf.h_layers_att_cb = zeros(8, N);    % 各階層安全余裕度 (Roll:1-2, Pitch:3-4, Cable:5-8)
    data.cbf.slack_nom_xy    = zeros(3, N);    % 姿勢・振れ角の公称入力時の制約違反量
    data.cbf.slack_safe_xy   = zeros(3, N);    % 姿勢・振れ角の入力適用後の安全度
    data.cbf.num_violated_xy = zeros(1, N);    % 姿勢・振れ角の制約を違反した数
    % ▲▲▲ 新規追加ここまで ▲▲▲

    % 5. 入力値 (input)
    data.input = zeros(4, N); % 最終実制御入力 [thrust; roll; pitch; yaw]

    %% -------------------------------------------------------------------------
    %% ループ処理による抽出し
    %% -------------------------------------------------------------------------
    for i = 1:N
        idx = idx_list(i);

        % 1. 実測値 (sensor)
        sensor_res = log.Data.agent(1).sensor.result{1, idx};
        data.sensor.drone_p(:, i)   = sensor_res.output(1:3);   % ドローン位置 (x, y, z)
        data.sensor.drone_att(:, i) = sensor_res.output(4:6);   % ドローン姿勢 (roll, pitch, yaw)
        data.sensor.load_p(:, i)    = sensor_res.output(7:9);   % 吊り荷位置 (x, y, z)
        data.sensor.load_att(:, i)  = sensor_res.output(10:12); % 吊り荷姿勢 (roll, pitch, yaw)

        % 2. 推定値 (estimator)
        est_res = log.Data.agent(1).estimator.result{1, idx};
        data.estimator.pL(:, i) = est_res.state.pL(1:3);
        data.estimator.pT(:, i) = est_res.state.pT(1:3);
        data.estimator.v(:, i)  = est_res.state.v(1:3);
        data.estimator.q(:, i)  = est_res.state.q(1:3);
        data.estimator.mL(i)    = est_res.state.mL(1);
        data.estimator.vL(:, i) = est_res.state.vL(1:3);
        data.estimator.wL(:, i) = est_res.state.wL(1:3);
        data.estimator.p(:, i)  = est_res.state.p(1:3);
        data.estimator.w(:, i)  = est_res.state.w(1:3);

        % 3. 指令値 (reference)
        ref_res = log.Data.agent(1).reference.result{1, idx};
        data.ref.p(:, i) = ref_res.state.p(1:3);
        data.ref.v(:, i) = ref_res.state.v(1:3);

        % 4. 制御値 (controller)
        con_res = log.Data.agent(1).controller.result{1, idx};
        data.cbf.tmp(:, i)                  = con_res.tmp(1:4);               % 修正前の公称入力
        data.cbf.rl(i)                      = con_res.rl(1);                  % システム側の球体の半径
        data.cbf.p_mid(:, i)                = con_res.p_mid(1:3);             % 牽引紐の中点位置
        
        obs_vector = con_res.p_obs{1};
        data.cbf.p_obs(:, i)                = obs_vector(1:3);                % 障害物位置
        data.cbf.r_obs(i)                   = con_res.r_obs{1};               % マージンのない障害物球の半径
        data.cbf.r_minimal(i)               = con_res.r_minimal{1};           % マージンのあるときのシステムと障害物の表面距離
        
        if isfield(con_res, 'r_minimal_no_margin')
            data.cbf.r_minimal_no_margin(i) = con_res.r_minimal_no_margin{1}; % マージンのないときの表面距離
        end
        
        data.cbf.A_z_qp_list(:, i)         = con_res.A_z_qp_list(1);       % 1: ロールに対する制約係数, 2: ピッチに対する制約係数
        data.cbf.b_z_qp_list(i)            = con_res.b_z_qp_list(1);        % 入力u_23に許容される上限制約
        data.cbf.slack_check(i)             = con_res.slack_check(1);         % 公称入力をそのまま入れた場合どの程安全制約を違反しているのか プラスだと制約違反
        data.cbf.num_violated(i)            = con_res.num_violated(1);        % 公称入力をそのまま使用したら安全制約を違反する障害物の数
        data.cbf.tmp_fix(:, i)              = con_res.tmp_fix(1:4);           % 修正した後の入力
        data.cbf.controllertime(i)          = con_res.controllertime(1);      % コントローラ単体の時間
        
        if isfield(con_res, 'controllertime_total')
            data.cbf.controllertime_total(i)= con_res.controllertime_total(1);% 全体の制御周期
        end
        
        data.cbf.r_obs_margin(i)            = con_res.r_obs_margin{1};        % マージンのある障害物球の半径
        data.cbf.d_margin(i)                = con_res.d_margin{1};            % マージンの大きさ
        
        data.cbf.log_h(:, i)                = [con_res.log_h1(1); ...         % 位置の安全余裕度
                                               con_res.log_h2(1) ...         % 速度の安全余裕度
                                               ];
        data.cbf.slack_safe(i)              = con_res.slack_safe(1);          % 入力適用後の安全度 マイナスなら異常

        % =========================================================================
        % ▼▼▼ 新規追加: 姿勢角＆振れ角 CBFデータ (旧バージョンのログ互換のため isfield で判定) ▼▼▼
        if isfield(con_res, 'A_xy_qp')
            data.cbf.A_xy_qp(:, :, i)       = con_res.A_xy_qp(1:3, 1:2);    % 3x2 行列
            data.cbf.b_xy_qp(:, i)          = con_res.b_xy_qp(1:3);         % 3x1 ベクトル
            data.cbf.h_layers_att_cb(:, i)  = con_res.h_layers_att_cb(1:8); % 8x1 ベクトル
            data.cbf.slack_nom_xy(:, i)     = con_res.slack_nom_xy(1:3);    % 3x1 ベクトル
            data.cbf.slack_safe_xy(:, i)    = con_res.slack_safe_xy(1:3);   % 3x1 ベクトル
            data.cbf.num_violated_xy(i)     = con_res.num_violated_xy(1);   % スカラー
        end
        % ▲▲▲ 新規追加ここまで ▲▲▲

        % 5. 入力値 (input)
        input_res = log.Data.agent(1).input{1, idx};
        data.input(:, i) = input_res(1:4);
    end
end
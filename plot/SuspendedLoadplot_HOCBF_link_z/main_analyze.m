%% 初期設定とパスのセットアップ
mfile_path = fileparts(mfilename('fullpath'));
cd(mfile_path);

% plots サブフォルダへのパスを通す
addpath(fullfile(mfile_path, 'plots'));

%% データの読み込み＆抽出
mat_file = 'HOCBF_LINK_Z_normal_Log(15-Aug-2026_14_46_24).mat';
data = extract_flight_data(mat_file, 102);

%% グラフの個別に呼び出し（必要なグラフだけONにする）
% plot_obstacle_and_reference(data); % 障害物と目標軌道　軌道の説明に使用
% plot_obstacle_body_margin_and_reference(data); % 障害物と目標軌道　マージン区別　軌道の説明に使用
% plot_3d_trajectory(data); % 牽引物の目標軌道と実際の軌道
% plot_3d_trajectory_margin_obs(data); % 牽引物の目標軌道と実際の軌道　障害物を近似とマージンに分ける
% plot_computation_time_all(data); %計算時間
% plot_load_position_tracking(data); % 荷物の位置と目標軌道の時系列
% plot_obstacle_clearance(data); % 障害物との距離
% plot_control_inputs(data); % 入力
% plot_drone_position(data); % 機体位置
% plot_velocities(data); % 機体・荷物速度
% plot_cable_states(data); % ケーブルの姿勢
% plot_estimated_mass(data); % 牽引物質量推定
% plot_attitude_angles(data); % 機体姿勢角
% plot_angular_velocities(data); % ケーブル角速度　機体角速度
plot_qp_constraints(data); % QP安全制約 A * u <= b の関係性
% plot_cbf_slack_and_violations(data); % 公称入力での違反量 vs 補正後の安全余裕度 違反障害物数
% plot_hocbf_margins(data); % 高次CBF（HOCBF）の各階層余裕度 h1〜h2

% 例: 別途作成した描画関数を呼び出す場合
% plot_inputs(data);
% plot_hocbf_logs(data);
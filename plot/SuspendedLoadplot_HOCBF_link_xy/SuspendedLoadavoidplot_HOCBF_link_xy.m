% % 現在のフォルダからパスが通っているかを確認して移動
% mfile_path = fileparts(mfilename('fullpath'));
% cd(mfile_path);
% 
% %% 生データから安全に一括抽出する（正確なデータパス版）
% t = log.Data.t(1:log.k);       % 時間軸
% phase = log.Data.phase;        % フライトフェーズ
% k = log.k;                     % データ数
% 
% for idx = 1:k
%     % 1. 実測値 (sensor) -> outputベクトルの要素番号から直接取得
%     sensor_res = log.Data.agent(1).sensor.result{1, idx};
%     drone_x_s(idx)     = sensor_res.output(1);
%     drone_y_s(idx)     = sensor_res.output(2);
%     drone_z_s(idx)     = sensor_res.output(3);
%     drone_roll_s(idx)  = sensor_res.output(4);
%     drone_pitch_s(idx) = sensor_res.output(5);
%     drone_yaw_s(idx)   = sensor_res.output(6);
%     load_x_s(idx)     = sensor_res.output(7);
%     load_y_s(idx)     = sensor_res.output(8);
%     load_z_s(idx)     = sensor_res.output(9);
%     load_roll_s(idx)  = sensor_res.output(10);
%     load_pitch_s(idx) = sensor_res.output(11);
%     load_yaw_s(idx)   = sensor_res.output(12);
% 
%     % 2. 推定値 (estimator) -> 同様にベクトルの要素番号から取得（通常12次元以上）
%     est_res = log.Data.agent(1).estimator.result{1, idx};
%     pL_est_x(idx) = est_res.state.pL(1);
%     pL_est_y(idx) = est_res.state.pL(2);
%     pL_est_z(idx) = est_res.state.pL(3);
%     pT_est_x(idx) = est_res.state.pT(1);
%     pT_est_y(idx) = est_res.state.pT(2);
%     pT_est_z(idx) = est_res.state.pT(3);
%     v_est_x(idx) = est_res.state.v(1);
%     v_est_y(idx) = est_res.state.v(2);
%     v_est_z(idx) = est_res.state.v(3);
%     q_est_x(idx) = est_res.state.q(1);
%     q_est_y(idx) = est_res.state.q(2);
%     q_est_z(idx) = est_res.state.q(3);
%     mL_est(idx) = est_res.state.mL(1);
%     vL_est_x(idx) = est_res.state.vL(1);
%     vL_est_y(idx) = est_res.state.vL(2);
%     vL_est_z(idx) = est_res.state.vL(3);
%     wL_est_x(idx) = est_res.state.wL(1);
%     wL_est_y(idx) = est_res.state.wL(2);
%     wL_est_z(idx) = est_res.state.wL(3);
%     p_est_x(idx) = est_res.state.p(1);
%     p_est_y(idx) = est_res.state.p(2);
%     p_est_z(idx) = est_res.state.p(3);
%     w_est_x(idx) = est_res.state.w(1);
%     w_est_y(idx) = est_res.state.w(2);
%     w_est_z(idx) = est_res.state.w(3);
% 
%     ref_res = log.Data.agent(1).reference.result{1, idx};
%     p_ref_x(idx) = est_res.state.p(1);
%     p_ref_y(idx) = est_res.state.p(2);
%     p_ref_z(idx) = est_res.state.p(3);
%     v_ref_x(idx) = est_res.state.v(1);
%     v_ref_y(idx) = est_res.state.v(2);
%     v_ref_z(idx) = est_res.state.v(3);
% 
%     con_res = log.Data.agent(1).controller.result{1, idx};
%     tmp_thrust(idx) = con_res.tmp(1);
%     tmp_roll(idx) = con_res.tmp(2);
%     tmp_pitch(idx) = con_res.tmp(3);
%     tmp_yaw(idx) = con_res.tmp(4);
%     rl(idx) = con_res.rl(1);
%     obs_vector = con_res.p_obs{1};
%     p_obs_x(idx) = obs_vector(1);
%     p_obs_y(idx) = obs_vector(2);
%     p_obs_z(idx) = obs_vector(3);
%     r_obs(idx) = con_res.r_obs(1);
%     r_minimal(idx) = con_res.r_minimal(1);
%     controllertime(idx) = con_res.controllertime(1);
% 
% 
%     input_res = log.Data.agent(1).input{1, idx};
%     input_thrust(idx) = input_res(1);
%     input_roll(idx) = input_res(2);
%     input_pitch(idx) = input_res(3);
%     input_yaw(idx) = input_res(4);
% 
% 
% end

% 現在のフォルダからパスが通っているかを確認して移動
mfile_path = fileparts(mfilename('fullpath'));
cd(mfile_path);
load('otamesi2_Log(12-Aug-2026_16_52_15).mat', '-mat');
%% 生データから102（フライト）のデータのみを詰めて抽出する
t_raw = log.Data.t(1:log.k);   % 元の時間軸
phase = log.Data.phase;        % フライトフェーズ
k = log.k;                     % データ数

% --- 102専用の格納カウンター ---
sel_idx = 0; 

for idx = 1:k
    % phaseが102ではないときは、これ以降の処理をスキップして次のループへ
    if phase(idx) ~= 102; continue; end 
    
    % 102のときだけここを通過し、格納位置を1つ進める
    sel_idx = sel_idx + 1;
    
    % 1. 実測値 (sensor)
    sensor_res = log.Data.agent(1).sensor.result{1, idx};
    drone_x_s(sel_idx)     = sensor_res.output(1);
    drone_y_s(sel_idx)     = sensor_res.output(2);
    drone_z_s(sel_idx)     = sensor_res.output(3);
    drone_roll_s(sel_idx)  = sensor_res.output(4);
    drone_pitch_s(sel_idx) = sensor_res.output(5);
    drone_yaw_s(sel_idx)   = sensor_res.output(6);
    load_x_s(sel_idx)      = sensor_res.output(7);
    load_y_s(sel_idx)      = sensor_res.output(8);
    load_z_s(sel_idx)      = sensor_res.output(9);
    load_roll_s(sel_idx)   = sensor_res.output(10);
    load_pitch_s(sel_idx)  = sensor_res.output(11);
    load_yaw_s(sel_idx)    = sensor_res.output(12);
    
    % 2. 推定値 (estimator)
    est_res = log.Data.agent(1).estimator.result{1, idx};
    pL_est_x(sel_idx) = est_res.state.pL(1);
    pL_est_y(sel_idx) = est_res.state.pL(2);
    pL_est_z(sel_idx) = est_res.state.pL(3);
    pT_est_x(sel_idx) = est_res.state.pT(1);
    pT_est_y(sel_idx) = est_res.state.pT(2);
    pT_est_z(sel_idx) = est_res.state.pT(3);
    v_est_x(sel_idx)  = est_res.state.v(1);
    v_est_y(sel_idx)  = est_res.state.v(2);
    v_est_z(sel_idx)  = est_res.state.v(3);
    q_est_x(sel_idx)  = est_res.state.q(1);
    q_est_y(sel_idx)  = est_res.state.q(2);
    q_est_z(sel_idx)  = est_res.state.q(3);
    mL_est(sel_idx)   = est_res.state.mL(1);
    vL_est_x(sel_idx) = est_res.state.vL(1);
    vL_est_y(sel_idx) = est_res.state.vL(2);
    vL_est_z(sel_idx) = est_res.state.vL(3);
    wL_est_x(sel_idx) = est_res.state.wL(1);
    wL_est_y(sel_idx) = est_res.state.wL(2);
    wL_est_z(sel_idx) = est_res.state.wL(3);
    p_est_x(sel_idx)  = est_res.state.p(1);
    p_est_y(sel_idx)  = est_res.state.p(2);
    p_est_z(sel_idx)  = est_res.state.p(3);
    w_est_x(sel_idx)  = est_res.state.w(1);
    w_est_y(sel_idx)  = est_res.state.w(2);
    w_est_z(sel_idx)  = est_res.state.w(3);
    
    % 3. 指令値 (reference)
    ref_res = log.Data.agent(1).reference.result{1, idx};
    p_ref_x(sel_idx) = ref_res.state.p(1);
    p_ref_y(sel_idx) = ref_res.state.p(2);
    p_ref_z(sel_idx) = ref_res.state.p(3);
    v_ref_x(sel_idx) = ref_res.state.v(1);
    v_ref_y(sel_idx) = ref_res.state.v(2);
    v_ref_z(sel_idx) = ref_res.state.v(3);
    
    % 4. 制御値 (controller)
    con_res = log.Data.agent(1).controller.result{1, idx};
    tmp_thrust(sel_idx) = con_res.tmp(1); %修正前の公称入力
    tmp_roll(sel_idx)   = con_res.tmp(2);
    tmp_pitch(sel_idx)  = con_res.tmp(3);
    tmp_yaw(sel_idx)    = con_res.tmp(4);
    rl(sel_idx)         = con_res.rl(1); % システム側の球体の半径
    p_mid_x(sel_idx)         = con_res.p_mid(1); %牽引紐の中点位置
    p_mid_y(sel_idx)         = con_res.p_mid(2);
    p_mid_x(sel_idx)         = con_res.p_mid(3);

    
    obs_vector = con_res.p_obs{1};
    p_obs_x(sel_idx) = obs_vector(1); % 障害物位置
    p_obs_y(sel_idx) = obs_vector(2);
    p_obs_z(sel_idx) = obs_vector(3);
    r_obs(sel_idx)     = con_res.r_obs{1}; % マージンのない障害物球の半径
    r_minimal(sel_idx) = con_res.r_minimal{1}; % マージンのあるときのシステムと障害物の表面距離
    r_minimal_no_margin(sel_idx) = con_res.r_minimal_no_margin{1}; % マージンのあるときのシステムと障害物の表面距離
    A_xy_qp_list_roll(sel_idx)         = con_res.A_xy_qp_list(1); % ロールに対する制約係数
    A_xy_qp_list_pitch(sel_idx)         = con_res.A_xy_qp_list(2); % ピッチに対する制約係数
    b_xy_qp_list(sel_idx)         = con_res.b_xy_qp_list(1); % 入力u_23に許容される上限制約
    slack_check(sel_idx) = con_res.slack_check(1); % 公称入力をそのまま入れた場合どの程安全制約を違反しているのか　プラスだと制約違反
    num_violated(sel_idx) = con_res.num_violated(1); % 公称入力をそのまま使用したら安全制約を違反する障害物の数
    tmp_fix_thrust(sel_idx) = con_res.tmp_fix(1); % 修正した後の入力
    tmp_fix_roll(sel_idx)   = con_res.tmp_fix(2);
    tmp_fix_pitch(sel_idx)  = con_res.tmp_fix(3);
    tmp_fix_yaw(sel_idx)    = con_res.tmp_fix(4);
    controllertime(sel_idx) = con_res.controllertime(1); % コントローラ単体の時間
    controllertime_total(sel_idx) = con_res.controllertime_total(1); % 全体の制御周期
    r_obs_margin(sel_idx)     = con_res.r_obs_margin{1}; % マージンのある障害物球の半径
    d_margin(sel_idx)     = con_res.d_margin{1}; % マージンの大きさ
    log_h1(sel_idx)     = con_res.log_h1(1); % 位置の安全余裕度
    log_h2(sel_idx)     = con_res.log_h2(1); % 速度の安全余裕度
    log_h3(sel_idx)     = con_res.log_h3(1); % 加速度の安全余裕度
    log_h4(sel_idx)     = con_res.log_h4(1); % か加速度の安全余裕度
    slack_safe(sel_idx) = con_res.slack_safe(1); % 入力適用後の安全度　マイナスなら異常
    
    % 5. 入力値 (input)
    input_res = log.Data.agent(1).input{1, idx};
    input_thrust(sel_idx) = input_res(1);
    input_roll(sel_idx)   = input_res(2);
    input_pitch(sel_idx)  = input_res(3);
    input_yaw(sel_idx)    = input_res(4);
end

% --- 時間軸（t）もフライト時間のみを切り出して長さを合わせる ---
t_flight_raw = t_raw(phase == 102);
% プロット用に時間軸のスタートを「0秒」からにリセットしたい場合は下の行のコメントアウトを解除
% t = t_flight_raw - t_flight_raw(1); 
t = t_flight_raw; 


%% 3Dグラフの作成 (障害物 ＆ 目標軌道) マージン有の障害物
figure;

% 1. 障害物の描画 (1番目のデータを使用)
obs_center_x = p_obs_x(1);
obs_center_y = p_obs_y(1);
obs_center_z = p_obs_z(1);
obs_radius   = r_obs_margin(1);

% 球体のメッシュデータを生成
[X_sphere, Y_sphere, Z_sphere] = sphere(50); 

% 中心位置と半径を適用
X_sphere = X_sphere * obs_radius + obs_center_x;
Y_sphere = Y_sphere * obs_radius + obs_center_y;
Z_sphere = Z_sphere * obs_radius + obs_center_z;

% 半透明の球体を描画
surf(X_sphere, Y_sphere, Z_sphere, 'FaceColor', [1, 0.5, 0], 'EdgeColor', 'none');
alpha(0.4); % 透明度 (0.0:完全に透明 〜 1.0:完全に不透明)
hold on;

% 2. 目標軌道 (p_ref) の描画 (赤い点線)
plot3(p_ref_x, p_ref_y, p_ref_z, 'r--', 'LineWidth', 2);

% 3. グラフの装飾
grid on;
axis equal; % アスペクト比を1:1:1に固定
xlabel('X [m]', 'FontSize', 12);
ylabel('Y [m]', 'FontSize', 12);
zlabel('Z [m]', 'FontSize', 12);

% 凡例の設定
legend('Obstacle', 'Reference', 'Location', 'best', 'FontSize', 15);

% 視点の調整 (見やすい3D角度に設定。必要に応じて数値を調整してください)
view(3); 
hold off;

fprintf('障害物半径  : %f [m]\n', obs_radius);

%% 3Dグラフの作成 (障害物 ＆ 牽引物 ＆ 目標軌道)
figure;

% 1. 障害物の描画 (オレンジ色の半透明球体)
if iscell(con_res.r_obs)
    obs_radius = con_res.r_obs{1};
else
    obs_radius = r_obs(1);
end
if iscell(obs_radius); obs_radius = obs_radius{1}; end

obs_center_x = p_obs_x(1);
obs_center_y = p_obs_y(1);
obs_center_z = p_obs_z(1);

if iscell(obs_center_x); obs_center_x = obs_center_x{1}; end
if iscell(obs_center_y); obs_center_y = obs_center_y{1}; end
if iscell(obs_center_z); obs_center_z = obs_center_z{1}; end

obs_radius   = double(obs_radius);
obs_center_x = double(obs_center_x);
obs_center_y = double(obs_center_y);
obs_center_z = double(obs_center_z);

[X_sphere, Y_sphere, Z_sphere] = sphere(50); 
X_sphere = X_sphere * obs_radius + obs_center_x;
Y_sphere = Y_sphere * obs_radius + obs_center_y;
Z_sphere = Z_sphere * obs_radius + obs_center_z;

% オレンジ色 [1, 0.5, 0] で描画
surf(X_sphere, Y_sphere, Z_sphere, 'FaceColor', [1, 0.5, 0], 'EdgeColor', 'none');
alpha(0.4); 
hold on;
camlight;           
lighting gouraud;   

% 2. 牽引物の軌道 (pL_est) の描画 (青い実線)
plot3(pL_est_x, pL_est_y, pL_est_z, 'b-', 'LineWidth', 2);

% 3. ドローンの目標軌道 (p_ref) の描画 (赤い点線)
plot3(p_ref_x, p_ref_y, p_ref_z, 'r--', 'LineWidth', 1.5);

% (オプション) もしドローン自身の推定軌道 (p_est) も緑の実線などで重ねたい場合は、下の行のコメントアウトを解除してください
% plot3(p_est_x, p_est_y, p_est_z, 'g-', 'LineWidth', 1.5);

% 4. グラフの装飾
grid on;
axis equal; 
xlabel('X [m]', 'FontSize', 12);
ylabel('Y [m]', 'FontSize', 12);
zlabel('Z [m]', 'FontSize', 12);

% 凡例の設定
legend('Obstacle', 'Estimator', 'Reference', ...
       'Location', 'best', 'FontSize', 15);

view(3); 
hold off;

%% 2Dグラフの作成 (横軸: 時間t 軸、縦軸: コントローラ計算時間) [統計値の追加版]
% figure;

% 1. t が 0 より大きい要素のインデックス（場所）を特定
valid_idx = (t > 0);

% 有効なデータだけを抽出
t_valid = t(valid_idx);
c_time_valid = controllertime(valid_idx);

% 2. 統計値（最大・最小・平均）の計算
max_val = max(c_time_valid);
min_val = min(c_time_valid);
mean_val = mean(c_time_valid);

% 計算結果をコマンドウィンドウにわかりやすく表示
fprintf('\n--- Controller Computation Time Statistics (Flight) ---\n');
fprintf('最大値 (Max)  : %f [s]\n', max_val);
fprintf('最小値 (Min)  : %f [s]\n', min_val);
fprintf('平均値 (Mean) : %f [s]\n', mean_val);
fprintf('-------------------------------------------------------\n');

% % 3. 有効なデータでメインのプロット (青い実線)
% plot(t_valid, c_time_valid, 'b-', 'LineWidth', 1.5);
% hold on;

%% 2Dグラフの作成 (横軸: 時間t 軸、縦軸: 計算時間) [統計値の追加版]
figure;

% 1. t が 0 より大きい要素のインデックス（場所）を特定
valid_idx = (t > 0);

% 有効なデータだけを抽出
t_valid = t(valid_idx);
time_total_valid = controllertime_total(valid_idx);

% 2. 統計値（最大・最小・平均）の計算
max_val_total = max(time_total_valid );
min_val_total = min(time_total_valid );
mean_val_total = mean(time_total_valid );

% 計算結果をコマンドウィンドウにわかりやすく表示
fprintf('\n--- Computation Time Statistics (Flight) ---\n');
fprintf('最大値 (Max)  : %f [s]\n', max_val_total);
fprintf('最小値 (Min)  : %f [s]\n', min_val_total);
fprintf('平均値 (Mean) : %f [s]\n', mean_val_total);
fprintf('-------------------------------------------------------\n');

% 3. 有効なデータでメインのプロット (青い実線)
plot(t_valid, time_total_valid, 'b-', 'LineWidth', 1.5);
hold on;



% 5. グラフの装飾
grid on;
xlabel('Time [s]', 'FontSize', 12);
ylabel('Computation Time [s]', 'FontSize', 12);


% 軸の範囲調整
ylim([0, max_val_total * 1.3]); % 最大値より少し上に余裕を持たせる
xlim([min(t_valid), max(t_valid)]);



hold off;

%% =========================================================================
%% ⭐【追加】データがある区間（t > 0）のみプロット（実線・点線の視認性大幅向上版）
%% =========================================================================

% 1. 102フェーズの中で、さらに時間が 0 秒より大きい要素のインデックス（位置）を見つける
t_pos_idx = (t > 0);

% 2. 時間 t > 0 のデータだけを新しく切り出し（データの末尾までしっかり確保）
t_plot     = t(t_pos_idx);
x_est_plot = pL_est_x(t_pos_idx);
y_est_plot = pL_est_y(t_pos_idx);
z_est_plot = pL_est_z(t_pos_idx);

x_ref_plot = p_ref_x(t_pos_idx);
y_ref_plot = p_ref_y(t_pos_idx);
z_ref_plot = p_ref_z(t_pos_idx);

% 3. グラフの描画開始
figure;%('Name', 'Load Position vs Reference over Time (t > 0 Only)', 'NumberTitle', 'off');

% --- 実測値（推定値）を「濃いめの実線」でプロット ---
plot(t_plot, x_est_plot, 'Color', [0.85 0.33 0.10], 'LineStyle', '-', 'LineWidth', 2); hold on; % 荷物X: 橙・実線
plot(t_plot, y_est_plot, 'Color', [0.47 0.67 0.19], 'LineStyle', '-', 'LineWidth', 2);          % 荷物Y: 黄緑・実線
plot(t_plot, z_est_plot, 'Color', [0.00 0.45 0.74], 'LineStyle', '-', 'LineWidth', 2);          % 荷物Z: 青・実線

% --- 目標軌道 (p_ref) を「識別しやすい別の色の点線」でプロット ---
plot(t_plot, x_ref_plot, 'Color', [0.49 0.18 0.56], 'LineStyle', '--', 'LineWidth', 1.5);       % 目標X: 紫・点線
plot(t_plot, y_ref_plot, 'Color', [0.30 0.75 0.93], 'LineStyle', '--', 'LineWidth', 1.5);       % 目標Y: 水色・点線
plot(t_plot, z_ref_plot, 'Color', [0.64 0.08 0.18], 'LineStyle', '--', 'LineWidth', 1.5);       % 目標Z: 暗赤・点線

% 4. グラフの装飾設定
grid on;
xlabel('Time [s]', 'FontSize', 12);
ylabel('Load Position [m]', 'FontSize', 12);


% 軸のフォントと「データが本当にある範囲」への完全絞り込み
set(gca, 'FontSize', 11);
xlim([t_plot(1), t_plot(end)]); % 横軸を表示データの最初から最後まで（余白ゼロ）に固定

% 凡例の設定（それぞれの線の色とスタイルに正確に対応）
legend('load\_x (Est)', 'load\_y (Est)', 'load\_z (Est)', ...
       'p\_ref\_x', 'p\_ref\_y', 'p\_ref\_z', ...
       'Location', 'best', 'NumColumns', 2, 'FontSize', 15);
hold off;

%% =========================================================================
%% ⭐【追加】障害物との安全境界距離（点線）と現在の距離（実線）の時系列プロット
%% =========================================================================

% 1. 102フェーズの中で、さらに時間が 0 秒より大きい要素のインデックスを見つける
t_pos_idx = (t > 0);

% 2. 時間 t > 0 のデータを新しく切り出し
t_plot       = t(t_pos_idx);
r_minimal   = r_minimal(t_pos_idx); % 荷物の現在位置 X


% =========================================================================
% ⭐【追加機能】一番近づいた時の距離（最小距離）を算出してコマンドウィンドウに表示
% =========================================================================
min_dist = min(r_minimal); % 最小距離とその時のインデックスを取得


fprintf('\n========================================\n');
fprintf('🔥 【高次CBF検証結果】障害物への最接近データ（t > 0）\n');
fprintf('----------------------------------------\n');
fprintf('一番近づいた時の実際の距離 (最小距離) : %.4f [m]\n', min_dist);
if min_dist >= 0
    fprintf('⇒ 判定: 🔴【安全確認】限界の壁を超えずに回避完了しています。\n');
else
    fprintf('⇒ 判定: ⚠️【警告】境界を割り込んでいます。ゲインを調整してください。\n');
end
fprintf('========================================\n\n');


% 4. グラフの描画開始
figure;

% 【現在の距離】をハッキリした黒の実線でプロット
plot(t_plot, r_minimal, 'r-', 'LineWidth', 2); hold on;

% 5. グラフの装飾設定
grid on;
xlabel('Time [s]', 'FontSize', 12);
ylabel('Distance [m]', 'FontSize', 12);

% 軸のフォントと「データが存在する区間のみ」への完全絞り込み
set(gca, 'FontSize', 11);
xlim([t_plot(1), t_plot(end)]); % 横軸を表示データの最初から最後まで（余白ゼロ）に固定

% 縦軸の範囲を少し広めにとって見やすくする
% ylim([min(r_minimal) * 1.1, max(r_minimal) * 1.1]);
ylim([0, max(r_minimal) * 1.1]);

% 凡例の設定
legend('Distance', ...
    'Location', 'best', 'FontSize', 15);
hold off;

%% =========================================================================
%% ⭐【追加】データがある区間（t > 0）のみプロット（実線・点線の視認性大幅向上版） 入力
%% =========================================================================

% 1. 102フェーズの中で、さらに時間が 0 秒より大きい要素のインデックス（位置）を見つける
t_pos_idx = (t > 0);

% 2. 時間 t > 0 のデータだけを新しく切り出し（データの末尾までしっかり確保）
t_plot     = t(t_pos_idx);
tmp_fix_thrust_plot = tmp_fix_thrust(t_pos_idx);
tmp_fix_roll_plot = tmp_fix_roll(t_pos_idx);
tmp_fix_pitch_plot = tmp_fix_pitch(t_pos_idx);
tmp_fix_yaw_plot = tmp_fix_yaw(t_pos_idx);

tmp_thrust_plot = tmp_thrust(t_pos_idx);
tmp_roll_plot = tmp_roll(t_pos_idx);
tmp_pitch_plot = tmp_pitch(t_pos_idx);
tmp_yaw_plot = tmp_yaw(t_pos_idx);

% 3. グラフの描画開始
figure;%('Name', 'Load Position vs Reference over Time (t > 0 Only)', 'NumberTitle', 'off');

% --- 実測値（推定値）を「濃いめの実線」でプロット ---
plot(t_plot, tmp_fix_thrust_plot, 'Color', [0.85 0.33 0.10], 'LineStyle', '-', 'LineWidth', 2); hold on; % 荷物X: 橙・実線
plot(t_plot, tmp_fix_roll_plot, 'Color', [0.47 0.67 0.19], 'LineStyle', '-', 'LineWidth', 2);          % 荷物Y: 黄緑・実線
plot(t_plot, tmp_fix_pitch_plot, 'Color', [0.00 0.45 0.74], 'LineStyle', '-', 'LineWidth', 2);          % 荷物Z: 青・実線
plot(t_plot, tmp_fix_yaw_plot, 'Color', [0.85 0.00 0.85], 'LineStyle', '-', 'LineWidth', 2);          % 荷物Z: 青・実線

% --- 目標軌道 (p_ref) を「識別しやすい別の色の点線」でプロット ---
plot(t_plot, tmp_thrust_plot, 'Color', [0.49 0.18 0.56], 'LineStyle', '--', 'LineWidth', 1.5);       % 目標X: 紫・点線
plot(t_plot, tmp_roll_plot, 'Color', [0.30 0.75 0.93], 'LineStyle', '--', 'LineWidth', 1.5);       % 目標Y: 水色・点線
plot(t_plot, tmp_pitch_plot, 'Color', [0.64 0.08 0.18], 'LineStyle', '--', 'LineWidth', 1.5);       % 目標Z: 暗赤・点線
plot(t_plot, tmp_yaw_plot, 'Color', [0.00 0.45 0.00], 'LineStyle', '--', 'LineWidth', 2);          % 荷物Z: 青・実線

% 4. グラフの装飾設定
grid on;
xlabel('Time [s]', 'FontSize', 12);
ylabel('Input', 'FontSize', 12);


% 軸のフォントと「データが本当にある範囲」への完全絞り込み
set(gca, 'FontSize', 11);
xlim([t_plot(1), t_plot(end)]); % 横軸を表示データの最初から最後まで（余白ゼロ）に固定

% 凡例の設定（それぞれの線の色とスタイルに正確に対応）
legend('input_{CBF}\_thrust', 'input_{CBF}\_roll', 'input_{CBF}\_pitch', 'input_{CBF}\_yaw', ...
    'input\_thrust', 'input\_roll', 'input\_pitch', 'input\_yaw', ...
    'Location', 'best', 'NumColumns', 2, 'FontSize', 15);
hold off;

%% =========================================================================
%% ⭐【追加】データがある区間（t > 0）のみプロット（実線・点線の視認性大幅向上版）機体位置
%% =========================================================================

% 1. 102フェーズの中で、さらに時間が 0 秒より大きい要素のインデックス（位置）を見つける
t_pos_idx = (t > 0);

% 2. 時間 t > 0 のデータだけを新しく切り出し（データの末尾までしっかり確保）
t_plot     = t(t_pos_idx);
p_est_x_plot = p_est_x(t_pos_idx);
p_est_y_plot = p_est_y(t_pos_idx);
p_est_z_plot = p_est_z(t_pos_idx);

% 3. グラフの描画開始
figure;

% --- 実測値（推定値）を「濃いめの実線」でプロット ---
plot(t_plot, p_est_x_plot, 'Color', 'r', 'LineStyle', '-', 'LineWidth', 2); hold on; % 荷物X: 橙・実線
plot(t_plot, p_est_y_plot, 'Color', 'b', 'LineStyle', '-', 'LineWidth', 2);          % 荷物Y: 黄緑・実線
plot(t_plot, p_est_z_plot, 'Color', 'g', 'LineStyle', '-', 'LineWidth', 2);          % 荷物Z: 青・実線

% 4. グラフの装飾設定
grid on;
xlabel('Time [s]', 'FontSize', 12);
ylabel('Drone Position [m]', 'FontSize', 12);


% 軸のフォントと「データが本当にある範囲」への完全絞り込み
set(gca, 'FontSize', 11);
xlim([t_plot(1), t_plot(end)]); % 横軸を表示データの最初から最後まで（余白ゼロ）に固定

% 凡例の設定（それぞれの線の色とスタイルに正確に対応）
legend('Drone\_x', 'Drone\_y', 'Drone\_z', ...
    'Location', 'best', 'NumColumns', 2, 'FontSize', 15);
hold off;

%% =========================================================================
%% ⭐【追加】データがある区間（t > 0）のみプロット（実線・点線の視認性大幅向上版）機体速度
%% =========================================================================

% 1. 102フェーズの中で、さらに時間が 0 秒より大きい要素のインデックス（位置）を見つける
t_pos_idx = (t > 0);

% 2. 時間 t > 0 のデータだけを新しく切り出し（データの末尾までしっかり確保）
t_plot     = t(t_pos_idx);
v_est_x_plot = v_est_x(t_pos_idx);
v_est_y_plot = v_est_y(t_pos_idx);
v_est_z_plot = v_est_z(t_pos_idx);

% 3. グラフの描画開始
figure;

% --- 実測値（推定値）を「濃いめの実線」でプロット ---
plot(t_plot, v_est_x_plot, 'Color', 'r', 'LineStyle', '-', 'LineWidth', 2); hold on; % 荷物X: 橙・実線
plot(t_plot, v_est_y_plot, 'Color', 'b', 'LineStyle', '-', 'LineWidth', 2);          % 荷物Y: 黄緑・実線
plot(t_plot, v_est_z_plot, 'Color', 'g', 'LineStyle', '-', 'LineWidth', 2);          % 荷物Z: 青・実線

% 4. グラフの装飾設定
grid on;
xlabel('Time [s]', 'FontSize', 12);
ylabel('Drone Velocity [m/s]', 'FontSize', 12);


% 軸のフォントと「データが本当にある範囲」への完全絞り込み
set(gca, 'FontSize', 11);
xlim([t_plot(1), t_plot(end)]); % 横軸を表示データの最初から最後まで（余白ゼロ）に固定

% 凡例の設定（それぞれの線の色とスタイルに正確に対応）
legend('Drone\_x', 'Drone\_y', 'Drone\_z', ...
    'Location', 'best', 'NumColumns', 2, 'FontSize', 15);
hold off;

%% =========================================================================
%% ⭐【追加】データがある区間（t > 0）のみプロット（実線・点線の視認性大幅向上版）機体速度
%% =========================================================================

% 1. 102フェーズの中で、さらに時間が 0 秒より大きい要素のインデックス（位置）を見つける
t_pos_idx = (t > 0);

% 2. 時間 t > 0 のデータだけを新しく切り出し（データの末尾までしっかり確保）
t_plot     = t(t_pos_idx);
v_est_x_plot = v_est_x(t_pos_idx);
v_est_y_plot = v_est_y(t_pos_idx);
v_est_z_plot = v_est_z(t_pos_idx);

% 3. グラフの描画開始
figure;

% --- 実測値（推定値）を「濃いめの実線」でプロット ---
plot(t_plot, v_est_x_plot, 'Color', 'r', 'LineStyle', '-', 'LineWidth', 2); hold on; % 荷物X: 橙・実線
plot(t_plot, v_est_y_plot, 'Color', 'b', 'LineStyle', '-', 'LineWidth', 2);          % 荷物Y: 黄緑・実線
plot(t_plot, v_est_z_plot, 'Color', 'g', 'LineStyle', '-', 'LineWidth', 2);          % 荷物Z: 青・実線

% 4. グラフの装飾設定
grid on;
xlabel('Time [s]', 'FontSize', 12);
ylabel('Drone Velocity [m/s]', 'FontSize', 12);


% 軸のフォントと「データが本当にある範囲」への完全絞り込み
set(gca, 'FontSize', 11);
xlim([t_plot(1), t_plot(end)]); % 横軸を表示データの最初から最後まで（余白ゼロ）に固定

% 凡例の設定（それぞれの線の色とスタイルに正確に対応）
legend('Drone\_x', 'Drone\_y', 'Drone\_z', ...
    'Location', 'best', 'NumColumns', 2, 'FontSize', 15);
hold off;
%% =========================================================================
%% ⭐【追加】データがある区間（t > 0）のみプロット（実線・点線の視認性大幅向上版）機体速度
%% =========================================================================

% 1. 102フェーズの中で、さらに時間が 0 秒より大きい要素のインデックス（位置）を見つける
t_pos_idx = (t > 0);

% 2. 時間 t > 0 のデータだけを新しく切り出し（データの末尾までしっかり確保）
t_plot     = t(t_pos_idx);
vL_est_x_plot = vL_est_x(t_pos_idx);
vL_est_y_plot = vL_est_y(t_pos_idx);
vL_est_z_plot = vL_est_z(t_pos_idx);

% 3. グラフの描画開始
figure;

% --- 実測値（推定値）を「濃いめの実線」でプロット ---
plot(t_plot, vL_est_x_plot, 'Color', 'r', 'LineStyle', '-', 'LineWidth', 2); hold on; % 荷物X: 橙・実線
plot(t_plot, vL_est_y_plot, 'Color', 'b', 'LineStyle', '-', 'LineWidth', 2);          % 荷物Y: 黄緑・実線
plot(t_plot, vL_est_z_plot, 'Color', 'g', 'LineStyle', '-', 'LineWidth', 2);          % 荷物Z: 青・実線

% 4. グラフの装飾設定
grid on;
xlabel('Time [s]', 'FontSize', 12);
ylabel('Load Velocity [m/s]', 'FontSize', 12);


% 軸のフォントと「データが本当にある範囲」への完全絞り込み
set(gca, 'FontSize', 11);
xlim([t_plot(1), t_plot(end)]); % 横軸を表示データの最初から最後まで（余白ゼロ）に固定

% 凡例の設定（それぞれの線の色とスタイルに正確に対応）
legend('Load\_x', 'Load\_y', 'Load\_z', ...
    'Location', 'best', 'NumColumns', 2, 'FontSize', 15);
hold off;

%% =========================================================================
%% ⭐【追加】データがある区間（t > 0）のみプロット（実線・点線の視認性大幅向上版）紐の単位ベクトル
%% =========================================================================

% 1. 102フェーズの中で、さらに時間が 0 秒より大きい要素のインデックス（位置）を見つける
t_pos_idx = (t > 0);

% 2. 時間 t > 0 のデータだけを新しく切り出し（データの末尾までしっかり確保）
t_plot     = t(t_pos_idx);
pT_est_x_plot = pT_est_x(t_pos_idx);
pT_est_y_plot = pT_est_y(t_pos_idx);
pT_est_z_plot = pT_est_z(t_pos_idx);

% 3. グラフの描画開始
figure;

% --- 実測値（推定値）を「濃いめの実線」でプロット ---
plot(t_plot, pT_est_x_plot, 'Color', 'r', 'LineStyle', '-', 'LineWidth', 2); hold on; % 荷物X: 橙・実線
plot(t_plot, pT_est_y_plot, 'Color', 'b', 'LineStyle', '-', 'LineWidth', 2);          % 荷物Y: 黄緑・実線
plot(t_plot, pT_est_z_plot, 'Color', 'g', 'LineStyle', '-', 'LineWidth', 2);          % 荷物Z: 青・実線

% 4. グラフの装飾設定
grid on;
xlabel('Time [s]', 'FontSize', 12);
ylabel('Unit direction vector of the link', 'FontSize', 12);


% 軸のフォントと「データが本当にある範囲」への完全絞り込み
set(gca, 'FontSize', 11);
xlim([t_plot(1), t_plot(end)]); % 横軸を表示データの最初から最後まで（余白ゼロ）に固定

% 凡例の設定（それぞれの線の色とスタイルに正確に対応）
legend('link\_x', 'link\_y', 'link\_z', ...
    'Location', 'best', 'NumColumns', 2, 'FontSize', 15);
hold off;
%% =========================================================================
%% ⭐【追加】データがある区間（t > 0）のみプロット（実線・点線の視認性大幅向上版）紐の角度
%% =========================================================================

% 1. 102フェーズの中で、さらに時間が 0 秒より大きい要素のインデックス（位置）を見つける
t_pos_idx = (t > 0);

% 2. 時間 t > 0 のデータだけを新しく切り出し（データの末尾までしっかり確保）
t_plot     = t(t_pos_idx);
pT_est_x_plot = pT_est_x(t_pos_idx);
pT_est_y_plot = pT_est_y(t_pos_idx);
pT_est_z_plot = pT_est_z(t_pos_idx);
theta_total_deg = acosd(-pT_est_z_plot);
theta_x_deg = atand(pT_est_x_plot ./ abs(pT_est_z_plot));
theta_y_deg = atand(pT_est_y_plot ./ abs(pT_est_z_plot));
% 3. グラフの描画開始
figure;

% --- 実測値（推定値）を「濃いめの実線」でプロット ---
plot(t_plot, theta_x_deg, 'Color', 'r', 'LineStyle', '-', 'LineWidth', 2); hold on; % 荷物X: 橙・実線
plot(t_plot, theta_y_deg, 'Color', 'b', 'LineStyle', '-', 'LineWidth', 2);          % 荷物Y: 黄緑・実線
% plot(t_plot, theta_total_deg, 'Color', 'g', 'LineStyle', '-', 'LineWidth', 2);          % 荷物Z: 青・実線

% 4. グラフの装飾設定
grid on;
xlabel('Time [s]', 'FontSize', 12);
ylabel('Cable angle [deg]', 'FontSize', 12);


% 軸のフォントと「データが本当にある範囲」への完全絞り込み
set(gca, 'FontSize', 11);
xlim([t_plot(1), t_plot(end)]); % 横軸を表示データの最初から最後まで（余白ゼロ）に固定

% % 凡例の設定（それぞれの線の色とスタイルに正確に対応）
% legend('Angle\_x', 'Angle\_y', 'Total Tilt Angle\_z', ...
%     'Location', 'best', 'NumColumns', 2, 'FontSize', 15);
% hold off;
% 凡例の設定（それぞれの線の色とスタイルに正確に対応）
legend('Angle\_x', 'Angle\_y', ...
    'Location', 'best', 'NumColumns', 2, 'FontSize', 15);
hold off;
%% =========================================================================
%% ⭐【追加】データがある区間（t > 0）のみプロット（実線・点線の視認性大幅向上版）質量推定
%% =========================================================================

% 1. 102フェーズの中で、さらに時間が 0 秒より大きい要素のインデックス（位置）を見つける
t_pos_idx = (t > 0);

% 2. 時間 t > 0 のデータだけを新しく切り出し（データの末尾までしっかり確保）
t_plot     = t(t_pos_idx);
mL_est_plot = mL_est(t_pos_idx);


% 3. グラフの描画開始
figure;

% --- 実測値（推定値）を「濃いめの実線」でプロット ---
plot(t_plot, mL_est_plot, 'Color', 'b', 'LineStyle', '-', 'LineWidth', 2); hold on; % 荷物X: 橙・実線


% 4. グラフの装飾設定
grid on;
xlabel('Time [s]', 'FontSize', 12);
ylabel('Estimated mass [kg]', 'FontSize', 12);


% 軸のフォントと「データが本当にある範囲」への完全絞り込み
set(gca, 'FontSize', 11);
xlim([t_plot(1), t_plot(end)]); % 横軸を表示データの最初から最後まで（余白ゼロ）に固定

% % 凡例の設定（それぞれの線の色とスタイルに正確に対応）
% legend('link\_x', 'link\_y', 'link\_z', ...
%     'Location', 'best', 'NumColumns', 2, 'FontSize', 15);
hold off;
%% =========================================================================
%% ⭐【改修版】機体姿勢角 q (オイラー角) を rad から deg (度) に変換してプロット
%% =========================================================================
% 1. 時間 t > 0 のデータ領域を抽出
t_pos_idx = (t > 0);
t_plot    = t(t_pos_idx);

% 2. rad から deg (度) への変換 (rad2deg 関数を使用)
%  q(1): Roll (ロール)  [deg]
%  q(2): Pitch (ピッチ) [deg]
%  q(3): Yaw (ヨー)     [deg]
q_est_x_deg = rad2deg(q_est_x(t_pos_idx));
q_est_y_deg = rad2deg(q_est_y(t_pos_idx));
q_est_z_deg = rad2deg(q_est_z(t_pos_idx));

% 3. グラフの描画開始
figure;

% --- 姿勢角 (deg) のプロット ---
plot(t_plot, q_est_x_deg, 'Color', [0.85, 0.325, 0.098], 'LineStyle', '-', 'LineWidth', 1.8); hold on; % 橙: Roll (φ)
plot(t_plot, q_est_y_deg, 'Color', [0.00, 0.447, 0.741], 'LineStyle', '-', 'LineWidth', 1.8);          % 青: Pitch (θ)
plot(t_plot, q_est_z_deg, 'Color', [0.466, 0.674, 0.188], 'LineStyle', '-', 'LineWidth', 1.8);          % 緑: Yaw (ψ)

% 4. グラフの装飾設定
grid on;
xlabel('Time [s]', 'FontSize', 12);
ylabel('Attitude angle [deg]', 'FontSize', 12); % Y軸単位を deg に変更

set(gca, 'FontSize', 11);
xlim([t_plot(1), t_plot(end)]); % 横軸をデータ範囲内に固定

% 凡例の設定（Roll, Pitch, Yaw の表記）
legend('Roll (\phi)', 'Pitch (\theta)', 'Yaw (\psi)', ...
    'Location', 'best', 'NumColumns', 3, 'FontSize', 12);

hold off;
%% =========================================================================
%% ⭐【1. ケーブル角速度 wL (振り子運動速度) のプロット [deg/s]】
%% =========================================================================
% 1. 時間 t > 0 のデータ領域を抽出
t_pos_idx = (t > 0);
t_plot    = t(t_pos_idx);

% 2. rad/s から deg/s (度毎秒) への変換
wL_est_x_deg = rad2deg(wL_est_x(t_pos_idx));
wL_est_y_deg = rad2deg(wL_est_y(t_pos_idx));
wL_est_z_deg = rad2deg(wL_est_z(t_pos_idx));

% 3. グラフの描画開始
figure;

plot(t_plot, wL_est_x_deg, 'Color', [0.85, 0.325, 0.098], 'LineStyle', '-', 'LineWidth', 1.8); hold on; % 橙: wL_x
plot(t_plot, wL_est_y_deg, 'Color', [0.00, 0.447, 0.741], 'LineStyle', '-', 'LineWidth', 1.8);          % 青: wL_y
plot(t_plot, wL_est_z_deg, 'Color', [0.466, 0.674, 0.188], 'LineStyle', '-', 'LineWidth', 1.8);          % 緑: wL_z

% 4. グラフの装飾設定
grid on;
xlabel('Time [s]', 'FontSize', 12);
ylabel('Cable angular velocity \omega_L [deg/s]', 'FontSize', 12); % Y軸単位: deg/s

set(gca, 'FontSize', 11);
xlim([t_plot(1), t_plot(end)]); % 横軸をデータ範囲内に固定

% 凡例の設定
legend('\omega_{Lx} (X-axis)', '\omega_{Ly} (Y-axis)', '\omega_{Lz} (Z-axis)', ...
    'Location', 'best', 'NumColumns', 3, 'FontSize', 12);

title('Cable Angular Velocity (\omega_L)', 'FontSize', 13);
hold off;
%% =========================================================================
%% ⭐【2. 機体角速度 w (ドローン本体の回転速度) のプロット [deg/s]】
%% =========================================================================
% 1. 時間 t > 0 のデータ領域を抽出 (共通t_plotを使用)
% t_plot は上記で定義済み

% 2. rad/s から deg/s (度毎秒) への変換
w_est_x_deg = rad2deg(w_est_x(t_pos_idx));
w_est_y_deg = rad2deg(w_est_y(t_pos_idx));
w_est_z_deg = rad2deg(w_est_z(t_pos_idx));

% 3. グラフの描画開始
figure;

plot(t_plot, w_est_x_deg, 'Color', [0.85, 0.325, 0.098], 'LineStyle', '-', 'LineWidth', 1.8); hold on; % 橙: Roll rate (p)
plot(t_plot, w_est_y_deg, 'Color', [0.00, 0.447, 0.741], 'LineStyle', '-', 'LineWidth', 1.8);          % 青: Pitch rate (q)
plot(t_plot, w_est_z_deg, 'Color', [0.466, 0.674, 0.188], 'LineStyle', '-', 'LineWidth', 1.8);          % 緑: Yaw rate (r)

% 4. グラフの装飾設定
grid on;
xlabel('Time [s]', 'FontSize', 12);
ylabel('Body angular velocity \omega [deg/s]', 'FontSize', 12); % Y軸単位: deg/s

set(gca, 'FontSize', 11);
xlim([t_plot(1), t_plot(end)]); % 横軸をデータ範囲内に固定

% 凡例の設定 (Roll, Pitch, Yaw の回転速度レート表記)
legend('Roll rate (\omega_x)', 'Pitch rate (\omega_y)', 'Yaw rate (\omega_z)', ...
    'Location', 'best', 'NumColumns', 3, 'FontSize', 12);

title('Drone Body Angular Velocity (\omega)', 'FontSize', 13);
hold off;
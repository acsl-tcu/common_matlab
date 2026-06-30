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
load('susupendedLoadavoid4_Log(29-Jun-2026_11_51_02).mat', '-mat');
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
    tmp_thrust(sel_idx) = con_res.tmp(1);
    tmp_roll(sel_idx)   = con_res.tmp(2);
    tmp_pitch(sel_idx)  = con_res.tmp(3);
    tmp_yaw(sel_idx)    = con_res.tmp(4);
    rl(sel_idx)         = con_res.rl(1);
    
    obs_vector = con_res.p_obs{1};
    p_obs_x(sel_idx) = obs_vector(1);
    p_obs_y(sel_idx) = obs_vector(2);
    p_obs_z(sel_idx) = obs_vector(3);
    r_obs(sel_idx)     = con_res.r_obs{1};
    r_minimal(sel_idx) = con_res.r_minimal{1};
    controllertime(sel_idx) = con_res.controllertime(1);
    controllertime_total(sel_idx) = con_res.controllertime_total(1);
    
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




%% 3Dグラフの作成 (障害物 ＆ 目標軌道)
figure;

% 1. 障害物の描画 (1番目のデータを使用)
obs_center_x = p_obs_x(1);
obs_center_y = p_obs_y(1);
obs_center_z = p_obs_z(1);
obs_radius   = r_obs(1);

% 球体のメッシュデータを生成
[X_sphere, Y_sphere, Z_sphere] = sphere(50); 

% 中心位置と半径を適用
X_sphere = X_sphere * obs_radius + obs_center_x;
Y_sphere = Y_sphere * obs_radius + obs_center_y;
Z_sphere = Z_sphere * obs_radius + obs_center_z;

% 半透明の球体を描画
surf(X_sphere, Y_sphere, Z_sphere, 'FaceColor', [1, 0.5, 0], 'EdgeColor', 'none');
alpha(0.6); % 透明度 (0.0:完全に透明 〜 1.0:完全に不透明)
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
legend('Obstacle', 'Reference', 'Location', 'best', 'FontSize', 10);

% 視点の調整 (見やすい3D角度に設定。必要に応じて数値を調整してください)
view(3); 
hold off;

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
legend('Obstacle', 'Payload', 'Reference', ...
       'Location', 'best', 'FontSize', 10);

view(3); 
hold off;

%% 2Dグラフの作成 (横軸: 時間t 軸、縦軸: コントローラ計算時間) [統計値の追加版]
figure;

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

% 3. 有効なデータでメインのプロット (青い実線)
plot(t_valid, c_time_valid, 'b-', 'LineWidth', 1.5);
hold on;

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
figure('Name', 'Load Position vs Reference over Time (t > 0 Only)', 'NumberTitle', 'off');

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
       'Location', 'best', 'NumColumns', 2);
hold off;

%% =========================================================================
%% ⭐【追加】障害物との安全境界距離（点線）と現在の距離（実線）の時系列プロット
%% =========================================================================

% 1. 102フェーズの中で、さらに時間が 0 秒より大きい要素のインデックスを見つける
t_pos_idx = (t > 0);

% 2. 時間 t > 0 のデータを新しく切り出し
t_plot       = t(t_pos_idx);
x_est_plot   = pL_est_x(t_pos_idx); % 荷物の現在位置 X
y_est_plot   = pL_est_y(t_pos_idx); % 荷物の現在位置 Y
z_est_plot   = pL_est_z(t_pos_idx); % 荷物の現在位置 Z

p_obs_x_plot = p_obs_x(t_pos_idx);  % 障害物の中心位置 X
p_obs_y_plot = p_obs_y(t_pos_idx);  % 障害物の中心位置 Y
p_obs_z_plot = p_obs_z(t_pos_idx);  % 障害物の中心位置 Z

r_obs_plot   = r_obs(t_pos_idx);    % 障害物の半径 r_obs
rl_plot      = rl(t_pos_idx);       % 牽引物の半径 rl

% 3. 数理定義に基づき、それぞれの距離を計算
% (a) 衝突の限界となる「境界距離のしきい値」 (r_obs + rl)
collision_limit = r_obs_plot + rl_plot;

% (b) 三次元空間における「現在の互いの中心間距離」 (ノルム計算)
current_distance = sqrt((x_est_plot - p_obs_x_plot).^2 + ...
                        (y_est_plot - p_obs_y_plot).^2 + ...
                        (z_est_plot - p_obs_z_plot).^2);

% =========================================================================
% ⭐【追加機能】一番近づいた時の距離（最小距離）を算出してコマンドウィンドウに表示
% =========================================================================
[min_dist, min_idx] = min(current_distance); % 最小距離とその時のインデックスを取得
corresponding_limit = collision_limit(min_idx); % 最接近時の衝突限界値
safety_margin = min_dist - corresponding_limit; % 安全マージン（どれだけ余裕があったか）

fprintf('\n========================================\n');
fprintf('🔥 【高次CBF検証結果】障害物への最接近データ（t > 0）\n');
fprintf('----------------------------------------\n');
fprintf('一番近づいた時の実際の距離 (最小距離) : %.4f [m]\n', min_dist);
fprintf('その時の衝突限界距離 (r_obs + r_l)    : %.4f [m]\n', corresponding_limit);
fprintf('衝突の壁に対する安全マージン (残り余裕) : %.4f [m]\n', safety_margin);
if safety_margin >= 0
    fprintf('⇒ 判定: 🔴【安全確認】限界の壁を超えずに回避完了しています。\n');
else
    fprintf('⇒ 判定: ⚠️【警告】境界を割り込んでいます。ゲインを調整してください。\n');
end
fprintf('========================================\n\n');


% 4. グラフの描画開始
figure('Name', 'Obstacle Distance vs Safety Limit', 'NumberTitle', 'off');

% 【現在の距離】をハッキリした黒の実線でプロット
plot(t_plot, current_distance, 'k-', 'LineWidth', 2.5); hold on;

% 【限界の壁（衝突距離）】を赤の点線でプロット
plot(t_plot, collision_limit, 'r--', 'LineWidth', 2);

% 5. グラフの装飾設定
grid on;
xlabel('Time [s]', 'FontSize', 12);
ylabel('Distance [m]', 'FontSize', 12);

% 軸のフォントと「データが存在する区間のみ」への完全絞り込み
set(gca, 'FontSize', 11);
xlim([t_plot(1), t_plot(end)]); % 横軸を表示データの最初から最後まで（余白ゼロ）に固定

% 縦軸の範囲を少し広めにとって見やすくする
ylim([0, max(current_distance) * 1.1]);

% 凡例の設定
legend('Current Distance', 'Safety Limit', ...
       'Location', 'best');
hold off;


%% ここから下に以前の plot などのコードを自由に貼り付けてください

% % --- 入力 (controller) ---
% input = log.Data.agent(1).controller.input(:, 1:k); 
% 
% if log.fExp
%     tmp_input = [log.Data.agent(1).inner_input{1, 1:k}];
%     transmitter_input = reshape(tmp_input, [8, length(tmp_input)/8])';
% end
% %まずは現在のフォルダからパスが通っているかを確認
% % このスクリプトが置いてあるフォルダの絶対パスを取得して移動
% mfile_path = fileparts(mfilename('fullpath'));
% cd(mfile_path);
% %%
% newLog1 = simplifyLogger(log);
% % newLog1 = log.Data.agent(1:end);
% % newLog1 = log.Data.agent(2:end);
% t = newLog1.t; %t:時間
% phase = newLog1.phase;%phase:アーミングやフライトなどの状態
% k = newLog1.k;%データ数
% rpy = newLog1.sensor.q;%Attitude上からroll,pitch,yawの実測値
% xyz = newLog1.sensor.p;%Position上からz,y,zの実測値
% rpy_est = newLog1.estimator.q;%上からz,y,zの推定値
% xyz_est = newLog1.estimator.p;%上からroll,pitch,yawの推定値
% v_est = newLog1.estimator.v;%速度の推定値
% w_est = newLog1.estimator.w;%角速度の推定値
% rpy_r = newLog1.reference.q;%上からz,y,zの指令値
% xyz_r = newLog1.reference.p;%上からroll,pitch,yawの指令値
% v_r = newLog1.reference.v;%速度の指令値
% input = newLog1.controller.input;%入力
% transmitter_input = newLog1.inner_input;%プロポからの指令
% % %%
% % newLog2 = simplifyLogger(log);
% % t2 = newLog2.t; %t:時間
% % phase2 = newLog2.phase;%phase:アーミングやフライトなどの状態
% % k2 = newLog2.k;%データ数
% % rpy2 = newLog2.sensor.q;%Attitude上からroll,pitch,yawの実測値
% % xyz2 = newLog2.sensor.p;%Position上からz,y,zの実測値
% % rpy_est2 = newLog2.estimator.q;%上からz,y,zの推定値
% % xyz_est2 = newLog2.estimator.p;%上からroll,pitch,yawの推定値
% % v_est2 = newLog2.estimator.v;%速度の推定値
% % w_est2 = newLog2.estimator.w;%角速度の推定値
% % rpy_r2 = newLog2.reference.q;%上からz,y,zの指令値(目標)
% % xyz_r2 = newLog2.reference.p;%上からroll,pitch,yawの指令値
% % v_r2 = newLog2.reference.v;%速度の指令値
% % input2 = newLog2.controller.input;%入力
% % transmitter_input2 = newLog2.inner_input;%プロポからの指令
% 
% %%
% %それぞれのlogのデータの詳しい抜き出し
% 
% roll_s = rpy(1, :);
% pitch_s = rpy(2, :);
% yaw_s = rpy(3, :);
% x_s = xyz(1, :);
% y_s = xyz(2, :);
% z_s = xyz(3, :);
% roll_est = rpy_est(1, :);
% pitch_est = rpy_est(2, :);
% yaw_est = rpy_est(3, :);
% x_est = xyz_est(1, :);
% y_est = xyz_est(2, :);
% z_est = xyz_est(3, :);
% vx_est = v_est(1, :);
% vy_est = v_est(2, :);
% vz_est = v_est(3, :);
% wx_est = w_est(1, :);
% wy_est = w_est(2, :);
% wz_est = w_est(3, :);
% roll_ref = rpy_r(1, :);
% pitch_ref = rpy_r(2, :);
% yaw_ref = rpy_r(3, :);
% x_ref = xyz_r(1, :);
% y_ref = xyz_r(2, :);
% z_ref = xyz_r(3, :);
% vx_ref = v_r(1, :);
% vy_ref = v_r(2, :);
% vz_ref = v_r(3, :);
% 
% % roll_s2 = rpy2(1, :);
% % pitch_s2 = rpy2(2, :);
% % yaw_s2 = rpy2(3, :);
% % x_s2 = xyz2(1, :);
% % y_s2 = xyz2(2, :);
% % z_s2 = xyz2(3, :);
% % roll_est2 = rpy_est2(1, :);
% % pitch_est2 = rpy_est2(2, :);
% % yaw_est2 = rpy_est2(3, :);
% % x_est2 = xyz_est2(1, :);
% % y_est2 = xyz_est2(2, :);
% % z_est2 = xyz_est2(3, :);
% % vx_est2 = v_est2(1, :);
% % vy_est2 = v_est2(2, :);
% % vz_est2 = v_est2(3, :);
% % wx_est2 = w_est2(1, :);
% % wy_est2 = w_est2(2, :);
% % wz_est2 = w_est2(3, :);
% % roll_ref2 = rpy_r2(1, :);
% % pitch_ref2 = rpy_r2(2, :);
% % yaw_ref2 = rpy_r2(3, :);
% % x_ref2 = xyz_r2(1, :);
% % y_ref2 = xyz_r2(2, :);
% % z_ref2 = xyz_r2(3, :);
% % vx_ref2 = v_r2(1, :);
% % vy_ref2 = v_r2(2, :);
% % vz_ref2 = v_r2(3, :);
% %%
% % %xyの軌道と目標軌道の比較
% % figure;
% % plot(x_est, y_est, '-','LineWidth',2);
% % grid on
% % xlabel('X[m]') 
% % ylabel('Y[m]')
% % set(gca().XAxis, 'Fontsize', 12)
% % set(gca().YAxis, 'Fontsize', 12)
% % daspect([1 1 1])
% % xlim([-1.5 1.5])
% % ylim([-1.5 1.5])
% % hold on
% % plot(x_ref, y_ref, '--','LineWidth',2);
% % legend('Estimater','Reference','fontsize',12)
% % hold off
% % 
% % % figure;
% % % plot(x_est2, y_est2, '-','LineWidth',2);
% % % grid on
% % % xlabel('X[m]') 
% % % ylabel('Y[m]')
% % % set(gca().XAxis, 'Fontsize', 12)
% % % set(gca().YAxis, 'Fontsize', 12)
% % % daspect([1 1 1])
% % % xlim([-1.5 1.5])
% % % ylim([-1.5 1.5])
% % % hold on
% % % plot(x_ref2, y_ref2, '--','LineWidth',2);
% % % legend('Estimater','Reference','fontsize',12)
% % % hold off
% 
% %%
% 
% % %xyzの時間変化による目標軌道との比較
% % figure;
% % plot(t, x_est, '-','LineWidth',2);
% % grid on
% % xlabel('Time[s]','FontSize',12) 
% % ylabel('Trajectory[m]','FontSize',12)
% % set(gca().XAxis, 'Fontsize', 12)
% % set(gca().YAxis, 'Fontsize', 12)
% % xlim([0 inf])
% % hold on
% % plot(t, y_est, '-','LineWidth',2);
% % plot(t, z_est, '-','LineWidth',2);
% % plot(t, x_ref, '--','LineWidth',2);
% % plot(t, y_ref, '--','LineWidth',2);
% % plot(t, z_ref, '--','LineWidth',2);
% % legend('X-estimator','Y-estimator','Z-estimator', ...
% %     'X-reference','Y-reference','Z-reference','Location', ...
% %     'southwest','fontsize',12)
% % hold off
% % 
% % % figure;
% % % plot(t2, x_est2, '-','LineWidth',2);
% % % grid on
% % % xlabel('Time[s]','FontSize',12) 
% % % ylabel('Trajectory[m]','FontSize',12)
% % % set(gca().XAxis, 'Fontsize', 12)
% % % set(gca().YAxis, 'Fontsize', 12)
% % % xlim([0 inf])
% % % hold on
% % % plot(t2, y_est2, '-','LineWidth',2);
% % % plot(t2, z_est2, '-','LineWidth',2);
% % % plot(t2, x_ref2, '--','LineWidth',2);
% % % plot(t2, y_ref2, '--','LineWidth',2);
% % % plot(t2, z_ref2, '--','LineWidth',2);
% % % legend('X-estimator','Y-estimator','Z-estimator', ...
% % %     'X-reference','Y-reference','Z-reference','Location', ...
% % %     'southwest','fontsize',12)
% % % hold off
% % 
% % a = 1;
% % 
% % while a <= k
% %     x_error(1,a) = x_est(1,a) - x_ref(1,a);
% %     y_error(1,a) = y_est(1,a) - y_ref(1,a);
% %     z_error(1,a) = z_est(1,a) - z_ref(1,a);
% %     a = a + 1;
% % end
% % 
% % % b = 1;
% % % 
% % % while b <= k2
% % %     x_error2(1,b) = x_est2(1,b) - x_ref2(1,b);
% % %     y_error2(1,b) = y_est2(1,b) - y_ref2(1,b);
% % %     z_error2(1,b) = z_est2(1,b) - z_ref2(1,b);
% % %     b = b + 1;
% % % end
% % 
% % figure;
% % plot(t, x_error, '-','LineWidth',2);
% % grid on
% % xlabel('Time[s]','FontSize',12) 
% % ylabel('Trajectory[m]','FontSize',12)
% % set(gca().XAxis, 'Fontsize', 12)
% % set(gca().YAxis, 'Fontsize', 12)
% % xlim([0 inf])
% % hold on
% % plot(t, y_error, '-','LineWidth',2);
% % plot(t, z_error, '-','LineWidth',2);
% % legend('X-error','Y-error','Z-error','Location', ...
% %     'southwest','fontsize',12)
% % hold off
% % 
% % % figure;
% % % plot(t2, x_error2, '-','LineWidth',2);
% % % grid on
% % % xlabel('Time[s]','FontSize',12) 
% % % ylabel('Trajectory[m]','FontSize',12)
% % % set(gca().XAxis, 'Fontsize', 12)
% % % set(gca().YAxis, 'Fontsize', 12)
% % % xlim([0 inf])
% % % hold on
% % % plot(t2, y_error2, '-','LineWidth',2);
% % % plot(t2, z_error2, '-','LineWidth',2);
% % % legend('X-error','Y-error','Z-error','Location', ...
% % %     'southwest','fontsize',12)
% % % hold off
% 
% %%
% % %vの時間変化による目標速度との比較
% % figure;
% % plot(t, vx_est, '-','LineWidth',2);
% % grid on
% % xlabel('Time[s]','FontSize',12) 
% % ylabel('Velocity[m/s]','FontSize',12)
% % set(gca().XAxis, 'Fontsize', 12)
% % set(gca().YAxis, 'Fontsize', 12)
% % xlim([0 inf])
% % hold on
% % plot(t, vy_est, '-','LineWidth',2);
% % plot(t, vz_est, '-','LineWidth',2);
% % plot(t, vx_ref, '--','LineWidth',2);
% % plot(t, vy_ref, '--','LineWidth',2);
% % plot(t, vz_ref, '--','LineWidth',2);
% % legend('VX-estimator','VY-estimator','VZ-estimator', ...
% %     'VX-reference','VY-reference','VZ-reference','Location', ...
% %     'southwest','fontsize',12)
% % hold off
% % 
% % % figure;
% % % plot(t2, vx_est2, '-','LineWidth',2);
% % % grid on
% % % xlabel('Time[s]','FontSize',12) 
% % % ylabel('Velocity[m/s]','FontSize',12)
% % % set(gca().XAxis, 'Fontsize', 12)
% % % set(gca().YAxis, 'Fontsize', 12)
% % % xlim([0 inf])
% % % hold on
% % % plot(t2, vy_est2, '-','LineWidth',2);
% % % plot(t2, vz_est2, '-','LineWidth',2);
% % % plot(t2, vx_ref2, '--','LineWidth',2);
% % % plot(t2, vy_ref2, '--','LineWidth',2);
% % % plot(t2, vz_ref2, '--','LineWidth',2);
% % % legend('VX-sensor','VY-sensor','VZ-sensor', ...
% % %     'VX-estimator','VY-estimator','VZ-estimator','Location', ...
% % %     'southwest','fontsize',12)
% % % hold off
% % 
% % a = 1;
% % 
% % while a <= k
% %     vx_error(1,a) = vx_est(1,a) - vx_ref(1,a);
% %     vy_error(1,a) = vy_est(1,a) - vy_ref(1,a);
% %     vz_error(1,a) = vz_est(1,a) - vz_ref(1,a);
% %     a = a + 1;
% % end
% % 
% % % b = 1;
% % % 
% % % while b <= k2
% % %     vx_error2(1,b) = vx_est2(1,b) - vx_ref2(1,b);
% % %     vy_error2(1,b) = vy_est2(1,b) - vy_ref2(1,b);
% % %     vz_error2(1,b) = vz_est2(1,b) - vz_ref2(1,b);
% % %     b = b + 1;
% % % end
% % 
% % figure;
% % plot(t, vx_error, '-','LineWidth',2);
% % grid on
% % xlabel('Time[s]','FontSize',12) 
% % ylabel('Velocity[m/s]','FontSize',12)
% % set(gca().XAxis, 'Fontsize', 12)
% % set(gca().YAxis, 'Fontsize', 12)
% % xlim([0 inf])
% % ylim([-1 1])
% % hold on
% % plot(t, vy_error, '-','LineWidth',2);
% % plot(t, vz_error, '-','LineWidth',2);
% % legend('VX-error','VY-error','VZ-error','Location', ...
% %     'southwest','fontsize',12)
% % hold off
% % 
% % % figure;
% % % plot(t2, vx_error2, '-','LineWidth',2);
% % % grid on
% % % xlabel('Time[s]','FontSize',12) 
% % % ylabel('Velocity[m/s]','FontSize',12)
% % % set(gca().XAxis, 'Fontsize', 12)
% % % set(gca().YAxis, 'Fontsize', 12)
% % % xlim([0 inf])
% % % ylim([-1 1])
% % % hold on
% % % plot(t2, vy_error2, '-','LineWidth',2);
% % % plot(t2, vz_error2, '-','LineWidth',2);
% % % legend('VX-error','VY-error','VZ-error','Location', ...
% % %     'southwest','fontsize',12)
% % % hold off
% 
% %%
% %xyの軌道と目標軌道の比較:flightのみ抜き出し
% aa = 1;
% ba = height(t);
% x_est_sel = [];
% y_est_sel = [];
% x_ref_sel = [];
% y_ref_sel = [];
% while aa <= ba
%     if phase(aa,1) == 102
%         x_est_sel = [x_est_sel,x_est(1,aa)];
%         y_est_sel = [y_est_sel,y_est(1,aa)];
%         x_ref_sel = [x_ref_sel,x_ref(1,aa)];
%         y_ref_sel = [y_ref_sel,y_ref(1,aa)];
%     end
%     aa = aa + 1;
% end
% 
% % aa2 = 1;
% % ba2 = height(t2);
% % x_est2_sel = [];
% % y_est2_sel = [];
% % x_ref2_sel = [];
% % y_ref2_sel = [];
% % while aa2 <= ba2
% %     if phase(aa2,1) == 102
% %         x_est2_sel = [x_est2_sel,x_est2(1,aa2)];
% %         y_est2_sel = [y_est2_sel,y_est2(1,aa2)];
% %         x_ref2_sel = [x_ref2_sel,x_ref2(1,aa2)];
% %         y_ref2_sel = [y_ref2_sel,y_ref2(1,aa2)];
% %     end
% %     aa2 = aa2 + 1;
% % end
% 
% figure;
% plot(x_est_sel, y_est_sel, '-','LineWidth',2);
% grid on
% xlabel('X[m]') 
% ylabel('Y[m]')
% set(gca().XAxis, 'Fontsize', 12)
% set(gca().YAxis, 'Fontsize', 12)
% daspect([1 1 1])
% xlim([-1.5 1.5])
% ylim([-1.5 1.5])
% hold on
% plot(x_ref_sel, y_ref_sel, '--','LineWidth',2);
% legend('Estimater','Reference','fontsize',12)
% hold off
% 
% % figure;
% % plot(x_est2_sel, y_est2_sel, '-','LineWidth',2);
% % grid on
% % xlabel('X[m]') 
% % ylabel('Y[m]')
% % set(gca().XAxis, 'Fontsize', 12)
% % set(gca().YAxis, 'Fontsize', 12)
% % daspect([1 1 1])
% % xlim([-1.5 1.5])
% % ylim([-1.5 1.5])
% % hold on
% % plot(x_ref2_sel, y_ref2_sel, '--','LineWidth',2);
% % legend('Estimater','Reference','fontsize',12)
% % hold off
% 
% %%
% %xyzの時間変化による目標軌道との比較 フライト時のみ
% aa = 1;
% ba = height(t);
% x_est_sel = [];
% y_est_sel = [];
% z_est_sel = [];
% x_ref_sel = [];
% y_ref_sel = [];
% z_ref_sel = [];
% t_sel = [];
% while aa <= ba
%     if phase(aa,1) == 102
%         x_est_sel = [x_est_sel,x_est(1,aa)];
%         y_est_sel = [y_est_sel,y_est(1,aa)];
%         z_est_sel = [z_est_sel,z_est(1,aa)];
%         x_ref_sel = [x_ref_sel,x_ref(1,aa)];
%         y_ref_sel = [y_ref_sel,y_ref(1,aa)];
%         z_ref_sel = [z_ref_sel,z_ref(1,aa)];
%         t_sel = [t_sel,t(aa,1)];
%     end
%     aa = aa + 1;
% end
% 
% figure;
% plot(t_sel, x_est_sel, '-','LineWidth',2);
% grid on
% xlabel('Time[s]','FontSize',12) 
% ylabel('Trajectory[m]','FontSize',12)
% set(gca().XAxis, 'Fontsize', 12)
% set(gca().YAxis, 'Fontsize', 12)
% xlim([t_sel(1,1) inf])
% ylim([-1.5 1.5])
% hold on
% plot(t_sel, y_est_sel, '-','LineWidth',2);
% plot(t_sel, z_est_sel, '-','LineWidth',2);
% plot(t_sel, x_ref_sel, '--','LineWidth',2);
% plot(t_sel, y_ref_sel, '--','LineWidth',2);
% plot(t_sel, z_ref_sel, '--','LineWidth',2);
% legend('X-estimator','Y-estimator','Z-estimator', ...
%     'X-reference','Y-reference','Z-reference','Location', ...
%     'southwest','fontsize',8,'NumColumns',2)
% hold off
% 
% % aa2 = 1;
% % ba2 = height(t);
% % x_est2_sel = [];
% % y_est2_sel = [];
% % z_est2_sel = [];
% % x_ref2_sel = [];
% % y_ref2_sel = [];
% % z_ref2_sel = [];
% % t2_sel = [];
% % 
% % figure;
% % plot(t2_sel, x_est2_sel, '-','LineWidth',2);
% % grid on
% % xlabel('Time[s]','FontSize',12) 
% % ylabel('Trajectory[m]','FontSize',12)
% % set(gca().XAxis, 'Fontsize', 12)
% % set(gca().YAxis, 'Fontsize', 12)
% % xlim([0 inf])
% % hold on
% % plot(t2_sel, y_est2_sel, '-','LineWidth',2);
% % plot(t2_sel, z_est2_sel, '-','LineWidth',2);
% % plot(t2_sel, x_ref2_sel, '--','LineWidth',2);
% % plot(t2_sel, y_ref2_sel, '--','LineWidth',2);
% % plot(t2_sel, z_ref2_sel, '--','LineWidth',2);
% % legend('X-estimator','Y-estimator','Z-estimator', ...
% %     'X-reference','Y-reference','Z-reference','Location', ...
% %     'southwest','fontsize',12)
% % hold off
% 
% x_error_sel = [];
% y_error_sel = [];
% z_error_sel = [];
%     x_error_sel = x_est_sel - x_ref_sel;
%     y_error_sel = y_est_sel - y_ref_sel;
%     z_error_sel = z_est_sel - z_ref_sel;
% 
%     avg_x_error = mean(x_error_sel);        
%     disp(avg_x_error);    
%     avg_y_error = mean(y_error_sel);        
%     disp(avg_y_error); 
%     avg_z_error = mean(z_error_sel);        
%     disp(avg_z_error);   
% % x_error2_sel = [];
% % y_error2_sel = [];
% % z_error2_sel = [];
% % 
% %     x_error2_sel = x_est2_sel - x_ref2_sel;
% %     y_error2_sel = y_est2_sel - y_ref2_sel;
% %     z_error2_sel = z_est2_sel - z_ref2_sel;
% 
% figure;
% plot(t_sel, x_error_sel, '-','LineWidth',2);
% grid on
% xlabel('Time[s]','FontSize',12) 
% ylabel('Trajectory[m]','FontSize',12)
% set(gca().XAxis, 'Fontsize', 12)
% set(gca().YAxis, 'Fontsize', 12)
% xlim([t_sel(1,1) inf])
% ylim([-1 1])
% hold on
% plot(t_sel, y_error_sel, '-','LineWidth',2);
% plot(t_sel, z_error_sel, '-','LineWidth',2);
% legend('X-error','Y-error','Z-error','Location', ...
%     'southwest','fontsize',8)
% hold off
% 
% % figure;
% % plot(t2_sel, x_error2_sel, '-','LineWidth',2);
% % grid on
% % xlabel('Time[s]','FontSize',12) 
% % ylabel('Trajectory[m]','FontSize',12)
% % set(gca().XAxis, 'Fontsize', 12)
% % set(gca().YAxis, 'Fontsize', 12)
% % xlim([t2_sel(1,1) inf])
% % hold on
% % plot(t2_sel, y_error2_sel, '-','LineWidth',2);
% % plot(t2_sel, z_error2_sel, '-','LineWidth',2);
% % legend('X-error','Y-error','Z-error','Location', ...
% %     'southwest','fontsize',8)
% % hold off
% 
% %%
% %vの時間変化による目標速度との比較 フライト中のみ
% aa = 1;
% ba = height(t);
% vx_est_sel = [];
% vy_est_sel = [];
% vz_est_sel = [];
% vx_ref_sel = [];
% vy_ref_sel = [];
% vz_ref_sel = [];
% t_sel = [];
% while aa <= ba
%     if phase(aa,1) == 102
%         vx_est_sel = [vx_est_sel,vx_est(1,aa)];
%         vy_est_sel = [vy_est_sel,vy_est(1,aa)];
%         vz_est_sel = [vz_est_sel,vz_est(1,aa)];
%         vx_ref_sel = [vx_ref_sel,vx_ref(1,aa)];
%         vy_ref_sel = [vy_ref_sel,vy_ref(1,aa)];
%         vz_ref_sel = [vz_ref_sel,vz_ref(1,aa)];
%         t_sel = [t_sel,t(aa,1)];
%     end
%     aa = aa + 1;
% end
% 
% 
% figure;
% plot(t_sel, vx_est_sel, '-','LineWidth',2);
% grid on
% xlabel('Time[s]','FontSize',12) 
% ylabel('Velocity[m/s]','FontSize',12)
% set(gca().XAxis, 'Fontsize', 12)
% set(gca().YAxis, 'Fontsize', 12)
% xlim([t_sel(1,1) inf])
% ylim([-1.5 1.5])
% hold on
% plot(t_sel, vy_est_sel, '-','LineWidth',2);
% plot(t_sel, vz_est_sel, '-','LineWidth',2);
% plot(t_sel, vx_ref_sel, '--','LineWidth',2);
% plot(t_sel, vy_ref_sel, '--','LineWidth',2);
% plot(t_sel, vz_ref_sel, '--','LineWidth',2);
% legend('VX-estimator','VY-estimator','VZ-estimator', ...
%     'VX-reference','VY-reference','VZ-reference','Location', ...
%     'southwest','fontsize',8,'NumColumns',2)
% hold off
% 
% % figure;
% % plot(t2, vx_est2, '-','LineWidth',2);
% % grid on
% % xlabel('Time[s]','FontSize',12) 
% % ylabel('Velocity[m/s]','FontSize',12)
% % set(gca().XAxis, 'Fontsize', 12)
% % set(gca().YAxis, 'Fontsize', 12)
% % xlim([0 inf])
% % hold on
% % plot(t2, vy_est2, '-','LineWidth',2);
% % plot(t2, vz_est2, '-','LineWidth',2);
% % plot(t2, vx_ref2, '--','LineWidth',2);
% % plot(t2, vy_ref2, '--','LineWidth',2);
% % plot(t2, vz_ref2, '--','LineWidth',2);
% % legend('VX-sensor','VY-sensor','VZ-sensor', ...
% %     'VX-estimator','VY-estimator','VZ-estimator','Location', ...
% %     'southwest','fontsize',12)
% % hold off
% 
% vx_error_sel = [];
% vy_error_sel = [];
% vz_error_sel = [];
%     vx_error_sel = vx_est_sel - vx_ref_sel;
%     vy_error_sel = vy_est_sel - vy_ref_sel;
%     vz_error_sel = vz_est_sel - vz_ref_sel;
% 
%     avg_vx_error = mean(vx_error_sel);        
%     disp(avg_vx_error);    
%     avg_vy_error = mean(vy_error_sel);        
%     disp(avg_vy_error); 
%     avg_vz_error = mean(vz_error_sel);        
%     disp(avg_vz_error);
% % while b <= k2
% %     vx_error2(1,b) = vx_est2(1,b) - vx_ref2(1,b);
% %     vy_error2(1,b) = vy_est2(1,b) - vy_ref2(1,b);
% %     vz_error2(1,b) = vz_est2(1,b) - vz_ref2(1,b);
% %     b = b + 1;
% % end
% 
% figure;
% plot(t_sel, vx_error_sel, '-','LineWidth',2);
% grid on
% xlabel('Time[s]','FontSize',12) 
% ylabel('Velocity[m/s]','FontSize',12)
% set(gca().XAxis, 'Fontsize', 12)
% set(gca().YAxis, 'Fontsize', 12)
% xlim([t_sel(1,1) inf])
% ylim([-1 1])
% hold on
% plot(t_sel, vy_error_sel, '-','LineWidth',2);
% plot(t_sel, vz_error_sel, '-','LineWidth',2);
% legend('VX-error','VY-error','VZ-error','Location', ...
%     'southwest','fontsize',8)
% hold off
% 
% % figure;
% % plot(t2, vx_error2, '-','LineWidth',2);
% % grid on
% % xlabel('Time[s]','FontSize',12) 
% % ylabel('Velocity[m/s]','FontSize',12)
% % set(gca().XAxis, 'Fontsize', 12)
% % set(gca().YAxis, 'Fontsize', 12)
% % xlim([0 inf])
% % ylim([-1 1])
% % hold on
% % plot(t2, vy_error2, '-','LineWidth',2);
% % plot(t2, vz_error2, '-','LineWidth',2);
% % legend('VX-error','VY-error','VZ-error','Location', ...
% %     'southwest','fontsize',12)
% % hold off
% 
% %% 3Dの軌道
% aa = 1;
% ba = height(t);
% x_est_sel = [];
% y_est_sel = [];
% z_est_sel = [];
% x_ref_sel = [];
% y_ref_sel = [];
% z_ref_sel = [];
% t_sel = [];
% while aa <= ba
%     if phase(aa,1) == 102
%         x_est_sel = [x_est_sel,x_est(1,aa)];
%         y_est_sel = [y_est_sel,y_est(1,aa)];
%         z_est_sel = [z_est_sel,z_est(1,aa)];
%         x_ref_sel = [x_ref_sel,x_ref(1,aa)];
%         y_ref_sel = [y_ref_sel,y_ref(1,aa)];
%         z_ref_sel = [z_ref_sel,z_ref(1,aa)];
%         t_sel = [t_sel,t(aa,1)];
%     end
%     aa = aa + 1;
% end
% 
% 
% 
% plot3(x_est_sel, y_est_sel, z_est_sel,  '-','LineWidth', 2);  % 軌道の太さを指定
% grid on                         % グリッドを表示
% xlabel('X[m]','FontSize',12) 
% ylabel('Y[m]','FontSize',12)
% zlabel('Z[m]','FontSize',12)
% set(gca().XAxis, 'Fontsize', 12)
% set(gca().YAxis, 'Fontsize', 12)
% set(gca().ZAxis, 'Fontsize', 12)
% xlim([-1.5 1.5])
% ylim([-1.5 1.5])
% zlim([0 2])
% hold on
% plot3(x_ref_sel, y_ref_sel, z_ref_sel, '--', 'LineWidth', 2)
% legend('Estimator','Reference','Location', ...
%     'southwest','fontsize',8)
% hold off
% 
% %%
% % %xyの軌道と目標軌道の比較:flightのみ抜き出し
% % aa = 1;
% % ba = height(t);
% % x_est_sel = [];
% % y_est_sel = [];
% % x_s_sel = [];
% % y_s_sel = [];
% % while aa <= ba
% %     if phase(aa,1) == 102
% %         x_est_sel = [x_est_sel,x_est(1,aa)];
% %         y_est_sel = [y_est_sel,y_est(1,aa)];
% %         x_s_sel = [x_s_sel,x_s(1,aa)];
% %         y_s_sel = [y_ref_sel,y_s(1,aa)];
% %     end
% %     aa = aa + 1;
% % end
% % 
% % % aa2 = 1;
% % % ba2 = height(t2);
% % % x_est2_sel = [];
% % % y_est2_sel = [];
% % % x_ref2_sel = [];
% % % y_ref2_sel = [];
% % % while aa2 <= ba2
% % %     if phase(aa2,1) == 102
% % %         x_est2_sel = [x_est2_sel,x_est2(1,aa2)];
% % %         y_est2_sel = [y_est2_sel,y_est2(1,aa2)];
% % %         x_ref2_sel = [x_ref2_sel,x_ref2(1,aa2)];
% % %         y_ref2_sel = [y_ref2_sel,y_ref2(1,aa2)];
% % %     end
% % %     aa2 = aa2 + 1;
% % % end
% % 
% % figure;
% % plot(x_est_sel, y_est_sel, '-','LineWidth',2);
% % grid on
% % xlabel('X[m]') 
% % ylabel('Y[m]')
% % set(gca().XAxis, 'Fontsize', 12)
% % set(gca().YAxis, 'Fontsize', 12)
% % daspect([1 1 1])
% % xlim([-1.5 1.5])
% % ylim([-1.5 1.5])
% % hold on
% % plot(x_s_sel, y_ref_sel, '--','LineWidth',2);
% % legend('Estimater','Reference','fontsize',12)
% % hold off
% % 
% % % figure;
% % % plot(x_est2_sel, y_est2_sel, '-','LineWidth',2);
% % % grid on
% % % xlabel('X[m]') 
% % % ylabel('Y[m]')
% % % set(gca().XAxis, 'Fontsize', 12)
% % % set(gca().YAxis, 'Fontsize', 12)
% % % daspect([1 1 1])
% % % xlim([-1.5 1.5])
% % % ylim([-1.5 1.5])
% % % hold on
% % % plot(x_ref2_sel, y_ref2_sel, '--','LineWidth',2);
% % % legend('Estimater','Reference','fontsize',12)
% % % hold off
% % 
% % %%
% % %xyzの時間変化による目標軌道との比較 フライト時のみ
% % aa = 1;
% % ba = height(t);
% % x_s_sel = [];
% % y_s_sel = [];
% % z_s_sel = [];
% % x_ref_sel = [];
% % y_ref_sel = [];
% % z_ref_sel = [];
% % t_sel = [];
% % while aa <= ba
% %     if phase(aa,1) == 102
% %         x_s_sel = [x_s_sel,x_s(1,aa)];
% %         y_s_sel = [y_s_sel,y_s(1,aa)];
% %         z_s_sel = [z_s_sel,z_s(1,aa)];
% %         x_ref_sel = [x_ref_sel,x_ref(1,aa)];
% %         y_ref_sel = [y_ref_sel,y_ref(1,aa)];
% %         z_ref_sel = [z_ref_sel,z_ref(1,aa)];
% %         t_sel = [t_sel,t(aa,1)];
% %     end
% %     aa = aa + 1;
% % end
% % 
% % figure;
% % plot(t_sel, x_s_sel, '-','LineWidth',2);
% % grid on
% % xlabel('Time[s]','FontSize',12) 
% % ylabel('Trajectory[m]','FontSize',12)
% % set(gca().XAxis, 'Fontsize', 12)
% % set(gca().YAxis, 'Fontsize', 12)
% % xlim([t_sel(1,1) inf])
% % hold on
% % plot(t_sel, y_s_sel, '-','LineWidth',2);
% % plot(t_sel, z_s_sel, '-','LineWidth',2);
% % plot(t_sel, x_ref_sel, '--','LineWidth',2);
% % plot(t_sel, y_ref_sel, '--','LineWidth',2);
% % plot(t_sel, z_ref_sel, '--','LineWidth',2);
% % legend('X-estimator','Y-estimator','Z-estimator', ...
% %     'X-reference','Y-reference','Z-reference','Location', ...
% %     'southwest','fontsize',8,'NumColumns',2)
% % hold off
% % 
% % % aa2 = 1;
% % % ba2 = height(t);
% % % x_est2_sel = [];
% % % y_est2_sel = [];
% % % z_est2_sel = [];
% % % x_ref2_sel = [];
% % % y_ref2_sel = [];
% % % z_ref2_sel = [];
% % % t2_sel = [];
% % % 
% % % figure;
% % % plot(t2_sel, x_est2_sel, '-','LineWidth',2);
% % % grid on
% % % xlabel('Time[s]','FontSize',12) 
% % % ylabel('Trajectory[m]','FontSize',12)
% % % set(gca().XAxis, 'Fontsize', 12)
% % % set(gca().YAxis, 'Fontsize', 12)
% % % xlim([0 inf])
% % % hold on
% % % plot(t2_sel, y_est2_sel, '-','LineWidth',2);
% % % plot(t2_sel, z_est2_sel, '-','LineWidth',2);
% % % plot(t2_sel, x_ref2_sel, '--','LineWidth',2);
% % % plot(t2_sel, y_ref2_sel, '--','LineWidth',2);
% % % plot(t2_sel, z_ref2_sel, '--','LineWidth',2);
% % % legend('X-estimator','Y-estimator','Z-estimator', ...
% % %     'X-reference','Y-reference','Z-reference','Location', ...
% % %     'southwest','fontsize',12)
% % % hold off
% % 
% % x_error_sel = [];
% % y_error_sel = [];
% % z_error_sel = [];
% %     x_error_sel = x_s_sel - x_ref_sel;
% %     y_error_sel = y_s_sel - y_ref_sel;
% %     z_error_sel = z_s_sel - z_ref_sel;
% % 
% % % x_error2_sel = [];
% % % y_error2_sel = [];
% % % z_error2_sel = [];
% % % 
% % %     x_error2_sel = x_est2_sel - x_ref2_sel;
% % %     y_error2_sel = y_est2_sel - y_ref2_sel;
% % %     z_error2_sel = z_est2_sel - z_ref2_sel;
% % 
% % figure;
% % plot(t_sel, x_error_sel, '-','LineWidth',2);
% % grid on
% % xlabel('Time[s]','FontSize',12) 
% % ylabel('Trajectory[m]','FontSize',12)
% % set(gca().XAxis, 'Fontsize', 12)
% % set(gca().YAxis, 'Fontsize', 12)
% % xlim([t_sel(1,1) inf])
% % hold on
% % plot(t_sel, y_error_sel, '-','LineWidth',2);
% % plot(t_sel, z_error_sel, '-','LineWidth',2);
% % legend('X-error','Y-error','Z-error','Location', ...
% %     'southwest','fontsize',8)
% % hold off
% % 
% % % figure;
% % % plot(t2_sel, x_error2_sel, '-','LineWidth',2);
% % % grid on
% % % xlabel('Time[s]','FontSize',12) 
% % % ylabel('Trajectory[m]','FontSize',12)
% % % set(gca().XAxis, 'Fontsize', 12)
% % % set(gca().YAxis, 'Fontsize', 12)
% % % xlim([t2_sel(1,1) inf])
% % % hold on
% % % plot(t2_sel, y_error2_sel, '-','LineWidth',2);
% % % plot(t2_sel, z_error2_sel, '-','LineWidth',2);
% % % legend('X-error','Y-error','Z-error','Location', ...
% % %     'southwest','fontsize',8)
% % % hold off
% aa = 1;
% ba = height(t);
% x_est_sel = [];
% y_est_sel = [];
% z_est_sel = [];
% x_ref_sel = [];
% y_ref_sel = [];
% z_ref_sel = [];
% t_sel = [];
% while aa <= ba
%     if phase(aa,1) == 102
%         x_est_sel = [x_est_sel,x_est(1,aa)];
%         y_est_sel = [y_est_sel,y_est(1,aa)];
%         z_est_sel = [z_est_sel,z_est(1,aa)];
%         x_ref_sel = [x_ref_sel,x_ref(1,aa)];
%         y_ref_sel = [y_ref_sel,y_ref(1,aa)];
%         z_ref_sel = [z_ref_sel,z_ref(1,aa)];
%         t_sel = [t_sel,t(aa,1)];
%     end
%     aa = aa + 1;
% end
% 
% 
% 
% plot3(x_ref_sel, y_ref_sel, z_ref_sel,  '-','LineWidth', 2);  % 軌道の太さを指定
% grid on                         % グリッドを表示
% xlabel('X[m]','FontSize',12) 
% ylabel('Y[m]','FontSize',12)
% zlabel('Z[m]','FontSize',12)
% set(gca().XAxis, 'Fontsize', 12)
% set(gca().YAxis, 'Fontsize', 12)
% set(gca().ZAxis, 'Fontsize', 12)
% xlim([-1.5 1.5])
% ylim([-1.5 1.5])
% zlim([0 1.5])
% hold on
% legend('Reference','Location', ...
%     'southwest','fontsize',8)
% hold off
% 
% % end
% 
% % plot(x_ref_sel, y_ref_sel,  '-','LineWidth', 2);  % 軌道の太さを指定
% % grid on                         % グリッドを表示
% % xlabel('X[m]','FontSize',12) 
% % ylabel('Y[m]','FontSize',12)
% % zlabel('Z[m]','FontSize',12)
% % set(gca().XAxis, 'Fontsize', 12)
% % set(gca().YAxis, 'Fontsize', 12)
% % xlim([-1.5 1.5])
% % ylim([-1.5 1.5])
% % hold on
% % legend('Reference','Location', ...
% %     'southwest','fontsize',8)
% % hold off

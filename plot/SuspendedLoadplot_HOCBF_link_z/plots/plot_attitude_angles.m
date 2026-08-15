% function plot_attitude_angles(data)
% % ドローン機体姿勢角（Roll, Pitch, Yaw）の時系列プロット関数
% % （deg 表記と rad 表記を2つの独立したウィンドウで出力）
% 
% % 1. 時間 t > 0 の要素のインデックスを見つける
% t_pos_idx = (data.t > 0);
% 
% % 2. 時間 t > 0 のデータだけを切り出し (1行目: Roll, 2行目: Pitch, 3行目: Yaw)
% t_plot    = data.t(t_pos_idx);
% q_est_rad = data.estimator.q(:, t_pos_idx);
% 
% q_est_x_rad = q_est_rad(1, :); % Roll  [rad]
% q_est_y_rad = q_est_rad(2, :); % Pitch [rad]
% q_est_z_rad = q_est_rad(3, :); % Yaw   [rad]
% 
% %% =========================================================================
% %% 3. ウィンドウ 1: 姿勢角 [deg] (Attitude Angle [deg])
% %% =========================================================================
% q_est_x_deg = rad2deg(q_est_x_rad);
% q_est_y_deg = rad2deg(q_est_y_rad);
% q_est_z_deg = rad2deg(q_est_z_rad);
% 
% figure('Name', 'Attitude Angle [deg]');
% 
% plot(t_plot, q_est_x_deg, 'Color', [0.85, 0.325, 0.098], 'LineStyle', '-', 'LineWidth', 1.8); hold on; % 橙: Roll (φ)
% plot(t_plot, q_est_y_deg, 'Color', [0.00, 0.447, 0.741], 'LineStyle', '-', 'LineWidth', 1.8);          % 青: Pitch (θ)
% plot(t_plot, q_est_z_deg, 'Color', [0.466, 0.674, 0.188], 'LineStyle', '-', 'LineWidth', 1.8);          % 緑: Yaw (ψ)
% 
% grid on;
% set(gca, 'FontSize', 14);
% xlabel('Time [s]', 'FontSize', 16);
% ylabel('Attitude angle [deg]', 'FontSize', 16);
% title('Attitude Angle [deg]', 'FontSize', 18);
% 
% xlim([t_plot(1), t_plot(end)]);
% 
% legend('Roll (\phi)', 'Pitch (\theta)', 'Yaw (\psi)', ...
%        'Location', 'best', 'NumColumns', 3, 'FontSize', 14);
% hold off;
% 
% %% =========================================================================
% %% 4. ウィンドウ 2: 姿勢角 [rad] (Attitude Angle [rad])
% %% =========================================================================
% figure('Name', 'Attitude Angle [rad]');
% 
% plot(t_plot, q_est_x_rad, 'Color', [0.85, 0.325, 0.098], 'LineStyle', '-', 'LineWidth', 1.8); hold on; % 橙: Roll (φ)
% plot(t_plot, q_est_y_rad, 'Color', [0.00, 0.447, 0.741], 'LineStyle', '-', 'LineWidth', 1.8);          % 青: Pitch (θ)
% plot(t_plot, q_est_z_rad, 'Color', [0.466, 0.674, 0.188], 'LineStyle', '-', 'LineWidth', 1.8);          % 緑: Yaw (ψ)
% 
% grid on;
% set(gca, 'FontSize', 14);
% xlabel('Time [s]', 'FontSize', 16);
% ylabel('Attitude angle [rad]', 'FontSize', 16);
% title('Attitude Angle [rad]', 'FontSize', 18);
% 
% xlim([t_plot(1), t_plot(end)]);
% 
% legend('Roll (\phi)', 'Pitch (\theta)', 'Yaw (\psi)', ...
%        'Location', 'best', 'NumColumns', 3, 'FontSize', 14);
% hold off;
% 
% end

function plot_attitude_angles(data)
% ドローン機体姿勢角（Roll, Pitch, Yaw）の時系列プロット関数
% （deg 表記と rad 表記を2つの独立したウィンドウで出力）

% 1. 時間 t > 0 の要素のインデックスを見つける
t_pos_idx = (data.t > 0);

% 2. 時間 t > 0 のデータだけを切り出し (1行目: Roll, 2行目: Pitch, 3行目: Yaw)
t_plot    = data.t(t_pos_idx);
q_est_rad = data.estimator.q(:, t_pos_idx);
q_est_x_rad = q_est_rad(1, :); % Roll  [rad]
q_est_y_rad = q_est_rad(2, :); % Pitch [rad]
q_est_z_rad = q_est_rad(3, :); % Yaw   [rad]

%% =========================================================================
%% 画面配置 & サイズ固定の位置計算（ピクセル単位）
%% =========================================================================
fig_width   = 600; % 横幅
fig_height  = 400; % 高さ
screen_size = get(0, 'ScreenSize');

% 画面中央を基準に2つのウィンドウを左右に少しずらして配置
center_x = (screen_size(3) - fig_width) / 2;
center_y = (screen_size(4) - fig_height) / 2;

fig_pos_deg = [center_x - 30, center_y + 20, fig_width, fig_height];
fig_pos_rad = [center_x + 30, center_y - 20, fig_width, fig_height];

%% =========================================================================
%% 3. ウィンドウ 1: 姿勢角 [deg] (Attitude Angle [deg])
%% =========================================================================
q_est_x_deg = rad2deg(q_est_x_rad);
q_est_y_deg = rad2deg(q_est_y_rad);
q_est_z_deg = rad2deg(q_est_z_rad);

figure('Name', 'Attitude Angle [deg]', 'Position', fig_pos_deg);

% --- 各姿勢角のプロット ---
plot(t_plot, q_est_x_deg, 'Color', [0.85, 0.325, 0.098], 'LineStyle', '-', 'LineWidth', 1.8); hold on; % 橙: Roll (\phi)
plot(t_plot, q_est_y_deg, 'Color', [0.00, 0.447, 0.741], 'LineStyle', '-', 'LineWidth', 1.8);          % 青: Pitch (\theta)
plot(t_plot, q_est_z_deg, 'Color', [0.466, 0.674, 0.188], 'LineStyle', '-', 'LineWidth', 1.8);          % 緑: Yaw (\psi)

% グラフの装飾設定
grid on;
set(gca, 'FontSize', 14); % 軸目盛文字サイズ
xlabel('Time [s]', 'FontSize', 16);
ylabel('Attitude angle [deg]', 'FontSize', 16);

% 軸を表示データの最初から最後まで（余白ゼロ）に固定
xlim([t_plot(1), t_plot(end)]);

% Y軸の余白を自動計算
all_deg_data = [q_est_x_deg, q_est_y_deg, q_est_z_deg];
y_min_deg = min(all_deg_data);
y_max_deg = max(all_deg_data);
y_range_deg = y_max_deg - y_min_deg;
if y_range_deg == 0, y_range_deg = 1; end
y_margin_deg = y_range_deg * 0.25;
ylim([y_min_deg - y_range_deg * 0.05, y_max_deg + y_margin_deg]);

% 凡例の設定
legend('Roll (\phi)', 'Pitch (\theta)', 'Yaw (\psi)', ...
       'Location', 'best', 'NumColumns', 3, 'FontSize', 12);
hold off;

%% =========================================================================
%% 4. ウィンドウ 2: 姿勢角 [rad] (Attitude Angle [rad])
%% =========================================================================
figure('Name', 'Attitude Angle [rad]', 'Position', fig_pos_rad);

% --- 各姿勢角のプロット ---
plot(t_plot, q_est_x_rad, 'Color', [0.85, 0.325, 0.098], 'LineStyle', '-', 'LineWidth', 1.8); hold on; % 橙: Roll (\phi)
plot(t_plot, q_est_y_rad, 'Color', [0.00, 0.447, 0.741], 'LineStyle', '-', 'LineWidth', 1.8);          % 青: Pitch (\theta)
plot(t_plot, q_est_z_rad, 'Color', [0.466, 0.674, 0.188], 'LineStyle', '-', 'LineWidth', 1.8);          % 緑: Yaw (\psi)

% グラフの装飾設定
grid on;
set(gca, 'FontSize', 14); % 軸目盛文字サイズ
xlabel('Time [s]', 'FontSize', 16);
ylabel('Attitude angle [rad]', 'FontSize', 16);

% 軸を表示データの最初から最後まで（余白ゼロ）に固定
xlim([t_plot(1), t_plot(end)]);

% Y軸の余白を自動計算
all_rad_data = [q_est_x_rad, q_est_y_rad, q_est_z_rad];
y_min_rad = min(all_rad_data);
y_max_rad = max(all_rad_data);
y_range_rad = y_max_rad - y_min_rad;
if y_range_rad == 0, y_range_rad = 1; end
y_margin_rad = y_range_rad * 0.25;
ylim([y_min_rad - y_range_rad * 0.05, y_max_rad + y_margin_rad]);

% 凡例の設定
legend('Roll (\phi)', 'Pitch (\theta)', 'Yaw (\psi)', ...
       'Location', 'best', 'NumColumns', 3, 'FontSize', 12);
hold off;

end
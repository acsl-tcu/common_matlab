% function plot_cable_states(data)
% % 紐（ケーブル）の単位方向ベクトルおよび角度（deg / rad）の時系列プロット関数
% % （3つの独立したウィンドウで出力）
% 
% % 1. 時間 t > 0 の要素のインデックスを見つける
% t_pos_idx = (data.t > 0);
% 
% % 2. 時間 t > 0 のデータだけを切り出し
% t_plot = data.t(t_pos_idx);
% 
% pT_est_x_plot = data.estimator.pT(1, t_pos_idx);
% pT_est_y_plot = data.estimator.pT(2, t_pos_idx);
% pT_est_z_plot = data.estimator.pT(3, t_pos_idx);
% 
% %% =========================================================================
% %% 3. ウィンドウ 1: 紐の単位方向ベクトル (Unit Direction Vector)
% %% =========================================================================
% figure('Name', 'Cable Unit Direction Vector');
% 
% plot(t_plot, pT_est_x_plot, 'Color', 'r', 'LineStyle', '-', 'LineWidth', 2); hold on; % link_x: 赤・実線
% plot(t_plot, pT_est_y_plot, 'Color', 'b', 'LineStyle', '-', 'LineWidth', 2);          % link_y: 青・実線
% plot(t_plot, pT_est_z_plot, 'Color', 'g', 'LineStyle', '-', 'LineWidth', 2);          % link_z: 緑・実線
% 
% grid on;
% set(gca, 'FontSize', 14);
% xlabel('Time [s]', 'FontSize', 16);
% ylabel('Unit direction vector of the link', 'FontSize', 16);
% title('Cable Unit Direction Vector', 'FontSize', 18);
% 
% xlim([t_plot(1), t_plot(end)]);
% 
% legend('link\_x', 'link\_y', 'link\_z', ...
%        'Location', 'best', 'NumColumns', 2, 'FontSize', 15);
% hold off;
% 
% %% =========================================================================
% %% 4. ウィンドウ 2: 紐の角度 [deg] (Cable Angle [deg])
% %% =========================================================================
% theta_x_deg = atand(pT_est_x_plot ./ abs(pT_est_z_plot));
% theta_y_deg = atand(pT_est_y_plot ./ abs(pT_est_z_plot));
% 
% figure('Name', 'Cable Angle [deg]');
% 
% plot(t_plot, theta_x_deg, 'Color', 'r', 'LineStyle', '-', 'LineWidth', 2); hold on; % Angle_x [deg]: 赤・実線
% plot(t_plot, theta_y_deg, 'Color', 'b', 'LineStyle', '-', 'LineWidth', 2);          % Angle_y [deg]: 青・実線
% 
% grid on;
% set(gca, 'FontSize', 14);
% xlabel('Time [s]', 'FontSize', 16);
% ylabel('Cable angle [deg]', 'FontSize', 16);
% title('Cable Angle [deg]', 'FontSize', 18);
% 
% xlim([t_plot(1), t_plot(end)]);
% 
% legend('Angle\_x', 'Angle\_y', ...
%        'Location', 'best', 'NumColumns', 2, 'FontSize', 15);
% hold off;
% 
% %% =========================================================================
% %% 5. ウィンドウ 3: 紐の角度 [rad] (Cable Angle [rad]) 【追加】
% %% =========================================================================
% theta_x_rad = atan(pT_est_x_plot ./ abs(pT_est_z_plot));
% theta_y_rad = atan(pT_est_y_plot ./ abs(pT_est_z_plot));
% 
% figure('Name', 'Cable Angle [rad]');
% 
% plot(t_plot, theta_x_rad, 'Color', 'r', 'LineStyle', '-', 'LineWidth', 2); hold on; % Angle_x [rad]: 赤・実線
% plot(t_plot, theta_y_rad, 'Color', 'b', 'LineStyle', '-', 'LineWidth', 2);          % Angle_y [rad]: 青・実線
% 
% grid on;
% set(gca, 'FontSize', 14);
% xlabel('Time [s]', 'FontSize', 16);
% ylabel('Cable angle [rad]', 'FontSize', 16);
% title('Cable Angle [rad]', 'FontSize', 18);
% 
% xlim([t_plot(1), t_plot(end)]);
% 
% legend('Angle\_x', 'Angle\_y', ...
%        'Location', 'best', 'NumColumns', 2, 'FontSize', 15);
% hold off;
% 
% end

function plot_cable_states(data)
% 紐（ケーブル）の単位方向ベクトルおよび角度（deg / rad）の時系列プロット関数
% （3つの独立したウィンドウで出力）

% 1. 時間 t > 0 の要素のインデックスを見つける
t_pos_idx = (data.t > 0);

% 2. 時間 t > 0 のデータだけを切り出し
t_plot = data.t(t_pos_idx);
pT_est_x_plot = data.estimator.pT(1, t_pos_idx);
pT_est_y_plot = data.estimator.pT(2, t_pos_idx);
pT_est_z_plot = data.estimator.pT(3, t_pos_idx);

%% =========================================================================
%% 画面中央配置 & サイズ固定の位置計算（ピクセル単位）
%% =========================================================================
fig_width  = 600; % お好みの横幅
fig_height = 400; % お好みの高さ

screen_size = get(0, 'ScreenSize');
pos_x = (screen_size(3) - fig_width) / 2;
pos_y = (screen_size(4) - fig_height) / 2;
fig_pos = [pos_x, pos_y, fig_width, fig_height];

%% =========================================================================
%% 3. ウィンドウ 1: 紐の単位方向ベクトル (Unit Direction Vector)
%% =========================================================================
figure('Name', 'Cable Unit Direction Vector', 'Position', fig_pos);
plot(t_plot, pT_est_x_plot, 'Color', 'r', 'LineStyle', '-', 'LineWidth', 2); hold on; % link_x: 赤・実線
plot(t_plot, pT_est_y_plot, 'Color', 'b', 'LineStyle', '-', 'LineWidth', 2);          % link_y: 青・実線
plot(t_plot, pT_est_z_plot, 'Color', 'g', 'LineStyle', '-', 'LineWidth', 2);          % link_z: 緑・実線
grid on;
set(gca, 'FontSize', 14);
xlabel('Time [s]', 'FontSize', 16);
ylabel('Unit direction vector of the link', 'FontSize', 16);
xlim([t_plot(1), t_plot(end)]);

% ★ Y軸の上下に少し余白を追加（下側: 5%, 上側: 15%）
y_data1  = [pT_est_x_plot, pT_est_y_plot, pT_est_z_plot];
y_min1   = min(y_data1);
y_max1   = max(y_data1);
y_range1 = y_max1 - y_min1;
ylim([y_min1 - y_range1 * 0.05, y_max1 + y_range1 * 0.15]);

legend('link\_x', 'link\_y', 'link\_z', ...
       'Location', 'best', 'NumColumns', 2, 'FontSize', 15);
hold off;

%% =========================================================================
%% 4. ウィンドウ 2: 紐の角度 [deg] (Cable Angle [deg])
%% =========================================================================
theta_x_deg = atand(pT_est_x_plot ./ abs(pT_est_z_plot));
theta_y_deg = atand(pT_est_y_plot ./ abs(pT_est_z_plot));

figure('Name', 'Cable Angle [deg]', 'Position', fig_pos);
plot(t_plot, theta_x_deg, 'Color', 'r', 'LineStyle', '-', 'LineWidth', 2); hold on; % Angle_x [deg]: 赤・実線
plot(t_plot, theta_y_deg, 'Color', 'b', 'LineStyle', '-', 'LineWidth', 2);          % Angle_y [deg]: 青・実線
grid on;
set(gca, 'FontSize', 14);
xlabel('Time [s]', 'FontSize', 16);
ylabel('Cable angle [deg]', 'FontSize', 16);
xlim([t_plot(1), t_plot(end)]);

% ★ Y軸の上下に少し余白を追加（下側: 5%, 上側: 15%）
y_data2  = [theta_x_deg, theta_y_deg];
y_min2   = min(y_data2);
y_max2   = max(y_data2);
y_range2 = y_max2 - y_min2;
ylim([y_min2 - y_range2 * 0.05, y_max2 + y_range2 * 0.15]);

legend('Angle_x', 'Angle_y', ...
       'Location', 'best', 'NumColumns', 2, 'FontSize', 15);
hold off;

%% =========================================================================
%% 5. ウィンドウ 3: 紐の角度 [rad] (Cable Angle [rad])
%% =========================================================================
theta_x_rad = atan(pT_est_x_plot ./ abs(pT_est_z_plot));
theta_y_rad = atan(pT_est_y_plot ./ abs(pT_est_z_plot));

figure('Name', 'Cable Angle [rad]', 'Position', fig_pos);
plot(t_plot, theta_x_rad, 'Color', 'r', 'LineStyle', '-', 'LineWidth', 2); hold on; % Angle_x [rad]: 赤・実線
plot(t_plot, theta_y_rad, 'Color', 'b', 'LineStyle', '-', 'LineWidth', 2);          % Angle_y [rad]: 青・実線
grid on;
set(gca, 'FontSize', 14);
xlabel('Time [s]', 'FontSize', 16);
ylabel('Cable angle [rad]', 'FontSize', 16);
xlim([t_plot(1), t_plot(end)]);

% ★ Y軸の上下に少し余白を追加（下側: 5%, 上側: 15%）
y_data3  = [theta_x_rad, theta_y_rad];
y_min3   = min(y_data3);
y_max3   = max(y_data3);
y_range3 = y_max3 - y_min3;
ylim([y_min3 - y_range3 * 0.05, y_max3 + y_range3 * 0.15]);

legend('Angle\_x', 'Angle\_y', ...
       'Location', 'best', 'NumColumns', 2, 'FontSize', 15);
hold off;

end
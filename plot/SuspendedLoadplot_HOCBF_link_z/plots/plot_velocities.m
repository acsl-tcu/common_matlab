function plot_velocities(data)
% ドローン速度および荷物速度（推定値）の時系列プロット関数
% （2つの独立したウィンドウで出力）

% 1. 時間 t > 0 の要素のインデックスを見つける
t_pos_idx = (data.t > 0);

% 2. 時間 t > 0 のデータだけを切り出し
t_plot = data.t(t_pos_idx);

% ドローン速度データ (v)
v_est_x_plot = data.estimator.v(1, t_pos_idx);
v_est_y_plot = data.estimator.v(2, t_pos_idx);
v_est_z_plot = data.estimator.v(3, t_pos_idx);

% 荷物速度データ (vL)
vL_est_x_plot = data.estimator.vL(1, t_pos_idx);
vL_est_y_plot = data.estimator.vL(2, t_pos_idx);
vL_est_z_plot = data.estimator.vL(3, t_pos_idx);

%% =========================================================================
%% 3. ウィンドウ 1: ドローン速度 (Drone Velocity)
%% =========================================================================
figure('Name', 'Drone Velocity');

plot(t_plot, v_est_x_plot, 'Color', 'r', 'LineStyle', '-', 'LineWidth', 2); hold on; % Drone v_x: 赤・実線
plot(t_plot, v_est_y_plot, 'Color', 'b', 'LineStyle', '-', 'LineWidth', 2);          % Drone v_y: 青・実線
plot(t_plot, v_est_z_plot, 'Color', 'g', 'LineStyle', '-', 'LineWidth', 2);          % Drone v_z: 緑・実線

grid on;
set(gca, 'FontSize', 14);
xlabel('Time [s]', 'FontSize', 16);
ylabel('Drone Velocity [m/s]', 'FontSize', 16);
title('Drone Velocity', 'FontSize', 18);

xlim([t_plot(1), t_plot(end)]);

legend('Drone\_v_x', 'Drone\_v_y', 'Drone\_v_z', ...
       'Location', 'best', 'NumColumns', 2, 'FontSize', 15);
hold off;

%% =========================================================================
%% 4. ウィンドウ 2: 荷物速度 (Load Velocity)
%% =========================================================================
figure('Name', 'Load Velocity');

plot(t_plot, vL_est_x_plot, 'Color', 'r', 'LineStyle', '-', 'LineWidth', 2); hold on; % Load v_x: 赤・実線
plot(t_plot, vL_est_y_plot, 'Color', 'b', 'LineStyle', '-', 'LineWidth', 2);          % Load v_y: 青・実線
plot(t_plot, vL_est_z_plot, 'Color', 'g', 'LineStyle', '-', 'LineWidth', 2);          % Load v_z: 緑・実線

grid on;
set(gca, 'FontSize', 14);
xlabel('Time [s]', 'FontSize', 16);
ylabel('Load Velocity [m/s]', 'FontSize', 16);
title('Load Velocity', 'FontSize', 18);

xlim([t_plot(1), t_plot(end)]);

legend('Load\_v_x', 'Load\_v_y', 'Load\_v_z', ...
       'Location', 'best', 'NumColumns', 2, 'FontSize', 15);
hold off;

end
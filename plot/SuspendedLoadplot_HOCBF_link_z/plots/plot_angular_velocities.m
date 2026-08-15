function plot_angular_velocities(data)
% ケーブル角速度 (wL) および機体角速度 (w) の時系列プロット関数
% （deg/s 表記と rad/s 表記を合計4つの独立したウィンドウで出力）

% 1. 時間 t > 0 の要素のインデックスを見つける
t_pos_idx = (data.t > 0);

% 2. 時間 t > 0 のデータだけを切り出し (1行目: X, 2行目: Y, 3行目: Z)
t_plot   = data.t(t_pos_idx);

wL_est_rad = data.estimator.wL(:, t_pos_idx); % ケーブル角速度 [rad/s]
w_est_rad  = data.estimator.w(:, t_pos_idx);  % 機体角速度 [rad/s]

%% =========================================================================
%% 1. ケーブル角速度 wL [deg/s] (Cable Angular Velocity [deg/s])
%% =========================================================================
wL_est_deg = rad2deg(wL_est_rad);

figure('Name', 'Cable Angular Velocity [deg/s]');

plot(t_plot, wL_est_deg(1, :), 'Color', [0.85, 0.325, 0.098], 'LineStyle', '-', 'LineWidth', 1.8); hold on; % 橙: wL_x
plot(t_plot, wL_est_deg(2, :), 'Color', [0.00, 0.447, 0.741], 'LineStyle', '-', 'LineWidth', 1.8);          % 青: wL_y
plot(t_plot, wL_est_deg(3, :), 'Color', [0.466, 0.674, 0.188], 'LineStyle', '-', 'LineWidth', 1.8);          % 緑: wL_z

grid on;
set(gca, 'FontSize', 14);
xlabel('Time [s]', 'FontSize', 16);
ylabel('Cable angular velocity \omega_L [deg/s]', 'FontSize', 16);
title('Cable Angular Velocity (\omega_L) [deg/s]', 'FontSize', 18);

xlim([t_plot(1), t_plot(end)]);

legend('\omega_{Lx} (X-axis)', '\omega_{Ly} (Y-axis)', '\omega_{Lz} (Z-axis)', ...
       'Location', 'best', 'NumColumns', 3, 'FontSize', 14);
hold off;

%% =========================================================================
%% 2. ケーブル角速度 wL [rad/s] (Cable Angular Velocity [rad/s])
%% =========================================================================
figure('Name', 'Cable Angular Velocity [rad/s]');

plot(t_plot, wL_est_rad(1, :), 'Color', [0.85, 0.325, 0.098], 'LineStyle', '-', 'LineWidth', 1.8); hold on; % 橙: wL_x
plot(t_plot, wL_est_rad(2, :), 'Color', [0.00, 0.447, 0.741], 'LineStyle', '-', 'LineWidth', 1.8);          % 青: wL_y
plot(t_plot, wL_est_rad(3, :), 'Color', [0.466, 0.674, 0.188], 'LineStyle', '-', 'LineWidth', 1.8);          % 緑: wL_z

grid on;
set(gca, 'FontSize', 14);
xlabel('Time [s]', 'FontSize', 16);
ylabel('Cable angular velocity \omega_L [rad/s]', 'FontSize', 16);
title('Cable Angular Velocity (\omega_L) [rad/s]', 'FontSize', 18);

xlim([t_plot(1), t_plot(end)]);

legend('\omega_{Lx} (X-axis)', '\omega_{Ly} (Y-axis)', '\omega_{Lz} (Z-axis)', ...
       'Location', 'best', 'NumColumns', 3, 'FontSize', 14);
hold off;

%% =========================================================================
%% 3. 機体角速度 w [deg/s] (Drone Body Angular Velocity [deg/s])
%% =========================================================================
w_est_deg = rad2deg(w_est_rad);

figure('Name', 'Drone Body Angular Velocity [deg/s]');

plot(t_plot, w_est_deg(1, :), 'Color', [0.85, 0.325, 0.098], 'LineStyle', '-', 'LineWidth', 1.8); hold on; % 橙: Roll rate (p)
plot(t_plot, w_est_deg(2, :), 'Color', [0.00, 0.447, 0.741], 'LineStyle', '-', 'LineWidth', 1.8);          % 青: Pitch rate (q)
plot(t_plot, w_est_deg(3, :), 'Color', [0.466, 0.674, 0.188], 'LineStyle', '-', 'LineWidth', 1.8);          % 緑: Yaw rate (r)

grid on;
set(gca, 'FontSize', 14);
xlabel('Time [s]', 'FontSize', 16);
ylabel('Body angular velocity \omega [deg/s]', 'FontSize', 16);
title('Drone Body Angular Velocity (\omega) [deg/s]', 'FontSize', 18);

xlim([t_plot(1), t_plot(end)]);

legend('Roll rate (\omega_x)', 'Pitch rate (\omega_y)', 'Yaw rate (\omega_z)', ...
       'Location', 'best', 'NumColumns', 3, 'FontSize', 14);
hold off;

%% =========================================================================
%% 4. 機体角速度 w [rad/s] (Drone Body Angular Velocity [rad/s])
%% =========================================================================
figure('Name', 'Drone Body Angular Velocity [rad/s]');

plot(t_plot, w_est_rad(1, :), 'Color', [0.85, 0.325, 0.098], 'LineStyle', '-', 'LineWidth', 1.8); hold on; % 橙: Roll rate (p)
plot(t_plot, w_est_rad(2, :), 'Color', [0.00, 0.447, 0.741], 'LineStyle', '-', 'LineWidth', 1.8);          % 青: Pitch rate (q)
plot(t_plot, w_est_rad(3, :), 'Color', [0.466, 0.674, 0.188], 'LineStyle', '-', 'LineWidth', 1.8);          % 緑: Yaw rate (r)

grid on;
set(gca, 'FontSize', 14);
xlabel('Time [s]', 'FontSize', 16);
ylabel('Body angular velocity \omega [rad/s]', 'FontSize', 16);
title('Drone Body Angular Velocity (\omega) [rad/s]', 'FontSize', 18);

xlim([t_plot(1), t_plot(end)]);

legend('Roll rate (\omega_x)', 'Pitch rate (\omega_y)', 'Yaw rate (\omega_z)', ...
       'Location', 'best', 'NumColumns', 3, 'FontSize', 14);
hold off;

end
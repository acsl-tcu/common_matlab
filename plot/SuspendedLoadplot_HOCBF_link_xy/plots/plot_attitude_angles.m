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
%% 3. ウィンドウ 1: 姿勢角 [deg] (Attitude Angle [deg])
%% =========================================================================
q_est_x_deg = rad2deg(q_est_x_rad);
q_est_y_deg = rad2deg(q_est_y_rad);
q_est_z_deg = rad2deg(q_est_z_rad);

figure('Name', 'Attitude Angle [deg]');

plot(t_plot, q_est_x_deg, 'Color', [0.85, 0.325, 0.098], 'LineStyle', '-', 'LineWidth', 1.8); hold on; % 橙: Roll (φ)
plot(t_plot, q_est_y_deg, 'Color', [0.00, 0.447, 0.741], 'LineStyle', '-', 'LineWidth', 1.8);          % 青: Pitch (θ)
plot(t_plot, q_est_z_deg, 'Color', [0.466, 0.674, 0.188], 'LineStyle', '-', 'LineWidth', 1.8);          % 緑: Yaw (ψ)

grid on;
set(gca, 'FontSize', 14);
xlabel('Time [s]', 'FontSize', 16);
ylabel('Attitude angle [deg]', 'FontSize', 16);
title('Attitude Angle [deg]', 'FontSize', 18);

xlim([t_plot(1), t_plot(end)]);

legend('Roll (\phi)', 'Pitch (\theta)', 'Yaw (\psi)', ...
       'Location', 'best', 'NumColumns', 3, 'FontSize', 14);
hold off;

%% =========================================================================
%% 4. ウィンドウ 2: 姿勢角 [rad] (Attitude Angle [rad])
%% =========================================================================
figure('Name', 'Attitude Angle [rad]');

plot(t_plot, q_est_x_rad, 'Color', [0.85, 0.325, 0.098], 'LineStyle', '-', 'LineWidth', 1.8); hold on; % 橙: Roll (φ)
plot(t_plot, q_est_y_rad, 'Color', [0.00, 0.447, 0.741], 'LineStyle', '-', 'LineWidth', 1.8);          % 青: Pitch (θ)
plot(t_plot, q_est_z_rad, 'Color', [0.466, 0.674, 0.188], 'LineStyle', '-', 'LineWidth', 1.8);          % 緑: Yaw (ψ)

grid on;
set(gca, 'FontSize', 14);
xlabel('Time [s]', 'FontSize', 16);
ylabel('Attitude angle [rad]', 'FontSize', 16);
title('Attitude Angle [rad]', 'FontSize', 18);

xlim([t_plot(1), t_plot(end)]);

legend('Roll (\phi)', 'Pitch (\theta)', 'Yaw (\psi)', ...
       'Location', 'best', 'NumColumns', 3, 'FontSize', 14);
hold off;

end
function plot_drone_position(data)
% ドローン位置（推定値 p）の時系列プロット関数

% 1. 時間 t > 0 の要素のインデックスを見つける
t_pos_idx = (data.t > 0);

% 2. 時間 t > 0 のデータだけを切り出し
t_plot       = data.t(t_pos_idx);
p_est_x_plot = data.estimator.p(1, t_pos_idx);
p_est_y_plot = data.estimator.p(2, t_pos_idx);
p_est_z_plot = data.estimator.p(3, t_pos_idx);

% 3. グラフの描画開始
figure('Name', 'Drone Position (Estimator)');

% --- 推定位置を実線でプロット ---
plot(t_plot, p_est_x_plot, 'Color', 'r', 'LineStyle', '-', 'LineWidth', 2); hold on; % Drone X: 赤・実線
plot(t_plot, p_est_y_plot, 'Color', 'b', 'LineStyle', '-', 'LineWidth', 2);          % Drone Y: 青・実線
plot(t_plot, p_est_z_plot, 'Color', 'g', 'LineStyle', '-', 'LineWidth', 2);          % Drone Z: 緑・実線

% 4. グラフの装飾設定
grid on;
set(gca, 'FontSize', 14); % 軸目盛文字サイズ
xlabel('Time [s]', 'FontSize', 16);
ylabel('Drone Position [m]', 'FontSize', 16);
title('Drone Position', 'FontSize', 18);

% 軸を表示データの最初から最後まで（余白ゼロ）に固定
xlim([t_plot(1), t_plot(end)]);

% 凡例の設定
legend('Drone\_x', 'Drone\_y', 'Drone\_z', ...
       'Location', 'best', 'NumColumns', 2, 'FontSize', 15);
hold off;

end
% function plot_load_position_tracking(data)
% % 荷物位置（推定値）と目標軌道（p_ref）の時系列比較プロット関数
% 
% % 1. 102フェーズの中で、さらに時間が 0 秒より大きい要素のインデックスを見つける
% t_pos_idx = (data.t > 0);
% 
% % 2. 時間 t > 0 のデータだけを切り出し
% t_plot     = data.t(t_pos_idx);
% x_est_plot = data.estimator.pL(1, t_pos_idx);
% y_est_plot = data.estimator.pL(2, t_pos_idx);
% z_est_plot = data.estimator.pL(3, t_pos_idx);
% 
% x_ref_plot = data.ref.p(1, t_pos_idx);
% y_ref_plot = data.ref.p(2, t_pos_idx);
% z_ref_plot = data.ref.p(3, t_pos_idx);
% 
% % 3. グラフの描画開始
% figure('Name', 'Load Position vs Reference');
% 
% % --- 実測値（推定値）を「濃いめの実線」でプロット ---
% plot(t_plot, x_est_plot, 'Color', [0.85 0.33 0.10], 'LineStyle', '-', 'LineWidth', 2); hold on; % 荷物X: 橙・実線
% plot(t_plot, y_est_plot, 'Color', [0.47 0.67 0.19], 'LineStyle', '-', 'LineWidth', 2);          % 荷物Y: 黄緑・実線
% plot(t_plot, z_est_plot, 'Color', [0.00 0.45 0.74], 'LineStyle', '-', 'LineWidth', 2);          % 荷物Z: 青・実線
% 
% % --- 目標軌道 (p_ref) を「識別しやすい別の色の点線」でプロット ---
% plot(t_plot, x_ref_plot, 'Color', [0.49 0.18 0.56], 'LineStyle', '--', 'LineWidth', 1.5);       % 目標X: 紫・点線
% plot(t_plot, y_ref_plot, 'Color', [0.30 0.75 0.93], 'LineStyle', '--', 'LineWidth', 1.5);       % 目標Y: 水色・点線
% plot(t_plot, z_ref_plot, 'Color', [0.64 0.08 0.18], 'LineStyle', '--', 'LineWidth', 1.5);       % 目標Z: 暗赤・点線
% 
% % 4. グラフの装飾設定
% grid on;
% set(gca, 'FontSize', 14); % 軸目盛文字サイズ
% xlabel('Time [s]', 'FontSize', 16);
% ylabel('Load Position [m]', 'FontSize', 16);
% title('Load Position Tracking', 'FontSize', 18);
% 
% % 軸を表示データの最初から最後まで（余白ゼロ）に固定
% xlim([t_plot(1), t_plot(end)]);
% 
% % 凡例の設定（それぞれの線の色とスタイルに正確に対応）
% legend('load\_x (Est)', 'load\_y (Est)', 'load\_z (Est)', ...
%        'p\_ref\_x', 'p\_ref\_y', 'p\_ref\_z', ...
%        'Location', 'best', 'NumColumns', 2, 'FontSize', 15);
% hold off;
% 
% end

function plot_load_position_tracking(data)
% 荷物位置（推定値）と目標軌道（p_ref）の時系列比較プロット関数

% 1. 102フェーズの中で、さらに時間が 0 秒より大きい要素のインデックスを見つける
t_pos_idx = (data.t > 0);

% 2. 時間 t > 0 のデータだけを切り出し
t_plot     = data.t(t_pos_idx);
x_est_plot = data.estimator.pL(1, t_pos_idx);
y_est_plot = data.estimator.pL(2, t_pos_idx);
z_est_plot = data.estimator.pL(3, t_pos_idx);

x_ref_plot = data.ref.p(1, t_pos_idx);
y_ref_plot = data.ref.p(2, t_pos_idx);
z_ref_plot = data.ref.p(3, t_pos_idx);

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
%% 3. グラフの描画開始
%% =========================================================================
figure('Name', 'Load Position vs Reference', 'Position', fig_pos);

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
set(gca, 'FontSize', 14); % 軸目盛文字サイズ
xlabel('Time [s]', 'FontSize', 16);
ylabel('Payload Position [m]', 'FontSize', 16);

% 軸を表示データの最初から最後まで（余白ゼロ）に固定
xlim([t_plot(1), t_plot(end)]);
% データのY軸範囲（最小値・最大値）を取得
all_y_data = [x_est_plot, y_est_plot, z_est_plot, x_ref_plot, y_ref_plot, z_ref_plot];
y_min = min(all_y_data);
y_max = max(all_y_data);

% 上側に少し余裕を持たせる（例: 上側に最大値の20%ぶんの隙間を追加）
% 最小値側も少し余裕を持たせたい場合は下限も調整します
y_margin = (y_max - y_min) * 0.25; % 20%の余白量

ylim([y_min - (y_max - y_min)*0.05, y_max + y_margin]);

% % 凡例の設定（それぞれの線の色とスタイルに正確に対応）
% legend('load\_x (Est)', 'load\_y (Est)', 'load\_z (Est)', ...
%     'p\_ref\_x', 'p\_ref\_y', 'p\_ref\_z', ...
%     'Location', 'best', 'NumColumns', 3, 'FontSize', 15);
% 凡例の設定（それぞれの線の色とスタイルに正確に対応）
legend('Estimator_x', 'Estimator_y', 'Estimator_z', ...
    'Reference_x', 'Reference_y', 'Reference_z', ...
    'Location', 'best', 'NumColumns', 3, 'FontSize', 12);
hold off;

end
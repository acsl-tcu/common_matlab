% function plot_estimated_mass(data)
% % 吊り荷の推定質量 (mL_est) の時系列プロット関数
% 
% % 1. 時間 t > 0 の要素のインデックスを見つける
% t_pos_idx = (data.t > 0);
% 
% % 2. 時間 t > 0 のデータだけを切り出し
% t_plot      = data.t(t_pos_idx);
% mL_est_plot = data.estimator.mL(t_pos_idx);
% 
% % 3. グラフの描画開始
% figure('Name', 'Estimated Mass');
% 
% % --- 推定質量を青色の実線でプロット ---
% plot(t_plot, mL_est_plot, 'Color', 'b', 'LineStyle', '-', 'LineWidth', 2); hold on;
% 
% % 4. グラフの装飾設定
% grid on;
% set(gca, 'FontSize', 14); % 軸目盛文字サイズ
% xlabel('Time [s]', 'FontSize', 16);
% ylabel('Estimated mass [kg]', 'FontSize', 16);
% title('Estimated Mass', 'FontSize', 18);
% 
% % 軸を表示データの最初から最後まで（余白ゼロ）に固定
% xlim([t_plot(1), t_plot(end)]);
% 
% hold off;
% 
% end

function plot_estimated_mass(data)
% 吊り荷の推定質量 (mL_est) の時系列プロット関数

% 1. 時間 t > 0 の要素のインデックスを見つける
t_pos_idx = (data.t > 0);

% 2. 時間 t > 0 のデータだけを切り出し
t_plot      = data.t(t_pos_idx);
mL_est_plot = data.estimator.mL(t_pos_idx);

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
figure('Name', 'Estimated Mass', 'Position', fig_pos);

% --- 推定質量を青色の実線でプロット ---
plot(t_plot, mL_est_plot, 'Color', 'b', 'LineStyle', '-', 'LineWidth', 2); hold on;

% 4. グラフの装飾設定
grid on;
set(gca, 'FontSize', 14); % 軸目盛文字サイズ
xlabel('Time [s]', 'FontSize', 16);
ylabel('Estimated mass [kg]', 'FontSize', 16);

% 軸を表示データの最初から最後まで（余白ゼロ）に固定
xlim([t_plot(1), t_plot(end)]);

% ★ Y軸の上下に少し余白を追加（下側: 5%, 上側: 15%）
y_min   = min(mL_est_plot);
y_max   = max(mL_est_plot);
y_range = y_max - y_min;

% データが一定値（変化なし）の場合のゼロ割り・エラー防止用分岐
if y_range == 0
    ylim([y_min - 1, y_max + 1]);
else
    ylim([y_min - y_range * 0.05, y_max + y_range * 0.05]);
end

hold off;

end
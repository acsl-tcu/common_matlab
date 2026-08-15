% function plot_control_inputs(data)
% % 制御入力（CBF補正後 tmp_fix vs 公称入力 tmp）の時系列比較プロット関数
% 
% % 1. 102フェーズの中で、時間が 0 秒より大きい要素のインデックスを見つける
% t_pos_idx = (data.t > 0);
% 
% % 2. 時間 t > 0 のデータだけを切り出し
% t_plot = data.t(t_pos_idx);
% 
% % CBF補正後の入力 (tmp_fix)
% tmp_fix_thrust_plot = data.cbf.tmp_fix(1, t_pos_idx);
% tmp_fix_roll_plot   = data.cbf.tmp_fix(2, t_pos_idx);
% tmp_fix_pitch_plot  = data.cbf.tmp_fix(3, t_pos_idx);
% tmp_fix_yaw_plot    = data.cbf.tmp_fix(4, t_pos_idx);
% 
% % 公称入力 (tmp)
% tmp_thrust_plot = data.cbf.tmp(1, t_pos_idx);
% tmp_roll_plot   = data.cbf.tmp(2, t_pos_idx);
% tmp_pitch_plot  = data.cbf.tmp(3, t_pos_idx);
% tmp_yaw_plot    = data.cbf.tmp(4, t_pos_idx);
% 
% % 3. グラフの描画開始
% figure('Name', 'Control Inputs (CBF vs Nominal)');
% 
% % --- CBF補正後の入力（濃いめの実線）でプロット ---
% plot(t_plot, tmp_fix_thrust_plot, 'Color', [0.85 0.33 0.10], 'LineStyle', '-', 'LineWidth', 2); hold on; % Thrust: 橙・実線
% plot(t_plot, tmp_fix_roll_plot,   'Color', [0.47 0.67 0.19], 'LineStyle', '-', 'LineWidth', 2);          % Roll: 黄緑・実線
% plot(t_plot, tmp_fix_pitch_plot,  'Color', [0.00 0.45 0.74], 'LineStyle', '-', 'LineWidth', 2);          % Pitch: 青・実線
% plot(t_plot, tmp_fix_yaw_plot,    'Color', [0.85 0.00 0.85], 'LineStyle', '-', 'LineWidth', 2);          % Yaw: 紫・実線
% 
% % --- 公称入力 (tmp) を「点線」でプロット ---
% plot(t_plot, tmp_thrust_plot, 'Color', [0.49 0.18 0.56], 'LineStyle', '--', 'LineWidth', 1.5);       % 公称Thrust: 暗紫・点線
% plot(t_plot, tmp_roll_plot,   'Color', [0.30 0.75 0.93], 'LineStyle', '--', 'LineWidth', 1.5);       % 公称Roll: 水色・点線
% plot(t_plot, tmp_pitch_plot,  'Color', [0.64 0.08 0.18], 'LineStyle', '--', 'LineWidth', 1.5);       % 公称Pitch: 暗赤・点線
% plot(t_plot, tmp_yaw_plot,    'Color', [0.00 0.45 0.00], 'LineStyle', '--', 'LineWidth', 2);          % 公称Yaw: 緑・点線
% 
% % 4. グラフの装飾設定
% grid on;
% set(gca, 'FontSize', 14); % 軸目盛文字サイズ
% xlabel('Time [s]', 'FontSize', 16);
% ylabel('Input', 'FontSize', 16);
% title('Control Inputs (CBF Modified vs Nominal)', 'FontSize', 18);
% 
% % 軸を表示データの最初から最後まで（余白ゼロ）に固定
% xlim([t_plot(1), t_plot(end)]);
% 
% % 凡例の設定（それぞれの線の色とスタイルに正確に対応）
% legend('input_{CBF}\_thrust', 'input_{CBF}\_roll', 'input_{CBF}\_pitch', 'input_{CBF}\_yaw', ...
%        'input\_thrust', 'input\_roll', 'input\_pitch', 'input\_yaw', ...
%        'Location', 'best', 'NumColumns', 2, 'FontSize', 15);
% hold off;
% 
% end

function plot_control_inputs(data)
% 制御入力（CBF補正後 tmp_fix vs 公称入力 tmp）の時系列比較プロット関数

% 1. 102フェーズの中で、時間が 0 秒より大きい要素のインデックスを見つける
t_pos_idx = (data.t > 0);

% 2. 時間 t > 0 のデータだけを切り出し
t_plot = data.t(t_pos_idx);

% CBF補正後の入力 (tmp_fix)
tmp_fix_thrust_plot = data.cbf.tmp_fix(1, t_pos_idx);
tmp_fix_roll_plot   = data.cbf.tmp_fix(2, t_pos_idx);
tmp_fix_pitch_plot  = data.cbf.tmp_fix(3, t_pos_idx);
tmp_fix_yaw_plot    = data.cbf.tmp_fix(4, t_pos_idx);

% 公称入力 (tmp)
tmp_thrust_plot = data.cbf.tmp(1, t_pos_idx);
tmp_roll_plot   = data.cbf.tmp(2, t_pos_idx);
tmp_pitch_plot  = data.cbf.tmp(3, t_pos_idx);
tmp_yaw_plot    = data.cbf.tmp(4, t_pos_idx);

%% =========================================================================
%% 画面中央配置 & サイズ固定の位置計算（ピクセル単位）
%% =========================================================================
fig_width   = 600; % お好みの横幅
fig_height  = 400; % お好みの高さ
screen_size = get(0, 'ScreenSize');
pos_x = (screen_size(3) - fig_width) / 2;
pos_y = (screen_size(4) - fig_height) / 2;
fig_pos = [pos_x, pos_y, fig_width, fig_height];

%% =========================================================================
%% 3. グラフの描画開始
%% =========================================================================
figure('Name', 'Control Inputs (CBF vs Nominal)', 'Position', fig_pos);

% --- CBF補正後の入力（濃いめの実線）でプロット ---
plot(t_plot, tmp_fix_thrust_plot, 'Color', [0.85 0.33 0.10], 'LineStyle', '-', 'LineWidth', 2); hold on; % Thrust: 橙・実線
plot(t_plot, tmp_fix_roll_plot,   'Color', [0.47 0.67 0.19], 'LineStyle', '-', 'LineWidth', 2);          % Roll: 黄緑・実線
plot(t_plot, tmp_fix_pitch_plot,  'Color', [0.00 0.45 0.74], 'LineStyle', '-', 'LineWidth', 2);          % Pitch: 青・実線
plot(t_plot, tmp_fix_yaw_plot,    'Color', [0.85 0.00 0.85], 'LineStyle', '-', 'LineWidth', 2);          % Yaw: 紫・実線

% --- 公称入力 (tmp) を「点線」でプロット ---
plot(t_plot, tmp_thrust_plot, 'Color', [0.49 0.18 0.56], 'LineStyle', '--', 'LineWidth', 1.5);       % 公称Thrust: 暗紫・点線
plot(t_plot, tmp_roll_plot,   'Color', [0.30 0.75 0.93], 'LineStyle', '--', 'LineWidth', 1.5);       % 公称Roll: 水色・点線
plot(t_plot, tmp_pitch_plot,  'Color', [0.64 0.08 0.18], 'LineStyle', '--', 'LineWidth', 1.5);       % 公称Pitch: 暗赤・点線
plot(t_plot, tmp_yaw_plot,    'Color', [0.00 0.45 0.00], 'LineStyle', '--', 'LineWidth', 2);          % 公称Yaw: 緑・点線

% 4. グラフの装飾設定
grid on;
set(gca, 'FontSize', 14); % 軸目盛文字サイズ
xlabel('Time [s]', 'FontSize', 16);
ylabel('Control Input', 'FontSize', 16);

% 軸を表示データの最初から最後まで（余白ゼロ）に固定
xlim([t_plot(1), t_plot(end)]);

% データのY軸範囲（最小値・最大値）を取得して上部余白を設定
all_y_data = [tmp_fix_thrust_plot, tmp_fix_roll_plot, tmp_fix_pitch_plot, tmp_fix_yaw_plot, ...
    tmp_thrust_plot, tmp_roll_plot, tmp_pitch_plot, tmp_yaw_plot];
y_min = min(all_y_data);
y_max = max(all_y_data);
y_range = y_max - y_min;

% データが一定値（変化なし）の場合のエラー回避処理
if y_range == 0
    y_range = 1;
end

y_margin = y_range * 0.25; % 凡例スペース用の上部余白（25%）
ylim([y_min - y_range * 0.05, y_max + y_margin*0.05]);

% 凡例の設定（それぞれの線の色とスタイルに正確に対応）
legend('input_{CBF}\_thrust', 'input_{CBF}\_roll', 'input_{CBF}\_pitch', 'input_{CBF}\_yaw', ...
    'input\_thrust', 'input\_roll', 'input\_pitch', 'input\_yaw', ...
    'Location', 'best', 'NumColumns', 3, 'FontSize', 12);

hold off;
end
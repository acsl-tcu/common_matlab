% function plot_cbf_attitude_cable_layers_evaluation(data)
% % PLOT_CBF_ATTITUDE_CABLE_LAYERS_EVALUATION
% % 姿勢角および牽引紐振れ角の各階層バリア関数値（安全余裕度）と
% % QP制約（A*u <= b）の評価推移を3つの独立ウィンドウで可視化する関数
% 
% % 1. 時間 t > 0 の要素のインデックスを見つける
% t_pos_idx = (data.t > 0);
% 
% % 2. 時間 t > 0 のデータ切り出し
% t_plot = data.t(t_pos_idx);
% 
% % 各階層バリア関数値 (8 x N)
% % 1-2: Roll(h1, h2), 3-4: Pitch(h1, h2), 5-8: Cable(h1, h2, h3, h4)
% h_layers = data.cbf.h_layers_att_cb(:, t_pos_idx);
% 
% % QP制約関連データ
% A_xy_qp   = data.cbf.A_xy_qp(:, :, t_pos_idx); % 3 x 2 x N
% b_xy_qp   = data.cbf.b_xy_qp(:, t_pos_idx);    % 3 x N
% u_nom_xy  = data.cbf.tmp(2:3, t_pos_idx);      % 2 x N (公称 Roll, Pitch トルク)
% u_safe_xy = data.cbf.tmp_fix(2:3, t_pos_idx);  % 2 x N (CBF補正後 Roll, Pitch トルク)
% 
% % A*u の時系列計算 (3 x N)
% N_pts = length(t_plot);
% A_u_nom  = zeros(3, N_pts);
% A_u_safe = zeros(3, N_pts);
% for k = 1:N_pts
%     A_mat = A_xy_qp(:, :, k);
%     A_u_nom(:, k)  = A_mat * u_nom_xy(:, k);
%     A_u_safe(:, k) = A_mat * u_safe_xy(:, k);
% end
% 
% %% =========================================================================
% %% 画面配置 & サイズ固定の位置計算（ピクセル単位）
% %% =========================================================================
% fig_width   = 620; % 横幅
% fig_height  = 400; % 高さ
% screen_size = get(0, 'ScreenSize');
% 
% center_x = (screen_size(3) - fig_width) / 2;
% center_y = (screen_size(4) - fig_height) / 2;
% 
% % 3つのウィンドウをずらして配置
% fig_pos_att   = [center_x - 60, center_y + 40, fig_width, fig_height];
% fig_pos_cable = [center_x,      center_y,      fig_width, fig_height];
% fig_pos_qp    = [center_x + 60, center_y - 40, fig_width, fig_height];
% 
% %% =========================================================================
% %% 1. ウィンドウ 1: 機体姿勢角（Roll / Pitch）各階層バリア関数値 (h1, h2)
% %% =========================================================================
% figure('Name', 'Attitude CBF Layers (Roll & Pitch)', 'Position', fig_pos_att);
% 
% % 基準線 0（安全境界: >= 0 で安全）
% yline(0, 'k--', 'LineWidth', 1.2, 'DisplayName', 'Safety Boundary (0)'); hold on;
% 
% % Roll 各階層
% plot(t_plot, h_layers(1, :), 'Color', [0.85, 0.33, 0.10], 'LineStyle', '-',  'LineWidth', 2.0, 'DisplayName', 'Roll h_1 (Angle Limit)');
% plot(t_plot, h_layers(2, :), 'Color', [0.85, 0.33, 0.10], 'LineStyle', '--', 'LineWidth', 1.6, 'DisplayName', 'Roll h_2 (Angular Vel Limit)');
% 
% % Pitch 各階層
% plot(t_plot, h_layers(3, :), 'Color', [0.00, 0.45, 0.74], 'LineStyle', '-',  'LineWidth', 2.0, 'DisplayName', 'Pitch h_1 (Angle Limit)');
% plot(t_plot, h_layers(4, :), 'Color', [0.00, 0.45, 0.74], 'LineStyle', '--', 'LineWidth', 1.6, 'DisplayName', 'Pitch h_2 (Angular Vel Limit)');
% 
% grid on;
% set(gca, 'FontSize', 14);
% xlabel('Time [s]', 'FontSize', 16);
% ylabel('Barrier Value h', 'FontSize', 16);
% title('Attitude CBF Layers History (Roll & Pitch)', 'FontSize', 18);
% xlim([t_plot(1), t_plot(end)]);
% 
% % Y軸余白調整
% y_data_att = [h_layers(1:4, :), 0];
% y_min = min(y_data_att(:)); y_max = max(y_data_att(:));
% y_range = y_max - y_min; if y_range == 0, y_range = 1; end
% ylim([y_min - y_range * 0.05, y_max + y_range * 0.25]);
% 
% legend('Location', 'best', 'NumColumns', 2, 'FontSize', 11);
% hold off;
% 
% %% =========================================================================
% %% 2. ウィンドウ 2: 牽引紐振れ角 各階層バリア関数値 (h1 〜 h4)
% %% =========================================================================
% figure('Name', 'Cable Swing Angle CBF Layers (h1 to h4)', 'Position', fig_pos_cable);
% 
% yline(0, 'k--', 'LineWidth', 1.2, 'DisplayName', 'Safety Boundary (0)'); hold on;
% 
% % Cable 各階層
% plot(t_plot, h_layers(5, :), 'Color', [0.47, 0.67, 0.19], 'LineStyle', '-',  'LineWidth', 2.0, 'DisplayName', 'Cable h_1 (Swing Angle)');
% plot(t_plot, h_layers(6, :), 'Color', [0.30, 0.75, 0.93], 'LineStyle', '--', 'LineWidth', 1.6, 'DisplayName', 'Cable h_2 (Swing Rate)');
% plot(t_plot, h_layers(7, :), 'Color', [0.49, 0.18, 0.56], 'LineStyle', ':',  'LineWidth', 1.8, 'DisplayName', 'Cable h_3 (Acceleration)');
% plot(t_plot, h_layers(8, :), 'Color', [0.64, 0.08, 0.18], 'LineStyle', '-.', 'LineWidth', 1.6, 'DisplayName', 'Cable h_4 (Jerk)');
% 
% grid on;
% set(gca, 'FontSize', 14);
% xlabel('Time [s]', 'FontSize', 16);
% ylabel('Barrier Value h', 'FontSize', 16);
% title('Cable Swing CBF Layers History (Relative Degree 4)', 'FontSize', 18);
% xlim([t_plot(1), t_plot(end)]);
% 
% % Y軸余白調整
% y_data_cb = [h_layers(5:8, :), 0];
% y_min = min(y_data_cb(:)); y_max = max(y_data_cb(:));
% y_range = y_max - y_min; if y_range == 0, y_range = 1; end
% ylim([y_min - y_range * 0.05, y_max + y_range * 0.25]);
% 
% legend('Location', 'best', 'NumColumns', 2, 'FontSize', 11);
% hold off;
% 
% %% =========================================================================
% %% 3. ウィンドウ 3: QP制約条件の評価推移 (A_qp*u <= b_qp)
% %% =========================================================================
% figure('Name', 'Attitude & Cable QP Constraints Fulfillment', 'Position', fig_pos_qp);
% 
% % 1: Roll, 2: Pitch, 3: Cable の最悪制約を代表プロット（または3要素を色分け）
% % --- 制約上限値 b_qp (橙実線) ---
% plot(t_plot, b_xy_qp(1, :), 'Color', [0.85, 0.33, 0.10], 'LineStyle', '-', 'LineWidth', 2.0, 'DisplayName', 'b_{qp} (Roll Bound)'); hold on;
% plot(t_plot, b_xy_qp(2, :), 'Color', [0.00, 0.45, 0.74], 'LineStyle', '-', 'LineWidth', 2.0, 'DisplayName', 'b_{qp} (Pitch Bound)');
% plot(t_plot, b_xy_qp(3, :), 'Color', [0.47, 0.67, 0.19], 'LineStyle', '-', 'LineWidth', 2.0, 'DisplayName', 'b_{qp} (Cable Bound)');
% 
% % --- 公称入力時の制約値 A*u_nom (紫破線: bを超えると制約違反) ---
% plot(t_plot, A_u_nom(1, :), 'Color', [0.85, 0.33, 0.10], 'LineStyle', '--', 'LineWidth', 1.5, 'DisplayName', 'A_{qp}u_{nom} (Roll)');
% plot(t_plot, A_u_nom(2, :), 'Color', [0.00, 0.45, 0.74], 'LineStyle', '--', 'LineWidth', 1.5, 'DisplayName', 'A_{qp}u_{nom} (Pitch)');
% plot(t_plot, A_u_nom(3, :), 'Color', [0.47, 0.67, 0.19], 'LineStyle', '--', 'LineWidth', 1.5, 'DisplayName', 'A_{qp}u_{nom} (Cable)');
% 
% % --- CBF補正後の制約値 A*u_safe (青実線/点線: b以下に収まる) ---
% plot(t_plot, A_u_safe(1, :), 'Color', [0.49, 0.18, 0.56], 'LineStyle', ':', 'LineWidth', 2.0, 'DisplayName', 'A_{qp}u^* (Roll Safe)');
% plot(t_plot, A_u_safe(2, :), 'Color', [0.30, 0.75, 0.93], 'LineStyle', ':', 'LineWidth', 2.0, 'DisplayName', 'A_{qp}u^* (Pitch Safe)');
% plot(t_plot, A_u_safe(3, :), 'Color', [0.64, 0.08, 0.18], 'LineStyle', ':', 'LineWidth', 2.0, 'DisplayName', 'A_{qp}u^* (Cable Safe)');
% 
% grid on;
% set(gca, 'FontSize', 14);
% xlabel('Time [s]', 'FontSize', 16);
% ylabel('Constraint Value', 'FontSize', 16);
% title('QP Constraint Fulfillment (A_{qp}u \le b_{qp})', 'FontSize', 18);
% xlim([t_plot(1), t_plot(end)]);
% 
% % Y軸余白調整
% y_data_qp = [b_xy_qp(:); A_u_nom(:); A_u_safe(:)];
% y_min = min(y_data_qp); y_max = max(y_data_qp);
% y_range = y_max - y_min; if y_range == 0, y_range = 1; end
% ylim([y_min - y_range * 0.05, y_max + y_range * 0.25]);
% 
% legend('Location', 'best', 'NumColumns', 3, 'FontSize', 10);
% hold off;
% 
% end

function plot_cbf_attitude_cable_layers_evaluation(data)
% PLOT_CBF_ATTITUDE_CABLE_LAYERS_EVALUATION
% 姿勢角および牽引紐振れ角の各階層バリア関数値（安全余裕度）と
% QP制約（A*u <= b）の評価推移を3つの独立ウィンドウで可視化する関数

% 1. 時間 t > 0 の要素のインデックスを見つける
t_pos_idx = (data.t > 0);

% 2. 時間 t > 0 のデータ切り出し
t_plot = data.t(t_pos_idx);

% 各階層バリア関数値 (8 x N)
% 1-2: Roll(h1, h2), 3-4: Pitch(h1, h2), 5-8: Cable(h1, h2, h3, h4)
h_layers = data.cbf.h_layers_att_cb(:, t_pos_idx);

% QP制約関連データ
A_xy_qp   = data.cbf.A_xy_qp(:, :, t_pos_idx); % 3 x 2 x N
b_xy_qp   = data.cbf.b_xy_qp(:, t_pos_idx);    % 3 x N
u_nom_xy  = data.cbf.tmp(2:3, t_pos_idx);      % 2 x N (公称 Roll, Pitch トルク)
u_safe_xy = data.cbf.tmp_fix(2:3, t_pos_idx);  % 2 x N (CBF補正後 Roll, Pitch トルク)

% A*u の時系列計算 (3 x N)
N_pts = length(t_plot);
A_u_nom  = zeros(3, N_pts);
A_u_safe = zeros(3, N_pts);
for k = 1:N_pts
    A_mat = A_xy_qp(:, :, k);
    A_u_nom(:, k)  = A_mat * u_nom_xy(:, k);
    A_u_safe(:, k) = A_mat * u_safe_xy(:, k);
end

%% =========================================================================
%% 画面配置 & サイズ固定の位置計算（ピクセル単位）
%% =========================================================================
fig_width   = 620; % 横幅
fig_height  = 400; % 高さ
screen_size = get(0, 'ScreenSize');

center_x = (screen_size(3) - fig_width) / 2;
center_y = (screen_size(4) - fig_height) / 2;

% 3つのウィンドウをずらして配置
fig_pos_att   = [center_x - 60, center_y + 40, fig_width, fig_height];
fig_pos_cable = [center_x,      center_y,      fig_width, fig_height];
fig_pos_qp    = [center_x + 60, center_y - 40, fig_width, fig_height];

%% =========================================================================
%% 1. ウィンドウ 1: 機体姿勢角（Roll / Pitch）各階層バリア関数値 (h1, h2)
%% =========================================================================
figure('Name', 'Attitude CBF Layers (Roll & Pitch)', 'Position', fig_pos_att);

% 基準線 0（安全境界: >= 0 で安全）
yline(0, 'k--', 'LineWidth', 1.2, 'DisplayName', 'Safety Boundary (0)'); hold on;

% Roll 各階層
plot(t_plot, h_layers(1, :), 'Color', [0.85, 0.33, 0.10], 'LineStyle', '-',  'LineWidth', 2.0, 'DisplayName', 'Roll h_1 (Angle Limit)');
plot(t_plot, h_layers(2, :), 'Color', [0.85, 0.33, 0.10], 'LineStyle', '--', 'LineWidth', 1.6, 'DisplayName', 'Roll h_2 (Angular Vel Limit)');

% Pitch 各階層
plot(t_plot, h_layers(3, :), 'Color', [0.00, 0.45, 0.74], 'LineStyle', '-',  'LineWidth', 2.0, 'DisplayName', 'Pitch h_1 (Angle Limit)');
plot(t_plot, h_layers(4, :), 'Color', [0.00, 0.45, 0.74], 'LineStyle', '--', 'LineWidth', 1.6, 'DisplayName', 'Pitch h_2 (Angular Vel Limit)');

grid on;
set(gca, 'FontSize', 14);
xlabel('Time [s]', 'FontSize', 16);
ylabel('Barrier Value h', 'FontSize', 16);
title('Attitude CBF Layers History (Roll & Pitch)', 'FontSize', 18);
xlim([t_plot(1), t_plot(end)]);

% Y軸余白調整（修正箇所：全要素を1次元化して0と連結）
y_data_att = [reshape(h_layers(1:4, :), [], 1); 0];
y_min = min(y_data_att); y_max = max(y_data_att);
y_range = y_max - y_min; if y_range == 0, y_range = 1; end
ylim([y_min - y_range * 0.05, y_max + y_range * 0.25]);

legend('Location', 'best', 'NumColumns', 2, 'FontSize', 11);
hold off;

%% =========================================================================
%% 2. ウィンドウ 2: 牽引紐振れ角 各階層バリア関数値 (h1 〜 h4)
%% =========================================================================
figure('Name', 'Cable Swing Angle CBF Layers (h1 to h4)', 'Position', fig_pos_cable);

yline(0, 'k--', 'LineWidth', 1.2, 'DisplayName', 'Safety Boundary (0)'); hold on;

% Cable 各階層
plot(t_plot, h_layers(5, :), 'Color', [0.47, 0.67, 0.19], 'LineStyle', '-',  'LineWidth', 2.0, 'DisplayName', 'Cable h_1 (Swing Angle)');
plot(t_plot, h_layers(6, :), 'Color', [0.30, 0.75, 0.93], 'LineStyle', '--', 'LineWidth', 1.6, 'DisplayName', 'Cable h_2 (Swing Rate)');
plot(t_plot, h_layers(7, :), 'Color', [0.49, 0.18, 0.56], 'LineStyle', ':',  'LineWidth', 1.8, 'DisplayName', 'Cable h_3 (Acceleration)');
plot(t_plot, h_layers(8, :), 'Color', [0.64, 0.08, 0.18], 'LineStyle', '-.', 'LineWidth', 1.6, 'DisplayName', 'Cable h_4 (Jerk)');

grid on;
set(gca, 'FontSize', 14);
xlabel('Time [s]', 'FontSize', 16);
ylabel('Barrier Value h', 'FontSize', 16);
title('Cable Swing CBF Layers History (Relative Degree 4)', 'FontSize', 18);
xlim([t_plot(1), t_plot(end)]);

% Y軸余白調整（修正箇所：全要素を1次元化して0と連結）
y_data_cb = [reshape(h_layers(5:8, :), [], 1); 0];
y_min = min(y_data_cb); y_max = max(y_data_cb);
y_range = y_max - y_min; if y_range == 0, y_range = 1; end
ylim([y_min - y_range * 0.05, y_max + y_range * 0.25]);

legend('Location', 'best', 'NumColumns', 2, 'FontSize', 11);
hold off;

%% =========================================================================
%% 3. ウィンドウ 3: QP制約条件の評価推移 (A_qp*u <= b_qp)
%% =========================================================================
figure('Name', 'Attitude & Cable QP Constraints Fulfillment', 'Position', fig_pos_qp);

% --- 制約上限値 b_qp (実線) ---
plot(t_plot, b_xy_qp(1, :), 'Color', [0.85, 0.33, 0.10], 'LineStyle', '-', 'LineWidth', 2.0, 'DisplayName', 'b_{qp} (Roll Bound)'); hold on;
plot(t_plot, b_xy_qp(2, :), 'Color', [0.00, 0.45, 0.74], 'LineStyle', '-', 'LineWidth', 2.0, 'DisplayName', 'b_{qp} (Pitch Bound)');
plot(t_plot, b_xy_qp(3, :), 'Color', [0.47, 0.67, 0.19], 'LineStyle', '-', 'LineWidth', 2.0, 'DisplayName', 'b_{qp} (Cable Bound)');

% --- 公称入力時の制約値 A*u_nom (破線) ---
plot(t_plot, A_u_nom(1, :), 'Color', [0.85, 0.33, 0.10], 'LineStyle', '--', 'LineWidth', 1.5, 'DisplayName', 'A_{qp}u_{nom} (Roll)');
plot(t_plot, A_u_nom(2, :), 'Color', [0.00, 0.45, 0.74], 'LineStyle', '--', 'LineWidth', 1.5, 'DisplayName', 'A_{qp}u_{nom} (Pitch)');
plot(t_plot, A_u_nom(3, :), 'Color', [0.47, 0.67, 0.19], 'LineStyle', '--', 'LineWidth', 1.5, 'DisplayName', 'A_{qp}u_{nom} (Cable)');

% --- CBF補正後の制約値 A*u_safe (点線) ---
plot(t_plot, A_u_safe(1, :), 'Color', [0.49, 0.18, 0.56], 'LineStyle', ':', 'LineWidth', 2.0, 'DisplayName', 'A_{qp}u^* (Roll Safe)');
plot(t_plot, A_u_safe(2, :), 'Color', [0.30, 0.75, 0.93], 'LineStyle', ':', 'LineWidth', 2.0, 'DisplayName', 'A_{qp}u^* (Pitch Safe)');
plot(t_plot, A_u_safe(3, :), 'Color', [0.64, 0.08, 0.18], 'LineStyle', ':', 'LineWidth', 2.0, 'DisplayName', 'A_{qp}u^* (Cable Safe)');

grid on;
set(gca, 'FontSize', 14);
xlabel('Time [s]', 'FontSize', 16);
ylabel('Constraint Value', 'FontSize', 16);
title('QP Constraint Fulfillment (A_{qp}u \le b_{qp})', 'FontSize', 18);
xlim([t_plot(1), t_plot(end)]);

% Y軸余白調整
y_data_qp = [b_xy_qp(:); A_u_nom(:); A_u_safe(:)];
y_min = min(y_data_qp); y_max = max(y_data_qp);
y_range = y_max - y_min; if y_range == 0, y_range = 1; end
ylim([y_min - y_range * 0.05, y_max + y_range * 0.25]);

legend('Location', 'best', 'NumColumns', 3, 'FontSize', 10);
hold off;

end
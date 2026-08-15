% function plot_computation_time_all(data)
% % コントローラ単体および全体の計算時間の統計値表示・個別のウィンドウ描画関数
% 
% % 1. 時間 t > 0 の有効データのみ抽出
% valid_idx = (data.t > 0);
% t_valid   = data.t(valid_idx);
% 
% c_time       = data.cbf.controllertime(valid_idx);       % コントローラ単体
% c_time_total = data.cbf.controllertime_total(valid_idx); % 全体制御周期
% 
% % 2. 統計値（最大・最小・平均）の計算
% max_single  = max(c_time);
% min_single  = min(c_time);
% mean_single = mean(c_time);
% 
% max_total  = max(c_time_total);
% min_total  = min(c_time_total);
% mean_total = mean(c_time_total);
% 
% % 3. コマンドウィンドウへの統計値出力
% fprintf('\n=======================================================\n');
% fprintf('⏱️ 【計算時間統計（コントローラ単体 vs 全体制御周期）】\n');
% fprintf('-------------------------------------------------------\n');
% fprintf(' [1] コントローラ単体の時間 (controllertime)\n');
% fprintf('  ├─ 最大値 (Max)  : %.6f [s]\n', max_single);
% fprintf('  ├─ 最小値 (Min)  : %.6f [s]\n', min_single);
% fprintf('  └─ 平均値 (Mean) : %.6f [s]\n', mean_single);
% fprintf('-------------------------------------------------------\n');
% fprintf(' [2] 全体の制御周期 (controllertime_total)\n');
% fprintf('  ├─ 最大値 (Max)  : %.6f [s]\n', max_total);
% fprintf('  ├─ 最小値 (Min)  : %.6f [s]\n', min_total);
% fprintf('  └─ 平均値 (Mean) : %.6f [s]\n', mean_total);
% fprintf('=======================================================\n\n');
% 
% %% =========================================================================
% %% 4. ウィンドウ 1: コントローラ単体の計算時間
% %% =========================================================================
% figure('Name', 'Controller Single Computation Time');
% plot(t_valid, c_time, 'b-', 'LineWidth', 1.5);
% 
% grid on;
% set(gca, 'FontSize', 14);
% xlabel('Time [s]', 'FontSize', 16);
% ylabel('Computation Time [s]', 'FontSize', 16);
% xlim([t_valid(1), t_valid(end)]);
% ylim([0, max_single * 1.3]);
% 
% %% =========================================================================
% %% 5. ウィンドウ 2: 全体の制御周期
% %% =========================================================================
% figure('Name', 'Total Computation Time');
% plot(t_valid, c_time_total, 'r-', 'LineWidth', 1.5);
% 
% grid on;
% set(gca, 'FontSize', 14);
% xlabel('Time [s]', 'FontSize', 16);
% ylabel('Computation Time [s]', 'FontSize', 16);
% xlim([t_valid(1), t_valid(end)]);
% ylim([0, max_total * 1.3]);
% 
% %% =========================================================================
% %% 6. ウィンドウ 3: 重ね合わせ比較
% %% =========================================================================
% figure('Name', 'Computation Time Comparison');
% plot(t_valid, c_time, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Controller Single'); hold on;
% plot(t_valid, c_time_total, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Total Cycle');
% 
% grid on;
% set(gca, 'FontSize', 14);
% xlabel('Time [s]', 'FontSize', 16);
% ylabel('Computation Time [s]', 'FontSize', 16);
% 
% legend('Controller Single', 'Total Cycle', 'Location', 'best', 'FontSize', 14);
% xlim([t_valid(1), t_valid(end)]);
% ylim([0, max(max_single, max_total) * 1.3]);
% hold off;
% 
% end

function plot_computation_time_all(data)
% コントローラ単体および全体の計算時間の統計値表示・個別のウィンドウ描画関数

% 1. 時間 t > 0 の有効データのみ抽出
valid_idx = (data.t > 0);
t_valid   = data.t(valid_idx);

c_time       = data.cbf.controllertime(valid_idx);       % コントローラ単体
c_time_total = data.cbf.controllertime_total(valid_idx); % 全体制御周期

% 2. 統計値（最大・最小・平均）の計算
max_single  = max(c_time);
min_single  = min(c_time);
mean_single = mean(c_time);

max_total  = max(c_time_total);
min_total  = min(c_time_total);
mean_total = mean(c_time_total);

% 3. コマンドウィンドウへの統計値出力
fprintf('\n=======================================================\n');
fprintf('⏱️ 【計算時間統計（コントローラ単体 vs 全体制御周期）】\n');
fprintf('-------------------------------------------------------\n');
fprintf(' [1] コントローラ単体の時間 (controllertime)\n');
fprintf('  ├─ 最大値 (Max)  : %.6f [s]\n', max_single);
fprintf('  ├─ 最小値 (Min)  : %.6f [s]\n', min_single);
fprintf('  └─ 平均値 (Mean) : %.6f [s]\n', mean_single);
fprintf('-------------------------------------------------------\n');
fprintf(' [2] 全体の制御周期 (controllertime_total)\n');
fprintf('  ├─ 最大値 (Max)  : %.6f [s]\n', max_total);
fprintf('  ├─ 最小値 (Min)  : %.6f [s]\n', min_total);
fprintf('  └─ 平均値 (Mean) : %.6f [s]\n', mean_total);
fprintf('=======================================================\n\n');

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
%% 4. ウィンドウ 1: コントローラ単体の計算時間
%% =========================================================================
figure('Name', 'Controller Single Computation Time', 'Position', fig_pos);
plot(t_valid, c_time, 'b-', 'LineWidth', 1.5);

grid on;
set(gca, 'FontSize', 14);
xlabel('Time [s]', 'FontSize', 16);
ylabel('Computation Time [s]', 'FontSize', 16);
xlim([t_valid(1), t_valid(end)]);
ylim([0, max_single * 1.3]);

%% =========================================================================
%% 5. ウィンドウ 2: 全体の制御周期
%% =========================================================================
figure('Name', 'Total Computation Time', 'Position', fig_pos);
plot(t_valid, c_time_total, 'r-', 'LineWidth', 1.5);

grid on;
set(gca, 'FontSize', 14);
xlabel('Time [s]', 'FontSize', 16);
ylabel('Computation Time [s]', 'FontSize', 16);
xlim([t_valid(1), t_valid(end)]);
ylim([0, max_total * 1.3]);

%% =========================================================================
%% 6. ウィンドウ 3: 重ね合わせ比較
%% =========================================================================
figure('Name', 'Computation Time Comparison', 'Position', fig_pos);
plot(t_valid, c_time, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Controller Single'); hold on;
plot(t_valid, c_time_total, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Total Cycle');

grid on;
set(gca, 'FontSize', 14);
xlabel('Time [s]', 'FontSize', 16);
ylabel('Computation Time [s]', 'FontSize', 16);

legend('Controller Single', 'Total Cycle', 'Location', 'best', 'FontSize', 14);
xlim([t_valid(1), t_valid(end)]);
ylim([0, max(max_single, max_total) * 1.3]);
hold off;

end
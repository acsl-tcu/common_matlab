% function plot_obstacle_clearance(data)
% % 障害物との距離評価関数（マージンあり・なしを一括描画＆判定する統合版）
% 
% % 1. 時間 t > 0 のデータを切り出し
% t_pos_idx = (data.t > 0);
% t_plot    = data.t(t_pos_idx);
% 
% % マージンありの距離（CBF判定用）
% r_minimal_margin = data.cbf.r_minimal(t_pos_idx);
% 
% % マージンなしの距離（物理的接触距離）
% if isfield(data.cbf, 'r_minimal_no_margin')
%     r_minimal_raw = data.cbf.r_minimal_no_margin(t_pos_idx);
% else
%     r_minimal_raw = data.cbf.r_minimal(t_pos_idx) + data.cbf.d_margin(t_pos_idx);
% end
% 
% % 2. 最小距離（最接近）の算出とコンソール判定表示
% min_dist_margin = min(r_minimal_margin);
% min_dist_raw    = min(r_minimal_raw);
% 
% fprintf('\n=======================================================\n');
% fprintf('🔥 【障害物最接近データ＆回避結果（t > 0）】\n');
% fprintf('-------------------------------------------------------\n');
% fprintf(' [1] マージンあり距離（CBF判定境界）\n');
% fprintf('  ├─ 最接近距離 : %.4f [m]\n', min_dist_margin);
% if min_dist_margin >= 0
%     fprintf('  └─ 判定       : 🔴【安全確認】マージン境界を超えずに回避完了。\n');
% else
%     fprintf('  └─ 判定       : ⚠️【警告】マージン境界を割り込みました。\n');
% end
% fprintf('-------------------------------------------------------\n');
% fprintf(' [2] マージンなし距離（障害物本体との物理距離）\n');
% fprintf('  ├─ 最接近距離 : %.4f [m]\n', min_dist_raw);
% if min_dist_raw > 0
%     fprintf('  └─ 判定       : 🟢【物理安全】本体との物理衝突は回避されました。\n');
% else
%     fprintf('  └─ 判定       : ❌【物理衝突】本体と接触しました！\n');
% end
% fprintf('=======================================================\n\n');
% 
% %% =========================================================================
% %% 3. ウィンドウ 1: マージンあり（CBF判定境界）の時系列グラフ
% %% =========================================================================
% figure('Name', 'Distance to Obstacle Boundary (Margin Included)');
% plot(t_plot, r_minimal_margin, 'r-', 'LineWidth', 2);
% 
% grid on;
% set(gca, 'FontSize', 14);
% xlabel('Time [s]', 'FontSize', 16);
% ylabel('Distance [m]', 'FontSize', 16);
% title('Distance to Obstacle Boundary (Margin Included)', 'FontSize', 18);
% 
% xlim([t_plot(1), t_plot(end)]);
% ylim([0, max(r_minimal_margin) * 1.1]);
% legend('Distance (Margin)', 'Location', 'best', 'FontSize', 15);
% 
% %% =========================================================================
% %% 4. ウィンドウ 2: マージンなし（障害物本体との物理距離）の時系列グラフ
% %% =========================================================================
% figure('Name', 'Physical Distance to Obstacle Body (No Margin)');
% plot(t_plot, r_minimal_raw, 'b-', 'LineWidth', 2);
% 
% grid on;
% set(gca, 'FontSize', 14);
% xlabel('Time [s]', 'FontSize', 16);
% ylabel('Distance [m]', 'FontSize', 16);
% title('Physical Distance to Obstacle Body (No Margin)', 'FontSize', 18);
% 
% xlim([t_plot(1), t_plot(end)]);
% ylim([0, max(r_minimal_raw) * 1.1]);
% legend('Physical Distance (No Margin)', 'Location', 'best', 'FontSize', 15);
% 
% end

function plot_obstacle_clearance(data)
% 障害物との距離評価関数（マージンあり・なしを一括描画＆判定する統合版）

% 1. 時間 t > 0 のデータを切り出し
t_pos_idx = (data.t > 0);
t_plot    = data.t(t_pos_idx);

% マージンありの距離（CBF判定用）
r_minimal_margin = data.cbf.r_minimal(t_pos_idx);

% マージンなしの距離（物理的接触距離）
if isfield(data.cbf, 'r_minimal_no_margin')
    r_minimal_raw = data.cbf.r_minimal_no_margin(t_pos_idx);
else
    r_minimal_raw = data.cbf.r_minimal(t_pos_idx) + data.cbf.d_margin(t_pos_idx);
end

% 2. 最小距離（最接近）の算出とコンソール判定表示
min_dist_margin = min(r_minimal_margin);
min_dist_raw    = min(r_minimal_raw);

fprintf('\n=======================================================\n');
fprintf('🔥 【障害物最接近データ＆回避結果（t > 0）】\n');
fprintf('-------------------------------------------------------\n');
fprintf(' [1] マージンあり距離（CBF判定境界）\n');
fprintf('  ├─ 最接近距離 : %.4f [m]\n', min_dist_margin);
if min_dist_margin >= 0
    fprintf('  └─ 判定       : 🔴【安全確認】マージン境界を超えずに回避完了。\n');
else
    fprintf('  └─ 判定       : ⚠️【警告】マージン境界を割り込みました。\n');
end
fprintf('-------------------------------------------------------\n');
fprintf(' [2] マージンなし距離（障害物本体との物理距離）\n');
fprintf('  ├─ 最接近距離 : %.4f [m]\n', min_dist_raw);
if min_dist_raw > 0
    fprintf('  └─ 判定       : 🟢【物理安全】本体との物理衝突は回避されました。\n');
else
    fprintf('  └─ 判定       : ❌【物理衝突】本体と接触しました！\n');
end
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
%% 3. ウィンドウ 1: マージンあり（CBF判定境界）の時系列グラフ
%% =========================================================================
figure('Name', 'Distance to Obstacle Boundary (Margin Included)', 'Position', fig_pos);
plot(t_plot, r_minimal_margin, 'r-', 'LineWidth', 2);
grid on;
set(gca, 'FontSize', 14);
xlabel('Time [s]', 'FontSize', 16);
ylabel('Distance [m]', 'FontSize', 16);
xlim([t_plot(1), t_plot(end)]);

% ★ Y軸の上下に少し余白を追加（下側: 5%, 上側: 15%）
y_min1 = min(r_minimal_margin);
y_max1 = max(r_minimal_margin);
y_range1 = y_max1 - y_min1;
ylim([y_min1 - y_range1 * 0.05, y_max1 + y_range1 * 0.05]);

% legend('Distance (Margin)', 'Location', 'best', 'FontSize', 15);

%% =========================================================================
%% 4. ウィンドウ 2: マージンなし（障害物本体との物理距離）の時系列グラフ
%% =========================================================================
figure('Name', 'Physical Distance to Obstacle Body (No Margin)', 'Position', fig_pos);
plot(t_plot, r_minimal_raw, 'b-', 'LineWidth', 2);
grid on;
set(gca, 'FontSize', 14);
xlabel('Time [s]', 'FontSize', 16);
ylabel('Distance [m]', 'FontSize', 16);
xlim([t_plot(1), t_plot(end)]);

% ★ Y軸の上下に少し余白を追加（下側: 5%, 上側: 15%）
y_min2 = min(r_minimal_raw);
y_max2 = max(r_minimal_raw);
y_range2 = y_max2 - y_min2;
ylim([y_min2 - y_range2 * 0.05, y_max2 + y_range2 * 0.05]);

% legend('Physical Distance (No Margin)', 'Location', 'best', 'FontSize', 15);

end
% function plot_hocbf_margins(data)
% % 高次CBF（HOCBF）の各階層余裕度 h1〜h4 (位置, 速度, 加速度, 加加速度) 描画関数
% % （すべて独立したウィンドウで出力）
% 
% % 1. 時間 t > 0 の要素のインデックスを見つける
% t_pos_idx = (data.t > 0);
% 
% % 2. 時間 t > 0 のデータ切り出し
% t_plot = data.t(t_pos_idx);
% 
% h1 = data.cbf.log_h(1, t_pos_idx); % 1階層: 位置レベル (h1 >= 0)
% h2 = data.cbf.log_h(2, t_pos_idx); % 2階層: 速度レベル (h2 >= 0)
% h3 = data.cbf.log_h(3, t_pos_idx); % 3階層: 加速度レベル (h3 >= 0)
% h4 = data.cbf.log_h(4, t_pos_idx); % 4階層: 加加速度レベル (h4 >= 0)
% 
% %% =========================================================================
% %% 1. ウィンドウ 1: h1 (位置レベル: Position Margin)
% %% =========================================================================
% figure('Name', 'HOCBF 1st Order Margin (h1: Position)');
% 
% plot(t_plot, h1, 'Color', [0.85, 0.33, 0.10], 'LineWidth', 2.0); hold on;
% yline(0, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Safety Boundary (0)');
% 
% grid on;
% set(gca, 'FontSize', 14);
% xlabel('Time [s]', 'FontSize', 16);
% ylabel('h_1 (Position)', 'FontSize', 16);
% title('1st Order: Position Level Margin (h_1 \geq 0)', 'FontSize', 18);
% 
% xlim([t_plot(1), t_plot(end)]);
% legend('h_1 (Position)', 'Safety Boundary (0)', 'Location', 'best', 'FontSize', 14);
% hold off;
% 
% %% =========================================================================
% %% 2. ウィンドウ 2: h2 (速度レベル: Velocity Margin)
% %% =========================================================================
% figure('Name', 'HOCBF 2nd Order Margin (h2: Velocity)');
% 
% plot(t_plot, h2, 'Color', [0.00, 0.45, 0.74], 'LineWidth', 2.0); hold on;
% yline(0, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Safety Boundary (0)');
% 
% grid on;
% set(gca, 'FontSize', 14);
% xlabel('Time [s]', 'FontSize', 16);
% ylabel('h_2 (Velocity)', 'FontSize', 16);
% title('2nd Order: Velocity Level Margin (h_2 \geq 0)', 'FontSize', 18);
% 
% xlim([t_plot(1), t_plot(end)]);
% legend('h_2 (Velocity)', 'Safety Boundary (0)', 'Location', 'best', 'FontSize', 14);
% hold off;
% 
% %% =========================================================================
% %% 3. ウィンドウ 3: h3 (加速度レベル: Acceleration Margin)
% %% =========================================================================
% figure('Name', 'HOCBF 3rd Order Margin (h3: Acceleration)');
% 
% plot(t_plot, h3, 'Color', [0.47, 0.67, 0.19], 'LineWidth', 2.0); hold on;
% yline(0, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Safety Boundary (0)');
% 
% grid on;
% set(gca, 'FontSize', 14);
% xlabel('Time [s]', 'FontSize', 16);
% ylabel('h_3 (Acceleration)', 'FontSize', 16);
% title('3rd Order: Acceleration Level Margin (h_3 \geq 0)', 'FontSize', 18);
% 
% xlim([t_plot(1), t_plot(end)]);
% legend('h_3 (Acceleration)', 'Safety Boundary (0)', 'Location', 'best', 'FontSize', 14);
% hold off;
% 
% %% =========================================================================
% %% 4. ウィンドウ 4: h4 (加加速度レベル: Jerk Margin)
% %% =========================================================================
% figure('Name', 'HOCBF 4th Order Margin (h4: Jerk)');
% 
% plot(t_plot, h4, 'Color', [0.49, 0.18, 0.56], 'LineWidth', 2.0); hold on;
% yline(0, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Safety Boundary (0)');
% 
% grid on;
% set(gca, 'FontSize', 14);
% xlabel('Time [s]', 'FontSize', 16);
% ylabel('h_4 (Jerk)', 'FontSize', 16);
% title('4th Order: Jerk Level Margin (h_4 \geq 0)', 'FontSize', 18);
% 
% xlim([t_plot(1), t_plot(end)]);
% legend('h_4 (Jerk)', 'Safety Boundary (0)', 'Location', 'best', 'FontSize', 14);
% hold off;
% 
% %% =========================================================================
% %% 5. ウィンドウ 5: 全階層 (h1〜h4) の重ね合わせ比較プロット
% %% =========================================================================
% figure('Name', 'HOCBF Margins Comparison (Overlaid)');
% 
% yline(0, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Safety Boundary (0)'); hold on;
% 
% plot(t_plot, h1, 'Color', [0.85, 0.33, 0.10], 'LineStyle', '-',  'LineWidth', 2.0, 'DisplayName', 'h_1 (Position)');
% plot(t_plot, h2, 'Color', [0.00, 0.45, 0.74], 'LineStyle', '--', 'LineWidth', 1.8, 'DisplayName', 'h_2 (Velocity)');
% plot(t_plot, h3, 'Color', [0.47, 0.67, 0.19], 'LineStyle', '-.', 'LineWidth', 1.8, 'DisplayName', 'h_3 (Acceleration)');
% plot(t_plot, h4, 'Color', [0.49, 0.18, 0.56], 'LineStyle', ':',  'LineWidth', 2.0, 'DisplayName', 'h_4 (Jerk)');
% 
% grid on;
% set(gca, 'FontSize', 14);
% xlabel('Time [s]', 'FontSize', 16);
% ylabel('HOCBF Margin Value', 'FontSize', 16);
% title('HOCBF Margins Over Time (h_1 \sim h_4)', 'FontSize', 18);
% 
% xlim([t_plot(1), t_plot(end)]);
% legend('Location', 'best', 'FontSize', 14);
% hold off;
% 
% end

function plot_hocbf_margins(data)
% 高次CBF（HOCBF）の各階層余裕度 h1〜h4 (位置, 速度, 加速度, 加加速度) 描画関数
% （すべて独立したウィンドウで出力）

% 1. 時間 t > 0 の要素のインデックスを見つける
t_pos_idx = (data.t > 0);

% 2. 時間 t > 0 のデータ切り出し
t_plot = data.t(t_pos_idx);
h1 = data.cbf.log_h(1, t_pos_idx); % 1階層: 位置レベル (h1 >= 0)
h2 = data.cbf.log_h(2, t_pos_idx); % 2階層: 速度レベル (h2 >= 0)
h3 = data.cbf.log_h(3, t_pos_idx); % 3階層: 加速度レベル (h3 >= 0)
h4 = data.cbf.log_h(4, t_pos_idx); % 4階層: 加加速度レベル (h4 >= 0)

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
%% 1. ウィンドウ 1: h1 (位置レベル: Position Margin)
%% =========================================================================
figure('Name', 'HOCBF 1st Order Margin (h1: Position)', 'Position', fig_pos);
plot(t_plot, h1, 'Color', [0.85, 0.33, 0.10], 'LineWidth', 2.0); hold on;
yline(0, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Safety Boundary (0)');
grid on;
set(gca, 'FontSize', 14);
xlabel('Time [s]', 'FontSize', 16);
ylabel('h_1 (Position)', 'FontSize', 16);
title('1st Order: Position Level Margin (h_1 \geq 0)', 'FontSize', 18);
xlim([t_plot(1), t_plot(end)]);

% ★ Y軸の上下に少し余白を追加（下側: 5%, 上側: 15%）
y_data1  = [h1, 0];
y_min1   = min(y_data1);
y_max1   = max(y_data1);
y_range1 = y_max1 - y_min1;

if y_range1 == 0
    ylim([y_min1 - 1, y_max1 + 1]);
else
    ylim([y_min1 - y_range1 * 0.05, y_max1 + y_range1 * 0.15]);
end

legend('h_1 (Position)', 'Safety Boundary (0)', 'Location', 'best', 'FontSize', 14);
hold off;

%% =========================================================================
%% 2. ウィンドウ 2: h2 (速度レベル: Velocity Margin)
%% =========================================================================
figure('Name', 'HOCBF 2nd Order Margin (h2: Velocity)', 'Position', fig_pos);
plot(t_plot, h2, 'Color', [0.00, 0.45, 0.74], 'LineWidth', 2.0); hold on;
yline(0, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Safety Boundary (0)');
grid on;
set(gca, 'FontSize', 14);
xlabel('Time [s]', 'FontSize', 16);
ylabel('h_2 (Velocity)', 'FontSize', 16);
title('2nd Order: Velocity Level Margin (h_2 \geq 0)', 'FontSize', 18);
xlim([t_plot(1), t_plot(end)]);

% ★ Y軸の上下に少し余白を追加（下側: 5%, 上側: 15%）
y_data2  = [h2, 0];
y_min2   = min(y_data2);
y_max2   = max(y_data2);
y_range2 = y_max2 - y_min2;

if y_range2 == 0
    ylim([y_min2 - 1, y_max2 + 1]);
else
    ylim([y_min2 - y_range2 * 0.05, y_max2 + y_range2 * 0.15]);
end

legend('h_2 (Velocity)', 'Safety Boundary (0)', 'Location', 'best', 'FontSize', 14);
hold off;

%% =========================================================================
%% 3. ウィンドウ 3: h3 (加速度レベル: Acceleration Margin)
%% =========================================================================
figure('Name', 'HOCBF 3rd Order Margin (h3: Acceleration)', 'Position', fig_pos);
plot(t_plot, h3, 'Color', [0.47, 0.67, 0.19], 'LineWidth', 2.0); hold on;
yline(0, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Safety Boundary (0)');
grid on;
set(gca, 'FontSize', 14);
xlabel('Time [s]', 'FontSize', 16);
ylabel('h_3 (Acceleration)', 'FontSize', 16);
title('3rd Order: Acceleration Level Margin (h_3 \geq 0)', 'FontSize', 18);
xlim([t_plot(1), t_plot(end)]);

% ★ Y軸の上下に少し余白を追加（下側: 5%, 上側: 15%）
y_data3  = [h3, 0];
y_min3   = min(y_data3);
y_max3   = max(y_data3);
y_range3 = y_max3 - y_min3;

if y_range3 == 0
    ylim([y_min3 - 1, y_max3 + 1]);
else
    ylim([y_min3 - y_range3 * 0.05, y_max3 + y_range3 * 0.15]);
end

legend('h_3 (Acceleration)', 'Safety Boundary (0)', 'Location', 'best', 'FontSize', 14);
hold off;

%% =========================================================================
%% 4. ウィンドウ 4: h4 (加加速度レベル: Jerk Margin)
%% =========================================================================
figure('Name', 'HOCBF 4th Order Margin (h4: Jerk)', 'Position', fig_pos);
plot(t_plot, h4, 'Color', [0.49, 0.18, 0.56], 'LineWidth', 2.0); hold on;
yline(0, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Safety Boundary (0)');
grid on;
set(gca, 'FontSize', 14);
xlabel('Time [s]', 'FontSize', 16);
ylabel('h_4 (Jerk)', 'FontSize', 16);
title('4th Order: Jerk Level Margin (h_4 \geq 0)', 'FontSize', 18);
xlim([t_plot(1), t_plot(end)]);

% ★ Y軸の上下に少し余白を追加（下側: 5%, 上側: 15%）
y_data4  = [h4, 0];
y_min4   = min(y_data4);
y_max4   = max(y_data4);
y_range4 = y_max4 - y_min4;

if y_range4 == 0
    ylim([y_min4 - 1, y_max4 + 1]);
else
    ylim([y_min4 - y_range4 * 0.05, y_max4 + y_range4 * 0.15]);
end

legend('h_4 (Jerk)', 'Safety Boundary (0)', 'Location', 'best', 'FontSize', 14);
hold off;

%% =========================================================================
%% 5. ウィンドウ 5: 全階層 (h1〜h4) の重ね合わせ比較プロット
%% =========================================================================
figure('Name', 'HOCBF Margins Comparison (Overlaid)', 'Position', fig_pos);
yline(0, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Safety Boundary (0)'); hold on;
plot(t_plot, h1, 'Color', [0.85, 0.33, 0.10], 'LineStyle', '-',  'LineWidth', 2.0, 'DisplayName', 'h_1 (Position)');
plot(t_plot, h2, 'Color', [0.00, 0.45, 0.74], 'LineStyle', '--', 'LineWidth', 1.8, 'DisplayName', 'h_2 (Velocity)');
plot(t_plot, h3, 'Color', [0.47, 0.67, 0.19], 'LineStyle', '-.', 'LineWidth', 1.8, 'DisplayName', 'h_3 (Acceleration)');
plot(t_plot, h4, 'Color', [0.49, 0.18, 0.56], 'LineStyle', ':',  'LineWidth', 2.0, 'DisplayName', 'h_4 (Jerk)');
grid on;
set(gca, 'FontSize', 14);
xlabel('Time [s]', 'FontSize', 16);
ylabel('HOCBF Margin Value', 'FontSize', 16);
title('HOCBF Margins Over Time (h_1 \sim h_4)', 'FontSize', 18);
xlim([t_plot(1), t_plot(end)]);

% ★ Y軸の上下に少し余白を追加（下側: 5%, 上側: 15%）
y_data5  = [h1, h2, h3, h4, 0];
y_min5   = min(y_data5);
y_max5   = max(y_data5);
y_range5 = y_max5 - y_min5;

if y_range5 == 0
    ylim([y_min5 - 1, y_max5 + 1]);
else
    ylim([y_min5 - y_range5 * 0.05, y_max5 + y_range5 * 0.15]);
end

legend('Location', 'best', 'FontSize', 14);
hold off;

%% =========================================================================
%% 6. HOCBF (h1〜h4) の制約違反判定 & 最初に起動した制約の解析（複数回違反対応版）
%% =========================================================================
eps_tol = 1e-6; % 浮動小数点誤差を考慮した許容スレッショルド

% 各階層のデータと名称の定義
h_matrix = [h1; h2; h3; h4];
h_names  = {'h1 (位置)', 'h2 (速度)', 'h3 (加速度)', 'h4 (加加速度/入力)'};

disp('====================================================');
disp('   HOCBF (h1〜h4) 階層別・制約検証および起動順解析   ');
disp('====================================================');

%% 1. 各階層の違反 (h_k < 0) 判定と詳細表示（離散区間の分解機能付き）
has_violation_any = false;

for k = 1:4
    hk_data = h_matrix(k, :);
    viol_idx = find(hk_data < -eps_tol);
    
    if isempty(viol_idx)
        fprintf('✅ %s : 違反なし (全区間で h_%d >= 0 を保持)\n', h_names{k}, k);
    else
        has_violation_any = true;
        
        % インデックスの飛躍（離散した区間）を検出してグループ分け
        % インデックスの差分が1より大きい場所で分割
        split_pts = find(diff(viol_idx) > 1);
        start_indices = [viol_idx(1), viol_idx(split_pts + 1)];
        end_indices   = [viol_idx(split_pts), viol_idx(end)];
        
        num_blocks = length(start_indices);
        fprintf('⚠️ 警告: %s で制約違反が発生！ (計 %d ポイント / 計 %d 回の期間に分散)\n', ...
                h_names{k}, length(viol_idx), num_blocks);
        
        % 違反区間（1回目、2回目...）ごとに詳細表示
        for b = 1:num_blocks
            b_start_idx = start_indices(b);
            b_end_idx   = end_indices(b);
            
            t_start = t_plot(b_start_idx);
            t_end   = t_plot(b_end_idx);
            
            % その違反期間中における最大違反量（最小値）
            [max_viol_val, local_min_idx] = min(hk_data(b_start_idx:b_end_idx));
            t_max_viol = t_plot(b_start_idx + local_min_idx - 1);
            
            fprintf('   ▶ 違反区間 [%d/%d]: %.3f 秒 ～ %.3f 秒 (ピーク時刻: %.3f 秒, 最大違反量: %.4e)\n', ...
                    b, num_blocks, t_start, t_end, t_max_viol, max_viol_val);
        end
        
        fprintf('   全違反発生時間 [s]:\n');
        disp(t_plot(viol_idx).');
    end
end

if ~has_violation_any
    disp('----------------------------------------------------');
    disp('🎉 すべての階層 (h1〜h4) で制約違反は発生しませんでした。');
end

%% 2. どの階層の制約が一番初めに「起動（マージン最小へ低下）」したかの判定
disp('----------------------------------------------------');
disp('🔍 各階層の制約活性化（最接近・限界到達）タイマー解析:');

first_activation_times = zeros(1, 4);
min_margins = zeros(1, 4);

for k = 1:4
    [min_val, min_idx] = min(h_matrix(k, :));
    min_margins(k) = min_val;
    first_activation_times(k) = t_plot(min_idx);
    
    fprintf('   - %s : 最小マージン到達時刻 t = %.3f 秒 (最小値: %.4e)\n', ...
            h_names{k}, first_activation_times(k), min_margins(k));
end

% 最も早くマージンが底を打った（起動した）階層を特定
[earliest_time, earliest_k] = min(first_activation_times);

disp('----------------------------------------------------');
fprintf('★ 一番初めに制約が起動（マージン最小に到達）した階層:\n');
fprintf('   👉 【 %s 】 (時刻: %.3f 秒 / 最小マージン: %.4e)\n', ...
        h_names{earliest_k}, earliest_time, min_margins(earliest_k));
disp('====================================================');

end
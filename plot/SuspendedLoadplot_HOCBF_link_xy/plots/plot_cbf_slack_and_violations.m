% function plot_cbf_slack_and_violations(data)
% % CBFのスラック値（公称入力での違反量 vs 補正後の安全余裕度）および
% % 違反障害物数の関係性を可視化する関数（2つの独立したウィンドウで出力）
% 
% % 1. 時間 t > 0 の要素のインデックスを見つける
% t_pos_idx = (data.t > 0);
% 
% % 2. 時間 t > 0 のデータ切り出し
% t_plot       = data.t(t_pos_idx);
% slack_check  = data.cbf.slack_check(t_pos_idx);  % 公称入力での違反量（>0 で違反）
% slack_safe   = data.cbf.slack_safe(t_pos_idx);   % 適用後の安全度（<0 で異常）
% num_violated = data.cbf.num_violated(t_pos_idx); % 違反障害物数
% 
% %% =========================================================================
% %% 1. ウィンドウ 1: スラック値（安全余裕度）の比較プロット
% %% =========================================================================
% figure('Name', 'CBF Slack Value Comparison');
% 
% % 基準線 0（境界線）
% yline(0, 'k--', 'LineWidth', 1.2, 'DisplayName', 'Safety Boundary (0)'); hold on;
% 
% % 公称入力での違反量 (slack_check): 赤点線
% plot(t_plot, slack_check, 'Color', [0.85, 0.33, 0.10], 'LineStyle', '--', 'LineWidth', 1.8, ...
%      'DisplayName', 'slack\_check (Nominal Violation: >0 Failure)');
% 
% % CBF補正後の安全度 (slack_safe): 青実線
% plot(t_plot, slack_safe, 'Color', [0.00, 0.45, 0.74], 'LineStyle', '-', 'LineWidth', 2.0, ...
%      'DisplayName', 'slack\_safe (CBF Margin: <0 Danger)');
% 
% grid on;
% set(gca, 'FontSize', 14);
% xlabel('Time [s]', 'FontSize', 16);
% ylabel('Slack Value', 'FontSize', 16);
% title('CBF Safety Margin Evaluation (Slack Values)', 'FontSize', 18);
% 
% xlim([t_plot(1), t_plot(end)]);
% 
% legend('Location', 'best', 'FontSize', 14);
% hold off;
% 
% %% =========================================================================
% %% 2. ウィンドウ 2: 公称入力時に発生する違反障害物数 (num_violated)
% %% =========================================================================
% figure('Name', 'Number of Violated Obstacles (Nominal Input)');
% 
% % 違反障害物数のステップ表示
% stairs(t_plot, num_violated, 'Color', [0.49, 0.18, 0.56], 'LineWidth', 2.0, ...
%        'DisplayName', 'Violated Obstacles Count');
% 
% grid on;
% set(gca, 'FontSize', 14);
% xlabel('Time [s]', 'FontSize', 16);
% ylabel('Number of Obstacles', 'FontSize', 16);
% title('Number of Violated Obstacles without CBF', 'FontSize', 18);
% 
% xlim([t_plot(1), t_plot(end)]);
% 
% % 縦軸を整数目盛に設定
% max_num = max(num_violated);
% ylim([0, max(max_num + 1, 2)]);
% yticks(0:max(max_num + 1, 2));
% 
% legend('Location', 'best', 'FontSize', 14);
% 
% end
% 
% % slack_check（赤点線）: 0 を超えてプラスになっている区間は、公称入力のままでは障害物に進入・衝突する状況であることを示します。
% % 
% % slack_safe（青実線）: 常に 0 以上（あるいは 0 ぴったりに張り付く）を維持していれば、CBFによって制約内に確実に押し戻され、安全が維持されていることが証明されます（マイナスに落ち込んだ場合は異常）。

function plot_cbf_slack_and_violations(data)
% CBFのスラック値（公称入力での違反量 vs 補正後の安全余裕度）および
% 違反障害物数の関係性を可視化する関数（2つの独立したウィンドウで出力）

% 1. 時間 t > 0 の要素のインデックスを見つける
t_pos_idx = (data.t > 0);

% 2. 時間 t > 0 のデータ切り出し
t_plot       = data.t(t_pos_idx);
slack_check  = data.cbf.slack_check(t_pos_idx);  % 公称入力での違反量（>0 で違反）
slack_safe   = data.cbf.slack_safe(t_pos_idx);   % 適用後の安全度（<0 で異常）
num_violated = data.cbf.num_violated(t_pos_idx); % 違反障害物数

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
%% 1. ウィンドウ 1: スラック値（安全余裕度）の比較プロット
%% =========================================================================
figure('Name', 'CBF Slack Value Comparison', 'Position', fig_pos);

% 基準線 0（境界線）
yline(0, 'k--', 'LineWidth', 1.2, 'DisplayName', 'Safety Boundary (0)'); hold on;

% 公称入力での違反量 (slack_check): 赤点線
plot(t_plot, slack_check, 'Color', [0.85, 0.33, 0.10], 'LineStyle', '--', 'LineWidth', 1.8, ...
    'DisplayName', 'slack\_check (Nominal Violation: >0 Failure)');

% CBF補正後の安全度 (slack_safe): 青実線
plot(t_plot, slack_safe, 'Color', [0.00, 0.45, 0.74], 'LineStyle', '-', 'LineWidth', 2.0, ...
    'DisplayName', 'slack\_safe (CBF Margin: <0 Danger)');

grid on;
set(gca, 'FontSize', 14);
xlabel('Time [s]', 'FontSize', 16);
ylabel('Slack Value', 'FontSize', 16);
title('CBF Safety Margin Evaluation (Slack Values)', 'FontSize', 18);
xlim([t_plot(1), t_plot(end)]);

% ★ Y軸の上下に少し余白を追加（下側: 5%, 上側: 15%）
y_data1  = [slack_check, slack_safe, 0]; % 基準線0も考慮
y_min1   = min(y_data1);
y_max1   = max(y_data1);
y_range1 = y_max1 - y_min1;

if y_range1 == 0
    ylim([y_min1 - 1, y_max1 + 1]);
else
    ylim([y_min1 - y_range1 * 0.05, y_max1 + y_range1 * 0.15]);
end

legend('Location', 'best', 'FontSize', 14);
hold off;

%% =========================================================================
%% 2. ウィンドウ 2: 公称入力時に発生する違反障害物数 (num_violated)
%% =========================================================================
figure('Name', 'Number of Violated Obstacles (Nominal Input)', 'Position', fig_pos);

% 違反障害物数のステップ表示
stairs(t_plot, num_violated, 'Color', [0.49, 0.18, 0.56], 'LineWidth', 2.0, ...
    'DisplayName', 'Violated Obstacles Count');

grid on;
set(gca, 'FontSize', 14);
xlabel('Time [s]', 'FontSize', 16);
ylabel('Number of Obstacles', 'FontSize', 16);
title('Number of Violated Obstacles without CBF', 'FontSize', 18);
xlim([t_plot(1), t_plot(end)]);

% ★ 件数データのため下限は0固定とし、上限側に余裕を持たせる設定
max_num = max(num_violated);
y_upper = max(max_num + 1, 2);

ylim([0, y_upper + 0.5]); % 上部に0.5件分の余白をプラス
yticks(0:y_upper);

legend('Location', 'best', 'FontSize', 14);

end
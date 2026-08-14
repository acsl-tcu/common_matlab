% function plot_qp_constraints(data)
% % QP安全制約 A * u <= b の関係性を可視化する関数
% % （2つの独立したウィンドウで出力）
% 
% % 1. 時間 t > 0 の要素のインデックスを見つける
% t_pos_idx = (data.t > 0);
% 
% % 2. 時間 t > 0 のデータ切り出し
% t_plot = data.t(t_pos_idx);
% 
% % 制約係数 A (1行目: Roll, 2行目: Pitch)
% A_roll  = data.cbf.A_xy_qp_list(1, t_pos_idx);
% A_pitch = data.cbf.A_xy_qp_list(2, t_pos_idx);
% 
% % 許容上限 b
% b_qp = data.cbf.b_xy_qp_list(t_pos_idx);
% 
% % 各入力 [Roll; Pitch]
% u_nom_roll  = data.cbf.tmp(2, t_pos_idx);     % 補正前 Roll
% u_nom_pitch = data.cbf.tmp(3, t_pos_idx);     % 補正前 Pitch
% 
% u_fix_roll  = data.cbf.tmp_fix(2, t_pos_idx); % 補正後 Roll
% u_fix_pitch = data.cbf.tmp_fix(3, t_pos_idx); % 補正後 Pitch
% 
% % 左辺 A * u の計算
% A_u_nom = A_roll .* u_nom_roll + A_pitch .* u_nom_pitch; % 公称入力時の A*u
% A_u_fix = A_roll .* u_fix_roll + A_pitch .* u_fix_pitch; % CBF補正後の A*u
% 
% %% =========================================================================
% %% 1. ウィンドウ 1: QP制約の成立関係 (A*u vs b) 【最重要】
% %% =========================================================================
% figure('Name', 'QP Constraint Fulfillment (A*u vs b)');
% 
% % 公称入力 A*u_nom (紫点線)
% plot(t_plot, A_u_nom, 'Color', [0.49, 0.18, 0.56], 'LineStyle', '--', 'LineWidth', 1.5, 'DisplayName', 'A \cdot u_{nom} (Nominal)'); hold on;
% 
% % CBF補正後 A*u_fix (青実線)
% plot(t_plot, A_u_fix, 'Color', [0.00, 0.45, 0.74], 'LineStyle', '-', 'LineWidth', 2.0, 'DisplayName', 'A \cdot u_{fix} (CBF Modified)');
% 
% % 許容上限 b_qp (赤実線/太線)
% plot(t_plot, b_qp, 'Color', [0.85, 0.33, 0.10], 'LineStyle', '-', 'LineWidth', 2.0, 'DisplayName', 'b_{qp} (Upper Bound)');
% 
% grid on;
% set(gca, 'FontSize', 14);
% xlabel('Time [s]', 'FontSize', 16);
% ylabel('Constraint Value', 'FontSize', 16);
% title('QP Safety Constraint: A \cdot u \leq b', 'FontSize', 18);
% 
% xlim([t_plot(1), t_plot(end)]);
% 
% legend('A \cdot u_{nom} (Nominal)', 'A \cdot u_{fix} (CBF Modified)', 'b_{qp} (Upper Bound)', ...
%        'Location', 'best', 'FontSize', 14);
% hold off;
% 
% %% =========================================================================
% %% 2. ウィンドウ 2: 各方向の制約係数 A_roll, A_pitch の推移
% %% =========================================================================
% figure('Name', 'QP Constraint Coefficients (A_roll, A_pitch)');
% 
% plot(t_plot, A_roll,  'Color', [0.85, 0.325, 0.098], 'LineStyle', '-', 'LineWidth', 2, 'DisplayName', 'A_{roll}'); hold on;
% plot(t_plot, A_pitch, 'Color', [0.47, 0.67, 0.19],  'LineStyle', '-', 'LineWidth', 2, 'DisplayName', 'A_{pitch}');
% 
% grid on;
% set(gca, 'FontSize', 14);
% xlabel('Time [s]', 'FontSize', 16);
% ylabel('Coefficient Value', 'FontSize', 16);
% title('Constraint Direction Coefficients (A_{qp})', 'FontSize', 18);
% 
% xlim([t_plot(1), t_plot(end)]);
% 
% legend('A_{roll}', 'A_{pitch}', 'Location', 'best', 'FontSize', 14);
% hold off;
% 
% end

function plot_qp_constraints(data)
% QP安全制約 A * u <= b の関係性を可視化する関数
% （2つの独立したウィンドウで出力）

% 1. 時間 t > 0 の要素のインデックスを見つける
t_pos_idx = (data.t > 0);

% 2. 時間 t > 0 のデータ切り出し
t_plot = data.t(t_pos_idx);

% 制約係数 A (1行目: Roll, 2行目: Pitch)
A_roll  = data.cbf.A_xy_qp_list(1, t_pos_idx);
A_pitch = data.cbf.A_xy_qp_list(2, t_pos_idx);

% 許容上限 b
b_qp = data.cbf.b_xy_qp_list(t_pos_idx);

% 各入力 [Roll; Pitch]
u_nom_roll  = data.cbf.tmp(2, t_pos_idx);     % 補正前 Roll
u_nom_pitch = data.cbf.tmp(3, t_pos_idx);     % 補正前 Pitch
u_fix_roll  = data.cbf.tmp_fix(2, t_pos_idx); % 補正後 Roll
u_fix_pitch = data.cbf.tmp_fix(3, t_pos_idx); % 補正後 Pitch

% 左辺 A * u の計算
A_u_nom = A_roll .* u_nom_roll + A_pitch .* u_nom_pitch; % 公称入力時の A*u
A_u_fix = A_roll .* u_fix_roll + A_pitch .* u_fix_pitch; % CBF補正後の A*u

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
%% 1. ウィンドウ 1: QP制約の成立関係 (A*u vs b) 【最重要】
%% =========================================================================
figure('Name', 'QP Constraint Fulfillment (A*u vs b)', 'Position', fig_pos);

% 公称入力 A*u_nom (紫点線)
plot(t_plot, A_u_nom, 'Color', [0.49, 0.18, 0.56], 'LineStyle', '--', 'LineWidth', 1.5, 'DisplayName', 'A \cdot u_{nom} (Nominal)'); hold on;
% CBF補正後 A*u_fix (青実線)
plot(t_plot, A_u_fix, 'Color', [0.00, 0.45, 0.74], 'LineStyle', '-', 'LineWidth', 2.0, 'DisplayName', 'A \cdot u_{fix} (CBF Modified)');
% 許容上限 b_qp (赤実線/太線)
plot(t_plot, b_qp, 'Color', [0.85, 0.33, 0.10], 'LineStyle', '-', 'LineWidth', 2.0, 'DisplayName', 'b_{qp} (Upper Bound)');

grid on;
set(gca, 'FontSize', 14);
xlabel('Time [s]', 'FontSize', 16);
ylabel('Constraint Value', 'FontSize', 16);
xlim([t_plot(1), t_plot(end)]);

% ★ Y軸の上下に少し余白を追加（下側: 5%, 上側: 15%）
y_data1  = [A_u_nom, A_u_fix, b_qp];
y_min1   = min(y_data1);
y_max1   = max(y_data1);
y_range1 = y_max1 - y_min1;

if y_range1 == 0
    ylim([y_min1 - 1, y_max1 + 1]);
else
    ylim([y_min1 - y_range1 * 0.05, y_max1 + y_range1 * 0.15]);
end

legend('A_{qp}\cdotu_{nom}', 'A_{qp}\cdotu^*', 'b_{qp}', ...
       'Location', 'best', 'FontSize', 14);
hold off;

% =========================================================================
% 制約違反（A_u_fix > b_qp）の自動判定および結果表示
% =========================================================================
eps_tol = 1e-6; % 浮動小数点誤差を考慮した許容スレッショルド
viol_idx = find(A_u_fix - b_qp > eps_tol);

if isempty(viol_idx)
    disp('========================================');
    disp('✅ 制約条件は全区間で完全に満たされています (A*u* <= b_qp)');
    disp('========================================');
else
    viol_times = t_plot(viol_idx);
    disp('========================================');
    fprintf('⚠️ 警告: 制約違反が発生しました (計 %d ポイント)\n', length(viol_idx));
    fprintf('違反時間帯: %.3f 秒 ～ %.3f 秒\n', viol_times(1), viol_times(end));
    disp('全違反発生時間 [s]:');
    disp(viol_times.');
    disp('========================================');
end

% %% =========================================================================
% %% 2. ウィンドウ 2: 各方向の制約係数 A_roll, A_pitch の推移
% %% =========================================================================
% figure('Name', 'QP Constraint Coefficients (A_roll, A_pitch)', 'Position', fig_pos);
% 
% plot(t_plot, A_roll,  'Color', [0.85, 0.325, 0.098], 'LineStyle', '-', 'LineWidth', 2, 'DisplayName', 'A_{roll}'); hold on;
% plot(t_plot, A_pitch, 'Color', [0.47, 0.67, 0.19],  'LineStyle', '-', 'LineWidth', 2, 'DisplayName', 'A_{pitch}');
% 
% grid on;
% set(gca, 'FontSize', 14);
% xlabel('Time [s]', 'FontSize', 16);
% ylabel('Coefficient Value', 'FontSize', 16);
% xlim([t_plot(1), t_plot(end)]);
% 
% % ★ Y軸の上下に少し余白を追加（下側: 5%, 上側: 15%）
% y_data2  = [A_roll, A_pitch];
% y_min2   = min(y_data2);
% y_max2   = max(y_data2);
% y_range2 = y_max2 - y_min2;
% 
% if y_range2 == 0
%     ylim([y_min2 - 1, y_max2 + 1]);
% else
%     ylim([y_min2 - y_range2 * 0.05, y_max2 + y_range2 * 0.05]);
% end
% 
% legend('A_{roll}', 'A_{pitch}', 'Location', 'best', 'FontSize', 14);
% hold off;

end
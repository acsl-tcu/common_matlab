clc;clear; close all;
% (ユーザーから提供された絶対パスを使用)
% file1_name = "\\Ws2023\ws2025\Work2025\YosukeKOSEKI\Drone results(Exp_data)\2025.12.01_For_IFAC_Exp\Ashigara\P2P_1_1_1.5_HLLQR_Log(01-Dec-2025_18_13_15).mat";
% file2_name = "\\Ws2023\ws2025\Work2025\YosukeKOSEKI\Drone results(Exp_data)\2025.12.01_For_IFAC_Exp\Ashigara\P2P_1_1_1.5_NNMEC_Log(01-Dec-2025_18_15_10).mat";
% %↑P2P

% file1_name = "\\Ws2023\ws2025\Work2025\YosukeKOSEKI\Drone results(Exp_data)\2025.12.01_For_IFAC_Exp\Ashigara\P2P_1_1_1.5_HLLQR_R=0.5_Log(01-Dec-2025_18_49_23).mat";
% file2_name = "\\Ws2023\ws2025\Work2025\YosukeKOSEKI\Drone results(Exp_data)\2025.12.01_For_IFAC_Exp\Ashigara\P2P_1_1_1.5_NNMEC_R=0.5_Log(01-Dec-2025_18_51_06).mat";
% % P2P R=0.5
% 
% file1_name = "\\Ws2023\ws2025\Work2025\YosukeKOSEKI\Drone results(Exp_data)\2025.12.01_For_IFAC_Exp\Ashigara\lemniscate_HLLQR_Log(01-Dec-2025_18_19_14).mat";
% file2_name = "\\Ws2023\ws2025\Work2025\YosukeKOSEKI\Drone results(Exp_data)\2025.12.01_For_IFAC_Exp\Ashigara\lemniscate_NNMEC_Log(01-Dec-2025_18_20_55).mat";
% % lemniscate
% 
% file1_name = "\\Ws2023\ws2025\Work2025\YosukeKOSEKI\Drone results(Exp_data)\2025.12.01_For_IFAC_Exp\Ashigara\Circle_HLLQR_Log(01-Dec-2025_18_33_56).mat";
% file2_name = "\\Ws2023\ws2025\Work2025\YosukeKOSEKI\Drone results(Exp_data)\2025.12.01_For_IFAC_Exp\Ashigara\Circle_NNMEC_Log(01-Dec-2025_18_35_31).mat";
% % ↑circle
% file1_name = "\\Ws2023\ws2025\Work2025\YosukeKOSEKI\Drone results(Exp_data)\2025.12.01_For_IFAC_Exp\Ashigara\Circle_HLLQR_R=0.5_Log(01-Dec-2025_18_42_29).mat";
% file2_name = "\\Ws2023\ws2025\Work2025\YosukeKOSEKI\Drone results(Exp_data)\2025.12.01_For_IFAC_Exp\Ashigara\Circle_NNMEC_R=0.5_Log(01-Dec-2025_18_46_08).mat";
% % ↑circle R=0.5
% file1_name = "\\Ws2023\ws2025\Work2025\YosukeKOSEKI\Drone results(Exp_data)\2025.12.01_For_IFAC_Exp\Ashigara\Circle_HLLQR_R=0.1_Log(01-Dec-2025_18_39_05).mat";
% file2_name = "\\Ws2023\ws2025\Work2025\YosukeKOSEKI\Drone results(Exp_data)\2025.12.01_For_IFAC_Exp\Ashigara\Circle_NNMEC_R=0.1_Log(01-Dec-2025_18_44_22).mat";
% % ↑circle R=0.1

% file1_name = "\\192.168.100.209\ws2025\Work2025\YosukeKOSEKI\Drone results(Exp_data)\2025.12.02_For_IFAC_Exp_again\Lemniscate_HLLQR_Log(02-Dec-2025_17_49_18).mat";
% file2_name = "\\192.168.100.209\ws2025\Work2025\YosukeKOSEKI\Drone results(Exp_data)\2025.12.02_For_IFAC_Exp_again\Lemniscate_NNMEC_Log(02-Dec-2025_17_59_21).mat";
% % ↑Exp lemniscate
file1_name = "\\192.168.100.209\ws2025\Work2025\YosukeKOSEKI\Drone results(Exp_data)\2025.12.02_For_IFAC_Exp_again\Lemniscate_HLLQR_R=0.1_Log(02-Dec-2025_18_07_43).mat";
file2_name = "\\192.168.100.209\ws2025\Work2025\YosukeKOSEKI\Drone results(Exp_data)\2025.12.02_For_IFAC_Exp_again\Lemniscate_NNMEC_R=0.1_Log(02-Dec-2025_18_09_45).mat";
% % ↑Exp lemniscate R=0.1

% file1_name = "\\192.168.100.209\ws2025\Work2025\YosukeKOSEKI\Drone results(Exp_data)\2025.12.02_For_IFAC_Exp_again\P2P_1_1_1.5_HLLQR_Log(02-Dec-2025_17_47_17).mat";
% file2_name = "\\192.168.100.209\ws2025\Work2025\YosukeKOSEKI\Drone results(Exp_data)\2025.12.02_For_IFAC_Exp_again\P2P_1_1_1.5_NNMEC_Log(02-Dec-2025_17_56_22).mat";
% % ↑Exp P2P
file1_name = "\\192.168.100.209\ws2025\Work2025\YosukeKOSEKI\Drone results(Exp_data)\2025.12.02_For_IFAC_Exp_again\P2P_1_1_1.5_HLLQR_R=0.1_Log(02-Dec-2025_18_14_33).mat";
file2_name = "\\192.168.100.209\ws2025\Work2025\YosukeKOSEKI\Drone results(Exp_data)\2025.12.02_For_IFAC_Exp_again\P2P_1_1_1.5_NNMEC_R=0.1_Log(02-Dec-2025_18_21_11).mat";
% ↑Exp P2P R=0.1

% file1_name = 
% file2_name = 
logger1 = LOGGER(file1_name);
logger2 = LOGGER(file2_name);


%% ===== 1. グローバル・プロットオプションの定義 =====
clearvars -except logger1 logger2 file1_name file2_name
FS = 16;  % Font Size (フォントサイズ)
LW = 1.5; % Line Width (ライン幅)

% --------------------- プロット設定 ---------------------
lgd1 = 'HLLQR';    % File 1の凡例名
lgd2 = 'HLLQR+NNMEC';    % File 2の凡例名
lgd_pos = 3;                 % 凡例を表示するグラフ番号 (1, 2, or 3)
fref = true;                 % リファレンスをプロットするかのフラグ (true/false)
% att = "q";                   % プロットする変数名 ("p", "v", "q", "w", "input"など)
% ylabel1 = '$\phi$ [rad]'; % 1軸目 (p1/v1/q1...) のY軸ラベル
% ylabel2 = '$\theta$ [rad]'; % 2軸目 (p2/v2/q2...) のY軸ラベル
% ylabel3 = '$\psi$ [rad]'; % 3軸目 (p3/v3/q3...) のY軸ラベル

att = "p";                   % プロットする変数名 ("p", "v", "q", "w", "input"など)
ylabel1 = '$x$ [m]'; % 1軸目 (p1/v1/q1...) のY軸ラベル
ylabel2 = '$y$ [m]'; % 2軸目 (p2/v2/q2...) のY軸ラベル
ylabel3 = '$z$ [m]'; % 3軸目 (p3/v3/q3...) のY軸ラベル
fp1_p2 = true;
% --------------------- グラフ出力設定 ---------------------
foutput = false;             % グラフ出力の有無 (true/false)
file_style = "eps";          % 出力ファイル形式 ("jpg", "png", "pdf", "eps")
folder_path = "plot/fig/IFAC"; 
file_name = "lemniscate_R=0.1_position";% 出力ファイル名 (拡張子なし)
% ------------------------------------------------------------------

LW_ref = LW - 0.5; % リファレンスのライン幅


% ===== 2. ファイル名と時間範囲の指定 =====
% ログの抽出設定
phase = "f"; % 対象フェーズ: "f" (Flight phase)
t_start = 0; % 抽出したい時間範囲の開始時間 [s]
t_end = 20;  % 抽出したい時間範囲の終了時間 [s] ←Simの時のみ有効, Expの時は大体の目安
xrange = [t_start t_end];

% ロギング間隔とインデックス計算 (dt=0.025を想定)
log_dt = 0.025; 
idx_start = t_start * (1/log_dt) + 1;
idx_end = t_end * (1/log_dt) + 1;

% プロット対象の変数を動的に設定
variable_base = att;
var1_str = strcat(variable_base, "1");
var2_str = strcat(variable_base, "2");
var3_str = strcat(variable_base, "3");

% ===== 3. データロードと抽出 =====

% 共通の時間軸データを抽出
t_all_1 = logger1.data(0, "t", "", "phase", phase);
t_all_2 = logger2.data(0, "t", "", "phase", phase);
t_1 = t_all_1(idx_start : idx_end) - t_all_1(1);
t_2 = t_all_2(idx_start : idx_end) - t_all_2(1);

% データ抽出ヘルパー関数
extract_data = @(logger, var, att_type) logger.data(1, var, att_type, "phase", phase);

% --- データ抽出 (Reference/Estimator 1/Estimator 2) ---
if fref
    ref_p1_all_1 = extract_data(logger1, var1_str, "r");
    ref_p2_all_1 = extract_data(logger1, var2_str, "r");
    ref_p3_all_1 = extract_data(logger1, var3_str, "r");
    ref_data_1 = {ref_p1_all_1(idx_start : idx_end), ref_p2_all_1(idx_start : idx_end), ref_p3_all_1(idx_start : idx_end)};

    ref_p1_all_2 = extract_data(logger2, var1_str, "r");
    ref_p2_all_2 = extract_data(logger2, var2_str, "r");
    ref_p3_all_2 = extract_data(logger2, var3_str, "r");
    ref_data_2 = {ref_p1_all_2(idx_start : idx_end), ref_p2_all_2(idx_start : idx_end), ref_p3_all_2(idx_start : idx_end)};
end

est1_p1_all = extract_data(logger1, var1_str, "e");
est1_p2_all = extract_data(logger1, var2_str, "e");
est1_p3_all = extract_data(logger1, var3_str, "e");
est1_data = {est1_p1_all(idx_start : idx_end), est1_p2_all(idx_start : idx_end), est1_p3_all(idx_start : idx_end)};

est2_p1_all = extract_data(logger2, var1_str, "e");
est2_p2_all = extract_data(logger2, var2_str, "e");
est2_p3_all = extract_data(logger2, var3_str, "e");
est2_data = {est2_p1_all(idx_start : idx_end), est2_p2_all(idx_start : idx_end), est2_p3_all(idx_start : idx_end)};

% プロット設定マッピング
plot_map = {
    {var1_str, ylabel1};
    {var2_str, ylabel2};
    {var3_str, ylabel3};
};

% --- RMSE Calculation (Estimator vs Reference) ---
N_1 = length(t_1);
N_2 = length(t_2);
if N_1 > 0 && N_2 > 0 && fref
    % RMSE for Estimator 1 (vs Reference)
    RMSE1_p1 = rmse(est1_data{1}, ref_data_1{1});
    RMSE1_p2 = rmse(est1_data{2}, ref_data_1{2});
    RMSE1_p3 = rmse(est1_data{3}, ref_data_1{3});
    
    % RMSE for Estimator 2 (vs Reference)
    RMSE2_p1 = rmse(est2_data{1}, ref_data_2{1});
    RMSE2_p2 = rmse(est2_data{2}, ref_data_2{2});
    RMSE2_p3 = rmse(est2_data{3}, ref_data_2{3});
    
    % 結果の表示
    disp(' ');
    disp('--- RMSE Results (Estimator vs Reference) ---');
    fprintf('%-15s | %-10s | %-10s | %-10s\n', 'Variable', [lgd1 ' RMSE'], [lgd2 ' RMSE'], 'Unit');
    fprintf('---------------------------------------------------\n');
    fprintf('%-15s | %10.4f | %10.4f | %-10s\n', strcat(variable_base, '1'), RMSE1_p1, RMSE2_p1, '[Unit]');
    fprintf('%-15s | %10.4f | %10.4f | %-10s\n', strcat(variable_base, '2'), RMSE1_p2, RMSE2_p2, '[Unit]');
    fprintf('%-15s | %10.4f | %10.4f | %-10s\n', strcat(variable_base, '3'), RMSE1_p3, RMSE2_p3, '[Unit]');
    disp('---------------------------------------------------');
else
    disp('Error: No data extracted in the specified range.');
end

% ===== 4. プロット実行 (3行1列のサブプロット) =====
figure(1);
clf;

for i = 1:3
    subplot(3, 1, i);
    
    current_est1 = est1_data{i};
    current_est2 = est2_data{i};
    current_var_str = plot_map{i}{1};
    current_ylabel = plot_map{i}{2};
    
    plegend = {};
    hold on;

    % リファレンスプロット (fref=trueの場合)
    if fref
        current_ref = ref_data_1{i};
        plot(t_1, current_ref, 'LineWidth', LW_ref, 'LineStyle', '--', 'Color', 'k'); % 黒色
        plegend = [plegend, {'Reference'}];
    end
    
    % Estimator 1 プロット (File 1)
    plot(t_1, current_est1, 'LineWidth', LW, 'LineStyle', '-', 'Color', 'b'); % 青色
    plegend = [plegend, {lgd1}];
    
    % Estimator 2 プロット (File 2)
    plot(t_2, current_est2, 'LineWidth', LW, 'LineStyle', '-', 'Color', 'r'); % 赤色
    plegend = [plegend, {lgd2}];

    hold off;
    
    % ラベル設定
    ylabel(current_ylabel, 'Interpreter','latex', 'FontSize', FS);
    grid on;
    set(gca, 'FontSize', FS-2);
    if t_1(1) < t_2(1), ini = t_1(1);
    else              , ini = t_2(1); end
    if t_1(end) > t_2(end), last = t_1(end);
    else                  , last = t_2(end); end
    xlim([ini last]);

    % 凡例は指定されたグラフ番号のみ表示
    if i == lgd_pos
        legend(plegend, 'FontSize', FS-4, 'Location', 'best');
    end

    % XLabelは一番下のグラフのみ表示
    if i == 3
        xlabel('Time [s]', 'FontSize', FS);
        % ylim([0.3 0.6])
    end
end

% ===== 5. グラフのエクスポート =====
if foutput
    % フォルダが存在しない場合は作成
    if ~exist(folder_path, 'dir')
        mkdir(folder_path);
    end
    
    % 出力ファイルパスの生成
    output_filename = fullfile(folder_path, file_name + "." + file_style);

    % Figure 1 をエクスポート
    fig = figure(1); 
    
    % exportgraphicsを使用して保存
    try
        % JPEG, PNG, PDF, EPSはexportgraphicsで対応可能
        exportgraphics(fig, output_filename, 'Resolution', 300); % 解像度を300 DPIに設定
        disp(['Graph successfully exported to: ', output_filename]);
    catch ME
        disp(['Error exporting graph: ', ME.message]);
        disp('Please ensure the specified file_style and folder_path are valid.');
    end
end

% --- 既存のコードの後にこのブロックを追加してください ---

% ===== 6. X-Y 軌跡プロット (p1 vs p2) =====

% プロット対象の変数を設定（ここではp1 vs p2を固定）
if fp1_p2
    variable_base_xy = "p";
    var1_str_xy = "p1";
    var2_str_xy = "p2";
    
    % データの再抽出（もし att が 'p' 以外に設定されていた場合でも、p1/p2を使用するため）
    % ただし、現在のコードでは att="p" なので、既存のデータ(ref_data, est*_data)を流用します。
    % X軸データ: p1 (ref_data{1}, est1_data{1}, est2_data{1})
    % Y軸データ: p2 (ref_data{2}, est1_data{2}, est2_data{2})
    
    figure(2); % 新しいウィンドウ（Figure 2）を使用
    clf;
    
    plegend = {};
    hold on;
    
    % --- リファレンス軌跡プロット (fref=trueの場合) ---
    if fref
        % plot(1, 1, ...
        %     'o', ...                   % マーカーの種類を円 ('o') に指定
        %     'MarkerSize', 10, ...      % マーカーのサイズを指定（任意）
        %     'MarkerEdgeColor', 'k', ...% マーカーの枠線の色を青 ('b') に指定
        %     'MarkerFaceColor', 'k');   % マーカーの内部を赤 ('r') で塗りつぶし
        plot(ref_data_1{1}, ref_data_1{2}, 'LineWidth', LW_ref, 'LineStyle', '--', 'Color', 'k'); % 黒色
        plegend = [plegend, {'Reference'}];
    end
    
    % --- Estimator 1 軌跡プロット (File 1) ---
    plot(est1_data{1}, est1_data{2}, 'LineWidth', LW-0.5, 'LineStyle', '-', 'Color', 'b'); % 青色
    plegend = [plegend, {lgd1}];
    
    % --- Estimator 2 軌跡プロット (File 2) ---
    plot(est2_data{1}, est2_data{2}, 'LineWidth', LW, 'LineStyle', '-', 'Color', 'r'); % 赤色
    plegend = [plegend, {lgd2}];
    
    hold off;
    
    % ラベル設定
    % title('X-Y Trajectory (p_1 vs p_2)','FontSize', FS);
    xlabel(ylabel1, 'Interpreter','latex', 'FontSize', FS); % p1のラベルをX軸に
    ylabel(ylabel2, 'Interpreter','latex', 'FontSize', FS); % p2のラベルをY軸に
    grid on;
    axis equal; % 軸のスケールを合わせる
    legend(plegend, 'FontSize', FS-4, 'Location', 'southeast');
    set(gca, 'FontSize', FS-2);
    % xlim([0.7 1.1])
    % ylim([0.7 1.1])
end

% --- グラフのエクスポート (オプション) ---
if foutput
    % 出力ファイルパスの生成 (ファイル名に '_XY' を追加)
    file_name_xy = file_name + "_XY";
    output_filename_xy = fullfile(folder_path, file_name_xy + "." + file_style);
    
    fig = figure(2); 
    
    try
        exportgraphics(fig, output_filename_xy, 'Resolution', 300);
        disp(['X-Y Trajectory Graph successfully exported to: ', output_filename_xy]);
    catch ME
        disp(['Error exporting X-Y graph: ', ME.message]);
    end
end


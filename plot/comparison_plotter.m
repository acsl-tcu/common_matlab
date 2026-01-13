%% Initialize settings
% データをloggerに読み込むため，このセクションを実行
% Run this section to import the data into logger.
% % % set path
% % clear all
% % cf = pwd;
% % 
% % if contains(mfilename('fullpath'), "mainGUI")
% %     cd(fileparts(mfilename('fullpath')));
% % else
% %     tmp = matlab.desktop.editor.getActive;
% %     cd(fileparts(tmp.Filename));
% % end
% % 
% % [~, tmp] = regexp(genpath('.'), '\.\\\.git.*?;', 'match', 'split');
% % cellfun(@(xx) addpath(xx), tmp, 'UniformOutput', false);
% % close all hidden; clear; clc;
% % userpath('clear');

clc; clear; close all;
plt.filepathes = {... % TODO: ファイルのパスを記述
    "C:\Users\hiyou\Github\common_matlab\Data\Exp_data\2025.12.11_405Exp_various_trajectories\HLLQR_lemniscate_Log(11-Dec-2025_18_42_48).mat";
    "C:\Users\hiyou\Github\common_matlab\Data\Exp_data\2025.12.11_405Exp_various_trajectories\NN21MEC_lemniscate_Log(11-Dec-2025_18_44_08).mat";

    };
plt.filepathes = {... % TODO: ファイルのパスを記述
    "C:\Users\hiyou\Github\common_matlab\Data\Sim_data\For IFAC2026\lemniscate_HLLQR_R=0.05_Log(30-Nov-2025_21_24_44).mat";
    "C:\Users\hiyou\Github\common_matlab\Data\Sim_data\For IFAC2026\lemniscate_NNMEC_R=0.05_Log(30-Nov-2025_21_31_32).mat";
    };


[~, plt.filenames, exts] = cellfun(@fileparts, plt.filepathes, 'UniformOutput', false); % ファイル名の自動取得
plt.logger = cell(length(plt.filepathes),1);
for i = 1:length(plt.logger)
    plt.logger{i} = LOGGER(plt.filepathes{i}); % LOGGERクラスとしてcell配列で登録
end
plt.Num = length(plt.logger);
disp('Finish loading the data.')
% plotGUI

%% Run this section to plot
%  プロットするために このセクションを実行
clearvars -except plt

% [TODO list]
% 開始時刻、終了時刻を設定して、Exp, Simどちらでも対応できるようにする
% GUI上でplt.settings, plt.saveを変更できるようにする & どんなFigをoutputしたいか？subplot or plot
% lgd_pos(数字、グラフ内外)

plt.save.savefolder = "plot\fig";
plt.save.savename   = "dummy";
plt.save.style      = "pdf"; % 出力ファイル形式 ("jpg", "png", "pdf", "eps")
plt.save.fsave      = false;


phase = "f";
lgd = {...
    "HL-LQR";
    "NN-MEC"
    };
FS = 18;
LW = 1.5;
Time_Range = [0,10]; % 指定した最初のフェーズを0秒としている
plt.settings = plot_settings("phase",phase, "FontSize",FS, "LineWidth",LW, "TimeRange",Time_Range, "LegendName",lgd);
plt.settings.lgd_pos = 1; % 凡例を表示するグラフ番号 (1, 2, 3 or 4)
plt.settings.fcolor = false;
plt.settings.alpha = 0.7;


plt.labelmap = subplot_label_mapping;
% plt.labelmap = oneplot_label_mapping;

% ===== 時間データの取り出し =====
plt.data.all_time = cell(plt.Num,1);
plt.data.time = cell(plt.Num,1);
plt.data.idx = cell(plt.Num, 1);
for i = 1:plt.Num % 時間＆idxの取り出し
    tmp = plt.logger{i}.data(0, "t", "", "phase", plt.settings.phase);
    plt.data.all_time{i} = tmp - tmp(1); % 開始時刻を0にする
    start_idx   = find(plt.data.all_time{i} >= plt.settings.range(1), 1, 'first');
    end_idx     = find(plt.data.all_time{i} <= plt.settings.range(2), 1, 'last');
    plt.data.time{i} = plt.data.all_time{i}(start_idx:end_idx);
    if isempty(plt.data.time{i}), error('時間設定幅 [%s]が大きすぎます\n使用可能範囲: [%.4f, %.4f]',num2str(plt.settings.range), plt.data.all_time{i}(1), plt.data.all_time{i}(end)); end
    plt.data.idx{i} = [start_idx, end_idx];
    if plt.data.time{i}(end) == plt.data.all_time{i}(end), warning('時間設定幅 [%s]が大きすぎます\n使用可能範囲: [%.4f, %.4f]',num2str(plt.settings.range), plt.data.all_time{i}(1), plt.data.all_time{i}(end)); end
end


% ===== targetデータの取り出し =====
plt.data = extract_data(plt);



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
        current_ref = ref_data_2{i};
        plot(t_2, current_ref, 'LineWidth', LW_ref, 'LineStyle', '--', 'Color', 'k'); % 黒色
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
    % ylim([-0.1 0.1]);

    % 凡例は指定されたグラフ番号のみ表示
    if i == lgd_pos
        legend(plegend, 'FontSize', FS-4, 'Location', 'best');
    end

    % XLabelは一番下のグラフのみ表示
    if i == 3
        xlabel('Time [s]', 'FontSize', FS);
        % ylim([1.2 1.6])
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
    % legend(plegend, 'FontSize', FS-4, 'Location', 'northoutside');
    set(gca, 'FontSize', FS-2);
    % xlim([-1.15 1.15])
    % ylim([-0.4 0.4])
    xlim([-1.15 1.15])
    ylim([-0.5 0.5])
end

% --- グラフのエクスポート (オプション) ---
if foutput && fp1_p2
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


% % ========= Δuのプロット ==============%
ua1_all = extract_data(logger1, "input", "");
ua1_data = {ua1_all(idx_start : idx_end,1), ua1_all(idx_start : idx_end,2), ua1_all(idx_start : idx_end,3), ua1_all(idx_start : idx_end,4)};


un2_all = extract_data(logger2, "controller.result.nominal_input", "");
du2_all = extract_data(logger2, "controller.result.delta_input", "");
ua2_all = extract_data(logger2, "input", "");

un2_data = {un2_all(idx_start : idx_end,1), un2_all(idx_start : idx_end,2), un2_all(idx_start : idx_end,3), un2_all(idx_start : idx_end,4)};
du2_data = {du2_all(idx_start : idx_end,1), du2_all(idx_start : idx_end,2), du2_all(idx_start : idx_end,3), du2_all(idx_start : idx_end,4)};
ua2_data = {ua2_all(idx_start : idx_end,1), ua2_all(idx_start : idx_end,2), ua2_all(idx_start : idx_end,3), ua2_all(idx_start : idx_end,4)};

input_ylabel = {'$T$ [N]', '$ \tau_{roll}$ [Nm]', '$\tau_{pitch}$ [Nm]', '$\tau_{yaw}$ [Nm]'};

figure(3)
clf;
LW_input = LW-0.5;
for i = 2:4
    plegend = {};

    subplot(3,1,i-1)
    u1 = plot(t_1, ua1_data{i}, 'LineWidth', LW_input, 'LineStyle', '-', 'Color', 'b');
    plegend = [plegend, {lgd1}];
    hold on;

    u2 = plot(t_2, ua2_data{i}, 'LineWidth', LW_input, 'LineStyle', '-', 'Color', 'r'); % 赤色
    plegend = [plegend, {lgd2}];

    hold off;
    u2.Color = [u2.Color(1:3), 0.5];

    % ラベル設定
    ylabel(input_ylabel{i}, 'Interpreter','latex', 'FontSize', FS);
    grid on;
    set(gca, 'FontSize', FS-2);
    if t_1(1) < t_2(1), ini = t_1(1);
    else              , ini = t_2(1); end
    if t_1(end) > t_2(end), last = t_1(end);
    else                  , last = t_2(end); end
    xlim([ini last]);

    % 凡例は指定されたグラフ番号のみ表示
    if i == 2
        legend(plegend, 'FontSize', FS-4, 'Location', 'best');
    end

    % XLabelは一番下のグラフのみ表示
    if i == 4
        xlabel('Time [s]', 'FontSize', FS);
        % ylim([0.3 0.6])
    end
end

if foutput
    % 出力ファイルパスの生成 (ファイル名に '_XY' を追加)
    output_filename_input = fullfile(folder_path, "input" + "." + file_style);
    
    fig = figure(3);
    
    try
        exportgraphics(fig, output_filename_input, 'Resolution', 300);
        disp(['Input Graph successfully exported to: ', output_filename_input]);
    catch ME
        disp(['Error exporting input graph: ', ME.message]);
    end
end


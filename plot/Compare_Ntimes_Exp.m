%% Compare_Ntimes_Exp
% 同じ条件で n 回行った実験のログ（LOGGER.save で保存した .mat）を条件（グループ）ごとに読み込み，
% 試行間の平均(太線) と 平均 ± nσ(バンド) を重ね描きして条件どうしを比較するスクリプト。
%   - 時刻合わせ : 実験は試行ごとに離陸・飛行開始の時刻が異なり dt も不等間隔のため，
%                  alignPhase（既定 'f'）の開始時刻を t = 0 に揃え，全試行に共通する区間を dtResample 間隔に線形補間して統計をとる
%   - 統計比較   : 試行ごとの位置 RMSE（推定値 - 目標値）と制御コスト（∫u_i^2 dt の成分ごととトータル），条件ごとの平均・標準偏差（alignPhase 区間）
%   - 描画       : 位置・位置誤差・姿勢・速度・入力・補償入力の時系列（平均 ± nσ），xy 軌跡と法線方向の ±nσ バンド，3次元軌跡，RMSE の比較
% ※ 実験では真値（plant）が無いため，位置・姿勢などは estimator の値を使う
% ※ ログは LOGGER で読み込み，値は trial_data(trial, variable, attribute)（LOGGER.data と同じ指定）で取り出す
%
% 2026/10 作成

% パス設定 ==========================================================================================================================================
scriptPath = mfilename('fullpath');
if isempty(scriptPath) || contains(scriptPath, "LiveEditorEvaluationHelper")
    tmp = matlab.desktop.editor.getActive;
    scriptPath = tmp.Filename;
end
rootDir = fileparts(fileparts(scriptPath)); % common_matlab
cd(rootDir);
close all hidden; clear; clc;
if ~exist("STATE_CLASS", "class") % ログ内の state を復元するためにクラス定義へのパスが必要
    addpath(genpath(pwd));
end

%% 比較設定 ===========================================================================================================================================
% 1行 = 1条件 : {凡例ラベル, ファイル}
%   凡例ラベルは LaTeX で解釈されるため，_ % & # は \_ のようにエスケープする（数式は $...$）
%   ファイルは rootDir からの相対パス・絶対パスで指定し，ワイルドカード（*）や string 配列で複数指定できる
%   ファイルを "" にすると選択ダイアログで複数選択する
groups = {
    "w/o NN-MEC", "Data/tmp_exp/LemniscateT=10x4_HL*_Log*.mat";
    "w/ NN-MEC", "Data/tmp_exp/LemniscateT=10x4_NNMEC*_Log*.mat";
    };
agentIdx = 1;           % ログ内の agent 番号
alignPhase = 'f';       % このフェーズの開始時刻を t = 0 に揃える（RMSE もこのフェーズ区間で計算する）
plotPhase = 'f';        % 描画するフェーズ（'t','f','l' の組合せ。'a' は離陸時に時刻がリセットされるため指定不可）
timeRange = [10 40];         % [t0 t1] : 描画する区間（alignPhase 開始からの相対時刻 [s]）。空なら全試行に共通する区間
dtResample = 0.01;      % [s] 共通時刻の間隔（実験ログは dt が不等間隔のため，この間隔に線形補間して試行間の統計をとる）
refPeriod = 10;         % [s] 目標軌道の周期 T。xy・3次元軌跡の目標値（reference）は alignPhase 開始から1周期分だけ描く（時系列は全区間）。空なら全区間を描く
costWeight = [1 1 1 1]; % 制御コストのトータル J_total = Σ costWeight_i ∫ u_i^2 dt の重み（u = [T, tau_x, tau_y, tau_z]）
nSigma = 1;             % 平均 ± nSigma×標準偏差 をバンドで描く
yMarginRatio = 1.1;     % 時系列の縦軸の範囲 : 描画した全要素の (最小値〜最大値) の幅をこの倍率で広げる（中心はそのまま）
fPlotEachTrial = false; % true: 時系列・xy 軌跡に各試行も細線で重ねて描く
fLegend = true;         % true: 全ての図に凡例を表示する
fSaveFig = false;       % true: 図を figDir に figFormats の形式で保存する
figFormats = ["png", "pdf"]; % 保存形式（複数指定可）: "png","jpg","tif" 等のラスタ形式，"pdf","eps","emf" のベクトル形式
figDir = fullfile("Data", "compare_exp_fig");

FS = 12; % FontSize
LW = 0.5;% LineWidth
MS = 7;  % MarkerSize [pt]
MA = 0.5;% MarkerAlpha（散布図のマーカーの不透明度。1: 不透明, 0: 透明）
LA = 0.6;% LineAlpha（時系列・xy・3次元軌跡の平均線の不透明度。1: 不透明, 0: 透明）
BA = 0.2;% BandAlpha（±nσ バンドの不透明度。1: 不透明, 0: 透明）

%% 読み込み ===========================================================================================================================================
if contains(plotPhase, 'a')
    error("Compare_Ntimes_Exp: plotPhase に 'a' は指定できません（離陸時に時刻がリセットされるため）。");
end
groupN = size(groups, 1);
group = repmat(struct("label", "", "files", strings(0, 1), "trials", []), groupN, 1);
for groupIdx = 1:groupN
    group(groupIdx).label = string(groups{groupIdx, 1});
    group(groupIdx).files = find_files(string(groups{groupIdx, 2}), group(groupIdx).label);
    fprintf("[%d] %s : %d 試行\n", groupIdx, group(groupIdx).label, numel(group(groupIdx).files));
    trialList = cell(numel(group(groupIdx).files), 1);
    for fileIdx = 1:numel(group(groupIdx).files)
        fprintf("    %s\n", group(groupIdx).files(fileIdx));
        trialList{fileIdx} = load_exp_trial(group(groupIdx).files(fileIdx), agentIdx, alignPhase, plotPhase);
    end
    group(groupIdx).trials = [trialList{:}];
end
labelList = [group.label];
colors = lines(groupN);

% 全試行に共通する区間を共通時刻とする
allTrials = [group.trials];
tStart = max(arrayfun(@(trial) trial.t(1), allTrials));
tEnd = min(arrayfun(@(trial) trial.t(end), allTrials));
if ~isempty(timeRange)
    if timeRange(1) < tStart || timeRange(2) > tEnd
        warning("Compare_Ntimes_Exp: timeRange [%g %g] が全試行に共通する区間 [%.3f %.3f] を超えるため，共通区間に切り詰めます。", timeRange, tStart, tEnd);
    end
    tStart = max(tStart, timeRange(1));
    tEnd = min(tEnd, timeRange(2));
end
tGrid = (tStart:dtResample:tEnd)';
fprintf("\n共通時刻 : %.3f 〜 %.3f s（%s 開始 = 0 s, dt = %g s）\n", tStart, tEnd, alignPhase, dtResample);
% xy・3次元軌跡で目標値を描く区間（alignPhase 開始（共通時刻が後から始まる場合はその開始）から refPeriod の1周期分）
if isempty(refPeriod)
    refMask = true(size(tGrid));
else
    refStart = max(tGrid(1), 0);
    refMask = tGrid >= refStart & tGrid <= refStart + refPeriod;
end

%% 統計比較 ===========================================================================================================================================
% 試行ごとの情報，位置 RMSE（estimator - reference），制御コスト（入力の2乗の時間積分）
% いずれも共通時刻のうち alignPhase の区間で計算する
rmseNames = ["rmseX", "rmseY", "rmseZ", "rmseNorm"];
costNames = ["costT", "costTauX", "costTauY", "costTauZ", "costTotal"];
settingText = "===== 実験ログ =====" + newline;
trialTable = table();
for groupIdx = 1:groupN
    for trialIdx = 1:numel(group(groupIdx).trials)
        trial = group(groupIdx).trials(trialIdx);
        phaseGrid = interp1(trial.t, trial.phase, tGrid, "previous");
        evalMask = phaseGrid == double(alignPhase);
        % 位置 RMSE
        posError = resample_value(trial, @(trial) trial_data(trial, "p", "e") - trial_data(trial, "p", "r"), tGrid);
        rmseXYZ = sqrt(mean(posError(evalMask, :) .^ 2, 1, "omitnan"));
        % 制御コスト J_i = ∫ u_i^2 dt（u = [T, tau_x, tau_y, tau_z]），J_total = Σ costWeight_i * J_i
        inputGrid = resample_value(trial, @(trial) trial_data(trial, "input", ""), tGrid);
        costEach = trapz(tGrid(evalMask), inputGrid(evalMask, 1:4) .^ 2, 1);
        costTotal = sum(costWeight(:)' .* costEach);
        row = table(group(groupIdx).label, string(trial.file), trial.alignStartTime, trial.alignDuration, ...
                    rmseXYZ(1), rmseXYZ(2), rmseXYZ(3), sqrt(sum(rmseXYZ .^ 2)), ...
                    costEach(1), costEach(2), costEach(3), costEach(4), costTotal, ...
                    'VariableNames', ["label", "file", "alignStart", "alignDuration", rmseNames, costNames]);
        trialTable = [trialTable; row]; %#ok<AGROW>
    end
end
rmseStatTable = groupsummary(trialTable, "label", ["mean", "std"], rmseNames);
rmseStatTable = rmseStatTable(match_label_order(rmseStatTable.label, labelList), :);
costStatTable = groupsummary(trialTable, "label", ["mean", "std"], costNames);
costStatTable = costStatTable(match_label_order(costStatTable.label, labelList), :);
settingText = settingText + sprintf("alignStart : 離陸からの %s 開始時刻 [s], alignDuration : %s の継続時間 [s]\n", alignPhase, alignPhase);
settingText = settingText + sprintf("評価区間 : 共通時刻 %.3f 〜 %.3f s のうち %s の区間\n", tStart, tEnd, alignPhase);
settingText = settingText + sprintf("RMSE : estimator.p - reference.p [m]\n");
settingText = settingText + sprintf("制御コスト : J_i = ∫ u_i^2 dt（u = [T, tau_x, tau_y, tau_z]），J_total = Σ w_i J_i（w = [%s]）\n\n", strjoin(string(costWeight), ", "));
settingText = settingText + formattedDisplayText(trialTable) + newline ...
              + "===== 条件ごとの RMSE（平均・標準偏差）=====" + newline + formattedDisplayText(rmseStatTable) + newline ...
              + "===== 条件ごとの制御コスト（平均・標準偏差）=====" + newline + formattedDisplayText(costStatTable);
fprintf("\n%s", settingText);

metricOpts = struct("fLegend", fLegend, "FS", FS, "LW", LW, "MS", MS, "MA", MA);
% RMSE の比較（各試行の点と 平均 ± 1σ）
rmseLabels = ["$\mathrm{RMSE}_x$", "$\mathrm{RMSE}_y$", "$\mathrm{RMSE}_z$", "$\mathrm{RMSE}_{\mathrm{norm}}$"] + " [m]";
plot_metric_compare(200, "rmse", trialTable, rmseNames, rmseLabels, labelList, colors, metricOpts);
% 制御コストの比較（成分ごととトータル，各試行の点と 平均 ± 1σ）
costLabels = ["$J_T$ [N$^2$s]", "$J_{\tau_x}$ [N$^2$m$^2$s]", "$J_{\tau_y}$ [N$^2$m$^2$s]", "$J_{\tau_z}$ [N$^2$m$^2$s]", "$J_{\mathrm{total}}$"];
plot_metric_compare(201, "control_cost", trialTable, costNames, costLabels, labelList, colors, metricOpts);

%% 描画：時系列比較（平均 ± nσ） =====================================================================================================================
fprintf("\n時系列描画 : phase '%s', 平均 ± %gσ\n", plotPhase, nSigma);
plotOpts = struct("nSigma", nSigma, "fPlotEachTrial", fPlotEachTrial, "fLegend", fLegend, "FS", FS, "LW", LW, "LA", LA, "BA", BA, "yMarginRatio", yMarginRatio);
% 描画する値は @(trial) trial_data(trial, variable, attribute) で指定する（例 : 角速度 trial_data(trial, "w", "e")）

% 位置（推定値），目標位置
plot_compare_band(300, "position", group, colors, @(trial) trial_data(trial, "p", "e"), ["$x$", "$y$", "$z$"] + " [m]", ...
                  @(trial) trial_data(trial, "p", "r"), tGrid, plotOpts);
% 位置誤差（推定値 - 目標位置）
plot_compare_band(301, "position_error", group, colors, @(trial) trial_data(trial, "p", "e") - trial_data(trial, "p", "r"), ["$e_x$", "$e_y$", "$e_z$"] + " [m]", ...
                  [], tGrid, plotOpts);
% 姿勢（推定値）
plot_compare_band(302, "attitude", group, colors, @(trial) trial_data(trial, "q", "e"), ["$\phi$", "$\theta$", "$\psi$"] + " [rad]", ...
                  [], tGrid, plotOpts);
% 速度（推定値），目標速度
plot_compare_band(303, "velocity", group, colors, @(trial) trial_data(trial, "v", "e"), ["$v_x$", "$v_y$", "$v_z$"] + " [m/s]", ...
                  @(trial) trial_data(trial, "v", "r"), tGrid, plotOpts);
% 入力
plot_compare_band(304, "input", group, colors, @(trial) trial_data(trial, "input", ""), ["$T$ [N]", "$\tau_x$ [Nm]", "$\tau_y$ [Nm]", "$\tau_z$ [Nm]"], ...
                  [], tGrid, plotOpts);
% NN による補償入力（delta_input を持たない制御器のグループは描画しない）
plot_compare_band(305, "delta_input", group, colors, @(trial) trial_data(trial, "controller.result.delta_input", ""), ["$\Delta T$ [N]", "$\Delta\tau_x$ [Nm]", "$\Delta\tau_y$ [Nm]", "$\Delta\tau_z$ [Nm]"], ...
                  [], tGrid, plotOpts);

% xy 軌跡（平均軌跡を太線。バンドは平均軌跡の法線方向（進行方向に垂直）の偏差の ±nσ，fPlotEachTrial で各試行を細線表示）
figure(306); clf; set(gcf, "Name", "xy_trajectory"); hold on
minSpeedXY = 1e-2; % [m/s] 平均軌跡の xy 速度がこれ未満のステップ（ホバなど）は法線方向が定まらないためバンドを描かない
fRefPlotted = false;
for groupIdx = 1:groupN
    [pMean, pStack] = stack_trials(group(groupIdx).trials, @(trial) trial_data(trial, "p", "e"), tGrid);
    xyMean = pMean(:, 1:2);
    velX = gradient(xyMean(:, 1), tGrid);
    velY = gradient(xyMean(:, 2), tGrid);
    speedXY = hypot(velX, velY);
    normalXY = [-velY, velX] ./ speedXY;
    normalXY(speedXY < minSpeedXY, :) = NaN;
    crossDev = reshape(sum((pStack(:, 1:2, :) - xyMean) .* normalXY, 2), size(pStack, 1), []); % [ステップ × 試行] 法線方向の偏差
    crossStd = std(crossDev, 0, 2, "omitnan");
    upperXY = xyMean + nSigma * crossStd .* normalXY;
    lowerXY = xyMean - nSigma * crossStd .* normalXY;
    % 自己交差する軌跡（8の字など）でも崩れないよう，隣接ステップ間の四角形を並べたパッチで描く（NaN を含む面は描画されない）
    stepN = size(xyMean, 1);
    bandFaces = [(1:stepN - 1)', (2:stepN)', stepN + (2:stepN)', stepN + (1:stepN - 1)'];
    patch("Faces", bandFaces, "Vertices", [upperXY; lowerXY], "FaceColor", colors(groupIdx, :), "FaceAlpha", BA, "EdgeColor", "none", ...
          "DisplayName", labelList(groupIdx) + " $\pm" + nSigma + "\sigma$");
    if fPlotEachTrial
        plot(squeeze(pStack(:, 1, :)), squeeze(pStack(:, 2, :)), "Color", [colors(groupIdx, :), 0.2], "LineWidth", 0.5, "HandleVisibility", "off");
    end
    plot(pMean(:, 1), pMean(:, 2), "Color", [colors(groupIdx, :), LA], "LineWidth", 2 * LW, "DisplayName", labelList(groupIdx) + " mean");
    if ~fRefPlotted
        refMean = stack_trials(group(groupIdx).trials, @(trial) trial_data(trial, "p", "r"), tGrid);
        plot(refMean(refMask, 1), refMean(refMask, 2), "k--", "LineWidth", LW, "DisplayName", "reference");
        fRefPlotted = true;
    end
end
xlabel("$x$ [m]", "Interpreter", "latex"); ylabel("$y$ [m]", "Interpreter", "latex"); axis equal; grid on; grid minor
if fLegend; legend("Interpreter", "latex", "Location", "best", "Orientation", "horizontal", "NumColumns", 3); end % 2行3列
set(gca, "FontSize", FS);

% 3次元軌跡（平均軌跡を太線，fPlotEachTrial で各試行を細線表示）
figure(307); clf; set(gcf, "Name", "xyz_trajectory"); hold on
fRefPlotted = false;
for groupIdx = 1:groupN
    [pMean, pStack] = stack_trials(group(groupIdx).trials, @(trial) trial_data(trial, "p", "e"), tGrid);
    if fPlotEachTrial
        plot3(squeeze(pStack(:, 1, :)), squeeze(pStack(:, 2, :)), squeeze(pStack(:, 3, :)), "Color", [colors(groupIdx, :), 0.2], "LineWidth", 0.5, "HandleVisibility", "off");
    end
    plot3(pMean(:, 1), pMean(:, 2), pMean(:, 3), "Color", [colors(groupIdx, :), LA], "LineWidth", 2 * LW, "DisplayName", labelList(groupIdx) + " mean");
    if ~fRefPlotted
        refMean = stack_trials(group(groupIdx).trials, @(trial) trial_data(trial, "p", "r"), tGrid);
        plot3(refMean(refMask, 1), refMean(refMask, 2), refMean(refMask, 3), "k--", "LineWidth", LW, "DisplayName", "reference");
        fRefPlotted = true;
    end
end
xlabel("$x$ [m]", "Interpreter", "latex"); ylabel("$y$ [m]", "Interpreter", "latex"); zlabel("$z$ [m]", "Interpreter", "latex"); axis equal; grid on; grid minor; view(3)
if fLegend; legend(legend_items_ref_first(gca), "Interpreter", "latex", "Location", "best", "Orientation", "horizontal", "NumColumns", 3); end % 2行3列
set(gca, "FontSize", FS);

%% 図の保存 ===========================================================================================================================================
if fSaveFig
    if ~exist(figDir, "dir"); mkdir(figDir); end
    vectorFormats = ["pdf", "eps", "emf"];
    for figHandle = findobj("Type", "figure")'
        % ファイル名 : fig<番号>_<Figure の Name>（Name が無い図は番号のみ）
        figBaseName = "fig" + figHandle.Number;
        if strlength(figHandle.Name) > 0
            figBaseName = figBaseName + "_" + regexprep(string(figHandle.Name), '[^\w\-]', '_');
        end
        for figFormat = lower(figFormats)
            figPath = fullfile(figDir, figBaseName + "." + figFormat);
            if ismember(figFormat, vectorFormats)
                exportgraphics(figHandle, figPath, "ContentType", "vector");
            else
                exportgraphics(figHandle, figPath, "Resolution", 300);
            end
        end
    end
    % 使用したログと RMSE の表も txt で保存する
    fileId = fopen(fullfile(figDir, "exp_summary.txt"), "w", "n", "UTF-8");
    fprintf(fileId, "%s", settingText);
    fclose(fileId);
    fprintf("図・ログ一覧(exp_summary.txt)の保存先 : %s\n", figDir);
end


%% ローカル関数 =======================================================================================================================================
function files = find_files(patterns, label)
% patterns（ワイルドカード可）に一致するファイルを返す。"" なら選択ダイアログで選ぶ
if all(strlength(patterns) == 0)
    files = strings(0, 1);
    startDir = "Data";
    while true
        dialogTitle = sprintf("%s の実験ログを選択（複数選択可，%d 件選択済み，キャンセルで選択終了）", label, numel(files));
        [selectedFiles, selectedDir] = uigetfile(fullfile(startDir, "*.mat"), dialogTitle, "MultiSelect", "on");
        if isequal(selectedFiles, 0); break; end
        selectedFiles = string(selectedFiles);
        files = [files; fullfile(string(selectedDir), selectedFiles(:))]; %#ok<AGROW>
        startDir = selectedDir;
    end
else
    files = strings(0, 1);
    for pattern = patterns(:)'
        found = dir(pattern);
        files = [files; fullfile(string({found.folder}'), string({found.name}'))]; %#ok<AGROW>
    end
end
files = unique(files, "stable");
if isempty(files)
    error("Compare_Ntimes_Exp: %s のファイルが見つかりません（%s）。", label, strjoin(patterns, ", "));
end
end

function trial = load_exp_trial(filePath, agentIdx, alignPhase, plotPhase)
% LOGGER.save で保存した実験ログを LOGGER として読み込み，plotPhase の区間の時刻とフェーズを取り出す
% trial.t は alignPhase の開始時刻を 0 とした相対時刻。値は trial_data(trial, variable, attribute) で取り出す
logger = LOGGER(char(filePath));
tAlign = logger.data("t", "", "", "phase", alignPhase);
if isempty(tAlign)
    error("Compare_Ntimes_Exp: %s に phase '%s' がありません。", filePath, alignPhase);
end
t = logger.data("t", "", "", "phase", plotPhase);
if any(diff(t) < 0)
    error("Compare_Ntimes_Exp: %s の plotPhase '%s' の区間で時刻が単調増加していません。", filePath, plotPhase);
end

[~, trial.file] = fileparts(filePath);
trial.logger = logger;
trial.agentIdx = agentIdx;
trial.plotPhase = plotPhase;
trial.alignStartTime = tAlign(1);
trial.alignDuration = tAlign(end) - tAlign(1);
trial.t = t - tAlign(1);
trial.phase = logger.data(0, "phase", "", "phase", plotPhase); % 'a','t','f','l' の文字コード
end

function value = trial_data(trial, variable, attribute)
% LOGGER.data で plotPhase の区間の値を取り出す（trial.t と同じ長さ）
% 例 : trial_data(trial, "p", "e"), trial_data(trial, "input", ""), trial_data(trial, "controller.result.delta_input", "")
value = trial.logger.data(trial.agentIdx, variable, attribute, "phase", trial.plotPhase);
end

function value = resample_value(trial, getValue, tGrid)
% getValue(trial) を共通時刻 tGrid に線形補間する（時刻が重複するステップは最初の値を使う）
[tUnique, uniqueIdx] = unique(trial.t, "stable");
value = getValue(trial);
value = interp1(tUnique, value(uniqueIdx, :), tGrid, "linear");
end

function [valueMean, valueStack] = stack_trials(trials, getValue, tGrid)
% 各試行の getValue(trial) を共通時刻に補間し，[ステップ × 列 × 試行] に積んで試行方向の平均を返す
valueStack = arrayfun(@(trial) resample_value(trial, getValue, tGrid), trials, "UniformOutput", false);
valueStack = cat(3, valueStack{:});
valueMean = mean(valueStack, 3, "omitnan");
end

function plot_compare_band(figNum, figName, group, colors, getValue, yLabels, getRef, tGrid, opts)
% 各グループの全試行について getValue(trial) の各列の 平均(太線) と 平均 ± nσ(バンド) を時系列で重ね描きする
% getRef があれば1つ目に描画できたグループの目標値の平均を黒破線で描く
figure(figNum); clf; set(gcf, "Name", figName);
tiledlayout(numel(yLabels), 1, "TileSpacing", "compact");
axList = gobjects(numel(yLabels), 1);
for colIdx = 1:numel(yLabels)
    axList(colIdx) = nexttile; hold on
end
fRefPlotted = false;
for groupIdx = 1:numel(group)
    try
        [valueMean, valueStack] = stack_trials(group(groupIdx).trials, getValue, tGrid);
    catch err
        fprintf("Figure %d : %s は描画しません（%s）\n", figNum, group(groupIdx).label, err.message);
        continue
    end
    valueStd = std(valueStack, 0, 3, "omitnan");
    color = colors(groupIdx, :);
    for colIdx = 1:min(numel(yLabels), size(valueMean, 2))
        ax = axList(colIdx);
        upper = valueMean(:, colIdx) + opts.nSigma * valueStd(:, colIdx);
        lower = valueMean(:, colIdx) - opts.nSigma * valueStd(:, colIdx);
        fill(ax, [tGrid; flipud(tGrid)], [upper; flipud(lower)], color, "FaceAlpha", opts.BA, "EdgeColor", "none", ...
             "DisplayName", group(groupIdx).label + " $\pm" + opts.nSigma + "\sigma$");
        if opts.fPlotEachTrial
            plot(ax, tGrid, squeeze(valueStack(:, colIdx, :)), "Color", [color, 0.2], "LineWidth", 0.5, "HandleVisibility", "off");
        end
        plot(ax, tGrid, valueMean(:, colIdx), "Color", [color, opts.LA], "LineWidth", 2 * opts.LW, "DisplayName", group(groupIdx).label + " mean");
    end
    if ~isempty(getRef) && ~fRefPlotted
        refMean = stack_trials(group(groupIdx).trials, getRef, tGrid);
        for colIdx = 1:min(numel(yLabels), size(refMean, 2))
            plot(axList(colIdx), tGrid, refMean(:, colIdx), "k--", "LineWidth", opts.LW, "DisplayName", "reference");
        end
        fRefPlotted = true;
    end
end
for colIdx = 1:numel(yLabels)
    ylabel(axList(colIdx), yLabels(colIdx), "Interpreter", "latex"); grid(axList(colIdx), "on"); grid(axList(colIdx), "minor");
    set(axList(colIdx), "FontSize", opts.FS);
    set_ylim_from_data(axList(colIdx), opts.yMarginRatio);
end
xlabel(axList(end), "time [s]", "Interpreter", "latex");
if opts.fLegend
    % 凡例は tiledlayout の外側上部（northoutside 相当）に置く。特定の軸の中に置かないため各軸の縦幅は同じになる
    timeLegend = legend(axList(1), legend_items_ref_first(axList(1)), "Interpreter", "latex", "Orientation", "horizontal");
    timeLegend.Layout.Tile = "north";
end
end

function plot_metric_compare(figNum, figName, trialTable, metricNames, metricLabels, labelList, colors, opts)
% 試行ごとの評価値（trialTable の metricNames 列）を条件ごとに 各試行の点 と 平均 ± 1σ で並べて描く
figure(figNum); clf; set(gcf, "Name", figName);
tiledlayout(1, numel(metricNames));
for metricIdx = 1:numel(metricNames)
    nexttile; hold on
    for groupIdx = 1:numel(labelList)
        values = trialTable.(metricNames(metricIdx))(trialTable.label == labelList(groupIdx));
        jitter = 0.15 * (linspace(0, 1, numel(values))' - 0.5); % 点の重なりを避けるため横にずらす
        scatter(groupIdx + jitter, values, opts.MS^2, colors(groupIdx, :), "filled", "MarkerFaceAlpha", opts.MA, "MarkerEdgeAlpha", opts.MA, "HandleVisibility", "off");
        errorbar(groupIdx + 0.3, mean(values), std(values), "o", "Color", colors(groupIdx, :), "MarkerFaceColor", colors(groupIdx, :), ...
                 "LineWidth", 1.5 * opts.LW, "CapSize", 10, "DisplayName", labelList(groupIdx) + " mean $\pm 1\sigma$");
    end
    xlim([0.5, numel(labelList) + 0.8]); xticks(1:numel(labelList)); xticklabels(labelList);
    ylabel(metricLabels(metricIdx), "Interpreter", "latex"); grid on; grid minor
    set(gca, "FontSize", opts.FS, "TickLabelInterpreter", "latex");
end
if opts.fLegend
    metricLegend = legend("Interpreter", "latex", "Orientation", "horizontal");
    metricLegend.Layout.Tile = "north";
end
end

function items = legend_items_ref_first(ax)
% 凡例に載せる要素を描画順に並べ，"reference" を先頭に移す（描画の重なり順は変えない）
items = flipud(ax.Children);
items = items(arrayfun(@(item) isprop(item, "DisplayName") && strcmp(item.HandleVisibility, "on"), items));
fRef = arrayfun(@(item) item.DisplayName == "reference", items);
items = [items(fRef); items(~fRef)];
end

function set_ylim_from_data(ax, marginRatio)
% 軸に描いた全要素（線・バンド）の最小値〜最大値の幅を marginRatio 倍した範囲を縦軸の範囲にする
yValues = [];
for child = [findobj(ax, "Type", "line"); findobj(ax, "Type", "patch")]'
    yValues = [yValues; child.YData(:)]; %#ok<AGROW>
end
yMin = min(yValues, [], "omitnan");
yMax = max(yValues, [], "omitnan");
if isempty(yMin); return; end
yCenter = (yMax + yMin) / 2;
yHalf = (yMax - yMin) / 2 * marginRatio;
if yHalf == 0 % 一定値のみの場合
    yHalf = max(abs(yCenter), 1) * (marginRatio - 1);
end
ylim(ax, [yCenter - yHalf, yCenter + yHalf]);
end

function order = match_label_order(labels, labelList)
% groupsummary でソートされた行を groups の順に並べ直す
[~, order] = ismember(labelList, labels);
order = order(order > 0);
end

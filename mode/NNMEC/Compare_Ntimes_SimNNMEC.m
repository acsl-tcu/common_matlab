%% Compare_Ntimes_SimNNMEC
% Ntimes_SimNNMEC.m で保存した結果ファイル(.mat)を複数読み込み，プロット・データ比較するスクリプト。
%   - 統計比較 : ファイルごとの失敗数，flight フェーズ RMSE の平均・標準偏差・最大
%   - 対応比較 : seed と サンプル値が同じファイル同士は，同じモデル誤差の試行どうしで RMSE 差を比較
%   - 描画     : RMSE の箱ひげ図，パラメータと RMSE の散布図，ファイル間 RMSE の散布図，
%                全試行の時系列(位置・位置誤差・姿勢・入力・補償入力)の平均(太線)と ±nσ バンド，平均 xy 軌跡
%
% 2026/10 作成

% パス設定 ==========================================================================================================================================
scriptPath = mfilename('fullpath');
if isempty(scriptPath) || contains(scriptPath, "LiveEditorEvaluationHelper")
    tmp = matlab.desktop.editor.getActive;
    scriptPath = tmp.Filename;
end
rootDir = fileparts(fileparts(fileparts(scriptPath))); % common_matlab
cd(rootDir);
close all hidden; clear; clc;

%% 比較設定 ===========================================================================================================================================
dataDir = fullfile("Data", "NNMEC_MonteCarlo");
% 比較するファイル（dataDir からの相対パス）。空なら選択ダイアログで複数選択する
fileNames = [
    % "2026-10-9_15_26_43__Ntimes_SimNNMEC__trial10.mat"
    % "2026-10-9_16_0_0__Ntimes_SimNNMEC__trial10.mat"
    "2026-10-9_16_14_57__Ntimes_SimHL__trial100.mat";
    "2026-10-9_17_56_54__Ntimes_SimNNMEC__trial100.mat";
    ];
% 凡例ラベル（fileNames と同じ数）。空ならファイルの ptName（無ければファイル名）を使う
labels = [
    "w/o HL";
    "w/ NN-MEC";
    ];
nSigma = 2;             % 時系列の描画で 平均 ± nSigma×標準偏差 をバンドで描く
fExcludeFailed = true;  % true: 時系列の統計から発散/墜落した試行を除く（途中で打ち切られて長さが揃わないため）
fPlotEachTrial = false; % true: 時系列・xy 軌跡に各試行も細線で重ねて描く
plotPhase = 'tf';       % 時系列を描画するフェーズ（'a','t','f','l' の組合せ）
fSaveFig = false;       % true: 図を figDir に png で保存する
figDir = fullfile(dataDir, "compare_fig");

FS = 14; % FontSize
LW = 1.5;% LineWidth

%% 読み込み ===========================================================================================================================================
if isempty(fileNames)
    [selectedFiles, selectedDir] = uigetfile(fullfile(dataDir, "*.mat"), "比較する結果ファイルを選択（複数選択可）", "MultiSelect", "on");
    if isequal(selectedFiles, 0)
        error("Compare_Ntimes_SimNNMEC: ファイルが選択されていません。");
    end
    filePaths = fullfile(string(selectedDir), string(selectedFiles));
else
    filePaths = fullfile(dataDir, string(fileNames));
end
filePaths = filePaths(:);
fileN = numel(filePaths);

data = repmat(struct("file", "", "label", "", "results", [], "summaryTable", [], "sampledParam", [], ...
                     "paramNames", [], "seed", [], "simInfo", struct()), fileN, 1);
for fileIdx = 1:fileN
    loaded = load(filePaths(fileIdx));
    [~, data(fileIdx).file] = fileparts(filePaths(fileIdx));
    data(fileIdx).results = loaded.results;
    data(fileIdx).summaryTable = loaded.summaryTable;
    data(fileIdx).sampledParam = loaded.sampledParam;
    data(fileIdx).paramNames = [loaded.errorParam.name];
    data(fileIdx).seed = loaded.seed;
    if isfield(loaded, "simInfo"); data(fileIdx).simInfo = loaded.simInfo; end

    if numel(labels) >= fileIdx
        data(fileIdx).label = string(labels(fileIdx));
    elseif isfield(data(fileIdx).simInfo, "ptName") && strlength(data(fileIdx).simInfo.ptName) > 0
        data(fileIdx).label = data(fileIdx).simInfo.ptName;
    else
        data(fileIdx).label = data(fileIdx).file;
    end
    fprintf("[%d] %s\n    label : %s\n", fileIdx, filePaths(fileIdx), data(fileIdx).label);
    if isfield(data(fileIdx).simInfo, "controller")
        fprintf("    controller : %s\n", data(fileIdx).simInfo.controller);
    end
end
if numel(labels) < fileN
    % ラベル未指定の場合：日付を除いた "__" 区切りの要素のうち，全ファイル共通の要素を除いて短いラベルにする
    labelTokens = arrayfun(@(dataItem) split(regexprep(erase(dataItem.label, [".pt", "_model"]), "^[\d_-]+__", ""), "__")', data, "UniformOutput", false);
    commonTokens = labelTokens{1};
    for fileIdx = 2:fileN
        commonTokens = intersect(commonTokens, labelTokens{fileIdx}, "stable");
    end
    for fileIdx = 1:fileN
        shortLabel = strjoin(setdiff(labelTokens{fileIdx}, commonTokens, "stable"), " ");
        data(fileIdx).label = strtrim("[" + fileIdx + "] " + shortLabel);
    end
end
labelList = [data.label];
fprintf("\n凡例ラベル : %s\n", strjoin(labelList, "  |  "));
colors = lines(fileN);

%% 統計比較 ===========================================================================================================================================
statTable = table('Size', [fileN, 9], ...
    'VariableTypes', ["string", "double", "double", "double", "double", "double", "double", "double", "double"], ...
    'VariableNames', ["label", "trialN", "failN", "rmseX_mean", "rmseY_mean", "rmseZ_mean", "rmseNorm_mean", "rmseNorm_std", "rmseNorm_max"]);
for fileIdx = 1:fileN
    st = data(fileIdx).summaryTable;
    validRow = ~st.fDiverged;
    statTable.label(fileIdx) = data(fileIdx).label;
    statTable.trialN(fileIdx) = height(st);
    statTable.failN(fileIdx) = sum(~validRow);
    statTable.rmseX_mean(fileIdx) = mean(st.rmseX(validRow));
    statTable.rmseY_mean(fileIdx) = mean(st.rmseY(validRow));
    statTable.rmseZ_mean(fileIdx) = mean(st.rmseZ(validRow));
    statTable.rmseNorm_mean(fileIdx) = mean(st.rmseNorm(validRow));
    statTable.rmseNorm_std(fileIdx) = std(st.rmseNorm(validRow));
    statTable.rmseNorm_max(fileIdx) = max(st.rmseNorm(validRow));
end
fprintf("\n===== 統計比較（RMSE は完走した試行のみ, flight フェーズ）=====\n");
disp(statTable);

% 対応比較：1つ目のファイルを基準に，同じ seed・同じサンプル値のファイルと試行ごとに比較
fPaired = false(fileN, 1);
for fileIdx = 2:fileN
    fPaired(fileIdx) = isequal(data(fileIdx).seed, data(1).seed) && isequal(data(fileIdx).sampledParam, data(1).sampledParam);
    if ~fPaired(fileIdx)
        fprintf("[%d] は [1] とサンプル値が異なるため，試行ごとの対応比較は行いません。\n", fileIdx);
        continue
    end
    stBase = data(1).summaryTable;
    st = data(fileIdx).summaryTable;
    pairTable = stBase(:, cellstr(data(1).paramNames));
    pairTable.("fail_" + 1) = stBase.fDiverged;
    pairTable.("fail_" + fileIdx) = st.fDiverged;
    pairTable.("rmseNorm_" + 1) = stBase.rmseNorm;
    pairTable.("rmseNorm_" + fileIdx) = st.rmseNorm;
    pairTable.diff = st.rmseNorm - stBase.rmseNorm;
    fprintf("\n===== 対応比較 [%d] - [1] (diff < 0 なら [%d] の方が誤差が小さい) =====\n", fileIdx, fileIdx);
    disp(pairTable);
    bothValid = ~stBase.fDiverged & ~st.fDiverged;
    fprintf("両方完走 : %d 試行  |  [%d] の方が RMSE が小さい : %d 試行  |  diff 平均 = %.4f m\n", ...
            sum(bothValid), fileIdx, sum(pairTable.diff(bothValid) < 0), mean(pairTable.diff(bothValid)));
end

%% 描画：試行全体の比較 ===============================================================================================================================
% RMSE の箱ひげ図（x, y, z, norm）
figure(200); clf;
rmseNames = ["rmseX", "rmseY", "rmseZ", "rmseNorm"];
tiledlayout(1, numel(rmseNames));
for rmseIdx = 1:numel(rmseNames)
    nexttile;
    values = [];
    groups = [];
    for fileIdx = 1:fileN
        st = data(fileIdx).summaryTable;
        validValue = st.(rmseNames(rmseIdx))(~st.fDiverged);
        values = [values; validValue]; %#ok<AGROW>
        groups = [groups; repmat(fileIdx, numel(validValue), 1)]; %#ok<AGROW>
    end
    boxHandle = boxchart(categorical(groups, 1:fileN, cellstr(labelList)), values);
    ylabel(rmseNames(rmseIdx) + " [m]"); grid on
    set(gca, "FontSize", FS, "TickLabelInterpreter", "none");
end
% 箱ひげ図の見方の凡例（凡例表示用のダミー描画。x 軸が categorical のため x も categorical の欠損値にする）
hold on
boxColor = boxHandle.BoxFaceColor;
dummyX = categorical(missing);
legendHandle = [ ...
    plot(dummyX, NaN, "s", "MarkerSize", 12, "MarkerFaceColor", boxColor * 0.4 + 0.6, "MarkerEdgeColor", boxColor, "DisplayName", "箱 : 25〜75 パーセンタイル"), ...
    plot(dummyX, NaN, "-", "Color", boxColor, "LineWidth", 2, "DisplayName", "箱内の線 : 中央値"), ...
    plot(dummyX, NaN, "-", "Color", boxColor, "DisplayName", "ひげ : 外れ値を除く最小〜最大"), ...
    plot(dummyX, NaN, "o", "Color", boxColor, "DisplayName", "○ : 外れ値（箱から 1.5×IQR 超）")];
boxLegend = legend(legendHandle, "Location", "southoutside", "NumColumns", 2);
boxLegend.Layout.Tile = "south";

% パラメータと RMSE(ノルム) の散布図（x: 失敗試行）
figure(201); clf;
paramN = numel(data(1).paramNames);
tiledlayout(1, paramN);
for paramIdx = 1:paramN
    nexttile; hold on
    for fileIdx = 1:fileN
        st = data(fileIdx).summaryTable;
        paramValue = data(fileIdx).sampledParam(:, paramIdx);
        scatter(paramValue(~st.fDiverged), st.rmseNorm(~st.fDiverged), 40, colors(fileIdx, :), "filled", "DisplayName", labelList(fileIdx));
        scatter(paramValue(st.fDiverged), zeros(sum(st.fDiverged), 1), 60, colors(fileIdx, :), "x", "LineWidth", LW, "HandleVisibility", "off");
    end
    xlabel(data(1).paramNames(paramIdx)); ylabel("RMSE norm [m]"); grid on
    set(gca, "FontSize", FS);
end
legend("Interpreter", "none", "Location", "best");

% ファイル間 RMSE の散布図（対応比較できる場合。y=x より下なら縦軸側の方が誤差が小さい）
for fileIdx = find(fPaired)'
    figure(202 + fileIdx); clf; hold on
    stBase = data(1).summaryTable;
    st = data(fileIdx).summaryTable;
    bothValid = ~stBase.fDiverged & ~st.fDiverged;
    scatter(stBase.rmseNorm(bothValid), st.rmseNorm(bothValid), 50, "filled");
    rmseMax = max([stBase.rmseNorm(bothValid); st.rmseNorm(bothValid)]);
    plot([0 rmseMax], [0 rmseMax], "k--", "LineWidth", LW);
    xlabel("RMSE norm [m] : " + labelList(1), "Interpreter", "none");
    ylabel("RMSE norm [m] : " + labelList(fileIdx), "Interpreter", "none");
    axis equal; grid on
    set(gca, "FontSize", FS);
end

%% 描画：全試行の時系列比較（平均 ± nσ） ============================================================================================================
fprintf("\n時系列描画 : phase '%s', 平均 ± %gσ\n", plotPhase, nSigma);
logList = cell(fileN, 1); % logList{fileIdx} : 統計に使う試行の log の cell 配列
for fileIdx = 1:fileN
    res = data(fileIdx).results;
    if ~isfield(res, "log")
        warning("Compare_Ntimes_SimNNMEC: [%d] にログがありません（log 保存前のファイル）。", fileIdx);
        continue
    end
    useTrial = ~arrayfun(@(resItem) isempty(resItem.log), res);
    if fExcludeFailed
        useTrial = useTrial & ~[res.fDiverged]';
    end
    logList{fileIdx} = {res(useTrial).log};
    fprintf("[%d] %s : %d / %d 試行を使用\n", fileIdx, labelList(fileIdx), sum(useTrial), numel(res));
end
plotOpts = struct("plotPhase", plotPhase, "nSigma", nSigma, "fPlotEachTrial", fPlotEachTrial, "FS", FS, "LW", LW);

% 位置（プラント真値），目標位置
plot_compare_band(300, logList, labelList, colors, @(trialLog) trialLog.plant.state.p, ["x", "y", "z"] + " [m]", ...
                  @(trialLog) trialLog.reference.state.p, plotOpts);
% 位置誤差（プラント真値 - 目標位置）
plot_compare_band(301, logList, labelList, colors, @(trialLog) trialLog.plant.state.p - trialLog.reference.state.p, ["e_x", "e_y", "e_z"] + " [m]", ...
                  [], plotOpts);
% 姿勢（プラント真値）
plot_compare_band(302, logList, labelList, colors, @(trialLog) trialLog.plant.state.q, ["roll", "pitch", "yaw"] + " [rad]", ...
                  [], plotOpts);
% 入力
plot_compare_band(303, logList, labelList, colors, @(trialLog) trialLog.input, ["T [N]", "\tau_x [Nm]", "\tau_y [Nm]", "\tau_z [Nm]"], ...
                  [], plotOpts);
% NN による補償入力（delta_input を持たない制御器のファイルは描画しない）
plot_compare_band(304, logList, labelList, colors, @(trialLog) trialLog.controller.delta_input, ["\DeltaT [N]", "\Delta\tau_x [Nm]", "\Delta\tau_y [Nm]", "\Delta\tau_z [Nm]"], ...
                  [], plotOpts);

% xy 軌跡（平均軌跡を太線。2次元のためバンドは描かず，fPlotEachTrial で各試行を細線表示）
figure(305); clf; hold on
fRefPlotted = false;
for fileIdx = 1:fileN
    if isempty(logList{fileIdx}); continue; end
    [~, pMean, pStack] = stack_trials(logList{fileIdx}, @(trialLog) trialLog.plant.state.p, plotPhase);
    if fPlotEachTrial
        plot(squeeze(pStack(:, 1, :)), squeeze(pStack(:, 2, :)), "Color", [colors(fileIdx, :), 0.2], "LineWidth", 0.5, "HandleVisibility", "off");
    end
    plot(pMean(:, 1), pMean(:, 2), "Color", colors(fileIdx, :), "LineWidth", 2 * LW, "DisplayName", labelList(fileIdx) + " mean");
    if ~fRefPlotted
        [~, refMean] = stack_trials(logList{fileIdx}, @(trialLog) trialLog.reference.state.p, plotPhase);
        plot(refMean(:, 1), refMean(:, 2), "k--", "LineWidth", LW, "DisplayName", "reference");
        fRefPlotted = true;
    end
end
xlabel("x [m]"); ylabel("y [m]"); axis equal; grid on
legend("Interpreter", "none", "Location", "best");
set(gca, "FontSize", FS);

%% 図の保存 ===========================================================================================================================================
if fSaveFig
    if ~exist(figDir, "dir"); mkdir(figDir); end
    for figHandle = findobj("Type", "figure")'
        exportgraphics(figHandle, fullfile(figDir, "fig" + figHandle.Number + ".png"));
    end
    fprintf("図の保存先 : %s\n", figDir);
end


%% ローカル関数 =======================================================================================================================================
function plot_compare_band(figNum, logList, labelList, colors, getValue, yLabels, getRef, opts)
% 各ファイルの全試行について getValue(trialLog) の各列の 平均(太線) と 平均 ± nσ(バンド) を時系列で重ね描きする
% getRef があれば1つ目に描画できたファイルの目標値の平均を黒破線で描く
figure(figNum); clf;
tiledlayout(numel(yLabels), 1, "TileSpacing", "compact");
axList = gobjects(numel(yLabels), 1);
for colIdx = 1:numel(yLabels)
    axList(colIdx) = nexttile; hold on
end
fRefPlotted = false;
for fileIdx = 1:numel(logList)
    if isempty(logList{fileIdx}); continue; end
    try
        [t, valueMean, valueStack] = stack_trials(logList{fileIdx}, getValue, opts.plotPhase);
    catch err
        fprintf("Figure %d : %s は描画しません（%s）\n", figNum, labelList(fileIdx), err.message);
        continue
    end
    valueStd = std(valueStack, 0, 3, "omitnan");
    color = colors(fileIdx, :);
    for colIdx = 1:min(numel(yLabels), size(valueMean, 2))
        ax = axList(colIdx);
        upper = valueMean(:, colIdx) + opts.nSigma * valueStd(:, colIdx);
        lower = valueMean(:, colIdx) - opts.nSigma * valueStd(:, colIdx);
        fill(ax, [t; flipud(t)], [upper; flipud(lower)], color, "FaceAlpha", 0.2, "EdgeColor", "none", ...
             "DisplayName", labelList(fileIdx) + " ±" + opts.nSigma + "\sigma");
        if opts.fPlotEachTrial
            plot(ax, t, squeeze(valueStack(:, colIdx, :)), "Color", [color, 0.2], "LineWidth", 0.5, "HandleVisibility", "off");
        end
        plot(ax, t, valueMean(:, colIdx), "Color", color, "LineWidth", 2 * opts.LW, "DisplayName", labelList(fileIdx) + " mean");
    end
    if ~isempty(getRef) && ~fRefPlotted
        [tRef, refMean] = stack_trials(logList{fileIdx}, getRef, opts.plotPhase);
        for colIdx = 1:min(numel(yLabels), size(refMean, 2))
            plot(axList(colIdx), tRef, refMean(:, colIdx), "k--", "LineWidth", opts.LW, "DisplayName", "reference");
        end
        fRefPlotted = true;
    end
end
for colIdx = 1:numel(yLabels)
    ylabel(axList(colIdx), yLabels(colIdx)); grid(axList(colIdx), "on");
    set(axList(colIdx), "FontSize", opts.FS);
end
xlabel(axList(end), "time [s]");
legend(axList(1), "Interpreter", "tex", "Location", "best");
end

function [t, valueMean, valueStack] = stack_trials(trialLogs, getValue, plotPhase)
% 各試行の getValue(trialLog) を指定フェーズで切り出し，[ステップ × 列 × 試行] に積んで試行方向の平均を返す
% 試行ごとに長さが異なる場合は最短の長さに揃える（同じ dt・フェーズ切替時刻なら完走した試行は同じ長さ）
trialN = numel(trialLogs);
valueList = cell(trialN, 1);
timeList = cell(trialN, 1);
for trialIdx = 1:trialN
    trialLog = trialLogs{trialIdx};
    ids = ismember(trialLog.phase, plotPhase);
    value = getValue(trialLog);
    valueList{trialIdx} = value(ids, :);
    timeList{trialIdx} = trialLog.t(ids);
end
stepN = min(cellfun(@(value) size(value, 1), valueList));
t = timeList{1}(1:stepN);
valueStack = cellfun(@(value) value(1:stepN, :), valueList, "UniformOutput", false);
valueStack = cat(3, valueStack{:});
valueMean = mean(valueStack, 3, "omitnan");
end

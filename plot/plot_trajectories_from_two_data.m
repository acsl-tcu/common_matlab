%% A・Bの軌道と目標軌道を重ねて表示するスクリプト
% このファイルはfunctionではなく、単体で実行できるスクリプトです。
% 実行ボタンを押すとA、Bの順にMATファイルを選択します。

clear;
close all;
clc;

%% MATファイルを指定
% ファイルパスを直接書く場合は、次の2行の "" 内に記入してください。
% 例: fileA = "C:\data\A.mat";
% 空欄のままなら、実行時にファイル選択画面が開きます。
fileA = "";
fileB = "";

if strlength(fileA) == 0
    [nameA, folderA] = uigetfile('*.mat', 'AのMATファイルを選択してください');
    if isequal(nameA, 0)
        error('AのMATファイルが選択されませんでした。');
    end
    fileA = string(fullfile(folderA, nameA));
end

if strlength(fileB) == 0
    [nameB, folderB] = uigetfile('*.mat', 'BのMATファイルを選択してください');
    if isequal(nameB, 0)
        error('BのMATファイルが選択されませんでした。');
    end
    fileB = string(fullfile(folderB, nameB));
end

%% A、Bをloadして全ログのデータを取り出す
matFiles = {char(fileA), char(fileB)};
trajectoryData = cell(1, 2);

for dataNumber = 1:2
    matFile = matFiles{dataNumber};
    if ~isfile(matFile)
        error('ファイルが見つかりません: %s', matFile);
    end

    % このスクリプト内でMATファイルをloadする。
    loadedData = load(matFile);

    if ~isfield(loadedData, 'log') || ...
            ~isfield(loadedData.log, 'Data') || ...
            ~isfield(loadedData.log.Data, 'display') || ...
            ~isfield(loadedData.log.Data, 't') || ...
            ~isfield(loadedData.log.Data, 'agent') || ...
            ~isfield(loadedData.log.Data.agent, 'sensor') || ...
            ~isfield(loadedData.log.Data.agent.sensor, 'result')
        error(['必要なlog.Data.display、log.Data.t、' ...
               'sensor.resultのいずれかがありません: %s'], matFile);
    end

    displayData = loadedData.log.Data.display;
    timeData = loadedData.log.Data.t(:);
    sensorResultData = loadedData.log.Data.agent.sensor.result;

    if ~isnumeric(timeData)
        error('log.Data.tが数値配列ではありません: %s', matFile);
    end

    numberOfLogs = min([numel(displayData), numel(timeData), ...
                        numel(sensorResultData)]);

    if isfield(loadedData.log, 'k') && ...
            isnumeric(loadedData.log.k) && ...
            isscalar(loadedData.log.k) && loadedData.log.k > 0
        numberOfLogs = min(numberOfLogs, floor(loadedData.log.k));
    end

    % t == 0はログ初期化時に作られたダミー行なので除外する。
    % 有効な時刻を持つ全ログを対象にする。
    selectedLogs = find(timeData(1:numberOfLogs) > 0 & ...
                        isfinite(timeData(1:numberOfLogs)));

    if isempty(selectedLogs)
        error('有効な時刻を持つログデータが見つかりません: %s', matFile);
    end

    trajectoryTime = timeData(selectedLogs);
    targetPosition = nan(numel(selectedLogs), 3);
    actualPosition = nan(numel(selectedLogs), 3);
    validLog = false(numel(selectedLogs), 1);

    % 目標位置Rは表示文字列から取得する。
    % 実位置は、小数第3位に丸められた表示文字列Pを使わず、
    % sensor.result.outputの先頭3要素[x,y,z]から直接取得する。
    for sampleNumber = 1:numel(selectedLogs)
        logNumber = selectedLogs(sampleNumber);

        if iscell(displayData)
            displayLine = char(string(displayData{logNumber}));
        else
            displayLine = char(string(displayData(logNumber)));
        end

        targetToken = regexp(displayLine, ...
            'R\s*\[\s*([^\]]+)\]', 'tokens', 'once');

        if isempty(targetToken)
            continue;
        end

        oneTarget = sscanf(strrep(targetToken{1}, ',', ' '), '%f').';

        if iscell(sensorResultData)
            oneSensorResult = sensorResultData{logNumber};
        else
            oneSensorResult = sensorResultData(logNumber);
        end

        if ~isstruct(oneSensorResult) || ...
                ~isfield(oneSensorResult, 'output') || ...
                ~isnumeric(oneSensorResult.output) || ...
                numel(oneSensorResult.output) < 3
            continue;
        end

        sensorOutput = oneSensorResult.output(:);
        onePosition = double(sensorOutput(1:3)).';

        if numel(oneTarget) == 3 && numel(onePosition) == 3 && ...
                all(isfinite(oneTarget)) && all(isfinite(onePosition))
            targetPosition(sampleNumber,:) = oneTarget;
            actualPosition(sampleNumber,:) = onePosition;
            validLog(sampleNumber) = true;
        end
    end

    if ~all(validLog)
        warning('%d個の読み取れないログレコードを除外しました: %s', ...
            nnz(~validLog), matFile);
        trajectoryTime = trajectoryTime(validLog);
        targetPosition = targetPosition(validLog,:);
        actualPosition = actualPosition(validLog,:);
    end

    if isempty(trajectoryTime)
        error('ログから軌道データを読み取れませんでした: %s', matFile);
    end

    trajectoryData{dataNumber}.time = trajectoryTime;
    trajectoryData{dataNumber}.target = targetPosition;
    trajectoryData{dataNumber}.position = actualPosition;
end

%% 読み込んだA、Bを分ける
trajectoryA = trajectoryData{1};
trajectoryB = trajectoryData{2};

% 目標軌道はAのデータを使用する。
% AとBで目標軌道が異なる場合は警告を表示する。
if ~isequal(size(trajectoryA.target), size(trajectoryB.target)) || ...
        any(abs(trajectoryA.target(:) - trajectoryB.target(:)) > 1e-9)
    warning(['AとBの目標軌道が一致しません。' ...
             'グラフにはAの目標軌道を表示します。']);
end

%% 目標軌道、Aの軌道、Bの軌道を1つのグラフに重ねる
% 目標が一点に静止している場合は、横方向の小さな動きを見やすくするため、
% 目標点を原点としたXY平面の変位をcm単位で表示する。
targetRange = max(trajectoryA.target, [], 1) - min(trajectoryA.target, [], 1);
targetIsStationary = max(targetRange) <= 1e-9;

figure('Color', 'w', 'Name', 'Trajectories: A vs B');
hold on;

if targetIsStationary
    targetPoint = trajectoryA.target(1,:);

    % 目標点からのXY方向のずれをmからcmへ変換する。
    trajectoryAxy = 100 * (trajectoryA.position(:,1:2) - targetPoint(1:2));
    trajectoryBxy = 100 * (trajectoryB.position(:,1:2) - targetPoint(1:2));

    plot(0, 0, 'kp', 'MarkerSize', 13, 'MarkerFaceColor', [1 0.85 0], ...
        'DisplayName', 'reference');
    plot(trajectoryAxy(:,1), trajectoryAxy(:,2), '-', ...
        'Color', [0 0.4470 0.7410], 'LineWidth', 1.8, ...
        'DisplayName', 'KL');
    plot(trajectoryBxy(:,1), trajectoryBxy(:,2), '-', ...
        'Color', [0.8500 0.3250 0.0980], 'LineWidth', 1.8, ...
        'DisplayName', 'LYKL');

    % xとyで同じ表示幅を使い、微小な揺れをつぶさずに表示する。
    allXY = [0 0; trajectoryAxy; trajectoryBxy];
    xMinimum = min(allXY(:,1));
    xMaximum = max(allXY(:,1));
    yMinimum = min(allXY(:,2));
    yMaximum = max(allXY(:,2));
    xySpan = max([xMaximum - xMinimum, yMaximum - yMinimum, 0.1]);
    xCenter = (xMinimum + xMaximum) / 2;
    yCenter = (yMinimum + yMaximum) / 2;
    xlim(xCenter + 0.6 * xySpan * [-1 1]);
    ylim(yCenter + 0.6 * xySpan * [-1 1]);

    axis equal;
    xlabel('$ x$ [cm]', 'Interpreter', 'latex');
    ylabel('$ y$ [cm]', 'Interpreter', 'latex');
    fprintf('Stationary target detected: showing the XY motion in cm.\n');
else
    plot3(trajectoryA.target(:,1), trajectoryA.target(:,2), trajectoryA.target(:,3), ...
        'k--', 'LineWidth', 2.2, 'DisplayName', 'reference');
    plot3(trajectoryA.position(:,1), trajectoryA.position(:,2), trajectoryA.position(:,3), ...
        '-', 'Color', [0 0.4470 0.7410], 'LineWidth', 1.8, ...
        'DisplayName', 'KL');
    plot3(trajectoryB.position(:,1), trajectoryB.position(:,2), trajectoryB.position(:,3), ...
        '-', 'Color', [0.8500 0.3250 0.0980], 'LineWidth', 1.8, ...
        'DisplayName', 'LYKL');

    axis equal;
    view(3);
    xlabel('$x$ [m]', 'Interpreter', 'latex');
    ylabel('$y$ [m]', 'Interpreter', 'latex');
    zlabel('z [m]');
end

grid on;
box on;
legend('Location', 'best');

fprintf('A: %.3f--%.3f s (%d samples)\n', ...
    trajectoryA.time(1), trajectoryA.time(end), numel(trajectoryA.time));
fprintf('B: %.3f--%.3f s (%d samples)\n', ...
    trajectoryB.time(1), trajectoryB.time(end), numel(trajectoryB.time));

%% Compare_Sim_movie
% LOGGER.save で保存した2つのシミュレーションログ（既存手法・提案手法）を読み込み，飛行の様子を比較する動画を作るスクリプト。
%   - 1つのウィンドウに 左 : x-y 平面（全体を固定表示），右 : 目標値（reference）の座標を追いかける3次元表示 を並べる
%   - 目標値は f フェーズを refPeriod [s]（1周期）分，f 以外のフェーズ（alignPhase = 'tfl' のときの 't', 'l'）をそのフェーズ中だけ全区間描き，各手法の軌跡は直近 tailTime [s] 分だけ尾のように描く
%   - 各手法の現在の位置・姿勢にドローン（円柱のアーム・モータ・厚みのあるロータ・機体中心の箱・機首方向の矢印）を陰影付きで描く
%   - 既存手法は薄く（細線・半透明），提案手法は solid（太線・不透明）で描いて提案手法を強調する
%   - fSave = true で VideoWriter により saveDir/fileName に保存する（false なら表示のみ）
% ※ 時刻は alignPhase（既定 'f'）の開始時刻を t = 0 に揃え，両ログに共通する区間を dtResample 間隔に線形補間して描く
% ※ 位置・姿勢は posAttr（既定 "" : シミュレーションなら plant の真値 "p"，実験なら estimator の "e"）を使う
% ※ 姿勢 q はオイラー角 [roll pitch yaw]（ZYX），クォータニオン [q0 q1 q2 q3]，回転行列（9要素）のいずれにも対応する
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

%% 設定 ===============================================================================================================================================
% {凡例ラベル, ファイル}（rootDir からの相対パス・絶対パス。ワイルドカード可で最初に一致したファイルを使う。"" なら選択ダイアログ）
%   凡例ラベルは LaTeX で解釈されるため，_ % & # は \_ のようにエスケープする（数式は $...$）
baseSetting = {"w/o NN-MEC", "Data/Sim_data/LemniscateT=10YawFollow__jxjyjz=0.18__woNNMEC_Log*.mat"};     % 既存手法（薄く描く）
propSetting = {"w/ NN-MEC", "Data/Sim_data/LemniscateT=10YawFollow__jxjyjz=0.18__wNNMEC_Log*.mat"};  % 提案手法（solid で描く）
agentIdx = 1;           % ログ内の agent 番号
posAttr = "";           % 位置・姿勢の attribute（"p": plant, "e": estimator, "": ログが実験なら "e"，シミュレーションなら "p"）
alignPhase = 'tfl';       % このフェーズの開始時刻を t = 0 に揃える
timeRange = [];         % [t0 t1] : 動画にする区間（alignPhase 開始からの相対時刻 [s]）。空なら両ログに共通する alignPhase の区間
dtResample = 0.01;      % [s] 補間の時間間隔（軌跡の尾の滑らかさ）
refPeriod = 10;          % [s] f フェーズの目標軌道の周期 T（refStartTime から1周期分の目標値を常に描く。f 以外のフェーズの目標値は全区間を描く）
refStartTime = 0;       % [s] 描画する1周期分の目標値の開始時刻（f フェーズ開始からの相対時刻）
tailTime = 1.0;         % [s] 軌跡の尾の長さ
followHalfWidth = [0.4 0.4 0.2]; % [m] 3次元表示で目標値の座標を中心に表示する範囲（中心 ± この値，[x y z]）
xyMargin = 0.2;         % [m] x-y 平面の表示範囲の余白
view3D = [-37.5 30];    % 3次元表示の視点 [方位角 仰角]

% 動画の設定
fps = 30;               % フレームレート
playbackSpeed = 1;      % 再生速度の倍率（1 : 実時間, 0.5 : スロー）
initialPause = 0;       % [s] 最初に初期位置で静止する時間
figSize = [1280 640];   % [px] ウィンドウサイズ（保存する動画の解像度になる）
axesPosXY = [0.06 0.10 0.40 0.70]; % x-y 平面の軸の位置 [左 下 幅 高さ]（ウィンドウに対する割合。目盛りラベルを除いた描画領域）
axesPos3D = [0.56 0.10 0.40 0.70]; % 3次元表示の軸の位置 [左 下 幅 高さ]
legendTop = 0.93;       % 凡例の上端の位置（ウィンドウに対する割合。これより上に時刻を表示する）
fSave = false;          % true: 動画を保存する, false: 表示のみ
saveDir = fullfile("Data", "movie");
fileName = "compare_sim.mp4";
videoProfile = "MPEG-4"; % VideoWriter のプロファイル（"MPEG-4", "Motion JPEG AVI" など）

% 見た目
FS = 14;                % FontSize
RGB = orderedcolors("gem"); % gem:MATLABデフォルトの淡い色
baseColor = RGB(1,:);       % 既存手法の色
propColor = RGB(2,:);       % 提案手法の色
refColor = [0 0 0];                 % 目標値の色
baseAlpha = 0.35;       % 既存手法の不透明度（1: 不透明, 0: 透明）
baseLW = 1.0;           % 既存手法の線幅
propLW = 2.5;           % 提案手法の線幅
refLW = 0.8;            % 目標値の線幅
MS = 100;                % 現在の目標値マーカーの大きさ（scatter の SizeData [pt^2]）
refMarker = "o";        % 現在の目標値マーカーの種類（"x", "o", "+", "*", "s", "d", "^", "p", "h" など）

% ドローンのフレーム（frameSize, rotorR は DRONE_PARAM の既定値。ロータは機体中心から [±Lx/2, ±Ly/2] に置く）
droneShape.frameSize = [0.16 0.16];     % [m] [Lx Ly]
droneShape.rotorR = 0.0392;             % [m] ロータ半径
droneShape.rotorH = 0.006;              % [m] ロータ（円盤）の厚み
droneShape.armR = 0.005;                % [m] アーム（円柱）の半径
droneShape.motorR = 0.009;              % [m] モータ（円柱）の半径
droneShape.motorH = 0.018;              % [m] モータの高さ（アーム中心からロータ下面まで）
droneShape.bodySize = [0.04 0.025 0.02]; % [m] 機体中心の箱の大きさ [x y z]
droneShape.noseColor = RGB(5,:);      % 機首方向を示す矢印の色（軌跡の色と見分けやすい色にする）
droneShape.noseSize = [0.035 0.04];      % [m] 機首方向を示す矢印の大きさ [長さ（機首方向） 幅]
droneScaleXY = 1;        % x-y 平面に描くドローンの拡大率（全体表示では小さく見えるため）
droneScale3D = 0.7;        % 3次元表示に描くドローンの拡大率

%% 読み込み ===========================================================================================================================================
base = load_sim_log(string(baseSetting{2}), string(baseSetting{1}), agentIdx, posAttr, alignPhase);
prop = load_sim_log(string(propSetting{2}), string(propSetting{1}), agentIdx, posAttr, alignPhase);

% 両ログに共通する区間を共通時刻とする
tStart = max(base.t(1), prop.t(1));
tEnd = min(base.t(end), prop.t(end));
if ~isempty(timeRange)
    if timeRange(1) < tStart || timeRange(2) > tEnd
        warning("Compare_Sim_movie: timeRange [%g %g] が両ログに共通する区間 [%.3f %.3f] を超えるため，共通区間に切り詰めます。", timeRange, tStart, tEnd);
    end
    tStart = max(tStart, timeRange(1));
    tEnd = min(tEnd, timeRange(2));
end
tGrid = (tStart:dtResample:tEnd)';
basePos = resample_value(base.t, base.pos, tGrid);
propPos = resample_value(prop.t, prop.pos, tGrid);
baseQuat = resample_quat(base.t, base.quat, tGrid);
propQuat = resample_quat(prop.t, prop.quat, tGrid);
refPos = resample_value(prop.t, prop.ref, tGrid); % 3次元表示で追いかける目標値（提案手法のログの目標値）
fprintf("共通時刻 : %.3f 〜 %.3f s（%s 開始 = 0 s）\n", tStart, tEnd, alignPhase);

% フェーズごとに描く目標値（提案手法のログのフェーズ区間を使う）
%   f フェーズ : refStartTime から1周期分を常に描く
%   f 以外のフェーズ（'t', 'l' など） : そのフェーズの全区間を，そのフェーズ中だけ描く
phases = prop.phases;
phaseRanges = prop.phaseRanges;
refPhasePos = cell(numel(phases), 1);
for phaseIdx = 1:numel(phases)
    phaseRange = phaseRanges(phaseIdx, :);
    if phases(phaseIdx) == 'f'
        refTimeStart = phaseRange(1) + refStartTime;
        refTimeEnd = min(refTimeStart + refPeriod, phaseRange(2));
        if refStartTime < 0 || refTimeStart + refPeriod > phaseRange(2)
            warning("Compare_Sim_movie: 目標値の1周期 [%g %g]（f 開始からの相対時刻）が f フェーズの区間 [0 %.3f] を超えるため，[%.3f %.3f] を描きます。", ...
                    refStartTime, refStartTime + refPeriod, diff(phaseRange), max(refTimeStart, phaseRange(1)) - phaseRange(1), refTimeEnd - phaseRange(1));
        end
        refPhasePos{phaseIdx} = resample_value(prop.t, prop.ref, (max(refTimeStart, phaseRange(1)):dtResample:refTimeEnd)');
    else
        refPhasePos{phaseIdx} = prop.ref(prop.t >= phaseRange(1) & prop.t <= phaseRange(2), :);
    end
end
% 各時刻のフェーズ（開始時刻が現在時刻以前のフェーズのうち最後のもの）
phaseNow = ones(size(tGrid));
for phaseIdx = 2:numel(phases)
    phaseNow(tGrid >= phaseRanges(phaseIdx, 1)) = phaseIdx;
end

% フレームごとの tGrid のインデックス（1フレームで playbackSpeed / fps [s] 進める）
frameIdx = unique(round(((tGrid(1):playbackSpeed / fps:tGrid(end)) - tGrid(1)) / dtResample) + 1);
frameIdx = [ones(1, round(initialPause * fps)), frameIdx];
tailLen = round(tailTime / dtResample);

%% 描画の準備 =========================================================================================================================================
% 軸の位置は固定する（tiledlayout だと3次元表示の目盛りラベルが変わるたびに配置が計算し直され，x-y 平面が拡大縮小して見える）
fig = figure("Color", "w", "Name", "Compare_Sim_movie", "Units", "pixels");
fig.Position(3:4) = figSize;
baseLabel = base.label;
propLabel = prop.label;

% 左 : x-y 平面
axXY = axes(fig, "Units", "normalized", "InnerPosition", axesPosXY, "PositionConstraint", "innerposition");
hold(axXY, "on");
refLinesXY = gobjects(numel(phases), 1);
for phaseIdx = 1:numel(phases)
    refLinesXY(phaseIdx) = plot(axXY, refPhasePos{phaseIdx}(:, 1), refPhasePos{phaseIdx}(:, 2), "--", "Color", refColor, "LineWidth", refLW);
end
baseTailXY = plot(axXY, NaN, NaN, "-", "Color", [baseColor baseAlpha], "LineWidth", baseLW);
propTailXY = plot(axXY, NaN, NaN, "-", "Color", propColor, "LineWidth", propLW);
refHeadXY = scatter(axXY, NaN, NaN, MS, refColor, refMarker, "LineWidth", 1.5);
baseDroneXY = gen_drone(axXY, droneShape, droneScaleXY, baseColor, baseAlpha, baseLW);
propDroneXY = gen_drone(axXY, droneShape, droneScaleXY, propColor, 1, propLW);
light(axXY, "Position", [0.5 -0.5 1], "Style", "infinite");
allRefPos = vertcat(refPhasePos{:});
allXY = [allRefPos(:, 1:2); basePos(:, 1:2); propPos(:, 1:2)];
xyMarginAll = xyMargin + droneScaleXY * (norm(droneShape.frameSize) / 2 + droneShape.rotorR); % ドローンが表示範囲からはみ出さないように広げる
xlim(axXY, [min(allXY(:, 1)) max(allXY(:, 1))] + [-1 1] * xyMarginAll);
ylim(axXY, [min(allXY(:, 2)) max(allXY(:, 2))] + [-1 1] * xyMarginAll);
% 軌跡は z = 0，ドローンは実際の高度に描くため，z の表示範囲を両方を含むように固定する（自動ではどちらかがクリップされて描画されない）
allZ = [0; basePos(:, 3); propPos(:, 3)];
zlim(axXY, [min(allZ) max(allZ)] + [-1 1] * xyMarginAll);
axis(axXY, "equal");
grid(axXY, "on");
box(axXY, "on");
xlabel(axXY, "$x$ [m]", "Interpreter", "latex", "FontSize", FS);
ylabel(axXY, "$y$ [m]", "Interpreter", "latex", "FontSize", FS);
title(axXY, "$x$-$y$ plane", "Interpreter", "latex", "FontSize", FS);
set(axXY, "FontSize", FS, "TickLabelInterpreter", "latex");

% 右 : 目標値の座標を追いかける3次元表示
ax3D = axes(fig, "Units", "normalized", "InnerPosition", axesPos3D, "PositionConstraint", "innerposition");
hold(ax3D, "on");
refLines3D = gobjects(numel(phases), 1);
for phaseIdx = 1:numel(phases)
    refLines3D(phaseIdx) = plot3(ax3D, refPhasePos{phaseIdx}(:, 1), refPhasePos{phaseIdx}(:, 2), refPhasePos{phaseIdx}(:, 3), "--", "Color", refColor, "LineWidth", refLW);
end
baseTail3D = plot3(ax3D, NaN, NaN, NaN, "-", "Color", [baseColor baseAlpha], "LineWidth", baseLW);
propTail3D = plot3(ax3D, NaN, NaN, NaN, "-", "Color", propColor, "LineWidth", propLW);
refHead3D = scatter3(ax3D, NaN, NaN, NaN, MS, refColor, refMarker, "LineWidth", 1.5);
baseDrone3D = gen_drone(ax3D, droneShape, droneScale3D, baseColor, baseAlpha, baseLW);
propDrone3D = gen_drone(ax3D, droneShape, droneScale3D, propColor, 1, propLW);
light(ax3D, "Position", [-0.5 -1 1.5], "Style", "infinite");
light(ax3D, "Position", [1 0.5 0.5], "Style", "infinite", "Color", 0.4 * [1 1 1]); % 影側を少し明るくする補助光
daspect(ax3D, [1 1 1]);
view(ax3D, view3D);
grid(ax3D, "on");
box(ax3D, "on");
xlabel(ax3D, "$x$ [m]", "Interpreter", "latex", "FontSize", FS);
ylabel(ax3D, "$y$ [m]", "Interpreter", "latex", "FontSize", FS);
zlabel(ax3D, "$z$ [m]", "Interpreter", "latex", "FontSize", FS);
title(ax3D, "Tracking view", "Interpreter", "latex", "FontSize", FS);
set(ax3D, "FontSize", FS, "TickLabelInterpreter", "latex");

% 凡例の Reference は f フェーズ（無ければ最初のフェーズ）の線で代表する
legendRefIdx = max([1, find(phases == 'f', 1)]);
lgd = legend(axXY, [refLinesXY(legendRefIdx), refHeadXY, baseTailXY, propTailXY], ["Reference", "Current reference", baseLabel, propLabel], ...
             "Interpreter", "latex", "Orientation", "horizontal", "FontSize", FS, "Units", "normalized");
lgd.Position(1:2) = [0.5 - lgd.Position(3) / 2, legendTop - lgd.Position(4)]; % 図の上部中央に固定する
timeText = annotation(fig, "textbox", [0 legendTop 1 1 - legendTop], "String", "", "Interpreter", "latex", "FontSize", FS, ...
                      "HorizontalAlignment", "center", "VerticalAlignment", "middle", "EdgeColor", "none");
% x-y 平面のカメラを固定する（自動のままだとドローンの姿勢に合わせて視野角が変わり，拡大縮小して見える）
drawnow;
set(axXY, "CameraPositionMode", "manual", "CameraTargetMode", "manual", "CameraUpVectorMode", "manual", "CameraViewAngleMode", "manual");

%% 動画の作成 =========================================================================================================================================
if fSave
    if ~isfolder(saveDir)
        mkdir(saveDir);
    end
    savePath = fullfile(saveDir, fileName);
    video = VideoWriter(savePath, videoProfile);
    video.FrameRate = fps;
    open(video);
    fprintf("録画を開始します（%d フレーム）...\n", numel(frameIdx));
end

for frame = 1:numel(frameIdx)
    kNow = frameIdx(frame);
    tailIds = max(1, kNow - tailLen):kNow;
    % x-y 平面
    set(baseTailXY, "XData", basePos(tailIds, 1), "YData", basePos(tailIds, 2));
    set(propTailXY, "XData", propPos(tailIds, 1), "YData", propPos(tailIds, 2));
    set(refHeadXY, "XData", refPos(kNow, 1), "YData", refPos(kNow, 2));
    baseDroneXY.Matrix = drone_matrix(basePos(kNow, :), baseQuat(kNow, :));
    propDroneXY.Matrix = drone_matrix(propPos(kNow, :), propQuat(kNow, :));
    % 3次元表示（表示範囲の中心を目標値の座標に合わせる）
    set(baseTail3D, "XData", basePos(tailIds, 1), "YData", basePos(tailIds, 2), "ZData", basePos(tailIds, 3));
    set(propTail3D, "XData", propPos(tailIds, 1), "YData", propPos(tailIds, 2), "ZData", propPos(tailIds, 3));
    set(refHead3D, "XData", refPos(kNow, 1), "YData", refPos(kNow, 2), "ZData", refPos(kNow, 3));
    baseDrone3D.Matrix = drone_matrix(basePos(kNow, :), baseQuat(kNow, :));
    propDrone3D.Matrix = drone_matrix(propPos(kNow, :), propQuat(kNow, :));
    set(ax3D, "XLim", refPos(kNow, 1) + [-1 1] * followHalfWidth(1), ...
              "YLim", refPos(kNow, 2) + [-1 1] * followHalfWidth(2), ...
              "ZLim", refPos(kNow, 3) + [-1 1] * followHalfWidth(3));
    % f 以外のフェーズの目標値はそのフェーズ中だけ表示する
    fShowRef = phases == 'f' | (1:numel(phases)) == phaseNow(kNow);
    set(refLinesXY(fShowRef), "Visible", "on");
    set(refLinesXY(~fShowRef), "Visible", "off");
    set(refLines3D(fShowRef), "Visible", "on");
    set(refLines3D(~fShowRef), "Visible", "off");
    timeText.String = sprintf("$t = %.2f$ s, phase: %s", tGrid(kNow), phase_name(phases(phaseNow(kNow))));
    drawnow;

    if fSave
        writeVideo(video, getframe(fig));
    else
        pause(1 / fps);
    end
end

if fSave
    close(video);
    fprintf("動画の保存が完了しました: %s\n", savePath);
end

%% ローカル関数 =======================================================================================================================================
function simLog = load_sim_log(pattern, label, agentIdx, posAttr, alignPhase)
% LOGGER.save で保存したログを読み込み，alignPhase の区間の時刻（開始を 0 とした相対時刻），位置，姿勢（クォータニオン），目標値を取り出す
if strlength(pattern) == 0
    [selectedFile, selectedDir] = uigetfile(fullfile("Data", "*.mat"), sprintf("%s のログを選択", label));
    if isequal(selectedFile, 0)
        error("Compare_Sim_movie: %s のログが選択されませんでした。", label);
    end
    filePath = fullfile(string(selectedDir), string(selectedFile));
else
    found = dir(pattern);
    if isempty(found)
        error("Compare_Sim_movie: %s のファイルが見つかりません（%s）。", label, pattern);
    end
    filePath = fullfile(string(found(1).folder), string(found(1).name));
    if numel(found) > 1
        warning("Compare_Sim_movie: %s に一致するファイルが %d 件あるため，最初のファイルを使います。", pattern, numel(found));
    end
end
fprintf("%s : %s\n", label, filePath);

logger = LOGGER(char(filePath));
if strlength(posAttr) == 0
    if ~isempty(logger.fExp) && logger.fExp
        posAttr = "e"; % 実験では真値（plant）が無い
    else
        posAttr = "p";
    end
end
t = logger.data("t", "", "", "phase", alignPhase);
if isempty(t)
    error("Compare_Sim_movie: %s に phase '%s' がありません。", filePath, alignPhase);
end
simLog.label = label;
simLog.t = t - t(1);
% alignPhase の各フェーズの区間（alignPhase 開始からの相対時刻，開始時刻順）。ログに無いフェーズは除く
simLog.phases = '';
simLog.phaseRanges = zeros(0, 2);
for phaseChar = unique(char(alignPhase), "stable")
    tPhase = logger.data("t", "", "", "phase", phaseChar);
    if ~isempty(tPhase)
        simLog.phases(end + 1) = phaseChar;
        simLog.phaseRanges(end + 1, :) = [tPhase(1) tPhase(end)] - t(1);
    end
end
[~, order] = sort(simLog.phaseRanges(:, 1));
simLog.phases = simLog.phases(order);
simLog.phaseRanges = simLog.phaseRanges(order, :);
simLog.pos = logger.data(agentIdx, "p", posAttr, "phase", alignPhase);
simLog.ref = logger.data(agentIdx, "p", "r", "phase", alignPhase);
simLog.quat = to_quat(logger.data(agentIdx, "q", posAttr, "phase", alignPhase));
end

function name = phase_name(phaseChar)
% フライトフェーズの文字（LOGGER の phase）を表示用の名前にする
switch phaseChar
    case 'a'
        name = "arming";
    case 't'
        name = "takeoff";
    case 'f'
        name = "flight";
    case 'l'
        name = "landing";
    otherwise
        name = string(phaseChar);
end
end

function quat = to_quat(q)
% 姿勢 q（各行 : オイラー角 [roll pitch yaw]（ZYX），クォータニオン [q0 q1 q2 q3]，回転行列（9要素, 列優先））をクォータニオン [q0 q1 q2 q3] に変換する
switch size(q, 2)
    case 3
        quat = Eul2Quat(q')';
    case 4
        quat = q;
    case 9
        quat = zeros(size(q, 1), 4);
        for row = 1:size(q, 1)
            quat(row, :) = rotm_to_quat(reshape(q(row, :), 3, 3));
        end
    otherwise
        error("Compare_Sim_movie: 姿勢 q の要素数 %d には対応していません。", size(q, 2));
end
quat = quat ./ vecnorm(quat, 2, 2);
end

function quat = rotm_to_quat(R)
% 回転行列をクォータニオン [q0 q1 q2 q3] に変換する
[~, maxIdx] = max([trace(R), R(1, 1), R(2, 2), R(3, 3)]);
switch maxIdx
    case 1
        s = 2 * sqrt(1 + trace(R));
        quat = [s / 4, (R(3, 2) - R(2, 3)) / s, (R(1, 3) - R(3, 1)) / s, (R(2, 1) - R(1, 2)) / s];
    case 2
        s = 2 * sqrt(1 + R(1, 1) - R(2, 2) - R(3, 3));
        quat = [(R(3, 2) - R(2, 3)) / s, s / 4, (R(1, 2) + R(2, 1)) / s, (R(1, 3) + R(3, 1)) / s];
    case 3
        s = 2 * sqrt(1 - R(1, 1) + R(2, 2) - R(3, 3));
        quat = [(R(1, 3) - R(3, 1)) / s, (R(1, 2) + R(2, 1)) / s, s / 4, (R(2, 3) + R(3, 2)) / s];
    case 4
        s = 2 * sqrt(1 - R(1, 1) - R(2, 2) + R(3, 3));
        quat = [(R(2, 1) - R(1, 2)) / s, (R(1, 3) + R(3, 1)) / s, (R(2, 3) + R(3, 2)) / s, s / 4];
end
end

function quat = resample_quat(t, quat, tGrid)
% クォータニオンを時刻 tGrid に補間する（符号の反転を揃えて成分ごとに線形補間し，正規化する）
for row = 2:size(quat, 1)
    if dot(quat(row, :), quat(row - 1, :)) < 0
        quat(row, :) = -quat(row, :);
    end
end
quat = resample_value(t, quat, tGrid);
quat = quat ./ vecnorm(quat, 2, 2);
end

function matrix = drone_matrix(pos, quat)
% 位置 pos と姿勢 quat（機体座標 → 慣性座標）から hgtransform の変換行列を作る
q0 = quat(1); q1 = quat(2); q2 = quat(3); q3 = quat(4);
R = [1 - 2 * (q2^2 + q3^2), 2 * (q1 * q2 - q0 * q3), 2 * (q1 * q3 + q0 * q2);
     2 * (q1 * q2 + q0 * q3), 1 - 2 * (q1^2 + q3^2), 2 * (q2 * q3 - q0 * q1);
     2 * (q1 * q3 - q0 * q2), 2 * (q2 * q3 + q0 * q1), 1 - 2 * (q1^2 + q2^2)];
matrix = [R, pos(:); 0 0 0 1];
end

function droneTransform = gen_drone(ax, shape, scale, color, alpha, lineWidth)
% 機体座標で原点を中心とするドローン（円柱の X 型アーム・モータ・厚みのあるロータ・機体中心の箱・機首方向）を hgtransform にまとめて描く
%   shape の長さはすべて scale 倍する。ロータは [±Lx/2, ±Ly/2] に置き，機首（機体 x 軸の正の向き）側のロータは塗りを濃くする
%   陰影は軸に置いた light で付く（light が無いと立体感が出ない）
droneTransform = hgtransform("Parent", ax);
d = scale * shape.frameSize / 2;
rotorR = scale * shape.rotorR;
rotorH = scale * shape.rotorH;
armR = scale * shape.armR;
motorR = scale * shape.motorR;
motorH = scale * shape.motorH;
bodyHalf = scale * shape.bodySize / 2;
nSide = 24; % 円柱の周方向の分割数
rotorPos = [d(1) d(2); d(1) -d(2); -d(1) d(2); -d(1) -d(2)];
darkColor = 0.6 * color;           % モータ・機体の色（本体色より暗く）
lightColor = 0.5 * color + 0.5;    % ロータの色（本体色より明るく）
% アーム（対角のロータ同士を結ぶ円柱）
add_cylinder(droneTransform, [rotorPos(1, :) 0], [rotorPos(4, :) 0], armR, color, alpha, nSide);
add_cylinder(droneTransform, [rotorPos(2, :) 0], [rotorPos(3, :) 0], armR, color, alpha, nSide);
% モータとロータ
for rotorIdx = 1:4
    add_cylinder(droneTransform, [rotorPos(rotorIdx, :) -armR], [rotorPos(rotorIdx, :) motorH], motorR, darkColor, alpha, nSide);
    faceAlpha = alpha * (0.3 + 0.35 * (rotorPos(rotorIdx, 1) > 0)); % 機首側を濃く
    rotorCap = add_cylinder(droneTransform, [rotorPos(rotorIdx, :) motorH], [rotorPos(rotorIdx, :) motorH + rotorH], rotorR, lightColor, faceAlpha, 2 * nSide);
    set(rotorCap, "EdgeColor", color, "EdgeAlpha", alpha, "LineWidth", 0.5 * lineWidth); % ロータの縁を描いて上から見ても輪郭が分かるようにする
end
% 機体中心の箱
[boxX, boxY, boxZ] = ndgrid([-1 1] * bodyHalf(1), [-1 1] * bodyHalf(2), [-1 1] * bodyHalf(3));
boxFaces = [1 3 4 2; 5 6 8 7; 1 2 6 5; 3 7 8 4; 1 5 7 3; 2 4 8 6];
patch("Parent", droneTransform, "Vertices", [boxX(:) boxY(:) boxZ(:)], "Faces", boxFaces, ...
      "FaceColor", darkColor, "FaceAlpha", alpha, "EdgeColor", "none", "FaceLighting", "flat", "AmbientStrength", 0.5);
% 機首方向（箱の上面に shape.noseColor の矢印を描く。機体からはみ出させず，軌跡と色を変えて見分けやすくする）
%   矢印は機体中心を基準に，先端を前方 0.6L，後端を後方 0.4L に置き，後端中央を 0.25L だけ切り欠く（L : 長さ）
noseSize = scale * shape.noseSize;
noseX = noseSize(1) * [0.6, -0.4, -0.15, -0.4];
noseY = noseSize(2) / 2 * [0, 1, 0, -1];
patch("Parent", droneTransform, "XData", noseX, "YData", noseY, "ZData", 1.05 * bodyHalf(3) * ones(1, 4), ...
      "FaceColor", shape.noseColor, "FaceAlpha", alpha, "EdgeColor", "k", "EdgeAlpha", alpha, "LineWidth", 0.5 * lineWidth, ...
      "FaceLighting", "none");
end

function topCap = add_cylinder(parent, p1, p2, radius, color, alpha, nSide)
% 点 p1 から p2 までを軸とする半径 radius の円柱（側面 + 両端の蓋）を parent に描き，p2 側の蓋を返す
axisDir = (p2 - p1) / norm(p2 - p1);
helper = [1 0 0];
if abs(dot(helper, axisDir)) > 0.9
    helper = [0 1 0];
end
e1 = cross(axisDir, helper);
e1 = e1 / norm(e1);
e2 = cross(axisDir, e1);
theta = linspace(0, 2 * pi, nSide + 1)';
ring = radius * (cos(theta) * e1 + sin(theta) * e2);
bottom = p1 + ring;
top = p2 + ring;
surface([bottom(:, 1)'; top(:, 1)'], [bottom(:, 2)'; top(:, 2)'], [bottom(:, 3)'; top(:, 3)'], ...
        "Parent", parent, "FaceColor", color, "FaceAlpha", alpha, "EdgeColor", "none", ...
        "FaceLighting", "gouraud", "AmbientStrength", 0.5);
patch("Parent", parent, "XData", bottom(:, 1), "YData", bottom(:, 2), "ZData", bottom(:, 3), ...
      "FaceColor", color, "FaceAlpha", alpha, "EdgeColor", "none", "FaceLighting", "flat", "AmbientStrength", 0.5);
topCap = patch("Parent", parent, "XData", top(:, 1), "YData", top(:, 2), "ZData", top(:, 3), ...
               "FaceColor", color, "FaceAlpha", alpha, "EdgeColor", "none", "FaceLighting", "flat", "AmbientStrength", 0.5);
end

function value = resample_value(t, value, tGrid)
% value を時刻 tGrid に線形補間する（時刻が重複するステップは最初の値を使う）
[tUnique, uniqueIdx] = unique(t, "stable");
value = interp1(tUnique, value(uniqueIdx, :), tGrid, "linear");
end

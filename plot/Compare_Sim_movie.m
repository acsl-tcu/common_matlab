%% Compare_Sim_movie
% LOGGER.save で保存した2つのシミュレーションログ（既存手法・提案手法）を読み込み，飛行の様子を比較する動画を作るスクリプト。
%   - 1つのウィンドウに 左 : x-y 平面（全体を固定表示），右 : 目標値（reference）の座標を追いかける3次元表示 を並べる
%   - 目標値は refPeriod [s]（1周期）分を常に描き，各手法の軌跡は直近 tailTime [s] 分だけ尾のように描く
%   - 各手法の現在の位置・姿勢にドローンのフレーム（アーム・ロータ・機首方向）を描く
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
baseSetting = {"w/o NN-MEC", "Data/Sim_data/LemniscateT=10__yaw_follow__wo_NN-MEC_Log*.mat"};     % 既存手法（薄く描く）
propSetting = {"w/ NN-MEC", "Data/Sim_data/LemniscateT=10__yaw_follow__w_NN-MEC_Log*.mat"};  % 提案手法（solid で描く）
agentIdx = 1;           % ログ内の agent 番号
posAttr = "";           % 位置・姿勢の attribute（"p": plant, "e": estimator, "": ログが実験なら "e"，シミュレーションなら "p"）
alignPhase = 'f';       % このフェーズの開始時刻を t = 0 に揃える
timeRange = [10 30];         % [t0 t1] : 動画にする区間（alignPhase 開始からの相対時刻 [s]）。空なら両ログに共通する alignPhase の区間
dtResample = 0.01;      % [s] 補間の時間間隔（軌跡の尾の滑らかさ）
refPeriod = 10;          % [s] 目標軌道の周期 T（refStartTime から1周期分の目標値を常に描く）
refStartTime = 0;       % [s] 描画する1周期分の目標値の開始時刻（alignPhase 開始からの相対時刻）
tailTime = 1.0;         % [s] 軌跡の尾の長さ
followHalfWidth = [0.3 0.3 0.3]; % [m] 3次元表示で目標値の座標を中心に表示する範囲（中心 ± この値，[x y z]）
xyMargin = 0.2;         % [m] x-y 平面の表示範囲の余白
view3D = [-37.5 30];    % 3次元表示の視点 [方位角 仰角]

% 動画の設定
fps = 30;               % フレームレート
playbackSpeed = 1;      % 再生速度の倍率（1 : 実時間, 0.5 : スロー）
initialPause = 0;       % [s] 最初に初期位置で静止する時間
figSize = [1280 640];   % [px] ウィンドウサイズ（保存する動画の解像度になる）
fSave = false;          % true: 動画を保存する, false: 表示のみ
saveDir = fullfile("Data", "movie");
fileName = "compare_sim.mp4";
videoProfile = "MPEG-4"; % VideoWriter のプロファイル（"MPEG-4", "Motion JPEG AVI" など）

% 見た目
FS = 14;                % FontSize
baseColor = [0 0.4470 0.7410];      % 既存手法の色
propColor = [0.8500 0.1250 0.0980]; % 提案手法の色
refColor = [0 0 0];                 % 目標値の色
baseAlpha = 0.35;       % 既存手法の不透明度（1: 不透明, 0: 透明）
baseLW = 1.0;           % 既存手法の線幅
propLW = 2.5;           % 提案手法の線幅
refLW = 0.8;            % 目標値の線幅
MS = 60;                % 現在の目標値マーカーの大きさ（scatter の SizeData [pt^2]）

% ドローンのフレーム（DRONE_PARAM の既定値。ロータは機体中心から [±Lx/2, ±Ly/2] に置く）
frameSize = [0.16 0.16]; % [m] [Lx Ly]
rotorR = 0.0392;         % [m] ロータ半径
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

% 常に描く1周期分の目標値
refTimeEnd = min(refStartTime + refPeriod, prop.t(end));
if refStartTime < prop.t(1) || refStartTime + refPeriod > prop.t(end)
    warning("Compare_Sim_movie: 目標値の1周期 [%g %g] がログの区間を超えるため，[%.3f %.3f] を描きます。", refStartTime, refStartTime + refPeriod, max(refStartTime, prop.t(1)), refTimeEnd);
end
refPeriodPos = resample_value(prop.t, prop.ref, (max(refStartTime, prop.t(1)):dtResample:refTimeEnd)');

% フレームごとの tGrid のインデックス（1フレームで playbackSpeed / fps [s] 進める）
frameIdx = unique(round(((tGrid(1):playbackSpeed / fps:tGrid(end)) - tGrid(1)) / dtResample) + 1);
frameIdx = [ones(1, round(initialPause * fps)), frameIdx];
tailLen = round(tailTime / dtResample);

%% 描画の準備 =========================================================================================================================================
fig = figure("Color", "w", "Name", "Compare_Sim_movie", "Units", "pixels");
fig.Position(3:4) = figSize;
layout = tiledlayout(fig, 1, 2, "TileSpacing", "compact", "Padding", "compact");
baseLabel = base.label;
propLabel = prop.label;

% 左 : x-y 平面
axXY = nexttile(layout);
hold(axXY, "on");
refXY = plot(axXY, refPeriodPos(:, 1), refPeriodPos(:, 2), "--", "Color", refColor, "LineWidth", refLW);
baseTailXY = plot(axXY, NaN, NaN, "-", "Color", [baseColor baseAlpha], "LineWidth", baseLW);
propTailXY = plot(axXY, NaN, NaN, "-", "Color", propColor, "LineWidth", propLW);
refHeadXY = scatter(axXY, NaN, NaN, MS, "k", "x", "LineWidth", 1.5);
baseDroneXY = gen_drone(axXY, frameSize * droneScaleXY, rotorR * droneScaleXY, baseColor, baseAlpha, baseLW);
propDroneXY = gen_drone(axXY, frameSize * droneScaleXY, rotorR * droneScaleXY, propColor, 1, propLW);
allXY = [refPeriodPos(:, 1:2); basePos(:, 1:2); propPos(:, 1:2)];
xyMarginAll = xyMargin + droneScaleXY * (norm(frameSize) / 2 + rotorR); % ドローンが表示範囲からはみ出さないように広げる
xlim(axXY, [min(allXY(:, 1)) max(allXY(:, 1))] + [-1 1] * xyMarginAll);
ylim(axXY, [min(allXY(:, 2)) max(allXY(:, 2))] + [-1 1] * xyMarginAll);
axis(axXY, "equal");
grid(axXY, "on");
box(axXY, "on");
xlabel(axXY, "$x$ [m]", "Interpreter", "latex", "FontSize", FS);
ylabel(axXY, "$y$ [m]", "Interpreter", "latex", "FontSize", FS);
title(axXY, "$x$-$y$ plane", "Interpreter", "latex", "FontSize", FS);
set(axXY, "FontSize", FS, "TickLabelInterpreter", "latex");

% 右 : 目標値の座標を追いかける3次元表示
ax3D = nexttile(layout);
hold(ax3D, "on");
plot3(ax3D, refPeriodPos(:, 1), refPeriodPos(:, 2), refPeriodPos(:, 3), "--", "Color", refColor, "LineWidth", refLW);
baseTail3D = plot3(ax3D, NaN, NaN, NaN, "-", "Color", [baseColor baseAlpha], "LineWidth", baseLW);
propTail3D = plot3(ax3D, NaN, NaN, NaN, "-", "Color", propColor, "LineWidth", propLW);
refHead3D = scatter3(ax3D, NaN, NaN, NaN, MS, "k", "x", "LineWidth", 1.5);
baseDrone3D = gen_drone(ax3D, frameSize * droneScale3D, rotorR * droneScale3D, baseColor, baseAlpha, baseLW);
propDrone3D = gen_drone(ax3D, frameSize * droneScale3D, rotorR * droneScale3D, propColor, 1, propLW);
daspect(ax3D, [1 1 1]);
view(ax3D, view3D);
grid(ax3D, "on");
box(ax3D, "on");
xlabel(ax3D, "$x$ [m]", "Interpreter", "latex", "FontSize", FS);
ylabel(ax3D, "$y$ [m]", "Interpreter", "latex", "FontSize", FS);
zlabel(ax3D, "$z$ [m]", "Interpreter", "latex", "FontSize", FS);
title(ax3D, "Tracking view", "Interpreter", "latex", "FontSize", FS);
set(ax3D, "FontSize", FS, "TickLabelInterpreter", "latex");

lgd = legend(axXY, [refXY, refHeadXY, baseTailXY, propTailXY], ["Reference", "Current reference", baseLabel, propLabel], ...
             "Interpreter", "latex", "Orientation", "horizontal", "FontSize", FS);
lgd.Layout.Tile = "north";
timeText = title(layout, "", "Interpreter", "latex", "FontSize", FS);

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
    timeText.String = sprintf("$t = %.2f$ s", tGrid(kNow));
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
simLog.pos = logger.data(agentIdx, "p", posAttr, "phase", alignPhase);
simLog.ref = logger.data(agentIdx, "p", "r", "phase", alignPhase);
simLog.quat = to_quat(logger.data(agentIdx, "q", posAttr, "phase", alignPhase));
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

function droneTransform = gen_drone(ax, frameSize, rotorR, color, alpha, lineWidth)
% 機体座標で原点を中心とするドローンのフレーム（X 型のアーム・ロータ・機首方向）を hgtransform にまとめて描く
%   ロータは [±Lx/2, ±Ly/2] に置き，機首（機体 x 軸の正の向き）側のロータは塗りを濃くする
droneTransform = hgtransform("Parent", ax);
d = frameSize / 2;
rotorPos = [d(1) d(2); d(1) -d(2); -d(1) d(2); -d(1) -d(2)];
% アーム
plot3(ax, [rotorPos(1, 1) rotorPos(4, 1) NaN rotorPos(2, 1) rotorPos(3, 1)], ...
          [rotorPos(1, 2) rotorPos(4, 2) NaN rotorPos(2, 2) rotorPos(3, 2)], zeros(1, 5), ...
      "-", "Color", [color alpha], "LineWidth", 1.5 * lineWidth, "Parent", droneTransform);
% 機首方向
plot3(ax, [0 d(1) + rotorR], [0 0], [0 0], "-", "Color", [color alpha], "LineWidth", lineWidth, "Parent", droneTransform);
% ロータ
theta = linspace(0, 2 * pi, 40);
for rotorIdx = 1:4
    faceAlpha = alpha * (0.25 + 0.35 * (rotorPos(rotorIdx, 1) > 0)); % 機首側を濃く
    patch(ax, rotorPos(rotorIdx, 1) + rotorR * cos(theta), rotorPos(rotorIdx, 2) + rotorR * sin(theta), zeros(size(theta)), color, ...
          "FaceAlpha", faceAlpha, "EdgeColor", color, "EdgeAlpha", alpha, "LineWidth", 0.5 * lineWidth, "Parent", droneTransform);
end
% 機体中心
bodyHalf = 0.2 * min(d);
patch(ax, bodyHalf * [-1 1 1 -1], bodyHalf * [-1 -1 1 1], zeros(1, 4), color, ...
      "FaceAlpha", alpha, "EdgeColor", "none", "Parent", droneTransform);
end

function value = resample_value(t, value, tGrid)
% value を時刻 tGrid に線形補間する（時刻が重複するステップは最初の値を使う）
[tUnique, uniqueIdx] = unique(t, "stable");
value = interp1(tUnique, value(uniqueIdx, :), tGrid, "linear");
end

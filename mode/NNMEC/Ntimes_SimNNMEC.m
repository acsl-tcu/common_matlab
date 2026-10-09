%% Ntimes_SimNNMEC
% SimNNMEC.m の設定を使い，プラントにモンテカルロ的なモデル誤差（物理パラメータ）を与えて
% GUI(SimExp)を介さずに複数回シミュレーションするスクリプト。
%   - 機体・推定器・リファレンス・制御器などの設定は毎試行 SimNNMEC.m をそのまま評価して生成する
%   - SimNNMEC.m 内のプラントパラメータ上書きは，本スクリプトでサンプリングした値で再度上書きする
%   - フェーズ遷移(a -> t -> f (-> l))はキー入力の代わりに時刻で自動切替する
%   - 結果(サンプル値，飛行フェーズの追従誤差，軌跡)は Data/NNMEC_MonteCarlo/ に保存する
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
[~, tmp] = regexp(genpath('.'), '\.\\\.git.*?;', 'match', 'split');
cellfun(@(xx) addpath(xx), tmp, 'UniformOutput', false);
close all hidden; clear; clc;
userpath('clear');

%% モンテカルロ設定 ===================================================================================================================================
simScript = "SimNNMEC.m"; % 設定を流用するシミュレーションスクリプト
trialN = 100;              % 試行回数
seed = 0;                 % 乱数シード（パラメータサンプリング・センサノイズの再現用）

% モデル誤差を与えるプラントパラメータ（一様分布 U[min, max]）
% 1:mass  |  2,3:Lx,y  |  4,5: lx,y  |  6,7,8: jx,y,z  |  9: gravity
% 10,11,12,13: km(各ロータ定数)  |  14,15,16,17: k(推力定数)  |  18: rotor_r
errorParam = struct( ...
    "name",  {"mass",          "jx",         "jy"}, ...
    "idx",   {1,               6,            7}, ...
    "range", {[0.7125 0.7875], [0.06 0.18], [0.06 0.18]});
fSameJxJy = true; % true: jy に jx と同じサンプル値を使う（機体の対称性を仮定）

% フェーズ遷移（キー入力の代替）
nArmingStep = 1;        % arming(a) のステップ数（時刻は進まない）
tFlightStart = 20;      % この時刻で take-off(t) -> flight(f)
tLandingStart = inf;    % この時刻で flight(f) -> landing(l)。inf なら landing なし（te で終了）
divergeLimit = 10;      % プラント位置のノルムがこれを超えたら発散とみなして試行を打ち切る [m]
zLowerLimit = 0;        % flight 中にプラントの z がこれを下回ったら墜落とみなして試行を打ち切る [m]

fDisplay = false;       % 各ステップのコンソール表示（SimNNMEC の logger.display_on）
fKeepLogger = false;    % true: 各試行の LOGGER を結果に保持する（メモリ・保存容量が大きくなる）
fSave = true;           % 結果を .mat で保存する
saveDir = fullfile("Data", "NNMEC_MonteCarlo");

%% パラメータのサンプリング ===========================================================================================================================
rng(seed);
paramN = numel(errorParam);
sampledParam = zeros(trialN, paramN);
for paramIdx = 1:paramN
    range = errorParam(paramIdx).range;
    sampledParam(:, paramIdx) = range(1) + (range(2) - range(1)) * rand(trialN, 1);
end
paramNames = [errorParam.name];
if fSameJxJy && all(ismember(["jx", "jy"], paramNames))
    sampledParam(:, paramNames == "jy") = sampledParam(:, paramNames == "jx");
end

%% モンテカルロシミュレーション =======================================================================================================================
phaseOpts = struct("nArmingStep", nArmingStep, "tFlightStart", tFlightStart, ...
                   "tLandingStart", tLandingStart, "divergeLimit", divergeLimit, "zLowerLimit", zLowerLimit);
results = repmat(struct("trial", [], "param", [], "fDiverged", false, "errorMsg", "", ...
                        "rmse", nan(1, 3), "maxError", nan(1, 3), "t", [], "p", [], "pr", [], "log", [], ...
                        "calcTime", nan, "logger", []), trialN, 1);

for trial = 1:trialN
    fprintf("\n===== Trial %d / %d =====\n", trial, trialN);
    results(trial).trial = trial;
    results(trial).param = sampledParam(trial, :);
    try
        [agent, time, logger, env, simInfo] = build_sim(simScript);
        logger.display_on = fDisplay;

        % モデル誤差をプラントに与える（SimNNMEC.m 内の上書きをさらに上書き）
        for paramIdx = 1:paramN
            agent.plant.param(errorParam(paramIdx).idx) = sampledParam(trial, paramIdx);
        end
        fprintf("plant param : %s\n", strjoin(paramNames + "=" + string(sampledParam(trial, :)), ", "));

        rng(seed + trial); % センサノイズ等の再現用
        tStart = tic;
        results(trial).fDiverged = run_headless(agent, time, logger, env, phaseOpts);
        results(trial).calcTime = toc(tStart);

        % 全フェーズ(空回しを除く)のログ: plant/estimator/sensor/reference の全状態，入力，controller の結果
        results(trial).log = extract_log_data(logger);
        % 飛行フェーズのプラント位置(真値)とリファレンス位置
        flightIds = results(trial).log.phase == 'f';
        results(trial).t = results(trial).log.t(flightIds);
        results(trial).p = results(trial).log.plant.state.p(flightIds, :);
        results(trial).pr = results(trial).log.reference.state.p(flightIds, :);
        posError = results(trial).p - results(trial).pr;
        results(trial).rmse = sqrt(mean(posError.^2, 1));
        results(trial).maxError = max(abs(posError), [], 1);
        if fKeepLogger
            results(trial).logger = logger;
        end
        fprintf("diverged : %d  |  RMSE [x y z] = [%.4f %.4f %.4f]  |  %.1f s\n", ...
                results(trial).fDiverged, results(trial).rmse, results(trial).calcTime);
    catch err
        results(trial).fDiverged = true;
        results(trial).errorMsg = string(err.message);
        warning("Ntimes_SimNNMEC: Trial %d failed : %s", trial, err.message);
    end
end

%% 結果の集計・保存 ===================================================================================================================================
summaryTable = array2table(sampledParam, "VariableNames", cellstr(paramNames));
summaryTable.fDiverged = [results.fDiverged]';
rmseAll = reshape([results.rmse], 3, [])';
summaryTable.rmseX = rmseAll(:, 1);
summaryTable.rmseY = rmseAll(:, 2);
summaryTable.rmseZ = rmseAll(:, 3);
summaryTable.rmseNorm = vecnorm(rmseAll, 2, 2);
disp(summaryTable);
validRow = ~summaryTable.fDiverged;
fprintf("発散/失敗 : %d / %d 試行\n", sum(~validRow), trialN);
fprintf("RMSE平均 [x y z] = [%.4f %.4f %.4f]\n", mean(rmseAll(validRow, :), 1));

if fSave
    if ~exist(saveDir, "dir"); mkdir(saveDir); end
    nowTime = datetime("now", "Format", "yyyy-M-d_H_m_s");
    saveName = fullfile(saveDir, string(nowTime) + "__Ntimes_SimNNMEC__trial" + trialN + ".mat");
    if ~exist("simInfo", "var"); simInfo = struct(); end % 全試行が構築段階で失敗した場合
    save(saveName, "results", "summaryTable", "errorParam", "sampledParam", "seed", "phaseOpts", "simScript", "simInfo", "-v7.3");
    fprintf("保存先 : %s\n", saveName);
end

%% 描画 ===============================================================================================================================================
FS = 16; % FontSize
LW = 1.5;% LineWidth

% 各パラメータと RMSE(ノルム) の関係
figure(100); clf;
tiledlayout(1, paramN);
for paramIdx = 1:paramN
    nexttile;
    scatter(sampledParam(validRow, paramIdx), summaryTable.rmseNorm(validRow), 40, "filled"); hold on
    scatter(sampledParam(~validRow, paramIdx), zeros(sum(~validRow), 1), 40, "rx", "LineWidth", LW);
    xlabel(paramNames(paramIdx)); ylabel("RMSE norm [m]"); grid on
    set(gca, "FontSize", FS);
end

% 各試行の xy 軌跡
figure(101); clf; hold on
for trial = find(validRow)'
    plot(results(trial).p(:, 1), results(trial).p(:, 2), "LineWidth", 0.8);
end
firstValid = find(validRow, 1);
if ~isempty(firstValid)
    plot(results(firstValid).pr(:, 1), results(firstValid).pr(:, 2), "k--", "LineWidth", LW);
end
xlabel("x [m]"); ylabel("y [m]"); axis equal; grid on
set(gca, "FontSize", FS);

% 各試行の z
figure(102); clf; hold on
for trial = find(validRow)'
    plot(results(trial).t, results(trial).p(:, 3), "LineWidth", 0.8);
end
if ~isempty(firstValid)
    plot(results(firstValid).t, results(firstValid).pr(:, 3), "k--", "LineWidth", LW);
end
xlabel("time [s]"); ylabel("z [m]"); grid on
set(gca, "FontSize", FS);


%% ローカル関数 =======================================================================================================================================
function [agent, time, logger, env, simInfo] = build_sim(simScript)
% simScript をこの関数のワークスペースで評価し，シミュレーションに必要なインスタンスを返す。
% （スクリプト内の clear 等が呼び出し元のワークスペースに影響しないよう関数内で実行）
% フォルダ付きで run するとカレントディレクトリが一時的に移動し，相対パス参照(pt ファイル等)が壊れるため名前だけで呼ぶ
% simInfo : 比較時の識別用に，使用したモデルファイル名・制御器クラス名・dt・te を記録する
env = [];
[scriptDir, scriptName] = fileparts(simScript);
if strlength(scriptDir) > 0
    addpath(scriptDir);
end
run(scriptName);

simInfo = struct("dt", time.dt, "te", time.te, "ptName", "", "onnxName", "", "RNNptName", "", "controller", "");
for varName = ["ptName", "onnxName", "RNNptName"]
    if exist(varName, "var")
        simInfo.(varName) = string(eval(varName));
    end
end
try
    controllerNames = string(agent.cha_allocation.controller);
    simInfo.controller = strjoin(controllerNames + ":" + arrayfun(@(name) string(class(agent.controller.(name))), controllerNames), ", ");
catch
    % cha_allocation.controller を使わない設定の場合は記録しない
end
end

function fDiverged = run_headless(agent, time, logger, env, opts)
% SimExp (start_app, loop, do_calculation, do_prop) の処理を GUI なしで再現する。
fDiverged = false;
for agentIdx = 1:numel(agent)
    agent(agentIdx).cha_allocation = expand_cha_allocation(agent(agentIdx));
end

% 空回し（SimExp.start_app: isReady=false で a,t,f,l を1回ずつ実行。プラントは更新しない）
for cha = ['a', 't', 'f', 'l']
    do_calculation(agent, time, logger, env, cha, false);
end
time.t = time.ts;

% 本番
armingStep = 0;
while time.t < time.te
    if armingStep < opts.nArmingStep
        cha = 'a';
        armingStep = armingStep + 1;
    elseif time.t < opts.tFlightStart
        cha = 't';
    elseif time.t < opts.tLandingStart
        cha = 'f';
    else
        cha = 'l';
    end
    do_calculation(agent, time, logger, env, cha, true);
    if contains('flts', cha)
        time.t = time.t + time.dt;
    else
        time.t = 0;
    end

    p = agent(1).plant.state.p;
    if any(~isfinite(p)) || norm(p) > opts.divergeLimit
        fDiverged = true;
        fprintf("diverged at t = %.3f s\n", time.t);
        break
    end
    if cha == 'f' && p(3) < opts.zLowerLimit
        fDiverged = true;
        fprintf("crashed at t = %.3f s\n", time.t);
        break
    end
end
end

function do_calculation(agent, time, logger, env, cha, isReady)
% SimExp.do_calculation 相当
for agentIdx = 1:numel(agent)
    agent(agentIdx).cha = cha;
    for prop = ["sensor", "estimator", "reference", "controller", "input_transform", "plant"]
        do_prop(agent, agentIdx, prop, time, logger, env, cha, isReady);
    end
end
logger.logging(time, cha, agent, []);
time.k = logger.k;
end

function do_prop(agent, agentIdx, prop, time, logger, env, cha, isReady)
% SimExp.do_prop 相当
self = agent(agentIdx);
if ~isReady && strcmp(prop, "plant")
    return
end
if ~isReady
    fCha = "0";
    time.t = 0;
else
    fCha = cha;
end
list = self.cha_allocation.(cha).(prop);
if isempty(list)
    self.(prop).result = self.(prop).do(time, fCha, logger, env, agent, agentIdx);
else
    self.(prop).result = self.(prop).(list(1)).do(time, fCha, logger, env, agent, agentIdx);
    for listIdx = 2:length(list)
        self.(prop).result = merge_result(self.(prop).result, self.(prop).(list(listIdx)).do(time, fCha, logger, env, agent, agentIdx));
    end
end
end

function logData = extract_log_data(logger)
% LOGGER.Data(agent 1) を時系列の数値配列に変換する（空回しの4つ分を除く。各行が1ステップ）
%   logData.t, logData.phase                 : 時刻，フェーズ('a','t','f','l')
%   logData.<prop>.state.<name>              : prop = plant/estimator/sensor/reference の各状態(STATE_CLASS の list 全て)
%   logData.<prop>.<field>                   : result の state 以外の数値フィールド（controller の delta_input 等）
%   logData.input                            : 制御入力
logIds = 5:logger.k;
agentData = logger.Data.agent(1);
logData.t = logger.Data.t(logIds);
logData.phase = char(logger.Data.phase(logIds));
for prop = ["plant", "estimator", "sensor", "reference", "controller"]
    logData.(prop) = stack_results(agentData.(prop).result(logIds));
end
logData.input = stack_rows(agentData.input(logIds));
end

function out = stack_results(resultCell)
% 各ステップの result 構造体の cell 配列を，フィールドごとの時系列に変換する
out = struct();
allNames = cellfun(@(res) string(fieldnames(res))', resultCell(cellfun(@isstruct, resultCell)), "UniformOutput", false);
fieldNames = unique([allNames{:}], "stable");
for fieldName = fieldNames
    values = cellfun(@(res) get_field_or_empty(res, fieldName), resultCell, "UniformOutput", false);
    firstIdx = find(~cellfun(@isempty, values), 1);
    if isempty(firstIdx)
        continue
    end
    firstValue = values{firstIdx};
    if isa(firstValue, "STATE_CLASS")
        % フェーズによって result を出すクラスが異なる(例: takeoff と time_varying)ため全ステップの状態名を集める
        stateLists = cellfun(@(state) reshape(string(state.list), 1, []), values(cellfun(@(value) isa(value, "STATE_CLASS"), values)), "UniformOutput", false);
        for stateName = unique([stateLists{:}], "stable")
            out.(fieldName).(stateName) = stack_rows(cellfun(@(state) get_state_or_empty(state, stateName), values, "UniformOutput", false));
        end
    elseif isnumeric(firstValue) || islogical(firstValue)
        out.(fieldName) = stack_rows(values);
    end
end
end

function data = stack_rows(values)
% 各ステップの値を行ベクトルにして縦に積む。値の無いステップは NaN で埋め，次元が揃わない場合は cell 列のまま返す
values = values(:);
emptyIds = cellfun(@isempty, values);
valueNum = cellfun(@numel, values(~emptyIds));
if ~isempty(valueNum) && all(valueNum == valueNum(1)) && all(cellfun(@(value) isnumeric(value) || islogical(value), values(~emptyIds)))
    values(emptyIds) = {nan(1, valueNum(1))};
    data = cell2mat(cellfun(@(value) double(value(:))', values, "UniformOutput", false));
else
    data = values;
end
end

function value = get_field_or_empty(res, fieldName)
value = [];
if isstruct(res) && isfield(res, fieldName)
    value = res.(fieldName);
end
end

function value = get_state_or_empty(state, stateName)
value = [];
if isa(state, "STATE_CLASS") && isprop(state, stateName)
    value = state.(stateName);
end
end

function AL = expand_cha_allocation(agent)
% SimExp.set_cha_allocation 相当
propList = ["sensor", "estimator", "controller", "reference", "input_transform", "plant"];
AL = struct();
for cha = ['a', 't', 'f', 'l']
    for prop = propList
        AL.(cha).(prop) = [];
    end
end
if isprop(agent, "cha_allocation")
    al = agent.cha_allocation;
    for prop = propList
        if isfield(al, prop)
            for cha = ['a', 't', 'f', 'l']
                AL.(cha).(prop) = al.(prop);
            end
        end
    end
    for cha = ['a', 't', 'f', 'l']
        if isfield(al, cha)
            for prop = propList
                if isfield(al.(cha), prop)
                    AL.(cha).(prop) = al.(cha).(prop);
                end
            end
        end
    end
end
end

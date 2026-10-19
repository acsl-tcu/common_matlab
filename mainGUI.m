%% 初期設定
% リポジトリ内のパスを追加する。
clear all
clc
cf = pwd;

if contains(mfilename('fullpath'), "mainGUI")
    cd(fileparts(mfilename('fullpath')));
else
    tmp = matlab.desktop.editor.getActive;
    cd(fileparts(tmp.Filename));
end

[~, tmp] = regexp(genpath('.'), '\.\\\.git.*?;', 'match', 'split');
cellfun(@(xx) addpath(xx), tmp, 'UniformOutput', false);
close all hidden; clear; clc;
userpath('clear');
%%
% 各処理の引数：時刻、動作段階、ログ、環境、機体一覧、機体番号。
clc
SimBaseMode = ["SimSuspendedLoad",  "SimCooperativeSuspendedLoad",  "SimSplitCooperateiveLoad",...
                "SimVoronoi",       "SimHL",                        "SimPointMass",...
                "SimVehicle",       "SimSuspendedLoad",             "SimFHL",...
                "SimFHL_Servo",     "SimLiDAR",                     "SimFT",...
                "SimEL",            "SimMPC_Koopman",               "SimMEC_K"];

ExpBaseMode = ["ExpSuspendedLoad",          "ExpCooperativeSuspendedLoad",  "ExpSuspendedLoadCoop",...
                "ExpTestMotiveConnection",  "ExpHL",                        "ExpFHL",...
                "ExpFHL_Servo",             "ExpFT",                        "ExpEL",...
                "ExpMPC_Koopman"];

Setting.fDebug = 1; % 1でデバッグ機能を有効にする。
Setting.PInterval = 0.6; % 緊急停止を確認する間隔 [s]
Setting.mode = SimBaseMode(1); % SimSuspendedLoad
% Setting.mode = SimBaseMode(2); % SimCoop
% Setting.mode = SimBaseMode(5); % SimHL
% % Setting.mode = SimBaseMode(7); % SimFHL
Setting.mode = SimBaseMode(15); % SimMEC_K

% Setting.mode = ExpBaseMode(1); % ExpSuspendedLoad
% Setting.mode = ExpBaseMode(4); % ExpTestMotiveConnection
% Setting.mode = ExpBaseMode(5); % ExpHL

Setting.mec_dataset = mec_dataset_settings();
% MEC・入力励振・外力・手動/自動の切替はmec_dataset_settings.mにまとめる。
Setting.trajectory_seed = Setting.mec_dataset.manual_seed;
Setting.fAutoTrials = Setting.mec_dataset.auto_trials;


if Setting.mec_dataset.enabled
    Setting.trajectory_seeds = Setting.mec_dataset.seeds;
end

if contains(Setting.mode, "Exp")
    Setting.fExp = 1;
else
    Setting.fExp = 0;
end

app = SimExp(Setting);
if Setting.fAutoTrials
    if Setting.mec_dataset.enabled
        app.run_auto_trials(numel(Setting.trajectory_seeds),"MEC_dataset");
    else
        app.run_auto_trials(1, ""); % 通常の評価シミュレーション
    end
end

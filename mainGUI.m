%% Initialize settings
% set path
clear all
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
% each method's arguments : app.time,app.cha,app.logger,app.env,app.agent,i
clc
SimBaseMode = ["SimSuspendedLoad","SimVoronoi", "SimHL","SimPointMass", "SimVehicle", "SimSuspendedLoad", "SimFHL", "SimFHL_Servo", "SimLiDAR", "SimFT", "SimMPC_Koopman_komatsu", "SimMPC_KMC_kyo","SimKQ_LMPC_EDMD"];
ExpBaseMode = ["ExpSuspendedLoad","ExpSuspendedLoadCoop","ExpTestMotiveConnection", "ExpHL", "ExpFHL", "ExpFHL_Servo", "ExpFT", "ExpEL", "ExpMPC_Koopman","ExpMPC_Koopman_kyo","ExpKQ_LMPC","ExpKQ_LMPC_EDMD"];

Setting.fDebug = 1; % 1: active : for debug function
Setting.PInterval = 0.6; % sec : poling interval for emergency stop

% Setting.mode = SimBRaseMode(13); % Sim
% Setting.mode = SimBaseMode(7); % SimFHL

Setting.mode = ExpBaseMode(12); % Exp
if contains(Setting.mode,"Exp")
    Setting.fExp = 1;
else
    Setting.fExp = 0;
end
app = SimExp(Setting);

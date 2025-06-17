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
SimBaseMode = ["SimSuspendedLoad","SimVoronoi", "SimHL","SimPointMass", "SimVehicle", "SimSuspendedLoad", "SimFHL", "SimFHL_Servo", "SimLiDAR", "SimFT", "SimEL", "SimMPC_Koopman"];
ExpBaseMode = ["ExpSuspendedLoad","ExpSuspendedLoadCoop","ExpTestMotiveConnection", "ExpHL", "ExpFHL", "ExpFHL_Servo", "ExpFT", "ExpEL", "ExpMPC_Koopman"];

Setting.fDebug = 1; % 1: active : for debug function
Setting.PInterval = 0.6; % sec : poling interval for emergency stop
Setting.mode = SimBaseMode(3); % SimHL
% Setting.mode = SimBaseMode(7); % SimFHL

% Setting.mode = ExpBaseMode(4); % ExpHL
if contains(Setting.mode,"Exp")
    Setting.fExp = 1;
else
    Setting.fExp = 0;
end
gui = SimExp(Setting);

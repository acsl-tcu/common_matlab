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
% SimBaseMode = ["SimSuspendedLoad", "SimCooperativeSuspendedLoad", "SimSplitCooperateiveLoad",...
%                 "SimVoronoi", "SimHL", "SimPointMass",...
%                 "SimVehicle", "SimSuspendedLoad", "SimFHL",...
%                 "SimFHL_Servo", "SimLiDAR", "SimFT",...
%                 "SimEL", "SimMPC_Koopman","SimSuspendedLoadHOCBFLoadzxylink","SimSuspendedLoadHOCBFLoadzlink","SimSuspendedLoadHOCBFLoadz","SimSuspendedLoadHOCBFLoadxy","SimSuspendedLoadHOCBFLoadxyzyaw","SimSuspendedLoadAvoidance"];
% SimBaseMode = ["SimSuspendedLoad", "SimCooperativeSuspendedLoad", "SimSplitCooperateiveLoad",...
%     "SimVoronoi", "SimHL", "SimPointMass",...
%     "SimVehicle", "SimSuspendedLoad", "SimFHL",...
%     "SimFHL_Servo", "SimLiDAR", "SimFT",...
%     "SimEL", "SimMPC_Koopman","SimSuspendedLoadHOCBFlinkxy","SimSuspendedLoadHOCBFlinkz","SimSuspendedLoadHOCBFlinkxyz","SimHLCBF","SimSuspendedLoadCBF","SimSuspendedLoadMPC_v2","Sim_Idea2_APF","Sim_Idea3_AttPriority","Sim_Idea1_Backup","Sim_Idea3_RealCBF","Sim_Idea4_VirtualCBF","Sim_SmoothInputCBF"];
SimBaseMode = ["SimSuspendedLoad_RotorLevel","Sim_SmoothInputCBF","Sim_Avoid_Sliding","Sim_Avoid_Fluid","Sim_Avoid_EgoBand","Sim_Avoid_MPC_Traj","Sim_SmoothInputCBF","SimSuspendedLoadCBF","Sim_Idea2_APF","SimSuspendedLoadMultiSphereCBF"];
ExpBaseMode = ["ExpSuspendedLoad", "ExpCooperativeSuspendedLoad", "ExpSuspendedLoadCoop",...
                "ExpTestMotiveConnection", "ExpHL","ExpFHL", "ExpFHL_Servo",...
                "ExpFT", "ExpEL", "ExpMPC_Koopman"];

Setting.fDebug = 1; % 1: active : fdotiraor debug function
Setting.PInterval = 0.6; % sec : poling interval for emergency stop
% Setting.mode = SimBaseMode(1); % SimSuspendedLoad
% Setting.mode = SimBaseMode(19); % SimSuspendedLoad
% Setting.mode = SimBaseMode(21); % SimSuspendedLoad
% Setting.mode = SimBaseMode(25); % SimSuspendedLoad
Setting.mode = SimBaseMode(1); % CBF
% Setting.mode = SimBaseMode(19); % CBF
% Setting.mode = SimBaseMode(2); % SimCoop
% Setting.mode = SimBaseMode(5); % SimHL
% Setting.mode = SimBaseMode(7); % SimFHL

% Setting.mode = ExpBaseMode(1); % ExpSuspendedLoad
% Setting.mode = ExpBaseMode(4); % ExpTestMotiveConnection
% Setting.mode = ExpBaseMode(5); % ExpHL
if contains(Setting.mode, "Exp")
    Setting.fExp = 1;
else
    Setting.fExp = 0;
end

app = SimExp(Setting);

tmp = matlab.desktop.editor.getActive;
dir = fileparts(tmp.Filename);
if ~contains(path,dir)
    cd(erase(dir,'\mode'));
[~, tmp] = regexp(genpath('.'), '\.\\\.git.*?;', 'match', 'split');
cellfun(@(xx) addpath(xx), tmp, 'UniformOutput', false);
close all hidden; clear ; clc;
userpath('clear');
end

%%
ts = 0; % initial time
dt = 0.025; % sampling period
te = 50; % terminal time
time = TIME(ts,dt,te); % instance of time class
in_prog_func = @(app) dfunc(app); % in progress plot
post_func = @(app) dfunc(app); % function working at the "draw button" pushed.
motive = Connector_Natnet_sim(dt); % imitation of Motive camera (motion capture system)
logger = LOGGER(1, size(ts:dt:te, 2), 0, [],[]); % instance of LOOGER class for data logging
initial_state.p = arranged_position([0, 0], 1, 1, 0);
initial_state.q = [1; 0; 0; 0];
initial_state.v = [0; 0; 0];
initial_state.w = [0; 0; 0];


agent = DRONE;
agent.parameter = DRONE_PARAM("DIATONE");
agent.parameter.parameter(6:8) = 0.02;
% agent.parameter.parameter(6) = 0.0124;
% agent.parameter.parameter(7) = 0.0130;
% agent.parameter.parameter(8) = 0.0237;

% プラントモデル定義 ================================================================================================================================
plant = Model_Quat13(dt, initial_state, 1);
% plant.param.method = "euler_parameter_thrust_force_physical_parameter_model";
agent.plant = MODEL_CLASS(agent, plant);
% agent.plant = MODEL_CLASS(agent,Model_EulerAngle(dt, initial_state, 1));

% デフォルト物理パラメータ(DRONE_PARAM.m準拠: 2025/07/07時点)
% 1:mass=0.75  |  2,3:Lx,y=0.16  |  4,5: lx,y=0.08  |  6,7,8: jx,y,z=0.06  |  9: gravity=9.81
% 10,11,12,13: km(各ロータ定数)=0.0301  |  14,15,16,17: k(推力定数)=8.0e-6  |  18: rotor_r=0.0392

% ↓パラメータの上書き モデル誤差をプラントに与える
% agent.plant.param(1) = 0.7875; % ５％増->0.7875, ５％減->0.7125
% % agent.plant.param(1) = 0.7125;
% agent.plant.param(4) = 0.07;
% agent.plant.param(5) = 0.07;
% agent.plant.param(6) = 0.22; % 0.18<jx,jy<0.22ぐらいが良き frequency=5の時
% agent.plant.param(7) = 0.22; % 同上
% agent.plant.param(10:13) = 0.003;
% agent.plant.param(10) = 0.0301*0.3;
% agent.plant.param(13) = 0.0301*0.3;
%===================================================================================================================================================
if contains(func2str(agent.plant.method), 'force')
    EKF_model               = Model_EulerAngle(dt, initial_state, 1);
    EKF_model.param.method  = "roll_pitch_yaw_thrust_force_physical_parameter_model";
    agent.estimator = EKF(agent, Estimator_EKF(agent,dt,MODEL_CLASS(agent,EKF_model),["p", "q"]));
else, agent.estimator = EKF(agent, Estimator_EKF(agent,dt,MODEL_CLASS(agent,Model_EulerAngle(dt, initial_state, 1)),["p", "q"]));
end
if contains(func2str(agent.plant.method), 'force'), agent.controller = HLC_THRUST_FORCE(agent, Controller_HL(dt));
else                                              , agent.controller = HLC(agent,Controller_HL(dt)); end
agent.sensor    = MOTIVE(agent, Sensor_Motive(1,0, motive));
run("ExpBase");
takeoff_zd = 1; % だいたい1m
agent.reference.takeoff.zd = takeoff_zd;
center = [0;0;takeoff_zd];
% agent.reference.time_varying = TIME_VARYING_REFERENCE(agent,{"gen_ref_saddle",{"freq",5,"orig",center,"size",[0,0,0]},"HL"});                       % center hovering
% agent.reference.time_varying = TIME_VARYING_REFERENCE(agent,{"gen_ref_saddle",{"freq",5,"orig",[1;1;takeoff_zd],"size",[0,0,0]},"HL"});         % point hovering
agent.reference.time_varying = TIME_VARYING_REFERENCE(agent,{"gen_ref_saddle",{"freq",5,"orig",center,"size",[1,1,0]},"HL"});                       % circle
% agent.reference.time_varying = TIME_VARYING_REFERENCE(agent,{"gen_ref_saddle",{"freq",10,"orig",center,"size",[1,1,0.2]},"HL"});                    % saddle
% agent.reference.time_varying = TIME_VARYING_REFERENCE(agent,{"gen_ref_spline",{"point",20, "order",9, "point_dt",3, "ManualSetting",0, "check",1}});  % random 9th spline
% agent.reference.time_varying = MY_POINT_REFERENCE(agent, {struct("f", center, "g", [1;0;takeoff_zd], "h",center, "j",[0;1;takeoff_zd], "k",center, "z",[0;0;takeoff_zd+1], "x",center...
%                                                                 , "c",[-1;-1;takeoff_zd], "v",center, "b",[1;-1;takeoff_zd+1], "n",center), 7.5});  % P2P
% agent.reference.time_varying = MY_POINT_REFERENCE(agent, {struct("f",center, "g",[1;0;takeoff_zd], "h",[1;1;takeoff_zd], "j",center, "k",[0;0;takeoff_zd+0.5]), 7.5}); % P2P

agent.cha_allocation.reference = "time_varying";
motive.getData(agent);

function dfunc(app)
LW = 1.5; % Linewidth 
FS = 16; % Fontsize
phase = "tfl";
% phase = "f";
% phase = "t";
app.logger.plot({1, "p", "er"},"ax",app.UIAxes,"phase",phase, "Linewidth",LW, "Fontsize",FS);
app.logger.plot({1, "p", "er"}, "phase",phase, "fig_num",1, "Linewidth",LW, "Fontsize",FS);
app.logger.plot({1, "q", "e"}, "phase",phase, "fig_num",2, "Linewidth",LW, "Fontsize",FS);
app.logger.plot({1, "v", "er"}, "phase",phase, "fig_num",3, "Linewidth",LW, "Fontsize",FS);
app.logger.plot({1, "w", "e"}, "phase",phase, "fig_num",4, "Linewidth",LW, "Fontsize",FS);
app.logger.plot({1, "input", ""}, "phase",phase, "fig_num",6, "Linewidth",LW, "Fontsize",FS);
app.logger.plot({1, "controller.result.before_input", ""}, "phase",phase, "fig_num",7, "Linewidth",LW, "Fontsize",FS);

app.logger.plot({1, "p1-p2", "er"}, "phase",phase, "color", 0, "fig_num",8, "Linewidth",LW, "Fontsize",FS);
app.logger.plot({1, "p1-p2-p3", "er"}, "phase",phase, "color", 0, "fig_num",9, "Linewidth",LW, "Fontsize",FS);

end
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
% in_prog_func = @(app) dfunc(app); % in progress plot
post_func = @(app) dfunc(app); % function working at the "draw button" pushed.
% motive = Connector_Natnet_sim(1, dt); % imitation of Motive camera (motion capture system)
logger = LOGGER(1, size(ts:dt:te, 2), 0, [],[]); % instance of LOOGER class for data logging
initial_state.p = arranged_position([0, 0], 1, 1, 0);
initial_state.q = [1; 0; 0; 0];
initial_state.v = [0; 0; 0];
initial_state.w = [0; 0; 0];

agent = DRONE;
agent.parameter = DRONE_PARAM("DIATONE"); % プラントでModel_EulerAngleを使うときはノミナルモデル

% プラントモデル定義 ================================================================================================================================
plant = Model_Quat13(dt, initial_state, 1);
% plant.param.method = "euler_parameter_thrust_force_physical_parameter_model";
% plant.param.dim = [13,4,18];
agent.plant = MODEL_CLASS(agent, plant);
% agent.plant = MODEL_CLASS(agent,Model_EulerAngle(dt, initial_state, 1));

% デフォルト物理パラメータ(DRONE_PARAM.m準拠: 2025/07/07時点)
% 1:mass=0.75  |  2,3:Lx,y=0.16  |  4,5: lx,y=0.08  |  6,7,8: jx,y,z=0.06  |  9: gravity=9.81
% 10,11,12,13: km(各ロータ定数)=0.0301  |  14,15,16,17: k(推力定数)=8.0e-6  |  18: rotor_r=0.0392

% ↓パラメータの上書き モデル誤差をプラントに与える
% agent.plant.param(1) = 0.7875; % ５％増->0.7875, ５％減->0.7125
% agent.plant.param(1) = 0.7125;
% agent.plant.param(6) = 0.14; % 0.18<jx,jy<0.22ぐらいが良き frequency=5の時
% agent.plant.param(7) = 0.14; % 同上
% agent.plant.param(10:13) = [0.003, 0.003, 0.003, 0.003];
% agent.plant.param(4) = 0.07;
% agent.plant.param(10) = 0.003;
%===================================================================================================================================================
agent.estimator = EKF(agent, Estimator_EKF(agent,dt,MODEL_CLASS(agent,Model_EulerAngle(dt, initial_state, 1)),["p", "q"]));
if contains(func2str(agent.plant.method), 'force') % 'thrust_force modelを使っている場合に上書き
    EKF_model               = Model_EulerAngle(dt, initial_state, 1);
    EKF_model.param.method  = "roll_pitch_yaw_thrust_force_physical_parameter_model";
    agent.estimator = EKF(agent, Estimator_EKF(agent,dt,MODEL_CLASS(agent,EKF_model),["p", "q"]));
end
agent.sensor = DIRECT_SENSOR(agent, 0.0); % modeファイル内で回すとき

run("ExpBase");
takeoff_zd = 1; % だいたい1m
agent.reference.takeoff.zd = takeoff_zd;
center = [0;0;takeoff_zd];
% agent.reference.time_varying = TIME_VARYING_REFERENCE(agent,{"gen_ref_saddle",{"freq",5,"orig",center,"size",[0,0,0]},"HL"});                       % center hovering
% agent.reference.time_varying = TIME_VARYING_REFERENCE(agent,{"gen_ref_saddle",{"freq",5,"orig",[-1;-1;takeoff_zd],"size",[0,0,0]},"HL"});           % point hovering
% agent.reference.time_varying = TIME_VARYING_REFERENCE(agent,{"gen_ref_saddle",{"freq",5,"orig",center,"size",[1,1,0]},"HL"});                       % circle
% agent.reference.time_varying = TIME_VARYING_REFERENCE(agent,{"gen_ref_triangle",{"freq",10,"orig",center,"size",1.0},"HL"});                        % triangle
agent.reference.time_varying = TIME_VARYING_REFERENCE(agent,{"gen_ref_flower",{"freq",10,"orig",center,"radius",1.0},"HL"});                        % flower
% agent.reference.time_varying = TIME_VARYING_REFERENCE(agent,{"gen_ref_heart",{"freq",10,"orig",center,"size",1.0},"HL"});                           % heart
% agent.reference.time_varying = TIME_VARYING_REFERENCE(agent,{"gen_ref_star",{"freq",15,"orig",center,"radius",1.0},"HL"});                          % star
% agent.reference.time_varying = TIME_VARYING_REFERENCE(agent,{"gen_ref_saddle",{"freq",10,"orig",center,"size",[1,1,0.2]},"HL"});                    % saddle
% agent.reference.time_varying = TIME_VARYING_REFERENCE(agent,{"gen_ref_spline",{"point",20,"order",9,"point_dt",2.5,"ManualSetting",0,"check",1}});  % random 9th spline
% agent.reference.time_varying = MY_POINT_REFERENCE(agent, {struct("f", center, "g", [1;0;takeoff_zd], "h",center, "j",[0;1;takeoff_zd], "k",center, "z",[0;0;takeoff_zd+1], "x",center...
%                                                                 , "c",[-1;-1;takeoff_zd], "v",center, "b",[1;-1;takeoff_zd+1], "n",center), 7.5});  % P2P
% agent.reference.time_varying = MY_POINT_REFERENCE(agent, {struct("f", center, "g", [-1;-1;takeoff_zd]), 10});                                       % P2P
% agent.reference.time_varying = MY_POINT_REFERENCE(agent, {struct("f",center, "g",[1;0;takeoff_zd], "h",[1;1;takeoff_zd], "j",center, "k",[0;0;takeoff_zd+0.5]), 7.5}); % P2P

agent.cha_allocation.reference = "time_varying";

if contains(func2str(agent.plant.method), 'force')
    % agent.controller.mec = DNNMEC_THRUST_FORCE(agent, "Step_4_DNNMEC_epoch_100000.onnx");
    % agent.controller.mec = DNNMEC_THRUST_FORCE(agent, "Sim_Data_DNNMEC_epoch_100000.onnx");
    % agent.controller.mec = DNNMEC_THRUST_FORCE(agent, "Sim_mixed_Data_DNNMEC_epoch_30000.onnx");
    % agent.controller.mec = DNNMEC_THRUST_FORCE(agent, "DNNMEC_Exp_data_epoch_100000.onnx");
    % agent.controller.mec = DNNMEC_THRUST_FORCE(agent, "Exp_data_DNNMEC_epoch_100000_e-6_0.001_0.001_0.8.onnx"); % <-jx,jy=0.18で暴れて性能劣化
    % agent.controller.mec = DNNMEC_THRUST_FORCE(agent, "Exp_data_No_coef_DNNMEC_epoch_100000.onnx");
else
    % agent.controller.mec = DNNMEC(agent, "Step_4_DNNMEC_epoch_100000.onnx");
    % agent.controller.mec = DNNMEC(agent, "Sim_Data_DNNMEC_epoch_100000.onnx");
    % agent.controller.mec = DNNMEC(agent, "Sim_mixed_Data_DNNMEC_epoch_30000.onnx");
    % agent.controller.mec = DNNMEC(agent, "DNNMEC_Exp_data_epoch_100000.onnx");
    % agent.controller.mec = DNNMEC(agent, "Exp_data_DNNMEC_epoch_100000_e-6_0.001_0.001_0.8.onnx"); % <-jx,jy=0.18で暴れて性能劣化
    agent.controller.mec = DNNMEC(agent, "Exp_data_No_coef_DNNMEC_epoch_100000.onnx");
    % agent.controller.mec = DNNMEC(agent, "z_state_coef_No_losscoef_DNNMEC_epoch_100000.onnx");
    % agent.controller.mec = DNNMEC(agent, "delta_u_step=1_No_losscoef_DNNMEC_epoch_100000.onnx");
end

if contains(func2str(agent.plant.method), 'force'), agent.controller.nominal = HLC_THRUST_FORCE(agent, Controller_HL(dt));
else                                              , agent.controller.nominal = HLC(agent,Controller_HL(dt)); end
agent.cha_allocation.controller=["nominal","mec"]; % cha_allocationにコントローラー登録

function dfunc(app)
LW = 1.5; % LineWidth
FS = 18; % FontSize
phase = "tfl";
% phase = "tf";
% phase = "f";
app.logger.plot({1, "p", "er"},"ax",app.UIAxes, "phase",phase, "fig_num",1, "Linewidth",LW, "Fontsize",FS);
app.logger.plot({1, "p", "er"}, "phase",phase, "fig_num",1, "Linewidth",LW, "Fontsize",FS);
app.logger.plot({1, "q", "e"}, "phase",phase, "fig_num",2, "Linewidth",LW, "Fontsize",FS);
app.logger.plot({1, "v", "er"}, "phase",phase, "fig_num",3, "Linewidth",LW, "Fontsize",FS);
app.logger.plot({1, "w", "e"}, "phase",phase, "fig_num",4, "Linewidth",LW, "Fontsize",FS);
% app.logger.plot({{1, "input", ""}, {1, "controller.result.nominal_input", ""},...
%     {1, "controller.result.delta_input", ""}}, "phase",phase,"fig_num",5); % inputをまとめて見る
app.logger.plot({1, "input", ""}, "phase",phase, "fig_num",6, "Linewidth",LW, "Fontsize",FS);
app.logger.plot({1, "controller.result.nominal_input", ""}, "phase",phase, "fig_num",7, "Linewidth",LW, "Fontsize",FS);
app.logger.plot({1, "controller.result.delta_input", ""}, "phase",phase, "fig_num",8, "Linewidth",LW, "Fontsize",FS);
app.logger.plot({1, "p1-p2", "er"}, "phase",phase, "color", 0, "fig_num",9, "Linewidth",LW, "Fontsize",FS);
% app.logger.plot({1, "p1-p2-p3", "er"}, "phase",phase, "color", 0, "fig_num",10, "Linewidth",LW, "Fontsize",FS);

% app.logger.plot({{1, "p", "e"},{1, "controller.result.nominal_p", "p"}}, "phase",phase, "fig_num",11, "Linewidth",LW, "Fontsize",FS);
% app.logger.plot({{1, "q", "e"},{1, "controller.result.nominal_q", "p"}}, "phase",phase, "fig_num",12, "Linewidth",LW, "Fontsize",FS);
% app.logger.plot({{1, "v", "e"},{1, "controller.result.nominal_v", "p"}}, "phase",phase, "fig_num",13, "Linewidth",LW, "Fontsize",FS);
% app.logger.plot({{1, "w", "e"},{1, "controller.result.nominal_w", "p"}}, "phase",phase, "fig_num",14, "Linewidth",LW, "Fontsize",FS);

% Calcurate RMSE
target = ["p", "v"];
RMSE = [];
for i=1:length(target)
    ref = app.logger.data(1,target(i),"r", "phase",phase);
    data = app.logger.data(1,target(i),"e", "phase",phase);
    RMSE = [RMSE; rmse(ref, data, 1)];
    fprintf('%s RMSE:\n', target(i))
    disp(RMSE(i,:))
    disp(sum(RMSE(i,:)))
end

% data = app.logger.data(1,"p","e","phase","f");
% min_z = min(data(:,3))
% max_z = max(data(:,3))
end
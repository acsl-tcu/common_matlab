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
% agent.plant = MODEL_CLASS(agent,Model_EulerAngle(dt, initial_state, 1));
agent.plant = MODEL_CLASS(agent,Model_Quat13(dt, initial_state, 1)); % Model_Quat13

% デフォルト物理パラメータ(DRONE_PARAM.m準拠: 2025/07/07時点)
% 1:mass=0.75  |  2,3:Lx,y=0.16  |  4,5: lx,y=0.08  |  6,7,8: jx,y,z=0.06  |  9: gravity=9.81
% 10,11,12,13: km(各ロータ定数)=0.0301  |  14,15,16,17: k(推力定数)=8.0e-6  |  18: rotor_r=0.0392

% ↓パラメータの上書き モデル誤差をプラントに与える
agent.plant.param(1) = 0.7875; % ５％増->0.7875, ５％減->0.7125
agent.plant.param(6) = 0.2; % 0.18<jx,jy<0.22ぐらいが良き frequency=5の時
agent.plant.param(7) = 0.2; % 同上
% agent.plant.param(8) = 0.36; % 0.18<jzぐらいが良き
% agent.plant.param(6) = 0.18; % 0.1<jx,jy<0.12 frequency=2.5の時
% agent.plant.param(7) = 0.18; % 
% agent.plant.param(8) = 0.1; % 
% 2~5,10~18はagent.plant.method='@roll_pitch_yaw_thrust_torque_physical_parameter_model'を使っている限り意味が無い
%===================================================================================================================================================

agent.sensor = DIRECT_SENSOR(agent, 0.0); % modeファイル内で回すとき
agent.estimator = EKF(agent, Estimator_EKF(agent,dt,MODEL_CLASS(agent,Model_EulerAngle(dt, initial_state, 1)),["p", "q"]));

run("ExpBase");
takeoff_zd = agent.reference.takeoff.zd; % だいたい1m
center = [0;0;takeoff_zd];
agent.reference.time_varying = TIME_VARYING_REFERENCE(agent,{"gen_ref_saddle",{"freq",5,"orig",center,"size",[1,1,0]},"HL"}); % circle
% agent.reference.time_varying = TIME_VARYING_REFERENCE(agent,{"gen_ref_saddle",{"freq",5,"orig",center,"size",[0,0,0]},"HL"}); % hovering
% agent.reference.time_varying = TIME_VARYING_REFERENCE(agent,{"gen_ref_saddle",{"freq",5,"orig",center,"size",[1,1,0.2]},"HL"}); % saddle
% agent.reference.time_varying = TIME_VARYING_REFERENCE(agent,{"gen_ref_spline",{"point",20,"order",9,"point_dt",2.5,"ManualSetting",0,"check",1}}); % random 9th spline
% agent.reference.time_varying = MY_POINT_REFERENCE(agent, {struct("f", center, "g", [1;0;takeoff_zd], "h",center, "j",[0;1;takeoff_zd], "k",center, "z",[0;0;takeoff_zd+1], "x",center...
%                                                                 , "c",[-1;-1;takeoff_zd], "v",center, "b",[1;-1;takeoff_zd+1], "n",center), 7.5}); % P2P
% agent.reference.time_varying = MY_POINT_REFERENCE(agent, {struct("f", center, "g", [-1;-1;takeoff_zd]), 10}); % P2P

agent.cha_allocation.reference = "time_varying";

% agent.controller.mec = DNNMEC_BEHIND(agent, "DNNMEC_epoch_1000000_0.0005_0.01_0.01_0.7.onnx"); % 中間報告会でメイン使用したもの
% agent.controller.mec = DNNMEC_BEHIND(agent, "DNNMEC_epoch_20000_0.0005_0.01_0.01_0.7.onnx");

% この重みの方が直感的に分かりやすい気がする
fMEC = 0;
% fMEC = 1;
% agent.controller.mec = DNNMEC_BEHIND(agent, "DNNMEC_epoch_1000000_0.001_0.01_0.01_0.1.onnx",fMEC); % 100万epoch
% agent.controller.mec = DNNMEC_BEHIND(agent, "DNNMEC_epoch_20000_0.001_0.01_0.01_0.1.onnx",fMEC); % 2万epoch（中間報告書に記載）


% % ↓2025/08/25 お試し
% agent.controller.mec = DNNMEC_BEHIND(agent, "DNNMEC_epoch_1000000_0.00001_0.01_0.01_0.8.onnx");
% agent.controller.mec = DNNMEC_BEHIND(agent, "DNNMEC_epoch_900000_0.00001_0.01_0.01_0.8.onnx");
% agent.controller.mec = DNNMEC(agent, "DNNMEC_epoch_800000_0.00001_0.01_0.01_0.8.onnx");
%        ↑↑↑ thrust過剰? Flightフェーズになると始めだけ暴れる．そのあとは収束
% agent.controller.mec = DNNMEC_BEHIND(agent, "DNNMEC_epoch_330000_0.00001_0.01_0.01_0.8.onnx");


% % 2025/09/18 Step数変更
% agent.controller.mec = DNNMEC(agent, "Step_1_DNNMEC_epoch_100000.onnx");
% agent.controller.mec = DNNMEC(agent, "Step_2_DNNMEC_epoch_100000.onnx");
% agent.controller.mec = DNNMEC(agent, "Step_3_DNNMEC_epoch_100000.onnx");
% agent.controller.mec = DNNMEC(agent, "Step_4_DNNMEC_epoch_100000.onnx");
% agent.controller.mec = DNNMEC(agent, "Step_5_DNNMEC_epoch_100000.onnx");
% agent.controller.mec = DNNMEC(agent, "Step_6_DNNMEC_epoch_100000.onnx");


% agent.controller.mec = DNNMEC(agent, "Sim_Data_DNNMEC_epoch_100000.onnx");
% agent.controller.mec = DNNMEC(agent, "Sim_mixed_Data_DNNMEC_epoch_30000.onnx");

agent.controller.nominal = HLC(agent,Controller_HL(dt));
agent.cha_allocation.controller=["nominal","mec"]; % cha_allocationにコントローラー登録

function dfunc(app)
LW = 1.5; % LineWidth
FS = 18; % FontSize
phase = "tfl";
% phase = "tf";
phase = "f";
app.logger.plot({1, "p", "er"},"ax",app.UIAxes, "phase",phase, "fig_num",1, "Linewidth",LW, "Fontsize",FS);
app.logger.plot({1, "p", "er"}, "phase",phase, "fig_num",1, "Linewidth",LW, "Fontsize",FS);
app.logger.plot({1, "q", "e"}, "phase",phase, "fig_num",2, "Linewidth",LW, "Fontsize",FS);
app.logger.plot({1, "v", "er"}, "phase",phase, "fig_num",3, "Linewidth",LW, "Fontsize",FS);
app.logger.plot({1, "w", "e"}, "phase",phase, "fig_num",4, "Linewidth",LW, "Fontsize",FS);
% app.logger.plot({{1, "input", ""}, {1, "controller.result.nominal_input", ""},...
%     {1, "controller.result.delta_input", ""}}, "phase",phase,"fig_num",5); % inputをまとめて見る
app.logger.plot({1, "input", ""}, "phase",phase, "fig_num",6, "Linewidth",LW, "Fontsize",FS);
app.logger.plot({1, "controller.result.nominal_input", ""}, "phase",phase, "fig_num",7, "Linewidth",LW, "Fontsize",FS);
if class(app.agent.controller.mec)=="DNNMEC_BEHIND",    app.logger.plot({1, "controller.result.mec_input", ""}, "phase","f", "fig_num",8, "Linewidth",LW, "Fontsize",FS);
elseif class(app.agent.controller.mec)=="DNNMEC",       app.logger.plot({1, "controller.result.delta_input", ""}, "phase","f", "fig_num",8, "Linewidth",LW, "Fontsize",FS); end
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
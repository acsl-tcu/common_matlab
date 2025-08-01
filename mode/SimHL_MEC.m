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
plant_model = Model_EulerAngle(dt, initial_state, 1);
% デフォルト物理パラメータ(DRONE_PARAM準拠: 2025/07/07時点)
% 1:mass=0.75  |  2,3:Lx,y=0.16  |  4,5: lx,y=0.08  |  6,7,8: jx,y,z=0.06  |  9: gravity=9.81
% 10,11,12,13: km(各ロータ定数)=0.0301  |  14,15,16,17: k(推力定数)=8.0e-6  |  18: rotor_r=0.0392

% ↓パラメータの上書き モデル誤差をプラントに与える
plant_model.param.param(1) = 0.7875; % ５％減->0.7125 ５％増->0.7875
plant_model.param.param(6) = 0.2; % 0.18<jx,jy<0.22ぐらいが良き
plant_model.param.param(7) = 0.2; % 同上
plant_model.param.param(8) = 0.2; % 0.18<jzぐらいが良き
% plant_model.param.param(10) = 0.6; % ５％減->0.028595
% plant_model.param.param(13) = 0.3;
% plant_model.param.param(14) = 0.008;
% 2~5,10~18はplant_model.param.method='roll_pitch_yaw_thrust_torque_physical_parameter_model'を使っている限り意味が無い

agent.plant = MODEL_CLASS(agent,plant_model);
% agent.plant = MODEL_CLASS(agent,Model_Quat13(dt, initial_state, 1)); % Model_Quat13
%===================================================================================================================================================

agent.sensor = DIRECT_SENSOR(agent, 0.0); % modeファイル内で回すとき
agent.estimator = EKF(agent, Estimator_EKF(agent,dt,MODEL_CLASS(agent,Model_EulerAngle(dt, initial_state, 1)),["p", "q"]));

agent.reference.time_varying = TIME_VARYING_REFERENCE(agent,{"gen_ref_saddle",{"freq",5,"orig",[0;0;1],"size",[1,1,0]},"HL"}); % circle
% agent.reference.time_varying = TIME_VARYING_REFERENCE(agent,{"gen_ref_saddle",{"freq",5,"orig",[0;0;1],"size",[0,0,0]},"HL"}); % hovering
% agent.reference.time_varying = TIME_VARYING_REFERENCE(agent,{"gen_ref_saddle",{"freq",5,"orig",[0;0;1],"size",[1,1,0.2]},"HL"}); % saddle
% agent.reference.time_varying = TIME_VARYING_REFERENCE(agent,{"gen_ref_spline",{"point",20,"order",9,"point_dt",2.5,"ManualSetting",0,"check",1}}); % random 9th spline
run("ExpBase");
agent.cha_allocation.reference = "time_varying";

agent.controller.nominal = HLC(agent,Controller_HL(dt));
agent.controller.mec = DNNMEC(agent, "DNNMEC_epoch_1000000.onnx");
agent.cha_allocation.controller=["nominal","mec"]; % cha_allocationにコントローラー登録

function dfunc(app)
LW = 2; % LineWidth
FS = 24; % FontSize
% phase = "tfl";
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
app.logger.plot({1, "controller.result.delta_input", ""}, "phase",phase, "fig_num",8, "Linewidth",LW, "Fontsize",FS);

app.logger.plot({1, "p1-p2", "er"}, "phase",phase, "color", 0, "fig_num",9, "Linewidth",LW, "Fontsize",FS);
% app.logger.plot({1, "p1-p2-p3", "er"}, "phase",phase, "color", 0, "fig_num",10, "Linewidth",LW, "Fontsize",FS);

% app.logger.plot({{1, "p", "e"},{1, "controller.result.nominal_p", "p"}}, "phase",phase, "fig_num",11, "Linewidth",LW, "Fontsize",FS);
% app.logger.plot({{1, "q", "e"},{1, "controller.result.nominal_q", "p"}}, "phase",phase, "fig_num",12, "Linewidth",LW, "Fontsize",FS);
% app.logger.plot({{1, "v", "e"},{1, "controller.result.nominal_v", "p"}}, "phase",phase, "fig_num",13, "Linewidth",LW, "Fontsize",FS);
% app.logger.plot({{1, "w", "e"},{1, "controller.result.nominal_w", "p"}}, "phase",phase, "fig_num",14, "Linewidth",LW, "Fontsize",FS);

% Calcurate RMSE
target = ["p", "v"];
for i=1:length(target)
    ref = app.logger.data(1,target(i),"r", "phase",phase);
    data = app.logger.data(1,target(i),"e", "phase",phase);
    RMSE = rmse(ref, data, 1);
    fprintf('%s RMSE:\n', target(i))
    disp(RMSE)
end
end
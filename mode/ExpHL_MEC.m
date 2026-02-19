ts = 0; % initial time
dt = 0.025; % sampling period
te = 10000; % termina time
time = TIME(ts,dt,te);
in_prog_func = @(app) in_prog(app);
post_func = @(app) post(app);
logger = LOGGER(1, size(ts:dt:te, 2), 1, [],[]);

motive = Connector_Natnet('192.168.100.4'); % connect to Motive
% motive = Connector_Natnet('192.168.120.3'); % connect to Motive（総研）
motive.getData([], []); % get data from Motive
rigid_ids = [1]; % rigid-body number on Motive
sstate = motive.result.rigid(rigid_ids);
initial_state.p = sstate.p;
initial_state.q = sstate.q;
initial_state.v = [0; 0; 0];
initial_state.w = [0; 0; 0];

agent = DRONE;
% agent.plant = DRONE_EXP_MODEL(agent,Model_Drone_Exp(dt, initial_state, "udp", )[1, 252]));
agent.plant = DRONE_EXP_MODEL(agent,Model_Drone_Exp(dt, initial_state, "serial", "COM4"));
agent.parameter = DRONE_PARAM("DIATONE");
agent.estimator = EKF(agent, Estimator_EKF(agent,dt,MODEL_CLASS(agent,Model_EulerAngle(dt, initial_state, 1)),["p", "q"]));
agent.sensor = MOTIVE(agent, Sensor_Motive(1,0, motive));
agent.input_transform = THRUST2THROTTLE_DRONE(agent,InputTransform_Thrust2Throttle_drone()); % 推力からスロットルに変換

run("ExpBase");

takeoff_zd = 1; % だいたい1m
agent.reference.takeoff.zd = takeoff_zd; % referenceクラスの目標高度zd書き換え
center = [0;0;takeoff_zd];
% agent.reference.time_varying = TIME_VARYING_REFERENCE(agent,{"gen_ref_saddle",{"freq",10,"center",center,"radius",[0,0,0]},"HL"});                      % center hovering
% agent.reference.time_varying = TIME_VARYING_REFERENCE(agent,{"gen_ref_saddle",{"freq",5,"center",center,"radius",[1,0,0], "phase",-pi/2},"HL"});        % x sin curve
% agent.reference.time_varying = TIME_VARYING_REFERENCE(agent,{"gen_ref_saddle",{"freq",5,"center",center,"radius",[0,1,0]},"HL"});                       % y sin curve
% agent.reference.time_varying = TIME_VARYING_REFERENCE(agent,{"gen_ref_saddle",{"freq",10,"center",[center+1],"radius",[0,0,0]},"HL"});          % point hovering-
% agent.reference.time_varying = TIME_VARYING_REFERENCE(agent,{"gen_ref_saddle",{"freq",-3,"center",center,"radius",[1,1,0]},"HL"});                       % circle
% agent.reference.time_varying = TIME_VARYING_REFERENCE(agent,{"gen_ref_circle_3D",{"freq",20,"orig",center,"size",[1,0], "phase",[-pi/2,0]},"HL"});       % 3D circle
% agent.reference.time_varying = TIME_VARYING_REFERENCE(agent,{"gen_ref_lemniscate",{"freq",-7.5,"orig",center,"radius",1, "x",1},"HL"});                 % lemniscate
% agent.reference.time_varying = TIME_VARYING_REFERENCE(agent,{"gen_ref_lemniscate_3D",{"freq",7.5,"orig",center,"size",[1,0.5], "x",1},"HL"});         % 3D lemniscate
% agent.reference.time_varying = TIME_VARYING_REFERENCE(agent,{"gen_ref_triangle",{"freq",10,"orig",center,"size",1.0},"HL"});                        % triangle
% agent.reference.time_varying = TIME_VARYING_REFERENCE(agent,{"gen_ref_flower",{"freq",15,"orig",center,"radius",1.0},"HL"});                        % flower
% agent.reference.time_varying = TIME_VARYING_REFERENCE(agent,{"gen_ref_heart",{"freq",10,"orig",center,"size",1.0},"HL"});                           % heart
% agent.reference.time_varying = TIME_VARYING_REFERENCE(agent,{"gen_ref_star",{"freq",15,"orig",center,"radius",1.0},"HL"});                          % star
agent.reference.time_varying = TIME_VARYING_REFERENCE(agent,{"gen_ref_saddle",{"freq",-4,"center",center,"radius",[1,1,0.25], "phase",-pi},"HL"});                    % saddle
% agent.reference.time_varying = TIME_VARYING_REFERENCE(agent,{"gen_ref_saddle_yaw",{"freq",-5,"center",center,"radius",[1,1,0.25], "yaw_rad",1, "phase",-pi},"HL"});   % saddle with yaw movement
% agent.reference.time_varying = TIME_VARYING_REFERENCE(agent,{"gen_ref_spline",{"point",20,"order",9,"point_dt",2.5,"ManualSetting",0,"check",1}});  % random 9th spline
% agent.reference.time_varying = MY_POINT_REFERENCE(agent, {struct("f", center, "g", [1;0;takeoff_zd], "h",center, "j",[0;1;takeoff_zd], "k",center, "z",[0;0;takeoff_zd+1], "x",center...
%                                                                 , "c",[-1;-1;takeoff_zd], "v",center, "b",[1;-1;takeoff_zd+1], "n",center), 7.5});  % P2P
% agent.reference.time_varying = MY_POINT_REFERENCE(agent, {struct("f",center, "g",center+1, "h",[center(1:2);1.5]), 10});                                       % P2P
% agent.reference.time_varying = MY_POINT_REFERENCE(agent, {struct("f",center, "g",[1;0;takeoff_zd], "h",[1;1;takeoff_zd], "j",center, "k",[0;0;takeoff_zd+0.5]), 7.5}); % P2P
% agent.reference.time_varying = MY_POINT_REFERENCE(agent, {struct("f",center, "g",[0;1;takeoff_zd], "h",[-1;1;takeoff_zd], "j",[1;1;takeoff_zd], "k",[0;1;takeoff_zd], "l",center), 10}); % P2P for wind
% agent.reference.time_varying = MY_POINT_REFERENCE(agent, {struct("f",center, "g",[0;-1;takeoff_zd], "h",[-1;-1;takeoff_zd], "j",[1;1;takeoff_zd]...
%                                                                 , "k",[-1;-1;takeoff_zd+0.5], "l",[-1;1;takeoff_zd-0.5], "z",center), 10}); % learge P2P
% agent.reference.time_varying = MY_POINT_REFERENCE(agent, {struct("f",center, "g",[0;0;takeoff_zd], "h",[1;1;takeoff_zd], "j",[1.5;1.5;takeoff_zd], "k",[2;2;takeoff_zd]...
%                                                             ,"l",[2.5;2.5;takeoff_zd], "z",[3;3;takeoff_zd], "x",[3.5;3.5;takeoff_zd], "c",[4;4;takeoff_zd]), 10});                                       % P2P

agent.cha_allocation.reference = "time_varying";
% Nominal Controller % % % % % % % % % % % % % % % % % % % % % % % % % % % %
agent.controller.nominal = HLC(agent,Controller_HL(dt));
% agent.controller.nominal = FUNCTIONAL_HLC_SERVO(agent, Controller_FHL_Servo(dt)); % 位置偏差に対するサーボ系HL
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %

% onnxName = "2025-11-11_12_35_26__DNN24__Plant_data_Exp__hidden=3__Euler__epoch_100000.onnx";
% onnxName = "2025-11-25_11_53_4__DNN24__Plant_data_Exp__hidden=1__Euler__epoch_100000.onnx";


onnxName = "2025-12-8_12_18_54__DNN21__Plant_data_Exp__Euler__hidden=3__epoch_100000.onnx"; % 本論に載せてるやつ(2026/02/09時点)
% onnxName = "2025-12-8_12_29_35__DNN21__Plant_data_Exp__Euler__hidden=1__epoch_100000.onnx";


% onnxName = "2026-1-29_18_0_49__DNN21__Plant_data_Exp__Euler__Step=1__epoch_100000.onnx";
% onnxName = "2026-1-30_10_26_11__DNN21__Plant_data_Exp__Euler__Step=2__epoch_100000.onnx";
% onnxName = "2026-2-2_10_49_9__DNN21__Plant_data_Exp__Euler__Step=3__100000epoch.onnx";
% onnxName = "2026-2-2_10_20_10__DNN21__Plant_data_Exp__RK4__Step=1__100000epoch.onnx";
% onnxName = "2026-2-3_9_50_55__DNN21__Plant_data_Exp__RK4__Step=2__100000epoch.onnx";
% % % onnxName = "RK4_step=3";
% onnxName = "2025-12-15_13_22_33__DNN21__Plant_data_Exp__RK4__hidden=3__step=4__epoch_100000.onnx";

onnxName = "2026-2-3_9_53_19__DNN12__Plant_data_Exp__Euler__100000epoch.onnx"; % 入力層が12次元のモデル

agent.controller.mec = DNNMEC(agent, onnxName);

agent.cha_allocation.controller = ["nominal","mec"]; % cha_allocationにコントローラー登録

function post(app)
LW = 1.5; % LineWidth
FS = 18; % FontSize
phase = "tfl";
% phase = "t";
% phase = "f";
app.logger.plot({1, "p", "er"},"ax",app.UIAxes, "phase",phase, "fig_num",1, "Linewidth",LW, "Fontsize",FS);
app.logger.plot({1, "p", "er"}, "phase",phase, "fig_num",1, "Linewidth",LW, "Fontsize",FS);
app.logger.plot({1, "q", "e"}, "phase",phase, "fig_num",2, "Linewidth",LW, "Fontsize",FS);
app.logger.plot({1, "v", "er"}, "phase",phase, "fig_num",3, "Linewidth",LW, "Fontsize",FS);
app.logger.plot({1, "w", "e"}, "phase",phase, "fig_num",4, "Linewidth",LW, "Fontsize",FS);
% app.logger.plot({{1, "input", ""}, {1, "controller.result.nominal_input", ""},...
%     {1, "controller.result.delta_input", ""}}, "phase",phase,"fig_num",5); % inputをまとめて見る
app.logger.plot({1, "input", ""}, "phase",phase, "fig_num",6, "Linewidth",LW, "Fontsize",FS);
app.logger.plot({1, "inner_input1:4", ""}, "phase",phase, "fig_num",7, "Linewidth",LW, "Fontsize",FS);
app.logger.plot({1, "controller.result.nominal_input", ""}, "phase",phase, "fig_num",8, "Linewidth",LW, "Fontsize",FS);
if class(app.agent.controller.mec)=="DNNMEC_BEHIND",    app.logger.plot({1, "controller.result.mec_input", ""}, "phase","f", "fig_num",9, "Linewidth",LW, "Fontsize",FS);
elseif class(app.agent.controller.mec)=="DNNMEC",       app.logger.plot({1, "controller.result.delta_input", ""}, "phase",phase, "fig_num",9, "Linewidth",LW, "Fontsize",FS); end
% app.logger.plot({1, "controller.result.delta_input", ""}, "phase","f", "fig_num",10, "Linewidth",LW, "Fontsize",FS);

app.logger.plot({1, "p1-p2", "er"}, "phase",phase, "color", 0, "fig_num",20, "Linewidth",LW, "Fontsize",FS);
app.logger.plot({1, "p1-p2-p3", "er"}, "phase",phase, "color", 0, "fig_num",21, "Linewidth",LW, "Fontsize",FS);

% app.logger.plot({{1, "p", "e"},{1, "controller.result.nominal_p", "p"}}, "phase",phase, "fig_num",11, "Linewidth",LW, "Fontsize",FS);
% app.logger.plot({{1, "q", "e"},{1, "controller.result.nominal_q", "p"}}, "phase",phase, "fig_num",12, "Linewidth",LW, "Fontsize",FS);
% app.logger.plot({{1, "v", "e"},{1, "controller.result.nominal_v", "p"}}, "phase",phase, "fig_num",13, "Linewidth",LW, "Fontsize",FS);
% app.logger.plot({{1, "w", "e"},{1, "controller.result.nominal_w", "p"}}, "phase",phase, "fig_num",14, "Linewidth",LW, "Fontsize",FS);


% 刻み時間描画
t0id = find(app.logger.Data.phase==97,1,'last')+1;
teid = find(app.logger.Data.phase==0,1,'first')-1;
dt = diff(app.logger.Data.t(t0id:teid));
t = app.logger.Data.t(t0id:teid-1);
figure(100)
[t,dt];
plot(t,dt, Linewidth=LW);
hold on
yline(0.025,"LineWidth",LW)
hold off
grid on
legend("dt","25 ms")
title("Calculation time")

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

end
function in_prog(app)
app.TextArea.Text = "estimator : " + app.agent(1).estimator.result.state.get();
end

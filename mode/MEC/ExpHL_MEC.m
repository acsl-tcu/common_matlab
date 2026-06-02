ts = 0; % initial time
dt = 0.025; % sampling period
te = 10000; % termina time
time = TIME(ts, dt, te);
in_prog_func = @(app) in_prog(app);
post_func = @(app) post(app);
logger = LOGGER(1, size(ts:dt:te, 2), 1, [], []);
logger.display_func = @(agent, time) build_display_vector(agent, time);
logger.display_on = true;
fprintf("表示物\nref:[px, py, pz]  est:[px, py, pz]  delta_U:[T, tx, ty, tz]\n\n");

motive = Connector_Natnet('192.168.100.59'); % connect to Motive 405
motive.getData([], []); % get data from Motive
rigid_ids = [1]; % rigid-body number on Motive
sstate = motive.result.rigid(rigid_ids);
initial_state.p = sstate.p;
initial_state.q = sstate.q;
initial_state.v = [0; 0; 0];
initial_state.w = [0; 0; 0];

agent = DRONE;
% agent.plant = DRONE_EXP_MODEL(agent,Model_Drone_Exp(dt, initial_state, "udp", )[1, 252]));
agent.plant = DRONE_EXP_MODEL(agent, Model_Drone_Exp(dt, initial_state, "serial", "COM3"));
agent.parameter = DRONE_PARAM("DIATONE");
agent.sensor.set_function_class("motive", MOTIVE(agent,motive));
agent.estimator.set_function_class("ekf", EKF(agent, Estimator_EKF(agent,dt,MODEL_CLASS(agent,Model_EulerAngle(dt, initial_state, 1)))));


% リファレンス設定 ~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-
% ~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-
takeoff_zd = 1.0; % だいたい1m
center = [0;0;takeoff_zd]; % 原点

% center_hover    = TIME_VARYING_REFERENCE(agent,{"gen_ref_saddle",{"freq",10, "center",center, "radius",[0,0,0]}, 4});
% point_hover     = TIME_VARYING_REFERENCE(agent,{"gen_ref_saddle",{"freq",10, "center",[base'+1;takeoff_zd+1], "radius",[0,0,0]}, 4});
% circle          = TIME_VARYING_REFERENCE(agent,{"gen_ref_circle",{"freq",10, "center",center, "radius",1.0}, 4});
% saddle          = TIME_VARYING_REFERENCE(agent,{"gen_ref_saddle",{"freq",5, "center",center, "radius",[1,1,0.25]}, 4});
lemniscate      = TIME_VARYING_REFERENCE(agent,{"gen_ref_lemniscate",{"freq",10, "center",center, "radius",1, "x",1}, 4});
% triangle        = TIME_VARYING_REFERENCE(agent,{"gen_ref_triangle",{"freq",10, "center",center, "radius",[1,1,0]}, 4});
% flower          = TIME_VARYING_REFERENCE(agent,{"gen_ref_flower",{"freq",10, "center",center, "radius",1.0}, 4});
% heart           = TIME_VARYING_REFERENCE(agent,{"gen_ref_heart",{"freq",10, "center",center, "radius",1.0}, 4});
% star            = TIME_VARYING_REFERENCE(agent,{"gen_ref_star",{"freq",15, "center",center, "radius",1.0}, 4});
% spline9th       = TIME_VARYING_REFERENCE(agent,{"gen_ref_spline",{"point",20, "order",9, "point_dt",4.5, "xylim",[-1,1], "zlim",[0.5,1.5]}}); % 20 points
% spline9th       = TIME_VARYING_REFERENCE(agent,{"gen_ref_spline",{"point",60, "order",9, "point_dt",4.5, "xylim",[-1,1], "zlim",[0.5,1.5]}}); % 60 points

% refpoints = {struct("f",center, "g",[1;0;takeoff_zd], "h",center, "j",[0;1;takeoff_zd], "k",center, "z",[0;0;takeoff_zd+1], "x",center, "c",[-1;-1;takeoff_zd], "v",center, "b",[1;-1;takeoff_zd+1], "n",center), 7.5};
% refpoints = {struct("f",center, "g",[1;0;takeoff_zd], "h",[1;1;takeoff_zd], "j",center, "k",[0;0;takeoff_zd+0.5]), 7.5};
% multiP2P        = MULTI_POINT_REFERENCE(agent,refpoints);


agent.reference.set_function_class("time_varying", lemniscate) % 最終的なset．２つ目の引数の名前を適宜変更
agent.reference.set_function_class("takeoff", TAKEOFF_REFERENCE(agent,"zd",takeoff_zd));
agent.reference.set_function_class("landing", LANDING_REFERENCE(agent,"dt",dt,"vd",0));
%~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-
%~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-

% コントローラ設定 =================================================================
% =================================================================================
agent.controller.set_function_class("nominal", HLC(agent,Controller_HL(dt)));
% agent.controller.set_function_class("nominal", FUNCTIONAL_HLC_SERVO(agent, Controller_FHL_Servo(dt))); % 位置偏差に対するサーボ系HL


onnxName = "2026-5-15_16_37_24__DNN12__Plant_data_Exp_random__Euler__Activation=ReLU__100000epoch.onnx";
onnxName = "2026-2-3_9_53_19__DNN12__Plant_data_Exp_random__Euler__Activation=SiLU__100000epoch.onnx"; % 2025年度卒論で使用

agent.controller.set_function_class("nnmec", NNMEC(agent, onnxName));
% =================================================================================
% =================================================================================

agent.input_transform.set_function_class("thrust2throttle", THRUST2THROTTLE_DRONE(agent, InputTransform_Thrust2Throttle_drone())); % 推力からスロットルに変換


% cha_allocation
agent.cha_allocation.f.reference = "time_varying";
agent.cha_allocation.a.reference = "takeoff";
agent.cha_allocation.t.reference = "takeoff";
agent.cha_allocation.l.reference = "landing";

agent.set_cha_allocation_for_all("controller", ["nominal","nnmec"])


function post(app)
LW = 1.5; % LineWidth
FS = 18; % FontSize
phase = "tfl";
phase = "tf";
calc_rmse(app.logger);
app.logger.plot({1, "p", "esr"},"ax",app.UIAxes, "phase",phase, "fig_num",1, "Linewidth",LW, "Fontsize",FS);
% app.logger.plot({1, "p", "er"}, "phase",phase, "fig_num",1, "Linewidth",LW, "Fontsize",FS, "color",fcolor);
% app.logger.plot({1, "p1:2", "er"}, "phase",phase, "fig_num",1, "Linewidth",LW, "Fontsize",FS, "color",fcolor);
% app.logger.plot({1, "q", "e"}, "phase",phase, "fig_num",2, "Linewidth",LW, "Fontsize",FS, "color",fcolor);
% app.logger.plot({1, "v", "er"}, "phase",phase, "fig_num",3, "Linewidth",LW, "Fontsize",FS, "color",fcolor);
% app.logger.plot({1, "w", "e"}, "phase",phase, "fig_num",4, "Linewidth",LW, "Fontsize",FS, "color",fcolor);
app.logger.plot({1, "input", ""}, "phase",phase,"fig_num",51, "Linewidth",LW, "Fontsize",FS);
app.logger.plot({1, "inner_input1:4", ""}, "phase",phase,"fig_num",52, "Linewidth",LW, "Fontsize",FS);
% app.logger.plot({{1, "input", ""}, {1, "controller.result.nominal_input", ""},...
%     {1, "controller.result.delta_input", ""}}, "phase",phase,"fig_num",5); % inputをまとめて見る
app.logger.plot({1, "p1-p2", "er"}, "phase",phase, "fig_num",6, "Linewidth",LW, "Fontsize",FS, "color",0);
plot_calc_time(app.logger);
% show_animation(app);
end

function dfunc(app)
app.logger.plot({1, "p", "er"},"ax",app.UIAxes,"phase","tfl");
end



function show_animation(app)
if app.logger.k <= 1
    return
end
  mov = DRAW_DRONE_MOTION(app.logger, "self", app.agent, "target", 1);
  mov.animation(app.logger, "self", app.agent, "target", 1, "Motive_ref", 1);
end



function v = build_display_vector(agent, time)
% コンソール表示用の文字列を作る。
% 参照位置、推定位置、入力を並べる。
idx = 1;
if ~isprop(agent(idx).reference.result.state, "xd")
    v = [];
    return
end
xd = agent(idx).reference.result.state.xd;
if isfield(agent(idx).estimator.result, "state")
    p = agent(idx).estimator.result.state.p;
else
    p = [NaN; NaN; NaN];
end
% u = agent(idx).controller.result.input;
% v = sprintf("%c %.3f : R [%7.3f,%7.3f,%7.3f] : P [%7.3f,%7.3f,%7.3f] : U [%7.3f,%7.3f,%7.3f,%7.3f]",agent(idx).cha, time.t, xd(1:3)', p', u');

du = agent(idx).controller.result.delta_input;
v = sprintf("%c %.3f : R [%7.3f,%7.3f,%7.3f] : P [%7.3f,%7.3f,%7.3f] : dU [%7.3f,%7.3f,%7.3f,%7.3f]",agent(idx).cha, time.t, xd(1:3)', p', du');
end

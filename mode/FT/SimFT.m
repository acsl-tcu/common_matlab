N = 1; % the number of agents
ts = 0; % initial time
dt = 0.025; % sampling period
te = 25; % terminal time
time = TIME(ts,dt,te,N); % instance of time class
in_prog_func = @(app) dfunc(app); % in progress plot
post_func = @(app) post(app); % function working at the "draw button" pushed.
motive = Connector_Natnet_sim(dt); % imitation of Motive camera (motion capture system)
logger = LOGGER(1, size(ts:dt:te, 2), 0, [],[]); % instance of LOOGER class for data logging
logger.display_func = @(agent, time) build_display_vector(agent, time);
logger.display_on = true;
logger.set_time_handler(time);
fprintf("表示物\nref:[px, py, pz]  est:[px, py, pz]  U:[T, tx, ty, tz]\n\n");
initial_state.p = arranged_position([0, 0], 1, 1, 0);
initial_state.q = [1; 0; 0; 0];
initial_state.v = [0; 0; 0];
initial_state.w = [0; 0; 0];

agent = DRONE;
agent.plant = MODEL_CLASS(agent,Model_EulerAngle(dt, initial_state, 1));
%外乱を与える==========
% agent.plant = MODEL_CLASS(agent,Model_EulerAngle_With_Disturbance(dt, initial_state, 1));%外乱用モデル
% agent.input_transform = THRUST_DST_DRONE(agent,InputTransform_Dst_drone(time)); % 外乱付与
%=====================
agent.parameter = DRONE_PARAM("DIATONE"); 
agent.estimator.set_function_class("ekf", EKF(agent, Estimator_EKF(agent,dt,MODEL_CLASS(agent,Model_EulerAngle(dt, initial_state, 1)),["p", "q"])));
agent.sensor.set_function_class("motive", MOTIVE(agent, motive));
agent.reference.set_function_class("timevarying", TIME_VARYING_REFERENCE(agent,{"gen_ref_saddle",{"freq",5,"center",[0;0;1],"radius",[2,2,0.5]},4}));
fApprox_FTxy = 0;%approximate x,y directional FTC input : 1
fNewParam = 0;%新しく更新する場合 : 1
fConfirmFig =0;%近似入力のfigureを確認する場合 : 1
agent.controller.set_function_class("ftc", FTC(agent,Controller_FT(dt, fApprox_FTxy, fNewParam, fConfirmFig)));
for i = 1:length(agent)
    agent(i).reference.set_function_class("takeoff", TAKEOFF_REFERENCE(agent(i),"zd",1.2,"te",3));
    agent(i).reference.set_function_class("landing", LANDING_REFERENCE(agent(i),"dt",dt,"vd",0,"te",5));
    agent(i).cha_allocation.a.reference = "takeoff";
    agent(i).cha_allocation.t.reference = "takeoff";
    agent(i).cha_allocation.l.reference = "landing";
end
function post(app)
app.logger.plot({1, "p", "pre"},"ax",app.UIAxes,"xrange",[app.time.ts,app.time.te]);
% app.logger.plot({1, "q", "s"},"ax",app.UIAxes2,"xrange",[app.time.ts,app.time.te]);
% app.logger.plot({1, "v", "er"},"ax",app.UIAxes3,"xrange",[app.time.ts,app.time.te]);
% app.logger.plot({1, "input", ""},"ax",app.UIAxes4,"xrange",[app.time.ts,app.time.t]);
show_animation(app);
end
function dfunc(app)
app.logger.plot({1, "p", "pre"},"ax",app.UIAxes,"xrange",[app.time.ts,app.time.te]);
% app.logger.plot({1, "q", "s"},"ax",app.UIAxes2,"xrange",[app.time.ts,app.time.te]);
% app.logger.plot({1, "v", "er"},"ax",app.UIAxes3,"xrange",[app.time.ts,app.time.te]);
% app.logger.plot({1, "input", ""},"ax",app.UIAxes4,"xrange",[app.time.ts,app.time.t]);
end
function show_animation(app)
% 協調吊り下げ（複数ドローン＋牽引物）のアニメーションを生成する。
% Nは機体数、牽引物はN+1
if app.logger.k <= 1
    return
end
  mov = DRAW_DRONE_MOTION(app.logger, "self", app.agent, "target", 1,...
          "lims", [ -5 5;  -5 5;  -3 5 ]);
  mov.animation(app.logger, "self", app.agent, "target", 1, "Motive_ref", 1);
end
function v = build_display_vector(agent, time)
% コンソール表示用の文字列を作る。
% 参照位置、推定位置、入力、推定質量を並べる。
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
u = agent(idx).controller.result.input;
v = sprintf("%c %.3f : R [%7.3f,%7.3f,%7.3f] : P [%7.3f,%7.3f,%7.3f] : U [%7.3f,%7.3f,%7.3f,%7.3f]",agent(idx).cha, time.t, xd(1:3)', p', u');
end

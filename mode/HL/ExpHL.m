N = 1; % the number of agents
ts = 0; % initial time
dt = 0.025; % sampling period
te = 10000; % termina time
time = TIME(ts, dt, te, N);
in_prog_func = @(app) in_prog(app);
post_func = @(app) post(app);
logger = LOGGER(1, size(ts:dt:te, 2), 1, [], []);
logger.display_func = @(agent, time) build_display_vector(agent, time);
logger.display_on = true;
fprintf("表示物\nref:[px, py, pz]  est:[px, py, pz]  U:[T, tx, ty, tz]\n\n");

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
agent.plant = DRONE_EXP_MODEL(agent, Model_Drone_Exp(dt, initial_state, "serial", "COM4"));
agent.parameter = DRONE_PARAM("DIATONE");
agent.sensor.set_function_class("motive", MOTIVE(agent,motive));
agent.estimator.set_function_class("ekf", EKF(agent, Estimator_EKF(agent,dt,MODEL_CLASS(agent,Model_EulerAngle(dt, initial_state, 1)))));

% agent.reference.set_function_class("time_varying", TIME_VARYING_REFERENCE(agent,{"gen_ref_circle",{"freq",10,"center",[0;0;1],"radius",0},4})); % hovering
agent.reference.set_function_class("time_varying", TIME_VARYING_REFERENCE(agent,{"gen_ref_circle",{"freq",10,"center",[0;0;1],"radius",1.0},4})); % circle
agent.reference.set_function_class("takeoff", TAKEOFF_REFERENCE(agent,"zd",1));
agent.reference.set_function_class("landing", LANDING_REFERENCE(agent,"dt",dt,"vd",0));

agent.controller.set_function_class("hlc", HLC(agent,Controller_HL(dt)));

agent.input_transform.set_function_class("thrust2throttle", THRUST2THROTTLE_DRONE(agent, InputTransform_Thrust2Throttle_drone())); % 推力からスロットルに変換

agent.cha_allocation.f.reference = "time_varying";
agent.cha_allocation.a.reference="takeoff";
agent.cha_allocation.t.reference="takeoff";
agent.cha_allocation.l.reference="landing";

function post(app)
phase = "tfl";
FS = 16; %FontSize
LW = 1.5;%LineWidth
app.logger.plot({1, "p", "esr"},"ax",app.UIAxes,"phase",phase);

% app.logger.plot({1, "p", "esr"},"fig_num",10, "phase",phase, "Fontsize",FS, "Linewidth",LW);
app.logger.plot({1, "q", "es"},"fig_num",20, "phase",phase, "Fontsize",FS, "Linewidth",LW);
% app.logger.plot({1, "v", "er"},"fig_num",30, "phase",phase, "Fontsize",FS, "Linewidth",LW);
% app.logger.plot({1, "w", "e"},"fig_num",40, "phase",phase, "Fontsize",FS, "Linewidth",LW);
% app.logger.plot({1, "input", ""},"fig_num",50, "phase",phase, "Fontsize",FS, "Linewidth",LW);
% app.logger.plot({1, "input2:4", ""},"fig_num",51, "phase",phase, "Fontsize",FS, "Linewidth",LW);
% app.logger.plot({1, "inner_input", ""},"fig_num",60, "phase",phase, "Fontsize",FS, "Linewidth",LW);
% app.logger.plot({1, "p1-p2", "er"},"fig_num",70, "color",0, "phase",phase, "Fontsize",FS, "Linewidth",LW);
% app.logger.plot({1, "p1-p2-p3", "er"},"fig_num",71, "color",0, "phase",phase, "Fontsize",FS, "Linewidth",LW);

% show_cooperative_animation(app);

plot_calc_time(app.logger);
end

function in_prog(app)
app.TextArea.Text = "estimator : " + app.agent(1).estimator.result.state.get();
end

function show_cooperative_animation(app)
% 協調吊り下げ（複数ドローン＋牽引物）のアニメーションを生成する。
% Nは機体数、牽引物はN+1
if app.logger.k <= 1
    return
end
mov = DRAW_DRONE_MOTION(app.logger, "self", app.agent, "target", 1, ...
    "lims", [ -5 5;  -5 5;  -3 5 ]);
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
u = agent(idx).controller.result.input;
v = sprintf("%c %.3f : R [%7.3f,%7.3f,%7.3f] : P [%7.3f,%7.3f,%7.3f] : U [%7.3f,%7.3f,%7.3f,%7.3f]",agent(idx).cha, time.t, xd(1:3)', p', u');
end

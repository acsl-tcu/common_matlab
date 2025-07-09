ts = 0; % initial time
dt = 0.025; % sampling period
te = 10000; % termina time
time = TIME(ts,dt,te);
in_prog_func = @(app) in_prog(app);
post_func = @(app) post(app);
logger = LOGGER(1, size(ts:dt:te, 2), 1, [],[]);

motive = Connector_Natnet('192.168.100.4'); % connect to Motive
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

% agent.reference.timevarying = TIME_VARYING_REFERENCE(agent,{"gen_ref_saddle",{"freq",10,"orig",[0;0;1],"size",[1,1,0.2]},"HL"});
agent.reference.time_varying = TIME_VARYING_REFERENCE(agent,{"gen_ref_spline",{"point",20,"order",9,"point_dt",5,"ManualSetting",0}});%HLを付けると軌道が微分される
agent.controller = HLC(agent,Controller_HL(dt));

run("ExpBase");
agent.cha_allocation.reference = "time_varying";
function post(app)
LW = 1.0; % Linewidth 
FS = 20; % Fontsize
app.logger.plot({1, "p", "er"},"ax",app.UIAxes,"phase","tfl", "Linewidth",LW, "Fontsize",FS);
app.logger.plot({1, "q", "e"}, "phase","tfl", "fig_num",2, "Linewidth",LW, "Fontsize",FS);
app.logger.plot({1, "v", "er"}, "phase","tfl", "fig_num",3, "Linewidth",LW, "Fontsize",FS);
app.logger.plot({1, "w", "e"}, "phase","tfl", "fig_num",4, "Linewidth",LW, "Fontsize",FS);
% app.logger.plot({{1, "input", ""}, {1, "controller.result.nominal_input", ""},...
%     {1, "controller.result.delta_input", ""}}, "phase","tfl","fig_num",5); % inputをまとめて見る
app.logger.plot({1, "input", ""}, "phase","tfl", "fig_num",6, "Linewidth",LW, "Fontsize",24);
app.logger.plot({1, "inner_input1:4", ""}, "phase","tfl", "fig_num",7, "Linewidth",LW, "Fontsize",24);
% app.logger.plot({1, "controller.result.nominal_input", ""}, "phase","tfl", "fig_num",8, "Linewidth",LW, "Fontsize",FS);
% app.logger.plot({1, "controller.result.delta_input", ""}, "phase","tfl", "fig_num",9, "Linewidth",LW, "Fontsize",FS);

app.logger.plot({1, "p1-p2", "er"}, "phase","tfl", "color", 0, "fig_num",10, "Linewidth",LW, "Fontsize",FS);
app.logger.plot({1, "p1-p2-p3", "er"}, "phase","tfl", "color", 0, "fig_num",11, "Linewidth",LW, "Fontsize",FS);
end
function in_prog(app)
app.TextArea.Text = "estimator : " + app.agent(1).estimator.result.state.get();
end
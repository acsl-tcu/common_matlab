ts = 0; % initial time
dt = 0.025; % sampling period
te = 10000; % termina time
time = TIME(ts,dt,te);
in_prog_func = @(app) in_prog(app);
post_func = @(app) post(app);
logger = LOGGER(1, size(ts:dt:te, 2), 1, [],[]);

motive = Connector_Natnet('192.168.100.59'); % connect to Motive
motive.getData([], []); % get data from Motive
rigid_ids = [1]; % rigid-body number on Motive
sstate = motive.result.rigid(rigid_ids);
initial_state.p = sstate.p;
initial_state.q = sstate.q;
initial_state.v = [0; 0; 0];
initial_state.w = [0; 0; 0];

agent = DRONE;
% agent.plant = DRONE_EXP_MODEL(agent,Model_Drone_Exp(dt, initial_state, "udp", )[1, 252]));
agent.plant = DRONE_EXP_MODEL(agent,Model_Drone_Exp(dt, initial_state, "serial", "COM3"));
agent.parameter = DRONE_PARAM("DIATONE");
agent.estimator = EKF(agent, Estimator_EKF(agent,dt,MODEL_CLASS(agent,Model_EulerAngle(dt, initial_state, 1)),["p", "q"]));
agent.sensor = MOTIVE(agent, Sensor_Motive(1,0, motive));
agent.input_transform = THRUST2THROTTLE_DRONE(agent,InputTransform_Thrust2Throttle_drone()); % 推力からスロットルに変換
% agent.reference.time_var = TIME_VARYING_REFERENCE(agent,{"gen_ref_figure8", {"freq",8,"orig",[0 0 0.6],"size",[1 1 0],"phase",0}});
% agent.reference.time_var= TIME_VARYING_REFERENCE(agent,{"gen_ref_saddle",{"freq",10,"orig",[0;0;0.6],"size",[1,1,0]},"HL"});%{"Case_study_trajectory",{[0,0,0.6]},"HL"});
% agent.reference.time_var=TIME_VARYING_REFERENCE(agent,{"gen_ref_hl_demo_multi", {"hover",[0 0 0.6]}});
agent.reference.time_var=TIME_VARYING_REFERENCE(agent, {"gen_ref_star", {"freq", 20, "orig", [0 0 0.6], "size", [1 1 0], "phase", pi/2}});
agent.controller.hlc = HLC(agent,Controller_HL(dt));
run("ExpBase");
agent.cha_allocation.reference = "time_var";
agent.cha_allocation.controller = "hlc";
agent.cha_allocation.f.controller = ["hlc"];
% agent.cha_allocation.f.controller = ["kmpc","hlc"];
%agent.cha_allocation.f.controller = ["kmpc"];
function post(app)
app.logger.plot({1, "p", "er"},"ax",app.UIAxes,"phase","tfl");
% app.logger.plot({1, "inner_input", ""}, "fig_num", 1,"xrange",[app.time.ts,app.time.te]);
 app.logger.plot({1, "q", "e"},"fig_num", 4,"phase","tfl");
app.logger.plot({1, "input", ""},"fig_num", 1,"phase","tfl");
app.logger.plot({1, "v", "er"}, "fig_num", 2,"phase","tfl");
% app.logger.plot({1, "input", ""},"ax",app.UIAxes5,"xrange",[app.time.ts,app.time.te]);
 app.logger.plot({1, "inner_input", ""},"fig_num",3,"phase","tfl");
 app.logger.plot({1, "p1-p2-p3", "er"},"fig_num", 6,"phase",'tfl', "color",0);
   % app.logger.plot({{1, "controller.result.hlc", ""},{1, "controller.result.kmpc", ""}},"fig_num", 5,"phase","f");
end
function in_prog(app)
app.TextArea.Text = "estimator : " + app.agent(1).estimator.result.state.get();
end
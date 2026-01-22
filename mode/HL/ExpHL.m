ts = 0; % initial time
dt = 0.025; % sampling period
te = 10000; % termina time
time = TIME(ts, dt, te);
in_prog_func = @(app) in_prog(app);
post_func = @(app) post(app);
logger = LOGGER(1, size(ts:dt:te, 2), 1, [], []);

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
agent.plant = DRONE_EXP_MODEL(agent, Model_Drone_Exp(dt, initial_state, "serial", "COM4"));
agent.parameter = DRONE_PARAM("DIATONE");
agent.sensor.set_function_class("motive", MOTIVE(agent,motive));
agent.estimator.set_function_class("ekf", EKF(agent, Estimator_EKF(agent,dt,MODEL_CLASS(agent,Model_EulerAngle(dt, initial_state, 1)))));

agent.reference.set_function_class("time_varying", TIME_VARYING_REFERENCE(agent,{"gen_ref_circle",{"freq",10,"init",[0;0;1],"radius",0},"HL"})); % hovering
% agent.reference.set_function_class("time_varying", TIME_VARYING_REFERENCE(agent,{"gen_ref_circle",{"freq",10,"init",[0;0;1],"radius",1.0},"HL"})); % circle
agent.reference.set_function_class("takeoff", TAKEOFF_REFERENCE(agent,"zd",1));
agent.reference.set_function_class("landing", LANDING_REFERENCE(agent,"dt",dt,"vd",0));

agent.controller.set_function_class("hlc", HLC(agent,Controller_HL(dt)));

agent.input_transform.set_function_class("thrust2throttle", THRUST2THROTTLE_DRONE(agent, InputTransform_Thrust2Throttle_drone())); % 推力からスロットルに変換

agent.cha_allocation.f.reference = "time_varying";
agent.cha_allocation.a.reference="takeoff";
agent.cha_allocation.t.reference="takeoff";
agent.cha_allocation.l.reference="landing";

function post(app)
app.logger.plot({1, "p", "ers"}, "ax", app.UIAxes, "phase", "tfl");
% app.logger.plot({1, "inner_input", ""},"ax",app.UIAxes2,"xrange",[app.time.ts,app.time.te]);
% app.logger.plot({1, "v", "e"},"ax",app.UIAxes3,"xrange",[app.time.ts,app.time.te]);
% app.logger.plot({1, "input", ""},"ax",app.UIAxes3,"xrange",[app.time.ts,app.time.te]);
% app.logger.plot({1, "input", ""},"ax",app.UIAxes5,"xrange",[app.time.ts,app.time.te]);
% app.logger.plot({1, "inner_input", ""},"ax",app.UIAxes6,"xrange",[app.time.ts,app.time.te]);
end

function in_prog(app)
app.TextArea.Text = "estimator : " + app.agent(1).estimator.result.state.get();
end

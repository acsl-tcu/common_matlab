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
agent.reference.timevarying = TIME_VARYING_REFERENCE(agent,{"gen_ref_saddle",{"freq",10,"orig",[0;0;0.5],"size",[0,0,0]},"HL"});
agent.controller = HLC(agent,Controller_HL(dt));
% agent.input_transform = THRUST2THROTTLE_DRONE(agent,InputTransform_Thrust2Throttle_drone()); % 推力からスロットルに変換
agent.input_transform=INPUTTRANSFORM_AUTOTUNE(agent,InputTransform_Thrust2Throttle_drone());
run("ExpBase");
agent.cha_allocation.reference = "timevarying";


% agent.input_transform.origin = THRUST2THROTTLE_DRONE(agent,InputTransform_Thrust2Throttle_drone()); % 推力からスロットルに変換
% agent.input_transform.autotune = INPUTTRANSFORM_AUTOTUNE(agent,1); %mode選択（0:通常、1:オフセット、2:ゲイン自動取得）
% agent.cha_allocation.input_transform=["origin","autotune"];

function post(app)
% app.logger.plot({1, "p", "ers"},"ax",app.UIAxes,"phase","tfl");
% app.logger.plot({1, "inner_input", ""},"ax",app.UIAxes2,"xrange",[app.time.ts,app.time.te]);
% app.logger.plot({1, "v", "e"},"ax",app.UIAxes3,"xrange",[app.time.ts,app.time.te]);
% app.logger.plot({1, "input", ""},"ax",app.UIAxes3,"xrange",[app.time.ts,app.time.te]);
% app.logger.plot({1, "input", ""},"ax",app.UIAxes5,"xrange",[app.time.ts,app.time.te]);
% app.logger.plot({1, "inner_input", ""},"ax",app.UIAxes6,"xrange",[app.time.ts,app.time.te]);
app.logger.plot({1, "p", "er"},"phase","tfl", "fig_num",1); % 位置: p_x,p_y,p_z
app.logger.plot({1, "q", "e"}, "phase","tf", "fig_num",2 ); % 角度: θ_roll, θ_pitch, θ_yaw
app.logger.plot({1, "v", "er"}, "phase","tf", "fig_num",3);% 速度: v_x, v_y, v_z
app.logger.plot({1, "w", "e"}, "phase","tf", "fig_num",4); % 角速度: ω_roll, ω_ptich, ω_yaw
app.logger.plot({1, "input", ""}, "phase","tf", "fig_num",5); % 制御入力: Thrust, roll, pitch, yaw
app.logger.plot({1,"inner_input",""},"phase","tf", "fig_num",6); % 制御入力: Thrust, roll, pitch, yaw
app.logger.plot({1, "p1-p2", "er"}, "phase","tf",  "fig_num",7); % x-y軌跡
app.logger.plot({1, "p1-p2-p3", "er"}, "phase","tf",  "fig_num",8); % x-y-z軌跡
end
function in_prog(app)
app.TextArea.Text = "estimator : " + app.agent(1).estimator.result.state.get();
end
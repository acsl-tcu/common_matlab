N = 1; % the number of agents
ts = 0; % initial time
dt = 0.025; % sampling period
te = 10000; % termina time
time = TIME(ts,dt,te,N);
in_prog_func = @(app) in_prog(app);
post_func = @(app) post(app);
logger = LOGGER(1, size(ts:dt:te, 2), 1, [],[]);
logger.set_time_handler(time);

motive = Connector_Natnet('192.168.100.59'); % connect to Motive 405
motive.getData([], []); % get data from Motive
rigid_ids = [1]; % rigid-body number on Motive
sstate = motive.result.rigid(rigid_ids);
initial_state.p = sstate.p;
initial_state.q = sstate.q;
initial_state.v = [0; 0; 0];
initial_state.w = [0; 0; 0];

agent = DRONE;
agent.plant = DRONE_EXP_MODEL(agent,Model_Drone_Exp(dt, initial_state, "udp", [100, 252]));
agent.parameter = DRONE_PARAM("DIATONE");
agent.estimator.set_function_class("ekf", EKF(agent, Estimator_EKF(agent,dt,initial_state,MODEL_CLASS(agent,Model_EulerAngle(dt, initial_state, 1)), ["p", "q"])));
agent.sensor.set_function_class("motive", MOTIVE(agent, motive));
agent.input_transform.set_function_class("thrust2throttle", THRUST2THROTTLE_DRONE(agent,InputTransform_Thrust2Throttle_drone())); % already
agent.reference.set_function_class("timevarying", TIME_VARYING_REFERENCE(agent,{"gen_ref_saddle",{"freq",10,"center",[0;0;1],"radius",[1,1,0]},4}));
fApprox_FTxy = 0;%approximate x,y directional FTC input : 1
fNewParam = 0;%新しく更新する場合 : 1
fConfirmFig =0;%近似入力のfigureを確認する場合 : 1
agent.controller.set_function_class("ftc", FTC(agent,Controller_FT(dt, fApprox_FTxy, fNewParam, fConfirmFig)));

agent.cha_allocation.sensor = "motive";
agent.cha_allocation.estimator = "ekf";
agent.cha_allocation.reference = "timevarying";
agent.cha_allocation.controller = "ftc";

for i = 1:length(agent)
    agent(i).reference.set_function_class("takeoff", TAKEOFF_REFERENCE(agent(i),"zd",1.2,"te",3));
    agent(i).reference.set_function_class("landing", LANDING_REFERENCE(agent(i),"dt",dt,"vd",0,"te",5));
    agent(i).cha_allocation.a.reference = "takeoff";
    agent(i).cha_allocation.t.reference = "takeoff";
    agent(i).cha_allocation.l.reference = "landing";
end

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

plot_calc_time(app.logger);
end
function in_prog(app)
app.TextArea.Text = ["estimator : " + app.agent(1).estimator.result.state.get()];
end

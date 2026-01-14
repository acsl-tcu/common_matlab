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
% agent.input_transform.param.pitch_offset = 535;

% agent.reference.timevarying = TIME_VARYING_REFERENCE(agent,{"gen_ref_saddle",{"freq",10,"orig",[0;0;1.3],"size",[1,1,0]},"HL"});
% agent.reference.timevarying = TIME_VARYING_REFERENCE(agent,{"gen_ref_spline",{"point",15,"order",9,"check",1,"point_dt",5,"ManualSetting",0}});%HLを付けると軌道が微分される
agent.reference.timevarying = TIME_VARYING_REFERENCE(agent,{"gen_ref_circle",{"freq",10,"init",[0;0;1.3],"radius",1.0},"HL"});
% agent.reference.timevarying = MY_POINT_REFERENCE(agent, {struct("f", [0;0;1.3], "g", [0.3;0.3;1.3], "h",[0;0;1.3]), 15}); % P2P
% agent.reference.time_varying = TIME_VARYING_REFERENCE(agent,{"gen_ref_lemniscate",{"freq",12,"orig",[0;0;1],"radius",1.0},"HL"});
% agent.reference.time_varying = TIME_VARYING_REFERENCE(agent,{"gen_ref_flower",{"freq",15,"orig",[0;0;1],"radius",1.0},"HL"});
% agent.reference.time_varying = TIME_VARYING_REFERENCE(agent,{"gen_ref_lemniscate_3D",{"freq",10,"orig",[0;0;1],"size",[1,0.5],"phase",[0,0]},"HL"});
% agent.reference.time_varying = TIME_VARYING_REFERENCE(agent,{"gen_ref_star",{"freq",15,"orig",[0;0;1],"radius",1.0},"HL"});
% agent.reference.time_varying = TIME_VARYING_REFERENCE(agent,{"gen_ref_triangle",{"freq",10,"orig",[0;0;1],"size",1.0},"HL"});
% agent.reference.time_varying = TIME_VARYING_REFERENCE(agent,{"gen_ref_heart",{"freq",10,"orig",[0;0;1],"size",1.0},"HL"});


run("ExpBase");
agent.cha_allocation.reference = "timevarying";
agent.controller.nominal=HLC(agent,Controller_HL(dt));
agent.controller.mec=MECKC(agent,Controller_HL(dt));
agent.cha_allocation.controller=["nominal","mec"];%cha_allocationにコントローラー登録
function post(app)
app.logger.plot({1, "p", "er"},"ax",app.UIAxes,"phase","tfl");
% app.logger.plot({1, "inner_input", ""},"ax",app.UIAxes2,"xrange",[app.time.ts,app.time.te]);
% app.logger.plot({1, "v", "e"},"ax",app.UIAxes3,"xrange",[app.time.ts,app.time.te]);

app.logger.plot({1, "q", "e"},"fig_num",3);
app.logger.plot({1, "v", "er"},"fig_num",4);
app.logger.plot({1, "input", ""},"fig_num",5,"phase","tfl");
app.logger.plot({1, "inner_input", ""},"fig_num",6,"phase","tfl");
app.logger.plot({1, "p1-p2-p3", "er"},"color",0,"fig_num",7);
app.logger.plot({1, "controller.result.delta_u", ""}, "fig_num",8,"time",[0 65]);

% % 刻み時間描画
 t0id = find(app.logger.Data.phase==97,1,'last')+1;
teid = find(app.logger.Data.phase==0,1,'first')-1;
dt = diff(app.logger.Data.t(t0id:teid));
t = app.logger.Data.t(t0id:teid-1);
figure(100)
ax = gca;
[t,dt];
plot(t,dt);
hold on
yline(0.025)
hold off
grid on
legend("dt","25 ms")
% set(ax.XAxis, fontsize=FS-2)
% set(ax.YAxis, fontsize=FS-2)
% set(ax.Legend, 'FontSize',FS-4);
end
function in_prog(app)
app.TextArea.Text = "estimator : " + app.agent(1).estimator.result.state.get();
end

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
agent.sensor = MOTIVE(agent, Sensor_Motive(rigid_ids,0, motive));
agent.input_transform = THRUST2THROTTLE_DRONE(agent,InputTransform_Thrust2Throttle_drone()); % 推力からスロットルに変換
% agent.controller = HLC(agent,Controller_HL(dt));
agent.controller = FUNCTIONAL_HLC_SERVO(agent, Controller_FHL_Servo(dt)); % 位置偏差に対するサーボ系HL

run("ExpBase");
takeoff_zd = 1; % だいたい1m
agent.reference.takeoff.zd = takeoff_zd;
center = [0;0;takeoff_zd];
% agent.reference.time_varying = TIME_VARYING_REFERENCE(agent,{"gen_ref_saddle",{"freq",10,"orig",[0;0;takeoff_zd],"size",[0,0,0]},"HL"});            % center hovering
% agent.reference.time_varying = TIME_VARYING_REFERENCE(agent,{"gen_ref_saddle",{"freq",10,"orig",[-1;-1;takeoff_zd],"size",[0,0,0]},"HL"});            % hovering
% agent.reference.time_varying = TIME_VARYING_REFERENCE(agent,{"gen_ref_saddle",{"freq",10,"orig",center,"size",[2,2,0]},"HL"});                     % circle
agent.reference.time_varying = TIME_VARYING_REFERENCE(agent,{"gen_ref_saddle",{"freq",10,"orig",center,"size",[1,1,0.25], "phase",0},"HL"});                   % saddle
% agent.reference.time_varying = TIME_VARYING_REFERENCE(agent,{"gen_ref_spline",{"point",20,"order",9,"point_dt",2.5,"ManualSetting",0,"check",1}});  % spline
% agent.reference.time_varying = MY_POINT_REFERENCE(agent, {struct("f", center, "g", [-1;-1;takeoff_zd], "h",[0;0;takeoff_zd+0.5]), 15});             % P2P
agent.cha_allocation.reference = "time_varying";
function post(app)
LW = 1.5; % Linewidth 
FS = 24; % Fontsize
phase = "tfl";
% phase = "t";
% phase = "f";
app.logger.plot({1, "p", "er"},"ax",app.UIAxes,"phase",phase, "Linewidth",LW, "Fontsize",FS);
app.logger.plot({1, "p", "er"}, "phase",phase, "fig_num",1, "Linewidth",LW, "Fontsize",FS);
app.logger.plot({1, "q", "e"}, "phase",phase, "fig_num",2, "Linewidth",LW, "Fontsize",FS);
app.logger.plot({1, "v", "er"}, "phase",phase, "fig_num",3, "Linewidth",LW, "Fontsize",FS);
app.logger.plot({1, "w", "e"}, "phase",phase, "fig_num",4, "Linewidth",LW, "Fontsize",FS);
app.logger.plot({1, "input", ""}, "phase",phase, "fig_num",5, "Linewidth",LW, "Fontsize",FS);
app.logger.plot({1, "inner_input1:4", ""}, "phase",phase, "fig_num",6, "Linewidth",LW, "Fontsize",FS);
app.logger.plot({1, "input2:4", ""}, "phase",phase, "fig_num",7, "Linewidth",LW, "Fontsize",FS);

app.logger.plot({1, "p1-p2", "er"}, "phase","tfl", "color", 0, "fig_num",10, "Linewidth",LW, "Fontsize",FS);
app.logger.plot({1, "p1-p2-p3", "er"}, "phase","tfl", "color", 0, "fig_num",11, "Linewidth",LW, "Fontsize",FS);


% 刻み時間描画
t0id = find(app.logger.Data.phase==97,1,'last')+1;
teid = find(app.logger.Data.phase==0,1,'first')-1;
dt = diff(app.logger.Data.t(t0id:teid));
t = app.logger.Data.t(t0id:teid-1);
figure(100)
ax = gca;
[t,dt];
plot(t,dt, Linewidth=LW);
hold on
yline(0.025,"LineWidth",LW)
hold off
grid on
legend("dt","25 ms")
set(ax.XAxis, fontsize=FS-2)
set(ax.YAxis, fontsize=FS-2)
set(ax.Legend, 'FontSize',FS-4);

end
function in_prog(app)
app.TextArea.Text = "estimator : " + app.agent(1).estimator.result.state.get();
end

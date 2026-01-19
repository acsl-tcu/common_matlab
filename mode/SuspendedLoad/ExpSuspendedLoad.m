clc
ts = 0; % initial time
dt = 0.025; %0.025; % sampling period
% dt=0.03%粉砕
te = 10000; % termina time
time = TIME(ts, dt, te);
in_prog_func = @(app) in_prog(app);
post_func = @(app) post(app);
% logger = LOGGER(1, size(ts:dt:te, 2), 0, [],[]);%今までのやつ
logger = LOGGER(1, size(ts:dt:te, 2), 1, [], []);

motive = Connector_Natnet('192.168.100.4'); % connect to Motive
motive.getData([], []); % get data from Motive
Drone = motive.result.rigid(1);
Load = motive.result.rigid(2);
initial_state.p = Drone.p;
initial_state.q = Drone.q;
initial_state.pL = Load.p;
initial_state.v = [0; 0; 0];
initial_state.w = [0; 0; 0];
initial_state.vL = [0; 0; 0];
initial_state.pT = [0; 0; -1];
initial_state.wL = [0; 0; 0];
%=推定方法を変える場合==========================================================================
%-拡張質量システム：
% Model_Suspended_Load(dt,initial,id,agent,isEstLoadMass):isEstLoadMass=1
%=============================================================================================

agent = DRONE;
agent.parameter = DRONE_PARAM_SUSPENDED_LOAD("DIATONE");
agent.parameter.set("cableL", 1.037); %0.992,0.647,p0.613,0.460
% agent.parameter.set("cableL",1.047);%0.992,0.647,p0.613,0.460
agent.parameter.set("loadmass", 0.075); %0.0968); %0.968
% agent.parameter.set("loadmass",0.6);%0.0968);%0.968 フィラトケース0.239
agent.plant = DRONE_EXP_MODEL(agent, Model_Drone_Exp(dt, initial_state, "serial", "COM4")); %有線プロポ
agent.sensor.set_function_class("motive", MOTIVE(agent, Sensor_Motive([1, 2], 0, motive))); % rigid_id,initial_yaw_angle,motive

% est.model = MODEL_CLASS(agent(1),Model_Suspended_Load(dt, initial_state,1,agent(1)));
agent(1).estimator.set_function_class("ekf", EKF(agent(1), Estimator_EKF(agent(1), dt, ...
    MODEL_CLASS(agent, Model_Suspended_Load(dt, initial_state, 1, agent(1), "Load_mL_HL")), ["p", "q", "pL", "pT"], "sensor_func", @sl_sensor_func)); %expの流用 質量推定有
agent(1).estimator.set_function_class("loadstate", SUSPENDED_LOAD_STATE_MANAGER(agent(1)));

% agent.estimator = EKF(agent, Estimator_EKF(agent,dt,...
%     MODEL_CLASS(agent,Model_Suspended_Load(dt, initial_state, 1,agent,"Load_mL_HL")),...
%     ["p", "q", "pL", "pT"],"sensor_func",@sensor_func));%expの流用

% agent.reference.timevarying = TIME_VARYING_REFERENCE(agent,{"gen_ref_saddle",{"freq",15,"orig",[0;0;0.5],"size",[1,1,0.2*0]*1},"HL"});
% agent.reference.timevarying = MULTI_POINT_REFERENCE(agent,{struct("f",[1;1;0.4],"g",[0;1;0.4],"h",[-1;1;0.4],"j",[-1;0;0.4],"k",[-1;-1;0.4]),10});

% agent.reference.timevarying = TIME_VARYING_REFERENCE(agent,{"gen_ref_p2p",{"p0",[0;0;0.5], "pf",[1;1;0.5], "T",10}, "HL"});
agent.reference.set_function_class("timevarying", TIME_VARYING_REFERENCE(agent, {"gen_ref_saddle", {"freq", 15, "orig", [0; 0; 0.5], "size", [0, 0, 0]}, "HL"}));
% agent.reference.origin  = agent.reference.timevarying;  %揺れ抑制用（既存）
% agent.reference.swaymod = SWAY_REF_MOD(agent, SwayRefMod_Param(dt)); %揺れ抑制
agent.reference.set_function_class("sload", SUSPENDED_LOAD_REF_ADJUST(agent));

agent.controller.set_function_class("hlc_suspended", HLC_SUSPENDED_LOAD(agent, Controller_HL_Suspended_Load(dt, agent)));
% agent.controller.hl = HLC(agent,Controller_HL(dt));
agent.input_transform.set_function_class("thrust2throttle", THRUST2THROTTLE_DRONE(agent, InputTransform_Thrust2Throttle_drone())); % 推力からスロットルに変換
run("ExpBase");
agent.cha_allocation.sensor = "motive";
agent.cha_allocation.estimator = ["ekf", "loadstate"];
agent.cha_allocation.reference = ["timevarying", "sload"];
% agent(1).cha_allocation.f.reference = ["origin","swaymod"]; %揺れ抑制
%%

function post(app)
% app.logger.plot({{1, "input", ""},{1, "controller.result.sus", ""}},"ax",app.UIAxes);
app.logger.plot({{1, "estimator.result.state.pL", "e"}}, "ax", app.UIAxes, "phase", "tf");

app.logger.plot({1, "p", "er"}, "phase", "tfl", "fig_num", 1); % 位置: p_x,p_y,p_z
% app.logger.plot({1, "q", "e"}, "phase","tf", "fig_num",2 ); % 角度: θ_roll, θ_pitch, θ_yaw
app.logger.plot({1, "v", "er"}, "phase", "tf", "fig_num", 3); % 速度: v_x, v_y, v_z
% app.logger.plot({1, "w", "e"}, "phase","tf", "fig_num",4); % 角速度: ω_roll, ω_ptich, ω_yaw
app.logger.plot({1, "input", ""}, "phase", "tf", "fig_num", 5); % 制御入力: Thrust, roll, pitch, yaw
app.logger.plot({1, "inner_input1:4", ""}, "phase", "tf", "fig_num", 6); % 制御入力: Thrust, roll, pitch, yaw
app.logger.plot({1, "p1-p2", "er"}, "phase", "tf", "fig_num", 7); % x-y軌跡
% app.logger.plot({1, "p1-p2-p3", "er"}, "phase","tf",  "fig_num",8); % x-y-z軌跡
% app.logger.plot({1, "p", "ers"},"phase","tf","fig_num",9);
% app.logger.plot({1, "sensor.result.", "er"},"phase","tf", "fig_num",10);
% app.logger.plot.("controller.result.xd","phase","tf","fig_num",11); % 位置: p_x,p_y,p_z
% app.logger.plot({1, "controller.result.xd1:3","r"},"fig_num",20);
% app.logger.plot({1, "controller.result.xd", "r"});
% app.logger.plot({1, "controller.result.mL",""},"fig_num",200);

% 刻み時間描画
t0id = find(app.logger.Data.phase == 97, 1, 'last') + 1;
teid = find(app.logger.Data.phase == 0, 1, 'first') - 1;
dt = diff(app.logger.Data.t(t0id:teid));
t = app.logger.Data.t(t0id:teid - 1);
figure(100)
[t, dt];
plot(t, dt);
% app.logger.plot({1,"p","e"})
hold on
% yline(0.025,"LineWidth",0.5)
% ylim([0 0.05])
hold off
grid on
legend("dt", "upper limit")

% Graphplot(app)
end

function in_prog(app)
app.TextArea.Text = ["estimator : " + app.agent(1).estimator.result.state.get()];
end

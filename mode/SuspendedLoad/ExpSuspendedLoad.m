clc
ts = 0; % initial time
dt = 0.025; %0.025; % sampling period
te = 10000; % termina time
time = TIME(ts, dt, te);
in_prog_func = @(app) in_prog(app);
post_func = @(app) post(app);
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
agent.sensor.set_function_class("motive", MOTIVE(agent, motive,"output_func",@motive_output,"rigid_id",[1,2],"state_list",{["p","q"],"p"}));
function y = motive_output(data)
    p = data.rigid(1).p;
    pT = data.rigid(2).p - p;
    pT = pT/norm(pT);
    y = [p;Quat2Eul(data.rigid(1).q);data.rigid(2).p;pT];
end

agent.estimator.set_function_class("ekf", EKF(agent, Estimator_EKF_SuspendedLoad(agent, dt, ...
    MODEL_CLASS(agent, Model_Suspended_Load(dt, initial_state, 1, agent, "Load_mL_HL")),...
    ["p", "q", "pL", "pT"]))); %expの流用 質量推定有
agent.estimator.set_function_class("loadstate", SUSPENDED_LOAD_STATE_MANAGER(agent));

L = agent.parameter.cableL;
agent.reference.set_function_class("timevarying", TIME_VARYING_REFERENCE(agent,{"gen_ref_saddle",{"freq",25,"orig",[0;0;1],"size",[0,0,0]},"HL"})); % hovering at(0,0,0)
% agent.reference.set_function_class("timevarying", TIME_VARYING_REFERENCE(agent,{"gen_ref_saddle",{"freq",25,"orig",[0;0;1],"size",[0.5,0.5,0.5]},"HL"})); % saddle
% agent.reference.origin  = agent.reference.timevarying;  %揺れ抑制用（既存）
% agent.reference.swaymod = SWAY_REF_MOD(agent, SwayRefMod_Param()); %揺れ抑制
agent.reference.set_function_class("sload", SUSPENDED_LOAD_REF_ADJUST(agent));
agent.reference.set_function_class("takeoff", TAKEOFF_REFERENCE(agent,"zd",1,"te",3));
agent.reference.set_function_class("landing", LANDING_REFERENCE(agent,"dt",dt,"zd",-L,"te",3)); % zd = -Lとするのがミソ

agent.controller.set_function_class("hlc_suspended", HLC_SUSPENDED_LOAD(agent,Controller_HL_Suspended_Load(dt,agent)));
% agent.controller.set_function_class("hl",HLC(agent,Controller_HL(dt)));
agent.input_transform.set_function_class("thrust2throttle", THRUST2THROTTLE_DRONE(agent, InputTransform_Thrust2Throttle_drone())); % 推力からスロットルに変換

agent.set_cha_allocation_for_all("sensor","motive");
agent.set_cha_allocation_for_all("estimator",["ekf","loadstate"]);
agent.cha_allocation.a.reference =["takeoff","sload"]; % aも忘れずにセットする
agent.cha_allocation.t.reference =["takeoff","sload"];
agent.cha_allocation.f.reference =["timevarying","sload"];
% agent.cha_allocation.f.reference = ["origin","swaymod"]; %揺れ抑制?
agent.cha_allocation.l.reference =["landing","sload"]; 
%%

function post(app)
% app.logger.plot({{1, "input", ""},{1, "controller.result.sus", ""}},"ax",app.UIAxes);
app.logger.plot({{1,"p","er"},{1, "estimator.result.state.pL", "e"}}, "ax", app.UIAxes, "phase", "tfl");
app.logger.plot({1, "state.mL", "e"},"phase","tfl");
% app.logger.plot({1, "p", "er"}, "phase", "tfl", "fig_num", 1); % 位置: p_x,p_y,p_z
% app.logger.plot({1, "q", "e"}, "phase","tf", "fig_num",2 ); % 角度: θ_roll, θ_pitch, θ_yaw
% app.logger.plot({1, "v", "er"}, "phase", "tf", "fig_num", 3); % 速度: v_x, v_y, v_z
% app.logger.plot({1, "w", "e"}, "phase","tf", "fig_num",4); % 角速度: ω_roll, ω_ptich, ω_yaw
app.logger.plot({1, "input", ""}, "phase", "tfl", "fig_num", 5); % 制御入力: Thrust, roll, pitch, yaw
app.logger.plot({1, "inner_input1:4", ""}, "phase", "tfl", "fig_num", 6); % 制御入力: Thrust, roll, pitch, yaw
% app.logger.plot({1, "p1-p2", "er"}, "phase", "tf", "fig_num", 7); % x-y軌跡
% app.logger.plot({1, "p1-p2-p3", "er"}, "phase","tf",  "fig_num",8); % x-y-z軌跡
% app.logger.plot({1, "p", "ers"},"phase","tf","fig_num",9);
% app.logger.plot({1, "sensor.result.", "er"},"phase","tf", "fig_num",10);
% app.logger.plot.("controller.result.xd","phase","tf","fig_num",11); % 位置: p_x,p_y,p_z
% app.logger.plot({1, "controller.result.xd1:3","r"},"fig_num",20);

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
app.TextArea.Text = ["estimator : " + app.agent.estimator.result.state.get()];
end

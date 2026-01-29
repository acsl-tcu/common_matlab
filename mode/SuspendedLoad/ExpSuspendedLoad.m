clc
ts = 0; % initial time
dt = 0.025; %0.025; % sampling period
te = 10000; % termina time
time = TIME(ts, dt, te);
in_prog_func = @(app) in_prog(app);
post_func = @(app) post(app);
logger = LOGGER(1, size(ts:dt:te, 2), 1, [], []);
logger.display_func = @(agent, time) build_display_vector(agent, time);
logger.display_on = true;
fprintf("表示物\nref:[px, py, pz]  est:[px, py, pz]  U:[T, tx, ty, tz]  mL\n\n");

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

agent = DRONE;
agent.parameter = DRONE_PARAM_SUSPENDED_LOAD("DIATONE");
agent.parameter.set("mass", 0.762); %0.0968); %0.968
agent.parameter.set("cableL", 1.037); %0.992,0.647,p0.613,0.460,0.956
% agent.parameter.set("cableL",1.047);%0.992,0.647,p0.613,0.460,0.956
agent.parameter.set("loadmass", 0.01); %0.0968); %0.968 %0.14棒入り
% agent.parameter.set("loadmass",0.6);%0.0968);%0.968 フィラトケース0.239
agent.parameter.set("jx", 0.06); %0.0968); %0.968
agent.parameter.set("jy", 0.06); %0.0968); %0.968
agent.parameter.set("jz", 0.09); %0.0968); %0.968

agent.plant = DRONE_EXP_MODEL(agent, Model_Drone_Exp(dt, initial_state, "serial", "COM4")); %有線プロポ
agent.sensor.set_function_class("motive", MOTIVE(agent, motive,"output_func",@motive_output,"rigid_id",[1,2],"state_list",{["p","q"],"p"}));
function y = motive_output(obj,data)
    p = data.rigid(obj.rigid_id(1)).p;
    pT = data.rigid(obj.rigid_id(2)).p - p;
    pT = pT/norm(pT);
    y = [p;Quat2Eul(data.rigid(obj.rigid_id(1)).q);data.rigid(obj.rigid_id(2)).p;pT];
end
agent.sensor.set_function_class("sload", SUSPENDED_LOAD_SENSOR_ADJUST(agent,"td",10));     

agent.estimator.set_function_class("ekf", EKF(agent, Estimator_EKF_SuspendedLoad(agent, dt, ...
    MODEL_CLASS(agent, Model_Suspended_Load(dt, initial_state, 1, agent, "Load_mL_HL")),...
    ["p", "q", "pL", "pT"]))); %expの流用 質量推定有
agent.estimator.set_function_class("loadstate", SUSPENDED_LOAD_STATE_MANAGER(agent,"td",10));

L = agent.parameter.cableL;
agent.reference.set_function_class("timevarying", TIME_VARYING_REFERENCE(agent,{"gen_ref_saddle",{"freq",10,"center",[0;0;1.5],"radius",[1,1,0]},4})); % hovering at(0,0,0)
% agent.reference.set_function_class("timevarying", TIME_VARYING_REFERENCE(agent,{"gen_ref_saddle",{"freq",25,"center",[0;0;1],"radius",[0.5,0.5,0.5]},4})); % saddle
% agent.reference.set_function_class("timevarying", TIME_VARYING_REFERENCE(agent,{"gen_ref_triangle",{"freq",15,"center",[0;0;1.5],"radius",[1,1,0]},4})); % triangle
% agent.reference.set_function_class("timevarying", TIME_VARYING_REFERENCE(agent,{"gen_ref_p2p_back_and_forth",{"p0",[-0.5;0;1.5], "p1",[1;0;1.5], "t_go",5.0, "t_hold",5.0, "t_back",5.0},4})); %P2P
agent.reference.set_function_class("swaymod", SWAY_REF_MOD(agent, SwayRefMod_Param(dt))); %揺れ抑制（位置＋速度）
agent.reference.set_function_class("sload", SUSPENDED_LOAD_REF_ADJUST(agent));
agent.reference.set_function_class("takeoff", TAKEOFF_REFERENCE(agent,"zd",1.5,"te",5));
agent.reference.set_function_class("landing", LANDING_REFERENCE(agent,"dt",dt,"zd",agent.estimator.result.state.p(3)-L,"te",10)); % zd = -Lとするのがミソ

agent.controller.set_function_class("hlc_suspended", HLC_SUSPENDED_LOAD(agent,Controller_HL_Suspended_Load(dt,agent)));
% agent.controller.set_function_class("hl",HLC(agent,Controller_HL(dt)));
agent.input_transform.set_function_class("thrust2throttle", THRUST2THROTTLE_DRONE(agent, InputTransform_Thrust2Throttle_drone())); % 推力からスロットルに変換

agent.set_cha_allocation_for_all("sensor",["motive","sload"]);
agent.set_cha_allocation_for_all("estimator",["ekf","loadstate"]);
agent.cha_allocation.a.reference =["takeoff","sload"]; % aも忘れずにセットする
agent.cha_allocation.t.reference =["takeoff","sload"];
% agent.cha_allocation.f.reference =["timevarying","sload"];
agent.cha_allocation.f.reference =["timevarying","sload","swaymod"];%揺れ抑制
agent.cha_allocation.l.reference =["landing","sload"]; 
%%

function post(app)
% app.logger.plot({{1, "input", ""},{1, "controller.result.sus", ""}},"ax",app.UIAxes);
tmp = app.logger.data(1,"sensor.result.output","");

custom = tmp(:,7:9);
phase = "tfl";
app.logger.plot({{1,"p","re"},{1,"p","s",custom},{1, "estimator.result.state.pL", "e"}}, "ax", app.UIAxes);
app.logger.plot({1, "state.mL", "e"},"phase",phase);
% app.logger.plot({1, "p", "er"}, "phase", phase, "fig_num", 1); % 位置: p_x,p_y,p_z
app.logger.plot({1, "q", "e"}, "phase",phase, "fig_num",2 ); % 角度: θ_roll, θ_pitch, θ_yaw
app.logger.plot({{1, "v", "er"},{1,"estimator.result.state.vL",""}}, "phase", phase, "fig_num", 3); % 速度: v_x, v_y, v_z
app.logger.plot({1, "w", "e"}, "phase",phase, "fig_num",4); % 角速度: ω_roll, ω_ptich, ω_yaw
app.logger.plot({1, "input", ""}, "phase", phase, "fig_num", 5); % 制御入力: Thrust, roll, pitch, yaw
app.logger.plot({1, "inner_input1:4", ""}, "phase", phase, "fig_num", 6); % 制御入力: Thrust, roll, pitch, yaw
% app.logger.plot({1, "p1-p2", "er"}, "phase", phase, "fig_num", 7); % x-y軌跡
app.logger.plot({1, "p1-p2-p3", "er"}, "phase",phase,  "fig_num",8, "color",0); % x-y-z軌跡
% app.logger.plot({1, "p", "ers"},"phase",phase,"fig_num",9);
% app.logger.plot({1, "sensor.result.", "er"},"phase",phase, "fig_num",10);
% app.logger.plot.("controller.result.xd","phase",phase,"fig_num",11); % 位置: p_x,p_y,p_z
% app.logger.plot({1, "controller.result.xd1:3","r"},"fig_num",20);
% show_suspended_load_animation(app);

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

function show_suspended_load_animation(app)
% 単機の吊り下げモデルをアニメーション表示する。
if app.logger.k <= 1
    return
end
mov = DRAW_SUSPENDED_LOAD(app.logger, ...
    "target", 1, ...
    "self", app.agent(1));
mov.animation(app.logger, ...
    "target", 1, ...
    "self", app.agent(1));
end

function v = build_display_vector(agent, time)
% コンソール表示用の文字列を作る。
% 参照位置、推定位置、入力、推定質量を並べる。
idx = 1;
if ~isprop(agent(idx).reference.result.state, "xd")
    v = [];
    return
end
xd = agent(idx).reference.result.state.xd;
if isfield(agent(idx).estimator.result, "state")
    p = agent(idx).estimator.result.state.p;
    if isprop(agent(idx).estimator.result.state, "mL")
        mL = agent(idx).estimator.result.state.mL;
    else
        mL = NaN;
    end
else
    p = [NaN; NaN; NaN];
    mL = NaN;
end
u = agent(idx).controller.result.input;
v = sprintf("%c %.3f : R [%7.3f,%7.3f,%7.3f] : P [%7.3f,%7.3f,%7.3f] : U [%7.3f,%7.3f,%7.3f,%7.3f] : mL %7.3f",agent(idx).cha, time.t, xd(1:3)', p', u', mL);
end

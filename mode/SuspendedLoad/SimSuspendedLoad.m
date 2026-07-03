ts = 0; % initial time
dt = 0.025; % sampling period
te = 50; % termina time
time = TIME(ts,dt,te);
in_prog_func = @(app) in_prog(app);
post_func = @(app) post(app);
logger = LOGGER(1, size(ts:dt:te, 2), 0, [],[]); % target, number, fExp, items, agent_items, option
logger.display_func = @(agent, time) build_display_vector(agent, time);
logger.display_on = true;
fprintf("表示物\nref:[px, py, pz]  est:[px, py, pz]  U:[T, tx, ty, tz]  mL\n\n");

% drone plant setting
agent = DRONE;
agent.parameter = DRONE_PARAM_SUSPENDED_LOAD("DIATONE");
agent.parameter.set("loadmass",0.14);%0.0968);%0.968
agent.parameter.set("cableL",2.0);%0.0968);%0.968
initial_state.q  = [0; 0; 0];
initial_state.w  = [0; 0; 0];
initial_state.vL = [0; 0; 0];
initial_state.p = [0; 0; 0];
initial_state.pT = [0; 0; -1];
initial_state.wL = [0; 0; 0];
initial_state.pL = initial_state.p + initial_state.pT*agent.parameter.cableL;
initial_state.v = [0; 0; 0];
% Note: set the model error after setting "plant"
agent.plant = MODEL_CLASS(agent,Model_Suspended_Load(dt, initial_state,1,agent));%dt,initial,id,agent,modelName
agent.parameter.set("loadmass",0.01);%0.0968);%0.968
agent.plant.projection = @create_angle_bias;
function x = create_angle_bias(x)
    bias_eul = [5; 0; 0]; % degreeだけ定常偏差を起こす
    bias_quat= quaternion(Eul2Quat([deg2rad(bias_eul)])');
    q = conj(bias_quat)*quaternion(Eul2Quat(x(4:6))');
    [q1,q2,q3,q4] = parts(q);
    x = [x(1:3);Quat2Eul([q1;q2;q3;q4]);x(7:end)];
end

% Sim only: getData works after setting "plant"
motive = Connector_Natnet_sim(dt, {{1,"p","q"},{1,"pL","pT"}}); % imitation of Motive camera (motion capture system)
motive.getData(agent);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% drone setting  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
agent.sensor.set_function_class("motive", MOTIVE(agent, motive, dt,"output_func",@motive_output,"rigid_id",[1,2],"state_list",{["p","q"],"p"}));
function y = motive_output(obj,data)
% Motiveの生データから、機体姿勢と吊り荷位置・ケーブル方向を整理する。
% 出力は推定器の入力として [p; euler; pL; pT] の形に整形する。
p = data.rigid(obj.rigid_id(1)).p;
pT = data.rigid(obj.rigid_id(2)).p - p;
pT = pT/norm(pT);
y = [p;Quat2Eul(data.rigid(obj.rigid_id(1)).q);data.rigid(obj.rigid_id(2)).p;pT];
end

agent.estimator.set_function_class("ekf", EKF(agent, Estimator_EKF_SuspendedLoad(agent,dt,...
    MODEL_CLASS(agent,Model_Suspended_Load(dt, initial_state, 1,agent,"Load_mL_HL")),...%"Load_mL_HL"
    ["p", "q","pL","pT"])));%expの流用 質量推定有

agent.estimator.set_function_class("loadstate", SUSPENDED_LOAD_STATE_MANAGER(agent));
L = agent.parameter.cableL;
agent.reference.set_function_class("timevarying", TIME_VARYING_REFERENCE(agent,{"gen_ref_saddle",{"freq",5,"center",[0;0;1.5],"radius",[0,0,0]},4})); %円系軌道
% agent.reference.set_function_class("timevarying", TIME_VARYING_REFERENCE(agent,{"gen_ref_p2p_back_and_forth",{"p0",[0;0;3.0], "p1",[2;2;3.0], "t_go",3.0, "t_hold",3.0, "t_back",3.0},4})); %P2P
% agent.reference.set_function_class("timevarying", TIME_VARYING_REFERENCE(agent,{"gen_ref_triangle",{"freq",9,"center",[0;0;1.5],"radius",[1,1,0]},4})); % triangle
agent.reference.set_function_class("sload", SUSPENDED_LOAD_REF_ADJUST(agent));
agent.reference.set_function_class("takeoff", TAKEOFF_REFERENCE(agent,"zd",3.0,"te",5));
agent.reference.set_function_class("landing", LANDING_REFERENCE(agent,"dt",dt,"zd",-L,"te",3)); % zd = -Lとするのがミソ：l移行時のrefは牽引物用なので

agent.controller.set_function_class("hlc_suspended", HLC_SUSPENDED_LOAD(agent,Controller_HL_Suspended_Load(dt,agent)));

agent.set_cha_allocation_for_all("sensor","motive");
agent.set_cha_allocation_for_all("estimator",["ekf","loadstate"]);
agent.cha_allocation.a.reference =["takeoff","sload"]; % aも忘れずにセットする
agent.cha_allocation.t.reference =["takeoff","sload"];
agent.cha_allocation.f.reference =["timevarying","sload"];
agent.cha_allocation.l.reference =["landing","sload"];

%%

function post(app)
% シミュレーション終了後の結果表示とアニメーション作成。
app.logger.plot({{1, "p", "er"},{1, "estimator.result.state.pL", "e"}},"ax",app.UIAxes,"phase","tfl");
% app.logger.plot({1, "state.mL", "e"},"phase","tfl");
%app.logger.plot({1, "estimator.result.ekf_mL", ""},"phase","tfl", "fig_num",2);
% app.logger.plot({1, "p", "er"},"phase","tf", "fig_num",1); % 位置: p_x,p_y,p_z
app.logger.plot({1, "q", "e"}, "phase","tfl", "fig_num",2 ); % 角度: θ_roll, θ_pitch, θ_yaw
% app.logger.plot({{1, "p", "er"},{1, "estimator.result.state.pL", "e"}},"phase","f","fig_num",2);
% app.logger.plot({{1, "v1:2", "er"},{1,"estimator.result.state.vL1:2",""}},"fig_num",3);% 速度: v_x, v_y, v_z
% app.logger.plot({1, "w", "e"}, "phase","tf", "fig_num",4); % 角速度: ω_roll, ω_ptich, ω_yaw
% app.logger.plot({1, "input", ""}, "phase","f", "fig_num",5); % 制御入力: Thrust, roll, pitch, yaw
% app.logger.plot({{1, "reference.result.state.xd1:3", ""},{1,"reference.result.state.xd5:7",""}},"phase","f","fig_num",4);
% app.logger.plot({{1, "reference.result.state.xd9:11", ""},{1,"reference.result.state.xd13:15",""}},"phase","f","fig_num",5);

show_suspended_load_animation(app); % アニメーション描画
end

function show_suspended_load_animation(app)
% 単機の吊り下げモデルをアニメーション表示する。
if app.logger.k <= 1
    return
end

mov = DRAW_SUSPENDED_LOAD(app.logger, ...
    "target", 1, ...
    "self", app.agent(1));
mov.animation(app.logger,"target", 1,"self", app.agent(1));%表示だけ用

% mov.animation(app.logger,"target", 1,"self", app.agent(1),"mp4", true, "pause", 0);%mp4保存用

% mov.animation(app.logger, "target", 1,"self", app.agent(1), "gif", "Data/suspended_load.gif", ...
%     "fps", 20,"gif_delay", 0.05,"skip", 2, "pause", 0);%gif保存用
end

function in_prog(app)
% 実行中に推定状態をUIに表示する。
app.TextArea.Text = "estimator : " + app.agent.estimator.result.state.get();
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
    q = agent(idx).estimator.result.state.q;
    if isprop(agent(idx).estimator.result.state, "mL")
        mL = agent(idx).estimator.result.state.mL;
    else
        mL = NaN;
    end
else
    p = [NaN; NaN; NaN];
    q = [NaN; NaN; NaN];
    mL = NaN;
end
u = agent(idx).controller.result.input;
v = sprintf("%c %.3f : R [%7.3f,%7.3f,%7.3f] : P [%7.3f,%7.3f,%7.3f] : Q [%7.3f,%7.3f,%7.3f] : U [%7.3f,%7.3f,%7.3f,%7.3f] : mL %7.3f",agent(idx).cha, time.t, xd(1:3)', p', q', u', mL);
end

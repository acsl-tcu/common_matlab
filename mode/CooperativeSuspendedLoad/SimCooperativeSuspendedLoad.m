%=====================
% 協調搬送モデル（mainGUI用）
% SimCooperativeSuspendedLoad
%=====================
N = 4; % 機体数

ts = 0;
dt = 0.025;
te = 10;
time = TIME(ts, dt, te);

in_prog_func = @(app) dfunc(app);
post_func = @(app) post(app);
logger = LOGGER(1:N + 1, size(ts:dt:te, 2), 0, [], []); % 1..N: 単機牽引, N+1: 牽引物
logger.display_func = @(agent, time) build_display_vector(agent, time);
logger.display_on = true;
fprintf("表示物\nref:[px, py, pz]  est:[px, py, pz]  U:[T, tx, ty, tz]  mL\n\n");

%% 全体ダイナミクスの初期状態（牽引物）
load_state.p = [0; 0; 0];
load_state.v = [0; 0; 0];
load_state.O = [0; 0; 0];
load_state.wi = repmat([0; 0; 0], N, 1);
load_state.Oi = repmat([0; 0; 0], N, 1);
load_state.a = [0; 0; 0];
load_state.dO = [0; 0; 0];

qtype = "zup"; % "eul":euler angle, "":euler parameter

if contains(qtype, "zup")
    load_state.qi = -1 * repmat([0; 0; 1], N, 1);
    pT_sgn = -1;
else
    load_state.qi = 1 * repmat([0; 0; 1], N, 1);
    pT_sgn = 1;
end

if contains(qtype, "eul")
    load_state.Q = [0; 0; 0];
    load_state.Qi = repmat([0; 0; 0], N, 1);
else
    load_state.Q = Eul2Quat([0; 0; 0 * pi / 180]);
    load_state.Qi = repmat([1; 0; 0; 0], N, 1);
end

%% 牽引物モデル (agent N+1)
agent(N + 1) = DRONE;
agent(N + 1).id = N + 1;
[G,pUp,pDown,rho,rhoini] = set_shape(N);
agent(N + 1).parameter = DRONE_PARAM_COOPERATIVE_LOAD("DIATONE", N, qtype,...
    "rho", rho);
% 形状を描画用にセット           
agent(N + 1).parameter.rhoini = rhoini; 
agent(N + 1).parameter.pUp = pUp;
agent(N + 1).parameter.pDown = pDown;                                                                                
agent(N + 1).parameter.G = G;   

% 紐qiの初期角度を設定
rho12 = [agent(N + 1).parameter.rho(1:2, :); zeros(1, N)];
rho12Unit = rho12 ./ vecnorm(rho12);
pTpre = rho12Unit * tan(4 * pi / 180) + [0; 0; 1];
load_state.qi = pT_sgn * reshape(pTpre ./ vecnorm(pTpre), [], 1);

agent(N + 1).plant = MODEL_CLASS(agent(N + 1), Model_Suspended_Cooperative_Load(dt, load_state, 1, N, qtype));
%% 牽引物のセンサ・推定・参照・制御
agent(N + 1).sensor.set_function_class("direct", DIRECT_SENSOR(agent(N + 1), 0.0, struct("output_list", ["p", "Q"])));
agent(N + 1).estimator.set_function_class("direct", DIRECT_ESTIMATOR(agent(N + 1), struct("model", MODEL_CLASS(agent(N + 1), Model_Suspended_Cooperative_Load(dt, load_state, 1, N, qtype)))));
agent(N + 1).reference.set_function_class("timevarying", TIME_VARYING_REFERENCE(agent(N + 1), {"gen_ref_saddle", {"freq", 10, "center", [0; 0; 2], "radius", [2, 2, 1]}, 5}));
agent(N + 1).controller.set_function_class("input_merge", COOPERATIVE_INPUT_MERGE(agent(N + 1), "payload_index", agent(N + 1).id));
agent(N + 1).set_cha_allocation_for_all("sensor", "direct");
agent(N + 1).set_cha_allocation_for_all("estimator", "direct");
agent(N + 1).set_cha_allocation_for_all("reference", "timevarying");
agent(N + 1).set_cha_allocation_for_all("controller", "input_merge");

%% Motive
assignment = cell(1, 2 * N);
assignment{1} = {N + 1, "p", "Q"}; % rigid 1: 牽引物
for i = 1:N
    assignment{2 * i} = {i, "p", "q"}; % rigid 2i: 機体
    if 2 * i - 1 > 1
        assignment{2 * i - 1} = {N + 1, "p", "Q"}; % 未使用：実験に合わせて設定（実験での牽引点に相当）
    end
end
motive = Connector_Natnet_sim(dt, assignment);

%% 単機牽引モデル (agent 1..N)
rho = agent(N + 1).parameter.rho;
R_load = RodriguesQuaternion(load_state.Q);
for i = 1:N
    agent_state(i).q = [0; 0; 0];
    agent_state(i).v = [0; 0; 0];
    agent_state(i).w = [0; 0; 0];
    agent_state(i).vL = [0; 0; 0];
    agent_state(i).pT = load_state.qi(3 * i - 2:3 * i, 1);
    agent_state(i).wL = [0; 0; 0];
    agent_state(i).p = load_state.p + R_load * rho(:, i) - agent(N + 1).parameter.li(i) * agent_state(i).pT;
    agent_state(i).pL = load_state.p + R_load * rho(:, i);
    agent(i) = DRONE();
    agent(i) = configure_single_agent(agent(i), i, dt, agent_state(i), agent(N + 1), motive, @motive_output);
end

motive.getData(agent);

function y = motive_output(obj, data, rho_i)
% Motiveの生データから、機体姿勢と吊り荷位置・ケーブル方向を整理する。
% 出力は推定器の入力として [p; euler; pL; pT] の形に整形する。
    drone_id = obj.rigid_id(1);
    load_id = obj.rigid_id(2);
    p = data.rigid(drone_id).p;
    load_p = data.rigid(load_id).p;
    load_R = RodriguesQuaternion(data.rigid(load_id).q);
    load_R = load_R(:, :, 1);
    pL = load_p + load_R * rho_i;
    pT = pL - p;
    pT = pT / norm(pT);
    y = [p; Quat2Eul(data.rigid(drone_id).q); pL; pT];
end

function agentObj = configure_single_agent(agentObj, idx, dt, init_state, load_agent, motive, output_func)
    agentObj.id = idx;

    li = load_agent.parameter.li(idx);
    mi = load_agent.parameter.mi(idx);
    jx = load_agent.parameter.Ji(1, idx);
    jy = load_agent.parameter.Ji(2, idx);
    jz = load_agent.parameter.Ji(3, idx);
    agentObj.parameter = DRONE_PARAM_SUSPENDED_LOAD("DIATONE", "cableL", li, "mass", mi, "loadmass", 0.05, "jx", jx, "jy", jy, "jz", jz);
    agentObj.plant = MODEL_CLASS(agentObj, Model_Suspended_Load(dt, init_state, 1, agentObj));

    rigid_ids = [2 * idx, 1]; % [機体, 牽引物]
    rho_i = load_agent.parameter.rho(:, idx);
    agentObj.sensor.set_function_class("loadsync", COOPERATIVE_LOAD_SYNC(agentObj, "payload_index", load_agent.id, "rho", rho_i, "li", agentObj.parameter.cableL));
    agentObj.sensor.set_function_class("motive", MOTIVE(agentObj, motive, "output_func", @(obj, data) output_func(obj, data, rho_i), "rigid_id", rigid_ids, "state_list", {["p", "q"], "p"}));

    agentObj.estimator.set_function_class("ekf", EKF(agentObj, Estimator_EKF_SuspendedLoad(agentObj, dt, ...
        MODEL_CLASS(agentObj, Model_Suspended_Load(dt, init_state, 1, agentObj, "Load_mL_HL")), ["p", "q", "pL", "pT"])));
    agentObj.estimator.set_function_class("loadstate", SUSPENDED_LOAD_STATE_MANAGER(agentObj));

    agentObj.reference.set_function_class("timevarying", TIME_VARYING_REFERENCE(agentObj, {"gen_ref_saddle", {"freq", 10, "center", [0; 0; 2], "radius", [2, 2, 1]}, 5}));
    agentObj.reference.set_function_class("offset", COOPERATIVE_LOAD_REF_OFFSET(agentObj, "payload_index", load_agent.id, "rho", load_agent.parameter.rho(:, idx)));
    agentObj.reference.set_function_class("avoid", COLLISION_AVOID_REF(agentObj, "payload_index", load_agent.id));
    agentObj.reference.set_function_class("sload", SUSPENDED_LOAD_REF_ADJUST(agentObj));
    agentObj.reference.set_function_class("takeoff", TAKEOFF_REFERENCE(agentObj,"zd",1,"te",3));
    L = agentObj.parameter.cableL;
    agentObj.reference.set_function_class("landing", LANDING_REFERENCE(agentObj,"dt",dt,"zd",-L,"te",3)); % zd = -Lとするのがミソ

    agentObj.controller.set_function_class("hlc_suspended", HLC_SUSPENDED_LOAD(agentObj, Controller_HL_Suspended_Load(dt, agentObj)));

    agentObj.set_cha_allocation_for_all("sensor", ["loadsync", "motive"]);
    agentObj.set_cha_allocation_for_all("estimator", ["ekf", "loadstate"]);
    agentObj.cha_allocation.a.reference = ["takeoff", "offset", "avoid", "sload"];
    agentObj.cha_allocation.t.reference = ["takeoff", "offset", "avoid", "sload"];
    agentObj.cha_allocation.f.reference = ["timevarying", "offset", "avoid", "sload"];
    agentObj.cha_allocation.l.reference = ["landing", "offset", "avoid", "sload"];
end

function dfunc(varargin)
% ループ内のフック用空関数。必要になったらここに処理を追加する。
end

function post(app)
% シミュレーション終了後の結果描画とアニメーション呼び出し。
t = app.logger.data(0,"t","");
rdata = app.logger.data(app.N,"p","r");
edata = app.logger.data(app.N,"p","e");
L = app.agent(1).parameter.cableL;
plot(t,rdata(:,3)-L,t,edata(:,3));
app.logger.plot({{app.N, "p", "er"}},"ax",app.UIAxes,"phase","tfl");
% app.logger.plot({1, "state.mL", "e"},"phase","tfl");
% show_cooperative_animation(app);
end

function show_cooperative_animation(app)
% 協調吊り下げ（複数ドローン＋牽引物）のアニメーションを生成する。
% Nは機体数、牽引物はN+1
if app.logger.k <= 1
    return
end

mov = DRAW_COOPERATIVE_DRONES(app.logger, ...
    "target", 1:app.N-1, ...          % 描画する機体ID
    "self", app.agent(app.N), ...
    "load_index", app.N,...
    "lims", [ -5 5;  -5 5;  -3 5 ]);
    % "system_size", [10 10 5]);             % 牽引物側パラメータ(rho, li)を持つエージェント
mov.animation(app.logger, ...
    "target", 1:app.N-1, ...
    "self", app.agent(app.N), ...
    "load_index", app.N);
end
%%
function [G,pUp,pDown,rho,rhoini] = set_shape(N)
% 牽引物形状（上下面ポリゴン）と接続点のオフセットを作る補助関数。
% 外周・内周を持つ多角形を生成し、重心と接続点からrhoを計算する。
  zUp = 0.1;
  zDown = 0.05;                                                                                         
                                                                                                        
  % 外周 (CCW) と内周 (CW)                                                                              
  outer = [1 -1 -1 1 1; 1 1 -1 -1 1];
  inner = 0.95*[1 1 -1 -1 1;1 -1 -1 1 1];
  
  % NaN で境界区切り                                                                                    
  pUp = [outer, [NaN;NaN], inner;                                                                       
         zUp*ones(1,size(outer,2)), NaN, zUp*ones(1,size(inner,2))];                                    
  pDown = [outer, [NaN;NaN], inner;                                                                     
           zDown*ones(1,size(outer,2)), NaN, zDown*ones(1,size(inner,2))];                              
                                                                                                        
  % 重心 (穴あきポリゴン)                                                                               
  ps = polyshape({outer(1,:), inner(1,:)}, {outer(2,:), inner(2,:)});                                   
  [Gx,Gy] = centroid(ps);                                                                               
  G = [Gx; Gy; 0];                                                                                      
                                                                                                        
  % 接続点 (例: 外周4点)                                                                                
  attach = [ outer(:,1:N);                                                                               
             zUp*ones(1,N) ];                                                                           
  rho = attach - G;                                                                                     
  rhoini = rho;  
end

function v = build_display_vector(agent, time)
% コンソール表示用の文字列を作る。
% 参照位置、推定位置、入力、推定質量を並べる。
idx = length(agent);
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

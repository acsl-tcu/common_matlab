% 複数機のSuspendedLoad制御を並列に走らせる実験モード
% ・各機は単機用の推定/制御クラス (EKF + HLC_SUSPENDED_LOAD) をそのまま利用
% ・Motiveの剛体から得た荷重位置/機体姿勢を各機のセンサー入力として共有
% ・牽引物エージェント (agent(1)) は参照軌道を決めるだけで制御は行わない

ts = 0; dt = 0.025; te = 10000;
time = TIME(ts, dt, te);
in_prog_func = @(app) in_prog(app);
post_func = @(app) post(app);

%% Motive接続と剛体情報
motive = Connector_Natnet('192.168.100.59'); % connect to Motive 405
motive.getData([], []);
rigid_num = motive.result.rigid_num; % 奇数: 牽引物含む協調、偶数: 単純に複数単機

targetDroneIds = 1:2; % このPCが担当する機体番号
numDrones = length(targetDroneIds);
isCoop = mod(rigid_num, 2) == 1;
firstDroneIndex = isCoop + 1; % agent(1) が荷重の場合は機体が index=2 から始まる
N = numDrones + isCoop; % agent 配列のサイズ（荷重 + 機数）
addId = (targetDroneIds(1) - 1) * 2; % Motive 剛体IDをシフト

%% 実験機ごとの設定
COMs = [3, 12]; % 使用するプロポのCOM番号
cableL = [0.9677 0.8883]; % 各機のケーブル長
refPointName = { ...
                    {struct("f", [0; 0; 0.5], "g", [1; 0; 0.5], "h", [0; 0; 0.5], "j", [0; 1; 0.5], "k", [0; 0; 0.5], "m", [-1; -1; 0.5], "n", [0; 0; 0.5]), 10}, ...
                    {struct("f", [0; 0; 0.5], "g", [0; 1; 0.5], "h", [0; 0; 0.5], "j", [-1; 0; 0.5], "k", [0; 0; 0.5], "m", [1; -1; 0.5], "n", [0; 0; 0.5]), 10} ...
                };

numAgents = max(N, 1);
agent(1, numAgents) = DRONE;
for k = 1:numAgents
    agent(k) = DRONE;
end

%% 牽引物エージェントの初期化 (剛体情報/参照のみを管理)
if isCoop
    [agent(1), rho] = configure_payload_agent(motive, addId, N);
else
    rho = [];
end

%% 各機体を単機SuspendedLoad構成でセットアップ
for idx = firstDroneIndex:N
    droneId = idx - firstDroneIndex + 1;
    rigidIndexDrone = 2 * idx - firstDroneIndex + addId;
    rigidIndexLoad = rigidIndexDrone + 1;
    init = derive_initial_state_from_motive(motive, rigidIndexDrone, cableL(droneId));

    agent(idx) = configure_single_agent( ...
        agent(idx), dt, init, COMs(droneId), cableL(droneId), ...
        motive, [rigidIndexDrone, rigidIndexLoad], ...
        refPointName{droneId}, isCoop, N, agent(1) ...
    );
end

logger = LOGGER(1:N, size(ts:dt:te, 2), 1, [], []);
logger.display_func = @(agent, time) build_display_vector(agent, time);
logger.display_on = true;
fprintf("表示物\nref:[px, py, pz]  est:[px, py, pz]  U:[T, tx, ty, tz]  mL\n\n");
for i = 1:length(agent)
    agent(i).reference.set_function_class("takeoff", TAKEOFF_REFERENCE(agent(i),"zd",1.2,"te",3));
    agent(i).reference.set_function_class("landing", LANDING_REFERENCE(agent(i),"dt",dt,"vd",0,"te",5));
    agent(i).cha_allocation.a.reference = "takeoff";
    agent(i).cha_allocation.t.reference = "takeoff";
    agent(i).cha_allocation.l.reference = "landing";
end

%% センサー合成 (Motive + FOR_LOAD)
function result = sensor_do(varargin)
result_motive = varargin{5}.sensor.motive.do(varargin);
result_forload = varargin{5}.sensor.forload.do(varargin);
result_forload.state.p = result_motive.state.p;
result_forload.state.q = result_motive.state.q;
varargin{5}.sensor.result = result_forload;
result = result_forload;
end

function post(app)
app.logger.plot({1, "q", "s"}, "ax", app.UIAxes, "xrange", [app.time.ts, app.time.te]);
dt = diff(app.logger.Data.t(1:find(app.logger.Data.phase == 0, 1, 'first') - 1));
t = app.logger.data(0, 't', []);
figure(100)
plot(t(1:end - 1), dt);
hold on
yline(0.025, "LineWidth", 0.5)
ylim([0 0.05])
hold off
% show_cooperative_animation(app);
end

function in_prog(app)
app.TextArea.Text = ["estimator : " + app.agent(1).estimator.result.state.get()];
end

%% helper : payload agent
function [payloadAgent, rho] = configure_payload_agent(motive, addId, N)
payloadAgent = DRONE;
payloadAgent.parameter = DRONE_PARAM_COOPERATIVE_LOAD("DIATONE", N, "zup");
payloadAgent.plant = struct("do", @(varargin)[], "arming", [], "stop", []);
payloadAgent.plant.connector.serial = [];

rigid = motive.result.rigid(1);
eul = Quat2Eul(rigid.q);
rho = zeros(3, N - 1);
rhoini = zeros(3, N - 1);

for i = 1:N - 1
    rhoini(:, i) = motive.result.rigid(1 + 2 * i + addId).p - rigid.p;
    rho(:, i) = rhoini(:, i);
end

payloadAgent.parameter = DRONE_PARAM_COOPERATIVE_LOAD("DIATONE", N, "zup", "rho", rho, "rhoini", rhoini);
payloadAgent.estimator.do = @(varargin)[];
payloadAgent.estimator.result.state = STATE_CLASS(struct('state_list', ["p", "q"], "num_list", [3, 3]));
payloadAgent.estimator.result.state.p = rigid.p;
payloadAgent.estimator.result.state.q = eul;
payloadAgent.sensor.set_function_class("motive", MOTIVE(payloadAgent, motive, "initial_yaw_angle", eul(3)));
payloadAgent.reference.set_function_class("timevarying", TIME_VARYING_REFERENCE(payloadAgent, {"gen_ref_saddle", {"freq", 12, "center", [0; 0; 0.7], "radius", [0.8, 0.8, 0]}, 5}));
payloadAgent.controller.do = @(varargin)[];
payloadAgent.controller.result.input = [0; 0; 0; 0];
payloadAgent.input_transform.set_function_class("identity", struct("do", @(varargin)[], "result", zeros(1, 8)));
plot_and_close(N * 2 - 1, payloadAgent);
end

%% helper : single SuspendedLoad agent configuration
function agentObj = configure_single_agent(agentObj, dt, initial_state, com, cableLen, motive, rigidIdPair, refPoint, isCoop, N, payloadAgent)
agentObj.parameter = DRONE_PARAM_SUSPENDED_LOAD("DIATONE");
agentObj.parameter.set("cableL", cableLen);
agentObj.plant = DRONE_EXP_MODEL(agentObj, Model_Drone_Exp(dt, initial_state, "serial", com));

agentObj.sensor.set_function_class("motive", MOTIVE(agentObj, motive,"output_func",@motive_output,"rigid_id",rigidIdPair,"state_list",{["p","q"],"p"}));

agentObj.estimator.set_function_class("ekf", EKF(agentObj, Estimator_EKF_CooperativeSuspendedLoad(agentObj,dt,...
    MODEL_CLASS(agentObj,Model_Suspended_Load(dt, initial_state, idx,agentObj,"Load_mL_HL")),...
    ["p", "q", "pL", "pT"])));
agentObj.estimator.set_function_class("loadstate", SUSPENDED_LOAD_STATE_MANAGER(agentObj));

L = agentObj.parameter.cableL;
agentObj.reference.set_function_class("timevarying", TIME_VARYING_REFERENCE(agentObj,...
    {"gen_ref_saddle",{"freq",15,"center",[0;0;1],"radius",[0.5,0.5,0]},4}));
agentObj.reference.set_function_class("sload", SUSPENDED_LOAD_REF_ADJUST(agentObj));
agentObj.reference.set_function_class("takeoff", TAKEOFF_REFERENCE(agentObj,"zd",1,"te",5));
agentObj.reference.set_function_class("landing", LANDING_REFERENCE(agentObj,"dt",dt,"zd",-L,"te",5)); % zd = -Lとするのがミソ
if ~isempty(refPoint)
    agentObj.reference.set_function_class("point", MULTI_POINT_REFERENCE(agentObj, refPoint));
    baseRefOrder = ["timevarying", "point"];
else
    baseRefOrder = "timevarying";
end
if isCoop
    agentObj.reference.set_function_class("split", TIME_VARYING_REFERENCE_SPLIT(agentObj, {"dammy", [], "Split", N}, payloadAgent));
    agentObj.cha_allocation.reference = [baseRefOrder, "split", "sload"];
else
    agentObj.cha_allocation.reference = [baseRefOrder, "sload"];
end

agentObj.controller.set_function_class("hlc_suspended", HLC_SUSPENDED_LOAD(agentObj,Controller_CooperativeSuspededLoad(dt,agentObj)));
agentObj.input_transform.set_function_class("thrust2throttle", THRUST2THROTTLE_DRONE(agentObj, dt, InputTransform_Thrust2Throttle_drone(dt))); % 推力からスロットルに変換

agentObj.set_cha_allocation_for_all("sensor","motive");
agentObj.set_cha_allocation_for_all("estimator",["ekf","loadstate"]);
agentObj.cha_allocation.a.reference =["takeoff","sload"]; % aも忘れずにセットする
agentObj.cha_allocation.t.reference =["takeoff","sload"];
agentObj.cha_allocation.f.reference =["timevarying","sload"];
agentObj.cha_allocation.l.reference =["landing","sload"]; 
end

function init = derive_initial_state_from_motive(motive, rid, cableLen)
rigid = motive.result.rigid(rid);
init.p = rigid.p;
init.q = rigid.q;
init.v = [0; 0; 0];
init.w = [0; 0; 0];
init.pT = [0; 0; -1];
init.pL = init.p + init.pT * cableLen;
init.vL = [0; 0; 0];
init.wL = [0; 0; 0];
end
function y = motive_output(obj,data)
    p = data.rigid(obj.rigid_id(1)).p;
    pT = data.rigid(obj.rigid_id(2)).p - p;
    pT = pT/norm(pT);
    y = [p;Quat2Eul(data.rigid(obj.rigid_id(1)).q);data.rigid(obj.rigid_id(2)).p;pT];
end

function show_cooperative_animation(app)
% 協調吊り下げ（複数ドローン＋牽引物）のアニメーションを生成する。
% Nは機体数、牽引物はN+1
if app.logger.k <= 1
    return
end
mov = DRAW_COOPERATIVE_DRONES(app.logger, ...
    "target", 1:app.N-1, ...
    "self", app.agent(app.N), ...
    "load_index", app.N,...
    "lims", [ -5 5;  -5 5;  -3 5 ]);
mov.animation(app.logger, ...
    "target", 1:app.N-1, ...
    "self", app.agent(app.N), ...
    "load_index", app.N);
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

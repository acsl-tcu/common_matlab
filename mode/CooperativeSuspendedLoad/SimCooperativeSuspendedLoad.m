% 複数機の単機SuspendedLoad制御を同時に回すシミュレーションモード
% それぞれの機体は mode/SuspendedLoad/SimSuspendedLoad と同じ制御器を持ち、
% Motiveシミュレータの剛体を個別に割り当てて走らせる。

numDrones = 3;
ts = 0; dt = 0.025; te = 30;
time        = TIME(ts,dt,te);
in_prog_func = @(app) dfunc(app);
post_func = @(app) post(app);
logger = LOGGER(1:numDrones, size(ts:dt:te, 2), 0, [],[]);

%% 各剛体に対応するMotiveシミュレータを作成
motiveCells = cell(2*numDrones,1);
for i = 1:numDrones
    motiveCells{2*i-1} = {i,"p","q"};
    motiveCells{2*i}   = {i,"pL","pT"};
end
motive = Connector_Natnet_sim(dt, motiveCells);

for i = 1:numDrones
    agent(i) = DRONE();
    agent(i) = configure_sim_agent(agent(i), dt, i);
end

motive.getData(agent);
for i = 1:numDrones
    rigidPair = [2*i, 2*i+1];
    agent(i).sensor.set_function_class("motive", MOTIVE(agent(i), motive,"output_func",@motive_output,"rigid_id",rigidPair,"state_list",{["p","q"],"p"}));
end
function y = motive_output(obj,data)
    p = data.rigid(obj.rigid_id(1)).p;
    pT = data.rigid(obj.rigid_id(2)).p - p;
    pT = pT/norm(pT);
    y = [p;Quat2Eul(data.rigid(obj.rigid_id(1)).q);data.rigid(obj.rigid_id(2)).p;pT];
end

%% loop helper (no GUI plotting for now)
function dfunc(varargin); end
function post(app)
app.logger.plot({{1, "p", "er"},{1, "estimator.result.state.pL", "e"}},"ax",app.UIAxes,"phase","tfl");
app.logger.plot({1, "state.mL", "e"},"phase","tfl");
end
%% helper : configure a single simulated suspended-load agent
function agentObj = configure_sim_agent(agentObj, dt, idx)
agentObj.parameter = DRONE_PARAM_SUSPENDED_LOAD("DIATONE");
initial_state.q  = [0; 0; 0];
initial_state.w  = [0; 0; 0];
initial_state.vL = [0; 0; 0];
initial_state.p  = [0.5*(idx-1); 0; 0]; % 横並びに配置
initial_state.pT = [0; 0; -1];
initial_state.wL = [0; 0; 0];
initial_state.pL = initial_state.p + initial_state.pT*agentObj.parameter.cableL;
initial_state.v  = [0; 0; 0];

agentObj.plant = MODEL_CLASS(agentObj,Model_Suspended_Load(dt, initial_state,idx,agentObj));

agentObj.estimator.set_function_class("ekf", EKF(agentObj, Estimator_EKF_CooperativeSuspendedLoad(agentObj,dt,...
    MODEL_CLASS(agentObj,Model_Suspended_Load(dt, initial_state, idx,agentObj,"Load_mL_HL")),...
    ["p", "q", "pL", "pT"])));
agentObj.estimator.set_function_class("loadstate", SUSPENDED_LOAD_STATE_MANAGER(agentObj));

L = agentObj.parameter.cableL;
agentObj.reference.set_function_class("timevarying", TIME_VARYING_REFERENCE(agentObj,...
    {"gen_ref_saddle",{"freq",15,"orig",[0;0;1],"size",[0.5,0.5,0]},"HL"}));
agentObj.reference.set_function_class("sload", SUSPENDED_LOAD_REF_ADJUST(agentObj));
agentObj.reference.set_function_class("takeoff", TAKEOFF_REFERENCE(agentObj,"zd",1,"te",5));
agentObj.reference.set_function_class("landing", LANDING_REFERENCE(agentObj,"dt",dt,"zd",-L,"te",5)); % zd = -Lとするのがミソ

agentObj.controller.set_function_class("hlc_suspended", HLC_SUSPENDED_LOAD(agentObj,Controller_CooperativeSuspededLoad(dt,agentObj)));

agentObj.set_cha_allocation_for_all("sensor","motive");
agentObj.set_cha_allocation_for_all("estimator",["ekf","loadstate"]);
agentObj.cha_allocation.a.reference =["takeoff","sload"]; % aも忘れずにセットする
agentObj.cha_allocation.t.reference =["takeoff","sload"];
agentObj.cha_allocation.f.reference =["timevarying","sload"];
agentObj.cha_allocation.l.reference =["landing","sload"]; 
end

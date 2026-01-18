% 複数機の単機SuspendedLoad制御を同時に回すシミュレーションモード
% それぞれの機体は mode/SuspendedLoad/SimSuspendedLoad と同じ制御器を持ち、
% Motiveシミュレータの剛体を個別に割り当てて走らせる。

numDrones = 3;
ts = 0; dt = 0.025; te = 30;
time        = TIME(ts,dt,te);
in_prog_func = @(app) dfunc(app);
post_func    = @(app) dfunc(app);
logger = LOGGER(1:numDrones, size(ts:dt:te, 2), 0, [],[]);

%% 各剛体に対応するMotiveシミュレータを作成
motiveCells = cell(2*numDrones,1);
for i = 1:numDrones
    motiveCells{2*i-1} = {2*i-1,"p","q"};
    motiveCells{2*i}   = {2*i,"pL","pT"};
end
motive = Connector_Natnet_sim(dt, motiveCells);

agent = repmat(DRONE,1,numDrones);

for i = 1:numDrones
    agent(i) = configure_sim_agent(agent(i), dt, i);
end

motive.getData(agent);
for i = 1:numDrones
    rigidPair = [2*i-1, 2*i];
    agent(i).sensor.motive = MOTIVE(agent(i), Sensor_Motive(rigidPair,0, motive));
end

run("ExpBase");

%% loop helper (no GUI plotting for now)
function dfunc(varargin); end

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

agentObj.estimator = struct();
agentObj.estimator.ekf = EKF(agentObj, Estimator_EKF(agentObj,dt,...
    MODEL_CLASS(agentObj,Model_Suspended_Load(dt, initial_state, idx,agentObj,"Load_mL_HL")),...
    ["p", "q", "pL", "pT"],"sensor_func",@sl_sensor_func));
agentObj.estimator.loadstate = SUSPENDED_LOAD_STATE_MANAGER(agentObj);

agentObj.reference.timevarying = TIME_VARYING_REFERENCE(agentObj,...
    {"gen_ref_saddle",{"freq",15,"orig",[0;0;1],"size",[0.5,0.5,0]},"HL"});
agentObj.reference.sload = SUSPENDED_LOAD_REF_ADJUST(agentObj);

agentObj.controller = HLC_SUSPENDED_LOAD(agentObj,Controller_HL_Suspended_Load(dt,agentObj));

agentObj.cha_allocation.sensor = "motive";
agentObj.cha_allocation.estimator = ["ekf","loadstate"];
agentObj.cha_allocation.reference = ["timevarying","sload"];
end

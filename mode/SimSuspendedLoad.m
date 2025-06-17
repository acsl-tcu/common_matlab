clc
ts = 0; % initial time
% dt = 0.025; % sampling period
dt = 0.025; % sampling period
te = 50; % termina time
time = TIME(ts,dt,te);
in_prog_func = @(app) in_prog(app);
post_func = @(app) post(app);
logger = LOGGER(1, size(ts:dt:te, 2), 0, [],[]); % target, number, fExp, items, agent_items, option

% drone plant setting
agent(1) = DRONE;
agent(1).parameter = DRONE_PARAM_SUSPENDED_LOAD("DIATONE");
%agent(1).parameter.set("loadmass", 0.1)
initial_state.q  = [0; 0; 0];
initial_state.w  = [0; 0; 0];
initial_state.pL = [0; 0; 0];
initial_state.vL = [0; 0; 0];
initial_state.pT = [0; 0; -1];
initial_state.wL = [0; 0; 0];
agent(1).plant = MODEL_CLASS(agent(1),Model_Suspended_Load(dt, initial_state,1,agent(1)));%dt,initial,id,agent,modelName


% Load setting
agent(2) = DRONE;
agent(2).parameter = DRONE_PARAM_SUSPENDED_LOAD("DIATONE");
initial_load.p = agent(1).plant.state.p;
initial_load.q = initial_state.q;
initial_load.pL = initial_state.pL;
initial_load.pT = initial_state.pT;
initial_load.pp = agent(1).plant.state.p;
initial_load.pq = initial_state.q;
initial_load.ppL = initial_state.pL;
initial_load.ppT = initial_state.pT;

Model.type = "discrete";
Model.name = "discrete";
Model.id = 2;
load_setting.dt = dt;
load_setting.method = @(~,u,~)u; % controller.result.input をそのまま状態に設定
load_setting.dim = [24,12,0]; % state, output, input
load_setting.state_list = ["p","q","pL","pT","pp","pq","ppL","ppT"];% pがついているのは１時刻前
load_setting.num_list = [3,3,3,3,3,3,3,3]; % dim of p, q
load_setting.initial = initial_load;
Model.param = load_setting;
agent(2).plant = MODEL_CLASS(agent(2),Model); % motiveがplantのp, qを取ってくるため plantを設定

% getDataするためにはplantを先に設定しておく必要がある
motive = Connector_Natnet_sim(2, dt, "state_name",{["p","q"],["pL","pT"]}); % imitation of Motive camera (motion capture system)
motive.getData(agent);

agent(2).estimator.do = @(obj, varargin)[];% dummy%@(varargin)struct('state',varargin{5}(1).plant.state);% dummy
agent(2).reference.do = @(obj, varargin)[];% dummy
agent(2).controller.do = @load_controller;% plant の状態に設定するため 
function result = load_controller(~,~,~,~,agent,~)
    % １時刻前の状態
    p = agent(2).plant.state.p;
    q = agent(2).plant.state.q;
    pL = agent(2).plant.state.pL;
    pT = agent(2).plant.state.pT;
    agent(2).plant.set_state("p",agent(1).plant.state.p,... % １時刻前の状態を使ったセンサー値を試す場合はこちら    
        "q",agent(1).plant.state.q,...
        "pL",agent(1).plant.state.pL,...
        "pT",agent(1).plant.state.pT,...
        "pp",p,"pq",q,"ppL",pL,"ppT",pT); 
    result.input = agent(2).plant.state; 
end
agent(2).sensor.motive = MOTIVE(agent(2), Sensor_Motive(2,0, motive));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% drone setting  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Estimator = Estimator_EKF(agent(1),dt,MODEL_CLASS(agent(1),Model_Suspended_Load(dt, initial_state, 1,agent(1),"Load_mL_HL")), ["p", "q", "pL", "pT"]);
Estimator.sensor_func = @EKF_sensor_multi_rigid;
function state = EKF_sensor_multi_rigid(self,varargin) 
r0 =self.sensor.direct.result.state;% direct sensorをそのまま使う場合
p = r0.p;
q = r0.q;
pL = r0.pL;
d = r0.pT;

% motive情報 模擬　この場合directを使う時と同じ結果になるはずだが...
% r0 = self.sensor.result.rigid; %：motive定義のstate_name = {["p","q"],["pL","q"]}
% p = r0(1).p;
% q = Quat2Eul(r0(1).q);
% d = r0(2).p - r0(1).p; % 機体から見たload位置
% pL = r0(2).p;

% r0 = self.sensor.result.pstate; % 1時刻前の drone情報が入っている ：motive定義のstate_name = {["p","q"],["p","q"]}
% p = r0.p;
% q = r0.q;
% d = r0.pL - r0.p;
% pL = r0.pL;
state = [p; q; pL; d/norm(d)];
end
agent(1).estimator.ekf = EKF(agent(1), Estimator );%expの流用
agent(1).sensor.motive = MOTIVE(agent(1), Sensor_Motive(1,0, motive));
 % agent(1).sensor.direct = DIRECT_SENSOR(agent(1), 0.002*0);
agent(1).sensor.direct = DIRECT_SENSOR(agent(1), 0.002*0,struct("do",@SensorDo));
function result = SensorDo(varargin)
tmp = varargin{5}(1).plant.state.get();
varargin{5}(1).sensor.direct.result.state.set_state(tmp);
    s = varargin{5}(2).plant.state;
    pstate = struct("p",s.pp,"q",s.pq,"pL",s.ppL,"pT",s.ppT); % p付き変数なら１時刻前の状態
    result.pstate = pstate;
end

agent(1).reference.timevarying = TIME_VARYING_REFERENCE(agent(1),{"gen_ref_saddle",{"freq",10,"orig",[0;0;1],"size",[2,2,0.5]},"HL"});
dummy_state = state_copy(agent(1).reference.timevarying.result.state);
agent(1).reference.dummy = struct("do",@(varargin) varargin{5}(1).reference.dummy.result, "result",struct("state",dummy_state));

agent(1).controller = HLC_SUSPENDED_LOAD(agent(1),Controller_HL_Suspended_Load(dt,agent(1)));
run("ExpBase");
agent(1).cha_allocation.sensor = ["motive","direct"];
agent(1).cha_allocation.estimator = "ekf";
% agent(1).cha_allocation.reference = "timevarying";
agent(1).cha_allocation.f.reference = "timevarying";
agent(1).cha_allocation.a.reference = "dummy";

agent(2).cha_allocation.sensor = "motive";
agent(2).cha_allocation.l = [];
agent(2).cha_allocation.t = [];

%%
% clc
% for i = 1:time.te
%     if i < 20 || rem(i, 10) == 0, i, end
%     agent(1).sensor.do(time, 'f');
%     agent(1).estimator.do(time, 'f');
%     agent(1).reference.do(time, 'f');
%     agent(1).controller.do(time, 'f',0,0,agent,1);
%     agent(1).plant.do(time, 'f');
%     logger.logging(time, 'f', agent);
%     time.t = time.t + time.dt;
%     %pause(1)
% end

%%
% logger.plot({1,"plant.result.state.pL","p"},{1,"input",""})
% logger.plot({1,"plant.result.state.pL","p"})
%%

function post(app)
app.logger.plot({1, "controller.result.xd1:3", ""},"ax",app.UIAxes3,"xrange",[app.time.ts,app.time.te]);
% app.logger.plot({1, "controller.result.x8:10", ""},"ax",app.UIAxes2,"xrange",[app.time.ts,app.time.te]);
app.logger.plot({1, "estimator.result.state.pL", ""},"ax",app.UIAxes2,"xrange",[app.time.ts,app.time.te]);
app.logger.plot({1, "p", "re"},"ax",app.UIAxes,"xrange",[app.time.ts,app.time.te]);
% app.logger.plot({1, "input", ""},"ax",app.UIAxes4,"xrange",[app.time.ts,app.time.te]);
%app.logger.plot({1, "input", ""},"ax",app.UIAxes4,"xrange",[app.time.ts,app.time.te]);
% figure();
% app.logger.plot({1, "estimator.result.state.wL", ""},"ax",gca,"xrange",[app.time.ts,app.time.te]);
% figure();
% app.logger.plot({1, "estimator.result.state.vL", ""},"ax",gca,"xrange",[app.time.ts,app.time.te]);
% app.logger.plot({1, "inner_input", ""},"ax",app.UIAxes6,"xrange",[app.time.ts,app.time.te]);

%%以下他figureで表示させるもの
figuure();
ax1=subplot(2,3,1);
app.logger.plot({1, "p", "er"},"ax",ax1,"xrange",[app.time.ts,app.time.te]);
figuure();
ax2=subplot(2,3,2);
app.logger.plot({1, "q", "e"},"ax",ax2,"xrange",[app.time.ts,app.time.te]);
figuure();
ax3=subplot(2,3,3);
app.logger.plot({1, "v", "e"},"ax",ax3,"xrange",[app.time.ts,app.time.te]);
figuure();
ax4=subplot(2,3,4);
app.logger.plot({1, "p1-p2", "pre"},"ax",ax4,"xrange",[app.time.ts,app.time.te]);
figuure();
ax5=subplot(2,3,5);
app.logger.plot({1, "q", "s"},"ax",ax5,"xrange",[app.time.ts,app.time.te]);
figuure();
ax6=subplot(2,3,6);
app.logger.plot({1, "input", ""},"ax",ax6,"xrange",[app.time.ts,app.time.te]);
end
function in_prog(app)
app.TextArea.Text = ["estimator : " + app.agent(1).estimator.result.state.get()];
end
clc
ts = 0; % initial time
dt = 0.025;%0.025; % sampling period
% dt=0.03%粉砕
te = 10000; % termina time
time = TIME(ts,dt,te);
in_prog_func = @(app) in_prog(app);
post_func = @(app) post(app);
 % logger = LOGGER(1, size(ts:dt:te, 2), 0, [],[]);%今までのやつ
logger = LOGGER(1, size(ts:dt:te, 2), 1, [],[]);

motive = Connector_Natnet('192.168.1.4'); % connect to Motive
motive.getData([], []); % get data from Motive
Drone = motive.result.rigid(1);
Load  = motive.result.rigid(2);
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
% agent.controller = HLC_SUSPENDED_LOAD(agent,Controller_HL_Suspended_Load(dt,agent));
%=============================================================================================

agent = DRONE;
agent.plant = DRONE_EXP_MODEL(agent,Model_Drone_Exp(dt, initial_state, "serial", "COM7"));%有線プロポ
agent.parameter = DRONE_PARAM_SUSPENDED_LOAD("DIATONE");
agent.parameter.set("cableL",0.99);%0.992,0.647,p0.613,0.460
agent.parameter.set("loadmass",0.05);%0.0968);%0.968
agent.sensor.motive = MOTIVE(agent, Sensor_Motive(1,0, motive)); % rigid_id,initial_yaw_angle,motive
Estimator = Estimator_EKF(agent,dt,MODEL_CLASS(agent,Model_Suspended_Load(dt, initial_state, 1,agent,"Load_mL_HL")), ["p", "q", "pL", "pT"]);
Estimator.sensor_func = @EKF_sensor_multi_rigid;
agent.estimator.ekf = EKF(agent, Estimator);
function state = EKF_sensor_multi_rigid(self,~) 
s = self.sensor.result.rigid;
ids = [1,2];
p = s(ids(1)).p;
q = Quat2Eul(s(ids(1)).q);
pL = s(ids(2)).p;
d = pL - p;
pT = d/norm(d);
state = [p;q;pL;pT];
end
agent.estimator.model = agent.estimator.ekf.model;
agent.input_transform = THRUST2THROTTLE_DRONE(agent,InputTransform_Thrust2Throttle_drone()); % 推力からスロットルに変換
agent.reference.timevarying = TIME_VARYING_REFERENCE(agent,{"gen_ref_saddle",{"freq",12,"orig",[0;0;0.5],"size",[1,1,0.2*0]*1},"HL"});
dummy_state = state_copy(agent.reference.timevarying.result.state);

agent.reference.dummy = struct("do",@(varargin) varargin{5}.reference.dummy.result, "result",struct("state",dummy_state));
agent.controller = HLC_SUSPENDED_LOAD(agent,Controller_HL_Suspended_Load(dt,agent));
run("ExpBase");
agent(1).cha_allocation.sensor = "motive";
agent(1).cha_allocation.estimator = "ekf";
agent(1).cha_allocation.f.reference = "timevarying";
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
% function result = sensor_do(varargin)
%     result_motive = varargin{5}.sensor.motive.do(varargin);
%     result_forload = varargin{5}.sensor.forload.do(varargin);
%     result_forload.state.p =  result_motive.state.p;
%     result_forload.state.q =  result_motive.state.q;
%     varargin{5}.sensor.result = result_forload;
%     result=result_forload;
% 
%     % sensor = varargin{5}.sensor;
%     % result = sensor.motive.do(varargin);
%     % result = merge_result(result,sensor.forload.do(varargin));
%     % varargin{5}.sensor.result = result;
% end

function post(app)
app.logger.plot({1, "controller.result.xd1:3", ""},"ax",app.UIAxes,"xrange",[app.time.ts,app.time.te]);
% app.logger.plot({1, "controller.result.x8:10", ""},"ax",app.UIAxes2,"xrange",[app.time.ts,app.time.te]);
% app.logger.plot({1, "estimator.result.state.mL", ""},"ax",app.UIAxes2,"xrange",[app.time.ts,app.time.te]);
% app.logger.plot({1, "estimator.result.state.pL", ""},"ax",app.UIAxes3,"xrange",[app.time.ts,app.time.te]);
% app.logger.plot({1, "p", "ser"},"ax",app.UIAxes,"xrange",[app.time.ts,app.time.te]);
% app.logger.plot({1, "v", "e"},"ax",app.UIAxes3,"xrange",[app.time.ts,app.time.te]);
% app.logger.plot({1, "input", ""},"ax",app.UIAxes3,"xrange",[app.time.ts,app.time.te]);
% app.logger.plot({1, "input", ""},"ax",app.UIAxes5,"xrange",[app.time.ts,app.time.te]);
% app.logger.plot({1, "inner_input", ""},"ax",app.UIAxes2,"xrange",[app.time.ts,app.time.te]);
% %%
% app.logger.plot({1, "p", "ser"},"ax",app.UIAxes,"xrange",[app.time.ts,app.time.te]);
% % app.logger.plot({1, "inner_input", ""},"ax",app.UIAxes2,"xrange",[app.time.ts,app.time.te]);
% % app.logger.plot({1, "sensor.result.state.pL", "s"},"ax",app.UIAxes3,"xrange",[app.time.ts,app.time.te]);
% app.logger.plot({1, "estimator.result.state.pL", "e"},"ax",app.UIAxes3,"xrange",[app.time.ts,app.time.te]);
% app.logger.plot({1, "sensor.result.state.pT", "s"},"ax",app.UIAxes2,"xrange",[app.time.ts,app.time.te]);
% app.logger.plot({1, "estimator.result.state.pT", "e"},"ax",app.UIAxes3,"xrange",[app.time.ts,app.time.te]);
% app.logger.plot({1, "sensor.result.state.p", "s"},"ax",app.UIAxes2,"xrange",[app.time.ts,app.time.te]);
% app.logger.plot({1, "sensor.result.state.pL", "s"},"ax",app.UIAxes3,"xrange",[app.time.ts,app.time.te]);
% app.logger.plot({1, "input", ""},"ax",app.UIAxes4,"xrange",[app.time.ts,app.time.te]);

% 刻み時間描画
dt = diff(app.logger.Data.t(1:find(app.logger.Data.phase==0,1,'first')-1));
t = app.logger.data(0,'t',[]);
figure(100)
plot(t(1:end-1),dt);
hold on
yline(0.025,"LineWidth",0.5)
ylim([0 0.05])
hold off
grid on
legend("dt","upper limit")

% Graphplot(app)
end
function in_prog(app)
app.TextArea.Text = ["estimator : " + app.agent(1).estimator.result.state.get()];
end
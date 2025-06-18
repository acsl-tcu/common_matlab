%clc
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
initial_state.vL = [0; 0; 0];
initial_state.p = [0; 0; 0];
initial_state.pT = [0; 0; -1];
initial_state.wL = [0; 0; 0];
initial_state.pL = initial_state.p + initial_state.pT*agent(1).parameter.cableL;
initial_state.v = [0; 0; 0];
agent(1).plant = MODEL_CLASS(agent(1),Model_Suspended_Load(dt, initial_state,1,agent(1)));%dt,initial,id,agent,modelName

% getDataするためにはplantを先に設定しておく必要がある
motive = Connector_Natnet_sim(dt, {{1,"p","q"},{1,"pL","pT"}}); % imitation of Motive camera (motion capture system)
motive.getData(agent);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% drone setting  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
agent(1).estimator.ekf = EKF(agent(1), Estimator_EKF(agent(1),dt,MODEL_CLASS(agent(1),Model_Suspended_Load(dt, initial_state, 1,agent(1),"Load_mL_HL")),...
    ["p", "q", "pL", "pT"],"sensor_func",@sensor_func));%expの流用
function y = sensor_func(self,~)
p = self.sensor.result.state(1).get('p');
q = self.sensor.result.state(1).getq('3');
pL = self.sensor.result.state(2).get('p');
pT = (pL - p);
pT = pT/vecnorm(pT);
y = [p;q;pL;pT];
end
agent(1).sensor.motive = MOTIVE(agent(1), Sensor_Motive([1,2],0, motive));
agent(1).reference.timevarying = TIME_VARYING_REFERENCE(agent(1),{"gen_ref_saddle",{"freq",10,"orig",[0;0;1],"size",[2,2,1]},"HL"});
agent(1).controller = HLC_SUSPENDED_LOAD(agent(1),Controller_HL_Suspended_Load(dt,agent(1)));
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
% logger.plot({1,"plant.result.state.pL","p"},{1,"input",""})
% logger.plot({1,"plant.result.state.pL","p"})
%%

function post(app)
% app.logger.plot({{1,"p","er"},{1,"estimator.result.state.pL","e"}},{1, "input", ""},"ax",app.UIAxes,"xrange",[app.time.ts,app.time.te]);
app.logger.plot({{1,"p","er"},{1,"estimator.result.state.pL","e"}},"ax",app.UIAxes,"xrange",[app.time.ts,app.time.te]);
% figure();
% app.logger.plot({1, "estimator.result.state.wL", ""},"ax",gca,"xrange",[app.time.ts,app.time.te]);
% figure();
% app.logger.plot({1, "estimator.result.state.vL", ""},"ax",gca,"xrange",[app.time.ts,app.time.te]);
end
function in_prog(app)
app.TextArea.Text = ["estimator : " + app.agent(1).estimator.result.state.get()];
end
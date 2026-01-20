ts = 0; % initial time
dt = 0.025; % sampling period
te = 50*2; % termina time
time = TIME(ts,dt,te);
in_prog_func = @(app) in_prog(app);
post_func = @(app) post(app);
logger = LOGGER(1, size(ts:dt:te, 2), 0, [],[]); % target, number, fExp, items, agent_items, option

% drone plant setting
agent(1) = DRONE;
agent(1).parameter = DRONE_PARAM_SUSPENDED_LOAD("DIATONE");
initial_state.q  = [0; 0; 0];
initial_state.w  = [0; 0; 0];
initial_state.vL = [0; 0; 0];
initial_state.p = [0; 0; 0];
initial_state.pT = [0; 0; -1];
initial_state.wL = [0; 0; 0];
initial_state.pL = initial_state.p + initial_state.pT*agent(1).parameter.cableL;
initial_state.v = [0; 0; 0];
agent.parameter.set("loadmass",0.075);%0.0968);%0.968
agent(1).plant = MODEL_CLASS(agent(1),Model_Suspended_Load(dt, initial_state,1,agent(1)));%dt,initial,id,agent,modelName
% Note: set the model error after setting "plant"
agent.parameter.set("loadmass",0.04);%0.0968);%0.968
%agent(1).parameter.set("loadmass", 0.1)

% Sim only: getData works after setting "plant"
motive = Connector_Natnet_sim(dt, {{1,"p","q"},{1,"pL","pT"}}); % imitation of Motive camera (motion capture system)
motive.getData(agent);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% drone setting  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
est.model = MODEL_CLASS(agent(1),Model_Suspended_Load(dt, initial_state,1,agent(1)));
agent(1).estimator.ekf = EKF(agent(1), Estimator_EKF(agent(1),dt,...
    MODEL_CLASS(agent(1),Model_Suspended_Load(dt, initial_state, 1,agent(1),"Load_mL_HL")),...
    ["p", "q", "pL", "pT"],"sensor_func",@sensor_func));%expの流用 質量推定有
function y = sensor_func(self,dt,~)
p = self.sensor.result.state(1).get('p');
q = self.sensor.result.state(1).getq('3');
switch self.cha
    case 't'
        pL = p;
        pL(3) = pL(3) - self.parameter.get("cableL");       
        pT = [0;0;-1];
        pT = (pL - p);
        pT = pT/norm(pT);
        QmL = 1e4;
        if self.estimator.ekf.Q(end,end) ~= QmL
            B = blkdiag([0.5*dt^2*eye(6);dt*eye(6)],[0.5*dt^2*eye(3);dt*eye(3)],[0.5*dt^2*eye(3);dt*eye(3)],1);%
            Q = blkdiag(eye(3)*1E1,eye(3)*1E1,eye(3)*1E1,eye(3)*1E1,QmL);       % システムノイズ（Modelクラス由来）B*Q*B'(Bは単位の次元を状態に合わせる，Qは標準偏差の二乗(分散))
            R = blkdiag(eye(3)*1e-6, eye(3)*1e-6,eye(3)*1e-6,eye(3)*1e-3);    %観測ノイズ
            self.estimator.ekf.B = B;
            self.estimator.ekf.Q = Q;
            self.estimator.ekf.R = R;            
            self.estimator.ekf.result.P = eye(25);
        end
    case {'a','l'}
        pL = p;
        pL(3) = pL(3) - self.parameter.get("cableL");
        pT = [0;0;-1];
    otherwise
        pL = self.sensor.result.state(2).get('p');
        pT = (pL - p);
        pT = pT/norm(pT);
        QmL = 1e-3;
        if self.estimator.ekf.Q(end,end) ~= QmL
            B = blkdiag([0.5*dt^2*eye(6);dt*eye(6)],[0.5*dt^2*eye(3);dt*eye(3)],[0.5*dt^2*eye(3);dt*eye(3)],1);%
            Q = blkdiag(eye(3)*1E1,eye(3)*1E1,eye(3)*1E1,eye(3)*1E1,QmL);       % システムノイズ（Modelクラス由来）B*Q*B'(Bは単位の次元を状態に合わせる，Qは標準偏差の二乗(分散))
            R = blkdiag(eye(3)*1e-6, eye(3)*1e-6,eye(3)*1e-6,eye(3)*1e-6);    %観測ノイズ
            self.estimator.ekf.B = B;
            self.estimator.ekf.Q = Q;
            self.estimator.ekf.R = R;
            self.estimator.ekf.result.P = eye(25);
        end
end
self.estimator.result.state.mL
y = [p;q;pL;pT];
end
agent(1).sensor.motive = MOTIVE(agent(1), Sensor_Motive([1,2],0, motive));
agent(1).reference.timevarying = TIME_VARYING_REFERENCE(agent(1),...
    {"gen_ref_saddle",{"freq",25,"orig",[0;0;1],"size",[1,1,0]},"HL"});
% agent(1).reference.origin  = agent(1).reference.timevarying;  %揺れ抑制用（既存）
% agent(1).reference.swaymod = SWAY_REF_MOD(agent(1), SwayRefMod_Param()); %揺れ抑制
agent(1).controller = HLC_SUSPENDED_LOAD(agent(1),Controller_HL_Suspended_Load(dt,agent(1)));

run("ExpBase");
agent(1).cha_allocation.sensor = "motive";
agent(1).cha_allocation.estimator = "ekf";
agent(1).cha_allocation.f.reference = "timevarying";
% agent(1).cha_allocation.f.reference = ["origin","swaymod"]; %揺れ抑制
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

function post(app)
app.logger.plot({{1, "p", "rep"},{1, "estimator.result.state.pL", "e"}},"ax",app.UIAxes,"phase","tf");
% app.logger.plot({1, "state.wL", "ep"},"phase","tf");
app.logger.plot({1, "p", "er"},"phase","tf", "fig_num",1); % 位置: p_x,p_y,p_z
% app.logger.plot({1, "q", "e"}, "phase","tf", "fig_num",2 ); % 角度: θ_roll, θ_pitch, θ_yaw
% app.logger.plot({1, "v", "er"}, "phase","tf", "fig_num",3);% 速度: v_x, v_y, v_z
% app.logger.plot({1, "w", "e"}, "phase","tf", "fig_num",4); % 角速度: ω_roll, ω_ptich, ω_yaw
app.logger.plot({1, "input", ""}, "phase","tf", "fig_num",5); % 制御入力: Thrust, roll, pitch, yaw
app.logger.plot({1, "p1-p2", "er"}, "phase","tf",  "fig_num",6); % x-y軌跡
% app.logger.plot({1, "p1-p2-p3", "er"}, "phase","tf",  "fig_num",7); % x-y-z軌跡
% figure(2)
% ax=gca;
% app.logger.plot({1, "estimator.result.state.mL", "e"},"phase","tf","ax",ax);
figure(3)
ax=gca;
% app.logger.plot({1, "state.w", "ep"},"phase","tf","ax",ax);
% 刻み時間描画
% t0id = find(app.logger.Data.phase==97,1,'last')+1;
% teid = find(app.logger.Data.phase==0,1,'first')-1;
% dt = diff(app.logger.Data.t(t0id:teid));
% t = app.logger.Data.t(t0id:teid-1);
% figure(100)
% [t,dt]
% plot(t,dt);
% % app.logger.plot({1,"p","e"})
% hold on
% % yline(0.025,"LineWidth",0.5)
% % ylim([0 0.05])
% hold off
% grid on
% legend("dt","upper limit")
% app.logger.plot({{1,"p","er"},{1,"estimator.result.state.pL","e"}},{1, "input", ""},"ax",app.UIAxes,"xrange",[app.time.ts,app.time.te]);
% app.logger.plot({{1,"estimator.result.state.mL","e"},{1,"estimator.result.state.pL","e"},{1,"p","r"}},"ax",app.UIAxes);
% app.logger.plot({{1,"estimator.result.state.mL","e"},{1,"p","re"}},"ax",app.UIAxes);
% figure();
% app.logger.plot({1, "estimator.result.state.wL", ""},"ax",gca,"xrange",[app.time.ts,app.time.te]);
% figure();
% app.logger.plot({1, "estimator.result.state.vL", ""},"ax",gca,"xrange",[app.time.ts,app.time.te]);
end
function in_prog(app)
app.TextArea.Text = ["estimator : " + app.agent(1).estimator.result.state.get()];
end
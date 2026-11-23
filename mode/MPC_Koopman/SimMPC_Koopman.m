clc
ts = 0; % initial time
dt = 0.025; % sampling period
te = 10; % terminal time
time = TIME(ts,dt,te); % instance of time class
in_prog_func = @(app) dfunc(app); % in progress plot
post_func = @(app) post(app); % function working at the "draw button" pushed.
motive = Connector_Natnet_sim(dt); % imitation of Motive camera (motion capture system)
logger = LOGGER(1, size(ts:dt:te, 2), 0, [],[]); % instance of LOOGER class for data logging
logger.display_func = @(agent, time) build_display_vector(agent, time);
logger.display_on = true;
fprintf("表示物\nref:[px, py, pz]  est:[px, py, pz]  U:[T, tx, ty, tz]\n\n");
initial_state.p = arranged_position([0, 0], 1, 1, 1); % [x, y], 機数，1, z (初期位置)
% initial_state.q = [1; 0; 0; 0];

initial_state.q = [0; 0; 0];
initial_state.v = [0; 0; 0];
initial_state.w = [0; 0; 0];

% agent = DRONE;
% agent.plant = MODEL_CLASS(agent,Model_EulerAngle(dt, initial_state, 1));
% agent.parameter = DRONE_PARAM("DIATONE");
% agent.estimator = EKF(agent, Estimator_EKF(agent,dt,MODEL_CLASS(agent,Model_EulerAngle(dt, initial_state, 1)),["p", "q"]));
% agent.sensor = MOTIVE(agent, motive);
% agent.reference = TIME_VARYING_REFERENCE(agent,{"Case_study_trajectory",{[0,0,0]},4});
% % agent.reference = MULTI_POINT_REFERENCE(agent,{struct("f",[0;0;0],"g",[0;0;1],"h",[0;0;0],"j",[0;0;1]),5});
% % agent.reference = TIME_VARYING_REFERENCE(agent,{"gen_ref_saddle",{"freq",60,"center",[0;0;1],"radius",[1,1,1]},4});
% % agent.controller = MPC_CONTROLLER_KOOPMAN_fmincon(agent,Controller_MPC_Koopman(agent)); %最適化手法：SQP
% agent.controller = MPC_CONTROLLER_KOOPMAN_quadprog(agent,Controller_MPC_Koopman(agent)); %最適化手法：QP
% run("ExpBase");

%% クープマンモデルをプラントに設定する場合

% load("EstimationResult_12state_2_7_Exp_sprine+zsprine+P2Pz_torque_incon_150data_vzからz算出.mat",'est') %vzから算出したzで学習、総推力
load("KL_26_MPC_fixed.mat","est")

A = est.A;
B = est.B;
C = est.C;
agent = DRONE;
agent.parameter = POINT_MASS_PARAM("rigid","row","A",A,"B",B,"C",C,"D",0);
plant_model = Model_Discrete(dt,initial_state,1,"FREE",agent);
plant_model.param.method = @(x,u,p) discrete_linear_model(observables_quaternion_base(x),u,p);
plant_model.param.dim(1) = size(C,1);
agent.plant = MODEL_CLASS(agent,plant_model);

% The EKF Jacobian uses rigid-body parameters in the Euler model's input order.
euler_model = Model_EulerAngle(dt,initial_state,1);
agent.parameter = DRONE_PARAM("DIATONE");
agent.parameter.parameter_order = euler_model.parameter_order;
agent.parameter.update_parameter();
agent.estimator.set_function_class("ekf", EKF(agent, Estimator_EKF(agent,dt,MODEL_CLASS(agent,euler_model),["p", "q"])));
agent.sensor.set_function_class("motive", MOTIVE(agent, motive));
trajectory_args = {"gen_ref_circle",{"freq",10,"center",[0;0;1],"radius",1.0},4};
agent.reference.set_function_class("time_varying", TIME_VARYING_REFERENCE(agent,trajectory_args));
controller_setting = Controller_MPC_Koopman(agent);
controller_setting.param.A = A;
controller_setting.param.B = B;
controller_setting.param.C = C;
controller_setting.param.ref_input = [agent.parameter.mass*agent.parameter.gravity;0;0;0];
controller_setting.param.input.u = controller_setting.param.ref_input;
% Light simulation tuning: improve tracking without the torque of the 4x case.
controller_setting.param.weight.P = diag([150;150;300]);
controller_setting.param.weight.Pf = controller_setting.param.weight.P;
if isfield(Setting,'mpc_weight')
    controller_setting.param.weight = Setting.mpc_weight;
end
agent.controller.set_function_class("mpc", MPC_CONTROLLER_KOOPMAN_quadprog_simulation(agent,controller_setting)); %最適化手法：QP

for i = 1:length(agent)
    agent(i).reference.set_function_class("takeoff", TAKEOFF_REFERENCE(agent(i),"zd",1.2,"te",3));
    agent(i).reference.set_function_class("landing", LANDING_REFERENCE(agent(i),"dt",dt,"vd",0,"te",5));
    agent(i).cha_allocation.f.reference = "time_varying";
    agent(i).cha_allocation.a.reference = "takeoff";
    agent(i).cha_allocation.t.reference = "takeoff";
    agent(i).cha_allocation.l.reference = "landing";
end

function post(app)
if app.logger.k == 0
    title(app.UIAxes,"ログがありません");
    return
end
positionFigure = figure('Name','Koopman position','NumberTitle','off');
positionAxes = axes(positionFigure);
trajectoryFigure = figure('Name','Koopman trajectory','NumberTitle','off');
trajectory = axes(trajectoryFigure);
inputFigure = figure('Name','Koopman input','NumberTitle','off');
inputs = axes(inputFigure);
samples = 1:app.logger.k;
data = app.logger.Data.agent(1);
t = app.logger.Data.t(samples);
est = cell2mat(cellfun(@(r) r.state.p(:),data.estimator.result(samples),'UniformOutput',false));
ref = cell2mat(cellfun(@(r) r.state.p(:),data.reference.result(samples),'UniformOutput',false));
u = cell2mat(data.input(samples));
lines = plot(positionAxes,t,est.','-',t,ref.','--');
for component = 1:3
    lines(component+3).Color = lines(component).Color;
end
axis(positionAxes,'tight');
title(positionAxes,'Position');
xlabel(positionAxes,'Time [s]');
ylabel(positionAxes,'Position [m]');
legend(positionAxes,{'x est','y est','z est','x ref','y ref','z ref'},'Location','best');
plot3(trajectory,est(1,:),est(2,:),est(3,:),'-',ref(1,:),ref(2,:),ref(3,:),'--');
axis(trajectory,'equal');
xlabel(trajectory,'x [m]'); ylabel(trajectory,'y [m]'); zlabel(trajectory,'z [m]');
title(trajectory,'Trajectory');
legend(trajectory,{'Estimate','Reference'});
plot(inputs,t,u.');
xlabel(inputs,'Time [s]');
title(inputs,'Thrust and torque');
legend(inputs,{'T [N]','Tx [Nm]','Ty [Nm]','Tz [Nm]'},'Location','best');
end

function dfunc(app)
post(app);
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
else
    p = [NaN; NaN; NaN];
end
u = agent(idx).controller.result.input;
v = sprintf("%c %.3f : R [%7.3f,%7.3f,%7.3f] : P [%7.3f,%7.3f,%7.3f] : U [%7.3f,%7.3f,%7.3f,%7.3f]",agent(idx).cha, time.t, xd(1:3)', p', u');
end

tmp = matlab.desktop.editor.getActive;
dir = fileparts(tmp.Filename);
if ~contains(path,dir)
    cd(erase(dir,'\mode'));
[~, tmp] = regexp(genpath('.'), '\.\\\.git.*?;', 'match', 'split');
cellfun(@(xx) addpath(xx), tmp, 'UniformOutput', false);
close all hidden; clear ; clc;
userpath('clear');
end
%%
ts = 0; % initial time
dt = 0.025; % sampling period
te = 25; % terminal time
time = TIME(ts,dt,te); % instance of time class
% in_prog_func = @(app) dfunc(app); % in progress plot
post_func = @(app) dfunc(app); % function working at the "draw button" pushed.
% motive = Connector_Natnet_sim(1, dt); % imitation of Motive camera (motion capture system)
logger = LOGGER(1, size(ts:dt:te, 2), 0, [],[]); % instance of LOOGER class for data logging
initial_state.p = arranged_position([0, 0], 1, 1, 0);
initial_state.q = [1; 0; 0; 0];
initial_state.v = [0; 0; 0];
initial_state.w = [0; 0; 0];

agent = DRONE;
agent.parameter = DRONE_PARAM("DIATONE"); %ノミナルモデル．DRONE_PARAMのパラメータを上書きしている．
agent.plant = MODEL_CLASS(agent,Model_EulerAngle(dt, initial_state, 1)); % Model_Quat13
agent.sensor = DIRECT_SENSOR(agent, 0.0); % modeファイル内で回すとき
agent.estimator = EKF(agent, Estimator_EKF(agent,dt,MODEL_CLASS(agent,Model_EulerAngle(dt, initial_state, 1)),["p", "q"]));

% agent.reference.time_varying = TIME_VARYING_REFERENCE(agent,{"gen_ref_saddle",{"freq",5,"orig",[0;0;1],"size",[1,1,0.5]},"HL"});
agent.reference.time_varying = TIME_VARYING_REFERENCE(agent,{"gen_ref_circle",{"freq",5,"init",[0;0;1],"radius",1.0},"HL"});
% agent.reference = TIME_VARYING_REFERENCE(agent,{"gen_ref_p2p",{"freq",5,"init",[0;0;1],"radius",1.0},"HL"});
agent.reference.takeoff = TAKEOFF_REFERENCE(agent,[]);
agent.reference.landing = LANDING_REFERENCE(agent,dt,0.1);
agent.cha_allocation.reference = "time_varying";
agent.cha_allocation.t.reference = "takeoff";
agent.cha_allocation.l.reference = "landing";%cha_allocationにレファレンス登録
% agent.cha_allocation = struct("reference",["time_varying"], ...
%     "t",struct("reference",["takeoff"]),"l",struct("reference","landing"));%この書き方だとcontrollerのこの行より上のcha_allocation消える

agent.controller.nominal=FUNCTIONAL_HLC(agent,Controller_FHL(dt));
agent.controller.mec=FUNCTIONAL_MECKC(agent,Controller_FHLMECK(dt));
agent.cha_allocation.controller=["nominal","mec"];%cha_allocationにコントローラー登録

function dfunc(app)
app.logger.plot({1, "p", "er"},"ax",app.UIAxes,"xrange",[app.time.ts,app.time.te],"fig_num",1,"row_col",[1 2]);
app.logger.plot({1, "q", "e"},"ax",app.UIAxes2,"xrange",[app.time.ts,app.time.te],"fig_num",2);
app.logger.plot({1, "v", "er"},"ax",app.UIAxes3,"xrange",[app.time.ts,app.time.te],"fig_num",3);
% app.logger.plot({1, "input1", ""}, "xrange",[app.time.ts,app.time.te],"fig_num",4);%スラスト
% app.logger.plot({1, "input2:4", ""}, "xrange",[app.time.ts,app.time.te],"fig_num",5);
app.logger.plot({1, "controller.result.u_nominal", ""}, "xrange",[app.time.ts,app.time.te],"fig_num",6);
app.logger.plot({1, "controller.result.delta_u", ""}, "xrange",[app.time.ts,app.time.te],"fig_num",7);

app.logger.plot({1, "p1-p2", "er"},"color", 0,"fig_num",8);
% app.logger.plot({1, "p1-p2-p3", "er"},"fig_num",9);

% un = app.agent.controller.nominal.result;%ΔuとuHLほしかった6/13(金)
% du = app.agent.controller.mec.result;
% plot([app.time.ts,app.time.te], un, [app.time.ts,app.time.te], du)
% legend('u_nominal', 'delta_u')
end
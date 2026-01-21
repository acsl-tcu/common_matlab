tmp = matlab.desktop.editor.getActive;
dir = fileparts(tmp.Filename);
if ~contains(path,dir)
    root_dir = fileparts(fileparts(dir));
    cd(root_dir);
    [~, tmp] = regexp(genpath('.'), '\.\\\.git.*?;', 'match', 'split');
    cellfun(@(xx) addpath(xx), tmp, 'UniformOutput', false);
    close all hidden; clear ; clc;
    userpath('clear');
end

%%
ts = 0; % initial time
dt = 0.025; % sampling period
te = 50; % terminal time
time = TIME(ts,dt,te); % instance of time class
in_prog_func = @(app) dfunc(app); % in progress plot
post_func = @(app) dfunc(app); % function working at the "draw button" pushed.
motive = Connector_Natnet_sim(dt); % imitation of Motive camera (motion capture system)
logger = LOGGER(1, size(ts:dt:te, 2), 0, [],[]); % instance of LOOGER class for data logging
initial_state.p = arranged_position([0, 0], 1, 1, 0);
initial_state.q = [1; 0; 0; 0];
initial_state.v = [0; 0; 0];
initial_state.w = [0; 0; 0];


agent = DRONE;
agent.parameter = DRONE_PARAM("DIATONE");
% agent.parameter = DRONE_PARAM("DIATONE", "mass", 0.7);
agent.plant = MODEL_CLASS(agent,Model_Quat13(dt, initial_state, 1));
%agent.parameter.set("mass",struct("mass",0.5))

agent.sensor.set_function_class("motive", MOTIVE(agent,motive));

agent.estimator.set_function_class("ekf", EKF(agent, Estimator_EKF(agent,dt,MODEL_CLASS(agent,Model_EulerAngle(dt, initial_state, 1)))));

agent.reference.set_function_class("time_varying", TIME_VARYING_REFERENCE(agent,{"gen_ref_circle",{"freq",10,"init",[0;0;1],"radius",1.0},"HL"}));
agent.reference.set_function_class("takeoff", TAKEOFF_REFERENCE(agent,"zd",1));
agent.reference.set_function_class("landing", LANDING_REFERENCE(agent,"dt",dt,"vd",0));

agent.controller.set_function_class("hlc", HLC(agent,Controller_HL(dt)));

agent.cha_allocation.f.reference = "time_varying";
agent.cha_allocation.a.reference="takeoff";
agent.cha_allocation.t.reference="takeoff";
agent.cha_allocation.l.reference="landing";
motive.getData(agent);

function dfunc(app)
app.logger.plot({1, "p", "er"},"ax",app.UIAxes,"phase","tfl");
% app.logger.plot({1, "p", "per"},"xrange",[app.time.ts,app.time.te]);
% app.logger.plot({1, "q", "s"},"ax",app.UIAxes2,"xrange",[app.time.ts,app.time.te]);
% app.logger.plot({1, "v", "er"},"ax",app.UIAxes3,"xrange",[app.time.ts,app.time.te]);
%app.logger.plot({1, "input", ""},"ax",app.UIAxes4,"xrange",[app.time.ts,app.time.t]);
end

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
agent.plant = MODEL_CLASS(agent,Model_Quat13(dt, initial_state, 1));
agent.estimator = EKF(agent, Estimator_EKF(agent,dt,MODEL_CLASS(agent,Model_EulerAngle(dt, initial_state, 1)),["p", "q"]));
agent.sensor = MOTIVE(agent, Sensor_Motive(1,0, motive));
% agent.reference.time_varying = TIME_VARYING_REFERENCE(agent,{"gen_ref_saddle",{"freq",7,"orig",[0;0;0.6],"size",[1,1,0]},"HL"});
 % agent.reference.time_varying = TIME_VARYING_REFERENCE(agent, {"gen_ref_heart", {"freq",20,"orig",[0 0 0.6],"size",[1 1 0],"phase",-pi/2}});
% agent.controller.hlc = HLC(agent,Controller_HL(dt));
 % agent.reference.time_varying = TIME_VARYING_REFERENCE(agent, {"gen_ref_spline", {"point",12,"order",9,"point_dt",5,"ManualSetting",0,"check",1}});
 % agent.reference.time_varying = TIME_VARYING_REFERENCE(agent, {"gen_ref_saddle", {"freq",7,"orig",[0;0;0.6],"size",[0,0,0]}, "HL"});
agent.reference.time_varying = TIME_VARYING_REFERENCE(agent,{"gen_ref_ptp", {"freq", 20}});
% agent.reference.time_varying = TIME_VARYING_REFERENCE(agent, {"gen_ref_star", {"freq", 20, "orig", [0 0 0.6], "size", [1 1 0], "phase", pi/2}}); 
agent.controller.hlmpc =HL_MPC(agent,Controller_HL_MPC(dt, agent));

agent.reference.takeoff = TAKEOFF_REFERENCE(agent,[]);
agent.reference.landing = LANDING_REFERENCE(agent,dt,0.1);
agent.cha_allocation = struct("reference","time_varying", ...
    "a",struct("reference","takeoff"), "t",struct("reference","takeoff"),"l",struct("reference","landing"));

agent.cha_allocation.controller = "hlmpc";
% agent.cha_allocation.f.controller = ["hlmpc"];
function dfunc(app)
app.logger.plot({1, "p", "er"},"ax",app.UIAxes,"xrange",[app.time.ts,app.time.te],"linewidth", 2.5, ...
    "fontsize", 14);
% app.logger.plot({1, "inner_input", ""}, "fig_num", 1,"xrange",[app.time.ts,app.time.te]);
% app.logger.plot({1, "v", "e"},"ax",app.UIAxes3,"xrange",[app.time.ts,app.time.te]);
app.logger.plot({1, "input", ""},"fig_num", 2,"xrange",[app.time.ts,app.time.te], "linewidth", 2.5, ...
    "fontsize", 14);
app.logger.plot({1, "v", "er"}, "fig_num", 3,"xrange",[app.time.ts,app.time.te], "linewidth", 2.5, ...
    "fontsize", 14);
 app.logger.plot({1, "p1-p2-p3", "er"},"fig_num", 4,"phase",'tfl', "color",0, "linewidth", 2.5, ...
    "fontsize", 14);
end
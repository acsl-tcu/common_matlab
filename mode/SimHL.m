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
te = 100; % terminal time
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
agent.plant = MODEL_CLASS(agent,Model_EulerAngle(dt, initial_state, 1));
agent.estimator = EKF(agent, Estimator_EKF(agent,dt,MODEL_CLASS(agent,Model_EulerAngle(dt, initial_state, 1)),["p", "q"]));
agent.sensor = MOTIVE(agent, Sensor_Motive(1,0, motive));
agent.controller = HLC(agent,Controller_HL(dt));
% agent.reference.time_varying = TIME_VARYING_REFERENCE(agent,{"gen_ref_saddle",{"freq",5,"orig",[0;0;1],"size",[1,1,0]},"HL"}); % circle
% agent.reference.time_varying = TIME_VARYING_REFERENCE(agent,{"gen_ref_saddle",{"freq",5,"orig",[0;0;1],"size",[0,0,0]},"HL"}); % hovering
% agent.reference.time_varying = TIME_VARYING_REFERENCE(agent,{"gen_ref_saddle",{"freq",5,"orig",[0;0;1],"size",[1,1,0.2]},"HL"}); % saddle
agent.reference.time_varying = TIME_VARYING_REFERENCE(agent,{"gen_ref_spline",{"point",20,"order",9,"point_dt",2,"ManualSetting",0}}); % random 9th spline
run("ExpBase");

agent.cha_allocation.reference = "time_varying";
motive.getData(agent);

function dfunc(app)
LW = 1.0; % Linewidth 
FS = 20; % Fontsize
app.logger.plot({1, "p", "er"},"ax",app.UIAxes,"phase","tfl", "Linewidth",LW, "Fontsize",FS);
app.logger.plot({1, "q", "e"}, "phase","tfl", "fig_num",2, "Linewidth",LW, "Fontsize",FS);
app.logger.plot({1, "v", "er"}, "phase","tfl", "fig_num",3, "Linewidth",LW, "Fontsize",FS);
app.logger.plot({1, "w", "e"}, "phase","tfl", "fig_num",4, "Linewidth",LW, "Fontsize",FS);
app.logger.plot({1, "input", ""}, "phase","tfl", "fig_num",6, "Linewidth",LW, "Fontsize",24);

app.logger.plot({1, "p1-p2", "er"}, "phase","tfl", "color", 0, "fig_num",8, "Linewidth",LW, "Fontsize",FS);
app.logger.plot({1, "p1-p2-p3", "er"}, "phase","tfl", "color", 0, "fig_num",9, "Linewidth",LW, "Fontsize",FS);

end
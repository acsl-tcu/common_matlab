dir = fileparts(mfilename('fullpath'));
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
time = TIME(ts,dt,te,1); % instance of time class
in_prog_func = @(app) dfunc(app); % in progress plot
post_func = @(app) dfunc(app); % function working at the "draw button" pushed.
motive = Connector_Natnet_sim(dt); % imitation of Motive camera (motion capture system)
logger = LOGGER(1, size(ts:dt:te, 2), 0, [],[]); % instance of LOOGER class for data logging
initial_state.p = arranged_position([0, 0], 1, 1, 0);
initial_state.q = [0; 0; 0]; % Euler angles [roll; pitch; yaw]
initial_state.v = [0; 0; 0];
initial_state.w = [0; 0; 0];


agent = DRONE;
agent.parameter = DRONE_PARAM("DIATONE");
% プラントモデル定義 ================================================================================================================================
plant_model = Model_EulerAngle(dt, initial_state, 1);
nominal_plant_param = plant_model.param.param;
% agent.parameter = DRONE_PARAM("DIATONE", "mass", 0.7);
% agent.plant = MODEL_CLASS(agent,Model_Quat13(dt, initial_state, 1));
%agent.parameter.set("mass",struct("mass",0.5))

% ↓パラメータの上書き モデル誤差をプラントに与える
plant_model.param.param(1) = 0.7875; % ５％減->0.7125 ５％増->0.7875
% plant_model.param.param(1) = 0.8;


% モデル誤差
% result確認するときこっち
% plant_model.param.param(6) = 0.21; % 0.18<jx,jy<0.22ぐらいが良き
% plant_model.param.param(7) = 0.21; %

% dataset確認するときこっち
plant_model.param.param(6) = 0.275; % 0.18<jx,jy<0.22ぐらいが良き
plant_model.param.param(7) = 0.275; %
Setting.simhl_nominal_plant_param = nominal_plant_param;
agent.plant = MODEL_CLASS(agent, plant_model);

agent.sensor.set_function_class("motive", MOTIVE(agent, motive, dt));
agent.estimator.set_function_class("ekf", EKF(agent, Estimator_EKF(agent,dt,MODEL_CLASS(agent,Model_EulerAngle(dt, initial_state, 1)),["p", "q"])));
if exist('Setting', 'var') && isfield(Setting, 'trajectory_seed')
    trajectory_seed = Setting.trajectory_seed;
else
    trajectory_seed = 1;
end

% Leave exactly one reference_args line uncommented.
% reference_args = {"gen_ref_circle",{"freq",10,"center",[0;0;0.6],"radius",1},4}; % circle
% reference_args = {"gen_ref_saddle",{"freq",10,"center",[0;0;0.6],"radius",[1,1,0.2]},4}; % saddle
reference_args = {"gen_ref_figure8",{"freq",10,"orig",[0;0;0.6],"size",[1,1,0],"phase",0}}; % figure8
% reference_args = {"gen_ref_circle",{"freq",10,"center",[0;0;0.6],"radius",0},4}; % hovering
% reference_args = {"gen_ref_spline_edmd", {"point",9,"order",9,"point_dt",5,"seed",trajectory_seed}};
Setting.simhl_trajectory = string(reference_args{1});
agent.reference.set_function_class("time_varying", TIME_VARYING_REFERENCE(agent, reference_args));


mec_model_dir = '\\192.168.100.209\ws2026\Work2026\YuriMiyazaki\kyosa_jikken\学習後model\取り直し';
addpath(mec_model_dir);
agent.controller.set_function_class("hlc", HLC(agent,Controller_HL(dt)));
%run("ExpBase");
agent.reference.set_function_class("takeoff", TAKEOFF_REFERENCE(agent,"zd",0.6));
agent.reference.set_function_class("landing", LANDING_REFERENCE(agent,"dt",dt,"vd",0.1));
agent.cha_allocation.f.reference = "time_varying";
agent.cha_allocation.a.reference = "takeoff";
agent.cha_allocation.t.reference = "takeoff";
agent.cha_allocation.l.reference = "landing";
motive.getData(agent);

function dfunc(app)
app.logger.plot({1, "p", "er"},"phase","atfl","ax",app.UIAxes);
% app.logger.plot({1, "p", "per"},"xrange",[app.time.ts,app.time.te]);
% app.logger.plot({1, "q", "s"},"ax",app.UIAxes2,"xrange",[app.time.ts,app.time.te]);
app.logger.plot({1, "v", "er"}, "fig_num", 3,"xrange",[app.time.ts,app.time.te], "linewidth", 2.5, ...
    "fontsize", 14);
app.logger.plot({1, "p1-p2-p3", "er"}, "fig_num", 4, "phase", 'tfl', "color", 0, ...
    "linewidth", 2.5, "fontsize", 14);
%app.logger.plot({1, "input", ""},"ax",app.UIAxes4,"xrange",[app.time.ts,app.time.t]);
end

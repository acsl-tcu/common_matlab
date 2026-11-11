% run_flight_SimSuspendedLoad.m
% GUI-less reproduction of SimSuspendedLoad.m using the same model,
% observer, reference, controller, sensor, and postprocessing settings.
%
% This script corresponds to SimBaseMode(3) = "SimSuspendedLoad" in the
% uploaded mainGUI.m, NOT to the currently selected SimBaseMode(4).
% It deliberately contains no CBF, replanner, obstacle, or corridor logic.
%
% Only the SimExp GUI and its UI callbacks have been replaced by a
% sequential simulation loop; the original SimExp.m was not provided.

%% Initialize MATLAB path (same project-root convention as mainGUI.m)
clear all;
close all hidden;
clc;

% Unlike the earlier headless script, this does not rely on the MATLAB
% editor being open or on a particular substring in the script filename.
script_dir = fileparts(mfilename('fullpath'));
if ~isempty(script_dir)
    cd(script_dir);
end
[~, tmp] = regexp(genpath('.'), '\.\\\.git.*?;', 'match', 'split');
cellfun(@(xx) addpath(xx), tmp, 'UniformOutput', false);
userpath('clear');

%% SimSuspendedLoad.m: time, logger and display function
% Original settings: ts=0, dt=0.025, te=50.
ts = 0;
dt = 0.025;
te = 50;
time = TIME(ts,dt,te);
logger = LOGGER(1, size(ts:dt:te, 2), 0, [],[]);
logger.display_func = @(agent, time) build_display_vector(agent, time);
logger.display_on = true;
fprintf("表示物\nref:[px, py, pz]  est:[px, py, pz]  U:[T, tx, ty, tz]  mL\n\n");

%% SimSuspendedLoad.m: plant and initial states (unchanged)
agent = DRONE;
agent.parameter = DRONE_PARAM_SUSPENDED_LOAD("DIATONE");
agent.parameter.set("loadmass",0.14);
agent.parameter.set("cableL",2.0);
initial_state.q  = [0; 0; 0];
initial_state.w  = [0; 0; 0];
initial_state.vL = [0; 0; 0];
initial_state.p  = [0; 0; 0];
initial_state.pT = [0; 0; -1];
initial_state.wL = [0; 0; 0];
initial_state.pL = initial_state.p + initial_state.pT*agent.parameter.cableL;
initial_state.v  = [0; 0; 0];

% Preserve the original order exactly: build the plant at loadmass=0.14,
% then change the parameter to 0.01 before constructing the estimator.
agent.plant = MODEL_CLASS(agent,Model_Suspended_Load(dt, initial_state,1,agent));
agent.parameter.set("loadmass",0.01);

%% SimSuspendedLoad.m: sensing and estimation (unchanged)
motive = Connector_Natnet_sim(dt, {{1,"p","q"},{1,"pL","pT"}});
motive.getData(agent);
agent.sensor.set_function_class("motive", MOTIVE(agent, motive, ...
    "output_func",@motive_output,"rigid_id",[1,2], ...
    "state_list",{["p","q"],"p"}));

agent.estimator.set_function_class("ekf", EKF(agent, ...
    Estimator_EKF_SuspendedLoad(agent,dt, ...
    MODEL_CLASS(agent,Model_Suspended_Load(dt, initial_state, 1,agent,"Load_mL_HL")), ...
    ["p", "q","pL","pT"])));
agent.estimator.set_function_class("loadstate", SUSPENDED_LOAD_STATE_MANAGER(agent));

%% SimSuspendedLoad.m: references and nominal HLC (unchanged)
L = agent.parameter.cableL;
% === FLIGHT WAYPOINT ROUTE (edit here) ===========================
% Route definitions are separate files in reference/WAYPOINT_REFERENCE/routes/
%   "gen_wp_straight"         : straight (open; intermediate points: no stops)
%   "gen_wp_gentle_turn"      : bent open waypoint route
%   "gen_wp_custom_points"    : editable XYZ waypoint table
%   "gen_wp_circle"           : continuous circle (loops indefinitely)
%   "gen_wp_saddle"           : continuous figure-eight / saddle example
%   "gen_wp_custom_formula"   : editable periodic equation
%   "gen_wp_formula_open"     : editable finite equation, sampled to points
flightRouteGenerator = "gen_wp_straight";
flightNominalSpeed = 0.20;  % nominal route progress [m/s]
% No rampTime is necessary: computed automatically from derivative limits.
flightMaxAccel = 0.30; % maximum progress acceleration [m/s^2]
flightMaxJerk  = 0.50; % maximum progress jerk [m/s^3]
flightMaxD6    = 1.00; % maximum 6th derivative of distance progress [m/s^6]
% Feedback governor. Compare estimated LOAD position with load reference.
flightProgressFeedback = true;
flightSlowdownStart = 0.30;  % distance error to start slowing [m]
flightSlowdownFull  = 0.90;  % distance error at minimum progress rate [m]
flightMinProgressScale = 0.05;  % >=0, residual progress rate
flightFilterTime = 0.70;  % each stage's first-order time constant [s]
% Shape and waypoints are edited inside the chosen gen_wp_*.m file.
% All generators accept "speed" and "rampTime"; route-specific options
% such as "radius" can optionally be appended to the parameter cell.
flightRouteParams = {"speed",flightNominalSpeed, ...
    "maxAccel",flightMaxAccel,"maxJerk",flightMaxJerk, ...
    "maxD6",flightMaxD6};
flightProgressSettings = struct( ...
    'enabled',flightProgressFeedback, ...
    'slowdownStart',flightSlowdownStart, ...
    'slowdownFull',flightSlowdownFull, ...
    'minScale',flightMinProgressScale, ...
    'filterTime',flightFilterTime);
agent.reference.set_function_class("waypoint", WAYPOINT_REFERENCE(agent, ...
    {char(flightRouteGenerator),flightRouteParams,6,flightProgressSettings}));
% ================================================================
agent.reference.set_function_class("sload", SUSPENDED_LOAD_REF_ADJUST(agent));
agent.reference.set_function_class("takeoff", TAKEOFF_REFERENCE(agent,"zd",3.0,"te",5));
agent.reference.set_function_class("landing", LANDING_REFERENCE(agent,"dt",dt,"zd",-L,"te",3));

% This is the original controller. No CBF wrapper or safety filter is used.
agent.controller.set_function_class("hlc_suspended", ...
    HLC_SUSPENDED_LOAD(agent,Controller_HL_Suspended_Load(dt,agent)));

agent.set_cha_allocation_for_all("sensor","motive");
agent.set_cha_allocation_for_all("estimator",["ekf","loadstate"]);
agent.cha_allocation.a.reference = ["takeoff","sload"];
agent.cha_allocation.t.reference = ["takeoff","sload"];
agent.cha_allocation.f.reference = ["waypoint","sload"];
agent.cha_allocation.l.reference = ["landing","sload"];

env = [];

%% Obstacle/ground GEOMETRY ONLY (not connected to HLC, CBF or planner yet)
% To switch a placement, change sceneName only. Scene contents live in
% environment/DRONE_OBSTACLES/scenes/scene_*.m .
sceneName = 'scene_gate_square';
approxMethod = 'multiple_ellipsoids';  % one of 6 methods
chosenGround = 'ground_z0';            % or 'none'
obstacleOptions = struct();
obstacleOptions.subdivisions = [2 2 1];
obstacleOptions.superSubdivisions = [1 1 1];
obstacleOptions.superExponent = 8;
obstacleOptions.hollowSides = 24;
obstacleOptions.polySides = 16;
% OPTIONAL: choose a different approximation for a named obstacle:
% obstacleOptions.perObstacle.square_gate = 'convex_polyhedra';
worldObstacles = DRONE_OBSTACLES(sceneName,approxMethod,chosenGround,obstacleOptions);
fprintf('Obstacle definition loaded: %s, approximation %s, %d parts\n', ...
    sceneName,approxMethod,numel(worldObstacles.parts));
% VIEW SETTINGS: switch only these flags; obstacles still do not affect HLC.
saveObstacle4Views = false;  % true: save a 3D+XY+XZ+YZ snapshot
saveFlightMovie = false;     % true: save MP4 of drone + obstacles
if saveObstacle4Views
    if ~exist('Data','dir'), mkdir('Data');end
    plot_obstacles_4views(worldObstacles,'time',0, ...
        'filename',fullfile('Data','ObstacleViews.png'));
end
% Static 3D/XY/XZ/YZ snapshots (use only when needed):
% plot_obstacles_4views(worldObstacles,'time',0,'filename','obstacles_views.png');
% Dynamic-only preview (optional; motion is not fed to controller):
% animate_obstacles_only(worldObstacles,0:0.1:10,'obstacles_motion.mp4');
% env remains [] intentionally; no control/safety effect at this stage.


%% Sequential, GUI-less substitute for SimExp initialization
% The warmup convention and property evaluation order are taken from the
% provided headless runner. Exact equivalence to SimExp cannot be asserted
% without SimExp.m.
for cha_init = ['a','t','f','l']
    agent.cha = cha_init;
    do_prop_local(time, cha_init, "0", logger, env, agent, "sensor",          1, false);
    do_prop_local(time, cha_init, "0", logger, env, agent, "estimator",       1, false);
    do_prop_local(time, cha_init, "0", logger, env, agent, "reference",       1, false);
    do_prop_local(time, cha_init, "0", logger, env, agent, "controller",      1, false);
    do_prop_local(time, cha_init, "0", logger, env, agent, "input_transform", 1, false);
end

%% Arming at t = 0 (one log entry)
time.t = ts;
cha = 'a';
agent.cha = cha;
motive.getData(agent);
start_total = tic;
do_prop_local(time, cha, cha, logger, env, agent, "sensor",          1, true);
do_prop_local(time, cha, cha, logger, env, agent, "estimator",       1, true);
do_prop_local(time, cha, cha, logger, env, agent, "reference",       1, true);
do_prop_local(time, cha, cha, logger, env, agent, "controller",      1, true);
do_prop_local(time, cha, cha, logger, env, agent, "input_transform", 1, true);
do_prop_local(time, cha, cha, logger, env, agent, "plant",           1, true);
agent.controller.result.controllertime_total = toc(start_total);
logger.logging(time, cha, agent, []);
time.k = logger.k;

%% Run takeoff -> flight -> landing without opening SimExp GUI
% The 5 s and 3 s boundaries reflect the original takeoff/landing refs.
% The provided headless runner also used t<5 and t>=47 for these phases.
% Index 1 was consumed by arming: do not execute t=0 a second time.
times = ts:dt:te;
for sample_idx = 2:numel(times)
    time.t = times(sample_idx);
    if time.t < 5.0
        cha = 't';
    elseif time.t < 47.0
        cha = 'f';
    else
        cha = 'l';
    end
    agent.cha = cha;

    motive.getData(agent);
    start_total = tic;
    do_prop_local(time, cha, cha, logger, env, agent, "sensor",          1, true);
    do_prop_local(time, cha, cha, logger, env, agent, "estimator",       1, true);
    do_prop_local(time, cha, cha, logger, env, agent, "reference",       1, true);
    do_prop_local(time, cha, cha, logger, env, agent, "controller",      1, true);
    do_prop_local(time, cha, cha, logger, env, agent, "input_transform", 1, true);
    do_prop_local(time, cha, cha, logger, env, agent, "plant",           1, true);
    agent.controller.result.controllertime_total = toc(start_total);
    logger.logging(time, cha, agent, []);
    time.k = logger.k;
end

%% SimSuspendedLoad.m: postprocessing without app.UIAxes
% Only app.UIAxes (the GUI destination) is replaced by MATLAB figure 1.
logger.plot({{1, "p", "er"},{1, "estimator.result.state.pL", "e"}}, ...
    "phase","tfl","fig_num",1);
logger.plot({{1, "p", "er"},{1, "estimator.result.state.pL", "e"}}, ...
    "phase","f","fig_num",2);
logger.plot({{1, "v1:2", "er"},{1,"estimator.result.state.vL1:2",""}}, ...
    "fig_num",3);
logger.plot({1, "controller.result.controllertime_total", ""}, ...
    "phase","f","fig_num",4);

if logger.k > 1
    % Original drone/load drawing + original shape (opaque) + approximation (translucent).
    % For a dynamic scene, the obstacles move independently with time.t.
    mov = DRAW_SUSPENDED_LOAD(logger, "target", 1, "self", agent(1), ...
        "obstacles", worldObstacles);
    if saveFlightMovie
        mov.animation(logger, "target", 1, "self", agent(1), ...
            "mp4", fullfile('Data','FlightWithObstacles.mp4'), "pause", 0);
    else
        mov.animation(logger, "target", 1, "self", agent(1));
    end
end

%% Local functions, moved to end as required for MATLAB scripts
function y = motive_output(obj,data)
% Identical conversion to SimSuspendedLoad.m.
p = data.rigid(obj.rigid_id(1)).p;
pT = data.rigid(obj.rigid_id(2)).p - p;
pT = pT/norm(pT);
y = [p;Quat2Eul(data.rigid(obj.rigid_id(1)).q);data.rigid(obj.rigid_id(2)).p;pT];
end

function do_prop_local(time, cha, fCha, logger, env, agent, prop, i, isReady)
% Same headless dispatch convention as the uploaded runner.
if ~isReady && strcmp(prop, "plant")
    return;
end
if isfield(agent(i).cha_allocation, cha) && ...
        isfield(agent(i).cha_allocation.(cha), prop)
    list = agent(i).cha_allocation.(cha).(prop);
else
    list = [];
end
if isempty(list)
    agent(i).(prop).result = agent(i).(prop).do(time, fCha, logger, env, agent, i);
else
    agent(i).(prop).result = agent(i).(prop).(list(1)).do(time, fCha, logger, env, agent, i);
    for j = 2:length(list)
        agent(i).(prop).result = merge_result( ...
            agent(i).(prop).result, ...
            agent(i).(prop).(list(j)).do(time, fCha, logger, env, agent, i));
    end
end
end

function v = build_display_vector(agent, time)
% Identical display logic to SimSuspendedLoad.m.
idx = 1;
if ~isprop(agent(idx).reference.result.state, "xd")
    v = [];
    return
end
xd = agent(idx).reference.result.state.xd;
if isfield(agent(idx).estimator.result, "state")
    p = agent(idx).estimator.result.state.p;
    if isprop(agent(idx).estimator.result.state, "mL")
        mL = agent(idx).estimator.result.state.mL;
    else
        mL = NaN;
    end
else
    p = [NaN; NaN; NaN];
    mL = NaN;
end
u = agent(idx).controller.result.input;
v = sprintf("%c %.3f : R [%7.3f,%7.3f,%7.3f] : P [%7.3f,%7.3f,%7.3f] : U [%7.3f,%7.3f,%7.3f,%7.3f] : mL %7.3f", ...
    agent(idx).cha, time.t, xd(1:3)', p', u', mL);
end

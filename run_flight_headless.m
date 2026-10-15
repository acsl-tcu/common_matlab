% run_flight_headless.m
% Standalone headless flight simulation based on SimSuspendedLoad_mellinger_corridor_swing.m
clear all; close all; clc;
cf = pwd;

if contains(mfilename('fullpath'), "run_flight_headless")
    cd(fileparts(mfilename('fullpath')));
else
    tmp = matlab.desktop.editor.getActive;
    cd(fileparts(tmp.Filename));
end

[~, tmp] = regexp(genpath('.'), '\.\\\.git.*?;', 'match', 'split');
cellfun(@(xx) addpath(xx), tmp, 'UniformOutput', false);
close all hidden; clear; clc;
userpath('clear');

%% 1. INITIALIZATION (Copied from SimSuspendedLoad_mellinger_corridor_swing.m)
ts = 0; % initial time
dt = 0.025; % sampling period
te = 50; % terminal time (Full simulation time)
time = TIME(ts,dt,te);

logger = LOGGER(1, size(ts:dt:te, 2), 0, [],[]); % target, number, fExp, items, agent_items, option
logger.display_on = false; % Turn off logger display to run cleanly in background

% drone plant setting
agent = DRONE;
agent.parameter = DRONE_PARAM_SUSPENDED_LOAD("DIATONE");
agent.parameter.set("loadmass",0.14);
agent.parameter.set("cableL",2.0);
initial_state.q  = [0; 0; 0];
initial_state.w  = [0; 0; 0];
initial_state.vL = [0; 0; 0];
initial_state.p = [0; 0; 0];
initial_state.pT = [0; 0; -1];
initial_state.wL = [0; 0; 0];
initial_state.pL = initial_state.p + initial_state.pT*agent.parameter.cableL;
initial_state.v = [0; 0; 0];

agent.plant = MODEL_CLASS(agent,Model_Suspended_Load(dt, initial_state,1,agent));
agent.parameter.set("loadmass",0.01);
motive = Connector_Natnet_sim(dt, {{1,"p","q"},{1,"pL","pT"}});
motive.getData(agent);

% drone setting
agent.sensor.set_function_class("motive", MOTIVE(agent, motive,"output_func",@motive_output,"rigid_id",[1,2],"state_list",{["p","q"],"p"}));
agent.estimator.set_function_class("ekf", EKF(agent, Estimator_EKF_SuspendedLoad(agent,dt,...
    MODEL_CLASS(agent,Model_Suspended_Load(dt, initial_state, 1,agent,"Load_mL_HL")),...
    ["p", "q","pL","pT"])));
agent.estimator.set_function_class("loadstate", SUSPENDED_LOAD_STATE_MANAGER(agent));
L = agent.parameter.cableL;

% 公称直線軌道 (z方向に0.3 m/s で上昇) -> 元スクリプトのコメントですが実際は direction [0,0,1], velocity 1.5
nominal_ref = TIME_VARYING_REFERENCE(agent, ...
    {"gen_ref_line", { ...
        "p0", [0, 0, 3], ...
        "velocity", 1.5, ...
        "direction", [0, 0, 1] ...
    }, 6});

replan_opts.trigger_dist = 15.0;
replan_opts.r_load       = 0.5;
replan_opts.r_drone      = 0.5;
replan_opts.cable        = 0.5;
opts.generator_backend_flag = 1;
% replanner_instance = REPLANNING_BSPLINE_NEW(agent, nominal_ref, replan_opts);
replanner_instance = REPLANNING_SYSTEM(agent, nominal_ref, replan_opts);

agent.reference.set_function_class("timevarying", replanner_instance);
agent.reference.set_function_class("sload", SUSPENDED_LOAD_REF_ADJUST(agent));
agent.reference.set_function_class("takeoff", TAKEOFF_REFERENCE(agent,"zd",3.0,"te",5));
agent.reference.set_function_class("landing", LANDING_REFERENCE(agent,"dt",dt,"zd",-L,"te",3));
agent.controller.set_function_class("hlc_suspended", HLC_SUSPENDED_LOAD(agent,Controller_HL_Suspended_Load(dt,agent)));

agent.set_cha_allocation_for_all("sensor","motive");
agent.set_cha_allocation_for_all("estimator",["ekf","loadstate"]);
agent.cha_allocation.a.reference =["takeoff","sload"]; 
agent.cha_allocation.t.reference =["takeoff","sload"];
agent.cha_allocation.f.reference =["timevarying","sload"];
agent.cha_allocation.l.reference =["landing","sload"];

env = [];

%% 2. WARMUP (Initialization like GUI start)
disp("Initializing simulation states...");
for cha_init = ['a', 't', 'f', 'l']
    agent.cha = cha_init;
    do_prop_local(time, cha_init, "0", logger, env, agent, "sensor", 1, false);
    do_prop_local(time, cha_init, "0", logger, env, agent, "estimator", 1, false);
    do_prop_local(time, cha_init, "0", logger, env, agent, "reference", 1, false);
    do_prop_local(time, cha_init, "0", logger, env, agent, "controller", 1, false);
    do_prop_local(time, cha_init, "0", logger, env, agent, "input_transform", 1, false);
end

%% 3. ARMING STEP
disp("Arming...");
time.t = 0;
cha = 'a';
if ~isempty(motive), motive.getData(agent); end
tStartTotal = tic;
do_prop_local(time, cha, cha, logger, env, agent, "sensor", 1, true);
do_prop_local(time, cha, cha, logger, env, agent, "estimator", 1, true);
do_prop_local(time, cha, cha, logger, env, agent, "reference", 1, true);
do_prop_local(time, cha, cha, logger, env, agent, "controller", 1, true);
do_prop_local(time, cha, cha, logger, env, agent, "input_transform", 1, true);
do_prop_local(time, cha, cha, logger, env, agent, "plant", 1, true);
agent.controller.result.controllertime_total = toc(tStartTotal);
logger.logging(time, cha, agent, []);
time.k = logger.k;

%% 4. SIMULATION LOOP (Takeoff -> Flight -> Landing)
disp("Starting Simulation...");
cha = 't';

while time.t <= time.te
    % フェーズの自動切り替え
    % 0～5秒: Takeoff (t), 5秒～47秒: Flight (f), 47秒以降: Landing (l)
    if time.t < 5.0
        if cha ~= 't'
            disp("Phase: Takeoff");
            cha = 't';
        end
    elseif time.t < 47.0
        if cha ~= 'f'
            disp("Phase: Flight (Replanning start)");
            cha = 'f';
        end
    else
        if cha ~= 'l'
            disp("Phase: Landing");
            cha = 'l';
        end
    end

    if ~isempty(motive), motive.getData(agent); end
    
    tStartTotal = tic;
    do_prop_local(time, cha, cha, logger, env, agent, "sensor", 1, true);
    do_prop_local(time, cha, cha, logger, env, agent, "estimator", 1, true);
    do_prop_local(time, cha, cha, logger, env, agent, "reference", 1, true);
    do_prop_local(time, cha, cha, logger, env, agent, "controller", 1, true);
    do_prop_local(time, cha, cha, logger, env, agent, "input_transform", 1, true);
    do_prop_local(time, cha, cha, logger, env, agent, "plant", 1, true);
    agent.controller.result.controllertime_total = toc(tStartTotal);
    
    logger.logging(time, cha, agent, []);
    time.k = logger.k;
    time.t = time.t + dt;
    
    % Print progress every 5 seconds
    if mod(time.t, 5.0) < dt
        fprintf('Time: %.1f / %.1f sec (Phase: %s)\n', time.t, time.te, cha);
    end
end

disp("Simulation complete! Drawing plots...");

%% 5. POST PROCESSING & PLOTS
figure(1);
logger.plot({{1, "p", "er"},{1, "estimator.result.state.pL", "e"}},"phase","tfl");
figure(2);
logger.plot({{1, "p", "er"},{1, "estimator.result.state.pL", "e"}},"phase","f");
figure(3);
logger.plot({{1, "v1:2", "er"},{1,"estimator.result.state.vL1:2",""}});
figure(4);
logger.plot({1, "controller.result.controllertime_total", ""}, "phase", "f");

disp("Drawing animation...");
mov = DRAW_SUSPENDED_LOAD_CORRIDOR_MOVE(logger, "target", 1, "self", agent(1), "replanner", replanner_instance);
mov.animation(logger, "target", 1, "self", agent(1));

%% LOCAL FUNCTIONS
function y = motive_output(obj,data)
    p = data.rigid(obj.rigid_id(1)).p;
    pT = data.rigid(obj.rigid_id(2)).p - p;
    pT = pT/norm(pT);
    y = [p;Quat2Eul(data.rigid(obj.rigid_id(1)).q);data.rigid(obj.rigid_id(2)).p;pT];
end

function do_prop_local(time, cha, fCha, logger, env, agent, prop, i, isReady)
    if ~isReady && strcmp(prop, "plant")
        return;
    end
    
    if isfield(agent(i).cha_allocation, cha) && isfield(agent(i).cha_allocation.(cha), prop)
        list = agent(i).cha_allocation.(cha).(prop);
    else
        list = [];
    end

    if isempty(list)
        agent(i).(prop).result = agent(i).(prop).do(time, fCha, logger, env, agent, i);
    else
        agent(i).(prop).result = agent(i).(prop).(list(1)).do(time, fCha, logger, env, agent, i);
        for j = 2:length(list)
            agent(i).(prop).result = merge_result(agent(i).(prop).result, agent(i).(prop).(list(j)).do(time, fCha, logger, env, agent, i));
        end
    end
end

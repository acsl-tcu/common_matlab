% % run_flight_headless.m
% % Standalone headless flight simulation based on SimSuspendedLoad_mellinger_corridor_swing.m
% clear all; close all; clc;
% cf = pwd;
% 
% if contains(mfilename('fullpath'), "run_flight_headless")
%     cd(fileparts(mfilename('fullpath')));
% else
%     tmp = matlab.desktop.editor.getActive;
%     cd(fileparts(tmp.Filename));
% end
% 
% [~, tmp] = regexp(genpath('.'), '\.\\\.git.*?;', 'match', 'split');
% cellfun(@(xx) addpath(xx), tmp, 'UniformOutput', false);
% close all hidden; clear; clc;
% userpath('clear');
% 
% %% 1. INITIALIZATION (Copied from SimSuspendedLoad_mellinger_corridor_swing.m)
% ts = 0; % initial time
% dt = 0.025; % sampling period
% te = 50; % terminal time (Full simulation time)
% time = TIME(ts,dt,te);
% 
% logger = LOGGER(1, size(ts:dt:te, 2), 0, [],[]); % target, number, fExp, items, agent_items, option
% logger.display_on = false; % Turn off logger display to run cleanly in background
% 
% % drone plant setting
% agent = DRONE;
% agent.parameter = DRONE_PARAM_SUSPENDED_LOAD("DIATONE");
% agent.parameter.set("loadmass",0.14);
% agent.parameter.set("cableL",2.0);
% initial_state.q  = [0; 0; 0];
% initial_state.w  = [0; 0; 0];
% initial_state.vL = [0; 0; 0];
% initial_state.p = [0; 0; 0];
% initial_state.pT = [0; 0; -1];
% initial_state.wL = [0; 0; 0];
% initial_state.pL = initial_state.p + initial_state.pT*agent.parameter.cableL;
% initial_state.v = [0; 0; 0];
% 
% agent.plant = MODEL_CLASS(agent,Model_Suspended_Load(dt, initial_state,1,agent));
% agent.parameter.set("loadmass",0.01);
% motive = Connector_Natnet_sim(dt, {{1,"p","q"},{1,"pL","pT"}});
% motive.getData(agent);
% 
% % drone setting
% agent.sensor.set_function_class("motive", MOTIVE(agent, motive,"output_func",@motive_output,"rigid_id",[1,2],"state_list",{["p"],"p"}));
% agent.estimator.set_function_class("ekf", EKF(agent, Estimator_EKF_SuspendedLoad(agent,dt,...
%     MODEL_CLASS(agent,Model_Suspended_Load(dt, initial_state, 1,agent,"Load_mL_HL")),...
%     ["p", "q","pL","pT"])));
% agent.estimator.set_function_class("loadstate", SUSPENDED_LOAD_STATE_MANAGER(agent));
% L = agent.parameter.cableL;
% 
% % 公称直線軌道 (z方向に0.3 m/s で上昇) -> 元スクリプトのコメントですが実際は direction [0,0,1], velocity 1.5
% nominal_ref = TIME_VARYING_REFERENCE(agent, ...
%     {"gen_ref_line", { ...
%         "p0", [0, 0, 3], ...
%         "velocity", 1.5, ...
%         "direction", [0, 0, 1] ...
%     }, 6});
% 
% replanner_instance = nominal_ref;
% 
% agent.reference.set_function_class("timevarying", replanner_instance);
% agent.reference.set_function_class("sload", SUSPENDED_LOAD_REF_ADJUST(agent));
% agent.reference.set_function_class("takeoff", TAKEOFF_REFERENCE(agent,"zd",3.0,"te",5));
% agent.reference.set_function_class("landing", LANDING_REFERENCE(agent,"dt",dt,"zd",-L,"te",3));
% agent.controller.set_function_class("hlc_suspended", HLC_SUSPENDED_LOAD(agent,Controller_HL_Suspended_Load(dt,agent)));
% 
% agent.set_cha_allocation_for_all("sensor","motive");
% agent.set_cha_allocation_for_all("estimator",["ekf","loadstate"]);
% agent.cha_allocation.a.reference =["takeoff","sload"]; 
% agent.cha_allocation.t.reference =["takeoff","sload"];
% agent.cha_allocation.f.reference =["timevarying","sload"];
% agent.cha_allocation.l.reference =["landing","sload"];
% 
% env = [];
% 
% %% 2. WARMUP (Initialization like GUI start)
% disp("Initializing simulation states...");
% for cha_init = ['a', 't', 'f', 'l']
%     agent.cha = cha_init;
%     do_prop_local(time, cha_init, "0", logger, env, agent, "sensor", 1, false);
%     do_prop_local(time, cha_init, "0", logger, env, agent, "estimator", 1, false);
%     do_prop_local(time, cha_init, "0", logger, env, agent, "reference", 1, false);
%     do_prop_local(time, cha_init, "0", logger, env, agent, "controller", 1, false);
%     do_prop_local(time, cha_init, "0", logger, env, agent, "input_transform", 1, false);
% end
% 
% %% 3. ARMING STEP
% disp("Arming...");
% time.t = 0;
% cha = 'a';
% if ~isempty(motive), motive.getData(agent); end
% tStartTotal = tic;
% do_prop_local(time, cha, cha, logger, env, agent, "sensor", 1, true);
% do_prop_local(time, cha, cha, logger, env, agent, "estimator", 1, true);
% do_prop_local(time, cha, cha, logger, env, agent, "reference", 1, true);
% do_prop_local(time, cha, cha, logger, env, agent, "controller", 1, true);
% do_prop_local(time, cha, cha, logger, env, agent, "input_transform", 1, true);
% do_prop_local(time, cha, cha, logger, env, agent, "plant", 1, true);
% agent.controller.result.controllertime_total = toc(tStartTotal);
% logger.logging(time, cha, agent, []);
% time.k = logger.k;
% 
% %% 4. SIMULATION LOOP (Takeoff -> Flight -> Landing)
% disp("Starting Simulation...");
% cha = 't';
% 
% while time.t <= time.te
%     % フェーズの自動切り替え
%     % 0～5秒: Takeoff (t), 5秒～47秒: Flight (f), 47秒以降: Landing (l)
%     if time.t < 5.0
%         if cha ~= 't'
%             disp("Phase: Takeoff");
%             cha = 't';
%         end
%     elseif time.t < 47.0
%         if cha ~= 'f'
%             disp("Phase: Flight (Replanning start)");
%             cha = 'f';
%         end
%     else
%         if cha ~= 'l'
%             disp("Phase: Landing");
%             cha = 'l';
%         end
%     end
% 
%     if ~isempty(motive), motive.getData(agent); end
% 
%     tStartTotal = tic;
%     do_prop_local(time, cha, cha, logger, env, agent, "sensor", 1, true);
%     do_prop_local(time, cha, cha, logger, env, agent, "estimator", 1, true);
%     do_prop_local(time, cha, cha, logger, env, agent, "reference", 1, true);
%     do_prop_local(time, cha, cha, logger, env, agent, "controller", 1, true);
%     do_prop_local(time, cha, cha, logger, env, agent, "input_transform", 1, true);
%     do_prop_local(time, cha, cha, logger, env, agent, "plant", 1, true);
%     agent.controller.result.controllertime_total = toc(tStartTotal);
% 
%     logger.logging(time, cha, agent, []);
%     time.k = logger.k;
%     time.t = time.t + dt;
% 
%     % Print progress every 5 seconds
%     if mod(time.t, 5.0) < dt
%         fprintf('Time: %.1f / %.1f sec (Phase: %s)\n', time.t, time.te, cha);
%     end
% end
% 
% disp("Simulation complete! Drawing plots...");
% 
% %% 5. POST PROCESSING & PLOTS
% figure(1);
% logger.plot({{1, "p", "er"},{1, "estimator.result.state.pL", "e"}},"phase","tfl");
% figure(2);
% logger.plot({{1, "p", "er"},{1, "estimator.result.state.pL", "e"}},"phase","f");
% figure(3);
% logger.plot({{1, "v1:2", "er"},{1,"estimator.result.state.vL1:2",""}});
% figure(4);
% logger.plot({1, "controller.result.controllertime_total", ""}, "phase", "f");
% 
% disp("Drawing animation...");
% mov = DRAW_SUSPENDED_LOAD_CORRIDOR_MOVE(logger, "target", 1, "self", agent(1), "replanner", replanner_instance);
% mov.animation(logger, "target", 1, "self", agent(1));
% 
% %% LOCAL FUNCTIONS
% function y = motive_output(obj,data)
%     p = data.rigid(obj.rigid_id(1)).p;
%     pT = data.rigid(obj.rigid_id(2)).p - p;
%     pT = pT/norm(pT);
%     y = [p;Quat2Eul(data.rigid(obj.rigid_id(1)).q);data.rigid(obj.rigid_id(2)).p;pT];
% end
% 
% function do_prop_local(time, cha, fCha, logger, env, agent, prop, i, isReady)
%     if ~isReady && strcmp(prop, "plant")
%         return;
%     end
% 
%     if isfield(agent(i).cha_allocation, cha) && isfield(agent(i).cha_allocation.(cha), prop)
%         list = agent(i).cha_allocation.(cha).(prop);
%     else
%         list = [];
%     end
% 
%     if isempty(list)
%         agent(i).(prop).result = agent(i).(prop).do(time, fCha, logger, env, agent, i);
%     else
%         agent(i).(prop).result = agent(i).(prop).(list(1)).do(time, fCha, logger, env, agent, i);
%         for j = 2:length(list)
%             agent(i).(prop).result = merge_result(agent(i).(prop).result, agent(i).(prop).(list(j)).do(time, fCha, logger, env, agent, i));
%         end
%     end
% end

% run_flight_headless_CBF.m
% Original run_flight_headless with CBF configuration and mode switch.
% IMPORTANT: research prototype, safety NOT certified. Test first in MATLAB.
clear all; close all; clc;
cf = pwd; %#ok<NASGU>

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

%% 1. INITIALIZATION -- original plant and estimator, no changes
ts = 0;
dt = 0.025;
te = 50;
time = TIME(ts,dt,te);
logger = LOGGER(1, size(ts:dt:te, 2), 0, [],[]);
logger.display_on = false;

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
% NOTE: Retained as in user's original script. It creates a possible
% plant/model mass mismatch (0.14 -> 0.01); check whether intentional.
agent.parameter.set("loadmass",0.01);
motive = Connector_Natnet_sim(dt, {{1,"p","q"},{1,"pL","pT"}});
motive.getData(agent);

agent.sensor.set_function_class("motive", MOTIVE(agent, motive,"output_func",@motive_output,"rigid_id",[1,2],"state_list",{["p"],"p"}));
agent.estimator.set_function_class("ekf", EKF(agent, Estimator_EKF_SuspendedLoad(agent,dt,...
    MODEL_CLASS(agent,Model_Suspended_Load(dt, initial_state, 1,agent,"Load_mL_HL")),...
    ["p", "q","pL","pT"])));
agent.estimator.set_function_class("loadstate", SUSPENDED_LOAD_STATE_MANAGER(agent));
L = agent.parameter.cableL;

nominal_ref = TIME_VARYING_REFERENCE(agent, ...
    {"gen_ref_line", {"p0", [0, 0, 3], "velocity", 1.5, "direction", [0, 0, 1]}, 6});
replanner_instance = nominal_ref;
agent.reference.set_function_class("timevarying", replanner_instance);
agent.reference.set_function_class("sload", SUSPENDED_LOAD_REF_ADJUST(agent));
agent.reference.set_function_class("takeoff", TAKEOFF_REFERENCE(agent,"zd",3.0,"te",5));
agent.reference.set_function_class("landing", LANDING_REFERENCE(agent,"dt",dt,"zd",-L,"te",3));

%% *** CBF CHANGE A: create/register HLC_SUSPENDED_CBF ***
hlc = HLC_SUSPENDED_CBF(agent,Controller_HL_Suspended_Load(dt,agent));
hlc.cbfMode = 'virtual'; % 'virtual' or 'physical'
hlc.cbfMethod = 'S3';   % 'S1','S2','S3','K1','K2','K3'
hlc.cbfEnabled = false; % keep off during warmup, arming, takeoff

% Geometry values below are PLACEHOLDERS, not verified physical sizes.
% Measure/enclose real UAV body, cable, and payload before safety claims.
rU = 0.45;    % outer sphere radius of UAV [m] (example)
rL = 0.30;    % outer sphere radius of payload [m] (example)
rC = 0.015;   % cable cross-section radius [m] (example)
Rmax = max([rU,rL,rC]);
shapes = struct();
shapes.single = sqrt(2)*[L/2 + Rmax; Rmax];

% Model 2: series of overlapping balls, end balls cover payload and UAV.
lambda = (0:0.1:1)';
ds = L*max(diff(lambda));
rInterior = sqrt(rC^2+(ds/2)^2);
rSphere = rInterior*ones(numel(lambda),1);
rSphere(1) = max(rSphere(1),rL);  % lambda=0: payload
rSphere(end) = max(rSphere(end),rU); % lambda=1: UAV
shapes.spheres = [lambda,rSphere];

% Model 3: UAV sphere (as ellipsoid), cable prolate ellipsoid, payload sphere.
shapes.uav = rU*ones(3,1);
aC = L/2 + 0.10;
bC = rC/sqrt(1-(L/(2*aC))^2);
shapes.cable = [aC;bC];
shapes.load = rL*ones(3,1);

hlc.cbfShapes = shapes;
hlc.cbfKappa = 2*ones(6,1);
hlc.cbfOpts = struct('requireInitialChain',true,'uLimits',[0;20;1;1;1]);
hlc.cbfAux = [];
hlc.cbfAuxMode = '';
hlc.cbfTime = 0;

agent.controller.set_function_class("hlc_suspended", hlc);

agent.set_cha_allocation_for_all("sensor","motive");
agent.set_cha_allocation_for_all("estimator",["ekf","loadstate"]);
agent.cha_allocation.a.reference = ["takeoff","sload"];
agent.cha_allocation.t.reference = ["takeoff","sload"];
agent.cha_allocation.f.reference = ["timevarying","sload"];
agent.cha_allocation.l.reference = ["landing","sload"];
env = [];

%% 2. WARMUP
fprintf('CBF mode=%s / method=%s / enabled during flight phase only\n',hlc.cbfMode,hlc.cbfMethod);
disp("Initializing simulation states...");
for cha_init = ['a', 't', 'f', 'l']
    agent.cha = cha_init;
    do_prop_local(time, cha_init, "0", logger, env, agent, "sensor", 1, false);
    do_prop_local(time, cha_init, "0", logger, env, agent, "estimator", 1, false);
    do_prop_local(time, cha_init, "0", logger, env, agent, "reference", 1, false);
    do_prop_local(time, cha_init, "0", logger, env, agent, "controller", 1, false);
    do_prop_local(time, cha_init, "0", logger, env, agent, "input_transform", 1, false);
end

%% 3. ARMING
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

%% 4. SIMULATION LOOP
disp("Starting Simulation...");
cha = 't';
while time.t <= time.te
    if time.t < 5.0
        if cha ~= 't'
            disp("Phase: Takeoff");
            cha = 't';
        end
    elseif time.t < 47.0
        if cha ~= 'f'
            disp("Phase: Flight (CBF enabled)");
            cha = 'f';
            hlc.cbfAux = []; % initialize only when entering flight phase
            hlc.cbfAuxMode = '';
        end
    else
        if cha ~= 'l'
            disp("Phase: Landing (CBF disabled)");
            cha = 'l';
        end
    end

    if ~isempty(motive), motive.getData(agent); end
    tStartTotal = tic;
    do_prop_local(time, cha, cha, logger, env, agent, "sensor", 1, true);
    do_prop_local(time, cha, cha, logger, env, agent, "estimator", 1, true);
    do_prop_local(time, cha, cha, logger, env, agent, "reference", 1, true);

    %% *** CBF CHANGE B: set phase and CURRENT time before controller ***
    hlc.cbfEnabled = (cha == 'f');
    hlc.cbfTime = time.t;

    do_prop_local(time, cha, cha, logger, env, agent, "controller", 1, true);
    do_prop_local(time, cha, cha, logger, env, agent, "input_transform", 1, true);
    do_prop_local(time, cha, cha, logger, env, agent, "plant", 1, true);
    agent.controller.result.controllertime_total = toc(tStartTotal);
    logger.logging(time, cha, agent, []);

    %% *** CBF CHANGE C: diagnostics; does not alter control ***
    if hlc.cbfEnabled && ~isempty(hlc.result.cbf) && mod(time.k,40)==0
        cbf = hlc.result.cbf;
        hvals = [cbf.rows.h];
        if isempty(hvals), hmin=NaN; else, hmin=min(hvals); end
        fprintf('[CBF] t=%.3f mode=%s method=%s QP=%d min(h)=%.5g CPU=%.1f ms\n', ...
            time.t,hlc.cbfMode,hlc.cbfMethod,cbf.success,hmin,1000*cbf.totalSeconds);
    end

    time.k = logger.k;
    time.t = time.t + dt;
    if mod(time.t, 5.0) < dt
        fprintf('Time: %.1f / %.1f sec (Phase: %s)\n',time.t,time.te,cha);
    end
end

disp("Simulation complete! Drawing plots...");
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

%% LOCAL FUNCTIONS -- unchanged
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

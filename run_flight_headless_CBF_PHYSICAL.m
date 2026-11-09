% run_flight_headless_CBF_PHYSICAL.m
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
hlc = HLC_SUSPENDED_CBF_PHYSICAL(agent,Controller_HL_Suspended_Load(dt,agent));
hlc.cbfMode = 'physical'; % selected architecture, do not switch
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
hlc.cbfKappa = 5*ones(6,1); % diagnostic candidate, NOT globally tuned
hlc.cbfOpts = struct( ...
    'requireInitialChain',true, ...
    'uLimits',[0;20;1;1;1], ...
    'onMargin',0.75, ... % EARLY ACTIVATION (by itself does NOT change the command)
    'offMargin',0.85, ... % hysteresis; must remain above onMargin
    'preemptivePush',0.03, ... % EXPERIMENTAL: soft extra CBF margin before sigma reaches zero
    'preemptiveWeight',100, ... % weight on soft-target slack; hard constraints unchanged
    'preemptiveMaxRows',3, ... % only the nearest-to-activation CBF rows
    'experimentalContinueAfterPsiViolation',true, ... % DIAGNOSTIC ONLY: psi<0 is logged, does not immediately abort
    'experimentalStopAtGeometricBoundary',true, ... % stop if S3 separating-plane h<=0 (conservative)
    'experimentalTraceFile','CBF_chain_recovery_trial.csv', ... % CSV, one row per ACTIVE step
    'releaseHoldSteps',8, ...
    'releaseForceTol',0.10, ...
    'releaseMomentTol',0.02, ...
    'releaseAuxTol',0.05, ...
    'allowPhysicalFallback',false, ... % forbid hidden controller-mode change
    'monitorOnly',false, ... % ENABLE intervention; first test 'true' for diagnostics only
    'emitEvents',true);
hlc.cbfAux = [];
hlc.cbfAuxMode = '';
hlc.cbfSupervisor = struct();
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
fprintf('CBF PHYSICAL mode=%s / method=%s / flight-phase supervision\n',hlc.cbfMode,hlc.cbfMethod);
fprintf('[CBF-TRIAL] SOFT ANTICIPATION: onMargin=%.3f, offMargin=%.3f; NOT safety-certified\n', ...
    hlc.cbfOpts.onMargin,hlc.cbfOpts.offMargin);
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
fprintf(2,['[CBF-EXP] PSI CHAIN RECOVERY TRIAL ENABLED: uncertified. ' ...
    'Hard QP and actuator bounds still enforced. Stop when min S-gap h<=0.\n']);
trialStopped=false;trialStopReason='';
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
            hlc.cbfSupervisor = struct(); % initialize monitor/actuator state
            hlc.cbfAux = [];
            hlc.cbfAuxMode = '';
        end
    else
        if cha ~= 'l'
            disp("Phase: Landing (CBF active until safe release, if needed)");
            cha = 'l';
        end
    end

    if ~isempty(motive), motive.getData(agent); end
    tStartTotal = tic;
    do_prop_local(time, cha, cha, logger, env, agent, "sensor", 1, true);
    CBF_TRACE_PIPELINE_STEP(time.t,'after_sensor',agent); % READ-ONLY, t near 14.95 only
    do_prop_local(time, cha, cha, logger, env, agent, "estimator", 1, true);
    CBF_TRACE_PIPELINE_STEP(time.t,'after_estimator',agent); % READ-ONLY
    do_prop_local(time, cha, cha, logger, env, agent, "reference", 1, true);

    %% *** CBF CHANGE B: set phase and CURRENT time before controller ***
    hlc.cbfEnabled = (cha == 'f') || ...
        (isfield(hlc.cbfSupervisor,'active') && hlc.cbfSupervisor.active); % retain active filter into landing
    hlc.cbfTime = time.t;

    try
        do_prop_local(time, cha, cha, logger, env, agent, "controller", 1, true);
        CBF_TRACE_PIPELINE_STEP(time.t,'after_controller',agent); % READ-ONLY
        do_prop_local(time, cha, cha, logger, env, agent, "input_transform", 1, true);
        CBF_TRACE_PIPELINE_STEP(time.t,'after_input_transform',agent); % READ-ONLY
        do_prop_local(time, cha, cha, logger, env, agent, "plant", 1, true);
        CBF_TRACE_PIPELINE_STEP(time.t,'after_plant',agent); % READ-ONLY
    catch ME
        % A diagnosed CBF limitation terminates the experimental run
        % cleanly, without silently applying uncertified nominal commands.
        % Programming/model errors elsewhere still throw normally.
        if strncmp(ME.identifier,'CBF:',4)
            trialStopped=true;
            trialStopReason=sprintf('%s: %s',ME.identifier,ME.message);
            fprintf(2,'\n[CBF-EXP] CONTROLLED STOP at t=%.5f: %s\n',...
                time.t,trialStopReason);
            break;
        else
            rethrow(ME);
        end
    end
    agent.controller.result.controllertime_total = toc(tStartTotal);
    logger.logging(time, cha, agent, []);

    %% ACTUATOR-AWARE MONITOR (not a proof of future safety)
    % Print at 0.25 s while ACTIVE, 1 s while inactive. Avoid missing
    % intermediate force/fDot evolution immediately before QP failure.
    if hlc.cbfEnabled && ~isempty(hlc.result.cbf)
        cbf = hlc.result.cbf;
        isActive = isfield(cbf.supervisor,'active') && cbf.supervisor.active;
        if isActive, interval = 10; else, interval = 40; end
        if mod(time.k,interval)==0
            if isActive && numel(cbf.supervisor.aux)>=2
                fCBF=cbf.supervisor.aux(1);
                fDotCBF=cbf.supervisor.aux(2);
            else
                fCBF=cbf.uBase(1);
                fDotCBF=0;
            end
            fprintf(['[CBF-AA] t=%.3f status=%s hMin=%.4g ' ...
                'sigma=%.4g bad=%d dU=%.4g f=%.5g fDot=%.5g CPU=%.1f ms\n'], ...
                time.t,cbf.status,cbf.monitor.minH, ...
                cbf.monitor.minNormalizedMargin,cbf.monitor.violations, ...
                norm(cbf.uSafe-cbf.uBase),fCBF,fDotCBF, ...
                1000*cbf.totalSeconds);
            if isfield(cbf,'experimental')
                fprintf('  [CBF-EXP] badChains=%d psiMin=%+.6g psiLastMin=%+.6g recoveries=%d (UNCERTIFIED)\n',...
                    cbf.experimental.badChains,cbf.experimental.minPsi,...
                    cbf.experimental.minPsiLast,cbf.experimental.recoveryCount);
            end
            if isfield(cbf,'preemptive') && cbf.preemptive.enabled
                fprintf('  [CBF-PUSH] rows=%s target+%.3g maxSlack=%.3g\n', ...
                    mat2str(cbf.preemptive.rowIndices.'), ...
                    hlc.cbfOpts.preemptivePush,cbf.preemptive.maxSlack);
            end
        end
    end

    time.k = logger.k;
    time.t = time.t + dt;
    if mod(time.t, 5.0) < dt
        fprintf('Time: %.1f / %.1f sec (Phase: %s)\n',time.t,time.te,cha);
    end
end

if trialStopped
    fprintf(2,'[CBF-EXP] Trial stopped, not safety-certified. %s\n',trialStopReason);
    fprintf('[CBF-EXP] Active-step CSV: %s\n',hlc.cbfOpts.experimentalTraceFile);
else
    disp('[CBF-EXP] Simulation end reached; this alone is NOT a safety proof.');
end
disp("Drawing available plots...");
figure(1);
logger.plot({{1, "p", "er"},{1, "estimator.result.state.pL", "e"}},"phase","tfl");
figure(2);
logger.plot({{1, "p", "er"},{1, "estimator.result.state.pL", "e"}},"phase","f");
figure(3);
logger.plot({{1, "v1:2", "er"},{1,"estimator.result.state.vL1:2",""}});
figure(4);
logger.plot({1, "controller.result.controllertime_total", ""}, "phase", "f");

if ~trialStopped
    disp("Drawing animation...");
    mov = DRAW_SUSPENDED_LOAD_CORRIDOR_MOVE(logger, "target", 1, "self", agent(1), "replanner", replanner_instance);
    mov.animation(logger, "target", 1, "self", agent(1));
else
    disp('[CBF-EXP] Animation omitted after controlled stop; logged plots shown.');
end

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

% run_VERTICAL_HLC_FLIGHT_READY.m
% 0-CBF CONTROL EXPERIMENT -- three switchable references (order=6).
% Based on SimSuspendedLoad.m (authoritative plant + mass EKF + HLC),
% and its existing GUI-free execution loop. No replanning and NO CBF.
% No changes to the original HLC, original SimSuspendedLoad or original EKF.
% MATLAB R2021b+ recommended; this file has NOT been executed in MATLAB.
% Note: CLI/headless time/GUI event scheduling is not proven identical to GUI.
clear all; close all; clc;
cf = pwd;

scriptPath=mfilename('fullpath');
if isempty(scriptPath)
    error('VERTICAL:RunLocation','Run the saved script by its file name.');
end
cd(fileparts(scriptPath));

[~, tmp] = regexp(genpath('.'), '\.\\\.git.*?;', 'match', 'split');
cellfun(@(xx) addpath(xx), tmp, 'UniformOutput', false);
close all hidden; clear; clc;
userpath('clear');

%% 1. INITIALIZATION (from SimSuspendedLoad.m)
ts = 0; % initial time
dt = 0.025; % sampling period
te = 50; % terminal time (Full simulation time)
time = TIME(ts,dt,te);

logger = LOGGER(1, size(ts:dt:te, 2), 0, [],[]); % target, number, fExp, items, agent_items, option

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
% Use the ORIGINAL common EKF with the ORIGINAL 25-state mass-estimation
% parameter configuration, exactly as in SimSuspendedLoad.m. No EKF changes.
agent.estimator.set_function_class("ekf", EKF(agent, Estimator_EKF_SuspendedLoad(agent,dt, ...
    MODEL_CLASS(agent,Model_Suspended_Load(dt,initial_state,1,agent,"Load_mL_HL")), ...
    ["p","q","pL","pT"])));
agent.estimator.set_function_class("loadstate", SUSPENDED_LOAD_STATE_MANAGER(agent));
L = agent.parameter.cableL;

%% 1A. SELECT ONE OF THREE INDEPENDENT REFERENCE TYPES
% Numeric switch. Change only this ONE number:
%   1 = original line, 2 = 13th-degree C6 polynomial,
%   3 = degree-7 C6 B-spline.
referenceMode = 3;

% Plot windows are generated first. Enable this only if GUI animation is needed.
showAnimation = false;  % CBF-like default: show all diagnostic plots reliably

% Method 1: original constant-speed line (unchanged Symbolic Toolbox route).
lineP0        = [0 0 3];
lineVelocity  = 1.5;             % m/s
lineDirection = [0 0 1];

% Method 2: WAYPOINTS -- the polynomial path passes THROUGH every point.
polyWaypoints = [0 0 3; 0.3 0 7.5; 1.1 0.4 16.5; ...
    1.8 0.8 25.5; 1.6 1.4 34.5; 0.7 1.8 43.5; -0.2 1.2 52.5; ...
    0 0.5 61.5; 0 0 66];

% Method 3: CONTROL POINTS -- the B-spline is only shaped by these points.
% Keep this a separate setting; intermediate control points are NOT
% waypoints and the spline normally does NOT pass through them.
bsplineControlPoints = [0 0 3; 0.3 0 7.5; 1.1 0.4 16.5; ...
    1.8 0.8 25.5; 1.6 1.4 34.5; 0.7 1.8 43.5; -0.2 1.2 52.5; ...
    0 0.5 61.5; 0 0 66];

% Shared nominal speed/endpoint-ramp settings, not a shared path method.
referenceMaxSpeed = 1.65;       % m/s (sampled nominal peak speed)
referenceRampTime = 3;          % seconds: smooth start and stop, not waypoint times
referenceYaw      = 0;          % radians: yaw stays constant

% The original TIME_VARYING_REFERENCE is used ONLY for mode 1.
% Modes 2 and 3 each use their OWN spatial-path class and generator.
% Method-specific registration keys avoid labeling spatial paths as
% 'timevarying'. All three return the same 28-entry HLC jet.
switch referenceMode
    case 1
        referenceLabel='original_line';
        referenceKey="timevarying";
        nominal_ref = TIME_VARYING_REFERENCE(agent, ...
            {"gen_ref_line", {"p0",lineP0, "velocity",lineVelocity, ...
             "direction",lineDirection}, 6});
        refDuration = NaN;
        fprintf('[HLC-REF] 1 original_line: speed=%.3f m/s\n',lineVelocity);
    case 2
        referenceLabel='poly_c6';
        referenceKey="polynomial_path";
        nominal_ref = PATH_REFERENCE_POLYNOMIAL(agent, ...
            {"gen_ref_c6_polynomial", {"waypoints",polyWaypoints, ...
             "maxSpeed",referenceMaxSpeed, "rampTime",referenceRampTime, ...
             "yaw",referenceYaw}, 6});
        refDuration=nominal_ref.ref.info.duration;
        fprintf('[HLC-REF] 2 poly_c6: %d waypoints, auto-duration=%.3f s\n', ...
            size(polyWaypoints,1),refDuration);
    case 3
        referenceLabel='bspline_c6';
        referenceKey="bspline_path";
        nominal_ref = PATH_REFERENCE_BSPLINE(agent, ...
            {"gen_ref_c6_bspline", {"controlPoints",bsplineControlPoints, ...
             "maxSpeed",referenceMaxSpeed, "rampTime",referenceRampTime, ...
             "yaw",referenceYaw}, 6});
        refDuration=nominal_ref.ref.info.duration;
        fprintf('[HLC-REF] 3 bspline_c6: %d control points, auto-duration=%.3f s\n', ...
            size(bsplineControlPoints,1),refDuration);
    otherwise
        error('HLC:UnknownReferenceMode','referenceMode must be 1, 2, or 3.');
end
assert(numel(nominal_ref.func(0))==28 && all(isfinite(nominal_ref.func(0))), ...
    'HLC:ReferenceJet','HLC reference must be finite 28x1 (order 6).');
fprintf('[HLC-REF] method=%d (%s), order=6, CBF=OFF.\n', ...
    referenceMode, referenceLabel);
agent.reference.set_function_class(referenceKey, nominal_ref);
agent.reference.set_function_class("sload", SUSPENDED_LOAD_REF_ADJUST(agent));
agent.reference.set_function_class("takeoff", TAKEOFF_REFERENCE(agent,"zd",3.0,"te",5));
agent.reference.set_function_class("landing", LANDING_REFERENCE(agent,"dt",dt,"zd",-L,"te",3));
agent.controller.set_function_class("hlc_suspended", HLC_SUSPENDED_LOAD(agent,Controller_HL_Suspended_Load(dt,agent)));

agent.set_cha_allocation_for_all("sensor","motive");
agent.set_cha_allocation_for_all("estimator",["ekf","loadstate"]);
agent.cha_allocation.a.reference =["takeoff","sload"]; 
agent.cha_allocation.t.reference =["takeoff","sload"];
agent.cha_allocation.f.reference =[referenceKey,"sload"];
agent.cha_allocation.l.reference =["landing","sload"];

env = [];

fprintf('[VERTICAL/HLC] no CBF | reference=%s | original EKF + mass estimation\n',referenceLabel);
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
disp("Starting reference-comparison HLC-only flight (no GUI/no planner/no CBF)...");
cha = 't';

verticalTrace = zeros(0,11);  % read-only telemetry; full 0..50 s
trajectoryTrace = zeros(0,17); % independent of LOGGER plots
runtimeTrace = zeros(0,6); % same timing columns as CBF run
refJetTrace = zeros(0,29); % read-only [time, 0..6 derivatives of x,y,z,yaw]
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
            fprintf('Phase: Flight (reference=%s, HLC only)\n',referenceLabel);
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
    tController=tic;
    do_prop_local(time, cha, cha, logger, env, agent, "controller", 1, true);
    controllerElapsed=toc(tController);
    do_prop_local(time, cha, cha, logger, env, agent, "input_transform", 1, true);
    do_prop_local(time, cha, cha, logger, env, agent, "plant", 1, true);
    totalElapsed=toc(tStartTotal);
    agent.controller.result.controllertime_total = totalElapsed;
    
    logger.logging(time, cha, agent, []);
    verticalTrace(end+1,:) = localVerticalDiagnostic(time.t,agent); %#ok<AGROW>
    trajectoryTrace(end+1,:) = localFlightPathSnapshot(time.t,cha,agent); %#ok<AGROW>
    refJetTrace(end+1,:) = localReferenceJetSnapshot(time.t,agent); %#ok<AGROW>
    runtimeTrace(end+1,:) = [time.t,double(cha=='f'), ...
        controllerElapsed,totalElapsed,0,0]; %#ok<AGROW>
    if time.t >= 5 && mod(time.k,40)==0
        fprintf("[VERTICAL/HLC] t=%.3f zL=%.4f zd=%.4f eZ=%+.4f mLhat=%.5f f=%.3f\n", ...
            verticalTrace(end,[1,2,3,4,5,6]));
    end
    time.k = logger.k;
    time.t = time.t + dt;
    
    % Print progress every 5 seconds
    if mod(time.t, 5.0) < dt
        fprintf('Time: %.1f / %.1f sec (Phase: %s)\n', time.t, time.te, cha);
    end
end

% Matching timing table (no CBF baseline).
if ~isempty(runtimeTrace)
    timingNames={'time_s','in_flight','controller_s','total_step_s', ...
        'cbf_s','cbf_active'};
    writetable(array2table(runtimeTrace,'VariableNames',timingNames), ...
        'VERTICAL_HLC_timing.csv');
    copyfile('VERTICAL_HLC_timing.csv', ['VERTICAL_HLC_timing_' referenceLabel '.csv']);
    ftime=runtimeTrace(runtimeTrace(:,2)==1,:);
    if ~isempty(ftime)
        fprintf('[HLC-TIME] flight controller median %.1f ms, max %.1f ms; overruns %d/%d\n', ...
            1000*median(ftime(:,3)),1000*max(ftime(:,3)), ...
            nnz(ftime(:,4)>dt),size(ftime,1));
    end
end

% Saved at end, including partial trajectory if physical CBF stops.
if ~isempty(verticalTrace)
    colNames = {'time_s','zLoad_m','zDesired_m','zError_m','mLhat_kg', ...
        'thrust_N','Mx_Nm','My_Nm','Mz_Nm','S3_min_h','CBF_deltaU'};
    writetable(array2table(verticalTrace,'VariableNames',colNames),'VERTICAL_HLC_timeseries.csv');
    fprintf('[VERTICAL] telemetry saved: VERTICAL_HLC_timeseries.csv (%d rows)\n',size(verticalTrace,1));
    copyfile('VERTICAL_HLC_timeseries.csv', ...
        ['VERTICAL_HLC_timeseries_' referenceLabel '.csv']);
end

% Save exact same-format trajectory as the CBF run for visual overlays.
if ~isempty(trajectoryTrace)
    pathNames={'time_s','phase_code','uav_x_m','uav_y_m','uav_z_m', ...
        'load_x_m','load_y_m','load_z_m','ekf_uav_x_m','ekf_uav_y_m', ...
        'ekf_uav_z_m','ekf_load_x_m','ekf_load_y_m','ekf_load_z_m', ...
        'ref_load_x_m','ref_load_y_m','ref_load_z_m'};
    writetable(array2table(trajectoryTrace,'VariableNames',pathNames), ...
        'VERTICAL_HLC_path.csv');
    fprintf('[VERTICAL-PATH] saved VERTICAL_HLC_path.csv (%d rows)\n',size(trajectoryTrace,1));
    copyfile('VERTICAL_HLC_path.csv', ['VERTICAL_HLC_path_' referenceLabel '.csv']);
    fprintf('[HLC-REF] comparison copy: VERTICAL_HLC_path_%s.csv\n',referenceLabel);
end

% The CBF plotting routine uses the same obstacle source. HLC-only run
% loads obstacles here for VISUALIZATION ONLY (never passed into control).
obstaclesForPlot = [];
try
    obstaclesForPlot = ENVIRONMENT_OBSTACLE_ELLIPSE_MOVE(5.0);
    fprintf('[HLC-PLOT] %d obstacles from t=5 s shown for context ONLY; CBF is OFF.\n', ...
        numel(obstaclesForPlot));
catch ME
    warning('HLC:ObstacleVisualization', ...
        'Obstacle display unavailable (HLC unchanged): %s',ME.message);
end

% Save the reference 0..6 derivative time series without changing its source.
if ~isempty(refJetTrace)
    jetNames=cell(1,29);jetNames{1}='time_s';
    axisNames={'x','y','z','yaw'};
    for k=0:6
        for a=1:4
            jetNames{2+4*k+a-1}=sprintf('d%d_%s',k,axisNames{a});
        end
    end
    jetFile=['VERTICAL_HLC_reference_jet_' referenceLabel '.csv'];
    writetable(array2table(refJetTrace,'VariableNames',jetNames),jetFile);
    fprintf('[HLC-REF] 0..6 reference derivatives saved: %s\n',jetFile);
end

disp('Simulation complete!');
disp('Drawing HLC-only flight plots (same layout as CBF: figures 1-4)...');
try
    localPlotFlightTrace(trajectoryTrace,verticalTrace,runtimeTrace, ...
        obstaclesForPlot,false,'',['HLC / ' referenceLabel],L);
    localPlotReferenceComparison(); % saved modes on Figure 6
    localPlotReferenceDerivatives(refJetTrace,referenceLabel); % Figure 7
    drawnow;
catch ME
    warning('HLC:Plotting', ...
        'Plotting failed, but CSV files were saved: %s',ME.message);
end
if showAnimation
    try
        disp('Drawing animation (optional)...');
        mov=DRAW_SUSPENDED_LOAD(logger,'target',1,'self',agent(1));
        mov.animation(logger,'target',1,'self',agent(1));
    catch ME
        warning('VERTICAL:Animation','Animation unavailable: %s',ME.message);
    end
end


%% LOCAL VERTICAL DIAGNOSTICS (READ-ONLY)
function row = localVerticalDiagnostic(t,agent)
% [time,z_load,z_reference,z_error,estimated_load_mass,thrust,Mx,My,Mz,CBF_hmin,CBF_input_delta]
row = nan(1,11);
row(1) = double(t);
try
    est = agent.estimator.result.state;
    ref = agent.reference.result.state;
    u = agent.controller.result.input(:);
    z = est.pL(3);
    if isprop(ref,'xd')
        zd = ref.xd(3);
    else
        xdtmp = ref.get();
        zd = xdtmp(3);
    end
    row(2:4) = [double(z),double(zd),double(z-zd)];
    if isprop(est,'mL')
        row(5) = double(est.mL);
    end
    if numel(u)>=4
        row(6:9) = double(u(1:4)).';
    end
    c = agent.controller.result;
    if isfield(c,'cbf') && isstruct(c.cbf) && isfield(c.cbf,'monitor')
        row(10) = c.cbf.monitor.minH;
        if isfield(c.cbf,'uSafe') && isfield(c.cbf,'uBase')
            row(11) = norm(c.cbf.uSafe(:)-c.cbf.uBase(:));
        end
    end
catch
    % Diagnostic failure must not affect the plant/controller.
end
end

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


%% READ-ONLY REFERENCE JET (same 28-entry HLC input, 0..6 derivatives)
function row=localReferenceJetSnapshot(t,agent)
row=nan(1,29);row(1)=double(t);
try
    ref=agent.reference.result.state;
    if isprop(ref,'xd')
        xd=ref.xd;
    else
        xd=ref.get();
    end
    xd=double(xd(:));
    if numel(xd)==28
        row(2:29)=xd.';
    end
catch
    % Read-only trace must never change the controller's behavior.
end
end

%% READ-ONLY FLIGHT PATH ACQUISITION AND PLOTTING
function v=localFlightPathSnapshot(t,phase,agent)
% t, phase, true UAV xyz, true load xyz, EKF UAV xyz, EKF load xyz,
% reference load xyz. This is independent of LOGGER plotting APIs.
persistent plantWarned estimatorWarned referenceWarned
v=nan(1,17);v(1)=double(t);
switch phase
    case 't',v(2)=1;
    case 'f',v(2)=2;
    case 'l',v(2)=3;
    otherwise,v(2)=0;
end
try
    v(3:5)=double(agent.plant.state.p(:)).';
    v(6:8)=double(agent.plant.state.pL(:)).';
catch ME
    if isempty(plantWarned)
        warning('VERTICAL:FlightPathPlant', ...
            'Cannot read plant position for flight plots: %s',ME.message);
        plantWarned=true;
    end
end
try
    v(9:11)=double(agent.estimator.result.state.p(:)).';
    v(12:14)=double(agent.estimator.result.state.pL(:)).';
catch ME
    if isempty(estimatorWarned)
        warning('VERTICAL:FlightPathEstimator', ...
            'Cannot read EKF position for flight plots: %s',ME.message);
        estimatorWarned=true;
    end
end
try
    ref=agent.reference.result.state;
    if isprop(ref,'xd')
        xd=ref.xd;
    else
        xd=ref.get();
    end
    v(15:17)=double(xd(1:3)).';
catch ME
    if isempty(referenceWarned)
        warning('VERTICAL:FlightPathReference', ...
            'Cannot read reference position for flight plots: %s',ME.message);
        referenceWarned=true;
    end
end
end

function localPlotFlightTrace(path,verticalTrace,runtimeTrace,obstacles,wasStopped,stopReason,label,cableLength)
% Uses only the saved trajectory arrays. Works on a partial QP-stop run.
if isempty(path)
    warning('VERTICAL:NoFlightPath','No path data to draw.');
    return;
end
T=path(:,1); fMask=path(:,2)==2;
tr=path(:,3:5); tl=path(:,6:8);
er=path(:,9:11);el=path(:,12:14);rr=path(:,15:17);
if ~any(all(isfinite(tr),2))
    warning('VERTICAL:NoPlantTrajectory', ...
        'Plant position unavailable: check agent.plant.state.p and .pL.');
end
other=[];
if strcmp(label,'CBF') && exist('VERTICAL_HLC_path.csv','file')==2
    try
        other=readmatrix('VERTICAL_HLC_path.csv');
        if size(other,2)~=17
            other=[];
        else
            % Overlay only the SAME time interval (QP stop may occur at 15s).
            other=other(other(:,1)<=T(end)+1e-9,:);
        end
    catch
        other=[]; % comparison overlay is optional, never blocks plotting
    end
end

% 1. 3D spatial trajectory with original obstacle bounding ellipsoids.
figure(1);clf; ax=gca;hold(ax,'on');grid(ax,'on');
localDrawObstacleEllipsoids(ax,obstacles);
plot3(ax,rr(:,1),rr(:,2),rr(:,3),'k--','LineWidth',1.3,'DisplayName','吊り荷目標');
plot3(ax,tr(:,1),tr(:,2),tr(:,3),'-','Color',[0 .40 .85], ...
    'LineWidth',1.7,'DisplayName','機体・実位置');
plot3(ax,tl(:,1),tl(:,2),tl(:,3),'-','Color',[.85 .20 .13], ...
    'LineWidth',1.7,'DisplayName','吊り荷・実位置');
if ~isempty(other)
    plot3(ax,other(:,3),other(:,4),other(:,5),'--', ...
        'Color',[0 .63 .85],'LineWidth',1,'DisplayName','CBFなし機体（比較）');
    plot3(ax,other(:,6),other(:,7),other(:,8),'--', ...
        'Color',[.8 .50 .28],'LineWidth',1,'DisplayName','CBFなし荷物（比較）');
end
last=find(all(isfinite(tr),2),1,'last');
if ~isempty(last)
    plot3(ax,tr(last,1),tr(last,2),tr(last,3),'o', ...
        'MarkerSize',8,'MarkerFaceColor',[0 .4 .85],'MarkerEdgeColor','k', ...
        'DisplayName','機体の最終記録点');
    plot3(ax,tl(last,1),tl(last,2),tl(last,3),'o', ...
        'MarkerSize',7,'MarkerFaceColor',[.85 .2 .13],'MarkerEdgeColor','k', ...
        'DisplayName','荷物の最終記録点');
end
xlabel(ax,'x [m]');ylabel(ax,'y [m]');zlabel(ax,'z [m]');
title(ax,sprintf('%s | 3D軌跡（障害物は表示のみ・CBFなし）%s',label,localStopTag(wasStopped)));
view(ax,35,23);legend(ax,'Location','best');

% 2. Top view is essential: long vertical flight hides lateral avoidance.
figure(2);clf;ax=gca;hold(ax,'on');grid(ax,'on');
localDrawObstacleFootprints(ax,obstacles);
plot(ax,rr(:,1),rr(:,2),'k--','LineWidth',1.4,'DisplayName','吊り荷目標');
plot(ax,tr(:,1),tr(:,2),'-','Color',[0 .4 .85], ...
    'LineWidth',1.7,'DisplayName','機体・実位置');
plot(ax,tl(:,1),tl(:,2),'-','Color',[.85 .20 .13], ...
    'LineWidth',1.7,'DisplayName','吊り荷・実位置');
plot(ax,er(:,1),er(:,2),':','Color',[0 .65 .7], ...
    'LineWidth',1,'DisplayName','機体EKF');
plot(ax,el(:,1),el(:,2),':','Color',[.6 0 .5], ...
    'LineWidth',1,'DisplayName','吊り荷EKF');
if ~isempty(other)
    plot(ax,other(:,3),other(:,4),'--','Color',[.25 .65 .95], ...
        'LineWidth',1,'DisplayName','CBFなし機体');
    plot(ax,other(:,6),other(:,7),'--','Color',[.95 .55 .4], ...
        'LineWidth',1,'DisplayName','CBFなし荷物');
end
if ~isempty(last)
    plot(ax,tr(last,1),tr(last,2),'bo','MarkerFaceColor','b', ...
        'HandleVisibility','off');
    plot(ax,tl(last,1),tl(last,2),'ro','MarkerFaceColor','r', ...
        'HandleVisibility','off');
end
axis(ax,'equal');xlabel(ax,'x [m]');ylabel(ax,'y [m]');
title(ax,sprintf('%s | 上面図：目標・機体・吊り荷（障害物は表示のみ）',label));
legend(ax,'Location','best');

% 3. Vertical tracking and lateral motion over simulation time.
figure(3);clf;
ax1=subplot(2,1,1);hold(ax1,'on');grid(ax1,'on');
plot(ax1,T,rr(:,3),'k--','LineWidth',1.3,'DisplayName','吊り荷z目標');
plot(ax1,T,tl(:,3),'Color',[.85 .2 .13],'LineWidth',1.4,'DisplayName','吊り荷z実位置');
plot(ax1,T,el(:,3),'Color',[.6 0 .5],'LineStyle',':', ...
    'DisplayName','吊り荷z推定');
plot(ax1,T,tr(:,3),'Color',[0 .4 .85],'LineWidth',1.3,'DisplayName','機体z実位置');
plot(ax1,T,rr(:,3)+cableLength,'Color',[.2 .4 .2],'LineStyle','--', ...
    'DisplayName','機体z目標（荷物目標+紐長）');
ylabel(ax1,'高度 [m]');title(ax1,sprintf('%s | 高度追従',label));
legend(ax1,'Location','best');
ax2=subplot(2,1,2);hold(ax2,'on');grid(ax2,'on');
plot(ax2,T,rr(:,1),'k--','LineWidth',1.2,'DisplayName','目標 x');
plot(ax2,T,rr(:,2),'k:','LineWidth',1.2,'DisplayName','目標 y');
plot(ax2,T,tl(:,1),'LineWidth',1.3,'DisplayName','荷物 x');
plot(ax2,T,tl(:,2),'LineWidth',1.3,'DisplayName','荷物 y');
plot(ax2,T,tr(:,1),'--','LineWidth',1.1,'DisplayName','機体 x');
plot(ax2,T,tr(:,2),'--','LineWidth',1.1,'DisplayName','機体 y');
xlabel(ax2,'時刻 [s]');ylabel(ax2,'横方向位置 [m]');
legend(ax2,'Location','best');

% 4. Existing useful time plot, now computed from the same run data.
figure(4);clf;ax=gca;hold(ax,'on');grid(ax,'on');
if ~isempty(runtimeTrace)
    plot(ax,runtimeTrace(:,1),1000*runtimeTrace(:,3), ...
        'LineWidth',1.2,'DisplayName','HLC制御計算 [ms]');
    plot(ax,runtimeTrace(:,1),1000*runtimeTrace(:,4), ...
        'LineWidth',1.0,'DisplayName','全step [ms]');
    if strcmp(label,'CBF')
        plot(ax,runtimeTrace(:,1),1000*runtimeTrace(:,5), ...
            'LineWidth',1.1,'DisplayName','CBF単独 [ms]');
    end
end
xlabel(ax,'時刻 [s]');ylabel(ax,'計算時間 [ms]');
title(ax,sprintf('%s | 計算時間（参考・ノートPC）',label));
legend(ax,'Location','best');

% 5. CBF-only run uses this figure for CBF safety margins.
% HLC-only has NO CBF margin; report actual swing/tracking/controls instead.
if ~isempty(verticalTrace)
    figure(5);clf;
    good=all(isfinite(tr),2) & all(isfinite(tl),2);
    swingDeg=nan(size(T));
    if any(good)
        dp=tr(good,:)-tl(good,:);
        swingDeg(good)=atan2d(hypot(dp(:,1),dp(:,2)),dp(:,3));
    end
    errXY=hypot(tl(:,1)-rr(:,1),tl(:,2)-rr(:,2));
    errZ=tl(:,3)-rr(:,3);
    a1=subplot(2,2,1);hold(a1,'on');grid(a1,'on');
    plot(a1,T,swingDeg,'Color',[.75 .2 .16],'LineWidth',1.2);
    ylabel(a1,'振れ角 [deg]');title(a1,'機体－荷物の相対方向（鉛直からの角度）');
    a2=subplot(2,2,2);hold(a2,'on');grid(a2,'on');
    plot(a2,T,errXY,'LineWidth',1.2,'DisplayName','荷物の水平誤差');
    plot(a2,T,errZ,'LineWidth',1.1,'DisplayName','荷物のz誤差');
    ylabel(a2,'追従誤差 [m]');title(a2,'吊り荷の目標追従誤差');
    legend(a2,'Location','best');
    a3=subplot(2,2,3);hold(a3,'on');grid(a3,'on');
    plot(a3,verticalTrace(:,1),verticalTrace(:,6),'LineWidth',1.2);
    ylabel(a3,'推力 [N]');xlabel(a3,'時刻 [s]');title(a3,'HLC指令推力');
    a4=subplot(2,2,4);hold(a4,'on');grid(a4,'on');
    for ii=1:3
        plot(a4,verticalTrace(:,1),verticalTrace(:,6+ii), ...
            'LineWidth',1.1,'DisplayName',sprintf('M_%s',char('x'+ii-1)));
    end
    ylabel(a4,'トルク [Nm]');xlabel(a4,'時刻 [s]');
    title(a4,'HLC指令トルク');legend(a4,'Location','best');
    fprintf('[HLC-PLOT] Figure 5: cable swing, tracking errors, thrust and torques (NO CBF).\n');
    fSwing=swingDeg(fMask & isfinite(swingDeg));
    if ~isempty(fSwing)
        fprintf('[HLC-PLOT] flight-phase maximum cable swing=%.3f deg.\n',max(fSwing));
    end
end

fprintf('[VERTICAL-PLOT] 3D, XY, altitude, timing%s plotted (%d samples, t<=%.3f s).\n', ...
    localCBFSuffix(label),size(path,1),T(end));
if wasStopped
    fprintf('[VERTICAL-PLOT] Stopped trial: trajectory is PARTIAL; last integrated sample at %.3f s.\n',T(end));
    fprintf('[VERTICAL-PLOT] Stop reason: %s\n',stopReason);
end
end

function s=localStopTag(stopped)
if stopped,s='・途中停止まで';else,s='';end
end
function s=localCBFSuffix(label)
if strcmp(label,'CBF'),s=', CBF margins and yaw';else,s='';end
end

function localDrawObstacleEllipsoids(ax,obstacles)
if isempty(obstacles),return;end
[xx,yy,zz]=sphere(16);
for i=1:numel(obstacles)
    ob=obstacles(i);r=double(ob.ellipsoid_radii(:));
    center=double(ob.p_center(:));R=double(ob.R_obs);
    xyz=R*diag(r)*[xx(:).';yy(:).';zz(:).'];
    xyz=xyz+center;
    surf(ax,reshape(xyz(1,:),size(xx)),reshape(xyz(2,:),size(xx)), ...
        reshape(xyz(3,:),size(xx)), ...
        'FaceColor',[.5 .5 .5],'EdgeColor','none','FaceAlpha',0.17, ...
        'HandleVisibility','off');
    plot3(ax,center(1),center(2),center(3),'kx','MarkerSize',7, ...
        'LineWidth',1.5,'HandleVisibility','off');
    text(ax,center(1),center(2),center(3),sprintf('  obs%d',i), ...
        'FontSize',9,'HandleVisibility','off');
end
end

function localDrawObstacleFootprints(ax,obstacles)
if isempty(obstacles),return;end
th=linspace(0,2*pi,121);
for i=1:numel(obstacles)
    ob=obstacles(i); c=double(ob.p_center(:));
    r=double(ob.ellipsoid_radii(:));
    margin=0;
    if isfield(ob,'d_margin'),margin=double(ob.d_margin);end
    % Horizontal projection of axis-aligned enclosing ellipsoid, with an
    % APPROXIMATE additive margin for visual orientation only (not S3 h).
    plot(ax,c(1)+(r(1)+margin)*cos(th),c(2)+(r(2)+margin)*sin(th), ...
        '-','Color',[.6 .6 .6],'LineWidth',1.1,'HandleVisibility','off');
    text(ax,c(1),c(2),sprintf('obs%d',i),'FontSize',9, ...
        'HandleVisibility','off');
end
end

%% COMPARE THREE NO-CBF REFERENCES (read-only, uses existing saved paths)
function localPlotReferenceComparison()
% Figure 6: compare flight-phase paths only; no CBF assumptions.
modes={'original_line','poly_c6','bspline_c6'};
labels={'Original line','C6 polynomial','C6 B-spline'};
colors=[0.10 0.35 0.90; 0.85 0.30 0.15; 0.15 0.60 0.33];
paths=cell(1,3);
for k=1:3
    file=['VERTICAL_HLC_path_' modes{k} '.csv'];
    if exist(file,'file')~=2, continue; end
    try
        P=readmatrix(file);
        if size(P,2)==17 && any(P(:,2)==2)
            paths{k}=P(P(:,2)==2,:);
        end
    catch ME
        warning('HLC:ReferenceComparison','Could not read %s: %s',file,ME.message);
    end
end
if nnz(~cellfun(@isempty,paths))<2
    fprintf('[HLC-REF] Figure 6: run a second referenceMode for overlay.\n');
    return;
end
figure(6);clf;
a1=subplot(2,2,1);hold(a1,'on');grid(a1,'on');
a2=subplot(2,2,2);hold(a2,'on');grid(a2,'on');
a3=subplot(2,2,3);hold(a3,'on');grid(a3,'on');
a4=subplot(2,2,4);hold(a4,'on');grid(a4,'on');
for k=1:3
    P=paths{k};
    if isempty(P),continue;end
    plot3(a1,P(:,15),P(:,16),P(:,17),'--', ...
        'Color',colors(k,:),'LineWidth',1.1,'DisplayName',[labels{k} ' 目標']);
    plot3(a1,P(:,6),P(:,7),P(:,8),'-', ...
        'Color',colors(k,:),'LineWidth',1.4,'DisplayName',[labels{k} ' 荷物']);
    plot(a2,P(:,15),P(:,16),'--', ...
        'Color',colors(k,:),'LineWidth',1.1,'DisplayName',[labels{k} ' 目標']);
    plot(a2,P(:,6),P(:,7),'-', ...
        'Color',colors(k,:),'LineWidth',1.4,'DisplayName',[labels{k} ' 荷物']);
    plot(a3,P(:,1),P(:,8)-P(:,17), ...
        'Color',colors(k,:),'LineWidth',1.4,'DisplayName',labels{k});
    d=P(:,3:5)-P(:,6:8);
    ang=atan2d(hypot(d(:,1),d(:,2)),d(:,3));
    plot(a4,P(:,1),ang, ...
        'Color',colors(k,:),'LineWidth',1.4,'DisplayName',labels{k});
end
xlabel(a1,'x [m]');ylabel(a1,'y [m]');zlabel(a1,'z [m]');
title(a1,'飛行区間：目標と荷物の3D軌跡');view(a1,3);legend(a1,'Location','best');
xlabel(a2,'x [m]');ylabel(a2,'y [m]');title(a2,'飛行区間：上面図');
axis(a2,'equal');legend(a2,'Location','best');
xlabel(a3,'時刻 [s]');ylabel(a3,'荷物z - 目標z [m]');
title(a3,'飛行区間：高度誤差');legend(a3,'Location','best');
xlabel(a4,'時刻 [s]');ylabel(a4,'振れ角 [deg]');
title(a4,'飛行区間：吊り荷の振れ角');legend(a4,'Location','best');
fprintf('[HLC-REF] Figure 6: HLC-only reference modes compared from saved CSVs.\n');
end

%% CHECK THE REFERENCE 6TH DERIVATIVE BY PLOTTING THE ACTUAL HLC JET
function localPlotReferenceDerivatives(jet,referenceLabel)
% Do not differentiate sampled positions numerically: plot 28 entries
% already supplied to HLC, including the full 0..6th reference derivatives.
if isempty(jet) || size(jet,2)~=29,return;end
mask=jet(:,1)>=5 & jet(:,1)<47;
J=jet(mask,:);
if isempty(J) || ~any(isfinite(J(:,2:end)),'all')
    warning('HLC:ReferenceJetPlot','No finite flight-phase 28-entry reference jet.');
    return;
end
figure(7);clf;
for k=1:6
    ax=subplot(3,2,k);hold(ax,'on');grid(ax,'on');
    idx=2+4*k+(0:2);
    plot(ax,J(:,1),J(:,idx),'LineWidth',1.0);
    ylabel(ax,sprintf('d^%d p / dt^%d',k,k));
    if k>=5,xlabel(ax,'時刻 [s]');end
    title(ax,sprintf('%d階微分（HLC入力そのもの）',k));
    if k==1,legend(ax,{'x','y','z'},'Location','best');end
end
try
    sgtitle(sprintf('目標参照値の1～6階時間微分 | %s',referenceLabel));
catch
end
fprintf('[HLC-REF] Figure 7: actual 1..6th time derivatives passed to HLC.\n');
end

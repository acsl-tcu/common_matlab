% RUN_RA_SENSOR_TRIGGER_FOUR_METHODS_FLIGHT_20261114.m
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

%% Reference obstacle avoidance: ONE authoritative obstacle method
% Obstacle approximation is defined ONLY here and passed to DRONE_OBSTACLES.
% The selected original generator may produce one or several parts per
% object. The sensor and trigger only inspect the ORIGINAL part.kind.
raObstacleMethod = 'single_ellipsoid';  % single_sphere / single_ellipsoid / multiple_spheres / multiple_ellipsoids
raSystemMode = 'C';                     % C / B / D (SYSTEM ONLY)
raSensorRadiusM = 20;                  % externally adjustable [m]
raVelocityWeight = 0.1;                % PV fit weight [seconds], not safety radius
raEnableTriggerMonitor = true;        % monitor ONLY; no avoidance input
if ~ismember(raObstacleMethod, ...
        {'single_sphere','single_ellipsoid','multiple_spheres','multiple_ellipsoids'})
    error('RA:RUN','Use one of the four original DRONE_OBSTACLES approximation methods');
end
SETUP_REFERENCE_OBSTACLE_AVOIDANCE();

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
% agent.reference.set_function_class("timevarying", TIME_VARYING_REFERENCE(agent, ...
%     {"gen_ref_saddle",{"freq",5,"center",[0;0;1.5],"radius",[1,1,0]},6}));
% 使用する軌道の定義ファイル
flightRouteGenerator = "gen_wp_straight";

% 公称進行速度 [m/s]
flightNominalSpeed = 1.5;
% 軌道パラメータ
flightRouteParams = {"speed", flightNominalSpeed};
% Waypoint参照生成器
agent.reference.set_function_class( ...
    "waypoint", ...
    WAYPOINT_REFERENCE(agent, ...
    {char(flightRouteGenerator), flightRouteParams, 6}));
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
% agent.cha_allocation.f.reference = ["timevarying","sload"];
agent.cha_allocation.f.reference = ["waypoint","sload"];
agent.cha_allocation.l.reference = ["landing","sload"];

env = [];
%% ============================================================
% 障害物・地面の定義
% ============================================================

% 使用する障害物配置
sceneName = 'scene_cylinders_high_side';

% sceneName = 'scene_gate_square';
% sceneName = 'scene_showcase_13';
% sceneName = 'scene_mixed_obstacles';

% ------------------------------------------------------------
% 近似方式（必ずどちらか一方を有効にする）
% ------------------------------------------------------------

% 球／楕円体は raObstacleMethod の1箇所で指定
approxMethod = raObstacleMethod; % the ONLY selected original approximation

% 複数球
% No second type toggle: sensor reads each actual part.kind from DRONE_OBSTACLES.

% ------------------------------------------------------------
% 地面
% ------------------------------------------------------------

chosenGround = 'ground_z0';

% ------------------------------------------------------------
% 近似の分割数・計算量設定
% ------------------------------------------------------------

obstacleOptions = struct();

obstacleOptions.maxPartsPerObstacle = 6; % generator default; single_* still generates 1
obstacleOptions.maxTotalParts = 36; % maximum GENERATED parts (not objects)
obstacleOptions.targetPartSize = 3.0;

obstacleOptions.superExponent = 8;
obstacleOptions.hollowSides = 16;
obstacleOptions.polySides = 8;

% 障害物生成
worldObstacles = DRONE_OBSTACLES( ...
    sceneName, ...
    approxMethod, ...
    chosenGround, ...
    obstacleOptions);
% Print the ACTUAL approximations returned by the existing generator.
% Neither sensor nor trigger refits/creates their own obstacle geometry.
raDefinitionReport = ra_obstacle_inspect_definition(worldObstacles,true);
raSensorCfg = ra_sim_sensor_config(raSensorRadiusM);
raSensorExpected = raDefinitionReport.worldMethod;

% Heavy PV matrices / route PCHIP and optional native MEX are prepared
% ONCE before arming, never inside the 25-ms Flight tick.
raPrep=struct();raNative=struct('available',false);
if raEnableTriggerMonitor
    hlcGains=agent.controller.hlc_suspended.param;
    wpCurrent=agent.reference.waypoint;
    raPrep=reference_avoidance_prepare_forecast( ...
        hlcGains,-1:dt:0,wpCurrent.filterTime,raVelocityWeight, ...
        wpCurrent.generator,0:dt:(te+4));
    if strcmp(raSystemMode,'C')
        raNative=reference_avoidance_mex_ensure('prepare');
    end
end
raTickState=struct();
raElapsedMs=nan(size(ts:dt:te));
raDetectedObjects=zeros(size(ts:dt:te));
raDetectedParts=zeros(size(ts:dt:te));
raStatus=repmat({''},size(ts:dt:te));
raTriggerTimeMs=nan(size(ts:dt:te));


% ------------------------------------------------------------
% 描画・保存の設定
% ------------------------------------------------------------

% 表示するかどうか
obstacleDrawSceneViews = true;
obstacleDrawFlightViews = true;
obstacleDrawFlightMovie = true;

% ファイルに保存するかどうか（初期状態はすべてOFF）
obstacleSaveImages = false;
obstacleSaveMovie  = false;

% 静止画で動的障害物を表示する時刻 [s]
obstacleFigureTime = 10.0;

% 動画設定
obstacleMovieSkip = 4;
obstacleMovieFPS  = 10;
obstacleMoviePause = 0;

% 保存先（保存ONのときだけ使用）
obstacleOutputDir = 'Data';

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
    % SENSOR at CURRENT simulator true drone position (NOT EKF p).
    % Called before plant.do updates the physical state for the next step.
    raSensorClock=tic;
    pDroneTrue = agent.plant.state.p;
    worldNow = obs_world_at_time(worldObstacles,time.t);
    if strcmp(cha,'f') && raEnableTriggerMonitor
        [RA,raTickState] = ra_sim_sensor_trigger_step( ...
            agent,worldNow,pDroneTrue,raSensorCfg,raSystemMode, ...
            raPrep,time.t,raTickState,raNative);
        raDetectedObjects(sample_idx)=RA.detectedObjects;
        raDetectedParts(sample_idx)=RA.detectedParts;
        raElapsedMs(sample_idx)=1000*toc(raSensorClock);
        raTriggerTimeMs(sample_idx)=RA.trigger.totalMs;
        raStatus{sample_idx}=RA.status;
    else
        % Takeoff/landing: sensor-only. Never run Flight predictor on a
        % takeoff/landing reference or reuse unknown reference history.
        [raVisible,raRep] = ra_sim_sensor_detect( ...
            worldNow,pDroneTrue,raSensorCfg); %#ok<NASGU>
        raDetectedObjects(sample_idx)=raRep.detectedObjectCount;
        raDetectedParts(sample_idx)=raRep.returnedPartCount;
        raElapsedMs(sample_idx)=1000*toc(raSensorClock);
        raStatus{sample_idx}='SENSOR_ONLY_NONFLIGHT';
    end
    do_prop_local(time, cha, cha, logger, env, agent, "controller",      1, true);
    do_prop_local(time, cha, cha, logger, env, agent, "input_transform", 1, true);
    do_prop_local(time, cha, cha, logger, env, agent, "plant",           1, true);
    agent.controller.result.controllertime_total = toc(start_total);
    logger.logging(time, cha, agent, []);
    time.k = logger.k;
end

%% Sensor + nominal trigger timing summary (not a CBF or safety guarantee)
active=(times>=5 & times<47);
validTimes=raElapsedMs(active & isfinite(raElapsedMs));
if ~isempty(validTimes)
    fprintf('[RA] %s | system=%s | sensor=%.2f m | parts=%d\n', ...
        raSensorExpected,raSystemMode,raSensorCfg.radiusM, ...
        raDefinitionReport.numParts);
    fprintf('[RA] sensor+nominal trigger: median=%.3f ms p95=%.3f ms max=%.3f ms\n', ...
        median(validTimes),prctile(validTimes,95),max(validTimes));
end
raRunSummary=struct('worldMethod',raSensorExpected, ...
    'systemMode',raSystemMode,'sensorRadiusM',raSensorCfg.radiusM, ...
    'time',times,'sensorAndTriggerMs',raElapsedMs, ...
    'triggerOnlyMs',raTriggerTimeMs, ...
    'detectedObjects',raDetectedObjects,'detectedParts',raDetectedParts, ...
    'status',{raStatus},'actualSystemInputModified',false, ...
    'noDynamicSafetyGuarantee',true);

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

%% ============================================================
% 障害物を含む3D・2Dグラフと動画
% ============================================================

% 保存がONなら出力フォルダを用意
if obstacleSaveImages || obstacleSaveMovie
    if ~exist(obstacleOutputDir, 'dir')
        mkdir(obstacleOutputDir);
    end
end

% ------------------------------------------------------------
% 1. 障害物のみ：3D / XY / XZ / YZ
% ------------------------------------------------------------

if obstacleDrawSceneViews

    imageName = '';

    if obstacleSaveImages
        imageName = fullfile( ...
            obstacleOutputDir, ...
            'ObstacleScene_4Views.png');
    end

    plot_obstacles_4views( ...
        worldObstacles, ...
        'time', obstacleFigureTime, ...
        'filename', imageName);
end

% ------------------------------------------------------------
% 2. 飛行軌跡＋障害物：3D / XY / XZ / YZ
% ------------------------------------------------------------

if logger.k > 1

    if obstacleDrawFlightViews

        imageName = '';

        if obstacleSaveImages
            imageName = fullfile( ...
                obstacleOutputDir, ...
                'Flight_4Views.png');
        end

        plot_flight_obstacles_4views( ...
            logger, worldObstacles, ...
            'time', obstacleFigureTime, ...
            'filename', imageName);
    end

    % --------------------------------------------------------
    % 3. 機体・吊り荷・障害物の動画
    % --------------------------------------------------------

    if obstacleDrawFlightMovie

        movieName = '';

        if obstacleSaveMovie
            movieName = fullfile( ...
                obstacleOutputDir, ...
                'FlightWithObstacles.mp4');
        end

        animate_suspended_with_obstacles( ...
            logger, agent(1), worldObstacles, ...
            'target', 1, ...
            'fps', obstacleMovieFPS, ...
            'skip', obstacleMovieSkip, ...
            'pause', obstacleMoviePause, ...
            'mp4', '', ...                 % 保存しない
            'cameraMode', 'wholeFlight', ...
            'trailSeconds', Inf, ...
            'originalAlpha', 0.68, ...
            'approxAlpha', 0.12);
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

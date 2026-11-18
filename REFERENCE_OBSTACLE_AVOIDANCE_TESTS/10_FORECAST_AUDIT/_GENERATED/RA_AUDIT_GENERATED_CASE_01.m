function trace=RA_AUDIT_GENERATED_CASE_01()
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
% AUDIT: do not clear caller state
% AUDIT: do not close user figures
% AUDIT: do not clear command window

% Unlike the earlier headless script, this does not rely on the MATLAB
% editor being open or on a particular substring in the script filename.
script_dir = 'C:\Users\student\Documents\GitHub\common_matlab';
if ~isempty(script_dir)
    cd(script_dir);
end
[~, tmp] = regexp(genpath('.'), '\.\\\.git.*?;', 'match', 'split');
cellfun(@(xx) addpath(xx), tmp, 'UniformOutput', false);
userpath('clear');
SETUP_REFERENCE_OBSTACLE_AVOIDANCE();

%% SimSuspendedLoad.m: time, logger and display function
% Original settings: ts=0, dt=0.025, te=50.
ts = 0;
dt = 0.025;
te = 16;
time = TIME(ts,dt,te);
logger = LOGGER(1, size(ts:dt:te, 2), 0, [],[]);
logger.display_func = @(agent, time) build_display_vector(agent, time);
logger.display_on = false;
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
raAuditHlcCore = Controller_HL_Suspended_Load(dt,agent);
agent.controller.set_function_class("hlc_suspended", ...
    HLC_SUSPENDED_LOAD(agent,raAuditHlcCore));

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

% 複数楕円体：直接フィット版
approxMethod = 'multiple_ellipsoids';

% 複数球
% approxMethod = 'multiple_spheres';

% ------------------------------------------------------------
% 地面
% ------------------------------------------------------------

chosenGround = 'ground_z0';

% ------------------------------------------------------------
% 近似の分割数・計算量設定
% ------------------------------------------------------------

obstacleOptions = struct();

obstacleOptions.maxPartsPerObstacle = 6;
obstacleOptions.maxTotalParts = 36;
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

%% 模擬センサ・現在の対象障害物

% 検知半径 [m]。外部設定可能
raSensorCfg = ra_sim_sensor_config(20.0);

% 現在検知中の障害物のみ格納
targetObstacleWorld = struct();
targetSensorReport = struct();

% 検知数・名前だけをログに残す
% 過去の障害物形状は保持しない
nObstacleSamples = numel(ts:dt:te);
targetObjectCountHistory = zeros(nObstacleSamples,1);
targetPartCountHistory = zeros(nObstacleSamples,1);
targetObjectNamesHistory = cell(nObstacleSamples,1);
% ------------------------------------------------------------
% 描画・保存の設定
% ------------------------------------------------------------

% 表示するかどうか
obstacleDrawSceneViews = false;
obstacleDrawFlightViews = false;
obstacleDrawFlightMovie = false;

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

raAuditLog = struct([]);
% Arming at t = 0 (one log entry)
time.t = ts;
cha = 'a';
agent.cha = cha;
motive.getData(agent);
start_total = tic;
do_prop_local(time, cha, cha, logger, env, agent, "sensor",          1, true);
do_prop_local(time, cha, cha, logger, env, agent, "estimator",       1, true);
% ===== 追加開始 =====
% 現在時刻の障害物情報
worldNow = obs_world_at_time(worldObstacles, time.t);

% 機体真値位置を中心に現在の障害物を検知
[targetObstacleWorld,targetSensorReport] = ...
    ra_sim_sensor_detect( ...
    worldNow, agent.plant.state.p, raSensorCfg);

% 時刻0の検知情報を記録
targetObjectCountHistory(1) = ...
    targetSensorReport.detectedObjectCount;

targetPartCountHistory(1) = ...
    targetSensorReport.returnedPartCount;

targetObjectNamesHistory{1} = ...
    targetSensorReport.detectedObjectNames;
% ===== 追加終了 =====
do_prop_local(time, cha, cha, logger, env, agent, "reference",       1, true);
do_prop_local(time, cha, cha, logger, env, agent, "controller",      1, true);
raAuditLog(end+1) = RA_AUDIT_CAPTURE_EKF_AND_ORACLE(agent,time);
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
    % ===== 追加開始 =====
    % 現在時刻の障害物情報
    worldNow = obs_world_at_time(worldObstacles, time.t);

    % 機体真値位置を中心に模擬センサで検知
    [targetObstacleWorld,targetSensorReport] = ...
        ra_sim_sensor_detect( ...
        worldNow, agent.plant.state.p, raSensorCfg);

    % 今周期の検知情報を記録
    targetObjectCountHistory(sample_idx) = ...
        targetSensorReport.detectedObjectCount;

    targetPartCountHistory(sample_idx) = ...
        targetSensorReport.returnedPartCount;

    targetObjectNamesHistory{sample_idx} = ...
        targetSensorReport.detectedObjectNames;
    % ===== 追加終了 =====
    do_prop_local(time, cha, cha, logger, env, agent, "reference",       1, true);
    do_prop_local(time, cha, cha, logger, env, agent, "controller",      1, true);
raAuditLog(end+1) = RA_AUDIT_CAPTURE_EKF_AND_ORACLE(agent,time);
    do_prop_local(time, cha, cha, logger, env, agent, "input_transform", 1, true);
    do_prop_local(time, cha, cha, logger, env, agent, "plant",           1, true);
    agent.controller.result.controllertime_total = toc(start_total);
    logger.logging(time, cha, agent, []);
    time.k = logger.k;
end

trace = struct();
trace.records=raAuditLog;
trace.generator=agent.reference.waypoint.generator;
trace.gains=struct('F1',raAuditHlcCore.F1,'F2',raAuditHlcCore.F2,'F3',raAuditHlcCore.F3);
trace.routeName=char(flightRouteGenerator);
trace.nominalSpeed=flightNominalSpeed;
trace.truthColumnsReservedForScoringOnly=true;
trace.originalRunWasNotModified=true;
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

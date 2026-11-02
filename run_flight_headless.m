% run_flight_headless.m
% ========================================================================
% 単機吊荷シミュレーション（GUIなし）＋REPLANNING_SYSTEMを周期呼出し。
% 終了後にLOGGERから軌道図・位置時系列・吊荷アニメーションを表示。
%
% REPLANNING_SYSTEM が障害物・模擬センサ・Sim LOGGER用結果を管理する。
% 将来衝突の数値予測を参考計測します。回避軌道への自動反映・安全認証はしません。
% 目標値は通常の TIME_VARYING_REFERENCE から HLC へ直接送ります。
% HLC / EKF / plant / SUSPENDED_LOAD_REF_ADJUST は元のままです。
% ========================================================================

clearvars;
clc;

%% 1. 実行設定（Simと同じ接続を使用）
ts = 0;
dt = 0.025;
te = 65;
takeoff_end_s = 5;
flight_end_s = 47;

% リプランナー用の外部設定。元の障害物定義には影響しません。
ENABLE_ALL_LOG = true;
LOG_OBSTACLES = true;
ENABLE_SENSOR = true;        % falseならセンサ計算自体を行わない
LOG_SENSOR = true;           % falseでもENABLE_SENSOR=trueなら検知は実行する
LOG_REPLANNER = true;        % REPLANNING_SYSTEM.update単独の処理時間を保存
SENSOR_RANGE_M = 20;         % 模擬センサ半,/径[m]。Flightごとに自由に変更可能

% Future collision trigger: external Flight configuration (no fake model values).
% The UAV and payload are separate spheres; cable is a FINITE line segment.
% Cable radius=0 means an ideal thin taut cable, not a vibration allowance.
% Only specify a positive cable radius for measured physical thickness;
% uncertainty due to sway/slack must be handled by its state and proof.
ENABLE_TRIGGER = true;
LOG_TRIGGER = true;
LOG_OBSTACLE_MEMORY = true;
TRIGGER_HORIZON_S = 4.0;       % [s] prediction horizon; NEVER shortened by time limit
% Explicit numerical integration accuracy (NOT physical error margins).
TRIGGER_NUMERIC_ABS_TOL = 1e-10;
TRIGGER_NUMERIC_REL_TOL = 1e-8;
% The current reference evaluator supports f-phase only. The real phase
% boundary is read from this run (do not extrapolate into landing).
TRIGGER_FLIGHT_PHASE_END_S = flight_end_s;
TRIGGER_UAV_RADIUS_M = 0.5;    % [m] spherical UAV radius, user-designated
TRIGGER_PAYLOAD_RADIUS_M = 0.5;% [m] spherical payload radius, user-designated
TRIGGER_CABLE_RADIUS_M = 0.0;  % [m] finite centerline; optional measured cross-section radius

SAVE_LOG_AFTER_FLIGHT = true;  % Sim の LOGGER.save で Data/Sim_data に保存

% 表示はFlight終了後のみ。制御・障害物定義・LOGGERには影響しない。
SHOW_POSTRUN_PLOTS = true;         % 3D軌道・位置時系列
SHOW_ANIMATION_WINDOW = true;      % UAV・索・吊荷のリプレイ画面を開く
AUTO_PLAY_ANIMATION = false;       % trueなら終了後に自動再生。falseでも画面のボタンで再生できる
ANIMATION_FPS = 20;               % リプレイの画面更新速度 [frames/s]

% MATLABパス：以前の研究版・退避ディレクトリを不用意に優先させない。
project_root = fileparts(mfilename('fullpath'));
if isempty(project_root) || ~isfolder(project_root)
    error('run_flight_headless:Root','実行ファイルの場所を特定できません。');
end
cd(project_root);
all_paths = strsplit(genpath(project_root),pathsep);
usable_paths = {};
for path_index = 1:numel(all_paths)
    candidate = all_paths{path_index};
    if isempty(candidate), continue; end
    relative = strrep(candidate,[project_root filesep],'');
    if strcmp(relative,candidate), relative = ''; end
    parts = strsplit(relative,filesep);
    hidden_or_historical = false;
    for part_index = 1:numel(parts)
        part = parts{part_index};
        if strcmpi(part,'.git') || startsWith(part,'_AF7') || ...
                startsWith(part,'AF7R') || strcmpi(part,'quarantine') || ...
                strcmpi(part,'backup')
            hidden_or_historical = true;
            break
        end
    end
    if ~hidden_or_historical
        usable_paths{end+1} = candidate; %#ok<SAGROW>
    end
end
addpath(usable_paths{:},'-end');
module_dir = fullfile(project_root,'reference','REPLANNING_SYSTEM');
if ~isfile(fullfile(module_dir,'REPLANNING_SYSTEM.m'))
    error('run_flight_headless:MissingSource', ...
        'reference/REPLANNING_SYSTEM/REPLANNING_SYSTEM.m がありません。');
end
% サブモジュールの構成は統括クラスに任せる。runは個別モジュールを呼ばない。
addpath(genpath(module_dir),'-begin');
expected_class = fullfile(module_dir,'REPLANNING_SYSTEM.m');
if ~strcmpi(which('REPLANNING_SYSTEM'),expected_class)
    error('run_flight_headless:ShadowedClass', ...
        '別の REPLANNING_SYSTEM.m が使用されています: %s',which('REPLANNING_SYSTEM'));
end

%% 2. 通常のSim LOGGER（独自の保存機構は追加しない）
time = TIME(ts,dt,te);
logger = LOGGER(1,numel(ts:dt:te)+2,0,[],[]);
logger.display_on = false; % 全周期のコンソール出力は不要
% システム全体の1周期時間は、Simの標準LOGGER.Dataに記録する。
% アーミングはFlight周期とは別なので、最初のレコードはNaNのまま。
logger.Data.system_cycle_time_ms = nan(size(logger.Data.t));

%% 3. 機体・吊荷・センサ・EKF（既存の動作設定を維持）
agent = DRONE;
agent.parameter = DRONE_PARAM_SUSPENDED_LOAD("DIATONE");
agent.parameter.set("loadmass",0.14);    % plant生成時のSim真値
agent.parameter.set("cableL",2.0);

initial_state.q  = [0;0;0];
initial_state.w  = [0;0;0];
initial_state.vL = [0;0;0];
initial_state.p  = [0;0;0];
initial_state.pT = [0;0;-1];
initial_state.wL = [0;0;0];
initial_state.pL = initial_state.p + initial_state.pT*agent.parameter.cableL;
initial_state.v  = [0;0;0];

agent.plant = MODEL_CLASS(agent,Model_Suspended_Load(dt,initial_state,1,agent));
agent.parameter.set("loadmass",0.01);    % 既存EKFの初期推定値

motive = Connector_Natnet_sim(dt,{{1,"p","q"},{1,"pL","pT"}});
motive.getData(agent);
agent.sensor.set_function_class("motive", ...
    MOTIVE(agent,motive,"output_func",@motive_output, ...
    "rigid_id",[1,2],"state_list",{["p","q"],"p"}));
agent.estimator.set_function_class("ekf", ...
    EKF(agent,Estimator_EKF_SuspendedLoad(agent,dt, ...
    MODEL_CLASS(agent,Model_Suspended_Load(dt,initial_state,1,agent,"Load_mL_HL")), ...
    ["p","q","pL","pT"])));
agent.estimator.set_function_class("loadstate",SUSPENDED_LOAD_STATE_MANAGER(agent));

%% 4. 元の目標軌道とHLC（リプランナーを経由させない）
nominal_ref = TIME_VARYING_REFERENCE(agent, ...
    {"gen_ref_line",{"p0",[0 0 3],"velocity",1.5, ...
    "direction",[0 1 0]},6});
agent.reference.set_function_class("timevarying",nominal_ref);
agent.reference.set_function_class("sload",SUSPENDED_LOAD_REF_ADJUST(agent));
agent.reference.set_function_class("takeoff",TAKEOFF_REFERENCE(agent,"zd",3.0,"te",5));
agent.reference.set_function_class("landing", ...
    LANDING_REFERENCE(agent,"dt",dt,"zd",-agent.parameter.cableL,"te",3));
agent.controller.set_function_class("hlc_suspended", ...
    HLC_SUSPENDED_LOAD(agent,Controller_HL_Suspended_Load(dt,agent)));

agent.set_cha_allocation_for_all("sensor","motive");
agent.set_cha_allocation_for_all("estimator",["ekf","loadstate"]);
agent.cha_allocation.a.reference = ["takeoff","sload"];
agent.cha_allocation.t.reference = ["takeoff","sload"];
agent.cha_allocation.f.reference = ["timevarying","sload"];
agent.cha_allocation.l.reference = ["landing","sload"];
env = [];

%% 5. リプランナー初期化（障害物・模擬センサは内部で管理）
obstacle_options = struct('ENABLE_ALL_LOG',ENABLE_ALL_LOG, ...
                          'LOG_OBSTACLES',LOG_OBSTACLES, ...
                          'ENABLE_SENSOR',ENABLE_SENSOR, ...
                          'LOG_SENSOR',LOG_SENSOR, ...
                          'LOG_REPLANNER',LOG_REPLANNER, ...
                          'LOG_OBSTACLE_MEMORY',LOG_OBSTACLE_MEMORY, ...
                          'ENABLE_TRIGGER',ENABLE_TRIGGER, ...
                          'LOG_TRIGGER',LOG_TRIGGER, ...
                          'TRIGGER_HORIZON_S',TRIGGER_HORIZON_S, ...
                          'TRIGGER_NUMERIC_ABS_TOL',TRIGGER_NUMERIC_ABS_TOL, ...
                          'TRIGGER_NUMERIC_REL_TOL',TRIGGER_NUMERIC_REL_TOL, ...
                          'TRIGGER_FLIGHT_PHASE_END_S',TRIGGER_FLIGHT_PHASE_END_S, ...
                          'TRIGGER_UAV_RADIUS_M',TRIGGER_UAV_RADIUS_M, ...
                          'TRIGGER_PAYLOAD_RADIUS_M',TRIGGER_PAYLOAD_RADIUS_M, ...
                          'TRIGGER_CABLE_RADIUS_M',TRIGGER_CABLE_RADIUS_M, ...
                          'SENSOR_RANGE_M',SENSOR_RANGE_M);
replanner_instance = REPLANNING_SYSTEM(obstacle_options);
obstacle_status = replanner_instance.get_status();
fprintf('[REPLANNING] obstacles=%d | sensor enabled=%d | range=%.3f m\n', ...
    obstacle_status.count,obstacle_status.ENABLE_SENSOR,obstacle_status.SENSOR_RANGE_M);

%% 6. Headless初期化（既存Simの各phaseを一度通す）
disp('Initializing simulation states...');
for cha_init = ['a','t','f','l']
    agent.cha = cha_init;
    motive.getData(agent);
    do_prop_local(time,cha_init,'0',logger,env,agent,'sensor',1);
    do_prop_local(time,cha_init,'0',logger,env,agent,'estimator',1);
    do_prop_local(time,cha_init,'0',logger,env,agent,'reference',1);
    do_prop_local(time,cha_init,'0',logger,env,agent,'controller',1);
    do_prop_local(time,cha_init,'0',logger,env,agent,'input_transform',1);
end

%% 6.5. REPLANNING_SYSTEMの完全な飛行前初期化
% Sim初期化後の実EKF/参照構造体を使用し、Armingより前に実行。
% 本番センサのscan/ID履歴・reference.result・LOGGERは変更しない。
preflight_info = replanner_instance.preflight_update(time,agent);
fprintf('[REPLANNING PREFLIGHT] full update path ready=%d | %.3f ms (pre-arm only)\n', ...
    preflight_info.ready,preflight_info.elapsed_ms);

%% 7. Arming
fprintf('Arming...\n');
time.t = 0;
cha = 'a';
agent.cha = cha;
motive.getData(agent);
do_prop_local(time,cha,cha,logger,env,agent,'sensor',1);
do_prop_local(time,cha,cha,logger,env,agent,'estimator',1);
do_prop_local(time,cha,cha,logger,env,agent,'reference',1);
do_prop_local(time,cha,cha,logger,env,agent,'controller',1);
do_prop_local(time,cha,cha,logger,env,agent,'input_transform',1);
do_prop_local(time,cha,cha,logger,env,agent,'plant',1);
% 模擬センサ実行・結果のSim LOGGER向け格納は、統括クラス内で行う。
replanner_instance.update(time,agent);
logger.logging(time,cha,agent,[]);
time.k = logger.k;

%% 8. Flight（通常Simの処理＋リプランナーを1回呼出し）
fprintf('Starting simulation...\n');
simulation_error = [];
try
    while time.t <= time.te
        if time.t < takeoff_end_s
            cha = 't';
        elseif time.t < flight_end_s
            cha = 'f';
        else
            cha = 'l';
        end
        agent.cha = cha;

        % [全システム周期] MOTIVEからLOGGER.loggingおよび周期内の処理まで。
        % dt=25msは比較基準であり、計測値の上限・打切りには使わない。
        start_iteration = tic;
        motive.getData(agent);
        do_prop_local(time,cha,cha,logger,env,agent,'sensor',1);
        do_prop_local(time,cha,cha,logger,env,agent,'estimator',1);
        do_prop_local(time,cha,cha,logger,env,agent,'reference',1);
        do_prop_local(time,cha,cha,logger,env,agent,'controller',1);
        do_prop_local(time,cha,cha,logger,env,agent,'input_transform',1);
        do_prop_local(time,cha,cha,logger,env,agent,'plant',1);
        replanner_instance.update(time,agent);
        % logger.loggingはSim本来のデータ収集のまま維持する。
        logger.logging(time,cha,agent,[]);
        time.k = logger.k;
        time.t = time.t + dt;

        if mod(round(time.t/dt),round(5/dt)) == 0
            fprintf('Time: %.1f / %.1f s (phase %s)\n',time.t,time.te,cha);
        end
        % LOGGER自身の書込みも含む実際の1周期時間を同じLOGGERに追記。
        % このフィールドはrunが所有し、リプランナーには渡さない。
        logger.Data.system_cycle_time_ms(logger.k) = 1000*toc(start_iteration);
    end
catch caught_error
    simulation_error = caught_error;
    fprintf(2,'[FLIGHT ERROR] %.6f s: %s | %s\n', ...
        time.t,caught_error.identifier,caught_error.message);
end

%% 9. LOGGERの保存（飛行後のみ。Simと同じ save を使用）
obstacle_status = replanner_instance.get_status();
fprintf('[OBSTACLES] logged frames=%d / LOGGER records=%d | load=%.3f ms\n', ...
    obstacle_status.frame_count,logger.k,obstacle_status.definition_load_ms);
s = obstacle_status.sensor_status;
fprintf('[SENSOR] scans=%d | timing records=%d | mean=%.4f ms | max=%.4f ms | geometry prepare=%.3f ms\n', ...
    s.scan_count,s.timing_count,s.time_mean_ms,s.time_max_ms,s.geometry_prepare_ms);
if SAVE_LOG_AFTER_FLIGHT && logger.k > 0
    logger.save('ObstacleSensorFlight');
    disp('[LOGGER] Saved by the normal Sim LOGGER into Data/Sim_data.');
end

%% 10. Flight後の可視化。既存LOGGERのデータを読むだけ（追加ログなし）
if (SHOW_POSTRUN_PLOTS || SHOW_ANIMATION_WINDOW) && logger.k > 0
    try
        FLIGHT_LOG_VISUALIZATION(logger, replanner_instance.obstacles, ...
            SHOW_POSTRUN_PLOTS, SHOW_ANIMATION_WINDOW, ...
            AUTO_PLAY_ANIMATION, ANIMATION_FPS, dt, replanner_instance);
    catch display_error
        % 可視化エラーを、正常に保存できたFlight・障害物ログの失敗と混同しない。
        warning('run_flight_headless:DisplayFailed', ...
            'Flight後の表示に失敗しました: %s | %s', ...
            display_error.identifier, display_error.message);
    end
end

if ~isempty(simulation_error)
    rethrow(simulation_error); % 部分ログを残したうえで元のエラーを返す。
end
fprintf('Simulation completed. Run CHECK_SENSOR_FLIGHT_LOG(logger,replanner_instance).\n');

%% 以下はheadless用の最小限の補助関数
function y = motive_output(obj,data)
    p = data.rigid(obj.rigid_id(1)).p;
    pT = data.rigid(obj.rigid_id(2)).p - p;
    pT = pT/norm(pT);
    y = [p;Quat2Eul(data.rigid(obj.rigid_id(1)).q); ...
        data.rigid(obj.rigid_id(2)).p;pT];
end

function do_prop_local(time,cha,fCha,logger,env,agent,prop,i)
    % 通常のphase allocation順に関数を呼ぶ。処理内容は変更しない。
    if isfield(agent(i).cha_allocation,cha) && ...
            isfield(agent(i).cha_allocation.(cha),prop)
        list = agent(i).cha_allocation.(cha).(prop);
    else
        list = [];
    end
    if isempty(list)
        agent(i).(prop).result = ...
            agent(i).(prop).do(time,fCha,logger,env,agent,i);
        return
    end
    agent(i).(prop).result = ...
        agent(i).(prop).(list(1)).do(time,fCha,logger,env,agent,i);
    for j = 2:numel(list)
        agent(i).(prop).result = merge_result( ...
            agent(i).(prop).result, ...
            agent(i).(prop).(list(j)).do(time,fCha,logger,env,agent,i));
    end
end



% run_flight_headless.m
% =========================================================================
% Headless runner based directly on the working suspended-load Sim setup.
%
% Simからの変更点は原則として以下だけ:
%   1) nominal TIME_VARYING_REFERENCE を REPLANNING_SYSTEM で包む
%   2) GUIなしで回すため、headless用の phase loop を置く
%   3) REPLANNING_SYSTEM 内蔵 observer hard-support 用に、
%      plant生成時に使った真のloadmassだけを明示して渡す
%   4) 終了後に機体・吊り荷・参照軌道の可視化を追加
%
% MOTIVE / EKF / HLC / state_list / plant設定はSim準拠。
% =========================================================================

% clearvars; close all; clc;
clear all
cf = pwd;

if contains(mfilename('fullpath'), "mainGUI")
    cd(fileparts(mfilename('fullpath')));
else
    tmp = matlab.desktop.editor.getActive;
    cd(fileparts(tmp.Filename));
end

[~, tmp] = regexp(genpath('.'), '\.\\\.git.*?;', 'match', 'split');
cellfun(@(xx) addpath(xx), tmp, 'UniformOutput', false);
close all hidden; clear; clc;
userpath('clear');
%% 1. SIMと同じ基本設定
ts = 0;          % initial time
dt = 0.025;      % sampling period
te = 45;         % terminal time
time = TIME(ts,dt,te);

% -------------------------------------------------------------------------
% Avoidance certificate / commit switches.
%
% Both certificate layers can be evaluated on the SAME generated candidate.
% This makes the comparison scientifically meaningful.
%
% REFERENCE_ONLY:
%   reference C7/flatness + continuous static-ellipsoid geometry.
%   Comparison/research authority only; never full-system safety authority.
%
% EXECUTION_AWARE:
%   additionally requires observer hard set -> HLC tracking tube ->
%   actuator reserve -> tracking-inflated geometry -> numerical authority.
%
% AVOIDANCE_COMMIT_POLICY decides which layer is allowed to mutate the
% active reference.  For the first trajectory-avoidance visualization run,
% REFERENCE_ONLY is useful because the execution-aware fixed-point theorem
% is still fail-closed.  Switch to EXECUTION_AWARE for the final system.
% -------------------------------------------------------------------------
ENABLE_REFERENCE_ONLY_CERTIFICATE = true;
ENABLE_EXECUTION_AWARE_CERTIFICATE = true;
AVOIDANCE_COMMIT_POLICY = "REFERENCE_ONLY";   % "REFERENCE_ONLY" or "EXECUTION_AWARE"
ENABLE_AUTO_REPLAN_COMMIT = true;

% Post-run visualization only.
% Nothing is written to disk by the visualization path.
% Figure 14 plays the logged 3-D UAV/payload/cable motion on screen.
ENABLE_POSTRUN_ANIMATION = true;
POSTRUN_ANIMATION_FPS = 20;

logger = LOGGER(1, size(ts:dt:te, 2), 0, [],[]);
logger.display_func = @(agent, time) build_display_vector(agent, time);
logger.display_on = true;

fprintf("表示物\nref:[px, py, pz]  est:[px, py, pz]  U:[T, tx, ty, tz]  mL\n\n");

%% 2. drone plant setting — Sim準拠
agent = DRONE;
agent.parameter = DRONE_PARAM_SUSPENDED_LOAD("DIATONE");

% Simでplant生成時に使っている真値。
plant_loadmass = 0.14;
agent.parameter.set("loadmass",plant_loadmass);
agent.parameter.set("cableL",2.0);

initial_state.q  = [0; 0; 0];
initial_state.w  = [0; 0; 0];
initial_state.vL = [0; 0; 0];
initial_state.p  = [0; 0; 0];
initial_state.pT = [0; 0; -1];
initial_state.wL = [0; 0; 0];
initial_state.pL = initial_state.p + initial_state.pT*agent.parameter.cableL;
initial_state.v  = [0; 0; 0];

% Note: set the model error after setting "plant"
agent.plant = MODEL_CLASS(agent,Model_Suspended_Load(dt,initial_state,1,agent));

% Simと同じ推定器側の初期質量値。
agent.parameter.set("loadmass",0.01);

% Sim only: getData works after setting "plant"
motive = Connector_Natnet_sim(dt, {{1,"p","q"},{1,"pL","pT"}});
motive.getData(agent);

%% 3. drone setting — Sim準拠
% Simと同じ state_list をそのまま使用する。
agent.sensor.set_function_class("motive", ...
    MOTIVE(agent,motive, ...
    "output_func",@motive_output, ...
    "rigid_id",[1,2], ...
    "state_list",{["p","q"],"p"}));

agent.estimator.set_function_class("ekf", ...
    EKF(agent,Estimator_EKF_SuspendedLoad(agent,dt, ...
    MODEL_CLASS(agent,Model_Suspended_Load(dt,initial_state,1,agent,"Load_mL_HL")), ...
    ["p","q","pL","pT"])));

agent.estimator.set_function_class("loadstate",SUSPENDED_LOAD_STATE_MANAGER(agent));

L = agent.parameter.cableL;

% %% 4. nominal reference — Simと同じ saddle 軌道
% nominal_ref = TIME_VARYING_REFERENCE(agent, ...
%     {"gen_ref_saddle", ...
%     {"freq",5,"center",[0;0;1.5],"radius",[1,1,0]}, ...
%     6});

% % 公称直線軌道
% nominal_ref = TIME_VARYING_REFERENCE(agent, ...
%     {"gen_ref_line", { ...
%     "p0", [0, 0, 3], ...
%     "velocity", 1.5, ...
%     "direction", [0, 0, 1] ...
%     }, 6});
nominal_ref = TIME_VARYING_REFERENCE(agent, ...
    {"gen_ref_line", { ...
    "p0", [0, 0, 3], ...
    "velocity", 1.5, ...
    "direction", [0, 1, 0] ...
    }, 6});


%% 5. replanningを nominal_ref の前段に追加
% ここがSimからの本質的な追加部分。
replan_opts = struct();
replan_opts.trigger_dist = 15.0;
replan_opts.r_load       = 0.5;
replan_opts.r_drone      = 0.5;
replan_opts.cable        = 0.5;
replan_opts.generator_backend_flag = 1;

% REPLANNING_SYSTEM内蔵observer hard-supportは通常ON。
% SIMではplant生成時の真のloadmassを明示的に渡す。
replan_opts.ENABLE_OBSERVER_HARD_SUPPORT = true;
replan_opts.OBSERVER_HARD_SUPPORT_MODE = "SIM_NUMERIC_TRUTH";
replan_opts.OBSERVER_SIM_TRUE_ML = plant_loadmass;

% Avoidance certificate/commit switches.
replan_opts.ENABLE_REFERENCE_ONLY_CERTIFICATE = ...
    ENABLE_REFERENCE_ONLY_CERTIFICATE;
replan_opts.ENABLE_EXECUTION_AWARE_CERTIFICATE = ...
    ENABLE_EXECUTION_AWARE_CERTIFICATE;
replan_opts.AVOIDANCE_COMMIT_POLICY = AVOIDANCE_COMMIT_POLICY;
replan_opts.ENABLE_AUTO_REPLAN_COMMIT = ENABLE_AUTO_REPLAN_COMMIT;

replanner_instance = REPLANNING_SYSTEM(agent,nominal_ref,replan_opts);

avoidance_status = replanner_instance.get_avoidance_mode_status();
fprintf(['[AVOIDANCE] commit=%s | ref-cert=%d | exec-cert=%d | ' ...
    'auto-commit=%d\n'], ...
    avoidance_status.commit_policy, ...
    avoidance_status.reference_only_evaluation_enabled, ...
    avoidance_status.execution_aware_evaluation_enabled, ...
    avoidance_status.auto_commit);

agent.reference.set_function_class("timevarying",replanner_instance);
agent.reference.set_function_class("sload",SUSPENDED_LOAD_REF_ADJUST(agent));
agent.reference.set_function_class("takeoff",TAKEOFF_REFERENCE(agent,"zd",3.0,"te",5));
agent.reference.set_function_class("landing",LANDING_REFERENCE(agent,"dt",dt,"zd",-L,"te",3));

agent.controller.set_function_class("hlc_suspended", ...
    HLC_SUSPENDED_LOAD(agent,Controller_HL_Suspended_Load(dt,agent)));

agent.set_cha_allocation_for_all("sensor","motive");
agent.set_cha_allocation_for_all("estimator",["ekf","loadstate"]);
agent.cha_allocation.a.reference = ["takeoff","sload"];
agent.cha_allocation.t.reference = ["takeoff","sload"];
agent.cha_allocation.f.reference = ["timevarying","sload"];
agent.cha_allocation.l.reference = ["landing","sload"];

env = [];

% -------------------------------------------------------------------------
% Runtime profile kept OUTSIDE LOGGER so logger's own cost can be measured.
% These arrays are diagnostic only and do not feed safety decisions.
% -------------------------------------------------------------------------
runtime_profile = struct();
runtime_profile.t = nan(size(ts:dt:te));
runtime_profile.phase = repmat(' ',size(ts:dt:te));
runtime_profile.sensor_acquisition_s = nan(size(ts:dt:te));
runtime_profile.online_before_plant_s = nan(size(ts:dt:te));
runtime_profile.simulation_plant_s = nan(size(ts:dt:te));
runtime_profile.logger_s = nan(size(ts:dt:te));
runtime_profile.full_iteration_s = nan(size(ts:dt:te));
runtime_profile.k = 0;

%% 6. Headless初期化
% GUIの代わりに各phaseの関数を一度通す。
% plantはここでは進めない。
disp("Initializing simulation states...");

for cha_init = ['a','t','f','l']
    agent.cha = cha_init;

    if ~isempty(motive)
        motive.getData(agent);
    end

    do_prop_local(time,cha_init,"0",logger,env,agent,"sensor",1,false);
    do_prop_local(time,cha_init,"0",logger,env,agent,"estimator",1,false);
    do_prop_local(time,cha_init,"0",logger,env,agent,"reference",1,false);
    do_prop_local(time,cha_init,"0",logger,env,agent,"controller",1,false);
    do_prop_local(time,cha_init,"0",logger,env,agent,"input_transform",1,false);
end

%% 7. Arming
disp("Arming...");

time.t = 0;
cha = 'a';
agent.cha = cha;

tIteration=tic;

tAcquire=tic;
if ~isempty(motive)
    motive.getData(agent);
end
sensor_acquisition_s=toc(tAcquire);

do_prop_local(time,cha,cha,logger,env,agent,"sensor",1,true);
do_prop_local(time,cha,cha,logger,env,agent,"estimator",1,true);
do_prop_local(time,cha,cha,logger,env,agent,"reference",1,true);
do_prop_local(time,cha,cha,logger,env,agent,"controller",1,true);
do_prop_local(time,cha,cha,logger,env,agent,"input_transform",1,true);

online_before_plant_s=toc(tIteration);

tPlantOnly=tic;
do_prop_local(time,cha,cha,logger,env,agent,"plant",1,true);
simulation_plant_s=toc(tPlantOnly);
control_plus_sim_plant_s=toc(tIteration);

agent.controller.result.sensor_acquisition_total=sensor_acquisition_s;
agent.controller.result.online_compute_total=online_before_plant_s;
agent.controller.result.simulation_plant_total=simulation_plant_s;
agent.controller.result.controllertime_total=control_plus_sim_plant_s;

tLogger=tic;
logger.logging(time,cha,agent,[]);
logger_s=toc(tLogger);
full_iteration_s=toc(tIteration);
time.k = logger.k;

runtime_profile.k=runtime_profile.k+1;
rk=runtime_profile.k;
runtime_profile.t(rk)=time.t;
runtime_profile.phase(rk)=cha;
runtime_profile.sensor_acquisition_s(rk)=sensor_acquisition_s;
runtime_profile.online_before_plant_s(rk)=online_before_plant_s;
runtime_profile.simulation_plant_s(rk)=simulation_plant_s;
runtime_profile.logger_s(rk)=logger_s;
runtime_profile.full_iteration_s(rk)=full_iteration_s;

%% 8. Headless simulation loop
% GUI版ではphase切替はGUI側が担当する。
% このrunだけはGUI無しで動かすため、自動切替を置く。
disp("Starting Simulation...");

cha = 't';

% AF7R36 R23 partial-plot-on-error: preserve the original failure.
simulation_error=[];
simulation_error_time=NaN;
simulation_error_phase='';
try
while time.t <= time.te
    if time.t < 5.0
        cha = 't';
    elseif time.t < 47.0
        cha = 'f';
    else
        cha = 'l';
    end

    agent.cha = cha;

    tIteration=tic;

    % Include sensor acquisition/connector update in the online deadline.
    tAcquire=tic;
    if ~isempty(motive)
        motive.getData(agent);
    end
    sensor_acquisition_s=toc(tAcquire);

    do_prop_local(time,cha,cha,logger,env,agent,"sensor",1,true);
    do_prop_local(time,cha,cha,logger,env,agent,"estimator",1,true);
    do_prop_local(time,cha,cha,logger,env,agent,"reference",1,true);
    do_prop_local(time,cha,cha,logger,env,agent,"controller",1,true);
    do_prop_local(time,cha,cha,logger,env,agent,"input_transform",1,true);

    % Onboard-compute proxy:
    % connector acquisition + sensor + estimator + reference/replanner
    % + controller + input_transform.  Numerical plant is excluded.
    online_before_plant_s=toc(tIteration);

    tPlantOnly=tic;
    do_prop_local(time,cha,cha,logger,env,agent,"plant",1,true);
    simulation_plant_s=toc(tPlantOnly);
    control_plus_sim_plant_s=toc(tIteration);

    agent.controller.result.sensor_acquisition_total=sensor_acquisition_s;
    agent.controller.result.online_compute_total=online_before_plant_s;
    agent.controller.result.simulation_plant_total=simulation_plant_s;

    % Compatibility plot: simulation control path through numerical plant.
    agent.controller.result.controllertime_total=control_plus_sim_plant_s;

    % LOGGER is simulation/diagnostic infrastructure. Measure separately
    % instead of silently including or excluding it from the 4 ms claim.
    tLogger=tic;
    logger.logging(time,cha,agent,[]);
    logger_s=toc(tLogger);
    full_iteration_s=toc(tIteration);

    time.k = logger.k;

    runtime_profile.k=runtime_profile.k+1;
    rk=runtime_profile.k;
    runtime_profile.t(rk)=time.t;
    runtime_profile.phase(rk)=cha;
    runtime_profile.sensor_acquisition_s(rk)=sensor_acquisition_s;
    runtime_profile.online_before_plant_s(rk)=online_before_plant_s;
    runtime_profile.simulation_plant_s(rk)=simulation_plant_s;
    runtime_profile.logger_s(rk)=logger_s;
    runtime_profile.full_iteration_s(rk)=full_iteration_s;

    time.t = time.t + dt;

    if mod(time.t,5.0) < dt
        fprintf('Time: %.1f / %.1f sec (Phase: %s)\n',time.t,time.te,cha);
    end
end
catch ME_SIM
    simulation_error=ME_SIM;
    simulation_error_time=double(time.t);
    simulation_error_phase=char(cha);
    fprintf(2,'\n[HEADLESS] Simulation stopped at t=%.9g s, phase=%s.\n', ...
        simulation_error_time,simulation_error_phase);
    fprintf(2,'[HEADLESS] Original error: %s | %s\n',ME_SIM.identifier,ME_SIM.message);
end

% Trim preallocated timing arrays to actual logged length.
Krt=runtime_profile.k;
runtime_profile.t=runtime_profile.t(1:Krt);
runtime_profile.phase=runtime_profile.phase(1:Krt);
runtime_profile.sensor_acquisition_s=runtime_profile.sensor_acquisition_s(1:Krt);
runtime_profile.online_before_plant_s=runtime_profile.online_before_plant_s(1:Krt);
runtime_profile.simulation_plant_s=runtime_profile.simulation_plant_s(1:Krt);
runtime_profile.logger_s=runtime_profile.logger_s(1:Krt);
runtime_profile.full_iteration_s=runtime_profile.full_iteration_s(1:Krt);

if isempty(simulation_error)
    disp("Simulation complete! Drawing plots...");
else
    disp("Simulation interrupted. Drawing all available partial plots before rethrow...");
end
plot_error=[];
try

%% 9. Post-run visualization — AF7R51 integrated dashboard
% Later visualization fix: replaces fragmented/blank post-run figures.
% Post-run only; controller/plant/safety semantics are unchanged.
assert(exist('replanner_instance','var')==1, ...
    '[AF7R51P] replanner_instance is unavailable after the run.');
draw_postrun_dashboard_local(logger,runtime_profile,dt,replanner_instance, ...
    ENABLE_POSTRUN_ANIMATION,POSTRUN_ANIMATION_FPS);
drawnow;
catch ME_PLOT
    plot_error=ME_PLOT;
    fprintf(2,'[HEADLESS] Partial plotting error: %s | %s\n',ME_PLOT.identifier,ME_PLOT.message);
    drawnow;
end

if ~isempty(simulation_error)
    fprintf(2,'[HEADLESS] Partial plots displayed through t=%.9g s. Rethrowing original simulation error.\n', ...
        simulation_error_time);
    rethrow(simulation_error);
end
if ~isempty(plot_error)
    rethrow(plot_error);
end

%% LOCAL FUNCTIONS
function y = motive_output(obj,data)
% Simと同じ。
% Motive生データから [p; Euler; pL; pT] を作る。
p = data.rigid(obj.rigid_id(1)).p;
pT = data.rigid(obj.rigid_id(2)).p - p;
pT = pT/norm(pT);

y = [ ...
    p; ...
    Quat2Eul(data.rigid(obj.rigid_id(1)).q); ...
    data.rigid(obj.rigid_id(2)).p; ...
    pT ...
    ];
end

function do_prop_local(time,cha,fCha,logger,env,agent,prop,i,isReady)
% GUI無しで、通常のphase allocation順に各propertyを呼ぶだけの薄いwrapper。
if ~isReady && strcmp(prop,"plant")
    return;
end

if isfield(agent(i).cha_allocation,cha) && ...
        isfield(agent(i).cha_allocation.(cha),prop)
    list = agent(i).cha_allocation.(cha).(prop);
else
    list = [];
end

if isempty(list)
    agent(i).(prop).result = ...
        agent(i).(prop).do(time,fCha,logger,env,agent,i);
else
    agent(i).(prop).result = ...
        agent(i).(prop).(list(1)).do(time,fCha,logger,env,agent,i);

    for j = 2:length(list)
        agent(i).(prop).result = merge_result( ...
            agent(i).(prop).result, ...
            agent(i).(prop).(list(j)).do(time,fCha,logger,env,agent,i));
    end
end
end

function v = build_display_vector(agent,time)
% Simと同じ表示内容。
idx = 1;

if ~isprop(agent(idx).reference.result.state,"xd")
    v = [];
    return
end

xd = agent(idx).reference.result.state.xd;

if isfield(agent(idx).estimator.result,"state")
    p = agent(idx).estimator.result.state.p;

    if isprop(agent(idx).estimator.result.state,"mL")
        mL = agent(idx).estimator.result.state.mL;
    else
        mL = NaN;
    end
else
    p = [NaN;NaN;NaN];
    mL = NaN;
end

u = agent(idx).controller.result.input;

v = sprintf( ...
    "%c %.3f : R [%7.3f,%7.3f,%7.3f] : P [%7.3f,%7.3f,%7.3f] : U [%7.3f,%7.3f,%7.3f,%7.3f] : mL %7.3f", ...
    agent(idx).cha,time.t,xd(1:3)',p',u',mL);
end


function plot_runtime_profile(P,figOnline,figFull)
if ~isstruct(P) || ~isfield(P,'k') || P.k<1
    return;
end

mask=(P.phase=='f');

figure(figOnline); clf;
hold on; grid on; box on;
plot(P.t(mask),1e3*P.sensor_acquisition_s(mask), ...
    'LineWidth',1.2,'DisplayName','sensor acquisition');
plot(P.t(mask),1e3*P.online_before_plant_s(mask), ...
    'LineWidth',1.4,'DisplayName','online before plant');
plot(P.t(mask),1e3*P.simulation_plant_s(mask), ...
    'LineWidth',1.2,'DisplayName','simulation plant');
xlabel('time [s]');
ylabel('runtime [ms]');
title('Runtime decomposition — Flight');
legend('Location','best');
hold off;

figure(figFull); clf;
hold on; grid on; box on;
plot(P.t(mask),1e3*P.online_before_plant_s(mask), ...
    'LineWidth',1.4,'DisplayName','online before plant');
plot(P.t(mask),1e3*P.full_iteration_s(mask), ...
    'LineWidth',1.4,'DisplayName','full simulation iteration incl. logger');
xlabel('time [s]');
ylabel('runtime [ms]');
title('Online vs full simulation iteration — Flight');
legend('Location','best');
hold off;
end

function plot_suspended_load_trajectory(logger,dt,varargin)
% 機体・吊り荷・HLCへ渡った参照の3D軌道を描く。
%
% 推定値:
%   UAV     = estimator.result.state.p
%   payload = estimator.result.state.pL
%
% 参照:
%   reference.result.state.xd(1:3)
%
% ケーブル線は見やすさのため約1秒ごとに描く。
opt=struct( ...
    "figure_number",5, ...
    "phase","tfl", ...
    "title_text","UAV / Suspended Load / Reference Trajectory");

for k=1:2:numel(varargin)
    opt.(char(varargin{k}))=varargin{k+1};
end

[t,p,pL,ref,pTrue,pLTrue]=extract_trajectory_log(logger,dt);
mask=phase_mask_from_time(t,opt.phase);

valid=mask & ...
    all(isfinite(p),2) & ...
    all(isfinite(pL),2);

if nnz(valid)<2
    warning('run_flight_headless:TrajectoryPlot', ...
        'Not enough valid trajectory samples for Figure %d.', ...
        opt.figure_number);
    return;
end

figure(opt.figure_number); clf;
hold on; grid on; box on;

% 機体・吊り荷の軌道
plot3(p(valid,1),p(valid,2),p(valid,3), ...
    'LineWidth',1.8, ...
    'DisplayName','UAV estimated p');

plot3(pL(valid,1),pL(valid,2),pL(valid,3), ...
    'LineWidth',1.8, ...
    'DisplayName','Payload estimated p_L');

% シミュレーションplant真値がloggerにあれば自動的に重ねる。
vt=mask & all(isfinite(pTrue),2);
if nnz(vt)>=2
    plot3(pTrue(vt,1),pTrue(vt,2),pTrue(vt,3), ...
        ':','LineWidth',1.3, ...
        'DisplayName','UAV plant truth');
end

vlt=mask & all(isfinite(pLTrue),2);
if nnz(vlt)>=2
    plot3(pLTrue(vlt,1),pLTrue(vlt,2),pLTrue(vlt,3), ...
        ':','LineWidth',1.3, ...
        'DisplayName','Payload plant truth');
end

% 実際にreference.resultへ入った参照を表示
vr=mask & all(isfinite(ref),2);
if nnz(vr)>=2
    plot3(ref(vr,1),ref(vr,2),ref(vr,3), ...
        '--','LineWidth',1.6, ...
        'DisplayName','Reference xd(1:3)');
end

% 約1秒ごとに機体と吊り荷をケーブル線で結ぶ。
idx=find(valid);
stride=max(1,round(1.0/dt));
cableIdx=idx(1:stride:end);

firstCable=true;
for ii=cableIdx(:).'
    if firstCable
        plot3([p(ii,1),pL(ii,1)], ...
              [p(ii,2),pL(ii,2)], ...
              [p(ii,3),pL(ii,3)], ...
              ':','LineWidth',0.8, ...
              'DisplayName','Cable snapshots');
        firstCable=false;
    else
        plot3([p(ii,1),pL(ii,1)], ...
              [p(ii,2),pL(ii,2)], ...
              [p(ii,3),pL(ii,3)], ...
              ':','LineWidth',0.8, ...
              'HandleVisibility','off');
    end
end

% 始点・終点
i0=idx(1);
i1=idx(end);

plot3(p(i0,1),p(i0,2),p(i0,3), ...
    'o','MarkerSize',7,'LineWidth',1.5, ...
    'DisplayName','UAV start');

plot3(p(i1,1),p(i1,2),p(i1,3), ...
    's','MarkerSize',7,'LineWidth',1.5, ...
    'DisplayName','UAV end');

plot3(pL(i0,1),pL(i0,2),pL(i0,3), ...
    'o','MarkerSize',7,'LineWidth',1.5, ...
    'HandleVisibility','off');

plot3(pL(i1,1),pL(i1,2),pL(i1,3), ...
    's','MarkerSize',7,'LineWidth',1.5, ...
    'HandleVisibility','off');

xlabel('x [m]');
ylabel('y [m]');
zlabel('z [m]');
title(opt.title_text,'Interpreter','none');
legend('Location','best');
axis equal;
view(3);
hold off;
end

function plot_position_components(logger,dt,varargin)
% Flight区間で、機体p・吊り荷pL・reference xd(1:3)を各軸比較。
opt=struct("figure_number",7,"phase","f");

for k=1:2:numel(varargin)
    opt.(char(varargin{k}))=varargin{k+1};
end

[t,p,pL,ref,~,~]=extract_trajectory_log(logger,dt);
mask=phase_mask_from_time(t,opt.phase);

if nnz(mask)<2
    warning('run_flight_headless:PositionPlot', ...
        'Not enough samples for component plot.');
    return;
end

figure(opt.figure_number); clf;

names={'x','y','z'};
for a=1:3
    subplot(3,1,a);
    hold on; grid on; box on;

    vp=mask & isfinite(p(:,a));
    vl=mask & isfinite(pL(:,a));
    vr=mask & isfinite(ref(:,a));

    if any(vp)
        plot(t(vp),p(vp,a),'LineWidth',1.3, ...
            'DisplayName','UAV estimated');
    end
    if any(vl)
        plot(t(vl),pL(vl,a),'LineWidth',1.3, ...
            'DisplayName','Payload estimated');
    end
    if any(vr)
        plot(t(vr),ref(vr,a),'--','LineWidth',1.3, ...
            'DisplayName','Reference');
    end

    ylabel(sprintf('%s [m]',names{a}));
    if a==1
        title('Position Comparison — Flight');
        legend('Location','best');
    end
    if a==3
        xlabel('time [s]');
    end
    hold off;
end
end

function [t,p,pL,ref,pTrue,pLTrue]=extract_trajectory_log(logger,dt)
% logger.Dataから必要な軌道だけ安全に抽出。
D=logger.Data;
A=D.agent;
if numel(A)>1
    A=A(1);
end

if isfield(D,'t') && ~isempty(D.t)
    t=double(D.t(:));
else
    n=max([ ...
        result_count(A,'estimator'), ...
        result_count(A,'reference')]);
    t=(0:n-1)'*dt;
end

n=numel(t);
p=nan(n,3);
pL=nan(n,3);
ref=nan(n,3);
pTrue=nan(n,3);
pLTrue=nan(n,3);

Cest=get_result_cells(A,'estimator');
Cref=get_result_cells(A,'reference');
Cplant=get_result_cells(A,'plant');

ne=min(n,numel(Cest));
for k=1:ne
    rr=Cest{k};
    st=get_member(rr,'state');

    q=get_member(st,'p');
    if isnumeric(q) && numel(q)>=3
        q=double(q(:));
        p(k,:)=q(1:3).';
    end

    q=get_member(st,'pL');
    if isnumeric(q) && numel(q)>=3
        q=double(q(:));
        pL(k,:)=q(1:3).';
    end
end

nr=min(n,numel(Cref));
for k=1:nr
    rr=Cref{k};
    st=get_member(rr,'state');
    q=get_member(st,'xd');

    if isnumeric(q) && numel(q)>=3
        q=double(q(:));
        ref(k,:)=q(1:3).';
    end
end

np=min(n,numel(Cplant));
for k=1:np
    rr=Cplant{k};
    st=get_member(rr,'state');

    q=get_member(st,'p');
    if isnumeric(q) && numel(q)>=3
        q=double(q(:));
        pTrue(k,:)=q(1:3).';
    end

    q=get_member(st,'pL');
    if isnumeric(q) && numel(q)>=3
        q=double(q(:));
        pLTrue(k,:)=q(1:3).';
    end
end
end

function mask=phase_mask_from_time(t,phase)
% このheadless runのphase切替にだけ使用する可視化用mask。
% 制御・安全判定には使用しない。
phase=char(string(phase));

switch phase
    case 'f'
        mask=(t>=5.0 & t<47.0);
    case 't'
        mask=(t>=0 & t<5.0);
    case 'l'
        mask=(t>=47.0);
    otherwise
        mask=true(size(t));
end

mask=logical(mask(:));
end

function n=result_count(A,name)
n=0;
if isstruct(A) && isfield(A,name)
    S=A.(name);
    if isstruct(S) && isfield(S,'result')
        n=numel(S.result);
    end
end
end

function C=get_result_cells(A,name)
C={};

if ~isstruct(A) || ~isfield(A,name)
    return;
end

S=A.(name);
if ~isstruct(S) || ~isfield(S,'result')
    return;
end

R=S.result;
if iscell(R)
    C=R(:);
else
    C=num2cell(R(:));
end
end

function x=get_member(S,name)
x=[];
try
    if isempty(S)
        return;
    end

    if isstruct(S)
        if isfield(S,name)
            x=S.(name);
        end
    elseif isobject(S)
        if isprop(S,name)
            x=S.(name);
        end
    end
catch
    x=[];
end
end

function OUT = draw_postrun_dashboard_local(logger,runtime_profile,dt,replanner,enableAnimation,animationFPS)
% AF7R33 post-run visualization.
% ONE switchable window only:
%   Trajectory | Timing | 3-D | Retiming/Cable | Animation
%
% Visualization is post-run only and never feeds controller/safety decisions.

[t,p,pL,ref,refAcc,pTrue,pLTrue,reqCount,status] = extract_log(logger); %#ok<ASGLU>
tf0=double(replanner.flight_reference_time_origin);
if ~isfinite(tf0), tf0=5.0; end
flightEnd=min(max(t),47.0);
flight=(t>=tf0 & t<flightEnd);

nom=nan(size(ref));
fh=replanner.base_ref.func;
for k=find(flight).'
    tau=max(0,double(t(k))-tf0);
    xd=double(fh(tau)); xd=xd(:);
    if numel(xd)>=3 && all(isfinite(xd(1:3)))
        nom(k,:)=xd(1:3).';
    end
end

validCorr=flight & all(isfinite(ref),2) & all(isfinite(nom),2);
corrNorm=nan(size(t));
corrNorm(validCorr)=sqrt(sum((ref(validCorr,:)-nom(validCorr,:)).^2,2));

commitTime=double(replanner.last_replan_commit_time);
rejoinTime=NaN;
if isstruct(replanner.active_repaired_reference) && ...
        isfield(replanner.active_repaired_reference,'rejoin_time')
    rejoinTime=double(replanner.active_repaired_reference.rejoin_time);
end

vn=flight & all(isfinite(nom),2);
va=flight & all(isfinite(ref),2);
vu=flight & all(isfinite(p),2);
vl=flight & all(isfinite(pL),2);
vut=flight & all(isfinite(pTrue),2);
vlt=flight & all(isfinite(pLTrue),2);

% One result window only.
for f=[1 2 3 4 6 7 8 9 10 11 12 13 14]
    try
        h=findobj(groot,'Type','figure','Number',f);
        if ~isempty(h),close(h);end
    catch
    end
end

figure(5);clf;
fig=gcf;
set(fig,'Name','AF7 Flight Results — switchable dashboard', ...
    'NumberTitle','off','Units','normalized','Position',[0.03 0.04 0.93 0.90]);

tg=uitabgroup(fig,'Position',[0 0 1 1]);
tabTraj=uitab(tg,'Title','Trajectory');
tabTime=uitab(tg,'Title','Timing');
tab3D=uitab(tg,'Title','3-D');
tabCable=uitab(tg,'Title','Retiming / Cable');
tabAnim=uitab(tg,'Title','Animation');

%% Trajectory
tl=tiledlayout(tabTraj,2,2,'TileSpacing','compact','Padding','compact');

ax=nexttile(tl,1);hold(ax,'on');grid(ax,'on');box(ax,'on');
draw_obstacles_on_axes(ax,replanner);
if nnz(vn)>=2,plot3(ax,nom(vn,1),nom(vn,2),nom(vn,3),'--','LineWidth',1.3,'DisplayName','Nominal reference');end
if nnz(va)>=2,plot3(ax,ref(va,1),ref(va,2),ref(va,3),'LineWidth',1.8,'DisplayName','Active/HLC reference');end
if nnz(vu)>=2,plot3(ax,p(vu,1),p(vu,2),p(vu,3),'LineWidth',1.2,'DisplayName','UAV estimated');end
if nnz(vl)>=2,plot3(ax,pL(vl,1),pL(vl,2),pL(vl,3),'LineWidth',1.2,'DisplayName','Payload estimated');end
xlabel(ax,'x [m]');ylabel(ax,'y [m]');zlabel(ax,'z [m]');
title(ax,'Flight 3-D overview');
set_dedicated_3d_limits(ax,[nom(vn,:);ref(va,:);p(vu,:);pL(vl,:)],3.0);
pbaspect(ax,[1 1 1]);view(ax,35,24);legend(ax,'Location','best');hold(ax,'off');

ax=nexttile(tl,2);hold(ax,'on');grid(ax,'on');box(ax,'on');
if nnz(vn)>=2,plot(ax,nom(vn,1),nom(vn,3),'--','LineWidth',1.3,'DisplayName','Nominal');end
if nnz(va)>=2,plot(ax,ref(va,1),ref(va,3),'LineWidth',1.8,'DisplayName','Active/HLC ref');end
if nnz(vu)>=2,plot(ax,p(vu,1),p(vu,3),'LineWidth',1.1,'DisplayName','UAV');end
if nnz(vl)>=2,plot(ax,pL(vl,1),pL(vl,3),'LineWidth',1.1,'DisplayName','Payload');end
xlabel(ax,'x [m]');ylabel(ax,'z [m]');title(ax,'XZ plane');axis(ax,'tight');legend(ax,'Location','best');hold(ax,'off');

ax=nexttile(tl,3);hold(ax,'on');grid(ax,'on');box(ax,'on');
if any(validCorr),plot(ax,t(validCorr),corrNorm(validCorr),'LineWidth',1.5,'DisplayName','||active - nominal||');end
if isfinite(commitTime),xline(ax,commitTime,'--','commit','HandleVisibility','off');end
if isfinite(rejoinTime),xline(ax,rejoinTime,'--','rejoin','HandleVisibility','off');end
xlabel(ax,'time [s]');ylabel(ax,'reference correction [m]');title(ax,'Avoidance correction magnitude');legend(ax,'Location','best');hold(ax,'off');

ax=nexttile(tl,4);hold(ax,'on');grid(ax,'on');box(ax,'on');
if any(vn),plot(ax,t(vn),nom(vn,1),'--','LineWidth',1.2,'DisplayName','Nominal x');end
if any(va),plot(ax,t(va),ref(va,1),'LineWidth',1.5,'DisplayName','Active x');end
if any(vu),plot(ax,t(vu),p(vu,1),'LineWidth',1.0,'DisplayName','UAV x');end
if any(vl),plot(ax,t(vl),pL(vl,1),'LineWidth',1.0,'DisplayName','Payload x');end
xlabel(ax,'time [s]');ylabel(ax,'x [m]');title(ax,'Primary travel axis');legend(ax,'Location','best');hold(ax,'off');
title(tl,'Trajectory / avoidance');

%% Timing
P=runtime_profile;
K=min(double(P.k),numel(P.t));
tp=double(P.t(1:K));phase=P.phase(1:K);
online=1e3*double(P.online_before_plant_s(1:K));
plant=1e3*double(P.simulation_plant_s(1:K));
logms=1e3*double(P.logger_s(1:K));
full=1e3*double(P.full_iteration_s(1:K));
mask=(phase=='f');

tl2=tiledlayout(tabTime,2,2,'TileSpacing','compact','Padding','compact');
ax=nexttile(tl2,1);hold(ax,'on');grid(ax,'on');box(ax,'on');
plot(ax,tp(mask),online(mask),'LineWidth',1.3,'DisplayName','Online before plant');
plot(ax,tp(mask),full(mask),'LineWidth',1.2,'DisplayName','Full iteration incl. logger');
yline(ax,25,'--','25 ms control period','HandleVisibility','off');
xlabel(ax,'time [s]');ylabel(ax,'runtime [ms]');title(ax,'Flight runtime — complete');legend(ax,'Location','best');hold(ax,'off');

ax=nexttile(tl2,2);hold(ax,'on');grid(ax,'on');box(ax,'on');
zoomMask=mask & tp>=9.70 & tp<=10.40;
plot(ax,tp(zoomMask),online(zoomMask),'LineWidth',1.4,'DisplayName','Online');
plot(ax,tp(zoomMask),full(zoomMask),'LineWidth',1.2,'DisplayName','Full');
plot(ax,tp(zoomMask),plant(zoomMask),'LineWidth',1.1,'DisplayName','Plant');
yline(ax,25,'--','25 ms','HandleVisibility','off');
xlabel(ax,'time [s]');ylabel(ax,'runtime [ms]');title(ax,'Replan window');legend(ax,'Location','best');hold(ax,'off');

ax=nexttile(tl2,3);hold(ax,'on');grid(ax,'on');box(ax,'on');
plot(ax,tp(mask),plant(mask),'LineWidth',1.1,'DisplayName','Numerical plant');
plot(ax,tp(mask),logms(mask),'LineWidth',1.1,'DisplayName','Logger');
xlabel(ax,'time [s]');ylabel(ax,'runtime [ms]');title(ax,'Simulation-only costs');legend(ax,'Location','best');hold(ax,'off');

ax=nexttile(tl2,4);hold(ax,'on');grid(ax,'on');box(ax,'on');
res=full-online-plant-logms;
plot(ax,tp(mask),res(mask),'LineWidth',1.1,'DisplayName','Full - online - plant - logger');
xlabel(ax,'time [s]');ylabel(ax,'runtime [ms]');title(ax,'Outer-loop residual');legend(ax,'Location','best');hold(ax,'off');
title(tl2,'Timing');

%% Dedicated 3-D
ax3=axes('Parent',tab3D,'Position',[0.07 0.09 0.78 0.86]);
hold(ax3,'on');grid(ax3,'on');box(ax3,'on');
draw_obstacles_on_axes(ax3,replanner);
if nnz(vn)>=2,plot3(ax3,nom(vn,1),nom(vn,2),nom(vn,3),'--','LineWidth',1.5,'DisplayName','Nominal reference');end
if nnz(va)>=2,plot3(ax3,ref(va,1),ref(va,2),ref(va,3),'LineWidth',2.2,'DisplayName','Active/HLC reference');end
if nnz(vu)>=2,plot3(ax3,p(vu,1),p(vu,2),p(vu,3),'LineWidth',1.4,'DisplayName','UAV estimated');end
if nnz(vl)>=2,plot3(ax3,pL(vl,1),pL(vl,2),pL(vl,3),'LineWidth',1.4,'DisplayName','Payload estimated');end
if nnz(vut)>=2,plot3(ax3,pTrue(vut,1),pTrue(vut,2),pTrue(vut,3),':','LineWidth',1.0,'DisplayName','UAV plant truth');end
if nnz(vlt)>=2,plot3(ax3,pLTrue(vlt,1),pLTrue(vlt,2),pLTrue(vlt,3),':','LineWidth',1.0,'DisplayName','Payload plant truth');end
idx3=find(vu&vl);stride3=max(1,round(1.0/dt));firstCable3=true;
for ii=idx3(1:stride3:end).'
    if firstCable3
        plot3(ax3,[p(ii,1),pL(ii,1)],[p(ii,2),pL(ii,2)],[p(ii,3),pL(ii,3)],':','LineWidth',0.8,'DisplayName','Cable snapshots');
        firstCable3=false;
    else
        plot3(ax3,[p(ii,1),pL(ii,1)],[p(ii,2),pL(ii,2)],[p(ii,3),pL(ii,3)],':','LineWidth',0.8,'HandleVisibility','off');
    end
end
xlabel(ax3,'x [m]');ylabel(ax3,'y [m]');zlabel(ax3,'z [m]');
title(ax3,'Dedicated 3-D: nominal / repaired / vehicle / payload / cable');
set_dedicated_3d_limits(ax3,[nom(vn,:);ref(va,:);p(vu,:);pL(vl,:);pTrue(vut,:);pLTrue(vlt,:)],3.0);
pbaspect(ax3,[1 1 1]);view(ax3,35,24);legend(ax3,'Location','eastoutside');hold(ax3,'off');

%% Retiming / cable
[qEst,qPlant,qRef,qdotEst,qdotPlant,qdotRef]=build_cable_diagnostics(t,p,pL,pTrue,pLTrue,ref,refAcc,dt);
PS=replanner.get_staged_pipeline_status();
tl3=tiledlayout(tabCable,2,2,'TileSpacing','compact','Padding','compact');

ax=nexttile(tl3,1);hold(ax,'on');grid(ax,'on');box(ax,'on');
if isfield(PS,'r32_best_entry_T') && isfinite(PS.r32_best_entry_T)
    e=double(PS.r32_best_entry_T);f=double(PS.r32_best_flow_T);x=double(PS.r32_best_exit_T);
    bar(ax,1,[e f x],'stacked');
    xticks(ax,1);xticklabels(ax,{'selected timing'});
    ylabel(ax,'duration [s]');
    title(ax,sprintf('Adaptive timing: alpha = %.6g',double(PS.r32_best_progress_scale)));
    legend(ax,{'entry','retimed flow','exit'},'Location','best');
else
    text(ax,0.5,0.5,'No retiming candidate available','HorizontalAlignment','center');
    xlim(ax,[0 1]);ylim(ax,[0 1]);title(ax,'Adaptive timing');
end
hold(ax,'off');

ax=nexttile(tl3,2);hold(ax,'on');grid(ax,'on');box(ax,'on');
tilEst=cable_tilt_deg(qEst);tilPlant=cable_tilt_deg(qPlant);tilRef=cable_tilt_deg(qRef);
ve=isfinite(tilEst)&flight;vp=isfinite(tilPlant)&flight;vrf=isfinite(tilRef)&flight;
if any(vrf),plot(ax,t(vrf),tilRef(vrf),'--','LineWidth',1.2,'DisplayName','desired');end
if any(ve),plot(ax,t(ve),tilEst(ve),'LineWidth',1.2,'DisplayName','estimated');end
if any(vp),plot(ax,t(vp),tilPlant(vp),'LineWidth',1.2,'DisplayName','plant');end
xlabel(ax,'time [s]');ylabel(ax,'cable tilt [deg]');title(ax,'Cable tilt from downward');legend(ax,'Location','best');hold(ax,'off');

ax=nexttile(tl3,3);hold(ax,'on');grid(ax,'on');box(ax,'on');
qe=sqrt(sum(qdotEst.^2,2));qp=sqrt(sum(qdotPlant.^2,2));qr=sqrt(sum(qdotRef.^2,2));
m=isfinite(qr)&flight;if any(m),plot(ax,t(m),qr(m),'--','LineWidth',1.2,'DisplayName','desired ||qdot||');end
m=isfinite(qe)&flight;if any(m),plot(ax,t(m),qe(m),'LineWidth',1.2,'DisplayName','estimated ||qdot||');end
m=isfinite(qp)&flight;if any(m),plot(ax,t(m),qp(m),'LineWidth',1.2,'DisplayName','plant ||qdot||');end
xlabel(ax,'time [s]');ylabel(ax,'||qdot|| [1/s]');title(ax,'Cable angular-rate magnitude');legend(ax,'Location','best');hold(ax,'off');

ax=nexttile(tl3,4);hold(ax,'on');grid(ax,'on');box(ax,'on');
errEst=cable_angle_error_deg(qEst,qRef);errPlant=cable_angle_error_deg(qPlant,qRef);
m=isfinite(errEst)&flight;if any(m),plot(ax,t(m),errEst(m),'LineWidth',1.2,'DisplayName','estimated vs desired');end
m=isfinite(errPlant)&flight;if any(m),plot(ax,t(m),errPlant(m),'LineWidth',1.2,'DisplayName','plant vs desired');end
xlabel(ax,'time [s]');ylabel(ax,'angle error [deg]');title(ax,'Cable direction error');legend(ax,'Location','best');hold(ax,'off');
title(tl3,'Adaptive timing and suspended-load cable behavior');

%% Animation
axA=axes('Parent',tabAnim,'Position',[0.07 0.10 0.77 0.83]);
hold(axA,'on');grid(axA,'on');box(axA,'on');
draw_obstacles_on_axes(axA,replanner);
if nnz(vn)>=2,plot3(axA,nom(vn,1),nom(vn,2),nom(vn,3),'--','LineWidth',1.0,'DisplayName','Nominal reference');end
if nnz(va)>=2,plot3(axA,ref(va,1),ref(va,2),ref(va,3),'LineWidth',1.3,'DisplayName','Active/HLC reference');end
set_dedicated_3d_limits(axA,[p(vu,:);pL(vl,:);ref(va,:);nom(vn,:)],3.0);
pbaspect(axA,[1 1 1]);view(axA,35,24);
xlabel(axA,'x [m]');ylabel(axA,'y [m]');zlabel(axA,'z [m]');
legend(axA,'Location','eastoutside');hold(axA,'off');

uicontrol('Parent',tabAnim,'Style','pushbutton','String','Replay animation', ...
    'Units','normalized','Position',[0.86 0.90 0.11 0.055], ...
    'Callback',@(~,~)play_animation_on_axes(axA,animationFPS,t,p,pL,flight,dt));

animationOut=struct('enabled',logical(enableAnimation),'played',false, ...
    'fps',double(animationFPS),'saved_to_disk',false);
drawnow;
if enableAnimation
    tg.SelectedTab=tabAnim;
    animationOut=play_animation_on_axes(axA,animationFPS,t,p,pL,flight,dt);
    tg.SelectedTab=tabTraj;
end
assignin('base','AF7_POSTRUN_DISPLAY',animationOut);

peakCorr=NaN;pathRatio=NaN;
if any(validCorr)
    peakCorr=max(corrNorm(validCorr));
    LA=path_length(ref(validCorr,:));LN=path_length(nom(validCorr,:));
    if LN>0,pathRatio=LA/LN;end
end

OUT=struct('schema','AF7R33_SINGLE_WINDOW_DISPLAY_V1', ...
    'peak_correction_m',peakCorr,'path_length_ratio',pathRatio, ...
    'commit_time_s',commitTime,'rejoin_time_s',rejoinTime, ...
    'display_animation',animationOut,'single_window',true, ...
    'saved_to_disk',false);
assignin('base','AF7R51_DASHBOARD_OUT',OUT);
end


function OUT=play_animation_on_axes(ax,fps,t,p,pL,flight,dt)
OUT=struct('enabled',true,'played',false,'fps',double(fps), ...
    'frame_count',0,'saved_to_disk',false);
idx=find(flight & all(isfinite(p),2) & all(isfinite(pL),2));
if numel(idx)<2,return;end
fps=max(1,double(fps));
stride=max(1,round((1/fps)/dt));
idx=idx(1:stride:end);

delete(findobj(ax,'Tag','AF7_ANIM_DYNAMIC'));
hold(ax,'on');
hU=plot3(ax,NaN,NaN,NaN,'o','MarkerSize',8,'LineWidth',1.6,'DisplayName','UAV','Tag','AF7_ANIM_DYNAMIC');
hL=plot3(ax,NaN,NaN,NaN,'o','MarkerSize',8,'LineWidth',1.6,'DisplayName','Payload','Tag','AF7_ANIM_DYNAMIC');
hC=plot3(ax,NaN,NaN,NaN,'-','LineWidth',1.3,'DisplayName','Cable','Tag','AF7_ANIM_DYNAMIC');
hUT=plot3(ax,NaN,NaN,NaN,'-','LineWidth',1.2,'DisplayName','UAV trail','Tag','AF7_ANIM_DYNAMIC');
hLT=plot3(ax,NaN,NaN,NaN,'-','LineWidth',1.2,'DisplayName','Payload trail','Tag','AF7_ANIM_DYNAMIC');
hold(ax,'off');

trail=nan(numel(idx),3);trailL=nan(numel(idx),3);
for k=1:numel(idx)
    if ~isgraphics(ax),return;end
    frameT=tic;i=idx(k);
    trail(k,:)=p(i,:);trailL(k,:)=pL(i,:);
    set(hU,'XData',p(i,1),'YData',p(i,2),'ZData',p(i,3));
    set(hL,'XData',pL(i,1),'YData',pL(i,2),'ZData',pL(i,3));
    set(hC,'XData',[p(i,1),pL(i,1)],'YData',[p(i,2),pL(i,2)],'ZData',[p(i,3),pL(i,3)]);
    set(hUT,'XData',trail(1:k,1),'YData',trail(1:k,2),'ZData',trail(1:k,3));
    set(hLT,'XData',trailL(1:k,1),'YData',trailL(1:k,2),'ZData',trailL(1:k,3));
    title(ax,sprintf('3-D suspended-load playback   t = %.2f s',t(i)));
    drawnow limitrate;
    rem=1/fps-toc(frameT);if rem>0,pause(rem);end
end
OUT.played=true;OUT.frame_count=numel(idx);
end


function [t,p,pL,ref,refAcc,pTrue,pLTrue,reqCount,status]=extract_log(logger)
D=logger.Data; A=D.agent; if numel(A)>1,A=A(1);end
t=double(D.t(:)); n=numel(t);
p=nan(n,3);pL=nan(n,3);ref=nan(n,3);refAcc=nan(n,3);pTrue=nan(n,3);pLTrue=nan(n,3);
reqCount=nan(n,1);status=nan(n,1);
Ce=get_result_cells(A,'estimator');Cr=get_result_cells(A,'reference');Cp=get_result_cells(A,'plant');
for k=1:min(n,numel(Ce))
    st=get_member(Ce{k},'state');p(k,:)=vec3(get_member(st,'p'));pL(k,:)=vec3(get_member(st,'pL'));
end
for k=1:min(n,numel(Cr))
    st=get_member(Cr{k},'state');
    xd0=get_member(st,'xd');
    ref(k,:)=vec3(xd0);
    if isnumeric(xd0) && numel(xd0)>=11
        xx=double(xd0(:));
        if all(isfinite(xx(9:11))), refAcc(k,:)=xx(9:11).'; end
    end
    reqCount(k)=scalar(get_member(st,'generator_request_count'));
    status(k)=scalar(get_member(st,'generator_last_status_code'));
end
for k=1:min(n,numel(Cp))
    st=get_member(Cp{k},'state');pTrue(k,:)=vec3(get_member(st,'p'));pLTrue(k,:)=vec3(get_member(st,'pL'));
end
end

function set_readable_3d_limits(X)
X=X(all(isfinite(X),2),:); if isempty(X),return;end
lo=min(X,[],1);hi=max(X,[],1);ctr=(lo+hi)/2;sp=hi-lo;major=max([sp(1),sp(3),1]);
if sp(2)<0.15*major
    lo(2)=ctr(2)-0.075*major;hi(2)=ctr(2)+0.075*major;
end
pad=0.05*max(hi-lo,1e-9);lo=lo-pad;hi=hi+pad;
xlim([lo(1) hi(1)]);ylim([lo(2) hi(2)]);zlim([lo(3) hi(3)]);
daspect([1 1 1]);
end

function draw_obstacles(replanner)
try
    P=replanner.obstacle_packet;
    if ~isstruct(P)||~isfield(P,'count')||double(P.count)<1,return;end
    [sx,sy,sz]=sphere(10);U=[sx(:)';sy(:)';sz(:)'];
    for i=1:double(P.count)
        c=double(P.C(:,i));R=reshape(double(P.R(:,i)),3,3);a=double(P.Araw(:,i));
        X=R*diag(a)*U+c;
        mesh(reshape(X(1,:),size(sx)),reshape(X(2,:),size(sy)),reshape(X(3,:),size(sz)), ...
            'FaceAlpha',0.04,'EdgeAlpha',0.18,'HandleVisibility','off');
    end
catch
end
end

function plot_obstacle_xz(replanner)
try
    P=replanner.obstacle_packet;
    for i=1:double(P.count)
        c=double(P.C(:,i));
        plot(c(1),c(3),'x','MarkerSize',8,'LineWidth',1.2,'HandleVisibility','off');
        text(c(1),c(3),sprintf(' O%.0f',double(P.ids(i))));
    end
catch
end
end

function set_dedicated_3d_limits(ax,X,minSpan)
X=X(all(isfinite(X),2),:); if isempty(X),return;end
lo=min(X,[],1);hi=max(X,[],1);
for j=1:3
    if hi(j)-lo(j)<minSpan
        c=0.5*(lo(j)+hi(j));lo(j)=c-0.5*minSpan;hi(j)=c+0.5*minSpan;
    end
end
pad=max(0.05*(hi-lo),0.15);lo=lo-pad;hi=hi+pad;
xlim(ax,[lo(1),hi(1)]);ylim(ax,[lo(2),hi(2)]);zlim(ax,[lo(3),hi(3)]);
end

function draw_obstacles_on_axes(ax,replanner)
try
    P=replanner.obstacle_packet;if ~isstruct(P)||~isfield(P,'count')||double(P.count)<1,return;end
    [sx,sy,sz]=sphere(18);U=[sx(:)';sy(:)';sz(:)'];
    for i=1:double(P.count)
        c=double(P.C(:,i));RR=reshape(double(P.R(:,i)),3,3);a=double(P.Araw(:,i));X=RR*diag(a)*U+c;
        mesh(ax,reshape(X(1,:),size(sx)),reshape(X(2,:),size(sy)),reshape(X(3,:),size(sz)), ...
            'FaceAlpha',0.06,'EdgeAlpha',0.20,'HandleVisibility','off');
    end
catch
end
end

function [qEst,qPlant,qRef,qdotEst,qdotPlant,qdotRef]=build_cable_diagnostics(t,p,pL,pTrue,pLTrue,ref,refAcc,dt)
n=numel(t);qEst=nan(n,3);qPlant=nan(n,3);qRef=nan(n,3);
ve=all(isfinite(p),2)&all(isfinite(pL),2);vp=all(isfinite(pTrue),2)&all(isfinite(pLTrue),2);
qEst(ve,:)=normalize_rows(pL(ve,:)-p(ve,:));qPlant(vp,:)=normalize_rows(pLTrue(vp,:)-pTrue(vp,:));
% Desired cable direction from the actually delivered reference acceleration
% xd(9:11), i.e. q_d = -(a_d + g e3)/||a_d + g e3||.  Finite-difference
% fallback is visualization-only and is used only if a logged acceleration is
% unavailable at a sample.
aRef=double(refAcc);
missing=~all(isfinite(aRef),2);
if any(missing)
    aFD=nan(n,3);
    for j=1:3
        x=ref(:,j);v=gradient_nan(x,dt);aFD(:,j)=gradient_nan(v,dt);
    end
    aRef(missing,:)=aFD(missing,:);
end
sf=aRef+[zeros(n,2),9.81*ones(n,1)];vs=all(isfinite(sf),2)&vecnorm(sf,2,2)>0;
qRef(vs,:)=-normalize_rows(sf(vs,:));
qdotEst=gradient_rows(qEst,dt);qdotPlant=gradient_rows(qPlant,dt);qdotRef=gradient_rows(qRef,dt);
end

function Y=normalize_rows(X)
N=sqrt(sum(X.^2,2));Y=X./N;Y(~isfinite(Y))=NaN;
end

function y=gradient_nan(x,dt)
y=nan(size(x));v=isfinite(x);idx=find(v);if numel(idx)>=2,y(idx)=gradient(x(idx),dt);end
end

function Y=gradient_rows(X,dt)
Y=nan(size(X));for j=1:size(X,2),Y(:,j)=gradient_nan(X(:,j),dt);end
end

function a=cable_tilt_deg(q)
a=nan(size(q,1),1);v=all(isfinite(q),2);c=-q(v,3);c=max(-1,min(1,c));a(v)=acosd(c);
end

function a=cable_angle_error_deg(q,qd)
a=nan(size(q,1),1);v=all(isfinite(q),2)&all(isfinite(qd),2);c=sum(q(v,:).*qd(v,:),2);c=max(-1,min(1,c));a(v)=acosd(c);
end

function plot_time_allocation_panel(P)
axis([0 1 0 1]);axis off;
vals=[P.r32_best_entry_T,P.r32_best_flow_T,P.r32_best_exit_T];
if all(isfinite(vals)) && all(vals>0)
    total=sum(vals);x0=0.06;W=0.88;labels={'entry','retimed flow','exit'};
    for k=1:3
        w=W*vals(k)/total;rectangle('Position',[x0,0.38,w,0.22]);
        text(x0+w/2,0.49,sprintf('%s\\n%.3g s',labels{k},vals(k)), ...
            'HorizontalAlignment','center','VerticalAlignment','middle');x0=x0+w;
    end
    text(0.06,0.78,sprintf('progress scale \\alpha = %.4g',P.r32_best_progress_scale),'FontWeight','bold');
    text(0.06,0.68,sprintf('nominal progress speed = %.4g m/s',P.r32_nominal_progress_speed));
    text(0.06,0.60,sprintf('best pre-speed UB = %.4g m/s',P.r32_best_pre_speed_ub));
else
    text(0.08,0.62,'No certified/constructed R32 timing allocation available','FontWeight','bold');
    text(0.08,0.48,sprintf('affine base detected = %.0f',P.r32_affine_base_detected));
    text(0.08,0.38,sprintf('timing trials = %.0f / exact candidates = %.0f',P.r32_timing_trial_count,P.r32_exact_candidate_count));
end
end


function L=path_length(X)
if size(X,1)<2,L=0;else,L=sum(sqrt(sum(diff(X,1,1).^2,2)));end
end

function v=vec3(x)
v=[NaN NaN NaN];
if isnumeric(x)&&numel(x)>=3
    x=double(x(:));if all(isfinite(x(1:3))),v=x(1:3).';end
end
end

function y=scalar(x)
y=NaN;if (isnumeric(x)||islogical(x))&&isscalar(x)&&isfinite(double(x)),y=double(x);end
end

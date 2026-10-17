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
%% Initialize settings
% set path
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
% clearvars; close all; clc;

%% 1. SIMと同じ基本設定
ts = 0;          % initial time
dt = 0.025;      % sampling period
te = 50;         % terminal time
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

% 公称直線軌道
nominal_ref = TIME_VARYING_REFERENCE(agent, ...
    {"gen_ref_line", { ...
    "p0", [0, 0, 3], ...
    "velocity", 1.5, ...
    "direction", [0, 0, 1] ...
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

% Trim preallocated timing arrays to actual logged length.
Krt=runtime_profile.k;
runtime_profile.t=runtime_profile.t(1:Krt);
runtime_profile.phase=runtime_profile.phase(1:Krt);
runtime_profile.sensor_acquisition_s=runtime_profile.sensor_acquisition_s(1:Krt);
runtime_profile.online_before_plant_s=runtime_profile.online_before_plant_s(1:Krt);
runtime_profile.simulation_plant_s=runtime_profile.simulation_plant_s(1:Krt);
runtime_profile.logger_s=runtime_profile.logger_s(1:Krt);
runtime_profile.full_iteration_s=runtime_profile.full_iteration_s(1:Krt);

disp("Simulation complete! Drawing plots...");

%% AF7R51 single post-run dashboard -- replaces fragmented Figures 1--11.
assert(exist('replanner_instance','var')==1, ...
    '[AF7R51P] replanner_instance is unavailable after the run.');
AF7R51_POSTRUN_DASHBOARD(logger,runtime_profile,dt,replanner_instance);
drawnow;

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

function OUT = AF7R7_CURRENT_SYSTEM_AUDIT_V3(agent,logger,replanner)
% AF7R7_CURRENT_SYSTEM_AUDIT
% Post-run read-only audit for the current integrated REPLANNING_SYSTEM.
%
% Usage:
%   AF7R7_CURRENT_SYSTEM_AUDIT
% or
%   AF7R7_CURRENT_SYSTEM_AUDIT(agent,logger,replanner_instance)

clc;
fprintf('\n');
fprintf('====================================================================================================\n');
fprintf(' AF7R7 | CURRENT INTEGRATED SYSTEM POST-RUN AUDIT V3\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] readOnly=1 | source+runtime=1 | sampledAcceptance=0 | trajectoryModification=0\n');

if nargin<1 || isempty(agent)
    assert(evalin('base','exist(''agent'',''var'')==1'), ...
        '[AF7R7 AUDIT] agent not found in base workspace.');
    agent=evalin('base','agent');
end
if nargin<2 || isempty(logger)
    assert(evalin('base','exist(''logger'',''var'')==1'), ...
        '[AF7R7 AUDIT] logger not found in base workspace.');
    logger=evalin('base','logger');
end
if nargin<3 || isempty(replanner)
    if evalin('base','exist(''replanner_instance'',''var'')==1')
        replanner=evalin('base','replanner_instance');
    else
        try
            replanner=agent.reference.timevarying;
        catch
            error('[AF7R7 AUDIT] replanner_instance not found.');
        end
    end
end

OUT=struct();
OUT.schema='AF7R7_CURRENT_SYSTEM_AUDIT_V3';
OUT.read_only=true;
OUT.final_integrated_authority_closed=false;

%% A. Source architecture
fprintf('\n[A. SOURCE ARCHITECTURE]\n');

rf=which('REPLANNING_SYSTEM');
assert(~isempty(rf)&&isfile(rf),'[AF7R7 AUDIT] REPLANNING_SYSTEM.m unresolved.');
raw=fileread(rf);
active=strip_comments(raw);

SRC=struct();
SRC.selfcontained_bezier = contains(active,'builtin_bezier_d15_c7_proposal');
SRC.old_external_R23_dependency = ...
    contains(active,'@G27V74R23_BEZIER_QARC_CTGEO_SHADOW_GENERATOR') || ...
    contains(active,'G27V74R23_BEZIER_QARC_CTGEO_SHADOW_GENERATOR.m');
SRC.base_workspace_capture = ...
    contains(active,'G27V74R23_CAPTURED_CTX') || ...
    contains(active,'G27V74R23_CAPTURED_PROPOSAL') || ...
    contains(active,'assignin(');
SRC.external_AF7R7_runtime_call = ...
    ~isempty(regexp(active,'AF7R7[A-Za-z0-9_]*\s*\(','once'));

invokeBody=method_body(active, ...
    'function invoke_replan_generator_shadow', ...
    'function OUT = builtin_bezier_d15_c7_proposal');
SRC.generator_auto_commit_connected = ...
    contains(invokeBody,'commit_repaired_reference(');

SRC.commit_interface_exists = ...
    contains(active,'function commit_repaired_reference(');

SRC.production_continuous_wholebody_builder = ...
    ~isempty(regexp(active, ...
    'function[^\n]*(continuous_whole|whole_body|wholebody)[^\n]*(cert|certificate)', ...
    'once'));

SRC.hlc_robust_tracking_installed = ...
    ~contains(active,'HLC robust tracking-error certificate not installed yet');

SRC.static_continuous_wholebody_method = ...
    contains(active,'build_static_continuous_wholebody_shadow_certificate');
SRC.constant_speed_required = ...
    contains(active,'constant_norm_required=true');
SRC.general_nominal_bound_interface = ...
    contains(active,'NOMINAL_DERIVATIVE_BOUND_FN');
SRC.legacy_vertical_actuator_enabled = ...
    contains(active,'legacy_positive_vertical_theorem_enabled=true');

SRC.observer_switch_default_on = ...
    contains(regexprep(active,'\s+',''), ...
    'ENABLE_OBSERVER_HARD_SUPPORT=true;');

fprintf(' source                                  = %s\n',rf);
fprintf(' self-contained BEZIER proposal          = %d\n',SRC.selfcontained_bezier);
fprintf(' old external R23 dependency             = %d\n',SRC.old_external_R23_dependency);
fprintf(' base-workspace capture                  = %d\n',SRC.base_workspace_capture);
fprintf(' external AF7R7 runtime helper call      = %d\n',SRC.external_AF7R7_runtime_call);
fprintf(' commit interface exists                 = %d\n',SRC.commit_interface_exists);
fprintf(' generator -> automatic commit connected = %d\n',SRC.generator_auto_commit_connected);
fprintf(' continuous whole-body builder connected = %d\n',SRC.production_continuous_wholebody_builder);
fprintf(' HLC robust tracking certificate installed = %d\n',SRC.hlc_robust_tracking_installed);
fprintf(' static continuous whole-body method     = %d\n',SRC.static_continuous_wholebody_method);
fprintf(' constant-speed requirement present      = %d\n',SRC.constant_speed_required);
fprintf(' general nominal bound interface         = %d\n',SRC.general_nominal_bound_interface);
fprintf(' legacy vertical actuator theorem enabled = %d\n',SRC.legacy_vertical_actuator_enabled);
fprintf(' observer default ON                     = %d\n',SRC.observer_switch_default_on);

OUT.source=SRC;

%% B. Simulation/logger integrity
fprintf('\n[B. SIMULATION / LOGGER INTEGRITY]\n');

D=member(logger,'Data');
assert(isstruct(D)&&isfield(D,'agent'),'[AF7R7 AUDIT] logger.Data.agent missing.');
A=D.agent;
if numel(A)>1, A=A(1); end

Rref=result_cells(member(member(A,'reference'),'result'));
Rest=result_cells(member(member(A,'estimator'),'result'));
Rctl=result_cells(member(member(A,'controller'),'result'));

N=max([numel(Rref),numel(Rest),numel(Rctl)]);
if isfield(D,'t') && ~isempty(D.t)
    t=double(D.t(:));
else
    t=(0:N-1)'*0.025;
end
N=min(N,numel(t));
t=t(1:N);

Rref=pad_cells(Rref,N);
Rest=pad_cells(Rest,N);
Rctl=pad_cells(Rctl,N);

refPosFinite=count_finite_vector(Rref,'state.xd',3);
ref28Finite=count_finite_vector(Rref,'state.xd',28);
estPFinite=count_finite_vector(Rest,'state.p',3);
estPLFinite=count_finite_vector(Rest,'state.pL',3);
ctlInputFinite=count_finite_vector(Rctl,'input',4);

% REPLANNING_SYSTEM is allocated only in Flight.  Detect those samples
% from its own runtime field rather than hardcoding any Flight start time.
rt_pre=series_scalar(Rref,'state.rt_light_e2e_ms');
flightRefMask=isfinite(rt_pre);
flightCount=nnz(flightRefMask);
flightRef28Finite=count_finite_vector_mask(Rref,'state.xd',28,flightRefMask);

RUN=struct();
RUN.samples=N;
RUN.ref_position_finite=refPosFinite;
RUN.ref28_finite_all=ref28Finite;
RUN.flight_samples=flightCount;
RUN.flight_ref28_finite=flightRef28Finite;
RUN.est_p_finite=estPFinite;
RUN.est_pL_finite=estPLFinite;
RUN.controller_input_finite=ctlInputFinite;

% Phase-aware integrity:
% - every logged phase must have a finite 3-D reference position,
% - the Flight REPLANNING_SYSTEM samples must have the full 28-D HLC packet,
% - estimator/controller must be finite over the run.
RUN.basic_chain_complete = N>0 && ...
    refPosFinite>0.95*N && ...
    (flightCount==0 || flightRef28Finite>0.95*flightCount) && ...
    estPFinite>0.95*N && estPLFinite>0.95*N && ...
    ctlInputFinite>0.95*N;

fprintf(' logged samples                          = %d\n',N);
if N>0
    fprintf(' time range                              = %.6f -- %.6f s\n',t(1),t(end));
end
fprintf(' finite reference xd(1:3) samples       = %d / %d\n',refPosFinite,N);
fprintf(' full 28-D reference samples (all)      = %d / %d\n',ref28Finite,N);
fprintf(' Flight REPLANNING_SYSTEM samples       = %d\n',flightCount);
fprintf(' finite Flight xd(1:28) samples         = %d / %d\n',flightRef28Finite,flightCount);
fprintf(' finite estimator p samples             = %d / %d\n',estPFinite,N);
fprintf(' finite estimator pL samples            = %d / %d\n',estPLFinite,N);
fprintf(' finite controller input samples        = %d / %d\n',ctlInputFinite,N);
fprintf(' phase-aware S/E/R/C chain integrity     = %d\n',RUN.basic_chain_complete);

%% C. Current certificate status
fprintf('\n[C. CURRENT CERTIFICATE STATUS]\n');

try
    CS=replanner.get_runtime_certificate_status();
catch ME
    fprintf(' get_runtime_certificate_status failed: %s\n',ME.message);
    CS=struct();
end

CERT=struct();
CERT.speed_trigger=certified(member(CS,'speed_trigger'));
CERT.sensor_horizon=certified(member(CS,'sensor_horizon'));
CERT.stop_geometry_4ms=certified(member(CS,'stop_geometry_4ms'));
CERT.command_limit=certified(member(CS,'command_limit'));
CERT.actuator_dynamics=certified(member(CS,'actuator_dynamics'));
CERT.hlc_robust=certified(member(CS,'hlc_robust'));
CERT.live_controller_verified=logical_scalar(member(CS,'live_controller_verified'));
CERT.live_gain_verified=logical_scalar(member(CS,'live_gain_verified'));

show_bool_struct(CERT);
OUT.certificates=CERT;

%% D. Observer hard-support
fprintf('\n[D. OBSERVER HARD-SUPPORT]\n');

try
    OS=replanner.get_observer_hard_support_status();
catch ME
    fprintf(' observer status failed: %s\n',ME.message);
    OS=struct();
end

OBS=struct();
OBS.enabled=logical_scalar(member(OS,'enabled'));
OBS.initialized=logical_scalar(member(OS,'initialized'));
OBS.selector_verified=logical_scalar(member(OS,'selector_verified'));
OBS.fail_count=scalar_or_nan(member(OS,'fail_count'));
OBS.update_count=scalar_or_nan(member(OS,'update_count'));
OBS.support_closed=closed_or_certified(member(OS,'observer_support'));
OBS.hlc_state_mass_bridge_closed=closed_or_certified(member(OS,'hlc_state_mass_bridge'));
OBS.current_saturation_closed=logical_scalar(member(OS,'current_saturation_closed'));
OBS.runtime_last_ms=scalar_or_nan(member(OS,'runtime_last_ms'));
OBS.runtime_max_ms=scalar_or_nan(member(OS,'runtime_max_ms'));
OBS.runtime_mean_ms=scalar_or_nan(member(OS,'runtime_mean_ms'));

fprintf(' enabled                                 = %d\n',OBS.enabled);
fprintf(' initialized                             = %d\n',OBS.initialized);
fprintf(' selector verified                       = %d\n',OBS.selector_verified);
fprintf(' updates / fails                         = %.0f / %.0f\n',OBS.update_count,OBS.fail_count);
fprintf(' hard support closed                     = %d\n',OBS.support_closed);
fprintf(' HLC state/mass bridge closed            = %d\n',OBS.hlc_state_mass_bridge_closed);
fprintf(' current saturation effect closed        = %d\n',OBS.current_saturation_closed);
fprintf(' observer runtime last/max/mean [ms]     = %.6f / %.6f / %.6f\n', ...
    OBS.runtime_last_ms,OBS.runtime_max_ms,OBS.runtime_mean_ms);

OUT.observer=OBS;

%% E. Replanning pipeline from actual log
fprintf('\n[E. REPLANNING PIPELINE -- ACTUAL RUN]\n');

rt=rt_pre;
flightMask=isfinite(rt);

det=series_scalar(Rref,'state.detected_obstacle_count_point');
need=series_scalar(Rref,'state.need_replan');
kcalled=series_scalar(Rref,'state.trigger_kernel_called');
tevent=series_scalar(Rref,'state.trigger_event_count');
greq=series_scalar(Rref,'state.generator_request_count');
gshadow=series_scalar(Rref,'state.generator_shadow_proposal_count');
bgen=series_scalar(Rref,'state.bezier_shadow_generated_count');
bpeak=series_scalar(Rref,'state.bezier_shadow_peak_safe_count');
gpass=series_scalar(Rref,'state.generator_validation_pass_count');
gfail=series_scalar(Rref,'state.generator_validation_fail_count');
gdead=series_scalar(Rref,'state.generator_deadline_fail_count');
gcode=series_scalar(Rref,'state.generator_last_status_code');
geomPass=series_scalar(Rref,'state.geometry_shadow_pass');
geomCT=series_scalar(Rref,'state.geometry_shadow_continuous_time');
geomFP=series_scalar(Rref,'state.geometry_shadow_ieee754_outward_closed');
commits=series_scalar(Rref,'state.replan_commit_count');
staleHlc=series_scalar(Rref,'state.hlc_optional_stale');

% Last shadow-proposal forensic fields are persisted in logger state.
breason=series_scalar(Rref,'state.bezier_shadow_last_reason_code');
bobs=series_scalar(Rref,'state.bezier_shadow_last_obstacle_id');
bcomp=series_scalar(Rref,'state.bezier_shadow_last_component_code');
bd=series_scalar(Rref,'state.bezier_shadow_last_displacement_m');
bmb=series_scalar(Rref,'state.bezier_shadow_last_metric_before');
bma=series_scalar(Rref,'state.bezier_shadow_last_metric_after');
bmp=series_scalar(Rref,'state.bezier_shadow_last_peak_min_metric');
bc7=series_scalar(Rref,'state.bezier_shadow_last_c7_exact');
bd1=series_scalar(Rref,'state.bezier_shadow_last_d1_ub');
bd2=series_scalar(Rref,'state.bezier_shadow_last_d2_ub');
bd3=series_scalar(Rref,'state.bezier_shadow_last_d3_ub');
bd4=series_scalar(Rref,'state.bezier_shadow_last_d4_ub');
bd5=series_scalar(Rref,'state.bezier_shadow_last_d5_ub');
bd6=series_scalar(Rref,'state.bezier_shadow_last_d6_ub');

P=struct();
P.flight_samples=nnz(flightMask);
P.max_detected_obstacles=maxfinite(det);
P.need_replan_samples=nnz(need>0.5);
P.trigger_kernel_called_samples=nnz(kcalled>0.5);
P.trigger_event_count=maxfinite(tevent);
P.generator_request_count=maxfinite(greq);
P.shadow_proposal_count=maxfinite(gshadow);
P.bezier_generated_count=maxfinite(bgen);
P.bezier_peak_safe_count=maxfinite(bpeak);
P.generator_validation_pass_count=maxfinite(gpass);
P.generator_validation_fail_count=maxfinite(gfail);
P.generator_deadline_fail_count=maxfinite(gdead);
P.generator_last_status_code=lastfinite(gcode);
P.geometry_shadow_pass=any(geomPass>0.5);
P.geometry_shadow_continuous_time=any(geomCT>0.5);
P.geometry_shadow_ieee754_outward_closed=any(geomFP>0.5);
P.replan_commit_count=maxfinite(commits);
P.hlc_optional_stale_seen=any(staleHlc>0.5);
P.bezier_reason_code=lastfinite(breason);
P.bezier_reason_text=bezier_reason_text(P.bezier_reason_code);
P.bezier_obstacle_id=lastfinite(bobs);
P.bezier_component_code=lastfinite(bcomp);
P.bezier_displacement_m=lastfinite(bd);
P.bezier_metric_before=lastfinite(bmb);
P.bezier_metric_after=lastfinite(bma);
P.bezier_peak_min_metric=lastfinite(bmp);
P.bezier_c7_exact=lastfinite(bc7);
P.bezier_derivative_ub=[lastfinite(bd1),lastfinite(bd2), ...
    lastfinite(bd3),lastfinite(bd4),lastfinite(bd5),lastfinite(bd6)];

fprintf(' Flight replanner samples                = %d\n',P.flight_samples);
fprintf(' max detected obstacles                 = %.0f\n',P.max_detected_obstacles);
fprintf(' need_replan samples                    = %d\n',P.need_replan_samples);
fprintf(' MEX trigger-kernel called samples      = %d\n',P.trigger_kernel_called_samples);
fprintf(' latched real trigger events            = %.0f\n',P.trigger_event_count);
fprintf(' generator requests                     = %.0f\n',P.generator_request_count);
fprintf(' shadow proposals                       = %.0f\n',P.shadow_proposal_count);
fprintf(' BEZIER generated                       = %.0f\n',P.bezier_generated_count);
fprintf(' BEZIER peak-safe                       = %.0f\n',P.bezier_peak_safe_count);
fprintf(' generic validation pass/fail           = %.0f / %.0f\n', ...
    P.generator_validation_pass_count,P.generator_validation_fail_count);
fprintf(' generator deadline fails               = %.0f\n',P.generator_deadline_fail_count);
fprintf(' last generator status code             = %.0f\n',P.generator_last_status_code);
fprintf(' continuous geometry shadow pass        = %d\n',P.geometry_shadow_pass);
fprintf(' geometry continuous-time flag          = %d\n',P.geometry_shadow_continuous_time);
fprintf(' geometry outward-FP flag               = %d\n',P.geometry_shadow_ieee754_outward_closed);
fprintf(' committed repaired references          = %.0f\n',P.replan_commit_count);

fprintf('\n[E2. BEZIER PROPOSAL FORENSIC]\n');
fprintf(' shadow reason code                     = %.0f\n',P.bezier_reason_code);
fprintf(' shadow reason                          = %s\n',P.bezier_reason_text);
fprintf(' triggering obstacle ID                 = %.0f\n',P.bezier_obstacle_id);
fprintf(' triggering component code              = %.0f\n',P.bezier_component_code);
fprintf(' displacement norm [m]                  = %.9g\n',P.bezier_displacement_m);
fprintf(' trigger metric before                  = %.9g\n',P.bezier_metric_before);
fprintf(' trigger metric after                   = %.9g\n',P.bezier_metric_after);
fprintf(' min peak metric across detected set    = %.9g\n',P.bezier_peak_min_metric);
fprintf(' C7 exact flag                          = %.0f\n',P.bezier_c7_exact);
fprintf(' derivative UB d1..d6                   = [%s]\n', ...
    sprintf(' %.6g',P.bezier_derivative_ub));

bt0=lastfinite(series_scalar(Rref,'state.bezier_shadow_last_t0'));
btm=lastfinite(series_scalar(Rref,'state.bezier_shadow_last_tm'));
bt1=lastfinite(series_scalar(Rref,'state.bezier_shadow_last_t1'));
fprintf(' proposal t0 / tm / t1 [s]              = %.9g / %.9g / %.9g\n',bt0,btm,bt1);
fprintf(' proposal Tup / Tdown [s]               = %.9g / %.9g\n',btm-bt0,bt1-btm);

try
    EV=replanner.get_first_trigger_event();
catch
    EV=struct();
end
evt=scalar_or_nan(member(EV,'event_time'));
ET=member(EV,'trigger');
evttc=scalar_or_nan(member(ET,'ttc'));
evtc=scalar_or_nan(member(ET,'collision_time'));
evj=scalar_or_nan(member(ET,'hit_index'));
fprintf(' first real trigger event t [s]          = %.9g\n',evt);
fprintf(' first real trigger TTC [s]              = %.9g\n',evttc);
fprintf(' first real collision witness time [s]   = %.9g\n',evtc);
fprintf(' first real trigger hit index            = %.0f\n',evj);

OUT.pipeline=P;
OUT.first_trigger_event=EV;

%% F. Timing
fprintf('\n[F. TIMING SCOPE + MEASURED DISTRIBUTIONS]\n');

T=struct();

rt=rt_pre;
flightMask=isfinite(rt);
T.replanner_light=stats_ms(rt(flightMask));

obsCycle=series_scalar(Rref,'state.rt_observer_ms');
T.observer_cycle=stats_ms(obsCycle(flightMask & isfinite(obsCycle)));

online_s=series_scalar(Rctl,'online_compute_total');
plant_s=series_scalar(Rctl,'simulation_plant_total');
full_s=series_scalar(Rctl,'controllertime_total');

T.online_from_controller_log=stats_ms(1e3*online_s(flightMask & isfinite(online_s)));
T.sim_plant_from_controller_log=stats_ms(1e3*plant_s(flightMask & isfinite(plant_s)));
T.full_control_plus_plant=stats_ms(1e3*full_s(flightMask & isfinite(full_s)));

RP=[];
if evalin('base','exist(''runtime_profile'',''var'')')==1
    RP=evalin('base','runtime_profile');
end

T.runtime_profile_available=isstruct(RP) && ...
    isfield(RP,'phase') && isfield(RP,'online_before_plant_s');

if T.runtime_profile_available
    fm=(RP.phase=='f');
    T.sensor_acquisition=stats_ms(1e3*RP.sensor_acquisition_s(fm));
    T.online_before_sim_plant=stats_ms(1e3*RP.online_before_plant_s(fm));
    T.simulation_plant=stats_ms(1e3*RP.simulation_plant_s(fm));
    T.logger=stats_ms(1e3*RP.logger_s(fm));
    T.full_iteration_including_logger=stats_ms(1e3*RP.full_iteration_s(fm));
else
    T.sensor_acquisition=stats_ms([]);
    T.online_before_sim_plant=T.online_from_controller_log;
    T.simulation_plant=T.sim_plant_from_controller_log;
    T.logger=stats_ms([]);
    T.full_iteration_including_logger=stats_ms([]);
end

runf=which('run_flight_headless');
T.run_source=runf;
T.outer_timer_before_sensor_acquisition=false;
T.sim_plant_separate=false;
T.logger_separate=false;

if ~isempty(runf) && isfile(runf)
    rr=fileread(runf);
    loopStart=strfind(rr,'while time.t <= time.te');
    if ~isempty(loopStart)
        tail=rr(loopStart(1):end);
        a=strfind(tail,'tIteration=tic;');
        b=strfind(tail,'motive.getData(agent)');
        c=strfind(tail,'online_before_plant_s=toc(tIteration)');
        d=strfind(tail,'do_prop_local(time,cha,cha,logger,env,agent,"plant"');
        e=strfind(tail,'tLogger=tic;');
        if ~isempty(a)&&~isempty(b)
            T.outer_timer_before_sensor_acquisition=a(1)<b(1);
        end
        if ~isempty(c)&&~isempty(d)
            T.sim_plant_separate=c(1)<d(1);
        end
        if ~isempty(e)&&~isempty(d)
            T.logger_separate=e(1)>d(1);
        end
    end
end

fprintf(' timing source outer-before-acquisition  = %d\n',T.outer_timer_before_sensor_acquisition);
fprintf(' numerical sim plant measured separately = %d\n',T.sim_plant_separate);
fprintf(' LOGGER measured separately              = %d\n',T.logger_separate);

print_stats('replanner internal E2E',T.replanner_light);
print_stats('observer per-cycle',T.observer_cycle);
print_stats('sensor acquisition',T.sensor_acquisition);
print_stats('ONLINE before simulation plant',T.online_before_sim_plant);
print_stats('simulation numerical plant',T.simulation_plant);
print_stats('LOGGER only',T.logger);
print_stats('full simulation iteration incl LOGGER',T.full_iteration_including_logger);

T.measured_online_under_4ms = ...
    isfinite(T.online_before_sim_plant.max) && ...
    T.online_before_sim_plant.max<=4;

fprintf(' measured ONLINE max <= 4 ms             = %d\n',T.measured_online_under_4ms);
fprintf(' formal WCET / final integrated authority = 0\n');
fprintf([' NOTE: tic/toc on this MATLAB/Windows simulation is an empirical ' ...
    'wall-clock measurement, not a formal target-hardware WCET proof.\n']);

OUT.timing=T;

%% G. Feature classification
fprintf('\n[G. FEATURE CLASSIFICATION]\n');

F=struct();
F.baseline_simulation = status(RUN.basic_chain_complete,true);
F.nominal_certificates = status( ...
    CERT.speed_trigger && CERT.sensor_horizon && ...
    CERT.stop_geometry_4ms && CERT.command_limit,true);
F.actuator_dynamics_certificate = status(CERT.actuator_dynamics,true);
F.observer_hard_support = status( ...
    OBS.enabled && OBS.initialized && OBS.selector_verified && ...
    OBS.fail_count==0 && OBS.support_closed,OBS.update_count>0);
F.observer_to_HLC_state_mass = status( ...
    OBS.hlc_state_mass_bridge_closed,OBS.update_count>0);
F.obstacle_sensor_gate = status(true, ...
    isfinite(P.max_detected_obstacles) && P.max_detected_obstacles>0);
F.future_collision_trigger = status(true,P.trigger_kernel_called_samples>0);
F.real_trigger_edge = status(true,P.trigger_event_count>0);
if ~(SRC.selfcontained_bezier && ~SRC.old_external_R23_dependency)
    F.selfcontained_C7_proposal='OPEN_NOT_IMPLEMENTED';
elseif P.generator_request_count<=0
    F.selfcontained_C7_proposal='IMPLEMENTED_NOT_EXERCISED';
elseif P.bezier_generated_count>0
    F.selfcontained_C7_proposal='IMPLEMENTED_AND_EXERCISED';
else
    F.selfcontained_C7_proposal='IMPLEMENTED_AND_EXERCISED__PROPOSAL_FAILED';
end
if ~SRC.static_continuous_wholebody_method
    F.continuous_wholebody_generated_path='OPEN_NOT_IMPLEMENTED';
elseif P.generator_request_count<=0
    F.continuous_wholebody_generated_path='IMPLEMENTED_NOT_EXERCISED';
elseif P.geometry_shadow_continuous_time && P.geometry_shadow_pass
    if P.geometry_shadow_ieee754_outward_closed
        F.continuous_wholebody_generated_path='IMPLEMENTED_AND_NUMERICALLY_CLOSED';
    else
        F.continuous_wholebody_generated_path= ...
            'IMPLEMENTED_MATHEMATICAL_CONTINUOUS_PASS__IEEE754_OUTWARD_OPEN';
    end
else
    F.continuous_wholebody_generated_path='IMPLEMENTED_AND_EXERCISED__CERTIFICATE_FAILED';
end
F.HLC_tracking_tube = ...
    ternary(CERT.hlc_robust,'IMPLEMENTED_AND_CLOSED','OPEN_NOT_INSTALLED');
F.saturation_virtual_effect = ...
    ternary(OBS.current_saturation_closed, ...
    'IMPLEMENTED_AND_CLOSED','OPEN_NOT_CLOSED');
F.automatic_certified_commit = ...
    ternary(SRC.generator_auto_commit_connected && P.replan_commit_count>0, ...
    'IMPLEMENTED_AND_EXERCISED','OPEN_NOT_CONNECTED');
F.active_obstacle_avoidance_reference = ...
    ternary(P.replan_commit_count>0, ...
    'IMPLEMENTED_AND_EXERCISED','NOT_ACTIVE__NO_CERTIFIED_COMMIT');
F.final_integrated_4ms = 'OPEN__NOT_ALL_REQUIRED_LAYERS_CONNECTED';

print_feature('Baseline S/E/R/C/P simulation',F.baseline_simulation);
print_feature('Nominal geometric/source certificates',F.nominal_certificates);
print_feature('Actuator dynamics certificate',F.actuator_dynamics_certificate);
print_feature('Observer hard support',F.observer_hard_support);
print_feature('Observer -> HLC state/mass bound',F.observer_to_HLC_state_mass);
print_feature('Obstacle sensor gate',F.obstacle_sensor_gate);
print_feature('Future collision trigger/MEX',F.future_collision_trigger);
print_feature('Real trigger-event latch',F.real_trigger_edge);
print_feature('Self-contained C7 BEZIER proposal',F.selfcontained_C7_proposal);
print_feature('Continuous all-span whole-body cert',F.continuous_wholebody_generated_path);
print_feature('HLC tracking tube / robust cert',F.HLC_tracking_tube);
print_feature('Saturation virtual effect',F.saturation_virtual_effect);
print_feature('Automatic certified commit',F.automatic_certified_commit);
print_feature('Active repaired avoidance reference',F.active_obstacle_avoidance_reference);
print_feature('Final integrated <=4 ms authority',F.final_integrated_4ms);

OUT.features=F;

%% H. Generality audit
fprintf('\n[H. GENERALITY / NON-CASE-SPECIFIC AUDIT]\n');

GEN=struct();
GEN.static_obstacle_scope_declared = ...
    contains(active,'build_static_continuous_wholebody_shadow_certificate');
GEN.obstacle_count_runtime = contains(active,'for ii=1:double(O.count)');
GEN.obstacle_pose_runtime = contains(active,'rotations_flat') && contains(active,'centers');
GEN.obstacle_axes_runtime = contains(active,'base_axes');
GEN.trigger_uses_runtime_obstacle_id = contains(active,'trigger_obstacle_id');
GEN.reference_uses_active_mode = contains(active,'active_reference_mode');
GEN.no_sampled_acceptance_in_geometry = ...
    contains(active,'''sampled_acceptance'',false');
GEN.constant_speed_required = SRC.constant_speed_required;
GEN.general_nominal_bound_interface = SRC.general_nominal_bound_interface;
GEN.legacy_vertical_actuator_enabled = SRC.legacy_vertical_actuator_enabled;

show_bool_struct(GEN);

fprintf(' current obstacle scope                   = STATIC oriented ellipsoids\n');
fprintf(' moving-obstacle theorem required now     = 0\n');

if GEN.constant_speed_required
    fprintf(' WARNING: constant-speed requirement still exists and must be removed.\n');
end
if ~GEN.general_nominal_bound_interface
    fprintf(' WARNING: general continuous nominal derivative-bound interface missing.\n');
end
if GEN.legacy_vertical_actuator_enabled
    fprintf(' WARNING: legacy vertical actuator theorem still enabled.\n');
end

OUT.generality=GEN;

%% I. Remaining work
fprintf('\n[I. REMAINING PRODUCTION WORK]\n');
open={};

if ~SRC.static_continuous_wholebody_method
    open{end+1}= ...
        'static continuous all-span/all-obstacle whole-body geometry theorem'; %#ok<AGROW>
elseif ~P.geometry_shadow_ieee754_outward_closed
    open{end+1}= ...
        'IEEE-754 outward-rounded numerical authority for the continuous geometry certificate'; %#ok<AGROW>
end

if ~CERT.hlc_robust
    open{end+1}= ...
        'general HLC robust tracking tube / observer-to-tracking disturbance fixed point'; %#ok<AGROW>
end
if ~OBS.current_saturation_closed
    open{end+1}= ...
        'same-cycle actuator saturation / nonlinear virtual-mismatch closure'; %#ok<AGROW>
end
if ~SRC.generator_auto_commit_connected || P.replan_commit_count==0
    open{end+1}= ...
        'generator -> geometry + tracking + actuator certificates -> commit_repaired_reference'; %#ok<AGROW>
end
open{end+1}= ...
    'final same-cycle all-online worst-case <=4 ms on the complete required online stack'; %#ok<AGROW>

for k=1:numel(open)
    fprintf(' %d) %s\n',k,open{k});
end

OUT.remaining=open;
OUT.final_integrated_authority_closed=false;

fprintf('\n[J. FINAL CLASSIFICATION]\n');
fprintf(' current simulation chain working       = %d\n',RUN.basic_chain_complete);
fprintf(' current observer chain working         = %d\n', ...
    OBS.enabled && OBS.initialized && OBS.fail_count==0 && OBS.support_closed);
fprintf(' actual certified avoidance committed   = %d\n',P.replan_commit_count>0);
fprintf(' final integrated authority closed      = 0\n');

OUT.classification= ...
    'BASELINE_AND_OBSERVER_RUNNING__SELFCONTAINED_C7_PROPOSAL_AVAILABLE__CONTINUOUS_TRACKING_COMMIT_AND_FINAL_4MS_OPEN';
fprintf(' classification                         = %s\n',OUT.classification);
fprintf('====================================================================================================\n');

assignin('base','AF7R7_CURRENT_SYSTEM_AUDIT_OUT',OUT);
end

function x=member(S,name)
x=[];
try
    if isempty(S), return; end
    if isstruct(S)
        if isfield(S,name), x=S.(name); end
    elseif isobject(S)
        if isprop(S,name), x=S.(name); end
    end
catch
    x=[];
end
end

function x=nested_member(S,path)
x=S;
parts=strsplit(path,'.');
for k=1:numel(parts)
    x=member(x,parts{k});
    if isempty(x), return; end
end
end

function C=result_cells(X)
if isempty(X)
    C={};
elseif iscell(X)
    C=X(:);
elseif isstruct(X) || isobject(X)
    C=num2cell(X(:));
else
    C={};
end
end

function C=pad_cells(C,N)
C=C(:);
if numel(C)<N
    C(end+1:N,1)={[]};
else
    C=C(1:N);
end
end

function n=count_finite_vector(C,path,m)
n=0;
for k=1:numel(C)
    x=nested_member(C{k},path);
    try
        x=double(x(:));
        if numel(x)>=m && all(isfinite(x(1:m)))
            n=n+1;
        end
    catch
    end
end
end

function n=count_finite_vector_mask(C,path,m,mask)
n=0;
mask=logical(mask(:));
N=min(numel(C),numel(mask));
for k=1:N
    if ~mask(k), continue; end
    x=nested_member(C{k},path);
    try
        x=double(x(:));
        if numel(x)>=m && all(isfinite(x(1:m)))
            n=n+1;
        end
    catch
    end
end
end

function v=series_scalar(C,path)
v=nan(numel(C),1);
for k=1:numel(C)
    x=nested_member(C{k},path);
    try
        if isnumeric(x)||islogical(x)
            x=double(x);
            if isscalar(x), v(k)=x; end
        end
    catch
    end
end
end

function tf=certified(C)
tf=false;
try
    tf=isstruct(C) && isfield(C,'certified') && ...
        isscalar(C.certified) && logical(C.certified);
catch
end
end

function tf=closed_or_certified(C)
tf=false;
try
    if isstruct(C)
        if isfield(C,'closed') && isscalar(C.closed)
            tf=logical(C.closed);
            return;
        end
        if isfield(C,'certified') && isscalar(C.certified)
            tf=logical(C.certified);
            return;
        end
    end
catch
end
end

function tf=logical_scalar(x)
tf=false;
try
    tf=(islogical(x)||isnumeric(x)) && isscalar(x) && ...
        isfinite(double(x)) && logical(x);
catch
end
end

function x=scalar_or_nan(x)
try
    x=double(x);
    if ~(isscalar(x)&&isfinite(x)), x=NaN; end
catch
    x=NaN;
end
end

function y=maxfinite(x)
x=double(x(:));
x=x(isfinite(x));
if isempty(x), y=NaN; else, y=max(x); end
end

function y=lastfinite(x)
x=double(x(:));
i=find(isfinite(x),1,'last');
if isempty(i), y=NaN; else, y=x(i); end
end

function txt=bezier_reason_text(code)
switch round(code)
    case 0
        txt='SUCCESS';
    case 1
        txt='NO_DETECTED_OBSTACLE_IN_REQUEST';
    case 2
        txt='INVALID_OR_NONPOSITIVE_TRIGGER_TTC';
    case 3
        txt='INVALID_TIME_ORDER_T0_TM_T1';
    case 4
        txt='TRIGGER_OBSTACLE_ID_NOT_IN_DETECTED_PACKET';
    case 5
        txt='INVALID_TRIGGER_COMPONENT_CODE';
    case 6
        txt='INVALID_ELLIPSOID_OR_TRIGGER_WITNESS';
    case 7
        txt='INVALID_OUTWARD_NORMAL';
    case 8
        txt='LAST_EXIT_SOLVE_FAILED_FOR_DETECTED_GEOMETRY';
    case 9
        txt='INVALID_CORRECTION_UP_DOWN_TIME';
    case 11
        txt='TRIGGER_COMPONENT_NOT_STRICTLY_EXTERIOR_AT_PEAK';
    case 12
        txt='AT_LEAST_ONE_DETECTED_PAYLOAD_UAV_CABLE_PEAK_METRIC_NOT_SAFE';
    case 13
        txt='CONTINUOUS_STATIC_WHOLEBODY_SUPPORT_TUBE_CERTIFICATE_FAILED';
    case -100
        txt='MALFORMED_GENERATOR_CONTEXT';
    case -101
        txt='BACKEND_CONTRACT_MISMATCH';
    otherwise
        if isnan(code)
            txt='NO_RECORDED_REASON';
        else
            txt=sprintf('UNKNOWN_REASON_%g',code);
        end
end
end

function Q=stats_ms(x)
x=double(x(:));
x=x(isfinite(x));
Q=struct('n',numel(x),'median',NaN,'p95',NaN,'p99',NaN,'max',NaN,'mean',NaN);
if isempty(x), return; end
x=sort(x);
Q.median=pct(x,0.50);
Q.p95=pct(x,0.95);
Q.p99=pct(x,0.99);
Q.max=x(end);
Q.mean=mean(x);
end

function y=pct(x,p)
n=numel(x);
if n==1, y=x(1); return; end
z=1+(n-1)*p;
i=floor(z);
a=z-i;
if i>=n
    y=x(end);
else
    y=(1-a)*x(i)+a*x(i+1);
end
end

function s=status(implemented,exercised)
if ~implemented
    s='OPEN_NOT_IMPLEMENTED';
elseif exercised
    s='IMPLEMENTED_AND_EXERCISED';
else
    s='IMPLEMENTED_NOT_EXERCISED';
end
end

function y=ternary(tf,a,b)
if tf, y=a; else, y=b; end
end

function print_feature(name,state)
fprintf(' %-42s : %s\n',name,state);
end

function print_stats(name,Q)
fprintf(' %-37s med/p95/p99/max [ms] = %.6f / %.6f / %.6f / %.6f\n', ...
    name,Q.median,Q.p95,Q.p99,Q.max);
end

function show_bool_struct(S)
f=fieldnames(S);
for k=1:numel(f)
    fprintf(' %-39s = %d\n',f{k},logical(S.(f{k})));
end
end

function s=strip_comments(raw)
raw=strrep(raw,sprintf('\r\n'),sprintf('\n'));
raw=strrep(raw,sprintf('\r'),sprintf('\n'));
L=splitlines(string(raw));
K=strings(0,1);
for i=1:numel(L)
    z=char(L(i));
    p=find(z=='%',1,'first');
    if ~isempty(p), z=z(1:p-1); end
    K(end+1,1)=string(z); %#ok<AGROW>
end
s=char(strjoin(K,newline));
end

function b=method_body(src,startToken,endToken)
i=strfind(src,startToken);
if isempty(i), b=''; return; end
i=i(1);
tail=src(i:end);
j=strfind(tail,endToken);
if isempty(j)
    b=tail;
else
    b=tail(1:j(1)-1);
end
end

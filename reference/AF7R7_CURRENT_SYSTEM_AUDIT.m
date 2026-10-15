function OUT = AF7R7_CURRENT_SYSTEM_AUDIT(agent,logger,replanner)
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
fprintf(' AF7R7 | CURRENT INTEGRATED SYSTEM POST-RUN AUDIT\n');
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
OUT.schema='AF7R7_CURRENT_SYSTEM_AUDIT_V1';
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

refFinite=count_finite_vector(Rref,'state.xd',28);
estPFinite=count_finite_vector(Rest,'state.p',3);
estPLFinite=count_finite_vector(Rest,'state.pL',3);
ctlInputFinite=count_finite_vector(Rctl,'input',4);

RUN=struct();
RUN.samples=N;
RUN.ref_finite=refFinite;
RUN.est_p_finite=estPFinite;
RUN.est_pL_finite=estPLFinite;
RUN.controller_input_finite=ctlInputFinite;
RUN.basic_chain_complete = N>0 && ...
    refFinite>0.95*N && estPFinite>0.95*N && ...
    estPLFinite>0.95*N && ctlInputFinite>0.95*N;

fprintf(' logged samples                          = %d\n',N);
if N>0
    fprintf(' time range                              = %.6f -- %.6f s\n',t(1),t(end));
end
fprintf(' finite reference xd(1:28) samples      = %d / %d\n',refFinite,N);
fprintf(' finite estimator p samples             = %d / %d\n',estPFinite,N);
fprintf(' finite estimator pL samples            = %d / %d\n',estPLFinite,N);
fprintf(' finite controller input samples        = %d / %d\n',ctlInputFinite,N);
fprintf(' S/E/R/C logged-chain integrity          = %d\n',RUN.basic_chain_complete);

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

rt=series_scalar(Rref,'state.rt_light_e2e_ms');
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

OUT.pipeline=P;

%% F. Timing
fprintf('\n[F. TIMING -- MEASURED DIAGNOSTIC, NOT WCET PROOF]\n');

T=struct();
T.replanner_light=stats_ms(rt(flightMask));

full_s=series_scalar(Rctl,'controllertime_total');
full_ms=1e3*full_s;
fm=flightMask & isfinite(full_ms);
T.full_sensor_to_plant=stats_ms(full_ms(fm));

fprintf(' replanner light E2E med/p95/p99/max [ms] = %.6f / %.6f / %.6f / %.6f\n', ...
    T.replanner_light.median,T.replanner_light.p95, ...
    T.replanner_light.p99,T.replanner_light.max);
fprintf(' full sensor->plant med/p95/p99/max [ms]  = %.6f / %.6f / %.6f / %.6f\n', ...
    T.full_sensor_to_plant.median,T.full_sensor_to_plant.p95, ...
    T.full_sensor_to_plant.p99,T.full_sensor_to_plant.max);

T.measured_replanner_under_4ms = ...
    isfinite(T.replanner_light.max) && T.replanner_light.max<=4;
T.measured_full_cycle_under_4ms = ...
    isfinite(T.full_sensor_to_plant.max) && T.full_sensor_to_plant.max<=4;

fprintf(' measured replanner max <= 4 ms         = %d\n',T.measured_replanner_under_4ms);
fprintf(' measured full sim cycle max <= 4 ms    = %d\n',T.measured_full_cycle_under_4ms);
fprintf(' formal/final integrated 4-ms authority = 0\n');

OUT.timing=T;

%% G. Feature classification
fprintf('\n[G. FEATURE CLASSIFICATION]\n');

F=struct();
F.baseline_simulation = status(RUN.basic_chain_complete,true);
F.nominal_certificates = status( ...
    CERT.speed_trigger && CERT.sensor_horizon && ...
    CERT.stop_geometry_4ms && CERT.command_limit,true);
F.observer_hard_support = status( ...
    OBS.enabled && OBS.initialized && OBS.selector_verified && ...
    OBS.fail_count==0 && OBS.support_closed,OBS.update_count>0);
F.observer_to_HLC_state_mass = status( ...
    OBS.hlc_state_mass_bridge_closed,OBS.update_count>0);
F.obstacle_sensor_gate = status(true, ...
    isfinite(P.max_detected_obstacles) && P.max_detected_obstacles>0);
F.future_collision_trigger = status(true,P.trigger_kernel_called_samples>0);
F.real_trigger_edge = status(true,P.trigger_event_count>0);
F.selfcontained_C7_proposal = status( ...
    SRC.selfcontained_bezier && ~SRC.old_external_R23_dependency, ...
    P.bezier_generated_count>0);
F.continuous_wholebody_generated_path = ...
    'OPEN_NOT_PRODUCTION_CONNECTED';
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
print_feature('Nominal safety certificates',F.nominal_certificates);
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

%% H. Remaining work
fprintf('\n[H. REMAINING PRODUCTION WORK]\n');
open={};

if ~SRC.production_continuous_wholebody_builder || ...
        ~P.geometry_shadow_continuous_time
    open{end+1}= ...
        'continuous all-span/all-obstacle whole-body certificate for generated C7 proposal'; %#ok<AGROW>
end
if ~CERT.hlc_robust
    open{end+1}= ...
        'HLC robust tracking tube / observer-to-tracking disturbance fixed point'; %#ok<AGROW>
end
if ~OBS.current_saturation_closed
    open{end+1}= ...
        'same-cycle actuator saturation / nonlinear virtual-mismatch closure'; %#ok<AGROW>
end
if ~SRC.generator_auto_commit_connected || P.replan_commit_count==0
    open{end+1}= ...
        'generator -> certificates -> commit_repaired_reference automatic production connection'; %#ok<AGROW>
end
open{end+1}= ...
    'final same-cycle all-online worst-case <=4 ms measurement after every required layer is connected'; %#ok<AGROW>

for k=1:numel(open)
    fprintf(' %d) %s\n',k,open{k});
end

OUT.remaining=open;
OUT.final_integrated_authority_closed=false;

fprintf('\n[I. FINAL CLASSIFICATION]\n');
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

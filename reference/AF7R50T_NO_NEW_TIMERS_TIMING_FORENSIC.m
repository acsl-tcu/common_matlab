function OUT = AF7R50T_NO_NEW_TIMERS_TIMING_FORENSIC(runtime_profile,logger,replanner)
% AF7R50T_NO_NEW_TIMERS_TIMING_FORENSIC
%
% Read-only timing attribution using ONLY timings already recorded during the
% run.  It adds no tic/toc/QPC checkpoint to the online path.
%
% Separates:
%   online before plant
%     -> REPLANNING_SYSTEM/reference e2e
%     -> non-reference online residual
%   reference e2e
%     -> front / load-mass / dependency / base-cert-geometry /
%        sensor-trigger-replan / output-state
%   request edge
%     -> native core / MATLAB callback wrapper / post-callback remainder
%   simulation-only
%     -> numerical plant / logger / residual
%
% Request edges are detected from increments of generator_request_count,
% not from the persistent status code.

clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R50T | NO-NEW-TIMERS CURRENT-RUN ATTRIBUTION\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] readOnly=1 | usesExistingRunTimestamps=1 | addedOnlineTimers=0 | safetyLogicUnchanged=1\n');

assert(isstruct(runtime_profile),'[AF7R50T] runtime_profile unavailable.');
assert(isa(replanner,'REPLANNING_SYSTEM'),'[AF7R50T] replanner type mismatch.');

[t,refState,onlineLog,plantLog] = extract_logger(logger);
P = normalize_profile(runtime_profile);

% Align by time rather than assuming identical array lengths.
n=numel(t);

% Existing REPLANNING_SYSTEM timing channels.
refE2E = nan(n,1);
front = nan(n,1);
mass = nan(n,1);
dep = nan(n,1);
baseGeom = nan(n,1);
sensorTrig = nan(n,1);
outState = nan(n,1);
reqCount = nan(n,1);
status = nan(n,1);
cycleHard = nan(n,1);
callbackQPC = nan(n,1);
callbackTic = nan(n,1);
nativeInternal = nan(n,1);
ctxMs = nan(n,1);
valMs = nan(n,1);
elapsedBefore = nan(n,1);
budgetRemain = nan(n,1);

for k=1:n
    st=refState{k};
    refE2E(k)=v(st,'rt_light_e2e_ms');
    front(k)=v(st,'rt_spike_stage_front_ms');
    mass(k)=v(st,'rt_spike_stage_loadmass_ms');
    dep(k)=v(st,'rt_spike_stage_dependency_ms');
    baseGeom(k)=v(st,'rt_spike_stage_base_cert_geom_ms');
    sensorTrig(k)=v(st,'rt_spike_stage_sensor_trigger_ms');
    outState(k)=v(st,'rt_spike_stage_output_state_ms');
    reqCount(k)=v(st,'generator_request_count');
    status(k)=v(st,'generator_last_status_code');
    cycleHard(k)=v(st,'generator_last_cycle_critical_ms');
    callbackQPC(k)=v(st,'r9_hr_callback_wall_ms');
    callbackTic(k)=v(st,'generator_last_callback_ms');
    nativeInternal(k)=v(st,'r9_stage_total_internal_ms');
    ctxMs(k)=v(st,'generator_last_context_ms');
    valMs(k)=v(st,'generator_last_validation_ms');
    elapsedBefore(k)=v(st,'generator_last_elapsed_before_ms');
    budgetRemain(k)=v(st,'generator_last_budget_remaining_ms');
end

% runtime_profile is authoritative for logger/full-iteration because logger
% cannot measure itself without ambiguity.
onlineProf=interp_profile(P.t,1e3*P.online_before_plant_s,t);
plantProf=interp_profile(P.t,1e3*P.simulation_plant_s,t);
loggerMs=interp_profile(P.t,1e3*P.logger_s,t);
fullMs=interp_profile(P.t,1e3*P.full_iteration_s,t);
sensorAcq=interp_profile(P.t,1e3*P.sensor_acquisition_s,t);

% Prefer runtime_profile for the outer boundaries.
online=onlineProf;
plant=plantProf;
bad=~isfinite(online); online(bad)=onlineLog(bad);
bad=~isfinite(plant); plant(bad)=plantLog(bad);

onlineMinusRef=online-refE2E;
fullResidual=fullMs-online-plant-loggerMs;

% Detailed stage sum exists only on cycles captured by the existing light
% spike audit.  No interpolation/fabrication.
stageSum=front+mass+dep+baseGeom+sensorTrig+outState;
refUnattributed=refE2E-stageSum;

callbackWrapper=callbackQPC-nativeInternal;
sensorTrigMinusCallback=sensorTrig-callbackQPC;

% Request edges: request_count increments at the actual invoke call.
requestEdge=false(n,1);
prev=NaN;
for k=1:n
    if isfinite(reqCount(k))
        if ~isfinite(prev)
            prev=reqCount(k);
        elseif reqCount(k)>prev
            requestEdge(k)=true;
            prev=reqCount(k);
        end
    end
end
% If logger begins after count=0 and first visible value is already 1, catch it.
firstFinite=find(isfinite(reqCount),1,'first');
if ~isempty(firstFinite) && reqCount(firstFinite)>0
    requestEdge(firstFinite)=true;
end

% -------------------------------------------------------------------------
% Instrumentation status. These are existing run settings, not measurements.
% -------------------------------------------------------------------------
fprintf('\n[A. CURRENT IN-RUN DIAGNOSTIC SWITCHES]\n');
print_prop_bool(replanner,'ENABLE_PROFILING');
print_prop_bool(replanner,'ENABLE_LIGHT_RUNTIME_AUDIT');
print_prop_bool(replanner,'CAPTURE_REAL_TRIGGER_EVENT');
print_prop_bool(replanner,'ENABLE_OBSERVER_HARD_SUPPORT');
fprintf(' post-run T25/R28/T23/V47/V48 cost         = 0 ms inside recorded run (executed after simulation)\n');

% -------------------------------------------------------------------------
% Windows: pre-trigger baseline, request neighborhood, steady repaired.
% They are descriptive only, never safety authority.
% -------------------------------------------------------------------------
W = struct();
W.baseline = (t>=8.50 & t<9.75);
W.request = (t>=9.80 & t<=10.30);
W.post = (t>=10.75 & t<=14.00);

fprintf('\n[B. WINDOW DECOMPOSITION]\n');
print_window('PRE-TRIGGER BASELINE 8.50--9.75 s',W.baseline);
print_window('REQUEST WINDOW 9.80--10.30 s',W.request);
print_window('POST-REPLAN STEADY 10.75--14.00 s',W.post);

% -------------------------------------------------------------------------
% Actual request edges.
% -------------------------------------------------------------------------
ri=find(requestEdge);
fprintf('\n[C. TRUE GENERATOR REQUEST EDGES]\n');
fprintf([' # req  t[s] stat online refE2E nonRef sensorTrig cbQPC native wrapper hard ' ...
         'plant logger full [ms]\n']);
for q=1:numel(ri)
    k=ri(q);
    fprintf('%2d %3.0f %6.3f %4.0f %6.2f %6.2f %6.2f %9.2f %6.3f %6.3f %7.3f %6.3f %5.2f %6.2f %6.2f\n', ...
        q,reqCount(k),t(k),status(k),online(k),refE2E(k),onlineMinusRef(k), ...
        sensorTrig(k),callbackQPC(k),nativeInternal(k),callbackWrapper(k), ...
        cycleHard(k),plant(k),loggerMs(k),fullMs(k));
end

% -------------------------------------------------------------------------
% Excess-over-baseline attribution. This is useful for answering whether the
% 30--50 ms spike is reference-side or elsewhere.
% -------------------------------------------------------------------------
fprintf('\n[D. EXCESS ABOVE PRE-TRIGGER MEDIAN AT REQUEST EDGES]\n');
channels = { ...
    'reference/replanner e2e',refE2E; ...
    'non-reference online',onlineMinusRef; ...
    'numerical plant',plant; ...
    'logger',loggerMs; ...
    'full residual',fullResidual; ...
    'sensor/trigger/replan stage',sensorTrig; ...
    'output/state stage',outState; ...
    'dependency stage',dep; ...
    'base/cert/geometry stage',baseGeom};

baseMed=nan(size(channels,1),1);
for j=1:size(channels,1)
    z=channels{j,2}(W.baseline);
    z=z(isfinite(z));
    if ~isempty(z), baseMed(j)=median(z); end
end

fprintf(' #  t[s]  onlineExcess  dominantMeasuredExcess                  excess [ms]\n');
for q=1:numel(ri)
    k=ri(q);
    ex=nan(size(channels,1),1);
    for j=1:size(channels,1)
        if isfinite(channels{j,2}(k)) && isfinite(baseMed(j))
            ex(j)=channels{j,2}(k)-baseMed(j);
        end
    end

    % Do not choose overlapping child and parent channels together for the
    % causal label. Prefer broad disjoint decomposition first.
    broad = [1 2 3 4 5];
    eb=ex(broad);
    eb(~isfinite(eb))=-Inf;
    [mx,jj]=max(eb);
    if isfinite(mx)
        nm=channels{broad(jj),1};
    else
        nm='UNRESOLVED';
        mx=NaN;
    end
    baselineOnline=median_finite(online(W.baseline));
    fprintf('%2d %6.3f %12.3f  %-38s %9.3f\n', ...
        q,t(k),online(k)-baselineOnline,nm,mx);
end

% -------------------------------------------------------------------------
% Fine reference-side breakdown for captured request/spike cycles.
% -------------------------------------------------------------------------
fprintf('\n[E. REFERENCE-SIDE FINE BREAKDOWN WHERE EXISTING STAGE DATA EXISTS]\n');
fprintf([' t[s] front mass dep baseGeom sensorTrig outState stageSum refE2E unattrib ' ...
         'cbQPC native cbWrap trigMinusCB [ms]\n']);
fine=find(W.request & isfinite(stageSum));
for kk=fine(:).'
    fprintf('%6.3f %5.2f %5.2f %5.2f %8.2f %10.2f %8.2f %8.2f %6.2f %8.2f %6.3f %6.3f %6.3f %11.3f\n', ...
        t(kk),front(kk),mass(kk),dep(kk),baseGeom(kk),sensorTrig(kk),outState(kk), ...
        stageSum(kk),refE2E(kk),refUnattributed(kk),callbackQPC(kk),nativeInternal(kk), ...
        callbackWrapper(kk),sensorTrigMinusCallback(kk));
end

% -------------------------------------------------------------------------
% Classification.
% -------------------------------------------------------------------------
fprintf('\n[F. CLASSIFICATION]\n');
bOnline=median_finite(online(W.baseline));
rOnline=max_finite(online(W.request));
bRef=median_finite(refE2E(W.baseline));
rRef=max_finite(refE2E(W.request));
bOther=median_finite(onlineMinusRef(W.baseline));
rOther=max_finite(onlineMinusRef(W.request));
bPlant=median_finite(plant(W.baseline));
rPlant=max_finite(plant(W.request));

fprintf(' baseline online median [ms]              = %.6g\n',bOnline);
fprintf(' request-window online max [ms]           = %.6g\n',rOnline);
fprintf(' reference e2e baseline med / req max     = %.6g / %.6g ms\n',bRef,rRef);
fprintf(' non-reference online base med / req max  = %.6g / %.6g ms\n',bOther,rOther);
fprintf(' plant base med / request max             = %.6g / %.6g ms\n',bPlant,rPlant);
fprintf(' request-window online >=25 ms cycles     = %d\n',nnz(W.request & online>=25));
fprintf(' request-window plant >=10 ms cycles      = %d\n',nnz(W.request & plant>=10));

if isfinite(rRef)&&isfinite(bRef)&&isfinite(rOther)&&isfinite(bOther)
    dRef=rRef-bRef;
    dOther=rOther-bOther;
    if dRef > dOther
        cls='REFERENCE/REPLANNER SIDE DOMINATES THE REQUEST-WINDOW EXCESS';
    else
        cls='NON-REFERENCE ONLINE SIDE IS COMPARABLE OR DOMINANT';
    end
else
    cls='INSUFFICIENT EXISTING CHANNELS';
end
fprintf(' primary current-run attribution          = %s\n',cls);

% Existing profiling overhead conclusion.
prof=get_prop_num(replanner,'ENABLE_PROFILING');
light=get_prop_num(replanner,'ENABLE_LIGHT_RUNTIME_AUDIT');
capture=get_prop_num(replanner,'CAPTURE_REAL_TRIGGER_EVENT');
fprintf(' heavy in-class profiler enabled          = %d\n',logical(prof));
fprintf(' light runtime audit enabled              = %d\n',logical(light));
fprintf(' true-trigger replay capture enabled      = %d\n',logical(capture));
if ~logical(prof)
    fprintf(' heavy multi-stage profiling as root cause= EXCLUDED BY SWITCH\n');
end
if logical(light)||logical(capture)
    fprintf(' remaining diagnostic overhead            = POSSIBLE; quantify only by a clean A/B run, not by subtraction guess\n');
end
fprintf('====================================================================================================\n');

OUT=struct();
OUT.schema='AF7R50T_NO_NEW_TIMERS_TIMING_FORENSIC_V1';
OUT.t=t;
OUT.request_edge=requestEdge;
OUT.online_ms=online;
OUT.reference_e2e_ms=refE2E;
OUT.nonreference_online_ms=onlineMinusRef;
OUT.plant_ms=plant;
OUT.logger_ms=loggerMs;
OUT.full_ms=fullMs;
OUT.full_residual_ms=fullResidual;
OUT.sensor_trigger_ms=sensorTrig;
OUT.callback_qpc_ms=callbackQPC;
OUT.native_internal_ms=nativeInternal;
OUT.callback_wrapper_ms=callbackWrapper;
OUT.stage_sum_ms=stageSum;
OUT.reference_unattributed_ms=refUnattributed;
OUT.classification=cls;
assignin('base','AF7R50T_TIMING_FORENSIC_OUT',OUT);

% ---------------- nested helpers ----------------
    function print_window(name,mask)
        fprintf('  %s\n',name);
        print_stats('online before plant',online(mask));
        print_stats('reference/replanner e2e',refE2E(mask));
        print_stats('non-reference online residual',onlineMinusRef(mask));
        print_stats('numerical plant',plant(mask));
        print_stats('logger',loggerMs(mask));
        print_stats('full iteration',fullMs(mask));
        print_stats('full residual',fullResidual(mask));
        print_stats('sensor acquisition',sensorAcq(mask));
        print_stats('sensor/trigger/replan',sensorTrig(mask));
        print_stats('dependency',dep(mask));
        print_stats('base/cert/geometry',baseGeom(mask));
        print_stats('output/state',outState(mask));
    end
end


function P=normalize_profile(P)
req={'t','online_before_plant_s','simulation_plant_s','logger_s','full_iteration_s','sensor_acquisition_s'};
for i=1:numel(req)
    assert(isfield(P,req{i}),'[AF7R50T] runtime_profile.%s missing.',req{i});
end
if isfield(P,'k') && isfinite(double(P.k))
    K=min(double(P.k),numel(P.t));
else
    K=numel(P.t);
end
F=fieldnames(P);
for i=1:numel(F)
    x=P.(F{i});
    if isnumeric(x)||islogical(x)||isstring(x)||ischar(x)
        if ~isscalar(x) && numel(x)>=K
            try, P.(F{i})=x(1:K); catch, end
        end
    end
end
P.t=double(P.t(:));
end


function y=interp_profile(tp,xp,t)
tp=double(tp(:)); xp=double(xp(:)); t=double(t(:));
y=nan(size(t));
if isempty(tp)||isempty(xp),return;end
for k=1:numel(t)
    [d,j]=min(abs(tp-t(k)));
    tol=1e-9*max(1,abs(t(k)))+1e-12;
    if d<=tol && j<=numel(xp)
        y(k)=xp(j);
    end
end
end


function [t,refState,online,plant]=extract_logger(logger)
D=logger.Data; A=D.agent; if numel(A)>1,A=A(1);end
assert(isfield(D,'t')&&~isempty(D.t),'[AF7R50T] logger.Data.t missing.');
t=double(D.t(:)); n=numel(t);
Cr=get_result_cells(A,'reference');
Cc=get_result_cells(A,'controller');
refState=cell(n,1); online=nan(n,1); plant=nan(n,1);
for k=1:n
    if k<=numel(Cr), refState{k}=get_member(Cr{k},'state'); else, refState{k}=struct(); end
    if k<=numel(Cc)
        online(k)=1e3*scalar(get_member(Cc{k},'online_compute_total'));
        plant(k)=1e3*scalar(get_member(Cc{k},'simulation_plant_total'));
    end
end
end


function C=get_result_cells(A,name)
C={};
if ~isstruct(A)||~isfield(A,name),return;end
S=A.(name);
if ~isstruct(S)||~isfield(S,'result'),return;end
R=S.result;
if iscell(R),C=R(:);else,C=num2cell(R(:));end
end


function x=get_member(a,n)
x=[];
try
    if isstruct(a)&&isfield(a,n),x=a.(n);
    elseif isobject(a)&&isprop(a,n),x=a.(n);
    end
catch
end
end


function y=v(st,name)
y=scalar(get_member(st,name));
end


function y=scalar(x)
y=NaN;
if (isnumeric(x)||islogical(x))&&isscalar(x)&&isfinite(double(x))
    y=double(x);
end
end


function print_stats(name,x)
x=double(x(:));x=x(isfinite(x));
if isempty(x)
    fprintf('    %-34s = NaN\n',name);
else
    fprintf('    %-34s min/med/max = %.6g / %.6g / %.6g ms\n', ...
        name,min(x),median(x),max(x));
end
end


function y=median_finite(x)
x=double(x(:));x=x(isfinite(x));
if isempty(x),y=NaN;else,y=median(x);end
end


function y=max_finite(x)
x=double(x(:));x=x(isfinite(x));
if isempty(x),y=NaN;else,y=max(x);end
end


function print_prop_bool(obj,name)
v=get_prop_num(obj,name);
fprintf(' %-39s = %d\n',name,logical(v));
end


function v=get_prop_num(obj,name)
v=NaN;
try
    if isprop(obj,name)
        x=obj.(name);
        if (isnumeric(x)||islogical(x))&&isscalar(x)
            v=double(x);
        end
    end
catch
end
end

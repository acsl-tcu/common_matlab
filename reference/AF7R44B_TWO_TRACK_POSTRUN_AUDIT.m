function OUT = AF7R44B_TWO_TRACK_POSTRUN_AUDIT(logger,replanner)
clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R44B | TWO-TRACK POST-RUN AUDIT: SENSOR MEMORY + REPLANNER LAYERS\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] readOnly=1 | diagnosticOnly=1 | safetyLogicUnchanged=1\n');

assert(isa(replanner,'REPLANNING_SYSTEM'));
assert(~isempty(logger) && isprop(logger,'Data'),'[AF7R44B] logger.Data unavailable.');

%% A. Current configuration / actual execution-aware status.
A=replanner.get_avoidance_mode_status();
O=replanner.get_observer_hard_support_status();
M=replanner.get_simulated_sensor_memory_status();

fprintf('\n[A. CURRENT SAFETY-LAYER EXECUTION]\n');
fprintf(' commit policy                            = %s\n',char(A.commit_policy));
fprintf(' reference certificate switch            = %d\n',logical(A.reference_only_evaluation_enabled));
fprintf(' execution-aware certificate switch      = %d\n',logical(A.execution_aware_evaluation_enabled));
fprintf(' reference certificate evaluations       = %.0f\n',double(A.reference_only_evaluation_count));
fprintf(' execution-aware evaluations             = %.0f\n',double(A.execution_aware_evaluation_count));
fprintf(' reference-only commits                  = %.0f\n',double(A.reference_only_commit_count));
fprintf(' execution-aware commits                 = %.0f\n',double(A.execution_aware_commit_count));
fprintf(' observer hard-support enabled           = %d\n',logical(O.enabled));
fprintf(' observer updates                        = %.0f\n',double(O.update_count));
fprintf(' observer runtime last/mean/max [ms]     = %.6g / %.6g / %.6g\n', ...
    double(O.runtime_last_ms),double(O.runtime_mean_ms),double(O.runtime_max_ms));

if strcmpi(char(A.commit_policy),'REFERENCE_ONLY')
    fprintf(' execution-aware hot-path evaluation     = DEFERRED_BY_REFERENCE_ONLY_POLICY\n');
else
    fprintf(' execution-aware hot-path evaluation     = POLICY_ACTIVE\n');
end

%% B. Logged states.
D=logger.Data;
AR=D.agent;
if numel(AR)>1, AR=AR(1); end

Cref=get_result_cells(AR,'reference');
Cctl=get_result_cells(AR,'controller');

if isfield(D,'t') && ~isempty(D.t)
    t=double(D.t(:));
else
    t=(0:max(numel(Cref),numel(Cctl))-1)'*0.025;
end

n=min([numel(t),numel(Cref),numel(Cctl)]);
t=t(1:n);

currentCount=nan(n,1);
knownCount=nan(n,1);
status=nan(n,1);
hardMs=nan(n,1);
refE2E=nan(n,1);
observerMs=nan(n,1);
depMs=nan(n,1);
baseMs=nan(n,1);
sensorTrigMs=nan(n,1);
outMs=nan(n,1);
onlineMs=nan(n,1);
releaseCount=nan(n,1);

currentMasks=cell(n,1);
knownMasks=cell(n,1);

for k=1:n
    sr=get_member(Cref{k},'state');
    currentCount(k)=scalar_or_nan(get_member(sr,'detected_obstacle_count_point'));
    knownCount(k)=scalar_or_nan(get_member(sr,'sensor_known_obstacle_count'));
    releaseCount(k)=scalar_or_nan(get_member(sr,'sensor_memory_release_count'));
    status(k)=scalar_or_nan(get_member(sr,'generator_last_status_code'));
    hardMs(k)=scalar_or_nan(get_member(sr,'generator_last_cycle_critical_ms'));
    refE2E(k)=scalar_or_nan(get_member(sr,'rt_light_e2e_ms'));
    observerMs(k)=scalar_or_nan(get_member(sr,'rt_observer_ms'));
    depMs(k)=scalar_or_nan(get_member(sr,'rt_spike_stage_dependency_ms'));
    baseMs(k)=scalar_or_nan(get_member(sr,'rt_spike_stage_base_cert_geom_ms'));
    sensorTrigMs(k)=scalar_or_nan(get_member(sr,'rt_spike_stage_sensor_trigger_ms'));
    outMs(k)=scalar_or_nan(get_member(sr,'rt_spike_stage_output_state_ms'));
    currentMasks{k}=row_logical(get_member(sr,'sensor_current_active_mask'));
    knownMasks{k}=row_logical(get_member(sr,'sensor_known_active_mask'));

    rc=Cctl{k};
    onlineMs(k)=1e3*scalar_or_nan(get_member(rc,'online_compute_total'));
end

%% C. Sensor-memory contract.
firstCurrent=first_true_time(t,currentMasks);
firstKnown=first_true_time(t,knownMasks);
unknownBeforeDetect = isnan(firstKnown) || ...
    (~isnan(firstCurrent) && firstKnown>=firstCurrent-1e-12);

memoryViolation=false;
prevKnown=[];
prevRelease=0;
for k=1:n
    km=knownMasks{k};
    if isempty(km), continue; end
    rel=releaseCount(k);
    if isempty(prevKnown)
        prevKnown=km;
        if isfinite(rel), prevRelease=rel; end
        continue;
    end
    if numel(km)==numel(prevKnown)
        lost=prevKnown & ~km;
        releaseAdvanced=isfinite(rel) && rel>prevRelease;
        if any(lost) && ~releaseAdvanced
            memoryViolation=true;
        end
    end
    prevKnown=km;
    if isfinite(rel), prevRelease=rel; end
end

fprintf('\n[B. SIMULATED SENSOR MEMORY]\n');
fprintf(' initial known obstacle count             = 0 (source contract)\n');
fprintf(' first current sensor detection [s]       = %.6g\n',firstCurrent);
fprintf(' first planner-known obstacle [s]         = %.6g\n',firstKnown);
fprintf(' no knowledge before first detection      = %d\n',unknownBeforeDetect);
fprintf(' final known obstacle count               = %.0f\n',double(M.known_count));
fprintf(' memory release count                     = %.0f\n',double(M.release_count));
fprintf(' retained-mask loss without release       = %d\n',memoryViolation);
fprintf(' release policy                           = %s\n',char(M.release_policy));

%% D. 10-second window: all current replanner layers.
win=(t>=9.75 & t<=10.30);
idx=find(win & (isfinite(onlineMs) | isfinite(refE2E)));

fprintf('\n[C. 9.75--10.30 s FULL ONLINE / REPLANNER WINDOW]\n');
fprintf(' #   t[s] online refE2E observer dep  baseGeom sensorTrig outState stat hard curObs knownObs [ms]\n');
for q=1:numel(idx)
    k=idx(q);
    fprintf('%2d %6.3f %6.2f %6.2f %7.3f %5.2f %8.2f %9.2f %8.2f %4.0f %5.2f %6.0f %8.0f\n', ...
        q,t(k),onlineMs(k),refE2E(k),observerMs(k),depMs(k),baseMs(k), ...
        sensorTrigMs(k),outMs(k),status(k),hardMs(k),currentCount(k),knownCount(k));
end

requestWindow=idx(isfinite(status(idx)) & status(idx)~=0);

fprintf('\n[D. REQUEST-WINDOW SUMMARY]\n');
print_summary('online before plant',onlineMs(requestWindow));
print_summary('reference/replanner e2e',refE2E(requestWindow));
print_summary('observer hard support',observerMs(requestWindow));
print_summary('dependency stage',depMs(requestWindow));
print_summary('base/cert/geometry',baseMs(requestWindow));
print_summary('sensor/trigger/replan',sensorTrigMs(requestWindow));
print_summary('output/state',outMs(requestWindow));
print_summary('generator hard critical',hardMs(requestWindow));

%% E. Classification of the "real-machine consideration is ON?" question.
execActuallyEvaluated=double(A.execution_aware_evaluation_count)>0;
observerActuallyRunning=logical(O.enabled)&&double(O.update_count)>0;

fprintf('\n[E. CLASSIFICATION]\n');
fprintf(' observer hard-support actually running   = %d\n',observerActuallyRunning);
fprintf(' full execution-aware cert actually run   = %d\n',execActuallyEvaluated);
fprintf(' full execution-aware commit authority    = %d\n', ...
    strcmpi(char(A.commit_policy),'EXECUTION_AWARE'));
fprintf(' sensor memory contract closed            = %d\n', ...
    unknownBeforeDetect && ~memoryViolation);
fprintf('====================================================================================================\n');

OUT=struct();
OUT.schema='AF7R44B_TWO_TRACK_POSTRUN_AUDIT_V1';
OUT.sensor_memory_closed=unknownBeforeDetect && ~memoryViolation;
OUT.first_current_detection_s=firstCurrent;
OUT.first_known_s=firstKnown;
OUT.avoidance=A;
OUT.observer=O;
OUT.memory=M;
OUT.execution_aware_actually_evaluated=execActuallyEvaluated;
OUT.observer_actually_running=observerActuallyRunning;
OUT.window_index=idx;
assignin('base','AF7R44B_TWO_TRACK_POSTRUN_AUDIT_OUT',OUT);
end


function C=get_result_cells(A,name)
C={};
if ~isstruct(A) || ~isfield(A,name), return; end
S=A.(name);
if ~isstruct(S) || ~isfield(S,'result'), return; end
R=S.result;
if iscell(R), C=R(:); else, C=num2cell(R(:)); end
end

function v=get_member(x,name)
v=[];
try
    if isstruct(x) && isfield(x,name)
        v=x.(name);
    elseif isobject(x) && isprop(x,name)
        v=x.(name);
    end
catch
    v=[];
end
end

function y=scalar_or_nan(x)
y=NaN;
if isnumeric(x)||islogical(x)
    if isscalar(x) && isfinite(double(x))
        y=double(x);
    end
end
end

function y=row_logical(x)
y=[];
if isnumeric(x)||islogical(x)
    y=logical(x(:).');
end
end

function tf=first_true_time(t,C)
tf=NaN;
for k=1:min(numel(t),numel(C))
    if ~isempty(C{k}) && any(C{k})
        tf=double(t(k));
        return;
    end
end
end

function print_summary(name,v)
v=double(v(:));
v=v(isfinite(v));
if isempty(v)
    fprintf(' %-37s = NaN\n',name);
else
    fprintf(' %-37s min/med/max = %.6g / %.6g / %.6g ms\n', ...
        name,min(v),median(v),max(v));
end
end

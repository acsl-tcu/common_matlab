function R = CHECK_TRIGGER_FLIGHT_FINAL(log_source,suppress_single_audit_zip)
% CHECK_TRIGGER_FLIGHT_FINAL  Post-Flight, READ-ONLY audit of the live trigger.
% Run after run_flight_headless in the SAME MATLAB session. No HLC, plant,
% sensor, generator, trajectory, runner or Flight-loop modification.
% No synthetic predictor is substituted: the actual Sim LOGGER is inspected.
% Reports first possible contact interval in the directional sweep.
% The redundant sampled point oracle is intentionally omitted in Flight.
% The sweep is NOT a true 24-state continuous-time safety certificate,
% nor a deterministic worst-case execution-time guarantee.
%
%   R = CHECK_TRIGGER_FLIGHT_FINAL();        % use base-workspace logger
%   R = CHECK_TRIGGER_FLIGHT_FINAL(logger);  % explicit logger
%
% Results and the SINGLE shareable audit ZIP are placed in common_matlab/.
root = locate_common_matlab_root(fileparts(mfilename('fullpath')));
if nargin<2,suppress_single_audit_zip=false;end
assert(isscalar(suppress_single_audit_zip) && ...
    (islogical(suppress_single_audit_zip) || isnumeric(suppress_single_audit_zip)), ...
    'CHECK_TRIGGER_FLIGHT_FINAL:ArchiveFlag','Invalid audit archive flag.');
if nargin < 1
    assert(evalin('base','exist(''logger'',''var'')') == 1, ...
        'CHECK_TRIGGER_FLIGHT_FINAL:MissingLogger', ...
        'Run run_flight_headless first, then CHECK_TRIGGER_FLIGHT_FINAL.');
    log_source = evalin('base','logger');
end
if isobject(log_source) && isprop(log_source,'Data')
    data = log_source.Data;
elseif isstruct(log_source) && isfield(log_source,'Data')
    data = log_source.Data;
else
    data = log_source;
end
assert(isstruct(data) && isscalar(data) && isfield(data,'agent') && ...
    isfield(data,'t'), 'CHECK_TRIGGER_FLIGHT_FINAL:LoggerShape', ...
    'Expected Sim LOGGER.Data with .agent and .t.');
A = data.agent;
assert(isstruct(A) && ~isempty(A) && isfield(A,'reference'), ...
    'CHECK_TRIGGER_FLIGHT_FINAL:ReferenceLog', ...
    'Logger contains no agent.reference results.');
ref = A(1).reference;
assert(isstruct(ref) && isfield(ref,'result'), ...
    'CHECK_TRIGGER_FLIGHT_FINAL:ReferenceLog', ...
    'Logger contains no reference.result history.');
if iscell(ref.result)
    frames = ref.result(:);
else
    frames = num2cell(ref.result(:));
end
log_times = double(data.t(:));
N = min(numel(log_times),numel(frames));
assert(N > 0,'CHECK_TRIGGER_FLIGHT_FINAL:Empty','LOGGER contains no frames.');

% Every observation below is read-only; no call to update(), evaluate(),
% the true-obstacle source, the native predictor or the real sensor.
T = nan(N,1); wall_ms = nan(N,1); full_ms = nan(N,1);
% Record already-logged runtime component timings; NEVER run a predictor or
% geometric oracle in the auditor. NaN marks an unavailable component.
phase_names = {'predictor_wall_ms', 'predictor_reference_window_ms', ...
    'predictor_hlc_time_ms', 'predictor_plant_time_ms', ...
    'nominal_interpolated_sweep_ms', 'geometry_wall_ms', ...
    'timing_pre_predictor_ms', 'timing_post_geometry_ms', ...
    'reference_cache_wall_ms'};
phase_ms = nan(N,numel(phase_names));
prediction_horizon_s = nan(N,1); sampling_step_s = nan(N,1);
n_observed = nan(N,1);
available = false(N,1); complete = false(N,1); early = false(N,1); fallback = false(N,1);
contact = false(N,1); possible = false(N,1);
newly_issued = false(N,1); native = false(N,1);
ids_known = cell(N,1); ids_event = cell(N,1);
status = cell(N,1); issue = cell(0,1);
count_logged = 0; count_known_consistency = 0;
last_revision = uint64(0);
issued_keys = cell(0,1);
event_rows = zeros(0,5); % log_idx, time_s, obstacle_id, ttc_s, revision
max_issue_lines = 80;
for k = 1:N
    frame = frames{k};
    if ~isstruct(frame) || ~isscalar(frame)
        continue
    end
    tr = getfield_or(frame,'future_collision_trigger',[]);
    if ~isstruct(tr) || ~isscalar(tr)
        continue
    end
    count_logged = count_logged+1;
    t = getfield_or(tr,'time_s',log_times(k));
    T(k) = numeric_scalar(t);
    status{k} = char(string(getfield_or(tr,'status','UNKNOWN')));
    wall_ms(k) = numeric_scalar(getfield_or(tr,'elapsed_ms',NaN));
    prediction_horizon_s(k) = numeric_scalar(getfield_or(tr,'horizon_s',NaN));
    sampling_step_s(k) = numeric_scalar(getfield_or(tr,'sample_dt_s',NaN));
    n_observed(k) = numeric_scalar(getfield_or(tr,'considered_count',NaN));
    for ci = 1:numel(phase_names)
        phase_ms(k,ci) = numeric_scalar(getfield_or(tr,phase_names{ci},NaN));
    end
    full_ms(k) = numeric_scalar(getfield_or(frame,'replanning_time_ms',NaN));
    available(k) = is_true(getfield_or(tr,'prediction_available',false));
    complete(k) = is_true(getfield_or(tr,'prediction_complete',false));
    early(k) = is_true(getfield_or(tr,'prediction_early_stopped',false));
    fallback(k) = is_true(getfield_or(tr,'prefix_probe_retried_full',false));
    contact(k) = is_true(getfield_or(tr,'sampled_collision',false));
    possible(k) = is_true(getfield_or(tr,'collision_possible_intersample',false));
    newly_issued(k) = is_true(getfield_or(tr,'new_event',false));
    native(k) = strcmp(char(string(getfield_or(tr,'predictor_dispatch',''))), ...
                       'DIRECT_NATIVE_VERIFIED_BINDING');
    known = reshape(double(getfield_or(tr,'considered_obstacle_ids',zeros(1,0))),1,[]);
    eids = reshape(double(getfield_or(tr,'new_event_ids',zeros(1,0))),1,[]);
    ids_known{k} = known; ids_event{k} = eids;
    % A failed prediction over a nonempty observed set must not quietly
    % count as successful collision clearance or functional Flight PASS.
    if ~isempty(known) && startsWith(status{k},'UNKNOWN_') && ...
            ~any(strcmp(status{k},{'UNKNOWN_NONFLIGHT_PHASE', ...
                    'UNKNOWN_FUTURE_PHASE_CHANGE_UNMODELLED'}))
        add_issue(k,'nonempty observed set with unresolved collision prediction');
    end
    if ~isempty(known) && (numel(unique(known)) ~= numel(known) || any(~isfinite(known)))
        add_issue(k,'non-unique/nonfinite known IDs');
    end
    if any(~ismember(eids,known)) || numel(unique(eids))~=numel(eids)
        add_issue(k,'new-event ID absent from currently observed-memory set');
    end
    if available(k) && ~native(k)
        add_issue(k,'prediction is not DIRECT_NATIVE_VERIFIED_BINDING');
    end
    if available(k)
        if ~(isfinite(prediction_horizon_s(k)) && prediction_horizon_s(k)>0 && ...
                isfinite(sampling_step_s(k)) && sampling_step_s(k)>0)
            add_issue(k,'invalid live prediction horizon/sample interval');
        end
        if ~is_true(getfield_or(tr,'runtime_authority_refreshed',false))
            add_issue(k,'live external runtime parameter snapshot was not refreshed');
        end
        if ~is_true(getfield_or(tr,'nominal_interpolated_sweep_checked',false))
            add_issue(k,'directional continuous nominal sweep not executed');
        end
        ht = numeric_scalar(getfield_or(tr,'completed_prediction_horizon_s',NaN));
        H = numeric_scalar(getfield_or(tr,'horizon_s',NaN));
        if complete(k) && ~(isfinite(ht) && isfinite(H) && ...
                abs(ht-H) <= 1e-9*max(1,abs(H)))
            add_issue(k,'complete prediction did not reach its full horizon');
        end
        if early(k) && (complete(k) || ~(contact(k)||possible(k)) || ...
                ~(isfinite(ht) && isfinite(H) && ht < H))
            add_issue(k,'early-stop output is inconsistent');
        end
        if ~complete(k) && ~early(k)
            add_issue(k,'available numerical prediction neither complete nor confirmed early-stop');
        end
        if fallback(k) && (~complete(k) || early(k))
            add_issue(k,'short-prefix miss did not return a full-horizon prediction');
        end
        if startsWith(status{k},'UNKNOWN_')
            add_issue(k,'available numeric prediction returned an unresolved trigger status');
        end
    end
    if contact(k)||possible(k)||is_true(getfield_or(tr,'collision_risk_predicted',false))
        id = numeric_scalar(getfield_or(tr,'collision_obstacle_id',NaN));
        tc = numeric_scalar(getfield_or(tr,'collision_ttc_s',NaN));
        at = numeric_scalar(getfield_or(tr,'collision_time_s',NaN));
        H = numeric_scalar(getfield_or(tr,'horizon_s',NaN));
        if ~(isfinite(id) && ismember(id,known) && isfinite(tc) && ...
                tc>=0 && tc<=H+1e-8 && isfinite(at) && ...
                abs(at-(T(k)+tc))<=1e-8*max(1,abs(at)))
            add_issue(k,'risk ID/TTC/absolute time invalid');
        end
        bracket = double(getfield_or(tr,'collision_time_bracket_s',[NaN NaN]));
        if ~(isnumeric(bracket) && numel(bracket)==2 && ...
                all(isfinite(bracket)) && bracket(1)==tc && ...
                bracket(2)>=bracket(1) && bracket(2)<=H+1e-8)
            add_issue(k,'predicted earliest possible collision time bracket invalid');
        end
        chosen = reshape(double(getfield_or(tr,'collision_obstacle_ids',zeros(1,0))),1,[]);
        if isempty(chosen) || ~ismember(id,chosen) || any(~ismember(chosen,known))
            add_issue(k,'selected risk ID list is inconsistent');
        end
    elseif newly_issued(k)
        add_issue(k,'new_event without sampled hit or nominal intersample risk');
    end
    if newly_issued(k) ~= ~isempty(eids)
        add_issue(k,'new_event and new_event_ids disagree');
    end
    if ~isempty(eids) && ~(contact(k)||possible(k))
        add_issue(k,'new-event IDs emitted without nominal sampled/swept risk');
    end
    if is_true(getfield_or(tr,'safe_to_commit',false)) || ...
            is_true(getfield_or(tr,'continuous_observed_set_safe',false))
        add_issue(k,'nominal-only trigger falsely advertised a physical certificate');
    end
    if isfield(frame,'observed_obstacle_memory')
        memory = frame.observed_obstacle_memory;
        if isstruct(memory) && isscalar(memory) && isfield(memory,'known_obstacle_ids')
            count_known_consistency = count_known_consistency+1;
            m_ids = reshape(double(memory.known_obstacle_ids),1,[]);
            if ~isequal(known,m_ids)
                add_issue(k,'trigger accessed IDs not equal to current observed memory');
            end
        end
    end
    revision = getfield_or(tr,'plan_revision',uint64(0));
    if ~(isnumeric(revision) && isscalar(revision) && isfinite(revision) && ...
            revision >= 0 && double(revision) == fix(double(revision)))
        add_issue(k,'invalid committed-plan revision');
        revision = uint64(0);
    else
        revision = uint64(revision);
    end
    if revision < last_revision
        add_issue(k,'plan revision decreased during Flight');
    end
    last_revision = revision;
    for q = 1:numel(eids)
        key = sprintf('%.0f:%.17g',double(revision),eids(q));
        if ismember(key,issued_keys)
            add_issue(k,'duplicate event for same obstacle and committed plan');
        else
            issued_keys{end+1,1} = key; %#ok<AGROW>
        end
        event_rows(end+1,:) = [k,T(k),eids(q), ...
            numeric_scalar(getfield_or(tr,'collision_ttc_s',NaN)),double(revision)]; %#ok<AGROW>
    end
end

% A full nominal Flight must exercise the live prediction. Do not mark a
% solely synthetic/disabled run as a real Flight verification.
if count_logged == 0
    issue{end+1,1}='No future_collision_trigger frames in Sim LOGGER (LOG_TRIGGER must be ON).';
end
if ~any(available)
    issue{end+1,1}='No real numerical prediction was executed (preflight binding/gate/phase may be unavailable).';
end
if any(~isfinite(wall_ms(available))) || any(wall_ms(available)<0)
    issue{end+1,1}='Invalid recorded prediction execution time.';
end
if isempty(issue)
    base_status = 'PASS_REAL_FLIGHT_DIRECTIONAL_SWEEP_WITH_PHASE_GAP';
else
    base_status = 'FAIL_REAL_FLIGHT_TRIGGER_AUDIT';
end
v=wall_ms(isfinite(wall_ms));
w=full_ms(isfinite(full_ms));
R = struct('status',base_status, 'failures',numel(issue), ...
    'messages',{issue},'logged_frames',count_logged, ...
    'flight_frames',N,'predictions',nnz(available), ...
    'full_horizon_predictions',nnz(available & complete), ...
    'confirmed_early_stops',nnz(available & early), ...
    'prefix_misses_retried_full',nnz(fallback & complete), ...
    'native_predictions',nnz(available & native), ...
    'sampled_contact_frames',nnz(contact), ...
    'unmodelled_future_phase_frames',sum(strcmp(status, ...
        'UNKNOWN_FUTURE_PHASE_CHANGE_UNMODELLED')), ...
    'nonflight_phase_frames',sum(strcmp(status,'UNKNOWN_NONFLIGHT_PHASE')), ...
    'possible_intersample_frames',nnz(possible), ...
    'new_event_frames',nnz(newly_issued), ...
    'new_event_table',event_rows, ...
    'memory_id_cross_checks',count_known_consistency, ...
    'trigger_median_ms',median_or_nan(v), ...
    'trigger_p95_ms',percentile_or_nan(v,95), ...
    'trigger_max_ms',max_or_nan(v), ...
    'replanning_median_ms',median_or_nan(w), ...
    'replanning_p95_ms',percentile_or_nan(w,95), ...
    'replanning_max_ms',max_or_nan(w), ...
    'trigger_above_4ms_count',nnz(v>4), ...
    'sampled_horizon_min_s',min_or_nan(prediction_horizon_s(available)), ...
    'sampled_horizon_max_s',max_or_nan(prediction_horizon_s(available)), ...
    'sample_dt_min_s',min_or_nan(sampling_step_s(available)), ...
    'sample_dt_max_s',max_or_nan(sampling_step_s(available)), ...
    'replanning_above_4ms_count',nnz(w>4), ...
    'observed_run_below_4ms',~isempty(v) && all(v<=4), ...
    'wcet_proven',false, ...
    'continuous_time_uncertainty_safety_proven',false, ...
    'meaning',['F-phase-only numeric nonlinear predictor and directional ' ...
    'nominal linear sweep; no prediction across f-to-landing boundary, ' ...
    'no physical uncertainty/WCET certificate']);
save(fullfile(root,'TRIGGER_FLIGHT_FINAL_RESULT.mat'),'R','T','wall_ms', ...
    'full_ms','available','complete','early','contact','newly_issued', ...
    'native','fallback','possible','ids_known','ids_event','status', ...
    'phase_names','phase_ms','prediction_horizon_s','sampling_step_s', ...
    'n_observed','-v7');
txt = fullfile(root,'TRIGGER_FLIGHT_FINAL_RESULT.txt');
fid = fopen(txt,'w');
assert(fid>0,'CHECK_TRIGGER_FLIGHT_FINAL:Output','Cannot write audit text file.');
clean = onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s\n',R.status);
fprintf(fid,'frames=%d logged=%d native_predictions=%d full=%d early=%d fallback_full=%d\n', ...
    N,count_logged,R.native_predictions,R.full_horizon_predictions, ...
    R.confirmed_early_stops,R.prefix_misses_retried_full);
fprintf(fid,'contact_frames=%d possible_intersample_frames=%d new_event_frames=%d events=%d memory_cross_checks=%d failures=%d\n', ...
    R.sampled_contact_frames,R.possible_intersample_frames,R.new_event_frames,size(event_rows,1), ...
    count_known_consistency,R.failures);
fprintf(fid,'nonflight_frames=%d future_phase_crossing_unmodelled_frames=%d\n', ...
    R.nonflight_phase_frames,R.unmodelled_future_phase_frames);
fprintf(fid,'trigger_ms median=%.6f P95=%.6f max=%.6f over_4ms=%d\n', ...
    R.trigger_median_ms,R.trigger_p95_ms,R.trigger_max_ms,R.trigger_above_4ms_count);
fprintf(fid,'replanning_ms median=%.6f P95=%.6f max=%.6f over_4ms=%d\n', ...
    R.replanning_median_ms,R.replanning_p95_ms,R.replanning_max_ms, ...
    R.replanning_above_4ms_count);
fprintf(fid,'runtime_horizon_s min=%.9g max=%.9g | sample_dt_s min=%.9g max=%.9g\n', ...
    R.sampled_horizon_min_s,R.sampled_horizon_max_s,R.sample_dt_min_s,R.sample_dt_max_s);
fprintf(fid,'observed_run_below_4ms=%d WCET_proven=0 continuous_safety_proven=0\n', ...
    R.observed_run_below_4ms);
for j = 1:numel(issue)
    fprintf(fid,'ISSUE %d: %s\n',j,issue{j});
end
fprintf(fid,'=== NEW EVENT REPORT (frame t id ttc plan_revision) ===\n');
for j = 1:size(event_rows,1)
    fprintf(fid,'%d %.9g %.17g %.9g %.0f\n',event_rows(j,:));
end
clear clean

% Single common_matlab-root ZIP: results plus exact live-source snapshots.
% Never zip generated logs inside reference/ or change any production file.
share = {'TRIGGER_FLIGHT_FINAL_RESULT.mat','TRIGGER_FLIGHT_FINAL_RESULT.txt'};
optional = {'TRIGGER_NOMINAL_SWEEP_RESULT.mat','TRIGGER_NOMINAL_SWEEP_RESULT.txt', ...
    'TRIGGER_FINAL_RESULT.mat','TRIGGER_FINAL_RESULT.txt', ...
    'TRIGGER_COLLISION_ONLY_RESULT.txt','TRIGGER_EARLY_CONTACT_FINAL_RESULT.txt'};
for j = 1:numel(optional)
    if isfile(fullfile(root,optional{j})), share{end+1}=optional{j};end %#ok<AGROW>
end
source = {'reference/REPLANNING_SYSTEM/REPLANNING_SYSTEM.m', ...
    'reference/REPLANNING_SYSTEM/FUTURE_COLLISION_TRIGGER/FUTURE_COLLISION_TRIGGER.m', ...
    'reference/REPLANNING_SYSTEM/FUTURE_COLLISION_TRIGGER/TRIGGER_EARLIEST_SAMPLED_CONTACT.m', ...
    'reference/REPLANNING_SYSTEM/FUTURE_COLLISION_TRIGGER/TRIGGER_REFERENCE_WINDOW_CACHE.m', ...
    'reference/REPLANNING_SYSTEM/FUTURE_COLLISION_TRIGGER/TRIGGER_NOMINAL_SWEPT_INTERVALS.m', ...
    'reference/REPLANNING_SYSTEM/FUTURE_COLLISION_TRIGGER/TRIGGER_MERGE_EARLIEST_NOMINAL_RISK.m'};
for j = 1:numel(source)
    if isfile(fullfile(root,source{j})),share{end+1}=source{j};end %#ok<AGROW>
end
R.share_zip='';
if ~suppress_single_audit_zip
    archive=fullfile(root,'TRIGGER_FINAL_AUDIT_FOR_CHATGPT.zip');
    if isfile(archive),delete(archive);end
    zip(archive,share,root);
    R.share_zip=archive;
end
fprintf('[FINAL-TRIGGER] %s | native=%d early=%d events=%d failures=%d\n', ...
    R.status,R.native_predictions,R.confirmed_early_stops, ...
    size(event_rows,1),R.failures);
if ~isempty(R.share_zip)
    fprintf('[FINAL-TRIGGER] single audit ZIP: %s\n',R.share_zip);
end

    function add_issue(idx,msg)
        if numel(issue)<max_issue_lines
            issue{end+1,1}=sprintf('frame %d t=%.9g: %s',idx,log_times(idx),msg); %#ok<AGROW>
        end
    end
end

function out=getfield_or(s,field,default)
out=default;
if isstruct(s) && isscalar(s) && isfield(s,field)
    out=s.(field);
end
end
function val=numeric_scalar(x)
val=NaN;
if isnumeric(x) && isreal(x) && isscalar(x) && isfinite(x)
    val=double(x);
end
end
function t=is_true(x)
t=(islogical(x)||isnumeric(x)) && isscalar(x) && isfinite(double(x)) && isequal(x,true);
end
function out=min_or_nan(x)
x=x(isfinite(x));
if isempty(x),out=NaN;else,out=min(x);end
end
function out=max_or_nan(x)
if isempty(x),out=NaN;else,out=max(x);end
end
function out=median_or_nan(x)
if isempty(x),out=NaN;else,out=median(x);end
end
function out=percentile_or_nan(x,p)
if isempty(x),out=NaN;return;end
x=sort(x(:));
k=max(1,ceil(double(p)/100*numel(x)));
out=x(k);
end

function root = locate_common_matlab_root(start_dir)
% Output reports must go to common_matlab/ even when this file was
% accidentally extracted under reference/REPLANNING_SYSTEM/.
root = start_dir;
for depth = 0:8
    base = fullfile(root,'reference','REPLANNING_SYSTEM');
    if isfile(fullfile(base,'REPLANNING_SYSTEM.m')) && ...
            isfile(fullfile(base,'FUTURE_COLLISION_TRIGGER', ...
                                    'FUTURE_COLLISION_TRIGGER.m'))
        return
    end
    parent = fileparts(root);
    if strcmp(parent,root)
        break
    end
    root = parent;
end
error('CHECK_TRIGGER_FLIGHT_FINAL:RootNotFound', ...
    'Cannot locate common_matlab root above: %s',start_dir);
end

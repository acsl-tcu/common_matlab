% AF7R58R48R2_THREE_ISSUES_AUDIT.m
% Read-only, single-file, source-bound audit of THREE issues ONLY:
% (1) natural return / overshoot / persistent correction,
% (2) REPLANNER stage, packet and cycle time (not HLC/controller spikes),
% (3) sensed-current-state vs active-reference-future trigger mismatch.
%
% Place this file in the same directory as REPLANNING_SYSTEM.m, then run:
%     run('AF7R58R48R2_THREE_ISSUES_AUDIT.m')
% No uigetfile, no ZIP prompts, no runtime plant/controller mutation.
% Does not change production, trigger logic, safety certificates or commit.
% Missing files/status never convert to PASS. This is NOT a safety proof.

fprintf('\n[R48R2] Three-issue SOURCE-BOUND diagnostic (read-only)\n');
OUT = r48r2_run_three_issues();
assignin('base','AF7R58R48R2_OUT',OUT);
fprintf('[R48R2] RET    = %s\n',OUT.return.status);
fprintf('[R48R2] TIMING = %s\n',OUT.timing.status);
fprintf('[R48R2] TRIG   = %s\n',OUT.trigger.status);
fprintf('[R48R2] REPORT = %s\n',OUT.report_file);
fprintf('[R48R2] MAT    = %s\n',OUT.mat_file);

function OUT = r48r2_run_three_issues()
root=fileparts(mfilename('fullpath'));
if isempty(root),root=pwd;end
source=fullfile(root,'REPLANNING_SYSTEM.m');
if ~isfile(source)
    located=which('REPLANNING_SYSTEM');
    if ~isempty(located), source=located;end
end
OUT=struct();
OUT.schema='AF7R58R48R2_THREE_ISSUES_READ_ONLY_V1';
OUT.source_file=source;
OUT.production_modified=false;
OUT.sensor_trigger_modified=false;
OUT.reference_commit_modified=false;
OUT.controller_or_plant_modified=false;
OUT.controller_24s_spike_in_scope=false;
OUT.return=struct('status','NOT_RUN__R47_ACTUAL_RETURN_NOT_FOUND');
OUT.timing=struct('status','NOT_RUN__R47_SOURCE_TIMING_NOT_FOUND');
OUT.trigger=struct('status','NOT_RUN__PRODUCTION_SOURCE_NOT_FOUND');
OUT.control_period_25ms_proved=false;
OUT.position_C7_proved_by_this_script=false;
OUT.yaw_C6_proved_by_this_script=false;
OUT.whole_body_continuous_safety_proved_by_this_script=false;
OUT.closed_loop_actual_state_prediction_proved_by_this_script=false;
OUT.source_pairing_verified=false;
OUT.r47_file='';

% 1. Identify the EXACT on-disk production source, then inspect the
% authoritative call sites; never assume an older cached source is current.
if isfile(source)
    contents=fileread(source);
    sourceInfo=dir(source);
    OUT.trigger=r48r2_trigger_source(contents);
    OUT.trigger.source_size_bytes=sourceInfo.bytes;
    OUT.trigger.source_mtime=sourceInfo.date;
    OUT.trigger.source_sha256=r48r2_hash_file(source);
    OUT.trigger.source_file=source;
end

% 2. Load last R47 measurement if available. R47 is already an explicitly
% source-labelled numerical report, not a complete production certificate.
file=fullfile(root,'AF7R58R47_OUT.mat');
if ~isfile(file)
    listings=dir(fullfile(root,'AF7R58R47_R3_RESULTS_*','AF7R58R47_OUT.mat'));
    if ~isempty(listings)
        [~,j]=max([listings.datenum]);
        file=fullfile(listings(j).folder,listings(j).name);
    end
end
if isfile(file)
    try
        S=load(file,'OUT');
        assert(isfield(S,'OUT') && isstruct(S.OUT) && isscalar(S.OUT), ...
            '[R48R2] R47 file has no scalar OUT.');
        R=S.OUT;
        assert(isfield(R,'schema') && ...
            strcmp(char(R.schema),'AF7R58R47_STANDALONE_AUTO_V3'), ...
            '[R48R2] R47 source schema unexpected.');
        OUT.r47_file=file;
        OUT.source_pairing_verified=logical(R.source_pairing_verified);
        OUT.return=r48r2_return(R);
        OUT.timing=r48r2_replanner_timing(R);
    catch ME
        OUT.return=struct('status','NOT_RUN__R47_INPUT_ERROR', ...
            'error_identifier',char(ME.identifier),'error_message',char(ME.message));
        OUT.timing=struct('status','NOT_RUN__R47_INPUT_ERROR', ...
            'error_identifier',char(ME.identifier),'error_message',char(ME.message));
    end
end

% Independent evidence: print the presence of exact named candidate/workspace
% structures without guessing they were committed or used by the plant.
known={'R44R2','R45R1','C44R2','L45','runtime_profile','avoidance_status'};
observed={};
for k=1:numel(known)
    try
        if evalin('base',sprintf('exist(''%s'',''var'')==1',known{k}))
            observed{end+1}=known{k}; %#ok<AGROW>
        end
    catch
        % Metadata discovery must never interrupt production/diagnostic work.
    end
end
OUT.workspace_named_evidence_present=observed;
OUT.candidate_committed_by_this_script=false;
OUT.trigger_closed_loop_altered_by_this_script=false;

% Save all findings in a NEW folder; never overwrite source or earlier logs.
outdir=fullfile(root,['AF7R58R48R2_RESULTS_' datestr(now,'yyyymmdd_HHMMSS_FFF')]);
assert(~isfolder(outdir),'[R48R2] Diagnostic output directory exists.');
mkdir(outdir);
OUT.mat_file=fullfile(outdir,'AF7R58R48R2_OUT.mat');
OUT.report_file=fullfile(outdir,'AF7R58R48R2_REPORT.txt');
save(OUT.mat_file,'OUT');
r48r2_write_report(OUT);
end

function T = r48r2_trigger_source(source)
% These are exact source-call signatures, NOT a runtime execution trace.
T=struct();
T.is_source_static_audit=true;
T.live_estimator_used_for_sensing=false;
T.future_reference_used_for_collision=false;
T.stage1_start_from_reference_packet=false;
T.closed_loop_from_latest_estimated_state_verified=false;
T.production_patch_performed=false;
T.trigger_corrected=false;
T.source_matches_expected=false;
T.lines=struct();
lines=regexp(source,'\r\n|\r|\n','split');
checks={ ...
 'estimated_state_reader', 'get_current_physical_geometry(obj,xd_nom)'; ...
 'actual_sensor_gate',    'obj.raw_sensor_gate(pQ_cur,obj.static_obstacle_map)'; ...
 'reference_packet',      'obj.get_active_reference_packet(time,cha,varargin,xd_nom)'; ...
 'trigger_ref_inputs',    'obj.trigger_core(active,pL_ref,q_ref,time.t)'; ...
 'stage1_packet',         'obj.staged_reference_packet_at(ta,varargin_in)'; ...
 'stage1_pL',             'pL_a=double(pL_future(:,1))'; ...
 'stage1_q',              'q_a=double(q_future(:,1))'; ...
 'latest_estimated_p',    'pQ_cur=est_state.p(:)'; ...
 'latest_estimated_q',    'q_cur=est_state.pT(:)'; ...
 'mex_trigger',           'g26_future_trigger_kernel_mex( '};
for i=1:size(checks,1)
    name=checks{i,1}; needle=regexprep(checks{i,2},'\s+','');
    matches=[];
    for j=1:numel(lines)
        L=regexprep(lines{j},'\s+','');
        if contains(L,needle),matches(end+1)=j;end %#ok<AGROW>
    end
    T.lines.(name)=matches;
end
T.live_estimator_used_for_sensing=~isempty(T.lines.actual_sensor_gate) && ...
    ~isempty(T.lines.latest_estimated_p) && ~isempty(T.lines.latest_estimated_q);
T.future_reference_used_for_collision=~isempty(T.lines.reference_packet) && ...
    ~isempty(T.lines.trigger_ref_inputs);
T.stage1_start_from_reference_packet=~isempty(T.lines.stage1_packet) && ...
    ~isempty(T.lines.stage1_pL) && ~isempty(T.lines.stage1_q);
T.source_matches_expected=T.live_estimator_used_for_sensing && ...
    T.future_reference_used_for_collision && T.stage1_start_from_reference_packet;
if T.source_matches_expected
    T.status='SOURCE_CONFIRMED__SENSING_ESTIMATE_BUT_COLLISION_PREDICTS_REFERENCE';
else
    T.status='SOURCE_DIFFERENT__MANUAL_SEMANTIC_REVIEW_REQUIRED';
end
T.suggested_design= ...
    ['Keep current-state sensor gate and H=4 s. Use latest EKF x(t), q, ' ...
     'qdot, live mL, HLC, actuator and plant to predict whole-body motion; ' ...
     'attach a continuous-time reachable error tube. Preserve active reference ' ...
     'for HLC commands, not as a substitute for actual initial state. ' ...
     'On replan, bind candidate C7 jet to the actually executed reference ' ...
     'without a discontinuity; independently certify whole-body safety.'];
T.reason_not_autopatched= ...
    ['Simply replacing pL_ref(:,1) with estimated pL creates inconsistent ' ...
     'velocity/acceleration/heading/rope and does not establish HLC trackability ' ...
     'or 4-s continuous-time reachable-set bounds.'];
end

function A = r48r2_return(R)
A=struct('status','NOT_RUN__REQUIRED_RETURN_FIELDS_MISSING');
if ~isfield(R,'return') || ~isstruct(R.return),return;end
D=R.return;
required={'live_terminal_left_limit_norm_m','live_rejoin_proved_by_source', ...
    'flight_end_s','live_terminal_hold_start_s','live_rejoin_time_s', ...
    'path_parameterization_kind','candidate_shape_is_live_commit'};
if ~all(isfield(D,required)),return;end
A.live_terminal_offset_norm_m=double(D.live_terminal_left_limit_norm_m);
A.live_rejoin_proved=logical(D.live_rejoin_proved_by_source);
A.flight_end_s=double(D.flight_end_s);
A.hold_start_s=double(D.live_terminal_hold_start_s);
A.live_rejoin_time_s=double(D.live_rejoin_time_s);
A.continuation_kind=char(D.path_parameterization_kind);
A.r45_numerical_candidate_committed=logical(D.candidate_shape_is_live_commit);
A.hold_after_flight=A.hold_start_s>A.flight_end_s;
A.return_not_proved=~A.live_rejoin_proved;
A.status='LIVE_RETURN_REQUIRES_CONTINUATION_REPAIR_AND_PROOF';
if A.live_rejoin_proved && ~A.hold_after_flight && ...
        A.r45_numerical_candidate_committed && isfinite(A.live_rejoin_time_s)
    A.status='SOURCE_MARKED_REJOIN__INDEPENDENT_SAFETY_STILL_REQUIRED';
end
A.overshoot_full_curve_not_proved_from_R47=true;
A.overshoot_source_recommendation= ...
    ['Use actual R44R2 two-layer Bezier control points. Examine signed ' ...
     'Bernstein extrema and full vector derivatives, not only sampled points ' ...
     'or the terminal norm. Optimize free terminal time and phase delay, ' ...
     'enforce C7 all joins, yaw C6, whole-body and towing constraints, ' ...
     'and forbid same-clock catch-up.'];
end

function A = r48r2_replanner_timing(R)
A=struct('status','NOT_RUN__REQUIRED_TIMING_FIELDS_MISSING');
if ~isfield(R,'timing') || ~isstruct(R.timing),return;end
D=R.timing;
needed={'top_rows','recorded_flight_cycles', ...
    'stage1_native_event_t_s','stage1_native_returned_ms', ...
    'all_flight_summary_ms'};
if ~all(isfield(D,needed)),return;end
X=double(D.top_rows);
if size(X,2)~=12 || any(~isfinite(X(:)),'all'),return;end
A.recorded_flight_cycles=double(D.recorded_flight_cycles);
S=D.all_flight_summary_ms;
A.reference_p50_p95_max_ms=double(S.reference_replanner(:)).';
A.controller_spikes_excluded_from_scope=true;
A.all_cycles_under_25ms_proved=false;
A.replanner_overhead_under_10ms_proved=false;
A.replanner_goal_added_ms=10;
A.replanner_preferred_added_ms=5;
% Columns: time, full, online, acquire, sensor, estimator, reference,
% controller, input, plant, logger, other. Sort ONLY reference column.
[~,ord]=sort(X(:,7),'descend');
idx=ord(1:min(12,numel(ord)));
A.replanner_ranked_top_rows=X(idx,[1 7 2 3 6 10 11 12]);
A.replanner_ranked_column_labels={ ...
    'time_s','reference_replanner_ms','full_cycle_ms', ...
    'online_ms','estimator_ms','plant_ms','logger_ms','other_ms'};
A.stage1_times_s=double(D.stage1_native_event_t_s(:));
A.stage1_native_ms=double(D.stage1_native_returned_ms(:));
A.stage1_reference_same_time_ms=NaN(size(A.stage1_times_s));
A.stage1_reference_minus_native_ms=NaN(size(A.stage1_times_s));
for k=1:numel(A.stage1_times_s)
    j=find(abs(X(:,1)-A.stage1_times_s(k))<1e-8,1);
    if ~isempty(j)
        A.stage1_reference_same_time_ms(k)=X(j,7);
        A.stage1_reference_minus_native_ms(k)=X(j,7)-A.stage1_native_ms(k);
    end
end
A.stage1_minus_native_is_proven_exclusive_wall_bucket=false;
A.same_run_source_pairing_verified=logical(R.source_pairing_verified);
A.status='REPLANNER_SPIKES_ISOLATED__STAGE1_WRAPPER_STAGE2_UNRESOLVED';
A.next_measure= ...
    ['Record nested QPC wall buckets for Stage1 native call, packet build, ' ...
     'MEX internal, wrapper/materialization, Stage2 certificate and bundle, ' ...
     'Stage3 precommit and postcommit/cache, aligned to one SAME Flight cycle; ' ...
     'do not optimize controller 24.95 s system spike.'];
end

function hash = r48r2_hash_file(path)
hash='UNAVAILABLE';
try
    fid=fopen(path,'rb');assert(fid>=0,'Source file read failed.');
    cleaner=onCleanup(@() fclose(fid));
    B=fread(fid,Inf,'*uint8');
    md=java.security.MessageDigest.getInstance('SHA-256');
    md.update(typecast(B(:),'int8'));
    digest=typecast(md.digest(),'uint8');
    hash=lower(reshape(dec2hex(digest,2).',1,[]));
    clear cleaner;
catch
    % Hash display optional; exact source path, size and call sites retained.
end
end

function r48r2_write_report(OUT)
fid=fopen(OUT.report_file,'wt');
assert(fid>=0,'[R48R2] Cannot write report file.');
cleaner=onCleanup(@() fclose(fid));
fprintf(fid,'AF7R58R48R2 | THREE ISSUES | SOURCE-BOUND READ-ONLY\n');
fprintf(fid,'production_modified=0 | trigger_modified=0 | commit_modified=0\n');
fprintf(fid,'controller_24s_spike_in_scope=0\n');
fprintf(fid,'source=%s\n',OUT.source_file);
fprintf(fid,'R47=%s\n',OUT.r47_file);
fprintf(fid,'source_pairing_verified=%d\n',OUT.source_pairing_verified);
fprintf(fid,'\n[A] RETURN / OVERSHOOT\nstatus=%s\n',OUT.return.status);
if isfield(OUT.return,'live_terminal_offset_norm_m')
    fprintf(fid,'live_offset_norm_m=%.15g\n',OUT.return.live_terminal_offset_norm_m);
    fprintf(fid,'flight_end_s=%.12g | hold_start_s=%.12g | rejoin_s=%.12g\n', ...
        OUT.return.flight_end_s,OUT.return.hold_start_s,OUT.return.live_rejoin_time_s);
    fprintf(fid,'r45_candidate_committed=%d | live_rejoin_proved=%d\n', ...
        OUT.return.r45_numerical_candidate_committed,OUT.return.live_rejoin_proved);
    fprintf(fid,'candidate_next=%s\n',OUT.return.overshoot_source_recommendation);
end
fprintf(fid,'\n[B] REPLANNER TIME ONLY (controller excluded from optimization)\nstatus=%s\n',OUT.timing.status);
if isfield(OUT.timing,'replanner_ranked_top_rows')
    fprintf(fid,'reference_p50_p95_max_ms=%.9g %.9g %.9g\n',OUT.timing.reference_p50_p95_max_ms);
    fprintf(fid,'RANK: t_s reference_ms full_ms online_ms estimator_ms plant_ms logger_ms other_ms\n');
    for k=1:size(OUT.timing.replanner_ranked_top_rows,1)
        fprintf(fid,'%.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n',OUT.timing.replanner_ranked_top_rows(k,:));
    end
    fprintf(fid,'STAGE1: time native_ms reference_same_time_ms ref_minus_native_ms\n');
    for k=1:numel(OUT.timing.stage1_times_s)
        fprintf(fid,'%.6f %.6f %.6f %.6f\n', ...
            OUT.timing.stage1_times_s(k),OUT.timing.stage1_native_ms(k), ...
            OUT.timing.stage1_reference_same_time_ms(k), ...
            OUT.timing.stage1_reference_minus_native_ms(k));
    end
    fprintf(fid,'CAUTION: ref_minus_native is not an internally closed timer ledger.\n');
    fprintf(fid,'next=%s\n',OUT.timing.next_measure);
end
fprintf(fid,'\n[C] TRIGGER START FROM CURRENT ESTIMATE\nstatus=%s\n',OUT.trigger.status);
if isfield(OUT.trigger,'lines')
    fprintf(fid,'source_sha256=%s\n',OUT.trigger.source_sha256);
    fs=fieldnames(OUT.trigger.lines);
    for k=1:numel(fs)
        v=OUT.trigger.lines.(fs{k});
        fprintf(fid,'%s lines=%s\n',fs{k},mat2str(v));
    end
    fprintf(fid,'estimated_position_sensor=%d | reference_packet_future=%d | stage1_reference_start=%d\n', ...
        OUT.trigger.live_estimator_used_for_sensing, ...
        OUT.trigger.future_reference_used_for_collision, ...
        OUT.trigger.stage1_start_from_reference_packet);
    fprintf(fid,'NOT A FIX: %s\n',OUT.trigger.reason_not_autopatched);
    fprintf(fid,'next=%s\n',OUT.trigger.suggested_design);
end
fprintf(fid,'\nLIMITS: no physical prediction executed, no candidate commit, no safety/wcet proof.\n');
fprintf(fid,'workspace_evidence_names=%s\n',strjoin(OUT.workspace_named_evidence_present,','));
clear cleaner;
end

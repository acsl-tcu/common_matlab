function S = CHECK_TRIGGER_5RUN_SPIKE_CAUSES()
% CHECK_TRIGGER_5RUN_SPIKE_CAUSES  Five real Flight runs with phase attribution.
% Put this script in common_matlab/; extract ZIP manually, then:
%   clear classes; rehash
%   S=CHECK_TRIGGER_5RUN_SPIKE_CAUSES(); disp(S.status)
%
% Uses the UNMODIFIED run_flight_headless and existing Sim LOGGER. The only
% runtime diagnostic change is REPLANNING_STAGE_TIMING=1 for the FIRST TWO
% Flight constructors. Flights 3-5 run without stage clocks for a timing
% comparison to the previous 5-flight baseline. No physical model, HLC, plant, observer, sensor,
% prediction, or collision semantics are changed by this auditor.
% Stage clocks have measurement overhead. Compare measured time distributions
% to the baseline WITH that caveat; do not claim strict WCET from five runs.
root=locate_root(fileparts(mfilename('fullpath')));
base=fullfile(root,'reference','REPLANNING_SYSTEM');
expected=fullfile(base,'FUTURE_COLLISION_TRIGGER','FUTURE_COLLISION_TRIGGER.m');
orig_dir=pwd;
orig_stage=getenv('REPLANNING_STAGE_TIMING');
orig_cpu=getenv('REPLANNING_THREAD_CPU_DIAG');
cleanup=onCleanup(@()restore_environment(orig_dir,orig_stage,orig_cpu)); %#ok<NASGU>
cd(root);
addpath(base,'-begin');
addpath(fullfile(base,'FUTURE_COLLISION_TRIGGER'),'-begin');
addpath(root,'-begin');
assert(strcmpi(strrep(which('FUTURE_COLLISION_TRIGGER'),'\','/'), ...
    strrep(expected,'\','/')), ...
    'CHECK_TRIGGER_5RUN_SPIKE_CAUSES:WrongClass', ...
    'Wrong active FUTURE_COLLISION_TRIGGER class. clear classes; rehash.');
assert(isfile(fullfile(root,'CHECK_TRIGGER_FLIGHT_FINAL.m')), ...
    'CHECK_TRIGGER_5RUN_SPIKE_CAUSES:MissingAudit', ...
    'Extract CHECK_TRIGGER_FLIGHT_FINAL.m into common_matlab/.');
% This check enforces that the retired sampled-only native payload is NOT
% being constructed again by the real-time collision trigger.
core_text=fileread(expected);
assert(~contains(core_text,'request.native_sampled_outer_balls='), ...
    'CHECK_TRIGGER_5RUN_SPIKE_CAUSES:OldSource', ...
    'The sampled-only native outer-ball payload is still created.');
assert(contains(core_text,'TRIGGER_NOMINAL_SWEPT_INTERVALS('), ...
    'CHECK_TRIGGER_5RUN_SPIKE_CAUSES:MissingSweep', ...
    'The continuous nominal swept test is not present.');
sweep_text=fileread(fullfile(base,'FUTURE_COLLISION_TRIGGER', ...
    'TRIGGER_NOMINAL_SWEPT_INTERVALS.m'));
assert(contains(sweep_text,'Fast certified clearance for either BODY segment'), ...
    'CHECK_TRIGGER_5RUN_SPIKE_CAUSES:OldSweptBodyPath', ...
    'Apply the complete directional sweep fast-path source first.');
assert(contains(sweep_text,'function D=validated_obstacle_support(sh)'), ...
    'CHECK_TRIGGER_5RUN_SPIKE_CAUSES:StaleShapeSweep', ...
    'Install the multi-shape source and run clear classes; rehash.');
% Pure theorem check for the exact segment-ball early separation.
G=CHECK_TRIGGER_SEGMENT_CLEARANCE_PROOF();
assert(strcmp(G.status,'PASS_SEGMENT_ENCLOSING_BALL_SHORTCUT'), ...
    'CHECK_TRIGGER_5RUN_SPIKE_CAUSES:SegmentClearanceRegression', ...
    'The independent geometric shortcut proof failed.');
% Fast synthetic semantic guard before the expensive five real Flights.
% The check does not alter a source or perform a predictive warmup.
H=CHECK_TRIGGER_SPIKE_HOTPATH();
assert(strcmp(H.status,'PASS_SWEEP_HOTPATH_PARITY'), ...
    'CHECK_TRIGGER_5RUN_SPIKE_CAUSES:HotpathRegression', ...
    'Do not run real Flight with a failed sweep/decision regression.');
% Check sphere, rotated ellipsoid, box, cylinder, capsule, cone/frustum,
% torus, pyramids, polyhedra and arbitrary polygonal prisms BEFORE Flight.
B=CHECK_TRIGGER_MULTISHAPE_FAST_PATH();
assert(strcmp(B.status,'PASS_MULTISHAPE_DIRECTIONAL_SUPPORT_SWEEP'), ...
    'CHECK_TRIGGER_5RUN_SPIKE_CAUSES:ShapeRegression', ...
    'Multi-shape contact / ID / time regression failed.');
% Verify the static sensor's cached-visible path against a fresh shape
% selection on unchanged, entered, exited and empty detection masks.
V=CHECK_SENSOR_STATIC_VISIBLE_PARITY();
assert(strcmp(V.status,'PASS_SENSOR_STATIC_VISIBLE_PARITY'), ...
    'CHECK_TRIGGER_5RUN_SPIKE_CAUSES:SensorCacheParity', ...
    'The optimized sensor must return the exact original visible structs.');
% Optional CPU-clock MEX is unrelated to these non-overlapping stage logs.
setenv('REPLANNING_THREAD_CPU_DIAG','0');
Nrun=5;
% Do not archive results accidentally left from an earlier failed run.
for stale_k=1:Nrun
    for ext={'mat','txt'}
        stale=fullfile(root,sprintf('TRIGGER_SPIKE_RUN_%02d.%s', ...
            stale_k,ext{1}));
        if isfile(stale),delete(stale);end
    end
end
results=cell(Nrun,1);
errors=cell(Nrun,1);
for k=1:Nrun
    diagnostic_run=k<=2;
    if diagnostic_run
        setenv('REPLANNING_STAGE_TIMING','1');
    else
        setenv('REPLANNING_STAGE_TIMING','0');
    end
    fprintf('[SPIKE-5] Real Flight %d/%d (stage clocks %s)\n', ...
        k,Nrun,mat2str(diagnostic_run));
    try
        cd(root);
        evalin('base','clear logger');
        evalin('base','run_flight_headless;');
        assert(evalin('base','exist(''logger'',''var'')')==1, ...
            'CHECK_TRIGGER_5RUN_SPIKE_CAUSES:NoLogger', ...
            'Original run_flight_headless did not return logger.');
        logobj=evalin('base','logger');
        A=CHECK_TRIGGER_FLIGHT_FINAL(logobj,true);
        D=extract_stage_log(logobj);
        clear logobj;
        assert(~diagnostic_run || (D.stage_rows>0 && D.sensor_diag_rows>0 && ...
            D.memory_diag_rows>0 && D.trigger_core_rows>0), ...
            'CHECK_TRIGGER_5RUN_SPIKE_CAUSES:TimingDisabled', ...
            'Stage timing was not recorded; check constructor environment.');
        mat_name=sprintf('TRIGGER_SPIKE_RUN_%02d.mat',k);
        txt_name=sprintf('TRIGGER_SPIKE_RUN_%02d.txt',k);
        save(fullfile(root,mat_name),'A','D','-v7');
        write_run_report(fullfile(root,txt_name),k,A,D);
        results{k}=struct('status',A.status, ...
            'stage_timing_on',diagnostic_run, ...
            'functional_failures',A.failures,'rows',D.N, ...
            'trigger_median_ms',median_finite(D.yellow_ms), ...
            'replanner_median_ms',median_finite(D.blue_ms), ...
            'trigger_p95_ms',prctile_finite(D.yellow_ms,95), ...
            'replanner_p95_ms',prctile_finite(D.blue_ms,95), ...
            'trigger_max_ms',max_finite(D.yellow_ms), ...
            'replanner_max_ms',max_finite(D.blue_ms), ...
            'trigger_over4',nnz(D.yellow_ms>4), ...
            'replanner_over4',nnz(D.blue_ms>4), ...
            'sensor_max_ms',max_finite(D.detail_ms(:,1)), ...
            'memory_max_ms',max_finite(D.detail_ms(:,2)), ...
            'observed_new_id_frames',nnz(D.new_id_count>0), ...
            'prediction_count',A.native_predictions, ...
            'event_table',A.new_event_table, ...
            'future_phase_gap_frames',A.unmodelled_future_phase_frames);
        fprintf(['[SPIKE-5] %s | yellow median/max %.3f/%.3f ms, ' ...
            'blue median/max %.3f/%.3f ms, over4 yellow/blue %d/%d\n'], ...
            A.status,results{k}.trigger_median_ms,results{k}.trigger_max_ms, ...
            results{k}.replanner_median_ms,results{k}.replanner_max_ms, ...
            results{k}.trigger_over4,results{k}.replanner_over4);
    catch ME
        errors{k}=[ME.identifier ': ' ME.message];
        results{k}=struct('status','FAIL_FLIGHT_OR_TIMING_AUDIT', ...
            'functional_failures',1);
        fprintf(2,'[SPIKE-5] Flight %d failure: %s\n',k,errors{k});
    end
end
pass=all(cellfun(@(r)isstruct(r) && isfield(r,'status') && ...
    strcmp(r.status,'PASS_REAL_FLIGHT_DIRECTIONAL_SWEEP_WITH_PHASE_GAP'),results));
if pass
    status='PASS_5_FLIGHTS_2_DETAIL_3_PRODUCTION';
else
    status='FAIL_OR_INCOMPLETE_5_FLIGHTS_SPIKE_AUDIT';
end
S=struct('status',status,'functional_pass',pass,'runs',{results}, ...
    'errors',{errors},'diagnostic_flights',2,'production_flights',3, ...
    'timing_instrumentation_overhead_on_first_two',true,'wcet_proven',false, ...
    'sampled_native_outer_payload_retired',true, ...
    'physical_continuous_ode_safety_proven',false);
% Functional success and latency are separate conclusions.
prod=results(3:end);
S.performance_status='NOT_MEASURED';
if all(cellfun(@(r)isstruct(r)&&isfield(r,'replanner_max_ms')&& ...
        isfinite(r.replanner_max_ms),prod))
    max_blue=max(cellfun(@(r)r.replanner_max_ms,prod));
    max_yellow=max(cellfun(@(r)r.trigger_max_ms,prod));
    if max_blue<=4 && max_yellow<=4
        S.performance_status='FIVE_RUNS_WITHIN_4MS__NOT_HARD_WCET';
    else
        S.performance_status='REMAINING_SPIKES_REQUIRE_EXPLANATION';
    end
end
write_spike_attribution(fullfile(root,'TRIGGER_SPIKE_ATTRIBUTION.txt'),root,Nrun,S);
save(fullfile(root,'TRIGGER_SPIKE_CAUSES_SUMMARY.mat'),'S','-v7');
write_summary(fullfile(root,'TRIGGER_SPIKE_CAUSES_SUMMARY.txt'),S);
files={'CHECK_TRIGGER_SEGMENT_CLEARANCE_PROOF.m', ...
    'CHECK_TRIGGER_5RUN_SPIKE_CAUSES.m', ...
    'CHECK_SENSOR_STATIC_VISIBLE_PARITY.m', ...
    'reference/REPLANNING_SYSTEM/SIMULATED_OBSTACLE_SENSOR/SIMULATED_OBSTACLE_SENSOR.m', ...
    'CHECK_TRIGGER_FLIGHT_FINAL.m','CHECK_TRIGGER_SPIKE_HOTPATH.m', ...
    'CHECK_TRIGGER_MULTISHAPE_FAST_PATH.m', ...
    'TRIGGER_MULTISHAPE_FAST_PATH_RESULT.txt', ...
    'reference/REPLANNING_SYSTEM/FUTURE_COLLISION_TRIGGER/FUTURE_COLLISION_TRIGGER.m', ...
    'reference/REPLANNING_SYSTEM/FUTURE_COLLISION_TRIGGER/TRIGGER_NOMINAL_SWEPT_INTERVALS.m', ...
    'TRIGGER_SPIKE_CAUSES_SUMMARY.mat','TRIGGER_SPIKE_CAUSES_SUMMARY.txt', ...
    'TRIGGER_SPIKE_ATTRIBUTION.txt'};
for k=1:Nrun
    if isfile(fullfile(root,sprintf('TRIGGER_SPIKE_RUN_%02d.mat',k))) && ...
            isfile(fullfile(root,sprintf('TRIGGER_SPIKE_RUN_%02d.txt',k)))
        files{end+1}=sprintf('TRIGGER_SPIKE_RUN_%02d.mat',k); %#ok<AGROW>
        files{end+1}=sprintf('TRIGGER_SPIKE_RUN_%02d.txt',k); %#ok<AGROW>
    end
end
archive=fullfile(root,'TRIGGER_BLUE_YELLOW_FOCUSED_RESULTS.zip');
if isfile(archive),delete(archive);end
zip(archive,files,root);
% Keep only the requested ONE archive for generated results. Audits are
% temporary; existing production sources and external Flight logs remain.
for j=1:numel(files)
    candidate=files{j};
    if startsWith(candidate,'TRIGGER_SPIKE_RUN_') || ...
            startsWith(candidate,'TRIGGER_SPIKE_CAUSES_SUMMARY.') || ...
            strcmp(candidate,'TRIGGER_SPIKE_ATTRIBUTION.txt') || ...
            strcmp(candidate,'TRIGGER_MULTISHAPE_FAST_PATH_RESULT.txt')
        delete(fullfile(root,candidate));
    end
end
if isfile(fullfile(root,'TRIGGER_FLIGHT_FINAL_RESULT.mat'))
    delete(fullfile(root,'TRIGGER_FLIGHT_FINAL_RESULT.mat'));
end
if isfile(fullfile(root,'TRIGGER_FLIGHT_FINAL_RESULT.txt'))
    delete(fullfile(root,'TRIGGER_FLIGHT_FINAL_RESULT.txt'));
end
fprintf('[SPIKE-5] %s\n',S.status);
fprintf('[SPIKE-5] Share ONE ZIP: %s\n',archive);
end

function D=extract_stage_log(logger)
if isobject(logger) && isprop(logger,'Data')
    data=logger.Data;
elseif isstruct(logger) && isfield(logger,'Data')
    data=logger.Data;
else
    data=logger;
end
assert(isstruct(data)&&isscalar(data) && isfield(data,'agent') && ...
    isfield(data,'t') && isfield(data.agent,'reference'), ...
    'CHECK_TRIGGER_5RUN_SPIKE_CAUSES:Logger', ...
    'Expected Sim LOGGER.Data.agent.reference and .t.');
raw=data.agent(1).reference.result;
if iscell(raw),frames=raw(:);else,frames=num2cell(raw(:));end
n=min(numel(frames),numel(data.t));
D=struct();
D.N=n;
D.time_s=nan(n,1);
D.blue_ms=nan(n,1);
D.yellow_ms=nan(n,1);
D.outside_trigger_ms=nan(n,1);
D.detail_ms=nan(n,6); % [sensor, memory, new shape, trigger, obstacle log, log attachments]
D.replanner_stage_ms=nan(n,6);
D.shadow_ms=nan(n,1);
D.sensor_stage_ms=nan(n,5); % [scan, exact(nested!), events, visible, finish]
D.memory_stage_ms=nan(n,4); % [validate, topology, refresh, finish]
D.sensor_elapsed_ms=nan(n,1);
D.memory_elapsed_ms=nan(n,1);
D.new_id_count=nan(n,1);
D.currently_detected_count=nan(n,1);
D.sensor_new_enter_count=nan(n,1);
D.sensor_left_range_count=nan(n,1);
D.sensor_mask_changed=nan(n,1);
D.observed_memory_count=nan(n,1);
D.trigger_parts_ms=nan(n,9); % [refNow, authority, core, prePredictor, predictor, refCache, sweep, geometry, outer after]
D.cache_hits=nan(n,1);
D.cache_fresh=nan(n,1);
D.cache_resets=nan(n,1);
D.verified_new_geometry=nan(n,1);
D.trigger_status=cell(n,1);
D.stage_rows=0;D.sensor_diag_rows=0;D.memory_diag_rows=0;D.trigger_core_rows=0;
for i=1:n
    f=frames{i};
    if ~isstruct(f)||~isscalar(f),continue;end
    tr=read_field(f,'future_collision_trigger',struct());
    if ~isstruct(tr)||~isscalar(tr),continue;end
    D.time_s(i)=num(read_field(tr,'time_s',NaN));
    D.yellow_ms(i)=num(read_field(tr,'elapsed_ms',NaN));
    D.blue_ms(i)=num(read_field(f,'replanning_time_ms',NaN));
    D.outside_trigger_ms(i)=D.blue_ms(i)-D.yellow_ms(i);
    D.trigger_status{i}=char(string(read_field(tr,'status','UNKNOWN')));
    a=read_field(f,'replanning_detail_ms',[]);
    if isnumeric(a)&&numel(a)==6
        D.detail_ms(i,:)=double(a(:)).';
        D.stage_rows=D.stage_rows+1;
    end
    a=read_field(f,'replanning_stage_ms',[]);
    if isnumeric(a)&&numel(a)==6
        D.replanner_stage_ms(i,:)=double(a(:)).';
    end
    D.shadow_ms(i)=num(read_field(f,'replanning_shadow_ms',NaN));
    sen=read_field(f,'simulated_obstacle_sensor',struct());
    if isstruct(sen)&&isscalar(sen)
        D.sensor_elapsed_ms(i)=num(read_field(sen,'elapsed_ms',NaN));
        D.currently_detected_count(i)=num(read_field(sen,'detected_count',NaN));
        appeared=read_field(sen,'newly_detected_ids',[]);
        disappeared=read_field(sen,'left_range_ids',[]);
        if isnumeric(appeared),D.sensor_new_enter_count(i)=numel(appeared);end
        if isnumeric(disappeared),D.sensor_left_range_count(i)=numel(disappeared);end
        changed=read_field(sen,'diagnostic_sensor_mask_changed',[]);
        if islogical(changed)&&isscalar(changed),D.sensor_mask_changed(i)=double(changed);end
        a=read_field(sen,'diagnostic_sensor_stage_ms',[]);
        if isnumeric(a)&&numel(a)==5
            D.sensor_stage_ms(i,:)=double(a(:)).';
            D.sensor_diag_rows=D.sensor_diag_rows+1;
        end
    end
    mem=read_field(f,'observed_obstacle_memory',struct());
    if isstruct(mem)&&isscalar(mem)
        D.memory_elapsed_ms(i)=num(read_field(mem,'elapsed_ms',NaN));
        D.observed_memory_count(i)=num(read_field(mem,'known_count',NaN));
        new_ids=read_field(mem,'newly_acquired_ids',[]);
        if isnumeric(new_ids),D.new_id_count(i)=numel(new_ids);end
        a=read_field(mem,'diagnostic_stage_ms',[]);
        if isnumeric(a)&&numel(a)==4
            D.memory_stage_ms(i,:)=double(a(:)).';
            D.memory_diag_rows=D.memory_diag_rows+1;
        end
    end
    fields={'timing_runtime_reference_now_ms', ...
        'timing_runtime_authority_ms','timing_runtime_core_ms', ...
        'timing_pre_predictor_ms','predictor_wall_ms', ...
        'predictor_reference_window_ms','nominal_interpolated_sweep_ms', ...
        'geometry_wall_ms','timing_post_geometry_ms'};
    for j=1:numel(fields)
        D.trigger_parts_ms(i,j)=num(read_field(tr,fields{j},NaN));
    end
    D.cache_hits(i)=num(read_field(tr,'reference_cache_hits',NaN));
    D.cache_fresh(i)=num(read_field(tr,'reference_cache_fresh',NaN));
    D.cache_resets(i)=double(is_true(read_field(tr,'reference_cache_reset',false)));
    D.verified_new_geometry(i)=double(is_true( ...
        read_field(tr,'geometry_first_use_verified_before_predictor',false)));
    if isfinite(D.trigger_parts_ms(i,3))
        D.trigger_core_rows=D.trigger_core_rows+1;
    end
end
end

function write_run_report(path,idx,A,D)
fid=fopen(path,'w');
assert(fid>0,'CHECK_TRIGGER_5RUN_SPIKE_CAUSES:File','Cannot write %s',path);
cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'RUN=%d functional_audit=%s failures=%d\n',idx,A.status,A.failures);
fprintf(fid,'stage_clocks=%d (first 2 diagnostic; last 3 production timing; not a WCET proof)\n',idx<=2);
fprintf(fid,'frames=%d stage=%d sensor_detail=%d memory_detail=%d core_detail=%d\n', ...
    D.N,D.stage_rows,D.sensor_diag_rows,D.memory_diag_rows,D.trigger_core_rows);
fprintf(fid,'trigger/yellow median=%.6f p95=%.6f max=%.6f over4=%d\n', ...
    median_finite(D.yellow_ms),prctile_finite(D.yellow_ms,95), ...
    max_finite(D.yellow_ms),nnz(D.yellow_ms>4));
fprintf(fid,'replanner/blue median=%.6f p95=%.6f max=%.6f over4=%d\n', ...
    median_finite(D.blue_ms),prctile_finite(D.blue_ms,95), ...
    max_finite(D.blue_ms),nnz(D.blue_ms>4));
fprintf(fid,'sensor elapsed MAX=%.6f memory elapsed MAX=%.6f new-observation frames=%d\n', ...
    max_finite(D.detail_ms(:,1)),max_finite(D.detail_ms(:,2)), ...
    nnz(D.new_id_count>0));
fprintf(fid,'WARNING: sensor exact-ms is INSIDE sensor scan-ms; never add both.\n');
fprintf(fid,'WARNING: core-ms is INSIDE trigger/yellow; prePredictor, predictor, sweep are portions of core.\n');
valid=find(isfinite(D.time_s)&isfinite(D.yellow_ms)&isfinite(D.blue_ms));
[~,a]=sort(D.blue_ms(valid),'descend');
write_top(fid,'TOP_BLUE_REPLANNER',D,valid(a(1:min(18,numel(a)))));
[~,a]=sort(D.yellow_ms(valid),'descend');
write_top(fid,'TOP_YELLOW_TRIGGER',D,valid(a(1:min(18,numel(a)))));
% Time-locked peaks in the baseline are included EVEN IF they no longer
% appear among the highest spikes. These are just audit observation times,
% not planning constraints or precomputed detection schedules.
probe=[0 5 5.125 14.025 14.05 19 38.35];
idxs=zeros(1,0);
for t=probe
    [v,j]=min(abs(D.time_s-t));
    if isfinite(v)&&v<=1e-6,idxs(end+1)=j;end %#ok<AGROW>
end
write_top(fid,'BASELINE_PHASE_LANDMARKS',D,unique(idxs,'stable'));
end

function write_top(fid,label,D,idx)
fprintf(fid,'\n=== %s ===\n',label);
fprintf(fid,['time_s,blue_ms,yellow_ms,blue_minus_yellow_ms,' ...
    'sensor_ms,memory_ms,new_shape_ms,obstacle_log_ms,attachment_ms,' ...
    'shadow_ms,new_IDs,entered_range,left_range,mask_changed,visible,known,' ...
    'ref_now_ms,authority_ms,core_ms,pre_predictor_ms,predictor_ms,' ...
    'ref_cache_ms,sweep_ms,geometry_ms,cache_hit,cache_fresh,' ...
    'verified_new_geom,status\n']);
for k=reshape(idx,1,[])
    d=D.detail_ms(k,:); t=D.trigger_parts_ms(k,:);
    fprintf(fid,['%.9g,%.6f,%.6f,%.6f,' ...
        '%.6f,%.6f,%.6f,%.6f,%.6f,' ...
        '%.6f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,' ...
        '%.6f,%.6f,%.6f,%.6f,%.6f,' ...
        '%.6f,%.6f,%.6f,%.0f,%.0f,%.0f,%s\n'], ...
        D.time_s(k),D.blue_ms(k),D.yellow_ms(k), ...
        D.outside_trigger_ms(k),d(1),d(2),d(3),d(5),d(6), ...
        D.shadow_ms(k),D.new_id_count(k), ...
        D.sensor_new_enter_count(k),D.sensor_left_range_count(k), ...
        D.sensor_mask_changed(k), ...
        D.currently_detected_count(k),D.observed_memory_count(k), ...
        t(1),t(2),t(3),t(4),t(5),t(6),t(7),t(8), ...
        D.cache_hits(k),D.cache_fresh(k), ...
        D.verified_new_geometry(k),D.trigger_status{k});
end
end

function write_summary(path,S)
fid=fopen(path,'w');
assert(fid>0,'CHECK_TRIGGER_5RUN_SPIKE_CAUSES:File', ...
    'Cannot write spike summary.');
cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s\n',S.status);
fprintf(fid,'ALL results use actual Sim LOGGER and original runner.\n');
fprintf(fid,'Flights 1-2: stage clocks ON; include instrumentation overhead.\n');
fprintf(fid,'Flights 3-5: normal production clocks; compare to prior baseline.\n');
fprintf(fid,'Guaranteed: no change to directionally swept nominal collision test.\n');
fprintf(fid,'Not claimed: physical true-ODE safety and hard WCET.\n');
fprintf(fid,['run,status,native_predictions,event_count,trigger_median_ms,' ...
   'trigger_p95_ms,trigger_max_ms,trigger_over4,' ...
   'replanner_median_ms,replanner_max_ms,replanner_over4,' ...
   'sensor_max_ms,memory_max_ms,new_id_frames,landing_gap_frames\n']);
for k=1:numel(S.runs)
    r=S.runs{k};
    if ~isfield(r,'prediction_count')
        fprintf(fid,'%d,%s\n',k,r.status);continue
    end
    fprintf(fid,['%d,%d,%s,%d,%d,%.6f,%.6f,%.6f,%d,' ...
        '%.6f,%.6f,%d,%.6f,%.6f,%d,%d\n'], ...
        k,r.stage_timing_on,r.status,r.prediction_count,size(r.event_table,1), ...
        r.trigger_median_ms,r.trigger_p95_ms,r.trigger_max_ms, ...
        r.trigger_over4,r.replanner_median_ms,r.replanner_max_ms, ...
        r.replanner_over4,r.sensor_max_ms,r.memory_max_ms, ...
        r.observed_new_id_frames,r.future_phase_gap_frames);
end
end

function x=read_field(s,n,default)
if isstruct(s)&&isscalar(s)&&isfield(s,n),x=s.(n);else,x=default;end
end
function z=num(x)
if isnumeric(x)&&isreal(x)&&isscalar(x),z=double(x);else,z=NaN;end
end
function b=is_true(x)
b=(islogical(x)||isnumeric(x))&&isscalar(x)&&isequal(x,true);
end
function v=median_finite(x)
x=x(isfinite(x));if isempty(x),v=NaN;else,v=median(x);end
end
function v=max_finite(x)
x=x(isfinite(x));if isempty(x),v=NaN;else,v=max(x);end
end
function v=prctile_finite(x,p)
x=sort(x(isfinite(x)));
if isempty(x),v=NaN;return;end
% Interpolated empirical quantile, not an exact deterministic WCET.
z=1+(numel(x)-1)*p/100;lo=floor(z);hi=ceil(z);
v=x(lo)+(z-lo)*(x(hi)-x(lo));
end
function root=locate_root(folder)
root=folder;
for i=1:9
    if isfile(fullfile(root,'reference','REPLANNING_SYSTEM','REPLANNING_SYSTEM.m')) ...
            && isfile(fullfile(root,'reference','REPLANNING_SYSTEM', ...
            'FUTURE_COLLISION_TRIGGER','FUTURE_COLLISION_TRIGGER.m'))
        return
    end
    up=fileparts(root);
    if strcmp(root,up),break;end
    root=up;
end
error('CHECK_TRIGGER_5RUN_SPIKE_CAUSES:Root', ...
    'Cannot find common_matlab above %s',folder);
end
function restore_environment(old_dir,stage,cpu)
setenv('REPLANNING_STAGE_TIMING',stage);
setenv('REPLANNING_THREAD_CPU_DIAG',cpu);
cd(old_dir);
end

function write_spike_attribution(path,root,Nrun,S)
% All derived data come from existing Sim LOGGER stage measurements.
% Nested wall clocks MUST NOT be double counted.
f=fopen(path,'w');
assert(f>0,'CHECK_TRIGGER_5RUN_SPIKE_CAUSES:AttributionOutput', ...
    'Could not open causal attribution log.');
c=onCleanup(@()fclose(f)); %#ok<NASGU>
fprintf(f,'FUNCTION=%s\nPERFORMANCE=%s\n',S.status,S.performance_status);
fprintf(f,['CAUTION: profiler clocks alter diagnostic runs. ' ...
    'Unattributed delay is NOT proof of OS jitter.\n']);
fprintf(f,['COLUMNS: run,time,blue,yellow,nontrigger,scan,memory,prepare,' ...
    'log,attach,shadow,unaccounted_nontrigger,' ...
    'refnow,authority,core,unaccounted_trigger,' ...
    'predictor,sweep,core_unaccounted,phase,newIDs\n']);
for k=1:Nrun
    name=fullfile(root,sprintf('TRIGGER_SPIKE_RUN_%02d.mat',k));
    if ~isfile(name),continue;end
    X=load(name,'D');D=X.D;
    n=numel(D.blue_ms);
    [~,ib]=sort(D.blue_ms,'descend');
    [~,iy]=sort(D.yellow_ms,'descend');
    flagged=unique([ib(1:min(16,n));iy(1:min(16,n)); ...
        find(D.blue_ms>4);find(D.yellow_ms>4)]);
    flagged=flagged(isfinite(D.blue_ms(flagged)) & ...
        isfinite(D.yellow_ms(flagged)));
    for ii=reshape(flagged,1,[])
        detail=D.detail_ms(ii,:);
        parts=D.trigger_parts_ms(ii,:);
        outside=D.blue_ms(ii)-D.yellow_ms(ii);
        % Six nonoverlapping additive stage clocks (no nested exact sensor!).
        accounted_outside=sum(detail([1 2 3 5 6]));
        sh=D.shadow_ms(ii);
        if isfinite(sh),accounted_outside=accounted_outside+sh;end
        outside_unaccounted=NaN;
        if all(isfinite(detail([1 2 3 5 6])))
            outside_unaccounted=outside-accounted_outside;
        end
        core=parts(3);
        trigger_unaccounted=NaN;
        core_unaccounted=NaN;
        if all(isfinite(parts(1:3)))
            trigger_unaccounted=D.yellow_ms(ii)-sum(parts(1:3));
        end
        if isfinite(core)&&isfinite(parts(5))&&isfinite(parts(8))
            core_unaccounted=core-parts(5)-parts(8);
        end
        if D.stage_rows>0
            phase='STAGE_DETAIL_AVAILABLE';
        else
            phase='NO_SUBSTAGE_CLOCKS__UNRESOLVED';
        end
        fprintf(f,['%d,%.6f,%.6f,%.6f,%.6f,' ...
            '%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,' ...
            '%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%s,%.0f\n'], ...
            k,D.time_s(ii),D.blue_ms(ii),D.yellow_ms(ii),outside, ...
            detail(1),detail(2),detail(3),detail(5),detail(6),sh, ...
            outside_unaccounted,parts(1),parts(2),core, ...
            trigger_unaccounted,parts(5),parts(7),core_unaccounted, ...
            phase,D.new_id_count(ii));
    end
end
fprintf(f,['INTERPRETATION: A slow yellow with slow predictor/sweep is ' ...
    'an algorithmic-stage cost. A simultaneous slow sensor and memory ' ...
    'without new IDs requires OS/JIT/thread evidence before attribution.\n']);
fprintf(f,['PRESENT LIMITS: only 2/5 flights have detailed clocks, ' ...
    'so large production spikes without a matched detailed trace ' ...
    'remain unresolved. No hard WCET claim.\n']);
end

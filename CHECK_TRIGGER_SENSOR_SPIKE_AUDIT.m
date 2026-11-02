function A=CHECK_TRIGGER_SENSOR_SPIKE_AUDIT(logger,replanner_instance,report_tag)
% CHECK_TRIGGER_SENSOR_SPIKE_AUDIT
% Read-only inspection of real Sim LOGGER records. No simulation, sensor
% scans, source geometry reads, model calls, or changes to flight objects.
% Optional stage fields are present only if REPLANNING_STAGE_TIMING=1
% was set BEFORE the flight began.
% Empty numeric ID sets are valid; diagnostic never modifies the sensor.
% If obstacle-memory logging is OFF, memory validity is NOT inferred.
% Optional 3rd argument is a report suffix (e.g., 'cold' or 'warm') so
% repeated MATLAB sessions do not overwrite one another's MAT/TXT.
if nargin<1 || ~isobject(logger) || ~isprop(logger,'Data')
    error('CHECK_TRIGGER_SENSOR_SPIKE_AUDIT:Logger', ...
        'Pass the existing Sim LOGGER object.');
end
D=logger.Data;
if ~isfield(D,'agent') || isempty(D.agent) || ...
        ~isfield(D.agent(1),'reference') || ...
        ~isfield(D.agent(1).reference,'result')
    error('CHECK_TRIGGER_SENSOR_SPIKE_AUDIT:Schema', ...
        'No recorded reference.result in the original Sim LOGGER.');
end
C=D.agent(1).reference.result;
if ~iscell(C), C=num2cell(C); end
n=min(numel(C),double(logger.k));
T=nan(n,1); TR=nan(n,1); RP=nan(n,1); SE=nan(n,1);
GE=nan(n,1); PR=nan(n,1); ST=nan(n,6);
status=cell(n,1); n_ids=nan(n,1); n_known=nan(n,1);
sensor_records=0; memory_records=0; trigger_records=0;
sensor_valid_records=0; considered_id_records=0;
failures=struct('index',{},'time_s',{},'kind',{},'details',{});
prev_ids=zeros(1,0); cumulative=zeros(1,0); have_prev=false;
for k=1:n
    r=C{k};
    if ~isstruct(r) || ~isscalar(r), continue; end
    if isfield(r,'replanning_time_ms'), RP(k)=scalar_number(r.replanning_time_ms); end
    if isfield(r,'trigger_time_ms'), TR(k)=scalar_number(r.trigger_time_ms); end
    if isfield(r,'replanning_stage_ms')
        v=r.replanning_stage_ms;
        if isnumeric(v) && isreal(v) && numel(v)==6 && all(isfinite(v(:)))
            ST(k,:)=double(v(:))';
        end
    end
    if isfield(r,'future_collision_trigger')
        q=r.future_collision_trigger;
        if isstruct(q) && isscalar(q)
            trigger_records=trigger_records+1;
            if isfield(q,'time_s'),T(k)=scalar_number(q.time_s);end
            if isfield(q,'status'),status{k}=char(q.status);end
            if isfield(q,'geometry_wall_ms'),GE(k)=scalar_number(q.geometry_wall_ms);end
            if isfield(q,'predictor_wall_ms'),PR(k)=scalar_number(q.predictor_wall_ms);end
        end
    end
    if ~isfield(r,'simulated_obstacle_sensor') || ...
            ~isstruct(r.simulated_obstacle_sensor) || ...
            ~isscalar(r.simulated_obstacle_sensor)
        continue
    end
    s=r.simulated_obstacle_sensor;
    sensor_records=sensor_records+1;
    if isfield(s,'time_s'), T(k)=scalar_number(s.time_s); end
    if isfield(s,'elapsed_ms'),SE(k)=scalar_number(s.elapsed_ms);end
    try
        must={'detected_ids','detected_count','detected_obstacles', ...
              'newly_detected_ids','left_range_ids'};
        if ~all(isfield(s,must))
            error('MissingSensorFields','Sensor result lacks required fields.');
        end
        ids=checked_ids(s.detected_ids);
        n_ids(k)=numel(ids);
        if ~isscalar(s.detected_count) || double(s.detected_count)~=numel(ids)
            error('Count','Detected count differs from ID count.');
        end
        obs=s.detected_obstacles;
        if ~isstruct(obs) || numel(obs)~=numel(ids) || ...
                ~isequal(ids,reshape(double([obs.id]),1,[]))
            error('GeometryIDs','Detected geometry and ID sequence differ.');
        end
        new_ids=checked_ids(s.newly_detected_ids);
        left_ids=checked_ids(s.left_range_ids);
        if have_prev
            if ~isequal(new_ids,setdiff(ids,prev_ids,'stable')) || ...
               ~isequal(left_ids,setdiff(prev_ids,ids,'stable'))
                error('Transitions','New/left ID events disagree with consecutive scans.');
            end
        end
        sensor_valid_records=sensor_valid_records+1;
        cumulative=union(cumulative,ids,'stable');
        if isfield(r,'observed_obstacle_memory') && ...
                isstruct(r.observed_obstacle_memory) && ...
                isscalar(r.observed_obstacle_memory)
            m=r.observed_obstacle_memory;
            memory_records=memory_records+1;
            if isfield(m,'currently_detected_ids') && ...
                    ~isequal(ids,checked_ids(m.currently_detected_ids))
                error('MemoryCurrent','Memory current IDs disagree with current scan.');
            end
            if isfield(m,'known_obstacle_ids')
                known=checked_ids(m.known_obstacle_ids);
                n_known(k)=numel(known);
                if ~isequal(known,cumulative)
                    error('MemoryAccumulated', ...
                        'Known obstacle IDs differ from cumulative actual detections.');
                end
            end
            if isfield(m,'automatic_release_enabled') && ...
                    ~isequal(logical(m.automatic_release_enabled),false)
                error('UnexpectedRelease','Automatic obstacle release was enabled.');
            end
        end
        if isfield(r,'future_collision_trigger') && ...
                isstruct(r.future_collision_trigger) && ...
                isfield(r.future_collision_trigger,'considered_obstacle_ids')
            use_ids=checked_ids(r.future_collision_trigger.considered_obstacle_ids);
            considered_id_records=considered_id_records+1;
            if ~all(ismember(use_ids,cumulative))
                error('FutureKnowledge','Trigger contains an obstacle never detected so far.');
            end
        end
        prev_ids=ids;have_prev=true;
    catch ME
        failures(end+1)=struct('index',k,'time_s',T(k), ...
            'kind',ME.identifier,'details',ME.message); %#ok<AGROW>
    end
end
if isfield(D,'system_cycle_time_ms') && isnumeric(D.system_cycle_time_ms)
    V=double(D.system_cycle_time_ms(:));
    CY=nan(n,1);CY(1:min(n,numel(V)))=V(1:min(n,numel(V)));
else
    CY=nan(n,1);
end
A=struct();
A.status='READ_ONLY_SENSOR_AND_TIMING_AUDIT';
A.records=n;A.sensor_records=sensor_records;
A.memory_records=memory_records;A.trigger_records=trigger_records;
A.sensor_valid_records=sensor_valid_records;
A.considered_id_records=considered_id_records;
A.sensor_invariants_passed=(sensor_records==n && sensor_valid_records==n && n>0);
A.memory_log_complete=(memory_records==n && n>0);
if memory_records==0
    A.memory_log_status='NOT_LOGGED__NOT_PROOF_OF_MEMORY_LOSS';
elseif memory_records==n
    A.memory_log_status='PRESENT_IN_ALL_RECORDED_FRAMES';
else
    A.memory_log_status='PARTIAL_MEMORY_LOG';
end
A.sensor_memory_invariants_passed=isempty(failures) && ...
    A.sensor_invariants_passed && A.memory_log_complete;
if A.sensor_memory_invariants_passed
    A.status='PASS_RECORDED_SENSOR_MEMORY_INVARIANTS__READ_ONLY';
elseif isempty(failures) && A.sensor_invariants_passed && memory_records==0
    A.status='PASS_SENSOR_INVARIANTS__MEMORY_LOG_UNAVAILABLE';
else
    A.status='INCONCLUSIVE_RECORDED_INVARIANTS__SEE_DETAILS';
end
A.failures=failures;A.final_observed_ids=cumulative;
A.timings=struct('cycle',dist(CY),'replanner',dist(RP), ...
    'sensor',dist(SE),'trigger',dist(TR), ...
    'predictor',dist(PR),'geometry',dist(GE));
A.stage_samples=nnz(all(isfinite(ST),2));
A.stage_names={'input','ekf_acquisition','sensor_memory', ...
               'trigger_plus_obstacle_log','logger_attachment','timing_bookkeeping'};
A.stage_stats=cell(1,6);
for j=1:6,A.stage_stats{j}=dist(ST(:,j));end
A.top_replanner_spikes=ranked(RP,T,CY,RP,TR,SE,PR,GE,ST,status,20);
A.top_trigger_spikes=ranked(TR,T,CY,RP,TR,SE,PR,GE,ST,status,20);
windows=[4.95 5.20;9.80 10.10;18 20;33 42];
W=struct('interval_s',{},'replanner_max_ms',{},'trigger_max_ms',{}, ...
    'sensor_max_ms',{},'predictor_max_ms',{},'geometry_max_ms',{});
for j=1:size(windows,1)
    mask=T>=windows(j,1) & T<=windows(j,2);
    W(j)=struct('interval_s',windows(j,:), ...
        'replanner_max_ms',max_or_nan(RP(mask)), ...
        'trigger_max_ms',max_or_nan(TR(mask)), ...
        'sensor_max_ms',max_or_nan(SE(mask)), ...
        'predictor_max_ms',max_or_nan(PR(mask)), ...
        'geometry_max_ms',max_or_nan(GE(mask)));
end
A.windows=W;
A.preflight=[];
A.sensor_scan_count_matches_logger=[];
if nargin>=2 && ~isempty(replanner_instance)
    try
        q=replanner_instance.get_status();
        A.preflight=struct('mode',q.preflight_update_mode, ...
            'ready',q.preflight_update_ready,'elapsed_ms',q.preflight_update_ms);
        if isfield(q,'sensor_status') && isstruct(q.sensor_status)
            A.reported_sensor_scan_count=q.sensor_status.scan_count;
            A.sensor_scan_count_matches_logger=isequal(double(q.sensor_status.scan_count),double(sensor_records));
        end
    catch
        A.preflight='UNAVAILABLE';
    end
end
root=fileparts(mfilename('fullpath'));
stem='TRIGGER_SENSOR_SPIKE_AUDIT';
if nargin>=3 && ~isempty(report_tag)
    if ~(ischar(report_tag)||(isstring(report_tag)&&isscalar(report_tag)))
        error('CHECK_TRIGGER_SENSOR_SPIKE_AUDIT:ReportTag', ...
            'Report suffix must be a text scalar.');
    end
    report_tag=char(report_tag);
    if isempty(regexp(report_tag,'^[A-Za-z0-9_]+$','once'))
        error('CHECK_TRIGGER_SENSOR_SPIKE_AUDIT:ReportTag', ...
            'Use only letters, digits, and underscores in the report suffix.');
    end
    stem=[stem '_' report_tag];
end
A.output_mat_file=fullfile(root,[stem '.mat']);
A.output_txt_file=fullfile(root,[stem '.txt']);
save(A.output_mat_file,'A','-v7');
fid=fopen(A.output_txt_file,'wt');
if fid>=0
    cleaner=onCleanup(@()fclose(fid)); %#ok<NASGU>
    fprintf(fid,'STATUS=%s\nrecords=%d sensor=%d memory=%d trigger=%d\n', ...
        A.status,A.records,sensor_records,memory_records,trigger_records);
    fprintf(fid,'sensor_invariants_passed=%d valid=%d/%d\n', ...
        A.sensor_invariants_passed,A.sensor_valid_records,n);
    fprintf(fid,'memory_log_status=%s checked=%d/%d\n', ...
        A.memory_log_status,A.memory_records,n);
    fprintf(fid,'trigger_considered_id_records=%d\n',A.considered_id_records);
    fprintf(fid,'sensor_memory_invariants_passed=%d failures=%d\n', ...
        A.sensor_memory_invariants_passed,numel(failures));
    fprintf(fid,'final_observed_ids=%s\n',mat2str(cumulative));
    if ~isempty(A.sensor_scan_count_matches_logger)
        fprintf(fid,'sensor_scan_count_matches_logger=%d\n',A.sensor_scan_count_matches_logger);
    end
    names=fieldnames(A.timings);
    for j=1:numel(names)
        x=A.timings.(names{j});
        fprintf(fid,'%s count=%d median=%.6f p95=%.6f max=%.6f ms\n', ...
            names{j},x.count,x.median_ms,x.p95_ms,x.max_ms);
    end
    fprintf(fid,'stage_timing_records=%d\n',A.stage_samples);
    for j=1:6
        x=A.stage_stats{j};
        fprintf(fid,'stage_%d_%s median=%.6f p95=%.6f max=%.6f ms\n', ...
            j,A.stage_names{j},x.median_ms,x.p95_ms,x.max_ms);
    end
    fprintf(fid,'TOP_REPLANNER_SPIKES time replanner trigger sensor predictor geometry cycle stage6 status\n');
    for j=1:numel(A.top_replanner_spikes)
        q=A.top_replanner_spikes(j);
        fprintf(fid,'%.6f %.6f %.6f %.6f %.6f %.6f %.6f %s %s\n', ...
            q.time_s,q.replanner_ms,q.trigger_ms,q.sensor_ms, ...
            q.predictor_ms,q.geometry_ms,q.cycle_ms,mat2str(q.stage_ms,4),q.trigger_status);
    end
    fprintf(fid,'WINDOWS [start end] replanner trigger sensor predictor geometry\n');
    for j=1:numel(W)
        q=W(j);
        fprintf(fid,'%s %.6f %.6f %.6f %.6f %.6f\n',mat2str(q.interval_s), ...
            q.replanner_max_ms,q.trigger_max_ms,q.sensor_max_ms, ...
            q.predictor_max_ms,q.geometry_max_ms);
    end
    for j=1:numel(failures)
        q=failures(j);
        fprintf(fid,'INVARIANT_FAIL index=%d t=%.6f kind=%s reason=%s\n', ...
            q.index,q.time_s,q.kind,q.details);
    end
end
fprintf('[TRIGGER SENSOR AUDIT] frames=%d sensor=%d; invariant_failures=%d; stage_samples=%d\n', ...
    n,sensor_records,numel(failures),A.stage_samples);
end

function y=scalar_number(x)
y=NaN;
if isnumeric(x) && isscalar(x) && isreal(x) && isfinite(x),y=double(x);end
end
function ids=checked_ids(x)
% The sensor intentionally writes [] for an empty detected/new/left ID set.
% In MATLAB, isvector([]) can reject the 0-by-0 empty value.  Empty is a
% valid numeric ID set and must NOT invalidate a Flight frame.
assert(isnumeric(x) && isreal(x) && ndims(x)<=2, ...
    'CHECK_TRIGGER_SENSOR_SPIKE_AUDIT:IDs','ID set must be a real numeric array.');
if isempty(x)
    ids=zeros(1,0);
    return
end
assert(isvector(x) && all(isfinite(x(:))) && ...
    all(x(:)==fix(x(:))) && numel(unique(x(:)))==numel(x(:)), ...
    'CHECK_TRIGGER_SENSOR_SPIKE_AUDIT:IDs','Invalid nonempty ID vector.');
ids=reshape(double(x),1,[]);
end
function o=dist(x)
x=sort(double(x(isfinite(x))));
o=struct('count',numel(x),'median_ms',NaN,'p95_ms',NaN,'max_ms',NaN);
if ~isempty(x)
    o.median_ms=median(x);o.p95_ms=x(max(1,ceil(.95*numel(x))));o.max_ms=x(end);
end
end
function v=max_or_nan(x)
x=x(isfinite(x));
if isempty(x),v=NaN;else,v=max(x);end
end
function out=ranked(metric,t,cy,rp,tr,se,pr,ge,st,status,limit)
ids=find(isfinite(metric));[~,ix]=sort(metric(ids),'descend');
ids=ids(ix(1:min(limit,numel(ix))));
out=struct('time_s',{},'replanner_ms',{},'trigger_ms',{}, ...
           'sensor_ms',{},'predictor_ms',{},'geometry_ms',{}, ...
           'cycle_ms',{},'stage_ms',{},'trigger_status',{});
for j=1:numel(ids)
    k=ids(j);
    out(j)=struct('time_s',t(k),'replanner_ms',rp(k), ...
        'trigger_ms',tr(k),'sensor_ms',se(k),'predictor_ms',pr(k), ...
        'geometry_ms',ge(k),'cycle_ms',cy(k), ...
        'stage_ms',st(k,:),'trigger_status',status{k});
end
end

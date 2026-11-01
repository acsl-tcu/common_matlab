function R = CHECK_SENSOR_FRESH_EVENT_SPIKES(logger)
% CHECK_SENSOR_FRESH_EVENT_SPIKES
% Fresh-sensor versus repeated identical sensor-state transitions.
% This is an offline, read-only analysis using the normal Sim LOGGER.
% It does not modify REPLANNING_SYSTEM, the production sensor, HLC, EKF,
% plant, run_flight_headless or the supplied logger.
%
% Invoke after an ordinary Flight:
%   R = CHECK_SENSOR_FRESH_EVENT_SPIKES(logger);
%   R.status
%   R.results
%
% The test compares (A) each event's original logged sensor time,
% (B) the first execution after a fresh sensor has replayed the earlier
% Flight history, and (C) repetitions of the exact preceding/event poses.
% Results are diagnostic ONLY, not worst-case execution time guarantees.

if nargin ~= 1 || ~isobject(logger) || ~isprop(logger, 'Data') || ...
        ~isprop(logger, 'k')
    error('CHECK_SENSOR_FRESH_EVENT_SPIKES:Logger', ...
        'Use CHECK_SENSOR_FRESH_EVENT_SPIKES(logger) with the original Sim LOGGER.');
end
if ~isfield(logger.Data, 'agent') || ...
        ~isfield(logger.Data.agent(1), 'reference') || ...
        ~isfield(logger.Data.agent(1).reference, 'result')
    error('CHECK_SENSOR_FRESH_EVENT_SPIKES:Layout', ...
        'Sim LOGGER reference.result field is missing.');
end
N = double(logger.k);
if N < 3 || ~isfinite(N) || N ~= fix(N)
    error('CHECK_SENSOR_FRESH_EVENT_SPIKES:Count', 'Invalid LOGGER record count.');
end
source = which('SIMULATED_OBSTACLE_SENSOR');
if isempty(source)
    error('CHECK_SENSOR_FRESH_EVENT_SPIKES:Sensor', 'Sensor class not on MATLAB path.');
end
obstacles = OBSTACLE_INFORMATION.definitions();
ref = logger.Data.agent(1).reference.result;
T = zeros(N,1);
P = zeros(3,N);
Range = zeros(N,1);
OriginalMs = nan(N,1);
truthIDs = cell(N,1);
truthNew = cell(N,1);
truthLeft = cell(N,1);
for i = 1:N
    if iscell(ref), rr = ref{i}; else, rr = ref(i); end
    if ~isstruct(rr) || ~isfield(rr,'simulated_obstacle_sensor')
        error('CHECK_SENSOR_FRESH_EVENT_SPIKES:Record', ...
            'Missing sensor record at index %d.', i);
    end
    q = rr.simulated_obstacle_sensor;
    T(i) = double(q.time_s);
    P(:,i) = double(q.uav_position_ekf_m(:));
    Range(i) = double(q.sensor_range_m);
    OriginalMs(i) = double(q.elapsed_ms);
    truthIDs{i} = q.detected_ids;
    truthNew{i} = q.newly_detected_ids;
    truthLeft{i} = q.left_range_ids;
end
isEvent = false(N,1);
for i = 2:N
    isEvent(i) = ~isempty(truthNew{i}) || ~isempty(truthLeft{i});
end
indices = find(isEvent);
if isempty(indices)
    error('CHECK_SENSOR_FRESH_EVENT_SPIKES:Events', ...
        'The Flight contains no detection events.');
end

freshRepeats = 4;  % independent fresh sensor instances, not 4 Flight runs
warmRepeats = 12;  % two-pose transitions using the same instance
nE = numel(indices);
first = nan(nE,2,freshRepeats);
warm = nan(nE,2,freshRepeats,warmRepeats);
prev = nan(nE,2,freshRepeats,warmRepeats);
errorCount = 0;
firstErrors = cell(0,1);
start = tic;
for e = 1:nE
    idx = indices(e);
    for flag = 0:1
        for rep = 1:freshRepeats
            ss = SIMULATED_OBSTACLE_SENSOR(obstacles,Range(1),logical(flag));
            for k = 1:idx-1
                if ss.range_m ~= Range(k)
                    ss.set_range(Range(k));
                end
                ss.observe(T(k),P(:,k));
            end
            if ss.range_m ~= Range(idx), ss.set_range(Range(idx)); end
            w = tic;
            observed = ss.observe(T(idx),P(:,idx));
            first(e,flag+1,rep) = 1000*toc(w);
            if ~matches(observed,idx)
                errorCount = errorCount+1;
                firstErrors{end+1,1} = sprintf( ...
                    'event=%d mode=%d fresh=%d (first event output mismatch)',... 
                    idx,flag,rep); %#ok<AGROW>
            end
            for r = 1:warmRepeats
                if ss.range_m ~= Range(idx-1), ss.set_range(Range(idx-1)); end
                w=tic;
                ss.observe(T(idx-1),P(:,idx-1));
                prev(e,flag+1,rep,r) = 1000*toc(w);
                if ss.range_m ~= Range(idx), ss.set_range(Range(idx)); end
                w=tic;
                observed = ss.observe(T(idx),P(:,idx));
                warm(e,flag+1,rep,r) = 1000*toc(w);
                if ~matches(observed,idx)
                    errorCount = errorCount+1;
                    if numel(firstErrors) < 24
                        firstErrors{end+1,1} = sprintf( ...
                            'event=%d mode=%d fresh=%d repeated=%d (output mismatch)', ...
                            idx,flag,rep,r); %#ok<AGROW>
                    end
                end
            end
        end
    end
end

rows=nan(nE,9);
for e=1:nE
    idx=indices(e);
    a=reshape(first(e,1,:),[],1);
    b=reshape(first(e,2,:),[],1);
    c=reshape(warm(e,1,:,:),[],1);
    d=reshape(warm(e,2,:,:),[],1);
    p=reshape(prev(e,2,:,:),[],1);
    rows(e,:)=[T(idx),OriginalMs(idx),median(a),max(a), ...
        median(b),max(b),median(c),median(d),median(p)];
end
R=struct();
R.status='PASS_FRESH_TRANSITION_REPLAY__CAUSE_NOT_PROVEN';
if errorCount>0, R.status='FAIL_REPLAY_OUTPUT_EQUIVALENCE'; end
R.source=source;
R.records=N;
R.event_indices=indices;
R.event_count=nE;
R.first_instance_count=freshRepeats;
R.warm_repeats_per_instance=warmRepeats;
R.columns={'time_s','original_flight_sensor_ms','fresh_no_timing_median_ms', ...
    'fresh_no_timing_max_ms','fresh_timing_median_ms', ...
    'fresh_timing_max_ms','repeat_no_timing_median_ms', ...
    'repeat_timing_median_ms','previous_pose_timing_median_ms'};
R.results=rows;
R.error_count=errorCount;
R.error_examples=firstErrors;
R.elapsed_audit_s=toc(start);
R.note=['All replay measurements include the function call and last_result write. ', ...
    'The Flight elapsed_ms excludes that write. ', ...
    'A single MATLAB session cannot isolate all JIT or OS effects. ', ...
    'This test does not certify a hard real-time bound.'];
root=fileparts(mfilename('fullpath'));
if isempty(root),root=pwd;end
save(fullfile(root,'SENSOR_FRESH_EVENT_AUDIT.mat'),'R');
fid=fopen(fullfile(root,'SENSOR_FRESH_EVENT_AUDIT.txt'),'wt');
if fid < 0
    error('CHECK_SENSOR_FRESH_EVENT_SPIKES:Output', 'Unable to write audit TXT.');
end
cleanup=onCleanup(@()fclose(fid));
fprintf(fid,'%s\n',R.status);
fprintf(fid,'sensor=%s\n',source);
fprintf(fid,'records=%d events=%d fresh_instances_per_log_mode=%d repetitions=%d error_count=%d\n', ...
    N,nE,freshRepeats,warmRepeats,errorCount);
fprintf(fid,'columns=%s\n',strjoin(R.columns,' | '));
for e=1:nE
    fprintf(fid,'%s\n',sprintf('%.6f ',rows(e,:)));
end
for k=1:numel(firstErrors), fprintf(fid,'ERROR %s\n',firstErrors{k});end
fprintf(fid,'NOTE %s\n',R.note);
fprintf('[FRESH EVENT] %s | events=%d | mismatches=%d\n',R.status,nE,errorCount);

    function tf=matches(q,k)
        tf=isequaln(q.detected_ids,truthIDs{k}) && ...
            isequaln(q.newly_detected_ids,truthNew{k}) && ...
            isequaln(q.left_range_ids,truthLeft{k});
    end
end

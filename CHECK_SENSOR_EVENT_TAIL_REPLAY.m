function R=CHECK_SENSOR_EVENT_TAIL_REPLAY(logger)
% CHECK_SENSOR_EVENT_TAIL_REPLAY
% Offline only: compare first genuine event after one previous pose,
% repeated genuine events, and identical-pose non-events on NEW sensor
% instances. No Sim/Flight/HLC/EKF/controller or production source edits.
%
% Run after Flight:
%   R=CHECK_SENSOR_EVENT_TAIL_REPLAY(logger);
%   R.status
%   R.summary
%
% This is a *distributional* diagnostic, not a WCET certificate.

if nargin~=1 || ~isobject(logger) || ~isprop(logger,'Data') || ~isprop(logger,'k')
    error('CHECK_SENSOR_EVENT_TAIL_REPLAY:Logger', ...
        'Pass the normal Sim LOGGER: CHECK_SENSOR_EVENT_TAIL_REPLAY(logger).');
end
if ~isfield(logger.Data,'agent') || ~isfield(logger.Data.agent(1),'reference')
    error('CHECK_SENSOR_EVENT_TAIL_REPLAY:Layout','LOGGER agent.reference not found.');
end
N=double(logger.k);
refs=logger.Data.agent(1).reference.result;
if isempty(refs) || N<3
    error('CHECK_SENSOR_EVENT_TAIL_REPLAY:Empty','Need completed Flight LOGGER.');
end
T=zeros(N,1); P=zeros(3,N); ranges=zeros(N,1);
newIds=cell(N,1); leftIds=cell(N,1); logMs=nan(N,1);
rec=cell(N,1);
for i=1:N
    if iscell(refs), row=refs{i}; else, row=refs(i); end
    if ~isstruct(row) || ~isfield(row,'simulated_obstacle_sensor')
        error('CHECK_SENSOR_EVENT_TAIL_REPLAY:Record','No sensor record at %d.',i);
    end
    q=row.simulated_obstacle_sensor;
    rec{i}=q;
    T(i)=double(q.time_s);
    P(:,i)=double(q.uav_position_ekf_m(:));
    ranges(i)=double(q.sensor_range_m);
    newIds{i}=q.newly_detected_ids;
    leftIds{i}=q.left_range_ids;
    logMs(i)=double(q.elapsed_ms);
end
indices=[];
for i=2:N
    if ~isempty(newIds{i}) || ~isempty(leftIds{i})
        indices(end+1)=i; %#ok<AGROW>
    end
end
if isempty(indices)
    error('CHECK_SENSOR_EVENT_TAIL_REPLAY:Events','No sensor transition found in Flight.');
end
obstacles=OBSTACLE_INFORMATION.definitions();
trials=40; ne=numel(indices);
% Columns: first event, same-pose no event, second event, previous pose.
raw=nan(ne,trials,4);
errors=0; firstErrors={};
for k=1:trials
    % Cycle through events to avoid always beginning with the same event.
    for offset=1:ne
        e=mod(offset+k-2,ne)+1;
        ix=indices(e);
        s=SIMULATED_OBSTACLE_SENSOR(obstacles,ranges(ix-1),true);
        % Only the actual previous pose. The *first* transition on this
        % instance is the logged target event, not the end of a replay of
        % all earlier events. The original Flight sample is only a truth
        % source and is not injected into the new sensor.
        w=tic;
        s.observe(T(ix-1),P(:,ix-1));
        raw(e,k,4)=1000*toc(w);
        if s.range_m~=ranges(ix),s.set_range(ranges(ix));end
        w=tic;
        a=s.observe(T(ix),P(:,ix));
        raw(e,k,1)=1000*toc(w);
        if ~sensor_equal_without_time(a,rec{ix})
            errors=errors+1;
            if numel(firstErrors)<16
                firstErrors{end+1}=sprintf('event=%d trial=%d first mismatch',ix,k); %#ok<AGROW>
            end
        end
        % Same pose; no new/left event by design.
        w=tic;
        b=s.observe(T(ix),P(:,ix));
        raw(e,k,2)=1000*toc(w);
        if ~isempty(b.newly_detected_ids)||~isempty(b.left_range_ids)
            errors=errors+1;
            if numel(firstErrors)<16
                firstErrors{end+1}=sprintf('event=%d trial=%d same-pose event',ix,k); %#ok<AGROW>
            end
        end
        if s.range_m~=ranges(ix-1),s.set_range(ranges(ix-1));end
        s.observe(T(ix-1),P(:,ix-1));
        if s.range_m~=ranges(ix),s.set_range(ranges(ix));end
        w=tic;
        c=s.observe(T(ix),P(:,ix));
        raw(e,k,3)=1000*toc(w);
        if ~sensor_equal_without_time(c,rec{ix})
            errors=errors+1;
            if numel(firstErrors)<16
                firstErrors{end+1}=sprintf('event=%d trial=%d repeated mismatch',ix,k); %#ok<AGROW>
            end
        end
    end
end
summary=nan(ne,17);
% time, Flight_ms, [first, no-event, repeat] median/p95/max,
% first >0.1/0.5/1ms counts, no-event >0.5 count,
% repeat >0.5 count, first minus same-pose median.
for e=1:ne
    ix=indices(e);
    a=reshape(raw(e,:,1),[],1);
    b=reshape(raw(e,:,2),[],1);
    c=reshape(raw(e,:,3),[],1);
    summary(e,:)=[T(ix),logMs(ix),median(a),prctile(a,95),max(a), ...
        median(b),prctile(b,95),max(b),median(c),prctile(c,95),max(c), ...
        sum(a>0.1),sum(a>0.5),sum(a>1),sum(b>0.5),sum(c>0.5),median(a-b)];
end
R=struct();
R.status='PASS_EVENT_TAIL_REPLAY__NOT_HARD_REALTIME_PROOF';
if errors>0,R.status='FAIL_EVENT_REPLAY_EQUIVALENCE';end
R.error_count=errors;
R.error_examples=firstErrors;
R.trials_per_event=trials;
R.event_indices=indices;
R.columns={'t_s','flight_sensor_ms','first_median','first_p95','first_max', ...
    'same_pose_median','same_pose_p95','same_pose_max', ...
    'repeat_median','repeat_p95','repeat_max', ...
    'first_over_0p1','first_over_0p5','first_over_1p0', ...
    'same_pose_over_0p5','repeat_over_0p5','paired_first_minus_same_median'};
R.summary=summary;
R.raw_times_ms=raw;
R.notes=['The original Flight elapsed_ms stops before last_result assignment; ', ...
    'all replay timings include the full observe() call. ', ...
    'The test is in one already-running MATLAB process; it cannot ', ...
    'exclude OS scheduling, JIT, or memory-management pauses. ', ...
    'All geometry remains inside the offline sensor; no unseen obstacle ', ...
    'is sent to the replanner.'];
root=fileparts(mfilename('fullpath'));
if isempty(root),root=pwd;end
save(fullfile(root,'SENSOR_EVENT_TAIL_REPLAY_AUDIT.mat'),'R');
fid=fopen(fullfile(root,'SENSOR_EVENT_TAIL_REPLAY_AUDIT.txt'),'wt');
if fid<0,error('CHECK_SENSOR_EVENT_TAIL_REPLAY:Write','Cannot open report.');end
cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s\n',R.status);
fprintf(fid,'Flight N=%d | events=%d | fresh sensor instances per event=%d | errors=%d\n',N,ne,trials,errors);
fprintf(fid,'cols: %s\n',strjoin(R.columns,' | '));
for e=1:ne
    fprintf(fid,'%s\n',sprintf('%.7f ',summary(e,:)));
end
for i=1:numel(firstErrors),fprintf(fid,'ERROR %s\n',firstErrors{i});end
fprintf(fid,'NOTE: %s\n',R.notes);
fprintf('[EVENT TAIL] %s | %d events | %d trials each | errors=%d\n', ...
    R.status,ne,trials,errors);
end

function same=sensor_equal_without_time(a,b)
% Ensure exact output contract, except the two timing-specific fields.
a=rmfield(a,'elapsed_ms');
b=rmfield(b,'elapsed_ms');
same=isequaln(a,b);
end

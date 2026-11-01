function R = CHECK_REPLANNER_OFFLINE_CAUSAL(logger, repeats)
% CHECK_REPLANNER_OFFLINE_CAUSAL  Flight-log-driven offline causal contrasts.
% This diagnostic DOES NOT edit production sources or the supplied LOGGER.
% It uses fresh REPLANNING_SYSTEM instances and an independent mock agent.
%   R = CHECK_REPLANNER_OFFLINE_CAUSAL(logger);
%   R.summary
%   R.event_comparison
%   R.top_flight
% Writes TXT/MAT alongside this diagnostic; no new Flight is required.
%
% Timing is an observation, NOT a WCET guarantee. Replay lacks Sim's other
% concurrent work, so failure to reproduce a spike is itself informative.

if nargin < 2 || isempty(repeats), repeats=3; end
if ~(isnumeric(repeats) && isscalar(repeats) && isfinite(repeats) && ...
        repeats>=1 && repeats<=12 && repeats==fix(repeats))
    error('CHECK_REPLANNER_OFFLINE_CAUSAL:Repeats','repeats must be an integer 1..12.');
end
if ~isobject(logger) || ~isprop(logger,'Data') || ~isprop(logger,'k')
    error('CHECK_REPLANNER_OFFLINE_CAUSAL:Logger','Use the original Sim LOGGER object.');
end
if ~isstruct(logger.Data) || ~isfield(logger.Data,'agent') || ...
        ~isfield(logger.Data.agent(1),'reference') || ...
        ~isfield(logger.Data.agent(1).reference,'result')
    error('CHECK_REPLANNER_OFFLINE_CAUSAL:Layout', ...
        'Expected logger.Data.agent(1).reference.result.');
end
N=double(logger.k);
if ~(isscalar(N) && isfinite(N) && N==fix(N) && N>=3)
    error('CHECK_REPLANNER_OFFLINE_CAUSAL:Count','Invalid LOGGER count.');
end
if isempty(which('REPLANNING_SYSTEM')) || ...
        isempty(which('SIMULATED_OBSTACLE_SENSOR')) || ...
        isempty(which('OBSTACLE_INFORMATION'))
    error('CHECK_REPLANNER_OFFLINE_CAUSAL:Path','Production replanner/sensor/definitions not found on MATLAB path.');
end
% The current sensor constructor can auto-compile a missing native binary.
% This read-only diagnostic MUST NOT trigger such a file write.
sensorFile=which('SIMULATED_OBSTACLE_SENSOR');
sensorDirectory=fileparts(sensorFile);
expectedMex=fullfile(sensorDirectory,['SENSOR_EVENT_IDS_NOHEAP_NATIVE.' mexext]);
if ~isfile(expectedMex)
    error('CHECK_REPLANNER_OFFLINE_CAUSAL:ReadOnly', ...
        ['Current sensor would auto-build a missing MEX: %s. ', ...
         'For a strict no-change audit, stop here.'],expectedMex);
end
raw=logger.Data.agent(1).reference.result;
T=zeros(N,1);P=zeros(3,N);Range=zeros(N,1);
flightSensor=nan(N,1);flightUpdate=nan(N,1);
flightCand=nan(N,1); flightDetected=zeros(N,1);
flightNew=false(N,1);flightLeft=false(N,1);
truth=cell(N,1);base=cell(N,1);
for i=1:N
    if iscell(raw), rr=raw{i}; else, rr=raw(i); end
    if ~isstruct(rr) || ~isscalar(rr) || ...
            ~isfield(rr,'simulated_obstacle_sensor') || ...
            ~isfield(rr,'replanning_time_ms')
        error('CHECK_REPLANNER_OFFLINE_CAUSAL:Record','Incomplete LOGGER record %d.',i);
    end
    q=rr.simulated_obstacle_sensor;
    T(i)=double(q.time_s);P(:,i)=double(q.uav_position_ekf_m(:));
    Range(i)=double(q.sensor_range_m);
    flightSensor(i)=double(q.elapsed_ms);
    flightUpdate(i)=double(rr.replanning_time_ms);
    flightCand(i)=double(q.candidates_checked);
    flightDetected(i)=double(q.detected_count);
    flightNew(i)=~isempty(q.newly_detected_ids);
    flightLeft(i)=~isempty(q.left_range_ids);
    truth{i}=q;
    remove=intersect(fieldnames(rr), ...
        {'simulated_obstacle_sensor','obstacle_information','replanning_time_ms'},'stable');
    if ~isempty(remove), rr=rmfield(rr,remove); end
    base{i}=rr;  % original reference fields, with replanner-owned fields omitted
end
% Current flight's time/range/IDs come only from its LOGGER.
if any(~isfinite(T)) || any(~isfinite(P(:))) || ...
        any(~isfinite(Range)) || any(Range<=0)
    error('CHECK_REPLANNER_OFFLINE_CAUSAL:Data','Nonfinite Flight input.');
end
obstacles=OBSTACLE_INFORMATION.definitions();
% Do not infer a fixed obstacle count, a fixed set of detections, or future
% detections from the replay. Definitions are used ONLY by the offline sensor.
firstIds=[obstacles.id];
for i=1:N
    got=truth{i}.detected_ids;
    if any(~ismember(got,firstIds))
        error('CHECK_REPLANNER_OFFLINE_CAUSAL:Definitions', ...
            'Flight ID missing from CURRENT obstacle definitions at record %d.',i);
    end
end
isEvent=flightNew|flightLeft;
isEvent(1)=false; % first log row is initial state, not a transition
isAfter=false(N,1); isAfter(2:end)=isEvent(1:end-1);
isWindow=isEvent|isAfter;
normal=~isWindow;
target=.4; % analysis threshold, NOT a controller/physical parameter
summaryLabels={'event','next_tick','other','zero_candidate','zero_detected','all'};
summaryMasks={isEvent,isAfter,normal,flightCand==0,flightDetected==0,true(N,1)};
summary=nan(numel(summaryMasks),9);
for j=1:numel(summaryMasks)
    ix=summaryMasks{j};
    if ~any(ix)
        summary(j,:)=[0,NaN,NaN,0,NaN,NaN,0,0,0];
    else
        summary(j,:)=[sum(ix),median(flightSensor(ix)),max(flightSensor(ix)), ...
            sum(flightSensor(ix)>target),median(flightUpdate(ix)), ...
            max(flightUpdate(ix)),sum(flightUpdate(ix)>target), ...
            sum((flightUpdate(ix)-flightSensor(ix))>target),sum(flightCand(ix)==0)];
    end
end
% Replay modes. All source code remains untouched. "sensor/log off" means
% only fresh OFFLINE replanner instances; the real Flight was already logged.
modeNames={'FULL_UPDATE','NO_OBSTACLE_LOG','NO_SENSOR','CORE_ONLY', ...
    'SENSOR_MEX_OFF','SENSOR_MEX_ON'};
nModes=numel(modeNames);
wall=nan(N,nModes,repeats);
internal=nan(N,nModes,repeats);
packSensor=nan(N,nModes,repeats);
modeAvailable=true(1,nModes);
sourceStatus=cell(nModes,repeats);
checkErrors=cell(0,1);
checks=0;
maxErrors=30;
auditStart=tic;

% Move execution order across repeats to reduce fixed-position warmup bias.
for rep=1:repeats
    if mod(rep,2)==1
        order=[1,5,2,6,3,4];
    else
        order=[4,3,6,2,5,1];
    end
    for vv=1:numel(order)
        m=order(vv);
        if m<=4
            withSensor=ismember(m,[1 2]);
            withObsLog=ismember(m,[1 3]);
            opts=struct('SENSOR_RANGE_M',Range(1), ...
                'ENABLE_ALL_LOG',true,'LOG_OBSTACLES',withObsLog, ...
                'ENABLE_SENSOR',withSensor,'LOG_SENSOR',withSensor, ...
                'LOG_REPLANNER',true);
            repl=REPLANNING_SYSTEM(opts);  % a new, isolated object
            a=SPIKE_OFFLINE_AGENT(base{1});
            % Preserve the original Arming-before-Flight warmup semantics,
            % but keep everything on these isolated offline objects.
            if ismethod(repl,'preflight_update')
                a.set_sample(P(:,1),base{1});
                repl.preflight_update(struct('t',T(1)),a);
            end
            initialStatus=repl.get_status();
            sourceStatus{m,rep}=initialStatus.sensor_status.native_event_pack_build_status;
            for i=1:N
                if repl.SENSOR_RANGE_M~=Range(i)
                    repl.set_sensor_range(Range(i));
                end
                a.set_sample(P(:,i),base{i});
                ti=struct('t',T(i));
                clk=tic;
                repl.update(ti,a);  % actual unmodified production update()
                wall(i,m,rep)=1000*toc(clk); % incl. final property write
                out=a.reference.result;
                internal(i,m,rep)=double(out.replanning_time_ms);
                if withSensor
                    q=out.simulated_obstacle_sensor;
                    packSensor(i,m,rep)=double(q.elapsed_ms);
                    if rep==1 && m==1
                        checks=checks+1;
                        q.elapsed_ms=NaN;
                        actual=truth{i}; actual.elapsed_ms=NaN;
                        if ~isequaln(q,actual) && numel(checkErrors)<maxErrors
                            checkErrors{end+1,1}=sprintf( ...
                                'FULL output mismatch record=%d t=%.9f',i,T(i)); %#ok<AGROW>
                        end
                    end
                end
            end
        else
            ss=SIMULATED_OBSTACLE_SENSOR(obstacles,Range(1),true);
            st=ss.get_status();
            if m==6 && ~st.native_event_pack_available
                modeAvailable(m)=false;
                sourceStatus{m,rep}='MEX_UNAVAILABLE';
                continue
            end
            ss.set_native_event_pack(m==6);
            sourceStatus{m,rep}=st.native_event_pack_build_status;
            for i=1:N
                if ss.range_m~=Range(i),ss.set_range(Range(i));end
                clk=tic;
                q=ss.observe(T(i),P(:,i)); % real sensor, no Sim/LOGGER
                wall(i,m,rep)=1000*toc(clk);
                packSensor(i,m,rep)=double(q.elapsed_ms);
                if rep==1 && m==5
                    checks=checks+1;
                    q.elapsed_ms=NaN;
                    actual=truth{i}; actual.elapsed_ms=NaN;
                    if ~isequaln(q,actual) && numel(checkErrors)<maxErrors
                        checkErrors{end+1,1}=sprintf( ...
                            'DIRECT output mismatch record=%d t=%.9f',i,T(i)); %#ok<AGROW>
                    end
                end
            end
        end
    end
end

% All Flight spikes are preserved; replay speeds aren't substituted for them.
summaryReplay=nan(nModes,7);
for m=1:nModes
    values=reshape(wall(:,m,:),[],1);
    values=values(isfinite(values));
    if isempty(values),continue;end
    summaryReplay(m,:)=[numel(values),median(values),prctile_local(values,95), ...
        max(values),sum(values>target), ...
        median(values(values<=prctile_local(values,99))), ...
        sum(values>1)];
end
idx=find(isEvent);
eventTable=nan(numel(idx),13);
for j=1:numel(idx)
    i=idx(j);
    full=reshape(wall(i,1,:),[],1);
    directOff=reshape(wall(i,5,:),[],1);
    directOn=reshape(wall(i,6,:),[],1);
    post=i+1;
    if post>N, post=i;end
    eventTable(j,:)=[i,T(i),flightSensor(i),flightUpdate(i), ...
        flightSensor(post),flightUpdate(post),flightCand(i), ...
        median(full,'omitnan'),max(full), ...
        median(directOff,'omitnan'),max(directOff), ...
        median(directOn,'omitnan'),max(directOn)];
end
[~,topIdx]=sort(flightUpdate,'descend');
topIdx=topIdx(1:min(20,N));
topRows=[topIdx,T(topIdx),double(isEvent(topIdx)),double(isAfter(topIdx)), ...
    flightCand(topIdx),flightDetected(topIdx),flightSensor(topIdx), ...
    flightUpdate(topIdx),flightUpdate(topIdx)-flightSensor(topIdx)];

R=struct();
R.status='PASS_OFFLINE_REPLAY__CAUSE_NOT_PROVEN';
if ~isempty(checkErrors),R.status='FAIL_REPLAY_OUTPUT_EQUIVALENCE';end
R.sensor_source=which('SIMULATED_OBSTACLE_SENSOR');
R.replanner_source=which('REPLANNING_SYSTEM');
R.definition_source=which('OBSTACLE_INFORMATION');
R.flight_records=N;
R.obstacle_count=numel(obstacles);
R.flight_events=idx;
R.flight_event_count=numel(idx);
R.target_ms=target;
R.summary_labels=summaryLabels;
R.summary_columns={'records','sensor_median','sensor_max','sensor_over_0p4', ...
    'update_median','update_max','update_over_0p4','other_proxy_over_0p4','zero_candidate_count'};
R.summary=summary;
R.mode_names=modeNames;
R.mode_available=modeAvailable;
R.mode_status=sourceStatus;
R.replay_columns={'samples','median_outer_ms','p95_outer_ms','max_outer_ms', ...
    'count_over_0p4','trimmed_median_ms','count_over_1p0'};
R.replay_summary=summaryReplay;
R.event_columns={'record','time_s','flight_sensor_ms','flight_update_ms', ...
    'next_sensor_ms','next_update_ms','candidates', ...
    'replay_full_median','replay_full_max', ...
    'direct_matlab_median','direct_matlab_max', ...
    'direct_mex_median','direct_mex_max'};
R.event_comparison=eventTable;
R.top_columns={'record','time_s','event','after_event','candidates', ...
    'detected','sensor_ms','update_ms','update_minus_sensor_proxy_ms'};
R.top_flight=topRows;
R.replay_wall_ms=wall;
R.replay_internal_update_ms=internal;
R.replay_sensor_logged_ms=packSensor;
R.checks=checks;
R.errors=numel(checkErrors);
R.error_examples=checkErrors;
R.wall_elapsed_s=toc(auditStart);
R.notes=['Offline playback uses new replanner/sensor instances, a mock agent ', ...
    'and exactly the logged EKF position/time/range. Flight scheduler, ', ...
    'Sim LOGGER, EKF execution, HLC and plant are NOT replayed. ', ...
    'Counters and time samples can differ. An event-local concentration ', ...
    'is a correlation, not a diagnosis of MATLAB/JIT/OS internals. ', ...
    'DO NOT use measurements as a worst-case execution-time guarantee.'];

folder=fileparts(mfilename('fullpath'));
if isempty(folder),folder=pwd;end
save(fullfile(folder,'REPLANNER_OFFLINE_CAUSAL_AUDIT.mat'),'R');
fid=fopen(fullfile(folder,'REPLANNER_OFFLINE_CAUSAL_AUDIT.txt'),'wt');
if fid<0,error('CHECK_REPLANNER_OFFLINE_CAUSAL:File','Cannot create audit TXT.');end
clean=onCleanup(@()fclose(fid));
fprintf(fid,'%s\n',R.status);
fprintf(fid,'records=%d obstacles=%d events=%d repeats=%d checks=%d errors=%d\n', ...
    N,numel(obstacles),numel(idx),repeats,checks,R.errors);
fprintf(fid,'sensor=%s\nupdate=%s\n',R.sensor_source,R.replanner_source);
fprintf(fid,'FLIGHT_MASK_GROUPS: %s\n',strjoin(R.summary_columns,' | '));
for j=1:numel(summaryLabels)
    fprintf(fid,'%s %s\n',summaryLabels{j},sprintf('%.6f ',summary(j,:)));
end
fprintf(fid,'REPLAY_MODES: %s\n',strjoin(R.replay_columns,' | '));
for m=1:nModes
    fprintf(fid,'%s available=%d %s\n',modeNames{m},modeAvailable(m), ...
        sprintf('%.6f ',summaryReplay(m,:)));
end
fprintf(fid,'EVENTS: %s\n',strjoin(R.event_columns,' | '));
for j=1:size(eventTable,1)
    fprintf(fid,'%s\n',sprintf('%.6f ',eventTable(j,:)));
end
fprintf(fid,'TOP_FLIGHT: %s\n',strjoin(R.top_columns,' | '));
for j=1:size(topRows,1)
    fprintf(fid,'%s\n',sprintf('%.6f ',topRows(j,:)));
end
for j=1:numel(checkErrors),fprintf(fid,'MISMATCH %s\n',checkErrors{j});end
fprintf(fid,'NOTE %s\n',R.notes);
fprintf('[OFFLINE SPIKE] %s | Flight events=%d | replay samples=%d | errors=%d\n', ...
    R.status,numel(idx),sum(isfinite(wall(:))),R.errors);
end

function y=prctile_local(x,p)
% Avoid Statistics Toolbox dependency.
x=sort(x(isfinite(x)));
if isempty(x),y=NaN;return;end
k=1+(numel(x)-1)*p/100;
i=floor(k);j=ceil(k);
y=x(i)+(k-i)*(x(j)-x(i));
end

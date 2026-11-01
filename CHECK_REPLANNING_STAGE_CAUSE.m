function A=CHECK_REPLANNING_STAGE_CAUSE(logger,replanner_instance)
% Read-only post-Flight analysis of the ORIGINAL Sim LOGGER. No Flight source edits.
% Requires REPLANNING_STAGE_TIMING=1 BEFORE running run_flight_headless.
% Save only this post-Flight report in the directory containing this script.
if ~isobject(logger)||~isprop(logger,'Data')||~isprop(logger,'k')
    error('CHECK_REPLANNING_STAGE_CAUSE:Logger','Pass the actual Flight LOGGER.');
end
if ~isobject(replanner_instance)||~ismethod(replanner_instance,'get_status')
    error('CHECK_REPLANNING_STAGE_CAUSE:Replanner','Pass actual replanner_instance.');
end
S=replanner_instance.get_status();
if ~isfield(S,'stage_timing_enabled')||~S.stage_timing_enabled
    error('CHECK_REPLANNING_STAGE_CAUSE:Disabled', ...
        'Set REPLANNING_STAGE_TIMING=1 BEFORE running the Flight.');
end
D=logger.Data;
n=double(logger.k);
if ~isstruct(D)||~isfield(D,'t')||~isfield(D,'agent')||n<2||numel(D.t)<n
    error('CHECK_REPLANNING_STAGE_CAUSE:InvalidLogger','Incomplete Flight LOGGER.');
end
R=D.agent(1).reference.result;
if iscell(R), R=R(:); else, R=num2cell(R(:)); end
if numel(R)<n
    error('CHECK_REPLANNING_STAGE_CAUSE:Records','LOGGER reference results incomplete.');
end
labels={'input_validation','EKF_read','sensor_and_result', ...
    'obstacle_log','sensor_log_attach','timing_bookkeeping'};
T=nan(n,6);
u=nan(n,1);s=nan(n,1);candidates=nan(n,1);detected=nan(n,1);
event=false(n,1);missing=0;
for k=1:n
    r=R{k};
    if ~isstruct(r)||~isscalar(r)||~isfield(r,'replanning_stage_ms')|| ...
            ~isfield(r,'replanning_time_ms')|| ...
            ~isfield(r,'simulated_obstacle_sensor')
        missing=missing+1;continue;
    end
    v=r.replanning_stage_ms;
    if ~isnumeric(v)||~isequal(size(v),[1,6])||any(~isfinite(v))||any(v<0)
        missing=missing+1;continue;
    end
    T(k,:)=double(v);
    u(k)=double(r.replanning_time_ms);
    q=r.simulated_obstacle_sensor;
    if isstruct(q)&&isscalar(q)
        s(k)=double(q.elapsed_ms);
        candidates(k)=double(q.candidates_checked);
        detected(k)=double(q.detected_count);
        event(k)=~isempty(q.newly_detected_ids)||~isempty(q.left_range_ids);
    end
end
idx=(2:n)';% First record is Arming, not Flight
valid=all(isfinite(T(idx,:)),2)&isfinite(u(idx))&isfinite(s(idx));
if missing>0||~all(valid)
    error('CHECK_REPLANNING_STAGE_CAUSE:MissingStages', ...
        'Stage traces or sensor logs missing/invalid: %d records.',missing);
end
ms=u(idx);sensor=s(idx);parts=T(idx,:);
stage_sum=sum(parts,2);
time_s=double(D.t(idx));time_s=time_s(:);
events=event(idx);
after_event=[false;events(1:end-1)];
errors={};
if ~strcmp(S.preflight_warmup_mode,'DISABLED_NO_WORLD_PROBES')|| ...
        ~strcmp(S.preflight_update_mode,'EMPTY_SYNTHETIC_WORLD')|| ...
        S.preflight_warmup_distance_checks~=0
    errors{end+1}='Real-world geometry warmup remains enabled.'; %#ok<AGROW>
end
if S.definition_load_count~=1
    errors{end+1}='Obstacle definitions loaded more than once.'; %#ok<AGROW>
end
if S.sensor_status.scan_count~=n
    errors{end+1}='Production sensor scan count does not match LOGGER count.'; %#ok<AGROW>
end
[~,order]=sort(ms,'descend');
top_count=min(25,numel(order));
top=nan(top_count,14);
for j=1:top_count
    i=order(j);
    top(j,:)=[idx(i),time_s(i),double(events(i)),double(after_event(i)), ...
        candidates(idx(i)),detected(idx(i)),sensor(i),ms(i),parts(i,:)];
end
max_per_stage=max(parts,[],1);
over=ms>0.4;
count_over_stage=sum(parts>0.4,1);
A=struct();
A.status='PASS_STAGE_TRACE_CAPTURED__CAUSE_NOT_PROVEN';
if ~isempty(errors),A.status='FAIL_STAGE_TRACE_INTEGRITY';end
A.errors=errors;
A.records=n;
A.flight_records=numel(idx);
A.target_ms=0.4;
A.event_count=sum(events);
A.time_s=time_s;
A.update_ms=ms;
A.sensor_ms=sensor;
A.stage_names=labels;
A.stage_ms=parts;
A.stage_sum_ms=stage_sum;
A.unattributed_proxy_ms=ms-stage_sum;
A.stage_max_ms=max_per_stage;
A.stage_over_0p4=count_over_stage;
A.update_median_ms=median(ms);
A.update_max_ms=max(ms);
A.sensor_median_ms=median(sensor);
A.sensor_max_ms=max(sensor);
A.update_over_0p4=sum(over);
A.event_or_next_over_0p4=sum(over & (events|after_event));
A.other_over_0p4=sum(over & ~(events|after_event));
A.top_rows=top;
A.top_headers={'record','time_s','event','after_event','candidates', ...
    'detected','sensor_ms','update_ms',labels{:}};
A.preflight_warmup_mode=S.preflight_warmup_mode;
A.preflight_update_mode=S.preflight_update_mode;
A.sensor_status=S.sensor_status;
A.replanner_status=S;
folder=fileparts(mfilename('fullpath'));
if isempty(folder),folder=pwd;end
save(fullfile(folder,'REPLANNING_STAGE_CAUSE_AUDIT.mat'),'A','-v7');
fid=fopen(fullfile(folder,'REPLANNING_STAGE_CAUSE_AUDIT.txt'),'w');
if fid<0,error('CHECK_REPLANNING_STAGE_CAUSE:Save','Cannot write TXT report.');end
c=onCleanup(@()fclose(fid));
fprintf(fid,'%s\n',A.status);
fprintf(fid,'Flight=%d events=%d | update median/max=%.6f/%.6f ms | sensor median/max=%.6f/%.6f ms\n', ...
    numel(idx),sum(events),A.update_median_ms,A.update_max_ms,A.sensor_median_ms,A.sensor_max_ms);
fprintf(fid,'0.4ms update exceed=%d | event-or-next=%d | other=%d\n', ...
    A.update_over_0p4,A.event_or_next_over_0p4,A.other_over_0p4);
fprintf(fid,'Warmup modes: %s | %s\n',A.preflight_warmup_mode,A.preflight_update_mode);
fprintf(fid,'STAGE  max_ms  over_0.4ms\n');
for j=1:6
    fprintf(fid,'%s %.6f %d\n',labels{j},max_per_stage(j),count_over_stage(j));
end
fprintf(fid,'TOP_UPDATE: record time event after_event candidates detected sensor_ms update_ms input EKF sensor_stage obstacle_log sensor_attach stats\n');
for j=1:size(top,1)
    fprintf(fid,'%d %.6f %d %d %d %d %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n',top(j,:));
end
fprintf(fid,'NOTE: Stage measurements have tic/toc and logging overhead; wall-time interruptions can affect any stage.\n');
fprintf(fid,'NOTE: replanning_time_ms is measured BEFORE stats and final reference.result assignment. Stage sum is not an exact partition of that metric.\n');
fprintf(fid,'NOTE: This is observational timing, not proof of MATLAB internal allocations or worst-case execution time.\n');
end

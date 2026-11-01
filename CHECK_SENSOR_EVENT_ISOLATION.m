function A=CHECK_SENSOR_EVENT_ISOLATION(logger,replanner_instance)
% CHECK_SENSOR_EVENT_ISOLATION -- Temporary read-only post-flight timing audit.
% The original 15-field sensor result is unchanged. This tool reads the
% diagnostic traces separately through REPLANNING_SYSTEM.get_status().
% No obstacle definitions, true plant state or prior knowledge are used.
% The normal Sim LOGGER exposes Data and k properties, not get().
% Use the exact schema already validated by CHECK_SENSOR_FLIGHT_LOG.
if nargin~=2 || ~isobject(logger) || ~isscalar(logger) || ...
        ~isprop(logger,'Data') || ~isprop(logger,'k')
    error('CHECK_SENSOR_EVENT_ISOLATION:Input', ...
        'Flight後のSim LOGGERとreplanner_instanceを指定してください。');
end
D=logger.Data;
n=double(logger.k);
if ~isstruct(D) || ~isscalar(D) || ~isfield(D,'t') || ...
        ~isfield(D,'agent') || isempty(D.agent) || ...
        ~(isnumeric(n) && isreal(n) && isscalar(n) && isfinite(n) ...
          && n>=2 && n==floor(n)) || numel(D.t)<n
    error('CHECK_SENSOR_EVENT_ISOLATION:Logger', ...
        'Sim LOGGERのData.t/Data.agentまたはkが不正です。');
end
if ~isstruct(D.agent(1)) || ~isfield(D.agent(1),'reference') || ...
        ~isstruct(D.agent(1).reference) || ...
        ~isfield(D.agent(1).reference,'result')
    error('CHECK_SENSOR_EVENT_ISOLATION:Logger', ...
        'Data.agent(1).reference.resultがありません。');
end
records=D.agent(1).reference.result;
if numel(records)<n
    error('CHECK_SENSOR_EVENT_ISOLATION:Logger','Reference record count mismatch.');
end
logged_sensor=nan(n,1); logged_replanner=nan(n,1);
t=nan(n,1); event=false(n,1); candidates=nan(n,1);
for i=1:n
    if iscell(records),r=records{i}; else,r=records(i);end
    if ~isstruct(r) || ~isfield(r,'simulated_obstacle_sensor') || ...
            ~isfield(r,'replanning_time_ms')
        error('CHECK_SENSOR_EVENT_ISOLATION:Record','Missing expected logging fields at record %d.',i);
    end
    s=r.simulated_obstacle_sensor;
    t(i)=double(s.time_s);
    logged_sensor(i)=double(s.elapsed_ms);
    logged_replanner(i)=double(r.replanning_time_ms);
    candidates(i)=double(s.candidates_checked);
    event(i)=~isempty(s.newly_detected_ids) || ~isempty(s.left_range_ids);
end
if any(~isfinite(t)) || any(~isfinite(logged_sensor)) || ...
        any(~isfinite(logged_replanner)) || ...
        any(logged_sensor<0) || any(logged_replanner<0)
    error('CHECK_SENSOR_EVENT_ISOLATION:Timing','Nonfinite or negative duration.');
end
if any(abs(t-reshape(double(D.t(1:n)),[],1))>1e-7)
    error('CHECK_SENSOR_EVENT_ISOLATION:TimeMismatch', ...
        'センサ時刻とLOGGERの時刻が一致しません。');
end
if ~isobject(replanner_instance) || ~ismethod(replanner_instance,'get_status')
    error('CHECK_SENSOR_EVENT_ISOLATION:Replanner','REPLANNING_SYSTEM instance is required.');
end
st=replanner_instance.get_status();
if ~isfield(st,'sensor_status') || ...
        ~isfield(st.sensor_status,'diag_isolation_trace') || ...
        ~isfield(st,'diag_update_trace')
    error('CHECK_SENSOR_EVENT_ISOLATION:InstrumentedSource', ...
        'The temporary instrumented source is not active. Check which -all and clear classes.');
end
ss=st.sensor_status;
S=double(ss.diag_isolation_trace); U=double(st.diag_update_trace);
if isempty(S),S=zeros(0,15);end
if isempty(U),U=zeros(0,15);end
if size(S,2)~=15 || size(U,2)~=15 || ...
        any(~isfinite(S(:))) || any(~isfinite(U(:)))
    error('CHECK_SENSOR_EVENT_ISOLATION:TraceShape','The trace has an invalid shape.');
end
if ss.diag_isolation_dropped>0 || st.diag_update_dropped>0
    error('CHECK_SENSOR_EVENT_ISOLATION:TraceDropped', ...
        'A diagnostic trace exceeded capacity. Do not call it complete.');
end
si=S(:,1);ui=U(:,1);
if any(si~=round(si)) || any(ui~=round(ui)) || ...
        any(si<1|si>n) || any(ui<1|ui>n) || ...
        numel(unique(si))~=numel(si) || numel(unique(ui))~=numel(ui)
    error('CHECK_SENSOR_EVENT_ISOLATION:TraceIndex','Invalid or duplicate record indices.');
end
if ~isempty(S) && (any(abs(S(:,2)-t(si))>1e-8) || ...
        any(abs(S(:,15)-logged_sensor(si))>1e-6) || ...
        any(logical(S(:,3))~=event(si)))
    error('CHECK_SENSOR_EVENT_ISOLATION:SensorMismatch','Trace and SENSOR LOGGER disagree.');
end
if ~isempty(U) && (any(abs(U(:,2)-t(ui))>1e-8) || ...
        any(abs(U(:,4)-logged_replanner(ui))>1e-6) || ...
        any(logical(U(:,3))~=event(ui)))
    error('CHECK_SENSOR_EVENT_ISOLATION:UpdateMismatch','Trace and UPDATE LOGGER disagree.');
end
fl=(2:n)';
events=fl(event(fl));
for k=1:numel(events)
    ix=events(k);
    if ~any(si==ix) || ~any(ui==ix)
        error('CHECK_SENSOR_EVENT_ISOLATION:EventMissing', ...
            'Missing diagnostic sample for event at record %d.',ix);
    end
end
[~,so]=sort(logged_sensor(fl),'descend');
[~,uo]=sort(logged_replanner(fl),'descend');
top_sensor=fl(so(1:min(12,numel(so))));
top_update=fl(uo(1:min(12,numel(uo))));
A=struct();
A.status='PASS_TRACE_CONSISTENCY__CAUSE_STILL_NEEDS_INTERPRETATION';
A.records=n;A.event_indices=events;A.event_times_s=t(events);
A.event_sensor_ms=logged_sensor(events);A.event_replanner_ms=logged_replanner(events);
A.event_count=numel(events);A.sensor_trace=S;A.sensor_trace_columns=ss.diag_isolation_columns;
A.replanner_trace=U;A.replanner_trace_columns=st.diag_update_columns;
A.top_sensor_indices=top_sensor;A.top_update_indices=top_update;
A.flight_sensor_max_ms=max(logged_sensor(fl));
A.flight_replanner_max_ms=max(logged_replanner(fl));
A.flight_sensor_non_event_median_ms=median(logged_sensor(fl(~event(fl))));
A.flight_sensor_event_median_ms=median(logged_sensor(events));
A.event_sensor_breakdown_ms=nan(numel(events),7);
A.event_update_breakdown_ms=nan(numel(events),8);
for k=1:numel(events)
    j=find(si==events(k),1);A.event_sensor_breakdown_ms(k,:)=S(j,8:14);
    j=find(ui==events(k),1);A.event_update_breakdown_ms(k,:)=U(j,5:12);
end
outdir=fileparts(mfilename('fullpath'));
save(fullfile(outdir,'SENSOR_EVENT_ISOLATION_AUDIT.mat'),'A');
fid=fopen(fullfile(outdir,'SENSOR_EVENT_ISOLATION_AUDIT.txt'),'wt');
if fid<0,error('CHECK_SENSOR_EVENT_ISOLATION:Write','Output directory is not writable.');end
cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s\n',A.status);
fprintf(fid,'records=%d events=%d sensor_trace=%d replanner_trace=%d dropped=0\n',n,numel(events),size(S,1),size(U,1));
fprintf(fid,'flight sensor max=%.6f ms replanner max=%.6f ms\n',A.flight_sensor_max_ms,A.flight_replanner_max_ms);
fprintf(fid,'event/non-event sensor median=%.6f/%.6f ms\n',A.flight_sensor_event_median_ms,A.flight_sensor_non_event_median_ms);
fprintf(fid,'SENSOR STAGES: geometry ids visible_struct event_masks result_pack state timing [ms]\n');
fprintf(fid,'UPDATE STAGES: time_ref ekf sensor_call last_result_assign obstacle_frame sensor_field accounting final_write [ms]\n');
fprintf(fid,'EVENT idx time sensor replanner candidate\n');
for k=1:numel(events)
    ix=events(k);
    fprintf(fid,'EVENT %d %.8f %.6f %.6f %.0f\n',ix,t(ix),logged_sensor(ix),logged_replanner(ix),candidates(ix));
    fprintf(fid,' SENSOR:');fprintf(fid,' %.6f',A.event_sensor_breakdown_ms(k,:));fprintf(fid,'\n');
    fprintf(fid,' UPDATE:');fprintf(fid,' %.6f',A.event_update_breakdown_ms(k,:));fprintf(fid,'\n');
    fprintf(fid,' SENSOR geometry/ids/visible/event_masks/pack/state/timing shares (%%):');
    fprintf(fid,' %.1f',100*A.event_sensor_breakdown_ms(k,:)/logged_sensor(ix));fprintf(fid,'\n');
    fprintf(fid,' UPDATE minus sensor_recorded = %.6f ms (the two clocks do not cover identical boundaries)\n', ...
        logged_replanner(ix)-logged_sensor(ix));
end
fprintf(fid,'TOP_SENSOR idx time_s event sensor_ms update_ms (sensor split if captured)\n');
for ix=top_sensor(:)'
    fprintf(fid,'%d %.8f %d %.6f %.6f',ix,t(ix),event(ix),logged_sensor(ix),logged_replanner(ix));
    j=find(si==ix,1);
    if isempty(j), fprintf(fid,' NOT_CAPTURED');
    else, fprintf(fid,' [');fprintf(fid,' %.6f',S(j,8:14));fprintf(fid,' ]');end
    fprintf(fid,'\n');
end
fprintf(fid,'TOP_UPDATE idx time_s event sensor_ms update_ms (update split if captured)\n');
for ix=top_update(:)'
    fprintf(fid,'%d %.8f %d %.6f %.6f',ix,t(ix),event(ix),logged_sensor(ix),logged_replanner(ix));
    j=find(ui==ix,1);
    if isempty(j), fprintf(fid,' NOT_CAPTURED');
    else, fprintf(fid,' [');fprintf(fid,' %.6f',U(j,5:12));fprintf(fid,' ]');end
    fprintf(fid,'\n');
end
fprintf(fid,'NOTE: The additional tic/toc calls can alter execution. Durations are not WCET proofs.\n');
fprintf(fid,'NOTE: Obstacle sets remain sensor-only; detected_obstacles contains only currently sensed items.\n');
fprintf('[SENSOR EVENT ISOLATION] %s | max sensor=%.4f ms | max replanner=%.4f ms\n', ...
    A.status,A.flight_sensor_max_ms,A.flight_replanner_max_ms);
end

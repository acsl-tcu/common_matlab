function R=CHECK_REPLANNER_SPIKE_GATE(logger,replanner_instance,target_ms)
% CHECK_REPLANNER_SPIKE_GATE  Observe FULL Flight spikes against an empirical target.
% Does not modify the runner, REPLANNING_SYSTEM, sensor or LOGGER.
% target_ms is a measurement goal, NOT a guaranteed upper bound.
if nargin<3, target_ms=0.4;end
if nargin<2
    error('CHECK_REPLANNER_SPIKE_GATE:Args', ...
        'Use CHECK_REPLANNER_SPIKE_GATE(logger,replanner_instance,0.4).');
end
if ~(isnumeric(target_ms)&&isscalar(target_ms)&&isreal(target_ms) ...
        &&isfinite(target_ms)&&target_ms>0)
    error('CHECK_REPLANNER_SPIKE_GATE:Target','target_ms must be positive finite scalar.');
end
if ~(isobject(logger)&&isprop(logger,'Data')&&isprop(logger,'k'))
    error('CHECK_REPLANNER_SPIKE_GATE:Logger','Expected Sim LOGGER object.');
end
n=double(logger.k);
if ~(isscalar(n)&&isfinite(n)&&n>=3&&n==floor(n))
    error('CHECK_REPLANNER_SPIKE_GATE:Records','LOGGER.k is invalid.');
end
if ~isfield(logger.Data,'agent') || isempty(logger.Data.agent) || ...
        ~isfield(logger.Data.agent(1),'reference') || ...
        ~isfield(logger.Data.agent(1).reference,'result')
    error('CHECK_REPLANNER_SPIKE_GATE:Records','No reference.result records.');
end
records=logger.Data.agent(1).reference.result;
if numel(records)<n
    error('CHECK_REPLANNER_SPIKE_GATE:Records','LOGGER has too few records.');
end
time_s=nan(n,1); sensor_ms=nan(n,1); update_ms=nan(n,1); ev=false(n,1);
for k=1:n
    if iscell(records),rec=records{k};else,rec=records(k);end
    if ~(isstruct(rec)&&isfield(rec,'simulated_obstacle_sensor') ...
            &&isfield(rec,'replanning_time_ms'))
        error('CHECK_REPLANNER_SPIKE_GATE:Record','Missing timing at record %d',k);
    end
    s=rec.simulated_obstacle_sensor;
    time_s(k)=double(s.time_s);
    sensor_ms(k)=double(s.elapsed_ms);
    update_ms(k)=double(rec.replanning_time_ms);
    ev(k)=~isempty(s.newly_detected_ids)||~isempty(s.left_range_ids);
end
if any(~isfinite(sensor_ms))||any(~isfinite(update_ms))|| ...
        any(sensor_ms<0)||any(update_ms<0)||any(~isfinite(time_s))
    error('CHECK_REPLANNER_SPIKE_GATE:Timing','Invalid time data or logging disabled.');
end
S=replanner_instance.get_status();
if S.sensor_status.scan_count~=n || S.sensor_status.timing_count~=n
    error('CHECK_REPLANNER_SPIKE_GATE:Counts','LOGGER and sensor counts differ.');
end
flight=(2:n)'; % first record is an Arming/preflight record
other_ms=update_ms-sensor_ms; % only proxy; clocks have different boundaries
R=struct();
R.status='OBSERVED_SPIKES_ABOVE_TARGET';
R.target_ms=double(target_ms);
R.count=numel(flight);R.events=sum(ev(flight));
R.native_enabled=S.sensor_status.native_event_pack_enabled;
R.native_build_status=S.sensor_status.native_event_pack_build_status;
R.sensor_median_ms=median(sensor_ms(flight));
R.update_median_ms=median(update_ms(flight));
R.sensor_max_ms=max(sensor_ms(flight));
R.update_max_ms=max(update_ms(flight));
R.other_proxy_max_ms=max(other_ms(flight));
R.sensor_over_target=sum(sensor_ms(flight)>target_ms);
R.update_over_target=sum(update_ms(flight)>target_ms);
R.other_proxy_over_target=sum(other_ms(flight)>target_ms);
R.event_sensor_max_ms=nan;R.event_update_max_ms=nan;
if any(ev(flight))
    R.event_sensor_max_ms=max(sensor_ms(flight(ev(flight))));
    R.event_update_max_ms=max(update_ms(flight(ev(flight))));
end
if R.update_over_target==0 && R.sensor_over_target==0
    R.status='OBSERVED_WITHIN_TARGET_THIS_FLIGHT__NOT_WCET';
end
[~,ord]=sort(update_ms(flight),'descend');
idx=flight(ord(1:min(15,numel(ord))));
R.top_update_rows=[double(idx),time_s(idx),double(ev(idx)),sensor_ms(idx),update_ms(idx),other_ms(idx)];
[~,ord2]=sort(other_ms(flight),'descend');
idx2=flight(ord2(1:min(15,numel(ord2))));
R.top_other_rows=[double(idx2),time_s(idx2),double(ev(idx2)),sensor_ms(idx2),update_ms(idx2),other_ms(idx2)];
R.note=['The update-sensor difference is only a diagnostic proxy, not an exact', ...
        ' partition of wall time. MATLAB/Windows WCET is not proven.'];
folder=fileparts(mfilename('fullpath'));
save(fullfile(folder,'REPLANNER_SPIKE_GATE_AUDIT.mat'),'R');
fid=fopen(fullfile(folder,'REPLANNER_SPIKE_GATE_AUDIT.txt'),'wt');
if fid<0,error('CHECK_REPLANNER_SPIKE_GATE:Write','Cannot write TXT report.');end
closer=onCleanup(@()fclose(fid));
fprintf(fid,'%s\n',R.status);
fprintf(fid,'target=%.6f ms | records=%d | events=%d | native=%d (%s)\n', ...
    target_ms,R.count,R.events,R.native_enabled,R.native_build_status);
fprintf(fid,'sensor median/max=%.6f/%.6f ms | update median/max=%.6f/%.6f ms\n', ...
    R.sensor_median_ms,R.sensor_max_ms,R.update_median_ms,R.update_max_ms);
fprintf(fid,'over_target sensor/update/other_proxy=%d/%d/%d\n', ...
    R.sensor_over_target,R.update_over_target,R.other_proxy_over_target);
fprintf(fid,'event sensor/update max=%.6f/%.6f ms\n', ...
    R.event_sensor_max_ms,R.event_update_max_ms);
fprintf(fid,'TOP_UPDATE index time_s event sensor_ms update_ms other_proxy_ms\n');
for j=1:size(R.top_update_rows,1),fprintf(fid,'%d %.6f %d %.6f %.6f %.6f\n',R.top_update_rows(j,:));end
fprintf(fid,'TOP_OTHER index time_s event sensor_ms update_ms other_proxy_ms\n');
for j=1:size(R.top_other_rows,1),fprintf(fid,'%d %.6f %d %.6f %.6f %.6f\n',R.top_other_rows(j,:));end
fprintf(fid,'NOTE %s\n',R.note);
end

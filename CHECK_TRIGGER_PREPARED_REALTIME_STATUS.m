function S=CHECK_TRIGGER_PREPARED_REALTIME_STATUS(logger,replanner_instance)
% Postflight read-only timing diagnostic from the existing Sim LOGGER.
% Never changes sensor/EKF/plant/HLC/reference/history.
assert(isobject(logger)&&isprop(logger,'Data')&&isfield(logger.Data,'agent'), ...
    'CHECK_TRIGGER_PREPARED_REALTIME_STATUS:Logger','Actual Sim LOGGER required.');
C=logger.Data.agent(1).reference.result;
if ~iscell(C),C=num2cell(C);end
n=min(double(logger.k),numel(C));
t=nan(n,1);q=nan(n,1);geometry=nan(n,1);predictor=nan(n,1);
prepared=false(n,1);available=false(n,1);
for k=1:n
    r=C{k};
    if ~isstruct(r)||~isscalar(r)||~isfield(r,'future_collision_trigger'),continue;end
    v=r.future_collision_trigger;
    if ~isstruct(v)||~isscalar(v),continue;end
    if isfield(v,'time_s'),t(k)=double(v.time_s);end
    if isfield(r,'trigger_time_ms'),q(k)=double(r.trigger_time_ms);end
    if isfield(v,'prediction_available')
        available(k)=isequal(v.prediction_available,true);
    end
    if isfield(v,'geometry_prepared_from_observed_sensor')
        prepared(k)=isequal(v.geometry_prepared_from_observed_sensor,true);
    end
    if isfield(v,'geometry_wall_ms')
        geometry(k)=double(v.geometry_wall_ms);
    end
    if isfield(v,'predictor_wall_ms')
        predictor(k)=double(v.predictor_wall_ms);
    end
end
S=struct('status','READ_ONLY_RUNTIME_TIMING_NOT_SAFETY_PROOF', ...
    'records',n,'completed_predictions',nnz(available), ...
    'observed_sensor_prepared_count',nnz(prepared), ...
    'trigger',stat(q),'geometry',stat(geometry),'predictor',stat(predictor), ...
    'preflight',[],'continuous_certified',false);
if nargin>=2 && isobject(replanner_instance)
    z=replanner_instance.get_status();
    S.preflight=struct('mode',z.preflight_update_mode, ...
       'ready',z.preflight_update_ready,'elapsed_ms',z.preflight_update_ms);
end
windows=[4.95 5.20; 9.80 10.10; 33 42];
W=struct('start_s',{},'end_s',{},'trigger_max_ms',{},'peak_time_s',{});
for j=1:size(windows,1)
    idx=find(t>=windows(j,1)&t<=windows(j,2)&isfinite(q));
    vmax=NaN;tt=NaN;
    if ~isempty(idx)
        [vmax,i]=max(q(idx));tt=t(idx(i));
    end
    W(j)=struct('start_s',windows(j,1),'end_s',windows(j,2), ...
        'trigger_max_ms',vmax,'peak_time_s',tt);
end
S.windows=W;
home=fileparts(mfilename('fullpath'));
save(fullfile(home,'TRIGGER_PREPARED_REALTIME_STATUS.mat'),'S','-v7');
fid=fopen(fullfile(home,'TRIGGER_PREPARED_REALTIME_STATUS.txt'),'w');
if fid>=0
    c=onCleanup(@()fclose(fid));
    fprintf(fid,'status=%s\nrecords=%d predictions=%d prepared_sensor_uses=%d\n', ...
        S.status,n,S.completed_predictions,S.observed_sensor_prepared_count);
    for key={'trigger','geometry','predictor'}
        name=key{1};v=S.(name);
        fprintf(fid,'%s count=%d median_ms=%.6f p95_ms=%.6f max_ms=%.6f\n', ...
            name,v.count,v.median_ms,v.p95_ms,v.max_ms);
    end
    if ~isempty(S.preflight)
        fprintf(fid,'preflight_mode=%s ready=%d elapsed_ms=%.6f\n', ...
            S.preflight.mode,S.preflight.ready,S.preflight.elapsed_ms);
    end
    for j=1:numel(W)
        fprintf(fid,'window=[%.2f %.2f] peak_time_s=%.6f max_ms=%.6f\n', ...
            W(j).start_s,W(j).end_s,W(j).peak_time_s,W(j).trigger_max_ms);
    end
    fprintf(fid,'continuous_safe_claim=0\n');
end
end
function T=stat(x)
x=sort(x(isfinite(x)));
T=struct('count',numel(x),'median_ms',NaN,'p95_ms',NaN,'max_ms',NaN);
if ~isempty(x)
    T.median_ms=median(x);T.p95_ms=x(max(1,ceil(.95*numel(x))));
    T.max_ms=x(end);
end
end

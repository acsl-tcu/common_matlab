function S=CHECK_TRIGGER_RECORDED_SPIKES(logger)
% Offline, read-only analysis of ORIGINAL Sim LOGGER (no agent needed).
% Existing logs: trigger elapsed, reported native HLC/solver/setup times.
% Patched Flight logs additionally provide predictor_wall_ms and
% geometry_wall_ms. Unattributed residual is NOT assumed to be geometry.
assert(nargin==1 && isobject(logger) && isprop(logger,'Data'), ...
    'CHECK_TRIGGER_RECORDED_SPIKES:Logger','Pass the saved actual Sim logger.');
D=logger.Data;
assert(isfield(D,'agent')&&isfield(D.agent(1),'reference'), ...
    'CHECK_TRIGGER_RECORDED_SPIKES:Schema','Sim reference log missing.');
C=D.agent(1).reference.result;
if ~iscell(C),C=num2cell(C);end
n=min(numel(C),double(logger.k));
t=nan(n,1);tr=nan(n,1);rp=nan(n,1);cy=nan(n,1);
hlc=nan(n,1);ode=nan(n,1);setup=nan(n,1);
geom=nan(n,1);predictor_wall=nan(n,1);status=cell(n,1);
if isfield(D,'t') && isnumeric(D.t) && numel(D.t)>=n
    t(:)=double(D.t(1:n));
end
if isfield(D,'system_cycle_time_ms') && isnumeric(D.system_cycle_time_ms)
    z=double(D.system_cycle_time_ms(:));cy(1:min(n,numel(z)))=z(1:min(n,numel(z)));
end
for k=1:n
    r=C{k};if ~isstruct(r)||~isscalar(r),continue;end
    if isfield(r,'trigger_time_ms'),tr(k)=as_ms(r.trigger_time_ms);end
    if isfield(r,'replanning_time_ms'),rp(k)=as_ms(r.replanning_time_ms);end
    if ~isfield(r,'future_collision_trigger'),continue;end
    v=r.future_collision_trigger;
    if ~isstruct(v)||~isscalar(v),continue;end
    if isfield(v,'time_s'),t(k)=as_ms(v.time_s);end
    if isfield(v,'status'),status{k}=char(v.status);end
    if isfield(v,'predictor_hlc_time_ms'),hlc(k)=as_ms(v.predictor_hlc_time_ms);end
    if isfield(v,'predictor_plant_time_ms'),ode(k)=as_ms(v.predictor_plant_time_ms);end
    if isfield(v,'predictor_setup_time_ms'),setup(k)=as_ms(v.predictor_setup_time_ms);end
    if isfield(v,'predictor_wall_ms'),predictor_wall(k)=as_ms(v.predictor_wall_ms);end
    if isfield(v,'geometry_wall_ms'),geom(k)=as_ms(v.geometry_wall_ms);end
end
% Values from different wall-clock scopes are diagnostic only.
% Native sum is not algebraically the same stopwatch as trigger elapsed.
native=hlc+ode+setup;residual=tr-native;
completed=isfinite(native)&isfinite(tr);
S=struct();
S.status='RECORDED_FLIGHT_DIAGNOSTIC_ONLY__NO_SAFETY_CLAIM';
S.records=n;
S.prediction_records=nnz(completed);
S.trigger_ms=distribution(tr);
S.replanner_ms=distribution(rp);
S.cycle_ms=distribution(cy);
S.native_reported_ms=distribution(native(completed));
S.unattributed_ms=distribution(residual(completed));
S.geometry_ms=distribution(geom);
S.predictor_wall_ms=distribution(predictor_wall);
S.known_geometry_split_available=nnz(isfinite(geom));
S.first_completed_time_s=NaN;
if any(completed),i=find(completed,1);S.first_completed_time_s=t(i);end
S.first_completed_trigger_ms=NaN;
if any(completed),i=find(completed,1);S.first_completed_trigger_ms=tr(i);end
valid_ix=find(isfinite(tr));
[~,order]=sort(tr(valid_ix),'descend');
ix=valid_ix(order); ix=ix(1:min(15,numel(ix)));
S.top_spikes=struct('time_s',{},'trigger_ms',{},'replanner_ms',{}, ...
    'cycle_ms',{},'native_hlc_ms',{},'native_solver_ms',{}, ...
    'native_setup_ms',{},'residual_ms',{}, ...
    'geometry_ms',{},'predictor_wall_ms',{},'status',{});
for z=1:numel(ix)
    k=ix(z);
    S.top_spikes(z)=struct('time_s',t(k),'trigger_ms',tr(k), ...
        'replanner_ms',rp(k),'cycle_ms',cy(k), ...
        'native_hlc_ms',hlc(k),'native_solver_ms',ode(k), ...
        'native_setup_ms',setup(k),'residual_ms',residual(k), ...
        'geometry_ms',geom(k),'predictor_wall_ms',predictor_wall(k), ...
        'status',status{k});
end
windows=[4.95 5.20;9.80 10.10;33 42];
S.windows=struct('start_s',{},'end_s',{},'trigger_max_ms',{}, ...
    'replanner_max_ms',{},'cycle_max_ms',{});
for j=1:size(windows,1)
    valid=t>=windows(j,1)&t<=windows(j,2);
    S.windows(j)=struct('start_s',windows(j,1),'end_s',windows(j,2), ...
        'trigger_max_ms',max_safe(tr(valid)), ...
        'replanner_max_ms',max_safe(rp(valid)), ...
        'cycle_max_ms',max_safe(cy(valid)));
end
home=fileparts(mfilename('fullpath'));
save(fullfile(home,'TRIGGER_RECORDED_SPIKES.mat'),'S','-v7');
fid=fopen(fullfile(home,'TRIGGER_RECORDED_SPIKES.txt'),'wt');
if fid>=0
    closer=onCleanup(@()fclose(fid));
    fprintf(fid,'status=%s\nrecords=%d\ncomplete=%d\n', ...
        S.status,S.records,S.prediction_records);
    fprintf(fid,'TRIGGER median=%.6f p95=%.6f max=%.6f ms\n', ...
        S.trigger_ms.median,S.trigger_ms.p95,S.trigger_ms.max);
    fprintf(fid,'NATIVE_SUM median=%.6f p95=%.6f max=%.6f ms\n', ...
        S.native_reported_ms.median,S.native_reported_ms.p95,S.native_reported_ms.max);
    fprintf(fid,'UNATTRIBUTED median=%.6f p95=%.6f max=%.6f ms\n', ...
        S.unattributed_ms.median,S.unattributed_ms.p95,S.unattributed_ms.max);
    fprintf(fid,'geometry_split_records=%d\n',S.known_geometry_split_available);
    fprintf(fid,'first_completed_t=%.9f first_completed_trigger_ms=%.6f\n', ...
        S.first_completed_time_s,S.first_completed_trigger_ms);
    fprintf(fid,'TOP_SPIKES time trigger replanner cycle HLC solver setup residual geometry predictor_wall STATUS\n');
    for k=1:numel(S.top_spikes)
        e=S.top_spikes(k);
        fprintf(fid,'%.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %s\n', ...
            e.time_s,e.trigger_ms,e.replanner_ms,e.cycle_ms, ...
            e.native_hlc_ms,e.native_solver_ms,e.native_setup_ms, ...
            e.residual_ms,e.geometry_ms,e.predictor_wall_ms,e.status);
    end
    fprintf(fid,'WINDOWS start end maxTrigger maxReplanner maxCycle\n');
    for k=1:numel(S.windows)
        w=S.windows(k);
        fprintf(fid,'%.6f %.6f %.6f %.6f %.6f\n', ...
            w.start_s,w.end_s,w.trigger_max_ms,w.replanner_max_ms,w.cycle_max_ms);
    end
end
fprintf('[TRIGGER SPIKES] records=%d complete=%d trigger median=%.4f max=%.4f ms\n', ...
    n,nnz(completed),S.trigger_ms.median,S.trigger_ms.max);
end
function n=as_ms(x)
n=NaN;
if isnumeric(x)&&isreal(x)&&isscalar(x)&&isfinite(x),n=double(x);end
end
function v=distribution(x)
x=x(isfinite(x));x=sort(x(:));n=numel(x);
v=struct('count',n,'median',NaN,'p95',NaN,'max',NaN);
if n>0
    v.median=median(x);v.p95=x(max(1,ceil(.95*n)));v.max=x(end);
end
end
function v=max_safe(x)
x=x(isfinite(x));v=NaN;
if ~isempty(x),v=max(x);end
end

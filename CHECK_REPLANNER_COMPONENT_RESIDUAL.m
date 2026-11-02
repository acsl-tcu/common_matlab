function R=CHECK_REPLANNER_COMPONENT_RESIDUAL(logger)
% Read-only: inspect stages where replanner time rises but trigger/sensor do not.
% Requires ordinary original Sim LOGGER. New detail field is OPTIONAL.
if ~isobject(logger) || ~isprop(logger,'Data') || ~isprop(logger,'k')
    error('CHECK_REPLANNER_COMPONENT_RESIDUAL:Input', ...
        'Pass the original Sim LOGGER after an ordinary Flight.');
end
L=logger.Data;
if ~isfield(L,'agent') || isempty(L.agent) || ...
        ~isfield(L.agent(1),'reference') || ...
        ~isfield(L.agent(1).reference,'result')
    error('CHECK_REPLANNER_COMPONENT_RESIDUAL:Data','No reference.result.');
end
C=L.agent(1).reference.result;
if ~iscell(C),C=num2cell(C);end
n=min(numel(C),double(logger.k));
t=nan(n,1); rp=nan(n,1); tr=nan(n,1); se=nan(n,1);
st=nan(n,6); dd=nan(n,6);
for k=1:n
    q=C{k};
    if ~isstruct(q) || ~isscalar(q),continue;end
    if isfield(q,'replanning_time_ms'),rp(k)=number(q.replanning_time_ms);end
    if isfield(q,'trigger_time_ms'),tr(k)=number(q.trigger_time_ms);end
    if isfield(q,'simulated_obstacle_sensor') && ...
            isstruct(q.simulated_obstacle_sensor)
        a=q.simulated_obstacle_sensor;
        if isfield(a,'time_s'),t(k)=number(a.time_s);end
        if isfield(a,'elapsed_ms'),se(k)=number(a.elapsed_ms);end
    end
    if isfield(q,'replanning_stage_ms') && ...
            isnumeric(q.replanning_stage_ms) && numel(q.replanning_stage_ms)==6
        st(k,:)=double(q.replanning_stage_ms(:))';
    end
    if isfield(q,'replanning_detail_ms') && ...
            isnumeric(q.replanning_detail_ms) && numel(q.replanning_detail_ms)==6
        dd(k,:)=double(q.replanning_detail_ms(:))';
    end
end
% This residual includes memory, EKF read, logger attachment, and bookkeeping.
% NOT a proof that one of these specific functions caused the excess.
res=rp-tr-se;
valid=find(isfinite(res));
[~,order]=sort(res(valid),'descend');
idx=valid(order);
idx=idx(1:min(20,numel(idx)));
R=struct('status','READ_ONLY_NONTRIGGER_SPIKE_BREAKDOWN', ...
    'record_count',n,'detail_samples',nnz(all(isfinite(dd(:,1:3)),2)), ...
    'residual_median_ms',median(res(isfinite(res))), ...
    'residual_p95_ms',nth95(res(isfinite(res))), ...
    'top_times_s',t(idx), ...
    'top_replanner_ms',rp(idx),'top_trigger_ms',tr(idx), ...
    'top_sensor_ms',se(idx),'top_residual_ms',res(idx), ...
    'top_stage_ms',st(idx,:), 'top_detail_ms',dd(idx,:), ...
    'stage_labels',{{'input','EKF','sensor_plus_memory', ...
        'trigger_plus_obstacle_log','result_attachment','timing_bookkeeping'}}, ...
    'detail_labels',{{'sensor_observe','memory_update', ...
        'prepared_handoff','trigger','obstacle_log','result_attachment'}});
root=fileparts(mfilename('fullpath'));
save(fullfile(root,'REPLANNER_COMPONENT_RESIDUAL.mat'),'R','-v7');
fid=fopen(fullfile(root,'REPLANNER_COMPONENT_RESIDUAL.txt'),'wt');
if fid>=0
    closer=onCleanup(@()fclose(fid)); %#ok<NASGU>
    fprintf(fid,'status=%s\nrecords=%d detail_samples=%d\n', ...
        R.status,R.record_count,R.detail_samples);
    fprintf(fid,'residual_median_ms=%.6f residual_p95_ms=%.6f\n', ...
        R.residual_median_ms,R.residual_p95_ms);
    fprintf(fid,'Top residual: t replanner trigger sensor residual stage[6] detail[6]\n');
    for j=1:numel(idx)
        fprintf(fid,'%.6f %.6f %.6f %.6f %.6f %s %s\n', ...
            t(idx(j)),rp(idx(j)),tr(idx(j)),se(idx(j)),res(idx(j)), ...
            mat2str(st(idx(j),:),5),mat2str(dd(idx(j),:),5));
    end
end
fprintf('[REPLANNER COMPONENTS] records=%d, detailed=%d, residual median=%.3f ms\n', ...
    n,R.detail_samples,R.residual_median_ms);
end
function v=number(x)
v=NaN;
if isnumeric(x)&&isscalar(x)&&isreal(x)&&isfinite(x),v=double(x);end
end
function p=nth95(x)
p=NaN;
if isempty(x),return;end
x=sort(x(:));p=x(max(1,ceil(.95*numel(x))));
end

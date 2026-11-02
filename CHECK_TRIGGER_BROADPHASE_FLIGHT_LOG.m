function R=CHECK_TRIGGER_BROADPHASE_FLIGHT_LOG(logger)
% Read-only. No EKF, HLC, sensor call, flight run, or source mutation.
assert(isobject(logger)&&isprop(logger,'Data')&& ...
    isfield(logger.Data,'agent')&& ...
    isfield(logger.Data.agent(1).reference,'result'), ...
    'CHECK_TRIGGER_BROADPHASE_FLIGHT_LOG:Logger','Flight logger required.');
c=logger.Data.agent(1).reference.result;
if ~iscell(c), c=num2cell(c); end
n=min(numel(c),double(logger.k));
T=nan(n,1);PR=T;GE=T;EX=zeros(n,1);NC=zeros(n,1);
active=false(n,1);
for k=1:n
    x=c{k};
    if ~isstruct(x)||~isscalar(x),continue;end
    if isfield(x,'trigger_time_ms'),T(k)=double(x.trigger_time_ms);end
    if isfield(x,'future_collision_trigger')&&isstruct(x.future_collision_trigger)
        p=x.future_collision_trigger;
        if isfield(p,'prediction_complete')&&isequal(p.prediction_complete,true)
            active(k)=true;
        end
        if isfield(p,'geometry_wall_ms')&&isscalar(p.geometry_wall_ms)
            GE(k)=double(p.geometry_wall_ms);
        end
        if isfield(p,'predictor_wall_ms')&&isscalar(p.predictor_wall_ms)
            PR(k)=double(p.predictor_wall_ms);
        end
        if isfield(p,'geometry_sampled_broadphase_exclusions')
            EX(k)=double(p.geometry_sampled_broadphase_exclusions);
        end
        if isfield(p,'geometry_exact_body_queries')
            NC(k)=double(p.geometry_exact_body_queries);
        end
    end
end
R=struct('status','READ_ONLY_NEW_BROADPHASE_FLIGHT_TIMING__NO_SAFETY_CLAIM', ...
    'records',n,'prediction_complete',nnz(active), ...
    'excluded_sampled_body_checks',sum(EX), ...
    'exact_body_query_count',sum(NC), ...
    'trigger_median_all_ms',median(T(isfinite(T))), ...
    'trigger_median_predicting_ms',median(T(active)), ...
    'trigger_p95_predicting_ms',local_p95(T(active)), ...
    'trigger_max_predicting_ms',max(T(active)), ...
    'geometry_median_predicting_ms',median(GE(active)), ...
    'predictor_median_predicting_ms',median(PR(active)), ...
    'continuous_time_certified',false);
root=fileparts(mfilename('fullpath'));
save(fullfile(root,'TRIGGER_BROADPHASE_FLIGHT_LOG.mat'),'R','-v7');
fid=fopen(fullfile(root,'TRIGGER_BROADPHASE_FLIGHT_LOG.txt'),'wt');
if fid>=0
    C=onCleanup(@()fclose(fid)); %#ok<NASGU>
    fprintf(fid,'status=%s\nrecords=%d\npredictions=%d\n',R.status,n,nnz(active));
    fprintf(fid,'excluded_sampled_body_checks=%d\n',R.excluded_sampled_body_checks);
    fprintf(fid,'trigger_median_predicting_ms=%.6f\n',R.trigger_median_predicting_ms);
    fprintf(fid,'trigger_p95_predicting_ms=%.6f\n',R.trigger_p95_predicting_ms);
    fprintf(fid,'trigger_max_predicting_ms=%.6f\n',R.trigger_max_predicting_ms);
end
end
function q=local_p95(x)
x=sort(x(isfinite(x)));
if isempty(x),q=NaN;else,q=x(max(1,ceil(.95*numel(x))));end
end

function S=CHECK_TRIGGER_TIMING_AND_MEMORY(logger,replanner_instance)
% Read-only postflight audit. Uses standard Sim LOGGER, not backup traces.
assert(nargin==2 && isobject(replanner_instance), ...
    'CHECK_TRIGGER_TIMING_AND_MEMORY:Usage', ...
    'Run CHECK_TRIGGER_TIMING_AND_MEMORY(logger,replanner_instance) after Flight.');
D=logger.Data;
assert(isfield(D,'agent') && isfield(D.agent(1),'reference'), ...
    'CHECK_TRIGGER_TIMING_AND_MEMORY:Logger','Reference log is missing.');
C=D.agent(1).reference.result;
if ~iscell(C),C=num2cell(C);end
n=min(numel(C),double(logger.k));
times=nan(n,1); statuses=cell(n,1); safety_claims=false(n,1);
known=cell(n,1);predicted=0;
partial=0;completed_horizon=nan(n,1);
hlc_ms=nan(n,1);ode_ms=nan(n,1);setup_ms=nan(n,1);

reason_examples=struct('status',{},'reason',{});
for i=1:n
    r=C{i};
    if ~isstruct(r),continue;end
    if isfield(r,'trigger_time_ms') && isnumeric(r.trigger_time_ms) && ...
            isscalar(r.trigger_time_ms) && isfinite(r.trigger_time_ms)
        times(i)=double(r.trigger_time_ms);
    end
    if isfield(r,'future_collision_trigger') && isstruct(r.future_collision_trigger)
        tr=r.future_collision_trigger;
        if isfield(tr,'status')
            statuses{i}=char(tr.status);
            if ~any(strcmp({reason_examples.status},statuses{i}))
                why='';
                if isfield(tr,'reason'),why=char(tr.reason);end
                reason_examples(end+1)=struct( ...
                    'status',statuses{i},'reason',why); %#ok<AGROW>
            end
        end
        if isfield(tr,'prediction_available') && isequal(tr.prediction_available,true)
            predicted=predicted+1;
        elseif isfield(tr,'status') && ...
                strcmp(tr.status,'UNKNOWN_PREDICTOR_ERROR')
            partial=partial+1;
        end
        if isfield(tr,'completed_prediction_horizon_s')
            completed_horizon(i)=tr.completed_prediction_horizon_s;
        end
        if isfield(tr,'predictor_hlc_time_ms')
            hlc_ms(i)=tr.predictor_hlc_time_ms;
            ode_ms(i)=tr.predictor_plant_time_ms;
            setup_ms(i)=tr.predictor_setup_time_ms;
        end
        if isfield(tr,'continuous_observed_set_safe')
            safety_claims(i)=isequal(tr.continuous_observed_set_safe,true);
        end
    end
    if isfield(r,'observed_obstacle_memory') && isstruct(r.observed_obstacle_memory)
        mr=r.observed_obstacle_memory;
        if isfield(mr,'known_obstacle_ids'),known{i}=double(mr.known_obstacle_ids);end
    end
end
valid=times(isfinite(times));
if isempty(valid)
    med=NaN;p95=NaN;mx=NaN;
else
    valid=sort(valid);med=median(valid);
    p95=valid(max(1,ceil(.95*numel(valid))));mx=valid(end);
end
rstatus=replanner_instance.get_status();
S=struct('records',n,'timed',numel(valid),'median_ms',med, ...
    'P95_ms',p95,'max_ms',mx,'actual_prediction_count',predicted, ...
    'unknown_model_count',partial, ...
    'median_completed_horizon_s',median(completed_horizon(isfinite(completed_horizon))), ...
    'total_hlc_time_ms',sum(hlc_ms(isfinite(hlc_ms))), ...
    'total_ode15s_time_ms',sum(ode_ms(isfinite(ode_ms))), ...
    'total_predictor_setup_ms',sum(setup_ms(isfinite(setup_ms))), ...
    'continuous_safe_claim_count',nnz(safety_claims), ...
    'last_trigger_status','', ...
    'known_ids',rstatus.memory_status.known_ids, ...
    'preflight_mode',rstatus.preflight_update_mode, ...
    'numeric_abs_tol',rstatus.TRIGGER_NUMERIC_ABS_TOL,...
    'numeric_rel_tol',rstatus.TRIGGER_NUMERIC_REL_TOL,...
    'flight_phase_end_s',rstatus.TRIGGER_FLIGHT_PHASE_END_S);
nonempty=statuses(~cellfun(@isempty,statuses));
if isempty(nonempty)
    S.status_counts=struct('status',{},'count',{});
else
    types=unique(nonempty,'stable');
    counts=zeros(numel(types),1);
    for ix=1:numel(types),counts(ix)=nnz(strcmp(nonempty,types{ix}));end
    S.status_counts=struct('status',types(:), ...
        'count',num2cell(counts(:)));
end
S.reason_examples=reason_examples;
idx=find(~cellfun(@isempty,statuses),1,'last');
if ~isempty(idx),S.last_trigger_status=statuses{idx};end
fprintf('[TRIGGER] measured=%d/%d | median=%.4f P95=%.4f max=%.4f ms | nominal predictions=%d\n', ...
    numel(valid),n,med,p95,mx,predicted);
disp(S);
fprintf('[TRIGGER COMPUTE] complete=%d, incomplete=%d | total HLC=%.3f ms, solver=%.3f ms, setup=%.3f ms\n', ...
    predicted,partial,S.total_hlc_time_ms,S.total_ode15s_time_ms,S.total_predictor_setup_ms);
if ~isempty(S.status_counts)
    fprintf('[TRIGGER] Status counts and first associated reason:\n');
    for k=1:numel(S.status_counts)
        key=S.status_counts(k).status;
        fprintf('  %6d %s\n',S.status_counts(k).count,key);
        ix=find(strcmp({S.reason_examples.status},key),1);
        if ~isempty(ix) && ~isempty(S.reason_examples(ix).reason)
            fprintf('           %s\n',S.reason_examples(ix).reason);
        end
    end
end
end

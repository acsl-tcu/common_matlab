function C=CHECK_TRIGGER_CAPTURE_RUN(logger,replanner_instance,run_id)
% One completed Flight -> immutable, detailed offline record.
% Call separately after each actual run; no runner or logger modification.
% Explicit run_id=1,2,3 prevents accidental mixing of older captures.
assert(nargin==3 && isnumeric(run_id) && isscalar(run_id) && ...
       ismember(double(run_id),1:3), ...
       'CHECK_TRIGGER_CAPTURE_RUN:RunID','Use 1, 2, or 3.');
assert(isobject(logger) && isobject(replanner_instance), ...
       'CHECK_TRIGGER_CAPTURE_RUN:Inputs','Pass live logger and replanner_instance.');
D=logger.Data;
assert(isstruct(D)&&isfield(D,'agent')&&isfield(D,'t')&& ...
       isfield(D.agent(1),'reference')&& ...
       isfield(D.agent(1).reference,'result'), ...
       'CHECK_TRIGGER_CAPTURE_RUN:Logger','Unexpected original LOGGER data.');
R=D.agent(1).reference.result;
if ~iscell(R),R=num2cell(R(:));else,R=R(:);end
n=min([numel(R),numel(D.t),double(logger.k)]);
assert(n>0,'CHECK_TRIGGER_CAPTURE_RUN:Empty');
C=struct();C.version='TRIGGER_DIAG_V3_SENSOR';C.run_id=double(run_id);C.n=n;
C.time_s=double(D.t(1:n));C.time_s=C.time_s(:);
keys={'replanner_ms','trigger_ms','sensor_ms','memory_ms', ...
    'predictor_ms','reference_ms','hlc_ms','plant_ms','setup_ms', ...
    'geometry_ms','shadow_ms','rhs_evals','rejected_steps','body_exact_queries', ...
    'cable_exact_queries','support_exclusions','obstacle_count', ...
    'detected_count','new_count','retained_count','stage', ...
    'memory_stage','geometry_stage', ...
    'cache_reset','cache_fresh','cache_hits','cache_calls', ...
    'cache_source_loop_ms','cache_wall_ms'};
for i=1:numel(keys)
    k=keys{i};
    if strcmp(k,'stage')
        C.stage=nan(n,6);
    elseif any(strcmp(k,{'memory_stage','geometry_stage'}))
        C.(k)=nan(n,4); % three counters plus unused diagnostic slot
    else
        C.(k)=nan(n,1);
    end
end
C.sensor_stage=nan(n,5);C.sensor_exact_calls=nan(n,1);
C.sensor_candidates=nan(n,1);C.sensor_screened=nan(n,1);
C.sensor_mask_changed=nan(n,1);C.sensor_native_events=nan(n,1);
C.detail=nan(n,6);C.predicted=false(n,1);C.complete=false(n,1);
C.blue_other_ms=nan(n,1);C.trigger_outer_ms=nan(n,1);
C.new_event=false(n,1);C.duplicate=false(n,1);
C.sampled_contact=false(n,1);C.detected_changed=false(n,1);
C.known_ids=cell(n,1);C.detected_ids=cell(n,1);C.new_ids=cell(n,1);
C.trigger_status=cell(n,1);C.trigger_reason=cell(n,1);
for j=1:n
    x=R{j};
    C.replanner_ms(j)=numv(x,'replanning_time_ms');
    C.trigger_ms(j)=numv(x,'trigger_time_ms');
    C.stage(j,:)=sixv(x,'replanning_stage_ms');
    C.detail(j,:)=sixv(x,'replanning_detail_ms');
    s=fld(x,'simulated_obstacle_sensor');
    m=fld(x,'observed_obstacle_memory');
    q=fld(x,'future_collision_trigger');
    C.sensor_ms(j)=numv(s,'elapsed_ms');
    C.sensor_stage(j,:)=fivev(s,'diagnostic_sensor_stage_ms');
    C.sensor_exact_calls(j)=numv(s,'diagnostic_sensor_exact_calls');
    C.sensor_candidates(j)=numv(s,'diagnostic_sensor_candidate_count');
    C.sensor_screened(j)=numv(s,'diagnostic_sensor_screened_count');
    C.sensor_mask_changed(j)=numv(s,'diagnostic_sensor_mask_changed');
    C.sensor_native_events(j)=numv(s,'diagnostic_sensor_native_events');
    C.memory_ms(j)=numv(m,'elapsed_ms');
    C.memory_stage(j,1:4)=fourv(m,'diagnostic_stage_ms');
    C.detected_ids{j}=vec(s,'detected_ids');
    C.known_ids{j}=vec(m,'known_obstacle_ids');
    C.new_ids{j}=vec(m,'newly_acquired_ids');
    C.predicted(j)=yes(q,'prediction_available');
    C.complete(j)=yes(q,'prediction_complete');
    C.new_event(j)=yes(q,'new_event');
    C.duplicate(j)=yes(q,'duplicate_suppressed');
    C.sampled_contact(j)=yes(q,'sampled_collision');
    C.obstacle_count(j)=numv(q,'considered_count');
    C.detected_count(j)=numel(C.detected_ids{j});
    C.new_count(j)=numel(C.new_ids{j});
    C.retained_count(j)=numel(C.known_ids{j})-C.detected_count(j);
    C.trigger_status{j}=strv(q,'status');
    C.trigger_reason{j}=strv(q,'reason');
    C.predictor_ms(j)=numv(q,'predictor_wall_ms');
    C.reference_ms(j)=numv(q,'predictor_reference_window_ms');
    C.hlc_ms(j)=numv(q,'predictor_hlc_time_ms');
    C.plant_ms(j)=numv(q,'predictor_plant_time_ms');
    C.setup_ms(j)=numv(q,'predictor_setup_time_ms');
    C.geometry_ms(j)=numv(q,'geometry_wall_ms');
    C.geometry_stage(j,1:3)=threev(q,'geometry_diagnostic_stage_ms');
    C.cache_reset(j)=numv(q,'reference_cache_reset');
    C.cache_fresh(j)=numv(q,'reference_cache_fresh');
    C.cache_hits(j)=numv(q,'reference_cache_hits');
    C.cache_calls(j)=numv(q,'reference_cache_source_calls');
    C.cache_source_loop_ms(j)=numv(q,'reference_cache_source_loop_ms');
    C.cache_wall_ms(j)=numv(q,'reference_cache_wall_ms');
    C.blue_other_ms(j)=C.replanner_ms(j)-C.trigger_ms(j)- ...
        C.sensor_ms(j)-C.memory_ms(j);
    C.trigger_outer_ms(j)=C.trigger_ms(j)-C.predictor_ms(j)- ...
        C.geometry_ms(j);
    C.shadow_ms(j)=numv(x,'replanning_shadow_ms');
    C.rhs_evals(j)=numv(q,'predictor_rhs_evaluations');
    C.rejected_steps(j)=numv(q,'predictor_rejected_steps');
    C.body_exact_queries(j)=numv(q,'geometry_exact_body_queries');
    C.cable_exact_queries(j)=numv(q,'geometry_exact_cable_queries');
    C.support_exclusions(j)=numv(q,'geometry_support_exclusions');
    if j>1
        C.detected_changed(j)=~isequal(C.detected_ids{j},C.detected_ids{j-1});
    end
end
C.acquired=C.new_count>0;
C.sensor_transition=C.detected_changed;
C.first_prediction=find(C.predicted,1,'first');
if isempty(C.first_prediction),C.first_prediction=NaN;end
C.first_contact=find(C.sampled_contact,1,'first');
if isempty(C.first_contact),C.first_contact=NaN;end
C.first_new_event=find(C.new_event,1,'first');
if isempty(C.first_new_event),C.first_new_event=NaN;end
C.stage_recorded=any(isfinite(C.stage(:))) && any(isfinite(C.detail(:)));
C.n_predictions=nnz(C.predicted);
C.n_completed=nnz(C.predicted & C.complete);
C.n_geometry_binding_failures=nnz(strcmp( ...
    C.trigger_status,'UNKNOWN_OBSERVED_GEOMETRY_BINDING'));
C.status='CAPTURED_NOMINAL_ONLY__NOT_FLIGHT_SAFETY';
if C.n_geometry_binding_failures>0
    C.status='FAIL_OBSERVED_GEOMETRY_BINDING__DO_NOT_BENCHMARK';
elseif C.n_predictions==0
    C.status='FAIL_ZERO_PREDICTIONS__DO_NOT_BENCHMARK';
elseif C.n_completed<C.n_predictions
    C.status='FAIL_INCOMPLETE_PREDICTIONS__DO_NOT_BENCHMARK';
elseif ~C.stage_recorded
    C.status='CAPTURED_WITHOUT_DETAILED_STAGE__SET_ENV_BEFORE_FLIGHT';
elseif ~any(isfinite(C.sensor_stage(:)))
    C.status='CAPTURED_WITHOUT_SENSOR_INTERNAL_STAGE__SOURCE_OR_ENV_MISMATCH';
end
% Do not query status to collect timings: a stale class previously broke
% this call. A separate interface check handles the class-version issue.
name=sprintf('TRIGGER_DIAG_RUN_%02d',C.run_id);
save([name '.mat'],'C','-v7.3');
fid=fopen([name '.txt'],'w');
assert(fid>=0,'CHECK_TRIGGER_CAPTURE_RUN:Write');
clean=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'status=%s\nrun=%d\nframes=%d\nstage_recorded=%d\n', ...
    C.status,C.run_id,n,C.stage_recorded);
fprintf(fid,'first_prediction_index=%.0f\n',C.first_prediction);
fprintf(fid,'prediction_count=%d\nprediction_complete_count=%d\n', ...
    C.n_predictions,C.n_completed);
fprintf(fid,'geometry_binding_failures=%d\n', ...
    C.n_geometry_binding_failures);
if C.n_geometry_binding_failures>0
    k=find(strcmp(C.trigger_status,'UNKNOWN_OBSERVED_GEOMETRY_BINDING'),1);
    fprintf(fid,'first_geometry_binding_failure_k=%d\n',k);
    fprintf(fid,'first_geometry_binding_reason=%s\n', ...
        regexprep(C.trigger_reason{k},'[\r\n]+',' '));
end
fprintf(fid,'observed_acquisitions=%d\ndetection_changes=%d\n', ...
    nnz(C.acquired),nnz(C.sensor_transition));
fprintf(fid,'prediction_rhs_available=%d\nrejected_steps_available=%d\n', ...
    nnz(isfinite(C.rhs_evals)),nnz(isfinite(C.rejected_steps)));
fprintf(fid,'NOTE: time measurements nested in different sources can overlap.\n');
fprintf(fid,'NOTE: high residual alone cannot identify JIT, GC, OS scheduling, or copies.\n');
fprintf(fid,'NOTE: event-correlated is NOT proof of causal implementation mechanism.\n');
fprintf(fid,'NOTE: predictor RHS/rejection metrics are logged only when exposed by live MEX.\n');
fprintf(fid,'Deep: memory=validation, topology, refresh, bookkeeping; geometry=broad,support,exact.\n');
fprintf(fid,'Deep reference: fresh/shift and measured source loop separate; full wall time may include scheduling.\n');
rank=C.replanner_ms;rank(~isfinite(rank))=-Inf;
[~,order]=sort(rank,'descend');
order=order(1:min(35,n));
fprintf(fid,'\nTOP_35_UPDATE\n');
fprintf(fid,['k,t,update,trigger,sensor,memory,detail_acquisition,detail_attach,' ...
   'pred,reference,hlc,plant,setup,geometry,shadow,rhs,rejected,body_q,cable_q,' ...
   'support_excl,known,detected,new_count,sensor_changed,new_event,duplicate,' ...
   'blue_other,trigger_outer,stage_input,stage_ekf,stage_sens_mem,' ...
   'stage_log,stage_attach,stage_bookkeeping,status\n']);
for k=reshape(order,1,[])
    fprintf(fid,['%d,%.9f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,' ...
        '%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.0f,%.0f,%.0f,%.0f,' ...
        '%.0f,%d,%d,%d,%d,%d,%d,' ...
        '%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%s\n'], ...
        k,C.time_s(k),C.replanner_ms(k),C.trigger_ms(k), ...
        C.sensor_ms(k),C.memory_ms(k),C.detail(k,3),C.detail(k,6), ...
        C.predictor_ms(k),C.reference_ms(k),C.hlc_ms(k),C.plant_ms(k), ...
        C.setup_ms(k),C.geometry_ms(k),C.shadow_ms(k), ...
        C.rhs_evals(k),C.rejected_steps(k), ...
        C.body_exact_queries(k),C.cable_exact_queries(k), ...
        C.support_exclusions(k),numel(C.known_ids{k}), ...
        numel(C.detected_ids{k}),C.new_count(k),C.detected_changed(k), ...
        C.new_event(k),C.duplicate(k), ...
        C.blue_other_ms(k),C.trigger_outer_ms(k), ...
        C.stage(k,1),C.stage(k,2),C.stage(k,3), ...
        C.stage(k,4),C.stage(k,5),C.stage(k,6), ...
        C.trigger_status{k});
end
fprintf(fid,'\nTOP_35_SENSOR\n');
fprintf(fid,['k,t,sensor_total,scan_including_exact,exact_nested,event_ids,', ...
    'visible,pack_and_book,exact_calls,candidates,screened,', ...
    'mask_changed,native_events,known,detected\n']);
rank_sensor=C.sensor_ms;rank_sensor(~isfinite(rank_sensor))=-Inf;
[~,top_sensor]=sort(rank_sensor,'descend');
top_sensor=top_sensor(1:min(35,n));
for k=reshape(top_sensor,1,[])
    fprintf(fid,['%d,%.9f,%.6f,%.6f,%.6f,%.6f,%.6f,', ...
        '%.6f,%.0f,%.0f,%.0f,%.0f,%.0f,%d,%d\n'], ...
        k,C.time_s(k),C.sensor_ms(k),C.sensor_stage(k,:), ...
        C.sensor_exact_calls(k),C.sensor_candidates(k), ...
        C.sensor_screened(k),C.sensor_mask_changed(k), ...
        C.sensor_native_events(k),numel(C.known_ids{k}), ...
        numel(C.detected_ids{k}));
end
fprintf(fid,'NOTE: sensor exact_ms is NESTED within scan_ms. Do not sum these.\n');
fprintf(fid,'\nTOP_35_DEEP\n');
fprintf(fid,['k,mem_check,mem_topology,mem_refresh,mem_bookkeeping,', ...
    'geom_broad,geom_support,geom_exact,', ...
    'reference_cache_reset,ref_fresh,ref_hits,ref_calls,ref_loop_ms,ref_wall_ms\n']);
for k=reshape(order,1,[])
    fprintf(fid,'%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.0f,%.0f,%.0f,%.0f,%.6f,%.6f\n', ...
        k,C.memory_stage(k,1:4),C.geometry_stage(k,1:3), ...
        C.cache_reset(k),C.cache_fresh(k),C.cache_hits(k), ...
        C.cache_calls(k),C.cache_source_loop_ms(k),C.cache_wall_ms(k));
end
fprintf('Captured run %d: %s.mat / .txt; stage=%d\n', ...
    C.run_id,name,C.stage_recorded);
end
function x=fld(s,k)
x=[];if isstruct(s)&&isscalar(s)&&isfield(s,k),x=s.(k);end
end
function x=numv(s,k)
x=fld(s,k);if ~((isnumeric(x)||islogical(x))&&isscalar(x)&&isreal(x))
    x=NaN;
else
    x=double(x);
end
end
function x=fivev(s,k)
x=fld(s,k);
if ~(isnumeric(x)&&isreal(x)&&numel(x)==5),x=nan(1,5);
else,x=reshape(double(x),1,5);end
end
function x=fourv(s,k)
x=fld(s,k);
if ~(isnumeric(x)&&isreal(x)&&numel(x)==4),x=nan(1,4);
else,x=reshape(double(x),1,4);end
end
function x=threev(s,k)
x=fld(s,k);
if ~(isnumeric(x)&&isreal(x)&&numel(x)==3),x=nan(1,3);
else,x=reshape(double(x),1,3);end
end
function x=sixv(s,k)
x=fld(s,k);
if ~(isnumeric(x)&&isreal(x)&&numel(x)==6)
    x=nan(1,6);
else
    x=reshape(double(x),1,6);
end
end
function x=vec(s,k)
x=fld(s,k);
if ~(isnumeric(x)&&isreal(x)&&isvector(x)&&all(isfinite(x(:))))
    x=zeros(1,0);
else
    x=reshape(double(x),1,[]);
end
end
function t=yes(s,k)
x=fld(s,k);
t=isscalar(x)&&(isnumeric(x)||islogical(x))&&isequal(double(x),1);
end
function x=strv(s,k)
x=fld(s,k);
if isstring(x)&&isscalar(x),x=char(x);end
if ~ischar(x),x='';end
end

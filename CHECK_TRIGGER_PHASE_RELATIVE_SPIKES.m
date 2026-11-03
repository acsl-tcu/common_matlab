function A=CHECK_TRIGGER_PHASE_RELATIVE_SPIKES(logger,replanner_instance)
% Offline analysis of genuine logged ticks. No assumptions about the
% Arming/Takeoff/Flight transition clock. First PREDICTED tick is NOT
% claimed to equal the first Flight tick when known obstacles are absent.
% No new scans, flight-loop edits, HLC/plant calls or extra rollouts.
A=struct('status','NOT_RUN','reason','', ...
    'first_prediction_time_s',NaN,'first_prediction_index',NaN, ...
    'predicted_count',0,'full_four_second_count',0, ...
    'first_prediction_ms',NaN,'first_eight_max_ms',NaN, ...
    'later_nonacquisition_max_ms',NaN, ...
    'later_nonacquisition_p95_ms',NaN, ...
    'phase_clock_known',false,'top_nonacquisition',[], ...
    'safe_to_commit',false);
try
    Q=CHECK_TRIGGER_FLIGHT_SPIKES(logger,replanner_instance);
    n=double(Q.n);
    t=double(Q.time_s(:));rp=double(Q.replanner_ms(:));
    tr=double(Q.trigger_ms(:));
    pred=logical(Q.prediction_available(:));
    complete=logical(Q.prediction_complete(:));
    ids=Q.new_ids;
    if iscell(ids)&&numel(ids)==1&&iscell(ids{1}),ids=ids{1};end
    assert(iscell(ids)&&numel(ids)==n && ...
        numel(t)==n&&numel(rp)==n&&numel(tr)==n, ...
        'CHECK_TRIGGER_PHASE_RELATIVE_SPIKES:Log', ...
        'Missing aligned per-tick timing / acquisition history.');
    acq=~cellfun(@isempty,ids(:));
    predicted=find(pred);
    assert(~isempty(predicted), ...
        'CHECK_TRIGGER_PHASE_RELATIVE_SPIKES:Prediction', ...
        'No Flight predictor ran. Cannot identify first predicted tick.');
    start=predicted(1);
    A.first_prediction_index=start;
    A.first_prediction_time_s=t(start);
    A.predicted_count=nnz(pred);
    A.full_four_second_count=nnz(complete & ...
        abs(double(Q.completed_horizon_s(:))-4)<1e-9);
    A.first_prediction_ms=rp(start);
    first8=predicted(1:min(8,numel(predicted)));
    A.first_eight_max_ms=max(rp(first8));
    later=pred;
    later(first8)=false;
    later=later & ~acq;
    late_idx=find(later & isfinite(rp));
    if ~isempty(late_idx)
        a=sort(rp(late_idx));
        A.later_nonacquisition_max_ms=a(end);
        A.later_nonacquisition_p95_ms=a(max(1,ceil(.95*numel(a))));
        [~,order]=sort(rp(late_idx),'descend');
        A.top_nonacquisition=late_idx(order(1:min(numel(order),12)));
    end
    % Diagnosis consumes the ACTUAL same-tick LOGGER result objects, so
    % it can report geometry/nominal reference cost without re-execution.
    D=logger.Data;
    X=D.agent(1).reference.result;
    if ~iscell(X),X=num2cell(X(:));else,X=X(:);end
    A.status='PASS_PHASE_RELATIVE_LOG_DECOMPOSITION__NOT_JIT_PROOF';
    fid=fopen('TRIGGER_PHASE_RELATIVE_SPIKES_RESULT.txt','w');
    assert(fid>=0, 'CHECK_TRIGGER_PHASE_RELATIVE_SPIKES:Output', ...
        'Could not open offline diagnostic result.');
    fclean=onCleanup(@()fclose(fid)); %#ok<NASGU>
    fprintf(fid,'source=ACTUAL_LOGGER__NO_FLIGHT_START_ASSUMPTION\n');
    fprintf(fid,'first_predicted_index=%d\nfirst_predicted_time_s=%.9f\n',start,t(start));
    fprintf(fid,'prediction_count=%d\nfull_4s_count=%d\n', ...
        A.predicted_count,A.full_four_second_count);
    fprintf(fid,'first_predicted_replanner_ms=%.6f\nfirst_8_predicted_max_ms=%.6f\n', ...
        A.first_prediction_ms,A.first_eight_max_ms);
    fprintf(fid,'later_without_new_id_max_ms=%.6f\n',A.later_nonacquisition_max_ms);
    fprintf(fid,'later_without_new_id_p95_ms=%.6f\n',A.later_nonacquisition_p95_ms);
    fprintf(fid,['Note: first PREDICTED time is not asserted to equal' ...
        ' actual Flight entry if no obstacles are yet known.\n']);
    fprintf(fid,['Note: timings from different nested fields can overlap;' ...
        ' do not sum or subtract as disjoint costs.\n']);
    fprintf(fid,['Note: no cause of JIT or OS scheduling jitter can be' ...
        ' established by durations alone.\n']);
    fprintf(fid,'\n--- largest later spikes WITHOUT new-obstacle acquisition ---\n');
    fprintf(fid,'index t[s] dt_since_first_pred[s] replanner[ms] trigger[ms] geometry[ms] ref_window[ms] predictor_setup[ms] hlc_native[ms] plant_native[ms] status\n');
    for k=reshape(A.top_nonacquisition,1,[])
        j=X{k};
        v=getfield_or(j,'future_collision_trigger'); %#ok<GFLD>
        fprintf(fid,'%d %.9f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %s\n', ...
            k,t(k),t(k)-t(start),rp(k),tr(k), ...
            scalar_or(v,'geometry_wall_ms'), ...
            scalar_or(v,'predictor_reference_window_ms'), ...
            scalar_or(v,'predictor_setup_time_ms'), ...
            scalar_or(v,'predictor_hlc_time_ms'), ...
            scalar_or(v,'predictor_plant_time_ms'), ...
            string_or(v,'status'));
    end
    fprintf(fid,'status=%s\nreason=%s\n',A.status,A.reason);
catch ME
    A.status='NOT_COMPLETED_PHASE_RELATIVE_LOG_AUDIT';
    A.reason=[ME.identifier ' | ' ME.message];
end
if ~strcmp(A.status,'PASS_PHASE_RELATIVE_LOG_DECOMPOSITION__NOT_JIT_PROOF')
    fid=fopen('TRIGGER_PHASE_RELATIVE_SPIKES_RESULT.txt','w');
    if fid>=0
        fprintf(fid,'status=%s\nreason=%s\n',A.status,A.reason);
        fclose(fid);
    end
end
save('TRIGGER_PHASE_RELATIVE_SPIKES_RESULT.mat','A');
end
function v=getfield_or(s,key)
v=struct();
if isstruct(s)&&isscalar(s)&&isfield(s,key),v=s.(key);end
end
function v=scalar_or(s,key)
v=NaN;
if isstruct(s)&&isscalar(s)&&isfield(s,key)
    x=s.(key);
    if isnumeric(x)&&isscalar(x)&&isreal(x)&&isfinite(x)
        v=double(x);
    end
end
end
function v=string_or(s,key)
v='';
if isstruct(s)&&isscalar(s)&&isfield(s,key)
    x=s.(key);
    if ischar(x),v=x;elseif isstring(x)&&isscalar(x),v=char(x);end
end
end

function O=CHECK_TRIGGER_RUNTIME_PHASE_PEAKS(logger)
% COMMON_ROOT disposable audit. Uses the ORIGINAL Sim LOGGER stored frames;
% never changes the runner, controller, EKF, shape state, Flight or events.
% Nested wall clocks are NOT CPU/WCET or proof of scheduler origin.
root=fileparts(mfilename('fullpath'));
fields={'timing_runtime_reference_now_ms', ...
    'timing_runtime_authority_ms','timing_runtime_core_ms', ...
    'timing_runtime_total_ms','timing_pre_predictor_ms', ...
    'predictor_wall_ms','timing_between_predictor_geometry_ms', ...
    'geometry_wall_ms','timing_post_geometry_ms', ...
    'geometry_sampled_block_support_exclusions', ...
    'geometry_sampled_block_support_probes'};
O=struct('status','UNKNOWN_RUNTIME_PHASE_LOG', ...
    'reason','','record_count',0,'complete_count',0, ...
    'phase_records',0,'max_trigger_ms',NaN, ...
    'top',zeros(0,numel(fields)+4),'column_labels',{{}}, ...
    'wall_clock_only',true,'proves_cause',false, ...
    'proves_wcet',false);
try
    assert(isobject(logger)&&isprop(logger,'Data') && ...
        isstruct(logger.Data)&&isfield(logger.Data,'agent') && ...
        ~isempty(logger.Data.agent) && ...
        isfield(logger.Data.agent(1),'reference') && ...
        isfield(logger.Data.agent(1).reference,'result'), ...
        'CHECK_TRIGGER_RUNTIME:Logger','Original LOGGER.Data.agent(1).reference.result required.');
    records=logger.Data.agent(1).reference.result;
    if ~iscell(records),records=num2cell(records(:));end
    records=records(:);N=numel(records);
    % k,time,trigger_ms,outer_ms, followed by original field values.
    X=nan(N,4+numel(fields));
    for k=1:N
        rr=records{k};
        if ~isstruct(rr)||~isscalar(rr) || ...
                ~isfield(rr,'future_collision_trigger')
            continue
        end
        f=rr.future_collision_trigger;
        if ~isstruct(f)||~isscalar(f),continue;end
        if ~isfield(f,'prediction_complete') || ...
                ~isequal(f.prediction_complete,true)
            continue
        end
        O.complete_count=O.complete_count+1;
        X(k,1)=k;
        if isfield(f,'time_s') && isnumeric(f.time_s) && ...
                isscalar(f.time_s)
            X(k,2)=double(f.time_s);
        end
        if isfield(rr,'trigger_time_ms') && isnumeric(rr.trigger_time_ms) && ...
                isscalar(rr.trigger_time_ms)
            X(k,3)=double(rr.trigger_time_ms);
        end
        for j=1:numel(fields)
            if isfield(f,fields{j})
                v=f.(fields{j});
                if isnumeric(v)&&isreal(v)&&isscalar(v)&&isfinite(v)
                    X(k,4+j)=double(v);
                end
            end
        end
        if isfinite(X(k,3)) && isfinite(X(k,8))
            X(k,4)=X(k,3)-X(k,8); % outer = total - runtime_total
        end
    end
    valid=isfinite(X(:,3));
    phase=isfinite(X(:,5))&isfinite(X(:,6))& ...
          isfinite(X(:,7))&isfinite(X(:,8));
    O.record_count=nnz(valid);
    O.phase_records=nnz(phase);
    O.column_labels=[{'k','time_s','trigger_ms','outer_ms'},fields];
    if nnz(valid)<1 || nnz(phase)<1
        O.reason='No completed Flight frames with new phase timing. Enable REPLANNING_STAGE_TIMING=1 and run one Flight.';
    else
        X=X(valid,:);
        [~,ord]=sort(X(:,3),'descend');
        O.top=X(ord(1:min(25,numel(ord))),:);
        O.max_trigger_ms=max(X(:,3));
        O.status='PASS_RUNTIME_PHASES_RECORDED__NO_CAUSAL_OR_WCET_PROOF';
    end
catch ME
    O.reason=[ME.identifier ' | ' ME.message];
end
save(fullfile(root,'TRIGGER_RUNTIME_PHASE_PEAKS_RESULT.mat'),'O');
f=fopen(fullfile(root,'TRIGGER_RUNTIME_PHASE_PEAKS_RESULT.txt'),'w');
if f>=0
    fprintf(f,'status=%s\nrecord_count=%d\ncomplete_count=%d\nphase_records=%d\n', ...
        O.status,O.record_count,O.complete_count,O.phase_records);
    fprintf(f,'max_trigger_ms=%.7f\n',O.max_trigger_ms);
    fprintf(f,'columns=%s\n',strjoin(O.column_labels,','));
    for j=1:size(O.top,1)
        fprintf(f,'%g',O.top(j,1));
        fprintf(f,',%.8g',O.top(j,2:end));
        fprintf(f,'\n');
    end
    fprintf(f,'reason=%s\n',O.reason);fclose(f);
end
end

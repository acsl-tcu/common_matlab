function A = CHECK_TRIGGER_FLIGHT_ACTIVE(logger,replanner_instance)
% Non-negotiable functional guard after one genuine Flight. The previous
% deep diagnostic erroneously treated a predictor-free Flight as fast.
% Does not declare closed-loop safety; checks only true 4s model execution.
A=struct('status','FAIL_NOT_EVALUATED','reason','', ...
    'frames',0,'predicted',0,'completed',0,'binding_unknown',0, ...
    'incomplete',0,'first_prediction_index',NaN,'max_rhs',NaN);
try
    assert(isobject(logger)&&isobject(replanner_instance), ...
        'CHECK_TRIGGER_FLIGHT_ACTIVE:Inputs', ...
        'Supply live original logger and replanner instance.');
    D=logger.Data;
    assert(isstruct(D) && isfield(D,'agent') && ...
        isfield(D.agent(1),'reference') && ...
        isfield(D.agent(1).reference,'result'), ...
        'CHECK_TRIGGER_FLIGHT_ACTIVE:Logger', ...
        'Original Sim logger has no reference results.');
    R=D.agent(1).reference.result;
    if ~iscell(R),R=num2cell(R(:));else,R=R(:);end
    n=min([numel(R),numel(D.t),double(logger.k)]);
    A.frames=n;
    rhs=[];n_horizon_mismatch=0;
    for k=1:n
        x=R{k};
        if ~isstruct(x)||~isscalar(x) || ...
                ~isfield(x,'future_collision_trigger'),continue;end
        q=x.future_collision_trigger;
        if ~isstruct(q)||~isscalar(q),continue;end
        if isfield(q,'status') && ...
                strcmp(char(q.status),'UNKNOWN_OBSERVED_GEOMETRY_BINDING')
            A.binding_unknown=A.binding_unknown+1;
            if isempty(A.reason) && isfield(q,'reason')
                A.reason=char(q.reason);
            end
        end
        predicted=isfield(q,'prediction_available') && ...
            isequal(q.prediction_available,true);
        if predicted
            A.predicted=A.predicted+1;
            if isnan(A.first_prediction_index),A.first_prediction_index=k;end
            complete=isfield(q,'prediction_complete') && ...
                isequal(q.prediction_complete,true);
            if complete,A.completed=A.completed+1;
            else,A.incomplete=A.incomplete+1;end
            if isfield(q,'predictor_rhs_evaluations') && ...
                    isnumeric(q.predictor_rhs_evaluations) && ...
                    isscalar(q.predictor_rhs_evaluations) && ...
                    isfinite(q.predictor_rhs_evaluations)
                rhs(end+1)=double(q.predictor_rhs_evaluations); %#ok<AGROW>
            end
            if isfield(q,'horizon_s') && ...
                    isfield(q,'completed_prediction_horizon_s') && ...
                    double(q.completed_prediction_horizon_s) + ...
                    128*eps(max(1,double(q.horizon_s))) < double(q.horizon_s)
                n_horizon_mismatch=n_horizon_mismatch+1;
            end
        end
    end
    if ~isempty(rhs),A.max_rhs=max(rhs);end
    if A.binding_unknown>0
        A.status='FAIL_OBSERVED_MEMORY_TRIGGER_API';
    elseif A.predicted==0
        A.status='FAIL_ZERO_FOUR_SECOND_PREDICTIONS';
        if isempty(A.reason)
            A.reason='No genuine 4s nominal predictions occurred; do not compare speeds.';
        end
    elseif A.incomplete>0 || n_horizon_mismatch>0
        A.status='FAIL_INCOMPLETE_PREDICTIONS';
        A.reason=sprintf('incomplete=%d short_horizon=%d', ...
            A.incomplete,n_horizon_mismatch);
    else
        A.status='PASS_FOUR_SECOND_PREDICTION_RUNNING__NOT_SAFETY_CERTIFICATE';
    end
catch ME
    A.status='FAIL_FLIGHT_LOG_AUDIT';
    A.reason=sprintf('%s: %s',ME.identifier,ME.message);
end
fid=fopen('TRIGGER_FLIGHT_ACTIVE_RESULT.txt','w');
if fid>=0
    cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
    fprintf(fid,'status=%s\nreason=%s\n',A.status,A.reason);
    fprintf(fid,'frames=%d\npredicted=%d\ncompleted=%d\n', ...
        A.frames,A.predicted,A.completed);
    fprintf(fid,'binding_unknown=%d\nincomplete=%d\nmax_rhs=%.0f\n', ...
        A.binding_unknown,A.incomplete,A.max_rhs);
    fprintf(fid,'first_prediction_index=%.0f\n',A.first_prediction_index);
end
end

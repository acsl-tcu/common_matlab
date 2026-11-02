function R = CHECK_TRIGGER_EXECUTION_MODE(logger,replanner_instance,label)
% READ-ONLY. Inspect completed Sim LOGGER to distinguish a genuine 4-s
% rollout from a fast fail-closed/disabled trigger. No sensor scan, EKF,
% HLC, plant, new rollout, or changes to runtime objects. The source
% check reads local source/MEX/audit file hashes OFFLINE, after the Flight.
if nargin<2, replanner_instance=[]; end
if nargin<3 || isempty(label), label='flight'; end
label=regexprep(char(string(label)),'[^A-Za-z0-9_-]','_');
if isempty(label),label='flight';end
assert(isobject(logger) && isprop(logger,'Data') && isprop(logger,'k'), ...
    'CHECK_TRIGGER_EXECUTION_MODE:Logger','Completed Sim LOGGER is required.');
D=logger.Data;
assert(isfield(D,'agent') && ~isempty(D.agent) && ...
    isfield(D.agent(1),'reference') && ...
    isfield(D.agent(1).reference,'result'), ...
    'CHECK_TRIGGER_EXECUTION_MODE:Schema','No reference.result in LOGGER.');
C=D.agent(1).reference.result;
if ~iscell(C),C=num2cell(C);end
n=min(numel(C),double(logger.k));
status=repmat({''},n,1);
phase=repmat({''},n,1);
elapsed=nan(n,1);complete=false(n,1);reported=false(n,1);
times=nan(n,1);reason=repmat({''},n,1);
for k=1:n
    if isfield(D,'t') && numel(D.t)>=k && isnumeric(D.t(k))
        times(k)=double(D.t(k));
    end
    if isfield(D,'phase') && numel(D.phase)>=k
        ph=D.phase(k);
        if iscell(ph),ph=ph{1};end
        phase{k}=char(ph);
    end
    a=C{k};
    if ~isstruct(a) || ~isscalar(a),continue;end
    if isfield(a,'trigger_time_ms') && isnumeric(a.trigger_time_ms) && ...
            isscalar(a.trigger_time_ms) && isfinite(a.trigger_time_ms)
        elapsed(k)=double(a.trigger_time_ms);
    end
    if ~isfield(a,'future_collision_trigger'),continue;end
    q=a.future_collision_trigger;
    if ~isstruct(q) || ~isscalar(q),continue;end
    reported(k)=true;
    if isfield(q,'status'),status{k}=char(string(q.status));end
    if isfield(q,'reason'),reason{k}=char(string(q.reason));end
    if isfield(q,'prediction_complete') && isscalar(q.prediction_complete)
        complete(k)=isequal(logical(q.prediction_complete),true);
    end
end
names=unique(status(reported),'stable');
counts=zeros(size(names));
for k=1:numel(names),counts(k)=sum(strcmp(status,names{k}));end
fullph=strcmp(phase,'f');
measure=elapsed(isfinite(elapsed));
R=struct();
R.status='READ_ONLY_EXECUTION_MODE_AUDIT__NOT_A_SAFETY_PROOF';
R.records=n;
R.trigger_result_records=nnz(reported);
R.prediction_complete_count=nnz(complete);
R.f_phase_records=nnz(fullph);
R.f_phase_prediction_complete_count=nnz(complete & fullph);
R.f_phase_without_prediction_count=nnz(fullph & ~complete);
R.trigger_median_ms=local_median(measure);
R.trigger_max_ms=local_max(measure);
R.status_names=names;
R.status_counts=counts;
R.first_f_phase_status='NO_F_PHASE_RECORDED';
k=find(fullph,1,'first');
if ~isempty(k),R.first_f_phase_status=status{k};end
R.first_f_phase_time_s=NaN;
if ~isempty(k),R.first_f_phase_time_s=times(k);end
R.first_f_phase_reason='';
if ~isempty(k),R.first_f_phase_reason=reason{k};end
R.configured_trigger=NaN;
R.runtime_gate_status='UNAVAILABLE';
R.preflight_ready=NaN;
if nargin>=2 && ~isempty(replanner_instance)
    try
        q=replanner_instance.get_status();
        if isfield(q,'ENABLE_TRIGGER'),R.configured_trigger=q.ENABLE_TRIGGER;end
        if isfield(q,'trigger_native_gate_status')
            R.runtime_gate_status=char(string(q.trigger_native_gate_status));
        end
        if isfield(q,'preflight_update_ready')
            R.preflight_ready=q.preflight_update_ready;
        end
    catch ME
        R.runtime_gate_status=['UNAVAILABLE: ' ME.identifier];
    end
end
R.source_pin_status='UNAVAILABLE';
R.source_pin_pass=NaN;
R.offline_parity_pass=NaN;
R.native_mex_sha256='';
R.native_mex_path=which('TRIGGER_SYMBOLIC_NATIVE_CORE');
if exist('TRIGGER_NATIVE_MODEL_SOURCE_CHECK','file')==2
    try
        q=TRIGGER_NATIVE_MODEL_SOURCE_CHECK(true);
        R.source_pin_status=char(string(q.status));
        R.source_pin_pass=logical(q.passed);
        R.offline_parity_pass=logical(q.audit_pass);
        R.native_mex_sha256=char(string(q.native_mex_sha256));
    catch ME
        R.source_pin_status=['CHECK_FAILED: ' ME.identifier];
    end
end
if R.prediction_complete_count==0
    R.interpretation='NO_COMPLETED_PREDICTIONS: A low elapsed time is NOT evidence of a faster four-second trigger.';
elseif R.f_phase_without_prediction_count>0
    R.interpretation='MIXED: some f-phase ticks did not complete the predictor; inspect statuses.';
else
    R.interpretation='PREDICTIONS_COMPLETED: compare timing on the same workload; safety remains uncertified.';
end
root=fileparts(mfilename('fullpath'));
base=fullfile(root,['TRIGGER_EXECUTION_MODE_' label]);
save([base '.mat'],'R','-v7');
fid=fopen([base '.txt'],'wt');
if fid>=0
    closer=onCleanup(@()fclose(fid));
    fprintf(fid,'status=%s\nrecords=%d\nf_phase=%d\ncompleted=%d\nf_phase_completed=%d\nf_phase_incomplete=%d\n', ...
        R.status,R.records,R.f_phase_records,R.prediction_complete_count, ...
        R.f_phase_prediction_complete_count,R.f_phase_without_prediction_count);
    fprintf(fid,'trigger_median_ms=%.6f\ntrigger_max_ms=%.6f\n',R.trigger_median_ms,R.trigger_max_ms);
    fprintf(fid,'configured_trigger=%s\npreflight_ready=%s\nruntime_gate_status=%s\nsource_pin_status=%s\nsource_pin_pass=%s\noffline_parity_pass=%s\n', ...
        mat2str(R.configured_trigger),mat2str(R.preflight_ready),R.runtime_gate_status, ...
        R.source_pin_status,mat2str(R.source_pin_pass),mat2str(R.offline_parity_pass));
    fprintf(fid,'native_mex_path=%s\nnative_mex_sha256=%s\nfirst_f_time_s=%.6f\nfirst_f_status=%s\nfirst_f_reason=%s\n', ...
        R.native_mex_path,R.native_mex_sha256,R.first_f_phase_time_s, ...
        R.first_f_phase_status,R.first_f_phase_reason);
    for i=1:numel(names)
        fprintf(fid,'status_count=%d %s\n',counts(i),names{i});
    end
    fprintf(fid,'interpretation=%s\n',R.interpretation);
end
fprintf('[TRIGGER MODE] predictions=%d | f phase=%d/%d | median=%.4f ms | %s\n', ...
    R.prediction_complete_count,R.f_phase_prediction_complete_count, ...
    R.f_phase_records,R.trigger_median_ms,R.interpretation);
end
function x=local_median(v)
if isempty(v),x=NaN;else,x=median(v);end
end
function x=local_max(v)
if isempty(v),x=NaN;else,x=max(v);end
end

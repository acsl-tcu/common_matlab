function W = TRAJECTORY_C9_HLC_LIMIT_WITNESS(trace, lower, upper)
%TRAJECTORY_C9_HLC_LIMIT_WITNESS  Exact sampled four-channel HLC limit witness.
% Inputs come ONLY from a recorded Sim LOGGER HLC audit and the ACTIVE HLC
% source-authority bounds. No assumed thrust/torque values, guessed margins,
% or in-flight computation. Contiguous sample runs are NOT continuous-time
% saturation intervals. This function is read-only and never commits a plan.
assert(isstruct(trace)&&isscalar(trace)&&all(isfield(trace, ...
    {'time_s','logger_index','unclamped_input4','clamped_input4', ...
     'sample_status','flight_records'})), ...
    'C9_LIMIT_WITNESS:Trace','A recorded HLC trace is required.');
assert(isnumeric(lower)&&isreal(lower)&&isequal(size(lower),[4 1])&& ...
    isnumeric(upper)&&isreal(upper)&&isequal(size(upper),[4 1])&& ...
    all(isfinite(lower))&&all(isfinite(upper))&&all(lower<upper), ...
    'C9_LIMIT_WITNESS:Limits', ...
    'Four authoritative HLC lower/upper values are required.');
lo=double(lower(:));hi=double(upper(:));
t=reshape(double(trace.time_s),1,[]);
idx=reshape(double(trace.logger_index),1,[]);
U=double(trace.unclamped_input4);
C=double(trace.clamped_input4);
N=numel(t);
assert(isequal(size(U),[4 N])&&isequal(size(C),[4 N])&& ...
    numel(idx)==N&&N==double(trace.flight_records)&& ...
    all(isfinite(t))&&all(diff(t)>0)&& ...
    all(isfinite(idx))&&all(diff(idx)>0)&& ...
    all(idx==fix(idx))&&numel(trace.sample_status)==N, ...
    'C9_LIMIT_WITNESS:Timeline','Invalid archived Flight sample alignment.');
% NaN/Inf in archived raw inputs invalidate the sampled bounds check, not
% a 'safe' zero violation count. Missing records remain explicitly unknown.
finite=all(isfinite(U)&isfinite(C),1);
lowviol=false(4,N);highviol=false(4,N);
for k=1:4
    lowviol(k,finite)=U(k,finite)<lo(k);
    highviol(k,finite)=U(k,finite)>hi(k);
end
viol=lowviol|highviol;
over=max(max(bsxfun(@minus,U,hi),bsxfun(@minus,lo,U)),0);
over(:,~finite)=NaN;
headroom=min([bsxfun(@minus,U,lo);bsxfun(@minus,hi,U)],[],1);
headroom(~finite)=NaN;
wrong_clamp=false(1,N);
if any(finite)
    expected=max(repmat(lo,1,N),min(repmat(hi,1,N),U));
    wrong_clamp(finite)=any(C(:,finite)~=expected(:,finite),1);
end
bad=find(any(viol,1));
% Consecutive LOGGER indices are a sampled episode, not a time continuum.
if isempty(bad)
    starts=[];ends=[];
else
    split=[true diff(idx(bad))~=1];
    start_at=find(split);
    end_at=[start_at(2:end)-1 numel(bad)];
    starts=bad(start_at);ends=bad(end_at);
end
events=repmat(struct('first_logger_index',NaN,'last_logger_index',NaN,...
    'first_time_s',NaN,'last_time_s',NaN,'sample_count',0, ...
    'violating_channels',false(4,1),'maximum_excess_by_channel',[0;0;0;0]), ...
    1,numel(starts));
for k=1:numel(starts)
    jj=starts(k):ends(k);
    events(k).first_logger_index=idx(starts(k));
    events(k).last_logger_index=idx(ends(k));
    events(k).first_time_s=t(starts(k));
    events(k).last_time_s=t(ends(k));
    events(k).sample_count=numel(jj);
    events(k).violating_channels=any(viol(:,jj),2);
    events(k).maximum_excess_by_channel=max(over(:,jj),[],2);
end
maxover=NaN(4,1);
minhead=NaN(4,1);
for k=1:4
    if any(finite)
        maxover(k)=max(over(k,finite));
        minhead(k)=min([U(k,finite)-lo(k) hi(k)-U(k,finite)]);
    end
end
W=struct('status','UNKNOWN_INCOMPLETE_RECORDED_HLC_WITNESS', ...
    'evaluated_sample_count',N,'nonfinite_sample_count',sum(~finite), ...
    'recorded_input_lower',lo,'recorded_input_upper',hi, ...
    'per_channel_violating_samples',sum(viol,2), ...
    'violating_any_input_samples',numel(bad), ...
    'maximum_excess_by_channel',maxover, ...
    'minimum_headroom_by_channel',minhead, ...
    'raw_input_min',NaN(4,1),'raw_input_max',NaN(4,1), ...
    'clamp_mismatch_samples',sum(wrong_clamp), ...
    'first_violation_time_s',NaN,'last_violation_time_s',NaN, ...
    'events',events,'event_count',numel(events), ...
    'all_samples_within_actual_HLC_limits',false, ...
    'reproduces_prior_audit_counters',false, ...
    'continuous_time_torque_certified',false,'safe_to_commit',false, ...
    'meaning','OBSERVED_DISCRETE_INPUT_WITNESS_NOT_CONTINUOUS_TIME');
if any(finite)
    W.raw_input_min=min(U(:,finite),[],2);
    W.raw_input_max=max(U(:,finite),[],2);
end
if ~isempty(bad)
    W.first_violation_time_s=t(bad(1));
    W.last_violation_time_s=t(bad(end));
end
status_count_ok=true;
if isfield(trace,'preclamp_saturation_count')
    status_count_ok=status_count_ok && ...
        isequal(double(trace.preclamp_saturation_count),double(numel(bad)));
end
if isfield(trace,'clamp_mismatch_count')
    status_count_ok=status_count_ok && ...
        isequal(double(trace.clamp_mismatch_count),double(sum(wrong_clamp)));
end
W.reproduces_prior_audit_counters=status_count_ok;
if N==0
    W.status='UNKNOWN_NO_FLIGHT_SAMPLES';
elseif ~all(finite)
    W.status='UNKNOWN_NONFINITE_HLC_INPUT_IN_RECORDED_FLIGHT';
elseif ~status_count_ok
    W.status='UNKNOWN_ARCHIVED_AUDIT_AND_RAW_INPUT_DISAGREE';
elseif any(wrong_clamp)
    W.status='REJECT_RECORDED_HLC_CLAMP_INCONSISTENT';
elseif ~isempty(bad)
    W.status='REJECT_RECORDED_HLC_INPUT_LIMIT_EXCEEDED';
else
    W.status='CHECKED_RECORDED_HLC_INPUT_LIMITS__NO_CONTINUOUS_PROOF';
    W.all_samples_within_actual_HLC_limits=true;
end
end

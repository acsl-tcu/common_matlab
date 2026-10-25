function R=AF7R58R105_STAGE_REPLAY(agent,eventTime,opts,previousA104)
% One read-only offline end-to-end assessment. Replay the exact stored
% EKF/known-obstacle event and HLC preview, then REUSE the R104 partial
% control points for nominal reference closure; never auto-commit.
assert(isstruct(opts)&&isscalar(opts), ...
    'R105:WorkOptionsRequired','An explicit work options struct is required.');
R=struct('status','NOT_STARTED','trigger',struct([]), ...
    'source_frame',struct([]),'exact_ekf_event_found',false, ...
    'source_event',struct([]),'physical_predictor',struct([]), ...
    'replan_requested',false,'generator',struct([]), ...
    'partial_closure',struct([]),'native_commit_called',false, ...
    'physical_closed_loop_certified',false, ...
    'production_commit_authorized',false);
% Reuse a caller-supplied R104 result without spending CPU generating
% the same 24 phase candidates again. Bind to the ACTUAL recorded double
% timestamp, not a rounded display literal like 19.75.
if nargin>=4 && ~isempty(previousA104)
    A=previousA104;
    assert(isstruct(A)&&isscalar(A)&&isfield(A,'snapshot_time_s')&& ...
        isfield(A,'reference_version'), ...
        'R105:PriorSearchSchema','Expected R104 result with exact source identity.');
    sourceTime=double(A.snapshot_time_s);
    assert(abs(double(eventTime)-sourceTime)<= ...
        64*eps(max([1,abs(double(eventTime)),abs(sourceTime)])), ...
        'R105:WrongEventSearchResult', ...
        'The requested event and R104 snapshot must be the same timestamp.');
else
    A=AF7R58R104_NATURAL_RETURN_SEARCH(agent,eventTime,opts);
    sourceTime=double(A.snapshot_time_s);
end
O=local_find(agent,5);
assert(~isempty(O),'R105:NativeReplannerRequired','Expected current live replanner.');
[F,E]=AF7R58R105_SOURCE_EVENT(O,sourceTime,double(A.reference_version));
% This replay is bound to an R104/R88 Flight snapshot, so require the
% same saved R88 geometry in addition to an actual estimator event.
if ~E.r88_exact_snapshot
    F=struct([]);
    E.status='NO_EXACT_R88_GEOMETRY_AUTHORITY_FOR_EVENT';
    E.ekf_frame_authenticated=false;
end
R.source_event=E;
R.source_frame=F;
R.exact_ekf_event_found=logical(E.ekf_frame_authenticated);
if isempty(F)
    R.status='NO_EXACT_EKF_EVENT__PHYSICAL_TRIGGER_BLOCKED';
elseif double(F.reference_version)~=double(A.reference_version)
    R.status='EVENT_REFERENCE_VERSION_MISMATCH__TRIGGER_BLOCKED';
else
    % This is an OFFLINE, historical snapshot. The existing R71 preview
    % method initializes from the CURRENT estimator and is not allowed to
    % stand in for an event-time state after Flight. Require a separate
    % actual snapshot-initialized HLC+plant rollout before a physical trigger.
    R.physical_predictor=struct( ...
        'status','HISTORICAL_EVENT_HLC_PLANT_FUTURE_NOT_CAPTURED', ...
        'invoked',false,'preview_structurally_valid',false, ...
        'closed_loop_interval_proved',false, ...
        'reason','Saved EKF endpoint is authentic but no exact HLC future is stored');
end
R.generator=A;
R.partial_closure=AF7R58R105_CLOSE_PARTIAL(agent,A,opts);
if R.replan_requested && R.partial_closure.reference_conditions_pass
    R.status='TRIGGER_AND_NOMINAL_C7_CONNECTED__EXECUTION_PROOF_OPEN';
elseif R.replan_requested
    R.status='PHYSICAL_REQUEST__NO_FULL_REFERENCE_PROOF';
elseif isempty(F)
    R.status='EXACT_EKF_EVENT_MISSING__REFERENCE_ANALYZED_ONLY';
elseif double(F.reference_version)~=double(A.reference_version)
    R.status='EVENT_REFERENCE_VERSION_MISMATCH__REFERENCE_ANALYZED_ONLY';
else
    R.status='EXACT_EKF_EVENT_BOUND__HLC_FUTURE_PENDING__REFERENCE_ANALYZED_ONLY';
end
end
function O=local_find(x,depth)
O=[];
if isa(x,'REPLANNING_SYSTEM')&&isscalar(x),O=x;return;end
if depth<1||isempty(x)||numel(x)~=1,return;end
if isobject(x)
    try,n=properties(x);catch,return;end
elseif isstruct(x)
    n=fieldnames(x);
else
    return;
end
skip={'self','result','parameter','plant','sensor','estimator', ...
    'controller','logger','time','state','parent'};
for j=1:numel(n)
    if any(strcmpi(n{j},skip)),continue;end
    try,O=local_find(x.(n{j}),depth-1);catch,O=[];end
    if ~isempty(O),return;end
end
end

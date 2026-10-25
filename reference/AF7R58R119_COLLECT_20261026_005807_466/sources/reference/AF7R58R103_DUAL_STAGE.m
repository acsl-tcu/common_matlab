function R=AF7R58R103_DUAL_STAGE(agent,eventTime,work,F,physicalPreview)
% SAME-EVENT connection: EKF+HLC future-trigger -> existing R102 trajectory
% generator -> existing REPLANNING_SYSTEM atomic commit authority.
% This version is explicitly a bounded replay/bridge, NOT to run inside a
% 25-ms Flight do(). No post-Flight inferred estimator mass is used.
R=struct('status','NOT_EVALUATED','physical_trigger',struct(), ...
    'generator',struct([]),'same_event_source_bound',false, ...
    'generator_invoked',false, ...
    'partial_reference_conditions_found',false, ...
    'reference_proved',false, ...
    'closed_loop_proved',false,'production_commit_performed',false, ...
    'ready_for_atomic_commit',false,'reason','');
assert(isstruct(work)&&isscalar(work)&&isnumeric(eventTime)&& ...
    isscalar(eventTime)&&isfinite(eventTime), ...
    'R103:ExplicitFlightAndWork','Explicit event time and work budget required.');
assert(isequal(double(F.time_s),double(eventTime)), ...
    'R103:EventTimeMismatch','Trigger and generator must use the same Flight instant.');
T=AF7R58R103_EKF_HLC_TRIGGER(F,physicalPreview);
R.physical_trigger=T;
if ~T.complete_preview
    R.status='BLOCKED_PHYSICAL_TRIGGER_NOT_READY';
    R.reason=T.status;return;
end
% Physical prediction is only a trigger signal; the native production
% certificate must still prove an execution-aware safety tube.
if ~T.request
    R.status='NO_NEW_PHYSICAL_COLLISION_REQUEST';return;
end
O=findReplanner(agent,5);
assert(~isempty(O),'R103:LiveReplannerNotFound', ...
    'Cannot bind actual REPLANNING_SYSTEM handle.');
assert(isprop(O,'r88_snapshots')&&~isempty(O.r88_snapshots), ...
    'R103:SameEventSnapshotMissing', ...
    'R102 generator requires a time/version-bound R88 Flight snapshot.');
idx=AF7R58R88_SELECT_TIME(O.r88_snapshots,eventTime);
if isempty(idx)
    R.status='BLOCKED_NO_EXACT_SAME_EVENT_SNAPSHOT';return;
end
S=O.r88_snapshots(idx);
assert(isequal(double(S.reference_version),T.reference_version)&& ...
    isequal(logical(S.known_mask(:).'),T.known_mask), ...
    'R103:SourceBindingMismatch', ...
    'Generator cannot use a different reference version or known-mask.');
R.same_event_source_bound=true;
A=AF7R58R102_ADAPTIVE_SAVED_SEARCH(agent,eventTime,work);
R.generator=A;R.generator_invoked=true;
if isfield(A,'partial_reference_found')
    R.partial_reference_conditions_found=logical(A.partial_reference_found);
end
% R102 certifies neither all future nor live HLC torque, robust tube,
% backup, IEEE outward arithmetic, deadline. It must not be wrapped in
% a synthetic 'certified' candidate or replace the production generator.
if isfield(A,'production_commit_authorized') && ...
        logical(A.production_commit_authorized)
    error('R103:UnsafeR102AuthorityEscalation', ...
        'Saved-reference generation attempted to promote itself to production.');
end
R.status='DUAL_CONNECTED_REFERENCE_ONLY__PRODUCTION_HANDOVER_BLOCKED';
R.reason=char(A.status);
end

function obj=findReplanner(x,depth)
obj=[];
if isa(x,'REPLANNING_SYSTEM')&&isscalar(x),obj=x;return;end
if depth<=0||isempty(x)||numel(x)~=1,return;end
if isobject(x)
    try,n=properties(x);catch,return;end
elseif isstruct(x)
    n=fieldnames(x);
else
    return;
end
skip={'self','result','parameter','plant','sensor','estimator', ...
    'controller','logger','time','state','parent'};
for k=1:numel(n)
    if any(strcmpi(n{k},skip)),continue;end
    try,obj=findReplanner(x.(n{k}),depth-1);catch,obj=[];end
    if ~isempty(obj),return;end
end
end

function U=AF7R58R106_STAGE_REPLAY(agent,A104,priorClosure,physicalPreview)
% Use an existing R104 candidate and an authenticated same-tick EKF frame.
% No offline generator work-options are required for event attachment.
% A preexisting R105 closure can be passed without any repeated solver work.
% Predictor remains a separate, genuine HLC-plant future.
% This function never calls the native production commit method.
if nargin<4,physicalPreview=[];end
assert(isstruct(A104)&&isscalar(A104)&& ...
    all(isfield(A104,{'snapshot_time_s','reference_version'})), ...
    'R106:PriorSearch','Pass your existing A104 result.');
if nargin<3,priorClosure=[];end
if ~isempty(priorClosure)
    assert(isstruct(priorClosure)&&isscalar(priorClosure)&& ...
        isfield(priorClosure,'reference_packet')&& ...
        isfield(priorClosure,'reference_conditions_pass'), ...
        'R106:PriorClosure','Expected an already evaluated G105 closure.');
    if ~isempty(priorClosure.reference_packet)
        pkt=priorClosure.reference_packet;
        assert(isfield(pkt,'reference_version')&& ...
            isfield(pkt,'snapshot_time_s')&& ...
            isequal(double(pkt.reference_version),double(A104.reference_version))&& ...
            isequal(double(pkt.snapshot_time_s),double(A104.snapshot_time_s)), ...
            'R106:PriorClosureIdentity','Stale G105 closure must not be reused.');
    end
end
O=local_find(agent,5);
assert(~isempty(O),'R106:ReplannerNotFound','Live replanner required.');
[F,D]=AF7R58R106_SOURCE_EVENT(O,double(A104.snapshot_time_s), ...
    double(A104.reference_version));
G=priorClosure; % No repeated CPU-intensive R94 certification in this bridge.
U=struct('status','R106_NO_AUTHENTIC_EVENT', ...
    'source_event',D,'exact_ekf_event_found',logical(D.authenticated), ...
    'source_frame',F,'physical_trigger',struct([]), ...
    'partial_closure',G,'physical_future_supplied',~isempty(physicalPreview), ...
    'full_closed_loop_certified',false, ...
    'production_commit_called',false,'production_commit_authorized',false);
if isempty(F)
    U.status='R106_EVENT_UNAVAILABLE__REFERENCE_RETAINED';return;
end
if isempty(physicalPreview)
    U.status='R106_EVENT_CONNECTED__HLC_4S_PREVIEW_REQUIRED';return;
end
% Never accept surrogate/reference-only preview. Uses unchanged R103 gate.
U.physical_trigger=AF7R58R103_EKF_HLC_TRIGGER(F,physicalPreview);
if logical(U.physical_trigger.complete_preview)
    U.status='R106_NUMERIC_EKF_HLC_TRIGGER_CONNECTED__COMMIT_NOT_CERTIFIED';
else
    U.status='R106_PHYSICAL_PREVIEW_REJECTED__REFERENCE_RETAINED';
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

function R=AF7R58R104_DUAL_EVENT_REPLAY(agent,eventTime,opts)
% Genuine stored SAME-EVENT R55 -> existing HLC preview -> R103 4s trigger
% -> new phase-decoupled R102 generator. Offline diagnostic INTEGRATION.
% No live Flight do(), no controller replacement, no certificate bypass.
R=struct('status','NOT_STARTED','source_event',struct(), ...
    'predictor',struct(),'physical_trigger',struct(), ...
    'generator',struct([]),'event_frame_bound',false, ...
    'physical_prediction_bound',false,'physical_request',false, ...
    'generator_invoked',false,'production_commit_authorized',false, ...
    'physical_closed_loop_proved',false,'reason','');
assert(isstruct(opts)&&isscalar(opts), ...
    'R104:ExplicitSearchBudget','A numeric search budget struct is required.');
O=local_find(agent,5);
assert(~isempty(O),'R104:NativeInstanceMissing', ...
    'The native REPLANNING_SYSTEM was not found.');
[F,E]=AF7R58R104_LIVE_EVENT_PACKET(O,eventTime);
R.source_event=E;
if isempty(F)
    R.status='BLOCKED_NO_EXACT_SENSOR_EKF_EVENT';R.reason=E.status;return;
end
R.event_frame_bound=true;
[P,D]=AF7R58R104_PHYSICAL_PREDICTOR(O,F);
R.predictor=D;
if ~D.preview_structurally_valid
    R.status='BLOCKED_PHYSICAL_HLC_PREDICTOR_NOT_READY';
    R.reason=D.status;return;
end
R.physical_prediction_bound=true;
T=AF7R58R103_EKF_HLC_TRIGGER(F,P);
R.physical_trigger=T;
if ~T.complete_preview
    R.status='BLOCKED_PHYSICAL_TRIGGER_INCONCLUSIVE';
    R.reason=T.status;return;
end
R.physical_request=T.request;
if ~T.request
    R.status='NO_HLC_PREVIEW_COLLISION_REQUEST';return;
end
idx=AF7R58R88_SELECT_TIME(O.r88_snapshots,eventTime);
if isempty(idx)||O.r88_snapshots(idx).reference_version~=F.reference_version|| ...
        ~isequal(logical(O.r88_snapshots(idx).known_mask(:).'),T.known_mask)
    R.status='BLOCKED_SAME_EVENT_R88_VERSION_MASK_MISMATCH';return;
end
A=AF7R58R104_NATURAL_RETURN_SEARCH(agent,eventTime,opts);
R.generator=A;R.generator_invoked=true;
R.status='PHYSICAL_REQUEST_TO_C7_GENERATOR_CONNECTED__NO_COMMIT';
R.reason=A.status;
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

function R=AF7R58R107_SAME_TICK_TRIGGER(replanner,t,knownMask,physicalFuture)
% Production-oriented single-event trigger adapter, no R88 snapshots,
% no offline R104 solver, no logging, no workers.
% A reference-only trajectory is NEVER treated as an HLC physical rollout.
if nargin<4,physicalFuture=[];end
[F,D]=AF7R58R107_LIVE_EKF_FRAME(replanner,t,knownMask);
R=struct('status',D.status,'event',D,'trigger',struct([]), ...
    'replan_request',false,'can_replace_legacy_trigger',false, ...
    'generator_request_allowed',false,'commit_authorized',false);
if isempty(F),return;end
if isempty(physicalFuture)
    R.status='PHYSICAL_4S_PREVIEW_NOT_AVAILABLE__KEEP_LEGACY_TRIGGER';
    return;
end
T=AF7R58R103_EKF_HLC_TRIGGER(F,physicalFuture);
R.trigger=T;R.status=T.status;
R.replan_request=logical(T.request);
R.can_replace_legacy_trigger=logical(T.complete_preview);
R.generator_request_allowed=R.can_replace_legacy_trigger&&R.replan_request;
% Numerical physical preview does NOT close reachability/tube proof and
% therefore NEVER authorizes adoption of a candidate by itself.
end

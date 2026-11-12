function result=wp_preview_4s(world,cfg,referenceSource,tNow,opts)
% WP_PREVIEW_4S  Sampled 4-s lookahead for C/B/D provisional systems.
%
%  result=wp_preview_4s(world,cfg,refFcn,tNow,opts)
%  refFcn(t): full physical (global) time -> WAYPOINT 28x1 (RAW before sload).
%  or source is the active WAYPOINT_REFERENCE object (read-only preview).
%
%  opts: horizon (4), sampleDt (.10), predictionMode ('flatness'/'vertical'),
%        gravity (9.81), flightStartTime (5), freefallThreshold (.15).
%        currentPose=struct('pDrone',3x1,'pLoad',3x1) [optional],
%        maxTrackingDeviation=0.30 [m] => warns if actual differs nominal.
%
% A reported 'sampled_clear' does NOT prove continuous-time safety.
% No vehicle state dynamics or feedback are propagated here. Future governor
% feedback is held constant. Dynamic obstacle trajectory IS updated by t.
if nargin<5,opts=struct();end
if ~isfield(opts,'horizon'),opts.horizon=4.0;end
if ~isfield(opts,'sampleDt'),opts.sampleDt=0.10;end
if ~isfield(opts,'predictionMode'),opts.predictionMode='flatness';end
if ~isfield(opts,'gravity'),opts.gravity=9.81;end
if ~isfield(opts,'flightStartTime'),opts.flightStartTime=5.0;end
if ~isfield(opts,'freefallThreshold'),opts.freefallThreshold=0.15;end
if ~isfield(opts,'maxTrackingDeviation'),opts.maxTrackingDeviation=0.30;end
assert(isscalar(tNow) && isfinite(tNow));
assert(isscalar(opts.horizon) && isfinite(opts.horizon) && opts.horizon>0);
assert(isscalar(opts.sampleDt) && isfinite(opts.sampleDt) && opts.sampleDt>0);
steps=ceil(opts.horizon/opts.sampleDt);
assert(steps<=10001,'Too many preview samples');
times=linspace(tNow,tNow+opts.horizon,steps+1);
tTotal=tic;
[X,refInfo]=wp_preview_reference_series(referenceSource,times,opts);
N=numel(times);
clearances=NaN(1,N);hit=false(1,N);valid=false(1,N);
pDrone=nan(3,N);pLoad=nan(3,N);
hitObstacle=cell(1,N);hitSystemPart=cell(1,N);minOb=nan(1,N);minGround=nan(1,N);
checkTime=zeros(1,N);
reason='';
for k=1:N
    [pD,pL,poseInfo]=wp_pose_from_reference(X(:,k),cfg,opts);
    if ~poseInfo.valid
        reason=sprintf('Invalid nominal pose at t=%.4f: %s',times(k),poseInfo.reason);
        break;
    end
    try
        shape=wp_system_shape_at_pose(cfg,pD,pL);
        tk=tic;
        report=wp_system_collision(cfg,shape,world,times(k));
        checkTime(k)=toc(tk);
    catch ex
        reason=sprintf('Collision evaluation invalid at t=%.4f: %s',times(k),ex.message);
        break;
    end
    valid(k)=true;pDrone(:,k)=pD;pLoad(:,k)=pL;
    clearances(k)=report.minClearance;hit(k)=report.collision;
    minOb(k)=report.minObstacleClearance;
    minGround(k)=report.minGroundClearance;
    hitObstacle{k}=report.worstObstacle;hitSystemPart{k}=report.worstSystemPart;
end
% At the start of a forecast, check measured/estimated geometry separately.
% This DOES NOT imply that the future tracking error stays bounded.
trackingDeviation=NaN; currentPoseChecked=false;
if isfield(opts,'currentPose') && ~isempty(opts.currentPose)
    cp=opts.currentPose;
    try
        sActual=wp_system_shape_at_pose(cfg,cp.pDrone,cp.pLoad);
        rActual=wp_system_collision(cfg,sActual,world,tNow);
        trackingDeviation=max(norm(cp.pDrone(:)-pDrone(:,1)), ...
                              norm(cp.pLoad(:)-pLoad(:,1)));
        currentPoseChecked=true;
        if rActual.collision
            hit(1)=true;clearances(1)=min(clearances(1),rActual.minClearance);
            hitObstacle{1}=rActual.worstObstacle;
            hitSystemPart{1}=rActual.worstSystemPart;
        end
    catch ex
        reason=sprintf('Current pose check invalid: %s',ex.message);
        valid(1)=false;
    end
end
idx=find(hit & valid,1,'first');
if isempty(idx),firstHitTime=NaN;firstHitObstacle='';firstHitPart='';
else,firstHitTime=times(idx);firstHitObstacle=hitObstacle{idx};firstHitPart=hitSystemPart{idx};end
if any(~valid)
    status='prediction_invalid';
elseif ~isempty(idx)
    status='sampled_collision';
elseif currentPoseChecked && trackingDeviation>opts.maxTrackingDeviation
    status='tracking_deviation_unbounded';
else
    status='sampled_clear_not_guaranteed';
end
result=struct('method',cfg.method,'predictionMode',char(opts.predictionMode), ...
    'status',status,'collisionDetected',~isempty(idx), ...
    'firstHitTime',firstHitTime,'firstHitObstacle',firstHitObstacle, ...
    'firstHitPart',firstHitPart,'times',times,'minClearance',clearances, ...
    'minObstacleClearance',minOb,'minGroundClearance',minGround, ...
    'collisionSamples',hit,'validSamples',valid,'pDrone',pDrone, ...
    'pLoad',pLoad,'perSampleCollisionTime',checkTime, ...
    'totalElapsedSeconds',toc(tTotal),'referenceInfo',refInfo, ...
    'invalidReason',reason,'sampledOnly',true,'cbfSafetyGuarantee',false, ...
    'currentPoseChecked',currentPoseChecked,'trackingDeviation',trackingDeviation, ...
    'systemGeometryProvisional',true, ...
    'marginMetric', 'D ellipsoid uses sign-correct proxy, not Euclidean');
end

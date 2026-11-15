function out = TEST_SIX_TRIGGER_REJOIN_TRACE( ...
    trueFlightTrace,forecast,observedWorld,mode,cover,model)
%TEST_SIX_TRIGGER_REJOIN_TRACE Optional actual-HLC Flight trace audit.
%
% Offline interface for REAL nonlinear/HLC data (not a synthetic flight):
%   trueFlightTrace.t         1xM, from 0 through 4 seconds
%   trueFlightTrace.pDrone    3xM, plant or independently checked truth
%   trueFlightTrace.pLoad     3xM
%   forecast                 independent 4-second predictor WITH bounds
%   observedWorld            original OBSERVED outer-cover world
%   mode/cover               ONE of the six choices (run six times if desired)
%   model                    wp_six_trigger_model_snapshot(current agent)
%
% Report pointwise position error bound violations and dense collision
% witnesses. Sampled tests NEVER establish an all-time true error bound.
if nargin==0
    out=struct('status','SKIPPED_REQUIRES_REAL_HLC_FLIGHT_TRACE', ...
       'actualHlcForecastTested',false);
    fprintf('SKIP real HLC trace audit: supply actual Flight trace and independent forecast.\n');
    return
end
external=wp_system_geometry_config(mode);
cfg=wp_six_trigger_config(mode,cover,external,model);
known=wp_six_trigger_prepare(observedWorld,cover);
result=wp_six_trigger_evaluate(known,cfg,forecast);

T=trueFlightTrace.t(:)';
D=trueFlightTrace.pDrone;L=trueFlightTrace.pLoad;
assert(numel(T)>=2 && abs(T(1))<=1e-12 && abs(T(end)-4)<=1e-12);
assert(all(diff(T)>0) && isequal(size(D),[3,numel(T)]) && ...
       isequal(size(L),[3,numel(T)]));
assert(all(isfinite([D(:);L(:)])));

% Pointwise errors against the provided forecast center. These do not
% certify bounds BETWEEN samples or disturbances of unseen future motion.
tau=forecast.timeOffsets(:)';
trueD=interp1(T,D',tau,'linear')';
trueL=interp1(T,L',tau,'linear')';
eD=sqrt(sum((trueD-forecast.pDrone).^2,1));
eL=sqrt(sum((trueL-forecast.pLoad).^2,1));
eDOverflow=max(eD-reshape(forecast.rhoDrone,1,[]));
eLOverflow=max(eL-reshape(forecast.rhoLoad,1,[]));

% Original uploaded geometry oracle, at every available true Flight sample.
witness=false;witnessT=NaN;
for j=1:numel(T)
    shape=wp_system_shape_at_pose(external,D(:,j),L(:,j));
    contact=wp_system_collision(external,shape,observedWorld,T(j));
    if contact.collision
        witness=true;witnessT=T(j);break
    end
end
falseClear=witness && strcmp(result.status,'CLEAR_CONDITIONAL');
if falseClear
    status='FAIL_ACTUAL_HLC_COLLISION_WITNESS_FALSE_CLEAR';
elseif eDOverflow>1e-8 || eLOverflow>1e-8
    status='FAIL_POINTWISE_MODEL_BOUND';
else
    status='PASS_SAMPLED_REAL_HLC_AUDIT_NOT_FULL_PROOF';
end
out=struct('status',status,'selectedMode',char(mode), ...
    'selectedCover',char(cover),'witnessContact',witness, ...
    'firstWitnessTime',witnessT,'falseClear',falseClear, ...
    'maxDronePositionBoundExcess',eDOverflow, ...
    'maxLoadPositionBoundExcess',eLOverflow, ...
    'triggerResult',result,'sampledOnly',true, ...
    'continuousPhysicalProof',false);
if falseClear || strcmp(status,'FAIL_POINTWISE_MODEL_BOUND')
    error('LightTrigger:ActualHlcAudit', ...
        'Actual nonlinear Flight trace contradicts the predicted conditional bound');
end
end

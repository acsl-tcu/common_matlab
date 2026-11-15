function R=COMPARE_TRIGGER_V3_HLC_TO_FLIGHT_LOG(nominal,flight)
%COMPARE_TRIGGER_V3_HLC_TO_FLIGHT_LOG Compare nominal rollout to actual Flight.
% The actual Flight trace is used ONLY as evaluation ground truth, never
% as forecast input nor to override the EKF-estimated payload mass.
%
% nominal: output of wp_six_trigger_predict_hlc_4s(agent,model)
% flight: struct with fields timeOffsets (1xN), pDrone (3xN), pLoad (3xN)
%         of the actual future *plant* simulation, same initial time origin.
% Does not assume that this retrospective error is a universal upper bound.
needed={'timeOffsets','pDrone','pLoad'};
for k=1:numel(needed)
    assert(isfield(flight,needed{k}),'Missing actual Flight field %s',needed{k});
end
assert(isfield(nominal,'fullState') && isfield(nominal,'fullTimes'));
t=reshape(flight.timeOffsets,1,[]);
assert(numel(t)>=2 && all(diff(t)>0) && t(1)>=0 && ...
    t(end)<=nominal.fullTimes(end)+1e-10);
assert(isequal(size(flight.pDrone),[3,numel(t)]) && ...
    isequal(size(flight.pLoad),[3,numel(t)]));
pD=interp1(nominal.fullTimes,nominal.fullState(1:3,:)',t,'linear')';
pL=interp1(nominal.fullTimes,nominal.fullState(13:15,:)',t,'linear')';
eD=sqrt(sum((flight.pDrone-pD).^2,1));
eL=sqrt(sum((flight.pLoad-pL).^2,1));
R=struct('status','COMPARISON_ONLY_NOT_A_SAFETY_BOUND', ...
    'times',t,'droneError',eD,'loadError',eL, ...
    'maxDroneError',max(eD),'maxLoadError',max(eL), ...
    'rmseDrone',sqrt(mean(eD.^2)), ...
    'rmseLoad',sqrt(mean(eL.^2)), ...
    'predictionMassFromCurrentEKF',nominal.estimatedLoadMassUsed, ...
    'plantTrueMassSuppliedToPredictor',false, ...
    'deterministicFutureErrorCertified',false);
end

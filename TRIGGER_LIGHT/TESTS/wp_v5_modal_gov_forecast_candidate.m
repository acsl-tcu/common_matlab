function F=wp_v5_modal_gov_forecast_candidate(H,gains,routeCache,varargin)
%WP_V5_MODAL_GOV_FORECAST_CANDIDATE Offline experimental fast 4s forecast.
%
% Inputs contain ONLY CURRENT source-model/estimator/governor values,
% past observed load+reference values, original HLC gains, and a planned
% route table cached outside the tick. They contain NO future HLC or plant.
% Recomputes six-stage virtual-time governor command each 25ms based on
% predicted TRACKING ERROR, with exact original ZOH state transition.
% Modal output error is derived from ORIGINAL dynamic feedback linearization,
% conditional on taut cable, nonsingular geometry, and unsaturated HLC.
% Tension/physical prediction error are NOT rigorously enclosed.
%
% Do NOT attach finite rho/acc envelopes from this unqualified model;
% downstream geometry must not return physically-certified clear.
required={'pastTimes','pastLoad','pastRefLoad','tauNow','rateNow', ...
    'heldCommandNow','estimatedML','cableLength','gravity', ...
    'slowdownStart','slowdownFull','minScale','filterTime'};
for i=1:numel(required)
    assert(isfield(H,required{i}),['Missing source input: ' required{i}]);
end
assert(isequal(size(H.pastLoad),[3,numel(H.pastTimes)]) && ...
    isequal(size(H.pastRefLoad),[3,numel(H.pastTimes)]));
assert(isequal(size(H.rateNow),[6,1]) && all(isfinite(H.rateNow)) && ...
    isfinite(H.estimatedML) && H.estimatedML>0 && ...
    H.cableLength>0 && H.gravity>0 && ...
    numel(H.pastTimes)>6 && abs(H.pastTimes(end))<1e-10);
assert(H.slowdownFull>H.slowdownStart && H.minScale>=0 && H.minScale<=1);
h=.025;N=round(4/h);
if nargin>=4 && ~isempty(varargin{1})
    K=varargin{1};
    assert(isequal(size(K.pastRelativeTimes),size(H.pastTimes)) && ...
        max(abs(K.pastRelativeTimes-H.pastTimes))<1e-9 && ...
        isequal(K.gainsF1,gains.F1) && ...
        isequal(K.gainsF2,gains.F2) && ...
        isequal(K.gainsF3,gains.F3) && ...
        abs(K.governor.filterTime-H.filterTime)<32*eps(max(1,H.filterTime)), ...
        'V5 precomputed gain/history cache is stale');
else
    K=wp_v5_modal_cache_prepare(gains,H.pastTimes,H.filterTime);
end
Cgov=K.governor;
start=tic;
profileWatch=tic;
errHist=H.pastLoad-H.pastRefLoad;
states=cell(1,3);
for j=1:3
    states{j}=K.HP{j}\errHist(j,:)';
    if any(~isfinite(states{j})), error('V6:ModalFit','Nonfinite fitted modal state');end
end
fitMs=1000*toc(profileWatch);
Phi=K.Phi;A2=K.A2;
E=zeros(3,N+1); E2=E;
tauHist=zeros(1,N+1);dr=zeros(7,N+1);cmdHist=zeros(1,N+1);
z=H.rateNow(:);tau=H.tauNow;cmd=H.heldCommandNow;
tauHist(1)=tau;dr(:,1)=[tau;Cgov.D*z];cmdHist(1)=cmd;
for j=1:3
    E(j,1)=states{j}(1);E2(j,1)=A2{j}(1,:)*states{j};
end
profileWatch=tic;
for k=1:N
    for j=1:3
        states{j}=Phi{j}*states{j};
        E(j,k+1)=states{j}(1);
        E2(j,k+1)=A2{j}(1,:)*states{j};
    end
    [z,tau,tauDer]=wp_six_trigger_governor_fast_step(z,tau,cmd,Cgov);
    tauHist(k+1)=tau;dr(:,k+1)=tauDer;
    ee=norm(E(:,k+1));
    if ee<=H.slowdownStart
        u=0;
    elseif ee>=H.slowdownFull
        u=1;
    else
        u=(ee-H.slowdownStart)/(H.slowdownFull-H.slowdownStart);
    end
    u=u*u*(3-2*u);
    cmd=1-(1-H.minScale)*u;
    cmdHist(k+1)=cmd;
end
modeGovMs=1000*toc(profileWatch);
if tauHist(1)<routeCache.tauGrid(1) || ...
   tauHist(end)>routeCache.tauGrid(end) || any(diff(tauHist)<0)
    error('V5:RouteTable','Predicted virtual time outside cached route');
end
% One batched interpolation each for position, 1st/2nd virtual-time jets.
profileWatch=tic;
ref=interp1(routeCache.tauGrid',routeCache.p',tauHist','pchip')';
dref=interp1(routeCache.tauGrid',routeCache.dp',tauHist','pchip')';
d2ref=interp1(routeCache.tauGrid',routeCache.ddp',tauHist','pchip')';
routeInterpMs=1000*toc(profileWatch);
profileWatch=tic;
ref(3,:)=ref(3,:)-H.cableLength;
refAcc=dref.*repmat(dr(3,:),3,1) + ...
       d2ref.*repmat(dr(2,:).^2,3,1);
pLoad=ref+E;
aLoad=refAcc+E2;
gvec=repmat([0;0;H.gravity],1,N+1);
rest=aLoad+gvec;
forceNorm=vecnorm(rest,2,1);
if any(~isfinite(forceNorm)) || ...
    min(forceNorm)<=sqrt(eps)*H.gravity
    error('V5:TautReconstruction','Cannot reconstruct positive-tension cable');
end
q=-rest./repmat(forceNorm,3,1);
pDrone=pLoad-H.cableLength*q;
reconstructMs=1000*toc(profileWatch);
F=struct('status','NOMINAL_MODAL_GOVERNOR_CANDIDATE_NOT_CERTIFIED', ...
    'timeOffsets',(0:N)*h,'pLoad',pLoad,'pDrone',pDrone, ...
    'predCableDirection',q,'predTrackingError',E, ...
    'virtualTime',tauHist, ...
    'governorHeldCommand',cmdHist,'governorTauDer',dr, ...
    'modelEstimatedMassUsed',H.estimatedML, ...
    'predictedModelTension_N',H.estimatedML*forceNorm, ...
    'rhoDrone',nan(1,N+1),'rhoLoad',nan(1,N+1), ...
    'accDrone',nan(1,N),'accLoad',nan(1,N), ...
    'notCertified',true,'isEligibleForSafeClear',false, ...
    'futureReferenceOracleUsed',false, ...
    'futurePlantUsed',false,'futureHlcUsed',false, ...
    'futureReferenceFromCachedOriginalRoute',true, ...
    'inputSaturationRuledOut',false,'requiresPastEstimatedOutputHistory',true, ...
    'timeMs',1000*toc(start), ...
    'stageMs',struct('modalFit',fitMs,'modalAndGovernor160',modeGovMs, ...
      'routeInterpolation',routeInterpMs,'tautReconstruction',reconstructMs), ...
    'cacheBuiltInTick',nargin<4 || isempty(varargin{1}));
end

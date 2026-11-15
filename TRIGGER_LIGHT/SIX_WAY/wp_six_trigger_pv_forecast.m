function F=wp_six_trigger_pv_forecast(H,K,routeCache)
%WP_SIX_TRIGGER_PV_FORECAST Causal PV-history 4s candidate; ONE 25ms tick.
%
% Inputs contain ONLY CURRENT source-model/estimator/governor values,
% past EKF load pL/vL and known reference values, original HLC gains, and a planned
% route table cached outside the tick. They contain NO future HLC or plant.
% Recomputes six-stage virtual-time governor command each 25ms based on
% predicted TRACKING ERROR, with exact original ZOH state transition.
% Position and velocity initial modes are estimated by a precomputed fit.
% Modal output error is derived from ORIGINAL dynamic feedback linearization,
% conditional on taut cable, nonsingular geometry, and unsaturated HLC.
% Tension/physical prediction error are NOT rigorously enclosed.
%
% Do NOT attach finite rho/acc envelopes from this unqualified model;
% downstream geometry must not return physically-certified clear.
required={'pastTimes','pastLoad','pastVelocity','pastRefLoad', ...
    'pastRefVelocity','tauNow','rateNow','heldCommandNow', ...
    'estimatedML','cableLength','gravity','slowdownStart', ...
    'slowdownFull','minScale','filterTime'};
for i=1:numel(required)
    if ~isfield(H,required{i})
        error('LightTrigger:PVInput','Missing current/past data: %s',required{i});
    end
end
m=numel(K.pastRelativeTimes);
if ~isequal(size(H.pastLoad),[3,m]) || ...
   ~isequal(size(H.pastVelocity),[3,m]) || ...
   ~isequal(size(H.pastRefLoad),[3,m]) || ...
   ~isequal(size(H.pastRefVelocity),[3,m]) || ...
   ~isequal(size(H.rateNow),[6,1]) || ...
   numel(H.pastTimes)~=m || ...
   max(abs(reshape(H.pastTimes,1,[])-K.pastRelativeTimes))>1e-9
    error('LightTrigger:PVInput','History or governor state shape is inconsistent');
end
if any(~isfinite([H.pastLoad(:);H.pastVelocity(:); ...
        H.pastRefLoad(:);H.pastRefVelocity(:);H.rateNow(:)])) || ...
        ~isfinite(H.estimatedML) || H.estimatedML<=0 || ...
        ~isfinite(H.cableLength) || H.cableLength<=0 || ...
        ~isfinite(H.gravity) || H.gravity<=0 || ...
        H.slowdownFull<=H.slowdownStart || H.minScale<0 || ...
        H.minScale>1 || ...
        abs(K.governor.filterTime-H.filterTime)>1e-12
    error('LightTrigger:PVInput','Invalid physical/governor data');
end
h=.025;N=160;
Cgov=K.governor;
start=tic;
profileWatch=tic;
errHist=H.pastLoad-H.pastRefLoad;
errVel=H.pastVelocity-H.pastRefVelocity;
states=cell(1,3);
for j=1:3
    states{j}=K.W{j}*[errHist(j,:)'; K.velocityWeight*errVel(j,:)'];
    if any(~isfinite(states{j})), error('LightTrigger:PVFit','Nonfinite estimated mode');end
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
F=struct('status','NOMINAL_PV_MODAL_GOVERNOR_NOT_CERTIFIED', ...
    'timeOffsets',(0:N)*h,'pLoad',pLoad,'pDrone',pDrone, ...
    'predCableDirection',q,'predTrackingError',E, ...
    'virtualTime',tauHist, ...
    'governorHeldCommand',cmdHist,'governorTauDer',dr, ...
    'modelEstimatedMassUsed',H.estimatedML,'modelLoadMass',H.estimatedML, ...
    'predictedModelTension_N',H.estimatedML*forceNorm, ...
    'rhoDrone',nan(1,N+1),'rhoLoad',nan(1,N+1), ...
    'accDrone',nan(1,N),'accLoad',nan(1,N), ...
    'notCertified',true,'isEligibleForSafeClear',false, ...
    'futureReferenceOracleUsed',false, ...
    'futurePlantUsed',false,'futureHlcUsed',false, ...
    'futureReferenceFromCachedOriginalRoute',true, ...
    'inputSaturationRuledOut',false,'requiresPastEstimatedOutputHistory',true, ...
    'requiresPastEstimatedVelocityHistory',true, ...
    'usesCurrentEKFPositionAndVelocity',true, ...
    'timeMs',1000*toc(start), ...
    'stageMs',struct('modalFit',fitMs,'modalAndGovernor160',modeGovMs, ...
      'routeInterpolation',routeInterpMs,'tautReconstruction',reconstructMs), ...
    'cacheBuiltInTick',false,'fitCondition',K.condition);
end

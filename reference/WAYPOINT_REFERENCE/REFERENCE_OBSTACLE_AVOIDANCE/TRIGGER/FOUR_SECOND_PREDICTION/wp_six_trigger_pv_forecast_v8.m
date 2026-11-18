function F=wp_six_trigger_pv_forecast_v8(H,K,routeCache)
%WP_SIX_TRIGGER_PV_FORECAST Causal PV-history 4s candidate; ONE 25ms tick.
%
% Inputs contain ONLY CURRENT source-model/estimator/governor values,
% past EKF load pL/vL and known reference values, original HLC gains, and a planned
% route table cached outside the tick. They contain NO future HLC or plant.
% Recomputes six-stage virtual-time governor command each 25ms based on
% predicted TRACKING ERROR, with exact original ZOH state transition.
% Current EKF load p/v are EXACT initial conditions. Only the missing
% high-order error modes are fitted to past EKF/reference samples.
% When provided, current EKF cable direction also constrains x/y load
% acceleration at t=0, consistent with the taut-cable reconstruction.
% No post-hoc position snapping or future truth is used.
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
if ~isfield(K,'WfreePV') || ~isfield(K,'WfreePVA')
    error('LightTrigger:PVCache','Rebuild the initial-value-constrained PV cache off-tick');
end
errHist=H.pastLoad-H.pastRefLoad;
errVel=H.pastVelocity-H.pastRefVelocity;
% The predicted route is evaluated from the prepared cache, whereas the
% historical reference is the ORIGINAL HLC reference. They can differ
% slightly even at current tau (PCHIP interpolation). Anchor the state
% against the SAME cached reference that will be used in the forecast,
% otherwise even an exact error fit would not give exact current EKF p/v.
if H.tauNow<routeCache.tauGrid(1) || H.tauNow>routeCache.tauGrid(end)
    error('LightTrigger:RouteTable','Current tau outside prepared reference');
end
if isfield(routeCache,'pchipCoefficientsPreparedOffTick') && ...
        routeCache.pchipCoefficientsPreparedOffTick
    pref0=ppval(routeCache.ppP,H.tauNow);
    dp0=ppval(routeCache.ppDP,H.tauNow);
    ddp0=ppval(routeCache.ppDDP,H.tauNow);
else
    pref0=interp1(routeCache.tauGrid',routeCache.p',H.tauNow,'pchip')';
    dp0=interp1(routeCache.tauGrid',routeCache.dp',H.tauNow,'pchip')';
    ddp0=interp1(routeCache.tauGrid',routeCache.ddp',H.tauNow,'pchip')';
end
pref0=pref0(:);dp0=dp0(:);ddp0=ddp0(:);
pref0(3)=pref0(3)-H.cableLength;
gDer=K.governor.D*H.rateNow(:);  % tau' and tau'' at t=0
vref0=dp0*gDer(1);
accRef0=dp0*gDer(2)+ddp0*gDer(1)^2;
fixedEKFErrorP=H.pastLoad(:,end)-pref0;
fixedEKFErrorV=H.pastVelocity(:,end)-vref0;
states=cell(1,3);
yObs=cell(1,3);
fitRMS=zeros(1,3);
for j=1:3
    yObs{j}=[errHist(j,:)';K.velocityWeight*errVel(j,:)'];
    fixed=[fixedEKFErrorP(j);fixedEKFErrorV(j)]; % exact EKF output
    unknown=K.WfreePV{j}*(yObs{j}-K.BfixedPV{j}*fixed);
    states{j}=[fixed;unknown];
    if any(~isfinite(states{j})),error('LightTrigger:PVFit','Nonfinite estimated mode');end
end
% The taut model gives q=-(aLoad+g)/||aLoad+g||. With the vertical
% 2nd-order mode fixed by EKF p/v, the z-acceleration is no longer free.
% If EKF q is available and feasible, constrain horizontal e_ddot(0)
% and fit ONLY the remaining 3 higher modes in each 6th-order axis.
% This maintains pLoad(0), vLoad(0), and (conditionally) q(0) without a jump.
initial=struct('exactEKFLoadPV',true,'cableAnchorRequested',false, ...
    'cableAnchorApplied',false,'cableAnchorStatus','NOT_PROVIDED', ...
    'EKFSourceGeometryMismatch_m',NaN,'EKFDroneMismatch_m',NaN, ...
    'EKFCableAngleMismatch_deg',NaN,'EKFLoadMismatch_m',NaN, ...
    'EKFLoadVelocityMismatch_mps',NaN,'modelFitWeightedRMS',nan(1,3), ...
    'estimatedInitialTension_N',NaN, ...
    'routeCacheCurrentReferenceMismatch_m',norm(pref0-H.pastRefLoad(:,end)), ...
    'routeCacheCurrentVelocityMismatch_mps',norm(vref0-H.pastRefVelocity(:,end)));
qEKF=[];
if isfield(H,'currentEKFCableDirection')
    initial.cableAnchorRequested=true;
    qEKF=H.currentEKFCableDirection(:);
    if numel(qEKF)~=3 || any(~isfinite(qEKF)) || norm(qEKF)<sqrt(eps)
        initial.cableAnchorStatus='INVALID_EKF_CABLE_DIRECTION';
    else
        qEKF=qEKF/norm(qEKF);
        restZ=accRef0(3)+K.A2{3}(1,:)*states{3}+H.gravity;
        % A taut cable requires rest=-lambda*q with lambda>0.
        % q_z near zero makes horizontal acceleration determination unstable.
        if abs(qEKF(3))<=1e-3
            initial.cableAnchorStatus='CABLE_Z_COMPONENT_SINGULAR';
        else
            forcePerMass=-restZ/qEKF(3);
            if ~isfinite(forcePerMass) || forcePerMass<=sqrt(eps)*H.gravity
                initial.cableAnchorStatus='NO_POSITIVE_TAUT_TENSION_CONSISTENCY';
            else
                for j=1:2
                    accNeeded=-forcePerMass*qEKF(j);
                    fixed=[states{j}(1:2);accNeeded-accRef0(j)];
                    free=K.WfreePVA{j}*(yObs{j}-K.BfixedPVA{j}*fixed);
                    states{j}=[fixed;free];
                end
                initial.cableAnchorApplied=true;
                initial.cableAnchorStatus='EXACT_EKF_CABLE_DIRECTION_TAUT_CONSISTENT';
                initial.estimatedInitialTension_N=H.estimatedML*forcePerMass;
            end
        end
    end
end
for j=1:3
    fitRMS(j)=norm(K.Bfull{j}*states{j}-yObs{j})/sqrt(numel(yObs{j}));
end
initial.modelFitWeightedRMS=fitRMS;
fitMs=1000*toc(profileWatch);
Phi=K.Phi;A2=K.A2;
E=zeros(3,N+1); E1=E; E2=E;
tauHist=zeros(1,N+1);dr=zeros(7,N+1);cmdHist=zeros(1,N+1);
z=H.rateNow(:);tau=H.tauNow;cmd=H.heldCommandNow;
tauHist(1)=tau;dr(:,1)=[tau;Cgov.D*z];cmdHist(1)=cmd;
for j=1:3
    E(j,1)=states{j}(1);E1(j,1)=states{j}(2);
    E2(j,1)=A2{j}(1,:)*states{j};
end
profileWatch=tic;
for k=1:N
    for j=1:3
        states{j}=Phi{j}*states{j};
        E(j,k+1)=states{j}(1);
        E1(j,k+1)=states{j}(2);
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
if isfield(routeCache,'pchipCoefficientsPreparedOffTick') && ...
        routeCache.pchipCoefficientsPreparedOffTick
    ref=ppval(routeCache.ppP,tauHist);
    dref=ppval(routeCache.ppDP,tauHist);
    d2ref=ppval(routeCache.ppDDP,tauHist);
else
    ref=interp1(routeCache.tauGrid',routeCache.p',tauHist','pchip')';
    dref=interp1(routeCache.tauGrid',routeCache.dp',tauHist','pchip')';
    d2ref=interp1(routeCache.tauGrid',routeCache.ddp',tauHist','pchip')';
end
routeInterpMs=1000*toc(profileWatch);
profileWatch=tic;
ref(3,:)=ref(3,:)-H.cableLength;
refAcc=dref.*repmat(dr(3,:),3,1) + ...
       d2ref.*repmat(dr(2,:).^2,3,1);
pLoad=ref+E;
vLoad=dref.*repmat(dr(2,:),3,1)+E1;
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
initial.EKFLoadMismatch_m=norm(pLoad(:,1)-H.pastLoad(:,end));
initial.EKFLoadVelocityMismatch_mps=norm(vLoad(:,1)-H.pastVelocity(:,end));
if ~isempty(qEKF) && numel(qEKF)==3 && all(isfinite(qEKF)) && norm(qEKF)>0
    initial.EKFCableAngleMismatch_deg=acos(max(-1,min(1,dot(q(:,1),qEKF))))*180/pi;
end
if isfield(H,'currentEKFDronePosition')
    pd=H.currentEKFDronePosition(:);
    if numel(pd)==3 && all(isfinite(pd))
        initial.EKFDroneMismatch_m=norm(pDrone(:,1)-pd);
        if ~isempty(qEKF) && numel(qEKF)==3 && all(isfinite(qEKF)) && norm(qEKF)>0
            initial.EKFSourceGeometryMismatch_m=norm(H.pastLoad(:,end)-pd-H.cableLength*qEKF);
        end
    end
end
reconstructMs=1000*toc(profileWatch);
F=struct('status','NOMINAL_PV_MODAL_GOVERNOR_NOT_CERTIFIED', ...
    'timeOffsets',(0:N)*h,'pLoad',pLoad,'vLoad',vLoad,'pDrone',pDrone, ...
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
    'cacheBuiltInTick',false,'fitCondition',K.condition, ...
    'initialAnchor',initial,'initialConditionsTakenFromLatestEKF',true);
end

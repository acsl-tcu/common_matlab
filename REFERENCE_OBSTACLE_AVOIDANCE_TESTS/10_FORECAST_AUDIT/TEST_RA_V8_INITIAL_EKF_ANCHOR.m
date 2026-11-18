function OUT=TEST_RA_V8_INITIAL_EKF_ANCHOR()
% Unit test ONLY: all forecast observations are synthetic causal EKF-like
% states. No plant state, oracle, future HLC, or physical safety claim.
agent=DRONE();
agent.parameter=DRONE_PARAM_SUSPENDED_LOAD('DIATONE');
core=Controller_HL_Suspended_Load(.025,agent);
gains=struct('F1',core.F1,'F2',core.F2,'F3',core.F3);
t=-1:.025:0;
K=wp_six_trigger_pv_cache_prepare(gains,t,.70,.10);
route=wp_six_trigger_route_cache_prepare_v8(gen_wp_straight('speed',1.5),0:.025:12);
% Steady governor, tau'=1, tau''=0 at the forecast start.
z=ones(6,1);tau=3; L=2; g=9.81;
tauPast=tau+t;
pr=ppval(route.ppP,tauPast);pr(3,:)=pr(3,:)-L;
vr=ppval(route.ppDP,tauPast);
err=zeros(3,numel(t));edot=err;
A6=diag(ones(5,1),1);
A={A6,A6,[0 1;-reshape(gains.F1,1,[])]};
A{1}(6,:)=-reshape(gains.F2,1,[]);
A{2}(6,:)=-reshape(gains.F3,1,[]);
eta={[.035;-.012;.018;-.008;.004;0], ...
     [-.025;.016;-.014;.006;0;0], [0;0]};
for axis=1:3
    for j=1:numel(t)
        x=expm(A{axis}*t(j))*eta{axis};
        err(axis,j)=x(1);edot(axis,j)=x(2);
    end
end
H=struct('pastTimes',t,'pastLoad',pr+err,'pastVelocity',vr+edot,...
    'pastRefLoad',pr,'pastRefVelocity',vr,...
    'tauNow',tau,'rateNow',z,'heldCommandNow',1,...
    'estimatedML',.14,'cableLength',L,'gravity',g,...
    'slowdownStart',.3,'slowdownFull',.9,'minScale',.05,'filterTime',.70);
F0=wp_six_trigger_pv_forecast_v8(H,K,route);
assert(norm(F0.pLoad(:,1)-H.pastLoad(:,end))<1e-10,...
    'Unconstrained t=0 load position');
assert(norm(F0.vLoad(:,1)-H.pastVelocity(:,end))<1e-10,...
    'Unconstrained t=0 load velocity');
assert(F0.initialAnchor.exactEKFLoadPV);
assert(~F0.initialAnchor.cableAnchorRequested);
q=[.07;-.025;-1];q=q/norm(q);
H.currentEKFCableDirection=q;
H.currentEKFDronePosition=H.pastLoad(:,end)-L*q;
F1=wp_six_trigger_pv_forecast_v8(H,K,route);
assert(F1.initialAnchor.cableAnchorApplied,...
    F1.initialAnchor.cableAnchorStatus);
assert(norm(F1.pLoad(:,1)-H.pastLoad(:,end))<1e-10);
assert(norm(F1.vLoad(:,1)-H.pastVelocity(:,end))<1e-10);
assert(norm(F1.predCableDirection(:,1)-q)<1e-10);
assert(norm(F1.pDrone(:,1)-H.currentEKFDronePosition)<1e-10);
% A cached source and latest exact HLC reference may differ. The
% forecast must still start EXACTLY at current EKF position/velocity.
H.pastRefLoad(:,end)=H.pastRefLoad(:,end)+[.013;-.005;.004];
H.pastRefVelocity(:,end)=H.pastRefVelocity(:,end)+[.02;0;0];
F2=wp_six_trigger_pv_forecast_v8(H,K,route);
assert(F2.initialAnchor.cableAnchorApplied);
assert(norm(F2.pLoad(:,1)-H.pastLoad(:,end))<1e-10);
assert(norm(F2.vLoad(:,1)-H.pastVelocity(:,end))<1e-10);
assert(norm(F2.pDrone(:,1)-H.currentEKFDronePosition)<1e-10);
assert(F2.initialAnchor.routeCacheCurrentReferenceMismatch_m>.01);
% An impossible taut initial constraint is explicitly NOT applied.
H.currentEKFCableDirection=[1;0;0];
F3=wp_six_trigger_pv_forecast_v8(H,K,route);
assert(~F3.initialAnchor.cableAnchorApplied);
assert(strcmp(F3.initialAnchor.cableAnchorStatus,'CABLE_Z_COMPONENT_SINGULAR'));
assert(F3.notCertified && ~F3.isEligibleForSafeClear);
assert(~F1.futurePlantUsed && ~F1.futureHlcUsed);
OUT=struct('status','PASS_EXACT_EKF_PV_AND_CONDITIONAL_CABLE_ANCHOR',...
    't0PositionError_m',F1.initialAnchor.EKFLoadMismatch_m,...
    't0VelocityError_mps',F1.initialAnchor.EKFLoadVelocityMismatch_mps,...
    't0DroneError_m',F1.initialAnchor.EKFDroneMismatch_m,...
    't0CableError_deg',F1.initialAnchor.EKFCableAngleMismatch_deg,...
    'routeCurrentDisagreement_m',F2.initialAnchor.routeCacheCurrentReferenceMismatch_m,...
    'noFutureTruth',true,'safetyCertified',false);
end

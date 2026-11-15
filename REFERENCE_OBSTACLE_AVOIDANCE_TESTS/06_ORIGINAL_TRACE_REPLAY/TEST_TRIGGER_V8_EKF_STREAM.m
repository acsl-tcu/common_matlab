function R=TEST_TRIGGER_V8_EKF_STREAM()
% TEST original trace as a STREAM OF simulated post-update EKF samples.
% This verifies *wiring* and 25ms history management, NOT live hardware EKF.
base=fullfile(fileparts(mfilename('fullpath')),'BASELINE_TRACES', ...
    'PURE_TRIGGER_V6_CAUSAL_AUDIT_20261113_013957.mat');
D=load(base,'R');R6=D.R.original6;
c=R6.results(3);tr=c.nominalTrace;tt=reshape(tr.time,1,[]);
settings=tr.governorSettings;
agent0=DRONE();agent0.parameter=DRONE_PARAM_SUSPENDED_LOAD('DIATONE');
agent0.plant=struct('name','TEST_ONLY');
ctrl=Controller_HL_Suspended_Load(.025,agent0);
K=wp_six_trigger_pv_cache_prepare( ...
    struct('F1',ctrl.F1,'F2',ctrl.F2,'F3',ctrl.F3), ...
    -1:.025:0,settings.filterTime,.1);
route=wp_six_trigger_route_cache_prepare_v8( ...
    gen_wp_straight('speed',1.5),0:.025:8);
known=wp_six_trigger_prepare(six_test_fixture( ...
    [30;30;30],[.1;.1;.1],false),'multiple_spheres');
external=wp_system_geometry_config('C');
cfg=wp_six_trigger_config('C','multiple_spheres',external, ...
    struct('loadMass',c.modelEstimatedMass,'cableLength',c.modelCableLength));
agent=struct();agent.parameter=DRONE_PARAM_SUSPENDED_LOAD('DIATONE');
agent.parameter.set("cableL",struct('cableL',c.modelCableLength));
agent.parameter.set("loadmass",struct('loadmass',999));
agent.estimator.result.state=struct('pL',zeros(3,1), ...
    'vL',zeros(3,1),'mL',c.modelEstimatedMass);
agent.reference=struct('result',struct('state',struct('xd',zeros(28,1))), ...
    'waypoint',struct());
S=struct();
for j=1:41
    agent.estimator.result.state.pL=tr.pLoad(:,j);
    agent.estimator.result.state.vL=tr.vLoad(:,j);
    tau=tr.governorTau(j);
    dp=ppval(route.ppDP,tau);
    xd=zeros(28,1);xd(1:3)=tr.refLoad(:,j);
    xd(5:7)=dp*tr.governorRateState(6,j);
    agent.reference.result.state.xd=xd;
    agent.reference.waypoint=struct( ...
        'virtualTime',tau,'rateState',tr.governorRateState(:,j), ...
        'heldRateCommand',tr.governorHeldCommand(j), ...
        'slowdownStart',settings.slowdownStart, ...
        'slowdownFull',settings.slowdownFull, ...
        'minScale',settings.minScale,'filterTime',settings.filterTime);
    [o,S]=wp_six_trigger_tick_v8(agent,S,K,route,known,cfg,tt(j));
    if j<41
        assert(strcmp(o.status,'UNKNOWN_HISTORY_WARMUP'));
    end
end
assert(S.filled==41 && isfinite(o.totalMs));
assert(~o.safeToCommit && ~o.physicalClearCertified);
assert(abs(o.estimatedMassUsed-c.modelEstimatedMass)<1e-12);
assert(~strcmp(o.status,'UNKNOWN'),o.reason);
% Read latest *new* mL without any source parameter update.
agent.estimator.result.state.mL=.13;
j=42;
agent.estimator.result.state.pL=tr.pLoad(:,j);
agent.estimator.result.state.vL=tr.vLoad(:,j);
tau=tr.governorTau(j);dp=ppval(route.ppDP,tau);
xd=zeros(28,1);xd(1:3)=tr.refLoad(:,j);
xd(5:7)=dp*tr.governorRateState(6,j);
agent.reference.result.state.xd=xd;
agent.reference.waypoint.virtualTime=tau;
agent.reference.waypoint.rateState=tr.governorRateState(:,j);
agent.reference.waypoint.heldRateCommand=tr.governorHeldCommand(j);
[o2,S]=wp_six_trigger_tick_v8(agent,S,K,route,known,cfg,tt(j));
assert(abs(o2.estimatedMassUsed-.13)<1e-12, ...
    'V8:Mass','Predicted model did not use LATEST EKF mL');
assert(abs(agent.parameter.get("loadmass")-999)<1e-12, ...
    'V8:NoWrites','Parameter loadmass changed by predictor');
assert(S.filled==41);
% Deliberately missed flight cycle must reset history (no stale forecasts).
[o3,S3]=wp_six_trigger_tick_v8(agent,S,K,route,known,cfg,tt(j)+.2);
assert(strcmp(o3.status,'UNKNOWN_HISTORY_WARMUP') && S3.filled==1);
R=struct('status','PASS_SIMULATED_CURRENT_EKF_STREAM', ...
    'warmupTicks',40,'historyLength',S.filled, ...
    'latestMassAtTick',o2.estimatedMassUsed, ...
    'parameterLoadMassAfter',agent.parameter.get("loadmass"), ...
    'missedTickResetsHistory',S3.filled==1, ...
    'actualLiveEKFUsed',false,'physicalSafetyCertified',false);
end

function R=TEST_RA_TICK_25MS_HISTORY()
% Tests the no-observed-obstacle path, 41 true samples and missing tick.
% Synthetic agent is a wiring-only fixture, not an EKF accuracy test.
agent=RA_TEST_AGENT();
agent.estimator.result.state.pL=[0;0;1];
agent.estimator.result.state.vL=zeros(3,1);
agent.reference.result.state.xd=zeros(28,1);
agent.reference.waypoint=struct('virtualTime',0,'rateState',zeros(6,1), ...
    'heldRateCommand',1,'slowdownStart',.1,'slowdownFull',1, ...
    'minScale',.1,'filterTime',.1);
Q=reference_avoidance_prepare_observed(agent,'B','multiple_spheres', ...
    struct('parts',struct([]),'approximation','multiple_spheres'));
K=struct('pastRelativeTimes',-1:.025:0,'samplePeriod_s',.025);
S=struct();
for k=0:40
    agent.estimator.result.state.mL=0.14 + 0.001*(k==40);
    [out,S]=reference_avoidance_tick(agent,S,K,struct(), ...
        Q.known,Q.cfg,k*.025);
    if k<40
        assert(strcmp(out.status,'UNKNOWN_HISTORY_WARMUP'));
    else
        assert(strcmp(out.status,'NO_OBSERVED_OBSTACLES_SKIP_PREDICTION'));
        assert(out.historySamples==41);
        assert(abs(out.estimatedMassUsed-0.141)<1e-12);
    end
end
assert(agent.parameter.loadmass==999); % no truth value read/write
% Missing a 25-ms tick invalidates the PV history.
[out,S]=reference_avoidance_tick(agent,S,K,struct(), ...
    Q.known,Q.cfg,1.050);
assert(strcmp(out.status,'UNKNOWN_HISTORY_WARMUP') && S.filled==1);
R=struct('status','PASS','historySamplesBeforeMiss',41, ...
    'historySamplesAfterMiss',S.filled, ...
    'latestMassOnly',true,'sensorImplemented',false);
end

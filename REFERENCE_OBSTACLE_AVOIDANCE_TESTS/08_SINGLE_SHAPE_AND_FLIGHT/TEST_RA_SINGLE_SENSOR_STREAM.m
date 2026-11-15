function T=TEST_RA_SINGLE_SENSOR_STREAM()
%TEST_RA_SINGLE_SENSOR_STREAM 41x25ms observation->selected-mode entry.
% Wiring-only synthetic fixture (NOT actual Flight dynamics).
agent=RA_TEST_AGENT();
agent.estimator.result.state.pL=[0;0;1];
agent.estimator.result.state.vL=zeros(3,1);
agent.reference.result.state.xd=zeros(28,1);
agent.reference.waypoint=struct('virtualTime',0,'rateState',zeros(6,1), ...
    'heldRateCommand',1,'slowdownStart',.1,'slowdownFull',1, ...
    'minScale',.1,'filterTime',.1);
world=DRONE_OBSTACLES('scene_single_sphere','single_ellipsoid','none');
I=ra_obstacle_method_info(world);
assert(I.singlePerObject && I.numParts==1);
P=struct('modalCache',struct('pastRelativeTimes',-1:.025:0, ...
   'samplePeriod_s',.025),'routeCache',struct());
cfg=ra_sim_sensor_config(20);
S=struct();
for k=0:40
    % Deliberately no observations: no future forecast is needed, but the
    % current EKF history MUST be updated without faking samples.
    [A,S]=ra_sim_sensor_trigger_step(agent,world,[100;0;0], ...
        cfg,'D',P,k*.025,S,struct('available',false));
    assert(strcmp(A.obstacleApproximation,'single_ellipsoid'));
    assert(A.detectedParts==0);
    assert(A.monitorOnly && ~A.trigger.safeToCommit);
    if k<40
        assert(strcmp(A.status,'UNKNOWN_HISTORY_WARMUP'));
    else
        assert(strcmp(A.status,'NO_OBSERVED_OBSTACLES_SKIP_PREDICTION'));
    end
end
assert(S.geometryRebuilds==1,'No unnecessary cache rebuild on same observation');
assert(S.history.filled==41 && abs(A.currentEstimatedMass-.14)<1e-12);
% Same physical world, but now sensor sees an obstacle. Cached observed
% parts must change once; the world source's method stays authoritative.
[B,S]=ra_sim_sensor_trigger_step(agent,world,[0;0;0], ...
    cfg,'D',P,1.025,S,struct('available',false));
assert(B.geometryCacheRebuilt && S.geometryRebuilds==2);
assert(B.detectedParts==1 && strcmp(B.obstacleKind,'ellipsoid'));
T=struct('status','PASS_CURRENT_SENSOR_TO_ONE_MODE_STREAM', ...
    'ticks',42,'cacheRebuilds',S.geometryRebuilds, ...
    'latestMassFromEKF',B.currentEstimatedMass, ...
    'actualFlightTested',false,'forecastAgainstHLCNotReplayed',true);
end

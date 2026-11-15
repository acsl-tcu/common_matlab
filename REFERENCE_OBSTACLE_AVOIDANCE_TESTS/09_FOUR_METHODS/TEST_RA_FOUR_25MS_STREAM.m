function T=TEST_RA_FOUR_25MS_STREAM()
%TEST_RA_FOUR_25MS_STREAM 4 original method tags, 41 x 25-ms EKF ticks.
% Synthetic empty-observation test, not full Flight dynamics / avoidance.
methods={'single_sphere','multiple_spheres','single_ellipsoid','multiple_ellipsoids'};
modes={'C','B','D','C'};
agent=RA_TEST_AGENT();
agent.estimator.result.state.pL=[0;0;1];
agent.estimator.result.state.vL=zeros(3,1);
agent.reference.result.state.xd=zeros(28,1);
agent.reference.waypoint=struct('virtualTime',0,'rateState',zeros(6,1), ...
    'heldRateCommand',1,'slowdownStart',.1,'slowdownFull',1, ...
    'minScale',.1,'filterTime',.1);
cache=struct('modalCache',struct('pastRelativeTimes',-1:.025:0, ...
    'samplePeriod_s',.025),'routeCache',struct());
rows=struct('method',{},'systemMode',{},'ticks',{}, ...
    'massAtFinalTick',{},'lastStatus',{});
for j=1:4
    world=struct('parts',struct([]),'objects',struct([]), ...
        'ground',[],'approximation',methods{j});
    state=struct();
    for k=0:40
        agent.estimator.result.state.mL=.14+.0001*k;
        [out,state]=ra_sim_sensor_trigger_step(agent,world,[100;0;0], ...
            ra_sim_sensor_config(20),modes{j},cache,k*.025,state, ...
            struct('available',false));
        assert(strcmp(out.obstacleApproximation,methods{j}));
        assert(out.detectedParts==0 && out.monitorOnly && ~out.trigger.safeToCommit);
    end
    assert(state.history.filled==41);
    assert(strcmp(out.status,'NO_OBSERVED_OBSTACLES_SKIP_PREDICTION'));
    assert(abs(out.currentEstimatedMass-.144)<1e-12);
    assert(state.geometryRebuilds==1);
    rows(end+1)=struct('method',methods{j},'systemMode',modes{j}, ...
        'ticks',41,'massAtFinalTick',out.currentEstimatedMass, ...
        'lastStatus',out.status); %#ok<AGROW>
end
assert(agent.parameter.loadmass==999); % must not read true/sentinel load mass
T=struct('status','PASS_FOUR_25MS_NO_OBSERVATION_STREAM', ...
    'records',rows,'ticksPerMethod',41,'massSource','LATEST_EKF', ...
    'realFlightTested',false);
end

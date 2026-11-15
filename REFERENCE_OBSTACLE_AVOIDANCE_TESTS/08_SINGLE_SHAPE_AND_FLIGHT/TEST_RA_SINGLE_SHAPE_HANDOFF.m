function T=TEST_RA_SINGLE_SHAPE_HANDOFF()
%TEST_RA_SINGLE_SHAPE_HANDOFF Authoritative DRONE_OBSTACLES->sensor->C/B/D.
% No HLC flight or CBF control: only single-part interface and synthetic
% nominal 4s geometry. The original generator must be on MATLAB path.
if exist('DRONE_OBSTACLES','file')~=2 || ...
        exist('scene_mixed_obstacles','file')~=2
    error('RA:GeneratorMissing', ...
        'Original DRONE_OBSTACLES + scenes must be on MATLAB path.');
end
methods={'single_sphere','single_ellipsoid'};
modes={'C','B','D'};
agent=RA_TEST_AGENT();
tested=0; evaluated=0; checks=0;
for m=1:2
    world=DRONE_OBSTACLES('scene_mixed_obstacles',methods{m},'none');
    I=ra_obstacle_method_info(world);
    assert(strcmp(I.method,methods{m}) && I.singlePerObject);
    assert(numel(world.parts)==numel(world.objects));
    [obs,sensorReport]=ra_sim_sensor_detect(world,[0;0;0],ra_sim_sensor_config(20));
    assert(strcmp(obs.approximation,methods{m}));
    assert(sensorReport.returnedPartCount==sensorReport.detectedObjectCount);
    assert(sensorReport.definitionReadOnly);
    inspector=ra_obstacle_inspect_definition(world,false);
    assert(inspector.numParts==numel(world.objects));
    assert(strcmp(inspector.worldMethod,methods{m}));
    for j=1:numel(obs.parts)
        ix=find(strcmp({world.parts.obstacleName},obs.parts(j).obstacleName),1);
        assert(isequaln(obs.parts(j),world.parts(ix)), ...
            'Sensor must return the ORIGINAL part unchanged');
    end
    % Each of C/B/D independently uses the exact same world-selected cover.
    for k=1:numel(modes)
        Q=reference_avoidance_prepare_observed(agent,modes{k},obs,[], ...
            struct('allowMexBuild',false));
        assert(strcmp(Q.cfg.obstacleApproximation,methods{m}));
        assert(strcmp(Q.known.approximation,methods{m}));
        assert(strcmp(Q.cfg.obstacleKind,I.kind));
        assert(Q.known.numParts==numel(obs.parts));
        assert(Q.singlePartPerObject && ~Q.cfg.useMexCChainPreclear);
        % Nominal geometry-only: no 4s future predictor/Flight launched.
        F=RA_TEST_FORECAST(Q.cfg);
        G=wp_six_trigger_evaluate_fast_v8(Q.known,Q.cfg,F);
        assert(~strcmp(G.status,'UNKNOWN'), ...
            'The selected shape must reach the geometry evaluator');
        evaluated=evaluated+1;
    end
    % Mismatch is an ERROR: no silent reinterpretation of sphere as ellipsoid.
    wrong=obs;
    if strcmp(methods{m},'single_sphere')
        wrong.approximation='single_ellipsoid';
    else
        wrong.approximation='single_sphere';
    end
    expectFailure(@()ra_obstacle_method_info(wrong));
    expectFailure(@()wp_six_trigger_prepare(obs,wrong.approximation));
    expectFailure(@()ra_sim_sensor_detect(world,[0;0;0], ...
        ra_sim_sensor_config(wrong.approximation,20)));
    tested=tested+1;checks=checks+3;
end
% Radius-only config never declares a sphere/ellipsoid type.
cfg=ra_sim_sensor_config();
assert(cfg.radiusM==20 && strcmp(cfg.approximation,'FROM_WORLD'));
cfg=ra_sim_sensor_config(31.5);assert(cfg.radiusM==31.5);
% Definition corruption: two parts for a one-part object -> reject.
w=DRONE_OBSTACLES('scene_mixed_obstacles','single_sphere','none');
w.parts(end+1)=w.parts(1);expectFailure(@()ra_obstacle_method_info(w));
T=struct('status','PASS_SINGLE_SOURCE_AND_SIX_INDEPENDENT_HANDOFF', ...
    'testedObstacleMethods',tested,'evaluatedSystemCombinations',evaluated, ...
    'mismatchFailuresExpected',checks,'actualFlightTested',false, ...
    'realCollisionGuarantee',false);
end
function expectFailure(fn)
raised=false;
try,fn();catch,raised=true;end
assert(raised,'Expected mismatch validation error');
end

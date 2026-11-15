function out=TEST_SIX_TRIGGER_DRONE_OBSTACLES()
%TEST_SIX_TRIGGER_DRONE_OBSTACLES Real original obstacle generator (offline).
% Simulation's world is a test fixture. It must NEVER bypass the sensor
% in actual Flight. This test chooses one approximation per independent run.
scenes={'scene_single_sphere','scene_gate_square','scene_mixed_obstacles'};
if exist('DRONE_OBSTACLES','file')~=2 || ...
        any(cellfun(@(s)exist(s,'file')~=2,scenes))
    out=struct('status','SKIPPED_REAL_OBSTACLE_GENERATOR_NOT_ON_PATH', ...
        'scenesChecked',0);
    fprintf('SKIP DRONE_OBSTACLES: original generator must be on MATLAB path.\n');
    return
end
options=struct('maxPartsPerObstacle',6,'maxTotalParts',36, ...
    'targetPartSize',3.0,'superExponent',8, ...
    'hollowSides',16,'polySides',8);
modes={'C','B','D'};covers={'multiple_spheres','multiple_ellipsoids'};
counts=zeros(numel(scenes),2);
for si=1:numel(scenes)
    for ci=1:2
        cover=covers{ci};
        world=DRONE_OBSTACLES(scenes{si},cover,'none',options);
        counts(si,ci)=numel(world.parts);
        for mi=1:3
            [~,cfg,F]=selected_test_context(modes{mi},cover);
            known=wp_six_trigger_prepare(world,cover);
            R=wp_six_trigger_evaluate(known,cfg,F);
            assert(~strcmp(R.status,'UNKNOWN'), ...
                'Unexpected UNKNOWN with original static obstacle generator');
            assert(strcmp(R.systemMode,modes{mi}));
        end
    end
    fprintf('PASS original obstacle scene %-23s  parts sphere %d / ellipsoid %d\n', ...
        scenes{si},counts(si,1),counts(si,2));
end
out=struct('status','PASS_ORIGINAL_GENERATOR_6_INDEPENDENT_RUNS', ...
    'scenesChecked',numel(scenes),'modeCalls',numel(scenes)*6, ...
    'partsByScene',counts,'onlyOfflineOmniscientFixture',true);
end

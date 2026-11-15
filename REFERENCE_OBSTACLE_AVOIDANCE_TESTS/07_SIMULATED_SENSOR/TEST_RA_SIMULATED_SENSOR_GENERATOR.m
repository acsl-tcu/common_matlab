function T=TEST_RA_SIMULATED_SENSOR_GENERATOR()
% Optional integration with the real DRONE_OBSTACLES generator (not bundled).
% Always report SKIPPED distinctly when original generator is absent.
if exist('DRONE_OBSTACLES','file')~=2 || ...
        exist('scene_mixed_obstacles','file')~=2
    T=struct('status','SKIPPED_ORIGINAL_GENERATOR_NOT_ON_PATH', ...
        'sceneName','scene_mixed_obstacles','testedCovers',0);return;
end
covers={'multiple_spheres','multiple_ellipsoids'};
for i=1:numel(covers)
    world=DRONE_OBSTACLES('scene_mixed_obstacles',covers{i},'none');
    for radius=[1,20,1000]
        cfg=ra_sim_sensor_config(covers{i},radius);
        [observed,info]=ra_sim_sensor_detect(world,[0;0;0],cfg);
        assert(numel(observed.parts)==info.returnedPartCount);
        if ~isempty(observed.parts)
            assert(all(ismember({observed.parts.obstacleName}, ...
                info.detectedObjectNames)));
        end
        % Verify each detected object's ALL original cover parts were kept.
        for k=1:numel(info.detectedObjectNames)
            name=info.detectedObjectNames{k};
            assert(nnz(strcmp({observed.parts.obstacleName},name))== ...
                nnz(strcmp({world.parts.obstacleName},name)));
        end
        assert(numel(observed.objects)==info.detectedObjectCount);
        if radius==1000
            assert(numel(observed.parts)==numel(world.parts));
        end
    end
end
T=struct('status','PASS_ORIGINAL_GENERATOR_TWO_COVERS', ...
    'sceneName','scene_mixed_obstacles','testedCovers',2, ...
    'testedRadii',[1 20 1000]);
end

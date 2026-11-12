%% TEMP: 4-second nominal reference lookahead, scenarios / numbers / speeds.
% Put this script in commonmatlab/ root. Requires DRONE_OBSTACLES.
root=fileparts(mfilename('fullpath'));addpath(genpath(root));
methods={'C','B','D'};
world=struct('parts',[],'ground',ground_z0(),'dynamicIndices',[]);
world.parts=obs_new_part('sphere',[1.8 0 2],eye(3),struct('radius',0.30));
world.parts.obstacleName='in_path';
for m=1:numel(methods)
  cfg=wp_system_geometry_config(methods{m});
  for speed=[0.2 0.5 1.5]
    ref=@(t) local_reference(t,speed,0,3,0);
    out=wp_preview_4s(world,cfg,ref,5,struct('sampleDt',0.10));
    assert(all(out.validSamples));
    assert(~strcmp(out.status,'prediction_invalid'));
    if speed>=0.5
        assert(out.collisionDetected, ...
            'Expected in-path collision: %s speed=%.2f',methods{m},speed);
    else
        assert(~out.collisionDetected, ...
            'Slow reference should not reach obstacle within 4s: %s',methods{m});
    end
    assert(numel(out.times)==41);
    fprintf('PASS 4s %s speed=%.2f firstHit=%.3f elapsed=%.4fs\n', ...
        methods{m},speed,out.firstHitTime,out.totalElapsedSeconds);
  end
end
% Tracking departure is NOT automatically a safe prediction.
slow=@(t) local_reference(t,0.2,0,3,0);
optsMeasured=struct('sampleDt',0.10,'currentPose', ...
    struct('pDrone',[0;2;3],'pLoad',[0;2;1]), ...
    'maxTrackingDeviation',0.30);
check=wp_preview_4s(world,wp_system_geometry_config('B'),slow,5,optsMeasured);
assert(strcmp(check.status,'tracking_deviation_unbounded'));
% Deliberate free-fall flatness singularity => unknown, NOT clear.
errRef=@(t) local_reference(t,0.5,0,3,-9.81);
out=wp_preview_4s(world,wp_system_geometry_config('B'),errRef,5);
assert(strcmp(out.status,'prediction_invalid') && ~all(out.validSamples));
% Static scenes and dynamic scene with both obstacle families.
opts=struct('maxPartsPerObstacle',6,'maxTotalParts',36,'targetPartSize',3, ...
    'polySides',8,'hollowSides',16);
scenes={'scene_gate_square','scene_mixed_obstacles', ...
        'scene_cylinders_high_side','scene_dynamic_independent'};
for i=1:numel(scenes)
  for n=1:2
    obstacleMethod='multiple_spheres';
    if n==2,obstacleMethod='multiple_ellipsoids';end
    world=DRONE_OBSTACLES(scenes{i},obstacleMethod,'ground_z0',opts);
    for m=1:3
       cfg=wp_system_geometry_config(methods{m});
       ref=@(t) local_reference(t,1.5,0,3,0);
       out=wp_preview_4s(world,cfg,ref,5,struct('sampleDt',0.2));
       assert(all(out.validSamples));
       fprintf('%-26s %-20s system=%s parts=%2d samples=%2d hit=%d time=%.4fs\n', ...
           scenes{i},obstacleMethod,methods{m},numel(world.parts), ...
           numel(out.times),out.collisionDetected,out.totalElapsedSeconds);
    end
  end
end
% Post-change no side effects: this function never touches the state of
% WAYPOINT_REFERENCE, as tests above use a read-only function handle.
fprintf('ALL TEMP 4s PREVIEW TESTS PASS (MATLAB execution)\n');

function xd=local_reference(t,speed,y0,z0,az)
% Flight starts at t=5; x advances with nominal constant speed for testing.
xd=zeros(28,1);xd(1:3)=[speed*max(0,t-5);y0;z0];
xd(5:7)=[speed;0;0];xd(9:11)=[0;0;az];
end

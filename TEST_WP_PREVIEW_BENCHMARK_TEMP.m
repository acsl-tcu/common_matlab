%% TEMP: diagnostic timing under obstacle number / sampleDt / approach.
% Put in commonmatlab/ root. Not a guarantee of 25-ms control feasibility.
root=fileparts(mfilename('fullpath'));addpath(genpath(root));
scenes={'scene_single_sphere','scene_mixed_obstacles', ...
    'scene_cylinders_high_side','scene_showcase_13'};
opts=struct('maxPartsPerObstacle',6,'maxTotalParts',36,'targetPartSize',3, ...
    'polySides',8,'hollowSides',16);
for s=1:numel(scenes)
 for ao=1:2
   obsMethod='multiple_spheres';if ao==2,obsMethod='multiple_ellipsoids';end
   w=DRONE_OBSTACLES(scenes{s},obsMethod,'ground_z0',opts);
   for mode={'C','B','D'}
    cfg=wp_system_geometry_config(mode{1});
    for v=[0.2 1.5]
     for step=[0.10 0.20]
       f=@(t) local_nominal_reference(t,v);
       out=wp_preview_4s(w,cfg,f,5,struct('sampleDt',step));
       assert(all(out.validSamples));
       fprintf('BENCH %-25s %-19s sys=%s speed=%.1f dt=%.2f obstacles=%2d parts=%2d eval=%2d wall=%.4fs maxSample=%.4fs\n', ...
           scenes{s},obsMethod,mode{1},v,step,numel(w.objects), ...
           numel(w.parts),numel(out.times),out.totalElapsedSeconds,max(out.perSampleCollisionTime));
     end
    end
   end
 end
end
fprintf('BENCHMARK FINISHED; compare speed against desired online budget yourself.\n');
function xd=local_nominal_reference(t,v)
xd=zeros(28,1);xd(1:3)=[v*max(t-5,0);0;3];xd(5:7)=[v;0;0];
end

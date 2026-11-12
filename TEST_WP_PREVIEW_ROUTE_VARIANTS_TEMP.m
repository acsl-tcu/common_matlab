%% TEMP: test actual V3 Waypoint route generators with 4s collision preview.
% Put in commonmatlab/ root. Does not modify live WAYPOINT_REFERENCE state.
root=fileparts(mfilename('fullpath'));addpath(genpath(root));
opts=struct('maxPartsPerObstacle',6,'maxTotalParts',36,'targetPartSize',3, ...
    'polySides',8,'hollowSides',16);
routeList={'gen_wp_straight','gen_wp_gentle_turn', ...
    'gen_wp_circle','gen_wp_saddle'};
methodList={'C','B','D'};
for ri=1:numel(routeList)
   speed=0.3;if mod(ri,2)==0,speed=1.0;end
   route=feval(routeList{ri},'speed',speed);
   assert(isa(route.evaluate,'function_handle'));
   ref=@(t) route.evaluate(max(0,t-5));
   scene='scene_gate_square';
   if mod(ri,2)==0,scene='scene_mixed_obstacles';end
   for oi=1:2
       obstacleMethod='multiple_spheres';
       if oi==2,obstacleMethod='multiple_ellipsoids';end
       world=DRONE_OBSTACLES(scene,obstacleMethod,'ground_z0',opts);
       for mi=1:3
           cfg=wp_system_geometry_config(methodList{mi});
           pred=wp_preview_4s(world,cfg,ref,5, ...
               struct('horizon',4,'sampleDt',0.20,'predictionMode','flatness'));
           assert(all(pred.validSamples), ...
               'Invalid prediction: %s/%s/%s, %s', ...
               routeList{ri},obstacleMethod,methodList{mi},pred.invalidReason);
           assert(numel(pred.times)==21);
           fprintf('PASS %-20s speed=%.2f scene=%s obstacle=%s C/B/D=%s hit=%d elapsed=%.4fs\n', ...
               routeList{ri},speed,scene,obstacleMethod,methodList{mi}, ...
               pred.collisionDetected,pred.totalElapsedSeconds);
       end
   end
end
fprintf('ALL TEMP V3 ROUTE PREVIEW TESTS PASS (24 route/shape/scene combinations).\n');

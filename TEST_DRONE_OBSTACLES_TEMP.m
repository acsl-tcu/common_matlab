% TEST_DRONE_OBSTACLES_TEMP.m
% TEMPORARY MATLAB verification. Place directly in commonmatlab root.
% Delete this file only after it passes in the actual MATLAB environment.
% Tests numerical samples only; does not prove full obstacle/CBF safety.
clearvars;close all;clc;
root_dir=fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(root_dir,'environment','DRONE_OBSTACLES')));
rng(20261111);
methods={'single_sphere','single_ellipsoid','multiple_spheres', ...
         'multiple_ellipsoids','superellipsoid','convex_polyhedra'};
scenes={'scene_showcase_13','scene_holes','scene_gate_square','scene_mixed_obstacles'};
opts=struct('subdivisions',[2 2 1],'superSubdivisions',[1 1 1], ...
 'superExponent',8,'hollowSides',16,'polySides',8);
passes=0; failures=cell(0,1);
for si=1:numel(scenes)
 objs=feval(scenes{si});
 for oi=1:numel(objs)
  o=objs(oi); C=obs_components(o,opts);
  [lo,hi]=obs_local_bounds(C);
  % Random points inside guaranteed local bounding box, then retain real solid points.
  Q=lo+rand(800,3).*(hi-lo);
  P=(o.R*Q')'+o.position;
  mask=obs_original_contains(o,P,opts); P=P(mask,:);
  % Check also every corner of conservative component bounding boxes.
  % (Polyhedral exact-hull mode need not include bounding-box corners.)
  % Every vertex of each convex component is also a relevant boundary point.
  for ci=1:numel(C)
   if strcmp(C(ci).kind,'poly')
    PP=(o.R*C(ci).vertices')'+o.position;
    P=[P;PP]; %#ok<AGROW>
   end
  end
  assert(~isempty(P),'No test points for %s',o.name);
  for mi=1:numel(methods)
   try
    [parts,meta]=obs_approximate(o,methods{mi},opts);
    inside=obs_world_contains(parts,P,5e-7);
    % Exact analytic primitive parts need not include AABB corners.
    if ~strcmp(methods{mi},'convex_polyhedra')
      cornersToCheck=zeros(0,3);
      for ci=1:numel(C)
        if strcmp(methods{mi},'multiple_spheres') && strcmp(C(ci).kind,'sphere'),continue;end
        if strcmp(methods{mi},'multiple_ellipsoids') && strcmp(C(ci).kind,'ellipsoid'),continue;end
        [a,b]=obs_component_bounds(C(ci)); VV=obs_box_vertices(a,b);
        cornersToCheck=[cornersToCheck;(o.R*VV')'+o.position]; %#ok<AGROW>
      end
      if ~isempty(cornersToCheck)
        assert(all(obs_world_contains(parts,cornersToCheck,5e-7)), ...
            'Certified bounding-box corner not enclosed');
      end
    end
    assert(all(inside),'Containment fail: %s (%s), %d points outside', ...
      o.name,methods{mi},sum(~inside));
    assert(meta.numParts==numel(parts),'Part count mismatch');
    passes=passes+1;
    fprintf('PASS %-29s %-22s parts=%4d samples=%d\n', ...
        o.name,methods{mi},numel(parts),size(P,1));
   catch ME
    failures{end+1,1}=sprintf('FAIL %s / %s: %s',o.name,methods{mi},ME.message); %#ok<SAGROW>
    fprintf(2,'%s\n',failures{end});
   end
  end
 end
end
% Ground definition and per-obstacle override
W=DRONE_OBSTACLES('scene_mixed_obstacles','multiple_ellipsoids','ground_z0',opts);
assert(isequal(ground_clearance(W.ground,[0 0 1;0 0 -1]),[1;-1]), ...
 'Ground half-space check failed');
opts.perObstacle=struct('large_box','convex_polyhedra');
W2=DRONE_OBSTACLES('scene_mixed_obstacles','multiple_ellipsoids','ground_z0',opts);
assert(strcmp(W2.meta(1).method,'convex_polyhedra'),'perObstacle override failed');
fprintf('\nCompleted %d approximation cases; failed=%d\n',passes,numel(failures));
if ~isempty(failures),error('DRONE_OBSTACLES tests failed; inspect FAIL messages');end
fprintf('All tests passed. The temporary TEST_DRONE_OBSTACLES_TEMP.m can now be removed.\n');
% Optional: obs_plot_world(W2,true);

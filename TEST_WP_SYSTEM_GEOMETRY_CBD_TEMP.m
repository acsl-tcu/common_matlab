%% TEMP: Geometry and collision sanity tests. Put THIS .m in commonmatlab/ root.
% Requires existing environment/DRONE_OBSTACLES and new
% reference/WAYPOINT_REFERENCE/SAFETY_PREVIEW_GEOMETRY in MATLAB path.
% MATLAB execution must be performed by the user; not an absolute proof.
root=fileparts(mfilename('fullpath'));addpath(genpath(root));
methods={'C','B','D'};samples=0:0.025:1;
world=struct('parts',[],'ground',ground_z0(),'dynamicIndices',[]);
for im=1:numel(methods)
    cfg=wp_system_geometry_config(methods{im}, ...
        struct('cableLength',2.0,'cableSegments',8));
    pD=[0;0;3];pL=[0;0;1];
    shape=wp_system_shape_at_pose(cfg,pD,pL);
    assert(shape.cableLengthActual==2);
    % Physical capsule of radius cableRadius+safetyMargin is included
    % by Model C and Model D, including off-axis directions.
    if any(strcmp(cfg.method,{'C','D'}))
        rad=cfg.cableRadius+cfg.safetyMargin;
        for s=samples
            for phi=[0 pi/3 2*pi/3 pi]
                pt=pD+s*(pL-pD)+rad*[cos(phi);sin(phi);0];
                cable=shape.items(2:end-1);
                covered=false;
                for j=1:numel(cable)
                    if strcmp(cable(j).kind,'sphere')
                        covered=covered || norm(pt-cable(j).center)<=cable(j).radius+1e-10;
                    else
                        u=cable(j).R'*(pt-cable(j).center);
                        covered=covered || sum((u./cable(j).axes).^2)<=1+1e-10;
                    end
                end
                assert(covered,'Cable point uncovered for method %s at s=%.3f',cfg.method,s);
            end
        end
    end
    % Collision with a sphere at the cable midpoint.
    world.parts=obs_new_part('sphere',[0.10 0 2],eye(3),struct('radius',0.20));
    world.parts.obstacleName='sphere_near_cable';
    report=wp_system_collision(cfg,shape,world,0);
    assert(report.collision,'Missing sphere collision for %s',cfg.method);
    % Distant obstacle: no collision.
    world.parts.center=[5 5 5];
    report=wp_system_collision(cfg,shape,world,0);
    assert(~report.collision,'False collision with far sphere for %s',cfg.method);
    % Rotated ellipsoid obstacle, near cable.
    R=obs_rpy_matrix([18 30 55]);
    world.parts=obs_new_part('ellipsoid',[0.14 0 2],R,struct('axes',[0.25 0.35 0.45]));
    world.parts.obstacleName='ellipsoid_near_cable';
    report=wp_system_collision(cfg,shape,world,0);
    assert(report.collision,'Missing ellipsoid collision for %s',cfg.method);
    world.parts.center=[8 8 8];
    report=wp_system_collision(cfg,shape,world,0);
    assert(~report.collision,'False far ellipse hit for %s',cfg.method);
    % Ground clearance accounts for whole system and optional shape radius.
    world.parts=[];
    shapeLow=wp_system_shape_at_pose(cfg,[0;0;2.04],[0;0;0.04]);
    report=wp_system_collision(cfg,shapeLow,world,0);
    assert(report.collision && report.minGroundClearance<=0, ...
        'Missing load-ground collision in %s',cfg.method);
    fprintf('PASS %s: sphere, rotated ellipsoid, far, ground, cable containment\n',cfg.method);
end
% Verify all world objects use the selected method with varying size/pose.
options=struct('maxPartsPerObstacle',6,'maxTotalParts',36,'targetPartSize',3, ...
    'polySides',8,'hollowSides',16);
scenes={'scene_gate_square','scene_mixed_obstacles', ...
        'scene_cylinders_high_side','scene_dynamic_independent'};
for i=1:numel(scenes)
  for k=1:2
    method='multiple_spheres';if k==2,method='multiple_ellipsoids';end
    world=DRONE_OBSTACLES(scenes{i},method,'ground_z0',options);
    for j=1:numel(methods)
        cfg=wp_system_geometry_config(methods{j});
        shape=wp_system_shape_at_pose(cfg,[0;0;3],[0;0;1]);
        r0=wp_system_collision(cfg,shape,world,0);
        r5=wp_system_collision(cfg,shape,world,5);
        assert(islogical(r0.collision) && islogical(r5.collision));
        assert(isfinite(r0.minClearance) && isfinite(r5.minClearance));
    end
    fprintf('PASS scene=%s obstacleApprox=%s parts=%d\n',scenes{i},method,numel(world.parts));
  end
end
fprintf('ALL TEMP SYSTEM GEOMETRY TESTS PASS (MATLAB execution)\n');

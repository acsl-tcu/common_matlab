function report=wp_system_collision(cfg,shape,world,t)
% Each model calls its dedicated collision loop; common geometric kernels.
% report.minClearance is metres for sphere/capsule comparisons, but for
% ellipsoid-vs-ellipsoid it is a sign-correct distance PROXY, not Euclidean.
if nargin<4,t=0;end
if isfield(world,'dynamicIndices') && ~isempty(world.dynamicIndices)
    snapshot=obs_world_at_time(world,t);
else
    snapshot=world;
end
switch cfg.method
    case 'C', report=wp_collision_C(cfg,shape,snapshot);
    case 'B', report=wp_collision_B(cfg,shape,snapshot);
    case 'D', report=wp_collision_D(cfg,shape,snapshot);
    otherwise, error('WPGeometry:CollisionMode','Unknown mode');
end
report.time=t;
report.method=cfg.method;
end

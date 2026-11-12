function report=wp_collision_B(cfg,shape,world)
% MODEL B specific: finite-segment capsule against spherical/ellipsoid parts.
report=wp_collision_common(cfg,shape,world,'capsule',@contact_B);
end
function [g,method]=contact_B(item,part,near)
[lb,skip]=wp_pair_far_bound(item,part,near);
if skip,g=lb;method='circumsphere_lower_bound';return;end
if strcmp(item.kind,'sphere')
    [g,method]=wp_sphere_vs_part(item,part);
elseif strcmp(item.kind,'capsule')
    if strcmp(part.kind,'sphere')
        g=wp_segment_point_distance(item.p0,item.p1,part.center(:)) ...
            -item.radius-part.params.radius;
        method='capsule_sphere_euclidean';
    else
        g=wp_segment_ellipsoid_distance(item.p0,item.p1,part.center(:), ...
            part.R,part.params.axes)-item.radius;
        method='capsule_ellipsoid_euclidean';
    end
else
    error('WPPreview:ModelBPart','Unexpected system part kind');
end
end

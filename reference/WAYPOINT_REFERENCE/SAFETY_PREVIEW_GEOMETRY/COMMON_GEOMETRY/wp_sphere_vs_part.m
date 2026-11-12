function [g,method]=wp_sphere_vs_part(item,part)
if strcmp(part.kind,'sphere')
    g=norm(item.center-part.center(:))-item.radius-part.params.radius;
    method='sphere_sphere_euclidean';
elseif strcmp(part.kind,'ellipsoid')
    g=wp_point_ellipsoid_distance(item.center,part.center(:),part.R,part.params.axes)-item.radius;
    method='sphere_ellipsoid_euclidean';
else
    error('WPPreview:ObstacleType','Sphere/ellipsoid only');
end
end

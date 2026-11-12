function report=wp_collision_D(cfg,shape,world)
% MODEL D specific: ellipsoid-to-ellipsoid contact via trust-region QP.
% Sign is exact subject to numerical tolerance; magnitude is not distance.
report=wp_collision_common(cfg,shape,world,'ellipsoid',@contact_D);
end
function [g,method]=contact_D(item,part,near)
[lb,skip]=wp_pair_far_bound(item,part,near);
if skip,g=lb;method='circumsphere_lower_bound';return;end
if strcmp(item.kind,'sphere')
    [g,method]=wp_sphere_vs_part(item,part);
elseif strcmp(item.kind,'ellipsoid')
    if strcmp(part.kind,'sphere')
        axes=part.params.radius*[1;1;1]; R=eye(3);
    else
        axes=part.params.axes;R=part.R;
    end
    g=wp_ellipsoid_ellipsoid_margin(item.center,item.R,item.axes, ...
        part.center(:),R,axes);
    method='ellipsoid_ellipsoid_overlap_proxy';
else
    error('WPPreview:ModelDPart','Unexpected system part kind');
end
end

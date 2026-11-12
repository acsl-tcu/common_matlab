function report=wp_collision_C(cfg,shape,world)
% MODEL C specific: only ball-against-ball and ball-against-ellipsoid.
report=wp_collision_common(cfg,shape,world,'sphere',@contact_C);
end
function [g,method]=contact_C(item,part,near)
[lb,skip]=wp_pair_far_bound(item,part,near);
if skip,g=lb;method='circumsphere_lower_bound';return;end
[g,method]=wp_sphere_vs_part(item,part);
end

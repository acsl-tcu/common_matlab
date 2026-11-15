function [lo,hi]=obs_aabb_world(o,C)
% Certified world-axis AABB from certified component local AABBs.
lo=[Inf Inf Inf]; hi=[-Inf -Inf -Inf];
for i=1:numel(C)
 [a,b]=obs_component_bounds(C(i));V=obs_box_vertices(a,b);
 X=(o.R*V')'+o.position;
 lo=min(lo,min(X,[],1));hi=max(hi,max(X,[],1));
end
end

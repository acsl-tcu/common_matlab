function [parts,meta]=obs_approximate(o,method,options)
% Conservative solid approximation. This only builds obstacle geometry.
% It does not implement CBF or certify real-time control feasibility.
if nargin<3,options=struct();end
options=obs_approx_defaults(options);
C=obs_components(o,options);
switch char(method)
 case 'single_sphere',parts=obs_approx_single_sphere(o,C,options);
 case 'single_ellipsoid',parts=obs_approx_single_ellipsoid(o,C,options);
 case 'multiple_spheres',parts=obs_approx_multiple_spheres(o,C,options);
 case 'multiple_ellipsoids',parts=obs_approx_multiple_ellipsoids(o,C,options);
 case 'superellipsoid',parts=obs_approx_superellipsoid(o,C,options);
 case 'convex_polyhedra',parts=obs_approx_convex_polyhedra(o,C,options);
 otherwise,error('Unknown approximation method: %s',string(method));
end
for i=1:numel(parts)
 parts(i).obstacleName=o.name;parts(i).method=char(method);
end
meta=struct('obstacleName',o.name,'method',char(method), ...
 'numComponents',numel(C),'numParts',numel(parts), ...
 'guarantee','construction-based outward enclosure', ...
 'note','Mathematical component enclosure; floating-point implementation must be tested');
end

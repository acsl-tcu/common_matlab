function V = obs_ellipsoid_cover_vertices(c,options)
% Vertices of a guaranteed convex OUTER cover of one source component.
% Polyhedral source: exact supplied convex-hull vertices (no AABB).
% Smooth source: outer polyhedron defined by analytic supporting planes.
% This is NOT a box-to-ellipsoid construction. Its only conservatism is
% the support-plane discretization, which is tunable by polySides.
if strcmp(c.kind,'poly')
    V=double(c.vertices);
else
    V=double(obs_support_poly_vertices(c,options));
end
assert(size(V,2)==3 && size(V,1)>=4 && all(isfinite(V(:))), ...
    'No finite 3D outer-cover vertices for ellipsoid fitting');
V=unique(V,'rows');
assert(rank(V-V(1,:))==3, ...
    'Ellipsoid source outer cover must have nonzero 3D volume');
end

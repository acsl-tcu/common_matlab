function V=obs_prism_vertices(polygonXY,height)
% Extrude a convex planar polygon between z=+-height/2.
validateattributes(height,{'numeric'},{'scalar','real','positive'});
assert(size(polygonXY,2)==2 && size(polygonXY,1)>=3,'polygonXY must be Nx2');
N=size(polygonXY,1);
V=[polygonXY,-height/2*ones(N,1);polygonXY,height/2*ones(N,1)];
end

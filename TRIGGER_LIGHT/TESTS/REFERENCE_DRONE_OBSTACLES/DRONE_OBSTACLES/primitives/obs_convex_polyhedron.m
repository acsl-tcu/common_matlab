function o = obs_convex_polyhedron(name,position,vertices,rpyDeg)
% Obstacles are solids. All dimensions in meters; orientation in degrees.
if nargin < 4, rpyDeg=[0 0 0]; end
o = obs_make('convex_polyhedron',name,position,struct('vertices',vertices),rpyDeg);
end

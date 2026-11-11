function o = obs_cylinder(name,position,radius, height,rpyDeg)
% Obstacles are solids. All dimensions in meters; orientation in degrees.
if nargin < 5, rpyDeg=[0 0 0]; end
o = obs_make('cylinder',name,position,struct('radius',radius,'height',height),rpyDeg);
end

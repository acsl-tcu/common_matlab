function o = obs_sphere(name,position,radius,rpyDeg)
% Obstacles are solids. All dimensions in meters; orientation in degrees.
if nargin < 4, rpyDeg=[0 0 0]; end
o = obs_make('sphere',name,position,struct('radius',radius),rpyDeg);
end

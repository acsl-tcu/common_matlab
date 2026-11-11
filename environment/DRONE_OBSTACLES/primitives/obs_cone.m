function o = obs_cone(name,position,radius, height,rpyDeg)
% Obstacles are solids. All dimensions in meters; orientation in degrees.
if nargin < 5, rpyDeg=[0 0 0]; end
o = obs_make('cone',name,position,struct('radius',radius,'height',height),rpyDeg);
end

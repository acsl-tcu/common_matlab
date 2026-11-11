function o = obs_ellipsoid(name,position,axes,rpyDeg)
% Obstacles are solids. All dimensions in meters; orientation in degrees.
if nargin < 4, rpyDeg=[0 0 0]; end
o = obs_make('ellipsoid',name,position,struct('axes',axes),rpyDeg);
end

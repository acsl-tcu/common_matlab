function o = obs_hollow_cylinder(name,position,outerRadius, innerRadius, height,rpyDeg)
% Obstacles are solids. All dimensions in meters; orientation in degrees.
if nargin < 6, rpyDeg=[0 0 0]; end
o = obs_make('hollow_cylinder',name,position,struct('outerRadius',outerRadius,'innerRadius',innerRadius,'height',height),rpyDeg);
end

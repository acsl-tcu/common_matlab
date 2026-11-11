function o = obs_box(name,position,dimensions,rpyDeg)
% Obstacles are solids. All dimensions in meters; orientation in degrees.
if nargin < 4, rpyDeg=[0 0 0]; end
o = obs_make('box',name,position,struct('dimensions',dimensions),rpyDeg);
end

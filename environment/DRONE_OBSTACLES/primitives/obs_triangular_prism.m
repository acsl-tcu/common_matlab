function o = obs_triangular_prism(name,position,baseXY, height,rpyDeg)
% Obstacles are solids. All dimensions in meters; orientation in degrees.
if nargin < 5, rpyDeg=[0 0 0]; end
o = obs_make('triangular_prism',name,position,struct('baseXY',baseXY,'height',height),rpyDeg);
end

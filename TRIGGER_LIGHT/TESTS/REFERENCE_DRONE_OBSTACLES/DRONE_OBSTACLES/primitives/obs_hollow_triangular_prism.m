function o = obs_hollow_triangular_prism(name,position,outerTriangle, innerScale, height,rpyDeg)
% Obstacles are solids. All dimensions in meters; orientation in degrees.
if nargin < 6, rpyDeg=[0 0 0]; end
o = obs_make('hollow_triangular_prism',name,position,struct('outerTriangle',outerTriangle,'innerScale',innerScale,'height',height),rpyDeg);
end

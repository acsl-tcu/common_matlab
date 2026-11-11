function o = obs_hollow_square_prism(name,position,outerXY, holeXY, height,rpyDeg)
% Obstacles are solids. All dimensions in meters; orientation in degrees.
if nargin < 6, rpyDeg=[0 0 0]; end
o = obs_make('hollow_square_prism',name,position,struct('outerXY',outerXY,'holeXY',holeXY,'height',height),rpyDeg);
end

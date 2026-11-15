function o = obs_triangular_pyramid(name,position,baseXY, apexXYZ,rpyDeg)
% Obstacles are solids. All dimensions in meters; orientation in degrees.
if nargin < 5, rpyDeg=[0 0 0]; end
o = obs_make('triangular_pyramid',name,position,struct('baseXY',baseXY,'apexXYZ',apexXYZ),rpyDeg);
end

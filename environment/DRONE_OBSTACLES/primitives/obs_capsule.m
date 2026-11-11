function o = obs_capsule(name,position,radius, straightLength,rpyDeg)
% Obstacles are solids. All dimensions in meters; orientation in degrees.
if nargin < 5, rpyDeg=[0 0 0]; end
o = obs_make('capsule',name,position,struct('radius',radius,'straightLength',straightLength),rpyDeg);
end

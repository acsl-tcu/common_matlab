function motion = obs_motion_linear(velocityXYZ,angularRateRPYDeg,startTime)
% 現在の障害物配置を初期位置・姿勢として等速度運動。単位 m/s, deg/s。
if nargin<2 || isempty(angularRateRPYDeg),angularRateRPYDeg=[0 0 0];end
if nargin<3,startTime=0;end
validateattributes(velocityXYZ,{'numeric'},{'vector','numel',3,'finite'});
validateattributes(angularRateRPYDeg,{'numeric'},{'vector','numel',3,'finite'});
motion=struct('type','linear', 'velocity',reshape(velocityXYZ,1,3), ...
    'angularRate',reshape(angularRateRPYDeg,1,3),'startTime',startTime);
end

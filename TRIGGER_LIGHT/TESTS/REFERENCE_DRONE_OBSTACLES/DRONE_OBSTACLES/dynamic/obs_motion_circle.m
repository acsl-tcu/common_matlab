function motion = obs_motion_circle(centerXYZ,radiusXY,period,phaseDeg,yawRateDeg)
% world座標の中心まわりにXY楕円周回。中心、2軸半径、周期[s]、位相[deg]。
% 元の obs_* の position はこの軌道では使わず、姿勢の基準値として rpyDeg を使う。
if nargin<4,phaseDeg=0;end
if nargin<5,yawRateDeg=0;end
validateattributes(centerXYZ,{'numeric'},{'vector','numel',3,'finite'});
validateattributes(radiusXY,{'numeric'},{'vector','numel',2,'nonnegative','finite'});
validateattributes(period,{'numeric'},{'scalar','positive','finite'});
motion=struct('type','circle','center',reshape(centerXYZ,1,3), ...
    'radius',reshape(radiusXY,1,2),'period',period,'phaseDeg',phaseDeg, ...
    'yawRateDeg',yawRateDeg);
end

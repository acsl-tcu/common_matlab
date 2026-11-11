function [pos,R,rpy] = obs_pose_at_time(obstacle,t)
% 指定時刻 t [s] における障害物の中心・姿勢。
% R=Rz(yaw)*Ry(pitch)*Rx(roll), 角度はすべてdegree。
validateattributes(t,{'numeric'},{'scalar','real','finite'});
pos=reshape(obstacle.position,1,3);
rpy=reshape(obstacle.rpyDeg,1,3);
if ~isfield(obstacle,'isDynamic') || ~obstacle.isDynamic
 R=obs_rpy_matrix(rpy);return;
end
m=obstacle.motion;
switch char(m.type)
 case 'linear'
  tau=max(0,t-m.startTime);
  pos=pos+tau*m.velocity;
  rpy=rpy+tau*m.angularRate;
 case 'circle'
  theta=(m.phaseDeg*pi/180)+(2*pi/m.period)*t;
  pos=m.center+[m.radius(1)*cos(theta),m.radius(2)*sin(theta),0];
  rpy=rpy+[0 0 m.yawRateDeg*t];
 case 'waypoints'
  tau=t;
  if m.loop
   tau=mod(t,m.times(end));
  else
   tau=max(m.times(1),min(m.times(end),t));
  end
  pos=reshape(interp1(m.times,m.positions,tau,'pchip'),1,3);
  rpy=reshape(interp1(m.times,m.rpy,tau,'pchip'),1,3);
 case 'formula'
  pos=reshape(m.positionFunction(t),1,[]);
  rpy=reshape(m.rpyFunction(t),1,[]);
 case 'static'
  % Keep original pose.
 otherwise
  error('Unknown motion type: %s',m.type);
end
assert(numel(pos)==3 && numel(rpy)==3 && all(isfinite([pos,rpy])), ...
    'Obstacle %s: motion must return finite position/RPY 1x3',obstacle.name);
R=obs_rpy_matrix(rpy);
end

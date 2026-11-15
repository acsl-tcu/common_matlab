function [lowerBound,skip]=wp_pair_far_bound(item,part,nearDistance)
% Conservative support-ball separation test. Safe if > nearDistance >=0.
if strcmp(part.kind,'sphere')
   rObs=part.params.radius;
elseif strcmp(part.kind,'ellipsoid')
   rObs=max(part.params.axes);
else
   error('WPPreview:ObstacleType','Expected sphere/ellipsoid obstacle part');
end
switch item.kind
  case 'sphere',rSys=item.radius;
  case 'capsule',rSys=norm(item.p1-item.p0)/2+item.radius;
  case 'ellipsoid',rSys=max(item.axes);
  otherwise,error('WPPreview:SystemPartType','Invalid system part type');
end
lowerBound=norm(item.center-part.center(:))-(rSys+rObs);
skip=lowerBound>nearDistance;
end

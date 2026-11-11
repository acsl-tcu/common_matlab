function inside=obs_component_contains(c,q,tol)
% Exact interior test (poly = convex hull of supplied vertices).
if nargin<3,tol=1e-9;end
switch c.kind
 case 'poly'
  [H,b]=obs_poly_halfspaces(c.vertices);inside=all(q*H'<=b'+tol,2);
 case 'sphere'
  inside=sum(q.^2,2)<=(c.params.radius+tol)^2;
 case 'ellipsoid'
  inside=sum((q./c.params.axes).^2,2)<=1+tol;
 case 'cylinder'
  inside=sum(q(:,1:2).^2,2)<=(c.params.radius+tol)^2 & ...
    abs(q(:,3))<=c.params.height/2+tol;
 case 'capsule'
  z=max(-c.params.straightLength/2,min(c.params.straightLength/2,q(:,3)));
  inside=sum(q(:,1:2).^2,2)+(q(:,3)-z).^2<=(c.params.radius+tol)^2;
 case 'cone'
  z=q(:,3);r=c.params.radius*(c.params.height/2-z)/c.params.height;
  inside=abs(z)<=c.params.height/2+tol & sum(q(:,1:2).^2,2)<=(max(r,0)+tol).^2;
 otherwise,error('invalid component kind');
end
end

function inside=obs_part_contains(part,pts,tol)
% Test points Nx3 against one conservative approximation part.
if nargin<3,tol=1e-9;end
assert(size(pts,2)==3,'pts must be Nx3');
q=(part.R'*(pts-part.center)')';
switch part.kind
 case 'sphere'
  inside=sum(q.^2,2)<=(part.params.radius+tol)^2;
 case 'ellipsoid'
  a=part.params.axes; inside=sum((q./a).^2,2)<=1+tol;
 case 'superellipsoid'
  a=part.params.axes; exponent=part.params.exponent;
  inside=sum(abs(q./a).^exponent,2)<=1+tol;
 case 'convex_polyhedron'
  inside=all(pts*part.H'<=part.b'+tol,2);
 otherwise,error('Unknown approximation kind: %s',part.kind);
end
end

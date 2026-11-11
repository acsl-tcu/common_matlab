function C=obs_components(o,opts)
% Exact component solids, or conservative component solids for hollow cylinder.
% Each component: kind='poly'|'sphere'|'ellipsoid'|'cylinder'|'capsule'|'cone';
% shape given in obstacle-local coordinates; vertices are N-by-3.
if nargin<2, opts=struct(); end
C=struct('kind',{},'vertices',{},'params',{});
p=o.params; t=o.type;
switch t
 case 'sphere'
  validateattributes(p.radius,{'numeric'},{'scalar','positive'});
  C=push(C,'sphere',[],struct('radius',p.radius));
 case 'ellipsoid'
  assert(numel(p.axes)==3 && all(p.axes(:)>0),'axes must be positive [a b c]');
  C=push(C,'ellipsoid',[],struct('axes',reshape(p.axes,1,3)));
 case 'box'
  assert(numel(p.dimensions)==3 && all(p.dimensions(:)>0),'box dimensions [x y z] must be positive');
  h=p.dimensions(:)'/2; C=push(C,'poly',obs_box_vertices(-h,h),struct());
 case 'cylinder'
  checkpos(p.radius,p.height); C=push(C,'cylinder',[],p);
 case 'capsule'
  validateattributes(p.radius,{'numeric'},{'scalar','positive'});
  validateattributes(p.straightLength,{'numeric'},{'scalar','nonnegative'});
  C=push(C,'capsule',[],p);
 case 'convex_polyhedron'
  V=p.vertices; assert(size(V,2)==3 && size(V,1)>=4,'polyhedron vertices must be Nx3');
  assert(rank(V-V(1,:))==3,'polyhedron must have 3D volume');
  C=push(C,'poly',V,struct()); % convex hull, not arbitrary nonconvex mesh
 case 'cone'
  checkpos(p.radius,p.height); C=push(C,'cone',[],p);
 case 'triangular_pyramid'
  assert(isequal(size(p.baseXY),[3 2]) && numel(p.apexXYZ)==3,'base 3x2 and apex 1x3');
  % Base triangle lies in local z=0; apex is user-defined in local coordinates.
  V=[p.baseXY,zeros(3,1);reshape(p.apexXYZ,1,3)];
  assert(rank(V-V(1,:))==3,'pyramid volume must be nonzero');
  C=push(C,'poly',V,struct());
 case 'triangular_prism'
  assert(isequal(size(p.baseXY),[3 2]),'baseXY must be 3x2');
  C=push(C,'poly',obs_prism_vertices(p.baseXY,p.height),struct());
 case 'hollow_square_prism'
  A=reshape(p.outerXY,1,2); B=reshape(p.holeXY,1,2); h=p.height;
  assert(all(A>B) && all(B>0) && h>0,'0<holeXY<outerXY and height>0');
  ax=A(1)/2; ay=A(2)/2; bx=B(1)/2; by=B(2)/2;
  polys={[-ax,-ay;-bx,-ay;-bx,ay;-ax,ay], ...
         [ bx,-ay; ax,-ay; ax,ay; bx,ay], ...
         [-bx,-ay; bx,-ay; bx,-by;-bx,-by], ...
         [-bx, by; bx, by; bx, ay;-bx, ay]};
  for i=1:4, C=push(C,'poly',obs_prism_vertices(polys{i},h),struct()); end
 case 'hollow_cylinder'
  ro=p.outerRadius; ri=p.innerRadius; H=p.height;
  assert(ro>ri && ri>0 && H>0,'0<innerRadius<outerRadius, height>0');
  N=opt(opts,'hollowSides',24); assert(N>=8 && N==round(N),'hollowSides >= 8 required');
  for i=1:N
   t0=2*pi*(i-1)/N; t1=2*pi*i/N;
   % Conservative convex sector: inner chord r_i and tangent outer edge R_o.
   inner=[ri*cos(t0),ri*sin(t0);ri*cos(t1),ri*sin(t1)];
   outerR=ro/cos(pi/N);
   outer=[outerR*cos(t0),outerR*sin(t0);outerR*cos(t1),outerR*sin(t1)];
   poly=[inner(1,:); outer(1,:); outer(2,:); inner(2,:)];
   C=push(C,'poly',obs_prism_vertices(poly,H),struct());
  end
 case 'hollow_triangular_prism'
  O=p.outerTriangle; s=p.innerScale; H=p.height;
  assert(isequal(size(O),[3 2]) && s>0 && s<1 && H>0,'outerTriangle 3x2, 0<innerScale<1');
  G=mean(O,1); I=G+s*(O-G);
  assert(abs(det([O(2,:)-O(1,:);O(3,:)-O(1,:)]))>1e-8,'outer triangle degenerate');
  for i=1:3
   j=mod(i,3)+1; poly=[O(i,:);O(j,:);I(j,:);I(i,:)];
   C=push(C,'poly',obs_prism_vertices(poly,H),struct());
  end
 case 'triangular_prism_mesh'
  V=p.prismVertices;
  assert(ndims(V)<=3 && size(V,1)==6 && size(V,2)==3,'prismVertices must be 6x3xN');
  for i=1:size(V,3)
   Vi=V(:,:,i); assert(rank(Vi-Vi(1,:))==3,'mesh prism has zero volume');
   C=push(C,'poly',Vi,struct());
  end
 otherwise, error('Unknown obstacle type: %s',t);
end
end
function C=push(C,kind,V,p)
C(end+1)=struct('kind',kind,'vertices',V,'params',p);
end
function checkpos(a,b)
assert(isscalar(a)&&isscalar(b)&&a>0&&b>0,'radius and height must be >0');
end
function v=opt(o,f,d)
if isfield(o,f),v=o.(f);else,v=d;end
end

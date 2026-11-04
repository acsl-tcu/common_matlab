function B=TRIGGER_SHAPE_SOURCE_OUTER_BALL(S)
% Build a SOURCE-DERIVED enclosing ball for an already observed static solid.
% Do not trust a self-asserted rigorous_outer_ball_certified boolean.
% Supported geometries are read from the actual prepared source fields.
% For arbitrary R, ||R||2 <= sqrt(1+||R'R-I||infinity), so the bound
% does not assume exact orthonormality of stored binary64 coefficients.
% The radius arithmetic is widened outwards at each positive operation.
% This closes one GEOMETRY provenance link, NOT EKF/plant/Flight safety.
B=struct('status','UNKNOWN_SHAPE_SOURCE_OUTER','radius_upper_m',NaN, ...
 'center_m',nan(3,1),'source_geometry_enclosed',false, ...
 'rotation_norm_upper',NaN,'reason','', 'safe_to_commit',false);
try
 assert(isstruct(S)&&isscalar(S)&&all(isfield(S, ...
  {'type','rotation','dimensions','center0','is_stationary'})) && ...
  isequal(S.is_stationary,true), ...
  'TRIGGER_SHAPE_SOURCE_OUTER_BALL:Shape','Static source-prepared shape mandatory.');
 R=double(S.rotation);c=double(S.center0(:));
 assert(isequal(size(R),[3 3])&&all(isfinite(R(:))) && ...
  isequal(size(c),[3 1])&&all(isfinite(c)), ...
  'TRIGGER_SHAPE_SOURCE_OUTER_BALL:Rotation','Finite 3D shape pose required.');
 % Upper infinity norm of R'*R-I with outward interval products/sums.
 row_upper=zeros(3,1);
 for i=1:3
  for j=1:3
   a=TRIGGER_OUTWARD_INTERVAL(0,0);
   for k=1:3
    a=a+TRIGGER_OUTWARD_INTERVAL(R(k,i),R(k,i)).* ...
      TRIGGER_OUTWARD_INTERVAL(R(k,j),R(k,j));
   end
   if i==j,a=a-TRIGGER_OUTWARD_INTERVAL(1,1);end
   e=max(abs(a.lo),abs(a.hi));
   row_upper(i)=upper_add(row_upper(i),e);
  end
 end
 m=upper_sqrt(upper_add(1,max(row_upper)));
 B.rotation_norm_upper=m;
 d=S.dimensions;
 if isnumeric(d)
  d=double(d(:));
  assert(all(isfinite(d))&&all(d>=0), ...
     'TRIGGER_SHAPE_SOURCE_OUTER_BALL:Dimensions','Nonnegative dimensions required.');
 end
 switch char(S.type)
  case 'sphere'
   assert(isnumeric(d)&&numel(d)>=1);local=d(1);
  case 'ellipsoid'
   assert(isnumeric(d)&&numel(d)>=3);local=max(d(1:3));
  case 'box'
   assert(isnumeric(d)&&numel(d)>=3);local=norm_upper(d(1:3)/2);
  case {'cylinder','cone'}
   assert(isnumeric(d)&&numel(d)>=2);
   local=norm_upper([d(1);d(2)/2]);
  case 'capsule'
   assert(isnumeric(d)&&numel(d)>=2);
   local=upper_add(d(1),d(2)/2);
  case 'frustum'
   assert(isnumeric(d)&&numel(d)>=3);
   local=norm_upper([max(d(1:2));d(3)/2]);
  case {'polyhedron','pyramid'}
   assert(isfield(S,'vertices'));
   v=double(S.vertices);
   assert(size(v,2)==3 && ~isempty(v) && all(isfinite(v(:))));
   local=0;
   for k=1:size(v,1),local=max(local,norm_upper(v(k,:).'));end
  case {'polygon_prism','regular_prism'}
   assert(isfield(S,'outline'));
   v=double(S.outline);
   assert(size(v,2)==2 && ~isempty(v) && all(isfinite(v(:))));
   planar=0;
   for k=1:size(v,1),planar=max(planar,norm_upper(v(k,:).'));end
   if isstruct(d)
     assert(isfield(d,'height') && isscalar(d.height) && ...
        isnumeric(d.height) && isfinite(d.height) && d.height>=0);
     half_height=double(d.height)/2;
   else
     assert(numel(d)>=3);half_height=d(3)/2;
   end
   local=norm_upper([planar;half_height]);
  case 'torus'
   assert(isnumeric(d)&&numel(d)>=2);
   local=upper_add(d(1),d(2));
  otherwise
   B.status='UNKNOWN_UNSUPPORTED_SHAPE_SOURCE';
   B.reason='A new shape must supply a proved local occupancy bound; no unproved enclosing radius is trusted.';
   return
 end
 radius=upper_mul(local,m);
 assert(isfinite(radius)&&radius>=0);
 B.center_m=c;B.radius_upper_m=radius;
 B.source_geometry_enclosed=true;
 B.status='SOURCE_DERIVED_STATIC_SHAPE_OUTER__OTHER_PHYSICAL_PROOFS_REQUIRED';
 B.reason=['Analytic local occupancy bound combined with a certified ' ...
    'upper matrix norm of the binary64 stored rotation. No caller ' ...
    'self-declared safety boolean was accepted.'];
catch ME
 B.reason=[ME.identifier ' | ' ME.message];
end
end
function r=norm_upper(a)
 r=0;
 for j=1:numel(a),r=upper_add(r,upper_mul(abs(a(j)),abs(a(j))));end
 r=upper_sqrt(r);
end
function c=upper_add(a,b)
 assert(a>=0&&b>=0&&isfinite(a)&&isfinite(b));
 c=TRIGGER_FLOAT_OUTWARD_ONE_ULP(double(a)+double(b),+1);
 assert(isfinite(c));
end
function c=upper_mul(a,b)
 assert(a>=0&&b>=0&&isfinite(a)&&isfinite(b));
 c=TRIGGER_FLOAT_OUTWARD_ONE_ULP(double(a)*double(b),+1);
 assert(isfinite(c));
end
function c=upper_sqrt(a)
 assert(a>=0&&isfinite(a));
 c=TRIGGER_FLOAT_OUTWARD_ONE_ULP(sqrt(double(a)),+1);
 assert(isfinite(c));
end

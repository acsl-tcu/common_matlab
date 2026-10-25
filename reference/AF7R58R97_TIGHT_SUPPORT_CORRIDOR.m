function D=AF7R58R97_TIGHT_SUPPORT_CORRIDOR(W,obs,clearance)
% Geometry-only candidate corridors. Segment/ellipsoid nearest-point normal
% supplies a support separating plane, if such a strict plane is found.
% Numerical test is not outward-rounded and is NOT final collision proof.
assert(isnumeric(W)&&size(W,1)==3&&size(W,2)>=2&&all(isfinite(W(:))), ...
 'R97:BadWaypoints','Expected finite 3xK waypoints.');
assert(isstruct(obs)&&isscalar(clearance)&&isnumeric(clearance)&& ...
 isfinite(clearance)&&clearance>=0,'R97:BadClearance', ...
 'Need nonnegative runtime clearance.');
S=size(W,2)-1;corr=cell(1,S);
D=struct('status','TIGHT_SUPPORT_CORRIDORS_FOUND__NOT_CERTIFIED', ...
 'corridors',{corr},'feasible_corridor_family',true, ...
 'first_bad_segment',NaN,'first_bad_obstacle',NaN, ...
 'min_endpoint_margin_m',Inf,'known_obstacles_checked',numel(obs), ...
 'whole_body_certified',false,'production_commit_authorized',false, ...
 'closest_distance_m',Inf,'tight_support_used',true);
for s=1:S
 a=W(:,s);b=W(:,s+1);v=b-a;
 if norm(v)==0
    D.status='COINCIDENT_KNOTS_LOCAL_CHART_REQUIRED';
    D.feasible_corridor_family=false;D.first_bad_segment=s;return;
 end
 N=zeros(0,3);h=zeros(0,1);
 for j=1:numel(obs)
  o=obs(j);
  assert(isfield(o,'center')&&isfield(o,'axes')&&isfield(o,'R')&& ...
    isequal(size(o.center),[3 1])&&isequal(size(o.axes),[3 1])&& ...
    isequal(size(o.R),[3 3])&&all(o.axes>0)&& ...
    all(isfinite([o.center;o.axes;o.R(:)]))&& ...
    norm(o.R'*o.R-eye(3),'fro')<1e-9&&abs(det(o.R)-1)<1e-9, ...
    'R97:InvalidOrientedEllipsoid','Bad oriented obstacle.');
  c=double(o.center);R=double(o.R);r=double(o.axes);
  tau=max(0,min(1,dot(c-a,v)/dot(v,v)));
  sf=norm(a+tau*v-c)-max(r)-clearance;
  guard=1024*eps(max([1;abs(a);abs(b);abs(c);r;clearance]));
  if sf>guard,continue;end % mathematically safe broadphase only
  [tau,x,dist]=chord_nearest(a,b,c,R,r);
  D.closest_distance_m=min(D.closest_distance_m,dist);
  if ~(isfinite(dist)&&dist>0)
     D.status='CHORD_INTERSECTS_ELLIPSOID';
     D.feasible_corridor_family=false;D.first_bad_segment=s;
     D.first_bad_obstacle=j;D.corridors=corr;return;
  end
  n=(a+tau*v-x)/dist;
  sh=dot(n,c)+norm(r.*(R'*n))+clearance;
  m=min(dot(n,a),dot(n,b))-sh;
  D.min_endpoint_margin_m=min(D.min_endpoint_margin_m,m);
  if ~(isfinite(m)&&m>guard)
    D.status='TIGHT_SUPPORT_NOT_STRICT__RESEED';
    D.feasible_corridor_family=false;D.first_bad_segment=s;
    D.first_bad_obstacle=j;D.corridors=corr;return;
  end
  N(end+1,:)=-n';h(end+1,1)=-sh; %#ok<AGROW>
 end
 corr{s}=struct('A',N,'b',h,'source','CHORD_CLOSEST_SUPPORT');
end
D.corridors=corr;
end

function [t,x,d]=chord_nearest(a,b,c,R,r)
v=b-a;
xa=project(a,c,R,r);xb=project(b,c,R,r);
g0=dot(a-xa,v);g1=dot(b-xb,v);
if g0>=0,t=0;x=xa;
elseif g1<=0,t=1;x=xb;
else
 lo=0;hi=1;
 for it=1:68
    mid=lo+(hi-lo)/2;
    if mid==lo||mid==hi,break;end
    xm=project(a+mid*v,c,R,r);
    if dot(a+mid*v-xm,v)>0,hi=mid;else,lo=mid;end
 end
 t=lo+(hi-lo)/2;x=project(a+t*v,c,R,r);
end
d=norm(a+t*v-x);
end

function x=project(p,c,R,r)
y=R'*(p-c);
if norm(y./r)<=1,x=p;return;end
lo=0;hi=max(r)^2;
if ~isfinite(hi)||hi<=0,error('R97:UnrepresentableSemiaxes');end
for k=1:256
 if sum((r.*y./(r.^2+hi)).^2)<=1,break;end
 hi=2*hi;if ~isfinite(hi),error('R97:UnrepresentableProjection');end
end
for k=1:72
 mid=lo+(hi-lo)/2;if mid==lo||mid==hi,break;end
 if sum((r.*y./(r.^2+mid)).^2)>1,lo=mid;else,hi=mid;end
end
x=c+R*(r.^2.*y./(r.^2+hi));
end

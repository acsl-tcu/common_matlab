function C=AF7R58R99_MULTI_SIZE_KNOT_CORRIDOR(route,obs,live)
% R99 multi-size 4-vertex knot prism initialization, ACETRANS-inspired.
% For each interval the vertices are [payload_i, UAV_i, payload_i+1,
% UAV_i+1] from FLAT JETS, *not* just payload chords. Each obstacle
% separating halfspace uses its EXACT oriented-ellipsoid support function.
% The radius of each end-body is rL/rQ, with cable radius rC applied at
% both endpoints. Convexity then covers the straight cable AT THE KNOTS.
% Adjacent corridors automatically share a strict interior pair of UAV/
% payload knot points. A finite normal library is a NON-COMPLETE seed
% search; failure is INCONCLUSIVE, never proof no collision-free path exists.
% Full trajectory, tracking tube, IEEE safety remain UNPROVEN.
assert(isstruct(route)&&isscalar(route)&&isfield(route,'knot_jets')&& ...
 size(route.knot_jets,1)==3&&size(route.knot_jets,2)>=3 && ...
 size(route.knot_jets,3)==route.num_segments+1 && ...
 isstruct(obs)&&isstruct(live)&&isscalar(live), ...
 'R99:RequiredLiveJetsAndObstacles','Need all shared route jets and live physics.');
needed={'gravity_m_s2','cable_length_m','rQ_m','rL_m','rC_m'};
for i=1:numel(needed)
 assert(isfield(live,needed{i})&&isnumeric(live.(needed{i}))&& ...
  isscalar(live.(needed{i}))&&isfinite(live.(needed{i})), ...
  'R99:MissingExternalBodyAuthority','Missing %s.',needed{i});
end
S=route.num_segments;K=S+1;L=live.cable_length_m;g=live.gravity_m_s2;
assert(L>0&&g>0&&all([live.rL_m live.rQ_m live.rC_m]>=0), ...
 'R99:InvalidPhysics','Live length/gravity/radii invalid.');
margin=0;if isfield(live,'d_margin_m'),margin=live.d_margin_m;end
assert(isnumeric(margin)&&isscalar(margin)&&isfinite(margin)&&margin>=0, ...
 'R99:BadLiveMargin','Margin must come from execution authority.');
pL=zeros(3,K);pQ=zeros(3,K);
for k=1:K
 pL(:,k)=route.knot_jets(:,1,k);
 f=route.knot_jets(:,3,k)+[0;0;g];
 if ~all(isfinite(f))||norm(f)<=0
   C=failed('INCONCLUSIVE_KNOT_SPECIFIC_FORCE_SINGULAR',S);
   C.first_bad_knot=k;return;
 end
 pQ(:,k)=pL(:,k)+L*f/norm(f);
end
rL=max(live.rL_m,live.rC_m);rQ=max(live.rQ_m,live.rC_m);
rho=[rL rQ rL rQ];
C=failed('MULTISIZE_KNOT_CORRIDOR_UNRESOLVED__NOT_NO_PATH',S);
C.minimum_support_margin_m=Inf;
if isempty(obs)
 C.status='INCONCLUSIVE_NO_KNOWN_OBSTACLE_SET';return;
end
for s=1:S
 v=[pL(:,s) pQ(:,s) pL(:,s+1) pQ(:,s+1)];
 A=zeros(0,3);b=zeros(0,1);
 for j=1:numel(obs)
  o=obs(j);c=double(o.center);R=double(o.R);axes=double(o.axes);
  assert(isequal(size(c),[3 1])&&isequal(size(R),[3 3])&& ...
   isequal(size(axes),[3 1])&&all(axes>0)&& ...
   all(isfinite([c;R(:);axes])), ...
   'R99:InvalidKnownEllipsoid','Runtime oriented ellipsoid is invalid.');
  centroid=mean(v,2);d=[v-c centroid-c (v(:,1)+v(:,2))/2-c ...
   (v(:,3)+v(:,4))/2-c (v(:,1)+v(:,3))/2-c ...
   (v(:,2)+v(:,4))/2-c R -R];
  best=-Inf;bestN=[];bestB=NaN;
  for i=1:size(d,2)
   ni=d(:,i);zn=norm(ni);
   if zn<=0||~isfinite(zn),continue;end
   ni=ni/zn;
   support=dot(ni,c)+norm(axes.*(R'*ni))+margin;
   slack=ni'*v-rho-support;
   m=min(slack);
   if m>best,best=m;bestN=ni;bestB=support;end
  end
  scale=max([1;abs(v(:));abs(c);axes;rho(:);margin]);
  if ~(isfinite(best)&&best>512*eps(scale))
   C.first_bad_segment=s;C.first_bad_obstacle=j;
   C.status='INCONCLUSIVE_MULTISIZE_SUPPORT_DIRECTIONS';return;
  end
  C.minimum_support_margin_m=min(C.minimum_support_margin_m,best);
  A(end+1,:)=-bestN';b(end+1,1)=-bestB; %#ok<AGROW>
 end
 C.cells{s}=struct('A',A,'b',b,'pL_start',pL(:,s), ...
  'pQ_start',pQ(:,s),'pL_end',pL(:,s+1),'pQ_end',pQ(:,s+1));
end
% Share physical knot states in the two adjacent convex cells, with each
% vertex/body's own inflated clearance. Thus a strict overlap exists for
% the reference *knot configuration*, not for entire moving rigid bodies.
for s=1:S-1
 c1=C.cells{s};c2=C.cells{s+1};
 for st=1:2
  if st==1,p=pL(:,s+1);r=rL;
  else,p=pQ(:,s+1);r=rQ;end
  A=[c1.A;c2.A];b=[c1.b;c2.b];
  slack=b-A*p-r;
  scale=max([1;abs(A(:));abs(b(:));abs(p);r]);
  if any(slack<=512*eps(scale))
   C.first_bad_segment=s;C.status='INCONCLUSIVE_MULTISIZE_KNOT_OVERLAP';return;
  end
 end
 C.adjacent_overlap_count=C.adjacent_overlap_count+1;
end
C.whole_body_knot_corridor_found=true;
C.status='MULTISIZE_KNOT_CELL_CHAIN_FOUND__NOT_CONTINUOUS_WHOLE_BODY_PROOF';
end
function C=failed(status,S)
C=struct('status',status,'whole_body_knot_corridor_found',false, ...
 'cells',{cell(1,S)},'adjacent_overlap_count',0, ...
 'minimum_support_margin_m',NaN,'first_bad_segment',NaN, ...
 'first_bad_obstacle',NaN,'first_bad_knot',NaN, ...
 'whole_trajectory_certified',false,'production_commit_authorized',false);
end

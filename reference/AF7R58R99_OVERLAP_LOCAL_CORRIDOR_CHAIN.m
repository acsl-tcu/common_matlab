function C=AF7R58R99_OVERLAP_LOCAL_CORRIDOR_CHAIN(route,obs,clearance,opts)
% R99: a CONNECTED SEQUENCE of local convex support corridors, not a
% single hull of an entire degree-15 polynomial segment. The trajectory
% itself is unchanged: de Casteljau subdivision is an exact real-arithmetic
% restriction of each polynomial. For each local leaf, ALL payload Bezier
% control points are strictly enclosed by a separating convex corridor
% for every currently known oriented ellipsoid. Adjacent cells must share
% a strictly interior transition witness, making the union connected.
% This is PAYLOAD-ONLY, numeric, not an IEEE proof. Independent R94
% UAV/CABLE/PAYLOAD continuous-time, yaw and dynamics remain mandatory.
assert(isstruct(route)&&isscalar(route)&&isfield(route,'segments')&& ...
 isfield(route,'num_segments')&&route.num_segments==numel(route.segments) && ...
 isstruct(obs)&&isnumeric(clearance)&&isscalar(clearance)&& ...
 isfinite(clearance)&&clearance>=0 && isstruct(opts)&&isscalar(opts)&& ...
 isfield(opts,'max_nodes_per_check'), ...
 'R99:ExplicitKnownObstaclesAndWorkRequired', ...
 'Route, known ellipsoids, clearance and CPU work must be supplied.');
if isfield(opts,'max_corridor_leaf_nodes'),budget=opts.max_corridor_leaf_nodes;
else,budget=opts.max_nodes_per_check;end
assert(isnumeric(budget)&&isscalar(budget)&&isfinite(budget)&& ...
 budget>=1&&fix(budget)==budget, 'R99:PositiveIntegerWorkRequired', ...
 'max_corridor_leaf_nodes must be a positive integer CPU budget.');
C=struct('status','NOT_EVALUATED','payload_corridor_certified',false, ...
 'nodes',0,'leaves',0,'overlap_count',0,'minimum_overlap_margin_m',Inf, ...
 'first_bad_segment',NaN,'first_bad_local_interval',[], ...
 'max_cp_plane_residual',-Inf,'corridor_chain',struct([]), ...
 'whole_body_certified',false,'ieee_directed_rounding_proved',false, ...
 'torque_tube_tail_certified',false,'production_commit_authorized',false);
if isempty(obs)
 C.status='INCONCLUSIVE_NO_KNOWN_OBSTACLES_IN_SOURCE';return;
end
leafTemplate=struct('segment',0,'u_left',0,'u_right',1,'start_p',zeros(3,1), ...
 'end_p',zeros(3,1),'A',zeros(0,3),'b',zeros(0,1), ...
 'min_control_point_margin_m',Inf);
chain=repmat(leafTemplate,1,0);
for s=1:route.num_segments
 P=double(route.segments(s).payload_cp);
 assert(isequal(size(P),[3 16])&&all(isfinite(P(:))), ...
 'R99:MalformedDegree15Segment','Every payload segment must be 3x16 finite.');
 todo={struct('P',P,'a',0,'b',1)};
 while ~isempty(todo)
  if C.nodes>=budget
   C.status='INCONCLUSIVE_LOCAL_CORRIDOR_WORK_BUDGET';C.first_bad_segment=s;
   C.corridor_chain=chain;return;
  end
  X=todo{end};todo(end)=[];C.nodes=C.nodes+1;
  G=AF7R58R97_TIGHT_SUPPORT_CORRIDOR(X.P(:,[1 end]),obs,clearance);
  accepted=false;smallestMargin=Inf;
  if G.feasible_corridor_family
   cr=G.corridors{1};
   if size(cr.A,2)==3 && size(cr.A,1)==numel(cr.b)
    if isempty(cr.A)
      accepted=true;
    else
      cpScale=max([1;abs(X.P(:));abs(cr.A(:));abs(cr.b(:));clearance]);
      numGuard=512*eps(cpScale);
      residual=cr.A*X.P-cr.b(:)*ones(1,16);
      C.max_cp_plane_residual=max(C.max_cp_plane_residual,max(residual(:)));
      smallestMargin=-max(residual(:));
      accepted=all(residual(:)<-numGuard);
    end
   end
  end
  if accepted
   leaf=leafTemplate;leaf.segment=s;leaf.u_left=X.a;leaf.u_right=X.b;
   leaf.start_p=X.P(:,1);leaf.end_p=X.P(:,end);
   leaf.A=cr.A;leaf.b=cr.b;leaf.min_control_point_margin_m=smallestMargin;
   chain(end+1)=leaf; %#ok<AGROW>
   C.leaves=C.leaves+1;
   continue;
  end
  mid=(X.a+X.b)/2;
  if ~(mid>X.a && mid<X.b)
   C.status='INCONCLUSIVE_NORMALIZED_SUBDIVISION_RESOLUTION';
   C.first_bad_segment=s;C.first_bad_local_interval=[X.a X.b];
   C.corridor_chain=chain;return;
  end
  [left,right]=split_midpoint(X.P);
  todo{end+1}=struct('P',right,'a',mid,'b',X.b); %#ok<AGROW>
  todo{end+1}=struct('P',left,'a',X.a,'b',mid); %#ok<AGROW>
 end
end
% The leaf corridors must be connected *in order* across all polynomial
% boundaries. A strict point in the intersection proves local nonempty
% interior (real arithmetic). A vanishing margin, never treated as safe.
for i=1:numel(chain)-1
 left=chain(i);right=chain(i+1);
 p=(left.end_p+right.start_p)/2;
 A=[left.A;right.A];b=[left.b;right.b];
 if isempty(A)
  margin=Inf;
 else
  rr=A*p-b;
  margin=-max(rr);
  scale=max([1;abs(p);abs(A(:));abs(b(:))]);
  if ~(all(isfinite(rr)) && all(rr < -512*eps(scale)))
    C.status='INCONCLUSIVE_ADJACENT_CORRIDOR_OVERLAP';
    C.corridor_chain=chain;C.first_bad_segment=right.segment;return;
  end
 end
 C.minimum_overlap_margin_m=min(C.minimum_overlap_margin_m,margin);
 C.overlap_count=C.overlap_count+1;
end
C.payload_corridor_certified=true;
C.corridor_chain=chain;
C.status='NUMERIC_OVERLAPPING_PAYLOAD_CORRIDOR_CHAIN__NOT_WHOLE_BODY_OR_IEEE';
end
function [left,right]=split_midpoint(P)
 n=size(P,2);D=P;left=zeros(size(P));right=zeros(size(P));
 left(:,1)=P(:,1);right(:,end)=P(:,end);
 for k=1:n-1
  D=(D(:,1:end-1)+D(:,2:end))/2;
  left(:,k+1)=D(:,1);right(:,n-k)=D(:,end);
 end
end

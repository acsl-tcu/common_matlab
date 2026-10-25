function [R,C]=AF7R58R96_C7_CORRIDOR_PROJECT(R,corridors,opts)
% Enforce A_s * every degree-15 Bezier control point <= b_s directly in
% GENERATION by adjusting ONLY shared internal jets (orders 1..7).
% Exact external 0..7 jets, all waypoint positions and C7 knot continuity
% remain fixed. Projected halfspaces are a SUFFICIENT payload-corridor
% constraint; separate certified UAV/cable/payload safety is still required.
% Hildreth dual coordinate projection; finite sweeps are COMPUTATION work,
% never a geometric obstacle-size or maneuver-time limit.
assert(isstruct(R)&&isscalar(R)&&isfield(R,'reference_candidate_found'), ...
    'R96:RouteStructRequired','A generated C7 route is required.');
assert(iscell(corridors)&&isstruct(opts)&&isscalar(opts), ...
    'R96:CorridorAndBudgetRequired','Expected corridor cell array and work options.');
C=struct('status','NOT_EVALUATED','control_point_contained',false, ...
 'projection_sweeps',0,'halfspace_count',0,'initial_max_violation_m',NaN, ...
 'final_max_violation_m',NaN,'adjusted_internal_jets',false, ...
 'endpoint_0_to_7_exact_input_preserved',false, ...
 'numeric_c7_0_to_7_max_residual',NaN,'production_commit_authorized',false);
if ~R.reference_candidate_found
 C.status='SOURCE_ROUTE_NOT_GENERATED';return;
end
S=R.num_segments;K=S+1;
assert(numel(corridors)==S&&isfield(R,'knot_jets')&& ...
 isequal(size(R.knot_jets),[3 8 K])&& ...
 all(diff(R.times_s)>0), ...
 'R96:MalformedSharedC7Knots','Segments, corridor count or jets inconsistent.');
if isfield(opts,'max_corridor_projection_sweeps')
 maxSweeps=opts.max_corridor_projection_sweeps;
else
 maxSweeps=128; % pure algorithmic iteration budget, not a path/time limit
end
assert(isnumeric(maxSweeps)&&isscalar(maxSweeps)&&isfinite(maxSweeps)&& ...
 maxSweeps>=1&&fix(maxSweeps)==maxSweeps,'R96:InvalidProjectionBudget', ...
 'Provide a positive integer numerical projection-work budget.');
original=R.knot_jets;
P0=zeros(3,16,S);
for s=1:S,P0(:,:,s)=double(R.segments(s).payload_cp);end
% Dimensionless (length-valued) scaled jet changes avoid the enormous
% conditioning disparity between velocity, acceleration and 7th derivative.
q=7*max(0,K-2);
B=zeros(16*S,q);
for k=2:K-1
 h=sqrt((R.times_s(k)-R.times_s(k-1))* ...
        (R.times_s(k+1)-R.times_s(k)));
 for d=1:7
  col=7*(k-2)+d;
  scale=exp(gammaln(d+1)-d*log(h)); % physical derivative per length-valued variable
  e=zeros(1,8);e(d+1)=scale;
  for s=max(1,k-1):min(S,k)
    dur=R.times_s(s+1)-R.times_s(s);
    if k==s
      Z=cp_from_jets(e,zeros(1,8),dur);
    else
      Z=cp_from_jets(zeros(1,8),e,dur);
    end
    B(16*(s-1)+(1:16),col)=Z(:);
  end
 end
end
% b-H*x >= 0: one halfspace for every control point and corridor face.
rowCount=0;
for s=1:S
 cr=corridors{s};
 assert(isstruct(cr)&&isfield(cr,'A')&&isfield(cr,'b'), ...
   'R96:MalformedCorridor','Each segment needs A and b.');
 a=double(cr.A);rhs=double(cr.b(:));
 assert(size(a,2)==3&&size(a,1)==numel(rhs)&& ...
   all(isfinite(a(:)))&&all(isfinite(rhs)), ...
   'R96:MalformedCorridorPlanes','Corridor must contain finite A,b.');
 rowCount=rowCount+16*size(a,1);
end
C.halfspace_count=rowCount;
if rowCount==0
 C.status='NO_CORRIDOR_FACES_TO_PROJECT';
 C.control_point_contained=true;
 C.endpoint_0_to_7_exact_input_preserved=true;
 C.numeric_c7_0_to_7_max_residual=R.max_jet_residual;
 return;
end
H=zeros(rowCount,3*q);b=zeros(rowCount,1);
row=0;
for s=1:S
 a=double(corridors{s}.A);rhs=double(corridors{s}.b(:));
 Bb=B(16*(s-1)+(1:16),:);
 for fi=1:size(a,1)
  n=a(fi,:);baseScalar=n*P0(:,:,s);
  % Roundoff guard is a numerical strict-interior tolerance, not a
  % tracking inflation; actual body radii remain in independent R80.
  scale=max([1;abs(rhs(fi));abs(n(:));abs(reshape(P0(:,:,s),[],1))]);
  guard=128*eps(scale);
  for j=1:16
   row=row+1;
   for axis=1:3
    H(row,(axis-1)*q+(1:q))=n(axis)*Bb(j,:);
   end
   b(row)=rhs(fi)-guard-baseScalar(j);
  end
 end
end
C.initial_max_violation_m=max(-b);
if all(b>=0)
 C.status='INITIAL_C7_CONTROL_POINTS_ALREADY_IN_CORRIDORS';
 C.control_point_contained=true;
 C.final_max_violation_m=max(-b);
 C.endpoint_0_to_7_exact_input_preserved=true;
 C.numeric_c7_0_to_7_max_residual=R.max_jet_residual;
 return;
end
if q==0
 C.status='NO_INTERNAL_C7_JETS_TO_MOVE__CORRIDOR_UNRESOLVED';return;
end
% Dual coordinate descent on convex Euclidean projection
% min 0.5 ||x||^2  subject to H*x <= b.
% x=-H'*lambda, lambda >=0. Early termination is allowed only when ALL
% Bernstein CP halfspaces are independently verified below.
lam=zeros(rowCount,1);x=zeros(3*q,1);
h2=sum(H.^2,2);
zeroRows=h2<=realmin;
if any(zeroRows & b<0)
 C.status='CORRIDOR_FIXED_BERNSTEIN_POINT_OUTSIDE';return;
end
for sweep=1:maxSweeps
 for i=1:rowCount
  if zeroRows(i),continue;end
  viol=H(i,:)*x-b(i);
  new=max(0,lam(i)+viol/h2(i));
  dx=new-lam(i);
  if dx~=0
   x=x-H(i,:)'*dx;
   lam(i)=new;
  end
 end
 C.projection_sweeps=sweep;
 if any(~isfinite(x))
  C.status='INCONCLUSIVE_NUMERICAL_PROJECTION';return;
 end
 if max(H*x-b)<=0,break;end
end
C.final_max_violation_m=max(H*x-b);
if C.final_max_violation_m>0
 C.status='INCONCLUSIVE_CORRIDOR_PROJECTION_WORK_BUDGET';return;
end
J=original;
for k=2:K-1
 h=sqrt((R.times_s(k)-R.times_s(k-1))* ...
        (R.times_s(k+1)-R.times_s(k)));
 for d=1:7
  col=7*(k-2)+d;
  factor=exp(gammaln(d+1)-d*log(h));
  for axis=1:3
   J(axis,d+1,k)=original(axis,d+1,k)+factor*x((axis-1)*q+col);
  end
 end
end
if any(~isfinite(J(:)))
 C.status='INCONCLUSIVE_NONFINITE_INTERNAL_JETS';return;
end
P=zeros(3,16,S);
for s=1:S
 P(:,:,s)=cp_from_jets(J(:,:,s),J(:,:,s+1), ...
  R.times_s(s+1)-R.times_s(s));
end
% RECONSTRUCT AND INDEPENDENTLY RECHECK CORRIDOR on actual CP, not H*x-b.
for s=1:S
 a=double(corridors{s}.A);rhs=double(corridors{s}.b(:));
 if ~isempty(a) && any(any(a*P(:,:,s)>rhs*ones(1,16)))
  C.status='INCONCLUSIVE_RECONSTRUCTED_CP_OUTSIDE';return;
 end
end
res=0;
for s=1:S
 D=P(:,:,s);dur=R.times_s(s+1)-R.times_s(s);
 for d=0:7
  res=max(res,max(abs([D(:,1)-J(:,d+1,s); ...
     D(:,end)-J(:,d+1,s+1)]),[],'all'));
  if d<7,D=(size(D,2)-1)/dur*diff(D,1,2);end
 end
end
R.knot_jets=J;
for s=1:S,R.segments(s).payload_cp=P(:,:,s);end
R.max_jet_residual=res;
R.status='R96_C7_CORRIDOR_CONSTRAINED_REFERENCE_ONLY';
R.minimum_8th_derivative_quadratic=NaN; % no false optimality claim
R.original_min8_is_the_only_globally_energy_optimal_route=false;
R.adjusted_jet_solution_is_a_feasibility_heuristic=true;
R.speed_aware_knot_adjusted=true;
C.adjusted_internal_jets=any(x~=0);
C.endpoint_0_to_7_exact_input_preserved=isequaln(J(:,:,1),original(:,:,1))&& ...
 isequaln(J(:,:,K),original(:,:,K));
C.numeric_c7_0_to_7_max_residual=res;
C.control_point_contained=true;
C.status='CORRIDOR_CONSTRAINED_C7_GENERATED__REQUIRES_FULL_CERT';
end

function P=cp_from_jets(A,B,T)
D=size(A,1);P=zeros(D,16);
for j=0:7
 x=zeros(D,1);y=zeros(D,1);
 for r=0:j
  falling=prod(15:-1:(16-r));
  f=nchoosek(j,r)*T^r/falling;
  x=x+f*A(:,r+1);
  y=y+(-1)^r*f*B(:,r+1);
 end
 P(:,j+1)=x;P(:,16-j)=y;
end
end

function Z=AF7R58R170_STRAIGHT_CABLE_CERT(route,packet,obstacles,opts)
% Bernstein-subdivision check of exact straight taut cable occupancy.
% Model: q=(pL''+g*e3)/||pL''+g*e3||, pQ=pL+L*q.
% For each time leaf and face n, bound n*q from its acceleration Bernstein
% convex hull via normalized-ball cone support; no L-radius cable sphere.
% Each cable segment is treated by its two endpoints and its own radius rC.
% For original convex obstacles, require one 3D normal separating ALL
% payload/cable/UAV occupied points over each leaf from original support.
% CONDITIONAL real-arithmetic reference certificate ONLY: numerical doubles
% are not outward-rounded and tracking/HLC/torque/tail are not proved.
if nargin<4||isempty(opts),opts=struct();end
if ~isfield(opts,'max_leaves_per_segment'),opts.max_leaves_per_segment=512;end
if ~isfield(opts,'robust_tube_on'),opts.robust_tube_on=true;end
Z=struct('status','UNKNOWN_INCOMPLETE_DATA','candidate_clear',false,...
 'corridor_clear',false,'original_shapes_clear',false,...
 'numerical_real_arithmetic_only',true,'IEEE_outward_interval_certified',false,...
 'uses_cable_length_sphere',false,'checked_leaves',0,'checked_segments',0,...
 'min_numeric_corridor_clearance_m',NaN,'min_numeric_shape_gap_m',NaN,...
 'first_unresolved_segment',NaN,'production_commit_authorized',false);
need={'live_physical_parameters_valid','source_cable_length_m',...
 'source_gravity_mps2','source_radii_m'};
if ~isstruct(packet)||~isscalar(packet)||~all(isfield(packet,need))||...
 ~isequal(packet.live_physical_parameters_valid,true)||...
 ~isstruct(route)||~isscalar(route)||~all(isfield(route,{'C7','cells'}))||...
 ~isstruct(route.C7)||~isfield(route.C7,'feasible')||...
 ~route.C7.feasible||~iscell(route.C7.control_points)||...
 ~iscell(route.cells)||~isfield(route.C7,'times_s'),return;end
L=double(packet.source_cable_length_m);g=double(packet.source_gravity_mps2);
r=double(packet.source_radii_m(:));
if numel(r)~=3||any(~isfinite([L;g;r]))||...
 L<=0||g<=0||any(r<0),return;end
margin=0;
if opts.robust_tube_on
 if ~all(isfield(packet,{'certified_tracking_tube_m','certified_tracking_tube_source'}))||...
 isempty(packet.certified_tracking_tube_source)||...
 ~isscalar(packet.certified_tracking_tube_m)||...
 ~isfinite(packet.certified_tracking_tube_m)||...
 packet.certified_tracking_tube_m<0
  Z.status='UNKNOWN_MISSING_PROVEN_TRACKING_TUBE';return;
 end
 margin=double(packet.certified_tracking_tube_m);
end
r=r+margin;
if isempty(obstacles)||~isstruct(obstacles)
 Z.status='UNKNOWN_NO_ORIGINAL_OBSTACLE_SET';return;
end
T=double(route.C7.times_s(:));n=numel(route.C7.control_points);
if numel(T)~=n+1||numel(route.cells)~=n||...
 any(~isfinite(T))||any(diff(T)<=0)||...
 ~isscalar(opts.max_leaves_per_segment)||...
 opts.max_leaves_per_segment<1||...
 fix(opts.max_leaves_per_segment)~=opts.max_leaves_per_segment,return;end
minCorr=Inf;minShape=Inf;
for s=1:n
 P=double(route.C7.control_points{s});
 if isequal(size(P),[16,3]),P=P';end
 H=double(route.cells{s}.A);b=double(route.cells{s}.b(:));
 if ~isequal(size(P),[3,16])||size(H,2)~=3||size(H,1)<4||...
 size(H,1)~=numel(b)||any(~isfinite([P(:);H(:);b(:)]))
  Z.status='UNKNOWN_MALFORMED_CP_OR_CORRIDOR';return;
 end
 Q={struct('P',P,'dt',T(s+1)-T(s))};used=0;
 while ~isempty(Q)
  a=Q{end};Q(end)=[];used=used+1;
  [good,cm,sm]=check_leaf(a.P,a.dt,H,b,L,g,r,obstacles);
  if good
   minCorr=min(minCorr,cm);minShape=min(minShape,sm);continue;
  end
  if used+numel(Q)+2>opts.max_leaves_per_segment
   Z.status='UNKNOWN_SPLIT_BUDGET_OR_UNSAFE_GEOMETRY';
   Z.first_unresolved_segment=s;Z.checked_leaves=Z.checked_leaves+used;return;
  end
  [A,B]=split_half(a.P);
  Q{end+1}=struct('P',B,'dt',a.dt/2); %#ok<AGROW>
  Q{end+1}=struct('P',A,'dt',a.dt/2); %#ok<AGROW>
 end
 Z.checked_leaves=Z.checked_leaves+used;Z.checked_segments=s;
end
Z.corridor_clear=true;Z.original_shapes_clear=true;
Z.candidate_clear=true;Z.min_numeric_corridor_clearance_m=minCorr;
Z.min_numeric_shape_gap_m=minShape;
Z.status='CONDITIONAL_FULL_STRAIGHT_CABLE_CORRIDOR_AND_SOURCE_SHAPES__NO_FLIGHT';
end
function [good,cs,ss]=check_leaf(P,dt,H,b,L,g,r,obs)
good=false;cs=-Inf;ss=-Inf;
if ~isfinite(dt)||dt<=0,return;end
F=(15*14/dt^2)*diff(P,2,2)+[0;0;g];
c=mean(F,2);rho=max(sqrt(sum((F-c).^2,1)));
[qu,~,ok]=AF7R58R170_DIRECTIONAL_SUPPORT(H,c,rho);
if ~ok,return;end
% R179 canonical component geometry: [UAV ball, cable capsule, load ball].
% The support uses the segment endpoints; L never serves as a ball radius.
contract=AF7R58R179_TAUT_PLANE(H,P,[],qu,L,r);
if ~contract.valid,return;end
face=max(contract.upper,[],2);
slack=b-face;
cs=min(slack);
guard=4096*eps(max([1;abs(b);abs(face)],[],'all'));
if ~isfinite(cs)||cs<=guard,return;end
center=mean(P,2)+L*c/(2*norm(c));
ss=Inf;
for k=1:numel(obs)
 o=obs(k);
 if ~isfield(o,'p_center')||numel(o.p_center)~=3||...
 any(~isfinite(double(o.p_center(:)))),return;end
 co=double(o.p_center(:));
 normals=[center-co,mean(P,2)-co,mean(P,2)+L*c/norm(c)-co,...
         P(:,1)-co,P(:,end)-co,eye(3),-eye(3),H'];
 found=false;best=-Inf;
 for j=1:size(normals,2)
  v=normals(:,j);nv=norm(v);if nv==0||~isfinite(nv),continue;end
  v=v/nv;
  [hu,valid]=AF7R58R164_SOURCE_SUPPORT(o,v);
  if ~valid,return;end
  [~,qlow,valid]=AF7R58R170_DIRECTIONAL_SUPPORT(v',c,rho);
  if ~valid,return;end
  contract=AF7R58R179_TAUT_PLANE(v',P,qlow,[],L,r);
  if ~contract.valid,return;end
  gap=min(contract.lower,[],2)-hu;
  best=max(best,gap);
  if gap>4096*eps(max(abs([1;contract.lower(:);hu;L;r(:)])))
    found=true;break;
  end
 end
 if ~found,ss=best;return;end
 ss=min(ss,best);
end
good=true;
end
function [A,B]=split_half(P)
N=size(P,2);X=P;A=zeros(size(P));B=zeros(size(P));
A(:,1)=P(:,1);B(:,end)=P(:,end);
for i=2:N
 X=.5*(X(:,1:end-1)+X(:,2:end));
 A(:,i)=X(:,1);B(:,N-i+1)=X(:,end);
end
end

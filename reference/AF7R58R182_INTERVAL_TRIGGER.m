function B=AF7R58R182_INTERVAL_TRIGGER(P,obstacles,radii,physics)
% R182 | local continuous-interval whole-body separation / risk SCREEN.
% Model: piecewise actual HLC+plant numerical rollout, not worst-input 4-s ball.
% UAV and payload are independent spheres. Cable is the COMPLETE straight
% segment between these endpoints, Minkowski-expanded by its own radius.
%
% Mathematical contract (conditional on exact numerical knots and model):
% If ||p''||<=A on [ti,ti+h], then for 0<=s<=1
% p(ti+s*h) is in conv(p_i,p_i+1) + Ball(A*h^2/8).
% Thus C(t) is contained in conv(Qi,Qi+1,Li,Li+1) + Ball(max(EQ,EL)+rC).
% Separation by a single oriented-ellipsoid supporting plane proves
% NO INTERSECTION on that whole interval in real arithmetic.
%
% INPUT WARNING: An R71 ODE rollout is NOT a validated integrator and the
% EKF is NOT the guaranteed true plant state. We DO NOT turn this geometric
% condition into actual 4-s certified closed-loop safety or aircraft commit.
B=struct('status','UNKNOWN_INPUT','model_conditional_clear',false,...
 'possible_risk_request',false,'first_unseparated_interval',NaN,...
 'first_unseparated_time_s',NaN,'first_obstacle_id',NaN,...
 'first_component_code',0,'unseparated_intervals',0,...
 'tested_intervals',0,'max_local_interpolation_Q_m',NaN,...
 'max_local_interpolation_L_m',NaN,...
 'conditional_model_only',true,'real_HLC_reach_proved',false,...
 'EKF_error_tube_proved',false,'validated_numeric_integration',false,...
 'IEEE_outward_rounding_proved',false,'production_commit_authorized',false,...
 'uses_length_sphere',false,'cable_shape','STRAIGHT_SEGMENT_CAPSULE',...
 'error_message','');
need={'t','pQ','pL','x24'};
phy={'mQ_kg','mL_kg','L_m','g_mps2','fmax_N','horizon_s'};
if ~isstruct(P)||~isscalar(P)||~all(isfield(P,need))||...
 ~isstruct(physics)||~isscalar(physics)||~all(isfield(physics,phy))||...
 ~isstruct(obstacles)||~isnumeric(radii)||numel(radii)~=3
 return;
end
t=double(P.t(:).');Q=double(P.pQ);L=double(P.pL);X=double(P.x24);
r=double(radii(:));
if numel(t)<2||~isequal(size(Q),[3,numel(t)])||...
 ~isequal(size(L),size(Q))||~isequal(size(X),[24,numel(t)])||...
 any(~isfinite([t(:);Q(:);L(:);X(:);r]))||any(r<0)||any(diff(t)<=0)||...
 ~all(isfield(obstacles,{'id','p_center','R_obs','ellipsoid_radii'}))
 return;
end
par=double([physics.mQ_kg physics.mL_kg physics.L_m physics.g_mps2 ...
 physics.fmax_N physics.horizon_s]);
if any(~isfinite(par))||any(par(1:4)<=0)||par(5)<0||par(6)~=4||...
 abs(t(end)-t(1)-4)>256*eps(max([1;abs(t(:))]))
 B.status='UNKNOWN_PHYSICS_OR_FOUR_SECOND_GRID';return;
end
% Nodal consistency and SO(3) shape precondition. Never project the EKF.
% Each comparison is 3-by-N.  Reduce ALL elements to scalar logicals
% before using the short-circuit OR operator.
deltaQ=abs(X(1:3,:)-Q);deltaL=abs(X(13:15,:)-L);
tolQ=1e-7*max(1,max(abs(Q(:))));
tolL=1e-7*max(1,max(abs(L(:))));
if any(deltaQ(:)>tolQ) || any(deltaL(:)>tolL)
 B.status='UNKNOWN_PHYSICAL_PREVIEW_STATE_MISMATCH';return;
end
for j=1:numel(obstacles)
 o=obstacles(j); a=double(o.ellipsoid_radii(:));c=double(o.p_center(:));...
 Rot=double(o.R_obs);
 if numel(a)~=3||numel(c)~=3||~isequal(size(Rot),[3 3])||...
  any(~isfinite([a;c;Rot(:)]))||any(a<=0)||...
  norm(Rot'*Rot-eye(3),'fro')>1e-7||det(Rot)<0.9
  B.status='UNKNOWN_NON_SO3_ORIGINAL_ELLIPSOID';return;
 end
end
m=par(1);ml=par(2);Lc=par(3);g=par(4);f=par(5);
mu=m*ml/(m+ml);
B.max_local_interpolation_Q_m=0;
B.max_local_interpolation_L_m=0;
% Report earliest interval, independent of the order of obstacle storage.
for k=1:numel(t)-1
 h=t(k+1)-t(k);
 % Under taut suspended-load dynamics only:
 % ||d/dt |qdot||| <= fmax/(mQ*Lc), since transverse gravity cancels.
 % The angular-speed bound is LOCAL, never expanded over 4 seconds.
 w0=norm(X(22:24,k));w1=norm(X(22:24,k+1));
 w=max(w0,w1)+f*h/(m*Lc);
 T=ml/(m+ml)*f+mu*Lc*w*w;
 aq=f/m+T/m+g;
 al=T/ml+g;
 eq=aq*h*h/8;el=al*h*h/8;
 if any(~isfinite([w,T,aq,al,eq,el]))||any([eq el]<0)
  B.status='UNKNOWN_LOCAL_DYNAMICS_OVERFLOW';return;
 end
 B.max_local_interpolation_Q_m=max(B.max_local_interpolation_Q_m,eq);
 B.max_local_interpolation_L_m=max(B.max_local_interpolation_L_m,el);
 % Check for an explicitly VERIFIED nodal solver-error/observer tube if
 % provided. Otherwise these are ZERO *conditional model* error terms only.
 nodeQ=0;nodeL=0;
 if isfield(physics,'node_error_Q_m') && isfield(physics,'node_error_L_m')
  nQ=double(physics.node_error_Q_m(:));nL=double(physics.node_error_L_m(:));
  if numel(nQ)~=numel(t)||numel(nL)~=numel(t)||...
   any(~isfinite([nQ;nL]))||any([nQ;nL]<0)
   B.status='UNKNOWN_NODE_ERROR_TUBE_SHAPE';return;
  end
  nodeQ=max(nQ(k:k+1));nodeL=max(nL(k:k+1));
 end
 % Slight round-off guard, NOT a directed IEEE-754 enclosure proof.
 scale=max([1;abs(Q(:,k));abs(Q(:,k+1));abs(L(:,k));abs(L(:,k+1))]);
 guard=8192*eps(scale);
 EQ=eq+nodeQ+guard;EL=el+nodeL+guard;
 for j=1:numel(obstacles)
  o=obstacles(j);c=double(o.p_center(:));Rot=double(o.R_obs);a=double(o.ellipsoid_radii(:));
  shapes={Q(:,k:k+1),L(:,k:k+1),[Q(:,k:k+1),L(:,k:k+1)]};
  infl=[r(1)+EQ;r(3)+EL;r(2)+max(EQ,EL)];
  codes=[2,1,3]; % native 2=Q, 1=payload, 3=cable
  for b=1:3
   points=shapes{b};
   if ~r182_separated(points,c,Rot,a,infl(b))
    B.unseparated_intervals=B.unseparated_intervals+1;
    if isnan(B.first_unseparated_interval)||...
      k<B.first_unseparated_interval||...
      (k==B.first_unseparated_interval && double(o.id)<B.first_obstacle_id)
     B.first_unseparated_interval=k;
     B.first_unseparated_time_s=t(k);
     B.first_obstacle_id=double(o.id);
     B.first_component_code=codes(b);
    end
   end
  end
 end
 B.tested_intervals=k;
end
if B.unseparated_intervals>0
 B.status='MODEL_INTERVAL_UNSEPARATED__NOT_PROVED_COLLISION';
 B.possible_risk_request=true;
else
 B.status='MODEL_INTERVAL_GEOMETRY_SEPARATED__NOT_CLOSED_LOOP_CERTIFIED';
 B.model_conditional_clear=true;
end
end

function separated=r182_separated(P,c,R,a,extra)
% Candidate separating normals are only a SEARCH heuristic; a successful
% positive support inequality is an exact (real-arithmetic) sufficient test.
% No use of an ellipsoid bounding sphere for the acceptance test.
separated=false;
% Cheap, rigorously sufficient (real arithmetic) local broad phase.
% For the four-point cable hull, bound its change from the INITIAL segment
% by the maximum endpoint displacement DURING THIS ONE CONTROL INTERVAL.
% This is NOT a cable-length-sized ball and is never a 4-second inflation.
if size(P,2)==2
 u=P(:,1);v=P(:,2);temporalShift=0;
else
 u=P(:,1);v=P(:,3);
 temporalShift=max(norm(P(:,2)-u),norm(P(:,4)-v));
end
d=v-u; den=d'*d;
if den>0
 alpha=min(1,max(0,((c-u)'*d)/den));
 closest=u+alpha*d;
else
 closest=u;
end
lowBound=norm(closest-c)-temporalShift-max(a)-extra;
if lowBound>4096*eps(max([1;abs(lowBound);abs(c);abs(P(:))]))
 separated=true;return;
end
S=[P- c,mean(P,2)-c,R,-R];
if size(P,2)==4
 u=mean(P(:,1:2),2);v=mean(P(:,3:4),2);
 d=v-u;den=d'*d;
 if den>0
  lambda=min(1,max(0,(c-u)'*d/den));
  S=[S,u+lambda*d-c]; %#ok<AGROW>
 end
end
for kk=1:size(S,2)
 n=S(:,kk);d=norm(n);
 if d<1e-12||~isfinite(d),continue;end
 n=n/d;
 support=n'*c+norm(a.*(R'*n));
 gap=min(n'*P,[],'all')-extra-support;
 if gap>64*eps(max([1;abs(gap);abs(support)]))
  separated=true;return;
 end
end
end

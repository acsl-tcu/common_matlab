function B=AF7R58R168_PHYSICAL_BUDGET(seed,packet,speedCert,alpha)
% Geometry-neutral, PHYSICS-DERIVED sufficient derivative envelopes.
% Every bound comes from current same-tick mQ,mL,L,g,f bounds,VQ and
% candidate's Bernstein derivative ratios at chosen timeline.
% Do not use as torque/closed-loop/IEEE proof. No empirical speed/jerk cap.
B=struct('status','UNKNOWN_SAME_TICK_AUTHORITY','usable',false,...
 'limits_mps_mps2_mps3_mps4',nan(1,4),'lambda',NaN,...
 'seed_hulls',nan(1,4),'hover_thrust_N',NaN,...
 'conditional_taut_force_speed_sufficient',false,...
 'production_commit_authorized',false);
req={'live_physical_parameters_valid','t_s','source_mass_kg',...
 'estimated_mL_kg','source_cable_length_m','source_gravity_mps2','thrust_bounds_N'};
if ~isstruct(packet)||~isscalar(packet)||~all(isfield(packet,req))||...
 ~isequal(packet.live_physical_parameters_valid,true)||...
 ~isstruct(speedCert)||~isscalar(speedCert)||...
 ~all(isfield(speedCert,{'source_time_s','VQ_limit_mps','bound_derived_from_live_sensing'}))||...
 ~isequal(speedCert.bound_derived_from_live_sensing,true)||...
 ~isequal(double(speedCert.source_time_s),double(packet.t_s)),return;end
mQ=double(packet.source_mass_kg);mL=double(packet.estimated_mL_kg);
L=double(packet.source_cable_length_m);g=double(packet.source_gravity_mps2);
f=double(packet.thrust_bounds_N(:));VQ=double(speedCert.VQ_limit_mps);
if numel(f)~=2||~isscalar(alpha)||~isfinite(alpha)||alpha<1||...
 ~isscalar(VQ)||~all(isfinite([mQ;mL;L;g;f;VQ]))||...
 min([mQ,mL,L,g,VQ])<=0||f(1)<0||f(2)<=f(1),return;end
if ~isstruct(seed)||~isfield(seed,'C7')||...
 ~isfield(seed.C7,'control_points')||~iscell(seed.C7.control_points)||...
 ~isfield(seed.C7,'times_s'),return;end
C=seed.C7;t=double(C.times_s(:));n=numel(C.control_points);
if n<1||numel(t)~=n+1||any(~isfinite(t))||any(diff(t)<=0),return;end
D=zeros(1,4);
for s=1:n
 P=double(C.control_points{s});if isequal(size(P),[16,3]),P=P';end
 if ~isequal(size(P),[3,16])||any(~isfinite(P(:))),return;end
 T=(t(s+1)-t(s))*alpha;
 for k=1:4
  M=prod(16-k:15)/T^k*diff(P,k,2);
  D(k)=max(D(k),max(vecnorm(M,2,1)));
 end
end
B.seed_hulls=D;
B.hover_thrust_N=(mQ+mL)*g;
if B.hover_thrust_N>=f(2)||B.hover_thrust_N<=f(1)
 B.status='NO_STRICT_HOVER_THRUST_MARGIN';return;end
% For envelopes A,J,S: sigma >= g-A>0,
% ||qdot|| <= J/(g-A), ||qddot|| <= S/(g-A)+3(J/(g-A))^2.
% ||vQ|| <= V+L*J/(g-A),
% fUpper <= (mQ+mL)*(g+A)+mQ*L*qddotUpper,
% fLower >= (mQ+mL)*(g-A)-mQ*L*qddotUpper.
% Find maximum uniform fraction of candidate's derivative hull permitted
% by THESE mathematical sufficient inequalities. Not an obstacle-direction
% filter, not an ad-hoc physical upper bound. Values are NOT a guarantee
% until all control-point derivative inequalities pass.
if ~feasible(0,D,mQ,mL,L,g,f,VQ)
 B.status='UNKNOWN_NO_FEASIBLE_DERIVATIVE_BUDGET';return;end
lo=0;hi=1;
if feasible(hi,D,mQ,mL,L,g,f,VQ),lo=hi;else
 for j=1:56
  mid=(lo+hi)/2;
  if feasible(mid,D,mQ,mL,L,g,f,VQ),lo=mid;else,hi=mid;end
 end
end
% Strict interior budget is used to avoid landing exactly on double bounds.
fac=1-4096*eps(1);
B.lambda=lo*fac;
B.limits_mps_mps2_mps3_mps4=B.lambda*D;
B.usable=all(isfinite(B.limits_mps_mps2_mps3_mps4))&&B.lambda>0;
B.conditional_taut_force_speed_sufficient=B.usable&&...
 feasible(B.lambda,D,mQ,mL,L,g,f,VQ);
if B.usable,B.status='CONDITIONAL_PHYSICS_DERIVED_QP_DERIVATIVE_BUDGET__NO_TORQUE_OR_FLIGHT';end
end
function tf=feasible(a,D,mQ,mL,L,g,f,VQ)
V=a*D(1);A=a*D(2);J=a*D(3);S=a*D(4);
sig=g-A;tf=false;
if sig<=0,return;end
w=J/sig;dd=S/sig+3*w^2;
hi=(mQ+mL)*(g+A)+mQ*L*dd;
lo=(mQ+mL)*(g-A)-mQ*L*dd;
v=V+L*w;
tf=isfinite(v)&&isfinite(hi)&&isfinite(lo)&&...
 hi<f(2)&&lo>f(1)&&v<VQ;
end

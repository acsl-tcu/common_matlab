function R=AF7R58R184_C7_HEADING_GATE(candidate)
% R184R2 | Reference-only exact-degree Bernstein C7 numeric-jets / yaw C6 gate.
% No HLC/plant modifications, no actuator/observer guarantees.
% A STRICT safety certificate cannot be inferred from approximate jet equality.
% Horizontal nonzero speed is established by a sufficient Bernstein separating
% direction; finite-precision numerics here are NOT directed-rounded proofs.
R=struct('status','UNKNOWN_MALFORMED_C7','numeric_jets_C7_close',false,...
 'heading_nonzero_sufficient_real_arithmetic',false,...
 'conditional_heading_C6',false,'minimum_tangent_lower_bound_mps',NaN,...
 'max_abs_jet_jump',NaN,'first_bad_joint',NaN,...
 'outward_rounding_proved',false,'production_commit_authorized',false);
if ~isstruct(candidate)||~isscalar(candidate)||...
 ~all(isfield(candidate,{'times_s','control_points'}))||...
 ~iscell(candidate.control_points),return;end
T=double(candidate.times_s(:));N=numel(candidate.control_points);
if numel(T)~=N+1||N<1||any(~isfinite(T))||any(diff(T)<=0),return;end
start=zeros(3,8,N);finish=zeros(3,8,N);
minspeed=Inf;
for i=1:N
 P=double(candidate.control_points{i});
 if isequal(size(P),[16,3]),P=P';end
 if ~isequal(size(P),[3,16])||any(~isfinite(P(:))),return;end
 h=T(i+1)-T(i);
 % Endpoint derivative of degree-15 Bezier on [T_i,T_{i+1}]:
 %   p^(k)(0) = 15!/(15-k)!/h^k * Delta^k P_0
 %   p^(k)(1) = 15!/(15-k)!/h^k * Delta^k P_(15-k)
 % MATLAB diff(A,0,dim) is invalid, so handle order 0 explicitly.
 D=P;
 for k=0:7
  if k>0
   D=diff(D,1,2); % exact order-k finite difference; no k=0 call
  end
  factor=prod(16-k:15)/h^k;
  start(:,k+1,i)=factor*D(:,1);
  finish(:,k+1,i)=factor*D(:,end);
 end
 % A common normal n with n'*V_i > 0 bounds horizontal speed away
 % from zero for all t in this segment (sufficient, not necessary).
 V=(15/h)*diff(P(1:2,:),1,2);
 normals=[mean(V,2),V];found=false;best=-Inf;
 for j=1:size(normals,2)
  n=normals(:,j);dn=norm(n);
  if dn<=0||~isfinite(dn),continue;end
  n=n/dn;bound=min(n'*V);
  scale=max([1;abs(V(:))]);
  if bound>4096*eps(scale)
   best=max(best,bound);found=true;
  end
 end
 if ~found
  R.status='UNKNOWN_HORIZONTAL_TANGENT_MAY_VANISH';return;
 end
 minspeed=min(minspeed,best);
end
R.minimum_tangent_lower_bound_mps=minspeed;
R.heading_nonzero_sufficient_real_arithmetic=true;
maxJump=0;
for i=1:N-1
 for k=0:7
  x=finish(:,k+1,i);y=start(:,k+1,i+1);
  d=max(abs(x-y));maxJump=max(maxJump,d);
  scale=max([1;abs(x);abs(y)]);
  if d>4096*eps(scale)
   R.max_abs_jet_jump=maxJump;R.first_bad_joint=i;
   R.status='REJECT_C7_JOINT_JET_MISMATCH';return;
  end
 end
end
R.max_abs_jet_jump=maxJump;
R.numeric_jets_C7_close=true;
R.conditional_heading_C6=true;
R.status='CONDITIONAL_NUMERIC_C7_JETS_AND_EXACT_TANGENT_YAW_C6__NOT_REAL_EXECUTION';
end

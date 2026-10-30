function R=AF7R58R168_CONSTRAINED_C7QP(route,J0,Jend,tseg,limits,options)
% Convex, rotation-invariant d1..d4 L2 constraints IN the C7 generator.
% Continuous-time sufficient constraints:
%  - All position Bernstein control points: A_i P_ij <= b_i.
%  - All derivative control points: ||D^k P_ij||_2 <= limit(k), k=1..4.
% Since Bernstein bases are nonnegative and sum to one, both guarantees
% hold over the ENTIRE polynomial segment in exact arithmetic.
% Enforce SOC norms using sequential QP cutting planes (separation oracle)
% u = violated_vector / norm(violated_vector), add u' D^k P_ij<=limit.
% This is NOT a direction heuristic: cuts come from actual 3D violations.
% Non-convergence, solver exception, missing authority => fail-closed.
% Double QP arithmetic is NOT interval/outward IEEE certification.
if nargin<6||isempty(options),options=struct();end
if ~isfield(options,'max_cut_rounds'),options.max_cut_rounds=28;end
assert(isscalar(options.max_cut_rounds)&&isfinite(options.max_cut_rounds)&&...
 options.max_cut_rounds>=1&&fix(options.max_cut_rounds)==options.max_cut_rounds,...
 'R168:InvalidCutRoundBudget');
R=struct('status','UNKNOWN_INPUT_OR_TOOLBOX','feasible',false,...
 'control_points',{{}},'times_s',[],'numeric_join_residual',NaN,...
 'max_corridor_excess',NaN,'max_derivative_excess',NaN,...
 'solver_exitflag',NaN,'production_commit_authorized',false,...
 'position_C7_structural',false,'derivative_norm_model','L2_SOCP_QP_SEPARATION_ORACLE',...
 'cone_cut_rounds',0,'cone_cuts_added',0);
if ~isstruct(route)||~isscalar(route)||~isfield(route,'cells')||...
 ~iscell(route.cells)||~isfield(route,'payload_knots')||...
 ~isequal(size(J0),[3,8])||~isequal(size(Jend),[3,8]),return;end
K=double(route.payload_knots);
if size(K,1)~=3&&size(K,2)==3,K=K';end
ns=size(K,2)-1;h=double(tseg(:)');lim=double(limits(:)');
if ~isequal(size(K),[3,ns+1])||ns<1||numel(route.cells)~=ns||...
 numel(h)~=ns||any(~isfinite(h))||any(h<=0)||numel(lim)~=4||...
 any(~isfinite(lim))||any(lim<0)||...
 any(~isfinite([K(:);J0(:);Jend(:)])),return;end
if norm(J0(:,1)-K(:,1),inf)>1e-9||...
 norm(Jend(:,1)-K(:,end),inf)>1e-9
 R.status='REJECT_JET_POSITION_NOT_CORRIDOR_ENDPOINT';return;end
if exist('quadprog','file')~=2,R.status='UNKNOWN_QUADPROG_UNAVAILABLE';return;end
% Exact shared-jet representation; optimize ALL interior derivatives d1..d7.
N=3*7*(ns-1);tau=sum(h)/ns;
Jbase=cell(1,ns+1);Jbase{1}=J0;Jbase{end}=Jend;
for s=2:ns
 Jbase{s}=zeros(3,8);Jbase{s}(:,1)=K(:,s);
end
base=assemble(Jbase,h);
B=cell(1,ns);for s=1:ns,B{s}=zeros(48,N);end
for v=1:N
 knot=2+floor((v-1)/21);r=mod(v-1,21);
 ord=1+floor(r/3);axis=1+mod(r,3);
 Jtmp=Jbase;Jtmp{knot}(axis,ord+1)=tau^(-ord);
 tmp=assemble(Jtmp,h);
 for s=max(1,knot-1):min(ns,knot)
  B{s}(:,v)=reshape(tmp{s}-base{s},48,1);
 end
end
AA=zeros(0,N);bb=zeros(0,1);HH=zeros(N);ff=zeros(N,1);
D0=cell(ns,4);DZ=cell(ns,4);
for s=1:ns
 C=route.cells{s};
 if ~isstruct(C)||~all(isfield(C,{'A','b'}))
  R.status='UNKNOWN_CORRIDOR_HALFSPACES';return;end
 A=double(C.A);b=double(C.b(:));
 if size(A,2)~=3||numel(b)~=size(A,1)||...
 any(~isfinite([A(:);b(:)]))
  R.status='UNKNOWN_CORRIDOR_HALFSPACES';return;end
 P=base{s};Z=B{s};
 for j=1:16
  ix=(j-1)*3+(1:3);
  AA=[AA;A*Z(ix,:)];bb=[bb;b-A*P(:,j)]; %#ok<AGROW>
 end
 for order=1:4
  D0{s,order}=deriv_cp(P,h(s),order);
  DZ{s,order}=zeros(3*(16-order),N);
  for a=1:N
   basis=reshape(Z(:,a),3,16);
   X=deriv_cp(basis,h(s),order);
   DZ{s,order}(:,a)=X(:);
  end
 end
 % Objective: least squared deviation from source 3D seed CP, or chord;
 % purely numerical regularization, no arbitrary motion-priority weights.
 guide=zeros(3,16);seedAvailable=false;
 if isfield(route,'C7')&&isstruct(route.C7)&&...
 isfield(route.C7,'control_points')&&iscell(route.C7.control_points)&&...
 numel(route.C7.control_points)==ns
  guide=double(route.C7.control_points{s});
  if isequal(size(guide),[16,3]),guide=guide';end
  seedAvailable=isequal(size(guide),[3,16])&&all(isfinite(guide(:)));
 end
 if ~seedAvailable
  for j=1:16
   u=(j-1)/15;guide(:,j)=(1-u)*K(:,s)+u*K(:,s+1);
  end
 end
 delta=reshape(P-guide,48,1);
 HH=HH+Z'*Z;ff=ff+Z'*delta;
end
% All SOC constraints are enforced via their exact separating halfspace
% for the current violation. A finite, certified-to-double solution is
% accepted only after checking every derivative CP in full L2 norm.
if N>0
 reg=1e-10*max(1,trace(HH)/N);
 HH=(HH+HH')/2+reg*eye(N);
 qpOpts=optimoptions('quadprog','Display','off',...
  'Algorithm','interior-point-convex','ConstraintTolerance',1e-10);
else
 qpOpts=[];
end
x=zeros(N,1);converged=false;
for it=1:options.max_cut_rounds
 R.cone_cut_rounds=it;
 if N==0
  if any(bb<-512*eps(max([1;abs(bb)])))
   R.status='REJECT_SINGLE_SEGMENT_FIXED_ENDPOINT_CORRIDOR';return;
  end
  exitflag=1;
 else
  try
   [x,~,exitflag]=quadprog(HH,ff,AA,bb,[],[],[],[],[],qpOpts);
  catch ME
   R.status=['UNKNOWN_QP_EXCEPTION_' ME.identifier];return;
  end
  if isempty(x)||exitflag<=0
   R.status='INCONCLUSIVE_C7_QP_PRIMAL';R.solver_exitflag=exitflag;return;
  end
 end
 R.solver_exitflag=exitflag;
 cutsA=zeros(0,N);cutsB=zeros(0,1);
 for s=1:ns
  for k=1:4
   Y=D0{s,k}+reshape(DZ{s,k}*x,3,16-k);
   for v=1:16-k
    val=Y(:,v);normval=norm(val);
    tol=256*eps(max([1;abs(val);lim(k)]));
    if normval>lim(k)+tol
     if N==0
      R.status='REJECT_SINGLE_SEGMENT_FIXED_ENDPOINT_DERIVATIVE_NORM';return;
     end
     u=val/normval;ix=(v-1)*3+(1:3);
     Zk=DZ{s,k};cutsA(end+1,:)=u'*Zk(ix,:); %#ok<AGROW>
     cutsB(end+1,1)=lim(k)-u'*D0{s,k}(:,v); %#ok<AGROW>
    end
   end
  end
 end
 if isempty(cutsB),converged=true;break;end
 AA=[AA;cutsA];bb=[bb;cutsB]; %#ok<AGROW>
 R.cone_cuts_added=R.cone_cuts_added+numel(cutsB);
end
if ~converged
 R.status='UNKNOWN_SOCP_SEPARATION_DID_NOT_CONVERGE_WITHIN_COMPUTE_BUDGET';return;
end
P=cell(1,ns);corr=-inf;derr=-inf;
for s=1:ns
 P{s}=base{s}+reshape(B{s}*x,3,16);
 C=route.cells{s};corr=max(corr,max(double(C.A)*P{s}-double(C.b(:)),[],'all'));
 for k=1:4
  V=deriv_cp(P{s},h(s),k);
  derr=max(derr,max(vecnorm(V,2,1)-lim(k)));
 end
end
R.control_points=P;R.times_s=[0,cumsum(h)];
R.max_corridor_excess=corr;R.max_derivative_excess=derr;
X=route;X.C7=struct('control_points',{P},'times_s',R.times_s);
J=AF7R58R168_JET_AUDIT(X);
R.numeric_join_residual=max(J.max_relative_jump_by_order);
allCP=cell2mat(P);scale=max([1;abs(allCP(:))]);
tol=512*eps(max(scale));
if corr>tol||derr>tol||R.numeric_join_residual>1e-7
 R.status='REJECT_POSTSOLVE_CORRIDOR_DERIVATIVE_NORM_OR_C7_JOIN';return;
end
R.position_C7_structural=true;R.feasible=true;
R.status='CONDITIONAL_3D_L2_SCP_C7QP_CORRIDOR_DERIVATIVES__PHYSICAL_GATE_PENDING';
end
function CP=assemble(J,h)
CP=cell(1,numel(h));
for s=1:numel(h)
 P=zeros(3,16);
 for k=0:7
  a=zeros(3,1);b=zeros(3,1);
  for j=0:k
   fall=1;if j>0,fall=prod(16-j:15);end
   gain=nchoosek(k,j)*h(s)^j/fall;
   a=a+gain*J{s}(:,j+1);
   b=b+(-1)^j*gain*J{s+1}(:,j+1);
  end
  P(:,k+1)=a;P(:,16-k)=b;
 end
 CP{s}=P;
end
end
function P=deriv_cp(P,T,k)
for j=1:k,P=(16-j)/T*diff(P,1,2);end
end

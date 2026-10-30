function Y=AF7R58R169_HEADING_C6(route,options)
% Reference-only heading yaw = continuous lift of atan2(vy,vx).
% C7 position implies v in C6. If vx^2+vy^2 has STRICT positive
% Bernstein coefficients on adaptively subdivided segments, then heading
% is well-defined C6 everywhere in exact arithmetic. No axis preference:
% only the prescribed horizontal heading projection is used.
% This is double-based sufficient evidence, NOT outward interval proof or
% torque feasibility; cannot by itself authorize a Flight commit.
if nargin<2||isempty(options),options=struct();end
if ~isfield(options,'max_leaves'),options.max_leaves=256;end
Y=struct('status','UNKNOWN_INVALID_C7','positive_horizontal_speed_bernstein',false,...
 'heading_exact_C6_conditional',false,'segment_count',0,'leaves',0,...
 'min_squared_speed_bernstein_coefficient',NaN,...
 'max_numeric_yaw_join_relative',NaN,'yaw_start_jets',{{}},...
 'yaw_end_jets',{{}},'yaw_samples',{{}},...
 'zero_speed_or_unresolved',true,...
 'position_C7_numeric_join_residual',NaN,...
 'production_commit_authorized',false);
if ~isstruct(route)||~isscalar(route)||~isfield(route,'C7')||...
 ~isstruct(route.C7)||~isfield(route.C7,'control_points')||...
 ~iscell(route.C7.control_points)||~isfield(route.C7,'times_s'),return;end
Pcell=route.C7.control_points;t=double(route.C7.times_s(:)');
ns=numel(Pcell);
if ns<1||numel(t)~=ns+1||any(~isfinite(t))||any(diff(t)<=0)||...
 ~isscalar(options.max_leaves)||options.max_leaves<1||...
 fix(options.max_leaves)~=options.max_leaves,return;end
J=AF7R58R168_JET_AUDIT(route);
if J.segments~=ns||any(~isfinite(J.max_relative_jump_by_order))
 Y.status='UNKNOWN_POSITION_C7_JET_AUDIT';return;
end
Y.position_C7_numeric_join_residual=max(J.max_relative_jump_by_order);
if Y.position_C7_numeric_join_residual>1e-6
 Y.status='REJECT_POSITION_C7_JUMP';return;
end
beg=cell(1,ns);fin=cell(1,ns);samples=cell(1,ns);
minCoef=Inf;used=0;prev=NaN;
for s=1:ns
 P=double(Pcell{s});if isequal(size(P),[16 3]),P=P';end
 if ~isequal(size(P),[3 16])||any(~isfinite(P(:))),return;end
 h=t(s+1)-t(s); V=15/h*diff(P,1,2);
 todo={V};leafCount=0;segmentTurn=0;
 while ~isempty(todo)
  cur=todo{end};todo(end)=[];leafCount=leafCount+1;
  [coefs,good]=local_squared_speed_coefs(cur);
  if good
   % Each accepted leaf must also lie in one open velocity half-plane.
   % The angle increment is then unambiguous (no hidden 2*pi turn).
   dir=mean(cur(1:2,:),2); dnorm=norm(dir);
   if dnorm>0
    dir=dir/dnorm; proj=dir'*cur(1:2,:);
    if min(proj)>4096*eps(max(1,max(abs(cur(:)))))
     wA=complex(cur(1,1),cur(2,1));
     wB=complex(cur(1,end),cur(2,end));
     segmentTurn=segmentTurn+angle(wB*conj(wA));
     minCoef=min(minCoef,min(coefs));continue;
    end
   end
  end
  if leafCount+numel(todo)+2>options.max_leaves
   Y.status='UNKNOWN_HEADING_ZERO_OR_BERNSTEIN_SPLIT_BUDGET';
   Y.segment_count=s;Y.leaves=used+leafCount;return;
  end
  [a,b]=local_split(cur);
  todo{end+1}=b;todo{end+1}=a; %#ok<AGROW>
 end
 used=used+leafCount;
 w0=local_velocity_jets(P,h,0);w1=local_velocity_jets(P,h,1);
 st=local_log_yaw_jets(w0);en=local_log_yaw_jets(w1);
 if isnan(prev)
  st(1)=atan2(imag(w0(1)),real(w0(1)));
 else
  st(1)=st(1)+2*pi*round((prev-st(1))/(2*pi));
 end
 en(1)=st(1)+segmentTurn;
 if ~isnan(prev)&&abs(st(1)-prev)>1e-6
  Y.status='UNKNOWN_HEADING_BRANCH_LIFT_INCONSISTENCY';return;
 end
 % Also compute samples unwrapped continuously, without assuming one branch
 u=linspace(0,1,101);ang=zeros(1,numel(u));
 for j=1:numel(u)
  v=local_eval_derivative(P,h,1,u(j));
  ang(j)=atan2(v(2),v(1));
 end
 ang=unwrap(ang);
 ang=ang+2*pi*round((st(1)-ang(1))/(2*pi));
 % No winding upper limit; evaluate unwrapping at 101 samples for display
 % only. The certificate is the strictly-positive speed and C7 jets.
 beg{s}=st;fin{s}=en;samples{s}=[t(s)+u*h;ang];prev=en(1);
end
res=0;
for s=1:ns-1
 a=fin{s};b=beg{s+1};
 res=max(res,max(abs(a-b)./max([ones(7,1),abs(a),abs(b)],[],2)));
end
Y.yaw_start_jets=beg;Y.yaw_end_jets=fin;Y.yaw_samples=samples;
Y.max_numeric_yaw_join_relative=res;Y.segment_count=ns;
Y.leaves=used;Y.min_squared_speed_bernstein_coefficient=minCoef;
Y.positive_horizontal_speed_bernstein=true;
Y.zero_speed_or_unresolved=false;
if res<=1e-6
 Y.heading_exact_C6_conditional=true;
 Y.status='CONDITIONAL_HEADING_EXACT_YAW_C6_BY_C7_AND_NONZERO_XY_SPEED';
else
 Y.status='UNKNOWN_YAW_JET_JUMP_AFTER_BRANCH_LIFT';
end
end
function [c,good]=local_squared_speed_coefs(V)
% Bernstein product: degree 14 x degree 14 -> degree 28.
n=size(V,2)-1;choose=arrayfun(@(i)nchoosek(n,i),0:n);
choose2=arrayfun(@(i)nchoosek(2*n,i),0:2*n);
c=zeros(1,2*n+1);
for i=0:n
 for j=0:n
  k=i+j;
  c(k+1)=c(k+1)+choose(i+1)*choose(j+1)/choose2(k+1)*...
   (V(1,i+1)*V(1,j+1)+V(2,i+1)*V(2,j+1));
 end
end
scale=max(1,max(abs(c)));
good=min(c)>4096*eps(scale);
end
function [a,b]=local_split(P)
n=size(P,2);X=P;a=zeros(size(P));b=zeros(size(P));
a(:,1)=P(:,1);b(:,end)=P(:,end);
for k=2:n
 X=(X(:,1:end-1)+X(:,2:end))/2;
 a(:,k)=X(:,1);b(:,n-k+1)=X(:,end);
end
end
function w=local_velocity_jets(P,T,u)
w=zeros(7,1);
for k=0:6
 v=local_eval_derivative(P,T,k+1,u);
 w(k+1)=complex(v(1),v(2));
end
end
function j=local_log_yaw_jets(w)
% If f=exp(g), n f_n = sum(k*g_k*f_{n-k}), f_n=w^(n)/n!.
f=zeros(7,1);g=zeros(7,1);
for k=0:6,f(k+1)=w(k+1)/factorial(k);end
g(1)=log(f(1));
for n=1:6
 sumv=0;
 for k=1:n-1,sumv=sumv+k*g(k+1)*f(n-k+1);end
 g(n+1)=(n*f(n+1)-sumv)/(n*f(1));
end
j=zeros(7,1);j(1)=imag(g(1));
for k=1:6,j(k+1)=factorial(k)*imag(g(k+1));end
end
function v=local_eval_derivative(P,T,k,u)
for j=1:k,P=(16-j)/T*diff(P,1,2);end
while size(P,2)>1,P=(1-u)*P(:,1:end-1)+u*P(:,2:end);end
v=P(:,1);
end

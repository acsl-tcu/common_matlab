function W=TRAJECTORY_C9_25D_LEAF_HULL_ROWS(req,shapes,oracle,rad)
%TRAJECTORY_C9_25D_LEAF_HULL_ROWS  All-time witness -> SAME 25D QP.
% A FAILED continuous-time oracle supplies a physical leaf [ta,tb].
% For PAYLOAD-only unresolved witnesses, de Casteljau restriction gives
% the EXACT Bernstein convex hull of the WHOLE physical time leaf.
% Every payload control on the leaf is constrained in one common 3D
% supporting halfspace of the TRUE observed shape's convex hull.
% d/dT includes both endpoints' physical-time phase drift, not just
% fixed-u variations. Search tangent only, not a certificate or a new
% planning stage. UAV and taut cable remain mandatory for the final proof.
W=struct('status','NO_PAYLOAD_LEAF_WITNESS', ...
    'A',zeros(25,0),'b',zeros(0,1),'rows',0, ...
    'physical_time_derivative_used',false,'safe_to_commit',false);
if ~(isstruct(oracle)&&isscalar(oracle)&& ...
        all(isfield(oracle,{'unresolved_time_interval_s', ...
        'unresolved_obstacle_ids','unresolved_body_indices'})))
    return
end
if ~any(double(oracle.unresolved_body_indices(:))==3)
    return
end
ids=double(oracle.unresolved_obstacle_ids(:)).';
if isempty(ids),return;end
leaf=double(oracle.unresolved_time_interval_s(:));
if ~(numel(leaf)==2&&all(isfinite(leaf))&&leaf(2)>leaf(1))
    W.status='UNKNOWN_C9_25D_LEAF_TIME_INVALID';return
end
T=double(req.duration_s);t0=double(req.t0_s);
if ~(leaf(1)>=t0&&leaf(2)<=t0+T)
    W.status='UNKNOWN_C9_25D_LEAF_OUTSIDE_CURRENT_PIECE';return
end
a=(leaf(1)-t0)/T;b=(leaf(2)-t0)/T;
if ~(a>=0&&b<=1&&a<b)
    W.status='UNKNOWN_C9_25D_LEAF_NORMALIZATION';return
end
[P,Pd]=TRAJECTORY_C9_27_SINGLE_PIECE_BASIS( ...
    req.start_jets,req.end_jets,req.nominal_free27,T);
[C,~]=restriction_transform(28,a,b);
Q=P*C;
% Phase sensitivity for FIXED PHYSICAL interval:
% d Q_i/d a = (n-i)/n * restricted(p'_u)_{i},
% d Q_i/d b = i/n * restricted(p'_u)_{i-1}.
Dp=27*diff(P,1,2);
[Cd,~]=restriction_transform(27,a,b);
Dsub=Dp*Cd;
Da=zeros(3,28);Db=zeros(3,28);
for i=0:27
    if i<27,Da(:,i+1)=(27-i)/27*Dsub(:,i+1);end
    if i>0,Db(:,i+1)=i/27*Dsub(:,i);end
end
QT=Pd*C-(a/T)*Da-(b/T)*Db;
margin=double(req.search_options.min_separation_m)+ ...
    double(req.geometry_options.numerical_guard_m);
A=zeros(25,28*numel(ids));rhs=zeros(28*numel(ids),1);m=0;
for j=1:numel(shapes)
    shape=shapes(j);
    if ~ismember(double(shape.id),ids),continue;end
    v=mean(Q,2)-shape.center;
    nv=norm(v);
    if ~(isfinite(nv)&&nv>0)
        W.status='UNKNOWN_C9_25D_LEAF_NORMAL_UNDEFINED';return
    end
    n=v/nv;
    h=TRAJECTORY_OBSTACLE_SUPPORT.value(shape,n);
    threshold=h+double(rad(3))+margin;
    for i=1:28
        grad=zeros(25,1);
        for f=1:8
            grad(3*(f-1)+(1:3))=n*C(10+f,i);
        end
        grad(25)=n.'*QT(:,i);
        deficit=threshold-n.'*Q(:,i);
        if ~(isfinite(deficit)&&all(isfinite(grad)))
            W.status='UNKNOWN_C9_25D_LEAF_TANGENT_NONFINITE';return
        end
        if norm(grad)==0
            if deficit>0
                W.status='UNKNOWN_C9_25D_LEAF_CONTROL_UNMOVABLE';return
            end
            continue
        end
        m=m+1;A(:,m)=grad;rhs(m)=deficit;
    end
end
W.A=A(:,1:m);W.b=rhs(1:m);W.rows=m;
W.physical_time_derivative_used=true;
W.status='LOCAL_C9_25D_PAYLOAD_FULLLEAF_ROWS__REQUIRES_RECERTIFICATION';
end

function [C,n]=restriction_transform(np,a,b)
% de Casteljau restriction of every canonical basis control at once.
assert(np>=2&&np==fix(np)&&a>=0&&a<b&&b<=1, ...
    'C9_25D_LEAF:Domain','Valid subinterval of a Bernstein polynomial.');
n=np-1;
C=eye(np);
if a>0
    [~,C]=split(C,a);
end
if b<1
    local=(b-a)/(1-a);
    [C,~]=split(C,local);
end
end

function [left,right]=split(A,u)
n=size(A,2)-1;
levels=A;left=zeros(size(A));right=zeros(size(A));
left(:,1)=levels(:,1);right(:,end)=levels(:,end);
for i=1:n
    levels=(1-u)*levels(:,1:end-1)+u*levels(:,2:end);
    left(:,i+1)=levels(:,1);
    right(:,end-i)=levels(:,end);
end
end

function [ok,n,beta,calls,iters]=TRAJECTORY_C9_SUPPORT_SEPARATOR(H,S,margin,reference,maxiter)
% Find ANY 3-D separating normal (not a fixed bypass direction).
% H = columns of convex hull of one body; S = true shape hull support.
% Accepted result ALWAYS satisfies min(n'*H)>support_O(n)+margin.
% No convergence or no separator found => UNKNOWN, never 'collision'.
% Gilbert/Frank-Wolfe only supplies normals, exact support checks certify.
assert(isnumeric(H)&&isreal(H)&&size(H,1)==3&&size(H,2)>=1&& ...
    all(isfinite(H(:)))&&isfinite(margin)&&margin>=0&& ...
    isnumeric(reference)&&isequal(size(reference),[3 1])&& ...
    all(isfinite(reference))&&isscalar(maxiter)&&maxiter>=0&& ...
    fix(maxiter)==maxiter,'TRAJECTORY_C9_SEPARATOR:Input', ...
    'Valid finite 3-D hull and caller-owned iteration budget required.');
ok=false;n=zeros(3,1);beta=NaN;calls=0;iters=0;
% These trial directions are data-dependent, covariant under arbitrary SO(3).
axes=[mean(H,2)-S.center,reference-S.center];
for k=1:size(axes,2)
    [ok,n,beta,used]=test_axis(axes(:,k),H,S,margin);
    calls=calls+used;
    if ok,return;end
end
% Zero centre displacement is resolved from geometry, never a world axis.
a=axes(:,1);
if norm(a)==0,a=axes(:,2);end
if norm(a)==0
    [~,ii]=max(sum((H-mean(H,2)).^2,1));
    a=H(:,ii)-S.center;
end
if norm(a)==0,return;end
[~,ii]=min(a.'*H);
[~,point]=TRAJECTORY_OBSTACLE_SUPPORT.value_and_point(S,a);
q=H(:,ii)-point;
for iter=1:maxiter
    iters=iter;
    [ok,n,beta,used]=test_axis(q,H,S,margin);
    calls=calls+used;
    if ok,return;end
    if ~(all(isfinite(q))&&norm(q)>0),return;end
    [~,idx]=min(q.'*H);
    [~,point]=TRAJECTORY_OBSTACLE_SUPPORT.value_and_point(S,q);
    d=H(:,idx)-point-q;
    d2=d.'*d;
    if ~(isfinite(d2)&&d2>0),return;end
    step=min(1,max(0,-(q.'*d)/d2));
    if ~(isfinite(step)&&step>0),return;end
    q=q+step*d;
end
[ok,n,beta,used]=test_axis(q,H,S,margin);
calls=calls+used;
end

function [ok,n,beta,calls]=test_axis(axis,H,S,margin)
ok=false;n=zeros(3,1);beta=NaN;calls=0;
scale=norm(axis);
if ~(isfinite(scale)&&scale>0),return;end
n=axis/scale;
h=TRAJECTORY_OBSTACLE_SUPPORT.value(S,n);
calls=1;beta=h+margin;
ok=isfinite(beta)&&min(n.'*H)>beta;
end

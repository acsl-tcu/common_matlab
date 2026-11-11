function [ok,n,beta,calls,iters,native_used]=TRAJECTORY_C9_SUPPORT_SEPARATOR(H,S,margin,reference,maxiter,use_native)
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
ok=false;n=zeros(3,1);beta=NaN;calls=0;iters=0;native_used=false;
if nargin<6,use_native=false;end
assert((isnumeric(use_native)||islogical(use_native))&& ...
    isscalar(use_native)&&isreal(use_native)&&isfinite(double(use_native))&& ...
    any(double(use_native)==[0 1]), ...
    'TRAJECTORY_C9_SEPARATOR:NativeFlag','Native proposal switch must be 0/1.');
% These trial directions are data-dependent, covariant under arbitrary SO(3).
axes=[mean(H,2)-S.center,reference-S.center];
for k=1:size(axes,2)
    [ok,n,beta,used]=test_axis(axes(:,k),H,S,margin);
    calls=calls+used;
    if ok,return;end
end
% Run native geometry only after the direct support-certified axes fail.
% This is a mathematical positive-certificate gate, not an empirical
% distance/iteration threshold. The canonical MATLAB proof still decides.
if use_native
    % Only a proposed normal is produced in native code.  Positive
    % certificates MUST be independently verified by the original MATLAB
    % canonical support function, not by the C++ floating point witness.
    folder=fileparts(mfilename('fullpath'));
    name='TRAJECTORY_C9_SEPARATOR_NORMAL_NATIVE';
    expected=fullfile(folder,[name '.' mexext]);
    % The explicit pre-Flight preflight validates the source signature.
    % Avoid a redundant signature MEX call inside every proof leaf.
    if exist(expected,'file')==3 && strcmpi(which(name),expected)
        try
            [direction,proposed,native_calls,native_iters]= ...
                TRAJECTORY_C9_SEPARATOR_NORMAL_NATIVE( ...
                    H,S,margin,reference,maxiter);
            native_used=true;
            % Native test-axis evaluations count as work even when the
            % proposed direction is not finally accepted by MATLAB.
            calls=double(native_calls);iters=double(native_iters);
            if proposed && isnumeric(direction)&&isequal(size(direction),[3 1]) ...
                    &&all(isfinite(direction))&&norm(direction)>0
                [verified,n,beta,used]=test_axis(direction,H,S,margin);
                calls=calls+used;
                if verified,ok=true;return;end
            end
        catch
            % The original MATLAB search remains available; a failed MEX
            % proposal never authorizes a positive safety assertion.
            native_used=false;calls=0;iters=0;
        end
    end
    % Native no-solution or rejected normal: restore original search,
    % including all generic directions, to avoid a coverage regression.
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

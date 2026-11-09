function R=TRAJECTORY_HEADING_CERTIFICATE(pieces,opts)
%TRAJECTORY_HEADING_CERTIFICATE  Prove horizontal speed never vanishes.
% If 0 is strictly separated from the convex hull of each Bernstein
% horizontal-velocity control polygon on every time subinterval, then
% vx(t)^2+vy(t)^2>0 for the FULL polynomial interval. Consequently,
% yaw(t)=atan2(vy,vx) has a continuous branch and is C6 when p is C7.
% The module evaluates orientation jets separately; the caller must keep
% a continuous 2*pi yaw lift during flight and across joined segments.
% No arbitrary minimum velocity is imposed. The caller owns a numerical
% separation guard and a subdivision budget.
assert(isstruct(pieces)&&~isempty(pieces)&&isstruct(opts)&&isscalar(opts)&& ...
    all(isfield(opts,{'max_subdivision_depth','numerical_guard_mps'}))&& ...
    isnumeric(opts.max_subdivision_depth)&&isscalar(opts.max_subdivision_depth)&& ...
    opts.max_subdivision_depth>=0&&isfinite(opts.max_subdivision_depth)&& ...
    fix(opts.max_subdivision_depth)==opts.max_subdivision_depth&& ...
    isnumeric(opts.numerical_guard_mps)&&isscalar(opts.numerical_guard_mps)&& ...
    isfinite(opts.numerical_guard_mps)&&opts.numerical_guard_mps>=0, ...
    'TRAJECTORY_HEADING:Authority','External proof budget and velocity guard required.');
failed=zeros(1,0);
for k=1:numel(pieces)
    p=pieces(k);
    assert(isfield(p,'control_points_m')&&isfield(p,'duration_s')&& ...
        isfinite(p.duration_s)&&p.duration_s>0, ...
        'TRAJECTORY_HEADING:Piece','Invalid polynomial.');
    P=p.control_points_m;
    assert(isnumeric(P)&&isreal(P)&&size(P,1)==3&&size(P,2)>=8&& ...
        all(isfinite(P(:))), ...
        'TRAJECTORY_HEADING:Piece','Need C7-capable polynomial controls.');
    n=size(P,2)-1;
    V=(n/p.duration_s)*diff(P(1:2,:),1,2);
    if ~nonnull(V,opts.max_subdivision_depth,opts.numerical_guard_mps)
        failed(end+1)=k; %#ok<AGROW>
    end
end
R=struct('status','UNKNOWN_HEADING_SINGULARITY_POSSIBLE', ...
    'certified',isempty(failed),'unknown_segment_indices',failed, ...
    'safe_to_commit',false, ...
    'meaning','Nonzero horizontal speed for full piece; yaw lift still required');
if isempty(failed),R.status='CERTIFIED_CONTINUOUS_NONZERO_HORIZONTAL_SPEED';end
end
function safe=nonnull(V,depth,guard)
% An explicit strictly positive projection excludes the 2-D origin.
C=[mean(V,2), V, [1;-0],[-1;0],[0;1],[0;-1]];
safe=false;
for j=1:size(C,2)
    n=C(:,j);d=norm(n);
    if d>0 && min((n/d).'*V)>guard
        safe=true;return;
    end
end
if depth==0,return;end
[A,B]=split_mid(V);
safe=nonnull(A,depth-1,guard)&&nonnull(B,depth-1,guard);
end
function [A,B]=split_mid(P)
n=size(P,2);v=P;A=zeros(size(P));B=zeros(size(P));
A(:,1)=v(:,1);B(:,n)=v(:,n);
for k=2:n
    v=(v(:,1:end-1)+v(:,2:end))*0.5;
    A(:,k)=v(:,1);B(:,n-k+1)=v(:,end);
end
end

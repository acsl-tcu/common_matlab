function [active,removed_ids,slacks_m]=TRAJECTORY_C7_CERTIFIED_OBSTACLE_EXCLUSION( ...
    shapes,start_jets,limits,duration_s,inflation)
%TRAJECTORY_C7_CERTIFIED_OBSTACLE_EXCLUSION Sound NLP constraint pruning.
% Given the externally supplied full-time speed bounds, every feasible
% UAV endpoint is in B(U0,V_u*T), and every feasible load endpoint is in
% B(L0,V_l*T). The taut straight cable is always in their convex hull.
% A single world-frame separating support plane proves that an observed
% obstacle cannot intersect ANY physically inflated body, on any candidate
% satisfying those same in-solve speed constraints. Only those obstacles
% may be removed from the NLP (not from sensor memory/final certificate).
% No geometric branch, no guessed speed, no obstacle-specific native model.
assert(isstruct(shapes)&&isstruct(start_jets)&& ...
    all(isfield(start_jets,{'uav','payload'}))&& ...
    isstruct(limits)&&all(isfield(limits,{'uav','payload'}))&& ...
    isfinite(duration_s)&&isscalar(duration_s)&&duration_s>0&& ...
    isnumeric(inflation)&&numel(inflation)==3&& ...
    all(isfinite(inflation(:)))&&all(inflation(:)>=0), ...
    'TRAJECTORY_C7_CERTIFIED_OBSTACLE_EXCLUSION:Input', ...
    'Full-time external geometry and speed data are required.');
U=double(start_jets.uav(:,1));L=double(start_jets.payload(:,1));
vu=double(limits.uav.derivative_norm_max(1));
vl=double(limits.payload.derivative_norm_max(1));
assert(isequal(size(U),[3,1])&&isequal(size(L),[3,1])&& ...
    all(isfinite([U;L]))&&isscalar(vu)&&isscalar(vl)&& ...
    all(isfinite([vu vl]))&&vu>=0&&vl>=0, ...
    'TRAJECTORY_C7_CERTIFIED_OBSTACLE_EXCLUSION:Speed', ...
    'No invented default or invalid speed is allowed.');
keep=true(1,numel(shapes));removed_ids=zeros(1,0);slacks_m=zeros(1,0);
D=L-U;dd=D.'*D;
for j=1:numel(shapes)
    center=double(shapes(j).center(:));
    alpha=0;
    if dd>0,alpha=max(0,min(1,(center-U).'*D/dd));end
    near=U+alpha*D;
    n=near-center;
    mag=norm(n);
    if ~(isfinite(mag)&&mag>0),continue,end
    n=n/mag;
    h=TRAJECTORY_OBSTACLE_SUPPORT.value(shapes(j),n);
    % Bound all possible cable points by the convex hull of endpoint
    % reachability balls. max(inflation) is only used to safely PRUNE;
    % it is never used for the final body geometry or corridor.
    margin=min(n.'*U-vu*duration_s, n.'*L-vl*duration_s) ...
        -h-max(inflation);
    if isfinite(margin)&&margin>0
        keep(j)=false;
        removed_ids(end+1)=shapes(j).id; %#ok<AGROW>
        slacks_m(end+1)=margin; %#ok<AGROW>
    end
end
active=shapes(keep);
end

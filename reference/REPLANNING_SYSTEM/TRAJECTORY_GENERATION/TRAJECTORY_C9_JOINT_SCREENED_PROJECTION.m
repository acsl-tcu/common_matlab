function [x,R]=TRAJECTORY_C9_JOINT_SCREENED_PROJECTION(A,b,max_pivots,tolerance)
% TRAJECTORY_C9_JOINT_SCREENED_PROJECTION
% Solve EXACTLY THE SAME single strictly convex 12-variable QP
%     min 1/2*||x||^2   subject to ALL rows A'*x>=b.
% This is constraint generation INSIDE ONE QP, not another trajectory
% planning stage or an obstacle-by-obstacle superposition. First solve
% with the constraints violated at x=0, then check EVERY supplied row.
% Add ALL newly violated rows and resolve until the FULL set is satisfied.
% Mathematical equivalence: an optimal restricted-QP solution satisfying
% every omitted inequality is also optimal for the full feasible set.
% Every finite iteration, rank or KKT failure returns UNKNOWN, and the
% final trajectory still needs the independent all-time C9 certificate.
% A completely initially satisfied plane is NOT ignored permanently.
assert(isnumeric(A)&&isreal(A)&&ismatrix(A)&& ...
    all(isfinite(A(:)))&&isnumeric(b)&&isreal(b)&& ...
    iscolumn(b)&&numel(b)==size(A,2)&&all(isfinite(b))&& ...
    isnumeric(max_pivots)&&isscalar(max_pivots)&& ...
    isfinite(max_pivots)&&max_pivots>=1&&max_pivots==fix(max_pivots)&& ...
    isnumeric(tolerance)&&isscalar(tolerance)&& ...
    isfinite(tolerance)&&tolerance>=0, ...
    'TRAJECTORY_C9_SCREENED_QP:Input', ...
    'Finite simultaneous QP planes, tolerance and pivot budget required.');
[n,m]=size(A);x=zeros(n,1);
R=struct('solved',false,'status','UNKNOWN_NOT_STARTED', ...
    'pivots',0,'active_plane_count',0, ...
    'maximum_primal_violation',max([0;b]), ...
    'kkt_stationarity',NaN,'full_plane_count',m, ...
    'working_plane_count',0,'constraint_generation_passes',0, ...
    'safe_to_commit',false);
selected=find(b>tolerance);
if isempty(selected)
    R.solved=true;R.status='SOLVED_QP_KKT__TRAJECTORY_PROOF_REQUIRED';
    R.maximum_primal_violation=max([0;b]);R.kkt_stationarity=0;
    return
end
while R.pivots<max_pivots
    R.constraint_generation_passes=R.constraint_generation_passes+1;
    [trial,sub]=TRAJECTORY_C9_JOINT_PROJECTION( ...
        A(:,selected),b(selected),max_pivots-R.pivots,tolerance);
    R.pivots=R.pivots+sub.pivots;
    R.active_plane_count=sub.active_plane_count;
    R.kkt_stationarity=sub.kkt_stationarity;
    R.working_plane_count=numel(selected);
    R.status=sub.status;
    x=trial;
    if ~sub.solved
        R.status=['UNKNOWN_SCREENED_QP_' sub.status];
        R.maximum_primal_violation=max([0;b-A.'*x]);
        return
    end
    violation=b-A.'*x;
    R.maximum_primal_violation=max([0;violation]);
    if R.maximum_primal_violation<=tolerance
        R.solved=true;
        R.status='SOLVED_QP_KKT__TRAJECTORY_PROOF_REQUIRED';
        return
    end
    % All omitted, newly violated rows enter the SAME simultaneous QP.
    % A prior row never leaves the working set. Failure to make progress
    % cannot be misclassified as a feasible QP or a collision.
    missing=find(violation>tolerance);
    new=missing(~ismember(missing,selected));
    if isempty(new)
        R.status='UNKNOWN_SCREENED_QP_NO_NEW_VIOLATED_ROW';return
    end
    selected=sort([selected(:);new(:)]);
end
R.status='UNKNOWN_SCREENED_QP_GLOBAL_PIVOT_BUDGET';
end

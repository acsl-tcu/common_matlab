function [x,R]=TRAJECTORY_C9_RELAXED_SEARCH_DIRECTION(A,b,max_iterations)
% TRAJECTORY_C9_RELAXED_SEARCH_DIRECTION  Convex search direction ONLY.
%
% The simultaneous local separating halfspaces A'*x>=b can be mutually
% inconsistent even when the ORIGINAL nonlinear 3-D obstacle-avoidance
% problem admits a safe path. A hard-QP failure must not be mislabeled a
% physical collision or a proof of trajectory infeasibility.
%
% Solve the *weight-free*, all-planes squared-hinge residual problem
%
%      min_x  1/2 * ||max(b-A'*x,0)||_2^2
%
% by deterministic Lipschitz gradient descent. This has no obstacle-
% specific displacement, no prescribed direction, no tunable penalty or
% made-up physical bound. The Lipschitz constant of the exact gradient is
% ||A||_2^2; the step is 1/L. The caller owns the finite iteration budget.
% The best residual-reducing iterate is returned without any implication
% that the linearized inequalities are feasible or that the real cable,
% UAV or load avoids any obstacle. ONLY the independent, full-time C9
% geometry / tautness / tension / thrust proof can certify a candidate.
%
% NO HLC COMMIT. This is part of the SAME 12-control simultaneous search,
% not a second trajectory-generation stage.
assert(isnumeric(A)&&isreal(A)&&ismatrix(A)&&all(isfinite(A(:)))&& ...
    isnumeric(b)&&isreal(b)&&iscolumn(b)&&size(A,2)==numel(b)&& ...
    all(isfinite(b))&&isnumeric(max_iterations)&&isreal(max_iterations)&& ...
    isscalar(max_iterations)&&isfinite(max_iterations)&& ...
    max_iterations>=1&&max_iterations==fix(max_iterations), ...
    'TRAJECTORY_C9_RELAXED:Input','Finite linearized constraints and budget required.');
n=size(A,1);x=zeros(n,1);
r0=max(b,0);f0=0.5*(r0.'*r0);
R=struct('status','UNKNOWN_NO_SEARCH_DIRECTION', ...
    'direction_available',false,'linearized_feasibility_proven',false, ...
    'safe_to_commit',false,'iterations',0, ...
    'hinge_objective_before_m2',f0, ...
    'hinge_objective_after_m2',f0, ...
    'maximum_linearized_violation_m',max([0;r0]), ...
    'scope','ONLY_CONVEX_PROBE_RESIDUAL__NOT_A_COLLISION_CERTIFICATE');
if isempty(A)||f0==0,return;end
H=A*A.';
H=0.5*(H+H.');
eigvals=eig(H);
L=max(real(eigvals));
if ~(isfinite(L)&&L>0),R.status='UNKNOWN_ZERO_OR_NONFINITE_GRADIENT';return;end
best=f0;bestx=x;
for k=1:max_iterations
    r=max(b-A.'*x,0);
    candidate=x+(A*r)/L;
    if ~all(isfinite(candidate))
        R.status='UNKNOWN_NONFINITE_ITERATE';return
    end
    rc=max(b-A.'*candidate,0);
    fc=0.5*(rc.'*rc);
    if ~isfinite(fc)
        R.status='UNKNOWN_NONFINITE_RESIDUAL';return
    end
    if fc<=best
        best=fc;bestx=candidate;
    end
    x=candidate;
    R.iterations=k;
end
x=bestx;
R.hinge_objective_after_m2=best;
R.maximum_linearized_violation_m=max([0;max(b-A.'*x,0)]);
% Scale this rejection-only numerical test with the objective itself.
% It introduces no physical clearance or safety margin.
roundoff=64*eps(max(1,f0));
R.direction_available=(best<f0-roundoff)&&any(x~=0);
if R.direction_available
    R.status='UNTRUSTED_RELAXED_PROBE_DIRECTION__INDEPENDENT_PROOF_REQUIRED';
else
    R.status='UNKNOWN_NO_RESIDUAL_REDUCTION';
end
end

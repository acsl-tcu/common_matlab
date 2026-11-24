function result = CBF_REAL_D2T_SOLVE(rows,nominal,weights)
% Research-only 3-variable hard QP. Does not apply the input to the plant.
% The thrust limit gives finite, state-dependent ddT bounds each hold.
% Missing actuator ddT bounds is not a reason to block a mathematical QP,
% but such a solution is NOT certified physically achievable.
% Requires quadprog (Optimization Toolbox). NO unverified fallback.
validateattributes(nominal,{'double'},{'vector','numel',3,'finite','real'});
validateattributes(weights,{'double'},{'vector','numel',3,'positive','finite'});
result=struct('success',false,'status','NOT_SOLVED','v',[], ...
    'minimumResidual',NaN,'exitflag',NaN,'elapsedMs',NaN);
if ~strcmp(rows.status,'ROWS_CONSTRUCTED')
    result.status=['BLOCKED_' rows.status];return;
end
if exist('quadprog','file')~=2
    result.status='QUADPROG_NOT_AVAILABLE';return;
end
tStart=tic;
W=diag(weights(:));nominal=nominal(:);
try
    opt=optimoptions('quadprog','Display','off');
    [v,~,flag]=quadprog(W,-W*nominal,-rows.A,-rows.b, ...
        [],[],[],[],[],opt);
catch ME
    result.status=['SOLVER_ERROR_' ME.identifier];
    result.elapsedMs=1000*toc(tStart);return
end
result.elapsedMs=1000*toc(tStart);result.exitflag=flag;
if isempty(v) || flag<=0 || any(~isfinite(v))
    result.status='INFEASIBLE_OR_NOT_CONVERGED';return
end
result.minimumResidual=min(rows.A*v-rows.b);
if result.minimumResidual < -1e-8
    result.status='NUMERICAL_CONSTRAINT_FAILURE';return
end
result.v=v;result.success=true;
result.status='POINTWISE_HOCBF_QP_FEASIBLE_NOT_SAMPLED_SAFE';
end

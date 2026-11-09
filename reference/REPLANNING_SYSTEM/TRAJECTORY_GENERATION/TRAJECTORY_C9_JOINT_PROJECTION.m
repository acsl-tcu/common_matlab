function [x,R]=TRAJECTORY_C9_JOINT_PROJECTION(A,b,max_pivots,feasibility_tolerance)
% TRAJECTORY_C9_JOINT_PROJECTION  Single simultaneous halfspace QP.
%    min_x 1/2*x'*x subject to A'*x >= b
%
% In the payload trajectory this is the exact minimum integrated squared
% acceleration-change QP after Cholesky whitening. All obstacles and all
% three body components share ONE x. There is no per-obstacle planning,
% fixed avoidance direction, or separate planning stage.
%
% Dual active-set method: x=A*lambda, lambda>=0. An unsatisfied row enters
% the working set and its multiplier grows; previously active multipliers
% decrease until their constraint leaves or the new constraint binds.
% Economic QR projection avoids normal-equation conditioning. The result
% is ONLY a sampled linearized search step: NEVER a safety certificate.
% Any rank/iteration/numerical failure returns unsolved. A separate exact
% continuous-time all-body/tension/thrust certificate must be run later.
assert(isnumeric(A)&&isreal(A)&&ismatrix(A)&&all(isfinite(A(:)))&& ...
    isnumeric(b)&&isreal(b)&&iscolumn(b)&& ...
    size(A,2)==numel(b)&&all(isfinite(b))&& ...
    isnumeric(max_pivots)&&isscalar(max_pivots)&& ...
    isfinite(max_pivots)&&max_pivots==fix(max_pivots)&&max_pivots>=1&& ...
    isnumeric(feasibility_tolerance)&&isscalar(feasibility_tolerance)&& ...
    isfinite(feasibility_tolerance)&&feasibility_tolerance>=0, ...
    'TRAJECTORY_C9_QP:Input','Finite simultaneous constraints and budgets required.');
[n,m]=size(A);
x=zeros(n,1);
R=struct('solved',false,'status','UNKNOWN_NOT_STARTED', ...
    'pivots',0,'active_plane_count',0, ...
    'maximum_primal_violation',max([0;b]),'kkt_stationarity',NaN);
if m==0
    R.solved=true;R.status='SOLVED_QP_KKT__TRAJECTORY_PROOF_REQUIRED';
    R.maximum_primal_violation=0;R.kkt_stationarity=0;return
end
lambda=zeros(m,1);
active=zeros(1,0);
% This protects numerical QR rank checks and KKT evaluation, NOT geometry.
roundoff=64*eps(max(1,norm(A,'fro')+norm(b)));
while R.pivots<max_pivots
    violation=b-A.'*x;
    [worst,p]=max(violation);
    if worst<=feasibility_tolerance
        residual=norm(x-A*lambda,inf);
        dual=min(lambda);
        complementarity=max(abs(lambda.*violation));
        R.maximum_primal_violation=max(0,worst);
        R.kkt_stationarity=residual;
        R.active_plane_count=numel(active);
        if residual<=roundoff && dual>=-roundoff && ...
                complementarity<=roundoff + ...
                    feasibility_tolerance*max(1,norm(lambda,1))
            R.solved=true;
            R.status='SOLVED_QP_KKT__TRAJECTORY_PROOF_REQUIRED';
        else
            R.status='UNKNOWN_QP_KKT_RESIDUAL';
        end
        return
    end
    if any(active==p)
        R.status='UNKNOWN_QP_ACTIVE_ROW_VIOLATED';return
    end
    ap=A(:,p);
    pending=0;
    while R.pivots<max_pivots
        R.pivots=R.pivots+1;
        if isempty(active)
            coeff=zeros(0,1);d=ap;
        else
            [Q,T]=qr(A(:,active),0);
            qr_tolerance=max(n,numel(active))* ...
                eps(max(1,norm(A(:,active),'fro')));
            if any(abs(diag(T))<=qr_tolerance)
                R.status='UNKNOWN_QP_DEPENDENT_ACTIVE_PLANES';return
            end
            coeff=T\(Q.'*ap);
            d=ap-Q*(Q.'*ap);
        end
        denom=ap.'*d;
        residual=b(p)-ap.'*x;
        if denom>roundoff*max(1,ap.'*ap)
            fullstep=residual/denom;
        else
            fullstep=Inf;
        end
        limit=Inf;leaving=0;
        for j=1:numel(active)
            if coeff(j)>0
                proposed=lambda(active(j))/coeff(j);
                if proposed<limit
                    limit=proposed;leaving=j;
                end
            end
        end
        if ~isfinite(fullstep)&&~isfinite(limit)
            R.status='UNKNOWN_QP_INFEASIBLE_LINEARIZATION';return
        end
        if fullstep<=limit
            step=max(0,fullstep);
            if ~isfinite(step)
                R.status='UNKNOWN_QP_NONFINITE_PIVOT';return
            end
            if ~isempty(active)
                lambda(active)=lambda(active)-step*coeff;
            end
            pending=pending+step;
            x=x+step*d;
            lambda(p)=pending;
            active(end+1)=p; %#ok<AGROW>
            break
        end
        if leaving==0||~isfinite(limit)
            R.status='UNKNOWN_QP_NONFINITE_LEAVING_ROW';return
        end
        step=max(0,limit);
        lambda(active)=lambda(active)-step*coeff;
        pending=pending+step;
        x=x+step*d;
        lambda(active(leaving))=0;
        active(leaving)=[];
    end
end
R.status='UNKNOWN_QP_ACTIVE_SET_PIVOT_BUDGET';
R.maximum_primal_violation=max(0,max(b-A.'*x));
R.kkt_stationarity=NaN;
R.active_plane_count=numel(active);
end

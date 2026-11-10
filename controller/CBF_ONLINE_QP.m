function [x,fval,exitflag] = CBF_ONLINE_QP(H,q,A,b,feasTol,lb)
% CBF_ONLINE_QP  Numerically robust solver for the EXISTING HARD CBF QPs.
% This is an ONLINE numerical optimizer, NOT a generated CBF derivative.
% It preserves the supplied quadratic objective, all inequalities, and
% bounds. The physical interpretation / model is NOT changed here.
%
% First solve the ORIGINAL QP. On failure only, center at its unconstrained
% minimizer and apply an EXACT invertible diagonal change of coordinates.
% If needed, obtain a feasible warm start by linear programming and retry
% quadprog with its active-set algorithm. An LP point alone is NEVER used
% as an actuator command. Any returned solution is verified in the
% ORIGINAL (unscaled) inequalities. No hard CBF constraint is relaxed.
%
% All solvers are in MATLAB Optimization Toolbox (quadprog, linprog).
% This function does NOT establish intersample safety or recursive feasibility.

n=numel(q);
q=q(:); b=b(:);
if nargin<5 || isempty(feasTol),feasTol=1e-7;end
if nargin<6 || isempty(lb),lb=-inf(n,1);end
lb=lb(:);
assert(isequal(size(H),[n,n]) && size(A,2)==n && ...
    size(A,1)==numel(b) && numel(lb)==n, ...
    'CBF:QPNumericShape','QP arrays have incompatible dimensions.');
assert(isfinite(feasTol) && feasTol>=0 && ...
    all(isfinite([H(:);q;A(:);b])) && ...
    all(isfinite(lb) | isinf(lb) & lb<0), ...
    'CBF:QPNumericNonfinite','QP coefficients or bounds invalid.');
H=(H+H.')/2;
assert(all(diag(H)>0),'CBF:QPNumericHessian', ...
    'This solver requires a positive definite tracking Hessian.');

x=nan(n,1);fval=NaN;exitflag=-999;
optDirect=optimoptions('quadprog','Display','off');
try
    [v,~,ef]=quadprog(H,q,A,b,[],[],lb,[],[],optDirect);
    exitflag=ef;
    if localAccepted(v,ef,A,b,lb,feasTol)
        x=v(:);fval=localCost(H,q,x);return;
    end
catch
    % A numerically unsuccessful direct solve is retried below.
end

% Exact reformulation: x=xCenter+diag(d)*z.
% The unconstrained linear term vanishes in the new coordinates; the
% objective differs only by an additive constant. No controller gain changes.
xCenter=-(H\q);
d=1./sqrt(diag(H));
if ~all(isfinite(xCenter)) || ~all(isfinite(d)) || any(d<=0)
    return
end
AD=bsxfun(@times,A,d.');
bShift=b-A*xCenter;
Hd=bsxfun(@times,bsxfun(@times,H,d),d.');
Hd=(Hd+Hd.')/2;
% Positive ROW scaling does not alter the feasible region.
r=max([ones(size(A,1),1),sqrt(sum(AD.^2,2)),abs(bShift)],[],2);
An=bsxfun(@rdivide,AD,r);
bn=bShift./r;
lbn=(lb-xCenter)./d;
if any(~isfinite([Hd(:);An(:);bn])) || any(isnan(lbn))
    return
end

% Retry using the well-centered objective and row-equilibrated inequalities.
try
    options=optimoptions('quadprog','Display','off', ...
        'Algorithm','interior-point-convex');
    [z,~,ef]=quadprog(Hd,zeros(n,1),An,bn,[],[],lbn,[],[],options);
    if localAccepted(z,ef,An,bn,lbn,feasTol) && ...
            localAccepted(xCenter+d.*z,ef,A,b,lb,feasTol)
        x=xCenter+d.*z;exitflag=ef;fval=localCost(H,q,x);return;
    end
catch
    % Proceed to a feasible-start active-set attempt.
end

% A feasible LP warm start helps quadprog handle very small CBF gradients.
% It is used strictly as x0 to optimize the ORIGINAL objective, NEVER as
% a final command. Infeasible hard rows remain infeasible.
try
    lpOpt=optimoptions('linprog','Display','off');
    [z0,~,lpEF]=linprog(zeros(n,1),An,bn,[],[],lbn,[],lpOpt);
    if ~localAccepted(z0,lpEF,An,bn,lbn,feasTol)
        return
    end
    optAS=optimoptions('quadprog','Display','off','Algorithm','active-set');
    [z,~,ef]=quadprog(Hd,zeros(n,1),An,bn,[],[],lbn,[],z0,optAS);
    if localAccepted(z,ef,An,bn,lbn,feasTol) && ...
            localAccepted(xCenter+d.*z,ef,A,b,lb,feasTol)
        x=xCenter+d.*z;exitflag=ef;fval=localCost(H,q,x);
        return
    end
catch
    % Fail closed. Never return a feasible-but-unoptimized LP command.
end
end

function yes=localAccepted(x,ef,A,b,lb,tol)
yes=ef>0 && numel(x)==numel(lb) && ...
    all(isfinite(x(:))) && ...
    all(A*x(:)<=b+tol) && all(x(:)>=lb-tol);
end

function v=localCost(H,q,x)
v=0.5*x.'*H*x+q.'*x;
end

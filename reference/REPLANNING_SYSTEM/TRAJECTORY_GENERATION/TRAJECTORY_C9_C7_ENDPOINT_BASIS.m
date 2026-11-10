function [P, dP_dT, Jfree] = TRAJECTORY_C9_C7_ENDPOINT_BASIS(J0, J1, X, T, end_order)
%TRAJECTORY_C9_C7_ENDPOINT_BASIS One segment, degree 23, exact endpoint jets.
%   [P,dP_dT,Jfree] = TRAJECTORY_C9_C7_ENDPOINT_BASIS(J0,J1,X,T,end_order)
%
%   J0,J1: 3-by-(end_order+1) PHYSICAL time derivatives [p,p',...,p^(r)]
%   X:     3-by-(24-2*(end_order+1)) free interior Bernstein controls
%   T:     strictly positive physical duration (s)
%   end_order: 7 (C7 at joins) or 9 (legacy C9 endpoint restriction)
%
%   P:      3-by-24 degree-23 Bernstein control points (one continuous piece)
%   dP_dT:  3-by-24 analytic partial derivative at FIXED J0,J1,X
%   Jfree:  optional sparse 72-by-numel(X) derivative d vec(P)/d vec(X)
%
%   The parameterization does NOT certify collision, yaw regularity, thrust,
%   torque, tracking, tautness, numerical directed rounding, or flight safety.
%   It is an independent REPRESENTATION primitive, not a planner and not a
%   replacement for the existing validated strict acceptance gates.
%
%   The endpoint formula follows exact real Bernstein differentiation:
%   P_j = sum_{k=0}^j nchoosek(j,k)*T^k/(n)_k * J0_k,
%   P_(n-j) = sum_{k=0}^j (-1)^k*nchoosek(j,k)*T^k/(n)_k * J1_k,
%   where (n)_k=n*(n-1)*...*(n-k+1), n=23.
%   This exact formula assumes endpoint physical jets stay FIXED as T changes.
%   If endpoint jets depend on T, the caller must include their chain rule.
%
%   end_order=7 preserves derivatives 0..7 (C7) but does not guarantee yaw
%   C6 if horizontal velocity becomes zero; the strict yaw oracle must prove
%   heading nondegeneracy or certify a valid stationary-heading branch.
%
%   Flight-dependent mass/geometry/HLC bounds are intentionally not inputs.
%   No Flight-dependent state is cached; the persistent coefficient matrix is
%   a flight-invariant combinatorial constant. There is no MEX requirement.
%
%   RESEARCH PREVIEW: do NOT substitute this representation into the live
%   13D optimizer without updating its coefficient count, Jacobians, QP,
%   C7/yaw and all-time torque/collision proof interfaces as one revision.

n = 23;
if nargin < 5 || isempty(end_order)
    end_order = 7;
end
if ~isscalar(end_order) || ~(end_order == 7 || end_order == 9)
    error('C9C7:EndOrder', 'end_order must be 7 or 9.');
end
r = double(end_order);
q = n+1-2*(r+1);
if ~isa(J0,'double') || ~isreal(J0) || ~isequal(size(J0),[3,r+1]) || any(~isfinite(J0(:))) || ...
   ~isa(J1,'double') || ~isreal(J1) || ~isequal(size(J1),[3,r+1]) || any(~isfinite(J1(:))) || ...
   ~isa(X,'double') || ~isreal(X) || ~isequal(size(X),[3,q]) || any(~isfinite(X(:))) || ...
   ~isa(T,'double') || ~isreal(T) || ~isscalar(T) || ~isfinite(T) || T <= 0
    error('C9C7:InvalidInput', 'Finite double inputs with valid sizes and T>0 are required.');
end

persistent A7 A9
if r == 7
    if isempty(A7), A7 = build_endpoint_coefficients(n,7); end
    A = A7;
else
    if isempty(A9), A9 = build_endpoint_coefficients(n,9); end
    A = A9;
end

powers = T.^(0:r);
if any(~isfinite(powers))
    error('C9C7:Overflow', 'Duration powers are not finite.');
end
M = bsxfun(@times,A,powers);
sign_end = (-1).^(0:r);
ME = bsxfun(@times,M,sign_end);

P = zeros(3,n+1);
P(:,1:r+1) = J0*M.';
P(:,n-r+1:n+1) = fliplr(J1*ME.');
free_indices = (r+2):(n-r);
P(:,free_indices) = X;

if nargout > 1
    power_deriv = [0, (1:r).*T.^(0:r-1)];
    MD = bsxfun(@times,A,power_deriv);
    MDE = bsxfun(@times,MD,sign_end);
    dP_dT = zeros(3,n+1);
    dP_dT(:,1:r+1) = J0*MD.';
    dP_dT(:,n-r+1:n+1) = fliplr(J1*MDE.');
else
    dP_dT = [];
end

if nargout > 2
    rows = zeros(1,3*q);
    cols = 1:(3*q);
    for j = 1:q
        ii = (3*j-2):(3*j);
        rows(ii) = 3*(free_indices(j)-1)+(1:3);
    end
    Jfree = sparse(rows,cols,ones(1,3*q),3*(n+1),3*q);
else
    Jfree = [];
end
end

function A = build_endpoint_coefficients(n,r)
A = zeros(r+1,r+1);
fall = 1;
for k=0:r
    if k > 0
        fall = fall*(n-k+1);
    end
    for j=k:r
        A(j+1,k+1) = nchoosek(j,k)/fall;
    end
end
end

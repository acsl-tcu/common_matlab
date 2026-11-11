function [left,right]=TRAJECTORY_C9_BERNSTEIN_MIDPOINT_SPLIT(P)
%TRAJECTORY_C9_BERNSTEIN_MIDPOINT_SPLIT Exact real-arithmetic C9 subdivision.
% For degree d=n-1, the left-side control j is
%   L_j = sum_{i=0}^j C(j,i) 2^{-j} P_i,
% and R is its mirrored right-side counterpart. These are the same
% de Casteljau midpoint controls IN REAL ARITHMETIC, so their convex hull
% enclosure and continuous-time proof meaning are unchanged.
%
% Precompute only the degree-dependent dyadic Pascal matrix; it depends
% on neither obstacle configuration nor Flight/mass/controller state.
% Degrees <=27 use one dense small matrix multiplication; any other input
% retains the original de Casteljau implementation. The degree-27 single-piece C9 extension also uses the cached matrix. This does not implement directed
% floating-point rounding; the existing numerical-certificate limitation
% remains, and this helper does not itself establish physical safety.
n=size(P,2);
assert(isnumeric(P)&&ismatrix(P)&&n>=1, ...
    'TRAJECTORY_C9_SPLIT:Input','Need a nonempty 2D numeric control matrix.');
if isa(P,'double') && n<=28
    persistent matrices
    if isempty(matrices),matrices=cell(1,28);end
    C=matrices{n};
    if isempty(C)
        % Integer binomial coefficients up to degree 24 are exact in
        % binary64. Division by 2^j is exact as a dyadic coefficient.
        L=zeros(n,n);
        for j=1:n
            for i=1:j
                L(i,j)=nchoosek(j-1,i-1)*2^(-(j-1));
            end
        end
        C=[L,flipud(fliplr(L))];
        matrices{n}=C;
    end
    Q=P*C;
    left=Q(:,1:n);
    right=Q(:,n+1:2*n);
else
    % Out-of-scope degree/class: preserve conventional de Casteljau.
    Q=P;
    left=zeros(size(P));right=zeros(size(P));
    left(:,1)=P(:,1);right(:,end)=P(:,end);
    for k=2:n
        Q=0.5*(Q(:,1:end-1)+Q(:,2:end));
        left(:,k)=Q(:,1);right(:,n-k+1)=Q(:,end);
    end
end
end

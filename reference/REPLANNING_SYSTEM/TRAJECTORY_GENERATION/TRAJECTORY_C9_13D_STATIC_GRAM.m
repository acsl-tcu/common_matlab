function [B,info]=TRAJECTORY_C9_13D_STATIC_GRAM(n,use_cache)
% Degree-only 13D C9 acceleration Gram cache; independent of Flight state.
% Identical gammaln expression and i/j loop order to the original local
% c9_13d_gram_matrix in TRAJECTORY_C9_13D_NATURAL_QP. The 21st degree
% is intrinsic to the 23rd-degree C9 payload acceleration representation.
% No obstacle, physical parameter, flight time or EKF state enters.
if nargin<2,use_cache=true;end
assert(isnumeric(n)&&isscalar(n)&&isreal(n)&&isfinite(n)&& ...
    n==fix(n)&&n>=0&&n<=32, ...
    'C9_GRAM:Degree','Expected nonnegative Bernstein degree <=32.');
assert((isnumeric(use_cache)||islogical(use_cache))&& ...
    isscalar(use_cache)&&any(double(use_cache)==[0 1]), ...
    'C9_GRAM:Mode','Cache option must be 0 or 1.');
persistent cached_degree cached_B
hit=logical(use_cache)&&isequal(cached_degree,n)&& ...
    isequal(size(cached_B),[n+1 n+1]);
if hit
    B=cached_B;
else
    B=zeros(n+1);
    for i=0:n
        for j=0:i
            logv=gammaln(n+1)-gammaln(i+1)-gammaln(n-i+1) + ...
                 gammaln(n+1)-gammaln(j+1)-gammaln(n-j+1) - ...
                 log(2*n+1) - gammaln(2*n+1) + ...
                 gammaln(i+j+1)+gammaln(2*n-i-j+1);
            B(i+1,j+1)=exp(logv);
            B(j+1,i+1)=B(i+1,j+1);
        end
    end
    if logical(use_cache)
        cached_degree=n;
        cached_B=B;
    end
end
if nargout>=2
    info=struct('cache_hit',hit,'degree',n, ...
        'static_only',true,'safe_to_commit',false);
end
end

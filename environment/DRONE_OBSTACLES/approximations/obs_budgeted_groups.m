function groups=obs_budgeted_groups(C,budget)
% Group adjacent source components into at most budget convex-cover groups.
% Each group stores a guaranteed AABB of ALL its members. For annular
% primitives, source components are ordered around the ring; grouping
% consecutive members generally preserves more of a large central opening.
% Other arbitrary meshes may not have a meaningful component order.
assert(isscalar(budget) && budget>=1 && mod(budget,1)==0, ...
    'budget must be a positive integer');
N=numel(C);
groups=struct('indices',{},'lo',{},'hi',{});
if N==0,return;end
K=min(N,budget);
for k=1:K
    a=floor((k-1)*N/K)+1;
    b=floor(k*N/K);
    indices=a:b;
    lo=[Inf Inf Inf];hi=[-Inf -Inf -Inf];
    for j=indices
        [cLo,cHi]=obs_component_bounds(C(j));
        lo=min(lo,cLo);hi=max(hi,cHi);
    end
    groups(k)=struct('indices',indices,'lo',lo,'hi',hi); %#ok<AGROW>
end
end

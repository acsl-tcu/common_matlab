function groups=obs_direct_cover_groups(C,options)
% Make up to maxPartsPerObstacle geometric groups WITHOUT AABB enclosures.
% 1. Build analytic support-plane outer covers of each source component.
% 2. Group consecutive components when the budget is smaller than their
%    number (important for ring-like hollow objects).
% 3. If budget remains, split a group's convex hull by its actual edges.
% 4. Fit MVEE to each group in obs_approx_multiple_ellipsoids.
%
% The intermediate convex hull can fill some concavities when the budget
% is tight, but it cannot omit any original obstacle volume.
N=numel(C);
budget=options.maxPartsPerObstacle;
assert(budget>=1 && mod(budget,1)==0);
verts=cell(1,N);
for i=1:N
    verts{i}=obs_ellipsoid_cover_vertices(C(i),options);
end
K=min(budget,N);
groups=struct('vertices',{},'componentIndex',{});
for k=1:K
    startIdx=floor((k-1)*N/K)+1;
    stopIdx=floor(k*N/K);
    V=vertcat(verts{startIdx:stopIdx});
    groups(end+1)=struct('vertices',unique(V,'rows'), ...
        'componentIndex',startIdx); %#ok<AGROW>
end

% Special meaning of targetPartSize: continue dividing spatially large
% groups until the budget is reached (rather than splitting blindly).
while numel(groups)<budget
    scores=-inf(1,numel(groups));
    for i=1:numel(groups)
        V=groups(i).vertices;
        span=max(V,[],1)-min(V,[],1);
        scores(i)=max(span);
    end
    [biggest,idx]=max(scores);
    if biggest<=options.targetPartSize+1e-10,break;end
    [A,B,ok]=obs_split_outer_cover(groups(idx).vertices);
    if ~ok,break;end
    old=groups(idx);
    groups(idx).vertices=A;
    groups(end+1)=struct('vertices',B, ...
        'componentIndex',old.componentIndex); %#ok<AGROW>
end
end

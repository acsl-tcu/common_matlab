function parts=obs_approx_superellipsoid(o,C,options)
% Smooth, even-exponent superellipsoids, with a strict number budget.
parts=repmat(obs_new_part('superellipsoid',[0 0 0],eye(3), ...
    struct('axes',[1 1 1],'exponent',8)),0,1);
pow=options.superExponent;
if numel(C)==1 && any(strcmp(C(1).kind,{'ellipsoid','sphere'}))
    if strcmp(C(1).kind,'sphere')
        a=[1 1 1]*C(1).params.radius;
    else
        a=reshape(C(1).params.axes,1,3);
    end
    % For p>=2, the p-ball encloses the 2-ball for equal semiaxes.
    p=obs_new_part('superellipsoid',o.position,o.R, ...
        struct('axes',a*(1+1e-10),'exponent',pow));
    p.componentIndex=1;parts=p;return;
end
G=obs_budgeted_groups(C,options.maxPartsPerObstacle);
for k=1:numel(G)
    mid=(G(k).lo+G(k).hi)/2;h=(G(k).hi-G(k).lo)/2;
    % Sum_j (h_j/a_j)^p <= 1 at every corner of the AABB.
    a=3^(1/pow)*max(h,1e-8)*(1+1e-10);
    p=obs_new_part('superellipsoid',o.position+(o.R*mid')',o.R, ...
        struct('axes',a,'exponent',pow));
    p.componentIndex=G(k).indices(1);
    parts(end+1)=p; %#ok<AGROW>
end
end

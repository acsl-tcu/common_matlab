function parts=obs_approx_multiple_ellipsoids(o,C,options)
% DIRECT ellipsoid fit to source geometry: NO AABB-to-ellipsoid step.
% Uniform ellipsoid family throughout the scene; maxPartsPerObstacle and
% maxTotalParts are respected by the caller's existing allocation logic.
parts=repmat(obs_new_part('ellipsoid',[0 0 0],eye(3), ...
    struct('axes',[1 1 1])),0,1);
% Spheres/ellipsoids already have an exact, one-part ellipsoid cover.
if numel(C)==1 && any(strcmp(C(1).kind,{'sphere','ellipsoid'}))
    parts=obs_approx_single_ellipsoid(o,C,options);
    return;
end
G=obs_direct_cover_groups(C,options);
for k=1:numel(G)
    [center,Rlocal,axes]=obs_fit_enclosing_ellipsoid(G(k).vertices,options);
    part=obs_new_part('ellipsoid',o.position+(o.R*center')', ...
        o.R*Rlocal,struct('axes',axes));
    part.componentIndex=G(k).componentIndex;
    parts(end+1)=part; %#ok<AGROW>
end
end

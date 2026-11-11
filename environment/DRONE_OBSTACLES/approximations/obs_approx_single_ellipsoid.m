function parts=obs_approx_single_ellipsoid(o,C,options)
% Direct single minimum-volume enclosing ellipsoid, not AABB->ellipsoid.
% Exact for ellipsoidal/spherical source. Other shapes are fitted to the
% convex hull of source vertices / guaranteed support-plane outer covers.
if nargin<3,options=struct();end
if numel(C)==1 && any(strcmp(C(1).kind,{'sphere','ellipsoid'}))
    if strcmp(C(1).kind,'sphere')
        axes=C(1).params.radius*[1 1 1];
    else
        axes=reshape(C(1).params.axes,1,3);
    end
    parts=obs_new_part('ellipsoid',o.position,o.R, ...
        struct('axes',axes*(1+1e-10)));
    parts.componentIndex=1;
    return;
end
V=zeros(0,3);
for i=1:numel(C)
    V=[V;obs_ellipsoid_cover_vertices(C(i),options)]; %#ok<AGROW>
end
[center,Rlocal,axes]=obs_fit_enclosing_ellipsoid(V,options);
parts=obs_new_part('ellipsoid',o.position+(o.R*center')', ...
    o.R*Rlocal,struct('axes',axes));
parts.componentIndex=1;
end

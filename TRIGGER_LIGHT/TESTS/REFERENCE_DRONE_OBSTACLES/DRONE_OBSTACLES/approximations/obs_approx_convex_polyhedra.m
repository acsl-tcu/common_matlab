function parts=obs_approx_convex_polyhedra(o,C,options)
% One conservative convex hull per bounded group of source components.
% Poly primitives are exact convex hulls; curved primitives are enclosed
% by support-plane polyhedra before merging (precomputation, not per step).
parts=repmat(obs_new_part('convex_polyhedron',[0 0 0],eye(3),struct(), ...
    obs_box_vertices([-1 -1 -1],[1 1 1])),0,1);
G=obs_budgeted_groups(C,options.maxPartsPerObstacle);
for k=1:numel(G)
    V=zeros(0,3);
    for j=G(k).indices
        if strcmp(C(j).kind,'poly')
            vv=C(j).vertices;
        else
            vv=obs_support_poly_vertices(C(j),options);
            vv=vv*(1+1e-8);
        end
        V=[V;vv]; %#ok<AGROW>
    end
    % Hull of a union contains all source components in this group.
    X=(o.R*V')'+o.position;
    p=obs_new_part('convex_polyhedron',[0 0 0],eye(3),struct(),X);
    p.componentIndex=G(k).indices(1);
    parts(end+1)=p; %#ok<AGROW>
end
end

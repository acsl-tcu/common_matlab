function inside=obs_world_contains(parts,pts,tol)
% Union of solid approximations; input points Nx3.
if nargin<3,tol=1e-9;end
inside=false(size(pts,1),1);
for i=1:numel(parts)
 inside=inside|obs_part_contains(parts(i),pts,tol);
end
end

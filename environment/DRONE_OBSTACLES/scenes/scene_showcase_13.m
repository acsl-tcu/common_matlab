function objects=scene_showcase_13()
% All thirteen primitive shapes. All are well separated for visualization.
t=[-1 -0.8;1 -0.8;0 1.4];
meshCells=zeros(6,3,2);
meshCells(:,:,1)=obs_prism_vertices(t,1.2);
meshCells(:,:,2)=obs_prism_vertices(t+[2.7 0],1.2);
objects(1)=obs_sphere('sphere_demo',[-9 0 2],0.8);
objects(2)=obs_ellipsoid('ellipsoid_demo',[-6 0 2],[.8 1.3 .5],[20 0 35]);
objects(3)=obs_box('box_demo',[-3 0 2],[1.5 2 .9],[10 20 15]);
objects(4)=obs_cylinder('cylinder_demo',[0 0 2],.8,1.6,[20 0 0]);
objects(5)=obs_capsule('capsule_demo',[3 0 2],.4,1.6,[20 40 0]);
objects(6)=obs_convex_polyhedron('polyhedron_demo',[6 0 2], ...
  [-1 -1 -1;1 -1 -1;1 1 -1;-1 1 -1;0 0 1.5],[15 20 30]);
objects(7)=obs_cone('cone_demo',[9 0 2],.8,1.8,[20 0 0]);
objects(8)=obs_triangular_pyramid('pyramid_demo',[-9 5 2],t,[0 0 2.0]);
objects(9)=obs_triangular_prism('prism_demo',[-6 5 2],t,1.6,[20 0 0]);
objects(10)=obs_hollow_square_prism('square_hole_demo',[-3 5 2],[2 2],[.9 .9],.8);
objects(11)=obs_hollow_cylinder('round_hole_demo',[0 5 2],1.1,.6,1.0,[0 20 0]);
objects(12)=obs_hollow_triangular_prism('triangle_hole_demo',[3 5 2],t,.4,.8);
objects(13)=obs_triangular_prism_mesh('prism_mesh_demo',[7 5 2],meshCells,[0 0 20]);
end

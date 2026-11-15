function objects=scene_mixed_obstacles()
% Scene contains different poses and scalable objects. All coordinates in m.
objects(1)=obs_box('large_box',[12 0 8],[15 15 15],[0 0 20]);
objects(2)=obs_ellipsoid('tilted_ellipsoid',[-2 3 2.5],[1.6 0.7 1.1],[20 15 35]);
objects(3)=obs_cylinder('column',[3 5 2.5],0.6,5,[5 10 0]);
objects(4)=obs_capsule('bar',[-3 7 2],0.25,4,[25 0 40]);
objects(5)=obs_hollow_square_prism('gate',[0 3 2.5],[3.5 3.5],[2 2],0.35,[90 0 0]);
end

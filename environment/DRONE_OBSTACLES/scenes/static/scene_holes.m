function objects=scene_holes()
% Three independent perforated solids (edit centers and RPY here).
objects(1)=obs_hollow_square_prism('square_gate',[-4 3 3],[4 4],[2 2],0.4,[90 0 0]);
objects(2)=obs_hollow_cylinder('round_tunnel',[0 3 3],2.0,1.0,0.5,[90 0 0]);
outer=[-2 -1; 2 -1; 0 2];
objects(3)=obs_hollow_triangular_prism('triangle_tunnel',[4 3 3],outer,0.45,0.5,[90 0 0]);
end

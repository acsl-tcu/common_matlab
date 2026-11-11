function drawer = obs_plot_world(world,showApprox)
% Backward-compatible 3D plot entrypoint. Use plot_obstacles_4views for 4 views.
if nargin<2,showApprox=true;end
drawer=DRAW_DRONE_OBSTACLES(world,'showApprox',showApprox);
end

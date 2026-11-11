function objects = scene_cylinders_high_side()
% Multiple cylinders placed well above and to the side of the origin.
% Purpose: orientation/approximation test scene with enough clearance from
% the nominal flight area around the origin.
%
% Position units: [m], orientation: [roll pitch yaw] in degrees.
% Convention of obs_cylinder:
%   obs_cylinder(name, centerPosition, radius, height, rpyDeg)
%
% Design intent:
% - keep obstacles away from the origin neighborhood,
% - distribute them mainly in +x, +y, and high-z regions,
% - vary pose so approximation and drawing can be checked visually.

objects(1) = obs_cylinder('cyl_A_vertical',          [ 8.0,  6.5,  6.0], 0.45, 5.0, [  0,   0,   0]);
objects(2) = obs_cylinder('cyl_B_yaw_tilt',          [10.5,  8.5,  7.2], 0.55, 4.2, [ 20,   0,  35]);
objects(3) = obs_cylinder('cyl_C_pitch_tilt',        [12.5,  5.5,  8.5], 0.50, 5.8, [  0,  28, -15]);
objects(4) = obs_cylinder('cyl_D_roll_pitch',        [ 9.0, 11.0,  9.0], 0.40, 6.0, [ 25,  18,  60]);
objects(5) = obs_cylinder('cyl_E_long_oblique',      [14.0,  9.5, 10.5], 0.60, 7.0, [ 35, -20,  15]);
objects(6) = obs_cylinder('cyl_F_short_oblique',     [ 7.5, 13.0,  7.8], 0.35, 3.6, [-15,  22, -50]);
objects(7) = obs_cylinder('cyl_G_high_near_cluster', [11.0, 12.5, 12.0], 0.45, 4.8, [ 10, -30,  85]);
objects(8) = obs_cylinder('cyl_H_far_side',          [15.5,  7.0,  6.8], 0.50, 5.0, [ -8,  12, 120]);

end

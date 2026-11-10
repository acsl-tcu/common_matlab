function ref = gen_wp_custom_points(opts)
%GEN_WP_CUSTOM_POINTS  Easiest place to draw an arbitrary open route.
% Each row has physical [x, y, z] in METRES, not [x-row; y-row; z-row].
% Define as many rows as you like. No intermediate waypoint stops.
arguments
    opts.speed (1,1) double {mustBePositive} = 0.20
    opts.rampTime (1,1) double = NaN % NaN = automatic; positive number = manual [s]
    opts.maxAccel (1,1) double {mustBePositive} = 0.30 % progress acceleration [m/s^2]
    opts.maxJerk (1,1) double {mustBePositive} = 0.50 % progress jerk [m/s^3]
    opts.maxD6 (1,1) double {mustBePositive} = 1.00 % 6th progress derivative [m/s^6]
    opts.blendRadius (1,1) double {mustBeNonnegative} = 0.40
    opts.yaw (1,1) double {mustBeFinite} = 0
end
%                  x[m]  y[m]  z[m]
pointsXYZ = [
                  0.0   0.0   3.0;  % Start
                  0.2   1.0   3.0;  % W1
                  0.8   2.0   3.0;  % W2
                  0.4   3.0   3.0;  % W3
                  0.0   4.0   3.0]; % Goal
ref = gen_ref_waypoint_smooth('pointsXYZ',pointsXYZ, ...
    'speed',opts.speed,'rampTime',opts.rampTime,'maxAccel',opts.maxAccel, ...
    'maxJerk',opts.maxJerk,'maxD6',opts.maxD6, ...
    'blendRadius',opts.blendRadius,'yaw',opts.yaw);
ref.routeName = 'custom_points';
end

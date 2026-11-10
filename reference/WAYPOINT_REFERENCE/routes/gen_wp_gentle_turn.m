function ref = gen_wp_gentle_turn(opts)
%GEN_WP_GENTLE_TURN  Open route through named [x y z] waypoints.
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
                  0.0   1.0   3.0;  % W1: before bend
                  0.6   2.0   3.0;  % W2: gentle turn
                  1.2   3.0   3.0;  % W3
                  1.2   4.0   3.0]; % Goal
ref = gen_ref_waypoint_smooth('pointsXYZ',pointsXYZ, ...
    'speed',opts.speed,'rampTime',opts.rampTime,'maxAccel',opts.maxAccel, ...
    'maxJerk',opts.maxJerk,'maxD6',opts.maxD6, ...
    'blendRadius',opts.blendRadius,'yaw',opts.yaw);
ref.routeName = 'gentle_turn';
end

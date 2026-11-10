function ref = gen_wp_formula_open(opts)
%GEN_WP_FORMULA_OPEN  Open (non-loop) formula sampled to waypoints.
% u is progress from 0 to 1, not a time variable. No stop at samples.
arguments
    opts.height (1,1) double {mustBeFinite} = 3.0
    opts.length (1,1) double {mustBePositive} = 4.0
    opts.lateralAmplitude (1,1) double {mustBeNonnegative} = 0.5
    opts.samples (1,1) double {mustBeInteger,mustBePositive} = 9
    opts.speed (1,1) double {mustBePositive} = 0.20
    opts.rampTime (1,1) double = NaN % NaN = automatic; positive number = manual [s]
    opts.maxAccel (1,1) double {mustBePositive} = 0.30 % progress acceleration [m/s^2]
    opts.maxJerk (1,1) double {mustBePositive} = 0.50 % progress jerk [m/s^3]
    opts.maxD6 (1,1) double {mustBePositive} = 1.00 % 6th progress derivative [m/s^6]
    opts.blendRadius (1,1) double {mustBeNonnegative} = 0.25
    opts.yaw (1,1) double {mustBeFinite} = 0
end
if opts.samples < 3
    error('WaypointRef:SampleCount','Set samples >= 3.');
end
u = linspace(0,1,opts.samples);
% ===== USER-EDITABLE FORMULA (u from 0 to 1) =====
x = opts.lateralAmplitude*sin(pi*u);
y = opts.length*u;
z = opts.height+zeros(size(u));
% =================================================
pointsXYZ = [x(:),y(:),z(:)];
ref = gen_ref_waypoint_smooth('pointsXYZ',pointsXYZ, ...
    'speed',opts.speed,'rampTime',opts.rampTime,'maxAccel',opts.maxAccel, ...
    'maxJerk',opts.maxJerk,'maxD6',opts.maxD6, ...
    'blendRadius',opts.blendRadius,'yaw',opts.yaw);
ref.routeName = 'formula_open';
end

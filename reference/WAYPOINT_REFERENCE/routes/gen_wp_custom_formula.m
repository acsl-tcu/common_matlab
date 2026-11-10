function ref = gen_wp_custom_formula(opts)
%GEN_WP_CUSTOM_FORMULA  Edit x(theta), y(theta), z(theta) in one file.
% Formula -> equally spaced sample points -> periodic Fourier interpolation.
% A full loop is theta in [0,2*pi); no duplicate endpoint. Infinite laps.
% Keep the formula smooth and periodic for a well-behaved 6th derivative.
arguments
    opts.height (1,1) double {mustBeFinite} = 3.0
    opts.scale (1,1) double {mustBePositive} = 1.0
    opts.samplesPerLap (1,1) double {mustBeInteger,mustBePositive} = 48
    opts.speed (1,1) double {mustBePositive} = 0.25
    opts.rampTime (1,1) double = NaN % NaN = automatic; positive number = manual [s]
    opts.maxAccel (1,1) double {mustBePositive} = 0.30 % progress acceleration [m/s^2]
    opts.maxJerk (1,1) double {mustBePositive} = 0.50 % progress jerk [m/s^3]
    opts.maxD6 (1,1) double {mustBePositive} = 1.00 % 6th progress derivative [m/s^6]
    opts.yaw (1,1) double {mustBeFinite} = 0
end
if opts.samplesPerLap < 16
    error('WaypointRef:SampleCount','Set samplesPerLap >= 16.');
end
phase = 2*pi*(0:opts.samplesPerLap-1)/opts.samplesPerLap;
% ===== USER-EDITABLE FORMULA (theta in radians) =====
x = opts.scale*(cos(phase)-1);         % starts at x=0
y = 0.8*opts.scale*sin(phase);          % starts at y=0
z = opts.height + 0.1*sin(2*phase);    % starts at z=height
% ================================================
pointsXYZ = [x(:), y(:), z(:)];
ref = gen_ref_waypoint_periodic('pointsXYZ',pointsXYZ, ...
    'speed',opts.speed,'rampTime',opts.rampTime,'maxAccel',opts.maxAccel, ...
    'maxJerk',opts.maxJerk,'maxD6',opts.maxD6,'yaw',opts.yaw);
ref.routeName = 'custom_formula';
end

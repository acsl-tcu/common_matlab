function ref = gen_wp_circle(opts)
%GEN_WP_CIRCLE  Circle specified by a FORMULA, then sampled to waypoints.
% The Fourier interpolant passes through the samples and loops forever.
% Default starts at [0 0 3] to match the takeoff goal of the current run.
arguments
    opts.radius (1,1) double {mustBePositive} = 1.0       % radius [m]
    opts.startXYZ (3,1) double = [0;0;3]              % flight start [x;y;z] [m]
    opts.samplesPerLap (1,1) double {mustBeInteger,mustBePositive} = 48
    opts.direction (1,1) double = 1
    opts.speed (1,1) double {mustBePositive} = 0.25
    opts.rampTime (1,1) double = NaN % NaN = automatic; positive number = manual [s]
    opts.maxAccel (1,1) double {mustBePositive} = 0.30 % progress acceleration [m/s^2]
    opts.maxJerk (1,1) double {mustBePositive} = 0.50 % progress jerk [m/s^3]
    opts.maxD6 (1,1) double {mustBePositive} = 1.00 % 6th progress derivative [m/s^6]
    opts.yaw (1,1) double {mustBeFinite} = 0
end
% Formula: p(theta) = [startX+R*(cos(theta)-1); startY+R*sin(theta); startZ]
% Change these three equations to customize the geometric shape.
if opts.samplesPerLap < 16 || ~ismember(opts.direction,[-1,1])
    error('WaypointRef:CircleOptions','samplesPerLap >= 16 and direction = +/-1 required.');
end
phase = opts.direction * (2*pi*(0:opts.samplesPerLap-1)/opts.samplesPerLap);
x = opts.startXYZ(1) + opts.radius*(cos(phase)-1);
y = opts.startXYZ(2) + opts.radius*sin(phase);
z = opts.startXYZ(3) + zeros(size(phase));
pointsXYZ = [x(:), y(:), z(:)]; % N x 3, endpoint NOT duplicated
ref = gen_ref_waypoint_periodic('pointsXYZ',pointsXYZ, ...
    'speed',opts.speed,'rampTime',opts.rampTime,'maxAccel',opts.maxAccel, ...
    'maxJerk',opts.maxJerk,'maxD6',opts.maxD6,'yaw',opts.yaw);
ref.routeName = 'circle';
end

function ref = gen_wp_saddle(opts)
%GEN_WP_SADDLE  Example periodic figure-eight / saddle-like route.
% This IS NOT claimed identical to the legacy gen_ref_saddle: its original
% equation was not contained in the supplied archive. Edit equations below
% to match the legacy saddle once that file is provided.
% Default phase=0 -> [0,0,3] and then loops indefinitely.
arguments
    opts.amplitudeX (1,1) double {mustBePositive} = 1.0
    opts.amplitudeY (1,1) double {mustBePositive} = 0.65
    opts.height (1,1) double {mustBeFinite} = 3.0
    opts.verticalAmplitude (1,1) double {mustBeNonnegative} = 0.0
    opts.samplesPerLap (1,1) double {mustBeInteger,mustBePositive} = 64
    opts.speed (1,1) double {mustBePositive} = 0.25
    opts.rampTime (1,1) double = NaN % NaN = automatic; positive number = manual [s]
    opts.maxAccel (1,1) double {mustBePositive} = 0.30 % progress acceleration [m/s^2]
    opts.maxJerk (1,1) double {mustBePositive} = 0.50 % progress jerk [m/s^3]
    opts.maxD6 (1,1) double {mustBePositive} = 1.00 % 6th progress derivative [m/s^6]
    opts.yaw (1,1) double {mustBeFinite} = 0
end
% A periodic figure-eight curve, two lobes, with optional vertical wave.
% Changing x(theta),y(theta),z(theta) changes the route.
if opts.samplesPerLap < 16
    error('WaypointRef:SampleCount','Set samplesPerLap >= 16.');
end
phase = 2*pi*(0:opts.samplesPerLap-1)/opts.samplesPerLap;
x = opts.amplitudeX*sin(phase);
y = opts.amplitudeY*sin(2*phase);
z = opts.height + opts.verticalAmplitude*(cos(2*phase)-1);
pointsXYZ = [x(:),y(:),z(:)];
ref = gen_ref_waypoint_periodic('pointsXYZ',pointsXYZ, ...
    'speed',opts.speed,'rampTime',opts.rampTime,'maxAccel',opts.maxAccel, ...
    'maxJerk',opts.maxJerk,'maxD6',opts.maxD6,'yaw',opts.yaw);
ref.routeName = 'saddle_example';
end

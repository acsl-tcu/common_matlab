function ref = gen_ref_hl_demo_multi(param)
% gen_ref_hl_demo_multi
% Continuous multi-trajectory reference for an HL demo.
%
% Sequence:
%   hover -> square -> hover -> figure-eight -> hover -> short saddle
%   -> hover -> circle -> hover
%
% The returned reference is a 20 x 1 vector:
%   [p;yaw; dp;dyaw; ddp;ddyaw; dddp;dddyaw; ddddp;ddddyaw].
% It is already in the HL format, so do not pass the "HL" flag to
% TIME_VARYING_REFERENCE when using this generator.

arguments
    param.hover (1,3) double = [0 0 0.6]
    param.loops (1,1) double = 5
    param.square_size (1,1) double = 1.0
    param.figure8_size (1,3) double = [0.85 0.85 0.0]
    param.saddle_size (1,3) double = [0.55 0.55 0.30]
    param.circle_radius (1,1) double = 1.0
    param.square_period (1,1) double = 12
    param.figure8_period (1,1) double = 15
    param.saddle_period (1,1) double = 12
    param.circle_period (1,1) double = 12
    param.hold_start (1,1) double = 4
    param.hold_between (1,1) double = 2
    param.hold_end (1,1) double = 5
    param.ramp_fraction (1,1) double = 0.12
end

cfg = struct();
cfg.hover = param.hover(:);
cfg.loops = max(1, round(param.loops));
cfg.square_size = param.square_size;
cfg.figure8_size = param.figure8_size(:);
cfg.saddle_size = param.saddle_size(:);
cfg.circle_radius = param.circle_radius;
cfg.square_period = param.square_period;
cfg.figure8_period = param.figure8_period;
cfg.saddle_period = param.saddle_period;
cfg.circle_period = param.circle_period;
cfg.hold_start = param.hold_start;
cfg.hold_between = param.hold_between;
cfg.hold_end = param.hold_end;
cfg.ramp_fraction = min(0.35, max(0.05, param.ramp_fraction));
cfg.segments = build_schedule(cfg);
cfg.total_time = cfg.segments(end).t1;

ref = @(t) demo_state(t, cfg);

fprintf('gen_ref_hl_demo_multi: total reference time = %.1f s\n', cfg.total_time);
fprintf('  hover -> square(%d laps) -> figure8(%d laps) -> saddle(%d laps) -> circle(%d laps) -> hover\n', ...
    cfg.loops, cfg.loops, cfg.loops, cfg.loops);
fprintf('  hover=[%.2f %.2f %.2f], square side=%.2f m, fig8=[%.2f %.2f %.2f], saddle=[%.2f %.2f %.2f], circle r=%.2f\n', ...
    cfg.hover(1), cfg.hover(2), cfg.hover(3), cfg.square_size, ...
    cfg.figure8_size(1), cfg.figure8_size(2), cfg.figure8_size(3), ...
    cfg.saddle_size(1), cfg.saddle_size(2), cfg.saddle_size(3), cfg.circle_radius);
end

function segments = build_schedule(cfg)
segments = struct('name', {}, 't0', {}, 't1', {});
t = 0;
[segments, t] = add_segment(segments, t, 'hold_start', cfg.hold_start);
[segments, t] = add_segment(segments, t, 'square', cfg.square_period * cfg.loops);
[segments, t] = add_segment(segments, t, 'hold_between', cfg.hold_between);
[segments, t] = add_segment(segments, t, 'figure8', cfg.figure8_period * cfg.loops);
[segments, t] = add_segment(segments, t, 'hold_between', cfg.hold_between);
[segments, t] = add_segment(segments, t, 'saddle', cfg.saddle_period * cfg.loops);
[segments, t] = add_segment(segments, t, 'hold_between', cfg.hold_between);
[segments, t] = add_segment(segments, t, 'circle', cfg.circle_period * cfg.loops);
[segments, ~] = add_segment(segments, t, 'hold_end', cfg.hold_end);
end

function [segments, t] = add_segment(segments, t, name, duration)
duration = max(0, duration);
segments(end + 1).name = name;
segments(end).t0 = t;
segments(end).t1 = t + duration;
t = t + duration;
end

function xd = demo_state(t, cfg)
t = min(max(double(t), 0), cfg.total_time);
idx = numel(cfg.segments);
for i = 1:numel(cfg.segments)
    if t <= cfg.segments(i).t1 || i == numel(cfg.segments)
        idx = i;
        break;
    end
end

seg = cfg.segments(idx);
tau = t - seg.t0;
duration = max(seg.t1 - seg.t0, eps);

switch seg.name
    case {'hold_start', 'hold_between', 'hold_end'}
        xd = pack_state(cfg.hover, zeros(3,1), zeros(3,1), zeros(3,1), zeros(3,1));
    case 'square'
        xd = square_state(tau, cfg);
    case 'figure8'
        xd = periodic_state(tau, duration, cfg, 'figure8');
    case 'saddle'
        xd = periodic_state(tau, duration, cfg, 'saddle');
    case 'circle'
        xd = periodic_state(tau, duration, cfg, 'circle');
    otherwise
        xd = pack_state(cfg.hover, zeros(3,1), zeros(3,1), zeros(3,1), zeros(3,1));
end
end

function xd = square_state(t, cfg)
L = cfg.square_size;
pts = cfg.hover + [ ...
    0, 0, 0; ...
    L, 0, 0; ...
    L, L, 0; ...
    0, L, 0; ...
    0, 0, 0]';

edge_time = cfg.square_period / 4;
total_time = cfg.square_period * cfg.loops;
t = min(max(t, 0), total_time);

if t >= total_time
    xd = pack_state(cfg.hover, zeros(3,1), zeros(3,1), zeros(3,1), zeros(3,1));
    return;
end

edge_global = floor(t / edge_time);
edge = mod(edge_global, 4) + 1;
tau = t - edge_global * edge_time;

p0 = pts(:, edge);
p1 = pts(:, edge + 1);
dp = p1 - p0;
[c0, c1, c2, c3, c4] = min_jerk_coeff(tau, edge_time);

pos = p0 + dp * c0;
vel = dp * c1;
acc = dp * c2;
jerk = dp * c3;
snap = dp * c4;
xd = pack_state(pos, vel, acc, jerk, snap);
end

function xd = periodic_state(t, duration, cfg, kind)
t = min(max(t, 0), duration);
omega = 2 * pi * cfg.loops / duration;
ramp_time = max(2.0, cfg.ramp_fraction * duration);
ramp_time = min(ramp_time, 0.45 * duration);
[env, denv, ddenv, dddenv, ddddenv] = envelope7(t, duration, ramp_time);

[a0, a1, a2, a3, a4] = periodic_shape(t, omega, cfg, kind);

pos = cfg.hover + env * a0;
vel = denv * a0 + env * a1;
acc = ddenv * a0 + 2 * denv * a1 + env * a2;
jerk = dddenv * a0 + 3 * ddenv * a1 + 3 * denv * a2 + env * a3;
snap = ddddenv * a0 + 4 * dddenv * a1 + 6 * ddenv * a2 + 4 * denv * a3 + env * a4;

xd = pack_state(pos, vel, acc, jerk, snap);
end

function [a0, a1, a2, a3, a4] = periodic_shape(t, omega, cfg, kind)
theta = omega * t;
switch kind
    case 'figure8'
        sx = cfg.figure8_size(1);
        sy = cfg.figure8_size(2);
        sz = cfg.figure8_size(3);
        a0 = [sx * sin(theta); sy * sin(2 * theta) / 2; sz * sin(theta)];
        a1 = [sx * omega * cos(theta); sy * omega * cos(2 * theta); sz * omega * cos(theta)];
        a2 = [-sx * omega^2 * sin(theta); -2 * sy * omega^2 * sin(2 * theta); -sz * omega^2 * sin(theta)];
        a3 = [-sx * omega^3 * cos(theta); -4 * sy * omega^3 * cos(2 * theta); -sz * omega^3 * cos(theta)];
        a4 = [sx * omega^4 * sin(theta); 8 * sy * omega^4 * sin(2 * theta); sz * omega^4 * sin(theta)];
    case 'saddle'
        sx = cfg.saddle_size(1);
        sy = cfg.saddle_size(2);
        sz = cfg.saddle_size(3);
        a0 = [sx * cos(theta); sy * sin(theta); sz * sin(2 * theta)];
        a1 = [-sx * omega * sin(theta); sy * omega * cos(theta); 2 * sz * omega * cos(2 * theta)];
        a2 = [-sx * omega^2 * cos(theta); -sy * omega^2 * sin(theta); -4 * sz * omega^2 * sin(2 * theta)];
        a3 = [sx * omega^3 * sin(theta); -sy * omega^3 * cos(theta); -8 * sz * omega^3 * cos(2 * theta)];
        a4 = [sx * omega^4 * cos(theta); sy * omega^4 * sin(theta); 16 * sz * omega^4 * sin(2 * theta)];
    case 'circle'
        r = cfg.circle_radius;
        a0 = [r * cos(theta); r * sin(theta); 0];
        a1 = [-r * omega * sin(theta); r * omega * cos(theta); 0];
        a2 = [-r * omega^2 * cos(theta); -r * omega^2 * sin(theta); 0];
        a3 = [r * omega^3 * sin(theta); -r * omega^3 * cos(theta); 0];
        a4 = [r * omega^4 * cos(theta); r * omega^4 * sin(theta); 0];
    otherwise
        a0 = zeros(3,1); a1 = zeros(3,1); a2 = zeros(3,1); a3 = zeros(3,1); a4 = zeros(3,1);
end
end

function [e, e1, e2, e3, e4] = envelope7(t, duration, ramp_time)
if t < ramp_time
    u = t / ramp_time;
    sgn = 1;
    scale = ramp_time;
elseif t > duration - ramp_time
    u = (duration - t) / ramp_time;
    sgn = -1;
    scale = ramp_time;
else
    e = 1; e1 = 0; e2 = 0; e3 = 0; e4 = 0;
    return;
end

[f0, f1, f2, f3, f4] = smooth7(u);
e = f0;
e1 = sgn * f1 / scale;
e2 = f2 / scale^2;
e3 = sgn * f3 / scale^3;
e4 = f4 / scale^4;
end

function [f0, f1, f2, f3, f4] = smooth7(u)
u = min(max(u, 0), 1);
f0 = 35*u^4 - 84*u^5 + 70*u^6 - 20*u^7;
f1 = 140*u^3 - 420*u^4 + 420*u^5 - 140*u^6;
f2 = 420*u^2 - 1680*u^3 + 2100*u^4 - 840*u^5;
f3 = 840*u - 5040*u^2 + 8400*u^3 - 4200*u^4;
f4 = 840 - 10080*u + 25200*u^2 - 16800*u^3;
end

function [c0, c1, c2, c3, c4] = min_jerk_coeff(t, T)
T = max(T, eps);
tau = min(max(t / T, 0), 1);
c0 = 10*tau^3 - 15*tau^4 + 6*tau^5;
c1 = (30*tau^2 - 60*tau^3 + 30*tau^4) / T;
c2 = (60*tau - 180*tau^2 + 120*tau^3) / T^2;
c3 = (60 - 360*tau + 360*tau^2) / T^3;
c4 = (-360 + 720*tau) / T^4;
end

function xd = pack_state(pos, vel, acc, jerk, snap)
xd = [ ...
    pos(:); 0; ...
    vel(:); 0; ...
    acc(:); 0; ...
    jerk(:); 0; ...
    snap(:); 0];
end

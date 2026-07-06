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
    param.loops (1,1) double = 3
    param.square_size (1,1) double = 1.0
    param.figure8_size (1,3) double = [0.85 0.85 0.0]
    param.saddle_size (1,3) double = [0.55 0.55 0.30]
    param.circle_radius (1,1) double = 1.0
    param.square_period (1,1) double = 12
    param.figure8_period (1,1) double = 12
    param.saddle_period (1,1) double = 12
    param.circle_period (1,1) double = 12
    param.hold_start (1,1) double = 4
    param.hold_between (1,1) double = 2
    param.hold_end (1,1) double = 5
    param.ramp_fraction (1,1) double = 0.12
    param.transition_time (1,1) double = 2.0
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
cfg.transition_time = max(0, param.transition_time);
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
fprintf('  each periodic section uses %.1f s transitions; loops count only full-size cycles\n', cfg.transition_time);
end

function segments = build_schedule(cfg)
segments = struct('name', {}, 't0', {}, 't1', {});
t = 0;
[segments, t] = add_segment(segments, t, 'hold_start', cfg.hold_start);
[segments, t] = add_segment(segments, t, 'square', cfg.square_period * cfg.loops);
[segments, t] = add_segment(segments, t, 'hold_between', cfg.hold_between);
[segments, t] = add_segment(segments, t, 'figure8', periodic_duration(cfg, 'figure8'));
[segments, t] = add_segment(segments, t, 'hold_between', cfg.hold_between);
[segments, t] = add_segment(segments, t, 'saddle', periodic_duration(cfg, 'saddle'));
[segments, t] = add_segment(segments, t, 'hold_between', cfg.hold_between);
[segments, t] = add_segment(segments, t, 'circle', periodic_duration(cfg, 'circle'));
[segments, ~] = add_segment(segments, t, 'hold_end', cfg.hold_end);
end

function duration = periodic_duration(cfg, kind)
duration = periodic_period(cfg, kind) * cfg.loops + 2 * cfg.transition_time;
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
period = periodic_period(cfg, kind);
main_duration = period * cfg.loops;
transition_time = min(cfg.transition_time, 0.25 * main_duration);
omega = 2 * pi / period;

[a0s, a1s, a2s, a3s, a4s] = periodic_shape(0, omega, cfg, kind);
[a0e, a1e, a2e, a3e, a4e] = periodic_shape(main_duration, omega, cfg, kind);

p_hover = cfg.hover;
z3 = zeros(3, 1);
p_start = cfg.hover + a0s;
p_end = cfg.hover + a0e;

if transition_time > 0 && t < transition_time
    xd = hermite9_state(t, transition_time, ...
        p_hover, z3, z3, z3, z3, ...
        p_start, a1s, a2s, a3s, a4s);
    return;
end

if t <= transition_time + main_duration
    tau = min(max(t - transition_time, 0), main_duration);
    [a0, a1, a2, a3, a4] = periodic_shape(tau, omega, cfg, kind);
    xd = pack_state(cfg.hover + a0, a1, a2, a3, a4);
    return;
end

tau = min(max(t - transition_time - main_duration, 0), transition_time);
xd = hermite9_state(tau, transition_time, ...
    p_end, a1e, a2e, a3e, a4e, ...
    p_hover, z3, z3, z3, z3);
end

function period = periodic_period(cfg, kind)
switch kind
    case 'figure8'
        period = cfg.figure8_period;
    case 'saddle'
        period = cfg.saddle_period;
    case 'circle'
        period = cfg.circle_period;
    otherwise
        period = 1;
end
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

function xd = hermite9_state(t, T, p0, v0, a0, j0, snap0, p1, v1, a1, j1, snap1)
T = max(T, eps);
t = min(max(t, 0), T);
u = t / T;

c = zeros(10, 3);
c(1, :) = p0(:)';
c(2, :) = (T * v0(:))';
c(3, :) = (T^2 * a0(:) / 2)';
c(4, :) = (T^3 * j0(:) / 6)';
c(5, :) = (T^4 * snap0(:) / 24)';

n_low = 0:4;
known = c(1:5, :);
target = [ ...
    p1(:)'; ...
    (T * v1(:))'; ...
    (T^2 * a1(:))'; ...
    (T^3 * j1(:))'; ...
    (T^4 * snap1(:))'];

known_terms = [ ...
    ones(1, 5); ...
    n_low; ...
    n_low .* max(n_low - 1, 0); ...
    n_low .* max(n_low - 1, 0) .* max(n_low - 2, 0); ...
    n_low .* max(n_low - 1, 0) .* max(n_low - 2, 0) .* max(n_low - 3, 0)] * known;

n_high = 5:9;
A = [ ...
    ones(1, 5); ...
    n_high; ...
    n_high .* (n_high - 1); ...
    n_high .* (n_high - 1) .* (n_high - 2); ...
    n_high .* (n_high - 1) .* (n_high - 2) .* (n_high - 3)];
c(6:10, :) = A \ (target - known_terms);

n = (0:9)';
u_pow = u .^ n;
pos = (u_pow' * c)';

du_pow = zeros(10, 1);
ddu_pow = zeros(10, 1);
dddu_pow = zeros(10, 1);
ddddu_pow = zeros(10, 1);
for i = 2:10
    ni = n(i);
    du_pow(i) = ni * u^(ni - 1);
    if ni >= 2
        ddu_pow(i) = ni * (ni - 1) * u^(ni - 2);
    end
    if ni >= 3
        dddu_pow(i) = ni * (ni - 1) * (ni - 2) * u^(ni - 3);
    end
    if ni >= 4
        ddddu_pow(i) = ni * (ni - 1) * (ni - 2) * (ni - 3) * u^(ni - 4);
    end
end

vel = (du_pow' * c)' / T;
acc = (ddu_pow' * c)' / T^2;
jerk = (dddu_pow' * c)' / T^3;
snap = (ddddu_pow' * c)' / T^4;

xd = pack_state(pos, vel, acc, jerk, snap);
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

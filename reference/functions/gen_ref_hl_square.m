function ref = gen_ref_hl_square(param)
% gen_ref_hl_square
% Smooth square reference around a hover point for HL experiments.
%
% The returned vector is already in HL format:
% [p;yaw; dp;dyaw; ddp;ddyaw; dddp;dddyaw; ddddp;ddddyaw].

arguments
    param.hover (1,3) double = [0 0 1.0]
    param.side (1,1) double = 1.0
    param.period (1,1) double = 16.0
    param.hold_start (1,1) double = 2.0
    param.hold_end (1,1) double = 2.0
end

cfg.hover = param.hover(:);
cfg.side = param.side;
cfg.period = max(param.period, param.hold_start + param.hold_end + 6.0);
cfg.hold_start = max(0, param.hold_start);
cfg.hold_end = max(0, param.hold_end);

fprintf('gen_ref_hl_square: side=%.2f m, period=%.1f s, hover=[%.2f %.2f %.2f]\n', ...
    cfg.side, cfg.period, cfg.hover(1), cfg.hover(2), cfg.hover(3));

ref = @(t) square_state(t, cfg);
end

function xd = square_state(t, cfg)
t = mod(max(double(t), 0), cfg.period);

move_time = cfg.period - cfg.hold_start - cfg.hold_end;
if t < cfg.hold_start || t > cfg.period - cfg.hold_end
    xd = pack_state(cfg.hover, zeros(3,1), zeros(3,1), zeros(3,1), zeros(3,1));
    return;
end

tau = t - cfg.hold_start;
seg_time = move_time / 6.0;
seg = min(5, floor(tau / seg_time));
r = (tau - seg * seg_time) / seg_time;

L = cfg.side;
c = cfg.hover;
corners = [ ...
    c + [-L/2; -L/2; 0], ...
    c + [ L/2; -L/2; 0], ...
    c + [ L/2;  L/2; 0], ...
    c + [-L/2;  L/2; 0], ...
    c + [-L/2; -L/2; 0]];

points = [c, corners, c];
p0 = points(:, seg + 1);
p1 = points(:, seg + 2);
d = p1 - p0;

[s, ds, dds, ddds, dddds] = smoothstep5(r, seg_time);

p = p0 + d * s;
v = d * ds;
a = d * dds;
j = d * ddds;
snap = d * dddds;
xd = pack_state(p, v, a, j, snap);
end

function [s, ds, dds, ddds, dddds] = smoothstep5(r, T)
r = min(max(r, 0), 1);
s = 10*r^3 - 15*r^4 + 6*r^5;
ds = (30*r^2 - 60*r^3 + 30*r^4) / T;
dds = (60*r - 180*r^2 + 120*r^3) / (T^2);
ddds = (60 - 360*r + 360*r^2) / (T^3);
dddds = (-360 + 720*r) / (T^4);
end

function xd = pack_state(p, v, a, j, snap)
xd = [ ...
    p(:); 0; ...
    v(:); 0; ...
    a(:); 0; ...
    j(:); 0; ...
    snap(:); 0];
end

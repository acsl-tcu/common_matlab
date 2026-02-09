function OUT = eval_sway_compare_from_series(S_no, S_on, theta_max, opt)
% eval_sway_compare_from_series
% Compare sway metrics & plots with (optionally) flight-phase-only + time alignment.
% Also supports SINGLE mode (only S_no), by passing [] or omitting S_on.
%
% Usage:
%   OUT = eval_sway_compare_from_series(S_no, S_on, theta_max, opt);   % compare
%   OUT = eval_sway_compare_from_series(S_one, [], theta_max, opt);    % single
%
% Required fields in S_*:
%   t, p, v, pL, vL
% Optional fields:
%   phase (numeric), cha (char/cellstr/string), xd or pref, etc.

% ---------- args ----------
if nargin < 4, opt = struct(); end
if nargin < 2, S_on = []; end
if isempty(S_on), single_mode = true; else, single_mode = false; end

% ---------- defaults ----------
opt = set_default(opt, 't_range', []);
opt = set_default(opt, 'sr', 0.3);
opt = set_default(opt, 'env_win_sec', 0.5);

opt = set_default(opt, 'use_flight_phase_only', false);
opt = set_default(opt, 'flight_phase_value', 0);
opt = set_default(opt, 'flight_cha', 'f');

opt = set_default(opt, 'align', true);
% NOTE: below alignment-by-event functions exist but default is "flight start = 0"
opt = set_default(opt, 'align_mode', "ref_move");  % "ref_move" or "speed_rise"
opt = set_default(opt, 'align_ref_eps', 0.03);
opt = set_default(opt, 'align_speed_eps', 0.15);
opt = set_default(opt, 'pre_sec', 1.0);
opt = set_default(opt, 'post_sec', inf);
opt = set_default(opt, 'zero_time_at_window_start', true);

% ---------- 1) flight phase crop ----------
A0 = crop_flight_phase(S_no, opt);
if ~single_mode
    A1 = crop_flight_phase(S_on, opt);
else
    A1 = [];
end

% safety
if isempty(A0.t)
    error('Flight crop resulted in empty series (S_no). Check opt.flight_cha and cha mapping.');
end
if ~single_mode && isempty(A1.t)
    error('Flight crop resulted in empty series (S_on). Check opt.flight_cha and cha mapping.');
end

% ---------- 2) alignment ----------
% Default behavior: "flight start = 0" for each series.
if opt.align
    A0.t = A0.t - A0.t(1);
    if ~single_mode
        A1.t = A1.t - A1.t(1);
    end
end

% ---------- 3) final window (relative time) ----------
% Backward compatible:
%  - opt.t_range      : applies to both (legacy)
%  - opt.t_range_no   : applies only to A0 (no)
%  - opt.t_range_on   : applies only to A1 (on)
%  - opt.t_range can be struct with fields .no/.on

tr0 = [];
tr1 = [];

if isfield(opt,'t_range') && ~isempty(opt.t_range)
    if isstruct(opt.t_range)
        if isfield(opt.t_range,'no'), tr0 = opt.t_range.no; end
        if isfield(opt.t_range,'on'), tr1 = opt.t_range.on; end
    else
        tr0 = opt.t_range;
        tr1 = opt.t_range;
    end
end
if isfield(opt,'t_range_no') && ~isempty(opt.t_range_no), tr0 = opt.t_range_no; end
if isfield(opt,'t_range_on') && ~isempty(opt.t_range_on), tr1 = opt.t_range_on; end

if ~isempty(tr0)
    A0 = crop_time_range(A0, tr0);
end
if ~single_mode && ~isempty(tr1)
    A1 = crop_time_range(A1, tr1);
end
if opt.zero_time_at_window_start
    A0.t = A0.t - A0.t(1);
    if ~single_mode
        A1.t = A1.t - A1.t(1);
    end
end

% ---------- 4) compute metrics ----------
no = compute_metrics(A0, theta_max, opt);
if ~single_mode
    on = compute_metrics(A1, theta_max, opt);
else
    on = [];
end
% ---------- has_theta_vr (for summary/plots) ----------
has_theta_vr = isfield(opt,'use_theta_vr_switch') && opt.use_theta_vr_switch ...
    && isfield(opt,'theta_on') && isfield(opt,'theta_off') ...
    && isfield(opt,'vr_on') && isfield(opt,'vr_off');

% ---------- 5) print summary ----------
if ~single_mode
    fprintf('\n=== comparison (aligned/flight-cropped) ===\n');
    fprintf('theta_rms [deg]  no=%.3f  on=%.3f  ratio=%.3f\n', no.theta_rms_deg, on.theta_rms_deg, safe_ratio(on.theta_rms_deg, no.theta_rms_deg));
    fprintf('theta_max [deg]  no=%.3f  on=%.3f  ratio=%.3f\n', no.theta_max_deg, on.theta_max_deg, safe_ratio(on.theta_max_deg, no.theta_max_deg));
    fprintf('violation_ratio  no=%.4f  on=%.4f\n', no.violation_ratio, on.violation_ratio);
    fprintf('vsway_rms [m/s]  no=%.4f  on=%.4f  ratio=%.3f\n', no.vsway_rms, on.vsway_rms, safe_ratio(on.vsway_rms, no.vsway_rms));
    fprintf('E_total          no=%.4f  on=%.4f  ratio=%.3f\n', no.vsway_energy, on.vsway_energy, safe_ratio(on.vsway_energy, no.vsway_energy));

    % paper-strong metrics
    fprintf('theta_p95 [deg]  no=%.3f  on=%.3f  ratio=%.3f\n', no.theta_p95_deg, on.theta_p95_deg, safe_ratio(on.theta_p95_deg, no.theta_p95_deg));
    fprintf('vsway_p95 [m/s]  no=%.4f  on=%.4f  ratio=%.3f\n', no.vsway_p95, on.vsway_p95, safe_ratio(on.vsway_p95, no.vsway_p95));
    fprintf('settle_t (to 1deg band) no=%.3f  on=%.3f\n', no.settle_time_s, on.settle_time_s);
    if has_theta_vr
    fprintf('theta_on_rate    no=%.4f  on=%.4f  ratio=%.3f\n', no.theta_on_rate, on.theta_on_rate, safe_ratio(on.theta_on_rate, no.theta_on_rate));
    fprintf('vr_on_rate       no=%.4f  on=%.4f  ratio=%.3f\n', no.vr_on_rate, on.vr_on_rate, safe_ratio(on.vr_on_rate, no.vr_on_rate));
    fprintf('oncond_rate      no=%.4f  on=%.4f  ratio=%.3f\n', no.oncond_rate, on.oncond_rate, safe_ratio(on.oncond_rate, no.oncond_rate));
    end
else
    fprintf('\n=== single (aligned/flight-cropped) ===\n');
    fprintf('theta_rms [deg]  %.3f\n', no.theta_rms_deg);
    fprintf('theta_max [deg]  %.3f\n', no.theta_max_deg);
    fprintf('violation_ratio  %.4f\n', no.violation_ratio);
    fprintf('vsway_rms [m/s]  %.4f\n', no.vsway_rms);
    fprintf('E_total          %.4f\n', no.vsway_energy);

    fprintf('theta_p95 [deg]  %.3f\n', no.theta_p95_deg);
    fprintf('vsway_p95 [m/s]  %.4f\n', no.vsway_p95);
    fprintf('settle_t (to 1deg band) %.3f\n', no.settle_time_s);
    if has_theta_vr
    fprintf('theta_on_rate    %.4f\n', no.theta_on_rate);
    fprintf('vr_on_rate       %.4f\n', no.vr_on_rate);
    fprintf('oncond_rate      %.4f\n', no.oncond_rate);
    end
end

% ---------- 6) plots ----------
if ~single_mode
    paper_plots(A0, A1, no, on, theta_max, opt);
else
    paper_plots_single(A0, no, theta_max, opt);
end

% ---------- return ----------
OUT.no = no;
OUT.A0 = A0;
if ~single_mode
    OUT.on = on;
    OUT.A1 = A1;
end

end

% ================= helpers =================

function S = set_default(S, name, val)
if ~isfield(S,name) || isempty(S.(name)), S.(name) = val; end
end

function R = safe_ratio(a,b)
if ~isfinite(b) || abs(b) < 1e-12
    R = NaN;
else
    R = a/b;
end
end

function A = crop_flight_phase(S, opt)
A = S;

if ~opt.use_flight_phase_only
    return;
end

t = S.t(:);

% 1) phase があれば phase を使う
if isfield(S,'phase') && ~isempty(S.phase)
    ph = S.phase(:);
    mask = (ph == opt.flight_phase_value);
    A = apply_mask(S, mask);
    return;
end

% 2) cha があれば cha を使う（char/cellstr/string どれでもOKに）
% 2) cha があれば cha を使う（型は全部 string に正規化して比較）
if isfield(S,'cha') && ~isempty(S.cha)
    ch = S.cha;

    % どの型でも string 配列に揃える
    if iscategorical(ch)
        chs = string(ch);
    elseif isstring(ch)
        chs = ch;
    elseif iscell(ch)
        chs = string(ch);
    elseif ischar(ch)
        % char の場合：行がサンプル（Nx1 や NxM）を想定
        % Nx1 なら各行1文字、NxM なら各行がラベル
        chs = string(ch);
    else
        % 念のため
        chs = string(ch);
    end

    chs = chs(:);  % 縦ベクトル化

    mask = (chs == string(opt.flight_cha));
    A = apply_mask(S, mask);
    return;
end


% 3) どっちも無い → 高度で flight 推定
ground_sec = 1.0;
dt = median(diff(t)); if ~isfinite(dt) || dt<=0, dt=0.025; end
Ng = max(3, round(ground_sec/dt));
Ng = min(Ng, size(S.p,1));
z0 = mean(S.p(1:Ng,3));  % drone z

if ~isfield(opt,'z_up') || isempty(opt.z_up)
    opt.z_up = 0.3;   % 例：30cm以上上がったら flight
end

mask = (S.p(:,3) > z0 + opt.z_up);
mask = keep_longest_true_segment(mask);

A = apply_mask(S, mask);

if isempty(A.t)
    warning('Flight crop resulted in empty series. Falling back to full series (no crop).');
    A = S;
end
end

function mask2 = keep_longest_true_segment(mask)
mask = mask(:);
d = diff([false; mask; false]);
st = find(d==1);
ed = find(d==-1)-1;
if isempty(st)
    mask2 = mask;
    return;
end
len = ed - st + 1;
[~,i] = max(len);
mask2 = false(size(mask));
mask2(st(i):ed(i)) = true;
end

function S2 = apply_mask(S, mask)
S2 = S;
fn = fieldnames(S);
for i=1:numel(fn)
    f = fn{i};
    x = S.(f);
    try
        if isnumeric(x) || islogical(x)
            if size(x,1) == numel(mask)
                S2.(f) = x(mask,:);
            end
        elseif iscell(x)
            if numel(x) == numel(mask)
                S2.(f) = x(mask);
            end
        elseif isstring(x)
            if numel(x) == numel(mask)
                S2.(f) = x(mask);
            end
        end
    catch
        % ignore
    end
end
end

function A = crop_time_range(A, tr)
t = A.t(:);
mask = (t >= tr(1)) & (t <= tr(2));
A = apply_mask(A, mask);
end

% ---- (optional) event-based alignment (not used by default) ----
function [A0, A1] = align_by_start(A0, A1, opt)
t0 = detect_start_time(A0, opt);
t1 = detect_start_time(A1, opt);

A0 = crop_around(A0, t0, opt);
A1 = crop_around(A1, t1, opt);

A0.t = A0.t - t0;
A1.t = A1.t - t1;
end

function t_start = detect_start_time(A, opt)
t = A.t(:);
if isempty(t)
    t_start = 0;
    return;
end
t_start = t(1);
dt = median(diff(t)); if ~isfinite(dt) || dt<=0, dt=0.025; end

mode = string(opt.align_mode);

if mode == "ref_move"
    if isfield(A,'xd') && ~isempty(A.xd)
        pref = A.xd(:,1:3);
    elseif isfield(A,'pref') && ~isempty(A.pref)
        pref = A.pref(:,1:3);
    else
        pref = A.p(:,1:3);
    end
    dp = vecnorm(pref - pref(1,:), 2, 2);
    k = find(dp > opt.align_ref_eps, 1, 'first');

elseif mode == "speed_rise"
    vrxy = A.vL(:,1:2) - A.v(:,1:2);
    s = vecnorm(vrxy,2,2);
    s_lp = movmean(s, max(3, round(0.5/dt)));
    k = find(s_lp > opt.align_speed_eps, 1, 'first');
else
    k = 1;
end

if ~isempty(k)
    t_start = t(k);
end
end

function A = crop_around(A, t0, opt)
t = A.t(:);
tmin = t0 - opt.pre_sec;
tmax = t0 + opt.post_sec;
mask = (t >= tmin) & (t <= tmax);
A = apply_mask(A, mask);
end

function out = compute_metrics(S, theta_max, opt)
t = S.t(:);
p  = S.p;   v  = S.v;
pL = S.pL;  vL = S.vL;

r  = pL - p;
vr = vL - v;
rxy  = r(:,1:2);
vrxy = vr(:,1:2);

vsway = vecnorm(vrxy,2,2);
rxy_n = vecnorm(rxy,2,2);

% swing angle magnitude: angle between cable direction and vertical-down
rz   = r(:,3);                        % r = pL - p
rho  = vecnorm(r(:,1:2), 2, 2);       % ||r_xy||
den  = max(1e-9, -rz);                % assumes load is below drone (rz<0)
theta = atan2(rho, den);              % [rad]

dt = median(diff(t));
if ~isfinite(dt) || dt<=0, dt=0.025; end

out.t = t;
out.theta = theta;
out.vsway = vsway;
out.rxy = rxy_n;

out.theta_max_deg = rad2deg(max(theta));
out.theta_rms_deg = rad2deg(rms(theta));
out.theta_p95_deg = rad2deg(prctile(theta,95));

out.vsway_rms = rms(vsway);
out.vsway_p95 = prctile(vsway,95);
out.vsway_energy = sum(vsway.^2)*dt;

out.violation_ratio = mean(theta > theta_max);

% settle time: first time after which theta stays within 1 deg band
band = deg2rad(1.0);
k_set = find_settle_time(t, theta, band);
if isempty(k_set)
    out.settle_time_s = NaN;
else
    out.settle_time_s = t(k_set);
end

% additional scalars
out.max_rxy = max(rxy_n);
out.rxy_rms = rms(rxy_n);

% envelope of S(t)
S_raw = vsway + opt.sr * rxy_n;
win = max(1, round(opt.env_win_sec / dt));
out.S = S_raw;
out.S_env = movmax(S_raw, win);
% ---- additional decision-rate metrics (only if theta/vr thresholds exist) ----
has_theta_vr = isfield(opt,'use_theta_vr_switch') && opt.use_theta_vr_switch ...
    && isfield(opt,'theta_on') && isfield(opt,'theta_off') ...
    && isfield(opt,'vr_on') && isfield(opt,'vr_off');

if has_theta_vr
    theta_on_mask = (theta > opt.theta_on);
    vr_on_mask    = (vsway > opt.vr_on);
    oncond_mask   = theta_on_mask | vr_on_mask;

    out.theta_on_rate = mean(theta_on_mask);
    out.vr_on_rate    = mean(vr_on_mask);
    out.oncond_rate   = mean(oncond_mask);

    % 参考：ON条件が連続でどれくらい続くか（最大連続時間）
    out.oncond_max_run_s = max_true_run_sec(oncond_mask, dt);
else
    out.theta_on_rate = NaN;
    out.vr_on_rate    = NaN;
    out.oncond_rate   = NaN;
    out.oncond_max_run_s = NaN;
end
% ---- additional: theta/vr threshold rates (for switching comparison) ----
if isfield(opt,'use_theta_vr_switch') && opt.use_theta_vr_switch ...
        && isfield(opt,'theta_on') && isfield(opt,'theta_off') ...
        && isfield(opt,'vr_on') && isfield(opt,'vr_off')

    out.theta_on_rate = mean(theta > opt.theta_on);
    out.theta_off_rate = mean(theta > opt.theta_off);   % (参考) off閾値超え率
    out.vr_on_rate    = mean(vsway > opt.vr_on);
    out.vr_off_rate   = mean(vsway > opt.vr_off);       % (参考)
    out.oncond_rate   = mean( (theta > opt.theta_on) | (vsway > opt.vr_on) );
else
    out.theta_on_rate = NaN;
    out.theta_off_rate = NaN;
    out.vr_on_rate = NaN;
    out.vr_off_rate = NaN;
    out.oncond_rate = NaN;
end
end

function k = find_settle_time(t, x, band)
k = [];
if isempty(t), return; end
for i=1:numel(t)
    if all(abs(x(i:end)) <= band)
        k = i;
        return;
    end
end
end

function paper_plots(A0, A1, no, on, theta_max, opt)

has_theta_vr = isfield(opt,'use_theta_vr_switch') && opt.use_theta_vr_switch ...
    && isfield(opt,'theta_on') && isfield(opt,'theta_off') ...
    && isfield(opt,'vr_on') && isfield(opt,'vr_off');

% 1) θ overlay
figure;
h1 = plot(no.t, rad2deg(no.theta), 'DisplayName','OFF'); hold on;
h2 = plot(on.t, rad2deg(on.theta), 'DisplayName','ON');
h3 = yline(rad2deg(theta_max),'--', 'DisplayName','\theta_{max}');

% danger線（theta_max - theta_gate）
if isfield(opt,'theta_max') && isfield(opt,'theta_gate') ...
        && isfinite(opt.theta_max) && isfinite(opt.theta_gate)
    yline(rad2deg(opt.theta_max - opt.theta_gate), ':', 'DisplayName','danger thresh');
end

% ★揺れ判定（theta_on/off）
if has_theta_vr
    yline(rad2deg(opt.theta_on),  '--', 'DisplayName','theta\_on');
    yline(rad2deg(opt.theta_off), ':',  'DisplayName','theta\_off');
end

grid on; xlabel('t (aligned) [s]'); ylabel('\theta [deg]');
legend('Location','best');
title('Swing angle');



% 2) vsway overlay: ||v_{r,xy}|| [m/s]
figure;
h1 = plot(no.t, no.vsway, 'DisplayName','OFF'); hold on;
h2 = plot(on.t, on.vsway, 'DisplayName','ON');

% ★ switching thresholds: vr_on/off
if has_theta_vr
    yline(opt.vr_on,  '--', 'DisplayName','vr\_on');
    yline(opt.vr_off, ':',  'DisplayName','vr\_off');
end

grid on; xlabel('t (aligned) [s]'); ylabel('||v_{r,xy}|| [m/s]');
legend('show','Location','best');
title('Relative horizontal speed');


% 3) Energy overlay (cumulative)  ∫||vr||^2 dt
E0 = cumtrapz(no.t, no.vsway.^2);
E1 = cumtrapz(on.t, on.vsway.^2);
figure;
plot(no.t, E0, 'DisplayName','OFF'); hold on;
plot(on.t, E1, 'DisplayName','ON');
grid on; xlabel('t (aligned) [s]'); ylabel('\int ||v_{r,xy}||^2 dt');
legend('show','Location','best');
title('Sway energy (cumulative)');


% 4) S envelope overlay
figure;
plot(no.t, no.S_env, 'DisplayName','OFF'); hold on;
plot(on.t, on.S_env, 'DisplayName','ON');
grid on; xlabel('t (aligned) [s]'); ylabel('S envelope');
legend('show','Location','best');
title('S envelope (movmax)');


% 5) phase portrait (theta vs theta_dot) [deg, deg/s]
dt0 = median(diff(no.t)); if ~isfinite(dt0)||dt0<=0, dt0=0.025; end
dt1 = median(diff(on.t)); if ~isfinite(dt1)||dt1<=0, dt1=0.025; end
thd0 = [0; diff(no.theta)/dt0];
thd1 = [0; diff(on.theta)/dt1];

figure;
plot(rad2deg(no.theta), rad2deg(thd0), 'DisplayName','OFF'); hold on;
plot(rad2deg(on.theta), rad2deg(thd1), 'DisplayName','ON');
grid on; xlabel('\theta [deg]'); ylabel('\dot{\theta} [deg/s]');
legend('show','Location','best');
title('Phase portrait (approx)');
end


function paper_plots_single(A0, no, theta_max, opt)

has_theta_vr = isfield(opt,'use_theta_vr_switch') && opt.use_theta_vr_switch ...
    && isfield(opt,'theta_on') && isfield(opt,'theta_off') ...
    && isfield(opt,'vr_on') && isfield(opt,'vr_off');

% 1) θ
figure;
plot(no.t, rad2deg(no.theta), 'DisplayName','data'); hold on;
yline(rad2deg(theta_max),'--', 'DisplayName','\theta_{max}');

if isfield(opt,'theta_gate') && ~isempty(opt.theta_gate) && isfinite(opt.theta_gate)
    yline(rad2deg(theta_max - opt.theta_gate), ':', 'DisplayName','danger thresh');
end
if has_theta_vr
    yline(rad2deg(opt.theta_on),  '--', 'DisplayName','\theta_{on}');
    yline(rad2deg(opt.theta_off), ':',  'DisplayName','\theta_{off}');
end

grid on; xlabel('t (aligned) [s]'); ylabel('\theta [deg]');
legend('show','Location','best');
title('Swing angle');

% 2) vsway
figure;
plot(no.t, no.vsway, 'DisplayName','data'); hold on;
if has_theta_vr
    yline(opt.vr_on,  '--', 'DisplayName','vr_{on}');
    yline(opt.vr_off, ':',  'DisplayName','vr_{off}');
end
grid on; xlabel('t (aligned) [s]'); ylabel('||v_{r,xy}|| [m/s]');
legend('show','Location','best');
title('Relative horizontal speed');

% 3) Energy cumulative
E0 = cumtrapz(no.t, no.vsway.^2);
figure;
plot(no.t, E0, 'DisplayName','data');
grid on; xlabel('t (aligned) [s]'); ylabel('\int ||v_{r,xy}||^2 dt');
title('Sway energy (cumulative)');

% 4) S envelope
figure;
plot(no.t, no.S_env, 'DisplayName','data');
grid on; xlabel('t (aligned) [s]'); ylabel('S envelope');
title('S envelope (movmax)');

% 5) phase portrait
dt0 = median(diff(no.t)); if ~isfinite(dt0)||dt0<=0, dt0=0.025; end
thd0 = [0; diff(no.theta)/dt0];
figure;
plot(rad2deg(no.theta), rad2deg(thd0), 'DisplayName','data');
grid on; xlabel('\theta [deg]'); ylabel('\dot{\theta} [deg/s]');
title('Phase portrait (approx)');
end

function dur = max_true_run_sec(mask, dt)
mask = mask(:);
d = diff([false; mask; false]);
st = find(d==1);
ed = find(d==-1)-1;
if isempty(st)
    dur = 0;
    return;
end
len = ed - st + 1;
dur = max(len) * dt;
end

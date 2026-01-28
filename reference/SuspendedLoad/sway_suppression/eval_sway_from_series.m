function out = eval_sway_from_series(S, theta_max, tag, opt)
% eval_sway_from_series
% S        : extract_series_from_logmat_cellresult の出力
% theta_max: 許容角 [rad]
% tag      : 表示用文字列（省略可）
% opt      : struct（省略可）
%   opt.t_range        = [t0 t1]
%   opt.use_sway_only  = true/false

if nargin < 3 || isempty(tag)
    tag = "";
end
if nargin < 4
    opt = struct();
end
if ~isfield(opt,'t_range')
    opt.t_range = [];
end
if ~isfield(opt,'use_sway_only')
    opt.use_sway_only = true;
end

t = S.t(:);

% ---- 区間指定 ----
idx = true(size(t));
if ~isempty(opt.t_range)
    t0 = opt.t_range(1);
    t1 = opt.t_range(2);
    idx = (t >= t0) & (t <= t1);
end

t  = t(idx);
p  = S.p(idx,:);   v  = S.v(idx,:);
pL = S.pL(idx,:);  vL = S.vL(idx,:);

% ---- 相対量 ----
r  = pL - p;
vr = vL - v;

rxy  = r(:,1:2);
vrxy = vr(:,1:2);

rxy_norm   = vecnorm(rxy,2,2);
vsway_norm = vecnorm(vrxy,2,2);

% ---- 揺れ角（L不要・堅牢）----
r_norm = vecnorm(r,2,2);
cos_th = (-r(:,3)) ./ max(r_norm,1e-9);
cos_th = min(max(cos_th,-1),1);
theta  = acos(cos_th);

dt = median(diff(t));
if ~isfinite(dt) || dt<=0
    dt = 0.025;
end

out.t = t;
out.theta = theta;
out.vsway = vsway_norm;
out.rxy   = rxy_norm;

out.theta_max_deg = rad2deg(max(theta));
out.theta_rms_deg = rad2deg(rms(theta));
out.vsway_rms     = rms(vsway_norm);
out.vsway_energy  = sum(vsway_norm.^2)*dt;
out.violation_ratio = mean(theta > theta_max);

% ---- 揺れ成分のみ（高周波）----
if opt.use_sway_only
    win_sec = 0.8;                     % 揺れ周期の半分〜1倍程度
    win = max(3, round(win_sec/dt));
    vs_lp = movmean(vsway_norm, win);
    vs_hp = vsway_norm - vs_lp;

    out.vsway_hp_rms    = rms(vs_hp);
    out.vsway_hp_energy = sum(vs_hp.^2)*dt;
end

% ---- 表示 ----
fprintf('\n=== sway metrics: %s ===\n', tag);
if ~isempty(opt.t_range)
    fprintf('t_range = [%.2f, %.2f] s\n', opt.t_range(1), opt.t_range(2));
end
fprintf('theta_max: %.3f deg, theta_rms: %.3f deg\n', ...
    out.theta_max_deg, out.theta_rms_deg);
fprintf('vsway_rms: %.4f m/s, int(v^2)dt: %.4f\n', ...
    out.vsway_rms, out.vsway_energy);

if opt.use_sway_only
    fprintf('vsway_hp_rms: %.4f m/s, int(v_hp^2)dt: %.4f\n', ...
        out.vsway_hp_rms, out.vsway_hp_energy);
end
fprintf('violation_ratio(theta>theta_max): %.4f\n', ...
    out.violation_ratio);

% ---- 図 ----
figure; plot(t, rad2deg(theta)); grid on; hold on;
yline(rad2deg(theta_max),'--');
xlabel('t [s]'); ylabel('\theta [deg]');
title("Sway angle " + tag);

figure; plot(t, vsway_norm); grid on;
xlabel('t [s]'); ylabel('|v_L - v|_{xy} [m/s]');
title("Relative horizontal speed " + tag);

if opt.use_sway_only
    figure; plot(t, vs_hp); grid on;
    xlabel('t [s]'); ylabel('|v|_{hp} [m/s]');
    title("High-pass sway component " + tag);
end
end

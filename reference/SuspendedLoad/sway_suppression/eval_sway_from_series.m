function out = eval_sway_from_series(S_no, S_on, theta_max, opt)
% plot_sway_compare_from_series
% - extract_series_from_logmat_cellresult の出力（S_no, S_on）だけで
%   S, S envelope, theta, energy を比較プロットする
% - sway_on(矩形)は、Sから擬似的に再構成（ヒステリシス+min_on_time）

if nargin < 4, opt = struct(); end
if ~isfield(opt,'t_range'), opt.t_range = []; end
if ~isfield(opt,'S_on'), opt.S_on = 0.25; end
if ~isfield(opt,'S_off'), opt.S_off = 0.15; end
if ~isfield(opt,'sr'), opt.sr = 0.3; end
if ~isfield(opt,'env_win_sec'), opt.env_win_sec = 0.5; end
if ~isfield(opt,'min_on_time'), opt.min_on_time = 0.5; end

% ---- それぞれ解析 ----
no = compute_metrics_from_series(S_no, theta_max, opt);
on = compute_metrics_from_series(S_on, theta_max, opt);

out.no = no;
out.on = on;

% ---- 1) S(t) + sway_on(擬似) ----
figure;
yyaxis left;  plot(no.t, no.S, '-', 'DisplayName','S (no)'); hold on;
plot(on.t, on.S, '-', 'DisplayName','S (on)');
ylabel('S');

yyaxis right;
plot(no.t, no.sway_on_hat, '--', 'DisplayName','sway\_on^ (no)');
plot(on.t, on.sway_on_hat, '--', 'DisplayName','sway\_on^ (on)');
ylabel('sway\_on^'); ylim([-0.2 1.2]);

grid on; xlabel('t [s]'); title('S and estimated sway\_on');
legend('Location','best');

% ---- 2) S envelope (movmax) ----
figure;
plot(no.t, no.S, '-', 'DisplayName','S (no)'); hold on;
plot(no.t, no.S_env, '-', 'DisplayName','S env (no)');
plot(on.t, on.S, '-', 'DisplayName','S (on)');
plot(on.t, on.S_env, '-', 'DisplayName','S env (on)');
grid on; xlabel('t [s]'); ylabel('S');
title('S envelope (movmax)');
legend('Location','best');

% ---- 3) theta vs theta_max ----
figure;
plot(no.t, rad2deg(no.theta), '-', 'DisplayName','\theta (no)'); hold on;
plot(on.t, rad2deg(on.theta), '-', 'DisplayName','\theta (on)');
yline(rad2deg(theta_max),'--','DisplayName','\theta_{max}');
grid on; xlabel('t [s]'); ylabel('\theta [deg]');
title('Swing angle \theta');
legend('Location','best');

% ---- 4) energy cumulative ----
figure;
plot(no.t, no.Evr, '-', 'DisplayName','Evr (no)'); hold on;
plot(on.t, on.Evr, '-', 'DisplayName','Evr (on)');
grid on; xlabel('t [s]'); ylabel('\int ||v_{r,xy}||^2 dt');
title('Sway energy (cumulative)');
legend('Location','best');

% ---- 数値比較 ----
fprintf('\n=== comparison (from .mat series only) ===\n');
fprintf('theta_rms [deg]  no=%.3f  on=%.3f  ratio=%.3f\n', ...
    rad2deg(no.theta_rms), rad2deg(on.theta_rms), on.theta_rms/no.theta_rms);
fprintf('violation_ratio  no=%.3f  on=%.3f  ratio=%.3f\n', ...
    no.violation_ratio, on.violation_ratio, on.violation_ratio/max(no.violation_ratio,1e-9));
fprintf('vsway_rms [m/s]  no=%.4f  on=%.4f  ratio=%.3f\n', ...
    no.vsway_rms, on.vsway_rms, on.vsway_rms/no.vsway_rms);
fprintf('E_total          no=%.4f  on=%.4f  ratio=%.3f\n', ...
    no.E_total, on.E_total, on.E_total/max(no.E_total,1e-9));

end

% ======================================================================
function out = compute_metrics_from_series(S, theta_max, opt)
t = S.t(:);

% ---- 区間指定 ----
idx = true(size(t));
if ~isempty(opt.t_range)
    idx = (t >= opt.t_range(1)) & (t <= opt.t_range(2));
end

t  = t(idx);
p  = S.p(idx,:);   v  = S.v(idx,:);
pL = S.pL(idx,:);  vL = S.vL(idx,:);

r  = pL - p;
vr = vL - v;

rxy  = r(:,1:2);
vrxy = vr(:,1:2);

rxy_norm   = vecnorm(rxy,2,2);
vsway_norm = vecnorm(vrxy,2,2);

% ---- S（SWAYで使ってた定義と同型）----
Sval = vsway_norm + opt.sr * rxy_norm;

% ---- theta（あなたのeval関数の定義を採用：L不要で頑丈）----
r_norm = vecnorm(r,2,2);
cos_th = (-r(:,3)) ./ max(r_norm,1e-9);
cos_th = min(max(cos_th,-1),1);
theta  = acos(cos_th);

% dt推定
dt = median(diff(t));
if ~isfinite(dt) || dt<=0
    dt = 0.025;
end

% ---- S envelope ----
win = max(1, round(opt.env_win_sec/dt));
S_env = movmax(Sval, win);

% ---- 擬似 sway_on（ヒステリシス+min_on_time）----
sway_on_hat = reconstruct_onoff(Sval, t, opt.S_on, opt.S_off, opt.min_on_time);

% ---- cumulative energy (Evr) ----
vr2 = vsway_norm.^2;
Evr = cumsum(vr2) * dt;

% ---- summary ----
out.t = t;
out.S = Sval;
out.S_env = S_env;
out.theta = theta;
out.vsway = vsway_norm;
out.rxy = rxy_norm;
out.sway_on_hat = sway_on_hat;
out.Evr = Evr;

out.theta_rms = rms(theta);
out.vsway_rms = rms(vsway_norm);
out.violation_ratio = mean(theta > theta_max);
out.E_total = Evr(end);

end

% ======================================================================
function on = reconstruct_onoff(S, t, S_on, S_off, min_on_time)
% S: Nx1, t: Nx1
on = zeros(size(S));
state = 0;
t_on = -inf;

for k = 1:numel(S)
    if state == 0
        if S(k) > S_on
            state = 1;
            t_on = t(k);
        end
    else
        % min_on_time中はOFFにしない
        if (t(k) - t_on) >= min_on_time
            if S(k) < S_off
                state = 0;
            end
        end
    end
    on(k) = state;
end
end


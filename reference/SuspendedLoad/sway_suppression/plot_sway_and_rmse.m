function OUT = plot_sway_and_rmse(app, opt)
% 実験後すぐ：SWAYログ＋RMSEをまとめて出す（型依存しない版）

if nargin<2, opt=struct(); end
opt = setdef(opt,'flight_only', true);
opt = setdef(opt,'t_range', []);
opt = setdef(opt,'phase_char', 'f');
opt = setdef(opt,'env_win_sec', 0.5);

A = app;
sw = A.agent.reference.swaymod;
D  = A.logger.Data;

% ===== SWAYログ =====
valid = ~isnan(sw.t_log);
t_sw  = sw.t_log(valid);

S     = sw.S_log(valid);
on    = sw.sway_on_log(valid);
theta = sw.theta_log(valid);
Evr   = sw.Evr_log(valid);
viol  = sw.violate_log(valid);

% theta_max
if isfield(sw.param,'theta_max_deg') && ~isempty(sw.param.theta_max_deg)
    theta_max = deg2rad(sw.param.theta_max_deg);
else
    theta_max = sw.param.theta_max;
end

% ===== flight mask（logger側）=====
t  = D.t(:);
ph = D.phase(:);
ph_f = double(opt.phase_char); % 'f' -> 102
mask_f = (ph == ph_f);

if opt.flight_only && ~any(mask_f)
    warning('flight_only=true ですが phase==%d が0件です。flight抽出をスキップします。', ph_f);
    mask_f = true(size(t));
end

k0 = find(mask_f, 1, 'first');
t0 = t(k0);
t_rel = t - t0;

mask_tr = true(size(t));
if ~isempty(opt.t_range)
    mask_tr = (t_rel >= opt.t_range(1)) & (t_rel <= opt.t_range(2));
end
mask = mask_f & mask_tr;

% ===== 変更前目標（SWAYログから）→ logger時刻へ補間 =====
pref_org_sw = sw.xd_origin_log(valid,1:3);  % Nsw x 3

% (A) sw.t_log は重複が出ることがあるので dt 格子に丸める
dt_sw = sw.param.dt;
t_sw_q = round(t_sw/dt_sw) * dt_sw;

% (B) 重複時刻を平均で潰す（列ごと）
[t_sw_u, ~, ic] = unique(t_sw_q, 'stable');
pref_org_sw_u = [ ...
    accumarray(ic, pref_org_sw(:,1), [], @mean), ...
    accumarray(ic, pref_org_sw(:,2), [], @mean), ...
    accumarray(ic, pref_org_sw(:,3), [], @mean)  ];

% (C) interp1 は x が単調増加を要求するので sort
[t_sw_u, is] = sort(t_sw_u);
pref_org_sw_u = pref_org_sw_u(is,:);

% (D) 補間（外側は NaN）
pref_org = interp1(t_sw_u, pref_org_sw_u, t, "linear", NaN); % Nlogger x 3


% ===== 実位置 p_hat を "どの型でも" 抜く =====
% ここが今回の肝：D.agent.estimator.result の中から Nx3 を作る
p_hat = extract_pos_series(D.agent.estimator.result, numel(t));

% ===== RMSE =====
e = p_hat(mask,:) - pref_org(mask,:);
rmse_xyz = sqrt(mean(e.^2, 1, 'omitnan'));
rmse_xy  = sqrt(mean(sum(e(:,1:2).^2,2), 'omitnan'));
rmse_3d  = sqrt(mean(sum(e.^2,2), 'omitnan'));

fprintf('\n=== RMSE (pref origin vs actual) ===\n');
fprintf('phase = "%s" only\n', opt.phase_char);
if ~isempty(opt.t_range)
    fprintf('t_range (relative) = [%.2f, %.2f] s\n', opt.t_range(1), opt.t_range(2));
else
    fprintf('t_range (relative) = ALL\n');
end
fprintf('Drone RMSE xyz [m] = [%g %g %g]\n', rmse_xyz(1), rmse_xyz(2), rmse_xyz(3));
fprintf('Drone RMSE xy  [m] = %g\n', rmse_xy);
fprintf('Drone RMSE 3D  [m] = %g\n', rmse_3d);

% ===== SWAYグラフ（flight開始に合わせた相対時間表示）=====
t_sw_rel = t_sw - t0;

mask_sw = true(size(t_sw_rel));
if opt.flight_only
    mask_sw = (t_sw_rel >= 0);
end
if ~isempty(opt.t_range)
    mask_sw = mask_sw & (t_sw_rel >= opt.t_range(1)) & (t_sw_rel <= opt.t_range(2));
end

tP = t_sw_rel(mask_sw);
SP = S(mask_sw);
onP = on(mask_sw);
thetaP = theta(mask_sw);
EvrP = Evr(mask_sw);
violP = viol(mask_sw);

% 1) S と sway_on
figure;
yyaxis left;  plot(tP, SP, 'DisplayName','S'); ylabel('S');
yyaxis right; plot(tP, onP, 'DisplayName','sway\_on'); ylabel('sway\_on'); ylim([-0.2 1.2]);
grid on; xlabel('t (flight-aligned) [s]'); title('S and sway\_on');
legend('Location','best');

% 2) S envelope
dt_sw = sw.param.dt;
win = max(1, round(opt.env_win_sec / dt_sw));
S_env = movmax(SP, win);

figure;
plot(tP, SP, 'DisplayName','S'); hold on;
plot(tP, S_env, 'DisplayName','S envelope (movmax)');
grid on; xlabel('t (flight-aligned) [s]'); ylabel('S');
legend('Location','best'); title('S envelope');

% 3) theta
figure;
plot(tP, thetaP, 'DisplayName','\theta'); hold on;
yline(theta_max,'--','DisplayName','\theta_{max}');
grid on; xlabel('t (flight-aligned) [s]'); ylabel('\theta [rad]');
legend('Location','best'); title('Swing angle \theta');

violation_ratio = mean(violP,'omitnan');
fprintf('violation_ratio(theta>theta_max) = %.3f\n', violation_ratio);

% 4) energy
figure;
plot(tP, EvrP, 'DisplayName','Evr'); grid on;
xlabel('t (flight-aligned) [s]'); ylabel('\int ||v_{r,xy}||^2 dt');
title('Sway energy (cumulative)');
legend('Location','best');

OUT.rmse_xyz = rmse_xyz;
OUT.rmse_xy  = rmse_xy;
OUT.rmse_3d  = rmse_3d;
OUT.p_hat    = p_hat;
OUT.pref_org = pref_org;
end

% ===== helper =====
function opt = setdef(opt, name, val)
if ~isfield(opt,name) || isempty(opt.(name))
    opt.(name) = val;
end
end

function p_hat = extract_pos_series(est_result, N)
% est_result が struct/object/セル/配列 どれでも Nx3 の位置系列を作る
p_hat = NaN(N,3);

% --- case 1: cell array ---
if iscell(est_result)
    M = min(N, numel(est_result));
    for k=1:M
        p_hat(k,:) = get_pos_from_any(est_result{k}).';
    end
    return;
end

% --- case 2: struct array or object array ---
if isstruct(est_result) || isobject(est_result)
    M = min(N, numel(est_result));
    for k=1:M
        p_hat(k,:) = get_pos_from_any(est_result(k)).';
    end
    return;
end

% --- case 3: numeric matrix (N x >=3) ---
if isnumeric(est_result) && ismatrix(est_result) && size(est_result,2) >= 3
    M = min(N, size(est_result,1));
    p_hat(1:M,:) = est_result(1:M,1:3);
    return;
end

% --- case 4: timeseries ---
if isa(est_result,'timeseries')
    x = est_result.Data;
    if isnumeric(x) && size(x,2) >= 3
        M = min(N, size(x,1));
        p_hat(1:M,:) = x(1:M,1:3);
        return;
    end
end

warning('extract_pos_series: unsupported estimator.result type: %s', class(est_result));
end

function p = get_pos_from_any(obj)
% obj の中から position(3) を探す
% 優先順位：
%  1) obj.state.p
%  2) obj.state.x(1:3)
%  3) obj.p
%  4) obj.x(1:3)
%  5) struct field candidates (pos, position, p, x)
p = [NaN;NaN;NaN];

% unwrap if it has .state
st = [];
if isstruct(obj)
    if isfield(obj,'state'), st = obj.state; end
elseif isobject(obj)
    if isprop(obj,'state'), st = obj.state; end
end
if ~isempty(st)
    p = get_pos_from_any(st); % 再帰
    if all(isfinite(p)), return; end
end

% direct p
v = try_get(obj,'p');
if isnumeric(v) && numel(v)>=3, p = v(1:3); return; end

% direct x
x = try_get(obj,'x');
if isnumeric(x) && numel(x)>=3, p = x(1:3); return; end

% other common names
v = try_get(obj,'pos');
if isnumeric(v) && numel(v)>=3, p = v(1:3); return; end
v = try_get(obj,'position');
if isnumeric(v) && numel(v)>=3, p = v(1:3); return; end

end

function v = try_get(obj, name)
v = [];
try
    if isobject(obj)
        if isprop(obj,name), v = obj.(name); end
    elseif isstruct(obj)
        if isfield(obj,name), v = obj.(name); end
    end
catch
    v = [];
end
end

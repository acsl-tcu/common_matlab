%% compare_mec_plot : 円・レムニスケート軌道の A/B 比較図 + 高度曲線図 + RMSE総括表
%  黒=Nominal(無payload実飛行), 赤=Nominal with error, 緑=Nominal + EDMD-MEC,
%  灰点線=Reference (legend入り)
%  コンソールに軌道ごとのRMSE総括表: 3D / xy / z × (nom, err, MEC) + 派生指標
clearvars -except agent; clc; close all;

%% ===== ここだけ手動編集 =====
REPO = 'C:\Users\student\Documents\GitHub\common_matlab';
CIRCLE_NOM = 'lpvmpc_circle_90s_Log(16-Jul-2026_16_10_14)';
CIRCLE_ERR = 'payload_lpvmpc_circle_89s_Log(16-Jul-2026_15_35_34)';
CIRCLE_MEC = 'lpvmpccirclecomp_Log(30-Jul-2026_15_35_31)';
LEM_NOM    = 'lpvmpc_figure_87s_Log(16-Jul-2026_16_13_32)';
LEM_ERR    = 'payload_lpvmpc_figure_85s_Log(16-Jul-2026_16_00_07)';
LEM_MEC    = 'lpvmpcfigure8comp_Log(30-Jul-2026_15_30_25)';
WARMUP_SEC = 5.0;
DT = 0.025;
%% ===========================

if isempty(which('LOGGER'))
    assert(isfolder(REPO), 'REPOパスを修正せよ');
    addpath(regexprep(genpath(REPO), '[^;]*\.git[^;]*;', ''));
end
assert(~isempty(which('LOGGER')), 'LOGGERが見つからない');

C_NOM = [0 0 0]; C_ERR = [0.86 0.22 0.15]; C_MEC = [0.00 0.62 0.36]; C_REF = [0.55 0.55 0.55];

make_all('circle', CIRCLE_NOM, CIRCLE_ERR, CIRCLE_MEC, WARMUP_SEC, DT, C_NOM, C_ERR, C_MEC, C_REF);
make_all('lemniscate', LEM_NOM, LEM_ERR, LEM_MEC, WARMUP_SEC, DT, C_NOM, C_ERR, C_MEC, C_REF);

%% ======================= 局所関数 =======================
function make_all(name, f_nom, f_err, f_mec, wu, dt, C_NOM, C_ERR, C_MEC, C_REF)
[pe_n, pr_n] = load_xy(f_nom, wu, dt);
[pe_e, pr_e] = load_xy(f_err, wu, dt);
[pe_m, pr_m] = load_xy(f_mec, wu, dt);
N = min([size(pe_n,2), size(pe_e,2), size(pe_m,2)]);
pe_n=pe_n(:,1:N); pr_n=pr_n(:,1:N); pe_e=pe_e(:,1:N); pr_e=pr_e(:,1:N); pe_m=pe_m(:,1:N); pr_m=pr_m(:,1:N);

% --- RMSE 8本: ①②③=参照軌道基準, ④=Nominal実飛行基準, 各 xy / z ---
r = @(pe, pr, d) sqrt(mean(sum((pe(d,:) - pr(d,:)).^2, 1)));
N1xy = r(pe_n,pr_n,1:2); N1z = r(pe_n,pr_n,3); N13 = r(pe_n,pr_n,1:3);   % ① Nominal
E2xy = r(pe_e,pr_e,1:2); E2z = r(pe_e,pr_e,3); E23 = r(pe_e,pr_e,1:3);   % ② Error
M3xy = r(pe_m,pr_m,1:2); M3z = r(pe_m,pr_m,3); M33 = r(pe_m,pr_m,1:3);   % ③ MEC
lag_m = best_lag(pr_n, pr_m);                                            % ④ MEC vs Nominal
[an, am] = align_pair(pe_n, pe_m, lag_m);
V4xy = r(am,an,1:2); V4z = r(am,an,3); V43 = r(am,an,1:3);

pct = @(a_, b_) (a_ - b_) / max(a_, 1e-9) * 100;
recv = @(n_, e_, m_) (e_ - m_) / max(e_ - n_, 1e-9) * 100;

fprintf('\n===== %s RMSE総括 (評価 %.1fs) =====\n', name, N*dt);
fprintf('[参照軌道(reference)基準]        xy        z       (3D)\n');
fprintf(' ① Nominal                  %7.3f   %7.3f   %7.3f [m]\n', N1xy, N1z, N13);
fprintf(' ② Nominal with error       %7.3f   %7.3f   %7.3f [m]\n', E2xy, E2z, E23);
fprintf(' ③ Nominal + EDMD-MEC       %7.3f   %7.3f   %7.3f [m]\n', M3xy, M3z, M33);
fprintf('[Nominal実飛行基準] (lag=%+d拍)\n', lag_m);
fprintf(' ④ MEC vs Nominal           %7.3f   %7.3f   %7.3f [m]\n', V4xy, V4z, V43);
fprintf('--- 改善率 ---\n');
fprintf(' 補償効果   ③vs②          %6.1f%%   %6.1f%%   %6.1f%%\n', pct(E2xy,M3xy), pct(E2z,M3z), pct(E23,M33));
fprintf(' 対Nominal  ③vs① (正=Nominalより参照に近い)\n');
fprintf('                            %+6.1f%%  %+6.1f%%  %+6.1f%%\n', pct(N1xy,M3xy), pct(N1z,M3z), pct(N13,M33));
fprintf(' 回復率 (②の劣化を何割回復) %6.1f%%   %6.1f%%   %6.1f%%\n', ...
    recv(N1xy,E2xy,M3xy), recv(N1z,E2z,M3z), recv(N13,E23,M33));
fprintf(' 形状距離(xy, 位相非依存)   Error %.3f → MEC %.3f m (対Nominal軌跡)\n', ...
    shape_dist(pe_e(1:2,:), pe_n(1:2,:)), shape_dist(pe_m(1:2,:), pe_n(1:2,:)));
fprintf(' 形状誤差(参照基準) ① %.3f  ② %.3f  ③ %.3f m → 補償効果 %.1f%%\n', ...
    shape_dist(pe_n(1:2,:), pr_n(1:2,:)), shape_dist(pe_e(1:2,:), pr_e(1:2,:)), ...
    shape_dist(pe_m(1:2,:), pr_m(1:2,:)), ...
    pct(shape_dist(pe_e(1:2,:), pr_e(1:2,:)), shape_dist(pe_m(1:2,:), pr_m(1:2,:))));
Rxy = [N1xy, E2xy, M3xy]; Rz = [N1z, E2z, M3z];
imp = @(v) (v(2)-v(3))/max(v(2),1e-9)*100;

% --- xy軌跡図 ---
figure('Color','w','Position',[80 80 700 620]);
plot(pr_m(1,:), pr_m(2,:), ':', 'Color', C_REF, 'LineWidth', 1.2); hold on;
plot(pe_n(1,:), pe_n(2,:), '-', 'Color', C_NOM, 'LineWidth', 1.6);
plot(pe_e(1,:), pe_e(2,:), '-', 'Color', C_ERR, 'LineWidth', 1.2);
plot(pe_m(1,:), pe_m(2,:), '-', 'Color', C_MEC, 'LineWidth', 1.2);
plot(pe_n(1,1), pe_n(2,1), 'o', 'MarkerFaceColor', C_NOM, 'MarkerEdgeColor', C_NOM, 'MarkerSize', 6, 'HandleVisibility', 'off');
plot(pe_e(1,1), pe_e(2,1), 'o', 'MarkerFaceColor', C_ERR, 'MarkerEdgeColor', C_ERR, 'MarkerSize', 6, 'HandleVisibility', 'off');
plot(pe_m(1,1), pe_m(2,1), 'o', 'MarkerFaceColor', C_MEC, 'MarkerEdgeColor', C_MEC, 'MarkerSize', 6, 'HandleVisibility', 'off');
axis equal; grid on;
xlabel('x [m]'); ylabel('y [m]');
legend({'Reference', 'Nominal', 'Nominal with error', 'Nominal + EDMD-MEC'}, 'Location', 'southwest');
title(sprintf('%s: Nominal / Error / EDMD-MEC  RMSE %.3f / %.3f -> %.3f m (%.1f%%)', ...
    name, Rxy(1), Rxy(2), Rxy(3), imp(Rxy)), 'FontWeight', 'bold');

% --- 高度図 ---
t = (0:N-1)*dt;
figure('Color','w','Position',[120 120 760 360]);
plot(t, pr_n(3,:), ':', 'Color', C_REF, 'LineWidth', 1.2); hold on;
plot(t, pe_n(3,:), '-', 'Color', C_NOM, 'LineWidth', 1.6);
plot(t, pe_e(3,:), '-', 'Color', C_ERR, 'LineWidth', 1.2);
plot(t, pe_m(3,:), '-', 'Color', C_MEC, 'LineWidth', 1.2);
grid on; xlim([0, t(end)]);
xlabel('Time [s]'); ylabel('z [m]');
legend({'Reference', 'Nominal', 'Nominal with error', 'Nominal + EDMD-MEC'}, 'Location', 'southeast');
title(sprintf('%s altitude: Nominal / Error / EDMD-MEC  RMSE %.3f / %.3f -> %.3f m (%.1f%%)', ...
    name, Rz(1), Rz(2), Rz(3), imp(Rz)), 'FontWeight', 'bold');
end

function L = best_lag(pr_a, pr_b)
% 2つの飛行の参照軌道が最も一致するラグ [拍] を探索 (位相合わせ)
N = min(size(pr_a,2), size(pr_b,2));
maxlag = round(N/4); best = inf; L = 0;
for l = -maxlag:maxlag
    ia = max(1, 1-l) : min(N, N-l);
    ib = ia + l;
    e = mean(sum((pr_a(:,ia) - pr_b(:,ib)).^2, 1));
    if e < best, best = e; L = l; end
end
end

function [a, b] = align_pair(pe_a, pe_b, L)
N = min(size(pe_a,2), size(pe_b,2));
ia = max(1, 1-L) : min(N, N-L);
a = pe_a(:, ia); b = pe_b(:, ia + L);
end

function dm = shape_dist(P, Q)
% Pの各点からQ軌跡への最近傍距離の平均 (位相・時間ずれに非依存な形状差)
n = size(P,2); dm = 0;
for i = 1:n
    dm = dm + min(vecnorm(Q - P(:,i)));
end
dm = dm / n;
end

function [pe, pr] = load_xy(matfile, warmup_sec, dt)
hit = dir(fullfile(pwd, '**', [char(matfile), '.mat']));
assert(~isempty(hit), 'ファイル未発見: %s.mat', matfile);
lg = LOGGER(fullfile(hit(1).folder, hit(1).name));
ph = lg.Data.phase(:)';
i102 = find(ph == 102); assert(~isempty(i102), '%s: 102なし', matfile);
blk = i102([true, diff(i102) > 1]);
if numel(blk) >= 2, idx = blk(2) : i102(end); else, idx = i102(1) : i102(end); end
idx = idx(ph(idx) == 102);
idx = idx(round(warmup_sec/dt) : end);
res = lg.Data.agent.estimator.result;
rr  = lg.Data.agent.reference.result;
n = min([numel(ph), size(res, 2), size(rr, 2)]);
idx = idx(idx <= n);
pe = zeros(3, numel(idx)); pr = zeros(3, numel(idx));
for j = 1:numel(idx)
    pe(:, j) = res{1, idx(j)}.state.p(:);
    pr(:, j) = rr{1, idx(j)}.state.p(:);
end
end
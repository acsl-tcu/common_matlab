%% compare_mec_plot : 円・レムニスケート軌道の A/B 比較図 + 高度曲線図を一键生成
%  黒=Nominal(無payload実飛行), 赤=Nominal with error, 緑=Nominal + EDMD-MEC
%  参照軌道は薄い灰色点線 (legend外)。RMSEは全て参照に対して算出。
%  タイトル: "circle: Nominal / Error / EDMD-MEC  RMSE 0.150 / 0.302 -> 0.299 m (1.0%)"
%  回復率 = (err-MEC)/(err-nom): 配重による劣化を何割取り戻したか (コンソール表示)
clearvars -except agent; clc; close all;

%% ===== ここだけ手動編集 =====
REPO = 'C:\Users\student\Documents\GitHub\common_matlab';
CIRCLE_NOM = 'lpvmpc_circle_90s_Log(16-Jul-2026_16_10_14)';                                % 円: 無payload (黒)
CIRCLE_ERR = 'payload_lpvmpc_circle_89s_Log(16-Jul-2026_15_35_34)';   % 円: 誤差あり・補償なし
CIRCLE_MEC = 'lpvmpccirclecomp_Log(30-Jul-2026_15_35_31)';            % 円: 誤差あり・EDMD-MEC
LEM_NOM    = 'lpvmpc_figure_87s_Log(16-Jul-2026_16_13_32)';                            % レムニスケート: 無payload (黒)
LEM_ERR    = 'payload_lpvmpc_figure_85s_Log(16-Jul-2026_16_00_07)';   % レムニスケート: mode 0
LEM_MEC    = 'lpvmpcfigure8comp_Log(30-Jul-2026_15_30_25)';            % レムニスケート: mode 5
WARMUP_SEC = 5.0;
DT = 0.025;
%% ===========================

if isempty(which('LOGGER'))
    assert(isfolder(REPO), 'REPOパスを修正せよ');
    addpath(regexprep(genpath(REPO), '[^;]*\.git[^;]*;', ''));
end
assert(~isempty(which('LOGGER')), 'LOGGERが見つからない');

C_NOM = [0 0 0];             % 黒 = 無payload実飛行
C_ERR = [0.86 0.22 0.15];    % 赤
C_MEC = [0.00 0.62 0.36];    % 緑
C_REF = [0.55 0.55 0.55];    % 灰 = 参照軌道 (点線, legend外)

make_panel('circle', CIRCLE_NOM, CIRCLE_ERR, CIRCLE_MEC, WARMUP_SEC, DT, C_NOM, C_ERR, C_MEC, C_REF);
make_panel('lemniscate', LEM_NOM, LEM_ERR, LEM_MEC, WARMUP_SEC, DT, C_NOM, C_ERR, C_MEC, C_REF);
make_altitude('circle', CIRCLE_NOM, CIRCLE_ERR, CIRCLE_MEC, WARMUP_SEC, DT, C_NOM, C_ERR, C_MEC, C_REF);
make_altitude('lemniscate', LEM_NOM, LEM_ERR, LEM_MEC, WARMUP_SEC, DT, C_NOM, C_ERR, C_MEC, C_REF);

%% ======================= 局所関数 =======================
function make_panel(name, f_nom, f_err, f_mec, wu, dt, C_NOM, C_ERR, C_MEC, C_REF)
[pe_n, pr_n] = load_xy(f_nom, wu, dt);
[pe_e, pr_e] = load_xy(f_err, wu, dt);
[pe_m, pr_m] = load_xy(f_mec, wu, dt);

% --- 時長裁剪: 三者の最短に揃える ---
N = min([size(pe_n,2), size(pe_e,2), size(pe_m,2)]);
pe_n = pe_n(:,1:N); pr_n = pr_n(:,1:N);
pe_e = pe_e(:,1:N); pr_e = pr_e(:,1:N);
pe_m = pe_m(:,1:N); pr_m = pr_m(:,1:N);

% --- RMSE (xy平面, 各自の参照に対して) ---
rmse_n = sqrt(mean(sum((pe_n(1:2,:) - pr_n(1:2,:)).^2, 1)));
rmse_e = sqrt(mean(sum((pe_e(1:2,:) - pr_e(1:2,:)).^2, 1)));
rmse_m = sqrt(mean(sum((pe_m(1:2,:) - pr_m(1:2,:)).^2, 1)));
imp = (rmse_e - rmse_m) / rmse_e * 100;
if rmse_e - rmse_n > 1e-6
    recov = (rmse_e - rmse_m) / (rmse_e - rmse_n) * 100;
else
    recov = NaN;
end

figure('Color', 'w', 'Position', [80 80 700 620]);
plot(pr_m(1,:), pr_m(2,:), ':', 'Color', C_REF, 'LineWidth', 1.0, 'HandleVisibility', 'off'); hold on;
plot(pe_n(1,:), pe_n(2,:), '-', 'Color', C_NOM, 'LineWidth', 1.6);
plot(pe_e(1,:), pe_e(2,:), '-', 'Color', C_ERR, 'LineWidth', 1.2);
plot(pe_m(1,:), pe_m(2,:), '-', 'Color', C_MEC, 'LineWidth', 1.2);
plot(pe_n(1,1), pe_n(2,1), 'o', 'MarkerFaceColor', C_NOM, 'MarkerEdgeColor', C_NOM, 'MarkerSize', 6, 'HandleVisibility', 'off');
plot(pe_e(1,1), pe_e(2,1), 'o', 'MarkerFaceColor', C_ERR, 'MarkerEdgeColor', C_ERR, 'MarkerSize', 6, 'HandleVisibility', 'off');
plot(pe_m(1,1), pe_m(2,1), 'o', 'MarkerFaceColor', C_MEC, 'MarkerEdgeColor', C_MEC, 'MarkerSize', 6, 'HandleVisibility', 'off');
axis equal; grid on;
xlabel('x [m]'); ylabel('y [m]');
legend({'Nominal', 'Nominal with error', 'Nominal + EDMD-MEC'}, 'Location', 'southwest');
title(sprintf('%s: Nominal / Error / EDMD-MEC  RMSE %.3f / %.3f -> %.3f m (%.1f%%)', ...
    name, rmse_n, rmse_e, rmse_m, imp), 'FontWeight', 'bold');
fprintf('[%s] RMSE: nom=%.3f, error=%.3f, MEC=%.3f m | 改善 %.1f%% | 回復率 %.1f%% (評価%.1fs)\n', ...
    name, rmse_n, rmse_e, rmse_m, imp, recov, N*dt);
end

function make_altitude(name, f_nom, f_err, f_mec, wu, dt, C_NOM, C_ERR, C_MEC, C_REF)
[pe_n, pr_n] = load_xy(f_nom, wu, dt);
[pe_e, ~   ] = load_xy(f_err, wu, dt);
[pe_m, ~   ] = load_xy(f_mec, wu, dt);
N = min([size(pe_n,2), size(pe_e,2), size(pe_m,2)]);
t = (0:N-1) * dt;
z_ref = pr_n(3, 1:N);
z_n = pe_n(3,1:N); z_e = pe_e(3,1:N); z_m = pe_m(3,1:N);
rmse_n = sqrt(mean((z_n - z_ref).^2));
rmse_e = sqrt(mean((z_e - z_ref).^2));
rmse_m = sqrt(mean((z_m - z_ref).^2));
imp = (rmse_e - rmse_m) / rmse_e * 100;
if rmse_e - rmse_n > 1e-6
    recov = (rmse_e - rmse_m) / (rmse_e - rmse_n) * 100;
else
    recov = NaN;
end

figure('Color', 'w', 'Position', [120 120 760 360]);
plot(t, z_ref, ':', 'Color', C_REF, 'LineWidth', 1.0, 'HandleVisibility', 'off'); hold on;
plot(t, z_n, '-', 'Color', C_NOM, 'LineWidth', 1.6);
plot(t, z_e, '-', 'Color', C_ERR, 'LineWidth', 1.2);
plot(t, z_m, '-', 'Color', C_MEC, 'LineWidth', 1.2);
grid on; xlim([0, t(end)]);
xlabel('Time [s]'); ylabel('z [m]');
legend({'Nominal', 'Nominal with error', 'Nominal + EDMD-MEC'}, 'Location', 'southeast');
title(sprintf('%s altitude: Nominal / Error / EDMD-MEC  RMSE %.3f / %.3f -> %.3f m (%.1f%%)', ...
    name, rmse_n, rmse_e, rmse_m, imp), 'FontWeight', 'bold');
fprintf('[%s/z] RMSE: nom=%.3f, error=%.3f, MEC=%.3f m | 改善 %.1f%% | 回復率 %.1f%%\n', ...
    name, rmse_n, rmse_e, rmse_m, imp, recov);
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
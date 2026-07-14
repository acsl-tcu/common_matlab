%% edmd_pipeline : 残差モデル 学習→体検→複放関門 一括実行 (単一ファイル版)
%  使い方: フレームワークのパスを通し, データフォルダ(または親)をカレントにして
%          >> edmd_pipeline
%  方針: 全関門は assert/error で即停止 (曖昧なwarningは出さない)。
%        追加の分析はこのファイル末尾に関数を足せばよい。
clearvars -except agent; clc; close all;

%% ===== 設定 =====
MASS   = 0.75;                                  % DRONE_PARAM("DIATONE") 名義質量 [kg]
G      = 9.81;
DT     = 0.025;
OUTMAT = "edmd_residual_model_LPVMPC_v4.mat";
LAMBDA_NOM = 1e-3;                              % 正規化ridge係数 (名義)
LAMBDA_ERR = 1e-2;                              % 正規化ridge係数 (残差) 複放FAIL時は10倍
NAMES = { ...
    'lpvmpc_figure70s_Log(14-Jul-2026_15_50_31)', ...
    'lpvmpc_circle_75s_Log(14-Jul-2026_15_47_58)', ...
    'lpvmpc_hover_60s_Log(14-Jul-2026_15_27_15)', ...
    'lpvmpc_p2p_100s_Log(14-Jul-2026_15_45_05)', ...
    'payload_lpvmpc_circle_75s_Log(14-Jul-2026_16_43_53)', ...
    'payload_lpvmpc_figure_80s_Log(14-Jul-2026_16_47_43)', ...
    'payload_lpvmpc_hover_70s_Log(14-Jul-2026_16_31_21)', ...
    'payload_lpvmpc_p2p_92s_Log(14-Jul-2026_16_34_28)'};

%% ===== 0. 前提チェック =====
assert(exist('LOGGER', 'class') == 8, 'LOGGERがパスにない → 先にいつものパス設定を実行');
files = cell(1, numel(NAMES));
for i = 1:numel(NAMES)
    hit = dir(fullfile(pwd, '**', [NAMES{i}, '.mat']));
    assert(~isempty(hit), 'ファイル未発見: %s.mat → データフォルダをカレントにせよ', NAMES{i});
    files{i} = fullfile(hit(1).folder, hit(1).name);
end
is_pay = contains(NAMES, 'payload');
u_hover = [MASS * G; 0; 0; 0];

%% ===== 1. 抽出 =====
fprintf('========== 抽出 ==========\n');
[Zn, Znp, Un, th_n] = collect_group(files(~is_pay), u_hover, DT);
[Zp, Zpp, Up, th_p] = collect_group(files(is_pay),  u_hover, DT);
fprintf('[data] nominal %d対 / payload %d対, 推力中央値 %.3f / %.3f (m*g=%.3f)\n', ...
    size(Zn,2), size(Zp,2), th_n, th_p, MASS*G);
assert(size(Zn,2) > 1000 && size(Zp,2) > 1000, '有効対が不足 (>1000必要) → データ/ガード閾値を確認');
assert(th_p > th_n + 0.1, 'payload推力がnominalより明確に大きくない → ファイル取り違えの疑い');

%% ===== 2. 学習 =====
fprintf('========== 学習 ==========\n');
[AB, vaf_nom] = ridge_norm(Zn, Un, Znp, LAMBDA_NOM);
A_nom = AB(:, 1:26); B_nom = AB(:, 27:30);
fprintf('[nom] VAF median=%.1f%%, max|eig|=%.4f\n', median(vaf_nom), max(abs(eig(A_nom))));
assert(median(vaf_nom) > 80, '名義VAFが低すぎる → 抽出かフィールド対応の異常');

R = Zpp - (A_nom * Zp + B_nom * Up);
r_dc = mean(R, 2);
fprintf('[res] 残差DC |mean|=%.4g (v系=%.4g)\n', norm(r_dc), norm(r_dc(7:9)));
assert(norm(r_dc) > 5e-4, '残差DCがほぼ零 → 配重誤差が見えていない (駐留データ不足)');

[AB, vaf_err] = ridge_norm(Zp, Up, R, LAMBDA_ERR);
A_err = AB(:, 1:26); B_err = AB(:, 27:30);
fprintf('[err] 残差VAF median=%.1f%%\n', median(vaf_err));

%% ===== 3. 係数体検 (即死関門) =====
mA = max(abs(A_err(:))); mB = max(abs(B_err(:)));
fprintf('[体検] max|A_err|=%.3g, max|B_err|=%.3g, rank(B_err)=%d\n', mA, mB, rank(B_err));
assert(mA < 0.5 && mB < 0.5, '病態回帰: 係数が過大 → LAMBDA_ERRを10倍にして再実行');
assert(rank(B_err) == 4, 'rank(B_err)<4 → 入力励起不足');

%% ===== 4. 複放関門 (payload実ログ上でδuを再現) =====
fprintf('========== 複放関門 ==========\n');
M = struct('A_nom', A_nom, 'B_nom', B_nom, 'A_err', A_err, 'B_err', B_err);
replay_files = files(is_pay);
figure('Name', 'replay check');
tiledlayout(numel(replay_files), 1, 'TileSpacing', 'compact');
for f = 1:numel(replay_files)
    [X, U, pairs] = extract_one(replay_files{f}, DT);
    du = zeros(4, numel(pairs));
    for j = 1:numel(pairs)
        k = pairs(j);
        z   = lift_z(X(:, k));
        rhs = lift_z(X(:, k+1)) - (M.A_nom*z + M.B_nom*(U(:,k)-u_hover)) - M.A_err*z;
        du(:, j) = (M.B_err'*M.B_err + 1e-2*eye(4)) \ (M.B_err' * rhs);
    end
    p95 = prctile(abs(du(2:4, :)), 95, 2);
    [~, base] = fileparts(replay_files{f});
    nexttile; plot(du(2:4, :)'); yline([0.05, -0.05], 'r--');
    title(base, 'Interpreter', 'none'); ylabel('\deltau'); legend('roll', 'pitch', 'yaw');
    fprintf('[replay] %s: p95|δu|=[%.3f %.3f %.3f]\n', base, p95);
    assert(all(isfinite(du(:))) && all(p95 < 0.05), ...
        '複放FAIL (%s) → 部署禁止。LAMBDA_ERRを10倍にして再実行', base);
end

%% ===== 5. 保存 (全関門通過後のみ到達) =====
results_edmd = M; %#ok<NASGU>
meta = struct('created', datestr(now), 'files', {files}, 'm', MASS, ...
    'u_convention', 'u_tilde = u - [m*g;0;0;0]', 'lambda', [LAMBDA_NOM, LAMBDA_ERR], ...
    'vaf_nom', vaf_nom, 'vaf_err', vaf_err, 'residual_dc', r_dc); %#ok<NASGU>
save(OUTMAT, 'results_edmd', 'meta');
fprintf(['========== 全関門PASS → %s 保存 ==========\n' ...
    '次: 1) matをmodel_fileフォルダへコピー+git commit\n' ...
    '    2) mode=5, du_max=[0;0.1;0.1;0], full_beta=0.1\n' ...
    '    3) agent再構築 → 自検 r=agent.controller.kqlmpc.residual (loaded=1)\n' ...
    '    4) A/B/A: mode 0→5→0, T=12円, 配重47.8g原位置\n'], OUTMAT);

%% ======================= 以下 局所関数 =======================
function [Z, Zp, Ut, thrust_med] = collect_group(files, u_hover, dt)
Z = []; Zp = []; Ut = []; th = [];
for i = 1:numel(files)
    [X, U, pairs] = extract_one(files{i}, dt);
    for k = pairs
        Z  = [Z,  lift_z(X(:, k))];    %#ok<AGROW>
        Zp = [Zp, lift_z(X(:, k+1))];  %#ok<AGROW>
        Ut = [Ut, U(:, k) - u_hover];  %#ok<AGROW>
        th(end+1) = U(1, k);           %#ok<AGROW>
    end
end
thrust_med = median(th);
end

function [X, U, pairs] = extract_one(matfile, dt)
% ログ→(X,U,有効対)。ガード: 102第2ブロック/yaw漂移0.25/駐留頭2sトリム/飽和除外
lg = LOGGER(char(matfile));
ph = lg.Data.phase(:)';
i102 = find(ph == 102);
assert(~isempty(i102), '%s: phase==102なし', matfile);
blk = i102([true, diff(i102) > 1]);
if numel(blk) >= 2, rng_idx = blk(2) : i102(end); else, rng_idx = i102(1) : i102(end); end
idx = rng_idx(ph(rng_idx) == 102);

U = lg.Data.agent.input;
if iscell(U)
    U = cellfun(@(c) u4(c), U(:)', 'UniformOutput', false); U = [U{:}];
end
if size(U, 1) ~= 4
    assert(size(U, 2) == 4, '%s: agent.input形状異常 [%s]', matfile, num2str(size(lg.Data.agent.input)));
    U = U';
end

res = lg.Data.agent.estimator.result;
n = min([numel(ph), size(U, 2), size(res, 2)]);
idx = idx(idx <= n - 1);
X = zeros(12, n);
need = false(1, n); need(idx) = true; need(min(idx+1, n)) = true;
for k = find(need)
    st = res{1, k}.state;
    X(:, k) = [st.p(:); st.q(:); st.v(:); st.w(:)];
end

yawf = X(6, idx);
bad = find(abs(yawf - median(yawf(1:min(40, end)))) > 0.25, 1);
if ~isempty(bad)
    fprintf('  [yaw-guard] %s: %d/%d 以降切捨て\n', matfile, bad, numel(idx));
    idx = idx(1 : bad-1);
    assert(numel(idx) >= 2, '%s: yaw-guard後に有効データなし', matfile);
end

n_trim = round(2.0 / dt);
vmag = vecnorm(X(7:9, :));
keep = true(1, n);
dwell = vmag < 0.05;
for s = find(dwell & ~[false, dwell(1:end-1)])
    pre = vmag(max(1, s-40) : max(1, s-1));
    if isempty(pre) || max(pre) < 0.15, continue; end
    keep(s : min(s + n_trim - 1, n)) = false;
end
keep(idx(1) : min(idx(1) + n_trim - 1, n)) = false;

ok = keep & all(abs(U(2:4, 1:n)) < 1.45, 1) & U(1, 1:n) > 0.2;
pairs = idx(ok(idx) & ok(idx + 1));
fprintf('  %s: 102ブロック%d個, 採用%d対\n', matfile, numel(blk), numel(pairs));
end

function [AB, vaf] = ridge_norm(Z, U, Y, lambda)
G = [Z; U];
s = std(G, 0, 2); s(s < 1e-6) = 1;
Gn = G ./ s;
AB = (Y * Gn') / (Gn * Gn' + lambda * size(Gn, 2) * eye(size(Gn, 1)));
AB = AB ./ s';
E = Y - AB * G;
vaf = max(0, (1 - var(E, 0, 2) ./ max(var(Y, 0, 2), 1e-12)) * 100);
end

function z = lift_z(x)
% コントローラ klift_edmd_residual と逐項一致 (26次元)
Q1=x(4); Q2=x(5); Q3=x(6); W1=x(10); W2=x(11); W3=x(12);
c1=cos(Q1); s1=sin(Q1); c2=cos(Q2); s2=sin(Q2); c3=cos(Q3); s3=sin(Q3);
c1s = sign(c1)*max(abs(c1),1e-3); if c1==0, c1s=1e-3; end
c2s = sign(c2)*max(abs(c2),1e-3); if c2==0, c2s=1e-3; end
R13 = c3*s2*c1 + s3*s1; R23 = s3*s2*c1 - c3*s1; R33 = c2*c1;
z = [x(1:12); R13; R23; R33; 1; ...
     W1*W2; W2*W3; W3*W1; W2*c1; W3*s1; ...
     W1*c2/c1s; W2*s1/c2s; W3*c1/c2s; W2*s1*s2/c2s; W3*c1*s2/c2s];
end

function u = u4(c)
if isempty(c), u = nan(4, 1);
else
    u = c(:); if numel(u) ~= 4, u = nan(4, 1); end
end
end
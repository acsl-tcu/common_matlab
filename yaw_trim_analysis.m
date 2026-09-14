%% yaw_trim_analysis : yaw角(sensor/estimator)と実出力からyaw配平値を算出
%  使い方: 下のFILESを自分のファイル名に書き換え, フレームワークのパスを通し,
%          データフォルダ(または親)をカレントにして一键実行
%  出力: ①ファイル別 yaw漂移・出力統計・図  ②推奨 Controller.yaw_trim 値
clearvars -except agent; clc; close all;

%% ===== ここだけ手動編集 =====
FILES = { ...
    'lpvmpc_hover_60s_Log(14-Jul-2026_15_27_15)', ...
    'lpvmpc_p2p_100s_Log(14-Jul-2026_15_45_05)', ...
    'lpvmpc_circle_75s_Log(14-Jul-2026_15_47_58)', ...
    'lpvmpc_figure70s_Log(14-Jul-2026_15_50_31)', ...
    'payload_lpvmpc_hover_70s_Log(14-Jul-2026_16_31_21)', ...
    'payload_lpvmpc_p2p_92s_Log(14-Jul-2026_16_34_28)', ...
    'payload_lpvmpc_circle_75s_Log(14-Jul-2026_16_43_53)', ...
    'payload_lpvmpc_figure_80s_Log(14-Jul-2026_16_47_43)'};
DT = 0.025;
%% ===========================
%% ===== パス自動設定 (LOGGERが未登録の場合のみ) =====
REPO = 'C:\Users\student\Documents\GitHub\common_matlab';
if isempty(which('LOGGER'))
    assert(isfolder(REPO), 'REPOパスを自分の環境に修正せよ: %s', REPO);
    p = genpath(REPO);
    p = regexprep(p, '[^;]*\.git[^;]*;', '');   % .git配下を除外 (boilerplateと同じ処理)
    addpath(p);
end
assert(~isempty(which('LOGGER')), 'REPO配下にLOGGERが見つからない → REPOパスを確認');
n_f = numel(FILES);
S = struct('name', {}, 'kind', {}, 'grp', {}, 'drift', {}, 'rate', {}, ...
           'tau_mean', {}, 'tau_q', {}, 'yaw_disagree', {}, 'stable', {});

figure('Name', 'yaw analysis', 'Position', [50 50 1300 800]);
tiledlayout(n_f, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

for i = 1:n_f
    hit = dir(fullfile(pwd, '**', [FILES{i}, '.mat']));
    assert(~isempty(hit), 'ファイル未発見: %s.mat', FILES{i});
    lg = LOGGER(fullfile(hit(1).folder, hit(1).name));

    % --- 飛行区間 (102の第2ブロック〜最後) ---
    ph = lg.Data.phase(:)';
    i102 = find(ph == 102);
    assert(~isempty(i102), '%s: phase==102なし', FILES{i});
    blk = i102([true, diff(i102) > 1]);
    if numel(blk) >= 2, idx = blk(2) : i102(end); else, idx = i102(1) : i102(end); end
    idx = idx(ph(idx) == 102);
    idx = idx(round(2/DT) : end);                 % 切替過渡2秒を捨てる
    t = (0 : numel(idx)-1) * DT;

    % --- estimator yaw ---
    res = lg.Data.agent.estimator.result;
    yaw_e = zeros(1, numel(idx));
    for j = 1:numel(idx), yaw_e(j) = res{1, idx(j)}.state.q(3); end

    % --- sensor yaw (候補パスを順に試す, 無ければスキップ) ---
    yaw_s = get_sensor_yaw(lg, idx);

    % --- 実出力 u4 ---
    U = lg.Data.agent.input;
    if iscell(U)
        U = cellfun(@(c) u4fix(c), U(:)', 'UniformOutput', false); U = [U{:}];
    end
    if size(U, 1) ~= 4, U = U'; end
    tau = U(4, idx);

    % --- 統計 ---
    drift = yaw_e(end) - yaw_e(1);
    p = polyfit(t, yaw_e, 1);                     % 漂移率 [rad/s]
    q4 = quarter_means(tau);                      % 出力の1/4区間平均 (電圧勾配の可視化)
    stable = abs(drift) < 0.1;                    % yawが保てていたか
    dis = NaN;
    if ~isempty(yaw_s), dis = max(abs(yaw_s - yaw_e)); end

    S(i) = struct('name', FILES{i}, 'kind', classify(FILES{i}), ...
        'grp', ternary(contains(FILES{i}, 'payload'), "payload", "nominal"), ...
        'drift', drift, 'rate', p(1), 'tau_mean', mean(tau), 'tau_q', q4, ...
        'yaw_disagree', dis, 'stable', stable);

    % --- 図 ---
    nexttile; plot(t, yaw_e, 'b'); hold on;
    if ~isempty(yaw_s), plot(t, yaw_s, 'r:'); legend('est', 'sensor'); end
    yline(0, 'k--'); title(sprintf('%s | yaw', FILES{i}), 'Interpreter', 'none');
    ylabel('\psi [rad]'); grid on;
    nexttile; plot(t, tau, 'Color', [.6 .6 .6]); hold on;
    plot(linspace(t(1), t(end), 4), q4, 'ro-', 'LineWidth', 2);
    yline(mean(tau), 'b--'); title('applied \tau_{yaw} (赤:1/4区間平均)');
    ylabel('[N·m]'); grid on;
end

%% ===== 集計と配平推奨値 =====
fprintf('\n============ ファイル別サマリ ============\n');
fprintf('%-52s %-8s 漂移[rad] 率[mrad/s] τ平均    τ勾配(末-初)\n', 'file', 'kind');
for i = 1:n_f
    fprintf('%-52s %-8s %+8.3f  %+8.2f  %+8.4f  %+8.4f %s\n', S(i).name, S(i).kind, ...
        S(i).drift, S(i).rate*1000, S(i).tau_mean, S(i).tau_q(end)-S(i).tau_q(1), ...
        ternary(S(i).stable, '', '★漂移'));
end
if any(~isnan([S.yaw_disagree]))
    fprintf('sensor-estimator yaw 最大乖離: %.4f rad (小さければ推定は健全)\n', max([S.yaw_disagree]));
end

fprintf('\n============ 配平値の算出 ============\n');
fprintf('原理: yawが保てていたファイルでは, 施加τの平均 = 外乱を打ち消していた値そのもの。\n');
fprintf('      漂移したファイルのτ平均は「足りなかった値」なので下限としてのみ参考。\n');
for g = ["nominal", "payload"]
    m_st = [S(strcmp([S.grp], g) & [S.stable]).tau_mean];
    m_all = [S(strcmp([S.grp], g)).tau_mean];
    if isempty(m_st)
        fprintf('[%s] 安定ファイルなし → 全体平均 %.4f は下限値 (真値はこれ以上)\n', g, mean(m_all));
        rec.(g) = mean(m_all);
    else
        fprintf('[%s] 安定ファイルのτ平均 = %.4f (n=%d) / 全体 %.4f\n', g, mean(m_st), numel(m_st), mean(m_all));
        rec.(g) = mean(m_st);
    end
end
fprintf('\n>>> 推奨設定 (飛行構成に合わせて選択):\n');
fprintf('    無配重:  Controller.yaw_trim = %+.4f;\n', rec.nominal);
fprintf('    配重時:  Controller.yaw_trim = %+.4f;\n', rec.payload);
fprintf('    (差分 %.4f が配重の寄与。τ勾配列が大きい場合は電圧跌落分 → 積分器の守備範囲)\n', ...
    rec.payload - rec.nominal);

%% ======================= 局所関数 =======================
function yaw_s = get_sensor_yaw(lg, idx)
% sensor(Motive)側のyawを候補パスから探す。見つからなければ空を返す(致命ではない)
yaw_s = [];
cands = {@(k) lg.Data.agent.sensor.result{1, k}.state.q(3), ...
         @(k) lg.Data.agent.sensor.result{1, k}.q(3), ...
         @(k) lg.Data.agent.sensor.result{1, k}.rigid.q(3)};
for c = 1:numel(cands)
    try
        tmp = zeros(1, numel(idx));
        for j = 1:numel(idx), tmp(j) = cands{c}(idx(j)); end
        yaw_s = tmp;
        return;
    catch
    end
end
fprintf('  [注意] sensor側yawのフィールドが特定できず → estimatorのみで分析 (配平算出には影響なし)\n');
fprintf('         必要なら get_sensor_yaw の候補パスに正しいパスを追加\n');
end

function q = quarter_means(x)
n = numel(x); e = round(linspace(0, n, 5));
q = arrayfun(@(k) mean(x(e(k)+1 : e(k+1))), 1:4);
end

function k = classify(name)
if contains(name, 'hover'), k = "hover";
elseif contains(name, 'p2p'), k = "p2p";
elseif contains(name, 'circle'), k = "circle";
elseif contains(name, 'figure'), k = "fig8";
else, k = "other"; end
end

function u = u4fix(c)
if isempty(c), u = nan(4, 1);
else, u = c(:); if numel(u) ~= 4, u = nan(4, 1); end
end
end

function s = ternary(c, a, b)
if c, s = a; else, s = b; end
end
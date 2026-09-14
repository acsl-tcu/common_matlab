%% compare_three_HL.m
% 3条件の比較プロット:
%   (1) 標称HL         : 質量・慣性が正常、reference に追従
%   (2) 劣化HL         : 質量・慣性変化 + ノイズ、追従が悪化
%   (3) 補償HL         : EDMD残差補償で追従を回復
%
% 使い方:
%   各条件のログ .mat を1つずつ指定して実行。
%   plant の真値軌跡と reference を取り出し、位置追従と誤差を比較する。

clear; clc; close all;

%% ========== ログファイル指定（3つ）==========
% バッチ保存名は save_prefix + run番号。例: HL_nominal_1.mat
% 各条件で代表的な run を1つずつ指定（同じ run番号にすると軌道が揃って比較しやすい）。
FILE_NOMINAL     = 'C:\Users\student\Documents\GitHub\common_matlab\Data\Sim_data\HL_nominal_1_Log(12-Jun-2026_14_33_40).mat';
FILE_DEGRADED    = 'C:\Users\student\Documents\GitHub\common_matlab\Data\Sim_data\HL_degraded_1_Log(12-Jun-2026_11_24_07).mat';
FILE_COMPENSATED = 'C:\Users\student\Documents\GitHub\common_matlab\Data\Sim_data\HL_compensated_1_Log(12-Jun-2026_12_53_33).mat';
% ============================================

dt = 0.025;  % ログの実dtに合わせる

% 3条件をまとめて処理
files  = {FILE_NOMINAL, FILE_DEGRADED, FILE_COMPENSATED};
labels = {'(1) Nominal HL', '(2) Degraded (mass+inertia+noise)', '(3) Compensated'};
colors = {[0.00 0.45 0.74], [0.85 0.33 0.10], [0.47 0.67 0.19]};  % 青/橙/緑

D = cell(1,3);  % 各条件のデータ格納

fprintf('===== ログ読み込み診断 =====\n');
for c = 1:3
    fprintf('\n[%s]\n  file: %s\n', labels{c}, files{c});
    if exist(files{c}, 'file') ~= 2
        fprintf('  → ファイルが存在しません。パス/ファイル名を確認してください。\n');
        D{c} = [];
        continue;
    end
    [D{c}, diag_msg] = load_hl_trajectory(files{c}, dt);
    if isempty(D{c})
        fprintf('  → 読み込み失敗: %s\n', diag_msg);
    else
        fprintf('  → OK: 有効点数 %d, 時間 %.1f s\n', size(D{c}.p,2), D{c}.t(end));
    end
end
fprintf('\n');

%% ===== 図1: XY平面追従 =====
figure('Name', 'XY tracking', 'Position', [80 80 600 560]);
hold on; grid on; axis equal;

% reference（劣化条件のものを代表として描画）
ref_drawn = false;
for c = [1 2 3]
    if ~isempty(D{c}) && ~ref_drawn
        plot(D{c}.ref_p(1,:), D{c}.ref_p(2,:), 'k--', 'LineWidth', 2.0, ...
            'DisplayName', 'Reference');
        ref_drawn = true;
    end
end
% 各条件の実軌跡
for c = 1:3
    if ~isempty(D{c})
        plot(D{c}.p(1,:), D{c}.p(2,:), '-', 'Color', colors{c}, ...
            'LineWidth', 2.0, 'DisplayName', labels{c});
    end
end
xlabel('x [m]'); ylabel('y [m]');
title('XY Trajectory Tracking');
legend('Location', 'best'); set(gca, 'FontSize', 12);

%% ===== 図2: 各軸 位置 vs 時間 =====
figure('Name', 'Position vs time', 'Position', [700 80 700 700]);
axis_names = {'x', 'y', 'z'};
for ax = 1:3
    subplot(3,1,ax); hold on; grid on;
    ref_drawn = false;
    for c = 1:3
        if isempty(D{c}); continue; end
        if ~ref_drawn
            plot(D{c}.t, D{c}.ref_p(ax,:), 'k--', 'LineWidth', 1.8, ...
                'DisplayName', 'Reference');
            ref_drawn = true;
        end
        plot(D{c}.t, D{c}.p(ax,:), '-', 'Color', colors{c}, ...
            'LineWidth', 1.6, 'DisplayName', labels{c});
    end
    ylabel(sprintf('%s [m]', axis_names{ax}));
    if ax == 1; title('Position Tracking per Axis'); legend('Location','best'); end
    if ax == 3; xlabel('time [s]'); end
    set(gca, 'FontSize', 11);
end

%% ===== 図3: 追従誤差ノルム vs 時間（核心の比較）=====
figure('Name', 'Tracking error', 'Position', [200 200 760 420]);
hold on; grid on;
for c = 1:3
    if isempty(D{c}); continue; end
    err = vecnorm(D{c}.p - D{c}.ref_p, 2, 1);
    plot(D{c}.t, err, '-', 'Color', colors{c}, 'LineWidth', 2.0, ...
        'DisplayName', labels{c});
end
xlabel('time [s]'); ylabel('||position error|| [m]');
title('Position Error Norm — lower is better');
legend('Location', 'best'); set(gca, 'FontSize', 12);

%% ===== 図4: z高度の拡大（質量変化の影響が出やすい）=====
figure('Name', 'Z height detail', 'Position', [260 260 760 420]);
hold on; grid on;
ref_drawn = false;
for c = 1:3
    if isempty(D{c}); continue; end
    if ~ref_drawn
        plot(D{c}.t, D{c}.ref_p(3,:), 'k--', 'LineWidth', 1.8, 'DisplayName', 'Reference');
        ref_drawn = true;
    end
    plot(D{c}.t, D{c}.p(3,:), '-', 'Color', colors{c}, 'LineWidth', 2.0, ...
        'DisplayName', labels{c});
end
xlabel('time [s]'); ylabel('z [m]');
title('Altitude Tracking (mass change shows here)');
legend('Location', 'best'); set(gca, 'FontSize', 12);

%% ===== 数値サマリ =====
fprintf('\n===== 追従性能サマリ（定常区間 後半50%%）=====\n');
fprintf('%-38s | RMSE_pos | RMSE_z  | std_z(平滑度)\n', '条件');
fprintf('%s\n', repmat('-', 1, 78));
for c = 1:3
    if isempty(D{c}); continue; end
    N = size(D{c}.p, 2);
    idx = round(N/2):N;   % 後半（定常）で評価
    err_vec = vecnorm(D{c}.p(:,idx) - D{c}.ref_p(:,idx), 2, 1);
    rmse_pos = sqrt(mean(err_vec.^2));
    err_z = D{c}.p(3,idx) - D{c}.ref_p(3,idx);
    rmse_z = sqrt(mean(err_z.^2));
    % 平滑度：z誤差の1階差分のstd（小さいほど滑らか＝抖動少ない）
    smooth_z = std(diff(D{c}.p(3,idx)));
    fprintf('%-38s | %7.4f  | %6.4f | %.5f\n', labels{c}, rmse_pos, rmse_z, smooth_z);
end
fprintf('%s\n', repmat('-', 1, 78));
fprintf('期待: (1)<(3)<(2) で RMSE が小さく、(3)のstd_zが(2)より小=滑らか\n');

%% =========================================================
%% ローカル関数: ログから plant真値軌跡と reference を取り出す
%% =========================================================
function [D, diag_msg] = load_hl_trajectory(fpath, dt)
    D = [];
    diag_msg = '';

    try
        raw = load(fpath);
    catch ME
        diag_msg = sprintf('load失敗: %s', ME.message);
        return;
    end

    if isfield(raw, 'log');      L = raw.log;
    elseif isfield(raw, 'Data'); L = raw;
    else; fn = fieldnames(raw);  L = raw.(fn{1});
    end

    try
        pr  = L.Data.agent.plant.result;       % plant真値
        rf  = L.Data.agent.reference.result;   % reference
        K   = min(numel(pr), numel(rf));
    catch
        diag_msg = 'plant.result / reference.result が見つからない';
        return;
    end
    if K < 2
        diag_msg = sprintf('フレーム数が少なすぎる (K=%d)', K);
        return;
    end

    p     = nan(3, K);
    v     = nan(3, K);
    ref_p = nan(3, K);
    n_p_ok = 0; n_ref_ok = 0;
    for k = 1:K
        try
            st = pr{1,k}.state;
            p(:,k) = double(st.p(:));
            v(:,k) = double(st.v(:));
            n_p_ok = n_p_ok + 1;
        catch; end
        try
            sr = rf{1,k}.state;
            if isprop(sr,'p') || (isstruct(sr)&&isfield(sr,'p'))
                ref_p(:,k) = double(sr.p(:));
                n_ref_ok = n_ref_ok + 1;
            elseif isprop(sr,'xd') || (isstruct(sr)&&isfield(sr,'xd'))
                xd = double(sr.xd(:));
                ref_p(:,k) = xd(1:3);
                n_ref_ok = n_ref_ok + 1;
            end
        catch; end
    end

    if n_p_ok < 2
        diag_msg = sprintf('plant状態の抽出に失敗 (有効%d/%d)', n_p_ok, K);
        return;
    end
    if n_ref_ok < 2
        diag_msg = sprintf('reference状態の抽出に失敗 (有効%d/%d)。state.p も state.xd も無い可能性', n_ref_ok, K);
        return;
    end

    % 離陸前の全ゼロフレームを除去
    valid = vecnorm(p,2,1) > 1e-6 & ~any(isnan(p),1) & ~any(isnan(ref_p),1);
    if nnz(valid) < 2
        diag_msg = sprintf('有効フレームが少なすぎる (valid=%d)', nnz(valid));
        return;
    end

    D = struct();
    D.p     = p(:, valid);
    D.v     = v(:, valid);
    D.ref_p = ref_p(:, valid);
    D.t     = (0:size(D.p,2)-1) * dt;
    diag_msg = 'OK';
end
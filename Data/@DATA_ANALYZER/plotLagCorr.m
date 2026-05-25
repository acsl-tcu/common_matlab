% ── ラグ相関プロット（入力→状態の全組み合わせ）─────────
function plotLagCorr(obj, inputIdx, stateIdx, maxStep)
    % plotLagCorr  step=1〜maxStep の各ラグにおける相関係数をプロット
    %
    % 横軸: ラグ (1〜maxStep ステップ)
    % 縦軸: 相関係数
    % サブプロット配置: 行=状態変数, 列=入力変数
    %
    % 引数
    %   inputIdx : 入力変数のインデックス (VarNames の列番号)
    %              省略時は全変数の後半(input)を自動推定
    %   stateIdx : 状態変数のインデックス (VarNames の列番号)
    %              省略時は全変数の前半(state)を自動推定
    %   maxStep  : 最大ラグ数 (省略時は obj.step、それも0なら10)
    %
    % 使用例
    %   obj.plotLagCorr();                    % 全自動
    %   obj.plotLagCorr(13:16, 1:12, 20);     % input=13〜16, state=1〜12, maxStep=20
    arguments
        obj
        inputIdx = []
        stateIdx = []
        maxStep  = []
    end

    % --- maxStep の決定 ---
    if isempty(maxStep)
        if obj.step > 0
            maxStep = obj.step;
        else
            maxStep = 10;
        end
    end

    % --- システム変数 インデックスの取り出し ---
    for i = 1:obj.M
        if strcmp(obj.SystemVals{i}, 'state')
            stateIdx = [stateIdx, i];
        elseif strcmp(obj.SystemVals{i}, 'input')
            inputIdx = [inputIdx, i];
        end
    end

    nIn  = numel(inputIdx);
    nSt  = numel(stateIdx);

    if nIn == 0 || nSt == 0
        warning('plotLagCorr: inputIdx または stateIdx が空です。');
        return;
    end

    % --- 各ラグの相関係数を計算 ---
    % lagCorr(s, i, j): sステップラグ, 状態i, 入力j の相関
    lagCorr = zeros(maxStep, nSt, nIn);

    orig_step = obj.step;   % 元の step を退避
    for s = 1:maxStep
        obj.step = s;
        % キャッシュをクリアして再計算
        cache_sz       = obj.numCacheSlots();
        obj.CorrMatrix = cell(cache_sz, 1);
        obj.CovMatrix  = cell(cache_sz, 1);

        for k = 1:obj.loopLen()
            ci = obj.cacheIdx(k);
            obj.calcCorrInternal(ci);
            R = obj.CorrMatrix{ci};   % M×M
            % stateIdx(行) × inputIdx(列) の部分行列を取り出す
            % CorrMatrix(i,j) = corr(変数i[k], 変数j[k+step])
            % → state[k] と input[k+step] の相関 を取得
            lagCorr(s, :, :) = lagCorr(s, :, :) + reshape(R(stateIdx, inputIdx), [1, nSt, nIn]);
        end
        % loopLen>1(divide)の場合は平均化しない（各試行は独立なので後でループ外に出す設計）
    end
    obj.step       = orig_step;   % step を元に戻す
    obj.CorrMatrix = cell(obj.numCacheSlots(), 1);   % キャッシュリセット
    obj.CovMatrix  = cell(obj.numCacheSlots(), 1);

    % --- 凡例スタイル定義（画像凡例に準拠）---
    % 色グループ: blue=p/vx/Ωroll, orange=py/vy/Ωpitch, green=pz/vz/Ωyaw
    % 線種グループ: p=実線, v=破線, φθψ=一点鎖線, Ω=点線
    state_colors = [
        0.122, 0.471, 0.706;   % p_x  : blue
        1.000, 0.498, 0.055;   % p_y  : orange
        0.173, 0.627, 0.173;   % p_z  : green
        0.392, 0.710, 0.965;   % v_x  : light blue
        1.000, 0.733, 0.471;   % v_y  : light orange
        0.400, 0.800, 0.400;   % v_z  : light green
        0.839, 0.153, 0.157;   % phi  : red
        0.580, 0.404, 0.741;   % theta: purple
        0.549, 0.337, 0.294;   % psi  : brown
        0.839, 0.153, 0.157;   % Omega_roll  : red (dotted)
        0.580, 0.404, 0.741;   % Omega_pitch : purple (dotted)
        0.549, 0.337, 0.294;   % Omega_yaw   : brown (dotted)
    ];
    state_lines = {'-', '-', '-', ...   % p: 実線
                   '--','--','--', ...  % v: 破線
                   '-.','-.','-.', ...  % φθψ: 一点鎖線
                   ':',':',':'}; ...    % Ω: 点線

    state_markers = {'o', 'o', 'o',...  % p: 位置
                     'x', 'x', 'x',...  % v: 速度
                     '.', '.', '.',...  % q: 姿勢角
                     '^', '^', '^'};    % w: 各速度

    % stateIdx の数に合わせて色・線種をクリップ
    nColors = size(state_colors, 1);
    nLines  = numel(state_lines);
    nmarkers= numel(state_markers);
    get_color = @(si) state_colors(mod(si-1, nColors)+1, :);
    get_line  = @(si) state_lines{mod(si-1, nLines)+1};
    get_marker= @(si) state_markers{mod(si-1, nmarkers)+1};

    % --- figure レイアウト定数（normalized 単位）---
    % 上部に sgtitle・凡例・サブプロットタイトル用の余白を確保し、
    % 各領域が重ならないよう axes を手動配置する。
    fig_w     = min(400 * nIn + 80, 1600);
    fig_h     = 520;
    margin_l  = 0.07;    % 左余白 (normalized)
    margin_r  = 0.02;    % 右余白
    margin_b  = 0.11;    % 下余白（xlabel用）
    ax_top    = 0.75;    % ax上端: 凡例+sgtitle+subplotタイトル分を下げる
    ax_height = ax_top - margin_b;
    gap       = 0.025;   % ax間ギャップ
    ax_width  = (1 - margin_l - margin_r - gap * (nIn - 1)) / nIn;

    lbl       = obj.trialLabel(1);
    lag_steps = 1:maxStep;

    fig = figure('Name', sprintf('Lag Correlation: %s', lbl), ...
                 'Position', [80, 80, fig_w, fig_h]);

    ax_handles     = gobjects(nIn, 1);
    legend_handles = gobjects(nSt, 1);

    for ii = 1:nIn
        left = margin_l + (ii - 1) * (ax_width + gap);
        ax   = axes('Parent', fig, ...
                    'Position', [left, margin_b, ax_width, ax_height]); %#ok<LAXES>
        hold(ax, 'on');
        ax_handles(ii) = ax;

        h = gobjects(nSt, 1);
        for si = 1:nSt
            r_vals = lagCorr(:, si, ii);
            h(si)  = plot(ax, lag_steps, r_vals, ...
                [get_line(si), get_marker(si)], ...
                'Color',           get_color(si), ...
                'LineWidth',        1.5, ...
                'MarkerSize',       2, ...
                'MarkerFaceColor',  get_color(si), ...
                'DisplayName',      obj.VarNames{stateIdx(si)});
        end

        % 基準線
        yline(ax,  0,   'k--', 'LineWidth', 0.8, 'HandleVisibility', 'off');
        yline(ax,  0.5, ':',   'Color', [0.6 0.6 0.6], 'LineWidth', 0.8, 'HandleVisibility', 'off');
        yline(ax, -0.5, ':',   'Color', [0.6 0.6 0.6], 'LineWidth', 0.8, 'HandleVisibility', 'off');
        hold(ax, 'off');

        ylim(ax, [-1, 1]);
        xlim(ax, [1, maxStep]);
        xticks(ax, lag_steps);
        grid(ax, 'on'); box(ax, 'on');
        ax.FontSize = obj.FS * 0.9;

        xlabel(ax, 'lag step', 'FontSize', obj.FS);
        if ii == 1
            ylabel(ax, 'correlation', 'FontSize', obj.FS);
        end
        
        title(ax, obj.VarNames{inputIdx(ii)}, ...
            'Interpreter', 'latex', 'FontSize', obj.FS);

        if ii == nIn
            legend_handles = h;
        end
    end

    % --- sgtitle（最上部）---
    if obj.fTitle
        sup = sprintf('Lag Correlation (step 1-%d): %s', maxStep, lbl);
        sgtitle(sup, 'FontSize', obj.FS, 'FontWeight', 'bold', 'Interpreter', 'none');
    end

    % --- 凡例を sgtitle 直下・ax 直上に 2行×6列で固定配置 ---
    % invisible axes をキャンバスとして使い Position で位置を固定する。
    % subplot の 'northoutside' は sgtitle と重なるため使用しない。
    drawnow;
    ax_dummy = axes('Parent', fig, ...
        'Position', [0, 0, 1, 1], ...
        'Visible',  'off', ...
        'HitTest',  'off');
    lg = legend(ax_dummy, legend_handles, ...
        'Interpreter', 'latex', ...
        'FontSize',    obj.FS, ...
        'Orientation', 'horizontal', ...
        'NumColumns',  6, ...
        'Location',    'north', ...
        'Box',         'on');
    lg.Title.String   = 'state';
    lg.Title.FontSize = obj.FS;

    % 凡例を ax_top のすぐ上に固定（水平中央揃え）
    drawnow;
    lg_w    = lg.Position(3);
    lg_h_v  = lg.Position(4);
    lg_left = max(0.01, 0.5 - lg_w / 2);
    lg_bot  = ax_top + 0.05;    % ax 上端から少し上
    lg.Position = [lg_left, lg_bot, lg_w, lg_h_v];
end

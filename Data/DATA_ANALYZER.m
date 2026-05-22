classdef DATA_ANALYZER < handle
% DATA_ANALYZER  LOGGERベースの時系列データ統計分析クラス
%
% 【概要】
%   matファイルの選択（GUI）・LOGGERによるデータ読み込み・
%   分散/相関係数の計算・可視化をまとめて担うクラス。
%   matファイルごとに独立した分析を行う（試行間比較用途）
%
% 【使い方】
%   % スクリプトフォルダを渡してインスタンス生成
%   %   → GUI が開き、matファイルを複数選択できる
%   %   → 選択ごとに LOGGER を生成してデータを取り込む
%   obj = DATA_ANALYZER();                        % デフォルト (divide モード)
%   obj = DATA_ANALYZER('mode', 'divide');        % 試行ごとに独立分析
%   obj = DATA_ANALYZER('mode', 'all');           % 全試行データを連結して分析
%   obj = DATA_ANALYZER('mode', 'all', 'FS', 12, 'fTitle', false);
%
%   % 全試行を一括分析・可視化
%   obj.runAll();
%
%   % 個別実行も可能
%   obj.computeVariance();
%   obj.computeCorrelation();
%   obj.testSignificance(0.05);
%   obj.plotHeatmap();
%   obj.plotScatterMatrixALL();
%   obj.plotVarianceBar();
%
% 【分析対象変数の設定】
%   コンストラクタ内の QUERY_DEFS を編集してください。
%   各行が { agentId, variable, attribute, 表示名 } の形式です。
%   例)
%     { 1, 'p', 'e', 'p_e'   }  % agent1 の推定位置
%     { 1, 'v', 'e', 'v_e'   }  % agent1 の推定速度
%
% 【stepオプション（ラグ相関）について】
%   step=s (s>0) を指定すると、kステップ目と(k+s)ステップ目の間の
%   クロス相関係数行列を計算します（ラグ相関）。
%   行列の (i,j) 成分は「変数i の kステップ」と「変数j の k+sステップ」の相関です。
%   step=0（既定）は通常の同時刻相関です。
%   例) obj = DATA_ANALYZER('step', 2);  % 2ステップ先との相関
%
% 【複数次元データについて】
%   logger.data() が N×D 行列を返す場合、各列を独立変数として扱います。
%   表示名は '表示名_1', '表示名_2', ... と自動的に連番が付きます。

    %% ----------------------------------------------------------------
    %  プロパティ
    %% ----------------------------------------------------------------
    properties (Access = public)
        % ---- データ本体 ----
        Loggers     % Loggers{i}     : i番目の試行の LOGGER インスタンス
        LogData     % LogData{i}     : i番目の試行の N×M データ行列（常に試行数分保持）
        FileNames   % FileNames{i}   : i番目の試行のファイル名（表示用）
        VarNames        % 変数名セル配列 {1×M}（全試行共通）
        NumTrials       % 選択したファイル数（mode に関わらず実際の試行数）
        M               % 変数数

        % ---- 分析結果キャッシュ（試行ごとに cell 配列で保持）----
        Means       % 平均値
        Variances   % 分散
        StdDevs     % 標準偏差(Standard deviation)
        CVs         % 変動係数(Coefficient of Variation): 平均値に対するデータのばらつきの相対的な大きさ[%]
        CovMatrix
        CorrMatrix
        PValues
    end

    properties (Access = public)
        % ---- 動作関連 ----
        mode        % 'all'    : 全試行データを縦結合して一括分析
                    % 'divide' : 試行ごとに独立分析（既定）
        step        % 何ステップ先のデータとの相関を見るか

        % ---- 描画設定関連 ----
        FS
        fTitle
    end

    %% ----------------------------------------------------------------
    %  コンストラクタ
    %% ----------------------------------------------------------------
    methods
        function obj = DATA_ANALYZER(args)
            % DATA_ANALYZER  コンストラクタ
            arguments
                args.FS {mustBeNumeric, mustBeNonnegative} = 10
                args.fTitle = true
                args.mode string {mustBeMember(args.mode, {'divide', 'all'})} = 'all'
                args.step {mustBeNumeric, mustBeNonnegative} = 0
                args.loggers (:,1) cell {DATA_ANALYZER.mustBeAllLoggers}
                args.FileNames
            end
            if isfield(args, "loggers") && ~isfield(args, "FileNames")
                error("DATA_ANALYZER: loggersを指定する場合はFileNamesも指定してください。")
            end
            obj.FS      = args.FS;
            obj.fTitle  = args.fTitle;
            obj.mode    = args.mode;
            obj.step    = args.step;

            % ============================================================
            %  ★ 分析対象変数の定義（ここを編集してください）★
            %  書式: { agentId, variable, attribute, '表示名' }
            %  attribute: 'e'=推定値, 'r'=参照値, 's'=センサ値, 'p'=プラント値
            % ============================================================
            QUERY_DEFS = {
                1, 'p1', 'e', '$x$'
                1, 'p2', 'e', '$y$'
                1, 'p3', 'e', '$z$'
                1, 'v1', 'e', '$v_x$'
                1, 'v2', 'e', '$v_y$'
                1, 'v3', 'e', '$v_z$'
                1, 'q1', 'e', '$\phi$'
                1, 'q2', 'e', '$\theta$'
                1, 'q3', 'e', '$\psi$'
                1, 'w1', 'e', '$\Omega_1$'
                1, 'w2', 'e', '$\Omega_2$'
                1, 'w3', 'e', '$\Omega_3$'
                1, 'input1', '', '$T$'
                1, 'input2', '', '$\tau_1$'
                1, 'input3', '', '$\tau_2$'
                1, 'input4', '', '$\tau_3$'
            };
            % ============================================================

            % ---- Step 1: GUI でmatファイルを複数選択 --------------------
            if isfield(args, 'loggers')
                loggers = args.loggers;
                obj.FileNames = args.FileNames;
                fprintf('引数で指定した %d件のLOGGERを読み込みました。\n',numel(args.loggers))
            else
                loggers = obj.selectAndLoadFiles('./Data');
                if isempty(loggers)
                    error('DATA_ANALYZER: 有効なLOGGERが生成できませんでした。');
                end
            end

            % ---- Step 2: QUERY_DEFS に従いデータを取り込む --------------
            [obj.LogData, obj.VarNames] = obj.extractData(loggers, QUERY_DEFS);

            obj.Loggers   = loggers;
            obj.NumTrials = numel(loggers);
            obj.M         = numel(obj.VarNames);

            if strcmp(obj.mode, 'all')
                total_n = sum(cellfun(@(d) size(d,1), obj.LogData));
                fprintf('[DATA_ANALYZER] allモード: %d 試行 (計 %d サンプル) を対象\n', ...
                    obj.NumTrials, total_n);
            end

            % キャッシュを試行数分確保（divide: 試行ごと / all: インデックス1のみ使用）
            cache_size     = obj.numCacheSlots();
            obj.Means      = cell(cache_size, 1);
            obj.Variances  = cell(cache_size, 1);
            obj.StdDevs    = cell(cache_size, 1);
            obj.CVs        = cell(cache_size, 1);
            obj.CovMatrix  = cell(cache_size, 1);
            obj.CorrMatrix = cell(cache_size, 1);
            obj.PValues    = cell(cache_size, 1);

            fprintf('[DATA_ANALYZER] 初期化完了: mode=%s / step=%d / %d 試行 × %d 変数\n', ...
                obj.mode, obj.step, obj.NumTrials, obj.M);
            fprintf('  変数: %s\n', strjoin(obj.VarNames, ', '));
        end
    end

    %% ----------------------------------------------------------------
    %  公開メソッド
    %% ----------------------------------------------------------------
    methods (Access = public)

        % ── 1. 分散・標準偏差の計算 ──────────────────────────────────
        function results = computeVariance(obj)
            % computeVariance  各試行・各変数の基本統計量を計算して表示
            %
            % 戻り値 results : 1×NumTrials struct 配列
            %   .trial      試行名
            %   .means      1×M 平均
            %   .variances  1×M 不偏分散
            %   .stddevs    1×M 標準偏差
            %   .cvs        1×M 変動係数[%]

            results = struct('trial', {}, 'means', {}, 'variances', {}, ...
                             'stddevs', {}, 'cvs', {});

            for k = 1:obj.loopLen()
                ci = obj.cacheIdx(k);   % キャッシュスロットのインデックス
                obj.calcVarianceInternal(ci);

                fprintf('\n== 基本統計量: %s ==\n', obj.trialLabel(k));
                hdr = sprintf('%-10s %10s %9s %12s %8s %10s\n%-10s %14s %10s %12s %10s %12s', ...
                    '変数', '平均', '中央値', '分散(不偏)', '標準偏差', '変動係数', ...
                    'Variable', 'Average', 'Median', 'Variance', 'Standard deviation', 'Coefficient of Variation');
                fprintf('%s\n%s\n', hdr, repmat('-', 1, 70));

                % 表示用データ: allモードは全試行結合, divideモードは試行k
                D = obj.getMergedData(k);
                for i = 1:obj.M
                    fprintf('%-14s %10.4f %10.4f %12.4f %12.4f %14.2f%%\n', ...
                        obj.VarNames{i}, ...
                        obj.Means{ci}(i), median(D(:,i)), ...
                        obj.Variances{ci}(i), obj.StdDevs{ci}(i), obj.CVs{ci}(i));
                end

                results(k).trial     = obj.trialLabel(k);
                results(k).means     = obj.Means{ci};
                results(k).variances = obj.Variances{ci};
                results(k).stddevs   = obj.StdDevs{ci};
                results(k).cvs       = obj.CVs{ci};
            end
        end

        % ── 2. 相関係数行列の計算 ─────────────────────────────────────
        function results = computeCorrelation(obj)
            % computeCorrelation  各試行の共分散行列・相関係数行列を計算して表示
            %
            % 戻り値 results : 1×NumTrials struct 配列
            %   .trial       試行名
            %   .cov_matrix  M×M 共分散行列
            %   .corr_matrix M×M 相関係数行列

            results = struct('trial', {}, 'cov_matrix', {}, 'corr_matrix', {});

            for k = 1:obj.loopLen()
                ci = obj.cacheIdx(k);
                obj.calcCorrInternal(ci);

                if obj.step == 0
                    fprintf('\n== 相関係数行列 (Pearson): %s ==\n', obj.trialLabel(k));
                else
                    fprintf('\n== ラグ相関係数行列 (Pearson, step=%d): %s ==\n', obj.step, obj.trialLabel(k));
                    fprintf('   行 = kステップの変数 / 列 = k+%dステップの変数\n', obj.step);
                end
                obj.printMatrix(obj.CorrMatrix{ci}, '%.4f');

                results(k).trial       = obj.trialLabel(k);
                results(k).cov_matrix  = obj.CovMatrix{ci};
                results(k).corr_matrix = obj.CorrMatrix{ci};
            end
        end

        % ── 3. 有意性検定 ────────────────────────────────────────────
        function results = testSignificance(obj, alpha)
            % testSignificance  各試行の相関係数に対する両側t検定
            %
            % 引数
            %   alpha : 有意水準（既定値 0.05=5%）
            % 戻り値 results : 1×NumTrials struct 配列
            %   .trial    試行名
            %   .r_matrix M×M 相関係数行列
            %   .p_matrix M×M p値行列

            if nargin < 2, alpha = 0.05; end

            results = struct('trial', {}, 'r_matrix', {}, 'p_matrix', {});

            for k = 1:obj.loopLen()
                ci = obj.cacheIdx(k);
                obj.calcCorrInternal(ci);

                % p値行列の計算
                if obj.step == 0
                    % 通常: corrcoef が直接返す p値を使用
                    [~, P] = corrcoef(obj.getMergedData(k));
                else
                    % ラグ相関: r値からt統計量経由でp値を算出
                    % n_eff = 試行ごとの有効サンプル数の合計
                    n_eff  = obj.getEffectiveN(k);
                    R      = obj.CorrMatrix{ci};
                    t_stat = R .* sqrt((n_eff - 2) ./ max(1 - R.^2, eps));
                    P      = 2 * (1 - tcdf(abs(t_stat), n_eff - 2));
                    P(1:obj.M+1:end) = 0;   % 対角成分を0に（自己相関は検定不要）
                end
                obj.PValues{ci} = P;

                if obj.step == 0
                    fprintf('\n== 有意性検定 (α=%.3f): %s ==\n', alpha, obj.trialLabel(k));
                else
                    fprintf('\n== ラグ有意性検定 (α=%.3f, step=%d): %s ==\n', alpha, obj.step, obj.trialLabel(k));
                end
                fprintf('%-14s  %-12s  %4s  %8s  %10s\n', ...
                    '変数1', '変数2', 'r値', 'p値', '有意');
                fprintf('%s\n', repmat('-', 1, 62));

                for i = 1:obj.M - 1
                    for j = i + 1:obj.M
                        fprintf('%-14s  %-14s  %8.4f  %10.4e  %s\n', ...
                            obj.VarNames{i}, obj.VarNames{j}, ...
                            obj.CorrMatrix{ci}(i,j), P(i,j), ...
                            obj.sigLabel(P(i,j)));
                    end
                end
                fprintf('凡例:\n***: p<0.001    **: p<0.01    *: p<0.05    n.s.: 有意差なし\n');

                results(k).trial    = obj.trialLabel(k);
                results(k).r_matrix = obj.CorrMatrix{ci};
                results(k).p_matrix = P;
            end
        end

        % ── 4. 相関ヒートマップ ──────────────────────────────────────
        function plotHeatmap(obj, showPValue)
            % plotHeatmap  各試行の相関係数行列をヒートマップで可視化
            %
            % 引数
            %   showPValue : true のとき有意なセルに * を付加（既定 true）

            if nargin < 2, showPValue = true; end

            for k = 1:obj.loopLen()
                ci = obj.cacheIdx(k);
                obj.calcCorrInternal(ci);
                if showPValue && isempty(obj.PValues{ci})
                    if obj.step == 0
                        [~, obj.PValues{ci}] = corrcoef(obj.getMergedData(k));
                    else
                        n_eff  = obj.getEffectiveN(k);
                        R      = obj.CorrMatrix{ci};
                        t_stat = R .* sqrt((n_eff - 2) ./ max(1 - R.^2, eps));
                        P      = 2 * (1 - tcdf(abs(t_stat), n_eff - 2));
                        P(1:obj.M+1:end) = 0;
                        obj.PValues{ci} = P;
                    end
                end

                % タイトル文字列
                lbl = obj.trialLabel(k);
                if obj.step == 0
                    hmap_title = sprintf('相関係数行列ヒートマップ\n%s', lbl);
                else
                    hmap_title = sprintf('ラグ相関係数行列ヒートマップ (step=%d)\n%s', obj.step, lbl);
                end

                figure('Name', sprintf('相関ヒートマップ: %s', lbl), ...
                       'Position', obj.figPos(1, k));

                imagesc(obj.CorrMatrix{ci});
                colormap(obj.redblueMap());
                cb = colorbar;
                cb.Label.String = 'ピアソン相関係数(Pearson r)';
                clim([-1, 1]);

                ax = gca;
                ax.TickLabelInterpreter = 'latex';
                ax.XTick = 1:obj.M;  ax.XTickLabel = obj.VarNames;
                ax.XTickLabelRotation = 45;
                ax.YTick = 1:obj.M;  ax.YTickLabel = obj.VarNames;
                ax.TickLength = [0, 0];
                ax.FontSize = obj.FS*1.1;
                if obj.step > 0
                    xlabel('base  [k]', 'FontSize', obj.FS);
                    ylabel(sprintf('lag  [k + %d]', obj.step), 'FontSize', obj.FS);
                end
                

                for i = 1:obj.M
                    for j = 1:obj.M
                        r_v   = obj.CorrMatrix{ci}(i, j);
                        r_str = sprintf('%.3f', r_v);
                        tc    = obj.cellTextColor(r_v);
                        % r値を1行目に表示
                        text(j, i - 0.15, r_str, ...
                            'HorizontalAlignment', 'center', ...
                            'VerticalAlignment',   'middle', ...
                            'FontSize', obj.FS, 'FontWeight', 'bold', ...
                            'Color', tc);
                        % 有意マークを2行目（少し下）に表示
                        if showPValue && i ~= j
                            sig_str = obj.sigMark(obj.PValues{ci}(i,j));
                            if ~isempty(sig_str)
                                text(j, i + 0.28, sig_str, ...
                                    'HorizontalAlignment', 'center', ...
                                    'VerticalAlignment',   'middle', ...
                                    'FontSize', obj.FS * 0.85, 'FontWeight', 'bold', ...
                                    'Color', tc);
                            end
                        end
                    end
                end

                if obj.fTitle
                    title(hmap_title, 'FontSize', obj.FS*1.2, 'FontWeight', 'bold');
                    if showPValue
                        subtitle('* p<0.05   ** p<0.01   *** p<0.001', 'FontSize', obj.FS*0.9);
                    end
                end
            end
        end

        % ── 5. 散布図行列 ────────────────────────────────────────────
        function plotScatterMatrixALL(obj)
            % plotScatterMatrixALL  各試行の散布図行列（対角: ヒストグラム）
            % [TODO]
            % 全ての散布図を同window内にプロットするのは見にくい。
            % windowを分けてプロットする"plotScatterMatrixDEVIDE"を作成
            % その際、最小サイズで良い。
            % figure生成時に画面上でsize(obj.M)個分を考慮して関係が取れるように配置する。

            for k = 1:obj.loopLen()
                ci      = obj.cacheIdx(k);
                step    = obj.step;
                colors = lines(obj.M);
                obj.calcCorrInternal(ci);

                % step対応: 試行境界をまたがないよう試行ごとにペアを作り結合
                [D_base, D_lag] = obj.getBaseLagData(k);

                lbl = obj.trialLabel(k);
                figure('Name', sprintf('散布図行列: %s', lbl), ...
                       'Position', obj.figPos(2, k));

                for i = 1:obj.M
                    for j = 1:obj.M
                        subplot(obj.M, obj.M, (i-1)*obj.M + j);

                        if i == j
                            histogram(D_base(:, i), 20, ...
                                'FaceColor', colors(i,:), ...
                                'EdgeColor', 'white', 'FaceAlpha', 0.8);
                            if step == 0
                                title(obj.VarNames{i}, 'FontSize', obj.FS*0.8, 'FontWeight', 'bold', 'Interpreter', 'latex');
                            else
                                title(sprintf('%s\\,[k]', obj.VarNames{i}), 'FontSize', obj.FS*0.8, 'FontWeight', 'bold', 'Interpreter', 'latex');
                            end
                        else
                            xd = D_base(:, j);
                            yd = D_lag(:, i);
                            scatter(xd, yd, 15, [0.25, 0.5, 0.75], ...
                                'filled', 'MarkerFaceAlpha', 0.4);
                            hold on;
                            p  = polyfit(xd, yd, 1);
                            xr = linspace(min(xd), max(xd), 60);
                            plot(xr, polyval(p, xr), 'r-', 'LineWidth', 1.5);

                            r_v = obj.CorrMatrix{ci}(i, j);
                            tc  = [0.3, 0.3, 0.3];
                            if abs(r_v) >= 0.5, tc = [0.75, 0.1, 0.1]; end
                            text(0.97, 0.96, sprintf('r=%.2f', r_v), ...
                                'Units', 'normalized', ...
                                'HorizontalAlignment', 'right', ...
                                'VerticalAlignment', 'top', ...
                                'FontSize', obj.FS*0.8, 'Color', tc);
                            hold off;
                        end

                        grid on; box on; set(gca, 'FontSize', obj.FS*0.7);
                        if step == 0
                            if j == 1,     ylabel(obj.VarNames{i}, 'FontSize', obj.FS*0.7, 'Interpreter', 'latex'); end
                            if i == obj.M, xlabel(obj.VarNames{j}, 'FontSize', obj.FS*0.7, 'Interpreter', 'latex'); end
                        else
                            if j == 1,     ylabel(sprintf('%s\\,[k+%d]', obj.VarNames{i}, step), 'FontSize', obj.FS*0.7, 'Interpreter', 'latex'); end
                            if i == obj.M, xlabel(sprintf('%s\\,[k]',    obj.VarNames{j}),    'FontSize', obj.FS*0.7, 'Interpreter', 'latex'); end
                        end
                    end
                end

                if obj.fTitle
                    if step == 0
                        sgtitle(sprintf('散布図行列: %s', lbl), 'FontSize', obj.FS, 'FontWeight', 'bold');
                    else
                        sgtitle(sprintf('ラグ散布図行列 (step=%d): %s', step, lbl), 'FontSize', obj.FS, 'FontWeight', 'bold');
                    end
                end
            end
        end

        % ── 6. 分散の棒グラフ ────────────────────────────────────────
        function plotVarianceBar(obj)
            % plotVarianceBar  各試行の分散・標準偏差を棒グラフで比較

            for k = 1:obj.loopLen()
                ci  = obj.cacheIdx(k);
                lbl = obj.trialLabel(k);
                obj.calcVarianceInternal(ci);

                figure('Name', sprintf('分散・標準偏差: %s', lbl), ...
                       'Position', obj.figPos(3, k));

                subplot(1, 2, 1);
                ax = gca;
                bar(obj.Variances{ci}, 'FaceColor', [0.2, 0.5, 0.8], 'FaceAlpha', 0.8);
                set(ax, 'XTick', 1:obj.M, 'XTickLabel', obj.VarNames, ...
                    'XTickLabelRotation', 30, 'FontSize', obj.FS);
                ax.TickLabelInterpreter = "latex";
                ylabel('分散 (Variance)');
                title('各変数の分散', 'FontWeight', 'bold');
                grid on; box off;

                subplot(1, 2, 2);
                ax = gca;
                bar(obj.StdDevs{ci}, 'FaceColor', [0.2, 0.7, 0.5], 'FaceAlpha', 0.8);
                set(ax, 'XTick', 1:obj.M, 'XTickLabel', obj.VarNames, ...
                    'XTickLabelRotation', 30, 'FontSize', obj.FS);
                ax.TickLabelInterpreter = "latex";
                ylabel('標準偏差 (Standard deviation)');
                title('各変数の標準偏差', 'FontWeight', 'bold');
                grid on; box off;

                if obj.fTitle
                    sgtitle(sprintf('分散・標準偏差の比較: %s', lbl), ...
                        'FontSize', obj.FS*1.1, 'FontWeight', 'bold');
                end
            end
        end

        % ── 7. 一括実行 ──────────────────────────────────────────────
        function runAll(obj, alpha)
            % runAll  全メソッドを試行ごとに順に実行
            %
            % 引数
            %   alpha : 有意水準（既定値 0.05）

            if nargin < 2, alpha = 0.05; end
            obj.computeVariance();
            obj.computeCorrelation();
            obj.testSignificance(alpha);
            obj.plotVarianceBar();
            obj.plotHeatmap(true);
            obj.plotScatterMatrixALL();
            fprintf('\n=== 全試行の分析が完了しました ===\n');
        end

    end % public methods

    %% ----------------------------------------------------------------
    %  内部メソッド
    %% ----------------------------------------------------------------
    methods (Access = private)

        % ---- ファイル選択 & LOGGER 生成 --------------------------------
        function loggers = selectAndLoadFiles(obj, script_dir)
            % GUI でmatファイルを複数選択し、LOGGERインスタンスを生成して返す

            fprintf('matファイルを選択してください（複数選択可）...\n');

            [file_names, file_dir] = uigetfile( ...
                {'*.mat', 'MATファイル (*.mat)'}, ...
                'matファイルを選択（複数選択: Ctrl / Shift）', ...
                fullfile(script_dir, '*.mat'), ...
                'MultiSelect', 'on');

            if isequal(file_names, 0)
                fprintf('キャンセルされました。\n');
                loggers = {};
                return;
            end
            if ischar(file_names)
                file_names = {file_names};   % 単一選択を cell に統一
            end

            n_files = numel(file_names);
            fprintf('%d 件を選択しました。LOGGERを生成中...\n', n_files);

            loggers        = cell(n_files, 1);
            obj.FileNames  = cell(n_files, 1);

            for i = 1:n_files
                mat_path = fullfile(file_dir, file_names{i});
                fprintf('  [%d/%d] %s\n', i, n_files, file_names{i});
                try
                    loggers{i}       = LOGGER(mat_path);
                    obj.FileNames{i} = file_names{i};
                catch ME
                    warning('LOGGER生成失敗: %s\n  理由: %s', file_names{i}, ME.message);
                    loggers{i} = [];
                end
            end

            % 失敗分を除去
            valid          = ~cellfun(@isempty, loggers);
            loggers        = loggers(valid);
            obj.FileNames  = obj.FileNames(valid);

            fprintf('%d / %d 件のLOGGER生成に成功しました。\n\n', numel(loggers), n_files);
        end

        % ---- QUERY_DEFS に従い各 LOGGER からデータを取り出す ----------
        function [log_data, var_names] = extractData(obj, loggers, query_defs)
            % QUERY_DEFS の各行を logger.data() で取得し、
            % N×M のデータ行列と変数名セル配列を返す。
            % 多次元変数は列ごとに分割して独立変数として扱う。

            n_trials   = numel(loggers);
            log_data   = cell(n_trials, 1);
            var_names  = {};    % 初回試行で確定

            for k = 1:n_trials
                lg      = loggers{k};
                cols    = {};
                vnames  = {};

                for q = 1:size(query_defs, 1)
                    agent_id  = query_defs{q, 1};
                    variable  = query_defs{q, 2};
                    attribute = query_defs{q, 3};
                    disp_name = query_defs{q, 4};

                    try
                        raw = lg.data(agent_id, variable, attribute);   % N×D
                        raw = double(raw);

                        if size(raw, 2) == 1
                            % スカラー時系列: そのまま1列
                            cols{end+1}   = raw;       %#ok<AGROW>
                            vnames{end+1} = disp_name; %#ok<AGROW>
                        else
                            % ベクトル時系列: 列ごとに分割
                            for d = 1:size(raw, 2)
                                cols{end+1}   = raw(:, d);                    %#ok<AGROW>
                                vnames{end+1} = sprintf('%s%d', disp_name, d); %#ok<AGROW>
                            end
                        end

                    catch ME
                        warning('Trial[%d] "%s" の取得失敗: %s', k, disp_name, ME.message);
                    end
                end

                if isempty(cols)
                    error('DATA_ANALYZER: Trial[%d] でデータを1件も取得できませんでした。', k);
                end

                % 最短サンプル数に揃えて行列化
                min_n       = min(cellfun(@numel, cols));
                log_data{k} = cell2mat(cellfun(@(c) c(1:min_n), cols, 'UniformOutput', false));

                % 変数名は最初の試行で確定
                if k == 1
                    var_names = vnames;
                end

                fprintf('  Trial[%d]: %d サンプル × %d 変数\n', k, min_n, numel(vnames));
            end
        end

        % ---- 基本統計量キャッシュ計算 ----------------------------------
        function calcVarianceInternal(obj, ci)
            % ci : キャッシュスロットインデックス（cacheIdx(k) の結果）
            if isempty(obj.Means{ci})
                D = obj.getMergedData(ci);   % allモード: 全試行単純結合（分散は境界無関係）
                obj.Means{ci}     = mean(D);
                obj.Variances{ci} = var(D);
                obj.StdDevs{ci}   = std(D);
                obj.CVs{ci}       = obj.StdDevs{ci} ./ abs(obj.Means{ci}) * 100;
            end
        end

        % ---- 相関係数キャッシュ計算 ------------------------------------
        function calcCorrInternal(obj, ci)
            % ci : キャッシュスロットインデックス（cacheIdx(k) の結果）
            if isempty(obj.CorrMatrix{ci})
                step = obj.step;

                % 対象となる試行インデックスを取得
                if strcmp(obj.mode, 'all')
                    trial_ids = 1:obj.NumTrials;   % 全試行
                else
                    trial_ids = ci;                % divide: ci == 試行番号
                end

                if step == 0
                    % 通常の同時刻相関: 単純結合で corrcoef
                    Data = vertcat(obj.LogData{trial_ids});
                    obj.CovMatrix{ci}  = cov(Data);
                    obj.CorrMatrix{ci} = corrcoef(Data);
                else
                    % ラグ相関: 各試行ごとに (D_base, D_lag) ペアを作り結合
                    %   → 試行境界をまたぐペアは一切含まれない
                    bases = cell(numel(trial_ids), 1);
                    lags  = cell(numel(trial_ids), 1);
                    for idx = 1:numel(trial_ids)
                        Data = obj.LogData{trial_ids(idx)};
                        N = size(Data, 1);
                        if N <= step
                            warning('DATA_ANALYZER: Trial[%d] サンプル数(%d) <= step(%d) のためスキップ', ...
                                trial_ids(idx), N, step);
                            continue;
                        end
                        bases{idx} = Data(1:N-step, :);
                        lags{idx}  = Data(1+step:N, :);
                    end
                    % 空エントリを除去して結合
                    valid  = ~cellfun(@isempty, bases);
                    Data_base = vertcat(bases{valid});
                    Data_lag  = vertcat(lags{valid});

                    n_eff = size(Data_base, 1);
                    if n_eff <= 2
                        error('DATA_ANALYZER: 有効サンプル数が少なすぎます (n_eff=%d)', n_eff);
                    end

                    % クロス共分散行列: (i,j) = cov(変数i_base, 変数j_lag)
                    C = (Data_base - mean(Data_base))' * (Data_lag - mean(Data_lag)) / (n_eff - 1);
                    obj.CovMatrix{ci} = C;

                    % クロス相関係数行列
                    std_base = std(Data_base);   % 1×M
                    std_lag  = std(Data_lag);    % 1×M
                    obj.CorrMatrix{ci} = C ./ (std_base' * std_lag);
                end
            end
        end

        % ---- mode に応じた結合データを返す ----------------------------
        function D = getMergedData(obj, k)
            % k : loopLen のループ変数（allモード時は1固定）
            if strcmp(obj.mode, 'all')
                D = vertcat(obj.LogData{:});   % 分散用なので単純結合で可
            else
                D = obj.LogData{k};
            end
        end

        % ---- mode に応じた (D_base, D_lag) ペアを返す ----------------
        function [D_base, D_lag] = getBaseLagData(obj, k)
            % k : loopLen のループ変数
            % step=0 の場合は D_base == D_lag（同じデータ）
            s = obj.step;
            if strcmp(obj.mode, 'all')
                trial_ids = 1:obj.NumTrials;
            else
                trial_ids = k;
            end
            bases = cell(numel(trial_ids), 1);
            lags  = cell(numel(trial_ids), 1);
            for idx = 1:numel(trial_ids)
                D = obj.LogData{trial_ids(idx)};
                N = size(D, 1);
                if s == 0
                    bases{idx} = D;
                    lags{idx}  = D;
                elseif N > s
                    bases{idx} = D(1:N-s, :);
                    lags{idx}  = D(1+s:N, :);
                end
            end
            valid  = ~cellfun(@isempty, bases);
            D_base = vertcat(bases{valid});
            D_lag  = vertcat(lags{valid});
        end

        % ---- 有効サンプル数を返す（ラグ相関用）-----------------------
        function n = getEffectiveN(obj, k)
            % allモード: 各試行の (N_i - step) の合計
            % divideモード: (N_k - step)
            s = obj.step;
            if strcmp(obj.mode, 'all')
                n = sum(cellfun(@(d) max(0, size(d,1) - s), obj.LogData));
            else
                n = max(0, size(obj.LogData{k}, 1) - s);
            end
        end

        % ---- ループ長（allモード=1, divideモード=NumTrials）-----------
        function L = loopLen(obj)
            if strcmp(obj.mode, 'all')
                L = 1;
            else
                L = obj.NumTrials;
            end
        end

        % ---- キャッシュスロットのインデックス --------------------------
        function ci = cacheIdx(obj, k)
            % allモード: 常にスロット1を使用
            % divideモード: 試行番号 k をそのまま使用
            if strcmp(obj.mode, 'all')
                ci = 1;
            else
                ci = k;
            end
        end

        % ---- キャッシュ確保サイズ --------------------------------------
        function n = numCacheSlots(obj)
            % allモード: スロット1つだけ確保
            % divideモード: 試行数分確保
            if strcmp(obj.mode, 'all')
                n = 1;
            else
                n = obj.NumTrials;
            end
        end

        % ---- 表示用ラベル ---------------------------------------------
        function lbl = trialLabel(obj, k)
            if strcmp(obj.mode, 'all')
                lbl = sprintf('all trials combined (%d trials)', obj.NumTrials);
            else
                lbl = obj.FileNames{k};
            end
        end

        % ---- 行列の整形表示 --------------------------------------------
        function printMatrix(obj, M, fmt)
            colW = 12;
            fprintf('%-14s', '');
            for j = 1:obj.M
                fprintf('%*s', colW, obj.VarNames{j});
            end
            fprintf('\n%s\n', repmat('-', 1, 14 + colW * obj.M));
            for i = 1:obj.M
                fprintf('%-14s', obj.VarNames{i});
                for j = 1:obj.M
                    fprintf(['%', num2str(colW), 's'], sprintf(fmt, M(i,j)));
                end
                fprintf('\n');
            end
        end

        % ---- ウィンドウ位置（試行番号でオフセット）--------------------
        function pos = figPos(~, type_idx, trial_idx)
            base_x = 30 + (trial_idx - 1) * 40;
            base_y = -10 + (type_idx  - 1) * 40;
            pos = [base_x, base_y, 900, 650];
        end

    end % private methods

    %% ----------------------------------------------------------------
    %  静的ヘルパー
    %% ----------------------------------------------------------------
    methods (Static, Access = private)

        function label = sigLabel(p)
            if p < 0.001,    label = '*** (p<0.001)';
            elseif p < 0.01, label = '**  (p<0.01)';
            elseif p < 0.05, label = '*   (p<0.05)';
            else,             label = 'n.s.';
            end
        end

        function mark = sigMark(p)
            if p < 0.001,    mark = '***';
            elseif p < 0.01, mark = '**';
            elseif p < 0.05, mark = '*';
            else,             mark = '';
            end
        end

        function c = cellTextColor(r_v)
            if abs(r_v) >= 0.4, c = 'white';
            else,                c = 'black';
            end
        end

        function cmap = redblueMap()
            n    = 128;
            r    = [linspace(0.10, 1.0, n), ones(1, n)            ];
            g    = [linspace(0.30, 1.0, n), linspace(1.0, 0.2, n) ];
            b    = [ones(1, n),             linspace(1.0, 0.1, n)  ];
            cmap = [r; g; b]';
        end

        function mustBeAllLoggers(in)
            % インスタンス生成時にloggerを引数として入れたときの判別を行う関数
            if ~all(cellfun(@(c) isa(c, 'LOGGER'), in))
                error('すべてのセル要素は LOGGER クラスである必要があります。');
            end
        end

    end % static methods

end % classdef
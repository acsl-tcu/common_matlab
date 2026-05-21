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
%   obj.plotScatterMatrix();
%   obj.plotVarianceBar();
%
% 【分析対象変数の設定】
%   コンストラクタ内の QUERY_DEFS を編集してください。
%   各行が { agentId, variable, attribute, 表示名 } の形式です。
%   例)
%     { 1, 'p', 'e', 'p_e'   }  % agent1 の推定位置
%     { 1, 'v', 'e', 'v_e'   }  % agent1 の推定速度
%
% 【複数次元データについて】
%   logger.data() が N×D 行列を返す場合、各列を独立変数として扱います。
%   表示名は '表示名_1', '表示名_2', ... と自動的に連番が付きます。

    %% ----------------------------------------------------------------
    %  プロパティ
    %% ----------------------------------------------------------------
    properties (Access = private)
        % ---- データ本体 ----
        Loggers     % Loggers{i}     : i番目の試行の LOGGER インスタンス
        LogData     % LogData{i}     : i番目の試行の N×M データ行列
        FileNames   % FileNames{i}   : i番目の試行のファイル名（表示用）
        VarNames        % 変数名セル配列 {1×M}（全試行共通）
        NumTrials       % 試行数（divide: 選択ファイル数 / all: 常に1）
        M               % 変数数

        % ---- 分析結果キャッシュ（試行ごとに cell 配列で保持）----
        Means
        Variances
        StdDevs
        CVs
        CovMatrix
        CorrMatrix
        PValues
    end

    properties (Access = public)
        % ---- 動作関連 ----
        mode        % 'all'    : 全試行データを縦結合して一括分析
                    % 'divide' : 試行ごとに独立分析（既定）

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
                args.FS mustBeNonnegative = 10
                args.fTitle = true
                args.mode string {mustBeMember(args.mode, {'divide', 'all'})} = 'all'
            end
            obj.FS      = args.FS;
            obj.fTitle  = args.fTitle;
            obj.mode    = args.mode;

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
            loggers = obj.selectAndLoadFiles('./Data');
            if isempty(loggers)
                error('DATA_ANALYZER: 有効なLOGGERが生成できませんでした。');
            end

            % ---- Step 2: QUERY_DEFS に従いデータを取り込む --------------
            [obj.LogData, obj.VarNames] = obj.extractData(loggers, QUERY_DEFS);

            obj.Loggers   = loggers;
            obj.NumTrials = numel(loggers);
            obj.M         = numel(obj.VarNames);

            % 'all' モード: 全試行データを縦結合して LogData{1} に集約
            if strcmp(obj.mode, 'all')
                combined = vertcat(obj.LogData{:});
                obj.LogData   = {combined};
                obj.FileNames = {'all trials (combined)'};
                obj.NumTrials = 1;
                fprintf('[DATA_ANALYZER] allモード: %d 試行を連結 → %d サンプル × %d 変数\n', ...
                    numel(loggers), size(combined, 1), obj.M);
            end

            % キャッシュを（有効な）試行数分だけ確保
            obj.Means      = cell(obj.NumTrials, 1);
            obj.Variances  = cell(obj.NumTrials, 1);
            obj.StdDevs    = cell(obj.NumTrials, 1);
            obj.CVs        = cell(obj.NumTrials, 1);
            obj.CovMatrix  = cell(obj.NumTrials, 1);
            obj.CorrMatrix = cell(obj.NumTrials, 1);
            obj.PValues    = cell(obj.NumTrials, 1);

            fprintf('[DATA_ANALYZER] 初期化完了: mode=%s / %d 試行 × %d 変数\n', ...
                obj.mode, obj.NumTrials, obj.M);
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

            for k = 1:obj.NumTrials
                obj.calcVarianceInternal(k);

                fprintf('\n== 基本統計量: %s ==\n', obj.FileNames{k});
                hdr = sprintf('%-14s %10s %10s %12s %10s %10s', ...
                    '変数', '平均', '中央値', '分散(不偏)', '標準偏差', '変動係数');
                fprintf('%s\n%s\n', hdr, repmat('-', 1, 70));

                D = obj.LogData{k};
                for i = 1:obj.M
                    fprintf('%-14s %10.4f %10.4f %12.4f %10.4f %9.2f%%\n', ...
                        obj.VarNames{i}, ...
                        obj.Means{k}(i), median(D(:,i)), ...
                        obj.Variances{k}(i), obj.StdDevs{k}(i), obj.CVs{k}(i));
                end

                results(k).trial     = obj.FileNames{k};
                results(k).means     = obj.Means{k};
                results(k).variances = obj.Variances{k};
                results(k).stddevs   = obj.StdDevs{k};
                results(k).cvs       = obj.CVs{k};
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

            for k = 1:obj.NumTrials
                obj.calcCorrInternal(k);

                fprintf('\n== 相関係数行列 (Pearson): %s ==\n', obj.FileNames{k});
                obj.printMatrix(obj.CorrMatrix{k}, '%.4f');

                results(k).trial       = obj.FileNames{k};
                results(k).cov_matrix  = obj.CovMatrix{k};
                results(k).corr_matrix = obj.CorrMatrix{k};
            end
        end

        % ── 3. 有意性検定 ────────────────────────────────────────────
        function results = testSignificance(obj, alpha)
            % testSignificance  各試行の相関係数に対する両側t検定
            %
            % 引数
            %   alpha : 有意水準（既定値 0.05）
            % 戻り値 results : 1×NumTrials struct 配列
            %   .trial    試行名
            %   .r_matrix M×M 相関係数行列
            %   .p_matrix M×M p値行列

            if nargin < 2, alpha = 0.05; end

            results = struct('trial', {}, 'r_matrix', {}, 'p_matrix', {});

            for k = 1:obj.NumTrials
                obj.calcCorrInternal(k);
                [~, P] = corrcoef(obj.LogData{k});
                obj.PValues{k} = P;

                fprintf('\n== 有意性検定 (α=%.3f): %s ==\n', alpha, obj.FileNames{k});
                fprintf('%-14s  %-14s  %8s  %10s  %s\n', ...
                    '変数1', '変数2', 'r値', 'p値', '有意');
                fprintf('%s\n', repmat('-', 1, 62));

                for i = 1:obj.M - 1
                    for j = i + 1:obj.M
                        fprintf('%-14s  %-14s  %8.4f  %10.4e  %s\n', ...
                            obj.VarNames{i}, obj.VarNames{j}, ...
                            obj.CorrMatrix{k}(i,j), P(i,j), ...
                            obj.sigLabel(P(i,j)));
                    end
                end
                fprintf('凡例: *** p<0.001  ** p<0.01  * p<0.05  n.s. 有意差なし\n');

                results(k).trial    = obj.FileNames{k};
                results(k).r_matrix = obj.CorrMatrix{k};
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

            for k = 1:obj.NumTrials
                obj.calcCorrInternal(k);
                if showPValue && isempty(obj.PValues{k})
                    [~, obj.PValues{k}] = corrcoef(obj.LogData{k});
                end

                figure('Name', sprintf('相関ヒートマップ: %s', obj.FileNames{k}), ...
                       'Position', obj.figPos(1, k));

                imagesc(obj.CorrMatrix{k});
                colormap(obj.redblueMap());
                cb = colorbar;
                cb.Label.String = 'Pearson r';
                clim([-1, 1]);

                ax = gca;
                ax.TickLabelInterpreter = 'latex';
                ax.XTick = 1:obj.M;  ax.XTickLabel = obj.VarNames; % xlabelを指定した状態(VarNames)に変更
                ax.XTickLabelRotation = 45;
                ax.YTick = 1:obj.M;  ax.YTickLabel = obj.VarNames;
                ax.TickLength = [0, 0];
                ax.FontSize = obj.FS*1.1;

                for i = 1:obj.M
                    % ヒートマップの各マスに相関係数を描画
                    for j = 1:obj.M
                        r_v   = obj.CorrMatrix{k}(i, j);
                        label = sprintf('%.3f', r_v);
                        if showPValue && i ~= j
                            label = [label, obj.sigMark(obj.PValues{k}(i,j))]; %#ok<AGROW>
                        end
                        text(j, i, label, ...
                            'HorizontalAlignment', 'center', ...
                            'FontSize', obj.FS, 'FontWeight', 'bold', ...
                            'Color', obj.cellTextColor(r_v));
                    end
                end

                if obj.fTitle
                    title(sprintf('相関係数行列ヒートマップ\n%s', obj.FileNames{k}), ...
                        'FontSize', obj.FS*1.2, 'FontWeight', 'bold');
                    if showPValue
                        subtitle('* p<0.05   ** p<0.01   *** p<0.001', 'FontSize', obj.FS*0.9);
                    end
                end
            end
        end

        % ── 5. 散布図行列 ────────────────────────────────────────────
        function plotScatterMatrix(obj)
            % plotScatterMatrix  各試行の散布図行列（対角: ヒストグラム）
            % ！！！！！動作がアホほど重たい！！！！！

            for k = 1:obj.NumTrials
                obj.calcCorrInternal(k);
                D      = obj.LogData{k};
                colors = lines(obj.M);

                figure('Name', sprintf('散布図行列: %s', obj.FileNames{k}), ...
                       'Position', obj.figPos(2, k));

                for i = 1:obj.M
                    for j = 1:obj.M
                        subplot(obj.M, obj.M, (i-1)*obj.M + j);

                        if i == j
                            histogram(D(:, i), 20, ...
                                'FaceColor', colors(i,:), ...
                                'EdgeColor', 'white', 'FaceAlpha', 0.8);
                            title(obj.VarNames{i}, 'FontSize', obj.FS*0.8, 'FontWeight', 'bold');
                        else
                            xd = D(:, j);  yd = D(:, i);
                            scatter(xd, yd, 15, [0.25, 0.5, 0.75], ...
                                'filled', 'MarkerFaceAlpha', 0.4);
                            hold on;
                            p  = polyfit(xd, yd, 1);
                            xr = linspace(min(xd), max(xd), 60);
                            plot(xr, polyval(p, xr), 'r-', 'LineWidth', 1.5);

                            r_v = obj.CorrMatrix{k}(i, j);
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
                        if j == 1,     ylabel(obj.VarNames{i}, 'FontSize', obj.FS*0.7, 'Interpreter','latex'); end
                        if i == obj.M, xlabel(obj.VarNames{j}, 'FontSize', obj.FS*0.7, 'Interpreter','latex'); end
                    end
                end

                sgtitle(sprintf('散布図行列: %s', obj.FileNames{k}), ...
                    'FontSize', obj.FS, 'FontWeight', 'bold');
            end
        end

        % ── 6. 分散の棒グラフ ────────────────────────────────────────
        function plotVarianceBar(obj)
            % plotVarianceBar  各試行の分散・標準偏差を棒グラフで比較

            for k = 1:obj.NumTrials
                obj.calcVarianceInternal(k);

                figure('Name', sprintf('分散・標準偏差: %s', obj.FileNames{k}), ...
                       'Position', obj.figPos(3, k));

                subplot(1, 2, 1);
                ax = gca;
                bar(obj.Variances{k}, 'FaceColor', [0.2, 0.5, 0.8], 'FaceAlpha', 0.8);
                set(ax, 'XTick', 1:obj.M, 'XTickLabel', obj.VarNames, ...
                    'XTickLabelRotation', 30, 'FontSize', obj.FS);
                ax.TickLabelInterpreter = "latex";
                ylabel('分散（不偏）');
                title('各変数の分散', 'FontWeight', 'bold');
                grid on; box off;

                subplot(1, 2, 2);
                ax = gca;
                bar(obj.StdDevs{k}, 'FaceColor', [0.2, 0.7, 0.5], 'FaceAlpha', 0.8);
                set(ax, 'XTick', 1:obj.M, 'XTickLabel', obj.VarNames, ...
                    'XTickLabelRotation', 30, 'FontSize', obj.FS);
                ax.TickLabelInterpreter = "latex";
                ylabel('標準偏差');
                title('各変数の標準偏差', 'FontWeight', 'bold');
                grid on; box off;

                sgtitle(sprintf('分散・標準偏差の比較: %s', obj.FileNames{k}), ...
                    'FontSize', obj.FS*1.1, 'FontWeight', 'bold');
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
            obj.plotScatterMatrix();
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
        function calcVarianceInternal(obj, k)
            if isempty(obj.Means{k})
                D             = obj.LogData{k};
                obj.Means{k}     = mean(D);
                obj.Variances{k} = var(D);
                obj.StdDevs{k}   = std(D);
                obj.CVs{k}       = obj.StdDevs{k} ./ abs(obj.Means{k}) * 100;
            end
        end

        % ---- 相関係数キャッシュ計算 ------------------------------------
        function calcCorrInternal(obj, k)
            if isempty(obj.CorrMatrix{k})
                obj.CovMatrix{k}  = cov(obj.LogData{k});
                obj.CorrMatrix{k} = corrcoef(obj.LogData{k});
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

    end % static methods

end % classdef
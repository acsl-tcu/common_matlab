% ── 有意性検定 ────────────────────────────────────────────
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

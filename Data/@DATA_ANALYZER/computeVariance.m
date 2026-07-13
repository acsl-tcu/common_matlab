% ── 分散・標準偏差の計算 ──────────────────────────────────
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


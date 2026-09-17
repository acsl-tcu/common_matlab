% ── 相関係数行列の計算 ─────────────────────────────────────
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
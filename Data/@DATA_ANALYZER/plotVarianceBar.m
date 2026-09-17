% ── 分散の棒グラフ ────────────────────────────────────────
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
                'FontSize', obj.FS*1.1, 'FontWeight', 'bold', 'Interpreter', 'none');
        end
    end
end
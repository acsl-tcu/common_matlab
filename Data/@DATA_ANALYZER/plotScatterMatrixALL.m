% ── 散布図行列 ────────────────────────────────────────────
function plotScatterMatrixALL(obj)
    % plotScatterMatrixALL  各試行の散布図行列（対角: ヒストグラム）

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
                sgtitle(sprintf('散布図行列: %s', lbl), 'FontSize', obj.FS, 'FontWeight', 'bold', 'Interpreter', 'none');
            else
                sgtitle(sprintf('ラグ散布図行列 (step=%d): %s', step, lbl), 'FontSize', obj.FS, 'FontWeight', 'bold', 'Interpreter', 'none');
            end
        end
    end
end

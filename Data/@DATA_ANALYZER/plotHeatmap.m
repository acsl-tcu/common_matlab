% ── 相関ヒートマップ ──────────────────────────────────────
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
            title(hmap_title, 'FontSize', obj.FS*1.2, 'FontWeight', 'bold', 'Interpreter', 'none');
            if showPValue
                subtitle('* p<0.05   ** p<0.01   *** p<0.001', 'FontSize', obj.FS*0.9, 'Interpreter', 'none');
            end
        end
    end
end

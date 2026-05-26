% ── 散布図（ペアごとに個別ウィンドウ）────────────────────
function plotScatterMatrixEACH(obj)
    % plotScatterMatrixEACH
    %   変数ペア (i,j) ごとに独立したウィンドウを生成する。
    %   M×M 個のウィンドウを画面上にタイル状に配置するため、
    %   画面解像度を取得してウィンドウサイズ・位置を自動算出する。

    for k = 1:obj.loopLen()
        ci      = obj.cacheIdx(k);
        step    = obj.step;
        lbl     = obj.trialLabel(k);
        obj.calcCorrInternal(ci);
        [D_base, D_lag] = obj.getBaseLagData(k);
        colors = lines(obj.M);

        % --- 画面サイズからウィンドウサイズを算出 ---
        screen_sz = get(0, 'ScreenSize');   % [left, bottom, width, height]
        sw = screen_sz(3);   % 画面幅 [px]
        sh = screen_sz(4);   % 画面高 [px]
        margin = 4;          % ウィンドウ間余白 [px]
        win_w  = floor((sw - margin * (obj.M + 1)) / obj.M);
        win_h  = floor((sh - margin * (obj.M + 1)) / obj.M);

        % --- 変数ペアごとにウィンドウを生成 ---
        for i = 1:obj.M
            for j = 1:obj.M
                % タイル位置（列j: 左→右, 行i: 上→下）
                pos_x = margin + (j - 1) * (win_w + margin);
                pos_y = sh - i * (win_h + margin);   % MATLABはY軸下から

                fig_name = sprintf('(%s) vs (%s)  |  %s', ...
                    obj.VarNames{i}, obj.VarNames{j}, lbl);
                figure('Name', fig_name, ...
                       'NumberTitle', 'off', ...
                       'Position', [pos_x, pos_y, win_w, win_h], ...
                       'MenuBar', 'none', 'ToolBar', 'none');
                ax = axes('Parent', gcf); %#ok<LAXES>

                if i == j
                    % 対角: ヒストグラム
                    histogram(ax, D_base(:, i), 20, ...
                        'FaceColor', colors(i,:), ...
                        'EdgeColor', 'white', 'FaceAlpha', 0.8);
                    if step == 0
                        xlabel(ax, obj.VarNames{i}, 'Interpreter', 'latex', 'FontSize', obj.FS);
                    else
                        xlabel(ax, sprintf('%s\,[k]', obj.VarNames{i}), 'Interpreter', 'latex', 'FontSize', obj.FS);
                    end
                    ylabel(ax, 'count', 'FontSize', obj.FS);
                else
                    % 非対角: 散布図 + 回帰直線
                    xd = D_base(:, j);
                    yd = D_lag(:, i);
                    scatter(ax, xd, yd, 12, [0.25, 0.5, 0.75], ...
                        'filled', 'MarkerFaceAlpha', 0.4);
                    hold(ax, 'on');
                    p  = polyfit(xd, yd, 1);
                    xr = linspace(min(xd), max(xd), 60);
                    plot(ax, xr, polyval(p, xr), 'r-', 'LineWidth', 1.5);
                    hold(ax, 'off');
                    r_v = obj.CorrMatrix{ci}(i, j);
                    tc  = [0.3, 0.3, 0.3];
                    if abs(r_v) >= 0.5, tc = [0.75, 0.1, 0.1]; end
                    text(ax, 0.97, 0.96, sprintf('r = %.3f', r_v), ...
                        'Units', 'normalized', ...
                        'HorizontalAlignment', 'right', ...
                        'VerticalAlignment', 'top', ...
                        'FontSize', obj.FS, 'Color', tc, 'FontWeight', 'bold');
                    if step == 0
                        xlabel(ax, obj.VarNames{j}, 'Interpreter', 'latex', 'FontSize', obj.FS);
                        ylabel(ax, obj.VarNames{i}, 'Interpreter', 'latex', 'FontSize', obj.FS);
                    else
                        xlabel(ax, sprintf('%s\,[k]',    obj.VarNames{j}), 'Interpreter', 'latex', 'FontSize', obj.FS);
                        ylabel(ax, sprintf('%s\,[k+%d]', obj.VarNames{i}, step), 'Interpreter', 'latex', 'FontSize', obj.FS);
                    end
                end
                grid(ax, 'on'); box(ax, 'on');
                ax.FontSize = obj.FS * 0.85;
            end
        end
    end
end
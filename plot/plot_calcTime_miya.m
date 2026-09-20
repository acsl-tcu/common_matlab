t = log.Data.t(:);
phase = log.Data.phase(:);
x = (1:length(t)).';

figure
hold on

% ===== メインの線 =====
hData = plot(x, t, ...
    'LineWidth', 1.5, ...
    'DisplayName', 't');

yl = ylim;

%% ===== phaseの切り替わり位置 =====

% 前後が同じphaseか判定
% NaN → NaN も「同じphase」とみなす
samePhase = ...
    (phase(2:end) == phase(1:end-1)) | ...
    (isnan(phase(2:end)) & isnan(phase(1:end-1)));

idx_change = [
    1;
    find(~samePhase) + 1;
    length(phase)+1
];

%% ===== 実際に存在するphase一覧 =====
% NaNは除外
phase_list = unique(phase(~isnan(phase)));

colors = lines(length(phase_list));

%% ===== phaseごとの背景 =====

for k = 1:length(idx_change)-1

    i1 = idx_change(k);
    i2 = idx_change(k+1)-1;

    ph = phase(i1);

    % NaN区間は背景を塗らない
    if isnan(ph)
        continue
    end

    cidx = find(phase_list == ph, 1);

    vertices = [
        i1, yl(1);
        i2, yl(1);
        i2, yl(2);
        i1, yl(2)
    ];

    patch( ...
        'Vertices', vertices, ...
        'Faces', [1 2 3 4], ...
        'FaceColor', colors(cidx,:), ...
        'FaceAlpha', 0.15, ...
        'EdgeColor', 'none', ...
        'HandleVisibility', 'off');

end

%% ===== phaseの凡例 =====

hPhase = gobjects(length(phase_list),1);

for k = 1:length(phase_list)

    hPhase(k) = plot(NaN, NaN, 's', ...
        'MarkerSize', 10, ...
        'MarkerFaceColor', colors(k,:), ...
        'MarkerEdgeColor', colors(k,:), ...
        'LineStyle', 'none', ...
        'DisplayName', sprintf('phase = %g', phase_list(k)));

end

%% ===== 表示調整 =====

uistack(hData, 'top');

legend([hData; hPhase], 'Location', 'best');

xlabel('Sample');
xlim([0 6500])
ylabel('t');

hold off
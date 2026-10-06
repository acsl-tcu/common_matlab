%% 保存済みgainファイルからAの固有値分布を再プロット

clear;
clc;
close all;

%% ===== 読み込むgainファイルを指定 =====

gainFileName = input('gainファイル名(.mat可): ', 's');

if isempty(regexp(gainFileName, '\.mat$', 'once'))
    gainFileName = [gainFileName '.mat'];
end

% gainファイルが保存されているフォルダ
gainFolder = fullfile(pwd, 'kalman_gainたち');

gainFilePath = fullfile(gainFolder, gainFileName);

if ~exist(gainFilePath, 'file')
    error('指定されたgainファイルが見つかりません。\n%s', gainFilePath);
end

%% ===== gainファイル読み込み =====

data = load(gainFilePath);

% 必要なデータがあるか確認
requiredFields = {'A','eigs_a','eigs_b','eigs_c','eigs_d'};

for i = 1:numel(requiredFields)
    if ~isfield(data, requiredFields{i})
        error('gainファイルに %s が保存されていません。', ...
            requiredFields{i});
    end
end

A = data.A;

eigs_a = data.eigs_a;
eigs_b = data.eigs_b;
eigs_c = data.eigs_c;
eigs_d = data.eigs_d;

n = size(A,1);

%% ===== 固有値を表示 =====

fprintf('\n===== 各カテゴリの固有値 =====\n');

local_print_spectrum('Xa 可制御・不可観測', eigs_a);
local_print_spectrum('Xb 可制御・可観測',   eigs_b);
local_print_spectrum('Xc 不可制御・可観測', eigs_c);
local_print_spectrum('Xd 不可制御・不可観測', eigs_d);

%% ===== 固有値分布をplot =====

figure('Color','w','Name','Kalman canonical decomposition');

hold on;
grid on;
axis equal;

% 単位円
theta = linspace(0, 2*pi, 400);
plot(cos(theta), sin(theta), ...
    'k:', ...
    'HandleVisibility','off');

% 各カテゴリ
local_plot_category( ...
    eigs_a, [1 0 0], 'o', ...
    'Xa: 可制御・不可観測');

local_plot_category( ...
    eigs_b, [0 0 1], 's', ...
    'Xb: 可制御・可観測');

local_plot_category( ...
    eigs_c, [0 0.6 0], 'd', ...
    'Xc: 不可制御・可観測');

local_plot_category( ...
    eigs_d, [0.7 0 0.7], 'x', ...
    'Xd: 不可制御・不可観測');

xlabel('実部');
ylabel('虚部');

title(sprintf('カルマン正準分解 (%d状態)', n));

legend('Location','northeastoutside');

%% ===== 表示範囲 =====

allEig = [eigs_a; eigs_b; eigs_c; eigs_d];

if ~isempty(allEig)

    lim = 1.1 * max([1.2; abs(allEig)]);

    xlim([-lim lim]);
    ylim([-lim lim]);

end


%% =========================================================
% 以下、補助関数
% ==========================================================

function local_print_spectrum(name, e)

    if isempty(e)

        fprintf('%s: なし\n', name);
        return;

    end

    if all(abs(e) < 1)
        stableText = 'YES';
    else
        stableText = 'NO';
    end

    fprintf('%s: %d個, max|lambda|=%.6g, 単位円内=%s\n', ...
        name, numel(e), max(abs(e)), stableText);

    disp(e(:));

end


function local_plot_category(e, color, marker, labelText)

    if isempty(e)
        return;
    end

    plot(real(e), imag(e), ...
        'LineStyle','none', ...
        'Marker',marker, ...
        'MarkerEdgeColor',color, ...
        'MarkerFaceColor','none', ...
        'LineWidth',1.5, ...
        'MarkerSize',9, ...
        'DisplayName',sprintf('%s (%d)', ...
        labelText, numel(e)));

end
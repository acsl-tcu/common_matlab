%紐のベクトルを位置から求めるプログラム　世界座標系と機体座標系
%% 説明
% 1) センサ値から求めた紐ベクトル (牽引物位置 - ドローン位置) と estimator.result.state.pT の比較
% 2) センサのドローン位置と推定値(p)の比較
% 3) センサの牽引物位置と推定値(estimator.result.state.pL)の比較
% 4) x-z平面, y-z平面での紐と鉛直のなす角 (atan2d) の比較 (センサ vs 推定)
% ※ plot_from_saved_result.m と同じplotフォルダに置くこと

%% 初期化&パスの設定
clear all
cf = pwd;

if contains(mfilename('fullpath'), "mainGUI")
    cd(fileparts(mfilename('fullpath')));
else
    tmp = matlab.desktop.editor.getActive;
    cd(fileparts(erase(tmp.Filename, "plot\plot_cable_vector.m")));
end

[~, tmp] = regexp(genpath('.'), '\.\\\.git.*?;', 'match', 'split');
cellfun(@(xx) addpath(xx), tmp, 'UniformOutput', false);
close all hidden; clear; clc;
userpath('clear');

%% データの読み込み
fprintf('MATファイルを選択してください:')
[filename, pathname] = uigetfile('*.mat', 'MATファイルを選択してください');
fprintf(filename);
logger = LOGGER(fullfile(pathname, filename));

%% プロット
clearvars -except logger filename
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fsave = 0;                  % 0:no save, 1:".fig", 2:".png", 3:".jpg", 4:".pdf", 5:".eps"
settings.phase = "f";
settings.fontsize = 11;
settings.legend_fontsize = settings.fontsize - 4;
settings.linewidth = 1.5;
settings.savefolder = 'plot\fig';
settings.savename = 'cable_vector';

settings.col_drone = 1:3;   % sensor.result.output 内のドローン位置の列
settings.col_load  = 7:9;   % sensor.result.output 内の牽引物位置の列
settings.cable_sign = 1;    % 1: 牽引物 - ドローン, -1: ドローン - 牽引物 (pTの向きに合わせる)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% ===== データ取得 =====
[t, idx] = get_phase_time(logger, settings.phase);
get = @(field, att) align_rows(logger, logger.data(1, field, att, "phase", settings.phase), idx);

so   = get("sensor.result.output", "s");
pT   = get("estimator.result.state.pT", "e");
p_e  = get("p", "e");
pL_e = get("estimator.result.state.pL", "e");

N = min([size(so,1), size(pT,1), size(p_e,1), size(pL_e,1), numel(t)]);
t    = t(1:N);
p_s  = so(1:N, settings.col_drone);
pL_s = so(1:N, settings.col_load);
v_s  = settings.cable_sign * (pL_s - p_s);   % センサ由来の紐ベクトル
v_e  = pT(1:N, 1:3);
p_e  = p_e(1:N, 1:3);
pL_e = pL_e(1:N, 1:3);

% ===== 図1: 紐ベクトル (センサ由来 vs pT) =====
fig1 = compare_plot(1, t, v_s, v_e, 'Cable vector', settings);
save_figure(fig1, fsave, settings, filename, 'cable_vector');

% ===== 図2: ドローン位置 (センサ vs 推定) =====
fig2 = compare_plot(2, t, p_s, p_e, 'Drone position [m]', settings);
save_figure(fig2, fsave, settings, filename, 'drone_position');

% ===== 図3: 牽引物位置 (センサ vs 推定) =====
fig3 = compare_plot(3, t, pL_s, pL_e, 'Load position [m]', settings);
save_figure(fig3, fsave, settings, filename, 'load_position');

% ===== 図4: x-z平面, y-z平面での紐と鉛直のなす角 (atan2d, センサ vs 推定) =====
% |vz| を使い、紐が真下に垂れていれば0deg、傾く向きは符号で表す
ang_s = [atan2d(v_s(:,1), abs(v_s(:,3))), atan2d(v_s(:,2), abs(v_s(:,3)))];
ang_e = [atan2d(v_e(:,1), abs(v_e(:,3))), atan2d(v_e(:,2), abs(v_e(:,3)))];

fig4 = figure(4); clf
ylabels = {'Angle in x-z plane [deg]', 'Angle in y-z plane [deg]'};
for k = 1:2
    subplot(2,1,k)
    plot(t, ang_s(:,k), '--', 'LineWidth', settings.linewidth)
    hold on
    ax = gca; ax.ColorOrderIndex = 1;
    plot(t, ang_e(:,k), '-', 'LineWidth', settings.linewidth)
    hold off
    grid on
    ylabel(ylabels{k}, 'FontSize', settings.fontsize)
    legend({'sen.','est.'}, 'Location','southeast', 'FontSize', settings.legend_fontsize)
    set(gca, 'FontSize', settings.fontsize)
end
xlabel('$t$ [s]', 'Interpreter','latex', 'FontSize', settings.fontsize)
save_figure(fig4, fsave, settings, filename, 'cable_angle');

%% Local functions

function fig = compare_plot(num, t, d_sen, d_est, ylab, settings)
% センサ(破線)と推定(実線)を x, y, z で色を揃えて重ねる
fig = figure(num); clf
plot(t, d_sen, '--', 'LineWidth', settings.linewidth)
hold on
ax = gca; ax.ColorOrderIndex = 1;
plot(t, d_est, '-', 'LineWidth', settings.linewidth)
hold off
grid on
xlabel('$t$ [s]', 'Interpreter','latex', 'FontSize', settings.fontsize)
ylabel(ylab, 'FontSize', settings.fontsize)
legend({'$x$ sen.','$y$ sen.','$z$ sen.','$x$ est.','$y$ est.','$z$ est.'}, ...
    'Interpreter','latex', 'Location','southeast', 'FontSize', settings.legend_fontsize)
set(gca, 'FontSize', settings.fontsize)
end

function [t, idx] = get_phase_time(logger, phase)
% LOGGER内部(C_Logger_Plot.get_data_range)と同じ方法でphase区間の添字と時刻を求める
phaseString = char(phase);
ids = contains(string(char(logger.Data.phase)), string(phaseString(:)));
idx = 4 + find(ids(5:end), 1):4 + find(ids(5:end), 1, 'last');   % 空回しの4つ分を除く
t = logger.Data.t(idx);
t = t(:);
end

function d = align_rows(logger, d, idx)
% 記録全体の長さで返ってきたデータは，phase区間に切り出す
if size(d,1) == numel(logger.Data.t) && size(d,1) ~= numel(idx)
    d = d(idx, :);
end
end

function save_figure(fig, fsave, settings, filename, tag)
if fsave < 1 || fsave > 5, return; end
if ~exist('plot/fig', 'dir'), mkdir('plot/fig'); end
exts = {'.fig','.png','.jpg','.pdf','.eps'};
str = [char(settings.savefolder), '\', char(settings.savename), '_', tag, exts{fsave}];
if fsave == 1
    savefig(fig, str);
else
    exportgraphics(fig, str);
end
end
%% 説明
% timerange_plot_save_result.m
% 2025/06 作成者：小関
% Exp / Simデータをプロットすることができるファイル
% 時間範囲（settings.trange）指定＋保存対応版

%% 初期化&パスの設定
clear all
cf = pwd;

if contains(mfilename('fullpath'), "mainGUI")
    cd(fileparts(mfilename('fullpath')));
else
    tmp = matlab.desktop.editor.getActive;
    cd(fileparts(erase(tmp.Filename, "plot\timerange_plot_save_result.m")));
end

[~, tmp] = regexp(genpath('.'), '\.\\\.git.*?;', 'match', 'split');
cellfun(@(xx) addpath(xx), tmp, 'UniformOutput', false);
close all hidden; clear; clc;
userpath('clear');

%% データの読み込み
fprintf('MATファイルを選択してください:')
[filename, pathname] = uigetfile('*.mat', 'MATファイルを選択してください');
fprintf(filename);
fullpath = fullfile(pathname, filename);
logger = LOGGER(fullpath);

%% プロット設定
clearvars -except logger filename

fsave = 0;                % 0:no save
ftitle = 0;
settings.fcolor = 0;

%%%%%%%%%%%%%%%%%%%%%%%% chose target %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
settings.target = ["p", "input", "inner_input", "p1-p2","controller.result.mL"]; %質量推定用
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 % ← コメントアウトで切替
 %コメントアウトしなかったら，time 指定が優先され、phase は無視される
settings.phase  = "tf";
settings.trange = [5 18];    

settings.fontsize  = 16;
settings.linewidth = 1.5;
settings.agent_id  = 1;
settings.savefolder = 'plot\fig';
settings.savename   = '2_HLonly_triangle';

if logger.fExp == 1
    settings.attribute = ["e", "s", "r"];
else
    settings.attribute = ["e", "s", "r", "p"];
end

%% ===== time / phase インデックス作成（★変更箇所）=====
t_all = logger.Data.t(:);
idx_t = true(size(t_all));

% time 指定（settings.trange が有効な場合）
if exist('settings','var') && isfield(settings,'trange') && ~isempty(settings.trange)
    idx_t = (t_all >= settings.trange(1)) & ...
            (t_all <= settings.trange(2));

% phase 指定（settings.trange がコメントアウトされている場合）
elseif exist('settings','var') && isfield(settings,'phase') && ~isempty(settings.phase)
    idx_t = contains(logger.Data.phase, settings.phase);
end

%% プロット本体 ===================================================================================
for i = 1:length(settings.target)

    fig = figure(i); clf
    ax  = axes(fig); hold(ax,'on'); grid(ax,'on')

    target = settings.target(i);
    fcolor = settings.fcolor;

    switch target
        case "p"
            ylabel = "Position [m]";
            tmp = settings.attribute;
            att = select_attribute(settings.target(i), tmp);
        case "q"
            ylabel = "Angle [rad]";
            tmp = settings.attribute;
            tmp(3) = []; % "esr"の内，無いものを消去
            att = select_attribute(settings.target(i), tmp);
        case "v"
            ylabel = "Velocity [m/s]";
            tmp = settings.attribute;
            tmp(2) = [];
            att = select_attribute(settings.target(i), tmp);
        case "w"
            ylabel = "Angular velocity [rad/s]";
            tmp = settings.attribute;
            tmp(2:3) = [];
            att = select_attribute(settings.target(i), tmp);
        case "input"
            ylabel = "Controller input [N],[Nm]";
            tmp = settings.attribute;
            tmp(2:3) = [];
            att = "";
        case "inner_input"
            ylabel = "Transmitter input [N],[Nm]";
            tmp = settings.attribute;
            tmp(:) = [];
            tmp = "";
            att = "";
        case "inner_input1:4"
            ylabel = "Transmitter input [N], [Nm]";
            tmp = settings.attribute;
            tmp(:) = [];
            tmp = "";
            att = "";
        case "p1-p2"
            xlabel = "$x$ [m]";
            ylabel = "$y$ [m]";
            tmp = settings.attribute;
            fcolor = 0;
            att = select_attribute(settings.target(i), tmp);
        case "p1-p2-p3"
            xlabel = "$x$ [m]";
            ylabel = "$y$ [m]";
            zlabel = "$z$ [m]";
            tmp = settings.attribute;
            fcolor = 0;
            att = select_attribute(settings.target(i), tmp);
            case "controller.result.xd1:3"
    ylabel = "Reference position $x_d$ [m]";
    tmp = "";        % attribute は使わない
    att = "";        % ← 重要

    end

    %% 通常プロット（LOGGER）
    logger.plot({settings.agent_id, target, att}, ...
        'fig_num', i, 'color', fcolor, ...
        'phase', settings.phase, ...
        'FontSize', settings.fontsize, ...
        'Linewidth', settings.linewidth);

    %% ===== idx_t で切り出し =====
    lines = findobj(ax,'Type','line');
    for h = lines.'
        X = get(h,'XData');
        Y = get(h,'YData');
        if numel(X) == numel(t_all)
            set(h,'XData', X(idx_t), 'YData', Y(idx_t))
        end
    end

    %% ラベル
    set(ax.YLabel,'String',ylabel_txt,'Interpreter','latex')

    %% タイトル除去
    if ftitle == 0
        title(ax,'')
    end

    %% 凡例
    if ~isempty(att)
        chars = string(split(att,""));
        chars(chars=="")=[];
        plegend = set_legend(target, chars);
        legend(ax,plegend,'Interpreter','latex','Location','southeast')
    end
end

disp_rmse(logger, idx_t)

%% ===== ローカル関数 =====
function att = select_attribute(target, attribute)
fprintf('\n<%s attribute input>\n',target)
fprintf('  available: {%s}\n',strjoin(attribute,""))
while true
    att = string(input('Input attribute: ','s'));
    chars = string(split(att,""));
    chars(chars=="")=[];
    if all(ismember(chars,attribute))
        break;
    else
        fprintf('incorrect\n')
    end
end
end

function legend = set_legend(target, chars)
att_map = containers.Map({'e','s','r','p'},...
    {'est.','sen.','ref.','true'});
legend = {};
switch target
    case "p"
        for i=1:numel(chars)
            legend{3*i-2} = "$x$ " + att_map(chars(i));
            legend{3*i-1} = "$y$ " + att_map(chars(i));
            legend{3*i} = "$z$ " + att_map(chars(i));
        end
    case "v"
        for i=1:numel(chars)
            legend{3*i-2} = "$v_x$ " + att_map(chars(i));
            legend{3*i-1} = "$v_y$ " + att_map(chars(i));
            legend{3*i} = "$v_z$ " + att_map(chars(i));
        end
    case "q"
        for i=1:numel(chars)
            legend{3*i-2} = "$\theta_{roll}$ " + att_map(chars(i));
            legend{3*i-1} = "$\theta_{pitch}$ " + att_map(chars(i));
            legend{3*i} = "$\theta_{yaw}$ " + att_map(chars(i));
        end
    case "w"
        for i=1:numel(chars)
            legend{3*i-2} = "$\omega_{roll}$ " + att_map(chars(i));
            legend{3*i-1} = "$\omega_{pitch}$ " + att_map(chars(i));
            legend{3*i} = "$\omega_{yaw}$ " + att_map(chars(i));
        end
    case "input"
        legend{1} = "$u_{thrust}$";
        legend{2} = "$u_{roll}$";
        legend{3} = "$u_{pitch}$";
        legend{4} = "$u_{yaw}$";
    case "input2:4"
        legend{1} = "$u_{roll}$";
        legend{2} = "$u_{pitch}$";
        legend{3} = "$u_{yaw}$";
    case "inner_input1:4"
        legend{1} = "$u_{roll}$";
        legend{2} = "$u_{pitch}$";
        legend{3} = "$u_{thrust}$";
        legend{4} = "$u_{yaw}$";
end
end

%% RMSEの計算
function disp_rmse(logger, idx_t)
p_est = logger.data(1,"p","e");
p_ref = logger.data(1,"p","r");

N = min(size(p_est,1), size(p_ref,1));
p_est = p_est(1:N,:);
p_ref = p_ref(1:N,:);
idx_t = idx_t(1:N);

p_est = p_est(idx_t,:);
p_ref = p_ref(idx_t,:);

diff = p_est - p_ref;

fprintf('\n===== Position RMSE =====\n');
fprintf(' RMSE_x = %.6f [m]\n', sqrt(mean(diff(:,1).^2)));
fprintf(' RMSE_y = %.6f [m]\n', sqrt(mean(diff(:,2).^2)));
fprintf(' RMSE_z = %.6f [m]\n', sqrt(mean(diff(:,3).^2)));
fprintf('=========================\n\n');
end

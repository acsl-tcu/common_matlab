%% 説明 plot_from_saved_result
% 2025/06 作成者：小関
% Exp / Simデータをプロットすることができるファイル
% 最初は全てのセクションを実行する．
% データの読み込みができたら，プロットセクションだけ実行すれば手間が省ける．
% プロットセクションのsettingsを変更して調整することでmainGUIの画面上とは違ういい感じのグラフが取れる
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 50行付近のsettingsは色々見てください！！  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 分からないことや追加したい機能などあったらSlackで聞いてください
% 凡例に関しては，書き方・位置を要検討
% プロットしたいフェーズを選べるようになると嬉しい

%% 初期化&パスの設定
clear all
cf = pwd;

if contains(mfilename('fullpath'), "mainGUI")
    cd(fileparts(mfilename('fullpath')));
else
    tmp = matlab.desktop.editor.getActive;
    cd(fileparts(erase(tmp.Filename, "plot\plot_from_saved_result.m")));
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

%% プロット
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fsave = 0;
% [Recomendation] Initially, you should check the figure with fsave = 0, then chose save style.
% [推奨] 最初はfsave = 0でfigureを確認し，その後 保存形式を選択
% 0:no save
% 1:save as ".fig"
% 2:save as ".png"
% 3:save as ".pdf"
% 4:save as ".eps"

ftitle = 1; % defalt=1 -> グラフタイトルあり
settings.fcolor = 1; % default=1 -> フェーズごとの背景色あり

%%%%%%%%%%%%%%%%%%%%%%%% chose target %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
settings.target = ["p", "q", "v", "w", "input", "p1-p2", "p1-p2-p3"];
% settings.target = ["p", "q", "v", "w", "input", "inner_input", "p1-p2", "p1-p2-p3", "contlloer.result.P(6)"];
% settings.target = ["p", "v", "input"];
% プロットしたいグラフの情報                                          %
% p: position    q: angle    v: velocity    w: angular velocity     %
% input: controller input                                           %
% p1-p2: x-y 2D plot    p1-p2-p3: x-y-z 3D plot                     %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

settings.fontsize = 11;    % default=11 オススメ=18
settings.linewidth = 1.5;    % default=0.5 オススメ=1.5
settings.agent_id = 1;

% estimator, sensor, reference, (plant) どの値を表示するかは
% 途中のキーボード入力で決定します．
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if logger.fExp == 1
    settings.attribute = ["e", "s", "r"];
else
    settings.attribute = ["e", "s", "r", "p"];
end

for i=1:length(settings.target)
    fcolor = settings.fcolor;
    switch settings.target(i)
        case "p"
            ylabel = "Position [m]";
            tmp = settings.attribute;
        case "q"
            ylabel = "Attitude [rad]";
            tmp = settings.attribute;
            tmp(3) = []; % "esr"の内，無いものを消去
        case "v"
            ylabel = "Velocity [m/s]";
            tmp = settings.attribute;
            tmp(2) = [];
        case "w"
            ylabel = "Angular velocity [rad/s]";
            tmp = settings.attribute;
            tmp(2:3) = [];
        case "input"
            ylabel = "Controller input [N]";
            tmp = settings.attribute;
            tmp(2:3) = [];
        case "inner_input"
            ylabel = "Controller input [N]";
            tmp = settings.attribute;
            tmp(2:3) = [];
        case "p1-p2"
            xlabel = "x [m]";
            ylabel = "y [m]";
            tmp = settings.attribute;
            fcolor = 0;
        case "p1-p2-p3"
            xlabel = "x [m]";
            ylabel = "y [m]";
            zlabel = "z [m]";
            tmp = settings.attribute;
            fcolor = 0;
        case "contlloer.result.P(6)"
            ylabel = "mass [kg]";
            tmp = settings.attribute;
            tmp(2:3) = [];
    end
    att = select_attribute(settings.target(i), tmp);
    logger.plot({settings.agent_id, settings.target(i), att}, ...
        'fig_num',i, 'color',fcolor, ...
        'FontSize',settings.fontsize, 'Linewidth',settings.linewidth)

    fig = gcf;
    ax = gca;

    chars = string(split(att, ""));
    chars(chars == "") = [];
    switch settings.target(i)
        case "p1-p2"
            set(ax.XLabel, 'String', xlabel)
            set(ax.YLabel, 'String', ylabel)
        case "p1-p2-p3"
            set(ax.XLabel, 'String', xlabel)
            set(ax.YLabel, 'String', ylabel)
            set(ax.ZLabel, 'String', zlabel)
        otherwise
            set(ax.YLabel, 'String', ylabel)
            legend = set_legend(settings.target(i), chars);
            set(ax.Legend, 'String', legend, 'Interpreter','latex');
    end
    if ftitle == 0
        set(ax.Title, 'String', [])
    end
    set(ax.Legend, 'Location', 'northwest', 'FontSize', settings.fontsize-4);

    if ~exist('plot/fig', 'dir')
        mkdir('plot/fig')
    end
    if fsave == 1
        savefig(['plot/fig/', erase(filename, '.mat'), char(settings.target(i)), '.fig']);
    elseif fsave == 2
        saveas(fig, ['plot/fig/', erase(filename, '.mat'), char(settings.target(i))], 'png');
    elseif fsave == 3
        saveas(fig, ['plot/fig/', erase(filename, '.mat'), char(settings.target(i))], 'pdf');
    elseif fsave == 4
        saveas(fig, ['plot/fig/', erase(filename, '.mat'), char(settings.target(i))], 'epsc');
    end
end

%% function
function att = select_attribute(target, attribute)
text = cell(1, 4);
text{1} = ['\n<キーボードで「', char(target), '」用の値の種類を入力>\n'];%'\n<Keybord input attribute for [', char(target), ']>\n',
text{2} = ['   {', char(strjoin(attribute, "")), '}が使えます  '];%'You can use {', char(strjoin(attribute, "")), '}\n
if length(attribute) == 4
    text{3} = ['例）', char(attribute(1)), ', ', [char(attribute(1)), char(attribute(2))], ...
        ', ', [char(attribute(1)), char(attribute(2)), char(attribute(3))],...
        ', ', [char(attribute(1)), char(attribute(2)), char(attribute(3)), char(attribute(4))],'\n'];
elseif length(attribute) == 3
    text{3} = ['例）', char(attribute(1)), ', ', [char(attribute(1)), char(attribute(2))], ...
        ', ', [char(attribute(1)), char(attribute(2)), char(attribute(3))], '\n'];
elseif length(attribute) == 2
    text{3} = ['例）', char(attribute(1)), ', ', [char(attribute(1)), char(attribute(2))], '\n'];
else
    text{3} = ['例）', char(attribute(1)), '\n'];
end
text{4} = 'Input attribute: ';
fprintf([text{1:3}])
while true
    att = string(input([text{4}], 's'));
    chars = string(split(att, ""));
    chars(chars == "") = [];
    if all(ismember(chars, attribute))
        break;
    else
        fprintf('!!!%s is incorrect!!! ', att)
    end
end
end



function legend = set_legend(target, chars)
att_map = containers.Map({'e', 's', 'r', 'p'},...
    {'est.', 'sen.', 'ref.', 'pla.'});
legend_num = numel(chars) * 3;
legend = cell(1, legend_num);
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
        legend_num = numel(chars) * 4;
        legend = cell(1, legend_num);
        for i=1:numel(chars)
            legend{4*i-3} = "$Thrust$ " + att_map(chars(i));
            legend{4*i-2} = "$roll$ " + att_map(chars(i));
            legend{4*i-1} = "$pitch$ " + att_map(chars(i));
            legend{4*i} = "$yaw$ " + att_map(chars(i));
        end
    case "inner_input"
        legend_num = numel(chars) * 4;
        legend = cell(1, legend_num);
        for i=1:numel(chars)
            legend{4*i-3} = "$Thrust$ " + att_map(chars(i));
            legend{4*i-2} = "$roll$ " + att_map(chars(i));
            legend{4*i-1} = "$pitch$ " + att_map(chars(i));
            legend{4*i} = "$yaw$ " + att_map(chars(i));
        end
end
end

%% ===== 全 result の x,y,z を連結して最終 RMSE を 1 回だけ計算 =====
ref_results  = logger.Data.agent.reference.result;
sens_results = logger.Data.agent.sensor.result;
numResults = numel(ref_results);
% 全データを貯める配列（可変長）
p_plant_all = [];
p_ref_all   = [];
for i = 1:numResults
    try
        %% --- reference (3×1) ---
        ref = ref_results{1,i}.state;
        if isfield(ref, "p")
            p_ref_single = ref.p;        % 3×1
        else
            p_ref_single = ref.xd(1:3);  % 3×1
        end
        %% --- sensor rigid ---
        sens = sens_results{1,i};
        if ~isfield(sens, 'rigid') || isempty(sens.rigid)
            continue;
        end
        Ns = numel(sens.rigid);
        % plant: 3×Ns
        p_plant = zeros(3, Ns);
        for k = 1:Ns
            pk = sens.rigid(k).p;
            p_plant(:,k) = pk(1:3);
        end
        % reference 3×Ns
        p_ref = repmat(p_ref_single, 1, Ns);
        % ---- 全データに追加 ----
        p_plant_all = [p_plant_all, p_plant];
        p_ref_all   = [p_ref_all,   p_ref];
    catch ME
        warning("result %d の読み込み中にエラー:\n%s", i, getReport(ME));
    end
end
%% ===== 総 RMSE 計算 =====
diff_all = p_plant_all - p_ref_all;
RMSE_x = sqrt(mean(diff_all(1,:).^2));
RMSE_y = sqrt(mean(diff_all(2,:).^2));
RMSE_z = sqrt(mean(diff_all(3,:).^2));
%% ===== 結果表示 =====
fprintf("\n===== 総時間（全 result 合計）の RMSE =====\n");
fprintf(" RMSE_x  = %.6f [m]\n", RMSE_x);
fprintf(" RMSE_y  = %.6f [m]\n", RMSE_y);
fprintf(" RMSE_z  = %.6f [m]\n", RMSE_z);
fprintf("===========================================\n\n");

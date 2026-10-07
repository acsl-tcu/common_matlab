%% 説明
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
clearvars -except logger filename
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fsave = 0;

% [Recomendation] Initially, you should check the figure with fsave = 0, then chose save style.
% [推奨] 最初はfsave = 0でfigureを確認し，その後 保存形式を選択
% 0:no save
% 1:save as ".fig"
% 2:save as ".png"
% 3:save as ".jpg"
% 4:save as ".pdf"
% 5:save as ".eps"

ftitle = 0; % default=1 -> グラフタイトルあり
settings.fcolor = 0; % default=1 -> フェーズごとの背景色あり

%%%%%%%%%%%%%%%%%%%%%%%% chose target %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% settings.target = ["p", "v", "q", "w", "input", "input2:4", "p1-p2"];
% settings.target = ["p",  "input", "estimator.result.state.pL",{{"p", "r"},{"estimator.result.state.pL", "e"}}, "p1-p2"];
settings.target = {"p","q", "input", {"p", "estimator.result.state.pL"}, "p1-p2","estimator.result.state.mL","estimator.result.theat","v"};
% settings.target = {"p","q","p1-p2", "input"};
% settings.target = { "p", "estimator.result.state.pL"};
% settings.target = ["p","q", "input"];
% settings.target = ["p", "input", "inner_input", "p1-p2","estimator.result.state.pL","estimator.result.state.mL"]; %質量推定用 exp
% settings.target = ["p", "v", "q", "w","input", "controller.result.nominal_input", "controller.result.delta_input", "p1-p2", "p1-p2-p3"];
% settings.target = ["p", "controller.result.delta_input"];
% settings.target = ["p", "v","p1-p2"];
% settings.target = ["p1-p2",];
% settings.target = "controller.result.xd";
% "controller.result.nominal_input","controller.result.delta_input"
% settings.target = ["input", "input2:4", "controller.result.nominal_input2:4", "controller.result.delta_input2:4"];
% プロットしたいグラフの情報                                        %
% p: position    q: angle    v: velocity    w: angular velocity     %
% input: controller input    inner_input1:4: transmitter input      %
% p1-p2: x-y 2D plot    p1-p2-p3: x-y-z 3D plot                     %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% settings.phase = "tfl";
settings.phase = "f";
settings.fontsize = 11;    % default=11 オススメ=18　
% settings.fontsize = 22;    % 報告書向け
% settings.fontsize = 24;    % スライド向け
settings.legend_fontsize = settings.fontsize - 4;   % 凡例のフォントサイズ（ここを変えれば全プロットに反映）
% settings.legend_fontsize = 8;
settings.linewidth = 1.5;    % default=0.5 オススメ=1.5
settings.agent_id = 1;
settings.savefolder = 'plot\fig';  % default
% settings.savefolder = "\\192.168.100.209\ws2025\Work2025\YosukeKOSEKI\Drone results(Exp_data)\2025.10.21_DNNMEC triangle and saddle\6_fig\png";

settings.savename = '2_HLonly_triangle';


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
    tgt = settings.target{i};

    if iscell(tgt)
        % ========== 複数フィールドを1つのfigureに重ねてプロットする ==========
        % 例: tgt = {"p", "estimator.result.state.pL"}
        parts = string(tgt);
        ylabel = "Position [m]";
        tmp = settings.attribute;

        field_atts = cell(1, numel(parts));
        for pIdx = 1:numel(parts)
            field_atts{pIdx} = select_attribute(parts(pIdx), tmp);
        end

        tstr = strjoin(parts, '_');   % ファイル名・識別用（switch文には通さない）
        att  = strjoin(cellfun(@char, field_atts, 'UniformOutput', false), '');

        figure(i); clf
        hold on
        legend_entries = {};
        for pIdx = 1:numel(parts)
            chars = string(split(field_atts{pIdx}, ""));
            chars(chars == "") = [];
            for cIdx = 1:numel(chars)
                data = logger.data(1, parts(pIdx), chars(cIdx), "phase", settings.phase);
                t = get_phase_time(logger, settings.phase, size(data,1));   % ★正しい時間軸を取得
                plot(t, data(:,1), t, data(:,2), t, data(:,3), ...
                    'LineWidth', settings.linewidth)
                plegend_part = set_legend("p", chars(cIdx));   % {$x$..,$y$..,$z$..} を再利用
                for axIdx = 1:3
                    legend_entries{end+1} = plegend_part{axIdx} + " (" + parts(pIdx) + ")"; %#ok<AGROW>
                end
            end
        end
        hold off
        grid on

        fig = gcf;
        ax = gca;
        set(ax.YLabel, 'String', ylabel, 'Interpreter','latex')
        set(ax.XLabel, 'String', '$t$ [s]', 'Interpreter','latex')
        legend(legend_entries, 'Interpreter','latex', 'FontSize', settings.legend_fontsize)

        data = [];
        for pIdx = 1:numel(parts)
            chars = string(split(field_atts{pIdx}, ""));
            chars(chars == "") = [];
            for cIdx = 1:numel(chars)
                data = [data; logger.data(1, parts(pIdx), chars(cIdx), "phase", settings.phase)]; %#ok<AGROW>
            end
        end
        y_min = min(data(:)); y_max = max(data(:));
        if y_min == y_max, y_max = y_max + 0.000000001; end
        ylim([y_min y_max])

    else
        % ========== 元の単一フィールド用の処理（変更なし） ==========
        tstr = string(tgt);
        switch tstr
            case "p"
                ylabel = "Position [m]";
                tmp = settings.attribute;
                att = select_attribute(tstr, tmp);
            case "q"
                ylabel = "Angle [rad]";
                tmp = settings.attribute;
                tmp(3) = [];
                att = select_attribute(tstr, tmp);
            case "v"
                ylabel = "Velocity [m/s]";
                tmp = settings.attribute;
                tmp(2) = [];
                att = select_attribute(tstr, tmp);
            case "w"
                ylabel = "Angular velocity [rad/s]";
                tmp = settings.attribute;
                tmp(2:3) = [];
                att = select_attribute(tstr, tmp);
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
                att = select_attribute(tstr, tmp);
            case "p1-p2-p3"
                xlabel = "$x$ [m]";
                ylabel = "$y$ [m]";
                zlabel = "$z$ [m]";
                tmp = settings.attribute;
                fcolor = 0;
                att = select_attribute(tstr, tmp);
            case "controller.result.xd1:3"
                ylabel = "Reference position $x_d$ [m]";
                tmp = "";
                att = "";

            otherwise
                if contains(tstr, 'input')
                    if contains(tstr, 'delta_input')
                        if contains(tstr, '2:4'), ylabel = "Compensation torque input [Nm]";
                        else                    , ylabel = "Compensation input [N] [Nm]"; end
                    elseif contains(tstr, 'nominal_input')
                        if contains(tstr, '2:4'), ylabel = "Nominal torque input [Nm]";
                        else                    , ylabel = "Nominal input [N] [Nm]"; end
                    else
                        if contains(tstr, '2:4'), ylabel = "Torque input [Nm]";
                        else                    , ylabel = "Input [N] [Nm]"; end
                    end
                    tmp = "";
                    att = "";
                else
                    tmp = settings.attribute;
                    att = select_attribute(tstr, tmp);
                end
        end
        logger.plot({settings.agent_id, tstr, att}, ...
            'fig_num',i, 'color',fcolor, "phase",settings.phase, ...
            'FontSize',settings.fontsize, 'Linewidth',settings.linewidth)

        fig = gcf;
        ax = gca;

        chars = string(split(att, ""));
        chars(chars == "") = [];
        switch tstr
            case "p1-p2"
                set(ax.XLabel, 'String', xlabel, 'Interpreter','latex')
                set(ax.YLabel, 'String', ylabel, 'Interpreter','latex')
                est_data = logger.data(1,"p","e","phase",settings.phase);
                ref_data = logger.data(1,"p","r","phase",settings.phase);
                data = [est_data;ref_data];
                xlim([min(data(:,1)) max(data(:,1))])
                ylim([min(data(:,2)) max(data(:,2))])
            case "p1-p2-p3"
                set(ax.XLabel, 'String', xlabel, 'Interpreter','latex')
                set(ax.YLabel, 'String', ylabel, 'Interpreter','latex')
                set(ax.ZLabel, 'String', zlabel, 'Interpreter','latex')
                est_data = logger.data(1,"p","e","phase",settings.phase);
                ref_data = logger.data(1,"p","r","phase",settings.phase);
                data = [est_data;ref_data];
                xlim([min(data(:,1)) max(data(:,1))])
                ylim([min(data(:,2)) max(data(:,2))])
                zlim([min(data(:,3)) max(data(:,3))])
            case "inner_input1:4"
                set(ax.YLabel, 'String', ylabel, 'Interpreter','latex')
                plegend = set_legend(tstr, chars);
                set(ax.Legend, 'String', plegend, 'Interpreter','latex');
                data = logger.data(1,tstr,"","phase",settings.phase);
                y_min=0; y_max=0;
                for j=1:size(data,2)
                    if y_min>min(data(:,j)), y_min=min(data(:,j)); end
                    if y_max<max(data(:,j)), y_max=max(data(:,j)); end
                end
                ylim([y_min y_max])
            case "p"
                set(ax.YLabel, 'String', ylabel, 'Interpreter','latex')
                plegend = set_legend(tstr, chars);
                set(ax.Legend, 'String', plegend, 'Interpreter','latex');
                est_data = logger.data(1,tstr,"e","phase",settings.phase);
                ref_data = logger.data(1,tstr,"r","phase",settings.phase);
                data = [est_data;ref_data];
                y_min=0; y_max=0;
                for j=1:size(est_data,2)
                    if y_min>min(data(:,j)), y_min=min(data(:,j)); end
                    if y_max<max(data(:,j)), y_max=max(data(:,j)); end
                end
                ylim([y_min y_max])

            case "controller.result.xd1:3"
                set(ax.YLabel, 'String', ylabel, 'Interpreter','latex')
                cr = logger.Data.agent(settings.agent_id).controller.result;
                N  = numel(cr);
                xd = zeros(N,3);
                for k = 1:N
                    xd(k,:) = cr{k}.xd(1:3).';
                end
                t = logger.Data.t(1:N);
                plot(ax, t, xd(:,1), t, xd(:,2), t, xd(:,3), 'LineWidth', settings.linewidth)
                grid(ax,'on')
                legend(ax, {'$x_d$','$y_d$','$z_d$'}, 'Interpreter','latex', 'FontSize', settings.legend_fontsize)
                ylim(ax, [-1.5 1.5])

            otherwise
                if contains(tstr, '2:4')
                    set(ax.YLabel, 'String', ylabel, 'Interpreter','latex')
                    plegend = set_legend(tstr, chars);
                    set(ax.Legend, 'String', plegend, 'Interpreter','latex');
                    h = findobj(ax, 'Type', 'line');
                    set(h(1), 'Color', [0.4940, 0.1840, 0.5560])
                    set(h(2), 'Color', [0.9290, 0.6940, 0.1250])
                    set(h(3), 'Color', [0.8500, 0.3250, 0.0980])
                    data = logger.data(1,tstr,"e","phase",settings.phase);
                    y_min=0; y_max=0;
                    for j=1:size(data,2)
                        if y_min>min(data(:,j)), y_min=min(data(:,j)); end
                        if y_max<max(data(:,j)), y_max=max(data(:,j)); end
                    end
                    if y_min==y_max, y_max=y_max+1; end
                    ylim([y_min y_max])
                else
                    set(ax.YLabel, 'String', ylabel, 'Interpreter','latex')
                    plegend = set_legend(tstr, chars);
                    set(ax.Legend, 'String', plegend, 'Interpreter','latex');
                    if att == ""
                        data = logger.data(1,tstr,"","phase",settings.phase);
                    else
                        data = logger.data(1,tstr,att,"phase",settings.phase);
                    end
                    y_min=0; y_max=0;
                    for j=1:size(data,2)
                        if y_min>min(data(:,j)), y_min=min(data(:,j)); end
                        if y_max<max(data(:,j)), y_max=max(data(:,j)); end
                    end
                    if y_min==y_max, y_max=y_max+0.000000001; end
                    ylim([y_min y_max])
                end
        end
    end

    % ========== 共通フッター（元のまま：タイトル・凡例位置・保存） ==========
    if ftitle == 0
        set(ax.Title, 'String', [])
    end
    set(ax.Legend, 'Location','southeast', 'FontSize',settings.legend_fontsize);

    if ~exist('plot/fig', 'dir')
        mkdir('plot/fig')
    end
    figname_att = erase(char(tstr),':');
    start_idx = strfind(figname_att, 'result.');
    if ~isempty(start_idx)
        end_of_match = start_idx + length('result.');
        figname_att = figname_att(end_of_match:end);
    end
    if isfield(settings, 'savename') && (ischar(settings.savename) || isstring(settings.savename))
        if fsave==1,    filename_cell = {settings.savefolder, '\', settings.savename, '_', figname_att, '.fig'};
        elseif fsave==2,filename_cell = {settings.savefolder, '\', settings.savename, '_', figname_att, '.png'};
        elseif fsave==3,filename_cell = {settings.savefolder, '\', settings.savename, '_', figname_att, '.jpg'};
        elseif fsave==4,filename_cell = {settings.savefolder, '\', settings.savename, '_', figname_att, '.pdf'};
        elseif fsave==5,filename_cell = {settings.savefolder, '\', settings.savename, '_', figname_att, '.eps'};
        else,           filename_cell = {""};
        end
        string_cell     = cellfun(@string, filename_cell, 'UniformOutput', false);
        string_array    = [string_cell{:}];
        str             = strjoin(string_array,'');
        if fsave==1
            savefig(str);
        elseif fsave==2 || fsave==3 || fsave==4 || fsave==5
            exportgraphics(fig, str);
        end
    else
        if fsave==1,    filename_cell = {settings.savefolder, '\', erase(filename, '.mat'), '_', figname_att, '.fig'};
        elseif fsave==2,filename_cell = {settings.savefolder, '\', erase(filename, '.mat'), '_', figname_att, '.png'};
        elseif fsave==3,filename_cell = {settings.savefolder, '\', erase(filename, '.mat'), '_', figname_att, '.jpg'};
        elseif fsave==4,filename_cell = {settings.savefolder, '\', erase(filename, '.mat'), '_', figname_att, '.pdf'};
        elseif fsave==5,filename_cell = {settings.savefolder, '\', erase(filename, '.mat'), '_', figname_att, '.eps'};
        else,           filename_cell = {""};
        end
        string_cell     = cellfun(@string, filename_cell, 'UniformOutput', false);
        string_array    = [string_cell{:}];
        str             = strjoin(string_array,'');
        if fsave==1
            savefig(str);
        elseif fsave==2 || fsave==3 || fsave==4 || fsave==5
            exportgraphics(fig, str);
        end
    end
end
disp_rmse(logger,settings.phase)
% disp_bode(logger, "f", 1) 
% disp_fft(logger, "estimator.result.state.pL", "e", settings.phase, settings.legend_fontsize)
% disp_fft(logger, "input", "", settings.phase, settings.legend_fontsize)
disp_fft_time(logger, "estimator.result.state.pL", "e", [20 30], settings.legend_fontsize)   % 20秒から30秒までのデータでFFT
disp_fft_time(logger, "input", "", [20 30], settings.legend_fontsize)
%% Local functions

function t = get_phase_time(logger, phase, N)
% LOGGERクラス内部(C_Logger_Plot.get_data_range)と同じロジックで
% phaseに対応する正しい時刻範囲を取得する
if nargin < 3
    N = [];
end

if isempty(phase)
    dataRange = 1:length(logger.Data.t);
else
    phaseString = char(phase);
    ids = contains(string(char(logger.Data.phase)), string(phaseString(:)));
    dataRange = 4 + find(ids(5:end), 1):4 + find(ids(5:end), 1, 'last'); % 空回しの4つ分を除く
end

t = logger.Data.t(dataRange);
t = t(:);

% 念のため、取得したデータ数と一致するか確認
if ~isempty(N) && numel(t) ~= N
    warning('get_phase_time:mismatch', ...
        'phase="%s"から計算した時刻の数(%d)がデータ数(%d)と一致しません。先頭からの切り出しにフォールバックします。', ...
        string(phase), numel(t), N);
    t = logger.Data.t(1:N);
end
end

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
    {'est.', 'sen.', 'ref.', 'true'});
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
    otherwise
        for i=1:legend_num, legend{i} = string(i); end % 例外が入ってきたら適当に入れる
        if contains(target, 'delta_input') % MEC用
            if contains(target, '2:4')
                legend{1} = "$\Delta u_{roll}$";
                legend{2} = "$\Delta u_{pitch}$";
                legend{3} = "$\Delta u_{yaw}$";
            else
                legend{1} = "$\Delta u_{thrust}$";
                legend{2} = "$\Delta u_{roll}$";
                legend{3} = "$\Delta u_{pitch}$";
                legend{4} = "$\Delta u_{yaw}$";
            end
        elseif contains(target, 'nominal_input') % MECのノミナル入力用
            if contains(target, '2:4')
                legend{1} = "$u_{n,roll}$";
                legend{2} = "$u_{n,pitch}$";
                legend{3} = "$u_{n,yaw}$";
            else
                legend{1} = "$u_{n,thrust}$";
                legend{2} = "$u_{n,roll}$";
                legend{3} = "$u_{n,pitch}$";
                legend{4} = "$u_{n,yaw}$";
            end
        end
end
end


function disp_rmse(logger, phase)
p_est = logger.data(1,"estimator.result.state.pL","e","phase",phase);
p_ref = logger.data(1,"p","r","phase",phase);
N = min(size(p_est,1), size(p_ref,1));
p_est = p_est(1:N,:);
p_ref = p_ref(1:N,:);
diff = p_est - p_ref;
RMSE_x = sqrt(mean(diff(:,1).^2));
RMSE_y = sqrt(mean(diff(:,2).^2));
RMSE_z = sqrt(mean(diff(:,3).^2));
fprintf('\n===== Position RMSE (total time) =====\n');
fprintf(' RMSE_x = %.6f [m]\n', RMSE_x);
fprintf(' RMSE_y = %.6f [m]\n', RMSE_y);
fprintf(' RMSE_z = %.6f [m]\n', RMSE_z);
fprintf('=====================================\n\n');
end


function disp_bode(logger, phase, method)
arguments
    logger
    phase
    method = 1
end

Fs = 1/0.025;
dt = 1/Fs;
n_segments = 4;

u   = logger.data(1, "controller.result.tmp", "", "phase", phase);
pL  = logger.data(1, "estimator.result.state.pL", "e", "phase", phase);
pLq = logger.data(1, "estimator.result.state.q", "e", "phase", phase);

for i = 1:4
    if i == 1
        ci = 1; co = 3; use_pLq = false; nm = 'z';
    elseif i == 2
        ci = 3; co = 1; use_pLq = false; nm = 'x';
    elseif i == 3
        ci = 2; co = 2; use_pLq = false; nm = 'y';
    else
        ci = 4; co = 3; use_pLq = true;  nm = 'yaw';
    end

    if use_pLq
        y_out = pLq(:,co);
    else
        y_out = pL(:,co);
    end

    N = length(y_out);
    u_n_minus_1 = u(1:N-1, ci);
    y_n         = y_out(2:N);

    Nd = length(y_n);
    wl = floor(Nd / n_segments * 2);
    wl = 2^floor(log2(wl));
    window = hann(wl);
    noverlap = round(wl/2);
    nfft = wl*2;

    switch method
        case 1
            [h,f] = tfestimate(u_n_minus_1, y_n, window, noverlap, nfft, Fs);
            omega = 2*pi*f;
        case 2
            data_id = iddata(y_n, u_n_minus_1, dt);
            g = etfe(data_id);
            [mag, ph, w] = bode(g);
            h = squeeze(mag) .* exp(1j*deg2rad(squeeze(ph)));
            omega = squeeze(w);
        case 3
            data_id = iddata(y_n, u_n_minus_1, dt);
            g = spa(data_id);
            [mag, ph, w] = bode(g);
            h = squeeze(mag) .* exp(1j*deg2rad(squeeze(ph)));
            omega = squeeze(w);
        case 4
            [Pyx, f] = cpsd(y_n, u_n_minus_1, window, noverlap, nfft, Fs);
            [Pxx, ~] = pwelch(u_n_minus_1, window, noverlap, nfft, Fs);
            h = Pyx ./ Pxx;
            omega = 2*pi*f;
        case 5
            data_id = iddata(y_n, u_n_minus_1, dt);
            sys = ssest(data_id, 4);
            [mag, ph, w] = bode(sys);
            h = squeeze(mag) .* exp(1j*deg2rad(squeeze(ph)));
            omega = squeeze(w);
        case 6
            data_id = iddata(y_n, u_n_minus_1, dt);
            sys = tfest(data_id, 2, 2);
            [mag, ph, w] = bode(sys);
            h = squeeze(mag) .* exp(1j*deg2rad(squeeze(ph)));
            omega = squeeze(w);
    end

    [cxy, fc] = mscohere(u_n_minus_1, y_n, window, noverlap, nfft, Fs);
    omega_c = 2*pi*fc;

    omega_pos = omega(omega > 0);
    dec_min = floor(log10(omega_pos(1)));
    dec_max = ceil(log10(omega_pos(end)));
    xticks_dec = 10.^(dec_min:dec_max);

    figure('Color','w', 'Position', [100 100 900 500]);
    t = tiledlayout(2,1, 'TileSpacing','compact', 'Padding','compact');
    title(t, sprintf('%s (u(n) \\rightarrow y(n+1)), phase=%s, wl=%d, method=%d', nm, phase, wl, method), 'FontSize', 17);

    nexttile;
    semilogx(omega, 20*log10(abs(h)), 'LineWidth', 1.5);
    grid on; box on;
    ylabel('Gain [dB]', 'FontSize', 15);
    set(gca, 'FontSize', 13, 'XTick', xticks_dec);
    xlim([omega_pos(1) omega(end)]);

    nexttile;
    semilogx(omega, unwrap(angle(h))*180/pi, 'LineWidth', 1.5);
    grid on; box on;
    ylabel('Phase [deg]', 'FontSize', 15);
    xlabel('\omega [rad/s]', 'FontSize', 15);
    set(gca, 'FontSize', 13, 'XTick', xticks_dec);
    xlim([omega_pos(1) omega(end)]);

    figure('Color','w', 'Position', [1050 100 900 300]);
    semilogx(omega_c, cxy, 'LineWidth', 1.5);
    grid on; box on;
    ylim([0 1]);
    xlabel('\omega [rad/s]', 'FontSize', 15);
    ylabel('Coherence', 'FontSize', 15);
    title(sprintf('%s coherence (wl=%d)', nm, wl), 'FontSize', 15);
    set(gca, 'FontSize', 13, 'XTick', xticks_dec);
    xlim([omega_pos(1) omega(end)]);
end
end


function disp_fft(logger, target, att, phase, legend_fontsize)
if nargin < 5
    legend_fontsize = 10;
end

data = logger.data(1, target, att, "phase", phase);

figure('Color','w', 'Position', [100 100 800 400]);
plot(abs(fft(data)), 'LineWidth', 1.5)

grid on; box on
xlabel('Sample index', 'FontSize', 13)
ylabel('$|FFT|$', 'Interpreter','latex', 'FontSize', 13)
if att == ""
    ttl = sprintf('FFT: %s, phase=%s', target, phase);
else
    ttl = sprintf('FFT: %s (%s), phase=%s', target, att, phase);
end
title(ttl, 'Interpreter','none')

nCols = size(data,2);
if nCols == 3
    labels = ["x","y","z"];
elseif nCols == 4 && contains(target, "input")
    labels = ["thrust","roll","pitch","yaw"];
elseif nCols == 3 && contains(target, "input")
    labels = ["roll","pitch","yaw"];
else
    labels = "ch" + string(1:nCols);
end
legend(labels, 'FontSize', legend_fontsize)
set(gca, 'FontSize', 12)
end

function disp_fft_time(logger, target, att, t_range, legend_fontsize)
if nargin < 5
    legend_fontsize = 10;
end

data = logger.data(1, target, att, "ranget", t_range);

figure('Color','w', 'Position', [100 100 800 400]);
plot(abs(fft(data)), 'LineWidth', 1.5)
grid on; box on
xlabel('Sample index', 'FontSize', 13)
ylabel('$|FFT|$', 'Interpreter','latex', 'FontSize', 13)

ttl_time = sprintf('t=[%.2f, %.2f]s', t_range(1), t_range(2));
if att == ""
    ttl = sprintf('FFT: %s, %s', target, ttl_time);
else
    ttl = sprintf('FFT: %s (%s), %s', target, att, ttl_time);
end
title(ttl, 'Interpreter','none')

nCols = size(data,2);
if nCols == 3
    labels = ["x","y","z"];
elseif nCols == 4 && contains(target, "input")
    labels = ["thrust","roll","pitch","yaw"];
elseif nCols == 3 && contains(target, "input")
    labels = ["roll","pitch","yaw"];
else
    labels = "ch" + string(1:nCols);
end
legend(labels, 'FontSize', legend_fontsize)
set(gca, 'FontSize', 12)
end
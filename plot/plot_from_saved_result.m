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
% fsave = 2;
% fsave = 5;
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
% settings.target = ["p", "input", "inner_input","p1-p2-p3"];
settings.target = ["p", "input", "inner_input", "p1-p2-p3","controller.result.mL"]; %質量推定用
% settings.target = ["p", "v", "q", "w","input", "controller.result.nominal_input", "controller.result.delta_input", "p1-p2", "p1-p2-p3"];
% settings.target = ["p", "q", "v", "w", "input", "controller.result.delta_input", "p1-p2-p3"];
% settings.target = ["controller.result.delta_input", "controller.result.delta_input2:4", "controller.result.nominal_input", "controller.result.nominal_input2:4"];
% settings.target = ["p", "controller.result.delta_input"];
% settings.target = ["p", "p1-p2"];
% settings.target = "input2:4";
% settings.target = "p1-p2";
% "controller.result.nominal_input","controller.result.delta_input"
% settings.target = ["input", "input2:4", "controller.result.nominal_input2:4", "controller.result.delta_input2:4"];
% プロットしたいグラフの情報                                        %
% p: position    q: angle    v: velocity    w: angular velocity     %
% input: controller input    inner_input1:4: transmitter input      %
% p1-p2: x-y 2D plot    p1-p2-p3: x-y-z 3D plot                     %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% settings.phase = "tfl";
settings.phase = "t";
settings.fontsize = 16;    % default=11 オススメ=18　
% settings.fontsize = 22;    % 報告書向け
% settings.fontsize = 24;    % スライド向け
settings.linewidth = 1.5;    % default=0.5 オススメ=1.5
settings.agent_id = 1;
settings.savefolder = 'plot\fig';  % default
% settings.savefolder = "\\192.168.100.209\ws2025\Work2025\YosukeKOSEKI\Drone results(Exp_data)\2025.10.21_DNNMEC triangle and saddle\6_fig\png";

settings.savename = '2_HLonly_triangle';
% settings.savename = '6_DNNMEC_triangle';
% settings.savename = '11_HLonly_saddle';
% settings.savename = '13_DNNMEC_saddle';
% settings.savename = '3_HLonly_P2P';
% settings.savename = '5_DNNMEC_P2P';

% settings.savename = 'sim_HL_only_Standard';
% settings.savename = 'sim_HL_only_m=0.7125';
% settings.savename = 'sim_HL_only_Ix=0.01';
% settings.savename = 'sim_HL_only_Iy=0.21';
% settings.savename = 'sim_HL_only_Iy=0.19_add';

% settings.savename = 'No1_m_Ix_Iy_DNNMEC';
% settings.savename = 'No1_m_Ix_Iy_HLonly';
% settings.savename = 'No2_m+_DNNMEC';
% settings.savename = 'No3_m-_DNNMEC';
% settings.savename = 'No4_Ix_DNNMEC';
% settings.savename = 'No5_Iy_DNNMEC';
% settings.savename = 'No6_Ix_Iy_DNNMEC';
% settings.savename = 'No6_Ix_Iy_HLonly';
% settings.savename = 'm=1.0_DNNMEC';

% settings.savename = 'Exp_data_for_learing_dt=2.5';

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
        otherwise
            if contains(settings.target(i), 'input') % "input"が入っていたら
                if contains(settings.target(i), 'delta_input') % MEC用
                    if contains(settings.target(i), '2:4'), ylabel = "Compensation torque input [Nm]";
                    else                                  , ylabel = "Compensation input [N] [Nm]"; end
                elseif contains(settings.target(i), 'nominal_input') % MECのノミナル入力用
                    if contains(settings.target(i), '2:4'), ylabel = "Nominal torque input [Nm]";
                    else                                  , ylabel = "Nominal input [N] [Nm]"; end
                else % 知らない"input"用
                    if contains(settings.target(i), '2:4'), ylabel = "Torque input [Nm]";
                    else                                  , ylabel = "Input [N] [Nm]"; end
                end
                tmp = "";
                att = "";
            else % 例外来たらこれ↓
                tmp = settings.attribute;
                att = select_attribute(settings.target(i), tmp);
            end
    end
    logger.plot({settings.agent_id, settings.target(i), att}, ...
        'fig_num',i, 'color',fcolor, "phase",settings.phase, ...
        'FontSize',settings.fontsize, 'Linewidth',settings.linewidth)

    fig = gcf;
    ax = gca;

    chars = string(split(att, ""));
    chars(chars == "") = [];
    switch settings.target(i)
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
            plegend = set_legend(settings.target(i), chars);
            set(ax.Legend, 'String', plegend, 'Interpreter','latex');
            data = logger.data(1,settings.target(i),"","phase",settings.phase);
            y_min=0;
            y_max=0;
            for j=1:size(data,2)
                if y_min>min(data(:,j)), y_min=min(data(:,j)); end
                if y_max<max(data(:,j)), y_max=max(data(:,j)); end
            end
            ylim([y_min y_max])
        case "p"
            set(ax.YLabel, 'String', ylabel, 'Interpreter','latex')
            plegend = set_legend(settings.target(i), chars);
            set(ax.Legend, 'String', plegend, 'Interpreter','latex');
            est_data = logger.data(1,settings.target(i),"e","phase",settings.phase);
            ref_data = logger.data(1,settings.target(i),"r","phase",settings.phase);
            data = [est_data;ref_data];
            y_min=0;
            y_max=0;
            for j=1:size(est_data,2)
                if y_min>min(data(:,j)), y_min=min(data(:,j)); end
                if y_max<max(data(:,j)), y_max=max(data(:,j)); end
            end
            ylim([y_min y_max])
        otherwise
            if contains(settings.target(i), '2:4') % target = "input2:4"用
                set(ax.YLabel, 'String', ylabel, 'Interpreter','latex')
                plegend = set_legend(settings.target(i), chars);
                set(ax.Legend, 'String', plegend, 'Interpreter','latex');
                h = findobj(ax, 'Type', 'line');
                set(h(1), 'Color', [0.4940, 0.1840, 0.5560]) % デフォルト紫
                set(h(2), 'Color', [0.9290, 0.6940, 0.1250]) % デフォルト黄色
                set(h(3), 'Color', [0.8500, 0.3250, 0.0980]) % デフォルト赤　　なぜか順番は逆
                data = logger.data(1,settings.target(i),"e","phase",settings.phase);
                y_min=0;
                y_max=0;
                for j=1:size(data,2)
                    if y_min>min(data(:,j)), y_min=min(data(:,j)); end
                    if y_max<max(data(:,j)), y_max=max(data(:,j)); end
                end
                if y_min==y_max, y_max=y_max+1; end
                ylim([y_min y_max])
            else
                set(ax.YLabel, 'String', ylabel, 'Interpreter','latex')
                plegend = set_legend(settings.target(i), chars);
                set(ax.Legend, 'String', plegend, 'Interpreter','latex');
                % data = logger.data(1,settings.target(i),"e","phase",settings.phase);
                if att == ""
                    data = logger.data(1,settings.target(i),"","phase",settings.phase);
                else
                    data = logger.data(1,settings.target(i),att,"phase",settings.phase);
                end

                y_min=0;
                y_max=0;
                for j=1:size(data,2)
                    if y_min>min(data(:,j)), y_min=min(data(:,j)); end
                    if y_max<max(data(:,j)), y_max=max(data(:,j)); end
                end
                if y_min==y_max, y_max=y_max+0.000000001; end
                ylim([y_min y_max])
            end
    end
    if ftitle == 0
        set(ax.Title, 'String', [])
    end
    set(ax.Legend, 'Location','southeast', 'FontSize',settings.fontsize-4);

    if ~exist('plot/fig', 'dir')
        mkdir('plot/fig')
    end
    figname_att = erase(char(settings.target(i)),':');
    start_idx = 0; % 初期化
    start_idx = strfind(figname_att, 'result.');
    if ~isempty(start_idx)
        % target="~~.result.~~"があったらresult.を含めてその前を削除
        end_of_match = start_idx + length('result.');
        figname_att = figname_att(end_of_match:end);
    end
    if isfield(settings, 'savename') && (ischar(settings.savename) || isstring(settings.savename)) % settings.savenameの存在確認
        if fsave==1,    filename_cell = {settings.savefolder, '\', settings.savename, '_', figname_att, '.fig'};
        elseif fsave==2,filename_cell = {settings.savefolder, '\', settings.savename, '_', figname_att, '.png'};
        elseif fsave==3,filename_cell = {settings.savefolder, '\', settings.savename, '_', figname_att, '.jpg'};
        elseif fsave==4,filename_cell = {settings.savefolder, '\', settings.savename, '_', figname_att, '.pdf'};
        elseif fsave==5,filename_cell = {settings.savefolder, '\', settings.savename, '_', figname_att, '.eps'};
        else,           filename_cell = {""};
        end % ファイルのフルパスをcell配列化
        string_cell     = cellfun(@string, filename_cell, 'UniformOutput', false);
        string_array    = [string_cell{:}];
        str             = strjoin(string_array,''); % str型に変更
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

%% Frequency Analysis
% % % phase = "f";
% % % FS = settings.fontsize;
% % % LW = settings.linewidth;
% % % time = logger.data(0,'t',"", "phase",phase);
% % % % data_name = "estimator.result.state.p";
% % % % data_name = "estimator.result.state.q";
% % % % data_name = "estimator.result.state.v";
% % % data_name = "estimator.result.state.w";
% % % % data_name = "controller.result.input";
% % % data_name = "controller.result.input2:4";
% % % % data_name = "controller.result.delta_input";
% % % % data_name = "controller.result.delta_input2:4";
% % % data = logger.data(1,data_name,"", "phase",phase);
% % %
% % % Y = fft(data);
% % % lenY = size(Y,2);
% % % if logger.fExp==1
% % %     dt = logger.data(1,"sensor.result.dt","", "phase",phase);
% % %     dt_ave = sum(dt)/length(dt);
% % % elseif logger.fExp==0
% % %     dt_ave = 0.025; % Simデータの場合dt=25msで固定
% % % end
% % % % dt_ave=0.025;
% % % Fs = 1/dt_ave;
% % % L = length(data);
% % % f = Fs*(0:(L/2))/L; % 周波数軸の作成
% % % f = f(1:end-1);
% % % P = zeros(length(f),lenY);
% % % for i=1:lenY, P(:,i) = abs(Y(1:floor(L/2),i))./(L/2); end % 片側スペクトルの計算
% % % fig = figure;
% % % ax = gca;
% % % plot(f,P(:,1),LineWidth=LW)
% % % hold on
% % % for i=2:lenY, plot(f,P(:,i),LineWidth=LW); end
% % % set(ax.YLabel, 'String', '$|P_1(f)|$', 'Interpreter','latex', 'FontSize',FS)
% % %
% % % % plot(f,Y(:,1))
% % % % hold on
% % % % for i=2:size(Y,2)
% % % %     plot(f,Y(:,i))
% % % % end
% % % % set(ax.YLabel, 'String', 'Fourier Transform', 'Interpreter','latex', 'FontSize',FS)
% % %
% % % % plot(f,10*log10(P.^2))
% % % % set(ax.YLabel, 'String', '$20log_{10}P_1(f)$', 'Interpreter','latex', 'FontSize',FS)
% % %
% % % if data_name == "estimator.result.state.p", legend('$x$', '$y$', '$z$', 'Interpreter','latex');
% % % elseif data_name == "estimator.result.state.v", legend('$v_x$', '$v_y$', '$v_z$', 'Interpreter','latex');
% % % elseif data_name == "estimator.result.state.q", legend('$\theta_{roll}$', '$\theta_{pitch}$', '$\theta_{yaw}$', 'Interpreter','latex');
% % % elseif data_name == "estimator.result.state.w", legend('$\omega_{roll}$', '$\omega_{pitch}$', '$\omega_{yaw}$', 'Interpreter','latex');
% % % elseif data_name == "controller.result.input", legend('$u_{thrust}$','$u_{roll}$', '$u_{pitch}$', '$u_{yaw}$', 'Interpreter','latex');
% % % elseif data_name == "controller.result.input2:4"
% % %     legend('$u_{roll}$', '$u_{pitch}$', '$u_{yaw}$', 'Interpreter','latex');
% % %     h = findobj(gca, 'Type', 'line');
% % %     set(h(1), 'Color', [0.4940, 0.1840, 0.5560])
% % %     set(h(2), 'Color', [0.9290, 0.6940, 0.1250])
% % %     set(h(3), 'Color', [0.8500, 0.3250, 0.0980])
% % % elseif data_name == "controller.result.delta_input", legend('$\Delta u_{thrust}$','$\Delta u_{roll}$', '$\Delta u_{pitch}$', '$\Delta u_{yaw}$', 'Interpreter','latex');
% % % elseif data_name == "controller.result.delta_input2:4"
% % %     legend('$\Delta u_{roll}$', '$\Delta u_{pitch}$', '$\Delta u_{yaw}$', 'Interpreter','latex');
% % %     h = findobj(gca, 'Type', 'line');
% % %     set(h(1), 'Color', [0.4940, 0.1840, 0.5560])
% % %     set(h(2), 'Color', [0.9290, 0.6940, 0.1250])
% % %     set(h(3), 'Color', [0.8500, 0.3250, 0.0980])
% % % else, legend;
% % % end
% % % xlim([0 2])
% % % % ylim([0 0.5])
% % % set(ax.Legend, 'FontSize',FS-4)
% % % % title(ax, data_name, Fontsize=FS);
% % % set(ax.XLabel, 'String', 'Frequency $f$ [Hz]', 'Interpreter','latex', 'FontSize',FS)
% % % set(ax.XAxis, fontsize=FS-2)
% % % set(ax.YAxis, fontsize=FS-2)
% % % grid on;

%% TODO
% % % % % ↓このプロットの仕方にも対応できるようにしたい↓
% % % % logger.plot({{1, "p", "e"},{1, "controller.result.nominal_p", ""}}, "phase",settings.phase, "fig_num",100, "Linewidth",settings.linewidth, "Fontsize",settings.fontsize, 'color',settings.fcolor);
% % % % logger.plot({{1, "v", "e"},{1, "controller.result.nominal_v", ""}}, "phase",settings.phase, "fig_num",101, "Linewidth",settings.linewidth, "Fontsize",settings.fontsize, 'color',settings.fcolor);
% % % % logger.plot({{1, "q", "e"},{1, "controller.result.nominal_q", ""}}, "phase",settings.phase, "fig_num",102, "Linewidth",settings.linewidth, "Fontsize",settings.fontsize, 'color',settings.fcolor);
% % % % logger.plot({{1, "w", "e"},{1, "controller.result.nominal_w", ""}}, "phase",settings.phase, "fig_num",103, "Linewidth",settings.linewidth, "Fontsize",settings.fontsize, 'color',settings.fcolor);

%% Local functions
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


% function disp_rmse(logger, phase)
% target = ["p","v"];
% for i=1:length(target)
%     ref = logger.data(1,target(i),"r", "phase",phase);
%     data = logger.data(1,target(i),"e", "phase",phase);
%     RMSE = rmse(ref, data, 1);
%     fprintf('%s RMSE:\n', target(i))
%     disp(RMSE)
%     disp(sum(RMSE))
% end

% end


function disp_rmse(logger, phase)
% estimator と reference の position を取得（Nx3）
p_est = logger.data(1,"p","e","phase",phase);
p_ref = logger.data(1,"p","r","phase",phase);
% サイズチェック
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


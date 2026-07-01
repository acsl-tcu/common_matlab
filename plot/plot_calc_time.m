function plot_calc_time(logger, time_class, opts)
%plot_calc_time 計算時間を確認・プロットする関数
%   logger: LOGGERクラス
%   time_class: インスタンスし、値が既に格納されているTIMEクラス
arguments
    logger
    time_class = [];
    opts.FontSize = 16;
    opts.LineWidth = 1.0;
    opts.set_dt = 0.025;
    opts.fset_dt = true;
    opts.phase = "tfl";
end
FS = opts.FontSize;
LW = opts.LineWidth;

if isempty(time_class)
    if isfield(logger.Data, "calc_time")
        calc_time = logger.Data.calc_time;
    else, error(gen_msg("logger.Data.calc_time"));
    end

    if isfield(logger.Data, "target4loop")
        target4loop = logger.Data.target4loop;
    else, error(gen_msg("logger.Data.target4loop"));
    end

    if isfield(logger.Data, "target4do")
        target4do = logger.Data.target4do;
    else, error(gen_msg("logger.Data.target4do"));
    end
else
    calc_time = time_class.calc_time;
    target4loop = time_class.target4loop;
    target4do = time_class.target4do;
end


set_dt_ms = opts.set_dt*10^3; % [ms]に変換

data_offset = 4; %空回し対策用
phase_data = logger.Data.phase(data_offset+1:end);
phase = double(char(opts.phase));
mask = ismember(phase_data, phase);
is_start = [mask(1); diff(mask)==1];
is_end   = [diff(mask)==-1; mask(end)];
data_range = find(is_start)+data_offset : find(is_end)+data_offset;
t = logger.Data.t(data_range);
phase_data = phase_data(data_range);
% phase_plot = [];
% for ph = phase
%     switch ph
%         case 97 %'a'
%         case 116 %'t'
%         case 102 %'f'
%         case 108 %'l'
%     end
% end
total_dt = calc_time.total(data_range).*10^3;
loop_dt = [];
loop_legtxt = [];
for tag = target4loop(2:end)
    loop_dt = [loop_dt, calc_time.(tag)(data_range)];
end
loop_dt = loop_dt.*10^3; % [ms]に変換

agentN = size(calc_time.(target4do(1)),2);
do_dt = cell(1,agentN);
for N = 1:agentN
    for tag = target4do
        if tag=="logging", do_dt{N} = [do_dt{N}, calc_time.(tag)(data_range,1)];
        else, do_dt{N} = [do_dt{N}, calc_time.(tag)(data_range,N)];
        end
    end
    do_dt{N} = do_dt{N}.*10^3; % [ms]に変換
end

figNumber = 1112; %基本的に被らないようなユニークなのが良き
figMarginFromLeft = 50;
fig1 = figure(figNumber);
clf
fig1.Name = 'Calculation time in loop';
fig1.Position(1) = figMarginFromLeft;
ax = gca;
plot(ax, t,total_dt, "LineWidth",LW);
hold on; grid on; grid minor;
plot(ax, t,loop_dt, "LineWidth",LW);
% xline(5, "Label","takeoff")
% xline(10, "Label","landing")
if opts.fset_dt, yline(set_dt_ms, "--", "Set sampling time", "LineWidth",LW*0.75, "FontSize",FS*0.75); end
xlim([min(t), max(t)]);
legend(replace(target4loop,"_"," "), "Location","best")
xlabel("Time [s]")
ylabel("Calculation time [ms]")
set(ax.XAxis, fontsize=FS-2)
set(ax.YAxis, fontsize=FS-2)
set(ax.Legend, 'FontSize',FS-4);
hold off;


fig2 = figure(figNumber+1);
clf
fig2.Name = 'Calculation time in do_calculation';
fig2.Position(1) = fig1.Position(1)+fig1.Position(3)+figMarginFromLeft;
tile = tiledlayout(fig2, 'flow');
for N = 1:agentN
    ax = nexttile;
    plot(ax, t,total_dt, "LineWidth",LW);
    hold on; grid on; grid minor;
    plot(ax, t,do_dt{N}, "LineWidth",LW);
    if opts.fset_dt, yline(set_dt_ms, "--", "Set sampling time", "LineWidth",LW*0.75, "FontSize",FS*0.75); end
    xlim([min(t), max(t)]);
    xlabel("Time [s]")
    ylabel("Calculation time [ms]")
    ax.Title.String = "agent"+string(N);
    set(ax.XAxis, fontsize=FS-2)
    set(ax.YAxis, fontsize=FS-2)
    set(ax.Title, "FontSize",FS-4)
    hold off;
    if N==agentN
        legend([target4loop(1), replace(target4do,"_"," ")], "Location","best")
        set(ax.Legend, 'FontSize',FS-4);
    end
end
end

%% local function
function msg = gen_msg(dont_exist_var)
    msg = "TIMEクラスが引数になく、" + string(dont_exist_var) + "が存在しないため実行できません。";
end


function [ts te] = find_phase_change_time(time, phase_data, cha)
% time, phase_data共にopts.phaseで指定した部分を取り出した後のもの
    % find()
end

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
    opts.fset_dt = true; % 設定した計算時間plotフラグ
    opts.phase = "tfl";
    opts.fphase = true; % phase切替線plotフラグ
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

is_valid_phase(opts.phase);


% ------- data_range設定 -------
data_offset = 4; %空回し対策用
phase_data = logger.Data.phase(data_offset+1:end);
phase = double(char(opts.phase));
mask = ismember(phase_data, phase);
is_start = [mask(1); diff(mask)==1];
is_end   = [diff(mask)==-1; mask(end)];
data_range = find(is_start)+data_offset : find(is_end)+data_offset;
% ------ ------

t = logger.Data.t(data_range);
phase_data = phase_data(data_range);
phase_plot_func = gen_phase_plot_func(t, phase_data, phase);
total_dt = calc_time.total(data_range).*10^3; % [ms]に変換

set_dt_ms = opts.set_dt*10^3; % [ms]に変換
if max(total_dt)>set_dt_ms
    set_dt_labelpos = "top";
else
    set_dt_labelpos = "bottom";
end

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

% ------- 統計情報(平均・最小・最大・最頻値)の計算・表示 -------
stats_table = calc_calc_time_stats(target4loop, target4do, total_dt, loop_dt, do_dt, agentN);
disp("===== 計算時間の統計 [ms] =====")
disp(stats_table)
% ------ ------

figNumber = 1112; %基本的に被らないようなユニークなのが良き
figMarginFromLeft = 50;
fig1 = figure(figNumber); %target4loop用のfig
clf
fig1.Name = 'Calculation time in loop';
fig1.Position(1) = figMarginFromLeft;
ax = gca;
plot(ax, t,total_dt, "LineWidth",LW);
hold on; grid on; grid minor;
plot(ax, t,loop_dt, "LineWidth",LW);
if opts.fphase, phase_plot_func(); end
if opts.fset_dt, plot_set_dt(set_dt_ms, FS, LW, set_dt_labelpos); end
xlim([min(t), max(t)]);
legend(replace(target4loop,"_"," "), "Location","best")
xlabel("Time [s]")
ylabel("Calculation time [ms]")
set(ax.XAxis, fontsize=FS-2)
set(ax.YAxis, fontsize=FS-2)
set(ax.Legend, 'FontSize',FS-4);
hold off;


fig2 = figure(figNumber+1); %target4do用のfig
clf
fig2.Name = 'Calculation time in do_calculation';
fig2.Position(1) = fig1.Position(1)+fig1.Position(3)+figMarginFromLeft;
tile = tiledlayout(fig2, 'flow');
for N = 1:agentN
    ax = nexttile;
    plot(ax, t,total_dt, "LineWidth",LW);
    hold on; grid on; grid minor;
    plot(ax, t,do_dt{N}, "LineWidth",LW);
    if opts.fphase, phase_plot_func(); end
    if opts.fset_dt, plot_set_dt(set_dt_ms, FS, LW, set_dt_labelpos); end
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
end %plot_calc_time

%% local function
function stats_table = calc_calc_time_stats(target4loop, target4do, total_dt, loop_dt, do_dt, agentN)
    % target4loop, target4doの各要素について
    % 平均値・最小値・最大値・最頻値を計算し、tableにまとめて返す
    % (target4doはエージェント全体のデータを結合して統計を算出)

    tags = [target4loop, target4do];
    n = length(tags);
    Mean_ms   = zeros(n,1);
    Min_ms    = zeros(n,1);
    Max_ms    = zeros(n,1);
    Median_ms = zeros(n,1);
    Mode_ms   = zeros(n,1);

    % --- target4loopの統計 ---
    for i = 1:length(target4loop)
        if i == 1
            data = total_dt; % "total"
        else
            data = loop_dt(:, i-1); % target4loop(2:end)に対応
        end
        Mean_ms(i)   = mean(data);
        Min_ms(i)    = min(data);
        Max_ms(i)    = max(data);
        Median_ms(i) = median(data);
        Mode_ms(i)   = mode(data);
    end

    % --- target4doの統計(全エージェント分を結合) ---
    offset = length(target4loop);
    for j = 1:length(target4do)
        data = [];
        for N = 1:agentN
            data = [data; do_dt{N}(:,j)];
        end
        Mean_ms(offset+j)   = mean(data);
        Min_ms(offset+j)    = min(data);
        Max_ms(offset+j)    = max(data);
        Median_ms(offset+j) = median(data);
        Mode_ms(offset+j)   = mode(data);
    end

    Tag = tags(:);
    stats_table = table(Tag, Mean_ms, Min_ms, Max_ms, Median_ms, Mode_ms);
end %calc_calc_time_stats

function msg = gen_msg(dont_exist_var)
    msg = "TIMEクラスが引数になく、" + string(dont_exist_var) + "が存在しないため実行できません。";
end

function is_valid_phase(inputStr)
    % 許可されているベース文字列
    allowedBase = "atfl";
    
    % 入力を文字列型に変換
    inputStr = string(inputStr);
    
    % 判定処理
    % contains が false、または空文字の場合にエラー
    if ~contains(allowedBase, inputStr) || strlength(inputStr) == 0
        error("エラー: '%s' は許可されていない文字列です。'atfl' の部分文字列である必要があります。", inputStr);
    end
end

function plot_set_dt(dt_ms, FS, LW, LabelPosition)
    yline(dt_ms, "--", "Set sampling time", "LineWidth",LW*0.75, "FontSize",FS*0.75, "LabelVerticalAlignment",LabelPosition);
end

function phase_plot_func = gen_phase_plot_func(time, phase_data, phase_seq)
    % nameMap: 数値と名称の対応
    nameMap = containers.Map({97, 116, 102, 108}, {'approach', 'takeoff', 'flight', 'landing'});
    
    % 境界点（遷移地点）を特定
    % phase_dataにおいて値が切り替わったインデックスを探す
    diff_indices = find(diff(phase_data) ~= 0);
    
    % 遷移対象を格納するリスト
    boundary_info = {};
    
    for i = 1:length(diff_indices)
        idx = diff_indices(i);
        prev_val = phase_data(idx);
        next_val = phase_data(idx + 1);
        
        % 指定された phase_seq 内の遷移であるか確認
        if ismember(prev_val, phase_seq) && ismember(next_val, phase_seq)
            boundary_info{end+1} = struct(...
                'time', time(idx + 1), ...
                'left_label', nameMap(prev_val), ...
                'right_label', nameMap(next_val));
        end
    end
    
    % 無名関数として返す
    phase_plot_func = @() plot_boundary_lines(boundary_info);
end %gen_phase_plot_func

% 補助関数：境界に2本の線を引く
function plot_boundary_lines(boundary_info)
    hold on;
    for i = 1:length(boundary_info)
        t = boundary_info{i}.time;
        
        linespec = '--k';
        labelpos = 'top';
        % 左側のラベルを持つ線 (右側にオフセットして文字を配置)
        xl1 = xline(t, linespec, boundary_info{i}.left_label, ...
            'LabelVerticalAlignment', labelpos, ...
            'LabelHorizontalAlignment', 'left');
        xl1.LabelOrientation = 'aligned';
        
        % 右側のラベルを持つ線 (左側にオフセットして文字を配置)
        xl2 = xline(t, linespec, boundary_info{i}.right_label, ...
            'LabelVerticalAlignment', labelpos, ...
            'LabelHorizontalAlignment', 'right');
        xl2.LabelOrientation = 'aligned';
    end
end
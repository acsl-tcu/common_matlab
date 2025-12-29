classdef PLOTTER < handle
    %PLOTTER このクラスの概要をここに記述
    %   詳細説明をここに記述
    
    properties
        filepathes cell
        filenames cell
        logger cell
        mode string = "subfig"; % "subfig" or "onefig"を切り替える変数
        Num         % 登録しているlogger数（各所のループで使用する）
        save        % 保存関連の設定を入れる構造体
        settings    % プロット関連の設定を入れる構造体
        map         % プロット時の関係性をマッピングする構造体．全てcell配列で扱う．
        data        % プロット時に使用するデータを保存する構造体．colmun:時系列, row:変数
    end
    
    methods
        function obj = PLOTTER(filepathes)
            arguments
                filepathes cell
            end
            agentID = 1;
            % TODO: agentID=[1,2...] 複数機にも対応させる？？(2025/12/29)
            % obj.data.(join(clearn_target, "_", agentID),'')... <-この形にするのがよき？

            obj.filepathes = filepathes;
            [~, obj.filenames, ~] = cellfun(@fileparts, obj.filepathes, 'UniformOutput', false); % ファイル名の自動取得
            obj.Num = length(obj.filepathes);
            obj.logger = cell(obj.Num,1);
            for i = 1:obj.Num
                obj.logger{i} = LOGGER(obj.filepathes{i}); % LOGGERクラスとしてcell配列で登録
            end
            disp('Finish loading the data.');

            % % Default Initialize
            obj.save.savefolder = "plot\fig";
            obj.save.savename   = "dummy";
            obj.save.style      = "pdf";
            obj.save.fsave      = false;
            
            % ====== plot setting ======
            dummy_lgd = cell(obj.Num,1);
            dummy_colors= cell(obj.Num,1);
            dummy_alpha = ones(obj.Num,1);
            colormap = orderedcolors("gem"); % MATLABデフォルトカラーのRGB配列
            % colormap = [1,0,0; 0,1,0; 0,0,1; 0,1,1; 1,0,1; 1,1,0; 0,0,0; 1,1,1]; % "r,g,b,c,m,y,k,w"のRGB配列
            for i = 1:obj.Num
                dummy_lgd{i} = "dummy";
                dummy_colors{i} = colormap(i,:);
            end
            obj.settings.phase  = "f";
            obj.settings.FS     = 18;
            obj.settings.LW     = 1.5;
            obj.settings.t_range= [0,30];
            obj.settings.lgd    = dummy_lgd;
            obj.settings.lgd_pos= 1; %1,2,3 or 4
            obj.settings.lgd_loc= 'north';
            obj.settings.targets= ["p", "v", "q", "w", "input", "input2:4"];
            obj.settings.atts   = ["er", "er", "e", "e", "", ""];
            obj.settings.splited_atts = cell(1,length(obj.settings.atts));
            obj.settings.alpha  = dummy_alpha;
            obj.settings.colors = dummy_colors;
            for i=1:length(obj.settings.targets)
                c_array = char(obj.settings.atts(i));   % (char型) に変換することで分割
                obj.settings.splited_atts{i} = (string(c_array'))';      % stringに戻す
            end
            
            % ====== mapping ======
            obj.map.(obj.mode).targets = num2cell(obj.settings.targets);
            [clearn_targets, fault_flag] = create_clearn_target(obj.settings.targets);
            obj.map.(obj.mode).clearn_targets= num2cell(clearn_targets');
            obj.map.(obj.mode).fault_flag   = num2cell(fault_flag');
            allFigs = findall(0, 'Type', 'figure');
            if ~ isempty(allFigs),  lastFigNum = max([allFigs.Number]);
            else,                   lastFigNum = 1; end
            switch obj.mode
                case "subfig"
                    obj.map.(obj.mode).fignum = num2cell([lastFigNum : lastFigNum+length(obj.settings.targets)-1]);
                    obj.map.(obj.mode).label = subfig_label_mapping(clearn_targets);
                case "onefig"
                    obj.map.(obj.mode).fignum = num2cell([lastFigNum : lastFigNum+obj.Num*length(obj.settings.targets)-1]);
                    obj.map.(obj.mode).label = onefig_label_mapping(clearn_targets);
            end
            fprintf('Finish initializing\n\n')
        end
        
        function set_save(obj,opt)
            %set_save 保存関連を設定するメソッド
            arguments
                obj 
                opt.savefolder  string
                opt.savename    string
                opt.savestyle   string % 出力ファイル形式 ("jpg", "png", "pdf", "eps")
                opt.fsave % true or false
            end
            if isfield(opt,"savefolder"),   obj.save.savefolder = opt.savefolder; end
            if isfield(opt,"savename"),     obj.save.savename   = opt.savename; end
            if isfield(opt,"savestyle"),    obj.save.style      = opt.savestyle; end
            if isfield(opt,"fsave"),        obj.save.fsave      = opt.fsave; end
        end

        function set_plot_settings(obj, opt)
            %set_plot_settings プロット関連を設定するメソッド
            arguments
                obj
                opt.phase       string
                opt.FontSize    double
                opt.LineWidth   double
                opt.TimeRange   (1,2) double
                opt.LegendNames cell
                opt.lgd_pos     double
                opt.lgd_loc     char
                opt.targets     (1,:) string
                opt.atts        (1,:) string
                opt.alpha       (:,1) double
            end
            if isfield(opt,"phase"),        obj.settings.phase      = opt.phase; end
            if isfield(opt,"FontSize"),     obj.settings.FS         = opt.FontSize; end
            if isfield(opt,"LineWidth"),    obj.settings.LW         = opt.LineWidth; end
            if isfield(opt,"TimeRange"),    obj.settings.range      = opt.TimeRange; end
            if isfield(opt,"LegendNames"),  obj.settings.lgd        = opt.LegendNames; end
            if isfield(opt,"lgd_pos"),      obj.settings.lgd_pos    = opt.lgd_pos; end
            if isfield(opt,"lgd_loc"),      obj.settings.lgd_loc    = opt.lgd_loc; end
            if isfield(opt,"targets"),      obj.settings.targets    = opt.targets; end
            if isfield(opt,"atts"),         obj.settings.atts       = opt.atts; end
            if isfield(opt,"alpha"),        obj.settings.alpha      = opt.alpha; end
        end

        function set_map(obj, mode, targets, atts)
            %set_map プロット時のlabel, target, attのマッピングを行うメソッド
            % TODO: cell配列化したのに対応させる．2025/12/29 13:00
            arguments
                obj
                mode    string % "subfig" or "onefig"
                targets (1,:) string
                atts    (1,:) string
            end
            if ~(mode=="subfig" || mode=="onefig"), error('mode指定値が間違っています．入力値:%s', mode); end
            if length(targets) ~= length(atts), error('targetsとattのサイズが異なります．targets,atts = %d, %d', length(targets), length(atts)); end

            [common, idx, ~] = intersect(obj.settings.targets, targets); % 既に存在しているものをチェック
            targets(idx) = [];
            fprintf('\n既に存在しているtargetを除外．対象: %s', char(common));
            % TODO: targetとattの両方が一致している場合に除外．
            % attは異なる場合、異なっている対象となるattに関する情報のみ追加．

            obj.settings.targets    = targets;
            obj.map.(mode).targets  = targets;
            [clearn_target, fault_flag] = create_clearn_target(targets);
            obj.map.(mode).clearn_target= clearn_target;
            obj.map.(mode).fault_flag   = [obj.map.(mode).fault_flag, fault_flag];
            switch mode
                case "subfig"
                    obj.map.(mode).label = subplot_label_mapping(clearn_target);
                case "onefig"
                    obj.map.(mode).label = oneplot_label_mapping(clearn_target);
            end

        end

        function extract_time(obj, timerange)
            %extract_time 時間データの取り出し
            arguments
                obj 
                timerange (1,2) double = obj.settings.t_range;
            end
            all_time= cell(obj.Num,1);
            time    = cell(obj.Num,1);
            idx     = cell(obj.Num,1);
            for i = 1:obj.Num
                tmp = obj.logger{i}.data(0,"t","","phase",obj.settings.phase);
                all_time{i} = tmp - tmp(1);
                start_idx   = find(all_time{i} >= timerange(1), 1, 'first');
                last_idx    = find(all_time{i} <= timerange(2), 1, 'last');
                
                time{i} = all_time{i}(start_idx:last_idx);
                if isempty(time{i}), error('時間設定幅 [%s]が大きすぎます\n使用可能範囲: [%.4f, %.4f]', num2str(timerange), all_time{i}(1), all_time{i}(end)); end
                idx{i} = [start_idx, last_idx];
                if time{i}(end) == all_time{i}(end), warning('時間設定幅 [%s]が大きすぎます\n使用可能範囲: [%.4f, %.4f]', num2str(timerange), all_time{i}(1), all_time{i}(end)); end
            end

            obj.data.all_time   = all_time; % 必要ない気もする
            obj.data.time       = time;
            obj.data.idx        = idx;
        end

        function extract_data(obj, agentID, targets, clearn_targets, splited_atts)
            %extract_data obj.data.(clearn_targets(i)).attに値を格納するメソッド
            arguments
                obj 
                agentID double = 1 % 複数機に対応させるには変更必要
                targets (1,:) string = obj.map.(obj.mode).targets;
                clearn_targets (1,:) cell = obj.map.(obj.mode).clearn_targets;
                splited_atts (1,:) cell = obj.settings.splited_atts;
            end
            for map_idx = 1:length(targets)
                if ~obj.map.(obj.mode).fault_flag{map_idx} % "p1"のように数字が入っているとlogger.dataメソッドが使えないのでそれを回避
                    for att = splited_atts{map_idx}
                        for N = 1:obj.Num
                            tmp = obj.logger{N}.data(agentID, targets(map_idx), att, "phase", obj.settings.phase);
                            if att == "",   obj.data.(clearn_targets{map_idx}){N} = tmp(obj.data.idx{N}(1) : obj.data.idx{N}(2),:);
                            else,           obj.data.(clearn_targets{map_idx}).(att){N} = tmp(obj.data.idx{N}(1) : obj.data.idx{N}(2),:); end
                        end
                    end
                else % "p1"のようなイレギュラーの場合
                    for i = 1:count(targets(map_idx), "-")+1
                    end
                end
            end % インデント深くてごめん
        end

        function onefig_plot(obj, map_idx)
            arguments
                obj 
                map_idx = 0 % map_idxを指定したらそこだけを実行する 
            end
            if map_idx ~= 0
                figure(obj.map.(obj.mode).fignum{map_idx});
                clf;

                xlabel = obj.map.(obj.mode).label.x{map_idx};
                ylabel = obj.map.(obj.mode).label.y{map_idx};
                FS = obj.settings.FS;
                target = obj.map.(obj.mode).clearn_targets{map_idx};
                splited_atts = obj.settings.splited_atts{map_idx};
                if length(splited_atts)>=2 && any(splited_atts=="r")
                    isR = (atts=="r"); % "r"の要素を抽出
                    splited_atts = [splited_atts(isR), splited_atts(~isR)]; % 先頭が"r"になるように並び替え
                end
                plegend = {};
                hold on
                if ~obj.map.(obj.mode).fault_flag{map_idx} || contains(obj.settings.targets{map_idx}, "-")
                    % TODO: ここクソほどめんどくさい
                else % "p1"のようなイレギュラーを検知
                    for N = 1:obj.Num
                        for att = splited_atts
                            if att=="r"
                                line = '--';
                                LW = obj.settings.LW*2/3;
                                color = "k";
                                plegend = [plegend, "Reference"];
                            else
                                if att=="s",line = ':';
                                else,       line = '-'; end
                                LW = obj.settings.LW;
                                color = obj.settings.color{N};
                            end
                            % % % xdata = obj.data.(target).(att){N};
                            % % % ydata = obj.data.(target).(att){N};
                            u = plot(xdata, ydata, 'LineWidth',LW, 'LineStyle',line, 'Color',color);
                        end
                    end
                end
            elseif map_idx == 0
            end
        end

        function subfig_plot(obj)
            if obj.mode ~= "subfig", error('modeが異なります．現在のモード: %s', obj.mode); end
            for map_idx = 1:length(obj.map.(obj.mode).fignum)
                if length(obj.map.(obj.mode).label.y{map_idx}) == 1 % length(ylabel)==1の時はonefigを使用
                    obj.onefig_plot(map_idx);
                else
                    figure(obj.map.(obj.mode).fignum{map_idx});
                    clf;
                    
                    ylabels = obj.map.(obj.mode).label.y{map_idx};
                    FS = obj.settings.FS;
                    target  = obj.map.(obj.mode).clearn_targets{map_idx};
                    subNum  = length(ylabels);
                    splited_atts = obj.settings.splited_atts{map_idx};
                    if length(splited_atts)>=2 && any(splited_atts=="r")
                        isR = (splited_atts=="r"); % "r"の要素を抽出
                        splited_atts = [splited_atts(isR), splited_atts(~isR)]; % 先頭が"r"になるように並び替え
                    end
                    for column = 1:subNum
                        subplot(subNum, 1, column);
                        plegend = {};
                        hold on;
                        for N = 1:obj.Num
                            xdata   = obj.data.time{N};
                            for att = splited_atts % attにrあり無しで全然違う･･･
                                if att=="r" && N==1 % referenceは最初だけプロットする
                                    line = '--';
                                    LW = obj.settings.LW*2/3;
                                    color = "k";
                                    plegend = [plegend, "Reference"];
                                    if att == "",   ydata = obj.data.(target){N}(:,column);
                                    else,           ydata = obj.data.(target).(att){N}(:,column); end
                                    u = plot(xdata, ydata, 'LineWidth',LW, 'LineStyle',line, 'Color',color);
                                elseif att=="s" || att=="e" || att=="p" || att==""
                                    if att=="s",line = ':';
                                    else,       line = '-'; end
                                    LW = obj.settings.LW;
                                    color = obj.settings.colors{N};
                                    if att == "",   ydata = obj.data.(target){N}(:,column);
                                    else,           ydata = obj.data.(target).(att){N}(:,column); end
                                    u = plot(xdata, ydata, 'LineWidth',LW, 'LineStyle',line, 'Color',color);
                                end
                            end
                            plegend = [plegend, obj.settings.lgd{N}];
                            u.Color = [u.Color(1:3), obj.settings.alpha(N)];
                        end
                        hold off;
                        ylabel(ylabels{column}, 'Interpreter','latex', 'FontSize',FS);
                        grid on;
                        grid minor;
                        set(gca, 'FontSize',FS-FS*0.1);

                        if column == obj.settings.lgd_pos, legend(plegend, 'FontSize',FS-FS*0.2, 'Location',obj.settings.lgd_loc); end % 凡例は指定されたグラフ番号にのみ表示
                        % TODO: ↑'outside'が入っている場合northで全グラフサイズを共通にしたい
                        if column == subNum, xlabel(obj.map.(obj.mode).label.x{map_idx}, 'FontSize',FS); end % xlabelは一番下のグラフのみ表示
                    end
                end
            end
        end
    end
end


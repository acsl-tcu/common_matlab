classdef C_Logger_Plot < handle
    % Plot helper for LOGGER.
    methods
        function plot(obj, logger, list, option)
            % list : setting for subplot
            % option : setting for all figure
            %
            % list consists of cell array
            %   each cell has agent id, plot target or state, and attribute
            % example {2,"p","es"}
            %         agent id = 2       : plot 2nd agent data
            %         plot target = "p"  : position
            %         attribute = "es"   : estimator and sensor data
            %
            % plot target allows following form
            %         p : position, q : attitude,
            %         v : velocity, w : angular velocity
            %         p1 : first element of p, p1:2 : first two elements
            %         p1-p2 : phase plot p1 vs p2
            %         p1-p2-p3 : 3D phase plot p1, p2, p3
            % attribute consists of
            %         s : sensor, e : estimator, r : reference,
            %         p : plant (only simulation)
            % option
            %         trange : time span
            %         fig_num : figure number
            %         row_col : row and column number of subplot
            %
            % usage plot({1,"p1:2:3","ser"},{2,"input",""},{1,"q","e"},
            %               {2,"p1-p2"},"time",[4 10], "fig_num",2,"row_col",[2 2],
            %                 "FontSize",18, "Linewidth",2)
            %       fig1 = agent1's p1,p3 data w.r.t sensor, estimator and reference
            %       fig2 = agent2's input
            %       fig3 = agent1's estimator q
            %       fig4 = agent2's phase plot
            %       In figure window 2, figures are aligned "[fig1, fig2; fig3, fig4]"
            %       order in one row. Each figure has time span [4 10].s
            arguments
                obj
                logger
            end

            arguments (Repeating)
                list
            end

            arguments
                option.time (1, 2) double = [0 logger.Data.t(logger.k)]
                option.fig_num {mustBeNumeric} = 1
                option.row_col (1, 2) {mustBeNumeric} = [ceil(length(list) / min(length(list), 3)) min(length(list), 3)]
                option.color {mustBeNumeric} = 1
                option.hold {mustBeNumeric} = 0
                option.FH = []
                option.ax = []
                option.xrange = []
                option.yrange = []
                option.zrange = []
                option.phase = []
                option.FontSize {mustBeNumeric} = 11;
                option.Linewidth {mustBeNumeric} = 0.5;
            end

            ranget = option.time; % time range
            fig_num = option.fig_num; % figure number
            frow = option.row_col(1); % subfigure row number
            fcol = option.row_col(2); % subfigure col number
            fcolor = option.color; % on/off flag for phase coloring
            fhold = option.hold; % on/off flag for holding (only active to last subfigure)

            dataRange = obj.get_data_range(logger, ranget, option.phase);
            t = logger.Data.t(dataRange); % time data
            if isempty(option.ax)
                figure(fig_num);
                ax = gca;
            else
                ax = option.ax;
            end
            switch frow
                case 1
                    yoffset = 0.05;
                case 2
                    yoffset = 0.1;
                otherwise
                    yoffset = 0;
            end

            for plotIndex = 1:length(list) % plotIndex : 図番号
                if isscalar(list)
                    spfi = ax;
                else
                    subplot(frow, fcol, plotIndex, ax);
                end
                plegend = [];
                if isscalar(list{plotIndex}{1})
                    plotList = {list{plotIndex}};
                else
                    plotList = list{plotIndex};
                end
                for listIndex = 1:length(plotList)
                    plotSetting = plotList{listIndex};
                    agentIndex = plotSetting{1}; % indices of variable drones. example : [1 2]
                    param = plotSetting{2}; % p,q,v,w, etc
                    attribute = plotSetting{3}; % e,s,r,p
                    customData = [];
                    if length(plotSetting) >= 4
                        customData = plotSetting{4};
                    end
                    if strcmp(attribute, ""); attribute = " "; end

                    for agentId = agentIndex

                        for attrIndex = 1:strlength(attribute)
                            ps = split(param, '-'); % separate by '-', each in {p q v w}
                            att = extract(attribute, attrIndex); % attribute {s e r p}

                            if isempty(customData)
                                switch length(ps)
                                    case 1 % 時間応答（時間を省略）
                                        tmpx = t;
                                        [tmpy, vrange] = logger.data(agentId, ps, att, "ranget", ranget, "phase", option.phase);
                                    case 2 % 縦横軸明記
                                        tmpx = logger.data(agentId, ps(1), att, "ranget", ranget, "phase", option.phase);
                                        tmpy = logger.data(agentId, ps(2), att, "ranget", ranget, "phase", option.phase);
                                    case 3 % ３次元プロット
                                        tmpx = logger.data(agentId, ps(1), att, "ranget", ranget, "phase", option.phase);
                                        tmpy = logger.data(agentId, ps(2), att, "ranget", ranget, "phase", option.phase);
                                        tmpz = logger.data(agentId, ps(3), att, "ranget", ranget, "phase", option.phase);
                                end
                            else
                                [tmpx, tmpy, tmpz] = obj.unpack_custom_data(customData, t, dataRange, length(ps));
                                vrange = [];
                            end

                            % plot
                            switch att % set line type
                                case 'r'
                                    lt = '--'; % dashed
                                case 's'
                                    lt = ':'; % dotted
                                otherwise
                                    lt = '-'; % line
                            end
                            if length(ps) == 3
                                plot3(ax, tmpx, tmpy, tmpz, LineStyle=lt, LineWidth=option.Linewidth);
                            else
                                plot(ax, tmpx, tmpy(:, :, 1), LineStyle=lt, LineWidth=option.Linewidth); % tmpy(1:size(tmpx,1),:,1)
                                if length(option.xrange) == 2
                                    xlim(ax, option.xrange);
                                else
                                    xlim(ax, [min(tmpx), max(tmpx)]);
                                end
                                ylim(ax, [min(tmpy, [], 'all'), max(tmpy, [], 'all') + 0.01]);
                            end

                            set(ax.XAxis, fontsize=option.FontSize-2)
                            set(ax.YAxis, fontsize=option.FontSize-2)
                            if length(ps)==3; set(ax.ZAxis, fontsize=option.FontSize-2); end

                            hold(ax, "on");
                            grid(ax, "on");

                            switch length(ps)
                                case 3
                                    yoffset = -1.2;
                            end

                            % set title label legend
                            if length(ps) == 1
                                ps = ["t", ps];
                                xlabel(ax, "Time [s]", Fontsize=option.FontSize);
                            else
                                xlabel(ax, ps(1), Fontsize=option.FontSize);
                            end

                            switch ps(2)
                                case "p"
                                    title(ax, strcat("Position p of agent", string(agentId)), Fontsize=option.FontSize);
                                case "q"
                                    title(ax, strcat("Attitude q of agent", string(agentId)), Fontsize=option.FontSize);
                                case "v"
                                    title(ax, strcat("Velocity v of agent", string(agentId)), Fontsize=option.FontSize);
                                case "w"
                                    title(ax, strcat("Angular velocity w of agent", string(agentId)), Fontsize=option.FontSize);
                                case "z"
                                    title(ax, strcat("Position error integration of agent", string(agentId)), Fontsize=option.FontSize);
                                case "input"
                                    title(ax, strcat("Input u of agent", string(agentId)), Fontsize=option.FontSize);
                                otherwise
                                    title(ax, ps(2), FontSize=option.FontSize);
                            end

                            if ps(1) ~= "t"
                                title(ax, strcat("phase plot : ", string(param)));

                                switch att
                                    case "s"
                                        plegend = [plegend, "sensor"];
                                    case "e"
                                        plegend = [plegend, "estimator"];
                                    case "r"
                                        plegend = [plegend, "reference"];
                                    case "p"
                                        plegend = [plegend, "plant"];
                                    otherwise
                                        plegend = [plegend, att];
                                end

                                daspect(ax, [1 1 1]);
                            else

                                if isempty(vrange)
                                    vrange = string(1:size(tmpy, 2));
                                else
                                    vrange = string(vrange);
                                end

                                plegend = [plegend, append(vrange, att)];
                            end

                            ylabel(ax, ps(2), Fontsize=option.FontSize);
                            if length(ps) == 3; zlabel(ax, ps(3), Fontsize=option.FontSize); end
                        end

                    end
                end
                legend(ax, plegend);

                if ~fhold
                    hold(ax, "off");
                end

                if fcolor
                    txt = {''};
                    tidM = size(logger.Data.t, 1); % max time index
                    if length([find(logger.Data.phase == 97, 1, 'last') + 1, find(logger.Data.phase == 116, 1, 'last')]) == 2
                        Square_coloring(logger.Data.t([min(tidM, find(logger.Data.phase == 97, 1, 'last') + 1), find(logger.Data.phase == 116, 1, 'last')]), [], [], [], ax); % take off phase
                        %                        txt = {txt{:},'{\color{yellow}■} :Take off phase'};
                        txt = {txt{:}, '{\color[rgb]{1.0,1.0,0.9}■} :Take off phase'};
                    end

                    if length([find(logger.Data.phase == 116, 1, 'last') + 1, find(logger.Data.phase == 102, 1, 'last')]) == 2
                        Square_coloring(logger.Data.t([min(tidM, find(logger.Data.phase ==  116, 1, 'last') + 1), find(logger.Data.phase == 102, 1, 'last')]), [0.9 1.0 1.0], [], [], ax); % flight phase
                        txt = {txt{:}, '{\color[rgb]{0.9,1.0,1.0}■} :Flight phase'};
                    end

                    if length([find(logger.Data.phase ==  102, 1, 'last') + 1, find(logger.Data.phase == 108, 1, 'last')]) == 2
                        Square_coloring(logger.Data.t([min(tidM, find(logger.Data.phase ==  102, 1, 'last') + 1), find(logger.Data.phase == 108, 1, 'last')]), [1.0 0.9 1.0], [], [], ax); % landing phase
                        txt = {txt{:}, '{\color[rgb]{1.0,0.9,1.0}■} :Landing phase'};
                    end

                    text(ax, ax.XLim(2) - (ax.XLim(2) - ax.XLim(1)) * 0.25, ax.YLim(2) + (ax.YLim(2) - ax.YLim(1)) * yoffset, txt);
                end
            end

        end
    end

    methods (Access = private)
        function dataRange = get_data_range(obj, logger, ranget, phase)
            if ranget(2) == 0
                ranget(2) = max(logger.Data.t);
            end
            ranget_max = min(ranget(2), max(logger.Data.t));
            ranget_min = max(ranget(1), min(logger.Data.t));
            dataRange = find((logger.Data.t - ranget_min) > 0, 1) - 1:find((logger.Data.t - ranget_max) >= 0, 1);
            if ~isempty(phase)
                phaseString = char(phase);
                ids = contains(string(char(logger.Data.phase)), string(phaseString(:)));
                dataRange = 4 + find(ids(5:end), 1):4 + find(ids(5:end), 1, 'last'); % 空回しの4つ分を除く
            end
        end

        function [tmpx, tmpy, tmpz] = unpack_custom_data(obj, customData, t, dataRange, axisCount)
            tmpx = [];
            tmpy = [];
            tmpz = [];
            if axisCount == 1
                tmpx = t;
                tmpy = obj.slice_custom(customData, dataRange);
            elseif axisCount == 2
                [tmpx, tmpy] = obj.extract_custom_axes(customData, dataRange, 2, t);
            else
                [tmpx, tmpy, tmpz] = obj.extract_custom_axes(customData, dataRange, 3, t);
            end
        end

        function sliced = slice_custom(obj, customData, dataRange)
            if isstruct(customData) && isfield(customData, "y")
                source = customData.y;
            else
                source = customData;
            end
            if size(source, 1) == numel(dataRange)
                sliced = source;
            elseif size(source, 1) >= max(dataRange)
                sliced = source(dataRange, :);
            else
                error("LOGGER:PlotCustomDataSize", "Custom data length does not match logger time length.");
            end
        end

        function varargout = extract_custom_axes(obj, customData, dataRange, axisCount, t)
            if isstruct(customData)
                if axisCount >= 2
                    x = customData.x;
                    y = customData.y;
                end
                if axisCount == 3
                    z = customData.z;
                end
            else
                x = customData(:, 1);
                y = customData(:, 2);
                if axisCount == 3
                    z = customData(:, 3);
                end
            end
            if isempty(x)
                x = t;
            end
            if size(x, 1) == numel(dataRange)
                x = x;
            elseif size(x, 1) >= max(dataRange)
                x = x(dataRange, :);
            else
                error("LOGGER:PlotCustomDataSize", "Custom data length does not match logger time length.");
            end
            if size(y, 1) == numel(dataRange)
                y = y;
            elseif size(y, 1) >= max(dataRange)
                y = y(dataRange, :);
            else
                error("LOGGER:PlotCustomDataSize", "Custom data length does not match logger time length.");
            end
            if axisCount == 3
                if size(z, 1) == numel(dataRange)
                    z = z;
                elseif size(z, 1) >= max(dataRange)
                    z = z(dataRange, :);
                else
                    error("LOGGER:PlotCustomDataSize", "Custom data length does not match logger time length.");
                end
                varargout = {x, y, z};
            else
                varargout = {x, y};
            end
        end
    end
end

classdef C_Logger_Query < handle
    % Data/query helper for LOGGER.
    methods
        function [data, vrange] = get_data(obj, logger, target, variable, attribute, option)
            % target : agent indices
            % variable : var name or path to var from agent
            %            or path from result if attribute is set.
            % attribute : "s","e","r","p","i"
            % option ranget : time range
            % Examples
            % time : data('t',[],[])
            % state : data(1,"p","e")                      : agent1's estimated position
            %         data(2,"state.xd","r")               : agent2's reference xd
            %         data(1,"sensor.result.state.q",[])   : agent1's measured attitude
            % input : data(1,[],"i") or data(1,"input",[]) : agent1's input data
            % items : data(0,"item_name",[])               : items which
            %               does not depend on agent requires first input 0
            % data(1,"p","r","ranget",[0,2])               : take the data in time span [0 2]
            arguments
                obj
                logger
                target
                variable
                attribute = "e"
                option.ranget (1, 2) double = [0 0]
                option.phase = []
            end
            if logger.k == 0
                data = [];
                vrange = [];
            else
                ranget = option.ranget;
                if option.ranget(2) == 0
                    option.ranget(2) = max(logger.Data.t);
                end
                ranget_max = min(option.ranget(2), max(logger.Data.t));
                ranget_min = max(option.ranget(1), min(logger.Data.t));
                data_range = find((logger.Data.t - ranget_min) > 0, 1) - 1:find((logger.Data.t - ranget_max) >= 0, 1);
                if ~isempty(option.phase)
                    phase = char(option.phase);
                    ids = contains(string(char(logger.Data.phase)), string(phase(:)));
                    data_range = 4 + find(ids(5:end), 1):4 + find(ids(5:end), 1, 'last'); % 空回しの4つ分を除く
                end
                if sum(strcmp(target, {'time', 't'}))
                    data = obj.get_data_org(logger, 0, 't', '', "data_range", data_range);
                elseif target == 0
                    data = obj.get_data_org(logger, 0, variable, attribute, "data_range", data_range);
                else
                    data = cell2mat(arrayfun(@(agentId) obj.get_data_org(logger, agentId, variable, attribute, "data_range", data_range), target, 'UniformOutput', false));
                end

                [~, vrange] = obj.full_var_name(variable, attribute);
            end
        end

        function [data, vrange] = get_data_org(obj, logger, agentIndex, variable, attribute, option)
            % agentIndex : agent index
            arguments
                obj
                logger
                agentIndex
                variable string = "p"
                attribute string = "e"
                option.data_range int16
            end

            [variable, vrange] = obj.full_var_name(variable, attribute);
            data_range = option.data_range;
            if isempty(data_range)
                data_range = [1];
            end
            if sum(strcmp(agentIndex, {'time', 't'})) % 時間軸データ
                data = logger.Data.t(data_range);
            elseif agentIndex == 0 % agentIndex=0 => obj.items のデータ
                variable = split(variable, '.'); % member毎に分割
                data = [logger.Data.(variable{1})];

                for varIndex = 2:length(variable)
                    data = [data.(variable{varIndex})];
                end

                data = data(data_range);
            else % agentに関するデータ
                variable = split(variable, '.'); % member毎に分割
                data = [logger.Data.agent(agentIndex)];

                switch variable
                    case "inner_input" % 横ベクトルの場合
                        data = [data.(variable)];
                        data = [data{1, data_range}];
                        data = reshape(data, [8, length(data) / 8])';
                    otherwise

                        for varIndex = 1:length(variable)
                            data = [data.(variable{varIndex})];

                            if iscell(data) % 時間方向はcell配列
                                data = [data{1, data_range}]';
                                data = obj.return_state_prop(variable(varIndex + 1:end), data);
                                break
                            end

                        end

                end

            end

            if ~isempty(vrange)
                data = obj.take_matrix(data, [], vrange);
            end

        end

        function data = return_state_prop(obj, variable, data)
            % function for get_data_org
            for varIndex = 1:length(variable)
                fieldNames = fieldnames(data); % フィールド名に数字を含む場合のケア

                if strcmp(variable(varIndex), 'state')
                    data = vertcat(data.(fieldNames{strcmp(fieldNames, variable(varIndex))}));

                    for row = 1:length(data)
                        %ndata(row, :, :) = data(row).(variable(varIndex + 1))(1:data(row).num_list(strcmp(data(row).list, variable(varIndex + 1))), :);
                        ndata(row, :, :) = data(row).(variable(varIndex + 1));
                    end

                    data = ndata;
                    break % WRN : stateから更に深い構造には対応していない
                else
                    for row = 1:length(data)
                        ndata(row, :, :) = data(row).(fieldNames{strcmp(fieldNames, variable(varIndex))});
                    end
                    data = ndata;
                end

            end

        end

        function [name, vrange] = full_var_name(obj, var, att)
            switch att
                case 's' % sensor
                    name = "sensor.result";
                case 'e' % estimator
                    name = "estimator.result";
                case 'r' % reference
                    name = "reference.result";
                case 'p' % plant
                    name = "plant.result";
                case 'i' %
                    name = "input";
                otherwise
                    name = "";
            end

            variable = regexprep(var, "[0-9:]", "");
            vrange = regexp(var, "[0-9:]", 'match');

            if ~isempty(vrange)
                vrange = eval(strjoin(vrange, '')); %#ok<EVLDIR>
            end
            switch variable
                case 'p'
                    name = strcat(name, ".state.p");
                case 'q'
                    name = strcat(name, ".state.q");
                case 'v'
                    name = strcat(name, ".state.v");
                case 'w'
                    name = strcat(name, ".state.w");
                case 'z'
                    name = strcat(name, ".state.z");
                case 'input'
                    name = "input";
                otherwise

                    if ~isempty(variable)

                        if ~contains(variable, "result") & name ~= "" % attribute が指定されている場合
                            name = strcat(name, '.', variable);
                        else
                            name = variable;
                        end

                    end

            end

        end

        function X = take_matrix(obj, x, row, col)
            if isempty(row)
                X = x(:, col);
            else
                X = x(row, col);
            end
        end
    end
end

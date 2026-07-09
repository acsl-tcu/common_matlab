classdef LOGGER < handle % handleクラスにしないとmethodの中で値を変えられない
    % データ保存用クラス
    % obj = LOGGER(target,row,items)
    % target : 保存対象の agent indices : example 2:4 : default 1:N
    % row : size(ts:dt:te,2)
    % items : [    "plant.state.p",    "model.state.p"  ...]
    % At every logging, LOGGER gathers the data listed in items from target
    % obj.Data.t : time
    % obj.Data.agent = {row, item_id, target_id}
    properties
        Data
        k; % time index for logging
        target % 保存対象の agent indices : example 2:4 : default 1:N
        items % 追加で保存するアイテム名
        item_num % 追加保存のアイテム数
        agent_items % result以外で追加保存するagent内の変数
        fExp
        overwrite_target = ["all"];
        displayHandler
        plotHandler
        queryHandler
        storageHandler
        replayHandler
        timeHandler = []; % TIMEクラス用
    end

    properties (Dependent)
        display_func % function handle to build display data
        display_on % print display data at logging
    end

    methods

        function obj = LOGGER(target, number, fExp, items, agent_items, option)
            % LOGGER(target,row,items)
            % target : ログを取る対象　example 1:3, usage agent(obj.target)
            % number : 確保するデータサイズ　length(ts:dt:te)
            % agent_items : default以外で保存するデータ ["inner_input"]
            % items  : agent 以外で保存するデータの名前
            %         以下のように名前と保存するものの対応が取れていなくても可
            %         LOGGER=LOGGER(~,~,"innerInput",~);
            %         LOGGER.logging(t,FH,agent,motive);
            % 【ログの呼び出し】
            % LOGGER.saveで保存したデータを呼び出してloggerとして登録することができる．
            % logger=LOGGER("Data/filename.mat")
            % logger=LOGGER("Data/dirname");
            arguments
                target
                number = []
                fExp = []
                items = []
                agent_items = []
                option.overwrite_target = []
            end
            obj.displayHandler = C_Logger_Display;
            obj.plotHandler = C_Logger_Plot;
            obj.queryHandler = C_Logger_Query;
            obj.storageHandler = C_Logger_Storage;
            obj.replayHandler = C_Logger_Replay;

            if isstring(target) || ischar(target) % save で保存されたデータを呼び出す場合
                obj.storageHandler.load_from_path(obj, target, "overwrite_target", option.overwrite_target);
            else
                obj.k = 0;
                obj.target = target;
                obj.Data.t = zeros(number, 1); % 時間
                obj.Data.phase = zeros(number, 1); % フライトフェーズ　a,t,f,l...
                obj.fExp = fExp;
                obj.items = items;
                obj.Data.display = cell(1, length(target));

                if ~isempty(items)

                    for i = items
                        obj.Data.(i) = {};
                    end

                end

                obj.agent_items = agent_items;
                obj.Data.agent = struct();
            end

        end

        function logging(obj, time, cha, agent, items)
            % logging(t,FH)
            % t : current time
            % FH : figure handle for keyboard input
            arguments
                obj
                time
                cha
                agent
            end
            arguments (Repeating)
                items
            end

            t = time.t;
            if isempty(cha)
                %                error("ACSL : FH is empty");
                cha = obj.Data.phase(obj.k);
            end
            % if t == 0 %初期値
            %   obj.k = 1;
            %   obj.Data.t(obj.k) = t;
            %   obj.Data.phase(obj.k) = cha;
            %   for n = obj.target
            %     obj.Data.agent(n).estimator.result{1}.state = state_copy(agent(n).estimator.result.state);
            %     obj.Data.agent(n).plant.result{1}.state = state_copy(agent(n).plant.state);
            %   end
            % else
            obj.k = obj.k + 1;
            obj.Data.t(obj.k) = t;
            obj.Data.phase(obj.k) = cha;

            for i = 1:length(obj.items)
                obj.Data.(obj.items(i)){obj.k} = items{i};
                % 注：サイズの固定されている数値データだけ保存可能
            end

            for n = 1:length(obj.target)
                N = obj.target(n);
                for i = 1:length(obj.agent_items) % sensor,estimator,reference以外のみ
                    str = strsplit(obj.agent_items(i), '.');
                    tmp = agent(N);
                    obj.Data.agent(n).(str{1}){obj.k} = tmp.(str{1});
                end

                obj.Data.agent(n).sensor.result{obj.k} = agent(N).sensor.result;
                obj.Data.agent(n).estimator.result{obj.k} = agent(N).estimator.result;
                obj.Data.agent(n).reference.result{obj.k} = agent(N).reference.result;
                obj.Data.agent(n).controller.result{obj.k} = agent(N).controller.result;

                if isfield(agent(N).sensor.result, "state")
                    obj.Data.agent(n).sensor.result{obj.k}.state = state_copy(agent(N).sensor.result.state);
                end

                if isfield(agent(N).estimator.result,'state')
                    obj.Data.agent(n).estimator.result{obj.k}.state = state_copy(agent(N).estimator.result.state);
                end
                if isfield(agent(N).reference.result,'state')
                    obj.Data.agent(n).reference.result{obj.k}.state = state_copy(agent(N).reference.result.state);
                end
                obj.Data.agent(n).input{obj.k} = agent(N).controller.result.input;

                if obj.fExp
                    obj.Data.agent(n).inner_input{obj.k} = agent(N).input_transform.result;
                else
                    obj.Data.agent(n).plant.result{obj.k}.state = state_copy(agent(N).plant.state);
                end

            end
            obj.displayHandler.capture_display(obj, time, agent);
            if obj.display_on
                obj.displayHandler.show_display(obj, "k", obj.k);
            end
            %end

        end

        function save(obj, name, opt)
            arguments
                obj
                name = []
                opt.range = 1:length(obj.Data.phase); %find(obj.Data.phase,1,'last');
                opt.separate = false;
            end
            if ~isempty(obj.timeHandler)
                obj.Data.calc_time = obj.timeHandler.calc_time;
                obj.Data.target4loop = obj.timeHandler.target4loop;
                obj.Data.target4do = obj.timeHandler.target4do;
            end
            obj.storageHandler.save_log(obj, name, opt);

        end

        function overwrite(obj, str, t, agent, n)
            % overwrite(str,t,agent,n)
            % agent(n).(str).result の情報をData情報で上書き
            obj.replayHandler.overwrite(obj, str, t, agent, n);
        end

        function [data, vrange] = data(obj, target, variable, attribute, option)
            arguments
                obj
                target
                variable
                attribute = "e"
                option.ranget (1, 2) double = [0 0]
                option.phase = []
            end
            if isempty(obj.queryHandler)
                obj.queryHandler = C_Logger_Query;
            end
            [data, vrange] = obj.queryHandler.get_data(obj, target, variable, attribute, "ranget", option.ranget, "phase", option.phase);
        end

        function [data, vrange] = data_org(obj, n, variable, attribute, option)
            arguments
                obj
                n
                variable string = "p"
                attribute string = "e"
                option.data_range int16
            end
            if isempty(obj.queryHandler)
                obj.queryHandler = C_Logger_Query;
            end
            [data, vrange] = obj.queryHandler.get_data_org(obj, n, variable, attribute, "data_range", option.data_range);

        end

        function data = return_state_prop(obj, variable, data)
            if isempty(obj.queryHandler)
                obj.queryHandler = C_Logger_Query;
            end
            data = obj.queryHandler.return_state_prop(variable, data);
        end

        function plot(obj, list, option)
            arguments
                obj
            end

            arguments (Repeating)
                list
            end

            arguments
                option.time (1, 2) double = [0 obj.Data.t(obj.k)]
                option.fig_num {mustBeNumeric} = 1
                option.row_col (1, 2) {mustBeNumeric} = [ceil(length(list) / min(length(list), 3)) min(length(list), 3)]
                option.color {mustBeNumeric} = 1
                option.hold {mustBeNumeric} = 0
                option.FH = [];
                option.ax = [];
                option.xrange = [];
                option.yrange = [];
                option.zrange = [];
                option.phase = [];
                option.FontSize {mustBeNumeric} = 11;
                option.Linewidth {mustBeNumeric} = 0.5;
            end
            obj.plotHandler.plot(obj, list{:}, ...
                "time", option.time, ...
                "fig_num", option.fig_num, ...
                "row_col", option.row_col, ...
                "color", option.color, ...
                "hold", option.hold, ...
                "FH", option.FH, ...
                "ax", option.ax, ...
                "xrange", option.xrange, ...
                "yrange", option.yrange, ...
                "zrange", option.zrange, ...
                "phase", option.phase, ...
                "FontSize", option.FontSize, ...
                "Linewidth", option.Linewidth);

        end

        function capture_display(obj, time, agent)
            arguments
                obj
                time
                agent
            end
            obj.displayHandler.capture_display(obj, time, agent);
        end

        function show_display(obj, option)
            arguments
                obj
                option.k (1, 1) double = obj.k
                option.target = []
                option.prefix string = ""
            end
            obj.displayHandler.show_display(obj, option);
        end

        function value = get.display_func(obj)
            if isempty(obj.displayHandler)
                obj.displayHandler = C_Logger_Display;
            end
            value = obj.displayHandler.displayFunc;
        end

        function set.display_func(obj, value)
            if isempty(obj.displayHandler)
                obj.displayHandler = C_Logger_Display;
            end
            obj.displayHandler.displayFunc = value;
        end

        function value = get.display_on(obj)
            if isempty(obj.displayHandler)
                obj.displayHandler = C_Logger_Display;
            end
            value = obj.displayHandler.displayOn;
        end

        function set.display_on(obj, value)
            if isempty(obj.displayHandler)
                obj.displayHandler = C_Logger_Display;
            end
            obj.displayHandler.displayOn = value;
        end

        function [name, vrange] = full_var_name(obj, var, att)
            if isempty(obj.queryHandler)
                obj.queryHandler = C_Logger_Query;
            end
            [name, vrange] = obj.queryHandler.full_var_name(var, att);
        end

        function X = mtake(obj, x, row, col)
            if isempty(obj.queryHandler)
                obj.queryHandler = C_Logger_Query;
            end
            X = obj.queryHandler.take_matrix(x, row, col);
        end

        function set_time_handler(obj, timeObj)
            obj.timeHandler = timeObj;
        end

    end

end

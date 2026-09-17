classdef C_Logger_Storage < handle
    % Save/load helper for LOGGER.
    methods
        function load_from_path(obj, logger, target, option)
            arguments
                obj
                logger
                target
                option.overwrite_target = []
            end
            if ~contains(target, ".mat") % separate で保存された場合

                if contains(target, "Data.mat")
                    target = erase(target, "/Data.mat");
                end

                tmp = load(target + "/Data.mat");
                fn = fieldnames(logger);

                for i = fn'
                    if isfield(tmp.log, i{1})
                        logger.(i{1}) = tmp.log.(i{1});
                    end
                end

                tmp = load(target + "/sensor.mat");
                logger.Data.agent.sensor = tmp.sensor;
                tmp = load(target + "/estimator.mat");
                logger.Data.agent.estimator = tmp.estimator;
                tmp = load(target + "/reference.mat");
                logger.Data.agent.reference = tmp.reference;
                tmp = load(target + "/input.mat");
                logger.Data.agent.input = tmp.input;
                tmp = load(target + "/controller.mat");
                logger.Data.agent.controller = tmp.controller;
                tmp = load(target + "/plant.mat");
                logger.Data.agent.plant = tmp.plant;
            else % 一つのファイルとして保存した場合
                tmp = load(target);
                log = tmp.log;
                fn = fieldnames(log);

                for i = fn'
                    if isfield(log, i{1})
                        logger.(i{1}) = log.(i{1});
                    end
                end

            end

            if ~isempty(option.overwrite_target)
                logger.overwrite_target = option.overwrite_target;
            end
        end

        function save_log(obj, logger, name, opt)
            % save log.Data keeping its structure as a file Data/Log(datetime).mat
            % retrieve it by logger = LOGGER("./Data/file.mat");
            if nargin < 4 || isempty(opt)
                opt = struct();
            end
            if ~isfield(opt, "range")
                opt.range = 1:length(logger.Data.phase); %find(obj.Data.phase,1,'last');
            end
            if ~isfield(opt, "separate")
                opt.separate = false;
            end

            drange = opt.range;

            if isempty(name)
                tmpname = strrep(strrep(strcat('Log(', datestr(datetime('now')), ')'), ':', '_'), ' ', '_');
            else
                tmpname = strrep(strrep(strcat('', name, '_Log(', datestr(datetime('now')), ')'), ':', '_'), ' ', '_');
            end

            dataRoot = obj.get_data_root();
            if opt.separate
                if logger.fExp == 1
                    dirname = fullfile(dataRoot, "Exp_data", tmpname);
                else
                    dirname = fullfile(dataRoot, "Sim_data", tmpname);
                end
                mkdir(dirname);
                filename = fullfile(dirname, "Data.mat");
                Data.t = logger.Data.t;
                Data.phase = logger.Data.phase;
                fn = fieldnames(logger);

                for i = fn'
                    if ~strcmp(i{1}, "Data") && ~strcmp(i{1}, "display_func") && ...
                            ~strcmp(i{1}, "displayHandler") && ~strcmp(i{1}, "plotHandler") && ...
                            ~strcmp(i{1}, "queryHandler") && ~strcmp(i{1}, "storageHandler") && ...
                            ~strcmp(i{1}, "replayHandler")
                        log.(i{1}) = logger.(i{1});
                    end

                end

                log.Data = Data;
                fn = fieldnames(logger.Data);

                for i = fn'

                    if ~strcmp(i{1}, "t") & ~strcmp(i{1}, "phase") & ~strcmp(i{1}, "agent")
                        log.Data.(i{1}) = logger.Data.(i{1});
                    end

                end

                save(filename, "log");
                sensor = logger.Data.agent.sensor;
                save(fullfile(dirname, "sensor.mat"), "sensor");
                estimator = logger.Data.agent.estimator;
                save(fullfile(dirname, "estimator.mat"), "estimator", "-v7.3");
                reference = logger.Data.agent.reference;
                save(fullfile(dirname, "reference.mat"), "reference");
                input = logger.Data.agent.input;
                save(fullfile(dirname, "input.mat"), "input");
                controller = logger.Data.agent.controller;
                save(fullfile(dirname, "controller.mat"), "controller");
                plant = logger.Data.agent.plant;
                save(fullfile(dirname, "plant.mat"), "plant");
            else
                filename = tmpname;
                if logger.fExp == 1
                    expDir = fullfile(dataRoot, "Exp_data");
                    if ~exist(expDir, "dir")
                        mkdir(expDir);
                    end
                    list = fullfile(expDir, filename + ".mat");
                else
                    simDir = fullfile(dataRoot, "Sim_data");
                    if ~exist(simDir, "dir")
                        mkdir(simDir);
                    end
                    list = fullfile(simDir, filename + ".mat");
                end
                log.Data = logger.Data;
                fn = fieldnames(logger);

                for i = fn'
                    if ~strcmp(i{1}, "Data") && ~strcmp(i{1}, "display_func") && ...
                            ~strcmp(i{1}, "displayHandler") && ~strcmp(i{1}, "plotHandler") && ...
                            ~strcmp(i{1}, "queryHandler") && ~strcmp(i{1}, "storageHandler") && ...
                            ~strcmp(i{1}, "replayHandler")
                        log.(i{1}) = logger.(i{1});
                    end

                end

                save(list, 'log');
            end

        end
    end

    methods (Access = private)
        function dataRoot = get_data_root(obj)
            dataRoot = "Data";
            if exist(dataRoot, "dir")
                return
            end
            loggerPath = which("LOGGER");
            if ~isempty(loggerPath)
                repoRoot = fileparts(fileparts(fileparts(loggerPath)));
                candidate = fullfile(repoRoot, "Data");
                if exist(candidate, "dir")
                    dataRoot = candidate;
                    return
                end
            end
        end
    end
end

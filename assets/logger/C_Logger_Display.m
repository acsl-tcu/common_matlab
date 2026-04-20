classdef C_Logger_Display < handle
    % Display helper for LOGGER: capture and show console output.
    properties
        displayFunc = [];
        displayOn = false;
    end

    methods
        function capture_display(obj, logger, time, agent)
            arguments
                obj
                logger
                time
                agent
            end
            if isempty(obj.displayFunc)
                return
            end
            if logger.k == 0
                return
            end
            if ~isfield(logger.Data, "display") || isempty(logger.Data.display)
                logger.Data.display = cell(1, 1);
            end
            logger.Data.display{logger.k} = obj.displayFunc(agent, time);
        end

        function show_display(obj, logger, option)
            arguments
                obj
                logger
                option.k (1, 1) double = logger.k
                option.target = []
                option.prefix string = ""
            end
            if ~isfield(logger.Data, "display") || isempty(logger.Data.display)
                return
            end
            val = logger.Data.display{option.k};
            if isstring(val) || ischar(val)
                msg = string(val);
            else
                msg = mat2str(val);
            end
            label = "";
            if option.prefix ~= ""
                label = option.prefix + " ";
            end
            disp(label + msg);
        end
    end
end

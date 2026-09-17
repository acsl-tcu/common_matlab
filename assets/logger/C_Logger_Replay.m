classdef C_Logger_Replay < handle
    % Replay/overwrite helper for LOGGER.
    methods
        function overwrite(obj, logger, str, t, agent, n)
            % overwrite(str,t,agent,n)
            % agent(n).(str).result の情報をData情報で上書き
            if sum(contains(logger.overwrite_target, str) + strcmp(logger.overwrite_target, "all")) > 0
                %tidx = find((logger.Data.t-t)>=0,1); % 現在時刻に最も近い過去のデータを参照
                [~, tidx] = min(abs(logger.Data.t - t)); % 現在時刻に最も近い過去のデータを参照

                switch str
                    case "sensor"
                        agent(n).sensor.result = logger.Data.agent(n).sensor.result{tidx};
                        agent(n).sensor.result.state = state_copy(logger.Data.agent(n).sensor.result{tidx}.state);
                    case "estimator"
                        agent(n).estimator.result = logger.Data.agent(n).estimator.result{tidx};
                        agent(n).estimator.result.state = state_copy(logger.Data.agent(n).estimator.result{tidx}.state);
                    case "reference"
                        agent(n).reference.result = logger.Data.agent(n).reference.result{tidx};
                        agent(n).reference.result.state = state_copy(logger.Data.agent(n).reference.result{tidx}.state);
                    case "controller"
                        agent(n).controller.result = logger.Data.agent(n).controller.result{tidx};
                        agent(n).input = logger.Data.agent(n).input{tidx};
                    case "plant"
                        agent(n).plant.state = state_copy(logger.Data.agent(n).plant.result{tidx}.state);
                end

            end

        end
    end
end

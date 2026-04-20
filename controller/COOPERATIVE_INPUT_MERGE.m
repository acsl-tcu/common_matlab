classdef COOPERATIVE_INPUT_MERGE < handle
    % Merge per-drone inputs into payload input for cooperative load model.
    properties
        self
        result
        payload_index
    end

    methods
        function obj = COOPERATIVE_INPUT_MERGE(self, opts)
            arguments
                self 
                opts.payload_index = [];
            end
            obj.payload_index = opts.payload_index;
            obj.self = self;
            obj.result.input = [];
        end

        function result = do(obj, varargin)
            agent_list = varargin{5};
            payload_id = obj.payload_index;
            if isempty(payload_id)
                payload_id = numel(agent_list);
            end

            if isprop(obj.self, "parameter") && isprop(obj.self.parameter, "N")
                N = obj.self.parameter.N;
            else
                N = numel(agent_list) - 1;
            end

            inputs = zeros(4 * N, 1);
            slot = 0;
            for k = 1:numel(agent_list)
                if k == payload_id
                    continue
                end
                slot = slot + 1;
                u = obj.get_agent_input(agent_list(k));
                if isempty(u)
                    u = zeros(4, 1);
                end
                inputs(4 * slot - 3:4 * slot, 1) = u(:);
            end

            obj.result.input = inputs;
            result = obj.result;
        end
    end

    methods (Access = private)
        function u = get_agent_input(~, agent_obj)
            u = [];
            if isprop(agent_obj, "controller") && isprop(agent_obj.controller, "result")
                res = agent_obj.controller.result;
                if isstruct(res)
                    if isfield(res, "input")
                        u = res.input;
                    end
                elseif isobject(res) && isprop(res, "input")
                    u = res.input;
                end
            end
        end
    end
end

classdef COOPERATIVE_LOAD_REF_OFFSET < handle
    % Offset reference position using payload pose and tether-point offset.
    % Uses self.reference.result as the input reference.
    properties
        self
        payload_index = []
        rho = []
    end

    methods
        function obj = COOPERATIVE_LOAD_REF_OFFSET(self, opts)
            arguments
                self
                opts.payload_index = []
                opts.rho = []
            end
            obj.self = self;
            obj.payload_index = opts.payload_index;
            obj.rho = opts.rho;
        end

        function result = do(obj, varargin)
            base_result = obj.self.reference.result;
            if isempty(base_result) || ~obj.has_field_or_prop(base_result, "state") || ~obj.has_field_or_prop(base_result.state, "xd")
                error("COOPERATIVE_LOAD_REF_OFFSET:MissingState", "Reference result.state.xd is missing.");
            end

            agent_list = varargin{5};
            payload_id = obj.payload_index;
            if isempty(payload_id)
                payload_id = numel(agent_list);
            end

            [payload_p, payload_Q] = obj.get_payload_pose(agent_list(payload_id));
            if isempty(payload_p) || isempty(payload_Q) || isempty(obj.rho)
                result = base_result;
                return
            end

            R_load = RodriguesQuaternion(payload_Q);
            R_load = R_load(:, :, 1);
            offset = R_load * obj.rho;

            xd = base_result.state.xd;
            if numel(xd) >= 3
                xd(1:3) = xd(1:3) + offset;
                base_result.state.xd = xd;
                base_result.state.p = xd(1:3);
            end

            result = base_result;
        end
    end

    methods (Access = private)
        function [p, Q] = get_payload_pose(~, payload_agent)
            p = [];
            Q = [];
            if isprop(payload_agent, "estimator") && isprop(payload_agent.estimator, "result")
                state = payload_agent.estimator.result.state;
                if isprop(state, "p") && isprop(state, "Q")
                    p = state.p;
                    Q = state.Q;
                    return
                end
            end
            if isprop(payload_agent, "sensor") && isprop(payload_agent.sensor, "result")
                state = payload_agent.sensor.result.state;
                if isprop(state, "p") && isprop(state, "Q")
                    p = state.p;
                    Q = state.Q;
                end
            end
        end

        function tf = has_field_or_prop(~, target, name)
            if isempty(target)
                tf = false;
                return
            end
            if isstruct(target)
                tf = all(isfield(target, name));
            elseif isobject(target)
                tf = all(isprop(target, name));
            else
                tf = false;
            end
        end
    end
end

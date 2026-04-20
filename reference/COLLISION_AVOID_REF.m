classdef COLLISION_AVOID_REF < handle
    % Adjust reference position to avoid nearby agents.
    % Uses self.reference.result as the input reference.
    properties
        self
        min_distance = 0.6
        margin = 0.1
        payload_index = []
    end

    methods
        function obj = COLLISION_AVOID_REF(self, opts)
            arguments
                self
                opts.min_distance = 0.6
                opts.margin = 0.1
                opts.payload_index = []
            end
            obj.self = self;
            obj.min_distance = opts.min_distance;
            obj.margin = opts.margin;
            obj.payload_index = opts.payload_index;
        end

        function result = do(obj, varargin)
            base_result = obj.self.reference.result;
            if isempty(base_result) || ~obj.has_field_or_prop(base_result, "state") || ~obj.has_field_or_prop(base_result.state, "xd")
                error("COLLISION_AVOID_REF:MissingState", "Reference result.state.xd is missing.");
            end

            agent_list = varargin{5};
            self_idx = varargin{6};
            payload_id = obj.payload_index;
            if isempty(payload_id)
                payload_id = numel(agent_list);
            end

            xd = base_result.state.xd;
            if numel(xd) < 3
                result = base_result;
                return
            end
            ref_p = xd(1:3);
            adjust = zeros(3, 1);
            target_dist = obj.min_distance + obj.margin;

            for k = 1:numel(agent_list)
                if k == self_idx || k == payload_id
                    continue
                end
                other_p = obj.get_agent_position(agent_list(k));
                if isempty(other_p)
                    continue
                end
                diff = ref_p - other_p;
                d = norm(diff);
                if d > 0 && d < target_dist
                    adjust = adjust + (target_dist - d) * (diff / d);
                end
            end

            if any(adjust)
                ref_p = ref_p + adjust;
                xd(1:3) = ref_p;
                base_result.state.xd = xd;
                base_result.state.p = ref_p;
            end

            result = base_result;
        end
    end

    methods (Access = private)
        function p = get_agent_position(~, agent_obj)
            p = [];
            if isprop(agent_obj, "estimator") && isprop(agent_obj.estimator, "result")
                state = agent_obj.estimator.result.state;
                if isprop(state, "p")
                    p = state.p;
                    return
                end
            end
            if isprop(agent_obj, "sensor") && isprop(agent_obj.sensor, "result")
                state = agent_obj.sensor.result.state;
                if isprop(state, "p")
                    p = state.p;
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

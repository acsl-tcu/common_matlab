classdef FUNCTION_CLASS_PROPERTY < dynamicprops
    % Container for function-class instances (sensor/estimator/etc.) with ordering support
    properties
        name string = string.empty
        result = []
        do = @(varargin) []
    end

    properties (Access = private)
        parent
        prop_name
    end

    methods
        function obj = FUNCTION_CLASS_PROPERTY(parent, prop_name)
            obj.parent = parent;
            obj.prop_name = prop_name;
        end

        function set_function_class(obj, fname, instance)
            arguments
                obj
                fname {mustBeTextScalar}
                instance
            end
            fname = string(fname);
            if ~isprop(obj, fname)
                addprop(obj, fname);
                obj.name = [obj.name, fname];
            end
            obj.(fname) = instance;
            obj.update_default_allocation();
        end
    end

    methods (Access = private)
        function update_default_allocation(obj)
            if isempty(obj.parent) || isempty(obj.name) || ~isprop(obj.parent, "cha_allocation")
                return
            end
            if isempty(obj.parent.cha_allocation)
                obj.parent.cha_allocation = struct();
            end
            list = obj.name;
            prop = obj.prop_name;
            obj.parent.cha_allocation.(prop) = list;
            phases = {'a', 't', 'f', 'l'};
            for k = 1:numel(phases)
                phase = phases{k};
                if ~isfield(obj.parent.cha_allocation, phase) || ~isstruct(obj.parent.cha_allocation.(phase))
                    obj.parent.cha_allocation.(phase) = struct();
                end
                obj.parent.cha_allocation.(phase).(prop) = list;
            end
        end
    end
end

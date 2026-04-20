classdef DIRECT_SENSOR < handle
% simulation用クラス：状態をそのまま返す
properties
    name = "direct";
    interface = @(x) x;
    result
    self
    noise
    do
    output_list
end

methods

    function obj = DIRECT_SENSOR(self, noise,opts)

        arguments
            self
            noise = 0;
            opts = [];
        end

        obj.self = self;
        obj.result.state = state_copy(self.plant.state);
        %            obj.result.state = state_copy(self.model.state);
        obj.noise = noise;
        if isfield(opts,"output_list")
          obj.output_list = string(opts.output_list);
        else
          obj.output_list = [];
        end
        if isfield(opts,"do")
          obj.do = opts.do;
        else
          obj.do = @obj.default_do;
        end
    end

    function result = default_do(obj, varargin)
        % 【入力】Target ：観測対象のModel_objのリスト
        tmp = obj.self.plant.state.get();
        obj.result.state.set_state(tmp + obj.noise * randn(size(tmp)));
        if isempty(obj.output_list)
            output = obj.result.state.get();
        else
            output = obj.result.state.get(obj.output_list);
        end
        obj.result.output = output;
        obj.result.y = output;
        result = obj.result;
    end

    function show(obj, varargin)

        if ~isempty(obj.result)
            obj.result
        else
            disp("do measure first.");
        end

    end

end

end

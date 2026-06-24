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
    q_type
end

methods

    function obj = DIRECT_SENSOR(self, noise, args)

        arguments
            self
            noise = 0;
            args.output_list = self.plant.state.list;
            args.do = [];
            args.q_type = 3; % 現状使っていない。モデルの姿勢角次元数に依存しない形で陽に指定して取り出せるようにしたい。
        end

        obj.self = self;
        obj.result.state = state_copy(self.plant.state);
        %            obj.result.state = state_copy(self.model.state);
        obj.noise = noise;
        obj.output_list = string(args.output_list);
        obj.q_type = args.q_type;
        if isempty(args.do)
          obj.do = @obj.default_do;
        else
          obj.do = args.do;
        end
    end

    function result = default_do(obj, varargin)
        % 【入力】Target ：観測対象のModel_objのリスト
        tmp = obj.self.plant.state.get();
        obj.result.state.set_state(tmp + obj.noise * randn(size(tmp)));
        output = obj.result.state.get(obj.output_list);
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

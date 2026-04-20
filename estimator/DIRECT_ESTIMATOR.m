classdef DIRECT_ESTIMATOR < handle
    % Directory generate the estimated state from the sensor output.
    % obj = DIRECT_ESTIMATOR(agent,~)
    properties
        % state
        result
        self
        model
    end
    methods
        function obj = DIRECT_ESTIMATOR(self,param)
            obj.self = self;
            obj.model = param.model;
            obj.result.state=state_copy(obj.model.state); % param.modelのSTATE_CLASSとしてコピー
            if ~isfield(obj.self.estimator,"result")
                obj.self.estimator.result = obj.result;
            end
        end

        function result=do(obj,varargin)
            % Copy field values corresponding to the field of obj.result.state (=model.state) only.
            % 【Input】
            % 【Output】void
            F = fieldnames(obj.result.state); %セル配列として取得
            for i = 1:length(F)
                if ~strcmp(F{i},'list') && ~strcmp(F{i},'num_list') && ~strcmp(F{i},'type')&& ~strcmp(F{i},'qlist')
                    %F{1}のリストと右辺が一致しているか
                    if contains(F{i}, fieldnames(obj.self.sensor.result.state))
                        %F{i}の中がfieldnames(obj.self.sensor.result.state)に含まれているか
                        obj.result.state.set_state(F{i},obj.self.sensor.result.state.(F{i}));
                        %set_stateの中にF{i}とresult.stateを格納
                    end
                end
            end
            result=obj.result;
        end
        
        function show(obj)
            obj.result.state
        end
    end
end


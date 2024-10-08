classdef STATE2HL_DRONE < handle
    % 階層型線形化入力を算出するクラス
    % 別のコントローラでone step 分予測したものをリファレンスとしてHL入力を求めるので，移動速度が遅過ぎて使えない．
    
    properties
        self
        result
        param
        flight_phase
        state
    end
    
    methods
        function obj = STATE2HL_DRONE(self,param)
            obj.self = self;
            obj.param = param;
            obj.param.P = self.parameter.get();
            obj.flight_phase = 's';
            obj.state=state_copy(obj.self.model.state);
            if ~isprop(obj.state,'v')
                addprop(obj.state,'v');
            end
            if ~isprop(obj.state,'q')
                addprop(obj.state,'q');
                addprop(obj.state,'w');
            end
        end
        
        function u = do(obj,input,param)
            if ~isempty(param)
                obj.param = param;
            end
            obj.state.set_state(obj.self.estimator.result.state.get());
            if ~isprop(obj.self.model.state,'v')
                if isprop(obj.self.sensor.result.state,'v')
                    obj.state.v = obj.self.sensor.result.state.v;
                else
                    obj.state.v = [0;0;0];
                end
            end
            if ~isprop(obj.self.model.state,'q')
                if isprop(obj.self.sensor.result.state,'q')
                    obj.state.q = obj.self.sensor.result.state.q;
                    obj.state.w = obj.self.sensor.result.state.w;
                    obj.state.type = obj.self.sensor.result.state.type;
                else
                    obj.state.q = [1;0;0;0];
                    obj.state.w = [0;0;0];
                    obj.state.type = 4;
                end
            end
            obj.self.model.do(input,param);
            u = HL_controller(obj.state,struct('xd',[obj.self.model.state.p]),obj.param); % input をreferenceとした階層型線形化入力算出
        end
    end
end


classdef SMOOTHENER_REFERENCE < handle
    % つなぎを滑らかにするためのファレンスを生成するクラス
    % obj = SMOOTHENER_REFERENCE()
    properties
        param
        func % 時間関数のハンドル
        self
        t=[];
        dfunc
        result
        fInit = 0;
        fSmooth = 1; % flag smooth transition
        base_state
        base_time
        duration
        target
        pre
    end

    methods
        function obj = SMOOTHENER_REFERENCE(self, option)
            % 【Input】ref_gen, param, "HL"
            % ref_gen : reference function generator
            % param : parameter to generate the reference function
            % "HL" : flag to decide the reference for HL
            arguments
                self
                option.target = "time_varying"
                option.pre = "takeoff"
                option.func = "class4_interpolation"
                option.duration = 5 % duration of transition [s]
            end
            obj.target = option.target;
            obj.pre = option.pre;
            obj.duration  = option.duration;
            obj.func = obj.default_gen_func(option.func,option.duration);%gen_ref_for_HL(@(t) option.func(t,option.duration,ps,vs,pe,ve));
            obj.self = self;
            obj.result.state = STATE_CLASS(struct('state_list', ["xd", "p", "q", "v"], 'num_list', [length(obj.func(0,zeros(4,1),zeros(4,1))), 3, 3, 3]));
            obj.result.state.set_state("xd",obj.func(0,zeros(4,1),zeros(4,1)));
            if isprop(obj.self.estimator.result.state,"p"), obj.result.state.set_state("p",obj.self.estimator.result.state.get("p"));end
            if isprop(obj.self.estimator.result.state,"q"), obj.result.state.set_state("q",obj.self.estimator.result.state.get("q"));end
            if isprop(obj.self.estimator.result.state,"v"), obj.result.state.set_state("v",obj.self.estimator.result.state.get("v"));end
            %syms t real
            %obj.dfunc = matlabFunction(diff(obj.func,t),"Vars",t);
        end
        function result = do(obj, varargin)
            org = obj.self.reference.(obj.target).result.state;
            if obj.fInit < 10 || isempty( obj.base_state ) %
                obj.base_time=varargin{1}.t;
                obj.base_state.p = obj.self.reference.(obj.pre).result.state.xd(1:4);
                obj.base_state.v = obj.self.reference.(obj.pre).result.state.xd(5:8);
                obj.fInit = obj.fInit + 1;
            end            
            state.xd = obj.gen_reference(varargin{1}.t-obj.base_time,obj.base_state.p,obj.base_state.v);
            state.p = state.xd(1:3);
            
            if length(state.xd)>4
                state.v = state.xd(5:7);
            else
                state.v = [0;0;0];
            end
            state.q(3,1) = state.xd(4);
            % obj.result.smoothener.state = state;%state_copy(obj.result.state);
            obj.result.state.set_state("xd",state.xd+org.xd,"p",state.p+org.p,"q",state.q+org.q,"v",state.v+org.v);
            result = obj.result;          
        end

        function xd = gen_reference(obj,t,p,v)
            if t<=obj.duration
                xd = obj.func(t,p,v);
            elseif t> obj.duration
                xd = zeros(20,1);
            end
        end
        function f = default_gen_func(obj,func_name,te)
            syms t real
            syms p0 [4 1] real
            syms v0 [4 1] real
            func = str2func(func_name);
            xd=func(t,te,p0,v0,0*p0,0*v0);
            dxd =diff(xd,t);
            ddxd =diff(dxd,t);
            dddxd =diff(ddxd,t);
            ddddxd =diff(dddxd,t);
            f = matlabFunction([xd;dxd;ddxd;dddxd;ddddxd],'vars',{t,p0,v0});

        end
    end
end

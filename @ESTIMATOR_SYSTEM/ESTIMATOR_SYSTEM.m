classdef ESTIMATOR_SYSTEM < matlab.System
    % untitled Add summary here
    %
    % This template includes the minimum set of functions required
    % to define a System object.

    % Public, tunable properties
    properties (DiscreteState)
    end
    properties(Access=public)
        type
    end
    properties (Access = private)
        % state : estimated state
        eclass
    end
    % Pre-computed constants or internal states
    properties (Access = private)
        param% =  [ 0.5000    0.1600    0.1600    0.0800    0.0800    0.0600    0.0600    0.0600    9.8100    0.0301    0.0301    0.0301    0.0301    0.0000    0.0000    0.0000    0.0000    0.0392];
        parameter_name = ["mass","Lx","Ly","lx","ly","jx","jy","jz","gravity","km1","km2","km3","km4","k1","k2","k3","k4"];
        t0
        result% = struct("state",zeros(12,1),"P",eye(12),"G",zeros(12,6));%struct("p",zeros(3,1),"v",zeros(3,1),"q",zeros(3,1),"w",zeros(3,1)));
        state
    end
    methods
        function obj = ESTIMATOR_SYSTEM(varargin)
            setProperties(obj,nargin,varargin{:})
        end
    end
    methods (Access = protected)

        function setupImpl(obj,dt,state,P)
            % obj.eclass=EKF(self, Estimator_EKF([],dt,MODEL_CLASS(self,Model_EulerAngle(dt, state, 1)),["p", "q"]));
            obj.eclass=Estimator_EKF([],dt,MODEL_CLASS([],Model_EulerAngle(dt, state, 1)),{"p", "q"});            
            obj.param = P;
            obj.state = state;
            obj.result.state = state;
            % obj.result = eparam.result;
            % obj.type = eparam.type;

        end
        function result = stepImpl(obj,time,y,u,agent)
            arguments
                obj
                time 
                y (6,1) {mustBeNumeric}
                u (4,1) {mustBeNumeric}
                agent
            end
            obj.result = obj.eclass.do(time,'f',[],u,agent,[]);           
            result=obj.result;
        end

        function resetImpl(obj)
            % Initialize / reset internal properties
        end
        function num = getNumInputsImpl(~)
            num = 4;
        end
    end
end

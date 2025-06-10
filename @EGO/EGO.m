classdef (StrictDefaults) EGO < matlab.System
    % EGO

    % Public, tunable properties
    properties (Access='private')
        controller
        estimator
        reference
        sensor
        parameter = struct("raw",struct(),"values",[]);
        sresult
        eresult
        rresult
        cresult
        time
        dt
        state
        control_signal
        publish_message
    end

    % Public, non-tunable properties
    properties (Nontunable)

    end

    % Discrete state properties
    % properties (DiscreteState)
    % end
    properties (Constant)
    end

    % Pre-computed constants or internal states
    properties (Access = protected)
        OutputBusName = 'myOutBus';%'bus_name';
    end

    methods
        % Constructor
        function obj = EGO(varargin)
            % Support name-value pair arguments when constructing object
            setProperties(obj,nargin,varargin{:})
            setting = coder.load("setting.mat","x0","u0","dt","rparam","parameter","state_name","cparam");
            obj.dt = setting.dt;
            ts = 0; % initial time
            te = 25; % terminal time
            obj.time = TIME(ts,obj.dt,te); % instance of time class
            % obj.state = setting.x0;
            gen_state = str2func(setting.state_name);
            obj.state = gen_state();
            obj.state.set(setting.x0);
            obj.control_signal = setting.u0;
            obj.parameter.values = setting.parameter.values;
            obj.publish_message = 0;
            obj.estimator = ESTIMATOR_SYSTEM();
            obj.controller = CONTROLLER_SYSTEM();
            obj.reference = REFERENCE_SYSTEM();
            % initial_state_eul = [obj.state(5:7);Quat2Eul(obj.state(1:4));obj.state(8:13)];
            setup(obj.reference,obj.dt,setting.rparam);
            obj.controller.setup(obj.dt,setting.cparam);
            obj.estimator.setup(obj.dt,obj.state,obj.parameter.values,[]);
            xd = zeros(19,1);
            xd(1) = 1;
            obj.controller.stepImpl([obj.state.q;obj.state.p;obj.state.v;obj.state.w],xd);
            obj.controller.result.input
        end
    end

    methods (Access = protected)
        %% Common functions
        function setupImpl(obj) % 初期化
            
        end

        function [u,pub] = stepImpl(obj,state,t) % 各時刻実行
            % Main algorithm
            disp(t) % to show the progress
            if t>100
                t
            end
            y = [obj.state.p;obj.state.getq(3)];
            obj.eresult = obj.estimator.stepImpl(obj.time,y,obj.control_signal,obj);
            obj.rresult = obj.reference.stepImpl(t,obj.eresult.state.get());
            x = [obj.eresult.state.q;obj.eresult.state.p;obj.eresult.state.v;obj.eresult.state.w];
            obj.cresult = obj.controller.stepImpl(x,obj.rresult.state.xd);
            obj.state.set(state);
            obj.control_signal = obj.cresult.input;           
            pub.rresult = obj.rresult;            
            pub.eresult.state = obj.eresult.state.get;
            pub.eresult.P = eye(12);%obj.eresult.P;
            % %  pub.eresult.G = obj.eresult.G;
            pub.cresult = obj.cresult;
            % pub.rresult.state.xd = zeros(20,1);
            % pub.rresult.state.p = zeros(3,1);
            % pub.rresult.state.v = zeros(3,1);
            % pub.rresult.state.q = zeros(3,1);
            % pub.eresult.state = zeros(13,1);
            % pub.eresult.P = eye(12);          
            % pub.cresult.input = [0;0;0;0];
            u = pub.cresult.input;
        end

        function resetImpl(obj) % リセット
            % Initialize / reset internal or discrete properties
            % setting = coder.load("setting.mat");
            % obj.parameter.values = setting.parameter.values;
            % gen_state = str2func(setting.state_name);
            % obj.state = gen_state();
            % obj.state.set(setting.x0);
            % obj.control_signal = setting.u0;
            % 
            % obj.publish_message = 0;
        end

        %% Backup/restore functions
        function s = saveObjectImpl(obj)
            % Set properties in structure s to values in object obj

            % Set public properties and states
            s = saveObjectImpl@matlab.System(obj);

            % Set private and protected properties
            %s.myproperty = obj.myproperty;
        end

        function loadObjectImpl(obj,s,wasLocked)
            % Set properties in object obj to values in structure s

            % Set private and protected properties
            % obj.myproperty = s.myproperty;

            % Set public properties and states
            loadObjectImpl@matlab.System(obj,s,wasLocked);
        end

        %% Simulink functions

        function icon = getIconImpl(obj)
            % Define icon for System block
            icon = mfilename("class"); % Use class name
            % icon = "My System"; % Example: text icon
            % icon = ["My","System"]; % Example: multi-line text icon
            % icon = matlab.system.display.Icon("myicon.jpg"); % Example: image file icon
        end

    end

    methods (Static, Access = protected)
        %% Simulink customization functions
        function header = getHeaderImpl
            % Define header panel for System block dialog
            header = matlab.system.display.Header(mfilename("class"));
        end

        function group = getPropertyGroupsImpl
            % Define property section(s) for System block dialog
            group = matlab.system.display.Section(mfilename("class"));
        end

        function [out,out1] = getOutputSizeImpl(obj)
            out = [4 1];
            out1 = [1 1];%propagatedInputSize(obj, 1);
        end

        function [out,out1] = isOutputComplexImpl(obj)
            out = propagatedInputComplexity(obj, 1);
            out1 = propagatedInputComplexity(obj, 1);
        end

        function [out,out1] = getOutputDataTypeImpl(obj)
            out = "double";
            out1 = obj.OutputBusName;
        end

        function [out,out1] = isOutputFixedSizeImpl(obj)
            out = propagatedInputFixedSize(obj, 1);
            out1 = propagatedInputFixedSize(obj, 1);
        end

    end
end

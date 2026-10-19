classdef MODEL_CLASS < dynamicprops & handle
    % General model class.
    % external_force_world : @(t,x) [Fx;Fy;Fz] [N] in world coordinates.
    % Only model instances with this property assigned receive external force.

    properties
        result
        name
        id
        method
        projection = @(x) x
        time_scale
        solver = str2func('ode15s')
        ts = 0
        dt = 0.025
        param
        dim
        noise
        fig
        self

        % Empty -> no disturbance. Set only on agent.plant.
        external_force_world = []
    end

    properties
        state
    end

    methods
        function obj = MODEL_CLASS(self,args)
            arguments
                self
                args
            end

            obj.self = self;
            if ~isempty(self.parameter)
                obj.param = self.parameter.get("all","row");
            end

            if isempty(regexp(args.type, 'EXP', 'once'))
                setting = args.param;
                obj.state = STATE_CLASS(setting);

                if isfield(setting,'initial')
                    obj.set_state(setting.initial);
                end

                obj.name = args.name;
                obj.dim = setting.dim;

                if isstring(setting.method) || ischar(setting.method)
                    obj.method = str2func(setting.method);
                else
                    obj.method = setting.method;
                end

                obj.time_scale = 'continuous';
                if contains(obj.name,'iscrete')
                    obj.time_scale = 'discrete';
                end

                names = fieldnames(setting);
                skip = {'qlist','initial','state_list','num_list', ...
                        'method','time_scale'};
                for j = 1:numel(names)
                    field = names{j};
                    if ismember(field,skip)
                        continue;
                    end
                    if strcmp(field,'solver')
                        if isa(setting.solver,'function_handle')
                            obj.solver = setting.solver;
                        else
                            obj.solver = str2func(setting.solver);
                        end
                    else
                        % Preserve existing model setup / parameter override.
                        obj.(field) = setting.(field);
                    end
                end
            end
        end

        function [] = show_do_setting(obj)
            if contains(obj.time_scale,'discrete')
                disp('discrete time model');
            else
                disp(['continuous time model ',char(obj.name)]);
                disp(['ts = ',num2str(obj.ts),'    dt = ',num2str(obj.dt)]);
            end
        end

        function result = do(obj,varargin)
            cha = varargin{2};
            if cha == 'q' || cha == 's' || cha == 'a'
                result = [];
                return;
            end

            u = obj.self.controller.result.input;
            if isempty(obj.param)
                obj.param = obj.self.parameter.get('all','row');
            end
            obj.dt = varargin{1}.dt;

            if contains(obj.time_scale,'discrete')
                if ~isempty(obj.external_force_world)
                    error('MODEL_CLASS:DiscreteExternalForce', ...
                        'External force is implemented only for continuous-time models.');
                end
                next_state = obj.method(obj.state.get(),u,obj.param);
                obj.set_state(obj.projection(next_state));
            else
                if isempty(obj.external_force_world)
                    % Original continuous-time dynamics (EKF etc. unaffected).
                    rhs = @(tau,x) obj.method(x,u,obj.param);
                else
                    % The ODE solver uses [obj.ts, obj.ts+dt]; translate to
                    % the global simulation time for the wind waveform.
                    t_step = varargin{1}.t;
                    rhs = @(tau,x) obj.dynamics_with_external_force( ...
                        t_step + (tau-obj.ts), x, u);
                end

                % For the supplied Euler-angle model obj.param is numeric.
                % Keep the optional legacy solver-option case if provided.
                if isstruct(obj.param) && isfield(obj.param,'solver_option')
                    [~,tmpx] = obj.solver(rhs, ...
                        [obj.ts obj.ts+obj.dt], obj.state.get(), ...
                        obj.param.solver_option);
                else
                    [~,tmpx] = obj.solver(rhs, ...
                        [obj.ts obj.ts+obj.dt], obj.state.get());
                end

                obj.set_state(obj.projection(tmpx(end,:)'));
            end

            obj.result = obj.state;
            result = obj.result;
        end

        function dx = dynamics_with_external_force(obj,t,x,u)
            % Euler-angle quadcopter state order:
            % x = [p(1:3); euler_angles(1:3); v(1:3); w(1:3)]
            % v and external force are both expressed in world coordinates.
            dx = obj.method(x,u,obj.param);

            if isempty(obj.external_force_world)
                return;
            end

            assert(isnumeric(obj.param) && numel(obj.param)>=1, ...
                'MODEL_CLASS:InvalidParameter', ...
                'Plant model parameters must be numeric with mass at index 1.');
            assert(numel(dx)>=9, 'MODEL_CLASS:InvalidState', ...
                'Model dynamics must contain translational acceleration at 7:9.');

            F = obj.external_force_world(t,x);
            F = F(:);
            assert(numel(F)==3 && all(isfinite(F)), ...
                'MODEL_CLASS:InvalidForce', ...
                'External force must be a finite 3-by-1 vector [N].');

            mass = obj.param(1);
            assert(isfinite(mass) && mass>0, ...
                'MODEL_CLASS:InvalidMass', 'Mass must be positive.');

            dx(7:9) = dx(7:9) + F/mass;
        end

        function [] = set_state(obj,varargin)
            if numel(varargin)>1
                obj.state.set_state(varargin{:});
            else
                obj.state.set_state(varargin{1});
            end
        end

        function state = get(obj,varargin)
            if strcmp(varargin{1},'state')
                state = obj.state;
            else
                state = obj.state.get(varargin{1});
            end
        end

        function show(obj)
            pp = patch(obj.fig(1),'FaceAlpha',0.3);
            pf = patch(obj.fig(2),'EdgeColor','flat', ...
                'FaceColor','none','LineWidth',0.2);
            pobj = [pp;pf];
            for i = 1:numel(pobj)
                pobj(i).Vertices = ...
                    (obj.state.getq('rotmat')*pobj(i).Vertices')' ...
                    + obj.state.p';
            end
        end

        function arming(obj)
        end

        function stop(obj)
        end
    end
end

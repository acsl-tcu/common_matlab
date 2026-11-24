function varargout=CBF_REAL_PAPER_INPUT_ADAPTER(action,varargin)
% ID3 ONLY. Input-side adapter; does NOT edit MODEL_CLASS or the plant RHS.
% MATLAB MODEL_CLASS.do calls plant.solver(odefun,tspan,x0). We replace
% ONLY the solver handle on this SIM plant instance; the original solver
% still performs the ODE integration, but sees physical T(t).
% 
% 'install',agent : attach to SIM plant before controller starts
% 'solve',agent,baseSolver,odefun,tspan,x0,... : invoked by MODEL_CLASS.do
% 'selftest' : local adapter algebra/integration test (no GUI required)
switch char(action)
    case 'install'
        agent=varargin{1};
        if ~isa(agent,'DRONE') || isempty(agent.plant) || ...
                ~isa(agent.plant,'MODEL_CLASS') || isa(agent.plant,'DRONE_EXP_MODEL')
            error('CBFPaper:SimOnly', ...
                'Paper thrust-profile adapter requires a simulation MODEL_CLASS plant.');
        end
        plant=agent.plant;
        oldSolver=plant.solver;
        if contains(func2str(oldSolver),'CBF_REAL_PAPER_INPUT_ADAPTER')
            error('CBFPaper:AlreadyInstalled','Paper input adapter is already installed');
        end
        if ~isa(oldSolver,'function_handle') || ~isa(plant.method,'function_handle')
            error('CBFPaper:ModelInterface','Plant ODE interface is unsupported');
        end
        % No modification of MODEL_CLASS.m or DRONE_EXP_MODEL.m.
        plant.solver=@(ode,tspan,x0,varargin) ...
            CBF_REAL_PAPER_INPUT_ADAPTER('solve',agent,oldSolver,ode,tspan,x0,varargin{:});
        fprintf('[ID3 paper input adapter] installed on SIM plant only\n');
    case 'solve'
        agent=varargin{1};solver=varargin{2};
        originalOde=varargin{3};tspan=double(varargin{4});x0=varargin{5};
        extra=varargin(6:end);
        if ~isa(originalOde,'function_handle') || numel(tspan)~=2
            error('CBFPaper:ODEInterface','Unsupported ODE callback or time span');
        end
        % Outside flight, use ORIGINAL solver and ORIGINAL physical T-ZOH.
        % This is the unmodified HLC takeoff/landing model, despite the
        % input-side solver adapter being installed at controller creation.
        if ~strcmp(char(agent.cha),'f')
            varargout=cell(1,max(nargout,1));
            [varargout{:}]=solver(originalOde,tspan,x0,extra{:});
            return;
        end
        if isempty(agent.controller) || ~isstruct(agent.controller.result) || ...
                ~isfield(agent.controller.result,'cbf') || ...
                ~isfield(agent.controller.result.cbf,'paperHold')
            error('CBFPaper:NoHold','No paper actuator hold at plant integration');
        end
        hold=agent.controller.result.cbf.paperHold;
        if ~isstruct(hold) || ~isfield(hold,'active') || ~hold.active || ...
                ~all(isfinite([hold.T0,hold.q0,hold.nu,hold.dt])) || ...
                abs((tspan(2)-tspan(1))-hold.dt)>1e-10
            error('CBFPaper:InvalidHold','Missing or mismatched paper actuator hold');
        end
        u=double(agent.controller.result.input(:));
        if numel(u)~=4 || any(~isfinite(u)) || abs(u(1)-hold.T0)>1e-8
            error('CBFPaper:ControlMismatch','Plant input does not match HOCBF hold');
        end
        plant=agent.plant;
        low=[0;-1;-1;-1];high=[20;1;1;1];
        if any(u<low-1e-9)||any(u>high+1e-9)
            error('CBFPaper:InputBox','Control outside physical input box');
        end
        t0=tspan(1);
        ode=@(t,x) plant.method(x, ...
            [hold.T0+hold.q0*(t-t0)+0.5*hold.nu*(t-t0).^2;u(2:4)], ...
            plant.param);
        varargout=cell(1,max(nargout,1));
        [varargout{:}]=solver(ode,tspan,x0,extra{:});
    case 'selftest'
        % Unit check: integrate xdot=T(t) with constant ddotT.
        T=8.2;q=-1.5;nu=80;dt=0.025;
        f=@(s) T+q*s+0.5*nu*s.^2;
        [~,xx]=ode45(@(t,x) f(t),[0 dt],0,odeset('RelTol',1e-11,'AbsTol',1e-12));
        expected=T*dt+q*dt^2/2+nu*dt^3/6;
        assert(abs(xx(end)-expected)<1e-9,'CBFPaper:AdapterSelftest');
        varargout={struct('passed',true,'integral',xx(end),'analytic',expected)};
    otherwise
        error('CBFPaper:UnknownAction','Unknown adapter action %s',char(action));
end
end

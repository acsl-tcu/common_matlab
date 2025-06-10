classdef MODEL_CLASS < handle
  % general model class
  % obj = MODEL_CLASS(name,param)
  %      name : 名前（obsolete）
  %      param : structure of
  %            "state_list" : ['p','v']
  %            "num_list" : [3,3]
  %            "initial" (optional)     : struct('p',[0;0;0],'v',[0;0;0])
  %            "dim"         : n, m, p :  number of state, input, and physical parameters
  properties
    result
    name % model dynamicsの名前
    id % 一意に決定するもの : integer
    method % = str2func(name)
    projection = @(x) x; % 射影が必要な時は設定する．(obsolete ? )
    time_scale % discrete or continuous
    solver = str2func('ode15s') % 指数1　のDAEを解ける．
    % solver = str2func('ode45') % 指数1　のDAEを解ける．
    ts = 0;
    dt = 0.025;
    % state.list % 例 ["p","q","v","w"]
    % state.num_list % 例 [3,4,3,3]
    param % parameters
    dim % n, m, p :  number of state, input, and physical parameters
    noise
    fig
    self
  end

  properties %(Access=private)
    state %=STATE_CLASS(); % 状態の構造体
  end

  methods

    function obj = MODEL_CLASS(self,args) % constructor

      arguments
        self
        args
      end
      obj.self = self;
      % if ~isempty(self.parameter)
      %    obj.param = obj.self.parameter.get("all","row");%varargin{5}.parameter.get();
      % end
        param = args.param;
        state_name = str2func(param.state_name);
        obj.state = state_name(param.initial);

        obj.name = args.name;
        obj.dim = param.dim;
  
        if isstring(param.method)
          obj.method = str2func(param.method);
        else
          obj.method = param.method;
        end

        obj.time_scale = 'continuous';

        if contains(obj.name, "iscrete")
          obj.time_scale = 'discrete';
        end

        F = fieldnames(param);

        for j = 1:length(F)

          if  ~strcmp(F{j}, 'state_name') && ~strcmp(F{j}, 'initial') &&  ~strcmp(F{j}, 'method') && ~strcmp(F{j}, 'time_scale')

            if strcmp(F{j}, 'solver')
              obj.solver = str2func(param.solver);
            else
              obj.(F{j}) = param.(F{j});
            end

          end

        end
    end

    function [] = show_do_setting(obj)

      if contains(obj.time_scale, 'discrete')
        disp("discrete time model");
      else
        disp(['continuous time model', obj.name]);
        disp(['ts = ', num2str(obj.ts), '    dt = ', num2str(obj.dt)]);
      end

    end

    function result = do(obj, varargin)
      
      cha = varargin{2};
      if (cha == 'q' || cha == 's' || cha == 'a')
        result = [];
        return
      end
      % u = varargin{5}.controller.result.input;
      u = varargin{4};
      if isempty(obj.param)
        obj.param = obj.self.parameter.get("all","row");%varargin{5}.parameter.get();
      end
      % if isfield(opts, 'param')
      %     obj.param = opts.param;
      % end
      obj.dt = varargin{1}.dt;
      % if ~isempty(obj.noise)
      %
      %     if ~isempty(obj.noise.seed)
      %         rng(obj.noise.seed * obj.param.t);
      %     else
      %         rng('shuffle');
      %     end
      %
      %     u = u + obj.noise.value .* randn(size(u));
      %            end

      % 状態更新
      if contains(obj.time_scale, 'discrete')
        obj.state.set(obj.method(obj.state.get(), u, obj.param));
      else

        % if isfield(obj.param, 'solver_option')
        %   [~, tmpx] = obj.solver(@(t, x) obj.method(x, u, obj.param), [obj.ts obj.ts + obj.dt], obj.state.get(), opts.solver_option);
        % else
        %   [~, tmpx] = obj.solver(@(t, x) obj.method(x, u, obj.param), [obj.ts obj.ts + obj.dt], obj.state.get());
        % end
        tmpx = obj.state.get()';
        obj.state.set(tmpx(end, :)');
      end

      obj.result.state = obj.state;
      result = obj.result;
    end

    function show(obj)
      %rad = norm(rot);
      %dir = rot/rad;
      pp = patch(obj.fig(1), 'FaceAlpha', 0.3);
      pf = patch(obj.fig(2), 'EdgeColor', 'flat', 'FaceColor', 'none', 'LineWidth', 0.2);

      pobj = [pp; pf];

      for i = 1:length(pobj)
        pobj(i).Vertices = (obj.state.getq('rotmat') * pobj(i).Vertices')' + obj.state.p';
      end

      %rotate(obj,dir,180*rad/pi,orig+trans);
    end
    function arming(obj)
    end
    function stop(obj)
    end

  end

end

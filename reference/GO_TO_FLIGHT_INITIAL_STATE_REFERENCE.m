classdef GO_TO_FLIGHT_INITIAL_STATE_REFERENCE < handle
    % TAKEOFF_REFERENCEの後にカスケードで実行されることを想定しているクラス
    % 9次多項式を用いて、緩やかにflightフェーズの初期状態[x;y;z;yaw]へと移行
  properties
    self
    base_state  % takeoff高度に収束し、移行し始めたときの[x; y; z; yaw]
    base_time   % takeoff高度に収束し、移行し始めたときの時刻
    goal_state % [x; y; z; yaw]
    takeoff_ref_class
    te          % takeoff高度に収束してから移行するまでの設定時間
    result
    sigma = 0.05 % 高度収束判定の閾値[m]
    fInit = 0
    fCount = 0
    fConvergence = false
  end

  methods
    function obj = GO_TO_FLIGHT_INITIAL_STATE_REFERENCE(self, opts)
        arguments
            self 
            opts.flight_ref_class = [] % flight refクラスを陽に指定しても良い。指定がない場合はdoメソッド内で値を取得
            opts.te = 5 % takeoff高度に収束してから何秒で移行完了させるか
        end
      obj.self = self;
      if isprop(obj.self.reference, "takeoff") % 名前をtakeoffに限定
          if ~isa(obj.self.reference.takeoff, "TAKEOFF_REFERENCE")
              error("agent.reference.takeoffにはTAKEOFF_REFERENCEを使用してください")
          end
      else
          error("agent.reference.takeoffを本クラスの前に定義してください");
      end
      obj.takeoff_ref_class = obj.self.reference.takeoff;

      if ~isempty(opts.flight_ref_class)
          xd = opts.flight_ref_class.func(0);
          obj.goal_state = xd(1:4);
      end

      if isfield(opts, "te"),obj.te = opts.te;
      else, obj.te = obj.takeoff_ref_class.te; % te指定がない時はtakeoffクラスのteを引継ぎ
      end

      obj.result.state = STATE_CLASS(struct('state_list',["xd","p","v"],'num_list',[20,3,3]));
    end

    function  result= do(obj,varargin)
        % [varargin] time,cha,logger,env
        if (obj.fInit < 2 || isempty(obj.goal_state)) % 初期化みたいなもの。本当はインスタンス生成時にやりたい
            xd = obj.self.reference.(obj.self.cha_allocation.f.reference).func(0);
            obj.goal_state = xd(1:4);
            if varargin{2} == 't'
                obj.fInit = obj.fInit + 1;
            end
        end

        if abs( obj.self.estimator.(obj.self.cha_allocation.t.estimator).result.state.p(3)-obj.takeoff_ref_class.zd ) < obj.sigma
            % 高度が収束判定の閾値obj.sigma以下ならカウントアップ
            obj.fCount = obj.fCount + 1;
            if obj.fCount >= 10, obj.fConvergence=true; end
        else
            obj.fCount = 0;
        end

        t = varargin{1}.t;
        if (t - obj.takeoff_ref_class.base_time > obj.takeoff_ref_class.te*1.5 && ... takeoff.te*1.5は余裕を持たせるためのテキトーな値
            obj.fConvergence)
            if obj.fInit == 2
                obj.base_time = t;
                obj.base_state = [obj.self.reference.result.state.p;...     % x,y,z : reference using at takeoff phase
                                  obj.self.estimator.result.state.q(3)];    % yaw: current yaw angle
                obj.fInit = obj.fInit + 1;
            end
            obj.result.state.xd = obj.gen_ref_for_go_to_flight_init(t - obj.base_time);
            obj.result.state.p = obj.result.state.xd(1:3,1);
            obj.result.state.v = obj.result.state.xd(5:7,1);
        else
            obj.result.state.xd = obj.takeoff_ref_class.result.state.xd;
            obj.result.state.p = obj.result.state.xd(1:3,1);
            obj.result.state.v = obj.result.state.xd(5:7,1);
        end
        result = obj.result;
    end


    function Xd = gen_ref_for_go_to_flight_init(obj, t)
        %% Setting
        % calc reference position and its higher time derivatives
        % reference designed as a 9-degree polynomial function of time
        % [Inputs]
        % t : current time
        %
        % [Output]
        % Xd : reference [[p;yd], [p^(1);0], [p^(2);0], [p^(3);0], [p^(4);0]] as column vector
        %    : Xd in R^20
        %    : yd is a yaw angle reference

        %% Variable set
        Xd  = zeros(20, 1);
        %% Set Xd
        if t<=obj.te
            xd = curve_interpolation_9order(t, obj.te, obj.base_state(1), 0, obj.goal_state(1), 0);
            yd = curve_interpolation_9order(t, obj.te, obj.base_state(2), 0, obj.goal_state(2), 0);
            zd = curve_interpolation_9order(t, obj.te, obj.base_state(3), 0, obj.goal_state(3), 0);
            yawd = curve_interpolation_9order(t, obj.te, obj.base_state(4), 0, obj.goal_state(4), 0);
            % TODO: ↑curve_interpolation_9orderをベクトルに対応させるべき
            tmp = [xd; yd; zd; yawd];
            Xd = tmp(:);
        elseif t> obj.te
            Xd(1:4) = obj.goal_state;
        end
    end

  end
end

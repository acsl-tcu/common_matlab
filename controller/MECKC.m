classdef MECKC < handle
  % Hierarchical linearization based controller for a quadcopter
  properties
    self
    result
    param
    parameter_name = ["mass","Lx","Ly","lx","ly","jx","jy","jz","gravity","km1","km2","km3","km4","k1","k2","k3","k4"];
    agent
    delta_u
    pre_input = [0;0;0;0];
    x_pre = [0;0;0;0;0;0;0;0;0;0;0;0];
  end

  methods
      function obj = MECKC(self,param)
      obj.self = self;
      obj.param = param;
      obj.param.P = self.parameter.get(obj.parameter_name);
      obj.result.input = zeros(4,1);
      end

    function result = do(obj,varargin)
      model = obj.self.estimator.result;
      x = [model.state.p(1);
             model.state.p(2);
             model.state.p(3);
             model.state.q(1);
             model.state.q(2);
             model.state.q(3);
             model.state.v(1);
             model.state.v(2);
             model.state.v(3);
             model.state.w(1);
             model.state.w(2);
             model.state.w(3);];
        if isfield(varargin{3}.Data.agent, "controller") && isfield(varargin{3}.Data.agent, "estimator") &&(numel(varargin{3}.Data.agent.estimator.result) >= 2)% ループの最初はLoggingされていなくて，参照できないのを回避
                obj.pre_input = varargin{3}.Data.agent.controller.result{end}.input; % LOGGERの中から前時刻の入力を取得
                obj.x_pre = varargin{3}.Data.agent.estimator.result{:,end-1}.state.get; % LOGGERの中から前時刻の状態を取得
        end
        dt = varargin{1}.dt;
        dx = roll_pitch_yaw_thrust_torque_physical_parameter_model(obj.x_pre, obj.pre_input, obj.param.P);
        x_n_now = obj.x_pre + dx*dt;%x_nominal[k+1]
        % delta_x = x-x_n_now;
        z_p=quaternions_all(x); %観測量z※プラントの状態を入れてる
        z_n=quaternions_all(x_n_now);%ノミナルの状態
        
        %---可制御部分をデカップリング---%
        % K_full=[zeros(4,24)];
        load('kalman_gainたち\retry_LYKL_gain.mat','K_full');
        e = z_n-z_p;
        obj.result.delta_u = -K_full*e;
        %%%%%-----lqr法終わり-----%%%%%
        %拡張状態取得
        % ※一気にz_pなどと取得するとplot時に警告が出る(50まで)
        obj.result.z_p_forward = z_p(1:13);%プラント拡張状態の前半
        obj.result.z_n_forward = z_n(1:13);
        obj.result.z_p_back = z_p(14:26);
        obj.result.z_n_back = z_n(14:26);%ノミナル拡張状態の後半

        % obj.result.delta_u = 0;%unだけ確認したいとき
        
      obj.result.input=varargin{5}.controller.result.input + obj.result.delta_u;%un+Δu      
      result = obj.result;
    end
  end
end
classdef FUNCTIONAL_MECKC < handle
% クアッドコプター用階層型線形化を使った入力算出
% シミュレーションに使ったMECプログラム
properties
    self
    result
    param
    parameter_name = ["mass", "Lx", "Ly", "lx", "ly", "jx", "jy", "jz", "gravity", "km1", "km2", "km3", "km4", "k1", "k2", "k3", "k4"];
    Vf
    Vs
    agent
    motive
    delta_u
    pre_input = [0;0;0;0];
    x_pre = [0;0;0;0;0;0;0;0;0;0;0;0];
end

methods

    function obj = FUNCTIONAL_MECKC(self, param)
        


        % obj.data_gen_mode = true;
        % obj.data_gen_mode = false;
        % true:Δuを生成・保存, false:MECの検証

        obj.self = self;
        obj.param = param;
        obj.param.P = self.parameter.get(obj.parameter_name);
        obj.result.input = zeros(self.estimator.model.dim(2),1);

        initial_state.p = self.plant.state.p;
        initial_state.q = self.plant.state.q;
        initial_state.v = self.plant.state.v;
        initial_state.w = self.plant.state.w;

        obj.Vf = obj.param.Vf; % 階層１の入力を生成する関数ハンドル
        obj.Vs = obj.param.Vs; % 階層２の入力を生成する関数ハンドル
        
    end

    function result = do(obj,varargin)
        model = obj.self.estimator.result;
        ref = obj.self.reference.result;
        disp(ref.state.p);
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
        % x = [R2q(Rb0' * model.state.getq("rotmat")); Rb0' * model.state.p; Rb0' * model.state.v; model.state.w]; % [q, p, v, w]に並べ替え
        %%MECK
        if isfield(varargin{3}.Data.agent, "controller") && isfield(varargin{3}.Data.agent, "estimator") &&(numel(varargin{3}.Data.agent.estimator.result) >= 2)% ループの最初はLoggingされていなくて，参照できないのを回避
                obj.pre_input = varargin{3}.Data.agent.controller.result{end}.input; % LOGGERの中から前時刻の入力を取得
                obj.x_pre = varargin{3}.Data.agent.estimator.result{:,end-1}.state.get; % LOGGERの中から前時刻の状態を取得
        end
        dt = varargin{1}.dt;
        dx = roll_pitch_yaw_thrust_torque_physical_parameter_model(obj.x_pre, obj.pre_input, obj.param.P);
        x_n_now = obj.x_pre + dx*dt;%x_nominal[k+1]
        delta_x = x-x_n_now;
        z_p=quaternions_all(x); %観測量z※プラントの状態を入れてる
        z_n=quaternions_all(x_n_now);%ノミナルの状態
        % y_p=obj.param.C*z_p;
        % y_n=obj.param.C*z_n;

        A = obj.param.est.A;%クープマンモデルのA,B,C
        B = obj.param.est.B;
        C = obj.param.est.C;
        
        %%-----スライディングモード制御-----%%
        % e=obj.x_pre-C*z_p;
        % e = C*(z_n-z_p);
        % Z=z_n-z_p;
        % fai=1;
        % S_1 = [0,0,1,0,0,0,0,0,0,0,0,0];
        % S_2 = [0,0,0,1,0,0,0,0,0,0,0,0];
        % S_3 = [0,0,0,0,0.1,0,0,0,0,0,0,0];
        % S_4 = [0,0,0,0,0,1,0,0,0,0,0,0];
        % S_all = [S_1;S_2;S_3;S_4];
        % sig = S_all*e;
        % sat = zeros(4,1);
        % for i = 1:length(sig)
        % if abs(sig(i)) <= fai
        %     sat(i,1) = sig(i)/fai;
        % else
        %     sat(i,1) = sign(sig(i));
        % end
        % end
        % SCB = S_all * C*B;
        % rank(SCB);
        % cond(SCB);
        % pinv_SCB = pinv(SCB,1e-3);
        % % u_equal = -pinv_SCB*(S_all*x_n_now-S_all*C*A*z_p);
        % u_equal = -pinv_SCB*S_all*C*A*(z_n-z_p);
        % u_controll = -pinv_SCB*diag([1 1 1 1])*sat;
        % obj.result.delta_u = u_equal+u_controll;%Δu計算
        %%-----スライディングモード終わり-----%%
        
        %%%%%-----lqr法-----%%%%%
        
        % Q = diag([1, 1, 1, 1,1,1,ones(1,20)]);
        % R = 1 * eye(4);
        % %---不可制御を含んだdlqr---%
        % [K_direct,~,~] = dlqr(A,B,Q,R);
        % K_full=K_direct ;
        %---不可制御を含んだdlqr終わり---%

        %---可制御部分をデカップリング---%
        load('kalman_gain.mat','K_full');
        e = z_n-z_p;
        obj.result.delta_u = -K_full*e;
        %%%%%-----lqr法終わり-----%%%%%

        % obj.result.delta_u = 0;%unだけ確認したいとき
        
        obj.result.input=varargin{5}.controller.nominal.result.u_nominal+obj.result.delta_u;%un+Δu
        result=obj.result;
    end

    function show(obj)
        obj.result
    end

end

end
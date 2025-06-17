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
    % MECNN       % DNNアーキテクチャ
    Pn_p_pre    % 前時刻のノミナルの位置←なんか使ってない…
    Pa_p_pre    % 前時刻のプラントの推定位置
    Pn_p_cur    % 状態更新後のノミナルの出力（位置p，速度v，角度q，各速度w）
    Pa_p_cur    % 状態更新後のプラントの推定値
    Pn_u        % ノミナルのコントローラから得られた制御入力
    delta_u
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
        xd = ref.state.xd;
        P = obj.param.P;
        F1 = obj.param.F1;
        F2 = obj.param.F2;
        F3 = obj.param.F3;
        F4 = obj.param.F4;
        xd = [xd; zeros(20 - size(xd, 1), 1)]; % 足りない分は０で埋める．

        Rb0 = RodriguesQuaternion(Eul2Quat([0; 0; xd(4)]));
        x = [R2q(Rb0' * model.state.getq("rotmat")); Rb0' * model.state.p; Rb0' * model.state.v; model.state.w]; % [q, p, v, w]に並べ替え
        xd(1:3) = Rb0' * xd(1:3);
        xd(4) = 0;
        xd(5:7) = Rb0' * xd(5:7);
        xd(9:11) = Rb0' * xd(9:11);
        xd(13:15) = Rb0' * xd(13:15);
        xd(17:19) = Rb0' * xd(17:19);

        %% calc Z
        z1 = Z1(x, xd', P);%z
        vf = obj.Vf(z1, F1);
        z2 = Z2(x, xd', vf, P);%x
        z3 = Z3(x, xd', vf, P);%y
        z4 = Z4(x, xd', vf, P);%yaw
        vs = obj.Vs(z2, z3, z4, F2, F3, F4);

        %% calc actual input
       tmp = Uf(x, xd', vf, P) + Us(x, xd', vf, vs, P);
        %%input of subsystems
        obj.result.uHL = [vf(1); vs];
        %differential virtual input first layer
        obj.result.vf = vf;
        %state of subsystems
        obj.result.z1 = z1;
        obj.result.z2 = z2;
        obj.result.z3 = z3;
        obj.result.z4 = z4;
        obj.result.input = [max(0,min(10,tmp(1)));max(-1,min(1,tmp(2)));max(-1,min(1,tmp(3)));max(-1,min(1,tmp(4)))];

        %%MECK
        z_p=quaternions_all(x); %観測量z※プラントの状態を入れてる
        % z_n=quaternions_all(ref.state.xd);%ノミナルの状態
        z_n=quaternions_all(xd);%ノミナルの状態
        y_p=obj.param.est.C*z_p;
        y_n=obj.param.est.C*z_n;
        
        D_zero=[1 1 1 0 0 0 0 0 0 0 0 0;%フィードバックゲイン4×12次元にしたい(5/27(火)に決めたテキトーゲイン)
               0 0 0 0 0 0 0 0 0 0 0 0;
               0 0 0 0 0 0 0 0 0 0 0 0;
               0 0 0 0 0 0 0 0 0 0 0 0];
        
        % D=D_zero+0.02*varargin{1}.t;%ゲイン半自動調整
        
        S=y_p-y_n;%スライディングモードの曲面　
    
        % sat = min(1,max(-1,S/dh));%-1<=S<=1
        sat = max(-1,S);%-1と比べて大きい方を返す
        sat=min(1,sat);%1と比べて小さい方を返す
        obj.result.delta_u = -D_zero*sat;%Δu計算
        % obj.result.delta_u = 0;%unだけ確認したいとき
        
        obj.result.input=varargin{5}.controller.nominal.result.u_nominal+obj.result.delta_u;%un+Δu
        result=obj.result;
    end

    function show(obj)
        obj.result
    end

end

end
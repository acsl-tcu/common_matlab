classdef MPC_CONTROLLER_HL <handle
    % MCMPC_CONTROLLER MCMPCのコントローラー

    properties
    self
    result
    param
    parameter_name = ["mass","Lx","Ly","lx","ly","jx","jy","jz","gravity","km1","km2","km3","km4","k1","k2","k3","k4"];
    end

    properties
%         options
        current_state
        previous_input
        input
        state
        const
        reference
        fRemove
        model
    end
    properties
        modelf
        modelp
        F1
        weight
        weightF
        weightR
        weightRp
        A
        B
        C
        H
        mpc
        qpparam
    end

    methods
        function obj = MPC_CONTROLLER_HL(self, param)
            %-- 変数定義
            obj.self = self;
            obj.param = param;
            obj.param.P = self.parameter.get(obj.parameter_name);
            obj.input = obj.param.input; % obj.param = Controller_HLMPC.mで設定したパラメーター

            %% HL関連
            obj.F1=lqrd([0 1;0 0],[0;1],diag([100,1]),0.1,param.dt);
            % 重みの配列サイズ変換
            obj.weight = blkdiag(obj.param.Z, obj.param.X, obj.param.Y, obj.param.PHI);
            obj.weightF = blkdiag(obj.param.Zf, obj.param.Xf, obj.param.Yf, obj.param.PHIf);
            obj.weightR = obj.param.R;  % 目標入力
            obj.weightRp = obj.param.RP; % 前ステップとの入力
            obj.H = obj.param.H;
            % HL. A, B行列定義
            % z, x, y, yawの順番
            A = blkdiag([0,1;0,0],diag([1,1,1],1),diag([1,1,1],1),[0,1;0,0]);
            B = blkdiag([0;1],[0;0;0;1],[0;0;0;1],[0;1]);
            sysd = c2d(ss(A,B,eye(12),0),param.dt); % 離散化
            obj.A = sysd.A;
            obj.B = sysd.B;
            obj.C = eye(12); 
            %% 入力
            obj.result.input = zeros(self.estimator.model.dim(2),1); % 入力事前定義(これがないと初期化で失敗する)
            obj.previous_input = repmat(obj.input.u, 1, obj.H); % MPC用の初期入力

            %% QP change_equationの共通項をあらかじめ計算
            Param = struct('A',obj.A,'B',obj.B,'C',obj.C,'weight',obj.weight,'weightF',obj.weightF,'weightR',obj.weightR,'weightRp',obj.weightRp,'H',obj.H);
            [obj.qpparam.H, obj.qpparam.F] = change_equation_drone(Param);
            % H: 変数
            % F: fを生成するために必要な行列
        end

        %-- main()的な
        function result = do(obj,varargin)
            tic
            %% ほぼ2コン
            %%initialize
            time = varargin{1};
            phase = varargin{2};
            obj.param.t = time.t;
            %% phaseによるcontrollerの選択
            % result: controllerで算出された入力

            if phase == 'a'
                obj.state.ref = repmat([0;0;1;0;0;0;0;0;0;0;0;0;obj.param.ref_input;0;0;0],1,obj.param.H);
                result = obj.controller_HLMPC(varargin);
                disp('controller: MPC  phase: a');
            elseif phase == 't' || phase == 'l'
                result = obj.controller_HL(varargin);
                disp('controller: HL   phase: t or l');
            elseif phase == 'f'
                obj.state.ref = obj.generate_reference();
                result = obj.controller_HLMPC(varargin);
                disp('controller: MPC  phase: f');
            end 
            toc
            % profile viewer
        end

        function result = controller_HL(obj,varargin)
            model = obj.self.estimator.result;
            ref = obj.self.reference.result;
            xd = ref.state.xd;
            xd0 =xd;
            P = obj.param.P;
            F1 = obj.param.F1;
            F2 = obj.param.F2;
            F3 = obj.param.F3;
            F4 = obj.param.F4;
            xd=[xd;zeros(20-size(xd,1),1)];% 足りない分は０で埋める．
    
            % yaw 角についてボディ座標に合わせることで目標姿勢と現在姿勢の間の2pi問題を緩和
            % TODO : 本質的にはx-xdを受け付ける関数にして，x-xdの状態で2pi問題を解決すれば良い．
            Rb0 = RodriguesQuaternion(Eul2Quat([0;0;xd(4)]));
            x = [R2q(Rb0'*model.state.getq("rotmat"));Rb0'*model.state.p;Rb0'*model.state.v;model.state.w]; % [q, p, v, w]に並べ替え
            xd(1:3)=Rb0'*xd(1:3);
            xd(4) = 0;
            xd(5:7)=Rb0'*xd(5:7);
            xd(9:11)=Rb0'*xd(9:11);
            xd(13:15)=Rb0'*xd(13:15);
            xd(17:19)=Rb0'*xd(17:19);
            %if isfield(obj.param,'dt')
            if isfield(varargin{1},'dt') && varargin{1}.dt <= obj.param.dt
                dt = varargin{1}.dt;
            else
                dt = obj.param.dt;
                % vf = Vf(x,xd',P,F1);
                % vs = Vs(x,xd',vf,P,F2,F3,F4);
            end
            vf = Vfd(dt,x,xd',P,F1);
            vs = Vsd(dt,x,xd',vf,P,F2,F3,F4);
            %disp([xd(1:3)',x(5:7)',xd(1:3)'-xd0(1:3)']);
            tmp = Uf(x,xd',vf,P) + Us(x,xd',vf,vs',P);
            % max,min are applied for the safty
            obj.result.input = [max(0,min(10,tmp(1)));max(-1,min(1,tmp(2)));max(-1,min(1,tmp(3)));max(-1,min(1,tmp(4)))];
            result = obj.result;
        end

        function result = controller_HLMPC(obj, varargin)
            xd = obj.self.reference.result.state.xd; % 目標値の取得 referenceクラス
            xd=[xd;zeros(32-size(xd,1),1)]; % 足りない分は０で埋める．
    
            model_HL = obj.self.estimator.result;
            Rb0 = RodriguesQuaternion(Eul2Quat([0;0;xd(4)]));
            xn = [R2q(Rb0'*model_HL.state.getq("rotmat"));Rb0'*model_HL.state.p;Rb0'*model_HL.state.v;model_HL.state.w]; % [q, p, v, w]に並べ替え
            xd(1:3)=Rb0'*xd(1:3);
            xd(4) = 0;
            xd(5:7)=Rb0'*xd(5:7);
            xd(9:11)=Rb0'*xd(9:11);
            xd(13:15)=Rb0'*xd(13:15);
            xd(17:19)=Rb0'*xd(17:19);
            P = obj.self.parameter.get();
            vfn = Vf(xn,xd',P,obj.param.F1); %v1
            z1n = Z1(xn,xd',P);
            z2n = Z2(xn,xd',vfn,P);
            z3n = Z3(xn,xd',vfn,P);
            z4n = Z4(xn,xd',vfn,P);
            obj.current_state = [z1n(1:2);z2n(1:4);z3n(1:4);z4n(1:2)];
    
            %% Referenceの取得、ホライズンごと
            obj.reference.xr = [zeros(16, obj.param.H)];

            % 最適化部分
            Param = struct('current_state',obj.current_state,'ref',obj.reference.xr,'u',obj.input.u,'qpH', obj.qpparam.H, 'qpF', obj.qpparam.F,'lb',obj.param.input.lb,'ub',obj.param.input.ub,'previous_input',obj.previous_input,'H',obj.H,'F',obj.param.F);
            [var, fval, exitflag] = obj.param.quad_drone(Param);
            %%--

            obj.previous_input = var; % 最適化の初期値

            vf = var(1, 1);     % 最適な入力の取得
            vs = var(2:4, 1);     % 最適な入力の取得
            tmp = Uf(xn,xd',vf,P) + Us(xn,xd',[vf,0,0],vs(:),P);  % 入力変換
            obj.result.input = tmp(:);%[tmp(1);tmp(2);tmp(3);tmp(4)]; 実入力変換
            % obj.result.input =
            % [max(0,min(10,tmp(1)));max(-1,min(1,tmp(2)));max(-1,min(1,tmp(3)));max(-1,min(1,tmp(4)))];
            % % HL同様の入力制限
            obj.input.u = obj.result.input; % 入力をcontroller内に保存
            obj.result.mpc.var = var;
            obj.result.mpc.exitflag = exitflag;
            obj.result.mpc.fval = fval;
            obj.result.mpc.xr = obj.state.ref;

            obj.result.input_v = [vf; vs]; % 仮想入力の保存
            % obj.result.xr = xr_real;
            result = obj.result;
        end
        
        function [xr] = generate_reference(obj, ~)
            xr = zeros(16, obj.param.H);    % initialize
            % 時間関数の取得→時間を代入してリファレンス生成
            RefTime = obj.self.reference.func;    % 時間関数の取得
            for h = 0:obj.param.H-1
                t = obj.param.t + obj.param.dt * h; % reference生成の時刻をずらす
                r = RefTime(t);
                xr(1:12,h+1) = [r(3);r(7);r(1);r(5);r(9);r(13);r(2);r(6);r(10);r(14);r(4);r(8)];
                xr(13:16,h+1)= obj.param.ref_input;
            end
        end

        function show(obj)
            clc;
            est_print = obj.self.estimator.result.state;
            fprintf("==================================================================\n")
            fprintf("==================================================================\n")
            fprintf("ps: %f %f %f \t vs: %f %f %f \t qs: %f %f %f \n",...
                est_print.p(1), est_print.p(2), est_print.p(3),...
                est_print.v(1), est_print.v(2), est_print.v(3),...
                est_print.q(1)*180/pi, est_print.q(2)*180/pi, est_print.q(3)*180/pi); % s:state 現在状態
            fprintf("pr: %f %f %f \t vr: %f %f %f \t qr: %f %f %f \n", ...
                obj.state.ref(3,1), obj.state.ref(7,1), obj.state.ref(1,1),...
                obj.state.ref(4,1), obj.state.ref(8,1), obj.state.ref(2,1),...
                0, 0, obj.state.ref(11,1)*180/pi)                             % r:reference 目標状態
            fprintf("t: %f \t input: %f %f %f %f", ...
                obj.param.t, obj.result.input(1), obj.result.input(2), obj.result.input(3), obj.result.input(4));
            fprintf("\n");
        end
    end
end

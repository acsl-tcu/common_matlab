classdef KQ_LMPC_CONTROLLER< handle

    properties
        options % QP
        param
        current_state
        input
        state
        const
        reference
        fRemove
        model
        result
        self
        sigma
        tss
    end
    properties
        % よく使うパラメータはobj.○○とする
        modelf
        modelp
        P % drone parameter
        N % 現時刻のパーティクル数
        H % horizon
        weight
        koopman
        qpparam % 二次計画法QPのパラメータ
        previous_input % 前時刻入力
        gen_beq
        removeN
        survive
        removeX
        flag
        reinput
        reEva
        StageStateSTLsum
        STL_period = [2,4]
        quadH
        quadf
        sw
        drf
        act
        m=3
        n=2
    end
    methods
        function obj = KQ_LMPC_CONTROLLER(self, param)
            %-- 変数定義
            obj.self = self; % agent
            obj.param = param; % param = Controller_MPC_HLMC.mで設定したパラメーター
            %% flag defination
            obj.modelf = obj.self.plant.method;
            obj.P = obj.self.parameter.get(); % ドローンのパラメータ（質量，ロータ間距離，慣性モーメントなど）
            obj.N = param.particle_num; % サンプル数
            obj.H = param.H; % ホライズン
            % 重みの配列サイズ変換
            obj.weight = param.weight; % 重みを変数に保存
            obj.weight.stagestate = blkdiag(obj.weight.P, obj.weight.Q, obj.weight.V, obj.weight.W); % blkdiagで配列同士を結合
            obj.weight.terminalstate = blkdiag(obj.weight.Pf, obj.weight.Qf, obj.weight.Vf, obj.weight.Wf);
            obj.weight.input = param.weight.R;  % 目標入力
            obj.weight.preinputdif = param.weight.RP; % 前ステップとの入力
            % 入力の初期化
            obj.result.input = obj.param.ref_input; % 目標入力 初期時刻にresultを定義しておかないと実行時にエラー出る
            obj.input = obj.param.input; %入力関連のみ
            obj.input.pre_u = repmat(obj.result.input,1,obj.H); % 前入力
            obj.input.var = repmat(obj.param.ref_input,obj.H,1);
            obj.result.Bestcost_STL = 0;
            obj.input.sigma = param.input.Initsigma;
            % MPCパラメータ初期化 = メモリの確保
            obj.result.bestx(1, :) = repmat(obj.input.Bestcost_now(1), obj.param.H, 1); % - 制約外は前の評価値を引き継ぐ
            obj.result.besty(1, :) = repmat(obj.input.Bestcost_now(1), obj.param.H, 1); % - 制約外は前の評価値を引き継ぐ
            obj.result.bestz(1, :) = repmat(obj.input.Bestcost_now(1), obj.param.H, 1); % - 制約外は前の評価値を引き継ぐ
            obj.state.state_data = zeros(obj.param.state_size,obj.H, obj.N);
            obj.result.Evaluationtra = zeros(obj.N, 2);
            obj.result.pre_u = obj.input.pre_u;
            % obj.input.mu = param.ref_input;
            % A, B行列定義 z, x, y, yawの順番ベクトル化 speical defination for koopman
            % obj.koopman = param.koopman;
            % C = repmat({obj.koopman.C}, 1, obj.H);
            % obj.koopman.ExC = blkdiag(C{:});
            % [obj.koopman.ExA,obj.koopman.ExB] = ExtendedCoefficientMatrix_kyo({obj.koopman.A,obj.koopman.B,obj.H,param.state_size}); % 一括計算 2025/1/21確認
            %%Koopman予測に基づく拡張行列  　
            obj.koopman.A =zeros(45,45);
            obj.koopman.B =zeros(45,4);
            obj.result.bestcost = obj.input.Bestcost_now;
            obj.koopman.A=obj.get_Koopman_A(obj.m,obj.n);
        end
        %-- main()的な
        function result = do(obj,varargin)
            time = varargin{1};
            phase = varargin{2};
            obj.param.t = time.t;
            obj.current_state = obj.self.estimator.result.state.get(); % 現在状態の取得
            obj.result2input();
            obj.state.current = obj.klift(obj.current_state,obj.m,obj.n);
            obj.state.ref = obj.generate_reference(); % vararginのrefをHorizonに拡張
            result= obj.controller_KMC(varargin);
            disp('controller: KMC,  phase: ');
            disp(phase);
            obj.show();
        end
        function result = controller_KMC(obj,varargin)
            obj.param.t = varargin{1}{1}.t; % 現在時刻
            obj.param.te = varargin{1}{1}.te; % 終了時間(default : 10s)
            obj.koopman.B=obj.get_Koopman_B(obj.current_state,obj.state.current,obj.m,obj.n,obj.param);
            obj.K_MPC();%qp
            result = obj.result;
        end
        function K_MPC(obj)
            [obj.koopman.ExA,obj.koopman.ExB] = obj.ExtendedCoefficientMatrix({obj.koopman.A,obj.koopman.B,obj.H,obj.param.state_size});
            n = size(obj.state.current,1);
            z_current = obj.state.current;
            Xr_vec = zeros(n* obj.param.H, 1);
            for k = 1:obj.H
                z_ref_k = obj.klift(obj.state.ref(1:12, k),obj.m, obj.n);
                Xr_vec((k-1)*n+1 : k*n) = z_ref_k;
            end
            Xr = Xr_vec;
            Ur = reshape(obj.state.ref(13:16, :), [], 1);
            w_vec = zeros(n, 1);
            w_vec(1:3) = diag(obj.weight.P);
            idx_v = 3*obj.m + 1;
            w_vec(idx_v : idx_v+2) = diag(obj.weight.V);
            idx_q = 9*obj.m + 1;
            w_vec(idx_q : idx_q+8) = mean(diag(obj.weight.Q));
            if obj.n >= 2
                idx_w = 9*obj.m + 10;
                w_vec(idx_w : idx_w+8) = mean(diag(obj.weight.W));
            end
            Q_stage = diag(w_vec);
            Q_bar = blkdiag(kron(eye(obj.H-1), Q_stage), Q_stage);
            R_bar  = kron(eye(obj.H), obj.weight.input);
            RP_bar = kron(eye(obj.H), obj.weight.preinputdif);
            Up = repmat(obj.input.pre_u, obj.H, 1);
            [obj.quadH, obj.quadf] = obj.gen_Hf(obj.koopman.ExA, obj.koopman.ExB, z_current, ...
                Q_bar, R_bar, RP_bar, ...
                Xr, Ur, Up);
            A = []; b = [];
            Aeq = []; beq = [];
            lb = repmat(obj.param.input_min,1,obj.param.H);
            ub = repmat(obj.param.input_max,1,obj.param.H);
            obj.options = optimset('Display', 'off');
            [var,fval,eflag,~,~] = quadprog(obj.quadH,obj.quadf,A,b,Aeq,beq,lb,ub,[],obj.options);

            if eflag ~= 1
                disp(['Warning: Quadprog failed to find a solution. eflag = ', num2str(eflag)]);
            end

            obj.result.input =var(1:4, 1); % 算出された入力
            obj.result.eflag = eflag;
            obj.result.var = var;
            obj.result.Bestcost_pre = obj.result.bestcost;
            obj.result.bestcost = [fval;0];
            obj.input.pre_u = obj.result.input;
            obj.result.pre_u = obj.input.pre_u;
            % obj.result.bestcost=obj.input.Bestcost_now ;
            % n = size(obj.state.current,1); % number of observables
            % %qp def
            % Q = blkdiag(kron(eye(obj.param.H-1),blkdiag(obj.weight.stagestate,0*eye(n-12))),blkdiag(obj.weight.terminalstate,0*eye(n-12)));
            % R = kron(eye(obj.param.H),obj.weight.input);
            % RP = kron(eye(obj.param.H),obj.weight.preinputdif);
            % Xr = reshape([obj.state.ref(1:12,:);zeros(n-12,obj.param.H)],[],1);
            % Ur = reshape(obj.state.ref(13:16,:),[],1);
            % [obj.quadH,obj.quadf]=obj.gen_Hf(obj.koopman.ExA,obj.koopman.ExB,obj.state.current,Q,R,RP,Xr,Ur,obj.input.var);
            % A = []; b = [];
            % Aeq = []; beq = [];
            % lb = repmat(obj.param.input_min,1,obj.param.H);
            % ub = repmat(obj.param.input_max,1,obj.param.H);
            % obj.options = optimset('Display', 'off');
            % [var,fval,eflag,~,~] = quadprog(obj.quadH,obj.quadf,A,b,Aeq,beq,lb,ub,[],obj.options);
            % if eflag ~= 1
            %     disp(['Warning: Quadprog failed to find a solution. eflag = ', num2str(eflag)]);
            % end
            % obj.result.input =var(1:4, 1); % 算出された入力
            % obj.result.eflag = eflag;
            % obj.result.var = var;
            % obj.result.Bestcost_pre = obj.result.bestcost;
            % obj.result.bestcost = [fval;0];
            % obj.input.pre_u = obj.result.input;
            % obj.result.pre_u = obj.input.pre_u;

        end
        function [H,f] = gen_Hf(obj,A,B,x0,Q,R,Rp,Xr,Ur,Up)
            % calc H and f matrices for quadprog
            % x0: current state
            % Xn = A*x0+B*U % prediction in horizon
            % dX = Xn-Xr
            % dX'*Q*dX = U'*B'*Q*B*U + 2(A*x0-Xr)'*Q*B*U + (x0'*x0 term)
            % dU = U - Ur
            % dU'*R*dU = U'*R*U - 2*Ur*R*U
            % dpU = U - Up
            % U'*Rp*U - 2*Up*Rp*U
            H = 2*(B'*Q*B+R+Rp);
            H = (H+H')/2;
            f = (2*(A*x0 - Xr)'*Q*B - 2*Ur'*R - 2*Up'*Rp)';

        end
        function result2input(obj)
            obj.result.pre_u = obj.input.u;
            obj.input.pre_u = obj.result.pre_u;
        end
        function calB = get_Koopman_B(obj,x,xlift,M,N,params)
            p1 = xlift(1:3);
            y1 = xlift(3*M+1 : 3*M+3);
            h1 = xlift(6*M+1 : 6*M+3);
            w = x(10:12);
            Q = x(4:6);
            c = cos(Q); s = sin(Q);
            R = [ c(2)*c(3), s(1)*s(2)*c(3)-c(1)*s(3),c(1)*s(2)*c(3)+s(1)*s(3);
                c(2)*s(3),  s(1)*s(2)*s(3)+c(1)*c(3),  c(1)*s(2)*s(3)-s(1)*c(3);
                -s(2),      s(1)*c(2),  c(1)*c(2)];
            e3 = [0;0;1];
            Omega = [0 -w(3) w(2); w(3) 0 -w(1); -w(2) w(1) 0];
            OmegaT = Omega';
            invJ = inv(diag([params.jx params.jy params.jz]));
            Hk = obj.get_HYP_Block(h1, M, OmegaT, invJ);
            Yk = obj.get_HYP_Block(y1, M, OmegaT, invJ);
            Pk = obj.get_HYP_Block(p1, M, OmegaT, invJ);
            calB = zeros(9*M + 9*N, 4);
            for j = 1:M
                thrust_vec = (1/params.m) * (OmegaT^(j-1)) * e3;
                idx_p = 3*j-2;
                idx_y = 3*M + 3*j-2;
                idx_h = 6*M + 3*j-2;
                calB(idx_p : idx_p+2, 2:4) = Hk(:,:,j);
                calB(idx_y : idx_y+2, 1)   = thrust_vec;
                calB(idx_y : idx_y+2, 2:4) = Yk(:,:,j);
                calB(idx_h : idx_h+2, 2:4) = Pk(:,:,j);
            end
            start_row = 9*M;
            for ii = 1:(N-1)
                C_ii_flat = obj.get_SO3_Block(ii, R, Omega, invJ);
                row_idx = start_row + 9*ii + 1;
                calB(row_idx : row_idx+8, 2:4) = C_ii_flat;
            end
        end
        function HYP = get_HYP_Block(obj,vec, M, OmegaT, invJ)
            HYP = zeros(3,3,M);
            for k = 2:M
                temp = zeros(3,3);
                for i = 1:(k-1)
                    term_skew = obj.skew_func((OmegaT^(k-1-i))*vec);
                    temp = temp + (OmegaT^(i-1)) * term_skew * invJ;
                end
                HYP(:,:,k) = temp;
            end
        end

        function C_out = get_SO3_Block(obj,ii, R, Omega, invJ)
            c1=zeros(3,3); c2=zeros(3,3); c3=zeros(3,3);
            for jj = 1:ii
                termA = R * (Omega^(jj-1));
                termC = Omega^(ii-jj);
                c1 = c1 + termA * obj.skew_func(invJ(:,1)) * termC;
                c2 = c2 + termA * obj.skew_func(invJ(:,2)) * termC;
                c3 = c3 + termA * obj.skew_func(invJ(:,3)) * termC;
            end
            C_out = [c1(:), c2(:), c3(:)];
        end

        function S = skew_func(obj,v)
            S = [0 -v(3) v(2); v(3) 0 -v(1); -v(2) v(1) 0];
        end
        function X_lifted=klift(obj,x,m,n)
            p=x(1:3);
            q=x(4:6);
            v=x(7:9);
            w=x(10:12);
            R = eul2rotm([q(3), q(2), q(1)], 'ZYX');
            gvec = [0; 0; obj.param.gravity];
            wx = obj.skew_func(w);
            ps = [];
            ys = [];
            hs = [];
            for i = 1:m
                term = (wx')^(i-1);
                h_vec = -term * R' * gvec;
                hs = [hs; h_vec];
                y_vec = term * R' * v;
                ys = [ys; y_vec];
                p_vec = term * R' * p;
                ps = [ps; p_vec];
            end
            z_rot = [];
            for k = 1:n
                z_mat = R * (wx)^(k-1);
                z_rot = [z_rot; z_mat(:)];
            end
            X_lifted = [ps; ys; hs; z_rot];
        end
        function A = get_Koopman_A(obj,M,N)
            Ap = zeros(9*M);
            Ap(1:3*(M-1), 4:3*M) = eye(3*(M-1));
            Ap(1:3*(M-1), 3*M+1:6*M-3) = eye(3*(M-1));
            Ap(3*M+1:6*M-3, 3*M+4:6*M) = eye(3*(M-1));
            Ap(3*M+1:6*M-3, 6*M+1:9*M-3) = eye(3*(M-1));
            Ap(6*M+1:9*M-3, 6*M+4:9*M) = eye(3*(M-1));
            A_so3 = zeros(9*N, 9*N);
            for i = 1:N
                if i < N
                    A_so3(9*i-8:9*i, 9*i+1:9*i+9) = eye(9);
                end
            end
            A = zeros(9*M + 9*N);
            A(1:9*M, 1:9*M) = Ap;
            A(9*M+1:end, 9*M+1:end) = A_so3;

        end
        %% 目標軌道生成
        function [xr] = generate_reference(obj)
            xr = zeros(obj.param.total_size, obj.H);    % initialize
            RefTime = obj.self.reference.time_var.func; % 時間関数の取得
            for h = 0:obj.param.H-1
                t = obj.param.t + obj.param.dt * h; % reference生成の時刻をずらす
                ref = RefTime(t);
                xr(1:3, h+1) = ref(1:3);
                xr(7:9, h+1) = ref(5:7);
                xr(4:6, h+1) =   [0;0;0]; % 姿勢角
                xr(10:12, h+1) = [0;0;0];
                xr(13:16, h+1) = obj.param.ref_input; % MC -> 0.6597,   HL -> 0
            end
            g = 9.81;
            prev_euler = zeros(3,1);
            for h = 0:obj.H-1
                t   = obj.param.t + obj.param.dt * h;    % reference生成の時刻をずらす
                ref = RefTime(t);                        % 20x1
                acc = ref(9:11);                         % ddx, ddy, ddz
                yaw = ref(4);                            % yaw
                s  = acc + [0;0;g];
                b3 = s / norm(s);
                b1c = [cos(yaw); sin(yaw); 0];
                v = cross(b3, b1c);
                if norm(v) < 1e-6
                    if abs(b3(3))<0.9, b1=[0;0;1]; else, b1=[1;0;0]; end
                    b2 = cross(b3,b1); b2=b2/norm(b2); b1=cross(b2,b3);
                else
                    b2 = v/norm(v);  b1 = cross(b2,b3);
                end
                Rd = [b1,b2,b3];
                phi   = atan2(Rd(3,2), Rd(3,3));
                theta = asin(-Rd(3,1));
                psi   = atan2(Rd(2,1), Rd(1,1));
                euler = [phi;theta;psi];
                if h == 0 && obj.H > 1
                    t_next   = t + obj.param.dt;
                    ref_next = RefTime(t_next);
                    acc_n    = ref_next(9:11);
                    yaw_n    = ref_next(4);
                    s_n  = acc_n + [0;0;g];
                    b3_n = s_n / norm(s_n);
                    b1c_n = [cos(yaw_n); sin(yaw_n); 0];
                    v_n = cross(b3_n, b1c_n);
                    if norm(v_n) < 1e-6
                        if abs(b3_n(3))<0.9, b1_n=[0;0;1]; else, b1_n=[1;0;0]; end
                        b2_n = cross(b3_n,b1_n); b2_n=b2_n/norm(b2_n); b1_n=cross(b2_n,b3_n);
                    else
                        b2_n = v_n/norm(v_n);  b1_n = cross(b2_n,b3_n);
                    end
                    Rd_n = [b1_n,b2_n,b3_n];
                    phi_n   = atan2(Rd_n(3,2), Rd_n(3,3));
                    theta_n = asin(-Rd_n(3,1));
                    psi_n   = atan2(Rd_n(2,1), Rd_n(1,1));
                    euler_n = [phi_n;theta_n;psi_n];
                    euler_dot = (euler_n - euler) / obj.param.dt;
                    euler_dot(3) = ref(8);
                elseif h == 0 && obj.H == 1
                    euler_dot = [0;0;ref(8)];
                else
                    euler_dot = (euler - prev_euler) / obj.param.dt;
                    euler_dot(3) = ref(8);
                end
                prev_euler = euler;
                T = [ 1, 0, -sin(theta);
                    0, cos(phi),  cos(theta)*sin(phi);
                    0, -sin(phi), cos(theta)*cos(phi) ];
                w = T * euler_dot;
                xr(1:3,   h+1) = ref(1:3);
                xr(7:9,   h+1) = ref(5:7);
                xr(4:6,   h+1) = euler;
                xr(10:12, h+1) = w;
                xr(13:16, h+1) = obj.result.input(:,1);
            end
        end

        function show(obj)
            % clc;
            % est_print = obj.self.estimator.result.state;
            est_print = obj.self.estimator.result.state;
            fprintf("==================================================================\n")
            fprintf("==================================================================\n")
            fprintf("ps: %f %f %f \t vs: %f %f %f \t qs: %f %f %f \n",...
                est_print.p(1), est_print.p(2), est_print.p(3),...
                est_print.v(1), est_print.v(2), est_print.v(3),...
                est_print.q(1), est_print.q(2), est_print.q(3)); % s:state 現在状態
            fprintf("pr: %f %f %f \t vr: %f %f %f \t qr: %f %f %f \n", ...
                obj.state.ref(1,1), obj.state.ref(2,1), obj.state.ref(3,1),...
                obj.state.ref(7,1), obj.state.ref(8,1), obj.state.ref(9,1),...
                0, 0, obj.state.ref(6,1))                             % r:reference 目標状態
            fprintf("t: %f \t input: %f %f %f %f \t J: %f \t sigma: %f", ...
                obj.param.t, obj.result.input(1), obj.result.input(2), obj.result.input(3), obj.result.input(4), obj.result.bestcost(1),obj.input.sigma(1));
            fprintf("\n");
        end
        function [ExA,ExB] = ExtendedCoefficientMatrix(obj,Param)
    % ECM:Extended Coeifficient Matrix
    A = Param{1};
    B = Param{2};
    
    Horizon = Param{3};
    Xnum = Param{4};

    S = zeros(Horizon*Xnum, Horizon*length(B(1,:)));

    % ホライズンの値によらない
    % A行列
    Am = [];
    for i = 1:Horizon
        Am = [Am; A^i]; %A
    end
    % B行列
    for i  = 1:Horizon
        for j = 1:Horizon
            if j <= i
                S(1+length(B(:,1))*(i-1):length(B(:,1))*i,1+length(B(1,:))*(j-1):length(B(1,:))*j) = A^(i-j)*B;
            end
        end
    end
    ExA = Am;
    ExB = S;

end
    end
end

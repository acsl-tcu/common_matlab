classdef HL_MPC < handle
    % Hierarchical linearization based controller for a quadcopter
    properties
        self
        result
        param
        Vf
        Vs
        parameter_name = ["mass","Lx","Ly","lx","ly","jx","jy","jz","gravity","km1","km2","km3","km4","k1","k2","k3","k4"];
    end

    methods
        function obj = HL_MPC(self,param)
            obj.self = self;
            obj.param = param;
            obj.param.P = self.parameter.get(obj.parameter_name);
            obj.result.input = zeros(self.estimator.model.dim(2),1);
            obj.Vf = obj.param.Vf; % 階層１の入力を生成する関数ハンドル
            obj.Vs = obj.param.Vs; % 階層２の入力を生成する関数ハンドル
        end

        function result = do(obj,varargin)
            tic
            model = obj.self.estimator.result;
            xd = obj.self.reference.result.state.xd;
            P = obj.param.P;

            %% expand reference: single xd -> 20 x H
            xd = xd(:);
            xd = [xd; zeros(max(0, 20 - size(xd, 1)), 1)];
            xd = repmat(xd, 1, obj.param.H);

            xd0 = xd(:, 1);

            %% yaw coordinate transform
            Rb0 = RodriguesQuaternion(Eul2Quat([0; 0; xd0(4)]));

            x = [
                R2q(Rb0' * model.state.getq("rotmat"));
                Rb0' * model.state.p;
                Rb0' * model.state.v;
                model.state.w
                ];

            for k = 1:size(xd, 2)
                xd(1:3, k)   = Rb0' * xd(1:3, k);
                xd(5:7, k)   = Rb0' * xd(5:7, k);
                xd(9:11, k)  = Rb0' * xd(9:11, k);
                xd(13:15, k) = Rb0' * xd(13:15, k);
                xd(17:19, k) = Rb0' * xd(17:19, k);
            end

            xd(4, :) = mod(xd(4, :) - xd0(4) + pi, 2*pi) - pi;
            xd(4, 1) = 0;
            xd0 = xd(:, 1);

            %% quadprog option
            if isfield(obj.param.mpc, "opt")
                opt = obj.param.mpc.opt;
            else
                opt = optimoptions('quadprog', 'Display', 'off');
            end

            %% calc z1 and vf by QP-MPC
            z1 = Z1(x, xd0', P);

            Sx = obj.param.mpc.Sx{1};
            Su = obj.param.mpc.Su{1};
            Qb = obj.param.mpc.Qb{1};
            Hq = obj.param.mpc.Hq{1};
            Hq = (Hq + Hq') / 2;

            f = 2 * Su' * Qb * Sx * z1;

            V = quadprog( ...
                Hq, f, [], [], [], [], ...
                obj.param.mpc.lbq{1}, obj.param.mpc.ubq{1}, [], opt);

            if isempty(V)
                V = zeros(obj.param.mpc.Nvf, 1);
            end

            vf = V(1:obj.param.mpc.Nvf)';   % 1 x 4

            %% calc Z2 Z3 Z4
            z2 = Z2(x, xd0', vf, P);
            z3 = Z3(x, xd0', vf, P);
            z4 = Z4(x, xd0', vf, P);

            %% calc vs by QP-MPC
            Zs = {z2, z3, z4};
            vs = zeros(3, 1);

            for s = 1:3
                id = s + 1;

                Sx = obj.param.mpc.Sx{id};
                Su = obj.param.mpc.Su{id};
                Qb = obj.param.mpc.Qb{id};
                Hq = obj.param.mpc.Hq{id};
                Hq = (Hq + Hq') / 2;

                f = 2 * Su' * Qb * Sx * Zs{s};

                V = quadprog( ...
                    Hq, f, [], [], [], [], ...
                    obj.param.mpc.lbq{id}, obj.param.mpc.ubq{id}, [], opt);

                if isempty(V)
                    V = zeros(obj.param.mpc.N, 1);
                end

                vs(s) = V(1);
            end

            %% calc actual input
            tmp = Uf(x, xd0', vf, P) + Us(x, xd0', vf, vs, P);

            %% result
            obj.result.uHL = [vf(1); vs];
            obj.result.vf = vf;
            obj.result.z1 = z1;
            obj.result.z2 = z2;
            obj.result.z3 = z3;
            obj.result.z4 = z4;

            obj.result.input = [
                max(0,  min(10, tmp(1)));
                max(-1, min(1,  tmp(2)));
                max(-1, min(1,  tmp(3)));
                max(-1, min(1,  tmp(4)))
                ];

            result = obj.result;
            obj.show();
toc
        end
        function show(obj)
            % clc;
            % est_print = obj.self.estimator.result.state;
            est_print = obj.self.estimator.result.state;
            ref_print =obj.self.reference.result.state;
            fprintf("==================================================================\n")
            fprintf("==================================================================\n")
            fprintf("ps: %f %f %f \t vs: %f %f %f \t qs: %f %f %f \n",...
                est_print.p(1), est_print.p(2), est_print.p(3),...
                est_print.v(1), est_print.v(2), est_print.v(3),...
                est_print.q(1), est_print.q(2), est_print.q(3)); % s:state 現在状態
            fprintf("pr: %f %f %f \t vr: %f %f %f \t qr: %f %f %f \n", ...
                ref_print.p(1), ref_print.p(2), ref_print.p(3),...
                ref_print.v(1), ref_print.v(2), ref_print.v(3),...
                ref_print.xd(4), ref_print.xd(5), ref_print.xd(6)); % r:reference 目標状態

        end
    end
end

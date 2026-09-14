classdef HL_MPC < handle
    % Hierarchical linearization based controller for a quadcopter
    properties
        self
        result
        param
        Vf
        Vs
        mec          % EDMD-MEC 模型
        du_prev = zeros(4,1);
        use_mec = 0;   % 想关掉补偿就设 false
        parameter_name = ["mass","Lx","Ly","lx","ly","jx","jy","jz","gravity","km1","km2","km3","km4","k1","k2","k3","k4"];
    end

    methods
        function obj = HL_MPC(self,param)
            obj.self = self;
            obj.param = param;
            obj.param.P = self.parameter.get(obj.parameter_name);
            obj.result.input = zeros(self.estimator.model.dim(2),1);
            obj.result.prev_vHL = zeros(4,1);
            obj.Vf = obj.param.Vf; % 階層１の入力を生成する関数ハンドル
            obj.Vs = obj.param.Vs; % 階層２の入力を生成する関数ハンドル
            if obj.use_mec
                L = load('mec_model_HLMPC.mat'); obj.mec = L.model;
            end
        end

        function result = do(obj,varargin)
           
            model = obj.self.estimator.result;
            P = obj.param.P;
             phase = varargin{2};
            %% reference horizon for MPC
            H = obj.param.H;
            dt = obj.param.dt;
            pad_xd = @(x) [x(:); zeros(max(0, 20 - numel(x)), 1)];

            xd0 = pad_xd(obj.self.reference.result.state.xd);
            xd_world = zeros(20, H);
            ref_obj = [];

            try
                ref_name = obj.self.cha_allocation.(char(phase)).reference;
                if ~isempty(ref_name)
                    ref_obj = obj.self.reference.(char(ref_name(1)));
                end
            catch
            end

            for j = 1:H
                tau = j * dt;
                xd_j = [];
                try
                    if ~isempty(ref_obj) && ismethod(ref_obj, "gen_ref_for_take_off")
                        xd_j = ref_obj.gen_ref_for_take_off(varargin{1}.t - ref_obj.base_time + tau);
                    elseif ~isempty(ref_obj) && ismethod(ref_obj, "gen_ref_for_landing")
                        xd_j = ref_obj.gen_ref_for_landing(varargin{1}.t - ref_obj.base_time + tau);
                    elseif ~isempty(ref_obj) && isprop(ref_obj, "func") && isprop(ref_obj, "t") && ~isempty(ref_obj.t)
                        xd_j = ref_obj.func(varargin{1}.t - ref_obj.t + tau);
                    end
                catch
                    xd_j = [];
                end

                if isempty(xd_j)
                    xd_j = xd0;
                    xd_j(1:4) = xd0(1:4) + xd0(5:8) * tau + xd0(9:12) * tau^2 / 2 + xd0(13:16) * tau^3 / 6 + xd0(17:20) * tau^4 / 24;
                    xd_j(5:8) = xd0(5:8) + xd0(9:12) * tau + xd0(13:16) * tau^2 / 2 + xd0(17:20) * tau^3 / 6;
                    xd_j(9:12) = xd0(9:12) + xd0(13:16) * tau + xd0(17:20) * tau^2 / 2;
                    xd_j(13:16) = xd0(13:16) + xd0(17:20) * tau;
                end

                xd_world(:, j) = pad_xd(xd_j);
            end

            xd = xd_world;

            %% yaw coordinate transform
            Rb0 = RodriguesQuaternion(Eul2Quat([0; 0; xd0(4)]));

            x = [
                R2q(Rb0' * model.state.getq("rotmat"));
                Rb0' * model.state.p;
                Rb0' * model.state.v;
                model.state.w
                ];

            xd0(1:3)   = Rb0' * xd0(1:3);
            xd0(5:7)   = Rb0' * xd0(5:7);
            xd0(9:11)  = Rb0' * xd0(9:11);
            xd0(13:15) = Rb0' * xd0(13:15);
            xd0(17:19) = Rb0' * xd0(17:19);

            for k = 1:size(xd, 2)
                xd(1:3, k)   = Rb0' * xd(1:3, k);
                xd(5:7, k)   = Rb0' * xd(5:7, k);
                xd(9:11, k)  = Rb0' * xd(9:11, k);
                xd(13:15, k) = Rb0' * xd(13:15, k);
                xd(17:19, k) = Rb0' * xd(17:19, k);
            end

            yaw0 = xd0(4);
            xd0(4) = 0;
            xd(4, :) = mod(xd(4, :) - yaw0 + pi, 2*pi) - pi;
            % fprintf('controller: HLMPC,  phase: %s \n',phase);
            % disp(obj.self.reference.result.state.p);
            %% quadprog option
            if isfield(obj.param.mpc, "opt")
                opt = obj.param.mpc.opt;
            else
                opt = optimoptions('quadprog', 'Display', 'off');
            end
            if isfield(obj.result, "prev_vHL")|| numel(obj.result.prev_vHL)~=4
                obj.result.prev_vHL = zeros(4,1);
            end
            if isfield(obj.param.mpc, "delay_step")
                delay_step = max(0,round(obj.param.mpc.delay_step));
            else
                delay_step = 0;
            end
            %% calc z1 and vf by QP-MPC
            z1 = Z1(x, xd0', P);

            Sx = obj.param.mpc.Sx{1};
            Su = obj.param.mpc.Su{1};
            Qb = obj.param.mpc.Qb{1};
            Hq = obj.param.mpc.Hq{1};
            Hq = (Hq + Hq') / 2;
      
            f = 2 * Su' * Qb * Sx * z1;
            Rd0 = obj.param.mpc.Rd0{1};
            Hq(1,1)=Hq(1,1)+2*Rd0;
            f(1) =f(1)-2*Rd0*obj.result.prev_vHL(1);

            V = quadprog( ...
                Hq, f, [], [], [], [], ...
                obj.param.mpc.lbq{1}, obj.param.mpc.ubq{1}, [], opt);

            if isempty(V)
                V = zeros(obj.param.mpc.Nvf, 1);
            end

            vf = V(1:obj.param.mpc.Nvf)';   % 1 x 4
            Vf_seq = V(:);
            z1_pred = reshape(Sx * z1 + Su * Vf_seq, 2, []);
            %% calc Z2 Z3 Z4
            z2 = Z2(x, xd0', vf, P);
            z3 = Z3(x, xd0', vf, P);
            z4 = Z4(x, xd0', vf, P);

            %% calc vs by QP-MPC
            Zs = {z2, z3, z4};
            vs = zeros(3, 1);
            Vs_seq = cell(3, 1);
            Zs_pred = cell(3, 1);
            for s = 1:3
                id = s + 1;

                Sx = obj.param.mpc.Sx{id};
                Su = obj.param.mpc.Su{id};
                Qb = obj.param.mpc.Qb{id};
                Hq = obj.param.mpc.Hq{id};
                Hq = (Hq + Hq') / 2;

                f = 2 * Su' * Qb * Sx * Zs{s};
                Rd0 = obj.param.mpc.Rd0{id};
                Hq(1,1)=Hq(1,1)+2*Rd0;
                f(1) =f(1)-2*Rd0*obj.result.prev_vHL(id);
                V = quadprog( ...
                    Hq, f, [], [], [], [], ...
                    obj.param.mpc.lbq{id}, obj.param.mpc.ubq{id}, [], opt);
                if isempty(V)
                    V = zeros(obj.param.mpc.N, 1);
                end
                Vs_seq{s} = V(:);
                nx = size(obj.param.mpc.A{id}, 1);
                Zs_pred{s} = reshape(Sx * Zs{s} + Su * Vs_seq{s}, nx, []);
               apply_id = min(1 + delay_step,numel(Vs_seq{s}));
               vs(s) = Vs_seq{s}(apply_id);
            end

            %% calc actual input
          
            tmp = Uf(x, xd0', vf, P) + Us(x, xd0', vf, vs, P);
            % ===== EDMD-MEC residual compensation =====
            u_nominal = tmp;
            du = zeros(4,1);

            if obj.use_mec && strcmp(char(phase), 'f')
                est = obj.self.estimator.result.state;

                g = 9.81;
                Rk = est.getq("rotmat");
                Re3_k = Rk(:,3);

                zk = build_phi(est.p(:), est.q(:), est.v(:), est.w(:), Re3_k);

                % HLMPC already predicts the reference horizon.
                % Use the first predicted reference as the one-step EDMD target.
                xd_ref_now = xd_world(:,1);
                xd_ref_now = [xd_ref_now; zeros(max(0, 20 - numel(xd_ref_now)), 1)];

                pref = xd_ref_now(1:3);
                vref = xd_ref_now(5:7);
                aref = xd_ref_now(9:11);

                if norm(vref(1:2)) < 1e-6
                    yawr = xd_ref_now(4);
                else
                    yawr = atan2(vref(2), vref(1));
                end

                acmd = aref + [0;0;g];
                if norm(acmd) < 1e-6
                    zb = [0;0;1];
                else
                    zb = acmd / norm(acmd);
                end

                xc = [cos(yawr); sin(yawr); 0];
                yb = cross(zb, xc);
                yb = yb / max(norm(yb), 1e-6);
                xb = cross(yb, zb);

                Rr = [xb, yb, zb];
                Re3r = zb;
                eulr = obj.R2eul_local(Rr);

                zref = build_phi(pref, eulr, vref, est.w(:), Re3r);

                rk     = zref - obj.mec.Ar * zk - obj.mec.Br * u_nominal;
                du_raw = obj.mec.M * rk;

                du = (1 - obj.mec.beta) * obj.du_prev + obj.mec.beta * ...
                    max(min(du_raw, obj.mec.dumax), -obj.mec.dumax);

                obj.du_prev = du;
                tmp = u_nominal + du;
            end

            obj.result.u_nominal = u_nominal;
            obj.result.input_nominal = u_nominal;
            obj.result.delta_u_mec = du;
            obj.result.delta_u_edmd = du;
% ========================================

            %% result
            obj.result.uHL = [vf(1); vs];
            obj.result.prev_vHL =[vf(1);vs];
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
            obj.result.hlmpc = obj.result.input;
            obj.result.pre_u = obj.result.input;
            Npred = min([ ...
                size(z1_pred, 2), ...
                size(Zs_pred{1}, 2), ...
                size(Zs_pred{2}, 2) ...
                ]);

            pred_local = [
                xd(1, 1:Npred) + Zs_pred{1}(1, 1:Npred);
                xd(2, 1:Npred) + Zs_pred{2}(1, 1:Npred);
                xd(3, 1:Npred) + z1_pred(1, 1:Npred)
                ];

            pred_world = Rb0 * pred_local;

            obj.result.hlmpc_pred_pos = pred_world;   % 3 x Npred，用于动画
            obj.result.hlmpc_pred_local = pred_local;
            obj.result.hlmpc_ref_pos = xd_world(1:3, 1:Npred);
            obj.result.hlmpc_pred_z = z1_pred;
            obj.result.hlmpc_pred_xy = Zs_pred;
            obj.result.hlmpc_Vf_seq = Vf_seq;
            obj.result.hlmpc_Vs_seq = Vs_seq;
            result = obj.result;
            % obj.show();

        end
        function e = R2eul_local(~, R)
            % ZYX -> [roll; pitch; yaw]
            e = [ ...
                atan2(R(3,2), R(3,3)); ...
                asin(max(min(-R(3,1), 1), -1)); ...
                atan2(R(2,1), R(1,1)) ];
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

classdef KQ_LMPC_EDMD_CONTROLLER< handle

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
        Q
        R
        A_d
        last_u_ff
        integral_error
       exec_counter
       residual
       residual_ref_state
    end
    methods (Static)
        function [Ad, Bd] = c2d_rk4(Ac, Bc, dt, damp_factor)
            if nargin < 4
                damp_factor = 1.0;
            end
            [n, ~] = size(Ac);
            I = eye(n);
            M = Ac * dt;
            M2 = M * M;      % M^2
            M3 = M2 * M;     % M^3
            M4 = M3 * M;     % M^4
            Ad_raw = I + M + (1/2)*M2 + (1/6)*M3 + (1/24)*M4;
            Ad = damp_factor * Ad_raw;
            B_int = I + (1/2)*M + (1/6)*M2 + (1/24)*M3;
            Bd = B_int * Bc * dt;
        end
    end
    methods
        function obj = KQ_LMPC_EDMD_CONTROLLER(self, param)
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
            sc = 0.01;
            px = [1, 1, 1];
            vx = [1, 1, 1];
            w_p = [800, 500, 0];
            w_v = [300, 300, 0];
            w_g = 0* ones(1, obj.m);
            w_z = [300, 200];
            q_vec = [ kron(w_p, px), ...
                kron(w_v, vx), ...
                kron(w_g, ones(1,3)), ...
                kron(w_z, ones(1,9)) ] * sc;
            obj.Q = diag(q_vec);
            obj.R = diag([10; 100; 100; 100]);
            % obj.Q = eye(45);
            % scale = 0.01;
            % obj.Q(1:3, 1:3) =500 * eye(3) * scale;%p
            % obj.Q(4:6, 4:6) = 500 * eye(3) * scale;%p＾2
            % obj.Q(7:9, 7:9) = 0 * eye(3) * scale;%p＾3
            % obj.Q(10:12, 10:12) = 300 * eye(3) * scale;%v
            % obj.Q(13:15, 13:15) = 300 * eye(3) * scale;%v＾2
            % obj.Q(16:18, 16:18) = 0 * eye(3) * scale;%v＾3
            % obj.Q(19:27, 19:27) = 0.1 * eye(9) * scale;  %g
            % obj.Q(28:36, 28:36) = 300 * eye(9) * scale;%q
            % obj.Q(37:45, 37:45) = 200 * eye(9) * scale;%w
            % obj.R = diag([2; 100; 100; 100]);
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
            obj.residual = obj.initialize_residual_model(param);
            obj.integral_error = zeros(4,1);
            obj.residual_ref_state = [];
            
        end
        %-- main()的な
        function result = do(obj,varargin)
            tic
            time = varargin{1};
            phase = varargin{2};
            obj.param.t = time.t;
            obj.current_state = obj.self.estimator.result.state.get(); % 現在状態の取得
            obj.state.current = obj.klift(obj.current_state,obj.m,obj.n);
            obj.state.ref = obj.generate_reference(); % vararginのrefをHorizonに拡張
            obj.param.tau = obj.param.tau + obj.param.dt;
            result= obj.controller_KMC(varargin);
            disp('controller: KqlMpC,  phase: ');
            disp(phase);
            obj.show();
          
            toc
        end
        function result = controller_KMC(obj,varargin)
            persistent firstRun
            if isempty(firstRun)
                firstRun = true;
                result = obj.result;
                return
            end
            obj.param.t  = varargin{1}{1}.t;
            obj.param.te = varargin{1}{1}.te;
            obj.koopman.B = obj.get_Koopman_B(obj.current_state,obj.state.current,obj.m,obj.n,obj.param);
            % obj.K_LQR();
            obj.K_MPC();
            result = obj.result;
        end

        function K_LQR(obj)

            n = size(obj.state.current,1);
            [A_d, B_d] = KQ_LMPC_EDMD_CONTROLLER.c2d_rk4(obj.koopman.A, obj.koopman.B, obj.param.dt,0.99);
            % disp(norm(A_d),norm(B_d),max(abs(B_d(:))));
            % disp(B_d(28:36,2:4));
            [K, ~, ~] = dlqr(A_d,  B_d, obj.Q, obj.R);
            z_err =  obj.state.current - obj.klift(obj.state.ref(1:12, 1),obj.m, obj.n);
            %%
            % % 最大誤差設定
            % pos_err_limit = 1;
            % vel_err_limit = 2;
            % pos_err = z_err(1:3);
            % pos_norm = norm(pos_err);
            % if pos_norm > pos_err_limit
            %     pos_err = pos_err / pos_norm * pos_err_limit;
            % end
            % idx_v = 10;
            % vel_err = z_err(idx_v : idx_v+2);
            % vel_norm = norm(vel_err);
            % if vel_norm > vel_err_limit
            %      vel_err = vel_err / vel_norm * vel_err_limit;
            % end
            % z_err_safe = z_err;
            % z_err_safe(1:3) = pos_err;
            % z_err_safe(idx_v : idx_v+2) = vel_err;
            % u_feedback= -K * z_err_safe;
            u_feedback = -K * z_err;
            %%
            % 角度情報含む推力
            % ref_roll  = obj.state.ref(4, 1);
            % ref_pitch = obj.state.ref(5, 1);
            % cos_factor = max(0.5, cos(ref_roll) * cos(ref_pitch));
            % ideal_thrust = (obj.param.m * 9.81) / cos_factor;
            % u_ff = [ideal_thrust; 0; 0; 0];
            %%
            u_ff = [obj.param.m * obj.param.gravity; 0; 0; 0];
            % u_ff = obj.state.ref(13:16, 1);
            delta = [1.5; 1; 1; 1];
            %%
            %%robust filter
            % alpha = 0.1;
            % if isempty(obj.last_u_ff), obj.last_u_ff = u_ff; end
            % u_ff_smooth = (1-alpha)*obj.last_u_ff + alpha*u_ff;
            % obj.last_u_ff = u_ff_smooth;
            % obj.result.kqlmpc = u_feedback + u_ff_smooth;
            %%
            obj.result.kqlmpc = u_feedback + u_ff;
            obj.result.input = min(max(obj.result.kqlmpc, u_ff - delta), u_ff + delta);
            obj.result.kqlmpc = obj.result.input;
            obj.input.pre_u = obj.result.input;
            obj.result.pre_u = obj.input.pre_u;
        end
        function K_MPC(obj)
            % B_d = obj.koopman.B * obj.param.dt;
            % sys_c = ss(obj.koopman.A, obj.koopman.B, [], []);
            % sys_d = c2d(sys_c, obj.param.dt, 'zoh');
            % A_d = sys_d.A;
            % B_d = sys_d.B;
            [A_d, B_d] = KQ_LMPC_EDMD_CONTROLLER.c2d_rk4(obj.koopman.A, obj.koopman.B, obj.param.dt);
            % obj.koopman_analysis(A_d, B_d);
            % disp(norm(A_d));
            % disp(norm(B_d));
            % disp(max(abs(B_d(:))));
            % disp(B_d(28:36,2:4));
            [obj.koopman.ExA,obj.koopman.ExB] = obj.ExtendedCoefficientMatrix({A_d,B_d,obj.H,obj.param.state_size});
            % [obj.koopman.ExA,obj.koopman.ExB] = obj.ExtendedCoefficientMatrix({obj.A_d,B_d,obj.H,obj.param.state_size});
            n = size(obj.state.current,1);
            q_curr = obj.current_state(4:6); 
            split = 9 * obj.m;               
            z_current = obj.state.current;
            Xr_vec = zeros(n* obj.param.H, 1);
            xref_residual = zeros(12, obj.H);
            for k = 1:obj.H
               xref = obj.state.ref(1:12, k);
                q_ref = xref(4:6); 
                ratio = (k-1) / (obj.H-1); 
                q_mix = q_curr * (1 - ratio) + q_ref * ratio;
                xref_ali = xref; 
                xref_ali(4:6) = q_mix; 
                xref_residual(:, k) = xref_ali;
                z_pos = obj.klift(xref_ali, obj.m, obj.n);
                z_att = obj.klift(xref, obj.m, obj.n);
                Xr_vec((k-1)*n+1 : k*n) = [z_pos(1:split); z_att(split+1:end)];
            end
            obj.residual_ref_state = xref_residual;
            Xr = Xr_vec;
            % Ki_z   = 5.0;  
            % Ki_yaw = 0.1;             
            % if isempty(obj.integral_error)
            %     obj.integral_error = zeros(4,1); % [x; y; z; yaw]
            % end
            % current_pos_yaw = [obj.current_state(1:3); obj.current_state(6)]; 
            % ref_pos_yaw     = [obj.state.ref(1:3, 1); obj.state.ref(6, 1)];
            % error_inst      = current_pos_yaw - ref_pos_yaw;          
            % if abs(error_inst(3)) > 0.005
            %     obj.integral_error(3) = obj.integral_error(3) + error_inst(3) * obj.param.dt;
            % end
            % obj.integral_error(4) = obj.integral_error(4) + error_inst(4) * obj.param.dt;
            % int_limit = [0; 0; 2.0; 0.5]; 
            % obj.integral_error = max(min(obj.integral_error, int_limit), -int_limit);
            % U_comp_single = zeros(4,1);
            % U_comp_single(1) = -Ki_z * obj.integral_error(3); 
            % U_comp_single(4) = -Ki_yaw * obj.integral_error(4);
            % Ur_vec = reshape(obj.state.ref(13:16, :), [], 1);
            % U_int_seq = repmat(U_comp_single, obj.H, 1);
            % Ur = Ur_vec + U_int_seq;
            Ur = reshape(obj.state.ref(13:16, :), [], 1);
            w_vec = zeros(n, 1);
            if obj.m >= 1
                idx_p1 = 1;
                w_vec(idx_p1 : idx_p1+2) = diag(obj.weight.P)*1;
            end
            if obj.m >= 2
                idx_p2 = 4;

                w_vec(idx_p2 : idx_p2+2) = diag(obj.weight.P) *0.7;
            end
            if obj.m >= 3
                idx_p2 = 7;
                w_vec(idx_p2 : idx_p2+2) = diag(obj.weight.P) *0.3;
            end
            base_y = 3 * obj.m + 1;
            if obj.m >= 1
                w_vec(base_y : base_y+2) = diag(obj.weight.V);
                w_vec(base_y+3 : base_y+5) = diag(obj.weight.V)*0.3;
                w_vec(base_y+6 : base_y+8) = diag(obj.weight.V)*0.0;
            end
            base_h = 6 * obj.m + 1;
            end_h = base_h + 3 * obj.m - 1;
            w_vec(base_h : end_h) = 0;
            base_z = 9 * obj.m + 1;
            for k = 1 : obj.n
                curr_idx = base_z + (k-1) * 9;
                curr_end = curr_idx + 8;
                if k == 1
                    w_vec(curr_idx : curr_end) = mean(diag(obj.weight.Q));
                     w_vec(curr_idx+1)=500;
                     w_vec(curr_idx+3)=500;
                elseif k == 2
                    w_vec(curr_idx : curr_end) = mean(diag(obj.weight.W));
                    w_vec(curr_idx+1)=100;
                    w_vec(curr_idx+3)=100;
                else
                    w_vec(curr_idx : curr_end) = 0;
                end
            end
            Q_stage = diag(w_vec);
            % try, Q_terminal = dare(A_d*0.995, B_d, Q_stage, obj.weight.input); catch, Q_terminal = Q_stage * 2; end
            Q_terminal = 1*Q_stage;
            Q_bar = blkdiag(kron(eye(obj.H-1), Q_stage), Q_terminal);
            R_bar  = kron(eye(obj.H), obj.weight.input);
            RP_bar = kron(eye(obj.H), obj.weight.preinputdif);
            Up = repmat(obj.input.pre_u(:, 1), obj.H, 1);
            [obj.quadH, obj.quadf] = obj.gen_Hf(obj.koopman.ExA, obj.koopman.ExB, z_current, ...
                Q_bar, R_bar, RP_bar, ...
                Xr, Ur, Up);
            A = []; b = [];
            Aeq = []; beq = [];
            obj.quadH = (obj.quadH + obj.quadH') / 2;
            obj.quadH = obj.quadH + eye(size(obj.quadH)) * 1e-6;
            lb = repmat(obj.param.input_min, obj.param.H, 1);
            ub = repmat(obj.param.input_max, obj.param.H, 1);
            obj.options = optimset('Display', 'off');

            [var,fval,eflag,~,~] = quadprog(obj.quadH,obj.quadf,A,b,Aeq,beq,lb,ub,[],obj.options);

            if eflag ~= 1
                disp(['Warning: Quadprog failed to find a solution. eflag = ', num2str(eflag)]);
            end
            obj.result.eflag= eflag;
            obj.result.input =var(1:4, 1); % 算出された入力
            obj.result.u_nom = obj.result.input;
            obj.result.delta_u_edmd = obj.compute_residual_delta_u(obj.result.u_nom);
            obj.result.delta_u_z = obj.compute_vertical_delta_u(obj.result.u_nom);
            obj.result.u_total_pre_sat = obj.result.u_nom + obj.result.delta_u_edmd + obj.result.delta_u_z;
            obj.result.input = obj.result.u_total_pre_sat;
            % if ~isfield(obj.result, 'd_est'), obj.result.d_est = zeros(4,1); obj.result.x_last = obj.state.current; end
            % pred_error = obj.state.current - (A_d * obj.result.x_last + B_d * obj.input.pre_u);
            % obj.result.d_est = 0.8 * obj.result.d_est + 0.2 * (pinv(B_d) * pred_error);
            % obj.result.input = obj.result.input - obj.result.d_est;
            % obj.result.x_last = obj.state.current;
            obj.result.input = max(min(obj.result.input, obj.param.input_max), obj.param.input_min);
            obj.result.eflag = eflag;
            obj.result.var = var;
            obj.result.Bestcost_pre = obj.result.bestcost;
            obj.result.bestcost = [fval;0];
            obj.result.kqlmpc = obj.result.input;
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
       
        function residual = initialize_residual_model(obj, param)
            residual = struct();
            residual.mode = 0;
            residual.loaded = false;
            residual.alpha = 1.0;
            residual.du_max = [0; 0; 0; 0];
            residual.use_reference = 1;
            residual.pinv_damping = 1e-3;
            residual.A_nom = [];
            residual.B_nom = [];
            residual.A_err = [];
            residual.B_err = [];
            residual.K = [];
            residual.dlqr_ok = false;
            residual.dlqr_message = '';
            residual.K_norm = 0;
            residual.B_rank = 0;
            residual.ctrb_rank = 0;
            residual.torque_scale = [1; 1; 1];
            residual.torque_beta = 1.0;
            residual.delta_tau_prev = zeros(3,1);
            residual.full_beta = 1.0;
            residual.delta_u_prev = zeros(4,1);
            residual.lqr_torque_only = 0;
            residual.lqr_beta = 1.0;
            residual.delta_u_lqr_prev = zeros(4,1);
            residual.use_aligned_reference = 1;
            residual.mode25_torque_only = 0;

            if ~isfield(param, 'residual') || ~isfield(param.residual, 'mode')
                return;
            end

            residual.mode = param.residual.mode;
            residual.alpha = param.residual.alpha;
            residual.du_max = param.residual.du_max;
            residual.use_reference = param.residual.use_reference;
            residual.pinv_damping = param.residual.pinv_damping;
            residual.torque_scale = param.residual.torque_scale(:);
            residual.torque_beta = param.residual.torque_beta;
            residual.full_beta = param.residual.full_beta;
            residual.lqr_torque_only = param.residual.lqr_torque_only;
            residual.lqr_beta = param.residual.lqr_beta;
            if isfield(param.residual, 'use_aligned_reference')
                residual.use_aligned_reference = param.residual.use_aligned_reference;
            end
            if isfield(param.residual, 'mode25_torque_only')
                residual.mode25_torque_only = param.residual.mode25_torque_only;
            end

            if residual.mode == 0
                return;
            end

            try
                mdl = load(param.residual.model_file, 'results_edmd');
                residual.A_nom = mdl.results_edmd.A_nom;
                residual.B_nom = mdl.results_edmd.B_nom;
                residual.A_err = mdl.results_edmd.A_err;
                residual.B_err = mdl.results_edmd.B_err;
                residual.loaded = true;
                residual.B_rank = rank(residual.B_err);
                residual.ctrb_rank = rank(ctrb(residual.A_err, residual.B_err));

                q_res = eye(size(residual.A_err, 1)) * param.residual.q_scale;
                r_res = eye(size(residual.B_err, 2)) * param.residual.r_scale;
                try
                    residual.K = dlqr(residual.A_err, residual.B_err, q_res, r_res);
                    residual.dlqr_ok = true;
                    residual.K_norm = norm(residual.K, 'fro');
                catch
                    residual.K = zeros(size(residual.B_err, 2), size(residual.A_err, 1));
                    residual.dlqr_ok = false;
                    residual.dlqr_message = 'dlqr failed, K forced to zero';
                    residual.K_norm = 0;
                end
            catch ME
                warning('Residual model load failed: %s', ME.message);
                residual.mode = 0;
            end
        end

        function delta_u = compute_residual_delta_u(obj, u_nom)
            delta_u = zeros(4, 1);
            obj.result.residual_mode = 0;

            if isempty(obj.residual) || ~isfield(obj.residual, 'mode') || obj.residual.mode == 0
                return;
            end
            if ~obj.residual.loaded
                return;
            end

            x_cur = obj.current_state(:);
            x16_cur = [x_cur; u_nom(:)];
            z_cur = obj.klift_edmd_residual(x16_cur);

            if obj.residual.use_reference == 1
                x_ref = obj.get_residual_reference_state(1);
                u_ref = obj.state.ref(13:16, 1);
                z_ref = obj.klift_edmd_residual([x_ref; u_ref]);
            else
                z_ref = zeros(size(z_cur));
            end
            z_err = z_cur - z_ref;

            switch obj.residual.mode
                case 1
                    delta_u_raw = -obj.residual.K * z_err;
                    if obj.residual.lqr_torque_only == 1
                        delta_u_raw(1) = 0;
                    end
                    beta = obj.residual.lqr_beta;
                    delta_u = (1 - beta) * obj.residual.delta_u_lqr_prev + beta * delta_u_raw;
                    obj.residual.delta_u_lqr_prev = delta_u;
                    obj.result.delta_u_lqr_raw = delta_u_raw;
                    obj.result.delta_u_lqr_filtered = delta_u;
                case 2
                    x_ref_next = obj.get_residual_reference_state(min(2, size(obj.state.ref, 2)));
                    u_ref_now = obj.state.ref(13:16, 1);
                    z_ref_next = obj.klift_edmd_residual([x_ref_next; u_ref_now]);
                    z_nom_next = obj.residual.A_nom * z_cur + obj.residual.B_nom * u_nom;
                    z_target_bar = z_ref_next - z_nom_next;
                    rhs = z_target_bar - obj.residual.A_err * z_cur;
                    if obj.residual.mode25_torque_only == 1
                        Be = obj.residual.B_err(:, 2:4);
                        reg = obj.residual.pinv_damping * eye(size(Be, 2));
                        delta_tau = (Be' * Be + reg) \ (Be' * rhs);
                        delta_u = [0; delta_tau];
                    else
                        Be = obj.residual.B_err;
                        reg = obj.residual.pinv_damping * eye(size(Be, 2));
                        delta_u = (Be' * Be + reg) \ (Be' * rhs);
                    end
                    obj.result.z_ref_next = z_ref_next;
                    obj.result.z_nom_next = z_nom_next;
                    obj.result.z_target_bar = z_target_bar;
                case 3
                    z_target_bar = -obj.residual.A_err * z_cur;
                    Be = obj.residual.B_err(:, 2:4);
                    reg = obj.residual.pinv_damping * eye(size(Be, 2));
                    delta_tau_raw = (Be' * Be + reg) \ (Be' * z_target_bar);
                    delta_tau_scaled = obj.residual.torque_scale .* delta_tau_raw;
                    beta = obj.residual.torque_beta;
                    delta_tau = (1 - beta) * obj.residual.delta_tau_prev + beta * delta_tau_scaled;
                    obj.residual.delta_tau_prev = delta_tau;
                    delta_u = [0; delta_tau];
                    obj.result.delta_tau_raw = delta_tau_raw;
                    obj.result.delta_tau_scaled = delta_tau_scaled;
                    obj.result.delta_tau_filtered = delta_tau;
                case 5
                    x_ref_next = obj.get_residual_reference_state(min(2, size(obj.state.ref, 2)));
                    u_ref_now = obj.state.ref(13:16, 1);
                    z_ref_next = obj.klift_edmd_residual([x_ref_next; u_ref_now]);
                    z_nom_next = obj.residual.A_nom * z_cur + obj.residual.B_nom * u_nom;
                    z_target_bar = z_ref_next - z_nom_next;
                    rhs = z_target_bar - obj.residual.A_err * z_cur;
                    if obj.residual.mode25_torque_only == 1
                        Be = obj.residual.B_err(:, 2:4);
                        reg = obj.residual.pinv_damping * eye(size(Be, 2));
                        delta_tau_raw = (Be' * Be + reg) \ (Be' * rhs);
                        delta_u_raw = [0; delta_tau_raw];
                    else
                        Be = obj.residual.B_err;
                        reg = obj.residual.pinv_damping * eye(size(Be, 2));
                        delta_u_raw = (Be' * Be + reg) \ (Be' * rhs);
                    end
                    beta = obj.residual.full_beta;
                    delta_u = (1 - beta) * obj.residual.delta_u_prev + beta * delta_u_raw;
                    obj.residual.delta_u_prev = delta_u;
                    obj.result.z_ref_next = z_ref_next;
                    obj.result.z_nom_next = z_nom_next;
                    obj.result.z_target_bar = z_target_bar;
                    obj.result.delta_u_mode2_raw = delta_u_raw;
                    obj.result.delta_u_mode5_filtered = delta_u;
                otherwise
                    delta_u = zeros(4, 1);
            end

            delta_u = obj.residual.alpha * delta_u;
            obj.result.delta_u_edmd_raw = delta_u;
            delta_u = max(min(delta_u, obj.residual.du_max), -obj.residual.du_max);
            obj.result.residual_mode = obj.residual.mode;
            obj.result.z_edmd_cur = z_cur;
            obj.result.z_edmd_ref = z_ref;
            obj.result.z_edmd_err = z_err;
            obj.result.z_edmd_err_norm = norm(z_err);
            obj.result.residual_K_norm = obj.residual.K_norm;
            obj.result.residual_dlqr_ok = obj.residual.dlqr_ok;
            obj.result.residual_B_rank = obj.residual.B_rank;
            obj.result.residual_ctrb_rank = obj.residual.ctrb_rank;
            obj.result.residual_dlqr_message = obj.residual.dlqr_message;
        end

        function delta_u = compute_vertical_delta_u(obj, u_nom)
            delta_u = zeros(4, 1);
            if isempty(obj.residual) || ~isfield(obj.residual, 'mode')
                return;
            end
            if obj.residual.mode ~= 3
                return;
            end

            z_err = obj.state.ref(3,1) - obj.current_state(3);
            vz_err = obj.state.ref(9,1) - obj.current_state(9);
            obj.integral_error(3) = obj.integral_error(3) + z_err * obj.param.dt;
            lim = obj.param.residual.z_int_limit;
            obj.integral_error(3) = max(min(obj.integral_error(3), lim), -lim);

            thrust_corr = obj.param.residual.z_kp * z_err + ...
                          obj.param.residual.z_kv * vz_err + ...
                          obj.param.residual.z_ki * obj.integral_error(3);

            thrust_corr = max(min(thrust_corr, obj.param.residual.du_max(1)), -obj.param.residual.du_max(1));
            delta_u(1) = thrust_corr;

            obj.result.z_pos_err = z_err;
            obj.result.z_vel_err = vz_err;
            obj.result.z_int_err = obj.integral_error(3);
        end

        function z = klift_edmd_residual(obj, x16)
            P1 = x16(1);
            P2 = x16(2);
            P3 = x16(3);
            Q1 = x16(4);
            Q2 = x16(5);
            Q3 = x16(6);
            V1 = x16(7);
            V2 = x16(8);
            V3 = x16(9);
            W1 = x16(10);
            W2 = x16(11);
            W3 = x16(12);

            c1 = cos(Q1);
            s1 = sin(Q1);
            c2 = cos(Q2);
            s2 = sin(Q2);
            c3 = cos(Q3);
            s3 = sin(Q3);

            c1_safe = obj.safe_nonzero(c1, 1e-3);
            c2_safe = obj.safe_nonzero(c2, 1e-3);

            R13 = c3 * s2 * c1 + s3 * s1;
            R23 = s3 * s2 * c1 - c3 * s1;
            R33 = c2 * c1;

            common_z = [P1; P2; P3; ...
                        Q1; Q2; Q3; ...
                        V1; V2; V3; ...
                        W1; W2; W3; ...
                        R13; R23; R33; ...
                        1];

            kyo_z = [W1 * W2; ...
                     W2 * W3; ...
                     W3 * W1; ...
                     W2 * c1; ...
                     W3 * s1; ...
                     W1 * c2 / c1_safe; ...
                     W2 * s1 / c2_safe; ...
                     W3 * c1 / c2_safe; ...
                     W2 * s1 * s2 / c2_safe; ...
                     W3 * c1 * s2 / c2_safe];

            z = [common_z; kyo_z];
        end

        function x_ref = get_residual_reference_state(obj, idx)
            idx = max(1, idx);
            if isfield(obj.residual, 'use_aligned_reference') && obj.residual.use_aligned_reference == 1 ...
                    && ~isempty(obj.residual_ref_state)
                idx = min(idx, size(obj.residual_ref_state, 2));
                x_ref = obj.residual_ref_state(:, idx);
            else
                idx = min(idx, size(obj.state.ref, 2));
                x_ref = obj.state.ref(1:12, idx);
            end
        end

        function y = safe_nonzero(obj, x, eps_val)
            if abs(x) < eps_val
                if x >= 0
                    y = eps_val;
                else
                    y = -eps_val;
                end
            else
                y = x;
            end
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
            Omega = [0 ,-w(3) ,w(2); w(3), 0 ,-w(1); -w(2) ,w(1) ,0];
            OmegaT = Omega';
            invJ = inv(diag([params.jx params.jy params.jz]));
            Hk = obj.get_HYP_Block(h1, M, OmegaT, invJ);
            Yk = obj.get_HYP_Block(y1, M, OmegaT, invJ);
            Pk = obj.get_HYP_Block(p1, M, OmegaT, invJ);
            calB = zeros(9*M + 9*N, 4);
            for j = 1:M
                thrust_vec = (1/(params.m)) * (OmegaT^(j-1)) * e3;
                idx_p = 3*j-2;
                idx_y = 3*M + 3*j-2;
                idx_h = 6*M + 3*j-2;
                calB(idx_p : idx_p+2, 2:4) = Pk(:,:,j);
                calB(idx_y : idx_y+2, 1)   = thrust_vec;
                calB(idx_y : idx_y+2, 2:4) = Yk(:,:,j);
                calB(idx_h : idx_h+2, 2:4) = Hk(:,:,j);
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
            S = [0 ,-v(3) ,v(2); v(3), 0 ,-v(1); -v(2), v(1) ,0];
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
        % function [xr] = generate_reference(obj)
        %     xr = zeros(obj.param.total_size, obj.H);    % initialize
        %     RefTime = obj.self.reference.time_var.func; % 時間関数の取得
        %     % for h = 0:obj.param.H-1
        %     %     t = obj.param.t + obj.param.dt * h; % reference生成の時刻をずらす
        %     %     ref = RefTime(t);
        %     %     xr(1:3, h+1) = ref(1:3);
        %     %     xr(7:9, h+1) = ref(5:7);
        %     %     xr(4:6, h+1) =   [0;0;0]; % 姿勢角
        %     %     xr(10:12, h+1) = [0;0;0];
        %     %     xr(13:16, h+1) = obj.param.ref_input; % MC -> 0.6597,   HL -> 0
        %     % end
        %     g = 9.81;
        %     prev_euler = zeros(3,1);
        %     for h = 0:obj.H-1
        %         t   = obj.param.t + obj.param.dt * h;    % reference生成の時刻をずらす
        %         ref = RefTime(t);                        % 20x1
        %         acc = ref(9:11);                         % ddx, ddy, ddz
        %         yaw = ref(4);                            % yaw
        %         s  = acc + [0;0;g];
        %         b3 = s / norm(s);
        %         b1c = [cos(yaw); sin(yaw); 0];
        %         v = cross(b3, b1c);
        %         if norm(v) < 1e-6
        %             if abs(b3(3))<0.9, b1=[0;0;1]; else, b1=[1;0;0]; end
        %             b2 = cross(b3,b1); b2=b2/norm(b2); b1=cross(b2,b3);
        %         else
        %             b2 = v/norm(v);  b1 = cross(b2,b3);
        %         end
        %         Rd = [b1,b2,b3];
        %         phi   = atan2(Rd(3,2), Rd(3,3));
        %         theta = asin(-Rd(3,1));
        %         psi   = atan2(Rd(2,1), Rd(1,1));
        %         euler = [phi;theta;psi];
        %         if h == 0 && obj.H > 1
        %             t_next   = t + obj.param.dt;
        %             ref_next = RefTime(t_next);
        %             acc_n    = ref_next(9:11);
        %             yaw_n    = ref_next(4);
        %             s_n  = acc_n + [0;0;g];
        %             b3_n = s_n / norm(s_n);
        %             b1c_n = [cos(yaw_n); sin(yaw_n); 0];
        %             v_n = cross(b3_n, b1c_n);
        %             if norm(v_n) < 1e-6
        %                 if abs(b3_n(3))<0.9, b1_n=[0;0;1]; else, b1_n=[1;0;0]; end
        %                 b2_n = cross(b3_n,b1_n); b2_n=b2_n/norm(b2_n); b1_n=cross(b2_n,b3_n);
        %             else
        %                 b2_n = v_n/norm(v_n);  b1_n = cross(b2_n,b3_n);
        %             end
        %             Rd_n = [b1_n,b2_n,b3_n];
        %             phi_n   = atan2(Rd_n(3,2), Rd_n(3,3));
        %             theta_n = asin(-Rd_n(3,1));
        %             psi_n   = atan2(Rd_n(2,1), Rd_n(1,1));
        %             euler_n = [phi_n;theta_n;psi_n];
        %             euler_dot = (euler_n - euler) / obj.param.dt;
        %             euler_dot(3) = ref(8);
        %         elseif h == 0 && obj.H == 1
        %             euler_dot = [0;0;ref(8)];
        %         else
        %             euler_dot = (euler - prev_euler) / obj.param.dt;
        %             euler_dot(3) = ref(8);
        %         end
        %         prev_euler = euler;
        %         T = [ 1, 0, -sin(theta);
        %             0, cos(phi),  cos(theta)*sin(phi);
        %             0, -sin(phi), cos(theta)*cos(phi) ];
        %         w = T * euler_dot;
        %         xr(1:3,   h+1) = ref(1:3);
        %         xr(7:9,   h+1) = ref(5:7);
        %         xr(4:6,   h+1) = euler;
        %         xr(10:12, h+1) = w;
        %         xr(13, h+1) = norm(s)*obj.param.m;
        %         xr(14:16, h+1) = 0;
        %     end
        % end
        function [xr] = generate_reference(obj)
            xr = zeros(obj.param.total_size, obj.H);
            RefTime = obj.self.reference.time_var.func;
            % RefTime = obj.self.reference.func;
            g = 9.81;
            if ~isfield(obj.param, 'tau') || isempty(obj.param.tau)
                obj.param.tau = 0;  % reference 的“自身时间”
            end
            for h = 0:obj.H-1
                t = obj.param.tau + obj.param.dt * (h);
                ref = RefTime(t);
                acc = ref(9:11);
                vel = ref(5:7);
                jerk = ref(13:15);
                yaw = 0;
                dyaw = 0;
                s = acc + [0;0;g];
                norm_s = norm(s);
                if norm_s < 1e-6
                    b3 = [0; 0; 1];
                else
                    b3 = s / norm_s;
                end
                b1c = [cos(yaw); sin(yaw); 0];
                v = cross(b3, b1c);
                if norm(v) < 1e-6
                    if abs(b3(3)) < 0.9, temp_b1=[0;0;1]; else, temp_b1=[1;0;0]; end
                    b2 = cross(b3, temp_b1); b2 = b2/norm(b2);
                else
                    b2 = v/norm(v);
                end
                b1 = cross(b2, b3);
                Rd = [b1,b2,b3];
                phi = atan2(Rd(3,2), Rd(3,3));
                theta = asin(-Rd(3,1));
                psi = atan2(Rd(2,1), Rd(1,1));
                euler = [0;0;0];
                if norm_s < 1e-6
                    w = [0; 0; 0];
                else
                    hw = (jerk - dot(b3, jerk) * b3) / norm_s;
                    w_x = -dot(hw, b2);
                    w_y = dot(hw, b1);
                    w_z = dot(b3, [0;0;1]) * dyaw;
                    w = [w_x; w_y; w_z];
                end
                xr(1:3, h+1) = ref(1:3);
                xr(7:9, h+1) = ref(5:7);
                xr(4:6, h+1) = euler;
                xr(10:12, h+1) = w;
                xr(13, h+1) = norm_s * obj.param.m;
                xr(14:16, h+1) = 0;
            end
        end
        function show(obj)
            % clc;
            % est_print = obj.self.estimator.result.state;
            est_print = obj.self.estimator.result.state;
            fprintf("==================================================================\n")
            fprintf("=================================================================\n")
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
        function koopman_analysis(obj,A, B)
            %         [n, ~] = size(A);
            %         [~, m] = size(B);
            %
            %         fprintf('========== Koopman系統解析 ==========\n');
            %         fprintf('状態次元: %d, 入力次元: %d\n\n', n, m);
            %
            %         %% 安定性解析
            %         fprintf('--- 安定性解析 ---\n');
            %         eig_vals = eig(A);
            %         max_eig = max(abs(eig_vals));
            %
            %         fprintf('固有値:\n');
            %         for i = 1:length(eig_vals)
            %             fprintf('  λ%d = %.4f%+.4fi (|λ| = %.4f)\n', ...
            %                 i, real(eig_vals(i)), imag(eig_vals(i)), abs(eig_vals(i)));
            %         end
            %
            %         fprintf('最大固有値の絶対値: %.4f\n', max_eig);
            %         if max_eig < 1
            %             fprintf('判定: ✓ 安定 (単位円内)\n');
            %         elseif max_eig == 1
            %             fprintf('判定: ⚠ 臨界安定\n');
            %         else
            %             fprintf('判定: ✗ 不安定\n');
            %         end
            %         fprintf('安定余裕: %.4f\n\n', 1 - max_eig);
            %
            %         %% 可制御性解析
            %         fprintf('--- 可制御性解析 ---\n');
            %         C = B;
            %         for i = 1:n-1
            %             C = [C, A^i * B];
            %         end
            %         rank_C = rank(C);
            %         ctrl_rate = (rank_C / n) * 100;
            %
            %         fprintf('可制御性行列のランク: %d/%d\n', rank_C, n);
            %         if rank_C == n
            %             fprintf('判定: ✓ 完全可制御\n');
            %         else
            %             fprintf('判定: ✗ 不完全可制御\n');
            %             fprintf('不可制御部分空間: %d次元\n', n - rank_C);
            %         end
            %         fprintf('可制御度: %.1f%%\n\n', ctrl_rate);
            %
            %         %% データ影響解析
            %         fprintf('--- データ影響解析 ---\n');
            %
            %         % A行列要素の影響
            %         fprintf('【A行列】重要要素トップ5:\n');
            %         A_inf = abs(A) / (sum(abs(A(:))) + eps);
            %         [~, idx] = sort(A_inf(:), 'descend');
            %         for i = 1:min(5, length(idx))
            %             [r, c] = ind2sub(size(A), idx(i));
            %             fprintf('  A(%d,%d)=%.4f, 影響率: %.2f%%\n', ...
            %                 r, c, A(r,c), A_inf(r,c)*100);
            %         end
            %
            %         % B行列要素の影響
            %         fprintf('【B行列】重要要素トップ5:\n');
            %         B_inf = abs(B) / (sum(abs(B(:))) + eps);
            %         [~, idx_B] = sort(B_inf(:), 'descend');
            %         for i = 1:min(5, length(idx_B))
            %             [r, c] = ind2sub(size(B), idx_B(i));
            %             fprintf('  B(%d,%d)=%.4f, 影響率: %.2f%%\n', ...
            %                 r, c, B(r,c), B_inf(r,c)*100);
            %         end
            %
            %         % 状態結合強度
            %         fprintf('【状態結合】各状態への影響:\n');
            %         coupling = sum(abs(A), 2);
            %         [~, s_idx] = sort(coupling, 'descend');
            %         for i = 1:n
            %             si = s_idx(i);
            %             fprintf('  x%d: 結合強度=%.4f, 影響率: %.2f%%\n', ...
            %                 si, coupling(si), (coupling(si)/sum(coupling))*100);
            %         end
            %
            %         % 入力チャネル影響
            %         fprintf('【入力チャネル】システムへの影響:\n');
            %         inp_inf = sum(abs(B), 1);
            %         [~, i_idx] = sort(inp_inf, 'descend');
            %         for i = 1:m
            %             ii = i_idx(i);
            %             fprintf('  u%d: 影響強度=%.4f, 影響率: %.2f%%\n', ...
            %                 ii, inp_inf(ii), (inp_inf(ii)/sum(inp_inf))*100);
            %         end
            %
            %         %% 総合評価
            %         fprintf('\n--- 総合評価 ---\n');
            %         fprintf('安定性: %s\n', obj.iif(max_eig<1, '✓ 安定', '✗ 不安定'));
            %         fprintf('可制御性: %s\n', obj.iif(rank_C==n, '✓ 完全可制御', sprintf('✗ 部分可制御(%.1f%%)', ctrl_rate)));
            %         fprintf('システム状態: ');
            %         if max_eig<1 && rank_C==n
            %             fprintf('良好 - 安定かつ完全可制御\n');
            %         elseif max_eig<1
            %             fprintf('中 - 安定だが不可制御モード有\n');
            %         elseif rank_C==n
            %             fprintf('中 - 不安定だが完全可制御\n');
            %         else
            %             fprintf('不良 - 不安定かつ不可制御モード有\n');
            %         end
            %         fprintf('=====================================\n');
            %     end
            %
            %     function out = iif(obj,cond, true_val, false_val)
            %         if cond
            %             out = true_val;
            %         else
            %             out = false_val;
            %         end
            %     end
            % end
           
        end
    end
end

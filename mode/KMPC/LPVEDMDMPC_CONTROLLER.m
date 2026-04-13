classdef LPVEDMDMPC_CONTROLLER< handle

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
        modelf
        modelp
        P
        N
        H
        weight
        koopman
        qpparam
        previous_input
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
        m,n
    end

    methods
        function obj = LPVEDMDMPC_CONTROLLER(self, param)
            obj.self = self;
            obj.param = param;
            obj.param.catchflag = 0;
            obj.flag.gpuflag = 0;
            obj.modelf = obj.self.plant.method;
            obj.P = obj.self.parameter.get();
            obj.N = param.particle_num;
            obj.H = param.H;
            obj.weight = param.weight;
            obj.weight.input = param.weight.R;
            obj.weight.preinputdif = param.weight.RP;
            obj.weight.weightstl = 10;
            obj.result.input = obj.param.ref_input;
            obj.input = obj.param.input;
            obj.input.pre_u = repmat(obj.result.input,1,obj.H);
            obj.input.var = repmat(obj.param.ref_input,obj.H,1);
            obj.result.Bestcost_STL = 0;
            obj.input.sigma = param.input.Initsigma;
            obj.result.bestx(1, :) = repmat(obj.input.Bestcost_now(1), obj.param.H, 1);
            obj.result.besty(1, :) = repmat(obj.input.Bestcost_now(1), obj.param.H, 1);
            obj.result.bestz(1, :) = repmat(obj.input.Bestcost_now(1), obj.param.H, 1);
            obj.state.state_data = zeros(obj.param.state_size,obj.H, obj.N);
            obj.result.Evaluationtra = zeros(obj.N, 2);
            obj.StageStateSTLsum = 0;
            obj.result.pre_u = obj.input.pre_u;
            obj.koopman = param.koopman;
            
            %% ====== structured_koopman_model 読み込み ======
            kr = load('structured_koopman_model3.mat');
            obj.koopman.A = kr.A_k;
            obj.koopman.B = kr.B_k * diag(kr.input_scales(:));
            obj.koopman.C = kr.C_k;
            obj.koopman.b_k = kr.b_k;   % ホバーバイアス補正
            obj.koopman.input_scales = kr.input_scales;
            obj.koopman.centers    = kr.centers;
            obj.koopman.x_mean_rbf = kr.x_mean_rbf;
            obj.koopman.x_std_rbf  = kr.x_std_rbf;
            obj.koopman.sigma_rbf  = kr.sigma_rbf;
            obj.koopman.keep_phys  = kr.keep_phys;
            obj.koopman.keep_edmd  = kr.keep_edmd;
            obj.koopman.keep_comb  = kr.keep_comb;
            obj.koopman.n_phys     = kr.n_phys;
            obj.koopman.n_edmd     = kr.n_edmd;
            
            obj.param.F = @(x) obj.lift_state_fn(x(1:12));
            
            C_cell = repmat({obj.koopman.C}, 1, obj.H);
            obj.koopman.ExC = blkdiag(C_cell{:});   
            obj.param.nz = size(kr.A_k, 1);  
            [obj.koopman.ExA, obj.koopman.ExB] = ExtendedCoefficientMatrix_kyo(...
                {obj.koopman.A, obj.koopman.B, obj.H, obj.param.nz});
            
            % ── ExBias: b_kのH歩累積 [nz*H x 1] ──────────────────
            % 真のKoopman伝播: z(k) = A^k*z0 + ExB*U + ExBias
            %   ExBias(k) = sum_{i=0}^{k-1} A^i * b_k
            %
            % MPC cost: sum || z(k) - Xr(k) ||^2
            %   = sum || (ExA*z0 + ExBias + ExB*U) - Xr ||^2
            %
            % [修正後の正しい実装]
            %   gen_Hf に x0_biased = ExA*z0 + ExBias を渡す
            %   → Xr はそのまま参照状態 F(x_ref) を使用
            %   → Xr から引く操作 (Xr - ExBias) は不要・誤り
            % ──────────────────────────────────────────────────────
            nz_loc = obj.param.nz;
            H_loc  = obj.H;
            ExBias = zeros(nz_loc * H_loc, 1);
            A_pow_b = kr.b_k;          % A^0 * b = b
            cumul_b = zeros(nz_loc, 1);
            for k = 1:H_loc
                cumul_b = cumul_b + A_pow_b;
                ExBias((k-1)*nz_loc+1 : k*nz_loc) = cumul_b;
                A_pow_b = kr.A_k * A_pow_b;  % A^k * b
            end
            obj.koopman.ExBias = ExBias;
            fprintf('ExBias構築完了: |ExBias|=%.4f\n', norm(ExBias));
            
            % ====== 力矩出力スケーリング係数 ======
            B_eff_loc = obj.koopman.B;
            dx_r = obj.koopman.C * B_eff_loc * [0;1;0;0];
            dx_p = obj.koopman.C * B_eff_loc * [0;0;1;0];
            dx_y = obj.koopman.C * B_eff_loc * [0;0;0;1];
            gain_ratio_roll  = abs(dx_r(10)) / (kr.dt / kr.Ixx);
            gain_ratio_pitch = abs(dx_p(11)) / (kr.dt / kr.Iyy);
            gain_ratio_yaw   = abs(dx_y(12)) / (kr.dt / kr.Izz);
            obj.koopman.torque_scale = mean([gain_ratio_roll, gain_ratio_pitch]);
            obj.koopman.yaw_scale    = gain_ratio_yaw;
            fprintf('力矩スケール: roll/pitch=%.3f  yaw=%.3f\n', ...
                obj.koopman.torque_scale, obj.koopman.yaw_scale);
            
            obj.flag.A = 0;
            obj.result.bestcost = obj.input.Bestcost_now;
            obj.sw.on = false;    
            obj.sw.k = 0;          
            obj.sw.Nsw = round(1/obj.param.dt);   
            obj.sw.u_warm = [];    
            obj.drf.beta = 0.8;  
            obj.drf.du_max = 0.35;    
            obj.act.u_prev = zeros(size(obj.input.pre_u(:,1,1))); 
        end

        %% do() — 元のまま
        function result = do(obj,varargin)
            persistent firstRun
            if isempty(firstRun)
                firstRun = true;
                result = obj.result;
                return
            end
            time = varargin{1};
            phase = varargin{2};
            obj.param.t = time.t;
            obj.act.u_prev = obj.input.pre_u(:,1,1);
            obj.current_state = obj.self.estimator.result.state.get();
            obj.state.current = obj.param.F(obj.current_state(1:12));
            obj.state.ref = obj.generate_reference();
            obj.param.tau = obj.param.tau + obj.param.dt;
            result = obj.controller_KMC(varargin);
            obj.result.kmpc = obj.result.input;
            u_kmpc_raw = obj.result.input;
            if ~isfield(obj.sw,'drf_flag') || isempty(obj.sw.drf_flag)
                obj.sw.drf_flag = 0;
            elseif obj.sw.drf_flag == 0
                obj.sw.drf_flag = 1;
                obj.sw.on = true;
                obj.sw.k = 0;
                obj.sw.u_warm = obj.result.pre_u(:,1);  
            end
            if obj.sw.on
                alpha = (min(1, obj.sw.k / obj.sw.Nsw))^2;
                w = 3*alpha^2 - 2*alpha^3;
                u_mix = (1-w)*obj.sw.u_warm + w*u_kmpc_raw;
                obj.sw.k = obj.sw.k + 1;
                if obj.sw.k >= obj.sw.Nsw, obj.sw.on = false; end
                du = u_mix - obj.act.u_prev;
                du = max(min(du, obj.drf.du_max), -obj.drf.du_max);
                u_step = obj.act.u_prev + du;
                u_cmd = obj.drf.beta*obj.act.u_prev + (1-obj.drf.beta)*u_step;
                obj.result.input = u_mix;
                obj.result.kmpc = u_mix;
                obj.act.u_prev = u_mix;
                obj.input.pre_u(:,1,1) = u_mix;        
                disp('controller: DRF,  phase: ');
                disp(phase);
            else
                obj.act.u_prev = obj.result.input;        
            end
            disp('controller: MC,  phase: ');
            disp(phase);
            obj.show();
        end

        %% controller_KMC — 元のまま
        function result = controller_KMC(obj,varargin)
            obj.param.t = varargin{1}{1}.t;
            obj.param.te = varargin{1}{1}.te;
            obj.QP_MPC();
            result = obj.result;
        end

        %% QP_MPC — バイアス補正修正版
        function QP_MPC(obj)
            n = size(obj.state.current, 1);
            Q_x_stage    = obj.param.weight.stagestate;
            Q_x_terminal = obj.param.weight.terminalstate;
            Q_x_stage    = (Q_x_stage + Q_x_stage')/2 + 1e-6*eye(n);
            Q_x_terminal = (Q_x_terminal + Q_x_terminal')/2 + 1e-6*eye(n);
            Q  = blkdiag(kron(eye(obj.param.H-1), Q_x_stage), Q_x_terminal);
            R  = kron(eye(obj.param.H), obj.weight.input);
            RP = kron(eye(obj.param.H), obj.weight.preinputdif);

            % Xr: 参照Koopman状態 [nz*H x 1]（そのまま使用、Exbiasは引かない）
            Xr = zeros(n * obj.param.H, 1);
            for h = 1:obj.param.H
                Xr((h-1)*n+1 : h*n) = obj.param.F(obj.state.ref(1:12, h));
            end

            u_hover = [obj.param.m * 9.81; 0; 0; 0];
            Ur = repmat(u_hover, obj.param.H, 1);

            % ── バイアス補正: x0_biased = ExA*z0 + ExBias ──────
            % コスト = || (ExA*z0 + ExBias + ExB*U) - Xr ||_Q^2
            %        = || x0_biased + ExB*U - Xr ||_Q^2
            % gen_Hf の A*x0 を x0_biased に置き換えることで
            % ExBias の効果を正しく組み込む
            % ────────────────────────────────────────────────────
            x0_biased = obj.koopman.ExA * obj.state.current + obj.koopman.ExBias;

            [obj.quadH, obj.quadf] = obj.gen_Hf(...
                obj.koopman.ExB, ...
                x0_biased, Q, R, RP, Xr, Ur, obj.input.var);

            lb = repmat(obj.param.input_min,1,obj.param.H);
            ub = repmat(obj.param.input_max,1,obj.param.H);
            obj.options = optimset('Display', 'off');
            [var,fval,eflag,~,~] = quadprog(obj.quadH,obj.quadf,[],[],[],[],lb,ub,[],obj.options);
            if eflag ~= 1
                disp(['Warning: Quadprog eflag = ', num2str(eflag)]);
            end
            obj.result.input = var(1:4, 1);
            
            % ====== 力矩出力スケーリング ======
            obj.result.input(2:3) = obj.result.input(2:3) * obj.koopman.torque_scale;
            obj.result.input(4)   = obj.result.input(4)   * obj.koopman.yaw_scale;
            obj.result.input = max(obj.param.input_min, min(obj.param.input_max, obj.result.input));
            
            obj.result.eflag = eflag;
            obj.result.var = var;
            obj.result.Bestcost_pre = obj.result.bestcost;
            obj.result.bestcost = [fval;0];
            obj.input.pre_u = obj.result.input; 
            obj.result.pre_u = obj.input.pre_u;
        end

        %% gen_Hf — バイアス修正版
        % 呼び出し元で x0_biased = ExA*z0 + ExBias を計算済み
        % B のみ受け取り、A*x0 の代わりに x0_biased をそのまま使う
        function [H,f] = gen_Hf(obj, B, x0_biased, Q, R, Rp, Xr, Ur, Up)
            H = 2*(B'*Q*B + R + Rp);
            H = (H+H')/2;
            % x0_biased = ExA*z0 + ExBias  (バイアスを加算済み)
            f = (2*(x0_biased - Xr)'*Q*B - 2*Ur'*R - 2*Up'*Rp)';
        end

        %% generate_reference — 元のまま
        function [xr] = generate_reference(obj)
            xr = zeros(12, obj.H);
            RefTime = obj.self.reference.time_var.func;
            g = 9.81;
            if ~isfield(obj.param, 'tau') || isempty(obj.param.tau)
                obj.param.tau = 0;
            end
            for h = 0:obj.H-1
                t = obj.param.tau + obj.param.dt * h;
                ref = RefTime(t);
                acc = ref(9:11); vel = ref(5:7); jerk = ref(13:15);
                yaw = 0; dyaw = 0;
                s = acc + [0;0;g]; norm_s = norm(s);
                if norm_s < 1e-6; b3=[0;0;1]; else; b3=s/norm_s; end
                b1c = [cos(yaw);sin(yaw);0];
                v = cross(b3,b1c);
                if norm(v) < 1e-6
                    if abs(b3(3))<0.9; tmp=[0;0;1]; else; tmp=[1;0;0]; end
                    b2=cross(b3,tmp); b2=b2/norm(b2);
                else; b2=v/norm(v); end
                b1 = cross(b2,b3);
                Rd = [b1,b2,b3];
                phi = atan2(Rd(3,2),Rd(3,3));
                theta = asin(-Rd(3,1));
                psi = atan2(Rd(2,1),Rd(1,1));
                if norm_s < 1e-6; w=[0;0;0];
                else
                    hw = (jerk - dot(b3,jerk)*b3)/norm_s;
                    w = [-dot(hw,b2); dot(hw,b1); dot(b3,[0;0;1])*dyaw];
                end
                xr(1:3,h+1)=ref(1:3); xr(4:6,h+1)=[phi;theta;psi];
                xr(7:9,h+1)=ref(5:7); xr(10:12,h+1)=w;
            end
        end
        
        %% show — 元のまま
        function show(obj)
            est_print = obj.self.estimator.result.state;
            fprintf("==================================================================\n")
            fprintf("=================================================================\n")
            fprintf("ps: %f %f %f \t vs: %f %f %f \t qs: %f %f %f \n",...
                est_print.p(1),est_print.p(2),est_print.p(3),...
                est_print.v(1),est_print.v(2),est_print.v(3),...
                est_print.q(1),est_print.q(2),est_print.q(3));
            fprintf("pr: %f %f %f \t vr: %f %f %f \t qr: %f %f %f \n", ...
                obj.state.ref(1,1),obj.state.ref(2,1),obj.state.ref(3,1),...
                obj.state.ref(7,1),obj.state.ref(8,1),obj.state.ref(9,1),...
                0,0,obj.state.ref(6,1));
            fprintf("t: %f \t input: %f %f %f %f \t J: %f \t sigma: %f", ...
                obj.param.t,obj.result.input(1),obj.result.input(2),...
                obj.result.input(3),obj.result.input(4),...
                obj.result.bestcost(1),obj.input.sigma(1));
            fprintf("\n");
        end

        %% ====== lift_state_fn: LPV(45d)+EDMD+RBF ======
        function z = lift_state_fn(obj, x_in)
            if size(x_in,1)>size(x_in,2); x_row=x_in'; else; x_row=x_in; end
            
            p_full = obj.compute_lpv_basis_single(x_row);
            p_sel = p_full(obj.koopman.keep_phys);   % keep_phys=1:45 → 全列
            
            e_full = obj.compute_edmd_basis_single(x_row);
            e_sel = e_full(obj.koopman.keep_edmd);
            
            x_n = (x_row - obj.koopman.x_mean_rbf) ./ obj.koopman.x_std_rbf;
            c_n = (obj.koopman.centers - obj.koopman.x_mean_rbf) ./ obj.koopman.x_std_rbf;
            dists = sum((x_n - c_n).^2, 2);
            rbf = exp(-dists / (2*obj.koopman.sigma_rbf^2));
            
            psi_full = [p_sel(:); e_sel(:); rbf(:)];
            z = psi_full(obj.koopman.keep_comb);
        end

        %% LPV基底（45次元）
        function Psi = compute_lpv_basis_single(obj, X)
            P1=X(1);P2=X(2);P3=X(3);
            Q1=X(4);Q2=X(5);Q3=X(6);
            V1=X(7);V2=X(8);V3=X(9);
            W1=X(10);W2=X(11);W3=X(12);
            c1=cos(Q1);s1=sin(Q1);c2=cos(Q2);s2=sin(Q2);c3=cos(Q3);s3=sin(Q3);
            R11=c2*c3;R12=c3*s2*s1-s3*c1;R13=c3*s2*c1+s3*s1;
            R21=c2*s3;R22=s3*s2*s1+c3*c1;R23=s3*s2*c1-c3*s1;
            R31=-s2;R32=c2*s1;R33=c2*c1;
            R_f=[R11,R12,R13;R21,R22,R23;R31,R32,R33];
            P_v=[P1;P2;P3];V_v=[V1;V2;V3];g_v=[0;0;9.81];
            Om=[0,-W3,W2;W3,0,-W1;-W2,W1,0];OmT=Om';RT=R_f';
            p1=RT*P_v;y1=RT*V_v;h1=-RT*g_v;
            p2=OmT*p1;y2=OmT*y1;h2=OmT*h1;
            p3=OmT*p2;y3=OmT*y2;h3=OmT*h2;
            z1=R_f(:);z2=R_f*Om;z2=z2(:);
            Psi=[p1;p2;p3;y1;y2;y3;h1;h2;h3;z1;z2];
        end

        %% EDMD基底（26次元）
        function Psi = compute_edmd_basis_single(obj, X)
            Q1=X(4);Q2=X(5);Q3=X(6);
            W1=X(10);W2=X(11);W3=X(12);
            c1=cos(Q1);s1=sin(Q1);c2=cos(Q2);s2=sin(Q2);c3=cos(Q3);s3=sin(Q3);
            R13=c3*s2*c1+s3*s1;R23=s3*s2*c1-c3*s1;R33=c2*c1;
            sc1=c1+sign(c1+1e-10)*1e-6;sc2=c2+sign(c2+1e-10)*1e-6;
            common=[X(1:12)';R13;R23;R33;1];
            kyo=[W1*W2;W2*W3;W3*W1;W2*c1;W3*s1;
                 W1*c2/sc1;W2*s1/sc2;W3*c1/sc2;W2*s1*s2/sc2;W3*c1*s2/sc2];
            Psi=[common;kyo];
        end

        function S = skew_func(obj,v)
            S = [0,-v(3),v(2);v(3),0,-v(1);-v(2),v(1),0];
        end
    end
end
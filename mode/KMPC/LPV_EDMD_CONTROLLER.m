classdef LPV_EDMD_CONTROLLER < handle
% ================================================================
% LPV_EDMD_FUSION_CONTROLLER
%
% 解析LPV（45次元）+ 学習済みEDMD（26次元）の融合コントローラ
% 観測量: z = [z_edmd(26); z_lpv(45)]  計71次元
%
% 代価関数（統一形式）:
%   min_U  sum_{k=1}^{H} ||z(k) - z_ref(k)||²_Q  +  ||u(k) - u_ref||²_R
%   z_ref(k) = [klift_edmd(x_ref(k)); klift_lpv(x_ref(k))]
%
% 重みQの設計（各観測量の物理的意味に基づく）:
%   EDMD部 (1:12)   [p,q,v,ω]     → P,Q,V,W を直接対応
%   EDMD部 (13:26)  [R13,1,kyo..] → 補助項 → ゼロ
%   LPV部  (27:29)  R'p           → P相当
%   LPV部  (30:32)  Ω'R'p         → P×0.5（高次減衰）
%   LPV部  (33:35)  Ω²R'p         → P×0.2
%   LPV部  (36:38)  R'v           → V相当
%   LPV部  (39:41)  Ω'R'v         → V×0.3
%   LPV部  (42:44)  Ω²R'v         → ゼロ
%   LPV部  (45:53)  重力項 h      → ゼロ
%   LPV部  (54:62)  R(:)=z1       → Q相当
%   LPV部  (63:71)  RΩ(:)=z2      → W相当
%
% generate_reference: MPC_CONTROLLER_KMC_kyo_guiexperiment と完全共通
% ================================================================
 
    properties
        options
        param
        current_state
        input
        state
        result
        self
        weight
        koopman         % 融合モデル（A,B,ExA,ExB）
        quadH
        quadf
        m = 3           % LPV次数（KQ_LMPCと同じ）
        n = 2
        n_edmd = 26
        n_lpv  = 45     % 9*m + 9*n
        n_z    = 71     % n_edmd + n_lpv
        H
        act
        sw
        drf
    end
 
    methods (Static)
        function [Ad, Bd] = c2d_rk4(Ac, Bc, dt)
            I = eye(size(Ac,1));
            M = Ac*dt; M2=M*M; M3=M2*M; M4=M3*M;
            Ad = I + M + M2/2 + M3/6 + M4/24;
            Bd = (I + M/2 + M2/6 + M3/24) * Bc * dt;
        end
    end
 
    methods
        %% ── コンストラクタ ──────────────────────────────────────
        function obj = LPV_EDMD_CONTROLLER(self, param)
            obj.self  = self;
            obj.param = param;
            obj.H     = param.H;
            obj.weight = param.weight;
            obj.weight.input       = param.weight.R;
            obj.weight.preinputdif = param.weight.RP;
 
            %% 入力初期化
            obj.result.input  = param.ref_input;
            obj.input         = param.input;
            obj.input.pre_u   = repmat(param.ref_input, 1, obj.H);
            obj.input.var     = repmat(param.ref_input, obj.H, 1);
            obj.result.bestcost      = param.input.Bestcost_now;
            obj.result.Bestcost_STL  = 0;
            obj.input.sigma   = param.input.Initsigma;
            obj.result.pre_u  = obj.input.pre_u;
 
            %% スムーズ切替（MPC_CONTROLLER_KMC_kyo_guiexperimentと同じ）
            obj.sw.on    = false;
            obj.sw.k     = 0;
            obj.sw.Nsw   = round(1/obj.param.dt);
            obj.sw.u_warm = [];
            obj.drf.beta  = 0.8;
            obj.drf.du_max = 0.35;
            obj.act.u_prev = zeros(4,1);
 
            %% 融合A/B行列の取得（param経由で渡される）
            obj.koopman = param.koopman;
 
            %% 拡張行列を一度だけ計算
            [Ad, Bd] = LPV_EDMD_CONTROLLER.c2d_rk4(...
                obj.koopman.lpvA, obj.koopman.lpvB, obj.param.dt);
             obj.koopman.A = blkdiag(A_edmd, Ad);           % 71×71
             obj.koopman.B = [B_edmd; Bd];             % 71×4

            [obj.koopman.ExA, obj.koopman.ExB] = ...
                obj.build_extended(Ad, Bd, obj.H, obj.n_z);
            
            fprintf('[FUSION] 初期化完了: n_z=%d (%dedmd+%dlpv)  H=%d\n', ...
                obj.n_z, obj.n_edmd, obj.n_lpv, obj.H);
        end
 
        %% ── メインループ ────────────────────────────────────────
        function result = do(obj, varargin)
            persistent firstRun
            if isempty(firstRun)
                firstRun = true;
                result = obj.result;
                return
            end
            time  = varargin{1};
            phase = varargin{2};
            obj.param.t   = time.t;
            obj.state.ref = obj.generate_reference();
            obj.param.tau = obj.param.tau + obj.param.dt;
            obj.act.u_prev = obj.input.pre_u(:,1,1);
 
            %% 状態リフティング
            obj.current_state = obj.self.estimator.result.state.get();
            x12 = obj.current_state(1:12);
            z_edmd = obj.klift_edmd(x12);
            z_lpv  = obj.klift_lpv(x12);
            obj.state.current = [z_edmd; z_lpv];   % 71×1
 
            %% LPVのB行列を実時刻で更新（質量誤差はここに入る）
            B_lpv_new = obj.build_analytic_B(x12, ...
                obj.param.m, obj.param.jx, obj.param.jy, obj.param.jz);
            obj.koopman.B(obj.n_edmd+1:end, :) = B_lpv_new;
 
            %% 拡張行列を更新（B_lpvが変わるため毎ステップ再計算）
            [Ad, Bd] = LPV_EDMD_CONTROLLER.c2d_rk4(...
                obj.koopman.A, obj.koopman.B, obj.param.dt);
            [obj.koopman.ExA, obj.koopman.ExB] = ...
                obj.build_extended(Ad, Bd, obj.H, obj.n_z);
 
            %% 参照軌道
            obj.state.ref = obj.generate_reference();
 
            %% QP求解
            obj.QP_MPC();
 
            %% スムーズ切替（MPC_CONTROLLER_KMC_kyo_guiexperimentと同じ）
            result = obj.result;
            u_raw = obj.result.input;
            if ~isfield(obj.sw,'drf_flag') || isempty(obj.sw.drf_flag)
                obj.sw.drf_flag = 0;
            elseif obj.sw.drf_flag == 0
                obj.sw.drf_flag = 1;
                obj.sw.on = true; obj.sw.k = 0;
                obj.sw.u_warm = obj.result.pre_u(:,1);
            end
            if obj.sw.on
                alpha = (min(1, obj.sw.k/obj.sw.Nsw))^2;
                w = 3*alpha^2 - 2*alpha^3;
                u_mix = (1-w)*obj.sw.u_warm + w*u_raw;
                obj.sw.k = obj.sw.k + 1;
                if obj.sw.k >= obj.sw.Nsw, obj.sw.on = false; end
                du = max(min(u_mix - obj.act.u_prev, obj.drf.du_max), -obj.drf.du_max);
                obj.result.input = obj.act.u_prev + du;
                obj.act.u_prev   = obj.result.input;
                obj.input.pre_u(:,1,1) = obj.result.input;
            else
                obj.act.u_prev = obj.result.input;
            end
            obj.show();
        end
 
        %% ── QP_MPC ──────────────────────────────────────────────
        function QP_MPC(obj)
            z_current = obj.state.current;   % 71×1
 
            %% 参照観測量ベクトル z_ref（71×H → H*71×1）
            Xr = zeros(obj.n_z * obj.H, 1);
            for k = 1:obj.H
                xref      = obj.state.ref(1:12, k);
                z_ref_k   = [obj.klift_edmd(xref); obj.klift_lpv(xref)];
                Xr((k-1)*obj.n_z+1 : k*obj.n_z) = z_ref_k;
            end
 
            %% 重み行列Q（71次元、意味別に設計）
            Q_stage    = obj.build_Q_stage();
            Q_terminal = 3 * Q_stage;
            Q_bar  = blkdiag(kron(eye(obj.H-1), Q_stage), Q_terminal);
            R_bar  = kron(eye(obj.H), obj.weight.input);
            RP_bar = kron(eye(obj.H), obj.weight.preinputdif);
 
            Ur = reshape(obj.state.ref(13:16, :), [], 1);
            Up = repmat(obj.input.pre_u(:,1), obj.H, 1);
 
            %% gen_Hf（KQ_LMPCと同じ形式）
            [obj.quadH, obj.quadf] = obj.gen_Hf(...
                obj.koopman.ExA, obj.koopman.ExB, z_current, ...
                Q_bar, R_bar, RP_bar, Xr, Ur, Up);
 
            obj.quadH = (obj.quadH + obj.quadH')/2 + eye(size(obj.quadH))*1e-6;
            lb = repmat(obj.param.input_min, obj.H, 1);
            ub = repmat(obj.param.input_max, obj.H, 1);
            opts = optimset('Display','off');
 
            [var, fval, eflag] = quadprog(...
                obj.quadH, obj.quadf, [], [], [], [], lb, ub, [], opts);
 
            if eflag ~= 1
                fprintf('[FUSION] Warning: quadprog eflag=%d\n', eflag);
            end
            obj.result.input    = max(min(var(1:4), obj.param.input_max), obj.param.input_min);
            obj.result.eflag    = eflag;
            obj.result.var      = var;
            obj.result.bestcost = [fval; 0];
            obj.input.pre_u     = obj.result.input;
            obj.result.pre_u    = obj.input.pre_u;
        end
 
        %% ── 重み行列Q構築（71次元, 物理的意味に対応）───────────
        function Q_stage = build_Q_stage(obj)
            % ── スケール係数 ──────────────────────────────────────
            % EDMDとLPVの観測量を同じ「位置誤差1cmあたりのペナルティ」に
            % 正規化するため、LPV項にスケール係数を掛ける
            % LPV p1 = R'*p ≈ p なのでスケール≈1
            % LPV p2 = Ω'*p1 ≈ ω*p → 通常|ω|<0.5なのでスケール≈0.5
            % LPV p3 ≈ ω²*p → スケール≈0.25
            sc_lpv = 0.01;   % LPV部の全体スケール（KQ_LMPCのsc=0.01と統一）
            sc_edmd = 10.0;   % EDMD部は物理単位なのでそのまま
 
            w = zeros(obj.n_z, 1);
 
            %% ─── EDMD部 (1:26) ──────────────────────────────────
            % z_edmd(1:3)  = p  → weight.P
            w(1:3)  = sc_edmd * diag(obj.weight.P);
            % z_edmd(4:6)  = q  → weight.Q
            w(4:6)  = sc_edmd * diag(obj.weight.Q);
            % z_edmd(7:9)  = v  → weight.V
            w(7:9)  = sc_edmd * diag(obj.weight.V);
            % z_edmd(10:12)= ω  → weight.W
            w(10:12)= sc_edmd * diag(obj.weight.W);
            % z_edmd(13:15)= [R13,R23,R33]  → 補助項、小さい重み
            w(13:15)= 0;
            % z_edmd(16)   = 1（常数項）    → ゼロ
            w(16)   = 0;
            % z_edmd(17:26)= kyo項（角速度積）→ ゼロ
            w(17:26)= 0;
 
            %% ─── LPV部 (27:71) ──────────────────────────────────
            base = obj.n_edmd;  % = 26、以下 base+k でLPV内インデックスを指す
 
            % p1 (27:29) = R'*p              → P相当
            w(base+1  : base+3)  = 2 *sc_lpv* diag(obj.weight.P);
            % p2 (30:32) = Ω'*R'*p           → P × 0.5（高次減衰）
            w(base+4  : base+6)  = 2 *sc_lpv* diag(obj.weight.P) * 0.7;
            % p3 (33:35) = Ω²*R'*p           → P × 0.2
            w(base+7  : base+9)  = 2 *sc_lpv* diag(obj.weight.P) * 0.3;
            % y1 (36:38) = R'*v              → V相当
            w(base+10 : base+12) =  5*sc_lpv* diag(obj.weight.V);
            % y2 (39:41) = Ω'*R'*v           → V × 0.3
            w(base+13 : base+15) = 5 *sc_lpv* diag(obj.weight.V) * 0.3;
            % y3 (42:44) = Ω²*R'*v           → ゼロ
            w(base+16 : base+18) = 0;
            % h1 (45:47) = -R'*g             → ゼロ（重力項、制御に使わない）
            w(base+19 : base+21) = 0;
            % h2 (48:50)                     → ゼロ
            w(base+22 : base+24) = 0;
            % h3 (51:53)                     → ゼロ
            w(base+25 : base+27) = 0;
            % z1 (54:62) = R(:)  姿勢行列    → Q相当
            w(base+28 : base+36) = 1/33 *sc_lpv* mean(diag(obj.weight.Q));
            % z2 (63:71) = (RΩ)(:) 角速度付  → W相当
            w(base+37 : base+45) = 1/5 *sc_lpv* mean(diag(obj.weight.W));
 
            Q_stage = diag(w);
        end
 
        %% ── gen_Hf（KQ_LMPCと同一）────────────────────────────
        function [H, f] = gen_Hf(obj, A, B, x0, Q, R, Rp, Xr, Ur, Up)
            H = 2*(B'*Q*B + R + Rp);
            H = (H+H')/2;
            f = (2*(A*x0 - Xr)'*Q*B - 2*Ur'*R - 2*Up'*Rp)';
        end
 
        %% ── 拡張行列 ────────────────────────────────────────────
        function [ExA, ExB] = build_extended(obj, A, B, H, n)
            nu  = size(B,2);
            ExA = zeros(H*n, n);
            ExB = zeros(H*n, H*nu);
            for i = 1:H
                ExA((i-1)*n+1:i*n, :) = A^i;
                for j = 1:i
                    ExB((i-1)*n+1:i*n, (j-1)*nu+1:j*nu) = A^(i-j)*B;
                end
            end
        end
 
        %% ── EDMDリフティング（26次元）──────────────────────────
        function z = klift_edmd(obj, x12)
            % select_observable で得られるF関数と同じ内容
            % ここではobj.param.F_edmdを使う（コンストラクタで設定済み）
            z = obj.param.F_edmd([x12;obj.input.pre_u(:,1,1)]);
        end
 
        %% ── LPVリフティング（45次元）────────────────────────────
        function z = klift_lpv(obj, x12)
            % KQ_LMPC_CONTROLLERのkliftと完全一致
            p=x12(1:3); q=x12(4:6); v=x12(7:9); w=x12(10:12);
            R   = eul2rotm([q(3),q(2),q(1)],'ZYX');
            g_v = [0;0;obj.param.gravity];
            wx  = obj.skew(w);
            ps=[]; ys=[]; hs=[];
            for i = 1:obj.m
                T  = (wx')^(i-1);
                ps = [ps; T*R'*p];
                ys = [ys; T*R'*v];
                hs = [hs; -T*R'*g_v];
            end
            zr = [];
            for k = 1:obj.n
                zr = [zr; reshape(R*(wx)^(k-1),[],1)];
            end
            z = [ps; ys; hs; zr];
        end
 
        %% ── LPV解析B行列（実時刻更新・質量誤差入口）───────────
        function calB = build_analytic_B(obj, x12, m_use, Ixx, Iyy, Izz)
            % KQ_LMPC_CONTROLLERのget_Koopman_Bと同一
            % m_use: 名義質量（ここに誤差を入れて実験する）
            M=obj.m; N=obj.n;
            z_lpv = obj.klift_lpv(x12);
            p1=z_lpv(1:3); y1=z_lpv(3*M+1:3*M+3); h1=z_lpv(6*M+1:6*M+3);
            w=x12(10:12); q=x12(4:6);
            c=cos(q); s=sin(q);
            R=[c(2)*c(3), s(1)*s(2)*c(3)-c(1)*s(3), c(1)*s(2)*c(3)+s(1)*s(3);
               c(2)*s(3), s(1)*s(2)*s(3)+c(1)*c(3), c(1)*s(2)*s(3)-s(1)*c(3);
              -s(2),       s(1)*c(2),                 c(1)*c(2)];
            e3=[0;0;1];
            Om=[0,-w(3),w(2); w(3),0,-w(1); -w(2),w(1),0];
            OmT=Om'; invJ=diag([5/Ixx,1/Iyy,1/Izz]);
            sk=@(v)[0,-v(3),v(2); v(3),0,-v(1); -v(2),v(1),0];
            calB=zeros(9*M+9*N,4);
            for j=1:M
                tv=(1/0.9)*(OmT^(j-1))*e3;  % ← m_use が誤差の入口
                ip=3*j-2; iy=3*M+3*j-2; ih=6*M+3*j-2;
                Pk=zeros(3,3); Yk=zeros(3,3); Hk=zeros(3,3);
                for i=1:(j-1)
                    Pk=Pk+(OmT^(i-1))*sk((OmT^(j-1-i))*p1)*invJ;
                    Yk=Yk+(OmT^(i-1))*sk((OmT^(j-1-i))*y1)*invJ;
                    Hk=Hk+(OmT^(i-1))*sk((OmT^(j-1-i))*h1)*invJ;
                end
                calB(ip:ip+2,2:4)=Pk; calB(iy:iy+2,1)=tv;
                calB(iy:iy+2,2:4)=Yk; calB(ih:ih+2,2:4)=Hk;
            end
            sr=9*M;
            for ii=1:(N-1)
                c1b=zeros(3,3);c2b=zeros(3,3);c3b=zeros(3,3);
                for jj=1:ii
                    tA=R*(Om^(jj-1)); tC=Om^(ii-jj);
                    c1b=c1b+tA*sk(invJ(:,1))*tC;
                    c2b=c2b+tA*sk(invJ(:,2))*tC;
                    c3b=c3b+tA*sk(invJ(:,3))*tC;
                end
                ri=sr+9*ii+1;
                calB(ri:ri+8,2:4)=[c1b(:),c2b(:),c3b(:)];
            end
        end
 
        %% ── 参照軌道生成（既存と完全共通）────────────────────
        function xr = generate_reference(obj)
            xr = zeros(obj.param.total_size, obj.H);
            RefTime = obj.self.reference.time_var.func;
            g = 9.81;
            if ~isfield(obj.param,'tau')||isempty(obj.param.tau)
                obj.param.tau = 0;
            end
            for h = 0:obj.H-1
                t    = obj.param.tau + obj.param.dt*h;
                ref  = RefTime(t);
                acc  = ref(9:11); jerk=ref(13:15); yaw=0; dyaw=0;
                s    = acc+[0;0;g]; ns=norm(s);
                if ns<1e-6, b3=[0;0;1]; else, b3=s/ns; end
                b1c=[cos(yaw);sin(yaw);0]; v=cross(b3,b1c);
                if norm(v)<1e-6
                    if abs(b3(3))<0.9, tmp=[0;0;1]; else, tmp=[1;0;0]; end
                    b2=cross(b3,tmp); b2=b2/norm(b2);
                else, b2=v/norm(v); end
                b1=cross(b2,b3); Rd=[b1,b2,b3];
                phi=atan2(Rd(3,2),Rd(3,3));
                theta=asin(-Rd(3,1));
                psi=atan2(Rd(2,1),Rd(1,1));
                if ns<1e-6, ww=[0;0;0];
                else
                    hw=(jerk-dot(b3,jerk)*b3)/ns;
                    ww=[-dot(hw,b2);dot(hw,b1);dot(b3,[0;0;1])*dyaw];
                end
                xr(1:3,h+1)=ref(1:3); xr(4:6,h+1)=[phi;theta;psi];
                xr(7:9,h+1)=ref(5:7); xr(10:12,h+1)=ww;
                xr(13,h+1)=ns*obj.param.m; xr(14:16,h+1)=0;
            end
        end
 
        %% ── show ────────────────────────────────────────────────
        function show(obj)
            est=obj.self.estimator.result.state;
            fprintf("ps: %f %f %f \t pr: %f %f %f \t u: %f %f %f %f\n",...
                est.p(1),est.p(2),est.p(3),...
                obj.state.ref(1,1),obj.state.ref(2,1),obj.state.ref(3,1),...
                obj.result.input(1),obj.result.input(2),...
                obj.result.input(3),obj.result.input(4));
        end
 
        function S = skew(~,v)
            S=[0,-v(3),v(2); v(3),0,-v(1); -v(2),v(1),0];
        end
    end
end
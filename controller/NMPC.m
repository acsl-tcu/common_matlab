classdef NMPC < handle
    % NMPC
    % CasADiを用いた四ロータ用の非線形MPC制御器
    %
    % インターフェースはHL_MPCと同じ形式:
    %     obj    = NMPC(self, param);
    %     result = obj.do(varargin{:});
    %
    % 状態モデル:
    %     x = [p(3); v(3); eul(3); w(3)]
    %     u = [T; tau(3)]
    %
    % 計算方法:
    %     多重打ち切り法，RK4離散化，IPOPTによるNLP求解を用いる。
    %     NLPはコンストラクタで一度だけ構築し，各制御周期ではパラメータのみ更新する。

    properties
        self
        result
        param
        P
        parameter_name = ["mass","Lx","Ly","lx","ly","jx","jy","jz","gravity","km1","km2","km3","km4","k1","k2","k3","k4"];
        nmpc            % CasADiで作成した最適化関数
        Xg              % 状態系列の初期推定値
        Ug              % 入力系列の初期推定値
        u_hover         % ホバリング入力
        N               % 予測ホライズン
        dt              % サンプリング時間
    end

    methods
        %% ---------------------------------------------------------------
        function obj = NMPC(self, param)
            % NMPC制御器を初期化し，最適化問題を構築する
            import casadi.*
            obj.self  = self;
            obj.param = param;
            obj.P     = self.parameter.get(obj.parameter_name);

            cfg    = param.nmpc;
            obj.N  = getdef(cfg, 'N',  20);
            obj.dt = getdef(cfg, 'dt', 0.05);
            N = obj.N;  Ts = obj.dt;
            % 予測ホライズンとサンプリング時間を設定する

            % ---- 物理パラメータ ----
            m  = obj.P(1);
            Jm = diag(obj.P(6:8));       % 慣性モーメント jx, jy, jz
            g  = obj.P(9);
            obj.u_hover = [m*g; 0; 0; 0];
            % ホバリング時の基準入力を計算する

            % ---- 重みと制約 ----
            Q    = diag(getdef(cfg, 'Q',  [10 10 10, 1 1 1, 1 1 1, 0.1 0.1 0.1]));
            QN   = diag(getdef(cfg, 'QN', 10*[10 10 10, 1 1 1, 1 1 1, 0.1 0.1 0.1]));
            Ru   = diag(getdef(cfg, 'R',  [0.01 0.1 0.1 0.1]));
            umin = getdef(cfg, 'umin', [0; -1; -1; -1]);
            umax = getdef(cfg, 'umax', [10; 1; 1; 1]);
            % 状態重み，終端重み，入力重み，入力制約を設定する

            % ================= 1. 連続時間モデル =================
            x = SX.sym('x', 12);
            u = SX.sym('u', 4);
            v   = x(4:6);
            phi = x(7);
            th  = x(8);
            psi = x(9);
            w   = x(10:12);
            % 状態と入力をCasADiのシンボリック変数として定義する

            Rx = [1 0 0; 0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)];
            Ry = [cos(th) 0 sin(th); 0 1 0; -sin(th) 0 cos(th)];
            Rz = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];
            R  = Rz*Ry*Rx;
            % body座標系からworld座標系への回転行列を作成する

            W = [1, sin(phi)*tan(th), cos(phi)*tan(th);
                 0, cos(phi),        -sin(phi);
                 0, sin(phi)/cos(th), cos(phi)/cos(th)];
            % 角速度からオイラー角速度への変換行列

            skew = @(a) [0 -a(3) a(2); a(3) 0 -a(1); -a(2) a(1) 0];

            xdot = [ v;
                     [0;0;-g] + R*[0;0;u(1)]/m;
                     W*w;
                     Jm \ (u(2:4) - skew(w)*(Jm*w)) ];
            f = Function('f', {x,u}, {xdot});
            % 四ロータの12次元連続時間運動方程式を定義する

            % ================= 2. RK4離散化 =================
            k1 = f(x,           u);
            k2 = f(x + Ts/2*k1, u);
            k3 = f(x + Ts/2*k2, u);
            k4 = f(x + Ts*k3,   u);
            F  = Function('F', {x,u}, {x + Ts/6*(k1+2*k2+2*k3+k4)});
            % RK4により1ステップ先の離散時間モデルを作成する

            % ================= 3. パラメータ化NLP =================
            opti = Opti();
            X    = opti.variable(12, N+1);
            U    = opti.variable(4,  N);
            x0   = opti.parameter(12, 1);
            Xref = opti.parameter(12, N+1);
            % 状態系列，入力系列，現在状態，参照軌道を定義する

            opti.subject_to( X(:,1) == x0 );
            % 初期状態制約

            J = 0;
            for k = 1:N
                e  = X(:,k) - Xref(:,k);
                du = U(:,k) - obj.u_hover;
                J  = J + e'*Q*e + du'*Ru*du;
                opti.subject_to( X(:,k+1) == F(X(:,k), U(:,k)) );
            end
            % 各予測ステップで追従誤差と入力変化を評価し，動力学制約を加える

            eN = X(:,N+1) - Xref(:,N+1);
            J  = J + eN'*QN*eN;
            opti.minimize(J);
            % 終端コストを加えて目的関数を最小化する

            for j = 1:4
                opti.subject_to( U(j,:) >= umin(j) );
                opti.subject_to( U(j,:) <= umax(j) );
            end
            % 入力制約を設定する

            opti.subject_to( X(7,:) >= -pi/3 );
            opti.subject_to( X(7,:) <=  pi/3 );
            opti.subject_to( X(8,:) >= -pi/3 );
            opti.subject_to( X(8,:) <=  pi/3 );
            % rollとpitchに制約を加え，オイラー角の特異点を避ける

            p_opts = struct('expand', true, 'print_time', false, 'error_on_fail', false);
            s_opts = struct('max_iter', 100, 'print_level', 0, 'tol', 1e-5, ...
                            'acceptable_tol', 1e-3, 'warm_start_init_point', 'yes');
            opti.solver('ipopt', p_opts, s_opts);
            % IPOPTソルバを設定する

            obj.nmpc = opti.to_function('nmpc', {x0, Xref, X, U}, {X, U});
            % 最適化問題を関数化し，毎周期高速に呼び出せるようにする

            obj.Xg = [];
            obj.Ug = [];
            obj.result.input = obj.u_hover;
            % 初期推定値と出力入力を初期化する
        end

        %% ---------------------------------------------------------------
        function result = do(obj, varargin)
            % 現在状態と参照軌道からNMPCを解き，最初の入力を出力する
            est = obj.self.estimator.result.state;
            N = obj.N;
            dt = obj.dt;

            % ---- 現在状態 ----
            eul = obj.R2eul(est.getq("rotmat"));
            x0  = [est.p(:); est.v(:); eul; est.w(:)];
            % 推定状態をNMPC用の12次元状態ベクトルに変換する

            % ---- 参照軌道 ----
            xd0 = obj.self.reference.result.state.xd(:);
            xd0 = [xd0; zeros(max(0, 20-numel(xd0)), 1)];
            Xref = zeros(12, N+1);
            % 参照状態を予測ホライズン分に展開する準備を行う

            for k = 1:N+1
                tau = (k-1)*dt;
                p_k = xd0(1:4) + xd0(5:8)*tau + xd0(9:12)*tau^2/2 + xd0(13:16)*tau^3/6;
                v_k = xd0(5:8) + xd0(9:12)*tau + xd0(13:16)*tau^2/2;
                yaw_k = mod(p_k(4) - x0(9) + pi, 2*pi) - pi + x0(9);
                Xref(:,k) = [p_k(1:3); v_k(1:3); 0; 0; yaw_k; 0; 0; v_k(4)];
            end
            % Taylor展開により位置，速度，yaw参照を予測区間へ外挿する

            % ---- 熱スタート ----
            if isempty(obj.Xg)
                Xg = repmat(x0, 1, N+1);
                Ug = repmat(obj.u_hover, 1, N);
            else
                Xg = [obj.Xg(:,2:end), obj.Xg(:,end)];
                Xg(:,1) = x0;
                Ug = [obj.Ug(:,2:end), obj.Ug(:,end)];
            end
            % 前回解を1ステップずらして初期値として使用する

            % ---- 求解 ----
            [Xo, Uo] = obj.nmpc(x0, Xref, Xg, Ug);
            Xo = full(Xo);
            Uo = full(Uo);
            % NMPC最適化問題を解く

            if any(~isfinite(Uo(:)))
                Xo = Xg;
                Uo = Ug;
            end
            % 求解に失敗した場合は前回の初期値を代用する

            obj.Xg = Xo;
            obj.Ug = Uo;
            % 次回の熱スタート用に解を保存する

            % ---- 出力 ----
            obj.result.input    = Uo(:,1);
            obj.result.pre_u    = Uo(:,1);
            obj.result.pred_pos = Xo(1:3,:);
            obj.result.ref_pos  = Xref(1:3,:);
            obj.result.X_seq    = Xo;
            obj.result.U_seq    = Uo;
            % 最初の入力を制御入力として出力し，予測軌道も保存する

            result = obj.result;
            obj.show();
        end

        %% ---------------------------------------------------------------
        function e = R2eul(~, R)
            % 回転行列をZYXオイラー角 [roll; pitch; yaw] に変換する
            e = [ atan2(R(3,2), R(3,3));
                  asin(max(min(-R(3,1), 1), -1));
                  atan2(R(2,1), R(1,1)) ];
        end

        function show(obj)
            % 現在状態と参照状態を表示する
            est_print = obj.self.estimator.result.state;
            ref_print = obj.self.reference.result.state;
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

function v = getdef(s, name, default)
    % 構造体に指定フィールドがある場合はその値を返し，ない場合はデフォルト値を返す
    if isstruct(s) && isfield(s, name) && ~isempty(s.(name))
        v = s.(name);
    else
        v = default;
    end
end
classdef KQ_LMPC_EDMD_CONTROLLER< handle
% =========================================================================
%  KQ-LMPC (Koopman Quasi-linear LPV-MPC) + 补偿系(EDMD残差/垂直PI/PID/DOB)
%  -----------------------------------------------------------------------
%  核心: K_MPC() 内の LPV-MPC (解析的Koopman A + 状態依存B(沿horizon更新) + QP)
%  补偿手段はすべて flag で切替, デフォルト全OFF (纯LPV-MPC):
%    - obj.pidflag         : 外部PID补偿       (デフォルト 0)
%    - obj.dobflag         : DOB扰动估计       (デフォルト 0)
%    - param.residual.mode : EDMD残差补偿      (デフォルト 0)
%    - obj.lpvflag         : B沿horizon逐步更新 (デフォルト 1 = 修正版, 0 = 旧冻结B)
%  旧モンテカルロ/STL/K_LQRテスト実装はコメントアウト済み
%  -----------------------------------------------------------------------
%  [構造修正一覧]
%   (1) B行列: 当前状态处冻结 → 沿horizon逐步更新 (lpvflag, 长horizon抖振の主因対策)
%   (2) A_d・B積分核: 定数なのに毎周期再離散化していた → コンストラクタでキャッシュ
%   (3) persistent変数(firstRun/pos_integ/yaw_integ) → プロパティ化
%       (同一MATLABセッションで再実験すると前回の値が残るバグを解消)
%   (4) DOB: 前馈符号が逆(外乱を倍加する向き) → 減算に修正 / 予測は前周期のBと対で実施
%   (5) PID条件積分: if/else両分岐が同一処理で条件が無意味だった → anti-windupとして実装
%   (6) 隠し重み diag([1,50,50,20]) → weight.RP_axis としてパラメータファイルへ
%   (7) generate_reference: 計算直後に破棄していた euler/w (死代码) を整理
% =========================================================================
% =========================================================================

    properties
        % ===== コア (LPV-MPC本体で使用) =====
        options        % quadprog options
        param          % Controller_KQ_LMPC_EDMD.m で設定したパラメータ
        current_state  % 現在状態 (12次元)
        input          % 入力関連
        state          % lifted状態・参照
        result         % 出力 (framework/loggerから参照)
        self           % agent
        % ----- 旧実装の名残り (未使用) -----
        % const
        % reference
        % fRemove
        % model
        % sigma
        % tss
    end
    properties
        modelf         % plantのmethod
        P              % ドローンパラメータ
        H              % 予測ホライズン
        weight         % MPC重み
        koopman        % 解析的Koopman A/B
        quadH          % QPのHessian
        quadf          % QPの勾配
        m = 3          % 観測量チェーン次数 (p, y, h)
        n = 2          % SO(3)ブロック数
        lpvflag = 1    % [修正] 1: B沿horizon逐步更新(真LPV, デフォルト), 0: 当前状态处冻结B(旧実装・対照用)
        first_run_done = 0 % [修正] 初回スキップ判定 (旧persistent firstRunの置き換え)
        % ----- 补偿系 (flag/modeで有効化, デフォルトOFF) -----
        pidflag            % 外部PID补偿 ON/OFF
        dobflag = 0        % DOB扰动估计 ON/OFF
        integral_error     % 垂直PI用 (residual.mode==3)
        residual           % EDMD残差补偿
        residual_ref_state
        z_prev_dob = []    % DOB: 前ステップ z
        u_prev_dob = []    % DOB: 前ステップ u
        u_offset_dob       % DOB: 入力オフセット推定
        B_prev_dob = []    % [修正] DOB: 前周期のB_d (予測はz,u,Bを同周期の組で行う)
        yaw_integ = 0  
        yawcompflag = 1% [修正] PID: yaw積分 (旧persistentの置き換え)
        pos_integ
        U_integ_single
        % ----- 旧MC/STL/K_LQRテスト実装の名残り (未使用) -----
        % N              % モンテカルロ粒子数
        % Q              % K_LQR用
        % R              % K_LQR用
        % A_d
        % last_u_ff
        % exec_counter
        % d_hat
        % qpparam
        % previous_input
        % gen_beq
        % removeN
        % survive
        % removeX
        % flag
        % reinput
        % reEva
        % StageStateSTLsum
        % sw
        % drf
        % act
        % modelp
        % STL_period = [2,4]
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
            %% ===== 基本設定 =====
            obj.self = self;                  % agent
            obj.param = param;                % Controller_KQ_LMPC_EDMD.m のパラメータ
            obj.modelf = obj.self.plant.method;
            obj.P = obj.self.parameter.get(); % ドローンのパラメータ (質量, 慣性モーメントなど)
            obj.H = param.H;                  % 予測ホライズン

            %% ===== 机能flag (补偿手段, 默认全部关闭 → 纯LPV-MPC) =====
            obj.pidflag = 0;   % 外部PID补偿 (1=ON)  ※テスト・補足用
            obj.dobflag = 0;   % DOB扰动估计 (1=ON)  ※テスト用
            obj.lpvflag = 1; 
            obj.yawcompflag = 1;  % yaw恒値外乱(反トルク不平衡)の積分補償 (1=ON)% [修正] B沿horizon逐步更新 (1=真LPV/修正版, 0=旧冻结B対照)
            % EDMD残差补偿は param.residual.mode で切替 (0=OFF, デフォルト)

            %% ===== MPC 重み =====
            obj.weight = param.weight;
            obj.weight.input = param.weight.R;        % 入力の目標値との差
            obj.weight.preinputdif = param.weight.RP; % 前ステップ入力との差 (Δu)
            if ~isfield(obj.weight, 'RP_axis')        % [修正] 旧: K_MPC内に隠れていた各軸倍率
                obj.weight.RP_axis = [1; 50; 50; 20]; % 旧パラメータファイルとの互換用フォールバック
            end
            % 下記2つは K_MPC 内で w_vec を直接構成するため未使用
            % obj.weight.stagestate = blkdiag(obj.weight.P, obj.weight.Q, obj.weight.V, obj.weight.W);
            % obj.weight.terminalstate = blkdiag(obj.weight.Pf, obj.weight.Qf, obj.weight.Vf, obj.weight.Wf);

            %% ===== 入力・結果の初期化 =====
            obj.result.input = obj.param.ref_input; % 初期時刻にresultを定義しておかないと実行時エラー
            obj.input = obj.param.input;
            obj.input.pre_u = repmat(obj.result.input, 1, obj.H); % 前入力
            obj.input.sigma = param.input.Initsigma;              % show()の表示専用
            obj.result.pre_u = obj.input.pre_u;
            obj.result.bestcost = obj.input.Bestcost_now;

            %% ===== 解析的 Koopman モデル (Aは構造固定, Bは毎ステップ現在状態で構成) =====
            obj.koopman.A = zeros(45, 45);
            obj.koopman.B = zeros(45, 4);
            obj.koopman.A = obj.get_Koopman_A(obj.m, obj.n);
            % [修正] Aは定数 → 離散化A_dとB積分核(4次Taylor)は初回のみ計算してキャッシュ
            %        旧実装は毎周期 c2d_rk4 で再計算していた (結果は同一, 計算だけ無駄)
            Mk = obj.koopman.A * obj.param.dt;
            obj.koopman.A_d  = eye(45) + Mk + (1/2)*Mk^2 + (1/6)*Mk^3 + (1/24)*Mk^4;
            obj.koopman.Bint = (eye(45) + (1/2)*Mk + (1/6)*Mk^2 + (1/24)*Mk^3) * obj.param.dt; % B_d = Bint * B_c

            %% ===== 补偿系の初期化 (flag OFFでも構造体だけ用意しておく) =====
            obj.residual = obj.initialize_residual_model(param);
            obj.integral_error = zeros(4, 1);
            obj.residual_ref_state = [];
            obj.pos_integ = zeros(3, 1);
            obj.yaw_integ = 0;
            obj.U_integ_single = zeros(4, 1);

            %% ===== quadprog ウォームアップ (初回求解の遅延防止) =====
            H = eye(2);
            f = zeros(2, 1);
            quadprog(H, f, [], [], [], [], [], [], [], optimset('Display', 'off'));
            fprintf('[warmup] Optimization toolbox preloaded.\n');

            %% ----- 以下は旧モンテカルロ/K_LQRテスト用 (未使用のためコメントアウト) -----
            % obj.N = param.particle_num;
            % obj.input.var = repmat(obj.param.ref_input, obj.H, 1);
            % obj.result.Bestcost_STL = 0;
            % obj.result.bestx(1, :) = repmat(obj.input.Bestcost_now(1), obj.param.H, 1);
            % obj.result.besty(1, :) = repmat(obj.input.Bestcost_now(1), obj.param.H, 1);
            % obj.result.bestz(1, :) = repmat(obj.input.Bestcost_now(1), obj.param.H, 1);
            % obj.state.state_data = zeros(obj.param.state_size, obj.H, obj.N); % 50000粒子分の巨大メモリ確保 (QP経路では不要)
            % obj.result.Evaluationtra = zeros(obj.N, 2);
            % --- K_LQR用の重み ---
            % sc = 0.01; px = [1, 1, 1]; vx = [1, 1, 1];
            % w_p = [800, 500, 0]; w_v = [300, 300, 0]; w_g = 0*ones(1, obj.m); w_z = [300, 200];
            % q_vec = [kron(w_p, px), kron(w_v, vx), kron(w_g, ones(1,3)), kron(w_z, ones(1,9))] * sc;
            % obj.Q = diag(q_vec);
            % obj.R = diag([10; 100; 100; 100]);
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
            % [修正] persistent → プロパティ化
            % 旧実装のpersistentはオブジェクトを作り直しても値が残るため,
            % 同一セッションで実験を再実行すると初回スキップが働かなかった
            if obj.first_run_done == 0
                obj.first_run_done = 1;
                % [熱起動] 切替直前にHLCが実際に印加していた入力でpre_uを初期化
                u_last = obj.self.controller.result.input;
                if numel(u_last) == 4 && all(isfinite(u_last))
                    obj.input.pre_u = repmat(u_last(:), 1, obj.H);
                end
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
        %% =================================================================
        %  [テスト用・未使用] K_LQR : lifted状態に対する dlqr フィードバック
        %  LPV-MPCとの比較実験用。復帰する場合:
        %   1) 下の %{ %} を外す
        %   2) コンストラクタの obj.Q, obj.R とプロパティ Q, R を復帰
        %   3) controller_KMC 内の obj.K_LQR() を有効化
        %% =================================================================
        %{
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
        %}
        %% =================================================================
        %  [核心] LPV-MPC: 解析Koopman A(固定) + B(沿horizon逐步更新, lpvflag=1) → QP求解
        %  流程: 离散化(缓存) → 逐步B + 扩展预测矩阵 → 参考lift → 权重 → gen_Hf → quadprog
        %  ※ DOB/EDMD/垂直PI/PID 均为可选补偿, flag默认OFF时本函数=纯LPV-MPC
        %% =================================================================
        function K_MPC(obj)
            %% ===== 离散化 + 预测矩阵 =====
            A_d = obj.koopman.A_d; % [修正] Aは定数: キャッシュ済みA_dを使用 (旧: 毎周期c2d_rk4)
            if obj.lpvflag == 1
                %% [修正·核心] B沿horizon逐步更新 (真LPV预测)
                % 旧実装: 当前状态のBを全horizonで冻结 → B(R,ω)は未来H步で不変と仮定
                %   → 长horizon时远端预测失真, 每拍计划翻转 = 输入抖振の構造要因
                % 修正: 各step kのBを「当前状态 → 参考状态」の線形ブレンド状態で評価
                %   (追従が良ければブレンド≈参考軌道, 誤差が大きい近端は当前状态を反映)
                B_seq = cell(obj.H, 1);
                B_seq{1} = obj.koopman.Bint * obj.koopman.B; % k=1は当前状态 (controller_KMCで構築済みのBを再利用)
                for k = 2:obj.H
                    ratio = (k-1) / max(obj.H-1, 1);
                    x_k = (1-ratio)*obj.current_state(:) + ratio*obj.state.ref(1:12, k);
                    Bc_k = obj.get_Koopman_B(x_k, obj.klift(x_k, obj.m, obj.n), obj.m, obj.n, obj.param);
                    B_seq{k} = obj.koopman.Bint * Bc_k;
                end
                B_d = B_seq{1}; % 近端B (DOB等で使用)
                [obj.koopman.ExA, obj.koopman.ExB] = obj.ExtendedCoefficientMatrix_LPV(A_d, B_seq, obj.H);
            else
                % 旧実装: 当前状态处冻结B (対照実験用に保持)
                B_d = obj.koopman.Bint * obj.koopman.B;
                [obj.koopman.ExA, obj.koopman.ExB] = obj.ExtendedCoefficientMatrix({A_d, B_d, obj.H, obj.param.state_size});
            end
            n = size(obj.state.current, 1);

          
            %% ===== [补偿·可选] DOB 估计扰动 (dobflag==1 时启用, 默认OFF) =====
            if obj.dobflag == 1
                if isempty(obj.z_prev_dob) || isempty(obj.u_prev_dob)
                    % 首次运行
                    obj.u_offset_dob = zeros(size(obj.input.pre_u(:,1)));
                else
                    % 实测扰动
                    if isempty(obj.B_prev_dob), obj.B_prev_dob = B_d; end
                    % [修正] 予測は「前周期のz,u,B」の組で行う (旧: 当周期のB_dを流用)
                    z_predicted = obj.B_prev_dob * obj.u_prev_dob + A_d * obj.z_prev_dob;
                    d_observed = obj.state.current - z_predicted;

                    % 死区：扰动太小则不更新（避免噪声放大）
                    if norm(d_observed) > 1e-4
                        % 正则化最小二乘（替代 pinv，数值更稳定）
                        lambda_reg = 0.01;
                        u_offset_inst = (obj.B_prev_dob' * obj.B_prev_dob + lambda_reg * eye(size(obj.B_prev_dob, 2))) \ (obj.B_prev_dob' * d_observed); % [修正] dと同じ周期のBで射影

                        % 一阶低通滤波
                        alpha_dob = 0.1;
                        if isempty(obj.u_offset_dob)
                            obj.u_offset_dob = u_offset_inst;
                        else
                            obj.u_offset_dob = alpha_dob * u_offset_inst + (1 - alpha_dob) * obj.u_offset_dob;
                        end
                    end
                    % 死区内不更新 u_offset_dob（保持上一次估计值）

                    % 抗饱和
                    u_offset_limit = obj.param.input_max * 0.3;
                    obj.u_offset_dob = max(min(obj.u_offset_dob, u_offset_limit), -u_offset_limit);
                    % [修正] デバッグ表示をここへ移動 (旧: K_MPC末尾で同時刻のzと比較する誤った再計算をしていた)
                    fprintf('[DOB] |d_obs|=%.4f, d_pos=[%.4f %.4f %.4f], u_offset=[%.4f %.4f %.4f %.4f]\n', ...
                        norm(d_observed), ...
                        d_observed(1), d_observed(2), d_observed(3), ...
                        obj.u_offset_dob(1), obj.u_offset_dob(2), obj.u_offset_dob(3), obj.u_offset_dob(4));
                end
            else
                obj.u_offset_dob = zeros(size(obj.input.pre_u(:,1)));
            end

            %% ===== 参考轨迹 Xr（保持原版 q_mix 逻辑）=====
            q_curr = obj.current_state(4:6);
            split = 9 * obj.m;
            z_current = obj.state.current;
            Xr_vec = zeros(n * obj.param.H, 1);
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
            obj.result.ref = obj.state.ref;
            obj.residual_ref_state = xref_residual;
            Xr = Xr_vec;

            %% ===== 输入参考 =====
            Ur = reshape(obj.state.ref(13:16, :), [], 1);

            %% ===== 权重矩阵 (lifted空间中的stage权重 w_vec) =====
            w_vec = zeros(n, 1);
            if obj.m >= 1, w_vec(1:3) = diag(obj.weight.P) * 1; end
            if obj.m >= 2, w_vec(4:6) = diag(obj.weight.P) * 0.7; end
            if obj.m >= 3, w_vec(7:9) = diag(obj.weight.P) * 0.3; end

            base_y = 3 * obj.m + 1;
            if obj.m >= 1
                w_vec(base_y   : base_y+2) = diag(obj.weight.V);
                w_vec(base_y+3 : base_y+5) = diag(obj.weight.V) * 0.3;
                w_vec(base_y+6 : base_y+8) = diag(obj.weight.V) * 0.0;
            end

            base_h = 6 * obj.m + 1;
            end_h  = base_h + 3 * obj.m - 1;
            w_vec(base_h : end_h) = 0;

            base_z = 9 * obj.m + 1;
            for k = 1:obj.n
                curr_idx = base_z + (k-1) * 9;
                curr_end = curr_idx + 8;
                if k == 1
                    w_vec(curr_idx : curr_end) = obj.weight.Q(1,1);
                    w_vec(curr_idx + 1) = obj.weight.Q(3,3);
                    w_vec(curr_idx + 3) = obj.weight.Q(3,3);
                    w_vec(curr_idx + 2) = obj.weight.Q(2,2);
                    w_vec(curr_idx + 5) = obj.weight.Q(1,1);
                elseif k == 2
                    w_vec(curr_idx : curr_end) = obj.weight.W(1,1);
                    w_vec(curr_idx + 1) = obj.weight.W(3,3);
                    w_vec(curr_idx + 3) = obj.weight.W(3,3);
                    w_vec(curr_idx + 2) = obj.weight.W(2,2);
                    w_vec(curr_idx + 5) = obj.weight.W(1,1);
                else
                    w_vec(curr_idx : curr_end) = 0;
                end
            end

            Q_stage    = diag(w_vec);
            Q_terminal = 1 * Q_stage;
            Q_bar      = blkdiag(kron(eye(obj.H-1), Q_stage), Q_terminal);
            R_bar      = kron(eye(obj.H), obj.weight.input);
            RP_bar     = kron(eye(obj.H), obj.weight.preinputdif*diag(obj.weight.RP_axis)); % [修正] 隠し倍率[1,50,50,20]を weight.RP_axis としてパラメータ化
            Up         = repmat(obj.input.pre_u(:,1), obj.H, 1);

            %% ===== QP 构造与求解 =====
            [obj.quadH, obj.quadf] = obj.gen_Hf(obj.koopman.ExA, obj.koopman.ExB, z_current, ...
                Q_bar, R_bar, RP_bar, ...
                Xr, Ur, Up);

            obj.quadH = (obj.quadH + obj.quadH') / 2 + 1e-6 * eye(size(obj.quadH));

            A = []; b = []; Aeq = []; beq = [];
            lb = repmat(obj.param.input_min, obj.param.H, 1);
            ub = repmat(obj.param.input_max, obj.param.H, 1);
            obj.options = optimset('Display', 'off');

            [var, fval, eflag, ~, ~] = quadprog(obj.quadH, obj.quadf, A, b, Aeq, beq, lb, ub, [], obj.options);

            if eflag ~= 1
                disp(['Warning: Quadprog failed to find a solution. eflag = ', num2str(eflag)]);
            end
            obj.result.eflag = eflag;

            %% ===== MPC 名义输出 =====
            obj.result.input = var(1:4, 1);
            obj.result.u_nom = obj.result.input;
           %% ===== [补偿] yaw恒値外乱の積分補償 (yawcompflag==1) =====
           if obj.yawcompflag == 1
               e_yaw = obj.state.ref(6, 1) - obj.current_state(6);
               obj.yaw_integ = obj.yaw_integ + e_yaw * obj.param.dt;
               obj.yaw_integ = max(min(obj.yaw_integ, 2.5), -2.5);   % 積分値クランプ
               u_yaw_comp = 0.02 * obj.yaw_integ;                     % Ki = 0.02
               u_yaw_comp = max(min(u_yaw_comp, 0.05), -0.05);        % 出力クランプ ±0.05 N·m
               obj.result.input(4) = obj.result.input(4) + u_yaw_comp;
              
           end
            %% ===== [补偿·可选] DOB 前馈补偿 (dobflag==1 时启用, 默认OFF) =====
            if obj.dobflag == 1
                % [修正] 符号: d ≈ B·δ と推定した δ (外乱の入力等価量) は指令から「引く」ことで打ち消す
                %        旧実装は加算しており, 外乱を打ち消すどころか倍加する向きだった
                obj.result.input = obj.result.input - obj.u_offset_dob;
                obj.result.input = max(min(obj.result.input, obj.param.input_max), obj.param.input_min);
            end

            %% ===== [补偿·可选] EDMD 残差补偿 (param.residual.mode~=0 时启用, 默认OFF) =====
            obj.result.delta_u_edmd  = obj.compute_residual_delta_u(obj.result.u_nom);
            obj.result.delta_u_z     = obj.compute_vertical_delta_u(obj.result.u_nom);
            obj.result.u_total_pre_sat = obj.result.input + obj.result.delta_u_edmd + obj.result.delta_u_z;
            obj.result.input = obj.result.u_total_pre_sat;
            obj.result.input = max(min(obj.result.input, obj.param.input_max), obj.param.input_min);

            %% ===== [补偿·可选] 外部PID补偿 (pidflag==1 时启用, 默认OFF) =====
            if obj.pidflag == 1
                % [修正] persistent → プロパティ化 (再実験時に積分値が残るバグを解消)
                e_pos = obj.state.ref(1:3, 1) - obj.current_state(1:3);
                e_vel = obj.state.ref(7:9, 1) - obj.current_state(7:9);
                e_yaw = obj.state.ref(6, 1) - obj.current_state(6);
                e_yaw_rate = obj.state.ref(12, 1) - obj.current_state(12);

                % [修正] 条件積分(anti-windup): 旧実装はif/else両分岐が同一処理で条件が無意味だった
                % 意図通り「xy誤差が大きい過渡中はxy積分を停止, zは常に積分」として実装
                if norm(e_pos(1:2)) < 0.2
                    obj.pos_integ = obj.pos_integ + e_pos * obj.param.dt;
                else
                    obj.pos_integ(3) = obj.pos_integ(3) + e_pos(3) * obj.param.dt;
                end
                obj.pos_integ = max(min(obj.pos_integ, [2.0; 2.0; 1.0]), -[2.0; 2.0; 1.0]);
                obj.yaw_integ = obj.yaw_integ + e_yaw * obj.param.dt;
                obj.yaw_integ = max(min(obj.yaw_integ, 0.5), -0.5);

                Kp_x = 0.20;  Kd_x = 0.50;  Ki_x = 0.05;
                Kp_y = 0.20;  Kd_y = 0.55;  Ki_y = 0.05;
                Kp_z = 2.0;   Kd_z = 1.0;   Ki_z = 0.5;
                Ki_yaw = 0.03; Kp_yaw = 0.2; Kd_yaw = 0.05;

                delta_u_pitch  =  (Kp_x * e_pos(1) + Kd_x * e_vel(1) + Ki_x * obj.pos_integ(1));
                delta_u_roll   = -(Kp_y * e_pos(2) + Kd_y * e_vel(2) + Ki_y * obj.pos_integ(2));
                delta_u_thrust =  (Kp_z * e_pos(3) + Kd_z * e_vel(3) + Ki_z * obj.pos_integ(3));
                delta_u_yaw    =   Kp_yaw * e_yaw + Kd_yaw * e_yaw_rate + Ki_yaw * obj.yaw_integ;

                obj.result.input(1) = obj.result.input(1) + delta_u_thrust;
                obj.result.input(2) = obj.result.input(2) + delta_u_roll;
                obj.result.input(3) = obj.result.input(3) + delta_u_pitch;
                obj.result.input(4) = obj.result.input(4) + delta_u_yaw;
                obj.result.input = max(min(obj.result.input, obj.param.input_max), obj.param.input_min);
            end

            %% ===== 更新结果和历史 =====
            obj.result.eflag         = eflag;
            obj.result.var           = var;
            obj.result.Bestcost_pre  = obj.result.bestcost;
            obj.result.bestcost      = [fval; 0];
            obj.result.kqlmpc        = obj.result.input;
            obj.input.pre_u          = obj.result.input;
            obj.result.pre_u         = obj.input.pre_u;

            %% ===== [补偿·可选] DOB 历史更新 (dobflag==1 时启用) =====
            if obj.dobflag == 1
                obj.z_prev_dob = obj.state.current;
                obj.u_prev_dob = obj.result.input; % [修正] 実際に印加する最終入力(補償込み)。旧コメント「不含PID/EDMD」は実装と矛盾していた
                obj.B_prev_dob = B_d;              % [修正] 次周期の予測用に同周期のBを対で保存
                % (旧: ここでz_predictedを再計算しデバッグ表示 → 同時刻のzと比較する誤った計算のため推定部へ移動済み)
            end

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

        %% =================================================================
        %  以下、补偿系 (flag/modeで有効化, デフォルトOFF)
        %% =================================================================
        function residual = initialize_residual_model(obj, param)
            residual = struct();
            residual.mode = 0;
            residual.loaded = false;
            residual.alpha = 1;
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
           u_hover_res = [obj.param.m * obj.param.gravity; 0; 0; 0]; % [修正] 残差学習は偏差座標 u~=u-hover
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
                    z_nom_next = obj.residual.A_nom * z_cur + obj.residual.B_nom *(u_nom - u_hover_res);
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
                    z_nom_next = obj.residual.A_nom * z_cur + obj.residual.B_nom *(u_nom - u_hover_res);
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

        %% =================================================================
        %  以下、解析的 Koopman モデル構成 (LPV-MPC核心の構成要素)
        %% =================================================================
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

        %% 目標軌道生成 (differential-flatness ベース)
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
                s = acc + [0;0;g];
                norm_s = norm(s); % 参考推力の大きさ (質量を掛けてxr(13)へ)
                % [整理] 参考姿态・角速度は現状「意図的にゼロ」(悬停系タスク前提)。
                % 旧実装はdifferential flatnessでeuler/wを計算した直後に破棄していた(死代码)。
                % 姿态・角速度前馈を使う場合は以下を有効化:
                yaw = 0; dyaw = 0;
                jerk = ref(13:15);
                b3 = s / max(norm_s, 1e-6);
                b1c = [cos(yaw); sin(yaw); 0];
                v = cross(b3, b1c);
                if norm(v) < 1e-6
                    if abs(b3(3)) < 0.9, temp_b1=[0;0;1]; else, temp_b1=[1;0;0]; end
                    b2 = cross(b3, temp_b1); b2 = b2/norm(b2);
                else
                    b2 = v/norm(v);
                end
                b1 = cross(b2, b3);
                Rd = [b1, b2, b3];
                euler = [atan2(Rd(3,2), Rd(3,3)); asin(-Rd(3,1)); atan2(Rd(2,1), Rd(1,1))];
                % hw = (jerk - dot(b3, jerk) * b3) / max(norm_s, 1e-6);
                % w = [-dot(hw, b2); dot(hw, b1); dot(b3, [0;0;1]) * dyaw];
                % % euler = [0; 0; 0];
                w = [0; 0; 0];
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
            fprintf('px=%.3f (err=%.3f) | pitch=%.4f | u(3)=%.4f\n', ...
                obj.current_state(1), ...
                obj.current_state(1) - obj.state.ref(1,1), ...
                obj.current_state(5), ...
                obj.result.input(3));
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
        function [ExA, ExB] = ExtendedCoefficientMatrix_LPV(obj, A_d, B_seq, Horizon)
            % [修正·新規] step毎に異なるB_kに対応した予測行列の構成 (真LPV)
            %   x_{k} = A^k x_0 + Σ_{j=1..k} A^(k-j) B_j u_{j}
            % B_seqが全て同一なら旧ExtendedCoefficientMatrixと数値的に一致する
            nx = size(A_d, 1);
            nu = size(B_seq{1}, 2);
            Apow = cell(Horizon + 1, 1); % Apow{k+1} = A_d^k
            Apow{1} = eye(nx);
            for k = 1:Horizon
                Apow{k+1} = A_d * Apow{k};
            end
            ExA = zeros(Horizon*nx, nx);
            for i = 1:Horizon
                ExA((i-1)*nx+1 : i*nx, :) = Apow{i+1};
            end
            ExB = zeros(Horizon*nx, Horizon*nu);
            for j = 1:Horizon
                Bj = B_seq{j};
                for i = j:Horizon
                    ExB((i-1)*nx+1 : i*nx, (j-1)*nu+1 : j*nu) = Apow{i-j+1} * Bj;
                end
            end
        end
    end
end
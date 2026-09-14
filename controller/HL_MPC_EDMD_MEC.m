classdef HL_MPC_EDMD_MEC < handle
    % HL_MPC with EDMD residual compensation.
    % 二層QP-MPC（z層 + x/y/yaw層）の最終入力 tmp に対し、
    % EDMD残差補償を外環として加える。
    % 補償ロジックは HLC_EDMD_MEC と同一（Mode2 + ローパス + alpha + du_max）。
    % MPCの最適性・予測・拘束は一切変更しない（純粋な外付け修正）。
    %
    % ------------------------------------------------------------------
    % 全体構成（カスケード）
    %   参照 xd ──► [内側] 二層QP-MPC ──► u_nom
    %                                        │
    %   EDMDリフト状態 z ──► [外環] 残差補償 ├─► Δu
    %                                        │
    %                       同定用激励 ──────┴─► Δu_exp
    %   最終入力: u = sat( u_nom + Δu + Δu_exp )
    %
    % 制御対象の想定モデル（離散・リフト空間）
    %   z_{k+1} ≈ (A_nom z_k + B_nom u_k) + (A_err z_k + B_err Δu_k)
    %              \___ 公称ダイナミクス ___/  \___ モデル誤差項 ___/
    %   残差補償は第2項で第1項の予測誤差を打ち消す Δu を求める。
    % ------------------------------------------------------------------

    properties
        self        % シミュレータ本体（plant / estimator / reference 等への参照）
        result      % 1ステップ分の出力・内部量をまとめた構造体
        param       % 制御パラメータ（mpc / residual / explore など）
        Vf          % z層（高度）の入力系列バッファ
        Vs          % x/y/yaw層の入力系列バッファ
        residual    % EDMD残差補償の設定・モデル・内部状態
        excite_counter = 0;   % 時刻が取れない場合の代替カウンタ（激励用）
        parameter_name = ["mass","Lx","Ly","lx","ly","jx","jy","jz","gravity","km1","km2","km3","km4","k1","k2","k3","k4"];
    end

    methods
        function obj = HL_MPC_EDMD_MEC(self, param)
            % コンストラクタ：機体パラメータ P の取得と各バッファの初期化
            obj.self = self;
            obj.param = param;
            obj.param.P = self.parameter.get(obj.parameter_name);  % 質量・慣性・アーム長など
            obj.result.input = zeros(self.estimator.model.dim(2), 1);
            obj.result.prev_vHL = zeros(4, 1);   % 直前の高レベル入力（差分ペナルティ用）
            obj.Vf = obj.param.Vf;
            obj.Vs = obj.param.Vs;
            obj.residual = obj.initialize_residual_model(param);   % EDMDモデル読み込み
        end

        function result = do(obj, varargin)
            % 1制御周期の処理。varargin{1}: 時刻情報, varargin{2}: フェーズ名
            phase = varargin{2};
            model = obj.self.estimator.result;   % 推定器の出力（状態推定値）
            P = obj.param.P;

            %% ===== ここから HL_MPC.do() と同一の二層QP-MPC =====
            H = obj.param.H;      % 予測ホライズン長
            dt = obj.param.dt;    % 制御周期
            pad_xd = @(x) [x(:); zeros(max(0, 20 - numel(x)), 1)];   % 参照を20次元に0埋め

            % 参照状態 xd = [p(1:4); v(5:8); a(9:12); j(13:16); s(17:20)]
            % （位置・速度・加速度・躍度・スナップ、各 x,y,z,yaw の4成分）
            xd0 = pad_xd(obj.self.reference.result.state.xd);
            xd_world = zeros(20, H);
            ref_obj = [];
            try
                % 現フェーズに割り当てられた参照生成器を取得
                ref_name = obj.self.cha_allocation.(char(phase)).reference;
                if ~isempty(ref_name)
                    ref_obj = obj.self.reference.(char(ref_name(1)));
                end
            catch
            end

            % --- ホライズン全域の参照軌道を生成 ---
            for j = 1:H
                tau = j * dt;   % 現在時刻からの先読み時間
                xd_j = [];
                try
                    % 参照生成器が使える場合は τ 先の値を直接評価する
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
                    % フォールバック：現在の微分値から Taylor 展開で外挿する
                    %   p(τ) = p + vτ + aτ²/2 + jτ³/6 + sτ⁴/24
                    %   v(τ) = v + aτ + jτ²/2 + sτ³/6
                    %   a(τ) = a + jτ + sτ²/2 ,  j(τ) = j + sτ
                    xd_j = xd0;
                    xd_j(1:4) = xd0(1:4) + xd0(5:8) * tau + xd0(9:12) * tau^2 / 2 + xd0(13:16) * tau^3 / 6 + xd0(17:20) * tau^4 / 24;
                    xd_j(5:8) = xd0(5:8) + xd0(9:12) * tau + xd0(13:16) * tau^2 / 2 + xd0(17:20) * tau^3 / 6;
                    xd_j(9:12) = xd0(9:12) + xd0(13:16) * tau + xd0(17:20) * tau^2 / 2;
                    xd_j(13:16) = xd0(13:16) + xd0(17:20) * tau;
                end
                xd_world(:, j) = pad_xd(xd_j);
            end
            xd = xd_world;   % 慣性座標系での参照軌道（この後ローカル系へ変換）

            % --- 参照ヨー角で定義される中間座標系 {b0} への変換 ---
            % Rb0 = Rz(ψd) とし、以降 x_local = Rb0' * x_world で扱う。
            % これによりヨー方向の非線形性を分離し、線形化の妥当性を高める。
            Rb0 = RodriguesQuaternion(Eul2Quat([0; 0; xd0(4)]));
            x = [R2q(Rb0' * model.state.getq("rotmat")); ...   % 姿勢（{b0}基準）
                 Rb0' * model.state.p; ...                     % 位置
                 Rb0' * model.state.v; ...                     % 速度
                 model.state.w];                               % 角速度（機体系のまま）

            % 現在参照 xd0 の並進系成分をすべて {b0} へ回す
            xd0(1:3)   = Rb0' * xd0(1:3);
            xd0(5:7)   = Rb0' * xd0(5:7);
            xd0(9:11)  = Rb0' * xd0(9:11);
            xd0(13:15) = Rb0' * xd0(13:15);
            xd0(17:19) = Rb0' * xd0(17:19);

            % ホライズン全体の参照も同様に {b0} へ変換
            for k = 1:size(xd, 2)
                xd(1:3, k)   = Rb0' * xd(1:3, k);
                xd(5:7, k)   = Rb0' * xd(5:7, k);
                xd(9:11, k)  = Rb0' * xd(9:11, k);
                xd(13:15, k) = Rb0' * xd(13:15, k);
                xd(17:19, k) = Rb0' * xd(17:19, k);
            end

            % ヨーは相対角で扱う。ψ ← wrap(ψ − ψ0) ∈ (−π, π]
            yaw0 = xd0(4);
            xd0(4) = 0;
            xd(4, :) = mod(xd(4, :) - yaw0 + pi, 2*pi) - pi;

            % QP ソルバのオプション（未設定なら表示なしの quadprog 既定）
            if isfield(obj.param.mpc, "opt")
                opt = obj.param.mpc.opt;
            else
                opt = optimoptions('quadprog', 'Display', 'off');
            end

            % --- 最新HL_MPCと同期: prev_vHL（入力差分平滑）と delay_step ---
            if ~isfield(obj.result, "prev_vHL") || numel(obj.result.prev_vHL) ~= 4
                obj.result.prev_vHL = zeros(4, 1);
            end
            if isfield(obj.param.mpc, "delay_step")
                % むだ時間補償：ホライズン内の delay_step 先の入力を実際に印加する
                delay_step = max(0, round(obj.param.mpc.delay_step));
            else
                delay_step = 0;
            end

            % ---- z層 QP ----
            % 高度方向の誤差ダイナミクス z1 = [e_z; ė_z] を先に解き、
            % その解 vf を下位層（x/y/yaw）の線形化点として渡す（カスケード構造）。
            z1 = Z1(x, xd0', P);
            Sx = obj.param.mpc.Sx{1};    % 予測行列：Z = Sx*z + Su*V
            Su = obj.param.mpc.Su{1};
            Qb = obj.param.mpc.Qb{1};    % ブロック対角の状態重み
            Hq = obj.param.mpc.Hq{1};    % Hq = 2(Su'*Qb*Su + Rb)
            Hq = (Hq + Hq') / 2;         % 数値誤差による非対称を除去
            % コスト J = V'HqV/2 + f'V,  f = 2*Su'*Qb*Sx*z
            f = 2 * Su' * Qb * Sx * z1;
            % Rd0: 直前入力との差分ペナルティ（最新HL_MPCと同期）
            % ΔJ = Rd0*(v0 − v_prev)² を加える ⇒ Hq(1,1) += 2Rd0, f(1) −= 2Rd0*v_prev
            Rd0 = obj.param.mpc.Rd0{1};
            Hq(1,1) = Hq(1,1) + 2 * Rd0;
            f(1) = f(1) - 2 * Rd0 * obj.result.prev_vHL(1);
            V = quadprog(Hq, f, [], [], [], [], obj.param.mpc.lbq{1}, obj.param.mpc.ubq{1}, [], opt);
            if isempty(V); V = zeros(obj.param.mpc.Nvf, 1); end   % 解が出ない場合は0入力
            vf = V(1:obj.param.mpc.Nvf)';
            Vf_seq = V(:);
            z1_pred = reshape(Sx * z1 + Su * Vf_seq, 2, []);       % 可視化用の予測状態列

            % ---- x/y/yaw層 QP ----
            % z層の解 vf（推力方向）を既知として、各軸の誤差系 z2,z3,z4 を独立に解く
            z2 = Z2(x, xd0', vf, P);   % x 方向
            z3 = Z3(x, xd0', vf, P);   % y 方向
            z4 = Z4(x, xd0', vf, P);   % yaw
            Zs = {z2, z3, z4};
            vs = zeros(3, 1);
            Vs_seq = cell(3, 1);
            Zs_pred = cell(3, 1);
            for s = 1:3
                id = s + 1;   % param.mpc 内のインデックス（1がz層、2〜4がx/y/yaw層）
                Sx = obj.param.mpc.Sx{id};
                Su = obj.param.mpc.Su{id};
                Qb = obj.param.mpc.Qb{id};
                Hq = obj.param.mpc.Hq{id};
                Hq = (Hq + Hq') / 2;
                f = 2 * Su' * Qb * Sx * Zs{s};
                % z層と同じく入力変化率にペナルティを付ける
                Rd0 = obj.param.mpc.Rd0{id};
                Hq(1,1) = Hq(1,1) + 2 * Rd0;
                f(1) = f(1) - 2 * Rd0 * obj.result.prev_vHL(id);
                V = quadprog(Hq, f, [], [], [], [], obj.param.mpc.lbq{id}, obj.param.mpc.ubq{id}, [], opt);
                if isempty(V); V = zeros(obj.param.mpc.N, 1); end
                Vs_seq{s} = V(:);
                nx = size(obj.param.mpc.A{id}, 1);
                Zs_pred{s} = reshape(Sx * Zs{s} + Su * Vs_seq{s}, nx, []);
                % むだ時間分だけ先の要素を採用（delay_step=0 なら通常の receding horizon）
                apply_id = min(1 + delay_step, numel(Vs_seq{s}));
                vs(s) = Vs_seq{s}(apply_id);
            end

            % 高レベル入力 (vf, vs) を実入力 u = [T; τx; τy; τz] へ逆変換
            %   u = Uf(vf) + Us(vf, vs)   （フィードフォワード項 + フィードバック項）
            tmp = Uf(x, xd0', vf, P) + Us(x, xd0', vf, vs, P);
            % prev_vHL を更新（次ステップの差分平滑用）
            obj.result.prev_vHL = [vf(1); vs];
            %% ===== ここまで 最新HL_MPC と同一。tmp が標称MPC最適入力 =====

            % 標称入力（補償前、クリップ済み）。残差の u はこれを使う。
            % 飽和域: u1 ∈ [0,10]（推力）, u2..u4 ∈ [−1,1]（各軸モーメント）
            u_nom = [max(0,  min(10, tmp(1))); ...
                     max(-1, min(1,  tmp(2))); ...
                     max(-1, min(1,  tmp(3))); ...
                     max(-1, min(1,  tmp(4)))];

            %% ===== EDMD残差補償 外環（HLC_EDMD_MEC と同一ロジック）=====
            % Δu を求め、有効時のみ u_nom に加算する
            delta_u_pred = obj.compute_residual_delta_u(u_nom);
            if obj.residual.apply_compensation == 1
                delta_u = delta_u_pred;
            else
                delta_u = zeros(4, 1);   % 比較実験用：Δuを計算だけして印加しない
            end
            u_comp_raw = u_nom + delta_u;
            u_comp = [max(0,  min(10, u_comp_raw(1))); ...
                      max(-1, min(1,  u_comp_raw(2))); ...
                      max(-1, min(1,  u_comp_raw(3))); ...
                      max(-1, min(1,  u_comp_raw(4)))];

            % 劣化飛行検証用の任意アクチュエータ外乱（激励）
            % 同定データの持続励振性（PE性）を確保するための付加信号
            delta_u_explore = obj.generate_identification_excitation(u_comp, phase, varargin{1});
            u_total = u_comp + delta_u_explore;
            u_total = [max(0,  min(10, u_total(1))); ...
                       max(-1, min(1,  u_total(2))); ...
                       max(-1, min(1,  u_total(3))); ...
                       max(-1, min(1,  u_total(4)))];

            %% ===== result =====
            % 解析・作図のため、各段階の入力とその差分を個別に保存する
            obj.result.u_nom = u_nom;                     % 補償前のMPC入力
            obj.result.uHL = [vf(1); vs];                 % 高レベル入力
            obj.result.vf = vf;
            obj.result.z1 = z1;
            obj.result.z2 = z2;
            obj.result.z3 = z3;
            obj.result.z4 = z4;
            obj.result.deltau = delta_u + delta_u_explore;      % 付加入力の合計
            obj.result.delta_u_edmd = delta_u;                  % 実際に印加したΔu
            obj.result.delta_u_explore = delta_u_explore;       % 激励成分
            obj.result.delta_u_edmd_pred = delta_u_pred;        % 計算値（印加の有無を問わず）
            obj.result.u_comp_raw = u_comp_raw;                 % 飽和前
            obj.result.u_comp = u_comp;                         % 飽和後
            obj.result.delta_u_edmd_cmd = delta_u;              % 指令値
            obj.result.delta_u_edmd_actual = u_comp - u_nom;    % 飽和後に実現した差分
            obj.result.delta_u_edmd_saturation_loss = delta_u - (u_comp - u_nom);  % 飽和で失われた分
            obj.result.delta_u_total_actual = u_total - u_nom;
            obj.result.u_explore = delta_u_explore;
            obj.result.input_before_explore = u_comp;
            obj.result.u_raw = u_total;
            obj.result.input = u_total;      % プラントへ渡す最終入力
            obj.result.hlmpc = u_total;
            obj.result.pre_u = u_total;
            obj.result.residual_mode = obj.residual.mode;
            obj.result.residual_loaded = obj.residual.loaded;
            obj.result.residual_apply_compensation = obj.residual.apply_compensation;

            % 予測軌跡（動画・可視化用、HL_MPC と同じ）
            % 各層で長さが異なりうるので最短に合わせ、参照 + 誤差予測 で絶対位置を再構成
            Npred = min([size(z1_pred, 2), size(Zs_pred{1}, 2), size(Zs_pred{2}, 2)]);
            pred_local = [xd(1, 1:Npred) + Zs_pred{1}(1, 1:Npred); ...   % x = xd + e_x
                          xd(2, 1:Npred) + Zs_pred{2}(1, 1:Npred); ...   % y = yd + e_y
                          xd(3, 1:Npred) + z1_pred(1, 1:Npred)];         % z = zd + e_z
            obj.result.hlmpc_pred_pos = Rb0 * pred_local;   % {b0} → 慣性系へ戻す
            obj.result.hlmpc_pred_local = pred_local;
            obj.result.hlmpc_ref_pos = xd_world(1:3, 1:Npred);

            result = obj.result;
            if isfield(obj.param, 'enable_show') && obj.param.enable_show
                fprintf('controller: HL_MPC_EDMD_MEC,  phase: %s \n', phase);
                obj.show();
            end
        end

        %% ============ 以下 HLC_EDMD_MEC からそのまま移植 ============

        function residual = initialize_residual_model(obj, param)
            % EDMD残差補償の初期化。
            % 既定値を並べたうえで param.residual で上書きし、最後にモデルを読み込む。
            residual = struct();
            residual.mode = 0;              % 0:無効, 1:LQR, 2:予測誤差打消, 3:誤差項のみ抑制
            residual.loaded = false;        % モデル読み込み成否
            residual.model_file = '';       % results_edmd を含む .mat のパス
            residual.state_source = 'estimator';  % 'estimator' or 'plant'
            residual.alpha = 1.0;           % Δu 全体のゲイン（保守側に落とす用）
            residual.du_max = [0; 0; 0; 0]; % チャネル別の Δu 上限（安全のため既定0）
            residual.pinv_damping = 1e-3;   % 最小二乗の正則化係数 λ
            residual.q_scale = 10.0;        % mode1 の dlqr 状態重み Q = q_scale·I
            residual.r_scale = 0.1;         % mode1 の dlqr 入力重み R = r_scale·I
            residual.beta = 1.0;            % ローパス係数（1で平滑なし）
            residual.apply_compensation = 1;% 0なら計算のみで印加しない
            residual.active_channels = [1; 1; 1; 1];  % 補償を許すチャネル
            residual.prev_delta_u = zeros(4, 1);      % ローパスの内部状態
            residual.max_pos_err = inf;     % 以下4つは補償を許す誤差エンベロープ
            residual.max_ang_err = inf;
            residual.max_vel_err = inf;
            residual.max_w_err = inf;
            residual.A_nom = [];            % 公称リフト系 z⁺ = A_nom z + B_nom u
            residual.B_nom = [];
            residual.A_err = [];            % 誤差リフト系 Δz⁺ = A_err z + B_err Δu
            residual.B_err = [];
            residual.K = [];                % mode1 の状態フィードバックゲイン
            residual.dlqr_ok = false;
            residual.use_reference = 1;

            if ~isfield(param, 'residual')
                return;
            end
            % ユーザ設定で既定値を上書き
            fns = fieldnames(param.residual);
            for i = 1:numel(fns)
                residual.(fns{i}) = param.residual.(fns{i});
            end
            if residual.mode == 0 || ~isfield(param.residual, 'model_file')
                return;   % 無効設定ならモデルを読まない
            end
            try
                % 事前にEDMD同定した行列を読み込む
                mdl = load(param.residual.model_file, 'results_edmd');
                residual.A_nom = mdl.results_edmd.A_nom;
                residual.B_nom = mdl.results_edmd.B_nom;
                residual.A_err = mdl.results_edmd.A_err;
                residual.B_err = mdl.results_edmd.B_err;
                residual.loaded = true;
                % mode1 用：誤差系に対する離散LQR
                %   J = Σ (z'Qz + Δu'RΔu) を最小化する K
                q_res = eye(size(residual.A_err, 1)) * residual.q_scale;
                r_res = eye(size(residual.B_err, 2)) * residual.r_scale;
                try
                    residual.K = dlqr(residual.A_err, residual.B_err, q_res, r_res);
                    residual.dlqr_ok = true;
                catch
                    % 可安定でない等でdlqrが失敗した場合はゲイン0（＝無補償）に落とす
                    residual.K = zeros(size(residual.B_err, 2), size(residual.A_err, 1));
                    residual.dlqr_ok = false;
                end
            catch ME
                warning('Residual model load failed: %s', ME.message);
                residual.mode = 0;   % 読み込み失敗時は安全側（補償なし）へ
            end
        end

        function delta_u = compute_residual_delta_u(obj, u_nom)
            % 残差補償入力 Δu の算出（本手法の中核）。
            % 手順: 状態取得 → エンベロープ判定 → リフト → モード別に Δu_raw
            %       → ローパス → αゲイン → 飽和
            delta_u = zeros(4, 1);
            obj.result.residual_rhs_norm = 0;
            obj.result.residual_raw_du_norm = 0;
            obj.result.residual_filtered_du_norm = 0;
            obj.result.residual_final_du_norm = 0;
            obj.result.residual_envelope_ok = 0;
            if isempty(obj.residual) || ~isfield(obj.residual, 'mode') || obj.residual.mode == 0
                return;   % 補償無効
            end
            if ~obj.residual.loaded
                return;   % モデル未読み込み
            end
            x_cur = obj.get_residual_state();     % 現在状態 x ∈ R¹²
            x_ref = obj.get_reference_state();    % 参照状態 x_ref ∈ R¹²
            if ~obj.within_compensation_envelope(x_cur, x_ref)
                return;   % 誤差が大きすぎる領域ではEDMDの外挿を信用しない
            end
            obj.result.residual_envelope_ok = 1;
            % 観測量をリフト： z = ψ(x, u)
            z_cur = obj.klift_edmd_residual([x_cur; u_nom(:)]);
            z_ref = obj.klift_edmd_residual([x_ref; u_nom(:)]);
            rhs = zeros(size(obj.residual.B_err, 1), 1);
            delta_u_raw = zeros(4, 1);
            switch obj.residual.mode
                case 1
                    % 誤差系に対する状態フィードバック： Δu = −K(z − z_ref)
                    z_err = z_cur - z_ref;
                    delta_u_raw = -obj.residual.K * z_err;
                case 2
                    % 1ステップ先の予測誤差を打ち消す（本命）
                    %   z⁺_nom = A_nom z + B_nom u
                    %   目標:  A_err z + B_err Δu ≈ z_ref − z⁺_nom
                    %   ⇒ B_err Δu ≈ (z_ref − z⁺_nom) − A_err z  =: rhs
                    z_nom_next = obj.residual.A_nom * z_cur + obj.residual.B_nom * u_nom;
                    z_target_bar = z_ref - z_nom_next;
                    rhs = z_target_bar - obj.residual.A_err * z_cur;
                    delta_u_raw = obj.solve_active_delta_u(rhs);
                case 3
                    % 参照を使わず、モデル誤差項そのものを打ち消す
                    %   B_err Δu ≈ −A_err z
                    rhs = -obj.residual.A_err * z_cur;
                    delta_u_raw = obj.solve_active_delta_u(rhs);
                otherwise
                    delta_u_raw = zeros(4, 1);
            end
            obj.result.residual_rhs_norm = norm(rhs);
            obj.result.residual_raw_du_norm = norm(delta_u_raw);
            if any(~isfinite(delta_u_raw))
                delta_u_raw = zeros(4, 1);   % NaN/Inf 混入時のフェイルセーフ
            end
            % 一次ローパス： Δu_k = (1−β)Δu_{k−1} + β Δu_raw
            % 高周波成分を抑え、アクチュエータのチャタリングを防ぐ
            beta = obj.residual.beta;
            delta_u = (1 - beta) * obj.residual.prev_delta_u + beta * delta_u_raw;
            obj.residual.prev_delta_u = delta_u;
            obj.result.residual_filtered_du_norm = norm(delta_u);
            delta_u = obj.residual.alpha * delta_u;              % 全体ゲイン α
            % チャネル別飽和： Δu ← clip(Δu, ±du_max)
            delta_u = max(min(delta_u, obj.residual.du_max), -obj.residual.du_max);
            obj.result.residual_final_du_norm = norm(delta_u);
        end

        function ok = within_compensation_envelope(obj, x_cur, x_ref)
            % 補償を許す誤差範囲の判定。
            % 学習データの分布から離れた領域ではEDMDの予測が外れるため、
            % 位置・姿勢・速度・角速度の誤差がすべて閾値内のときのみ補償する。
            e = zeros(12, 1);
            e(1:3) = x_cur(1:3) - x_ref(1:3);                       % 位置誤差
            e(4:6) = obj.wrap_to_pi(x_cur(4:6) - x_ref(4:6));       % 姿勢誤差（±πに正規化）
            e(7:9) = x_cur(7:9) - x_ref(7:9);                       % 速度誤差
            e(10:12) = x_cur(10:12) - x_ref(10:12);                 % 角速度誤差
            ok = norm(e(1:3)) <= obj.residual.max_pos_err && ...
                 all(abs(e(4:6)) <= obj.residual.max_ang_err) && ...
                 norm(e(7:9)) <= obj.residual.max_vel_err && ...
                 norm(e(10:12)) <= obj.residual.max_w_err;
        end

        function delta_u = solve_active_delta_u(obj, rhs)
            % B_err Δu = rhs を有効チャネルのみで解く（Tikhonov正則化付き最小二乗）
            %   min ‖B_e Δu − rhs‖² + λ‖Δu‖²
            %   ⇒ Δu = (B_eᵀB_e + λI)⁻¹ B_eᵀ rhs
            % λ = pinv_damping。B_err の条件数が悪い場合の発散を防ぐ。
            delta_u = zeros(4, 1);
            active_channels = logical(obj.residual.active_channels(:));
            if numel(active_channels) ~= 4
                active_channels = [true; true; true; true];
            end
            idx = find(active_channels);
            if isempty(idx)
                return;   % 全チャネル無効なら補償なし
            end
            Be = obj.residual.B_err(:, idx);
            reg = obj.residual.pinv_damping * eye(size(Be, 2));
            delta_active = (Be' * Be + reg) \ (Be' * rhs);
            delta_u(idx) = delta_active;   % 無効チャネルは0のまま
        end

        function x12 = get_residual_state(obj)
            % 補償に使う現在状態 x = [p; Θ; v; ω] ∈ R¹² を取得。
            % 'plant' は真値（デバッグ用）、既定の 'estimator' は推定値（実機相当）。
            source = lower(obj.residual.state_source);
            switch source
                case 'plant'
                    st = obj.self.plant.state;
                otherwise
                    st = obj.self.estimator.result.state;
            end
            q = st.getq('euler');   % 姿勢はオイラー角で取り出す
            x12 = [double(st.p(:)); double(q(:)); double(st.v(:)); double(st.w(:))];
        end

        function x12 = get_reference_state(obj)
            % 参照側の x_ref ∈ R¹²。参照生成器によって姿勢・角速度を
            % 持たない場合があるので、無ければ xd から補うか0とする。
            sr = obj.self.reference.result.state;
            if (isstruct(sr) && isfield(sr, 'q')) || isprop(sr, 'q')
                qref = double(sr.q(:));
            else
                qref = double(sr.xd(4:6));
            end
            if (isstruct(sr) && isfield(sr, 'w')) || isprop(sr, 'w')
                wref = double(sr.w(:));
            else
                wref = zeros(3, 1);   % 角速度参照なし ⇒ 0
            end
            x12 = [double(sr.p(:)); double(qref(:)); double(sr.v(:)); wref];
        end

        function z = klift_edmd_residual(obj, x16)
            % EDMD の観測関数（リフト写像） z = ψ(x, u)。
            % 入力 x16 = [p(1:3); Θ(4:6); v(7:9); ω(10:12); u(13:16)]。
            % 同定時と完全に同じ基底でなければならない点に注意。
            P1 = x16(1); P2 = x16(2); P3 = x16(3);
            Q1 = x16(4); Q2 = x16(5); Q3 = x16(6);      % roll, pitch, yaw
            V1 = x16(7); V2 = x16(8); V3 = x16(9);
            W1 = x16(10); W2 = x16(11); W3 = x16(12);
            c1 = cos(Q1); s1 = sin(Q1);
            c2 = cos(Q2); s2 = sin(Q2);
            c3 = cos(Q3); s3 = sin(Q3);
            % ジンバルロック近傍（cos≈0）でのゼロ割を回避
            c1_safe = obj.safe_nonzero(c1, 1e-3);
            c2_safe = obj.safe_nonzero(c2, 1e-3);
            % 回転行列の第3列（推力方向ベクトル）。並進の非線形性を表す主要基底。
            R13 = c3 * s2 * c1 + s3 * s1;
            R23 = s3 * s2 * c1 - c3 * s1;
            R33 = c2 * c1;
            % 線形基底 + 推力方向 + 定数項（アフィン項を表現するための 1）
            common_z = [P1; P2; P3; Q1; Q2; Q3; V1; V2; V3; W1; W2; W3; ...
                        R13; R23; R33; 1];
            % 非線形基底：ジャイロ項 ω×Jω と、オイラー角運動学 Θ̇ = T(Θ)ω の展開項
            kyo_z = [W1 * W2; W2 * W3; W3 * W1; ...                     % ジャイロ結合項
                     W2 * c1; W3 * s1; ...                              % 姿勢×角速度
                     W1 * c2 / c1_safe; W2 * s1 / c2_safe; W3 * c1 / c2_safe; ...
                     W2 * s1 * s2 / c2_safe; W3 * c1 * s2 / c2_safe];   % T(Θ) 由来の項
            z = [common_z; kyo_z];
        end

        function y = safe_nonzero(~, x, eps_val)
            % |x| < eps のとき符号を保ったまま eps に置換（ゼロ割防止）
            if abs(x) < eps_val
                if x >= 0; y = eps_val; else; y = -eps_val; end
            else
                y = x;
            end
        end

        function ang = wrap_to_pi(~, ang)
            % 角度を (−π, π] に正規化
            ang = mod(ang + pi, 2 * pi) - pi;
        end

        function du = generate_identification_excitation(obj, u_cmd, phase, time_info)
            % 同定用の激励信号。EDMDの学習データに十分な周波数成分を与える。
            % 既定は多正弦（multisine）：
            %   Δu_i(τ) = Σ_j a_ij sin(2π f_ij τ + φ_ij),  τ = t − t_start
            % 立ち上がりは ramp = min(1, τ/T_ramp) で滑らかにする。
            cfg = obj.get_explore_config();
            du = zeros(4, 1);
            active = obj.get_explore_active_flag(phase);
            t = obj.get_time_value(time_info);
            tau = max(0, t - cfg.start_time);
            ramp = min(1.0, tau / max(cfg.ramp_time, 1e-6));
            if ~cfg.enable
                return;   % 激励無効
            end
            if cfg.flight_only && ~active
                return;   % 離着陸中は入れない（飛行フェーズ限定）
            end
            switch lower(char(string(cfg.mode)))
                case 'legacy_relative_random'
                    % 旧方式：入力に比例した一様乱数（周波数特性は不定）
                    du = cfg.legacy_scale .* abs(u_cmd) .* (2 * rand(4, 1) - 1);
                otherwise
                    % 多正弦：チャネルごとに異なる周波数を割り当て、相関を避ける
                    du(1) = sum(cfg.u1_amp(:) .* sin(2 * pi * cfg.u1_freq(:) * tau + cfg.u1_phase(:)));
                    du(2) = sum(cfg.u2_amp(:) .* sin(2 * pi * cfg.u2_freq(:) * tau + cfg.u2_phase(:)));
                    du(3) = sum(cfg.u3_amp(:) .* sin(2 * pi * cfg.u3_freq(:) * tau + cfg.u3_phase(:)));
                    du(4) = sum(cfg.u4_amp(:) .* sin(2 * pi * cfg.u4_freq(:) * tau + cfg.u4_phase(:)));
                    du = ramp * du;
            end
            du = max(min(du, cfg.du_cap(:)), -cfg.du_cap(:));   % 振幅上限で飽和
        end

        function cfg = get_explore_config(obj)
            % 激励信号の既定設定。param.explore があれば上書きする。
            cfg = struct();
            cfg.enable = 0;                        % 既定は無効
            cfg.flight_only = 1;                   % 飛行フェーズのみ
            cfg.mode = 'structured_multisine';
            cfg.start_time = 0.0;
            cfg.ramp_time = 1.0;                   % 立ち上がり時間 [s]
            cfg.legacy_scale = 0.2 * ones(4, 1);
            % 各チャネル2成分の多正弦（振幅 / 周波数[Hz] / 位相[rad]）
            cfg.u1_amp = [0.35; 0.15];             % 推力：振幅は大きめ
            cfg.u1_freq = [0.35; 0.90];
            cfg.u1_phase = [0.20; 1.10];
            cfg.u2_amp = [0.010; 0.005];           % ロール
            cfg.u2_freq = [0.55; 1.25];
            cfg.u2_phase = [0.40; 1.30];
            cfg.u3_amp = [0.010; 0.005];           % ピッチ
            cfg.u3_freq = [0.70; 1.55];
            cfg.u3_phase = [0.90; 0.30];
            cfg.u4_amp = [0.0010; 0.0005];         % ヨー：姿勢を乱さぬよう最小
            cfg.u4_freq = [0.45; 1.05];
            cfg.u4_phase = [0.50; 1.40];
            cfg.du_cap = [0.55; 0.018; 0.018; 0.003];   % 安全上の振幅上限
            if isfield(obj.param, 'explore')
                fns = fieldnames(obj.param.explore);
                for i = 1:numel(fns)
                    cfg.(fns{i}) = obj.param.explore.(fns{i});
                end
            end
            if isfield(obj.param, 'edmd_disturbance_enable')
                cfg.enable = logical(obj.param.edmd_disturbance_enable);   % 旧フラグ互換
            end
        end

        function t = get_time_value(obj, time_info)
            % 現在時刻の取得。時刻情報が無い場合はステップ数×dtで代用する。
            if nargin >= 2 && isstruct(time_info) && isfield(time_info, 't')
                t = double(time_info.t);
                return;
            end
            obj.excite_counter = obj.excite_counter + 1;
            if isfield(obj.param, 'dt') && ~isempty(obj.param.dt)
                t = (obj.excite_counter - 1) * obj.param.dt;
            else
                t = obj.excite_counter - 1;
            end
        end

        function active = get_explore_active_flag(~, phase)
            % フェーズ名が 'f'（flight）で始まるときのみ激励を有効とみなす
            if isstring(phase) || ischar(phase)
                phase_str = lower(char(phase));
                active = ~isempty(phase_str) && phase_str(1) == 'f';
            else
                active = true;
            end
        end

        function show(obj)
            % デバッグ表示：推定状態・参照状態と、補償前後の入力を1行で出力
            est_print = obj.self.estimator.result.state;
            ref_print = obj.self.reference.result.state;
            fprintf("==================================================================\n")
            fprintf("ps: %f %f %f \t vs: %f %f %f \t qs: %f %f %f \n",...
                est_print.p(1), est_print.p(2), est_print.p(3),...
                est_print.v(1), est_print.v(2), est_print.v(3),...
                est_print.q(1), est_print.q(2), est_print.q(3));
            fprintf("pr: %f %f %f \t vr: %f %f %f \n", ...
                ref_print.p(1), ref_print.p(2), ref_print.p(3),...
                ref_print.v(1), ref_print.v(2), ref_print.v(3));
            fprintf("comp_on: %d | u_nom: %.3f %.3f %.3f %.3f | du_pred: %.3f %.3f %.3f %.3f | u: %.3f %.3f %.3f %.3f\n", ...
                obj.result.residual_apply_compensation, ...
                obj.result.u_nom(1), obj.result.u_nom(2), obj.result.u_nom(3), obj.result.u_nom(4), ...
                obj.result.delta_u_edmd_pred(1), obj.result.delta_u_edmd_pred(2), obj.result.delta_u_edmd_pred(3), obj.result.delta_u_edmd_pred(4), ...
                obj.result.input(1), obj.result.input(2), obj.result.input(3), obj.result.input(4));
        end
    end
end
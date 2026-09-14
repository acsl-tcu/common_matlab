classdef HLC_EDMD_MEC < handle
    % EDMD残差モデルを用いてHLC入力を補償する制御器

    properties
        self
        result
        param
        residual
        excite_counter = 0;
    end

    methods
        function obj = HLC_EDMD_MEC(self, param)
            % コントローラを初期化し，EDMD残差モデルを読み込む
            obj.self = self;
            obj.param = param;
            obj.param.P = self.parameter.get();
            obj.result.input = zeros(self.estimator.model.dim(2), 1);
            obj.residual = obj.initialize_residual_model(param);
        end

        function result = do(obj, varargin)
            % HLC入力にEDMD残差補償と必要な励振入力を加えて出力する
            phase = varargin{2};

            model = obj.self.estimator.result;
            ref = obj.self.reference.result;
            xd = ref.state.xd;

            P = obj.param.P;
            F1 = obj.param.F1;
            F2 = obj.param.F2;
            F3 = obj.param.F3;
            F4 = obj.param.F4;

            xd = [xd; zeros(20 - size(xd, 1), 1)];
            % 参照状態ベクトルが不足する場合は0で補う

            Rb0 = RodriguesQuaternion(Eul2Quat([0; 0; xd(4)]));
            x = [R2q(Rb0' * model.state.getq("rotmat")); Rb0' * model.state.p; Rb0' * model.state.v; model.state.w];
            % 目標yaw角を基準とした座標系へ現在状態を変換する

            xd(1:3) = Rb0' * xd(1:3);
            xd(4) = 0;
            xd(5:7) = Rb0' * xd(5:7);
            xd(9:11) = Rb0' * xd(9:11);
            xd(13:15) = Rb0' * xd(13:15);
            xd(17:19) = Rb0' * xd(17:19);
            % 参照状態も同じyaw基準座標系に変換する

            if isfield(varargin{1}, 'dt') && varargin{1}.dt <= obj.param.dt
                dt = varargin{1}.dt;
                vf = Vfd(dt, x, xd', P, F1);
                vs = Vsd(dt, x, xd', vf, P, F2, F3, F4);
            else
                vf = Vf(x, xd', P, F1);
                vs = Vs(x, xd', vf, P, F2, F3, F4);
            end
            % サンプリング時間に応じて仮想入力を計算する

            tmp = Uf(x, xd', vf, P) + Us(x, xd', vf, vs', P);
            % 高階線形化に基づくHLC入力を計算する

            u_nom = [max(0, min(10, tmp(1))); ...
                     max(-1, min(1, tmp(2))); ...
                     max(-1, min(1, tmp(3))); ...
                     max(-1, min(1, tmp(4)))];
            % 安全のため，HLC入力を上下限で制限する

            delta_u_pred = obj.compute_residual_delta_u(u_nom);
            % EDMD残差モデルに基づく補償入力を計算する

            if obj.residual.apply_compensation == 1
                delta_u = delta_u_pred;
            else
                delta_u = zeros(4, 1);
            end
            % 補償を適用するかどうかを切り替える

            u_comp = u_nom + delta_u;
            u_comp = [max(0, min(10, u_comp(1))); ...
                      max(-1, min(1, u_comp(2))); ...
                      max(-1, min(1, u_comp(3))); ...
                      max(-1, min(1, u_comp(4)))];
            % HLC入力にEDMD補償を加えた入力を制限する

            delta_u_explore = obj.generate_identification_excitation(u_comp, phase, varargin{1});
            % 必要に応じて，同定・検証用の励振入力を生成する

            u_total = u_comp + delta_u_explore;
            u_total = [max(0, min(10, u_total(1))); ...
                       max(-1, min(1, u_total(2))); ...
                       max(-1, min(1, u_total(3))); ...
                       max(-1, min(1, u_total(4)))];
            % 最終入力にも上下限制限を行う

            obj.result.u_nom = u_nom;
            obj.result.deltau = delta_u + delta_u_explore;
            obj.result.delta_u_edmd = delta_u;
            obj.result.delta_u_explore = delta_u_explore;
            obj.result.delta_u_edmd_pred = delta_u_pred;
            obj.result.delta_u_edmd_applied = delta_u;
            obj.result.delta_u_edmd_raw = obj.residual.prev_delta_u;
            obj.result.u_explore = delta_u_explore;
            obj.result.input_before_explore = u_comp;
            obj.result.u_raw = u_total;
            obj.result.input = u_total;
            obj.result.hlc = u_total;
            obj.result.pre_u = u_total;
            obj.result.hlc_x = x;
            obj.result.hlc_xd = xd;
            obj.result.residual_mode = obj.residual.mode;
            obj.result.residual_loaded = obj.residual.loaded;
            obj.result.residual_apply_compensation = obj.residual.apply_compensation;
            obj.result.explore_active = obj.get_explore_active_flag(phase);
            obj.result.explore_mode = obj.get_explore_config().mode;
            % 制御入力，補償入力，励振入力，診断情報を保存する

            result = obj.result;
            if isfield(obj.param, 'enable_show') && obj.param.enable_show
                fprintf('controller: HLC_EDMD_MEC,  phase: %s \n', phase);
                obj.show();
            end
        end

        function residual = initialize_residual_model(obj, param)
            % EDMD残差補償器の初期値を設定し，学習済みモデルを読み込む
            residual = struct();
            residual.mode = 0;
            residual.loaded = false;
            residual.model_file = '';
            residual.state_source = 'estimator';
            residual.alpha = 1.0;
            residual.du_max = [0; 0; 0; 0];
            residual.pinv_damping = 1e-3;
            residual.q_scale = 10.0;
            residual.r_scale = 0.1;
            residual.beta = 1.0;
            residual.apply_compensation = 1;
            residual.active_channels = [1; 1; 1; 1];
            residual.prev_delta_u = zeros(4, 1);
            residual.max_pos_err = inf;
            residual.max_ang_err = inf;
            residual.max_vel_err = inf;
            residual.max_w_err = inf;
            residual.A_nom = [];
            residual.B_nom = [];
            residual.A_err = [];
            residual.B_err = [];
            residual.K = [];
            residual.dlqr_ok = false;
            residual.use_reference = 1;

            if ~isfield(param, 'residual')
                return;
            end
            % residual設定がない場合は補償を無効のままにする

            fns = fieldnames(param.residual);
            for i = 1:numel(fns)
                residual.(fns{i}) = param.residual.(fns{i});
            end
            % param.residual の設定でデフォルト値を上書きする

            if residual.mode == 0 || ~isfield(param.residual, 'model_file')
                return;
            end
            % modeが0，またはモデルファイルがない場合は読み込まない

            try
                mdl = load(param.residual.model_file, 'results_edmd');
                residual.A_nom = mdl.results_edmd.A_nom;
                residual.B_nom = mdl.results_edmd.B_nom;
                residual.A_err = mdl.results_edmd.A_err;
                residual.B_err = mdl.results_edmd.B_err;
                residual.loaded = true;

                q_res = eye(size(residual.A_err, 1)) * residual.q_scale;
                r_res = eye(size(residual.B_err, 2)) * residual.r_scale;
                try
                    residual.K = dlqr(residual.A_err, residual.B_err, q_res, r_res);
                    residual.dlqr_ok = true;
                catch
                    residual.K = zeros(size(residual.B_err, 2), size(residual.A_err, 1));
                    residual.dlqr_ok = false;
                end
                % mode 1 用のLQRゲインを計算する
            catch ME
                warning('Residual model load failed: %s', ME.message);
                residual.mode = 0;
            end
        end

        function delta_u = compute_residual_delta_u(obj, u_nom)
            % EDMD残差モデルを用いて補償入力を計算する
            delta_u = zeros(4, 1);

            if isempty(obj.residual) || ~isfield(obj.residual, 'mode') || obj.residual.mode == 0
                return;
            end
            if ~obj.residual.loaded
                return;
            end
            % 補償が無効，またはモデル未読込の場合は0を返す

            x_cur = obj.get_residual_state();
            x_ref = obj.get_reference_state();
            if ~obj.within_compensation_envelope(x_cur, x_ref)
                return;
            end
            % 状態誤差が大きすぎる場合は補償を行わない

            z_cur = obj.klift_edmd_residual([x_cur; u_nom(:)]);
            z_ref = obj.klift_edmd_residual([x_ref; u_nom(:)]);
            delta_u_raw = zeros(4, 1);
            % 現在状態と参照状態をEDMD用のlifted stateに変換する

            switch obj.residual.mode
                case 1
                    z_err = z_cur - z_ref;
                    delta_u_raw = -obj.residual.K * z_err;
                    % lifted state誤差に対するLQR補償

                case 2
                    z_nom_next = obj.residual.A_nom * z_cur + obj.residual.B_nom * u_nom;
                    z_target_bar = z_ref - z_nom_next;
                    rhs = z_target_bar - obj.residual.A_err * z_cur;
                    delta_u_raw = obj.solve_active_delta_u(rhs);
                    % 公称予測と目標との差から補償入力を反算する

                case 3
                    rhs = -obj.residual.A_err * z_cur;
                    delta_u_raw = obj.solve_active_delta_u(rhs);
                    % 残差成分を保守的に打ち消す

                otherwise
                    delta_u_raw = zeros(4, 1);
            end

            if any(~isfinite(delta_u_raw))
                delta_u_raw = zeros(4, 1);
            end
            % 数値異常がある場合は補償を0にする

            beta = obj.residual.beta;
            if isfield(obj.param, 'enable_show') && obj.param.enable_show
                fprintf('[EDMD diag] raw du: %.4f %.4f %.4f %.4f | A_err*z norm: %.4f\n', ...
                    delta_u_raw(1), delta_u_raw(2), delta_u_raw(3), delta_u_raw(4), ...
                    norm(obj.residual.A_err * z_cur));
            end

            delta_u = (1 - beta) * obj.residual.prev_delta_u + beta * delta_u_raw;
            obj.residual.prev_delta_u = delta_u;
            % 補償入力を一次フィルタで平滑化する

            delta_u = obj.residual.alpha * delta_u;
            delta_u = max(min(delta_u, obj.residual.du_max), -obj.residual.du_max);
            % 補償強度を調整し，上限値で制限する
        end

        function ok = within_compensation_envelope(obj, x_cur, x_ref)
            % 補償を適用してよい誤差範囲内か判定する
            e = zeros(12, 1);
            e(1:3) = x_cur(1:3) - x_ref(1:3);
            e(4:6) = obj.wrap_to_pi(x_cur(4:6) - x_ref(4:6));
            e(7:9) = x_cur(7:9) - x_ref(7:9);
            e(10:12) = x_cur(10:12) - x_ref(10:12);

            ok = norm(e(1:3)) <= obj.residual.max_pos_err && ...
                 all(abs(e(4:6)) <= obj.residual.max_ang_err) && ...
                 norm(e(7:9)) <= obj.residual.max_vel_err && ...
                 norm(e(10:12)) <= obj.residual.max_w_err;
        end

        function delta_u = solve_active_delta_u(obj, rhs)
            % 有効な入力チャンネルのみを使って補償入力を反算する
            delta_u = zeros(4, 1);
            active_channels = logical(obj.residual.active_channels(:));
            if numel(active_channels) ~= 4
                active_channels = [true; true; true; true];
            end

            idx = find(active_channels);
            if isempty(idx)
                return;
            end

            Be = obj.residual.B_err(:, idx);
            reg = obj.residual.pinv_damping * eye(size(Be, 2));
            delta_active = (Be' * Be + reg) \ (Be' * rhs);
            delta_u(idx) = delta_active;
        end

        function x12 = get_residual_state(obj)
            % 現在状態を12次元ベクトルとして取得する
            source = lower(obj.residual.state_source);
            switch source
                case 'plant'
                    st = obj.self.plant.state;
                otherwise
                    st = obj.self.estimator.result.state;
            end

            q = st.getq('euler');
            x12 = [double(st.p(:)); double(q(:)); double(st.v(:)); double(st.w(:))];
        end

        function x12 = get_reference_state(obj)
            % 参照状態を12次元ベクトルとして取得する
            sr = obj.self.reference.result.state;
            if (isstruct(sr) && isfield(sr, 'q')) || isprop(sr, 'q')
                qref = double(sr.q(:));
            else
                qref = double(sr.xd(4:6));
            end

            if (isstruct(sr) && isfield(sr, 'w')) || isprop(sr, 'w')
                wref = double(sr.w(:));
            else
                wref = zeros(3, 1);
            end

            x12 = [double(sr.p(:)); double(qref(:)); double(sr.v(:)); wref];
        end

        function z = klift_edmd_residual(obj, x16)
            % EDMD残差モデルで使用するlifted stateを構成する
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
            % 0除算を避けるため，cos項を安全化する

            R13 = c3 * s2 * c1 + s3 * s1;
            R23 = s3 * s2 * c1 - c3 * s1;
            R33 = c2 * c1;
            % 回転行列の第3列に相当する成分を計算する

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
            % 基本状態と非線形特徴量を結合する
        end

        function y = safe_nonzero(~, x, eps_val)
            % 小さすぎる値をeps_valで置き換え，0除算を防ぐ
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

        function ang = wrap_to_pi(~, ang)
            % 角度を -pi から pi の範囲に正規化する
            ang = mod(ang + pi, 2 * pi) - pi;
        end

        function du = generate_identification_excitation(obj, u_cmd, phase, time_info)
            % 同定または検証用の励振入力を生成する
            cfg = obj.get_explore_config();
            du = zeros(4, 1);

            if ~cfg.enable
                return;
            end
            if cfg.flight_only && ~obj.get_explore_active_flag(phase)
                return;
            end
            % 励振が無効，または飛行中でない場合は入力しない

            t = obj.get_time_value(time_info);
            tau = max(0, t - cfg.start_time);
            ramp = min(1.0, tau / max(cfg.ramp_time, 1e-6));
            % 励振を徐々に立ち上げる

            switch lower(char(string(cfg.mode)))
                case 'legacy_relative_random'
                    du = cfg.legacy_scale .* abs(u_cmd) .* (2 * rand(4, 1) - 1);
                    % 公称入力に比例したランダム励振

                otherwise
                    du(1) = sum(cfg.u1_amp(:) .* sin(2 * pi * cfg.u1_freq(:) * tau + cfg.u1_phase(:)));
                    du(2) = sum(cfg.u2_amp(:) .* sin(2 * pi * cfg.u2_freq(:) * tau + cfg.u2_phase(:)));
                    du(3) = sum(cfg.u3_amp(:) .* sin(2 * pi * cfg.u3_freq(:) * tau + cfg.u3_phase(:)));
                    du(4) = sum(cfg.u4_amp(:) .* sin(2 * pi * cfg.u4_freq(:) * tau + cfg.u4_phase(:)));
                    du = ramp * du;
                    % 各入力チャンネルにマルチサイン励振を加える
            end

            du = max(min(du, cfg.du_cap(:)), -cfg.du_cap(:));
            % 励振入力を安全範囲内に制限する
        end

        function cfg = get_explore_config(obj)
            % 励振入力の設定を取得する
            cfg = struct();
            cfg.enable = 0;
            cfg.flight_only = 1;
            cfg.mode = 'structured_multisine';
            cfg.start_time = 0.0;
            cfg.ramp_time = 1.0;
            cfg.legacy_scale = 0.2 * ones(4, 1);

            cfg.u1_amp = [0.35; 0.15];
            cfg.u1_freq = [0.35; 0.90];
            cfg.u1_phase = [0.20; 1.10];

            cfg.u2_amp = [0.010; 0.005];
            cfg.u2_freq = [0.55; 1.25];
            cfg.u2_phase = [0.40; 1.30];

            cfg.u3_amp = [0.010; 0.005];
            cfg.u3_freq = [0.70; 1.55];
            cfg.u3_phase = [0.90; 0.30];

            cfg.u4_amp = [0.0010; 0.0005];
            cfg.u4_freq = [0.45; 1.05];
            cfg.u4_phase = [0.50; 1.40];

            cfg.du_cap = [0.55; 0.018; 0.018; 0.003];
            % 推力，roll，pitch，yawの励振振幅・周波数・上限値を設定する

            if isfield(obj.param, 'explore')
                fns = fieldnames(obj.param.explore);
                for i = 1:numel(fns)
                    cfg.(fns{i}) = obj.param.explore.(fns{i});
                end
            end
            % 外部設定がある場合はデフォルト値を上書きする

            if isfield(obj.param, 'edmd_disturbance_enable')
                cfg.enable = logical(obj.param.edmd_disturbance_enable);
            end
        end

        function t = get_time_value(obj, time_info)
            % 励振信号生成に使用する時刻を取得する
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
            % 現在のフェーズが励振対象か判定する
            if isstring(phase) || ischar(phase)
                phase_str = lower(char(phase));
                active = ~isempty(phase_str) && phase_str(1) == 'f';
            else
                active = true;
            end
        end

        function show(obj)
            % 現在状態，参照状態，補償入力，最終入力を表示する
            est_print = obj.self.estimator.result.state;
            ref_print = obj.self.reference.result.state;
            fprintf("==================================================================\n")
            fprintf("==================================================================\n")
            fprintf("ps: %f %f %f \t vs: %f %f %f \t qs: %f %f %f \n",...
                est_print.p(1), est_print.p(2), est_print.p(3),...
                est_print.v(1), est_print.v(2), est_print.v(3),...
                est_print.q(1), est_print.q(2), est_print.q(3));
            fprintf("pr: %f %f %f \t vr: %f %f %f \t qr: %f %f %f \n", ...
                ref_print.p(1), ref_print.p(2), ref_print.p(3),...
                ref_print.v(1), ref_print.v(2), ref_print.v(3),...
                ref_print.xd(4), ref_print.xd(5), ref_print.xd(6));
            fprintf("comp_on: %d | u_nom: %f %f %f %f | du_pred: %f %f %f %f | du_exp: %f %f %f %f | u: %f %f %f %f\n", ...
                obj.result.residual_apply_compensation, ...
                obj.result.u_nom(1), obj.result.u_nom(2), obj.result.u_nom(3), obj.result.u_nom(4), ...
                obj.result.delta_u_edmd_pred(1), obj.result.delta_u_edmd_pred(2), obj.result.delta_u_edmd_pred(3), obj.result.delta_u_edmd_pred(4), ...
                obj.result.delta_u_explore(1), obj.result.delta_u_explore(2), obj.result.delta_u_explore(3), obj.result.delta_u_explore(4), ...
                obj.result.input(1), obj.result.input(2), obj.result.input(3), obj.result.input(4));
        end
    end
end
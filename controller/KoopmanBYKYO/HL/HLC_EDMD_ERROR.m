classdef HLC_EDMD_ERROR < handle
    % EDMDで学習した閉ループ誤差補償を加えるHLC制御器

    properties
        self
        result
        param
        comp
        H = 12
        parameter_name = ["mass","Lx","Ly","lx","ly","jx","jy","jz","gravity","km1","km2","km3","km4","k1","k2","k3","k4"];
    end

    methods
        function obj = HLC_EDMD_ERROR(self, param)
            % コントローラの初期化を行う
            obj.self = self;
            obj.param = param;
            obj.param.P = self.parameter.get(obj.parameter_name);
            obj.result.input = zeros(self.estimator.model.dim(2), 1);
            obj.comp = obj.initialize_compensator(param);
        end

        function result = do(obj, varargin)
            % HLC入力を計算し，EDMD誤差補償を加えて最終入力を出力する
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
            % 目標状態ベクトルが不足している場合は0で補う

            Rb0 = RodriguesQuaternion(Eul2Quat([0; 0; xd(4)]));
            x = [R2q(Rb0' * model.state.getq("rotmat")); Rb0' * model.state.p; Rb0' * model.state.v; model.state.w];
            % 目標yaw角を基準とした座標系に現在状態を変換する

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
            % サンプリング時間に応じて仮想入力 vf, vs を計算する

            tmp = Uf(x, xd', vf, P) + Us(x, xd', vf, vs', P);
            % 高階線形化に基づく基本制御入力を計算する

            u_nom = [max(0, min(10, tmp(1))); ...
                     max(-1, min(1, tmp(2))); ...
                     max(-1, min(1, tmp(3))); ...
                     max(-1, min(1, tmp(4)))];
            % 安全のため，HLC入力を上下限で制限する

            [delta_u, comp_diag] = obj.compute_error_compensation();
            % EDMDで学習した誤差モデルに基づき補償入力を計算する

            u_total = u_nom + delta_u;
            u_total = [max(0, min(10, u_total(1))); ...
                       max(-1, min(1, u_total(2))); ...
                       max(-1, min(1, u_total(3))); ...
                       max(-1, min(1, u_total(4)))];
            % HLC入力に補償入力を加え，最終入力も上下限で制限する

            obj.result.u_nom = u_nom;
            obj.result.deltau = delta_u;
            obj.result.delta_u_edmd = delta_u;
            obj.result.delta_u_thrust = [delta_u(1); 0; 0; 0];
            obj.result.delta_u_torque = [0; delta_u(2:4)];
            obj.result.u_raw = u_total;
            obj.result.input = u_total;
            obj.result.hlc = u_total;
            obj.result.pre_u = u_total;
            obj.result.residual_loaded = obj.comp.loaded;
            obj.result.comp_e = comp_diag.e;
            obj.result.comp_zv = comp_diag.zv;
            obj.result.comp_za = comp_diag.za;
            obj.result.comp_du1_raw = comp_diag.du1_raw;
            obj.result.comp_dutau_raw = comp_diag.dutau_raw;
            % 制御入力と補償に関する診断情報を保存する

            fprintf('controller: HLC_EDMD_ERROR,  phase: %s \n', phase);
            result = obj.result;
            obj.show();
        end

        function comp = initialize_compensator(obj, param)
            % EDMD補償器の初期設定と学習済みモデルの読み込みを行う
            comp = struct();
            comp.loaded = false;
            comp.model_file = '';
            comp.state_source = 'estimator';
            comp.alpha = 1.0;
            comp.du_max = [0.6; 0.12; 0.12; 0.12];
            comp.beta_z = 0.25;
            comp.beta_att = 0.20;
            comp.Az = [];
            comp.Bz = [];
            comp.Aa = [];
            comp.Ba = [];
            comp.Kz = [];
            comp.Ka = [];
            comp.du1_prev = 0;
            comp.dutau_prev = zeros(3, 1);

            if ~isfield(param, 'comp')
                return;
            end
            % 補償器設定がない場合は補償を無効のままにする

            fns = fieldnames(param.comp);
            for i = 1:numel(fns)
                comp.(fns{i}) = param.comp.(fns{i});
            end
            % param.comp の設定でデフォルト値を上書きする

            if ~isfield(param.comp, 'model_file') || isempty(param.comp.model_file)
                return;
            end
            % 学習済みモデルファイルが指定されていない場合は読み込まない

            try
                mdl = load(param.comp.model_file, 'hl_edmd_error_model');
                res = mdl.results_hl_edmd_error;
                comp.Az = res.Az;
                comp.Bz = res.Bz;
                comp.Aa = res.Aa;
                comp.Ba = res.Ba;
                comp.Kz = res.Kz;
                comp.Ka = res.Ka;
                comp.loaded = true;
            catch ME
                warning('HLC error compensator load failed: %s', ME.message);
            end
            % 学習済みの垂直方向モデルと姿勢方向モデルを読み込む
        end

        function [delta_u, diag_out] = compute_error_compensation(obj)
            % 現在状態と参照状態の誤差からEDMD補償入力を計算する
            delta_u = zeros(4, 1);
            diag_out = struct();
            diag_out.e = zeros(12, 1);
            diag_out.zv = zeros(3, 1);
            diag_out.za = zeros(12, 1);
            diag_out.du1_raw = 0;
            diag_out.dutau_raw = zeros(3, 1);

            if ~obj.comp.loaded
                return;
            end
            % 学習済みモデルが読み込まれていない場合は補償を行わない

            x_cur = obj.get_state12();
            x_ref = obj.get_reference12();
            e = obj.compute_error_state(x_cur, x_ref);
            % 12次元の状態誤差を計算する

            zv = obj.lift_vertical_error(e);
            za = obj.lift_attitude_error(e);
            % 垂直方向と姿勢方向の誤差をリフトアップする

            du1_raw = -obj.comp.Kz * zv;
            dutau_raw = -obj.comp.Ka * za;
            % 学習済みゲインにより推力補償と姿勢補償を計算する

            du1 = (1 - obj.comp.beta_z) * obj.comp.du1_prev + obj.comp.beta_z * du1_raw;
            dutau = (1 - obj.comp.beta_att) * obj.comp.dutau_prev + obj.comp.beta_att * dutau_raw;
            % 補償入力を一次フィルタで平滑化する

            obj.comp.du1_prev = du1;
            obj.comp.dutau_prev = dutau;

            delta_u = [du1; dutau];
            delta_u = obj.comp.alpha * delta_u;
            delta_u = max(min(delta_u, obj.comp.du_max), -obj.comp.du_max);
            % 補償強度を調整し，安全のため最大値を制限する

            diag_out.e = e;
            diag_out.zv = zv;
            diag_out.za = za;
            diag_out.du1_raw = du1_raw;
            diag_out.dutau_raw = dutau_raw;
            % 補償計算の確認用データを出力する
        end

        function x12 = get_state12(obj)
            % 現在状態を [位置; 姿勢角; 速度; 角速度] の12次元ベクトルとして取得する
            source = lower(obj.comp.state_source);
            switch source
                case 'plant'
                    st = obj.self.plant.state;
                otherwise
                    st = obj.self.estimator.result.state;
            end
            % state_source により，真値または推定値を選択する

            q = st.getq('euler');
            x12 = [double(st.p(:)); double(q(:)); double(st.v(:)); double(st.w(:))];
        end

        function x12 = get_reference12(obj)
            % 参照状態を [位置; 姿勢角; 速度; 角速度] の12次元ベクトルとして取得する
            st = obj.self.reference.result.state;

            if isprop(st, 'q')
                qref = double(st.q(:));
            elseif isprop(st, 'xd')
                xd = double(st.xd(:));
                if numel(xd) >= 6
                    qref = xd(4:6);
                else
                    qref = zeros(3, 1);
                end
            else
                qref = zeros(3, 1);
            end
            % 参照姿勢角を取得する

            if isprop(st, 'w')
                wref = double(st.w(:));
            elseif isprop(st, 'xd')
                xd = double(st.xd(:));
                if numel(xd) >= 12
                    wref = xd(10:12);
                else
                    wref = zeros(3, 1);
                end
            else
                wref = zeros(3, 1);
            end
            % 参照角速度を取得する

            x12 = [double(st.p(:)); qref; double(st.v(:)); wref];
        end

        function e = compute_error_state(~, x, xr)
            % 現在状態と参照状態から12次元誤差を計算する
            e = zeros(12, 1);
            e(1:3) = x(1:3) - xr(1:3);
            e(4:6) = mod(x(4:6) - xr(4:6) + pi, 2 * pi) - pi;
            e(7:9) = x(7:9) - xr(7:9);
            e(10:12) = x(10:12) - xr(10:12);
            % 姿勢角誤差は -pi から pi の範囲に正規化する
        end

        function z = lift_vertical_error(~, e)
            % 高度誤差と鉛直速度誤差から垂直方向のリフトアップ状態を作る
            ez = e(3);
            evz = e(9);
            z = [ez; evz; ez * evz];
        end

        function z = lift_attitude_error(~, e)
            % 姿勢角誤差と角速度誤差から姿勢方向のリフトアップ状態を作る
            eq = e(4:6);
            ew = e(10:12);
            z = [eq; ew; sin(eq); eq .* ew];
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
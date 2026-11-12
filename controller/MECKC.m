
classdef MECKC < handle
    % Hierarchical linearization based controller for a quadcopter
    properties

        self
        result
        param
        parameter_name = ["mass","Lx","Ly","lx","ly","jx","jy","jz","gravity","km1","km2","km3","km4","k1","k2","k3","k4"];
        agent
        delta_u
        pre_input = [0;0;0;0];
        x_pre = [0;0;0;0;0;0;0;0;0;0;0;0];

    end
    properties (Constant)
        GAIN_FILE = '4reiki_last_LYKL_err_20261106_163610_gain_controllable_dlqr_QQ1000_R1_20261106_164032_189.mat';
    end

    properties
        K_full
    end

    methods
        function obj = MECKC(self,param)
            % ゲインを読み込む
            S = load(MECKC.GAIN_FILE, 'K_full');
            obj.K_full = S.K_full;
            obj.self = self;
            obj.param = param;
            obj.param.P = self.parameter.get(obj.parameter_name);
            obj.result.input = zeros(4,1);
            % obj.result.input = zeros(self.estimator.model.dim(2),1);
            % %これが元々のコード，modelのところでエラーが出るから上記に変更

        end

        function result = do(obj,varargin)
            % vararginの中身：do_loopのところに１個(?)あって，do(app.time,fCha,app.logger,app.env,app.agent,i)
            model = obj.self.estimator.result;
            t = varargin{1,1}.t;
            x = [model.state.p(1);
                model.state.p(2);
                model.state.p(3);
                model.state.q(1);
                model.state.q(2);
                model.state.q(3);
                model.state.v(1);
                model.state.v(2);
                model.state.v(3);
                model.state.w(1);
                model.state.w(2);
                model.state.w(3);];


            % LOGGERに必要なデータが存在するか確認
            logger_ready = ...
                isfield(varargin{3}.Data.agent, "controller") && ...
                isfield(varargin{3}.Data.agent, "estimator") && ...
                numel(varargin{3}.Data.agent.estimator.result) >= 2 && ...
                ~isempty(varargin{3}.Data.agent.controller.result);

            if logger_ready
                % 直前の入力（励起入力込み）
                obj.pre_input = varargin{3}.Data.agent.controller.result{end}.input;

                % 直前の状態
                obj.x_pre = varargin{3}.Data.agent.estimator.result{:,end}.state.get;
            end

            dt = varargin{1}.dt;    %25msとか

            dx = roll_pitch_yaw_thrust_torque_physical_parameter_model(obj.x_pre, obj.pre_input, obj.param.P);

            x_n_now = obj.x_pre + dx*dt;%x_nominal[k+1]
            % delta_x = x-x_n_now;
            z_p = quaternions_all(x); %観測量z※プラントの状態を入れてる
            z_n = quaternions_all(x_n_now);%ノミナルの状態

            %---可制御部分をデカップリング---%

            % K_full=[zeros(4,24)];
            % load('errore_02_spline_4Reiki__KL_err_gain_controllable_dlqr_Q1_R1000_20261028_181411_073.mat','K_full');
            % load('errore_02_spline_4Reiki_2__LYKL_err_gain_controllable_dlqr_Q1_R1000_20261028_181400_978','K_full');
            % load('errore_02_spline_4Reiki_LYKL_err_20261105_160010_gain_controllable_dlqr_QP100_R1_20261105_160255_713', 'K_full'); %errore_02_circle_LKMEC_result
            % load('4reiki_motodoori_use_reduced_LYKL_err_20261106_151748_gain_controllable_dlqr_Q1000_R1_20261106_152843_129.mat','K_full');%全然ダメ
            % errore_02_spline_4Reiki_LYKL_err_20261105_160010_gain_controllable_dlqr_QP100_R1_20261105_162120_295
            % errore_02_spline_4Reiki__KL_err_gain_controllable_dlqr_QP100_R1_20261105_162148_564

            %load(

            % % K = data.K;
            e = z_n-z_p;
            obj.result.e = e;
            %%%%%-----lqr法終わり-----%%%%%
            %拡張状態取得

            % ※一気にz_pなどと取得するとplot時に警告が出る(50まで)
            obj.result.z_p_forward = z_p(1:13);%プラント拡張状態の前半
            obj.result.z_n_forward = z_n(1:13);
            obj.result.z_p_back = z_p(14:26);
            obj.result.z_n_back = z_n(14:26);%ノミナル拡張状態の後半

            function T_ex = excitation_sweep_cos(t)
                % 変更：4入力それぞれの振幅
                A = [2;
                    0.1;
                    0.1;
                    0.1];
                % 変更：4入力で異なる周波数スイープ
                f_start = [0.1;
                    0.2;
                    0.3;
                    0.4];
                f_end = [2.0;
                    1.8;
                    1.6;
                    1.4];

                t_start = 7.0;            % 励起開始時刻
                t_end = 47.0;             % 励起終了時刻
                T_sweep = t_end - t_start; % 励起時間40秒
                T_ramp = 2.0;

                if t < t_start || t >= t_end
                    T_ex = zeros(4,1);
                    return
                end

                tau = t - t_start;

                % 線形周波数スイープ
                % 変更：kは4×1ベクトル
                k = (f_end - f_start) / T_sweep;

                % 位相
                % 変更：phaseは4×1ベクトル
                phase = 2*pi*(f_start*tau + 0.5*k*tau^2);

                % ===== smooth envelope =====
                if tau < T_ramp
                    % 0 -> 1
                    envelope = 0.5*(1 - cos(pi*tau/T_ramp));

                elseif tau > T_sweep - T_ramp
                    % 1 -> 0
                    tau_end = T_sweep - tau;
                    envelope = 0.5*(1 - cos(pi*tau_end/T_ramp));

                else
                    envelope = 1;
                end

                % 変更：4×1の励起入力を生成
                T_ex = A .* envelope .* cos(phase);
            end


            % ===== 補償入力・励起入力の切り替え =====

            % obj.result.delta_u = zeros(4,1);  % 補償・励起ともオフ
            % % 各入力を -3 ～ 3 に制限
            % delta_u_raw = -obj.K_full*e;
            % obj.result.delta_u = max(-3, min(3, delta_u_raw));

            % 補償入力オン
            % obj.result.delta_u = -obj.K_full*e;

            obj.result.delta_u = excitation_sweep_cos(t);  % 4入力励起オン

            % obj.result.delta_u = -K_full*e + excitation_sweep_cos(t);
            % 補償入力＋4入力励起オン


            % obj.result.input = varargin{5}.controller.result.input + obj.result.delta_u;%un+Δu+励起入力
            % result = obj.result;
            %% aの間はMEC補償を無効化
            % if ismember(string(obj.self.cha), "a")
            %     obj.result.delta_u = zeros(4,1);
            % end

            %% 最終入力
            obj.result.input = varargin{5}.controller.result.input + obj.result.delta_u;

            result = obj.result;
        end
    end
end

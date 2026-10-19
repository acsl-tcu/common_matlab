classdef MECKC < handle
    % HLCの入力にMEC補償と、学習用の4入力励振を加える。
    properties
        self
        result
        param
        parameter_name = ["mass","Lx","Ly","lx","ly","jx","jy","jz","gravity","km1","km2","km3","km4","k1","k2","k3","k4"];
        pre_input = zeros(4,1);
        x_pre = zeros(12,1);
        previous_time = [];
        gain = zeros(4,26);
        collect_dataset = false;
        excitation_enabled = true;
        use_compensation = true;
        excitation_profile = [];
        flight_start = [];
    end

    methods
        function obj = MECKC(self,param)
            obj.self = self;
            obj.param = param;
            obj.param.P = self.parameter.get(obj.parameter_name);
            obj.result.input = zeros(4,1);
            use_compensation = true;
            if isfield(param,'dataset') && param.dataset.enabled
                obj.collect_dataset = true;
                if isfield(param.dataset,'excitation_enabled')
                    obj.excitation_enabled = param.dataset.excitation_enabled;
                end
                use_compensation = param.dataset.use_compensation;
                obj.excitation_profile = make_mec_excitation(param.trajectory_seed,param.dataset);
            end
            % 明示指定があれば、通常モード・学習モードのどちらでも優先する。
            if isfield(param,'use_compensation')
                use_compensation = param.use_compensation;
            end
            obj.use_compensation = use_compensation;
            % ゲインは補償を使う場合だけ、初期化時に読み込む。
            if use_compensation
                % 旧ゲインは比較用に残す。正則化・検証付きの設計結果を既定にする。
                % gain_file = 'ai_data_spline_KL_err_gain_controllable_dlqr_20261016_174715_924.mat';
                repo_root = fileparts(fileparts(mfilename('fullpath')));
                gain_file = fullfile(repo_root,'Data','MEC_design','mec_gain_regularized.mat');
                if isfield(param,'gain_file')
                    gain_file = param.gain_file;
                end
                % gain_data = load(gain_file,'K_full');
                if isfile(gain_file)
                    gain_data = load(gain_file);
                else
                    % MATファイルがない環境では、検証後に保存した係数ソースを使う。
                    source_file = fullfile(repo_root,'controller','mec_gain_regularized.m');
                    assert(isfile(source_file),'MECKC:MissingGain', ...
                        'ゲインがありません。先にtrain_mec_gainを実行してください：%s',gain_file);
                    addpath(fileparts(source_file));
                    gain_data = mec_gain_regularized();
                end
                if isfield(gain_data,'error_definition')
                    assert(strcmp(gain_data.error_definition,'nominal_input_one_step_v1'), ...
                        'MECKC:GainDefinition','ゲイン設計時と現在の誤差定義が異なります。');
                    assert(isfield(param,'dt') && abs(gain_data.dt-param.dt)<1e-10, ...
                        'MECKC:GainSampleTime','ゲイン設計時と現在の周期が異なります。');
                else
                    warning('MECKC:LegacyGain','旧ゲインには誤差定義・周期の検証情報がありません。');
                end
                obj.gain = gain_data.K_full;
                assert(isequal(size(obj.gain),[4,26]) && all(isfinite(obj.gain(:))), ...
                    'MECKC:InvalidGain','K_fullは有限な4行26列の行列にしてください。');
            end
        end

        function result = do(obj,varargin)
            % 呼び出し順：時刻、動作段階、ログ、環境、機体一覧、機体番号。
            x = obj.self.estimator.result.state.get();
            time = varargin{1};
            dt = time.dt;
            t = time.t;
            % 前回の推定状態と同時刻のノミナル入力から、現在状態を予測する。
            % 空回しでは時刻が進まないので、その誤差を学習に使わない。
            obj.result.e_valid = ~isempty(obj.previous_time) && t>obj.previous_time;
            if obj.result.e_valid
                assert(abs(t-obj.previous_time-dt)<1e-8, ...
                    'MECKC:TimeGap','予測と観測の時間間隔がサンプリング周期と一致しません。');
                dx = roll_pitch_yaw_thrust_torque_physical_parameter_model( ...
                    obj.x_pre,obj.pre_input,obj.param.P);
                x_nominal = obj.x_pre+dx*dt;
            else
                x_nominal = x;
            end
            z_plant = quaternions_all(x);
            z_nominal = quaternions_all(x_nominal);
            obj.result.e = z_nominal-z_plant;
            obj.result.error_definition = 'nominal_input_one_step_v1';
            obj.result.x_nominal = x_nominal;
            obj.result.x_estimated = x;

            % 描画時の成分数制限を避けるため、26観測量を前後半に分ける。
            obj.result.z_p_forward = z_plant(1:13);
            obj.result.z_n_forward = z_nominal(1:13);
            obj.result.z_p_back = z_plant(14:26);
            obj.result.z_n_back = z_nominal(14:26);

            obj.result.delta_u_mec = -obj.gain*obj.result.e;
            u_nominal = obj.self.controller.result.input;
            obj.result.excitation = zeros(4,1);
            if obj.collect_dataset && obj.excitation_enabled && varargin{2}=='f'
                if isempty(obj.flight_start)
                    obj.flight_start = t;
                end
                obj.result.excitation = mec_input_excitation(t-obj.flight_start,obj.excitation_profile);
            end
            % 同定入力は補償量と励振を含む、実機側とノミナル側の入力差。
            obj.result.input_nominal = u_nominal;
            obj.result.input = u_nominal+obj.result.delta_u_mec+obj.result.excitation;
            obj.result.input_plant = obj.result.input;
            obj.result.delta_u = obj.result.input_plant-obj.result.input_nominal;

            % 次回の予測に必要な状態と入力を、同じステップで保存する。
            obj.x_pre = x;
            obj.pre_input = u_nominal;
            obj.previous_time = t;
            result = obj.result;
        end
    end
end

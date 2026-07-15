classdef PythonNNMEC
    %PythonNNMEC
    %   クアッドコプター用モデル誤差補償器(MEC)のプログラム
    %   ニューラルネットワーク(NN)で補償器を設計
    %   推論時にPythonスクリプトを呼び出してdoするクラス
    %   Required: common_matlab\Pythonenv ←Pythonの仮想実行環境
    %           controller\NNMEC\Readme4PythonNNMEC.txtに環境構築方法を記載
    %
    %   [Inputs]
    %    self: ドローンのagent
    %    NN_model_filename = "NNMEC.onnx": インポートするonnxファイルの名前
    %    Requied: "NN12": 次元数(12,21,24)，"Euler":状態更新手法("Euler", "RK4")
    
    %   2026/07 作成者:M1小関      学番:2681030
    
    properties
        self
        result
        physical_param
        parameter_name = ["mass", "Lx", "Ly", "lx", "ly", "jx", "jy", "jz", "gravity", "km1", "km2", "km3", "km4", "k1", "k2", "k3", "k4"];
        agent
        NN_model_filename   % 読み込みたいptファイルの名前
        NNMEC_model         % コード内でのモデル名
        dx_func             % 状態方程式の関数ハンドル
        state_renew_func    % 学習時の状態更新手法に合わせるための関数ハンドル
        gen_data_func       % 入力の次元数に合わせたデータ生成関数ハンドル
        x_pre               % 前時刻の状態
        pre_input           % 前時刻の制御入力
        thrust_lim = 5      % 補償推力入力ΔT の制限値
        tau_lim = 0.5       % 補償トルク入力Δτ の制限絶対値
        LPF                 % Low Pass Filterクラス
    end
    
    methods
        function obj = PythonNNMEC(self, NN_model_filename, opts)
            arguments
                self % agent
                NN_model_filename
                opts.view_model_info = true;
            end
            % インスタンス
            obj.self = self;
            obj.physical_param = self.parameter.get(obj.parameter_name);
            obj.dx_func = @roll_pitch_yaw_thrust_torque_physical_parameter_model;

            %-%-%-% NN model import & define %-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%
            obj.NN_model_filename = string(NN_model_filename);
            if ~exist("controller\NNMEC\NN_Model\pt", "dir")
                mkdir("controller\NNMEC\NN_Model\pt")
            elseif isempty(dir("controller\NNMEC\NN_Model\pt\*.pt"))
                error("PythonNNMEC: Do not exist <pt> file in controller\NNMEC\NN_Model\pt. ")
            end
            ptPath = fullfile("controller", "NNMEC", "NN_Model", "pt", obj.NN_model_filename);
            if ~endsWith(obj.NN_model_filename, ".pt")
                ptPath = ptPath + ".pt";
            end

            common_matlab_fullpath = erase(fileparts(mfilename('fullpath')),'\controller\NNMEC');
            Pythonenv_path = fullfile(common_matlab_fullpath, 'Pythonenv', 'Scripts', 'python.exe');
            if ~exist("Pythonenv\", "dir")
                msg = sprintf("PythonNNMEC: <Pythonenv>が作成されていません。\n" + ...
                    "controller>NNMEC>Readme4PythonNNMEC.pptx を参照してセットアップしてください。\n");
                error(msg)
            end
            %TODO: 既にPyenvを使う設定なら下は実行しないようにしたい。
            msg = sprintf("Python仮想環境をロード中...\n" + ...
                          "Loading Python virtual environment...\n");
            fin_msg = sprintf("=== ロード完了  Complete loading ===\n");
            if isempty(pyenv)
                disp(msg);
                pyenv("Version",Pythonenv_path); % MATLABにPython仮想実行環境を登録
                disp(fin_msg);
            elseif ~contains(pyenv().Executable,"common_matlab")
                disp(msg);
                pyenv("Version",Pythonenv_path); % MATLABにPython仮想実行環境を登録
                disp(fin_msg);
            end
            Python_script_path = fullfile(common_matlab_fullpath, "controller", "NNMEC");
            if count(py.sys.path, Python_script_path) == 0
                insert(py.sys.path, int32(0), Python_script_path);
            end

            % NNに続く数字を抽出 (例: "NN24" -> "24")
            tokens = regexp(obj.NN_model_filename, 'NN(\d+)', 'tokens');
            if ~isempty(tokens)
                model_num = tokens{1}{1}; % 文字列としての数字を取得
                switch model_num
                    case '21'
                        dim = 21;
                        obj.gen_data_func = @(x_p,x_n) [x_p(1:3)-x_n(1:3); x_p(4:end); x_n(4:end)];
                    case '12'
                        dim = 12;
                        obj.gen_data_func = @(x_p,x_n) x_p - x_n;
                    case '24'
                        dim = 24;
                        obj.gen_data_func = @(x_p,x_n) [x_p; x_n];
                    otherwise
                        % 例外処理
                        error('未知のモデル次元です: %s', model_num);
                end
            else % "NN"の記述がない場合のデフォルト
                dim = 12;
                obj.gen_data_func = @(x_p,x_n) x_p - x_n;
            end

            py.importlib.import_module('NNMEC_inference');
            obj.NNMEC_model = py.NNMEC_inference.InferenceWrapper(ptPath);
            if opts.view_model_info, disp(obj.NNMEC_model.architecture_info); end

            if contains(NN_model_filename, 'RK4') % 状態更新手法を動的に変更
                obj.state_renew_func = @(x_pre, pre_input, dt) obj.RK4(x_pre, pre_input, dt);
            else
                obj.state_renew_func = @(x_pre, pre_input, dt) obj.Euler(x_pre, pre_input, dt); % default: Euler
            end
            %-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%

            fc = 1.5; % LPFのカットオフ周波数
            Ts = 0.025; % サンプリング周波数
            % obj.LPF = LOW_PASS_FILTER(fc, Ts);
            
            % データ関連の初期化
            obj.result.nominal_p = zeros(3,1);
            obj.result.nominal_q = zeros(3,1);
            obj.result.nominal_v = zeros(3,1);
            obj.result.nominal_w = zeros(3,1);
            est_name = self.estimator.name;
            input_dim = self.estimator.(est_name).model.dim(2);
            obj.result.nominal_input = zeros(input_dim,1);
            obj.result.delta_input = zeros(input_dim,1);
            obj.result.input = zeros(input_dim,1);
            obj.x_pre = self.estimator.result.state.get;
            obj.pre_input = zeros(input_dim,1);
            fprintf('Model file name: %s\n', obj.NN_model_filename);
        end % function PythonNNMEC
        
        function result = do(obj, varargin)
            %-%-%-% ノミナル状態更新 ※状態更新の手法は学習時のものと合わせる %-%-%-%
            if isfield(varargin{3}.Data.agent, "controller") && isfield(varargin{3}.Data.agent, "estimator")... % ループの最初はLoggingされていなくて，参照できないのを回避
            && length(varargin{3}.Data.agent.estimator.result)>=2
                obj.pre_input = varargin{3}.Data.agent.controller.result{end}.input; % LOGGERから前時刻の入力を取得
                obj.x_pre = varargin{3}.Data.agent.estimator.result{end}.state.get; % LOGGERから前時刻の状態を取得
            end
            dt = varargin{1}.dt;
            x_nominal = obj.state_renew_func(obj.x_pre, obj.pre_input, dt);
            %-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%
            obj.result.nominal_p = x_nominal(1:3);
            obj.result.nominal_q = x_nominal(4:6);
            obj.result.nominal_v = x_nominal(7:9);
            obj.result.nominal_w = x_nominal(10:12);

            % プラント値取得
            x_plant = obj.self.estimator.result.state.get; % 現時刻の推定値
            % -> size = 12*1, contents = [p; q; v; w];

            % NNへの入力データ
            data = obj.gen_data_func(x_plant, x_nominal);
            data_py = py.numpy.array(data);
            py_result = obj.NNMEC_model.predict(data_py); % Pythonスクリプトでの推論
            obj.result.delta_input = (-1*double(py_result))';
            if abs(obj.result.delta_input(1))>obj.thrust_lim, obj.result.delta_input(1) = 0; end % NN関係　閾値での入力制限
            if abs(obj.result.delta_input(2))>obj.tau_lim, obj.result.delta_input(2) = 0; end
            if abs(obj.result.delta_input(3))>obj.tau_lim, obj.result.delta_input(3) = 0; end
            if abs(obj.result.delta_input(4))>obj.tau_lim, obj.result.delta_input(4) = 0; end

            % obj.result.delta_input = [0;0;0;0]; % 補償入力を無くしてNN-MECを入れない
            % obj.result.delta_input = obj.LPF.update(obj.result.delta_input); % 補償入力delta_uにLPFを掛ける

            obj.result.nominal_input = varargin{5}.controller.nominal.result.input; % ノミナル入力を保存
            obj.result.input = obj.result.nominal_input + obj.result.delta_input;
            obj.result.data = data;
            result = obj.result;
        end % function do



        function x_plus = Euler(obj, x_pre, pre_input, dt)
            % 前進オイラー法による時間発展
            dx = obj.dx_func(x_pre, pre_input, obj.physical_param);
            x_plus = x_pre + dx*dt;
        end

        function x_plus = RK4(obj, x_pre, pre_input, dt)
            % 4次ルンゲ・クッタ法による時間発展
            dx = @(x) obj.dx_func(x, pre_input, obj.physical_param);

            k1 = dx(x_pre);
            k2 = dx(x_pre + dt*k1/2);
            k3 = dx(x_pre + dt*k2/2);
            k4 = dx(x_pre + dt*k3);

            x_plus = x_pre + dt/6*(k1 + 2*k2 + 2*k3 + k4);
        end
    end
end


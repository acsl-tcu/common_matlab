classdef DNNMEC < handle
    %DNNMEC
    %   クアッドコプター用モデル誤差補償器(MEC)のプログラム
    %   ディープニューラルネットワーク(DNN)で補償器を設計
    %   [Inputs]
    %    self: ドローンのagent
    %    NN_model_filename = "NNMEC.onnx": インポートするonnxファイルの名前
    %    "NN12": 次元数(12,21,24)，"Euler":状態更新手法("Euler", "RK4")がファイル名に必要
    
    %   2025/07 作成者:B4小関      学番:2212044
    %   最終更新：2026/04/21
    
    properties
        self
        result
        physical_param
        parameter_name = ["mass", "Lx", "Ly", "lx", "ly", "jx", "jy", "jz", "gravity", "km1", "km2", "km3", "km4", "k1", "k2", "k3", "k4"];
        agent
        NN_model_filename  % 読み込みたいONNXモデルのファイル名
        DNNMEC_model        % コード内でのモデル名
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
        function obj = DNNMEC(self, NN_model_filename)
            % インスタンス
            obj.self = self;
            obj.physical_param = self.parameter.get(obj.parameter_name);
            obj.dx_func = @roll_pitch_yaw_thrust_torque_physical_parameter_model;

            %-%-%-% DNN model import & define %-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%
            obj.NN_model_filename = NN_model_filename;
            if ~exist("controller/DNN_MODEL", "dir")
                mkdir("controller\DNN_MODEL")
            elseif isempty(dir("controller/DNN_MODEL/*.onnx"))
                error("ACSL: Do not exist <onnx> file in controller/DNN_MODEL. ")
            end
            DNN_model = importNetworkFromONNX("\DNN_MODEL\"+obj.NN_model_filename,... % ONNXファイルインポート
                                                "InputDataFormats", "BC", ... % 入力層定義
                                                "OutputDataFormats", "BC");   % 出力層定義 "BC" -> [バッチサイズ, 特徴量]の意味
            
            % NNに続く数字を抽出 (例: "NN24" -> "24")
            tokens = regexp(NN_model_filename, 'NN(\d+)', 'tokens');
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
            else % NNの記述がない場合のデフォルト
                dim = 24;
                obj.gen_data_func = @(x_p,x_n) [x_p; x_n];
            end
            dummyInput = dlarray(randn(dim,1,'single'), 'CB'); % 初期化のためのdummy入力
            obj.DNNMEC_model = initialize(DNN_model, dummyInput); % モデルの初期化

            if contains(NN_model_filename, 'Euler') % 状態更新手法を動的に変更
                obj.state_renew_func = @(x_pre, pre_input, dt) obj.Euler(x_pre, pre_input, dt);
            elseif contains(NN_model_filename, 'RK4')
                obj.state_renew_func = @(x_pre, pre_input, dt) obj.RK4(x_pre, pre_input, dt);
            end
            %-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%

            fc = 1.5; % LPFのカットオフ周波数
            Ts = 0.025; % サンプリング周波数
            obj.LPF = LowPassFilter(fc, Ts);
            
            % データ関連の初期化
            obj.result.nominal_p = zeros(3,1);
            obj.result.nominal_q = zeros(3,1);
            obj.result.nominal_v = zeros(3,1);
            obj.result.nominal_w = zeros(3,1);
            obj.result.nominal_input = zeros(self.estimator.model.dim(2),1);
            obj.result.delta_input = zeros(self.estimator.model.dim(2),1);
            obj.result.input = zeros(self.estimator.model.dim(2),1);
            obj.x_pre = self.estimator.result.state.get;
            obj.pre_input = zeros(self.estimator.model.dim(2),1);
            fprintf('Model file name: %s\n', obj.NN_model_filename);
            msg = "表示内容\n" + ...
                "ref:px, py, pz,  NaN   est:px, py, pz,  NaN   Delta_input: T, tau_{roll}, tau_{pitch}, tau_{yaw}\n\n";
            fprintf(msg);
        end
        
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
            obj.result.delta_input = -1*double(predict(obj.DNNMEC_model, data'))'; % predict関数での推論
            if abs(obj.result.delta_input(1))>obj.thrust_lim, obj.result.delta_input(1) = 0; end % NN関係　閾値での入力制限
            if abs(obj.result.delta_input(2))>obj.tau_lim, obj.result.delta_input(2) = 0; end
            if abs(obj.result.delta_input(3))>obj.tau_lim, obj.result.delta_input(3) = 0; end
            if abs(obj.result.delta_input(4))>obj.tau_lim, obj.result.delta_input(4) = 0; end

            % obj.result.delta_input = [0;0;0;0]; % 補償入力を無くしてNN-MECを入れない
            obj.result.delta_input = obj.LPF.update(obj.result.delta_input); % 補償入力delta_uにLPFを掛ける

            obj.result.nominal_input = varargin{5}.controller.nominal.result.input; % ノミナル入力を保存
            obj.result.input = obj.result.nominal_input + obj.result.delta_input;
            result = obj.result;
            disp([obj.self.reference.result.state.p', NaN,  obj.self.estimator.result.state.p', NaN, obj.result.delta_input']);
        end



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



        function Delta_input = infer_model(obj, plant_state, nominal_state)
            % モデルの推論を行う
            % [Inputs]
            %   plant_state: [p; q; v; w] (12,:)
            %   nominal_state: [p; q; v; w] (12,:)
            %
            % [Outputs]
            %   Delta_input: 推論結果 (4,:)
            %   実際にdoループで使うときには plant_input = nominal_input + Delta_input となる

            size_plant = size(plant_state);
            size_nonimal = size(nominal_state);
            if size_plant(1)~=size_nonimal(1) || size_plant(2)~=size_nonimal(2)
                msg = "plantとnominalのサイズが一致しません\n" + ...
                    "plant size:   " + num2str(size_plant) + "\n" + ...
                    "nominal size: " + num2str(size_nonimal) + "\n\n";
                error(sprintf(msg));
            end

            Delta_input = zeros(4,size_plant(2));
            for row = 1:size_plant(2)
                data = obj.gen_data_func(plant_state(:,row), nominal_state(:,row));
                Delta_input(:,row) = -1*double(predict(obj.DNNMEC_model, data'))'; % predict関数での推論
            end
        end
    end
end


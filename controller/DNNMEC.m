classdef DNNMEC < handle
    %DNNMEC
    %   クアッドコプター用モデル誤差補償器(MEC)のプログラム
    %   ディープニューラルネットワーク(DNN)で補償器を設計
    %   [Inputs]
    %    self: ドローンのagent
    %    DNN_model_filename="DNNMEC.onnx": インポートするonnxファイルの名前
    %    fMEC=0: MECなしでΔu=0
    
    %   2025/07 作成者:小関      学番:2212044
    
    properties
        self
        result
        physical_param
        parameter_name = ["mass", "Lx", "Ly", "lx", "ly", "jx", "jy", "jz", "gravity", "km1", "km2", "km3", "km4", "k1", "k2", "k3", "k4"];
        agent
        DNN_model_filename  % 読み込みたいONNXモデルのファイル名
        DNNMEC_model        % コード内でのモデル名
        dx_func             % 状態方程式の関数ハンドル
        state_renew_func    % 学習時の状態更新手法に合わせるための関数ハンドル
        gen_data_func       % 入力の次元数に合わせたデータ生成関数ハンドル
        x_pre               % 前時刻の状態
        pre_input           % 前時刻の制御入力
    end
    
    methods
        function obj = DNNMEC(self, DNN_model_filename)
            % インスタンス
            obj.self = self;
            obj.physical_param = self.parameter.get(obj.parameter_name);
            obj.dx_func = @roll_pitch_yaw_thrust_torque_physical_parameter_model;

            %-%-%-% DNN model import & define %-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%
            obj.DNN_model_filename = DNN_model_filename;
            if ~exist("controller/DNN_MODEL", "dir")
                mkdir("controller\DNN_MODEL")
            elseif isempty(dir("controller/DNN_MODEL/*.onnx"))
                error("ACSL: Do not exist <onnx> file in controller/DNN_MODEL. ")
            end
            DNN_model = importNetworkFromONNX("\DNN_MODEL\"+obj.DNN_model_filename,... % ONNXファイルインポート
                                                "InputDataFormats", "BC", ... % 入力層定義
                                                "OutputDataFormats", "BC");   % 出力層定義 "BC" -> [バッチサイズ, 特徴量]の意味
            if contains(DNN_model_filename, '21')
                dim = 21; % 21次元
                obj.gen_data_func = @(x_p,x_n) [x_p(1:3)-x_n(1:3); x_p(4:end); x_n(4:end)];
            else
                dim = 24; % 24次元
                obj.gen_data_func = @(x_p,x_n) [x_p; x_n];
            end
            dummyInput = dlarray(randn(dim,1,'single'), 'CB'); % 初期化のためのdummy入力
            obj.DNNMEC_model = initialize(DNN_model, dummyInput); % モデルの初期化
            %-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%

            obj.result.nominal_p = zeros(3,1);
            obj.result.nominal_q = zeros(3,1);
            obj.result.nominal_v = zeros(3,1);
            obj.result.nominal_w = zeros(3,1);
            obj.result.nominal_input = zeros(self.estimator.model.dim(2),1);
            obj.result.delta_input = zeros(self.estimator.model.dim(2),1);
            obj.result.input = zeros(self.estimator.model.dim(2),1);
            obj.x_pre = self.estimator.result.state.get;
            obj.pre_input = zeros(self.estimator.model.dim(2),1);
            if contains(DNN_model_filename, 'Euler') % 状態更新手法を動的に変更
                obj.state_renew_func = @(x_pre, pre_input, dt) obj.Euler(x_pre, pre_input, dt);
            elseif contains(DNN_model_filename, 'RK4')
                obj.state_renew_func = @(x_pre, pre_input, dt) obj.RK4(x_pre, pre_input, dt);
            end
            fprintf('Model file name: %s\n', obj.DNN_model_filename);
            disp('obj.result.delta_inputを表示します')
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

            % DNNへの入力データ
            data = obj.gen_data_func(x_plant, x_nominal);

            % DNN関係　閾値での制限
            obj.result.delta_input = -1*double(predict(obj.DNNMEC_model, data'))'; % predict関数での推論
            if abs(obj.result.delta_input(1))>5, obj.result.delta_input(1) = 0; end
            if abs(obj.result.delta_input(2))>1, obj.result.delta_input(2) = 0; end
            if abs(obj.result.delta_input(3))>1, obj.result.delta_input(3) = 0; end
            if abs(obj.result.delta_input(4))>1, obj.result.delta_input(4) = 0; end
            obj.result.delta_input = [0;0;0;0];

            obj.result.nominal_input = varargin{5}.controller.nominal.result.input; % ノミナル入力を保存
            obj.result.input = obj.result.nominal_input + obj.result.delta_input;
            result = obj.result;
            disp(obj.result.delta_input')
        end

        function x_plus = Euler(obj, x_pre, pre_input, dt)
            dx = obj.dx_func(x_pre, pre_input, obj.physical_param);
            x_plus = x_pre + dx*dt;
        end

        function x_plus = RK4(obj, x_pre, pre_input, dt)
            dx = @(x) obj.dx_func(x, pre_input, obj.physical_param);

            k1 = dx(x_pre);
            k2 = dx(x_pre + dt*k1/2);
            k3 = dx(x_pre + dt*k2/2);
            k4 = dx(x_pre + dt*k3);

            x_plus = x_pre + dt/6*(k1 + 2*k2 + 2*k3 + k4);
        end
    end
end


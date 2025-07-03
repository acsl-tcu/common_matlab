classdef DNNMEC < handle
    %HLC_DNNMEC
    %   2025/07 作成者:小関  学番:2212044
    %   クアッドコプター用モデル誤差補償器(MEC)のプログラム
    %   ディープニューラルネットワーク(DNN)で補償器を設計した
    
    properties
        self
        result
        param
        parameter_name = ["mass", "Lx", "Ly", "lx", "ly", "jx", "jy", "jz", "gravity", "km1", "km2", "km3", "km4", "k1", "k2", "k3", "k4"];
        agent
        DNN_model_filename
        DNNMEC_model
        x_pre
        pre_input
    end
    
    methods
        function obj = DNNMEC(self, DNN_model_filename)
            %HLC_DNNMECインスタンス
            %   [Inputs]
            %    self: agentを指す
            %    DNN_model_filename="DNNMEC.onnx": インポートするonnxファイルの名前
            obj.self = self;
            obj.param = self.parameter.get(obj.parameter_name);

            % DNN model import & define
            obj.DNN_model_filename = DNN_model_filename;
            if ~exist("controller/DNN_MODEL", "dir")
                error("ACSL: Do not exist <DNN_MODEL> folder in controller folder.")
            elseif isempty(dir("controller/DNN_MODEL/*.onnx"))
                error("ACSL: Do not exist <onnx> file in controller/DNN_MODEL. ")
            end
            DNN_model = importNetworkFromONNX("\DNN_MODEL\"+obj.DNN_model_filename);
            % input_layer = dlarray(rand(1, 24), 'CS');
            % obj.DNNMEC_model = initialize(DNN_model, input_layer);
            numerical_input_layer = inputLayer([1, 24], "CS"); % "SC"の意味が分かってない(2025/07/02時点)
            obj.DNNMEC_model = addInputLayer(DNN_model, numerical_input_layer);
            summary(obj.DNNMEC_model)

            obj.result.nominal_input = zeros(self.estimator.model.dim(2),1);
            obj.result.delta_input = zeros(self.estimator.model.dim(2),1);
            obj.result.input = zeros(self.estimator.model.dim(2),1);
            obj.x_pre = self.estimator.result.state.get;
            obj.pre_input = zeros(self.estimator.model.dim(2),1);
            disp('obj.result.delta_inputを表示します')
        end
        
        function result = do(obj, varargin)
            %doメソッド
            % ノミナル状態更新 ※状態更新の手法は学習時のものと合わせる
            if isfield(varargin{3}.Data.agent, "controller") && isfield(varargin{3}.Data.agent, "estimator") % ループの最初はLoggingされていなくて，参照できないのを回避
                obj.pre_input = varargin{3}.Data.agent.controller.result{end}.input; % LOGGERの中から前時刻の入力を取得
                obj.x_pre = varargin{3}.Data.agent.estimator.result{end}.state.get; % LOGGERの中から前時刻の状態を取得
            end
            dt = varargin{1}.dt;
            y_nominal = euler_approximation_drone(obj.x_pre, obj.pre_input, obj.param, dt);

            % プラント値取得
            y_plant = obj.self.estimator.result.state.get; % 現時刻の推定値
            % -> size = 12*1, contents = [p; q; v; w];

            % DNN関係
            tmp = double(predict(obj.DNNMEC_model, [y_plant; y_nominal]'))';
            % max,min are applied for the safty
            obj.result.delta_input = [max(0,min(10,tmp(1)));max(-1,min(1,tmp(2)));max(-1,min(1,tmp(3)));max(-1,min(1,tmp(4)))];

            % obj.result.delta_input = [-5; 0; 0; 0]; % 定数を入れてお試し
            obj.result.nominal_input = varargin{5}.controller.nominal.result.input;
            obj.result.input = obj.result.nominal_input - obj.result.delta_input;
            result = obj.result;
            disp(obj.result.delta_input')
        end
    end
end


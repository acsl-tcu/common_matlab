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
            obj.DNN_model_filename = DNN_model_filename;
            obj.result.input = zeros(self.estimator.model.dim(2),1);
            obj.result.delta_input = zeros(self.estimator.model.dim(2),1);
            obj.pre_input = zeros(self.estimator.model.dim(2),1);
        end
        
        function result = do(obj, varargin)
            %doメソッド
            % ノミナル状態更新 ※状態更新の手法は学習時のものと合わせる
            obj.pre_input = varargin{3}.Data.agent.controller.result{end}; % LOGGERの中から前時刻の入力を取得
            dt = varargin{1}.dt;
            x_pre = varargin{3}.Data.agent.estimator.result{end}.state.get; % LOGGERの中から前時刻の状態を取得
            y_nominal = euler_approximation_drone(x_pre, obj.pre_input, obj.param, dt);

            % プラント値取得
            y_plant = obj.self.estimator.result.state.get; % 現時刻の推定値
            % -> size = 12*1, contents = [w; q; p; v];

            % DNN関係
            DNN_model = importNetworkFromONNX("\DNN_MODEL\"+obj.DNN_model_filename);
            DNN_model.Initialized;
            obj.result.delta_input = predict(DNN_model, [y_plant; y_nominal]);

            obj.result.input = varargin{5}.controller.nominal.result.input + obj.result.delta_input;
            result = obj.result;
        end
    end
end


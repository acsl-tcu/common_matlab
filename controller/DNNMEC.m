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
            obj.DNN_model_filename = DNN_model_filename;
            obj.result.nominal_input = zeros(self.estimator.model.dim(2),1);
            obj.result.delta_input = zeros(self.estimator.model.dim(2),1);
            obj.result.input = zeros(self.estimator.model.dim(2),1);
            obj.x_pre = self.estimator.result.state.get;
            obj.pre_input = zeros(self.estimator.model.dim(2),1);
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

            % % DNN関係
            % if ~exist("controller/DNN_MODEL", "dir")
            %     error("ACSL: Do not exist <DNN_MODEL> folder in controller folder.")
            % elseif isempty(dir("controller/DNN_MODEL/*.onnx"))
            %     error("ACSL: Do not exist <onnx> file in controller/DNN_MODEL. ")
            % end
            % DNN_model = importNetworkFromONNX("\DNN_MODEL\"+obj.DNN_model_filename);
            % DNN_model.Initialized;
            % obj.result.delta_input = predict(DNN_model, [y_plant; y_nominal]);
            
            obj.result.delta_input = [5; 0; 0; 0]; % 定数を入れてお試し
            obj.result.nominal_input = varargin{5}.controller.nominal.result.input;
            obj.result.input = obj.result.nominal_input + obj.result.delta_input;
            result = obj.result;
        end
    end
end


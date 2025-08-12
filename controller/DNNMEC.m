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
        param
        parameter_name = ["mass", "Lx", "Ly", "lx", "ly", "jx", "jy", "jz", "gravity", "km1", "km2", "km3", "km4", "k1", "k2", "k3", "k4"];
        agent
        DNN_model_filename
        DNNMEC_model
        x_pre
        pre_input
        fMEC = 1
    end
    
    methods
        function obj = DNNMEC(self, DNN_model_filename, fMEC)
            %DNNMECインスタンス
            obj.self = self;
            obj.param = self.parameter.get(obj.parameter_name);
            if exist('fMEC', 'var'), obj.fMEC = fMEC; end

            % DNN model import & define
            obj.DNN_model_filename = DNN_model_filename;
            if ~exist("controller/DNN_MODEL", "dir")
                mkdir("controller\DNN_MODEL")
            elseif isempty(dir("controller/DNN_MODEL/*.onnx"))
                error("ACSL: Do not exist <onnx> file in controller/DNN_MODEL. ")
            end
            DNN_model = importNetworkFromONNX("\DNN_MODEL\"+obj.DNN_model_filename,... % ONNXファイルインポート
                                                "InputDataFormats", "BC", ... % 入力層定義
                                                "OutputDataFormats", "BC");   % 出力層定義 "BC" -> [バッチサイズ, 特徴量]の意味
            dummyInput = dlarray(randn(24,1,'single'), 'CB'); % 初期化のためのdummy入力
            obj.DNNMEC_model = initialize(DNN_model, dummyInput); % モデルの初期化
            if obj.fMEC==1, summary(obj.DNNMEC_model); end

            obj.result.nominal_p = zeros(3,1);
            obj.result.nominal_q = zeros(3,1);
            obj.result.nominal_v = zeros(3,1);
            obj.result.nominal_w = zeros(3,1);
            obj.result.nominal_input = zeros(self.estimator.model.dim(2),1);
            obj.result.delta_input = zeros(self.estimator.model.dim(2),1);
            obj.result.input = zeros(self.estimator.model.dim(2),1);
            obj.x_pre = self.estimator.result.state.get;
            obj.pre_input = zeros(self.estimator.model.dim(2),1);
            if obj.fMEC==1, fprintf('Model file name: %s\n', obj.DNN_model_filename);
            else, disp('No DNNMEC'); end
            disp('obj.result.delta_inputを表示します')
        end
        
        function result = do(obj, varargin)
            if obj.fMEC==1
                % ノミナル状態更新 ※状態更新の手法は学習時のものと合わせる
                if isfield(varargin{3}.Data.agent, "controller") && isfield(varargin{3}.Data.agent, "estimator")... % ループの最初はLoggingされていなくて，参照できないのを回避
                && length(varargin{3}.Data.agent.estimator.result)>=2
                    obj.pre_input = varargin{3}.Data.agent.controller.result{end}.input; % LOGGERから前時刻の入力を取得
                    obj.x_pre = varargin{3}.Data.agent.estimator.result{end}.state.get; % LOGGERから前時刻の状態を取得
                end
                dt = varargin{1}.dt;
                dx = roll_pitch_yaw_thrust_torque_physical_parameter_model(obj.x_pre, obj.pre_input, obj.param);
                x_nominal = obj.x_pre + dx*dt;
                obj.result.nominal_p = x_nominal(1:3);
                obj.result.nominal_q = x_nominal(4:6);
                obj.result.nominal_v = x_nominal(7:9);
                obj.result.nominal_w = x_nominal(10:12);
    
                % プラント値取得
                x_plant = obj.self.estimator.result.state.get; % 現時刻の推定値
                % -> size = 12*1, contents = [p; q; v; w];
    
                % % DNN関係 上下限値での制限
                % tmp = double(predict(obj.DNNMEC_model, [y_plant; y_nominal]'))';
                % % max,min are applied for the safty
                % obj.result.delta_input = [max(0,min(10,tmp(1)));max(-1,min(1,tmp(2)));...
                %                             max(-1,min(1,tmp(3)));max(-1,min(1,tmp(4)))];
    
                % DNN関係　閾値での制限
                obj.result.delta_input = -1*double(predict(obj.DNNMEC_model, [x_plant; x_nominal]'))';
                if abs(obj.result.delta_input(1))>5, obj.result.delta_input(1) = 0; end
                if abs(obj.result.delta_input(2))>0.6, obj.result.delta_input(2) = 0; end
                if abs(obj.result.delta_input(3))>0.6, obj.result.delta_input(3) = 0; end
                if abs(obj.result.delta_input(4))>0.6, obj.result.delta_input(4) = 0; end

            else
                obj.result.delta_input = zeros(obj.self.estimator.model.dim(2),1); % Δu=0
            end

            obj.result.nominal_input = varargin{5}.controller.nominal.result.input;
            obj.result.input = obj.result.nominal_input + obj.result.delta_input;
            result = obj.result;
            disp(obj.result.delta_input')
        end
    end
end


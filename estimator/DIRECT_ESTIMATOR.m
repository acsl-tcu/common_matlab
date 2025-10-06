classdef DIRECT_ESTIMATOR < handle
    % Directory generate the estimated state from the sensor output.
    % obj = DIRECT_ESTIMATOR(agent,~)
    % MOTIVEクラスからのデータ（STATE_CLASSの配列）を扱えるように修正済み
    properties
        % state
        result
        self
        model
    end
    methods
        function obj = DIRECT_ESTIMATOR(self,param)
            obj.self = self;
            obj.model = param.model;
            obj.result.state=state_copy(obj.model.state); % STATE_CLASSとしてコピー
        end

        function result=do(obj,varargin)
            % センサーデータ（Motiveの結果）を取得して、自身の推定結果を更新する
            
            % センサーデータが空の場合は何もしない
            if isempty(obj.self.sensor.result) || isempty(obj.self.sensor.result.state)
                result = obj.result;
                return;
            end

            % センサーの状態オブジェクトを取得 (Motiveは配列を返すため先頭要素を取得)
            sensor_state = obj.self.sensor.result.state(1);

            % 推定器が持つべき全プロパティ名リストを取得
            estimator_fields = fieldnames(obj.result.state);

            % 推定器の全プロパティをループ
            for i = 1:length(estimator_fields)
                fname = estimator_fields{i};

                % センサーの状態オブジェクトに同じ名前のプロパティが存在するかを安全にチェック
                if isprop(sensor_state, fname)
                    % 存在すれば、その値を取得して推定結果にセットする
                    value_from_sensor = sensor_state.(fname);
                    obj.result.state.set_state(fname, value_from_sensor);
                end
            end
            result=obj.result;
        end
        
        function show(obj)
            obj.result.state
        end
    end
end
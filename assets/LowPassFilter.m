classdef LowPassFilter < handle
    % LOWPASSFILTER 離散時間1次遅れフィルタクラス
    % [Inputs]
    %   fc: カットオフ周波数 [Hz]
    %   Ts: サンプリング周期 [s]

    %   2026/02 作成者:B4小関      学番:2212044
    
    properties (Access = public)
        Alpha       % カットオフ周波数とサンプリング周期から求まる係数
    end
    
    properties (Access = private)
        State       % 前回の出力値 y[n-1]
    end
    
    methods

        function obj = LowPassFilter(fc, Ts)
            if nargin > 0
                obj.setAlpha(fc, Ts);
            else
                obj.Alpha = 1.0; % フィルタなしの状態
            end
            obj.reset();
        end
        

        function setAlpha(obj, fc, Ts)
            % 離散時間1次遅れの設計式
            obj.Alpha = (2*pi*fc*Ts) / (1 + 2*pi*fc*Ts);
        end
        
        % --- 状態リセット ---
        function reset(obj, initialState)
            if nargin < 2, initialState = 0; end % initialStateを指定しない場合
            obj.State = initialState;
        end
        
        % --- 逐次処理 (1サンプルずつ) ---
        function y = update(obj, x)
            % y[n] = (1 - alpha) * y[n-1] + alpha * x[n]
            y = (1 - obj.Alpha) * obj.State + obj.Alpha * x;
            obj.State = y; % 次回のために状態を更新
        end
        
        % --- バッチ処理 (ベクトル一括) ---
        function y_vec = filter(obj, x_vec)
            y_vec = zeros(size(x_vec));
            for i = 1:length(x_vec)
                y_vec(i) = obj.update(x_vec(i));
            end
        end
    end
end
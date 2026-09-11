classdef THRUST2ROTOR_SUSPENDED_LOAD < handle
% THRUST2ROTOR_SUSPENDED_LOAD
% SimExp フレームワーク準拠の入力変換クラス。
% コントローラ出力 [T; tau_x; tau_y; tau_z] を
% 各ロータ推力 [f1; f2; f3; f4] に変換して result に格納する。
%
% 使用方法:
%   agent.input_transform.set_function_class("rotor", THRUST2ROTOR_SUSPENDED_LOAD(agent));
%   agent.cha_allocation.f.input_transform = "rotor";
%
% do(obj, time, cha, logger, env, agent, agent_idx) を SimExp から呼び出される。

    properties
        self    % agent インスタンス
        result  % 変換後のロータ推力 [f1; f2; f3; f4] [N]
        IIT     % 逆アロケーション行列 inv(B)
        f_max   % ロータ最大推力 [N] (デフォルト 5.0 N)
    end

    methods
        function obj = THRUST2ROTOR_SUSPENDED_LOAD(self, f_max)
            arguments
                self
                f_max = 5.0  % 各ロータの最大推力 [N]
            end
            obj.self  = self;
            obj.f_max = f_max;

            % アロケーション行列 B の構築
            % B * [f1;f2;f3;f4] = [T; tau_x; tau_y; tau_z]
            Lx  = self.parameter.Lx;
            Ly  = self.parameter.Ly;
            lx  = self.parameter.lx;
            ly  = self.parameter.ly;
            km1 = self.parameter.km1;
            km2 = self.parameter.km2;
            km3 = self.parameter.km3;
            km4 = self.parameter.km4;

            B = [1,    1,       1,       1;
                 -ly,  -ly,     (Ly-ly), (Ly-ly);
                  lx,  -(Lx-lx), lx,   -(Lx-lx);
                  km1, -km2,    -km3,    km4];

            obj.IIT = inv(B);  % [f1;f2;f3;f4] = IIT * [T; tau_x; tau_y; tau_z]
            obj.result = zeros(4, 1);
        end

        function u = do(obj, varargin)
            % SimExp フレームワーク準拠の呼び出しインタフェース
            % varargin{1} = time
            % varargin{2} = cha  (フライトフェーズ文字: 'f', 't', 'l', ...)
            % varargin{3} = logger
            % varargin{4} = env
            % varargin{5} = agent (配列)
            % varargin{6} = agent_idx

            % コントローラの出力 [T; tau_x; tau_y; tau_z] を取得
            agent_idx = varargin{6};
            ctrl_input = varargin{5}(agent_idx).controller.result.input;

            % 逆アロケーション: [T; tau_x; tau_y; tau_z] -> [f1; f2; f3; f4]
            f_rotors = obj.IIT * ctrl_input(1:4);

            % ロータ推力を物理限界にクリップ（非負・最大値）
            f_rotors = max(0, min(obj.f_max, f_rotors));

            obj.result = f_rotors;
            u = f_rotors;
        end
    end
end

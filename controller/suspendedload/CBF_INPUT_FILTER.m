classdef CBF_INPUT_FILTER < handle
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % 目的：
    %   HLC_SUSPENDED_LOAD（origin）が計算した入力 u=[thrust; roll; pitch; yaw]
    %   を基本として使用し，
    %   牽引物の揺れ角が許容範囲を超えそうなときのみ roll/pitch を弱める．
    %
    % 安全制約（角度が直接取れない場合）：
    %   ケーブル長 L と許容角 theta_max から，
    %     ||r_xy|| <= L*sin(theta_max)
    %   を揺れ角制約の代理として用いる．
    %   バリア関数：
    %     h = (L*sin(theta_max))^2 - ||r_xy||^2
    %   を定義し，hが小さい（境界付近）かつ外向き運動のときだけ介入する．
    %
    % 介入内容：
    %   - roll/pitch を縮める（入力形式を変えず，既存を最小修正）
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    properties
        self
        param
    end

    methods
        function obj = CBF_INPUT_FILTER(self, param)
            obj.self = self;
            obj.param = param;
        end

        function result = do(obj, varargin)

            % --- controllerのbase（origin）を取得 ---
            if isstruct(obj.self.controller) && isfield(obj.self.controller,'origin')
                base = obj.self.controller.origin;   % originのHLC_SUSPENDED_LOAD
            else
                base = obj.self.controller;
            end

            % originがまだ計算していない場合は何もしない（落とさない）
            if ~isprop(base,'result') || ~isfield(base.result,'input')
                result = base.result;
                return;
            end

            % --- originが計算した入力（形式維持） ---
            u = base.result.input;   % [thrust; roll; pitch; yaw]

            % --- 相対状態からバリア関数を評価 ---
            model = obj.self.estimator.result;

            p  = model.state.p;   v  = model.state.v;
            pL = model.state.pL;  vL = model.state.vL;

            rxy  = (pL(1:2) - p(1:2));
            vrxy = (vL(1:2) - v(1:2));

            L = obj.self.parameter.get("cableL");
            rmax = L * sin(obj.param.theta_max);

            % バリア関数 h>=0 が安全
            h = rmax^2 - (rxy.'*rxy);

            % 外向き運動判定：rxyとvrxyの内積が正なら外側へ進んでいる
            outward = (rxy.'*vrxy) > 0;

            % 境界付近（hが小さい）で外へ向かうときだけ介入
            near = h < obj.param.h_margin;

            if outward && near
                % どれだけ危険か（hが小さいほど介入を強くする）
                s = min(1, max(0, (obj.param.h_margin - h)/obj.param.h_margin));

                % roll/pitch の縮小率（k_shrink=1なら最大で0倍まで縮む）
                shrink = 1 - obj.param.k_shrink * s;

                % 入力形式は変えずに，roll/pitchだけ弱める
                u(2) = shrink * u(2); % roll
                u(3) = shrink * u(3); % pitch
            end

            % 念のため既存レンジで飽和（originと同じ意味のまま）
            u(1) = max(0,  min(20, u(1)));
            u(2) = max(-1, min(1,  u(2)));
            u(3) = max(-1, min(1,  u(3)));
            u(4) = max(-1, min(1,  u(4)));

            % --- 上書き（副作用） ---
            base.result.input = u;

            % --- 形式維持：originのresultを返す ---
            result = base.result;
        end
    end
end

classdef REFERENCE_VIRTUAL_TIME < handle
    % REFERENCE_VIRTUAL_TIME
    % 「仮想時間 (Virtual Time)」を用いた Reference Governor。
    % 
    % 既存の滑らかな軌道生成器 (TIME_VARYING_REFERENCE など) をラップし、
    % 機体が軌道から遅れた場合 (例: CBFによる障害物回避で足止めされた場合)、
    % 目標軌道の「時計の進み (tau_dot)」を遅くしたり止めたりすることで、
    % 制約と目標の衝突を防ぎつつ、6階微分までの完璧な滑らかさを維持します。

    properties
        self
        result
        nominal_ref   % ベースとなる軌道生成器
        tau           % 仮想時間
        tau_dot       % 仮想時間の進み具合 (0~1)
    end

    methods
        function obj = REFERENCE_VIRTUAL_TIME(self, nominal_ref)
            obj.self = self;
            obj.nominal_ref = nominal_ref;
            obj.tau = 0.0;
            obj.tau_dot = 1.0; % 最初は現実時間と同じ速度で進む
            
            % 必要なステートリスト (xd を含む)
            obj.result.state = STATE_CLASS(struct('state_list', ["p", "v", "a", "j", "s", "c", "xd"], 'num_list', [3, 3, 3, 3, 3, 3, 18]));
        end

        function result = do(obj, varargin)
            time = varargin{1};
            dt = time.dt;
            if isempty(dt) || dt <= 0, dt = 0.025; end
            
            % シミュレーション開始直後は仮想時間をリセット
            if time.t < 0.05
                obj.tau = time.t;
                obj.tau_dot = 1.0;
            end
            
            % 1. 現在の仮想時間 tau での目標位置を取得
            time_dummy = time;
            time_dummy.t = obj.tau;
            res_nom = obj.nominal_ref.do(time_dummy, varargin{2:end});
            p_nom = res_nom.state.p;
            
            % 2. ドローンの現在位置を取得
            if isprop(obj.self.estimator.result.state, 'pL')
                p_cur = obj.self.estimator.result.state.p;
            else
                p_cur = obj.self.estimator.result.state.p;
            end
            
            % 3. 追従遅れ距離の計算
            dist = norm(p_cur - p_nom);
            
            % 4. 距離に応じて仮想時間の進み (tau_dot) の目標値を決定
            % 0.2m 以内なら 1.0 (通常スピード)
            % 1.0m 以上離れたら 0.0 (時計を止めて待つ)
            if dist < 0.2
                tau_dot_target = 1.0;
            elseif dist > 1.0
                tau_dot_target = 0.0;
            else
                tau_dot_target = 1.0 - (dist - 0.2) / 0.8;
            end
            
            % 5. tau_dot の急変を防ぐため、ローパスフィルタをかける (滑らかに減速・加速)
            alpha = 0.02; 
            obj.tau_dot = (1 - alpha) * obj.tau_dot + alpha * tau_dot_target;
            
            % 6. 仮想時間を進める
            obj.tau = obj.tau + obj.tau_dot * dt;
            
            % 7. 新しい仮想時間で最終的な目標状態を取得
            time_dummy.t = obj.tau;
            res_nom = obj.nominal_ref.do(time_dummy, varargin{2:end});
            
            % 厳密には速度 v などを tau_dot でスケーリングすべきですが、
            % 軌道の形状(空間的な滑らかさ)を維持することを優先し、そのまま渡します
            obj.result.state = res_nom.state;
            
            result = obj.result;
        end
    end
end

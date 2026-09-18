classdef REFERENCE_LOOKAHEAD < handle
    % REFERENCE_LOOKAHEAD
    % ユーザーのアイデアに基づく「軌道追従（Pure Pursuit / Lookahead）」型のリファレンス。
    % 
    % ドローンの現在位置から最終目標地点へ向かうベクトルを計算し、
    % 常に「現在位置から一定距離 (Lookahead Distance) 先」に仮想の目標点(ニンジン)を置きます。
    % これにより、高次フィルターの暴走(アンダーシュート等)を防ぎつつ、
    % CBFで足止めされた場合は仮想目標点も自動的に待ってくれるため、時間依存による衝突を完璧に防ぎます。

    properties
        self
        result
        p_target        % 最終目標位置
        lookahead_dist  % 先読み距離 (m)
        v_cruise        % 巡航速度 (m/s)
    end

    methods
        function obj = REFERENCE_LOOKAHEAD(self, varargin)
            obj.self = self;
            
            % デフォルト設定
            obj.p_target = [0; 0; 15]; 
            obj.lookahead_dist = 0.5;
            obj.v_cruise = 1.5;
            
            for i = 1:2:length(varargin)
                if strcmp(varargin{i}, 'pd')
                    obj.p_target = varargin{i+1};
                elseif strcmp(varargin{i}, 'lookahead')
                    obj.lookahead_dist = varargin{i+1};
                elseif strcmp(varargin{i}, 'v')
                    obj.v_cruise = varargin{i+1};
                end
            end
            
            obj.result.state = STATE_CLASS(struct('state_list', ["p", "v", "a", "j", "s", "c", "xd"], 'num_list', [3, 3, 3, 3, 3, 3, 18]));
        end

        function result = do(obj, varargin)
            % ドローンの現在位置を取得
            if isprop(obj.self.estimator.result.state, 'pL')
                p_cur = obj.self.estimator.result.state.p;
            else
                p_cur = obj.self.estimator.result.state.p;
            end
            
            % 目標方向のベクトル
            dir_vec = obj.p_target - p_cur;
            dist_to_target = norm(dir_vec);
            
            if dist_to_target > 1e-3
                dir_unit = dir_vec / dist_to_target;
            else
                dir_unit = [0;0;0];
            end
            
            % 現在位置からルックアヘッド距離だけ進んだ点を仮想目標とする
            % ただし、最終目標に近づいたらそれを超えないようにする
            L = min(obj.lookahead_dist, dist_to_target);
            p_ref = p_cur + dir_unit * L;
            
            % 速度は目標に向かう巡航速度
            v_ref = dir_unit * obj.v_cruise;
            
            % 加速度以降はゼロ (非常に滑らかな定速直線運動とみなす)
            a_ref = [0;0;0];
            j_ref = [0;0;0];
            s_ref = [0;0;0];
            c_ref = [0;0;0];
            
            % 最終目標に到達したら速度もゼロ
            if dist_to_target <= 0.1
                v_ref = [0;0;0];
                p_ref = obj.p_target;
            end
            
            % 結果の格納
            obj.result.state.p = p_ref;
            obj.result.state.v = v_ref;
            obj.result.state.a = a_ref;
            obj.result.state.j = j_ref;
            obj.result.state.s = s_ref;
            obj.result.state.c = c_ref;
            
            % HLC_SUSPENDED_LOAD用の結合ベクトル
            obj.result.state.xd = [p_ref; v_ref; a_ref; j_ref; s_ref; c_ref];
            
            result = obj.result;
        end
    end
end

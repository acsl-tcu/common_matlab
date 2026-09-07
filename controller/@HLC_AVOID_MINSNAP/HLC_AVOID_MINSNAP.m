classdef HLC_AVOID_MINSNAP < HLC_SUSPENDED_LOAD
    % =========================================================================
    % 位置制約に基づく滑らかな軌道変形 (Minimum Snap / B-Spline アプローチ)
    % APFのような「力」を使わず、障害物との距離に基づいて幾何学的な
    % 安全な回避経由点（Corridor）を計算し、そこへ至る高次多項式を生成します。
    % =========================================================================
    properties
        avoid_state   % 0: 通常, 1: 回避中
        t_start       % 回避開始時間
        T_avoid       % 回避にかける時間
        dev_dir       % 回避する方向ベクトル
        dev_max       % 最大回避距離
    end
    
    methods
        function obj = HLC_AVOID_MINSNAP(self, param)
            obj@HLC_SUSPENDED_LOAD(self, param);
            obj.avoid_state = 0;
            obj.t_start = 0;
            obj.T_avoid = 4.0;
            obj.dev_dir = zeros(3,1);
            obj.dev_max = 2.0;
        end
        
        function result = do(obj, time, varargin)
            agent_obj = varargin{4};
            idx = varargin{5};
            
            xd_nom = agent_obj(idx).reference.result.state.xd;
            if length(xd_nom) < 28; xd_nom = [xd_nom; zeros(28 - length(xd_nom), 1)]; end
            
            p_curr = agent_obj(idx).estimator.result.state.p;
            v_curr = agent_obj(idx).estimator.result.state.v;
            
            % 障害物の取得
            obs_list = ENVIRONMENT_OBSTACLE_ELLIPSOID();
            
            % 状態遷移ロジック (簡易的なローカルプランナ)
            if obj.avoid_state == 0
                for k = 1:length(obs_list)
                    obs = obs_list(k);
                    diff = obs.p_obs - p_curr;
                    dist = norm(diff);
                    
                    % 前方に障害物がある場合 (距離が7m以下、かつ進行方向)
                    if dist < 7.0 && dot(diff, v_curr) > 0.1
                        obj.avoid_state = 1;
                        obj.t_start = time.t;
                        
                        % 進行方向(v_curr)に対して直交する安全な回避方向(Corridor)を決定
                        v_norm = v_curr / norm(v_curr);
                        % Y軸方向に避けるか、X軸方向に避けるかの簡単な判定
                        cross_vec = cross(v_norm, [0;0;1]); 
                        if norm(cross_vec) < 0.1
                            cross_vec = [1;0;0];
                        end
                        obj.dev_dir = cross_vec / norm(cross_vec);
                        obj.dev_max = 2.5; % 2.5m横に避ける
                        break;
                    end
                end
            end
            
            % Minimum Snap (7次多項式) オフセットの計算
            p_off = zeros(3,1); v_off = zeros(3,1); a_off = zeros(3,1);
            j_off = zeros(3,1); s_off = zeros(3,1); d5_off = zeros(3,1); d6_off = zeros(3,1);
            
            if obj.avoid_state == 1
                tau = (time.t - obj.t_start) / obj.T_avoid;
                
                if tau <= 1.0
                    % 0 -> dev_max への滑らかな遷移 (Minimum Snap 7次多項式)
                    % s(tau) = -20*tau^7 + 70*tau^6 - 84*tau^5 + 35*tau^4
                    s   = -20*tau^7 + 70*tau^6 - 84*tau^5 + 35*tau^4;
                    ds  = (-140*tau^6 + 420*tau^5 - 420*tau^4 + 140*tau^3) / obj.T_avoid;
                    dds = (-840*tau^5 + 2100*tau^4 - 1680*tau^3 + 420*tau^2) / (obj.T_avoid^2);
                    jks = (-4200*tau^4 + 8400*tau^3 - 5040*tau^2 + 840*tau) / (obj.T_avoid^3);
                    sps = (-16800*tau^3 + 25200*tau^2 - 10080*tau + 840) / (obj.T_avoid^4);
                    
                    mag = obj.dev_max;
                    p_off = mag * s * obj.dev_dir;
                    v_off = mag * ds * obj.dev_dir;
                    a_off = mag * dds * obj.dev_dir;
                    j_off = mag * jks * obj.dev_dir;
                    s_off = mag * sps * obj.dev_dir;
                    
                elseif tau > 1.0 && tau <= 2.0
                    % dev_max -> 0 への復帰
                    tau_ret = tau - 1.0;
                    s   = 1.0 - (-20*tau_ret^7 + 70*tau_ret^6 - 84*tau_ret^5 + 35*tau_ret^4);
                    ds  = -(-140*tau_ret^6 + 420*tau_ret^5 - 420*tau_ret^4 + 140*tau_ret^3) / obj.T_avoid;
                    dds = -(-840*tau_ret^5 + 2100*tau_ret^4 - 1680*tau_ret^3 + 420*tau_ret^2) / (obj.T_avoid^2);
                    jks = -(-4200*tau_ret^4 + 8400*tau_ret^3 - 5040*tau_ret^2 + 840*tau_ret) / (obj.T_avoid^3);
                    sps = -(-16800*tau_ret^3 + 25200*tau_ret^2 - 10080*tau_ret + 840) / (obj.T_avoid^4);
                    
                    mag = obj.dev_max;
                    p_off = mag * s * obj.dev_dir;
                    v_off = mag * ds * obj.dev_dir;
                    a_off = mag * dds * obj.dev_dir;
                    j_off = mag * jks * obj.dev_dir;
                    s_off = mag * sps * obj.dev_dir;
                    
                else
                    % 回避完了
                    obj.avoid_state = 0;
                end
            end
            
            % xd_nom に Minimum Snap オフセットを加算
            xd_mod = xd_nom;
            xd_mod(1:3)   = xd_nom(1:3)   + p_off;
            xd_mod(5:7)   = xd_nom(5:7)   + v_off;
            xd_mod(9:11)  = xd_nom(9:11)  + a_off;
            xd_mod(13:15) = xd_nom(13:15) + j_off;
            xd_mod(17:19) = xd_nom(17:19) + s_off;
            xd_mod(21:23) = xd_nom(21:23) + d5_off;
            xd_mod(25:27) = xd_nom(25:27) + d6_off;
            
            agent_obj(idx).reference.result.state.xd = xd_mod;
            
            % スーパークラスの呼び出し (ベースコントローラ)
            result = do@HLC_SUSPENDED_LOAD(obj, time, varargin{:});
            
            agent_obj(idx).reference.result.state.xd = xd_nom;
            obj.result.p_off = p_off;
            result = obj.result;
        end
    end
end

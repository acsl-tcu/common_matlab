classdef HLC_DUAL_LAYER_DYN_DYN < HLC_SUSPENDED_LOAD_ELLIPSOID_CBF_DYN
    % =========================================================================
    % 双対レイヤー安全制御 (Dual-Layer Safety Controller)
    % 上位レイヤー: C^6連続 楕円体人工ポテンシャル場 (APF) による局所解・動的障害物回避
    % 下位レイヤー: 推力動的拡張を用いた入力限界厳密HOCBF (継承元)
    % =========================================================================
    properties
        p_off; v_off; a_off; j_off; s_off; d5_off;
    end
    
    methods
        function obj = HLC_DUAL_LAYER_DYN(self, param)
            obj@HLC_SUSPENDED_LOAD_ELLIPSOID_CBF_DYN(self, param);
            obj.p_off  = zeros(3,1);
            obj.v_off  = zeros(3,1);
            obj.a_off  = zeros(3,1);
            obj.j_off  = zeros(3,1);
            obj.s_off  = zeros(3,1);
            obj.d5_off = zeros(3,1);
        end
        
        function result = do(obj, time, varargin)
            agent_obj = varargin{4};
            idx = varargin{5};
            
            % ノミナル軌道取得
            xd_nom = agent_obj(idx).reference.result.state.xd;
            if length(xd_nom) < 28; xd_nom = [xd_nom; zeros(28 - length(xd_nom), 1)]; end
            
            % 状態取得 (機体と荷物)
            p_drone = agent_obj(idx).estimator.result.state.p;
            p_load = agent_obj(idx).estimator.result.state.pL;
            v_drone = agent_obj(idx).estimator.result.state.v;
            
            % ========================================================
            % 【上位レイヤー】楕円体APFによる動的障害物回避力の計算
            % 機体と荷物の中点 p_mid を代表点としてポテンシャルを計算
            % ========================================================
            p_mid = (p_drone + p_load) / 2.0;
            obs_list = ENVIRONMENT_OBSTACLE_DYNAMIC(time); % 動的障害物モデルに変更
            F_rep = zeros(3,1);
            
            for k = 1:length(obs_list)
                obs = obs_list(k);
                
                % 動的障害物対応 (v_obsが存在すれば相対速度を計算)
                v_obs = zeros(3,1);
                if isfield(obs, 'v_obs') && ~isempty(obs.v_obs)
                    v_obs = obs.v_obs;
                end
                
                % 楕円体近似距離計算 (HOCBFと同じマージン感覚)
                diff = p_mid - obs.p_obs;
                diff_norm = obs.R_obs' * diff;
                Q_inv = inv(obs.Q_obs);
                
                dist_ellip = sqrt(diff_norm' * (Q_inv^2) * diff_norm) - 1.0;
                d_surf = dist_ellip - obs.d_margin;
                
                r_influence = 5.0; 
                if d_surf < r_influence && d_surf > 0.01
                    eta = 2.0; % 斥力ゲイン
                    dir = obs.R_obs * (Q_inv^2) * diff_norm; 
                    dir = dir / norm(dir);
                    
                    % 相対速度に基づく予測ブレーキ強化 (動的障害物への対応)
                    v_rel = v_drone - v_obs;
                    v_approach = max(0, -(dir' * v_rel));
                    
                    % 距離が近く、かつ接近速度が速いほど強い斥力を発生
                    rep_mag = eta * (1.0 / d_surf - 1.0 / r_influence) * (1.0 / d_surf^2) * (1.0 + 0.5 * v_approach);
                    F_rep = F_rep + rep_mag * dir;
                end
            end
            
            % ========================================================
            % C^6クリティカルダンピングフィルタ
            % ========================================================
            dt = 0.025; if isprop(time, 'dt') && time.dt > 0; dt = time.dt; end
            lambda = 4.0;
            c0 = lambda^6; c1 = 6*lambda^5; c2 = 15*lambda^4; c3 = 20*lambda^3; c4 = 15*lambda^2; c5 = 6*lambda;
            
            d6_calc = c0 * F_rep - (c5 * obj.d5_off + c4 * obj.s_off + c3 * obj.j_off + c2 * obj.a_off + c1 * obj.v_off + c0 * obj.p_off);
            
            old_d5 = obj.d5_off;
            obj.d5_off = obj.d5_off + d6_calc    * dt;
            obj.s_off  = obj.s_off  + obj.d5_off * dt;
            obj.j_off  = obj.j_off  + obj.s_off  * dt;
            obj.a_off  = obj.a_off  + obj.j_off  * dt;
            obj.v_off  = obj.v_off  + obj.a_off  * dt;
            obj.p_off  = obj.p_off  + obj.v_off  * dt;
            d6_smooth = (obj.d5_off - old_d5) / dt;
            
            % ========================================================
            % オフセット合成
            % ========================================================
            xd_mod = xd_nom;
            xd_mod(1:3)   = xd_nom(1:3)   + obj.p_off;
            xd_mod(5:7)   = xd_nom(5:7)   + obj.v_off;
            xd_mod(9:11)  = xd_nom(9:11)  + obj.a_off;
            xd_mod(13:15) = xd_nom(13:15) + obj.j_off;
            xd_mod(17:19) = xd_nom(17:19) + obj.s_off;
            xd_mod(21:23) = xd_nom(21:23) + obj.d5_off;
            xd_mod(25:27) = xd_nom(25:27) + d6_smooth;
            
            % ========================================================
            % 【下位レイヤー】実入力HOCBFの実行
            % ========================================================
            agent_obj(idx).reference.result.state.xd = xd_mod;
            
            % スーパークラス(下位レイヤーCBF)の呼び出し
            result = do@HLC_SUSPENDED_LOAD_ELLIPSOID_CBF_DYN(obj, time, varargin{:});
            
            % ノミナル軌道に戻す
            agent_obj(idx).reference.result.state.xd = xd_nom;
            
            obj.result.p_off = obj.p_off;
            result = obj.result;
        end
    end
end

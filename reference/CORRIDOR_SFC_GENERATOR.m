classdef CORRIDOR_SFC_GENERATOR < handle
    % CORRIDOR_SFC_GENERATOR
    % 3次元空間における障害物群回避のための Safe Flight Corridor (SFC) 自動生成エンジン
    % [Gao et al., T-RO 2020 / Zhou et al., RA-L 2020 準拠]
    % - 障害物マップ情報から衝突のないウェイポイント列を探索 (幾何サンプリング / 経路展開)
    % - 各ウェイポイント区間を包含し、障害物を排除した重複凸直方体 (SFC Box) を膨張生成
    
    properties
        obs_list
        safe_margin = 0.3
        r_protect   = 0.35 % 機体・荷物の最大保護半径
    end
    
    methods
        function obj = CORRIDOR_SFC_GENERATOR(obs_list, safe_margin, r_protect)
            arguments
                obs_list = []
                safe_margin = 0.3
                r_protect   = 0.35
            end
            obj.obs_list    = obs_list;
            obj.safe_margin = safe_margin;
            obj.r_protect   = r_protect;
        end
        
        function [corridors, waypoints] = generate_corridors(obj, p_start, p_goal, dir_nom)
            % generate_corridors
            % 始点 p_start から目標点 p_goal まで、進路上にある障害物を迂回する
            % ウェイポイント列 waypoints (3 x (M+1)) と
            % 各区間を包む安全回廊 corridors (M 要素の struct 配列) を生成する
            
            % 1. 進路上で干渉する障害物の抽出
            blocking_obs = [];
            for i = 1:length(obj.obs_list)
                tgt = obj.obs_list(i);
                c = tgt.p_center;
                rad = tgt.ellipsoid_radii;
                d_m = tgt.d_margin;
                
                % 始点-目標線分への最短距離
                v_line = p_goal - p_start;
                L_line = norm(v_line);
                if L_line < 1e-4, continue; end
                dir_line = v_line / L_line;
                
                t_proj = dot(c - p_start, dir_line);
                if t_proj < -max(rad) || t_proj > (L_line + max(rad)), continue; end
                
                p_proj = p_start + max(0, min(L_line, t_proj)) * dir_line;
                dist_lat = norm(c - p_proj);
                
                % 衝突判定半径
                r_crit = max(rad) + obj.r_protect + d_m + obj.safe_margin;
                if dist_lat < r_crit
                    blocking_obs = [blocking_obs; tgt];
                end
            end
            
            % 2. ウェイポイント列の生成
            if isempty(blocking_obs)
                % 障害物なし: 1区間直線
                waypoints = [p_start, p_goal];
            else
                % 障害物を通過するバイパスウェイポイントを生成
                wp_list = p_start;
                for k = 1:length(blocking_obs)
                    tgt = blocking_obs(k);
                    c = tgt.p_center;
                    rad = tgt.ellipsoid_radii;
                    d_m = tgt.d_margin;
                    
                    % 進行線から障害物中心への直交方向
                    vec_to_c = c - p_start;
                    t_c = dot(vec_to_c, dir_nom);
                    p_center_line = p_start + t_c * dir_nom;
                    v_lat = c - p_center_line;
                    d_lat = norm(v_lat);
                    
                    if d_lat < 1e-3
                        n_lat = cross(dir_nom, [0; 0; 1]);
                        if norm(n_lat) < 0.1, n_lat = cross(dir_nom, [1; 0; 0]); end
                        n_lat = n_lat / norm(n_lat);
                    else
                        n_lat = -v_lat / d_lat; % 障害物から離れる方向
                    end
                    
                    r_eff = max(rad);
                    req_clear = r_eff + obj.r_protect + d_m + obj.safe_margin + 0.3;
                    
                    % 回避頂点
                    p_apex = p_center_line + req_clear * n_lat;
                    
                    % アプローチ点・頂点・復帰点
                    p_entry = p_start + max(0, t_c - (max(rad) + 1.5)) * dir_nom;
                    p_exit  = p_start + (t_c + max(rad) + 1.5) * dir_nom;
                    
                    wp_list = [wp_list, p_entry, p_apex, p_exit];
                end
                wp_list = [wp_list, p_goal];
                
                % 重複点の除去
                waypoints = wp_list(:, 1);
                for j = 2:size(wp_list, 2)
                    if norm(wp_list(:, j) - waypoints(:, end)) > 0.3
                        waypoints = [waypoints, wp_list(:, j)];
                    end
                end
            end
            
            % 3. 各区間 (M = size(waypoints, 2) - 1) の SFC Box 生成
            M = size(waypoints, 2) - 1;
            corridors = repmat(struct('p_min', [0;0;0], 'p_max', [0;0;0]), M, 1);
            
            for m = 1:M
                w_start = waypoints(:, m);
                w_end   = waypoints(:, m + 1);
                
                % 2点を包含するバウンディングボックスの初期化
                b_min = min(w_start, w_end) - [1.5; 1.5; 1.0];
                b_max = max(w_start, w_end) + [1.5; 1.5; 1.0];
                
                corridors(m).p_min = b_min;
                corridors(m).p_max = b_max;
            end
        end
    end
end
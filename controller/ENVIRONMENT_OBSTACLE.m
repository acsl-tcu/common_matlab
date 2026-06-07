function obs = ENVIRONMENT_OBSTACLE()
% ENVIRONMENT_OBSTACLE 障害物の配置情報を一元管理する関数（全形状網羅・動画同期版）
% 
% 制御バリア関数 (CBF) の球体判定を満たすため、
% さまざまな原形をすっぽり包み込む「最小の真球 (Bounding Sphere)」を自動計算します。

    % 共通の安全マージン厚み
    margin_val = 0.7;

    %% =========================================================================
    %  1. 円柱 (Cylinder) -> ポールや電柱など (現在アクティブ)
    %  =========================================================================
    % 前進軌道（Y=0からY=15）の途中でしっかりすれ違うよう、中心座標を調整しています
    p_center1 = [0.0; 14.5; 3]; 
    cyl_param = [0.5, 6.0]; % [底面半径 r, 高さ h]
    
    obs(1).type = 'cylinder';
    obs(1).raw_param = cyl_param;
    obs(1).p_center = p_center1;
    [obs(1).p_obs, obs(1).r_obs] = get_bounding_sphere('cylinder', p_center1, cyl_param);
    obs(1).margin = margin_val;            
    obs(1).R_safe = obs(1).r_obs + obs(1).margin; 

    % %% =========================================================================
    % %  2. 真球 (Sphere)
    % % =========================================================================
    % p_center2 = [-2.0; 6.0; 1.0]; 
    % r_sphere  = 0.5; % 単純な半径
    % 
    % obs(2).type = 'sphere';
    % obs(2).raw_param = r_sphere;
    % obs(2).p_center = p_center2;
    % [obs(2).p_obs, obs(2).r_obs] = get_bounding_sphere('sphere', p_center2, r_sphere);
    % obs(2).margin = margin_val;            
    % obs(2).R_safe = obs(2).r_obs + obs(2).margin; 
    % 
    % %% =========================================================================
    % %  3. 直方体 / 四角柱 (Box) -> コンテナ、ビル、壁など
    % % =========================================================================
    % p_center3 = [2.0; 6.0; 1.0];
    % box_param = [1.0, 0.8, 1.5]; % [X幅 dx, Y幅 dy, Z幅 dz]
    % 
    % obs(3).type = 'box';
    % obs(3).raw_param = box_param;
    % obs(3).p_center = p_center3;
    % [obs(3).p_obs, obs(3).r_obs] = get_bounding_sphere('box', p_center3, box_param);
    % obs(3).margin = margin_val;            
    % obs(3).R_safe = obs(3).r_obs + obs(3).margin; 
    % 
    % %% =========================================================================
    % %  4. 正三角柱 (Prism) -> 屋根状の構造物など
    % % =========================================================================
    % p_center4 = [-1.5; 10.0; 1.0];
    % prism_param = [1.2, 2.0]; % [底面正三角形の1辺の長さ a, 高さ h]
    % 
    % obs(4).type = 'prism';
    % obs(4).raw_param = prism_param;
    % obs(4).p_center = p_center4;
    % [obs(4).p_obs, obs(4).r_obs] = get_bounding_sphere('prism', p_center4, prism_param);
    % obs(4).margin = margin_val;            
    % obs(4).R_safe = obs(4).r_obs + obs(4).margin; 
    % 
    % %% =========================================================================
    % %  5. 円錐 (Cone) -> カラーコーン、木（簡易表現）など
    % % =========================================================================
    % p_center5 = [1.5; 10.0; 1.0];
    % cone_param = [0.6, 1.8]; % [底面半径 r, 高さ h]
    % 
    % obs(5).type = 'cone';
    % obs(5).raw_param = cone_param;
    % obs(5).p_center = p_center5;
    % [obs(5).p_obs, obs(5).r_obs] = get_bounding_sphere('cone', p_center5, cone_param);
    % obs(5).margin = margin_val;            
    % obs(5).R_safe = obs(5).r_obs + obs(5).margin; 
    % 
    % %% =========================================================================
    % %  6. 正四角錐 (Pyramid) -> ピラミッド形状、テントなど
    % % =========================================================================
    % p_center6 = [0.0; 12.0; 1.0];
    % pyramid_param = [1.5, 1.2]; % [底面の1辺の長さ a, 高さ h]
    % 
    % obs(6).type = 'pyramid';
    % obs(6).raw_param = pyramid_param;
    % obs(6).p_center = p_center6;
    % [obs(6).p_obs, obs(6).r_obs] = get_bounding_sphere('pyramid', p_center6, pyramid_param);
    % obs(6).margin = margin_val;            
    % obs(6).R_safe = obs(6).r_obs + obs(6).margin; 
    % 
    % %% =========================================================================
    % %  7. カスタム多面体 (Custom) -> 斜めの柱、歪んだ幾何学形状など
    % % =========================================================================
    % vertices = [
    %     -0.5,  0.5,  0.5, -0.5, -0.2,  0.2,  0.1, -0.3; % X
    %     14.5, 14.5, 15.5, 15.5, 14.7, 14.7, 15.3, 15.3; % Y
    %      0.0,  0.0,  0.0,  0.0,  2.5,  2.5,  2.5,  2.5  % Z
    % ];
    % 
    % obs(7).type = 'custom';
    % obs(7).raw_param = vertices;
    % obs(7).p_center = mean(vertices, 2);
    % [obs(7).p_obs, obs(7).r_obs] = get_bounding_sphere('custom', [], vertices);
    % obs(7).margin = margin_val;            
    % obs(7).R_safe = obs(7).r_obs + obs(7).margin; 
end





% %% =========================================================================
% %  サブ関数: 真球包囲ロジック (コア計算部分)
% %  =========================================================================
% function [p_obs, r_obs] = get_bounding_sphere(type, p_center, params)
% % GET_BOUNDING_SPHERE 任意の形状をすっぽり覆う真球の中心と半径を計算する（拡張版）
% %
% % 入力:
% %   type     : 'sphere', 'cylinder', 'box', 'prism', 'cone', 'pyramid', 'custom'
% %   p_center : 形状の中心（または基準点）座標 [x; y; z]
% %   params   : 各形状に応じたパラメータ
% %
% % 出力:
% %   p_obs    : 真球の中心座標 [x; y; z]
% %   r_obs    : 真球の半径 (スカラー)
% 
%     p_obs = p_center; 
% 
%     switch lower(type)
%         case 'sphere' % 球
%             r_obs = params; % params: 半径
% 
%         case 'cylinder' % 円柱
%             % params: [底面半径 r, 高さ h]
%             r_obs = sqrt(params(1)^2 + (params(2)/2)^2);
% 
%         case 'box' % 四角柱
%             % params: [X幅dx, Y幅dy, Z幅dz]
%             r_obs = sqrt((params(1)/2)^2 + (params(2)/2)^2 + (params(3)/2)^2);
% 
%         case 'prism' % 三角錐
%             % params: [底面正三角形の1辺 a, 高さ h]
%             r_tri = params(1) / sqrt(3);
%             r_obs = sqrt(r_tri^2 + (params(2)/2)^2);
% 
%         case 'cone' % 円錐
%             % --- 5. 円錐 ---
%             % params: [底面半径 r, 高さ h]
%             r_cone = params(1);
%             h_cone = params(2);
%             % 簡易的に底面中心から高さh/2を真球中心とし、頂点と底面端をカバーする
%             % 頂点までの距離: h/2, 底面フチまでの距離: sqrt(r^2 + (h/2)^2)
%             r_obs = max(h_cone/2, sqrt(r_cone^2 + (h_cone/2)^2));
% 
%         case 'pyramid' % 資格推
%             % --- 6. 正四角錐 ---
%             % params: [底面の1辺 a, 高さ h]
%             a = params(1);
%             h = params(2);
%             r_bottom = sqrt(2) * (a / 2); % 底面の中心から角までの距離
%             % 底面中心から高さh/2を真球中心とする
%             r_obs = max(h/2, sqrt(r_bottom^2 + (h/2)^2));
% 
%         case 'custom' % カスタム
%             % --- 7. カスタム（底面・上面の全頂点指定） ---
%             % params: [3 x N] の行列 (各列が頂点の [x; y; z] 座標)
%             % ※p_center は無視し、頂点の平均から自動で中心を再計算します。
% 
%             vertices = params; 
%             p_obs = mean(vertices, 2); % 全頂点の平均を真球の中心とする
% 
%             % 中心から最も遠い頂点までの距離を半径にする
%             max_dist = 0;
%             num_vertices = size(vertices, 2);
%             for i = 1:num_vertices
%                 dist = norm(vertices(:, i) - p_obs);
%                 if dist > max_dist
%                     max_dist = dist;
%                 end
%             end
%             r_obs = max_dist;
% 
%         otherwise
%             error('未対応の形状タイプです。');
%     end
% end
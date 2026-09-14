function obs = ENVIRONMENT_OBSTACLE_ELLIPSE()
% ENVIRONMENT_OBSTACLE_ELLIPSE
% 任意の幾何形状（円柱、直方体、角錐等）とその姿勢(R_obs)から、
% 最小外接楕円体 (Bounding Ellipsoid) および外接真球 (Bounding Sphere) を一元計算する定義関数。
%
% 出力構造体フィールド (既存・新規全互換):
%   - type             : 元形状タイプ ('cylinder', 'box', 'sphere', etc.)
%   - raw_param        : 元形状のパラメータ (描画用)
%   - p_center         : 形状の中心座標 [x; y; z]
%   - R_obs            : 形状の回転姿勢行列 (3x3)
%   - d_margin         : 安全マージン [m]
%   - p_obs            : 外接真球の中心 [x; y; z] (既存CBF/球体コントローラ互換)
%   - r_obs            : 外接真球のコア半径 [m] (既存CBF/球体コントローラ互換)
%   - r_obs_margin     : マージン込み真球半径 [m]
%   - Q_obs            : 楕円体形状行列 diag([1/a^2, 1/b^2, 1/c^2]) (楕円コントローラ用)
%   - ellipsoid_radii  : 楕円体主軸半径 [a; b; c] (楕円コントローラ・描画用)

    obs = struct([]);

    %% =========================================================================
    %  1. 円柱 (Cylinder: ポール・電柱など)
    %  =========================================================================
    p_center1 = [-0.3; 7.5; 1.5]; 
    cyl_param1 = [0.5, 0.5]; % [底面半径 r, 高さ h]
    R_obs1 = eye(3);         % 鉛直 (Z軸方向)
    d_margin1 = 0.5;

    obs(1) = build_obstacle_data('cylinder', p_center1, cyl_param1, R_obs1, d_margin1);

    %% =========================================================================
    %  2. 円柱 (Cylinder: 上空 Z=15m の通過障害物)
    %  =========================================================================
    p_center2 = [0.3; 0.3; 15.0]; 
    cyl_param2 = [0.5, 1.0]; % [底面半径 r, 高さ h]
    R_obs2 = eye(3);         % 鉛直
    d_margin2 = 0.5;

    obs(2) = build_obstacle_data('cylinder', p_center2, cyl_param2, R_obs2, d_margin2);

    % %% =========================================================================
    % %  【見本3】真球 (Sphere)
    % %  =========================================================================
    % p_center3 = [-2.0; 6.0; 1.0];
    % r_sphere  = 0.5; % 半径
    % obs(3)    = build_obstacle_data('sphere', p_center3, r_sphere, eye(3), 0.3);
    % 
    % %% =========================================================================
    % %  【見本4】直方体 / 四角柱 (Box: コンテナ・ビル・壁など)
    % %  =========================================================================
    % p_center4 = [2.0; 6.0; 1.0];
    % box_param = [1.0, 0.8, 1.5]; % [dx, dy, dz]
    % % 例: Z軸周りに45度傾いた壁
    % yaw = pi/4;
    % R_box = [cos(yaw), -sin(yaw), 0; sin(yaw), cos(yaw), 0; 0, 0, 1];
    % obs(4)    = build_obstacle_data('box', p_center4, box_param, R_box, 0.4);
    % 
    % %% =========================================================================
    % %  【見本5】正三角柱 (Prism: 屋根・三角構造物など)
    % %  =========================================================================
    % p_center5   = [-1.5; 10.0; 1.0];
    % prism_param = [1.2, 2.0]; % [底面正三角形の1辺 a, 高さ h]
    % obs(5)      = build_obstacle_data('prism', p_center5, prism_param, eye(3), 0.3);
    % 
    % %% =========================================================================
    % %  【見本6】円錐 (Cone: カラーコーン・樹木など)
    % %  =========================================================================
    % p_center6  = [1.5; 10.0; 1.0];
    % cone_param = [0.6, 1.8]; % [底面半径 r, 高さ h]
    % obs(6)     = build_obstacle_data('cone', p_center6, cone_param, eye(3), 0.3);
    % 
    % %% =========================================================================
    % %  【見本7】正四角錐 (Pyramid: ピラミッド・テントなど)
    % %  =========================================================================
    % p_center7     = [0.0; 12.0; 1.0];
    % pyramid_param = [1.5, 1.2]; % [底面1辺 a, 高さ h]
    % obs(7)        = build_obstacle_data('pyramid', p_center7, pyramid_param, eye(3), 0.3);
    % 
    % %% =========================================================================
    % %  【見本8】カスタム多面体 (Custom: 歪んだ壁など)
    % %  =========================================================================
    % vertices = [
    %     -0.5,  0.5,  0.5, -0.5, -0.2,  0.2,  0.1, -0.3; 
    %     14.5, 14.5, 15.5, 15.5, 14.7, 14.7, 15.3, 15.3; 
    %      0.0,  0.0,  0.0,  0.0,  2.5,  2.5,  2.5,  2.5  
    % ];
    % obs(8) = build_obstacle_data('custom', mean(vertices, 2), vertices, eye(3), 0.3);
end

%% =========================================================================
%  サブ関数: 障害物データビルダー (楕円体 & 真球 同時包囲)
%  =========================================================================
function item = build_obstacle_data(type, p_center, raw_param, R_obs, d_margin)
    item.type       = type;
    item.raw_param  = raw_param;
    item.p_center   = p_center;
    item.R_obs      = R_obs;
    item.d_margin   = d_margin;

    % 1. 最小外接楕円体 (Bounding Ellipsoid) の主軸半径 [a; b; c] 算出
    switch lower(type)
        case 'sphere'
            r = raw_param;
            radii = [r; r; r];
            
        case 'cylinder'
            % raw_param: [底面半径 r, 高さ h] (ローカル座標 Z 軸が円柱の軸)
            r = raw_param(1);
            h = raw_param(2);
            a = r * sqrt(2);
            b = r * sqrt(2);
            c = (h / 2) * sqrt(2);
            radii = [a; b; c];
            
        case 'box'
            % raw_param: [dx, dy, dz]
            dx = raw_param(1); dy = raw_param(2); dz = raw_param(3);
            a = (dx / 2) * sqrt(3);
            b = (dy / 2) * sqrt(3);
            c = (dz / 2) * sqrt(3);
            radii = [a; b; c];
            
        case 'prism'
            % raw_param: [底面1辺 a_tri, 高さ h]
            r_tri = raw_param(1) / sqrt(3);
            h = raw_param(2);
            a = r_tri * sqrt(2);
            b = r_tri * sqrt(2);
            c = (h / 2) * sqrt(2);
            radii = [a; b; c];
            
        case 'cone'
            % raw_param: [底面半径 r, 高さ h]
            r = raw_param(1);
            h = raw_param(2);
            r_bound = max(h / 2, sqrt(r^2 + (h / 2)^2));
            radii = [r_bound; r_bound; r_bound];
            
        case 'pyramid'
            % raw_param: [底面1辺 a_p, 高さ h]
            r_bot = sqrt(2) * (raw_param(1) / 2);
            h = raw_param(2);
            r_bound = max(h / 2, sqrt(r_bot^2 + (h / 2)^2));
            radii = [r_bound; r_bound; r_bound];
            
        case 'custom'
            % raw_param: [3 x N] 頂点群
            vertices = raw_param;
            p_c = mean(vertices, 2);
            item.p_center = p_c;
            diffs = vertices - p_c;
            % 各主軸方向の最大変位
            a = max(abs(diffs(1, :))) * sqrt(3);
            b = max(abs(diffs(2, :))) * sqrt(3);
            c = max(abs(diffs(3, :))) * sqrt(3);
            radii = [max(a, 0.1); max(b, 0.1); max(c, 0.1)];
            
        otherwise
            error('未対応の形状タイプです: %s', type);
    end

    item.ellipsoid_radii = radii;
    item.Q_obs           = diag(1 ./ (radii.^2));

    % 2. 最小外接真球 (Bounding Sphere) の算出 (既存コード互換)
    item.p_obs = item.p_center;
    item.r_obs = max(radii); % 楕円体を包む最大半径
    item.r_obs_margin = item.r_obs + d_margin;
end
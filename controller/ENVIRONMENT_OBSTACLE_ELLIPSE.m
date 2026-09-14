function obs = ENVIRONMENT_OBSTACLE_ELLIPSE()
% ENVIRONMENT_OBSTACLE_ELLIPSE
% 任意の幾何形状（円柱、直方体、角錐等）とその姿勢(R_obs)から、
% 最小外接楕円体 (Bounding Ellipsoid) および外接真球 (Bounding Sphere) を一元計算する定義関数。
%
% 出力構造体フィールド:
%   - type             : 元形状タイプ ('cylinder', 'box', 'sphere', etc.)
%   - raw_param        : 元形状のパラメータ (描画用)
%   - p_center         : 形状の中心座標 [x; y; z]
%   - R_obs            : 形状の回転姿勢行列 (3x3)
%   - d_margin         : 安全マージン [m]
%   - p_obs            : 外接真球の中心 [x; y; z] (既存コード互換)
%   - r_obs            : 外接真球のコア半径 [m] (既存コード互換)
%   - r_obs_margin     : マージン込み真球半径 [m]
%   - Q_obs            : 楕円体形状行列 diag([1/a^2, 1/b^2, 1/c^2])
%   - ellipsoid_radii  : 楕円体主軸半径 [a; b; c]

    obs = [];

    %% =========================================================================
    %  アクティブ障害物 1: 円柱 (Cylinder: Y前進軌道 Y=7.5m)
    %  =========================================================================
    p_center1  = [-0.3; 7.5; 1.5]; 
    cyl_param1 = [0.5, 0.5];        % [底面半径 r, 高さ h]
    R_obs1     = eye(3);            % 鉛直 (Z軸方向)
    d_margin1  = 0.5;
    obs = [obs, build_obstacle_data('cylinder', p_center1, cyl_param1, R_obs1, d_margin1)];

    %% =========================================================================
    %  アクティブ障害物 2: 円柱 (Cylinder: Z上昇軌道 Z=15m)
    %  =========================================================================
    p_center2  = [0.3; 0.3; 15.0]; 
    cyl_param2 = [3.0, 1.0];        % [底面半径 r, 高さ h] (パンケーキ型)
    R_obs2     = eye(3);            % 鉛直
    d_margin2  = 0.5;
    obs = [obs, build_obstacle_data('cylinder', p_center2, cyl_param2, R_obs2, d_margin2)];

    % %% =========================================================================
    % %  【見本 1】真球 (Sphere)
    % %  =========================================================================
    % p_center_sp = [-2.0; 6.0; 1.0];
    % r_sphere    = 0.5;               % 半径
    % R_sp        = eye(3);
    % obs = [obs, build_obstacle_data('sphere', p_center_sp, r_sphere, R_sp, 0.3)];
    % 
    % %% =========================================================================
    % %  【見本 2】傾いた直方体 / 四角柱 (Box: ビル・コンテナ・傾斜壁)
    % %  =========================================================================
    % p_center_box = [2.0; 6.0; 1.0];
    % box_param    = [1.0, 0.8, 1.5];  % [X幅 dx, Y幅 dy, Z幅 dz]
    % % 姿勢指定: Z軸周りに 45 度 (Yaw) 回転した壁
    % yaw = deg2rad(45);
    % R_box = [cos(yaw), -sin(yaw), 0; 
    %          sin(yaw),  cos(yaw), 0; 
    %                 0,         0, 1];
    % obs = [obs, build_obstacle_data('box', p_center_box, box_param, R_box, 0.4)];
    % 
    % %% =========================================================================
    % %  【見本 3】横倒しの円柱 (Cylinder: パイプ・トンネル)
    % %  =========================================================================
    % p_center_pipe = [0.0; 10.0; 2.0];
    % pipe_param    = [0.6, 4.0];      % [半径 r=0.6, 長さ h=4.0]
    % % 姿勢指定: Y軸周りに 90 度回転させて X 軸方向に寝かせた円柱
    % pitch = deg2rad(90);
    % R_pipe = [ cos(pitch), 0, sin(pitch);
    %                     0, 1,          0;
    %           -sin(pitch), 0, cos(pitch)];
    % obs = [obs, build_obstacle_data('cylinder', p_center_pipe, pipe_param, R_pipe, 0.3)];
    % 
    % %% =========================================================================
    % %  【見本 4】正三角柱 (Prism: 屋根・三角構造物)
    % %  =========================================================================
    % p_center_prism = [-1.5; 10.0; 1.0];
    % prism_param    = [1.2, 2.0];     % [底面正三角形の1辺 a, 高さ h]
    % obs = [obs, build_obstacle_data('prism', p_center_prism, prism_param, eye(3), 0.3)];
    % 
    % %% =========================================================================
    % %  【見本 5】円錐 (Cone: カラーコーン・樹木など)
    % %  =========================================================================
    % p_center_cone = [1.5; 10.0; 1.0];
    % cone_param    = [0.6, 1.8];      % [底面半径 r, 高さ h]
    % obs = [obs, build_obstacle_data('cone', p_center_cone, cone_param, eye(3), 0.3)];
    % 
    % %% =========================================================================
    % %  【見本 6】正四角錐 (Pyramid: テント・ピラミッド構造)
    % %  =========================================================================
    % p_center_pyr = [0.0; 12.0; 1.0];
    % pyr_param    = [1.5, 1.2];       % [底面1辺 a, 高さ h]
    % obs = [obs, build_obstacle_data('pyramid', p_center_pyr, pyr_param, eye(3), 0.3)];
    % 
    % %% =========================================================================
    % %  【見本 7】直接指定の楕円体 (Ellipsoid: ドローンや荷物の安全境界そのもの)
    % %  =========================================================================
    % p_center_ell = [1.0; 8.0; 2.5];
    % ell_radii    = [2.0; 0.8; 0.5];  % [X半径 a, Y半径 b, Z半径 c]
    % obs = [obs, build_obstacle_data('ellipsoid', p_center_ell, ell_radii, eye(3), 0.3)];
    % 
    % %% =========================================================================
    % %  【見本 8】カスタム多面体 (Custom: 歪んだ岩や任意の3D頂点群)
    % %  =========================================================================
    % vertices = [
    %     -0.5,  0.5,  0.5, -0.5, -0.2,  0.2,  0.1, -0.3; % X頂点
    %     14.5, 14.5, 15.5, 15.5, 14.7, 14.7, 15.3, 15.3; % Y頂点
    %      0.0,  0.0,  0.0,  0.0,  2.5,  2.5,  2.5,  2.5  % Z頂点
    % ];
    % obs = [obs, build_obstacle_data('custom', mean(vertices, 2), vertices, eye(3), 0.3)];

end

%% =========================================================================
%  サブ関数: 障害物データビルダー (全形状対応・外接楕円体＆真球同時包囲)
%  =========================================================================
function item = build_obstacle_data(type, p_center, raw_param, R_obs, d_margin)
    item.type       = type;
    item.raw_param  = raw_param;
    item.p_center   = p_center;
    item.R_obs      = R_obs;
    item.d_margin   = d_margin;

    % 各形状の寸法から最小外接楕円体の主軸半径 [a; b; c] を幾何計算
    switch lower(type)
        case 'sphere'
            r = raw_param;
            radii = [r; r; r];
            
        case 'cylinder'
            % raw_param: [底面半径 r, 高さ h]
            r = raw_param(1);
            h = raw_param(2);
            % 円形断面を包む半径 a=r*sqrt(2), 高さ方向 c=(h/2)*sqrt(2)
            radii = [r * sqrt(2); r * sqrt(2); (h / 2) * sqrt(2)];
            
        case 'box'
            % raw_param: [dx, dy, dz]
            dx = raw_param(1); dy = raw_param(2); dz = raw_param(3);
            radii = [dx/2; dy/2; dz/2] * sqrt(3);
            
        case 'prism'
            % raw_param: [底面正三角形の1辺 a_tri, 高さ h]
            r_tri = raw_param(1) / sqrt(3);
            h = raw_param(2);
            radii = [r_tri * sqrt(2); r_tri * sqrt(2); (h / 2) * sqrt(2)];
            
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
            
        case 'ellipsoid'
            % raw_param: [a; b; c]
            radii = raw_param(:);
            
        case 'custom'
            % raw_param: [3 x N] 頂点群
            vertices = raw_param;
            p_c = mean(vertices, 2);
            item.p_center = p_c;
            diffs = vertices - p_c;
            a = max(abs(diffs(1, :))) * sqrt(3);
            b = max(abs(diffs(2, :))) * sqrt(3);
            c = max(abs(diffs(3, :))) * sqrt(3);
            radii = [max(a, 0.1); max(b, 0.1); max(c, 0.1)];
            
        otherwise
            error('未対応の形状タイプです: %s', type);
    end

    item.ellipsoid_radii = radii;
    item.Q_obs           = diag(1 ./ (radii.^2));

    % 既存コード互換（真球包囲データ）
    item.p_obs           = item.p_center;
    item.r_obs           = max(radii);
    item.r_obs_margin    = item.r_obs + d_margin;
end
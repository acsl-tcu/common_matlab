function obs = ENVIRONMENT_OBSTACLE_ELLIPSE_MOVE(t)
    % ENVIRONMENT_OBSTACLE_ELLIPSE_MOVE
    % 時間 t に応じて移動する障害物（動的障害物）のリストを生成します。
    % リプランナーの時空間最適化（Spatiotemporal SFC）テスト用です。

    if nargin < 1
        t = 0.0; % 引数なしで呼ばれた場合のフォールバック
    end

    obs = [];

    % %% =========================================================================
    % %  障害物 1: 等速直線運動 (Linear Motion)
    % %  ゆっくりと横切る障害物
    % %  =========================================================================
    % p0_1  = [0.3; 0.3; 30.0]; 
    % v_1   = [-0.0; -0.0; -0.5]; % 速度ベクトル [m/s]
    % 
    % p_center1 = p0_1 + v_1 * t;
    % cyl_param1 = [2.0, 1.0];        % [底面半径 r, 高さ h]
    % R_obs1     = eye(3);
    % d_margin1  = 0.5;
    % 
    % item1 = build_obstacle_data('cylinder', p_center1, cyl_param1, R_obs1, d_margin1);
    % item1.v_center = v_1; % 速度ベクトルを格納 (extract_obstacle_velocity用)
    % obs = [obs, item1];
    % 
    % %% =========================================================================
    % %  障害物 2: サイン波運動 (Sinusoidal Motion)
    % %  左右にフェイントをかけるように蛇行する障害物
    % %  =========================================================================
    % p0_2  = [0.3; 0.3; 15.0]; 
    % v_base_2 = [0.0; 0.0; 0.0]; 
    % amp_2 = 7.0; % 振幅
    % omega_2 = 0.075; % 角周波数
    % 
    % p_center2 = p0_2 + v_base_2 * t + [0; amp_2 * sin(omega_2 * t); 0];
    % v_inst_2  = v_base_2 + [0; amp_2 * omega_2 * cos(omega_2 * t); 0]; % 瞬時速度
    % 
    % cyl_param2 = [3.0, 1.0];
    % R_obs2     = eye(3);
    % d_margin2  = 0.5;
    % 
    % item2 = build_obstacle_data('cylinder', p_center2, cyl_param2, R_obs2, d_margin2);
    % item2.v_center = v_inst_2;
    % obs = [obs, item2];

    % %% =========================================================================
    % %  障害物 3: 円運動 (Circular Motion)
    % %  特定の領域をぐるぐると回る障害物
    % %  =========================================================================
    % p_center_orbit = [-4.5; -4.5; 15.0]; 
    % radius_3 = 2.5;
    % omega_3 = 0.8;
    % 
    % p_center3 = p_center_orbit + [radius_3 * cos(omega_3 * t); radius_3 * sin(omega_3 * t); 0];
    % v_inst_3  = [-radius_3 * omega_3 * sin(omega_3 * t); radius_3 * omega_3 * cos(omega_3 * t); 0];
    % 
    % cyl_param3 = [1.0, 1.0];
    % R_obs3     = eye(3);
    % d_margin3  = 0.5;
    % 
    % item3 = build_obstacle_data('cylinder', p_center3, cyl_param3, R_obs3, d_margin3);
    % item3.v_center = v_inst_3;
    % obs = [obs, item3];

    %% =========================================================================
    %  障害物 1: 等速直線運動 (Linear Motion)
    %  ゆっくりと横切る障害物
    %  =========================================================================
    p0_1  = [0.3; 0.3; 20.0]; 
    v_1   = [0.0; 0.0; 0]; % 速度ベクトル [m/s]

    p_center1 = p0_1 + v_1 * t;
    cyl_param1 = [2.5, 1.0];        % [底面半径 r, 高さ h]
    R_obs1     = eye(3);
    d_margin1  = 0.5;

    item1 = build_obstacle_data('cylinder', p_center1, cyl_param1, R_obs1, d_margin1);
    item1.v_center = v_1; % 速度ベクトルを格納 (extract_obstacle_velocity用)
    obs = [obs, item1];

    %% =========================================================================
    %  障害物 2: サイン波運動 (Sinusoidal Motion)
    %  左右にフェイントをかけるように蛇行する障害物
    %  =========================================================================
    p_center2 = [0; 18; 2];
    v0_2  = [0; 0; 0]; % 瞬時速度

    cyl_param2 = [3.0, 1.0];
    R_obs2     = eye(3);
    d_margin2  = 0.5;

    item2 = build_obstacle_data('cylinder', p_center2, cyl_param2, R_obs2, d_margin2);
    item2.v_center = v0_2;
    obs = [obs, item2];

    %% =========================================================================
    %  障害物 3: 円運動 (Circular Motion)
    %  特定の領域をぐるぐると回る障害物
    %  =========================================================================
    p_center3 = [0; 28; 0];
    v_3  = [0; 0; 0];

    cyl_param3 = [1.0, 1.0];
    R_obs3     = eye(3);
    d_margin3  = 0.5;

    item3 = build_obstacle_data('cylinder', p_center3, cyl_param3, R_obs3, d_margin3);
    item3.v_center = v_3;
    obs = [obs, item3];

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
    item.v_center   = [0; 0; 0]; % デフォルトの速度ベクトル

    % 各形状の寸法から最小外接楕円体の主軸半径 [a; b; c] を幾何計算
    switch lower(type)
        case 'sphere'
            r = raw_param;
            radii = [r; r; r];
            
        case 'cylinder'
            % raw_param: [底面半径 r, 高さ h]
            r = raw_param(1);
            h = raw_param(2);
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

    item.p_obs           = item.p_center;
    item.r_obs           = max(radii);
    item.r_obs_margin    = item.r_obs + d_margin;
end

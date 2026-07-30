function obs = ENVIRONMENT_OBSTACLE_ellipsoid()
% ENVIRONMENT_OBSTACLE 障害物の配置情報を一元管理する関数（全形状網羅・楕円体バウンディング版）
% 
% 制御バリア関数 (CBF) の評価を満たしつつ、無駄な干渉領域を減らすため、
% さまざまな原形をぴったり包み込む「最小包含楕円体 (Bounding Ellipsoid)」を自動計算します。

    %% =========================================================================
    %  1. 円柱 (Cylinder) -> ポールや電柱など
    %  =========================================================================
    p_center1 = [0.3; 7.5; 2.2]; 
    cyl_param = [0.5, 3.0]; % [底面半径 r, 高さ h]
    d_margin(1) = 0.5;      % 安全マージン
    
    obs(1).type = 'cylinder';
    obs(1).raw_param = cyl_param;
    obs(1).p_center = p_center1;
    obs(1).d_margin = d_margin(1);
    [obs(1).p_obs, obs(1).r_obs] = get_bounding_ellipsoid('cylinder', p_center1, cyl_param, d_margin(1)); 

    %% =========================================================================
    %  2. 真球 (Sphere) -> 球状の障害物、ドローンなど
    % =========================================================================
    p_center2 = [-2.0; 6.0; 1.0]; 
    r_sphere  = 0.5; % 単純な半径
    d_margin(2) = 0.3;

    obs(2).type = 'sphere';
    obs(2).raw_param = r_sphere;
    obs(2).p_center = p_center2;
    obs(2).d_margin = d_margin(2);
    [obs(2).p_obs, obs(2).r_obs] = get_bounding_ellipsoid('sphere', p_center2, r_sphere, d_margin(2));

    %% =========================================================================
    %  3. 直方体 / 四角柱 (Box) -> コンテナ、ビル、壁など
    % =========================================================================
    p_center3 = [2.0; 6.0; 1.0];
    box_param = [1.0, 0.8, 1.5]; % [X幅 dx, Y幅 dy, Z幅 dz]
    d_margin(3) = 0.3;

    obs(3).type = 'box';
    obs(3).raw_param = box_param;
    obs(3).p_center = p_center3;
    obs(3).d_margin = d_margin(3);
    [obs(3).p_obs, obs(3).r_obs] = get_bounding_ellipsoid('box', p_center3, box_param, d_margin(3));

    %% =========================================================================
    %  4. 正三角柱 (Prism) -> 屋根状の構造物など
    % =========================================================================
    p_center4 = [-1.5; 10.0; 1.0];
    prism_param = [1.2, 2.0]; % [底面正三角形の1辺の長さ a, 高さ h]
    d_margin(4) = 0.3;

    obs(4).type = 'prism';
    obs(4).raw_param = prism_param;
    obs(4).p_center = p_center4;
    obs(4).d_margin = d_margin(4);
    [obs(4).p_obs, obs(4).r_obs] = get_bounding_ellipsoid('prism', p_center4, prism_param, d_margin(4));

    %% =========================================================================
    %  5. 円錐 (Cone) -> カラーコーン、木（簡易表現）など
    % =========================================================================
    p_center5 = [1.5; 10.0; 1.0];
    cone_param = [0.6, 1.8]; % [底面半径 r, 高さ h]
    d_margin(5) = 0.3;

    obs(5).type = 'cone';
    obs(5).raw_param = cone_param;
    obs(5).p_center = p_center5;
    obs(5).d_margin = d_margin(5);
    [obs(5).p_obs, obs(5).r_obs] = get_bounding_ellipsoid('cone', p_center5, cone_param, d_margin(5));

    %% =========================================================================
    %  6. 正四角錐 (Pyramid) -> ピラミッド形状、テントなど
    % =========================================================================
    p_center6 = [0.0; 12.0; 1.0];
    pyramid_param = [1.5, 1.2]; % [底面の1辺の長さ a, 高さ h]
    d_margin(6) = 0.3;

    obs(6).type = 'pyramid';
    obs(6).raw_param = pyramid_param;
    obs(6).p_center = p_center6;
    obs(6).d_margin = d_margin(6);
    [obs(6).p_obs, obs(6).r_obs] = get_bounding_ellipsoid('pyramid', p_center6, pyramid_param, d_margin(6));

    %% =========================================================================
    %  7. カスタム多面体 (Custom) -> 斜めの柱、歪んだ幾何学形状など
    % =========================================================================
    vertices = [
        -0.5,  0.5,  0.5, -0.5, -0.2,  0.2,  0.1, -0.3; % X
        14.5, 14.5, 15.5, 15.5, 14.7, 14.7, 15.3, 15.3; % Y
         0.0,  0.0,  0.0,  0.0,  2.5,  2.5,  2.5,  2.5  % Z
    ];
    d_margin(7) = 0.3;

    obs(7).type = 'custom';
    obs(7).raw_param = vertices;
    obs(7).p_center = mean(vertices, 2);
    obs(7).d_margin = d_margin(7);
    [obs(7).p_obs, obs(7).r_obs] = get_bounding_ellipsoid('custom', obs(7).p_center, vertices, d_margin(7));

end

% function obs = ENVIRONMENT_OBSTACLE_ellipsoid()
% % ENVIRONMENT_OBSTACLE 障害物の配置情報を一元管理する関数（全形状網羅・楕円体バウンディング版）
% % 
% % 制御バリア関数 (CBF) の評価を満たしつつ、無駄な干渉領域を減らすため、
% % さまざまな原形をぴったり包み込む「最小包含楕円体 (Bounding Ellipsoid)」を自動計算します。
% 
%     %% =========================================================================
%     %  1. 円柱 (Cylinder) -> ポールや電柱など
%     %  =========================================================================
%     p_center1 = [0.3; 7.5; 2.2]; 
%     cyl_param = [0.5, 3.0]; % [底面半径 r, 高さ h]
%     d_margin(1) = 0.5;      % 安全マージン
% 
%     obs(1).type = 'cylinder';
%     obs(1).raw_param = cyl_param;
%     obs(1).p_center = p_center1;
%     obs(1).d_margin = d_margin(1);
%     [obs(1).p_obs, obs(1).r_obs] = get_bounding_ellipsoid('cylinder', p_center1, cyl_param, d_margin(1)); 
% 
%     %% =========================================================================
%     %  2. 真球 (Sphere) -> 球状の障害物、ドローンなど
%     % =========================================================================
%     p_center2 = [-2.0; 6.0; 1.0]; 
%     r_sphere  = 0.5; % 単純な半径
%     d_margin(2) = 0.3;
% 
%     obs(2).type = 'sphere';
%     obs(2).raw_param = r_sphere;
%     obs(2).p_center = p_center2;
%     obs(2).d_margin = d_margin(2);
%     [obs(2).p_obs, obs(2).r_obs] = get_bounding_ellipsoid('sphere', p_center2, r_sphere, d_margin(2));
% 
%     %% =========================================================================
%     %  3. 直方体 / 四角柱 (Box) -> コンテナ、ビル、壁など
%     % =========================================================================
%     p_center3 = [2.0; 6.0; 1.0];
%     box_param = [1.0, 0.8, 1.5]; % [X幅 dx, Y幅 dy, Z幅 dz]
%     d_margin(3) = 0.3;
% 
%     obs(3).type = 'box';
%     obs(3).raw_param = box_param;
%     obs(3).p_center = p_center3;
%     obs(3).d_margin = d_margin(3);
%     [obs(3).p_obs, obs(3).r_obs] = get_bounding_ellipsoid('box', p_center3, box_param, d_margin(3));
% 
%     %% =========================================================================
%     %  4. 正三角柱 (Prism) -> 屋根状の構造物など
%     % =========================================================================
%     p_center4 = [-1.5; 10.0; 1.0];
%     prism_param = [1.2, 2.0]; % [底面正三角形の1辺の長さ a, 高さ h]
%     d_margin(4) = 0.3;
% 
%     obs(4).type = 'prism';
%     obs(4).raw_param = prism_param;
%     obs(4).p_center = p_center4;
%     obs(4).d_margin = d_margin(4);
%     [obs(4).p_obs, obs(4).r_obs] = get_bounding_ellipsoid('prism', p_center4, prism_param, d_margin(4));
% 
%     %% =========================================================================
%     %  5. 円錐 (Cone) -> カラーコーン、木（簡易表現）など
%     % =========================================================================
%     p_center5 = [1.5; 10.0; 1.0];
%     cone_param = [0.6, 1.8]; % [底面半径 r, 高さ h]
%     d_margin(5) = 0.3;
% 
%     obs(5).type = 'cone';
%     obs(5).raw_param = cone_param;
%     obs(5).p_center = p_center5;
%     obs(5).d_margin = d_margin(5);
%     [obs(5).p_obs, obs(5).r_obs] = get_bounding_ellipsoid('cone', p_center5, cone_param, d_margin(5));
% 
%     %% =========================================================================
%     %  6. 正四角錐 (Pyramid) -> ピラミッド形状、テントなど
%     % =========================================================================
%     p_center6 = [0.0; 12.0; 1.0];
%     pyramid_param = [1.5, 1.2]; % [底面の1辺の長さ a, 高さ h]
%     d_margin(6) = 0.3;
% 
%     obs(6).type = 'pyramid';
%     obs(6).raw_param = pyramid_param;
%     obs(6).p_center = p_center6;
%     obs(6).d_margin = d_margin(6);
%     [obs(6).p_obs, obs(6).r_obs] = get_bounding_ellipsoid('pyramid', p_center6, pyramid_param, d_margin(6));
% 
%     %% =========================================================================
%     %  7. カスタム多面体 (Custom) -> 斜めの柱、歪んだ幾何学形状など
%     % =========================================================================
%     vertices = [
%         -0.5,  0.5,  0.5, -0.5, -0.2,  0.2,  0.1, -0.3; % X
%         14.5, 14.5, 15.5, 15.5, 14.7, 14.7, 15.3, 15.3; % Y
%          0.0,  0.0,  0.0,  0.0,  2.5,  2.5,  2.5,  2.5  % Z
%     ];
%     d_margin(7) = 0.3;
% 
%     obs(7).type = 'custom';
%     obs(7).raw_param = vertices;
%     obs(7).p_center = mean(vertices, 2);
%     obs(7).d_margin = d_margin(7);
%     [obs(7).p_obs, obs(7).r_obs] = get_bounding_ellipsoid('custom', obs(7).p_center, vertices, d_margin(7));
% 
% end
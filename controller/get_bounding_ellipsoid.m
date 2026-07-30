function [p_obs, r_obs] = get_bounding_ellipsoid(type, p_center, params, safety_buffer_width)
% GET_BOUNDING_ELLIPSOID 各種形状を密着して包み込む楕円体（Bounding Ellipsoid）の要素を計算
%
% 入力:
%   type                : 'sphere', 'cylinder', 'box', 'prism', 'cone', 'pyramid', 'custom'
%   p_center            : 形状の中心座標 [x; y; z]
%   params              : 形状に応じたパラメータ
%   safety_buffer_width : 安全マージン（全軸に均等に加算、または必要に応じ調整）
%
% 出力:
%   p_obs : 楕円体の中心座標 [x; y; z]
%   r_obs : 楕円体の各軸の半径 [rx; ry; rz] (ベクトル)

    if nargin < 4 || isempty(safety_buffer_width)
        safety_buffer_width = 0.0;
    end

    p_obs = p_center; 

    switch lower(type)
        case 'sphere'
            % params: 半径 r
            r_base = params;
            r_obs = [r_base; r_base; r_base] + safety_buffer_width;

        case 'cylinder'
            % params: [底面半径 r, 高さ h] (Z軸方向に伸びていると仮定)
            r_cyl = params(1);
            h_cyl = params(2);
            % X-Y平面は底面半径、Z方向は高さの半分をカバー
            r_obs = [r_cyl; r_cyl; h_cyl / 2] + safety_buffer_width;

        case 'box'
            % params: [X幅 dx, Y幅 dy, Z幅 dz]
            dx = params(1); dy = params(2); dz = params(3);
            r_obs = [dx / 2; dy / 2; dz / 2] + safety_buffer_width;

        case 'prism'
            % params: [底面正三角形の1辺 a, 高さ h] (Z軸方向へ延伸)
            a = params(1);
            h = params(2);
            r_tri = a / sqrt(3); % 外接円半径
            r_obs = [r_tri; r_tri; h / 2] + safety_buffer_width;

        case 'cone'
            % params: [底面半径 r, 高さ h]
            r_cone = params(1);
            h_cone = params(2);
            r_obs = [r_cone; r_cone; h_cone / 2] + safety_buffer_width;

        case 'pyramid'
            % params: [底面の1辺 a, 高さ h]
            a = params(1);
            h = params(2);
            r_bottom = (sqrt(2) * a) / 2; % 底面正方形の外接円半径
            r_obs = [r_bottom; r_bottom; h / 2] + safety_buffer_width;

        case 'custom'
            % params: [3 x N] 頂点行列
            vertices = params;
            p_obs = mean(vertices, 2);
            % 各軸における中心からの最大距離を抽出
            diffs = abs(vertices - p_obs);
            max_xyz = max(diffs, [], 2);
            r_obs = max_xyz + safety_buffer_width;

        otherwise
            error('未対応の形状タイプです。');
    end
end
function [p_obs, r_obs] = get_bounding_sphere(type, p_center, params)
% GET_BOUNDING_SPHERE 任意の形状をすっぽり覆う真球の中心と半径を計算する（拡張版）
%
% 入力:
%   type     : 'sphere', 'cylinder', 'box', 'prism', 'cone', 'pyramid', 'custom'
%   p_center : 形状の中心（または基準点）座標 [x; y; z]
%   params   : 各形状に応じたパラメータ
%
% 出力:
%   p_obs    : 真球の中心座標 [x; y; z]
%   r_obs    : 真球の半径 (スカラー)

    p_obs = p_center; 

    switch lower(type)
        case 'sphere' % 球
            r_obs = params; % params: 半径

        case 'cylinder' % 円柱
            % params: [底面半径 r, 高さ h]
            r_obs = sqrt(params(1)^2 + (params(2)/2)^2);

        case 'box' % 四角柱
            % params: [X幅dx, Y幅dy, Z幅dz]
            r_obs = sqrt((params(1)/2)^2 + (params(2)/2)^2 + (params(3)/2)^2);

        case 'prism' % 三角錐
            % params: [底面正三角形の1辺 a, 高さ h]
            r_tri = params(1) / sqrt(3);
            r_obs = sqrt(r_tri^2 + (params(2)/2)^2);

        case 'cone' % 円錐
            % --- 5. 円錐 ---
            % params: [底面半径 r, 高さ h]
            r_cone = params(1);
            h_cone = params(2);
            % 簡易的に底面中心から高さh/2を真球中心とし、頂点と底面端をカバーする
            % 頂点までの距離: h/2, 底面フチまでの距離: sqrt(r^2 + (h/2)^2)
            r_obs = max(h_cone/2, sqrt(r_cone^2 + (h_cone/2)^2));

        case 'pyramid' % 資格推
            % --- 6. 正四角錐 ---
            % params: [底面の1辺 a, 高さ h]
            a = params(1);
            h = params(2);
            r_bottom = sqrt(2) * (a / 2); % 底面の中心から角までの距離
            % 底面中心から高さh/2を真球中心とする
            r_obs = max(h/2, sqrt(r_bottom^2 + (h/2)^2));

        case 'custom' % カスタム
            % --- 7. カスタム（底面・上面の全頂点指定） ---
            % params: [3 x N] の行列 (各列が頂点の [x; y; z] 座標)
            % ※p_center は無視し、頂点の平均から自動で中心を再計算します。
            
            vertices = params; 
            p_obs = mean(vertices, 2); % 全頂点の平均を真球の中心とする
            
            % 中心から最も遠い頂点までの距離を半径にする
            max_dist = 0;
            num_vertices = size(vertices, 2);
            for i = 1:num_vertices
                dist = norm(vertices(:, i) - p_obs);
                if dist > max_dist
                    max_dist = dist;
                end
            end
            r_obs = max_dist;

        otherwise
            error('未対応の形状タイプです。');
    end
end
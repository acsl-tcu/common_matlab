classdef OBSTACLE_SHAPES
    % OBSTACLE_SHAPES  障害物の元形状と寸法の表現・検査だけを担当
    %
    % 全寸法は m。中心・回転・速度・ID は OBSTACLE_INFORMATION.m で指定。
    % 円柱などの「高さ方向」は形状のローカル +z 軸。
    % 中心を原点に配置し、上下方向は -height/2 ～ +height/2。
    % cone / pyramid は下側が底面で、上側が頂点。
    % polygon_prism の outline はローカル xy 座標で指定する。
    % 半径は半径、box の大きさは辺全体の長さ（半幅ではない）。
    %
    % 形状名           dimensions の書き方
    % ---------------------------------------------------------------
    % sphere           r                          球の半径
    % ellipsoid        [a b c]                    楕円体の3軸半径
    % box              [width depth height]       直方体の3辺
    % cylinder         [r height]                 円柱
    % capsule          [r straight_length]       円柱部分に半球2個
    % cone             [r height]                 底面は -z、頂点は +z
    % frustum          [bottom_r top_r height]    円錐台（top_r=0 も可）
    % pyramid          [base_side height]         正方形底面の角錐
    % regular_prism    [sides circumradius height] 正n角柱 (sides>=3)
    % torus            [major_r minor_r]          円環 (major_r>minor_r)
    % polygon_prism    struct('outline',Nx2,'height',h)
    %                  outline は反時計回り又は時計回りの多角形頂点。
    %                  頂点を重複して閉じない。凹多角形も指定可能。
    % polyhedron       struct('vertices',Nx3,'faces',Mx3)
    %                  ローカル座標の頂点と三角形面。閉じた表面メッシュ。
    %                  凹形状も指定可能。頂点だけから凸包に変換しない。
    %
    % 注意：ellipsoid は「元の障害物が楕円体」である場合の定義。
    %       他の形状を楕円体へ近似・包絡する処理は一切しない。
    %       距離計算・衝突判定・安全マージンもここにはない。

    methods (Static)
        function geometry = define(shape_name, dimensions)
            if ~(ischar(shape_name) || (isstring(shape_name) && isscalar(shape_name)))
                error('OBSTACLE_SHAPES:Name', '形状名は文字列で指定してください。');
            end
            geometry = struct('type', lower(char(shape_name)), 'dimensions', dimensions);
            OBSTACLE_SHAPES.validate(geometry);
        end

        function validate(geometry)
            if ~isstruct(geometry) || ~isscalar(geometry) || ...
                    ~isfield(geometry, 'type') || ~isfield(geometry, 'dimensions')
                error('OBSTACLE_SHAPES:Format', ...
                    '形状には type と dimensions の両方が必要です。');
            end
            type = geometry.type;
            dimensions = geometry.dimensions;
            if ~ischar(type) || ~isrow(type)
                error('OBSTACLE_SHAPES:Name', '形状名が不正です。');
            end

            switch type
                case 'sphere'
                    OBSTACLE_SHAPES.positive_vector(dimensions, 1, type);
                case 'ellipsoid'
                    OBSTACLE_SHAPES.positive_vector(dimensions, 3, type);
                case 'box'
                    OBSTACLE_SHAPES.positive_vector(dimensions, 3, type);
                case {'cylinder', 'cone', 'pyramid'}
                    OBSTACLE_SHAPES.positive_vector(dimensions, 2, type);
                case 'capsule'
                    dims = OBSTACLE_SHAPES.nonnegative_vector(dimensions, 2, type);
                    if dims(1) <= 0
                        error('OBSTACLE_SHAPES:Size', 'capsule の半径は正である必要があります。');
                    end
                case 'frustum'
                    dims = OBSTACLE_SHAPES.nonnegative_vector(dimensions, 3, type);
                    if dims(1) <= 0 || dims(3) <= 0
                        error('OBSTACLE_SHAPES:Size', 'frustum の底半径と高さは正です。');
                    end
                case 'regular_prism'
                    dims = OBSTACLE_SHAPES.positive_vector(dimensions, 3, type);
                    if dims(1) < 3 || dims(1) ~= fix(dims(1))
                        error('OBSTACLE_SHAPES:Size', 'regular_prism の辺数は3以上の整数です。');
                    end
                case 'torus'
                    dims = OBSTACLE_SHAPES.positive_vector(dimensions, 2, type);
                    if dims(1) <= dims(2)
                        error('OBSTACLE_SHAPES:Size', ...
                            'torus は major_r > minor_r としてください。');
                    end
                case 'polygon_prism'
                    if ~isstruct(dimensions) || ~isscalar(dimensions) || ...
                            ~isfield(dimensions,'outline') || ~isfield(dimensions,'height')
                        error('OBSTACLE_SHAPES:Polygon', ...
                            'polygon_prism は outline と height を持つ構造体です。');
                    end
                    p = dimensions.outline;
                    if ~isnumeric(p) || ~isreal(p) || size(p,2) ~= 2 || ...
                            size(p,1) < 3 || any(~isfinite(p(:)))
                        error('OBSTACLE_SHAPES:Polygon', ...
                            'outline は有限な N x 2 頂点列（N>=3）です。');
                    end
                    OBSTACLE_SHAPES.positive_vector(dimensions.height, 1, 'height');
                    OBSTACLE_SHAPES.check_polygon(double(p));
                case 'polyhedron'
                    if ~isstruct(dimensions) || ~isscalar(dimensions) || ...
                            ~isfield(dimensions,'vertices') || ~isfield(dimensions,'faces')
                        error('OBSTACLE_SHAPES:Mesh', ...
                            'polyhedron は vertices と faces を持つ構造体です。');
                    end
                    OBSTACLE_SHAPES.check_mesh(dimensions.vertices, dimensions.faces);
                otherwise
                    error('OBSTACLE_SHAPES:Unsupported', ...
                        '未対応の形状です: %s', type);
            end
        end
    end

    methods (Static, Access = private)
        function dims = positive_vector(value, n, label)
            dims = OBSTACLE_SHAPES.nonnegative_vector(value, n, label);
            if any(dims <= 0)
                error('OBSTACLE_SHAPES:Size', ...
                    '%s の寸法は正の値で指定してください。', label);
            end
        end

        function dims = nonnegative_vector(value, n, label)
            if ~isnumeric(value) || ~isreal(value) || numel(value) ~= n || ...
                    any(~isfinite(value(:))) || any(value(:) < 0)
                error('OBSTACLE_SHAPES:Size', ...
                    '%s の寸法は有限で非負の実数 %d 個で指定してください。', label, n);
            end
            dims = double(value(:));
        end

        function check_polygon(p)
            % 辺の交差と重複頂点を禁止。凹多角形は許可する。
            n = size(p,1);
            if size(unique(p,'rows'),1) ~= n
                error('OBSTACLE_SHAPES:Polygon', 'outline に重複頂点があります。');
            end
            pnext = p([2:n 1],:);
            edges = pnext - p;
            if any(sum(edges.^2,2) == 0)
                error('OBSTACLE_SHAPES:Polygon', '長さ0の辺があります。');
            end
            signed_twice_area = sum(p(:,1).*pnext(:,2) - pnext(:,1).*p(:,2));
            scale = max(max(p,[],1) - min(p,[],1));
            if abs(signed_twice_area) <= 1e-12 * max(scale^2, eps)
                error('OBSTACLE_SHAPES:Polygon', '多角形の面積が0です。');
            end
            for i = 1:n
                a = p(i,:); b = pnext(i,:);
                for j = (i+1):n
                    if j == i+1 || (i == 1 && j == n)
                        continue; % 隣り合う2辺の共有頂点は交差扱いしない。
                    end
                    if OBSTACLE_SHAPES.segments_intersect(a,b,p(j,:),pnext(j,:))
                        error('OBSTACLE_SHAPES:Polygon', ...
                            'outline の辺が自己交差しています。');
                    end
                end
            end
        end

        function yes = segments_intersect(a,b,c,d)
            % 向き判定と、一直線上の接触の両方を調べる。
            scale = max([max(abs(b-a)), max(abs(d-c)), max(abs(c-a)), 1]);
            tol = 1e-12 * scale^2;
            ab_c = OBSTACLE_SHAPES.cross2(b-a, c-a);
            ab_d = OBSTACLE_SHAPES.cross2(b-a, d-a);
            cd_a = OBSTACLE_SHAPES.cross2(d-c, a-c);
            cd_b = OBSTACLE_SHAPES.cross2(d-c, b-c);
            yes = (ab_c*ab_d < -tol && cd_a*cd_b < -tol);
            if yes, return; end
            if abs(ab_c) <= tol && OBSTACLE_SHAPES.on_segment(a,b,c,tol), yes = true; return; end
            if abs(ab_d) <= tol && OBSTACLE_SHAPES.on_segment(a,b,d,tol), yes = true; return; end
            if abs(cd_a) <= tol && OBSTACLE_SHAPES.on_segment(c,d,a,tol), yes = true; return; end
            if abs(cd_b) <= tol && OBSTACLE_SHAPES.on_segment(c,d,b,tol), yes = true; end
        end

        function z = cross2(a,b)
            z = a(1)*b(2) - a(2)*b(1);
        end

        function yes = on_segment(a,b,p,tol)
            yes = all(p >= min(a,b)-tol) && all(p <= max(a,b)+tol);
        end

        function check_mesh(vertices, faces)
            % 近似や凸包の生成はしない。ユーザー指定の三角形面を保持する。
            if ~isnumeric(vertices) || ~isreal(vertices) || ...
                    size(vertices,2) ~= 3 || size(vertices,1) < 4 || ...
                    any(~isfinite(vertices(:)))
                error('OBSTACLE_SHAPES:Mesh', 'vertices は有限な N x 3（N>=4）です。');
            end
            n = size(vertices,1);
            if size(unique(vertices,'rows'),1) ~= n
                error('OBSTACLE_SHAPES:Mesh', 'vertices に重複した頂点があります。');
            end
            if ~isnumeric(faces) || ~isreal(faces) || size(faces,2) ~= 3 || ...
                    size(faces,1) < 4 || any(~isfinite(faces(:))) || ...
                    any(faces(:) < 1) || any(faces(:) > n) || ...
                    any(faces(:) ~= fix(faces(:))) || ...
                    any(faces(:,1)==faces(:,2) | faces(:,2)==faces(:,3) | faces(:,3)==faces(:,1))
                error('OBSTACLE_SHAPES:Mesh', ...
                    'faces は有効な頂点番号からなる M x 3 の三角形リストです。');
            end
            v = double(vertices); f = double(faces);
            normals = cross(v(f(:,2),:)-v(f(:,1),:), v(f(:,3),:)-v(f(:,1),:), 2);
            if any(sum(normals.^2,2) <= 1e-24)
                error('OBSTACLE_SHAPES:Mesh', '面積0の三角形があります。');
            end
            % 各辺は表裏で必ず2回現れる。開いた/不整合なメッシュを拒否。
            directed = [f(:,[1 2]); f(:,[2 3]); f(:,[3 1])];
            undirected = sort(directed,2);
            [unique_edges,~,ic] = unique(undirected,'rows'); %#ok<ASGLU>
            edge_counts = accumarray(ic, 1);
            if any(edge_counts ~= 2)
                error('OBSTACLE_SHAPES:Mesh', ...
                    'polyhedron は穴のない閉じた三角形メッシュにしてください。');
            end
            forward = directed(:,1) == undirected(:,1);
            forward_counts = accumarray(ic, double(forward));
            if any(forward_counts ~= 1)
                error('OBSTACLE_SHAPES:Mesh', ...
                    '隣接面の向きが一致しません。面の頂点順を見直してください。');
            end
            volume6 = sum(dot(v(f(:,1),:), ...
                cross(v(f(:,2),:),v(f(:,3),:),2), 2));
            extent = max(max(v,[],1) - min(v,[],1));
            if abs(volume6) <= 1e-12 * extent^3
                error('OBSTACLE_SHAPES:Mesh', ...
                    'polyhedron の内部体積が0か、面の向きが不正です。');
            end
        end
    end
end

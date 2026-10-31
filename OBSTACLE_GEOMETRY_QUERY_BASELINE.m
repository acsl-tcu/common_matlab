classdef OBSTACLE_GEOMETRY_QUERY_BASELINE
    % OBSTACLE_GEOMETRY_QUERY_BASELINE  元形状と点のユークリッド距離を計算する共通入口。
    %
    % これはセンサ専用の「幾何問い合わせ」であり、障害物定義を変更しない。
    % 距離 = min_{x in 元障害物の占有領域} ||p-x|| >= 0 [m]。
    % p が障害物内部なら距離は 0。検知球との交差は d <= range で判定できる。
    %
    % prepare() : Flight開始時に形状を検査し、固定情報を一度だけ準備する。
    % distance(): 準備済み形状へ一律のインターフェースで距離を問い合わせる。
    %
    % 曲面は点群・三角形メッシュで近似せず解析的な原形状を使う。
    % polyhedron だけはユーザーが定義した閉三角形メッシュそのものが原形状。
    % polygon_prism は凹多角形を含む厳密な押出し領域を扱う。
    % 形状ごとの処理はこのクラスに隔離し、センサ本体に分岐を置かない。
    % 将来MEX化する場合も、このクラスの prepare/distance を置換すればよい。

    methods (Static)
        function shapes = prepare(obstacles)
            % 既存 OBSTACLE_INFORMATION の値を読み取るだけ。書き換えない。
            OBSTACLE_INFORMATION.validate_list(obstacles);
            blank = struct('id',0,'type','','dimensions',[], ...
                'center0',[0;0;0],'rotation',eye(3),'velocity',[0;0;0], ...
                'reference_time_s',0,'bounding_radius_m',NaN, ...
                'outline',[],'vertices',[],'faces',[]);
            shapes = repmat(blank,1,numel(obstacles));
            for k = 1:numel(obstacles)
                o = obstacles(k);
                type = o.geometry.type;
                d = o.geometry.dimensions;
                s = blank;
                s.id = o.id;
                s.type = type;
                s.dimensions = d;
                s.center0 = o.initial_position(:);
                s.rotation = o.rotation;
                s.velocity = o.velocity(:);
                s.reference_time_s = o.reference_time_s;
                switch type
                    case 'sphere'
                        s.bounding_radius_m = double(d(1));
                    case {'ellipsoid','box'}
                        if strcmp(type,'ellipsoid')
                            s.bounding_radius_m = max(double(d(:)));
                        else
                            s.bounding_radius_m = norm(double(d(:)))/2;
                        end
                    case {'cylinder','cone'}
                        s.bounding_radius_m = hypot(double(d(1)),double(d(2))/2);
                    case 'capsule'
                        s.bounding_radius_m = double(d(1))+double(d(2))/2;
                    case 'frustum'
                        s.bounding_radius_m = hypot(max(double(d(1:2))),double(d(3))/2);
                    case 'pyramid'
                        a=double(d(1)); h=double(d(2));
                        s.bounding_radius_m = hypot(a/sqrt(2),h/2);
                        h2=h/2; a2=a/2;
                        s.vertices = [-a2 -a2 -h2; a2 -a2 -h2; ...
                                       a2 a2 -h2; -a2 a2 -h2; 0 0 h2];
                        s.faces = [1 3 2; 1 4 3; 1 2 5; 2 3 5; 3 4 5; 4 1 5];
                    case 'regular_prism'
                        n = double(d(1)); radius=double(d(2)); h=double(d(3));
                        theta=(0:n-1)'*(2*pi/n);
                        s.outline = radius*[cos(theta),sin(theta)];
                        s.bounding_radius_m = hypot(radius,h/2);
                    case 'torus'
                        s.bounding_radius_m = double(d(1))+double(d(2));
                    case 'polygon_prism'
                        s.outline = double(d.outline);
                        radius=max(hypot(s.outline(:,1),s.outline(:,2)));
                        s.bounding_radius_m = hypot(radius,double(d.height)/2);
                    case 'polyhedron'
                        s.vertices = double(d.vertices);
                        s.faces = double(d.faces);
                        s.bounding_radius_m = max(sqrt(sum(s.vertices.^2,2)));
                    otherwise
                        error('OBSTACLE_GEOMETRY_QUERY_BASELINE:Unsupported', ...
                            '形状 %s の幾何問い合わせがありません。',type);
                end
                if ~(isfinite(s.bounding_radius_m) && s.bounding_radius_m>=0)
                    error('OBSTACLE_GEOMETRY_QUERY_BASELINE:Bound','障害物ID %d の形状境界が不正です。',o.id);
                end
                shapes(k)=s;
            end
        end

        function center = center_at(shape,time_s)
            % 静止なら不変。等速運動の定義だけ将来用に解釈できる。
            center=shape.center0+(double(time_s)-shape.reference_time_s)*shape.velocity;
        end

        function d = distance(point_world,shape,time_s)
            % 原形状の占有領域に対する非負の最短距離 [m]。
            center=OBSTACLE_GEOMETRY_QUERY_BASELINE.center_at(shape,time_s);
            if any(~isfinite(center))
                error('OBSTACLE_GEOMETRY_QUERY_BASELINE:Motion','障害物ID %d の位置が非有限です。',shape.id);
            end
            q=shape.rotation'*(double(point_world(:))-center);
            dims=shape.dimensions;
            switch shape.type
                case 'sphere'
                    d=max(0,norm(q)-double(dims(1)));
                case 'ellipsoid'
                    d=OBSTACLE_GEOMETRY_QUERY_BASELINE.ellipsoid_distance(q,double(dims(:)));
                case 'box'
                    d=norm(max(abs(q)-double(dims(:))/2,0));
                case 'cylinder'
                    outside=[hypot(q(1),q(2))-double(dims(1));abs(q(3))-double(dims(2))/2];
                    d=norm(max(outside,0));
                case 'capsule'
                    along=max(abs(q(3))-double(dims(2))/2,0);
                    d=max(0,hypot(hypot(q(1),q(2)),along)-double(dims(1)));
                case 'cone'
                    d=OBSTACLE_GEOMETRY_QUERY_BASELINE.revolved_frustum_distance(q,double(dims(1)),0,double(dims(2)));
                case 'frustum'
                    d=OBSTACLE_GEOMETRY_QUERY_BASELINE.revolved_frustum_distance(q,double(dims(1)),double(dims(2)),double(dims(3)));
                case 'pyramid'
                    d=OBSTACLE_GEOMETRY_QUERY_BASELINE.mesh_solid_distance(q,shape.vertices,shape.faces);
                case {'regular_prism','polygon_prism'}
                    if strcmp(shape.type,'regular_prism')
                        h=double(dims(3));
                    else
                        h=double(dims.height);
                    end
                    dxy=OBSTACLE_GEOMETRY_QUERY_BASELINE.polygon_distance(q(1:2),shape.outline);
                    dz=max(0,abs(q(3))-h/2);
                    d=hypot(dxy,dz);
                case 'torus'
                    radial=hypot(q(1),q(2));
                    d=max(0,hypot(radial-double(dims(1)),q(3))-double(dims(2)));
                case 'polyhedron'
                    d=OBSTACLE_GEOMETRY_QUERY_BASELINE.mesh_solid_distance(q,shape.vertices,shape.faces);
                otherwise
                    error('OBSTACLE_GEOMETRY_QUERY_BASELINE:Unsupported','未対応形状: %s',shape.type);
            end
            if ~isfinite(d) || d<0
                error('OBSTACLE_GEOMETRY_QUERY_BASELINE:Numeric','障害物ID %d の距離計算に失敗しました。',shape.id);
            end
        end
    end

    methods (Static,Access=private)
        function d=ellipsoid_distance(q,a)
            % 楕円体の最近点のラグランジュ乗数を二分法で解く。
            % 曲面を三角形や点群に近似しない。
            if sum((q./a).^2)<=1
                d=0;return
            end
            a2=a.^2;
            low=0; high=max(a2);
            for k=1:1024
                phi=sum((a.*q./(a2+high)).^2);
                if phi<=1,break;end
                high=high*2;
                if ~isfinite(high)
                    error('OBSTACLE_GEOMETRY_QUERY_BASELINE:Ellipsoid','楕円体最近点の数値範囲を超えました。');
                end
            end
            if phi>1
                error('OBSTACLE_GEOMETRY_QUERY_BASELINE:Ellipsoid','楕円体最近点を囲い込めません。');
            end
            for k=1:75
                lambda=low+0.5*(high-low);
                phi=sum((a.*q./(a2+lambda)).^2);
                if phi>1,low=lambda;else,high=lambda;end
            end
            lambda=low+0.5*(high-low);
            nearest=(a2.*q)./(a2+lambda);
            d=norm(q-nearest);
        end

        function d=revolved_frustum_distance(q,rb,rt,h)
            % 回転対称な円錐・円錐台の真の占有領域への距離。
            % (rho,z)断面の台形（円錐は三角形）に対する距離と同一。
            rho=hypot(q(1),q(2)); z=q(3);
            bottom=-h/2;top=h/2;
            if z>=bottom && z<=top
                radius=rb+(rt-rb)*(z-bottom)/h;
                if rho<=radius,d=0;return;end
            end
            p=[rho;z];
            a=[0;bottom]; b=[rb;bottom];c=[rt;top];f=[0;top];
            d=min([OBSTACLE_GEOMETRY_QUERY_BASELINE.segment_distance(p,a,b), ...
                   OBSTACLE_GEOMETRY_QUERY_BASELINE.segment_distance(p,b,c), ...
                   OBSTACLE_GEOMETRY_QUERY_BASELINE.segment_distance(p,c,f)]);
        end

        function d=polygon_distance(p,outline)
            % 任意（凹形状を含む）多角形の塗りつぶし領域への距離。
            [inside,on]=inpolygon(p(1),p(2),outline(:,1),outline(:,2));
            if inside || on,d=0;return;end
            d=inf;
            n=size(outline,1);
            for i=1:n
                a=outline(i,:)';b=outline(mod(i,n)+1,:)';
                d=min(d,OBSTACLE_GEOMETRY_QUERY_BASELINE.segment_distance(p,a,b));
            end
        end

        function d=segment_distance(p,a,b)
            ab=b-a;
            n2=dot(ab,ab);
            if n2==0
                d=norm(p-a);return
            end
            t=max(0,min(1,dot(p-a,ab)/n2));
            d=norm(p-(a+t*ab));
        end

        function d=mesh_solid_distance(p,vertices,faces)
            % メッシュそのものが原形状。三角形表面への最近点 + 内部判定。
            % 平面での凸包近似はしない。凹形状も保持。
            % 注: 元定義のメッシュは閉じた非自己交差面であることを前提とする。
            p=p(:);
            d=inf;
            solid_angle=0;
            for k=1:size(faces,1)
                a=vertices(faces(k,1),:)';
                b=vertices(faces(k,2),:)';
                c=vertices(faces(k,3),:)';
                d=min(d,OBSTACLE_GEOMETRY_QUERY_BASELINE.triangle_distance(p,a,b,c));
                if d==0,return;end
                % 有向立体角：閉曲面に対して内部なら絶対値がおよそ 4*pi。
                u=a-p;v=b-p;w=c-p;
                lu=norm(u);lv=norm(v);lw=norm(w);
                numerator=dot(u,cross(v,w));
                denominator=lu*lv*lw + dot(u,v)*lw + ...
                    dot(v,w)*lu + dot(w,u)*lv;
                solid_angle=solid_angle+2*atan2(numerator,denominator);
            end
            if abs(solid_angle)>2*pi,d=0;end
        end

        function d=triangle_distance(p,a,b,c)
            % 三角形に対する厳密な最近点（内部と3辺・3頂点の全領域）。
            ab=b-a;ac=c-a;ap=p-a;
            d1=dot(ab,ap); d2=dot(ac,ap);
            if d1<=0 && d2<=0,d=norm(ap);return;end
            bp=p-b;d3=dot(ab,bp);d4=dot(ac,bp);
            if d3>=0 && d4<=d3,d=norm(bp);return;end
            vc=d1*d4-d3*d2;
            if vc<=0 && d1>=0 && d3<=0
                v=d1/(d1-d3);d=norm(p-(a+v*ab));return
            end
            cp=p-c;d5=dot(ab,cp);d6=dot(ac,cp);
            if d6>=0 && d5<=d6,d=norm(cp);return;end
            vb=d5*d2-d1*d6;
            if vb<=0 && d2>=0 && d6<=0
                w=d2/(d2-d6);d=norm(p-(a+w*ac));return
            end
            va=d3*d6-d5*d4;
            if va<=0 && (d4-d3)>=0 && (d5-d6)>=0
                w=(d4-d3)/((d4-d3)+(d5-d6));
                d=norm(p-(b+w*(c-b)));return
            end
            den=va+vb+vc;
            if den==0,error('OBSTACLE_GEOMETRY_QUERY_BASELINE:DegenerateTriangle','零面積の面があります。');end
            v=vb/den;w=vc/den;
            d=norm(p-(a+v*ab+w*ac));
        end
    end
end

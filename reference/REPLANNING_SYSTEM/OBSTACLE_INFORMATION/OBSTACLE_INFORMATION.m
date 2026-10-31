classdef OBSTACLE_INFORMATION
    % OBSTACLE_INFORMATION  シミュレーションに存在する障害物の定義
    %
    % このファイルで扱うもの
    %   ・障害物を一つずつ定義する（必ず固有の ID を付ける）
    %   ・配置（位置、向き）と移動の情報を持つ
    %   ・Sim の LOGGER に渡せる構造体を作る
    %
    % 扱わないもの
    %   ・センサによる検知、可視性、既知障害物の管理
    %   ・障害物との距離、衝突、外接楕円体、マージン、安全証明
    %
    % 形状の寸法と入力チェックは OBSTACLE_SHAPES.m が担当する。
    % ここにセンサやトリガの処理は追加しない。

    methods (Static)
        function obstacles = definitions()
            % ===== 障害物を追加・変更する場所はここだけ =====
            %
            % 1個の書き方:
            %   OBSTACLE_INFORMATION.define(ID, '形状名', [x y z], 寸法)
            %
            % 任意の追加指定（名前/値のペア）:
            %   'name',      'North wall'    人が読むための名前。IDとは別。
            %   'yaw_deg',    30            ローカル z 軸周りの回転 [deg]
            %   'euler_deg', [r p y]        roll, pitch, yaw [deg]; Rz*Ry*Rx
            %   'rotation',   R             3x3 回転行列 (SO(3))
            %   'velocity',  [vx vy vz]    世界座標での並進速度 [m/s]
            %   'reference_time_s', t0     指定位置を与えた時刻 [s]（省略時は 0）
            %
            % 無指定なら静止、無回転。寸法・位置は省略不可。
            % 形状ごとの寸法と頂点入力例は OBSTACLE_SHAPES.m の先頭を参照。
            %
            % ID は 1, 2, 3 ... の連番でなくてもよいが、重複は不可。
            % 形状に安全マージンは含めない。
            %
            % 現行シナリオ（元ファイルと同じ位置・寸法の円柱3本）:
            % =====================================================================
% 障害物の定義方法・対応形状一覧
% =====================================================================
%
% 【基本的な書き方】
%
% OBSTACLE_INFORMATION.define(ID, '形状名', [x y z], 寸法)
%
% ID        : 障害物ごとの固有番号（正の整数、重複不可）
% 形状名    : 下記の形状名を指定
% [x y z]   : 障害物の基準位置 [m]（世界座標系）
% 寸法      : 形状ごとに異なる（下記参照）
%
% 【座標と寸法の共通ルール】
%
% ・長さと位置の単位はすべて [m]
% ・角度の単位は [deg]
% ・寸法は安全マージンを含まない元形状の寸法
% ・位置 [x y z] は形状のローカル原点を世界座標に配置する位置
% ・通常の形状は中心をローカル原点とする
% ・円柱、円錐、角柱などの高さ方向はローカル z 軸
% ・回転を指定しなければローカル軸と世界座標軸は一致
% ・各障害物の ID は重複させない
%
%
% =====================================================================
% 1. 球体 sphere
% =====================================================================
% 寸法 : r
% r : 球の半径 [m]
% 例：中心 [3, 0, 4]、半径 1.2 m の球
% OBSTACLE_INFORMATION.define(10, 'sphere', [3 0 4], 1.2)
% =====================================================================
% 2. 楕円体 ellipsoid
% =====================================================================
% 寸法 : [a b c]
% a : ローカル x 方向の半径 [m]
% b : ローカル y 方向の半径 [m]
% c : ローカル z 方向の半径 [m]
% 注意：直径ではなく半径を指定する。
%       元の障害物が楕円体の場合のみ使用する。
%       他の形状を楕円体近似するためのものではない。
% 例：中心 [4, 2, 4]、3軸半径 [2, 1, 3] m
% OBSTACLE_INFORMATION.define(11, 'ellipsoid', [4 2 4], [2 1 3])
% =====================================================================
% 3. 直方体 box
% ====================================================================
% 寸法 : [width depth height]
% width  : ローカル x 方向の全幅 [m]
% depth  : ローカル y 方向の全奥行 [m]
% height : ローカル z 方向の全高さ [m]
% 注意：半幅ではなく、各辺の全長を指定する。
% 例：中心 [8, 3, 4]、幅2 m、奥行3 m、高さ5 m
% OBSTACLE_INFORMATION.define(12, 'box', [8 3 4], [2 3 5])
% =====================================================================
% 4. 円柱 cylinder
% =====================================================================
% 寸法 : [r height]
% r      : 底面円の半径 [m]
% height : 円柱の全高さ [m]
% 円柱の中心を指定し、高さ方向はローカル z 軸。
% 上下面は中心からそれぞれ ±height/2 の位置。
% 例：中心 [0.3, 0.3, 20]、半径2.5 m、高さ1 m
% OBSTACLE_INFORMATION.define(13, 'cylinder', [0.3 0.3 20], [2.5 1])
% =====================================================================
% 5. カプセル capsule
% =====================================================================
% 寸法 : [r straight_length]
% r               : 断面円および両端半球の半径 [m]
% straight_length : 中央の円柱部分の長さ [m]
% ローカル z 軸方向に伸びるカプセル形状。
% 両端に半球が付く。
% 全長 = straight_length + 2*r
% straight_length = 0 の場合は球になる。
% 例：半径0.5 m、中央直線部分3 m、全長4 m
% OBSTACLE_INFORMATION.define(14, 'capsule', [2 5 4], [0.5 3])
% =====================================================================
% 6. 円錐 cone
% =====================================================================
% 寸法 : [r height]
% r      : 円形底面の半径 [m]
% height : 底面から頂点までの高さ [m]
% 底面はローカル -z 側、頂点は +z 側。
% 指定位置は底面と頂点の高さ方向の中間。
% 例：底面半径2 m、高さ4 m
% OBSTACLE_INFORMATION.define(15, 'cone', [3 4 5], [2 4])
% =====================================================================
% 7. 円錐台 frustum
% =====================================================================
% 寸法 : [bottom_r top_r height]
% bottom_r : 下側の円形底面の半径 [m]
% top_r    : 上側の円形上面の半径 [m]
% height   : 下底から上面までの高さ [m]
% 下底はローカル -z 側、上面は +z 側。
% top_r = 0 とすると円錐形状になる。
% 例：下底半径3 m、上面半径1 m、高さ5 m
% OBSTACLE_INFORMATION.define(16, 'frustum', [4 5 3], [3 1 5])
% =====================================================================
% 8. 四角錐 pyramid
% =====================================================================
% 寸法 : [base_side height]
% base_side : 正方形底面の一辺の長さ [m]
% height    : 底面から頂点までの高さ [m]
% 底面はローカル -z 側、頂点は +z 側。
% 底面は正方形。
% 例：底面の一辺4 m、高さ6 m
% OBSTACLE_INFORMATION.define(17, 'pyramid', [3 5 4], [4 6])
% =====================================================================
% 9. 正多角柱 regular_prism
% =====================================================================
% 寸法 : [sides circumradius height]
% sides        : 底面正多角形の辺の数（3以上の整数）
% circumradius : 底面中心から頂点までの距離 [m]
% height       : 柱体の全高さ [m]
% sides = 3 : 正三角柱
% sides = 4 : 正四角柱
% sides = 5 : 正五角柱
% sides = 6 : 正六角柱
% 注意：circumradius は外接円半径であり、
%       多角形の一辺の長さではない。
% 例：正六角柱、外接円半径2 m、高さ5 m
% OBSTACLE_INFORMATION.define(18, 'regular_prism', [1 9 3], [6 2 5])
% =====================================================================
% 10. トーラス torus
% =====================================================================
% 寸法 : [major_r minor_r]
% major_r : トーラス中心から管の中心線までの半径 [m]
% minor_r : 管そのものの断面半径 [m]
% major_r > minor_r が必要。
% ローカル xy 平面上に円環が配置される。
% 円環の中心軸はローカル z 軸。
% 円環全体の外側半径 = major_r + minor_r
% 中央の穴の内側半径 = major_r - minor_r
% 例：中心線半径2 m、管半径0.5 m
% OBSTACLE_INFORMATION.define(19, 'torus', [0 8 4], [2 0.5])
% =====================================================================
% 11. 任意多角形の柱体 polygon_prism
% =====================================================================
% 寸法 : struct('outline', vertices2D, 'height', h)
% outline : 底面多角形の頂点座標 [N x 2]
%           各行が [x y]（ローカル座標 [m]）
% height  : 柱体の全高さ [m]
% ・頂点は外周に沿った順番で指定する。
% ・時計回り、反時計回りの両方に対応。
% ・凹多角形も定義できる。
% ・辺同士の自己交差は禁止。
% ・最初の頂点を最後に重複して書かない。
% ・高さ方向はローカル z 軸。
% 例：L字型の底面、高さ3 m
% outline = [ ...
%     0 0;
%     4 0;
%     4 1;
%     1 1;
%     1 3;
%     0 3
% ];
% dims = struct('outline', outline, 'height', 3);
% OBSTACLE_INFORMATION.define(20, 'polygon_prism', [0 0 4], dims)
% =====================================================================
% 12. 任意多面体 polyhedron
% =====================================================================
% 寸法 : struct('vertices', V, 'faces', F)
% vertices : 3次元頂点座標 [N x 3]
%            各行が [x y z]（ローカル座標 [m]）
% faces    : 三角形面の頂点番号 [M x 3]
%            各行が [頂点番号1 頂点番号2 頂点番号3]
% ・任意の閉じた三角形メッシュを指定する。
% ・凹形状も定義できる。
% ・各面の頂点の並びは外向きに統一する。
% ・穴のない閉じたメッシュが必要。
% ・頂点番号は vertices の行番号を使用する。
% ・頂点だけを入力して自動的に凸包にする処理はない。
% ・頂点座標は指定位置を原点とするローカル座標。
% 例：四面体
% V = [ ...
%     0 0 0;
%     1 0 0;
%     0 1 0;
%     0 0 1
% ];
% F = [ ...
%     1 3 2;
%     1 2 4;
%     1 4 3;
%     2 3 4
% ];
% dims = struct('vertices', V, 'faces', F);
% OBSTACLE_INFORMATION.define(21, 'polyhedron', [3 4 5], dims)
% =====================================================================
% 追加指定（すべての形状で使用可能）
% =====================================================================
% 'name', '名前'
%     障害物に付ける表示・ログ用の名前。
%     IDとは別に管理する。
% 'yaw_deg', yaw
%     ローカル z 軸周りの回転角度 [deg]
% 'euler_deg', [roll pitch yaw]
%     roll  : x 軸周りの回転角度 [deg]
%     pitch : y 軸周りの回転角度 [deg]
%     yaw   : z 軸周りの回転角度 [deg]
%     回転行列は Rz(yaw)*Ry(pitch)*Rx(roll)。
% 'rotation', R
%     3x3 の回転行列。
%     正規直交かつ det(R)=+1 が必要。
% 注意：
%     yaw_deg、euler_deg、rotation は
%     同じ障害物に同時指定できない。
% 'velocity', [vx vy vz]
%     世界座標系での並進速度 [m/s]
%     現在は基本的に省略（静止障害物）。
% 'reference_time_s', t0
%     指定した位置の基準時刻 [s]。
%     省略すると t0 = 0。
% =====================================================================
% 追加指定の具体例
% =====================================================================
% 名前付きの直方体：
% OBSTACLE_INFORMATION.define(22, 'box', [2 3 4], [2 4 6], ...
%     'name', 'Building')
% 水平面内で45度回転した直方体：
% OBSTACLE_INFORMATION.define(23, 'box', [2 3 4], [2 4 6], ...
%     'yaw_deg', 45)
% 3軸回転した楕円体：
% OBSTACLE_INFORMATION.define(24, 'ellipsoid', [3 4 5], [2 1 3], ...
%     'euler_deg', [10 20 30])
% 将来の移動障害物用の定義：
% OBSTACLE_INFORMATION.define(25, 'sphere', [0 5 4], 1, ...
%     'velocity', [0.2 0 0], ...
%     'reference_time_s', 0)
% 上記の移動障害物は
%     時刻0秒で [0 5 4] に位置し、
%     世界座標の +x 方向に 0.2 m/s で移動する。
%
% 現時点では静止障害物を基本とする。
            obstacles = [ ...
                OBSTACLE_INFORMATION.define(1, 'cylinder', [0.3  0.3 20], [2.5 1], ...
                    'name', 'Upper cylinder'); ...
                OBSTACLE_INFORMATION.define(2, 'cylinder', [5    5   18], [3   1], ...
                    'name', 'Middle cylinder'); ...
                OBSTACLE_INFORMATION.define(3, 'cylinder', [0.2 25    3], [5  10], ...
                    'name', 'Tall cylinder') ...
            ];

            % 追加するときは上の配列に要素を足す。
            % 例：次の行は説明用なので動作しない（コメントのまま）。
            % OBSTACLE_INFORMATION.define(4, 'sphere', [3 0 4], 1.2)
            % OBSTACLE_INFORMATION.define(5, 'ellipsoid', [4 2 4], [2 1 3], 'yaw_deg', 35)
            % OBSTACLE_INFORMATION.define(6, 'box', [8 3 4], [2 3 5])
            % OBSTACLE_INFORMATION.define(7, 'torus', [0 8 4], [2 0.5])
            % OBSTACLE_INFORMATION.define(8, 'regular_prism', [1 9 3], [6 2 5])
            % OBSTACLE_INFORMATION.define(9, 'sphere', [0 5 4], 1, ...
            %     'velocity', [0.2 0 0]);  % 将来の移動障害物用。現在は未使用。

            % 1個でも寸法不正・ID重複などがあればエラーにする。
            OBSTACLE_INFORMATION.validate_list(obstacles);
        end

        function obstacle = define(id, shape_name, position, dimensions, varargin)
            % 位置と時間情報はここで管理。形状定義は別ファイルに委譲。
            geometry = OBSTACLE_SHAPES.define(shape_name, dimensions);
            OBSTACLE_INFORMATION.check_id(id);
            position = OBSTACLE_INFORMATION.vector3(position, 'position');

            name = sprintf('Obstacle %d', id);
            rotation = eye(3);
            velocity = zeros(3, 1);
            reference_time_s = 0;
            rotation_set = false;
            used_options = {};

            if mod(numel(varargin), 2) ~= 0
                error('OBSTACLE_INFORMATION:Options', ...
                    '追加指定は ''項目名'', 値 のペアで指定してください。');
            end
            for k = 1:2:numel(varargin)
                key = varargin{k};
                if ~(ischar(key) || (isstring(key) && isscalar(key)))
                    error('OBSTACLE_INFORMATION:Options', '項目名には文字列を使ってください。');
                end
                key = lower(char(key));
                if any(strcmp(key, used_options))
                    error('OBSTACLE_INFORMATION:DuplicateOption', ...
                        '同じ項目を繰り返し指定できません: %s', key);
                end
                used_options{end+1} = key; %#ok<AGROW>
                value = varargin{k+1};
                switch key
                    case 'name'
                        if ~(ischar(value) || (isstring(value) && isscalar(value))) || ...
                                isempty(strtrim(char(value)))
                            error('OBSTACLE_INFORMATION:Name', 'name は空でない文字列にしてください。');
                        end
                        name = char(value);
                    case {'rotation', 'yaw_deg', 'euler_deg'}
                        if rotation_set
                            error('OBSTACLE_INFORMATION:Rotation', ...
                                '回転指定は rotation / yaw_deg / euler_deg のいずれか1つです。');
                        end
                        rotation_set = true;
                        switch key
                            case 'rotation'
                                rotation = value;
                            case 'yaw_deg'
                                OBSTACLE_INFORMATION.finite_scalar(value, 'yaw_deg');
                                rotation = OBSTACLE_INFORMATION.euler_deg([0 0 value]);
                            case 'euler_deg'
                                rotation = OBSTACLE_INFORMATION.euler_deg(value);
                        end
                    case 'velocity'
                        velocity = OBSTACLE_INFORMATION.vector3(value, 'velocity');
                    case 'reference_time_s'
                        OBSTACLE_INFORMATION.finite_scalar(value, 'reference_time_s');
                        reference_time_s = double(value);
                    otherwise
                        error('OBSTACLE_INFORMATION:UnknownOption', ...
                            '未対応の追加指定です: %s', key);
                end
            end
            OBSTACLE_INFORMATION.check_rotation(rotation);

            % 形状・寸法は geometry に集約する。近似形状は作らない。
            obstacle = struct( ...
                'id', double(id), ...
                'name', name, ...
                'geometry', geometry, ...
                'initial_position', position, ...
                'position', position, ...
                'rotation', double(rotation), ...
                'velocity', velocity, ...
                'reference_time_s', reference_time_s, ...
                'motion', 'static');

            if any(velocity ~= 0)
                obstacle.motion = 'constant_velocity';
            end
        end

        function obstacles = at_time(t)
            % 定義した位置を時刻 t における位置へ変換する。
            % 現在有効な障害物はすべて静止なので、位置は変わらない。
            % 将来の等速移動用の情報はここに確保してある。
            OBSTACLE_INFORMATION.finite_scalar(t, 'time_s');
            obstacles = OBSTACLE_INFORMATION.definitions();
            for k = 1:numel(obstacles)
                o = obstacles(k);
                obstacles(k).position = o.initial_position + ...
                    (double(t) - o.reference_time_s) * o.velocity;
            end
        end

        function entry = log_frame(t)
            % LOGGER に格納するための、通常の MATLAB 構造体を返す。
            % ID、名前、元形状、寸法、位置、姿勢、速度、時刻を欠落なく含む。
            % ファイル保存や LOGGER への登録自体は行わない。
            %
            % 将来、呼出し側で例えば次のように参照結果へ格納すれば、
            % Sim の LOGGER が各時刻の reference.result とともに保存できる。
            %   agent.reference.result.obstacle_information = ...
            %       OBSTACLE_INFORMATION.log_frame(time.t);
            % ※この接続は今回変更しない。Sim 側への自動記録はまだ行われない。
            entry.time_s = double(t);
            entry.obstacles = OBSTACLE_INFORMATION.at_time(t);
        end

        function validate_list(obstacles)
            % 障害物全体の整合性。未知の情報を補完しない。
            if ~isstruct(obstacles)
                error('OBSTACLE_INFORMATION:List', '障害物は構造体配列で定義してください。');
            end
            ids = zeros(1, numel(obstacles));
            for k = 1:numel(obstacles)
                o = obstacles(k);
                needed = {'id','name','geometry','initial_position','position', ...
                    'rotation','velocity','reference_time_s','motion'};
                if ~all(isfield(o, needed))
                    error('OBSTACLE_INFORMATION:MissingField', ...
                        '障害物 %d に必須情報が欠けています。', k);
                end
                OBSTACLE_INFORMATION.check_id(o.id);
                ids(k) = o.id;
                OBSTACLE_SHAPES.validate(o.geometry);
                OBSTACLE_INFORMATION.vector3(o.initial_position, 'initial_position');
                OBSTACLE_INFORMATION.vector3(o.position, 'position');
                OBSTACLE_INFORMATION.vector3(o.velocity, 'velocity');
                OBSTACLE_INFORMATION.check_rotation(o.rotation);
                OBSTACLE_INFORMATION.finite_scalar(o.reference_time_s, 'reference_time_s');
                if ~(ischar(o.name) && ~isempty(strtrim(o.name)))
                    error('OBSTACLE_INFORMATION:Name', '障害物名が不正です。');
                end
                if strcmp(o.motion, 'static')
                    if any(o.velocity ~= 0)
                        error('OBSTACLE_INFORMATION:Motion', ...
                            '静止障害物に非ゼロ速度を指定できません。');
                    end
                elseif ~strcmp(o.motion, 'constant_velocity')
                    error('OBSTACLE_INFORMATION:Motion', ...
                        '今定義できる移動方式は static / constant_velocity のみです。');
                end
            end
            if numel(unique(ids)) ~= numel(ids)
                error('OBSTACLE_INFORMATION:DuplicateID', ...
                    '障害物IDが重複しています。各障害物に固有のIDを付けてください。');
            end
        end
    end

    methods (Static, Access = private)
        function check_id(id)
            if ~isnumeric(id) || ~isreal(id) || ~isscalar(id) || ...
                    ~isfinite(id) || id < 1 || id ~= fix(id) || id > flintmax
                error('OBSTACLE_INFORMATION:ID', 'ID は正の整数（重複不可）です。');
            end
        end

        function x = vector3(value, label)
            if ~isnumeric(value) || ~isreal(value) || numel(value) ~= 3 || ...
                    any(~isfinite(value(:)))
                error('OBSTACLE_INFORMATION:Vector', '%s は有限の [x y z] で指定してください。', label);
            end
            x = double(value(:));
        end

        function finite_scalar(value, label)
            if ~isnumeric(value) || ~isreal(value) || ~isscalar(value) || ...
                    ~isfinite(value)
                error('OBSTACLE_INFORMATION:Scalar', '%s は有限の実数1個にしてください。', label);
            end
        end

        function R = euler_deg(angles)
            if ~isnumeric(angles) || ~isreal(angles) || numel(angles) ~= 3 || ...
                    any(~isfinite(angles(:)))
                error('OBSTACLE_INFORMATION:Euler', ...
                    'euler_deg は [roll pitch yaw]（度）で指定してください。');
            end
            a = double(angles(:)) * pi / 180;
            cr = cos(a(1)); sr = sin(a(1));
            cp = cos(a(2)); sp = sin(a(2));
            cy = cos(a(3)); sy = sin(a(3));
            Rx = [1 0 0; 0 cr -sr; 0 sr cr];
            Ry = [cp 0 sp; 0 1 0; -sp 0 cp];
            Rz = [cy -sy 0; sy cy 0; 0 0 1];
            R = Rz * Ry * Rx;
        end

        function check_rotation(R)
            if ~isnumeric(R) || ~isreal(R) || ~isequal(size(R), [3 3]) || ...
                    any(~isfinite(R(:))) || ...
                    norm(double(R)' * double(R) - eye(3), 'fro') > 1e-8 || ...
                    abs(det(double(R)) - 1) > 1e-8
                error('OBSTACLE_INFORMATION:Rotation', ...
                    'rotation は正しい 3x3 SO(3) 回転行列で指定してください。');
            end
        end
    end
end

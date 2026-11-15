DRONE_OBSTACLES v1 / 日本語マニュアル（形状定義・配置・近似・地面）
================================================================

【目的】
この段階では障害物・地面の「定義」と「6方式の保守的な幾何近似」を作ります。
HLC、Waypoint、CBF、4秒先読み、障害物回避、実機向け安全保証は変更・実装しません。

【ZIP展開場所】
ZIPの中の environment/ と TEST_DRONE_OBSTACLES_TEMP.m は、
既存の commonmatlab（プロジェクトルート）直下へコピーする。
ZIP内には既存のenvironmentを上書き削除する操作は含まない。
run_flight_SimSuspendedLoad_trj_OBSTACLE_EXAMPLE.m は参考版であり、
既存runを置き換える必要はない。RUN_追加箇所_コピペ用.txt を使用できる。

【推奨配置】
commonmatlab/                         ← 既存プロジェクトのルート
  run_flight_SimSuspendedLoad_trj.m  ← 既存runを維持
  TEST_DRONE_OBSTACLES_TEMP.m        ← 一時検証用。問題なければ削除
  environment/                       ← 既存のenvironmentフォルダ
    DRONE_OBSTACLES/
      DRONE_OBSTACLES.m               ← ゲートウェイ
      primitives/                     ← 13種類の形状定義
      scenes/                         ← 配置・位置・姿勢を編集する場所
      approximations/                 ← 6方式の生成関数
      geometry/                       ← 回転、外接箱、包含判定、描画など
      ground/ground_z0.m             ← 地面 z=0

【runからの呼び出し】
% 既存の env=[]; は変更しない。下記をその直後に追加。
obstacleScene  = 'scene_gate_square';
obstacleApprox = 'multiple_ellipsoids';
groundName     = 'ground_z0';
obstacleOpts = struct();
obstacleOpts.subdivisions = [2 2 1];       % 複数球・複数楕円体
obstacleOpts.superSubdivisions = [1 1 1];  % スーパー楕円体
obstacleOpts.superExponent = 8;           % 偶数
obstacleOpts.hollowSides = 24;            % 中空円柱の分割数
obstacleOpts.polySides = 16;              % 凸多面体外接近似の方向数
worldObstacles = DRONE_OBSTACLES(obstacleScene,obstacleApprox,groundName,obstacleOpts);
% obs_plot_world(worldObstacles,true); % 必要なら形状を描画（図が開く）

重要: ここでは障害物を読み込むだけです。既存envへ上書きしません。
将来WaypointやCBFと統合するときは worldObstacles を明示的に受け渡します。
この時点では動作するドローンには障害物の影響がありません。

【シーン切替・編集】
シーンファイルを scenes/ に作成。ファイル名が関数名と一致する必要があります。
例: scene_gate_square.m を編集し、ゲートの位置・寸法・回転を変える。
runでは obstacleScene = 'scene_gate_square'; のみ選ぶ。
他: scene_empty, scene_single_sphere, scene_mixed_obstacles, scene_holes, scene_showcase_13。
場面ごとに独立した .m ファイルを置くので編集が他のシーンへ影響しません。

【座標・姿勢の約束】
位置 [x y z] [m]。姿勢 [roll pitch yaw] [deg]。
回転 R = Rz(yaw)*Ry(pitch)*Rx(roll)。形状の奥行き軸はローカル+z。
球以外にも任意姿勢の設定ができます。

【13形状を作る関数】
obs_sphere(name,position,radius,rpyDeg)
obs_ellipsoid(name,position,[a b c],rpyDeg)
obs_box(name,position,[width depth height],rpyDeg)
obs_cylinder(name,position,radius,height,rpyDeg)
obs_capsule(name,position,radius,straightLength,rpyDeg)
obs_convex_polyhedron(name,position,verticesNx3,rpyDeg)
obs_cone(name,position,radius,height,rpyDeg)
obs_triangular_pyramid(name,position,base3x2,apex1x3,rpyDeg)
obs_triangular_prism(name,position,base3x2,height,rpyDeg)
obs_hollow_square_prism(name,position,outerXY,holeXY,height,rpyDeg)
obs_hollow_cylinder(name,position,outerRadius,innerRadius,height,rpyDeg)
obs_hollow_triangular_prism(name,position,outerTriangle3x2,innerScale,height,rpyDeg)
obs_triangular_prism_mesh(name,position,prismVertices6x3xN,rpyDeg)
末尾 rpyDeg は省略可能、既定[0 0 0]。
「円柱の穴」はローカルz方向へ貫通する。穴の向きを変えるときは全体を回転する。
「三角形の穴」は相似・同心・同姿勢の三角形の貫通穴で、任意の傾いた穴ではない。
「三角柱メッシュ」は、1要素6頂点の凸三角柱をN個まとめる体積メッシュ。
STLなどの任意三角形表面メッシュのインポータではない。

【6近似方式】
'single_sphere'       全体の保証外接箱を包含する単一球。穴は完全に塞いでよい。
'single_ellipsoid'    全体の保証外接箱を包含する単一楕円体。軸ごとに半径変更可能。
'multiple_spheres'    部品ごとの外接箱を細分し、各小箱を球で包含。
'multiple_ellipsoids' 部品ごとの外接箱を細分し、各小箱を楕円体で包含。
'superellipsoid'     部品ごとの外接箱を偶数指数のスーパー楕円体で包含。
'convex_polyhedra'   角柱等は頂点の凸包を使用。曲面形状は有限個の接平面
                     で外側から囲んだ凸多面体。分割した凹形状にも対応。

いずれも原形 O を近似形状の和集合 U で包含する設計（O⊆U）。
ただし空間的な過大近似はあります。各近似は最小個数・最小体積とは限りません。
元の穴を保持することは単一球・単一楕円体では求めません。
分割型でも外接箱への膨張により開口を狭める／塞ぐ場合があります。
中空円柱では、保証外接の角柱扇形に分解してから近似する。

【方法を障害物ごとに切り替える】
obstacleOpts.perObstacle = struct();
obstacleOpts.perObstacle.large_box = 'convex_polyhedra';
obstacleOpts.perObstacle.tilted_ellipsoid = 'single_ellipsoid';
obstacleScene='scene_mixed_obstacles';
これで指定したものだけ全体方式に優先して上書き。

【地面】
'ground_z0': z=0を境界、z<0を禁止側とする無限半空間。
'none': 地面を読み込まない。
地面は球に近似せず、worldObstacles.ground に別格納。
原点の点だけの地面判定は実物体の半径を考えていない。

【出力】
worldObstacles.objects : 元形状（真の幾何）
worldObstacles.parts   : 近似の全部品（world座標）
worldObstacles.meta    : 方式、部品数、概略保証説明
worldObstacles.ground  : 地面
worldObstacles.options : 近似パラメータ

【検証について】
commonmatlab/TEST_DRONE_OBSTACLES_TEMP.m を実行する。
元形状の内側にある点をランダム・境界近傍でサンプリングして確認する。
これは数値による回帰テストで、数学的な完全証明ではない。
包含性の理由は「解析形状そのものまたは保証外接箱を覆う」という構成にある。
テストで問題なければ一時テストファイルは削除してよい。
MATLAB環境での実行はユーザー側で行う必要がある。

【既知の制約／CBF実装前に行うこと】
1) 高階Lie微分の式と計算時間は未評価。
2) 複数楕円体により球より良い開口を得られる場合はあるが、必ず優位ではない。
3) 曲面の凸多面体は接平面から構成するので、面数が増えると計算時間が増える。
4) CBFの制約構成方法、候補障害物削減方法、安全余裕は未設計。
5) シーンと近似の設定を変更しても参照生成器・HLCはまだ障害物を使わない。
6) 外接箱の細分化は計算上安全だが、分割数次第で過大な近似や非常に多い形状要素が発生する。
7) 非凸メッシュ全般、無制限の任意形状、動的障害物は未実装。

【補足：楕円体と大型箱】
単一楕円体は障害物固有の座標系において3軸が独立しているので、
細長い直方体なら単一球より有利な場合がある。ただし15m立方体なら
3軸が等しくなり、最初の保守的な外接楕円体は球とほぼ同じになる。
現在の単一楕円体は「最小体積外接楕円体」を最適化していない。
複数楕円体は元の凸部品ごとに長方形を細分し、楕円体で保証包含する方式。
近似部品数と禁止領域が大きい場合、subdivisionsを調整して比較する。

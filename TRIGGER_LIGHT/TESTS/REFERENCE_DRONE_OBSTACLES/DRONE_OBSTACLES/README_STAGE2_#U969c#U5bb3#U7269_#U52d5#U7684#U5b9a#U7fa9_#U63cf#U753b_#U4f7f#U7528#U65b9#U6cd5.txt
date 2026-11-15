DRONE_OBSTACLES Stage 2 — 日本語使用ガイド
=========================================
更新内容: 三角柱メッシュの構文バグ修正 / 動的障害物定義 / DRAW障害物表示

【0. 今回の実装範囲】
・13種類の静的障害物形状と6種類の外側近似はStage1の仕様を維持。
・三角柱メッシュ obs_triangular_prism_mesh.m の構文を修正。
・静的シーンと動的シーンを別フォルダで管理。
・動的障害物は「障害物ごと」に別々の軌道と姿勢変化を持てる。
・元形状は濃い不透明色、近似形状は薄い半透明色で重ねる。
・3D、上面XY、正面XZ、側面YZの4面静止画を出力できる。
・DRONEの動画にも障害物を重ねられる。障害物だけのMP4も作れる。
・CBF、回避、軌道再計画、動的障害物の速度予測による安全制約はまだ実装しない。
・この環境ではMATLAB本体でテストできない。実機MATLAB実行は必要。

【1. フォルダの配置】
commonmatlab/
├─ run_flight_SimSuspendedLoad_trj.m  ← 利用中のrun（差分参照）
├─ TEST_DRONE_OBSTACLES_TEMP.m       ← Stage1と同じ一時テスト
├─ TEST_DRONE_OBSTACLES_DYNAMIC_DRAW_TEMP.m ← 新たな一時テスト
├─ environment/
│  └─ DRONE_OBSTACLES/
│     ├─ DRONE_OBSTACLES.m           ← 共通ゲート
│     ├─ primitives/                 ← 13原形
│     ├─ approximations/             ← 6外側近似
│     ├─ scenes/
│     │  ├─ static/                   ← 静的配置ファイル
│     │  └─ dynamic/                  ← 動的配置ファイル＋軌道関数
│     ├─ dynamic/                    ← 各障害物の軌道を扱う共通関数
│     ├─ geometry/                   ← 座標・形状の評価関数
│     └─ ground/                     ← ground_z0.m
└─ plot/
   ├─ DRAW_DRONE_OBSTACLES.m        ← 新規DRAW本体
   ├─ obs_draw_original_shape.m     ← 元形状描画
   ├─ obs_draw_approx_parts.m       ← 近似形状描画
   ├─ plot_obstacles_4views.m       ← 3D+3断面(投影)静止画
   ├─ animate_obstacles_only.m      ← 障害物だけのアニメーション/MP4
   └─ DRAW_SUSPENDED_LOAD.m         ← 元のクラスに障害物レイヤーを追加した版

「plot/DRAW_SUSPENDED_LOAD.m」はユーザー提供の元コードの実際に動く前半を基に、
既存アニメーション内の障害物重ね描きだけを追加したものです。元ファイルをバックアップのうえ、
MATLABが実際に読み込むファイルのある場所へ上書きしてください。
which DRAW_SUSPENDED_LOAD -all で読み込み先を確かめられます。

【2. 最初に直したエラー】
obs_triangular_prism_mesh.m の struct の文字列定義が壊れていました。
正しい形式：
  o = obs_make('triangular_prism_mesh', name, position, ...
       struct('prismVertices',prismVertices), rpyDeg);
ここは6行×3列×N個の三角柱体積要素を格納します。
scene_showcase_13 の最後の障害物に対応しています。

【3. 静的障害物の配置を決める場所】
  environment/DRONE_OBSTACLES/scenes/static/scene_gate_square.m
  environment/DRONE_OBSTACLES/scenes/static/scene_showcase_13.m
などにあります。1つのファイルが1つの障害物シナリオです。
シーンを追加したい場合は scene_my_static.m を作成し、
戻り値を objects(1)=obs_box(...), objects(2)=obs_cylinder(...), ... の形式にします。

配置例：
  objects(1)=obs_box('wall_1',[2 3 2],[5 0.5 3],[0 0 25]);
  objects(2)=obs_ellipsoid('slender_1',[4 -1 2],[0.5 1.7 0.8],[0 15 50]);

[x y z] は世界座標[m]。寸法は各形状の定義ファイルの引数で指定。
姿勢[r p y]は[roll pitch yaw] [度]。
回転行列は R=Rz(yaw)*Ry(pitch)*Rx(roll)。円柱の軸はローカルZ軸。
凸多面体・三角柱メッシュの頂点は、障害物の基準位置に対するローカル座標[m]です。

【4. runから静的シーンを読み込む】
  sceneName = 'scene_gate_square';
  approxMethod = 'multiple_ellipsoids';
  chosenGround = 'ground_z0';
  obstacleOptions = struct('subdivisions',[2 2 1], ...
      'superSubdivisions',[1 1 1], 'superExponent',8, ...
      'hollowSides',24, 'polySides',16);
  worldObstacles = DRONE_OBSTACLES( ...
      sceneName,approxMethod,chosenGround,obstacleOptions);

近似を変えるには approxMethod のみ変更。6種類：
 'single_sphere' / 'single_ellipsoid' / 'multiple_spheres'
 'multiple_ellipsoids' / 'superellipsoid' / 'convex_polyhedra'
障害物ごとに変更する例：
  obstacleOptions.perObstacle.square_gate='convex_polyhedra';
地面を無効にする場合は chosenGround='none';
地面 z=0 を使用する場合は chosenGround='ground_z0';

【5. 動的障害物：静的とは別シーンにする】
動的配置の保存先:
 environment/DRONE_OBSTACLES/scenes/dynamic/scene_dynamic_independent.m
それぞれの形状は通常のobs_*と全く同じ。作った後に個別motionをつけます。

例1) 円周移動する箱:
  objects(1)=obs_box('moving_box',[0 0 2],[1.3 0.8 1.1],[10 0 20]);
  objects(1)=obs_attach_motion(objects(1), ...
     obs_motion_circle([0 1.5 2.5],[1.6 0.9],9,0,15));
    centerXYZ=[0,1.5,2.5] [m]: 軌道中心の絶対座標
    radiusXY=[rx ry] [m], 9: 一周時間[s]
    0: 開始位相[deg], 15: yaw角速度[deg/s]
  ※円の場合、obs_boxのpositionは軌道中心にはなりません。円の初期位置は上式で決まります。

例2) 自由なキーフレームを持つ楕円体:
  objects(2)=obs_ellipsoid('moving_ellipsoid',[0 -3 1.8],[.55 .35 1],[0 30 0]);
  objects(2)=obs_attach_motion(objects(2), ...
     obs_motion_waypoints([0 3 6 10], ...
       [-2 -3 1.8; -1 -2.4 2.1; 0 -3.3 1.6; 2 -3 2], ...
       [0 30 0; 0 10 30; 10 -10 70; 0 30 90], false));
  位置・姿勢: 各行が1時刻。姿勢もそれぞれ変更可能。
  PCHIPによる補間を使用（C1であり、C6は保証しません）。

例3) 位置と姿勢を自分で数式定義:
  objects(3)=obs_cone('moving_formula',[4 -2 2.3],.65,1.3,[0 0 0]);
  objects(3)=obs_attach_motion(objects(3), ...
     obs_motion_formula(@motion_demo_figure8_position,@motion_demo_figure8_rpy));
  ファイル motion_demo_figure8_position.m で位置[x y z]を、
  motion_demo_figure8_rpy.m で姿勢[roll pitch yaw]を時刻tから直接決めます。
  それぞれ別の.mファイルに保存すれば編集しやすくなります。

例4) 現在位置・姿勢から等速度移動:
  objects(1)=obs_attach_motion(objects(1), ...
     obs_motion_linear([0.2 0 0],[0 0 10],0));

各障害物の軌道は完全に独立です。シーンのいずれかの障害物にmotionを
付けない場合、その障害物は静止します。

【6. 動的シーンをrunから読み込む / 時刻の姿勢を知る】
  sceneName='scene_dynamic_independent';
  worldObstacles=DRONE_OBSTACLES(sceneName,'multiple_ellipsoids','ground_z0');
  atTime=obs_world_at_time(worldObstacles,2.5);
  atTime.objects(1).position   % 2.5秒の中心位置
  atTime.objects(1).rpyDeg     % 2.5秒のロール・ピッチ・ヨー
  atTime.parts(1)             % 2.5秒の外側近似パーツ

近似は時刻0で作成して、その後は剛体変換で位置・姿勢を更新します。
元の形状や近似の軸長を変えることはありません。
ただし現段階では動的障害物のHOCBF・時間微分や衝突予測機能はありません。
特にキーフレームでの高階微分の滑らかさと、未知の動作に対する保証は別の課題です。

【7. 静止画（新しいplotフォルダの関数）】
  plot_obstacles_4views(worldObstacles);
  plot_obstacles_4views(worldObstacles,'time',2.5);
  plot_obstacles_4views(worldObstacles,'time',2.5, ...
      'filename',fullfile('Data','Obstacle4Views.png'));

1つのFigureに「3D / XY / XZ / YZ」を同時表示します。
元形状は濃いグレー、不透明度0.88。近似形状は明るい青色、透明度0.16。
穴のある円柱の元形状は実際の環状表面を描きます。
軸は x/y/z [m]、立体の縦横比は1:1:1で表示。

単独の3D描画:
  drawer=DRAW_DRONE_OBSTACLES(worldObstacles);
  drawer.update(3.0); % 動的障害物だけ再配置（表示時刻を変更）

元形状と近似の表示を切り替える場合:
  drawer=DRAW_DRONE_OBSTACLES(worldObstacles, ...
      'showOriginal',true,'showApprox',true, ...
      'originalAlpha',.92,'approxAlpha',.13);

【8. 動画（新しいplotフォルダ）】
障害物だけで運動を確認する場合:
  animate_obstacles_only(worldObstacles,0:0.1:12);
  animate_obstacles_only(worldObstacles,0:0.1:12,'Data/ObstacleMovie.mp4');
動的な元形状と近似が一緒に移動・回転します。静的障害物はその場に残ります。
動的シーンは表示範囲を決める必要があるため、DRAW_DRONE_OBSTACLESの
'motionRange',[0 12] オプションを必要に応じて変更します。

【9. 吊り荷の動画に障害物を重ねる】
既存のDRAW_SUSPENDED_LOADを今回同梱の修正版に差し替えます。
  mov=DRAW_SUSPENDED_LOAD(logger,'target',1,'self',agent(1), ...
      'obstacles',worldObstacles);
  mov.animation(logger,'target',1,'self',agent(1));

ファイルに保存:
  mov.animation(logger,'target',1,'self',agent(1), ...
      'mp4',fullfile('Data','FlightWithObstacles.mp4'),'pause',0);

元の DRAW_SUSPENDED_LOAD のアニメーション処理に追加したのは、
  constructorでDRAW_DRONE_OBSTACLESを生成する部分と、
  各フレーム時刻t(i)で obstacle_drawer.update(t(i)) を呼ぶ部分です。
機体、吊り荷、入力、ログの表示は元コードを維持しています。

※ DRAW_DRONE_OBSTACLES は描画のためのクラスです。安全判定やCBFへ
   接続したことにはなりません。

【10. テストファイル（commonmatlab直下。合格したら削除）】
  TEST_DRONE_OBSTACLES_TEMP
  TEST_DRONE_OBSTACLES_DYNAMIC_DRAW_TEMP

最初のテストは13形状×6近似などのサンプル包含検査です。
新しいテストは、構文エラー、三角柱メッシュ、3つの独立した動的軌道、
6近似の時刻変換、4ビュー表示を確かめます。

注意：乱数で元形状の点を検査するだけでは、全空間の厳密な包含証明になりません。
Stage1の外側包含の数学的構成を維持し、数値面の検証を別途行う必要があります。
また本ファイル群はこの環境でMATLAB実行されていないため、
実際にMATLABテストに合格してから仮テストを削除してください。

【11. 本プロジェクトの位置づけ】
現在のrunでは env=[] とし、障害物は表示されるだけです。
HLC、CBF、Waypointに衝突制約をまだ渡していません。
障害物は既知であり、動的なモデルは将来の機能準備としての定義のみです。

【12. 編集しやすい日本語シーン見本】
  sceneName = 'scene_editable_layout';
  environment/DRONE_OBSTACLES/scenes/static/scene_editable_layout.m
壁・傾いた円柱・四角穴ゲートについて、名前、中心XYZ、幅や高さ、
ロール/ピッチ/ヨーをそれぞれ別変数として記載しています。
描画では各障害物の名前を表示します（showLabels=false で非表示可）。

【13. 静的障害物と動的障害物の区別】
  worldObstacles.staticIndices   % 静的オブジェクトの番号
  worldObstacles.dynamicIndices  % 動的オブジェクトの番号
  worldObstacles.sourceObjects   % 登録した元の位置・姿勢・motion
  worldObstacles.baseObjects     % 時刻0で評価した位置・姿勢

【14. 注意：静的と動的の表示範囲】
動的な等速度軌道は長い時間では遠くまで移動します。
DRAW_DRONE_OBSTACLES(world,'motionRange',[0 50]) で表示範囲を調整できます。
これは視覚的な調整であって、衝突の先読みを意味しません。

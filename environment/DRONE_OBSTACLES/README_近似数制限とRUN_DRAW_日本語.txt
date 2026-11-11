障害物近似の要素数制限 + run描画連携（差分パッチ）
====================================================

■ 前提
Stage 2のDRONE_OBSTACLES一式、DRAW専用パッチ内の
plot/DRAW_DRONE_OBSTACLES/ にあるファイルを既に導入済みであること。
DRAW_SUSPENDED_LOAD.m は変更しません。このパッチ内にも含めていません。

ユーザー確認済み：従来の13原形＋6近似など132ケース PASS、
静的DRAW・動的DRAW PASS。今回の要素数制限は新実装なので別途検証が必要です。

■ パッチで置き換える場所
・environment/DRONE_OBSTACLES/DRONE_OBSTACLES.m
・environment/DRONE_OBSTACLES/approximations/obs_approx_defaults.m
・environment/DRONE_OBSTACLES/approximations/obs_approx_multiple_spheres.m
・environment/DRONE_OBSTACLES/approximations/obs_approx_multiple_ellipsoids.m
・environment/DRONE_OBSTACLES/approximations/obs_approx_superellipsoid.m
・environment/DRONE_OBSTACLES/approximations/obs_approx_convex_polyhedra.m
・同フォルダに obs_budgeted_boxes.m, obs_budgeted_groups.m を追加
・共通フォルダ直下：run_flight_SimSuspendedLoad_trj.m は修正済みrun例。
  元のrunを念のためバックアップしてから置換してください。
  公称進行速度は以前ご指定の1.5 m/sにしています。
  既存runを直接置換する場合は、軌道速度、HLC設定、出力設定等を見比べてください。
・commonmatlab直下：TEST_DRONE_OBSTACLES_BUDGET_TEMP.m を追加。
  MATLABでPASS後は消去してかまいません。

■ 意図
障害物の形状ごとに近似方式を切り替えない。1つのシーンに対して
'single_sphere', 'single_ellipsoid', 'multiple_spheres',
'multiple_ellipsoids', 'superellipsoid', 'convex_polyhedra'
のうち、必ず1つの方式を統一して適用する。
球・楕円体の「単一」「複数」は別の比較方法であり、
複数方式でも結果が1個になることは正常。

■ runの設定
sceneName='scene_gate_square';
approxMethod='multiple_ellipsoids';  % 二次式をそろえる、当面の基準方式
chosenGround='ground_z0';
obstacleOptions=struct();
obstacleOptions.maxPartsPerObstacle=6;  % 1障害物最大6部品
obstacleOptions.maxTotalParts=36;      % 全シーン合計最大36部品
obstacleOptions.targetPartSize=3.0;   % 分割目安[m]
obstacleOptions.superExponent=8;      % スーパー楕円体の場合のみ使用
obstacleOptions.hollowSides=16;       % 穴円柱の原形分割の数
obstacleOptions.polySides=8;          % 外接凸多面体の平面サンプリング
worldObstacles=DRONE_OBSTACLES(sceneName,approxMethod,chosenGround,obstacleOptions);

3D＋XY/XZ/YZ（障害物シーン）：
plot_obstacles_4views(worldObstacles,'time',10, ...
 'filename','Data/ObstacleScene_4Views.png');

3D＋XY/XZ/YZ（機体・吊り荷の軌跡付き）：
plot_flight_obstacles_4views(logger,worldObstacles,'time',10, ...
 'filename','Data/Flight_4Views.png');

機体・吊り荷＋障害物の動画：
animate_suspended_with_obstacles(logger,agent(1),worldObstacles, ...
 'target',1,'fps',20,'skip',8,'pause',0, ...
 'mp4','Data/FlightWithObstacles.mp4');

runでは3つとも飛行の後処理として呼びます。飛行中のHLCには負荷をかけません。
動的障害物も元形状と近似を同じ剛体変換で動かします。
env=[] は維持しており、現状は回避制御・衝突判定を行いません。

■ 上限に達した場合
部品を捨てずに連続する元形状の部品を統合し、その集合の外側を
包含する球・楕円体・スーパー楕円体・凸多面体を作ります。
厳密に同じ形状を保つことより「取り逃さない」ことを優先。
ただし統合が大きいと、穴や通路を塞ぐことがあります。
特に直方体15m級と穴付き形状は、表示して保守性を評価してください。
多面体モードは元部品ごとに外接の支持面から生成します。
生成済みメッシュは静的なら開始時に1回、動的なら運動に応じた剛体移動のみです。

■ 注意：要素数は単独で最適化する指標ではありません
指定した6/36という値は研究で検証した最適値ではなく、初期の計算予算。
4/24、6/36、8/48などを比較し、
A) 1回の近似評価の時間、B) 将来の高次CBFの微分時間、
C) 近似が占める余剰体積・通路幅、D) 制御周期25ms、を測る。
二次式の楕円体は高次Lie微分に向くため基準方式に採用。
箱の単一楕円体による過剰保守性にはスーパー楕円体(p=8)が有力。
ただし指数8の高次微分とスケーリングは別途評価が必要。
形状ごとに方式を混ぜることはしません。

■ 新しいテスト
MATLABで TEST_DRONE_OBSTACLES_BUDGET_TEMP を実行。
元形状のサンプル点と頂点の包含、タイプ統一、総部品数、
障害物ごとの最大部品数、動的姿勢更新を確認します。
全ケースPASSしたらテストファイルを削除して問題ありません。
ランダム点試験だけでは全連続領域の厳密な包含証明にはなりません。
旧TEST_DRONE_OBSTACLES_TEMP.mの最後には perObstacle override の
成功を期待する検査があるため、今回は設計上意図的に失敗します。
以前の旧テストは今回の形式には対応していません。

■ 先行研究（設計思想の参考。要素数6/36は文献値ではない）
Potdar, de Croon, Alonso-Mora (2020), Online trajectory planning and control
of a MAV payload system in dynamic environments, Autonomous Robots.
吊り荷付きMAVと外接楕円体、計算効率および障害物バッファ。
https://link.springer.com/article/10.1007/s10514-020-09919-8

Rimon & Boyd (1997), Obstacle Collision Detection Using Best Ellipsoid Fit.
最小体積外接楕円体などのオフライン幾何簡約。
https://web.stanford.edu/~boyd/papers/dist-comp.html

Geometry-Aware Control Barrier Functions for Collision Avoidance via
Bernstein Polynomial Approximations, ICRA 2026.
過保守な球・スーパー楕円体と多数要素の計算コストのトレードオフ。
https://ieeexplore.ieee.org/abstract/document/11696181

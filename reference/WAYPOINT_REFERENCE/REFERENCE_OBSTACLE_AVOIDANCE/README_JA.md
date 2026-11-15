# 障害物回避トリガ・模擬センサ（4種類の元近似に対応）

## 重要な仕様

障害物の幾何定義は **DRONE_OBSTACLES が生成した `world.parts` のみ** が元です。センサとトリガは寸法・形状を作り直しません。`parts(j).kind` が `sphere` なら `params.radius`、`ellipsoid` なら `params.axes` と `R` を直接使用します。

- `single_sphere` と `multiple_spheres`: どちらも **球の式** を使用します。
- `single_ellipsoid` と `multiple_ellipsoids`: どちらも **回転楕円体の式** を使用します。
- single/multiple は元生成器の1物体あたりのパーツ数の違いです。センサの幾何式の分岐には使いません。`world.approximation` と `parts.kind` が矛盾したらエラーにします。
- 1つの元障害物のパーツが1つでも検知球と交差したら、**その障害物のすべてのパーツ**を元データのまま返します。
- センサ中心はシミュレーションの**機体真値位置**、デフォルト検知半径は20 m、外部変更可能。保持履歴はありません。
- システム近似 C（9球索）・B（カプセル索）・D（楕円体索）のうち、**1回の制御周期に1方式のみ実行**します。
- 推定吊り荷質量は毎回EKF `state.mL`を使用し、プラントの真値質量を予測に渡しません。

## 簡単な使い方

```matlab
% 元DRONE_OBSTACLESへのパス追加とSETUP...を行った後
method='multiple_ellipsoids'; % 4種類のどれでも可
options=struct('maxPartsPerObstacle',6,'maxTotalParts',36,'targetPartSize',3);
world=DRONE_OBSTACLES('scene_mixed_obstacles',method,'ground_z0',options);
ra_obstacle_inspect_definition(world,true); % 元定義の確認・変更しない
sensorCfg=ra_sim_sensor_config(20);          % 球/楕円を指定しない
[observed, report]=ra_sim_sensor_detect(world,agent.plant.state.p,sensorCfg);
P=reference_avoidance_prepare_observed(agent,'C',observed); % C/B/Dのみ選択
disp(report);disp(P.obstacleKind);disp(P.known.axes);disp(P.known.R);
```

## フォルダ構成

`reference/WAYPOINT_REFERENCE/REFERENCE_OBSTACLE_AVOIDANCE/` の下に `SYSTEM_APPROXIMATION/{COMMON,MODEL_C_...,MODEL_B_...,MODEL_D_...}`、`SIMULATED_SENSOR`、`TRIGGER/{ENTRY_AND_OBSERVATION,FOUR_SECOND_PREDICTION,GEOMETRY_SCREENING,MEX_ACCELERATION}`を配置しています。テストは `common_matlab/REFERENCE_OBSTACLE_AVOIDANCE_TESTS/` 内に機能ごとに分けています。

## 4方式の元の生成方法

| 元生成器の指定 | 受信するparts.kind | 元の形状情報 |
|---|---|---|
| single_sphere | sphere | center, params.radius |
| multiple_spheres | sphere | 複数のcenter, params.radius, obstacleName |
| single_ellipsoid | ellipsoid | center, params.axes, R |
| multiple_ellipsoids | ellipsoid | 複数のcenter, params.axes, R, obstacleName |

同一シーン内は球か楕円体のどちらか一方です。`single_*` の名称はデータ整合性検査に使いますが、検知距離やトリガの形状関数は `parts.kind` で判定します。

## MEX

C方式のMEXソースと準備・検証・バイナリは `TRIGGER/MEX_ACCELERATION` 配下です。キャッシュ準備時に生成物がなければ事前コンパイルを試み、数値一致検証を通れば使用します。実Flightの25ms周期中は再コンパイルしません。Cコンパイラがない場合はMATLAB版に戻します。

## 制限

実Flightへの回避入力接続ではなく**監視用統合run**です。動的障害物を現在位置で検知することはできますが、既存トリガは動的障害物の4秒未来モデルを持たず、該当時はUNKNOWNとなります。名目軌跡での非接触判定は実機の厳密安全保証を意味しません。

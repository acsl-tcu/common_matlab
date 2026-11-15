# システム近似3方式と共通の形状定義

## 定義ファイルと役割

`COMMON/wp_system_geometry_config.m`は元の暫定設定の唯一の入口。`COMMON/wp_system_shape_at_pose.m`は機体中心 `pDrone(3x1)` と吊り荷中心 `pLoad(3x1)` から、**選択された**方法の幾何要素を生成します。各方式のshape/collisionはそれぞれのフォルダにあります。

| 方式 | フォルダ | 索の外接形状 | 要素数（現設定） |
|---|---|---|---|
| C | `MODEL_C_MULTIPLE_SPHERES` | 8区間に沿う9球 | 機体1＋索9＋荷1=11 |
| B | `MODEL_B_SPHERE_CAPSULE` | 線分を半径付きカプセル化 | 3 |
| D | `MODEL_D_SPHERE_CABLE_ELLIPSOID` | 索カプセルを外接する楕円体 | 3 |

Cの索近似球半径は `hypot(L/(2*N), cableRadius+safetyMargin)`。Bの索カプセル半径は `cableRadius+safetyMargin`。Dは半長 `h=L/2`, 半径 `r=cableRadius+safetyMargin`, `eta>0` に対して、長半軸 `sqrt((1+eta)*h^2+(1+1/eta)*r^2)`、短半軸 `sqrt((1+1/eta)*r^2)`。すべて元のコードの数式です。地面は下側禁止の平面として処理します。

## 暫定値（現在の配布コード）

| 項目 | 設定 | 単位・意味 |
|---|---:|---|
| `droneRadius` | 0.25 | m・機体球 |
| `loadRadius` | 0.16 | m・吊り荷球 |
| `cableRadius` | 0.015 | m・索チューブ |
| `safetyMargin` | 0.05 | m・原本にあった追加寸法、根拠の最終確定は未実施 |
| `cableSegments` | 8 | C方式の分割数（球9個） |
| `ellipsoidEta` | 0.25 | D方式の外接楕円体パラメータ |
| `cableLength` | 2.0 | m・プレビュー用暫定値。**トリガではモデル索長に置換** |
| `cableLengthTol` | 0.15 | m・プレビューの整合性許容値 |
| `checkGround` | true | 地面判定が既知の場合に実施 |

`g=wp_system_geometry_config('C');` が初期値。各方式の必要な数値を与えるには `g=wp_system_geometry_config('C',struct('droneRadius',0.24,'cableSegments',8));` のように**変更した理由が追跡可能な形で**明示してください（数値は構文例であり推奨値ではありません）。既存CBFの寸法に正式に置換する際は3方式とCBFの幾何を同一基準で照合してください。

```matlab
g=wp_system_geometry_config('C');
pD=[0;0;g.cableLength+1];pL=[0;0;1];
shape=wp_system_shape_at_pose(g,pD,pL);
numel(shape.items)  % 現暫定設定では11
```

**注意**：`wp_system_shape_at_pose`の長さチェックはプレビュー用`g.cableLength`に依存します。トリガに使う設定 `cfg.cableLength` は現在の `agent.parameter.get("cableL")` 等から取得します。予測器が使う最新の吊り荷質量はEKFの `mL` のみ。形状の`isProvisional=true`/`cbfMatched=false`はまだ外さないでください。

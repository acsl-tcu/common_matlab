# 模擬センサ — 現在時刻の観測範囲足切り

本センサは **球または楕円体の既存近似パーツを使う検知範囲判定だけ** を行います。CBF、制御、EKF、距離記憶、遮蔽推定、未来予測は行いません。

## 幾何・観測契約

- センサ球の中心: **シミュレータの機体真値位置** `pDroneTrue=[x;y;z]`（m）。EKF推定値 `p` や参照位置を渡さない。
- デフォルト半径: **20 m**。`ra_sim_sensor_config(cover,radiusM)` で自由に変更可能。
- センサの視野は全方向の球。取り付けオフセット0、遮蔽判定なし、見える数の上限なし。
- 1つの呼出しで `multiple_spheres` **または** `multiple_ellipsoids` のどちらか一方。物体ごとに方式を変えない。
- 近似パーツの**立体形状とセンサ球が境界でも交差したら検知**。中心点だけの判定にしない。
- 同一 `obstacleName` の**どれか1パーツが交差すれば、その障害物の全パーツと元の `objects` の物体情報を返す**。その物体の別パーツが20 mの外にあっても省かない。
- 1周期ごとに、現在の `worldNow` だけから結果を返す。以前見えていた障害物でも、今範囲外なら出力しない。履歴・保持・失探時の補完は実装しない。
- `worldNow.parts` は**現在時刻で更新済みの**近似形状。動的物体なら、呼出し元で `obs_world_at_time(world,t)` を済ませて渡す。
- `observed.parts` と `observed.objects` は検知された対象のみ。センサ以外の`baseParts`/`sourceObjects`に含まれる未検知物体はコピーしない。
- `ground` は障害物センサの検知対象とせず、独立した環境拘束としてそのまま渡す。

球と検知球の交差は `||c-pDroneTrue|| <= sensorRadius + sphereRadius`。楕円体は外接球による早期除外の後、回転楕円体立体への最短距離 `d(pDroneTrue,E) <= sensorRadius` を判定します。外接球が重なるだけでは検知しません。浮動小数点の境界では `64*eps(長さスケール)` 程度を接触側として扱います。

## 使い方 — MATLABからZIPを展開しない

展開済みの `common_matlab` で、`SETUP_REFERENCE_OBSTACLE_AVOIDANCE()` を一度呼びます。実環境での `reference/WAYPOINT_REFERENCE/...` 配置でも、モジュールの `.m` がMATLABパスに載っていれば構いません。

```matlab
sensorCfg = ra_sim_sensor_config('multiple_ellipsoids'); % 半径20m
% sensorCfg = ra_sim_sensor_config('multiple_spheres', 30); % 30mに変更

% 使用する物体近似は既存DRONE_OBSTACLESから1方式だけ生成
world0 = DRONE_OBSTACLES('scene_mixed_obstacles', ...
                        sensorCfg.approximation, 'ground_z0');

% 動的物体のあるシーンなら現在時刻の剛体配置を更新
worldNow = obs_world_at_time(world0, flightTime);

% 必ずシミュレータの機体の真値位置を使用：EKFではない
pDroneTrue = [trueX; trueY; trueZ];  % 実装側の真値を代入
[observedWorld, sensorReport] = ra_sim_sensor_detect( ...
                    worldNow, pDroneTrue, sensorCfg);

% 選択したシステム近似はC/B/Dの1方式のみ
mode='C';
Q = reference_avoidance_prepare_observed( ...
        agent, mode, sensorCfg.approximation, observedWorld);
```

`trueX/Y/Z` は説明用の記号であり、実行時には **既存プラントの真値位置** を明示して渡します。推定吊り荷質量はセンサで使わず、従来のトリガがEKF `state.mL` を取得します。

### 注意: 毎周期の負荷

現在の `ra_sim_sensor_detect` は毎回検知集合を算出する無状態関数です。`reference_avoidance_prepare_observed` はMEX準備や幾何キャッシュ再構築を伴うため、**観測された物体／近似形状が変化しない周期には毎回呼び直さない**ことを推奨します。今回、観測集合の差分キャッシュ管理や実Flightへの自動挿入はまだ行っていません。

動的障害物を検知すること自体はできますが、従来のトリガは動的予測を未実装と記録するので、将来の動的衝突予測の安全保証にはなりません。

## 検証

`REFERENCE_OBSTACLE_AVOIDANCE_TESTS/07_SIMULATED_SENSOR/` の `RUN_RA_SIMULATED_SENSOR_TESTS()` が、境界接触、外接球だけでは誤検知する細長い楕円体、回転、元物体の全パーツ返却、半径変更、無履歴、空シーン、トリガのキャッシュ受け渡し、36パーツ計測を検査します。元の `DRONE_OBSTACLES` がMATLABパス上にある場合は、実ジェネレータの2近似形式も追加で試験します。欠けている場合は `SKIPPED` と表示し、実行していないのにPASSと表記しません。

今回の配送時点ではMATLAB実行環境がないため、テストを「PASS済み」とは主張しません。テスト結果は `REFERENCE_OBSTACLE_AVOIDANCE_TESTS/RESULTS/RA_SENSOR_*.zip` に日時付きで保存されます。

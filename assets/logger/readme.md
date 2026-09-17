LOGGER (assets/logger)
======================

概要
----
LOGGER はシミュレーション/実験のログ収集・可視化・保存/復元・再生を行うためのクラス群です。
本体 (LOGGER.m) は後方互換 API を保ち、機能ごとに分割した C_Logger_* を内部で利用します。

ファイル構成
------------
- LOGGER.m: 互換 API の薄いラッパー
- C_Logger_Display.m: 表示（console）関連
- C_Logger_Plot.m: プロット関連
- C_Logger_Query.m: データ取得/抽出関連
- C_Logger_Storage.m: 保存/復元関連
- C_Logger_Replay.m: overwrite/replay 関連

機能一覧（現行 LOGGER の機能と対応）
-----------------------------------
- 初期化/復元
  - 通常初期化
  - 保存済みログ（単一 .mat / 分割保存ディレクトリ）からの復元
- ログ取得本体
  - 時刻/フェーズ/agent 結果/inner_input/追加 items の記録
  - state の deep copy（state_copy）
- 表示用データの収集/表示
  - display_func を用いた文字列/数値生成
  - display_on によるコンソール出力
- 保存/復元
  - 単一ファイル保存
  - 分割保存（sensor/estimator/reference/...）
- 上書き/再生（replay）
  - overwrite でログから agent へ復元
- データ取得 API
  - data / data_org による時系列抽出
  - time/phase 範囲指定
  - attribute 指定（s/e/r/p/i）
- 変数名解決
  - full_var_name（p/q/v/w/input 等の短縮表現）
  - p1, p1:3 などの次元抽出
- プロット機能
  - 時間応答/位相/3D
  - フェーズ背景色
  - 凡例生成

機能とクラスの対応表
--------------------
- 初期化/復元: LOGGER, C_Logger_Storage
- ログ取得本体: LOGGER
- 表示: C_Logger_Display
- 保存/復元: C_Logger_Storage
- 上書き/再生: C_Logger_Replay
- データ取得: C_Logger_Query
- プロット: C_Logger_Plot
- 変数名解決/抽出: C_Logger_Query

基本的な使い方
--------------

1) 生成
```
logger = LOGGER(1, size(ts:dt:te, 2), 0, [], []);
```
引数の意味:
- 1: ログ対象の agent インデックス（例: 1:3）
- size(ts:dt:te, 2): ログの確保長（サンプル数）
- 0: 実験フラグ（0=シミュレーション, 1=実験）
- []: 追加で保存する items（agent 外のデータ）
- []: agent_items（sensor/estimator/reference 以外の agent 内データ）

2) ログ収集
```
logger.logging(time, 'f', agent);
```
引数の意味:
- time: TIME クラスのインスタンス
- 'f': フライトフェーズ（a/t/f/l などの1文字）
- agent: DRONE クラス配列（ログ対象）

3) 保存
```
logger.save("example"); % Data/Sim_data または Data/Exp_data に保存
```
引数の意味:
- "example": 保存ファイルのプレフィックス名

オプション:
```
logger.save("example", "range", 1:1000, "separate", true);
```
引数の意味:
- "range": 保存するログのインデックス範囲
- "separate": 分割保存（true/false）

4) 復元
```
logger = LOGGER("Data/Sim_data/Log(...).mat");
% または分割保存ディレクトリ
logger = LOGGER("Data/Sim_data/Log(...)");
```
引数の意味:
- "Data/Sim_data/Log(...).mat": 単一ファイル保存のログ
- "Data/Sim_data/Log(...)": 分割保存ディレクトリ

表示（console）
--------------
display_func で表示文字列/数値を組み立て、display_on で出力します。

```
logger.display_func = @(agent, time) sprintf("t=%.2f", time.t);
logger.display_on = true;
```

プロット
--------
```
logger.plot({1, "p", "er"}, "ax", app.UIAxes, "phase", "tfl");
```
引数の意味:
- {1, "p", "er"}: list の1要素（agentId=1, variable="p", attribute="er"）
- "ax", app.UIAxes: 描画先の axes
- "phase", "tfl": フェーズ指定（例: takeoff/flight/landing を含む範囲）

加工データのプロット:
```
raw = logger.data(1, "p", "e");
custom = raw;
custom(:, 3) = abs(custom(:, 3));
logger.plot({1, "p", "e", custom});
```
引数の意味:
- {1, "p", "e", custom}: 4番目に加工済みデータを渡す
- custom の行数は logger の時系列長と一致させる

位相/3D用の加工データ:
```
customPhase = struct("x", x, "y", y);
logger.plot({1, "p1-p2", "e", customPhase});
```

list の要素は {agentId, variable, attribute} です。
- agentId: 対象機体（例: 1, 1:3）
- variable: p, q, v, w, input, 例 "p1-p2"
- attribute: s（sensor）, e（estimator）, r（reference）, p（plant）

主なオプション（name-value）:
- "time": 時間範囲 [t0 t1]
- "fig_num": 図番号
- "row_col": サブプロット行列 [row col]
- "color": フェーズ着色の有無 (0/1)
- "hold": hold の有無 (0/1)
- "ax": 描画先 axes
- "xrange"/"yrange"/"zrange": 軸範囲
- "phase": フェーズ抽出 (例 "tfl")
- "FontSize": フォントサイズ
- "Linewidth": 線幅

list の要素は {agentId, variable, attribute} です。
- variable: p, q, v, w, input, 例 "p1-p2"
- attribute: s（sensor）, e（estimator）, r（reference）, p（plant）

データ取得
----------
```
pos = logger.data(1, "p", "e", "ranget", [0, 10]);
ref = logger.data(1, "state.xd", "r");
```
引数の意味:
- 1: agentId
- "p": variable（短縮表現）
- "e": attribute（estimator）
- "ranget", [0, 10]: 時間範囲
- "state.xd": 参照のフルパス指定（reference.result.state.xd 相当）
- "r": attribute（reference）

主なオプション（name-value）:
- "ranget": 時間範囲 [t0 t1]
- "phase": フェーズ抽出 (例 "tfl")

補足:
- logger.data(0, "item_name", []) で items を取得可能
- logger.data("t", [], []) で時間軸を取得可能

overwrite / replay
------------------
```
logger.overwrite("estimator", time.t, agent, 1);
```
引数の意味:
- "estimator": 上書き対象（sensor/estimator/reference/controller/plant）
- time.t: 現在時刻
- agent: DRONE クラス配列
- 1: 対象 agent のインデックス

display / show_display
----------------------
```
logger.display_func = @(agent, time) sprintf("t=%.2f", time.t);
logger.display_on = true;
logger.show_display("k", 10);
```
引数の意味:
- display_func: 文字列/数値を作る関数（agent, time を受け取る）
- display_on: logging 時に自動表示するか
- "k", 10: 表示するログのインデックス

補足:
- logger.capture_display(time, agent) で明示的に display を作成可能
- logger.Data.display に表示内容が保存される

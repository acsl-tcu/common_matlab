V7：最新EKF値を使う「PV履歴＋軽量Governor」予測と、選択1方式の幾何判定
===========================================================================

【今回やったこと】
1. 既存の位置だけの1秒逆算を置き換える候補として、EKFの吊り荷位置・速度を利用した
   2/6次の元HLC誤差モード初期化を追加。復元行列はコントローラゲイン／履歴格子が
   変わったときだけ外部で計算。毎25ms周期に「逆行列」を再計算しない。
2. 4秒間、25msごとに誤差モードと元の6段Governorを軽量更新。
   未来入力を固定せず、未来のプラント・HLC・実参照軌跡を読み出さない。
3. 最新の agent.estimator.result.state.mL を毎周期取り込む。parameter.loadmassの
   古い値を使ったり、真値を要求したり、質量誤差の長い試験を走らせたりしない。
4. 観測済み障害物の外接形状キャッシュと、選択されたONE C/B/D × ONE球/楕円体の
   V4カーネルにつなぐ。1tickに6方式を実行しない。
5. 障害物なし＋地面無効の場合、4秒予測そのものを省略。

【重要：安全性の表現】
このV7は「名目軌跡の危険候補」診断です。
実運動と予測の4秒差の物理保証はないため、名目軌跡を41点で折線化した
幾何判定から、真の連続時間衝突がないことは証明しない。
内部V4カーネルは名目折線比較用にrho=acc=0で呼ばれるが、これは
「真の不確かさがゼロ」という仮定ではありません。
外部公開結果 safeToCommit=false / physicalClearCertified=false を常に維持。
出力 NOMINAL_POLYLINE_SEPARATED__NOT_CERTIFIED はCLEAR_CONDITIONALと同等ではない。
UNKNOWN/飽和/真の位置誤差の扱いは本番安全設計で改めて必要。

【実際のFlightで持つべき履歴（約1秒、41点）】
・EKF pL(3x41), vL(3x41)
・その各周期の既知の reference result.state.xd(1:3)（吊り荷の基準）
・同 xd(5:7)（基準の実時間1階微分）
・現在Governorの virtualTime, rateState(6), heldRateCommand と設定
・最新の EKF mL（毎周期 wp_six_trigger_model_snapshot で取得）

テストで使うPV初期化の速度重み0.1[s]は、位置測定1mm・速度測定10mm/sという
想定ノイズの比です。衝突用のマージンや物理保証値ではありません。
実機EKFの共分散があればその値に応じて更新するのが妥当ですが、今回は
研究の項目を増やさず固定の試験設定にしています。

【ファイル】
SIX_WAY/wp_six_trigger_pv_cache_prepare.m
SIX_WAY/wp_six_trigger_pv_forecast.m
SIX_WAY/wp_six_trigger_route_cache_prepare.m
SIX_WAY/wp_six_trigger_selected_nominal_v7.m
TESTS/TEST_TRIGGER_V7_PV_AND_SELECTED_GEOMETRY.m
RUN_PURE_TRIGGER_V7_SELECTED_AUDIT.m
TESTS/BASELINE_TRACES/PURE_TRIGGER_V6_CAUSAL_AUDIT_20261113_013957.mat

【準備・実行】
cd('C:\Users\student\Documents\GitHub\common_matlab')
addpath(genpath(pwd))
cd('TRIGGER_LIGHT')
clear classes
rehash
R = RUN_PURE_TRIGGER_V7_SELECTED_AUDIT();
disp(R.status)
disp(struct2table(R.forecastRecords))
disp(struct2table(R.noiseRecords))
disp(struct2table(R.geometryRecords))
disp(R.resultsZip)

結果ZIPはTESTS/RESULTS/にタイムスタンプ付きで生成。
同名の過去ZIPを上書きしない。

【実装／性能の現時点の位置づけ】
・今回はMATLAB実行環境がないため、上記TESTのMATLABでの速度と合否は未確認。
・独立Pythonで保存済み6秒HLC名目軌跡を使うPV履歴初期化の数値比較は実施。
  ただし、元制御器ゲインを表示された丸め固有値から近似した探索結果であり、
  MATLABの元ゲインに対するPASS判定には用いない。
・テストの4ケース・2開始時刻×6方式はあくまでオフライン比較であり、
  オンラインのtickはONE方式。新規障害物前処理とルートキャッシュ作成はtick外。
・入力飽和条件、実Flight/EKFによる軌跡比較、真の誤差上界、25ms全制御周期
  までの統合は完了していない。
・1～2msは「予測＋ONE方式の幾何判定＋質量取得」の実測で判定する。

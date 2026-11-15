純粋移植へ戻す / TRIGGER_LIGHT FAST V3 (2026-11-12)
=====================================================

【重要】まだ本番に移植しない。common_matlab/TRIGGER_LIGHT/ で検証する。
既存HLC/EKF/plant/CBF/Waypoint/run は本ZIPで上書きしない。

【実行中に適用するのは1方式だけ】
 C x multiple_spheres
 C x multiple_ellipsoids
 B x multiple_spheres
 B x multiple_ellipsoids
 D x multiple_spheres
 D x multiple_ellipsoids
を各々単独でテスト実行する。6方式の時間は合算しない。
目標は選択1方式の予測+足切り+幾何判定の合計1～2ms、全系25ms。
本パッケージでこの性能はまだ達成・検証されていない。

【旧V2は維持 / V3はA/B用の候補】
 SIX_WAY/wp_six_trigger_evaluate.m        元のV2、未変更。
 SIX_WAY/wp_six_trigger_evaluate_fast_v3.m 新しいV3比較候補。
 SIX_WAY/wp_six_trigger_interval_broadphase.m
   V3用3身体の一括・区間AABB足切り。C/B/Dで包含式を別々に計算。
   前処理の費用で逆効果になるケースもあり得るため自動でV2を置換しない。

【元の24状態モデルと実HLCを使った診断】
 SIX_WAY/wp_six_trigger_original_model_binding.m
   get_model_name('Load_HL')を解決し、元with_load_model_euler_for_HLを使う。
   source physical parameters 21個を取得し、LOCAL COPYの20番だけ
   現在EKF推定値mLへ置換する。元agent.parameter/plantを変更しない。
 SIX_WAY/WP_TRIGGER_PARAMETER_VIEW.m
   元HLCが呼ぶparameter.getの読み出し専用代理。
 SIX_WAY/wp_six_trigger_predict_hlc_4s.m
   今の24状態EKF、今の参照28値、6段Governor状態から4s先を予測。
   各0.025sで実HLC_SUSPENDED_LOAD.doを計算、原24状態RHSで積分。
   将来の推定状態は予測された状態を使用。未来EKFセンサ補正は不明。
   omega, qと形状を含む診断に利用可能。
   rhoDrone/rhoLoad/accDrone/accLoad=NaNとして返す。これを
   「物理的全時間CLEAR」に直接使用することは禁止。
   160回HLC+ODEなのでオンライン1～2msに収まる想定ではない。

【テストを一括実行】
    cd('C:\Users\student\Documents\GitHub\common_matlab')
    addpath(genpath(pwd));
    cd('TRIGGER_LIGHT');
    clear classes; rehash;
    R=RUN_PURE_TRIGGER_FAST_V3_TESTS(60,100,true);
    disp(R.status)
    disp(R.sourceEquations)
    disp(R.originalHlc)
    disp(R.geometry)
    disp(R.benchmark.records)

  補足: すでに他のクラスがメモリ上にある場合、clear classes は
  既存の動作中Flightを終了してから行うこと。
  モデル本体やHLCのパスがなければoriginalHlc/sourceEquationsはSKIPPED。
  事前に次を確認:
     which with_load_model_euler_for_HL -all
     which HLC_SUSPENDED_LOAD -all
     which WAYPOINT_REFERENCE -all
     which gen_wp_straight -all
  未検出なら元のcommon_matlab/model/substance、controller、
  WAYPOINT_REFERENCEをMATLABパスに追加。テスト用の偽モデルで代用しない。

  結果ZIP:
  TRIGGER_LIGHT/TESTS/RESULTS/PURE_TRIGGER_V3_RESULTS_FOR_CHATGPT.zip

【試験の意味】
TEST_TRIGGER_V3_24STATE_FORMULAS:
  元RHSと張り索理論の加速度・方向・角速度を300個の状態で照合。
TEST_TRIGGER_V3_ORIGINAL_HLC_4S:
  元24状態RHS+元HLC+現在のGovernorから160ステップの名目予測。
  推定mL=0.14、別のParameter.loadmass=0.01のテスト用agentで
  推定質量優先を確認。元Flight agentは触らない。
TEST_TRIGGER_V3_GEOMETRY_AUDIT:
  V2/V3各モード単独の一致と、事前足切りの数学的包含を監査。
BENCH_TRIGGER_V3_ONE_MODE_AB:
  V2/V3単体の中央値・P95・最大、内訳を比較。HLC予測は除く。
COMPARE_TRIGGER_V3_HLC_TO_FLIGHT_LOG:
  実Flightトレース(機体・吊り荷位置)との比較。後付け結果は誤差保証ではない。

【このZIPに入っていない機能】
- 実Flightの現在EKFへトリガをオンライン接続するコード。
- 将来のEKF測定補正を含む予測。
- 保証付き予測誤差の数学的導出（未知のパラメータ誤差上界が必要）。
- 予測+衝突判定すべてで1～2msの実証。
- 通常処理・CBF・軌道生成を含む25ms全工程の統合実測。
- 6方式のCBF側最終形状定義。現在は仮形状。

【累積引継ぎ】
PURE_PORT_CUMULATIVE_HANDOFF.txtは過去全文を保持し、
今回の結果・未完了・原ソース所在・先行研究の結論を先頭に追記。

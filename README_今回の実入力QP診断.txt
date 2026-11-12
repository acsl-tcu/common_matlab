CBF 実入力閉ループ・ステップ12診断（追記）
=======================================
目的:
  これまでのtest_CBF_PreIntegration()が実入力方式のstep12で停止した原因を特定する。
  test_CBF_StandaloneRollout.mや実際のCBF-QPを変更して成功に見せかけない。

配置:
  common_matlab/test_CBF_DiagnoseRealRollout.m （テストのみ直下）
  既存の生成器は従来どおり model/substance/generate_cbf_suspended/

実行方法:
  test_CBF_All();
  report = test_CBF_DiagnoseRealRollout();

結果:
  生成器/CBF/test_results/real_B_sphere_diagnosis.csv
  生成器/CBF/test_results/real_B_sphere_failure_snapshot.mat

判定:
  ・LP最小正規化スラック > 約1e-7: 元のハード制約の競合が疑われる。
  ・LPスラックが0付近、quadprogだけ失敗: 数値条件・解法設定を疑う。
  ・min(psi)<0: 初期高次安全集合の逸脱。今の入力のQPが解けても安全証明とは異なる。
  ・input limit violation: 拡張実入力の制約設計または積分刻みを検証。

LPの緩和変数は診断専用で、制御に適用しない。
これは任意姿勢・任意軌道の安全保証のための試験ではない。

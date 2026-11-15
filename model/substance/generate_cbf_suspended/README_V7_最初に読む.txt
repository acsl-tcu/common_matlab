CBF V7 — 使い方と変更点（2026-11-12）
======================================
【最初に重要なこと】
今回はCのケーブル球列と、QPの異常挙動を見つける検証機能を変更した。
実HLC接続前。どの姿勢/軌道でもQPが必ず解ける安全保証は未完成。
MATLAB R2026aでのV7実行はまだ確認していない。成功を捏造しない。

【配置】
ZIPの common_matlab/ 以下を、手元の common_matlab/ に重ねてコピー。
元のgenerateModelとgenerateControllerは含まれない。
テストはcommon_matlab直下、生成器はmodel/substance/generate_cbf_suspended。
生成出力は同生成器/CBF/real および同生成器/CBF/virtual に分離。
以前のV5の generated/CBF/real,virtual は test_CBF_All で上書き生成する。

【実行】
cd('common_matlabのパス');
test_CBF_V7_All();
% 特定テストのみなら：
test_CBF_All();
test_CBF_CableSpheres();
test_CBF_ExplicitDecisionBounds();
test_CBF_RealGroundFeasibilitySweep([1.2 1.5 2.0]);
test_CBF_RobustScenarioSweep(12);
test_CBF_ReferenceTrajectoryCases(25);
% 新PCでの形状別・障害物数別計測：
results=benchmark_CBF_DetailedProfile(20,[1 3 5 10]);
% 旧ストレス試験も原因診断として残す：
rollout=test_CBF_MultiObstacleRollout({'B','D','C'},40);

【C方式球列の実行時設定】
settings=test_CBF_Fixture();   % テスト数値のみ。利用時は外部から設定
plan=cbf_choose_cable_spheres(settings);
plan.numberOfSpheres   % ケーブル内部追加球数（端球は数えない）
plan.totalSystemSpheres% 内部＋機体球＋吊り荷球
plan.centerFractions   % 機体中心から吊り荷中心への0..1の中心位置
plan.sphereRadius      % この球の半径

% 半径を外部から与える場合（生成し直す必要なし）：
settings.cable.sphereRadius=0.035;
settings.cable.radiusRatioToSmallBody=[];
settings.cable.numberOfSpheres=[];
plan=cbf_choose_cable_spheres(settings);

% 小さい方の端球から比率を指定：
settings.cable.sphereRadius=[];
settings.cable.radiusRatioToSmallBody=0.35;
settings.cable.numberOfSpheres=[];
plan=cbf_choose_cable_spheres(settings);

【変更した関数】
cbf_choose_cable_spheres.m / cbf_geometry_core.m / cbf_geometry_taylor.m
CBF_RuntimeSettings_Template.m / cbf_parameter_vector.m
cbf_solve_hocbf_qp.m (第4引数の試験用決定入力ハード限界を追加)
テスト: test_CBF_CableSpheres, test_CBF_ExplicitDecisionBounds,
 test_CBF_RealGroundFeasibilitySweep, test_CBF_RobustScenarioSweep,
 test_CBF_ReferenceTrajectoryCases, test_CBF_V7_All。
V5の高次Taylor係数の計算方式とQP目的関数・物理モデルは変更しない。

【地面QP問題を「直した」か？】
まだ全状態で不可行を回避できていない。V5ストレス条件では
地面6次HOCBFと高次入力上下限制約が矛盾した。
V7は虚偽のPASSを防ぎ、複数姿勢・ゲインで可行領域を診断する。
普遍的な解決には制御器・安全領域・相対次数・可行性維持の再設計が必要。

【仮想Dの発散を「直した」か？】
異常巨大入力を有界な試験決定入力集合で拒否し、
QP不可行としてログに残す安全な試験動作へ変更した。
これにより自動的に実入力安全が保証されるわけではない。
結果はCBF/test_results/*.csvに出る。

【引き継ぎ】
CBF_研究実装試験_全経緯引き継ぎ_V7_20261112.txt
＋旧CBF_全経緯_設計実装試験引き継ぎ_20261112.txt を参照。

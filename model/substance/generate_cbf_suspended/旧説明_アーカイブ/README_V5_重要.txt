CBF 12構成 V5 — 地面CBF可行性と複数障害物の検証（2026-11-12）
============================================================

【重要】これはMATLAB上での接続前検証版です。MATLAB本体での実行・HLC接続・実機試験は未確認。
単にQPが1回解けることと、全時間の安全性・将来のQP可行性は異なります。

[フォルダ配置]
 ZIP内の common_matlab/ から手元の common_matlab/ にコピーしてください。
 common_matlab/test_*.m, benchmark_*.m  ... 共通matlab直下
 common_matlab/model/substance/generate_cbf_suspended/
      generateCBF_RealInput.m
      generateCBF_VirtualInput.m
      cbf_common/ ... 導出・幾何・シーン共通部
      CBF/real, CBF/virtual, CBF/test_results ... 実行時作成
 元のgenerateModel_SupendedLoad / generateController_SuspendedLoad は入れていません。
 生成済みの各12 .m本体は user のMATLAB上で test_CBF_All() により再生成できます。

[実行順]
 MATLAB作業フォルダを common_matlab にする。
 1. test_CBF_All();
 2. test_CBF_V5_Review();
 3. test_CBF_GroundGainDiagnostics([1.2 1.5 2.0 3.0]);
    （real_B_sphere_failure_snapshot.mat を CBF/test_results/ に置く）
 4. report=test_CBF_RealGroundPolicyRollout([1.2 1.5 2],25);
 5. benchmark_CBF_MultiObstacle(5,[1 3 5 10]);
 6. benchmark_CBF_SceneOptimization(10,[1 3 5 10]);
 7. results=test_CBF_MultiObstacleRollout({'B','D'},40);
    重いC方式の閉ループは必要なら {'C'} を指定。

[変更点１ — 地面HOCBF専用ゲイン]
 settings.cbf.groundGain = 1.5;    % 任意。設定しなければ hocbfGain と同じ
 設計階数6を勝手に下げず、正のゲインのHOCBF形式を維持。
 地面と入力限界の制約はハードのまま。スラックや入力クリップなし。
 保存ステップ12の数値を再分析するとゲイン1.2でQP不可行、
 同じ高次CBF形式で地面ゲイン1.5ならその時点のQPが可行となる。
 これはゲイン1.5が安全/最適という意味ではない。
 前回のstress nuNom=[0;20;0;0] を変えず全期間成功するかは未確認。
 ゲイン変更後の高次補助条件と離散ステップの安全を必ず計測する。

[変更点２ — 重複計算の除去]
 cbf_hocbf_scene: まず動力学のTaylor係数を「入力0＋4基底」の5本だけ先に作り、
  その結果をすべての障害物で共有する（障害物姿勢は各行の幾何評価へ反映）。
 2個目以降の障害物では地面/入力限界のTaylor係数を
 生成しない。従来は計算した後で捨てていた。1個目で1回だけ計算。
 test_CBF_SceneEquivalence で従来全計算版とのA,b,ラベル一致を比較。

[変更点３ — 数学的に冗長な地面行だけを除去]
 cbf_prune_proven_ground:
 B: 半径がケーブルより大きい機体球・荷物球が各端点の地面条件を包含。
 C real: 両端球半径>=被覆球半径、全地面制約6次・同一地面ゲインなら
         中間点の地面高次CBF行は両端の凸結合から従う。
 C virtual: 機体4次・荷物6次が混在するため、上の証明は使わない。
 D: 楕円体地面制約は独立のまま維持。
 無効化したい場合 settings.cbf.pruneRedundantGround=false;
 幾何hは変更しない。障害物の制約を距離で省略する処理もない。

[変更点４ — 複数障害物閉ループ]
 test_CBF_MultiObstacleRollout:
 HLCは呼び出さない。実入力拡張系は独立RHS/RK4、仮想系は6次積分器。
 前面、三方向、狭い通路、5障害物の条件で、多障害物QPを毎ステップ解く。
 原因を記録して停止：QP不可行、初期HOCBF集合逸脱、張力モデル失効、衝突等。
 完走しても目標到達や閉ループ全時間の安全性の証明ではない。
 比較条件は実/仮想で名目入力・作用が異なるためそのまま性能比較不可。
 CSV：CBF/test_results/multi_rollout_*.csv および multi_rollout_summary.csv。

[計算時間]
 現時点で「生成」と呼んでいるのは MATLAB .m ファイルの生成。
 MEX実行ではありません。コード生成にはMATLAB Coderと対応コードの整備が必要。
 benchmark_CBF_SceneOptimization で重複計算除去前後の係数時間を比較。
 benchmark_CBF_MultiObstacle で係数/quadprog/合計時間を計測。
 高次CBF係数評価とQPの時間を別々に表示。平均や1周期の最悪値は中央値とは異なる。

[残る理論上の課題]
 ・元モデルの一律6階HOCBFは高次拡張による保守性があり、QP不可行があり得る。
 ・地面専用ゲインで一時点の可行性が改善しても再帰的可行性は証明されない。
 ・仮想入力側の実HLC逆変換による推力/トルク制約は未接続。
 ・張ったケーブルの正張力と逆変換非特異性は適用領域条件。
 ・移動障害物、障害物の高速姿勢変動、弛んだ紐の安全性は扱っていない。
 ・CBFによる高次補助初期集合の成立と離散化誤差の検証が必要。

[参考]
 Xiao & Belta, High-Order Control Barrier Functions, IEEE TAC 67(7), 2022.
 DOI: https://doi.org/10.1109/TAC.2021.3105491
 Xiao, Belta & Cassandras, Sufficient conditions for feasibility of optimal control problems using CBFs,
 Automatica 135 (2022) 109960. DOI: https://doi.org/10.1016/j.automatica.2021.109960
 MathWorks, codegen: https://www.mathworks.com/help/coder/ref/codegen.html
 MathWorks, quadprog code generation: https://www.mathworks.com/help/optim/ug/code-generation-in-quadprog.html

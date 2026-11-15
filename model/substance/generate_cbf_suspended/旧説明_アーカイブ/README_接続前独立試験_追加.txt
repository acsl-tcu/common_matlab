3D CBF — HLC接続前の独立試験 V3 (2026-11-12)
================================================

【結論：何を検査する版なのか】
前回の test_CBF_All() で通った12構成の幾何計算・高階係数確認に対して、
HLCを呼ばない状態でQPの実解、元モデルの力学一致、入力の同時作用、
任意回転障害物、短時間のサンプル値閉ループまで試験する。
これらの数値テストの合格は、安全性の数学的証明ではない。

【配置】
common_matlab/
  test_CBF_All.m                         (前回から維持)
  test_CBF_PreIntegration.m              (今回の入口)
  test_CBF_ModelParity.m                 (追加)
  test_CBF_IndependentPlantRHS.m          (追加;テスト専用の独立運動方程式)
  test_CBF_IndependentDynamics.m          (追加)
  test_CBF_ObstacleStress.m               (追加)
  test_CBF_QPContract.m                   (追加)
  test_CBF_GeometricClearance.m           (追加)
  test_CBF_InitialSet.m                   (追加)
  test_CBF_InputAffinity.m                (追加)
  test_CBF_StandaloneRollout.m            (追加)
  benchmark_CBF_PreIntegration.m         (追加;計測は任意)
  model/substance/generate_cbf_suspended/
    generateCBF_RealInput.m               (前回版を維持)
    generateCBF_VirtualInput.m            (前回版を維持)
    cbf_common/                           (前回版を維持)
    CBF/real/                             (実行時自動生成)
    CBF/virtual/                          (実行時自動生成)
    CBF/test_results/                     (追加試験で自動作成)

【実行】
1. 現在のフォルダを common_matlab にする。
2. 前回の一括試験が実行済みなら再生成は不要だが、ファイルを入れ替えたら：
      test_CBF_All();
3. 追加試験：
      test_CBF_PreIntegration();
4. 計算時間の測定（任意、ベンチマークは合否に含めない）：
      benchmark_CBF_PreIntegration(3);

Optimization Toolboxのquadprogが必要。見つからない場合は明示的にエラーとする。
生成時の寸法値や入力制約は固定しない。試験数値は従来どおりtest_CBF_Fixtureにのみ置く。
元のgenerateModel.m, generateController.mやHLC実装はこのZIPに含まない。

【追加テストの内容】
1) ModelParity:
   単一の張ったケーブル状態を実入力・仮想入力の両方で表し、C/B/D×球/回転楕円体のh一致。
2) IndependentDynamics:
   張力正の非ホバリング30状態で、2つの独立な数式記述による状態一階微分の一致。
   ※ 現実の元MATLAB関数を直接実行した一致試験ではない。
3) ObstacleStress:
   SO(3)の24ランダム姿勢と、球＋回転楕円体が共存する複数障害物の係数集約。
4) QPContract:
   作為的な可行問題と不可行問題、12構成の遠方障害物＋ホバリング可行状態を検査。
   制約を黙って緩めたり入力をクリップしない。
5) GeometricClearance:
   楕円体までの独立した点距離最適化を使い、h>=0が保守的な十分条件か照合。
   45ランダム障害物、各形状、ケーブル中心線51サンプル。
6) InitialSet:
   h>0でも高速接近で高次補助条件psi<0になる状態を検出。
7) InputAffinity:
   4つの決定入力を同時に非ゼロにして、6次/4次係数のアフィン性を照合。
8) StandaloneRollout:
   HLCではなく明示的なストレス名目入力を用いる。
   virtual: 理想6重積分器を厳密な多項式で24ステップ。
   real: 非線形拡張モデルを独立RK4で25ステップ。
   QPの解→積分→状態と安全関数/入力上下限/張力を再検査。
   CSVをCBF/test_resultsへ出力する。

【独立閉ループ試験の限界】
・名目入力を作るのはHLCではない。入力飽和やHLC逆変換は検証していない。
・この短い時間積分は指定した1軌道・1初期条件の数値試験である。
・サンプル間安全、確率外乱、予測再帰可行性を証明しない。
・状態は自由に選べるが、モデルが成立する張力正の状態が前提。
・障害物が動く場合は障害物の高階時間微分が必要。今回の障害物は静止。
・実入力側のT^(4), tau''と、HLCからの名目T,tauの接続はまだ未実装。
・仮想入力側のpL^(6), yaw''と、HLCのv1..v4の変換/初期微分の接続も未実装。
・12構成すべての閉ループ飛行を証明したわけではない。

【合格後のHLC接続の最低条件】
A. HLCなしの追加試験8種がPASS、ログの全min_h>=0、最小QP残差>=-1e-7。
B. real/virtualについて、実入力上下限と高次補助集合初期条件を確認。
C. 現在のHLCのUf / Beta2 / V2_alpha2の逆変換式をテスト状態で一致検証。
D. v1の拡張状態の初期値・高階微分の生成方法と、HLC実入力名目値の時間連続化を決定。
E. 適用対象として張力>0、姿勢正規化、特異性/入力係数の退化を診断。
F. 有限サンプリング周期で最低距離を高頻度の独立計算でもチェック。
G. 新しい実行環境の計算時間（p50, p95, 最大）を計測。

なお将来のQP実行不能を絶対に起こさないことは、これらのテストでは証明できない。
最初のHLC接続試験では非飽和・単一静止球・安全距離が大きい条件から始める。

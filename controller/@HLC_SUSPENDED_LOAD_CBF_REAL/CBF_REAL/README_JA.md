# REAL HLCのCBF一括切替（第18段階）

## どこを編集するか

CBF方式番号 `cbfId` **1つだけ**で、方式・機体形状・紐形状・牽引物形状・地面の扱い・数値最適化を一体で選択する。

- `cbfId=0`: 従来HLCの入力クリップ（CBFなし）。
- `cbfId=1`: 実入力4変数同時最適化（Stage16試験版）＋機体球＋牽引物球＋紐楕円体＋地面。
- 新方式は `CBF/SCHEMES/CBF_REAL_SCHEME_02_*.m` のように**1ファイルで**作る。必要なら `CBF_REAL_INIT.m` と `CBF_REAL_DISPATCH.m` へ番号を追加する。

## 新しいフォルダ

```text
@HLC_SUSPENDED_LOAD_CBF_REAL/
├── HLC_SUSPENDED_LOAD_CBF_REAL.m
├── (既存のHLCメソッド群:変更しない)
└── CBF/
    ├── CBF_REAL_INIT.m
    ├── CBF_REAL_DISPATCH.m
    ├── README_JA.md
    └── SCHEMES/
        └── CBF_REAL_SCHEME_01_JOINT_ZOH.m
```

**第17段階の `CBF/METHODS/` と `CBF/GEOMETRY/` は不要。** このZIPには含めていない。ローカルに残っていても第18段階のコードからは参照しないが、紛らわしいので内容を確認してから古い2フォルダを削除してよい。 `CBF_GENERATOR/COMMON/CBF_COMMON_GEOMETRY.m` は**共通の物理幾何計算**として引き続き使う。近似方式を選ぶ分岐は `SCHEMES/` 側の1ファイルに集約した。

## シミュレーションごとのCBF選択

```matlab
realCBFId=1; % この番号だけを変更
hlcParam=Controller_HL_Suspended_Load(dt,agent);
hlc=HLC_SUSPENDED_LOAD_CBF_REAL(agent,hlcParam,realCBFId);
agent.controller.set_function_class("hlc_suspended",hlc);
```

従来の `HLC_SUSPENDED_LOAD_CBF_REAL(agent,hlcParam)` は `cbfId=0` と同じ。番号はコンストラクタで確定し、飛行中に変更しない。既存のREAL/VIRTUAL切替があるSimではREAL側だけ修正する。

**第17段階の `struct('methodId',1,'geometryId',1)` は使用しない。** 第18段階では第3引数に数字 `1` を渡す。

## 方式01の近似や制約の設定

`SCHEMES/CBF_REAL_SCHEME_01_JOINT_ZOH.m` 冒頭の「方式01の設定」だけを編集する。

```matlab
geometryOptions=struct();  % 形状近似の数値設定
solverOptions=struct();    % 非線形最適化の数値設定
storeFullResult=false;    % ログ容量を節約
```

関数内で `CBF_COMMON_GEOMETRY(P,geometryOptions)` を呼び、センサに観測された地面情報を反映する。観測障害物 `cbfObservedWorld.parts` のみを使用し、Simが保持する全障害物や真値を使わない。

障害物側の `approxMethod='single_ellipsoid'` は従来のSim側の設定。`realCBFId` は機体・紐・吊り荷の近似とCBFアルゴリズムをまとめた番号なので、混同しない。

## 実行に必要なファイル

以前から使用中の `CBF_GENERATOR` は**変更しない**。 MATLABで `generate_CBF_REAL_HOCBF_ZOH` を実行して、次の3つが生成済みであることを方式01の初期化時に検査する。

- `CBF_REAL_ZOH_DIAG_EVAL.m`
- `CBF_REAL_ZOH_HOCBF_INEQUALITY.m`
- `CBF_REAL_ZOH_JOINT_FILTER.m`

HLCはその後、毎周期 `tmp=[uf(1);us]` の直後に `CBF_REAL_DISPATCH` を1回呼ぶ。CBF ONでは出力後に再クリップせず、解が得られないときはエラー停止する。

## 注意

- 方式01は4入力同時非線形最適化の**試験版**。安全性が数学的に保証された標準HOCBFではない。
- 推力は2階、トルクは一般状態で4階から作用するモデルを使用。第16段階のgeneratorで検査済みだが、任意の状態で係数が非零という意味ではない。
- 観測なし、古いセンサ時刻、現在既に安全でない状態、最適化失敗、数値照合失敗などで停止し得る。失敗を公称HLCへ隠して戻さない。
- サンプル時間25 msへのリアルタイム適合、モデル誤差、連続時間の安全保証は今後の検証項目。
- MATLAB実機の実行試験は本成果物では未実施。静的な参照・構成・差分を確認した。

# 2026-11-14：4つの元障害物方法の試験入口

旧テスト入口はフォルダの隣接を仮定する場合があるため、**新しい起動ファイル** `common_matlab/RUN_RA_FOUR_METHODS_TESTS_20261114.m` を使用してください。MATLABでZIPを展開しません。

```matlab
cd('C:\Users\student\Documents\GitHub\common_matlab')
R=RUN_RA_FOUR_METHODS_TESTS_20261114(pwd,true);
disp(R.status);disp(R.failures);disp(R.resultsZip);
```

`09_FOUR_METHODS` は単一球・複数球・単一楕円体・複数楕円体の形状読み取り、各C/B/Dへの受け渡し（12組合せ）、元ジェネレータのパーツ同一性、36パーツセンサベンチマークを検査します。旧テストも残してあります。元の`DRONE_OBSTACLES`がMATLABパスにない場合は元生成器接続試験のみSKIPPEDとなります。

---

# 本番とは分離したテスト群

**配置エラー時**はZIP最上位の `INSTALL_AND_TEST_README_JA.md` を参照してください。
`RUN_REFERENCE_AVOIDANCE_QUICKSTART.m` はZIP展開場所から自動でテスト入口を検出します。

このフォルダは `common_matlab/REFERENCE_OBSTACLE_AVOIDANCE_TESTS/` に置き、トリガ本番コードとは別に管理します。

| フォルダ | 内容 |
|---|---|
| `01_INSTALLATION` | 移設後のパス・MEX位置・旧フォルダに依存しないこと |
| `02_GEOMETRY` | C/B/D × 球/楕円体の6方式独立判定、空障害物テスト |
| `03_MEX` | C方式のMEX自動ビルド、80条件数値一致、非MEX分類一致、Bでは不使用 |
| `04_TRIGGER` | 最新EKF推定質量、25msの41実サンプル、欠測履歴リセット |
| `05_PERFORMANCE` | 6方式の**幾何部分だけ**の中央値・P95・最大。予測込み2ms判定ではない |
| `06_ORIGINAL_TRACE_REPLAY` | **任意**：以前の元HLC 6秒保存軌跡でV7/V8の96条件を再比較。元DRONE・元HLC等がMATLABパスに必要 |
| `SUPPORT` | フィクスチャ。テストの質量999は不正利用を検出するための番兵であり真値には使わない |
| `RESULTS` | MAT/ZIP出力。既存ファイルを上書きせず日時付き |

## 実行

```matlab
cd('C:\Users\student\Documents\GitHub\common_matlab')
addpath(fullfile(pwd,'WAYPOINT_REFERENCE','REFERENCE_OBSTACLE_AVOIDANCE'))
addpath(fullfile(pwd,'REFERENCE_OBSTACLE_AVOIDANCE_TESTS'))
R=RUN_REFERENCE_AVOIDANCE_TESTS(true);
disp(R.status)
disp(R.mex.native.status)
disp(struct2table(R.performance.records))
disp(R.resultsZip)
```

**MATLAB実行環境がない配布者側では、これらの結果をPASSと主張しません。** 現時点の自動テストはインストール、幾何6方式、MEX自動生成と分類一致、EKF履歴の配線を対象にします。実Flightでの6秒比較、推定ノイズを含む予測精度、全25ms工程と実安全の検証は、このテスト群だけでは完了しません。

## 以前の96条件を再検証する場合（任意）

旧V8の96条件比較コードと元HLCの6秒保存MATを`06_ORIGINAL_TRACE_REPLAY`内に隔離しています。`DRONE`・`DRONE_PARAM_SUSPENDED_LOAD`・`Controller_HL_Suspended_Load`・`gen_wp_straight`を元プロジェクトからパスに加えてから、`R=RUN_REFERENCE_AVOIDANCE_TESTS(true,true);`。必要なクラスがない場合はその追加試験のみSKIPPEDと記録します。この旧テストは移設後の**内部関数**を呼ぶ比較で、外側の新しいMEX準備関数の実行まで含むものではありません。

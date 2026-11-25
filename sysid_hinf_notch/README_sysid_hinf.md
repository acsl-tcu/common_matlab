# 単機牽引ドローン：牽引物の揺れの同定と H∞ 抑制

単機牽引（吊り下げ負荷 = *suspended load*）ドローンで牽引物がふらふらする問題に対し、
**「ピッチトルク τy → 牽引物の揺れ角 θ」** をシステム同定してボード線図を描き、
**H∞（混合感度）制御**で揺れを抑えるための MATLAB スクリプト一式です。共通プログラム
（[`acsl-tcu/common_matlab`](https://github.com/acsl-tcu/common_matlab)）の
吊り下げ負荷モデル（`Model_Suspended_Load` / `mode/SuspendedLoad`）をそのまま使います。

## なぜふらふらするのか（物理）

ピッチ面（x–z 平面）の微小角モデルは次の 2 つの式で書けます。

- 機体姿勢：`Jy·β'' = τy − c_att·β'`　（β：機体ピッチ角、c_att：ロータ抗力）
- 負荷振り子：`M·L·θ'' + (M+mp)·g·θ = −f0·β`

ここから、揺れ角の運動は

```
θ'' = −ωn²·(θ + β) − 2ζ_p·ωn·θ' ,     ωn² = (M+mp)·g / (M·L)
```

となり、伝達関数（減衰を無視した理想形）は

```
                −ωn²
P(s) = θ/τy = ───────────────────
              Jy·s² · (s² + ωn²)
```

つまり **姿勢の二重積分 × 軽減衰の振り子共振 ωn** を直列にした 4 次系です。
この振り子共振（下記の値では約 0.77 Hz）が「ふらふら」の正体で、減衰比が小さい
（ζ_p ≈ 0.03 程度）ため、一度揺れると数十秒も収まりません。H∞ 制御でこの共振に
減衰を与えるのが狙いです。

### DIATONE パラメータでの数値
（`DRONE_PARAM_SUSPENDED_LOAD("DIATONE")` の既定値：M=0.762, mp=0.0556 kg, L=0.46 m, Jy=0.02985, g=9.81）

- 振り子固有周波数 **ωn ≈ 4.78 rad/s ＝ 0.76 Hz**
- ホバリング推力 f0 ＝ (M+mp)g ≈ 8.02 N
- 開ループの揺れ減衰比 ζ ≈ 0.03（＝ふらふら）
- 参考：能動制振でこの共振に ζ ≈ 0.5 を与えれば、10° の揺れは約 2 秒で収束します
  （`preview_sway.png` 参照。これは Python での成立性確認で、実設計は MATLAB の H∞）。

> **ωn はケーブル長 L で大きく変わります**：`ωn ≈ √(g/L)·√((M+mp)/M)`。
> 例）L=0.97 m なら ≈0.52 Hz、L=2.0 m なら ≈0.36 Hz。`run_sysid_experiment.m` と
> `susp_load_linear_model.m` では対象の cableL / loadmass / mass を必ず実値に設定してください。

## ファイル構成と流れ

| ファイル | 役割 |
|---|---|
| `susp_load_linear_model.m` | 物理から τy→θ の**解析線形モデル**を構築（同定の初期値・H∞ のノミナル） |
| `run_sysid_experiment.m` | 共通プログラムのプラントを**チャープ励振**し、(τy, θ) を `sysid_data.*` に保存 |
| `identify_sway_tf.m` | 励振データから **tfest/ssest** で同定、解析モデルと Bode で照合 |
| `design_hinf_sway.m` | **mixsyn** による混合感度 H∞ で制御器 K を設計、S/T/KS・余裕・時間応答を評価 |
| `main_sysid_hinf.m` | 上記を束ねるトップスクリプト（STEP0〜3） |

処理の流れ：

```
物理モデル ──▶ 励振シミュ ──▶ 同定(tfest/ssest) ──▶ ボード線図 ──▶ H∞設計 ──▶ 評価
 (STEP0)       (STEP1)          (STEP2)              (STEP2)        (STEP3)
```

## 使い方

必要 Toolbox：**Control System / System Identification / Robust Control**。

> **重要（既定の挙動）**：`main_sysid_hinf` の既定は STEP0＋STEP3 のみで、
> **解析線形モデルだけ**で完結します。この経路は common_matlab の非線形モデル
> （`with_load_model_euler_for_HL`）を参照しないため、リポジトリの外でも toolbox
> さえあれば動きます。リポジトリの実ダイナミクスで同定したい場合は STEP1
> （`run_sysid_experiment`）と STEP2 を有効化してください。`run_sysid_experiment`
> は common_matlab のクラスを必須とし、見つからなければ明示的にエラーになります
> （実行時に `使用プラント dynamics = with_load_model_euler_for_HL` と表示して自己申告します）。

通常運用の入口は `mainGUI`（パス設定 → `SimExp(Setting)` でモード実行）ですが、
ここの一式は **同定・設計用の単独スクリプト**で、AGENTS.md にある `experiment/test_*.m`
と同じく `mainGUI` とは独立に直接実行します（各スクリプトが `addpath(genpath(pwd))`
でパスを通すため、`set_for_simulink` は不要です）。

1. `common_matlab` のルート（`DRONE.m` がある場所）に `sysid_hinf/` を置く
   （必要なら `experiment/` 配下に置いてもよい）。
2. まずは解析モデルだけで最後まで通す：
   ```matlab
   main_sysid_hinf     % STEP0（解析Bode）→ STEP3（H∞設計）まで実行
   ```
3. シミュレーションで同定したい場合は、`main_sysid_hinf.m` の STEP1/STEP2 の
   コメントを外す（または個別に実行）：
   ```matlab
   run_sysid_experiment                 % sysid_data.mat / .csv を生成
   [Pid,~,fit] = identify_sway_tf;      % 同定 & 解析モデルと照合
   [K,CL,gam]  = design_hinf_sway(Pid); % 同定モデルで H∞ 設計
   ```
4. 実機データを使う場合は、`t, tau_y, theta, dt` を持つ `.mat`
   （または CSV: 列 = t, τy[N·m], θ[rad]）に整えて `identify_sway_tf` に渡す。

## 設計（H∞ 混合感度）の考え方

`design_hinf_sway.m` は `mixsyn(P, W1, W2, W3)` で

```
min_K  ‖ [ W1·S ; W2·K·S ; W3·T ] ‖∞
```

を解きます。重みの意図：

- **W1（性能・感度 S）**：低域で高ゲイン → 積分作用と外乱抑制。交差周波数を
  `wb ≈ 2·ωn` に置き、**振り子共振を帯域内に入れて減衰**させる。高域は `1/Ms`（Ms≈2）。
- **W3（ロバスト性・相補感度 T）**：高域で T を絞り、未モデル化ダイナミクス・
  センサ雑音に対する頑健性を確保。
- **W2（制御感度 KS）**：高域の制御入力（トルク）を抑える。

`gamma`（達成 H∞ ノルム）が 1 前後なら重み仕様をほぼ満たしています。出力される図：
感度 S/T/KS と重み、開ループ L の余裕、外乱→θ の閉ループ Bode（共振ピークの低減）、
そして 10° リリースの時間応答（開ループの長いふらつき vs H∞ の速い収束）。

### 調整のヒント
- 揺れをもっと速く止めたい → `design_hinf_sway(P,"wb",3*wn)` のように帯域を上げる
  （制御入力・雑音感度とのトレードオフ）。
- `mixsyn` が原点極（姿勢の自由積分）で警告/失敗する場合：`susp_load_linear_model`
  の `c_att` を少し大きく（例 0.05）するか、`hinfsyn`/`ncfsyn`（正規化左既約分解）に
  切り替える。積分器自体は W1 の高DCゲインが補償するため通常は通ります。
- 実機は τy が「直接の物理トルク」でなく**ロータ推力配分経由**なので、同定は
  必ず実データ（`run_sysid_experiment` か実験）で ωn・ζ を較正してから H∞ に渡すこと。

## 注意（モデルの前提）
- ピッチ面（τy→θ）を対象。ロール面（τx→φ）は対称なので同じ手順で設計可能。
- 微小角・剛体ケーブル・ホバリング近傍の線形化。大振幅や急旋回では非線形シミュ
  （`SimSuspendedLoad`）で検証すること。
- `θ = atan2(pT_x, −pT_z)`（`pT` は負荷側ケーブル方向の単位ベクトル）で揺れ角を定義。

# SimMEC_K：手動操作・データ収集・オン／オフ設定

この手順は、`mainGUI.m` から `SimMEC_K.m` を使う**シミュレーション**用です。
GUIのキー操作で離陸・飛行・着陸を切り替えます。ジョイスティックによる操縦や実機通信の手順ではありません。

## 1. まず、どのファイルを変更するか

| 変更したいこと | ファイル | 変更する値 |
|---|---|---|
| 自動実行／GUI手動実行 | `mainGUI.m` | `Setting.fAutoTrials` |
| 円軌道／ランダム軌道の収集モード | `mainGUI.m` | `Setting.mec_dataset.enabled` |
| 手動1試行のシード | `mainGUI.m` | `Setting.trajectory_seed` |
| 自動収集するシード一覧 | `mec_dataset_settings.m` | `setting.seeds` |
| 4入力励振のオン／オフ | `mec_dataset_settings.m` | `setting.excitation_enabled` |
| 励振の各軸の強さ | `mec_dataset_settings.m` | `setting.amplitude` |
| 収集モードのMEC補償のオン／オフ | `mec_dataset_settings.m` | `setting.use_compensation` |
| 通常モードも含めたMEC補償の明示指定 | `SimMEC_K.m` | `mec_param.use_compensation` のコメント行 |
| 世界座標の外力のオン／オフ・強さ・方向 | `SimMEC_K.m` | `wind_mode`、`wind_amplitude`、`wind_axis` |
| プラントの慣性のモデル誤差 | `SimMEC_K.m` | `plant_model.param.param(6)` と `(7)` |
| MECゲインのファイル | `SimMEC_K.m` | `mec_param.gain_file` のコメント行 |
| 学習用ファイルの保存先 | `mec_dataset_settings.m` | `setting.output_dir` |

**設定を変更したら、ログを保存してから `mainGUI` を再実行してください。**
作成済みの `app` は初期化時の設定を保持します。設定ファイルを書き換えるだけでは、その試行には反映されません。

## 2. GUIを手動操作して1試行を収集する

### 2.1 起動前の設定

`mainGUI.m` の該当箇所を、次の値にします。周囲の初期化処理は残してください。

```matlab
Setting.mode = SimBaseMode(15); % SimMEC_K
Setting.trajectory_seed = 1;    % 今回収集する軌道・励振のシード
Setting.fAutoTrials = false;   % 起動直後の自動試行を止める
Setting.mec_dataset = mec_dataset_settings();
Setting.mec_dataset.enabled = true; % ランダムスプラインと学習ログを使う
```

`mec_dataset_settings.m` は、最初は次の組合せを使えます。

```matlab
setting.excitation_enabled = true;
setting.use_compensation = false; % HLCは使い、MEC補償だけを止める
setting.amplitude = [0.5;0.008;0.008;0.006];
```

この状態は「HLC＋4入力励振」です。世界座標の外力と慣性のモデル誤差は、別設定として残ります。

### 2.2 GUIの操作順

1. `mainGUI` を実行します。`fAutoTrials=false` なので、自動試行は始まりません。
2. GUIの **Start** を押します。
3. GUIにフォーカスを置き、小文字の **a** を押してアーミングします。
4. **t** を押して離陸します。目標高度は1 m、離陸参照の立ち上がりは既定で5秒です。
5. GUIのシミュレーション時刻を見て、離陸開始から約7秒を目安に **f** を押します。
6. ランダム軌道を飛行させます。4入力励振は、この **f** の区間だけ有効です。
7. 飛行開始から40秒以内を目安に **l** を押します。
8. 着陸参照の降下時間は既定で3秒です。降下後に **q** またはスペースを押して終了します。
9. **Save data** を押します。通常ログと、条件を満たした学習用データを保存します。
10. **Draw graph** を押して軌道・入力・励振を確認します。

| キー | 動作 |
|---|---|
| `a` | アーミング。シミュレーション時刻は0に保たれる |
| `t` | 離陸 |
| `f` | 軌道飛行。軌道と入力励振の経過時間が始まる |
| `l` | 着陸 |
| `s` | プラントの更新を停止。ループの終了・保存は行わない |
| `q`／スペース | ループを終了する。保存は **Save data** で行う |

時間は**実時間ではなく、GUIに表示されるシミュレーション時刻**で判断します。
計算速度によっては、実時間より速く進みます。

手動実行では、自動試行の「7秒で飛行へ切替」「40秒で着陸へ切替」は行いません。
ランダム軌道は既定で飛行開始から40秒までしか定義されていないので、
それを超えて `f` のままにすると参照時刻の範囲外エラーになります。
全体の終了時刻 `te` に到達しても、着陸への自動切替・自動保存は行いません。
自動試行にある「入力の絶対値が20以上」の中断判定も、手動ループには適用されません。
手動で収集した試行は、描画した軌道・入力を確認してから同定に使用してください。

### 2.3 次のシードで取り直す

最も分かりやすい方法は、保存後に `Setting.trajectory_seed` を2、3、…へ変更して `mainGUI` を再実行する方法です。

GUIを残して変更する場合は、**試行を終了して保存した後**、コマンドウィンドウで次を実行します。

```matlab
app.initial_setting.trajectory_seed = 2;
app.reset_app(); % 機体・軌道・励振・ログを新しく作り直す
```

その後、再び Start → a → t → f → l → q の順で操作します。
**Reload** だけを押した場合は、同じ設定・同じシードで最初からやり直します。
手動実行では `setting.seeds = 1:10` の一覧は使わず、`Setting.trajectory_seed` の1個だけを使います。

## 3. 「外乱」と「MEC補償」を分けて切り替える

入力の構造は次のとおりです。

```text
実機側入力 = HLCのノミナル入力 + MEC補償 + 4入力励振
学習用Δu = 実機側入力 - ノミナル入力 = MEC補償 + 4入力励振
```

世界座標の外力は、この式の入力和とは別に、プラントの並進加速度へ加わります。

### 3.1 世界座標の外力を切り替える

`SimMEC_K.m` の次の値を変更します。

```matlab
wind_mode = 'none';  % 外力OFF
% wind_mode = 'sweep'; % 外力ON：周波数が変わる波形
% wind_mode = 'step';  % 外力ON：一定値の区間
wind_amplitude = 0.2; % 外力振幅 [N]
wind_axis = 3;        % 1=x、2=y、3=z（世界座標）
```

`wind_mode` は代入する行を1つだけ有効にします。
この設定をOFFにしても、4入力励振やMEC補償はOFFになりません。

| 波形 | 現在の有効区間 | 詳細を変更する場所 |
|---|---|---|
| `sweep` | シミュレーション時刻7～47秒 | 同ファイル末尾 `wind_force_sweep_local` の `t_start`、`T_sweep`、`f_start`、`f_end`、`T_ramp` |
| `step` | シミュレーション時刻10～20秒 | `case 'step'` 内の `t>=10 && t<20` |
| `none` | なし | `agent.plant.external_force_world = []` が設定される |

外力の時刻は**飛行開始からの経過時間ではなく、シミュレーション全体の時刻**です。
手動で `f` に入る時刻を変えると、外力と飛行区間の重なり方も変わります。
自動試行と同じ重なりにするなら、時刻7秒付近で `f` に入ります。

### 3.2 推力・roll・pitch・yawへの入力励振を切り替える

`mec_dataset_settings.m` で変更します。`mainGUI.m` の `Setting.mec_dataset.enabled=true` が前提です。

```matlab
setting.excitation_enabled = true;  % 4入力励振ON
% setting.excitation_enabled = false; % 4入力励振だけOFF
setting.amplitude = [0.5;0.008;0.008;0.006];
```

振幅の並びは **`[推力 N; rollトルク N m; pitchトルク N m; yawトルク N m]`** です。
姿勢角へ直接角度を加える処理ではありません。

| 比較したい条件 | `setting.amplitude` の例 |
|---|---|
| 4入力すべて | `[0.5;0.008;0.008;0.006]` |
| 推力だけ | `[0.5;0;0;0]` |
| rollだけ | `[0;0.008;0;0]` |
| pitchだけ | `[0;0;0.008;0]` |
| yawだけ | `[0;0;0;0.006]` |
| 全軸OFF | `[0;0;0;0]`、または `excitation_enabled=false` |

```matlab
setting.frequency_range = [0.1,2.0]; % 各軸の周波数範囲 [Hz]
setting.component_count = 5;          % 各軸に加える正弦波の本数
setting.ramp_time = 2;               % 立ち上げ・立ち下げ時間 [s]
setting.flight_time = 40;            % 軌道と励振を定義する飛行時間 [s]
```

各軸で異なる位相・周波数を使い、同じシードなら同じ励振になります。
励振は `f` に入ってからの経過時間で動き、離陸・着陸中は0です。
`flight_time` を変えても、世界座標の外力の区間や、全体の `te` と同じ設定にはなりません。
手動操作では全体の `te` も `SimMEC_K.m` で確認してください。

### 3.3 MEC補償だけを切り替える

収集モードでは `mec_dataset_settings.m` を変更します。

```matlab
setting.use_compensation = false; % MEC補償OFF：HLC＋励振
% setting.use_compensation = true;  % MEC補償ON：HLC＋MEC＋励振
```

MEC補償をOFFにしても、HLC、誤差 `e` の計算、入力差 `Δu` の記録は残ります。
OFFの場合はゲインファイルも読み込みません。

通常の円軌道モードでも補償を止めたい場合や、収集設定より優先して指定したい場合は、
`SimMEC_K.m` の次のコメント行を有効にします。

```matlab
mec_param.use_compensation = false; % どちらのモードでもMEC補償OFF
```

この行をコメントアウトすると、収集モードでは `setting.use_compensation` に戻り、
通常モードでは既定のMEC補償ONに戻ります。
明示的にONへ固定する場合は `mec_param.use_compensation = true` とします。

### 3.4 プラントのモデル誤差を切り替える

現在、`SimMEC_K.m` には次の上書きがあります。

```matlab
plant_model.param.param(6) = 0.185; % jx
plant_model.param.param(7) = 0.185; % jy
```

この2行を両方コメントアウトすると、現在の既定値である `jx=jy=0.06` に戻り、
この2成分の慣性のモデル誤差がなくなります。片方だけコメントアウトすれば、その軸だけ元に戻ります。
世界座標の外力・入力励振・MEC補償は、それぞれの設定のままです。

### 3.5 使用するMECゲインを変える

`SimMEC_K.m` に用意した次のコメント行を有効にし、ファイル名を指定します。

```matlab
mec_param.gain_file = 'new_gain.mat'; % K_full：4行26列
```

コメントアウトすると、正則化・検証済みの既定ゲインへ戻ります。補償OFFにはなりません。
パス上にないファイルは、絶対パスを指定してください。
今回の誤差定義で収集したデータからゲインを再設計し、旧定義のデータとは混ぜないでください。
ゲインの再同定と非線形検証は、リポジトリルートから `train_mec_gain` を実行します。
詳細は [ゲイン設計の説明](GAIN_DESIGN.md) を参照してください。

## 4. コメントアウトしてよい箇所・使うべき切替

| 操作 | 結果 |
|---|---|
| `Setting.fAutoTrials=false` | 自動試行だけOFF。GUI・手動実行は残る |
| `plant_model.param.param(6/7)=...` をコメントアウト | 該当する慣性パラメータの上書きだけOFF |
| `mec_param.use_compensation=...` をコメントアウト | 明示指定を解除し、モードごとの補償設定へ戻る |
| `mec_param.gain_file=...` をコメントアウト | 既定ゲインへ戻る。MEC補償のON／OFFは変わらない |
| `setting.excitation_enabled` の行を削除／コメントアウト | OFFにはならない。省略時は励振ONとして扱われるため、`false` を指定する |
| `Setting.mec_dataset.enabled=false` | 収集モード自体がOFF。円軌道・励振なし・通常ログのみへ切替 |

`nominal` や `mec` の登録行、`agent.cha_allocation.controller` は残してください。
MECモジュール自体を削除すると、誤差 `e` や入力差のログもなくなり、収集・描画に使えなくなります。
`obj.result.input` や `delta_u` の計算式をコメントアウトしてON／OFFする必要はありません。

## 5. 保存・確認・同定への受け渡し

**Save data** は、最初に通常ログを `Data/Sim_data` へ保存します。
収集モードなら、続いて飛行中の有効な隣接データを `datasets/MEC_4input` へ書き出します。
保存名の欄は通常ログの接頭辞です。学習用ファイルはシードと日時で命名します。
同じ試行で保存を繰り返すと日時の異なるファイルが増えるので、同定時に同じ試行を重複して選ばないでください。

```matlab
app.dataset_files % このappで保存できた学習用ファイルの一覧
```

4入力の独立性が不足する場合などは、警告とともに学習用ファイルの保存を見送ります。
**その場合も通常ログは保存済み**です。励振OFF・1軸だけの比較実験は、通常ログで確認できます。
4入力モデルの学習用には、各入力を独立に励振したデータを使ってください。
回帰行列のランク不足は別途警告されます。

| 図 | 確認できる内容 |
|---|---|
| GUI内の位置グラフ | 推定位置と目標軌道 |
| Figure 6 | 最終的な推力・トルク入力 |
| Figure 7 | 学習用入力差 `Δu`：MEC補償＋励振 |
| Figure 8 | 推力のノミナル値・入力差・最終値 |
| Figure 9 | 3次元軌道 |
| Figure 11 | 世界座標の外力 |
| Figure 13・14 | ノミナル側と実機側の26観測量 |
| Figure 15 | 4入力励振そのもの |

同定には `main_ERROR_KoopmanLinearByData_tutorial.m` を使います。
`datasets/MEC_4input` にMATファイルがあれば、そのフォルダを優先します。
学習用と検証用でシードを分け、不要な試行は同定対象フォルダへ混ぜないでください。
データの式と保存形式は [DATASET.md](DATASET.md) にあります。

## 6. 自動収集へ戻す場合と設定の優先順位

`mainGUI.m` の `Setting.fAutoTrials=true` に戻すと、自動試行を再開します。
収集モードの自動試行では `setting.seeds` を順に使います。

```matlab
% GUIを手動操作せず、指定したシードを自動で収集する別の入口
files = collect_mec_dataset(1:10, ...
    'amplitude',[0.5;0.008;0.008;0.006], ...
    'use_compensation',false);
```

- `mainGUI` は `mec_dataset_settings()` を読み込んでから、`mainGUI.m` 内の代入で上書きします。
  `enabled` の切替は `mainGUI.m` で行ってください。
- `collect_mec_dataset(...)` は常に自動収集です。`Setting.fAutoTrials` は参照しません。
- この関数の引数 `amplitude`、`flight_time`、`use_compensation` は、関数側の既定値も含めて設定ファイルの値を上書きします。
  これらを変更する場合は呼び出し時に指定してください。
- `SimMEC_K.m` に `mec_param.use_compensation` を明示すると、その指定が最優先になります。
- 世界座標の外力と慣性の上書きは、手動・自動のどちらも `SimMEC_K.m` の設定を使います。

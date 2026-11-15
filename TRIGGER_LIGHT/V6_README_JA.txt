V6: 元HLC 6秒基準 + 未来情報を借りない4秒予測候補（オフライン監査）

【実行】
cd('C:\Users\student\Documents\GitHub\common_matlab'); addpath(genpath(pwd));
cd('TRIGGER_LIGHT'); clear classes; rehash;
R=RUN_PURE_TRIGGER_V6_CAUSAL_AUDIT(false);
disp(R.status);
if isstruct(R.causalForecast) && isfield(R.causalForecast,'records')
    disp(struct2table(R.causalForecast.records));
    disp(struct2table(R.causalForecast.noiseRecords));
end

【必ず確認】
- MATLAB本番実行は提供環境で未実施。PASSはユーザーPCの結果で判断する。
- 元24状態RHS、元HLC、元WAYPOINT_REFERENCE、その他元ソースがMATLABパスに必要。
- TEST_TRIGGER_V6_HLC_STRESS_6Sは元TEST_TRIGGER_V4_HLC_STRESSを上書きしない。
- 原HLC+24状態ODEを6秒×4ケースのオフライン比較基準として計算。途中t=1秒とt=2秒からそれぞれ4秒を比較するため6秒が必要。1秒からの窓には初期索振動のピーク1.30秒が含まれる。
- 予測器は当該t=1/2秒までの1秒の荷位置と目標位置履歴、現在のGovernor 6状態・保持コマンド、既知の経路、HLCゲイン、推定質量だけを使う。
- 未来の真の荷位置・未来HLC入力・記録済み未来Governor軌跡を予測器へ渡さない。
- ただし履歴はこのテストでは元HLC名目シミュレーションに由来し、実EKF履歴ではない。
- ゲインと履歴から高次誤差状態を逆推定するため雑音に敏感。実EKFによる誤差検証が必要。
- 元の6次・2次HLC誤差モードは無飽和、張り索、特異点のないフィードバック線形化成立時のみ。
- オンライン側では未来4秒の推定質量を固定（当該周期の現在の推定値）する仮定。真値は使わない。
- 経路の28要素jetは既知の元ルートからオフライン表を作り、pchipで補間。補間誤差の証明なし。原評価との偏差をログで評価。
- 軽量予測出力のrhoDrone/rhoLoad/accDrone/accLoadはNaNのまま。
  物理的誤差上界なしに幾何のCLEAR判定へ渡してはならない。
- 全方式を本番同時実行する構造ではなく、幾何比較は別計画。
- 候補の時間は幾何、履歴の収集、キャッシュ更新、EKF、CBF、Flightを含まない。
- 実Flightでの合流比較、飽和4秒、総1～2ms、25msは未検証。

【診断】
- `R.causalForecast.records` = 各ケースの未来4秒の荷/機体最大位置誤差、Governor誤差、計算時間、モード復元行列条件数。さらに実装内訳: モード復元、160周期モード+Governor、参照補間、索方向再構成。計測器自身のオーバーヘッドを含む。
- `R.causalForecast.noiseRecords` = 過去位置に0.1mm/1mmの合成ノイズを入れた誤差感度。実EKF測定値ではない。
- `R.oneStepSaturation` = `RUN_PURE_TRIGGER_V6_CAUSAL_AUDIT(true)`の場合にだけ追加。一歩の飽和調査であって4秒の飽和検証ではない。
- ZIPは TESTS/RESULTS に時刻付きで保存され、既存ファイルを上書きしない。

【評価基準】
(1) 4ケースで成功、未来軌跡情報が予測器に混入しない。
(2) 途中の最大位置誤差と偽陽性/偽陰性を調べる。本テストにはまだ障害物回避の偽陰性・偽陽性判定はない。
(3) 事後に1～2msの予算（予測+選択1幾何+前処理）を確認する。
(4) 無資格状態を偽のCLEARにすることは禁止。

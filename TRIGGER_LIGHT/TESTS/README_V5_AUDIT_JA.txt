V5 TRACE AUDIT (2026-11-13)

目的:
4秒の元HLC名目ロールアウトを、再計算せずに実測の通過点・
振れ角・仮想時間・入力変化で監査する。元HLC、24状態RHS、CBF、
生産用TRIGGER_LIGHT/SIX_WAYのソースは変更しない。

MATLAB:
  cd('C:\Users\student\Documents\GitHub\common_matlab')
  addpath(genpath(pwd))
  A=load('TRIGGER_LIGHT/TESTS/RESULTS/HLC_STRESS_20261113_001524.mat');
  X=TEST_TRIGGER_V5_TRACE_FACTS(A.R);
  % ファイルが保存済みの場合のみ、任意で:
  % X=RUN_TRIGGER_V5_TRACE_AUDIT(fullfile(pwd,'TRIGGER_LIGHT','TESTS','RESULTS',...
  %   'HLC_STRESS_20261113_001524.mat'));

本ZIP RESULTS/INDEPENDENT_V5_4S_TRACE_AUDIT.csv:
別途ユーザ提供MATに基づきPythonで行った数値試験。
元HLCの出力誤差ダイナミクス(x,y=6次、z=2次)について、
「直前1秒の位置誤差だけ」を最小二乗にフィットしたモード予測を比較。
ただし未来の参照軌道は元の4秒結果から与えたORACLE。
現時点で使えるオンライン予測器ではない。
未来1mmノイズ試験は仮定した合成ノイズ。EKF誤差の保証ではない。
元データが4秒分しかないため時刻t=1から3秒、t=2から2秒先しか
検証できない。任意現在時刻から4秒はまだ未検証。

決して「4秒軽量予測PASS」「実運動保証PASS」と解釈しない。
元HLCの未来入力一定仮定は試験にも本番にも使わない。
この監査の目的は、6次モードの同定可能性と誤差感度の検証。

次工程:
(1) 6秒の元HLCストレス履歴を4条件で取得し、t=2秒から先4秒を
    現在時点までの履歴＋未来時点に未知な参照値を使わず比較する。
(2) 元Governorの現在6状態と経路の既知定義から未来参照を推定し、
    将来の吊り荷誤差を使いGovernor速度を更新する。元の離散化係数と
    1周期の同値性を試験する。
(3) 入力飽和枝、誤差の多い大きな逸脱、履歴EKFノイズの条件を試す。
(4) 一方式幾何と合算で 1--2 ms、さらにFlight 25 msを計測する。
(5) 明確な誤差包絡が無いままSAFE保証や見逃しゼロを主張しない。

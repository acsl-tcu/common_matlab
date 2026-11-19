# REAL / HOCBF_ZOH_THRUST — 初回生成・相対次数監査

## 目的

吊り荷付きクアッドロータの実入力 `u = [T; tau_x; tau_y; tau_z]` を使うCBFのため、**ユーザーの MATLAB 上で**モデル・安全形状・時間微分を再生成／検査します。

- ドローン：球
- 吊り荷：球
- ケーブル：有限長ケーブルを包絡する1個の回転楕円体
- 障害物：`DRONE_OBSTACLES(...,'single_ellipsoid',...)` の楕円体（中心・姿勢・軸長を使う）
- 地面：`z=groundZ`。機体球・ケーブル楕円体・吊り荷球ごとに別制約
- **この手法だけ**：ZOH区間で実推力・実トルクを一定として時間Taylor展開（`dot(T)=0`）。`COMMON` にはこの仮定を入れていません。

**最初からHLCに接続する完成済みCBFではありません。** 相対次数の要求と物理式の一致を調べ、誤った6階CBFを出さないための段階です。

## フォルダ

```
CBF_GENERATOR/
  COMMON/
    CBF_COMMON_MODEL.m          # 与えられたモデル生成式と同じ19状態の一般物理モデル
    CBF_COMMON_GEOMETRY.m       # 全方式で共通利用できる物理包絡形状
  REAL/
    HOCBF_ZOH_THRUST/
      generate_CBF_REAL_HOCBF_ZOH.m         # 実行入口
      CBF_REAL_ZOH_JET_SOURCE.m             # 微分アルゴリズムのソース
      check_CBF_REAL_LOAD_BETA4_SYMBOLIC.m # 吊り荷4階作用の記号式監査
      GENERATED/                            # 実行時に生成
      VALIDATION/                           # 実行時に結果を記録
  VIRTUAL/                                   # 今回は触れない
```

## MATLABでの実行

`common_matlab/model/substance/CBF_GENERATOR/REAL/HOCBF_ZOH_THRUST/` を開いて：

```matlab
result = generate_CBF_REAL_HOCBF_ZOH();
disp(result.status);
disp(result.reportPath);
```

`GENERATED/CBF_REAL_ZOH_DIAG_EVAL.m` は、この実行時に生成する**診断専用**の微分評価関数です。HLCから直接使わないでください。

`VALIDATION/RELATIVE_DEGREE_REPORT.txt` と `RELATIVE_DEGREE_SENSITIVITIES.mat` に、必要次数・検出次数・感度・モデル検査が保存されます。Symbolic Math Toolbox がある場合は `LOAD_BETA4_SYMBOLIC_PROOF.txt` も出力します。

**赤文字（`fprintf(2,...)`）で相対次数NGを表示します。** その場合、安全フィルタと称した `CBF_REAL_ZOH_HOCBF_INEQUALITY.m` は生成しません。

## 正確なモデルと状態順序

出典：アップロードされた `generateModel_SupendedLoad.m` の、階層型線形化用の `x=[q;ob;pl;dpl;pT;ol]` に対応。

```
x(1:4)   q      : 機体四元数（実数部先頭、単位長）
x(5:7)   omega  : 機体角速度
x(8:10)  pL     : 吊り荷位置
x(11:13) vL     : 吊り荷速度
x(14:16) pT     : 機体→吊り荷方向の単位ベクトル
x(17:19) omegaL : ケーブル角速度
```

機体位置は `pD=pL-cableL*pT`。`P.m`、`P.mL`、`P.J=[jx;jy;jz]`、`P.cableL`、`P.gravity`、必要なら `P.dstx/P.dsty` を受け取ります。

ジェネレータ内の数値パラメータ・姿勢は**相対次数診断のテスト点**であり、運用用の機体パラメータではありません。HLCへつなぐ際には実際の推定状態・パラメータに置き換えてください。

## なぜ吊り荷の希望次数6を停止するか

**要求**：トルクに対して機体4、ケーブル4、吊り荷6。

物理モデルで吊り荷の並進加速度は

```
aL = -g*e3 + ((T*(pT'*R*e3)-m*L*norm(pTdot)^2)/(m+mL))*pT
```

です。固定障害物に対する吊り荷球の楕円体分離バリア

```
hL = (pL-c)'*M*(pL-c) - 1
```

を使うと、トルクに対する4階係数は一般状態で

```
d hL^(4) / d[tau_x;tau_y;tau_z] =
  (2*T/(m+mL))*(pT'*M*(pL-c)) * ...
  [ -pT'*R(:,2)/Jx,  pT'*R(:,1)/Jy,  0 ]
```

であり、通常ゼロではありません。ジェネレータはこれを独立に計算し、時間Taylor微分の結果と照合します。**推力一定はこの係数を消しません**。ホバリングなど1点でこの係数が消えても、周辺全体の相対次数6を意味しません。

推力`T`は通常2階から作用します。したがって「実4入力を同時に決定する」場合は混合次数かつ `T*tau` などの積が現れ、単純な線形制約QPと決めつけることはできません。

## 安全形状の考え方

Model D と同じ、ケーブル長 `L`、半長 `L/2`、膨張半径 `r=cableRadius+safetyMargin`、`eta>0` による**外接楕円体**を使います：

```
aLong^2  = (1+eta)*(L/2)^2 + (1+1/eta)*r^2
aShort^2 = (1+1/eta)*r^2
Qc       = aShort^2*I + (aLong^2-aShort^2)*pT*pT'
```

球または楕円体の形状行列 `Qs` と障害物楕円体の `Qo` から、ミンコフスキー和を包含する楕円体を

```
Qouter = (1+etaOuter)*Qs + (1+1/etaOuter)*Qo
h = (centerSys-centerObs)'*(Qouter\(centerSys-centerObs)) - 1
```

として構成します。**`h>0` なら両楕円体は交差しません**（外側の保守的近似）。逆方向、すなわち `h<0` は衝突確定を意味しません。

障害物の現在位置・姿勢は与えた `parts` で固定して微分します。動的障害物の時間微分は**未対応**です。模擬センサで「今見えた障害物」に動的障害物が含まれるなら、将来その運動を安全式に含める必要があります。

## この段階の確認範囲・未実装

1. モデル生成式と0次Taylor微分 `dx/dt` の一致。
2. 3つの一般姿勢で、機体・ケーブル・吊り荷と地面の各バリアについて、トルク作用がどの階数に最初に現れるかを検査。
3. 推力の最初の作用階も別途検査。
4. 吊り荷の4階トルク係数を独立解析式と比較。
5. Symbolic Math Toolbox がある場合は上記係数が恒等的ゼロかを記号的に検査。

**未実装**：HOCBFによる制約生成（希望の相対次数が不一致なため）、4実入力の最適化、不可行時処理、HLCへの接続、V81_CARRY、障害物の速度・加速度・高階微分、25 msサンプル間の安全証明、センサ遅延・状態推定誤差。

## 次の設計判断

当該バリアでの混合次数4/4/4と退化への対処を採用するか、吊り荷に別の安全関数・動的拡張・Rectified CBF / safe backstepping 等を用いて6階構造を理論的に成立させるかを決める。**4/4/6の名目だけを残して安全と主張する実装はしない**。

3次元 吊り荷付きドローン 12構成・高次CBF試作 V2
===============================================

【前提／今回できた範囲】
前版の12構成の幾何安全関数 h(x) を維持し、4次・6次HOCBFの候補不等式
    qp.A * nu >= qp.b
を実行時に作る機能を追加しました。

生成はパラメータなし：
    generateCBF_RealInput();
    generateCBF_VirtualInput();

生成時に質量・長さ・半径・障害物形状・CBFゲイン・目標軌道の数値は固定しません。
生成結果の保存先：
    common_matlab/model/substance/generate_cbf_suspended/CBF/real/
    common_matlab/model/substance/generate_cbf_suspended/CBF/virtual/

テストは common_matlab 直下に配置し、ここから実行：
    test_CBF_All();
高次CBFのみ：
    test_CBF_HOCBF();
元のgenerateModelとgenerateControllerはこの配布物に含めていません。

【使い方】
    s = CBF_RuntimeSettings_Template();
    % すべての物理量と形状、CBFゲイン等を実行時に設定する。
    % 設定項目は CBF_RuntimeSettings_Template.m を参照。
    % テストでのみ test_CBF_Fixture() の仮パラメータを使用。
    obs.center = [ ... ; ... ; ... ];
    obs.axes = [ ... ; ... ; ... ];
    obs.R = ...;              % 正規直交でdet=+1の3x3回転行列
    [h, info, qp] = CBF_virtual_C_ellipsoid(xVirtual,s,obs);
    % h : 3D幾何安全関数
    % info : ラベル、ケーブル球数、形状等
    % qp.A, qp.b: qp.A * nu >= qp.b
    % qp.order: 行ごとの設計階数
    % qp.psi: h, (D+k)h, ..., (D+k)^(r-1)h の初期値
    % qp.degenerate: 決定入力係数の退化
    % qp.uncontrollableViolation: 係数が消失し、その状態の制約が満たせない行
    % qp.relativeDegreeAtState: 退化した行はNaN
    % qp.modelDiagnostics: 張力等のモデル成立診断

    % 入力を柔らかくするスラックは導入しない。
    % QP解法はquadprog(Optimization Toolbox)があるときのみ実行可能。
    W = diag([ ... , ... , ... , ... ]);
    nuNom = [ ... ; ... ; ... ; ... ];
    result = cbf_solve_hocbf_qp(qp,nuNom,W);
    % result.exitflag<=0ならQPが解けたとみなさない。

【状態と決定入力を厳密に区別】
実入力状態 xReal (29):
  [qBody(4); omegaBody(3); pLoad(3);vLoad(3); qCable(3);omegaCable(3);
   T;dotT;ddotT;T^(3);tau(3);dotTau(3)]
  nuReal=[T^(4);tauX^(2);tauY^(2);tauZ^(2)]
  運動方程式: アップロードされたgenerateModelの19状態モデルに
  Tの4重積分器とtauの2重積分器を追加。
  幾何CBFの設計階数: すべて6。Tとtauの上下限は4/2次の追加行。
  ※ 具体的な状態によって入力係数はゼロとなる。全方向の相対次数が
     常に6であるという意味ではない。

仮想入力状態 xVirtual (20):
  [pLoad;pLoad';pLoad'';pLoad^(3);pLoad^(4);pLoad^(5); yaw; yaw']
  nuVirtual=[pLoad_x^(6);pLoad_y^(6);pLoad_z^(6);yaw'']
  張りケーブルの関係 qCable=-(pLoad''+g*e3-distXY)/norm(...)
  機体・ケーブルの設計階数4、吊り荷6。
  ※ HLCのv1を4階動的拡張し、既存HLCの入力逆変換と一致させる
     追加の接続が必要。今回のxVirtualは理想平坦出力系です。

【名目HLCと入力制約】
現在の実入力用QPのnuRealは、HLCが出す直接の[T;tau]とは違う。
HLCの実入力から連続性を保ったnuRealの名目値を作る仕組みは次段階。

仮想側のnuVirtualもHLCの誤差入力[v2;v3;v1^(4);v4]そのものではない。
軌道微分を含めた絶対値である（pL^(6),yaw''）。
仮想側の実トルク制限は、実HLCの非線形逆変換から得た
  tau = tauOffset + tauMap*nuVirtual
を使用して、cbf_attach_virtual_input_limits(qp,Tnow,tauOffset,tauMap,settings)
で追加可能。ただしTnowの現在値検査だけでは将来の推力制限を保証できない。
実HLCの逆変換行列・特異性・拡張状態を含む完全な接続は未実装。

【安全保証の限界と適用範囲】
・障害物は静止した球または任意姿勢の回転楕円体（SO(3)）。移動・回転する
  障害物の時間微分は未実装。毎周期障害物姿勢が変わる場合、QP行は不正確。
・3方式C/B/Dは前版の十分条件（安全側の支持関数分離）を維持。
  真の最短距離と一致せず、カプセル最近接点の切替を回避する代わりに
  保守的な条件となる。
・障害物姿勢が任意でも数式は評価可能だが、どんな初期状態・目標軌道でも
  QPが実行可能、安全性が成立するわけではない。
・張った質量ゼロケーブル・正張力・単位クォータニオンがモデルの仮定。
・高次CBFはqp.psiの全階層の初期非負性、モデル一致、QP可行性が必要。
・連続時間でのHOCBF不等式。離散時間実行時の誤差・保証は未実証。
・実時間性能・閉ループ回避・安全性の保証は未検証。
・物理値・ゲインは生成時に決めず、実行時settingsで渡す。

【高次微分の方法】
時間Taylor係数x_k = x^(k)(0)/k! を基礎に、非線形モデルを逐次展開。
非線形演算（積・内積・外積・平方根・逆数）も級数で行うため、
有限差分の刻み幅選択や巨大なSymbolic式展開は不要。
決定入力0および4つの単位ベクトルを代入し、6次（または4次）係数の
差から A を取り出す。低次に入力が現れていないかも実行時に診断する。
これは数値的に係数を取り出す試験実装であり、低次入力の不在を全状態で
記号的に証明したことにはならない。

【試験後に追加で必要なもの】
1. 元のFLxyDst/GLxyDstとの19状態の一致（既存の任意試験を利用）。
2. 既存HLCのUf、Beta2、V2_alpha2、Vfd、Vsとの接続検証。
3. xVirtualの実状態からの抽出（特にpLoad^(3)～pLoad^(5)）。
4. 仮想側の実推力上下限を保証する追加の状態安全条件。
5. QP実行不能・入力退化時の対応と、将来の可行性維持法。
6. 時間計測と生成係数の高速化。

READMEは生成後の使い方の説明であり、安全保証を主張するものではありません。

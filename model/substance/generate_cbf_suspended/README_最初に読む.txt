3次元吊り荷ドローン CBF 12構成の生成基盤（読みやすさ重視版）
更新日: 2026-11-12

■ このZIPの目的
  - MATLABの既存generateModel_SupendedLoad.m / generateController_SuspendedLoad.mは変更しません。
  - 実入力と仮想入力を別々の生成ファイルに分けます。
  - 機体側3方式（C,B,D）×障害物2方式（球,任意姿勢楕円体）×入力2方式=12構成。
  - まず共通の幾何安全関数を準備する段階です。完成したCBF-QPコントローラではありません。
  - 数値テストは commonmatlab 直下にあります。

■ commonmatlab にコピーするもの
  本ZIPのcommonmatlab/以下のファイル・フォルダを、手元のcommonmatlab直下へコピー。
  既存generateModel / generateControllerは含めていません。

  commonmatlab/
      CBF_initialize.m                  パスを設定
      CBF_Settings_Example.m            外部設定例（自分の数値に変更）
      generateCBF_RealInput.m           実入力の6構成生成
      generateCBF_VirtualInput.m        仮想入力の6構成生成
      demo_CBF_Usage.m                  外部設定・障害物・関数呼び出し例
      test_CBF_All.m                    まとめて試験
      test_CBF_CableSpheres.m           球数・被覆の試験
      test_CBF_Geometry.m               12構成の形状試験
      test_CBF_GenerateFiles.m          生成スクリプトの試験
      test_CBF_OptionalOriginalModel.m  元のFL/GLとの比較（任意）
      cbf_common/                        共通の計算部品（極力小さく分割）
      CBF_generated/real/               生成済み・再生成される実入力用関数
      CBF_generated/virtual/            生成済み・再生成される仮想入力用関数

■ 最初に実行する3行
  cd('あなたのcommonmatlabのパス')
  CBF_initialize
  test_CBF_All

■ 実際の設定を使って生成
  settings = CBF_Settings_Example();
  settings.physical.massDrone = （あなたの機体質量）;
  settings.physical.massLoad = （あなたの吊り荷質量）;
  settings.physical.cableLength = （あなたのケーブル長）;
  settings.shape.droneRadius = （外接球半径）;
  settings.shape.loadRadius = （吊り荷外接球半径）;
  settings.shape.cableRadius = （実ケーブル半径）;
  settings.cable.maxExtraRadius = 0.01;  % 許容するケーブル被覆の追加半径
  generateCBF_RealInput(settings)
  generateCBF_VirtualInput(settings)

  注：CBF_Settings_Example() の数値は動作説明用。過去に受け取った飛行用パラメータ値ではありません。
      外部設定を渡さないと生成できない設計です。

■ 複数球 C の球数は可変
  settings.cable.numberOfSpheres = [];   % 空 [] なら自動
  settings.cable.maxExtraRadius = 0.01;   % 物理ケーブルからの追加半径許容[m]

  ケーブル長L,物理半径rc,球数n（端点を含む）とすると、等間隔配置時の
  間隔 d=L/(n-1)、安全側被覆球半径は rs=sqrt(rc^2+(d/2)^2)。
  自動設定では rs-rc <= maxExtraRadius を満たす最小のnを選ぶ。

  settings.cable.numberOfSpheres = 20; % 直接20個にしたいとき
  ただし20個では被覆半径の許容増分を超えるとエラー。

  これは「指定被覆誤差に対して等間隔球数が最小」の意味であり、
  QPの制約数・実行時間まで含む最適な球数ではない。数値比較で最終調整。

■ 障害物は全て外部から受け取る
  球:
     obs = struct('center',[1;0;1.2],'radius',0.3);
     [h,info] = CBF_real_C_sphere(xReal,settings,obs);

  任意姿勢楕円体:
     obs = struct('center',[1;0;1.2],'axes',[0.2;0.4;0.6], ...
                  'R',Rworld);
     [h,info] = CBF_virtual_B_ellipsoid(xVirtual,settings,obs);
     Rworld は det=1 の正規直交3x3回転行列。オイラー角は任意の順序で
     自分でRworldに変換して渡す（回転順序の誤解を防ぐため）。

  混在する複数障害物:
     obs1=struct('type','sphere','center',[...],'radius',...);
     obs2=struct('type','ellipsoid','center',[...],'axes',[...],'R',...);
     [h,labels] = cbf_evaluate_scene('real','B',xReal,settings,{obs1,obs2});

■ 安全関数の中身
  機体・吊り荷: 球
  C: ケーブル全区間を等間隔球で安全側に完全被覆
  B: カプセルの支持関数による滑らかな保守的分離
  D: カプセルを包含する細長い楕円体の支持関数で保守的分離
  BとDは最近接点切替を避ける代わりに真の最短距離より保守的。
  h>=0 は非衝突の十分条件であり、その実現可否や将来の安全性を保証しない。

■ 実入力と仮想入力は「状態が異なる」
  実入力拡張状態 xReal(29):
    [機体クォータニオン4;機体角速度3;吊り荷位置3;吊り荷速度3;
     ケーブル方向3;ケーブル角速度3;推力Tと1〜3階微分4;
     トルク3とトルク1階微分3]
    QP候補は [T^(4);tau_x^(2);tau_y^(2);tau_z^(2)]。

  仮想入力理想状態 xVirtual(20):
    [pL;dpL;ddpL;pL^(3);pL^(4);pL^(5);yaw;dyaw]
    QP候補は [pLx^(6);pLy^(6);pLz^(6);ddYaw]。
    このままでは既存HLCを置き換えるコントローラではない。
    既存v1とその4階拡張を整合的につなぐことが次のステップ。

■ シンボリック導出について
  settings.generate.exportSymbolicGeometry=true にすると、
  現在の球数でシンボリック安全関数を.matlabFunctionで書き出す。
  このシンボリック出力は生成時点の球数に固定される。
  球数設定を変更したら再生成が必要（通常の数値幾何関数は自動追従）。

  exportOneHOCBFRow=true は高次Lie微分を1個だけ試験的に生成する。
  低次に入力が出てしまうとエラーで停止する（不正な相対次数を許さない）。
  非線形モデルの高階Lie微分は計算量が大きい。初期試験はOFF推奨。

■ 未完成の点・必要な次工程
  1) 高次CBF係数A,bの相対次数・退化の理論解析と数値チェック
  2) 既存HLCの仮想入力v1微分・逆変換への接続
  3) 実入力方式での拡張状態積分器と名目HLCとの接続
  4) 推力・トルク限界を含むハードQP、地面制約のHOCBF化
  5) QP可行性・前方不変性・閉ループ3次元回避の検証
  6) ケーブル弛みには本モデルは未対応（張り仮定）。

■ 実行環境について
  このZIPはMATLAB環境では実行していません。数値のケーブル被覆式と
  楕円体/カプセルの包含関係は独立に数式・数値で確認しましたが、
  最初はお手元のMATLABで test_CBF_All を実行してください。

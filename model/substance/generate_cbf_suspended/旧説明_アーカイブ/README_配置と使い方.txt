3D CBF生成基盤：高次CBF係数対応版・実行時パラメータ方式
=============================================

1. 正しい配置
-----------------------
common_matlab/
  test_CBF_All.m                  ← テストはここに置く
  test_CBF_GenerateFiles.m
  test_CBF_CableSpheres.m
  test_CBF_Geometry.m
  test_CBF_RuntimeParameters.m
  test_CBF_SetupPaths.m
  test_CBF_Fixture.m              ← テスト数値のみ
  test_CBF_OptionalOriginalModel.m
  model/
    substance/
      generate_cbf_suspended/
        CBF_initialize.m
        generateCBF_RealInput.m
        generateCBF_VirtualInput.m
        CBF_RuntimeSettings_Template.m
        demo_CBF_Usage.m
        cbf_common/
          ...共通の数式計算関数...
        CBF/                    ← 生成時に新設される
          real/                 ← 実入力のCBF本体6件
            CBF_real_C_sphere.m
            CBF_real_C_ellipsoid.m
            CBF_real_B_sphere.m など
          virtual/              ← 仮想入力のCBF本体6件
            CBF_virtual_C_sphere.m など

※ 元のgenerateModel / generateController は同梱しません。
※ commonmatlabとcommon_matlabのフォルダ名の違いはテスト内で自動吸収します。
※ CBF_initialize は generator側に置いてあり、テストを始める前に呼ぶ必要はありません。
※ 高次CBFの詳細は README_高次CBF_試験手順.txt を先に参照してください。

2. 初回テスト
-----------------------
MATLABのカレントディレクトリを共通MATLABフォルダへ変更し、
    test_CBF_All();
これだけでパス登録→CBFファイル生成→幾何安全関数と4次/6次HOCBF係数の試験を実行します。

3. 生成方法（生成時の数値設定は不要）
-----------------------
    test_CBF_SetupPaths();
    generateCBF_RealInput();
    generateCBF_VirtualInput();

4. 生成後の関数の使い方
-----------------------
    settings = CBF_RuntimeSettings_Template();
    % settings.physical.massDrone等の必要な全項目を実行時に指定
    % 規格化ケーブル方向・状態xの大きさは実際のシステムに合わせる
    % 球障害物なら obstacle.center=[...]; obstacle.radius=...;
    [h,info,qp] = CBF_real_C_sphere(xReal,settings,obstacle);

    % 任意姿勢楕円体ならobstacle.axes=[a;b;c], obstacle.R=3x3回転行列
    [h,info,qp] = CBF_virtual_B_ellipsoid(xVirtual,settings,obstacle);
    % qp.A * nu >= qp.b  （実入力/仮想入力でnuの定義が異なります）

5. 外部パラメータに関する注意
-----------------------
- ケーブル長・物理半径・追加膨張量・機体/吊り荷半径は生成時に固定しません。
- 球数は「生成後の関数実行時」にケーブル長・許容追加膨張から自動選択します。
- settings.cable.numberOfSpheres=[] は自動、整数を指定すると手動です。
- 出力hの長さはC方式の球数とともに変化します。制御器側では可変長として扱う必要があります。
- テストにある数値は機能検証専用です。生成関数からは参照しません。
- 例示の寸法・入力限界を実装で自動採用する機能はありません。

6. 現時点の数学的な完成範囲
-----------------------
CBF_real/CBF_virtualという名称ですが、出力は幾何安全関数hとメタ情報です。
4次/6次HOCBFのLie微分、QPの解決、閉ループ前方不変性の保証は未完成です。
以前の版の「HOCBFを1行だけ自動生成する」実験設定は、数値球数を固定しない
今回の要求と混同しないように、今回の通常の生成入口から取り除きました。
cbf_symbolic_model.m / cbf_high_order_row.m は研究用の補助関数として残しています。
後にコントローラを生成する際は、可変球数に対応する動的制約組立てと、
各入力方式の混合相対次数・入力退化を独立に導出する必要があります。

7. パスエラー時の確認
-----------------------
MATLABで
    which test_CBF_All -all
    test_CBF_SetupPaths();
    which generateCBF_RealInput -all
    which CBF_real_C_sphere -all
を確認してください。旧版のcommonmatlab/CBF_generatedや、旧版generateファイルが
パスに残っている場合は、旧版のパスを外してください。

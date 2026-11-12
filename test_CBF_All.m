function test_CBF_All()
%TEST_CBF_ALL common_matlab直下から実行する一括テスト。
%
% 実行方法（CBF_initializeは不要です）:
%   cd('.../common_matlab');
%   test_CBF_All();
%
% テスト数値はtest_CBF_Fixtureにのみ存在し、本体生成には埋め込みません。

%% 1. パスを自動で見つける
paths = test_CBF_SetupPaths();
fprintf('\n=== CBFのテスト開始 ===\n');
fprintf('生成器: %s\n',paths.generator);

%% 2. 12個のCBF関数を生成して確認
% 最初はCBF/realおよびCBF/virtualフォルダが存在しなくてもOK。
test_CBF_GenerateFiles();

%% 3. 数値幾何とケーブルの被覆を検証
test_CBF_CableSpheres();
test_CBF_Geometry();
test_CBF_RuntimeParameters();

%% 4. 動的拡張・高次CBF係数・相対次数の試験
test_CBF_HOCBF();

fprintf('\n=== 全テスト合格（数値幾何・高次係数の候補・入力作用チェック）===\n');
fprintf('※ 閉ループ安全性・QP将来可行性・実HLC逆変換は未検証です。\n');
end

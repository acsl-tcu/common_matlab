function test_CBF_V5_Review()
%TEST_CBF_V5_REVIEW 原因・係数等価性・閉ループ状況の接続前レビュー。
% 重いベンチマークはこのスクリプトでは自動実行しない。
% 既存の test_CBF_All が通った後に実行する。
test_CBF_SetupPaths();
fprintf('\n=== V5: 診断と安全性を保持した計算整理の試験 ===\n');
test_CBF_SceneEquivalence();
test_CBF_MEX_Status();
paths=test_CBF_SetupPaths();
snapshot=fullfile(paths.generator,'CBF','test_results','real_B_sphere_failure_snapshot.mat');
if exist(snapshot,'file')==2
    test_CBF_GroundGainDiagnostics();
else
    fprintf('[SKIP] 保存されたステップ12のsnapshotがないため地面ゲイン診断は未実施。\n');
end
fprintf('[INFO] 実入力元のストレス試験はステップ12のQP実行不能が未解決です。\n');
fprintf('[INFO] 複数障害物の閉ループ試験: test_CBF_MultiObstacleRollout();\n');
fprintf('[INFO] 1/3/5/10障害物速度: benchmark_CBF_MultiObstacle(5,[1 3 5 10]);\n');
fprintf('=== V5レビュー終了（安全保証・HLC接続試験ではありません） ===\n');
end

function report=test_CBF_V7_All()
%TEST_CBF_V7_ALL V7の新規テストだけを順番に実行（HLCなし）。
% common_matlab直下から呼び出す。
test_CBF_All();
test_CBF_SceneEquivalence();
test_CBF_ExplicitDecisionBounds();
report.ground=test_CBF_RealGroundFeasibilitySweep([1.2 1.5 2.0]);
report.robust=test_CBF_RobustScenarioSweep(12);
report.trajectories=test_CBF_ReferenceTrajectoryCases(25);
fprintf('\n=== V7テスト終了 ===\n');
fprintf('注意: 失敗状態を隠さずCSVへ報告します。HLCの実入力保証は未完成です。\n');
end

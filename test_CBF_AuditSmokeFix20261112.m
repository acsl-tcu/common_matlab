function report = test_CBF_AuditSmokeFix20261112()
%TEST_CBF_AUDITSMOKEFIX20261112 最小の地面監査でtable関数の衝突解消を確認。
% 既存CBFモデル、生成器、QP本体を変更しません。
% 総合監査の前に1状態だけ実行し、CSVまで出力できるかを確認します。
test_CBF_SetupPaths();
options=struct();
options.altitudes=1.5;
options.verticalSpeeds=0;
options.pitchAngles=0;
options.cableTilts=0;
options.groundGains=1.2;
options.inputGains=1.2;
report=test_CBF_AuditGroundGrid(options);
assert(istable(report) && size(report,1)==1, ...
    'CBF:AuditSmokeRowCount','地面監査は1状態を返す必要があります。');
fixture=test_CBF_Fixture();
assert(isfield(fixture,'physical'), ...
    'CBF:AuditSmokeFixture','設定読込が不正です。');
fprintf('[PASS] 最小地面監査: table 1行、CSV保存完了。\n');
end

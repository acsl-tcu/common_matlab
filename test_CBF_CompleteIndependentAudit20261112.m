function results=test_CBF_CompleteIndependentAudit20261112(options)
%TEST_CBF_COMPLETEINDEPENDENTAUDIT20261112
% 既存V7本体・QP・generateは無変更。4段階を一括で診断する追加テスト。
%
% 使用例:
%  results=test_CBF_CompleteIndependentAudit20261112();
%  results=test_CBF_CompleteIndependentAudit20261112(struct('full',false));
%
% full=trueは高度・速度・姿勢・ゲインを広く試験。MATLAB所要時間はPCに依存。
% エラーを握りつぶした成功表示はしない。各段階のエラーはsummaryへ残す。
if nargin<1,options=struct();end
paths=test_CBF_SetupPaths();s=test_CBF_Fixture();
if isfield(options,'settings'),s=options.settings;end
options.settings=s;
if ~isfield(options,'full'),options.full=true;end
massDrone=s.physical.massDrone;massLoad=s.physical.massLoad;
assert(massDrone>0 && massLoad>0);
folder=fullfile(paths.generator,'CBF','test_results');
if ~exist(folder,'dir'),mkdir(folder);end
fprintf('\n=== CBF 全4段階・本体無変更の一括独立監査 ===\n');
fprintf('機体質量=%g（既知）／吊り荷質量=%g（推定値）。質量差・外乱は追加しません。\n',...
    massDrone,massLoad);
labels={'地面の成立領域','相対次数・段別ゲイン','仮想B/Dの過大入力',...
    '参照軌道・接近閉ループ','遠方障害物とハード制約'};
functions={@test_CBF_AuditGroundRegion20261112,...
    @test_CBF_AuditDegreeAndGains20261112,...
    @test_CBF_AuditVirtualDCondition20261112,...
    @test_CBF_AuditLongApproach20261112,...
    @test_CBF_AuditFarAndHard20261112};
fields={'ground','degreeAndGains','virtualD','rollout','farHard'};
log=cell(numel(labels),4);results=struct();
for k=1:numel(labels)
    fprintf('\n[%d/5] %s\n',k,labels{k});
    t=tic;
    try
        results.(fields{k})=functions{k}(options);
        log(k,:)={k,labels{k},'executed',toc(t)};
        fprintf('[実行完了] %s (試験内の違反・不可行はCSVを参照)\n',labels{k});
    catch ME
        log(k,:)={k,labels{k},['MATLAB_ERROR: ' ME.identifier],toc(t)};
        results.([fields{k} '_error'])=getReport(ME,'extended','hyperlinks','off');
        fprintf(2,'[監査コード実行エラー] %s: %s\n',labels{k},ME.message);
    end
end
results.summary=cell2table(log,'VariableNames',...
    {'testNumber','testName','executionStatus','elapsedSeconds'});
results.outputFolder=folder;
writetable(results.summary,fullfile(folder,'audit4_suite_execution_summary.csv'));
assert(s.physical.massDrone==massDrone && s.physical.massLoad==massLoad,...
 'CBF:MassChanged','監査中に質量の設定が変わりました。');
fprintf('\n=== 全監査コマンド終了：失敗も重要な検証結果です ===\n');
disp(results.summary);
fprintf('CSV: %s\n',folder);
fprintf('実HLC逆変換・連続時間前方不変性・実機性能は本試験では保証しません。\n');
end

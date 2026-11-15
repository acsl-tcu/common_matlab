function results=test_CBF_V8_All(options)
% TEST_CBF_V8_ALL 本体無変更のV8監査。テストの完走と安全性成立を区別する。
% result=test_CBF_V8_All();           % 少数ケース
% result=test_CBF_V8_All(struct('full',true));  % 全件（長め）
if nargin<1,options=struct();end
if ~isfield(options,'full'),options.full=false;end
paths=test_CBF_SetupPaths();s=test_CBF_Fixture();
if isfield(options,'settings'),s=options.settings;end
options.settings=s;
folder=fullfile(paths.generator,'CBF','test_results');if ~exist(folder,'dir'),mkdir(folder);end
assert(s.physical.massDrone>0 && s.physical.massLoad>0);
fprintf('\n=== V8 CBF診断開始：V7生成器・QP・物理パラメータ無変更 ===\n');
fprintf('既知の機体質量 %g kg / 吊り荷の推定質量 %g kg\n',...
 s.physical.massDrone,s.physical.massLoad);
labels={'参照軌道の高階微分の自己検算',...
        '仮想B/D ホバリングQPと名目入力の分離',...
        '地面生微分・6階HOCBF・理想鉛直制動',...
        '元モデルの次数と段別ゲイン（旧監査との再照合）',...
        '遠方障害物の制約行',...
        '0～5階が一致する参照軌道の独立閉ループ',...
        '既存HLC生成関数の実入力逆変換インターフェース'};
fns={@test_CBF_V8_ReferenceConsistency,@test_CBF_V8_HoverQP,@test_CBF_V8_GroundStructure,...
     @test_CBF_AuditDegreeAndGains20261112,@test_CBF_V8_FarRows,...
     @test_CBF_V8_AlignedTrajectories,@test_CBF_V8_HLCInverseInterface};
fields={'referenceCheck','hover','ground','degreeGains','far','reference','hlc'};
log=cell(numel(fns),5);results=struct();
for k=1:numel(fns)
 t=tic;
 try
  results.(fields{k})=fns{k}(options);
  log(k,:)={k,labels{k},'EXECUTED_CHECK_CSV',toc(t),''};
  fprintf('[実行終了] %s：制御上の成功とは限りません。\n',labels{k});
 catch ME
  log(k,:)={k,labels{k},'MATLAB_ERROR',toc(t),[ME.identifier ' ' ME.message]};
  results.([fields{k} '_error'])=getReport(ME,'extended','hyperlinks','off');
  fprintf(2,'[V8 MATLABエラー] %s: %s\n',labels{k},ME.message);
 end
end
results.summary=cell2table(log,'VariableNames',{'caseId','testName',...
 'executionStatus','elapsedSeconds','errorText'});
writetable(results.summary,fullfile(folder,'v8_execution_summary.csv'));
fprintf('\n=== V8一括診断終了 ===\n');disp(results.summary);
fprintf('CSV 保存先: %s\n',folder);
fprintf('ハード制約を緩めた試験ではありません。実HLC接続・連続時間安全保証は未完了。\n');
end

function report=test_CBF_V13_R2StructureSummary(opts)
%TEST_CBF_V13_R2STRUCTURESUMMARY Structural audit, not a valid R2 HOCBF.
% Recomputes mixed-degree input actions from the existing 19-state model.
if nargin<1,opts=struct();end
T=test_CBF_V12_R2StructuralRows(opts);
if any(strcmp(T.barrier,'TEST_ERROR'))
    error('CBF:V13R2SourceError','Underlying structural audit contains errors.');
end
keys=unique(T(:,{'shape','barrier','chosenOrder','channel'}),'rows');
row=cell(height(keys),10);
for i=1:height(keys)
    s=strcmp(T.shape,keys.shape{i}) & strcmp(T.barrier,keys.barrier{i}) &...
      T.chosenOrder==keys.chosenOrder(i) & T.channel==keys.channel(i);
    C=T(s,:);
    if isempty(C),error('CBF:V13EmptyGroup','Internal grouped audit error.');end
    earlier=sum(C.hasEarlierInput>0);nonaffine=sum(C.hasNonaffine>0);
    if earlier>0 || nonaffine>0
        verdict='NAIVE_HOCBF_ORDER_NOT_VALID';
    elseif max(abs(C.chosenOrderInputCoefficient))<1e-9
        verdict='DESIGN_ORDER_CONTROL_ZERO_IN_SAMPLED_STATES';
    else
        verdict='PASSED_LOCAL_AFFINITY_CHECK_ONLY';
    end
    row(i,:)={keys.shape{i},keys.barrier{i},keys.chosenOrder(i),...
      keys.channel(i),height(C),earlier,nonaffine,...
      max(C.maxEarlierOrderEffect),max(C.chosenOrderCurvature),verdict};
end
report=cell2table(row,'VariableNames',...
 {'shape','barrier','chosenOrder','channel','sampleCount',...
  'earlyInputCount','nonaffineCount','maximumEarlyInputEffect',...
  'maximumDesignOrderCurvature','verdict'});
writetable(report,fullfile(CBF_V11_OutputFolder(),'v13_r2_structure_summary.csv'));
fprintf('[V13] R2混合4/6構造: %d集計行。通常HOCBFへの流用禁止。\n',height(report));
end

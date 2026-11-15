function results=test_CBF_V16_All(opts)
%TEST_CBF_V16_ALL User-facing V16 independent comparison; no core edit.
if nargin<1,opts=struct();end
if ~isfield(opts,'full'),opts.full=false;end
if ~isfield(opts,'failFast'),opts.failFast=false;end
tests={'test_CBF_V16_Smoke','test_CBF_V16_R1Rectified',...
  'test_CBF_V16_R1ZOCBF','test_CBF_V16_R2Three',...
  'test_CBF_V16_VirtualGuard'};
R=cell(numel(tests),4);
for k=1:numel(tests)
    tm=tic;stat='NOT_RUN';err='';
    try
        if k==1,feval(tests{k});stat='SMOKE_PASS';
        else
            result=feval(tests{k},opts);
            stat=sprintf('EXECUTED_%d_ROWS_REVIEW_CSV',height(result));
        end
    catch ME
        stat='TEST_CODE_ERROR';err=[ME.identifier ': ' ME.message];
        fprintf('[V16 %d/%d] %s: %s\n',k,numel(tests),ME.identifier,ME.message);
        if opts.failFast,rethrow(ME);end
    end
    R(k,:)={tests{k},stat,toc(tm),err};
end
results=cell2table(R,'VariableNames',{'testName','status','seconds','errorText'});
writetable(results,fullfile(CBF_V11_OutputFolder(),'v16_execution_summary.csv'));
disp(results);
fprintf('[V16] No core/Model/HLC overwritten. Test completion != safety proof.\n');
end

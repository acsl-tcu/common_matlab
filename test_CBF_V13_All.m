function results=test_CBF_V13_All(opts)
%TEST_CBF_V13_ALL V12 evidence-driven next checks, TEST ONLY.
% No CBF generator/HLC/controller changes. A solver exitflag is NOT a pass.
if nargin<1,opts=struct();end
if ~isfield(opts,'full'),opts.full=false;end
if ~isfield(opts,'failFast'),opts.failFast=false;end
test_CBF_SetupPaths();
tests={'smoke','R2Structure','virtualQPRowAndThrustPreview','R1GroundNonlinearRollout'};
fun={@()test_CBF_V13_Smoke(),...
 @()test_CBF_V13_R2StructureSummary(opts),...
 @()test_CBF_V13_VirtualQPDiagnosis(opts),...
 @()test_CBF_V13_R1GroundRollout(opts)};
status=cell(4,1);seconds=zeros(4,1);errs=cell(4,1);data=struct();
fprintf('\n=== V13: R1 nonlinear ground, R2 structure, virtual QP/HLC diagnosis ===\n');
for i=1:4
    t=tic;try
        dat=fun{i}();data.(tests{i})=dat;
        if isstruct(dat) && isfield(dat,'status') && strcmp(dat.status,'PASS')
            status{i}='SMOKE_PASS';
        elseif istable(dat)
            if any(strcmp(dat.Properties.VariableNames,'status')) &&...
               any(startsWith(string(dat.status),'TEST_ERROR'))
                status{i}='EXECUTED_CONTAINS_TEST_ERRORS';
            else
                status{i}='EXECUTED_RESULTS_REQUIRE_REVIEW';
            end
        else
            status{i}='EXECUTED_RESULTS_REQUIRE_REVIEW';
        end
        errs{i}='';
    catch ME
        status{i}='TEST_CODE_ERROR';errs{i}=[ME.identifier ': ' ME.message];
        data.(tests{i})=[];
        fprintf(2,'[V13] %s: %s\n',tests{i},errs{i});
        if opts.failFast,rethrow(ME);end
    end
    seconds(i)=toc(t);
end
summary=table(tests(:),status,seconds,errs,...
 'VariableNames',{'testName','executionStatus','seconds','errorText'});
writetable(summary,fullfile(CBF_V11_OutputFolder(),'v13_execution_summary.csv'));
disp(summary);
results=struct('summary',summary,'details',data);
fprintf(['警告: 1点QP解、初期補助集合、物理入力条件、張力、実閉ループ安全性は別判定。'...
 'R1は地面のみ／独立診断制御、仮想短時間推力は凍結状態の参考値。\n']);
end

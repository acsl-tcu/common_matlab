function results=test_CBF_V14_All(opts)
%TEST_CBF_V14_ALL Independent V14 tests, with no changes to CBF/HLC core.
% Every status must be reviewed. No safety slack, clipping or unproved
% obstacle culling is introduced by these tests.
if nargin<1,opts=struct();end
if ~isfield(opts,'full'),opts.full=false;end
if ~isfield(opts,'failFast'),opts.failFast=false;end
test_CBF_SetupPaths();
tests={'smoke','virtualDegeneracy','R1PhysicalBrakingWitness','R2InputOrderSweep'};
fun={@()test_CBF_V14_Smoke(),...
     @()test_CBF_V14_VirtualDegeneracy(opts),...
     @()test_CBF_V14_R1BrakingWitness(opts),...
     @()test_CBF_V14_R2OrderSweep(opts)};
status=cell(4,1);seconds=zeros(4,1);errs=cell(4,1);data=struct();
fprintf('\n=== V14: virtual zero-action, R1 physical braking, R2 action order ===\n');
for k=1:4
    t=tic;
    try
        report=fun{k}();data.(tests{k})=report;
        if isstruct(report) && isfield(report,'status') && strcmp(report.status,'PASS')
            status{k}='SMOKE_PASS';
        elseif istable(report) && any(strcmp(report.Properties.VariableNames,'status')) &&...
                any(strcmp(report.status,'TEST_ERROR'))
            status{k}='EXECUTED_WITH_ROW_ERRORS';
        else
            status{k}='EXECUTED_REVIEW_CSV_NOT_PASS';
        end
        errs{k}='';
    catch ME
        status{k}='TEST_CODE_ERROR';errs{k}=[ME.identifier ': ' ME.message];
        data.(tests{k})=[];
        fprintf(2,'[V14 %d/4] %s: %s\n',k,tests{k},errs{k});
        if opts.failFast,rethrow(ME);end
    end
    seconds(k)=toc(t);
end
summary=table(tests(:),status,seconds,errs,...
    'VariableNames',{'testName','executionStatus','seconds','errorText'});
writetable(summary,fullfile(CBF_V11_OutputFolder(),'v14_execution_summary.csv'));
disp(summary);
results=struct('summary',summary,'details',data);
fprintf(['注意: 「停止できた」は有限時間の開ループ実験であり制御不変性の証明ではない。\n'...
  '仮想の near-zero は数値的な不確定であり、QP成功扱いしない。\n']);
end

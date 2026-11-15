function results=test_CBF_V12_All(opts)
%TEST_CBF_V12_ALL R1/R2 and corrected VIRTUAL design audits, TEST ONLY.
% No generator, no installed CBF, no HLC modifications. All new .m are
% meant to be placed directly in common_matlab alongside V11 test helpers.
if nargin<1,opts=struct();end
if ~isfield(opts,'full'),opts.full=false;end
test_CBF_SetupPaths();
list={'smoke','virtualTaylorParity','R1GroundPhysicalQP',...
 'R2MixedDegreeRows','virtualHLCPhysicalHardQP'};
fun={@()test_CBF_V12_Smoke(),...
 @()test_CBF_V12_VirtualTaylorParity(opts),...
 @()test_CBF_V12_R1Ground(opts),...
 @()test_CBF_V12_R2StructuralRows(opts),...
 @()test_CBF_V12_VirtualHardQP(opts)};
statuses=cell(numel(list),1);duration=zeros(numel(list),1);
errors=cell(numel(list),1);data=struct();
fprintf('\n=== V12: 実R1/R2 + 仮想6階/4階正しい微分 + HLC実入力制限 ===\n');
for i=1:numel(list)
    ticStart=tic;
    try
        outcome=fun{i}();data.(list{i})=outcome;
        if isstruct(outcome)&&isfield(outcome,'status')&&strcmp(outcome.status,'PASS')
            statuses{i}='SMOKE_PASS';
        elseif istable(outcome)
            errs=false;
            for name={'status','errorText'}
                if any(strcmp(outcome.Properties.VariableNames,name{1}))
                    vals=string(outcome.(name{1}));
                    errs=errs || any(startsWith(vals,'TEST_ERROR') | contains(vals,'CBF:V12'));
                end
            end
            if errs,statuses{i}='EXECUTED_WITH_ERRORS_REVIEW_CSV';
            else,statuses{i}='EXECUTED_RESULTS_REQUIRE_REVIEW';end
        else
            statuses{i}='EXECUTED_RESULTS_REQUIRE_REVIEW';
        end
        errors{i}='';
    catch ME
        statuses{i}='TEST_CODE_ERROR';
        errors{i}=[ME.identifier ': ' ME.message];
        data.(list{i})=[];
        fprintf(2,'[V12 %d/%d] %s\n',i,numel(list),errors{i});
    end
    duration(i)=toc(ticStart);
end
summary=table(list(:),statuses,duration,errors,...
 'VariableNames',{'testName','status','seconds','errorText'});
writetable(summary,fullfile(CBF_V11_OutputFolder(),'v12_execution_summary.csv'));
disp(summary);
results=struct('summary',summary,'detail',data);
fprintf(['注意: QPの一点可行・初期補助集合・物理入力上下限・' ...
 '閉ループ安全性は別の判定。本体変更なし。\n']);
end

function results=test_CBF_V11_All(opts)
%TEST_CBF_V11_ALL common_matlab直下で動かす、既存CBF無変更の独立監査。
% 監査が実行できたことと、安全保証が成立したことは別。
% opts.full=false(標準), true(全条件)
% opts.hlcFolder：省略時は common_matlab/controller を自動検出。
if nargin<1,opts=struct();end
if ~isfield(opts,'full'),opts.full=false;end
fprintf('\n=== V11: 実R0/R1/R2 + C/B/D + 仮想HLC 微分鎖テスト ===\n');
test_CBF_SetupPaths();
names={'smoke','realOrder','R2HardQP','threeWay','virtualHLC'};
functions={@()test_CBF_V11_Smoke(), ...
 @()test_CBF_V11_RealOrder(opts), ...
 @()test_CBF_V11_R2QP(opts), ...
 @()test_CBF_V11_CompareThreeReal(opts), ...
 @()test_CBF_V11_VirtualHLC(opts)};
statuses=cell(numel(names),1);times=zeros(numel(names),1);
errors=cell(numel(names),1);data=struct();
for i=1:numel(names)
 t=tic;
 try
    output=functions{i}();
    data.(names{i})=output;
    statuses{i}='EXECUTED_REVIEW_CSV';errors{i}='';
    if isstruct(output) && isfield(output,'status') && strcmp(output.status,'PASS')
       statuses{i}='SMOKE_PASS';
    end
    if istable(output)
       keys=output.Properties.VariableNames;
       for j=1:numel(keys)
           key=keys{j};
           if any(strcmp(key,{'status','solverStatus','R0SolverStatus','R2SolverStatus'}))
               values=string(output.(key));
               if any(contains(values,'HLC_CLASS_MISSING'))
                   statuses{i}='HLC_MISSING_NOT_TESTED';
               elseif any(contains(values,'TEST_ERROR')|startsWith(values,'ERROR_') | ...
                          contains(values,'HLC_METHOD_MISSING') | contains(values,'HLC_CLASS_CONSTRUCTOR_ERROR'))
                   statuses{i}='EXECUTED_WITH_ROW_ERRORS';
               end
           end
       end
       if strcmp(statuses{i},'EXECUTED_WITH_ROW_ERRORS') && ...
          any(strcmp(keys,'errorText'))
          firstErr=find(strlength(string(output.errorText))>0,1,'first');
          if ~isempty(firstErr)
              errors{i}=char(string(output.errorText(firstErr)));
          end
       end
    end
 catch ME
    data.(names{i})=[];
    statuses{i}='TEST_CODE_ERROR';
    errors{i}=sprintf('%s: %s',ME.identifier,ME.message);
    fprintf(2,'[V11 %d/%d] %s\n',i,numel(names),errors{i});
 end
 times(i)=toc(t);
end
summary=table(names(:),statuses,times,errors, ...
 'VariableNames',{'testName','status','seconds','errorText'});
writetable(summary,fullfile(CBF_V11_OutputFolder(),'v11_execution_summary.csv'));
disp(summary);
results=struct('summary',summary,'detail',data);
fprintf(['重要：QP可行・初期補助条件・物理入力上限・閉ループ安全性は別判定。' ...
    'V11は候補の構造監査であり、本体変更・安全保証ではありません。\n']);
end

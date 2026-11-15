function results=test_CBF_AuditFarAndHard20261112(options)
% 遠方障害物によるQP矛盾と「ソフト制約無し」を追加監査。
% 実際のCBFを一切緩めない。保存QPからのハード診断のみ。
if nargin<1,options=struct();end
paths=test_CBF_SetupPaths();s=test_CBF_Fixture();
if isfield(options,'settings'),s=options.settings;end
full=getopt(options,'full',true);
if full,distances=[.65 1 3 10 100];else,distances=[1 100];end
configs=[1.5 0 0;1.2 -.65 0;2 .3 .2];
rows=cell(0,14);i=0;
for j=1:size(configs,1)
 c=configs(j,:);xr=CBF_Audit_State('real',s,c(1),c(2),0,c(3),0,0,0);
 for sh={'C','B','D'}
  shape=sh{1};
  for kindCell={'sphere','ellipsoid'}
   kind=kindCell{1};
   for dist=distances
    ob=CBF_Audit_State('obstacle',kind,xr(8:10),dist,'front',.5);
    qp=cbf_hocbf_scene('real',shape,xr,s,{ob});
    gg=contains(qp.labels,'ground_');
    ii=contains(qp.labels,'_thrust_')|contains(qp.labels,'_tau_');
    rBase=CBF_Audit_LP(qp,gg|ii);rFull=CBF_Audit_LP(qp);
    obsRows=~(gg|ii);
    nontrivial=sum(abs(qp.b(obsRows))>1e-10);
    maxRHS=max(qp.b(obsRows));
    maxAnorm=max(sqrt(sum(qp.A(obsRows,:).^2,2)));
    i=i+1;
    rows(end+1,:)={i,j,shape,kind,dist,rBase.feasible,...
       rFull.feasible,all(qp.psiInitiallyNonnegative(gg|ii)),...
       all(qp.psiInitiallyNonnegative(obsRows)),nontrivial,...
       maxRHS,maxAnorm,rFull.exitflag,nnz(obsRows)}; %#ok<AGROW>
   end
  end
 end
end
results=cell2table(rows,'VariableNames',{'caseId','stateId','systemShape',...
    'obstacleType','distance','groundAndInputFeasible','allHardFeasible',...
    'groundAndInputPsiOK','obstaclePsiOK','nonzeroObstacleRHSRows',...
    'maxObstacleRHS','maxObstacleEffectNorm','LPexitflag','obstacleRows'});
folder=fullfile(paths.generator,'CBF','test_results');if ~exist(folder,'dir'),mkdir(folder);end
writetable(results,fullfile(folder,'audit4_far_obstacle_hard_feasibility.csv'));
% 必ず不可行な対称ハード制約を与え、ソルバが入力を捏造しないことを確認。
qp0=struct('A',[1 0 0 0;-1 0 0 0],...
    'b',[1;1],'psiInitiallyNonnegative',true(2,1),...
    'inputLimitIncluded',false);
test=cbf_solve_hocbf_qp(qp0,zeros(4,1),eye(4));
if test.exitflag>0
    error('CBF:AuditHardQPBroken','明らかに不可行なハードQPで成功が返りました。');
end
fprintf('[遠方/ハード] %d条件。遠方障害物でもQP行を削除せず評価。不可行の偽装なし。\n',...
    size(results,1));
end
function v=getopt(o,k,d),if isfield(o,k)&&~isempty(o.(k)),v=o.(k);else,v=d;end,end

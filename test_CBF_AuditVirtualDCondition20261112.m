function report=test_CBF_AuditVirtualDCondition20261112(options)
% D/Bの過大な仮想決定入力を、入力作用係数・行スケール・QP実行可能性で比較。
% 追加の入力箱制限やスラックを導入しない。HLC逆変換は未接続。
if nargin<1,options=struct();end
paths=test_CBF_SetupPaths();s=test_CBF_Fixture();
if isfield(options,'settings'),s=options.settings;end
full=getopt(options,'full',true);
if full
    distances=[.37 .46 .6 1 2];
    pitches=[-.2 0 .2]; vxs=[-.8 0 .8];
else
    distances=[.46 1];pitches=[0 .2];vxs=[0 .8];
end
rows=cell(0,20);idx=0;
for pitch=pitches
  for vx=vxs
    xr=CBF_Audit_State('real',s,1.6,-.1,.08,pitch,.12,vx,.10,.35);
    xv=CBF_Audit_State('virtualfromreal',xr,s);
    for typ={'sphere','ellipsoid'}
      kind=typ{1};
      for dist=distances
        obs=CBF_Audit_State('obstacle',kind,xr(8:10),dist,'front',.7);
        for sh={'B','D'}
           shape=sh{1};idx=idx+1;status='';nuinf=NaN;active='';
           hmin=NaN;minPsi=NaN;isInit=false;lpOk=false;
           minEffect=NaN;maxNeed=NaN;condA=NaN;rankA=NaN;
           residual=NaN;maxA=NaN;maxB=NaN;zeroRisk=NaN;
           try
             [h,~]=cbf_evaluate_case('virtual',shape,kind,xv,s,obs);
             qp=cbf_hocbf_scene('virtual',shape,xv,s,{obs});
             hmin=min(h); minPsi=min(cellfun(@min,qp.psi));
             isInit=all(qp.psiInitiallyNonnegative);
             lp=CBF_Audit_LP(qp);lpOk=lp.feasible;
             minEffect=min(qp.effectNorm);maxA=max(abs(qp.A(:)));
             maxB=max(abs(qp.b));
             need=qp.b./max(qp.effectNorm,eps);
             maxNeed=max(need);
             zeroRisk=sum(qp.effectNorm<=s.cbf.degeneracyTolerance & qp.b>0);
             % yawが幾何に作用しないので、3つの位置入力チャネルで診断。
             sv=svd(qp.A(:,1:3));rankA=sum(sv>1e-10*max(1,max(sv)));
             positive=sv(sv>1e-10*max(1,max(sv)));
             if ~isempty(positive),condA=max(positive)/min(positive);end
             if lpOk
                sol=cbf_solve_hocbf_qp(qp,zeros(4,1),eye(4));
                if sol.exitflag>0
                   status='QP_solved';nuinf=norm(sol.nu,Inf);
                   residual=sol.minResidual;
                   sl=qp.A*sol.nu-qp.b;
                   [~,ix]=min(sl./max(1,abs(qp.b)));
                   active=qp.labels{ix};
                else
                   status='quadprog_failed';
                end
             else
                status='LP_infeasible';
             end
           catch ME
             status=['error_' ME.identifier];
           end
           rows(end+1,:)={idx,shape,kind,pitch,vx,dist,...
               hmin,minPsi,isInit,lpOk,nuinf,minEffect,maxNeed,...
               condA,rankA,zeroRisk,maxA,maxB,active,status}; %#ok<AGROW>
        end
      end
    end
  end
end
report=cell2table(rows,'VariableNames',{'caseId','systemShape','obstacleType',...
    'pitch','loadVelocityX','obstacleDistance','minH','minPsi','initialPsiOK',...
    'LPfeasible','QPdecisionInfinityNorm','minimumInputEffectNorm',...
    'largestNormalizedRowDemand','effectiveConditionNumber',...
    'effectiveRank','uncontrollablePositiveRows','maxAbsA','maxAbsB',...
    'mostTightRow','status'});
folder=fullfile(paths.generator,'CBF','test_results');if ~exist(folder,'dir'),mkdir(folder);end
writetable(report,fullfile(folder,'audit4_virtual_B_D_input_condition.csv'));
large=report.QPdecisionInfinityNorm>1e4;
fprintf('[仮想B/D] %d試験、QP大入力(>1e4)=%d、QP可行=%d、初期集合成立=%d\n',...
    size(report,1),sum(large),sum(report.LPfeasible),sum(report.initialPsiOK));
end
function v=getopt(o,k,d),if isfield(o,k)&&~isempty(o.(k)),v=o.(k);else,v=d;end,end

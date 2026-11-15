function report = test_CBF_V8_HoverQP(options)
% V8: 実状態から整合したホバリング初期状態を作り、仮想B/Dの1ステップQPを行別に解析。
% 目標誤差に起因する巨大なnominal入力と、CBF制約による修正を区別する。
% 制約緩和・入力の飽和・CBF本体の書き換えは行わない。
if nargin<1,options=struct();end
paths=test_CBF_SetupPaths(); s=test_CBF_Fixture();
if isfield(options,'settings'),s=options.settings;end
folder=fullfile(paths.generator,'CBF','test_results');if ~exist(folder,'dir'),mkdir(folder);end
errors=getOpt(options,'positionErrors',[0 .1 .5 1 2]);
velErrors=getOpt(options,'velocityErrors',[0 .2]);
trackingGain=getOpt(options,'trackingGain',1.6);
rows=cell(0,19); details=cell(0,19); idx=0; rid=0;
xr=CBF_Audit_State('real',s,1.5,0,0,0,0,0,0);
x=CBF_Audit_State('virtualfromreal',xr,s);
for kindCell={'sphere','ellipsoid'}
    kind=kindCell{1};
    obs=CBF_Audit_State('obstacle',kind,xr(8:10),.7,'front',.7);
    for sh={'B','D'}
        shape=sh{1};
        qp=cbf_hocbf_scene('virtual',shape,x,s,{obs});
        zeroResidual=qp.A*zeros(4,1)-qp.b;
        zeroOK=all(zeroResidual>=-1e-8*(1+norm(qp.b,Inf)));
        rankInfo=svd(qp.A(:,1:3));
        posSing=rankInfo(rankInfo>max(1e-12,1e-10*max(rankInfo)));
        condNonzero=NaN;
        if ~isempty(posSing),condNonzero=max(posSing)/min(posSing);end
        lp=CBF_Audit_LP(qp);
        for posErr=errors(:).'
            for velErr=velErrors(:).'
                % 初期「実状態」は不変。参照をずらすだけで追従誤差を生成。
                % z, y, yawの参照は現在状態に整合させる。x方向のみ故意に誤差。
                nuNom=zeros(4,1);
                nuNom(1)=trackingGain^6*posErr+6*trackingGain^5*velErr;
                [nu,status,exitflag,minResidual]=solveOne(qp,nuNom);
                if all(isfinite(nu))
                    diffNorm=norm(nu-nuNom,Inf);realNorm=norm(nu,Inf);
                    res=qp.A*nu-qp.b;
                else
                    diffNorm=NaN;realNorm=NaN;res=nan(size(qp.b));
                end
                idx=idx+1;
                rows(end+1,:)={idx,shape,kind,posErr,velErr,...
                  all(qp.psiInitiallyNonnegative),lp.feasible,zeroOK,min(zeroResidual),...
                  norm(nuNom,Inf),realNorm,diffNorm,exitflag,minResidual,...
                  condNonzero,min(qp.effectNorm),max(qp.b),status,...
                  max(0,-min(zeroResidual))}; %#ok<AGROW>
                for j=1:numel(qp.b)
                    rid=rid+1;
                    details(end+1,:)={rid,idx,shape,kind,qp.labels{j},qp.order(j),...
                      qp.A(j,1),qp.A(j,2),qp.A(j,3),qp.A(j,4),...
                      norm(qp.A(j,:)),qp.b(j),zeroResidual(j),...
                      qp.A(j,:)*nuNom-qp.b(j),res(j),...
                      min(qp.psi{j}),qp.psiInitiallyNonnegative(j),...
                      norm(qp.A(j,:))<s.cbf.degeneracyTolerance,status}; %#ok<AGROW>
                end
            end
        end
    end
end
report=cell2table(rows,'VariableNames',{'caseId','shape','obstacleType','referencePositionErrorX',...
    'referenceVelocityErrorX','initialPsiOK','LPfeasible','zeroDecisionFeasible',...
    'minZeroDecisionResidual','nominalInputInfinity','QPInputInfinity','QPcorrectionInfinity',...
    'QPexitflag','minQPResidual','effectiveConditionNumber','minimumInputEffectNorm',...
    'maximumCBFRHS','QPstatus','zeroDecisionViolation'});
byRow=cell2table(details,'VariableNames',{'rowId','caseId','shape','obstacleType',...
    'label','designOrder','A1','A2','A3','A4','effectNorm','rhsB',...
    'zeroInputResidual','nominalInputResidual','solutionResidual','minimumPsi',...
    'initialPsiOK','degenerate','QPstatus'});
writetable(report,fullfile(folder,'v8_virtual_hover_qp_sweep.csv'));
writetable(byRow,fullfile(folder,'v8_virtual_hover_qp_rows.csv'));
fprintf('[V8 仮想B/D] %d条件。ゼロ入力可行=%d、QP成功=%d。nominalとQP修正を別記録。\n',...
    height(report),sum(report.zeroDecisionFeasible),sum(report.QPexitflag>0));
end
function [u,status,flag,mr]=solveOne(qp,nuNom)
u=nan(4,1);flag=NaN;mr=NaN;status='';
try
 sol=cbf_solve_hocbf_qp(qp,nuNom,eye(4));flag=sol.exitflag;
 if flag>0 && ~isempty(sol.nu)
    u=sol.nu;mr=min(qp.A*u-qp.b);
    if mr<-1e-7*(1+norm(qp.b,Inf)),status='QP_residual_negative';
    else,status='QP_solved';end
 else,status='QP_infeasible_or_solver_failure';end
catch ME
 status=['MATLAB_ERROR_' ME.identifier];
end
end
function v=getOpt(s,k,d)
if isfield(s,k)&&~isempty(s.(k)),v=s.(k);else,v=d;end
end

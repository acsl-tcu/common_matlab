function report=test_CBF_AuditReferenceInitial(options)
%TEST_CBF_AUDITREFERENCEINITIAL 解析参照軌道に対して仮想CBFの初期値・追従を検査。
% 初期0～5階微分は参照軌道に一致させ、外部から位置/速度誤差を加える。
% 6階微分を名目入力にする。HLC逆変換は使用していない。
if nargin<1,options=struct();end
paths=test_CBF_SetupPaths();s=test_CBF_Fixture();
if isfield(options,'settings'),s=options.settings;end
steps=getopt(options,'steps',12);dt=getopt(options,'dt',.01);
trajectories={'hover','line','circle','ascent','figure8'};
shapes=getopt(options,'shapes',{'B','D'});
errors=[0 0; .2 0; 0 .4; -.2 -.4]; % [x方向の位置ずれ, 速度ずれ]
rows=cell(0,15);id=0;
for tr=trajectories
    name=tr{1};
    for sh=shapes
        shape=sh{1};
        for kindCell={'sphere','ellipsoid'}
            kind=kindCell{1};
            for e=1:size(errors,1)
                x=zeros(20,1);
                for j=0:5
                    x(3*j+1:3*j+3)=CBF_Audit_State('reference',name,0,j);
                end
                x(1)=x(1)+errors(e,1);x(4)=x(4)+errors(e,2);
                % 軌道の周囲に障害物を置き、接近/再合流時の初期条件を検証。
                ob=CBF_Audit_State('obstacle',kind, ...
                    CBF_Audit_State('reference',name,0,0),.8,'front',.53);
                status='completed';completed=0;maxNu=0;
                hInitial=cbf_evaluate_scene('virtual',shape,x,s,{ob});
                minH=min(hInitial);minPsi=Inf;maxTrackErr=0;qpFeasibleInitial=NaN;
                for k=1:steps
                    t=(k-1)*dt;
                    try
                        qp=cbf_hocbf_scene('virtual',shape,x,s,{ob});
                        minPsi=min(minPsi,min(cellfun(@min,qp.psi)));
                        if k==1
                            lpInitial=CBF_Audit_LP(qp);
                            qpFeasibleInitial=lpInitial.feasible;
                        end
                        if ~all(qp.psiInitiallyNonnegative)
                            status='initial_HOCBF_set_violation';break;
                        end
                        % 実際のHLC補正ではなく、参照の6階フィードフォワード。
                        nuNom=[CBF_Audit_State('reference',name,t,6);0];
                        sol=cbf_solve_hocbf_qp(qp,nuNom,eye(4));
                        if sol.exitflag<=0 || ~isfinite(sol.minResidual) || ...
                           sol.minResidual<-1e-7
                            status='qp_infeasible';break;
                        end
                        maxNu=max(maxNu,norm(sol.nu,Inf));
                        if maxNu>1e5
                            status='excessive_unbounded_virtual_input';break;
                        end
                        xn=CBF_Audit_State('stepvirtual',x,sol.nu,dt);
                        diag=cbf_model_diagnostics('virtual',xn,s);
                        if ~diag.flatMapValid || any(~isfinite(xn)) || ...
                           norm(xn(1:18),Inf)>1e5
                            status='virtual_model_or_state_invalid';break;
                        end
                        h=cbf_evaluate_scene('virtual',shape,xn,s,{ob});
                        minH=min(minH,min(h));
                        if any(~isfinite(h))||min(h)<-1e-7
                            status='geometric_violation_after_integration';break;
                        end
                        maxTrackErr=max(maxTrackErr,norm(xn(1:3)- ...
                            CBF_Audit_State('reference',name,t+dt,0)));
                        x=xn;completed=k;
                    catch err
                        status=['error_' err.identifier];break;
                    end
                end
                if completed==steps && strcmp(status,'completed')
                    try
                        qp=cbf_hocbf_scene('virtual',shape,x,s,{ob});
                        if ~all(qp.psiInitiallyNonnegative)
                            status='final_HOCBF_set_violation';
                        end
                    catch
                        status='final_coefficient_error';
                    end
                end
                id=id+1;
                rows(end+1,:)={id,name,shape,kind,e,errors(e,1), ...
                    errors(e,2),steps,completed,status,qpFeasibleInitial, ...
                    minH,minPsi,maxNu,maxTrackErr}; %#ok<AGROW>
            end
        end
    end
end
report=cell2table(rows,'VariableNames',{'caseId','reference','shape', ...
    'obstacleType','errorCase','initialXOffset','initialVxOffset', ...
    'requestedSteps','completedSteps','status','initialQPFeasible', ...
    'minH','minPsi','maxDecisionInput','maxTrackingPositionError'});
folder=fullfile(paths.generator,'CBF','test_results');
if ~exist(folder,'dir'),mkdir(folder);end
writetable(report,fullfile(folder,'audit_virtual_reference_initial.csv'));
fprintf('[参照軌道] 完走=%d/%d。実HLC追従性能の評価ではありません。\n', ...
    sum(strcmp(report.status,'completed')),size(report,1));
end
function v=getopt(o,k,d)
if isfield(o,k)&&~isempty(o.(k)),v=o.(k);else,v=d;end
end

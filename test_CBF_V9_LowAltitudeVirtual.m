function report=test_CBF_V9_LowAltitudeVirtual(options)
% [5a] 低高度へ下降→減速→横方向に巡航→上昇する理想仮想チェーンのCBF-QP。
% 実HLC/実推力・トルクは接続していないので成功判定は理想系のみ。
% 参照の0～5階が初期と完全に一致する状態を使う。
if nargin<1,options=struct();end
s=CBF_V9_Util('options',options,'settings',test_CBF_Fixture());
full=CBF_V9_Util('options',options,'full',false);
if full,shapes={'C','B','D'};kinds={'sphere','ellipsoid'};routes={'blocking','offroute'};dt=.01;
else,shapes={'B','D'};kinds={'sphere','ellipsoid'};routes={'blocking','offroute'};dt=.02;end
dt=CBF_V9_Util('options',options,'virtualDt',dt);
steps=ceil(3.6/dt);
lam=CBF_V9_Util('options',options,'trackingGain',1.6);
limit=CBF_V9_Util('options',options,'diagnosticInputThreshold',1e7);
rows=cell(0,23);id=0;
startZ=CBF_V9_Util('options',options,'zHigh',.45);
xr=CBF_Audit_State('real',s,startZ,0,0,0,0,0,0);
xInit=CBF_Audit_State('virtualfromreal',xr,s);
initErr=0;
for d=0:5
 val=CBF_V9_LowReference(0,d,options);
 initErr=max(initErr,norm(val-xInit(3*d+1:3*d+3),Inf));
end
if initErr>1e-10,error('CBF:V9InitialMismatch','仮想参照初期0～5階が一致しません');end
for sh=shapes
 shape=sh{1};
 for kd=kinds
  kind=kd{1};
  for rt=routes
   route=rt{1};id=id+1;
   zL=CBF_V9_Util('options',options,'zLow',.24);
   if strcmp(route,'blocking'),center=[.25;0;zL];
   else,center=[.25;.75;zL];end
   if strcmp(kind,'sphere')
     obs=struct('type','sphere','center',center,'radius',.12);
   else
     theta=.5;R=[cos(theta) -sin(theta) 0;sin(theta) cos(theta) 0;0 0 1];
     obs=struct('type','ellipsoid','center',center,'axes',[.12;.17;.20],'R',R);
   end
   x=xInit;status='completed';stepDone=0;minGeom=Inf;minPsi=Inf;
   minGround=Inf;minResidual=Inf;maxNom=0;maxNu=0;maxError=0;
   firstBadLabel='';errorText='';initialLP=false;
   firstBadTime=NaN;phaseReached=0;
   try
    for k=1:steps
     t=(k-1)*dt;
     q=cbf_hocbf_scene('virtual',shape,x,s,{obs});
     if k==1
      lp=CBF_Audit_LP(q);initialLP=lp.feasible;
     end
     minPsi=min(minPsi,min(cellfun(@min,q.psi)));
     if ~all(q.psiInitiallyNonnegative)
       status='initial_or_later_auxiliary_failure';
       j=find(~q.psiInitiallyNonnegative,1);
       firstBadLabel=q.labels{j};firstBadTime=t;break;
     end
     nominal=zeros(4,1);
     for j=0:5
       reference=CBF_V9_LowReference(t,j,options);
       nominal(1:3)=nominal(1:3)-nchoosek(6,j)*lam^(6-j)*...
                (x(3*j+1:3*j+3)-reference);
     end
     nominal(1:3)=nominal(1:3)+CBF_V9_LowReference(t,6,options);
     nominal(4)=-lam^2*x(19)-2*lam*x(20);
     maxNom=max(maxNom,norm(nominal,Inf));
     solution=cbf_solve_hocbf_qp(q,nominal,eye(4));
     if solution.exitflag<=0 || isempty(solution.nu)
       status='QP_failed';firstBadTime=t;break;
     end
     nu=solution.nu;
     residual=q.A*nu-q.b;
     [worst,j]=min(residual./(1+abs(q.b)));
     minResidual=min(minResidual,worst);
     if worst<-1e-8
       status='QP_row_residual_negative';firstBadLabel=q.labels{j};firstBadTime=t;break;
     end
     maxNu=max(maxNu,norm(nu,Inf));
     if ~all(isfinite(nu)) || maxNu>limit
       status='huge_virtual_input_diagnostic_stop';firstBadTime=t;break;
     end
     xNext=CBF_Audit_State('stepvirtual',x,nu,dt);
     if ~all(isfinite(xNext)) || norm(xNext,Inf)>1e8
       status='state_divergence';firstBadTime=t;break;
     end
     model=cbf_model_diagnostics('virtual',xNext,s);
     if ~model.flatMapValid || model.tensionUnderTautAssumption<=0
       status='virtual_flatness_domain_failed';firstBadTime=t;break;
     end
     [h,labs]=cbf_evaluate_scene('virtual',shape,xNext,s,{obs});
     [mval,j]=min(h);minGeom=min(minGeom,mval);
     gm=startsWith(string(labs),'obs1_ground_');
     if any(gm),minGround=min(minGround,min(h(gm)));end
     if mval<-1e-8
       status='geometry_violation';firstBadLabel=labs{j};firstBadTime=t;break;
     end
     x=xNext;stepDone=k;phaseReached=max(phaseReached,min(3,ceil(k*dt/1.2)));
     maxError=max(maxError,norm(x(1:3)-CBF_V9_LowReference(k*dt,0,options)));
    end
   catch ME
      status='MATLAB_ERROR';errorText=[ME.identifier ': ' ME.message];
   end
   if strcmp(status,'completed')
      try
         qFinal=cbf_hocbf_scene('virtual',shape,x,s,{obs});
         minPsi=min(minPsi,min(cellfun(@min,qFinal.psi)));
         if ~all(qFinal.psiInitiallyNonnegative)
            status='final_auxiliary_set_failed';
         end
      catch MEfinal
         status='MATLAB_ERROR';errorText=[MEfinal.identifier ': ' MEfinal.message];
      end
   end
   distance=norm(x(1:3)-xInit(1:3));
   ref=CBF_V9_LowReference(steps*dt,0,options);
   rows(end+1,:)={id,shape,kind,route,status,stepDone,steps,dt,...
      initErr,initialLP,minGeom,minGround,minPsi,minResidual,...
      maxNom,maxNu,maxError,distance,norm(x(1:3)-ref),...
      phaseReached,firstBadTime,firstBadLabel,errorText}; %#ok<AGROW>
  end
 end
end
report=cell2table(rows,'VariableNames',{'caseId','shape','obstacleType','route',...
 'status','completedSteps','requestedSteps','dt','initial0to5Mismatch',...
 'initialLPFeasible','minimumGeometryH','minimumGroundH','minimumAuxiliaryPsi',...
 'minRowScaledResidual','maxNominal','maxQPDecision','maxTrackingError',...
 'displacement','finalTrackingError','phaseReached','firstFailureTime',...
 'firstFailureLabel','errorText'});
writetable(report,fullfile(CBF_V9_Util('folder'),'v9_low_altitude_virtual_rollouts.csv'));
fprintf('[V9理想仮想系] %d条件、最後まで完走=%d（実機追従・回避成功とは別）\n',...
 height(report),sum(strcmp(report.status,'completed')));
end

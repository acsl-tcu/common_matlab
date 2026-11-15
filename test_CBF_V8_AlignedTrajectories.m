function report=test_CBF_V8_AlignedTrajectories(options)
% 仮想系の整合初期状態から3D参照を実際に追うテスト。実HLCでない。
% nominalは6階チェーンの解析フィードバック。CBFを緩めず、異常を分類。
if nargin<1,options=struct();end
paths=test_CBF_SetupPaths();s=test_CBF_Fixture();
if isfield(options,'settings'),s=options.settings;end
full=getOpt(options,'full',false);
dt=getOpt(options,'dt',.02);
if full,steps=getOpt(options,'steps',150);else,steps=getOpt(options,'steps',55);end
T=getOpt(options,'trajectoryDuration',3);
lambda=getOpt(options,'trackingGain',1.6);
if full,shapes={'C','B','D'};else,shapes={'B','D'};end
names={'hover','line','circle','ascent','figure8'};
rows=cell(0,18);id=0;
xr=CBF_Audit_State('real',s,1.5,0,0,0,0,0,0);
x0=CBF_Audit_State('virtualfromreal',xr,s);
p0=x0(1:3);
for shapeCell=shapes
 shape=shapeCell{1};
 for kindCell={'sphere','ellipsoid'}
  kind=kindCell{1};
  for nameCell=names
   name=nameCell{1};
   if strcmp(name,'ascent'),direction='above';dist=.48;
   else,direction='front';dist=.52;end
   obs=CBF_Audit_State('obstacle',kind,p0,dist,direction,.8);
   % このテストは障害物1個。多障害物試験は旧suiteのtest_CBF_AuditLongApproachを利用。
   obstacles={obs};
   x=x0;status='completed';stepCount=0;maxNom=0;maxNu=0;
   minH=Inf;minPsi=Inf;maxTrackingError=0;minResidual=Inf;exitflag=NaN;
   initialError=0;initialFeasible=false;
   for j=0:5
      ref=CBF_V8_Reference(name,0,j,p0,T);
      initialError=max(initialError,norm(x0(3*j+1:3*j+3)-ref,Inf));
   end
   if initialError>1e-10,error('CBF:V8BadInitialAlignment','0～5階が開始時に一致しません');end
   try
     for k=1:steps
       t=(k-1)*dt;
       qp=cbf_hocbf_scene('virtual',shape,x,s,obstacles);
       if k==1
          lp=CBF_Audit_LP(qp);initialFeasible=lp.feasible;
       end
       minPsi=min(minPsi,min(cellfun(@min,qp.psi)));
       if ~all(qp.psiInitiallyNonnegative)
         status='HOCBF_auxiliary_set_violated';break;
       end
       nuNom=zeros(4,1);
       for j=0:5
          ref=CBF_V8_Reference(name,t,j,p0,T);
          nuNom(1:3)=nuNom(1:3)-nchoosek(6,j)*lambda^(6-j)*...
               (x(3*j+1:3*j+3)-ref);
       end
       nuNom(1:3)=nuNom(1:3)+CBF_V8_Reference(name,t,6,p0,T);
       nuNom(4)=-2*lambda*x(20)-lambda^2*x(19);
       maxNom=max(maxNom,norm(nuNom,Inf));
       sol=cbf_solve_hocbf_qp(qp,nuNom,eye(4));exitflag=sol.exitflag;
       if exitflag<=0 || isempty(sol.nu)
         status='QP_infeasible_or_solver_failure';break;
       end
       minResidual=min(minResidual,min(qp.A*sol.nu-qp.b));
       if minResidual<-1e-7*(1+norm(qp.b,Inf))
         status='QP_residual_negative';break;
       end
       maxNu=max(maxNu,norm(sol.nu,Inf));
       if ~all(isfinite(sol.nu)) || maxNu>1e7
         status='nonphysical_virtual_decision_input';break;
       end
       xnew=CBF_Audit_State('stepvirtual',x,sol.nu,dt);
       if ~all(isfinite(xnew)) || norm(xnew,Inf)>1e7
         status='state_divergence';break;
       end
       diag=cbf_model_diagnostics('virtual',xnew,s);
       if ~diag.flatMapValid || diag.tensionUnderTautAssumption<=0
         status='invalid_flat_mapping_or_tension';break;
       end
       h=cbf_evaluate_scene('virtual',shape,xnew,s,obstacles);
       minH=min(minH,min(h));
       if any(~isfinite(h)) || min(h)<-1e-8
         status='geometry_violated_after_step';break;
       end
       refNext=CBF_V8_Reference(name,k*dt,0,p0,T);
       maxTrackingError=max(maxTrackingError,norm(xnew(1:3)-refNext));
       x=xnew;stepCount=k;
     end
   catch ME
     status=['MATLAB_ERROR_' ME.identifier];
   end
   displacement=norm(x(1:3)-p0);
   target=CBF_V8_Reference(name,steps*dt,0,p0,T);
   finalError=norm(x(1:3)-target);
   id=id+1;
   rows(end+1,:)={id,shape,kind,name,steps,stepCount,dt,status,...
       initialError,initialFeasible,minH,minPsi,maxNom,maxNu,...
       minResidual,displacement,finalError,maxTrackingError}; %#ok<AGROW>
  end
 end
end
report=cell2table(rows,'VariableNames',{'caseId','shape','obstacleType','reference',...
 'requestedSteps','completedSteps','dt','status','initialDerivativeMismatch',...
 'initialQPFeasible','minH','minPsi','maxNominalInput','maxQPDecisionInput',...
 'minQPResidual','loadDisplacement','finalTrackingError','maxTrackingError'});
folder=fullfile(paths.generator,'CBF','test_results');if ~exist(folder,'dir'),mkdir(folder);end
writetable(report,fullfile(folder,'v8_aligned_reference_rollouts.csv'));
fprintf('[V8 整合参照] 完走 %d/%d。変位・誤差を確認するまで回避成功と扱わない。\n',...
 sum(strcmp(report.status,'completed')),height(report));
end
function v=getOpt(a,f,d)
if isfield(a,f)&&~isempty(a.(f)),v=a.(f);else,v=d;end
end

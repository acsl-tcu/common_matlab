function results=test_CBF_AuditLongApproach20261112(options)
%TEST_CBF_AUDITLONGAPPROACH  既存CBFで直線/円/上昇/8の字と障害物へ近づく閉ループ。
% C/B/D、球/任意回転楕円体は別条件。QPの安全行にスラックなし。
% 名目入力は理想6重積分器の解析的追従入力であり、既存HLCではない。
% 実入力系の初期条件試験は既存AuditRolloutsをそのまま呼んで分離記録。
if nargin<1,options=struct();end
paths=test_CBF_SetupPaths();s=test_CBF_Fixture();
if isfield(options,'settings'),s=options.settings;end
full=getopt(options,'full',true);
if full,defaultSteps=150;else,defaultSteps=40;end
steps=getopt(options,'steps',defaultSteps);
dt=getopt(options,'dt',.01);
refGain=getopt(options,'referenceGain',1.6);
if full,distances=[.65 1.10];sceneSizes=[1 3];
else,distances=[.85];sceneSizes=1;end
names={'hover','line','circle','ascent','figure8'};
shapes={'C','B','D'};kinds={'sphere','ellipsoid'};
rows=cell(0,20); idx=0;
for nameCell=names
 name=nameCell{1};
 for sh=shapes
  shape=sh{1};
  for kindCell=kinds
   kind=kindCell{1};
   for dist=distances
    for nObs=sceneSizes
    idx=idx+1; status='completed';firstProblem='';completed=0;
    maxNu=0;maxD=0; minH=Inf;minPsi=Inf;maxDecisionCorrection=0;
    qpFeasibleSteps=0;trajectoryRmsErr=NaN;
    try
       % 全軌道で同一の物理モデルから、吊り荷の0～5階初期値を構成。
       vx=0.2;if strcmp(name,'hover'),vx=0;end
       xr=CBF_Audit_State('real',s,1.5,0,0,0,0,vx,0);
       x=CBF_Audit_State('virtualfromreal',xr,s);
       ob=CBF_Audit_State('obstacle',kind,xr(8:10),dist,'front',.6);
       obs={ob};
       if nObs>=2
           obs{2}=CBF_Audit_State('obstacle',kind,xr(8:10),1.05,'side',-.4);
       end
       if nObs>=3
           obs{3}=CBF_Audit_State('obstacle',kind,xr(8:10),1.30,'above',.75);
       end
       xInitial=x;errSq=0;
       [hInitial,~]=cbf_evaluate_scene('virtual',shape,x,s,obs);
       minH=min(hInitial);
       for k=1:steps
           t=(k-1)*dt;
           qp=cbf_hocbf_scene('virtual',shape,x,s,obs);
           minPsi=min(minPsi,min(cellfun(@min,qp.psi)));
           if ~all(qp.psiInitiallyNonnegative)
               jj=find(~qp.psiInitiallyNonnegative,1);
               status='initial_or_intermediate_HOCBF_set_violation';
               firstProblem=qp.labels{jj};break;
           end
           % 解析軌道の微分と、(D+lambda)^6 e = 0 の追従名目入力。
           nuNom=zeros(4,1);
           for j=0:5
               desired=CBF_Audit_State('reference',name,t,j);
               actual=x(3*j+1:3*j+3);
               nuNom(1:3)=nuNom(1:3)-nchoosek(6,j)*...
                    refGain^(6-j)*(actual-desired);
           end
           nuNom(1:3)=nuNom(1:3)+CBF_Audit_State('reference',name,t,6);
           nuNom(4)=-2*refGain*x(20)-refGain^2*x(19);
           sol=cbf_solve_hocbf_qp(qp,nuNom,eye(4));
           if sol.exitflag<=0 || ~isfinite(sol.minResidual) || ...
                   sol.minResidual < -1e-7*(1+norm(qp.b,Inf))
               status='qp_infeasible_or_solver_failure';firstProblem='hard_QP';break;
           end
           qpFeasibleSteps=qpFeasibleSteps+1;
           maxNu=max(maxNu,norm(sol.nu,Inf));
           maxDecisionCorrection=max(maxDecisionCorrection,norm(sol.nu-nuNom,Inf));
           if norm(sol.nu,Inf)>1e6
               status='huge_virtual_input';firstProblem='input_norm_above_1e6';break;
           end
           xnext=CBF_Audit_State('stepvirtual',x,sol.nu,dt);
           if any(~isfinite(xnext)) || norm(xnext(1:18),Inf)>1e6
               status='state_divergence';firstProblem='virtual_state';break;
           end
           diag=cbf_model_diagnostics('virtual',xnext,s);
           if ~diag.flatMapValid || diag.tensionUnderTautAssumption<=0
               status='taut_cable_or_flat_map_invalid';break;
           end
           [hh,~]=cbf_evaluate_scene('virtual',shape,xnext,s,obs);
           minH=min(minH,min(hh));
           if min(hh)<-1e-8
               status='geometric_collision_after_step';break;
           end
           desired=CBF_Audit_State('reference',name,t+dt,0);
           errSq=errSq+norm(xnext(1:3)-desired)^2;
           x=xnext;completed=k;
       end
       maxD=norm(x(1:3)-xInitial(1:3));
       if completed>0,trajectoryRmsErr=sqrt(errSq/completed);end
       % 最終点の補助集合も点検し、終端だけの違反を見落とさない。
       if strcmp(status,'completed')
           qp=cbf_hocbf_scene('virtual',shape,x,s,obs);
           if ~all(qp.psiInitiallyNonnegative)
               status='final_HOCBF_set_violation';
           end
       end
    catch ME
       status='matlab_error';firstProblem=[ME.identifier ' ' ME.message];
    end
    rows(end+1,:)={idx,'virtual',shape,kind,name,dist,nObs,steps,completed,...
       status,firstProblem,minH,minPsi,maxNu,maxDecisionCorrection,...
       maxD,trajectoryRmsErr,qpFeasibleSteps,dt,refGain}; %#ok<AGROW>
    end
   end
  end
 end
end
results.virtual=cell2table(rows,'VariableNames',...
 {'caseId','mode','systemShape','obstacleType','reference','obstacleDistance','obstacleCount',...
 'requestedSteps','completedSteps','status','firstProblem',...
 'minimumGeometricSafety','minimumHOCBFPsi','maxVirtualInput',...
 'maxNominalCorrection','loadDisplacement','trackingRMSError',...
 'QPStepsSolved','dt','referenceGain'});
folder=fullfile(paths.generator,'CBF','test_results');if ~exist(folder,'dir'),mkdir(folder);end
writetable(results.virtual,fullfile(folder,'audit4_virtual_long_approach.csv'));
% 実入力の様々な初期速度・姿勢は既存の非HLC名目入力・実プラントで検査。
% この試験で実入力軌道追従が成立したと主張しない。
if getopt(options,'runRealEntry',true)
    ropt=struct('settings',s,'steps',min(steps,40),'dt',dt,...
        'shapes',{{'C','B','D'}},'obstacleTypes',{{'sphere','ellipsoid'}});
    results.realEntry=test_CBF_AuditRollouts(ropt);
end
fprintf('[接近/参照] 仮想系 %d条件、完走=%d、初期違反=%d。結果は達成距離と合わせて評価。\n',...
    size(results.virtual,1),sum(strcmp(results.virtual.status,'completed')),...
    sum(contains(results.virtual.status,'HOCBF_set_violation')));
end
function v=getopt(o,k,d),if isfield(o,k)&&~isempty(o.(k)),v=o.(k);else,v=d;end,end

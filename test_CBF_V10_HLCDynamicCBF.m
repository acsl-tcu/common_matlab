function [report,history]=test_CBF_V10_HLCDynamicCBF(options)
%TEST_CBF_V10_HLCDYNAMICCBF 実HLCのraw入力を名目として動的拡張系を比較。
% 注意: HLCのraw [T;tau] をCBF決定入力[T4;tau2]に代入することはできない。
% 本試験はT(4)・tau(2)を用いる明示したTEST-ONLY追従フィルタで接続。
% フィルタは元HLCには存在しない。制御保証/本番統合と混同しないこと。
if nargin<1,options=struct();end
s=localOpt(options,'settings',test_CBF_Fixture());
H=CBF_V10_HLC('init',s,options);dt=H.dt;
full=localOpt(options,'full',false);
if full,routes={'hover','line','lowcycle'}; shapes={'C','B','D'};
else,routes={'hover','lowcycle'};shapes={'B','D'};end
modes={'filter_only','cbf_hard'};
rows=cell(0,22); trace=cell(0,19);id=0;
wt=localOpt(options,'thrustFilterBandwidth',2.0);
wq=localOpt(options,'torqueFilterBandwidth',3.0);
assert(wt>0 && wq>0,'CBF:V10FilterGain','帯域は正値');
for routeCell=routes
 route=routeCell{1};
 for shCell=shapes
  shape=shCell{1};
  for modCell=modes
   mode=modCell{1};id=id+1;
   x=CBF_Audit_State('real',s,localOpt(options,'zHigh',.45),0,0,0,0,0,0);
   status='completed';err='';badLabel='';completed=0;
   minGround=Inf;minGeometry=Inf;minPsi=Inf;minInput=Inf;minTension=Inf;
   minQPres=Inf;maxHLC=0;maxNu=0;maxTracking=0;maxCBFCorrection=0;
   minBeta=Inf;firstFailure=NaN;
   N=ceil(4.5/dt);
   if ~full,N=min(N,35);end
   if strcmp(route,'hover'),N=min(N,ceil(.7/dt));end
   try
     % 初期のTとtauを実HLCのraw出力に合わせる。ただし高階微分は0。
     cmd0=CBF_V10_HLC('command',H,x,s,route,0,options);
     if ~cmd0.withinInputLimits
       status='INITIAL_HLC_RAW_OUTSIDE_INPUT_LIMITS';
     else
       x(20)=cmd0.rawPhysicalInput(1);
       x(24:26)=cmd0.rawPhysicalInput(2:4);
     end
   catch ME
     status='MATLAB_ERROR';err=[ME.identifier ': ' ME.message];
   end
   if strcmp(status,'completed')
    for k=1:N
     t=(k-1)*dt;
     try
       cmd=CBF_V10_HLC('command',H,x,s,route,t,options);
       desired=cmd.rawPhysicalInput;
       maxHLC=max(maxHLC,norm(desired,Inf));
       minBeta=min(minBeta,cmd.rcondBeta);
       if ~cmd.withinInputLimits
          status='HLC_RAW_OUTSIDE_INPUT_LIMITS';firstFailure=t;break;
       end
       d=cbf_model_diagnostics('real',x,s);
       margin=CBF_V9_Util('groundmargin',x,s);
       minGround=min(minGround,min(margin));
       [geometry,geomLabels]=cbf_evaluate_scene('real',shape,x,s,{localObstacle(route)});
       minGeometry=min(minGeometry,min(geometry));
       if any(geometry < -1e-8)
         status='PHYSICAL_GEOMETRY_VIOLATION';firstFailure=t;
         jj=find(geometry < -1e-8,1);badLabel=geomLabels{jj};break;
       end
       minTension=min(minTension,d.tension);
       bounds=[x(20)-s.input.thrustMin;s.input.thrustMax-x(20);...
           x(24:26)-s.input.torqueMin(:);s.input.torqueMax(:)-x(24:26)];
       minInput=min(minInput,min(bounds));
       if min(margin)<-1e-9,status='GROUND_VIOLATED';firstFailure=t;break;end
       if min(bounds)<-1e-8,status='PHYSICAL_INPUT_VIOLATED';firstFailure=t;break;end
       if d.tension<=0 || d.quaternionNormError>1e-4 || ...
            d.cableDirectionNormError>1e-4
         status='MODEL_DOMAIN_FAILED';firstFailure=t;break;
       end
       % HOLDした希望入力を追う4次/2次安定多項式。これは任意の別の
       % アクチュエータフィルタであり、HLCのT^(4)/tau''ではない。
       nuNom=[wt^4*(desired(1)-x(20))-4*wt^3*x(21)-...
          6*wt^2*x(22)-4*wt*x(23);...
          wq^2*(desired(2:4)-x(24:26))-2*wq*x(27:29)];
       nu=nuNom;corr=0;psiMin=NaN;res=NaN;
       if strcmp(mode,'cbf_hard')
         obs=localObstacle(route);
         qp=cbf_hocbf_scene('real',shape,x,s,{obs});
         psiMin=min(cellfun(@min,qp.psi));minPsi=min(minPsi,psiMin);
         if ~all(qp.psiInitiallyNonnegative)
           status='HOCBF_INITIAL_OR_LATER_AUXILIARY_FAILED';
           jj=find(~qp.psiInitiallyNonnegative,1);
           badLabel=qp.labels{jj};firstFailure=t;break;
         end
         sol=cbf_solve_hocbf_qp(qp,nuNom,eye(4));
         if sol.exitflag<=0 || isempty(sol.nu)
           status='HARD_QP_INFEASIBLE';firstFailure=t;break;
         end
         nu=sol.nu;
         residual=(qp.A*nu-qp.b)./(1+abs(qp.b));
         [res,jj]=min(residual);
         minQPres=min(minQPres,res);
         if res < -1e-8
           status='HARD_QP_RESIDUAL_FAILED';firstFailure=t;
           badLabel=qp.labels{jj};break;
         end
         corr=norm(nu-nuNom,Inf);
         maxCBFCorrection=max(maxCBFCorrection,corr);
       end
       if any(~isfinite(nu)) || norm(nu,Inf)>1e12
         status='NONFINITE_OR_HUGE_EXTENSION_DECISION';firstFailure=t;break;
       end
       maxNu=max(maxNu,norm(nu,Inf));
       r=CBF_V10_Reference(route,t,0,options);
       tracking=norm(x(8:10)-r);maxTracking=max(maxTracking,tracking);
       trace(end+1,:)={id,route,shape,mode,t,x(8),x(9),x(10),x(13),...
           x(20),x(24),x(25),x(26),desired(1),...
           norm(desired(2:4),Inf),norm(nuNom,Inf),norm(nu,Inf),...
           psiMin,corr}; %#ok<AGROW>
       xNext=CBF_Audit_State('rk4real',x,nu,dt,s);
       if any(~isfinite(xNext)) || norm(xNext,Inf)>1e9
         status='EXTENDED_PLANT_DIVERGED';firstFailure=t;break;
       end
       x=xNext;completed=k;
     catch ME
       status='MATLAB_ERROR';err=[ME.identifier ': ' ME.message];firstFailure=t;break;
     end
    end
   end
   rows(end+1,:)={id,route,shape,mode,status,completed,N,dt,...
       minGround,minGeometry,minTension,minInput,minPsi,minQPres,maxHLC,...
       maxNu,maxCBFCorrection,maxTracking,minBeta,x(10),firstFailure,...
       [badLabel ' ' err]}; %#ok<AGROW>
  end
 end
end
report=cell2table(rows,'VariableNames',{'caseId','route','shape','variant','status',...
 'completedSteps','requestedSteps','dt','minGroundMargin','minGeometryH','minTension',...
 'minPhysicalInputMargin','minHOCBFPsi','minScaledQPResidual','maxHLCRawPhysicalInput',...
 'maxDecisionInput','maxCBFCorrection','maxTrackingError','minRcondBeta2',...
 'finalLoadZ','failureTime','failureRowOrError'});
history=cell2table(trace,'VariableNames',{'caseId','route','shape','variant',...
 'time','pLx','pLy','pLz','vLz','actualT','actualTauX','actualTauY','actualTauZ',...
 'HLCRawT','HLCRawTauInf','nominalExtensionInputInf','QPDecisionInputInf',...
 'minHOCBFPsi','CBFCorrectionInf'});
out=CBF_V9_Util('folder');
writetable(report,fullfile(out,'v10_hlc_extension_cbf_summary.csv'));
writetable(history,fullfile(out,'v10_hlc_extension_cbf_timeline.csv'));
fprintf('[V10 HLC＋試験用動的拡張CBF] %d件、完走%d（HLCとの厳密同値ではない）\n',...
 size(report,1),sum(strcmp(report.status,'completed')));
end
function obs=localObstacle(route)
if strcmp(route,'hover')
 obs=struct('type','sphere','center',[100;0;0.45],'radius',.12);
else
 obs=struct('type','sphere','center',[.30;0;.24],'radius',.12);
end
end
function v=localOpt(s,key,def)
if isfield(s,key) && ~isempty(s.(key)),v=s.(key);else,v=def;end
end

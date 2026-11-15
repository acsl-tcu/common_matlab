function report=test_CBF_V9_LowAltitudeRealCBF(options)
% [5c] 実29状態＋実入力動的拡張＋地面/入力ハードCBF-QPの閉ループ。
% 名目nu=[m_total*z_ref^(6),0,0,0]。HLCの全状態追従制御ではない。
% 障害物行は削り、地面と実入力限界の動作だけを独立検証する。
if nargin<1,options=struct();end
s=CBF_V9_Util('options',options,'settings',test_CBF_Fixture());
full=CBF_V9_Util('options',options,'full',false);
if full,gears=[1.2 3];postures=[0 .12];else,gears=[1.2 3];postures=0;end
dt=CBF_V9_Util('options',options,'realCBFDt',.02);
N=ceil(3*CBF_V9_Util('options',options,'phaseDuration',1.2)/dt);
mt=s.physical.massDrone+s.physical.massLoad;
rows=cell(0,19);id=0;
for gain=gears
 for pitch=postures
  id=id+1;st=s;st.cbf.groundGain=gain;
  zH=CBF_V9_Util('options',options,'zHigh',.45);
  x=CBF_Audit_State('real',st,zH,0,0,pitch,0,0,0);
  status='completed';err='';completed=0;
  minPhysicalMargin=Inf;minPsi=Inf;minResidual=Inf;
  minTension=Inf;minInputMargin=Inf;maxNom=0;maxNu=0;
  maxRefErr=0;firstFailure=NaN;firstLabel='';
  for k=1:N
   t=(k-1)*dt;
   try
    q=CBF_V9_Util('groundqp','real','B',x,st);
    margins=CBF_V9_Util('groundmargin',x,st);
    minPhysicalMargin=min(minPhysicalMargin,min(margins));
    d=cbf_model_diagnostics('real',x,st);
    minTension=min(minTension,d.tension);
    minInputMargin=min(minInputMargin,min([x(20)-st.input.thrustMin;...
       st.input.thrustMax-x(20);x(24:26)-st.input.torqueMin(:);...
       st.input.torqueMax(:)-x(24:26)]));
    minPsi=min(minPsi,min(cellfun(@min,q.psi)));
    if min(margins)<-1e-8
      status='physical_ground_violation';firstFailure=t;break;
    end
    if d.tension<=0 || d.quaternionNormError>1e-4 || d.cableDirectionNormError>1e-4
      status='model_domain_failure';firstFailure=t;break;
    end
    if minInputMargin<-1e-5
      status='real_input_bound_violation';firstFailure=t;break;
    end
    if ~all(q.psiInitiallyNonnegative)
      status='HOCBF_auxiliary_set_failed';firstFailure=t;
      loc=find(~q.psiInitiallyNonnegative,1);firstLabel=q.labels{loc};break;
    end
    dz6=CBF_V9_LowReference(t,6,options);
    nominal=[mt*dz6(3);0;0;0];
    maxNom=max(maxNom,norm(nominal,Inf));
    sol=cbf_solve_hocbf_qp(q,nominal,eye(4));
    if sol.exitflag<=0 || isempty(sol.nu)
      status='QP_no_solution';firstFailure=t;break;
    end
    residual=q.A*sol.nu-q.b;
    [worst,loc]=min(residual./(1+abs(q.b)));
    minResidual=min(minResidual,worst);
    if worst<-1e-8
      status='QP_row_residual_failed';firstFailure=t;firstLabel=q.labels{loc};break;
    end
    maxNu=max(maxNu,norm(sol.nu,Inf));
    xnew=CBF_Audit_State('rk4real',x,sol.nu,dt,st);
    if any(~isfinite(xnew)) || norm(xnew,Inf)>1e9
      status='real_state_divergence';firstFailure=t;break;
    end
    x=xnew;completed=k;
    ref=CBF_V9_LowReference(k*dt,0,options);
    maxRefErr=max(maxRefErr,abs(x(10)-ref(3)));
   catch ME
    status='MATLAB_ERROR';err=[ME.identifier ': ' ME.message];break;
   end
  end
  if all(isfinite(x))
    lastMargin=CBF_V9_Util('groundmargin',x,st);
    minPhysicalMargin=min(minPhysicalMargin,min(lastMargin));
    lastDiag=cbf_model_diagnostics('real',x,st);
    minTension=min(minTension,lastDiag.tension);
    minInputMargin=min(minInputMargin,min([x(20)-st.input.thrustMin;...
      st.input.thrustMax-x(20);x(24:26)-st.input.torqueMin(:);...
      st.input.torqueMax(:)-x(24:26)]));
    if strcmp(status,'completed') && min(lastMargin)<-1e-8
      status='physical_ground_violation';
    end
    if strcmp(status,'completed') && minInputMargin<-1e-5
      status='real_input_bound_violation';
    end
    if strcmp(status,'completed') && lastDiag.tension<=0
      status='model_domain_failure';
    end
    if strcmp(status,'completed')
       try
          finalQ=CBF_V9_Util('groundqp','real','B',x,st);
          minPsi=min(minPsi,min(cellfun(@min,finalQ.psi)));
          if ~all(finalQ.psiInitiallyNonnegative)
             status='final_auxiliary_set_failed';
          end
       catch MEnd
          status='MATLAB_ERROR';err=[MEnd.identifier ': ' MEnd.message];
       end
    end
  elseif strcmp(status,'completed')
    status='real_state_divergence';
  end
  refEnd=CBF_V9_LowReference(N*dt,0,options);
  rows(end+1,:)={id,gain,pitch,status,completed,N,dt,minPhysicalMargin,...
    minPsi,minResidual,minTension,minInputMargin,maxNom,maxNu,maxRefErr,...
    abs(x(10)-refEnd(3)),firstFailure,firstLabel,err}; %#ok<AGROW>
 end
end
report=cell2table(rows,'VariableNames',{'caseId','groundGain','pitch','status',...
 'completedSteps','requestedSteps','dt','minimumGroundMargin','minimumPsi',...
 'minimumScaledQPResidual','minimumTension','minimumActualInputMargin',...
 'maximumNominalInput','maximumQPDecision','maximumZTrackingError',...
 'finalZError','firstFailureTime','firstFailureLabel','errorText'});
writetable(report,fullfile(CBF_V9_Util('folder'),'v9_low_altitude_real_cbf_rollouts.csv'));
fprintf('[V9実入力CBF閉ループ] %d条件、完走%d。障害物/HLCは未接続。\n',...
 height(report),sum(strcmp(report.status,'completed')));
end

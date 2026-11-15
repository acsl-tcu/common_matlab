function report=test_CBF_V9_AttitudeCable(options)
% [6] roll / pitch / ケーブル角度 / 角速度が初期集合・QP可行性・停止に及ぼす影響。
% 機体質量は既知、吊り荷の推定質量だけを物理モデルに使用する。
if nargin<1,options=struct();end
s=CBF_V9_Util('options',options,'settings',test_CBF_Fixture());
full=CBF_V9_Util('options',options,'full',false);
if full
  altitudes=[.2,.3,.5];speeds=[-.25,-.65];
  attitudes=[0 0 0;0 .15 0;.15 0 0;0 .15 .12;.15 .15 .12;.20 .2 .2];
  angularRates=[0 .2];
else
  altitudes=[.2,.5];speeds=[-.25];
  attitudes=[0 0 0;0 .15 .12;.15 .1 .1];angularRates=[0 .2];
end
rows=cell(0,22);id=0;
for z=altitudes
 for vz=speeds
  for i=1:size(attitudes,1)
   roll=attitudes(i,1);pitch=attitudes(i,2);tilt=attitudes(i,3);
   for wr=angularRates
    for sh={'B','D'}
     shape=sh{1};id=id+1;err='';
     hL=NaN;hD=NaN;tension=NaN;minPsi=NaN;gpsi=false;
     hard=false;effect=NaN;nZero=NaN;stop=false;stopMargin=NaN;
     brakeTension=NaN;modelInvalid=false;initialT=NaN;
     try
      x=CBF_Audit_State('real',s,z,vz,roll,pitch,tilt,0,wr);
      d=cbf_model_diagnostics('real',x,s);
      margin=CBF_V9_Util('groundmargin',x,s);
      hL=margin(1);hD=margin(2);tension=d.tension;initialT=x(20);
      q=CBF_V9_Util('groundqp','real',shape,x,s);
      minPsi=min(cellfun(@min,q.psi));gpsi=all(q.psiInitiallyNonnegative);
      lp=CBF_Audit_LP(q);hard=lp.feasible;
      effect=min(q.effectNorm);nZero=sum(q.uncontrollableViolation);
      brake=CBF_V9_BrakeRollout(x,s,.15,options);
      stop=brake.stopped && ~brake.modelInvalid && ~brake.groundViolated;
      stopMargin=min(brake.minMargins);brakeTension=brake.minTension;
      modelInvalid=brake.modelInvalid;
     catch ME
       err=[ME.identifier ': ' ME.message];
     end
     rows(end+1,:)={id,shape,z,vz,roll,pitch,tilt,wr,hL,hD,...
        tension,minPsi,gpsi,hard,effect,nZero,stop,stopMargin,...
        brakeTension,modelInvalid,initialT,err}; %#ok<AGROW>
    end
   end
  end
 end
end
report=cell2table(rows,'VariableNames',{'caseId','shape','loadAltitude','loadVz',...
 'roll','pitch','cableTilt','angularRateScale','loadMargin','droneMargin',...
 'initialTension','minimumPsi','allInitialPsiOK','groundInputLPFeasible',...
 'minimumDecisionInputEffectNorm','uncontrollableRows','brakeSafeStopObserved',...
 'minBrakingMargin','minimumBrakingTension','brakeModelInvalid',...
 'initialThrust','errorText'});
writetable(report,fullfile(CBF_V9_Util('folder'),'v9_attitude_cable_map.csv'));
fprintf('[V9姿勢・ケーブル] %d条件、初期集合+QP両立=%d、試験入力で安全停止=%d\n',...
 height(report),sum(report.allInitialPsiOK&report.groundInputLPFeasible),...
 sum(report.brakeSafeStopObserved));
end

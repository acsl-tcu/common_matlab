function report=test_CBF_V9_Braking(options)
% [3] 実29状態モデルで推力の滑らかな立ち上がりを積分し、地面/張力/入力制限を確認。
% 入力は事前に定めたC3滑らかなT(t)、tau(t)=0、QP介入なし。
% 『この一つの制御が安全に停止した』の証拠。可到達集合の証明ではない。
if nargin<1,options=struct();end
s=CBF_V9_Util('options',options,'settings',test_CBF_Fixture());
full=CBF_V9_Util('options',options,'full',false);
if full,alts=[.20,.30,.50,.75];speeds=[-.25,-.65,-1.2];
 transitions=[.15,.35,.50];configs=[0 0 0;0 .12 .1;.12 .12 .1;.2 .2 .2];
else,alts=[.30,.50];speeds=[-.65,-1.2];
 transitions=[.15,.50];configs=[0 0 0;0 .12 .1];end
rows=cell(0,25);id=0;
for alt=alts
 for vz=speeds
  for tr=transitions
   for ci=1:size(configs,1)
    roll=configs(ci,1);pitch=configs(ci,2);tilt=configs(ci,3);
    id=id+1;err='';stopped=false;groundViolation=false;
    invalid=false;safe=false;stopT=NaN;hL=NaN;hD=NaN;hC=NaN;
    tmin=NaN;tmax=NaN;Tinitial=NaN;tension=NaN;
    initPsi=false;initQP=false;stopReason='not_run';
    finalVz=NaN;lastTime=NaN;
    try
     x=CBF_Audit_State('real',s,alt,vz,roll,pitch,tilt,0,0);
     q=CBF_V9_Util('groundqp','real','B',x,s);
     initPsi=all(q.psiInitiallyNonnegative);
     lp=CBF_Audit_LP(q);initQP=lp.feasible;
     out=CBF_V9_BrakeRollout(x,s,tr,options);
     stopped=out.stopped;groundViolation=out.groundViolated;
     invalid=out.modelInvalid;safe=stopped && ~groundViolation && ~invalid;
     stopT=out.stopTime;hL=out.minMargins(1);hD=out.minMargins(2);
     hC=out.minMargins(3);tmin=out.minThrust;tmax=out.maxThrust;
     Tinitial=out.initialThrust;tension=out.minTension;
     stopReason=out.reason;finalVz=out.finalLoadVz;lastTime=out.elapsed;
    catch ME
      err=[ME.identifier ': ' ME.message];
      stopReason='MATLAB_ERROR';
    end
    rows(end+1,:)={id,alt,vz,tr,roll,pitch,tilt,stopped,stopT,...
      hL,hD,hC,groundViolation,invalid,safe,initPsi,initQP,...
      Tinitial,tmin,tmax,tension,finalVz,lastTime,stopReason,err}; %#ok<AGROW>
   end
  end
 end
end
report=cell2table(rows,'VariableNames',{'caseId','loadAltitude','loadVz',...
 'thrustRiseSeconds','roll','pitch','cableTilt','stopped',...
 'stopTime','minLoadGroundMargin','minDroneGroundMargin',...
 'minCableGroundMargin','groundViolation','modelInvalid','safeStopObserved',...
 'initialAllAuxiliaryOK','initialGroundInputLPFeasible',...
 'initialThrust','minThrust','maxThrust','minimumTension',...
 'finalLoadVz','elapsed','reason','errorText'});
writetable(report,fullfile(CBF_V9_Util('folder'),'v9_real_braking_rollouts.csv'));
fprintf('[V9元モデル制動] %d試験、制動・地面・張力・入力制限すべて成立 %d\n',...
 height(report),sum(report.safeStopObserved));
end

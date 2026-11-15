function report=test_CBF_V9_LowAltitudeReal(options)
% [5b] 29状態モデルを鉛直下降→低高度滞在→再上昇させる入力指定試験。
% 高度参照からT=m_total*(g+z_ref'')を作り、決定入力nuT=m_total*z_ref^(6)。
% 水平方向のHLC追従制御ではない。発生した地面CBF残差は診断のみ。
% QPの不等式を満たさなくても勝手に入力を変えたりスラックを入れない。
if nargin<1,options=struct();end
s=CBF_V9_Util('options',options,'settings',test_CBF_Fixture());
full=CBF_V9_Util('options',options,'full',false);
dt=CBF_V9_Util('options',options,'realDt',.005);
if full,configs=[0 0 0;0 .12 .1;.12 .12 .1];
else,configs=[0 0 0;0 .12 .1];end
T=3*CBF_V9_Util('options',options,'phaseDuration',1.2);
N=ceil(T/dt);mTot=s.physical.massDrone+s.physical.massLoad;
rows=cell(0,20);seq=0;
for k=1:size(configs,1)
  seq=seq+1;r=configs(k,1);p=configs(k,2);tilt=configs(k,3);
  x=CBF_Audit_State('real',s,CBF_V9_Util('options',options,'zHigh',.45),...
                   0,r,p,tilt,0,0);
  minMargins=inf(3,1);minTension=Inf;minInputMargin=Inf;
  minGroundHOCBFResidual=Inf;minPsi=Inf;minH=Inf;
  violated=false;auxFailed=false;lpFailed=false;modelBad=false;
  actualSteps=0;errorText='';lastState=x;maxZRefError=0;
  for j=1:N
   t=(j-1)*dt;
   try
    mg=CBF_V9_Util('groundmargin',x,s);
    minMargins=min(minMargins,mg);
    diag=cbf_model_diagnostics('real',x,s);
    minTension=min(minTension,diag.tension);
    minInputMargin=min(minInputMargin,min([x(20)-s.input.thrustMin;...
        s.input.thrustMax-x(20);x(24:26)-s.input.torqueMin(:);...
        s.input.torqueMax(:)-x(24:26)]));
    if any(mg<0),violated=true;end
    if diag.tension<=0 || diag.quaternionNormError>1e-4 ||...
         diag.cableDirectionNormError>1e-4 || ~all(isfinite(x))
       modelBad=true;break;
    end
    % 現行HOCBFへの適合は計算するが、入力への上書きはしない。
    if mod(j-1,25)==0
      q=CBF_V9_Util('groundqp','real','B',x,s);
      [gm,im]=CBF_V9_Util('masks',q);
      minPsi=min(minPsi,min(cellfun(@min,q.psi(gm))));
      if ~all(q.psiInitiallyNonnegative(gm|im)),auxFailed=true;end
      lp=CBF_Audit_LP(q,gm|im);
      if ~lp.feasible,lpFailed=true;end
      refSix=CBF_V9_LowReference(t,6,options);
      requested=[mTot*refSix(3);0;0;0];
      resid=q.A*requested-q.b;
      minGroundHOCBFResidual=min(minGroundHOCBFResidual,min(resid./(1+abs(q.b))));
    end
    u1=localU(t,mTot,options);u2=localU(t+dt/2,mTot,options);
    u4=localU(t+dt,mTot,options);
    a=test_CBF_IndependentPlantRHS(x,u1,s);
    b=test_CBF_IndependentPlantRHS(x+.5*dt*a,u2,s);
    c=test_CBF_IndependentPlantRHS(x+.5*dt*b,u2,s);
    d=test_CBF_IndependentPlantRHS(x+dt*c,u4,s);
    lastState=x;
    x=x+dt*(a+2*b+2*c+d)/6;
    actualSteps=j;
    referenceNow=CBF_V9_LowReference(j*dt,0,options);
    maxZRefError=max(maxZRefError,abs(x(10)-referenceNow(3)));
   catch ME
    errorText=[ME.identifier ': ' ME.message];break;
   end
  end
  % 最終積分点も評価する。最後の一歩での接触や上限違反を見落とさない。
  if all(isfinite(x))
    finalMargin=CBF_V9_Util('groundmargin',x,s);
    minMargins=min(minMargins,finalMargin);
    diagFinal=cbf_model_diagnostics('real',x,s);
    minTension=min(minTension,diagFinal.tension);
    minInputMargin=min(minInputMargin,min([x(20)-s.input.thrustMin;...
      s.input.thrustMax-x(20);x(24:26)-s.input.torqueMin(:);...
      s.input.torqueMax(:)-x(24:26)]));
    if any(finalMargin<0),violated=true;end
    if diagFinal.tension<=0 || diagFinal.quaternionNormError>1e-4 ||...
      diagFinal.cableDirectionNormError>1e-4,modelBad=true;end
  else,modelBad=true;end
  finalRef=CBF_V9_LowReference(actualSteps*dt,0,options);
  status='completed';
  if ~isempty(errorText),status='MATLAB_ERROR';
  elseif modelBad,status='model_invalid';
  elseif violated,status='ground_crossed';
  elseif minInputMargin<-1e-5,status='physical_input_limit_exceeded';end
  rows(end+1,:)={seq,r,p,tilt,status,actualSteps,N,dt,minMargins(1),...
   minMargins(2),minMargins(3),minTension,minInputMargin,...
   minGroundHOCBFResidual,minPsi,auxFailed,lpFailed,...
   maxZRefError,abs(x(10)-finalRef(3)),errorText}; %#ok<AGROW>
end
report=cell2table(rows,'VariableNames',{'caseId','roll','pitch','cableTilt',...
 'status','completedSteps','requestedSteps','dt','minLoadMargin','minDroneMargin',...
 'minCableMargin','minTension','minActualInputMargin','minRequestedHOCBFScaledResidual',...
 'minGroundPsi','auxiliarySetEverFailed','groundInputLPEverFailed',...
 'maxZTrackingError','finalZTrackingError','errorText'});
writetable(report,fullfile(CBF_V9_Util('folder'),'v9_low_altitude_real_feedforward.csv'));
fprintf('[V9実29状態] %d条件。鉛直入力指定（HLC/横移動は含まない）。\n',height(report));
end
function nu=localU(t,mt,opts)
ref6=CBF_V9_LowReference(t,6,opts);
nu=[mt*ref6(3);0;0;0];
end

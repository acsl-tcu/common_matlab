function result=CBF_V9_BrakeRollout(x0,s,tr,options)
% CBF_V9_BRAKEROLLOUT 元の29状態非線形モデルでの試験用開ループ停止。
% 推力はT0からTmaxまでC3スムーズステップで増やす。トルクは初期値のまま。
% 物理的に可能なひとつの入力を示すだけであり、制御可能集合の証明ではない。
% CBF/QPにこの入力を押し付けず、入力制限違反・地面接触も隠さず記録する。
if nargin<4,options=struct();end
step=CBF_V9_Util('options',options,'brakeDt',.002);
maxT=CBF_V9_Util('options',options,'brakeMaxTime',1.5);
if step<=0 || maxT<=0 || tr<=0,error('CBF:V9BrakeTime','時間は正値');end
x=x0(:);T0=x(20);Tmax=s.input.thrustMax;
if T0>Tmax || T0<s.input.thrustMin,error('CBF:V9BrakeThrust','初期推力が制限外');end
minMargin=inf(3,1);minTension=inf;minThrust=inf;maxThrust=-inf;
maxTorque=0;maxCableError=0;maxQuatError=0;
stopped=(x(13)>=0);stopTime=NaN;if stopped,stopTime=0;end
violated=false;invalid=false;reason='time_limit';t=0;lastState=x;minControlResidual=NaN;
for j=0:ceil(maxT/step)
  margins=CBF_V9_Util('groundmargin',x,s);
  minMargin=min(minMargin,margins);
  diag=cbf_model_diagnostics('real',x,s);
  minTension=min(minTension,diag.tension);
  minThrust=min(minThrust,x(20));maxThrust=max(maxThrust,x(20));
  maxTorque=max(maxTorque,norm(x(24:26),Inf));
  maxCableError=max(maxCableError,diag.cableDirectionNormError);
  maxQuatError=max(maxQuatError,diag.quaternionNormError);
  if any(margins<0),violated=true;end
  if diag.tension<=0 || ~isfinite(diag.tension) || ...
     diag.cableDirectionNormError>1e-4 || diag.quaternionNormError>1e-4 ||...
     ~all(isfinite(x))
      invalid=true;reason='model_invalid';break;
  end
  if x(20)<s.input.thrustMin-1e-5 || x(20)>Tmax+1e-5 ||...
     ~diag.torqueInBounds
      invalid=true;reason='real_input_bound_violation';break;
  end
  if x(13)>=0 && j>0
      stopped=true;stopTime=t;reason='vertical_speed_stopped';break;
  end
  if j==ceil(maxT/step),break;end
  k1=test_CBF_IndependentPlantRHS(x,localNu(t,T0,Tmax,tr),s);
  k2=test_CBF_IndependentPlantRHS(x+step*k1/2,localNu(t+step/2,T0,Tmax,tr),s);
  k3=test_CBF_IndependentPlantRHS(x+step*k2/2,localNu(t+step/2,T0,Tmax,tr),s);
  k4=test_CBF_IndependentPlantRHS(x+step*k3,localNu(t+step,T0,Tmax,tr),s);
  lastState=x;
  x=x+(step/6)*(k1+2*k2+2*k3+k4);
  t=t+step;
end
result=struct('stopped',stopped,'stopTime',stopTime,'minMargins',minMargin,...
 'minTension',minTension,'minThrust',minThrust,'maxThrust',maxThrust,...
 'maxTorque',maxTorque,'maxCableError',maxCableError,'maxQuaternionError',maxQuatError,...
 'groundViolated',violated,'modelInvalid',invalid,'reason',reason,'finalX',x,...
 'lastX',lastState,'elapsed',t,'initialThrust',T0,'transitionTime',tr,...
 'finalLoadVz',x(13),'minControlResidual',minControlResidual);
end
function nu=localNu(t,T0,Tmax,tr)
% 7次 smootherstep の4階微分。t>=trでT4=0。T,dT,d2T,d3Tが連続。
if t>=tr, fourth=0;
elseif t<=0,fourth=840;
else
  u=t/tr;
  fourth=840-10080*u+25200*u^2-16800*u^3;
end
nu=[(Tmax-T0)*fourth/tr^4;0;0;0];
end

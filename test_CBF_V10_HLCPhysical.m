function [summary,detail]=test_CBF_V10_HLCPhysical(options)
%TEST_CBF_V10_HLCPHYSICAL 実HLC生成関数→生の物理入力→19状態モデル。
% HLC.do()は呼ばない（do内のハードクリップを安全性の証拠にしない）。
% CBFフィルタはここではOFF。HLC名目制御の物理成立性だけを調べる。
if nargin<1,options=struct();end
s=localOption(options,'settings',test_CBF_Fixture());
H=CBF_V10_HLC('init',s,options);
dt=H.dt;full=localOption(options,'full',false);
if full
 routes={'hover','line','lowcycle','recover'};
 postures=[0 0 0 0; .10 0 0 0; 0 .12 0 0; 0 0 .12 0; 0 0 0 .12; .08 .10 .10 .10];
else
 routes={'hover','lowcycle'};postures=[0 0 0 0];
end
rows=cell(0,22);hist=cell(0,20);id=0;
for routeCell=routes
 route=routeCell{1};
 for postureIndex=1:size(postures,1)
  posture=postures(postureIndex,:);
  roll=posture(1);pitch=posture(2);tilt=posture(3);vx=posture(4);
  id=id+1;status='completed';err='';steps=0;
  maxT=0;maxTau=0;minMargin=inf;minTension=inf;minObsH=inf;
  maxTracking=0;minRcond=inf;maxQuaternion=0;maxCableError=0;
  x=CBF_Audit_State('real',s,localOption(options,'zHigh',.45),0,...
       roll,pitch,tilt,vx,0);
  x=x(1:19);
  N=ceil(4.5/dt);
  if ~full,N=min(N,80);end
  if strcmp(route,'hover'),N=min(N,ceil(1/dt));end
  for k=1:N
   t=(k-1)*dt;
   try
     cmd=CBF_V10_HLC('command',H,x,s,route,t,options);
     u=cmd.rawPhysicalInput;
     maxT=max(maxT,abs(u(1)));maxTau=max(maxTau,norm(u(2:4),Inf));
     minRcond=min(minRcond,cmd.rcondBeta);
     z=CBF_V10_Plant('state29',x,u);
     m=CBF_V9_Util('groundmargin',z,s);
     diag=cbf_model_diagnostics('real',z,s);
     minMargin=min(minMargin,min(m));minTension=min(minTension,diag.tension);
     maxQuaternion=max(maxQuaternion,diag.quaternionNormError);
     maxCableError=max(maxCableError,diag.cableDirectionNormError);
     obs=localObstacle();
     [h,labs]=cbf_evaluate_scene('real','B',z,s,{obs});
     % 地面行を除外し、障害物だけの形状安全関数を記録する。
     obsRows=~contains(string(labs),'ground_');
     if any(obsRows)
       minObsH=min(minObsH,min(h(obsRows)));
       if min(h(obsRows)) < -1e-9
         status='OBSTACLE_GEOMETRY_VIOLATED';break;
       end
     end
     r=CBF_V10_Reference(route,t,0,options);
     tracking=norm(x(8:10)-r);maxTracking=max(maxTracking,tracking);
     hist(end+1,:)={id,route,roll,pitch,tilt,vx,t,x(8),x(9),x(10),...
       x(11),x(12),x(13),u(1),u(2),u(3),u(4),...
       min(m),diag.tension,tracking}; %#ok<AGROW>
     if ~cmd.withinInputLimits,status='HLC_RAW_INPUT_OUTSIDE_LIMITS';break;end
     if diag.tension<=0 || diag.quaternionNormError>1e-4 || ...
           diag.cableDirectionNormError>1e-4
       status='MODEL_DOMAIN_FAILED';break;
     end
     if min(m)<-1e-9,status='GROUND_CONTACT';break;end
     % 遠方足切りはしない。ただしこのHLC単独試験は障害物で制御しない。
     xnew=CBF_V10_Plant('rk4',x,u,dt,s);
     if any(~isfinite(xnew)) || norm(xnew,Inf)>1e8
       status='STATE_DIVERGED';break;
     end
     x=xnew;steps=k;
   catch ME
     status='MATLAB_ERROR';err=[ME.identifier ': ' ME.message];break;
   end
  end
  rows(end+1,:)={id,route,roll,pitch,tilt,vx,status,steps,N,dt,maxT,maxTau,...
    minMargin,minTension,minObsH,maxTracking,minRcond,maxQuaternion,...
    maxCableError,x(10),x(8),err}; %#ok<AGROW>
 end
end
summary=cell2table(rows,'VariableNames',{'caseId','route','initialRoll','initialPitch','initialCableTilt','initialVx','status',...
 'completedSteps','requestedSteps','dt','maxRawThrust','maxRawTorque',...
 'minGroundMargin','minTension','minObstacleH','maxTrackingError',...
 'minRcondBeta2','maxQuaternionError','maxCableNormError','finalZ','finalX','errorText'});
detail=cell2table(hist,'VariableNames',{'caseId','route','initialRoll','initialPitch','initialCableTilt','initialVx','time',...
 'pLx','pLy','pLz','vLx','vLy','vLz','rawThrust','rawTauX','rawTauY','rawTauZ',...
 'minGroundMargin','tension','trackingError'});
out=CBF_V9_Util('folder');
writetable(summary,fullfile(out,'v10_hlc_physical_summary.csv'));
writetable(detail,fullfile(out,'v10_hlc_physical_timeline.csv'));
fprintf('[V10 HLC実入力→物理モデル] %d条件、完走%d、raw入力クリップなし\n',...
 size(summary,1),sum(strcmp(summary.status,'completed')));
end
function obs=localObstacle()
obs=struct('type','sphere','center',[.30;0;.24],'radius',.12);
end
function v=localOption(s,name,def)
if isfield(s,name)&&~isempty(s.(name)),v=s.(name);else,v=def;end
end

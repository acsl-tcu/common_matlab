function results=test_CBF_V8_GroundStructure(options)
% V8 地面制約: 現行6次HOCBF vs 初期集合 vs 入力ハード可行性 vs 鉛直制動候補。
% 低次の入力作用は「初期の物理推力・トルク」を摂動して検査する。
% 低次候補は診断のみ。実入力系の決定入力はT^(4),tau''なので、
% Tを即時に変更できる低次CBFを本番QPに使ってよいという意味ではない。
if nargin<1,options=struct();end
paths=test_CBF_SetupPaths();s=test_CBF_Fixture();
if isfield(options,'settings'),s=options.settings;end
folder=fullfile(paths.generator,'CBF','test_results');if ~exist(folder,'dir'),mkdir(folder);end
full=getOpt(options,'full',false);
if full
 altitudes=[.5 .75 1.2 1.8 3];speeds=[-1.2 -.65 -.25 0 .4];
 pitches=[0 .20];tilts=[0 .12];gains=[1.2 2 3];
else
 altitudes=[.65 1.5];speeds=[-1.2 -.65 0];
 pitches=[0 .18];tilts=0;gains=[1.2 3];
end
rows=cell(0,19);id=0;
for gain=gains
 for z=altitudes
  for vz=speeds
   for p=pitches
    for tilt=tilts
       id=id+1;d='';[gpsi,gqp,ipsi,iqp]=deal(false);hmin=NaN;
       stopD=NaN;brakeH=NaN;oldMinPsi=NaN;minResid=NaN;
       allfeas=false;onlyG=false;onlyI=false;isolated=false;
       try
        st=s;st.cbf.groundGain=gain;
        xr=CBF_Audit_State('real',st,z,vz,0,p,tilt,0,0);
        obs=CBF_Audit_State('obstacle','sphere',xr(8:10),100,'front',0);
        qp=cbf_hocbf_scene('real','B',xr,st,{obs});
        gm=contains(qp.labels,'ground_');
        im=contains(qp.labels,'_thrust_')|contains(qp.labels,'_tau_');
        gpsi=all(qp.psiInitiallyNonnegative(gm));
        ipsi=all(qp.psiInitiallyNonnegative(im));
        rg=CBF_Audit_LP(qp,gm);onlyG=rg.feasible;
        ri=CBF_Audit_LP(qp,im);onlyI=ri.feasible;
        rgi=CBF_Audit_LP(qp,gm|im);gqp=rgi.feasible;
        ra=CBF_Audit_LP(qp);allfeas=ra.feasible;
        psiG=qp.psi(gm);oldMinPsi=min(cellfun(@min,psiG));
        groundGeom=cbf_evaluate_case('real','B','sphere',xr,st,obs);
        hmin=min(groundGeom(end-3:end));
        P=cbf_parameter_vector(st);
        jet=cbf_real_state_taylor(xr,zeros(4,1),P,1);
        vzD=jet(10,2)-P(3)*jet(16,2);
        hD=xr(10)-P(3)*xr(16)-st.shape.droneRadius-st.ground.height-st.cbf.safetyMargin;
        hL=xr(10)-st.shape.loadRadius-st.ground.height-st.cbf.safetyMargin;
        accel=st.input.thrustMax/(st.physical.massDrone+st.physical.massLoad)-st.physical.gravity;
        if accel>0
          stopD=max(0,-vz)^2/(2*accel);
          brakeH=min(hD-max(0,-vzD)^2/(2*accel),hL-stopD);
        end
        [nu,~,~,minResid]=solveHard(qp,zeros(4,1)); %#ok<ASGLU>
        isolated=true;
       catch ME
        d=[ME.identifier ': ' ME.message];
       end
       rows(end+1,:)={id,z,vz,p,tilt,gain,hmin,oldMinPsi,gpsi,ipsi,...
         onlyG,onlyI,gqp,allfeas,stopD,brakeH,minResid,isolated,d}; %#ok<AGROW>
    end
   end
  end
 end
end
results.grid=cell2table(rows,'VariableNames',{'caseId','loadAltitude','loadVerticalSpeed',...
  'pitch','cableTilt','groundGain','minGroundH','minGroundPsi','groundInitialPsiOK',...
  'inputInitialPsiOK','groundOnlyFeasible','inputOnlyFeasible','groundAndInputFeasible',...
  'allIncludingFarObstacleFeasible','optimisticStopDistance','optimisticBrakingMargin',...
  'QPminimumResidual','evaluationCompleted','errorText'});
writetable(results.grid,fullfile(folder,'v8_ground_design_grid.csv'));
% 「Tを即時に変えられる」と仮定した低次安全関数が誤用されないよう、
% 現在の物理T・tauを微小摂動して、地面生微分への初出を別記録する。
results.physicalAction=physicalActionAudit(s);
writetable(results.physicalAction,fullfile(folder,'v8_ground_physical_action_orders.csv'));
% 推力を実際にT0->Tmaxへ滑らかに移す（T,dT,ddT,T3を連続にする）
% 理想鉛直・張り紐の不変部分系での参考試験。全3Dで停止可能の証明ではない。
results.verticalBrake=verticalBrakeAudit(s);
writetable(results.verticalBrake,fullfile(folder,'v8_ground_vertical_braking.csv'));
fprintf('[V8 地面] %d条件：初期集合OK=%d, 地面+入力QP可行=%d, 両立=%d。\n',...
 height(results.grid),sum(results.grid.groundInitialPsiOK),...
 sum(results.grid.groundAndInputFeasible),...
 sum(results.grid.groundInitialPsiOK & results.grid.groundAndInputFeasible));
end
function result=physicalActionAudit(s)
P=cbf_parameter_vector(s);
x=CBF_Audit_State('real',s,1.5,0,0,0,0,0,0);
% 実入力T,tauの物理状態値を変えて同じ運動方程式の生微分を比較。
rows=cell(0,9);id=0;vareps=1e-4;
for posture=[0 .15]
 x(1:4)=[cos(posture/2);0;sin(posture/2);0];
 % Tは基準ホバリング相当（傾斜時は完全な平衡ではない）
 x(20)=(s.physical.massDrone+s.physical.massLoad)*s.physical.gravity/cos(posture);
 for component=1:4
  for target={'ground_drone','ground_load'}
   nm=target{1};delta=zeros(29,1);
   ix=[20 24 25 26];delta(ix(component))=vareps;
   p=rawDerivatives(x+delta,P,nm);m=rawDerivatives(x-delta,P,nm);
   sens=(p-m)/(2*vareps);first=find(abs(sens)>1e-7,1);
   if isempty(first), firstOrder=NaN;else,firstOrder=first-1;end
   id=id+1;
   rows(end+1,:)={id,posture,nm,component,firstOrder, ...
    sens(2),sens(3),sens(5),mat2str(sens,8)}; %#ok<AGROW>
  end
 end
end
result=cell2table(rows,'VariableNames',{'caseId','pitch','groundLabel',...
 'physicalInputComponent','firstObservedDerivativeOrder','hDotSensitivity',...
 'hDDotSensitivity','hFourthSensitivity','allOrders0to6'});
end
function d=rawDerivatives(x,P,target)
Z=cbf_real_state_taylor(x,zeros(4,1),P,6);
if strcmp(target,'ground_drone'),coeff=Z(10,:)-P(3)*Z(16,:);
else,coeff=Z(10,:);end
d=coeff.*factorial(0:6);
end
function result=verticalBrakeAudit(s)
m=s.physical.massDrone+s.physical.massLoad;g=s.physical.gravity;
T0=m*g;Tmax=s.input.thrustMax;
if Tmax<T0,error('CBF:HoverExceedsThrust','ホバリング推力が上限を超えています');end
rows=cell(0,9);id=0;
for z=[.5 .75 1.2 1.8]
 for vz=[-1.2 -.65 -.25]
  for Tr=[.15 .5]
   id=id+1;
   t=0:.001:3;sw=min(t/Tr,1);
   smooth=35*sw.^4-84*sw.^5+70*sw.^6-20*sw.^7;
   T=T0+(Tmax-T0)*smooth;
   a=T/m-g;
   v=vz+cumtrapz(t,a);
   zz=z+cumtrapz(t,v);
   stop=find(v>=0,1);
   if isempty(stop),stopped=false;stopT=NaN;minHeight=min(zz);
   else,stopped=true;stopT=t(stop);minHeight=min(zz(1:stop));end
   minMargin=minHeight-s.shape.loadRadius-s.cbf.safetyMargin-s.ground.height;
   rows(end+1,:)={id,z,vz,Tr,stopped,stopT,minMargin,...
        min(T),max(T)}; %#ok<AGROW>
  end
 end
end
result=cell2table(rows,'VariableNames',{'caseId','loadAltitude','initialVz',...
 'thrustTransitionTime','verticalStopAchieved','stopTime','minLoadFloorMargin',...
 'minThrust','maxThrust'});
end
function [u,flag,ok,mr]=solveHard(qp,u0)
u=nan(4,1);flag=NaN;ok=false;mr=NaN;
sol=cbf_solve_hocbf_qp(qp,u0,eye(4));flag=sol.exitflag;
if flag>0,u=sol.nu;mr=sol.minResidual;ok=mr>=-1e-7*(1+norm(qp.b,Inf));end
end
function v=getOpt(a,f,d)
if isfield(a,f)&&~isempty(a.(f)),v=a.(f);else,v=d;end
end

function [report,timeline]=test_CBF_V13_R1GroundRollout(opts)
%TEST_CBF_V13_R1GROUNDROLLOUT R1 physical [T;tau], ground-only CBF on 19-state model.
% Diagnostic: nonlinear RK4 held-input closed loop, no clipping/slack.
% This is NOT a HLC rollout and does NOT enforce obstacle constraints.
% It is a first comparison of r=2 ground CBF vs physical stopping ability.
if nargin<1,opts=struct();end
test_CBF_SetupPaths();s=test_CBF_Fixture();
if ~isfield(opts,'dt'),opts.dt=0.02;end
if ~isfield(opts,'full'),opts.full=false;end
if ~isscalar(opts.dt) || opts.dt<=0 || ~isfinite(opts.dt)
    error('CBF:V13InvalidDt','dt must be positive and finite.');
end
if opts.full
    ids=[0 2 3]; zStarts=[0.35 0.50];vzStarts=[0 -0.25 -0.65];
else
    ids=[0 2]; zStarts=0.50;vzStarts=[0 -0.25];
end
shapes={'C','B','D'};gains=[1.2 3.0];
% The 0..2.3 s reference descends smoothly, stays low, then recovers.
endTime=2.3;steps=ceil(endTime/opts.dt);
R=cell(0,17);T=cell(0,12);nr=0;nt=0;
for id=ids
 for zstart=zStarts
  for vz0=vzStarts
   for si=1:numel(shapes)
    for gain=gains
      shape=shapes{si};s.cbf.groundGain=gain;
      if id==0,[x,~]=CBF_V12_PureHover(s);
      else,[x,~]=CBF_V11_Examples(id,s,'sphere',100);end
      x(10)=zstart;x(13)=vz0;
      status='COMPLETED_POINTWISE_ONLY';err='';good=0;minH=inf;minPsi=inf;
      minTension=inf;minInputMargin=inf;minQPres=inf;maxThrust=0;
      maxVz=abs(vz0);maxTrack=0;minQuaternion=inf;maxQuaternion=0;
      for step=1:steps
        t=(step-1)*opts.dt;
        [zr,vzr,azr]=localVerticalReference(t,zstart,0.22);
        % Explicit diagnostic physical-input nominal (not the actual HLC).
        KP=8;KD=5;
        Tnom=(s.physical.massDrone+s.physical.massLoad)* ...
             (s.physical.gravity+azr+KP*(zr-x(10))+KD*(vzr-x(13)));
        unom=[Tnom;0;0;0];
        msg='';
        try
          [A,b,h,psi]=localGroundQP(shape,x,s,unom,gain);
          minH=min(minH,min(h));minPsi=min(minPsi,min(psi));
          if min(h)<-1e-8,status='INITIAL_GEOMETRIC_GROUND_VIOLATION';break;end
          if min(psi)<-1e-8,status='INITIAL_CBF_AUXILIARY_SET_INVALID';break;end
          lb=[s.input.thrustMin;s.input.torqueMin(:)];
          ub=[s.input.thrustMax;s.input.torqueMax(:)];
          W=diag([1 10 10 10]);qpOpt=optimoptions('quadprog','Display','off');
          [u,~,exitflag]=quadprog(2*W,-2*W*unom,-A,-b,...
              [],[],lb,ub,[],qpOpt);
          if exitflag<=0 || isempty(u) || ~all(isfinite(u))
              status='QP_INFEASIBLE_OR_NUMERICAL_FAILURE';break;
          end
          qpRes=min([A*u-b;u-lb;ub-u]);
          minQPres=min(minQPres,qpRes);
          if ~isfinite(qpRes) || qpRes<-1e-7
              status='QP_SUCCESS_BUT_HARD_RESIDUAL_FAILURE';break;
          end
          minInputMargin=min(minInputMargin,min([u-lb;ub-u]));
          maxThrust=max(maxThrust,u(1));
          % Constant physical input during one dt. Check internal RK4 nodes.
          xn=x;
          for sub=1:4
              subdt=opts.dt/4;
              k1=localRHS(xn,u,s);
              k2=localRHS(xn+subdt*k1/2,u,s);
              k3=localRHS(xn+subdt*k2/2,u,s);
              k4=localRHS(xn+subdt*k3,u,s);
              xn=xn+subdt*(k1+2*k2+2*k3+k4)/6;
              % No model-state projection, no force clipping.
              hh=localGroundH(shape,xn,u,s);
              minH=min(minH,min(hh));
              if min(hh)<-1e-6
                  status='NONLINEAR_GROUND_COLLISION';break;
              end
              x29=[xn;u(1);zeros(3,1);u(2:4);zeros(3,1)];
              d=cbf_model_diagnostics('real',x29,s);
              minTension=min(minTension,d.tension);
              minQuaternion=min(minQuaternion,norm(xn(1:4)));
              maxQuaternion=max(maxQuaternion,norm(xn(1:4)));
              if ~isfinite(d.tension)||d.tension<=0
                  status='CABLE_NOT_TAUT_TENSION_FAILURE';break;
              end
              if d.quaternionNormError>1e-4 || ...
                      d.cableDirectionNormError>1e-4 ||...
                      d.cableAngularOrthogonality>1e-4
                  status='PHYSICAL_STATE_CONSTRAINT_DRIFT';break;
              end
          end
          if ~strcmp(status,'COMPLETED_POINTWISE_ONLY'),break;end
          x=xn;good=step;
          maxVz=max(maxVz,abs(x(13)));
          maxTrack=max(maxTrack,abs(x(10)-zr));
          nt=nt+1;
          T(nt,:)={id,zstart,vz0,shape,gain,t+opts.dt,...
             x(10),x(13),u(1),min(hh),min(psi),qpRes};
        catch ME
          msg=[ME.identifier ': ' ME.message];
          status='TEST_CODE_ERROR';err=msg;break;
        end
      end
      nr=nr+1;
      R(nr,:)={id,zstart,vz0,shape,gain,status,good,steps,...
         minH,minPsi,minTension,minInputMargin,minQPres,...
         maxThrust,maxVz,maxTrack,err};
    end
   end
  end
 end
end
report=cell2table(R,'VariableNames',...
 {'stateId','startAltitude_m','startVz_m_s','shape','gain','status',...
  'completedSteps','requestedSteps','minimumGroundH','minimumAuxiliaryPsi',...
  'minimumTension_N','minimumPhysicalInputMargin',...
  'minimumQPResidual','maximumThrust_N','maximumAbsVz_m_s',...
  'maximumTrackingError_m','errorText'});
timeline=cell2table(T,'VariableNames',...
 {'stateId','startAltitude_m','startVz_m_s','shape','gain','time_s',...
  'loadAltitude_m','loadVz_m_s','physicalThrust_N','groundH',...
  'auxiliaryPsi','QPResidual'});
out=CBF_V11_OutputFolder();
writetable(report,fullfile(out,'v13_r1_ground_rollout_summary.csv'));
writetable(timeline,fullfile(out,'v13_r1_ground_rollout_timeline.csv'));
fprintf('[V13] R1実物理19状態＋地面のみハードQP: %d条件。HLC/障害物未接続。\n',height(report));
end

function [A,b,h,psi]=localGroundQP(shape,x,s,unom,k)
ob=struct('type','sphere','center',[100;0;x(10)],'radius',0.25);
[H,labels]=CBF_V11_JetRows('R1',shape,x,zeros(4,1),s,ob,2);
keep=strncmp(labels,'ground_',7);H=H(keep,:);
if isempty(H),error('CBF:V13NoGround','No ground rows.');end
h=H(:,1);psi=H(:,2)+k*H(:,1);
b=-(2*H(:,3)+2*k*H(:,2)+k^2*H(:,1));
A=zeros(numel(b),4);
for j=1:4
    e=zeros(4,1);e(j)=1;
    Hp=CBF_V11_JetRows('R1',shape,x,e,s,ob,2);
    Hm=CBF_V11_JetRows('R1',shape,x,-e,s,ob,2);
    Hp=Hp(keep,:);Hm=Hm(keep,:);
    if max(abs(Hp(:,1:2)-Hm(:,1:2)),[],'all')>1e-8 || ...
      max(abs(Hp(:,3)+Hm(:,3)-2*H(:,3)))>1e-7
        error('CBF:V13R1NotAffine','Ground r2 does not satisfy input-affine condition.');
    end
    A(:,j)=Hp(:,3)-Hm(:,3);
end
if any(~isfinite([A(:);b(:);h(:);psi(:)]))
    error('CBF:V13R1Nonfinite','Ground QP has nonfinite coefficients.');
end
end

function h=localGroundH(shape,x,u,s)
ob=struct('type','sphere','center',[100;0;x(10)],'radius',0.25);
[H,labels]=CBF_V11_JetRows('R1',shape,x,u,s,ob,1);
h=H(strncmp(labels,'ground_',7),1);
end

function dx=localRHS(x,u,s)
Z=CBF_V11_RealTaylor('R1',x,u,s,1);
dx=Z(:,2);
end

function [z,dz,ddz]=localVerticalReference(t,zStart,zLow)
% Quintic descent 0..1.0s, hold 1.0..1.35s, ascend 1.35..2.3s.
if t<1
    [h,dh,ddh]=localMinJerk(t/1);
    z=zStart+(zLow-zStart)*h;
    dz=(zLow-zStart)*dh/1;
    ddz=(zLow-zStart)*ddh/1^2;
elseif t<1.35
    z=zLow;dz=0;ddz=0;
else
    duration=0.95;
    u=min(1,max(0,(t-1.35)/duration));
    [h,dh,ddh]=localMinJerk(u);
    z=zLow+(zStart-zLow)*h;
    dz=(zStart-zLow)*dh/duration;
    ddz=(zStart-zLow)*ddh/duration^2;
end
end

function [h,dh,ddh]=localMinJerk(t)
h=10*t^3-15*t^4+6*t^5;
dh=30*t^2-60*t^3+30*t^4;
ddh=60*t-180*t^2+120*t^3;
end

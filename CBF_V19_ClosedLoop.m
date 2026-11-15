function summary=CBF_V19_ClosedLoop(mode,scenario,opts)
% V19: real generated HLC nominal -> CBF candidate -> ORIGINAL 3D 19/23
% state dynamics; only cable bounding ellipsoid vs static rotated ellipsoids.
% Modes: NOMINAL, R1, R2-1, R2-3. R2-2/VIRTUAL explicitly NOT implemented
% as sound closed-loop controllers. NO ground/drone/load collision barriers.
% Multi-obstacle rows are COUPLED. Any safety/control failure stops this run.
% A sampled pass is not an intersample/recursive feasibility certificate.
if nargin<3, opts=struct();end
if ~isfield(opts,'dt'),opts.dt=.025;end
if ~isfield(opts,'substeps'),opts.substeps=4;end
if ~isfield(opts,'gamma'),opts.gamma=.06;end
if ~isfield(opts,'maxIterations'),opts.maxIterations=45;end
if ~isfield(opts,'parameterSet'),opts.parameterSet='R193';end
if ~isfield(opts,'outputDir'),opts.outputDir=fullfile(pwd,'cbf_v19_results');end
if ~isfield(opts,'stopOnViolation'),opts.stopOnViolation=true;end
if ~isfield(opts,'plant'),opts.plant='V11_19';end
if ~isfield(opts,'snapBandwidth'),opts.snapBandwidth=5;end
assert(opts.dt>0 && opts.substeps>=2 && opts.gamma>0 && opts.gamma<=1);
if ~isfolder(opts.outputDir),mkdir(opts.outputDir);end
mode=upper(char(mode));
assert(any(strcmp(mode,{'NOMINAL','R1','R2-1','R2-2','R2-3','VIRTUAL'})));
sc=CBF_V19_Scenarios(scenario,opts.parameterSet);
s=sc.settings;obs=sc.obstacles;N=numel(obs);sc.dt=opts.dt;
assert(N>=1);
% The real HLC 4-output mapping uses the generated HLC functions unchanged.
nominal=initHLC(s,opts.dt,sc.estimatedLoadMass);
x=sc.x0;
if any(strcmp(mode,{'R2-1','R2-2','R2-3'}))
 x=[x;(s.physical.massDrone+s.physical.massLoad)*s.physical.gravity;0;0;0];
end
if any(strcmp(mode,{'R2-2','VIRTUAL'}))
 status=['NOT_IMPLEMENTED_CLOSED_LOOP_' mode];
 summary=struct('mode',mode,'scenario',scenario,'status',status,'steps',0,...
 'parameterSet',opts.parameterSet,'plant',opts.plant,'nObstacles',N,...
 'completedSeconds',0,'minSampleBarrier',NaN,'minSampleTension_N',NaN,...
 'finalReferenceError_m',NaN,'firstFailedStepH',NaN,...
 'maxFilterMilliseconds',NaN,'maxRawNominalSaturation',NaN,...
 'errorText','Controller not constructed; V18 provides pointwise audit only.',...
 'scope','NOT_IMPLEMENTED_CLOSED_LOOP');
 fprintf(2,'[V19] %s: %s\n',scenario,status);
 return;
end
K=ceil(sc.duration/opts.dt); log=nan(K+1,23+N);last=0;
status='COMPLETED_SAMPLED_NOT_CERTIFIED';errorText='';firstFailedStepH=NaN;
for k=0:K
 t=min(k*opts.dt,sc.duration);
 try
  raw=nominalCommand(x(1:19),t,sc,nominal);
  % Nominal HLC source performs saturation. Expose saturation explicitly;
  % final CBF decisions are never silently clipped.
  lb=[s.input.thrustMin;s.input.torqueMin(:)];
  ub=[s.input.thrustMax;s.input.torqueMax(:)];
  unom=min(ub,max(lb,raw));sat=norm(unom-raw,Inf);
  [hs,tension]=geometry(x,obs,s,unom(1));
  if any(hs< -1e-9),status='INITIAL_OR_PROPAGATED_GEOMETRY_VIOLATION';break;end
  if tension<=0,status='NONPOSITIVE_CABLE_TENSION';break;end
  if strcmp(mode,'NOMINAL')||k==K
    if numel(x)==19, u=unom;else,u=[0;unom(2:4)];end
    residual=NaN;filterMs=0;
  else
    ticFilter=tic;
    if strcmp(mode,'R1')
      [u,residual,statusFilter]=filterR1(x,s,obs,unom);
    elseif strcmp(mode,'R2-1')
      [u,residual,statusFilter]=filterR21(x,s,obs,unom,opts);
    else
      [u,residual,statusFilter]=filterR23(x,s,obs,unom,opts);
    end
    filterMs=toc(ticFilter)*1000;
    if ~strcmp(statusFilter,'OK')
      status=['FILTER_STOP_' statusFilter];break;
    end
  end
  if any(~isfinite(u))
   status='NONFINITE_FILTER_INPUT';break;
  end
  if numel(x)==19,physicalU=u;else,physicalU=[x(20);u(2:4)];end
  if physicalU(1)<s.input.thrustMin-1e-7 || physicalU(1)>s.input.thrustMax+1e-7 || ...
      any(physicalU(2:4)<s.input.torqueMin(:)-1e-7) || ...
      any(physicalU(2:4)>s.input.torqueMax(:)+1e-7)
     status='APPLIED_INPUT_HARD_BOUNDS_VIOLATION';break;
  end
  pL=x(8:10);q=x(14:16);pD=pL-s.physical.cableLength*q;
  % [t,hMin,tension,rawT,rawTauInf,appliedT,appliedTauInf,j,filterMs,
  %  sat,reference xyz,load xyz,maxNormError,individual obstacle h's]
  row=[t,min(hs),tension,raw(1),norm(raw(2:4),Inf),...
       physicalU(1),norm(physicalU(2:4),Inf),...
       double(numel(x)==23)*u(1),filterMs,sat,sc.ref(t,0).',pL.',...
       max(abs([norm(x(1:4))-1,norm(q)-1,norm(q.'*x(17:19))])),...
       (pL-s.physical.cableLength*q/2).',q.',hs(:).'];
  log(k+1,:)=row;last=k+1;
  if k==K,break;end
  % CBF decision u is held for EXACTLY one control cycle (ZOH).
  if strcmpi(opts.plant,'R193_24')
   xnext=CBF_V19_OriginalPlantStep(x,u,s,opts.dt,opts.substeps);
  else
   xnext=integrate(x,u,s,opts.dt,opts.substeps);
  end
  [hNext,tNext]=geometry(xnext,obs,s,physicalU(1));
  if any(~isfinite(xnext)) || any(~isfinite(hNext))
    status='DYNAMICS_NONFINITE';break;
  end
  if opts.stopOnViolation && (any(hNext< -1e-7)||tNext<=0)
    status='PROPAGATION_SAMPLED_SAFETY_FAILURE';
    firstFailedStepH=min(hNext);break;
  end
  if max(abs([norm(xnext(1:4))-1,norm(xnext(14:16))-1,...
      xnext(14:16).'*xnext(17:19)]))>1e-3
    status='MANIFOLD_INVARIANT_DRIFT';break;
  end
  x=xnext;
 catch ME
  status=['ERROR_' ME.identifier];errorText=ME.message;
  fprintf(2,'[V19] %s %s step %d: %s\n',mode,scenario,k,ME.message);
  break;
 end
end
log=log(1:last,:);
names={'time_s','minCableBarrier','tension_N','rawHLCThrust_N',...
    'rawHLCTorqueInf_Nm','appliedThrust_N','appliedTorqueInf_Nm',...
    'snap_N_s4','filterMilliseconds','nominalSaturationInf',...
    'referenceX','referenceY','referenceZ','loadX','loadY','loadZ',...
    'manifoldError','cableCenterX','cableCenterY','cableCenterZ',...
    'cableAxisX','cableAxisY','cableAxisZ'};
for j=1:N,names{end+1}=sprintf('h_obstacle_%d',j);end
if isempty(log), tab=array2table(zeros(0,numel(names)),'VariableNames',names);
else,tab=array2table(log,'VariableNames',names);end
label=sprintf('v19_%s_%s',regexprep(lower(mode),'[^a-z0-9]','_'),scenario);
writetable(tab,fullfile(opts.outputDir,[label '_trace.csv']));
if isempty(log),minH=NaN;minT=NaN;maxMs=NaN;maxSat=NaN;
else,minH=min(log(:,2));minT=min(log(:,3));maxMs=max(log(:,9));maxSat=max(log(:,10));end
summary=struct('mode',mode,'scenario',scenario,'parameterSet',opts.parameterSet,...
 'nObstacles',N,'plant',opts.plant,'status',status,'completedSeconds',max(0,last-1)*opts.dt,...
 'steps',last,'minSampleBarrier',minH,'minSampleTension_N',minT,...
 'finalReferenceError_m',localFinalError(log),...
 'maxFilterMilliseconds',maxMs,'maxRawNominalSaturation',maxSat,...
 'firstFailedStepH',firstFailedStepH,...
 'errorText',errorText,'scope','SAMPLED_3D_GENERATED_HLC_OPENLOOP_STATE_NOT_FULL_FLIGHT_EKF_NO_INTERSAMPLE_CERTIFICATE');
fprintf('[V19] %-7s %-18s %-43s t=%6.2fs minH=% .4g maxFilter=%g ms\n',...
 mode,scenario,status,summary.completedSeconds,minH,maxMs);
end

function H=initHLC(s,dt,estimatedLoadMass)
if exist('lqrd','file')~=2,error('CBF:V19LQRD','Control System Toolbox lqrd is required.');end
if exist('HLC_SUSPENDED_LOAD','class')~=8
 error('CBF:V19HLCMissing','HLC_SUSPENDED_LOAD not on path: addpath(common_matlab/controller)');end
A2=[0 1;0 0];B2=[0;1];A6=diag(ones(5,1),1);B6=[zeros(5,1);1];
H.F1=lqrd(A2,B2,diag([150,10]),.01,dt);
H.F2=lqrd(A6,B6,diag([100,150,100,10,1,.1]),.001,dt);
H.F3=H.F2;
H.F4=lqrd(A2,B2,diag([10,1]),.01,dt);
H.obj=HLC_SUSPENDED_LOAD([],struct());
H.P=[s.physical.massDrone;s.physical.inertia(:);s.physical.gravity;...
    estimatedLoadMass;s.physical.cableLength;s.physical.disturbanceXY(:)].';
end
function u=nominalCommand(x,t,sc,H)
XD=zeros(1,28);for d=0:6,XD((1:3)+4*d)=sc.ref(t,d);end
% yaw and all higher derivatives zero in these three tests.
XD(4)=0;
vf=H.obj.Vfd_SuspendedLoadxyDst(sc.dt,x,XD,H.F1);
vs=H.obj.Vs_SuspendedLoadxyDst(x,XD,vf,H.P,H.F2,H.F3,H.F4);
uf=H.obj.Uf_SuspendedLoadxyDst(x,XD,vf,H.P);
beta=H.obj.Beta2_SuspendedLoadxyDst(x,XD,vf,H.P);
if ~isequal(size(beta),[3 3]) || rcond(beta)<1e-12
 error('CBF:V19BetaSingular','HLC Beta2 singular / ill-conditioned');end
alpha=H.obj.V2_alpha2_SuspendedLoadxyDst(x,XD,vf,vs(:),H.P);
u=[uf(1);beta\alpha(:)];
assert(all(isfinite(u)),'CBF:V19HLCNonfinite','HLC output nonfinite');
end
function [h,ten]=geometry(x,obs,s,thrustR1)
L=s.physical.cableLength;rc=s.shape.cableRadius;dm=s.cbf.safetyMargin;
a=L/2+rc;b=sqrt(a*rc);q=x(14:16);pL=x(8:10);c=pL-L*q/2;
h=zeros(numel(obs),1);
for i=1:numel(obs)
 o=obs{i};d=o.center(:)-c;
 supC=sqrt(b^2*(d.'*d)+(a*a-b*b)*(d.'*q)^2+s.cbf.smoothEpsilon^2);
 supO=sqrt(sum((o.axes(:).*(o.R.'*d)).^2)+s.cbf.smoothEpsilon^2);
 h(i)=d.'*d-supC-supO-dm*sqrt(d.'*d+s.cbf.smoothEpsilon^2);
end
quat=x(1:4);bt=[2*(quat(2)*quat(4)+quat(1)*quat(3));...
 2*(quat(3)*quat(4)-quat(1)*quat(2));...
 quat(1)^2-quat(2)^2-quat(3)^2+quat(4)^2];
if numel(x)==23,T=x(20);else,T=thrustR1;end
qd=cross(x(17:19),q);
% Tension must use the ACTUAL R1 thrust when supplied; call in filter below
scalar=(q.'*(T*bt)-s.physical.massDrone*L*(qd.'*qd))/...
 (s.physical.massDrone+s.physical.massLoad);
ten=-s.physical.massLoad*scalar;
end
function [u,res,status]=filterR1(x,s,obs,unom)
lo=s.input.thrustMin;hi=s.input.thrustMax;res=Inf;status='OK';
for i=1:numel(obs)
 r=CBF_V18_R1_RectifiedCable(x,s,obs{i});
 if r.hR< -1e-9,status='INITIAL_RECTIFIED_SET_INVALID';break;end
 if r.nonAffineH2>1e-7 || r.earlyTorqueEffect>1e-8
   status='NONAFFINE_OR_EARLY_TORQUE';break;end
 A=r.rectifiedCoefficient;rhs=r.rectifiedRHS;
 if abs(A)<1e-10
   if rhs>1e-9,status='ZERO_INPUT_AUTHORITY_POSITIVE_RHS';break;end
 elseif A>0,lo=max(lo,rhs/A);
 else,hi=min(hi,rhs/A);
 end
 res=min(res,A*unom(1)-rhs);
end
if lo>hi+1e-8,status='INFEASIBLE_SHARED_THRUST';end
if strcmp(status,'OK'),u=[min(max(unom(1),lo),hi);unom(2:4)];
else,u=nan(4,1);end
end
function [u,res,status]=filterR21(x,s,obs,unom,opts)
res=NaN;status='OK';A=zeros(0,4);rhs=zeros(0,1);
for i=1:numel(obs)
 r=CBF_V18_R2_1_RectifiedCable4(x,s,obs{i},struct('dt',opts.dt));
 if r.hRectified3< -1e-9,status='INITIAL_RECTIFIED_SET_INVALID';break;end
 if r.nonaffineAudit>1e-6 || r.earlyInitialEffect>1e-8 || abs(r.inputRow(1))>1e-7
  status='MIXED_ORDER_OR_NONAFFINE_NOT_VALIDATED';break;end
 A=[A;r.inputRow];rhs=[rhs;r.rhs];
end
if ~strcmp(status,'OK'),u=nan(4,1);return;end
[lo,hi]=snapBox(x,s,opts.dt);
nom=[nomSnap(x,unom(1),opts);unom(2:4)];nom(1)=min(max(nom(1),lo),hi);
try
  scale=[max(1,abs(hi-lo)/2);1;1;1];
  op=optimoptions('quadprog','Display','off');
  [v,~,ef]=quadprog(2*eye(4),-2*(nom./scale),-A*diag(scale),-rhs,...
         [],[],[lo;s.input.torqueMin(:)]./scale,...
         [hi;s.input.torqueMax(:)]./scale,[],op);
  if ef<=0||isempty(v),status='SHARED_TORQUE_QP_INFEASIBLE';u=nan(4,1);return;end
  u=v.*scale;res=min(A*u-rhs);
  if res< -1e-7,status='SHARED_QP_RESIDUAL_VIOLATION';end
catch ME
 status=['QP_ERROR_' ME.identifier];u=nan(4,1);
end
end
function [u,res,status]=filterR23(x,s,obs,unom,opts)
res=NaN;status='OK';dt=opts.dt;
[lo,hi]=snapBox(x,s,dt);
nom=[nomSnap(x,unom(1),opts);unom(2:4)];
nom(1)=min(max(nom(1),lo),hi);
scale=[max(1,abs(hi-lo)/2);1;1;1];
lb=[lo;s.input.torqueMin(:)]./scale;ub=[hi;s.input.torqueMax(:)]./scale;
[h0,ten0]=geometry(x,obs,s,0);
if any(h0<0)||ten0<=0,status='INITIAL_GEOMETRY_OR_TENSION';u=nan(4,1);return;end
[rNom,~]=residual(nom);
if all(isfinite(rNom)) && min(rNom)>=-1e-8,u=nom;res=min(rNom);return;end
try
  op=optimoptions('fmincon','Algorithm','sqp','Display','off',...
    'MaxIterations',opts.maxIterations,'MaxFunctionEvaluations',1200,...
    'ConstraintTolerance',1e-8);
  [v,~,ef]=fmincon(@(v)sum((v.*scale-nom).^2./(scale.^2)),...
    nom./scale,[],[],[],[],lb,ub,@nlc,op);
  u=v.*scale;[r,~]=residual(u);res=min(r);
  if ef<=0||any(~isfinite(u))||res< -1e-7
    status='SAMPLED_OPTIMIZATION_FAILED_OR_INFEASIBLE';end
catch ME
 status=['FMINCON_ERROR_' ME.identifier];u=nan(4,1);
end
 function [c,ceq]=nlc(v)
   [r,~]=residual(v.*scale);c=-r;ceq=[];
 end
 function [r,info]=residual(v)
   cur=x;H=dt/opts.substeps;mins=inf(numel(obs),1);
   tmin=Inf;
   for j=0:opts.substeps
      [hj,tj]=geometry(cur,obs,s,0);mins=min(mins,hj);tmin=min(tmin,tj);
      if j<opts.substeps
         cur=integrate(cur,v,s,H,1);
      end
   end
   [hend,~]=geometry(cur,obs,s,0);
   thExt=thrustBounds(x,v(1),dt);
   r=[mins;hend-(1-opts.gamma)*h0;tmin;...
       thExt(1)-s.input.thrustMin;s.input.thrustMax-thExt(2);...
       v(2:4)-s.input.torqueMin(:);s.input.torqueMax(:)-v(2:4)];
   if any(~isfinite(r)),r(:)=-1e12;end
   info=[];
 end
end
function j=nomSnap(x,Tref,opts)
w=opts.snapBandwidth;
j=w^4*(Tref-x(20))-4*w^3*x(21)-6*w^2*x(22)-4*w*x(23);
end
function [lo,hi]=snapBox(x,s,dt)
base=x(20)+dt*x(21)+dt^2*x(22)/2+dt^3*x(23)/6;
lo=24*(s.input.thrustMin-base)/dt^4;
hi=24*(s.input.thrustMax-base)/dt^4;
end
function ext=thrustBounds(x,j,dt)
coef=[j/6,x(23)/2,x(22),x(21)];tt=[0;dt];
if any(abs(coef)>1e-14)
 rt=roots(coef);ix=abs(imag(rt))<1e-9 & real(rt)>=0 & real(rt)<=dt;
 tt=[tt;real(rt(ix))];
end
th=x(20)+x(21)*tt+x(22)*tt.^2/2+x(23)*tt.^3/6+j*tt.^4/24;
ext=[min(th),max(th)];
end
function y=integrate(x,u,s,h,nsub)
dt=h/nsub;y=x;
mode='R1';if numel(x)==23,mode='R2';end
for k=1:nsub
 a=rhs(y);b=rhs(y+dt*a/2);c=rhs(y+dt*b/2);d=rhs(y+dt*c);
 y=y+dt*(a+2*b+2*c+d)/6;
end
 function f=rhs(xi)
  J=CBF_V11_RealTaylor(mode,xi,u,s,1);f=J(:,2);
 end
end

function err=localFinalError(log)
if isempty(log),err=NaN;else,err=norm(log(end,11:13)-log(end,14:16));end
end

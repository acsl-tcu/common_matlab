function T=CBF_V22_OneCycleAudit(s,states,obs,opts)
% ONE 25-ms zero-order-hold cycle, NOT a closed-loop controller.
% Apply SAME current physical inputs for R1/R2 while R2 Tddot is zero.
% Perturb each input component separately to quantify tiny effects at dt.
% All evaluations use the full original nonlinear physical dynamics.
if nargin<4,opts=struct();end
if ~isfield(opts,'dt'),opts.dt=.025;end
if ~isfield(opts,'substeps'),opts.substeps=4;end
if ~isfield(opts,'gamma'),opts.gamma=.06;end
if ~(opts.dt>0 && opts.dt<=.1 && opts.substeps>=2),error('V22 invalid dt');end
rows={};
for ii=1:numel(states)
 x=states{ii}(:);
 T0=(s.physical.massDrone+s.physical.massLoad)*s.physical.gravity;
 for mode={'R1','R2_TWO'}
  name=mode{1};
  if strcmp(name,'R1'),x0=x;u0=[T0;0;0;0];
  else,x0=[x;T0;0];u0=zeros(4,1);end
  h0=cableH(x0,s,obs);
  for j=0:4
   u=u0;
   if j>0
    if j==1,delta=.1;else,delta=.1;end
    u(j)=u(j)+delta;
   end
   xNext=x0;minH=h0;minTension=Inf;
   dT=opts.dt/opts.substeps;
   for step=1:opts.substeps
    minTension=min(minTension,tension(xNext,u,s,name));
    xNext=rk4(name,xNext,u,s,dT);
    minH=min(minH,cableH(xNext,s,obs));
   end
   minTension=min(minTension,tension(xNext,u,s,name));
   hEnd=cableH(xNext,s,obs);
   if strcmp(name,'R1'), thrustMin=u(1);thrustMax=u(1);
   else
    Th0=x0(20);Tdot=x0(21);a=u(1);
    samples=[0,opts.dt];
    if abs(a)>1e-12
       tstar=-Tdot/a;
       if tstar>0 && tstar<opts.dt,samples(end+1)=tstar;end %#ok<AGROW>
    end
    vals=Th0+Tdot*samples+0.5*a*samples.^2;
    thrustMin=min(vals);thrustMax=max(vals);
   end
   torqueMargin=min([u(2:4)-s.input.torqueMin(:);...
       s.input.torqueMax(:)-u(2:4)]);
   sampledPass=(minH>=0)&&(hEnd>=(1-opts.gamma)*h0) &&...
      thrustMin>=s.input.thrustMin && thrustMax<=s.input.thrustMax &&...
      minTension>0 && torqueMargin>=0;
   rows(end+1,:)={ii,name,j,opts.dt,h0,hEnd,minH, ...
       hEnd-(1-opts.gamma)*h0,thrustMin,thrustMax,minTension,torqueMargin,sampledPass, ...
       'ZOH_ONE_CYCLE_SAMPLED_ONLY_NO_OPTIMIZER_NO_INTERSAMPLE_GUARANTEE'}; %#ok<AGROW>
  end
 end
end
T=cell2table(rows,'VariableNames',{'stateId','model','perturbedInputIndex',...
 'dt','hStart','hEnd','hMinSamples','oneStepBarrierResidual',...
 'thrustMinExact','thrustMaxExact','minTensionSamples','torqueBoxMargin','sampledPass','scope'});
end
function xx=rk4(mode,x,u,s,dt)
F=@(xx)rhs(mode,xx,u,s);
a=F(x);b=F(x+.5*dt*a);c=F(x+.5*dt*b);d=F(x+dt*c);
xx=x+dt*(a+2*b+2*c+d)/6;
end
function d=rhs(mode,x,u,s)
if strcmp(mode,'R1'),Z=CBF_V11_RealTaylor('R1',x,u,s,1);
else,Z=CBF_V22_R2TwoTaylor(x,u,s,1);end
d=Z(:,2);
end
function h=cableH(x,s,obs)
L=s.physical.cableLength;rc=s.shape.cableRadius;
a=L/2+rc;b=sqrt(a*rc);
p=x(8:10);q=x(14:16);c=p-L*q/2;
d=obs.center(:)-c;eps=s.cbf.smoothEpsilon;
h=d.'*d-sqrt(b*b*(d.'*d)+(a*a-b*b)*(d.'*q)^2+eps^2)...
  -sqrt(sum((obs.axes(:).*(obs.R.'*d)).^2)+eps^2)...
  -s.cbf.safetyMargin*sqrt(d.'*d+eps^2);
end

function ten=tension(x,u,s,mode)
q=x(14:16);wc=x(17:19);quat=x(1:4);
b=[2*(quat(2)*quat(4)+quat(1)*quat(3));...
   2*(quat(3)*quat(4)-quat(1)*quat(2));...
   quat(1)^2-quat(2)^2-quat(3)^2+quat(4)^2];
if strcmp(mode,'R1'),T=u(1);else,T=x(20);end
m=s.physical.massDrone;mL=s.physical.massLoad;L=s.physical.cableLength;
qd=cross(wc,q);
scalar=(q.'*(T*b)-m*L*(qd.'*qd))/(m+mL);
ten=-mL*scalar;
end

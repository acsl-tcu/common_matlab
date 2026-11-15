function out=CBF_V15_PredictiveFilter(mode,shape,x,s,obstacles,opts)
%CBF_V15_PREDICTIVEFILTER Independent R1/R2 hard nonlinear predictive safety test.
% R1: physical x(19), decision [T; tau_xyz]; no input integrators.
% R2: physical x(19), T,Tdot,Tddot,T3 (23), decision [T4; tau_xyz].
% No tau augmentation. Safety constraints: C/B/D obstacle and ground h >= 0,
% thrust and torque bounds, positive cable tension at prediction samples.
% ZOCBF-style sampled decrease + nonpenetration constraints, not certified
% continuous-time CBF nor a fast real-time controller.
% Intersample collision / recursive feasibility are NOT certified.
if nargin<6,opts=struct();end
if ~isfield(opts,'horizon'),opts.horizon=0.35;end
if ~isfield(opts,'nSegments'),opts.nSegments=7;end
if ~isfield(opts,'substeps'),opts.substeps=2;end
if ~isfield(opts,'maxIterations'),opts.maxIterations=45;end
if ~isfield(opts,'zocbfGamma'),opts.zocbfGamma=0.9;end
if ~isscalar(opts.zocbfGamma) || opts.zocbfGamma<=0 || opts.zocbfGamma>1
    error('CBF:V15Gamma','zocbfGamma must satisfy 0<gamma<=1.');
end
mode=upper(char(mode));shape=upper(char(shape));x=x(:);
if ~any(strcmp(mode,{'R1','R2'})) || ...
  (strcmp(mode,'R1') && numel(x)~=19) || ...
  (strcmp(mode,'R2') && numel(x)~=23)
    error('CBF:V15Input','R1 needs x19, R2 needs x23.');
end
if ~any(strcmp(shape,{'C','B','D'}))
    error('CBF:V15Shape','C, B, or D required.');
end
if ~iscell(obstacles) || isempty(obstacles)
    error('CBF:V15Obstacles','Use a nonempty obstacle cell array.');
end
if ~(isscalar(opts.horizon) && opts.horizon>0 && ...
        opts.nSegments>=1 && opts.nSegments==floor(opts.nSegments) && ...
        opts.substeps>=1 && opts.substeps==floor(opts.substeps))
    error('CBF:V15Options','Invalid horizon/segment/substep options.');
end
inLB=[s.input.thrustMin;s.input.torqueMin(:)];
inUB=[s.input.thrustMax;s.input.torqueMax(:)];
Tref=(s.physical.massDrone+s.physical.massLoad)*s.physical.gravity;
if strcmp(mode,'R1')
    uNom=[Tref;0;0;0];
    lo=inLB;hi=inUB;
else
    uNom=zeros(4,1);
    % Derive exact NECESSARY snap bounds from the physical thrust bounds
    % at the end of the constant-snap horizon. No invented snap saturation.
    H=opts.horizon;
    baseT=x(20)+H*x(21)+H^2*x(22)/2+H^3*x(23)/6;
    jmin=24*(inLB(1)-baseT)/H^4;
    jmax=24*(inUB(1)-baseT)/H^4;
    lo=[jmin;inLB(2:4)];hi=[jmax;inUB(2:4)];
    snapScale=max([abs(jmin),abs(jmax),1]);
end
if strcmp(mode,'R1'),snapScale=NaN;end
% Precompute geometric model arguments; every obstacle, every point, hard.
P=cbf_parameter_vector(s);plan=cbf_choose_cable_spheres(s);
Os=cell(size(obstacles));
for i=1:numel(obstacles)
    o=obstacles{i};
    if ~isfield(o,'type'),error('CBF:V15ObstacleType','Obstacle type missing.');end
    Os{i}=cbf_obstacle_vector(o,o.type);
end
% Verify initial geometric non-penetration and physically valid taut cable.
[initGeom,initTen,~,~,~]=localPathCheck(x,mode,uNom,shape,s,obstacles,P,Os,plan,0,1,opts.zocbfGamma);
if min(initGeom)<0 || min(initTen)<=0 || ...
   (strcmp(mode,'R2') && (x(20)<inLB(1) || x(20)>inUB(1)))
    out=localResult('INVALID_INITIAL_GEOMETRY_OR_TENSION',uNom,-3,NaN, ...
        min(initGeom),min(initTen),NaN,NaN,0,0,NaN);
    return;
end
% No slack. The predicted trajectory under a constant held decision is
% constrained at all RK substeps and includes ground and obstacle rows.
[gn,tn,~,~,metaNom,zn]=localPathCheck(x,mode,uNom,shape,s,obstacles,P,Os,plan, ...
    opts.horizon,opts.nSegments*opts.substeps,opts.zocbfGamma);
[thrLo,thrHi]=localThrustExtrema(mode,x,uNom,opts.horizon);
nomResidual=min([gn;tn;zn;thrLo-inLB(1);inUB(1)-thrHi; ...
    uNom(2:4)-inLB(2:4);inUB(2:4)-uNom(2:4)]);
if strcmp(mode,'R1')
    nomResidual=min(nomResidual,min([uNom-inLB;inUB-uNom]));
end
if isfinite(nomResidual) && nomResidual>=-1e-9
    out=localResult('NOMINAL_SAMPLED_FEASIBLE',uNom,1,nomResidual, ...
        min(gn),min(tn),thrLo,thrHi,metaNom.nChecked,0,snapScale);
    return;
end
% Optimize a dimensionless j=T4/snapScale in R2; unscaled finite
% differences near j=0 cannot reliably resolve the sixth-order effect.
if strcmp(mode,'R2'),scale=[snapScale;1;1;1];
else,scale=ones(4,1);end
objective=@(v)localCost(v(:).*scale,mode,Tref,x,opts,snapScale,inLB,inUB);
constraints=@(v)localConstraints(v(:).*scale,x,mode,shape,s,obstacles,P,Os,plan,opts,inLB,inUB);
opt=optimoptions('fmincon','Algorithm','sqp','Display','off', ...
    'MaxIterations',opts.maxIterations, ...
    'MaxFunctionEvaluations',1600, ...
    'ConstraintTolerance',1e-8,'StepTolerance',1e-10);
tStart=tic;
try
    [scaledSol,~,ef]=fmincon(objective,uNom./scale,[],[],[],[],lo./scale,hi./scale,constraints,opt);
    elapsed=1000*toc(tStart);
    sol=scaledSol(:).*scale;
    if isempty(sol) || any(~isfinite(sol))
        out=localResult('SOLVER_EMPTY_OR_NONFINITE',uNom,ef,NaN, ...
            NaN,NaN,NaN,NaN,0,elapsed,snapScale);return;
    end
    [hg,te,~,~,meta,zd]=localPathCheck(x,mode,sol,shape,s,obstacles,P,Os,plan, ...
        opts.horizon,opts.nSegments*opts.substeps,opts.zocbfGamma);
    [tlo,thi]=localThrustExtrema(mode,x,sol,opts.horizon);
    % Report true constraint residual; do not declare success by exitflag.
    [c,~]=constraints(sol./scale);
    residual=-max(c);
    if ef>0 && all(isfinite(c)) && residual>=-1e-7
        status='SOLVER_SAMPLED_FEASIBLE_NOT_CERTIFIED';
    else
        status='INFEASIBLE_OR_NUMERICAL_FAILURE';
    end
    out=localResult(status,sol,ef,residual,min(hg),min(te), ...
        tlo,thi,meta.nChecked,elapsed,snapScale);
catch ME
    out=localResult(['SOLVER_ERROR_' ME.identifier],uNom,-9,NaN, ...
        NaN,NaN,NaN,NaN,0,1000*toc(tStart),snapScale);
    out.errorText=ME.message;
end

    function z=localResult(status,u,exitflag,residual,minH,minTension,Tlo,Thi,nc,ms,scale)
        z=struct('status',status,'mode',mode,'shape',shape,'decision',u(:), ...
            'exitflag',exitflag,'minConstraintResidual',residual, ...
            'minimumGeometricH',minH,'minimumTension_N',minTension, ...
            'minimumThrust_N',Tlo,'maximumThrust_N',Thi, ...
            'nSamples',nc,'solverMilliseconds',ms, ...
            'snapDecisionScaleForCost',scale,'horizon_s',opts.horizon, ...
            'zocbfGamma',opts.zocbfGamma, ...
            'errorText','', ...
            'scope','SAMPLED_TEST_ONLY_NOT_CONTINUOUSLY_CERTIFIED');
    end
end

function score=localCost(u,mode,Tref,x,opts,snapScale,inLB,inUB)
% Dimensionless physical-input tracking. No softening of safety constraints.
span=max(inUB(1)-inLB(1),eps);
tauSpan=max(inUB(2:4)-inLB(2:4),eps);
if strcmp(mode,'R1')
    z=(u(1)-Tref)/span;
    score=z^2+sum(((u(2:4))./tauSpan).^2);
else
    H=opts.horizon;
    Tend=x(20)+H*x(21)+H^2*x(22)/2+H^3*x(23)/6+H^4*u(1)/24;
    score=((Tend-Tref)/span)^2+0.01*(u(1)/snapScale)^2+ ...
        sum((u(2:4)./tauSpan).^2);
end
end

function [c,ceq]=localConstraints(u,x,mode,shape,s,obs,P,Os,plan,opts,lb,ub)
[h,tension,~,~,~,zocbf]=localPathCheck(x,mode,u,shape,s,obs,P,Os,plan, ...
     opts.horizon,opts.nSegments*opts.substeps,opts.zocbfGamma);
[Tmin,Tmax]=localThrustExtrema(mode,x,u,opts.horizon);
% c <= 0; no collision/ground/tension/input slack.
c=[-h; -tension; -zocbf; lb(1)-Tmin;Tmax-ub(1); ...
   lb(2:4)-u(2:4);u(2:4)-ub(2:4)];
ceq=[];
if any(~isfinite(c)),c(:)=1e12;end
end

function [hMin,tensionMin,thrustMin,thrustMax,meta,zocbf]=localPathCheck(x,mode,u,shape,s,obs,P,Os,plan,H,nSteps,gamma)
% Evaluates every geometric h at every RK4 substep, including t=0.
% Does NOT certify extrema between substeps.
nObs=numel(obs);hMin=zeros(0,1);tensionMin=zeros(0,1);
thrustMin=inf;thrustMax=-inf;cur=x(:);
dt=H/max(1,nSteps);meta=struct('nChecked',0);
zocbf=zeros(0,1);prevH=[];
for k=0:nSteps
    if strcmp(mode,'R1'),T=u(1);else,T=cur(20);end
    thrustMin=min(thrustMin,T);thrustMax=max(thrustMax,T);
    % Full original C/B/D geometry, both sphere/ellipsoid obstacle types.
    currentH=zeros(0,1);
    for oi=1:nObs
        [h,~]=cbf_geometry_core('real',shape,obs{oi}.type,cur(1:19),P,Os{oi},plan);
        hMin=[hMin;h(:)];currentH=[currentH;h(:)];
        meta.nChecked=meta.nChecked+numel(h);
    end
    if ~isempty(prevH)
        zocbf=[zocbf;currentH-(1-gamma)*prevH];
    end
    prevH=currentH;
    q=cur(1:4);qc=cur(14:16);wc=cur(17:19);
    bz=[2*(q(2)*q(4)+q(1)*q(3)); ...
        2*(q(3)*q(4)-q(1)*q(2)); ...
        q(1)^2-q(2)^2-q(3)^2+q(4)^2];
    qdot=cross(wc,qc);
    scalar=(dot(qc,T*bz)-P(1)*P(3)*dot(qdot,qdot))/(P(1)+P(2));
    tensionMin=[tensionMin;-P(2)*scalar];
    if k==nSteps,break;end
    try
        k1=localRHS(mode,cur,u,s);
        k2=localRHS(mode,cur+0.5*dt*k1,u,s);
        k3=localRHS(mode,cur+0.5*dt*k2,u,s);
        k4=localRHS(mode,cur+dt*k3,u,s);
        cur=cur+(dt/6)*(k1+2*k2+2*k3+k4);
    catch
        % Keep nonlinear constraint vector length fixed for fmincon.
        [hMin,tensionMin,zocbf]=localPadBad(hMin,tensionMin,zocbf,...
            numel(currentH),nSteps);break;
    end
    if any(~isfinite(cur))
        [hMin,tensionMin,zocbf]=localPadBad(hMin,tensionMin,zocbf,...
            numel(currentH),nSteps);break;
    end
end
end


function [h,t,z]=localPadBad(h,t,z,nRows,nSteps)
% Enforce fixed c(u) dimension even when a trial integration fails.
nH=nRows*(nSteps+1)-numel(h);
nT=(nSteps+1)-numel(t);
nZ=nRows*nSteps-numel(z);
if nH>0,h=[h;-1e12*ones(nH,1)];end
if nT>0,t=[t;-1e12*ones(nT,1)];end
if nZ>0,z=[z;-1e12*ones(nZ,1)];end
end

function dx=localRHS(mode,x,u,s)
J=CBF_V11_RealTaylor(mode,x,u,s,1);
dx=J(:,2);
end

function [lo,hi]=localThrustExtrema(mode,x,u,H)
if strcmp(mode,'R1')
    lo=u(1);hi=u(1);return;
end
% T(t)=T0+T1*t+T2*t^2/2+T3*t^3/6+j*t^4/24.
% Test all continuous-time extrema of thrust over held j input.
coef=[u(1)/6, x(23)/2, x(22), x(21)];
ts=[0;H];
if any(abs(coef)>1e-14)
    rt=roots(coef);
    good=abs(imag(rt))<1e-9 & real(rt)>=0 & real(rt)<=H;
    ts=[ts;real(rt(good))];
end
v=x(20)+x(21)*ts+x(22)*ts.^2/2+x(23)*ts.^3/6+u(1)*ts.^4/24;
lo=min(v);hi=max(v);
end

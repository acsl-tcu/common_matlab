function out=CBF_V16_ZOCBFStep(mode,shape,x,s,obstacle,opts)
%CBF_V16_ZOCBFSTEP One 25-ms sample, R1 or thrust-only R2. Test-only.
% Implements sampled ZOCBF DIFFERENCE h(x_next)-h(x_now)>=-gamma*h(x_now)+delta.
% Unlike V15 it uses initial-to-next interval, NOT repeated pseudo-gamma
% at numerical integration substeps. Still NOT the paper's certified ZOCBF:
% exact flow, a proven intersample delta>=h_x*M*dt and recursive feasibility
% are unavailable. RK4 substeps and nonpenetration are only checks.
% No slack, no torque augmentation, no output clipping, no obstacle culling.
if nargin<6,opts=struct();end
if ~isfield(opts,'dt'),opts.dt=0.025;end
if ~isfield(opts,'substeps'),opts.substeps=4;end
if ~isfield(opts,'gamma'),opts.gamma=0.1;end
if ~isfield(opts,'deltaGround'),opts.deltaGround=0;end
if ~isfield(opts,'deltaObstacle'),opts.deltaObstacle=0;end
if ~isfield(opts,'maxIterations'),opts.maxIterations=55;end
mode=upper(char(mode));shape=upper(char(shape));x=x(:);
assert(any(strcmp(mode,{'R1','R2'})) && any(strcmp(shape,{'C','B','D'})));
assert((strcmp(mode,'R1')&&numel(x)==19)||(strcmp(mode,'R2')&&numel(x)==23));
assert(opts.dt>0&&opts.substeps>=2&&opts.gamma>0&&opts.gamma<=1);
assert(opts.deltaGround>=0&&opts.deltaObstacle>=0);
P=cbf_parameter_vector(s);O=cbf_obstacle_vector(obstacle,obstacle.type);
plan=cbf_choose_cable_spheres(s);
[h0,labels]=localGeom(x);n=numel(h0);
labels=labels(:);isGround=strncmp(labels,'ground_',7);
delta=ones(n,1)*opts.deltaObstacle;delta(isGround)=opts.deltaGround;
Tref=(s.physical.massDrone+s.physical.massLoad)*s.physical.gravity;
lo=[s.input.thrustMin;s.input.torqueMin(:)];hi=[s.input.thrustMax;s.input.torqueMax(:)];
if strcmp(mode,'R1')
    uNom=[Tref;0;0;0];lb=lo;ub=hi;scale=ones(4,1);
else
    uNom=zeros(4,1);H=opts.dt;
    base=x(20)+H*x(21)+H^2*x(22)/2+H^3*x(23)/6;
    jlo=24*(lo(1)-base)/H^4;jhi=24*(hi(1)-base)/H^4;
    lb=[jlo;lo(2:4)];ub=[jhi;hi(2:4)];
    scale=[max(abs([jlo,jhi,1]));1;1;1];
end
if any(h0<0)
    out=localResult('INVALID_INITIAL_GEOMETRY',uNom,-3,NaN,NaN,NaN,NaN,0);return;
end
[rNom,metaNom]=localResidual(uNom);
if all(isfinite(rNom))&&min(rNom)>=-1e-9
    out=localResult('NOMINAL_SAMPLED_PASS_NOT_CERTIFIED',uNom,1,min(rNom),metaNom.minH,metaNom.minTension,metaNom.minThrust,0);return;
end
try
    options=optimoptions('fmincon','Algorithm','sqp','Display','off',...
        'MaxIterations',opts.maxIterations,'MaxFunctionEvaluations',1200,...
        'ConstraintTolerance',1e-8);
    obj=@(v)sum((v(:)-uNom./scale).^2);
    cons=@(v)localCons(v(:).*scale);
    tic;
    [v,~,ef]=fmincon(obj,uNom./scale,[],[],[],[],lb./scale,ub./scale,cons,options);
    ms=1000*toc;
    u=v(:).*scale;
    [r,m]=localResidual(u);
    if ef>0 && all(isfinite(r))&&min(r)>=-1e-7
        status='SOLVER_SAMPLED_PASS_NOT_CERTIFIED';
    else
        status='SOLVER_FAILED_OR_CONSTRAINT_VIOLATION';
    end
    out=localResult(status,u,ef,min(r),m.minH,m.minTension,m.minThrust,ms);
catch ME
    out=localResult('SOLVER_MATLAB_ERROR',uNom,-9,NaN,NaN,NaN,NaN,NaN);
    out.errorText=[ME.identifier ': ' ME.message];
end
    function [h,lab]=localGeom(state)
        [h,info]=cbf_geometry_core('real',shape,obstacle.type,state(1:19),P,O,plan);
        lab=info.labels;h=h(:);
    end
    function ten=localTension(state,T)
        q=state(1:4);qc=state(14:16);wc=state(17:19);
        b=[2*(q(2)*q(4)+q(1)*q(3));...
           2*(q(3)*q(4)-q(1)*q(2));...
           q(1)^2-q(2)^2-q(3)^2+q(4)^2];
        dqc=cross(wc,qc);
        scalar=(dot(qc,T*b)-P(1)*P(3)*dot(dqc,dqc))/(P(1)+P(2));
        ten=-P(2)*scalar;
    end
    function [r,met]=localResidual(u)
        current=x;dt=opts.dt/opts.substeps;
        hMin=inf(n,1);tMin=Inf;
        for ii=0:opts.substeps
            [hNow,~]=localGeom(current);hMin=min(hMin,hNow);
            if strcmp(mode,'R1'),Tnow=u(1);else,Tnow=current(20);end
            tMin=min(tMin,localTension(current,Tnow));
            if ii==opts.substeps,break;end
            k1=CBF_V11_RealTaylor(mode,current,u,s,1);f1=k1(:,2);
            k2=CBF_V11_RealTaylor(mode,current+dt*f1/2,u,s,1);f2=k2(:,2);
            k3=CBF_V11_RealTaylor(mode,current+dt*f2/2,u,s,1);f3=k3(:,2);
            k4=CBF_V11_RealTaylor(mode,current+dt*f3,u,s,1);f4=k4(:,2);
            current=current+dt/6*(f1+2*f2+2*f3+f4);
        end
        if strcmp(mode,'R1'),Tl=u(1);Th=u(1);
        else
            rootsCoeff=[u(1)/6,x(23)/2,x(22),x(21)];times=[0;opts.dt];
            if any(abs(rootsCoeff)>1e-14)
                rr=roots(rootsCoeff);keep=abs(imag(rr))<1e-9 &...
                    real(rr)>=0 & real(rr)<=opts.dt;
                times=[times;real(rr(keep))];
            end
            tt=x(20)+x(21)*times+x(22)/2*times.^2+...
               x(23)/6*times.^3+u(1)/24*times.^4;
            Tl=min(tt);Th=max(tt);
        end
        r=[hMin;hNow-(1-opts.gamma)*h0-delta;tMin;...
            Tl-lo(1);hi(1)-Th;u(2:4)-lo(2:4);hi(2:4)-u(2:4)];
        met=struct('minH',min(hMin),'minTension',tMin,...
            'minThrust',Tl,'maxThrust',Th);
        if any(~isfinite(r)),r= -1e12*ones(size(r));end
    end
    function [c,ceq]=localCons(u)
        [r,~]=localResidual(u);c=-r;ceq=[];
    end
    function o=localResult(status,u,ef,res,hmin,ten,tmin,ms)
        o=struct('status',status,'mode',mode,'shape',shape,...
          'decision',u(:),'solverExitflag',ef,...
          'hardSampleResidual',res,'minGeometricH',hmin,...
          'minTension',ten,'minThrust',tmin,'timeMilliseconds',ms,...
          'gamma',opts.gamma,'deltaGround',opts.deltaGround,...
          'deltaObstacle',opts.deltaObstacle,'errorText','',...
          'scope','SAMPLED_RK4_ONLY_NO_PROVEN_INTERSAMPLE_DELTA_OR_FEASIBILITY');
    end
end

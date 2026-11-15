function out=CBF_V18_R2_ZOCable(x23,s,obs,opts)
% R2-3 ONLY: one-step sampled-data obstacle-only ZOCBF using actual 23D RHS.
% NO ground, drone/load spheres. NO slack, skipping, silent clipping.
% Physical input bounds and positive tension are kept as model-validity constraints.
% RK4 and delta=0 do NOT prove intersample safety nor recursive feasibility.
if nargin<4,opts=struct();end
if ~isfield(opts,'dt'),opts.dt=.025;end
if ~isfield(opts,'substeps'),opts.substeps=4;end
if ~isfield(opts,'gamma'),opts.gamma=.1;end
if ~isfield(opts,'delta'),opts.delta=0;end
if ~isfield(opts,'maxIterations'),opts.maxIterations=60;end
assert(numel(x23)==23 && opts.dt>0 && opts.substeps>=2);
assert(opts.gamma>0&&opts.gamma<=1&&opts.delta>=0);
lo=[s.input.thrustMin;s.input.torqueMin(:)];hi=[s.input.thrustMax;s.input.torqueMax(:)];
x23=x23(:);uNom=zeros(4,1);H=opts.dt;
base=x23(20)+H*x23(21)+H^2*x23(22)/2+H^3*x23(23)/6;
jlb=24*(lo(1)-base)/H^4;jub=24*(hi(1)-base)/H^4;
lb=[jlb;lo(2:4)];ub=[jub;hi(2:4)];
scale=[max(1,max(abs([jlb,jub])));1;1;1];
h0=geometry(x23);
if h0<0
  out=result('INVALID_INITIAL_GEOMETRY',uNom,-3,NaN,NaN,NaN,0);return;
end
[resNom,mNom]=residual(uNom);
if all(isfinite(resNom))&&min(resNom)>=-1e-8
  out=result('NOMINAL_SAMPLED_PASS_NOT_CERTIFIED',uNom,1,min(resNom),mNom.hmin,mNom.tension,0);return;
end
try
 opt=optimoptions('fmincon','Algorithm','sqp','Display','off',...
     'MaxIterations',opts.maxIterations,'MaxFunctionEvaluations',1500,...
     'ConstraintTolerance',1e-8);
 tic;[v,~,ef]=fmincon(@(v)sum((v(:)-uNom./scale).^2),uNom./scale,...
       [],[],[],[],lb./scale,ub./scale,@constraints,opt);ms=toc*1000;
 u=v(:).*scale;[res,m]=residual(u);
 if ef>0&&all(isfinite(res))&&min(res)>=-1e-7
    status='SOLVER_SAMPLED_PASS_NOT_CERTIFIED';
 else,status='SOLVER_FAILED_OR_HARD_CONSTRAINT_VIOLATION';end
 out=result(status,u,ef,min(res),m.hmin,m.tension,ms);
catch ME
 out=result('SOLVER_MATLAB_ERROR',uNom,-9,NaN,NaN,NaN,NaN);
 out.errorText=[ME.identifier ': ' ME.message];
end
    function h=geometry(x)
        J=CBF_V18_CableOnly('R2',x,zeros(4,1),s,obs,1);
        h=J(1);
    end
    function [r,m]=residual(u)
       x=x23;hh=opts.dt/opts.substeps;hmin=Inf;tmin=Inf;
       for k=0:opts.substeps
          hcur=geometry(x);hmin=min(hmin,hcur);
          q=x(1:4);qc=x(14:16);wc=x(17:19);
          b=[2*(q(2)*q(4)+q(1)*q(3));...
             2*(q(3)*q(4)-q(1)*q(2));...
             q(1)^2-q(2)^2-q(3)^2+q(4)^2];
          qdot=cross(wc,qc);
          scalar=(qc.'*(x(20)*b)-s.physical.massDrone*s.physical.cableLength*(qdot.'*qdot))/...
                 (s.physical.massDrone+s.physical.massLoad);
          tmin=min(tmin,-s.physical.massLoad*scalar);
          if k==opts.substeps,break;end
          z1=CBF_V11_RealTaylor('R2',x,u,s,1);f1=z1(:,2);
          z2=CBF_V11_RealTaylor('R2',x+hh*f1/2,u,s,1);f2=z2(:,2);
          z3=CBF_V11_RealTaylor('R2',x+hh*f2/2,u,s,1);f3=z3(:,2);
          z4=CBF_V11_RealTaylor('R2',x+hh*f3,u,s,1);f4=z4(:,2);
          x=x+hh*(f1+2*f2+2*f3+f4)/6;
       end
       % Continuous extrema of T(t), not just endpoints.
       coeff=[u(1)/6,x23(23)/2,x23(22),x23(21)];
       ts=[0;opts.dt];
       if any(abs(coeff)>1e-14)
          rr=roots(coeff);good=abs(imag(rr))<1e-9 & real(rr)>=0 & real(rr)<=opts.dt;
          ts=[ts;real(rr(good))];
       end
       TT=x23(20)+x23(21)*ts+x23(22)/2*ts.^2+...
          x23(23)/6*ts.^3+u(1)/24*ts.^4;
       tlo=min(TT);thi=max(TT);
       r=[hmin;hcur-(1-opts.gamma)*h0-opts.delta;tmin;...
          tlo-lo(1);hi(1)-thi;u(2:4)-lo(2:4);hi(2:4)-u(2:4)];
       m=struct('hmin',hmin,'tension',tmin,'minT',tlo,'maxT',thi);
       if any(~isfinite(r)),r=-1e12*ones(size(r));end
    end
    function [c,ceq]=constraints(v)
       [r,~]=residual(v(:).*scale);c=-r;ceq=[];
    end
    function o=result(status,u,ef,res,hmin,ten,ms)
       o=struct('status',status,'input',u,'exitflag',ef,'hardResidual',res,...
         'h0',h0,'minSampleH',hmin,'minSampleTension',ten,...
         'milliseconds',ms,'gamma',opts.gamma,'delta',opts.delta,'errorText','',...
         'scope','FULL3D_ONE_ELLIPSOID_RK4_SAMPLED_ONLY_NO_CERTIFIED_INTERSAMPLE_DELTA');
    end
end

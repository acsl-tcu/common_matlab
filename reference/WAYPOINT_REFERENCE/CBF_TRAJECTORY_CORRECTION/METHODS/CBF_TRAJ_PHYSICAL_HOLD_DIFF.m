function [deltaPV,diag]=CBF_TRAJ_PHYSICAL_HOLD_DIFF(controllerResult,P,dt,cbfId,method,nSub)
% Physical, SAME-INITIAL-STATE, ONE-INTERVAL correction caused by CBF.
% - ID3: quadratic thrust T(s)=T0+q0*s+nu*s^2/2 + ZOH torques.
% - ID0/1/2: four physical input components are ZOH.
% - Baseline: bounded nominal action, NOT unbounded raw HLC thrust.
% - `diag.valid=false` if a comparable baseline cannot be constructed.
% No unknown future controller inputs are assumed; no safety proof claimed.
% Two integrator choices for comparison: RK4 (reference) and MIDPOINT (fast).
% P=[droneMass,Jx,Jy,Jz,g,loadMass,cableLength], x has 19 states.
deltaPV=zeros(6,1);
diag=struct('valid',false,'active',false,'status','NOT_EVALUATED', ...
    'deltaControl',nan(4,1),'baseline',nan(4,1),'applied',nan(4,1), ...
    'T0',NaN,'Tdot0',NaN,'TddotApplied',NaN,'TddotBaseline',NaN, ...
    'TendApplied',NaN,'TendBaseline',NaN, ...
    'TdotEndApplied',NaN,'TdotEndBaseline',NaN,'elapsedMs',NaN);
start=tic;
validateattributes(P,{'numeric'},{'vector','numel',7,'finite','real'});
validateattributes(dt,{'numeric'},{'scalar','finite','positive'});
validateattributes(nSub,{'numeric'},{'scalar','integer','>=',1,'<=',16});
if ~isstruct(controllerResult) || ~isfield(controllerResult,'x') || ...
        ~isfield(controllerResult,'tmp') || ~isfield(controllerResult,'input')
    diag.status='CONTROLLER_SNAPSHOT_MISSING';return
end
x=double(controllerResult.x(:));raw=double(controllerResult.tmp(:));
u=double(controllerResult.input(:));P=double(P(:));
if numel(x)~=19 || numel(raw)~=4 || numel(u)~=4 || ...
        any(~isfinite([x;raw;u])) || any(P([1:5,6,7])<=0)
    diag.status='INVALID_PHYSICAL_SNAPSHOT';return
end
if abs(norm(x(1:4))-1)>0.01 || abs(norm(x(14:16))-1)>0.01
    diag.status='STATE_OUTSIDE_UNIT_MANIFOLD';return
end
if any(u<[0;-1;-1;-1]-1e-8) || any(u>[20;1;1;1]+1e-8)
    diag.status='APPLIED_INPUT_OUTSIDE_BOX';return
end
uBase=min(max(raw,[0;-1;-1;-1]),[20;1;1;1]);
if cbfId==3
    if ~isfield(controllerResult,'cbf') || ~isstruct(controllerResult.cbf)
        diag.status='ID3_INFO_MISSING';return
    end
    c=controllerResult.cbf;
    if isfield(c,'paperActuatorFallbackRateReset') && ...
            ~isempty(c.paperActuatorFallbackRateReset) && ...
            logical(c.paperActuatorFallbackRateReset)
        diag.status='ID3_FALLBACK_RATE_RESET_NOT_COMPARABLE';return
    end
    if ~isfield(c,'paperHold') || ~isstruct(c.paperHold)
        diag.status='ID3_HOLD_MISSING';return
    end
    h=c.paperHold;
    req={'active','T0','q0','nu','Tend','qEnd','dt'};
    if ~all(isfield(h,req)) || ~isscalar(h.active) || ~logical(h.active)
        diag.status='ID3_HOLD_INVALID';return
    end
    if ~isfield(c,'paperNominalNuRaw') || ...
            ~isfield(c,'paperAdmissibleNuAfterViability') || ...
            numel(c.paperAdmissibleNuAfterViability)~=2
        diag.status='ID3_BASELINE_ENVELOPE_UNKNOWN';return
    end
    vals=[h.T0;h.q0;h.nu;h.Tend;h.qEnd;h.dt; ...
          c.paperNominalNuRaw;c.paperAdmissibleNuAfterViability(:)];
    if any(~isfinite(vals)) || abs(h.dt-dt)>1e-10 || ...
            abs(u(1)-h.T0)>1e-8
        diag.status='ID3_HOLD_INCONSISTENT';return
    end
    bounds=double(c.paperAdmissibleNuAfterViability(:));
    if bounds(1)>bounds(2)
        diag.status='ID3_EMPTY_BASELINE_ENVELOPE';return
    end
    nuBase=min(max(double(c.paperNominalNuRaw),bounds(1)),bounds(2));
    applied=[h.T0;u(2:4)];
    uBase=[h.T0;uBase(2:4)];
    diag.T0=h.T0;diag.Tdot0=h.q0;diag.TddotApplied=h.nu;
    diag.TddotBaseline=nuBase;
    diag.TendApplied=h.Tend;
    diag.TendBaseline=h.T0+dt*h.q0+0.5*dt^2*nuBase;
    diag.TdotEndApplied=h.qEnd;
    diag.TdotEndBaseline=h.q0+dt*nuBase;
    diffPhysical=[diag.TendApplied-diag.TendBaseline;u(2:4)-uBase(2:4)];
    policyA=[h.T0,h.q0,h.nu];policyB=[h.T0,h.q0,nuBase];
else
    applied=u;uBase=uBase;
    diffPhysical=applied-uBase;
    policyA=[u(1),0,0];policyB=[uBase(1),0,0];
end
diag.baseline=uBase;diag.applied=applied;diag.deltaControl=diffPhysical;
diag.active=any(abs(diffPhysical)>1e-8);
if ~diag.active
    diag.valid=true;diag.status='EQUIVALENT_BOUNDED_ACTUATION';
    diag.elapsedMs=1000*toc(start);return
end
switch upper(char(method))
    case {'RK4','MIDPOINT'}
    otherwise
        error('CBFTraj:BadIntegrator','Unknown physical hold method %s',char(method));
end
xA=propagate(x,applied,policyA,P,dt,nSub,method);
xB=propagate(x,uBase,policyB,P,dt,nSub,method);
deltaPV=[xA(8:10)-xB(8:10);xA(11:13)-xB(11:13)];
if any(~isfinite(deltaPV))
    diag.status='NONFINITE_PHYSICAL_DIFFERENCE';return
end
diag.valid=true;diag.status='OK';diag.elapsedMs=1000*toc(start);
end

function y=propagate(x,u,thrust,P,dt,nSub,method)
h=dt/nSub;y=x;
for k=1:nSub
    t=(k-1)*h;
    if strcmpi(char(method),'RK4')
        k1=rhs(y,at(u,thrust,t),P);
        k2=rhs(y+h*k1/2,at(u,thrust,t+h/2),P);
        k3=rhs(y+h*k2/2,at(u,thrust,t+h/2),P);
        k4=rhs(y+h*k3,at(u,thrust,t+h),P);
        y=y+h*(k1+2*k2+2*k3+k4)/6;
    else
        k1=rhs(y,at(u,thrust,t),P);
        k2=rhs(y+h*k1/2,at(u,thrust,t+h/2),P);
        y=y+h*k2;
    end
end
end

function uk=at(u,profile,s)
uk=u;
uk(1)=profile(1)+profile(2)*s+0.5*profile(3)*s^2;
end

function out=rhs(x,u,p)
% Same 19-state suspended-load ideal dynamics used in V81 research.
% x=[quaternion(4); bodyOmega(3); loadP(3); loadV(3);
%    cableDirection(3); cableOmega(3)].
m=p(1);J=p(2:4);g=p(5);ml=p(6);L=p(7);
q=x(1:4);omega=x(5:7);dir=x(14:16);omegaL=x(17:19);
b3=[2*(q(2)*q(4)+q(1)*q(3)); ...
    2*(q(3)*q(4)-q(1)*q(2)); ...
    q(1)^2-q(2)^2-q(3)^2+q(4)^2];
qd=0.5*[-q(2:4)'*omega; q(1)*omega+cross(q(2:4),omega)];
wd=(u(2:4)-cross(omega,J.*omega))./J;
crossLD=cross(omegaL,dir);
radial=(u(1)*dot(dir,b3)-m*L*dot(crossLD,crossLD))/(m+ml);
out=[qd;wd;x(11:13);-g*[0;0;1]+radial*dir; ...
    crossLD;-(u(1)/(m*L))*cross(dir,b3)];
end

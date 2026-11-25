function [deltaPV,diag,nextComparison]=CBF_TRAJ_PHYSICAL_HOLD_DIFF(controllerResult,P,dt,cbfId,hlc,comparison,trackComparison,allowComparison)
% Single method: matched-reference closed-loop causal comparison.
% Actual and comparison use the recorded same reference; comparison HLC
% is evaluated purely on its own state (no do/CBF/live state mutation).
% Only the previous completed interval is evaluated. No future plant input.
nSub=2;
deltaPV=zeros(6,1);nextComparison=[];
diag=CBF_TRAJ_PHYSICAL_DIAG('NOT_EVALUATED');
start=tic;
if ~isnumeric(P) || ~isreal(P) || ~isvector(P) || numel(P)~=7 || any(~isfinite(P(:)))
    diag.status='INVALID_PREVIOUS_PARAMETERS';return
end
validateattributes(dt,{'numeric'},{'scalar','finite','positive'});
validateattributes(nSub,{'numeric'},{'scalar','integer','>=',1,'<=',16});
if ~isstruct(controllerResult) || ~isfield(controllerResult,'x') || ...
        ~isfield(controllerResult,'tmp') || ~isfield(controllerResult,'input')
    diag.status='CONTROLLER_SNAPSHOT_MISSING';return
end
x=double(controllerResult.x(:));raw=double(controllerResult.tmp(:));
u=double(controllerResult.input(:));P=double(P(:));
if numel(x)~=19 || numel(raw)~=4 || numel(u)~=4 || ...
        ~isreal([x;raw;u]) || any(~isfinite([x;raw;u])) || any(P<=0)
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
            abs(u(1)-h.T0)>1e-8 || ...
            abs(h.Tend-(h.T0+dt*h.q0+0.5*dt^2*h.nu))>1e-8 || ...
            abs(h.qEnd-(h.q0+dt*h.nu))>1e-8
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
    applied=u;
    diffPhysical=applied-uBase;
    policyA=[u(1),0,0];policyB=[uBase(1),0,0];
end
diag.baseline=uBase;diag.applied=applied;diag.deltaControl=diffPhysical;
diag.active=any(abs(diffPhysical)>1e-8);
[diag.returnGate,diag.returnGateReason]=CBF_TRAJ_RETURN_GATE(controllerResult,diag,cbfId);
if ~diag.active && ~trackComparison
    diag.valid=true;diag.status='EQUIVALENT_BOUNDED_ACTUATION';
    diag.elapsedMs=1000*toc(start);return
end
try
    xA=project_state(propagate(x,applied,policyA,P,dt,nSub));
    if all(diffPhysical==0),xB=xA;else,xB=project_state(propagate(x,uBase,policyB,P,dt,nSub));end
    deltaPV=xA(8:13)-xB(8:13);
    if any(~isfinite(deltaPV)),error('CBFTraj:NonfiniteLocal');end
catch ME
    diag.status=['LOCAL_INVALID:' ME.identifier];diag.returnGate=false;
    diag.elapsedMs=1000*toc(start);return
end
diag.valid=true;diag.status='OK';
if ~diag.active,diag.status='EQUIVALENT_BOUNDED_ACTUATION';end
if ~allowComparison
    diag.comparisonStatus='LATCHED_OFF';diag.elapsedMs=1000*toc(start);return
end
try
    if ~isfield(controllerResult,'xd') || numel(controllerResult.xd)~=28
        error('CBFTraj:ComparisonReferenceMissing');
    end
    xd=double(controllerResult.xd(:));
    if any(~isfinite(xd)),error('CBFTraj:ComparisonReferenceInvalid');end
    if isempty(comparison)
        % Verify the generated pure HLC matches the recorded live request.
        check=nominal_hlc(hlc,x,xd,P,false);
        diag.replayRawError=max(abs(check-raw));
        if any(abs(check-raw)>1e-7*(1+abs(raw)))
            error('CBFTraj:HLCReplayMismatch','Recorded raw input does not match pure HLC');
        end
        xb=x;tb=policyA(1:2).';rawB=raw;
        % At the common starting state use the exact recorded envelope.
        yb=xB;tbEnd=[diag.TendBaseline;diag.TdotEndBaseline];
        if cbfId~=3,tbEnd=[uBase(1);0];end
        diag.comparisonStarted=true;
    else
        xb=project_state(rebase_state(x,comparison.actual,comparison.nominal));
        tb=comparison.thrust;
        rawB=nominal_hlc(hlc,xb,xd,P,true);
        diag.comparisonRaw=rawB;
        if any(rawB<[0;-1;-1;-1]-1e-8 | rawB>[20;1;1;1]+1e-8)
            error('CBFTraj:ComparisonRawOutsideBox');
        end
        [yb,tbEnd]=baseline_step(xb,tb,rawB,P,dt,nSub,cbfId,hlc.cbfConfig);
    end
    % Reject a comparison outside its configured diagnostic domain;
    % never clip its output and never retain a divergent velocity.
    diag.comparisonRaw=rawB;
    if any(~isfinite(rawB)) || any(rawB<[0;-1;-1;-1]-1e-8 | rawB>[20;1;1;1]+1e-8)
        error('CBFTraj:ComparisonRawOutsideBox');
    end
    [~,ra0]=rhs(x,at(applied,policyA,0),P);
    [~,ra1]=rhs(xA,at(applied,policyA,dt),P);
    [~,rb0]=rhs(x,at(uBase,policyB,0),P);
    [~,rb1]=rhs(xB,at(uBase,policyB,dt),P);
    if any(~isfinite([ra0,ra1,rb0,rb1])) || max([ra0,ra1,rb0,rb1])>=0
        error('CBFTraj:ComparisonCableSlack');
    end
    pv=xA(8:13)-yb(8:13);
    diag.modelActual=xA;diag.modelNominal=yb;diag.modelOffsetPV=pv;
    diag.comparisonThrustEnd=tbEnd;
    if any(~isfinite([pv;tbEnd])) || norm(pv(1:3),Inf)>5 || norm(pv(4:6),Inf)>5
        error('CBFTraj:ComparisonDomain');
    end
    nextComparison=struct('actual',xA,'nominal',yb,'thrust',tbEnd);
    diag.comparisonValid=true;diag.comparisonStatus='OK';
catch ME
    diag.comparisonStatus=ME.identifier;nextComparison=[];
end
diag.elapsedMs=1000*toc(start);
end

function raw=nominal_hlc(hlc,x,xd,P,adjustYaw)
% Verbatim generated pure-function sequence of HLC_SUSPENDED_LOAD_CBF_REAL.do.
% The methods take all numerical inputs explicitly; no live controller do().
if adjustYaw
    q=x(1:4);yaw=atan2(2*(q(1)*q(4)+q(2)*q(3)),1-2*(q(3)^2+q(4)^2));
    xd(4)=yaw-atan2(sin(yaw-xd(4)),cos(yaw-xd(4)));
end
P=[P(:).',0,0];a=hlc.param;
vf=hlc.Vfd_SuspendedLoadxyDst(a.dt,x,xd.',a.F1);
vs=hlc.Vs_SuspendedLoadxyDst(x,xd.',vf,P,a.F2,a.F3,a.F4);
uf=hlc.Uf_SuspendedLoadxyDst(x,xd.',vf,P);
beta=hlc.Beta2_SuspendedLoadxyDst(x,xd.',vf,P);
b=hlc.V2_alpha2_SuspendedLoadxyDst(x,xd.',vf,vs.',P);
if any(~isfinite(beta(:))) || rcond(beta)<1e-12,error('CBFTraj:ComparisonHLCIllConditioned');end
raw=[uf(1);beta\b];
if ~isreal(raw) || any(~isfinite(raw)),error('CBFTraj:ComparisonHLCNonfinite');end
end

function y=propagate(x,u,thrust,P,dt,nSub)
h=dt/nSub;y=x;
for k=1:nSub
    t=(k-1)*h;
    k1=rhs(y,at(u,thrust,t),P);
    k2=rhs(y+h*k1/2,at(u,thrust,t+h/2),P);
    k3=rhs(y+h*k2/2,at(u,thrust,t+h/2),P);
    k4=rhs(y+h*k3,at(u,thrust,t+h),P);
    y=y+h*(k1+2*k2+2*k3+k4)/6;
end
end

function uk=at(u,profile,s)
uk=u;
uk(1)=profile(1)+profile(2)*s+0.5*profile(3)*s^2;
end

function [out,radial]=rhs(x,u,p)
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


function [y,tEnd]=baseline_step(x,t,raw,P,dt,nSub,cbfId,ac)
u=min(max(raw,[0;-1;-1;-1]),[20;1;1;1]);
if cbfId==3
    % Exact same actuator-only feasibility routine as ID3 itself, but on
    % the independent comparison actuator state. No live config mutation.
    ac.paperThrustState=struct('initialized',true,'T',t(1),'q',t(2));
    [u,fb]=CBF_REAL_SCHEME_03_PAPER_EXTENSION('fallback',ac,raw,dt);
    if fb.rateReset || abs(u(1)-t(1))>1e-8
        error('CBFTraj:ComparisonRateReset','Comparison actuator reset rejected');
    end
    h=fb.paperHold;policy=[h.T0,h.q0,h.nu];tEnd=[h.Tend;h.qEnd];
else
    policy=[u(1),0,0];tEnd=[u(1);0];
end
% Taut-cable model validity check at integration endpoints (not a proof
% over the whole interval). pT points from the drone towards the load.
[~,r0]=rhs(x,at(u,policy,0),P);
y=project_state(propagate(x,u,policy,P,dt,nSub));
[~,r1]=rhs(y,at(u,policy,dt),P);
if ~all(isfinite([r0,r1])) || max([r0,r1])>=0
    error('CBFTraj:ComparisonCableSlack','Nonpositive model cable tension');
end
end

function y=project_state(y)
if any(~isfinite(y)) || abs(norm(y(1:4))-1)>0.01 || ...
        abs(norm(y(14:16))-1)>0.01
    error('CBFTraj:ComparisonManifold','Invalid integrated manifold state');
end
y(1:4)=y(1:4)/norm(y(1:4));y(14:16)=y(14:16)/norm(y(14:16));
end
function b=align_quat(a,b)
if dot(a(1:4),b(1:4))<0,b(1:4)=-b(1:4);end
end
function b=rebase_state(x,a,bOld)
a=project_state(a);bOld=project_state(align_quat(a,bOld));
b=x+(bOld-a);
% Preserve relative rotation in SO(3) and shortest rotation on S^2.
qRel=qmul(bOld(1:4),[a(1);-a(2:4)]);
b(1:4)=qmul(qRel,x(1:4)/norm(x(1:4)));
v=a(14:16);w=bOld(14:16);c=dot(v,w);
if c<=-1+1e-8,error('CBFTraj:ComparisonAntipodal','Cable rotation ambiguous');end
k=cross(v,w);z=x(14:16)/norm(x(14:16));
b(14:16)=z+cross(k,z)+cross(k,cross(k,z))/(1+c);
end
function q=qmul(a,b)
q=[a(1)*b(1)-dot(a(2:4),b(2:4)); ...
    a(1)*b(2:4)+b(1)*a(2:4)+cross(a(2:4),b(2:4))];
end

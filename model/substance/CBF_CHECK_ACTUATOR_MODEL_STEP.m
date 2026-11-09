function R = CBF_CHECK_ACTUATOR_MODEL_STEP(snapshotFile)
% CBF_CHECK_ACTUATOR_MODEL_STEP
% Independent, non-controlling audit of a saved active physical-CBF step.
% Compares the ONE-STEP state prediction under
%   A: quadratic thrust interpolation (as used by the CBF dynamic extension)
%   B: piecewise-constant thrust (possible simulator ZOH behavior).
% Then evaluates the same fourth-order UAV S3 chain at the predicted states.
% Does NOT change HLC, plant, references, supervisor, or QP.
%
% Usage:
%   R=CBF_CHECK_ACTUATOR_MODEL_STEP('CBF_physical_QP_failure.mat');
%
% This comparison is NOT proof that the simulator implements ZOH: inspect
% Model_Suspended_Load and input_transform. Estimated x may contain error.
if nargin<1 || isempty(snapshotFile)
    snapshotFile='CBF_physical_QP_failure.mat';
end
if ~isfile(snapshotFile), error('Snapshot not found: %s',snapshotFile); end
S=load(snapshotFile);
if ~isfield(S,'D'), error('Expected snapshot variable D.'); end
D=S.D;
if ~isfield(D,'supervisor') || ~isfield(D.supervisor,'lastActiveTrace') || isempty(D.supervisor.lastActiveTrace)
    error('D.supervisor.lastActiveTrace is missing. Use a failure snapshot taken after one successful ACTIVE QP.');
end
prev=D.supervisor.lastActiveTrace;
needed={'x','auxBefore','auxAfter','w','uSafe','time'};
for i=1:numel(needed)
    if ~isfield(prev,needed{i}),error('Prior accepted QP trace missing %s',needed{i});end
end
x0=prev.x(:); xActual=D.x(:); P=D.P(:); dt=D.dt;
if numel(P)~=9 || numel(x0)~=19 || numel(xActual)~=19
    error('Unexpected physical state or parameter sizes.');
end
f0=prev.auxBefore(1); fDot0=prev.auxBefore(2);
fDDot=prev.w(1); M=prev.uSafe(2:4);
if ~isfinite(dt) || dt<=0, error('Invalid dt'); end
odeopts=odeset('RelTol',1e-10,'AbsTol',1e-12);
[~,Yquadratic]=ode45(@(tau,x)localRHS(x,f0+tau*fDot0+.5*tau^2*fDDot,M,P),[0 dt],x0,odeopts);
[~,Yhold]=ode45(@(tau,x)localRHS(x,f0,M,P),[0 dt],x0,odeopts);
xQuadratic=Yquadratic(end,:).';
xHold=Yhold(end,:).';
R=struct();
R.timeStart=prev.time;R.timeEnd=D.time;R.dt=dt;
R.xActual=xActual;R.xPredQuadratic=xQuadratic;R.xPredZOH=xHold;
R.errQuadratic=xQuadratic-xActual;
R.errZOH=xHold-xActual;
R.f0=f0;R.fDot0=fDot0;R.fDDot=fDDot;R.fPredEnd=f0+dt*fDot0+.5*dt^2*fDDot;
R.auxEnd=prev.auxAfter(:);
% Position, velocity, cable direction, and body-rate state discrepancies.
blocks={8:10,11:13,14:16,5:7};
labels={'pLoad','vLoad','pCableDirection','bodyRate'};
fprintf('\n=== PHYSICAL CBF ONE-STEP MODEL COMPARISON ===\n');
fprintf('t: %.6f -> %.6f, dt=%.6f s\n',prev.time,D.time,dt);
fprintf('f0=%.9g N, fDot0=%.9g N/s, fDDot=%.9g N/s^2\n',f0,fDot0,fDDot);
fprintf('predicted end thrust (quadratic)=%.9g N; zero-order hold=%.9g N\n',R.fPredEnd,f0);
for i=1:numel(blocks)
    ii=blocks{i};
    a=norm(R.errQuadratic(ii),2); b=norm(R.errZOH(ii),2);
    fprintf('%-18s error quadratic=% .6e, ZOH=% .6e\n',labels{i},a,b);
    R.errorByBlock.(labels{i})=[a,b];
end
fprintf('NOTE: these errors compare against ESTIMATED current x, not true plant state.\n');
% Evaluate the same obstacle/component if available in failure snapshot.
R.psi=struct();
try
    obsIDs=arrayfun(@(o)double(o.id),D.obsList);
    target=find(obsIDs==2,1,'first');
    if isempty(target),target=min(2,numel(D.obsList));end
    obst=D.obsList(target);
    ref=D.xd([3,7,11,15,19,23,27]);
    kappa=D.kappa;
    geo=D.shapes.uav;
    R.psi.predQuadratic=localEvalPsi(xQuadratic,prev.auxAfter,ref,D.F1,P,obst,geo,kappa);
    R.psi.predZOHwithCBFAux=localEvalPsi(xHold,prev.auxAfter,ref,D.F1,P,obst,geo,kappa);
    R.psi.predZOHwithPhysicalAux=localEvalPsi(xHold,[f0;0],ref,D.F1,P,obst,geo,kappa);
    R.psi.actualWithCBFAux=localEvalPsi(xActual,prev.auxAfter,ref,D.F1,P,obst,geo,kappa);
    fprintf('UAV obs=%g psi3: actual with CBF aux=%+.8g\n',double(obst.id),R.psi.actualWithCBFAux(end));
    fprintf('           model predicted (quadratic)=%+.8g\n',R.psi.predQuadratic(end));
    fprintf('           predicted ZOH x, CBF aux=%+.8g\n',R.psi.predZOHwithCBFAux(end));
    fprintf('           predicted ZOH x, [f0;0] aux=%+.8g\n',R.psi.predZOHwithPhysicalAux(end));
catch ME
    R.psi.auditError=ME.message;
    fprintf('Optional psi3 comparison unavailable: %s\n',ME.message);
end
fprintf('This is a discrepancy audit, NOT a sampled-data safety certificate.\n');
end

function psi=localEvalPsi(x,aux,ref,F1,P,obs,geom,kappa)
% Unit-length states may accumulate tiny integration errors.
x=x(:);x(1:4)=x(1:4)/norm(x(1:4));x(14:16)=x(14:16)/norm(x(14:16));
x(17:19)=x(17:19)-x(14:16)*(x(14:16).'*x(17:19));
ev=Generate_CBF_ActuatorAware_6Methods('evaluate','physical','S3','uav',...
    x,aux,ref,F1,P,obs,geom,kappa);
psi=ev.info.psiChain(:);
end

function dx=localRHS(x,f,M,P)
q=x(1:4);wb=x(5:7);vL=x(11:13);pt=x(14:16);wl=x(17:19);
m=P(1);J=P(2:4);g=P(5);mL=P(6);L=P(7);
% Body z axis in world frame under quaternion [qw,qx,qy,qz].
b3=[2*(q(2)*q(4)+q(1)*q(3));...
    2*(q(3)*q(4)-q(1)*q(2));...
    q(1)^2-q(2)^2-q(3)^2+q(4)^2];
force=f*b3;
pTDot=cross(wl,pt);
wLDot=cross(-pt,force)/(m*L);
aL=[P(8);P(9);-g]+((dot(pt,force)-m*L*dot(pTDot,pTDot))/(m+mL))*pt;
qDot=.5*[-dot(q(2:4),wb);...
    q(1)*wb(1)-q(4)*wb(2)+q(3)*wb(3);...
    q(4)*wb(1)+q(1)*wb(2)-q(2)*wb(3);...
    -q(3)*wb(1)+q(2)*wb(2)+q(1)*wb(3)];
wBDot=(M(:)-cross(wb,J.*wb))./J;
dx=[qDot;wBDot;vL;aL;pTDot;wLDot];
end

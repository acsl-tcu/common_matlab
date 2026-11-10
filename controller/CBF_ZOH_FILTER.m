function out = CBF_ZOH_FILTER(action,varargin)
% CBF_ZOH_FILTER  Experimental sample-and-hold nonlinear predictive filter.
% This file does NOT generate CBF functions and does NOT modify the HLC,
% EKF, reference, plant, obstacles or old HOCBF kernels.
%
% 'filter': mode,obj,x19,xd28,vfNom6,P9,uRaw4,obs,method,shapes,opts,dt
% 'selftest': independent geometry and hover-dynamics consistency checks.
%
% In both modes, decision U=[f;Mx;My;Mz] is the command applied by MODEL_CLASS
% and held constant on [t_k,t_k+dt]. PHYSICAL minimizes physical-input
% deviation. VIRTUAL minimizes deviation of the EXACT current HLC inverse
% coordinates [v1; v2; v3; v4] corresponding to the same physical command.
% This is a NEW static virtual-input optimization; it is NOT the old
% fourth-order virtual auxiliary dynamical extension.
%
% Safety constraints use support-plane separation of S1--S4 from static,
% arbitrarily oriented ellipsoid enclosures. Every collocation node must
% have enough clearance for a SPEED-BOUND tube to the inter-node midpoint.
% Speed-bound reasoning is rigorous for the written continuous ODE given an
% EXACT node state. RK4 integration error and estimator error are NOT bounded
% here; hence NOT a formal intersample or recursive-feasibility guarantee.
% The 3.5 s preview is finite-horizon feasibility, NOT a terminal invariant
% set; no recursive-feasibility or 25 ms realtime claim. Failure is CLOSED.
if nargin < 1, error('CBF:ZOHAction','Action required.'); end
switch lower(char(action))
    case 'filter'
        out=localFilter(varargin{:});
    case 'selftest'
        out=localSelftest();
    otherwise
        error('CBF:ZOHAction','Unknown ZOH action: %s',char(action));
end
end

function out=localFilter(mode,obj,x,xd,vf,P,uRaw,obs,method,shapes,opts,dt)
clock=tic;
mode=lower(char(mode));method=upper(char(method));
assert(any(strcmp(mode,{'virtual','physical'})),'CBF:ZOHMode');
assert(any(strcmp(method,{'S1','S2','S3','S4'})),'CBF:ZOHGeometry');
assert(numel(x)==19 && numel(xd)==28 && numel(vf)==6 && ...
    numel(P)==9 && numel(uRaw)==4 && isstruct(obs), 'CBF:ZOHDimensions');
assert(isscalar(dt) && isfinite(dt) && dt>0 && dt<=0.1,'CBF:ZOHdt');
x=double(x(:));xd=double(xd(:));vf=double(vf(:));
P=double(P(:));uRaw=double(uRaw(:));
assert(all(isfinite([x;xd;vf;P;uRaw])) && P(1)>0 && ...
    all(P(2:4)>0) && P(6)>0 && P(7)>0 && P(5)>0, ...
    'CBF:ZOHInvalidState','Only positive estimated payload mass is supported.');
assert(abs(norm(x(1:4))-1)<1e-3 && ...
    abs(norm(x(14:16))-1)<1e-3 && ...
    abs(dot(x(14:16),x(17:19)))<3e-3, 'CBF:ZOHManifold');
if ~isfield(opts,'uLimits'),opts.uLimits=[0;20;1;1;1];end
lim=double(opts.uLimits(:));
assert(numel(lim)==5 && lim(1)>=0 && lim(1)<lim(2) && ...
    all(lim(3:5)>0) && all(isfinite(lim)),'CBF:ZOHBounds');
if ~isfield(opts,'zohSubsteps'),opts.zohSubsteps=4;end
nSub=opts.zohSubsteps;
assert(isscalar(nSub) && nSub>=2 && nSub<=40 && nSub==floor(nSub), ...
    'CBF:ZOHSubsteps');
if ~isfield(opts,'zohNumericAllowance_m'),opts.zohNumericAllowance_m=0;end
numericAllowance=opts.zohNumericAllowance_m;
assert(isfinite(numericAllowance) && numericAllowance>=0,'CBF:ZOHNumericAllowance');
assert(~isempty(obs),'CBF:ZOHNoObstacles');
for k=1:numel(obs)
    localCheckObstacle(obs(k));
end
lb=[lim(1);-lim(3:5)];ub=[lim(2);lim(3:5)];
uBase=min(max(uRaw,lb),ub);
if isfield(opts,'lockNominalYaw') && opts.lockNominalYaw
    lb(4)=uBase(4);ub(4)=uBase(4); % original yaw-lock trial 1
end
% Use EXACTLY the estimated load mass supplied by the original HLC (P(6)).
% Do not read physical-plant truth or replace it with a fixed value.
Vbound=localSpeedBound(x,P,lim(2),dt,method,shapes);
bridge=Vbound*dt/(2*nSub) + numericAllowance;
% HLC virtual-coordinate inverse is checked before any optimization.
if strcmp(mode,'virtual')
    mapper=localBuildVirtualMapper(obj,x,xd,vf,P,uBase,lim);
else
    mapper=[];
end
% READ-ONLY one-step model innovation. This cannot separate EKF noise,
% mass-estimation error, and integration error, but exposes discrepancies
% that the analytical selftest (same P on both sides) cannot detect.
innovationPosition=NaN;innovationVelocity=NaN;
if isfield(opts,'zohWarmStart') && isstruct(opts.zohWarmStart) && ...
        isfield(opts.zohWarmStart,'previousState19') && ...
        isfield(opts.zohWarmStart,'previousCommand4') && ...
        isfield(opts.zohWarmStart,'previousP9')
    prior=opts.zohWarmStart;
    try
        if numel(prior.previousState19)==19 && ...
                numel(prior.previousCommand4)==4 && numel(prior.previousP9)==9
            rPrev=localSample(prior.previousCommand4(:), ...
                prior.previousState19(:),prior.previousP9(:), ...
                obs,method,shapes,dt,nSub,0);
            innovationPosition=norm(x(8:10)-rPrev.terminalState19(8:10));
            innovationVelocity=norm(x(11:13)-rPrev.terminalState19(11:13));
        end
    catch
        % Diagnostic only. It must not replace the hard safety checks.
    end
end
[h0,labels]=localGeometryGaps(x,P,obs,method,shapes);
base=localSample(uBase,x,P,obs,method,shapes,dt,nSub,bridge);
% PREDICTIVE SAFETY FILTER: the original 25 ms check is necessary, but
% cannot intervene in time for the 4th/6th-order load/cable dynamics.
% Future inputs are a piecewise-ZOH plan, NOT a rewritten reference path.
% The initial command is STILL physically held for exactly dt.
if ~isfield(opts,'zohPreviewHorizon_s'),opts.zohPreviewHorizon_s=3.5;end
if ~isfield(opts,'zohPreviewStages'),opts.zohPreviewStages=5;end
if ~isfield(opts,'zohPreviewStepsPerStage'),opts.zohPreviewStepsPerStage=10;end
H=double(opts.zohPreviewHorizon_s);
K=double(opts.zohPreviewStages);
nEach=double(opts.zohPreviewStepsPerStage);
assert(isfinite(H) && H>=dt && H<=8 && K>=2 && K==floor(K) && ...
    K<=10 && nEach>=2 && nEach==floor(nEach) && nEach<=20, ...
    'CBF:ZOHPreviewConfiguration');
assert(H/(K*nEach)<=0.1,'CBF:ZOHPreviewGridTooCoarse');
uPlan0=repmat(uBase,1,K);
previewBase=localPreview(uPlan0,x,P,obs,method,shapes, ...
    H,nEach,lim(2),numericAllowance);
failed=false;ef=NaN;fval=NaN;solverMessage='';
optReport=struct('minimumSlack',NaN,'exitflag',NaN,'hardFeasible',false);
planStart=NaN;
if isfield(opts,'zohTime_s') && isscalar(opts.zohTime_s)
    planStart=double(opts.zohTime_s);
end
if base.feasible && previewBase.feasible
    uChosen=uBase;
    selectedPlan=uPlan0;
    selectedPreview=previewBase;
    status='MONITOR';
    ef=0;
else
    % Reuse an already validated plan as long as the *remaining* future
    % trajectory, from the NEW estimated state, still passes every hard
    % constraint. The stored control sequence is NOT unconditionally trusted.
    reused=false;
    if isfield(opts,'zohWarmStart') && isstruct(opts.zohWarmStart) && ...
            isfield(opts.zohWarmStart,'previewPlan') && ...
            isfield(opts.zohWarmStart,'previewStart_s') && ...
            isfield(opts,'zohTime_s')
        old=opts.zohWarmStart;
        oldPlan=old.previewPlan;
        if isnumeric(oldPlan) && isequal(size(oldPlan),size(uPlan0)) && ...
                all(isfinite(oldPlan(:))) && isfinite(old.previewStart_s)
            elapsed=max(0,double(opts.zohTime_s)-double(old.previewStart_s));
            shift=floor(elapsed/(H/K)+1e-10);
            if shift<K
                candidate=oldPlan(:,[min(K,(1:K)+shift)]);
                candidate=min(max(candidate,repmat(lb,1,K)),repmat(ub,1,K));
                prev=localPreview(candidate,x,P,obs,method,shapes, ...
                    H,nEach,lim(2),numericAllowance);
                step=localSample(candidate(:,1),x,P,obs,method,shapes,dt,nSub,bridge);
                if prev.feasible && step.feasible
                    reused=true;
                    selectedPlan=candidate;
                    selectedPreview=prev;
                    uChosen=candidate(:,1);
                    status='ACTIVE';
                    ef=2;
                    if shift>0
                        planStart=double(opts.zohTime_s);
                    else
                        planStart=double(old.previewStart_s);
                    end
                    solverMessage='Previous future plan revalidated at current state.';
                end
            end
        end
    end
    if ~reused
    [selectedPlan,optReport]=localOptimizePreview( ...
        uPlan0,lb,ub,mode,mapper,x,xd,P,obs,method,shapes, ...
        H,nEach,lim(2),numericAllowance,opts,dt,nSub,bridge);
    ef=optReport.exitflag;
    fval=optReport.cost;
    solverMessage=optReport.message;
    if optReport.hardFeasible
        uChosen=selectedPlan(:,1);
        selectedPreview=localPreview(selectedPlan,x,P,obs,method,shapes, ...
            H,nEach,lim(2),numericAllowance);
        status='ACTIVE';
    else
        uChosen=[];
        selectedPlan=[];
        selectedPreview=previewBase;
        status='PREVIEW_INFEASIBLE';
        failed=true;
    end
    end % !reused
end
if ~failed
    selected=localSample(uChosen,x,P,obs,method,shapes,dt,nSub,bridge);
    assert(selected.feasible && all(uChosen>=lb-1e-8) && ...
        all(uChosen<=ub+1e-8),'CBF:ZOHPostcheck');
else
    selected=base;
end
if failed
    wasModified=false;
else
    wasModified=norm(uChosen-uBase,inf)>1e-8;
end
supervisor=struct('active',wasModified,'mode',mode,'aux',[], ...
    'previewPlan',selectedPlan,'previewStart_s',planStart, ...
    'previousState19',x,'previousP9',P,'previousCommand4',uChosen);

monitorRows=repmat(struct('obstacleID',NaN,'component','', ...
    'h',NaN,'normalizedMargin',NaN,'margin',NaN),numel(h0),1);
for i=1:numel(h0)
    monitorRows(i).obstacleID=labels(i).obstacleID;
    monitorRows(i).component=labels(i).component;
    monitorRows(i).h=h0(i);
    monitorRows(i).margin=h0(i)-bridge;
end
monitor=struct('minH',min(h0),'minNormalizedMargin',NaN, ...
    'violations',nnz(h0<0),'rows',monitorRows);
virtualBaseBetaRcond=NaN;
if strcmp(mode,'virtual')
    virtualBaseBetaRcond=mapper.baseBetaRcond;
end
out=struct('success',~failed,'reason',solverMessage,'mode',mode, ...
    'method',method,'uBase',uBase,'uSafe',uChosen,'status',status, ...
    'supervisor',supervisor,'engaged',supervisor.active,'monitor',monitor, ...
    'activeRows',[],'activeQP',struct('exitflag',ef), ...
    'totalSeconds',toc(clock),'minimumPredictedGap',selected.minH, ...
    'minimumSampledTubeMargin',selected.minCertifiedGap, ...
    'predictedTerminalState19',selected.terminalState19, ...
    'speedUpperBound_mps',Vbound,'tubeHalfStepAllowance_m',bridge, ...
    'estimatedLoadmass_kg',P(6),'virtualBaseBeta2Rcond',virtualBaseBetaRcond, ...
    'nonlinearOptimizerCost',fval, ...
    'previewHorizon_s',H,'previewNominalMinGap_m',previewBase.minH, ...
    'previewMinimumGap_m',selectedPreview.minH, ...
    'previewMinimumTubeMargin_m',selectedPreview.minCertifiedGap, ...
    'previewMinimumSlack_m',optReport.minimumSlack, ...
    'oneStepEKFPositionInnovation_m',innovationPosition, ...
    'oneStepEKFVelocityInnovation_mps',innovationVelocity, ...
    'predictionModel','preview piecewise-ZOH-RK4 quaternion/cable model', ...
    'intersampleCertified',false,'recursiveFeasibilityCertified',false);
if failed
    out.reason=sprintf(['ZOH PREVIEW %s/%s fmincon exitflag=%d, current 25ms gap=%.7g m, ' ...
        'nominal %.2fs preview gap=%.7g m, minimum prediction slack=%.7g m. %s'], ...
        mode,method,ef,base.minH,H,previewBase.minH,optReport.minimumSlack,solverMessage);
    % One failure snapshot for post-mortem. The experiment already writes
    % to temporary per-trial output directories inside ONE results ZIP.
    CBF_ZOH_FAILURE=struct('mode',mode,'method',method,'x',x,'P',P, ...
        'uRaw',uRaw,'uBase',uBase,'obstacles',obs,'shapes',shapes, ...
        'dt',dt,'bridge_m',bridge,'speedBound_mps',Vbound, ...
        'base',base,'previewBase',previewBase, ...
        'previewOpt',optReport,'massEstimate_kg',P(6), ...
        'oneStepEKFPositionInnovation_m',innovationPosition, ...
        'oneStepEKFVelocityInnovation_mps',innovationVelocity, ...
        'message',out.reason);
    try,save('CBF_ZOH_failure.mat','CBF_ZOH_FAILURE');catch,end
end
end

function [plan,report]=localOptimizePreview(seed0,lb,ub,mode,mapper,x,xd,P,obs,method,shapes,H,nEach,fMax,numericAllowance,opts,dt,nSub,bridge)
% Two-stage predictive safety filter, with *exactly equivalent* normalized
% actuator coordinates z in [0,1] (no change to the real input bounds).
% First find a hard safe plan. A soft slack search is a feasibility
% recovery DIAGNOSTIC; a positive-slack plan is never used for control.
K=size(seed0,2);nvar=4*K;
lbz=repmat(lb,K,1);ubz=repmat(ub,K,1);span=ubz-lbz;
forward=@(z)lbz+span.*z(:); % normalized z -> physical input sequence
inverse=@(u)(u(:)-lbz)./max(span,eps);
physicalObj=@(u)localPreviewObjective(u,seed0,mode,mapper,x,xd,P,obs,method,shapes,H,nEach,fMax,numericAllowance,lb,ub);
obj=@(z)physicalObj(forward(z));
con=@(z)localPreviewNonlcon(z,lbz,span,x,P,obs,method,shapes,H,nEach,fMax,numericAllowance);
options=optimoptions('fmincon','Algorithm','sqp','Display','off', ...
    'MaxIterations',55,'MaxFunctionEvaluations',2500, ...
    'ConstraintTolerance',1e-6,'OptimalityTolerance',1e-6, ...
    'StepTolerance',1e-9);
seeds={min(max(inverse(seed0),0),1)};
if isfield(opts,'zohWarmStart') && isstruct(opts.zohWarmStart) && ...
        isfield(opts.zohWarmStart,'previewPlan')
    previous=opts.zohWarmStart.previewPlan;
    if isnumeric(previous) && isequal(size(previous),size(seed0)) && ...
            all(isfinite(previous(:)))
        seeds=[{min(max(inverse(previous),0),1)},seeds]; %#ok<AGROW>
    end
end
plan=[];best=inf;ef=-999;msg='Hard predictive optimization has no verified feasible solution.';
for is=1:numel(seeds)
    try
        [z,cost,flag,output]=fmincon(obj,seeds{is},[],[],[],[], ...
            zeros(nvar,1),ones(nvar,1),con,options);
        u=forward(z);
        if all(isfinite(u)) && all(u>=lbz-1e-7) && all(u<=ubz+1e-7)
            pp=localPreview(reshape(u,4,K),x,P,obs,method,shapes,H,nEach,fMax,numericAllowance);
            step=localSample(u(1:4),x,P,obs,method,shapes,dt,nSub,bridge);
            if pp.feasible && step.feasible && cost<best
                plan=reshape(u,4,K);best=cost;ef=flag;msg=output.message;
            end
        end
        if isempty(plan),ef=flag;msg=output.message;end
    catch ME
        msg=ME.message;
    end
    if ~isempty(plan),break;end
end
minimumSlack=NaN;
if isempty(plan)
    z0=[seeds{1};1];
    sObj=@(z)z(end)+1e-4*obj(z(1:nvar));
    sCon=@(z)localPreviewSoftConstraint(z,nvar,lbz,span, ...
        x,P,obs,method,shapes,H,nEach,fMax,numericAllowance);
    try
        [z,~,flag,output]=fmincon(sObj,z0,[],[],[],[], ...
            [zeros(nvar,1);0],[ones(nvar,1);1e4],sCon,options);
        minimumSlack=max(0,z(end));ef=flag;msg=output.message;
        % A soft solver returning nearly zero slack is NOT enough; verify
        % the ORIGINAL non-relaxed constraints and first held 25 ms.
        if all(isfinite(z)) && numel(z)==nvar+1
            u=forward(z(1:nvar));
            candidate=reshape(u,4,K);
            pp=localPreview(candidate,x,P,obs,method,shapes,H,nEach,fMax,numericAllowance);
            step=localSample(candidate(:,1),x,P,obs,method,shapes,dt,nSub,bridge);
            if all(u>=lbz-1e-7) && all(u<=ubz+1e-7) && ...
                    pp.feasible && step.feasible
                plan=candidate;best=physicalObj(u);
            end
        end
    catch ME
        msg=[msg ' | minimum-slack recovery error: ' ME.message];
        minimumSlack=NaN;
    end
end
report=struct('minimumSlack',minimumSlack,'exitflag',ef, ...
    'hardFeasible',~isempty(plan),'message',msg,'cost',best);
end

function [c,ceq]=localPreviewSoftConstraint(z,nvar,lbz,span,x,P,obs,method,shapes,H,nEach,fMax,numericAllowance)
u=lbz+span.*z(1:nvar);
c=localPreviewConstraintVector(u,x,P,obs,method,shapes,H,nEach,fMax,numericAllowance)-z(end);
ceq=[];
end

function [c,ceq]=localPreviewNonlcon(z,lbz,span,x,P,obs,method,shapes,H,nEach,fMax,numericAllowance)
u=lbz+span.*z(:);
c=localPreviewConstraintVector(u,x,P,obs,method,shapes,H,nEach,fMax,numericAllowance);
ceq=[];
end

function c=localPreviewConstraintVector(z,x,P,obs,method,shapes,H,nEach,fMax,numericAllowance)
K=numel(z)/4;
r=localPreview(reshape(z,4,K),x,P,obs,method,shapes,H,nEach,fMax,numericAllowance);
c=r.constraints(:);
end

function J=localPreviewObjective(z,seed0,mode,mapper,x,xd,P,obs,method,shapes,H,nEach,fMax,numericAllowance,lb,ub)
K=numel(z)/4;U=reshape(z,4,K);
span=max(ub-lb,1e-8);
D=bsxfun(@rdivide,bsxfun(@minus,U,seed0(:,1)),span);
J=sum(D(:).^2)/K;
if K>1
    jumps=bsxfun(@rdivide,diff(U,1,2),span);
    J=J+sum(jumps(:).^2)/(K-1);
end
if strcmp(mode,'virtual')
    try
        J=J+localVirtualCost(U(:,1),mapper);
    catch
        J=J+1e8; % singular/nonfinite inverse is never an accepted optimum
    end
end
% Objective compares predictions to the ORIGINAL time-varying 28-jet
% reference; the reference/trajectory generator is NEVER changed.
% A finite Taylor jet is only a local prediction of the given reference,
% not a newly generated path. Safety constraints do not depend on it.
pr=localPreview(U,x,P,obs,method,shapes,H,nEach,fMax,numericAllowance);
if any(~isfinite(pr.terminalState19))
    J=J+1e8;return;
end
pTarget=zeros(3,1);vTarget=zeros(3,1);
for j=0:6
    idx=1+4*j;
    pTarget=pTarget+xd(idx:idx+2)*H^j/factorial(j);
    if j>=1,vTarget=vTarget+xd(idx:idx+2)*H^(j-1)/factorial(j-1);end
end
% These scales are physical: cable length and the current reference speed,
% not hand-tuned collision detection thresholds.
pScale=max(P(7),1);
vScale=max(norm(xd(5:7)),1);
J=J+sum(((pr.terminalState19(8:10)-pTarget)/pScale).^2) ...
   +sum(((pr.terminalState19(11:13)-vTarget)/vScale).^2);
if ~isfinite(J),J=1e10;end
end

function out=localPreview(U,x,P,obs,method,shapes,H,nEach,fMax,numericAllowance)
% Piecewise-ZOH future inputs; every interval has its own conservative
% inter-node speed bound. No change to original reference or HLC.
K=size(U,2);dT=H/(K*nEach);
[hh,~]=localGeometryGaps(x,P,obs,method,shapes);
nRows=numel(hh);nNodes=K*nEach;
c=-inf(nRows,nNodes+1);X=x(:);minH=min(hh);
if any(~isfinite(X)) || any(~isfinite(U(:)))
    out=struct('feasible',false,'minH',-inf,'minCertifiedGap',-inf, ...
        'terminalState19',nan(19,1),'constraints',1e5*ones(nRows,nNodes+1));
    return;
end
try
    step=0;
    for j=1:K
        u=U(:,j);
        for k=1:nEach
            step=step+1;
            tube=localSpeedBound(X,P,fMax,dT,method,shapes)*dT/2+numericAllowance;
            c(:,step)=max(c(:,step),tube-hh);
            a=localRHS(X,u,P);
            b=localRHS(X+(dT/2)*a,u,P);
            d=localRHS(X+(dT/2)*b,u,P);
            e=localRHS(X+dT*d,u,P);
            X=X+(dT/6)*(a+2*b+2*d+e);
            [hh,~]=localGeometryGaps(X,P,obs,method,shapes);
            c(:,step+1)=tube-hh;
            minH=min(minH,min(hh));
            if any(~isfinite(X)) || any(~isfinite(hh)) || norm(X)>1e8
                error('CBF:PreviewNonfinite','Prediction state diverged.');
            end
        end
    end
catch
    c(:)=1e5;X=nan(19,1);minH=-inf;
end
out=struct('feasible',all(c(:)<=1e-8), ...
    'minH',minH,'minCertifiedGap',-max(c(:)), ...
    'terminalState19',X,'constraints',c);
end

function [c,ceq]=localConstraint(u,x,P,obs,method,shapes,dt,nSub,bridge)
r=localSample(u,x,P,obs,method,shapes,dt,nSub,bridge);
c=bridge-r.gaps(:);ceq=[];
end

function result=localSample(u,x,P,obs,method,shapes,dt,nSub,bridge)
X=x(:);dT=dt/nSub;
[hh,~]=localGeometryGaps(X,P,obs,method,shapes);
gaps=zeros(numel(hh),nSub+1);gaps(:,1)=hh;
for k=1:nSub
    k1=localRHS(X,u,P);
    k2=localRHS(X+(dT/2)*k1,u,P);
    k3=localRHS(X+(dT/2)*k2,u,P);
    k4=localRHS(X+dT*k3,u,P);
    X=X+(dT/6)*(k1+2*k2+2*k3+k4);
    [gaps(:,k+1),~]=localGeometryGaps(X,P,obs,method,shapes);
end
result=struct('gaps',gaps,'terminalState19',X, ...
    'minH',min(gaps(:)), ...
    'minCertifiedGap',min(gaps(:))-bridge, ...
    'feasible',all(isfinite(gaps(:))) && all(gaps(:)>=bridge-1e-8));
end

function dx=localRHS(x,u,P)
% Compare with Generate... localJetRHS at polynomial order zero. The same
% 19-state quaternion, rigid-cable dynamics is used for both architectures.
q=x(1:4);w=x(5:7);vl=x(11:13);pt=x(14:16);wL=x(17:19);
m=P(1);J=P(2:4);g=P(5);mL=P(6);L=P(7);
f=u(1);M=u(2:4);
b3=[2*(q(2)*q(4)+q(1)*q(3)); ...
    2*(q(3)*q(4)-q(1)*q(2)); ...
    q(1)^2-q(2)^2-q(3)^2+q(4)^2];
qdot=.5*[-q(2:4).'*w; ...
    q(1)*w(1)-q(4)*w(2)+q(3)*w(3); ...
    q(4)*w(1)+q(1)*w(2)-q(2)*w(3); ...
    -q(3)*w(1)+q(2)*w(2)+q(1)*w(3)];
wdot=(M-cross(w,J.*w))./J;
ptdot=cross(wL,pt);
force=f*b3;
speed2=dot(ptdot,ptdot);
vdot=pt*((dot(pt,force)-m*L*speed2)/(m+mL)) ...
    +[P(8);P(9);-g];
wLdot=cross(-pt,force)/(m*L);
dx=[qdot;wdot;vl;vdot;ptdot;wLdot];
end

function bound=localSpeedBound(x,P,fMax,dt,method,shapes)
% Exact kinematic/acceleration bounds on [0,dt] for bounded thrust.
% |wL(t)| <= |wL(0)|+dt*Fmax/(m*L).
m=P(1);mL=P(6);L=P(7);g=P(5);
W=norm(x(17:19))+dt*fMax/(m*L);
aMax=g+norm(P(8:9))+fMax/(m+mL)+m*L*W^2/(m+mL);
vBound=norm(x(11:13))+dt*aMax;
% A cable-aligned ellipsoid's farthest point moves faster under rotation
% than the physical centerline. Account for enclosing representation.
switch method
    case 'S1'
        factor=max(L,L/2+shapes.single(1));
    case 'S3'
        factor=max(L,L/2+shapes.cable(1));
    case 'S4'
        e=shapes.capsule(2);
        factor=max(L,L/2+(L/2)*sqrt(1+e^2));
    otherwise
        factor=L;
end
bound=vBound+factor*W;
assert(isfinite(bound) && bound>=0,'CBF:ZOHSpeedBound');
end

function [h,labels]=localGeometryGaps(x,P,obs,method,shapes)
L=P(7);pL=x(8:10);pt=x(14:16);
if abs(norm(pt)-1)>1e-3
    error('CBF:ZOHManifold','Numerical trajectory departed unit cable-direction manifold.');
end
pt=pt/norm(pt);
switch method
    case 'S1'
        items={'single'};
    case 'S2'
        items=repmat({'sphere'},1,size(shapes.spheres,1));
    case 'S3'
        items={'uav','cable','load'};
    case 'S4'
        items={'uav','capsule','load'};
end
h=zeros(numel(obs)*numel(items),1);
labels=repmat(struct('obstacleID',NaN,'component',''),numel(h),1);
idx=0;
for io=1:numel(obs)
    ob=obs(io);cObs=ob.p_center(:);
    r=ob.ellipsoid_radii(:);R=ob.R_obs;
    for j=1:numel(items)
        name=items{j};
        switch name
            case 'single'
                lambda=.5;a=shapes.single(1);b=shapes.single(2);
                support=@(n)sqrt(b^2+(a^2-b^2)*(dot(n,pt)^2));
            case 'sphere'
                lambda=shapes.spheres(j,1);radius=shapes.spheres(j,2);
                support=@(~)radius;
            case 'uav'
                lambda=1;radii=shapes.uav(:);
                support=@(n)norm(radii.*n);
            case 'load'
                lambda=0;radii=shapes.load(:);
                support=@(n)norm(radii.*n);
            case 'cable'
                lambda=.5;a=shapes.cable(1);b=shapes.cable(2);
                support=@(n)sqrt(b^2+(a^2-b^2)*(dot(n,pt)^2));
            case 'capsule'
                lambda=.5;rad=shapes.capsule(1);epsi=shapes.capsule(2);
                support=@(n)rad+0.5*L*sqrt(dot(n,pt)^2+epsi^2);
        end
        center=pL-lambda*L*pt;
        d=cObs-center;dn=norm(d);
        if dn<1e-8
            gap=-1e6;
        else
            n=d/dn;
            margin=0;if isfield(ob,'d_margin'),margin=ob.d_margin;end
            gap=dn-support(n)-norm(r.*(R.'*n))-margin;
        end
        idx=idx+1;h(idx)=gap;
        if isfield(ob,'id'),labels(idx).obstacleID=double(ob.id);
        else,labels(idx).obstacleID=io;end
        labels(idx).component=name;
    end
end
end

function localCheckObstacle(ob)
assert(isfield(ob,'p_center') && isfield(ob,'ellipsoid_radii') && ...
    isfield(ob,'R_obs'),'CBF:ZOHObstacleData');
assert(numel(ob.p_center)==3 && numel(ob.ellipsoid_radii)==3 && ...
    all(ob.ellipsoid_radii>0) && isequal(size(ob.R_obs),[3,3]) && ...
    norm(ob.R_obs.'*ob.R_obs-eye(3),'fro')<1e-5 && det(ob.R_obs)>0, ...
    'CBF:ZOHObstacleGeometry');
assert(all(isfinite([ob.p_center(:);ob.ellipsoid_radii(:);ob.R_obs(:)])), ...
    'CBF:ZOHObstacleNonfinite');
if isfield(ob,'d_margin')
    assert(isscalar(ob.d_margin) && isfinite(ob.d_margin) && ...
        ob.d_margin>=0,'CBF:ZOHObstacleMargin');
end
if isfield(ob,'v_center') && norm(ob.v_center)>1e-12
    error('CBF:ZOHMovingObstacle','Time-varying obstacles are not supported by this ZOH model.');
end
end

function mapper=localBuildVirtualMapper(obj,x,xd,vf,P,uBase,lim)
% Numerical verification of thrust-affine first-layer inversion.
ref=reshape(xd,1,[]);pRow=reshape(P,1,[]);v0=reshape(vf,1,[]);
v0(1)=0;v1=v0;v1(1)=1;v2=v0;v2(1)=2;
f0=obj.Uf_SuspendedLoadxyDst(x,ref,v0,pRow);
f1=obj.Uf_SuspendedLoadxyDst(x,ref,v1,pRow);
f2=obj.Uf_SuspendedLoadxyDst(x,ref,v2,pRow);
a=f0(1);slope=f1(1)-a;
assert(isfinite(slope) && abs(slope)>1e-9 && ...
    abs(f2(1)-a-2*slope)<1e-6*(1+abs(a)+abs(slope)), ...
    'CBF:ZOHVirtualThrustMap','HLC thrust map is nonaffine/singular.');
mapper=struct('obj',obj,'x',x,'ref',ref,'vf',v0,'P',pRow, ...
    'f0',a,'slope',slope,'bounds',lim);
[mapper.vBase,mapper.baseBetaRcond]=localInverseVirtual(uBase,mapper);
% Derived from allowable PHYSICAL thrust and moment spans, not heuristic
% HOCBF activation gains.
mapper.scale=[(lim(2)-lim(1))/abs(slope);ones(3,1)];
B=obj.Beta2_SuspendedLoadxyDst(x,ref,v0,pRow);
% Physical -> virtual coordinates uses B*M, never inv(B).  A rank-deficient
% B does NOT make this forward map undefined. It DOES mean that the
% inverse HLC map / virtual actuation is not locally one-to-one; record
% the conditioning for the research comparison, but do not conflate it
% with inability to evaluate this physical-input objective.
assert(all(isfinite(B(:))),'CBF:ZOHVirtualNonfinite', ...
    'Nonfinite HLC Beta2 in virtual objective reference frame.');
mapper.scale(2:4)=max(sum(bsxfun(@times,abs(B),lim(3:5).'),2),1e-6);
end

function [v,betaRcond]=localInverseVirtual(u,m)
% This is a FORWARD evaluation from a feasible PHYSICAL command to its
% HLC virtual-output coordinates. No matrix inverse is taken here.
vv=m.vf;vv(1)=(u(1)-m.f0)/m.slope;
B=m.obj.Beta2_SuspendedLoadxyDst(m.x,m.ref,vv,m.P);
if any(~isfinite(B(:)))
    error('CBF:ZOHVirtualNonfinite', ...
        'Nonfinite HLC Beta2 for physical thrust %.9g N.',u(1));
end
betaRcond=rcond(B); % audit only: invertibility is a DIFFERENT question
r0=m.obj.V2_alpha2_SuspendedLoadxyDst(m.x,m.ref,vv,zeros(3,1),m.P);
% Original HLC relation B*M=v2-alpha2; with r0=-alpha2,
% the virtual coordinate is v2=B*M-r0. This can be evaluated even if
% B is rank-deficient, unless the generated expression is nonfinite.
v=[vv(1); B*u(2:4)-r0(:)];
if any(~isfinite(v))
    error('CBF:ZOHVirtualNonfinite', ...
        'Virtual coordinate not finite at physical thrust %.9g N.',u(1));
end
end

function cost=localVirtualCost(u,mapper)
v=localInverseVirtual(u,mapper);
d=(v-mapper.vBase)./mapper.scale;
cost=sum(d.^2);
end

function result=localSelftest()
P=[0.762;0.012;0.013;0.020;9.81;0.14;2;0;0];
x=[1;0;0;0; zeros(3,1); 0;0;5; zeros(3,1);0;0;-1;zeros(3,1)];
ob=struct('p_center',[20;10;20], ...
    'ellipsoid_radii',[1;2;3],'R_obs',eye(3), ...
    'd_margin',0.5,'v_center',zeros(3,1),'id',1);
sh=struct('single',[2.1;0.7],'spheres',[(0:.1:1)', .3*ones(11,1)], ...
    'uav',.45*ones(3,1),'cable',[1.1;.05], ...
    'capsule',[.015;.005],'load',.3*ones(3,1));
hover=[(P(1)+P(6))*P(5);0;0;0];
reference=localSample(hover,x,P,ob,'S4',sh,.025,4,0);
assert(reference.feasible && reference.minH>5,'CBF:ZOHSelftestGeometry');
qdot=localRHS(x,hover,P);
assert(norm(qdot(8:19))<1e-10 && norm(qdot(1:7))<1e-10, ...
    'CBF:ZOHSelftestHoverDynamics');
for method={'S1','S2','S3','S4'}
    [hs,~]=localGeometryGaps(x,P,ob,method{1},sh);
    assert(all(isfinite(hs)) && all(hs>0),'CBF:ZOHSelftestShape');
end
% Independent cross-check against the ORIGINAL 24-state PLANT code at an
% intentionally nontrivial cable/body-rate state. No physical-plant object
% is touched; only its pure dynamics function is evaluated.
assert(exist('with_load_model_euler_for_HL','file')==2, ...
    'CBF:ZOHOriginalPlantMissing', ...
    'Original model/substance/with_load_model_euler_for_HL.m must be on MATLAB path.');
plantMismatch=NaN;
if exist('with_load_model_euler_for_HL','file')==2
    pT=[0.4;-0.2;-sqrt(0.8)];
    pT=pT/norm(pT);
    wCable=cross(pT,[0.6;-0.3;0.2]);
    wBody=[0.3;-0.2;0.1];vLoad=[0.2;-0.1;0.6];
    xTest=[1;0;0;0; wBody; 1;2;5; vLoad; pT; wCable];
    uTest=[8.7;0.07;-0.04;0.02];
    p24=zeros(24,1);
    p24(7:9)=vLoad;p24(10:12)=wBody;
    p24(13:15)=[1;2;5]-P(7)*pT;
    p24(16:18)=vLoad;p24(19:21)=pT;p24(22:24)=wCable;
    p21=zeros(1,21);
    p21([1,6,7,8,9,20,21])=P([1,2,3,4,5,6,7]);
    pPlant=with_load_model_euler_for_HL(p24,uTest,p21);
    pZOH=localRHS(xTest,uTest,P);
    delta=[pZOH(5:7)-pPlant(10:12); ...
        pZOH(8:10)-pPlant(13:15); ...
        pZOH(11:13)-pPlant(16:18); ...
        pZOH(14:16)-pPlant(19:21); ...
        pZOH(17:19)-pPlant(22:24)];
    plantMismatch=norm(delta,inf);
    assert(plantMismatch<1e-8,'CBF:ZOHPlantMismatch', ...
        'Quaternion predictor and original Euler plant disagree at zero Euler angles: %.4g', ...
        plantMismatch);
end
% Independent prediction gate: initially well-clear but original
% upward reference motion intersects an ellipsoid within 3.5 seconds.
% The new filter must catch this BEFORE the next 25 ms becomes unsafe.
xs=x;xs(10)=12;xs(13)=1.5;
ob1=struct('p_center',[.3;.3;20], ...
    'ellipsoid_radii',[3.5355;3.5355;.7071], ...
    'R_obs',eye(3),'d_margin',.5,'v_center',zeros(3,1),'id',1);
initialGap=min(localGeometryGaps(xs,P,ob1,'S3',sh));
forecast=localPreview(repmat(hover,1,5),xs,P,ob1,'S3',sh,3.5,10,20,0);
assert(initialGap>0 && forecast.minH<0 && ~forecast.feasible, ...
    'CBF:ZOHPreviewNotTriggered', ...
    'Future obstacle must be detected before a 25-ms collision.');
result=struct('passed',true,'plantRhsMaxDifference',plantMismatch, ...
    'hoverStateResidual',norm(qdot), ...
    'sampledMinGap',reference.minH, ...
    'previewSyntheticInitialGap_m',initialGap, ...
    'previewSyntheticFutureGap_m',forecast.minH, ...
    'previewSyntheticFutureViolationDetected',true, ...
    'note','Analytical model and nominal preview check only; no MATLAB flight or recursive safety proof.');
end

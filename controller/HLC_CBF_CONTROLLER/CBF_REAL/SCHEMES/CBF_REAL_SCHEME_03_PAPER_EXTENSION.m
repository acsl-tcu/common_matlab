function [u,info]=CBF_REAL_SCHEME_03_PAPER_EXTENSION(agent,cfg,x,uNom,Praw,dt,varargin)
% ID3 SIM-ONLY: Liu et al. (2026) dynamic thrust extension, Eq.(13),(17).
% Augmented state y=[x19; T; qT]; held controls w=[nuT;tauX;tauY].
% Plant thrust WITHIN the interval is T(s)=T+qT*s+nuT*s^2/2.
% Raw HLC T is used ONLY to form the UNCLIPPED endpoint-tracking nuNom.
% Continuous-time HOCBF-QP does NOT prove sampled-data invariance.
% REV60: full-cable ellipsoid plus optional endpoint spheres and mixed-degree rows.
% Standalone actuator-bound regression, independent of Sim and generator.
if nargin>=1 && (ischar(agent) || (isstring(agent) && isscalar(agent)))
    action=lower(char(agent));
    if strcmp(action,'selftest') && nargin==1
        u=paper_viability_selftest();
        info=struct('success',true,'status','SELFTEST_PASS');
        return
    elseif strcmp(action,'fallback') && nargin==4
        % Usage: [physicalU,details]=SCHEME('fallback',cfg,rawHLC,dt).
        % Reuse this file's EXACT scalar actuator interval functions.
        [u,info]=paper_bounded_continuation(cfg,x,uNom);
        return
    elseif strcmp(action,'continuetest') && nargin==1
        u=paper_continuation_selftest();
        info=struct('success',true,'status','CONTINUE_SELFTEST_PASS');
        return
    end
end
start=tic;
validateattributes(x,{'numeric'},{'vector','numel',19,'finite','real'});
validateattributes(uNom,{'numeric'},{'vector','numel',4,'finite','real'});
validateattributes(Praw,{'numeric'},{'vector','numel',9,'finite','real'});
validateattributes(dt,{'numeric'},{'scalar','positive','finite'});
raw=double(uNom(:)); x=double(x(:)); pp=double(Praw(:));
P=struct('m',pp(1),'J',pp(2:4),'gravity',pp(5),'mL',pp(6), ...
    'cableL',pp(7),'dstx',pp(8),'dsty',pp(9));
if P.m<=0 || P.mL<0 || P.cableL<=0 || any(P.J<=0)
    error('CBFPaper:Parameters','Invalid physical parameters');
end
% First active control cycle starts from the physically requested nominal
% thrust in the physical box. Later cycles inherit T,qT, NEVER reset to HLC.
state=cfg.paperThrustState;
first=~state.initialized;
if first
    T0=min(max(raw(1),cfg.paperThrustBounds(1)),cfg.paperThrustBounds(2));
    q0=0;
else
    T0=double(state.T);q0=double(state.q);
end
tb=cfg.paperTorqueBounds;
tauZ=min(max(raw(4),tb(3,1)),tb(3,2));
u=[T0;min(max(raw(2:3),tb(1:2,1)),tb(1:2,2));tauZ];
info=struct('cbfId',3,'success',false,'status','PAPER_NOT_EVALUATED', ...
    'nominal',raw,'nominalRaw',raw,'initializationFromClippedRaw',first, ...
    'modified',true,'paperThrust0',T0,'paperRate0',q0, ...
    'paperNu',NaN,'paperThrustEnd',NaN,'paperRateEnd',NaN, ...
    'paperNominalNuRaw',NaN,'paperHold',[], ...
    'paperStageValues',[],'paperCurrentH',[],'paperHOCBFResidual',[], ...
    'stageValues',[],'barrierResiduals',[],'maxViolation',NaN, ...
    'nominalFeasible',false,'observedParts',0,'input',u, ...
    'paperControlEffect',[],'paperThrustInterval',[], ...
    'paperViabilityAtStart',[],'paperViabilityAtEnd',[], ...
    'paperAdmissibleNuBeforeViability',[],'paperAdmissibleNuAfterViability',[], ...
    'paperActuatorFailureReason','', ...
    'paperControlScale',cfg.paperControlScale(:), ...
    'paperThrustCap',NaN, ... % obsolete scalar cap: no longer used
    'paperRateCap',NaN, ...
    'solverUsed',false,'solverExitflag',NaN,'elapsedMs',NaN, ...
    'groundActive',false,'recursiveFeasibilityCertified',false, ...
    'sampledDataObstacleCertified',false, ...
    'proofScope','CT HOCBF; sampled-data obstacle/tilt/ground/tension and joint future feasibility NOT certified');
try
    validateattributes(cfg.paperThrustBounds,{'numeric'},{'numel',2,'finite','nonnegative'});
    assert(diff(cfg.paperThrustBounds(:))>0 && cfg.paperThrustBounds(2)<=20,'CBF60:PhysicalBox','Configured thrust must remain inside original plant physical box [0,20].');
    assert(isequal(size(tb),[3,2])&&all(isfinite(tb(:)))&&all(tb(:,1)>=-1)&&all(tb(:,2)<=1)&&all(tb(:,1)<tb(:,2)),'CBF60:PhysicalBox','Torque box must remain inside original plant limits.');
    % Actual state-dependent input limits; used by both normal and fallback.
    K=CBF_REAL_D2T_INPUT_KERNEL(T0,q0,dt,cfg.paperThrustBounds);
    info.paperThrustInterval=K.hold;
    info.paperAdmissibleNuBeforeViability=K.hold;
    info.paperAdmissibleNuAfterViability=K.nu;
    info.paperViabilityAtStart=K.startMargin;
    info.paperRateInterval=K.qRange;
    info.paperNominalNuRaw=2*(raw(1)-T0-dt*q0)/dt^2;
    if ~K.ok
        info.paperActuatorFailureReason=K.status;info.status=['THRUST_' K.status];info.elapsedMs=1000*toc(start);return;
    end
    nuLo=K.nu(1);nuHi=K.nu(2);lo=[nuLo;tb(1:2,1)];hi=[nuHi;tb(1:2,2)];
    if ~isfield(agent.sensor.result,'cbfObservedWorld')
        info.status='SENSOR_SNAPSHOT_MISSING';info.elapsedMs=1000*toc(start);return;
    end
    obs=agent.sensor.result.cbfObservedWorld;
    if ~isstruct(obs) || ~isfield(obs,'parts') || ~isstruct(obs.parts)
        info.status='INVALID_OBSTACLE_SNAPSHOT';info.elapsedMs=1000*toc(start);return;
    end
    parts=obs.parts;
    info.observedParts=numel(parts);
    if isfield(obs,'dynamicIndices') && ~isempty(obs.dynamicIndices)
        info.status='MOVING_OBSTACLES_UNSUPPORTED';info.elapsedMs=1000*toc(start);return;
    end
    if numel(parts)>cfg.paperMaxParts
        info.status='TOO_MANY_OBSTACLES';info.elapsedMs=1000*toc(start);return;
    end
    if isfield(agent.sensor.result,'cbfSensorTime') && ~isempty(varargin) && ...
            isobject(varargin{1}) && isprop(varargin{1},'t') && ...
            abs(double(varargin{1}.t)-double(agent.sensor.result.cbfSensorTime))>1.5*dt
        info.status='STALE_OBSTACLES';info.elapsedMs=1000*toc(start);return;
    end
    if abs(norm(x(1:4))-1)>cfg.paperStateProjectionLimit || ...
       abs(norm(x(14:16))-1)>cfg.paperStateProjectionLimit
        info.status='STATE_MANIFOLD_OUT_OF_RANGE';info.elapsedMs=1000*toc(start);return;
    end
    x(1:4)=x(1:4)/norm(x(1:4));
    x(14:16)=x(14:16)/norm(x(14:16));
    % Reject any ellipsoid not encompassing the physical cable tube.
    ellA=max(P.cableL/2+cfg.paperCableAxialMarginM,cfg.paperCableRadiusM);
    if cfg.paperCableAxialMarginM<0 || ...
         cable_margin(P.cableL/2,ellA,cfg.paperCableRadiusM) ...
            <cfg.paperPhysicalCableRadiusM-1e-10
        info.status='CABLE_ELLIPSOID_NOT_ENCLOSING_CABLE';info.elapsedMs=1000*toc(start);return;
    end
    geom=struct('collisionMode','cable_ellipsoid', ...
        'cableAxisShort',cfg.paperCableRadiusM, ...
        'cableAxisLong',ellA,'cableCenterFromLoad',0.5);
    y=[x;T0;q0];
    common=CBF_COMMON_GEOMETRY(P,struct());
    geom.droneSafetyRadius=common.droneSafetyRadius;
    geom.loadSafetyRadius=common.loadSafetyRadius;
    phase='';if isobject(agent)&&isprop(agent,'cha'),phase=char(agent.cha);elseif numel(varargin)>=2,phase=char(varargin{2});end
    o=struct('mode',cfg.paperPlaneMethod,'droneSphereEnabled',cfg.paperDroneSphereEnabled, ...
        'loadSphereEnabled',cfg.paperLoadSphereEnabled,'extras',true, ...
        'bodyEnabled',cfg.paperBodyTiltEnabled,'cableEnabled',cfg.paperCableTiltEnabled, ...
        'groundEnabled',cfg.paperGroundEnabled&&strcmp(phase,'f'), ...
        'tensionEnabled',cfg.paperTensionEnabled,'obsLambda',cfg.paperLambda, ...
        'bodyLambda',cfg.paperBodyTiltLambda,'cableLambda',cfg.paperCableTiltLambda, ...
        'groundLambda',cfg.paperGroundLambda,'tensionLambda',cfg.paperTensionLambda, ...
        'bodyDeg',cfg.paperBodyTiltMaxDeg,'cableDeg',cfg.paperCableTiltMaxDeg, ...
        'groundZ',cfg.paperGroundZ,'tensionMin',cfg.paperTensionMinN);
    for nm={'bodyDeg','cableDeg'},validateattributes(o.(nm{1}),{'numeric'},{'scalar','real','finite','>',0,'<',90});end
    validateattributes(o.groundZ,{'numeric'},{'scalar','real','finite'});
    validateattributes(o.tensionMin,{'numeric'},{'scalar','real','finite','nonnegative'});
    for nm={'droneSphereEnabled','loadSphereEnabled','bodyEnabled','cableEnabled','groundEnabled','tensionEnabled'}
        assert(isscalar(o.(nm{1}))&&ismember(o.(nm{1}),[0,1]),'CBF60:Flag','Invalid constraint flag');
    end
    for nm={'obsLambda','cableLambda','groundLambda'},validateattributes(o.(nm{1}),{'numeric'},{'numel',4,'real','positive','finite'});end
    for nm={'bodyLambda','tensionLambda'},validateattributes(o.(nm{1}),{'numeric'},{'numel',2,'real','positive','finite'});end
    % Same evaluator and support function as REV58; eta held within tick.
    z=CBF_REAL_D2T_ALL_EVAL(y,zeros(3,1),tauZ,P,geom,parts,o);
    rows=struct('barrierBias',z.b,'barrierInfluence',z.A,'lowerStagesValid',all(z.lower>=0));
    % Legacy 5-row view only for fourth-order constraints; mixed rows logged
    % separately, preserving actual relative degrees and avoiding fake psi4.
    deg=cellfun(@numel,z.stages)-1;four=find(deg==4);
    rows.lowerStages=zeros(4,numel(four));
    for j=1:numel(four),rows.lowerStages(:,j)=z.stages{four(j)}(1:4);end
    info.paperStageValues=rows.lowerStages;
    info.paperCurrentH=z.h(:).';info.currentH=z.h(:);
    info.paperAllStages=z.stages;info.paperConstraintDegree=deg;
    info.paperAllLowerStages=z.lower;info.paperPlanePlan=z.plan;
    info.paperPlaneMethod=char(o.mode);info.constraintNames=z.names;
    info.paperFourthOrderIndices=four;
    info.paperControlEffect=z.A;info.paperAffineError=NaN; % analytic coefficient, no online probe
    info.paperLowerStagesValid=rows.lowerStagesValid;
    info.groundActive=o.groundEnabled;info.paperCurrentTensionN=z.tensionN;
    info.paperTautModelAtStart=z.tensionN>0; % diagnostic, not interval certificate
    info.paperGroundEntryValid=true;
    gi=find(startsWith(z.names,'GROUND_'));
    if ~isempty(gi),info.paperGroundEntryValid=all(cellfun(@(v)all(v(1:end-1)>=0),z.stages(gi)));end
    info.stageValues=[rows.lowerStages;z.b(four).'];
    % Never discard a hard row because a lower stage is negative.
    % Recovery candidates remain explicitly uncertified, including ground.
    % Paper Eq.(17) exact tracking map: HLC raw desired ENDPOINT thrust.
    % The adapter from raw T to nuNom is application-specific, not in paper.
    nuNom=2*(raw(1)-T0-dt*q0)/dt^2;
    info.paperNominalNuRaw=nuNom;
    wNom=[nuNom;raw(2:3)];
    info.nominalBarrierResiduals=rows.barrierBias+rows.barrierInfluence*wNom;
    info.paperInputLower=lo;info.paperInputUpper=hi;
    info.nominalFeasible=rows.lowerStagesValid && ...
        all(wNom >= lo) && ...
        all(wNom <= hi) && ...
        all(rows.barrierBias+rows.barrierInfluence*wNom>=-1e-9);
    S=cfg.paperControlScale(:);
    if any(S<=0) || any(~isfinite(S))
        info.status='INVALID_CONTROL_SCALE';info.elapsedMs=1000*toc(start);return;
    end
    % Strictly convex normalized box-QP in z=S^-1*w (three variables).
    B=rows.barrierInfluence; bias=rows.barrierBias;
    Az=-bsxfun(@times,B,S.'); bz=bias;
    if ~isempty(bz)
        rowScale=max([abs(bz),max(abs(Az),[],2), ...
                     1e-9*ones(size(bz))],[],2);
        Az=bsxfun(@rdivide,Az,rowScale);
        bz=bz./rowScale;
    end
    zNom=wNom./S;
    zlo=lo./S;zhi=hi./S;
    zbox=min(max(zNom,zlo),zhi);
    if isempty(bz)||all(bias+B*(S.*zbox)>=0)
        z=zbox;exitflag=1;
    elseif numel(bz)==1
        [z,exitflag]=one_row_box_qp(bz, -Az, zNom,zlo,zhi);
    elseif exist('quadprog','file')==2 || exist('quadprog','file')==6
        persistent qopts
        if isempty(qopts),qopts=optimoptions('quadprog','Display','off');end
        info.solverUsed=true;
        [z,~,exitflag]=quadprog(eye(3),-zNom,Az,bz,[],[], ...
            zlo,zhi,[],qopts);
    else
        info.status='MULTI_OBSTACLE_QUADPROG_REQUIRED';info.elapsedMs=1000*toc(start);return;
    end
    info.solverExitflag=exitflag;
    if isempty(z) || exitflag<=0 || any(~isfinite(z(:)))
        info.status='PAPER_QP_INFEASIBLE';info.elapsedMs=1000*toc(start);return;
    end
    % Round-off after scaling must not escape certified actuator box.
    w=min(hi,max(lo,S.*z(:)));
    if any(~isfinite(w)) || ...
         any(bias+B*w < -1e-7)
        info.status='QP_RESIDUAL_FAILED';info.elapsedMs=1000*toc(start);return;
    end
    nu=w(1);
    Tend=T0+dt*q0+0.5*dt^2*nu;
    qEnd=q0+dt*nu;
    if any(~isfinite([Tend,qEnd])) || ...
        Tend<cfg.paperThrustBounds(1)-1e-8 || ...
        Tend>cfg.paperThrustBounds(2)+1e-8
        info.status='EXTENDED_UPDATE_OUT_OF_BOUNDS';info.elapsedMs=1000*toc(start);return;
    end
    u=[T0;w(2:3);tauZ]; % T applied at START; plant varies it inside dt.
    info.paperNu=nu;
    info.paperThrustEnd=Tend;
    info.paperRateEnd=qEnd;
    info.paperViabilityAtEnd=[Tend+dt*qEnd/2-cfg.paperThrustBounds(1);cfg.paperThrustBounds(2)-Tend-dt*qEnd/2];
    if ~CBF_REAL_D2T_INPUT_KERNEL(T0,q0,dt,cfg.paperThrustBounds,nu)
        info.status='INTERNAL_VIABILITY_RESIDUAL_FAILED';info.elapsedMs=1000*toc(start);return;
    end
    info.paperHOCBFResidual=bias+B*w;
    info.barrierResiduals=info.paperHOCBFResidual;
    info.stageValues=[rows.lowerStages;info.paperHOCBFResidual(four).'];
    for j=1:numel(info.paperAllStages),info.paperAllStages{j}(end)=info.paperHOCBFResidual(j);end
    % Preserve the violated lower stages in diagnostics even when the QP
    % finds a feasible psi4 control and that control is applied.
    info.maxViolation=max([0;-info.paperHOCBFResidual(:);-info.paperAllLowerStages(:)]);
    info.input=u;
    info.paperHold=struct('active',true,'T0',T0,'q0',q0, ...
        'nu',nu,'dt',dt,'Tend',Tend,'qEnd',qEnd);
    info.success=true;
    info.modified=norm([Tend;w(2:3)]-raw(1:3),inf)>1e-9;
    if rows.lowerStagesValid
        info.status='PAPER_CT_QP_FEASIBLE_SAMPLED_UNCERTIFIED';
    else
        info.status='PAPER_CT_QP_RECOVERY_LOWER_STAGE_NEGATIVE_UNCERTIFIED';
    end
catch ME
    info.status='PAPER_EVALUATION_ERROR';
    info.errorIdentifier=ME.identifier;
    info.errorMessage=ME.message;
end
info.elapsedMs=1000*toc(start);
end

function margin=cable_margin(halfL,a,b)
c=a*a-b*b;t=[0,1];
if c>halfL*halfL
    sq=halfL*halfL*b*b/(c*(c-halfL*halfL));
    if sq>0 && sq<1,t(end+1)=sqrt(sq);end
end
margin=min(sqrt(b*b+c*t.^2)-halfL*t);
end

function [z,exitflag]=one_row_box_qp(b,a,desired,lo,hi)
% Exact 3-var isotropic box QP with one affine safety halfspace.
% Constraint b+a'*z>=0; objective 0.5*||z-desired||^2.
a=a(:);lo=lo(:);hi=hi(:);desired=desired(:);
z=min(max(desired,lo),hi);exitflag=1;
if b+a.'*z>=-1e-10,return;end
best=lo.*(a<0)+hi.*(a>=0);
if b+a.'*best<-1e-9 || norm(a)<1e-13
    z=[];exitflag=-1;return;
end
L=0;H=1;
for it=1:96
    zh=min(max(desired+H*a,lo),hi);
    if b+a.'*zh>=0,break;end
    H=2*H;
end
for it=1:64
    mid=(L+H)/2;
    zm=min(max(desired+mid*a,lo),hi);
    if b+a.'*zm>=0,H=mid;else,L=mid;end
end
z=min(max(desired+H*a,lo),hi);
if b+a.'*z<-1e-8,z=[];exitflag=-1;end
end


function [u,fb]=paper_bounded_continuation(cfg,raw,dt)
% Actuator-only fallback; no fake rate reset, no CBF claim.
raw=double(raw(:));B=cfg.paperThrustBounds(:);tb=cfg.paperTorqueBounds;
assert(numel(B)==2&&all(isfinite(B))&&B(1)>=0&&B(2)<=20&&B(2)>B(1),'CBF60:PhysicalBox','Invalid thrust bounds');
assert(isequal(size(tb),[3,2])&&all(isfinite(tb(:)))&&all(tb(:,1)>=-1)&&all(tb(:,2)<=1)&&all(tb(:,1)<tb(:,2)),'CBF60:PhysicalBox','Invalid torque bounds');
state=cfg.paperThrustState;
if state.initialized,T0=double(state.T);q0=double(state.q);else,T0=min(B(2),max(B(1),raw(1)));q0=0;end
K=CBF_REAL_D2T_INPUT_KERNEL(T0,q0,dt,B);
if ~K.ok,error('CBF60:NoActuatorContinuation','Cannot certify actuator continuation: %s',K.status);end
nuNom=2*(raw(1)-T0-dt*q0)/dt^2;nu=min(K.nu(2),max(K.nu(1),nuNom));
Tend=T0+dt*q0+.5*dt^2*nu;qEnd=q0+dt*nu;
if ~CBF_REAL_D2T_INPUT_KERNEL(T0,q0,dt,B,nu)
    % Try the mathematical stopping input, not an unmodelled reset.
    nu=K.stop;Tend=T0+dt*q0+.5*dt^2*nu;qEnd=q0+dt*nu;
    if ~CBF_REAL_D2T_INPUT_KERNEL(T0,q0,dt,B,nu),error('CBF60:NoRoundedContinuation','No certified rounded continuation');end
end
u=[T0;min(tb(:,2),max(tb(:,1),raw(2:4)))];
fb=struct('status','ACTUATOR_ONLY_CONTINUE_UNCERTIFIED','success',false, ...
    'paperThrust0',T0,'paperRate0',q0,'paperNu',nu,'paperThrustEnd',Tend,'paperRateEnd',qEnd, ...
    'rateReset',false,'rateResetReason','','paperHold',struct('active',true,'T0',T0,'q0',q0, ...
    'nu',nu,'dt',dt,'Tend',Tend,'qEnd',qEnd));
end
function report=paper_viability_selftest()
report=CBF_REAL_D2T_REV60_SELFTEST();
end
function report=paper_continuation_selftest()
report=CBF_REAL_D2T_REV60_SELFTEST();
end

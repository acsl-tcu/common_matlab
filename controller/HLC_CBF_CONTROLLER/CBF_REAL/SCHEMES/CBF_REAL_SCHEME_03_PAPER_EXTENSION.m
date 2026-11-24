function [u,info]=CBF_REAL_SCHEME_03_PAPER_EXTENSION(agent,cfg,x,uNom,Praw,dt,varargin)
% ID3 SIM-ONLY: Liu et al. (2026) dynamic thrust extension, Eq.(13),(17).
% Augmented state y=[x19; T; qT]; held controls w=[nuT;tauX;tauY].
% Plant thrust WITHIN the interval is T(s)=T+qT*s+nuT*s^2/2.
% Raw HLC T is used ONLY to form the UNCLIPPED endpoint-tracking nuNom.
% Continuous-time HOCBF-QP does NOT prove sampled-data invariance.
% Existing obstacle geometry is the project's full-cable rotating ellipsoid.
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
tauZ=min(max(raw(4),-1),1);
u=[T0;min(max(raw(2:3),-1),1);tauZ];
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
    'paperThrustCap',cfg.paperAccelMaxNps2, ...
    'paperRateCap',cfg.paperRateMaxNps, ...
    'solverUsed',false,'solverExitflag',NaN,'elapsedMs',NaN, ...
    'groundActive',false,'recursiveFeasibilityCertified',false, ...
    'sampledDataObstacleCertified',false, ...
    'proofScope','CT HOCBF; sampled-data/inter-sample and future feasibility NOT certified');
try
    if ~isfield(agent.sensor.result,'cbfObservedWorld')
        info.status='SENSOR_SNAPSHOT_MISSING';return;
    end
    obs=agent.sensor.result.cbfObservedWorld;
    if ~isstruct(obs) || ~isfield(obs,'parts') || ~isstruct(obs.parts)
        info.status='INVALID_OBSTACLE_SNAPSHOT';return;
    end
    parts=obs.parts;
    info.observedParts=numel(parts);
    if isfield(obs,'dynamicIndices') && ~isempty(obs.dynamicIndices)
        info.status='MOVING_OBSTACLES_UNSUPPORTED';return;
    end
    if numel(parts)>cfg.paperMaxParts
        info.status='TOO_MANY_OBSTACLES';return;
    end
    if isfield(agent.sensor.result,'cbfSensorTime') && ~isempty(varargin) && ...
            isobject(varargin{1}) && isprop(varargin{1},'t') && ...
            abs(double(varargin{1}.t)-double(agent.sensor.result.cbfSensorTime))>1.5*dt
        info.status='STALE_OBSTACLES';return;
    end
    if abs(norm(x(1:4))-1)>cfg.paperStateProjectionLimit || ...
       abs(norm(x(14:16))-1)>cfg.paperStateProjectionLimit
        info.status='STATE_MANIFOLD_OUT_OF_RANGE';return;
    end
    x(1:4)=x(1:4)/norm(x(1:4));
    x(14:16)=x(14:16)/norm(x(14:16));
    % Reject any ellipsoid not encompassing the physical cable tube.
    ellA=max(P.cableL/2+cfg.paperCableAxialMarginM,cfg.paperCableRadiusM);
    if cfg.paperCableAxialMarginM<0 || ...
         cable_margin(P.cableL/2,ellA,cfg.paperCableRadiusM) ...
            <cfg.paperPhysicalCableRadiusM-1e-10
        info.status='CABLE_ELLIPSOID_NOT_ENCLOSING_CABLE';return;
    end
    geom=struct('collisionMode','cable_ellipsoid', ...
        'cableAxisShort',cfg.paperCableRadiusM, ...
        'cableAxisLong',ellA,'cableCenterFromLoad',0.5);
    y=[x;T0;q0];
    if any(~isfinite(y)) || T0<cfg.paperThrustBounds(1)-1e-10 || ...
       T0>cfg.paperThrustBounds(2)+1e-10 || ...
       abs(q0)>cfg.paperRateMaxNps+1e-10
        info.status='INVALID_EXTENDED_STATE';return;
    end
    % Existing GENERATED 4th-order Lie jet, mathematically affine in w.
    % Exact full-interval bounds are derived in the local
    % paper_nu_interval function (no external helper dependency).
    rows=paper_rows_from_generated(y,tauZ,P,geom,parts,cfg.paperLambda(:),dt, ...
         cfg.paperThrustBounds(:),cfg.paperAccelMaxNps2);
    info.paperStageValues=rows.lowerStages;
    info.paperCurrentH=rows.hValues;
    info.paperControlEffect=rows.barrierInfluence;
    info.paperThrustInterval=rows.thrustExactDdTInterval;
    info.paperAffineError=rows.affinityError;
    info.stageValues=[rows.lowerStages;rows.barrierBias.'];
    if ~rows.affineValid
        info.status='NONAFFINE_CBF_JET';return;
    end
    % A negative lower HOCBF stage is logged, NOT an early QP exit.
    % Solve the bounded 3-input QP anyway; a feasible psi4 row gives a
    % recovery candidate, NOT a proof of invariance from an outside state.
    info.paperLowerStagesValid=rows.lowerStagesValid;
    if ~rows.thrustHoldValid
        if ~strcmp(rows.thrustHoldStatus,'OK')
            reason=rows.thrustHoldStatus;
        else
            reason='ACCEL_CAP_NOT_ENOUGH_TO_HOLD_T_WITHIN_BOX';
        end
        info.paperActuatorFailureReason=reason;
        info.status=sprintf(['THRUST_HOLD_INFEASIBLE_%s_' ...
            'T=%.6g_q=%.6g_nuRequired=[%.5g,%.5g]_cap=%.5g'], ...
            reason,T0,q0,rows.thrustExactDdTInterval(1), ...
            rows.thrustExactDdTInterval(2),cfg.paperAccelMaxNps2);
        return;
    end
    % qT(s)=q0+s*nuT: endpoint limits suffice for all s.
    nuLo=max([rows.effectiveDdTInterval(1), ...
          (-cfg.paperRateMaxNps-q0)/dt]);
    nuHi=min([rows.effectiveDdTInterval(2), ...
          (cfg.paperRateMaxNps-q0)/dt]);
    info.paperAdmissibleNuBeforeViability=[nuLo;nuHi];
    if ~(nuLo<=nuHi)
        info.status=sprintf(['EMPTY_EXTENDED_INPUT_BOX_T=%.6g_q=%.6g_' ...
            'nuLo=%.6g_nuHi=%.6g'],T0,q0,nuLo,nuHi);
        return;
    end
    % A second-order bounded actuator needs future braking authority:
    % if q<0: T-Tmin >= q^2/(2*nuMax); if q>0: Tmax-T >= q^2/(2*nuMax).
    % Restrict the SAME scalar nu interval so (Tend,qEnd) remains in this
    % viability kernel. This preserves actuator-only recursive feasibility
    % for exact constant-nu updates; it does NOT certify obstacle CBF safety.
    [nuLo,nuHi,viable,initialMargins,reason]= ...
        paper_nu_viability_interval(T0,q0,dt, ...
        cfg.paperThrustBounds(1),cfg.paperThrustBounds(2), ...
        cfg.paperAccelMaxNps2,nuLo,nuHi);
    info.paperViabilityAtStart=initialMargins;
    info.paperAdmissibleNuAfterViability=[nuLo;nuHi];
    if ~viable
        info.paperActuatorFailureReason=reason;
        info.status=sprintf(['THRUST_ACTUATOR_NOT_VIABLE_%s_' ...
            'T=%.6g_q=%.6g_margins=[%.6g,%.6g]_nu=[%.5g,%.5g]'], ...
            reason,T0,q0,initialMargins(1),initialMargins(2),nuLo,nuHi);
        return;
    end
    lo=[nuLo;-1;-1];hi=[nuHi;1;1];
    % Paper Eq.(17) exact tracking map: HLC raw desired ENDPOINT thrust.
    % The adapter from raw T to nuNom is application-specific, not in paper.
    nuNom=2*(raw(1)-T0-dt*q0)/dt^2;
    info.paperNominalNuRaw=nuNom;
    wNom=[nuNom;raw(2:3)];
    info.nominalFeasible=rows.lowerStagesValid && ...
        all(wNom >= [nuLo;-1;-1]) && ...
        all(wNom <= [nuHi;1;1]) && ...
        all(rows.barrierBias+rows.barrierInfluence*wNom>=-1e-9);
    S=cfg.paperControlScale(:);
    if any(S<=0) || any(~isfinite(S))
        info.status='INVALID_CONTROL_SCALE';return;
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
    if isempty(bz)
        z=min(max(zNom,zlo),zhi);exitflag=1;
    elseif numel(bz)==1
        [z,exitflag]=one_row_box_qp(bz, -Az, zNom,zlo,zhi);
    elseif exist('quadprog','file')==2 || exist('quadprog','file')==6
        persistent qopts
        if isempty(qopts),qopts=optimoptions('quadprog','Display','off');end
        info.solverUsed=true;
        [z,~,exitflag]=quadprog(eye(3),-zNom,Az,bz,[],[], ...
            zlo,zhi,[],qopts);
    else
        info.status='MULTI_OBSTACLE_QUADPROG_REQUIRED';return;
    end
    info.solverExitflag=exitflag;
    if isempty(z) || exitflag<=0 || any(~isfinite(z(:)))
        info.status='PAPER_QP_INFEASIBLE';return;
    end
    w=S.*z(:);
    if any(w<lo-1e-8) || any(w>hi+1e-8) || ...
         any(bias+B*w < -1e-7)
        info.status='QP_RESIDUAL_FAILED';return;
    end
    nu=w(1);
    Tend=T0+dt*q0+0.5*dt^2*nu;
    qEnd=q0+dt*nu;
    if any(~isfinite([Tend,qEnd])) || ...
        Tend<cfg.paperThrustBounds(1)-1e-8 || ...
        Tend>cfg.paperThrustBounds(2)+1e-8 || ...
        abs(qEnd)>cfg.paperRateMaxNps+1e-8
        info.status='EXTENDED_UPDATE_OUT_OF_BOUNDS';return;
    end
    u=[T0;w(2:3);tauZ]; % T applied at START; plant varies it inside dt.
    info.paperNu=nu;
    info.paperThrustEnd=Tend;
    info.paperRateEnd=qEnd;
    info.paperViabilityAtEnd=paper_viability_margins(Tend,qEnd, ...
        cfg.paperThrustBounds(1),cfg.paperThrustBounds(2),cfg.paperAccelMaxNps2);
    if any(info.paperViabilityAtEnd < -1e-8)
        info.status='INTERNAL_VIABILITY_RESIDUAL_FAILED';return;
    end
    info.paperHOCBFResidual=bias+B*w;
    info.barrierResiduals=info.paperHOCBFResidual;
    info.stageValues=[rows.lowerStages;info.paperHOCBFResidual.'];
    % Preserve the violated lower stages in diagnostics even when the QP
    % finds a feasible psi4 control and that control is applied.
    info.maxViolation=max([0;-info.paperHOCBFResidual(:);-rows.lowerStages(:)]);
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


function rows=paper_rows_from_generated(y,tauZ,P,G,parts,lambda,dt,Tbounds,nuCap)
% Single generated call at v=0 and three unit-direction calls produce all
% obstacle rows simultaneously. NO finite differences and NO nonexistent
% QP_ROWS helper. This is EXACT affine-in-w CT-HOCBF at the supplied y.
M=numel(parts);
zero=CBF_REAL_D2T_OBSTACLE_EVAL(y,zeros(3,1),tauZ,P,G,parts,lambda);
B=zeros(M,3);
for j=1:3
    unit=zeros(3,1);unit(j)=1;
    z=CBF_REAL_D2T_OBSTACLE_EVAL(y,unit,tauZ,P,G,parts,lambda);
    B(:,j)=z.stages(5,:).'-zero.stages(5,:).';
end
probe=[31;0.071;-0.043];
v=CBF_REAL_D2T_OBSTACLE_EVAL(y,probe,tauZ,P,G,parts,lambda);
bias=zero.stages(5,:).';
residual=v.stages(5,:).'-bias-B*probe;
if M==0
    err=0;
else
    err=max(abs(residual)./max([ones(M,1),abs(v.stages(5,:).'),abs(bias+B*probe)],[],2));
end
rows=struct();
rows.barrierBias=bias;rows.barrierInfluence=B;
rows.lowerStages=zero.stages(1:4,:);
rows.hValues=zero.stages(1,:);
rows.affinityError=err;
rows.affineValid=all(isfinite([bias;B(:)])) && err<1e-7;
rows.lowerStagesValid=all(rows.lowerStages(:)>=-1e-9);
% Exact constant-nu admissible interval for polynomial T(s).
[lo,hi,valid,msg]=paper_nu_interval(y(20),y(21),dt,Tbounds(1),Tbounds(2));
rows.thrustExactDdTInterval=[lo;hi];
rows.effectiveDdTInterval=[max(lo,-nuCap);min(hi,nuCap)];
rows.thrustHoldValid=valid && rows.effectiveDdTInterval(1)<=rows.effectiveDdTInterval(2);
rows.thrustHoldStatus=msg;
end

function [lo,hi,valid,msg]=paper_nu_interval(T,q,dt,Tmin,Tmax)
% For all 0<s<=dt, Tmin<=T+q*s+0.5*nu*s^2<=Tmax.
% Each s gives a halfspace on nu, so intersect its extreme bounds.
lo=-Inf;hi=Inf;msg='OK';valid=true;
if T<Tmin-1e-10 || T>Tmax+1e-10
    valid=false;msg='INITIAL_T_OUT_OF_BOUNDS';return
end
for which=1:2
    % Float-roundoff guard only: a state 3e-15 N inside T_max can make
    % the analytically feasible nu=-a_max appear 8e-7 N/s^2 infeasible
    % because T_max-T loses significant digits near an exact stopping arc.
    % Maximum tolerated thrust boundary rounding is 1e-10 N.
    thrustTol=1e-10;
    if which==1,B=Tmin-thrustTol;else,B=Tmax+thrustTol;end
    A=B-T;
    samples=dt;
    if abs(q)>1e-14
        stat=2*A/q;
        if stat>0 && stat<dt,samples=[samples,stat];end
    end
    c=2*(A-q*samples)./(samples.^2);
    if which==1
        lo=max(lo,max(c));
        if abs(A)<1e-12 && q<0,lo=Inf;end
        if abs(A)<1e-12 && abs(q)<1e-14,lo=max(lo,0);end
    else
        hi=min(hi,min(c));
        if abs(A)<1e-12 && q>0,hi=-Inf;end
        if abs(A)<1e-12 && abs(q)<1e-14,hi=min(hi,0);end
    end
end
if lo>hi,valid=false;msg='NO_THRUST_PROFILE_IN_BOX';end
end


function margin=paper_viability_margins(T,q,Tmin,Tmax,amax)
% Viability kernel for the input-constrained double integrator
% Tdot=q, qdot=nu, |nu|<=amax, Tmin<=T<=Tmax.
% Stop before hitting the thrust boundary, even if q is nonzero.
margin=[T-Tmin - max(-q,0)^2/(2*amax); ...
        Tmax-T - max(q,0)^2/(2*amax)];
end

function [lo,hi,ok,startMargin,reason]=paper_nu_viability_interval( ...
        T,q,dt,Tmin,Tmax,amax,lo,hi)
% A sampled-data controlled actuator with constant nu during [0,dt].
% Exact interval thrust bounds AND endpoint viability are both required.
% Both terminal viability residuals are monotone in nu (one increases,
% one decreases). Scalar bisection gives tight limits without nonlinear QP.
startMargin=paper_viability_margins(T,q,Tmin,Tmax,amax);
reason='OK';ok=false;
if ~(isscalar(amax) && isfinite(amax) && amax>0)
    reason='INVALID_ACCEL_LIMIT';return
end
if any(startMargin < -1e-8)
    reason='INITIAL_EXTENDED_STATE_OUTSIDE_BRAKING_KERNEL';return
end
if ~(isfinite(lo) && isfinite(hi) && lo<=hi)
    reason='NO_FINITE_ACCELERATION_INTERVAL';return
end
base=T+dt*q;
halfdt2=0.5*dt^2;
lowMargin=@(v) base+halfdt2*v-Tmin ...
                   -max(-(q+dt*v),0).^2/(2*amax);
upMargin=@(v) Tmax-base-halfdt2*v ...
                   -max(q+dt*v,0).^2/(2*amax);
if lowMargin(hi)<-1e-10
    reason='CANNOT_BRAKE_BEFORE_THRUST_MIN';return
end
if upMargin(lo)<-1e-10
    reason='CANNOT_BRAKE_BEFORE_THRUST_MAX';return
end
if lowMargin(lo)<0
    L=lo;H=hi;
    for k=1:52
        mid=(L+H)/2;
        if lowMargin(mid)>=0,H=mid;else,L=mid;end
    end
    lo=H;
end
if upMargin(hi)<0
    L=lo;H=hi;
    for k=1:52
        mid=(L+H)/2;
        if upMargin(mid)>=0,L=mid;else,H=mid;end
    end
    hi=L;
end
if lo>hi+1e-9
    reason='NO_ACCELERATION_PRESERVES_FUTURE_THRUST_FEASIBILITY';return
end
if lo>hi
    lo=(lo+hi)/2;hi=lo;
end
ok=true;
end

function report=paper_viability_selftest()
% No collision geometry / main program required. Deliberately stress the
% actuator toward both bounds with an impossible nominal thrust request.
dt=0.025;amax=100;qmax=40;Tmin=0;Tmax=20;
caseCount=0;minMargin=Inf;
for signOfNom=[-1,1]
    T=8.849;q=0;
    for k=1:100
        [nlo,nhi,v,msg]=paper_nu_interval(T,q,dt,Tmin,Tmax);
        assert(v,['paper_nu_interval invalid: ' msg]);
        nlo=max([nlo,-amax,(-qmax-q)/dt]);
        nhi=min([nhi,amax,(qmax-q)/dt]);
        [nlo,nhi,v,m0,reason]=paper_nu_viability_interval( ...
            T,q,dt,Tmin,Tmax,amax,nlo,nhi);
        assert(v,['paper_nu_viability_interval invalid: ' reason]);
        % As raw HLC T tends to +-1e6, the optimal 3-input box QP with no
        % obstacle tends to the corresponding one-dimensional interval end.
        nu=signOfNom==1;
        if nu,nu=nhi;else,nu=nlo;end
        for s=linspace(0,dt,17)
            Ts=T+q*s+0.5*nu*s^2;
            assert(Ts>=Tmin-1e-8 && Ts<=Tmax+1e-8, ...
                 'Intersample thrust limit violated');
        end
        T=T+dt*q+0.5*dt^2*nu;
        q=q+dt*nu;
        m1=paper_viability_margins(T,q,Tmin,Tmax,amax);
        assert(all(m1>=-1e-8),'Viability lost at next sample');
        assert(abs(q)<=qmax+1e-8,'Thrust rate exceeded');
        minMargin=min([minMargin;m0(:);m1(:)]);
        caseCount=caseCount+1;
    end
end
report=struct('passed',true,'cycles',caseCount, ...
    'testedDt',dt,'accelLimit',amax,'rateLimit',qmax, ...
    'minimumViabilityMargin',minMargin);
fprintf('[ID3 paper actuator viability selftest] PASS (%d cycles)\n',caseCount);
end

function [u,fb]=paper_bounded_continuation(cfg,raw,dt)
% Uncertified CBF failure fallback for PAPER ID3 (NOT an obstacle CBF).
% Only [T,q] actuator constraints are enforced. The closest raw-HLC
% endpoint thrust is selected within the scalar interval of admissible nu.
% Must be called only after a CBF failure; success=false remains unchanged.
raw=double(raw(:)); dt=double(dt);
Tmin=double(cfg.paperThrustBounds(1));
Tmax=double(cfg.paperThrustBounds(2));
amax=double(cfg.paperAccelMaxNps2);
qmax=double(cfg.paperRateMaxNps);
state=cfg.paperThrustState;
resetReason='';
if isstruct(state) && isfield(state,'initialized') && state.initialized && ...
        isfinite(state.T) && isfinite(state.q)
    T0=double(state.T);q0=double(state.q);
else
    T0=min(max(raw(1),Tmin),Tmax);q0=0;
    resetReason='INITIALIZE_FROM_RAW_HLC';
end
% Repair corrupted/out-of-viability actuator state FOR DIAGNOSTIC
% CONTINUATION ONLY. This represents an unmodelled rate reset; it must be
% clearly logged, never mistaken for a physical safety guarantee.
T0=min(max(T0,Tmin),Tmax);
[lo,hi,valid,why]=paper_nu_interval(T0,q0,dt,Tmin,Tmax);
lo=max([lo,-amax,(-qmax-q0)/dt]);
hi=min([hi,amax,(qmax-q0)/dt]);
if valid && lo<=hi
    [lo,hi,valid,~,why]=paper_nu_viability_interval( ...
        T0,q0,dt,Tmin,Tmax,amax,lo,hi);
end
if ~valid || lo>hi
    % Exceptional, non-physical state repair to let the research Sim run.
    % At q0=0, nu=0 always keeps T within its physical box.
    q0=0;lo=0;hi=0;
    resetReason=['RATE_RESET_AFTER_' char(why)];
end
nuNom=2*(raw(1)-T0-dt*q0)/(dt*dt);
nu=min(max(nuNom,lo),hi);
Tend=T0+dt*q0+0.5*dt^2*nu;
qEnd=q0+dt*nu;
% Keep numerical roundoff within the physical actuator box.
assert(all(isfinite([T0,q0,nu,Tend,qEnd])) && ...
       T0>=Tmin-1e-8 && T0<=Tmax+1e-8 && ...
       Tend>=Tmin-1e-8 && Tend<=Tmax+1e-8 && ...
       abs(qEnd)<=qmax+1e-8,'CBFPaper:InvalidContinuation');
% The adapter expects the *initial* thrust plus constant XY/yaw torques.
u=[T0;min(max(raw(2:4),-1),1)];
fb=struct('status','ACTUATOR_ONLY_CONTINUE_UNCERTIFIED', ...
   'success',false,'paperThrust0',T0,'paperRate0',q0, ...
   'paperNu',nu,'paperThrustEnd',Tend,'paperRateEnd',qEnd, ...
   'rateReset',~isempty(resetReason),'rateResetReason',resetReason, ...
   'paperHold',struct('active',true,'T0',T0,'q0',q0, ...
        'nu',nu,'dt',dt,'Tend',Tend,'qEnd',qEnd));
end

function result=paper_continuation_selftest()
% Exercise 500 consecutive unsatisfiable raw-HLC commands in both
% directions, verifying that the backup never needs an obstacle CBF solver.
dt=0.025;cfg=struct('paperThrustBounds',[0;20], ...
    'paperAccelMaxNps2',100,'paperRateMaxNps',40, ...
    'paperThrustState',struct('initialized',true,'T',8.849,'q',0));
cycles=0;rateResets=0;minT=Inf;maxT=-Inf;
for desired=[-436,436]
    cfg.paperThrustState=struct('initialized',true,'T',8.849,'q',0);
    for k=1:250
        [u,fb]=paper_bounded_continuation(cfg,[desired;0.8;-0.7;0.1],dt);
        assert(numel(u)==4 && all(isfinite(u)) && ...
            all(u>=[0;-1;-1;-1]-1e-8) && ...
            all(u<=[20;1;1;1]+1e-8));
        hh=fb.paperHold;
        for ss=linspace(0,dt,17)
            Ts=hh.T0+hh.q0*ss+0.5*hh.nu*ss^2;
            assert(Ts>=-1e-8 && Ts<=20+1e-8, ...
                'CBFPaper:ContinuationThrustProfileViolation');
            minT=min(minT,Ts);maxT=max(maxT,Ts);
        end
        assert(abs(hh.qEnd)<=40+1e-8);
        cfg.paperThrustState=struct('initialized',true,'T',hh.Tend,'q',hh.qEnd);
        rateResets=rateResets+double(fb.rateReset);
        cycles=cycles+1;
    end
end
result=struct('passed',true,'cycles',cycles, ...
    'rateResets',rateResets,'minT',minT,'maxT',maxT);
fprintf('[ID3 bounded CONTINUE selftest] PASS (%d cycles, resets=%d)\n', ...
    cycles,rateResets);
end

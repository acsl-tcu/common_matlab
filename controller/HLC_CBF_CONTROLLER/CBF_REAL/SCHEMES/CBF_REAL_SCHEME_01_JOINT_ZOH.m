function [u,info] = CBF_REAL_SCHEME_01_JOINT_ZOH(agent,cfg,x,uNom,Praw,dt,varargin)
% CBF_REAL_SCHEME_01_JOINT_ZOH
% One existing ellipsoid per observed obstacle; NO second enclosing
% ellipsoid for the obstacle/body Minkowski sum.
% Separating support planes of the actual observed (rotated) ellipsoid and
% drone sphere / core-cable ellipsoid / payload sphere, together with
% interval bounds on their support-plane clearance over ONE ZOH period.
% The three safety shapes cover the complete cable capsule as a UNION.
% Physical inputs [T;tauX;tauY;tauZ] are held ONLY for the current dt.
% IMPORTANT: The core-cable ellipse is meaningful together with the two
% endpoint spheres. Do not use the core ellipse as a replacement for the
% entire cable without the endpoint spheres.
%
% For every separating plane, log h^(0)..h^(4) and psi_0..psi_4 as
% DIAGNOSTICS. The mixed-input physical plant does not satisfy the
% standard uniform-relative-degree-4 HOCBF assumptions. In particular,
% psi_2 depends on thrust, so psi_2..psi_4 are NOT hard CBF conditions.
%
% The actual hard condition is sampled-data (one-step) barrier decay:
% To allow torque in the candidate safety constraint, use the stronger of:
% (1) the earlier acceleration/shape-rate bound and (2) a fourth-order
%     ZOH Taylor lower bound with a fifth-derivative envelope, for drone/load.
% The latter contains direct physical torque effects. It is a provisional
%     bound: its fifth-derivative envelope still requires independent global
%     proof/validation and does NOT prove recursive feasibility.
% The narrow core cable uses only the acceleration/shape-rate bound.
%   min_{0<=s<=dt} h_n(x(s),u_k) >= exp(-rate*dt)*h_n(x_k),
% where n is the fixed separating-plane normal for this step. This is
% strictly stronger than one-step non-intersection when h_n(x_k)>0.
% The geometric lower bound must be a mathematically valid interval lower
% bound for the actual plant. The current quick bound assumes the supplied
% physical parameters and exact state; state/parameter errors are NOT
% certified here. No next-step feasible-input guarantee is claimed.
%
% On each sampling interval T and tau are held fixed, so Tdot=0 and
% taudot=0 THERE. At switching instants u may jump; no differentiability
% across switching instants is assumed. The candidate u is always the
% post-CBF input being evaluated, including modified thrust.
start=tic; u=[];
if cfg.cbfId~=1, error('CBFReal:SchemeId','Scheme 01 requires cbfId=1.');end
validateattributes(dt,{'double'},{'scalar','positive','finite'});
validateattributes(x,{'double'},{'vector','numel',19,'real','finite'});
validateattributes(uNom,{'double'},{'vector','numel',4,'real','finite'});
validateattributes(Praw,{'double'},{'vector','numel',9,'real','finite'});
x=double(x(:));uNom=double(uNom(:));p=double(Praw(:));
P=struct('m',p(1),'J',p(2:4),'gravity',p(5),'mL',p(6), ...
    'cableL',p(7),'dstx',p(8),'dsty',p(9));
if P.m<=0 || P.mL<0 || P.cableL<=0 || any(P.J<=0) || P.gravity<0
    error('CBFReal:PhysicalParameters','Invalid plant parameters.');
end
if abs(norm(x(1:4))-1)>1e-5 || abs(norm(x(14:16))-1)>1e-5
    error('CBFReal:UnitStates','Unit quaternion/cable direction required.');
end
sensorResult=agent.sensor.result;
if ~isstruct(sensorResult) || ~isfield(sensorResult,'cbfObservedWorld') || ...
        ~isfield(sensorResult,'cbfSensorTime')
    error('CBFReal:NoCBFSensor','CBF observation/time missing.');
end
observed=sensorResult.cbfObservedWorld;
if ~isstruct(observed) || ~isscalar(observed) || ~isfield(observed,'parts') ...
        || ~isstruct(observed.parts)
    error('CBFReal:Observation','Observed parts missing.');
end
parts=observed.parts;
sensorTime=double(sensorResult.cbfSensorTime);
if ~isscalar(sensorTime) || ~isfinite(sensorTime)
    error('CBFReal:SensorTime','Invalid timestamp.');
end
if ~isempty(varargin) && isobject(varargin{1}) && isprop(varargin{1},'t') ...
        && abs(double(varargin{1}.t)-sensorTime)>1.5*dt
    error('CBFReal:StaleSensor','Stale observation.');
end
phase='';if numel(varargin)>=2, phase=char(varargin{2});end
G=zoh_geometry(P,observed);
G.includeGround=G.includeGround && strcmp(phase,'f');
% Use the CURRENT estimated payload mass provided by the HLC at this step.
% No fixed true-plant mass or constant [0,0.14] envelope is substituted.
% A rigorous uncertainty envelope needs a proven estimator error bound,
% not just two guessed mass endpoints; this implementation assumes
% the supplied parameter equals the actual mass for its safety certificate.
loadMassRange=[P.mL P.mL];
lb=[0;-1;-1;-1];ub=[20;1;1;1];
scale=[10;1;1;1];weights=ones(4,1);
% Separate optimizer tolerance from independent floating-point acceptance.
% In metres; this handles roundoff near exact equality, NOT model errors.
tol=1e-8;
acceptTol=1e-10;
inputTol=1e-10;
% Stage values are retained for audit only (NOT hard constraints).
stageLambda=[4 4 4 4];
% Nominal-step fraction of separation to retain. This is a sampled-data
% CBF design parameter, NOT a recursive-feasibility certificate. The
% chosen default is an initial test value and must be evaluated in Sim.
barrierRate=1.5; % 1/s
if isfield(cfg,'zohBarrierRatePerSecond')
    barrierRate=double(cfg.zohBarrierRatePerSecond);
end
validateattributes(barrierRate,{'double'}, ...
    {'scalar','real','finite','positive'});
rho=exp(-barrierRate*dt);
% Normals are chosen ONCE at the beginning of this sampling period. Every
% fixed normal provides a sufficient separating-plane inequality. Failure
% to find a separating normal does not imply geometric intersection.
planes=zoh_make_planes(x,parts,P,G);
names={planes.name}.';
[h0,h1]=zoh_plane_initial(x,P,G,planes);
info=struct('success',false,'status','NOT_EVALUATED','cbfId',cfg.cbfId, ...
    'nominal',uNom,'input',[],'modified',false, ...
    'observedParts',numel(parts),'sensorTime',sensorTime, ...
    'phase',phase,'groundActive',logical(G.includeGround), ...
    'loadMassRange',loadMassRange,'massEstimateUsed',P.mL, ...
    'massModelAssumption','estimated mass treated as exact; robustness unproven', ...
    'zohBarrierRatePerSecond',barrierRate,'zohBarrierFactor',rho, ...
    'nominalFeasible',false, ...
    'barrierNumericalToleranceMetres',acceptTol, ...
    'hardBarrierDependsOnTorque',false, ...
    'candidateLowerBoundUsesTorqueJet',false, ...
    'nominalBarrierJacobian',[], ...
    'candidateBarrierJacobian',[], ...
    'nominalTorqueSensitivity',[], ...
    'candidateTorqueSensitivity',[], ...
    'maxViolation',NaN,'elapsedMs',NaN, ...
    'solverUsed',false,'solverExitflag',NaN,'solverIterations',NaN, ...
    'constraintNames',{names},'constraintValues',[], ...
    'minSafetyMargin',NaN,'criticalConstraint','', ...
    'intervalLowerBounds',[],'certificateBranch','DIRECT_SUPPORT_PLANE_ZOH', ...
    'currentH',h0,'currentHdot',h1,'minCurrentH',NaN, ...
    'closestCurrentConstraint','','nominalIntervalMargins',[], ...
    'candidateInput',[],'criticalH0',NaN,'criticalHdot0',NaN, ...
    'criticalObstacleCenter',[NaN;NaN;NaN], ...
    'criticalObstacleAxes',[NaN;NaN;NaN], ...
    'criticalBodyCenterDistance',NaN, ...
    'bodyPhysicalRadii',struct('drone',G.droneRadius, ...
        'cable',G.cableRadius,'load',G.loadRadius), ...
    'bodySafetyRadii',struct('drone',G.droneSafetyRadius, ...
        'cable',G.cableSafetyRadius,'load',G.loadSafetyRadius), ...
    'cableEllipsoidAxes',[G.cableAxisShort;G.cableAxisShort;G.cableAxisLong], ...
    'cableCenterFromLoad',G.cableCenterFromLoad, ...
    'cableTrimFromDrone',G.cableTrimDrone, ...
    'cableTrimFromLoad',G.cableTrimLoad, ...
    'cableCoreLength',G.cableCoreLength, ...
    'geometryMethod','ORIGINAL_OBSERVED_ELLIPSOID_SUPPORT', ...
    'marginUnits','metres (conservative signed separating-plane gap)', ...
    'planeNormals',reshape([planes.n],3,[]).', ...
    'planeSeparationMargins',h0, ...
    'stageLambdaPerSecond',stageLambda, ...
    'stageLabels',{{'h','psi1','psi2','psi3','psi4'}}, ...
    'nominalEvaluatedInput',[], ...
    'nominalStageValues',[], ...
    'nominalRawDerivatives',[], ...
    'nominalMinByStage',[], ...
    'stageValues',[], ...
    'rawDerivatives',[], ...
    'minByStage',[], ...
    'stageCriticalNames',{{}}, ...
    'stageCriticalOrder',NaN,'criticalTest','SAMPLED_DATA_ZOH_DECAY', ...
    'stageCriticalValue',NaN, ...
    'nominalPsi2IntervalLower',[], ...
    'psi2IntervalLower',[], ...
    'psi2IntervalIsHardConstraint',false, ...
    'highOrderStagesAreHardConstraints',false, ...
    'barrierResiduals',[],'nominalBarrierResiduals',[], ...
    'nominalCombinedMinimum',NaN, ...
    'combinedMinimum',NaN, ...
    'proofScope','Conditional one-step barrier. Torque-aware Taylor majorant NOT independently certified; no recursive feasibility or mass-estimation-error guarantee');
if ~isempty(h0)
    [info.minCurrentH,ic]=min(h0);
    info.closestCurrentConstraint=names{ic};
end
% An initially negative plane gap is an *uncertified* configuration. It is
% not proof of collision (particularly for the cable's approximate normal).
if any(h0 < -tol)
    [~,ic]=min(h0);
    info.status='NO_INITIAL_SEPARATION_CERTIFICATE';
    info.criticalConstraint=names{ic};
    info.maxViolation=max(0,-h0(ic));
    info=zoh_critical(info,planes,ic);
    info.elapsedMs=1000*toc(start);
    fprintf(2,'[CBF REAL ZOH] initial separation not certified: %s gap=%.9g m\n', ...
        names{ic},h0(ic));
    return
end
u0=min(max(uNom,lb),ub);
if isempty(planes)
    u=u0;
    info.success=true;info.input=u;info.status='NO_ACTIVE_CONSTRAINTS_BOUNDS_ONLY';
    info.nominalFeasible=all(uNom>=lb & uNom<=ub);
    info.modified=norm(u-uNom,inf)>1e-10;
    info.maxViolation=0;info.elapsedMs=1000*toc(start);
    return
end
% Evaluate ALL derivatives/stages for DIAGNOSTICS. The jet uses
% post-CBF candidate u as a constant ZOH input: h^(j)=j!*coefficient(j+1).
[stage0,deriv0,psi2Zoh0]=zoh_stage_evaluate( ...
    u0,x,P,G,planes,dt,stageLambda);
[g0,branches0]=zoh_support_interval_margins(u0,x,P,G,planes,dt,loadMassRange);
info.nominalEvaluatedInput=u0;
info.nominalStageValues=stage0;
info.nominalRawDerivatives=deriv0;
info.nominalMinByStage=min(stage0,[],2);
info.nominalPsi2IntervalLower=psi2Zoh0;
info.nominalIntervalMargins=g0;
info.constraintValues=g0;info.intervalLowerBounds=g0;
[info.minSafetyMargin,ic]=min(g0);
info.certificateBranch=branches0{ic};
info.criticalConstraint=names{ic};info=zoh_critical(info,planes,ic);
% Nominal passes only when the interval lower bound exceeds rho*h0.
% psi_0..psi_4 are logged separately but do NOT enter feasibility.
residual0=g0-rho*h0;
[combined0,idx0]=min(residual0);
order0=NaN; % no HOCBF derivative order is a hard constraint
info.nominalBarrierResiduals=residual0;
info.nominalCombinedMinimum=combined0;
% Finite-difference check of the REAL admissibility test. Only near the
% boundary, to keep nominal unconstrained steps inexpensive.
if combined0 < 0.05
    info.nominalBarrierJacobian=zoh_barrier_jacobian( ...
        u0,x,P,G,planes,dt,loadMassRange);
    info.nominalTorqueSensitivity=max(abs( ...
        info.nominalBarrierJacobian(:,2:4)),[],1);
end
info.criticalConstraint=names{idx0};info=zoh_critical(info,planes,idx0);
info.stageCriticalOrder=order0;info.stageCriticalValue=combined0;
info.stageCriticalNames={names{idx0}};
info.maxViolation=max([0;-residual0(:);lb-u0(:);u0(:)-ub]);
nomInBounds=all(uNom>=lb & uNom<=ub);
info.nominalFeasible=nomInBounds && combined0>=-acceptTol;
result=u0;
if any(~isfinite(stage0(:)))
    error('CBFReal:Stage','Non-finite CBF stage values.');
end
% The instantaneous psi_i values are NEVER hard constraints here.
% Their sign may help diagnose approaching obstacles, but demanding a
% uniform-order-4 HOCBF would be mathematically unjustified for T/tau.
if combined0>=-acceptTol
    if nomInBounds
        info.status='ZOH_NOMINAL_SAMPLED_DATA_BARRIER_PASSED';
    else
        info.status='ZOH_CLIPPED_SAMPLED_DATA_BARRIER_PASSED';
    end
    exitflag=1;
else
    if exist('fmincon','file')~=2 && exist('fmincon','file')~=6
        info.status='FMINCON_MISSING';
        info.elapsedMs=1000*toc(start);
        return
    end
    info.solverUsed=true;
    fun=@(v)sum(weights.*((v(:)-uNom)./scale).^2);
    con=@(v)zoh_sampled_data_nlcon( ...
        v,x,P,G,planes,dt,loadMassRange,h0,rho);
    opts=optimoptions('fmincon','Algorithm','sqp','Display','off', ...
        'MaxIterations',65,'MaxFunctionEvaluations',400, ...
        'ConstraintTolerance',tol,'OptimalityTolerance',1e-8, ...
        'FiniteDifferenceType','central', ...
        'FiniteDifferenceStepSize',[1e-3;5e-2;5e-2;5e-2], ...
        'TypicalX',[10;0.2;0.2;0.2]);
    try
        [result,~,exitflag,output]=fmincon(fun,u0,[],[],[],[],lb,ub,con,opts);
    catch ME
        info.status='SOLVER_EXCEPTION';info.solverMessage=ME.message;
        info.solverIdentifier=ME.identifier;
        info.elapsedMs=1000*toc(start);
        return
    end
    if isfield(output,'iterations'),info.solverIterations=output.iterations;end
end
info.solverExitflag=exitflag;
if any(~isfinite(result)) || numel(result)~=4
    info.status='NONFINITE_CANDIDATE';info.elapsedMs=1000*toc(start);return
end
[stage,derivatives,psi2Zoh]=zoh_stage_evaluate( ...
    result,x,P,G,planes,dt,stageLambda);
[g,branches]=zoh_support_interval_margins(result,x,P,G,planes,dt,loadMassRange);
info.candidateInput=result(:);info.constraintValues=g;
info.intervalLowerBounds=g;
info.stageValues=stage;info.rawDerivatives=derivatives;
info.minByStage=min(stage,[],2);
info.psi2IntervalLower=psi2Zoh;
[info.minSafetyMargin,ic]=min(g);
info.criticalConstraint=names{ic};info=zoh_critical(info,planes,ic);
residual=g-rho*h0;
[combined,criticalIdx]=min(residual);
criticalOrder=NaN;
info.barrierResiduals=residual;
info.combinedMinimum=combined;
if info.solverUsed
    info.candidateBarrierJacobian=zoh_barrier_jacobian( ...
        result,x,P,G,planes,dt,loadMassRange);
    info.candidateTorqueSensitivity=max(abs( ...
        info.candidateBarrierJacobian(:,2:4)),[],1);
end
info.stageCriticalOrder=criticalOrder;
info.criticalConstraint=names{criticalIdx};
info=zoh_critical(info,planes,criticalIdx);
info.stageCriticalValue=combined;
info.stageCriticalNames={names{criticalIdx}};
info.maxViolation=max([0;-residual(:);lb-result(:);result(:)-ub]);
info.certificateBranch=branches{ic};
info.candidateLowerBoundUsesTorqueJet=any(strcmp(branches, ...
    'ZOH_TAYLOR4_FIFTH_BOUND_ASSUMED'));
% This flag only says the torque-aware branch actually won the bound
% comparison.  A nonzero derivative is checked separately by Jacobian.
info.hardBarrierDependsOnTorque=info.candidateLowerBoundUsesTorqueJet;
% A nonpositive SQP exitflag does not automatically reject a candidate;
% independently check the SAME sampled-data barrier and actuator bounds.
% Negative higher-stage diagnostics are never used to reject the command.
if combined < -acceptTol || any(result(:)<lb-inputTol) || ...
        any(result(:)>ub+inputTol)
    info.status='INFEASIBLE_OR_UNSATISFIED_SAMPLED_DATA_ZOH_BARRIER';
    info.elapsedMs=1000*toc(start);
    zoh_report_stage_failure(sensorTime,phase,info,stage,psi2Zoh,names,stageLambda);
    zoh_send_failure_to_view(sensorTime,info,varargin);
    return
end
u=result(:);info.input=u;info.success=true;
info.modified=norm(u-uNom,inf)>1e-10;
if info.solverUsed
    fprintf('[CBF REAL ZOH] %s: nominal DIAG [h psi1 psi2 psi3 psi4]=[%.4g %.4g %.4g %.4g %.4g]\n', ...
        names{criticalIdx},stage0(:,criticalIdx));
    fprintf('[CBF REAL ZOH] %s: applied DIAG [h psi1 psi2 psi3 psi4]=[%.4g %.4g %.4g %.4g %.4g], psi2ZOH=%.4g\n', ...
        names{criticalIdx},stage(:,criticalIdx),psi2Zoh(criticalIdx));
    info.status='ZOH_SAMPLED_DATA_BARRIER_CORRECTED';
    if exitflag<=0,info.status='ZOH_FEASIBLE_CANDIDATE_NONCONVERGED';end
end
% Store the whole per-order result even on the nominal-fast path.
if isempty(info.stageValues)
    info.stageValues=stage0;
    info.rawDerivatives=deriv0;
    info.minByStage=min(stage0,[],2);
    info.psi2IntervalLower=psi2Zoh0;
    info.combinedMinimum=combined0;
end
if isnan(info.stageCriticalOrder)
    info.stageCriticalOrder=order0;
    info.stageCriticalValue=combined0;
    info.stageCriticalNames={names{idx0}};
end
info.elapsedMs=1000*toc(start);
end

function [c,ceq]=zoh_sampled_data_nlcon(u,x,P,G,planes,dt,massRange,h0,rho)
% The SAME predicate is checked for the nominal, SQP candidates, and the
% applied input. The lower bound is over the entire one-step ZOH interval.
g=zoh_support_interval_margins(u,x,P,G,planes,dt,massRange);
c=rho*h0(:)-g(:);
ceq=[];
end

function [stage,derivatives,psi2Lower]=zoh_stage_evaluate(u,x,P,G,planes,dt,lambda,computePsi2Diagnostic)
% h(t): signed support-plane gap in metres (one frozen separating normal).
% psi_k = (D+lambda_k)...(D+lambda_1) h, for k=1,...,4.
% Relative degrees vary: T generally enters h''; tau enters h''''.
% None of psi2..psi4 is used as a hard constraint for mixed-input ZOH.
% The optional psi2(t) estimate may be extremely conservative for thin
% cable ellipsoids; it is diagnostic only, not an input constraint.
if nargin<8,computePsi2Diagnostic=true;end
n=numel(planes);
stage=zeros(5,n);derivatives=zeros(5,n);psi2Lower=NaN(n,1);
if n==0,return;end
Gjet=G;Gjet.includeGround=false;
J=CBF_REAL_ZOH_DIAG_EVAL(x,u(:),struct([]),P,Gjet,4);
if computePsi2Diagnostic
    M5=zoh_plane_fifth_bounds(x,u,P,G,dt);
end
for k=1:n
    coeff=zoh_plane_series(J.stateTaylor,planes(k),P,G,4);
    derivatives(:,k)=factorial(0:4).'.*coeff(:);
    % Different gains are allowed at each order; no factorial shortcuts.
    for ord=0:4
        poly=[1];
        for j=1:ord,poly=conv(poly,[lambda(j) 1]);end
        stage(ord+1,k)=poly*derivatives(1:ord+1,k);
    end
    if computePsi2Diagnostic
        % Diagnostic only: the 5th-derivative envelope may be excessively
        % conservative, especially for a narrow cable ellipsoid.
        lamSum=lambda(1)+lambda(2);
        lamProd=lambda(1)*lambda(2);
        h=derivatives(:,k);
        B5=M5(planes(k).body);
        B4=abs(h(5))+B5*dt;
        B3=abs(h(4))+abs(h(5))*dt+0.5*B5*dt^2;
        psi2ThirdBound=B5+lamSum*B4+lamProd*B3;
        c0=stage(3,k);
        c1=h(4)+lamSum*h(3)+lamProd*h(2);
        c2=(h(5)+lamSum*h(4)+lamProd*h(3))/2;
        psi2Lower(k)=zoh_poly_min( ...
            [c0 c1 c2 -psi2ThirdBound/6],dt);
    end
end
end

function zoh_report_stage_failure(sensorTime,phase,info,stage,psi2Lower,names,~)
% A certificate/optimization failure is NOT a demonstrated collision.
[bad,i]=min(info.barrierResiduals);
if isempty(i) || ~isfinite(bad)
    bad=info.combinedMinimum;
    i=find(strcmp(names,info.criticalConstraint),1);
end
if isempty(i),i=1;end
fprintf(2,['[CBF REAL ZOH] no verified sampled-data input: t=%.3f ' ...
    'phase=%s critical=%s residual=%.6g m exitflag=%.0f iterations=%.0f\n'], ...
    sensorTime,phase,names{i},bad,info.solverExitflag,info.solverIterations);
fprintf(2,['[CBF REAL ZOH] %s: h=%.6g intervalLower=%.6g ' ...
    'required=%.6g; psi1..4 DIAGNOSTIC=[%.4g %.4g %.4g %.4g], ' ...
    'psi2ZOH DIAGNOSTIC=%.4g\n'], ...
    names{i},stage(1,i),info.intervalLowerBounds(i), ...
    info.zohBarrierFactor*stage(1,i),stage(2:5,i),psi2Lower(i));
end

function planes=zoh_make_planes(x,parts,P,G)
template=struct('name','','kind','','body',0,'n',[0;0;1], ...
    'center',[NaN;NaN;NaN],'axes',[NaN;NaN;NaN],'Q',zeros(3), ...
    'currentPoint',[NaN;NaN;NaN]);
planes=repmat(template,0,1);
cf=[1 G.cableCenterFromLoad 0];kinds={'drone','cable','load'};
for j=1:numel(parts)
    [ctr,Q,ax,lab]=zoh_obstacle_geometry(parts(j),j);
    for k=1:3
        c=cf(k);cent=x(8:10)-c*P.cableL*x(14:16);
        if k==1,Qs=(G.droneSafetyRadius^2)*eye(3);
        elseif k==3,Qs=(G.loadSafetyRadius^2)*eye(3);
        else
            Qs=G.cableAxisShort^2*eye(3)+(G.cableAxisLong^2-G.cableAxisShort^2)* ...
                (x(14:16)*x(14:16).');
        end
        n=zoh_best_plane_normal(cent-ctr,Q,Qs,k~=2);
        item=template;item.kind='obstacle';item.name=[lab '/' kinds{k}];
        item.body=k;item.n=n;item.Q=Q;item.center=ctr;
        item.axes=ax;item.currentPoint=cent;
        planes(end+1,1)=item; %#ok<AGROW>
    end
end
if G.includeGround
    for k=1:3
        item=template;item.kind='ground';item.name=['ground/' kinds{k}];
        item.body=k;item.n=[0;0;1];item.center=[0;0;G.groundZ];
        item.currentPoint=x(8:10)-cf(k)*P.cableL*x(14:16);
        planes(end+1,1)=item; %#ok<AGROW>
    end
end
end

function n=zoh_best_plane_normal(d,Qo,Qb,sphere)
% Each tested normal is a valid supporting-plane certificate; the search
% quality affects conservatism, not soundness. For spherical bodies, the
% normal of the nearest Euclidean point on the ellipsoid is exact.
D=norm(d);
if D<1e-12,n=[1;0;0];return;end
n0=d/D;
n1=zoh_closest_point_normal(d,Qo);
if sphere,n=n1;return;end
[V,~]=eig(Qo);
signs=sign(V.'*d);signs(signs==0)=1;
starts=[n0 n1 V*diag(signs)];
best=-inf;n=n1;
for j=1:size(starts,2)
    v=starts(:,j);v=v/max(norm(v),eps);
    for k=1:14
        margin=zoh_plane_gap(d,Qo,Qb,v);
        if margin>best,best=margin;n=v;end
        go=Qo*v/sqrt(v.'*Qo*v);
        gb=Qb*v/sqrt(v.'*Qb*v);
        grad=d-go-gb;
        tangent=grad-v*(v.'*grad);
        tnorm=norm(tangent);
        if tnorm<1e-11,break;end
        step=0.7;
        for it=1:8
            vv=v+step*tangent;
            vv=vv/norm(vv);
            if zoh_plane_gap(d,Qo,Qb,vv)>=margin
                v=vv;break
            end
            step=step/2;
        end
        if it==8 && zoh_plane_gap(d,Qo,Qb,vv)<margin,break;end
    end
    margin=zoh_plane_gap(d,Qo,Qb,v);
    if margin>best,best=margin;n=v;end
end
end

function n=zoh_closest_point_normal(d,Q)
% Nearest point on origin-centred ellipsoid with shape matrix Q.
[V,S]=eig((Q+Q.')/2);
a2=diag(S);v=V.'*d;
if sum((v.^2)./a2)<=1
    n=d/max(norm(d),eps);return
end
lo=0;hi=max(a2);
for k=1:64
    if sum(a2.*(v.^2)./(a2+hi).^2)<1,break;end
    hi=hi*2;
end
for k=1:65
    mid=(lo+hi)/2;
    if sum(a2.*(v.^2)./(a2+mid).^2)>1
        lo=mid;
    else
        hi=mid;
    end
end
foot=V*((a2.*v)./(a2+(lo+hi)/2));
n=d-foot;
n=n/max(norm(n),eps);
end

function g=zoh_plane_gap(d,Qo,Qb,n)
g=n.'*d-sqrt(max(0,n.'*Qo*n))-sqrt(max(0,n.'*Qb*n));
end

function [h0,h1]=zoh_plane_initial(x,P,G,planes)
h0=zeros(numel(planes),1);h1=h0;
pT=x(14:16);wL=x(17:19);pTdot=cross(wL,pT);
cf=[1 G.cableCenterFromLoad 0];
for i=1:numel(planes)
    z=planes(i);k=z.body;c=cf(k);n=z.n;
    cent=x(8:10)-c*P.cableL*pT;
    vel=x(11:13)-c*P.cableL*pTdot;
    if k==2
        a=G.cableAxisShort;delta=G.cableAxisLong^2-a^2;
        y=n.'*pT;yd=n.'*pTdot;
        sb=sqrt(a^2+delta*y^2);
        sdot=delta*y*yd/sb;
    elseif k==1
        sb=G.droneSafetyRadius;sdot=0;
    else
        sb=G.loadSafetyRadius;sdot=0;
    end
    if strcmp(z.kind,'obstacle')
        obsSupport=sqrt(n.'*z.Q*n);
        h0(i)=n.'*(cent-z.center)-obsSupport-sb;
    else
        h0(i)=cent(3)-G.groundZ-sb;
    end
    h1(i)=n.'*vel-sdot;
end
end

function [g,branch]=zoh_support_interval_margins(u,x,P,G,planes,dt,massRange)
% TWO candidate-dependent lower bounds for the SAME ZOH physical input:
% A) acceleration/shape-rate envelope (independent of torque, conservative);
% B) 4th-order Taylor + 5th-derivative envelope for sphere components.
%    B is a conditional research certificate; its global majorant must
%    be validated independently before a formal invariant-set claim.
% The maximum of two proven lower bounds would also be a lower bound.
% For the central thin cable, only use A (avoid the large shape series).
% In all cases the obstacle is the ORIGINAL observed ellipsoid.
nc=numel(planes);g=zeros(nc,1);branch=cell(nc,1);
if nc==0,return;end
[h0,h1]=zoh_plane_initial(x,P,G,planes);
T=abs(u(1));m=P.m;L=P.cableL;
Omega=norm(x(17:19))+(T/(m*L))*dt;
Aload=P.gravity+norm([P.dstx;P.dsty])+ ...
    (T+m*L*Omega^2)/(m+massRange(1));
pt2=T/(m*L)+Omega^2;
cf=[1 G.cableCenterFromLoad 0];
for j=1:nc
    k=planes(j).body;c=cf(k);
    shapeRate=0;shapeDot0=0;
    if k==2
        delta=G.cableAxisLong^2-G.cableAxisShort^2;
        n=planes(j).n;
        y=n.'*x(14:16);
        ydot=n.'*cross(x(17:19),x(14:16));
        shapeDot0=delta*y*ydot/sqrt(G.cableAxisShort^2+delta*y^2);
        shapeRate=sqrt(delta)*Omega;
    end
    Ac=Aload+c*L*pt2;
    lowerEnd=h0(j)+(h1(j)+shapeDot0-shapeRate)*dt-0.5*Ac*dt^2;
    g(j)=min(h0(j),lowerEnd);
    branch{j}='SUPPORT_ACCELERATION_BOUND';
end
% The Taylor alternative is evaluated once per candidate using the
% generated 19-state ZOH physical dynamics, including all three torques.
% It is useful for the sphere components near an active safety boundary.
% Never use this variant for the thin cable shape without a verified majorant.
if any([planes.body]~=2)
    gJet=zoh_four_input_taylor_lower(u,x,P,G,planes,dt);
    for j=1:nc
        if planes(j).body==2 || ~isfinite(gJet(j)),continue;end
        if gJet(j)>g(j)
            g(j)=gJet(j);
            branch{j}='ZOH_TAYLOR4_FIFTH_BOUND_ASSUMED';
        end
    end
end
end

function g=zoh_four_input_taylor_lower(u,x,P,G,planes,dt)
% Interval lower bound h(t)>=Taylor4(t)-M5*t^5/5!, conditional on
% M5 being a verified uniform bound on |d^5 h/dt^5| over [0,dt].
% The generated ZOH jet includes T and (tauX,tauY,tauZ) directly.
nc=numel(planes);g=-inf(nc,1);
Gjet=G;Gjet.includeGround=false;
J=CBF_REAL_ZOH_DIAG_EVAL(x,u(:),struct([]),P,Gjet,4);
M5=zoh_plane_fifth_bounds(x,u(:),P,G,dt);
for j=1:nc
    body=planes(j).body;
    if body==2,continue;end
    coeff=zoh_plane_series(J.stateTaylor,planes(j),P,G,4);
    if ~isfinite(M5(body)) || M5(body)<0,continue;end
    minorant=[coeff(:).' -M5(body)/factorial(5)];
    g(j)=zoh_poly_min(minorant,dt);
end
end

function J=zoh_barrier_jacobian(u,x,P,G,planes,dt,massRange)
% Differentiates the ACTUAL nonlinear constraint g(u), NOT the 4-input
% objective.  Units: m/N for T, m/(N*m) for each torque.
% Central differences at feasible bounded inputs; no fabricated sensitivity.
if isempty(planes),J=zeros(0,4);return;end
lb=[0;-1;-1;-1];ub=[20;1;1;1];
step=[0.01;0.05;0.05;0.05];
J=nan(numel(planes),4);
for k=1:4
    up=u(:);um=u(:);
    up(k)=min(ub(k),up(k)+step(k));
    um(k)=max(lb(k),um(k)-step(k));
    if up(k)==um(k),continue;end
    gp=zoh_support_interval_margins(up,x,P,G,planes,dt,massRange);
    gm=zoh_support_interval_margins(um,x,P,G,planes,dt,massRange);
    J(:,k)=(gp(:)-gm(:))/(up(k)-um(k));
end
end

function c=zoh_plane_series(X,z,P,G,N)
cf=[1 G.cableCenterFromLoad 0];cBody=cf(z.body);n=z.n;
cent=X(8:10,1:N+1)-cBody*P.cableL*X(14:16,1:N+1);
c=(n.'*cent);
if strcmp(z.kind,'obstacle')
    c(1)=c(1)-n.'*z.center-sqrt(n.'*z.Q*n);
else
    c(1)=c(1)-G.groundZ;
end
if z.body==2
    as=G.cableAxisShort;delta=G.cableAxisLong^2-as^2;
    y=n.'*X(14:16,1:N+1);
    b=delta*conv(y,y);b=b(1:N+1);b(1)=b(1)+as^2;
    s=zeros(1,N+1);s(1)=sqrt(b(1));
    for k=1:N
        prev=0;
        for i=1:k-1,prev=prev+s(i+1)*s(k-i+1);end
        s(k+1)=(b(k+1)-prev)/(2*s(1));
    end
    c=c-s;
elseif z.body==1
    c(1)=c(1)-G.droneSafetyRadius;
else
    c(1)=c(1)-G.loadSafetyRadius;
end
end

function M=zoh_plane_fifth_bounds(x,u,P,G,dt)
% Uniform coefficient majorants over the current ZOH flow. The fifth
% derivative of each projected body centre is bounded by the positive
% Taylor-coefficient convolution of the physical 19-state RHS.
T=abs(u(1));tauN=norm(u(2:4));m=P.m;mL=P.mL;L=P.cableL;
Jmin=min(P.J);Jmax=max(P.J);
W=(sqrt(sum(P.J(:).*x(5:7).^2))+tauN*dt/sqrt(Jmin))/sqrt(Jmin);
Omega=norm(x(17:19))+(T/(m*L))*dt;
Aload=P.gravity+norm([P.dstx;P.dsty])+(T+m*L*Omega^2)/(m+mL);
Vload=norm(x(11:13))+Aload*dt;
N=5;Q=zeros(1,N+1);WB=Q;PT=Q;WC=Q;VL=Q;
Q(1)=1;WB(1)=W;PT(1)=1;WC(1)=Omega;VL(1)=Vload;
for k=0:N-1
    WB(k+2)=(Jmax/Jmin*zoh_conv_at(WB,WB,k)+ ...
        double(k==0)*tauN/Jmin)/(k+1);
    Q(k+2)=0.5*zoh_conv_at(Q,WB,k)/(k+1);
    PT(k+2)=zoh_conv_at(WC,PT,k)/(k+1);
    B3=conv(Q,Q);
    WC(k+2)=(T/(m*L))*zoh_conv_at(PT,B3,k)/(k+1);
    CV=conv(WC,PT);
    scalar=zeros(1,N+1);
    for j=0:k
        scalar(j+1)=T*zoh_conv_at(PT,B3,j)+m*L*zoh_conv_at(CV,CV,j);
    end
    al=zoh_conv_at(scalar,PT,k)/(m+mL);
    if k==0,al=al+P.gravity+norm([P.dstx;P.dsty]);end
    VL(k+2)=al/(k+1);
end
PL=zeros(1,N+1);
for k=1:N,PL(k+1)=VL(k)/k;end
% Support shape s=sqrt(as^2+delta*y^2), |y^(k)|/k! <= PT(k+1).
% Binomial series around any instant with s>=as gives an upper bound on
% the coefficient of t^5 of s. This avoids sign-unsafe sqrt recurrences.
as=G.cableAxisShort;delta=G.cableAxisLong^2-as^2;
bDelta=zeros(1,N+1);
for k=1:N,bDelta(k+1)=delta*zoh_conv_at(PT,PT,k);end
shape5=0;pow=zeros(1,N+1);pow(1)=1;
binom=1;
for j=1:N
    pow=conv(pow,bDelta);pow=pow(1:N+1);
    binom=binom*(0.5-(j-1))/j;
    shape5=shape5+abs(binom)*as^(1-2*j)*pow(N+1);
end
M=factorial(N)*[PL(N+1)+L*PT(N+1), ...
    PL(N+1)+G.cableCenterFromLoad*L*PT(N+1)+shape5, PL(N+1)];
end

function s=zoh_conv_at(a,b,k)
s=0;for j=0:k,s=s+a(j+1)*b(k-j+1);end
end

function m=zoh_poly_min(c,dt)
% Exact polynomial stationary-point minimization in real arithmetic.
% Floating-point roots are not a formal validated interval computation.
if any(~isfinite(c)),m=-inf;return;end
v=fliplr((1:numel(c)-1).*c(2:end));
r=roots(v);
r=real(r(abs(imag(r))<1e-9));
ts=[0;dt;r(r>0 & r<dt)];
m=min(polyval(fliplr(c),ts));
end

function info=zoh_critical(info,planes,i)
z=planes(i);info.criticalH0=info.currentH(i);
info.criticalHdot0=info.currentHdot(i);
if strcmp(z.kind,'obstacle')
    info.criticalObstacleCenter=z.center;
    info.criticalObstacleAxes=z.axes;
    info.criticalBodyCenterDistance=norm(z.currentPoint-z.center);
else
    info.criticalObstacleCenter=[NaN;NaN;NaN];
    info.criticalObstacleAxes=[NaN;NaN;NaN];
    info.criticalBodyCenterDistance=NaN;
end
end

function [center,Q,axes,label]=zoh_obstacle_geometry(part,index)
if ~isfield(part,'center'),error('CBFReal:Obstacle','Missing center.');end
center=part.center(:);
if numel(center)~=3 || any(~isfinite(center))
    error('CBFReal:Obstacle','Invalid center.');
end
kind='ellipsoid';if isfield(part,'kind'),kind=char(part.kind);end
switch kind
    case 'ellipsoid'
        if isfield(part,'params'),axes=part.params.axes(:);
        else,axes=part.axes(:);end
        R=part.R;
    case 'sphere'
        if isfield(part,'params'),r=part.params.radius;
        else,r=part.radius;end
        axes=repmat(r,3,1);R=eye(3);
    otherwise
        error('CBFReal:Obstacle','Unsupported kind: %s',kind);
end
if numel(axes)~=3 || any(~isfinite(axes)) || any(axes<=0) || ...
        ~isequal(size(R),[3 3]) || any(~isfinite(R(:))) || ...
        norm(R.'*R-eye(3),'fro')>1e-5
    error('CBFReal:Obstacle','Invalid ellipsoid parameters.');
end
Q=R*diag(axes.^2)*R.';
if isfield(part,'obstacleName'),label=char(part.obstacleName);
elseif isfield(part,'name'),label=char(part.name);
else,label=sprintf('obstacle%d',index);end
end

function G=zoh_geometry(P,observed)
G=CBF_COMMON_GEOMETRY(P,struct());
if ~isfield(observed,'ground')
    error('CBFReal:Ground','Sensor ground data missing.');
end
if isempty(observed.ground),G.includeGround=false;return;end
ground=observed.ground;
if ~strcmp(char(ground.kind),'halfspace') || ...
        norm(ground.normal(:)-[0;0;1])>1e-9 || ...
        (isfield(ground,'forbiddenSide') && ...
         ~strcmp(char(ground.forbiddenSide),'negative'))
    error('CBFReal:Ground','Only upward ground halfspace supported.');
end
margin=0;
if isfield(ground,'safetyMargin'),margin=ground.safetyMargin;end
if ~isfinite(margin) || margin<0
    error('CBFReal:Ground','Invalid margin.');
end
G.groundZ=ground.point(3)+margin;
G.includeGround=true;
end

function zoh_send_failure_to_view(sensorTime,info,args)
% Plot already-recorded LOGGER history plus this failed CBF step BEFORE
% HLC raises its existing error. This is observational only.
% If the optional plotting function is not installed, do not change control.
if numel(args)<3,return;end
if exist('CBF_REAL_DIAG_VIEW','file')~=2,return;end
try
    CBF_REAL_DIAG_VIEW('failure',args{3},sensorTime,info);
catch ME
    warning('CBFReal:HistoryPlotFailed', ...
        'CBF history plotting failed (control still stops): %s',ME.message);
end
end

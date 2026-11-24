function [u,info]=CBF_REAL_SCHEME_03_D2T_OBSTACLE_ZOH(agent,cfg,x,uNom,Praw,dt,varargin)
% ID3 ONLY: discrete thrust second difference and XY torque in ONE QP.
% For the actual sample: T_k=2*T_{k-1}-T_{k-2}+Delta2T_N;
% u_k=[T_k;tauX_k;tauY_k;tauZ_nom] is held by the ORIGINAL Sim ZOH.
% The 4-stage discrete nested CBF plus one ZOH interval clearance
% guard share the SAME 3 decision vars. All four lambdas are used.
% No drone/load/ground/tilt constraints or preview guidance.
% Conditional exact-sampled-map invariance; numerical flow and geometry
% bound are not rigorously validated here.
start=tic;u=[];
validateattributes(x,{'numeric'},{'vector','numel',19,'finite','real'});
validateattributes(uNom,{'numeric'},{'vector','numel',4,'finite','real'});
validateattributes(Praw,{'numeric'},{'vector','numel',9,'finite','real'});
validateattributes(dt,{'numeric'},{'scalar','positive','finite'});
if cfg.cbfId~=3 || abs(cfg.dt-dt)>1e-12
    error('CBFD2T:Configuration','Incorrect ID3 or sampling time');
end
x=double(x(:));raw=double(uNom(:));p=double(Praw(:));
P=struct('m',p(1),'J',p(2:4),'gravity',p(5),'mL',p(6), ...
    'cableL',p(7),'dstx',p(8),'dsty',p(9));
if P.m<=0 || P.mL<0 || P.cableL<=0 || any(P.J<=0)
    error('CBFD2T:PhysicalParameters','Non-physical parameter');
end
lo=[0;-1;-1;-1];hi=[20;1;1;1];nom=min(max(raw,lo),hi);
if ~isobject(agent) || ~isprop(agent,'sensor') || ...
        ~isfield(agent.sensor.result,'cbfObservedWorld')
    error('CBFD2T:Sensor','Sensor obstacle snapshot missing');
end
obs=agent.sensor.result.cbfObservedWorld;
if ~isstruct(obs) || ~isfield(obs,'parts') || ~isstruct(obs.parts)
    error('CBFD2T:Obstacles','Observed parts missing');
end
parts=obs.parts;
old=[nom(1);nom(1)];
if cfg.d2tHistory.initialized
    old=[cfg.d2tHistory.Tkm1;cfg.d2tHistory.Tkm2];
end
Tbase=2*old(1)-old(2);
% The decision is the DISCRETE virtual input delta2T, not the physical T.
% delta2T and T have the same units [N] and identical deviations.
% Actual QP metric is normalized by physical actuator half-ranges,
% NOT the unnormalized identity metric used in the previous patch.
yNom=[nom(1)-Tbase;nom(2:3)];
lb=[lo(1)-Tbase;lo(2:3)];ub=[hi(1)-Tbase;hi(2:3)];
a=max(P.cableL/2+cfg.d2tCableAxialMarginM,cfg.d2tCableRadiusM);
geom=struct('a',a,'b',cfg.d2tCableRadiusM,'centerFromLoad',0.5);
info=struct('cbfId',3,'success',false,'input',[],'candidateInput',[], ...
    'status','NOT_EVALUATED','nominal',raw,'physicalNominal',nom, ...
    'nominalFeasible',false,'modified',false,'maxViolation',NaN, ...
    'elapsedMs',NaN,'solverUsed',false,'solverIterations',0, ...
    'solverExitflag',NaN,'generatedCalled',false,'generatedCallCount',0, ...
    'generatedEvaluator','CBF_REAL_D2T_SAMPLED_EVAL', ...
    'jointQpColumns',{{'Delta2T_N','tauX','tauY'}}, ...
    'controlWeights',[cfg.d2tThrustWeight;cfg.d2tTorqueWeights(:)], ...
    'controlScales',[cfg.d2tThrustScaleN;cfg.d2tTorqueScaleNm(:)], ...
    'actuatorWeightCoordinates','[Delta2T_N/sThrustN,tauX/sTauXNm,tauY/sTauYNm]', ...
    'stageValues',[],'nominalStageValues',[],'currentH',[], ...
    'discreteCurrentStages',[],'discreteNextStages',[], ...
    'discretePartKeys',{{}},'barrierHistoryBootstrap',false, ...
    'stageConvention','phi_i=dt^i*psi_i [m]', ...
    'discreteStageValues',[],'barrierResiduals',[], ...
    'nominalBarrierResiduals',[],'intervalLowerBounds',[], ...
    'generatedControlSensitivity',[],'controlSensitivityRank',NaN, ...
    'constraintNames',{{}},'observedParts',numel(parts), ...
    'historyBeforeApply',old,'historyColdStart',~cfg.d2tHistory.initialized, ...
    'discreteSecondDifference',NaN,'delta2ThrustN',NaN, ...
    'fixedYawTorque',nom(4),'controlledInputIndices',[1;2;3], ...
    'groundActive',false,'bodyTiltEnabled',false,'cableTiltEnabled',false, ...
    'massUsedKg',P.mL,'massEstimatorKg',NaN, ...
    'sampledDataObstacleCertified',false,'recursiveFeasibilityCertified',false, ...
    'proofScope',['Exact sampled four-stage barrier gives conditional ', ...
    'invariance when initialized nonnegative and always feasible; ', ...
    'numerical RK4 error, intersample safety, mass uncertainty, ', ...
    'sensor changes and recursive feasibility not certified.']);
try
    ms=agent.estimator.result.state;
    if isprop(ms,'mL') && isfinite(double(ms.mL))
        info.massEstimatorKg=double(ms.mL);
    end
catch
end
if any(~isfinite(old)) || any(old<0) || any(old>20)
    info.status='INVALID_THRUST_HISTORY';u=nom;return
end
if isfield(obs,'dynamicIndices') && ~isempty(obs.dynamicIndices)
    info.status='MOVING_OBSTACLES_UNSUPPORTED';u=nom;return
end
if numel(parts)>cfg.d2tMaxParts
    info.status='TOO_MANY_OBSTACLES';u=nom;return
end
if isfield(agent.sensor.result,'cbfSensorTime') && ~isempty(varargin) && ...
        isobject(varargin{1}) && isprop(varargin{1},'t') && ...
        isfinite(double(agent.sensor.result.cbfSensorTime)) && ...
        abs(double(varargin{1}.t)-double(agent.sensor.result.cbfSensorTime))>1.5*dt
    info.status='STALE_OBSTACLES';u=nom;return
end
% Same support-envelope test as the original XY cable representation.
% Reject a fitted ellipsoid that fails to contain the physical cable tube.
if a<geom.b || cfg.d2tCableAxialMarginM<0 || ...
        d2t_capsule_margin(P.cableL/2,a,geom.b) < ...
        cfg.d2tPhysicalCableRadiusM-1e-10
    u=nom;info.status='CABLE_ELLIPSOID_DOES_NOT_ENCLOSE_CABLE';
    info.elapsedMs=1000*toc(start);return
end
if isempty(parts)
    info.discreteCurrentStages=zeros(4,0);info.discretePartKeys={};
    u=nom;info.success=true;info.nominalFeasible=true;
    info.status='NO_OBSTACLES';info.maxViolation=0;
    info.delta2ThrustN=yNom(1);info.discreteSecondDifference=yNom(1)/dt^2;
    info.elapsedMs=1000*toc(start);return
end
if abs(norm(x(1:4))-1)>cfg.d2tStateProjectionLimit || ...
        abs(norm(x(14:16))-1)>cfg.d2tStateProjectionLimit
    u=nom;info.status='STATE_MANIFOLD_OUT_OF_RANGE';return
end
x(1:4)=x(1:4)/norm(x(1:4));
x(14:16)=x(14:16)/norm(x(14:16));
lambda=cfg.d2tLambda(:);steps=cfg.d2tRkSteps;
scale=info.controlScales(:);
assert(all(isfinite(scale)) && all(scale>0),'CBFD2T:WeightScale');
% Paper-style input normalization: H = S^{-T} diag(w) S^{-1}.
% T and Delta2T_N have identical *differences*, unlike Tddot [N/s^2].
W=diag(info.controlWeights(:)./(scale.^2));
% Historical stages are saved from the previous MEASURED state by HLC.
% Do NOT substitute predicted x(k+1), and do not assume future ZOH inputs.
keys=d2t_part_keys(parts,geom,dt);
previous=[];
if isfield(cfg,'d2tBarrierHistory') && ...
        isfield(cfg.d2tBarrierHistory,'initialized') && ...
        cfg.d2tBarrierHistory.initialized && ...
        isequal(cfg.d2tBarrierHistory.keys,keys) && ...
        isequal(size(cfg.d2tBarrierHistory.previousStages),[4,numel(parts)])
    previous=cfg.d2tBarrierHistory.previousStages;
end
info.discretePartKeys=keys;
compute=@(y) CBF_REAL_D2T_SAMPLED_EVAL(x,old(1),old(2), ...
    y,nom(4),P,parts,geom,dt,steps,lambda,cfg.d2tNumericalReserveM,previous);
try
    evalNom=compute(yNom);info.generatedCalled=true;
    info.generatedCallCount=1;
catch ME
    u=nom;info.status='GENERATED_DISCRETE_EVAL_ERROR';
    info.errorIdentifier=ME.identifier;info.errorMessage=ME.message;
    info.elapsedMs=1000*toc(start);return
end
info.currentH=evalNom.h0;
info.constraintNames=[strcat(evalNom.names,'/phi4_next'); ...
    strcat(evalNom.names,'/ZOH_hold_lower')];
info.discreteCurrentStages=evalNom.currentStages;
info.discreteNextStages=evalNom.nextStages;
info.discreteStageValues=[evalNom.currentStages;evalNom.nextStages];
info.barrierHistoryBootstrap=evalNom.bootstrapUsed;
info.nominalBarrierResiduals=evalNom.residual;
currentSafe=all(evalNom.currentStages(:)>=-1e-9);
info.nominalFeasible=currentSafe && all(evalNom.residual>=-1e-9);
if info.nominalFeasible
    selected=evalNom;y=yNom;info.status='NOMINAL_SAMPLED_D2T_PASS';
else
    % Hard QP, three variables Delta2T_N, tauX and tauY simultaneously.
    % No future constant-input assumption. A local linearized QP can fail.
    y=yNom;selected=evalNom;info.status='JOINT_QP_NOT_SOLVED';
    if exist('quadprog','file')~=2 && exist('quadprog','file')~=6
        info.status='QUADPROG_UNAVAILABLE';
    else
        opts=optimoptions('quadprog','Display','off');
        info.solverUsed=true;
        for iter=1:cfg.d2tMaxIterations
            r=selected.residual;
            if all(r>=-1e-9),break;end
            J=zeros(numel(r),3);
            for j=1:3
                hs=cfg.d2tGradientSteps(j);yp=y;ym=y;
                yp(j)=min(ub(j),y(j)+hs);
                ym(j)=max(lb(j),y(j)-hs);
                if yp(j)-ym(j)<1e-12
                    info.status='GRADIENT_BOX_DEGENERATE';break
                end
                try
                    ep=compute(yp);em=compute(ym);
                    info.generatedCallCount=info.generatedCallCount+2;
                    J(:,j)=(ep.residual-em.residual)/(yp(j)-ym(j));
                catch ME
                    info.status='GRADIENT_EVAL_ERROR';
                    info.errorIdentifier=ME.identifier;
                    info.errorMessage=ME.message;
                    break
                end
            end
            if ~strcmp(info.status,'JOINT_QP_NOT_SOLVED'),break;end
            info.generatedControlSensitivity=J;
            info.controlSensitivityRank=rank(J);
            sc=max([ones(size(r)),abs(r),max(abs(J),[],2)],[],2);
            A=-bsxfun(@rdivide,J,sc);
            rhs=(r-J*y)./sc;
            try
                [yn,~,exitflag]=quadprog(2*W,-2*W*yNom,A,rhs, ...
                    [],[],lb,ub,[],opts);
            catch ME
                info.status='QUADPROG_ERROR';info.errorMessage=ME.message;break
            end
            info.solverIterations=iter;info.solverExitflag=exitflag;
            if exitflag<=0 || isempty(yn) || any(~isfinite(yn))
                info.status='LINEARIZED_QP_INFEASIBLE';break
            end
            if norm(yn-y,inf)<1e-10
                info.status='LINEARIZED_QP_STAGNATED';break
            end
            y=yn(:);
            try
                selected=compute(y);info.generatedCallCount=info.generatedCallCount+1;
            catch ME
                info.status='CANDIDATE_EVAL_ERROR';info.errorMessage=ME.message;break
            end
        end
        if all(selected.residual>=-1e-9)
            info.status='JOINT_SAMPLED_D2T_CANDIDATE';
        elseif strcmp(info.status,'JOINT_QP_NOT_SOLVED')
            info.status='JOINT_NONLINEAR_INFEASIBLE';
        end
    end
end
% Select an actual checked candidate; an infeasible iterate is never
% labelled safe, and the research failure policy handles continuation.
if all(selected.residual>=-1e-9)
    u=selected.physicalInput;
    info.success=currentSafe;
    if ~currentSafe,info.status='RECOVERY_FROM_OUTSIDE_NESTED_SAFE_SET';end
else
    u=nom;selected=evalNom; % Do not claim safety for a failed QP.
end
info.input=u;info.candidateInput=u;
info.delta2ThrustN=u(1)-Tbase;
info.discreteSecondDifference=info.delta2ThrustN/dt^2;
info.barrierResiduals=selected.residual;
info.intervalLowerBounds=selected.intervalLower;
info.discreteNextStages=selected.nextStages;
info.maxViolation=max([0;-selected.residual(:)]);
info.modified=norm(u-raw,inf)>1e-10;
info.elapsedMs=1000*toc(start);
end

function margin=d2t_capsule_margin(halfL,a,b)
% Equivalent to the original XY analytical support-envelope check.
c=a*a-b*b;t=[0,1];
if c>halfL*halfL
    sq=halfL*halfL*b*b/(c*(c-halfL*halfL));
    if sq>0 && sq<1,t(end+1)=sqrt(sq);end
end
margin=min(sqrt(b*b+c*t.^2)-halfL*t);
end

function keys=d2t_part_keys(parts,geom,dt)
% Exact sensor-part identity for historical barrier stage association.
% Reordered/moved/replaced obstacles require new history bootstrap.
keys=cell(numel(parts),1);
for j=1:numel(parts)
    p=parts(j);
    if isfield(p,'obstacleName'),name=char(p.obstacleName);
    elseif isfield(p,'name'),name=char(p.name);
    else,name=sprintf('part%d',j);end
    key=[name '|' char(p.kind) '|' sprintf('%.15g,',double(p.center(:)))];
    if strcmp(char(p.kind),'sphere')
        if isfield(p,'params') && ~isempty(p.params),val=p.params.radius;
        else,val=p.radius;end
    else
        if isfield(p,'params') && ~isempty(p.params),val=p.params.axes(:);
        else,val=p.axes(:);end
        if isfield(p,'R'),val=[double(val(:));double(p.R(:))];end
    end
    key=[key '|' sprintf('%.15g,',double(val(:))) '|' ...
        sprintf('%.15g,',[geom.a;geom.b;dt])];
    keys{j}=key;
end
end

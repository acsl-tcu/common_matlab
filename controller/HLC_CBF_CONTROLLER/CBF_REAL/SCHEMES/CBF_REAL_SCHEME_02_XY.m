function [u,info] = CBF_REAL_SCHEME_02_XY(agent,cfg,x,uNom,Praw,dt,varargin)
% Scheme 02 (research): direct 3-D rotated-ellipsoid support separation.
% Geometry: only the attitude-dependent FULL CABLE ELLIPSOID.
% Input optimization: tauX/tauY only, no dynamic extension.
% The HLC thrust and yaw torque are PHYSICALLY CLIPPED BEFORE derivatives.
% Continuous-time order-4 HOCBF is a sampled implementation, NOT an
% intersample/recursive guarantee. Failed certification is never hidden.
% On uncertified outcomes, a bounded candidate is returned alongside
% info.success=false; a generic HLC failure policy may apply it in Sim.
start=tic;u=[];
validateattributes(x,{'numeric'},{'vector','numel',19,'finite','real'});
validateattributes(uNom,{'numeric'},{'vector','numel',4,'finite','real'});
validateattributes(Praw,{'numeric'},{'vector','numel',9,'finite','real'});
validateattributes(dt,{'numeric'},{'scalar','positive','finite'});
if ~isfield(cfg,'cbfId') || cfg.cbfId~=2
    error('CBFXY:SchemeId','Scheme requires cbfId=2');
end
x=double(x(:));raw=double(uNom(:));p=double(Praw(:));
P=struct('m',p(1),'J',p(2:4),'gravity',p(5),'mL',p(6), ...
    'cableL',p(7),'dstx',p(8),'dsty',p(9));
if P.m<=0 || P.mL<0 || P.cableL<=0 || any(P.J<=0)
    error('CBFXY:Model','Invalid physical parameters');
end
if ~isobject(agent) || ~isprop(agent,'sensor') || ...
        ~isfield(agent.sensor.result,'cbfObservedWorld')
    error('CBFXY:Sensor','Spherical sensor observation missing');
end
observed=agent.sensor.result.cbfObservedWorld;
if ~isstruct(observed) || ~isfield(observed,'parts') || ...
        ~isstruct(observed.parts)
    error('CBFXY:Observation','Observed obstacle parts missing');
end
sensorTime=NaN;
if isfield(agent.sensor.result,'cbfSensorTime')
    sensorTime=double(agent.sensor.result.cbfSensorTime);
end
phase='';if numel(varargin)>=2,phase=char(varargin{2});end
lambda=xy_setting(cfg,'xyLambda',[4;4;4;4]);
weights=xy_setting(cfg,'xyTorqueWeights',[1;1]);
validateattributes(lambda,{'double'},{'vector','numel',4,'positive','finite'});
validateattributes(weights,{'double'},{'vector','numel',2,'positive','finite'});
% Independent hard attitude limits. Angles are TILT relative to gravity,
% NOT yaw or separate Euler roll/pitch limits.
bodyOn=xy_flag(cfg,'xyBodyTiltEnabled',true);
cableOn=xy_flag(cfg,'xyCableTiltEnabled',true);
bodyAngle=xy_setting(cfg,'xyBodyTiltMaxDeg',45);
cableAngle=xy_setting(cfg,'xyCableTiltMaxDeg',60);
bodyLambda=xy_setting(cfg,'xyBodyTiltLambda',[2;2]);
cableLambda=xy_setting(cfg,'xyCableTiltLambda',[1;1.2;1.4;1.6]);
validateattributes(bodyAngle,{'double'},{'scalar','finite','real','>',0,'<',90});
validateattributes(cableAngle,{'double'},{'scalar','finite','real','>',0,'<',90});
validateattributes(bodyLambda,{'double'},{'vector','numel',2,'positive','finite'});
validateattributes(cableLambda,{'double'},{'vector','numel',4,'positive','finite'});
tiltCfg=struct('bodyEnabled',bodyOn,'cableEnabled',cableOn, ...
    'bodyMaxDeg',bodyAngle,'cableMaxDeg',cableAngle, ...
    'bodyLambda',bodyLambda(:),'cableLambda',cableLambda(:));
cap=xy_setting(cfg,'xyMaxObstacles',Inf);
if numel(observed.parts)>cap
    error('CBFXY:Cap','Obstacle cap exceeded; no constraints dropped');
end
b=xy_setting(cfg,'xyCableRadiusM',0.2);
axialMargin=xy_setting(cfg,'xyCableAxialMarginM',0.02);
validateattributes(b,{'double'},{'scalar','positive','finite'});
validateattributes(axialMargin,{'double'},{'scalar','nonnegative','finite'});
G=CBF_COMMON_GEOMETRY(P,struct('includeGround',false, ...
    'trimCableWithBodySpheres',false));
% Independent cable ellipsoid: centre = middle of actual complete cable.
% Long full axis is L+2*axialMargin; transverse SEMI-axis is b.
G.cableCenterFromLoad=0.5;
G.cableAxisShort=b;
G.cableAxisLong=max(P.cableL/2+axialMargin,b);
% The single ellipsoid must contain the ACTUAL finite cable capsule.
% Shape support along any unit direction with axial cosine t is
% sqrt(b^2+(a^2-b^2)t^2); capsule support is (L/2)*t+rPhysical.
% The analytical minimum over t in [0,1] must be >=0.
if xy_capsule_support_margin(P.cableL/2,G.cableAxisLong,b) ...
        < G.cableRadius-1e-10
    error('CBFXY:CableNotEnclosed', ...
        'Cable ellipsoid axes do not enclose the physical cable capsule');
end
% PHYSICAL BOUNDS: never feed unbounded HLC T/yaw into derivative model.
lb=[0;-1;-1;-1];ub=[20;1;1;1];
uPhysical=min(max(raw,lb),ub);
nom=uPhysical(2:3);
u=uPhysical;
% Keep original input state for diagnostics. The HOCBF 19-state jet
% requires q and pT on their unit manifolds. Project SMALL integrator drift
% only inside this evaluator; never alter plant/estimator state.
qNorm=norm(x(1:4));pTNorm=norm(x(14:16));
stateNorms=[qNorm;pTNorm];stateNormErrors=abs(stateNorms-1);
stateUnitTolerance=1e-5;
stateUnitOK=all(stateNormErrors<=stateUnitTolerance);
maxStateProjectionError=xy_setting(cfg,'xyStateProjectionMaxError',0.005);
validateattributes(maxStateProjectionError,{'double'}, ...
    {'scalar','positive','finite'});
projectionPossible=all(stateNorms>0.2) && ...
    all(stateNormErrors<=maxStateProjectionError);
xModel=x;
if projectionPossible
    xModel(1:4)=x(1:4)/qNorm;
    xModel(14:16)=x(14:16)/pTNorm;
end
stateProjected=projectionPossible && any(stateNormErrors>1e-12);
nParts=numel(observed.parts);
info=struct('success',false,'status','NOT_EVALUATED', ...
    'cbfId',2,'input',u,'candidateInput',u, ...
    'nominal',raw,'physicalNominal',uPhysical, ...
    'fixedInputClipped',any(raw([1 4])~=uPhysical([1 4])), ...
    'nominalClippedMask',raw<lb | raw>ub, ...
    'inputDeltaFromRaw',u-raw, ...
    'inputDeltaFromClippedNominal',u-uPhysical, ...
    'appliedAtBoundMask',abs(u-lb)<=1e-9 | abs(u-ub)<=1e-9, ...
    'stateNorms',stateNorms,'stateNormErrors',stateNormErrors, ...
    'stateUnitTolerance',stateUnitTolerance,'stateUnitOK',stateUnitOK, ...
    'stateProjected',stateProjected,'stateProjectionLimit',maxStateProjectionError, ...
    'modified',any(abs(u-raw)>1e-10), ...
    'sensorTime',sensorTime,'phase',phase, ...
    'controlledInputIndices',[2;3],'fixedInputIndices',[1;4], ...
    'fixedThrust',uPhysical(1),'fixedYawTorque',uPhysical(4), ...
    'massUsedKg',P.mL,'massEstimatorKg',NaN, ...
    'inputLower',lb,'inputUpper',ub,'groundActive',false, ...
    'bodyConstraint','CABLE_OBSTACLE_PLUS_BODY_AND_CABLE_TILT','coordinateDimension',3, ...
    'obstacleProjection','NONE','cableAttitudeDependent',true, ...
    'cableEllipsoidAxes',[G.cableAxisShort;G.cableAxisShort;G.cableAxisLong], ...
    'geometryMethod','DIRECT_ROTATED_ELLIPSOID_SUPPORT_STATE_NORMAL', ...
    'observedParts',nParts,'constraintNames',{{}}, ...
    'constraintOrders',[],'tiltLimitNames',{{}}, ...
    'tiltBodyDegrees',NaN,'tiltCableDegrees',NaN, ...
    'tiltBodyMaxDeg',bodyAngle,'tiltCableMaxDeg',cableAngle, ...
    'tiltBodyEnabled',bodyOn,'tiltCableEnabled',cableOn, ...
    'tiltBodyStages',[],'tiltCableStages',[], ...
    'tiltBodyNominalStages',[],'tiltCableNominalStages',[], ...
    'tiltMinResidual',NaN, ...
    'constraintValues',[],'stageValues',[],'nominalStageValues',[], ...
    'stageLambdaPerSecond',lambda,'stageLabels',{{'h','psi1','psi2','psi3','psi4'}}, ...
    'currentH',[],'currentHdot',[],'minCurrentH',NaN, ...
    'criticalConstraint','','barrierResiduals',[],'nominalBarrierResiduals',[], ...
    'combinedMinimum',NaN,'nominalFeasible',false, ...
    'solverUsed',false,'solverExitflag',NaN,'solverIterations',0, ...
    'maxViolation',NaN,'numInequalities',nParts+double(bodyOn)+double(cableOn), ...
    'activeSetIndices',[], ...
    'torqueInfluenceMatrix',[],'psi4Bias',[], ...
    'stageCriticalOrder',NaN,'stageCriticalValue',NaN, ...
    'lowerStageTorqueError',NaN,'affinityError',NaN, ...
    'violations',struct('obstacle',{},'stage',{},'stageLabel',{},'value',{}), ...
    'stageViolationCount',0,'minByStage',[], ...
    'maxStageViolationByOrder',[],'usedRecovery',false, ...
    'researchContinuationCandidate',true,'elapsedMs',NaN, ...
    'previewEnabled',xy_flag(cfg,'xyPreviewEnabled',false), ...
    'previewStatus','NOT_REQUESTED','previewCandidates',0, ...
    'previewSelectedSide',0,'previewSelectedInput',[NaN;NaN], ...
    'previewCurrentReserve',NaN,'previewNextReserve',NaN, ...
    'previewBackupReserve',NaN,'previewBaseNextReserve',NaN, ...
    'previewOneStepFeasible',false,'previewBackupRolloutFeasible',false, ...
    'previewStrictGuarantee',false,'previewExtraTorque',[0;0], ...
    'previewElapsedMs',0,'hardQpElapsedMs',NaN, ...
    'previewGuideActivation',0,'previewGuideShiftNm',0, ...
    'previewBaselineLowerMargin',NaN,'previewChosenLowerMargin',NaN, ...
    'previewPredictedParts',0,'previewPreviousSource','NONE', ...
    'previewCurrentFeasible',false,'previewNextFeasible',false, ...
    'zohGuardEnabled',false,'zohGuardRows',0, ...
    'zohGuardStatus','NOT_EVALUATED','zohGuardMinNormalized',NaN, ...
    'proofScope',['Sampled continuous-time HOCBF: cable-obstacle separation ', ...
        '(order 4), body tilt (order 2), cable tilt (order 4). ', ...
        'Physical XY torques hard-bounded. Small state-norm drift locally ', ...
        'projected for evaluation. No intersample/recursive guarantee. ', ...
        'T/tauZ held fixed after clipping.']);
% Passive provenance of the mass actually used by the CBF (Praw(6)).
% Do not assume this equals the displayed EKF estimate.  No mass change.
try
    st=agent.estimator.result.state;
    if isprop(st,'mL')
        me=double(st.mL);
        if isscalar(me) && isfinite(me),info.massEstimatorKg=me;end
    end
catch
end
% A tiny unit norm drift is not a physical reason to turn off CBF.
% Large/nonfinite corruptions remain explicit uncertified failures.
if ~projectionPossible
    info.status='XY_STATE_PROJECTION_LIMIT_EXCEEDED';
    info.evaluationError=sprintf(['State projection limit %.3g exceeded: ', ...
        'norm(q)=%.12g, norm(pT)=%.12g'], ...
        maxStateProjectionError,qNorm,pTNorm);
    info.usedRecovery=true;
    info.maxViolation=NaN;
    info.elapsedMs=1000*toc(start);
    return
end
if nParts==0 && ~bodyOn && ~cableOn
    info.success=true;info.status='NO_ACTIVE_CONSTRAINTS';
    info.nominalFeasible=all(raw>=lb & raw<=ub);
    info.maxViolation=0;info.elapsedMs=1000*toc(start);return
end
% Evaluate zero, two basis torques and one mixed candidate. The four
% evaluations detect non-affine input effects rather than assume them.
zero=[uPhysical(1);0;0;uPhysical(4)];
try
    b0=CBF_REAL_XY_EVAL(xModel,zero,P,G,observed.parts,lambda,tiltCfg);
    bx=CBF_REAL_XY_EVAL(xModel,zero+[0;1;0;0],P,G,observed.parts,lambda,tiltCfg);
    by=CBF_REAL_XY_EVAL(xModel,zero+[0;0;1;0],P,G,observed.parts,lambda,tiltCfg);
    mixed=CBF_REAL_XY_EVAL(xModel,zero+[0;0.37;-0.41;0],P,G,observed.parts,lambda,tiltCfg);
catch ME
    % Research continuation cannot certify barrier when geometry/jet is
    % undefined. Preserve the physical clipped nominal and explicit status.
    info.status=['XY_EVALUATION_FAILED_' char(ME.identifier)];
    info.evaluationError=ME.message;info.usedRecovery=true;
    info.maxViolation=Inf;info.elapsedMs=1000*toc(start);return
end
% Each active CBF contributes one affine hard inequality in real XY
% torques, with its correct relative degree (2 for body tilt; 4 for cable).
base=b0.stages;
Bobs=[bx.stages(5,:)-base(5,:);by.stages(5,:)-base(5,:)].';
[tilt0,tiltNames,tiltOrders]=xy_tilt_final(b0,tiltCfg);
[tiltX,~,~]=xy_tilt_final(bx,tiltCfg);
[tiltY,~,~]=xy_tilt_final(by,tiltCfg);
[tiltMixed,~,~]=xy_tilt_final(mixed,tiltCfg);
Btilt=[tiltX-tilt0,tiltY-tilt0];
B=[Bobs;Btilt];
bias=[base(5,:).';tilt0];
names=[b0.names(:);tiltNames(:)];
orders=[4*ones(nParts,1);tiltOrders(:)];
lowObs=[bx.stages(1:4,:)-base(1:4,:), ...
     by.stages(1:4,:)-base(1:4,:), ...
     mixed.stages(1:4,:)-base(1:4,:)];
lowTilt=xy_tilt_low_errors(b0,bx,by,mixed,tiltCfg);
errLow=max([0;abs(lowObs(:));abs(lowTilt(:))]);
errAffine=max([0;abs(mixed.stages(5,:).'-base(5,:).'-Bobs*[0.37;-0.41]); ...
    abs(tiltMixed-tilt0-Btilt*[0.37;-0.41])]);
scale=max([1;abs(base(:));abs(bx.stages(:)); ...
    abs(by.stages(:));abs(mixed.stages(:)); ...
    abs(tilt0(:));abs(tiltX(:));abs(tiltY(:));abs(tiltMixed(:))]);
info.lowerStageTorqueError=errLow;
info.affinityError=errAffine;
info.constraintNames=names;
info.constraintOrders=orders;
info.torqueInfluenceMatrix=B;info.psi4Bias=bias;
info.currentH=base(1,:).';
info.currentHdot=base(2,:).'-lambda(1)*info.currentH;
if ~isempty(info.currentH),info.minCurrentH=min(info.currentH);end
info.tiltLimitNames=tiltNames;
info.tiltBodyDegrees=b0.tiltBodyDegrees;
info.tiltCableDegrees=b0.tiltCableDegrees;
info.tiltBodyStages=b0.tiltBodyStages;
info.tiltCableStages=b0.tiltCableStages;
if errLow>1e-8*scale || errAffine>1e-8*scale
    info.status='RELATIVE_DEGREE_OR_AFFINITY_CHECK_FAILED';
    info.usedRecovery=true;info.maxViolation=max(errLow,errAffine);
    info.elapsedMs=1000*toc(start);return
end
stageNom=[base(1:4,:);(base(5,:).'+Bobs*nom).'];
info.nominalStageValues=stageNom;
info.tiltBodyNominalStages=xy_stages_nominal(b0.tiltBodyStages, ...
    bx.tiltBodyStages,by.tiltBodyStages,nom);
info.tiltCableNominalStages=xy_stages_nominal(b0.tiltCableStages, ...
    bx.tiltCableStages,by.tiltCableStages,nom);
nominalFinal=bias+B*nom;
info.nominalBarrierResiduals=nominalFinal;
outside=any(base(1:4,:)<-1e-8,'all') || ...
    xy_any_negative_lower(b0.tiltBodyStages,1e-8) || ...
    xy_any_negative_lower(b0.tiltCableStages,1e-8);
info.nominalFeasible=all(raw>=lb & raw<=ub) && ...
    ~outside && all(nominalFinal>=-1e-8);
% Optional one-ZOH *MODEL-POLYNOMIAL* lower-stage guard.  It uses the
% SAME four current EVAL results and does not perform extra rollouts.
% This is NOT an interval proof: all higher Taylor remainders and model/
% mass-estimation error are unbounded in this implementation.
% The rows are normalised and appended only if nonredundant over the box.
Bguard=zeros(0,2);bguard=zeros(0,1);
useGuard=xy_flag(cfg,'xyZohGuardEnabled',true) && nParts>0;
info.zohGuardEnabled=useGuard;
if useGuard
    [Bguard,bguard]=xy_zoh_stage_rows(b0,bx,by,lambda, ...
        bodyLambda,cableLambda,bodyOn,cableOn,dt, ...
        xy_setting(cfg,'xyZohGuardMargin',0));
end
info.zohGuardRows=numel(bguard);
% Remove box-redundant inequalities: b - ||B||_1 >= 0 means satisfied
% for EVERY torque in [-1,1]^2.  This is an exact, safe reduction of the
% CURRENT affine constraints, independent of the number of obstacles.
[Bcore,bcore]=xy_xy_drop_box_redundant(B,bias);
[Bextra,bextra]=xy_xy_drop_box_redundant(Bguard,bguard);
Bfull=[Bcore;Bextra];bfull=[bcore;bextra];
% Solve the common current hard set + model ZOH lower-stage guard.
% If their intersection is empty, keep the original sampled hard-QP
% without falsely claiming a ZOH/recursive certificate.
[zg,~]=xy_xy_fast_qp(Bfull,bfull,nom,weights);
if isempty(zg)
    [z,active]=xy_xy_fast_qp(Bcore,bcore,nom,weights);
    info.zohGuardStatus='NO_COMMON_GUARD_INPUT_FALLBACK';
    Bopt=Bcore;bopt=bcore;
else
    z=zg;active=[];
    if useGuard
        info.zohGuardStatus='MODEL_ONE_ZOH_GUARD_FEASIBLE';
    else
        info.zohGuardStatus='NOT_ENABLED';
    end
    Bopt=Bfull;bopt=bfull;
end
isFeasible=~isempty(z);
% Compare nominal against the actual currently selected common constraints.
info.solverUsed=any(bias+B*nom<-1e-8) || ...
    any(bguard+Bguard*nom<-1e-8);
if ~isFeasible
    % No physically bounded torque satisfies ALL sampled hard conditions.
    z=xy_maximize_worst_margin(B,bias,nom,weights);
    active=[];info.usedRecovery=true;
    Bopt=Bcore;bopt=bcore;
    info.zohGuardStatus='CURRENT_HARD_QP_EMPTY';
end
info.previewCurrentFeasible=isFeasible;
info.hardQpElapsedMs=1000*toc(start);
previewTimer=tic;
% The progress objective is SOFT.  It cannot relax the current hard
% conditions or any accepted model-ZOH guard row.  Only ONE QP is solved
% to compute the guided torque, with no future CBF_EVAL or trajectory rollouts.
if info.previewEnabled
    if ~isFeasible
        info.previewStatus='SKIP_CURRENT_HARD_QP_EMPTY';
    elseif outside
        info.previewStatus='SKIP_OUTSIDE_HOCBF_SET';
    elseif nParts==0
        info.previewStatus='SKIP_NO_OBSTACLE';
    else
        try
            [zGuide,pv]=xy_single_qp_progress( ...
                xModel,uPhysical,P,G,lambda,nom,weights,z,dt,cfg, ...
                b0,agent,Bopt,bopt);
            info.previewStatus=pv.status;
            info.previewCandidates=pv.candidates;
            info.previewExtraTorque=zGuide-z;
            info.previewSelectedInput=zGuide;
            info.previewGuideActivation=pv.activation;
            info.previewGuideShiftNm=pv.shift;
            info.previewPreviousSource=pv.previousSource;
            z=zGuide;
            active=[];
        catch ME
            info.previewStatus=['GUIDANCE_ERROR_' char(ME.identifier)];
            info.previewError=ME.message;
        end
    end
end
info.previewElapsedMs=1000*toc(previewTimer);
if ~isempty(bguard)
    info.zohGuardMinNormalized=min(bguard+Bguard*z);
end
u=[uPhysical(1);z;uPhysical(4)];
finalPsi=bias+B*z;
info.stageValues=[base(1:4,:);(base(5,:).'+Bobs*z).'];
info.tiltBodyStages=xy_stages_final(b0.tiltBodyStages, ...
    bx.tiltBodyStages,by.tiltBodyStages,z);
info.tiltCableStages=xy_stages_final(b0.tiltCableStages, ...
    bx.tiltCableStages,by.tiltCableStages,z);
if ~isempty(tilt0),info.tiltMinResidual=min(finalPsi(nParts+1:end));end
info.constraintValues=finalPsi;info.barrierResiduals=finalPsi;
if ~isempty(base),
    info.minByStage=min(info.stageValues,[],2);
    info.maxStageViolationByOrder=max(0,-info.minByStage);
end
info.candidateInput=u;info.input=u;
info.inputDeltaFromRaw=u-raw;
info.inputDeltaFromClippedNominal=u-uPhysical;
info.appliedAtBoundMask=abs(u-lb)<=1e-9 | abs(u-ub)<=1e-9;
info.combinedMinimum=min(finalPsi);
info.maxViolation=max([0;-finalPsi;[-1;-1]-z;z-[1;1]]);
if info.previewEnabled && any(abs(info.previewExtraTorque)>1e-10)
    % The active hard constraints are those of the FINAL selected input.
    info.activeSetIndices=find(abs(finalPsi)<1e-7);
else
    info.activeSetIndices=active;
end
info.modified=any(abs(u-raw)>1e-10);
info.violations=xy_list_violations(info.stageValues,b0.names,1e-8);
info.violations=[info.violations, ...
    xy_attitude_violations(info.tiltBodyStages,info.tiltCableStages,1e-8)];
info.stageViolationCount=numel(info.violations);
if ~isempty(info.violations)
    v=[info.violations.value];[vmin,ix]=min(v);
    info.criticalConstraint=info.violations(ix).obstacle;
    info.stageCriticalOrder=info.violations(ix).stage;
    info.stageCriticalValue=vmin;
else
    [val,k]=min(finalPsi);info.criticalConstraint=names{k};
    info.stageCriticalOrder=orders(k);info.stageCriticalValue=val;
end
barrierBad=any(finalPsi<-1e-8);
info.success=isFeasible && ~outside && ~barrierBad;
if ~isFeasible
    info.status='XY_HARD_QP_INFEASIBLE_RECOVERY';info.usedRecovery=true;
elseif outside
    info.status='OUTSIDE_SET_RECOVERY';info.usedRecovery=true;
elseif any(abs(z-nom)>1e-10)
    info.status='XY_TORQUE_CORRECTED';
elseif any(raw~=uPhysical)
    info.status='XY_PHYSICAL_INPUT_CLIPPED';
else
    info.status='NOMINAL_XY_PASSED';
end
info.solverExitflag=double(isFeasible);
info.elapsedMs=1000*toc(start);
end


function [B,b]=xy_zoh_stage_rows(E0,Ex,Ey,lambda,bodyLambda, ...
                    cableLambda,bodyOn,cableOn,dt,margin)
% Polynomial prediction for ALL LOWER HOCBF stages at dt/2 and dt.
% Affine in XY torques because only the highest h derivative depends on
% XY torque.  No extra EVAL is performed.  Not a rigorous remainder bound.
validateattributes(margin,{'double'},{'scalar','finite','nonnegative'});
B=zeros(0,2);b=zeros(0,1);
for i=1:size(E0.stages,2)
    [B,b]=xy_append_model_stages(B,b,E0.stages(:,i), ...
        Ex.stages(:,i),Ey.stages(:,i),lambda,dt,margin);
end
if bodyOn
    [B,b]=xy_append_model_stages(B,b,E0.tiltBodyStages(:), ...
        Ex.tiltBodyStages(:),Ey.tiltBodyStages(:),bodyLambda,dt,margin);
end
if cableOn
    [B,b]=xy_append_model_stages(B,b,E0.tiltCableStages(:), ...
        Ex.tiltCableStages(:),Ey.tiltCableStages(:),cableLambda,dt,margin);
end
end

function [B,b]=xy_append_model_stages(B,b,s0,sx,sy,lambda,dt,margin)
% Recover h derivatives from recursive HOCBF stages.  The Taylor jet is
% taken from the current constant-input dynamics.  Reconstructing from
% HOCBF stages also supports future relative-degree-2 ground/body rows.
r=numel(lambda);
if numel(s0)~=r+1 || numel(sx)~=r+1 || numel(sy)~=r+1
    error('CBFXY:ZohStages','Inconsistent HOCBF stage dimensions');
end
h0=xy_unpack_hocbf(s0,lambda);
hx=xy_unpack_hocbf(sx,lambda);
hy=xy_unpack_hocbf(sy,lambda);
for fraction=[0.5 1]
    t=dt*fraction;
    for order=0:r-1
        v0=xy_project_stage(h0,lambda,order,t);
        vx=xy_project_stage(hx,lambda,order,t);
        vy=xy_project_stage(hy,lambda,order,t);
        row=[vx-v0,vy-v0];
        scale=max(1,abs(v0)+sum(abs(row)));
        % Remove strictly box-redundant rows here; no constraint discarded
        % unless it is satisfied for every physically bounded torque.
        v=v0-margin*scale;
        if v-sum(abs(row))>=-1e-12,continue;end
        B(end+1,:) = row/scale; %#ok<AGROW>
        b(end+1,1) = v/scale; %#ok<AGROW>
    end
end
end

function h=xy_unpack_hocbf(s,lambda)
r=numel(lambda);h=zeros(r+1,1);
for order=0:r
    p=1;
    for j=1:order,p=conv(p,[lambda(j),1]);end
    h(order+1)=s(order+1)-p(1:end-1)*h(1:order);
end
end

function val=xy_project_stage(h,lambda,order,t)
r=numel(lambda);d=zeros(1,order+1);
for j=0:order
    for q=j:r
        d(j+1)=d(j+1)+h(q+1)*t^(q-j)/factorial(q-j);
    end
end
p=1;
for j=1:order,p=conv(p,[lambda(j),1]);end
val=p*d.';
end

function [z,active]=xy_xy_fast_qp(B,b,nom,w)
% Most non-critical cycles require no QP vertex enumeration.
if all(B*nom(:)+b(:)>=-1e-8) && all(abs(nom(:))<=1+1e-10)
    z=nom(:);active=[];
else
    [z,active]=xy_solve_2d_qp(-B,b,nom,w,[-1;-1],[1;1]);
end
end

function [Br,br]=xy_xy_drop_box_redundant(B,b)
if isempty(B),Br=zeros(0,2);br=zeros(0,1);return;end
keep=b(:)-sum(abs(B),2)<-1e-12;
Br=B(keep,:);br=b(keep);
end

function [z,diag]=xy_single_qp_progress(x,uFix,P,G,lambda,nom,w, ...
                             baseline,dt,cfg,E0,agent,Bhard,bhard)
% Smooth tangential progress using ONE safety-constrained QP and a fixed
% number (3) of mass-consistent state JETs, independent of obstacle count.
% No multi-step rollout or 2-input max-min vertex enumeration.
z=baseline(:);
diag=struct('status','SKIP_GUIDANCE_INACTIVE', 'candidates',0, ...
            'activation',0,'shift',0,'previousSource','NONE');
onDist=xy_setting(cfg,'xyGuideStartH',3.5);
[direction,risk,approach]=xy_adaptive_direction(x,E0,lambda,agent,onDist);
diag.activation=risk;
if risk<1e-5 || norm(direction)<1e-8,return;end
G4=xy_preview_progress_matrix(x,uFix,P,G);
vel=x(11:13);vel(3)=0;
% Damp movement in the intended sidestep direction, not forward speed:
% forward speed must not suppress a genuinely necessary early manoeuvre.
latSpeed=max(0,direction(:).'*vel);
velDamping=xy_setting(cfg,'xyGuideVelocityDamping',0.45);
grad=G4.'*(direction-velDamping*latSpeed*direction/(1+latSpeed));
if any(~isfinite(grad)) || norm(grad)<1e-10
    diag.status='SKIP_GUIDANCE_DEGENERATE';return;
end
maxShift=xy_setting(cfg,'xyGuideMaxShiftNm',0.36);
validateattributes(maxShift,{'double'},{'scalar','finite','>=',0,'<=',1});
shift=maxShift*risk*(0.55+0.45*approach);
shift=shift/(1+xy_setting(cfg,'xyGuideVelocityScale',0.45)*latSpeed);
diag.shift=shift;
[prev,source]=xy_adaptive_previous_input(agent,baseline);
diag.previousSource=source;
smooth=xy_setting(cfg,'xyGuideSmoothWeight',0.16)*risk;
Wt=w(:)+smooth;
% Strictly convex quadratic objective.  Previous torque is a preference,
% not an additional hard rate limit that could destroy feasibility.
target=(w(:).*(nom(:)+shift*grad/norm(grad))+smooth*prev(:))./Wt;
[zopt,~]=xy_solve_2d_qp(-Bhard,bhard,target,Wt,[-1;-1],[1;1]);
if isempty(zopt)
    diag.status='GUIDANCE_QP_INFEASIBLE_BASE_RETAINED';return;
end
% Optional guide cap is applied by convex mixing: both endpoints satisfy
% the SAME hard halfspaces, hence the intermediate point is feasible.
maxExtra=xy_setting(cfg,'xyGuideMaxExtraNm',0.36);
diff=zopt(:)-baseline(:);
if maxExtra==0,zopt=baseline;
elseif norm(diff,inf)>maxExtra
    zopt=baseline+maxExtra/norm(diff,inf)*diff;
end
if any(Bhard*zopt+bhard<-1e-8) || any(abs(zopt)>1+1e-9)
    diag.status='GUIDANCE_HARD_CHECK_FAILED';return;
end
z=zopt(:);diag.candidates=1;
if norm(z-baseline,inf)<1e-8
    diag.status='GUIDANCE_EQUALS_BASELINE';
else
    diag.status='SINGLE_QP_GUIDANCE_ACCEPTED';
end
end

function [chosen,diag]=xy_preview_select(x,uFix,P,G,parts,lambda,tiltCfg, ...
        B,b,nom,w,baseline,dt,cfg,E0,agent,sensorTime)
% v2 ADAPTIVE GUIDANCE: single convex-QP candidate; ONE-ZOH preview of
% baseline AND guided candidate. All observed obstacle parts are evaluated.
% Explicit cost cap by maximum number of parts; when exceeded, ORIGINAL
% hard-QP result remains, with ALL obstacles still constrained.
% No claimed recursive feasibility, intersample proof or Backup CBF.
chosen=baseline;
diag=struct('status','NOT_RUN','candidates',0,'side',0, ...
    'currentReserve',NaN,'nextReserve',NaN,'backupReserve',NaN, ...
    'baseNextReserve',NaN,'oneStepFeasible',false, ...
    'backupRolloutFeasible',false,'activation',0,'guideShiftNm',0, ...
    'baseLowerMargin',NaN,'chosenLowerMargin',NaN, ...
    'predictedParts',numel(parts),'previousSource','NONE', ...
    'nextFeasible',false);
if isempty(parts),diag.status='SKIP_NO_OBSTACLE';return;end
maxParts=xy_setting(cfg,'xyPreviewMaxPredictedParts',4);
validateattributes(maxParts,{'double'},{'scalar','integer','>=',1,'<=',30});
if numel(parts)>maxParts
    diag.status='SKIP_PREDICTION_PART_BUDGET_HARD_QP_RETAINED';return;
end
onDist=xy_setting(cfg,'xyGuideStartH',3.0);
validateattributes(onDist,{'double'},{'scalar','positive','finite'});
% All parts contribute to the guide, not just the closest part. The
% hDot is recovered from psi1-lambda1*h (not from finite differences).
[direction,risk,approach]=xy_adaptive_direction(x,E0,lambda,agent,onDist);
diag.activation=risk;
if risk<1e-5 || norm(direction)<1e-8
    diag.status='SKIP_GUIDANCE_INACTIVE';return;
end
% The model-based torque sensitivity is a gradient in the optimizer,
% NOT a command to hold torque for a long prediction horizon.
G4=xy_preview_progress_matrix(x,uFix,P,G);
vel=x(11:13);vel(3)=0;
% Damp lateral overrun in velocity, relative to current desired heading.
grad=G4.'*(direction-xy_setting(cfg,'xyGuideVelocityDamping',0.35)* ...
    vel/(1+norm(vel)));
if any(~isfinite(grad)) || norm(grad)<1e-12
    diag.status='SKIP_TORQUE_PROGRESS_DEGENERATE';return;
end
% Adaptive input shift is always <= a configured MAXIMUM, never a fixed
% magnitude of 0.4 Nm added every step.
maxShift=xy_setting(cfg,'xyGuideMaxShiftNm',0.22);
validateattributes(maxShift,{'double'},{'scalar','finite','>=',0,'<=',1});
shift=maxShift*risk*(0.35+0.65*approach);
shift=shift/(1+xy_setting(cfg,'xyGuideVelocityScale',0.8)*norm(vel));
diag.guideShiftNm=shift;
[previous,source]=xy_adaptive_previous_input(agent,baseline);
diag.previousSource=source;
smooth=xy_setting(cfg,'xyGuideSmoothWeight',0.16);
validateattributes(smooth,{'double'},{'scalar','finite','nonnegative'});
Wprev=smooth*risk*ones(2,1);
Wtotal=w(:)+Wprev;
target=(w(:).*(nom(:)+shift*grad/norm(grad))+Wprev.*previous)./Wtotal;
% All current constraints remain hard, including ALL obstacle parts.
[guide,~]=xy_solve_2d_qp(-B,b,target,Wtotal,[-1;-1],[1;1]);
if isempty(guide),diag.status='GUIDANCE_CURRENT_HARD_QP_EMPTY';return;end
% Bound ONLY the optional deviation from the already-feasible hard QP;
% convex mixing with the baseline preserves all CURRENT affine constraints.
maxExtra=xy_setting(cfg,'xyGuideMaxExtraNm',0.22);
validateattributes(maxExtra,{'double'},{'scalar','finite','>=',0,'<=',2});
diff=guide(:)-baseline(:);
if norm(diff,inf)>maxExtra && maxExtra>0
    guide=baseline+maxExtra/norm(diff,inf)*diff;
elseif maxExtra==0
    guide=baseline;
end
if any(B*guide+b<-1e-8) || any(abs(guide)>1+1e-9)
    diag.status='GUIDANCE_HARD_QP_PROJECTION_FAILED';return;
end
if norm(guide-baseline,inf)<1e-7
    diag.status='GUIDANCE_EQUALS_BASELINE';return;
end
% Two evaluations ONLY: baseline vs proposed guidance; one ZOH step each.
% At the predicted endpoint, evaluate ALL constraints and lower stages.
Z=[baseline(:),guide(:)];
reserve=[NaN NaN];lower=[NaN NaN];good=[false false];
lowerStages=cell(1,2);
for k=1:2
    [x1,ok]=xy_preview_zoh_state(x,uFix,P,Z(:,k),dt);
    if ~ok,continue;end
    [F1,ok]=xy_preview_linearize(x1,uFix,P,G,parts,lambda,tiltCfg);
    if ~ok,continue;end
    % Search a COMMON XY torque that satisfies all future hard inequalities.
    % A positive normalized max-min margin indicates common feasibility.
    [~,m]=xy_preview_maxmin(F1.B,F1.b,Z(:,k),w);
    reserve(k)=m;
    lowerStages{k}=F1.lowerValues;
    if ~isempty(F1.lowerValues)
        lower(k)=min(F1.lowerValues./(1+abs(F1.lowerScale)));
    end
    good(k)=F1.lowerSafe && isfinite(m) && m>=-1e-8;
end
diag.candidates=2;
[~,diag.currentReserve]=xy_preview_maxmin(B,b,nom,w);
diag.baseNextReserve=reserve(1);
diag.baseLowerMargin=lower(1);
diag.oneStepFeasible=good(1);
if ~isfinite(reserve(2))
    diag.status='GUIDANCE_PREDICTION_FAILED_BASE_RETAINED';
    diag.nextReserve=reserve(1);diag.chosenLowerMargin=lower(1);
    diag.nextFeasible=good(1);return
end
if ~good(2)
    diag.status='GUIDANCE_NEXT_UNSAFE_BASE_RETAINED';
elseif ~good(1)
    % If baseline fails and guide succeeds, prefer the guide. This remains
    % an APPROXIMATE model forecast, not a recursive guarantee.
    chosen=guide;
    diag.status='GUIDANCE_PREDICTED_RECOVERY_CANDIDATE';
else
    tol=xy_setting(cfg,'xyGuideMaxReserveLoss',0.012);
    validateattributes(tol,{'double'},{'scalar','finite','nonnegative'});
    if reserve(2) < reserve(1)-tol
        diag.status='GUIDANCE_RESERVE_LOSS_BASE_RETAINED';
    else
        % Do not accept a significant degradation of ANY lower HOCBF stage
        % even when final-stage feasibility has a comfortable margin.
        baseEval=lowerStages{1};guideEval=lowerStages{2};
        drop=xy_setting(cfg,'xyGuideMaxLowerDrop',0.025);
        validateattributes(drop,{'double'}, ...
            {'scalar','finite','nonnegative'});
        if ~isempty(baseEval) && ~isempty(guideEval) && ...
                all(guideEval>=baseEval-drop*(1+abs(baseEval)))
            chosen=guide;
            diag.status='ADAPTIVE_GUIDE_ONE_STEP_ACCEPTED';
        else
            diag.status='GUIDANCE_LOWER_STAGE_LOSS_BASE_RETAINED';
        end
    end
end
if norm(chosen-guide,inf)<1e-8
    diag.nextReserve=reserve(2);diag.chosenLowerMargin=lower(2);
    diag.nextFeasible=good(2);
else
    diag.nextReserve=reserve(1);diag.chosenLowerMargin=lower(1);
    diag.nextFeasible=good(1);
end
end

function [direction,risk,approach]=xy_adaptive_direction(x,E0,lambda,agent,onDist)
% Smoothly blend all nearby obstacle tangents. No hard left/right latch.
N=size(E0.stages,2);
risk=0;approach=0;direction=zeros(3,1);
if N==0,return;end
goal=[];
try
    r=agent.reference.result.state;
    if isprop(r,'xd'),v=double(r.xd(:));else,v=double(r.get());end
    if numel(v)>=3 && all(isfinite(v(1:3))),goal=v(1:3);end
catch
end
if isempty(goal)
    goal=x(8:10)+x(11:13);
end
horizontal=goal-x(8:10);horizontal(3)=0;
if norm(horizontal)<1e-6
    horizontal=x(11:13);horizontal(3)=0;
end
if norm(horizontal)<1e-6,horizontal=[0;1;0];end
horizontal=horizontal/norm(horizontal);
weights=zeros(N,1);tangents=zeros(3,N);speeds=zeros(N,1);
for i=1:N
    h=E0.stages(1,i);
    hDot=E0.stages(2,i)-lambda(1)*h;
    n=E0.normal(:,i);
    if any(~isfinite(n)) || norm(n)<1e-8,continue;end
    n=n/norm(n);
    % Smoothstep: exactly zero at h >= onDist, grows toward the obstacle.
    a=max(0,min(1,(onDist-h)/onDist));
    a=a*a*(3-2*a);
    ap=max(0,min(1,-hDot/0.75));
    weights(i)=a*(0.4+0.6*ap);
    speeds(i)=ap;
    nh=[n(1);n(2);0];
    if norm(nh)>1e-8
        % True horizontal-plane tangent to the 3-D normal: n'*t=0.
        t=horizontal-nh*(nh.'*horizontal)/(nh.'*nh);
    else
        t=horizontal; % obstacle normal nearly vertical
    end
    if norm(t)<0.04
        % Deterministic geometric tie break. Side is NOT locked.
        t=[-n(2);n(1);0];
    end
    if norm(t)>1e-9,tangents(:,i)=t/norm(t);end
end
% Bounded convex average prevents gain from multiplying with part count.
risk=max(weights);
if sum(weights)<1e-12,return;end
approach=sum(weights.*speeds)/sum(weights);
direction=(0.25*horizontal+sum(bsxfun(@times,tangents,weights.'),2)/sum(weights));
if norm(direction)>1e-9,direction=direction/norm(direction);end
end

function [u,source]=xy_adaptive_previous_input(agent,fallback)
% Prefer the already-delivered previous physical output if available.
% Do NOT cache torque globally: simulations may contain multiple agents.
u=fallback(:);source='HARD_QP_FALLBACK';
try
    old=agent.controller.result.input;
    if isnumeric(old) && numel(old)==4 && all(isfinite(old(:))) && ...
            all(abs(old(2:3))<=1+1e-8)
        u=double(old(2:3));source='PREVIOUS_CONTROLLER_OUTPUT';
    end
catch
end
end

function [F,ok]=xy_preview_linearize(x,uFix,P,G,parts,lambda,tiltCfg)
% Analytic torque affine recalculation at a predicted NEXT state.
F=struct('B',[],'b',[],'lowerSafe',false, ...
    'lowerValues',[],'lowerScale',[]);ok=false;
u0=[uFix(1);0;0;uFix(4)];
try
    E0=CBF_REAL_XY_EVAL(x,u0,P,G,parts,lambda,tiltCfg);
    Ex=CBF_REAL_XY_EVAL(x,u0+[0;1;0;0],P,G,parts,lambda,tiltCfg);
    Ey=CBF_REAL_XY_EVAL(x,u0+[0;0;1;0],P,G,parts,lambda,tiltCfg);
    [f0,~,~]=xy_tilt_final(E0,tiltCfg);
    [fx,~,~]=xy_tilt_final(Ex,tiltCfg);
    [fy,~,~]=xy_tilt_final(Ey,tiltCfg);
    Bobs=[Ex.stages(5,:)-E0.stages(5,:); ...
          Ey.stages(5,:)-E0.stages(5,:)].';
    Btilt=[fx-f0,fy-f0];
    F.B=[Bobs;Btilt];
    F.b=[E0.stages(5,:).';f0];
    F.lowerSafe=all(E0.stages(1:4,:)>=-1e-8,'all') && ...
        ~xy_any_negative_lower(E0.tiltBodyStages,1e-8) && ...
        ~xy_any_negative_lower(E0.tiltCableStages,1e-8);
    F.lowerValues=E0.stages(1:4,:);F.lowerValues=F.lowerValues(:);
    if tiltCfg.bodyEnabled
        F.lowerValues=[F.lowerValues;E0.tiltBodyStages(1:end-1)];
    end
    if tiltCfg.cableEnabled
        F.lowerValues=[F.lowerValues;E0.tiltCableStages(1:end-1)];
    end
    F.lowerScale=F.lowerValues;
    ok=all(isfinite([F.B(:);F.b(:);F.lowerValues(:)]));
catch
    ok=false;
end
end

function [xf,ok]=xy_preview_zoh_state(x,uFix,P,z,dt)
% Input is constant ONLY over the actual present ZOH sample. Taylor4
% uses the existing 19-state jet. No arbitrary infinite hold assumption.
xf=x;ok=false;
try
    jf=CBF_REAL_XY_JET(x,[uFix(1);z(:);uFix(4)],P);
    xf=jf.stateTaylor*(dt.^(0:4)).';
    if ~all(isfinite(xf)),return;end
    qn=norm(xf(1:4));ln=norm(xf(14:16));
    if min(qn,ln)<0.5 || max(abs([qn,ln]-1))>0.01,return;end
    xf(1:4)=xf(1:4)/qn;
    xf(14:16)=xf(14:16)/ln;
    ok=true;
catch
    ok=false;
end
end

function [z,margin]=xy_preview_maxmin(B,b,nom,w)
% EXACT max-min normalized affine margin for a 2-input box, found from
% vertices of the 3-variable LP (z1,z2,m). No toolbox dependency.
% Valid for small numbers of constraints; caller caps total count.
z=[];margin=-inf;
n=size(B,1);
if n==0,z=min(max(nom(:),-1),1);margin=Inf;return;end
s=max(1,sum(abs(B),2));
C=[-bsxfun(@rdivide,B,s),ones(n,1); ...
    1 0 0;-1 0 0;0 1 0;0 -1 0];
d=[b(:)./s;1;1;1;1];
% The optimum is attained at a vertex. Include every triple intersection.
for i=1:size(C,1)-2
    for j=i+1:size(C,1)-1
        for k=j+1:size(C,1)
            rows=[i j k];A=C(rows,:);
            if abs(det(A))<1e-11*max(1,norm(A,'fro')^3),continue;end
            p=A\d(rows);
            if any(~isfinite(p)) || any(C*p>d+1e-8),continue;end
            better=p(3)>margin+1e-10;
            if ~better && abs(p(3)-margin)<1e-10 && ~isempty(z)
                better=sum(w(:).*(p(1:2)-nom(:)).^2)< ...
                    sum(w(:).*(z-nom(:)).^2);
            end
            if better,margin=p(3);z=p(1:2);end
        end
    end
end
if isempty(z)
    % Numerically degenerate LP: deliberately mark unknown.
    margin=-inf;
end
end

function G4=xy_preview_progress_matrix(x,uFix,P,G)
% Torque influence on actual cable-ELLIPSOID CENTRE 4th derivative.
u0=[uFix(1);0;0;uFix(4)];
J0=CBF_REAL_XY_JET(x,u0,P);
Jx=CBF_REAL_XY_JET(x,u0+[0;1;0;0],P);
Jy=CBF_REAL_XY_JET(x,u0+[0;0;1;0],P);
ix=[8:10];ip=[14:16];
c0=J0.stateTaylor(ix,5)-G.cableCenterFromLoad*P.cableL*J0.stateTaylor(ip,5);
cx=Jx.stateTaylor(ix,5)-G.cableCenterFromLoad*P.cableL*Jx.stateTaylor(ip,5);
cy=Jy.stateTaylor(ix,5)-G.cableCenterFromLoad*P.cableL*Jy.stateTaylor(ip,5);
G4=24*[cx-c0,cy-c0];
end

function flag=xy_flag(cfg,key,default)
if isfield(cfg,key),v=cfg.(key);else,v=default;end
validateattributes(v,{'numeric','logical'},{'scalar','finite'});
if ~(isequal(v,0) || isequal(v,1))
    error('CBFXY:Option','%s must be 0 or 1',key);
end
flag=logical(v);
end

function [f,names,orders]=xy_tilt_final(E,cfg)
f=zeros(0,1);names=cell(0,1);orders=zeros(0,1);
if cfg.bodyEnabled
    f(end+1,1)=E.tiltBodyStages(end);
    names{end+1,1}='bodyTilt';orders(end+1,1)=2;
end
if cfg.cableEnabled
    f(end+1,1)=E.tiltCableStages(end);
    names{end+1,1}='cableTilt';orders(end+1,1)=4;
end
end

function out=xy_tilt_low_errors(a,b,c,d,cfg)
out=zeros(0,1);
if cfg.bodyEnabled
    out=[out; b.tiltBodyStages(1:2)-a.tiltBodyStages(1:2); ...
        c.tiltBodyStages(1:2)-a.tiltBodyStages(1:2); ...
        d.tiltBodyStages(1:2)-a.tiltBodyStages(1:2)];
end
if cfg.cableEnabled
    out=[out; b.tiltCableStages(1:4)-a.tiltCableStages(1:4); ...
        c.tiltCableStages(1:4)-a.tiltCableStages(1:4); ...
        d.tiltCableStages(1:4)-a.tiltCableStages(1:4)];
end
end

function y=xy_stages_final(a,b,c,z)
y=a;
if ~isempty(a)
    y(end)=a(end)+(b(end)-a(end))*z(1)+(c(end)-a(end))*z(2);
end
end

function y=xy_stages_nominal(a,b,c,z)
y=xy_stages_final(a,b,c,z);
end

function bad=xy_any_negative_lower(s,tol)
bad=~isempty(s) && any(s(1:end-1)<-tol);
end

function violations=xy_attitude_violations(body,cable,tol)
violations=struct('obstacle',{},'stage',{},'stageLabel',{},'value',{});
V={body,cable};names={'bodyTilt','cableTilt'};
for k=1:2
    stage=V{k};
    for j=1:numel(stage)
        if stage(j)<-tol
            if j==1,label='h';else,label=sprintf('psi%d',j-1);end
            violations(end+1)=struct('obstacle',names{k}, ...
                'stage',j-1,'stageLabel',label,'value',stage(j)); %#ok<AGROW>
        end
    end
end
end

function m=xy_capsule_support_margin(halfL,a,b)
% Minimum ellipsoid support minus axial support of a centred line segment.
c=a*a-b*b;
t=[0 1];
if c>halfL*halfL
    tsquared=halfL*halfL*b*b/(c*(c-halfL*halfL));
    if tsquared>0 && tsquared<1,t(end+1)=sqrt(tsquared);end
end
m=min(sqrt(b*b+c*t.^2)-halfL*t);
end

function value=xy_setting(cfg,key,default)
if isfield(cfg,key),value=double(cfg.(key));else,value=default;end
end

function v=xy_list_violations(stages,names,tol)
v=struct('obstacle',{},'stage',{},'stageLabel',{},'value',{});
labels={'h','psi1','psi2','psi3','psi4'};
for j=1:size(stages,2)
    for k=1:5
        if stages(k,j)<-tol
            v(end+1)=struct('obstacle',names{j}, ...
                'stage',k-1,'stageLabel',labels{k}, ...
                'value',stages(k,j)); %#ok<AGROW>
        end
    end
end
end

function [z,active]=xy_solve_2d_qp(A,b,nom,w,lb,ub)
% Weighted projection onto a 2D halfspace polygon. Exact finite candidates
% (nominal, each metric projection, all pairwise active intersections).
C=[A;eye(2);-eye(2)];d=[b;ub;-lb];
candidates=nom(:);active=[];invW=1./w(:);
for i=1:size(C,1)
    ci=C(i,:).';den=sum(ci.*invW.*ci);
    if den>1e-15
        candidates(:,end+1)=nom(:)+invW.*ci* ...
            ((d(i)-ci.'*nom(:))/den); %#ok<AGROW>
    end
end
for i=1:size(C,1)
    for j=i+1:size(C,1)
        M=C([i j],:);
        if abs(det(M))>1e-12*max(1,norm(M,'fro')^2)
            candidates(:,end+1)=M\d([i j]); %#ok<AGROW>
        end
    end
end
valid=find(all(C*candidates<=d+1e-9,1));
if isempty(valid),z=[];return;end
scores=sum(w(:).*(candidates(:,valid)-nom(:)).^2,1);
[~,idx]=min(scores);z=candidates(:,valid(idx));
active=find(abs(C*z-d)<1e-7);
end

function z=xy_maximize_worst_margin(B,b,nom,w)
% Max-min psi4 linear functions on a two-dimensional box: check vertices
% of the polyhedral break-line arrangement, plus nominal/corners.
% This is only an UNCERTIFIED fallback when strict QP is infeasible.
z0=min(max(nom(:),-1),1);
points=[z0,[-1;-1],[-1;1],[1;-1],[1;1]];
if size(B,1)>8
    % Bounded-time best effort when many obstacles are simultaneously seen.
    % A globally optimal fallback is NOT claimed; this only runs after
    % strict QP infeasibility, and never creates a certified success.
    bestZ=z0;bestMargin=min(b(:)+B*z0);
    for seed=1:size(points,2)
        q=points(:,seed);
        for iter=1:40
            [score,worst]=min(b(:)+B*q);
            if score>bestMargin,bestMargin=score;bestZ=q;end
            grad=B(worst,:).';
            if norm(grad)<1e-12,break;end
            q=min(max(q+(0.2/sqrt(iter))*grad/max(norm(grad),1e-12),-1),1);
        end
    end
    z=bestZ;return
end
rows=zeros(0,3);
for i=1:size(B,1)
    for j=i+1:size(B,1)
        rows(end+1,:)=[B(i,:)-B(j,:),b(j)-b(i)]; %#ok<AGROW>
    end
end
% Include all torque box edges as lines.
rows=[rows;1 0 -1;1 0 1;0 1 -1;0 1 1];
for i=1:size(rows,1)
    for j=i+1:size(rows,1)
        M=rows([i j],1:2);
        if abs(det(M))>1e-12*max(1,norm(M,'fro')^2)
            q=M\rows([i j],3);
            if all(q>=-1-1e-9 & q<=1+1e-9)
                points(:,end+1)=min(max(q,-1),1); %#ok<AGROW>
            end
        end
    end
end
margins=min(b(:)+B*points,[],1);
best=max(margins);
winners=find(margins>=best-max(1e-9,1e-9*abs(best)));
scores=sum(w(:).*(points(:,winners)-nom(:)).^2,1);
[~,ix]=min(scores);z=points(:,winners(ix));
end

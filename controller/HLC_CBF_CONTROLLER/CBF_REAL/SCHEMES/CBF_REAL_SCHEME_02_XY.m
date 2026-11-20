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
% Diagnostics must inspect the ORIGINAL estimated state, not silently
% normalize quaternion or cable direction (which would change the plant).
qNorm=norm(x(1:4));
pTNorm=norm(x(14:16));
stateNorms=[qNorm;pTNorm];
stateNormErrors=abs(stateNorms-1);
stateUnitTolerance=1e-5;
stateUnitOK=all(stateNormErrors<=stateUnitTolerance);
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
    'modified',any(abs(u-raw)>1e-10), ...
    'sensorTime',sensorTime,'phase',phase, ...
    'controlledInputIndices',[2;3],'fixedInputIndices',[1;4], ...
    'fixedThrust',uPhysical(1),'fixedYawTorque',uPhysical(4), ...
    'inputLower',lb,'inputUpper',ub,'groundActive',false, ...
    'bodyConstraint','CABLE_ONLY','coordinateDimension',3, ...
    'obstacleProjection','NONE','cableAttitudeDependent',true, ...
    'cableEllipsoidAxes',[G.cableAxisShort;G.cableAxisShort;G.cableAxisLong], ...
    'geometryMethod','DIRECT_ROTATED_ELLIPSOID_SUPPORT_STATE_NORMAL', ...
    'observedParts',nParts,'constraintNames',{{}}, ...
    'constraintValues',[],'stageValues',[],'nominalStageValues',[], ...
    'stageLambdaPerSecond',lambda,'stageLabels',{{'h','psi1','psi2','psi3','psi4'}}, ...
    'currentH',[],'currentHdot',[],'minCurrentH',NaN, ...
    'criticalConstraint','','barrierResiduals',[],'nominalBarrierResiduals',[], ...
    'combinedMinimum',NaN,'nominalFeasible',false, ...
    'solverUsed',false,'solverExitflag',NaN,'solverIterations',0, ...
    'maxViolation',NaN,'numInequalities',nParts,'activeSetIndices',[], ...
    'torqueInfluenceMatrix',[],'psi4Bias',[], ...
    'stageCriticalOrder',NaN,'stageCriticalValue',NaN, ...
    'lowerStageTorqueError',NaN,'affinityError',NaN, ...
    'violations',struct('obstacle',{},'stage',{},'stageLabel',{},'value',{}), ...
    'stageViolationCount',0,'minByStage',[], ...
    'maxStageViolationByOrder',[],'usedRecovery',false, ...
    'researchContinuationCandidate',true,'elapsedMs',NaN, ...
    'proofScope',['Discrete implementation of continuous-time HOCBF; ', ...
        'T/tauZ are clipped but not optimized; no interval or recursive ', ...
        'feasibility guarantee. Only cable 3D ellipsoid considered.']);
% An invalid unit state is a STATE-INTEGRITY failure, not a barrier
% violation and not a QP infeasibility. Continue the research simulation
% only with a physically clipped nominal candidate, UNCERTIFIED. This
% avoids throwing an exception before generic HLC failurePolicy runs.
% Do NOT normalize the estimated state or claim a certified CBF correction.
if ~stateUnitOK
    info.status='XY_INVALID_STATE_UNIT_NORM';
    info.evaluationError=sprintf(['State norms violate tolerance %.3g: ', ...
        'norm(q)=%.12g (error=%+.4g), norm(pT)=%.12g (error=%+.4g)'], ...
        stateUnitTolerance,qNorm,qNorm-1,pTNorm,pTNorm-1);
    info.usedRecovery=true;
    info.maxViolation=NaN;       % psi_4 cannot be evaluated here
    info.stageValues=[];        % do not manufacture a zero residual
    info.nominalFeasible=false;
    info.elapsedMs=1000*toc(start);
    return
end
if nParts==0
    info.success=true;info.status='NO_OBSERVED_CABLE_CONSTRAINTS';
    info.nominalFeasible=all(raw>=lb & raw<=ub);
    info.maxViolation=0;info.elapsedMs=1000*toc(start);return
end
% Evaluate zero, two basis torques and one mixed candidate. The four
% evaluations detect non-affine input effects rather than assume them.
zero=[uPhysical(1);0;0;uPhysical(4)];
try
    b0=CBF_REAL_XY_EVAL(x,zero,P,G,observed.parts,lambda);
    bx=CBF_REAL_XY_EVAL(x,zero+[0;1;0;0],P,G,observed.parts,lambda);
    by=CBF_REAL_XY_EVAL(x,zero+[0;0;1;0],P,G,observed.parts,lambda);
    mixed=CBF_REAL_XY_EVAL(x,zero+[0;0.37;-0.41;0],P,G,observed.parts,lambda);
catch ME
    % Research continuation cannot certify barrier when geometry/jet is
    % undefined. Preserve the physical clipped nominal and explicit status.
    info.status=['XY_EVALUATION_FAILED_' char(ME.identifier)];
    info.evaluationError=ME.message;info.usedRecovery=true;
    info.maxViolation=Inf;info.elapsedMs=1000*toc(start);return
end
base=b0.stages;
B=[bx.stages(5,:)-base(5,:);by.stages(5,:)-base(5,:)].';
low=[bx.stages(1:4,:)-base(1:4,:), ...
     by.stages(1:4,:)-base(1:4,:), ...
     mixed.stages(1:4,:)-base(1:4,:)];
errLow=max(abs(low(:)));
errAffine=max(abs(mixed.stages(5,:).'-base(5,:).'-B*[0.37;-0.41]));
scale=max([1;abs(base(:));abs(bx.stages(:)); ...
    abs(by.stages(:));abs(mixed.stages(:))]);
info.lowerStageTorqueError=errLow;
info.affinityError=errAffine;
info.constraintNames=b0.names;
info.torqueInfluenceMatrix=B;info.psi4Bias=base(5,:).';
info.currentH=base(1,:).';
info.currentHdot=base(2,:).'-lambda(1)*info.currentH;
info.minCurrentH=min(info.currentH);
if errLow>1e-8*scale || errAffine>1e-8*scale
    info.status='RELATIVE_DEGREE_OR_AFFINITY_CHECK_FAILED';
    info.usedRecovery=true;info.maxViolation=max(errLow,errAffine);
    info.elapsedMs=1000*toc(start);return
end
stageNom=[base(1:4,:);(base(5,:).'+B*nom).'];
info.nominalStageValues=stageNom;
info.nominalBarrierResiduals=stageNom(5,:).';
info.nominalFeasible=all(raw>=lb & raw<=ub) && ...
    all(stageNom(:)>=-1e-8);
% Even if psi_0..psi_3 is negative, DO NOT stop trying to improve psi_4.
% A feasible psi4 is a recovery experiment, not a valid HOCBF certificate.
if all(stageNom(5,:)>=-1e-8)
    z=nom;active=[];isFeasible=true;
else
    [z,active]=xy_solve_2d_qp(-B,base(5,:).',nom,weights,[-1;-1],[1;1]);
    isFeasible=~isempty(z);
    info.solverUsed=true;
end
if ~isFeasible
    % No input satisfies all psi4 constraints. Choose the two bounded
    % torques that maximize the WORST psi4 margin (not a safety guarantee).
    z=xy_maximize_worst_margin(B,base(5,:).',nom,weights);
    active=[];info.usedRecovery=true;
end
u=[uPhysical(1);z;uPhysical(4)];
finalPsi=base(5,:).'+B*z;
info.stageValues=[base(1:4,:);finalPsi.'];
info.constraintValues=finalPsi;info.barrierResiduals=finalPsi;
info.minByStage=min(info.stageValues,[],2);
info.maxStageViolationByOrder=max(0,-info.minByStage);
info.candidateInput=u;info.input=u;
info.inputDeltaFromRaw=u-raw;
info.inputDeltaFromClippedNominal=u-uPhysical;
info.appliedAtBoundMask=abs(u-lb)<=1e-9 | abs(u-ub)<=1e-9;
info.combinedMinimum=min(finalPsi);
info.maxViolation=max([0;-finalPsi;[-1;-1]-z;z-[1;1]]);
info.activeSetIndices=active;
info.modified=any(abs(u-raw)>1e-10);
info.violations=xy_list_violations(info.stageValues,b0.names,1e-8);
info.stageViolationCount=numel(info.violations);
if ~isempty(info.violations)
    v=[info.violations.value];[vmin,ix]=min(v);
    info.criticalConstraint=info.violations(ix).obstacle;
    info.stageCriticalOrder=info.violations(ix).stage;
    info.stageCriticalValue=vmin;
else
    [val,k]=min(finalPsi);info.criticalConstraint=b0.names{k};
    info.stageCriticalOrder=4;info.stageCriticalValue=val;
end
outside=any(base(1:4,:)<-1e-8,'all');
barrierBad=any(finalPsi<-1e-8);
info.success=isFeasible && ~outside && ~barrierBad;
if ~isFeasible
    info.status='XY_INFEASIBLE_RECOVERY';info.usedRecovery=true;
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

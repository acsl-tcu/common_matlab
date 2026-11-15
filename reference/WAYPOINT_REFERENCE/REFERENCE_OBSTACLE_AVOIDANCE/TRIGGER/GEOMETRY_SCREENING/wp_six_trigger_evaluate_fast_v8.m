function result = wp_six_trigger_evaluate_fast_v8(known,cfg,forecast)
%WP_SIX_TRIGGER_EVALUATE_FAST_V8 Refined C sphere-chain AABBs; ONE mode.
% V8 refinement: exact selected C sphere-chain swept AABBs.
% Previous V4 result remains available and untouched for comparisons.

%
% One call = ONE system model (C/B/D) x ONE obstacle outer cover.
% Six-way comparisons MUST call this function six separate times in tests.
%
% Inputs:
%  known: wp_six_trigger_prepare(OBSERVED_WORLD,'multiple_spheres' etc.)
%  cfg:   wp_six_trigger_config(mode,cover,externalGeometry,modelSnapshot)
%  forecast: 4-second state forecast on the FIXED sample grid, with
%            guaranteed position errors and TRUE acceleration upper bounds
%            provided by an independent, physically justified predictor.
%
% Output status:
%   CLEAR_CONDITIONAL: no contact with the provided known outer covers,
%     GIVEN the forecast bounds, taut straight cable, and numeric assumptions.
%   POSSIBLE_CONTACT: a separating plane could not be established.
%   UNKNOWN: missing/inconsistent model/forecast/geometry assumptions.
%
% NO controller, EKF or trajectory planner is executed here. No fallback
% from invalid bounds to zero uncertainty. No constant-future-input model.
% A sphere's AABB equals its radius box. For B/D and small sphere
% workloads the original V4/V3 kernel is used without added C preclear.
% Preserve existing V3/V4 for B/D; apply the optional refined C sphere
% chain broadphase ONLY to C. Old C evaluator is kept as the fallback.
if cfg.systemMode~='C' || ...
        (strcmp(cfg.obstacleApproximation,'multiple_spheres') && ...
         known.numParts<3)
    result=wp_six_trigger_evaluate_fast_v4(known,cfg,forecast);
    result.v8Strategy='UNMODIFIED_V4_OR_V3_FOR_B_D';
    return
end
startTime=tic;
result=blankResult(cfg);
if ~isfield(known,'orientedAabbHalfWidth') || ...
        ~isfield(known,'geometryCacheFormatVersion') || ...
        known.geometryCacheFormatVersion~=4
    result.reason='rebuild_v4_observation_cache';
    result.elapsedMs=1000*toc(startTime);
    return
end
profile=isfield(cfg,'enableStageTiming') && logical(cfg.enableStageTiming);
if profile, stageWatch=tic; end
[valid,why,t]=validateForecast(forecast,cfg);
if profile, result.stageMs.validation=1000*toc(stageWatch); end
if ~valid
    result.reason=why;
    result.elapsedMs=1000*toc(startTime);
    return
end
if ~strcmp(known.approximation,cfg.obstacleApproximation) || ...
        known.dynamicUnsupported || ~known.groundSupported
    result.reason='geometry_cover_mismatch_or_unsupported_obstacles';
    result.elapsedMs=1000*toc(startTime);
    return
end
if known.groundEnabled && ~cfg.checkGround
    result.reason='known_ground_is_a_hard_constraint_and_cannot_be_ignored';
    result.elapsedMs=1000*toc(startTime);
    return
end

% Without observed obstacles, only the ground requires checking.
P=known.numParts;
groundActive=cfg.checkGround && known.groundEnabled;
if P==0 && ~groundActive
    result=markClear(result,numel(t)-1,'no_observed_obstacles');
    result.elapsedMs=1000*toc(startTime);
    return
end

% One conservative interval enclosure shared across the three body parts.
if profile, stageWatch=tic; end
[intervalRadius, maxInflation] = intervalInflations(forecast,cfg,t);
if profile, result.stageMs.inflation=1000*toc(stageWatch); end
allKnots=[forecast.pDrone,forecast.pLoad];
center=mean(allKnots,2);
delta=bsxfun(@minus,allKnots,center);
wholeBallRadius=sqrt(max(sum(delta.^2,1))) + maxInflation;

% Broad-phase over the entire 4 s. Every C/B/D body is inside this ball.
if profile, stageWatch=tic; end
if P>0
    d=bsxfun(@minus,known.center,center);
    safeBall=outsideSphere(d,known.outerRadius+wholeBallRadius);
    % Tightened BUT STILL OUTER broad phase: every swept body lies in
    % this expanded AABB. Separation from the obstacle enclosing sphere
    % proves clearance even when the 4-s whole-system sphere overlaps.
    low=min(allKnots,[],2)-maxInflation;
    high=max(allKnots,[],2)+maxInflation;
    safeBox=outsideAabb(known.center,known.outerRadius,low,high);
    tightBox=wp_six_trigger_oriented_box_clear(known,1:P,low,high);
    safeOutside=safeBall | safeBox | tightBox;
    result.tightAabbCleared=result.tightAabbCleared+nnz(tightBox & ~safeBall & ~safeBox);
    result.globalAabbCleared=sum(~safeBall & safeBox);
    activePart=find(~safeOutside);
    result.globalBroadphaseCleared=sum(safeOutside);
else
    activePart=[];
end

if profile, result.stageMs.globalBroadphase=1000*toc(stageWatch); end
% Vectorized body-specific AABB preclearance, once for the 4-s window.
% It only removes pairs for which ALL THREE selected-mode body covers
% are separated. Any unresolved pair follows the original V2 checker.
if profile, stageWatch=tic; end
[intervalPossible,preMeta]=wp_six_trigger_interval_broadphase_v4(known,cfg,forecast,activePart);
if ~isempty(activePart) && any(intervalPossible(:)) && numel(activePart)>=3
    [intervalPossible,chainMeta]=wp_six_trigger_C_chain_aabb_v8( ...
        known,cfg,forecast,activePart,intervalPossible);
else
    chainMeta=struct('initialUnresolved',nnz(intervalPossible), ...
        'extraCleared',0,'remaining',nnz(intervalPossible), ...
        'allSafeIntervals',nnz(~any(intervalPossible,2)));
end
result.v8ChainExtraCleared=chainMeta.extraCleared;
result.v8ChainUnresolved=chainMeta.remaining;
result.precomputedPairsCleared=preMeta.pairsCleared+chainMeta.extraCleared;
result.precomputedClearIntervals=chainMeta.allSafeIntervals;
if profile, result.stageMs.precomputedBroadphase=1000*toc(stageWatch); end
% Whole-horizon floor separation can avoid all interval loops.
groundNeedsIntervals=false;
if groundActive
    n=known.groundNormal;
    minZ=min(n' * bsxfun(@minus,allKnots,known.groundPoint));
    groundNeedsIntervals=~(minZ-maxInflation-known.groundMargin>0);
end
if ~groundNeedsIntervals && ~any(intervalPossible(:))
    result=markClear(result,numel(t)-1,'all_selected_C_spheres_precleared');
    result.elapsedMs=1000*toc(startTime);
    return
end

for k=1:numel(t)-1
    if profile, stageWatch=tic; end
    % Keep original order of inspected intervals and obstacle IDs.
    % New preclearance is a stronger sufficient CLEAR test, not a new hit test.
    activeNow=activePart(intervalPossible(k,:));
    if isempty(activeNow) && ~groundNeedsIntervals
        result.intervalsChecked=k;
        if profile, result.stageMs.intervalBroadphase=result.stageMs.intervalBroadphase+1000*toc(stageWatch); end
        continue
    end
    endpoints=[forecast.pDrone(:,k:k+1),forecast.pLoad(:,k:k+1)];
    localCenter=mean(endpoints,2);
    localDelta=bsxfun(@minus,endpoints,localCenter);
    localRadius=sqrt(max(sum(localDelta.^2,1))) + intervalRadius(k);
    if ~isempty(activeNow)
        d=bsxfun(@minus,known.center(:,activeNow),localCenter);
        outsideBall=outsideSphere(d,known.outerRadius(activeNow)+localRadius);
        low=min(endpoints,[],2)-intervalRadius(k);
        high=max(endpoints,[],2)+intervalRadius(k);
        outsideBox=outsideAabb(known.center(:,activeNow),known.outerRadius(activeNow),low,high);
        tightBox=wp_six_trigger_oriented_box_clear(known,activeNow,low,high);
        outside=outsideBall | outsideBox | tightBox;
        result.tightAabbCleared=result.tightAabbCleared+nnz(tightBox & ~outsideBall & ~outsideBox);
        candidates=activeNow(~outside);
        result.intervalAabbCleared=result.intervalAabbCleared+sum(~outsideBall & outsideBox);
        result.intervalBroadphaseCleared=result.intervalBroadphaseCleared+sum(outside);
    else
        candidates=[];
    end
    if profile, result.stageMs.intervalBroadphase=result.stageMs.intervalBroadphase+1000*toc(stageWatch); end
    if isempty(candidates) && ~groundNeedsIntervals
        result.intervalsChecked=k;
        continue
    end

    % Ground can be excluded for the entire interval using the same ball.
    if groundNeedsIntervals
        n=known.groundNormal;
        localGround=min(n'*bsxfun(@minus,endpoints,known.groundPoint)) ...
                    -intervalRadius(k)-known.groundMargin;
        checkBodiesGround=~(localGround>0);
    else
        checkBodiesGround=false;
    end
    if isempty(candidates) && ~checkBodiesGround
        result.intervalsChecked=k;
        continue
    end

    % Build just ONE mode's physical body covers on a suspicious interval.
    if profile, stageWatch=tic; end
    bodies=wp_six_trigger_make_sweep(cfg,forecast,k,cfg.systemMode);
    if profile, result.stageMs.sweepBuild=result.stageMs.sweepBuild+1000*toc(stageWatch); end
    if checkBodiesGround
        for b=1:numel(bodies)
            floorClear=min(known.groundNormal'*bsxfun(@minus,bodies(b).vertices,known.groundPoint)) ...
                       -bodies(b).radius-known.groundMargin;
            if ~(floorClear>0)
                result=markRisk(result,k,t,0,'ground',bodies(b).name,'ground_swept_possible');
                result.elapsedMs=1000*toc(startTime);
                return
            end
        end
    end

    if isempty(candidates)
        result.intervalsChecked=k;
        continue
    end

    % Drone / load are independent spheres including their EXTERNAL radii.
    for b=1:2
        [safe,result]=checkBody(bodies(b),known,candidates,result,profile);
        if ~safe
            result=markRisk(result,k,t,result.firstPartIndex,result.firstObstacle, ...
                            bodies(b).name,'body_swept_possible');
            result.elapsedMs=1000*toc(startTime);return
        end
    end

    % Cable of C/B/D: cheap containing capsule first. For C, if the
    % containing capsule is inconclusive, check its ORIGINAL sphere chain.
    cable=bodies(3);
    if cfg.systemMode=='C'
        V=[forecast.pDrone(:,k:k+1),forecast.pLoad(:,k:k+1)];
        maxCableRadius=max([bodies(3:end).radius]);
        cable=struct('name','cable_chain_outer','vertices',V, ...
                     'radius',maxCableRadius);
    end
    [coarseSafe,result,unresolved]=checkBody(cable,known,candidates,result,profile);
    if ~coarseSafe
        if cfg.systemMode=='C'
            % A broad enclosing capsule cannot prove collision. Resolve
            % each original cable sphere separately: never treat it as B.
            for b=3:numel(bodies)
                [safe,result]=checkBody(bodies(b),known,unresolved,result,profile);
                if ~safe
                    result=markRisk(result,k,t,result.firstPartIndex,result.firstObstacle, ...
                                    bodies(b).name,'cable_sphere_swept_possible');
                    result.elapsedMs=1000*toc(startTime);return
                end
            end
        else
            result=markRisk(result,k,t,result.firstPartIndex,result.firstObstacle, ...
                            cable.name,'cable_swept_possible');
            result.elapsedMs=1000*toc(startTime);return
        end
    end
    result.intervalsChecked=k;
end
result=markClear(result,numel(t)-1,'all_intervals_separated_given_external_bounds');
result.elapsedMs=1000*toc(startTime);
end

function [yes,result,unresolved]=checkBody(body,known,candidateIndices,result,profile)
% Cheap exact-sufficient outer-box exclusion BEFORE any support direction.
% Each body is conv(vertices)+ball(radius), contained in the AABB below.
V=body.vertices;
if profile,watch=tic;end
center=mean(V,2);
D=bsxfun(@minus,V,center);
rBall=sqrt(max(sum(D.^2,1)))+body.radius;
c=known.center(:,candidateIndices);
d=bsxfun(@minus,c,center);
farBall=outsideSphere(d,rBall+known.outerRadius(candidateIndices));
low=min(V,[],2)-body.radius;
high=max(V,[],2)+body.radius;
farBox=outsideAabb(c,known.outerRadius(candidateIndices),low,high);
tightBox=wp_six_trigger_oriented_box_clear(known,candidateIndices,low,high);
far=farBall | farBox | tightBox;
result.tightAabbCleared=result.tightAabbCleared+nnz(tightBox & ~farBall & ~farBox);
result.componentBroadphaseCleared=result.componentBroadphaseCleared+sum(far);
result.componentAabbCleared=result.componentAabbCleared+sum(farBox & ~farBall);
result.componentCandidatePairs=result.componentCandidatePairs+numel(candidateIndices);
if profile
    result.stageMs.componentBroadphase=result.stageMs.componentBroadphase+1000*toc(watch);
end
unresolved=candidateIndices(~far);
if ~isempty(unresolved)
    if profile,watch=tic;end
    [separated,normalCounts]=wp_six_trigger_separation_batch(body,known,unresolved);
    result.narrowphasePairs=result.narrowphasePairs+numel(unresolved);
    result.supportNormalTests=result.supportNormalTests+sum(normalCounts);
    unresolved=unresolved(~separated);
    if profile
        result.stageMs.supportSeparation=result.stageMs.supportSeparation+1000*toc(watch);
    end
end
yes=isempty(unresolved);
if ~yes
    result.firstPartIndex=unresolved(1);
    result.firstObstacle=known.id{unresolved(1)};
end
end

function safe=outsideAabb(centers,outerR,low,high)
% Lower bound on center-to-AABB distance, compared to containing sphere.
% This is a sufficient nonintersection certificate; touching is not clear.
if isempty(outerR),safe=false(1,0);return;end
gap=max(max(bsxfun(@minus,low,centers),bsxfun(@minus,centers,high)),0);
scale=max(1,max(abs([low;high]))+outerR);
r=outerR+128*eps(scale);
safe=sum(gap.^2,1)>r.^2;
end

function safe=outsideSphere(displacements,sumRadii)
% Strict inequality with a floating-point guard: near-tangent => not clear.
scale=max(1,sumRadii);
threshold=sumRadii+64*eps(scale);
safe=sum(displacements.^2,1)>threshold.^2;
end

function [radii,maxRadius]=intervalInflations(F,cfg,t)
N=numel(t)-1;
radii=zeros(1,N);
rD0=cfg.droneRadius+cfg.safetyMargin;
rL0=cfg.loadRadius+cfg.safetyMargin;
rC=cfg.cableRadius+cfg.safetyMargin;
knots=sqrt(sum((F.pLoad-F.pDrone).^2,1));
for k=1:N
    h=t(k+1)-t(k);
    eD=max(F.rhoDrone(k:k+1))+F.accDrone(k)*h*h/8;
    eL=max(F.rhoLoad(k:k+1))+F.accLoad(k)*h*h/8;
    lengthUpper=max(knots(k:k+1))+eD+eL;
    switch cfg.systemMode
        case 'C'
            cableExtra=hypot(lengthUpper/(2*cfg.cableSegments),rC);
        case 'B'
            cableExtra=rC;
        case 'D'
            b=sqrt(1+1/cfg.ellipsoidEta)*rC;
            half=lengthUpper/2;
            a=sqrt((1+cfg.ellipsoidEta)*half*half+b*b);
            cableExtra=max(b,a-half);
    end
    radii(k)=max([eD+rD0,eL+rL0,max(eD,eL)+cableExtra]);
end
maxRadius=max(radii);
end

function [ok,reason,t]=validateForecast(F,cfg)
ok=false;reason='';t=[];
required={'timeOffsets','pDrone','pLoad','rhoDrone','rhoLoad', ...
          'accDrone','accLoad','modelLoadMass'};
for i=1:numel(required)
    if ~isstruct(F) || ~isfield(F,required{i})
        reason=['missing_',required{i}];return
    end
end
t=reshape(F.timeOffsets,1,[]); N=numel(t);
if N<2 || any(~isfinite(t)) || abs(t(1))>1e-12 || ...
    abs(t(end)-4)>1e-12 || any(diff(t)<=0) || ...
    ~isequal(size(F.pDrone),[3,N]) || ~isequal(size(F.pLoad),[3,N])
    reason='invalid_4s_grid_or_positions';return
end
rD=reshape(F.rhoDrone,1,[]);rL=reshape(F.rhoLoad,1,[]);
aD=reshape(F.accDrone,1,[]);aL=reshape(F.accLoad,1,[]);
if numel(rD)~=N || numel(rL)~=N || numel(aD)~=N-1 || ...
    numel(aL)~=N-1 || any(~isfinite([F.pDrone(:);F.pLoad(:);rD(:);rL(:);aD(:);aL(:)])) || ...
    any([rD,rL,aD,aL]<0)
    reason='invalid_or_missing_motion_bounds';return
end
if ~isscalar(F.modelLoadMass) || ~isfinite(F.modelLoadMass) || ...
    abs(F.modelLoadMass-cfg.currentLoadMass)>64*eps(max(1,cfg.currentLoadMass))
    reason='stale_or_invalid_estimated_mass_snapshot';return
end
% In a taut model the true endpoint separation is cableLength. Bounds must
% allow this at every knot (otherwise the input model is inconsistent).
separations=sqrt(sum((F.pLoad-F.pDrone).^2,1));
if any(abs(separations-cfg.cableLength)>rD+rL+64*eps(max(1,cfg.cableLength)))
    reason='taut_cable_length_inconsistent_with_position_bounds';return
end
ok=true;
end

function r=blankResult(cfg)
r=struct('systemMode',cfg.systemMode, ...
    'obstacleApproximation',cfg.obstacleApproximation, ...
    'status','UNKNOWN','reason','', 'shouldRequestReplan',true, ...
    'firstInterval',[NaN,NaN], 'firstPartIndex',NaN, ...
    'firstObstacle','','firstBody','', 'intervalsChecked',0, ...
    'globalBroadphaseCleared',0,'intervalBroadphaseCleared',0, ...
    'componentBroadphaseCleared',0, ...
    'globalAabbCleared',0,'intervalAabbCleared',0, ...
    'componentAabbCleared',0,'componentCandidatePairs',0, ...
    'precomputedPairsCleared',0,'precomputedClearIntervals',0, ...
    'tightAabbCleared',0, ...
    'narrowphasePairs',0,'supportNormalTests',0,'elapsedMs',0, ...
    'geometryProvisional',cfg.isProvisional,'safeToCommit',false, ...
        'stageMs',struct('validation',0,'inflation',0,'globalBroadphase',0, ...
        'intervalBroadphase',0,'precomputedBroadphase',0,'sweepBuild',0,'componentBroadphase',0, ...
        'supportSeparation',0), ...
    'actualHlcPredictionProven',false,'physicalSafetyCertified',false);
end
function r=markClear(r,N,why)
r.status='CLEAR_CONDITIONAL';r.shouldRequestReplan=false;
r.reason=why;r.intervalsChecked=N;
r.firstPartIndex=NaN;r.firstObstacle='';r.firstBody='';
end
function r=markRisk(r,k,t,j,obstacle,body,why)
r.status='POSSIBLE_CONTACT';r.shouldRequestReplan=true;
r.reason=why;r.firstInterval=t(k:k+1);
r.firstPartIndex=j;r.firstObstacle=char(obstacle);r.firstBody=char(body);
r.intervalsChecked=k;
end

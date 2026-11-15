function result = wp_six_trigger_evaluate_baseline_20261112(known,cfg,forecast)
%WP_SIX_TRIGGER_EVALUATE Lightweight conditional collision gate, ONE mode.
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
startTime=tic;
result=blankResult(cfg);
[valid,why,t]=validateForecast(forecast,cfg);
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
[intervalRadius, maxInflation] = intervalInflations(forecast,cfg,t);
allKnots=[forecast.pDrone,forecast.pLoad];
center=mean(allKnots,2);
delta=bsxfun(@minus,allKnots,center);
wholeBallRadius=sqrt(max(sum(delta.^2,1))) + maxInflation;

% Broad-phase over the entire 4 s. Every C/B/D body is inside this ball.
if P>0
    d=bsxfun(@minus,known.center,center);
    safeOutside=outsideSphere(d,known.outerRadius+wholeBallRadius);
    activePart=find(~safeOutside);
    result.globalBroadphaseCleared=sum(safeOutside);
else
    activePart=[];
end

% Whole-horizon floor separation can avoid all interval loops.
groundNeedsIntervals=false;
if groundActive
    n=known.groundNormal;
    minZ=min(n' * bsxfun(@minus,allKnots,known.groundPoint));
    groundNeedsIntervals=~(minZ-maxInflation-known.groundMargin>0);
end
if isempty(activePart) && ~groundNeedsIntervals
    result=markClear(result,numel(t)-1,'whole_horizon_outer_ball_clear');
    result.elapsedMs=1000*toc(startTime);
    return
end

for k=1:numel(t)-1
    endpoints=[forecast.pDrone(:,k:k+1),forecast.pLoad(:,k:k+1)];
    localCenter=mean(endpoints,2);
    localDelta=bsxfun(@minus,endpoints,localCenter);
    localRadius=sqrt(max(sum(localDelta.^2,1))) + intervalRadius(k);
    if ~isempty(activePart)
        d=bsxfun(@minus,known.center(:,activePart),localCenter);
        outside=outsideSphere(d,known.outerRadius(activePart)+localRadius);
        candidates=activePart(~outside);
        result.intervalBroadphaseCleared=result.intervalBroadphaseCleared+sum(outside);
    else
        candidates=[];
    end
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
    bodies=wp_six_trigger_make_sweep(cfg,forecast,k,cfg.systemMode);
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
        [safe,result]=checkBody(bodies(b),known,candidates,result);
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
    [coarseSafe,result,unresolved]=checkBody(cable,known,candidates,result);
    if ~coarseSafe
        if cfg.systemMode=='C'
            % A broad enclosing capsule cannot prove collision. Resolve
            % each original cable sphere separately: never treat it as B.
            for b=3:numel(bodies)
                [safe,result]=checkBody(bodies(b),known,unresolved,result);
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

function [yes,result,unresolved]=checkBody(body,known,candidateIndices,result)
% Per-body spherical broad phase, followed by a certified sufficient
% separating-support direction (a failed direction proves NOTHING).
V=body.vertices;
center=mean(V,2);
D=bsxfun(@minus,V,center);
rBall=sqrt(max(sum(D.^2,1)))+body.radius;
d=bsxfun(@minus,known.center(:,candidateIndices),center);
far=outsideSphere(d,rBall+known.outerRadius(candidateIndices));
result.componentBroadphaseCleared=result.componentBroadphaseCleared+sum(far);
unresolved=[];
for j=candidateIndices(~far)
    [separate,count]=wp_six_trigger_separation(body,known,j);
    result.narrowphasePairs=result.narrowphasePairs+1;
    result.supportNormalTests=result.supportNormalTests+count;
    if ~separate
        unresolved(end+1)=j; %#ok<AGROW> % only near/unknown parts
    end
end
yes=isempty(unresolved);
if ~yes
    result.firstPartIndex=unresolved(1);
    result.firstObstacle=known.id{unresolved(1)};
end
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
    'componentBroadphaseCleared',0,'narrowphasePairs',0, ...
    'supportNormalTests',0,'elapsedMs',0, ...
    'geometryProvisional',cfg.isProvisional,'safeToCommit',false, ...
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

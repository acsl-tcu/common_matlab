function [possible,meta] = wp_six_trigger_interval_broadphase_v4(known,cfg,F,activePart)
%WP_SIX_TRIGGER_INTERVAL_BROADPHASE_V4 Original bounds plus tight ellipsoid AABB.
% One SELECTED C/B/D mode and ONE obstacle cover; never OR six modes.
%
% possible(k,j) = true  -> at least one body remains unresolved; must check.
% possible(k,j) = false -> ALL drone/load/cable swept outer AABBs are
%                         separated from obstacle j's OUTER SPHERE.
%
% Body AABBs include external drone/load/cable physical radii, model
% endpoint tubes, and A*dt^2/8 interpolation errors. The cable bound uses
% the selected C/B/D outer geometry (C sphere chain, B capsule, D ellipsoid).
% No assumptions about tracking, future input or cable swing limits are made.
% Only a *proven* separation skips work. Touching is never declared clear.
% This is numerical double arithmetic, not directed-rounded certification.
N=numel(F.timeOffsets)-1;
P=numel(activePart);
possible=true(N,P);
meta=struct('pairsConsidered',N*P,'pairsCleared',0,'allSafeIntervals',0);
if N==0 || P==0,meta.allSafeIntervals=N;return;end
h=diff(reshape(F.timeOffsets,1,[]));
rD=max(F.rhoDrone(1:end-1),F.rhoDrone(2:end)) + F.accDrone.*(h.^2)/8;
rL=max(F.rhoLoad(1:end-1),F.rhoLoad(2:end)) + F.accLoad.*(h.^2)/8;
Dlo=min(F.pDrone(:,1:end-1),F.pDrone(:,2:end));
Dhi=max(F.pDrone(:,1:end-1),F.pDrone(:,2:end));
Llo=min(F.pLoad(:,1:end-1),F.pLoad(:,2:end));
Lhi=max(F.pLoad(:,1:end-1),F.pLoad(:,2:end));
rPhysicalD=cfg.droneRadius+cfg.safetyMargin;
rPhysicalL=cfg.loadRadius+cfg.safetyMargin;
rPhysicalC=cfg.cableRadius+cfg.safetyMargin;
lengthKnots=sqrt(sum((F.pLoad-F.pDrone).^2,1));
lengthUpper=max(lengthKnots(1:end-1),lengthKnots(2:end))+rD+rL;
switch cfg.systemMode
    case 'C'
        cableRad=hypot(lengthUpper/(2*cfg.cableSegments),rPhysicalC)+max(rD,rL);
    case 'B'
        cableRad=rPhysicalC+max(rD,rL);
    case 'D'
        b=sqrt(1+1/cfg.ellipsoidEta)*rPhysicalC;
        half=lengthUpper/2;
        a=sqrt((1+cfg.ellipsoidEta)*half.^2+b^2);
        cableRad=max(b,a-half)+max(rD,rL);
    otherwise
        error('LightTrigger:Mode','Unknown selected system mode');
end

% Three body-specific boxes. A single global four-second sphere is NOT
% mistaken for a reachable load-position set.
bodyLo=cat(3,bsxfun(@minus,Dlo,rD+rPhysicalD), ...
             bsxfun(@minus,Llo,rL+rPhysicalL), ...
             bsxfun(@minus,min(Dlo,Llo),cableRad));
bodyHi=cat(3,bsxfun(@plus,Dhi,rD+rPhysicalD), ...
             bsxfun(@plus,Lhi,rL+rPhysicalL), ...
             bsxfun(@plus,max(Dhi,Lhi),cableRad));
cent=reshape(known.center(:,activePart),3,P,1);
outer=reshape(known.outerRadius(activePart),1,P,1);
allSeparated=false(N,P);
for bidx=1:3
    low=reshape(bodyLo(:,:,bidx),3,1,N);
    high=reshape(bodyHi(:,:,bidx),3,1,N);
    gap=max(max(bsxfun(@minus,low,cent),bsxfun(@minus,cent,high)),0);
    scale=max(1,outer);
    safe=squeeze(sum(gap.^2,1) > (outer+128*eps(scale)).^2);
    safe=reshape(safe,P,N)';
    % Exact axis-aligned outer box of each rotated obstacle ellipsoid.
    % This is an additional sufficient test; the older sphere/AABB
    % distance remains available for diagonally separated geometry.
    if ~isfield(known,'orientedAabbHalfWidth')
        error('LightTrigger:StaleCache','V4 requires the cached ellipsoid AABB');
    end
    obstacleLo=known.center(:,activePart)-known.orientedAabbHalfWidth(:,activePart);
    obstacleHi=known.center(:,activePart)+known.orientedAabbHalfWidth(:,activePart);
    boxSafe=false(N,P);
    for axis=1:3
        lowerBody=bodyLo(axis,:,bidx).';
        upperBody=bodyHi(axis,:,bidx).';
        loObs=obstacleLo(axis,:);
        hiObs=obstacleHi(axis,:);
        guard=128*eps(max(1,max(abs(loObs),abs(hiObs))));
        boxSafe=boxSafe | ...
            bsxfun(@gt,lowerBody,hiObs+guard) | ...
            bsxfun(@lt,upperBody,loObs-guard);
    end
    safe=safe | boxSafe;
    if bidx==1
        allSeparated=safe;
    else
        allSeparated=allSeparated & safe;
    end
end
possible=~allSeparated;
meta.pairsCleared=nnz(allSeparated);
meta.allSafeIntervals=sum(all(allSeparated,2));
end

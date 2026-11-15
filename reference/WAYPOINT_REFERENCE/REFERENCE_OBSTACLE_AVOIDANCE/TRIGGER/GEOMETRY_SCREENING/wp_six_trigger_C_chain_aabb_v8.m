function [possible,meta]=wp_six_trigger_C_chain_aabb_v8(known,cfg,F,activePart,coarsePossible)
%WP_SIX_TRIGGER_C_CHAIN_AABB_V8 Exact C sphere-chain OUTER AABB separation.
% For one time interval, every C cable sphere sweeps the segment joining
% its two endpoint centres, enlarged by the EXACT wp_six_trigger_make_sweep
% radius. The resulting AABB encloses every point of that swept sphere.
% An obstacle's orientedAabbHalfWidth is an outer AABB of its original
% sphere/ellipsoid, respecting its 3D rotation. Strict axis separation of
% these two AABBs is sufficient to conclude NONCONTACT for that body.
% A pair is precleared ONLY when ALL 2+(cableSegments+1) bodies separate.
% The old evaluator checks every pair not established as separated here.
% C remains a chain of spheres; it is NOT replaced by a B capsule.
% No model/error bounds are guessed. Input F already carries the external
% bounds for the selected use. Numeric double arithmetic, NOT directed
% rounding. Intended for stage-1 BROADPHASE, not as a safety certificate.
if cfg.systemMode~='C'
    error('LightTrigger:CMode','C chain preclear is only for mode C');
end
M=numel(F.timeOffsets)-1;P=numel(activePart);
if nargin<5 || isempty(coarsePossible)
    coarsePossible=true(M,P);
end
assert(isequal(size(coarsePossible),[M,P]));
possible=logical(coarsePossible);
meta=struct('initialUnresolved',nnz(possible),'extraCleared',0, ...
    'remaining',nnz(possible),'allSafeIntervals',0);
if M==0 || P==0
    meta.allSafeIntervals=M;return
end
if ~isfield(known,'orientedAabbHalfWidth')
    error('LightTrigger:StaleCache','Expected observed outer AABB cache');
end
% Optional compiled kernel: explicitly selected, and only if installed.
% A missing MEX never prevents normal MATLAB evaluation.
if isfield(cfg,'useMexCChainPreclear') && cfg.useMexCChainPreclear && ...
        exist('wp_six_trigger_C_chain_numeric_mex_v8','file')==3 && P<=128
    possible=wp_six_trigger_C_chain_numeric_mex_v8( ...
        F.pDrone,F.pLoad,F.rhoDrone,F.rhoLoad,F.accDrone,F.accLoad, ...
        reshape(F.timeOffsets,1,[]),uint32(cfg.cableSegments), ...
        cfg.droneRadius,cfg.loadRadius,cfg.cableRadius,cfg.safetyMargin, ...
        known.center(:,activePart),known.orientedAabbHalfWidth(:,activePart), ...
        logical(coarsePossible));
    meta.extraCleared=nnz(coarsePossible & ~possible);
    meta.remaining=nnz(possible);
    meta.allSafeIntervals=nnz(~any(possible,2));
    return
end
% We only need refine intervals that survived the old conservative gate.
rows=find(any(possible,2));
if isempty(rows)
    meta.allSafeIntervals=M;return
end
N=cfg.cableSegments;
if ~(isscalar(N)&&isfinite(N)&&N>=1&&N==round(N))
    error('LightTrigger:Segments','Invalid external C cableSegments');
end
h=diff(reshape(F.timeOffsets,1,[]));
rD=max(F.rhoDrone(1:end-1),F.rhoDrone(2:end)) + F.accDrone.*(h.^2)/8;
rL=max(F.rhoLoad(1:end-1),F.rhoLoad(2:end)) + F.accLoad.*(h.^2)/8;
D0=F.pDrone(:,1:end-1);D1=F.pDrone(:,2:end);
L0=F.pLoad(:,1:end-1);L1=F.pLoad(:,2:end);
lengthKnots=sqrt(sum((F.pLoad-F.pDrone).^2,1));
lengthUpper=max(lengthKnots(1:end-1),lengthKnots(2:end))+rD+rL;
chainRadius=hypot(lengthUpper/(2*N),cfg.cableRadius+cfg.safetyMargin);
obsC=known.center(:,activePart);
obsHalf=known.orientedAabbHalfWidth(:,activePart);
% If any body AABB overlaps the obstacle AABB, that body is unresolved.
% Start by requiring clearance of ALL bodies; vectorized 40xP arrays.
separatedAll=true(numel(rows),P);
for b=1:N+3
    if b==1
        V0=D0(:,rows); V1=D1(:,rows);
        radius=rD(rows)+cfg.droneRadius+cfg.safetyMargin;
    elseif b==2
        V0=L0(:,rows); V1=L1(:,rows);
        radius=rL(rows)+cfg.loadRadius+cfg.safetyMargin;
    else
        s=(b-3)/N;
        V0=(1-s)*D0(:,rows)+s*L0(:,rows);
        V1=(1-s)*D1(:,rows)+s*L1(:,rows);
        radius=chainRadius(rows)+(1-s)*rD(rows)+s*rL(rows);
    end
    % AABB of conv(V0,V1) (+) B(radius): mathematically containing.
    mid=(V0+V1)*.5;
    hw=abs(V1-V0)*.5+repmat(radius,3,1);
    sep=false(numel(rows),P);
    for axis=1:3
        left=mid(axis,:)';right=obsC(axis,:);
        fullWidth=bsxfun(@plus,hw(axis,:)',obsHalf(axis,:));
        % Near tangent remains unresolved; scale is dimensioned in metres.
        guard=128*eps(max(1,bsxfun(@plus,abs(left),abs(right)) +fullWidth));
        sep=sep | bsxfun(@gt,abs(bsxfun(@minus,left,right)),fullWidth+guard);
    end
    separatedAll=separatedAll & sep;
    if ~any(separatedAll(:))
        % No candidate pair has all bodies separated; further body tests
        % cannot turn this interval-wide conjunction true.
        break
    end
end
% Never overwrite a coarse-failed entry as clear except on sufficient
% ALL-bodies certificate. Inactive/coarse-cleared entries stay cleared.
was=possible(rows,:);
possible(rows,:)=was & ~separatedAll;
meta.extraCleared=nnz(was & separatedAll);
meta.remaining=nnz(possible);
meta.allSafeIntervals=nnz(~any(possible,2));
end

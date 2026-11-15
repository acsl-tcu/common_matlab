function [observed, report] = ra_sim_sensor_detect(worldNow, dronePositionTrue, cfg)
%RA_SIM_SENSOR_DETECT Current-snapshot obstacle selection only.
% [observed,report]=ra_sim_sensor_detect(worldNow,pDroneTrue,cfg)
% - Sensor sphere: center=SIMULATOR TRUE DRONE POSITION, radius=cfg.radiusM.
% - Any sphere/rotated ellipsoid part touching sphere => whole obstacle seen.
% - Every approximation part of that obstacle is returned UNMODIFIED,
%   including the parts outside the range; original object metadata is kept.
% - No previous observations/memory; objects outside range simply disappear.
% - worldNow must be the CURRENT dynamic snapshot if obstacles can move.
% - Ground is a separate environmental constraint and is passed unchanged.
% - Does NOT use the EKF estimate, forecast, or future simulator truth.
start=tic;
if nargin~=3 || ~isstruct(cfg) || numel(cfg)~=1 || ...
        ~isfield(cfg,'radiusM')
    error('RaSensor:Config','Use ra_sim_sensor_config().');
end
% Shapes are decided from original part.kind; world.approximation is validated provenance.
expected='';
if isfield(cfg,'expectedMethod') && ~isempty(cfg.expectedMethod)
    expected=char(cfg.expectedMethod);
elseif isfield(cfg,'approximation') && ~strcmp(char(cfg.approximation),'FROM_WORLD')
    expected=char(cfg.approximation); % old API: validation only
end
cfg=ra_sim_sensor_config(cfg.radiusM);
validateattributes(dronePositionTrue,{'numeric'}, ...
    {'vector','numel',3,'real','finite'},mfilename,'dronePositionTrue');
pD=double(dronePositionTrue(:));
if ~isstruct(worldNow) || numel(worldNow)~=1 || ~isfield(worldNow,'parts') ...
        || ~isstruct(worldNow.parts)
    error('RaSensor:World','worldNow.parts must be a struct array.');
end
cover=ra_obstacle_method_info(worldNow); % read-only authoritative method
if ~isempty(expected) && ~strcmp(expected,cover.method)
    error('RaSensor:CoverMismatch', ...
        'Sensor legacy expectedMethod differs from world.approximation');
end
parts=worldNow.parts; n=numel(parts);
% Detect at the APPROXIMATION PART level, then expand whole-object groups.
partHits=false(1,n);
nDistanceSolves=0;
% All original parts were independently checked to share the same kind.
kindExpected=cover.kind;
names=cell(1,n);
for j=1:n
    part=parts(j);
    if ~isfield(part,'kind') || ~strcmp(part.kind,kindExpected) || ...
            ~isfield(part,'center') || ~isfield(part,'params') || ...
            ~isfield(part,'R') || ~isfield(part,'obstacleName')
        error('RaSensor:InvalidPart', ...
            'Part %d lacks the selected approximation or required fields.',j);
    end
    names{j}=char(part.obstacleName);
    if isempty(names{j})
        error('RaSensor:MissingObjectId', ...
            'Part %d needs obstacleName to return the entire object.',j);
    end
    c=double(part.center(:));
    if numel(c)~=3 || any(~isfinite(c))
        error('RaSensor:InvalidPart','Part %d center must be finite 3D.',j);
    end
    if ~isequal(size(part.R),[3,3]) || any(~isfinite(part.R(:))) || ...
            norm(part.R'*part.R-eye(3),'fro')>1e-6 || det(part.R)<0.999999
        error('RaSensor:InvalidRotation','Part %d rotation must be SO(3).',j);
    end
    radialDistance=norm(c-pD);
    % Per-part branch: not single_/multiple_ world labels.
    if strcmp(char(part.kind),'sphere')
        if ~isfield(part.params,'radius')
            error('RaSensor:InvalidPart','Sphere %d lacks radius.',j);
        end
        a=double(part.params.radius);
        if ~isscalar(a) || ~isreal(a) || ~isfinite(a) || a<=0
            error('RaSensor:InvalidPart','Sphere %d has invalid radius.',j);
        end
        tol=64*eps(max([1,radialDistance,cfg.radiusM,a]));
        partHits(j)=(radialDistance<=cfg.radiusM+a+tol);
    else
        if ~isfield(part.params,'axes')
            error('RaSensor:InvalidPart','Ellipsoid %d lacks semiaxes.',j);
        end
        a=double(part.params.axes(:));
        if numel(a)~=3 || any(~isfinite(a)) || any(a<=0)
            error('RaSensor:InvalidPart','Ellipsoid %d has invalid semiaxes.',j);
        end
        tol=64*eps(max([1;radialDistance;cfg.radiusM;a])); %#ok<NBRAK>
        if radialDistance>cfg.radiusM+max(a)+tol
            partHits(j)=false; % exact outer-sphere rejection
        elseif radialDistance<=cfg.radiusM+min(a)+tol
            partHits(j)=true; % ellipsoid contains a sphere of radius min(a)
        else
            d=ra_sim_sensor_ellipsoid_point_distance(pD,c,part.R,a);
            nDistanceSolves=nDistanceSolves+1;
            partHits(j)=(d<=cfg.radiusM+tol);
        end
    end
end
% Select by ID and return ALL parts for each detected object.
seenNames=unique(names(partHits),'stable');
keepParts=ismember(names,seenNames);
% Only copy currently observed data; do not leak world.sourceObjects or
% world.baseParts (they include undetected/old-pose objects).
observed=struct('parts',parts(keepParts),'objects',struct([]), ...
    'meta',struct([]),'ground',[],'approximation',cover.method, ...
    'scene','','time',[], ...
    'staticIndices',[],'dynamicIndices',[], ...
    'sensorSource','SIMULATED_SPHERICAL_TRUE_DRONE_CENTER');
if isfield(worldNow,'scene'),observed.scene=worldNow.scene;end
if isfield(worldNow,'time'),observed.time=worldNow.time;end
if isfield(worldNow,'ground'),observed.ground=worldNow.ground;end
if isfield(worldNow,'objects') && ~isempty(worldNow.objects)
    obj=worldNow.objects;
    if ~isfield(obj,'name')
        error('RaSensor:InvalidWorldObjects','worldNow.objects needs names.');
    end
    objectNames={obj.name};
    if numel(unique(objectNames))~=numel(objectNames)
        error('RaSensor:DuplicateNames','Original obstacle names must be unique.');
    end
    if ~all(ismember(seenNames,objectNames))
        error('RaSensor:InconsistentWorld', ...
            'A detected part has no corresponding worldNow.objects entry.');
    end
    keepObjects=ismember(objectNames,seenNames);
    observed.objects=obj(keepObjects);
    if isfield(worldNow,'meta') && ~isempty(worldNow.meta)
        if numel(worldNow.meta)~=numel(obj)
            error('RaSensor:MetaMismatch','worldNow.meta length mismatch.');
        end
        observed.meta=worldNow.meta(keepObjects);
    end
    if isfield(worldNow,'partBudgets') && ...
            numel(worldNow.partBudgets)==numel(obj)
        observed.partBudgets=worldNow.partBudgets(keepObjects);
    end
    if isfield(worldNow,'desiredPartBudgets') && ...
            numel(worldNow.desiredPartBudgets)==numel(obj)
        observed.desiredPartBudgets=worldNow.desiredPartBudgets(keepObjects);
    end
    if isfield(observed.objects,'isDynamic')
        observed.dynamicIndices=find([observed.objects.isDynamic]);
        observed.staticIndices=find(~[observed.objects.isDynamic]);
    end
end
% For parts-only fixtures, the output .objects is empty by design.
% report counts relate to *current* snapshot only; no observation memory.
report=struct('status','CURRENT_SNAPSHOT_FILTERED', ...
    'approximation',cover.method,'radiusM',cfg.radiusM, ...
    'centerTrue',pD,'totalParts',n, ...
    'intersectingPartCount',nnz(partHits), ...
    'detectedObjectCount',numel(seenNames), ...
    'returnedPartCount',nnz(keepParts), ...
    'detectedObjectNames',{seenNames}, ...
    'exactEllipsoidDistancesSolved',nDistanceSolves, ...
    'currentOnly',true,'groundPassedSeparately',true, ...
    'singlePerObject',cover.singlePerObject,'shapeFromOriginalPartKind',true, ...
    'definitionReadOnly',true, ...
    'elapsedMs',1000*toc(start));
end

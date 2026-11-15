function I=ra_obstacle_method_info(world)
%RA_OBSTACLE_METHOD_INFO Original obstacle shape inferred from parts.kind.
% The original DRONE_OBSTACLES world.approximation is only a provenance tag:
% single/multiple never selects a different geometric formula in the sensor.
% No geometry is approximated, fitted, resized or regenerated here.
if ~isstruct(world) || ~isscalar(world) || ...
        ~isfield(world,'parts') || ~isstruct(world.parts)
    error('RA:ObstacleMethod','World must have its original struct parts.');
end
methods={'single_sphere','multiple_spheres','single_ellipsoid','multiple_ellipsoids'};
method='';
if isfield(world,'approximation') && ~isempty(world.approximation)
    method=char(world.approximation);
    if ~ismember(method,methods)
        error('RA:ObstacleMethod','Unsupported original approximation: %s',method);
    end
end
p=world.parts;
if ~isempty(p)
    if ~isfield(p,'kind') || ~isfield(p,'obstacleName')
        error('RA:ObstacleMethod','Each part needs kind and obstacleName.');
    end
    k=char(p(1).kind);
    if ~ismember(k,{'sphere','ellipsoid'})
        error('RA:ObstacleMethod','Unsupported part.kind: %s',k);
    end
    if ~all(strcmp({p.kind},k))
        error('RA:ObstacleMethod','This source must use one sphere/ellipsoid kind per scene.');
    end
    names={p.obstacleName};
    if any(cellfun(@isempty,names))
        error('RA:ObstacleMethod','Every part needs its original obstacleName.');
    end
    if isfield(p,'method') && ~isempty(method)
        partsMethod={p.method};
        if any(~cellfun(@isempty,partsMethod) & ~strcmp(partsMethod,method))
            error('RA:ObstacleMethod','Original method tag and parts.method disagree.');
        end
    end
else
    names={};k='';
end
if isempty(method)
    if isempty(k)
        error('RA:ObstacleMethod','Empty parts need world.approximation to identify shape kind.');
    end
    % Prefer the original per-part method metadata, if present and uniform.
    % Otherwise choose a compatibility method LABEL based on part.kind only.
    % Neither branch fits or alters geometry.
    if isfield(p,'method')
        pm={p.method};pm=pm(~cellfun(@isempty,pm));
        if ~isempty(pm)
            if numel(unique(pm))~=1 || ~ismember(pm{1},methods)
                error('RA:ObstacleMethod','Inconsistent original parts.method values.');
            end
            method=pm{1};
        end
    end
    if isempty(method)
        if strcmp(k,'sphere'),method='multiple_spheres';
        else,method='multiple_ellipsoids';end
    end
    inferred=true;
else
    inferred=false;
end
if isfield(p,'method') && ~isempty(p)
    pm={p.method};
    if any(~cellfun(@isempty,pm) & ~strcmp(pm,method))
        error('RA:ObstacleMethod','Original part.method and inferred/declared method disagree.');
    end
end
if any(strcmp(method,{'single_sphere','multiple_spheres'}))
    declaredKind='sphere';kindCode=1;
else
    declaredKind='ellipsoid';kindCode=2;
end
if isempty(k),k=declaredKind;end
if ~strcmp(k,declaredKind)
    error('RA:ObstacleMethod','Original world.approximation and parts.kind disagree.');
end
isSingle=startsWith(method,'single_');
if isfield(world,'objects') && ~isempty(world.objects)
    if ~isfield(world.objects,'name')
        error('RA:ObstacleMethod','Original objects require names.');
    end
    namesObj={world.objects.name};
    if numel(unique(namesObj))~=numel(namesObj) || ~all(ismember(names,namesObj))
        error('RA:ObstacleMethod','Duplicate or unmatched original object names.');
    end
    if isSingle
        for j=1:numel(namesObj)
            if nnz(strcmp(names,namesObj{j}))~=1
                error('RA:SinglePart','Original single_* metadata has a non-single object: %s',namesObj{j});
            end
        end
    end
elseif isSingle && numel(unique(names))~=numel(names)
    error('RA:SinglePart','Original single_* metadata has duplicate obstacleName.');
end
I=struct('method',method,'kind',k,'kindCode',kindCode, ...
    'singlePerObject',isSingle,'numParts',numel(p), ...
    'numObjects',numel(unique(names)),'readOnly',true, ...
    'kindFromParts',~isempty(p),'methodInferred',inferred);
end

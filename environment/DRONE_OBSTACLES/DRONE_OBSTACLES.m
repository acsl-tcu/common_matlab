function world=DRONE_OBSTACLES(sceneName,approxMethod,groundName,options)
% Scene + ONE consistent approximation method + independent ground setting.
% No obstacle-specific method switching: all generated parts have one kind.
% Geometry is precomputed here, never inside the HLC simulation loop.
if nargin<4,options=struct();end
if nargin<3 || isempty(groundName),groundName='ground_z0';end
if nargin<2 || isempty(approxMethod),approxMethod='multiple_ellipsoids';end
options=obs_approx_defaults(options);
sceneName=char(sceneName);approxMethod=char(approxMethod);
validMethods={'single_sphere','single_ellipsoid','multiple_spheres', ...
    'multiple_ellipsoids','superellipsoid','convex_polyhedra'};
assert(any(strcmp(approxMethod,validMethods)), ...
    'Unknown scene-wide approximation: %s',approxMethod);
if exist(sceneName,'file')~=2,error('Scene not on MATLAB path: %s',sceneName);end
objects=feval(sceneName);
assert(isstruct(objects),'Scene must return a struct array');
sourceObjects=objects;
for i=1:numel(objects)
    [xyz,R,rpy]=obs_pose_at_time(sourceObjects(i),0);
    objects(i).position=xyz;objects(i).R=R;objects(i).rpyDeg=rpy;
end
if numel(objects)>1
    assert(numel(unique({objects.name}))==numel(objects), ...
        'Obstacle names must be unique');
end
for j=1:numel(objects)
    assert(isvarname(objects(j).name), ...
        'Obstacle names must be valid MATLAB field names');
end
if strcmpi(char(groundName),'none')
    ground=[];
else
    if exist(char(groundName),'file')~=2,error('Ground not on MATLAB path');end
    ground=feval(char(groundName));
end

% Allocate a small per-object budget and a hard total budget.
% Assign 1 part first to every obstacle. Give extra parts ONLY to objects
% whose geometry is large or has multiple connected pieces.
N=numel(objects);
if N>options.maxTotalParts
    error('DRONE_OBSTACLES:BudgetTooSmall', ...
        '%d obstacles require >=%d total approximation parts.',N,N);
end
desired=ones(1,N);
if ~any(strcmp(approxMethod,{'single_sphere','single_ellipsoid'}))
    for i=1:N
        C=obs_components(objects(i),options);
        nC=numel(C);
        [lo,hi]=obs_local_bounds(C);
        ext=max(hi-lo,0);
        if strcmp(approxMethod,'convex_polyhedra') || ...
                strcmp(approxMethod,'superellipsoid')
            d=nC;
        elseif strcmp(approxMethod,'multiple_spheres') && ...
                nC==1 && strcmp(C(1).kind,'sphere')
            d=1;
        elseif strcmp(approxMethod,'multiple_ellipsoids') && ...
                nC==1 && any(strcmp(C(1).kind,{'sphere','ellipsoid'}))
            d=1;
        else
            d=max([nC,ceil(max(ext)/options.targetPartSize), ...
                ceil(prod(ext)/options.targetPartSize^3)]);
        end
        desired(i)=max(1,min(options.maxPartsPerObstacle,d));
    end
end
allocated=ones(1,N);
remaining=options.maxTotalParts-N;
while remaining>0 && any(allocated<desired)
    gap=desired-allocated;
    score=gap./sqrt(allocated);
    score(gap<=0)=-Inf;
    [~,idx]=max(score);
    allocated(idx)=allocated(idx)+1;
    remaining=remaining-1;
end
if any(allocated<desired)
    warning('DRONE_OBSTACLES:GlobalBudget', ...
        'Global part budget forced coarser outward covers for some obstacles.');
end
part0=obs_new_part('sphere',[0 0 0],eye(3),struct('radius',1));
parts=repmat(part0,0,1);
meta=struct('obstacleName',{},'method',{},'numComponents',{}, ...
    'numParts',{},'guarantee',{},'note',{});
for i=1:N
    localOptions=options;
    localOptions.maxPartsPerObstacle=allocated(i);
    [q,detail]=obs_approximate(objects(i),approxMethod,localOptions);
    assert(numel(q)<=allocated(i), ...
        'Approximation exceeded per-object part budget');
    if ~isempty(q)
        % Important for downstream CBF: NEVER mix shape types in one scene.
        expectedKind=method_kind(approxMethod);
        assert(all(strcmp({q.kind},expectedKind)), ...
            'Approximation unexpectedly mixed geometry types');
    end
    parts=[parts;q(:)]; %#ok<AGROW>
    meta(end+1)=detail; %#ok<AGROW>
end
assert(numel(parts)<=options.maxTotalParts, ...
    'Approximation exceeded scene-wide part budget');
if isempty(objects)
    staticIdx=[];dynamicIdx=[];
else
    staticIdx=find(~[objects.isDynamic]);
    dynamicIdx=find([objects.isDynamic]);
end
world=struct('scene',sceneName,'approximation',approxMethod, ...
    'objects',objects,'parts',parts,'meta',meta,'ground',ground, ...
    'options',options,'partBudgets',allocated,'desiredPartBudgets',desired, ...
    'sourceObjects',sourceObjects,'baseObjects',objects, ...
    'baseParts',parts,'time',0, ...
    'staticIndices',staticIdx,'dynamicIndices',dynamicIdx);
end

function kind=method_kind(method)
switch method
    case {'single_sphere','multiple_spheres'},kind='sphere';
    case {'single_ellipsoid','multiple_ellipsoids'},kind='ellipsoid';
    case 'superellipsoid',kind='superellipsoid';
    case 'convex_polyhedra',kind='convex_polyhedron';
    otherwise,error('Unknown approximation method');
end
end

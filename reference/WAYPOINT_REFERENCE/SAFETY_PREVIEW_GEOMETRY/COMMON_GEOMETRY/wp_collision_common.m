function report=wp_collision_common(cfg,shape,world,expectedKind,contactFcn)
% Shared iteration over system and obstacle primitives; per-model dispatch
% stays inside the corresponding MODEL_* folder.
assert(strcmp(shape.method,cfg.method));
if ~isempty(expectedKind)
    for i=1:numel(shape.items)
        if strcmp(shape.items(i).tag,'cable') || strncmp(shape.items(i).tag,'cable_',6)
            assert(strcmp(shape.items(i).kind,expectedKind));
        end
    end
end
assert(isfield(world,'parts') && isfield(world,'ground'));
N=numel(shape.items);K=numel(world.parts);
minG=Inf;worstTag='';worstObstacle='';worstMethod='';hits=0;usedBound=false;
for j=1:N
    item=shape.items(j);
    for i=1:K
        part=world.parts(i);
        [g,checkKind]=contactFcn(item,part,cfg.nearDistance);
        if strcmp(checkKind,'circumsphere_lower_bound'),usedBound=true;end
        if g<minG
            minG=g;worstTag=item.tag;worstObstacle=part.obstacleName;worstMethod=checkKind;
        end
        if g<=0,hits=hits+1;end
    end
end
groundMin=Inf;groundTag='';
if cfg.checkGround && ~isempty(world.ground)
    ground=world.ground;
    if ~strcmp(ground.kind,'halfspace') || ~strcmp(ground.forbiddenSide,'negative')
        error('WPPreview:UnsupportedGround','Ground requires halfspace negative side');
    end
    n=ground.normal(:);n=n/norm(n);p0=ground.point(:);
    if isfield(ground,'safetyMargin'), gm=ground.safetyMargin;else,gm=0;end
    for i=1:N
        item=shape.items(i);
        switch item.kind
            case 'sphere'
                z=dot(n,item.center-p0)-item.radius-gm;
            case 'capsule'
                z=min(dot(n,item.p0-p0),dot(n,item.p1-p0))-item.radius-gm;
            case 'ellipsoid'
                z=dot(n,item.center-p0)-norm(item.axes.*(item.R'*n))-gm;
        end
        if z<groundMin,groundMin=z;groundTag=item.tag;end
        if z<=0,hits=hits+1;end
    end
end
if groundMin<minG
    worstTag=groundTag;worstObstacle=world.ground.name;worstMethod='ground_halfspace';
end
report=struct('collision',hits>0,'hitCount',hits,'minObstacleClearance',minG, ...
    'minGroundClearance',groundMin, 'minClearance',min(minG,groundMin), ...
    'worstSystemPart',worstTag,'worstObstacle',worstObstacle, ...
    'worstCheckMethod',worstMethod,'worstGroundPart',groundTag, ...
    'numSystemParts',N,'numObstacleParts',K,'status','sampled_geometry', ...
    'clearanceIsExactEuclidean',~strcmp(cfg.method,'D') && ~usedBound, ...
    'usedConservativeDistanceLowerBound',usedBound);
end

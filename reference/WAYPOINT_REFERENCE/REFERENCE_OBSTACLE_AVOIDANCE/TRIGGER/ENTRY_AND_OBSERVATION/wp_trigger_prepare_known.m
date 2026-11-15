function known = wp_trigger_prepare_known(world)
%WP_TRIGGER_PREPARE_KNOWN Cache geometry supplied by an OBSERVATION/MEMORY source.
% Does NOT query the environment or obtain unknown obstacles.
% Supported: static outer-cover spheres/ellipsoids and a plane ground.
% Rebuild ONLY after a change of observed obstacle geometry or ground.
if ~isstruct(world) || ~isfield(world,'parts')
    error('Trigger:KnownWorld','Pass the observed/memorized world.parts only');
end
P=numel(world.parts);
known.center=zeros(3,P); known.outerRadius=zeros(1,P);
known.axes=zeros(3,P); known.R=zeros(3,3,P); known.kind=zeros(1,P);
known.id=cell(1,P);
for j=1:P
    p=world.parts(j);
    c=p.center(:);
    if numel(c)~=3 || any(~isfinite(c)) || ~isequal(size(p.R),[3,3])
        error('Trigger:InvalidPart','Invalid center/rotation at part %d',j);
    end
    if norm(p.R'*p.R-eye(3),'fro')>1e-7 || det(p.R)<0.999999
        error('Trigger:InvalidRotation','Part %d rotation must be SO(3)',j);
    end
    if strcmp(p.kind,'sphere')
        a=repmat(p.params.radius,3,1); kind=1;
    elseif strcmp(p.kind,'ellipsoid')
        a=p.params.axes(:); kind=2;
    else
        error('Trigger:UnsupportedPart','Use an outward sphere/ellipsoid cover, not %s',p.kind);
    end
    if numel(a)~=3 || any(~isfinite(a)) || any(a<=0)
        error('Trigger:InvalidSize','Part %d has invalid semiaxes',j);
    end
    known.center(:,j)=c; known.outerRadius(j)=max(a);
    known.axes(:,j)=a; known.R(:,:,j)=p.R;
    known.kind(j)=kind;
    if isfield(p,'obstacleName') && ~isempty(p.obstacleName)
        known.id{j}=char(p.obstacleName);
    else
        known.id{j}=sprintf('part_%d',j);
    end
end
known.numParts=P;
known.dynamicUnsupported=isfield(world,'dynamicIndices') && ...
    ~isempty(world.dynamicIndices);
known.groundEnabled=false; known.groundSupported=true;
known.groundNormal=[0;0;1]; known.groundPoint=[0;0;0];
known.groundMargin=0;
if isfield(world,'ground') && ~isempty(world.ground)
    g=world.ground;
    if ~isfield(g,'kind') || ~strcmp(g.kind,'halfspace') || ...
            ~isfield(g,'forbiddenSide') || ~strcmp(g.forbiddenSide,'negative')
        known.groundSupported=false;
    else
        n=g.normal(:);p=g.point(:);
        if numel(n)~=3 || numel(p)~=3 || any(~isfinite([n;p])) || norm(n)<=0
            known.groundSupported=false;
        else
            known.groundEnabled=true;
            known.groundNormal=n/norm(n);
            known.groundPoint=p;
            if isfield(g,'safetyMargin'),known.groundMargin=g.safetyMargin;end
            if ~isfinite(known.groundMargin) || known.groundMargin<0
                known.groundSupported=false;
            end
        end
    end
end
known.staticOnly=~known.dynamicUnsupported;
known.isObservationSourceVerified=false; % Must be established by the caller.
known.isOuterCoverageVerified=false;   % An approximation label is not proof.
end

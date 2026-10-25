function R=AF7R58R94_SUPPORT_SEED_ROUTES(pStart,pEnd,knownObs,inflation,maxAssignments)
% Geometry-only candidate homotopies around any positive-size oriented
% ellipsoids. No fixed obstacle radius, obstacle count, path-speed or
% avoidance-time cap. CPU effort is controlled by maxAssignments ONLY.
% Every candidate remains UNPROVED until whole-body continuous certificates.
assert(isnumeric(pStart)&&isequal(size(pStart),[3 1])&&all(isfinite(pStart)) ...
    &&isnumeric(pEnd)&&isequal(size(pEnd),[3 1])&&all(isfinite(pEnd)), ...
    'R94:FiniteWorldEndpointsRequired');
assert(isstruct(knownObs),'R94:KnownObstacleStructRequired');
assert(isnumeric(inflation)&&isscalar(inflation)&&isfinite(inflation)&&inflation>=0, ...
    'R94:NonnegativePhysicalSeedInflationRequired');
assert(isnumeric(maxAssignments)&&isscalar(maxAssignments)&& ...
    isfinite(maxAssignments)&&maxAssignments>=1&&maxAssignments==fix(maxAssignments), ...
    'R94:FiniteCPUWorkBudgetRequired');
vec=pEnd-pStart;dist=norm(vec);
if ~(isfinite(dist)&&dist>0)
    % Position-coincident endpoints require another local chart, especially
    % for path-tangent yaw. Never pretend this is an infeasible environment.
    R=struct('status','INCONCLUSIVE_ZERO_CHORD_NEEDS_LOCAL_CHART', ...
        'routes',{{}},'route_length_m',[], ...
        'known_obstacle_count',numel(knownObs), ...
        'seed_relevant_known_indices',[], 'assignments_considered',0, ...
        'all_assignments_exhausted',false,'reference_certified',false);return;
end
u=vec/dist;ref=eye(3);[~,j]=min(abs(u));
n1=cross(u,ref(:,j));n1=n1/norm(n1);n2=cross(u,n1);n2=n2/norm(n2);
directions=[n1,-n1,n2,-n2];
relevant=false(1,numel(knownObs));progress=zeros(1,numel(knownObs));
for k=1:numel(knownObs)
    O=knownObs(k);
    assert(isfield(O,'center')&&isfield(O,'R')&&isfield(O,'axes')&& ...
        isequal(size(O.center),[3 1])&&isequal(size(O.axes),[3 1])&& ...
        isequal(size(O.R),[3 3])&&all(isfinite(O.center))&& ...
        all(isfinite(O.axes))&&all(O.axes>0)&&all(isfinite(O.R(:))) && ...
        norm(O.R'*O.R-eye(3),'fro')<1e-9 && abs(det(O.R)-1)<1e-9, ...
        'R94:InvalidExternalOrientedEllipsoid');
    c=double(O.center);tau=max(0,min(1,dot(c-pStart,u)/dist));
    foot=pStart+tau*vec;
    % Bounding sphere is an inclusion-only *SEED* broadphase. Do not
    % filter endpoint-overlap ellipsoids using a strict 0<tau<1 condition.
    % Final per-body R80 checks ALL sensor-known obstacles without filtering.
    relevant(k)=norm(c-foot)<=max(double(O.axes))+inflation;
    progress(k)=tau;
end
idx=find(relevant);[~,ord]=sort(progress(idx));idx=idx(ord);
if isempty(idx)
    R=struct('status','DIRECT_SEED__FINAL_CERT_REQUIRED', ...
        'routes',{{zeros(3,0)}},'route_length_m',dist, ...
        'seed_relevant_known_indices',idx, ...
        'known_obstacle_count',numel(knownObs), ...
        'assignments_considered',1,'all_assignments_exhausted',true, ...
        'reference_certified',false);return;
end
M=numel(idx);digits=zeros(1,M);paths=cell(1,0);lengths=[];
exhausted=false;orientations=0;
% maxAssignments counts returned candidate routes, not ellipsoid size.
while numel(paths)<maxAssignments
    orientations=orientations+1;
    W1=zeros(3,M);W2=zeros(3,2*M);
    for k=1:M
        O=knownObs(idx(k));n=directions(:,digits(k)+1);
        Rc=double(O.R);axes=double(O.axes);c=double(O.center);
        S=Rc*diag(axes.^2)*Rc';
        sn=sqrt(n'*S*n);su=sqrt(u'*S*u);
        % Exact ellipsoid support point and conservative seed clearance.
        % Two points give an alternative 'shoulder' homotopy for broad
        % obstacles whose one-waypoint seed clips the tangent transition.
        normalSupport=S*n/sn;
        W1(:,k)=c+normalSupport+inflation*n;
        W2(:,2*k-1)=c-su*u+(sn+inflation)*n;
        W2(:,2*k)=c+su*u+(sn+inflation)*n;
    end
    variants={W1,W2};
    for v=1:numel(variants)
        if numel(paths)>=maxAssignments,break;end
        w=variants{v};chain=[pStart,w,pEnd];
        ell=sum(vecnorm(diff(chain,1,2),2,1));
        if isfinite(ell)&&ell>0
            paths{end+1}=w; %#ok<AGROW>
            lengths(end+1)=ell; %#ok<AGROW>
        end
    end
    carry=true;
    for k=M:-1:1
        if carry
            digits(k)=digits(k)+1;
            if digits(k)==4,digits(k)=0;else,carry=false;end
        end
    end
    if carry,exhausted=true;break;end
end
[lengths,order]=sort(lengths);paths=paths(order);
R=struct('status','ANALYTIC_ORIENTED_SUPPORT_SEEDS__NOT_CERTIFIED', ...
    'routes',{paths},'route_length_m',lengths, ...
    'seed_relevant_known_indices',idx, ...
    'known_obstacle_count',numel(knownObs), ...
    'assignments_considered',numel(paths), ...
    'support_orientations_tested',orientations, ...
    'all_assignments_exhausted',exhausted, ...
    'reference_certified',false);
end

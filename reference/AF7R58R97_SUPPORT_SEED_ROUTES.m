function R=AF7R58R97_SUPPORT_SEED_ROUTES(pStart,pEnd,obs,clearance,maxAssignments)
% Geometry-first path seeds. Every returned segment has a numeric strict
% payload support corridor against ALL known obstacles (not full body).
% Adaptive shoulders track actual ellipsoid size. No global distance/time
% cutoff. No route found within finite CPU work is INCONCLUSIVE, not unsafe.
assert(isnumeric(pStart)&&isequal(size(pStart),[3 1])&&all(isfinite(pStart)) ...
 &&isnumeric(pEnd)&&isequal(size(pEnd),[3 1])&&all(isfinite(pEnd)), ...
 'R97:Endpoints','Require finite 3x1 endpoints.');
assert(isstruct(obs)&&isscalar(clearance)&&isnumeric(clearance)&& ...
 isfinite(clearance)&&clearance>=0&&isnumeric(maxAssignments)&& ...
 isscalar(maxAssignments)&&isfinite(maxAssignments)&& ...
 maxAssignments>=1&&fix(maxAssignments)==maxAssignments, ...
 'R97:GeometryAndCPUBudget','Need known obstacle struct, clearance and work budget.');
R=struct('status','INCONCLUSIVE_NO_GEOMETRY_FIRST_ROUTE', ...
 'routes',{{}},'route_length_m',[],'seed_relevant_known_indices',1:numel(obs), ...
 'known_obstacle_count',numel(obs),'assignments_considered',0, ...
 'all_assignments_exhausted',false,'reference_certified',false, ...
 'chord_corridors_verified',false,'incomplete_branch_count',0);
d=pEnd-pStart;D=norm(d);
if D<=0
 R.status='COINCIDENT_ENDPOINTS_NEED_LOCAL_CHART';return;
end
% Reject impossible fixed boundaries before expensive shoulder searches.
% The ellipsoid interior test is a necessary collision witness (a stronger
% condition than merely being inside the inflated obstacle).
for ii=1:numel(obs)
 o=obs(ii);
 for p=[pStart pEnd]
    z=double(o.R)'*(p-double(o.center));
    if norm(z./double(o.axes))<=1
       R.status='FIXED_ENDPOINT_INSIDE_KNOWN_ELLIPSOID__NO_UNSAFE_SEED';
       return;
    end
 end
end
pre=AF7R58R97_TIGHT_SUPPORT_CORRIDOR([pStart pEnd],obs,clearance);
if pre.feasible_corridor_family
 R.routes={zeros(3,0)};R.route_length_m=D;
 R.assignments_considered=1;R.all_assignments_exhausted=true;
 R.chord_corridors_verified=true;
 R.status='DIRECT_CHORD_CORRIDOR_AVAILABLE__CURVE_NOT_CERTIFIED';return;
end
u=d/D;E=eye(3);[~,k]=min(abs(E'*u));
e1=cross(u,E(:,k));e1=e1/norm(e1);e2=cross(u,e1);
side=[e1,-e1,e2,-e2];
for j=1:numel(obs)
 o=obs(j);Rj=double(o.R);ax=double(o.axes);
 [~,ix]=sort(ax,'descend');
 for i=1:2
    n=Rj(:,ix(i));n=n-u*dot(u,n);
    if norm(n)>128*eps(1)
        n=n/norm(n);side=[side n -n]; %#ok<AGROW>
    end
 end
end
% Preserve order, drop repeated *same orientation* only.
keep=true(1,size(side,2));
for i=1:size(side,2)
 if ~keep(i),continue;end
 for j=i+1:size(side,2)
    if dot(side(:,i),side(:,j))>1-1e-12,keep(j)=false;end
 end
end
side=side(:,keep);
paths={};lens=[];
for branch=1:maxAssignments
 direction=side(:,1+mod(branch-1,size(side,2)));
 chain=[pStart pEnd];refinements=0;
 workBudget=max(1,4*numel(obs)*maxAssignments); % CPU work only
 while refinements<workBudget
    C=AF7R58R97_TIGHT_SUPPORT_CORRIDOR(chain,obs,clearance);
    if C.feasible_corridor_family
        P=chain(:,2:end-1);
        if ~isduplicate(paths,P)
            paths{end+1}=P; %#ok<AGROW>
            lens(end+1)=sum(vecnorm(diff(chain,1,2),2,1)); %#ok<AGROW>
        end
        break;
    end
    if ~isfinite(C.first_bad_segment)||~isfinite(C.first_bad_obstacle)
        R.incomplete_branch_count=R.incomplete_branch_count+1;break;
    end
    s=C.first_bad_segment;j=C.first_bad_obstacle;
    a=chain(:,s);b=chain(:,s+1);v=b-a;
    if norm(v)<=0
        R.incomplete_branch_count=R.incomplete_branch_count+1;break;
    end
    v=v/norm(v);o=obs(j);
    n=direction-v*dot(v,direction);
    if norm(n)<=128*eps(1)
        E=eye(3);[~,idx]=min(abs(E'*v));n=cross(v,E(:,idx));
    end
    n=n/norm(n);
    center=double(o.center);ax=double(o.axes);Ro=double(o.R);
    su=norm(ax.*(Ro'*v));sn=norm(ax.*(Ro'*n));
    h=sn+clearance+1024*eps(max([1;ax;abs(center);clearance]));
    success=false;
    for it=1:40 % iteration budget, NOT geometric height cap
        w1=center-su*v+h*n;w2=center+su*v+h*n;
        check=AF7R58R97_TIGHT_SUPPORT_CORRIDOR([a w1 w2 b],o,clearance);
        if check.feasible_corridor_family
            success=true;break;
        end
        h=2*h;
        if ~isfinite(h),break;end
    end
    if ~success
        R.incomplete_branch_count=R.incomplete_branch_count+1;break;
    end
    chain=[chain(:,1:s),w1,w2,chain(:,s+1:end)]; %#ok<AGROW>
    refinements=refinements+1;
 end
 if refinements>=workBudget
    R.incomplete_branch_count=R.incomplete_branch_count+1;
 end
end
R.assignments_considered=maxAssignments;
if ~isempty(paths)
 [lens,ord]=sort(lens);R.routes=paths(ord);R.route_length_m=lens;
 R.chord_corridors_verified=true;
 R.status='GEOMETRY_FIRST_CHORD_CORRIDORS_FOUND__FULL_CERT_REQUIRED';
end
end

function tf=isduplicate(paths,p)
tf=false;
for i=1:numel(paths)
 q=paths{i};
 if isequal(size(p),size(q))&&all(p(:)==q(:))
    tf=true;return;
 end
end
end

function G=TRAJECTORY_C9_CONFLICT_GROUP_GUIDE(piece,shapes,proof,payload_radius,clearance,gravity_world_mps2)
% TRAJECTORY_C9_CONFLICT_GROUP_GUIDE  SO(3)-covariant multi-obstacle QP aid.
% This routine selects one data-derived common separating normal for a
% CONNECTED group of obstacles that blocks a failed continuous-time proof.
% Only the same existing 12-variable C9 QP uses it. NO obstacle is omitted
% from that QP or from the subsequent independent full-time certificate.
% It is NOT a collision certificate, nor a proof that another homotopy is
% infeasible. No fixed left/right/up choice or shape-count-dependent solver.
% Optional live gravity is used ONLY to resolve a geometrically collinear
% path/obstacle configuration. Both transverse signs are explored; gravity
% never forces a preferred bypass side. Without a nonzero transverse
% physical direction, return UNKNOWN instead of choosing a world axis.
if nargin<6,gravity_world_mps2=[];end
if ~isempty(gravity_world_mps2)
    assert(isnumeric(gravity_world_mps2)&&isreal(gravity_world_mps2)&& ...
        numel(gravity_world_mps2)==3&& ...
        all(isfinite(gravity_world_mps2(:))), ...
        'TRAJECTORY_C9_GROUP:GravityAuthority', ...
        'Only measured/live gravity may resolve collinear symmetry.');
    gravity_world_mps2=double(gravity_world_mps2(:));
end
G=struct('active',false,'status','NOT_APPLICABLE', ...
    'ids',zeros(1,0),'normal',zeros(3,1), ...
    'interval_u',[NaN NaN],'deficit_m',NaN, ...
    'support_evaluations',0,'group_size',0, ...
    'adjacency_pair_tests',0);
if numel(shapes)<2 || ~isstruct(proof) || ...
        ~isfield(proof,'unresolved_time_interval_s') || ...
        ~isfield(proof,'unresolved_obstacle_ids') || ...
        ~isfield(proof,'unresolved_body_indices') || ...
        ~any(proof.unresolved_body_indices==3)
    G.status='NOT_A_MULTIOBSTACLE_PAYLOAD_WITNESS';return
end
ids=double([shapes.id]);focus=double(proof.unresolved_obstacle_ids(:).');
roots=find(ismember(ids,focus));
if isempty(roots),G.status='NO_KNOWN_FOCUS_ID';return;end
interval=double(proof.unresolved_time_interval_s(:).');
if numel(interval)~=2||~all(isfinite(interval))||interval(2)<=interval(1)
    G.status='NO_VALID_CERTIFICATE_LEAF';return
end
T=double(piece.duration_s);t0=double(piece.t0_s);
ub=(interval-t0)/T;
if any(~isfinite(ub))||ub(1)<0||ub(2)>1
    G.status='WITNESS_OUTSIDE_POLYNOMIAL';return
end
u=mean(ub);P=double(piece.control_points_m);
if ~isequal(size(P),[3 24])||~all(isfinite(P(:)))
    G.status='INVALID_CONTROL_POINTS';return
end
p=P*bern(23,u);
v=(23/T)*diff(P,1,2)*bern(22,u);
% Derive a cable axis from the SAME Flight gravity authority as the proof;
% it is not available in this helper and is therefore deliberately NOT
% guessed. World-frame directions come only from actual geometry and p'.
% Expand the proof-failing roots through obstacles whose PAYLOAD-INFLATED
% support intervals overlap along the line between their true centres.
% This is conservative grouping of *search* constraints only.
group=false(1,numel(shapes));group(roots)=true;
% Breadth-first closure of the TRUE shape-support adjacency graph.
% Each ordered (already-connected, not-yet-connected) pair is tested AT
% MOST ONCE. The previous repeated full-group scans could retest the same
% pair in every iteration, approaching O(N^3) in dense observed scenes.
% This graph is built from the current detected shape geometry only, never
% from a precomputed obstacle-slot mask or a fixed avoidance direction.
queue=roots(:).';cursor=1;
while cursor<=numel(queue)
    i=queue(cursor);cursor=cursor+1;
    for j=1:numel(shapes)
        if group(j),continue;end
        G.adjacency_pair_tests=G.adjacency_pair_tests+1;
        d=shapes(j).center-shapes(i).center;
        ds=norm(d);
        if ~(isfinite(ds)&&ds>=0),continue;end
        if ds>0
            n=d/ds;
            hi=TRAJECTORY_OBSTACLE_SUPPORT.value(shapes(i),n)- ...
                n.'*shapes(i).center;
            hj=TRAJECTORY_OBSTACLE_SUPPORT.value(shapes(j),-n)+ ...
                n.'*shapes(j).center;
            G.support_evaluations=G.support_evaluations+2;
            gap=ds-hi-hj-2*payload_radius;
        else
            gap=-Inf;
        end
        if gap<=clearance
            group(j)=true;
            queue(end+1)=j; %#ok<AGROW>
        end
    end
end
I=find(group);G.ids=ids(I);G.group_size=numel(I);
if numel(I)<2,G.status='NO_CONNECTED_OBSTACLE_PAIR';return;end
% SO(3) equivariance: candidate normals are constructed only from actual
% obstacle centres, payload point and its velocity. Both orientations of
% every nonzero data-derived direction are explored; no fixed-world axis.
V=[];nv=norm(v);
if isfinite(nv)&&nv>0,t=v/nv;else,t=[];end
for k=I
    d=p-shapes(k).center;
    if isempty(t)
        V=[V,d]; %#ok<AGROW>
    else
        % Topological bypass is transverse to the observed path tangent:
        % do not classify a mere backward longitudinal shift as a bypass.
        V=[V,d-t*(t.'*d),cross(t,d)]; %#ok<AGROW>
    end
end
for ai=1:numel(I)
    for bi=ai+1:numel(I)
        d=shapes(I(bi)).center-shapes(I(ai)).center;
        if isempty(t)
            V=[V,d]; %#ok<AGROW>
        else
            V=[V,d-t*(t.'*d),cross(t,d)]; %#ok<AGROW>
        end
    end
end
if ~isempty(t)
    centroid=mean(reshape([shapes(I).center],3,[]),2);
    V=[V,cross(t,p-centroid)];
    % Degenerate collinear case: all obstacle-derived transverse vectors
    % vanish. The actual gravitational acceleration is a physical vector
    % already available in the Flight contract, not an arbitrary world-axis
    % bypass instruction. cross(t,g) has both signs examined below.
    % Do NOT invoke the fallback if obstacle geometry already distinguishes
    % transverse directions. Thus ordinary cases retain their prior QP.
    % Roundoff from SO(3) transforms must not masquerade as a physical
    % obstacle offset. This threshold is scaled to the actual coordinates.
    guard=64*eps(max([1;abs(p(:)); ...
        abs(reshape([shapes(I).center],[],1))]));
    usable=false;
    for vi=1:size(V,2)
        usable=usable || (isfinite(norm(V(:,vi)))&&norm(V(:,vi))>guard);
    end
    if ~usable && ~isempty(gravity_world_mps2)
        transverse=cross(t,gravity_world_mps2);
        if isfinite(norm(transverse))&&norm(transverse)>0
            V=[V,transverse]; %#ok<AGROW>
        end
    end
end
if ~isempty(t)
    direction_guard=guard;
else
    direction_guard=0;
end
% Require a positive local deficit: a direction already satisfying the
% common group plane cannot move a repeatedly failing candidate.
best=Inf;normal=[];
for k=1:size(V,2)
    s=norm(V(:,k));
    if ~(isfinite(s)&&s>direction_guard),continue;end
    for signum=[-1 1]
        n=signum*V(:,k)/s;
        d=-Inf;
        for j=I
            h=TRAJECTORY_OBSTACLE_SUPPORT.value(shapes(j),n);
            G.support_evaluations=G.support_evaluations+1;
            d=max(d,h+payload_radius+clearance-n.'*p);
        end
        % Rotated symmetric groups have mathematically tied scores.
        % Resolve only roundoff-level ties in enumeration order, not by
        % fluctuating floating-point last bits or any fixed world axis.
        score_guard=64*eps(max([1;abs(p(:)); ...
            abs(reshape([shapes(I).center],[],1));abs(d)]));
        if ~isempty(normal)
            score_guard=max(score_guard,64*eps(max(1,abs(best))));
        end
        if isfinite(d)&&d>0&& ...
                (isempty(normal)||d<best-score_guard)
            best=d;normal=n;
        end
    end
end
if isempty(normal),G.status='NO_DATA_DERIVED_VIOLATED_GROUP_PLANE';return;end
G.active=true;G.status='GROUP_SUPPORT_PLANE_FOR_SAME_QP__NOT_A_CERTIFICATE';
G.normal=normal;G.deficit_m=best;G.interval_u=ub;
end

function b=bern(n,u)
% Stable Bernstein basis for n=21,22,23; independent of obstacle geometry.
b=zeros(n+1,1);
if u<=0,b(1)=1;return;end
if u>=1,b(end)=1;return;end
if u<=0.5
    b(1)=(1-u)^n;
    for k=1:n,b(k+1)=b(k)*(n-k+1)/k*u/(1-u);end
else
    b(end)=u^n;
    for k=n:-1:1,b(k)=b(k+1)*k/(n-k+1)*(1-u)/u;end
end
end

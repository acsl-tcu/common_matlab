function R=TRIGGER_NOMINAL_SWEPT_INTERVALS(U,L,t,shapes,ru,rl,rc,announced_ids,sampled_ids)
%TRIGGER_NOMINAL_SWEPT_INTERVALS  Directional continuous nominal sweep.
% Every linearly interpolated straight cable segment over [t(k),t(k+1)]
% lies in conv{U(k),U(k+1),L(k),L(k+1)}. The UAV and load sweeps are the
% two corresponding segments. No axis-aligned box is used as a collision
% surrogate. For any reported CLEAR interval, a separating plane has been
% found between each component's entire convex swept hull (inflated by
% its own physical radius) and the observed obstacle's convex hull.
%
% One cheap, DIRECTIONAL broad phase uses the middle-time cable segment
% and a temporal displacement bound, not a sphere around the UAV and not
% an AABB. Only intervals surviving this NECESSARY test reach support
% separation. A failed separation means POSSIBLE contact, never safety.
% Source obstacle shape support is exact for the listed convex shapes;
% for non-convex shapes it is their convex hull (conservative), and for
% unknown shapes it falls back to the validated enclosing ball.
%
% Guarantee scope: the piecewise-AFFINE INTERPOLANT of numerical sample
% positions, with straight cable. No true-ODE, estimator-error or actual
% tracking-error bound is implied. Numerical slack is an engineering guard,
% not certified directed-rounding interval arithmetic.
% sampled_ids intentionally unused: later sampled contacts MUST NOT hide
% an earlier possible contact during a preceding time interval.
assert(isnumeric(U)&&isreal(U)&&size(U,1)==3 && ...
    isequal(size(U),size(L))&&all(isfinite(U(:)))&&all(isfinite(L(:))) && ...
    isnumeric(t)&&isrow(t)&&numel(t)==size(U,2) && ...
    numel(t)>=2&&all(isfinite(t))&&all(diff(t)>0) && ...
    isstruct(shapes)&&all(isfinite([ru rl rc]))&&all([ru rl rc]>=0), ...
    'TRIGGER_NOMINAL_SWEPT_INTERVALS:Inputs', ...
    'Finite 3xN endpoints, increasing time, observed shapes and radii required.');
if nargin<8,announced_ids=zeros(1,0);end
if nargin<9,sampled_ids=zeros(1,0);end
ids=reshape(double([shapes.id]),1,[]);
assert(numel(unique(ids))==numel(ids) && all(isfinite(ids)) && ...
    all(isfinite(announced_ids(:))) && all(isfinite(sampled_ids(:))), ...
    'TRIGGER_NOMINAL_SWEPT_INTERVALS:IDs','All observed IDs must be unique.');
N=numel(t)-1;
% Mid-time segment and *half* the largest endpoint displacement. Every
% point of the swept cable hull lies within this directional capsule:
% d(point, [midU,midL]) <= .5*max(||dU||,||dL||).
% It also contains both body centre sweeps. This is a broad-phase bound,
% NOT a physical obstacle-inflation radius or collision decision.
U0=U(:,1:end-1); U1=U(:,2:end);
L0=L(:,1:end-1); L1=L(:,2:end);
midU=(U0+U1)*0.5; midL=(L0+L1)*0.5;
span=midL-midU;
span2=sum(span.^2,1);
move_half=0.5*max(sqrt(sum((U1-U0).^2,1)), ...
                  sqrt(sum((L1-L0).^2,1)));
physical_max=max([ru rl rc]);
% Motion-only magnitude is identical for all observed obstacles. Cache
% this scalar once, not inside the vectorized test for every obstacle ID.
% The guard expression remains numerically identical (same max inputs).
frame_scale=max([1;abs(midU(:));abs(midL(:));physical_max]);
first=N+1; first_ids=zeros(1,0); first_parts=cell(1,0);
first_new=N+1; new_ids=zeros(1,0); new_parts=cell(1,0);
possible_ids=zeros(1,0); intervals_considered=0;
for j=1:numel(shapes)
    sh=shapes(j);id=ids(j);
    assert(isfield(sh,'is_stationary')&&isequal(sh.is_stationary,true)&& ...
        isfield(sh,'center0')&&isfield(sh,'bounding_radius_m'), ...
        'TRIGGER_NOMINAL_SWEPT_INTERVALS:Shape', ...
        'A stationary, prepared, observed obstacle is required.');
    c=double(sh.center0(:));b=double(sh.bounding_radius_m);
    assert(isequal(size(c),[3 1])&&all(isfinite(c)) && ...
        isscalar(b)&&isfinite(b)&&b>=0, ...
        'TRIGGER_NOMINAL_SWEPT_INTERVALS:OuterBall', ...
        'An observed obstacle must have a valid outer-ball enclosure.');
    is_new=~any(announced_ids==id);
    stop=min(first,N);
    if is_new,stop=min(first_new,N);end
    if stop==0,continue;end
    % ONE vectorized necessary filter: point-to-MID-CABLE distance. Unlike
    % an AABB, a stationary sloping cable's far corners are NEVER included.
    v=c-midU(:,1:stop);
    alpha=zeros(1,stop);
    nonzero=span2(1:stop)>0;
    if any(nonzero)
        z=sum(v(:,nonzero).*span(:,nonzero),1)./span2(nonzero);
        alpha(nonzero)=min(1,max(0,z));
    end
    q=midU(:,1:stop)+span(:,1:stop).*alpha;
    offset=q-c;
    scale=max([frame_scale;abs(c);b]);
    guard=1024*eps(scale);
    limit=b+physical_max+move_half(1:stop)+guard;
    candidates=find(sum(offset.^2,1)<=limit.^2);
    % For long cylinders/ellipsoids, the enclosing BALL can be metres
    % larger than their true width. Restrict surviving intervals by the
    % orthogonal projection of a second, SHAPE-DIRECTIONAL capsule. Both
    % predicates are necessary bounds, never extra collision decisions.
    if ~isempty(candidates)
        [ax,width]=long_axis_enclosure(sh,b);
        if width<b
            ind=candidates;
            a0=midU(:,ind)-c;a1=midL(:,ind)-c;
            a0=a0-ax*(ax.'*a0);a1=a1-ax*(ax.'*a1);
            dv=a1-a0;dv2=sum(dv.^2,1);
            beta=zeros(1,numel(ind));active=dv2>0;
            if any(active)
                beta(active)=min(1,max(0, ...
                    -sum(a0(:,active).*dv(:,active),1)./dv2(active)));
            end
            radial=a0+dv.*beta;
            radial_limit=width+physical_max+move_half(ind)+guard;
            candidates=ind(sum(radial.^2,1)<=radial_limit.^2);
        end
    end
    % Validate the ACTUAL OBSERVED shape once only if its sweep passed
    % the necessary broad-phase tests. All supported types use this same
    % immutable descriptor; malformed/unsupported descriptions fall back
    % to the original conservative enclosing-ball support, never CLEAR.
    if isempty(candidates),continue;end
    support=validated_obstacle_support(sh);
    for k=candidates
        intervals_considered=intervals_considered+1;
        a=U0(:,k);aa=U1(:,k);d=L0(:,k);dd=L1(:,k);
        scale_k=max([1;abs(c);abs(a);abs(aa);abs(d);abs(dd);b;ru;rl;rc]);
        slack=1024*eps(scale_k);
        hit='';
        % Separate each component using its OWN radius. Separation is a
        % sufficient proof of no intersection for the entire interval.
        if ~separated_hull(c,sh,[a aa],ru,slack,b,support)
            hit='UAV';
        elseif ~separated_hull(c,sh,[d dd],rl,slack,b,support)
            hit='PAYLOAD';
        elseif ~separated_hull(c,sh,[a aa d dd],rc,slack,b,support)
            hit='CABLE';
        end
        if isempty(hit),continue;end
        possible_ids(end+1)=id; %#ok<AGROW>
        if k<first
            first=k;first_ids=id;first_parts={hit};
        elseif k==first
            first_ids(end+1)=id;first_parts{end+1}=hit; %#ok<AGROW>
        end
        if is_new
            if k<first_new
                first_new=k;new_ids=id;new_parts={hit};
            elseif k==first_new
                new_ids(end+1)=id;new_parts{end+1}=hit; %#ok<AGROW>
            end
        end
        break
    end
end
R=struct('status','DIRECTIONAL_CONVEX_HULL_NOMINAL_SWEEP', ...
    'possible_obstacle_ids',possible_ids,'all_sampled_ids_swept',true, ...
    'first_obstacle_ids',first_ids,'first_interval_index',NaN, ...
    'first_ttc_s',NaN,'first_time_bracket_s',[NaN NaN], ...
    'first_parts',{first_parts}, ...
    'new_obstacle_ids',new_ids,'new_interval_index',NaN, ...
    'new_ttc_s',NaN,'new_time_bracket_s',[NaN NaN], ...
    'new_parts',{new_parts},'evaluated_intervals',intervals_considered, ...
    'max_midpoint_sweep_bound_m',max(move_half), ...
    'continuous_linear_interpolant_may_intersect',first<=N, ...
    'real_plant_continuous_safety_proven',false);
if first<=N
    R.first_interval_index=first;R.first_ttc_s=t(first);
    R.first_time_bracket_s=t(first:first+1);
end
if first_new<=N
    R.new_interval_index=first_new;R.new_ttc_s=t(first_new);
    R.new_time_bracket_s=t(first_new:first_new+1);
end
end

function separated=separated_hull(c,sh,V,r,tol,b,support)
% A successful inequality n'*(V-c) > support_obstacle(n)+r+tol
% is a separating plane for the FULL convex swept hull. A failure to find
% a plane is conservatively an INTERSECTION POSSIBILITY.
W=V-c;
% Tested for ALL source shapes, not just cylinders: if the complete
% convex hull is strictly outside any enclosing local-axis slab, a plane
% separates the entire swept body/cable from the original shape. This is
% a SUFFICIENT clearance certificate, not an obstacle or trajectory AABB.
% It cannot lose a true contact, even for rotated ellipsoids/boxes, cones,
% polygon prisms, polyhedra, torus convex hulls or arbitrary SO(3).
if ~isempty(support)
    local=support.R.'*W;
    if any(min(local,[],2)>support.upper+r+tol | ...
           max(local,[],2)<support.lower-r-tol)
        separated=true;return
    end
    % For elongated round / prismatic shapes, the source's exact radial
    % extent can exclude many extra candidates without any GJK iterations.
    % A witness normal is used only if its projection is strictly beyond
    % an enclosing disk of the entire source geometry.
    if support.radial<b
        if size(V,2)==2
            xy=closest_segment([0;0],local(1:2,1),local(1:2,2));
        else
            xy=closest_segment([0;0], ...
                0.5*(local(1:2,1)+local(1:2,2)), ...
                0.5*(local(1:2,3)+local(1:2,4)));
        end
        rho=norm(xy);
        if rho>tol
            nxy=xy/rho;
            if min(nxy.'*local(1:2,:))>support.radial+r+tol
                separated=true;return
            end
        end
    end
end
% Use a point IN the hull close to the obstacle centre as first normal.
if size(V,2)==2
    q=closest_segment(c,V(:,1),V(:,2));
else
    q=closest_segment(c,0.5*(V(:,1)+V(:,2)), ...
                          0.5*(V(:,3)+V(:,4)));
end
n=q-c; nn=norm(n);
separated=false;
% Fast certified clearance for either BODY segment (UAV or payload).
% q=closest_segment(c,V(:,1),V(:,2)) is the EXACT closest point on
% conv(V). An observed solid is inside B(c,b), so if
%     dist(c,conv(V)) > b + physical_radius + numerical_slack,
% the WHOLE swept body segment is disjoint. The original code would also
% establish separation using n=(q-c)/||q-c||, since this nearest-point
% normal has min(n.'*(V-c)) >= ||q-c|| and obstacle_support(n)<=b.
% This skips an unnecessary shape-support call and any Gilbert iterations.
% DO NOT extend this shortcut to 4-point cable hulls: their q below is
% only an interior witness, NOT the nearest point of that hull.
if size(V,2)==2 && nn>b+r+tol
    separated=true;return
end
% q is itself a point of the swept hull. For a spherical obstacle this
% proves a possible overlap immediately, without iterative support work.
if isfield(sh,'type') && strcmp(sh.type,'sphere') && nn<=b+r+tol
    return
end
if nn>tol
    n=n/nn;
    [h,~]=obstacle_support(sh,n,b,support);
    if min(n.'*W)>h+r+tol
        separated=true;return
    end
end
% Gilbert/GJK-style conditional separator for thin, rotated shapes. It
% works in the Minkowski difference hull(V) - (obstacle + radius ball).
% Only a STRICT positive separating projection is accepted; convergence
% failure or iteration limit ALWAYS returns possible overlap.
if nn<=tol
    % The observed obstacle centre lies in the hull (mid-cable is in it),
    % hence the convex hulls truly intersect.
    return
end
[~,s]=obstacle_support(sh,n,b,support);
p=q-c-s-r*n;  % one actual point of the convex Minkowski difference
for it=1:24
    pn=norm(p);
    if ~(isfinite(pn)&&pn>tol),return;end
    normal=p/pn;
    [~,s]=obstacle_support(sh,normal,b,support);
    [~,ix]=min(normal.'*V);
    w=V(:,ix)-c-s-r*normal;
    if normal.'*w>tol
        separated=true;return
    end
    delta=w-p; den=dot(delta,delta);
    if ~(isfinite(den)&&den>tol^2),return;end
    fraction=min(1,max(0,-dot(p,delta)/den));
    if fraction<=4*eps(1),return;end
    p=p+fraction*delta;
end
end

function q=closest_segment(c,a,b)
d=b-a;den=dot(d,d);
if den==0
    q=a;
else
    alpha=min(1,max(0,dot(c-a,d)/den));
    q=a+alpha*d;
end
end

function [h,s]=obstacle_support(sh,n,b,support)
% Exact support POINT of the source shape's CONVEX HULL, relative to its
% centre, plus a safe enclosing-ball fallback. Non-convex holes are
% intentionally not used as a clearance certificate.
s=b*n;h=b;
% The supported source type, rotation, dimensions and local geometry have
% been checked ONCE per observed obstacle candidate, not on every Gilbert
% iteration. The original slow conservative path remains for any malformed
% or unknown geometry; no unobserved geometry is ever accessed.
if ~isempty(support)
    R=support.R;d=support.d;
    local=R.'*n;
    radial=[1;0;0];
    if support.uses_radial
        rho=hypot(local(1),local(2));
        if rho>0,radial=[local(1);local(2);0]/rho;end
    end
    switch support.kind
        case 'sphere'
            p=d(1)*local;
        case 'ellipsoid'
            width=norm(d(1:3).*local);
            if width>0,p=d(1:3).^2.*local/width;
            else,p=zeros(3,1);end
        case 'box'
            v=sign(local);v(v==0)=1;
            p=0.5*d(1:3).*v;
        case 'cylinder'
            v=sign(local(3));if v==0,v=1;end
            p=d(1)*radial+[0;0;0.5*d(2)*v];
        case 'capsule'
            v=sign(local(3));if v==0,v=1;end
            p=d(1)*local+[0;0;0.5*d(2)*v];
        case {'cone','frustum'}
            if strcmp(support.kind,'cone')
                rb=d(1);rt=0;hgt=d(2);
            else
                rb=d(1);rt=d(2);hgt=d(3);
            end
            bot=rb*radial-[0;0;hgt/2];
            top=rt*radial+[0;0;hgt/2];
            if local.'*bot>=local.'*top,p=bot;else,p=top;end
        case 'torus'
            p=d(1)*radial+d(2)*local;
        case {'pyramid','polyhedron'}
            [~,ix]=max(support.vertices*local);
            p=support.vertices(ix,:).';
        case {'regular_prism','polygon_prism'}
            [~,ix]=max(support.outline*local(1:2));
            v=sign(local(3));if v==0,v=1;end
            p=[support.outline(ix,:).';support.height*v/2];
    end
    s=R*p;h=n.'*s;
    if ~isfinite(h) || h < -1024*eps(max(1,abs(h)))
        s=b*n;h=b;return
    end
    h=max(0,h);
    return
end
try
    if ~isfield(sh,'rotation')||~isfield(sh,'dimensions')|| ...
            ~isfield(sh,'type'),return;end
    R=double(sh.rotation);
    if ~isequal(size(R),[3 3])||any(~isfinite(R(:)))|| ...
            norm(R.'*R-eye(3),'fro')>1e-10,return;end
    local=R.'*n;
    if isnumeric(sh.dimensions)
        d=double(sh.dimensions(:));
        if any(~isfinite(d))||any(d<0),return;end
    else
        d=sh.dimensions;
    end
    rho=hypot(local(1),local(2));
    radial=[1;0;0];
    if rho>0,radial=[local(1);local(2);0]/rho;end
    switch char(sh.type)
        case 'sphere'
            p=d(1)*local;
        case 'ellipsoid'
            weighted=d(1:3).*local;
            width=norm(weighted);
            if width>0
                p=(d(1:3).^2).*local/width;
            else
                p=zeros(3,1);
            end
        case 'box'
            v=sign(local);v(v==0)=1;
            p=0.5*d(1:3).*v;
        case 'cylinder'
            v=sign(local(3));if v==0,v=1;end
            p=d(1)*radial+[0;0;0.5*d(2)*v];
        case 'capsule'
            v=sign(local(3));if v==0,v=1;end
            p=d(1)*local+[0;0;0.5*d(2)*v];
        case {'cone','frustum'}
            if strcmp(sh.type,'cone')
                rb=d(1); rt=0; height=d(2);
            else
                rb=d(1); rt=d(2); height=d(3);
            end
            bot=rb*radial-[0;0;height/2];
            top=rt*radial+[0;0;height/2];
            if local.'*bot>=local.'*top,p=bot;else,p=top;end
        case 'torus'
            p=d(1)*radial+d(2)*local;
        case {'pyramid','polyhedron'}
            verts=double(sh.vertices);
            if size(verts,2)~=3||isempty(verts)||any(~isfinite(verts(:))),return;end
            [~,ix]=max(verts*local);p=verts(ix,:).';
        case {'regular_prism','polygon_prism'}
            outline=double(sh.outline);
            if size(outline,2)~=2||isempty(outline)|| ...
                    any(~isfinite(outline(:))),return;end
            [~,ix]=max(outline*local(1:2));
            if strcmp(sh.type,'regular_prism')
                height=d(3);
            else
                height=double(d.height);
            end
            if ~isscalar(height)||~isfinite(height)||height<0,return;end
            v=sign(local(3));if v==0,v=1;end
            p=[outline(ix,:).';height*v/2];
        otherwise
            return
    end
    if ~isequal(size(p),[3 1])||any(~isfinite(p)),return;end
    s=R*p;
    h=n.'*s;
    if ~isfinite(h)||h< -1024*eps(max(1,abs(h)))
        s=b*n;h=b;return
    end
    % A support vector on the actual hull is required for the Gilbert
    % update. Roundoff is covered by the external strict separation tol.
    h=max(0,h);
catch
    s=b*n;h=b;
end
end

function D=validated_obstacle_support(sh)
% A zero-allocation-from-world, per-candidate description from the REAL
% sensor/observed-memory shape. Bounds are exact local-axis outer extents
% (nonconvex source shapes use their convex hull), not inflated margins.
% If preparation is not obviously valid, use the existing ball fallback.
D=[];
try
    if ~all(isfield(sh,{'type','rotation','dimensions'})),return;end
    kind=char(sh.type);
    R=double(sh.rotation);
    if ~isnumeric(sh.rotation)||~isreal(sh.rotation)|| ...
            ~isequal(size(R),[3 3])||any(~isfinite(R(:)))|| ...
            norm(R.'*R-eye(3),'fro')>1e-10,return;end
    d=sh.dimensions;
    if isnumeric(d)
        if ~isreal(d),return;end
        d=double(d(:));
        if any(~isfinite(d))||any(d<0),return;end
    end
    verts=zeros(0,3);outline=zeros(0,2);height=0;
    switch kind
        case 'sphere'
            if ~isnumeric(d)||numel(d)<1,return;end
            ext=repmat(d(1),3,1);low=-ext;high=ext;
            radial=d(1);
        case 'ellipsoid'
            if ~isnumeric(d)||numel(d)<3,return;end
            ext=d(1:3);low=-ext;high=ext;
            radial=max(d(1:2)); % exact projected radial extent
        case 'box'
            if ~isnumeric(d)||numel(d)<3,return;end
            ext=d(1:3)/2;low=-ext;high=ext;
            radial=hypot(ext(1),ext(2));
        case 'cylinder'
            if ~isnumeric(d)||numel(d)<2,return;end
            ext=[d(1);d(1);d(2)/2];low=-ext;high=ext;
            radial=d(1);
        case 'capsule'
            if ~isnumeric(d)||numel(d)<2,return;end
            ext=[d(1);d(1);d(1)+d(2)/2];low=-ext;high=ext;
            radial=d(1);
        case 'cone'
            if ~isnumeric(d)||numel(d)<2,return;end
            ext=[d(1);d(1);d(2)/2];low=-ext;high=ext;
            radial=d(1);
        case 'frustum'
            if ~isnumeric(d)||numel(d)<3,return;end
            ext=[max(d(1:2));max(d(1:2));d(3)/2];
            low=-ext;high=ext;radial=ext(1);
        case 'torus'
            if ~isnumeric(d)||numel(d)<2,return;end
            ext=[sum(d(1:2));sum(d(1:2));d(2)];
            low=-ext;high=ext;radial=ext(1);
        case {'pyramid','polyhedron'}
            if ~isfield(sh,'vertices'),return;end
            verts=double(sh.vertices);
            if ~isnumeric(sh.vertices)||~isreal(sh.vertices)|| ...
                    size(verts,2)~=3||isempty(verts)|| ...
                    any(~isfinite(verts(:))),return;end
            low=min(verts,[],1).';high=max(verts,[],1).';
            radial=max(hypot(verts(:,1),verts(:,2)));
        case {'regular_prism','polygon_prism'}
            if ~isfield(sh,'outline'),return;end
            outline=double(sh.outline);
            if ~isnumeric(sh.outline)||~isreal(sh.outline)|| ...
                    size(outline,2)~=2||isempty(outline)|| ...
                    any(~isfinite(outline(:))),return;end
            if strcmp(kind,'regular_prism')
                if ~isnumeric(d)||numel(d)<3,return;end
                height=d(3);
            else
                if ~isstruct(d)||~isfield(d,'height'),return;end
                height=double(d.height);
            end
            if ~isscalar(height)||~isreal(height)|| ...
                    ~isfinite(height)||height<0,return;end
            low=[min(outline,[],1).';-height/2];
            high=[max(outline,[],1).';height/2];
            radial=max(hypot(outline(:,1),outline(:,2)));
        otherwise
            return
    end
    if ~all(isfinite([low;high;radial]))||any(low>high)||radial<0
        return
    end
    % A numerically supplied SO(3) is never exactly orthogonal. For the
    % world-space plane with normal R(:,i), the obstacle's projection is
    % row i of (R'*R) times its LOCAL coordinates. Enclose that projection
    % rather than silently treating R'*R as exact identity near contact.
    G=R.'*R;
    Gpositive=max(G,0);Gnegative=min(G,0);
    bound_lower=Gpositive*low+Gnegative*high;
    bound_upper=Gpositive*high+Gnegative*low;
    % Bound any small non-orthogonality also in the 2-D radial separator.
    E=G(1:2,:)-[1 0 0;0 1 0];
    radial=radial+norm(E,'fro')*norm(max(abs(low),abs(high))) + ...
        256*eps(max(1,radial));
    uses_radial=any(strcmp(kind,{'cylinder','cone','frustum', ...
        'torus','regular_prism','polygon_prism'}));
    D=struct('kind',kind,'R',R,'d',d,'uses_radial',uses_radial, ...
        'lower',bound_lower,'upper',bound_upper, ...
        'radial',radial,'vertices',verts,'outline',outline,'height',height);
catch
    D=[];
end
end

function [axis,width]=long_axis_enclosure(sh,b)
% The source solid lies in the infinite cylinder around this source-frame
% direction. An outer cylinder never rejects a real contact. This is
% useful only for elongated shapes, and has NO effect on a sphere.
axis=[0;0;1];width=b;
try
    if ~isfield(sh,'rotation')||~isfield(sh,'type')|| ...
            ~isfield(sh,'dimensions'),return;end
    R=double(sh.rotation);
    if ~isequal(size(R),[3 3])||any(~isfinite(R(:)))|| ...
            norm(R.'*R-eye(3),'fro')>1e-10,return;end
    d=sh.dimensions;
    switch char(sh.type)
        case 'ellipsoid'
            d=double(d(:));[~,k]=max(d(1:3));
            others=d(1:3);others(k)=[];
            width=max(others);
        case 'box'
            d=double(d(:));[~,k]=max(d(1:3));
            others=d(1:3);others(k)=[];
            width=norm(others)/2;
        case 'cylinder'
            k=3;width=double(d(1));
        case 'capsule'
            k=3;width=double(d(1));
        case 'cone'
            k=3;width=double(d(1));
        case 'frustum'
            k=3;width=max(double(d(1:2)));
        case 'regular_prism'
            k=3;width=max(hypot(sh.outline(:,1),sh.outline(:,2)));
        case 'polygon_prism'
            k=3;width=max(hypot(sh.outline(:,1),sh.outline(:,2)));
        otherwise
            return
    end
    if ~(isscalar(width)&&isfinite(width)&&width>=0)
        width=b;return
    end
    axis=R(:,k);
    % Expand the projected enclosing radius by a floating-point guard;
    % exact source geometry remains unchanged.
    width=min(b,width+256*eps(max(1,b)));
catch
    axis=[0;0;1];width=b;
end
end

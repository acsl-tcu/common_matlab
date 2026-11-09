function C=TRAJECTORY_SAFE_CORRIDOR(segments,observed,radius,options)
%TRAJECTORY_SAFE_CORRIDOR  Certified FULL-BODY corridor for a supplied guide.
% A corridor is an intersection of halfspaces for each time leaf.  For each
% observed static obstacle O, three separating planes are constructed:
%   nU'*U >= h_O(nU)+rU+delta    (UAV)
%   nL'*L >= h_O(nL)+rL+delta    (payload)
%   nC'*U,L >= h_O(nC)+rC+delta (the ENTIRE straight cable)
% Applied to EVERY Bezier control point of each de Casteljau time leaf,
% these finite LINEAR constraints imply noncollision at ALL times of the
% leaf, not only its samples.  The shared cable plane is essential.
% No system-enclosing ball or AABB obstacle approximation is used.
% Plane search uses the shape support witness in a rotation-equivariant
% convex Minkowski-distance iteration; no predetermined bypass direction.
%
% Input segments(k).uav/.payload are time-aligned Bezier pieces with
% .control_points_m, .t0_s and .duration_s.  The supplied GUIDE MUST
% already admit a corridor: if it penetrates an obstacle, construction
% returns UNKNOWN; this is not a detour-search or trajectory optimizer.
% Observed is ONLY the sensor-known/retained static obstacle array.
% Shape support is exact for convex primitives; for nonconvex primitives
% it is their convex-hull support (conservative but never exclusion-unsafe).
% Flags, physical radii, robust error bounds, positive clearance and
% computational budgets are REQUIRED from the caller: no guessed values.
%
% This is a NOMINAL POLYNOMIAL GEOMETRY certificate, not a certificate of
% feasibility under the HLC, force/torque/tension, tracking or rounding.
% No candidate may be COMMITTED from this certificate alone.
start=tic;
assert(isstruct(segments)&&~isempty(segments)&&isstruct(observed), ...
    'TRAJECTORY_CORRIDOR:Input','Guide segments and observed shapes required.');
needed={'robust_tube_enabled','min_separation_m','numerical_guard_m', ...
    'max_subdivision_depth','max_leaf_count','max_normal_iterations'};
assert(isstruct(options)&&isscalar(options)&&all(isfield(options,needed)), ...
    'TRAJECTORY_CORRIDOR:Authority','Missing externally defined proof options.');
assert((islogical(options.robust_tube_enabled)||isnumeric(options.robust_tube_enabled)) && ...
    isscalar(options.robust_tube_enabled)&& ...
    (isequal(options.robust_tube_enabled,0)||isequal(options.robust_tube_enabled,1)), ...
    'TRAJECTORY_CORRIDOR:Tube','Tube toggle must be 0/1.');
assert(isscalar(options.min_separation_m)&&isreal(options.min_separation_m)&& ...
    isfinite(options.min_separation_m)&&options.min_separation_m>0 && ...
    isscalar(options.numerical_guard_m)&&isreal(options.numerical_guard_m)&& ...
    isfinite(options.numerical_guard_m)&&options.numerical_guard_m>=0, ...
    'TRAJECTORY_CORRIDOR:Margin','External positive clearance and guard are required.');
for field={'max_subdivision_depth','max_leaf_count','max_normal_iterations'}
    v=options.(field{1});
    assert(isnumeric(v)&&isscalar(v)&&isreal(v)&&isfinite(v)&&v>=0&&v==fix(v), ...
        'TRAJECTORY_CORRIDOR:Budget','All proof budgets must be integers >=0.');
end
assert(options.max_leaf_count>=1 && options.max_normal_iterations>=1, ...
    'TRAJECTORY_CORRIDOR:Budget','Positive leaf and normal budgets required.');
fields={'uav_m','payload_m','cable_m'};
assert(isstruct(radius)&&isscalar(radius)&&all(isfield(radius,fields)), ...
    'TRAJECTORY_CORRIDOR:Radii','Caller-owned body and cable radii required.');
r=zeros(1,3);
for i=1:3
    v=radius.(fields{i});
    assert(isnumeric(v)&&isreal(v)&&isscalar(v)&&isfinite(v)&&v>=0, ...
        'TRAJECTORY_CORRIDOR:Radii','Invalid physical radius.');
    r(i)=double(v);
end
if options.robust_tube_enabled
    assert(isfield(options,'uncertainty_bounds_m')&& ...
        isstruct(options.uncertainty_bounds_m)&& ...
        all(isfield(options.uncertainty_bounds_m,fields)), ...
        'TRAJECTORY_CORRIDOR:Tube','External per-component tube bounds required.');
    for i=1:3
        v=options.uncertainty_bounds_m.(fields{i});
        assert(isnumeric(v)&&isreal(v)&&isscalar(v)&&isfinite(v)&&v>=0, ...
            'TRAJECTORY_CORRIDOR:Tube','Invalid supplied uncertainty bound.');
        r(i)=r(i)+double(v);
    end
end
delta=double(options.min_separation_m)+double(options.numerical_guard_m);
assert(isfinite(delta)&&delta>0&&all(isfinite(r)), ...
    'TRAJECTORY_CORRIDOR:Range','Inflated radii/margins are invalid.');
shapes=cell(1,numel(observed));ids=zeros(1,numel(observed));
for j=1:numel(observed)
    shapes{j}=prepare_shape(observed(j));
    ids(j)=shapes{j}.id;
end
assert(numel(unique(ids))==numel(ids), ...
    'TRAJECTORY_CORRIDOR:IDs','Duplicate observed IDs.');
C=struct('status','UNKNOWN_GUIDE_NOT_SEPARABLE','certified',false, ...
    'safe_to_commit',false,'scope','NOMINAL_POLYNOMIAL_ONLY', ...
    'observed_obstacle_ids',ids,'observed_snapshot',{observed}, ...
    'proof_margin_m',double(options.min_separation_m), ...
    'numerical_guard_m',double(options.numerical_guard_m), ...
    'effective_radii_m',r, 'leaves',struct([]), ...
    'segment_info',struct([]),'variable_count',0, ...
    'failed_segment_index',NaN,'failed_time_interval_s',[NaN NaN], ...
    'halfspace_count',0,'leaf_count',0,'subdivision_count',0, ...
    'duration_ms',NaN,'meaning', ...
    'Linear Bezier corridor for supplied guide, NOT dynamical feasibility');
if options.robust_tube_enabled
    C.scope='POLYNOMIAL_WITH_SUPPLIED_UNCERTAINTY_RADII';
end
next_var=1;last_end=NaN;
for k=1:numel(segments)
    assert(all(isfield(segments(k),{'uav','payload'})), ...
        'TRAJECTORY_CORRIDOR:Segment','UAV and payload pieces required.');
    [U,t,T]=read_piece(segments(k).uav);
    [L,tL,TL]=read_piece(segments(k).payload);
    assert(t==tL&&T==TL, ...
        'TRAJECTORY_CORRIDOR:Clock','UAV and payload clocks must be identical.');
    if k>1
        assert(t==last_end, ...
            'TRAJECTORY_CORRIDOR:Clock','Guide segments must have contiguous clocks.');
    end
    last_end=t+T;
    nu=size(U,2);nl=size(L,2);
    info=struct('t0_s',t,'duration_s',T,'uav_controls',nu, ...
        'payload_controls',nl,'uav_variable_start',next_var, ...
        'payload_variable_start',next_var+3*nu);
    C.segment_info=[C.segment_info info]; %#ok<AGROW>
    next_var=next_var+3*(nu+nl);
    Mu=eye(nu);Ml=eye(nl);
    [ok,leafset,badtime,nodecount]=subdivide(U,L,Mu,Ml,t,T,0, ...
        k,shapes,ids,r,delta,options,options.max_leaf_count-numel(C.leaves));
    C.subdivision_count=C.subdivision_count+nodecount;
    if ~ok
        C.failed_segment_index=k;
        C.failed_time_interval_s=badtime;
        C.status='UNKNOWN_NO_CERTIFIED_CORRIDOR_FOR_GUIDE_OR_BUDGET';
        C.duration_ms=toc(start)*1000;
        C.leaves=struct([]); % never expose a partial unsafe corridor
        C.variable_count=next_var-1;
        return
    end
    if isempty(C.leaves),C.leaves=leafset;
    else,C.leaves=[C.leaves leafset];end %#ok<AGROW>
end
C.variable_count=next_var-1;
C.certified=true;
C.status='CERTIFIED_FULL_BODY_CONTINUOUS_SAFE_CORRIDOR';
C.leaf_count=numel(C.leaves);
for i=1:C.leaf_count
    l=C.leaves(i);
    C.halfspace_count=C.halfspace_count+size(l.uav_normals,2)+ ...
        size(l.payload_normals,2)+size(l.cable_normals,2);
end
C.duration_ms=toc(start)*1000;
end

function [ok,leaves,badtime,split_count]=subdivide(U,L,Mu,Ml,t,T,depth,k,shapes,ids,r,delta,opt,budget)
% At a leaf, all shape planes must contain the guide control hull.
% If any plane cannot be witnessed, split the TIME representation, not
% the physical obstacle; de Casteljau maps remain linear in original P.
leaves=struct([]);badtime=[t,t+T];split_count=0;
if budget<1,ok=false;return;end
nu=zeros(3,0);bu=zeros(0,1);nl=zeros(3,0);bl=zeros(0,1);
nc=zeros(3,0);bc=zeros(0,1);
for i=1:numel(shapes)
    [a,p,q]=plane(U,shapes{i},r(1)+delta,opt.max_normal_iterations);
    [b,v,w]=plane(L,shapes{i},r(2)+delta,opt.max_normal_iterations);
    [c,x,y]=plane([U L],shapes{i},r(3)+delta,opt.max_normal_iterations);
    if ~(a&&b&&c),break;end
    nu(:,end+1)=p;bu(end+1,1)=q; %#ok<AGROW>
    nl(:,end+1)=v;bl(end+1,1)=w; %#ok<AGROW>
    nc(:,end+1)=x;bc(end+1,1)=y; %#ok<AGROW>
end
if numel(bu)==numel(shapes)
    leaves=struct('segment_index',k,'time_interval_s',[t t+T], ...
        'uav_map',Mu,'payload_map',Ml, ...
        'uav_normals',nu,'uav_offsets',bu, ...
        'payload_normals',nl,'payload_offsets',bl, ...
        'cable_normals',nc,'cable_offsets',bc, ...
        'obstacle_ids',ids);
    ok=true;return
end
if depth>=opt.max_subdivision_depth||budget<2
    ok=false;return
end
[U0,U1]=split_mid(U);[L0,L1]=split_mid(L);
[Mu0,Mu1]=split_mid(Mu);[Ml0,Ml1]=split_mid(Ml);
[a,ca,ba,na]=subdivide(U0,L0,Mu0,Ml0,t,T/2,depth+1,k, ...
    shapes,ids,r,delta,opt,budget-1);
if ~a,ok=false;badtime=ba;split_count=na+1;return;end
[b,cb,bb,nb]=subdivide(U1,L1,Mu1,Ml1,t+T/2,T/2,depth+1,k, ...
    shapes,ids,r,delta,opt,budget-numel(ca));
if ~b,ok=false;badtime=bb;split_count=na+nb+1;return;end
leaves=[ca cb];ok=true;split_count=na+nb+1;
end

function [success,n,beta]=plane(P,S,inflate,maxiter)
% Rotation-equivariant separation search for a convex Bernstein hull and
% an observed obstacle convex hull. No global up/left/right direction, no
% per-shape avoidance side, no fixed global basis is used.
%
% Gilbert/Frank-Wolfe nearest-point search in D = conv(P-c)-conv(O-c):
% q_(j+1) = q_j + alpha*(s_j-q_j), where
% s_j = argmin_{d in D} q_j' d.  The linear minimizer is computed using
% the exact SHAPE support witness, and an actual control point of P.
% A positive separating plane is accepted ONLY after a fresh independent
% strict support-value check in testaxis(), not from optimizer convergence.
% Inconclusive iteration budget => UNKNOWN and optional time subdivision.
W=P-S.c;
success=false;n=zeros(3,1);beta=NaN;
if isempty(W),return;end
center=mean(W,2)-support_reference_shift(S);
[success,n,beta]=testaxis(W,S,inflate,center);
if success,return;end
% If the relative centroids coincide, both convex hulls contain the
% same centroid, so no strict separator exists. No world-axis fallback.
if ~any(center),return;end
q=mean(W,2)-support_witness(S,center);
for iter=1:maxiter
    [success,n,beta]=testaxis(W,S,inflate,q);
    if success,return;end
    [~,idx]=min(q.'*W);
    s=W(:,idx)-support_witness(S,q);
    d=s-q;
    d2=d.'*d;
    if ~isfinite(d2)||d2==0,break;end
    alpha=min(1,max(0,-(q.'*d)/d2));
    if alpha==0,break;end
    q=q+alpha*d;
end
[success,n,beta]=testaxis(W,S,inflate,q);
end

function v=support_reference_shift(S)
% A genuine occupied-hull point or barycenter, not an axis heuristic.
% Polygonal obstacles may define vertices away from local origin.
if strcmp(S.kind,'polyhedron')
    p=mean(double(S.d.vertices),1).';
elseif strcmp(S.kind,'polygon_prism')
    p=[mean(double(S.d.outline),1).';0];
else
    p=zeros(3,1);
end
v=S.R*p;
end

function v=support_witness(S,world_direction)
% argmax_{o in conv(O-c)} world_direction' * o. Every returned point
% lies in the convex hull of the ORIGINAL occupied shape. For a torus it
% may lie inside its convex hull, which only makes exclusion conservative.
w=S.R.'*world_direction;
d=S.d;rho=hypot(w(1),w(2));wn=norm(w);
% The zero direction is harmless: any local origin is in the canonical
% convex hull. This is NEVER an acceptance or safety decision.
if wn==0,v=zeros(3,1);return;end
if rho>0,xy=w(1:2)/rho;else,xy=zeros(2,1);end
switch S.kind
    case 'sphere'
        p=double(d(1))*w/wn;
    case 'ellipsoid'
        a=double(d(:));den=norm(a.*w);
        if den>0,p=(a.^2.*w)/den;else,p=zeros(3,1);end
    case 'box'
        p=double(d(:)).*sign(w)/2;
    case 'cylinder'
        p=[double(d(1))*xy;double(d(2))*sign(w(3))/2];
    case 'capsule'
        p=double(d(1))*w/wn+[0;0;double(d(2))*sign(w(3))/2];
    case 'cone'
        r=double(d(1));h=double(d(2));
        if r*rho-h*w(3)/2>=h*w(3)/2
            p=[r*xy;-h/2];
        else
            p=[0;0;h/2];
        end
    case 'frustum'
        r0=double(d(1));r1=double(d(2));h=double(d(3));
        if r0*rho-h*w(3)/2>=r1*rho+h*w(3)/2
            p=[r0*xy;-h/2];
        else
            p=[r1*xy;h/2];
        end
    case 'torus'
        p=[double(d(1))*xy;0]+double(d(2))*w/wn;
    case 'pyramid'
        a=double(d(1))/2;h=double(d(2))/2;
        vertices=[a a -h;a -a -h;-a a -h;-a -a -h;0 0 h];
        [~,idx]=max(vertices*w);p=vertices(idx,:).';
    case 'regular_prism'
        k=0:(double(d(1))-1);a=2*pi*k/double(d(1));
        xyv=double(d(2))*[cos(a(:)) sin(a(:))];
        [~,idx]=max(xyv*w(1:2));
        p=[xyv(idx,:).';double(d(3))*sign(w(3))/2];
    case 'polygon_prism'
        vertices=double(d.outline);
        [~,idx]=max(vertices*w(1:2));
        p=[vertices(idx,:).';double(d.height)*sign(w(3))/2];
    case 'polyhedron'
        vertices=double(d.vertices);
        [~,idx]=max(vertices*w);p=vertices(idx,:).';
    otherwise
        error('TRAJECTORY_CORRIDOR:Support','Unknown observed shape.');
end
v=S.R*p;
assert(all(isfinite(v)),'TRAJECTORY_CORRIDOR:Witness','Nonfinite support witness.');
end

function [good,n,beta]=testaxis(W,S,inflate,q)
good=false;n=zeros(3,1);beta=NaN;
nn=norm(q);
if ~(isfinite(nn)&&nn>0),return;end
n=q/nn;
h=support(S,n);
beta=n.'*S.c+h+inflate;
% Seed must have STRICT slack. The eventual optimizer may use equality
% because inflate contains an explicit strictly positive margin.
good=min(n.'*W)>h+inflate && isfinite(beta);
end

function [L,R]=split_mid(P)
n=size(P,2);A=P;L=zeros(size(P));R=zeros(size(P));
L(:,1)=A(:,1);R(:,n)=A(:,n);
for j=2:n
    A=(A(:,1:end-1)+A(:,2:end))*0.5;
    L(:,j)=A(:,1);R(:,n-j+1)=A(:,end);
end
end
function [P,t,T]=read_piece(p)
assert(isstruct(p)&&isscalar(p)&&all(isfield(p, ...
    {'control_points_m','t0_s','duration_s'})), ...
    'TRAJECTORY_CORRIDOR:Piece','Missing Bezier controls/time.');
P=double(p.control_points_m);t=double(p.t0_s);T=double(p.duration_s);
assert(isnumeric(p.control_points_m)&&isreal(p.control_points_m)&& ...
    size(P,1)==3&&size(P,2)>=2&&all(isfinite(P(:)))&& ...
    isscalar(t)&&isfinite(t)&&isscalar(T)&&isfinite(T)&&T>0, ...
    'TRAJECTORY_CORRIDOR:Piece','Invalid Bezier geometry/time.');
end
function S=prepare_shape(o)
assert(isfield(o,'id')&&isnumeric(o.id)&&isscalar(o.id)&& ...
    isfinite(o.id)&&o.id>=1&&o.id==fix(o.id), ...
    'TRAJECTORY_CORRIDOR:Obstacle','Observed ID invalid.');
if isfield(o,'geometry')
    assert(isstruct(o.geometry)&&all(isfield(o.geometry,{'type','dimensions'}))&& ...
        isfield(o,'position')&&isfield(o,'motion')&&strcmp(char(o.motion),'static'), ...
        'TRAJECTORY_CORRIDOR:Obstacle','Only complete observed static shapes.');
    kind=char(o.geometry.type);dim=o.geometry.dimensions;c=o.position;
elseif all(isfield(o,{'type','dimensions','center0','is_stationary'}))
    assert(isequal(o.is_stationary,true), ...
        'TRAJECTORY_CORRIDOR:Obstacle','Moving obstacles not supported.');
    kind=char(o.type);dim=o.dimensions;c=o.center0;
else
    error('TRAJECTORY_CORRIDOR:Obstacle','Unsupported observed shape schema.');
end
assert(isfield(o,'rotation'), ...
    'TRAJECTORY_CORRIDOR:Obstacle','SO(3) pose required.');
R=double(o.rotation);c=double(c(:));
assert(isequal(size(R),[3 3])&&all(isfinite(R(:)))&& ...
    norm(R.'*R-eye(3),'fro')<1e-8&&abs(det(R)-1)<1e-8&& ...
    isequal(size(c),[3 1])&&all(isfinite(c)), ...
    'TRAJECTORY_CORRIDOR:Obstacle','Invalid obstacle pose.');
assert(exist('OBSTACLE_SHAPES','class')==8, ...
    'TRAJECTORY_CORRIDOR:Validator','Canonical obstacle shape class missing.');
OBSTACLE_SHAPES.validate(struct('type',kind,'dimensions',dim));
S=struct('id',double(o.id),'kind',kind,'d',dim,'R',R,'c',c);
end
function h=support(S,n)
% Exact convex-HULL support of 12 canonical primitive occupancies.
% For torus and other nonconvex shapes, a convex hull may over-exclude.
w=S.R.'*n;d=S.d;rho=hypot(w(1),w(2));
switch S.kind
    case 'sphere',h=double(d(1));
    case 'ellipsoid',h=norm(double(d(:)).*w);
    case 'box',h=sum(double(d(:)).*abs(w))/2;
    case 'cylinder',h=double(d(1))*rho+double(d(2))*abs(w(3))/2;
    case 'capsule',h=double(d(1))+double(d(2))*abs(w(3))/2;
    case 'cone',h=max(double(d(1))*rho-double(d(2))*w(3)/2, ...
        double(d(2))*w(3)/2);
    case 'frustum',h=max(double(d(1))*rho-double(d(3))*w(3)/2, ...
        double(d(2))*rho+double(d(3))*w(3)/2);
    case 'torus',h=double(d(1))*rho+double(d(2));
    case 'pyramid'
        s=double(d(1))/2;z=double(d(2))/2;
        v=[s s -z;s -s -z;-s s -z;-s -s -z;0 0 z];
        h=max(v*w);
    case 'regular_prism'
        k=0:(double(d(1))-1);a=2*pi*k/double(d(1));
        h=max(double(d(2))*(cos(a)*w(1)+sin(a)*w(2)))+ ...
            double(d(3))*abs(w(3))/2;
    case 'polygon_prism'
        h=max(double(d.outline)*w(1:2))+double(d.height)*abs(w(3))/2;
    case 'polyhedron'
        h=max(double(d.vertices)*w);
    otherwise
        error('TRAJECTORY_CORRIDOR:Support','Unknown shape (fail closed).');
end
assert(isfinite(h),'TRAJECTORY_CORRIDOR:Support','Invalid support value.');
end

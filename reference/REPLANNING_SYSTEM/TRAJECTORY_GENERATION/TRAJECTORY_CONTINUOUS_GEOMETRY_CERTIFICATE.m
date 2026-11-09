function report=TRAJECTORY_CONTINUOUS_GEOMETRY_CERTIFICATE(segments, observed, radius, options)
%TRAJECTORY_CONTINUOUS_GEOMETRY_CERTIFICATE  Whole-body nominal Bernstein proof.
% segments(k).uav and .payload must each have CONTROL_POINTS_M 3x(N+1),
% common t0_s / duration_s; may be degree 15, 23 or any finite Bernstein.
%
% For each parameter subinterval I, U(I) is inside conv(U_controls(I)),
% P(I) inside conv(P_controls(I)); a straight cable point is
% (1-lambda)*U(t)+lambda*P(t), so the FULL cable sweep is inside
% conv([U_controls(I), P_controls(I)]), not a sphere around the system.
% Every component (UAV, cable, payload) requires its OWN separating plane
% against EVERY currently observed obstacle. All certificates are based
% on STRICT projection inequalities on continuous-time Bernstein hulls.
% If no plane is found, subdivide by exact de Casteljau. Exhausting the
% external computation budget returns UNKNOWN, NEVER a safety claim.
% Nonconvex source shapes use their exact CONVEX-HULL SUPPORT: valid for
% exclusions, but possibly conservative around holes/concavities.
%
% Guarantee: under the *specified polynomial UAV AND PAYLOAD paths*,
% straight taut cable geometry, correct input shapes and real arithmetic.
% This is NOT a bound on the physical ODE/tracking error, actuator limits,
% tension or IEEE directed rounding. Robust tube radii must be PROVIDED;
% turning tube OFF yields explicitly NOMINAL-ONLY certification.
%
% Mandatory radius: struct('uav_m',...,'payload_m',...,'cable_m',...).
% Mandatory options: robust_tube_enabled (0/1), uncertainty_bounds_m
% when ON (struct with uav_m, payload_m, cable_m); max_subdivision_depth
% (nonnegative integer); numerical_guard_m (>=0, engineering slack).
% The infinite ground z=0 is an ALWAYS-KNOWN environment halfspace,
% not an observed finite obstacle. It is checked even if observed=[].
% Nothing is taken from unrevealed/unobserved obstacle definitions.
start_tick=tic;
assert(isstruct(segments)&&~isempty(segments)&&isstruct(observed), ...
    'TRAJECTORY_CERTIFICATE:Inputs','Require Bernstein segments and observed shapes.');
required={'uav_m','payload_m','cable_m'};
assert(isstruct(radius)&&isscalar(radius)&&all(isfield(radius,required)) && ...
    isstruct(options)&&isscalar(options)&& ...
    all(isfield(options,{'robust_tube_enabled','max_subdivision_depth','numerical_guard_m'})), ...
    'TRAJECTORY_CERTIFICATE:Authority','Missing caller-owned radii or proof options.');
for j=1:3
    v=radius.(required{j});
    assert(isnumeric(v)&&isreal(v)&&isscalar(v)&&isfinite(v)&&v>=0, ...
        'TRAJECTORY_CERTIFICATE:Radius','All physical radii must be finite and nonnegative.');
end
on=options.robust_tube_enabled;
assert((islogical(on)||isnumeric(on))&&isscalar(on)&&(isequal(on,0)||isequal(on,1)), ...
    'TRAJECTORY_CERTIFICATE:Tube','Tube toggle must be 0 or 1.');
assert(isnumeric(options.max_subdivision_depth)&&isscalar(options.max_subdivision_depth)&& ...
    isfinite(options.max_subdivision_depth)&&options.max_subdivision_depth>=0&& ...
    fix(options.max_subdivision_depth)==options.max_subdivision_depth && ...
    isnumeric(options.numerical_guard_m)&&isscalar(options.numerical_guard_m)&& ...
    isfinite(options.numerical_guard_m)&&options.numerical_guard_m>=0, ...
    'TRAJECTORY_CERTIFICATE:Budget','Caller must provide subdivision budget and numeric guard.');
if on
    assert(isfield(options,'uncertainty_bounds_m') && ...
        isstruct(options.uncertainty_bounds_m)&& ...
        all(isfield(options.uncertainty_bounds_m,required)), ...
        'TRAJECTORY_CERTIFICATE:Uncertainty','Tube ON requires external componentwise bounds.');
    for j=1:3
        v=options.uncertainty_bounds_m.(required{j});
        assert(isnumeric(v)&&isreal(v)&&isscalar(v)&&isfinite(v)&&v>=0, ...
            'TRAJECTORY_CERTIFICATE:Uncertainty','Invalid supplied tube radius.');
        radius.(required{j})=radius.(required{j})+v;
    end
end
ids=zeros(1,numel(observed));
shapes=cell(1,numel(observed));
for i=1:numel(observed)
    o=observed(i);
    assert(isfield(o,'id')&&isnumeric(o.id)&&isscalar(o.id)&&isfinite(o.id)&& ...
        o.id>=1&&o.id==fix(o.id), ...
        'TRAJECTORY_CERTIFICATE:ObservedID','Observed obstacle must have valid ID.');
    ids(i)=double(o.id);
    shapes{i}=prepare_obstacle(o);
end
assert(numel(unique(ids))==numel(ids), ...
    'TRAJECTORY_CERTIFICATE:ObservedID','Duplicate observed obstacle IDs.');
[ground_active,trajectory_phase]=TRAJECTORY_GROUND_SCOPE(options);
if ground_active
    ground=GROUND_INFORMATION.definition();
    assert(strcmp(ground.type,'ground_halfspace')&& ...
        isequal(ground.normal_world,[0;0;1])&&ground.height_m==0&& ...
        strcmp(ground.occupied_side,'negative'), ...
        'TRAJECTORY_CERTIFICATE:Ground','Unsupported ground definition.');
end
ground_intervals=zeros(0,2);
intervals=zeros(0,2); unresolved_ids=zeros(1,0); unresolved_parts=cell(1,0);
checks=0; nodes=0; split_count=0;
for s=1:numel(segments)
    assert(all(isfield(segments(s),{'uav','payload'})), ...
        'TRAJECTORY_CERTIFICATE:Segment','Need both UAV AND payload polynomial segments.');
    [U,t0,T]=read_piece(segments(s).uav);
    [L,tL,TL]=read_piece(segments(s).payload);
    assert(t0==tL&&T==TL, ...
        'TRAJECTORY_CERTIFICATE:TimeAlignment','UAV and payload polynomial clocks must agree.');
    % The z=0 ground is a permanent halfspace, so it is never sent
    % through a finite obstacle support/sensor interface.
    if ground_active
        [gok,ginterval,gnodes,gsplit]=ground_recursive( ...
            U,L,t0,T,radius,options,0,ground.height_m);
        nodes=nodes+gnodes;split_count=split_count+gsplit;
        checks=checks+3*gnodes;
        if ~gok,ground_intervals=[ground_intervals;ginterval];end %#ok<AGROW>
    end
    for o=1:numel(observed)
        [ok,failed_parts,local_intervals,nc,nn,ns]=check_recursive( ...
            U,L,t0,T,shapes{o},radius,options,0);
        checks=checks+nc;nodes=nodes+nn;split_count=split_count+ns;
        if ~ok
            unresolved_ids(end+1)=ids(o); %#ok<AGROW>
            unresolved_parts{end+1}=failed_parts; %#ok<AGROW>
            intervals=[intervals;local_intervals]; %#ok<AGROW>
        end
    end
end
unresolved_ids=unique(unresolved_ids,'stable');
if isempty(unresolved_ids)&&isempty(ground_intervals)
    status='CERTIFIED_CONTINUOUS_POLYNOMIAL_GEOMETRY';
else,status='UNKNOWN_POSSIBLE_COLLISION_OR_GROUND';end
if on,scope='POLYNOMIAL_WITH_SUPPLIED_UNCERTAINTY_RADII';
else,scope='NOMINAL_POLYNOMIAL_ONLY';end
report=struct('status',status, ...
    'certified',isempty(unresolved_ids)&&isempty(ground_intervals), ...
    'safe_to_commit',false, ... % dynamics, yaw, input, tension and controller gates NOT included
    'scope',scope,'considered_obstacle_ids',ids, ...
    'unresolved_obstacle_ids',unresolved_ids, ...
    'unresolved_time_intervals_s',intervals, ...
    'ground_checked',ground_active, ...
    'ground_certified',ground_active&&isempty(ground_intervals), ...
    'ground_status',ground_status(ground_active,ground_intervals), ...
    'trajectory_phase',trajectory_phase, ...
    'ground_unresolved_intervals_s',ground_intervals, ...
    'unresolved_components',{unresolved_parts}, ...
    'checks',checks,'hull_nodes',nodes,'subdivisions',split_count, ...
    'duration_ms',toc(start_tick)*1000, ...
    'meaning','Continuous-time geometry of supplied UAV+payload Bernstein polynomials only; no dynamical/physical execution proof');
end

function status=ground_status(active,intervals)
if ~active
    status='NOT_APPLICABLE_OUTSIDE_FLIGHT';
elseif isempty(intervals)
    status='CERTIFIED_ALL_TIME_ABOVE_GROUND';
else
    status='UNKNOWN_GROUND_CLEARANCE_NOT_PROVEN';
end
end

function [ok,intervals,nodes,splits]=ground_recursive(U,L,t0,T,radius,opt,level,z0)
% Bernstein convex hull ensures all times are above ground for each body.
% The cable is the whole interpolated segment, hence z is bounded by
% min of BOTH endpoint curves, not a sampled set of cable stations.
minu=min(U(3,:));minl=min(L(3,:));
ok=minu>z0+radius.uav_m+opt.numerical_guard_m&& ...
    minl>z0+radius.payload_m+opt.numerical_guard_m&& ...
    min(minu,minl)>z0+radius.cable_m+opt.numerical_guard_m;
nodes=1;splits=0;intervals=zeros(0,2);
if ok,return;end
if level>=opt.max_subdivision_depth
    intervals=[t0,t0+T];return
end
[U0,U1]=split_mid(U);[L0,L1]=split_mid(L);
[a,ia,na,sa]=ground_recursive(U0,L0,t0,T/2,radius,opt,level+1,z0);
[b,ib,nb,sb]=ground_recursive(U1,L1,t0+T/2,T/2,radius,opt,level+1,z0);
ok=a&&b;intervals=[ia;ib];nodes=nodes+na+nb;splits=1+sa+sb;
end

function [P,t0,T]=read_piece(piece)
assert(isstruct(piece)&&isscalar(piece)&& ...
    all(isfield(piece,{'control_points_m','t0_s','duration_s'})), ...
    'TRAJECTORY_CERTIFICATE:Piece','Expected Bernstein control points and timing.');
P=double(piece.control_points_m);
t0=double(piece.t0_s);T=double(piece.duration_s);
assert(isnumeric(piece.control_points_m)&&isreal(piece.control_points_m)&& ...
    size(P,1)==3&&size(P,2)>=2&&all(isfinite(P(:)))&& ...
    isscalar(t0)&&isfinite(t0)&&isscalar(T)&&isfinite(T)&&T>0, ...
    'TRAJECTORY_CERTIFICATE:Piece','Invalid control geometry or timing.');
end

function [ok,parts,intervals,checks,nodes,ns]=check_recursive(U,L,t0,T,shape,radius,opt,level)
checks=0;nodes=1;ns=0;
part_names={'UAV','CABLE','PAYLOAD'};
hulls={U,[U L],L};
radii=[radius.uav_m radius.cable_m radius.payload_m];
failed=false(1,3);
for k=1:3
    checks=checks+1;
    if ~separated(hulls{k},shape,radii(k),opt.numerical_guard_m)
        failed(k)=true;
    end
end
if ~any(failed)
    ok=true;parts=cell(1,0);intervals=zeros(0,2);return
end
if level>=opt.max_subdivision_depth
    ok=false;parts=part_names(failed);intervals=[t0,t0+T];return
end
[U0,U1]=split_mid(U);[L0,L1]=split_mid(L);
[a,pa,ia,c1,n1,s1]=check_recursive(U0,L0,t0,T/2,shape,radius,opt,level+1);
[b,pb,ib,c2,n2,s2]=check_recursive(U1,L1,t0+T/2,T/2,shape,radius,opt,level+1);
ok=a&&b;parts=unique([pa pb],'stable');intervals=[ia;ib];
checks=checks+c1+c2;nodes=nodes+n1+n2;ns=1+s1+s2;
end

function [L,R]=split_mid(P)
n=size(P,2);A=P;L=zeros(3,n);R=zeros(3,n);
L(:,1)=A(:,1);R(:,n)=A(:,n);
for k=2:n
    A=0.5*(A(:,1:end-1)+A(:,2:end));
    L(:,k)=A(:,1);R(:,n-k+1)=A(:,end);
end
end

function tf=separated(P,shape,r,tol)
% Only TRUE on a strict, checked separating-plane inequality.
W=P-shape.c;
% Strong cheap candidates first: obstacle-local axes, hull centroid, then
% control-point and closest-hull directions. Failure => UNKNOWN.
axes=[shape.R,-shape.R];
ctr=mean(W,2);if norm(ctr)>0,axes=[ctr,axes];end
nP=size(W,2);
% For a near-linear segment, the chord middle is a useful normal witness.
if nP>=2
    mid=0.5*(W(:,1)+W(:,end));
    if norm(mid)>0,axes=[axes mid];end
end
for j=1:size(axes,2)
    if pass_axis(W,shape,r,tol,axes(:,j)),tf=true;return;end
end
% Closest point of hull to centre: only a candidate normal, not a proof.
q=mean(W,2);
for j=1:18
    [~,ix]=min(q.'*W);
    w=W(:,ix);d=w-q;den=d.'*d;
    if den<=0,break;end
    beta=min(1,max(0,-(q.'*d)/den));
    q=q+beta*d;
end
if pass_axis(W,shape,r,tol,q),tf=true;return;end
% On curved hulls, vertex-to-obstacle normals can resolve oblique cases.
for j=1:nP
    if pass_axis(W,shape,r,tol,W(:,j)),tf=true;return;end
end
tf=false;
end

function tf=pass_axis(W,shape,r,tol,n)
nn=norm(n);
if ~(isfinite(nn)&&nn>0),tf=false;return;end
n=n/nn;
h=obstacle_support(shape,n);
% The set of all Bezier points has projection >= min of control points.
% A strict gap separates the WHOLE curve/swept cable for all real times.
tf=min(n.'*W)>h+r+tol;
end

function S=prepare_obstacle(o)
% Accept either the persistent OBSTACLE_INFORMATION schema or sensor's
% prepared observed-only schema. Never read globally defined obstacles.
if isfield(o,'geometry')
    assert(isstruct(o.geometry)&&all(isfield(o.geometry,{'type','dimensions'}))&& ...
        isfield(o,'position')&&isfield(o,'motion')&&strcmp(char(o.motion),'static'), ...
        'TRAJECTORY_CERTIFICATE:Shape','Observed geometry must be static and specified.');
    kind=char(o.geometry.type);d=o.geometry.dimensions;c=o.position;
elseif all(isfield(o,{'type','dimensions','center0','is_stationary'}))
    assert(isequal(o.is_stationary,true), ...
        'TRAJECTORY_CERTIFICATE:Shape','Moving shape not supported.');
    kind=char(o.type);d=o.dimensions;c=o.center0;
else
    error('TRAJECTORY_CERTIFICATE:Shape','Incomplete observed shape.');
end
assert(isfield(o,'rotation'),'TRAJECTORY_CERTIFICATE:Shape','Rotation is required.');
R=double(o.rotation);c=double(c(:));
assert(isequal(size(R),[3 3])&&all(isfinite(R(:)))&& ...
    abs(det(R)-1)<1e-8&&norm(R.'*R-eye(3),'fro')<1e-8&& ...
    isequal(size(c),[3 1])&&all(isfinite(c)), ...
    'TRAJECTORY_CERTIFICATE:Shape','Invalid pose SO(3) or centre.');
% Validate all source dimensions; use actual geometry, never guessed radii.
if any(strcmp(kind,{'sphere','ellipsoid','box','cylinder','capsule', ...
        'cone','frustum','torus','pyramid','regular_prism'}))
    assert(isnumeric(d)&&isreal(d)&&all(isfinite(d(:)))&&all(d(:)>=0), ...
        'TRAJECTORY_CERTIFICATE:Shape','Invalid numeric dimensions.');
    d=double(d(:));
end
switch kind
    case 'sphere',valid=numel(d)==1&&d(1)>0;
    case {'ellipsoid','box'},valid=numel(d)==3&&all(d>0);
    case {'cylinder','cone','pyramid'},valid=numel(d)==2&&all(d>0);
    case 'capsule',valid=numel(d)==2&&d(1)>0;
    case 'frustum',valid=numel(d)==3&&d(1)>0&&d(3)>0;
    case 'torus',valid=numel(d)==2&&d(1)>d(2)&&d(2)>0;
    case 'regular_prism',valid=numel(d)==3&&d(1)>=3&&fix(d(1))==d(1)&&all(d(2:3)>0);
    case 'polygon_prism'
        valid=isstruct(d)&&isscalar(d)&&all(isfield(d,{'outline','height'}));
        if valid
            p=d.outline; h=d.height;
            valid=isnumeric(p)&&isreal(p)&&size(p,2)==2&&size(p,1)>=3&& ...
                all(isfinite(p(:)))&&isnumeric(h)&&isscalar(h)&&isfinite(h)&&h>0;
        end
    case 'polyhedron'
        valid=isstruct(d)&&isscalar(d)&&all(isfield(d,{'vertices','faces'}));
        if valid
            V=d.vertices;F=d.faces;
            valid=isnumeric(V)&&isreal(V)&&size(V,2)==3&&size(V,1)>=4&& ...
                all(isfinite(V(:)))&&isnumeric(F)&&size(F,2)==3&&size(F,1)>=4&& ...
                all(isfinite(F(:)))&&all(F(:)>=1)&&all(F(:)<=size(V,1))&&all(F(:)==fix(F(:)));
        end
    otherwise,valid=false;
end
assert(valid,'TRAJECTORY_CERTIFICATE:Shape','Unsupported or malformed observed shape.');
% The canonical obstacle definition performs additional mesh/polygon checks.
% Reject malformed original shapes rather than extrapolating guessed geometry.
assert(exist('OBSTACLE_SHAPES','class')==8, ...
    'TRAJECTORY_CERTIFICATE:ShapeValidator', ...
    'Canonical OBSTACLE_SHAPES must be on the production MATLAB path.');
OBSTACLE_SHAPES.validate(struct('type',kind,'dimensions',d));
S=struct('kind',kind,'d',d,'R',R,'c',c);
end

function h=obstacle_support(S,n)
% Support of the *convex hull* of the exact observed source shape.
% All normals n are unit world vectors. Translation is in W=P-c.
d=S.d;w=S.R.'*n;
rho=hypot(w(1),w(2));
switch S.kind
    case 'sphere', h=d(1);
    case 'ellipsoid',h=norm(d.*w);
    case 'box',h=sum(d.*abs(w))/2;
    case 'cylinder',h=d(1)*rho+d(2)*abs(w(3))/2;
    case 'capsule',h=d(1)+d(2)*abs(w(3))/2;
    case 'cone',h=max(d(1)*rho-d(2)*w(3)/2,d(2)*w(3)/2);
    case 'frustum',h=max(d(1)*rho-d(3)*w(3)/2,d(2)*rho+d(3)*w(3)/2);
    case 'torus',h=d(1)*rho+d(2);
    case 'pyramid'
        s=d(1)/2;v=[s s -d(2)/2;s -s -d(2)/2; ...
            -s s -d(2)/2;-s -s -d(2)/2;0 0 d(2)/2];
        h=max(v*w);
    case 'regular_prism'
        k=0:(d(1)-1);a=2*pi*k/d(1);
        h=max(d(2)*(cos(a)*w(1)+sin(a)*w(2)))+d(3)*abs(w(3))/2;
    case 'polygon_prism'
        h=max(double(d.outline)*w(1:2))+double(d.height)*abs(w(3))/2;
    case 'polyhedron'
        h=max(double(d.vertices)*w);
    otherwise
        error('TRAJECTORY_CERTIFICATE:Support','Unexpected unsupported shape.');
end
assert(isfinite(h),'TRAJECTORY_CERTIFICATE:Support','Nonfinite shape support.');
end

function R=TRAJECTORY_TAUT_FULLBODY_CERTIFICATE(load_pieces,observed,radii,authority,options)
% TRAJECTORY_TAUT_FULLBODY_CERTIFICATE  Whole-time geometry of a C9 load flat output.
%
% PROOF MODEL (not a physical tracking guarantee):
%   w(t)=g-pL''(t), q(t)=w(t)/||w(t)||,
%   pU(t)=pL(t)-L*q(t), cable(t,s)=pL(t)-s*L*q(t), 0<=s<=1.
% One 23rd-degree load polynomial is handled by exact de Casteljau splitting.
% Multi-piece C9/C7 joins are not certified by this function.
% For each time leaf, the second-derivative Bernstein polygon yields
%       w(t) in ball(c,rho), c=mean(W_i), rho=max_i ||W_i-c||.
% Provided ||c||>rho, q(t) lies within the spherical cone around c/||c||
% of half-angle asin(rho/||c||). Hence
%       ||q(t)-c/||c|||| <= sqrt(2*(1-sqrt(1-(rho/||c||)^2))).
% (True in exact real arithmetic.) Thus the actual taut-load UAV and cable
% are enclosed by a translated LOAD Bernstein hull and a *per-leaf*
% direction error ball, NOT by an all-system sphere or an AABB:
%   UAV    in conv(P_i-L*q0) + ball(L*direction_error)
%   CABLE  in conv([P_i,P_i-L*q0]) + ball(L*direction_error)
%   PAYLOAD in conv(P_i).
% Each inflated hull is tested against every OBSERVED obstacle via the
% common 12-shape support-function geometric certificate. A sufficient
% model tension bound is mL*(||c||-rho)>minimum_tension+mL*guard.
%
% UNKNOWN (never SAFE) is returned on unresolved geometry, invalid tension,
% insufficient caller-owned budget, or nonconforming time/data. Physical
% tracking uncertainty can be supplied through existing robust tube option.
% No HLC commitment, thrust/torque proof, numeric ODE error or WCET claim.
%
% Externally supplied: radii.uav_m/payload_m/cable_m; physical authority
% gravity_world_mps2, cable_length_m, payload_mass_kg, minimum_tension_N,
% acceleration_guard_mps2; options robust_tube_enabled, uncertainty_bounds_m
% if enabled, max_subdivision_depth, max_node_count, numerical_guard_m.
% No arbitrarily assumed physical parameters, margins, or fallback bounds.

tick=tic;
required={'max_subdivision_depth','max_node_count','numerical_guard_m', ...
    'robust_tube_enabled'};
assert(isstruct(options)&&isscalar(options)&&all(isfield(options,required)), ...
    'TRAJECTORY_TAUT_FULLBODY:Options','Explicit proof budget and tube options required.');
for key={'max_subdivision_depth','max_node_count'}
    v=options.(key{1});
    assert(isnumeric(v)&&isreal(v)&&isscalar(v)&&isfinite(v)&& ...
        v>=0&&fix(v)==v,'TRAJECTORY_TAUT_FULLBODY:Budget', ...
        'Proof budgets must be nonnegative finite integers.');
end
assert(options.max_node_count>=1 && ...
    isnumeric(options.numerical_guard_m)&&isreal(options.numerical_guard_m)&& ...
    isscalar(options.numerical_guard_m)&&isfinite(options.numerical_guard_m)&& ...
    options.numerical_guard_m>=0, ...
    'TRAJECTORY_TAUT_FULLBODY:Budget','Need at least one node and nonnegative guard.');
assert((islogical(options.robust_tube_enabled)||isnumeric(options.robust_tube_enabled))&& ...
    isscalar(options.robust_tube_enabled)&& ...
    (isequal(options.robust_tube_enabled,0)||isequal(options.robust_tube_enabled,1)), ...
    'TRAJECTORY_TAUT_FULLBODY:Tube','Tube switch must be 0 or 1.');
phys={'cable_length_m','payload_mass_kg','minimum_tension_N', ...
    'acceleration_guard_mps2','gravity_world_mps2'};
assert(isstruct(authority)&&isscalar(authority)&&all(isfield(authority,phys)), ...
    'TRAJECTORY_TAUT_FULLBODY:Authority','Live cable, mass, tension and gravity required.');
for key={'cable_length_m','payload_mass_kg'}
    v=authority.(key{1});
    assert(isnumeric(v)&&isreal(v)&&isscalar(v)&&isfinite(v)&&v>0, ...
        'TRAJECTORY_TAUT_FULLBODY:Authority','Positive live value required.');
end
for key={'minimum_tension_N','acceleration_guard_mps2'}
    v=authority.(key{1});
    assert(isnumeric(v)&&isreal(v)&&isscalar(v)&&isfinite(v)&&v>=0, ...
        'TRAJECTORY_TAUT_FULLBODY:Authority','Nonnegative external bound required.');
end
g=double(authority.gravity_world_mps2);
assert(isnumeric(authority.gravity_world_mps2)&&isreal(authority.gravity_world_mps2)&& ...
    isequal(size(g),[3,1])&&all(isfinite(g(:))), ...
    'TRAJECTORY_TAUT_FULLBODY:Authority','Gravity must be a finite 3x1 vector.');
keys={'uav_m','payload_m','cable_m'};
assert(isstruct(radii)&&isscalar(radii)&&all(isfield(radii,keys)), ...
    'TRAJECTORY_TAUT_FULLBODY:Radii','Explicit separate body and cable radii required.');
for k=1:numel(keys)
    v=radii.(keys{k});
    assert(isnumeric(v)&&isreal(v)&&isscalar(v)&&isfinite(v)&&v>=0, ...
        'TRAJECTORY_TAUT_FULLBODY:Radii','Invalid external radius.');
end
if options.robust_tube_enabled
    assert(isfield(options,'uncertainty_bounds_m')&& ...
        isstruct(options.uncertainty_bounds_m)&& ...
        all(isfield(options.uncertainty_bounds_m,keys)), ...
        'TRAJECTORY_TAUT_FULLBODY:Tube', ...
        'Tube ON requires live per-component positional bounds.');
    for k=1:numel(keys)
        v=options.uncertainty_bounds_m.(keys{k});
        assert(isnumeric(v)&&isreal(v)&&isscalar(v)&&isfinite(v)&&v>=0, ...
            'TRAJECTORY_TAUT_FULLBODY:Tube','Invalid live tube bound.');
    end
end
assert(isstruct(load_pieces)&&isscalar(load_pieces)&& ...
    isstruct(observed), 'TRAJECTORY_TAUT_FULLBODY:Input', ...
    'One C9 load piece and caller-supplied observed obstacles required. Segment joins need a separate C9 connection proof.');
ids=zeros(1,numel(observed));
for j=1:numel(observed)
    assert(isfield(observed(j),'id')&&isnumeric(observed(j).id)&& ...
        isscalar(observed(j).id)&&isfinite(observed(j).id)&& ...
        observed(j).id>0&&fix(observed(j).id)==observed(j).id, ...
        'TRAJECTORY_TAUT_FULLBODY:ObstacleID','Invalid observed obstacle ID.');
    ids(j)=double(observed(j).id);
end
assert(numel(unique(ids))==numel(ids), ...
    'TRAJECTORY_TAUT_FULLBODY:ObstacleID','Duplicate obstacle IDs.');
R=struct('status','UNKNOWN_NOT_CERTIFIED','certified',false, ...
    'safe_to_commit',false,'scope','IDEAL_TAUT_FLAT_OUTPUT_ALL_TIME', ...
    'considered_obstacle_ids',ids,'checked_leaf_count',0, ...
    'checked_node_count',0,'geometry_calls',0, ...
    'maximum_cable_cone_padding_m',0, ...
    'minimum_model_tension_lower_bound_N',Inf, ...
    'unresolved_time_interval_s',[NaN NaN], ...
    'unresolved_obstacle_ids',zeros(1,0), ...
    'leaf_proofs',struct([]),'elapsed_ms',NaN, ...
    'meaning','Ideal-taut nonlinear flat output and supplied observation geometry; no HLC/ODE/tracking/actuator certificate');
if options.robust_tube_enabled
    R.scope='IDEAL_TAUT_WITH_EXTERNALLY_SUPPLIED_POSITION_TUBES';
end
last_end=NaN;
for k=1:numel(load_pieces)
    p=load_pieces(k);
    assert(all(isfield(p,{'control_points_m','t0_s','duration_s'})), ...
        'TRAJECTORY_TAUT_FULLBODY:Piece','Require Bernstein control points/time.');
    P=double(p.control_points_m);t0=double(p.t0_s);T=double(p.duration_s);
    assert(isnumeric(p.control_points_m)&&isreal(p.control_points_m)&& ...
        isequal(size(P),[3 24])&&all(isfinite(P(:)))&& ...
        isscalar(t0)&&isreal(t0)&&isfinite(t0)&& ...
        isscalar(T)&&isreal(T)&&isfinite(T)&&T>0&& ...
        isfinite(t0+T), 'TRAJECTORY_TAUT_FULLBODY:Piece', ...
        'Expected degree-23 load Bernstein controls and finite positive duration.');
    if k>1
        assert(t0==last_end, ...
            'TRAJECTORY_TAUT_FULLBODY:Clock', ...
            'Piece times must connect without a gap (no time extrapolation).');
    end
    last_end=t0+T;
    % Recompute acceleration directly from the supplied control points.
    % This prevents a stale derivative cache being accepted as a proof.
    W=repmat(g,1,22)-(23*22/T^2)*diff(P,2,2);
    [ok,R]=check_leaf(P,W,t0,T,0,observed,radii,authority,options,R);
    if ~ok
        R.elapsed_ms=toc(tick)*1000;
        return
    end
end
R.certified=true;
R.unresolved_obstacle_ids=zeros(1,0);
R.unresolved_time_interval_s=[NaN NaN];
R.status='CERTIFIED_IDEAL_TAUT_FULLBODY_CONTINUOUS_GEOMETRY';
R.elapsed_ms=toc(tick)*1000;
end

function [ok,R]=check_leaf(P,W,t0,T,depth,observed,radii,A,opt,R)
ok=false;
if R.checked_node_count>=opt.max_node_count
    R.status='UNKNOWN_PROOF_NODE_BUDGET';
    R.unresolved_time_interval_s=[t0 t0+T];return
end
R.checked_node_count=R.checked_node_count+1;
c=mean(W,2);s=norm(c);
rho=max(sqrt(sum((W-c).^2,1)));
minacc=A.minimum_tension_N/A.payload_mass_kg+A.acceleration_guard_mps2;
taut=isfinite(s)&&isfinite(rho)&&s>rho&&s-rho>minacc;
if taut
    ratio=rho/s;
    % Stable identity: chord = ratio*sqrt(2/(1+sqrt(1-ratio^2))).
    % It equals sqrt(2*(1-sqrt(1-ratio^2))) without cancellation.
    angle_chord=ratio*sqrt(2/(1+sqrt(max(0,1-ratio^2))));
    extra=A.cable_length_m*angle_chord;
    q0=c/s;
    inflated=radii;
    inflated.uav_m=radii.uav_m+extra;
    inflated.cable_m=radii.cable_m+extra;
    fakeU=P-A.cable_length_m*q0;
    seg=struct('uav',struct('t0_s',t0,'duration_s',T, ...
        'control_points_m',fakeU), ...
        'payload',struct('t0_s',t0,'duration_s',T, ...
        'control_points_m',P));
    pass_options=opt;
    pass_options.max_subdivision_depth=0;
    R.geometry_calls=R.geometry_calls+1;
    G=TRAJECTORY_CONTINUOUS_GEOMETRY_CERTIFICATE( ...
        seg,observed,inflated,pass_options);
    if G.certified
        R.checked_leaf_count=R.checked_leaf_count+1;
        R.minimum_model_tension_lower_bound_N=min( ...
            R.minimum_model_tension_lower_bound_N, ...
            A.payload_mass_kg*(s-rho));
        R.maximum_cable_cone_padding_m=max( ...
            R.maximum_cable_cone_padding_m,extra);
        proof=struct('time_interval_s',[t0,t0+T], ...
            'cable_direction_center',q0, ...
            'direction_chord_max',angle_chord, ...
            'additional_position_enclosure_m',extra, ...
            'tension_lower_bound_N',A.payload_mass_kg*(s-rho));
        if isempty(R.leaf_proofs),R.leaf_proofs=proof;
        else,R.leaf_proofs(end+1)=proof;end %#ok<AGROW>
        ok=true;return
    end
    R.unresolved_obstacle_ids=G.unresolved_obstacle_ids;
else
    R.status='UNKNOWN_TAUT_TENSION_CONE';
end
if depth>=opt.max_subdivision_depth
    if taut,R.status='UNKNOWN_GEOMETRY_NOT_SEPARATED';end
    R.unresolved_time_interval_s=[t0,t0+T];return
end
[Pleft,Pright]=split_mid(P);
[Wleft,Wright]=split_mid(W);
[ok,R]=check_leaf(Pleft,Wleft,t0,T/2,depth+1, ...
    observed,radii,A,opt,R);
if ~ok,return;end
[ok,R]=check_leaf(Pright,Wright,t0+T/2,T/2,depth+1, ...
    observed,radii,A,opt,R);
end

function [left,right]=split_mid(P)
n=size(P,2);Q=P;left=zeros(size(P));right=zeros(size(P));
left(:,1)=P(:,1);right(:,n)=P(:,n);
for j=2:n
    Q=0.5*(Q(:,1:end-1)+Q(:,2:end));
    left(:,j)=Q(:,1);right(:,n-j+1)=Q(:,end);
end
end

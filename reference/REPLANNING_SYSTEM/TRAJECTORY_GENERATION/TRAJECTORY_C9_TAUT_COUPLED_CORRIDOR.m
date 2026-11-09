function C=TRAJECTORY_C9_TAUT_COUPLED_CORRIDOR(guide,observed,radii,authority,proof_options,corridor_options)
% TRAJECTORY_C9_TAUT_COUPLED_CORRIDOR
% Construct a whole-time, IDEAL-TAUT, PHYSICALLY COUPLED local convex
% corridor around a previously certified 23rd-degree C9 payload guide.
%
% The payload Bernstein polygon is P; its acceleration polygon yields
% W_i = gravity - pL''_i.  In each certified time leaf constrain every
% candidate acceleration controls W_i to a variable radius R(alpha)
% around the FIXED guide center c.  If ||c||-R(alpha) exceeds the live
% tension threshold, q=W/||W|| belongs to the corresponding spherical
% cone around q0=c/||c||, with chord error d(R(alpha)).  UAV=pL-L*q;
% the entire cable joins pL to pL-L*q.  Direction-neutral support-function
% halfspaces constrain each SUBDIVIDED payload polygon and both endpoints,
% with exactly the additional cone inflation demanded by R(alpha).
% Hence candidates satisfying these constraints are geometrically safe
% in the ideal-taut flat-output model (and given positional tubes).
%
% No fixed bypass direction, obstacle-specific planner, AABB or whole-
% system 2m sphere. The cone is adaptive, not preinflated to worst case.
% The only inputs used are current certified geometry,
% the user's physical parameters and explicit proof budgets.  An unsafe
% guide or failed corridor yields UNKNOWN (never a candidate).
%
% C is NOT a physical HLC tracking/actuator guarantee or HLC commit token.
tick=tic;
need={'min_separation_m','max_normal_iterations','max_ray_iterations'};
assert(isstruct(corridor_options)&&isscalar(corridor_options)&& ...
    all(isfield(corridor_options,need)), ...
    'TRAJECTORY_C9_COUPLED:Options','Require a caller-owned strict gap and normal search budget.');
assert(isnumeric(corridor_options.min_separation_m)&& ...
    isreal(corridor_options.min_separation_m)&& ...
    isscalar(corridor_options.min_separation_m)&& ...
    isfinite(corridor_options.min_separation_m)&& ...
    corridor_options.min_separation_m>0 && ...
    isnumeric(corridor_options.max_normal_iterations)&& ...
    isscalar(corridor_options.max_normal_iterations)&& ...
    isfinite(corridor_options.max_normal_iterations)&& ...
    corridor_options.max_normal_iterations>=1&& ...
    fix(corridor_options.max_normal_iterations)==corridor_options.max_normal_iterations && ...
    isnumeric(corridor_options.max_ray_iterations)&& ...
    isscalar(corridor_options.max_ray_iterations)&& ...
    isfinite(corridor_options.max_ray_iterations)&& ...
    corridor_options.max_ray_iterations>=1&& ...
    fix(corridor_options.max_ray_iterations)==corridor_options.max_ray_iterations, ...
    'TRAJECTORY_C9_COUPLED:Options','Invalid positive user-supplied gap/budget.');
assert(isstruct(guide)&&isscalar(guide)&& ...
    all(isfield(guide,{'control_points_m','t0_s','duration_s'}))&& ...
    isequal(size(guide.control_points_m),[3,24])&& ...
    all(isfield(guide,{'start_jets','end_jets'}))&& ...
    isfield(authority,'gravity_world_mps2')&& ...
    isfield(authority,'cable_length_m')&& ...
    isfield(authority,'payload_mass_kg')&& ...
    isfield(authority,'minimum_tension_N')&& ...
    isfield(authority,'acceleration_guard_mps2'), ...
    'TRAJECTORY_C9_COUPLED:Input','Require full C9 guide and live physical authority.');
[ground_active,trajectory_phase]=TRAJECTORY_GROUND_SCOPE(proof_options);
if ground_active
    ground_snapshot=GROUND_INFORMATION.definition();
else
    ground_snapshot=struct([]);
end
C=struct('status','UNKNOWN_GUIDE_NOT_CERTIFIED', ...
    'certified',false,'safe_to_commit',false, ...
    'scope','IDEAL_TAUT_C9_FIXED_CONE_LOCAL_CORRIDOR', ...
    'leaves',struct([]),'observed_snapshot',{observed}, ...
    'guide_piece',guide,'radii',radii,'authority',authority, ...
    'ground_snapshot',ground_snapshot, ...
    'ground_applicable',ground_active,'trajectory_phase',trajectory_phase, ...
    'proof_options',proof_options,'corridor_options',corridor_options, ...
    'obstacle_ids',zeros(1,0),'elapsed_ms',NaN, ...
    'minimum_model_tension_bound_N',NaN, ...
    'maximum_cone_padding_m',NaN, ...
    'unresolved_time_interval_s',[NaN NaN]);
G=TRAJECTORY_TAUT_FULLBODY_CERTIFICATE(guide,observed,radii,authority,proof_options);
C.obstacle_ids=G.considered_obstacle_ids;
if ~G.certified
    C.status=['UNKNOWN_GUIDE_' G.status];C.elapsed_ms=toc(tick)*1000;return
end
C.minimum_model_tension_bound_N=G.minimum_model_tension_lower_bound_N;
C.maximum_cone_padding_m=G.maximum_cable_cone_padding_m;
P0=double(guide.control_points_m);
tstart=double(guide.t0_s);T0=double(guide.duration_s);
g=double(authority.gravity_world_mps2(:));L=double(authority.cable_length_m);
minw=double(authority.minimum_tension_N)/double(authority.payload_mass_kg)+ ...
    double(authority.acceleration_guard_mps2);
leaves=struct([]);
for k=1:numel(G.leaf_proofs)
    witness=G.leaf_proofs(k);
    ta=witness.time_interval_s(1);tb=witness.time_interval_s(2);
    t=(ta-tstart)/T0;u=(tb-tstart)/T0;
    M=interval_map(24,t,u);
    Pl=P0*M;dt=tb-ta;
    W=repmat(g,1,22)-(23*22/dt^2)*diff(Pl,2,2);
    c=mean(W,2);s=norm(c);
    rho0=max(sqrt(sum((W-c).^2,1)));
    if ~(s>rho0&&s-rho0>minw)
        C.status='UNKNOWN_LEAF_TENSION';C.elapsed_ms=toc(tick)*1000;
        C.unresolved_time_interval_s=[ta tb];return
    end
    q0=c/s;
    extra0=L*chord(rho0/s);
    inflated=radii;
    inflated.uav_m=inflated.uav_m+extra0;
    inflated.cable_m=inflated.cable_m+extra0;
    seg=struct('uav',struct('control_points_m',Pl-L*q0, ...
        't0_s',ta,'duration_s',dt), ...
        'payload',struct('control_points_m',Pl, ...
        't0_s',ta,'duration_s',dt));
    opts=struct('robust_tube_enabled',proof_options.robust_tube_enabled, ...
        'min_separation_m',corridor_options.min_separation_m, ...
        'numerical_guard_m',proof_options.numerical_guard_m, ...
        'max_subdivision_depth',0,'max_leaf_count',1, ...
        'max_normal_iterations',corridor_options.max_normal_iterations);
    if proof_options.robust_tube_enabled
        opts.uncertainty_bounds_m=proof_options.uncertainty_bounds_m;
    end
    if isfield(proof_options,'trajectory_phase')
        opts.trajectory_phase=proof_options.trajectory_phase;
    end
    base=TRAJECTORY_SAFE_CORRIDOR(seg,observed,inflated,opts);
    if ~(base.certified&&numel(base.leaves)==1)
        C.status='UNKNOWN_FIXED_HALFSPACE_NOT_SEPARABLE';
        C.unresolved_time_interval_s=[ta tb];C.elapsed_ms=toc(tick)*1000;return
    end
    b=base.leaves(1);
    % This plane system is valid for ANY other C9 payload polygon Pnew
    % if the same fixed normals separate its load/control proxies after
    % EXACTLY the extra cone inflation required by candidate acceleration.
    % No fixed maximum cone is selected here (which would consume guide
    % clearance even when the candidate hardly changes its acceleration).
    if ~halfspaces_pass(Pl,q0,L,b,0)
        C.status='UNKNOWN_GUIDE_OUTSIDE_FIXED_HALFSPACES';
        C.unresolved_time_interval_s=[ta tb];C.elapsed_ms=toc(tick)*1000;return
    end
    leaf=struct('time_interval_s',[ta tb],'map',M, ...
        'acceleration_center_mps2',c, ...
        'guide_acceleration_radius_mps2',rho0, ...
        'guide_acceleration_controls_mps2',W, ...
        'gravity_minus_load_acceleration_center_mps2',c, ...
        'q0',q0,'guide_cone_padding_m',extra0, ...
        'guide_tension_lower_bound_N', ...
        authority.payload_mass_kg*(s-rho0), ...
        'uav_normals',b.uav_normals,'uav_offsets',b.uav_offsets, ...
        'payload_normals',b.payload_normals,'payload_offsets',b.payload_offsets, ...
        'cable_normals',b.cable_normals,'cable_offsets',b.cable_offsets);
    if isempty(leaves),leaves=leaf;else,leaves(end+1)=leaf;end %#ok<AGROW>
end
C.leaves=leaves;C.certified=true;
C.status='CERTIFIED_C9_ADAPTIVE_CONE_CORRIDOR_TEMPLATE__NOT_HLC';
C.elapsed_ms=toc(tick)*1000;
end

function yes=halfspaces_pass(P,q,L,b,tol)
yes=true;
if ~isempty(b.uav_offsets)
    yes=yes && all(min(b.uav_normals.'*(P-L*q),[],2)>=b.uav_offsets-tol);
end
if ~isempty(b.payload_offsets)
    yes=yes && all(min(b.payload_normals.'*P,[],2)>=b.payload_offsets-tol);
end
if ~isempty(b.cable_offsets)
    yes=yes && all(min(b.cable_normals.'*(P-L*q),[],2)>=b.cable_offsets-tol) && ...
        all(min(b.cable_normals.'*P,[],2)>=b.cable_offsets-tol);
end
end

function c=chord(v)
assert(v>=0&&v<1,'TRAJECTORY_C9_COUPLED:Cone','Invalid cone ratio.');
c=v*sqrt(2/(1+sqrt(max(0,1-v*v))));
end

function M=interval_map(n,a,b)
% Degree-(n-1) Bernstein control map for [a,b] within [0,1].
assert(a>=0&&b<=1&&b>a&&isfinite(a+b), ...
    'TRAJECTORY_C9_COUPLED:Time','Invalid certified subinterval.');
M=eye(n);
if a>0
    [~,M]=split(M,a);
end
if b<1
    [M,~]=split(M,(b-a)/(1-a));
end
end
function [X,Y]=split(P,t)
n=size(P,2);Q=P;X=zeros(size(P));Y=zeros(size(P));
X(:,1)=Q(:,1);Y(:,end)=Q(:,end);
for j=2:n
    Q=(1-t)*Q(:,1:end-1)+t*Q(:,2:end);
    X(:,j)=Q(:,1);Y(:,n-j+1)=Q(:,end);
end
end

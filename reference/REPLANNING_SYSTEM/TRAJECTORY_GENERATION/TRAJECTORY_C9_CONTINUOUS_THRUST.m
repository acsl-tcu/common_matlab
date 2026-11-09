function R=TRAJECTORY_C9_CONTINUOUS_THRUST(load_piece,authority,proof_options)
% TRAJECTORY_C9_CONTINUOUS_THRUST
% ALL-TIME SUFFICIENT certificate for ideal-taut C9 LOAD-flat-output
% thrust magnitude AND positive cable tension. Does not certify attitude,
% torques, HLC tracking or actual plant execution; never authorizes commit.
%
% Model: w=g-pL'', q=w/||w|| (quad -> payload), pU=pL-L*q.
% F=mU*(pU''-g)+mL*(pL''-g)
%   =-(mU+mL)*w-mU*L*q''.
% Let w(t) lie in B(c,rho) from Bernstein acceleration controls, with
% s_min=||c||-rho>0. Bernstein derivative controls give ||w'||<=D1,
% ||w''||<=D2 over the ENTIRE interval. Differentiating q=w/||w|| gives
% ||q''|| <= D2/s_min + 3*(D1/s_min)^2 := Q2.
% Consequently over the whole interval,
%   max(0,(mU+mL)*s_min-mU*L*Q2) <= ||F(t)||
%   <= (mU+mL)*(||c||+rho)+mU*L*Q2.
% This bound is conservative but mathematically valid in real arithmetic.
% Split control polygons exactly with de Casteljau if the interval cannot
% be certified. Budget/force guard and ALL physical parameters are CALLER
% supplied: no heuristic fallback, no sampled-only safety claim.
%
% The externally supplied payload mass is read each call; old certificate
% snapshots must NOT be reused after it changes. This is NOT a certificate
% for the (as-yet-unimplemented) full attitude/torque feasibility.
start=tic;
phys={'cable_length_m','uav_mass_kg','payload_mass_kg', ...
    'gravity_world_mps2','thrust_limits_N','minimum_tension_N', ...
    'acceleration_guard_mps2'};
assert(isstruct(authority)&&isscalar(authority)&&all(isfield(authority,phys)), ...
    'TRAJECTORY_C9_THRUST:Authority','Live taut model and thrust authority required.');
for key={'cable_length_m','uav_mass_kg','payload_mass_kg'}
    v=authority.(key{1});
    assert(isnumeric(v)&&isreal(v)&&isscalar(v)&&isfinite(v)&&v>0, ...
        'TRAJECTORY_C9_THRUST:Authority','Live masses/length must be positive.');
end
for key={'minimum_tension_N','acceleration_guard_mps2'}
    v=authority.(key{1});
    assert(isnumeric(v)&&isreal(v)&&isscalar(v)&&isfinite(v)&&v>=0, ...
        'TRAJECTORY_C9_THRUST:Authority','Live guard/tension must be nonnegative.');
end
assert(isnumeric(authority.gravity_world_mps2)&& ...
    isreal(authority.gravity_world_mps2)&& ...
    isequal(size(authority.gravity_world_mps2),[3 1])&& ...
    all(isfinite(authority.gravity_world_mps2(:)))&& ...
    isnumeric(authority.thrust_limits_N)&&isreal(authority.thrust_limits_N)&& ...
    numel(authority.thrust_limits_N)==2&& ...
    all(isfinite(authority.thrust_limits_N(:)))&& ...
    authority.thrust_limits_N(1)>=0&& ...
    authority.thrust_limits_N(2)>=authority.thrust_limits_N(1), ...
    'TRAJECTORY_C9_THRUST:Authority','Invalid gravity or thrust limits.');
req={'max_subdivision_depth','max_node_count','force_guard_N'};
assert(isstruct(proof_options)&&isscalar(proof_options)&& ...
    all(isfield(proof_options,req)), ...
    'TRAJECTORY_C9_THRUST:Options','Caller must supply budgets and force guard.');
for key={'max_subdivision_depth','max_node_count'}
    v=proof_options.(key{1});
    assert(isnumeric(v)&&isreal(v)&&isscalar(v)&&isfinite(v)&& ...
        v>=0&&fix(v)==v,'TRAJECTORY_C9_THRUST:Budget','Invalid proof budget.');
end
assert(proof_options.max_node_count>=1 && ...
    isnumeric(proof_options.force_guard_N)&& ...
    isreal(proof_options.force_guard_N)&& ...
    isscalar(proof_options.force_guard_N)&& ...
    isfinite(proof_options.force_guard_N)&&proof_options.force_guard_N>=0, ...
    'TRAJECTORY_C9_THRUST:Guard','Need nonnegative caller-owned N guard.');
assert(isstruct(load_piece)&&isscalar(load_piece)&& ...
    all(isfield(load_piece,{'control_points_m','t0_s','duration_s'}))&& ...
    isnumeric(load_piece.control_points_m)&& ...
    isreal(load_piece.control_points_m)&& ...
    isequal(size(load_piece.control_points_m),[3 24])&& ...
    all(isfinite(load_piece.control_points_m(:)))&& ...
    isnumeric(load_piece.t0_s)&&isreal(load_piece.t0_s)&& ...
    isscalar(load_piece.t0_s)&&isfinite(load_piece.t0_s)&& ...
    isnumeric(load_piece.duration_s)&&isreal(load_piece.duration_s)&& ...
    isscalar(load_piece.duration_s)&&isfinite(load_piece.duration_s)&& ...
    load_piece.duration_s>0, ...
    'TRAJECTORY_C9_THRUST:Piece','Need one finite degree-23 C9 payload piece.');
R=struct('status','UNKNOWN_NOT_CERTIFIED', ...
    'certified',false,'certified_continuous_thrust',false, ...
    'certified_continuous_tension',false,'safe_to_commit',false, ...
    'checked_node_count',0,'checked_leaf_count',0, ...
    'minimum_tension_lower_bound_N',Inf, ...
    'thrust_lower_bound_N',Inf,'thrust_upper_bound_N',0, ...
    'unresolved_time_interval_s',[NaN NaN], ...
    'leaf_bounds',struct([]),'elapsed_ms',NaN, ...
    'authority_snapshot',authority,'scope', ...
    'IDEAL_TAUT_ALL_TIME_FORCE_MAGNITUDE_ONLY__NO_TORQUE_OR_HLC');
[good,R]=check_leaf(double(load_piece.control_points_m), ...
    double(load_piece.t0_s),double(load_piece.duration_s),0, ...
    authority,proof_options,R);
R.certified=good;
R.certified_continuous_thrust=good;
R.certified_continuous_tension=good;
if good
    R.status='CERTIFIED_IDEAL_TAUT_ALL_TIME_THRUST_AND_TENSION__NO_TORQUE';
    R.unresolved_time_interval_s=[NaN NaN];
end
R.elapsed_ms=toc(start)*1000;
end

function [good,R]=check_leaf(P,t0,dt,depth,A,opt,R)
good=false;
if R.checked_node_count>=opt.max_node_count
    R.status='UNKNOWN_PROOF_NODE_BUDGET';
    R.unresolved_time_interval_s=[t0 t0+dt];return
end
R.checked_node_count=R.checked_node_count+1;
g=double(A.gravity_world_mps2(:));
% The controls for w,w',w'' are DERIVED from the same source P on EACH leaf.
W=repmat(g,1,22)-(23*22/dt^2)*diff(P,2,2);
W1=-(23*22*21/dt^3)*diff(P,3,2);
W2=-(23*22*21*20/dt^4)*diff(P,4,2);
c=mean(W,2);sc=norm(c);
rho=max(sqrt(sum((W-c).^2,1)));
low=sc-rho;
D1=max(sqrt(sum(W1.^2,1)));
D2=max(sqrt(sum(W2.^2,1)));
min_required=A.minimum_tension_N/A.payload_mass_kg+ ...
    A.acceleration_guard_mps2;
finite=all(isfinite([sc,rho,low,D1,D2]));
if finite&&low>min_required
    qdd_max=D2/low+3*(D1/low)^2;
    M=double(A.uav_mass_kg)+double(A.payload_mass_kg);
    f_margin=double(A.uav_mass_kg)*double(A.cable_length_m)*qdd_max;
    f_lo=max(0,M*low-f_margin);
    f_hi=M*(sc+rho)+f_margin;
    Tmin=double(A.payload_mass_kg)*low;
    R.minimum_tension_lower_bound_N=min( ...
        R.minimum_tension_lower_bound_N,Tmin);
    R.thrust_lower_bound_N=min(R.thrust_lower_bound_N,f_lo);
    R.thrust_upper_bound_N=max(R.thrust_upper_bound_N,f_hi);
    limits=double(A.thrust_limits_N(:));
    guard=double(opt.force_guard_N);
    passed=isfinite(f_lo)&&isfinite(f_hi)&& ...
        f_lo>=limits(1)+guard && f_hi<=limits(2)-guard;
    if passed
        proof=struct('time_interval_s',[t0,t0+dt], ...
            'tension_lower_bound_N',Tmin, ...
            'thrust_lower_bound_N',f_lo, ...
            'thrust_upper_bound_N',f_hi, ...
            'q_second_derivative_upper_bound_s2',qdd_max);
        if isempty(R.leaf_bounds)
            R.leaf_bounds=proof;
        else
            R.leaf_bounds(end+1)=proof; %#ok<AGROW>
        end
        R.checked_leaf_count=R.checked_leaf_count+1;
        good=true;return
    end
    R.status='UNKNOWN_THRUST_BOUND_NOT_PROVEN';
else
    R.status='UNKNOWN_TENSION_BOUND_NOT_PROVEN';
end
if depth>=opt.max_subdivision_depth
    R.unresolved_time_interval_s=[t0 t0+dt];return
end
[Pl,Pr]=half(P);
[yes,R]=check_leaf(Pl,t0,dt/2,depth+1,A,opt,R);
if ~yes,return;end
[good,R]=check_leaf(Pr,t0+dt/2,dt/2,depth+1,A,opt,R);
end

function [L,R]=half(P)
% Degree-23 de Casteljau at exactly 1/2, preserving real-arithmetic hulls.
n=size(P,2);Q=P;L=zeros(size(P));R=zeros(size(P));
L(:,1)=Q(:,1);R(:,end)=Q(:,end);
for k=2:n
    Q=0.5*(Q(:,1:end-1)+Q(:,2:end));
    L(:,k)=Q(:,1);R(:,n-k+1)=Q(:,end);
end
end

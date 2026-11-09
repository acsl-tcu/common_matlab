function R=TRAJECTORY_C9_TAUT_CORRIDOR_RAY(C,target_free,observed,radii,authority,proof_options,force_options)
% TRAJECTORY_C9_TAUT_CORRIDOR_RAY
% Fast ONE-DIMENSIONAL, coupled C9 refinement in a certified template.
% alpha in [0,1]: free(alpha)=guide_free+alpha*(target_free-guide_free).
%
% Each fixed obstacle plane evaluates the *entire* subdivided load Bezier
% polygon.  The cone inflation is recomputed from the ACTUAL candidate
% acceleration control points, not the far larger worst-case cable sphere.
% All acceleration controls are contained in B(c,rho(alpha)). The exact
% cone bound follows if norm(c)>rho(alpha) and the live tension lower bound
% is positive.  Thus UAV/load/cable halfspaces form a sufficient all-time
% noncollision certificate for the ideal taut-load model.
%
% The feasible alpha set containing zero is an interval: Bernstein
% halfspace margins are affine in alpha, and rho(alpha)=max_i||a_i+alpha*b_i||
% is convex; the cone chord function is increasing and convex on [0,1).
% Bisection uses only explicit caller-owned iteration budget; no slow NLP.
% A final INDEPENDENT full-body certificate is ALWAYS evaluated.
% THIS DOES NOT CERTIFY HLC/plant execution, tracking or torques.
if nargin<7,force_options=[];end
tick=tic;
R=struct('status','UNKNOWN_NO_CERTIFIED_CORRIDOR', ...
    'candidate_available',false,'certified_ideal_taut_geometry',false, ...
    'safe_to_commit',false,'ray_fraction',NaN, ...
    'piece',struct(),'fullbody_certificate',struct(), ...
    'elapsed_ms',NaN,'constraints_checked',0, ...
    'ray_evaluation_count',0, ...
    'reason','Actuator torque/HLC model and tracking are not certified');
assert(isstruct(C)&&isscalar(C)&&isfield(C,'certified'), ...
    'TRAJECTORY_C9_RAY:Corridor','Require a certified C9 corridor.');
if ~C.certified,R.elapsed_ms=toc(tick)*1000;return;end
assert(isnumeric(target_free)&&isreal(target_free)&& ...
    isequal(size(target_free),[3 4])&&all(isfinite(target_free(:))), ...
    'TRAJECTORY_C9_RAY:Target','Require finite 3x4 free controls.');
[ground_active,trajectory_phase]=TRAJECTORY_GROUND_SCOPE(proof_options);
if ground_active
    expected_ground=GROUND_INFORMATION.definition();
else
    expected_ground=struct([]);
end
if ~isfield(C,'ground_snapshot')|| ...
   ~isfield(C,'ground_applicable')|| ...
   ~isequal(C.ground_applicable,ground_active)|| ...
   ~isfield(C,'trajectory_phase')|| ...
   ~strcmp(C.trajectory_phase,trajectory_phase)|| ...
   ~isequaln(C.ground_snapshot,expected_ground)|| ...
   ~isequaln(C.observed_snapshot,observed)|| ...
   ~isequaln(C.radii,radii)||~isequaln(C.authority,authority)|| ...
   ~isequaln(C.proof_options,proof_options)
    R.status='UNKNOWN_STALE_OBSTACLES_OR_PHYSICAL_AUTHORITY';
    R.elapsed_ms=toc(tick)*1000;return
end
P0=double(C.guide_piece.control_points_m);
P1=P0;P1(:,11:14)=double(target_free);
DP=P1-P0;
g=double(authority.gravity_world_mps2(:));
L=double(authority.cable_length_m);
minw=double(authority.minimum_tension_N)/ ...
    double(authority.payload_mass_kg)+double(authority.acceleration_guard_mps2);
% One-time precompute of all candidate-affine Bernstein controls.
leaves=C.leaves;
ground=C.ground_snapshot;
grad=[radii.uav_m,radii.payload_m,radii.cable_m];
if proof_options.robust_tube_enabled
    tub=proof_options.uncertainty_bounds_m;
    grad=grad+[tub.uav_m,tub.payload_m,tub.cable_m];
end
grad=double(grad);
ground_guard=double(proof_options.numerical_guard_m);
n=numel(leaves);
Pg=cell(1,n);DPc=cell(1,n);Wg=cell(1,n);DWc=cell(1,n);
for k=1:n
    leaf=leaves(k);
    Pg{k}=P0*leaf.map;
    DPc{k}=DP*leaf.map;
    scale=23*22/(diff(leaf.time_interval_s)^2);
    Wg{k}=repmat(g,1,22)-scale*diff(Pg{k},2,2);
    DWc{k}=-scale*diff(DPc{k},2,2);
end
[yes,count]=feasible(1,leaves,Pg,DPc,Wg,DWc,L,minw,ground,grad,ground_guard,ground_active);
checks=count;evals=1;
if yes
    alpha=1;
else
    lower=0;upper=1;
    for k=1:C.corridor_options.max_ray_iterations
        mid=(lower+upper)*0.5;
        [yes,count]=feasible(mid,leaves,Pg,DPc,Wg,DWc,L,minw,ground,grad,ground_guard,ground_active);
        checks=checks+count;evals=evals+1;
        if yes
            lower=mid;
        else
            upper=mid;
        end
    end
    % The lower endpoint is always INSIDE the corridor, not an upper
    % estimate. This retreat is purely a floating-arithmetic guard.
    alpha=max(0,lower-16*eps(max(1,lower)));
end
R.ray_fraction=alpha;
R.constraints_checked=checks;
R.ray_evaluation_count=evals;
candidate=TRAJECTORY_C9_FLAT_BERNSTEIN.build( ...
    C.guide_piece.t0_s,C.guide_piece.duration_s, ...
    C.guide_piece.start_jets,C.guide_piece.end_jets, ...
    P0(:,11:14)+alpha*(target_free-P0(:,11:14)));
% Independent proof uses actual candidate derivatives, not any cached
% guide acceleration, and checks the FULL observed/retained shape array.
if isempty(force_options)
    % Legacy geometry-only independent check (existing callers unchanged).
    final=TRAJECTORY_TAUT_FULLBODY_CERTIFICATE( ...
        candidate,observed,radii,authority,proof_options);
else
    % Combined all-time geometry + taut tension + thrust proof.
    % No reliance on the corridor's earlier guide certificate.
    final=TRAJECTORY_C9_JOINT_FAST_CERTIFICATE( ...
        candidate,observed,radii,authority,proof_options,force_options);
end
R.fullbody_certificate=final;
if final.certified
    R.candidate_available=true;
    R.certified_ideal_taut_geometry=true;
    R.piece=candidate;
    if isempty(force_options)
        R.status='CERTIFIED_C9_RAY_IDEAL_TAUT_ALL_TIME__HLC_NOT_CERTIFIED';
    else
        R.status='CERTIFIED_C9_RAY_GEOMETRY_TENSION_THRUST__TORQUE_HLC_PENDING';
    end
else
    R.status=['UNKNOWN_FINAL_INDEPENDENT_CERTIFICATE_' final.status];
end
R.elapsed_ms=toc(tick)*1000;
end

function [yes,checks]=feasible(alpha,leaves,Pg,DPc,Wg,DWc,L,minw,ground,grad,ground_guard,ground_active)
yes=true;checks=0;
for k=1:numel(leaves)
    leaf=leaves(k);
    P=Pg{k}+alpha*DPc{k};
    W=Wg{k}+alpha*DWc{k};
    c=leaf.acceleration_center_mps2;
    s=norm(c);
    rho=max(sqrt(sum((W-c).^2,1)));
    checks=checks+22;
    if ~(isfinite(rho)&&s>rho&&s-rho>minw)
        yes=false;return
    end
    extra=L*chord(rho/s);
    extra_delta=extra-leaf.guide_cone_padding_m;
    q=leaf.q0;
    if ground_active
        [groundok,~]=TRAJECTORY_C9_GROUND_LEAF( ...
            P,q,extra,L,grad,ground,ground_guard);
        checks=checks+3;
        if ~groundok,yes=false;return;end
    end
    if ~isempty(leaf.uav_offsets)
        slack=leaf.uav_normals.'*(P-L*q)- ...
            (leaf.uav_offsets+extra_delta);
        checks=checks+numel(slack);
        if any(slack(:)<0),yes=false;return;end
    end
    if ~isempty(leaf.payload_offsets)
        slack=leaf.payload_normals.'*P-leaf.payload_offsets;
        checks=checks+numel(slack);
        if any(slack(:)<0),yes=false;return;end
    end
    if ~isempty(leaf.cable_offsets)
        beta=leaf.cable_offsets+extra_delta;
        a=leaf.cable_normals.'*(P-L*q)-beta;
        b=leaf.cable_normals.'*P-beta;
        checks=checks+numel(a)+numel(b);
        if any(a(:)<0)||any(b(:)<0),yes=false;return;end
    end
end
end

function c=chord(v)
assert(isfinite(v)&&v>=0&&v<1, ...
    'TRAJECTORY_C9_RAY:Cone','Uncertified cable direction cone.');
c=v*sqrt(2/(1+sqrt(max(0,1-v*v))));
end

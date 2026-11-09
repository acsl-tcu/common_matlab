function R=TRAJECTORY_C9_SIMULTANEOUS_GUIDE_SEARCH(req)
% TRAJECTORY_C9_SIMULTANEOUS_GUIDE_SEARCH  Collision-start C9 guide search.
%
% A mathematical single 12-variable simultaneous-obstacle search, NOT an
% HLC trajectory command. This operates on the 3x4 free Bernstein controls
% of one degree-23 payload polynomial. The externally prescribed 0..9
% payload endpoint jets are NEVER modified; with taut, nonzero w this
% preserves UAV 0..7 joins. Duration is provided by the caller, not
% silently set equal to the 4-second collision-trigger horizon.
%
% At each iteration, the actual canonical shape support h_O(n) is used to
% form supporting-plane constraints for ALL observed obstacles and each
% of UAV, the nearest straight-cable point, and payload. These are
% *affine local models*, not claimed obstacle certificates: q depends on
% the free controls. Direction is inferred from instantaneous full-body
% geometry. For a perfectly symmetric placement, tangent x cable supplies
% an SO(3)-covariant direction if nonzero; otherwise UNKNOWN is returned.
% No fixed +x/-x/up/left bypass rule, per-shape planner, obstaclewise
% displacement superposition, or predetermined obstacle-count mask exists.
%
% The strictly convex QP objective is integrated squared LOAD
% ACCELERATION CHANGE, not an arbitrary weighted path heuristic:
%     integral_0^T || delta pL''(t) ||^2 dt.
% This minimizes acceleration disturbance (and consequent cable-direction
% change) from the nominal load polynomial subject to the same simultaneous
% separation planes. The 4x4 Bernstein derivative Gram is exact.
% A Cholesky change of variable makes it Euclidean. If G=U'*U,
% X=DeltaF*U', then gradient wrt X is grad/U (NOT grad/U').
% A dual-feasible active-set QP projection
% solves ALL local separation halfspaces simultaneously. Its
% finite iteration budget is caller-owned. No projection result is a
% certificate: the independent C9 joint continuous-time proof decides.
% Sampling supplies candidate-plane *search* information ONLY, never a
% collision-free or full-time safety claim. The local planes use the EXACT
% first-order Jacobian of q=(g-pL'')/||g-pL''||, rather than treating the cable
% direction as constant during a free-control update. When all sampled
% inequalities hold but the independent all-time certificate is UNKNOWN
% because continuous geometry remains unproved, a controlled continuation
% increases geometric clearance along the already-computed joint direction.
% The independent proof's first unresolved time interval is fed into the
% SAME joint linearization as extra search probes. No separate planning
% stage, secondary independent trajectory generator, or obstacle-specific
% path is used. Any certified continuation is shortened by proof calls to avoid
% unnecessary detours. All force, tautness, ground, exact shape support,
% and robust tube constraints are always independently proven afterwards.
% If the hard local QP is infeasible, an acceleration-coordinate squared-
% hinge direction may reduce simultaneous linearization violations. It is
% NOT a feasible QP or a safety certificate; it goes to the exact same
% independent proof before any CERTIFIED status can be emitted.
% If no proof succeeds: UNKNOWN; NEVER 'collision proven', NEVER COMMIT.
%
% Explicitly NOT YET: free-time optimization, real EKF to payload 9-jet
% authority, full 3-axis torque/attitude, tracking tube derivation, and
% HLC integration. No run_flight_headless, trigger, EKF or HLC is changed.
start=tic;
required={'t0_s','duration_s','flight_interval_s','start_jets','end_jets', ...
    'nominal_free','observed','radii','authority', ...
    'geometry_options','force_options','search_options'};
assert(isstruct(req)&&isscalar(req)&&all(isfield(req,required)), ...
    'TRAJECTORY_C9_GUIDE:Input','All external candidate and proof inputs required.');
opt=req.search_options;
assert(isstruct(opt)&&isscalar(opt)&&all(isfield(opt, ...
    {'probe_count','max_outer_iterations','max_projection_sweeps', ...
     'projection_tolerance_m','min_separation_m'})), ...
    'TRAJECTORY_C9_GUIDE:Budget','Caller-owned finite search budget required.');
for key={'probe_count','max_outer_iterations','max_projection_sweeps'}
    v=opt.(key{1});
    assert(isnumeric(v)&&isreal(v)&&isscalar(v)&&isfinite(v)&& ...
        v>=1&&v==fix(v),'TRAJECTORY_C9_GUIDE:Budget', ...
        'Search iteration limits must be positive integers.');
end
assert(opt.probe_count>=3, 'TRAJECTORY_C9_GUIDE:Budget', ...
    'Need at least 3 Chebyshev-Lobatto search probes.');
for key={'projection_tolerance_m','min_separation_m'}
    v=opt.(key{1});
    assert(isnumeric(v)&&isreal(v)&&isscalar(v)&&isfinite(v)&&v>=0, ...
        'TRAJECTORY_C9_GUIDE:Budget','Invalid optimization tolerance.');
end
assert(opt.min_separation_m>0, 'TRAJECTORY_C9_GUIDE:Budget', ...
    'Strict independent supporting-plane separation is required.');
% One diagnostic switch for paired A/B timing; production default = ON.
% Only the algebra used to assemble the SAME local QP planes changes.
fast_probe_algebra=true;
if isfield(opt,'fast_probe_algebra')
    z=opt.fast_probe_algebra;
    assert((islogical(z)||isnumeric(z))&&isscalar(z)&& ...
        isfinite(double(z))&&(double(z)==0||double(z)==1), ...
        'TRAJECTORY_C9_GUIDE:FastProbeFlag', ...
        'fast_probe_algebra must be boolean 0/1.');
    fast_probe_algebra=logical(z);
end
assert(isstruct(req.observed), 'TRAJECTORY_C9_GUIDE:Observed', ...
    'Provide the actual sensor-observed/retained obstacle array only.');
assert(isstruct(req.geometry_options)&&isscalar(req.geometry_options), ...
    'TRAJECTORY_C9_GUIDE:Phase','Proof options must be a scalar struct.');
[ground_active,phase]=TRAJECTORY_GROUND_SCOPE(req.geometry_options);
R=struct('status','UNKNOWN_NOT_STARTED','candidate_available',false, ...
    'safe_to_commit',false,'candidate_piece',struct(), ...
    'final_certificate',struct(),'initial_certificate',struct(), ...
    'trajectory_phase',phase,'ground_checked',ground_active, ...
    'considered_obstacle_ids',zeros(1,0), ...
    'search_iterations',0,'projection_sweeps',0, ...
    'joint_qp_primal_residual_m',NaN, ...
    'joint_qp_kkt_stationarity',NaN, ...
    'joint_qp_status','NOT_CALLED', ...
    'joint_qp_working_planes',0, ...
    'joint_qp_full_planes',0, ...
    'joint_qp_generation_passes',0, ...
    'relaxed_probe_attempts',0,'relaxed_probe_accepted_directions',0, ...
    'relaxed_probe_iterations',0, ...
    'relaxed_probe_first_hinge_m2',NaN, ...
    'relaxed_probe_last_hinge_m2',NaN, ...
    'linear_constraints_generated',0, ...
    'proof_witness_probe_count',0, ...
    'hull_witness_plane_count',0, ...
    'hull_witness_time_ms',0, ...
    'hull_witness_max_deficit_m',0, ...
    'hull_witness_skipped_immovable',0, ...
    'hull_witness_enabled',false, ...
    'hull_witness_policy','NOT_USED', ...
    'hull_witness_invocations',0, ...
    'hull_witness_skipped_existing_planes',0, ...
    'hull_witness_skipped_no_diagnostic_id',0, ...
    'hull_witness_skipped_not_payload',0, ...
    'hull_witness_target_ids',zeros(1,0), ...
    'topology_group_enabled',false, ...
    'topology_group_attempts',0,'topology_group_active',0, ...
    'topology_group_ids',zeros(1,0),'topology_group_normal',zeros(3,1), ...
    'point_intersection_prefilter_enabled',false, ...
    'point_intersection_skipped_full_proofs',0, ...
    'point_intersection_last_obstacle_id',NaN, ...
    'point_intersection_last_body',NaN, ...
    'point_intersection_time_total_ms',0, ...
    'point_torque_search_enabled',false, ...
    'point_torque_screen_evaluations',0, ...
    'point_torque_witness_candidates',0, ...
    'point_torque_preoracle_skips',0, ...
    'point_torque_local_cuts',0, ...
    'point_torque_local_jacobian_evaluations',0, ...
    'point_torque_time_total_ms',0, ...
    'point_torque_max_violation_Nm',0, ...
    'point_torque_last_witness_times_s',zeros(1,0), ...
    'point_torque_last_status','NOT_USED', ...
    'nominal_torque_feedback_enabled',false, ...
    'nominal_torque_feedback_calls',0, ...
    'nominal_torque_feedback_ms',0, ...
    'nominal_torque_witness_times_added',0, ...
    'nominal_torque_conditional_proven',false, ...
    'nominal_torque_last_proof_status','NOT_RUN', ...
    'nominal_torque_last_feedback_status','NOT_RUN', ...
    'fixed_endpoint_torque_gate_enabled',false, ...
    'fixed_endpoint_torque_gate_status','NOT_RUN', ...
    'fixed_endpoint_torque_gate_ms',0, ...
    'fixed_endpoint_torque_evaluated_count',0, ...
    'fixed_endpoint_torque_values_Nm',NaN(3,2), ...
    'fixed_endpoint_torque_outside_limits',false(1,2), ...
    'topology_group_support_calls',0,'topology_group_status','NOT_USED', ...
    'duplicate_candidate_proofs_avoided',0, ...
    'linear_constraints_unresolved',0, ...
    'ground_search_planes',0,'body_search_planes',0, ...
    'ground_immovable_violation_count',0, ...
    'ground_immovable_max_deficit_m',0, ...
    'force_witness_plane_count',0, ...
    'force_witness_calls',0, ...
    'force_witness_max_deficit_N',0, ...
    'force_witness_status','NOT_CALLED', ...
    'endpoint_necessary_gate',struct(), ...
    'certificate_clearance_expansions',0, ...
    'initial_independent_proof_ms',NaN, ...
    'lazy_initial_proof_enabled',false, ...
    'initial_proof_deferred',false, ...
    'initial_proof_fallback_executed',false, ...
    'initial_probe_max_deficit_m',NaN, ...
    'initial_probe_algebra_ms',0, ...
    'independent_proof_time_total_ms',0, ...
    'plane_construction_time_total_ms',0, ...
    'joint_qp_time_total_ms',0, ...
    'certified_shortening_trials',0, ...
    'first_hit_minimum_qp_policy_enabled',true, ...
    'first_hit_minimum_qp_accepted',false, ...
    'first_hit_shortening_skipped_proofs_potential',0, ...
    'first_hit_strict_qp_solved',false, ...
    'first_hit_accepted_outer_iteration',NaN, ...
    'first_hit_actual_payload_detour_upper_bound_m',NaN, ...
    'integrated_squared_acceleration_change_m2ps3',NaN, ...
    'independent_proof_calls',0, ...
    'geometry_snapshot_cache_hits',0, ...
    'geometry_snapshot_prepare_ms_total',0, ...
    'initial_shapes_shared_from_proof',false, ...
    'fixed_probe_basis_cache_hits',0, ...
    'taut_qp_embedding_enabled',false, ...
    'taut_qp_embedded_rows',0, 'taut_qp_embedding_calls',0, ...
    'taut_qp_embedding_ms',0, 'taut_qp_skipped_status','NOT_USED', ...
    'proof_status_history',{{}}, ...
    'proof_unresolved_time_history_s',zeros(0,2), ...
    'proof_unresolved_obstacle_id_history',{{}}, ...
    'proof_checked_node_history',zeros(1,0), ...
    'integrated_squared_deviation_m2',NaN, ...
    'probe_count',opt.probe_count, ...
    'fast_probe_algebra_enabled',fast_probe_algebra, ...
    'duration_s',double(req.duration_s), ...
    'flight_interval_s',double(req.flight_interval_s(:)).', ...
    'rejoin_time_s',[], 'failure_reason','', 'elapsed_ms',NaN, ...
    'scope','ONE_C9_PIECE__GEOMETRY_TENSION_THRUST__HLC_TORQUE_PENDING');
% Optional SQP local INPUT feasibility cuts in the SAME 12-variable C9
% obstacle QP. This does NOT certify continuous-time torque, and uses
% only the caller's live mass snapshot. All geometric proof calls and
% COMMIT rules remain independent and unchanged. No fixed direction or
% obstacle-specific timing is introduced. Default OFF until MATLAB A/B.
torque_search=false;torque_times=zeros(1,0);pending_torque=false;
if isfield(opt,'enable_torque_aware_joint_search')
    v=opt.enable_torque_aware_joint_search;
    assert((islogical(v)||isnumeric(v))&&isscalar(v)&& ...
        isfinite(double(v))&&any(double(v)==[0 1]), ...
        'TRAJECTORY_C9_GUIDE:TorqueSearchSwitch','Boolean 0/1 required');
    torque_search=logical(v);
end
if torque_search
    assert(isfield(req,'torque_options')&&isstruct(req.torque_options)&& ...
        isscalar(req.torque_options)&& ...
        isfield(opt,'torque_probe_count')&& ...
        isnumeric(opt.torque_probe_count)&& ...
        isscalar(opt.torque_probe_count)&& ...
        isfinite(opt.torque_probe_count)&& ...
        opt.torque_probe_count>=3&& ...
        opt.torque_probe_count==fix(opt.torque_probe_count), ...
        'TRAJECTORY_C9_GUIDE:TorqueBudget', ...
        'Provide current physical torque options and caller-owned count');
    torque_times=double(req.t0_s)+double(req.duration_s)* ...
        linspace(0,1,opt.torque_probe_count);
end
% Optional proof feedback: only available in the existing SAME 12D
% torque-aware QP. This is NOT another planning stage. OFF until MATLAB
% confirms whether it improves full-time feasibility and latency.
torque_proof_feedback=false;
if isfield(opt,'enable_nominal_torque_proof_feedback')
    v=opt.enable_nominal_torque_proof_feedback;
    assert((islogical(v)||isnumeric(v))&&isscalar(v)&& ...
        isfinite(double(v))&&any(double(v)==[0 1]), ...
        'TRAJECTORY_C9_GUIDE:TorqueProofFeedbackSwitch', ...
        'Nominal proof feedback must be boolean 0/1.');
    torque_proof_feedback=logical(v);
end
assert(~torque_proof_feedback||torque_search, ...
    'TRAJECTORY_C9_GUIDE:TorqueFeedbackNeedsSearch', ...
    'Full-time torque feedback requires the existing torque-aware same QP.');
R.nominal_torque_feedback_enabled=torque_proof_feedback;
R.point_torque_search_enabled=torque_search;
assert(isnumeric(req.flight_interval_s)&&isreal(req.flight_interval_s)&& ...
    numel(req.flight_interval_s)==2&& ...
    all(isfinite(req.flight_interval_s(:)))&& ...
    req.flight_interval_s(1)<req.flight_interval_s(2), ...
    'TRAJECTORY_C9_GUIDE:Interval', ...
    'Authoritative Flight start and end times required.');
if ~strcmp(phase,'f')
    R.status='UNKNOWN_NOT_A_FLIGHT_PIECE';
    R.failure_reason='This planner only generates Flight-phase candidates.';
    R.elapsed_ms=toc(start)*1000;return
end
if req.t0_s<req.flight_interval_s(1)|| ...
        req.t0_s+req.duration_s>req.flight_interval_s(2)
    R.status='UNKNOWN_CANDIDATE_CROSSES_NONFLIGHT_BOUNDARY';
    R.failure_reason='Whole C9 piece must remain inside the external Flight interval.';
    R.elapsed_ms=toc(start)*1000;return
end
% Canonical support oracle validates arbitrary static rotated shape records.
% Obstacle preparation is done by the INITIAL independent certificate
% below. This prevents a second canonical validation in the
% same search call. No unchecked input is used to create planes.
R.considered_obstacle_ids=zeros(1,0);
radii=req.radii;
assert(isstruct(radii)&&isscalar(radii)&& ...
    all(isfield(radii,{'uav_m','payload_m','cable_m'})), ...
    'TRAJECTORY_C9_GUIDE:Radii','Three separate live radii required.');
rad=[double(radii.uav_m),double(radii.cable_m),double(radii.payload_m)];
assert(all(isfinite(rad))&&all(rad>=0), ...
    'TRAJECTORY_C9_GUIDE:Radii','Invalid physical radii.');
gopt=req.geometry_options;
assert(isfield(gopt,'numerical_guard_m')&& ...
    isfield(gopt,'robust_tube_enabled'), ...
    'TRAJECTORY_C9_GUIDE:Tube','Proof guards and robust flag required.');
assert(gopt.robust_tube_enabled==0||gopt.robust_tube_enabled==1, ...
    'TRAJECTORY_C9_GUIDE:Tube','Robust tube flag must be 0/1.');
if gopt.robust_tube_enabled
    assert(isfield(gopt,'uncertainty_bounds_m')&& ...
        all(isfield(gopt.uncertainty_bounds_m, ...
            {'uav_m','payload_m','cable_m'})), ...
        'TRAJECTORY_C9_GUIDE:Tube','External body tracking tubes required.');
    tube=gopt.uncertainty_bounds_m;
    rad=rad+[tube.uav_m,tube.cable_m,tube.payload_m];
end
assert(all(isfinite(rad))&&all(rad>=0), ...
    'TRAJECTORY_C9_GUIDE:Tube','Invalid inflated radii.');
assert(isnumeric(gopt.numerical_guard_m)&& ...
    isscalar(gopt.numerical_guard_m)&& ...
    isfinite(gopt.numerical_guard_m)&&gopt.numerical_guard_m>=0, ...
    'TRAJECTORY_C9_GUIDE:Guard','Invalid numerical guard.');
assert(isstruct(req.authority)&&isscalar(req.authority)&& ...
    all(isfield(req.authority,{'gravity_world_mps2','cable_length_m'})), ...
    'TRAJECTORY_C9_GUIDE:Authority','External physical authority required.');
g=double(req.authority.gravity_world_mps2(:));
L=double(req.authority.cable_length_m);
assert(isequal(size(g),[3 1])&&all(isfinite(g))&& ...
    isscalar(L)&&isfinite(L)&&L>0, ...
    'TRAJECTORY_C9_GUIDE:Authority','Invalid gravity or taut-cable length.');
% Endpoint C9 jets are immutable in this 12-variable joint optimization.
% A force/tension violation already present at a fixed endpoint cannot be
% repaired by changing internal Bernstein controls. This is a necessary
% model invariant only: a PASS cannot establish any continuous-time safety.
endpoint_gate=TRAJECTORY_C9_ENDPOINT_FORCE_NECESSITY( ...
    req.start_jets,req.end_jets,req.authority,req.force_options);
R.endpoint_necessary_gate=endpoint_gate;
if ~endpoint_gate.possible
    R.status='UNKNOWN_FIXED_ENDPOINT_FORCE_OR_TENSION_INCOMPATIBLE';
    R.failure_reason=endpoint_gate.status;
    R.elapsed_ms=toc(start)*1000;
    return
end
base=TRAJECTORY_C9_FLAT_BERNSTEIN.build(req.t0_s,req.duration_s, ...
    req.start_jets,req.end_jets,req.nominal_free);
% MATHEMATICAL NECESSITY, NEVER SUFFICIENCY: a degree-23 C9 polynomial
% fixes the actual 0..9 endpoint jets through its first/last 10 controls;
% the 3x4 free interior controls cannot change them at fixed duration.
% Thus a clear nominal torque-limit witness at a valid endpoint cannot be
% repaired by a 12D interior QP.  Evaluate BEFORE costly geometry proofs.
% This is a floating-point NOMINAL point witness, not a formal interval
% violation proof or an HLC execution guarantee. Invalid/zero-speed point
% evaluations are INCONCLUSIVE (e.g., legitimate stationary yaw hold) and
% MUST NOT be treated as evidence of infeasibility.
endpoint_torque_gate=false; % opt-in until measured; no heuristic threshold
if isfield(opt,'enable_fixed_endpoint_torque_necessity')
    flag=opt.enable_fixed_endpoint_torque_necessity;
    assert((islogical(flag)||isnumeric(flag))&&isscalar(flag)&& ...
        isreal(flag)&&isfinite(double(flag))&&any(double(flag)==[0 1]), ...
        'TRAJECTORY_C9_GUIDE:EndpointTorqueGateFlag', ...
        'The fixed-endpoint torque necessity switch must be 0/1');
    endpoint_torque_gate=logical(flag);
end
R.fixed_endpoint_torque_gate_enabled=endpoint_torque_gate&&torque_search;
if R.fixed_endpoint_torque_gate_enabled
    tEndpoint=tic;
    endpoint_times=[double(req.t0_s), double(req.t0_s+req.duration_s)];
    E=TRAJECTORY_C9_TORQUE_POINT_EVALUATE( ...
        base,req.authority,req.torque_options,endpoint_times);
    R.fixed_endpoint_torque_gate_ms=1000*toc(tEndpoint);
    R.fixed_endpoint_torque_evaluated_count=sum(E.valid);
    R.fixed_endpoint_torque_values_Nm=real(E.torque_Nm);
    R.fixed_endpoint_torque_outside_limits=E.valid & E.outside_limits;
    if any(R.fixed_endpoint_torque_outside_limits)
        R.fixed_endpoint_torque_gate_status= ...
            'NUMERIC_FIXED_ENDPOINT_TORQUE_LIMIT_WITNESS';
        R.status='UNKNOWN_FIXED_ENDPOINT_NOMINAL_TORQUE_LIMIT';
        R.failure_reason=['The immutable C9 endpoint jet has a numerical ' ...
            'nominal torque-limit witness. No 12D interior QP can change it.'];
        R.elapsed_ms=1000*toc(start);return
    elseif all(E.valid)
        R.fixed_endpoint_torque_gate_status= ...
            'PASS_ENDPOINT_NECESSARY_POINT_INPUTS_ONLY';
    else
        R.fixed_endpoint_torque_gate_status= ...
            'INCONCLUSIVE_ENDPOINT_HEADING_OR_FORCE__NO_REJECTION';
    end
end
% Opt-in SINGLE degree-23 C9, 13-variable (free(:), physical T) mode.
% This is an exclusive optimization mode, never an extra planning stage
% after the 12D search. Geometry certification remains independent.
% Experimental until MATLAB regression; default OFF.
variable_time_13d=false;
if isfield(opt,'enable_13d_variable_time_joint_search')
    v=opt.enable_13d_variable_time_joint_search;
    assert((islogical(v)||isnumeric(v))&&isscalar(v)&& ...
        isreal(v)&&isfinite(double(v))&&any(double(v)==[0 1]), ...
        'TRAJECTORY_C9_GUIDE:DurationSwitch','13D flag must be boolean.');
    variable_time_13d=logical(v);
end
if variable_time_13d
    % No 12D point-torque tangent may be silently used for a moving T.
    % The nominal all-time torque certificate still runs on each
    % geometrically proven candidate. Actual HLC is ALWAYS pending.
    if torque_search || torque_proof_feedback
        R.status='UNKNOWN_13D_TORQUE_SENSITIVITY_NOT_COUPLED';
        R.failure_reason=['The old 12D point-torque rows are not valid ' ...
            'as 13D duration rows. Disable them for the separate 13D ' ...
            'nominal-only experimental mode.'];
        R.elapsed_ms=1000*toc(start);return
    end
    try
        R=TRAJECTORY_C9_13D_TIME_JOINT_SEARCH(req,R);
    catch ME
        % Fail closed; preserve exact MATLAB diagnostic for the CHECK.
        R.status='UNKNOWN_13D_SEARCH_EXCEPTION';
        R.failure_reason=[ME.identifier ' | ' ME.message];
        R.duration_13d_exception_identifier=ME.identifier;
        R.safe_to_commit=false;R.candidate_available=false;
    end
    R.elapsed_ms=1000*toc(start);
    return
end
free0=double(req.nominal_free);
current=base;
% Only build the whitened QP kernel if search is actually required.
% A certified unchanged polynomial must not pay QP setup cost.
Gacc=[];Gpos=[];U=[];
% Defer ONLY the first expensive oracle if the unmodified candidate already
% violates a sampled JOINT separating plane. This is an OPTIMIZATION
% SCHEDULING decision, NOT proof of collision or of unsafety. The returned
% candidate is still required to pass the full independent all-time oracle.
% The original candidate is certified as a fallback, within the original
% maximum total number of oracle calls, if no new candidate is certified.
% Explicit OFF allows paired complete-pipeline verification.
lazy_first=false; % opt-in until paired full-pipeline MATLAB timing proves a win
if isfield(opt,'lazy_initial_proof_on_violated_plane')
    k=opt.lazy_initial_proof_on_violated_plane;
    assert((islogical(k)||isnumeric(k))&&isscalar(k)&& ...
        isfinite(double(k))&&any(double(k)==[0 1]), ...
        'TRAJECTORY_C9_GUIDE:LazyProofSwitch', ...
        'lazy_initial_proof_on_violated_plane must be 0/1.');
    lazy_first=logical(k);
end
R.lazy_initial_proof_enabled=lazy_first;
first_A=zeros(12,0);first_b=zeros(0,1);first_diag=struct();
proof=struct('status','NOT_EVALUATED_LAZY_INITIAL_ORACLE', ...
    'certified',false,'unresolved_time_interval_s',[NaN NaN], ...
    'unresolved_obstacle_ids',zeros(1,0), ...
    'unresolved_body_indices',zeros(1,0));
if lazy_first && ~isempty(req.observed)
    tInitialProbe=tic;
    [Gacc,Gpos]=free_gram_metrics();
    U=chol(0.5*(Gacc+Gacc.'),'upper');
    % Shape definitions are checked by the SAME canonical validator used
    % inside the full-time proof. This is not a cached proof result.
    shapes=TRAJECTORY_OBSTACLE_SUPPORT.prepare(req.observed);
    [first_A,first_b,first_diag]=make_local_planes( ...
        current,shapes,rad,g,L,opt.probe_count, ...
        opt.min_separation_m+gopt.numerical_guard_m,U, ...
        ground_active,[],opt.projection_tolerance_m,fast_probe_algebra);
    R.initial_probe_algebra_ms=1000*toc(tInitialProbe);
    if isempty(first_b)
        R.initial_probe_max_deficit_m=-Inf;
    else
        R.initial_probe_max_deficit_m=max(first_b);
    end
    R.initial_proof_deferred=~first_diag.unmovable && ...
        first_diag.ambiguous==0 && ...
        any(first_b>opt.projection_tolerance_m);
end
if R.initial_proof_deferred
    R.considered_obstacle_ids=[shapes.id];
    R.initial_certificate=proof;R.final_certificate=proof;
    R.initial_shapes_shared_from_proof=false;
else
    [proof,shapes]=TRAJECTORY_C9_JOINT_FAST_CERTIFICATE( ...
        current,req.observed,radii,req.authority,gopt,req.force_options);
    R.considered_obstacle_ids=[shapes.id];
    R.initial_shapes_shared_from_proof=true;
    R.initial_certificate=proof;R.final_certificate=proof;
    R.independent_proof_calls=1;
    R.initial_independent_proof_ms=proof.elapsed_ms;
    R.independent_proof_time_total_ms=proof.elapsed_ms;
    R=account_geometry_snapshot(R,proof);
    if proof.certified
        if torque_search
            pt=TRAJECTORY_C9_TORQUE_POINT_EVALUATE( ...
                current,req.authority,req.torque_options,torque_times);
            R=account_torque_point(R,pt);
            pending_torque=all(pt.valid)&&any(pt.outside_limits);
        end
        if torque_search && ~all(pt.valid)
            R.status='UNKNOWN_EXISTING_C9_TORQUE_POINT_DEGENERACY';
            R.failure_reason=pt.status;
            R.elapsed_ms=toc(start)*1000;return
        end
        if torque_proof_feedback && ~pending_torque
            D=TRAJECTORY_C9_TORQUE_CERTIFICATE_FEEDBACK( ...
                current,req.authority,req.torque_options,torque_times);
            R=account_nominal_feedback(R,D);
            if D.can_reoptimize
                torque_times=D.updated_times_s;
                pending_torque=true;
            elseif ~D.nominal_all_time_conditional
                R.status='UNKNOWN_EXISTING_C9_NOMINAL_TORQUE_NOT_PROVEN';
                R.failure_reason=D.status;
                R.elapsed_ms=toc(start)*1000;return
            end
        end
        if ~pending_torque
            R.status='CERTIFIED_EXISTING_C9_GUIDE__HLC_TORQUE_PENDING';
            R.candidate_available=true;R.candidate_piece=current;
            R.integrated_squared_deviation_m2=0;
            R.integrated_squared_acceleration_change_m2ps3=0;
            R.elapsed_ms=toc(start)*1000;return
        end
        % Geometry is certified, but this candidate has a nominal input
        % witness. Continue the SAME C9-QP with local torque cuts instead
        % of mislabelling the candidate as input-feasible.
    end
end
if isempty(U)
    [Gacc,Gpos]=free_gram_metrics();
    U=chol(0.5*(Gacc+Gacc.'),'upper');
end
% Efficient first-certified-candidate policy (never a safety shortcut):
% A strict KKT-solved 12-variable QP is the minimum *integrated squared
% acceleration change* for the CURRENT local simultaneous 3-D planes.
% The *first* candidate from it still passes the full independent all-time
% geometry/tension/thrust oracle. Once it passes, further line-search
% certificates serve only to shorten the already-safe detour. Skipping them
% trades a possible path-length improvement for lower online latency; it
% cannot create a false geometry certificate. Only the FIRST QP step is
% eligible because later QP steps minimize change around a shifted guide.
% No fixed displacement/obstacle-specific threshold is introduced.
first_hit_policy=true;
if isfield(opt,'accept_first_certified_minimum_qp')
    v=opt.accept_first_certified_minimum_qp;
    assert((isnumeric(v)||islogical(v))&&isscalar(v)&& ...
        isfinite(double(v))&&any(double(v)==[0 1]), ...
        'TRAJECTORY_C9_GUIDE:FirstHitSwitch', ...
        'accept_first_certified_minimum_qp must be 0 or 1.');
    first_hit_policy=logical(v);
end
R.first_hit_minimum_qp_policy_enabled=first_hit_policy;
last_step=zeros(3,4);
% Exact duplicate controls cannot change the deterministic all-time proof
% under unchanged candidate authority and observed shapes. Avoid paying
% for the same continuous-time oracle again if a local QP cycles.
seen_free_controls={current.control_points_m(:,11:14)};
for outer=1:opt.max_outer_iterations
    R.search_iterations=outer;
    strict_minimum_qp_step=false;
    time_planes=tic;
    % Add the midpoint of an unresolved continuous-time certificate leaf
    % to the same joint search. It is an oracle-directed *search probe*,
    % NEVER a pointwise safety claim or a second planning stage.
    witness_u=NaN;
    % Geometry UNKNOWN may be localised to a short proof leaf that the
    % global Chebyshev search grid never saw. Inject quarter/mid/quarter
    % witnesses from THAT LEAF into the SAME all-obstacle QP. These are
    % search probes, NEVER time samples used to certify safety. Do not
    % spend extra geometric probes when ONLY force/tension is unresolved:
    % those produce analytic bound planes below instead.
    if (proof_geometry_unknown(proof)|| ...
            strcmp(proof.status,'UNKNOWN_JOINT_GROUND_CLEARANCE_NOT_PROVEN')) && ...
            isfield(proof,'unresolved_time_interval_s')
        interval=double(proof.unresolved_time_interval_s(:));
        if numel(interval)==2&&all(isfinite(interval))&& ...
                interval(1)<interval(2)
            w0=(interval(1)-req.t0_s)/req.duration_s;
            w1=(interval(2)-req.t0_s)/req.duration_s;
            witness_u=w0+(w1-w0)*[0.25,0.50,0.75];
        end
    end
    % Only after a failed previous candidate, inspect a CONNECTED group of
    % true-shaped obstacles that block the same payload certificate leaf.
    % Grouped direction is an SO(3)-covariant QP search aid, never safety.
    topology=struct('active',false,'ids',zeros(1,0), ...
        'normal',zeros(3,1),'interval_u',[NaN NaN]);
    group_topology_enabled=false; % OFF until paired MATLAB evidence demonstrates value.
    if isfield(opt,'enable_connected_group_topology')
        flag=opt.enable_connected_group_topology;
        assert((islogical(flag)||isnumeric(flag))&&isscalar(flag)&& ...
            isfinite(double(flag))&&any(double(flag)==[0 1]), ...
            'TRAJECTORY_C9_GUIDE:GroupTopologySwitch', ...
            'Connected group search flag must be boolean 0/1.');
        group_topology_enabled=logical(flag);
    end
    R.topology_group_enabled=group_topology_enabled;
    if group_topology_enabled && outer>=2 && numel(shapes)>=2 && ...
            proof_geometry_unknown(proof)
        R.topology_group_attempts=R.topology_group_attempts+1;
        topology=TRAJECTORY_C9_CONFLICT_GROUP_GUIDE( ...
            current,shapes,proof,rad(3), ...
            opt.min_separation_m+gopt.numerical_guard_m,g);
        R.topology_group_status=topology.status;
        R.topology_group_support_calls=R.topology_group_support_calls+ ...
            topology.support_evaluations;
        if topology.active
            R.topology_group_active=R.topology_group_active+1;
            R.topology_group_ids=topology.ids;
            R.topology_group_normal=topology.normal;
        end
    end
    if outer==1 && R.initial_proof_deferred
        % Reuse the exact planes from the pre-oracle probe; do not rebuild
        % them merely to enter the first QP. There is no proof witness yet.
        A=first_A;b=first_b;diagnosis=first_diag;
        R.plane_construction_time_total_ms= ...
            R.plane_construction_time_total_ms+R.initial_probe_algebra_ms;
    else
        [A,b,diagnosis]=make_local_planes(current,shapes,rad,g,L, ...
            opt.probe_count,opt.min_separation_m+gopt.numerical_guard_m, ...
            U,ground_active,witness_u,opt.projection_tolerance_m, ...
            fast_probe_algebra,topology);
    end
    % OPTIONAL candidate-search aid, NEVER a safety certificate.
    % Benchmarked unconditional payload hull cuts did not reduce the number
    % of independent proof calls and made the spherical scene much slower.
    % The optional ADAPTIVE policy invokes this extra work ONLY when the
    % existing geometry probes show NO violated plane (the case where the
    % discrete search is blind but continuous-time geometry is UNKNOWN).
    % Importantly, the oracle's reported unresolved shape ID is used only
    % to TARGET this auxiliary cut; the existing simultaneous QP planes
    % and the final independent proof still include ALL known obstacles.
    % Explicit legacy true/false preserve paired A/B reproducibility.
    hull_policy='off'; % default OFF: measured no new certificates and slower on sphere
    if isfield(opt,'enable_payload_hull_witness_planes')
        v=opt.enable_payload_hull_witness_planes;
        assert((isnumeric(v)||islogical(v))&&isscalar(v)&& ...
            isfinite(double(v))&&any(double(v)==[0 1]), ...
            'TRAJECTORY_C9_GUIDE:HullWitnessSwitch', ...
            'Hull witness flag must be boolean 0/1.');
        if logical(v),hull_policy='always';else,hull_policy='off';end
    end
    if isfield(opt,'payload_hull_witness_policy')
        v=opt.payload_hull_witness_policy;
        assert((ischar(v)||(isstring(v)&&isscalar(v)))&& ...
            any(strcmp(char(v),{'adaptive','always','off'})), ...
            'TRAJECTORY_C9_GUIDE:HullWitnessPolicy', ...
            'Policy must be adaptive, always, or off.');
        hull_policy=char(v);
    end
    R.hull_witness_policy=hull_policy;
    R.hull_witness_enabled=~strcmp(hull_policy,'off');
    geometric_unknown=proof_geometry_unknown(proof);
    has_probe_violation=~isempty(b)&&any(b>opt.projection_tolerance_m);
    payload_is_unresolved=isfield(proof,'unresolved_body_indices') && ...
        any(proof.unresolved_body_indices==3);
    if geometric_unknown && strcmp(hull_policy,'adaptive') && ...
            ~payload_is_unresolved
        R.hull_witness_skipped_not_payload= ...
            R.hull_witness_skipped_not_payload+1;
    end
    if geometric_unknown && strcmp(hull_policy,'adaptive') && ...
            has_probe_violation
        R.hull_witness_skipped_existing_planes= ...
            R.hull_witness_skipped_existing_planes+1;
    end
    invoke_hull=geometric_unknown && ...
        (strcmp(hull_policy,'always') || ...
         (strcmp(hull_policy,'adaptive') && ~has_probe_violation && ...
          payload_is_unresolved));
    if invoke_hull
        target_shapes=shapes;
        if strcmp(hull_policy,'adaptive')
            % No documented ID means the proof failed for another reason,
            % e.g. its finite node budget. Do not guess a shape to avoid.
            ids=[];
            if isfield(proof,'unresolved_obstacle_ids')
                ids=double(proof.unresolved_obstacle_ids(:).');
            end
            if isempty(ids)
                target_shapes=shapes([]);
                R.hull_witness_skipped_no_diagnostic_id= ...
                    R.hull_witness_skipped_no_diagnostic_id+1;
            else
                target_shapes=shapes(ismember([shapes.id],ids));
            end
        end
        if ~isempty(target_shapes)
            thull=tic;
            [HA,Hb,HD]=TRAJECTORY_C9_CERTIFICATE_HULL_CUTS( ...
                current,target_shapes,rad(3), ...
                opt.min_separation_m+gopt.numerical_guard_m,U, ...
                proof.unresolved_time_interval_s, ...
                opt.projection_tolerance_m);
            R.hull_witness_invocations=R.hull_witness_invocations+1;
            R.hull_witness_target_ids=unique([ ...
                R.hull_witness_target_ids, [target_shapes.id]],'stable');
            A=[A,HA];b=[b;Hb];
            R.hull_witness_plane_count=R.hull_witness_plane_count+numel(Hb);
            R.hull_witness_max_deficit_m=max( ...
                R.hull_witness_max_deficit_m,HD.maximum_deficit_m);
            R.hull_witness_skipped_immovable= ...
                R.hull_witness_skipped_immovable+HD.skipped_immovable;
            R.hull_witness_time_ms=R.hull_witness_time_ms+1000*toc(thull);
        end
    end
    % A failed continuous-time THRUST/TENSION leaf must influence the
    % SAME joint QP rather than terminating with no geometric violation.
    % These are locally linearized sufficient Bernstein force bounds,
    % never accepted as safety certificates themselves.
    % Embed an already-proven common taut cone into THIS SAME simultaneous
    % C9 QP. There are at most six affine rows (only the four free
    % Bernstein controls move). This preserves a useful sufficient
    % condition *during candidate generation*, instead of repeatedly
    % generating candidates which lose it and then rejecting them.
    % For a curved path without a common cone: SKIP, never declare it
    % impossible. The mandatory independent final certificate is unchanged.
    % Caller may disable this optimization to compare completeness/time.
    % Measured paired whole-pipeline CHECK: always-on embedding did NOT
    % reduce oracle calls and increased median collision-case runtime.
    % Retain as explicit optional search aid, but default OFF. The
    % independent all-time tension proof is never skipped.
    embed=false;
    if isfield(opt,'embed_taut_bernstein_halfspaces')
        v=opt.embed_taut_bernstein_halfspaces;
        assert((islogical(v)||isnumeric(v))&&isscalar(v)&& ...
            isfinite(double(v))&&any(double(v)==[0 1]), ...
            'TRAJECTORY_C9_GUIDE:EmbeddedTautSwitch', ...
            'Taut constraint embedding toggle must be 0 or 1.');
        embed=logical(v);
    end
    R.taut_qp_embedding_enabled=embed;
    if embed
        tEmbed=tic;
        [TA,Tb,Tdiag]=TRAJECTORY_C9_TAUT_BERNSTEIN_QP_ROWS( ...
            current,req.authority,U);
        R.taut_qp_embedding_ms=R.taut_qp_embedding_ms+1000*toc(tEmbed);
        R.taut_qp_embedding_calls=R.taut_qp_embedding_calls+1;
        R.taut_qp_embedded_rows=R.taut_qp_embedded_rows+numel(Tb);
        R.taut_qp_skipped_status=Tdiag.status;
        A=[A,TA];b=[b;Tb];
    end
    if strcmp(proof.status,'UNKNOWN_JOINT_THRUST_NOT_PROVEN')|| ...
            strcmp(proof.status,'UNKNOWN_JOINT_TENSION_NOT_PROVEN')
        witness=double(proof.unresolved_time_interval_s(:));
        if numel(witness)==2&&all(isfinite(witness))&& ...
                witness(2)>witness(1)
            [FA,Fb,Fdiag]=TRAJECTORY_C9_FORCE_WITNESS_PLANE( ...
                current,req.authority,req.force_options,U,witness);
            R.force_witness_calls=R.force_witness_calls+1;
            R.force_witness_status=Fdiag.status;
            R.force_witness_plane_count=R.force_witness_plane_count+Fdiag.planes;
            R.force_witness_max_deficit_N=max( ...
                R.force_witness_max_deficit_N,Fdiag.max_deficit_N);
            if Fdiag.immovable_violation
                R.plane_construction_time_total_ms= ...
                    R.plane_construction_time_total_ms+1000*toc(time_planes);
                R.status='UNKNOWN_FORCE_WITNESS_IMMOVABLE';
                R.failure_reason=Fdiag.status;
                break
            end
            A=[A,FA];b=[b;Fb];
        end
    end
    R.plane_construction_time_total_ms= ...
        R.plane_construction_time_total_ms+1000*toc(time_planes);
    R.linear_constraints_generated=R.linear_constraints_generated+size(A,2);
    R.linear_constraints_unresolved=R.linear_constraints_unresolved+diagnosis.ambiguous;
    R.body_search_planes=R.body_search_planes+diagnosis.body_planes;
    R.proof_witness_probe_count=R.proof_witness_probe_count+ ...
        diagnosis.witness_probes;
    R.fixed_probe_basis_cache_hits= ...
        R.fixed_probe_basis_cache_hits+double(diagnosis.basis_cache_hit);
    R.ground_search_planes=R.ground_search_planes+diagnosis.ground_planes;
    R.ground_immovable_violation_count= ...
        R.ground_immovable_violation_count+diagnosis.ground_immovable_count;
    R.ground_immovable_max_deficit_m=max( ...
        R.ground_immovable_max_deficit_m,diagnosis.ground_immovable_deficit_m);
    if diagnosis.unmovable
        R.status='UNKNOWN_UNMOVABLE_ENDPOINT_OR_ZERO_FREE_BASIS';
        R.failure_reason='A violated endpoint plane cannot be changed by free controls.';
        break
    end
    if diagnosis.ambiguous>0
        R.status='UNKNOWN_UNRESOLVED_GEOMETRIC_SYMMETRY';
        R.failure_reason='No unique SO(3)-covariant normal from input geometry.';
        break
    end
    oldfree=current.control_points_m(:,11:14);
    % Enforce the SIX real-analytic local torque inequalities together
    % with every obstacle/body/ground plane in the SAME whitened QP.
    % The pointwise cuts are necessary search information, not an
    % all-time input certificate or an independent planning stage.
    if torque_search && pending_torque
        K=TRAJECTORY_C9_TORQUE_LOCAL_CUTS( ...
            current,req.authority,req.torque_options,torque_times,U);
        R.point_torque_time_total_ms= ...
            R.point_torque_time_total_ms+K.elapsed_ms;
        R.point_torque_local_jacobian_evaluations= ...
            R.point_torque_local_jacobian_evaluations+K.jacobian_evaluations;
        R.point_torque_last_status=K.status;
        if K.cut_count==0
            R.status='UNKNOWN_C9_NOMINAL_TORQUE_LOCAL_CUTS_UNAVAILABLE';
            R.failure_reason=K.status;
            break
        end
        R.point_torque_local_cuts=R.point_torque_local_cuts+K.cut_count;
        A=[A K.A]; %#ok<AGROW>
        b=[b;K.b]; %#ok<AGROW>
    end
    % IMPORTANT: initially satisfied planes must remain in the SAME QP:
    % a simultaneous update can violate them. Do not prefilter by b>0.
    if isempty(b)||max(b)<=opt.projection_tolerance_m
        % Search probes are NOT a continuous-time certificate.
        % The existing geometric proof witness can still require more
        % clearance, using the current same joint deformation direction.
        if any(last_step(:)~=0)&&proof_geometry_unknown(proof)
            updated=oldfree+last_step;
            R.certificate_clearance_expansions= ...
                R.certificate_clearance_expansions+1;
        else
            R.status='UNKNOWN_NO_VIOLATED_SEARCH_PLANE_BUT_PROOF_FAILED';
            R.failure_reason=proof.status;
            break
        end
    else
        % One simultaneous minimum-acceleration QP in whitened physical
        % coordinates. Dual active set solves *all* local planes, including
        % planes which are satisfied before the update. The ONLY safety
        % oracle is the later independent continuous-time certificate.
        time_qp=tic;
        [x,qp]=TRAJECTORY_C9_JOINT_SCREENED_PROJECTION( ...
            A,b,opt.max_projection_sweeps,opt.projection_tolerance_m);
        R.projection_sweeps=R.projection_sweeps+qp.pivots;
        R.joint_qp_status=qp.status;
        strict_minimum_qp_step=logical(qp.solved)&& ...
            strcmp(qp.status,'SOLVED_QP_KKT__TRAJECTORY_PROOF_REQUIRED')&& ...
            isfinite(qp.maximum_primal_violation)&& ...
            qp.maximum_primal_violation<=opt.projection_tolerance_m&& ...
            isfinite(qp.kkt_stationarity);
        R.joint_qp_working_planes=R.joint_qp_working_planes+qp.working_plane_count;
        R.joint_qp_full_planes=R.joint_qp_full_planes+qp.full_plane_count;
        R.joint_qp_generation_passes=R.joint_qp_generation_passes+ ...
            qp.constraint_generation_passes;
        R.joint_qp_primal_residual_m=qp.maximum_primal_violation;
        R.joint_qp_kkt_stationarity=qp.kkt_stationarity;
        if ~qp.solved
            % A locally infeasible linearization is not a proof that the
            % nonlinear 3-D collision-avoidance problem is infeasible.
            % Stay in the SAME simultaneous 12-variable optimization and
            % seek a direction reducing ALL unsatisfied local planes.
            % This direction is NEVER accepted without the independent
            % full-time geometry/tension/thrust certificate below.
            R.relaxed_probe_attempts=R.relaxed_probe_attempts+1;
            [x,relaxed]=TRAJECTORY_C9_RELAXED_SEARCH_DIRECTION( ...
                A,b,opt.max_projection_sweeps);
            R.relaxed_probe_iterations=R.relaxed_probe_iterations+ ...
                relaxed.iterations;
            R.relaxed_probe_first_hinge_m2= ...
                relaxed.hinge_objective_before_m2;
            R.relaxed_probe_last_hinge_m2= ...
                relaxed.hinge_objective_after_m2;
            if ~relaxed.direction_available
                R.status='UNKNOWN_JOINT_QP_NOT_SOLVED';
                R.failure_reason=[qp.status ' / ' relaxed.status];
                R.joint_qp_time_total_ms= ...
                    R.joint_qp_time_total_ms+1000*toc(time_qp);
                break
            end
            R.relaxed_probe_accepted_directions= ...
                R.relaxed_probe_accepted_directions+1;
            R.joint_qp_status= ...
                'UNSOLVED_HARD_QP__RELAXED_DIRECTION_ONLY__PROOF_REQUIRED';
            R.joint_qp_primal_residual_m= ...
                relaxed.maximum_linearized_violation_m;
        end
        % Include BOTH the hard QP and any relaxed local-search effort.
        % Neither may be hidden from the online time budget.
        R.joint_qp_time_total_ms= ...
            R.joint_qp_time_total_ms+1000*toc(time_qp);
        df=reshape(x,3,4)/U.';
        updated=oldfree+df;
        last_step=df;
    end
    if ~(all(isfinite(updated(:)))&&any(updated(:)~=oldfree(:)))
        R.status='UNKNOWN_NO_FINITE_FREE_CONTROL_PROGRESS';
        R.failure_reason='No finite admissible update in the free C9 controls.';
        break
    end
    if any(cellfun(@(seen)isequal(seen,updated),seen_free_controls))
        R.duplicate_candidate_proofs_avoided= ...
            R.duplicate_candidate_proofs_avoided+1;
        R.status='UNKNOWN_JOINT_QP_CYCLE_NO_NEW_CANDIDATE';
        R.failure_reason='Identical C9 free controls were already independently checked.';
        break
    end
    seen_free_controls{end+1}=updated; %#ok<AGROW>
    current=TRAJECTORY_C9_FLAT_BERNSTEIN.build( ...
        req.t0_s,req.duration_s,req.start_jets,req.end_jets,updated);
    % A nominal point-limit witness is sufficient to avoid expending an
    % expensive continuous-time GEOMETRY oracle on this unpromising
    % candidate. The converse never follows: a candidate with no point
    % violation still needs every original continuous-time proof. The
    % floating-point point witness does not certify physical infeasibility.
    if torque_search
        pt=TRAJECTORY_C9_TORQUE_POINT_EVALUATE( ...
            current,req.authority,req.torque_options,torque_times);
        R=account_torque_point(R,pt);
        if ~all(pt.valid)
            R.status='UNKNOWN_C9_TORQUE_POINT_DEGENERACY';
            R.failure_reason=pt.status;
            break
        end
        if any(pt.outside_limits)
            pending_torque=true;
            R.point_torque_preoracle_skips= ...
                R.point_torque_preoracle_skips+1;
            R.status='UNKNOWN_C9_POINT_TORQUE_WITNESS__REOPTIMIZE';
            R.failure_reason= ...
                'Nominal point input violation; full-time torque and HLC pending.';
            continue
        else
            pending_torque=false;
        end
    end
    % Optional cheap candidate REJECTION only. A true surface point within
    % the candidate's inflated occupied body proves this candidate CANNOT
    % satisfy the required separation, with no need for a deep oracle call.
    % The converse is NEVER inferred: if no point is found the full
    % independent continuous-time oracle is executed unchanged.
    point_prefilter=false; % OFF until paired MATLAB end-to-end evidence.
    if isfield(opt,'enable_point_intersection_prefilter')
        flag=opt.enable_point_intersection_prefilter;
        assert((isnumeric(flag)||islogical(flag))&&isscalar(flag)&& ...
            isfinite(double(flag))&&any(double(flag)==[0 1]), ...
            'TRAJECTORY_C9_GUIDE:IntersectionPrefilter', ...
            'Point intersection prefilter must be boolean 0/1.');
        point_prefilter=logical(flag);
    end
    R.point_intersection_prefilter_enabled=point_prefilter;
    rejected=false;
    if point_prefilter && proof_geometry_unknown(proof)
        witness=TRAJECTORY_C9_POINT_INTERSECTION_PREFILTER( ...
            current,shapes,proof,rad,g,L);
        R.point_intersection_time_total_ms= ...
            R.point_intersection_time_total_ms+witness.elapsed_ms;
        if witness.definite_intersection
            rejected=true;
            R.point_intersection_skipped_full_proofs= ...
                R.point_intersection_skipped_full_proofs+1;
            R.point_intersection_last_obstacle_id=witness.obstacle_id;
            R.point_intersection_last_body=witness.body_index;
            % Preserve the previous positive-length failure leaf as the
            % next QP witness. This is an UNKNOWN, never a collision verdict
            % for the whole trajectory or a positive safety certificate.
            proof=struct('certified',false, ...
                'status','UNKNOWN_GEOMETRY_POINT_INTERSECTION_WITNESS', ...
                'unresolved_time_interval_s', ...
                    proof.unresolved_time_interval_s, ...
                'unresolved_obstacle_ids',witness.obstacle_id, ...
                'unresolved_body_indices',witness.body_index, ...
                'checked_node_count',0,'elapsed_ms',witness.elapsed_ms, ...
                'screened_out',true);
        end
    end
    if ~rejected
        proof=TRAJECTORY_C9_JOINT_FAST_CERTIFICATE( ...
            current,req.observed,radii,req.authority,gopt,req.force_options);
        R.independent_proof_calls=R.independent_proof_calls+1;
        R.independent_proof_time_total_ms= ...
            R.independent_proof_time_total_ms+proof.elapsed_ms;
    end
    R=account_geometry_snapshot(R,proof);
    R.final_certificate=proof;
    if proof.certified
        if torque_proof_feedback
            D=TRAJECTORY_C9_TORQUE_CERTIFICATE_FEEDBACK( ...
                current,req.authority,req.torque_options,torque_times);
            R=account_nominal_feedback(R,D);
            if D.can_reoptimize
                torque_times=D.updated_times_s;
                pending_torque=true;
                R.status='UNKNOWN_TORQUE_POINT_WITNESS__REOPTIMIZE_SAME_QP';
                R.failure_reason=D.proof_status;
                continue
            elseif ~D.nominal_all_time_conditional
                R.status='UNKNOWN_GEOMETRY_VALID_TORQUE_NOT_PROVEN';
                R.failure_reason=D.status;
                break
            end
        end
        % The immediately preceding piece did NOT pass the independent
        % proof. Reduce this single step by certificate-checked bisection;
        % every tested midpoint is independently checked in continuous
        % time. Remaining outer-budget count caps the extra proof calls.
        lower=oldfree;upper=updated;winning_proof=proof;
        % First-hit policy only applies to a FULLY certified candidate
        % whose FIRST QP step passed the KKT and primal checks. A relaxed
        % search direction, later replanning step or incomplete proof
        % continues the original certificate-checked shortening.
        first_hit=first_hit_policy && outer==1 && strict_minimum_qp_step;
        if first_hit
            R.first_hit_minimum_qp_accepted=true;
            R.first_hit_strict_qp_solved=true;
            R.first_hit_accepted_outer_iteration=outer;
            R.first_hit_shortening_skipped_proofs_potential= ...
                max(0,opt.max_outer_iterations-outer);
            % Exact real-arithmetic continuous-time upper bound on the
            % deviation of the PAYLOAD position from the initial payload
            % polynomial: Bernstein convex-hull + identical endpoints.
            % It is an output diagnostic, NOT a physical safety radius.
            change=updated-free0;
            R.first_hit_actual_payload_detour_upper_bound_m= ...
                max(sqrt(sum(change.^2,1)));
            shortening_trials=0;
        else
            shortening_trials=opt.max_outer_iterations-outer;
        end
        if torque_search
            % A geometry-valid interpolation between two points does not
            % preserve the nonlinear torque-point tests. Avoid unchecked
            % shortening when torque search is enabled. No safety waiver.
            shortening_trials=0;
        end
        for shrink=1:shortening_trials
            mid=0.5*(lower+upper);
            if isequal(mid,lower)||isequal(mid,upper),break;end
            trial=TRAJECTORY_C9_FLAT_BERNSTEIN.build( ...
                req.t0_s,req.duration_s,req.start_jets,req.end_jets,mid);
            test=TRAJECTORY_C9_JOINT_FAST_CERTIFICATE( ...
                trial,req.observed,radii,req.authority,gopt,req.force_options);
            R.independent_proof_calls=R.independent_proof_calls+1;
            R.independent_proof_time_total_ms= ...
                R.independent_proof_time_total_ms+test.elapsed_ms;
            R=account_geometry_snapshot(R,test);
            R.certified_shortening_trials=R.certified_shortening_trials+1;
            if test.certified
                upper=mid;winning_proof=test;
            else
                lower=mid;
            end
        end
        updated=upper;
        current=TRAJECTORY_C9_FLAT_BERNSTEIN.build( ...
            req.t0_s,req.duration_s,req.start_jets,req.end_jets,updated);
        proof=winning_proof;
        R.final_certificate=proof;
        R.status='CERTIFIED_C9_SIMULTANEOUS_OBSTACLE_GUIDE__HLC_TORQUE_PENDING';
        R.candidate_available=true;
        R.candidate_piece=current;
        dF=updated-free0;
        R.integrated_squared_deviation_m2=sum(sum((dF*Gpos).*dF));
        R.integrated_squared_acceleration_change_m2ps3= ...
            sum(sum((dF*Gacc).*dF))/req.duration_s^3;
        R.failure_reason='';
        break
    end
    R.status='UNKNOWN_C9_CANDIDATE_NOT_FULLY_CERTIFIED';
    R.failure_reason=proof.status;
end
% An initial search plane is only a local approximation. It does not imply
% that the original polynomial is unsafe. If lazy scheduling has produced no
% certified alternative, optionally prove the original within the SAME
% pre-existing oracle-call limit (initial + outer iterations). This prevents
% a false-negative loss of an already-safe original reference.
if R.initial_proof_deferred && ~R.candidate_available && ...
        R.independent_proof_calls<opt.max_outer_iterations+1
    original=TRAJECTORY_C9_JOINT_FAST_CERTIFICATE( ...
        base,req.observed,radii,req.authority,gopt,req.force_options);
    R.independent_proof_calls=R.independent_proof_calls+1;
    R.independent_proof_time_total_ms= ...
        R.independent_proof_time_total_ms+original.elapsed_ms;
    R.initial_proof_fallback_executed=true;
    R.initial_certificate=original;
    R.initial_independent_proof_ms=original.elapsed_ms;
    R=account_geometry_snapshot(R,original);
    if original.certified
        point_witness=false;
        if torque_search
            pt=TRAJECTORY_C9_TORQUE_POINT_EVALUATE( ...
                base,req.authority,req.torque_options,torque_times);
            R=account_torque_point(R,pt);
            point_witness=~all(pt.valid)||any(pt.outside_limits);
        end
        % Lazy original fallback must obey the same nominal proof gate as
        % the primary candidate; otherwise an unproved original could be
        % returned after all nonlinear searches were exhausted.
        nominal_fallback_unknown=false;
        if torque_proof_feedback && ~point_witness
            D=TRAJECTORY_C9_TORQUE_CERTIFICATE_FEEDBACK( ...
                base,req.authority,req.torque_options,torque_times);
            R=account_nominal_feedback(R,D);
            nominal_fallback_unknown=~D.nominal_all_time_conditional;
        end
        if point_witness
            R.status='UNKNOWN_INITIAL_GEOMETRY_VALID_BUT_POINT_TORQUE_LIMIT';
            R.failure_reason='Original C9 nominal torque point witness.';
        elseif nominal_fallback_unknown
            R.status='UNKNOWN_LAZY_ORIGINAL_NOMINAL_TORQUE_NOT_PROVEN';
            R.failure_reason=D.status;
        else
            R.status='CERTIFIED_EXISTING_C9_GUIDE__HLC_TORQUE_PENDING';
            R.candidate_available=true;R.candidate_piece=base;
            R.final_certificate=original;
            R.integrated_squared_deviation_m2=0;
            R.integrated_squared_acceleration_change_m2ps3=0;
            R.failure_reason='';
        end
    end
end
R.elapsed_ms=toc(start)*1000;
end

function R=account_torque_point(R,pt)
% Read-only bookkeeping. No point search event can authorize COMMIT.
R.point_torque_screen_evaluations=R.point_torque_screen_evaluations+1;
R.point_torque_time_total_ms=R.point_torque_time_total_ms+pt.elapsed_ms;
R.point_torque_last_status=pt.status;
R.point_torque_last_witness_times_s= ...
    pt.time_s(pt.outside_limits);
if all(pt.valid)&&any(pt.outside_limits)
    R.point_torque_witness_candidates= ...
        R.point_torque_witness_candidates+1;
end
if isfinite(pt.maximum_point_violation_Nm)
    R.point_torque_max_violation_Nm=max( ...
        R.point_torque_max_violation_Nm,pt.maximum_point_violation_Nm);
end
end

function R=account_nominal_feedback(R,D)
% The nominal torque certificate is a search gate only, NEVER HLC COMMIT.
R.nominal_torque_feedback_calls=R.nominal_torque_feedback_calls+1;
R.nominal_torque_feedback_ms=R.nominal_torque_feedback_ms+D.elapsed_ms;
R.nominal_torque_last_proof_status=D.proof_status;
R.nominal_torque_last_feedback_status=D.status;
R.nominal_torque_witness_times_added= ...
    R.nominal_torque_witness_times_added+D.new_witness_time_count;
R.nominal_torque_conditional_proven=logical(D.nominal_all_time_conditional);
end

function R=account_geometry_snapshot(R,proof)
% Diagnostics only. Neither cache use nor timing changes proof semantics.
R.proof_status_history{end+1}=proof.status;
R.proof_unresolved_time_history_s(end+1,:)= ...
    proof.unresolved_time_interval_s;
R.proof_unresolved_obstacle_id_history{end+1}= ...
    proof.unresolved_obstacle_ids;
R.proof_checked_node_history(end+1)=proof.checked_node_count;
if isfield(proof,'geometry_snapshot_cache_hit')&& ...
        proof.geometry_snapshot_cache_hit
    R.geometry_snapshot_cache_hits=R.geometry_snapshot_cache_hits+1;
end
if isfield(proof,'geometry_snapshot_prepare_ms')
    R.geometry_snapshot_prepare_ms_total= ...
        R.geometry_snapshot_prepare_ms_total+proof.geometry_snapshot_prepare_ms;
end
end

function yes=proof_geometry_unknown(proof)
% Do not blindly increase geometric motion when force or tautness failed.
yes=isstruct(proof)&&isfield(proof,'status')&& ...
    (strcmp(proof.status,'UNKNOWN_JOINT_GEOMETRY_NOT_SEPARATED')|| ...
     strcmp(proof.status,'UNKNOWN_GEOMETRY_POINT_INTERSECTION_WITNESS')|| ...
     strcmp(proof.status,'UNKNOWN_JOINT_PROOF_NODE_BUDGET'));
end

function [Ga,Gp]=free_gram_metrics()
% Analytic exact Beta products (no time-grid quadrature and no learned
% weights). Cached 4x4 objects depend ONLY on degree 23 and free indices.
persistent A0 P0
if isempty(A0)
    n=23;idx=10:13;
    P0=zeros(4,4);A0=zeros(4,4);
    for i=1:4
        for j=1:4
            P0(i,j)=bernstein_product_integral(n,idx(i),idx(j));
            v=0;
            for a=0:2
                for b=0:2
                    v=v+(-1)^(a+b)*nchoosek(2,a)*nchoosek(2,b)* ...
                        bernstein_product_integral(21,idx(i)-a,idx(j)-b);
                end
            end
            A0(i,j)=(n*(n-1))^2*v;
        end
    end
    P0=0.5*(P0+P0.');A0=0.5*(A0+A0.');
    chol(A0); % Never use an indefinite "smoothness" objective.
end
Ga=A0;Gp=P0;
end

function v=bernstein_product_integral(n,i,j)
if i<0||i>n||j<0||j>n,v=0;return;end
c1=gammaln(n+1)-gammaln(i+1)-gammaln(n-i+1);
c2=gammaln(n+1)-gammaln(j+1)-gammaln(n-j+1);
a=i+j+1;b=2*n-i-j+1;
v=exp(c1+c2+gammaln(a)+gammaln(b)-gammaln(a+b));
end

function [A,b,diag]=make_local_planes(piece,shapes,rad,g,L,N,delta,U,ground_on,witness_u,violation_tolerance,fast_probe_algebra,topology)
if nargin<13,topology=struct('active',false);end
% Geometry-covariant local search only. Collision-free claims are NEVER
% based on these probes. All 3 bodies and ALL observed obstacles participate.
P=double(piece.control_points_m);T=piece.duration_s;
Dp=(23/T)*diff(P,1,2);
Dpp=(23*22/T^2)*diff(P,2,2);
% A certificate-directed unresolved leaf supplies its quartile search
% probes. ALL observed obstacles are included at each added point, not
% just the obstacle which happened to appear first in a failed proof.
% These do NOT change the all-time acceptance condition.
[kernel,basis_cache_hit]=fixed_probe_basis_kernel(N,T,U);
% A single batched GEMM for the fixed search probes replaces 3*N
% independent degree-23/22/21 matrix-vector products.
% The witness probes remain evaluated at their exact requested time.
if fast_probe_algebra
    fixed_p=P*kernel.b23;
    fixed_v=Dp*kernel.b22;
    fixed_a=Dpp*kernel.b21;
end
probes=kernel.probes;
added=0;
for j=1:numel(witness_u)
    u=witness_u(j);
    if isfinite(u)&&u>0&&u<1&& ...
            all(abs(probes-u)>4*eps(max(1,abs(u))))
        probes(end+1)=u; %#ok<AGROW>
        added=added+1;
    end
end
% The active group normal and its true-shape support maxima are fixed
% within this entire local QP assembly. Hoist support evaluations out of
% the per-probe/per-body loops. This changes NO supporting plane, no
% observed-obstacle membership and NO independent all-time certificate.
% Each subsequent QP still contains all individually checked UAV/cable
% constraints and the group payload plane only on the failing leaf.
group_members=false(1,numel(shapes));
group_hmax=-Inf;
if topology.active
    group_members=ismember([shapes.id],topology.ids);
    for gj=find(group_members)
        group_hmax=max(group_hmax, ...
            TRAJECTORY_OBSTACLE_SUPPORT.value( ...
                shapes(gj),topology.normal));
    end
end
maxplanes=3*(numel(shapes)+double(ground_on))*numel(probes);
A=zeros(12,maxplanes);b=zeros(maxplanes,1);nplanes=0;
diag=struct('ambiguous',0,'unmovable',false, ...
    'witness_probes',added,'basis_cache_hit',basis_cache_hit, ...
    'ground_planes',0,'body_planes',0, ...
    'ground_immovable_count',0,'ground_immovable_deficit_m',0);
for k=1:numel(probes)
    % Search probes ONLY; independent proof covers the full interval.
    u=probes(k);
    if k<=N
        h=kernel.b23(:,k);h1=kernel.b22(:,k);h2=kernel.b21(:,k);
        freebasis=kernel.free(:,k);d2basis=kernel.d2(:,k);
        if fast_probe_algebra
            p=fixed_p(:,k);v=fixed_v(:,k);a=fixed_a(:,k);
            white_p=kernel.free_white(:,k).';
            white_a=kernel.d2_white(:,k).';
        end
    else
        % Only certificate-directed witness times are dynamic.
        h=bernstein_basis(23,u);
        h1=bernstein_basis(22,u);
        h2=bernstein_basis(21,u);
        freebasis=h(11:14);
        d2basis=(23*22/T^2)*(h2(9:12)-2*h2(10:13)+h2(11:14));
        if fast_probe_algebra
            white_p=freebasis.'/U;
            white_a=d2basis.'/U;
        end
    end
    if ~fast_probe_algebra
        p=P*h;v=Dp*h1;a=Dpp*h2;
    end
    w=g-a;sw=norm(w);
    if ~(isfinite(sw)&&sw>0)
        diag.ambiguous=diag.ambiguous+1;break
    end
    q=w/sw;
    uv=p-L*q;
    for j=1:numel(shapes)
        S=shapes(j);
        lambda=max(0,min(1,(q.'*(p-S.center))/L));
        cp=p-lambda*L*q;
        points=[uv,cp,p];
        lambdas=[1,lambda,0];
        for body=1:3
            % When individual obstacle contact planes conflict, one
            % geometry-derived common halfspace replaces ONLY the payload
            % probe planes of that connected group on the unresolved leaf.
            % All other obstacle/body planes and independent proof remain.
            grouped=topology.active && body==3 && ...
                group_members(j) && ...
                u>=topology.interval_u(1) && u<=topology.interval_u(2);
            if grouped && S.id~=topology.ids(1)
                continue
            end
            point=points(:,body);
            if grouped
                normal=topology.normal;
                bound=group_hmax+rad(body)+delta-normal.'*point;
            else
            axis=point-S.center;
            if norm(axis)==0,axis=cross(v,q);end
            sn=norm(axis);
            if ~(isfinite(sn)&&sn>0)
                diag.ambiguous=diag.ambiguous+1;continue
            end
            normal=axis/sn;
            bound=TRAJECTORY_OBSTACLE_SUPPORT.value(S,normal)+ ...
                rad(body)+delta-normal.'*point;
            % The centre ray is optimal for a sphere, but can create an
            % unnecessarily restrictive LOCAL search plane for an
            % oriented box, cylinder, or anisotropic ellipsoid. For a
            % currently violated plane only, inspect the true support
            % point of the shape (or its conservative hull) and try the
            % data-derived contact displacement as an alternate normal.
            % Accept it only if its EXACT support inequality reduces the
            % present deficit. This is not a collision certificate; the
            % independent all-time C9 proof remains mandatory. No
            % world-axis direction, per-shape path, or parameter tuning.
            if bound>0 && ~strcmp(S.kind,'sphere')
                [~,contact]=TRAJECTORY_OBSTACLE_SUPPORT.value_and_point( ...
                    S,normal);
                contact_axis=point-contact;
                contact_norm=norm(contact_axis);
                if isfinite(contact_norm)&&contact_norm>0
                    % Scale-free bisector of the centre ray and genuine
                    % shape-contact displacement. With exact support
                    % acceptance below it cannot worsen this local plane.
                    candidate_normal=normal+contact_axis/contact_norm;
                    candidate_norm=norm(candidate_normal);
                    if isfinite(candidate_norm)&&candidate_norm>0
                        alternate=candidate_normal/candidate_norm;
                        alternate_bound= ...
                            TRAJECTORY_OBSTACLE_SUPPORT.value(S,alternate)+ ...
                            rad(body)+delta-alternate.'*point;
                        if isfinite(alternate_bound) && alternate_bound<bound
                            normal=alternate;
                            bound=alternate_bound;
                        end
                    end
                end
            end
            end % group vs individual true-shape support plane
            % Keep initially satisfied planes: they can bind after the
            % simultaneous update. Final acceptance is independent proof.
            % EXACT first-order derivative for a fixed cable lambda:
            % dq=-(I-qq')*da/||w||, d(p-lambda*L*q)=dp+
            % lambda*L*(I-qq')*da/||w||. Higher orders are NOT
            % linearized safety certificates.
            projected=normal-q*(q.'*normal);
            if fast_probe_algebra
                % Right-division is linear: (n*f' + c*p*a')/U =
                % n*(f'/U) + c*p*(a'/U). Compute f'/U, a'/U once
                % per probe instead of once per obstacle/body plane.
                arow=reshape(normal*white_p+ ...
                    (lambdas(body)*L/sw)*projected*white_a,12,1);
            else
                grad=normal*freebasis.'+ ...
                    (lambdas(body)*L/sw)*projected*d2basis.';
                arow=reshape(grad/U,12,1);
            end
            if ~(all(isfinite(arow))&&arow.'*arow>0)
                if bound>violation_tolerance
                    diag.unmovable=true;break
                end
                continue  % An already satisfied immovable plane.
            end
            nplanes=nplanes+1;
            A(:,nplanes)=arow;b(nplanes)=bound;
            diag.body_planes=diag.body_planes+1;
        end
        if diag.unmovable,break;end
    end
    if diag.unmovable,break;end
    if ground_on
        % Infinite true ground z<=0, ALWAYS known only in Flight. Each
        % candidate cable is linear in lambda: its lowest z occurs at
        % one of the two endpoints. This is NOT a whole-system sphere.
        lowcable=double(q(3)>0);
        ground_lambdas=[1,lowcable,0];
        for body=1:3
            lambda=ground_lambdas(body);
            point=p-lambda*L*q;
            bound=rad(body)+delta-point(3);
            normal=[0;0;1];
            projected=normal-q*(q.'*normal);
            if fast_probe_algebra
                arow=reshape(normal*white_p+ ...
                    (lambda*L/sw)*projected*white_a,12,1);
            else
                grad=normal*freebasis.'+ ...
                    (lambda*L/sw)*projected*d2basis.';
                arow=reshape(grad/U,12,1);
            end
            if ~(all(isfinite(arow))&&arow.'*arow>0)
                if bound>violation_tolerance
                    % An endpoint has zero free-control sensitivity. Its
                    % violated GROUND inequality MUST be rejected, not
                    % silently treated as a missing sampled ground plane.
                    % Count separately from planes actually sent to QP.
                    diag.ground_immovable_count= ...
                        diag.ground_immovable_count+1;
                    diag.ground_immovable_deficit_m=max( ...
                        diag.ground_immovable_deficit_m,bound);
                    diag.unmovable=true;break
                end
                continue
            end
            diag.ground_planes=diag.ground_planes+1;
            nplanes=nplanes+1;
            A(:,nplanes)=arow;b(nplanes)=bound;
        end
    end
    if diag.unmovable,break;end
end
A=A(:,1:nplanes);b=b(1:nplanes);
end

function [K,hit]=fixed_probe_basis_kernel(N,T,U)
% Degrees are intrinsic to the single degree-23 C9 polynomial. Cached
% geometry-independent Chebyshev probe bases are valid for arbitrary 3-D
% obstacles, any count, pose, or order. Duration is part of the exact key;
% a new Flight duration always obtains the correct acceleration factors.
% No obstacle/detection combination or trajectory certificate is cached.
persistent lastN lastT lastU lastKernel
hit=~isempty(lastKernel)&&isequal(N,lastN)&&isequal(T,lastT)&& ...
    isequal(U,lastU);
if hit,K=lastKernel;return;end
K=struct();K.probes=(1-cos(pi*(0:N-1)/(N-1)))/2;
K.b23=zeros(24,N);K.b22=zeros(23,N);K.b21=zeros(22,N);
K.free=zeros(4,N);K.d2=zeros(4,N);
for j=1:N
    u=K.probes(j);
    K.b23(:,j)=bernstein_basis(23,u);
    K.b22(:,j)=bernstein_basis(22,u);
    K.b21(:,j)=bernstein_basis(21,u);
    h2=K.b21(:,j);
    K.free(:,j)=K.b23(11:14,j);
    K.d2(:,j)=(23*22/T^2)*(h2(9:12)-2*h2(10:13)+h2(11:14));
end
% This whitening is exact in real arithmetic and depends ONLY on the
% Bernstein degree/Gram, number of probes, and piece duration.
% No obstacle or physical parameter is cached. U is part of the key.
K.free_white=U.'\K.free;
K.d2_white=U.'\K.d2;
lastN=N;lastT=T;lastU=U;lastKernel=K;
end

function b=bernstein_basis(n,u)
b=zeros(n+1,1);
if u<=0,b(1)=1;return;end
if u>=1,b(end)=1;return;end
if u<=0.5
    b(1)=(1-u)^n;
    for k=1:n
        b(k+1)=b(k)*(n-k+1)/k*u/(1-u);
    end
else
    b(end)=u^n;
    for k=n:-1:1
        b(k)=b(k+1)*k/(n-k+1)*(1-u)/u;
    end
end
end

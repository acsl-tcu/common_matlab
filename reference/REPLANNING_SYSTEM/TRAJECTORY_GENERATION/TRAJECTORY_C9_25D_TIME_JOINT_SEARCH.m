function R=TRAJECTORY_C9_25D_TIME_JOINT_SEARCH(req,R)
%TRAJECTORY_C9_25D_TIME_JOINT_SEARCH One degree-27/C9/25-variable search.
% [24 interior 3D Bernstein coordinates + one physical duration] are jointly
% changed by a single linearized QP. Candidate geometry is subsequently
% certified for ALL obstacles, ALL bodies and ALL time by the existing
% independent degree-27 full-time oracle. Heading and torque are separately
% certified on exactly the same candidate. No sampling-based PASS.
% This experimental solver is OFF by default and is not a Flight command.
clock=tic;
R.safe_to_commit=false;R.candidate_available=false;
R.duration_25d_enabled=true;
R.duration_25d_qp_count=0;
R.duration_25d_qp_ms=0;
R.duration_25d_plane_ms=0;
R.duration_25d_proof_ms=0;
R.duration_25d_torque_ms=0;
R.duration_25d_heading_ms=0;
R.duration_25d_torque_tangent_ms=0;
R.duration_25d_torque_point_rows=0;
R.duration_25d_physical_leaf_rows=0;
R.duration_25d_candidate_history_s=[];
R.duration_25d_last_oracle_status='NOT_RUN';
R.duration_25d_nominal_torque_proof_bundle=struct();
R.duration_25d_heading_proof_bundle=struct();
R.duration_25d_conditional_nominal_torque_proven=false;
R.duration_25d_nominal_torque_status='NOT_RUN';
R.duration_25d_qp_max_violation_history=[];
R.duration_25d_last_qp_construction_status='NOT_RUN';
R.duration_25d_optional_leaf_skips=0;
R.duration_25d_optional_torque_skips=0;
R.duration_25d_metric_ms=0;
R.duration_25d_tangent_ms=0;
R.duration_25d_geometry_rows_ms=0;
R.duration_25d_leaf_rows_ms=0;
R.duration_25d_native_torque_calls=0;
R.duration_25d_native_torque_ms=0;
R.duration_25d_fused_native_enabled=false;
R.duration_25d_fused_native_witness_free=false;
R.duration_25d_fused_native_calls=0;
R.duration_25d_fused_native_positive=0;
R.duration_25d_fused_native_ms=0;
R.duration_25d_fused_native_fallback_calls=0;
R.duration_25d_fused_generic_calls=0;
R.duration_25d_fused_proof_nodes=0;
R.duration_25d_fused_shape_support_calls=0;
R.duration_25d_fused_inherited_shape_prunes=0;
R.duration_25d_fused_banked_shape_skips=0;
R.duration_25d_fused_near_checks=0;
R.duration_25d_fused_outer_rejections=0;
R.duration_25d_fused_shape_prepare_ms=0;
R.duration_25d_fused_generic_positives=0;
R.duration_25d_fused_generic_iterations=0;
R.duration_25d_fused_gjk_steps=0;
R.duration_25d_fused_gjk_fallbacks=0;
R.duration_25d_fused_gjk_convex_hull_nonseparation_witnesses=0;
R.duration_25d_fused_native_witness_free_qps=0;
R.duration_25d_fused_native_witness_qps=0;
R.duration_25d_fused_native_witness_fallbacks=0;
R.duration_25d_canonical_oracle_calls=0;
R.duration_25d_terminal_native_unknown_skips=0;
R.duration_25d_risk_seed_calls=0;
R.duration_25d_risk_seed_ms=0;
R.duration_25d_risk_seed_rows=0;
R.duration_25d_risk_seed_status='NOT_RUN';
R.duration_25d_risk_seed_batches=0;
R.duration_25d_risk_seed_evaluations=0;
R.duration_25d_risk_seed_degenerate_directions=0;
R.duration_25d_group_graph_ms=0;
R.duration_25d_group_graph_pair_tests=0;
R.duration_25d_group_corridor_count=0;
R.duration_25d_group_corridor_ms=0;
R.duration_25d_group_corridor_removed_scalar_rows=0;
R.duration_25d_group_corridor_added_rows=0;
R.duration_25d_group_corridor_candidate_directions=0;
R.duration_25d_group_corridor_support_evaluations=0;
% Bounded by actual outer iterations, NEVER a safety witness. Diagnostics
% record which obstacle/body/time remains unresolved and why QP stops.
R.duration_25d_iteration_diagnostics=struct( ...
 'iteration',{},'candidate_duration_s',{},'oracle_status',{}, ...
 'native_status',{},'unresolved_obstacle_ids',{}, ...
 'unresolved_body_indices',{},'unresolved_time_interval_s',{}, ...
 'oracle_ms',{},'risk_seed_status',{},'risk_seed_count',{}, ...
 'risk_seed_ms',{},'QP_build_status',{},'QP_build_ms',{}, ...
 'QP_solve_status',{},'QP_solve_ms',{},'step_norm',{}, ...
 'corridor_groups',{},'corridor_rows',{}, ...
 'corridor_status',{},'corridor_ms',{});

R.duration_25d_witness_bank_calls=0;
R.duration_25d_witness_bank_distinct_ids=0;
R.duration_25d_witness_bank_targeted_phases=0;
R.duration_25d_witness_bank_targeted_rows=0;
% Optional certified-witness-driven lazy *SEARCH* torque rows. Geometry
% failure alone is not a proof of torque infeasibility, so expensive torque
% point-Jacobians are omitted until the independent all-time torque oracle
% really fails on a geometry-safe candidate. This does NOT remove any
% final all-time torque check or introduce a second optimization stage.
R.duration_25d_lazy_torque_enabled=false;
R.duration_25d_lazy_torque_skipped_qps=0;
R.duration_25d_lazy_torque_activated_qps=0;
R.duration_25d_torque_failure_witnesses=0;
R.scope='ONE_C9_DEGREE27_25D_SIMULTANEOUS__HLC_EXECUTION_PENDING';
[ground_active,phase_name]=TRAJECTORY_GROUND_SCOPE(req.geometry_options);
if ~strcmp(phase_name,'f')
    R.status='UNKNOWN_C9_25D_NONFLIGHT';R.elapsed_ms=1000*toc(clock);return
end
Tmax=req.flight_interval_s(2)-req.t0_s;
if ~(isfinite(Tmax)&&Tmax>0&&req.duration_s>0&&req.duration_s<=Tmax)
    R.status='UNKNOWN_C9_25D_DURATION_OUTSIDE_FLIGHT';
    R.elapsed_ms=1000*toc(clock);return
end
% Embed the exact SAME degree-23 nominal curve into the degree-27 class.
% This does not create a second trajectory segment or a 13D preliminary
% optimization. Caller may instead provide authoritative 3x8 free controls.
if isfield(req,'nominal_free27')
    free=double(req.nominal_free27);
    assert(isequal(size(free),[3 8])&&all(isfinite(free(:))), ...
        'C9_25D:InitialFree','Explicit 27-degree free controls must be 3x8.');
else
    initial23=TRAJECTORY_C9_FLAT_BERNSTEIN.build( ...
        req.t0_s,req.duration_s,req.start_jets,req.end_jets,req.nominal_free);
    elevated=double(initial23.control_points_m);
    for n=23:26
        next=zeros(3,n+2);
        next(:,1)=elevated(:,1);next(:,end)=elevated(:,end);
        for i=1:n
            next(:,i+1)=(i/(n+1))*elevated(:,i)+ ...
                (1-i/(n+1))*elevated(:,i+1);
        end
        elevated=next;
    end
    free=elevated(:,11:18);
end
current=TRAJECTORY_C9_FLAT_BERNSTEIN.build27( ...
    req.t0_s,req.duration_s,req.start_jets,req.end_jets,free);
R.duration_25d_initial_degree23_embedded=~isfield(req,'nominal_free27');
R.duration_25d_candidate_history_s=current.duration_s;
R.trajectory_phase=phase_name;R.ground_checked=ground_active;
% No torque/search authority may be invented by this module.
if ~(isfield(req,'torque_options')&&isstruct(req.torque_options)&& ...
        isfield(req,'heading_options')&&isstruct(req.heading_options))
    R.status='UNKNOWN_C9_25D_HEADING_TORQUE_AUTHORITY_MISSING';
    R.elapsed_ms=1000*toc(clock);return
end
rad=[double(req.radii.uav_m),double(req.radii.cable_m), ...
    double(req.radii.payload_m)];
if req.geometry_options.robust_tube_enabled
    u=req.geometry_options.uncertainty_bounds_m;
    rad=rad+[double(u.uav_m),double(u.cable_m),double(u.payload_m)];
end
assert(all(isfinite(rad))&&all(rad>=0), ...
    'C9_25D:Radii','Current physical plus tube radii required.');
N=double(req.search_options.probe_count);
phases=(1-cos(pi*(0:N-1)/(N-1)))/2;
initial_risk_probe_phases=phases; % non-growing seed grid; not a collision certificate
seen=zeros(25,0);
shapes=[];
lazy_torque=false;
if isfield(req.search_options,'enable_25d_certificate_driven_torque_rows')
    sw=req.search_options.enable_25d_certificate_driven_torque_rows;
    assert((isnumeric(sw)||islogical(sw))&&isscalar(sw)&& ...
        isfinite(double(sw))&&any(double(sw)==[0 1]), ...
        'C9_25D:LazyTorqueFlag','Expected 0/1.');
    lazy_torque=logical(sw);
end
R.duration_25d_lazy_torque_enabled=lazy_torque;
torque_failure_witness=false;
corridor_graph=[];
% Structural all-time oracle, one compiled tree per candidate (no MATLAB
% per-leaf callbacks). Positive results are independently sufficient under
% the stated real-arithmetic model. Numeric directed rounding remains
% explicitly NOT certified; safe_to_commit always false.
% Search-only witness-free mode avoids expensive canonical UNKNOWN probes
% on intermediate iterations. Final positive certification remains mandatory.
fused=false;witness_free=false;
if isfield(req.search_options,'enable_25d_fused_native_alltime_oracle')
 v=req.search_options.enable_25d_fused_native_alltime_oracle;
 assert(isscalar(v)&&any(double(v)==[0 1]),'C9_25D:FusedFlag', ...
     'Fused oracle flag must be 0/1.');
 fused=logical(v);
end
if isfield(req.search_options,'enable_25d_fused_native_witness_free_search')
 v=req.search_options.enable_25d_fused_native_witness_free_search;
 assert(isscalar(v)&&any(double(v)==[0 1]),'C9_25D:WitnessFreeFlag', ...
     'Witness-free flag must be 0/1.');
 witness_free=logical(v);
end
if witness_free && ~fused
  error('C9_25D:FusedSwitch','Witness-free search requires fused oracle.');
end
R.duration_25d_fused_native_enabled=fused;
witness_driven=false;
if isfield(req.search_options,'enable_25d_fused_native_witness_driven_search')
    v=req.search_options.enable_25d_fused_native_witness_driven_search;
    assert(isscalar(v)&&any(double(v)==[0 1]), ...
        'C9_25D:WitnessDrivenFlag','Must be 0/1.');
    witness_driven=logical(v);
end
if witness_driven && (~fused || witness_free)
    error('C9_25D:WitnessDrivenFlags', ...
        'Native proof-tree witness mode requires fused=1, witness-free=0.');
end
R.duration_25d_fused_native_witness_driven=witness_driven;
R.duration_25d_fused_native_witness_free=witness_free;
for iteration=0:req.search_options.max_outer_iterations
    R.search_iterations=iteration;
    toracle=tic;
    nativePositive=false;skipCanonical=false;
    if fused
        [nativeProof,nativeShapes]=TRAJECTORY_C9_FUSED_NOMINAL_ORACLE( ...
            current,req.observed,req.radii,req.authority, ...
            req.geometry_options,req.force_options);
        R.duration_25d_fused_native_calls=R.duration_25d_fused_native_calls+1;
        R.duration_25d_fused_native_ms= ...
            R.duration_25d_fused_native_ms+nativeProof.elapsed_ms;
        R.duration_25d_fused_generic_calls= ...
            R.duration_25d_fused_generic_calls+nativeProof.generic_separator_calls;
        R.duration_25d_fused_generic_positives= ...
            R.duration_25d_fused_generic_positives+nativeProof.generic_separator_positives;
        R.duration_25d_fused_generic_iterations= ...
            R.duration_25d_fused_generic_iterations+nativeProof.generic_separator_iterations;
        R.duration_25d_fused_gjk_steps= ...
            R.duration_25d_fused_gjk_steps+nativeProof.gjk_steps;
        R.duration_25d_fused_gjk_fallbacks= ...
            R.duration_25d_fused_gjk_fallbacks+nativeProof.gjk_fallbacks;
        R.duration_25d_fused_gjk_convex_hull_nonseparation_witnesses= ...
            R.duration_25d_fused_gjk_convex_hull_nonseparation_witnesses+ ...
            nativeProof.gjk_convex_hull_nonseparation_witnesses;
        R.duration_25d_fused_proof_nodes= ...
            R.duration_25d_fused_proof_nodes+nativeProof.checked_node_count;
        R.duration_25d_fused_shape_support_calls= ...
            R.duration_25d_fused_shape_support_calls+nativeProof.shape_support_calls;
        R.duration_25d_fused_inherited_shape_prunes= ...
            R.duration_25d_fused_inherited_shape_prunes+nativeProof.inherited_shape_prunes;
        R.duration_25d_fused_banked_shape_skips= ...
            R.duration_25d_fused_banked_shape_skips+nativeProof.banked_shape_skips;
        R.duration_25d_fused_near_checks= ...
            R.duration_25d_fused_near_checks+nativeProof.near_obstacle_leaf_checks;
        R.duration_25d_fused_outer_rejections= ...
            R.duration_25d_fused_outer_rejections+nativeProof.outer_rejections;
        R.duration_25d_fused_shape_prepare_ms= ...
            R.duration_25d_fused_shape_prepare_ms+nativeProof.geometry_snapshot_prepare_ms;
        if isfield(nativeProof,'witness_bank_count') && ...
                nativeProof.witness_bank_count>0
            R.duration_25d_witness_bank_calls= ...
                R.duration_25d_witness_bank_calls+1;
            R.duration_25d_witness_bank_distinct_ids= ...
                R.duration_25d_witness_bank_distinct_ids+ ...
                nativeProof.witness_bank_count;
        end
        nativePositive=nativeProof.certified;
        if nativePositive
            proof=nativeProof;shapes=nativeShapes;
            R.duration_25d_fused_native_positive= ...
                R.duration_25d_fused_native_positive+1;
            R.independent_proof_calls=R.independent_proof_calls+1;
        elseif witness_driven && ...
                iteration<req.search_options.max_outer_iterations && ...
                startsWith(nativeProof.native_status,'UNKNOWN_') && ...
                numel(nativeProof.unresolved_obstacle_ids)>=1 && ...
                numel(nativeProof.unresolved_body_indices)>=1 && ...
                all(isfinite(nativeProof.unresolved_time_interval_s)) && ...
                nativeProof.unresolved_time_interval_s(2)> ...
                    nativeProof.unresolved_time_interval_s(1)
            % Native proof-tree witness supplies SEARCH information in the
            % same call as failed full-time proof. Final candidate still
            % must pass full geometry, heading and torque certification.
            proof=nativeProof;shapes=nativeShapes;skipCanonical=true;
            R.duration_25d_fused_native_witness_qps= ...
                R.duration_25d_fused_native_witness_qps+1;
        elseif witness_free && iteration<req.search_options.max_outer_iterations && ...
                ~startsWith(nativeProof.status,'UNKNOWN_FUSED_NATIVE_EXCEPTION') && ...
                ~strcmp(nativeProof.native_status,'NOT_RUN')
            % No inference that the candidate is unsafe. Only an optional
            % SEARCH witness is absent. Same 25D QP and final certificate.
            proof=nativeProof;shapes=nativeShapes;skipCanonical=true;
            R.duration_25d_fused_native_witness_free_qps= ...
                R.duration_25d_fused_native_witness_free_qps+1;
        end
    end
    % Search-only terminal acceleration: an already UNKNOWN final native
    % candidate cannot become an accepted Flight trajectory via this flag.
    % Canonical may be stronger and prove the very same candidate, so this
    % choice can LOWER search completeness. It is OFF by default and A/B
    % must demonstrate non-regression before enabling. It never returns PASS.
    terminal_fast=false;
    if isfield(req.search_options,'enable_25d_terminal_native_unknown_only')
        f=req.search_options.enable_25d_terminal_native_unknown_only;
        assert(isscalar(f)&&any(double(f)==[0 1]), ...
           'C9_25D:TerminalFlag','Expected 0/1.');
        terminal_fast=logical(f);
    end
    if ~nativePositive && ~skipCanonical && fused && terminal_fast && ...
            iteration>=req.search_options.max_outer_iterations && ...
            startsWith(nativeProof.status,'UNKNOWN_FUSED_NATIVE_') && ...
            ~contains(nativeProof.status,'EXCEPTION') && ...
            ~contains(nativeProof.status,'MEX_MISSING') && ...
            ~contains(nativeProof.status,'MEX_SIGNATURE') && ...
            ~strcmp(nativeProof.native_status,'NOT_RUN')
        proof=nativeProof;shapes=nativeShapes;skipCanonical=true;
        R.duration_25d_terminal_native_unknown_skips= ...
            R.duration_25d_terminal_native_unknown_skips+1;
    end
    if ~nativePositive && ~skipCanonical
        [proof,shapes]=TRAJECTORY_C9_JOINT_FAST_CERTIFICATE( ...
            current,req.observed,req.radii,req.authority, ...
            req.geometry_options,req.force_options);
        R.independent_proof_calls=R.independent_proof_calls+1;
        R.duration_25d_canonical_oracle_calls= ...
            R.duration_25d_canonical_oracle_calls+1;
        if witness_driven
            R.duration_25d_fused_native_witness_fallbacks= ...
                R.duration_25d_fused_native_witness_fallbacks+1;
        end
        if fused
            R.duration_25d_fused_native_fallback_calls= ...
                R.duration_25d_fused_native_fallback_calls+1;
        end
    end
    R.duration_25d_proof_ms=R.duration_25d_proof_ms+1000*toc(toracle);
    R.final_certificate=proof;
    R.duration_25d_last_oracle_status=proof.status;
    nativeStatus='NOT_REQUESTED';
    if fused,nativeStatus=nativeProof.native_status;end
    unknownIds=[];unknownBodies=[];unknownInterval=[NaN NaN];
    if isfield(proof,'unresolved_obstacle_ids')
        unknownIds=double(proof.unresolved_obstacle_ids(:)).';
    end
    if isfield(proof,'unresolved_body_indices')
        unknownBodies=double(proof.unresolved_body_indices(:)).';
    end
    if isfield(proof,'unresolved_time_interval_s')
        unknownInterval=double(proof.unresolved_time_interval_s(:)).';
    end
    diag=struct('iteration',iteration, ...
       'candidate_duration_s',double(current.duration_s), ...
       'oracle_status',proof.status,'native_status',nativeStatus, ...
       'unresolved_obstacle_ids',unknownIds, ...
       'unresolved_body_indices',unknownBodies, ...
       'unresolved_time_interval_s',unknownInterval, ...
       'oracle_ms',1000*toc(toracle), ...
       'risk_seed_status','NOT_REQUESTED','risk_seed_count',0, ...
       'risk_seed_ms',0, ...
       'QP_build_status','NOT_RUN','QP_build_ms',0, ...
       'QP_solve_status','NOT_RUN','QP_solve_ms',0, ...
       'step_norm',NaN,'corridor_groups',0,'corridor_rows',0, ...
       'corridor_status','NOT_REQUESTED','corridor_ms',0);
    R.duration_25d_iteration_diagnostics(end+1)=diag;

    if iteration==0,R.initial_certificate=proof;end
    if proof.certified
        th=tic;
        heading=TRAJECTORY_C9_UAV_C7_YAW_C6_AUDIT( ...
            current,req.authority,req.heading_options);
        R.duration_25d_heading_ms= ...
            R.duration_25d_heading_ms+1000*toc(th);
        if ~heading.certified
            R.status='UNKNOWN_C9_25D_HEADING_NOT_CERTIFIED';
            R.failure_reason=heading.status;break
        end
        R.duration_25d_heading_proof_bundle=struct( ...
            'piece',current,'authority',req.authority, ...
            'options',req.heading_options,'proof',heading, ...
            'proof_call_ms',1000*toc(th));
        tt=tic;
        torque=TRAJECTORY_C9_NOMINAL_TORQUE_INTERVAL( ...
            current,req.authority,req.torque_options);
        torque_ms=1000*toc(tt);
        R.duration_25d_torque_ms=R.duration_25d_torque_ms+torque_ms;
        R.duration_25d_nominal_torque_status=torque.status;
        if torque.conditional_nominal_all_time_torque_proven
            R.status= ...
                'CONDITIONAL_C9_25D_DEGREE27_NOMINAL_FULLTIME__HLC_PENDING';
            R.candidate_available=true;
            R.candidate_piece=current;
            R.duration_s=current.duration_s;
            R.duration_25d_conditional_nominal_torque_proven=true;
            R.duration_25d_nominal_torque_proof_bundle=struct( ...
                'piece',current,'authority',req.authority, ...
                'options',req.torque_options,'proof',torque, ...
                'proof_call_ms',torque_ms);
            R.failure_reason='Actual HLC reachable tube/4 inputs not certified.';
            break
        end
        % The full-time torque oracle (not a pointwise linearization)
        % now supplied a real failure witness.  From this moment, allow
        % 25D torque search cuts in the SAME simultaneous QP.
        torque_failure_witness=true;
        R.duration_25d_torque_failure_witnesses= ...
            R.duration_25d_torque_failure_witnesses+1;
        % A failed all-time torque proof feeds the SAME 25D local QP.
        % Do not confuse the sampled torque tangent with the certificate.
        local_torque_enabled=true;
        if isfield(req.search_options,'enable_25d_nominal_torque_local_rows')
            local_torque_enabled= ...
                isequal(req.search_options.enable_25d_nominal_torque_local_rows,1);
        end
        if ~local_torque_enabled
            R.status='UNKNOWN_C9_25D_NOMINAL_TORQUE_REPAIR_DISABLED';
            R.failure_reason=torque.status;break
        end
        if isfield(torque,'unresolved_interval_s')
            leaf=double(torque.unresolved_interval_s(:));
            if numel(leaf)==2&&all(isfinite(leaf))&& ...
                    leaf(2)>leaf(1)&&leaf(1)>=current.t0_s&& ...
                    leaf(2)<=current.t0_s+current.duration_s
                probes=leaf(1)+(leaf(2)-leaf(1))*[0 0.5 1];
                phases=unique([phases(:); ...
                    (probes(:)-current.t0_s)/current.duration_s]).';
            end
        end
    end
    % Extra phases are witness LOCATIONS, not a full-time certificate.
    if isfield(proof,'unresolved_time_interval_s')
        witness=double(proof.unresolved_time_interval_s(:));
        if numel(witness)==2&&all(isfinite(witness))&& ...
                witness(2)>witness(1)&&witness(1)>=current.t0_s&& ...
                witness(2)<=current.t0_s+current.duration_s
            ts=witness(1)+(witness(2)-witness(1))*[0.25 0.5 0.75];
            phases=unique([phases(:); ...
                (ts(:)-current.t0_s)/current.duration_s]).';
        end
    end
    if iteration>=req.search_options.max_outer_iterations
        R.status='UNKNOWN_C9_25D_OUTER_ITERATION_LIMIT';
        R.failure_reason=proof.status;break
    end
    local=req;
    % Only actual sensor-known shapes and current external radii may build
    % the one-ring support graph. Once built for THIS replanning call, all
    % 25D QP iterations reuse it; no Flight-precomputed obstacle patterns.
    cg_on=isfield(req.search_options,'enable_25d_group_support_corridor') && ...
          isequal(double(req.search_options.enable_25d_group_support_corridor),1);
    if cg_on && isempty(corridor_graph)
        graph=TRAJECTORY_C9_25D_OBSERVED_NEIGHBOR_GRAPH(shapes,rad, ...
            req.search_options.min_separation_m+ ...
            req.geometry_options.numerical_guard_m);
        R.duration_25d_group_graph_ms=graph.elapsed_ms;
        R.duration_25d_group_graph_pair_tests=graph.pair_tests;
        if ~graph.valid
            R.status='UNKNOWN_C9_25D_GROUP_GRAPH';
            R.failure_reason=graph.status;break
        end
        corridor_graph=graph.adjacency;
    end
    if cg_on
        local.search_options.group_corridor_neighbour_graph=corridor_graph;
    end
    local.duration_s=current.duration_s;
    local.nominal_free27=current.control_points_m(:,11:18);
    if lazy_torque
        permitted=true;
        if isfield(req.search_options,'enable_25d_nominal_torque_local_rows')
            permitted=isequal( ...
                req.search_options.enable_25d_nominal_torque_local_rows,1);
        end
        run_torque_search_rows=permitted && torque_failure_witness;
        local.search_options.enable_25d_nominal_torque_local_rows= ...
            double(run_torque_search_rows);
        if run_torque_search_rows
            R.duration_25d_lazy_torque_activated_qps= ...
                R.duration_25d_lazy_torque_activated_qps+1;
        elseif permitted
            R.duration_25d_lazy_torque_skipped_qps= ...
                R.duration_25d_lazy_torque_skipped_qps+1;
        end
    end
    % A failed all-time leaf is NOT a proof of collision.  The native
    % search-only witness bank supplies one mathematical midpoint per
    % different obstacle ID and the body that failed separation there.
    % Crucially, these extra times must NOT multiply every obstacle/body
    % row by every new time.  Only matching witness pairs get QP rows.
    qp_phases=phases;
    if isfield(proof,'unresolved_witness_bank') && ...
            isstruct(proof.unresolved_witness_bank) && ...
            ~isempty(proof.unresolved_witness_bank)
        bank=proof.unresolved_witness_bank;
        witness_times=NaN(numel(bank),1);
        for wi=1:numel(bank)
            sec=double(bank(wi).time_interval_s);
            if ~(numel(sec)==2 && all(isfinite(sec)) && ...
                  sec(1)>=current.t0_s && ...
                  sec(2)<=current.t0_s+current.duration_s && ...
                  sec(2)>sec(1) && ...
                  ismember(double(bank(wi).id),[shapes.id]) && ...
                  isequal(size(bank(wi).body_mask),[1 3]) && ...
                  any(bank(wi).body_mask))
                witness_times=[];break
            end
            witness_times(wi)=(mean(sec)-current.t0_s)/current.duration_s;
        end
        if ~isempty(witness_times)
            local.search_options.witness_bank_base_phases=phases;
            local.search_options.witness_bank_targeted_phases=witness_times(:).';
            local.search_options.witness_bank_targeted_ids=[bank.id];
            local.search_options.witness_bank_targeted_body_mask= ...
                vertcat(bank.body_mask);
            qp_phases=unique([phases(:);witness_times(:)]).';
            R.duration_25d_witness_bank_targeted_phases= ...
                R.duration_25d_witness_bank_targeted_phases+ ...
                numel(witness_times);
        end
    end
    % A cheap search-only 3D support scan catches missed collisions
    % BETWEEN the base temporal probes, one worst midpoint per obstacle.
    % It never certifies safety and does not relax any final full-time test.
    seed_on=false;
    if isfield(req.search_options,'enable_25d_support_risk_seed_bank')
        fl=req.search_options.enable_25d_support_risk_seed_bank;
        assert(isscalar(fl)&&any(double(fl)==[0 1]), ...
          'C9_25D:SeedFlag','Expected 0/1.');
        seed_on=logical(fl);
    end
    if seed_on && ~isempty(shapes)
        tscan=tic;
        W=TRAJECTORY_C9_25D_SUPPORT_RISK_SEEDS(current,shapes, ...
            initial_risk_probe_phases,rad,req.authority, ...
            req.search_options.min_separation_m, ...
            req.geometry_options.numerical_guard_m);
        R.duration_25d_risk_seed_calls=R.duration_25d_risk_seed_calls+1;
        R.duration_25d_risk_seed_ms=R.duration_25d_risk_seed_ms+1000*toc(tscan);
        R.duration_25d_risk_seed_status=W.status;
        R.duration_25d_iteration_diagnostics(end).risk_seed_status=W.status;
        R.duration_25d_iteration_diagnostics(end).risk_seed_count=numel(W.ids);
        R.duration_25d_iteration_diagnostics(end).risk_seed_ms=1000*toc(tscan);
        if isfield(W,'support_batch_calls')
           R.duration_25d_risk_seed_batches= ...
               R.duration_25d_risk_seed_batches+W.support_batch_calls;
        end
        if isfield(W,'support_evaluations')
           R.duration_25d_risk_seed_evaluations= ...
               R.duration_25d_risk_seed_evaluations+W.support_evaluations;
        end
        if isfield(W,'degenerate_directions')
           R.duration_25d_risk_seed_degenerate_directions= ...
               R.duration_25d_risk_seed_degenerate_directions+W.degenerate_directions;
        end

        if W.valid && ~isempty(W.ids)
            % Do not multiply temporal probes at each outer iteration.
            % Existing witness phases are already handled by the base QP;
            % no additional row is needed at the identical time.
            % Machine-rounding-equivalent probe times must not create
            % duplicate SEARCH rows (no safety tolerance is modified).
            tol=32*eps(max([1;abs(W.phases(:));abs(phases(:))]));
            fresh=~any(abs(W.phases(:)-phases(:).')<=tol,2);
            W.phases=W.phases(fresh);
            W.ids=W.ids(fresh);
            W.body_mask=W.body_mask(fresh,:);
        end
        if W.valid && ~isempty(W.ids)
            if ~isfield(local.search_options,'witness_bank_targeted_phases')
                local.search_options.witness_bank_base_phases=phases;
                local.search_options.witness_bank_targeted_phases=W.phases.';
                local.search_options.witness_bank_targeted_ids=W.ids.';
                local.search_options.witness_bank_targeted_body_mask=W.body_mask;
            else
                local.search_options.witness_bank_targeted_phases= ...
                    [local.search_options.witness_bank_targeted_phases,W.phases.'];
                local.search_options.witness_bank_targeted_ids= ...
                    [local.search_options.witness_bank_targeted_ids,W.ids.'];
                local.search_options.witness_bank_targeted_body_mask= ...
                    [local.search_options.witness_bank_targeted_body_mask;W.body_mask];
            end
            qp_phases=unique([qp_phases(:);W.phases(:)]).';
            R.duration_25d_risk_seed_rows= ...
                R.duration_25d_risk_seed_rows+numel(W.ids);
        end
    end
    tq=tic;
    Q=TRAJECTORY_C9_25D_NATURAL_QP( ...
        local,shapes,qp_phases,rad,ground_active,proof);
    R.duration_25d_plane_ms=R.duration_25d_plane_ms+1000*toc(tq);
    R.duration_25d_group_corridor_count= ...
        R.duration_25d_group_corridor_count+Q.group_corridor_count;
    R.duration_25d_group_corridor_ms= ...
        R.duration_25d_group_corridor_ms+Q.group_corridor_ms;
    R.duration_25d_group_corridor_removed_scalar_rows= ...
        R.duration_25d_group_corridor_removed_scalar_rows+ ...
        Q.group_corridor_removed_scalar_rows;
    R.duration_25d_group_corridor_added_rows= ...
        R.duration_25d_group_corridor_added_rows+Q.group_corridor_added_rows;
    R.duration_25d_group_corridor_candidate_directions= ...
        R.duration_25d_group_corridor_candidate_directions+ ...
        Q.group_corridor_candidate_directions;
    R.duration_25d_group_corridor_support_evaluations= ...
        R.duration_25d_group_corridor_support_evaluations+ ...
        Q.group_corridor_support_evaluations;
    R.duration_25d_witness_bank_targeted_rows= ...
        R.duration_25d_witness_bank_targeted_rows+Q.targeted_witness_rows;
    R.duration_25d_torque_tangent_ms= ...
        R.duration_25d_torque_tangent_ms+Q.torque_tangent_ms;
    R.duration_25d_torque_point_rows= ...
        R.duration_25d_torque_point_rows+Q.torque_point_rows;
    R.duration_25d_physical_leaf_rows= ...
        R.duration_25d_physical_leaf_rows+Q.physical_leaf_rows;
    R.duration_25d_last_qp_construction_status=Q.status;
    R.duration_25d_iteration_diagnostics(end).QP_build_status=Q.status;
    R.duration_25d_iteration_diagnostics(end).corridor_groups= ...
        Q.group_corridor_count;
    R.duration_25d_iteration_diagnostics(end).corridor_rows= ...
        Q.group_corridor_added_rows;
    R.duration_25d_iteration_diagnostics(end).corridor_status= ...
        Q.group_corridor_status;
    R.duration_25d_iteration_diagnostics(end).corridor_ms= ...
        Q.group_corridor_ms;
    R.duration_25d_iteration_diagnostics(end).QP_build_ms=1000*toc(tq);

    R.duration_25d_metric_ms=R.duration_25d_metric_ms+Q.metric_ms;
    R.duration_25d_tangent_ms=R.duration_25d_tangent_ms+Q.tangent_ms;
    R.duration_25d_geometry_rows_ms= ...
        R.duration_25d_geometry_rows_ms+Q.geometry_rows_ms;
    R.duration_25d_leaf_rows_ms=R.duration_25d_leaf_rows_ms+Q.leaf_rows_ms;
    R.duration_25d_native_torque_calls= ...
        R.duration_25d_native_torque_calls+double(Q.native_25d_torque_executed);
    R.duration_25d_native_torque_ms= ...
        R.duration_25d_native_torque_ms+Q.native_25d_torque_ms;
    R.duration_25d_optional_leaf_skips= ...
        R.duration_25d_optional_leaf_skips+double(Q.optional_leaf_skipped);
    R.duration_25d_optional_torque_skips= ...
        R.duration_25d_optional_torque_skips+ ...
        double(Q.optional_torque_tangent_skipped);
    if ~Q.solved
        R.status='UNKNOWN_C9_25D_QP_CONSTRUCTION';
        R.failure_reason=Q.status;break
    end
    tproj=tic;
    [z,S]=TRAJECTORY_C9_JOINT_SCREENED_PROJECTION( ...
        Q.A,Q.b,req.search_options.max_projection_sweeps, ...
        req.search_options.projection_tolerance_m);
    R.duration_25d_qp_ms=R.duration_25d_qp_ms+1000*toc(tproj);
    R.duration_25d_qp_count=R.duration_25d_qp_count+1;
    R.duration_25d_qp_max_violation_history(end+1)=S.maximum_primal_violation;
    R.joint_qp_status=S.status;
    R.duration_25d_iteration_diagnostics(end).QP_solve_status=S.status;
    R.duration_25d_iteration_diagnostics(end).QP_solve_ms=1000*toc(tproj);

    if ~S.solved
        R.status='UNKNOWN_C9_25D_QP_NOT_SOLVED';
        R.failure_reason=S.status;break
    end
    dx=Q.chol_upper\z;
    R.duration_25d_iteration_diagnostics(end).step_norm=norm(dx);

    newT=current.duration_s+dx(25);
    newF=local.nominal_free27+reshape(dx(1:24),3,8);
    if ~(isfinite(newT)&&newT>0&&newT<=Tmax&& ...
            all(isfinite(newF(:))))
        R.status='UNKNOWN_C9_25D_QP_STEP_OUTSIDE_FLIGHT';
        R.failure_reason='Linearized QP step did not remain in physical domain.';
        break
    end
    x=[newF(:);newT];
    if isequal(x,[local.nominal_free27(:);local.duration_s]) || ...
            any(all(seen==x,1))
        R.status='UNKNOWN_C9_25D_QP_NO_PROGRESS';
        R.failure_reason=proof.status;break
    end
    seen(:,end+1)=x; %#ok<AGROW>
    current=TRAJECTORY_C9_FLAT_BERNSTEIN.build27( ...
        req.t0_s,newT,req.start_jets,req.end_jets,newF);
    R.duration_25d_candidate_history_s(end+1)=newT;
end
R.independent_proof_time_total_ms=R.duration_25d_proof_ms;
R.plane_construction_time_total_ms=R.duration_25d_plane_ms;
R.joint_qp_time_total_ms=R.duration_25d_qp_ms;
R.elapsed_ms=1000*toc(clock);
R.safe_to_commit=false;
end

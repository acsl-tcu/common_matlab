function R=TRAJECTORY_C9_13D_TIME_JOINT_SEARCH(req,R)
% Exclusive SINGLE-polynomial 13-variable C9 search, research-only.
% x=[four free 3D Bernstein controls; physical duration]. No discrete
% multistage path, no CBF/AP, and no fixed avoidance direction.
% A nonlinear update is NEVER accepted from local linearization alone:
% the independent full-time geometry/tension/thrust oracle runs on each
% candidate; nominal rigid-body torque proof runs only after that passes.
% Real HLC tracking/actuators and directed-rounding bounds remain pending.
% Numeric evaluations do not authorize HLC COMMIT.
start=tic;
R.safe_to_commit=false;R.candidate_available=false;
R.duration_13d_enabled=true;
R.duration_13d_qp_count=0;
R.duration_13d_qp_ms=0;
R.duration_13d_plane_ms=0;
R.duration_13d_proof_ms=0;
R.duration_13d_inherited_shape_skips=0;
R.duration_13d_parent_shape_certificates=0;
R.duration_13d_parent_outer_clearances=0;
R.duration_13d_torque_ms=0;
R.duration_13d_metric_status='NOT_RUN';
R.duration_13d_candidate_history_s=double(req.duration_s);
R.duration_13d_nominal_torque_status='NOT_RUN';
% Rejection-only numerical necessary-condition gate, NOT a safety oracle.
R.duration_13d_pre_torque_screen_enabled=false;
R.duration_13d_pre_torque_screen_calls=0;
R.duration_13d_pre_torque_screen_ms=0;
R.duration_13d_pre_torque_screen_cache_hits=0;
R.duration_13d_pre_torque_screen_witnesses=0;
R.duration_13d_pre_torque_saved_oracle_calls=0;
R.duration_13d_witness_plane_replacements=0;
R.duration_13d_witness_support_calls=0;
R.duration_13d_witness_ms=0;
R.duration_13d_coherent_plane_groups=0;
R.duration_13d_coherent_fallback_groups=0;
R.duration_13d_coherent_support_evaluations=0;
R.duration_13d_qp_outer_ball_enabled=false;
R.duration_13d_shared_jet_uses=0;
R.duration_13d_native_torque_batch_uses=0;
R.duration_13d_native_torque_batch_ms=0;
R.duration_13d_native_separator_proposals=0;
R.duration_13d_batched_qp_uses=0;
R.duration_13d_batched_qp_ms=0;
R.duration_13d_batched_qp_rows=0;
R.duration_13d_certificate_leaf_cuts=0;
R.duration_13d_certificate_leaf_support_calls=0;
R.duration_13d_body_time_focus_qp_uses=0;
R.duration_13d_body_time_outer_guard_uses=0;
R.duration_13d_body_time_outer_guard_rejections=0;
R.duration_13d_body_time_outer_guard_far_checks=0;
R.duration_13d_body_time_outer_guard_near_checks=0;
R.duration_13d_body_time_focus_skipped_shape_phase_groups=0;
R.duration_13d_body_time_focus_skipped_body_planes=0;
R.duration_13d_qp_outer_ball_groups_tested=0;
R.duration_13d_qp_outer_ball_groups_skipped=0;
% Oracle-driven ACTIVE constraint generation: only QP rows are filtered;
% the independent continuous-time oracle ALWAYS receives every obstacle.
R.duration_13d_active_shape_focus_enabled=false;
% Geometry was already certified but torque needs correction: optional
% QP constraint-generation branch. The NEXT oracle always checks ALL shapes.
R.duration_13d_torque_only_qp_enabled=false;
R.duration_13d_torque_only_qp_uses=0;
R.duration_13d_torque_only_qp_skipped_planes_upper=0;
R.duration_13d_torque_repair_point_witnesses=0;
R.duration_13d_torque_repair_qp_attempts=0;
R.duration_13d_active_shape_ids=[];
R.duration_13d_active_shape_counts=[];
R.duration_13d_skipped_qp_shape_planes_upper=0;
% Optional execution service budget (not a hard execution-time guarantee).
R.duration_13d_execution_budget_ms=Inf;
R.duration_13d_execution_budget_exhausted=false;
R.duration_13d_max_oracle_call_ms=0;
R.duration_13d_max_qp_build_call_ms=0;
R.duration_13d_max_line_search_call_ms=0;
R.duration_13d_max_torque_call_ms=0;
R.duration_13d_last_oracle_status='NOT_RUN';
R.duration_13d_last_unresolved_obstacle_ids=[];
R.duration_13d_last_unresolved_body_indices=[];
R.duration_13d_last_unresolved_interval_s=[NaN NaN];
R.duration_13d_free_step_norm_history_m=[];
R.duration_13d_qp_max_violation_history=[];
R.duration_13d_linesearch_enabled=false;
R.duration_13d_linesearch_ms=0;
R.duration_13d_interval_hull_enabled=false;
R.duration_13d_point_torque_prefilter_enabled=false;
R.duration_13d_point_torque_prefilter_calls=0;
R.duration_13d_point_torque_prefilter_skips=0;
R.duration_13d_point_torque_prefilter_ms=0;
R.duration_13d_interval_hull_evaluations=0;
R.duration_13d_interval_hull_rejected_trials=0;
R.duration_13d_interval_hull_before_after_m=zeros(0,2);
R.duration_13d_linesearch_backtracks=0;
R.duration_13d_linesearch_alpha_history=[];
R.duration_13d_point_merit_before_after_m=zeros(0,2);
R.duration_13d_max_unrestricted_step_norm_m=0;
R.duration_13d_conditional_nominal_torque_proven=false;
% Same-invocation evidence transfer ONLY. The integrated pipeline MUST
% validate exact equality of piece/authority/options before using it.
R.duration_13d_nominal_torque_proof_bundle=struct();
R.scope='ONE_C9_13D_SIMULTANEOUS_FULLBODY_SEARCH__HLC_NOT_CERTIFIED';
% Reuse exactly the independently validated shape snapshot from the
% full-time oracle. Preparing it twice is redundant and can drift.
shapes=[];
radii=req.radii;
rad=[double(radii.uav_m),double(radii.cable_m),double(radii.payload_m)];
if req.geometry_options.robust_tube_enabled
    u=req.geometry_options.uncertainty_bounds_m;
    rad=rad+[u.uav_m,u.cable_m,u.payload_m];
end
assert(all(isfinite(rad))&&all(rad>=0), ...
    'C9_13D:Radius','Invalid physical body/tube radius');
[ground_active,phase]=TRAJECTORY_GROUND_SCOPE(req.geometry_options);
if ~strcmp(phase,'f')
    R.status='UNKNOWN_13D_NOT_FLIGHT';R.elapsed_ms=1000*toc(start);return
end
Tmax=req.flight_interval_s(2)-req.t0_s;
if ~(isfinite(Tmax)&&Tmax>0&&req.duration_s>0&&req.duration_s<=Tmax)
    R.status='UNKNOWN_13D_FLIGHT_TIME_DOMAIN';
    R.elapsed_ms=1000*toc(start);return
end
current=TRAJECTORY_C9_FLAT_BERNSTEIN.build(req.t0_s,req.duration_s, ...
    req.start_jets,req.end_jets,req.nominal_free);
C0=current;
seen=zeros(13,0);
active_shape_ids=[];
use_focus=false;
if isfield(req.search_options,'enable_13d_oracle_focused_qp_shapes')
    v=req.search_options.enable_13d_oracle_focused_qp_shapes;
    assert((isnumeric(v)||islogical(v))&&isscalar(v)&& ...
        isreal(v)&&isfinite(double(v))&&any(double(v)==[0 1]), ...
        'C9_13D:FocusFlag','Oracle focus flag must be 0/1.');
    use_focus=logical(v);
end
R.duration_13d_active_shape_focus_enabled=use_focus;
torque_only=false;
if isfield(req.search_options,'enable_13d_torque_only_qp_after_geometry_pass')
    v=req.search_options.enable_13d_torque_only_qp_after_geometry_pass;
    assert((isnumeric(v)||islogical(v))&&isscalar(v)&&isreal(v)&& ...
        isfinite(double(v))&&any(double(v)==[0 1]), ...
        'C9_13D:TorqueOnlyFlag','Torque-only constraint generation must be 0/1.');
    torque_only=logical(v);
end
R.duration_13d_torque_only_qp_enabled=torque_only;
budget_ms=Inf;
if isfield(req.search_options,'max_13d_search_elapsed_ms')
    budget_ms=double(req.search_options.max_13d_search_elapsed_ms);
    assert(isscalar(budget_ms)&&isreal(budget_ms)&& ...
        isfinite(budget_ms)&&budget_ms>0, ...
        'C9_13D:Budget','Explicit wall-clock service budget must be positive.');
end
R.duration_13d_execution_budget_ms=budget_ms;
% Chebyshev-Lobatto probes are search locations ONLY, not certificate
% sample points or assumed collision-free intervals.
N=req.search_options.probe_count;
phases=(1-cos(pi*(0:N-1)/(N-1)))/2;
for iter=0:req.search_options.max_outer_iterations
    R.search_iterations=iter;
    if 1000*toc(start)>=budget_ms
        R.status='UNKNOWN_13D_EXECUTION_BUDGET_EXHAUSTED';
        R.failure_reason='Search budget exhausted; no new certificate/COMMIT.';
        R.duration_13d_execution_budget_exhausted=true;
        break
    end
    % Necessary-only numerical screen: if the CURRENT C9 candidate is
    % already outside a nominal torque limit at any search probe, it
    % cannot receive an all-time nominal torque PASS. Skip the expensive
    % GEOMETRY oracle on that iteration, and repair using the SAME QP.
    % This can only REJECT candidates, never certify or COMMIT one.
    % Disabled by default pending full 15-case MATLAB timing/coverage A/B.
    use_screen=false;
    if isfield(req.search_options,'enable_13d_torque_point_proof_prefilter')
        v=req.search_options.enable_13d_torque_point_proof_prefilter;
        assert((islogical(v)||isnumeric(v))&&isscalar(v)&& ...
            isreal(v)&&isfinite(double(v))&&any(double(v)==[0 1]), ...
            'C9_13D:PrefilterFlag','Point prefilter must be 0/1.');
        use_screen=logical(v);
    end
    R.duration_13d_point_torque_prefilter_enabled=use_screen;
    skipped=false;
    % Reset per-iteration: a prior candidate's point violation must NEVER
    % authorize a new candidate's torque-only local QP or its certificate.
    point_witness=false;
    point_witness_count=0;
    % This cached point snapshot is confined to the CURRENT loop
    % iteration: no candidate/free controls, phases or authority change
    % between the numerical pre-oracle screen and the torque stage.
    point_cache_valid=false;
    if use_screen && iter>0 && ~isempty(shapes) && ...
            isfield(req,'torque_options') && isstruct(req.torque_options)
        tm=tic;
        R.duration_13d_point_torque_prefilter_calls= ...
            R.duration_13d_point_torque_prefilter_calls+1;
        pt=TRAJECTORY_C9_TORQUE_POINT_EVALUATE( ...
            current,req.authority,req.torque_options, ...
            current.t0_s+current.duration_s*phases);
        point_cache_valid=true;
        R.duration_13d_point_torque_prefilter_ms= ...
            R.duration_13d_point_torque_prefilter_ms+1000*toc(tm);
        skipped=numel(pt.valid)==numel(pt.outside_limits) && ...
            any(pt.valid(:)&pt.outside_limits(:));
    end
    if skipped
        R.duration_13d_point_torque_prefilter_skips= ...
            R.duration_13d_point_torque_prefilter_skips+1;
        oracle=struct('certified',false, ...
            'status','SKIPPED_FULL_TIME_GEOMETRY_FOR_POINT_TORQUE_NECESSITY', ...
            'unresolved_obstacle_ids',[], ...
            'unresolved_body_indices',[], ...
            'unresolved_time_interval_s',[NaN NaN], ...
            'elapsed_ms',0);
    else
        t_oracle=tic;
        [oracle,validated_shapes]=TRAJECTORY_C9_JOINT_FAST_CERTIFICATE( ...
            current,req.observed,req.radii,req.authority, ...
            req.geometry_options,req.force_options);
        R.duration_13d_max_oracle_call_ms=max( ...
            R.duration_13d_max_oracle_call_ms,1000*toc(t_oracle));
        shapes=validated_shapes;
        R.independent_proof_calls=R.independent_proof_calls+1;
        R.duration_13d_proof_ms=R.duration_13d_proof_ms+oracle.elapsed_ms;
        if isfield(oracle,'native_separator_proposals')
            R.duration_13d_native_separator_proposals= ...
                R.duration_13d_native_separator_proposals+ ...
                oracle.native_separator_proposals;
        end
        if isfield(oracle,'ancestral_shape_skipped_node_checks')
            R.duration_13d_inherited_shape_skips= ...
                R.duration_13d_inherited_shape_skips+ ...
                oracle.ancestral_shape_skipped_node_checks;
            R.duration_13d_parent_shape_certificates= ...
                R.duration_13d_parent_shape_certificates+ ...
                oracle.ancestral_shape_certified_parent_clears;
            R.duration_13d_parent_outer_clearances= ...
                R.duration_13d_parent_outer_clearances+ ...
                oracle.ancestral_shape_inherited_outer_clears;
        end
    end
    R.final_certificate=oracle;
    R.duration_13d_last_oracle_status=oracle.status;
    % Do not start another expensive stage after a consumed service budget.
    % A single indivisible oracle call MAY exceed the budget. This is a
    % fail-closed soft deadline, NOT a hard real-time or safety certificate.
    if 1000*toc(start)>=budget_ms
        R.status='UNKNOWN_13D_EXECUTION_BUDGET_EXHAUSTED';
        R.failure_reason=oracle.status;
        R.duration_13d_execution_budget_exhausted=true;
        break
    end
    R.duration_13d_last_unresolved_obstacle_ids=oracle.unresolved_obstacle_ids;
    R.duration_13d_last_unresolved_body_indices=oracle.unresolved_body_indices;
    R.duration_13d_last_unresolved_interval_s=oracle.unresolved_time_interval_s;
    if iter==0,R.initial_certificate=oracle;end
    if oracle.certified
        % No nominal 3-axis torque assertion from geometry alone.
        if ~isfield(req,'torque_options') || ~isstruct(req.torque_options)
            R.status='UNKNOWN_13D_GEOMETRY_PASS_TORQUE_AUTHORITY_MISSING';
            R.failure_reason='Nominal 3-axis torque proof needs current HLC limits.';
            break
        end
        % A single VALID nominal input violating the HLC physical limit
        % at a search probe is a NECESSARY failure of the all-time limit.
        % This is a numerical REJECTION-ONLY optimization; it cannot mark
        % the candidate safe. A singular/invalid point, even if some other
        % probes are valid, is not treated as a PASS or a witness.
        % Default OFF: turning it ON is an explicit search-performance A/B.
        gate_on=false;
        if isfield(req.search_options,'enable_13d_pre_torque_point_rejection')
            sw=req.search_options.enable_13d_pre_torque_point_rejection;
            assert((islogical(sw)||isnumeric(sw))&&isscalar(sw)&& ...
                isreal(sw)&&isfinite(double(sw))&&any(double(sw)==[0 1]), ...
                'C9_13D:PreTorqueScreenFlag','Expected explicit 0/1 switch');
            gate_on=logical(sw);
        end
        R.duration_13d_pre_torque_screen_enabled=gate_on;
        point_witness=false;
        if gate_on
            R.duration_13d_pre_torque_screen_calls= ...
                R.duration_13d_pre_torque_screen_calls+1;
            if point_cache_valid
                % Same-invocation, same candidate and same probe times.
                % Count this in pre-oracle time, never twice.
                R.duration_13d_pre_torque_screen_cache_hits= ...
                    R.duration_13d_pre_torque_screen_cache_hits+1;
            else
                pt_clock=tic;
                pt=TRAJECTORY_C9_TORQUE_POINT_EVALUATE( ...
                    current,req.authority,req.torque_options, ...
                    current.t0_s+current.duration_s*phases);
                R.duration_13d_pre_torque_screen_ms= ...
                    R.duration_13d_pre_torque_screen_ms+1000*toc(pt_clock);
            end
            if numel(pt.valid)==numel(pt.outside_limits)
                % An invalid probe is ignored, not inferred to be safe.
                point_witness=any(pt.valid(:)&pt.outside_limits(:));
                point_witness_count=sum(pt.valid(:)&pt.outside_limits(:));
            end
            if 1000*toc(start)>=budget_ms
                R.status='UNKNOWN_13D_EXECUTION_BUDGET_EXHAUSTED';
                R.failure_reason='Necessary-point screen exhausted service budget.';
                R.duration_13d_execution_budget_exhausted=true;
                break
            end
        end
        if point_witness
            R.duration_13d_pre_torque_screen_witnesses= ...
                R.duration_13d_pre_torque_screen_witnesses+ ...
                point_witness_count;
            R.duration_13d_pre_torque_saved_oracle_calls= ...
                R.duration_13d_pre_torque_saved_oracle_calls+1;
            R.duration_13d_nominal_torque_status= ...
                'NOT_CERTIFIED__NUMERIC_POINT_TORQUE_LIMIT_WITNESS';
            R.duration_13d_torque_witness_count=point_witness_count;
            % Continue to SAME 13D QP. All final PASS still requires the
            % original continuous-time torque interval certificate.
        else
            ttorque=tic;
            C=TRAJECTORY_C9_NOMINAL_TORQUE_INTERVAL( ...
                current,req.authority,req.torque_options);
            torque_call_ms=1000*toc(ttorque);
            R.duration_13d_torque_ms=R.duration_13d_torque_ms+torque_call_ms;
            R.duration_13d_max_torque_call_ms=max( ...
                R.duration_13d_max_torque_call_ms,torque_call_ms);
            R.duration_13d_nominal_torque_status=C.status;
            R.duration_13d_conditional_nominal_torque_proven= ...
                C.conditional_nominal_all_time_torque_proven;
            if 1000*toc(start)>=budget_ms
                R.status='UNKNOWN_13D_EXECUTION_BUDGET_EXHAUSTED';
                R.failure_reason='Torque call exhausted the available service budget.';
                R.duration_13d_execution_budget_exhausted=true;
                break
            end
            if C.conditional_nominal_all_time_torque_proven
                % Only nominal real-arithmetic candidate available.
                R.status='CONDITIONAL_13D_NOMINAL_GEOMETRY_TORQUE__HLC_PENDING';
                R.candidate_available=true;
                R.candidate_piece=current;
                R.duration_s=current.duration_s;
                % Retain the ACTUAL successful all-time interval proof,
                % not just its Boolean. No recomputation or estimated bounds.
                % The caller verifies every exact input prior to reuse.
                R.duration_13d_nominal_torque_proof_bundle=struct( ...
                    'piece',current,'authority',req.authority, ...
                    'options',req.torque_options,'proof',C, ...
                    'proof_call_ms',torque_call_ms);
                R.failure_reason='Actual HLC tube/4-input and rounding not certified.';
                break
            end
            % Feed a FAILED all-time torque certificate into the SAME 13D
            % search. No point test can turn UNKNOWN into a PASS.
            leaf=double(C.unresolved_interval_s(:));
            if numel(leaf)==2 && all(isfinite(leaf)) && ...
                    leaf(2)>=leaf(1) && leaf(1)>=current.t0_s && ...
                    leaf(2)<=current.t0_s+current.duration_s
                ts=[leaf(1),(leaf(1)+leaf(2))/2,leaf(2)];
                us=(ts-current.t0_s)/current.duration_s;
                phases=unique([phases(:);us(:)]).';
            end
            point=TRAJECTORY_C9_TORQUE_POINT_EVALUATE( ...
                current,req.authority,req.torque_options, ...
                current.t0_s+current.duration_s*phases);
            if numel(point.valid)~=numel(point.outside_limits) || ...
                    ~any(point.valid(:)&point.outside_limits(:))
                R.status='UNKNOWN_13D_TORQUE_LEAF_WITHOUT_POINT_WITNESS';
                R.failure_reason=C.status;
                break
            end
            % This was missing: the interval oracle's unresolved leaf
            % produced a VALID numerical torque violation, but the local
            % torque-repair branch previously only inspected the unrelated
            % pre-oracle screening flag. Carry the CURRENT point evidence
            % into the SAME 13D QP; it is a rejection/repair hint only.
            % It NEVER proves torque safe for all time.
            point_witness=true;
            point_witness_count=sum(point.valid(:)&point.outside_limits(:));
            R.duration_13d_torque_witness_count=point_witness_count;
        end % point_witness: otherwise original full-time torque oracle
    end
    if ~oracle.certified && isfield(oracle,'unresolved_time_interval_s')
        % All shapes and all bodies are reconsidered at these search
        % probes. They are NEVER used as continuous-time certificates.
        leaf=double(oracle.unresolved_time_interval_s(:));
        if numel(leaf)==2 && all(isfinite(leaf)) && ...
                leaf(2)>leaf(1) && leaf(1)>=current.t0_s && ...
                leaf(2)<=current.t0_s+current.duration_s
            w=leaf(1)+(leaf(2)-leaf(1))*[0.25 0.50 0.75];
            us=(w-current.t0_s)/current.duration_s;
            phases=unique([phases(:);us(:)]).';
        end
    end
    if iter==req.search_options.max_outer_iterations
        R.status='UNKNOWN_13D_ITERATION_LIMIT';
        if oracle.certified && exist('point_witness','var') && point_witness
            R.failure_reason='NUMERIC_POINT_TORQUE_LIMIT_WITNESS__NO_ALL_TIME_CERT';
        else
            R.failure_reason=oracle.status;
        end
        break
    end
    if 1000*toc(start)>=budget_ms
        R.status='UNKNOWN_13D_EXECUTION_BUDGET_EXHAUSTED';
        R.failure_reason='No time left for local QP; no candidate committed.';
        R.duration_13d_execution_budget_exhausted=true;
        break
    end
    % An oracle failure identifies obstacles that currently obstruct
    % certification. Grow a monotone ACTIVE set from these exact IDs.
    % The local 13D QP need not have planes from every safe obstacle;
    % after every update, the INDEPENDENT ORACLE checks ALL obstacles.
    % This is constraint generation, not reduced safety certification.
    qp_shapes=shapes;
    % Only after the independent FULL-time oracle has already proved
    % geometry, tension and thrust for CURRENT piece, torque correction
    % may omit obstacle rows from the LOCAL QP. This never declares the
    % updated piece safe. At the next iteration the UNFILTERED oracle
    % verifies every observed obstacle and the ground again. If its
    % geometry fails, normal active-set constraint generation resumes.
    % A full-time geometry/tension/thrust PASS and a CURRENT valid
    % point torque witness are necessary for this search-only omission.
    % The next iteration STILL re-proves ALL observed shapes and ground.
    torque_repair=torque_only && oracle.certified && point_witness;
    if torque_repair
        R.duration_13d_torque_repair_point_witnesses= ...
            R.duration_13d_torque_repair_point_witnesses+point_witness_count;
        R.duration_13d_torque_repair_qp_attempts= ...
            R.duration_13d_torque_repair_qp_attempts+1;
        qp_shapes=shapes([]);
        R.duration_13d_torque_only_qp_uses= ...
            R.duration_13d_torque_only_qp_uses+1;
        R.duration_13d_torque_only_qp_skipped_planes_upper= ...
            R.duration_13d_torque_only_qp_skipped_planes_upper+ ...
            3*numel(shapes)*numel(phases);
    end
    if ~torque_repair && use_focus && ~isempty(shapes) && ...
            strcmp(oracle.status,'UNKNOWN_JOINT_GEOMETRY_NOT_SEPARATED')
        ids=double(oracle.unresolved_obstacle_ids(:)).';
        all_ids=arrayfun(@(q)double(q.id),shapes);
        ids=ids(isfinite(ids)&ismember(ids,all_ids));
        if ~isempty(ids)
            active_shape_ids=unique([active_shape_ids,ids],'stable');
        end
    end
    if ~torque_repair && use_focus && ~isempty(active_shape_ids) && ~isempty(shapes)
        all_ids=arrayfun(@(q)double(q.id),shapes);
        selected=ismember(all_ids,active_shape_ids);
        if any(selected)
            qp_shapes=shapes(selected);
        end
    end
    R.duration_13d_active_shape_ids=active_shape_ids;
    R.duration_13d_active_shape_counts(end+1)=numel(qp_shapes);
    R.duration_13d_skipped_qp_shape_planes_upper= ...
        R.duration_13d_skipped_qp_shape_planes_upper + ...
        (numel(shapes)-numel(qp_shapes))*numel(phases)*3;
    tplane=tic;
    local=req;
    local.duration_s=current.duration_s;
    local.nominal_free=current.control_points_m(:,11:14);
    Q=TRAJECTORY_C9_13D_NATURAL_QP( ...
        local,qp_shapes,phases,rad,ground_active,oracle,iter);
    R.duration_13d_shared_jet_uses= ...
        R.duration_13d_shared_jet_uses+double(Q.shared_jet_executed);
    R.duration_13d_native_torque_batch_uses= ...
        R.duration_13d_native_torque_batch_uses+ ...
        double(Q.native_torque_batch_executed);
    R.duration_13d_native_torque_batch_ms= ...
        R.duration_13d_native_torque_batch_ms+Q.native_torque_batch_ms;
    R.duration_13d_batched_qp_uses= ...
        R.duration_13d_batched_qp_uses+double(Q.batched_geometry_executed);
    R.duration_13d_batched_qp_ms= ...
        R.duration_13d_batched_qp_ms+Q.batched_geometry_ms;
    R.duration_13d_batched_qp_rows= ...
        R.duration_13d_batched_qp_rows+Q.batched_geometry_row_count;
    R.duration_13d_certificate_leaf_cuts= ...
        R.duration_13d_certificate_leaf_cuts+Q.certificate_leaf_cut_count;
    R.duration_13d_certificate_leaf_support_calls= ...
        R.duration_13d_certificate_leaf_support_calls+ ...
        Q.certificate_leaf_cut_support_calls;
    R.duration_13d_body_time_outer_guard_uses= ...
        R.duration_13d_body_time_outer_guard_uses+ ...
        double(Q.oracle_body_time_outer_guard_checked);
    R.duration_13d_body_time_outer_guard_rejections= ...
        R.duration_13d_body_time_outer_guard_rejections+ ...
        double(Q.oracle_body_time_outer_guard_rejected);
    R.duration_13d_body_time_outer_guard_far_checks= ...
        R.duration_13d_body_time_outer_guard_far_checks+ ...
        Q.oracle_body_time_outer_guard_far_body_checks;
    R.duration_13d_body_time_outer_guard_near_checks= ...
        R.duration_13d_body_time_outer_guard_near_checks+ ...
        Q.oracle_body_time_outer_guard_near_body_checks;
    R.duration_13d_body_time_focus_qp_uses= ...
        R.duration_13d_body_time_focus_qp_uses+ ...
        double(Q.oracle_body_time_focus_active);
    R.duration_13d_body_time_focus_skipped_shape_phase_groups= ...
        R.duration_13d_body_time_focus_skipped_shape_phase_groups+ ...
        Q.oracle_body_time_focus_skipped_shape_phase_groups;
    R.duration_13d_body_time_focus_skipped_body_planes= ...
        R.duration_13d_body_time_focus_skipped_body_planes+ ...
        Q.oracle_body_time_focus_skipped_body_planes;
    R.duration_13d_witness_plane_replacements= ...
        R.duration_13d_witness_plane_replacements+Q.witness_plane_replacements;
    R.duration_13d_witness_support_calls= ...
        R.duration_13d_witness_support_calls+Q.witness_support_calls;
    R.duration_13d_witness_ms=R.duration_13d_witness_ms+Q.witness_plane_build_ms;
    R.duration_13d_coherent_plane_groups= ...
        R.duration_13d_coherent_plane_groups+Q.coherent_plane_groups;
    R.duration_13d_coherent_fallback_groups= ...
        R.duration_13d_coherent_fallback_groups+Q.coherent_fallback_groups;
    R.duration_13d_coherent_support_evaluations= ...
        R.duration_13d_coherent_support_evaluations+Q.coherent_support_evaluations;
    R.duration_13d_qp_outer_ball_enabled=Q.qp_outer_ball_enabled;
    R.duration_13d_qp_outer_ball_groups_tested= ...
        R.duration_13d_qp_outer_ball_groups_tested+Q.qp_outer_ball_groups_tested;
    R.duration_13d_qp_outer_ball_groups_skipped= ...
        R.duration_13d_qp_outer_ball_groups_skipped+Q.qp_outer_ball_groups_skipped;
    qp_build_call_ms=1000*toc(tplane);
    R.duration_13d_plane_ms=R.duration_13d_plane_ms+qp_build_call_ms;
    R.duration_13d_max_qp_build_call_ms=max( ...
        R.duration_13d_max_qp_build_call_ms,qp_build_call_ms);
    R.duration_13d_metric_status=Q.status;
    if ~Q.solved
        R.status='UNKNOWN_13D_QP_MODEL_UNAVAILABLE';
        R.failure_reason=Q.status;break
    end
    if 1000*toc(start)>=budget_ms
        R.status='UNKNOWN_13D_EXECUTION_BUDGET_EXHAUSTED';
        R.failure_reason='No time left for QP projection.';
        R.duration_13d_execution_budget_exhausted=true;
        break
    end
    tqp=tic;
    [z,S]=TRAJECTORY_C9_JOINT_SCREENED_PROJECTION( ...
        Q.A,Q.b,req.search_options.max_projection_sweeps, ...
        req.search_options.projection_tolerance_m);
    R.duration_13d_qp_ms=R.duration_13d_qp_ms+1000*toc(tqp);
    R.duration_13d_qp_count=R.duration_13d_qp_count+1;
    R.joint_qp_status=S.status;
    R.duration_13d_qp_max_violation_history(end+1)= ...
        S.maximum_primal_violation;
    if ~S.solved
        R.status='UNKNOWN_13D_QP_NOT_SOLVED';
        R.failure_reason=S.status;break
    end
    dx=Q.chol_upper\z;
    R.duration_13d_max_unrestricted_step_norm_m=max( ...
        R.duration_13d_max_unrestricted_step_norm_m,norm(dx(1:12)));
    % Globalize the local tangent using actual nonlinear nominal body
    % positions. This is a SEARCH descent test, never a safety proof.
    % Disabled by default until independent multi-shape Flight A/B audit.
    use_merit=false;
    if isfield(req.search_options,'enable_13d_nonlinear_merit_line_search')
        v=req.search_options.enable_13d_nonlinear_merit_line_search;
        assert((islogical(v)||isnumeric(v))&&isscalar(v)&& ...
            isreal(v)&&isfinite(double(v))&&any(double(v)==[0 1]), ...
            'C9_13D:MeritSwitch','Nonlinear merit flag must be 0/1.');
        use_merit=logical(v);
    end
    R.duration_13d_linesearch_enabled=use_merit;
    if 1000*toc(start)>=budget_ms
        R.status='UNKNOWN_13D_EXECUTION_BUDGET_EXHAUSTED';
        R.failure_reason='No time left for nonlinear candidate validation.';
        R.duration_13d_execution_budget_exhausted=true;
        break
    end
    if use_merit
        local.search_options.internal_13d_start_clock=start;
        local.search_options.internal_13d_budget_ms=budget_ms;
        [alpha,D]=TRAJECTORY_C9_13D_NONLINEAR_PLANE_MERIT( ...
            Q,local,current,dx,Tmax);
        R.duration_13d_linesearch_ms= ...
            R.duration_13d_linesearch_ms+D.elapsed_ms;
        R.duration_13d_max_line_search_call_ms=max( ...
            R.duration_13d_max_line_search_call_ms,D.elapsed_ms);
        R.duration_13d_linesearch_backtracks= ...
            R.duration_13d_linesearch_backtracks+D.backtracks;
        R.duration_13d_interval_hull_enabled=D.interval_hull_enabled;
        R.duration_13d_interval_hull_evaluations= ...
            R.duration_13d_interval_hull_evaluations+D.interval_evaluations;
        R.duration_13d_interval_hull_rejected_trials= ...
            R.duration_13d_interval_hull_rejected_trials+D.interval_rejected_trials;
        R.duration_13d_interval_hull_before_after_m(end+1,:)= ...
            [D.interval_before_m,D.interval_after_m];
        R.duration_13d_linesearch_alpha_history(end+1)=alpha;
        R.duration_13d_point_merit_before_after_m(end+1,:)= ...
            [D.merit_before_m,D.merit_after_m];
        R.duration_13d_linesearch_status=D.status;
        if ~D.accepted
            if strcmp(D.status,'UNKNOWN_13D_EXECUTION_BUDGET_EXHAUSTED')
                R.status=D.status;
                R.duration_13d_execution_budget_exhausted=true;
            else
                R.status='UNKNOWN_13D_NONLINEAR_MODEL_GLOBALIZATION';
            end
            R.failure_reason=D.status;
            break
        end
        dx=alpha*dx;
    end
    Fnew=local.nominal_free+reshape(dx(1:12),3,4);
    R.duration_13d_free_step_norm_history_m(end+1)=norm(dx(1:12));
    Tnew=local.duration_s+dx(13);
    if ~(all(isfinite(Fnew(:)))&&isfinite(Tnew)&& ...
            Tnew>0&&Tnew<=Tmax)
        R.status='UNKNOWN_13D_NONPOSITIVE_OR_NONFLIGHT_DURATION';
        R.failure_reason='The QP tangent touched a time-domain boundary.';break
    end
    if isequal(Fnew,local.nominal_free)&&Tnew==local.duration_s
        R.status='UNKNOWN_13D_NO_LOCAL_QP_PROGRESS';
        R.failure_reason=oracle.status;break
    end
    x=[Fnew(:);Tnew];
    if any(all(seen==x,1))
        R.status='UNKNOWN_13D_QP_CYCLE';
        R.failure_reason='A duration/free candidate was already searched.';break
    end
    seen(:,end+1)=x; %#ok<AGROW>
    current=TRAJECTORY_C9_FLAT_BERNSTEIN.build( ...
        req.t0_s,Tnew,req.start_jets,req.end_jets,Fnew);
    R.duration_13d_candidate_history_s(end+1)=Tnew;
end
R.duration_13d_elapsed_ms=1000*toc(start);
R.duration_13d_execution_budget_overrun_ms= ...
    max(0,R.duration_13d_elapsed_ms-budget_ms);
if ~isfinite(budget_ms)
    R.duration_13d_execution_budget_overrun_ms=0;
end
R.independent_proof_time_total_ms=R.duration_13d_proof_ms;
R.plane_construction_time_total_ms=R.duration_13d_plane_ms;
R.joint_qp_time_total_ms=R.duration_13d_qp_ms;
R.safe_to_commit=false;
% Even when the single C9 nominal candidate passes, real HLC tracking,
% time-varying mass and numerical directed-rounding are NOT certified.
end

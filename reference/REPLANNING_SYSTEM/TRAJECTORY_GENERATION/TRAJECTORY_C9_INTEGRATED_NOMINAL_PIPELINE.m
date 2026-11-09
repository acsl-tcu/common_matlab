function R=TRAJECTORY_C9_INTEGRATED_NOMINAL_PIPELINE(request)
% TRAJECTORY_C9_INTEGRATED_NOMINAL_PIPELINE
% One SAME C9 simultaneous-obstacle search, with a single all-time joint
% geometry/tension/thrust oracle and mandatory C7/yaw-C6 and nominal
% 3-axis torque tests. This is NOT a multi-stage trajectory generator.
%
% ALL search candidates are certified exclusively by the existing
% TRAJECTORY_C9_JOINT_FAST_CERTIFICATE inside the one search invocation.
% Its final proof is shared here only because this function itself invoked
% the search and has not mutated its result or the external inputs.
% An arbitrary externally supplied 'certified' struct is NEVER accepted.
%
% CONDITIONAL NOMINAL certification is NOT execution certification:
% continuous-time floating-point enclosure, current EKF-to-HLC tracking,
% closed-loop attainable set, torque saturation, mass uncertainty validity,
% and controller/observer error tube are not proven here. NEVER COMMIT.
%
% All authority, observed shapes, numerical guards, proof/search budgets,
% desired time, and C7/yaw join tolerances are supplied by the caller.
% In production use the currently bound Flight/EKF/HLC authority; this
% function never guesses a future payload mass or a controller limit.
timer=tic;
R=struct('status','UNKNOWN_INTEGRATED_NOT_STARTED', ...
    'nominal_model_complete',false,'safe_to_commit',false, ...
    'actual_HLC_tracking_proven',false, ...
    'closed_loop_actuator_limits_proven',false, ...
    'numerically_directed_rounding_proven',false, ...
    'future_mass_envelope_proven',false, ...
    'candidate_available',false, 'candidate_piece',struct(), ...
    'live_EKF_entry_supplied',false, ...
    'live_EKF_entry_necessary_pass',false, ...
    'live_EKF_entry_status','NOT_SUPPLIED_NOMINAL_ONLY', ...
    'live_EKF_entry_evaluation_ms',0, ...
    'search_status','NOT_CALLED','heading_status','NOT_CALLED', ...
    'torque_status','NOT_CALLED', ...
    'search_result',struct(), 'heading_certificate',struct(), ...
    'nominal_torque_certificate',struct(), ...
    'joint_proof_reused_from_this_search',false, ...
    'nominal_geometry_tension_thrust_proven',false, ...
    'C7_yaw_C6_all_time_and_join_proven',false, ...
    'nominal_three_axis_torque_proven',false, ...
    'proof_calls',0, 'obstacle_count',0, ...
    'total_elapsed_ms',NaN,'search_elapsed_ms',0, ...
    'joint_geometry_force_proof_ms',0, 'separation_planes_ms',0, ...
    'joint_QP_ms',0,'search_other_ms',0, ...
    'geometry_snapshot_cache_hits',0, ...
    'geometry_snapshot_prepare_ms_total',0, ...
    'heading_join_ms',0,'nominal_torque_ms',0, ...
    'integration_overhead_ms',0, ...
    'measurement_scope', ...
    'C9_SEARCH_PLUS_INTERNAL_JOINT_PROOFS_PLUS_C7_YAW_PLUS_NOMINAL_TORQUE', ...
    'excluded_from_timing', ...
    'EKF_HLC_BINDING_TRIGGER_SENSOR_FULL_CONTROLLER_SIM_LOGGER', ...
    'failure_detail','');
try
    if ~(isstruct(request)&&isscalar(request)&& ...
            all(isfield(request,{'heading_options','nominal_torque_options', ...
                                 'authority','geometry_options','observed'})))
        R.status='UNKNOWN_REQUIRED_FULL_NOMINAL_OPTIONS_MISSING';
        finish();return
    end
    a=request.authority;
    if ~(isstruct(a)&&isscalar(a)&& ...
            all(isfield(a,{'uav_inertia_kgm2','torque_limits_Nm', ...
                           'payload_mass_kg','thrust_limits_N'})))
        R.status='UNKNOWN_CURRENT_HLC_TORQUE_AUTHORITY_MISSING';
        finish();return
    end
    if ~(isstruct(request.observed))
        R.status='UNKNOWN_OBSERVED_SHAPES_MALFORMED';
        finish();return
    end
    R.obstacle_count=numel(request.observed);
    % An optional real-EKF entry snapshot is a NECESSARY execution gate.
    % The nominal-only fixture API remains usable for offline research,
    % but never returns safe_to_commit. If either live binding field is
    % provided, BOTH are mandatory and the current mass/position/velocity/
    % cable/UAV entry must match before any expensive search begins.
    has_live_state=isfield(request,'current_ekf_state');
    has_live_tol=isfield(request,'entry_tolerances');
    if xor(has_live_state,has_live_tol)
        R.status='UNKNOWN_INCOMPLETE_LIVE_EKF_ENTRY_BINDING';
        finish();return
    end
    if has_live_state
        R.live_EKF_entry_supplied=true;
        te=tic;
        E=TRAJECTORY_C9_ENTRY_STATE_GATE( ...
            request,request.current_ekf_state,request.entry_tolerances);
        R.live_EKF_entry_evaluation_ms=1000*toc(te);
        R.live_EKF_entry_status=E.status;
        R.live_EKF_entry_necessary_pass=E.necessary_conditions_met;
        if ~E.necessary_conditions_met
            R.status='UNKNOWN_CURRENT_EKF_TO_C9_ENTRY_INCONSISTENT';
            R.failure_detail=E.status;
            finish();return
        end
    end
    % Avoid downstream repeated work if a required nominal proof option
    % is missing. Neither section is optional in this integrated entry.
    if ~(isstruct(request.heading_options)&& ...
            isstruct(request.nominal_torque_options))
        R.status='UNKNOWN_PROOF_OPTION_FORMAT';finish();return
    end
    % Duration-aware joint mode requires the SAME physical torque options
    % in both the single 13D QP and the later independent all-time proof.
    duration13=false;
    if isfield(request,'search_options') && ...
            isstruct(request.search_options) && ...
            isfield(request.search_options,'enable_13d_variable_time_joint_search')
        duration13=isequal(request.search_options.enable_13d_variable_time_joint_search,true) || ...
            isequal(request.search_options.enable_13d_variable_time_joint_search,1);
    end
    if duration13
        if isfield(request,'torque_options') && ...
                ~isequal(request.torque_options,request.nominal_torque_options)
            R.status='UNKNOWN_CONFLICTING_13D_TORQUE_OPTIONS';
            finish();return
        end
        request.torque_options=request.nominal_torque_options;
    end
    S=TRAJECTORY_C9_SIMULTANEOUS_GUIDE_SEARCH(request);
    R.search_result=S;
    R.search_status=S.status;
    R.search_elapsed_ms=S.elapsed_ms;
    R.proof_calls=S.independent_proof_calls;
    R.joint_geometry_force_proof_ms=S.independent_proof_time_total_ms;
    R.separation_planes_ms=S.plane_construction_time_total_ms;
    R.joint_QP_ms=S.joint_qp_time_total_ms;
    R.geometry_snapshot_cache_hits=S.geometry_snapshot_cache_hits;
    R.geometry_snapshot_prepare_ms_total=S.geometry_snapshot_prepare_ms_total;
    R.search_other_ms=max(0,R.search_elapsed_ms- ...
        (R.joint_geometry_force_proof_ms+R.separation_planes_ms+R.joint_QP_ms));
    if ~S.candidate_available
        R.status='UNKNOWN_C9_SEARCH_DID_NOT_CERTIFY_ALL_TIME';
        R.failure_detail=S.status;
        finish();return
    end
    % Never rerun the expensive joint proof for the SAME immutable
    % candidate already certified inside this search invocation.
    P=S.final_certificate;
    if ~(isstruct(P)&&isfield(P,'certified')&& ...
            isequal(P.certified,true)&& ...
            isequal(P.certified_continuous_geometry,true)&& ...
            isequal(P.certified_continuous_tension,true)&& ...
            isequal(P.certified_continuous_thrust,true)&& ...
            isequal(P.authority_snapshot,a)&& ...
            ~P.safe_to_commit&& ...
            isequal(P.trajectory_phase,'f')&& ...
            isequal(P.ground_checked, ...
                TRAJECTORY_GROUND_SCOPE(request.geometry_options))&& ...
            ~S.safe_to_commit)
        R.status='UNKNOWN_SEARCH_PROOF_BINDING_OR_SCOPE';
        finish();return
    end
    R.joint_proof_reused_from_this_search=true;
    R.nominal_geometry_tension_thrust_proven=true;
    piece=S.candidate_piece;
    % A changed T is legal ONLY in the explicitly enabled single 13D
    % optimization.  The start is still EXACT current EKF time, and the
    % final endpoint must remain inside the authoritative Flight interval.
    % It is not tied to the 4-second trigger horizon or original clock.
    time_identity=isstruct(piece)&&isscalar(piece)&& ...
        isfield(piece,'duration_s')&& ...
        isequal(piece.duration_s,request.duration_s);
    time_in_flight=duration13 && isstruct(piece)&&isscalar(piece)&& ...
        all(isfield(piece,{'t0_s','duration_s'}))&& ...
        isfield(request,'flight_interval_s')&& ...
        numel(request.flight_interval_s)==2 && ...
        isnumeric(piece.duration_s)&&isreal(piece.duration_s)&& ...
        isfinite(piece.duration_s)&&piece.duration_s>0 && ...
        piece.t0_s+piece.duration_s<=request.flight_interval_s(2);
    if ~(isstruct(piece)&&isscalar(piece)&& ...
            isequal(piece.t0_s,request.t0_s)&& ...
            (time_identity || time_in_flight)&& ...
            isfield(piece,'control_points_m')&& ...
            isequal(size(piece.control_points_m),[3,24]))
        R.status='UNKNOWN_INTERNAL_CANDIDATE_IDENTITY';
        finish();return
    end
    tHeading=tic;
    H=TRAJECTORY_C9_UAV_C7_YAW_C6_AUDIT( ...
        piece,a,request.heading_options);
    R.heading_join_ms=1000*toc(tHeading);
    R.heading_certificate=H;
    R.heading_status=H.status;
    R.C7_yaw_C6_all_time_and_join_proven= ...
        H.certified&&H.all_time_yaw_C6_defined_proven&& ...
        H.derived_uav_C7_regular&&H.derived_yaw_C6_regular;
    if ~R.C7_yaw_C6_all_time_and_join_proven
        R.status='UNKNOWN_C7_YAW_C6_OR_JOIN_NOT_CERTIFIED';
        R.failure_detail=H.status;
        finish();return
    end
    tTorque=tic;
    torque_options=request.nominal_torque_options;
    if H.certified_stationary_yaw_hold
        % Bind the constant-yaw torque identity to the SAME held angle
        % certified by the C6 heading audit. Do not trust a mismatched
        % independent torque-option anchor.
        held_yaw=double(request.heading_options.stationary_yaw_anchor_rad);
        if isfield(torque_options,'stationary_yaw_anchor_rad') && ...
                ~isequal(double(torque_options.stationary_yaw_anchor_rad),held_yaw)
            R.status='UNKNOWN_STATIONARY_YAW_ANCHOR_CONFLICT';
            R.failure_detail=R.status;
            finish();return
        end
        torque_options.stationary_yaw_anchor_rad=held_yaw;
    end
    Q=TRAJECTORY_C9_NOMINAL_TORQUE_INTERVAL( ...
        piece,a,torque_options);
    R.nominal_torque_ms=1000*toc(tTorque);
    R.nominal_torque_certificate=Q;
    R.torque_status=Q.status;
    R.nominal_three_axis_torque_proven= ...
        Q.conditional_nominal_all_time_torque_proven&& ...
        ~Q.actual_HLC_tracking_proven&&~Q.safe_to_commit;
    if ~R.nominal_three_axis_torque_proven
        R.status='UNKNOWN_NOMINAL_3_AXIS_TORQUE_NOT_CERTIFIED';
        R.failure_detail=Q.status;
        finish();return
    end
    R.nominal_model_complete=true;
    R.candidate_available=true;
    R.candidate_piece=piece;
    R.status='CONDITIONAL_INTEGRATED_C9_C7_YAW_TORQUE_NOMINAL__HLC_PENDING';
catch ME
    R.status='UNKNOWN_INTEGRATED_EXCEPTION';
    R.failure_detail=[ME.identifier ' | ' ME.message];
end
finish();

    function finish()
        R.total_elapsed_ms=1000*toc(timer);
        R.integration_overhead_ms=max(0,R.total_elapsed_ms- ...
            R.search_elapsed_ms-R.heading_join_ms-R.nominal_torque_ms);
        % This API always returns candidates only. No physical controller
        % commit / HLC execution certificate exists here.
        R.safe_to_commit=false;
    end
end

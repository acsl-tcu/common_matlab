function R=TRAJECTORY_C9_27_NOMINAL_CHAIN(piece,observed,radii,authority, ...
    geometry_options,force_options,heading_options,torque_options)
% TRAJECTORY_C9_27_NOMINAL_CHAIN
% Independent ALL-TIME ideal taut model chain for ONE C9 degree-27 piece.
% Actual 3x28 polynomial is used by geometry/tension/thrust, heading and
% torque proofs. No sampled geometry, no fixed avoidance direction.
% Does not assert feasibility of search, the HLC/plant tracking tube,
% directed rounding, or real-time execution. Never sets safe_to_commit.
clock=tic;
R=struct('status','UNKNOWN_C9_27_NOT_CHECKED','safe_to_commit',false, ...
    'nominal_chain_proven',false,'geometry_force',struct(), ...
    'heading',struct(),'torque',struct(), ...
    'geometry_ms',NaN,'heading_ms',NaN,'torque_ms',NaN,'elapsed_ms',NaN, ...
    'actual_HLC_tracking_proven',false, ...
    'scope','DEGREE27_IDEAL_TAUT_NOMINAL_ONLY__HLC_REACHABILITY_PENDING');
try
    if ~(isstruct(piece)&&isscalar(piece)&& ...
            isfield(piece,'control_points_m')&& ...
            isequal(size(piece.control_points_m),[3 28]))
        R.status='UNKNOWN_C9_27_PIECE_INVALID';
        R.elapsed_ms=1000*toc(clock);return
    end
    % Recompute and bind ALL derivatives before any expensive all-time step.
    actual=TRAJECTORY_C9_FLAT_BERNSTEIN.actual_derivative_controls(piece);
    if ~(isfield(piece,'derivative_controls')&& ...
            iscell(piece.derivative_controls)&& ...
            numel(piece.derivative_controls)==10)
        R.status='UNKNOWN_C9_27_DERIVATIVE_CACHE_MISSING';
        R.elapsed_ms=1000*toc(clock);return
    end
    for k=1:10
        if ~isequal(piece.derivative_controls{k},actual{k})
            R.status='UNKNOWN_C9_27_DERIVATIVE_CACHE_UNBOUND';
            R.elapsed_ms=1000*toc(clock);return
        end
    end
    t=tic;
    R.geometry_force=TRAJECTORY_C9_JOINT_FAST_CERTIFICATE( ...
        piece,observed,radii,authority,geometry_options,force_options);
    R.geometry_ms=1000*toc(t);
    if ~R.geometry_force.certified
        R.status=['UNKNOWN_C9_27_GEOMETRY_' R.geometry_force.status];
        R.elapsed_ms=1000*toc(clock);return
    end
    t=tic;
    R.heading=TRAJECTORY_C9_UAV_C7_YAW_C6_AUDIT( ...
        piece,authority,heading_options);
    R.heading_ms=1000*toc(t);
    if ~R.heading.certified
        R.status=['UNKNOWN_C9_27_HEADING_' R.heading.status];
        R.elapsed_ms=1000*toc(clock);return
    end
    t=tic;
    R.torque=TRAJECTORY_C9_NOMINAL_TORQUE_INTERVAL( ...
        piece,authority,torque_options);
    R.torque_ms=1000*toc(t);
    if ~R.torque.conditional_nominal_all_time_torque_proven
        R.status=['UNKNOWN_C9_27_NOMINAL_TORQUE_' R.torque.status];
        R.elapsed_ms=1000*toc(clock);return
    end
    % Physical provenance must be shared by all three checks. This is a
    % NOMINAL sufficient chain only, not an actual flight controller proof.
    R.nominal_chain_proven=true;
    R.status='CONDITIONAL_C9_27_NOMINAL_GEOMETRY_TENSION_THRUST_C7_YAW_TORQUE__HLC_PENDING';
catch ME
    R.status=['UNKNOWN_C9_27_CHAIN_EXCEPTION_' ME.identifier];
end
R.elapsed_ms=1000*toc(clock);
end

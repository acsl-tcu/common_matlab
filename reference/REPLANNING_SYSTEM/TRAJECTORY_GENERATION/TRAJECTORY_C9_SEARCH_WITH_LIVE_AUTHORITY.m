function [R,snapshot]=TRAJECTORY_C9_SEARCH_WITH_LIVE_AUTHORITY(req,binding,time,agent)
% Event-time entrypoint for the SAME simultaneous C9 search.
% Capture the current EKF-estimated load mass and real HLC limits BEFORE
% a trajectory proposal. Never accept caller-cached mass/bounds, and never
% silently retain an earlier certified piece after mass changes.
% This function is NOT installed in the Flight runner and NEVER commits to
% HLC. New: require a real-EKF zero/first-jet start gate BEFORE costly C9
% optimization. This is necessary, NOT an HLC tracking or C7 7-jet proof.
snapshot=TRAJECTORY_C9_HLC_FORCE_AUTHORITY.current(binding,agent,time);
R=struct('status','UNKNOWN_LIVE_FORCE_AUTHORITY', ...
    'candidate_available',false,'safe_to_commit',false, ...
    'source_snapshot',snapshot,'candidate_piece',struct(), ...
    'certified_force_headroom_N',NaN,'search_result',struct(), ...
    'authority_unchanged_at_search_end',false, ...
    'post_search_source_snapshot',struct(), ...
    'ekf_start_join_gate',struct(), ...
    'start_join_necessary_pass',false, ...
    'flight_nonmass_contract_checked',false, ...
    'flight_nonmass_contract_status','NOT_PROVIDED', ...
    'future_mass_force_certificate',struct(), ...
    'future_mass_force_conditionally_proven',false, ...
    'nominal_torque_certificate',struct(), ...
    'nominal_torque_all_time_conditionally_proven',false);
if ~snapshot.valid
    R.status=snapshot.status;
    return
end
assert(isstruct(req)&&isscalar(req)&&isfield(req,'t0_s')&& ...
    isfinite(req.t0_s)&&isequal(double(req.t0_s),double(time.t)), ...
    'TRAJECTORY_C9_LIVE_SEARCH:EntryTime', ...
    'The trajectory must start at the current EKF/Flight event time.');
% The old req.authority is intentionally ignored. Production physical
% authority is reconstructed from the active HLC/estimator EVERY call.
req.authority=snapshot.authority;
% A Flight-wide contract is explicit rather than silently invented.
% Without it the candidate remains only a nominal research candidate,
% never an execution/tracking certificate.
if isfield(req,'flight_parameter_contract')
    guard=TRAJECTORY_C9_FLIGHT_PARAMETER_CONTRACT.check( ...
        req.flight_parameter_contract,binding,agent,time,req);
    R.flight_nonmass_contract_status=guard.status;
    if ~guard.valid
        R.status=guard.status;
        return
    end
    R.flight_nonmass_contract_checked=true;
end
% The current EKF is not a 9-jet estimator. Check the observable necessary
% 0/1 derivative and cable state before expensive simultaneous optimization.
% No guessed tolerance or caller-cached measured state is acceptable.
if ~isfield(req,'ekf_start_join_tolerances')
    R.status='UNKNOWN_REQUIRED_LIVE_EKF_JOIN_TOLERANCES_MISSING';
    return
end
try
    % Free controls cannot change the C9 start jet. This seed piece is
    % strictly for checking that endpoint; it is NOT a second planner.
    seed=TRAJECTORY_C9_FLAT_BERNSTEIN.build(req.t0_s, ...
        req.duration_s,req.start_jets,req.end_jets,req.nominal_free);
    join=TRAJECTORY_C9_LIVE_EKF_JOIN_GATE(seed,binding,agent,time, ...
        req.ekf_start_join_tolerances);
catch ME
    R.status=['UNKNOWN_LIVE_EKF_START_GATE_ERROR_' ME.identifier];
    return
end
R.ekf_start_join_gate=join;
R.start_join_necessary_pass=join.necessary_start_pass;
if ~join.necessary_start_pass
    R.status=join.status;
    return
end
result=TRAJECTORY_C9_SIMULTANEOUS_GUIDE_SEARCH(req);
% Re-read the live EKF/HLC authority after the (potentially expensive)
% search. An old mass/inertia/controller binding cannot remain certified.
latest=TRAJECTORY_C9_HLC_FORCE_AUTHORITY.current(binding,agent,time);
R.post_search_source_snapshot=latest;
R.authority_unchanged_at_search_end= ...
    TRAJECTORY_C9_HLC_FORCE_AUTHORITY.same_force_authority(snapshot,latest);
R.search_result=result;
if R.flight_nonmass_contract_checked
    second=TRAJECTORY_C9_FLIGHT_PARAMETER_CONTRACT.check( ...
        req.flight_parameter_contract,binding,agent,time,req);
    if ~second.valid
        R.status=second.status;
        R.candidate_available=false;
        R.candidate_piece=struct();
        return
    end
end
if ~R.authority_unchanged_at_search_end
    R.status='UNKNOWN_AUTHORITY_CHANGED_DURING_SEARCH';
    R.safe_to_commit=false;
    R.candidate_available=false;
    R.candidate_piece=struct();
    return
end
R.status=result.status;
R.candidate_available=result.candidate_available;
R.candidate_piece=result.candidate_piece;
R.safe_to_commit=false;
if result.candidate_available
    H=TRAJECTORY_C9_HLC_FORCE_AUTHORITY.certified_headroom( ...
        result.final_certificate,snapshot.authority);
    if ~H.certified
        R.status='UNKNOWN_FORCE_HEADROOM_NOT_CERTIFIED';
        R.candidate_available=false;
        R.candidate_piece=struct();
    else
        R.certified_force_headroom_N=H.minimum_headroom_N;
        % Prove future payload-mass variation only if an EXTERNAL bound
        % covers this ENTIRE piece. An estimator sample is NOT such a bound.
        if R.flight_nonmass_contract_checked && ...
                req.flight_parameter_contract.has_future_mass_envelope
            if ~isfield(req,'mass_acceleration_roundoff_guard_mps2')
                R.status='UNKNOWN_FUTURE_MASS_NUMERIC_GUARD_MISSING';
                R.candidate_available=false;
                R.candidate_piece=struct();
                return
            end
            M=TRAJECTORY_C9_VARIABLE_MASS_FORCE_CERTIFICATE( ...
                result.candidate_piece,result.final_certificate, ...
                snapshot.authority,req.flight_parameter_contract.mass_envelope, ...
                req.mass_acceleration_roundoff_guard_mps2);
            R.future_mass_force_certificate=M;
            R.future_mass_force_conditionally_proven= ...
                M.conditional_ideal_taut_force_proven;
            if ~M.conditional_ideal_taut_force_proven
                R.status=M.status;
                R.candidate_available=false;
                R.candidate_piece=struct();
                return
            end
        end
    end
end
% An OPTIONAL strictly stronger necessary nominal 3-axis torque gate.
% This is part of the SAME coupled C9 result verification, NOT a new
% search stage or a replacement for HLC closed-loop tracking proof.
% Passing it must never set safe_to_commit=true.
if R.candidate_available && isfield(req,'nominal_torque_options')
    T=TRAJECTORY_C9_NOMINAL_TORQUE_INTERVAL( ...
        R.candidate_piece,snapshot.authority,req.nominal_torque_options);
    R.nominal_torque_certificate=T;
    R.nominal_torque_all_time_conditionally_proven= ...
        T.conditional_nominal_all_time_torque_proven;
    if ~T.conditional_nominal_all_time_torque_proven
        R.status=T.status;
        R.candidate_available=false;
        R.candidate_piece=struct();
        return
    end
    % Mass/physical authority may have changed during this EXTRA proof.
    latest_after_torque=TRAJECTORY_C9_HLC_FORCE_AUTHORITY.current( ...
        binding,agent,time);
    if ~TRAJECTORY_C9_HLC_FORCE_AUTHORITY.same_force_authority( ...
            snapshot,latest_after_torque)
        R.status='UNKNOWN_AUTHORITY_CHANGED_DURING_TORQUE_CERTIFICATION';
        R.candidate_available=false;
        R.candidate_piece=struct();
        R.nominal_torque_all_time_conditionally_proven=false;
        return
    end
end
end

function R=TRAJECTORY_C9_ENTRY_STATE_GATE(request, ekf_state, tolerances)
% TRAJECTORY_C9_ENTRY_STATE_GATE: CURRENT EKF -> C9 entry NECESSARY checks.
%
% The very first control-point jets must describe the *current* payload
% position/velocity, cable attitude and UAV position. Reject stale nominal
% endpoint data before spending time on a multi-obstacle QP. All physical
% constants and tolerances are obtained from the current Flight request;
% no fallbacks, synthetic uncertainty bounds or frozen load mass.
%
% This is deliberately NOT a sufficient C7/yaw-C6 join, HLC tracking
% certificate, actuator-feasible tube, or proof of being collision-free.
% High derivatives 2..9 cannot be inferred uniquely from EKF pL/vL/pT.
% Returned safe_to_commit is ALWAYS false.
R=struct('status','UNKNOWN_ENTRY_INPUT', ...
    'necessary_conditions_met',false,'safe_to_commit',false, ...
    'C7_high_order_join_proven',false, ...
    'HLC_tracking_and_preclamp_proven',false, ...
    'payload_position_error_m',NaN, ...
    'payload_velocity_error_mps',NaN, ...
    'cable_direction_error_rad',NaN, ...
    'uav_position_error_m',NaN, ...
    'mass_snapshot_matches_EKF',false, ...
    'all_inputs_valid',false, ...
    'scope','ONLY_EKF_TO_C9_ENTRY_NECESSARY__HLC_AND_HIGH_JETS_UNKNOWN');
try
    if ~(isstruct(request)&&isscalar(request)&& ...
            all(isfield(request,{'start_jets','authority','t0_s'}))&& ...
            isstruct(ekf_state)&&isscalar(ekf_state)&& ...
            all(isfield(ekf_state,{'pL','vL','pT','p','mL'}))&& ...
            isstruct(tolerances)&&isscalar(tolerances)&& ...
            all(isfield(tolerances,{'payload_position_m', ...
            'payload_velocity_mps','cable_direction_rad','uav_position_m'})))
        R.status='UNKNOWN_ENTRY_REQUIRED_LIVE_FIELDS_MISSING';return
    end
    A=request.authority;
    if ~(isstruct(A)&&isscalar(A)&& ...
            all(isfield(A,{'gravity_world_mps2', ...
            'cable_length_m','payload_mass_kg'})))
        R.status='UNKNOWN_ENTRY_MISSING_FLIGHT_AUTHORITY';return
    end
    P=request.start_jets;
    if ~(isnumeric(P)&&isreal(P)&&isequal(size(P),[3 10])&& ...
            all(isfinite(P(:)))&& ...
            isnumeric(request.t0_s)&&isscalar(request.t0_s)&& ...
            isfinite(request.t0_s))
        R.status='UNKNOWN_ENTRY_INVALID_C9_JETS';return
    end
    fields={'pL','vL','pT','p'};
    for i=1:numel(fields)
        v=ekf_state.(fields{i});
        if ~(isnumeric(v)&&isreal(v)&&numel(v)==3&& ...
                all(isfinite(v(:))))
            R.status='UNKNOWN_ENTRY_INVALID_EKF_GEOMETRY';return
        end
    end
    if ~(isnumeric(ekf_state.mL)&&isreal(ekf_state.mL)&& ...
            isscalar(ekf_state.mL)&&isfinite(ekf_state.mL)&& ...
            ekf_state.mL>0)
        R.status='UNKNOWN_ENTRY_INVALID_EKF_MASS';return
    end
    if ~(isnumeric(A.payload_mass_kg)&&isreal(A.payload_mass_kg)&& ...
            isscalar(A.payload_mass_kg)&&isfinite(A.payload_mass_kg)&& ...
            A.payload_mass_kg>0 && ...
            isnumeric(A.cable_length_m)&&isreal(A.cable_length_m)&& ...
            isscalar(A.cable_length_m)&&isfinite(A.cable_length_m)&& ...
            A.cable_length_m>0 && ...
            isnumeric(A.gravity_world_mps2)&&isreal(A.gravity_world_mps2)&& ...
            numel(A.gravity_world_mps2)==3&& ...
            all(isfinite(A.gravity_world_mps2(:))))
        R.status='UNKNOWN_ENTRY_INVALID_FLIGHT_PHYSICS';return
    end
    R.mass_snapshot_matches_EKF=isequal(double(ekf_state.mL), ...
        double(A.payload_mass_kg));
    if ~R.mass_snapshot_matches_EKF
        R.status='UNKNOWN_ENTRY_STALE_EKF_PAYLOAD_MASS';return
    end
    names={'payload_position_m','payload_velocity_mps', ...
        'cable_direction_rad','uav_position_m'};
    for i=1:numel(names)
        v=tolerances.(names{i});
        if ~(isnumeric(v)&&isreal(v)&&isscalar(v)&& ...
                isfinite(v)&&v>=0)
            R.status='UNKNOWN_ENTRY_TOLERANCE_NOT_EXTERNALLY_BOUND';return
        end
    end
    w=double(A.gravity_world_mps2(:))-double(P(:,3));
    if ~all(isfinite(w))||norm(w)==0
        R.status='UNKNOWN_ENTRY_TAUT_CABLE_DIRECTION_SINGULAR';return
    end
    q=w/norm(w);
    pT=double(ekf_state.pT(:));
    if norm(pT)==0
        R.status='UNKNOWN_ENTRY_EKF_CABLE_DIRECTION_SINGULAR';return
    end
    pT=pT/norm(pT);
    pL=double(ekf_state.pL(:));
    R.payload_position_error_m=norm(double(P(:,1))-pL);
    R.payload_velocity_error_mps=norm(double(P(:,2))-double(ekf_state.vL(:)));
    R.cable_direction_error_rad=atan2(norm(cross(q,pT)),dot(q,pT));
    R.uav_position_error_m=norm(double(P(:,1))- ...
        double(A.cable_length_m)*q-double(ekf_state.p(:)));
    R.all_inputs_valid=true;
    if R.payload_position_error_m>tolerances.payload_position_m
        R.status='UNKNOWN_ENTRY_PAYLOAD_POSITION_MISMATCH';return
    end
    if R.payload_velocity_error_mps>tolerances.payload_velocity_mps
        R.status='UNKNOWN_ENTRY_PAYLOAD_VELOCITY_MISMATCH';return
    end
    if R.cable_direction_error_rad>tolerances.cable_direction_rad
        R.status='UNKNOWN_ENTRY_CABLE_DIRECTION_MISMATCH';return
    end
    if R.uav_position_error_m>tolerances.uav_position_m
        R.status='UNKNOWN_ENTRY_UAV_POSITION_MISMATCH';return
    end
    R.necessary_conditions_met=true;
    R.status='PASS_EKF_TO_C9_ENTRY_NECESSARY__C7_AND_HLC_PENDING';
catch ME
    R.status=['UNKNOWN_ENTRY_EVALUATION_' ME.identifier];
end
end

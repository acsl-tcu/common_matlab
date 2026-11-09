function D=TRAJECTORY_C9_TORQUE_CERTIFICATE_FEEDBACK(piece,authority,torque_options,existing_times_s)
% TORQUE_CERTIFICATE_FEEDBACK  Proof-guided points for the SAME C9-QP.
%
% Run the existing independent ALL-TIME nominal rigid-body torque proof.
% If it is unresolved, use ONLY that proof's last unresolved time leaf
% to locate a definite numerical POINT violation of a physical torque
% limit. The witness is a search constraint, never a safety certificate.
%
% All numbers used here are inherited from the proof leaf and the actual
% actuator limits. There are NO obstacle-specific cases, tuning weights,
% direction presets, velocity floors, or invented physical thresholds.
% Endpoints and midpoint of the unresolved leaf are diagnostic locations;
% their absence of violation NEVER proves feasibility. Failure is UNKNOWN.
%
% Even a successful nominal continuous-time proof excludes floating-point
% directed rounding, a time-varying mL(t), and actual closed-loop HLC.
clock=tic;
D=struct('status','UNKNOWN_NOT_TESTED', ...
    'nominal_all_time_conditional',false,'can_reoptimize',false, ...
    'proof_status','NOT_RUN','proof_elapsed_ms',NaN, ...
    'point_elapsed_ms',0, ...
    'proof_interval_s',[NaN NaN], ...
    'probe_times_s',zeros(1,0), ...
    'witness_times_s',zeros(1,0), ...
    'new_witness_time_count',0, ...
    'updated_times_s',zeros(1,0), ...
    'maximum_point_violation_Nm',NaN, ...
    'safe_to_commit',false, ...
    'scope','NOMINAL_REAL_ARITHMETIC_PROOF_FEEDBACK_ONLY__NOT_HLC', ...
    'elapsed_ms',NaN);
try
    existing=double(existing_times_s(:).');
    assert(isnumeric(existing_times_s)&&isreal(existing_times_s)&& ...
        all(isfinite(existing))&&all(existing>=piece.t0_s)&& ...
        all(existing<=piece.t0_s+piece.duration_s), ...
        'C9_TORQUE_FEEDBACK:Times','Finite physical probe times required');
    D.updated_times_s=unique(existing);
    C=TRAJECTORY_C9_NOMINAL_TORQUE_INTERVAL(piece,authority,torque_options);
    D.proof_status=C.status;
    D.proof_elapsed_ms=C.elapsed_ms;
    if C.conditional_nominal_all_time_torque_proven
        D.nominal_all_time_conditional=true;
        D.status='CONDITIONAL_NOMINAL_TORQUE_ALL_TIME__HLC_UNCERTIFIED';
        D.elapsed_ms=1000*toc(clock);
        return
    end
    leaf=double(C.unresolved_interval_s(:).');
    D.proof_interval_s=leaf;
    if numel(leaf)~=2||~all(isfinite(leaf))||leaf(1)>leaf(2)|| ...
            leaf(1)<piece.t0_s||leaf(2)>piece.t0_s+piece.duration_s
        D.status='UNKNOWN_TORQUE_PROOF_NO_VALID_TIME_WITNESS';
        D.elapsed_ms=1000*toc(clock);
        return
    end
    % The 3 mathematically determined points of the actual failed leaf.
    % This does not impose a collocation grid for safety certification.
    D.probe_times_s=unique([leaf(1),leaf(1)+(leaf(2)-leaf(1))/2,leaf(2)]);
    P=TRAJECTORY_C9_TORQUE_POINT_EVALUATE( ...
        piece,authority,torque_options,D.probe_times_s);
    D.point_elapsed_ms=P.elapsed_ms;
    D.maximum_point_violation_Nm=P.maximum_point_violation_Nm;
    if ~all(P.valid)
        D.status='UNKNOWN_TORQUE_WITNESS_POINT_DEGENERACY';
    elseif any(P.outside_limits)
        D.witness_times_s=P.time_s(P.outside_limits);
        D.updated_times_s=unique([existing,D.witness_times_s]);
        D.new_witness_time_count= ...
            numel(setdiff(D.updated_times_s,unique(existing)));
        if D.new_witness_time_count>0
            D.can_reoptimize=true;
            D.status='TORQUE_POINT_VIOLATION_WITNESSED__SAME_C9_QP_REOPTIMIZE';
        else
            D.status='UNKNOWN_TORQUE_WITNESS_NO_NEW_TIME';
        end
    else
        D.status='UNKNOWN_TORQUE_LEAF_BUT_NO_POINT_INPUT_VIOLATION';
    end
catch ME
    D.status=['UNKNOWN_TORQUE_PROOF_FEEDBACK_' ME.identifier];
end
D.elapsed_ms=1000*toc(clock);
end

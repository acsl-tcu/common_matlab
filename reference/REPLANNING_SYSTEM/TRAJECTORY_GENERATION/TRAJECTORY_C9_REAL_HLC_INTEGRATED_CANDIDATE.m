function R=TRAJECTORY_C9_REAL_HLC_INTEGRATED_CANDIDATE(request,binding,time,agent)
%TRAJECTORY_C9_REAL_HLC_INTEGRATED_CANDIDATE Real Flight authority -> SAME C9.
% Read-only event-time entry for the already existing simultaneous C9
% search + all-time nominal geometry/tension/thrust + C7/yaw + torque.
% The ORIGINAL HLC source four-channel clamps are read DIRECTLY by
% TRAJECTORY_C9_HLC_FORCE_AUTHORITY.bind once EACH Flight (outside the
% real-time event), and the current HLC/estimator values are captured
% once at the current EKF event. Caller-provided
% authority and EKF snapshots are NOT trusted. No new generator or HLC edit.
%
% No source-file rescans, duplicate Flight contract snapshots, repeated EKF
% entry gates, or debug negative fixtures run inside a replan event. The
% immutable per-Flight binding is matched with the checked contract. If
% HLC source/class changes, the next Flight MUST re-bind before execution.
%
% IMPORTANT: even if nominal_model_complete is true, this is NOT a
% closed-loop reachability or pre-clamp future controller,
% numerical-directed-rounding, or physical mass-uncertainty certificate.
% It NEVER authorizes COMMIT.
clock=tic;
R=struct('status','UNKNOWN_LIVE_C9_NOT_STARTED','stage','INIT', ...
    'safe_to_commit',false,'candidate_available',false, ...
    'nominal_model_complete',false, ...
    'flight_parameter_contract_checked',false, ...
    'entry_necessary_pass',false,'entry_status','NOT_CHECKED', ...
    'source_hlc_file','','source_limits_revalidated',false, ...
    'source_limits_bound_this_flight',false, ...
    'source_read_performed_this_event',false, ...
    'source_read_elapsed_ms',0, ...
    'current_authority_capture_count',0, ...
    'entry_gate_count',0, ...
    'flight_contract_check_count',0, ...
    'input_lower_4',NaN(4,1), ...
    'input_upper_4',NaN(4,1),'presearch_snapshot',struct(), ...
    'postsearch_snapshot',struct(), ...
    'authority_unchanged_after_search',false, ...
    'postsearch_authority_check_required_before_commit',true, ...
    'live_EKF_mass_used_kg',NaN, ...
    'nominal_wrench_lower_4',NaN(4,1), ...
    'nominal_wrench_upper_4',NaN(4,1), ...
    'nominal_wrench_headroom_4x2',NaN(4,2), ...
    'admissible_HLC_preclamp_correction_lower_4',NaN(4,1), ...
    'admissible_HLC_preclamp_correction_upper_4',NaN(4,1), ...
    'HLC_feedback_correction_enclosed',false, ...
    'HLC_feedback_bound_status','NOT_DERIVED_FROM_CONTROLLER_EQUATIONS', ...
    'nominal_4_input_within_live_software_limits',false, ...
    'HLC_closed_loop_tracking_proven',false, ...
    'future_HLC_preclamp_inputs_proven',false, ...
    'integrated_result',struct(),'search_elapsed_ms',NaN, ...
    'total_elapsed_ms',NaN,'error_identifier','','error_message','', ...
    'scope','CURRENT_REAL_HLC_SOFTWARE_LIMITS_AND_LIVE_EKF__NOMINAL_ONLY');
try
    perform();
catch ME
    R.status='UNKNOWN_REAL_HLC_C9_PIPELINE_EXCEPTION';
    R.error_identifier=ME.identifier;
    R.error_message=ME.message;
end
R.total_elapsed_ms=1000*toc(clock);
R.safe_to_commit=false;

    function perform()
    R.stage='VALIDATE_REQUEST';
    if ~(isstruct(request)&&isscalar(request)&& ...
            all(isfield(request,{'t0_s','flight_parameter_contract', ...
                'entry_tolerances','radii','geometry_options', ...
                'force_options','start_jets'})) && ...
            isscalar(request.t0_s)&&isnumeric(request.t0_s)&& ...
            isfinite(request.t0_s))
        R.status='UNKNOWN_MISSING_LIVE_FLIGHT_CONTRACT_OR_START';return
    end
    if ~(isstruct(binding)&&isscalar(binding)&& ...
            all(isfield(binding,{'source_file','input_lower','input_upper'})))
        R.status='UNKNOWN_REAL_HLC_NOT_BOUND_THIS_FLIGHT';return
    end
    R.source_hlc_file=binding.source_file;
    % Source clamps were parsed DIRECTLY at the beginning of THIS Flight
    % by HLC_FORCE_AUTHORITY.bind. A MATLAB class cannot change its loaded
    % controller implementation during normal Flight without invalidating
    % the Flight session; a fresh Flight requires a fresh source binding.
    % Validate the same source-derived binding, do not reread the source
    % file for each search candidate (expensive and not a controller proof).
    R.stage='VERIFY_CURRENT_FLIGHT_SOURCE_BINDING';
    contract=request.flight_parameter_contract;
    if ~(isstruct(contract)&&isscalar(contract)&& ...
            isfield(contract,'valid')&&isequal(contract.valid,true)&& ...
            isfield(contract,'binding')&&isequal(binding,contract.binding))
        R.status='UNKNOWN_HLC_BINDING_CHANGED_SINCE_FLIGHT_START';return
    end
    R.source_limits_bound_this_flight=true;
    % The historical field now means *fresh source revalidation*, not a
    % cached-binding validation. Keep FALSE; do not misreport a source scan.
    R.source_limits_revalidated=false;
    % Never trust the caller's copied authority or stored EKF snapshot.
    R.stage='CAPTURE_DIRECT_HLC_AND_CURRENT_EKF';
    before=TRAJECTORY_C9_HLC_FORCE_AUTHORITY.current(binding,agent,time);
    R.current_authority_capture_count=R.current_authority_capture_count+1;
    R.presearch_snapshot=before;
    if ~before.valid,R.status=before.status;return;end
    if ~isequal(double(request.t0_s),double(before.tick_time_s))
        R.status='UNKNOWN_C9_START_NOT_CURRENT_EKF_TIME';return
    end
    R.live_EKF_mass_used_kg=before.estimated_payload_mass_kg;
    R.input_lower_4=double(before.authority.input_limits.lower(:));
    R.input_upper_4=double(before.authority.input_limits.upper(:));
    if ~(isequal(R.input_lower_4,double(binding.input_lower(:)))&& ...
         isequal(R.input_upper_4,double(binding.input_upper(:))))
        R.status='UNKNOWN_REAL_HLC_FOUR_BOUNDS_BINDING_CHANGED';return
    end
    R.stage='VERIFY_SAME_FLIGHT_NONMASS_CONTRACT';
    flight=TRAJECTORY_C9_FLIGHT_PARAMETER_CONTRACT.check( ...
        contract,binding,agent,time,request,before);
    R.flight_contract_check_count=R.flight_contract_check_count+1;
    if ~flight.valid,R.status=flight.status;return;end
    R.flight_parameter_contract_checked=true;
    R.stage='READ_REAL_EKF_START';
    state=agent.estimator.result.state;
    required={'pL','vL','pT','p','mL'};
    live=struct();
    for k=1:numel(required)
        % Supports both struct-valued and object-valued EKF state.
        key=required{k};
        live.(key)=state.(key);
    end
    request.current_ekf_state=live;
    request.authority=before.authority;
    % The SAME integrated pipeline performs the live EKF entry gate
    % BEFORE optimization. Calling it again here wasted work and did not
    % strengthen the mathematical condition. It remains mandatory there.
    R.stage='SAME_SIMULTANEOUS_C9_FULL_NOMINAL_PIPELINE';
    integrated=TRAJECTORY_C9_INTEGRATED_NOMINAL_PIPELINE(request);
    R.integrated_result=integrated;
    R.entry_gate_count=double(integrated.live_EKF_entry_supplied);
    R.entry_status=integrated.live_EKF_entry_status;
    R.entry_necessary_pass=integrated.live_EKF_entry_necessary_pass;
    R.search_elapsed_ms=integrated.total_elapsed_ms;
    R.nominal_model_complete=integrated.nominal_model_complete;
    % Synchronous MATLAB search receives request/value snapshots, not the
    % mutable Flight agent; there is no new EKF tick during this invocation.
    % Re-running current() and the nonmass Flight contract here checks the
    % same cached state twice. It is NOT a future tracking guarantee.
    % The eventual HLC COMMIT path must take a FRESH timestamped EKF/HLC
    % snapshot and validate it separately. This API never permits COMMIT.
    if ~integrated.nominal_model_complete
        R.status=integrated.status;return
    end
    R.stage='CHECK_NOMINAL_FOUR_INPUT_BOX_AGAINST_REAL_HLC_CLAMPS';
    P=integrated.search_result.final_certificate;
    Q=integrated.nominal_torque_certificate;
    if ~(isstruct(P)&&isstruct(Q)&& ...
         isfield(P,'certified_continuous_thrust')&& ...
         isfield(P,'thrust_lower_bound_N')&& ...
         isfield(P,'thrust_upper_bound_N')&& ...
         isfield(Q,'conditional_nominal_all_time_torque_proven')&& ...
         isfield(Q,'torque_bounds_Nm')&& ...
         isequal(P.certified_continuous_thrust,true)&& ...
         isequal(Q.conditional_nominal_all_time_torque_proven,true))
        R.status='UNKNOWN_NOMINAL_WRENCH_CERTIFICATES_MISSING';return
    end
    box=[double([P.thrust_lower_bound_N,P.thrust_upper_bound_N]); ...
         double(Q.torque_bounds_Nm)];
    if ~(isnumeric(box)&&isequal(size(box),[4 2])&& ...
         isreal(box)&&all(isfinite(box(:)))&&all(box(:,1)<=box(:,2)))
        R.status='UNKNOWN_NOMINAL_FOUR_WRENCH_BOX_INVALID';return
    end
    R.nominal_wrench_lower_4=box(:,1);
    R.nominal_wrench_upper_4=box(:,2);
    R.nominal_wrench_headroom_4x2=[box(:,1)-R.input_lower_4, ...
                                  R.input_upper_4-box(:,2)];
    R.nominal_4_input_within_live_software_limits= ...
        all(R.nominal_wrench_headroom_4x2(:)>=0);
    if ~R.nominal_4_input_within_live_software_limits
        R.status='UNKNOWN_NOMINAL_INPUT_BOX_EXCEEDS_REAL_HLC_CLAMPS';return
    end
    % Algebraically exact admissible correction box for the ACTUAL
    % PRE-CLAMP HLC request u_pre(t)=u_nom(t)+Delta_HLC(t). If the unknown
    % feedback/observer mismatch Delta_HLC is enclosed by this box for
    % every time, all software limits are met. This does NOT prove such
    % an enclosure: deriving it requires the actual HLC equations,
    % controller/observer reachability with actual Flight authority.
    % No new run-time source scan or duplicate trajectory audit here.
    R.admissible_HLC_preclamp_correction_lower_4= ...
        R.input_lower_4-box(:,1);
    R.admissible_HLC_preclamp_correction_upper_4= ...
        R.input_upper_4-box(:,2);
    R.HLC_feedback_bound_status= ...
        'REQUIRED_FUTURE_CLOSED_LOOP_WRENCH_ERROR_ENVELOPE__NOT_YET_PROVEN';
    % These are only the nominal inverse-dynamics inputs, NOT the
    % future unsaturated values that the feedback HLC will demand.
    R.candidate_available=true;
    R.status='CONDITIONAL_LIVE_EKF_C9_NOMINAL_4INPUT_WITHIN_REAL_HLC_CLAMPS__HLC_UNCERTIFIED';
    end
end

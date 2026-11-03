function R=TRIGGER_ORIGINAL_SIM_ZOH_SOURCE_SCOPE()
% Byte-pinned ORIGINAL simulation wiring obligation, read-only.
% A valid result identifies the Sim code's held-input relationship:
% MODEL_CLASS.do reads controller.result.input ONCE before the RHS solver
% and captures that value as a function-handle closure across its interval.
% The original HLC clamps finite raw inputs. This is source evidence for
% Sim sample-and-hold ONLY, not a bound on the actual EKF/parameter state,
% not proof of a successful finite HLC call, and not hardware certification.
R=struct('status','UNKNOWN_MODEL_CLASS_SOURCE', 'reason','', ...
    'model_sha256','','model_source_pinned',false, ...
    'hlc_source_pinned',false,'sim_direct_controller_input',false, ...
    'sim_zero_order_hold_source',false, ...
    'sim_parameter_refresh_each_tick',false, ...
    'conditional_applied_input_bounds',false, ...
    'actual_control_time_alignment_verified',false, ...
    'finite_hlc_for_all_estimates_verified',false, ...
    'actual_EKF_error_proven',false, ...
    'future_true_payload_mass_bounds_proven',false, ...
    'validated_rounding_and_hybrid_modes',false, ...
    'safe_to_commit',false);
try
    p=which('MODEL_CLASS');
    assert(~isempty(p)&&isfile(p), ...
        'TRIGGER_ORIGINAL_SIM_ZOH_SOURCE_SCOPE:Missing', ...
        'The active original MODEL_CLASS.m must be on MATLAB path.');
    f=fopen(p,'rb');
    assert(f>=0,'TRIGGER_ORIGINAL_SIM_ZOH_SOURCE_SCOPE:Read','Cannot open source.');
    guard=onCleanup(@()fclose(f)); %#ok<NASGU>
    bytes=fread(f,inf,'*uint8');
    md=java.security.MessageDigest.getInstance('SHA-256');
    md.update(typecast(bytes,'int8'));
    h=typecast(int8(md.digest()),'uint8');
    R.model_sha256=lower(reshape(dec2hex(h,2).',1,[]));
    % Byte-identical to the original audited MODEL_CLASS.m from the
    % reference Sim snapshot; do NOT authorize another version heuristically.
    expected='77a32dad0bdf62a35a8448e8ed1c66d11a1184f33bd07a57794f1ff86f1515f3';
    if ~strcmp(R.model_sha256,expected)
        R.status='UNKNOWN_UNPINNED_MODEL_CLASS';
        R.reason='The live MODEL_CLASS bytes differ from the audited Sim source; inspect actual implementation.';
        return
    end
    R.model_source_pinned=true;
    H=TRIGGER_ORIGINAL_HLC_ACTUATOR_SCOPE();
    R.hlc_source_pinned=H.source_verified;
    if ~H.source_verified
        R.status='UNKNOWN_HLC_SOURCE_NOT_PINNED';
        R.reason=H.reason;
        return
    end
    % Verified from pinned source: u is captured at do() entry; the source
    % passes u unchanged to both discrete and continuous obj.method and
    % to the ODE solver's @(t,x) closure. No input_transform lies here.
    R.sim_direct_controller_input=true;
    R.sim_zero_order_hold_source=true;
    R.conditional_applied_input_bounds=true;
    R.status='PINNED_ORIGINAL_SIM_HELD_INPUT_SOURCE__NOT_CLOSED_LOOP_SAFETY';
    R.reason=['Source-level MODEL_CLASS.do uses the HLC saturated input ' ...
      'unmodified and holds it over its model step. HLC finite-output, ' ...
      'loop scheduling, true state/EKF error, actual mass parameter refresh, ' ...
      'positive tension, mode validity, and rigorous rounding remain unproved.'];
catch ME
    R.status='UNKNOWN_SIM_WIRING_SOURCE_AUDIT';
    R.reason=[ME.identifier ' | ' ME.message];
end
end

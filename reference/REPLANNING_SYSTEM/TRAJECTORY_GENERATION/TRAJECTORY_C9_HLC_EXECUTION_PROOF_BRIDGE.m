function R=TRAJECTORY_C9_HLC_EXECUTION_PROOF_BRIDGE(binding,agent,time,proof)
%TRAJECTORY_C9_HLC_EXECUTION_PROOF_BRIDGE  Read-only LIVE-HLC proof interface.
% Called at an explicitly chosen current Flight tick (not in the 25ms
% Flight loop by default). REAL HLC_SUSPENDED_LOAD source, active instance,
% gains, sample period, four input clamps and freshest EKF payload mass
% are validated by the existing TRAJECTORY_C9_HLC_FORCE_AUTHORITY.current.
% It composes three mathematically valid CONDITIONAL lemmas:
%   sampled virtual error -> separate UAV/cable/payload error radii;
%   beta2\rhs perturbation -> four unsaturated HLC input bounds.
%
% RESEARCH MODEL-SCOPE CONTRACT (explicit proof.model_scope):
%   'IDEAL_VIRTUAL_SIMULATION': the mathematical system IS the sampled
%   virtual error model. At each Flight tick, the CURRENT EKF load-mass
%   estimate is the mass parameter of THAT model (not a true-mass claim).
%   The model still requires proven initial virtual errors, uniform virtual
%   residual enclosures (zero only if exactly zero by the model equations),
%   full-time input enclosures and an already certified nominal corridor.
%   Only MODEL-CONDITIONAL statements result; physical execution and
%   actual HLC/EKF/plant equivalence are neither claimed nor required.
%   Omitted scope preserves the previous REAL_CLOSED_LOOP obligations.
%
% REQUIREMENTS STILL NOT PROVIDED FOR REAL_CLOSED_LOOP: a mechanically or
% interval-certified bound on the mismatch between real HLC+EKF+plant and
% the sampled virtual chain (including mass changes and disturbances),
% exact interval enclosures of beta2, rhs and thrust throughout the tube,
% and directed-rounded continuous-time nominal geometric certificate.
% proof is a collection of externally CERTIFIED enclosures, not sampled
% Flight extrema. Data are never turned into a proof just by saying PASS.
% All outcomes keep safe_to_commit=false, including conditional PASS.
R=struct('status','UNKNOWN_LIVE_HLC_PROOF_BRIDGE_NOT_STARTED', ...
    'safe_to_commit',false,'actual_HLC_tracking_proven',false, ...
    'real_HLC_four_input_proven',false,'physical_trajectory_safe',false, ...
    'live_HLC_verified',false,'latest_EKF_mass_kg',NaN, ...
    'time_s',NaN,'input_lower',NaN(4,1),'input_upper',NaN(4,1), ...
    'live_preclamp_tick_status','NOT_REQUESTED', ...
    'live_preclamp_sample_4',NaN(4,1), ...
    'virtual_tube_status','NOT_EVALUATED', ...
    'physical_tube_status','NOT_EVALUATED', ...
    'input_lemma_status','NOT_EVALUATED', ...
    'uav_tube_m',NaN,'cable_tube_m',NaN,'payload_tube_m',NaN, ...
    'conditional_input_headroom',NaN, ...
    'conditional_raw_input_enclosure',NaN(4,2), ...
    'input_bound_mode','NORM_PERTURBATION', ...
    'conditional_mass_tension_status','NOT_REQUESTED', ...
    'conditional_tension_lower_N',NaN, ...
    'conditional_corridor_tube_status','NOT_REQUESTED', ...
    'conditional_corridor_min_clearance_m',NaN, ...
    'elapsed_ms',NaN,'model_scope','REAL_CLOSED_LOOP', ...
    'model_mass_convention','TRUE_MASS_ERROR_REQUIRED', ...
    'model_only_conditional_proven',false, ...
    'model_execution_tube_enabled',NaN, ...
    'guarantee_tier','UNSELECTED', ...
    'derived_tension_lower_N',NaN, ...
    'derived_tension_headroom_N',NaN, ...
    'scope','LIVE_HLC_SOURCE_PINNED_CONDITIONAL_MATH_ONLY');
clock=tic;
if ~isstruct(proof)||~isscalar(proof)
    R.status='UNKNOWN_EXTERNAL_PROOF_ENCLOSURES_MISSING';
    R.elapsed_ms=1000*toc(clock);return
end
% Avoid silently changing legacy real-plant assertions: the reduced
% simulation scope must be explicitly selected by the caller.
model_only=false;
if isfield(proof,'model_scope')
    requested=proof.model_scope;
    if isstring(requested)&&isscalar(requested)
        requested=char(requested);
    end
    if ~ischar(requested)||~isrow(requested)
        R.status='UNKNOWN_MODEL_SCOPE_INVALID';
        R.elapsed_ms=1000*toc(clock);return
    end
    if strcmp(requested,'IDEAL_VIRTUAL_SIMULATION')
        model_only=true;
    elseif ~strcmp(requested,'REAL_CLOSED_LOOP')
        R.status='UNKNOWN_MODEL_SCOPE_INVALID';
        R.elapsed_ms=1000*toc(clock);return
    end
end
% Three proof tiers. The optional tier is explicit, so a nominal-only
% study does not need fictional virtual-error or real-plant error bounds.
requested_tier='';
if isfield(proof,'guarantee_tier')
    requested_tier=proof.guarantee_tier;
    if isstring(requested_tier)&&isscalar(requested_tier)
        requested_tier=char(requested_tier);
    end
    if ~ischar(requested_tier)||~isrow(requested_tier)|| ...
            ~any(strcmp(requested_tier,{'NOMINAL','MODEL_TUBE','PHYSICAL_ROBUST'}))
        R.status='UNKNOWN_GUARANTEE_TIER';
        R.elapsed_ms=1000*toc(clock);return
    end
    if model_only&&strcmp(requested_tier,'PHYSICAL_ROBUST') || ...
            ~model_only&&~strcmp(requested_tier,'PHYSICAL_ROBUST')
        R.status='UNKNOWN_MODEL_SCOPE_TIER_CONFLICT';
        R.elapsed_ms=1000*toc(clock);return
    end
end
nominal_only=strcmp(requested_tier,'NOMINAL');
if ~isempty(requested_tier),R.guarantee_tier=requested_tier;end
if model_only
    R.model_scope='IDEAL_VIRTUAL_SIMULATION';
    R.model_mass_convention='LATEST_EKF_ESTIMATE_IS_MODEL_MASS_AT_TICK';
    R.scope='CONDITIONAL_IDEAL_VIRTUAL_MODEL_NOT_PLANT_EXECUTION';
end
try
    S=TRAJECTORY_C9_HLC_FORCE_AUTHORITY.current(binding,agent,time);
catch ME
    R.status=['UNKNOWN_LIVE_HLC_AUTHORITY_' ME.identifier];
    R.elapsed_ms=1000*toc(clock);return
end
if ~S.valid
    R.status=S.status;
    R.elapsed_ms=1000*toc(clock);return
end
R.live_HLC_verified=true;
R.time_s=S.tick_time_s;
R.latest_EKF_mass_kg=S.estimated_payload_mass_kg;
R.input_lower=S.input_limits_lower;
R.input_upper=S.input_limits_upper;
% Optional REAL current-tick diagnostics: post-HLC.do only. This uses the
% existing source-bound actual HLC witness, which checks the 28D reference
% sent, the yaw branch lift, the four raw inputs and the four clamps.
% One-time samples are never promoted to all-time proofs.
if isfield(proof,'expected_ref28') || isfield(proof,'reference_tolerance28')
    if ~(isfield(proof,'expected_ref28')&& ...
            isfield(proof,'reference_tolerance28'))
        R.status='UNKNOWN_INCOMPLETE_REAL_HLC_TICK_REFERENCE_BINDING';
        R.elapsed_ms=1000*toc(clock);return
    end
    try
        W=TRAJECTORY_C9_HLC_LIVE_ACTUATOR_TICK( ...
            binding,agent,time,proof.expected_ref28, ...
            proof.reference_tolerance28);
        R.live_preclamp_tick_status=W.status;
        R.live_preclamp_sample_4=W.unclamped_input_4;
    catch ME
        R.status=['UNKNOWN_REAL_HLC_TICK_' ME.identifier];
        R.elapsed_ms=1000*toc(clock);return
    end
end
% Keep previous norm-perturbation interface unchanged; direct interval
% evaluation is an explicitly selected alternative, not an implicit pass.
inputMode='NORM_PERTURBATION';
if isfield(proof,'input_bound_mode')
    v=proof.input_bound_mode;
    if isstring(v)&&isscalar(v), v=char(v);end
    if ~ischar(v)||~isrow(v)|| ...
            ~any(strcmp(v,{'NORM_PERTURBATION','DIRECT_INTERVAL'}))
        R.status='UNKNOWN_FOUR_INPUT_BOUND_MODE_INVALID';
        R.elapsed_ms=1000*toc(clock);return
    end
    inputMode=v;
end
R.input_bound_mode=inputMode;
if nominal_only,R.input_bound_mode='NOMINAL_CERTIFIED_INTERVAL';end
if nominal_only
    % Geometry-only research mode A does not claim tracked execution.
    % Full-time HLC raw-input enclosures are still required if HLC input
    % bounds are requested; no guessed initial-error or residual envelope.
    names={'horizon_s','shared_corridor', ...
        'nominal_preclamp_input_interval_4x2'};
else
    names={'initial_virtual_error_norm_bounds_4', ...
        'intersample_virtual_residual_bounds_4', ...
        'nominal_w_lower_mps2', ...
        'vertical_acceleration_residual_mps2','horizon_s'};
end
if ~nominal_only
    if strcmp(inputMode,'NORM_PERTURBATION')
        names=[names,{'nominal_preclamp_input_interval_4x2', ...
            'thrust_deviation_bound_N','inverse_beta2_norm_bound', ...
            'beta2_variation_norm_bound','rhs_variation_norm_bound', ...
            'nominal_torque_norm_bound'}];
    else
        names=[names,{'full_time_beta2_interval_3x3x2', ...
            'full_time_rhs_interval_3x2', ...
            'full_time_thrust_interval_1x2'}];
    end
end
if ~all(isfield(proof,names))
    R.status='UNKNOWN_MODEL_HLC_INTERVAL_OR_TUBE_BOUNDS_REQUIRED';
    R.elapsed_ms=1000*toc(clock);return
end
if nominal_only
    % The existing corridor gate takes a body-error witness. The zero
    % witness below is NOT a claim of zero HLC tracking error: in tier A
    % the proposition being checked is NOMINAL geometry only.
    P=struct('status', ...
        'CONDITIONAL_SEPARATE_UAV_CABLE_PAYLOAD_TUBE__EXECUTION_UNPROVEN', ...
        'uav_m',0,'payload_m',0,'cable_m',0, ...
        'payload_acceleration_error_mps2',0, ...
        'nominal_w_lower_mps2',0);
    R.virtual_tube_status='NOT_REQUIRED_NOMINAL_ONLY';
    R.physical_tube_status='NOT_REQUIRED_NOMINAL_ONLY';
    R.uav_tube_m=0;R.cable_tube_m=0;R.payload_tube_m=0;
else
    V=TRAJECTORY_C9_HLC_SAMPLED_VIRTUAL_TUBE( ...
        binding.controller_gains,binding.sample_dt_s, ...
        double(proof.horizon_s), ...
        proof.initial_virtual_error_norm_bounds_4, ...
        proof.intersample_virtual_residual_bounds_4);
    R.virtual_tube_status=V.status;
    if ~strcmp(V.status, ...
            'CONDITIONAL_SAMPLED_VIRTUAL_TUBE__REAL_HLC_MODEL_PENDING')
        R.status=V.status;R.elapsed_ms=1000*toc(clock);return
    end
    P=TRAJECTORY_C9_HLC_PHYSICAL_TUBE_LIFT( ...
        V,proof.nominal_w_lower_mps2, ...
        proof.vertical_acceleration_residual_mps2, ...
        S.authority.cable_length_m);
    R.physical_tube_status=P.status;
    if ~strcmp(P.status, ...
            'CONDITIONAL_SEPARATE_UAV_CABLE_PAYLOAD_TUBE__EXECUTION_UNPROVEN')
        R.status=P.status;R.elapsed_ms=1000*toc(clock);return
    end
    R.uav_tube_m=P.uav_m;R.cable_tube_m=P.cable_m;
    R.payload_tube_m=P.payload_m;
end
if nominal_only
    I=TRAJECTORY_C9_HLC_NOMINAL_FOUR_INPUT_GATE( ...
        S.input_limits_lower,S.input_limits_upper, ...
        proof.nominal_preclamp_input_interval_4x2);
elseif strcmp(inputMode,'DIRECT_INTERVAL')
    % For each t in the certified horizon, assume beta2(t), rhs(t),
    % Uf(t) belong to the provided intervals. Direct 3x3 Cramer's-rule
    % interval extension avoids guessed inverse-matrix sensitivity norms.
    % All-time provenance of the intervals is STILL an input obligation.
    I=TRAJECTORY_C9_HLC_FOUR_INPUT_INTERVAL_DIRECT( ...
        S.input_limits_lower,S.input_limits_upper, ...
        proof.full_time_beta2_interval_3x3x2, ...
        proof.full_time_rhs_interval_3x2, ...
        proof.full_time_thrust_interval_1x2, ...
        c9_direct_method(proof));
else
    I=TRAJECTORY_C9_HLC_FOUR_INPUT_PERTURBATION( ...
        S.input_limits_lower,S.input_limits_upper, ...
        proof.nominal_preclamp_input_interval_4x2, ...
        proof.thrust_deviation_bound_N, ...
        proof.inverse_beta2_norm_bound,proof.beta2_variation_norm_bound, ...
        proof.rhs_variation_norm_bound,proof.nominal_torque_norm_bound);
end
R.input_lemma_status=I.status;
R.conditional_input_headroom=I.minimum_conditional_headroom;
R.conditional_raw_input_enclosure=I.conditional_raw_input_enclosure;
% Optional ALL-TIME corridor composition. In REAL_CLOSED_LOOP a separate
% certified true-mass error bound is still mandatory; in the explicitly
% chosen IDEAL_VIRTUAL_SIMULATION model it is not a physical requirement.
request_mass=isfield(proof,'certified_true_mass_error_bound_kg');
request_corridor=isfield(proof,'shared_corridor');
if ~isempty(requested_tier)&&~request_corridor
    R.status='UNKNOWN_PROOF_TIER_REQUIRES_CORRIDOR';
    R.elapsed_ms=1000*toc(clock);return
end
if model_only
    if request_mass
        R.status='UNKNOWN_IDEAL_MODEL_CONFLICTING_TRUE_MASS_BINDING';
        R.elapsed_ms=1000*toc(clock);return
    end
elseif xor(request_mass,request_corridor)
    R.status='UNKNOWN_INCOMPLETE_REAL_HLC_MASS_CORRIDOR_BINDING';
    R.elapsed_ms=1000*toc(clock);return
end
if request_corridor
    c=proof.shared_corridor;
    if ~isstruct(c)||~isscalar(c)||~isfield(c,'guide_piece')|| ...
            ~isstruct(c.guide_piece)|| ...
            ~all(isfield(c.guide_piece,{'t0_s','duration_s'}))|| ...
            ~isequal(double(c.guide_piece.t0_s),double(S.tick_time_s))|| ...
            ~(double(c.guide_piece.duration_s)<=double(proof.horizon_s))
        R.status='UNKNOWN_HLC_CORRIDOR_TIME_HORIZON_NOT_ALIGNED';
        R.elapsed_ms=1000*toc(clock);return
    end
    Pgate=P;
    if ~model_only
        % Tier C spatial uncertainty is ADDITIONAL to the model tracking
        % tube. Without all-time certified per-body radii, robust corridor
        % safety is not a valid conclusion. No hidden zero assumptions.
        if ~isfield(proof,'certified_physical_position_error_bounds_m')
            R.status='UNKNOWN_PHYSICAL_BODY_ERROR_BOUNDS_MISSING';
            R.elapsed_ms=1000*toc(clock);return
        end
        E=proof.certified_physical_position_error_bounds_m;
        if ~isstruct(E)||~isscalar(E)|| ...
                ~all(isfield(E,{'uav_m','payload_m','cable_m'}))
            R.status='UNKNOWN_PHYSICAL_BODY_ERROR_BOUNDS_INVALID';
            R.elapsed_ms=1000*toc(clock);return
        end
        values={E.uav_m,E.payload_m,E.cable_m};
        if ~all(cellfun(@(x)isnumeric(x)&&isreal(x)&&isscalar(x),values))
            R.status='UNKNOWN_PHYSICAL_BODY_ERROR_BOUNDS_INVALID';
            R.elapsed_ms=1000*toc(clock);return
        end
        e=double([values{:}]);
        if numel(e)~=3|| ...
                any(~isfinite(e))||any(e<0)
            R.status='UNKNOWN_PHYSICAL_BODY_ERROR_BOUNDS_INVALID';
            R.elapsed_ms=1000*toc(clock);return
        end
        Pgate.uav_m=P.uav_m+e(1);
        Pgate.payload_m=P.payload_m+e(2);
        % The cable-specific physical deviation is not interchangeable
        % with an endpoint deviation. Keep it ABOVE the affine endpoint
        % envelope to avoid accidentally dropping cable bending.
        extra_model_cable=max(0,P.cable_m-max(P.uav_m,P.payload_m));
        Pgate.cable_m=max(Pgate.uav_m,Pgate.payload_m)+ ...
            extra_model_cable+e(3);
        if ~isfinite(Pgate.cable_m)
            R.status='UNKNOWN_PHYSICAL_CABLE_TUBE_OVERFLOW';
            R.elapsed_ms=1000*toc(clock);return
        end
    end
    if model_only
        % The research tube switch is independent of the input bounds.
        % OFF certifies NOMINAL geometry only, never simulated execution.
        if ~isfield(c,'proof_options')|| ...
                ~isstruct(c.proof_options)|| ...
                ~isfield(c.proof_options,'robust_tube_enabled')|| ...
                ~(isnumeric(c.proof_options.robust_tube_enabled)|| ...
                  islogical(c.proof_options.robust_tube_enabled))|| ...
                ~isscalar(c.proof_options.robust_tube_enabled)|| ...
                ~any(double(c.proof_options.robust_tube_enabled)==[0 1])
            R.status='UNKNOWN_IDEAL_MODEL_TUBE_FLAG_INVALID';
            R.elapsed_ms=1000*toc(clock);return
        end
        R.model_execution_tube_enabled= ...
            double(c.proof_options.robust_tube_enabled);
        if nominal_only&&R.model_execution_tube_enabled~=0
            R.status='UNKNOWN_NOMINAL_TIER_WITH_ROBUST_TUBE_ON';
            R.elapsed_ms=1000*toc(clock);return
        end
        if strcmp(requested_tier,'MODEL_TUBE')&& ...
                R.model_execution_tube_enabled~=1
            R.status='UNKNOWN_MODEL_TUBE_TIER_DISABLED';
            R.elapsed_ms=1000*toc(clock);return
        end
        if R.model_execution_tube_enabled==0
            Pgate.uav_m=0;
            Pgate.payload_m=0;
            Pgate.cable_m=0;
        end
    end
    CT=TRAJECTORY_C9_HLC_CORRIDOR_TUBE_GATE(c,Pgate);
    R.conditional_corridor_tube_status=CT.status;
    R.conditional_corridor_min_clearance_m=CT.min_total_residual_m;
    if ~CT.conditional_all_body_separated
        R.status=CT.status;
        R.elapsed_ms=1000*toc(clock);return
    end
    if model_only
        % Tier A: EXACT nominal acceleration, zero tracking inflation.
        % Tier B: MODEL error from the existing virtual-tube lemma.
        % No extra physical-mass or model-discrepancy guards.
        if R.model_execution_tube_enabled==0
            tier='NOMINAL';delta_model=[];
        else
            tier='MODEL_TUBE';
            delta_model=P.payload_acceleration_error_mps2;
        end
        MT=TRAJECTORY_C9_TAUT_TENSION_TIERS( ...
            CT.recomputed_nominal_w_lower_mps2, ...
            S.estimated_payload_mass_kg,tier,delta_model,[],[], ...
            S.authority.minimum_tension_N);
    else
        % Tier C explicitly requires source-certified additional plant
        % acceleration and mass uncertainty: never invent either bound.
        if ~isfield(proof,'certified_physical_acceleration_error_mps2')
            R.status='UNKNOWN_PHYSICAL_ACCELERATION_ERROR_NOT_PROVIDED';
            R.elapsed_ms=1000*toc(clock);return
        end
        MT=TRAJECTORY_C9_TAUT_TENSION_TIERS( ...
            CT.recomputed_nominal_w_lower_mps2, ...
            S.estimated_payload_mass_kg,'PHYSICAL_ROBUST', ...
            P.payload_acceleration_error_mps2, ...
            proof.certified_physical_acceleration_error_mps2, ...
            proof.certified_true_mass_error_bound_kg, ...
            S.authority.minimum_tension_N);
    end
    R.guarantee_tier=MT.tier;
    R.conditional_mass_tension_status=MT.status;
    if model_only&&isempty(requested_tier) && ...
            MT.strict_tautness_proven_under_assumptions
        % Preserve older CHECK output for callers that omit tier.
        R.conditional_mass_tension_status= ...
            'CONDITIONAL_IDEAL_MODEL_POSITIVE_TAUT_TENSION';
    end
    R.conditional_tension_lower_N=MT.tension_lower_N;
    R.derived_tension_lower_N=MT.tension_lower_N;
    R.derived_tension_headroom_N=MT.tension_headroom_N;
    target=['CONDITIONAL_' MT.tier '_POSITIVE_TAUT_TENSION'];
    if ~strcmp(MT.status,target)
        R.status=MT.status;
        R.elapsed_ms=1000*toc(clock);return
    end
end
if I.conditional_headroom_all_positive
    if model_only
        if request_corridor
            if R.model_execution_tube_enabled==1
                R.status=['CONDITIONAL_IDEAL_VIRTUAL_MODEL_ALL_BODY_' ...
                    'TUBE_TENSION_FOUR_INPUT__MODEL_BOUNDS_ASSUMED'];
                R.model_only_conditional_proven=true;
            else
                R.status=['CONDITIONAL_IDEAL_VIRTUAL_MODEL_NOMINAL_' ...
                    'CORRIDOR_TENSION_FOUR_INPUT__NO_EXECUTION_SAFETY'];
                R.model_only_conditional_proven=false;
            end
        else
            R.status=['CONDITIONAL_IDEAL_VIRTUAL_MODEL_TUBE_FOUR_INPUT_' ...
                '__MODEL_BOUNDS_ASSUMED'];
        end
    elseif request_mass&&request_corridor
        R.status=['CONDITIONAL_HLC_TUBE_INPUT_TENSION_CORRIDOR_' ...
             'PROOF_OBLIGATIONS_REMAIN'];
    else
        R.status='CONDITIONAL_LIVE_HLC_TUBE_AND_FOUR_INPUT__NOT_EXECUTION_PROVEN';
    end
else
    R.status=I.status;
end
% In model-only mode, conditional guarantees concern EXACTLY the explicitly
% defined virtual model. They are not a guarantee about the simulated
% plant trajectory unless equality to the model is separately established.
% Never turn a conditional numerical calculation into real-Flight COMMIT.
R.elapsed_ms=1000*toc(clock);
end

function mode=c9_direct_method(proof)
% Optional exact interval-algebra A/B. Original Cramer mode is the default.
mode='CRAMER';
if isfield(proof,'direct_interval_solver_mode')
    mode=proof.direct_interval_solver_mode;
end
end

function A=AF7R58R114_RUNTIME_STATE(P)
% READ ONLY: report ACTUAL production integration and incomplete proofs.
% Call after the existing Flight, not inside the 25 ms loop.
assert(isa(P,'REPLANNING_SYSTEM'),'R114:RequireLiveReplanner', ...
    'Pass the live REPLANNING_SYSTEM handle (e.g., replanner_instance).');
A=struct();
A.active_reference_mode=char(P.active_reference_mode);
A.active_reference_version=double(P.active_reference_version);
A.trigger_backend='LEGACY_REFERENCE_PLUS_EKF_NOW_MONITOR';
A.physical_hlc_preview_enabled=logical(P.r108_dual_engine_armed) && ...
    isa(P.r108_live_physical_preview_fn,'function_handle');
A.generator_backend='af7r46_single_core_proposal_compact';
if logical(P.r108_dual_engine_armed) && ...
   isa(P.r108_live_c7_generator_fn,'function_handle')
    A.generator_backend='r108_live_c7_generator_fn';
end
A.obstacle_sensing_range_m=double(P.trigger_dist);
A.collision_trigger_horizon_s=double(P.horizon_s);
A.flight_period_s=double(P.prediction_dt);
A.stage1_count=double(P.staged_replan_stage1_count);
A.stage2_count=double(P.staged_replan_stage2_count);
A.commit_count=double(P.staged_replan_commit_count);
A.fail_count=double(P.staged_replan_fail_count);
A.last_pipeline_reason=char(P.staged_replan_last_reason);
A.last_pipeline_state=char(P.staged_replan_state);
A.last_stage1_ms=double(P.staged_replan_last_stage1_ms);
A.last_stage2_ms=double(P.staged_replan_last_stage2_ms);
A.last_stage3_ms=double(P.staged_replan_last_stage3_ms);
A.simulation_only_commit=logical(P.R26_SIMULATION_ONLY_EXPERIMENT);
A.commit_policy=char(P.AVOIDANCE_COMMIT_POLICY);
A.reference_only_commit_count=double(P.reference_only_commit_count);
A.execution_aware_commit_count=double(P.execution_aware_commit_count);
A.command_limits=P.command_limit_certificate;
A.actuator_dynamics=P.actuator_dynamics_certificate;
A.hlc_robust=P.hlc_robust_certificate;
A.real_hlc_closed_loop_certified=isstruct(A.actuator_dynamics) && ...
    isfield(A.actuator_dynamics,'certified') && ...
    isequal(A.actuator_dynamics.certified,true) && ...
    isstruct(A.hlc_robust) && isfield(A.hlc_robust,'certified') && ...
    isequal(A.hlc_robust.certified,true);
A.note='R114 geometric MEX proof is independent of real HLC/actuator/tube certification.';
end

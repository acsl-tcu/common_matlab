function [P,RAW] = af7r34_single_core_proposal(replanner,CTX)
% One hot-path MEX call for the heavy R8/R9 planner/certificate core.
%
% Basic trigger/context validation remains outside this helper.  The native
% core covers episode -> corridor -> sign-QP -> control points ->
% R9 Stage A/B -> rejoin -> continuous whole-body.

assert(isa(replanner,'REPLANNING_SYSTEM'));
assert(exist('af7r34_single_core_mex','file')==3,'[AF7R34] core MEX missing.');

pkt=af7r34_make_core_packet(replanner,CTX,[]);
RAW=af7r34_single_core_mex(1,pkt);

P=replanner.empty_bezier_d15_c7_proposal();
P.shadow_proposal=true;
P.backend='BEZIER_D15_C7';
P.reason_code=double(RAW.reason_code);
P.t0=double(RAW.t0);
P.tm=double(RAW.tm);
P.trigger_collision_time=double(RAW.tm);
P.hold_end=double(RAW.hold_end);
P.t1=double(RAW.t1);
P.timing_projected=false;
P.obstacle_id=double(CTX.request.trigger_obstacle_id);
P.component_code=double(CTX.request.trigger_component_code);
P.base_eval_fn=replanner.snapshot_active_reference_eval_fn();
P.base_flat_eval_fn=replanner.snapshot_active_reference_flat_eval_fn();
P.execution_tracking_inflation_m=0;

EPI=struct();
EPI.certified=true;
EPI.continuous_time=true;
EPI.sampled_acceptance=false;
EPI.t0=P.t0;
EPI.search_end_s=P.t1;
EPI.obstacle_count=double(CTX.obstacles.count);
EPI.episode_count=double(RAW.episode_count);
EPI.component_count=double(RAW.component_count);
EPI.episodes=struct('obstacle_index',{},'obstacle_id',{}, ...
    'start_s',{},'end_s',{},'component_index',{});
for k=1:EPI.episode_count
    EPI.episodes(k)=struct( ...
        'obstacle_index',double(RAW.episode_obstacle_index(k)), ...
        'obstacle_id',double(RAW.episode_obstacle_id(k)), ...
        'start_s',double(RAW.episode_start(k)), ...
        'end_s',double(RAW.episode_end(k)), ...
        'component_index',double(RAW.episode_component_index(k)));
end
EPI.components=struct('start_s',{},'end_s',{},'duration_s',{},'episode_indices',{});
for j=1:EPI.component_count
    ids=find(double(RAW.episode_component_index)==j);
    EPI.components(j)=struct('start_s',double(RAW.component_start(j)), ...
        'end_s',double(RAW.component_end(j)), ...
        'duration_s',double(RAW.component_end(j)-RAW.component_start(j)), ...
        'episode_indices',ids);
end
EPI.theorem=['ADAPTIVE_CONTINUOUS_SUPPORT_INTERVAL_COVER__' ...
    'SAFE_ONLY_BY_WHOLE_LEAF_CERTIFICATE__UNRESOLVED_LEQ_LIVE_DT_IS_UNSAFE'];
P.unsafe_episode_certificate=EPI;

SYN=struct('certified',~isempty(RAW.control_points), ...
    'continuous_time',true,'sampled_acceptance',false, ...
    'representation','PIECEWISE_DEGREE15_C7_WITH_AFFINE_UNSAFE_COMPONENTS', ...
    'component_count',double(RAW.component_count), ...
    'episode_count',double(RAW.episode_count), ...
    'qp_solver','AF7R34_SINGLE_NATIVE_CORE_EXACT_ACTIVE_SET', ...
    'objective','EXACT_INTEGRAL_SQUARED_CORRECTION_SNAP_ON_SAFE_TRANSITIONS', ...
    'terminal_rejoin_source','FINITE_REFERENCE_DOMAIN_END', ...
    't1',P.t1, ...
    'towing_theorem',['AFFINE_UNSAFE_CORRECTION_IMPLIES_D2_ZERO__Q_EQUALS_BASE__' ...
       'PAYLOAD_UAV_CABLE_COMMON_TRANSLATION'], ...
    'safety_authority','INDEPENDENT_CONTINUOUS_WHOLE_BODY_CERTIFICATE');
P.piecewise_synthesis_certificate=SYN;
P.corridor_certificate=SYN;

P.r9_stage_timing=struct('episode_ms',double(RAW.episode_ms), ...
    'synthesis_ms',double(RAW.synthesis_ms), ...
    'candidate_loop_ms',double(RAW.candidate_ms), ...
    'total_internal_ms',double(RAW.total_ms));
P.piecewise_speed_pass_count=double(RAW.speed_pass_count);
P.piecewise_rejoin_pass_count=double(RAW.rejoin_pass_count);
P.piecewise_geometry_test_count=double(RAW.geometry_test_count);

if ~logical(RAW.success)
    P.success=false;
    return;
end

P.generated=true;
P.success=true;
P.c7_exact=true;
P.representation_code=2;
P.degree=15;
P.piecewise_segment_t0=double(RAW.segment_t0(:).');
P.piecewise_segment_t1=double(RAW.segment_t1(:).');
P.piecewise_control_points=RAW.control_points;
C=double(RAW.component_count);
kind=cell(1,2*C+1);
for j=1:(2*C+1)
    if mod(j,2)==1, kind{j}='SAFE_TRANSITION';
    else, kind{j}='UNSAFE_AFFINE_TOWING_CORRIDOR'; end
end
P.piecewise_segment_kind=kind;
P.piecewise_component_start=double(RAW.component_start(:).');
P.piecewise_component_end=double(RAW.component_end(:).');
P.piecewise_component_position=double(RAW.component_position);
P.piecewise_component_velocity=double(RAW.component_velocity);
P.piecewise_corridor_signs=double(RAW.corridor_signs(:));
P.piecewise_qp_cost=double(RAW.qp_cost);
P.correction_derivative_ub=double(RAW.correction_derivative_ub(:));
P.displacement_norm_m=double(RAW.max_control_norm_m);
P.displacement=double(RAW.displacement(:));
P.direction=double(RAW.direction(:));

P.towing_structure_certificate=struct( ...
    'certified',true,'continuous_time',true,'sampled_acceptance',false, ...
    'continuity_order',7,'unsafe_component_affine_correction',true, ...
    'unsafe_component_correction_acceleration_zero',true, ...
    'q_equals_base_on_unsafe_components',true, ...
    'uav_payload_cable_common_translation_on_unsafe_components',true, ...
    'internal_component_velocity_free',true,'endpoint_zero_jet_order',7, ...
    'theorem',['D2_CORRECTION_ZERO_IMPLIES_IDENTICAL_SPECIFIC_FORCE__' ...
       'IDENTICAL_Q__COMMON_WHOLE_BODY_TRANSLATION']);

targetVersion=double(CTX.request.next_reference_version);
SC=struct();
SC.certified=true;
SC.continuous_time=true;
SC.sampled_acceptance=false;
SC.reference_mode='REPAIRED';
SC.reference_version=targetVersion;
SC.payload_speed_UB_m_s=double(RAW.speed_payload);
SC.payload_accel_UB_m_s2=double(RAW.speed_accel);
SC.payload_jerk_UB_m_s3=double(RAW.speed_jerk);
SC.specific_force_norm_LB_m_s2=double(RAW.speed_sigma);
SC.qdot_UB_rad_s=double(RAW.speed_qdot);
SC.uav_speed_UB_m_s=double(RAW.speed_uav);
SC.base_reference_mode=char(replanner.active_reference_mode);
SC.base_reference_version=double(replanner.active_reference_version);
if logical(RAW.speed_stageA)
    SC.source='TRIANGLE_BOUNDED_C7_REPAIRED_FLATNESS_KINEMATICS';
    SC.certificate_stage='R9_STAGE_A_GLOBAL_TRIANGLE';
else
    SC.source='R9_LOCAL_VECTOR_COUPLED_ADAPTIVE_BERNSTEIN_FLATNESS_SPEED';
    SC.certificate_stage='R9_STAGE_B_LOCAL_VECTOR';
    SC.local_leaf_count=double(RAW.speed_leaf_count);
    SC.local_split_count=double(RAW.speed_split_count);
    SC.vector_coupled_payload_velocity=true;
    SC.suspended_load_flatness=true;
    SC.fixed_depth_cap=false;
end
P.reference_speed_certificate=SC;

if logical(RAW.speed_stageA)
    P.derivative_bound_certificate=struct('certified',true, ...
        'continuous_time',true,'sampled_acceptance',false, ...
        'fixed_depth_cap',false,'orders_tightened',[1 2 3], ...
        'd1_ub',P.correction_derivative_ub(1), ...
        'd2_ub',P.correction_derivative_ub(2), ...
        'd3_ub',P.correction_derivative_ub(3), ...
        'theorem','ADAPTIVE_BERNSTEIN_CONVEX_HULL_SUP_BOUND_NO_FIXED_DEPTH');
    P.local_vector_speed_certificate=struct('certified',false, ...
        'not_required',true,'reason','STAGE_A_ALREADY_CERTIFIED');
else
    P.derivative_bound_certificate=struct('certified',false, ...
        'continuous_time',true,'sampled_acceptance',false);
    P.local_vector_speed_certificate=struct('certified',true, ...
        'mathematical_pass',true,'continuous_time',true, ...
        'sampled_acceptance',false,'fixed_depth_cap',false, ...
        'vector_coupled_payload_velocity',true, ...
        'suspended_load_flatness',true, ...
        'leaf_count',double(RAW.speed_leaf_count), ...
        'split_count',double(RAW.speed_split_count), ...
        'VQ_UB_m_s',double(RAW.speed_uav), ...
        'payload_speed_UB_m_s',double(RAW.speed_payload), ...
        'payload_accel_UB_m_s2',double(RAW.speed_accel), ...
        'payload_jerk_UB_m_s3',double(RAW.speed_jerk), ...
        'specific_force_norm_LB_m_s2',double(RAW.speed_sigma), ...
        'qdot_UB_rad_s',double(RAW.speed_qdot), ...
        'reason','LOCAL_VECTOR_CONTINUOUS_SPEED_CERTIFIED', ...
        'theorem','LOCAL_VECTOR_BERNSTEIN_PLUS_SUSPENDED_LOAD_FLATNESS_QDOT');
end

G=struct();
G.mathematical_pass=logical(RAW.geometry_pass);
G.certified=logical(RAW.geometry_pass);
G.continuous_time=true;
G.whole_body=true;
G.all_detected_obstacles=true;
G.static_obstacles_only=true;
G.sampled_acceptance=false;
G.finite_cover_speed_tube=true;
G.support_function_separation=true;
G.corrected_flatness_cable_direction=true;
G.ieee754_outward_closed=false;
G.reason_code=double(RAW.geometry_reason);
G.min_payload_margin_m=double(RAW.geometry_min_payload_margin);
G.min_uav_margin_m=double(RAW.geometry_min_uav_margin);
G.min_cable_margin_m=double(RAW.geometry_min_cable_margin);
G.min_margin_m=double(RAW.geometry_min_margin);
G.leaf_count=double(RAW.geometry_leaf_count);
G.accepted_leaf_count=double(RAW.geometry_accepted_leaf_count);
G.split_count=double(RAW.geometry_split_count);
G.max_depth_used=double(RAW.geometry_max_depth);
G.max_depth_cap=Inf;
G.resource_exhausted=false;
G.static_separation_open=G.reason_code==111;
G.representability_open=G.reason_code==112;
G.cover_theorem='MIDPOINT_STATIC_SUPPORT_MINUS_SPEED_TIMES_HALFWIDTH';
G.min_specific_force_norm_lb=double(RAW.speed_sigma);
G.max_qdot_ub=double(RAW.speed_qdot);
G.max_q_arc_radius=double(RAW.speed_qdot)*double(CTX.live.dt)/2;
G.max_q_arc_inflation_m=double(CTX.live.cable_length)*G.max_q_arc_radius;
G.tracking_inflation_m=0;
G.reference_only_authority=true;
G.final_system_authority=false;
P.geometry_shadow_certificate=G;
end

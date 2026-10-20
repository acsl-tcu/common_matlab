function OUT=AF7R58ZZY_R32_SELFTEST(classSource)
if nargin<1||isempty(classSource)
    classSource=which('REPLANNING_SYSTEM_R32_DIRECT_CANDIDATE_STORE');
end
clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R58ZZY-R32 | DIRECT-CANDIDATE-STORE SOURCE SELFTEST\n');
fprintf('====================================================================================================\n');
OUT=struct(); OUT.closed=false;

assert(~isempty(classSource)&&isfile(classSource),'[R32 SELFTEST] class source missing.');
K=fileread(classSource);

OUT.native_raw_absent=~contains(K,'P.native_raw=RAW');
OUT.direct_generator_present=contains(K, ...
    'r32_single_core_proposal_direct(obj,CTX)');
OUT.large_candidate_return_removed=~contains(K, ...
    'candidate=obj.af7r46_single_core_proposal_compact(CTX);');
OUT.external_candidate_assignment_removed=~contains(K, ...
    'obj.staged_replan_candidate=candidate;');
OUT.direct_candidate_property_store=contains(K, ...
    'obj.staged_replan_candidate=P;');
OUT.deadline_fail_clears_candidate=contains(K, ...
    'obj.staged_replan_candidate=struct();')&& ...
    contains(K,'R32_STAGE1_CONTROL_PERIOD_EXCEEDED');

OUT.r31_stage3_direct_store_retained=contains(K, ...
    'r31_prepare_core_bundle_direct(P,ta)')&& ...
    contains(K,'''r31_direct_store_exact'',true');
OUT.exact_batch_retained=contains(K,'af7r58i_prepare_batch_model(')&& ...
    ~contains(K,'af7r80_fast_batch_append_R29(');
OUT.schedule_unchanged=contains(K,'pipeline_intervals=3')&& ...
    contains(K,'pipeline_delay_s=3*double(obj.prediction_dt)');
OUT.continuous_geometry_guard=contains(K,'REFERENCE_GEOMETRY_CERTIFICATE_NOT_CLOSED');
OUT.continuous_flatness_guard=contains(K,'REFERENCE_ONLY_CONTINUOUS_FLATNESS_CERTIFICATE_NOT_CLOSED');
OUT.c7_guard=contains(K,'REFERENCE_ONLY_C7_CERTIFICATE_NOT_CLOSED');

OUT.packet_mex_present=(exist('af7r79_packet_retarget_mex','file')==3);
if OUT.packet_mex_present
    rev=af7r79_packet_retarget_mex(2);
    OUT.packet_mex_revision_exact=ischar(rev)&& ...
        strcmp(rev,'AF7R58ZZY_R29_PACKET_RETARGET_V1');
else
    OUT.packet_mex_revision_exact=false;
end

OUT.closed=OUT.native_raw_absent&&OUT.direct_generator_present&& ...
    OUT.large_candidate_return_removed&&OUT.external_candidate_assignment_removed&& ...
    OUT.direct_candidate_property_store&&OUT.deadline_fail_clears_candidate&& ...
    OUT.r31_stage3_direct_store_retained&&OUT.exact_batch_retained&& ...
    OUT.schedule_unchanged&&OUT.continuous_geometry_guard&& ...
    OUT.continuous_flatness_guard&&OUT.c7_guard&&OUT.packet_mex_revision_exact;

fprintf(' unused native RAW mirror absent             = %d\n',OUT.native_raw_absent);
fprintf(' direct Stage1 generator present             = %d\n',OUT.direct_generator_present);
fprintf(' large candidate return removed              = %d\n',OUT.large_candidate_return_removed);
fprintf(' external candidate assignment removed       = %d\n',OUT.external_candidate_assignment_removed);
fprintf(' candidate constructed directly in property  = %d\n',OUT.direct_candidate_property_store);
fprintf(' deadline fail clears direct candidate       = %d\n',OUT.deadline_fail_clears_candidate);
fprintf(' R31 direct Stage3 store retained            = %d\n',OUT.r31_stage3_direct_store_retained);
fprintf(' exact AF7R58I batch retained                = %d\n',OUT.exact_batch_retained);
fprintf(' measurement schedule unchanged              = %d\n',OUT.schedule_unchanged);
fprintf(' continuous geometry guard retained          = %d\n',OUT.continuous_geometry_guard);
fprintf(' continuous flatness guard retained          = %d\n',OUT.continuous_flatness_guard);
fprintf(' C7 guard retained                           = %d\n',OUT.c7_guard);
fprintf(' packet MEX revision exact                   = %d\n',OUT.packet_mex_revision_exact);
fprintf(' SELFTEST CLOSED                             = %d\n',OUT.closed);
fprintf('====================================================================================================\n');
assert(OUT.closed,'[R32 SELFTEST] source/theory closure failed.');
end

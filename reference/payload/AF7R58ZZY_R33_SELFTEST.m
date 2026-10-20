function OUT=AF7R58ZZY_R33_SELFTEST(classSource)
if nargin<1||isempty(classSource)
    classSource=which('REPLANNING_SYSTEM_R33_R31_PLUS_NO_NATIVE_RAW');
end
clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R58ZZY-R33 | ISOLATED NO-NATIVE-RAW SELFTEST\n');
fprintf('====================================================================================================\n');
OUT=struct(); OUT.closed=false;

assert(~isempty(classSource)&&isfile(classSource),'[R33 SELFTEST] class source missing.');
K=fileread(classSource);

% The one isolated change.
OUT.native_raw_absent=~contains(K,'P.native_raw=RAW');

% R31 Stage1 semantics MUST be restored exactly in structure:
% large candidate is returned, validated, then stored by caller.
OUT.r31_stage1_return_restored=contains(K, ...
    'candidate=obj.af7r46_single_core_proposal_compact(CTX);');
OUT.r31_stage1_external_store_restored=contains(K, ...
    'obj.staged_replan_candidate=candidate;');
OUT.r32_direct_candidate_path_absent= ...
    ~contains(K,'r32_single_core_proposal_direct') && ...
    ~contains(K,'R32_STAGE1_CONTROL_PERIOD_EXCEEDED');

% R31 successful Stage3 optimization remains.
OUT.r31_stage3_direct_store_retained=contains(K, ...
    'r31_prepare_core_bundle_direct(P,ta)')&& ...
    contains(K,'''r31_direct_store_exact'',true');
OUT.stage3_large_B_return_absent=~contains(K, ...
    '[ok,reason,B]=obj.prepare_staged_reference_only_bundle(P,ta);');
OUT.stage3_external_bundle_store_absent=~contains(K, ...
    'obj.staged_replan_bundle=B;');

% Theory/schedule authorities remain.
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

OUT.closed=OUT.native_raw_absent&& ...
    OUT.r31_stage1_return_restored&&OUT.r31_stage1_external_store_restored&& ...
    OUT.r32_direct_candidate_path_absent&&OUT.r31_stage3_direct_store_retained&& ...
    OUT.stage3_large_B_return_absent&&OUT.stage3_external_bundle_store_absent&& ...
    OUT.exact_batch_retained&&OUT.schedule_unchanged&& ...
    OUT.continuous_geometry_guard&&OUT.continuous_flatness_guard&& ...
    OUT.c7_guard&&OUT.packet_mex_revision_exact;

fprintf(' unused native RAW mirror absent          = %d\n',OUT.native_raw_absent);
fprintf(' R31 Stage1 candidate return restored     = %d\n',OUT.r31_stage1_return_restored);
fprintf(' R31 Stage1 external store restored       = %d\n',OUT.r31_stage1_external_store_restored);
fprintf(' R32 direct-candidate path absent         = %d\n',OUT.r32_direct_candidate_path_absent);
fprintf(' R31 direct Stage3 store retained         = %d\n',OUT.r31_stage3_direct_store_retained);
fprintf(' Stage3 large B return remains absent     = %d\n',OUT.stage3_large_B_return_absent);
fprintf(' Stage3 external B store remains absent   = %d\n',OUT.stage3_external_bundle_store_absent);
fprintf(' exact AF7R58I batch retained             = %d\n',OUT.exact_batch_retained);
fprintf(' measurement schedule unchanged           = %d\n',OUT.schedule_unchanged);
fprintf(' continuous geometry guard retained       = %d\n',OUT.continuous_geometry_guard);
fprintf(' continuous flatness guard retained       = %d\n',OUT.continuous_flatness_guard);
fprintf(' C7 guard retained                        = %d\n',OUT.c7_guard);
fprintf(' packet MEX revision exact                = %d\n',OUT.packet_mex_revision_exact);
fprintf(' SELFTEST CLOSED                          = %d\n',OUT.closed);
fprintf('====================================================================================================\n');
assert(OUT.closed,'[R33 SELFTEST] isolated-change closure failed.');
end

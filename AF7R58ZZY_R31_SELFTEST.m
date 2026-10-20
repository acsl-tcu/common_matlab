function OUT=AF7R58ZZY_R31_SELFTEST(classSource)
if nargin<1||isempty(classSource)
    classSource=which('REPLANNING_SYSTEM_R31_DIRECT_STAGE_STORE');
end
clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R58ZZY-R31 | DIRECT-STAGE-STORE SOURCE SELFTEST\n');
fprintf('====================================================================================================\n');
OUT=struct(); OUT.closed=false;

assert(~isempty(classSource)&&isfile(classSource),'[R31 SELFTEST] class source missing.');
K=fileread(classSource);

OUT.exact_batch_retained=contains(K,'af7r58i_prepare_batch_model(')&& ...
    ~contains(K,'af7r80_fast_batch_append_R29(');
OUT.schedule_unchanged=contains(K,'pipeline_intervals=3')&& ...
    contains(K,'pipeline_delay_s=3*double(obj.prediction_dt)');
OUT.direct_core_present=contains(K,'function [ok,reason] = r31_prepare_core_bundle_direct');
OUT.no_large_B_return=~contains(K,'[ok,reason,B]=obj.prepare_staged_reference_only_bundle(P,ta);');
OUT.no_external_bundle_assignment=~contains(K,'obj.staged_replan_bundle=B;');
OUT.no_stage3_oncleanup=~contains(K,'c=onCleanup(@()obj.r26_clear_core_prepare_tokens())');
OUT.direct_property_build=contains(K,'obj.staged_replan_bundle=struct( ...')&& ...
    contains(K,'''r31_direct_store_exact'',true');
OUT.exception_safe_token_clear=contains(K,'catch ME')&& ...
    contains(K,'obj.r26_clear_core_prepare_tokens();')&&contains(K,'rethrow(ME);');

% The exact continuous/certificate authorities are still present.
OUT.continuous_geometry_guard=contains(K,'REFERENCE_GEOMETRY_CERTIFICATE_NOT_CLOSED');
OUT.continuous_flatness_guard=contains(K,'REFERENCE_ONLY_CONTINUOUS_FLATNESS_CERTIFICATE_NOT_CLOSED');
OUT.c7_guard=contains(K,'REFERENCE_ONLY_C7_CERTIFICATE_NOT_CLOSED');
OUT.spacing_guard=contains(K,'af7r58i_commit_spacing_guard');
OUT.native_binding_guard=contains(K,'r30_validate_native_binding_once');

% Existing packet MEX must remain exact live authority.
OUT.packet_mex_present=(exist('af7r79_packet_retarget_mex','file')==3);
if OUT.packet_mex_present
    rev=af7r79_packet_retarget_mex(2);
    OUT.packet_mex_revision_exact=ischar(rev)&&strcmp(rev,'AF7R58ZZY_R29_PACKET_RETARGET_V1');
else
    OUT.packet_mex_revision_exact=false;
end

OUT.closed=OUT.exact_batch_retained&&OUT.schedule_unchanged&& ...
    OUT.direct_core_present&&OUT.no_large_B_return&&OUT.no_external_bundle_assignment&& ...
    OUT.no_stage3_oncleanup&&OUT.direct_property_build&&OUT.exception_safe_token_clear&& ...
    OUT.continuous_geometry_guard&&OUT.continuous_flatness_guard&&OUT.c7_guard&& ...
    OUT.spacing_guard&&OUT.native_binding_guard&&OUT.packet_mex_present&& ...
    OUT.packet_mex_revision_exact;

fprintf(' exact AF7R58I batch retained             = %d\n',OUT.exact_batch_retained);
fprintf(' measurement schedule unchanged           = %d\n',OUT.schedule_unchanged);
fprintf(' direct Stage3 core present               = %d\n',OUT.direct_core_present);
fprintf(' large B return removed                   = %d\n',OUT.no_large_B_return);
fprintf(' external bundle assignment removed       = %d\n',OUT.no_external_bundle_assignment);
fprintf(' Stage3 onCleanup object removed          = %d\n',OUT.no_stage3_oncleanup);
fprintf(' bundle constructed directly in property  = %d\n',OUT.direct_property_build);
fprintf(' token cleanup exception-safe             = %d\n',OUT.exception_safe_token_clear);
fprintf(' continuous geometry guard retained       = %d\n',OUT.continuous_geometry_guard);
fprintf(' continuous flatness guard retained       = %d\n',OUT.continuous_flatness_guard);
fprintf(' C7 guard retained                        = %d\n',OUT.c7_guard);
fprintf(' spacing guard retained                   = %d\n',OUT.spacing_guard);
fprintf(' native binding guard retained            = %d\n',OUT.native_binding_guard);
fprintf(' packet MEX revision exact                = %d\n',OUT.packet_mex_revision_exact);
fprintf(' SELFTEST CLOSED                          = %d\n',OUT.closed);
fprintf('====================================================================================================\n');
assert(OUT.closed,'[R31 SELFTEST] source/theory closure failed.');
end

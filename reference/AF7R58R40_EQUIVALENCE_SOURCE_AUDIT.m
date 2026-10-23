function Q=AF7R58R40_EQUIVALENCE_SOURCE_AUDIT()
% Read-only syntax-level assertions. Does NOT prove behavioral equivalence
% or validate math/real time; planned MATLAB simulation remains necessary.
s=fileread(which('REPLANNING_SYSTEM'));
assert(contains(s,'obj.staged_replan_candidate=candidate;'), ...
    'Candidate authority unexpectedly absent.');
assert(contains(s,'obj.r17_capture(''STAGE1_NATIVE_RETURNED'',t_now,stage_ms,candidate);'), ...
    'Native Stage1 summary was lost.');
assert(contains(s,'obj.r26_deferred_candidate=candidate;'), ...
    'Deferred native diagnostic authority was lost.');
assert(contains(s,'obj.r17_capture(''STAGE2_ENTERED'',t_now,NaN,struct());'), ...
    'Stage2 diagnostic-only change missing.');
assert(contains(s,'if strcmp(state,''IDLE'')'), ...
    'Proof of IDLE early return not found.');
Q=AF7R58R40_SOURCE_PREFLIGHT();
Q.stage1_native_diagnostic_preserved=true;
Q.require_runtime_comparison=true;
Q.exact_source_patch_equality_verified_by_packager=true;
end

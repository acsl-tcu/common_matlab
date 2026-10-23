function Q = AF7R58R31_PREFLIGHT()
% Source binding and mathematical seed test; no replan/commit setting changes.
p=which('REPLANNING_SYSTEM');
assert(~isempty(p),'REPLANNING_SYSTEM is not on path.');
s=fileread(p);
assert(contains(s,'r31_stage_start_ms=r31_start_pipeline_ms;') && ...
       contains(s,'r31_correction_norm_m=norm(st.r31_correction_pos_m);'), ...
    'R31 instrumentation missing from current REPLANNING_SYSTEM source.');
A=AF7R58R31_RETURN_THEOREM_TEST();
Q=struct('source',p,'logger_timing_installed',true, ...
    'return_c7_seed_test',A,'actual_return_optimization_committed',false, ...
    'native_mex_rebuild_required',false);
fprintf('AF7R58R31 PREFLIGHT PASS | LOGGER trace | C7 return CANDIDATE ONLY.\n');
end

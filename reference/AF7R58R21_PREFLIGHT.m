function Q=AF7R58R21_PREFLIGHT()
% Pure source preflight. Never changes runtime flags/commit policy.
Q=AF7R58R20_PREFLIGHT();
S=fileread(Q.source);
assert(contains(S,'r21_active_base_stage1_requests') && ...
    contains(S,'r21_active_base_stage2_prepared'), ...
    '[R21] Active-base diagnostic source not installed.');
assert(exist('AF7R58R21_AUTO_RESULT','file')==2, ...
    '[R21] result function missing.');
header=fullfile(fileparts(Q.source),'af7r36_core.hpp');
assert(isfile(header),'[R21] Header file missing.');
H=fileread(header);
assert(contains(H,'r58_reduce_hessian_selected') && ...
    contains(H,'r58_reduce_rows_selected') && ...
    contains(H,'const auto grams=[]()'), ...
    '[R21] Optimized C++ header missing.');
Q.r21_source_revision='R21_SELECTED_LOJET_HESSIAN_AND_ROW_EQUIVALENCE';
Q.mex_rebuild_required=true;
Q.reference_activation_forbidden=true;
Q.active_replan_requires_prior_commit=true;
fprintf('AF7R58R21 PREFLIGHT PASS | MEX REBUILD REQUIRED | NO COMMIT\n');
end

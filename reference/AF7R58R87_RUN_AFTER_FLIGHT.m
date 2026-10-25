function R=AF7R58R87_RUN_AFTER_FLIGHT(agent,tglobal,opts)
% R87 source-bound read-only regression: native R86 jet authority -> R81.
% Do not run inside Flight. No actuator, HLC, replanner commit action.
% R86 runner uses R86's OLD exact SHA so it is invalid after this hotfix;
% use this new source-bound entrypoint instead.
S=AF7R58R87_SOURCE_CHECK();
assert(S.already_installed,'R87:InstallHotfixAndRestartMATLABFirst');
R=AF7R58R83_RUN_AFTER_FLIGHT(agent,tglobal,opts);
assert(isstruct(R)&&isfield(R,'r81')&&isstruct(R.r81) && ...
    isfield(R.r81,'reference_candidate_found')&& ...
    isfield(R.r81,'production_commit_authorized') && ...
    ~logical(R.r81.production_commit_authorized) && ...
    isfield(R,'production_committed')&&~logical(R.production_committed), ...
    'R87:ReferenceOnlyNoProductionCommit');
R.r87_source_hotfix_valid=true;
R.r87_hlc_physical_safety_proved=false;
R.r87_nominal_tail_proved=false;
end

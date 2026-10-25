function R=AF7R58R86_RUN_AFTER_FLIGHT(agent,tglobal,opts)
% READ-ONLY integration regression on the current active reference.
% Do not invoke inside Flight; R83 routes R81 through the R86 exact native
% 0..6/7 derivative authority. No commit or controller action.
S=AF7R58R86_SOURCE_CHECK();
assert(S.source_already_patched,'R86:MustInstallAndRestartFirst');
R=AF7R58R83_RUN_AFTER_FLIGHT(agent,tglobal,opts);
assert(~logical(R.production_committed), ...
    'R86:NoFlightCommitAuthorized');
R.r86_input_consistency_tested=true;
R.r86_continuous_closed_loop_proved=false;
end

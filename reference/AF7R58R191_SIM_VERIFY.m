% ONE command in reference: AF7R58R191_SIM_VERIFY
% Runs unchanged canonical run_flight_headless SIM ONLY, saves full R191 data.
P191=AF7R58R189_ARM_SIM();
assert(P191.sim_flight_permitted,'R191:PreflightBlocked',...
 'R189 source preflight must pass.');
setenv('AF7R58R191_FAST_TICK','ON');
run(P191.runner_path);
R191F=AF7R58R191_POSTFLIGHT(replanner_instance);
R190F=AF7R58R190_REPORT(replanner_instance);
R189F=AF7R58R189_POSTFLIGHT(replanner_instance);
fprintf('[R191] events=%d C7=%d/%d; preview median=%.3fms max=%.3fms\n',...
 R191F.unique_risk_events,R191F.c7_generated,R191F.c7_attempts,...
 R191F.preview_p50_ms,R191F.preview_max_ms);
setenv('AF7R58R191_FAST_TICK','OFF');

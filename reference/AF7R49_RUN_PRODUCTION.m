% AF7R49_RUN_PRODUCTION
% One-button production-like run wrapper.
% run_flight_headless remains the simulation authority.  After it returns,
% only post-run diagnostics/plots are executed; nothing below feeds back to
% the controller/replanner/safety decision.

run_flight_headless

C49 = AF7R49C_PRODUCTION_CLOSEOUT(logger,replanner_instance,runtime_profile);
D49 = AF7R49_DASHBOARD(logger,replanner_instance,runtime_profile);

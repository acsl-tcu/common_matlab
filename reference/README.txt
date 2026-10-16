AF7R46 — current final two realtime bottlenecks

Targets only: Flight-entry 5 s and replan/commit 9.85--10.1 s.

10 s: identical AF7R36 MEX core; compact MATLAB candidate; evaluator handles deferred until after generator gate; changed-speed packet rebuilt with exact existing formulas from certified raw geometry; packet/sensor prepared before active authority mutation so a preparation deadline miss needs no rollback.

5 s: observer equations are unchanged. Only state-independent selector and physical constants are cached in existing pre-motion setup. Actual hidden support starts from the real Flight-entry event.

Current avoidance: after status 30, active_reference_mode is REPAIRED and write_active_output feeds repaired xd downstream. Before status 30 it is NOMINAL. Full execution-aware authority is still not active under REFERENCE_ONLY.

Run:
  addpath('AF7R46 package folder','-begin');
  I46=AF7R46_INSTALL_LAST_TWO_REALTIME();
  dbclear all; clear classes; clear functions; rehash
  A46=AF7R46_PRE_RUN_AUDIT(); A46.closed
  run_flight_headless
  B46=AF7R46B_LAST_TWO_POSTRUN_AUDIT(logger,replanner_instance);
  T25=AF7R25A_HIGHRES_TIMING_AUDIT(logger);
  R28=AF7R28_BLUE_SPIKE_ATTRIBUTION(runtime_profile,logger);
  T23=AF7R23A_R9_STAGE_TIMING_AUDIT(logger);

AF7R45 ded3e68fb1b7da5187985e1f171dc970afeb82d9ba09fecacbc4f07f553936d9
AF7R46 890876116b4782725d8c337e53923a14892a9b5f4f00a0e7de030b9d4433c446

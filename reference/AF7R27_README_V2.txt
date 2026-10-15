AF7R27 R10 V2
===============

This V2 fixes the syntax error in the original installer. The original
installer accidentally contained Python raw-string delimiters (r''' ... ''').

Keep these files together in one folder:
  AF7R27_INSTALL_R10_REALTIME_CORE_V2.m
  AF7R27_R10A_SNIPPET.txt
  AF7R27_R10B_SNIPPET.txt
  AF7R27_R10C_SNIPPET.txt
  AF7R27A_R10_SOURCE_THEOREM_AUDIT.m
  AF7R27B_CORRIDOR_HULL_ALGEBRA_AUDIT.m

Run:
  OUT27 = AF7R27_INSTALL_R10_REALTIME_CORE_V2();

  dbclear all
  clear classes
  clear functions
  rehash

  R27A = AF7R27A_R10_SOURCE_THEOREM_AUDIT();
  R27B = AF7R27B_CORRIDOR_HULL_ALGEBRA_AUDIT();

Require:
  R27A.closed == 1
  R27B.closed == 1

Then run:
  run_flight_headless

Afterward:
  T25 = AF7R25A_HIGHRES_TIMING_AUDIT(logger);
  T24 = AF7R24A_R9_SYNTHESIS_TIMING_AUDIT(logger);
  R26 = AF7R26_FULL_ITERATION_REPLAN_ATTRIBUTION(runtime_profile,logger);

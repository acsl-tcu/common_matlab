AF7R27 / R10 real-time core
============================

Goal:
  Remove the 9.85--10.40 s replanning retry burst while preserving the
  continuous-time suspended-load safety architecture.

Run:
  OUT27 = AF7R27_INSTALL_R10_REALTIME_CORE();

  dbclear all
  clear classes
  clear functions
  rehash

  R27A = AF7R27A_R10_SOURCE_THEOREM_AUDIT();
  R27B = AF7R27B_CORRIDOR_HULL_ALGEBRA_AUDIT();

Require:
  R27A.closed == 1
  R27B.closed == 1

Then:
  run_flight_headless

Afterward:
  T25 = AF7R25A_HIGHRES_TIMING_AUDIT(logger);
  R26 = AF7R26_FULL_ITERATION_REPLAN_ATTRIBUTION(runtime_profile,logger);

If AF7R24 instrumentation is still installed:
  T24 = AF7R24A_R9_SYNTHESIS_TIMING_AUDIT(logger);

Inspect:
  - first 9.85 s callback QPC
  - status -20 count
  - episode QPC
  - synthesis/QP QPC
  - R24 corridor row count (previously 508)
  - full_iteration band around 10 s
  - whether a reference-only commit now occurs

Windows/MATLAB timing is not an onboard hard-WCET proof. Final 4-ms authority
requires a later instrumentation-light outer-QPC pass and target-Linux/onboard
measurement.

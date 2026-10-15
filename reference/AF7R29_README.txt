AF7R29 / R11 noncommit hot-path cleanup
=========================================

Purpose
-------
AF7R28 showed:
- failed proposals after 13.9 s: backend callback <1 ms
- but sensor/trigger/replan stage: about 12--17 ms

The extra work is read-only theorem/foundation diagnostics on proposals that
cannot commit anyway.

R11 default behavior:
  ENABLE_NONCOMMIT_DIAGNOSTIC_EVAL = 0

Therefore:
- failed proposal -> lightweight status and return
- deadline-missed successful proposal -> lightweight status and return
- committable, within-deadline proposal -> full reference/execution safety
  evaluation is unchanged

Dedicated forensic runs can set the switch to 1.

Run
---
  O29 = AF7R29_INSTALL_R11_NONCOMMIT_HOTPATH();

  dbclear all
  clear classes
  clear functions
  rehash

  A29 = AF7R29A_R11_HOTPATH_SOURCE_AUDIT();
  assert(A29.closed==1)

  run_flight_headless

  R28 = AF7R28_BLUE_SPIKE_ATTRIBUTION(runtime_profile,logger);
  T25 = AF7R25A_HIGHRES_TIMING_AUDIT(logger);

Expected effect
---------------
- 13.9--16.35 s request spikes should collapse strongly.
- 9.85--10.40 s full-cycle spikes should reduce by post-callback diagnostic
  cost, BUT generator qpcCB itself remains above 4 ms until the native/MEX
  planner work is completed.

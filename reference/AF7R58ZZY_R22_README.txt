AF7R58ZZY-R22 ACCOUNTING DIAGNOSTIC
===================================
Purpose:
  Diagnose the unexplained +6..8 ms around the 9.85/9.875/9.900 s staged replan,
  while also splitting Stage1/2/3 internals. This is NOT an optimization release.

Authority:
  - Inherits R21 behavior (R19 numerical evaluator + R21 Stage2/Stage3 balance).
  - Production REPLANNING_SYSTEM.m is not modified.
  - Controller/HLC/plant math is not modified.
  - Trajectory/certificate acceptance/commit policy are not modified.
  - Runner is modified only to add tic/toc boundary measurements and to expand
    the already-fixed Flight reference list [timevarying,sload] exactly.
  - Detailed profiling adds overhead. Do not use R22 absolute timings as the
    final lean-performance authority. Use it to identify composition and close sums.

Run:
  AF7R58ZZY_R22_INSTALL
  run_flight_headless
  AF7R58ZZY_R22_REPORT

Send back:
  Console output from AF7R58ZZY_R22_REPORT, or AF7R58ZZY_R22_REPORT.txt.

Rollback:
  AF7R58ZZY_R22_ROLLBACK

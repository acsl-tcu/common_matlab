AF7R25 high-resolution timing package
=======================================

Purpose
-------
Replace reliance on nested MATLAB tic/toc for sub-millisecond / few-millisecond
timing conclusions with:

  QueryPerformanceCounter : wall-time interval
  QueryThreadCycleTime    : current-thread work counter
  QueryProcessCycleTime   : all MATLAB-process threads work counter

Cycle counters are NOT converted to milliseconds.

Execution
---------
1) Run:
   B25 = AF7R25_BUILD_HIGHRES_TIMER();

2) Install diagnostic instrumentation:
   I25 = AF7R25_INSTALL_HIGHRES_TIMING();

3) Reload class:
   dbclear all
   clear classes
   clear functions
   rehash

4) Run:
   run_flight_headless

5) Audit:
   T25 = AF7R25A_HIGHRES_TIMING_AUDIT(logger);

Important
---------
- R25 does not change trajectory/safety/solver/deadline logic.
- It resets all high-resolution fields to NaN on every request and tags them
  with generator_request_count, so stale values are rejected.
- QPC wall time is the elapsed-time measurement.
- Thread/process cycle counts are causal work indicators only.
- Windows/MATLAB measurements do not establish onboard Linux hard WCET.
- Once the algorithm approaches the 4 ms target, remove diagnostic inner-stage
  instrumentation and keep only an outer end-to-end QPC measurement.

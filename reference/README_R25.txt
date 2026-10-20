AF7R58ZZY-R25 LEAN ONLINE PIPELINE
=================================

Purpose
-------
Reduce the R23 end-to-end online runtime without changing trajectory mathematics,
continuous-time geometry/C7/speed certificate acceptance, activation time, stale
rejection, or controller law.

R25 changes
-----------
1. Waiting-cycle freshness deferral
   - GENERATED before Stage2 time and PREPARED before Stage3 time return before
     deep struct freshness comparisons.
   - The complete original freshness gate still runs immediately before Stage2
     and Stage3 work.

2. Stage1 AF7R33 model reuse
   - af7r34_make_core_packet receives the already-certified nominal/repaired
     batch model when its version binding is current.
   - Native candidate MEX and all candidate/certificate fields are unchanged.

3. Stage2 obstacle packet duplicate-check removal under a same-call-stack token
   - advance_staged_replan_pipeline first closes the complete freshness gate.
   - Only in that synchronous call stack, the packet rebuild reuses cached raw
     obstacle axes instead of re-checking static map equality and rebuilding the
     raw enclosing ellipsoid.
   - All target inflation formulas and moving-obstacle rejection remain.

4. Exact compiled-correction builder MEX
   - The same R23 recurrence Q_{r+1}=((15-r)/T)*diff(Q_r,1,2) is used.
   - Installer requires a bitwise comparison against the audited R23 MATLAB
     compiler before patching the runner.

5. Flight reference merge fast path
   - SUSPENDED_LOAD_REF_ADJUST edits the existing STATE_CLASS handle in place.
   - If r1.state and r2.state are the same handle, R25 skips merge_result's
     redundant state_copy but still copies all non-state top-level fields.
   - If they are not the same handle, it falls back to merge_result unchanged.

4 ms note
---------
R25 does NOT introduce a 4-ms normal replanning gate. The existing 4-ms
stop/fallback geometry certificate in REPLANNING_SYSTEM is left untouched.
The current performance objective is to reduce actual runtime as far as useful.

Install and run
---------------
1. Unzip this package.
2. In MATLAB, put the package folder on the path or open AF7R58ZZY_R25_INSTALL.m.
3. Run:

   AF7R58ZZY_R25_INSTALL

   The installer verifies the exact audited R23/R21/R19/production/runner SHA
   values, builds the small MEX, runs equivalence tests, backs up the current
   runner, and patches only run_flight_headless.m.

4. Run a clean simulation:

   run_flight_headless

5. Then run:

   AF7R58ZZY_R25_REPORT

6. Return AF7R58ZZY_R25_REPORT.txt and AF7R58ZZY_R25_OUT.mat (or paste the
   console report) to ChatGPT.

Production files intentionally not modified
-------------------------------------------
- reference/REPLANNING_SYSTEM.m
- REPLANNING_SYSTEM_R21_BALANCED.m
- REPLANNING_SYSTEM_R19_FAST.m
- controller / estimator / plant sources
- safety/certificate MEX kernels

The installer modifies run_flight_headless.m only after all preflight gates pass.
A timestamped backup directory is created beside the runner.

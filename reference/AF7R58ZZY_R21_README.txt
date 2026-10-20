AF7R58ZZY-R21 | THEORY-PRESERVING STAGE2/STAGE3 LOAD BALANCE

Purpose
-------
Discard the failed R20 fast path and return to R19 numerical authority.
Move ONLY the first post-commit repaired-reference cache construction from
Stage2 to Stage3 immediately before the existing atomic activation.

Safety/theory contract
----------------------
- Continuous geometry certificate unchanged.
- C7 position/reference certificate unchanged.
- Speed/specific-force certificate unchanged.
- Resource/batch model unchanged (R19/production path).
- Runtime sweep/obstacle/sensor packet is still prepared in Stage2.
- Exact Stage3 source/dependency/static-map freshness gates unchanged.
- Atomic activation remains at trigger + 2*dt.
- No calculation is moved to Arming/Takeoff/Landing.
- The deferred cache is not an acceptance certificate. Production source
  consumes it only after assert_runtime_core_certificates_current_hot().
- If deferred cache construction or the 25-ms control-period check fails,
  no new reference is committed.

Install
-------
AF7R58ZZY_R21_INSTALL
run_flight_headless

After run
---------
S = replanner_instance.get_staged_pipeline_status();
R = replanner_instance.get_r21_performance_status();

Rollback target
---------------
AF7R58ZZY_R21_ROLLBACK
This returns the runner to REPLANNING_SYSTEM_R19_FAST, not R20.

AF7R58ZZY-R26-R3
=================

R26-R2 failure cause
--------------------
The structural selftest passed, but MATLAB class parsing rejected a direct
superclass call made from a DIFFERENT method:

  advance_staged_replan_pipeline()
      -> prepare_staged_reference_only_bundle@REPLANNING_SYSTEM_R21_BALANCED(...)

R3 fixes only that dispatch mechanism. The exact R21 core-bundle calculation
is now reached through a legal same-name override bridge:

  advance_staged_replan_pipeline()
      -> obj.prepare_staged_reference_only_bundle(...)
      -> prepare_staged_reference_only_bundle@REPLANNING_SYSTEM_R21_BALANCED(...)

A private core-only flag returns immediately after the exact R21 bundle, before
the inherited R25 preflight compile. Final Stage4 still performs the exact R23
compile and the existing transactional activation.

Safety / theory
---------------
Unchanged:
- fixed H = 4 s collision horizon
- continuous-time certificate authority
- C7 trajectory representation
- arbitrary oriented ellipsoid geometry authority
- freshness/stale rejection
- exact target-version checks
- atomic rollback/activation
- controller code/law

Performance target
------------------
The current research target is reference/replanner computation:
  desired ~15 ms
  immediate acceptable target <=17 ms

Controller timing remains reported separately but is not modified.

Run
---
AF7R58ZZY_R26_R3_INSTALL
run_flight_headless
AF7R58ZZY_R26_R3_REPORT

Return
------
AF7R58ZZY_R26_R3_REPORT.txt
AF7R58ZZY_R26_R3_OUT.mat

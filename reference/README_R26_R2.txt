AF7R58ZZY-R26-R2
=================

This package corrects the R26 V1 installer selftest only and keeps the same
theory-preserving four-stage load-balanced replanning architecture.

Why R26 V1 failed
-----------------
Every printed structural condition was 1. The V1 selftest initialized
OUT.closed=false and then ANDed every logical field in OUT, including
OUT.closed itself. Therefore the aggregate could never become true.
The runner rollback passed, so the expected starting runner remains exact R25.

Performance intent
------------------
The current research target is the replanner/reference path, not the controller.
R26-R2 preserves total end-to-end online timing, and additionally reports a
controller-excluded metric for diagnosis. The desired replan spike is about
15 ms, with <=17 ms as the immediate target.

Scheduling
----------
trigger          : candidate generation
trigger + 1*dt   : reference certificate
trigger + 2*dt   : exact R21 core bundle preparation
trigger + 3*dt   : exact R23 compile + transactional atomic activation

H = 4 s unchanged. No continuous-time certificate, obstacle geometry,
freshness/stale gate, C7 trajectory representation, or controller law is changed.

Run
---
AF7R58ZZY_R26_R2_INSTALL
run_flight_headless
AF7R58ZZY_R26_R2_REPORT

Return
------
AF7R58ZZY_R26_R2_REPORT.txt
AF7R58ZZY_R26_R2_OUT.mat

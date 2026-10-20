# AF7R58ZZY-R30 — Single-Build Exact Stage3

R30 starts from the actually flown R29 checkpoint.

R29 established that the fast binding works (~0.14 ms) but its replacement
batch path regressed to ~3.4 ms and total Stage3 stayed ~12.77 ms.

R30 therefore does only three things:

1. Restores the exact reviewed `af7r58i_prepare_batch_model` resource theorem.
2. Keeps the bitwise-closed packet-retarget MEX and the cheap native/canonical
   support binding.
3. Builds the final reference `R` and final bundle `B` once. The R28 compiled
   packet is attached to `R` before `B` is constructed; there is no Flight-path
   `B.reference.*` mutation after the large nested bundle has been shared.

Unchanged:
- trajectory-generation mathematics
- C7 continuity
- continuous speed / acceleration / jerk / specific-force / qdot certificate
- continuous payload/UAV/cable geometry certificate
- sensor horizon theorem
- freshness / rollback / fail-closed behavior
- R28 native D0..D6 representation
- four-cycle schedule
- controller, estimator, plant
- production `reference/REPLANNING_SYSTEM.m`

Run:

1. `AF7R58ZZY_R30_INSTALL`
2. only if `SELFTEST CLOSED = 1` and `R30 INSTALL COMPLETE`
3. `AF7R58ZZY_R30_RUN_CANONICAL`
4. `AF7R58ZZY_R30_REPORT`

Primary success signal:
- Stage3 core and the Stage3 controller-excluded cycle should fall materially
  below R29 (`12.7695 ms` core, `18.8097 ms` controller-excluded).
